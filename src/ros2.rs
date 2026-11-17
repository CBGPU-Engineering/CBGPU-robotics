//! Native DDS/Nav2 remote adapter. No Python process, motor pin guesses or cancel-all.
use crate::{
    Result,
    controller::{Adapter, Safety},
    now, protocol, require,
    ros2_state::*,
};
use futures::FutureExt;
use ros2_client::{
    ActionTypeName, Context, ContextOptions, Message, MessageTypeName, Name, Node, NodeName,
    NodeOptions, QosProfile, RmwRequestId, ServiceMapping, Subscription,
    action::{ActionClientQosPolicies, GoalId, GoalStatusEnum},
    qos::{Durability, History, WhenFull},
};
use serde_json::{Value, json};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::{self, Receiver, Sender},
    },
    thread,
    time::{Duration, Instant},
};
#[cfg(feature = "ros2-jazzy")]
type ResultMessage = NavResult;
#[cfg(not(feature = "ros2-jazzy"))]
type ResultMessage = HumbleNavResult;
const TICK: Duration = Duration::from_millis(20);
#[derive(Clone)]
pub struct Config {
    pub robot_id: String,
    pub base_frame: String,
    pub map_topic: String,
    pub scan_topic: String,
    pub arm_topic: String,
    pub estop_topic: String,
    pub image_topic: Option<String>,
    pub nav_action: String,
    pub domain: u16,
    pub commissioned: bool,
}
impl Config {
    pub fn from_env() -> Result<Self> {
        let get =
            |name: &str, default: &str| std::env::var(name).unwrap_or_else(|_| default.into());
        let distro = if cfg!(feature = "ros2-jazzy") {
            "jazzy"
        } else {
            "humble"
        };
        if let Ok(actual) = std::env::var("ROS_DISTRO") {
            require(
                actual == distro,
                &format!("This binary targets ROS2 {distro}; use a matching build"),
            )?;
        }
        let c = Self {
            robot_id: get("CBGPU_ROBOT_ID", "ros2-lidar-robot"),
            base_frame: get("CBGPU_BASE_FRAME", "base_link"),
            map_topic: get("CBGPU_MAP_TOPIC", "/map"),
            scan_topic: get("CBGPU_SCAN_TOPIC", "/scan"),
            arm_topic: get("CBGPU_ARM_TOPIC", "/cbgpu/armed"),
            estop_topic: get("CBGPU_ESTOP_TOPIC", "/cbgpu/emergency_stop"),
            image_topic: std::env::var("CBGPU_IMAGE_TOPIC")
                .ok()
                .filter(|v| !v.is_empty()),
            nav_action: get("CBGPU_NAV_ACTION", "/navigate_to_pose"),
            domain: get("ROS_DOMAIN_ID", "0")
                .parse()
                .map_err(|_| anyhow::anyhow!("ROS_DOMAIN_ID must be 0..232"))?,
            commissioned: get("CBGPU_ROBOT_COMMISSIONED", "0") == "1",
        };
        protocol::robot_id(&c.robot_id)?;
        require(c.domain <= 232, "ROS_DOMAIN_ID must be 0..232")?;
        for n in [
            &c.map_topic,
            &c.scan_topic,
            &c.arm_topic,
            &c.estop_topic,
            &c.nav_action,
        ] {
            name(n)?;
        }
        if let Some(n) = &c.image_topic {
            name(n)?;
        }
        Ok(c)
    }
}
fn name(value: &str) -> Result<Name> {
    require(
        value.starts_with('/') && value.len() <= 200 && !value.contains("//"),
        "ROS topic/action must be an absolute bounded name",
    )?;
    let (ns, base) = value.rsplit_once('/').unwrap();
    Ok(Name::new(if ns.is_empty() { "/" } else { ns }, base)?)
}
fn reliable(depth: usize) -> QosProfile {
    QosProfile::publisher_default()
        .reliability_reliable(WhenFull::Fail)
        .durability(Durability::Volatile)
        .history(History::KeepLast { depth })
}
fn subscribe<M: Message + 'static>(
    node: &mut Node,
    path: &str,
    pkg: &str,
    ty: &str,
    qos: QosProfile,
) -> Result<Subscription<M>> {
    let t = node.create_topic(&name(path)?, MessageTypeName::new(pkg, ty), &qos)?;
    Ok(node.create_subscription(&t, None)?)
}
type Reply = Sender<std::result::Result<bool, String>>;
enum Command {
    Navigate(Value, Reply),
    Stop(Reply),
}
struct Active {
    goal: Value,
    id: GoalId,
    accept: RmwRequestId,
    result: Option<RmwRequestId>,
    accepted: bool,
    stopping: bool,
    cancel: Option<RmwRequestId>,
    reply: Reply,
    stoppers: Vec<Reply>,
    started: Instant,
    last_cancel: Instant,
}
pub struct Ros2Adapter {
    config: Config,
    state: Arc<Mutex<Ros2State>>,
    tx: Sender<Command>,
    priority: Sender<Command>,
    quit: Arc<AtomicBool>,
}
impl Ros2Adapter {
    pub fn open(config: Config) -> Result<Self> {
        let state = Arc::new(Mutex::new(Ros2State::new(config.base_frame.clone())?));
        let (tx, rx) = mpsc::channel();
        let (priority, stops) = mpsc::channel();
        let quit = Arc::new(AtomicBool::new(false));
        let (ready, wait) = mpsc::sync_channel(1);
        let c = config.clone();
        let s = state.clone();
        let q = quit.clone();
        thread::Builder::new()
            .name("cbgpu-ros2".into())
            .spawn(move || {
                let r = worker(c, s.clone(), rx, stops, q, ready);
                if let Err(e) = r
                    && let Ok(mut s) = s.lock()
                {
                    s.navigation_ready = false;
                    s.error = format!("ROS2 transport stopped: {e}");
                }
            })?;
        wait.recv_timeout(Duration::from_secs(10))
            .map_err(|_| anyhow::anyhow!("ROS2 initialization timed out"))?
            .map_err(anyhow::Error::msg)?;
        Ok(Self {
            config,
            state,
            tx,
            priority,
            quit,
        })
    }
    fn request_stop(&self) -> Result<()> {
        let (tx, rx) = mpsc::channel();
        self.priority.send(Command::Stop(tx))?;
        rx.recv_timeout(Duration::from_secs(3))
            .map_err(|_| anyhow::anyhow!("Nav2 stop not confirmed; local robot lock retained"))?
            .map_err(anyhow::Error::msg)?;
        Ok(())
    }
}
impl Drop for Ros2Adapter {
    fn drop(&mut self) {
        let _ = self.request_stop();
        self.quit.store(true, Ordering::Release);
    }
}
impl Adapter for Ros2Adapter {
    fn robot_id(&self) -> String {
        self.config.robot_id.clone()
    }
    fn capabilities(&self) -> Value {
        json!({})
    }
    fn commissioned(&self) -> bool {
        self.config.commissioned
    }
    fn observe(&self) -> Result<Value> {
        self.remote_snapshot()
    }
    fn safety(&self) -> Result<Safety> {
        Ok(self
            .state
            .lock()
            .map_err(|_| anyhow::anyhow!("ROS2 state unavailable"))?
            .safety(now()))
    }
    fn validate_step(&self, _: &protocol::Step) -> Result<bool> {
        Ok(false)
    }
    fn execute(&self, _: &str, _: &Value, _: Arc<AtomicBool>) -> Result<()> {
        anyhow::bail!("ROS2 adapter supports bounded remote Nav2 goals only")
    }
    fn stop(&self) -> Result<()> {
        self.request_stop()
    }
    fn remote_snapshot(&self) -> Result<Value> {
        Ok(self
            .state
            .lock()
            .map_err(|_| anyhow::anyhow!("ROS2 state unavailable"))?
            .snapshot(now()))
    }
    fn validate_remote_goal(&self, g: &Value) -> Result<bool> {
        Ok(self
            .state
            .lock()
            .map_err(|_| anyhow::anyhow!("ROS2 state unavailable"))?
            .validate_goal(g, now()))
    }
    fn navigate_to(&self, g: &Value, stop: Arc<AtomicBool>) -> Result<bool> {
        require(
            self.commissioned(),
            "ROS2 robot is not locally commissioned",
        )?;
        self.safety()?.check()?;
        require(
            self.validate_remote_goal(g)?,
            "Goal does not match the current free map",
        )?;
        let (tx, rx) = mpsc::channel();
        self.tx.send(Command::Navigate(g.clone(), tx))?;
        let start = Instant::now();
        loop {
            if stop.load(Ordering::Acquire) || start.elapsed() > Duration::from_secs(180) {
                self.request_stop()?;
                return Ok(false);
            }
            match rx.recv_timeout(TICK) {
                Ok(result) => return result.map_err(anyhow::Error::msg),
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(_) => anyhow::bail!("ROS2 worker unavailable; verify robot stopped"),
            }
        }
    }
}
fn nav_goal(v: &Value) -> Result<NavGoal> {
    let x = v["x"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Missing x"))?;
    let y = v["y"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Missing y"))?;
    let yaw = v["yaw"].as_f64().unwrap_or(0.);
    require(
        [x, y, yaw].iter().all(|v| v.is_finite()),
        "Invalid Nav2 coordinates",
    )?;
    let t = now();
    Ok(NavGoal {
        pose: PoseStamped {
            header: Header {
                stamp: Stamp {
                    sec: t.floor() as i32,
                    nanosec: ((t - t.floor()) * 1e9) as u32,
                },
                frame_id: v["frame"].as_str().unwrap_or("").into(),
            },
            pose: RosPose {
                position: Vector3 { x, y, z: 0. },
                orientation: Quaternion {
                    x: 0.,
                    y: 0.,
                    z: (yaw / 2.).sin(),
                    w: (yaw / 2.).cos(),
                },
            },
        },
        behavior_tree: String::new(),
    })
}
fn finish(active: &mut Option<Active>, success: bool) {
    if let Some(a) = active.take() {
        let _ = a.reply.send(Ok(success));
        for s in a.stoppers {
            let _ = s.send(Ok(true));
        }
    }
}
fn worker(
    config: Config,
    state: Arc<Mutex<Ros2State>>,
    rx: Receiver<Command>,
    stops: Receiver<Command>,
    quit: Arc<AtomicBool>,
    ready: mpsc::SyncSender<std::result::Result<(), String>>,
) -> Result<()> {
    let setup = (|| -> Result<_> {
        let context = Context::with_options(ContextOptions::new().domain_id(config.domain))?;
        let mut node = context.new_node(
            NodeName::new(
                "/cbgpu",
                &format!("remote_{}", uuid::Uuid::new_v4().simple()),
            )?,
            NodeOptions::default(),
        )?;
        let map = subscribe::<OccupancyGrid>(
            &mut node,
            &config.map_topic,
            "nav_msgs",
            "OccupancyGrid",
            reliable(1).durability(Durability::TransientLocal),
        )?;
        let sensor = reliable(1).reliability_best_effort();
        let scan = subscribe::<LaserScan>(
            &mut node,
            &config.scan_topic,
            "sensor_msgs",
            "LaserScan",
            sensor.clone(),
        )?;
        let arm = subscribe::<RosBool>(
            &mut node,
            &config.arm_topic,
            "std_msgs",
            "Bool",
            reliable(10),
        )?;
        let estop = subscribe::<RosBool>(
            &mut node,
            &config.estop_topic,
            "std_msgs",
            "Bool",
            reliable(10),
        )?;
        let tf = subscribe::<TFMessage>(&mut node, "/tf", "tf2_msgs", "TFMessage", sensor.clone())?;
        let static_tf = subscribe::<TFMessage>(
            &mut node,
            "/tf_static",
            "tf2_msgs",
            "TFMessage",
            reliable(10).durability(Durability::TransientLocal),
        )?;
        let image = config
            .image_topic
            .as_ref()
            .map(|path| {
                subscribe::<CompressedImage>(
                    &mut node,
                    path,
                    "sensor_msgs",
                    "CompressedImage",
                    sensor,
                )
            })
            .transpose()?;
        let q = reliable(10);
        let nav = node.create_action_client::<NavGoal, ResultMessage, NavFeedback>(
            ServiceMapping::Enhanced,
            &name(&config.nav_action)?,
            &ActionTypeName::new("nav2_msgs", "NavigateToPose"),
            ActionClientQosPolicies {
                goal_service: q.clone(),
                result_service: q.clone(),
                cancel_service: q.clone(),
                feedback_subscription: q.clone(),
                status_subscription: q,
            },
        )?;
        Ok((
            context, node, map, scan, arm, estop, tf, static_tf, image, nav,
        ))
    })();
    let (_context, mut node, map, scan, arm, estop, tf, static_tf, image, nav) = match setup {
        Ok(v) => v,
        Err(e) => {
            let _ = ready.send(Err(e.to_string()));
            return Err(e);
        }
    };
    // Packing a 16M-cell map must never hold the safety/control mutex or delay stop.
    let pending_map = Arc::new(Mutex::new(None));
    let maps = pending_map.clone();
    let map_quit = quit.clone();
    let base = config.base_frame.clone();
    let map_thread = thread::Builder::new()
        .name("cbgpu-ros2-map".into())
        .spawn(move || {
            let mut packing = Ros2State::new(base).expect("validated base frame");
            while !map_quit.load(Ordering::Acquire) {
                match map.take() {
                    Ok(Some((message, _))) => {
                        let error = packing
                            .on_map(message)
                            .err()
                            .map(|e| e.to_string())
                            .unwrap_or_default();
                        *maps.lock().unwrap() = Some((packing.map.take(), error));
                    }
                    Ok(None) => thread::sleep(TICK),
                    Err(_) => {
                        *maps.lock().unwrap() = Some((None, "Invalid ROS map message".into()));
                        thread::sleep(TICK);
                    }
                }
            }
        })?;
    let spinner = node.spinner()?;
    let spin = thread::Builder::new()
        .name("cbgpu-ros2-discovery".into())
        .spawn(move || futures::executor::block_on(spinner.spin()))?;
    let _ = ready.send(Ok(()));
    let mut active: Option<Active> = None;
    let mut discovery = Instant::now() - Duration::from_secs(1);
    let result = (|| -> Result<()> {
        while !quit.load(Ordering::Acquire) {
            // Stop requests have priority over map deserialization / navigation requests.
            for cmd in stops.try_iter() {
                if let Command::Stop(reply) = cmd {
                    if let Some(a) = active.as_mut() {
                        a.stopping = true;
                        a.stoppers.push(reply)
                    } else {
                        let _ = reply.send(Ok(true));
                    }
                }
            }
            if discovery.elapsed() >= Duration::from_millis(500) {
                let available = nav
                    .goal_client()
                    .wait_for_service(&node)
                    .now_or_never()
                    .is_some()
                    && nav
                        .cancel_client()
                        .wait_for_service(&node)
                        .now_or_never()
                        .is_some()
                    && nav
                        .result_client()
                        .wait_for_service(&node)
                        .now_or_never()
                        .is_some();
                state.lock().unwrap().navigation_ready = available;
                discovery = Instant::now();
            }
            macro_rules! drain{($sub:expr,$method:ident$(,$arg:expr)?)=>{for _ in 0..16{match $sub.take(){Ok(Some((m,_)))=>{let mut s=state.lock().unwrap();if let Err(e)=s.$method(m$(,$arg)?){s.error=e.to_string();}},Ok(None)=>break,Err(_)=>{state.lock().unwrap().error="Invalid ROS message".into();break}}}};}
            if let Some((map, error)) = pending_map.lock().unwrap().take() {
                let mut local = state.lock().unwrap();
                local.map = map;
                local.error = error;
            }
            drain!(scan, on_scan);
            drain!(tf, on_tf, false);
            drain!(static_tf, on_tf, true);
            for (sub, is_arm) in [(&arm, true), (&estop, false)] {
                for _ in 0..16 {
                    match sub.take() {
                        Ok(Some((m, _))) => {
                            let mut s = state.lock().unwrap();
                            if is_arm {
                                s.on_arm(m.data)
                            } else {
                                s.on_estop(m.data)
                            }
                        }
                        _ => break,
                    }
                }
            }
            if let Some(image) = &image {
                drain!(image, on_image)
            }
            for command in rx.try_iter() {
                if let Command::Navigate(goal, reply) = command {
                    if active.is_some() || !state.lock().unwrap().validate_goal(&goal, now()) {
                        let _ = reply.send(Err("ROS2 busy or goal/sensors no longer valid".into()));
                        continue;
                    }
                    match nav.send_goal(nav_goal(&goal)?) {
                        Ok((request, id)) => {
                            active = Some(Active {
                                goal: goal.clone(),
                                id,
                                accept: request,
                                result: None,
                                accepted: false,
                                stopping: false,
                                cancel: None,
                                reply,
                                stoppers: Vec::new(),
                                started: Instant::now(),
                                last_cancel: Instant::now() - Duration::from_secs(1),
                            })
                        }
                        Err(_) => {
                            let _ = reply.send(Err("Nav2 goal could not be sent".into()));
                        }
                    }
                }
            }
            if let Some(a) = active.as_mut() {
                if !state.lock().unwrap().validate_goal(&a.goal, now())
                    || state.lock().unwrap().safety(now()).check().is_err()
                    || a.started.elapsed() > Duration::from_secs(180)
                {
                    a.stopping = true;
                }
                if !a.accepted
                    && let Some(response) = nav.receive_goal_response(a.accept)?
                {
                    if !response.accepted {
                        finish(&mut active, false);
                        continue;
                    }
                    a.accepted = true;
                    a.result = Some(nav.request_result(a.id)?);
                }
                if let Some(id) = a.result
                    && let Some((status, result)) = nav.receive_result(id)?
                {
                    #[cfg(feature = "ros2-jazzy")]
                    let ok = result.error_code == 0;
                    #[cfg(not(feature = "ros2-jazzy"))]
                    let ok = {
                        let _ = result;
                        true
                    };
                    let success = status == GoalStatusEnum::Succeeded && ok && !a.stopping;
                    finish(&mut active, success);
                    continue;
                }
                if a.stopping && a.last_cancel.elapsed() > Duration::from_millis(250) {
                    if let Some(id) = a.cancel {
                        let _ = nav.receive_cancel_response(id)?;
                    }
                    a.cancel = Some(nav.cancel_goal(a.id)?);
                    a.last_cancel = Instant::now();
                }
                // A cancel acknowledgment means CANCELING, not stopped. Only a terminal result releases the lock.
                let _ = nav.receive_feedback(a.id)?;
                let _ = nav.receive_status()?;
            }
            thread::sleep(TICK);
        }
        Ok(())
    })();
    if let Some(a) = active.as_ref() {
        let _ = nav.cancel_goal(a.id);
    }
    quit.store(true, Ordering::Release);
    node.stop_spinner();
    let _ = spin.join();
    let _ = map_thread.join();
    result
}
