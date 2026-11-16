use crate::{
    Result,
    client::{Client, DEFAULT},
    controller::{Adapter, Ledger},
    now,
    remote_protocol::{Grid, Pose, free_goal, grid, pose},
    require, valid_id,
};
use base64::{Engine, engine::general_purpose::STANDARD};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
// Mirrors the public map packing contract across adapters.
#[allow(clippy::too_many_arguments)]
pub fn pack_grid(
    width: u32,
    height: u32,
    resolution: f64,
    origin: Pose,
    data: &[i64],
    frame: &str,
    session: &str,
    version: u64,
) -> Result<Grid> {
    require(
        width > 0
            && height > 0
            && u64::from(width) * u64::from(height) <= 16_000_000
            && data.len() as u64 == u64::from(width) * u64::from(height),
        "Invalid source occupancy map",
    )?;
    require(
        data.iter().all(|v| (-1..=100).contains(v)),
        "Invalid occupancy value",
    )?;
    let scale = width.max(height).div_ceil(512).max(1);
    let w = width.div_ceil(scale);
    let h = height.div_ceil(scale);
    let mut rle: Vec<i64> = vec![];
    for row in 0..h {
        for col in 0..w {
            let mut occupied = 0;
            let mut unknown = false;
            for y in row * scale..((row + 1) * scale).min(height) {
                for x in col * scale..((col + 1) * scale).min(width) {
                    let v = data[(y * width + x) as usize];
                    occupied = occupied.max(v);
                    unknown |= v == -1;
                }
            }
            let v = if occupied > 0 {
                occupied
            } else if unknown {
                -1
            } else {
                0
            };
            if rle.len() >= 2 && rle[rle.len() - 2] == v {
                let last = rle.len() - 1;
                rle[last] += 1;
            } else {
                rle.extend([v, 1]);
            }
        }
    }
    grid(
        &json!({"width":w,"height":h,"resolution":resolution*f64::from(scale),"origin":origin,"frame":frame,"session":session,"version":version,"rle":rle}),
    )
}
#[derive(Serialize, Deserialize)]
pub struct Credentials {
    pub device_id: String,
    pub device_token: String,
    #[serde(default = "default_url")]
    pub base_url: String,
}
fn default_url() -> String {
    DEFAULT.into()
}
pub fn credentials_dir() -> Result<PathBuf> {
    let home = std::env::var_os(if cfg!(windows) { "USERPROFILE" } else { "HOME" })
        .ok_or_else(|| anyhow::anyhow!("Home directory unavailable"))?;
    Ok(PathBuf::from(home).join(".cbgpu/robotics/devices"))
}
pub fn load_credentials(root: &Path, id: &str) -> Result<Credentials> {
    require(valid_id(id), "Invalid device ID")?;
    let p = root.join(format!("{id}.json"));
    crate::private_file(&p)?;
    let bytes = std::fs::read(p)?;
    require(bytes.len() < 8192, "Invalid device credential")?;
    let c: Credentials = serde_json::from_slice(&bytes)?;
    require(c.device_id == id, "Device credential mismatch")?;
    Ok(c)
}
pub fn enroll(code: &str, base: &str, root: &Path) -> Result<Credentials> {
    let response = Client::new("pairing".into(), base, Duration::from_secs(20))?.request(
        "/robotics/agent/enroll",
        reqwest::Method::POST,
        Some(&json!({"pair_code":code,"agent_session":uuid::Uuid::new_v4().to_string()})),
    )?;
    let c = Credentials {
        device_id: response["device_id"].as_str().unwrap_or_default().into(),
        device_token: response["device_token"].as_str().unwrap_or_default().into(),
        base_url: base.into(),
    };
    require(
        valid_id(&c.device_id) && !c.device_token.is_empty(),
        "Invalid enrollment response",
    )?;
    std::fs::create_dir_all(root)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(root, std::fs::Permissions::from_mode(0o700))?;
    }
    let p = root.join(format!("{}.json", c.device_id));
    let mut options = std::fs::OpenOptions::new();
    options.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        options.mode(0o600);
    }
    use std::io::Write;
    let mut file = options.open(p)?;
    file.write_all(&serde_json::to_vec(&c)?)?;
    file.sync_all()?;
    Ok(c)
}
#[derive(Default)]
struct State {
    last_sync: Option<Instant>,
    operator_live: bool,
    current: Option<String>,
    ack: Option<Value>,
    failed: bool,
    epoch: Option<u64>,
}
pub struct RemoteAgent {
    adapter: Arc<dyn Adapter>,
    ledger: Arc<Ledger>,
    client: Client,
    allow_navigation: bool,
    session: String,
    state: Arc<Mutex<State>>,
    motion_stop: Arc<AtomicBool>,
    quit: Arc<AtomicBool>,
    worker: Option<std::thread::JoinHandle<()>>,
    sent_map: Option<(String, u64)>,
    last_image: Option<Instant>,
}
fn check(
    adapter: &dyn Adapter,
    state: &Mutex<State>,
    stop: &AtomicBool,
    quit: &AtomicBool,
    enabled: bool,
) -> Result<()> {
    adapter.safety()?.check()?;
    require(
        enabled && adapter.commissioned(),
        "Remote navigation not commissioned/enabled",
    )?;
    let s = state.lock().unwrap();
    require(
        s.operator_live
            && s.last_sync
                .is_some_and(|t| t.elapsed() < Duration::from_millis(1500)),
        "Remote operator/connection stale",
    )?;
    require(
        !stop.load(Ordering::Acquire) && !quit.load(Ordering::Acquire),
        "Navigation cancelled",
    )
}
impl RemoteAgent {
    pub fn new(
        adapter: Arc<dyn Adapter>,
        ledger: Arc<Ledger>,
        credentials: Credentials,
        enabled: bool,
        quit: Arc<AtomicBool>,
    ) -> Result<Self> {
        Ok(Self {
            adapter,
            ledger,
            client: Client::new(
                credentials.device_token,
                &credentials.base_url,
                Duration::from_secs(2),
            )?,
            allow_navigation: enabled,
            session: uuid::Uuid::new_v4().to_string(),
            state: Arc::new(Mutex::new(State::default())),
            motion_stop: Arc::new(AtomicBool::new(false)),
            quit,
            worker: None,
            sent_map: None,
            last_image: None,
        })
    }
    fn stop(&self, force: bool) {
        self.motion_stop.store(true, Ordering::Release);
        let s = self.state.lock().unwrap();
        if force || s.current.is_some() || s.failed {
            drop(s);
            let _ = self.adapter.stop();
        }
    }
    fn accept(&mut self, command: &Value) -> Result<()> {
        let id = command["id"].as_str().unwrap_or_default();
        require(valid_id(id), "Invalid remote command ID")?;
        if self.worker.as_ref().is_some_and(|w| !w.is_finished()) {
            return Ok(());
        }
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
        if let Some(previous) = self.ledger.previous(id)? {
            self.state.lock().unwrap().ack = Some(
                json!({"id":id,"state":if ["succeeded","failed","cancelled"].contains(&previous.as_str()){previous}else{"failed".into()},"error":"Previous mission will not be replayed"}),
            );
            return Ok(());
        }
        {
            let mut s = self.state.lock().unwrap();
            if s.failed {
                s.ack = Some(
                    json!({"id":id,"state":"failed","error":"Local controller needs recovery"}),
                );
                return Ok(());
            }
            s.current = Some(id.into());
        }
        self.motion_stop = Arc::new(AtomicBool::new(false));
        let a = self.adapter.clone();
        let ledger = self.ledger.clone();
        let state = self.state.clone();
        let stop = self.motion_stop.clone();
        let quit = self.quit.clone();
        let enabled = self.allow_navigation;
        let command = command.clone();
        let id = id.to_string();
        self.worker = Some(std::thread::spawn(move || {
            let mut reserved = false;
            let mut driver = None;
            let result = (|| -> Result<()> {
                check(a.as_ref(), &state, &stop, &quit, enabled)?;
                let snap = a.remote_snapshot()?;
                let m = grid(&snap["map"])?;
                let g = &command["payload"];
                require(
                    g["map_session"] == m.session && g["frame"] == m.frame,
                    "Mapping session changed",
                )?;
                let x = g["x"].as_f64().unwrap_or(f64::NAN);
                let y = g["y"].as_f64().unwrap_or(f64::NAN);
                free_goal(&m, x, y)?;
                require(
                    command["expires_at"]
                        .as_f64()
                        .is_some_and(|t| t.is_finite() && now() < t),
                    "Goal delivery expired",
                )?;
                require(a.validate_remote_goal(g)?, "Local navigation rejected goal")?;
                ledger.reserve(&a.robot_id(), &id)?;
                reserved = true;
                let driver_a = a.clone();
                let driver_g = g.clone();
                let driver_s = stop.clone();
                driver = Some(std::thread::spawn(move || {
                    driver_a.navigate_to(&driver_g, driver_s)
                }));
                state.lock().unwrap().ack = Some(json!({"id":id,"state":"running"}));
                let deadline = Instant::now() + Duration::from_secs(180);
                while !driver.as_ref().unwrap().is_finished() {
                    std::thread::sleep(Duration::from_millis(50));
                    check(a.as_ref(), &state, &stop, &quit, enabled)?;
                    require(Instant::now() < deadline, "Navigation exceeded 180 seconds")?;
                    require(
                        a.remote_snapshot()?["map"]["session"] == g["map_session"],
                        "Mapping session changed",
                    )?;
                }
                require(
                    driver
                        .take()
                        .unwrap()
                        .join()
                        .map_err(|_| anyhow::anyhow!("Driver panicked"))??,
                    "Navigation did not reach goal",
                )?;
                Ok(())
            })();
            stop.store(true, Ordering::Release);
            let stop_result = a.stop();
            let deadline = Instant::now() + Duration::from_millis(500);
            while driver.as_ref().is_some_and(|h| !h.is_finished()) && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(10));
            }
            let release = driver.as_ref().is_none_or(|h| h.is_finished()) && stop_result.is_ok();
            let status = if result.is_ok() && stop_result.is_ok() {
                "succeeded"
            } else if stop.load(Ordering::Acquire) && quit.load(Ordering::Acquire) {
                "cancelled"
            } else {
                "failed"
            };
            if reserved {
                let _ = ledger.finish(&a.robot_id(), &id, status, release);
            }
            let mut s = state.lock().unwrap();
            s.failed |= !release || stop_result.is_err();
            s.ack = Some(
                json!({"id":id,"state":status,"error":if result.is_ok()&&stop_result.is_ok(){""}else{"Local navigation stopped; inspect local driver and safety state"}}),
            );
            s.current = None;
        }));
        Ok(())
    }
    pub fn sync_once(&mut self) -> Result<()> {
        let snapshot = self.adapter.remote_snapshot()?;
        let m = if snapshot["map"].is_object() {
            Some(grid(&snapshot["map"])?)
        } else {
            None
        };
        let safety = self.adapter.safety()?;
        let age = (now() - safety.sensors_at).max(0.);
        require(age.is_finite(), "Invalid sensor timestamp")?;
        let p = if snapshot["pose"].is_object() {
            Some(pose(&snapshot["pose"])?)
        } else {
            None
        };
        let telemetry = json!({"armed":safety.armed,"emergency_stop":safety.emergency_stop,"ready":snapshot["ready"]==true&&m.is_some()&&safety.healthy&&age<=1.,"navigation_enabled":self.allow_navigation&&self.adapter.commissioned()&&!self.state.lock().unwrap().failed,"sensor_age_s":age,"pose":p,"map_session":m.as_ref().map(|m|m.session.as_str()).unwrap_or(""),"map_version":m.as_ref().map(|m|m.version).unwrap_or(0),"mode":snapshot["mode"].as_str().unwrap_or("hardware"),"message":snapshot["message"].as_str().unwrap_or("").chars().take(300).collect::<String>(),"image_at":snapshot["image_at"]});
        let mut body = json!({"agent_session":self.session,"telemetry":telemetry});
        let marker = m.as_ref().map(|m| (m.session.clone(), m.version));
        if marker != self.sent_map
            && let Some(m) = m
        {
            body["map"] = serde_json::to_value(m)?;
        }
        if self
            .last_image
            .is_none_or(|t| t.elapsed() >= Duration::from_millis(500))
            && let Some(image) = snapshot["image_jpeg"].as_str()
        {
            let bytes = STANDARD.decode(image)?;
            if bytes.len() <= 256000
                && bytes.starts_with(&[0xff, 0xd8])
                && bytes.ends_with(&[0xff, 0xd9])
            {
                body["image"] = Value::String(image.into());
                self.last_image = Some(Instant::now());
            }
        }
        let ack = self.state.lock().unwrap().ack.clone();
        if let Some(ack) = &ack {
            body["ack"] = ack.clone();
        }
        let r = self
            .client
            .request("/robotics/agent/sync", reqwest::Method::POST, Some(&body))?;
        let live = r["operator_live"]
            .as_bool()
            .ok_or_else(|| anyhow::anyhow!("Invalid control lease response"))?;
        let epoch = r["stop_epoch"]
            .as_u64()
            .ok_or_else(|| anyhow::anyhow!("Invalid stop epoch"))?;
        let changed = {
            let mut s = self.state.lock().unwrap();
            s.last_sync = Some(Instant::now());
            s.operator_live = live;
            let changed = s.epoch.is_some_and(|old| old != epoch);
            s.epoch = Some(epoch);
            if s.ack == ack && ack.as_ref().is_some_and(|a| a["state"] != "running") {
                s.ack = None;
            }
            changed
        };
        self.sent_map = marker;
        if changed {
            self.stop(true);
        }
        if !live {
            self.stop(false);
        }
        if r["command"].is_object() {
            self.accept(&r["command"])?;
        }
        Ok(())
    }
    pub fn run(&mut self) -> Result<()> {
        while !self.quit.load(Ordering::Acquire) {
            let began = Instant::now();
            if self.sync_once().is_err() {
                self.state.lock().unwrap().operator_live = false;
                self.stop(false);
                eprintln!("Connection/control paused; retrying without moving");
                std::thread::sleep(Duration::from_secs(1));
            }
            if let Some(wait) = Duration::from_millis(250).checked_sub(began.elapsed()) {
                std::thread::sleep(wait);
            }
        }
        self.stop(false);
        if let Some(w) = self.worker.take() {
            let deadline = Instant::now() + Duration::from_secs(2);
            while !w.is_finished() && Instant::now() < deadline {
                std::thread::sleep(Duration::from_millis(20));
            }
            if w.is_finished() {
                let _ = w.join();
            }
        }
        Ok(())
    }
}
impl Drop for RemoteAgent {
    fn drop(&mut self) {
        self.quit.store(true, Ordering::Release);
        self.stop(false);
    }
}
