//! Bounded ROS 2 wire types and robot-local state. No network or actuator access.
use crate::{
    Result,
    controller::Safety,
    remote::pack_grid,
    remote_protocol::{Grid, Pose, free_goal},
    require,
};
use base64::{Engine, engine::general_purpose::STANDARD};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    collections::{HashMap, HashSet, VecDeque},
    time::Instant,
};
#[derive(Clone, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct Stamp {
    pub sec: i32,
    pub nanosec: u32,
}
impl Stamp {
    pub fn seconds(&self) -> Result<f64> {
        require(self.nanosec < 1_000_000_000, "Invalid ROS timestamp")?;
        Ok(f64::from(self.sec) + f64::from(self.nanosec) * 1e-9)
    }
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Stamp,
    pub frame_id: String,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
impl Default for Quaternion {
    fn default() -> Self {
        Self {
            x: 0.,
            y: 0.,
            z: 0.,
            w: 1.,
        }
    }
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RosPose {
    pub position: Vector3,
    pub orientation: Quaternion,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: RosPose,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct MapMetadata {
    pub map_load_time: Stamp,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: RosPose,
}
fn bounded<'de, D, T, const N: usize>(d: D) -> std::result::Result<Vec<T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: Deserialize<'de>,
{
    struct Bound<T, const N: usize>(std::marker::PhantomData<T>);
    impl<'de, T: Deserialize<'de>, const N: usize> serde::de::Visitor<'de> for Bound<T, N> {
        type Value = Vec<T>;
        fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
            write!(f, "at most {N} ROS entries")
        }
        fn visit_seq<A: serde::de::SeqAccess<'de>>(
            self,
            mut a: A,
        ) -> std::result::Result<Self::Value, A::Error> {
            if a.size_hint().is_some_and(|n| n > N) {
                return Err(serde::de::Error::custom("ROS sequence exceeds local bound"));
            }
            let mut out = Vec::new();
            while let Some(item) = a.next_element()? {
                if out.len() >= N {
                    return Err(serde::de::Error::custom("ROS sequence exceeds local bound"));
                }
                out.push(item)
            }
            Ok(out)
        }
    }
    d.deserialize_seq(Bound::<T, N>(std::marker::PhantomData))
}
fn cells<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Vec<i8>, D::Error> {
    bounded::<D, i8, 16_000_000>(d)
}
fn scan_values<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Vec<f32>, D::Error> {
    bounded::<D, f32, 65_536>(d)
}
fn image_bytes<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<Vec<u8>, D::Error> {
    bounded::<D, u8, 256_000>(d)
}
fn transforms<'de, D: serde::Deserializer<'de>>(
    d: D,
) -> std::result::Result<Vec<TransformStamped>, D::Error> {
    bounded::<D, TransformStamped, 256>(d)
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct OccupancyGrid {
    pub header: Header,
    pub info: MapMetadata,
    #[serde(deserialize_with = "cells")]
    pub data: Vec<i8>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    #[serde(deserialize_with = "scan_values")]
    pub ranges: Vec<f32>,
    #[serde(deserialize_with = "scan_values")]
    pub intensities: Vec<f32>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RosBool {
    pub data: bool,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CompressedImage {
    pub header: Header,
    pub format: String,
    #[serde(deserialize_with = "image_bytes")]
    pub data: Vec<u8>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TransformStamped {
    pub header: Header,
    pub child_frame_id: String,
    pub transform: Transform,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TFMessage {
    #[serde(deserialize_with = "transforms")]
    pub transforms: Vec<TransformStamped>,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NavGoal {
    pub pose: PoseStamped,
    pub behavior_tree: String,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NavResult {
    pub error_code: u16,
    pub error_msg: String,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Empty {}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct HumbleNavResult {
    pub result: Empty,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RosDuration {
    pub sec: i32,
    pub nanosec: u32,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct NavFeedback {
    pub current_pose: PoseStamped,
    pub navigation_time: RosDuration,
    pub estimated_time_remaining: RosDuration,
    pub number_of_recoveries: i16,
    pub distance_remaining: f32,
}
#[cfg(any(feature = "ros2-jazzy", feature = "ros2-humble"))]
macro_rules! messages{($($t:ty),*)=>{$(impl ros2_client::Message for $t{})*};}
#[cfg(any(feature = "ros2-jazzy", feature = "ros2-humble"))]
messages!(
    OccupancyGrid,
    LaserScan,
    RosBool,
    CompressedImage,
    TFMessage,
    NavGoal,
    NavResult,
    HumbleNavResult,
    NavFeedback
);
fn frame(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 100
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b"_/-".contains(&b))
}
fn yaw(q: &Quaternion) -> Result<f64> {
    let values = [q.x, q.y, q.z, q.w];
    require(
        values.iter().all(|v| v.is_finite())
            && (values.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 0.001,
        "Invalid ROS quaternion",
    )?;
    Ok((2. * (q.w * q.z + q.x * q.y)).atan2(1. - 2. * (q.y * q.y + q.z * q.z)))
}
fn pose2(p: &Vector3, q: &Quaternion) -> Result<Pose> {
    require(
        [p.x, p.y, p.z]
            .iter()
            .all(|v| v.is_finite() && v.abs() <= 1_000_000.),
        "Invalid ROS translation",
    )?;
    Ok(Pose {
        x: p.x,
        y: p.y,
        yaw: yaw(q)?,
    })
}
fn compose(a: &Pose, b: &Pose) -> Pose {
    let (c, s) = (a.yaw.cos(), a.yaw.sin());
    Pose {
        x: a.x + c * b.x - s * b.y,
        y: a.y + s * b.x + c * b.y,
        yaw: (a.yaw + b.yaw).sin().atan2((a.yaw + b.yaw).cos()),
    }
}
fn inverse(p: &Pose) -> Pose {
    let (c, s) = (p.yaw.cos(), p.yaw.sin());
    Pose {
        x: -c * p.x - s * p.y,
        y: -p.y * p.yaw.cos() + p.x * p.yaw.sin(),
        yaw: -p.yaw,
    }
}
#[derive(Clone)]
struct Edge {
    pose: Pose,
    stamp: f64,
    received: Instant,
    is_static: bool,
}
impl Edge {
    fn age(&self, time: f64) -> f64 {
        if self.is_static {
            return 0.;
        }
        let age = time - self.stamp;
        if age < -0.2 {
            return f64::INFINITY;
        }
        age.max(self.received.elapsed().as_secs_f64())
    }
}
pub struct Ros2State {
    pub map: Option<Grid>,
    pub navigation_ready: bool,
    pub error: String,
    base: String,
    session: String,
    version: u64,
    epoch: Option<(Stamp, String)>,
    edges: HashMap<(String, String), Edge>,
    scan: Option<(Instant, f64)>,
    arm: Option<Instant>,
    estop: Option<Instant>,
    armed: bool,
    emergency_stop: bool,
    image: Option<(Vec<u8>, f64, Instant)>,
}
impl Ros2State {
    pub fn new(base: String) -> Result<Self> {
        require(frame(&base), "Invalid ROS base frame")?;
        Ok(Self {
            map: None,
            navigation_ready: false,
            error: String::new(),
            base,
            session: uuid::Uuid::new_v4().to_string(),
            version: 0,
            epoch: None,
            edges: HashMap::new(),
            scan: None,
            arm: None,
            estop: None,
            armed: false,
            emergency_stop: true,
            image: None,
        })
    }
    pub fn on_map(&mut self, msg: OccupancyGrid) -> Result<()> {
        self.map = None;
        require(frame(&msg.header.frame_id), "Invalid map frame")?;
        msg.info.map_load_time.seconds()?;
        let epoch = (msg.info.map_load_time.clone(), msg.header.frame_id.clone());
        let changed = self.epoch.as_ref().is_some_and(|old| old != &epoch);
        let session = if changed {
            uuid::Uuid::new_v4().to_string()
        } else {
            self.session.clone()
        };
        let version = if changed {
            1
        } else {
            self.version
                .checked_add(1)
                .ok_or_else(|| anyhow::anyhow!("Map version exhausted"))?
        };
        let data: Vec<i64> = msg.data.iter().map(|v| i64::from(*v)).collect();
        let map = pack_grid(
            msg.info.width,
            msg.info.height,
            f64::from(msg.info.resolution),
            pose2(&msg.info.origin.position, &msg.info.origin.orientation)?,
            &data,
            &msg.header.frame_id,
            &session,
            version,
        )?;
        self.map = Some(map);
        self.session = session;
        self.version = version;
        self.epoch = Some(epoch);
        self.error.clear();
        Ok(())
    }
    pub fn on_scan(&mut self, msg: LaserScan) -> Result<()> {
        self.scan = None;
        let valid = msg
            .ranges
            .iter()
            .filter(|v| v.is_finite() && **v >= msg.range_min && **v <= msg.range_max)
            .count();
        if !(valid >= 5
            && msg.range_min.is_finite()
            && msg.range_max.is_finite()
            && msg.range_min >= 0.
            && msg.range_max > msg.range_min
            && msg.angle_increment.is_finite()
            && msg.angle_increment != 0.)
        {
            self.scan = None;
            anyhow::bail!("Insufficient valid LiDAR ranges")
        };
        self.scan = Some((Instant::now(), msg.header.stamp.seconds()?));
        Ok(())
    }
    pub fn on_arm(&mut self, value: bool) {
        self.armed = value;
        self.arm = Some(Instant::now())
    }
    pub fn on_estop(&mut self, value: bool) {
        self.emergency_stop = value;
        self.estop = Some(Instant::now())
    }
    pub fn on_tf(&mut self, msg: TFMessage, is_static: bool) -> Result<()> {
        require(msg.transforms.len() <= 256, "Too many TF transforms")?;
        for t in msg.transforms {
            require(
                frame(&t.header.frame_id)
                    && frame(&t.child_frame_id)
                    && t.header.frame_id != t.child_frame_id,
                "Invalid TF frame",
            )?;
            let edge = Edge {
                pose: pose2(&t.transform.translation, &t.transform.rotation)?,
                stamp: t.header.stamp.seconds()?,
                received: Instant::now(),
                is_static,
            };
            let key = (t.header.frame_id, t.child_frame_id);
            if !self.edges.contains_key(&key)
                && self.edges.len() >= 256
                && let Some(oldest) = self
                    .edges
                    .iter()
                    .min_by_key(|(_, v)| v.received)
                    .map(|(k, _)| k.clone())
            {
                self.edges.remove(&oldest);
            }
            self.edges.insert(key, edge);
        }
        Ok(())
    }
    pub fn on_image(&mut self, msg: CompressedImage) -> Result<()> {
        self.image = None;
        require(
            msg.data.len() <= 256_000
                && msg.data.starts_with(&[255, 216])
                && msg.data.ends_with(&[255, 217]),
            "Invalid bounded JPEG",
        )?;
        self.image = Some((msg.data, msg.header.stamp.seconds()?, Instant::now()));
        Ok(())
    }
    pub fn pose(&self, time: f64) -> Option<(Pose, f64)> {
        let map = self.map.as_ref()?;
        let mut queue = VecDeque::from([(
            map.frame.clone(),
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.,
            },
            0_f64,
        )]);
        let mut visited = HashSet::new();
        while let Some((at, p, age)) = queue.pop_front() {
            if !visited.insert(at.clone()) {
                continue;
            }
            if at == self.base {
                return Some((p, age));
            }
            for ((parent, child), edge) in &self.edges {
                if parent == &at {
                    queue.push_back((
                        child.clone(),
                        compose(&p, &edge.pose),
                        age.max(edge.age(time)),
                    ))
                } else if child == &at {
                    queue.push_back((
                        parent.clone(),
                        compose(&p, &inverse(&edge.pose)),
                        age.max(edge.age(time)),
                    ))
                }
            }
        }
        None
    }
    pub fn safety(&self, time: f64) -> Safety {
        let tf = self.pose(time).map(|(_, age)| age).unwrap_or(f64::INFINITY);
        let scan = self
            .scan
            .as_ref()
            .map(|(received, stamp)| {
                let age = time - stamp;
                if age < -0.2 {
                    f64::INFINITY
                } else {
                    age.max(received.elapsed().as_secs_f64())
                }
            })
            .unwrap_or(f64::INFINITY);
        let age = [
            tf,
            scan,
            self.arm
                .map(|t| t.elapsed().as_secs_f64())
                .unwrap_or(f64::INFINITY),
            self.estop
                .map(|t| t.elapsed().as_secs_f64())
                .unwrap_or(f64::INFINITY),
        ]
        .into_iter()
        .fold(0., f64::max);
        Safety {
            armed: self.armed,
            emergency_stop: self.emergency_stop,
            sensors_at: if age.is_finite() { time - age } else { 0. },
            healthy: self.map.is_some() && self.navigation_ready && age <= 1.,
        }
    }
    pub fn validate_goal(&self, g: &Value, time: f64) -> bool {
        let Some(map) = &self.map else { return false };
        g["frame"] == map.frame
            && g["map_session"] == map.session
            && g["x"]
                .as_f64()
                .zip(g["y"].as_f64())
                .is_some_and(|(x, y)| free_goal(map, x, y).is_ok())
            && self.safety(time).healthy
    }
    pub fn snapshot(&self, time: f64) -> Value {
        let pose = self.pose(time);
        let image = self.image.as_ref().filter(|(_, at, received)| {
            (-0.2..=1.).contains(&(time - at)) && received.elapsed().as_secs_f64() <= 1.
        });
        json!({"map":self.map,"pose":pose.as_ref().map(|(p,_)|p),"ready":self.map.is_some()&&pose.is_some_and(|(_,age)|age<=1.)&&self.navigation_ready,"mode":"hardware","message":if self.error.is_empty(){"ROS2 map / TF / LiDAR / Nav2; local arming required"}else{self.error.as_str()},"image_jpeg":image.map(|(data,_,_)|STANDARD.encode(data)),"image_at":image.map(|(_,at,_)|at)})
    }
}
