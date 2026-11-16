use crate::{
    Result,
    controller::{Adapter, Safety},
    now,
    protocol::{Step, validate},
    remote::pack_grid,
    remote_protocol::{Pose, free_goal, grid},
    require,
};
use serde_json::{Value, json};
use std::sync::{
    Arc, Mutex,
    atomic::{AtomicBool, Ordering},
};
pub struct Simulator {
    pub events: Mutex<Vec<Value>>,
    pub estop: AtomicBool,
    pub armed: AtomicBool,
    pub commissioned: AtomicBool,
    pub stopped: AtomicBool,
    position: Mutex<Pose>,
    session: String,
    started: std::time::Instant,
}
impl Default for Simulator {
    fn default() -> Self {
        Self {
            events: Mutex::new(vec![]),
            estop: AtomicBool::new(false),
            armed: AtomicBool::new(true),
            commissioned: AtomicBool::new(true),
            stopped: AtomicBool::new(false),
            position: Mutex::new(Pose {
                x: 2.,
                y: 2.,
                yaw: 0.,
            }),
            session: uuid::Uuid::new_v4().to_string(),
            started: std::time::Instant::now(),
        }
    }
}
impl Adapter for Simulator {
    fn robot_id(&self) -> String {
        "virtual-robot".into()
    }
    fn capabilities(&self) -> Value {
        json!([{"name":"navigate_to","description":"Navigate to a known named waypoint using local obstacle avoidance.","parameters":{"type":"object","properties":{"waypoint":{"type":"string","enum":["desk","charging_station"],"maxLength":32}},"required":["waypoint"],"additionalProperties":false},"max_duration_s":10},{"name":"set_gripper","description":"Open or close the local gripper.","parameters":{"type":"object","properties":{"closed":{"type":"boolean"}},"required":["closed"],"additionalProperties":false},"max_duration_s":2},{"name":"say","description":"Speak a short sentence.","parameters":{"type":"object","properties":{"text":{"type":"string","maxLength":100}},"required":["text"],"additionalProperties":false},"max_duration_s":5}])
    }
    fn commissioned(&self) -> bool {
        self.commissioned.load(Ordering::Acquire)
    }
    fn observe(&self) -> Result<Value> {
        Ok(
            json!({"location":"desk","gripper_closed":false,"known_waypoints":["desk","charging_station"],"mode":"simulation"}),
        )
    }
    fn safety(&self) -> Result<Safety> {
        Ok(Safety {
            armed: self.armed.load(Ordering::Acquire),
            emergency_stop: self.estop.load(Ordering::Acquire),
            sensors_at: now(),
            healthy: true,
        })
    }
    fn validate_step(&self, step: &Step) -> Result<bool> {
        let caps = self.capabilities();
        let cap = caps
            .as_array()
            .unwrap()
            .iter()
            .find(|c| c["name"] == step.capability)
            .ok_or_else(|| anyhow::anyhow!("Unknown capability"))?;
        validate(&step.arguments, &cap["parameters"])?;
        Ok(true)
    }
    fn execute(&self, name: &str, args: &Value, stop: Arc<AtomicBool>) -> Result<()> {
        for _ in 0..20 {
            if stop.load(Ordering::Acquire) {
                return Ok(());
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        self.events
            .lock()
            .unwrap()
            .push(json!({"capability":name,"arguments":args}));
        Ok(())
    }
    fn stop(&self) -> Result<()> {
        self.stopped.store(true, Ordering::Release);
        Ok(())
    }
    fn remote_snapshot(&self) -> Result<Value> {
        let elapsed = self.started.elapsed().as_secs();
        let radius = (9 + elapsed / 2).min(29) as i32;
        let mut data = vec![];
        for y in 0i32..64 {
            for x in 0i32..64 {
                data.push(if (x - 16).abs() > radius || (y - 16).abs() > radius {
                    -1
                } else if x == 0 || x == 63 || y == 0 || y == 63 || x == 36 && (9..48).contains(&y)
                {
                    100
                } else {
                    0
                });
            }
        }
        let m = pack_grid(
            64,
            64,
            0.125,
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.,
            },
            &data,
            "map",
            &self.session,
            (elapsed / 2 + 1).min(41),
        )?;
        Ok(
            json!({"map":m,"pose":*self.position.lock().unwrap(),"ready":true,"mode":"simulation","message":"Virtual online mapping; no physical robot attached"}),
        )
    }
    fn validate_remote_goal(&self, g: &Value) -> Result<bool> {
        free_goal(
            &grid(&self.remote_snapshot()?["map"])?,
            g["x"].as_f64().unwrap_or(f64::NAN),
            g["y"].as_f64().unwrap_or(f64::NAN),
        )?;
        Ok(true)
    }
    fn navigate_to(&self, g: &Value, stop: Arc<AtomicBool>) -> Result<bool> {
        let start = self.position.lock().unwrap().clone();
        let x = g["x"].as_f64().unwrap_or(f64::NAN);
        let y = g["y"].as_f64().unwrap_or(f64::NAN);
        let yaw = g["yaw"].as_f64().unwrap_or(f64::NAN);
        require(yaw.is_finite(), "Invalid yaw")?;
        for i in 1..=40 {
            if stop.load(Ordering::Acquire) {
                return Ok(false);
            }
            std::thread::sleep(std::time::Duration::from_millis(50));
            let p = Pose {
                x: start.x + (x - start.x) * f64::from(i) / 40.,
                y: start.y + (y - start.y) * f64::from(i) / 40.,
                yaw,
            };
            free_goal(&grid(&self.remote_snapshot()?["map"])?, p.x, p.y)?;
            *self.position.lock().unwrap() = p;
        }
        Ok(true)
    }
}
