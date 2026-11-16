use crate::{Result, now, protocol, require};
use rusqlite::{Connection, params};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::{
    path::Path,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Safety {
    pub armed: bool,
    pub emergency_stop: bool,
    pub sensors_at: f64,
    pub healthy: bool,
}
impl Safety {
    pub fn check(&self) -> Result<()> {
        require(
            self.armed && !self.emergency_stop && self.healthy,
            "Robot is not armed/healthy or emergency stop is active",
        )?;
        let age = now() - self.sensors_at;
        require(
            age.is_finite() && (-0.2..=1.).contains(&age),
            "Robot sensor state is stale",
        )
    }
}
/// Implementations must make stop() independent of a blocked execute()/navigate_to().
pub trait Adapter: Send + Sync + 'static {
    fn robot_id(&self) -> String;
    fn capabilities(&self) -> Value;
    fn commissioned(&self) -> bool;
    fn observe(&self) -> Result<Value>;
    fn safety(&self) -> Result<Safety>;
    fn validate_step(&self, step: &protocol::Step) -> Result<bool>;
    fn execute(&self, name: &str, args: &Value, stop: Arc<AtomicBool>) -> Result<()>;
    fn stop(&self) -> Result<()>;
    fn remote_snapshot(&self) -> Result<Value> {
        anyhow::bail!("Remote mapping unsupported by adapter")
    }
    fn validate_remote_goal(&self, _goal: &Value) -> Result<bool> {
        Ok(false)
    }
    fn navigate_to(&self, _goal: &Value, _stop: Arc<AtomicBool>) -> Result<bool> {
        Ok(false)
    }
}
pub struct Ledger {
    db: Mutex<Connection>,
}
impl Ledger {
    pub fn open(path: &Path) -> Result<Self> {
        if let Some(p) = path.parent() {
            std::fs::create_dir_all(p)?;
        }
        if path.exists() {
            crate::private_file(path)?;
        }
        let mut options = std::fs::OpenOptions::new();
        options.read(true).write(true).create(true);
        #[cfg(unix)]
        {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let _file = options.open(path)?;
        crate::private_file(path)?;
        let db = Connection::open_with_flags(
            path,
            rusqlite::OpenFlags::SQLITE_OPEN_READ_WRITE
                | rusqlite::OpenFlags::SQLITE_OPEN_CREATE
                | rusqlite::OpenFlags::SQLITE_OPEN_NOFOLLOW,
        )?;
        db.busy_timeout(Duration::from_secs(10))?;
        db.execute_batch("CREATE TABLE IF NOT EXISTS missions(id TEXT PRIMARY KEY,robot TEXT,state TEXT,at REAL); CREATE TABLE IF NOT EXISTS robot_locks(robot TEXT PRIMARY KEY,mission TEXT);")?;
        Ok(Self { db: Mutex::new(db) })
    }
    pub fn reserve(&self, robot: &str, id: &str) -> Result<()> {
        let mut db = self.db.lock().unwrap();
        let tx = db.transaction_with_behavior(rusqlite::TransactionBehavior::Immediate)?;
        tx.execute("INSERT INTO robot_locks VALUES(?1,?2)", params![robot, id])
            .map_err(|_| anyhow::anyhow!("Robot locked; verify stopped before local recovery"))?;
        tx.execute(
            "INSERT INTO missions VALUES(?1,?2,'running',?3)",
            params![id, robot, now()],
        )
        .map_err(|_| anyhow::anyhow!("Mission replay blocked"))?;
        tx.commit()?;
        Ok(())
    }
    pub fn finish(&self, robot: &str, id: &str, state: &str, release: bool) -> Result<()> {
        let mut db = self.db.lock().unwrap();
        let tx = db.transaction()?;
        tx.execute(
            "UPDATE missions SET state=?1 WHERE id=?2",
            params![state, id],
        )?;
        if release {
            tx.execute(
                "DELETE FROM robot_locks WHERE robot=?1 AND mission=?2",
                params![robot, id],
            )?;
        }
        tx.commit()?;
        Ok(())
    }
    pub fn previous(&self, id: &str) -> Result<Option<String>> {
        use rusqlite::OptionalExtension;
        Ok(self
            .db
            .lock()
            .unwrap()
            .query_row("SELECT state FROM missions WHERE id=?1", [id], |r| r.get(0))
            .optional()?)
    }
    pub fn unlock(&self, robot: &str, confirmed: bool) -> Result<()> {
        require(
            confirmed,
            "Confirm physical robot stopped before clearing its lock",
        )?;
        self.db
            .lock()
            .unwrap()
            .execute("DELETE FROM robot_locks WHERE robot=?1", [robot])?;
        Ok(())
    }
}
pub struct Controller {
    pub adapter: Arc<dyn Adapter>,
    pub ledger: Arc<Ledger>,
    running: AtomicBool,
}
impl Controller {
    pub fn new(adapter: Arc<dyn Adapter>, ledger: Arc<Ledger>) -> Self {
        Self {
            adapter,
            ledger,
            running: AtomicBool::new(false),
        }
    }
    pub fn run(&self, mission: &Value, execute: bool, approve: bool) -> Result<Value> {
        let caps = self.adapter.capabilities();
        let robot = self.adapter.robot_id();
        let plan = protocol::envelope(mission, &caps, &robot, now())?;
        require(!plan.needs_clarification, "Plan requires clarification")?;
        if !execute {
            return Ok(json!({"mode":"dry-run","steps":plan.steps,"summary":plan.summary}));
        }
        require(
            approve && self.adapter.commissioned(),
            "Execution needs local approval and commissioned adapter",
        )?;
        require(
            !self.running.swap(true, Ordering::AcqRel),
            "Controller already running",
        )?;
        struct Reset<'a>(&'a AtomicBool);
        impl Drop for Reset<'_> {
            fn drop(&mut self) {
                self.0.store(false, Ordering::Release)
            }
        }
        let _reset = Reset(&self.running);
        self.adapter.safety()?.check()?;
        let id = mission["id"].as_str().unwrap_or_default();
        self.ledger.reserve(&robot, id)?;
        let stop = Arc::new(AtomicBool::new(false));
        let mut handle = None;
        let result = (|| {
            for step in &plan.steps {
                protocol::envelope(mission, &caps, &robot, now())?;
                self.adapter.safety()?.check()?;
                require(
                    protocol::digest(&self.adapter.capabilities())? == protocol::digest(&caps)?,
                    "Capabilities changed",
                )?;
                require(
                    self.adapter.validate_step(step)?,
                    "Local adapter rejected step",
                )?;
                let a = self.adapter.clone();
                let s = stop.clone();
                let step = step.clone();
                let deadline = Instant::now() + Duration::from_secs_f64(step.duration_s);
                handle = Some(std::thread::spawn(move || {
                    a.execute(&step.capability, &step.arguments, s)
                }));
                while !handle.as_ref().unwrap().is_finished() {
                    std::thread::sleep(Duration::from_millis(50));
                    self.adapter.safety()?.check()?;
                    require(
                        now() < mission["expires_at"].as_f64().unwrap_or(0.),
                        "Mission expired",
                    )?;
                    require(
                        Instant::now() < deadline || handle.as_ref().unwrap().is_finished(),
                        "Step deadline exceeded",
                    )?;
                }
                handle
                    .take()
                    .unwrap()
                    .join()
                    .map_err(|_| anyhow::anyhow!("Adapter panicked"))??;
            }
            Ok(json!({"mode":"executed","steps":plan.steps.len()}))
        })();
        stop.store(true, Ordering::Release);
        let stopped = self.adapter.stop();
        let deadline = Instant::now() + Duration::from_millis(500);
        while handle.as_ref().is_some_and(|h| !h.is_finished()) && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        let release = handle.as_ref().is_none_or(|h| h.is_finished()) && stopped.is_ok();
        self.ledger.finish(
            &robot,
            id,
            if result.is_ok() && stopped.is_ok() {
                "succeeded"
            } else {
                "failed"
            },
            release,
        )?;
        stopped?;
        result
    }
}
