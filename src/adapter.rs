//! Explicit local process boundary for existing ROS/serial drivers. Never a cloud shell runner.
use crate::{
    Result,
    controller::{Adapter, Safety},
    protocol::Step,
    require,
};
use serde_json::{Value, json};
use std::{
    io::{Read, Write},
    path::PathBuf,
    process::{Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
pub struct ProcessAdapter {
    path: PathBuf,
    description: Value,
}
impl ProcessAdapter {
    pub fn open(path: PathBuf) -> Result<Self> {
        require(
            path.is_absolute() && std::fs::metadata(&path)?.is_file(),
            "Adapter must be an explicit absolute local executable",
        )?;
        let description = invoke(&path, "describe", &json!({}), None, Duration::from_secs(3))?;
        crate::protocol::robot_id(description["robot_id"].as_str().unwrap_or_default())?;
        crate::protocol::capabilities(&description["capabilities"])?;
        Ok(Self { path, description })
    }
    fn call(&self, method: &str, value: &Value) -> Result<Value> {
        invoke(&self.path, method, value, None, Duration::from_secs(2))
    }
}
fn invoke(
    path: &PathBuf,
    method: &str,
    value: &Value,
    stop: Option<Arc<AtomicBool>>,
    timeout: Duration,
) -> Result<Value> {
    let payload = serde_json::to_vec(value)?;
    require(
        payload.len() <= 4_194_304,
        "Local adapter request too large",
    )?;
    let mut child = Command::new(path)
        .arg("--cbgpu-method")
        .arg(method)
        .env_clear()
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()?;
    let mut stdin = child.stdin.take().unwrap();
    let (written_tx, written_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let _ = written_tx.send(stdin.write_all(&payload));
    });
    let stdout = child.stdout.take().unwrap();
    let (read_tx, read_rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let result = (|| -> std::io::Result<Vec<u8>> {
            let mut bytes = Vec::new();
            stdout.take(4_194_305).read_to_end(&mut bytes)?;
            Ok(bytes)
        })();
        let _ = read_tx.send(result);
    });
    let deadline = Instant::now() + timeout;
    let status = loop {
        if let Some(status) = child.try_wait()? {
            break status;
        }
        if stop.as_ref().is_some_and(|s| s.load(Ordering::Acquire)) || Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            anyhow::bail!("Local adapter cancelled or deadline exceeded");
        }
        std::thread::sleep(Duration::from_millis(10));
    };
    require(status.success(), "Local adapter rejected operation")?;
    written_rx
        .recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .map_err(|_| anyhow::anyhow!("Adapter input did not close"))??;
    let bytes = read_rx
        .recv_timeout(deadline.saturating_duration_since(Instant::now()))
        .map_err(|_| anyhow::anyhow!("Adapter output did not close"))??;
    require(bytes.len() <= 4_194_304, "Adapter response too large")?;
    let response: Value = serde_json::from_slice(&bytes)?;
    require(response["ok"] == true, "Local adapter operation failed")?;
    Ok(response["result"].clone())
}
impl Adapter for ProcessAdapter {
    fn robot_id(&self) -> String {
        self.description["robot_id"].as_str().unwrap().into()
    }
    fn capabilities(&self) -> Value {
        self.description["capabilities"].clone()
    }
    fn commissioned(&self) -> bool {
        self.description["commissioned"] == true
    }
    fn observe(&self) -> Result<Value> {
        self.call("observe", &json!({}))
    }
    fn safety(&self) -> Result<Safety> {
        Ok(serde_json::from_value(self.call("safety", &json!({}))?)?)
    }
    fn validate_step(&self, step: &Step) -> Result<bool> {
        Ok(self.call("validate_step", &serde_json::to_value(step)?)? == true)
    }
    fn execute(&self, name: &str, args: &Value, stop: Arc<AtomicBool>) -> Result<()> {
        require(
            invoke(
                &self.path,
                "execute",
                &json!({"capability":name,"arguments":args}),
                Some(stop),
                Duration::from_secs(31),
            )? == true,
            "Adapter did not acknowledge execution",
        )
    }
    fn stop(&self) -> Result<()> {
        require(
            self.call("stop", &json!({}))? == true,
            "Adapter did not acknowledge stop",
        )
    }
    fn remote_snapshot(&self) -> Result<Value> {
        self.call("remote_snapshot", &json!({}))
    }
    fn validate_remote_goal(&self, goal: &Value) -> Result<bool> {
        Ok(self.call("validate_remote_goal", goal)? == true)
    }
    fn navigate_to(&self, goal: &Value, stop: Arc<AtomicBool>) -> Result<bool> {
        Ok(invoke(
            &self.path,
            "navigate_to",
            goal,
            Some(stop),
            Duration::from_secs(180),
        )? == true)
    }
}
