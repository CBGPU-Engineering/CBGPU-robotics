//! A process-ABI example only. It has no serial, GPIO, ROS or motor access.
use cbgpu_robotics_client::{Result, controller::Adapter, require, simulator::Simulator};
use serde_json::{Value, json};
use std::io::{Read, Write};
fn main() {
    if run().is_err() {
        let _ = std::io::stdout().write_all(b"{\"ok\":false,\"result\":null}");
        std::process::exit(1);
    }
}
fn run() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    require(
        args.len() == 3 && args[1] == "--cbgpu-method",
        "Expected local adapter method",
    )?;
    let mut bytes = vec![];
    std::io::stdin().take(4_194_305).read_to_end(&mut bytes)?;
    require(bytes.len() <= 4_194_304, "Request too large")?;
    let value: Value = serde_json::from_slice(&bytes)?;
    let robot = Simulator::default();
    let result = match args[2].as_str() {
        "describe" => {
            json!({"robot_id":robot.robot_id(),"capabilities":robot.capabilities(),"commissioned":false})
        }
        "observe" => {
            json!({"mode":"simulation","credentials_inherited":std::env::var_os("CBGPU_API_KEY").is_some()||std::env::var_os("CBGPU_DEVICE_TOKEN").is_some()})
        }
        "safety" => serde_json::to_value(robot.safety()?)?,
        "validate_step" => json!(robot.validate_step(&serde_json::from_value(value)?)?),
        "stop" => {
            robot.stop()?;
            json!(true)
        }
        // Hardware integrations must implement these locally, with independent stop/watchdog.
        "execute" | "navigate_to" | "validate_remote_goal" => json!(false),
        _ => anyhow::bail!("Unsupported method"),
    };
    println!("{}", json!({"ok":true,"result":result}));
    Ok(())
}
