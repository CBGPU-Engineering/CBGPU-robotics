use cbgpu_robotics_client::{
    Result,
    adapter::ProcessAdapter,
    client::{Client, DEFAULT},
    controller::{Adapter, Controller, Ledger},
    now, protocol,
    remote::{RemoteAgent, credentials_dir, enroll, load_credentials},
    require,
    simulator::Simulator,
};
use clap::{Parser, Subcommand};
use serde_json::json;
use std::{
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};
#[derive(Parser)]
#[command(
    version,
    about = "Outbound-only robotics SDK; simulation and dry-run by default"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}
#[derive(Subcommand)]
enum Commands {
    Demo {
        #[arg(long)]
        execute_simulation: bool,
    },
    Plan {
        instruction: String,
        #[arg(long)]
        adapter: Option<PathBuf>,
        #[arg(long)]
        execute: bool,
        #[arg(long)]
        approve: bool,
        #[arg(long, default_value = "cbgpu-coder")]
        model: String,
        #[arg(long,default_value="medium",value_parser=["low","medium","high"])]
        effort: String,
    },
    Connect {
        #[arg(long,default_value=DEFAULT)]
        base_url: String,
    },
    Agent {
        #[arg(long)]
        device: String,
        #[arg(long)]
        adapter: Option<PathBuf>,
        #[arg(long)]
        simulator: bool,
        #[arg(long)]
        allow_navigation: bool,
    },
    Unlock {
        robot_id: String,
        #[arg(long)]
        confirm_stopped: bool,
    },
}
fn adapter(path: Option<PathBuf>) -> Result<Arc<dyn Adapter>> {
    Ok(if let Some(path) = path {
        Arc::new(ProcessAdapter::open(path)?)
    } else {
        Arc::new(Simulator::default())
    })
}
fn main() {
    if let Err(error) = run() {
        eprintln!("CBGPU Robotics: {error}");
        std::process::exit(1);
    }
}
fn run() -> Result<()> {
    let cli = Cli::parse();
    let root = credentials_dir()?.parent().unwrap().to_path_buf();
    if let Commands::Connect { base_url } = cli.command {
        let code = rpassword::prompt_password("One-time pairing code (hidden): ")?;
        let c = enroll(code.trim(), &base_url, &credentials_dir()?)?;
        println!(
            "Device connected: {}\nRun: cbgpu-robotics agent --device {} --simulator",
            c.device_id, c.device_id
        );
        return Ok(());
    }
    let ledger = Arc::new(Ledger::open(&root.join("missions.sqlite"))?);
    match cli.command {
        Commands::Demo { execute_simulation } => {
            let robot = adapter(None)?;
            let t = now();
            let m = json!({"protocol_version":1,"id":uuid::Uuid::new_v4().to_string(),"robot_id":robot.robot_id(),"capabilities_digest":protocol::digest(&robot.capabilities())?,"issued_at":t,"observed_at":t,"expires_at":t+60.,"requires_local_approval":true,"plan":{"summary":"Offline virtual-robot demonstration","needs_clarification":false,"question":"","steps":[{"capability":"navigate_to","arguments":{"waypoint":"charging_station"},"duration_s":5}]}});
            println!(
                "{}",
                serde_json::to_string_pretty(&Controller::new(robot, ledger).run(
                    &m,
                    execute_simulation,
                    execute_simulation
                )?)?
            );
        }
        Commands::Plan {
            instruction,
            adapter: path,
            execute,
            approve,
            model,
            effort,
        } => {
            let robot = adapter(path)?;
            let key = std::env::var("CBGPU_API_KEY").unwrap_or_default();
            let m = Client::new(key, DEFAULT, Duration::from_secs(20))?.plan(
                &instruction,
                &robot.robot_id(),
                &robot.capabilities(),
                &robot.observe()?,
                &model,
                &effort,
                Duration::from_secs(180),
            )?;
            println!(
                "{}",
                serde_json::to_string_pretty(
                    &Controller::new(robot, ledger).run(&m, execute, approve)?
                )?
            );
        }
        Commands::Unlock {
            robot_id,
            confirm_stopped,
        } => {
            protocol::robot_id(&robot_id)?;
            ledger.unlock(&robot_id, confirm_stopped)?;
            println!("Local lock cleared; previously used mission IDs remain blocked.");
        }
        Commands::Agent {
            device,
            adapter: path,
            simulator,
            allow_navigation,
        } => {
            require(
                simulator != path.is_some(),
                "Choose --simulator or --adapter absolute-path",
            )?;
            let credentials = load_credentials(&credentials_dir()?, &device)?;
            let quit = Arc::new(AtomicBool::new(false));
            let q = quit.clone();
            ctrlc::set_handler(move || q.store(true, Ordering::Release))?;
            let mut agent =
                RemoteAgent::new(adapter(path)?, ledger, credentials, allow_navigation, quit)?;
            println!(
                "Outbound agent running; navigation {}",
                if allow_navigation {
                    "requires local commissioning and live control lease"
                } else {
                    "disabled"
                }
            );
            agent.run()?;
        }
        Commands::Connect { .. } => unreachable!(),
    }
    Ok(())
}
