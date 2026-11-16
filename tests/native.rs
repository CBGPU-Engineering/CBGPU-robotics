use cbgpu_robotics_client::{
    Result,
    client::Client,
    controller::{Adapter, Controller, Ledger, Safety},
    now, protocol,
    remote::{Credentials, RemoteAgent, pack_grid},
    remote_protocol::{Pose, cell, free_goal},
    simulator::Simulator,
};
use serde_json::{Value, json};
use std::{
    io::{Read, Write},
    net::TcpListener,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};
fn mission(robot: &dyn Adapter) -> Value {
    let t = now();
    json!({"protocol_version":1,"id":uuid::Uuid::new_v4().to_string(),"robot_id":robot.robot_id(),"capabilities_digest":protocol::digest(&robot.capabilities()).unwrap(),"issued_at":t,"observed_at":t,"expires_at":t+60.,"requires_local_approval":true,"plan":{"summary":"Move","needs_clarification":false,"question":"","steps":[{"capability":"navigate_to","arguments":{"waypoint":"desk"},"duration_s":2}]}})
}
fn setup() -> (tempfile::TempDir, Arc<Simulator>, Arc<Ledger>) {
    let tmp = tempfile::tempdir().unwrap();
    let r = Arc::new(Simulator::default());
    let l = Arc::new(Ledger::open(&tmp.path().join("missions.sqlite")).unwrap());
    (tmp, r, l)
}
#[test]
fn dry_run_approval_commissioning_and_replay() {
    let (_t, r, l) = setup();
    let c = Controller::new(r.clone(), l.clone());
    let m = mission(r.as_ref());
    assert_eq!(c.run(&m, false, false).unwrap()["mode"], "dry-run");
    assert!(r.events.lock().unwrap().is_empty());
    assert!(c.run(&m, true, false).is_err());
    r.commissioned.store(false, Ordering::Release);
    assert!(c.run(&m, true, true).is_err());
    r.commissioned.store(true, Ordering::Release);
    assert_eq!(c.run(&m, true, true).unwrap()["mode"], "executed");
    assert!(r.stopped.load(Ordering::Acquire));
    assert!(Controller::new(r, l).run(&m, true, true).is_err());
}
#[test]
fn rejects_robot_expiry_hash_unknown_extra_bounds_and_clarification() {
    let (_t, r, l) = setup();
    for field in 0..8 {
        let mut m = mission(r.as_ref());
        match field {
            0 => m["robot_id"] = json!("other"),
            1 => m["expires_at"] = json!(0),
            2 => m["capabilities_digest"] = json!("bad"),
            3 => m["plan"]["steps"][0]["capability"] = json!("shell"),
            4 => m["plan"]["steps"][0]["arguments"]["command"] = json!("no"),
            5 => m["plan"]["steps"][0]["duration_s"] = json!(100),
            6 => m["plan"]["steps"][0]["arguments"]["waypoint"] = json!("hallucinated"),
            _ => {
                m["plan"]["needs_clarification"] = json!(true);
                m["plan"]["question"] = json!("Where?");
                m["plan"]["steps"] = json!([]);
            }
        }
        assert!(
            Controller::new(r.clone(), l.clone())
                .run(&m, true, true)
                .is_err()
        );
    }
    assert!(r.events.lock().unwrap().is_empty());
}
struct Fault {
    base: Simulator,
    stale: bool,
    block: bool,
    trip: bool,
    done: Arc<AtomicBool>,
}
impl Adapter for Fault {
    fn robot_id(&self) -> String {
        self.base.robot_id()
    }
    fn capabilities(&self) -> Value {
        self.base.capabilities()
    }
    fn commissioned(&self) -> bool {
        true
    }
    fn observe(&self) -> Result<Value> {
        self.base.observe()
    }
    fn safety(&self) -> Result<Safety> {
        let mut s = self.base.safety()?;
        if self.stale {
            s.sensors_at -= 10.;
        }
        Ok(s)
    }
    fn validate_step(&self, s: &protocol::Step) -> Result<bool> {
        self.base.validate_step(s)
    }
    fn execute(&self, n: &str, a: &Value, stop: Arc<AtomicBool>) -> Result<()> {
        if self.trip {
            self.base.estop.store(true, Ordering::Release);
        }
        if self.block {
            while !self.done.load(Ordering::Acquire) {
                std::thread::sleep(Duration::from_millis(10));
            }
            return Ok(());
        }
        self.base.execute(n, a, stop)
    }
    fn stop(&self) -> Result<()> {
        self.base.stop()
    }
}
#[test]
fn stale_and_estop_fail_before_motion() {
    for stale in [false, true] {
        let tmp = tempfile::tempdir().unwrap();
        let r = Arc::new(Fault {
            base: Simulator::default(),
            stale,
            block: false,
            trip: false,
            done: Arc::new(AtomicBool::new(false)),
        });
        if !stale {
            r.base.estop.store(true, Ordering::Release);
        }
        let m = mission(r.as_ref());
        assert!(
            Controller::new(
                r.clone(),
                Arc::new(Ledger::open(&tmp.path().join("db")).unwrap())
            )
            .run(&m, true, true)
            .is_err()
        );
        assert!(r.base.events.lock().unwrap().is_empty());
    }
}
#[test]
fn mid_step_estop_and_blocked_driver_retain_crash_lock() {
    for block in [false, true] {
        let tmp = tempfile::tempdir().unwrap();
        let done = Arc::new(AtomicBool::new(false));
        let r = Arc::new(Fault {
            base: Simulator::default(),
            stale: false,
            block,
            trip: !block,
            done: done.clone(),
        });
        let l = Arc::new(Ledger::open(&tmp.path().join("db")).unwrap());
        let mut m = mission(r.as_ref());
        m["plan"]["steps"][0]["duration_s"] = json!(0.05);
        assert!(
            Controller::new(r.clone(), l.clone())
                .run(&m, true, true)
                .is_err()
        );
        assert!(r.base.stopped.load(Ordering::Acquire));
        if block {
            assert!(
                l.reserve(&r.robot_id(), &uuid::Uuid::new_v4().to_string())
                    .is_err()
            );
            assert!(l.unlock(&r.robot_id(), false).is_err());
            l.unlock(&r.robot_id(), true).unwrap();
        }
        done.store(true, Ordering::Release);
    }
}
#[test]
fn schema_and_digest_match_backend() {
    let r = Simulator::default();
    let mut c = r.capabilities();
    c[0]["parameters"]["additionalProperties"] = json!(true);
    assert!(protocol::capabilities(&c).is_err());
    let c = r.capabilities();
    assert_eq!(protocol::digest(&c).unwrap().len(), 64);
    let mut c = c;
    c[0]["parameters"]["properties"]["waypoint"]["pattern"] = json!(".*");
    assert!(protocol::capabilities(&c).is_err());
}
#[test]
fn map_rotation_unknown_and_downsample_conservative() {
    let m = pack_grid(
        4,
        4,
        0.5,
        Pose {
            x: 1.,
            y: 2.,
            yaw: std::f64::consts::FRAC_PI_2,
        },
        &[0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 100, 100, 100, 100],
        "map",
        "session",
        1,
    )
    .unwrap();
    assert_eq!(cell(&m, 0.75, 2.25).unwrap(), 0);
    assert!(free_goal(&m, -0.75, 2.25).is_err());
    let mut data = vec![0; 1024];
    data[0] = 100;
    data[2] = -1;
    let m = pack_grid(
        1024,
        1,
        0.05,
        Pose {
            x: 0.,
            y: 0.,
            yaw: 0.,
        },
        &data,
        "map",
        "s",
        1,
    )
    .unwrap();
    assert_eq!(m.width, 512);
    assert_eq!(cell(&m, 0.01, 0.01).unwrap(), 100);
    assert_eq!(cell(&m, 0.11, 0.01).unwrap(), -1);
    assert!(
        pack_grid(
            1,
            1,
            1.,
            Pose {
                x: 0.,
                y: 0.,
                yaw: 0.
            },
            &[101],
            "map",
            "s",
            1
        )
        .is_err()
    );
}
#[test]
fn transport_rejects_credentials_redirect_urls() {
    for u in [
        "http://example.com",
        "https://u:secret@example.com",
        "https://example.com/?token=x",
        "https://example.com/#x",
    ] {
        assert!(Client::new("key".into(), u, Duration::from_secs(1)).is_err());
    }
}
fn server(
    count: usize,
    handler: impl Fn(usize, String, Value) -> Value + Send + 'static,
) -> (String, std::thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let worker = std::thread::spawn(move || {
        for (i, s) in listener.incoming().take(count).enumerate() {
            let mut s = s.unwrap();
            s.set_read_timeout(Some(Duration::from_secs(4))).unwrap();
            let mut bytes = vec![];
            let mut one = [0; 1];
            while !bytes.ends_with(b"\r\n\r\n") {
                s.read_exact(&mut one).unwrap();
                bytes.push(one[0]);
            }
            let header = String::from_utf8(bytes).unwrap();
            let len = header
                .lines()
                .find_map(|l| {
                    l.to_lowercase()
                        .strip_prefix("content-length: ")
                        .map(|v| v.parse::<usize>().unwrap())
                })
                .unwrap_or(0);
            let mut body = vec![0; len];
            s.read_exact(&mut body).unwrap();
            let value = if body.is_empty() {
                Value::Null
            } else {
                serde_json::from_slice(&body).unwrap()
            };
            let reply = serde_json::to_vec(&handler(i, header, value)).unwrap();
            write!(s,"HTTP/1.1 200 OK\r\nContent-Length: {}\r\nContent-Type: application/json\r\nConnection: close\r\n\r\n",reply.len()).unwrap();
            s.write_all(&reply).unwrap();
        }
    });
    (url, worker)
}
#[test]
fn authenticated_async_planning_round_trip() {
    let r = Simulator::default();
    let m = mission(&r);
    let expected = m.clone();
    let (url, h) = server(2, move |i, headers, body| {
        assert!(
            headers
                .to_lowercase()
                .contains("authorization: bearer test-key")
        );
        if i == 0 {
            assert_eq!(body["robot_id"], "virtual-robot");
            json!({"id":m["id"],"state":"queued"})
        } else {
            json!({"id":m["id"],"state":"succeeded","mission":m})
        }
    });
    let c = Client::new("test-key".into(), &url, Duration::from_secs(2)).unwrap();
    assert_eq!(
        c.plan(
            "Move",
            &r.robot_id(),
            &r.capabilities(),
            &r.observe().unwrap(),
            "cbgpu-coder",
            "medium",
            Duration::from_secs(3)
        )
        .unwrap(),
        expected
    );
    h.join().unwrap();
}
#[test]
fn remote_upload_caches_map_and_never_embeds_token() {
    let (_t, r, l) = setup();
    let (url, h) = server(2, move |i, headers, body| {
        assert!(headers.contains("/robotics/agent/sync"));
        assert!(
            headers
                .to_lowercase()
                .contains("authorization: bearer device-secret")
        );
        assert!(body.get("device_token").is_none());
        assert_eq!(body.get("map").is_some(), i == 0);
        assert_eq!(body["telemetry"]["navigation_enabled"], false);
        json!({"operator_live":false,"stop_epoch":0,"command":null})
    });
    let mut a = RemoteAgent::new(
        r.clone(),
        l,
        Credentials {
            device_id: uuid::Uuid::new_v4().to_string(),
            device_token: "device-secret".into(),
            base_url: url,
        },
        false,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    a.sync_once().unwrap();
    a.sync_once().unwrap();
    assert!(!r.stopped.load(Ordering::Acquire));
    h.join().unwrap();
}
#[test]
fn remote_stop_epoch_forces_stop_without_active_mission() {
    let (_t, r, l) = setup();
    let (url, h) = server(
        2,
        move |i, _, _| json!({"operator_live":false,"stop_epoch":i,"command":null}),
    );
    let mut a = RemoteAgent::new(
        r.clone(),
        l,
        Credentials {
            device_id: uuid::Uuid::new_v4().to_string(),
            device_token: "device-secret".into(),
            base_url: url,
        },
        false,
        Arc::new(AtomicBool::new(false)),
    )
    .unwrap();
    a.sync_once().unwrap();
    assert!(!r.stopped.load(Ordering::Acquire));
    a.sync_once().unwrap();
    assert!(r.stopped.load(Ordering::Acquire));
    h.join().unwrap();
}
#[test]
fn legacy_sqlite_missions_and_locks_preserved() {
    let t = tempfile::tempdir().unwrap();
    let p = t.path().join("db");
    let db = rusqlite::Connection::open(&p).unwrap();
    db.execute_batch("CREATE TABLE missions(id TEXT PRIMARY KEY,robot TEXT,state TEXT,at REAL);CREATE TABLE robot_locks(robot TEXT PRIMARY KEY,mission TEXT);INSERT INTO missions VALUES('legacy','robot','running',1);INSERT INTO robot_locks VALUES('robot','legacy');").unwrap();
    drop(db);
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)).unwrap();
    }
    let l = Ledger::open(&p).unwrap();
    assert_eq!(l.previous("legacy").unwrap().as_deref(), Some("running"));
    assert!(l.reserve("robot", "new").is_err());
}
#[test]
fn private_credential_symlink_rejected() {
    let t = tempfile::tempdir().unwrap();
    let p = t.path().join("file");
    std::fs::write(&p, b"secret").unwrap();
    #[cfg(unix)]
    {
        use std::os::unix::fs::{PermissionsExt, symlink};
        std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o600)).unwrap();
        let link = t.path().join("link");
        symlink(&p, &link).unwrap();
        assert!(cbgpu_robotics_client::private_file(&link).is_err());
    }
}
#[test]
fn remote_navigation_goal_is_once_only_and_connection_loss_stops() {
    for lose_connection in [false, true] {
        let (_t, r, l) = setup();
        let map = r.remote_snapshot().unwrap()["map"].clone();
        let id = uuid::Uuid::new_v4().to_string();
        let command = json!({"id":id,"expires_at":now()+20.,"payload":{"x":2.2,"y":2.1,"yaw":0.,"frame":map["frame"],"map_session":map["session"],"map_version":map["version"]}});
        let acknowledgments = Arc::new(std::sync::Mutex::new(Vec::new()));
        let received = acknowledgments.clone();
        let count = if lose_connection { 1 } else { 28 };
        let (url, h) = server(count, move |_, _, body| {
            if !body["ack"].is_null() {
                received.lock().unwrap().push(body["ack"].clone());
            }
            json!({"operator_live":true,"stop_epoch":0,"command":command})
        });
        let mut a = RemoteAgent::new(
            r.clone(),
            l.clone(),
            Credentials {
                device_id: uuid::Uuid::new_v4().to_string(),
                device_token: "device-secret".into(),
                base_url: url,
            },
            true,
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        a.sync_once().unwrap();
        if lose_connection {
            h.join().unwrap();
            std::thread::sleep(Duration::from_millis(1900));
            assert_ne!(l.previous(&id).unwrap().as_deref(), Some("succeeded"));
            assert!(r.stopped.load(Ordering::Acquire));
        } else {
            for _ in 1..count {
                std::thread::sleep(Duration::from_millis(100));
                a.sync_once().unwrap();
            }
            h.join().unwrap();
            assert_eq!(l.previous(&id).unwrap().as_deref(), Some("succeeded"));
            assert!(
                acknowledgments
                    .lock()
                    .unwrap()
                    .iter()
                    .any(|a| a["state"] == "succeeded")
            );
        }
    }
}
#[test]
fn remote_default_disabled_and_unknown_cells_rejected() {
    for enabled in [false, true] {
        let (_t, r, l) = setup();
        let map = r.remote_snapshot().unwrap()["map"].clone();
        let id = uuid::Uuid::new_v4().to_string();
        let command = json!({"id":id,"expires_at":now()+20.,"payload":{"x":7.,"y":7.,"yaw":0.,"frame":map["frame"],"map_session":map["session"],"map_version":map["version"]}});
        let (url, h) = server(
            1,
            move |_, _, _| json!({"operator_live":true,"stop_epoch":0,"command":command}),
        );
        let mut a = RemoteAgent::new(
            r.clone(),
            l.clone(),
            Credentials {
                device_id: uuid::Uuid::new_v4().to_string(),
                device_token: "device-secret".into(),
                base_url: url,
            },
            enabled,
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        a.sync_once().unwrap();
        h.join().unwrap();
        std::thread::sleep(Duration::from_millis(100));
        assert!(l.previous(&id).unwrap().is_none());
        let p = r.remote_snapshot().unwrap();
        assert_eq!(p["pose"]["x"], 2.);
    }
}
#[test]
fn explicit_process_adapter_is_uncommissioned_and_has_no_cloud_credentials() {
    use cbgpu_robotics_client::adapter::ProcessAdapter;
    let r = ProcessAdapter::open(std::path::PathBuf::from(env!(
        "CARGO_BIN_EXE_cbgpu-robotics-adapter-example"
    )))
    .unwrap();
    assert!(!r.commissioned());
    assert_eq!(r.observe().unwrap()["credentials_inherited"], false);
    r.safety().unwrap().check().unwrap();
    r.stop().unwrap();
    assert!(ProcessAdapter::open(std::path::PathBuf::from("relative-driver")).is_err());
}
#[test]
fn pairing_is_scoped_hidden_and_saved_private_once() {
    use cbgpu_robotics_client::remote::{enroll, load_credentials};
    let t = tempfile::tempdir().unwrap();
    let id = uuid::Uuid::new_v4().to_string();
    let expected = id.clone();
    let (url, h) = server(1, move |_, headers, body| {
        assert!(headers.contains("/robotics/agent/enroll"));
        assert_eq!(body["pair_code"], "one-time-code");
        assert!(
            body["agent_session"]
                .as_str()
                .is_some_and(cbgpu_robotics_client::valid_id)
        );
        json!({"device_id":id,"device_token":"private-device-token","name":"test"})
    });
    let c = enroll("one-time-code", &url, t.path()).unwrap();
    assert_eq!(c.device_id, expected);
    assert_eq!(
        load_credentials(t.path(), &expected).unwrap().device_token,
        "private-device-token"
    );
    h.join().unwrap();
    cbgpu_robotics_client::private_file(&t.path().join(format!("{expected}.json"))).unwrap();
}
#[test]
fn redirects_and_oversize_responses_are_rejected_without_following() {
    for redirect in [false, true] {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let worker = std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            let mut input = [0; 4096];
            let _ = s.read(&mut input);
            if redirect {
                write!(s,"HTTP/1.1 302 Found\r\nLocation: https://example.com/collect-secret\r\nContent-Length: 0\r\n\r\n").unwrap();
            } else {
                let bytes = vec![b' '; 1_048_577];
                write!(
                    s,
                    "HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n",
                    bytes.len()
                )
                .unwrap();
                let _ = s.write_all(&bytes);
            }
        });
        let client = Client::new("must-not-leak".into(), &url, Duration::from_secs(2)).unwrap();
        let err = client
            .request("/robotics/devices", reqwest::Method::GET, None)
            .unwrap_err()
            .to_string();
        assert!(!err.contains("must-not-leak"));
        assert!(err.contains(if redirect { "redirects" } else { "too large" }));
        worker.join().unwrap();
    }
}
struct BadStop(Simulator);
impl Adapter for BadStop {
    fn robot_id(&self) -> String {
        self.0.robot_id()
    }
    fn capabilities(&self) -> Value {
        self.0.capabilities()
    }
    fn commissioned(&self) -> bool {
        true
    }
    fn observe(&self) -> Result<Value> {
        self.0.observe()
    }
    fn safety(&self) -> Result<Safety> {
        self.0.safety()
    }
    fn validate_step(&self, s: &protocol::Step) -> Result<bool> {
        self.0.validate_step(s)
    }
    fn execute(&self, n: &str, v: &Value, s: Arc<AtomicBool>) -> Result<()> {
        self.0.execute(n, v, s)
    }
    fn stop(&self) -> Result<()> {
        anyhow::bail!("Stop not acknowledged")
    }
}
#[test]
fn failed_stop_does_not_release_physical_robot_lock() {
    let t = tempfile::tempdir().unwrap();
    let r = Arc::new(BadStop(Simulator::default()));
    let l = Arc::new(Ledger::open(&t.path().join("db")).unwrap());
    let m = mission(r.as_ref());
    assert!(
        Controller::new(r.clone(), l.clone())
            .run(&m, true, true)
            .is_err()
    );
    assert!(
        l.reserve(&r.robot_id(), &uuid::Uuid::new_v4().to_string())
            .is_err()
    );
}
#[test]
fn wire_digest_retains_existing_unicode_float_canonical_hash() {
    let caps = json!([{"name":"set_speed","description":"속도 설정","parameters":{"type":"object","properties":{"speed":{"type":"number","minimum":0.00001,"maximum":1e20}},"required":["speed"],"additionalProperties":false},"max_duration_s":1.0}]);
    assert_eq!(
        protocol::digest(&caps).unwrap(),
        "a397c62cf2cabac3258c26038de2deab35b202bd80738acc6350f182b68339d9"
    );
}
