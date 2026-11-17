//! Opt-in, isolated in-process DDS fixture. Never connects to /navigate_to_pose.
#![cfg(any(feature = "ros2-jazzy", feature = "ros2-humble"))]
use cbgpu_robotics_client::{
    controller::Adapter,
    now,
    ros2::{Config, Ros2Adapter},
    ros2_state::*,
};
use ros2_client::{
    ActionTypeName, Context, ContextOptions, Message, MessageTypeName, Name, Node, NodeName,
    NodeOptions, Publisher, QosProfile, ServiceMapping,
    action::{ActionServerQosPolicies, GetResultResponse, GoalStatusEnum, SendGoalResponse},
    qos::{Durability, History, WhenFull},
};
use serde_json::json;
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};
#[cfg(feature = "ros2-jazzy")]
type R = NavResult;
#[cfg(not(feature = "ros2-jazzy"))]
type R = HumbleNavResult;
fn qos() -> QosProfile {
    QosProfile::publisher_default()
        .reliability_reliable(WhenFull::Fail)
        .history(History::KeepLast { depth: 10 })
        .durability(Durability::Volatile)
}
fn publisher<T: Message + 'static>(
    n: &mut Node,
    prefix: &str,
    suffix: &str,
    pkg: &str,
    ty: &str,
    q: QosProfile,
) -> Publisher<T> {
    let t = n
        .create_topic(
            &Name::new(prefix, suffix).unwrap(),
            MessageTypeName::new(pkg, ty),
            &q,
        )
        .unwrap();
    n.create_publisher(&t, None).unwrap()
}
fn stamp() -> Stamp {
    let t = now();
    Stamp {
        sec: t.floor() as i32,
        nanosec: ((t - t.floor()) * 1e9) as u32,
    }
}
#[test]
#[ignore = "explicit local DDS fixture: run --ignored; private domain 232 and random namespace"]
fn native_dds_maps_nav_success_and_owner_only_confirmed_cancel() {
    let prefix = format!("/cbgpu_test_{}", uuid::Uuid::new_v4().simple());
    let cfg = Config {
        robot_id: "test-robot".into(),
        base_frame: "base_link".into(),
        map_topic: format!("{prefix}/map"),
        scan_topic: format!("{prefix}/scan"),
        arm_topic: format!("{prefix}/armed"),
        estop_topic: format!("{prefix}/estop"),
        image_topic: None,
        nav_action: format!("{prefix}/nav"),
        domain: 232,
        commissioned: true,
    };
    // TF uses standard names, but DDS domain is explicit/private; no system ROS_DEFAULT_DOMAIN is read.
    let quit = Arc::new(AtomicBool::new(false));
    let q = quit.clone();
    let (started, wait) = mpsc::channel();
    let (observed, cancels) = mpsc::channel();
    let release_terminal = Arc::new(AtomicBool::new(false));
    let terminal = release_terminal.clone();
    let server = thread::spawn(move || {
        let context = Context::with_options(ContextOptions::new().domain_id(232)).unwrap();
        let mut node = context
            .new_node(
                NodeName::new(&prefix, "fake_nav").unwrap(),
                NodeOptions::default(),
            )
            .unwrap();
        let map = publisher::<OccupancyGrid>(
            &mut node,
            &prefix,
            "map",
            "nav_msgs",
            "OccupancyGrid",
            qos().durability(Durability::TransientLocal),
        );
        let scan = publisher::<LaserScan>(
            &mut node,
            &prefix,
            "scan",
            "sensor_msgs",
            "LaserScan",
            qos(),
        );
        let arm = publisher::<RosBool>(&mut node, &prefix, "armed", "std_msgs", "Bool", qos());
        let estop = publisher::<RosBool>(&mut node, &prefix, "estop", "std_msgs", "Bool", qos());
        let tf = publisher::<TFMessage>(&mut node, "/", "tf", "tf2_msgs", "TFMessage", qos());
        let nav = node
            .create_action_server::<NavGoal, R, NavFeedback>(
                ServiceMapping::Enhanced,
                &Name::new(&prefix, "nav").unwrap(),
                &ActionTypeName::new("nav2_msgs", "NavigateToPose"),
                ActionServerQosPolicies {
                    goal_service: qos(),
                    result_service: qos(),
                    cancel_service: qos(),
                    feedback_publisher: qos(),
                    status_publisher: qos().durability(Durability::TransientLocal),
                },
            )
            .unwrap();
        let spinner = node.spinner().unwrap();
        let spin = thread::spawn(move || futures::executor::block_on(spinner.spin()));
        started.send(()).unwrap();
        let mut goal = None;
        let mut result_request = None;
        let mut count = 0;
        let mut canceled = false;
        let mut delayed_accept = None;
        while !q.load(Ordering::Acquire) {
            let _ = map.publish(OccupancyGrid {
                header: Header {
                    frame_id: "map".into(),
                    stamp: stamp(),
                },
                info: MapMetadata {
                    width: 3,
                    height: 3,
                    resolution: 1.,
                    ..Default::default()
                },
                data: vec![0; 9],
            });
            let _ = scan.publish(LaserScan {
                header: Header {
                    stamp: stamp(),
                    ..Default::default()
                },
                angle_increment: 0.1,
                range_min: 0.1,
                range_max: 10.,
                ranges: vec![1.; 5],
                ..Default::default()
            });
            let _ = arm.publish(RosBool { data: true });
            let _ = estop.publish(RosBool { data: false });
            let _ = tf.publish(TFMessage {
                transforms: vec![TransformStamped {
                    header: Header {
                        frame_id: "map".into(),
                        stamp: stamp(),
                    },
                    child_frame_id: "base_link".into(),
                    transform: Transform::default(),
                }],
            });
            if let Some((id, g)) = nav.receive_goal().unwrap() {
                count += 1;
                goal = Some(g.goal_id);
                result_request = None;
                canceled = false;
                if count == 3 {
                    delayed_accept = Some((id, Instant::now()));
                } else {
                    nav.send_goal_response(
                        id,
                        SendGoalResponse {
                            accepted: true,
                            stamp: ros2_client::builtin_interfaces::Time::ZERO,
                        },
                    )
                    .unwrap();
                }
            }
            if let Some((id, at)) = delayed_accept
                && at.elapsed() >= Duration::from_millis(800)
            {
                nav.send_goal_response(
                    id,
                    SendGoalResponse {
                        accepted: true,
                        stamp: ros2_client::builtin_interfaces::Time::ZERO,
                    },
                )
                .unwrap();
                delayed_accept = None;
            }
            if let Some((id, _)) = nav.receive_result_request().unwrap() {
                result_request = Some(id);
            }
            if let Some((id, req)) = nav.receive_cancel_request().unwrap() {
                let value = serde_json::to_value(req).unwrap();
                assert_eq!(
                    value["goal_info"]["goal_id"],
                    serde_json::to_value(goal.unwrap()).unwrap()
                );
                assert_ne!(
                    value["goal_info"]["goal_id"],
                    serde_json::to_value(ros2_client::action::GoalId::ZERO).unwrap()
                );
                observed.send(()).unwrap();
                let accepted = delayed_accept.is_none();
                canceled = accepted;
                nav.send_cancel_response(
                    id,
                    ros2_client::action_msgs::CancelGoalResponse {
                        return_code: if accepted {
                            ros2_client::action_msgs::CancelGoalResponseEnum::None
                        } else {
                            ros2_client::action_msgs::CancelGoalResponseEnum::UnknownGoal
                        },
                        goals_canceling: if accepted {
                            vec![ros2_client::action_msgs::GoalInfo {
                                goal_id: goal.unwrap(),
                                stamp: ros2_client::builtin_interfaces::Time::ZERO,
                            }]
                        } else {
                            vec![]
                        },
                    },
                )
                .unwrap();
            }
            if (count == 1 || canceled)
                && (count != 4 || terminal.load(Ordering::Acquire))
                && let Some(id) = result_request.take()
            {
                nav.send_result(
                    id,
                    GetResultResponse {
                        status: if canceled {
                            GoalStatusEnum::Canceled
                        } else {
                            GoalStatusEnum::Succeeded
                        },
                        result: R::default(),
                    },
                )
                .unwrap();
            }
            thread::sleep(Duration::from_millis(40));
        }
        node.stop_spinner();
        let _ = spin.join();
    });
    wait.recv_timeout(Duration::from_secs(10)).unwrap();
    let adapter = Arc::new(Ros2Adapter::open(cfg).unwrap());
    let start = Instant::now();
    while adapter.safety().unwrap().check().is_err() {
        assert!(
            start.elapsed() < Duration::from_secs(30),
            "DDS readiness timeout: {}",
            adapter.remote_snapshot().unwrap()
        );
        thread::sleep(Duration::from_millis(50));
    }
    let snapshot = adapter.remote_snapshot().unwrap();
    let g =
        json!({"frame":"map","map_session":snapshot["map"]["session"],"x":0.5,"y":0.5,"yaw":0.});
    assert!(
        adapter
            .navigate_to(&g, Arc::new(AtomicBool::new(false)))
            .unwrap()
    );
    let a = adapter.clone();
    let g2 = g.clone();
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let task = thread::spawn(move || a.navigate_to(&g2, flag));
    thread::sleep(Duration::from_millis(500));
    stop.store(true, Ordering::Release);
    assert!(!task.join().unwrap().unwrap());
    cancels.recv_timeout(Duration::from_secs(5)).unwrap();
    adapter.stop().unwrap();
    // A stop received before a late goal acceptance must still cancel that goal.
    let a = adapter.clone();
    let goal = g.clone();
    let flag = Arc::new(AtomicBool::new(false));
    let stop = flag.clone();
    let task = thread::spawn(move || a.navigate_to(&goal, flag));
    thread::sleep(Duration::from_millis(200));
    stop.store(true, Ordering::Release);
    assert!(!task.join().unwrap().unwrap());
    // An ACK with CANCELING but no terminal result cannot release a physical lock.
    let a = adapter.clone();
    let goal = g.clone();
    let task = thread::spawn(move || a.navigate_to(&goal, Arc::new(AtomicBool::new(false))));
    thread::sleep(Duration::from_millis(300));
    assert!(adapter.stop().is_err());
    release_terminal.store(true, Ordering::Release);
    assert!(!task.join().unwrap().unwrap());
    adapter.stop().unwrap();
    drop(adapter);
    quit.store(true, Ordering::Release);
    server.join().unwrap();
}
