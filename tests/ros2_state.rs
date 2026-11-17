use cbgpu_robotics_client::{now, ros2_state::*};
use serde_json::json;
fn stamp(t: f64) -> Stamp {
    Stamp {
        sec: t.floor() as i32,
        nanosec: ((t - t.floor()) * 1e9) as u32,
    }
}
fn map() -> OccupancyGrid {
    OccupancyGrid {
        header: Header {
            frame_id: "map".into(),
            ..Default::default()
        },
        info: MapMetadata {
            width: 3,
            height: 3,
            resolution: 1.,
            ..Default::default()
        },
        data: vec![0, 0, 100, 0, -1, 0, 0, 0, 0],
    }
}
fn ready() -> Ros2State {
    let mut s = Ros2State::new("base_link".into()).unwrap();
    s.on_map(map()).unwrap();
    s.navigation_ready = true;
    s.on_tf(
        TFMessage {
            transforms: vec![TransformStamped {
                header: Header {
                    frame_id: "map".into(),
                    stamp: stamp(now()),
                },
                child_frame_id: "base_link".into(),
                transform: Transform {
                    translation: Vector3 {
                        x: 0.5,
                        y: 0.5,
                        z: 0.,
                    },
                    ..Default::default()
                },
            }],
        },
        false,
    )
    .unwrap();
    s.on_scan(LaserScan {
        header: Header {
            stamp: stamp(now()),
            ..Default::default()
        },
        angle_increment: 0.1,
        range_min: 0.1,
        range_max: 10.,
        ranges: vec![1.; 5],
        ..Default::default()
    })
    .unwrap();
    s.on_arm(true);
    s.on_estop(false);
    s
}
#[test]
fn sensor_arm_estop_map_and_nav_are_all_required() {
    let mut s = ready();
    s.safety(now()).check().unwrap();
    s.on_estop(true);
    assert!(s.safety(now()).check().is_err());
    s.on_estop(false);
    s.on_arm(false);
    assert!(s.safety(now()).check().is_err());
    s.on_arm(true);
    s.navigation_ready = false;
    assert!(s.safety(now()).check().is_err());
    let s = Ros2State::new("base_link".into()).unwrap();
    assert!(s.safety(now()).check().is_err());
}
#[test]
fn rejects_obstacle_unknown_old_session_and_map_reset() {
    let mut s = ready();
    let old = s.map.as_ref().unwrap().session.clone();
    let g = json!({"frame":"map","map_session":old,"x":0.5,"y":0.5});
    assert!(s.validate_goal(&g, now()));
    assert!(!s.validate_goal(
        &json!({"frame":"map","map_session":old,"x":1.5,"y":1.5}),
        now()
    ));
    assert!(!s.validate_goal(
        &json!({"frame":"map","map_session":old,"x":2.5,"y":0.5}),
        now()
    ));
    let mut next = map();
    next.info.map_load_time = Stamp { sec: 1, nanosec: 0 };
    s.on_map(next).unwrap();
    assert!(!s.validate_goal(&g, now()));
    assert_ne!(s.map.as_ref().unwrap().session, old);
}
#[test]
fn future_and_stale_sensor_stamps_fail_closed() {
    let mut s = ready();
    let scan = |t| LaserScan {
        header: Header {
            stamp: stamp(t),
            ..Default::default()
        },
        angle_increment: 0.1,
        range_min: 0.1,
        range_max: 10.,
        ranges: vec![1.; 5],
        ..Default::default()
    };
    s.on_scan(scan(now() + 5.)).unwrap();
    assert!(s.safety(now()).check().is_err());
    s.on_scan(scan(now() - 5.)).unwrap();
    assert!(s.safety(now()).check().is_err());
    assert!(s.on_scan(LaserScan::default()).is_err());
    assert!(s.safety(now()).check().is_err());
}
#[test]
fn composes_inverse_and_dynamic_tf_chain() {
    let mut s = Ros2State::new("base_link".into()).unwrap();
    s.on_map(map()).unwrap();
    s.on_tf(
        TFMessage {
            transforms: vec![
                TransformStamped {
                    header: Header {
                        frame_id: "odom".into(),
                        ..Default::default()
                    },
                    child_frame_id: "map".into(),
                    transform: Transform {
                        translation: Vector3 {
                            x: -1.,
                            y: 0.,
                            z: 0.,
                        },
                        ..Default::default()
                    },
                },
                TransformStamped {
                    header: Header {
                        frame_id: "odom".into(),
                        stamp: stamp(now()),
                    },
                    child_frame_id: "base_link".into(),
                    transform: Transform {
                        translation: Vector3 {
                            x: 2.,
                            y: 0.,
                            z: 0.,
                        },
                        ..Default::default()
                    },
                },
            ],
        },
        true,
    )
    .unwrap();
    let (p, age) = s.pose(now()).unwrap();
    assert!((p.x - 3.).abs() < 1e-9 && p.y.abs() < 1e-9 && age <= 1.);
    let invalid = TFMessage {
        transforms: vec![TransformStamped {
            header: Header {
                frame_id: "map".into(),
                ..Default::default()
            },
            child_frame_id: "base_link".into(),
            transform: Transform {
                rotation: Quaternion {
                    x: 0.,
                    y: 0.,
                    z: 0.,
                    w: 0.,
                },
                ..Default::default()
            },
        }],
    };
    assert!(s.on_tf(invalid, false).is_err());
}
#[test]
fn image_must_be_bounded_jpeg_and_fresh() {
    let mut s = ready();
    s.on_image(CompressedImage {
        header: Header {
            stamp: stamp(now()),
            ..Default::default()
        },
        format: "jpeg".into(),
        data: vec![255, 216, 255, 217],
    })
    .unwrap();
    assert!(s.snapshot(now())["image_jpeg"].is_string());
    assert!(s.snapshot(now() + 2.)["image_jpeg"].is_null());
    assert!(s.on_image(CompressedImage::default()).is_err());
    let bytes = json!({"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"camera"},"format":"jpeg","data":vec![0;256001]});
    assert!(serde_json::from_value::<CompressedImage>(bytes).is_err());
}
#[test]
fn malformed_map_and_timestamp_are_rejected() {
    let mut s = ready();
    let mut m = map();
    m.data.pop();
    assert!(s.on_map(m).is_err());
    let mut m = map();
    m.data[0] = -2;
    assert!(s.on_map(m).is_err());
    assert!(
        Stamp {
            sec: 1,
            nanosec: 1_000_000_000
        }
        .seconds()
        .is_err()
    );
    assert!(Ros2State::new("../not a frame".into()).is_err());
}
