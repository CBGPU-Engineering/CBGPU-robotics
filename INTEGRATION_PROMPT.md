# Implement the complete CBGPU robot integration in this existing project

You are working on the robot's own development computer. Implement, test and document the integration in this repository; do not stop at proposing an architecture or writing a generic guide. Preserve unrelated code and credentials.

## Known target and constraints

The first target is a Raspberry Pi 5 with Ubuntu, one rotating 2D LiDAR, and a differential-drive car with two motors. An Arduino already drives the motors from serial commands, and the existing Arduino/serial code is in this project. There are NO wheel encoders and NO prebuilt SLAM map. Reuse the existing serial protocol. Do not guess motor pins, command strings, baud rate, wheel dimensions, LiDAR model or calibration. Inspect repository source and read-only device/OS information first, and ask only for facts you cannot determine. If this repository actually targets another OS or robot, keep the same CBGPU remote protocol and adapt the local driver rather than imposing this hardware profile.

The end result must let the operator open CBGPU on a different PC or phone, watch the SLAM map being built in real time, see the robot pose, click a known free point, and request navigation there. The robot must not require the operator to type each navigation command on the Pi. Keep local sensing, localization, path following, collision avoidance and stopping on the Pi. A map click goes directly through the authenticated remote channel to local navigation; it must not wait for an LLM or school GPU.

## Use the released SDK and inspect its real interfaces

Read https://github.com/CBGPU-Engineering/CBGPU-robotics, including cbgpu_robotics/remote.py, remote_protocol.py and ros2_nav2.py. Use version 0.2.0 or a later compatible release. Prefer the project's existing virtual environment. For ROS Python packages on Ubuntu, create a venv with the matching system Python and --system-site-packages so rclpy and ROS message packages remain visible; do not install a mismatched pip substitute for ROS.

Linux / Raspberry Pi example:
  python3 -m venv --system-site-packages .venv
  .venv/bin/python -m pip install https://github.com/CBGPU-Engineering/CBGPU-robotics/releases/download/v0.2.0/cbgpu_robotics-0.2.0-py3-none-any.whl
  .venv/bin/cbgpu-robotics demo --execute-simulation

Windows: use py -3 -m venv .venv and .venv\Scripts\python / cbgpu-robotics. macOS: use the Linux-style venv paths. The core SDK, simulator and HTTPS protocol are portable; the optional ROS2 adapter needs an installed compatible ROS2/Nav2 environment. Other platforms can implement the same authenticated HTTPS protocol.

## Connect the remote UI first with a harmless simulator

1. The operator signs into https://app.cbgpu.com/ai?tool=robotics and registers a device under the remote map section.
2. Run cbgpu-robotics connect locally and have the operator paste the one-time pairing code into its hidden input. Never ask them to put pairing codes, device tokens, API keys or passwords in AI chat, source code, shell arguments, screenshots or logs. Credentials are saved in the user's private ~/.cbgpu/robotics/devices/{device_id}.json.
3. Start cbgpu-robotics agent --device DEVICE_ID --simulator --allow-navigation. This is explicitly a virtual robot, with no hardware access. Verify the remote web page shows the growing virtual occupancy map, robot pose and simulation label.
4. In the web page, claim remote control, select a known free point and submit the goal. Verify receipt, changing pose, completion, stop/cancel, zoom/pan, and reconnect/replay handling. Do not label the simulation as real SLAM or real hardware.
5. Stop the simulator before starting a physical adapter with the same device credentials. Do not run duplicate agents.

Device authentication is separate from CBGPU AI API keys. The remote agent uses its paired device token; AI keys are needed only if adding natural-language planning. The agent makes outbound HTTPS requests, so do not open an unauthenticated inbound port or expose the Arduino/ROS network to the internet.

## Implement local LiDAR-only mapping with no saved map

Inspect /etc/os-release, the installed ROS distribution (if any), existing LiDAR driver, serial bridge, motor command timeout and stop behavior. Identify the LiDAR without moving the motors. Preserve functioning drivers.

Provide a local pipeline with LaserScan, LiDAR scan-matching odometry, the base/laser TF transforms, online SLAM, and a collision-aware navigation stack. There is no wheel odometry: never fabricate measured odometry from commanded motor speeds. RF2O's ROS2 branch is a candidate for laser odometry, but check compatibility with the actual ROS distribution and test it; do not assume an apt package or a branch works unmodified. Reuse another validated scan-matching implementation if already present.

Start at an arbitrary local origin with no map file. Build the occupancy map while moving through observed free space. One stationary 360-degree scan sees only line of sight, not an entire floor or through walls. First use supervised low-risk manual mapping in a controlled non-fire environment. If adding automatic exploration, use a bounded local free-space/frontier planner and stop when pose, scans or a safe path are unavailable. Unknown destinations such as a charging station cannot be treated as known waypoints before discovery/registration.

## Implement the physical remote adapter

Use the SDK's ROS2 adapter if the project already has the needed Nav2 interfaces, or implement a trusted project-local factory with these exact methods:
- robot_id: stable local identifier; commissioned defaults to False.
- safety() -> cbgpu_robotics.Safety: actual local arming, emergency stop, health and sensor capture timestamp. Never refresh timestamps for stale data.
- remote_snapshot(): {map, pose, ready, mode, message}, optionally image_jpeg bytes and image_at. Use real sensor/TF data, not invented pose values.
- validate_remote_goal(goal): reject unknown/occupied/unreachable or stale goals using the latest local map, pose and robot footprint.
- navigate_to(goal, stop_event): use the existing collision-aware navigation stack, cooperate with cancellation, and return True only after actual goal success.
- stop(): promptly cancel navigation and invoke the existing serial/motor stop path. It must work independently of the cloud and model.
- close(): release the local adapter's resources if needed.

Maps use the SDK's bounded occupancy-grid format: width, height, resolution in meters, origin {x,y,yaw}, frame, a mapping session ID, increasing version and RLE occupancy cells (-1 unknown, 0 free, 1..100 occupied). Row zero is the map's bottom row. Prefer remote.pack_grid() to conservatively downsample large maps; preserve frame/origin/orientation. Cache packed maps on updates instead of repacking large grids on every telemetry poll. Change the mapping session ID whenever SLAM resets or relocalizes to a different map, so old goals are cancelled. Check the real source for current limits.

Send map updates as SLAM publishes them, plus fresh robot pose and status. SLAM map transmission alone satisfies the remote image/map requirement; do not assume a camera exists. If the robot has a camera, optionally send bounded JPEG previews (max 256 KB, up to about 2 fps). Do not send raw camera buffers or full LiDAR scans to the web/API as images. The map UI uses real occupancy geometry for point selection, not pixel guesses from a screenshot.

For the optional SDK ROS2 adapter, configure the actual map/scan/base frame/navigation action topic names via its CBGPU_* environment variables. It subscribes to /map, /scan, TF and the NavigateToPose action by default. It also requires local /cbgpu/armed and /cbgpu/emergency_stop Bool state heartbeats; wire these to the project's actual local control/stop state, not unconditional auto-arm publishers. Missing or stale local state must disable navigation. CBGPU_ROBOT_COMMISSIONED=1 and --allow-navigation may be enabled only after explicit human commissioning, never automatically during setup.

## Physical control and failure handling

Keep the existing Arduino serial bridge as the last motor-control stage. Verify or implement an independent bounded command watchdog so stale commands cannot keep motors running. Check the actual firmware/protocol before any changes. Keep obstacle avoidance, local e-stop, scan/pose freshness, speed/geometry limits and safe stopping on the robot. A remote web stop is network-dependent and does not replace a physical/local stop.

The web operator explicitly claims a short control lease and sends a goal. The agent independently requires local commissioning/arming and --allow-navigation. It stops when remote control/connection expires, mapping resets, scans/pose become stale, navigation fails or cancellation arrives. Persistent local mission IDs prevent replay and concurrent controllers. Never auto-clear crash locks, auto-arm at boot, or expose arbitrary shell/serial/motor commands through the cloud. If a driver cannot stop cooperatively, keep the robot locked and require local recovery.

This is a prototype intended first for controlled non-fire tests. Indoor mapping success does not establish fire-scene readiness. Smoke, heat, terrain, drops, communication loss and hardware endurance need separate verification; a planar LiDAR alone does not establish stair/drop-off safety.

## Finish with working code and verification

Add tests with fake LiDAR/serial/Nav2 interfaces covering mapping without a map file, coordinate/origin/yaw conversion, occupied/unknown goals, stale scans/pose, lost internet, expired operator lease, duplicate goals, map reset, unreachable paths, cancellation and Arduino stop/watchdog behavior. Run the portable SDK simulator and the local project tests. Verify remote live map updates and map-click navigation in simulation before requesting physical motion.

Then provide exact commands for this repository to start its real LiDAR/odometry/online-SLAM/navigation stack and its agent. Example agent command (after the stack is ready):
  cbgpu-robotics agent --device DEVICE_ID --adapter my_robot:create
The initial invocation streams data only. After human commissioning/arming, use --allow-navigation. If using the SDK ROS2 adapter, select --adapter cbgpu_robotics.ros2_nav2:create.

Implement the code and launch/config files now, preserving existing unrelated functionality. Do not declare physical tests passed without actual hardware evidence. Finish with the tested commands, how to stop everything, how to view/click the map remotely, and only the remaining hardware facts/tests that could not be verified. Keep credentials out of the final report.
