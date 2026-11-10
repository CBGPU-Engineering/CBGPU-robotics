# CBGPU Robotics

[![CI](https://github.com/CBGPU-Engineering/CBGPU-robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/CBGPU-Engineering/CBGPU-robotics/actions/workflows/ci.yml)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Platforms](https://img.shields.io/badge/Platforms-Windows%20%7C%20macOS%20%7C%20Linux-555)](#platforms)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-D22128)](./LICENSE)

<img src="./assets/cbgpu.png" width="96" alt="CBGPU">

CBGPU Robotics turns natural-language instructions into bounded plans for a robot's
existing capabilities. School GPUs generate the plan; a **local adapter** validates
and executes it using the robot's own drivers. The server never runs motor commands,
connects to the robot, or receives a remote shell.

The same interface works for a mobile robot, an arm, a gripper, a camera or another
device. Each adapter exposes the capabilities its actual hardware implements.
This is a task planner using CBGPU's text models. It is not a pretrained universal
vision-action model, and does not infer objects or joint trajectories from images.

Try the [web workspace](https://cbgpu.com/ai?tool=robotics) or give your robot's coding
assistant the [integration prompt](./INTEGRATION_PROMPT.md).

## Platforms

- **Windows, macOS, Linux and Raspberry Pi / ARM:** Python 3.10+ SDK and CLI;
  no third-party runtime dependencies. CI runs Python 3.10, 3.12 and 3.14 on all
  three desktop operating systems. Hardware-driver availability depends on the adapter.
- **Android, iOS and desktop browsers:** the web workspace can request and inspect plans.
  A browser does not directly control motors.
- **Other languages and operating systems:** use the same HTTPS JSON API. The host needs
  an HTTPS client, JSON support and a locally implemented adapter. The Python wheel is
  portable; it does not make Python or a hardware driver available on every OS or RTOS.

No GPU is required on the robot. No inbound port or public robot IP is required.

## Getting started

### Offline demonstration

On Linux, macOS or Raspberry Pi:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install https://github.com/CBGPU-Engineering/CBGPU-robotics/releases/download/v0.2.0/cbgpu_robotics-0.2.0-py3-none-any.whl
.venv/bin/cbgpu-robotics demo
.venv/bin/cbgpu-robotics demo --execute-simulation
```

On Windows PowerShell:

```powershell
py -3 -m venv .venv
.venv\Scripts\python -m pip install https://github.com/CBGPU-Engineering/CBGPU-robotics/releases/download/v0.2.0/cbgpu_robotics-0.2.0-py3-none-any.whl
.venv\Scripts\cbgpu-robotics demo
.venv\Scripts\cbgpu-robotics demo --execute-simulation
```

The first command validates and prints a dry-run plan. The second executes it in
`VirtualRobot`, which has **no hardware access**. Neither needs a network connection
or API key after installation. Activate the environment, or use the full executable
path shown above, for the remaining commands.

### Plan with a school GPU

Create a private AI API key at [AI → API](https://cbgpu.com/ai?tool=api).
Set `CBGPU_API_KEY` in your local environment or credential manager. Keep it out of
source control, command arguments, URLs and logs.

```bash
cbgpu-robotics plan "go to the charging station"
```

This requests a real model plan for the virtual robot, then prints a dry-run.
The default model is `cbgpu-coder`; `--model cbgpu-oss` selects GPT-OSS 20B.
`--effort low|medium|high` selects effort. School reservations take priority:
plans can queue, pause, fail or expire before an unreserved GPU becomes available.
Capture fresh observations and make a new request after expiry.

### Connect your robot

Implement the interface in [examples/adapter_template.py](./examples/adapter_template.py).
The SDK loads an explicitly selected, trusted local Python factory:

```bash
cbgpu-robotics plan "go to the charging station" --adapter my_robot:create
```

The default remains **dry-run**. Commission the adapter, verify its stop/watchdog
and sensor handling on real hardware, and locally arm the robot before using:

```bash
cbgpu-robotics plan "go to the charging station" --adapter my_robot:create --execute --approve
```

`--approve` is an explicit local approval for this invocation, not a remote server
permission. Review each mission and use a suitable controlled test area. The
integration prompt helps a coding assistant inspect your existing project and
write an adapter; it cannot discover unknown wiring or verify physical safety.

## Adapter contract

| Member | Purpose |
| --- | --- |
| `robot_id` | Stable local robot identifier |
| `capabilities` | Named operations with closed bounded parameter schemas and deadlines |
| `commissioned` | Remains false until the human validates the real adapter |
| `observe()` | Real structured observations for a new planning request |
| `safety()` | `Safety(armed, emergency_stop, sensors_at, healthy)` from local hardware |
| `validate_step(step)` | Check current local preconditions; return exactly `True` to accept |
| `execute(name, arguments, stop_event)` | Call a known local driver and cooperate with cancellation |
| `stop()` | Promptly stop the hardware; must work even if a driver call is blocked |

Use known waypoint or object IDs and finite ranges. Parameter schemas support
`object`, `array`, `string`, `number`, `integer`, `boolean`, enum, required fields
and explicit bounds. Objects must set `additionalProperties: false`; arrays need
`maxItems <= 16`; strings need `maxLength <= 256`; numbers need finite minimum and
maximum. Unknown keywords and unsupported calls are rejected.

For navigation, expose a high-level operation backed by the robot's existing
navigation and obstacle-avoidance stack. For an arm, keep joint, force and collision
limits in its driver. Local perception must resolve object IDs before planning.
Do not expose arbitrary code, shell commands or unbounded actuator values.

## Planning API

Base: `https://cbgpu.com/api/ai/v1`

All calls require `Authorization: Bearer YOUR_CBGPU_AI_KEY` (or a current approved
CBGPU browser session). Records are private to the account that created them.
Planning endpoints do not actuate hardware. Separately paired remote navigation is described below.

### Create a plan

`POST /robotics/plans` returns HTTP 202 with `{id, state, protocol_version}`.

```json
{
  "robot_id": "my-robot",
  "instruction": "Go to the charging station",
  "model": "cbgpu-coder",
  "effort": "medium",
  "observed_at": 0,
  "observations": {"location": "desk"},
  "capabilities": [{
    "name": "navigate_to",
    "description": "Navigate to a known waypoint using local obstacle avoidance",
    "max_duration_s": 10,
    "parameters": {
      "type": "object",
      "properties": {"waypoint": {"type": "string", "enum": ["desk", "charging_station"], "maxLength": 32}},
      "required": ["waypoint"],
      "additionalProperties": false
    }
  }]
}
```

Replace `observed_at: 0` with the current Unix timestamp in seconds, captured with
fresh observations. Requests older than 30 seconds or over five seconds in the
future are rejected; synchronize the robot's clock. Instructions are limited to
2,000 characters, capability definitions to 14 KB and observations to 12 KB.

### Read or cancel

- `GET /robotics/plans/{id}` returns state and progress, then a validated `mission`
  on success, or an explicit error. Poll about once per second.
- `DELETE /robotics/plans/{id}` cancels queued/running planning. It is not a motor
  emergency stop, and does not stop a robot that has already begun local execution.

A mission binds its ID, robot ID and capability digest to a plan, observation and
issuance timestamps, an expiry and `requires_local_approval: true`. A plan contains:

```json
{
  "summary": "Move to the charging station",
  "needs_clarification": false,
  "question": "",
  "steps": [{"capability": "navigate_to", "arguments": {"waypoint": "charging_station"}, "duration_s": 10}]
}
```

If more information is needed, `needs_clarification` is true, `question` explains
what is missing and `steps` is empty. The SDK refuses to execute that response.

## Execution and recovery

The local controller independently revalidates the entire mission and every step.
It checks robot ID, capability digest, expiry, known arguments and current local
preconditions, and requires locally armed, healthy state with sensor timestamps
no more than one second old. It checks safety about every 100 ms while a step runs.
A step deadline, expired mission, stale sensor, e-stop, failed driver or keyboard
interrupt requests cancellation and calls the adapter's `stop()`.

Plans have at most six steps and 60 seconds total, with no individual deadline
over 30 seconds. They expire no later than 120 seconds after their observations.
The persistent SQLite ledger prevents replay and concurrent missions for the same
robot across controller instances/processes sharing that ledger. It reserves the
mission before the first actuator call.

A Python thread cannot forcibly stop a noncooperative driver or guarantee physical
stopping. The adapter needs an independent local motor watchdog and emergency stop.
If a worker stays alive after cancellation, its robot lock remains held. A process
crash also leaves its lock held. After physically stopping the robot and terminating
old control processes, recover explicitly:

```bash
cbgpu-robotics unlock my-robot --confirm-stopped
```

Used mission IDs remain blocked. Do not automatically clear locks or auto-arm on boot.
Separate local ledger files cannot coordinate one shared physical robot; use one
controller/ledger and enforce exclusive ownership in the driver.

The SDK stores the ledger in `~/.cbgpu/robotics/missions.sqlite`. On Unix it sets
owner-only file permissions. Windows users should keep the containing user profile
private and use OS credential storage or environment variables for API keys.

## Development

```bash
python -m pip install -e .
python -m unittest discover -s tests -v
python -m cbgpu_robotics.cli demo --execute-simulation
```

Tests use virtual hardware and loopback HTTP. They cover default dry-run, explicit
approval, replay, wrong-robot/schema binding, bounds, stale sensors, e-stop,
timeouts, noncooperative driver locks and authenticated asynchronous transport.
Real robot operation requires separate hardware commissioning; CI cannot verify it.

## Project structure

```text
cbgpu_robotics/
  client.py       HTTPS planning client and async polling
  protocol.py     Shared schema/mission validation (also vendored by the backend)
  controller.py   Local safety checks, deadlines, stopping and persistent ledger
  simulator.py    Offline virtual navigation/gripper/speech adapter
  cli.py          Cross-platform commands
examples/         Adapter starting point, initially uncommissioned
INTEGRATION_PROMPT.md   Copyable instructions for the robot's coding assistant
tests/            Synthetic transport and safety tests
.github/workflows/     Windows/macOS/Linux checks and portable wheel build
```

Original code is licensed under [Apache-2.0](./LICENSE). Hardware drivers and
model weights retain their own licenses.

## Mapless LiDAR car profile

For a Raspberry Pi 5 with Ubuntu, one rotating LiDAR and two motors driven by
existing Arduino serial commands, use [MAPLESS_CAR_PROMPT.md](./MAPLESS_CAR_PROMPT.md).
It explicitly assumes **no encoders and no saved map**. Keep the existing serial
protocol; identify the actual LiDAR and ROS/OS versions before selecting drivers.

The local pipeline needs LiDAR scan-matching odometry, online SLAM and navigation.
Map the visible space while moving through it; one stationary rotation does not
map rooms hidden by walls. Map-click goals belong to the local navigation stack,
not the cloud model's inference loop. This profile is integration guidance for the
robot's existing project, not an included preconfigured Arduino/LiDAR driver.

Relevant upstream references:
[Nav2 sensor setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/sensors/setup_sensors/),
[RF2O ROS2 source](https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros2),
[Nav2 goal selection](https://docs.nav2.org/rolling/getting_started/quickstart/quickstart/).
Check compatibility with the installed ROS distribution rather than assuming a
specific package/version works. Real fire-scene operation needs separate physical
validation; success in an indoor simulator does not establish that capability.

## Remote SLAM map and point navigation (0.2.0)

Open [Robotics](https://app.cbgpu.com/ai?tool=robotics) on your PC or phone. Register
a device in the remote map section. On the robot computer, run `cbgpu-robotics connect`
and paste the one-time pairing code into the hidden prompt. The code expires in
10 minutes and can be used once; never put it or the resulting device token in chat,
source, shell arguments or logs. It is independent of an AI API key.

First verify the entire remote channel using the harmless simulator:

```bash
cbgpu-robotics agent --device DEVICE_ID --simulator --allow-navigation
```

The web shows a **virtual** map progressively becoming known, robot pose and command
status. Claim remote control, select an observed free cell and send the goal.
Use cancel/stop to test interruption. The simulator has no GPIO, serial or hardware
access, and is not evidence of physical SLAM correctness.

For the real robot, start its existing LiDAR/odometry/online-SLAM/Nav2 stack, then:

```bash
# Stream maps and pose only
cbgpu-robotics agent --device DEVICE_ID --adapter cbgpu_robotics.ros2_nav2:create

# After human commissioning and local arming, permit remote navigation
cbgpu-robotics agent --device DEVICE_ID --adapter my_robot:create --allow-navigation
```

The optional `ros2_nav2` adapter needs the matching system ROS Python packages
(`rclpy`, Nav2/TF and message packages), not third-party pip substitutes. On Ubuntu
use a matching Python venv with `--system-site-packages`. Its topic defaults are
`/map`, `/scan`, `/navigate_to_pose`, base frame `base_link`, `/cbgpu/armed` and
`/cbgpu/emergency_stop`. `CBGPU_MAP_TOPIC`, `CBGPU_SCAN_TOPIC`, `CBGPU_NAV_ACTION`,
`CBGPU_BASE_FRAME`, `CBGPU_ARM_TOPIC`, `CBGPU_ESTOP_TOPIC`, `CBGPU_ROBOT_ID` and the
optional JPEG `CBGPU_IMAGE_TOPIC` configure these. Local arm/e-stop status must have
fresh periodic heartbeats. `CBGPU_ROBOT_COMMISSIONED=1` is a human commissioning
switch, not a setup default. Keep all actual motor limits, collision avoidance,
Arduino command watchdog and emergency-stop handling in the existing robot project.
The adapter cancels its own Nav2 goal; a custom adapter must also invoke the project's
serial stop path where needed.

Custom adapters implement `remote_snapshot()`, `safety()`, `validate_remote_goal()`,
`navigate_to(goal, stop_event)` and `stop()`, plus an optional `close()`. Return a
current occupancy map and pose. `remote.pack_grid()` conservatively downsamples maps
to at most 512×512 cells while preserving metric origin/frame/yaw; occupied/unknown
cells are never converted to free cells. Cache packed maps between SLAM updates.
Optional camera previews are JPEG bytes up to 256 KB, sent at most about 2 fps.
The protocol transports real grid geometry rather than guessing coordinates from a
screenshot. A new mapping session invalidates previous goals.

The agent makes outbound HTTPS requests only. It checks local safety during motion,
requires a live remote operator lease, stops after a lost connection and never
replays an already reserved mission ID. Browser control leases last 10 seconds and
renew every 3 seconds; loss of the operator can therefore take up to the lease
window to be detected. The agent's own connection watchdog uses 1.5 seconds plus
local processing/driver stopping time. These are software bounds, not hard real-time
or physical braking guarantees. A web stop may be delayed by the network and cannot
replace local emergency stop or an independent Arduino motor watchdog.

The web polls maps/pose approximately once per second and displays request-to-device
acknowledgement time when available. This is not a guarantee of 0.2-second internet
latency. Real latency depends on network, map size and robot processing. Cloud AI
inference is not involved in point navigation.

Management API (CBGPU browser JWT only):

- `GET/POST /robotics/devices`: list/register own devices.
- `GET/DELETE /robotics/devices/{id}`: inspect/revoke own device and stored preview.
- `POST .../{id}/pair`: revoke old token and issue a new one-time pairing code.
- `POST .../{id}/control`: claim/renew a browser lease (`lease` UUID), or release it.
- `POST .../{id}/goal`: request `{lease,x,y,yaw,map_session,map_version}`.
- `POST .../{id}/stop`: cancel navigation and invalidate operator control.

Agent API (paired device token in the Authorization header):

- `POST /robotics/agent/enroll`: exchange `{pair_code,agent_session}` once.
- `POST /robotics/agent/sync`: upload state/map/optional JPEG and get a pending goal,
  operator lease state and stop epoch; acknowledge commands on subsequent syncs.

See [INTEGRATION_PROMPT.md](./INTEGRATION_PROMPT.md) for the single complete prompt
covering existing Arduino code, LiDAR-only/no-map SLAM, remote maps/images, navigation,
stopping, simulation and real-robot commissioning.
