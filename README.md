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
.venv/bin/python -m pip install https://github.com/CBGPU-Engineering/CBGPU-robotics/releases/download/v0.1.0/cbgpu_robotics-0.1.0-py3-none-any.whl
.venv/bin/cbgpu-robotics demo
.venv/bin/cbgpu-robotics demo --execute-simulation
```

On Windows PowerShell:

```powershell
py -3 -m venv .venv
.venv\Scripts\python -m pip install https://github.com/CBGPU-Engineering/CBGPU-robotics/releases/download/v0.1.0/cbgpu_robotics-0.1.0-py3-none-any.whl
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
There is deliberately **no remote execution endpoint**.

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
