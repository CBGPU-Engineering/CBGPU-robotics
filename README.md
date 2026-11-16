# CBGPU Robotics

[![Rust](https://img.shields.io/badge/Rust-1.88%2B-000000?logo=rust)](https://www.rust-lang.org/)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-D22128?logo=apache&logoColor=white)](./LICENSE)

<img src="./assets/cbgpu.png" width="120" alt="CBGPU">

CBGPU Robotics connects a local robot project to CBGPU's planning API and remote
map dashboard. The native Rust client runs on Linux, macOS and Windows. A local
adapter owns the robot's drivers, mapping, obstacle avoidance and emergency stop.
The server supplies bounded plans and click-to-navigate goals; it never supplies
executable code or raw motor commands.

The repository contains:

- **Planning client** — HTTPS, scoped API keys, asynchronous GPU-backed planning,
  strict capability schemas and a robot-bound, time-limited mission envelope.
- **Local controller** — dry-run by default, explicit approval, commissioning,
  fresh sensor checks, independent stop and persistent mission replay protection.
- **Remote agent** — outbound-only pairing, occupancy maps, optional JPEG images,
  telemetry, operator leases, goal acknowledgments and connection-loss stopping.
- **Simulator** — offline planning and online map transport without hardware.
- **Adapter interface** — a Rust trait or an explicit local executable using JSON.
  Existing ROS, Arduino and serial code stays in the robot project.

## Transition status

The Rust client is a source candidate. Existing Python 0.2 clients and source are
retained for compatibility with currently published onboarding and robot drivers.
No robot is switched automatically. Native Linux/macOS/Windows and Raspberry Pi
ARM64 release artifacts must be verified before onboarding changes or source
retirement. The two clients use the same remote API and replay-ledger schema,
but their local adapter ABIs differ. No new native binary or PyPI release is
published by this commit.

## Getting started

Build with Rust 1.88 or later and a C compiler for bundled SQLite:

```bash
cargo build --locked --release
cargo install --locked --path . --bin cbgpu-robotics
cbgpu-robotics demo
cbgpu-robotics demo --execute-simulation
```

The same Cargo commands work on Windows and macOS. No Python environment is
required. Linux ARM64 devices such as Raspberry Pi 5 can build locally using the
same commands. This source release does not publish a new PyPI package or promise
that an old wheel contains the native client.

The first demo only validates and prints a plan. The second runs the virtual robot.
Neither accesses serial ports, GPIO, ROS topics or motors.

## Plan a task

Set `CBGPU_API_KEY` in the local process environment using your OS secret settings.
Keep it out of command arguments, source files, screenshots and chat. Then run:

```bash
cbgpu-robotics plan "Move to the charging station"
cbgpu-robotics plan "Inspect the room" --adapter /absolute/path/to/my-robot-adapter
```

An adapter is a trusted local executable selected by the operator. The default is
the simulator. A real adapter requires both `--execute --approve` and its own
`commissioned: true` declaration before motion is possible. Local arming,
emergency stop, health and sensor freshness are still checked during execution.

The planning API remains `https://cbgpu.com/api/ai/v1`:

| Method | Path | Purpose |
| --- | --- | --- |
| POST | `/robotics/plans` | Queue a bounded plan |
| GET | `/robotics/plans/{id}` | Poll the caller's plan |
| DELETE | `/robotics/plans/{id}` | Cancel the caller's plan |

Use `--model cbgpu-coder` or `--model cbgpu-oss`, with `--effort low`, `medium`
or `high`. A mission is bound to the robot ID and canonical capability digest,
requires local approval, expires within 120 seconds and contains at most six
steps totaling 60 seconds. Each capability runs for at most 30 seconds. Unknown
capabilities, extra parameters, unbounded schemas and expired plans are rejected.
Schema and digest implementation is shared with the reviewed Rust server protocol.

## Remote maps and click navigation

1. Register a device in the CBGPU Robotics web page.
2. Run `cbgpu-robotics connect` and paste its one-time pairing code into the hidden
   prompt. Pairing is single-use and expires on the server.
3. Start a read-only simulator:

```bash
cbgpu-robotics agent --device DEVICE_ID --simulator
```

4. To test virtual navigation, explicitly add `--allow-navigation`.
5. For a physical robot, select its commissioned local adapter:

```bash
cbgpu-robotics agent --device DEVICE_ID --adapter /absolute/path/to/my-robot-adapter
```

The first physical invocation only uploads state. Enable navigation only after
local commissioning and human verification. The web operator must hold the
server's control lease; the local agent stops active navigation when the lease or
connection becomes stale. Stop epochs invalidate active work. Used command IDs
are never replayed after reconnect or process restart.

Remote endpoints and authentication remain compatible with the existing service:

| Authentication | Method / path | Purpose |
| --- | --- | --- |
| CBGPU browser session | `GET/POST /robotics/devices` | List/register owned devices |
| CBGPU browser session | `GET/DELETE /robotics/devices/{id}` | Inspect/revoke own device |
| CBGPU browser session | `POST /robotics/devices/{id}/pair` | Rotate pairing credentials |
| CBGPU browser session | `POST /robotics/devices/{id}/control` | Claim/renew/release lease |
| CBGPU browser session | `POST /robotics/devices/{id}/goal` | Submit map-bound goal |
| CBGPU browser session | `POST /robotics/devices/{id}/stop` | Cancel/invalidate control |
| Single-use pairing code | `POST /robotics/agent/enroll` | Enroll local device |
| Scoped device bearer | `POST /robotics/agent/sync` | Upload state, receive command |

Only outbound HTTPS is needed. HTTP is allowed only for explicit loopback tests;
redirects are rejected. Credentials are saved under `~/.cbgpu/robotics/devices`
with mode 600 on Unix and are never included in telemetry. On Windows keep this
folder in the private user profile with its normal user-only ACL.

Maps are RLE occupancy grids with at most 512×512 cells. Larger input maps are
conservatively downsampled: occupied or unknown cells never become free. Origin,
resolution, frame and mapping session are retained. Session changes stop an old
goal; unknown/occupied/outside-map goals are rejected locally. Changed maps are
uploaded once per session/version. JPEGs are limited to 256 KB and about 2 fps.
The adapter's JSON snapshot carries JPEG bytes as base64 `image_jpeg`.

The local connection watchdog is 1.5 seconds plus local sensor/stop processing.
Normal sync is approximately 250 ms; this is not a hard-real-time guarantee.
The cloud AI is never the collision avoidance or motor watchdog loop.

## Local adapters

Rust projects implement `controller::Adapter`. Other robot projects provide an
absolute executable selected with `--adapter`. The SDK invokes it as:

```text
/absolute/path/adapter --cbgpu-method METHOD
```

Each invocation receives one JSON request on stdin and returns:

```json
{"ok":true,"result":{}}
```

| Method | Input | Result |
| --- | --- | --- |
| `describe` | `{}` | `{robot_id, capabilities, commissioned}` |
| `observe` | `{}` | Current bounded planning observations |
| `safety` | `{}` | `{armed, emergency_stop, sensors_at, healthy}` |
| `validate_step` | `{capability, arguments, duration_s}` | Boolean precondition check |
| `execute` | `{capability, arguments}` | `true` when local operation completed |
| `stop` | `{}` | `true` only when stop was acknowledged locally |
| `remote_snapshot` | `{}` | `{map, pose, ready, mode, message, image_jpeg?, image_at?}` |
| `validate_remote_goal` | Goal object | Boolean local navigation check |
| `navigate_to` | Goal object | `true` only after reaching the goal |

The goal contains `x`, `y`, `yaw`, `frame`, `map_session` and `map_version`.
`map` uses `width`, `height`, `resolution`, `origin:{x,y,yaw}`, `frame`, `session`,
`version` and interleaved occupancy/count `rle` values.

The executable is a small client for an existing locally supervised driver. It
must preserve that driver's state across invocations, and `stop` must work while
`execute`/`navigate_to` is blocked. It must not invent fresh timestamps for stale
sensor data. API credentials and parent environment are **not inherited**; load
only the robot's explicit local configuration. Never require shell expansion.
Responses are bounded; calls have deadlines. A child is killed on cancellation,
and independent `stop` is invoked. Killing a client process does not guarantee
physical stopping: the driver must have its own local watchdog and emergency stop.

[`examples/adapter.rs`](./examples/adapter.rs) is a native, uncommissioned example
with no hardware access. It intentionally refuses execution/navigation. Build it
with `cargo build --bin cbgpu-robotics-adapter-example`.

### Existing ROS2 integrations

The former Python `module:factory` plugin ABI is not loaded by the native client.
Existing installations are unchanged; do not remove their working environment
before validating a native adapter against simulation. The previous bundled
`rclpy` helper is not silently replaced with guessed serial/motor code.

A ROS2 robot project supplies its existing LaserScan, scan-matching odometry,
base/laser transforms, online SLAM and Nav2 navigation through the adapter ABI.
Use the ROS distribution's supported Rust/C++ bindings or its existing local
supervisor. Match actual topics, QoS, frames and action names. No ROS distribution,
LiDAR model, motor pin, encoder or serial protocol is assumed by this SDK.
See [the integration prompt](./INTEGRATION_PROMPT.md) and
[the mapless car prompt](./MAPLESS_CAR_PROMPT.md).

## Crash recovery

The local SQLite ledger keeps the existing `missions` and `robot_locks` schema,
IDs and state. This is device-local replay protection, not the hosted PostgreSQL
database. Existing ledgers open without resetting old missions or locks.

If a driver fails to stop, its robot lock is retained. Verify the physical robot is
stopped before explicitly recovering:

```bash
cbgpu-robotics unlock ROBOT_ID --confirm-stopped
```

This removes only the lock; previous mission IDs remain blocked.

## Development and verification

```bash
cargo fmt --all -- --check
cargo test --locked
cargo clippy --locked --all-targets -- -D warnings
```

Tests use temporary ledgers, local mock HTTP servers and virtual motors only.
They cover planning transport, credentials, map rotation/downsampling, replay,
arming/estop/freshness, blocked drivers, connection loss, remote acknowledgments
and independent stops. CI runs on Linux, Windows and macOS; cross-platform CI
results must be checked before distributing binaries for those systems.

## License

Licensed under the [Apache License 2.0](./LICENSE).
