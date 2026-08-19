# ur_redis_driver

A Universal Robots driver with no ROS interfaces: the robot is driven entirely
through Redis. Merges the older `ur_script_driver` and `r2r_ur_controller`.

Motion commands are Tera-templated URScript injected over the realtime interface;
robot state is decoded from the same stream and published to Redis; and the UR
Dashboard Server provides program and safety control (stop, pause, power,
protective-stop recovery).

## Running

```bash
ROBOT_ID=r1 ROBOT_MODEL=ur5e UR_ADDRESS=192.168.1.10 cargo run
```

| Env var | Default | Meaning |
|---|---|---|
| `ROBOT_ID` | `r1` | Prefix for every Redis key this driver reads and writes |
| `ROBOT_MODEL` | `ur20` | Picks `<UR_DESCRIPTION_DIR>/urdf/<model>.urdf` and the mesh directory |
| `UR_DESCRIPTION_DIR` | `src/ur_description/` | URDF, meshes and per-model config |
| `UR_ADDRESS` | `0.0.0.0` | Robot IP. Ports are appended, not configurable |
| `REDIS_HOST` / `REDIS_PORT` | `127.0.0.1` / `6379` | Read by `micro_sp` |

`templates/` is resolved **relative to the working directory**, so the process must
be started from the repository root. A missing or unparsable template set is fatal.

Ports used:

| Port | Direction | Purpose |
|---|---|---|
| 30003 | out, long-lived | Realtime stream, read at 125 Hz |
| 30003 | out, one per goal | URScript is written here |
| 29999 | out, long-lived | Dashboard Server |
| 50000 | **in** (listener on this host) | The running URScript connects back for handshake, feedback and result |

The address the robot dials back on is auto-detected with `local_ip()`. On a
multi-homed host this picks one interface with no way to override it.

## Values in Redis

Every key is a JSON-serialised `micro_sp` `SPValue`, not a bare scalar. A string is
`{"type":"String","value":{"String":"move_j"}}`, a bool is
`{"type":"Bool","value":{"Bool":true}}`. Writing a bare `true` will fail the
request that reads it (and say which key was unreadable).

All keys below are prefixed with `ROBOT_ID`, shown here as `r1_`.

## Motion requests

Write the parameters, then set the trigger:

```
r1_command_type          the template to run, without ".script"
r1_request_state         set to "initial" before triggering
r1_request_trigger       set true to submit
```

`request_state` then walks `initial → executing → succeeded | failed`, and
`r1_request_result` carries a human-readable reason. **A request always reaches a
terminal state** — a rejected request fails rather than sitting at `initial`.

Set `r1_request_cancel` true to abort the running goal. Cancellation issues a
dashboard `stop`.

| Key | Type | Meaning |
|---|---|---|
| `r1_acceleration` / `r1_velocity` | float | `move_j`: leading-axis rad/s², rad/s. `move_l`: m/s², m/s |
| `r1_global_acceleration_scaling` / `r1_global_velocity_scaling` | float | 0.0–1.0 |
| `r1_use_execution_time` / `r1_execution_time` | bool / float | Fixed motion duration; takes priority over speed and acceleration |
| `r1_use_blend_radius` / `r1_blend_radius` | bool / float | Blend through the point instead of stopping at it |
| `r1_use_joint_positions` / `r1_joint_positions` | bool / array[6] | Move to joint angles directly, skipping IK and the TF lookup |
| `r1_use_preferred_joint_config` / `r1_preferred_joint_config` | bool / array[6] | IK seed |
| `r1_use_relative_pose` / `r1_relative_pose` | bool / array[6] | Pose relative to the current TCP |
| `r1_use_payload` / `r1_payload` | bool / string | `mass,[cogx,cogy,cogz],[ixx,iyy,izz,ixy,ixz,iyz]` |
| `r1_baseframe_id` | string | Default `base_link` |
| `r1_faceplate_id` | string | Default `tool0` |
| `r1_goal_feature_id` | string | Target frame; looked up in TF against `baseframe_id` |
| `r1_tcp_id` | string | TCP frame; looked up against `faceplate_id` |
| `r1_force_threshold` | float | Force guard for the `safe_*` templates |
| `r1_waypoints` | array of maps | Blended trajectory, see below |
| `r1_request_feedback` | string | Latest line the running script sent back |
| `r1_total_fail_counter` | int | Cumulative failures, never reset |
| `r1_subsequent_fail_counter` | int | Consecutive failures, reset to 0 on success |

Available `command_type` values are exactly the filenames in `templates/`:
`safe_move_j`, `safe_move_l`, `safe_move_l_relative`, `unsafe_move_j`,
`unsafe_move_l`, `unsafe_move_l_relative`, `trajectory_unsafe_move_j`,
`trajectory_unsafe_move_l`, `pick_vacuum`, `place_vacuum`, `start_vacuum`,
`stop_vacuum`, `lock_rsp`, `unlock_rsp`, `set_payload`, `get_force`. An
unrecognised name fails the request. Adding a template adds a command; any field
it references must exist on `RobotCommand` in `src/core/structs.rs`.

`r1_waypoints` is an array of maps carrying the same per-point fields as above,
plus `use_relative_pose`, `baseframe_id`, `faceplate_id`, `goal_feature_id`,
`tcp_id` and `root_frame_id`. A waypoint that fails to decode fails the whole
request — a blended trajectory with a point missing is a different path, not a
shorter one.

Only one goal runs at a time; a second request is rejected while one is active.

## Dashboard requests

Same handshake, on its own keys, so a slow dashboard command never stalls motion:

```
r1_dashboard_command          command name
r1_dashboard_command_arg      argument, for the commands that take one
r1_dashboard_request_state    set to "initial" before triggering
r1_dashboard_request_trigger  set true to submit
r1_dashboard_request_result   the controller's reply
```

| Command | Arg | Notes |
|---|---|---|
| `stop`, `pause`, `play` | | See the caveat below |
| `power_on`, `power_off`, `brake_release` | | |
| `unlock_protective_stop` | | Closes the safety popup, waits out the settle, unlocks, then confirms against the realtime safety mode. Alias: `reset_protective_stop` |
| `close_safety_popup`, `close_popup`, `restart_safety` | | |
| `load`, `load_installation` | filename | |
| `popup`, `add_to_log` | text | |
| `shutdown` | | Powers down the controller |
| `safety_status`, `robot_mode`, `program_state`, `is_program_running`, `is_in_remote_control`, `get_loaded_program`, `get_robot_model`, `polyscope_version` | | Queries; the reply lands in `r1_dashboard_request_result` |

Most action commands require the robot to be in **Remote Control**. In Local mode
the controller answers `Failed to execute: <command>` and the request fails with
that text.

**Pause/play caveat.** This driver injects scripts over port 30003 rather than
loading a `.urp`, so `play` does not resume an injected script that `pause`
suspended — it tries to start the *loaded pendant program*. `stop` does reliably
kill an injected script. Treat `pause` as a hold you release with `stop` followed
by re-issuing the motion request.

## Published state

Written by the state publisher, guarded so a stationary robot does not rewrite
unchanged values.

| Key | Type | Source |
|---|---|---|
| `r1_joint_states` | array[6] | Realtime, packet 252 |
| `r1_tcp_pose` | array[6] | Realtime, packet 444 — `[x,y,z,rx,ry,rz]` |
| `r1_tcp_force` | array[6] | Realtime, packet 540 |
| `r1_force_feedback` | float | Magnitude of the translational part of `tcp_force` |
| `r1_safety_mode` | string | Realtime, packet 812 — `NORMAL`, `REDUCED`, `PROTECTIVE_STOP`, `RECOVERY`, `SAFEGUARD_STOP`, `SYSTEM_EMERGENCY_STOP`, `ROBOT_EMERGENCY_STOP`, `VIOLATION`, `FAULT` |
| `r1_robot_mode` | string | Realtime, packet 756 — `POWER_OFF`, `IDLE`, `RUNNING`, … |
| `r1_speed_scaling` | float | Realtime, packet 940. Reads 0.0 when no program is running |
| `r1_digital_inputs` / `r1_digital_outputs` | int | Realtime — bitmasks |
| `r1_program_state` | string | **Dashboard**, `programState` |
| `r1_program_running` | bool | **Dashboard**, `running` |
| `r1_remote_control` | bool | **Dashboard**, refreshed every 2 s |
| `r1_robot_connected` / `r1_dashboard_connected` | bool | Socket liveness |

TF frames go to `tf:<child_frame_id>`: `base_link_inertia`, `shoulder_link`,
`upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`,
`flange`, `ft_frame`, `tool0`, plus a `<link>_visual` frame per mesh. The driver
**owns** these frames and reasserts its own parent every tick, so an external
`reparent_transform` of one of them will not survive.

Two notes on the realtime stream. `r1_program_state` deliberately does not come
from realtime packet 1052: despite being documented as "Program state" it does not
carry the stopped/playing/paused enum (on PolyScope 5.25 it reads 1 with nothing
running and 4 with an interface script live). And the dashboard reports
`programState` for the *loaded pendant program*, so it stays `STOPPED` while an
injected script runs — `r1_program_running` is the signal that tracks this
driver's own scripts.

## Behaviour under failure

- Losing the robot does not stop the driver. The realtime and dashboard sockets
  reconnect on their own, `r1_robot_connected` / `r1_dashboard_connected` go false,
  and work resumes when the controller comes back — no restart needed.
- A safety stop (`PROTECTIVE_STOP`, `SAFEGUARD_STOP`, either emergency stop,
  `VIOLATION`, `FAULT`) fails the active goal. `REDUCED` does **not** — it is a
  normal operating mode inside a reduced-speed zone.
- A malformed value in Redis fails the one request that reads it, naming the key.
- A script killed out from under the driver (dashboard `stop`, an abort on the
  pendant) fails its goal rather than leaving the driver unable to accept new work.

## Known gaps

- No goal timeout. A script that neither finishes nor closes its socket — a robot
  yanked off the network mid-move — leaves the goal live and blocks later requests.
- Realtime parsing uses fixed offsets and requires a 1220-byte frame, so it is tied
  to this PolyScope generation. RTDE (port 30004) would be version-independent and
  would also allow *writing* speed scaling and digital outputs.
- No tests.
- The gripper interface (`generate_gripper_interface_state`) is written but not
  wired up.
- Digital outputs cannot be set from Redis.
