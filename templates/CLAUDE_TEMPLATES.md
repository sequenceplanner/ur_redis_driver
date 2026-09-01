# claude_* templates

Drop-in alternatives to the existing templates, plus the ones that were missing.
Nothing under `templates/` was modified - every file here is new and prefixed
`claude_`. They are picked up automatically: `main.rs` globs `templates/*.script`
into Tera and `command_server` accepts any template name as a `command_type`, so
`claude_safe_move_l` is usable the moment the driver restarts.

Every template renders and is block-balanced under two opposite request shapes
(Cartesian target / joint target, blend / no blend, payload / no payload,
waypoints / none, scaling 0.5 / 0.0). Function names and signatures were checked
against `docs/scriptmanualG5_.pdf`. **None of it has been run on a robot.**

## Bugs these fix

### 1. `force_threshold` defaults to 0.0, and that aborts every guarded move

`command_server` reads it with `get_float_or_default_to_zero`, so a caller who
does not set `<robot>_force_threshold` gets `0.0`. The guard in
`safe_move_j` / `safe_move_l` / `safe_move_l_relative` is

```
force = force()
if force > 0.0 or force < -0.0:
```

`force()` is a magnitude, so this is true on the first sample and the move aborts
before it starts. The `claude_` templates treat `<= 0.0` as "monitoring
disabled", say so on the socket, and run the move unguarded.

### 2. The force guard measured absolute force, not contact force

`force()` and `get_tcp_force()` include the weight of the tool and the payload,
so a UR20 with a 2 kg tool reads ~20 N with nothing touching it. Any threshold
below that trips immediately; any threshold above it is really "20 N plus your
number". The `claude_` templates zero the sensor (optional,
`use_zero_ftsensor`), sample a baseline wrench at standstill, and compare the
delta. A threshold now means newtons of contact.

They also wait `force_settle_time` (0.3 s) before arming, so the acceleration
transient at the start of the move does not trip the guard.

### 3. `set_payload({{ payload }})` is a runtime error

`Payload::to_string()` emits `mass,[cog],[inertia]` - three arguments. Per the
manual, `set_payload(m, cog)` takes two; `set_target_payload(m, cog, inertia)` is
the one that matches. `pick_vacuum.script` and `pick_vacuum_broken.script` call
`set_payload`. Everything else already uses `set_target_payload`.

### 4. Two templates fight over the same digital output

```
start_vacuum.script    set_digital_out(1, True)           -> standard DO 1
unlock_rsp.script      set_standard_digital_out(1, True)  -> standard DO 1
pick_vacuum.script     set_standard_digital_out(0, True)  -> standard DO 0
```

`set_digital_out` is only the deprecated alias of `set_standard_digital_out`, so
`start_vacuum` and `unlock_rsp` write the same physical pin: asking for vacuum
releases the RSP tool changer. `pick_vacuum` and `place_vacuum` meanwhile use
DO 0 for the vacuum.

**This needs checking against the cabinet - it is a wiring question, not a code
question.** The `claude_` templates default the vacuum to DO 0 (matching
pick/place, which were clearly written against the real cell) and the RSP to
DO 1, and make both a parameter so the answer lives in one place.

### 5. `pick_vacuum_broken.script` never stops the descent

Its force monitors switch the vacuum on and `break` out of their own loop, but
neither ever sets `force_detected`. The wait below them is
`while not move_done and not force_detected`, so nothing stops the move - the arm
keeps pressing the cup into the part for the rest of the 0.2 m stroke.

### 6. `global_velocity_scaling` / `global_acceleration_scaling` were never used

Both are decoded from Redis for the request and for every waypoint, and no
template referenced either. The `claude_` templates apply them, treating a value
outside `(0.0, 1.0]` as "no scaling" so the `0.0` default does not freeze the
arm.

### 7. Waypoint progress reporting breaks the blend it is reporting on

`trajectory_unsafe_move_*.script` emit `socket_send_line("waypoint_reached n")`
*between* two blended moves - the comment on `report_waypoint_progress` in
`RobotCommand` already admits this may flush the look-ahead buffer, which is why
the flag exists. That makes the trade "smooth path" versus "resumable path".

The `claude_` versions keep the moves as one uninterrupted block and watch the
arm from a separate thread, reporting each waypoint when the TCP comes within a
tolerance derived from that waypoint's own blend radius (blending cuts the
corner, so the tolerance has to be at least the blend radius). Both properties at
once. The emitted line and index are unchanged, so `socket_server`'s
`WAYPOINT_REACHED_PREFIX` handling needs no change.

### 8. A relative waypoint drives the tool into the base

`WaypointRaw` decodes `use_relative_pose`, but `Waypoint` - the struct that
reaches the template - has no such field. When a waypoint sets it, `command_server`
skips the transform lookup (`if !wpr.use_joint_positions && !wpr.use_relative_pose`)
and `target_in_base` stays the identity pose it was initialised to. The template
cannot tell, and moves to `p[0,0,0,0,0,0]` - the middle of the robot's own base.

**The real fix is in Rust** (see below). Meanwhile the `claude_` trajectory
templates refuse any waypoint whose target is within 1 mm of the base origin,
which is never a legitimate TCP target.

### 9. Smaller things

- `force = force()` shadows the builtin `force()` for the rest of the scope.
  Renamed everywhere.
- `force < -threshold` is dead code; `force()` is never negative.
- `safe_move_l.script` sets the payload twice, once outside the move thread and
  once inside it, and calls `set_tcp()` from inside the thread while the force
  monitor is already sampling through that TCP.
- `stopj()` was used to stop linear moves; `stopl()` keeps the tool on its line
  while braking.
- IK was solved twice - once to test, once to move. `get_inverse_kin` is not
  guaranteed to return the same branch on two calls. Solved once now.
- `set_tcp()` came after `get_inverse_kin()` in places, so IK solved for the
  wrong tool.
- A force stop returned `False` with no reason, indistinguishable in Redis from
  a dashboard stop. It now reports peak force and threshold first.
- `unsafe_move_j.script` has an unreachable `return True` after its if/else.
- Pre-move checks now also use `is_within_safety_limits()`, which considers
  safety planes, joint limits and the tool orientation limit -
  `get_inverse_kin_has_solution()` only answers "is it physically reachable".
- `is_steady()` needs 500 ms of standstill, so checking it once fails a script
  that starts right after the previous one finished braking. The templates that
  need standstill wait for it instead (`steady_timeout`, 1.0 s).

## New templates

| Template | What it is for |
| --- | --- |
| `claude_trajectory_safe_move_j` / `_l` | A blended trajectory with a force guard. There was none: a motion loses its guard the moment it is expressed as a waypoint list, which is the long unattended motion where a guard matters most. Waypoint progress makes a force stop resumable. |
| `claude_move_until_contact` | Contact as the *goal*, not a failure. Finds the top of a stack, seats a part, measures a workpiece. Reports the contact pose. |
| `claude_force_mode_push` | Holds a controlled force along tool Z. Everything force-related today is a guard on a position-controlled move; this makes force the objective. |
| `claude_spiral_search` | Compliant insertion search - absorbs the few millimetres a vision pick or a fixture stack-up leaves. **Try this in URSim first.** |
| `claude_check_reachable` | Dry run. Answers "can the arm get there" without sending the arm at it, so a sequence can be validated before it is committed. |
| `claude_wait_digital_in` | Waits on the cell - a clamp, a conveyor, a vacuum switch - with a timeout, instead of polling from Redis or sleeping a guessed duration. |
| `claude_set_digital_out` | The general form of start/stop vacuum and lock/unlock RSP. A new actuator no longer needs a new template. |
| `claude_move_j_relative` | Joint-space nudge. Unwinds a wrapped wrist or steps a joint clear of a singularity - both things Cartesian relative motion cannot express, and both the cause of the IK failure that put the arm there. |
| `claude_home` | Goes to `SAFE_HOME_JOINT_STATE`, which the driver already falls back to but has no command for. Caps speed, because it is the command used when something has already gone wrong. |
| `claude_freedrive` | Bounded freedrive, so an operator can position the arm for a teach-in. The pendant's freedrive button is not available in remote control. |
| `claude_set_tcp` | Establishes a TCP that outlives one script, and echoes back the pose the driver looked up. |
| `claude_zero_ftsensor` | Tares the F/T sensor once after a tool change, rather than on every move. |
| `claude_stop` | A deliberate, controlled deceleration. Not a replacement for the dashboard `stop` that `handle_request` uses to cancel - that one is out of band and works on a wedged script. |
| `claude_get_state` | One-line snapshot taken at a known point in a sequence, including per-pin digital inputs. |

## Optional parameters

Several templates accept parameters that `RobotCommand` does not carry yet. They
use Tera's `default` filter, so today they render with the default shown in each
file's header and behave exactly as described. To make one settable:

1. add the field to `RobotCommand` in `src/core/structs.rs`,
2. add its suffix to the `suffixes` array in `src/interfaces/command_server.rs`,
3. read it next to the others and put it in the `RobotCommand` literal,
4. seed the key in `generate_robot_interface_state`.

Note step 4 is not optional: `command_server` fails any request whose full key
set is not present in Redis.

The ones worth wiring first are `vacuum_pin` / `rsp_pin` (they encode the
question in bug 4), `vacuum_ok_pin` (turns a vacuum switch into a real pick
verification), and `dout_pin` / `din_pin` (without them
`claude_set_digital_out` and `claude_wait_digital_in` can only reach pin 0).

## Rust-side fixes these templates cannot make

- **`Waypoint` is missing `use_relative_pose`** (bug 8). `WaypointRaw` decodes
  it and it is dropped on the way to `Waypoint`, so a relative waypoint silently
  becomes a move to the base origin. Add the field, carry it through the
  conversion in `command_server`, and the trajectory templates can render a
  `pose_trans(get_forward_kin(), p[...])` for it. The template-level guard here
  turns the crash into a refused request, which is the right failure but not the
  right feature.
- **Feedback lines overwrite each other.** Every line a script sends lands in
  `<robot>_request_feedback`, so a multi-line report is a race against the
  reader. That is why each `claude_` template sends its result as one line. If
  richer reporting is wanted, `publish_script_feedback` would need to append or
  to route by prefix the way `waypoint_reached` already is.
