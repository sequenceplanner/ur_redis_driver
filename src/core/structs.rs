use std::sync::{Mutex, MutexGuard};
use tokio::sync::{mpsc, oneshot};

use micro_sp::*;
use serde::{Deserialize, Serialize};

use crate::core::dashboard_types::{DashboardIdentity, RobotMode, SafetyMode};


/// One accepted motion goal, as handed to `handle_request`.
pub struct ScriptRequest {
    pub uuid: String,
    /// The rendered script, ready to write to port 30003.
    pub script: String,
    /// The command it was rendered from, retained so a resume can re-render the
    /// remainder of the motion instead of replaying it from the start.
    pub command: RobotCommand,
}

pub struct DriverState {
    pub running: bool,
    /// The realtime (30003) stream is up.
    pub connected: bool,
    /// The dashboard (29999) socket is up.
    pub dashboard_connected: bool,
    /// The controller reports it is in Remote Control. Refreshed by the dashboard
    /// keepalive; dashboard commands like `play` and `brake release` are refused by
    /// PolyScope when this is false.
    pub remote_control: bool,
    pub goal_id: Option<String>,
    pub goal_sender: Option<oneshot::Sender<bool>>,
    pub handshake_sender: Option<oneshot::Sender<bool>>,
    pub feedback_sender: Option<mpsc::Sender<String>>,
    /// UR *safety* mode, RT packet offset 812. See `safety_mode_name`.
    ///
    /// This used to be called `robot_state`, which was a misreading: offset 812 is
    /// Safety Mode, and Robot Mode is the separate field below.
    pub safety_mode: i32,
    /// UR *robot* mode, RT packet offset 756. See `robot_mode_name`.
    pub robot_mode: i32,
    /// Raw RT packet offset 1052, kept for diagnostics only.
    ///
    /// UR documents this as "Program state", but it does not carry the
    /// stopped/playing/paused enum the dashboard reports: on PolyScope 5.25 it
    /// reads 1 with nothing running and 4 with an interface script live. Nothing
    /// should branch on it - use `program_state` below, which comes from the
    /// dashboard and is authoritative.
    pub program_state_raw: i32,
    /// Program state as the dashboard reports it, e.g. "STOPPED", "PLAYING",
    /// "PAUSED". Refreshed by the dashboard keepalive.
    pub program_state: String,
    /// Whether the controller reports a program as running. Dashboard-sourced.
    pub program_running: bool,
    /// PolyScope operational mode as the dashboard reports it: "MANUAL",
    /// "AUTOMATIC", or "NONE" when no mode password is set. Dashboard-sourced;
    /// the realtime stream does not carry it.
    pub operational_mode: String,
    pub joint_values: Vec<f64>,
    pub joint_speeds: Vec<f64>,
    /// Actual TCP pose as [x, y, z, rx, ry, rz], RT packet offset 444.
    pub tcp_pose: Vec<f64>,
    /// Active speed scaling (0.0 - 1.0), RT packet offset 940.
    pub speed_scaling: f64,
    pub digital_inputs: u32,
    pub digital_outputs: u32,
    pub forces: Vec<f64>,
    pub cancel_sender: Option<mpsc::Sender<()>>,
    /// The robot is being held by a dashboard `pause`.
    ///
    /// Set by the dashboard task once the controller confirms `PAUSED`, cleared on
    /// resume and on goal teardown. While it is true `command_server` rejects new
    /// motion requests: accepting a move on a robot an operator has deliberately
    /// held would be a surprise.
    pub motion_paused: bool,
    /// The command behind the live goal, retained so a resume can re-render it.
    pub active_command: Option<RobotCommand>,
    /// How many trajectory waypoints the running script has reported reaching.
    ///
    /// Fed by the `waypoint_reached <n>` lines the trajectory templates send on the
    /// script socket. A resume drops this many waypoints so the robot continues
    /// forward instead of driving back through the path it already covered.
    pub waypoints_completed: usize,
    /// A goal is between the script that was paused and the one that will finish it.
    ///
    /// `socket_server` clears `goal_id` as soon as the killed script's socket
    /// closes, which would otherwise leave a window where admission control sees no
    /// live goal and lets a new request in on top of the resume.
    pub reissuing: bool,
    /// Mirrors `cancel_sender`: signals the live goal to re-issue itself, which is
    /// the fallback when a dashboard `play` does not resume an injected script.
    pub resume_sender: Option<mpsc::Sender<()>>,
    /// Fixed controller facts, read once per dashboard connection.
    pub identity: Option<DashboardIdentity>,
}

impl DriverState {
    pub fn new() -> Self {
        DriverState {
            running: true,
            connected: false,
            dashboard_connected: false,
            remote_control: false,
            goal_id: None,
            goal_sender: None,
            handshake_sender: None,
            feedback_sender: None,
            safety_mode: 0,
            robot_mode: 0,
            program_state_raw: 0,
            program_state: "UNKNOWN".to_string(),
            program_running: false,
            operational_mode: "UNKNOWN".to_string(),
            joint_values: vec![],
            joint_speeds: vec![],
            tcp_pose: vec![],
            speed_scaling: 0.0,
            digital_inputs: 0,
            digital_outputs: 0,
            forces: vec![],
            cancel_sender: None,
            motion_paused: false,
            active_command: None,
            waypoints_completed: 0,
            reissuing: false,
            resume_sender: None,
            identity: None,
        }
    }
}

/// Take the `DriverState` lock, recovering from poisoning.
///
/// Every task in the driver shares this one mutex. With `.lock().unwrap()` a panic
/// in any single task poisons it and every other task then panics too, which turns
/// one local bug into a whole-process outage. The data behind the lock is plain
/// values and channel handles - there is no invariant a panicking writer could
/// leave half-built - so taking the inner value back is the right recovery.
pub fn lock_driver_state(driver_state: &Mutex<DriverState>) -> MutexGuard<'_, DriverState> {
    driver_state
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// UR safety mode (RT packet offset 812), as a name.
///
/// A thin wrapper over [`SafetyMode`], which is the type to reach for in new code.
/// This exists because the driver stores safety mode as the raw `i32` off the
/// realtime packet and several call sites only want it spelled out.
pub fn safety_mode_name(mode: i32) -> &'static str {
    SafetyMode::from_rt(mode).as_str()
}

/// UR robot mode (RT packet offset 756), as a name. See `safety_mode_name`.
pub fn robot_mode_name(mode: i32) -> &'static str {
    RobotMode::from_rt(mode).as_str()
}

/// Safety modes in which an in-flight goal can no longer complete.
///
/// Deliberately excludes `2 == REDUCED`, which is a normal operating mode: the
/// robot slows down inside a reduced-speed zone but keeps running.
pub fn safety_mode_aborts_goal(mode: i32) -> bool {
    SafetyMode::from_rt(mode).aborts_goal()
}

/// Safety modes in which a new motion request may be accepted.
pub fn safety_mode_accepts_goal(mode: i32) -> bool {
    SafetyMode::from_rt(mode).accepts_goal()
}

// This is to be sent out in the orbot command
#[derive(Serialize, Deserialize, Clone)]
pub struct Waypoint {
    pub acceleration: f64,
    pub velocity: f64,
    pub global_acceleration_scaling: f64,
    pub global_velocity_scaling: f64,
    pub use_execution_time: bool,
    pub execution_time: f64,
    pub use_blend_radius: bool,
    pub blend_radius: f64,
    pub use_joint_positions: bool,
    pub joint_positions: Vec<f64>,
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: Vec<f64>,
    pub use_payload: bool,
    pub payload: String,
    pub target_in_base: String,
    pub relative_pose: Vec<f64>,
    pub tcp_in_faceplate: String,
    pub force_threshold: f64,
    /// Travel to this waypoint in a straight tool line (`movel`) rather than by
    /// joint interpolation (`movej`).
    ///
    /// Orthogonal to `use_joint_positions`, which says how the *target* is
    /// specified rather than how the arm gets there - `movel` accepts a joint
    /// vector and moves to its forward kinematics in a straight line, which is
    /// exactly what `trajectory_unsafe_move_l` already relies on.
    pub use_linear_motion: bool,
}

// This arrives in the "waypoints" from redis
#[derive(Serialize, Deserialize, Clone)]
pub struct WaypointRaw {
    pub acceleration: f64,
    pub velocity: f64,
    pub global_acceleration_scaling: f64,
    pub global_velocity_scaling: f64,
    pub use_execution_time: bool,
    pub execution_time: f64,
    pub use_blend_radius: bool,
    pub blend_radius: f64,
    pub use_joint_positions: bool,
    pub joint_positions: Vec<f64>,
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: Vec<f64>,
    pub use_relative_pose: bool,
    pub relative_pose: Vec<f64>,
    pub use_payload: bool,
    pub payload: String,
    pub baseframe_id: String,
    pub faceplate_id: String,
    pub goal_feature_id: String,
    pub tcp_id: String,
    pub root_frame_id: String,
    pub force_threshold: f64,
    /// See `Waypoint::use_linear_motion`. Optional on the wire, defaulting to
    /// `false` (`movej`), which is what every existing publisher means.
    pub use_linear_motion: bool,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct RobotCommand {
    // MoveJ, Movel, StartVacuum...
    pub command_type: String,
    // If command is 'move_j', joint acceleration of leading axis [rad/s^2].
    // If command is 'move_l', tool acceleration [m/s^2].
    pub acceleration: f64,
    // If command is 'move_j', joint velocity of leading axis [rad/s].
    // If command is 'move_l', tool velocity [m/s].
    pub velocity: f64,
    pub global_acceleration_scaling: f64, // Between 0.0 and 1.0
    pub global_velocity_scaling: f64,     // Between 0.0 and 1.0
    // Movement execution time is the alternative parameter
    // that can control the speed of the robot. If set, the robot will
    // execute the motion in the time specified here (in seconds).
    // Time setting has priority over speed and acceleration settings.
    pub use_execution_time: bool,
    pub execution_time: f64,
    // Blend radius. If a blend radius is set, the robot arm trajectory
    // will be modified to avoid the robot stopping at the point.
    // However, if the blend region of this move overlaps with the blend
    // radius of previous or following waypoints, this move will be
    // skipped, and an ’Overlapping Blends’ warning message will be generated.
    pub use_blend_radius: bool,
    pub blend_radius: f64,
    // If executing a 'move_j', direct joint positions can be used
    // instead of finding an inverse kinematics solution. Otherwise,
    // the IK solver will calculate the joint positions based on the
    // tcp_id and goal_feature_id
    pub use_joint_positions: bool,
    pub joint_positions: Vec<f64>,
    // If executing a 'move_l', a preferred joint configuration
    // can be set, so that the IK solver can choose something close to it if possible.
    pub use_preferred_joint_config: bool,
    pub preferred_joint_config: Vec<f64>,
    // If a payload should be used. Mass, CoG and Inertia can be set.
    pub use_payload: bool,
    pub payload: String,
    // base_link if simulation, base if real or ursim
    // pub baseframe_id: String,
    // usually tool0, but could be rsp if that is the setup
    // pub faceplate_id: String,
    // Name of the frame to go to.
    // pub goal_feature_id: String,
    // Name of the TCP to be used to go to the goal feature frame.
    // pub tcp_id: String,
    // Calculated transforms with the lookup
    pub target_in_base: String, // use pose_to_string
    // pub set_tcp: bool, // if false, no tcp will be set (will remain 0.0.0.0.0.0.0)
    pub relative_pose: Vec<f64>, // use pose_to_string, relative to current TCP pose
    pub tcp_in_faceplate: String, // use pose_to_string
    pub force_threshold: f64,
    // Whether the trajectory templates should report each waypoint they reach on
    // the script socket. Progress is what lets a paused blended trajectory resume
    // where it stopped, but the socket write sits between two blended moves and may
    // flush the controller's look-ahead buffer, breaking the blend. Off means a
    // guaranteed-smooth blend and no mid-trajectory resume.
    pub report_waypoint_progress: bool,
    /// Name of a trajectory stored under the trajectory directory, without the
    /// `.json`. Empty means "there is no stored trajectory": either this is not a
    /// trajectory command at all, or it is one to be planned from `waypoints`.
    pub trajectory_id: String,
    pub waypoints: Vec<Waypoint>
    // pub gripper_velocity: f64,
    // pub gripper_force: f64,
    // pub gripper_ref_pos_percentage: i64,
    // pub gripper_position: i64 // open: 100, closed: 0, or anything inbetween
}

// #[derive(Debug, Serialize, Deserialize, Clone)]
// pub struct GripperCommand {
//     // open, close, move_to, set force, activate, etc...
//     pub command_type: String,
//     pub velocity: f64,
//     pub force: f64,
//     pub ref_pos_percentage: i64, // fully closed: 100, fully open 0, or anything inbetween
// }

#[derive(Serialize, Deserialize, Clone)]
pub struct Payload {
    /// Payload Mass in kilograms.
    pub mass: f32,

    /// Payload Center of Gravity offsets (in meters) from the tool mount.
    pub cog_x: f32,
    pub cog_y: f32,
    pub cog_z: f32,

    /// Payload Inertia Matrix (in kg*m^2) with origin at the CoG and axes aligned with the tool flange axes.
    pub ixx: f32,
    pub iyy: f32,
    pub izz: f32,
    pub ixy: f32,
    pub ixz: f32,
    pub iyz: f32,
}

impl Default for Payload {
    fn default() -> Self {
        Payload {
            mass: 0.0,
            cog_x: 0.0,
            cog_y: 0.0,
            cog_z: 0.0,
            ixx: 0.0,
            iyy: 0.0,
            izz: 0.0,
            ixy: 0.0,
            ixz: 0.0,
            iyz: 0.0,
        }
    }
}

impl Payload {
    pub fn to_string(&self) -> String {
        format!(
            "{},[{},{},{}],[{},{},{},{},{},{}]",
            self.mass,
            self.cog_x,
            self.cog_y,
            self.cog_z,
            self.ixx,
            self.iyy,
            self.izz,
            self.ixy,
            self.ixz,
            self.iyz
        )
    }
}

// pub fn payload_to_string(p: Payload) -> String {
//     format!(
//         "{},[{},{},{}],[{},{},{},{},{},{}]",
//         p.mass, p.cog_x, p.cog_y, p.cog_z, p.ixx, p.iyy, p.izz, p.ixy, p.ixz, p.iyz
//     )
// }

// fn joint_vector_to_string(j: &[f64]) -> String {
//     match j.len() == 6 {
//         true => format!("[{},{},{},{},{},{}]", j[0], j[1], j[2], j[3], j[4], j[5]),
//         false => "".to_string(),
//     }
// }

pub fn transform_to_string(tf_stamped: &SPTransformStamped) -> String {
    let x = tf_stamped.transform.translation.x;
    let y = tf_stamped.transform.translation.y;
    let z = tf_stamped.transform.translation.z;
    let rot = tf_stamped.transform.rotation.clone();
    let angle = 2.0 * rot.w.acos();
    let den = (1.0 - rot.w.powi(2)).sqrt();

    // Normalize quaternion for safety
    let norm = (rot.w.powi(2) + rot.x.powi(2) + rot.y.powi(2) + rot.z.powi(2)).sqrt();

    let x_r = rot.x / norm;
    let y_r = rot.y / norm;
    let z_r = rot.z / norm;

    let (rx, ry, rz) = if den.abs() < f64::EPSILON {
        (x_r * angle, y_r * angle, z_r * angle)
    } else {
        (
            (x_r / den) * angle,
            (y_r / den) * angle,
            (z_r / den) * angle,
        )
    };

    format!("p[{},{},{},{},{},{}]", x, y, z, rx, ry, rz)
}

pub fn pose_to_string(pose: [f64; 6]) -> String {
    format!(
        "p[{},{},{},{},{},{}]",
        pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]
    )
}

pub struct URDFParameters {
    pub name: String,
    pub ur_type: String,
    pub safety_limits: bool,
    pub safety_pos_margin: f64,
    pub safety_k_position: f64,
    pub description_file: String,
    pub rviz_config_file: String,
    pub tf_prefix: String,
    pub ur_meshes_path: String
}

impl URDFParameters {
    /// This robot's name for a URDF link.
    ///
    /// An empty `tf_prefix` returns `name` unchanged, which is what keeps a
    /// single-robot cell publishing the bare `base_link`..`tool0` that its scene files
    /// and its model already name. The prefix carries its own separator (`"r2_"`), so
    /// empty is exactly the identity - do not reintroduce a `format!("{}_{}", ..)`
    /// here, which cannot express "no prefix".
    pub fn frame(&self, name: &str) -> String {
        match self.tf_prefix.is_empty() {
            true => name.to_string(),
            false => format!("{}{}", self.tf_prefix, name),
        }
    }
}

impl Default for URDFParameters {
    fn default() -> Self {
        URDFParameters {
            name: "robot_1".to_string(),
            ur_type: "ur10e".to_string(),
            safety_limits: true,
            safety_pos_margin: 0.15,
            safety_k_position: 20.0,
            description_file: "TODO!".to_string(),
            rviz_config_file: "TODO!".to_string(),
            tf_prefix: "".to_string(),
            ur_meshes_path: "TODO!".to_string(),
        }
    }
}

impl WaypointRaw {
    /// Decode `{robot}_waypoints` - an `SPValue::Array` of `SPValue::Map` - into a
    /// waypoint list.
    ///
    /// A blended trajectory is only meaningful as a whole: a waypoint silently
    /// dropped from the middle changes the path the robot takes. So any field that
    /// fails to decode aborts the entire list rather than yielding a shorter one,
    /// and each failure is logged with the waypoint index and field name so the
    /// offending value can be found without guessing.
    pub fn vec_from_sp_value(value: Option<SPValue>, log_target: &str) -> Option<Vec<WaypointRaw>> {
        let SPValue::Array(ArrayOrUnknown::Array(arr)) = value? else {
            return None;
        };

        let mut extracted = Vec::with_capacity(arr.len());

        for (index, item) in arr.iter().enumerate() {
            let SPValue::Map(MapOrUnknown::Map(map)) = item else {
                log::error!(target: log_target, "Waypoint at index {} is NOT a Map! It is: {:?}", index, item);
                return None;
            };

            let get_val = |k: &str| -> Option<&SPValue> {
                map.iter()
                    .find(|(key_sp, _)| {
                        matches!(key_sp, SPValue::String(StringOrUnknown::String(s)) if s == k)
                    })
                    .map(|(_, val_sp)| val_sp)
            };

            let get_f64 = |k: &str| -> Option<f64> {
                match get_val(k) {
                    Some(SPValue::Float64(FloatOrUnknown::Float64(f))) => Some(f.into_inner()),
                    Some(SPValue::Int64(IntOrUnknown::Int64(i))) => Some(*i as f64),
                    Some(other) => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' failed! Expected Float64/Int64, got: {:?}", index, k, other);
                        None
                    }
                    None => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' is MISSING from the map!", index, k);
                        None
                    }
                }
            };

            let get_bool = |k: &str| -> Option<bool> {
                match get_val(k) {
                    Some(SPValue::Bool(BoolOrUnknown::Bool(b))) => Some(*b),
                    Some(other) => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' failed! Expected Bool, got: {:?}", index, k, other);
                        None
                    }
                    None => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' is MISSING from the map!", index, k);
                        None
                    }
                }
            };

            let get_string = |k: &str| -> Option<String> {
                match get_val(k) {
                    Some(SPValue::String(StringOrUnknown::String(s))) => Some(s.clone()),
                    Some(other) => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' failed! Expected String, got: {:?}", index, k, other);
                        None
                    }
                    None => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' is MISSING from the map!", index, k);
                        None
                    }
                }
            };

            let get_f64_vec = |k: &str| -> Option<Vec<f64>> {
                match get_val(k) {
                    Some(SPValue::Array(ArrayOrUnknown::Array(a))) => {
                        let mut vec = Vec::with_capacity(a.len());
                        for (i, v) in a.iter().enumerate() {
                            match v {
                                SPValue::Float64(FloatOrUnknown::Float64(f)) => {
                                    vec.push(f.into_inner())
                                }
                                SPValue::Int64(IntOrUnknown::Int64(val)) => vec.push(*val as f64),
                                other => {
                                    log::error!(target: log_target, "Waypoint {}: Field '{}' array element at index {} failed! Expected Float64/Int64, got: {:?}", index, k, i, other);
                                    return None;
                                }
                            }
                        }
                        Some(vec)
                    }
                    Some(other) => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' failed! Expected Array, got: {:?}", index, k, other);
                        None
                    }
                    None => {
                        log::error!(target: log_target, "Waypoint {}: Field '{}' is MISSING from the map!", index, k);
                        None
                    }
                }
            };

            // Optional fields must NOT use `get_bool(..)?`: a missing key there
            // returns None, which aborts the whole waypoint list and therefore the
            // whole request. Every publisher written before a field existed would
            // start failing. New fields get a default instead.
            let get_bool_or = |k: &str, default: bool| -> bool {
                match get_val(k) {
                    Some(SPValue::Bool(BoolOrUnknown::Bool(b))) => *b,
                    Some(other) => {
                        log::warn!(target: log_target, "Waypoint {}: Field '{}' expected Bool, got {:?}; using {}.", index, k, other, default);
                        default
                    }
                    None => default,
                }
            };

            let use_joint_positions = get_bool("use_joint_positions")?;

            extracted.push(WaypointRaw {
                acceleration: get_f64("acceleration")?,
                velocity: get_f64("velocity")?,
                global_acceleration_scaling: get_f64("global_acceleration_scaling")?,
                global_velocity_scaling: get_f64("global_velocity_scaling")?,
                use_execution_time: get_bool("use_execution_time")?,
                execution_time: get_f64("execution_time")?,
                use_blend_radius: get_bool("use_blend_radius")?,
                blend_radius: get_f64("blend_radius")?,
                use_joint_positions,
                joint_positions: get_f64_vec("joint_positions")?,
                use_preferred_joint_config: get_bool("use_preferred_joint_config")?,
                preferred_joint_config: get_f64_vec("preferred_joint_config")?,
                use_relative_pose: get_bool("use_relative_pose")?,
                relative_pose: get_f64_vec("relative_pose")?,
                use_payload: get_bool("use_payload")?,
                payload: get_string("payload")?,
                baseframe_id: get_string("baseframe_id")?,
                faceplate_id: get_string("faceplate_id")?,
                // Moving to joint positions needs no goal frame, and callers leave
                // it out in that case rather than sending a placeholder.
                goal_feature_id: if use_joint_positions {
                    String::new()
                } else {
                    get_string("goal_feature_id")?
                },
                tcp_id: get_string("tcp_id")?,
                root_frame_id: get_string("root_frame_id")?,
                force_threshold: get_f64("force_threshold")?,
                use_linear_motion: get_bool_or("use_linear_motion", false),
            });
        }

        Some(extracted)
    }
}

/// Read a bool from a `State` that may not contain the key.
///
/// `State::get_value` - which every `get_*_or_default_*` accessor funnels through
/// - logs and then **panics** when a key is absent, and `build_state` silently
/// drops any key whose stored value fails to deserialize. Together those mean one
/// malformed value in Redis, hand-written or left over from an older build, takes
/// down the whole driver rather than failing the one request that touched it.
///
/// Checking membership first turns that into the default. A key we seeded at
/// startup going missing is worth a log line, so it is not silent.
pub fn state_bool_or(state: &State, key: &str, default: bool, log_target: &str) -> bool {
    if !state.contains(key) {
        log::warn!(target: log_target, "'{}' is missing or unreadable, using {}.", key, default);
        return default;
    }
    state.get_bool_or_value(key, default, log_target)
}

/// Read a string from a `State` that may not contain the key. See `state_bool_or`.
pub fn state_string_or(state: &State, key: &str, default: &str, log_target: &str) -> String {
    if !state.contains(key) {
        log::warn!(target: log_target, "'{}' is missing or unreadable, using '{}'.", key, default);
        return default.to_string();
    }
    state.get_string_or_value(key, default.to_string(), log_target)
}

/// Read an i64 from a `State` that may not contain the key. See `state_bool_or`.
pub fn state_int_or(state: &State, key: &str, default: i64, log_target: &str) -> i64 {
    if !state.contains(key) {
        log::warn!(target: log_target, "'{}' is missing or unreadable, using {}.", key, default);
        return default;
    }
    state.get_int_or_value(key, default, log_target)
}

#[cfg(test)]
mod urdf_parameters_tests {
    use crate::URDFParameters;

    /// The property the whole single-robot run rests on: with no prefix configured,
    /// every frame name the driver publishes is the URDF link name unchanged. If this
    /// breaks, `shared_folder/transforms/base_link.json` and every
    /// `.baseframe_id("base_link")` in the model stop matching what is in Redis.
    #[test]
    fn no_prefix_is_the_identity() {
        let params = URDFParameters::default();
        assert_eq!(params.tf_prefix, "");
        for link in [
            "base_link",
            "base_link_inertia",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "flange",
            "ft_frame",
            "tool0",
            "shoulder_link_visual",
        ] {
            assert_eq!(params.frame(link), link, "'{link}' must survive unchanged");
        }
    }

    /// The prefix carries its own separator, so it is a plain concatenation - no
    /// underscore is inserted and none is assumed.
    #[test]
    fn a_prefix_is_concatenated_verbatim() {
        let mut params = URDFParameters::default();
        params.tf_prefix = "r2_".to_string();
        assert_eq!(params.frame("base_link"), "r2_base_link");
        assert_eq!(params.frame("tool0"), "r2_tool0");
        // The `_visual` suffix is already part of the name by the time it gets here,
        // so the prefix lands on the outside where the marker namespace expects it.
        assert_eq!(
            params.frame("shoulder_link_visual"),
            "r2_shoulder_link_visual"
        );
    }
}
