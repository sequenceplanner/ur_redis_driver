use std::fmt;

use micro_sp::*;
use serde::{Deserialize, Serialize};
// use tokio::sync::oneshot;

#[derive(Serialize, Deserialize, Clone)]
pub enum CommandType {
    UNKNOWN,
    ConnectGripper,
    CloseGripper,
    OpenGripper,
    MoveL,
    MoveJ,
    SafeMoveJ,
    SafeMoveL,
    PickVacuum,
    PlaceVacuum,
    StartVacuum,
    StopVacuum,
}

#[derive(Serialize, Deserialize, Clone)]
pub enum DashboardCommandType {
    UNKNOWN,
    Stop,
    ResetProtectiveStop,
}

impl fmt::Display for CommandType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            CommandType::ConnectGripper => "connect_robotiq_gripper",
            CommandType::CloseGripper => "close_robotiq_gripper",
            CommandType::OpenGripper => "open_robotiq_gripper",
            CommandType::MoveL => "move_l",
            CommandType::MoveJ => "move_j",
            CommandType::SafeMoveJ => "safe_move_j",
            CommandType::SafeMoveL => "safe_move_l",
            CommandType::PickVacuum => "pick_vacuum",
            CommandType::PlaceVacuum => "place_vacuum",
            CommandType::StartVacuum => "start_vacuum",
            CommandType::StopVacuum => "stop_vacuum",
            CommandType::UNKNOWN => "unknown",
        };
        write!(f, "{}", s)
    }
}

impl fmt::Display for DashboardCommandType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            DashboardCommandType::Stop => "stop",
            DashboardCommandType::ResetProtectiveStop => "reset_protective_stop",
            DashboardCommandType::UNKNOWN => "unknown",
        };
        write!(f, "{}", s)
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct RobotCommand {
    // MoveJ, Movel, StartVacuum...
    pub command_type: String,
    // If command is 'move_j', joint acceleration of leading axis [rad/s^2].
    // If command is 'move_l', tool acceleration [m/s^2].
    pub accelleration: f64,
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
    format!("p[{},{},{},{},{},{}]", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
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
        }
    }
}
