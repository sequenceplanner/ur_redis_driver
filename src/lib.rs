// TODO: Measure these a few more times and take the average, RSP only weighed another time 0.83kg

pub const UR_DRIVER_SOCKET_PORT: u16 = 50000;
pub const ROBOT_STATE_UPDATE_INTERVAL_MS: u16 = 20;

pub const RSP_ONLY_PAYLOAD: &str = "0.69,[0.026,-0.008,0.012],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SPONGE_PAYLOAD: &str = "1.88,[0.002,0.003,0.071],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_GRIPPER_PAYLOAD: &str = "2.24,[-0.001,0.002,0.068],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_BVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_PHOTONEO_PAYLOAD: &str = "3.29,[0.008,0.003,0.082],[0.0,0.0,0.0,0.0,0.0,0.0]";

pub mod core;
pub use core::state::*;
pub use core::structs::*;

pub mod interfaces;
pub use interfaces::state_publisher::*;
// pub use ros::action_client::*;
// // pub use ros::dashboard_client::*;
// // pub use ros::control_ghost::*;
// pub use ros::robot_state_publisher::*;
// pub use ros::ur_script_driver::*;
// pub use ros::urdf_parsing::*;
// pub use ros::joint_subscriber::*;

pub mod tools;
pub use tools::generate_ur_script::*;

pub mod driver;
pub use driver::realtime_reader::*;
