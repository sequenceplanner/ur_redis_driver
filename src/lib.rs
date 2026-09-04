// TODO: Measure these a few more times and take the average, RSP only weighed another time 0.83kg

/// The port the driver's script-feedback socket server listens on when nothing says
/// otherwise. The standard value; a single-driver host never needs to change it.
pub const DEFAULT_UR_DRIVER_SOCKET_PORT: u16 = 50000;

/// The port this driver's socket server binds, and the port the URScript generated for
/// this driver is told to call back on.
///
/// Read once from `UR_DRIVER_SOCKET_PORT`. It has to be a single value resolved in one
/// place because two callers depend on agreeing: `driver::socker_server` binds it, and
/// `tools::generate_ur_script` writes it into the `socket_open` line of the program
/// uploaded to the robot. If those ever disagreed, the robot would dial a different
/// driver than the one that sent it the program.
///
/// Two drivers on one host must be given different values - the bind address comes from
/// `local_ip()`, so it is the same for both, and the second `bind` fails with
/// `EADDRINUSE` and takes the whole driver down with it.
pub fn ur_driver_socket_port() -> u16 {
    static PORT: std::sync::LazyLock<u16> = std::sync::LazyLock::new(|| {
        match std::env::var("UR_DRIVER_SOCKET_PORT") {
            Ok(raw) => match raw.trim().parse::<u16>() {
                Ok(port) => port,
                Err(e) => {
                    log::warn!(target: "ur_redis_driver", "UR_DRIVER_SOCKET_PORT is not a port number ({}): {}. Using {}.", raw, e, DEFAULT_UR_DRIVER_SOCKET_PORT);
                    DEFAULT_UR_DRIVER_SOCKET_PORT
                }
            },
            Err(_) => DEFAULT_UR_DRIVER_SOCKET_PORT,
        }
    });
    *PORT
}
pub const ROBOT_STATE_UPDATE_INTERVAL_MS: u16 = 5;

pub const RSP_ONLY_PAYLOAD: &str = "0.69,[0.026,-0.008,0.012],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SPONGE_PAYLOAD: &str = "1.88,[0.002,0.003,0.071],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_GRIPPER_PAYLOAD: &str = "2.24,[-0.001,0.002,0.068],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_BVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_SVT_PAYLOAD: &str = "1.3,[0.001,0.005,0.06],[0.0,0.0,0.0,0.0,0.0,0.0]";
pub const RSP_AND_PHOTONEO_PAYLOAD: &str = "3.29,[0.008,0.003,0.082],[0.0,0.0,0.0,0.0,0.0,0.0]";

pub mod core;
pub use core::dashboard_types::*;
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

pub mod trajectory;
// Only the online bridge is re-exported at the crate root. The rest of the module
// defines `Pose`, `Limits` and friends, which would collide with the `k::` and
// `micro_sp::` globs already in scope across this crate.
pub use trajectory::online::*;

pub mod tools;
pub use tools::generate_ur_script::*;

pub mod driver;
pub use driver::realtime_reader::*;
