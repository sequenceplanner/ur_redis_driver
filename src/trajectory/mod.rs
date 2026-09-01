//! Time-optimal, collision-unaware trajectory generation for the UR arms.
//!
//! See `frames` for the base-frame convention, which everything else depends on.

pub mod frames;
pub mod kinematics;
pub mod limits;
pub mod path;
pub mod emit;
pub mod online;
pub mod plan;
pub mod store;
pub mod timing;
