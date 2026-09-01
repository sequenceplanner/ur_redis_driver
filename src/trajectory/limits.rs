//! Joint limits, and the scaling convention the driver's templates already use.
//!
//! Position and velocity limits are read from the same URDF the driver loads, so
//! they cannot drift from the model being driven. Acceleration is the awkward one:
//! **UR does not publish acceleration limits.** Every
//! `src/ur_description/config/<model>/joint_limits.yaml` says
//! `has_acceleration_limits: false`, and URScript's documented defaults (`movej
//! a=1.4 rad/s^2`) are deliberately timid rather than a limit.
//!
//! So the acceleration limits here are **engineering estimates, not measurements**:
//! each joint is given the acceleration that would bring it from rest to its
//! published maximum velocity in [`RAMP_TIME_S`]. For a UR20 that yields roughly
//! `[4.2, 4.2, 5.2, 7.3, 7.3, 7.3] rad/s^2`. To keep the out-of-the-box behaviour
//! well short of a guess, [`PlanScaling::default`] applies an acceleration scaling
//! of 0.5 on top.
//!
//! Raising them is an empirical exercise. The honest procedure is to command
//! `speedj` with increasing `a` while logging joint velocity off the realtime
//! stream, and find the point at which the measured slope stops tracking the
//! commanded one.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Time a joint is assumed to need to reach its maximum velocity from rest.
///
/// The single number behind every default acceleration limit. Deliberately one
/// constant rather than a per-joint table, so that the assumption is visible and
/// adjustable in one place.
pub const RAMP_TIME_S: f64 = 0.5;

/// The joint names of a UR arm, in the order the URDF chain and every joint vector
/// in this driver uses.
pub const JOINT_NAMES: [&str; 6] = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
];

/// Per-joint kinematic limits for one arm.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    pub position_min: [f64; 6],
    pub position_max: [f64; 6],
    pub velocity_max: [f64; 6],
    /// Estimated, not published. See the module docs.
    pub acceleration_max: [f64; 6],
}

impl Limits {
    /// Read position and velocity limits from a URDF, estimating acceleration.
    ///
    /// Uses `urdf-rs`, which was already a declared dependency of this crate and
    /// otherwise unused. Reading the same file the driver loads for FK means the
    /// limits always describe the arm actually being driven.
    ///
    /// `type="continuous"` joints - the wrist 3 of a ur3 and ur3e - carry no
    /// position limit in the URDF and are given `+-2pi`, which is the range the
    /// controller allows.
    pub fn from_urdf(path: &str) -> Result<Self, String> {
        let robot = urdf_rs::read_file(path).map_err(|e| format!("{}: {}", path, e))?;

        let mut position_min = [0.0; 6];
        let mut position_max = [0.0; 6];
        let mut velocity_max = [0.0; 6];

        for (i, name) in JOINT_NAMES.iter().enumerate() {
            let joint = robot
                .joints
                .iter()
                .find(|j| j.name == *name)
                .ok_or_else(|| format!("{} has no joint '{}'", path, name))?;

            let continuous = matches!(joint.joint_type, urdf_rs::JointType::Continuous);
            if continuous || joint.limit.lower == joint.limit.upper {
                position_min[i] = -2.0 * PI;
                position_max[i] = 2.0 * PI;
            } else {
                position_min[i] = joint.limit.lower;
                position_max[i] = joint.limit.upper;
            }

            if joint.limit.velocity <= 0.0 {
                return Err(format!("{}: joint '{}' has no velocity limit", path, name));
            }
            velocity_max[i] = joint.limit.velocity;
        }

        Ok(Limits {
            position_min,
            position_max,
            velocity_max,
            acceleration_max: estimate_acceleration(&velocity_max),
        })
    }

    /// Apply velocity and acceleration scaling, returning the effective limits.
    ///
    /// Position limits are untouched: scaling makes the arm slower, not smaller.
    pub fn scaled(&self, scaling: &PlanScaling) -> Self {
        let mut out = *self;
        let (sv, sa) = (scaling.velocity(), scaling.acceleration());
        for i in 0..6 {
            out.velocity_max[i] *= sv;
            out.acceleration_max[i] *= sa;
        }
        out
    }

    /// Replace the estimated acceleration limits with explicit values.
    pub fn with_acceleration(mut self, acceleration_max: [f64; 6]) -> Self {
        self.acceleration_max = acceleration_max;
        self
    }

    /// Whether a configuration is inside the position limits.
    pub fn contains(&self, q: &[f64; 6]) -> bool {
        (0..6).all(|i| q[i] >= self.position_min[i] && q[i] <= self.position_max[i])
    }

    /// The first joint outside its position limits, if any.
    pub fn first_violation(&self, q: &[f64; 6]) -> Option<(usize, f64, f64, f64)> {
        (0..6)
            .find(|&i| q[i] < self.position_min[i] || q[i] > self.position_max[i])
            .map(|i| (i, q[i], self.position_min[i], self.position_max[i]))
    }
}

/// The default acceleration estimate: reach maximum velocity in [`RAMP_TIME_S`].
pub fn estimate_acceleration(velocity_max: &[f64; 6]) -> [f64; 6] {
    let mut a = [0.0; 6];
    for i in 0..6 {
        a[i] = velocity_max[i] / RAMP_TIME_S;
    }
    a
}

/// Velocity and acceleration scaling for one plan.
///
/// Follows the convention the `claude_*` templates established: a value outside
/// `(0.0, 1.0]` means "no scaling". That matters because `command_server` reads
/// these keys with `get_float_or_default_to_zero`, so a caller who never set them
/// gets `0.0` - which must mean "full speed", not "frozen".
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PlanScaling {
    pub velocity: f64,
    pub acceleration: f64,
}

impl Default for PlanScaling {
    fn default() -> Self {
        // Full velocity, but half of an acceleration limit nobody has measured.
        PlanScaling { velocity: 1.0, acceleration: 0.5 }
    }
}

impl PlanScaling {
    pub fn velocity(&self) -> f64 {
        clamp_scale(self.velocity)
    }
    pub fn acceleration(&self) -> f64 {
        clamp_scale(self.acceleration)
    }
}

/// `(0.0, 1.0]` passes through; anything else means "no scaling".
pub fn clamp_scale(s: f64) -> f64 {
    if s > 0.0 && s <= 1.0 { s } else { 1.0 }
}
