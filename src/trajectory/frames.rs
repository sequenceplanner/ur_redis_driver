//! Pose representation and the `base_link` / UR `base` frame convention.
//!
//! Two frames matter here and they are 180 degrees apart about Z.
//!
//! * **UR `base`** is what the controller means by a `p[...]` pose. It is where
//!   the standard UR Denavit-Hartenberg parameters apply, and it is the frame
//!   `get_actual_tcp_pose()` and realtime packet offset 444 report in.
//! * **URDF `base_link`** is REP-103 aligned (X+ forward). The URDF says so in a
//!   comment on `base_link-base_link_inertia`: "the internal frames of the
//!   robot/controller have X+ pointing backwards".
//!
//! In `src/ur_description/urdf/ur20.urdf` both `base_link -> base_link_inertia`
//! (the start of the serial chain) and `base_link -> base` are the same fixed
//! `rpy="0 0 pi"`, so **`base` and `base_link_inertia` are the same frame** and
//! `T_base_link->tool0(q) = Rz(pi) * DH(q)`.
//!
//! # Why the default is *not* to apply that rotation
//!
//! `state_publisher.rs` adds `pi` to joint 0 before publishing transforms. Because
//! the first joint origin is a pure translation along Z, `Rz(pi)` commutes through
//! it:
//!
//! ```text
//! T_base_link->shoulder = Rz(pi) * Tz(d1) * Rz(q1)  =  Tz(d1) * Rz(q1 + pi)
//! ```
//!
//! so that offset makes the *published TF tree* satisfy
//! `T_base_link->tool0 = DH(q)` - i.e. it makes the tree's `base_link` behave
//! numerically as the UR `base` frame. That is precisely why the existing path of
//! `lookup_transform(base_link, goal) -> transform_to_string -> p[...] ->
//! get_inverse_kin` works on the real cell today.
//!
//! Applying `Rz(pi)` on top of that would mirror every already-calibrated pose. So
//! [`FrameConfig::base_link_is_ur_base`] defaults to `true`: poses looked up
//! against `base_link` are taken to already be in the UR base frame. The
//! conversion is implemented and tested so that flipping one flag is all it takes
//! once the TF tree is fixed to match the URDF.
//!
//! # The tool side needs no correction
//!
//! `tool0` **is** the UR flange frame: the composition
//! `wrist_3 -> flange -> tool0` in the URDF equals DH position 6 exactly. So
//! `tcp_in_faceplate`, which `command_server` looks up as `tool0 -> tcp_id` and
//! hands to `set_tcp()`, is already right. Note that the ROS-Industrial `flange`
//! link is *not* the UR flange - it differs by a further `Rz(pi/2) * Ry(pi/2)` -
//! and must not be used as one.

use k::nalgebra as na;
use na::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// A rigid transform. Alias so call sites read as poses rather than as isometries.
pub type Pose = Isometry3<f64>;

/// Which frame a `[x, y, z, rx, ry, rz]` array is expressed in.
///
/// Carried around rather than assumed, because the two differ by `Rz(pi)` and a
/// silent mix-up mirrors the robot about its own base instead of failing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseFrame {
    /// The UR controller's `Base`. What `p[...]` means to URScript.
    UrBase,
    /// The URDF `base_link`, REP-103 aligned.
    UrdfBaseLink,
}

/// How this driver's TF tree relates to the UR controller's base frame.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct FrameConfig {
    /// `true` when the TF tree's `base_link` is numerically the UR `base` frame,
    /// which is the case while `state_publisher`'s `joints_for_tf[0] += PI` is in
    /// place. Poses looked up against `base_link` then need no correction.
    ///
    /// Set `false` once the TF tree is published to match the URDF, at which point
    /// `Rz(pi)` is applied to every pose crossing into the planner.
    pub base_link_is_ur_base: bool,
}

impl Default for FrameConfig {
    fn default() -> Self {
        // Matches the driver as it actually behaves today. Changing this default
        // silently re-aims every taught pose, so it is a deliberate, coordinated
        // change - see the module docs.
        FrameConfig { base_link_is_ur_base: true }
    }
}

impl FrameConfig {
    /// Reinterpret a pose that was looked up against `base_link` as a UR base pose.
    pub fn base_link_to_ur_base(&self, pose: &Pose) -> Pose {
        if self.base_link_is_ur_base { *pose } else { rz_pi() * pose }
    }

    /// The inverse of [`base_link_to_ur_base`](Self::base_link_to_ur_base).
    pub fn ur_base_to_base_link(&self, pose: &Pose) -> Pose {
        if self.base_link_is_ur_base { *pose } else { rz_pi() * pose }
    }
}

/// The `base_link` -> UR `base` rotation: 180 degrees about Z, no translation.
///
/// It is its own inverse, which is why both directions above share an
/// implementation.
///
/// This is a **change of parent frame**, i.e. a left-multiply. It is not a sign
/// flip on the rotation vector: `Rz(pi)` negates x and y of the *translation*, but
/// the rotation vector of the product is not related to the original's by negating
/// components. `tests/trajectory_kinematics.rs` asserts the naive version is wrong
/// so nobody "simplifies" it back.
pub fn rz_pi() -> Pose {
    Isometry3::from_parts(
        Translation3::new(0.0, 0.0, 0.0),
        UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, PI)),
    )
}

/// Build a pose from URScript's `[x, y, z, rx, ry, rz]` layout.
///
/// `rx, ry, rz` is a rotation vector (axis-angle scaled by the angle), which is
/// what UR uses everywhere and what `pose_to_string` in `core::structs` emits.
pub fn pose_from_array(p: [f64; 6]) -> Pose {
    Isometry3::from_parts(
        Translation3::new(p[0], p[1], p[2]),
        UnitQuaternion::from_scaled_axis(Vector3::new(p[3], p[4], p[5])),
    )
}

/// The inverse of [`pose_from_array`].
pub fn pose_to_array(pose: &Pose) -> [f64; 6] {
    let t = pose.translation.vector;
    let r = pose.rotation.scaled_axis();
    [t.x, t.y, t.z, r.x, r.y, r.z]
}

/// Parse a URScript pose literal, `p[x,y,z,rx,ry,rz]`.
///
/// Accepts what `core::structs::pose_to_string` and `transform_to_string` produce,
/// with or without the leading `p` and with arbitrary internal whitespace, so a
/// stored trajectory and a Redis-sourced pose parse through the same path.
pub fn parse_pose_string(s: &str) -> Result<[f64; 6], String> {
    let trimmed = s.trim();
    let body = trimmed
        .strip_prefix('p')
        .unwrap_or(trimmed)
        .trim()
        .strip_prefix('[')
        .and_then(|r| r.strip_suffix(']'))
        .ok_or_else(|| format!("'{}' is not a p[...] pose literal", s))?;

    let mut out = [0.0f64; 6];
    let mut seen = 0usize;
    for part in body.split(',') {
        if seen == 6 {
            return Err(format!("'{}' has more than six components", s));
        }
        out[seen] = part
            .trim()
            .parse::<f64>()
            .map_err(|e| format!("'{}' component {} is not a number: {}", s, seen, e))?;
        seen += 1;
    }
    if seen != 6 {
        return Err(format!("'{}' has {} components, expected six", s, seen));
    }
    Ok(out)
}

/// Format a pose as a URScript literal, matching `core::structs::pose_to_string`.
pub fn pose_to_string(pose: &Pose) -> String {
    let p = pose_to_array(pose);
    format!("p[{},{},{},{},{},{}]", p[0], p[1], p[2], p[3], p[4], p[5])
}
