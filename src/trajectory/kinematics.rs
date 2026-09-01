//! Analytic forward and inverse kinematics for the UR arms, in the UR base frame.
//!
//! The controller already has `get_inverse_kin()`, so why solve it here? Three
//! reasons the on-robot solver cannot cover:
//!
//! * **Offline planning.** Nothing can be planned, timed or validated without a
//!   solver that runs in this process.
//! * **Branch continuity.** `get_inverse_kin` returns "the solution closest to
//!   qnear", one solution, with no way to see the others - and
//!   `templates/CLAUDE_TEMPLATES.md` already documents that two calls are not
//!   guaranteed to agree. Enumerating all eight branches is what lets a trajectory
//!   be checked for a wrist flip before the arm takes one.
//! * **Speed limits near singularities.** The Jacobian below is what caps a
//!   `movel` whose straight line passes close to a wrist singularity. The
//!   controller does not do this for you.
//!
//! Everything in this module is expressed in the **UR base frame**, where the
//! standard UR Denavit-Hartenberg parameters apply. See [`crate::trajectory::frames`].

use k::nalgebra as na;
use na::{Isometry3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

use super::frames::Pose;

/// The six Denavit-Hartenberg lengths that distinguish one UR model from another.
///
/// UR publishes these; they also fall out of the URDF joint origins, which is what
/// `tests/trajectory_kinematics.rs` cross-checks them against. `a2` and `a3` are
/// negative in UR's own convention and are kept that way here.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DhParameters {
    pub d1: f64,
    pub a2: f64,
    pub a3: f64,
    pub d4: f64,
    pub d5: f64,
    pub d6: f64,
}

impl DhParameters {
    /// DH parameters for a UR model name as used by `ROBOT_MODEL`, e.g. `"ur20"`.
    ///
    /// Values extracted from `src/ur_description/config/<model>/default_kinematics.yaml`
    /// and cross-checked against `urdf/<model>.urdf`. A calibrated cell has
    /// per-robot deviations from these nominal figures; see the note on
    /// `calibration` in the module docs of `limits`.
    pub fn for_model(model: &str) -> Option<Self> {
        let p = |d1, a2, a3, d4, d5, d6| Some(DhParameters { d1, a2, a3, d4, d5, d6 });
        match model {
            "ur3" => p(0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819),
            "ur3e" => p(0.15185, -0.24355, -0.2132, 0.13105, 0.08535, 0.0921),
            "ur5" => p(0.089159, -0.425, -0.39225, 0.10915, 0.09465, 0.0823),
            "ur5e" => p(0.1625, -0.425, -0.3922, 0.1333, 0.0997, 0.0996),
            "ur10" => p(0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922),
            "ur10e" => p(0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655),
            "ur16e" => p(0.1807, -0.4784, -0.36, 0.17415, 0.11985, 0.11655),
            "ur20" => p(0.2363, -0.862, -0.7287, 0.201, 0.1593, 0.1543),
            "ur30" => p(0.2363, -0.637, -0.5037, 0.201, 0.1593, 0.1543),
            _ => None,
        }
    }

    /// The DH row for joint `i` (0-based) as `(d, a, alpha)`.
    ///
    /// The UR convention: `alpha` is `+pi/2` at joints 1 and 4, `-pi/2` at joint 5,
    /// and zero elsewhere; the link lengths sit on `a` at joints 2 and 3 and on `d`
    /// everywhere else.
    fn row(&self, i: usize) -> (f64, f64, f64) {
        match i {
            0 => (self.d1, 0.0, PI / 2.0),
            1 => (0.0, self.a2, 0.0),
            2 => (0.0, self.a3, 0.0),
            3 => (self.d4, 0.0, PI / 2.0),
            4 => (self.d5, 0.0, -PI / 2.0),
            5 => (self.d6, 0.0, 0.0),
            _ => unreachable!("UR arms have six joints"),
        }
    }
}

/// One Denavit-Hartenberg link transform, `Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)`.
fn dh_transform(theta: f64, d: f64, a: f64, alpha: f64) -> Pose {
    let rz = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), theta),
    );
    let tz = Isometry3::from_parts(Translation3::new(0.0, 0.0, d), UnitQuaternion::identity());
    let tx = Isometry3::from_parts(Translation3::new(a, 0.0, 0.0), UnitQuaternion::identity());
    let rx = Isometry3::from_parts(
        Translation3::identity(),
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), alpha),
    );
    rz * tz * tx * rx
}

/// Forward kinematics: UR base frame -> tool flange (`tool0`), for a zero TCP.
///
/// This is exactly what `get_actual_tcp_pose()` reports when no TCP is set, and
/// `tests/trajectory_kinematics.rs` pins it against `k::Chain`'s reading of the
/// URDF so the frame convention cannot drift.
pub fn forward_kinematics(dh: &DhParameters, q: &[f64; 6]) -> Pose {
    let mut t = Isometry3::identity();
    for i in 0..6 {
        let (d, a, alpha) = dh.row(i);
        t *= dh_transform(q[i], d, a, alpha);
    }
    t
}

/// Forward kinematics to every joint frame, base first, flange last.
///
/// Used by the Jacobian and by the path sampler, which both need intermediate
/// frames and should not pay for six recomputations to get them.
pub fn forward_kinematics_all(dh: &DhParameters, q: &[f64; 6]) -> [Pose; 6] {
    let mut out = [Isometry3::identity(); 6];
    let mut t = Isometry3::identity();
    for i in 0..6 {
        let (d, a, alpha) = dh.row(i);
        t *= dh_transform(q[i], d, a, alpha);
        out[i] = t;
    }
    out
}

/// Wrap an angle into `(-pi, pi]`.
pub fn wrap_to_pi(a: f64) -> f64 {
    let mut x = (a + PI) % (2.0 * PI);
    if x <= 0.0 {
        x += 2.0 * PI;
    }
    x - PI
}

/// How close to a singularity a value has to be before it is treated as one.
const SINGULAR_EPS: f64 = 1e-7;

/// Slack allowed on a cosine before a branch is judged out of reach.
///
/// The cosines that decide `theta5` and `theta3` sit exactly at `+-1` on the edge
/// of the workspace, and rounding can push a legitimate boundary pose a hair past
/// it. Values inside this tolerance are clamped back; anything beyond is a branch
/// that genuinely cannot reach the pose, and is dropped rather than clamped.
///
/// Clamping instead of dropping is not a harmless simplification: it *fabricates*
/// a joint vector that the arm will happily move to, hundreds of millimetres from
/// the pose that was asked for.
const REACH_TOLERANCE: f64 = 1e-9;

/// All closed-form inverse-kinematics branches for a flange pose in the UR base
/// frame.
///
/// Returns up to eight solutions with every joint wrapped into `(-pi, pi]`. They
/// are *not* filtered against joint limits - see [`inverse_kinematics_filtered`]
/// for that, because filtering also wants the `+-2pi` branches that wrapping
/// throws away.
///
/// An unreachable pose yields an empty vector rather than an error: with eight
/// branches, "some of them exist" is the normal case and the caller decides what a
/// shortfall means.
///
/// # Structure of the derivation
///
/// `theta1` comes from the wrist centre `p - d6 * z6`, `theta5` from the tool
/// approach vector, and `theta6` from the tool's own axes. With those three known,
/// `T14 = T01^-1 * T06 * (T45 * T56)^-1` isolates joints 2, 3 and 4, which form a
/// planar two-link arm of lengths `a2` and `a3` in frame 1's xy-plane - because
/// joints 2 and 3 have `alpha = 0`, so `z1`, `z2` and `z3` are parallel. That
/// reduces the last three joints to the textbook two-link problem plus one
/// orientation term.
pub fn inverse_kinematics(dh: &DhParameters, target: &Pose) -> Vec<[f64; 6]> {
    let m = target.to_homogeneous();
    let n = Vector3::new(m[(0, 0)], m[(1, 0)], m[(2, 0)]);
    let o = Vector3::new(m[(0, 1)], m[(1, 1)], m[(2, 1)]);
    let a = Vector3::new(m[(0, 2)], m[(1, 2)], m[(2, 2)]);
    let p = Vector3::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);

    let mut out = Vec::with_capacity(8);

    // --- theta1: the wrist centre must lie on a cylinder of radius d4 ---
    let p05 = p - dh.d6 * a;
    let r = (p05.x * p05.x + p05.y * p05.y).sqrt();
    if r < dh.d4.abs() {
        // The wrist centre is inside the shoulder-offset cylinder: unreachable.
        return out;
    }
    let phi = (dh.d4 / r).clamp(-1.0, 1.0).acos();
    // atan2(p05.x, -p05.y) is atan2(p05.y, p05.x) + pi/2, the frame-1 offset.
    let base = p05.x.atan2(-p05.y);

    for theta1 in [base + phi, base - phi] {
        let (s1, c1) = theta1.sin_cos();

        // --- theta5: from the tool approach vector against the shoulder offset ---
        let c5_raw = (p.x * s1 - p.y * c1 - dh.d4) / dh.d6;
        if c5_raw.abs() > 1.0 + REACH_TOLERANCE {
            // This shoulder branch cannot produce the requested tool approach
            // direction at all.
            continue;
        }
        let a5 = c5_raw.clamp(-1.0, 1.0).acos();

        for theta5 in [a5, -a5] {
            let s5 = theta5.sin();

            // --- theta6: undefined when the wrist is straight (s5 == 0) ---
            let theta6 = if s5.abs() < SINGULAR_EPS {
                // Joints 4 and 6 are collinear; the split between them is free.
                // Zero is as good as anything, and the caller reseeds from the
                // preferred configuration when it selects a branch.
                0.0
            } else {
                let sign = if s5 > 0.0 { 1.0 } else { -1.0 };
                (sign * (-o.x * s1 + o.y * c1)).atan2(sign * (n.x * s1 - n.y * c1))
            };

            // --- joints 2, 3, 4: a planar two-link arm in frame 1 ---
            let t01 = dh_transform(theta1, dh.d1, 0.0, PI / 2.0);
            let t45 = dh_transform(theta5, dh.d5, 0.0, -PI / 2.0);
            let t56 = dh_transform(theta6, dh.d6, 0.0, 0.0);
            let t14 = t01.inverse() * target * (t45 * t56).inverse();

            let p13x = t14.translation.vector.x;
            let p13y = t14.translation.vector.y;
            let reach2 = p13x * p13x + p13y * p13y;

            let c3_raw = (reach2 - dh.a2 * dh.a2 - dh.a3 * dh.a3) / (2.0 * dh.a2 * dh.a3);
            if c3_raw.abs() > 1.0 + REACH_TOLERANCE {
                // The two-link sub-arm cannot span the distance this branch asks
                // of it. This is the check that makes an out-of-reach pose return
                // nothing instead of a plausible-looking lie.
                continue;
            }
            let a3ang = c3_raw.clamp(-1.0, 1.0).acos();

            // Rz(theta2+theta3+theta4) = R14 * Rx(-pi/2), which pins theta4 once
            // theta2 and theta3 are chosen.
            let m14 = (t14.rotation
                * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -PI / 2.0))
            .to_rotation_matrix();
            let theta234 = m14[(1, 0)].atan2(m14[(0, 0)]);

            for theta3 in [a3ang, -a3ang] {
                let theta2 = p13y.atan2(p13x)
                    - (dh.a3 * theta3.sin()).atan2(dh.a2 + dh.a3 * theta3.cos());
                let theta4 = theta234 - theta2 - theta3;

                out.push([
                    wrap_to_pi(theta1),
                    wrap_to_pi(theta2),
                    wrap_to_pi(theta3),
                    wrap_to_pi(theta4),
                    wrap_to_pi(theta5),
                    wrap_to_pi(theta6),
                ]);
            }
        }
    }

    out
}

/// Inverse kinematics, expanded across `+-2pi` branches and filtered to limits.
///
/// [`inverse_kinematics`] wraps every joint into `(-pi, pi]`, but the UR joints
/// travel to `+-2pi`, so a wrapped solution can be rejected as out of limits while
/// the same configuration shifted by a turn is perfectly legal - and, more
/// usefully, a shifted branch is often much closer to where the arm already is.
/// This expands each solution over the turns its limits allow and keeps whatever
/// lands inside them.
pub fn inverse_kinematics_filtered(
    dh: &DhParameters,
    target: &Pose,
    lower: &[f64; 6],
    upper: &[f64; 6],
) -> Vec<[f64; 6]> {
    let mut out = Vec::new();
    for sol in inverse_kinematics(dh, target) {
        expand_turns(&sol, lower, upper, 0, &mut [0.0; 6], &mut out);
    }
    out
}

/// Recursively offset each joint by whole turns that stay inside its limits.
fn expand_turns(
    sol: &[f64; 6],
    lower: &[f64; 6],
    upper: &[f64; 6],
    i: usize,
    acc: &mut [f64; 6],
    out: &mut Vec<[f64; 6]>,
) {
    if i == 6 {
        out.push(*acc);
        return;
    }
    // A UR joint spans at most +-2pi, so at most one extra turn either way can fit.
    for turn in [-1.0, 0.0, 1.0] {
        let v = sol[i] + turn * 2.0 * PI;
        if v >= lower[i] && v <= upper[i] {
            acc[i] = v;
            expand_turns(sol, lower, upper, i + 1, acc, out);
        }
    }
}

/// Per-joint weights used when ranking IK branches.
///
/// The proximal joints carry the arm's mass and sweep the most space, so a
/// solution that saves a little wrist rotation at the cost of a large shoulder
/// swing should lose. These are relative weights, not physical quantities.
pub const BRANCH_WEIGHTS: [f64; 6] = [4.0, 4.0, 3.0, 1.0, 1.0, 1.0];

/// Weighted distance between two configurations, as the largest weighted joint move.
///
/// Max-norm rather than a sum: what makes a motion slow or surprising is the one
/// joint that has to travel furthest, not the total.
pub fn configuration_distance(a: &[f64; 6], b: &[f64; 6]) -> f64 {
    (0..6)
        .map(|i| BRANCH_WEIGHTS[i] * (a[i] - b[i]).abs())
        .fold(0.0, f64::max)
}

/// Pick the branch closest to `preferred`.
///
/// This is the Rust counterpart of URScript's `qnear`, with the difference that it
/// can see every branch at once and is deterministic - `CLAUDE_TEMPLATES.md`
/// records that two `get_inverse_kin` calls are not guaranteed to agree.
pub fn select_nearest(solutions: &[[f64; 6]], preferred: &[f64; 6]) -> Option<[f64; 6]> {
    solutions
        .iter()
        .map(|s| (configuration_distance(s, preferred), s))
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(_, s)| *s)
}

/// The geometric Jacobian at `q`, in the UR base frame.
///
/// Rows 0..3 map joint rates to linear flange velocity, rows 3..6 to angular
/// velocity. Used to convert a requested Cartesian speed into the joint speeds it
/// would actually demand, which is what caps a `movel` near a singularity.
pub fn jacobian(dh: &DhParameters, q: &[f64; 6]) -> na::Matrix6<f64> {
    let frames = forward_kinematics_all(dh, q);
    let p_end = frames[5].translation.vector;

    let mut j = na::Matrix6::zeros();
    for i in 0..6 {
        // The axis of joint i is the z of the frame *before* it; for joint 0 that
        // is the base frame itself.
        let (z, p) = if i == 0 {
            (Vector3::z(), Vector3::zeros())
        } else {
            let f = &frames[i - 1];
            (
                f.rotation.transform_vector(&Vector3::z()),
                f.translation.vector,
            )
        };
        let lin = z.cross(&(p_end - p));
        j[(0, i)] = lin.x;
        j[(1, i)] = lin.y;
        j[(2, i)] = lin.z;
        j[(3, i)] = z.x;
        j[(4, i)] = z.y;
        j[(5, i)] = z.z;
    }
    j
}
