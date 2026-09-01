//! Path geometry: resolving waypoints to joint space, and sizing corner blends.
//!
//! Two jobs live here.
//!
//! **Resolving.** A waypoint may be given as joint angles or as a tool pose. Poses
//! are solved here rather than on the robot, so a branch flip or an unreachable
//! point is a plan-time error with a reason instead of a script that aborts on the
//! pendant. Each pose is solved against every branch and the one nearest the
//! caller's preferred configuration - or nearest the previous waypoint, which is
//! what keeps a trajectory from flipping its wrist halfway through - is kept.
//!
//! **Blending.** UR's blend radius is in **metres of tool travel**, even for
//! `movej` (script manual §14.1.22), so sizing it needs forward kinematics rather
//! than joint distances. Two constraints bound it: how far the path may stray from
//! the corner, and the fact that overlapping blend regions make the controller
//! *silently skip the move* with an "Overlapping Blends" warning.

use super::frames::Pose;
use super::kinematics::{
    self, DhParameters, configuration_distance, inverse_kinematics_filtered, select_nearest,
};
use super::limits::Limits;
use k::nalgebra as na;
use na::Vector3;

/// Blend regions closer together than this fraction of the chord are treated as
/// overlapping.
///
/// The manual states that overlapping blends cause the move to be skipped, but
/// never defines the overlap predicate. The working rule is
/// `r_i + r_{i+1} <= chord`, i.e. each radius at most half the chord; 0.4 rather
/// than 0.5 leaves margin against a rule we have inferred rather than read.
const CHORD_FRACTION: f64 = 0.4;

/// Blend radii below this are not worth emitting.
///
/// A sub-millimetre blend does nothing useful and costs a parameter on the move;
/// `r = 0` is cleaner and means the same thing at that scale.
const MIN_USEFUL_BLEND: f64 = 0.001;

/// Beyond this deflection a corner is a reversal and cannot be blended.
const MAX_BLENDABLE_DEFLECTION: f64 = 170.0 * std::f64::consts::PI / 180.0;

/// Why a waypoint could not be turned into a joint configuration.
#[derive(Debug, Clone)]
pub enum ResolveError {
    /// No IK branch at all: the pose is outside the arm's reach.
    Unreachable { index: usize },
    /// Branches exist, but every one of them violates a joint limit.
    OutsideJointLimits { index: usize },
    /// A joint configuration was supplied directly and is outside the limits.
    GivenConfigurationOutsideLimits {
        index: usize,
        joint: usize,
        value: f64,
        min: f64,
        max: f64,
    },
}

impl std::fmt::Display for ResolveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ResolveError::Unreachable { index } => {
                write!(f, "waypoint {index} is outside the arm's reach")
            }
            ResolveError::OutsideJointLimits { index } => write!(
                f,
                "waypoint {index} is reachable, but every inverse-kinematics branch \
                 violates a joint limit"
            ),
            ResolveError::GivenConfigurationOutsideLimits { index, joint, value, min, max } => {
                write!(
                    f,
                    "waypoint {index} sets joint {joint} to {value}, outside [{min}, {max}]"
                )
            }
        }
    }
}

/// Resolve one tool pose to the joint configuration nearest a seed.
///
/// `seed` is the caller's preferred configuration when they gave one, and the
/// previous waypoint's configuration otherwise. Seeding from the previous waypoint
/// is what makes a trajectory continuous: without it, two adjacent poses can land
/// on different IK branches and the arm flips its wrist between them.
pub fn resolve_pose(
    dh: &DhParameters,
    limits: &Limits,
    target: &Pose,
    seed: &[f64; 6],
    index: usize,
) -> Result<[f64; 6], ResolveError> {
    let raw = kinematics::inverse_kinematics(dh, target);
    if raw.is_empty() {
        return Err(ResolveError::Unreachable { index });
    }
    let within =
        inverse_kinematics_filtered(dh, target, &limits.position_min, &limits.position_max);
    if within.is_empty() {
        return Err(ResolveError::OutsideJointLimits { index });
    }
    Ok(select_nearest(&within, seed).expect("non-empty solution set"))
}

/// Check a directly supplied joint configuration against the position limits.
pub fn check_configuration(
    limits: &Limits,
    q: &[f64; 6],
    index: usize,
) -> Result<(), ResolveError> {
    match limits.first_violation(q) {
        None => Ok(()),
        Some((joint, value, min, max)) => Err(ResolveError::GivenConfigurationOutsideLimits {
            index,
            joint,
            value,
            min,
            max,
        }),
    }
}

/// The largest per-joint change between two configurations.
///
/// This is the quantity `movej`'s leading-axis speed is defined against.
pub fn joint_delta(a: &[f64; 6], b: &[f64; 6]) -> [f64; 6] {
    let mut d = [0.0; 6];
    for i in 0..6 {
        d[i] = b[i] - a[i];
    }
    d
}

/// The deflection angle at a corner, in radians.
///
/// Zero means the path continues straight through; `pi` means it reverses. Measured
/// between the incoming and outgoing **Cartesian** directions, because that is the
/// space UR's blend radius lives in.
///
/// Returns `None` when either adjacent segment has no Cartesian length, in which
/// case there is no corner to speak of.
pub fn corner_deflection(prev: &Vector3<f64>, at: &Vector3<f64>, next: &Vector3<f64>) -> Option<f64> {
    let incoming = at - prev;
    let outgoing = next - at;
    let (li, lo) = (incoming.norm(), outgoing.norm());
    if li <= 1e-12 || lo <= 1e-12 {
        return None;
    }
    let cos = (incoming.dot(&outgoing) / (li * lo)).clamp(-1.0, 1.0);
    Some(cos.acos())
}

/// The blend radius to command at a corner, in metres.
///
/// Bounded by three things:
///
/// * **Path deviation.** An arc inscribed in a corner with tangent length `r` and
///   deflection `phi` strays `r * tan(phi/4)` from the corner, so holding the
///   deviation to `max_deviation` means `r <= max_deviation / tan(phi/4)`. UR's
///   blend is not exactly a circular arc, so treat this as a budget rather than a
///   guarantee.
/// * **Overlap.** Blend regions that overlap make the controller skip the move
///   entirely - see [`CHORD_FRACTION`].
/// * **What the caller asked for**, when they pinned a radius.
///
/// A near-reversal returns zero: there is no arc that turns a path back on itself
/// without leaving the corner entirely.
pub fn blend_radius(
    deflection: f64,
    chord_in: f64,
    chord_out: f64,
    max_deviation: f64,
    requested: Option<f64>,
) -> f64 {
    if deflection >= MAX_BLENDABLE_DEFLECTION {
        return 0.0;
    }

    let mut r = CHORD_FRACTION * chord_in.min(chord_out);

    // A straight-through corner has no deviation to budget for, so only the chord
    // and the caller bound it.
    let quarter = (deflection / 4.0).tan();
    if quarter > 1e-9 {
        r = r.min(max_deviation / quarter);
    }

    if let Some(cap) = requested {
        r = r.min(cap.max(0.0));
    }

    if r < MIN_USEFUL_BLEND { 0.0 } else { r }
}

/// Joint rates produced by unit tool speed, sampled along a straight segment.
///
/// Feeds [`super::timing::cartesian_speed_cap`]. Each sample is the joint velocity
/// vector that a tool moving at 1 m/s along the segment would demand at that point,
/// so the smallest limit-respecting scale factor across all samples is the fastest
/// the segment may be run.
///
/// Orientation is interpolated by spherical linear interpolation, matching what a
/// `movel` does between two poses.
///
/// Returns `None` if any sample is unreachable or leaves the joint limits, since a
/// straight line that passes outside the workspace is not a plannable segment even
/// when both of its endpoints are fine.
pub fn sample_linear_segment(
    dh: &DhParameters,
    limits: &Limits,
    from_q: &[f64; 6],
    to_pose: &Pose,
    step: f64,
    max_samples: usize,
) -> Option<Vec<[f64; 6]>> {
    let from_pose = kinematics::forward_kinematics(dh, from_q);
    let distance = (to_pose.translation.vector - from_pose.translation.vector).norm();

    let count = if distance <= step {
        2
    } else {
        ((distance / step).ceil() as usize + 1).min(max_samples)
    };

    let mut rates = Vec::with_capacity(count);
    let mut seed = *from_q;

    for k in 0..count {
        let t = k as f64 / (count - 1) as f64;
        let pose = from_pose.lerp_slerp(to_pose, t);

        let sols = inverse_kinematics_filtered(
            dh,
            &pose,
            &limits.position_min,
            &limits.position_max,
        );
        let q = select_nearest(&sols, &seed)?;

        // A jump between branches mid-line is a wrist flip, not a straight move.
        if k > 0 && configuration_distance(&q, &seed) > std::f64::consts::PI {
            return None;
        }

        let j = kinematics::jacobian(dh, &q);
        let inv = j.try_inverse()?;

        // The twist of a tool moving at unit speed along the line. Angular part is
        // the constant rate slerp implies over the segment, expressed per metre.
        let dir = to_pose.translation.vector - from_pose.translation.vector;
        let dir = if distance > 1e-12 { dir / distance } else { Vector3::zeros() };
        let rot = (to_pose.rotation * from_pose.rotation.inverse()).scaled_axis();
        let omega = if distance > 1e-12 { rot / distance } else { Vector3::zeros() };

        let twist = na::Vector6::new(dir.x, dir.y, dir.z, omega.x, omega.y, omega.z);
        let qd = inv * twist;

        let mut r = [0.0; 6];
        for i in 0..6 {
            r[i] = qd[i];
        }
        rates.push(r);
        seed = q;
    }

    Some(rates)
}
