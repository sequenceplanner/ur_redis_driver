//! Per-segment speed and acceleration, at the joint limits.
//!
//! # Why there is no time-optimal path parameterisation here
//!
//! The obvious thing to build is a TOPP integrator (Kunz & Stilman, what MoveIt
//! ships as TOTG): a velocity profile along the path that saturates the joint
//! limits everywhere. It was deliberately not built, because **its output cannot
//! be commanded through a blended `movej` chain**:
//!
//! * `movej`'s `t=` parameter overrides `a` and `v` entirely (script manual
//!   §14.1.22) and is unbounded - a `t` shorter than the joints can manage faults
//!   rather than saturating. It is not a safe way to impose a timing law.
//! * With `r=`, the controller merges the deceleration of one move into the
//!   acceleration of the next and **chooses the junction velocity itself**. A TOPP
//!   profile is therefore re-timed before it executes.
//! * Joint-linear segments have zero curvature, so the only place a TOPP profile
//!   differs from a per-segment peak speed *is* those junctions - exactly the part
//!   that is not ours to set.
//!
//! What is both optimal and commandable has a closed form, and that is what this
//! module computes. See [`joint_segment_speed`].
//!
//! Executing a genuine TOPP profile would need an unrolled `servoj` block at the
//! controller's own rate, which is a different execution model from the one this
//! driver uses.

use super::limits::Limits;
use serde::{Deserialize, Serialize};

/// Joint motions below this are treated as "this joint does not move".
///
/// Guards the division in [`joint_segment_speed`]: a joint that barely moves would
/// otherwise divide by near-zero and drive the whole segment's speed to zero.
const MOTIONLESS_JOINT: f64 = 1e-9;

/// The speed and acceleration to command for one segment, and which joint set them.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SegmentSpeed {
    /// `movej`: joint speed of the leading axis, rad/s. `movel`: tool speed, m/s.
    pub velocity: f64,
    /// `movej`: joint acceleration of the leading axis, rad/s^2. `movel`: m/s^2.
    pub acceleration: f64,
    /// The joint that limited `velocity`. Reported so a slow segment can be
    /// explained rather than guessed at.
    pub limiting_joint: usize,
}

/// The fastest legal `movej` for a joint-space displacement.
///
/// `movej`'s `v` and `a` are the *leading axis* values: the controller scales every
/// joint so they all start and stop together, so joint `i` peaks at
/// `v * |dq_i| / max_j |dq_j|`. Requiring that to stay under `vmax_i` for every
/// joint gives
///
/// ```text
/// dmax = max_j |dq_j|
/// v*   = min over moving i of ( vmax_i * dmax / |dq_i| )
/// a*   = min over moving i of ( amax_i * dmax / |dq_i| )
/// ```
///
/// which is **tight**: at least one joint ends up exactly on its limit. That is the
/// whole gain over what the driver does today, where one global `velocity` covers
/// the entire path and therefore every segment runs at the slowest joint's limit.
/// On a UR20 a wrist-led segment gets 3.665 rad/s instead of 2.094.
///
/// Returns `None` for a displacement of nothing, which has no meaningful speed.
pub fn joint_segment_speed(dq: &[f64; 6], limits: &Limits) -> Option<SegmentSpeed> {
    let dmax = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if dmax <= MOTIONLESS_JOINT {
        return None;
    }

    let mut velocity = f64::INFINITY;
    let mut acceleration = f64::INFINITY;
    let mut limiting_joint = 0usize;

    for i in 0..6 {
        let travel = dq[i].abs();
        if travel <= MOTIONLESS_JOINT {
            continue;
        }
        let ratio = dmax / travel;
        let v = limits.velocity_max[i] * ratio;
        if v < velocity {
            velocity = v;
            limiting_joint = i;
        }
        acceleration = acceleration.min(limits.acceleration_max[i] * ratio);
    }

    if !velocity.is_finite() || !acceleration.is_finite() {
        return None;
    }
    Some(SegmentSpeed { velocity, acceleration, limiting_joint })
}

/// The fastest legal tool speed for a Cartesian segment.
///
/// A `movel` holds the *tool* speed constant, so the joint speeds it demands vary
/// along the line and blow up near a singularity. The controller does **not** cap
/// this for you: it will happily ask a wrist for a rate it cannot deliver, and the
/// result is a speed-limit fault or a lurch.
///
/// `joint_rates` is one entry per sample along the segment, each holding the joint
/// rates produced by a *unit* tool speed at that sample - i.e. `J^-1` applied to
/// the unit path twist. The cap is then the smallest tool speed that keeps every
/// joint inside its limit at every sample.
///
/// `ceiling` caps the result regardless, since an unconstrained straight line in
/// free space would otherwise be allowed to run as fast as the joints permit.
pub fn cartesian_speed_cap(
    joint_rates: &[[f64; 6]],
    limits: &Limits,
    ceiling: f64,
) -> SegmentSpeed {
    let mut velocity = ceiling;
    let mut limiting_joint = 0usize;

    for rates in joint_rates {
        for i in 0..6 {
            let rate = rates[i].abs();
            if rate <= MOTIONLESS_JOINT {
                continue;
            }
            let v = limits.velocity_max[i] / rate;
            if v < velocity {
                velocity = v;
                limiting_joint = i;
            }
        }
    }

    // The acceleration ceiling is scaled by the same factor the velocity was, so a
    // line that had to be slowed for a singularity also ramps more gently into it.
    let ratio = if ceiling > 0.0 { velocity / ceiling } else { 1.0 };
    SegmentSpeed {
        velocity,
        acceleration: default_cartesian_acceleration(limits) * ratio,
        limiting_joint,
    }
}

/// A tool-space acceleration ceiling, in m/s^2.
///
/// There is no published Cartesian acceleration limit either, and the joint-space
/// estimate does not convert without a Jacobian at every point. This uses the same
/// ramp-time assumption as the joint limits, applied to the Cartesian speed
/// ceiling, and stays close to URScript's own `movel a=1.2` default.
pub fn default_cartesian_acceleration(_limits: &Limits) -> f64 {
    DEFAULT_MAX_CARTESIAN_SPEED / super::limits::RAMP_TIME_S
}

/// Tool speed a straight segment is never allowed to exceed, m/s.
///
/// Not a robot limit - a sanity ceiling. A UR20 in free space can move its flange
/// faster than is comfortable to stand next to, and a Cartesian move is usually
/// near something worth not hitting.
pub const DEFAULT_MAX_CARTESIAN_SPEED: f64 = 0.5;

/// How long a rest-to-rest trapezoid over `distance` takes at peak `v` and ramp `a`.
///
/// Used only to estimate a duration for the whole trajectory. It is an *estimate*:
/// a blended segment never comes to rest at its far end, so the real motion is
/// faster than the sum of these. Reported as such rather than as a promise.
pub fn trapezoid_duration(distance: f64, v: f64, a: f64) -> f64 {
    let d = distance.abs();
    if d <= 0.0 || v <= 0.0 || a <= 0.0 {
        return 0.0;
    }
    let ramp_distance = v * v / a; // both ramps together
    if ramp_distance >= d {
        // Triangular: the segment ends before the peak speed is reached.
        2.0 * (d / a).sqrt()
    } else {
        2.0 * (v / a) + (d - ramp_distance) / v
    }
}

/// The peak speed actually reached over `distance`, which may be below `v`.
///
/// A short segment never gets to its commanded speed. Knowing the real peak is what
/// lets the corner-speed and blend-radius logic size a blend for the speed the arm
/// will actually be carrying rather than the one it was allowed.
pub fn achievable_peak_speed(distance: f64, v: f64, a: f64) -> f64 {
    let d = distance.abs();
    if d <= 0.0 || a <= 0.0 {
        return 0.0;
    }
    v.min((a * d).sqrt())
}
