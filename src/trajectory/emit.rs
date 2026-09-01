//! Turning a [`PlannedTrajectory`] into the driver's own command types.
//!
//! # The double-scaling trap
//!
//! The planner bakes `global_velocity_scaling` and `global_acceleration_scaling`
//! into the `velocity` and `acceleration` it emits. The `claude_*` templates *also*
//! apply those fields, so passing them through would square the scaling and produce
//! an arm that crawls. Every emitted waypoint therefore sets both to `0.0`, which
//! is precisely the value those templates read as "no scaling" - so a planned
//! trajectory is safe even if it is pointed at one of the older templates.
//! `tests/trajectory_planning.rs` asserts this.

use crate::core::structs::{RobotCommand, Waypoint};

use super::plan::{PlannedTrajectory, PlannedWaypoint};

/// The value the templates read as "do not scale".
///
/// Their guard is `if scaling > 0.0 and scaling <= 1.0`, so anything outside that
/// interval means no scaling. Zero is what `command_server` itself defaults to.
const NO_TEMPLATE_SCALING: f64 = 0.0;

/// Convert one planned waypoint into the driver's `Waypoint`.
fn to_waypoint(wp: &PlannedWaypoint, tcp_in_faceplate: &str, payload: &str) -> Waypoint {
    Waypoint {
        acceleration: wp.acceleration,
        velocity: wp.velocity,
        // See the module docs: scaling is already applied.
        global_acceleration_scaling: NO_TEMPLATE_SCALING,
        global_velocity_scaling: NO_TEMPLATE_SCALING,
        // Never `t=`: it overrides `a` and `v` and is unbounded, so a duration the
        // joints cannot meet faults rather than saturating. See `super::timing`.
        use_execution_time: false,
        execution_time: 0.0,
        use_blend_radius: wp.blend_radius > 0.0,
        blend_radius: wp.blend_radius,
        // The planner has already solved every pose, so the controller never runs
        // inverse kinematics for a planned trajectory. That removes the branch
        // nondeterminism `CLAUDE_TEMPLATES.md` documents, and it is what lets the
        // template skip its per-waypoint IK validation.
        use_joint_positions: true,
        joint_positions: wp.joint_positions.to_vec(),
        use_preferred_joint_config: false,
        preferred_joint_config: vec![0.0; 6],
        use_payload: !payload.is_empty(),
        payload: payload.to_string(),
        // Unused when `use_joint_positions` is set, but the template still
        // references it, so it must be a well-formed pose literal rather than an
        // empty string.
        target_in_base: "p[0,0,0,0,0,0]".to_string(),
        relative_pose: vec![0.0; 6],
        tcp_in_faceplate: tcp_in_faceplate.to_string(),
        force_threshold: 0.0,
        use_linear_motion: wp.use_linear_motion,
    }
}

/// Convert a whole trajectory into waypoints.
pub fn to_waypoints(
    trajectory: &PlannedTrajectory,
    tcp_in_faceplate: &str,
    payload: &str,
) -> Vec<Waypoint> {
    trajectory
        .waypoints
        .iter()
        .map(|w| to_waypoint(w, tcp_in_faceplate, payload))
        .collect()
}

/// Build a complete `RobotCommand` for a planned trajectory.
///
/// `command_type` selects the template, so it must name a file in `templates/`.
pub fn to_robot_command(
    trajectory: &PlannedTrajectory,
    command_type: &str,
    tcp_in_faceplate: &str,
    payload: &str,
    report_waypoint_progress: bool,
) -> RobotCommand {
    let waypoints = to_waypoints(trajectory, tcp_in_faceplate, payload);
    RobotCommand {
        command_type: command_type.to_string(),
        // The command-level speed fields are unused by a trajectory template, which
        // reads them per waypoint. They are filled with the first segment's values
        // rather than zero so that anything logging the command sees something
        // meaningful.
        acceleration: waypoints.first().map(|w| w.acceleration).unwrap_or(0.0),
        velocity: waypoints.first().map(|w| w.velocity).unwrap_or(0.0),
        global_acceleration_scaling: NO_TEMPLATE_SCALING,
        global_velocity_scaling: NO_TEMPLATE_SCALING,
        use_execution_time: false,
        execution_time: 0.0,
        use_blend_radius: false,
        blend_radius: 0.0,
        use_joint_positions: true,
        joint_positions: trajectory.start_joint_positions.to_vec(),
        use_preferred_joint_config: false,
        preferred_joint_config: vec![0.0; 6],
        use_payload: !payload.is_empty(),
        payload: payload.to_string(),
        target_in_base: "p[0,0,0,0,0,0]".to_string(),
        tcp_in_faceplate: tcp_in_faceplate.to_string(),
        force_threshold: 0.0,
        relative_pose: vec![0.0; 6],
        report_waypoint_progress,
        trajectory_id: String::new(),
        waypoints,
    }
}
