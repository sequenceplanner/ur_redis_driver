//! The bridge between a Redis motion request and the planner.
//!
//! Kept in its own file so that wiring the planner into `command_server` is a
//! handful of lines there rather than a rewrite.

use crate::core::structs::{RobotCommand, Waypoint};

use super::emit;
use super::frames::{FrameConfig, PoseFrame, parse_pose_string};
use super::limits::PlanScaling;
use super::plan::{
    self, Interpolation, PlanRequest, PlanWaypoint, PlannedTrajectory, WaypointTarget,
};
use super::store;

/// The `command_type` that runs a planned trajectory.
///
/// Dispatch is by template filename, so this must match
/// `templates/optimal_trajectory.script`.
pub const OPTIMAL_TRAJECTORY_COMMAND: &str = "optimal_trajectory";

/// Whether a command should go through the planner.
pub fn is_planner_command(command_type: &str) -> bool {
    command_type == OPTIMAL_TRAJECTORY_COMMAND
}

/// The robot model to plan for.
///
/// Read from the environment the same way `main.rs` reads it, rather than threaded
/// through `command_server`'s signature, to keep the wiring additive. If the two
/// ever disagree the planner is planning for a different arm than the driver is
/// driving, so they read the same variable with the same default.
pub fn robot_model_from_env() -> String {
    std::env::var("ROBOT_MODEL").unwrap_or_else(|_| "ur20".to_string())
}

/// Produce the waypoints a planned-trajectory request should actually execute.
///
/// Two paths, chosen by `trajectory_id`:
///
/// * **empty** - plan now from the waypoints the request carries, starting from
///   where the arm currently is.
/// * **set** - load the trajectory of that name from disk and run it.
///
/// A stored trajectory is *not* re-planned from the current position: it was
/// planned from a specific start, and silently re-aiming it would change the path
/// an operator reviewed. Instead the caller is told if the arm is not where the
/// trajectory begins.
pub fn waypoints_for_command(
    command: &RobotCommand,
    current_joints: &[f64],
) -> Result<(Vec<Waypoint>, PlannedTrajectory), String> {
    let trajectory = if command.trajectory_id.is_empty() {
        plan_from_command(command, current_joints)?
    } else {
        let stored = store::load(&command.trajectory_id)?;
        check_start_matches(&stored.trajectory, current_joints)?;
        stored.trajectory
    };

    let waypoints = emit::to_waypoints(
        &trajectory,
        &command.tcp_in_faceplate,
        if command.use_payload { &command.payload } else { "" },
    );
    Ok((waypoints, trajectory))
}

/// How far a joint may be from a stored trajectory's start before it is refused.
///
/// A blended trajectory starting from the wrong place traces a different path, so
/// this is a correctness guard, not a convenience. Generous enough to tolerate the
/// arm settling after a previous move.
pub const START_TOLERANCE_RAD: f64 = 0.05;

fn check_start_matches(
    trajectory: &PlannedTrajectory,
    current_joints: &[f64],
) -> Result<(), String> {
    if current_joints.len() < 6 {
        // No joint state yet - the realtime stream has not delivered one. Refusing
        // is safer than assuming the arm is where the trajectory wants it.
        return Err("no joint state available to check the trajectory's start against".to_string());
    }
    for i in 0..6 {
        let d = (current_joints[i] - trajectory.start_joint_positions[i]).abs();
        if d > START_TOLERANCE_RAD {
            return Err(format!(
                "the arm is not at this trajectory's start: joint {i} is {:.4} rad away \
                 (tolerance {START_TOLERANCE_RAD}). Move there first, or re-plan.",
                d
            ));
        }
    }
    Ok(())
}

/// Build a [`PlanRequest`] out of a Redis-sourced command and plan it.
fn plan_from_command(
    command: &RobotCommand,
    current_joints: &[f64],
) -> Result<PlannedTrajectory, String> {
    if command.waypoints.is_empty() {
        return Err("a planned trajectory needs at least one waypoint".to_string());
    }
    if current_joints.len() < 6 {
        return Err("no joint state available to plan from".to_string());
    }

    let mut start = [0.0f64; 6];
    start.copy_from_slice(&current_joints[..6]);

    let mut waypoints = Vec::with_capacity(command.waypoints.len());
    for (i, wp) in command.waypoints.iter().enumerate() {
        waypoints.push(PlanWaypoint {
            target: waypoint_target(wp, i)?,
            preferred_joint_config: if wp.use_preferred_joint_config {
                Some(to_six(&wp.preferred_joint_config, i, "preferred_joint_config")?)
            } else {
                None
            },
            blend_radius: if wp.use_blend_radius { Some(wp.blend_radius) } else { None },
            interpolation: if wp.use_linear_motion {
                Interpolation::Linear
            } else {
                Interpolation::Joint
            },
        });
    }

    let request = PlanRequest {
        robot_model: robot_model_from_env(),
        start: WaypointTarget::Joints(start),
        waypoints,
        scaling: PlanScaling {
            velocity: command.global_velocity_scaling,
            acceleration: command.global_acceleration_scaling,
        },
        acceleration_max: None,
        max_cartesian_speed: super::timing::DEFAULT_MAX_CARTESIAN_SPEED,
        max_path_deviation: plan::DEFAULT_MAX_PATH_DEVIATION,
        frame_config: FrameConfig::default(),
    };

    plan::plan(&request).map_err(|e| e.to_string())
}

fn waypoint_target(wp: &Waypoint, index: usize) -> Result<WaypointTarget, String> {
    if wp.use_joint_positions {
        Ok(WaypointTarget::Joints(to_six(&wp.joint_positions, index, "joint_positions")?))
    } else {
        let pose = parse_pose_string(&wp.target_in_base)
            .map_err(|e| format!("waypoint {index}: {e}"))?;
        Ok(WaypointTarget::Pose {
            values: pose,
            // `command_server` looks goal frames up against `baseframe_id`, which
            // defaults to `base_link`.
            frame: PoseFrame::UrdfBaseLink,
        })
    }
}

fn to_six(v: &[f64], index: usize, field: &str) -> Result<[f64; 6], String> {
    if v.len() != 6 {
        return Err(format!(
            "waypoint {index}: {field} has {} values, expected 6",
            v.len()
        ));
    }
    let mut out = [0.0; 6];
    out.copy_from_slice(&v[..6]);
    Ok(out)
}
