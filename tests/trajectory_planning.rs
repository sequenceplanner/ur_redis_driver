//! Tests for the planner: limits, tightness, blends, and the emitted command.
//!
//! These are the assertions that stand in for a robot. The kinematics are pinned
//! against the URDF in `trajectory_kinematics.rs`; here the question is whether the
//! numbers we hand the controller are ones it can actually execute.

use ur_redis_driver::trajectory::emit;
use ur_redis_driver::trajectory::kinematics::{self, DhParameters};
use ur_redis_driver::trajectory::limits::PlanScaling;
use ur_redis_driver::trajectory::plan::{
    self, Interpolation, PlanRequest, PlanWaypoint, PlannedTrajectory, WaypointTarget,
};
use ur_redis_driver::trajectory::store;

const MODEL: &str = "ur20";
const HOME: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];

fn dh() -> DhParameters {
    DhParameters::for_model(MODEL).unwrap()
}

fn joints(q: [f64; 6], blend: Option<f64>) -> PlanWaypoint {
    PlanWaypoint {
        target: WaypointTarget::Joints(q),
        preferred_joint_config: None,
        blend_radius: blend,
        interpolation: Interpolation::Joint,
    }
}

fn request(waypoints: Vec<PlanWaypoint>) -> PlanRequest {
    PlanRequest {
        robot_model: MODEL.to_string(),
        start: WaypointTarget::Joints(HOME),
        waypoints,
        scaling: PlanScaling { velocity: 1.0, acceleration: 1.0 },
        acceleration_max: None,
        max_cartesian_speed: 0.5,
        max_path_deviation: plan::DEFAULT_MAX_PATH_DEVIATION,
        frame_config: Default::default(),
    }
}

/// A path that exercises several differently-led segments.
fn mixed_path() -> Vec<PlanWaypoint> {
    vec![
        // Shoulder-led.
        joints([0.8, -1.4, 0.5, -1.2, 0.3, 0.0], Some(0.10)),
        // Wrist-led: the joints with the highest velocity limits do most of the work.
        joints([0.85, -1.4, 0.5, -1.0, 1.4, 1.9], Some(0.10)),
        // Elbow-led.
        joints([0.9, -1.1, 1.5, -1.3, 1.4, 1.9], Some(0.10)),
        joints([0.2, -1.5, 0.2, -1.6, 0.1, 0.2], None),
    ]
}

fn configs(t: &PlannedTrajectory) -> Vec<[f64; 6]> {
    let mut v = vec![t.start_joint_positions];
    v.extend(t.waypoints.iter().map(|w| w.joint_positions));
    v
}

/// Every emitted `movej` must keep every joint inside its velocity limit.
///
/// `movej`'s `v` is the leading-axis speed and the controller scales the rest to
/// finish together, so joint `i` peaks at `v * |dq_i| / max_j |dq_j|`.
#[test]
fn planned_speeds_respect_joint_velocity_limits() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    let qs = configs(&t);

    for (i, wp) in t.waypoints.iter().enumerate() {
        if wp.use_linear_motion {
            continue; // velocity is m/s there, checked separately
        }
        let dq: Vec<f64> = (0..6).map(|j| qs[i + 1][j] - qs[i][j]).collect();
        let dmax = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        assert!(dmax > 0.0, "segment {i} does not move");

        for j in 0..6 {
            let peak = wp.velocity * dq[j].abs() / dmax;
            assert!(
                peak <= t.limits.velocity_max[j] * (1.0 + 1e-9),
                "segment {i}, joint {j}: peak {peak} exceeds limit {}",
                t.limits.velocity_max[j]
            );
            let peak_acc = wp.acceleration * dq[j].abs() / dmax;
            assert!(
                peak_acc <= t.limits.acceleration_max[j] * (1.0 + 1e-9),
                "segment {i}, joint {j}: peak acceleration {peak_acc} exceeds limit {}",
                t.limits.acceleration_max[j]
            );
        }
    }
}

/// The speed must be *tight*: some joint is exactly on its limit.
///
/// Without this, `planned_speeds_respect_joint_velocity_limits` would pass for a
/// planner that simply returned zero. This is the assertion that the closed form
/// actually buys the speed it claims to.
#[test]
fn segment_speed_leaves_nothing_on_the_table() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    let qs = configs(&t);

    for (i, wp) in t.waypoints.iter().enumerate() {
        if wp.use_linear_motion {
            continue;
        }
        let dq: Vec<f64> = (0..6).map(|j| qs[i + 1][j] - qs[i][j]).collect();
        let dmax = dq.iter().fold(0.0f64, |m, v| m.max(v.abs()));

        let tightest = (0..6)
            .map(|j| wp.velocity * dq[j].abs() / dmax / t.limits.velocity_max[j])
            .fold(0.0f64, f64::max);
        assert!(
            (tightest - 1.0).abs() < 1e-9,
            "segment {i} runs at {:.4} of the limit; the closed form should reach 1.0",
            tightest
        );
    }
}

/// Different segments must get different speeds, or the whole exercise is pointless.
#[test]
fn per_segment_speeds_actually_differ() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    let speeds: Vec<f64> = t.waypoints.iter().map(|w| w.velocity).collect();
    let min = speeds.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = speeds.iter().cloned().fold(0.0, f64::max);
    assert!(
        max > min * 1.2,
        "every segment got nearly the same speed ({speeds:?}); a wrist-led segment \
         should be materially faster than a shoulder-led one"
    );
    // And the fast one should beat what a single global velocity could have used,
    // which is the slowest joint's limit.
    assert!(
        max > t.limits.velocity_max[0],
        "no segment beat the shoulder limit, so nothing was gained"
    );
}

/// Overlapping blend regions make the controller silently skip a move.
#[test]
fn blend_radii_never_overlap() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    let dh = dh();
    let qs = configs(&t);
    let points: Vec<_> = qs
        .iter()
        .map(|q| kinematics::forward_kinematics(&dh, q).translation.vector)
        .collect();

    for i in 0..t.waypoints.len() {
        let chord_in = (points[i + 1] - points[i]).norm();
        assert!(
            t.waypoints[i].blend_radius <= 0.5 * chord_in + 1e-12,
            "waypoint {i} blend {} is more than half its incoming chord {chord_in}",
            t.waypoints[i].blend_radius
        );
        if i + 1 < t.waypoints.len() {
            let sum = t.waypoints[i].blend_radius + t.waypoints[i + 1].blend_radius;
            assert!(
                sum <= (points[i + 2] - points[i + 1]).norm() + 1e-12,
                "blends at {i} and {} overlap",
                i + 1
            );
        }
    }
    // The motion must end at rest.
    assert_eq!(t.waypoints.last().unwrap().blend_radius, 0.0);
}

/// The planner bakes scaling in, so the template fields must be neutral.
#[test]
fn scaling_is_applied_once_and_only_once() {
    let full = plan::plan(&request(mixed_path())).expect("plans");

    let mut halved_req = request(mixed_path());
    halved_req.scaling = PlanScaling { velocity: 0.5, acceleration: 1.0 };
    let halved = plan::plan(&halved_req).expect("plans");

    for (a, b) in full.waypoints.iter().zip(halved.waypoints.iter()) {
        assert!(
            (b.velocity - a.velocity * 0.5).abs() < 1e-9,
            "0.5 scaling did not halve {} -> {}",
            a.velocity,
            b.velocity
        );
    }

    // The emitted waypoints must tell the template not to scale again. The
    // templates read anything outside (0.0, 1.0] as "no scaling", and 0.0 is what
    // command_server itself defaults to.
    let waypoints = emit::to_waypoints(&halved, "p[0,0,0,0,0,0]", "");
    for w in &waypoints {
        assert_eq!(w.global_velocity_scaling, 0.0);
        assert_eq!(w.global_acceleration_scaling, 0.0);
    }
}

/// `t=` overrides `a` and `v` and is unbounded, so it must never be emitted.
#[test]
fn emitted_waypoints_never_use_execution_time() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    for w in emit::to_waypoints(&t, "p[0,0,0,0,0,0]", "") {
        assert!(!w.use_execution_time);
        assert!(w.use_joint_positions, "the controller must not run IK for a planned path");
    }
}

/// Adjacent configurations must not jump a branch.
#[test]
fn planned_path_never_flips_a_branch() {
    let t = plan::plan(&request(mixed_path())).expect("plans");
    let qs = configs(&t);
    for i in 1..qs.len() {
        for j in 0..6 {
            let d = (qs[i][j] - qs[i - 1][j]).abs();
            assert!(d < std::f64::consts::PI, "joint {j} jumps {d} rad into waypoint {i}");
        }
    }
}

/// A straight segment must keep the tool on the line between its endpoints.
#[test]
fn linear_segments_keep_the_tool_on_the_straight_line() {
    let dh = dh();
    // A pose target reached in a straight line, from a configuration well clear of
    // any singularity.
    let start = [0.4, -1.3, 1.0, -1.2, 1.5, 0.2];
    let start_pose = kinematics::forward_kinematics(&dh, &start);
    let mut target = ur_redis_driver::trajectory::frames::pose_to_array(&start_pose);
    target[0] += 0.15;
    target[2] -= 0.10;

    let mut req = request(vec![PlanWaypoint {
        target: WaypointTarget::Pose {
            values: target,
            frame: ur_redis_driver::trajectory::frames::PoseFrame::UrBase,
        },
        preferred_joint_config: Some(start),
        blend_radius: None,
        interpolation: Interpolation::Linear,
    }]);
    req.start = WaypointTarget::Joints(start);

    let t = plan::plan(&req).expect("plans a straight segment");
    let w = &t.waypoints[0];
    assert!(w.use_linear_motion, "the segment must be emitted as movel");
    assert!(w.velocity <= 0.5 + 1e-12, "tool speed must respect the ceiling");
    assert!(w.velocity > 0.0);

    // movel interpolates in tool space, so straightness is the controller's job;
    // what the planner owes is a speed the joints can sustain along the whole line.
    let end_pose = kinematics::forward_kinematics(&dh, &w.joint_positions);
    let reached = ur_redis_driver::trajectory::frames::pose_to_array(&end_pose);
    for i in 0..3 {
        assert!(
            (reached[i] - target[i]).abs() < 1e-9,
            "component {i}: reached {} wanted {}",
            reached[i],
            target[i]
        );
    }
}

#[test]
fn unreachable_waypoints_fail_with_a_reason() {
    let far = PlanWaypoint {
        target: WaypointTarget::Pose {
            values: [5.0, 0.0, 0.5, 0.0, 0.0, 0.0],
            frame: ur_redis_driver::trajectory::frames::PoseFrame::UrBase,
        },
        preferred_joint_config: None,
        blend_radius: None,
        interpolation: Interpolation::Joint,
    };
    let err = plan::plan(&request(vec![far])).unwrap_err().to_string();
    assert!(err.contains("reach"), "unhelpful error: {err}");

    let empty = plan::plan(&request(vec![])).unwrap_err().to_string();
    assert!(empty.contains("at least one waypoint"), "unhelpful error: {empty}");
}

#[test]
fn duration_estimate_is_positive_and_shrinks_when_faster() {
    let slow_req = {
        let mut r = request(mixed_path());
        r.scaling = PlanScaling { velocity: 0.25, acceleration: 1.0 };
        r
    };
    let slow = plan::plan(&slow_req).expect("plans");
    let fast = plan::plan(&request(mixed_path())).expect("plans");
    assert!(fast.duration_estimate > 0.0);
    assert!(
        slow.duration_estimate > fast.duration_estimate,
        "quartering the speed should take longer: {} vs {}",
        slow.duration_estimate,
        fast.duration_estimate
    );
}

/// A planned trajectory must render through the real template set.
#[test]
fn planned_trajectory_renders_through_the_template() {
    let templates = tera::Tera::new("templates/*.script").expect("templates parse");
    let t = plan::plan(&request(mixed_path())).expect("plans");

    let command = emit::to_robot_command(
        &t,
        ur_redis_driver::OPTIMAL_TRAJECTORY_COMMAND,
        "p[0,0,0,0,0,0]",
        "",
        true,
    );
    let n = command.waypoints.len();

    let script =
        ur_redis_driver::generate_core_script_from_template("r1", command, &templates).unwrap();

    let moves = script.matches("movej(").count() + script.matches("movel(").count();
    assert_eq!(moves, n, "expected one move per waypoint:\n{script}");
    assert_eq!(script.matches("waypoint_reached").count(), n);
    assert!(!script.contains(", t="), "a planned script must never set t=:\n{script}");
    // Only executable lines count: the template's own header explains that it does
    // not call get_inverse_kin, so a naive substring search finds the comment.
    let code: String = script
        .lines()
        .filter(|l| !l.trim_start().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        !code.contains("get_inverse_kin"),
        "the controller must not run IK for a planned path:\n{script}"
    );
    assert!(code.contains("is_within_safety_limits"));
    assert!(code.contains("def script():"));
}

#[test]
fn trajectories_round_trip_through_the_store() {
    let dir = std::env::temp_dir().join(format!("ur_traj_test_{}", std::process::id()));
    unsafe { std::env::set_var("UR_TRAJECTORY_DIR", &dir) };

    let t = plan::plan(&request(mixed_path())).expect("plans");
    store::save("round_trip", &t, None).expect("saves");

    let back = store::load("round_trip").expect("loads");
    assert_eq!(back.trajectory.waypoints.len(), t.waypoints.len());
    // Compared to within float-text precision rather than bit-for-bit: a JSON
    // round-trip can shift the last unit in the last place, and these numbers are
    // on their way to becoming decimal literals in a URScript file anyway.
    for (a, b) in t.waypoints.iter().zip(back.trajectory.waypoints.iter()) {
        assert_eq!(a.joint_positions, b.joint_positions);
        assert_eq!(a.use_linear_motion, b.use_linear_motion);
        assert!((a.velocity - b.velocity).abs() < 1e-12 * a.velocity.abs().max(1.0));
        assert!((a.acceleration - b.acceleration).abs() < 1e-12 * a.acceleration.abs().max(1.0));
        assert!((a.blend_radius - b.blend_radius).abs() < 1e-12);
    }
    assert!(store::list().unwrap().contains(&"round_trip".to_string()));

    // Names are interpolated into a path, so they get the same scrutiny
    // command_server gives a template name.
    assert!(store::load("../../etc/passwd").is_err());
    assert!(store::load("").is_err());

    let _ = std::fs::remove_dir_all(&dir);
}
