//! The planner: waypoints in, a blended `movej`/`movel` chain out.
//!
//! The result is deliberately *commandable*: every waypoint carries a velocity and
//! acceleration the controller will honour, and a blend radius that will not make
//! it skip a move. See [`super::timing`] for why the timing is a per-segment
//! closed form rather than a path parameterisation.

use serde::{Deserialize, Serialize};

use super::frames::{FrameConfig, Pose, PoseFrame, pose_from_array};
use super::kinematics::{self, DhParameters};
use super::limits::{Limits, PlanScaling};
use super::path::{self, ResolveError};
use super::timing::{self, DEFAULT_MAX_CARTESIAN_SPEED};

/// How far the tool may stray from a corner when blending, in metres.
pub const DEFAULT_MAX_PATH_DEVIATION: f64 = 0.01;

/// Cartesian sampling step used to check a straight segment, in metres.
pub const LINEAR_SAMPLE_STEP: f64 = 0.005;

/// Samples one straight segment is allowed, however long it is.
pub const MAX_LINEAR_SAMPLES: usize = 2000;

/// Emitted waypoints beyond which a plan is refused.
///
/// Each waypoint becomes an unrolled `movej`/`movel` line plus a validation line
/// and a progress-watch line, and the controller parses the whole program before it
/// moves. This is a legibility and start-up-latency ceiling, not a protocol limit.
pub const MAX_WAYPOINTS: usize = 400;

/// Where a waypoint is, and in what space it was given.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WaypointTarget {
    /// Joint angles, in the order `JOINT_NAMES` gives.
    Joints([f64; 6]),
    /// A tool pose, `[x, y, z, rx, ry, rz]` with a rotation vector.
    Pose {
        /// `[x, y, z, rx, ry, rz]`, rotation vector for the orientation.
        values: [f64; 6],
        #[serde(default = "default_pose_frame")]
        frame: PoseFrame,
    },
}

fn default_pose_frame() -> PoseFrame {
    // Matches what `command_server` hands out: a lookup against `baseframe_id`,
    // which defaults to `base_link`.
    PoseFrame::UrdfBaseLink
}

/// How the arm should travel *to* a waypoint.
///
/// Independent of how the target was specified: a joint-space target can still be
/// reached along a straight tool path, and a pose can still be reached by
/// joint interpolation. Conflating the two is a mistake the existing templates
/// already avoid - `claude_trajectory_unsafe_move_l.script` renders
/// `movel(joint_positions)` precisely because the two are orthogonal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Interpolation {
    /// `movej`: fastest, but the tool traces a curve.
    #[default]
    Joint,
    /// `movel`: the tool travels in a straight line.
    Linear,
}

/// One waypoint as the caller specifies it.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanWaypoint {
    pub target: WaypointTarget,
    /// Which inverse-kinematics branch to prefer. Without one, the branch nearest
    /// the previous waypoint is chosen, which is what keeps the wrist from
    /// flipping mid-path.
    #[serde(default)]
    pub preferred_joint_config: Option<[f64; 6]>,
    /// An upper bound on this corner's blend radius, in metres. The planner may
    /// choose less; it never chooses more.
    #[serde(default)]
    pub blend_radius: Option<f64>,
    #[serde(default)]
    pub interpolation: Interpolation,
}

/// A complete planning problem.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanRequest {
    /// `ROBOT_MODEL`, e.g. `"ur20"`.
    pub robot_model: String,
    /// Where the motion begins - normally the arm's current configuration.
    pub start: WaypointTarget,
    pub waypoints: Vec<PlanWaypoint>,
    #[serde(default)]
    pub scaling: PlanScaling,
    /// Overrides the estimated acceleration limits. See [`Limits`].
    #[serde(default)]
    pub acceleration_max: Option<[f64; 6]>,
    #[serde(default = "default_max_cartesian_speed")]
    pub max_cartesian_speed: f64,
    #[serde(default = "default_max_path_deviation")]
    pub max_path_deviation: f64,
    #[serde(default)]
    pub frame_config: FrameConfig,
}

fn default_max_cartesian_speed() -> f64 {
    DEFAULT_MAX_CARTESIAN_SPEED
}
fn default_max_path_deviation() -> f64 {
    DEFAULT_MAX_PATH_DEVIATION
}

/// One waypoint as the controller will receive it.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PlannedWaypoint {
    pub joint_positions: [f64; 6],
    /// `movej`: leading-axis rad/s. `movel`: tool m/s.
    pub velocity: f64,
    /// `movej`: leading-axis rad/s^2. `movel`: tool m/s^2.
    pub acceleration: f64,
    /// Metres. Zero means "stop here".
    pub blend_radius: f64,
    pub use_linear_motion: bool,
    /// Which joint limited this segment's speed. Diagnostic only.
    pub limiting_joint: usize,
}

/// A planned trajectory, ready to store or to render.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlannedTrajectory {
    pub robot_model: String,
    pub start_joint_positions: [f64; 6],
    pub waypoints: Vec<PlannedWaypoint>,
    /// Seconds. An **estimate**: it sums rest-to-rest segment times, and a blended
    /// segment never comes to rest, so the real motion is faster.
    pub duration_estimate: f64,
    pub limits: Limits,
    /// Things worth telling the caller that did not stop the plan.
    pub warnings: Vec<String>,
}

/// Why a plan could not be produced.
#[derive(Debug, Clone)]
pub enum PlanError {
    UnknownModel(String),
    NoWaypoints,
    TooManyWaypoints { count: usize, max: usize },
    Resolve(ResolveError),
    /// A straight segment leaves the workspace, or crosses a branch flip, between
    /// two endpoints that are themselves fine.
    LinearSegmentNotFollowable { index: usize },
    /// Two adjacent waypoints are the same point.
    DegenerateSegment { index: usize },
}

impl std::fmt::Display for PlanError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanError::UnknownModel(m) => write!(f, "unknown robot model '{m}'"),
            PlanError::NoWaypoints => write!(f, "a trajectory needs at least one waypoint"),
            PlanError::TooManyWaypoints { count, max } => {
                write!(f, "{count} waypoints exceeds the limit of {max}")
            }
            PlanError::Resolve(e) => write!(f, "{e}"),
            PlanError::LinearSegmentNotFollowable { index } => write!(
                f,
                "the straight path to waypoint {index} leaves the workspace or requires \
                 a wrist flip; use joint interpolation or add an intermediate waypoint"
            ),
            PlanError::DegenerateSegment { index } => {
                write!(f, "waypoint {index} is the same configuration as the one before it")
            }
        }
    }
}

impl From<ResolveError> for PlanError {
    fn from(e: ResolveError) -> Self {
        PlanError::Resolve(e)
    }
}

/// Plan a trajectory.
///
/// The returned waypoints do **not** include the start: the arm is already there.
pub fn plan(request: &PlanRequest) -> Result<PlannedTrajectory, PlanError> {
    let dh = DhParameters::for_model(&request.robot_model)
        .ok_or_else(|| PlanError::UnknownModel(request.robot_model.clone()))?;

    if request.waypoints.is_empty() {
        return Err(PlanError::NoWaypoints);
    }
    if request.waypoints.len() > MAX_WAYPOINTS {
        return Err(PlanError::TooManyWaypoints {
            count: request.waypoints.len(),
            max: MAX_WAYPOINTS,
        });
    }

    // Limits come from the model table, are overridden if the caller supplied
    // accelerations, and are then scaled. Scaling is baked into the emitted
    // numbers, which is why `emit` zeroes the template-side scaling fields.
    let base_limits = model_limits(&request.robot_model, &dh)?;
    let base_limits = match request.acceleration_max {
        Some(a) => base_limits.with_acceleration(a),
        None => base_limits,
    };
    let limits = base_limits.scaled(&request.scaling);

    let mut warnings = Vec::new();

    // --- resolve every waypoint to joint space ---
    let start_q = resolve(&request.start, None, &dh, &limits, &request.frame_config, &[0.0; 6], 0)?;
    let mut configs: Vec<[f64; 6]> = Vec::with_capacity(request.waypoints.len() + 1);
    configs.push(start_q);

    for (i, wp) in request.waypoints.iter().enumerate() {
        let seed = wp.preferred_joint_config.unwrap_or(configs[configs.len() - 1]);
        let q = resolve(
            &wp.target,
            wp.preferred_joint_config.as_ref(),
            &dh,
            &limits,
            &request.frame_config,
            &seed,
            i + 1,
        )?;
        configs.push(q);
    }

    // --- Cartesian positions, for chords and blend geometry ---
    let poses: Vec<Pose> = configs
        .iter()
        .map(|q| kinematics::forward_kinematics(&dh, q))
        .collect();
    let points: Vec<_> = poses.iter().map(|p| p.translation.vector).collect();

    // --- per-segment speed ---
    let mut planned: Vec<PlannedWaypoint> = Vec::with_capacity(request.waypoints.len());
    let mut duration = 0.0;

    for (i, wp) in request.waypoints.iter().enumerate() {
        let from = configs[i];
        let to = configs[i + 1];
        let dq = path::joint_delta(&from, &to);
        let chord = (points[i + 1] - points[i]).norm();

        let linear = wp.interpolation == Interpolation::Linear;

        let speed = if linear {
            let rates = path::sample_linear_segment(
                &dh,
                &limits,
                &from,
                &poses[i + 1],
                LINEAR_SAMPLE_STEP,
                MAX_LINEAR_SAMPLES,
            )
            .ok_or(PlanError::LinearSegmentNotFollowable { index: i + 1 })?;
            timing::cartesian_speed_cap(&rates, &limits, request.max_cartesian_speed)
        } else {
            timing::joint_segment_speed(&dq, &limits)
                .ok_or(PlanError::DegenerateSegment { index: i + 1 })?
        };

        // Distance in whatever space this move is timed in.
        let distance = if linear {
            chord
        } else {
            dq.iter().fold(0.0f64, |m, v| m.max(v.abs()))
        };
        duration += timing::trapezoid_duration(distance, speed.velocity, speed.acceleration);

        if linear && chord < 1e-6 {
            warnings.push(format!(
                "waypoint {} asks for a straight move but the tool does not travel; \
                 only the orientation changes",
                i + 1
            ));
        }

        planned.push(PlannedWaypoint {
            joint_positions: to,
            velocity: speed.velocity,
            acceleration: speed.acceleration,
            // Filled in below, once the following segment is known.
            blend_radius: 0.0,
            use_linear_motion: linear,
            limiting_joint: speed.limiting_joint,
        });
    }

    // --- corner blends ---
    //
    // The blend at waypoint i is a property of the *corner* there, so it needs the
    // segment after it as well. The final waypoint keeps r = 0 so the motion ends
    // at rest.
    for i in 0..planned.len().saturating_sub(1) {
        let deflection = match path::corner_deflection(&points[i], &points[i + 1], &points[i + 2]) {
            Some(d) => d,
            None => continue,
        };
        let chord_in = (points[i + 1] - points[i]).norm();
        let chord_out = (points[i + 2] - points[i + 1]).norm();
        let r = path::blend_radius(
            deflection,
            chord_in,
            chord_out,
            request.max_path_deviation,
            request.waypoints[i].blend_radius,
        );
        if r == 0.0 && deflection < std::f64::consts::PI / 2.0 {
            warnings.push(format!(
                "waypoint {} cannot be blended, so the arm will stop there",
                i + 1
            ));
        }
        planned[i].blend_radius = r;
    }

    Ok(PlannedTrajectory {
        robot_model: request.robot_model.clone(),
        start_joint_positions: start_q,
        waypoints: planned,
        duration_estimate: duration,
        limits,
        warnings,
    })
}

/// Limits for a model: from the vendored URDF when it is there, else estimated.
fn model_limits(model: &str, _dh: &DhParameters) -> Result<Limits, PlanError> {
    let urdf = format!("src/ur_description/urdf/{model}.urdf");
    if let Ok(l) = Limits::from_urdf(&urdf) {
        return Ok(l);
    }
    if let Ok(dir) = std::env::var("UR_DESCRIPTION_DIR") {
        let alt = format!("{}/urdf/{}.urdf", dir.trim_end_matches('/'), model);
        if let Ok(l) = Limits::from_urdf(&alt) {
            return Ok(l);
        }
    }
    Err(PlanError::UnknownModel(model.to_string()))
}

fn resolve(
    target: &WaypointTarget,
    preferred: Option<&[f64; 6]>,
    dh: &DhParameters,
    limits: &Limits,
    frames: &FrameConfig,
    seed: &[f64; 6],
    index: usize,
) -> Result<[f64; 6], PlanError> {
    match target {
        WaypointTarget::Joints(q) => {
            path::check_configuration(limits, q, index)?;
            Ok(*q)
        }
        WaypointTarget::Pose { values, frame } => {
            let p = pose_from_array(*values);
            let in_ur_base = match frame {
                PoseFrame::UrBase => p,
                PoseFrame::UrdfBaseLink => frames.base_link_to_ur_base(&p),
            };
            let seed = preferred.copied().unwrap_or(*seed);
            Ok(path::resolve_pose(dh, limits, &in_ur_base, &seed, index)?)
        }
    }
}
