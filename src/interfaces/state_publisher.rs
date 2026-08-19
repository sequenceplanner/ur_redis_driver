use k::nalgebra::{self, Quaternion};
use k::{Isometry3, Vector3};
// use micro_sp::management::transforms;
use micro_sp::*;
use ordered_float::OrderedFloat;
use crate::{DriverState, URDFParameters, lock_driver_state, robot_mode_name, safety_mode_name};
use roxmltree::Document;
use std::collections::HashMap;
use std::fs;
use std::sync::{Arc, Mutex};
use std::time::{Duration, SystemTime};
use tokio::time::{MissedTickBehavior, interval};

use crate::ROBOT_STATE_UPDATE_INTERVAL_MS;

pub async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
    robot_params: URDFParameters,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    // A missing or malformed URDF is a real startup failure, but it should say
    // which file it could not load rather than panicking with an opaque message.
    let chain: k::Chain<f64> = k::Chain::<f64>::from_urdf_file(&robot_params.description_file)
        .map_err(|e| {
            format!(
                "Failed to load the URDF at '{}': {}",
                robot_params.description_file, e
            )
        })?;

    let mut con = connection_manager.get_connection().await;

    // The templates carry the `parent_frame_id` / `metadata` / flags that the
    // per-tick publish must not lose. Keeping them here is what lets the hot
    // path write transforms without reading them back out of Redis first.
    let robot_transforms = initialize_robot_transforms(&robot_params, &mut con).await;
    initialize_visual_transforms(&robot_params, &mut con).await;

    // A `sleep` inside the loop makes the real period "10 ms plus however long
    // the tick took", so the publish rate silently drifts below the rate the
    // realtime reader feeds it at. An interval paces from a fixed schedule;
    // `Delay` stops it from bursting to catch up after a slow tick.
    let mut interval = interval(Duration::from_millis(ROBOT_STATE_UPDATE_INTERVAL_MS.into()));
    interval.set_missed_tick_behavior(MissedTickBehavior::Delay);

    // A stationary robot republishes the same numbers forever otherwise. The
    // joint vector is the only input to both writes below, so it is enough to
    // decide whether either has anything new to say.
    let mut last_published_joints: Option<Vec<f64>> = None;
    // Measured state changes independently of the joints - safety mode flips while
    // the robot is stationary - so it needs its own change guard rather than
    // riding on the joint one.
    let mut last_measured: Option<MeasuredState> = None;

    loop {
        interval.tick().await;

        let (joints, speeds, measured) = {
            let ds = lock_driver_state(&driver_state);
            (
                ds.joint_values.clone(),
                ds.joint_speeds.clone(),
                MeasuredState {
                    safety_mode: ds.safety_mode,
                    robot_mode: ds.robot_mode,
                    program_state: ds.program_state.clone(),
                    program_running: ds.program_running,
                    tcp_pose: ds.tcp_pose.clone(),
                    forces: ds.forces.clone(),
                    speed_scaling: ds.speed_scaling,
                    digital_inputs: ds.digital_inputs,
                    digital_outputs: ds.digital_outputs,
                    robot_connected: ds.connected,
                    dashboard_connected: ds.dashboard_connected,
                    remote_control: ds.remote_control,
                },
            )
        };

        if measured.differs_from(last_measured.as_ref()) {
            publish_measured_state(&robot_params.name, &measured, &mut con).await;
            last_measured = Some(measured);
        }

        if last_published_joints.as_deref() == Some(joints.as_slice()) {
            continue;
        }

        // this caused the big error
        let mut joints_for_tf = joints.clone();
        if !joints.is_empty() {
            joints_for_tf[0] += std::f64::consts::PI;
        }

        publish_joint_states(&robot_params.name, &joints, &speeds, &mut con).await;
        publish_robot_transforms(&chain, &joints_for_tf, &robot_transforms, &mut con).await;

        last_published_joints = Some(joints.clone());
    }
}

/// TODO: Implement this to publish/send joint states to your custom system.
async fn publish_joint_states(
    robot_id: &str,
    joints: &[f64],
    _speeds: &[f64],
    con: &mut SPConnection,
) {

    // StateManager::set_sp_value(
    //                 &mut con,
    //                 &format!("{robot_name}_joint_states"),
    //                 &joint_states.to_spvalue(),
    //             )
    //             .await;
    let _ = StateManager::set_sp_value(
        con,
        &format!("{}_joint_states", &robot_id),
        &micro_sp::SPValue::Array(micro_sp::ArrayOrUnknown::Array(
            joints.iter().map(|x| x.to_spvalue()).collect(),
        )),
    ).await;
    // TransformsManager::move_transform(con, &frame_name, relative_transform_sp).await;
    // println!("Joints: {:?}", joints);
    // println!("Speeds: {:?}", speeds);
}

fn rpy_to_quaternion(roll: f64, pitch: f64, yaw: f64) -> (f64, f64, f64, f64) {
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();

    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;

    (x, y, z, w)
}

pub async fn initialize_visual_transforms(
    robot_params: &URDFParameters,
    con: &mut SPConnection,
) {
    let mut transforms_to_insert = vec![];
    let urdf_content =
        fs::read_to_string(&robot_params.description_file).expect("Failed to read URDF file");
    let doc = Document::parse(&urdf_content).expect("Failed to parse URDF XML");

    let mesh_links = vec![
        ("base_link_inertia", "base.dae"),
        ("shoulder_link", "shoulder.dae"),
        ("upper_arm_link", "upperarm.dae"),
        ("forearm_link", "forearm.dae"),
        ("wrist_1_link", "wrist1.dae"),
        ("wrist_2_link", "wrist2.dae"),
        ("wrist_3_link", "wrist3.dae"),
    ];

    for (link_name, mesh_file) in mesh_links {
        if let Some(link_node) = doc
            .descendants()
            .find(|n| n.has_tag_name("link") && n.attribute("name") == Some(link_name))
        {
            let mut x = 0.0;
            let mut y = 0.0;
            let mut z = 0.0;
            let mut roll = 0.0;
            let mut pitch = 0.0;
            let mut yaw = 0.0;

            if let Some(visual_node) = link_node.children().find(|n| n.has_tag_name("visual")) {
                if let Some(origin_node) = visual_node.children().find(|n| n.has_tag_name("origin"))
                {
                    if let Some(xyz_str) = origin_node.attribute("xyz") {
                        let coords: Vec<f64> = xyz_str
                            .split_whitespace()
                            .filter_map(|s| s.parse().ok())
                            .collect();
                        if coords.len() == 3 {
                            x = coords[0];
                            y = coords[1];
                            z = coords[2];
                        }
                    }

                    if let Some(rpy_str) = origin_node.attribute("rpy") {
                        let angles: Vec<f64> = rpy_str
                            .split_whitespace()
                            .filter_map(|s| s.parse().ok())
                            .collect();
                        if angles.len() == 3 {
                            roll = angles[0];
                            pitch = angles[1];
                            yaw = angles[2];
                        }
                    }
                }
            }

            let (qx, qy, qz, qw) = rpy_to_quaternion(roll, pitch, yaw);

            let mut sp_transform = SPTransform::default();
            sp_transform.translation.x = OrderedFloat(x);
            sp_transform.translation.y = OrderedFloat(y);
            sp_transform.translation.z = OrderedFloat(z);
            sp_transform.rotation.x = OrderedFloat(qx);
            sp_transform.rotation.y = OrderedFloat(qy);
            sp_transform.rotation.z = OrderedFloat(qz);
            sp_transform.rotation.w = OrderedFloat(qw);

            let visual_transform = SPTransformStamped {
                parent_frame_id: link_name.to_string(),
                child_frame_id: format!("{}_visual", link_name),
                transform: sp_transform,
                active_transform: false,
                enable_transform: true,
                time_stamp: SystemTime::now(),
                metadata: MapOrUnknown::Map(vec![
                    (
                        "override_meshes_dir".to_spvalue(),
                        robot_params.ur_meshes_path.to_spvalue(),
                    ),
                    ("mesh_file".to_spvalue(), mesh_file.to_spvalue()),
                    ("mesh_scale".to_spvalue(), 1.0.to_spvalue()),
                    ("visualize_mesh".to_spvalue(), true.to_spvalue()),
                    ("mesh_a".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_r".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_g".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_b".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_use_embedded_materials".to_spvalue(), true.to_spvalue()),
                    
                ]),
            };

            transforms_to_insert.push(visual_transform);
        }
    }

    let _ = TransformsManager::insert_transforms(con, &transforms_to_insert).await;
}

/// Seed the kinematic frames and hand back what was written, keyed by child
/// frame id.
///
/// The returned map is the authority on which frames this driver owns and what
/// their `parent_frame_id` / flags / metadata are. `publish_robot_transforms`
/// uses it to rebuild a full `SPTransformStamped` per tick without reading the
/// old one back from Redis first.
async fn initialize_robot_transforms(
    _robot_params: &URDFParameters,
    con: &mut SPConnection,
) -> HashMap<String, SPTransformStamped> {
    let mut transforms_to_insert = vec![];

    // let mut transforms_to_insert = vec!();
    let relations = vec![
        ("base_link", "base_link_inertia", Some("base.dae"), true),
        ("base_link_inertia", "shoulder_link", None, false),
        (
            "shoulder_link",
            "upper_arm_link",
            Some("shoulder.dae"),
            true,
        ),
        ("upper_arm_link", "forearm_link", Some("upperarm.dae"), true),
        ("forearm_link", "wrist_1_link", Some("forearm.dae"), true),
        ("wrist_1_link", "wrist_2_link", Some("wrist1.dae"), true),
        ("wrist_2_link", "wrist_3_link", Some("wrist2.dae"), true),
        ("wrist_3_link", "flange", Some("wrist3.dae"), true),
        ("wrist_3_link", "ft_frame", None, false),
        ("flange", "tool0", None, false),
    ];

    for (parent, child, _mesh, _visualize) in relations {
        let initial_transform = SPTransformStamped {
            // parent_frame_id: format!("{}_{}", robot_params.name, parent), // add this later
            // child_frame_id: format!("{}_{}", robot_params.name, child), // add this later
            parent_frame_id: format!("{}", parent),
            child_frame_id: format!("{}", child),
            transform: SPTransform::default(),
            active_transform: true,
            enable_transform: true,
            time_stamp: SystemTime::now(),
            metadata: MapOrUnknown::UNKNOWN,
        };
        transforms_to_insert.push(initial_transform);
    }

    let _ = TransformsManager::insert_transforms(con, &transforms_to_insert).await;

    transforms_to_insert
        .into_iter()
        .map(|transform| (transform.child_frame_id.clone(), transform))
        .collect()
}

/// Publish the current kinematic frames as a single `MSET`.
///
/// This used to call `TransformsManager::move_transform` per link, which is a
/// `GET` + deserialize + serialize + `SET` awaited one frame at a time - about
/// 22 sequential round trips every tick. `move_transform` only did that read to
/// preserve the parent id and metadata stored in Redis, and this driver wrote
/// those itself in `initialize_robot_transforms`, so `robot_transforms` supplies
/// them instead and the whole set goes out in one `insert_transforms`.
///
/// Frames missing from `robot_transforms` are skipped, which is deliberate:
/// `move_transform` used to *fail* for a chain node that had no transform in
/// Redis (the error was discarded), whereas `insert_transforms` would create
/// one. Skipping keeps chain nodes named after joints rather than links out of
/// the TF tree, exactly as before.
///
/// The assumption this makes explicit: the driver owns its kinematic frames.
/// `move_transform` preserved whatever parent was in Redis, so an external
/// `reparent_transform` of, say, `tool0` used to survive; now the template wins
/// and the driver reasserts its own parent on the next tick. That only matters
/// for the frames listed in `initialize_robot_transforms` - the `_visual`
/// frames are written under different child ids and are never touched here.
async fn publish_robot_transforms(
    chain: &k::Chain<f64>,
    joints: &[f64],
    robot_transforms: &HashMap<String, SPTransformStamped>,
    con: &mut SPConnection,
) {
    if let Ok(()) = chain.set_joint_positions(joints) {
        chain.update_link_transforms();

        let mut transforms_to_publish = Vec::with_capacity(robot_transforms.len());

        for node in chain.iter() {
            let frame_name = match &*node.link() {
                Some(link) => link.name.clone(),
                None => node.joint().name.clone(),
            };

            if frame_name == "base_link" {
                continue;
            }

            // Not a frame this driver owns - `move_transform` would have failed
            // here too, so skipping is the existing behaviour.
            let Some(template) = robot_transforms.get(&frame_name) else {
                continue;
            };

            let child_world = node
                .world_transform()
                .unwrap_or_else(nalgebra::Isometry3::identity);

            let relative_transform = if let Some(parent) = node.parent() {
                let parent_world = parent
                    .world_transform()
                    .unwrap_or_else(nalgebra::Isometry3::identity);

                parent_world.inverse() * child_world
            } else {
                child_world
            };

            let mut updated = template.clone();
            updated.transform = isometry_to_sp_transform(relative_transform);
            updated.time_stamp = SystemTime::now();
            transforms_to_publish.push(updated);
        }

        // `insert_transforms` logs an error on an empty vector, and this runs at
        // the tick rate, so do not hand it one.
        if !transforms_to_publish.is_empty() {
            let _ = TransformsManager::insert_transforms(con, &transforms_to_publish).await;
        }
    };
}

fn isometry_to_sp_transform(isometry: Isometry3<f64>) -> SPTransform {
    let translation_vector: &Vector3<f64> = &isometry.translation.vector;
    let rotation_quaternion: &Quaternion<f64> = isometry.rotation.quaternion();

    let sp_translation = SPTranslation {
        x: OrderedFloat(translation_vector.x),
        y: OrderedFloat(translation_vector.y),
        z: OrderedFloat(translation_vector.z),
    };

    let sp_rotation = SPRotation {
        w: OrderedFloat(rotation_quaternion.w),
        x: OrderedFloat(rotation_quaternion.i),
        y: OrderedFloat(rotation_quaternion.j),
        z: OrderedFloat(rotation_quaternion.k),
    };

    SPTransform {
        translation: sp_translation,
        rotation: sp_rotation,
    }
}

/// Everything `realtime_reader` decodes that is not a joint angle, plus the two
/// connection flags.
#[derive(Clone)]
struct MeasuredState {
    safety_mode: i32,
    robot_mode: i32,
    program_state: String,
    program_running: bool,
    tcp_pose: Vec<f64>,
    forces: Vec<f64>,
    speed_scaling: f64,
    digital_inputs: u32,
    digital_outputs: u32,
    robot_connected: bool,
    dashboard_connected: bool,
    remote_control: bool,
}

/// Force change, in newtons, that counts as news.
///
/// A load cell at rest still reports a fraction of a newton of noise, so an exact
/// comparison would report "changed" on essentially every frame and the guard below
/// would never fire. This is well under any useful `force_threshold` while being
/// comfortably above the noise floor.
const FORCE_EPSILON: f64 = 0.25;

/// Pose change, in metres/radians, that counts as news. The TCP pose is derived
/// from the joint encoders and jitters in the last few digits while stationary.
const POSE_EPSILON: f64 = 1e-4;

impl MeasuredState {
    /// Has anything changed enough to be worth a Redis round trip?
    ///
    /// The discrete fields - modes, IO bits, connection flags - compare exactly,
    /// because a single-tick flip of any of them is exactly the event a supervisor
    /// is watching for and must never be filtered out. Only the continuous
    /// measurements get a tolerance, and only to keep sensor noise from turning a
    /// 5 ms tick into a permanent 200 writes/second even with the robot parked.
    fn differs_from(&self, previous: Option<&MeasuredState>) -> bool {
        let Some(previous) = previous else {
            return true;
        };

        if self.safety_mode != previous.safety_mode
            || self.robot_mode != previous.robot_mode
            || self.program_state != previous.program_state
            || self.program_running != previous.program_running
            || self.digital_inputs != previous.digital_inputs
            || self.digital_outputs != previous.digital_outputs
            || self.robot_connected != previous.robot_connected
            || self.dashboard_connected != previous.dashboard_connected
            || self.remote_control != previous.remote_control
        {
            return true;
        }

        if (self.speed_scaling - previous.speed_scaling).abs() > f64::EPSILON {
            return true;
        }

        exceeds(&self.forces, &previous.forces, FORCE_EPSILON)
            || exceeds(&self.tcp_pose, &previous.tcp_pose, POSE_EPSILON)
    }
}

/// True if the two readings differ in length or in any element by more than `eps`.
fn exceeds(current: &[f64], previous: &[f64], eps: f64) -> bool {
    current.len() != previous.len()
        || current
            .iter()
            .zip(previous)
            .any(|(a, b)| (a - b).abs() > eps)
}

/// Publish measured robot state to Redis.
///
/// The modes go out as names rather than the raw integers off the wire, so a rule
/// on the Redis side can compare against `"PROTECTIVE_STOP"` instead of hard-coding
/// UR's numbering.
///
/// One `set_state` (a single `MSET`) rather than a `set_sp_value` per key. Twelve
/// awaited round trips per tick would be twelve times the latency for one logical
/// "publish this tick" step, and the change guard in the caller does not help while
/// the robot is moving: the TCP forces are noisy floats that differ on essentially
/// every frame. This is the same trade the transform publish below already makes.
async fn publish_measured_state(robot_id: &str, measured: &MeasuredState, con: &mut SPConnection) {
    let key = |suffix: &str| format!("{robot_id}_{suffix}");

    let to_array = |values: &[f64]| {
        micro_sp::SPValue::Array(micro_sp::ArrayOrUnknown::Array(
            values.iter().map(|x| x.to_spvalue()).collect(),
        ))
    };

    // Magnitude of the translational part of the TCP force. `force_threshold` in a
    // request is compared against the same quantity inside the URScript templates,
    // so this is the value that decides whether a force guard would trip.
    let force_feedback = if measured.forces.len() >= 3 {
        (measured.forces[0].powi(2) + measured.forces[1].powi(2) + measured.forces[2].powi(2))
            .sqrt()
    } else {
        0.0
    };

    let log_target = format!("{robot_id}_state_publisher");
    let state = State::new();

    let state = state.add(
        assign!(v!(&&key("safety_mode")), safety_mode_name(measured.safety_mode).to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(v!(&&key("robot_mode")), robot_mode_name(measured.robot_mode).to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(v!(&&key("program_state")), measured.program_state.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(bv!(&&key("program_running")), measured.program_running.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(av!(&&key("tcp_pose")), to_array(&measured.tcp_pose)),
        &log_target,
    );
    let state = state.add(
        assign!(av!(&&key("tcp_force")), to_array(&measured.forces)),
        &log_target,
    );
    let state = state.add(
        assign!(fv!(&&key("force_feedback")), force_feedback.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(fv!(&&key("speed_scaling")), measured.speed_scaling.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(iv!(&&key("digital_inputs")), (measured.digital_inputs as i64).to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(iv!(&&key("digital_outputs")), (measured.digital_outputs as i64).to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(bv!(&&key("robot_connected")), measured.robot_connected.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(bv!(&&key("dashboard_connected")), measured.dashboard_connected.to_spvalue()),
        &log_target,
    );
    let state = state.add(
        assign!(bv!(&&key("remote_control")), measured.remote_control.to_spvalue()),
        &log_target,
    );

    StateManager::set_state(con, &state).await;
}
