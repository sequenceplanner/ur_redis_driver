use k::nalgebra::{self, Quaternion};
use k::{Isometry3, Vector3};
// use micro_sp::management::transforms;
use micro_sp::{
    ConnectionManager, MapOrUnknown, SPRotation, SPTransform, SPTransformStamped, SPTranslation,
    ToSPValue, TransformsManager,
};
use ordered_float::OrderedFloat;
use redis::aio::MultiplexedConnection;
// use std::collections::HashMap;
use std::time::SystemTime;
use crate::{DriverState, URDFParameters};
use std::sync::{Arc, Mutex};
use roxmltree::Document;
use std::fs;

pub async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
    robot_params: URDFParameters,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    let chain: k::Chain<f64> =
        k::Chain::<f64>::from_urdf_file(&robot_params.description_file).unwrap();

    let mut con = connection_manager.get_connection().await;
    initialize_robot_transforms(&robot_params, &mut con).await;
    initialize_visual_transforms(&robot_params, &mut con).await;

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(20)).await;

        let (joints, speeds, state, prog_state, forces, inputs, outputs) = {
            let ds = driver_state.lock().unwrap();
            (
                ds.joint_values.clone(),
                ds.joint_speeds.clone(),
                ds.robot_state,
                ds.program_state,
                ds.forces.clone(),
                ds.digital_inputs,
                ds.digital_outputs,
            )
        };

        publish_joint_states(&joints, &speeds).await;
        // let con_clone = con.clone();
        publish_robot_transforms(&chain, &joints, &mut con).await;
        publish_measured_state(state, prog_state, &forces, inputs, outputs).await;
    }
}

/// TODO: Implement this to publish/send joint states to your custom system.
async fn publish_joint_states(joints: &[f64], speeds: &[f64]) {
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
    con: &mut MultiplexedConnection,
) {
    let mut transforms_to_insert = vec![];
    let urdf_content = fs::read_to_string(&robot_params.description_file)
        .expect("Failed to read URDF file");
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
        if let Some(link_node) = doc.descendants().find(|n| {
            n.has_tag_name("link") && n.attribute("name") == Some(link_name)
        }) {
            let mut x = 0.0; let mut y = 0.0; let mut z = 0.0;
            let mut roll = 0.0; let mut pitch = 0.0; let mut yaw = 0.0;

            if let Some(visual_node) = link_node.children().find(|n| n.has_tag_name("visual")) {
                if let Some(origin_node) = visual_node.children().find(|n| n.has_tag_name("origin")) {
                    
                    if let Some(xyz_str) = origin_node.attribute("xyz") {
                        let coords: Vec<f64> = xyz_str.split_whitespace()
                                                      .filter_map(|s| s.parse().ok())
                                                      .collect();
                        if coords.len() == 3 {
                            x = coords[0]; y = coords[1]; z = coords[2];
                        }
                    }

                    if let Some(rpy_str) = origin_node.attribute("rpy") {
                        let angles: Vec<f64> = rpy_str.split_whitespace()
                                                      .filter_map(|s| s.parse().ok())
                                                      .collect();
                        if angles.len() == 3 {
                            roll = angles[0]; pitch = angles[1]; yaw = angles[2];
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
                    ("override_meshes_dir".to_spvalue(), robot_params.ur_meshes_path.to_spvalue()),
                    ("mesh_file".to_spvalue(), mesh_file.to_spvalue()),
                    ("mesh_scale".to_spvalue(), 1.0.to_spvalue()),
                    ("visualize_mesh".to_spvalue(), true.to_spvalue()),
                    ("mesh_a".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_r".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_g".to_spvalue(), 0.0.to_spvalue()),
                    ("mesh_b".to_spvalue(), 0.0.to_spvalue()),
                ]),
            };

            transforms_to_insert.push(visual_transform);
        }
    }

    let _ = TransformsManager::insert_transforms(con, &transforms_to_insert).await;
}

async fn initialize_robot_transforms(
    robot_params: &URDFParameters,
    con: &mut MultiplexedConnection,
) {
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

    for (parent, child, mesh, visualize) in relations {
        let initial_transform = SPTransformStamped {
            // parent_frame_id: format!("{}_{}", robot_params.name, parent), // add this later
            // child_frame_id: format!("{}_{}", robot_params.name, child), // add this later
            parent_frame_id: format!("{}", parent),
            child_frame_id: format!("{}", child),
            transform: SPTransform::default(),
            active_transform: true,
            enable_transform: true,
            time_stamp: SystemTime::now(),
            metadata: MapOrUnknown::UNKNOWN
        };
        transforms_to_insert.push(initial_transform);
    }

    let _ = TransformsManager::insert_transforms(con, &transforms_to_insert).await;
}

async fn publish_robot_transforms(
    chain: &k::Chain<f64>,
    joints: &[f64],
    con: &mut MultiplexedConnection,
) {
    chain.set_joint_positions(joints).unwrap();
    chain.update_link_transforms();

    for node in chain.iter() {
        let frame_name = match &*node.link() {
            Some(link) => link.name.clone(),
            None => node.joint().name.clone(),
        };

        if frame_name == "base_link" {
            continue;
        }

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

        let relative_transform_sp = isometry_to_sp_transform(relative_transform);

        let _ = TransformsManager::move_transform(con, &frame_name, relative_transform_sp).await;
    }
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

/// TODO: Implement this to publish/send robot measured states to your custom system.
async fn publish_measured_state(
    robot_state: i32,
    program_state: i32,
    forces: &[f64],
    inputs: u32,
    outputs: u32,
) {
    // println!("Robot State: {}, Forces: {:?}", robot_state, forces);
}
