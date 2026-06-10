use micro_sp::{ConnectionManager, MapOrUnknown, SPTransform, SPTransformStamped, ToSPValue, TransformsManager};
use redis::{aio::MultiplexedConnection};
use micro_sp::management::transforms;

use crate::{DriverState, URDFParameters};
use std::sync::{Arc, Mutex};

pub async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
    robot_params: URDFParameters,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {
    let chain: k::Chain<f64> = k::Chain::<f64>::from_urdf_file(&robot_params.description_file).unwrap();

    let mut con = connection_manager.get_connection().await;
    initialize_robot_transforms(robot_params, &mut con).await;

    loop {
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;

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
        publish_robot_transforms(&chain, &joints).await;
        publish_measured_state(state, prog_state, &forces, inputs, outputs).await;
    }
}

/// TODO: Implement this to publish/send joint states to your custom system.
async fn publish_joint_states(joints: &[f64], speeds: &[f64]) {
    println!("Joints: {:?}", joints);
    println!("Speeds: {:?}", speeds);
}

// async fn publish_robot_transforms(chain: &k::Chain<f64>, joints: &[f64]) {
//     let current_joint_states = joints.to_vec();
//     chain.set_joint_positions(&current_joint_states).unwrap();
//     chain.update_link_transforms();

//     for node in chain.iter_links() {

//         let frame_name = node.name.clone(); //().name.clone();

//         // This returns a nalgebra::Isometry3 representing the pose
//         let transform = node.inertial.world_transform().unwrap_or_default();
//         // let transform = node.world_transform

//         println!("Frame: {}", frame_name);
//         println!("Translation [X, Y, Z]: {:?}", transform.translation);
//         println!("Rotation (Quat): {:?}", transform.rotation);
//         println!("---");
//     }
//     println!("Joints: {:?}", joints);
// }

// async fn publish_robot_transforms(chain: &k::Chain<f64>, joints: &[f64]) {
//     let current_joint_states = joints.to_vec();
//     chain.set_joint_positions(&current_joint_states).unwrap();
//     chain.update_link_transforms();

//     for node in chain.iter_links() {
//         let frame_name = node.name.clone();

//         // 1. Get the world transform of the current link (the child)
//         let child_world = node.inertial.world_transform().unwrap_or_default();

//         // 2. Check if the node has a parent to calculate the relative transform
//         let relative_transform = if let Some(parent_node) = node.parent() {
//             // Get the world transform of the parent link
//             let parent_world = parent_node.inertial.world_transform().unwrap_or_default();

//             // Calculate child relative to parent: Parent^-1 * Child
//             parent_world.inverse() * child_world
//         } else {
//             // If the node has no parent (e.g., the root/base link), its relative transform is its world transform
//             child_world
//         };

//         println!("Frame: {} (Relative to Parent)", frame_name);
//         println!("Translation [X, Y, Z]: {:?}", relative_transform.translation);
//         println!("Rotation (Quat): {:?}", relative_transform.rotation);
//         println!("---");
//     }
//     println!("Joints: {:?}", joints);
// }

use std::collections::HashMap;
use std::time::SystemTime;

async fn initialize_robot_transforms(robot_params: URDFParameters, con: &mut MultiplexedConnection) {

    let mut transforms_to_insert = vec!();

    // let mut transforms_to_insert = vec!();
    let relations = vec![
        ("base_link", "base_link_inertia", Some("base.dae")),
        ("base_link_inertia", "shoulder_link", None),
        ("shoulder_link", "upper_arm_link", Some("shoulder.dae")),
        ("upper_arm_link", "forearm_link", Some("upperarm.dae")),
        ("forearm_link", "wrist_1_link", Some("forearm.dae")),
        ("wrist_1_link", "wrist_2_link", Some("wrist1.dae")),
        ("wrist_2_link", "wrist_3_link", Some("wrist2.dae")),
        ("wrist_3_link", "flange", Some("wrist3.dae")),
        ("wrist_3_link", "ft_frame", None),
        ("flange", "tool0", None),
    ];

    for (parent, child, mesh) in relations {
        let initial_transform = SPTransformStamped {
            // parent_frame_id: format!("{}_{}", robot_params.name, parent), // add this later
            // child_frame_id: format!("{}_{}", robot_params.name, child), // add this later
            parent_frame_id: format!("{}", parent),
            child_frame_id: format!("{}", child),
            transform: SPTransform::default(),
            active_transform: true,
            enable_transform: true,
            time_stamp: SystemTime::now(),
            metadata: MapOrUnknown::Map(vec![
                ("override_meshes_dir".to_spvalue(), robot_params.ur_meshes_path.to_spvalue()),
                if let Some(mesh_file) = mesh {
                    ("mesh_file".to_spvalue(), mesh_file.to_spvalue())
                } else {
                    ("mesh_file".to_spvalue(), "".to_spvalue())
                },
                ("mesh_scale".to_spvalue(), 1.0.to_spvalue()),
                ("visualize_mesh".to_spvalue(), true.to_spvalue()),
                // BElow everything has to be 0 in order for ros2 to pick up .dae colors
                ("mesh_a".to_spvalue(), 0.0.to_spvalue()),
                ("mesh_r".to_spvalue(), 0.0.to_spvalue()),
                ("mesh_g".to_spvalue(), 0.0.to_spvalue()),
                ("mesh_b".to_spvalue(), 0.0.to_spvalue()),
            ]),
        };
        transforms_to_insert.push(initial_transform);
    }

    let _ = TransformsManager::insert_transforms(con, &transforms_to_insert).await;

}

async fn publish_robot_transforms(chain: &k::Chain<f64>, joints: &[f64]) {
    let current_joint_states = joints.to_vec();
    chain.set_joint_positions(&current_joint_states).unwrap();
    chain.update_link_transforms();

    let mut world_transforms = HashMap::new();
    for node in chain.iter_links() {
        let frame_name = node.name.clone();
        let transform = node.inertial.world_transform().unwrap_or_default();
        world_transforms.insert(frame_name, transform);
    }

    let relations = vec![
        ("base_link", "base_link_inertia"),
        ("base_link_inertia", "shoulder_link"),
        ("shoulder_link", "upper_arm_link"),
        ("upper_arm_link", "forearm_link"),
        ("forearm_link", "wrist_1_link"),
        ("wrist_1_link", "wrist_2_link"),
        ("wrist_2_link", "wrist_3_link"),
        ("wrist_3_link", "flange"),
        ("wrist_3_link", "ft_frame"),
        ("flange", "tool0"),
    ];

    let mut child_to_parent = HashMap::new();
    for (parent, child) in relations {
        child_to_parent.insert(child.to_string(), parent.to_string());
    }

    for node in chain.iter_links() {
        let frame_name = node.name.clone();
        let child_world = world_transforms.get(&frame_name).unwrap();

        let relative_transform = if let Some(parent_name) = child_to_parent.get(&frame_name) {
            if let Some(parent_world) = world_transforms.get(parent_name) {
                parent_world.inverse() * child_world
            } else {
                // maybe handle this differently
                *child_world
            }
        } else {
            *child_world
        };

        println!("Frame: {} (Relative to Parent)", frame_name);
        println!(
            "Translation [X, Y, Z]: {:?}",
            relative_transform.translation.vector.as_slice()
        );
        println!(
            "Rotation (Quat): {:?}",
            relative_transform.rotation.coords.as_slice()
        );
        println!("---");
    }
    println!("Joints: {:?}", joints);
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
