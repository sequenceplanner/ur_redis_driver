use micro_sp::management::transforms;

use crate::DriverState;
use std::sync::{Arc, Mutex};

pub async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let chain: k::Chain<f64> = k::Chain::<f64>::from_urdf_file("src/urdf/ur20.urdf").unwrap();

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

        // Call the boilerplate hook functions
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

async fn publish_robot_transforms(chain: &k::Chain<f64>, joints: &[f64]) {
    let current_joint_states = joints.to_vec();
    chain.set_joint_positions(&current_joint_states).unwrap();
    chain.update_transforms();
    chain.update_link_transforms();

    // for link in chain.iter_links().
    //  c.iter_links()
    //                 .map(|l| l.name.clone())
    //                 .collect::<Vec<String>>(),

    for node in chain.iter_links() {
        
        let frame_name = node.name.clone(); //().name.clone();

        // This returns a nalgebra::Isometry3 representing the pose
        let transform = node.inertial.world_transform().unwrap_or_default();
        // let transform = node.world_transform

        println!("Frame: {}", frame_name);
        println!("Translation [X, Y, Z]: {:?}", transform.translation);
        println!("Rotation (Quat): {:?}", transform.rotation);
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
