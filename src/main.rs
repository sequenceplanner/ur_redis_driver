use local_ip_address::local_ip;
use micro_sp::{ConnectionManager, StateManager, initialize_env_logger};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
use ur_redis_driver::driver::dashboard::{dashboard, handle_dashboard_commands_loop};
use ur_redis_driver::driver::socker_server::socket_server;
use ur_redis_driver::interfaces::command_server::command_server;
use ur_redis_driver::{
    DriverState, URDFParameters, generate_robot_interface_state, realtime_reader, state_publisher,
};

async fn run() -> Result<(), Box<dyn std::error::Error>> {
    initialize_env_logger();
    let robot_id = match std::env::var("ROBOT_ID") {
        Ok(id) => id,
        Err(e) => {
            log::warn!(target: &&format!("r1_ur_redis_driver"), "Failed to read ROBOT_ID environment variable: {}", e);
            log::warn!(target: &&format!("r1_ur_redis_driver"), "Setting ROBOT_ID to r1.");
            "r1".to_string()
        }
    };
    let log_target = format!("{}_ur_redis_driver", robot_id);
    let robot_model = match std::env::var("ROBOT_MODEL") {
        Ok(id) => id,
        Err(e) => {
            log::warn!(target: &log_target, "Failed to read ROBOT_MODEL environment variable: {}", e);
            log::warn!(target: &log_target, "Setting ROBOT_MODEL to ur20.");
            "ur20".to_string()
        }
    };
    let ur_description_dir = match std::env::var("UR_DESCRIPTION_DIR") {
        Ok(id) => id,
        Err(e) => {
            log::warn!(target: &log_target, "Failed to read UR_DESCRIPTION_DIR environment variable: {}", e);
            log::warn!(target: &log_target, "Setting UR_DESCRIPTION_DIR to local dir.");
            "src/ur_description/".to_string()
        }
    };
    let templates_dir = "templates/".to_string();
    let override_host_address = local_ip().ok().map(|ip| ip.to_string());
    match &override_host_address {
        Some(host_address) => log::info!(target: &log_target, "Auto setting OVERRIDE HOST ADDRESS to: {}", host_address),
        None => log::warn!(target: &log_target, "Automatic OVERRIDE HOST ADDRESS not set."),
    }

    let ur_address = match std::env::var("UR_ADDRESS") {
        Ok(id) => id,
        Err(e) => {
            log::warn!(target: &log_target, "Failed to read UR_ADDRESS environment variable: {}", e);
            log::warn!(target: &log_target, "Setting UR_ADDRESS to 0.0.0.0");
            "0.0.0.0".to_string()
        }
    };
    let ur_dashboard_address = format!("{}:29999", ur_address);
    let ur_address = format!("{}:30003", ur_address);

    let mut path_urdf = PathBuf::from(&ur_description_dir);
    let mut path_ur_meshes = PathBuf::from(&ur_description_dir);
    path_urdf.push(format!("urdf/{}.urdf", robot_model));
    path_ur_meshes.push(format!("meshes"));
    path_ur_meshes.push(robot_model.clone());
    path_ur_meshes.push(format!("visual"));
    let urdf_path = path_urdf.to_string_lossy().to_string();
    let ur_meshes_path = path_ur_meshes.to_string_lossy().to_string();

    let mut params = URDFParameters::default();
    params.name = robot_id.clone();
    params.ur_type = robot_model;
    params.description_file = urdf_path.clone();
    params.ur_meshes_path = ur_meshes_path;

    let templates: tera::Tera = {
        let tera = match tera::Tera::new(&format!("{}/*.script", templates_dir)) {
            Ok(t) => {
                log::warn!(target: &log_target, "Looking for Tera templates...",);
                t
            }
            Err(e) => {
                log::error!(target: &log_target, "UR Script template parsing error(s): {}", e);
                ::std::process::exit(1);
            }
        };
        tera
    };

    let template_names = templates
        .get_template_names()
        .map(|x| x.to_string())
        .collect::<Vec<String>>();
    if template_names.len() == 0 {
        log::error!(target: &log_target, "Couldn't find any Tera templates.");
    } else {
        log::info!(target: &log_target, "Found templates.");
    }

    let state = generate_robot_interface_state(&robot_id, &log_target);
    // Skip the gripper for now, but it can be added to be used with URCaps
    // let gripper_state = generate_gripper_interface_state("g1", &log_target);
    // let state = state.extend(gripper_state, true);

    let connection_manager = ConnectionManager::new().await;
    StateManager::set_state(&mut connection_manager.get_connection().await, &state).await;
    let con_arc = Arc::new(connection_manager);

    let (tx_dashboard, rx_dashboard) = mpsc::channel(10);
    let shared_state = Arc::new(Mutex::new(DriverState::new()));
    let (local_addr_sender, local_addr_receiver) = watch::channel(None);

    let dashboard_task = handle_dashboard_commands_loop(tx_dashboard.clone());
    let command_server = command_server(
        &ur_address,
        &robot_id,
        &con_arc,
        shared_state.clone(),
        &local_addr_receiver,
        tx_dashboard.clone(),
        &templates,
    );

    let realtime_task = realtime_reader(
        shared_state.clone(),
        ur_address.to_string(),
        override_host_address,
        local_addr_sender,
    );

    let con_arc_clone = con_arc.clone();
    let state_publisher_task = state_publisher(shared_state.clone(), params, &con_arc_clone);
    let socket_server_task = socket_server(shared_state.clone(), local_addr_receiver.clone());
    let dashboard_connection = dashboard(rx_dashboard, ur_dashboard_address);

    let ret = tokio::try_join!(
        command_server,
        realtime_task,
        socket_server_task,
        dashboard_connection,
        state_publisher_task,
        dashboard_task,
    );

    if let Err(e) = ret {
        shared_state.lock().unwrap().running = false;
        return Err(e.into());
    }

    std::fs::File::create("/tmp/robot_controller_ready.flag").unwrap();

    Ok(())
}

#[tokio::main]
async fn main() {
    loop {
        if let Err(e) = run().await {
            println!("fatal error: {}", e);
            tokio::time::sleep(Duration::from_secs(2)).await;
        }
    }
}
