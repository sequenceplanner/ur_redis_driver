use local_ip_address::local_ip;
use micro_sp::{
    ConnectionManager, DEFAULT_HEALTH_CHECK_PERIOD, StateManager, initialize_env_logger,
};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
use ur_redis_driver::driver::dashboard::dashboard;
use ur_redis_driver::driver::socket_server::socket_server;
use ur_redis_driver::interfaces::command_server::command_server;
use ur_redis_driver::interfaces::dashboard_server::dashboard_server;
use ur_redis_driver::{
    DriverState, URDFParameters, generate_robot_interface_state, lock_driver_state, realtime_reader,
    state_publisher,
};

/// A TCP port from the environment, warning and falling back rather than aborting -
/// a typo in a launcher's .env should not take the driver down.
fn env_port(name: &str, default: u16, log_target: &str) -> u16 {
    match std::env::var(name) {
        Ok(raw) => match raw.trim().parse::<u16>() {
            Ok(port) => port,
            Err(e) => {
                log::warn!(target: log_target, "{} is not a port number ({}): {}. Using {}.", name, raw, e, default);
                default
            }
        },
        Err(_) => default,
    }
}

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
    // Namespace for the frames this driver publishes. Empty by default and empty is
    // exactly the identity, so a single-robot cell keeps publishing the bare
    // `base_link`..`tool0` its scene files and its model already name.
    //
    // Deliberately NOT derived from ROBOT_ID: the existing single-robot run has
    // ROBOT_ID=r1 and must not start publishing `r1_base_link`. Opt-in only.
    //
    // Named UR_TF_PREFIX rather than TF_PREFIX because micro_sp already exports a
    // `TF_PREFIX` (its Redis key prefix, "tf:") into this scope via `use micro_sp::*`.
    let tf_prefix = std::env::var("UR_TF_PREFIX").unwrap_or_default();
    if !tf_prefix.is_empty() {
        log::info!(target: &log_target, "Publishing robot frames under the prefix '{}'.", tf_prefix);
    }

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
        Some(host_address) => {
            log::info!(target: &log_target, "Auto setting OVERRIDE HOST ADDRESS to: {}", host_address)
        }
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
    // Both default to the standard UR ports, so a real robot and a single-URSim run
    // need neither variable. They exist because a second URSim container on the same
    // host has to publish 29999/30003 on different host ports.
    let ur_dashboard_port = env_port("UR_DASHBOARD_PORT", 29999, &log_target);
    let ur_realtime_port = env_port("UR_REALTIME_PORT", 30003, &log_target);
    let ur_dashboard_address = format!("{}:{}", ur_address, ur_dashboard_port);
    let ur_address = format!("{}:{}", ur_address, ur_realtime_port);

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
    params.tf_prefix = tf_prefix.clone();
    params.ur_type = robot_model;
    params.description_file = urdf_path.clone();
    params.ur_meshes_path = ur_meshes_path;

    let templates: Arc<tera::Tera> = Arc::new({
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
    });

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

    // The `Arc` has to exist before the health monitor can be spawned - it takes
    // `self: &Arc<Self>` so the background task can hold its own reference.
    let con_arc = Arc::new(ConnectionManager::new().await);
    let mut con = con_arc.connection();
    StateManager::set_state(&mut con, &state).await;

    // One PING every few seconds for the whole process, purely so an unreachable
    // Redis shows up in the log. Nothing depends on it to recover: the handles
    // handed to the tasks below reconnect themselves. The `JoinHandle` is
    // dropped deliberately - this task never returns, so it must not be joined
    // by the `try_join!` at the end of `run`.
    con_arc.spawn_health_monitor(&log_target, DEFAULT_HEALTH_CHECK_PERIOD);

    let (tx_dashboard, rx_dashboard) = mpsc::channel(10);
    let shared_state = Arc::new(Mutex::new(DriverState::new()));
    let (local_addr_sender, local_addr_receiver) = watch::channel(None);

    // let dashboard_task = handle_dashboard_commands_loop(tx_dashboard.clone());
    let command_server = command_server(
        &ur_address,
        &robot_id,
        &tf_prefix,
        &con_arc,
        shared_state.clone(),
        &local_addr_receiver,
        tx_dashboard.clone(),
        templates.clone(),
    );

    let dashboard_command_server =
        dashboard_server(&robot_id, &con_arc, tx_dashboard.clone());

    let realtime_task = realtime_reader(
        shared_state.clone(),
        ur_address.to_string(),
        override_host_address,
        local_addr_sender,
        log_target.clone(),
    );

    let con_arc_clone = con_arc.clone();
    let state_publisher_task = state_publisher(shared_state.clone(), params, &con_arc_clone);
    let socket_server_task = socket_server(shared_state.clone(), local_addr_receiver.clone());
    let dashboard_connection = dashboard(
        rx_dashboard,
        ur_dashboard_address,
        shared_state.clone(),
        log_target.clone(),
    );

    // A readiness marker for whatever supervises this process. Failing to write it
    // says nothing about whether the driver can talk to the robot, so it must not
    // stop startup - this used to be an `.unwrap()`.
    // An env var rather than deriving the name from ROBOT_ID, so the existing
    // launcher's `rm -f /tmp/robot_controller_ready.flag` keeps matching and only a
    // two-robot run has to give each driver its own path.
    let ready_flag = std::env::var("ROBOT_CONTROLLER_READY_FLAG")
        .unwrap_or_else(|_| "/tmp/robot_controller_ready.flag".to_string());
    if let Err(e) = std::fs::File::create(&ready_flag) {
        log::warn!(target: &log_target, "Could not write the readiness flag: {}", e);
    }

    let ret = tokio::try_join!(
        command_server,
        dashboard_command_server,
        realtime_task,
        socket_server_task,
        dashboard_connection,
        state_publisher_task,
    );

    if let Err(e) = ret {
        lock_driver_state(&shared_state).running = false;
        return Err(e.into());
    }

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
