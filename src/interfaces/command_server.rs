use std::{
    net::SocketAddr,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};

use base64::{Engine as _, engine::general_purpose::STANDARD};

// use std::net::TcpStream;
// use std::io;

use futures::StreamExt;
use micro_sp::*;
use tokio::{
    sync::{mpsc, oneshot, watch},
    time::interval,
};
use tokio_util::task::LocalPoolHandle;

// use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::{driver::handle_request::handle_request, *};

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 250;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base_link"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
// pub static DEFAULT_TCP_ID: &'static str = "svt_tcp";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn command_server(
    ur_address: &str,
    robot_name: &str,
    connection_manager: &Arc<ConnectionManager>,
    driver_state: Arc<Mutex<DriverState>>,
    local_addr: &watch::Receiver<Option<SocketAddr>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = format!("{robot_name}_action_client");
    let mut interval = interval(Duration::from_millis(ROBOT_STATE_UPDATE_INTERVAL_MS.into()));
    let local_pool = LocalPoolHandle::new(1);

    let suffixes = [
        "request_trigger",
        "request_state",
        "request_result",
        "request_cancel",
        "request_feedback",
        "command_type",
        "accelleration",
        "velocity",
        "global_acceleration_scaling",
        "global_velocity_scaling",
        "use_execution_time",
        "execution_time",
        "use_blend_radius",
        "blend_radius",
        "use_joint_positions",
        "joint_positions",
        "use_preferred_joint_config",
        "preferred_joint_config",
        "use_payload",
        "payload",
        "baseframe_id",
        "faceplate_id",
        "goal_feature_id",
        "tcp_id",
        "root_frame_id",
        "force_threshold",
        "use_relative_pose",
        "relative_pose",
        "force_feedback",
        "reset_request_mechanism",
        "waypoints_raw",
    ];

    let keys: Vec<String> = suffixes
        .iter()
        .map(|s| format!("{robot_name}_{s}"))
        .collect();

    loop {
        interval.tick().await;
        if connection_manager
            .check_redis_health(&log_target)
            .await
            .is_err()
        {
            continue;
        }

        let mut con = connection_manager.get_connection().await;
        let state = match StateManager::get_state_for_keys(&mut con, &keys, &log_target).await {
            Some(s) => s,
            None => continue,
        };

        let key = |suffix: &str| format!("{robot_name}_{suffix}");

        let cancel_goal = state.get_bool_or_default_to_false(&key("request_cancel"), &log_target);

        if cancel_goal {
            StateManager::set_sp_value(&mut con, &key("request_cancel"), &false.to_spvalue()).await;

            let mut ds = driver_state.lock().unwrap();
            if let Some(sender) = ds.cancel_sender.take() {
                println!("Cancel goal requested from Redis! Aborting active script...");
                let _ = sender.try_send(());
            } else {
                println!("Cancel goal requested, but no active goal is running.");
            }
        }

        let mut request_trigger =
            state.get_bool_or_default_to_false(&key("request_trigger"), &log_target);
        let request_state =
            state.get_string_or_default_to_unknown(&key("request_state"), &log_target);

        if request_trigger {
            request_trigger = false;
            StateManager::set_sp_value(
                &mut con,
                &key("request_trigger"),
                &request_trigger.to_spvalue(),
            )
            .await;

            if request_state == ActionRequestState::Initial.to_string() {
                let command_type =
                    state.get_string_or_default_to_unknown(&key("command_type"), &log_target);
                let accelleration =
                    state.get_float_or_default_to_zero(&key("accelleration"), &log_target);
                let velocity = state.get_float_or_default_to_zero(&key("velocity"), &log_target);
                let global_acceleration_scaling = state
                    .get_float_or_default_to_zero(&key("global_acceleration_scaling"), &log_target);
                let global_velocity_scaling = state
                    .get_float_or_default_to_zero(&key("global_velocity_scaling"), &log_target);
                let use_execution_time =
                    state.get_bool_or_default_to_false(&key("use_execution_time"), &log_target);
                let execution_time =
                    state.get_float_or_default_to_zero(&key("execution_time"), &log_target);
                let use_blend_radius =
                    state.get_bool_or_default_to_false(&key("use_blend_radius"), &log_target);
                let blend_radius =
                    state.get_float_or_default_to_zero(&key("blend_radius"), &log_target);
                let use_joint_positions =
                    state.get_bool_or_default_to_false(&key("use_joint_positions"), &log_target);
                let force_threshold =
                    state.get_float_or_default_to_zero(&key("force_threshold"), &log_target);
                let use_preferred_joint_config = state
                    .get_bool_or_default_to_false(&key("use_preferred_joint_config"), &log_target);
                let use_payload =
                    state.get_bool_or_default_to_false(&key("use_payload"), &log_target);
                let payload = state.get_string_or_value(
                    &key("payload"),
                    Payload::default().to_string(),
                    &log_target,
                );
                let baseframe_id = state.get_string_or_value(
                    &key("baseframe_id"),
                    DEFAULT_BASEFRAME_ID.to_string(),
                    &log_target,
                );
                let faceplate_id = state.get_string_or_value(
                    &key("faceplate_id"),
                    DEFAULT_FACEPLATE_ID.to_string(),
                    &log_target,
                );
                let goal_feature_id =
                    state.get_string_or_default_to_unknown(&key("goal_feature_id"), &log_target);
                let tcp_id = state.get_string_or_default_to_unknown(&key("tcp_id"), &log_target);
                let _root_frame_id = state.get_string_or_value(
                    &key("root_frame_id"),
                    DEFAULT_ROOT_FRAME_ID.to_string(),
                    &log_target,
                );
                let use_relative_pose =
                    state.get_bool_or_default_to_false(&key("use_relative_pose"), &log_target);

                let extract_f64_array = |state_key: &str, default_arr: &[f64]| -> Vec<f64> {
                    if let Some(micro_sp::SPValue::Array(ArrayOrUnknown::Array(values))) =
                        state.get_value(state_key, &log_target)
                    {
                        values
                            .iter()
                            .enumerate()
                            .map(|(i, val)| {
                                if let micro_sp::SPValue::Float64(FloatOrUnknown::Float64(
                                    ordered_float,
                                )) = val
                                {
                                    ordered_float.into_inner()
                                } else {
                                    default_arr[i]
                                }
                            })
                            .collect()
                    } else {
                        default_arr.to_vec()
                    }
                };

                let joint_positions =
                    extract_f64_array(&key("joint_positions"), &SAFE_HOME_JOINT_STATE);
                let preferred_joint_config =
                    extract_f64_array(&key("preferred_joint_config"), &SAFE_HOME_JOINT_STATE);
                let relative_pose = extract_f64_array(&key("relative_pose"), &[0.0; 6]);

                let mut target_in_base = transform_to_string(&SPTransformStamped {
                    active_transform: true,
                    enable_transform: true,
                    time_stamp: SystemTime::now(),
                    parent_frame_id: "".to_string(),
                    child_frame_id: "".to_string(),
                    transform: SPTransform::default(),
                    metadata: MapOrUnknown::UNKNOWN,
                });

                let mut tcp_in_faceplate = target_in_base.clone();

                if !use_joint_positions
                    && !use_relative_pose
                    && command_type != "lock_rsp"
                    && command_type != "unlock_rsp"
                {
                    target_in_base = match TransformsManager::lookup_transform(
                        &mut con,
                        &baseframe_id,
                        &goal_feature_id,
                    )
                    .await
                    {
                        Ok(transform) => transform_to_string(&transform),
                        Err(_) => continue,
                    };

                    tcp_in_faceplate =
                        match TransformsManager::lookup_transform(&mut con, &faceplate_id, &tcp_id)
                            .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => continue,
                        };
                }

                let waypoints_raw_str =
                    state.get_string_or_default_to_unknown(&key("waypoints_raw"), &log_target);

                let waypoints_raw: Vec<WaypointRaw> =
                    if waypoints_raw_str != "UNKNOWN" && !waypoints_raw_str.is_empty() {
                        match base64::engine::general_purpose::STANDARD.decode(&waypoints_raw_str) {
                            Ok(decoded_bytes) => {
                                let json_str = String::from_utf8_lossy(&decoded_bytes);
                                match serde_json::from_str(&json_str) {
                                    Ok(parsed_waypoints_raw) => parsed_waypoints_raw,
                                    Err(e) => {
                                        println!(
                                            "Failed to parse waypoints JSON for {}: {}",
                                            robot_name, e
                                        );
                                        vec![]
                                    }
                                }
                            }
                            Err(e) => {
                                println!(
                                    "Failed to decode Base64 waypoints for {}: {}",
                                    robot_name, e
                                );
                                vec![]
                            }
                        }
                    } else {
                        vec![]
                    };

                let mut waypoints = vec![];
                for wpr in waypoints_raw {
                    let mut wp_target_in_base = transform_to_string(&SPTransformStamped {
                        active_transform: true,
                        enable_transform: true,
                        time_stamp: SystemTime::now(),
                        parent_frame_id: "".to_string(),
                        child_frame_id: "".to_string(),
                        transform: SPTransform::default(),
                        metadata: MapOrUnknown::UNKNOWN,
                    });

                    let mut wp_tcp_in_faceplate = wp_target_in_base.clone();

                    if !wpr.use_joint_positions && !wpr.use_relative_pose {
                        wp_target_in_base = match TransformsManager::lookup_transform(
                            &mut con,
                            &wpr.baseframe_id,
                            &wpr.goal_feature_id,
                        )
                        .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => continue,
                        };

                        wp_tcp_in_faceplate = match TransformsManager::lookup_transform(
                            &mut con,
                            &wpr.faceplate_id,
                            &wpr.tcp_id,
                        )
                        .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => continue,
                        };
                    }

                    waypoints.push(Waypoint {
                        accelleration: wpr.accelleration,
                        velocity: wpr.velocity,
                        global_acceleration_scaling: wpr.global_acceleration_scaling,
                        global_velocity_scaling: wpr.global_velocity_scaling,
                        use_execution_time: wpr.use_execution_time,
                        execution_time: wpr.execution_time,
                        use_blend_radius: wpr.use_blend_radius,
                        blend_radius: wpr.blend_radius,
                        use_joint_positions: wpr.use_joint_positions,
                        joint_positions: wpr.joint_positions,
                        use_preferred_joint_config: wpr.use_preferred_joint_config,
                        preferred_joint_config: wpr.preferred_joint_config,
                        use_payload: wpr.use_payload,
                        payload: wpr.payload,
                        target_in_base: wp_target_in_base,
                        relative_pose: wpr.relative_pose,
                        tcp_in_faceplate: wp_tcp_in_faceplate,
                        force_threshold: wpr.force_threshold,
                    });
                }

                let robot_command = RobotCommand {
                    command_type,
                    accelleration,
                    velocity,
                    global_acceleration_scaling,
                    global_velocity_scaling,
                    use_execution_time,
                    execution_time,
                    use_blend_radius,
                    blend_radius,
                    use_joint_positions,
                    joint_positions,
                    use_preferred_joint_config,
                    preferred_joint_config,
                    use_payload,
                    payload,
                    target_in_base,
                    tcp_in_faceplate,
                    force_threshold,
                    relative_pose,
                    waypoints,
                };

                println!(
                    "Sending {} waypoints to Tera for template {}",
                    robot_command.waypoints.len(),
                    robot_command.command_type
                );

                let script = match generate_core_script_from_template(
                    robot_name,
                    robot_command,
                    templates,
                ) {
                    Ok(script) => script,
                    Err(_) => {
                        log::error!(target: &&format!("robot"), 
                                "Failed to generate UR Script.");
                        continue;
                    }
                };

                let local_addr = local_addr.borrow().clone();

                // For now just generate a uuid for each request here, but ideally from upstream
                let uuid = nanoid::nanoid!(10, &NANOID_ALPHABET);
                if local_addr.is_none() || !driver_state.lock().unwrap().connected {
                    println!("Not connected to robot yet, rejecting request: {}", uuid);
                    publish_script_result(&uuid, false);
                    continue;
                }
                let local_addr_str = local_addr.unwrap().ip().to_string();

                if driver_state.lock().unwrap().robot_state != 1 {
                    println!("Robot not in normal mode, rejecting request: {}", uuid);
                    publish_script_result(&uuid, false);
                    continue;
                }

                if driver_state.lock().unwrap().goal_id.is_some() {
                    println!("Already have an active goal, rejecting request: {}", uuid);
                    publish_script_result(&uuid, false);
                    continue;
                }

                println!("Accepting goal request with goal id: {}", uuid);

                // Note: If you want cancellation, you must hook `cancel_sender` up to your custom interface.
                let (cancel_sender, cancel_receiver) = mpsc::channel(1);

                {
                    let mut ds = driver_state.lock().unwrap();
                    ds.cancel_sender = Some(cancel_sender);
                }

                let req = ScriptRequest { uuid, script };

                let task_ur_address = ur_address.to_string().clone();
                let task_dashboard_commands = dashboard_commands.clone();
                let task_driver_state = driver_state.clone();

                let con_clone = con.clone();
                // let keys_clone = keys.clone();
                let robot_name_clone = robot_name.to_string().clone();
                local_pool.spawn_pinned(move || async {
                    let result = handle_request(
                        task_ur_address,
                        robot_name_clone,
                        local_addr_str,
                        task_driver_state,
                        task_dashboard_commands,
                        req,
                        cancel_receiver,
                        con_clone,
                    )
                    .await;

                    if let Err(e) = result {
                        println!("Error while handing goal: {}", e);
                    }
                });

                // call the urscript driver here
            }
        }
    }
}

/// TODO: Implement this to handle script execution feedback (e.g., standard output/errors).
pub fn publish_script_feedback(uuid: &str, feedback: &str) {
    println!("Script [{}] Feedback: {}", uuid, feedback);
}

/// TODO: Implement this to handle script completion results.
pub fn publish_script_result(uuid: &str, success: bool) {
    println!("Script [{}] Result: {}", uuid, success);
}

/// TODO: Implement this to yield new script requests from your custom interface.
async fn wait_for_script_request() -> Option<ScriptRequest> {
    // Example: Read from a custom channel or API
    std::future::pending().await
}
