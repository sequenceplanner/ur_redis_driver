use std::{
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};

// use std::net::TcpStream;
// use std::io;

use futures::StreamExt;
use micro_sp::*;
use tokio::time::interval;

// use crate::core::structs::{transform_to_string, CommandType, Payload};
use crate::*;

pub const UR_ACTION_SERVER_TICKER_RATE: u64 = 250;
pub static SAFE_HOME_JOINT_STATE: [f64; 6] = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0];
pub static DEFAULT_BASEFRAME_ID: &'static str = "base_link"; // base_link if simulation, base if real or ursim
pub static DEFAULT_FACEPLATE_ID: &'static str = "tool0";
// pub static DEFAULT_TCP_ID: &'static str = "svt_tcp";
pub static DEFAULT_ROOT_FRAME_ID: &'static str = "world";

pub async fn command_server(
    _ur_address: &str,
    robot_name: &str,
    connection_manager: &Arc<ConnectionManager>,
    templates: &tera::Tera,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = format!("{robot_name}_action_client");
    let mut interval = interval(Duration::from_millis(ROBOT_STATE_UPDATE_INTERVAL_MS.into()));

    let suffixes = [
        "request_trigger",
        "request_state",
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

        let mut request_trigger =
            state.get_bool_or_default_to_false(&key("request_trigger"), &log_target);
        let mut request_state =
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
                };

                let _script = match generate_core_script_from_template(
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

                // call the urscript driver here
            }

            StateManager::set_sp_value(
                &mut con,
                &key("request_state"),
                &request_state.to_spvalue(),
            )
            .await;
            StateManager::set_sp_value(
                &mut con,
                &key("request_trigger"),
                &request_trigger.to_spvalue(),
            )
            .await;
        }
    }
}