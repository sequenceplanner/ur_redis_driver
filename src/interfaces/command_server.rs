use std::{
    net::SocketAddr,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime},
};

use micro_sp::*;
use tokio::{
    sync::{mpsc, oneshot, watch},
    time::{MissedTickBehavior, interval},
};
use tokio_util::task::LocalPoolHandle;

use crate::{driver::handle_request::handle_request, *};

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
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<DashboardReply>)>,
    // `Arc` rather than a reference: `handle_request` needs its own handle to
    // re-render the remainder of a motion when a paused goal resumes, and it runs
    // as a spawned task, so it cannot borrow.
    templates: Arc<tera::Tera>,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = format!("{robot_name}_action_client");

    // Tokio's default missed-tick behaviour is `Burst`: if one tick overruns the
    // period the interval then fires back-to-back, with no delay, until it has
    // caught up. At a 10 ms period one slow Redis reply is enough to turn this
    // loop into a spin that starves every other task on the runtime. `Delay`
    // keeps a full period between ticks, so a loop that cannot keep up simply
    // runs slower.
    let mut interval = interval(Duration::from_millis(ROBOT_STATE_UPDATE_INTERVAL_MS.into()));
    interval.set_missed_tick_behavior(MissedTickBehavior::Delay);

    let local_pool = LocalPoolHandle::new(1);

    let suffixes = [
        "request_trigger",
        "request_state",
        "request_result",
        "request_cancel",
        "request_feedback",
        "command_type",
        "acceleration",
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
        "report_waypoint_progress",
        "waypoints",
    ];

    let keys: Vec<String> = suffixes
        .iter()
        .map(|s| format!("{robot_name}_{s}"))
        .collect();

    // Nothing in the loop body happens unless one of these two is set, so an
    // idle tick reads just these instead of the whole request key set above.
    let fast_keys: Vec<String> = ["request_cancel", "request_trigger"]
        .iter()
        .map(|s| format!("{robot_name}_{s}"))
        .collect();

    // One long-lived handle for the whole task rather than one per tick, and no
    // pre-flight PING before the real work. `SPConnection` is cheap to clone,
    // multiplexed and self-healing, so this handle stays valid across
    // reconnects; a dropped socket surfaces as an error on the command itself,
    // which the callee already logs, and skipping the tick is the right answer.
    let mut con = connection_manager.get_connection().await;

    let key = |suffix: &str| format!("{robot_name}_{suffix}");

    loop {
        interval.tick().await;

        // Idle tick: two keys instead of the full set.
        let flags = match StateManager::get_state_for_keys(&mut con, &fast_keys, &log_target).await
        {
            Some(s) => s,
            None => continue,
        };
        // These two keys are written by whoever drives the driver, so a malformed
        // value here is an input error, not an invariant violation. The plain
        // accessors panic on a key that failed to deserialize; these do not.
        let cancel_goal = state_bool_or(&flags, &key("request_cancel"), false, &log_target);
        let triggered = state_bool_or(&flags, &key("request_trigger"), false, &log_target);
        if !cancel_goal && !triggered {
            continue;
        }

        let state = match StateManager::get_state_for_keys(&mut con, &keys, &log_target).await {
            Some(s) => s,
            None => continue,
        };

        let cancel_goal = state_bool_or(&state, &key("request_cancel"), false, &log_target);

        if cancel_goal {
            StateManager::set_sp_value(&mut con, &key("request_cancel"), &false.to_spvalue()).await;

            let cancel_sender = lock_driver_state(&driver_state).cancel_sender.take();
            match cancel_sender {
                Some(sender) => {
                    log::info!(target: &log_target, "Cancel requested, aborting the active script.");
                    let _ = sender.try_send(());
                }
                None => {
                    log::warn!(target: &log_target, "Cancel requested, but no goal is running.");
                }
            }
        }

        let mut request_trigger =
            state_bool_or(&state, &key("request_trigger"), false, &log_target);
        let request_state = state_string_or(
            &state,
            &key("request_state"),
            &ActionRequestState::UNKNOWN.to_string(),
            &log_target,
        );

        if request_trigger {
            request_trigger = false;
            StateManager::set_sp_value(
                &mut con,
                &key("request_trigger"),
                &request_trigger.to_spvalue(),
            )
            .await;

            if request_state == ActionRequestState::Initial.to_string() {
                // Every accessor below panics on a key that is absent from the
                // fetched state, and `build_state` drops any key whose stored value
                // will not deserialize. Checking the whole set once turns a
                // malformed request parameter into a failed request instead of a
                // dead driver, and reports every bad key at once rather than the
                // first one.
                let missing: Vec<&str> = keys
                    .iter()
                    .filter(|k| !state.contains(k))
                    .map(|k| k.as_str())
                    .collect();
                if !missing.is_empty() {
                    fail_request(
                        &mut con,
                        robot_name,
                        &format!("missing or unreadable request keys: {}", missing.join(", ")),
                        &log_target,
                    )
                    .await;
                    continue;
                }

                let command_type =
                    state_string_or(&state, &key("command_type"), "UNKNOWN", &log_target);

                // The command type is interpolated straight into a template
                // filename, so an unrecognised one has to be rejected here rather
                // than left to fail inside Tera - both to keep an arbitrary Redis
                // string out of a path, and because a render error further down
                // used to drop the request without ever answering the caller.
                let template_name = format!("{}.script", command_type);
                if !templates.get_template_names().any(|n| n == template_name) {
                    fail_request(
                        &mut con,
                        robot_name,
                        &format!("unknown command_type '{}'", command_type),
                        &log_target,
                    )
                    .await;
                    continue;
                }

                let acceleration =
                    state.get_float_or_default_to_zero(&key("acceleration"), &log_target);
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
                // Defaults to true: a trajectory that can be resumed where it
                // stopped is the more useful default, and the flag exists to turn
                // the reporting off if the socket write turns out to break blending
                // on this controller.
                let report_waypoint_progress = state.get_bool_or_value(
                    &key("report_waypoint_progress"),
                    true,
                    &log_target,
                );

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
                    && command_type != "trajectory_unsafe_move_j"
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
                        Err(_) => {
                            fail_request(
                                &mut con,
                                robot_name,
                                &format!(
                                    "no transform from '{}' to '{}'",
                                    baseframe_id, goal_feature_id
                                ),
                                &log_target,
                            )
                            .await;
                            continue;
                        }
                    };

                    tcp_in_faceplate =
                        match TransformsManager::lookup_transform(&mut con, &faceplate_id, &tcp_id)
                            .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => {
                                fail_request(
                                    &mut con,
                                    robot_name,
                                    &format!(
                                        "no transform from '{}' to '{}'",
                                        faceplate_id, tcp_id
                                    ),
                                    &log_target,
                                )
                                .await;
                                continue;
                            }
                        };
                }

                let waypoints_sp = state.get_value(&key("waypoints"), &log_target);
                let waypoints_raw = match waypoints_sp {
                    // Absent or non-array simply means "no waypoints", which every
                    // non-trajectory command is.
                    None | Some(SPValue::Array(ArrayOrUnknown::UNKNOWN)) => vec![],
                    other => match WaypointRaw::vec_from_sp_value(other, &log_target) {
                        Some(waypoints) => waypoints,
                        None => {
                            fail_request(
                                &mut con,
                                robot_name,
                                "one or more waypoints failed to decode",
                                &log_target,
                            )
                            .await;
                            continue;
                        }
                    },
                };

                let mut waypoints = Vec::with_capacity(waypoints_raw.len());
                let mut waypoint_error = None;
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

                    if !wpr.use_joint_positions && !wpr.use_relative_pose {
                        wp_target_in_base = match TransformsManager::lookup_transform(
                            &mut con,
                            &baseframe_id,
                            &wpr.goal_feature_id,
                        )
                        .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => {
                                waypoint_error = Some(format!(
                                    "no transform from '{}' to waypoint frame '{}'",
                                    baseframe_id, wpr.goal_feature_id
                                ));
                                break;
                            }
                        };

                        tcp_in_faceplate = match TransformsManager::lookup_transform(
                            &mut con,
                            &faceplate_id,
                            &tcp_id,
                        )
                        .await
                        {
                            Ok(transform) => transform_to_string(&transform),
                            Err(_) => {
                                waypoint_error = Some(format!(
                                    "no transform from '{}' to '{}'",
                                    faceplate_id, tcp_id
                                ));
                                break;
                            }
                        };
                    }

                    waypoints.push(Waypoint {
                        acceleration: wpr.acceleration,
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
                        tcp_in_faceplate: tcp_in_faceplate.clone(), // we dont want to change tcps in a blended move
                        force_threshold: wpr.force_threshold,
                    });
                }

                // A blended trajectory with a waypoint missing is a different path,
                // not a shorter one, so a failed lookup fails the whole request.
                if let Some(reason) = waypoint_error {
                    fail_request(&mut con, robot_name, &reason, &log_target).await;
                    continue;
                }

                let robot_command = RobotCommand {
                    command_type,
                    acceleration,
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
                    report_waypoint_progress,
                    waypoints,
                };

                let script = match generate_core_script_from_template(
                    robot_name,
                    robot_command.clone(),
                    &templates,
                ) {
                        Ok(script) => script,
                        Err(e) => {
                            fail_request(
                                &mut con,
                                robot_name,
                                &format!("failed to render the UR Script template: {}", e),
                                &log_target,
                            )
                            .await;
                            continue;
                        }
                    };

                let local_addr = local_addr.borrow().clone();

                // For now just generate a uuid for each request here, but ideally from upstream
                let uuid = nanoid::nanoid!(10, &NANOID_ALPHABET);

                // Admission control. Each rejection has to write a terminal state:
                // the trigger was already consumed above, so a caller waiting on
                // `request_state` would otherwise sit at `initial` forever.
                let rejection = {
                    let ds = lock_driver_state(&driver_state);
                    if local_addr.is_none() || !ds.connected {
                        Some("not connected to the robot".to_string())
                    } else if ds.motion_paused {
                        // An operator has deliberately held the robot. Starting a
                        // new move into that hold would be a surprise, and `resume`
                        // would then resume the wrong goal.
                        Some("motion is paused".to_string())
                    } else if !safety_mode_accepts_goal(ds.safety_mode) {
                        Some(format!(
                            "robot safety mode is {}",
                            safety_mode_name(ds.safety_mode)
                        ))
                    } else if ds.goal_id.is_some() || ds.reissuing {
                        // `reissuing` covers the gap between a paused script being
                        // killed and its replacement being accepted, during which
                        // `goal_id` is momentarily clear but the goal is very much
                        // still live.
                        Some("a goal is already running".to_string())
                    } else {
                        None
                    }
                };

                if let Some(reason) = rejection {
                    log::warn!(target: &log_target, "Rejecting request {}: {}.", uuid, reason);
                    fail_request(&mut con, robot_name, &reason, &log_target).await;
                    continue;
                }

                // Checked as `Some` in the block above.
                let local_addr_str = local_addr.unwrap().ip().to_string();

                log::info!(target: &log_target, "Accepting goal request with goal id: {}", uuid);

                StateManager::set_sp_value(
                    &mut con,
                    &key("request_state"),
                    &ActionRequestState::Executing.to_string().to_spvalue(),
                )
                .await;

                // Note: If you want cancellation, you must hook `cancel_sender` up to your custom interface.
                let (cancel_sender, cancel_receiver) = mpsc::channel(1);
                // Signalled by the dashboard task when a `play` fails to resume the
                // injected script. Kept in `DriverState` for the life of the goal
                // rather than taken, because a goal can be paused and resumed more
                // than once.
                let (resume_sender, resume_receiver) = mpsc::channel(1);

                {
                    let mut ds = lock_driver_state(&driver_state);
                    ds.cancel_sender = Some(cancel_sender);
                    ds.resume_sender = Some(resume_sender);
                    ds.active_command = Some(robot_command.clone());
                    ds.waypoints_completed = 0;
                }

                let req = ScriptRequest { uuid, script, command: robot_command };

                let task_ur_address = ur_address.to_string().clone();
                let task_dashboard_commands = dashboard_commands.clone();
                let task_driver_state = driver_state.clone();

                let con_clone = con.clone();
                let robot_name_clone = robot_name.to_string().clone();
                let task_templates = templates.clone();
                local_pool.spawn_pinned(move || async {
                    let result = handle_request(
                        task_ur_address,
                        robot_name_clone,
                        local_addr_str,
                        task_driver_state,
                        task_dashboard_commands,
                        req,
                        cancel_receiver,
                        resume_receiver,
                        task_templates,
                        con_clone,
                    )
                    .await;

                    if let Err(e) = result {
                        println!("Error while handing goal: {}", e);
                    }
                });
            }
        }
    }
}

/// Terminate a request that never made it to the robot.
///
/// Every early exit in the loop above has already consumed `request_trigger`, so
/// without this the caller is left watching a `request_state` that will never
/// leave `initial`. The reason is written to `request_result` so the failure is
/// diagnosable from Redis rather than only from this process's log.
pub async fn fail_request(
    con: &mut SPConnection,
    robot_name: &str,
    reason: &str,
    log_target: &str,
) {
    log::error!(target: log_target, "Request failed: {}.", reason);

    let counter_keys = vec![
        format!("{robot_name}_total_fail_counter"),
        format!("{robot_name}_subsequent_fail_counter"),
    ];

    // Read-modify-write on the failure path only, so the happy path pays nothing
    // for it.
    if let Some(counters) =
        StateManager::get_state_for_keys(con, &counter_keys, log_target).await
    {
        let total = state_int_or(&counters, &counter_keys[0], 0, log_target);
        let subsequent = state_int_or(&counters, &counter_keys[1], 0, log_target);
        StateManager::set_sp_value(con, &counter_keys[0], &(total + 1).to_spvalue()).await;
        StateManager::set_sp_value(con, &counter_keys[1], &(subsequent + 1).to_spvalue()).await;
    }

    publish_script_result(con, robot_name, reason, false).await;
}

/// Publish a feedback line emitted by the running UR Script.
pub async fn publish_script_feedback(
    con: &mut SPConnection,
    robot_name: &str,
    uuid: &str,
    feedback: &str,
) {
    log::info!(target: &format!("{robot_name}_action_client"), "Script [{}] feedback: {}", uuid, feedback);
    StateManager::set_sp_value(
        con,
        &format!("{robot_name}_request_feedback"),
        &feedback.to_spvalue(),
    )
    .await;
}

/// Publish the outcome of a request.
///
/// Only writes `request_result`; the caller owns `request_state`, because the two
/// have different writers on the success path (`handle_request`) and the rejection
/// path (`fail_request`).
pub async fn publish_script_result(
    con: &mut SPConnection,
    robot_name: &str,
    result: &str,
    success: bool,
) {
    StateManager::set_sp_value(
        con,
        &format!("{robot_name}_request_result"),
        &result.to_spvalue(),
    )
    .await;

    if !success {
        StateManager::set_sp_value(
            con,
            &format!("{robot_name}_request_state"),
            &ActionRequestState::Failed.to_string().to_spvalue(),
        )
        .await;
    }
}
