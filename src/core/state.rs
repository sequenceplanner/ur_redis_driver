use micro_sp::*;

pub fn generate_robot_interface_state(robot_name: &str, log_target: &str) -> State {
    let state = State::new();

    let request_trigger = bv!(&&format!("{}_request_trigger", robot_name));
    let request_state = v!(&&format!("{}_request_state", robot_name));
    let request_cancel = bv!(&&format!("{}_request_cancel", robot_name));
    // `command_server` has always read `request_result` and `request_feedback`, but
    // neither was ever seeded here, so both silently fell through to defaults and
    // no caller could see why a request failed.
    let request_result = v!(&&format!("{}_request_result", robot_name));
    let request_feedback = v!(&&format!("{}_request_feedback", robot_name));
    let dashboard_request_trigger = bv!(&&format!("{}_dashboard_request_trigger", robot_name));
    let dashboard_request_state = v!(&&format!("{}_dashboard_request_state", robot_name));
    let dashboard_request_cancel = bv!(&&format!("{}_dashboard_request_cancel", robot_name));
    let dashboard_command = v!(&&format!("{}_dashboard_command", robot_name));
    let dashboard_command_arg = v!(&&format!("{}_dashboard_command_arg", robot_name));
    let dashboard_request_result = v!(&&format!("{}_dashboard_request_result", robot_name));
    let total_fail_counter = iv!(&&format!("{}_total_fail_counter", robot_name));
    let subsequent_fail_counter = iv!(&&format!("{}_subsequent_fail_counter", robot_name));

    let state = state.add(assign!(request_trigger, false.to_spvalue()), &log_target);
    let state = state.add(assign!(request_state, "initial".to_spvalue()), &log_target);
    let state = state.add(assign!(request_cancel, false.to_spvalue()), &log_target);
    let state = state.add(assign!(request_result, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(request_feedback, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(dashboard_request_trigger, false.to_spvalue()), &log_target);
    let state = state.add(assign!(dashboard_request_state, "initial".to_spvalue()), &log_target);
    let state = state.add(assign!(dashboard_request_cancel, false.to_spvalue()), &log_target);
    let state = state.add(assign!(dashboard_command, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(dashboard_command_arg, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(dashboard_request_result, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(total_fail_counter, 0.to_spvalue()), &log_target);
    let state = state.add(assign!(subsequent_fail_counter, 0.to_spvalue()), &log_target);

    let command_type = v!(&&format!("{}_command_type", robot_name));
    let acceleration = fv!(&&format!("{}_acceleration", robot_name));
    let velocity = fv!(&&format!("{}_velocity", robot_name));
    let global_acceleration_scaling = fv!(&&format!("{}_global_acceleration_scaling", robot_name));
    let global_velocity_scaling = fv!(&&format!("{}_global_velocity_scaling", robot_name));
    let use_execution_time = bv!(&&format!("{}_use_execution_time", robot_name));
    let execution_time = fv!(&&format!("{}_execution_time", robot_name));
    let use_blend_radius = bv!(&&format!("{}_use_blend_radius", robot_name));
    let blend_radius = fv!(&&format!("{}_blend_radius", robot_name));
    let use_joint_positions = bv!(&&format!("{}_use_joint_positions", robot_name));
    let joint_positions = av!(&&format!("{}_joint_positions", robot_name));
    let joint_states = av!(&&format!("{}_joint_states", robot_name));
    let use_preferred_joint_config = bv!(&&format!("{}_use_preferred_joint_config", robot_name));
    let preferred_joint_config = av!(&&format!("{}_preferred_joint_config", robot_name));
    let use_payload = bv!(&&format!("{}_use_payload", robot_name));
    let payload = v!(&&format!("{}_payload", robot_name));
    let baseframe_id = v!(&&format!("{}_baseframe_id", robot_name));
    let faceplate_id = v!(&&format!("{}_faceplate_id", robot_name));
    let goal_feature_id = v!(&&format!("{}_goal_feature_id", robot_name));
    let tcp_id = v!(&&format!("{}_tcp_id", robot_name));
    let root_frame_id = v!(&&format!("{}_root_frame_id", robot_name));
    let cancel_current_goal = bv!(&&format!("{}_cancel_current_goal", robot_name));
    let force_threshold = fv!(&&format!("{}_force_threshold", robot_name));
    let force_feedback = fv!(&&format!("{}_force_feedback", robot_name));
    let estimated_position = v!(&&format!("{}_estimated_position", robot_name));
    let use_relative_pose = bv!(&&format!("{}_use_relative_pose", robot_name));
    let relative_pose = av!(&&format!("{}_relative_pose", robot_name));
    let gripper_force = fv!(&&format!("{}_gripper_force", robot_name));
    let gripper_velocity = fv!(&&format!("{}_gripper_velocity", robot_name));
    let gripper_ref_pos_percentage = iv!(&&format!("{}_gripper_ref_pos_percentage", robot_name));
    let waypoints = av!(&&format!("{}_waypoints", robot_name));
    // Whether the trajectory templates report each waypoint they reach. Only a
    // reported trajectory can resume mid-path; see `RobotCommand`.
    let report_waypoint_progress =
        bv!(&&format!("{}_report_waypoint_progress", robot_name));

    // Measured state, written by `state_publisher` from what `realtime_reader`
    // decodes off the RT stream. None of this used to leave the process.
    let safety_mode = v!(&&format!("{}_safety_mode", robot_name));
    let robot_mode = v!(&&format!("{}_robot_mode", robot_name));
    let program_state = v!(&&format!("{}_program_state", robot_name));
    let program_running = bv!(&&format!("{}_program_running", robot_name));
    let tcp_pose = av!(&&format!("{}_tcp_pose", robot_name));
    let tcp_force = av!(&&format!("{}_tcp_force", robot_name));
    let speed_scaling = fv!(&&format!("{}_speed_scaling", robot_name));
    let digital_inputs = iv!(&&format!("{}_digital_inputs", robot_name));
    let digital_outputs = iv!(&&format!("{}_digital_outputs", robot_name));
    let robot_connected = bv!(&&format!("{}_robot_connected", robot_name));
    let dashboard_connected = bv!(&&format!("{}_dashboard_connected", robot_name));
    let remote_control = bv!(&&format!("{}_remote_control", robot_name));
    let operational_mode = v!(&&format!("{}_operational_mode", robot_name));
    let motion_paused = bv!(&&format!("{}_motion_paused", robot_name));
    // Read once per dashboard connection rather than polled.
    let robot_model = v!(&&format!("{}_robot_model", robot_name));
    let serial_number = v!(&&format!("{}_serial_number", robot_name));
    let polyscope_version = v!(&&format!("{}_polyscope_version", robot_name));

    let state = state.add(assign!(command_type, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(acceleration, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(velocity, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(global_acceleration_scaling, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(global_velocity_scaling, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_execution_time, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(execution_time, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_blend_radius, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(blend_radius, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_joint_positions, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(joint_positions, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(joint_states, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_preferred_joint_config, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(preferred_joint_config, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_payload, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(payload, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(baseframe_id, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(faceplate_id, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(goal_feature_id, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(tcp_id, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(root_frame_id, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(cancel_current_goal, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(estimated_position, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(force_threshold, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(force_feedback, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(use_relative_pose, SPValue::Bool(BoolOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(relative_pose, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(gripper_force, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(gripper_velocity, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(gripper_ref_pos_percentage, SPValue::Int64(IntOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(waypoints, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    // Seeded true, matching the default `command_server` falls back to.
    let state = state.add(assign!(report_waypoint_progress, true.to_spvalue()), &log_target);

    let state = state.add(assign!(safety_mode, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(robot_mode, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(program_state, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(program_running, false.to_spvalue()), &log_target);
    let state = state.add(assign!(tcp_pose, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(tcp_force, SPValue::Array(ArrayOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(speed_scaling, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(digital_inputs, SPValue::Int64(IntOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(digital_outputs, SPValue::Int64(IntOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(robot_connected, false.to_spvalue()), &log_target);
    let state = state.add(assign!(dashboard_connected, false.to_spvalue()), &log_target);
    let state = state.add(assign!(remote_control, false.to_spvalue()), &log_target);
    let state = state.add(assign!(operational_mode, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(motion_paused, false.to_spvalue()), &log_target);
    let state = state.add(assign!(robot_model, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(serial_number, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(polyscope_version, SPValue::String(StringOrUnknown::UNKNOWN)), &log_target);

    state
}

pub fn generate_gripper_interface_state(gripper_id: &str, log_target: &str) -> State {
    let state = State::new();

    let request_trigger = bv!(&&format!("{}_request_trigger", gripper_id));
    let request_state = v!(&&format!("{}_request_state", gripper_id));
    let command_type = v!(&&format!("{}_command_type", gripper_id));
    let velocity = fv!(&&format!("{}_velocity", gripper_id));
    let force = fv!(&&format!("{}_force", gripper_id));
    let ref_pos_percentage = iv!(&&format!("{}_ref_pos_percentage", gripper_id));

    let state = state.add(assign!(request_trigger, false.to_spvalue()), &log_target);
    let state = state.add(assign!(request_state, "initial".to_spvalue()), &log_target);
    let state = state.add(assign!(
        command_type,
        SPValue::String(StringOrUnknown::UNKNOWN)
    ), &log_target);
    let state = state.add(assign!(velocity, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(force, SPValue::Float64(FloatOrUnknown::UNKNOWN)), &log_target);
    let state = state.add(assign!(
        ref_pos_percentage,
        SPValue::Int64(IntOrUnknown::UNKNOWN)
    ), &log_target);

    state
}