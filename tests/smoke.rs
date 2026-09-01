use ur_redis_driver::*;

#[test]
fn parses_controller_replies() {
    // Label prefixes the controller actually sends.
    assert_eq!(RobotMode::parse("Robotmode: RUNNING"), RobotMode::Running);
    assert_eq!(RobotMode::parse("POWER_OFF"), RobotMode::PowerOff);
    assert_eq!(SafetyMode::parse("Safetymode: PROTECTIVE_STOP"), SafetyMode::ProtectiveStop);
    assert_eq!(
        SafetyStatus::parse("Safetystatus: AUTOMATIC_MODE_SAFEGUARD_STOP"),
        SafetyStatus::AutomaticModeSafeguardStop
    );
    assert_eq!(OperationalMode::parse("NONE"), OperationalMode::None);
    assert_eq!(OperationalMode::parse("MANUAL"), OperationalMode::Manual);

    // programState carries the program name after the state.
    let (state, program) = ProgramState::parse("STOPPED myprog.urp");
    assert_eq!(state, ProgramState::Stopped);
    assert_eq!(program.as_deref(), Some("myprog.urp"));
    let (state, program) = ProgramState::parse("PAUSED");
    assert_eq!(state, ProgramState::Paused);
    assert_eq!(program, None);

    // RT offsets must agree with the dashboard spellings.
    assert_eq!(SafetyMode::from_rt(8), SafetyMode::Violation);
    assert_eq!(RobotMode::from_rt(-1), RobotMode::NoController);
    assert_eq!(safety_mode_name(3), "PROTECTIVE_STOP");
    assert_eq!(robot_mode_name(7), "RUNNING");
    assert!(safety_mode_aborts_goal(3) && !safety_mode_aborts_goal(2));
    assert!(safety_mode_accepts_goal(1) && safety_mode_accepts_goal(2));
}

#[test]
fn typed_query_replies() {
    let running = DashboardCommand::IsProgramRunning.parse_reply("Program running: true");
    assert_eq!(running.and_then(|v| v.as_flag()), Some(true));

    let remote = DashboardCommand::IsInRemoteControl.parse_reply("false");
    assert_eq!(remote.and_then(|v| v.as_flag()), Some(false));

    // isProgramSaved answers "true <program.name>".
    let saved = DashboardCommand::IsProgramSaved.parse_reply("true myprog.urp");
    assert_eq!(saved.and_then(|v| v.as_flag()), Some(true));

    let loaded = DashboardCommand::GetLoadedProgram.parse_reply("Loaded program: /programs/a.urp");
    assert_eq!(
        loaded.as_ref().and_then(|v| v.as_text()),
        Some("/programs/a.urp")
    );

    // An action has nothing to type.
    assert!(DashboardCommand::Stop.parse_reply("Stopped").is_none());
}

#[test]
fn command_wire_and_parse() {
    let cases = [
        ("stop", "", "stop"),
        ("pause", "", "pause"),
        ("resume", "", "play"),
        ("power_on", "", "power on"),
        ("brake_release", "", "brake release"),
        ("unlock_protective_stop", "", "unlock protective stop"),
        ("reset_protective_stop", "", "unlock protective stop"),
        ("restart_safety", "", "restart safety"),
        ("close_popup", "", "close popup"),
        ("close_safety_popup", "", "close safety popup"),
        ("set_operational_mode", "automatic", "set operational mode automatic"),
        ("get_operational_mode", "", "get operational mode"),
        ("clear_operational_mode", "", "clear operational mode"),
        ("safety_mode", "", "safetymode"),
        ("safety_status", "", "safetystatus"),
        ("is_program_saved", "", "isProgramSaved"),
        ("get_serial_number", "", "get serial number"),
        ("version", "", "version"),
        ("generate_flight_report", "controller", "generate flight report controller"),
        ("generate_flight_report", "", "generate flight report system"),
        ("generate_support_file", "/programs/usb", "generate support file /programs/usb"),
        ("load", "a.urp", "load a.urp"),
    ];
    for (name, arg, wire) in cases {
        let cmd = DashboardCommand::parse(name, arg)
            .unwrap_or_else(|e| panic!("'{}' should parse: {}", name, e));
        assert_eq!(cmd.wire(), wire, "wire for '{}'", name);
    }

    // Bad or missing arguments are reported, not silently accepted.
    assert!(DashboardCommand::parse("set_operational_mode", "sideways").is_err());
    assert!(DashboardCommand::parse("load", "").is_err());
    assert!(DashboardCommand::parse("generate_support_file", "").is_err());
    assert!(DashboardCommand::parse("not_a_command", "").is_err());
    // `quit` is deliberately unreachable: it would close the driver's socket.
    assert!(DashboardCommand::parse("quit", "").is_err());
}

#[test]
fn per_command_timeouts() {
    use std::time::Duration;
    // The bug this fixes: `load` used to share a 2 s ceiling with every query.
    assert!(DashboardCommand::Load("a.urp".into()).reply_timeout() > Duration::from_secs(20));
    assert!(
        DashboardCommand::GenerateSupportFile("/d".into()).reply_timeout()
            > Duration::from_secs(300)
    );
    assert_eq!(DashboardCommand::RobotMode.reply_timeout(), Duration::from_secs(2));
    assert!(DashboardCommand::PowerOn.reply_timeout() > Duration::from_secs(2));

    assert!(DashboardCommand::Resume.requires_remote_control());
    assert!(!DashboardCommand::RobotMode.requires_remote_control());
}

/// The templates must still render, with and without waypoint progress.
#[test]
fn trajectory_templates_render() {
    let templates = tera::Tera::new("templates/*.script").expect("templates parse");

    let waypoint = || Waypoint {
        acceleration: 1.0,
        velocity: 0.5,
        global_acceleration_scaling: 1.0,
        global_velocity_scaling: 1.0,
        use_execution_time: false,
        execution_time: 0.0,
        use_blend_radius: true,
        blend_radius: 0.05,
        use_joint_positions: true,
        joint_positions: vec![0.0; 6],
        use_preferred_joint_config: false,
        preferred_joint_config: vec![0.0; 6],
        use_payload: false,
        payload: String::new(),
        target_in_base: "p[0,0,0,0,0,0]".to_string(),
        relative_pose: vec![0.0; 6],
        tcp_in_faceplate: "p[0,0,0,0,0,0]".to_string(),
        force_threshold: 0.0,
        use_linear_motion: false,
    };

    let command = |report: bool| RobotCommand {
        command_type: "trajectory_unsafe_move_j".to_string(),
        acceleration: 1.0,
        velocity: 0.5,
        global_acceleration_scaling: 1.0,
        global_velocity_scaling: 1.0,
        use_execution_time: false,
        execution_time: 0.0,
        use_blend_radius: true,
        blend_radius: 0.05,
        use_joint_positions: true,
        joint_positions: vec![0.0; 6],
        use_preferred_joint_config: false,
        preferred_joint_config: vec![0.0; 6],
        use_payload: false,
        payload: String::new(),
        target_in_base: "p[0,0,0,0,0,0]".to_string(),
        relative_pose: vec![0.0; 6],
        tcp_in_faceplate: "p[0,0,0,0,0,0]".to_string(),
        force_threshold: 0.0,
        report_waypoint_progress: report,
        trajectory_id: String::new(),
        waypoints: vec![waypoint(), waypoint(), waypoint()],
    };

    let on = generate_core_script_from_template("r1", command(true), &templates).unwrap();
    assert_eq!(on.matches("waypoint_reached").count(), 3, "one line per waypoint:\n{}", on);
    assert!(on.contains("waypoint_reached 0") && on.contains("waypoint_reached 2"));

    let off = generate_core_script_from_template("r1", command(false), &templates).unwrap();
    assert!(!off.contains("waypoint_reached"), "progress must be off:\n{}", off);

    // Every other template must still render too.
    for name in ["unsafe_move_j", "unsafe_move_l", "trajectory_unsafe_move_l"] {
        let mut c = command(true);
        c.command_type = name.to_string();
        generate_core_script_from_template("r1", c, &templates)
            .unwrap_or_else(|e| panic!("{} failed to render: {}", name, e));
    }
}
