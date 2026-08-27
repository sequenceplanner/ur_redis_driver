//! The typed contract for the UR Dashboard Server on port 29999.
//!
//! Three groups of types live here:
//!
//! * [`DashboardCommand`] - everything the driver can *send*, one variant per
//!   command in `docs/DashboardServer_e-Series_2022.pdf`, with the wire spelling,
//!   the reply substring that means success, and how long the controller gets to
//!   answer.
//! * [`DashboardValue`] and the mode enums - everything a query can *return*,
//!   parsed out of the reply line instead of handed on as a bare `String`.
//! * [`DashboardStatus`] and [`DashboardIdentity`] - what the connection *polls*:
//!   the former every keepalive tick, the latter once per connection.

use std::time::Duration;

/// Strip a `"Label: value"` prefix if the controller sent one.
///
/// `robotmode` answers `"Robotmode: RUNNING"` and `running` answers
/// `"Program running: true"`, but `get operational mode` answers a bare
/// `"MANUAL"`. Splitting on the last colon covers both without a per-command rule.
fn payload_of(response: &str) -> &str {
    match response.rsplit_once(':') {
        Some((_, tail)) => tail.trim(),
        None => response.trim(),
    }
}

/// Parse a `"true"`/`"false"` answer, tolerating a label prefix and a trailing
/// program name (`isProgramSaved` answers `"true <program.name>"`).
fn parse_flag(response: &str) -> bool {
    payload_of(response)
        .split_whitespace()
        .next()
        .map(|v| v.eq_ignore_ascii_case("true"))
        .unwrap_or(false)
}

/// UR *robot* mode: the power/boot state of the arm.
///
/// Reported both by the dashboard `robotmode` query and by realtime packet offset
/// 756, hence [`RobotMode::from_rt`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RobotMode {
    NoController,
    Disconnected,
    ConfirmSafety,
    Booting,
    PowerOff,
    PowerOn,
    Idle,
    Backdrive,
    Running,
    UpdatingFirmware,
    Unknown,
}

impl RobotMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            RobotMode::NoController => "NO_CONTROLLER",
            RobotMode::Disconnected => "DISCONNECTED",
            RobotMode::ConfirmSafety => "CONFIRM_SAFETY",
            RobotMode::Booting => "BOOTING",
            RobotMode::PowerOff => "POWER_OFF",
            RobotMode::PowerOn => "POWER_ON",
            RobotMode::Idle => "IDLE",
            RobotMode::Backdrive => "BACKDRIVE",
            RobotMode::Running => "RUNNING",
            RobotMode::UpdatingFirmware => "UPDATING_FIRMWARE",
            RobotMode::Unknown => "UNKNOWN",
        }
    }

    /// Decode realtime packet offset 756.
    pub fn from_rt(mode: i32) -> Self {
        match mode {
            -1 => RobotMode::NoController,
            0 => RobotMode::Disconnected,
            1 => RobotMode::ConfirmSafety,
            2 => RobotMode::Booting,
            3 => RobotMode::PowerOff,
            4 => RobotMode::PowerOn,
            5 => RobotMode::Idle,
            6 => RobotMode::Backdrive,
            7 => RobotMode::Running,
            8 => RobotMode::UpdatingFirmware,
            _ => RobotMode::Unknown,
        }
    }

    /// Parse a dashboard `robotmode` reply, with or without its label prefix.
    pub fn parse(response: &str) -> Self {
        match payload_of(response).to_ascii_uppercase().as_str() {
            "NO_CONTROLLER" => RobotMode::NoController,
            "DISCONNECTED" => RobotMode::Disconnected,
            "CONFIRM_SAFETY" => RobotMode::ConfirmSafety,
            "BOOTING" => RobotMode::Booting,
            "POWER_OFF" => RobotMode::PowerOff,
            "POWER_ON" => RobotMode::PowerOn,
            "IDLE" => RobotMode::Idle,
            "BACKDRIVE" => RobotMode::Backdrive,
            "RUNNING" => RobotMode::Running,
            "UPDATING_FIRMWARE" => RobotMode::UpdatingFirmware,
            _ => RobotMode::Unknown,
        }
    }
}

/// UR *safety* mode.
///
/// Reported by realtime packet offset 812 and by the deprecated dashboard
/// `safetymode` query. [`SafetyStatus`] is the finer-grained dashboard-only
/// version of the same idea.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SafetyMode {
    Normal,
    Reduced,
    ProtectiveStop,
    Recovery,
    SafeguardStop,
    SystemEmergencyStop,
    RobotEmergencyStop,
    Violation,
    Fault,
    Unknown,
}

impl SafetyMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            SafetyMode::Normal => "NORMAL",
            SafetyMode::Reduced => "REDUCED",
            SafetyMode::ProtectiveStop => "PROTECTIVE_STOP",
            SafetyMode::Recovery => "RECOVERY",
            SafetyMode::SafeguardStop => "SAFEGUARD_STOP",
            SafetyMode::SystemEmergencyStop => "SYSTEM_EMERGENCY_STOP",
            SafetyMode::RobotEmergencyStop => "ROBOT_EMERGENCY_STOP",
            SafetyMode::Violation => "VIOLATION",
            SafetyMode::Fault => "FAULT",
            SafetyMode::Unknown => "UNKNOWN",
        }
    }

    /// Decode realtime packet offset 812.
    pub fn from_rt(mode: i32) -> Self {
        match mode {
            1 => SafetyMode::Normal,
            2 => SafetyMode::Reduced,
            3 => SafetyMode::ProtectiveStop,
            4 => SafetyMode::Recovery,
            5 => SafetyMode::SafeguardStop,
            6 => SafetyMode::SystemEmergencyStop,
            7 => SafetyMode::RobotEmergencyStop,
            8 => SafetyMode::Violation,
            9 => SafetyMode::Fault,
            _ => SafetyMode::Unknown,
        }
    }

    pub fn parse(response: &str) -> Self {
        match payload_of(response).to_ascii_uppercase().as_str() {
            "NORMAL" => SafetyMode::Normal,
            "REDUCED" => SafetyMode::Reduced,
            "PROTECTIVE_STOP" => SafetyMode::ProtectiveStop,
            "RECOVERY" => SafetyMode::Recovery,
            "SAFEGUARD_STOP" => SafetyMode::SafeguardStop,
            "SYSTEM_EMERGENCY_STOP" => SafetyMode::SystemEmergencyStop,
            "ROBOT_EMERGENCY_STOP" => SafetyMode::RobotEmergencyStop,
            "VIOLATION" => SafetyMode::Violation,
            "FAULT" => SafetyMode::Fault,
            _ => SafetyMode::Unknown,
        }
    }

    /// Whether an in-flight goal can no longer complete in this mode.
    ///
    /// Deliberately excludes `Reduced`, which is a normal operating mode: the robot
    /// slows down inside a reduced-speed zone but keeps running.
    pub fn aborts_goal(&self) -> bool {
        matches!(
            self,
            SafetyMode::ProtectiveStop
                | SafetyMode::SafeguardStop
                | SafetyMode::SystemEmergencyStop
                | SafetyMode::RobotEmergencyStop
                | SafetyMode::Violation
                | SafetyMode::Fault
        )
    }

    /// Whether a new motion request may be accepted in this mode.
    pub fn accepts_goal(&self) -> bool {
        matches!(self, SafetyMode::Normal | SafetyMode::Reduced)
    }
}

/// The dashboard `safetystatus` answer.
///
/// Strictly more detailed than [`SafetyMode`]: it distinguishes which kind of
/// safeguard stop is active. Dashboard-only - the realtime stream has no
/// equivalent field.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SafetyStatus {
    Normal,
    Reduced,
    ProtectiveStop,
    Recovery,
    SafeguardStop,
    SystemEmergencyStop,
    RobotEmergencyStop,
    Violation,
    Fault,
    AutomaticModeSafeguardStop,
    SystemThreePositionEnablingStop,
    Unknown,
}

impl SafetyStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            SafetyStatus::Normal => "NORMAL",
            SafetyStatus::Reduced => "REDUCED",
            SafetyStatus::ProtectiveStop => "PROTECTIVE_STOP",
            SafetyStatus::Recovery => "RECOVERY",
            SafetyStatus::SafeguardStop => "SAFEGUARD_STOP",
            SafetyStatus::SystemEmergencyStop => "SYSTEM_EMERGENCY_STOP",
            SafetyStatus::RobotEmergencyStop => "ROBOT_EMERGENCY_STOP",
            SafetyStatus::Violation => "VIOLATION",
            SafetyStatus::Fault => "FAULT",
            SafetyStatus::AutomaticModeSafeguardStop => "AUTOMATIC_MODE_SAFEGUARD_STOP",
            SafetyStatus::SystemThreePositionEnablingStop => {
                "SYSTEM_THREE_POSITION_ENABLING_STOP"
            }
            SafetyStatus::Unknown => "UNKNOWN",
        }
    }

    pub fn parse(response: &str) -> Self {
        match payload_of(response).to_ascii_uppercase().as_str() {
            "NORMAL" => SafetyStatus::Normal,
            "REDUCED" => SafetyStatus::Reduced,
            "PROTECTIVE_STOP" => SafetyStatus::ProtectiveStop,
            "RECOVERY" => SafetyStatus::Recovery,
            "SAFEGUARD_STOP" => SafetyStatus::SafeguardStop,
            "SYSTEM_EMERGENCY_STOP" => SafetyStatus::SystemEmergencyStop,
            "ROBOT_EMERGENCY_STOP" => SafetyStatus::RobotEmergencyStop,
            "VIOLATION" => SafetyStatus::Violation,
            "FAULT" => SafetyStatus::Fault,
            "AUTOMATIC_MODE_SAFEGUARD_STOP" => SafetyStatus::AutomaticModeSafeguardStop,
            "SYSTEM_THREE_POSITION_ENABLING_STOP" => {
                SafetyStatus::SystemThreePositionEnablingStop
            }
            _ => SafetyStatus::Unknown,
        }
    }
}

/// State of the *loaded pendant program*, from the dashboard `programState` query.
///
/// Note this tracks the program loaded on the pendant, not a script this driver
/// injected over port 30003 - those keep `programState` at `STOPPED` while running.
/// `pause` and `play` do act on an injected script, so `Paused` is still the signal
/// that a pause took effect.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ProgramState {
    Stopped,
    Playing,
    Paused,
    Unknown,
}

impl ProgramState {
    pub fn as_str(&self) -> &'static str {
        match self {
            ProgramState::Stopped => "STOPPED",
            ProgramState::Playing => "PLAYING",
            ProgramState::Paused => "PAUSED",
            ProgramState::Unknown => "UNKNOWN",
        }
    }

    /// Split a `"STOPPED <program name>"` reply into the state and the name.
    pub fn parse(response: &str) -> (Self, Option<String>) {
        let mut parts = response.trim().splitn(2, char::is_whitespace);
        let state = match parts.next().unwrap_or("").to_ascii_uppercase().as_str() {
            "STOPPED" => ProgramState::Stopped,
            "PLAYING" => ProgramState::Playing,
            "PAUSED" => ProgramState::Paused,
            _ => ProgramState::Unknown,
        };
        let program = parts
            .next()
            .map(str::trim)
            .filter(|p| !p.is_empty())
            .map(str::to_string);
        (state, program)
    }
}

/// PolyScope operational mode, from `get operational mode`.
///
/// `None` is the answer when no mode password has been set in Settings, not an
/// error - it means the mode is simply not in use on this robot.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum OperationalMode {
    Manual,
    Automatic,
    None,
    Unknown,
}

impl OperationalMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            OperationalMode::Manual => "MANUAL",
            OperationalMode::Automatic => "AUTOMATIC",
            OperationalMode::None => "NONE",
            OperationalMode::Unknown => "UNKNOWN",
        }
    }

    /// The spelling `set operational mode` expects. `None`/`Unknown` have none -
    /// clearing the mode is `clear operational mode`, a separate command.
    pub fn wire(&self) -> Option<&'static str> {
        match self {
            OperationalMode::Manual => Some("manual"),
            OperationalMode::Automatic => Some("automatic"),
            OperationalMode::None | OperationalMode::Unknown => None,
        }
    }

    pub fn parse(response: &str) -> Self {
        match payload_of(response).to_ascii_uppercase().as_str() {
            "MANUAL" => OperationalMode::Manual,
            "AUTOMATIC" => OperationalMode::Automatic,
            "NONE" => OperationalMode::None,
            _ => OperationalMode::Unknown,
        }
    }
}

/// Which flight report `generate flight report` should produce.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FlightReportType {
    Controller,
    Software,
    System,
}

impl FlightReportType {
    pub fn wire(&self) -> &'static str {
        match self {
            FlightReportType::Controller => "controller",
            FlightReportType::Software => "software",
            FlightReportType::System => "system",
        }
    }

    /// The manual makes `system` the default when no type is given.
    pub fn parse(arg: &str) -> Self {
        match arg.trim().to_ascii_lowercase().as_str() {
            "controller" => FlightReportType::Controller,
            "software" => FlightReportType::Software,
            _ => FlightReportType::System,
        }
    }
}

/// How long the controller gets to answer a query or an instant action.
const QUERY_TIMEOUT: Duration = Duration::from_secs(2);

/// Actions that move the robot, the brakes or the safety system. These are
/// acknowledged quickly but not instantly, and a 2 s ceiling was tight enough to
/// drop the socket on a busy controller.
const ACTION_TIMEOUT: Duration = Duration::from_secs(5);

/// `load` and `load installation` do not answer until the program *and* its
/// installation have finished loading.
const LOAD_TIMEOUT: Duration = Duration::from_secs(30);

/// The manual: "Command can take few minutes to complete."
const FLIGHT_REPORT_TIMEOUT: Duration = Duration::from_secs(300);

/// The manual: "Command can take up to 10 minutes to complete."
const SUPPORT_FILE_TIMEOUT: Duration = Duration::from_secs(600);

/// Everything the driver can ask the UR Dashboard Server to do.
///
/// One variant per command in `docs/DashboardServer_e-Series_2022.pdf`, except
/// `quit`: it closes the connection this driver keeps open for the life of the
/// process, so it is deliberately not exposed.
///
/// Query variants - the ones whose [`DashboardCommand::expect`] is `None` - return
/// the controller's reply as the payload of [`DashboardReply`], typed into
/// [`DashboardValue`] where there is something to type.
#[derive(Clone, PartialEq, Debug)]
pub enum DashboardCommand {
    // Program control
    Stop,
    Pause,
    /// The bare `play` primitive.
    Play,
    /// `play`, sequenced: confirmed against the program state, and falling back to
    /// re-issuing the live goal when `play` does not resume an injected script.
    /// See `driver::dashboard::resume_motion`.
    Resume,
    // Power
    PowerOn,
    PowerOff,
    BrakeRelease,
    // Safety recovery
    UnlockProtectiveStop,
    CloseSafetyPopup,
    ClosePopup,
    RestartSafety,
    // Program / installation
    Load(String),
    LoadInstallation(String),
    // Operational mode
    SetOperationalMode(OperationalMode),
    GetOperationalMode,
    ClearOperationalMode,
    // Queries
    RobotMode,
    SafetyStatus,
    /// Deprecated by UR in favour of `safetystatus`, which distinguishes the kinds
    /// of safeguard stop. Kept because it is what realtime offset 812 reports, so
    /// it is the query that cross-checks the realtime stream.
    SafetyMode,
    ProgramState,
    IsProgramRunning,
    IsProgramSaved,
    IsInRemoteControl,
    GetLoadedProgram,
    GetRobotModel,
    GetSerialNumber,
    PolyscopeVersion,
    /// PolyScope 5.13.0 and later only; older controllers answer with an error.
    Version,
    // Diagnostics
    GenerateFlightReport(FlightReportType),
    GenerateSupportFile(String),
    // Misc
    Popup(String),
    AddToLog(String),
    Shutdown,
}

impl DashboardCommand {
    /// Map the string written to `{robot}_dashboard_command` onto a command.
    ///
    /// `arg` comes from `{robot}_dashboard_command_arg` and is ignored by the
    /// variants that do not take one. `Err` carries a message meant for
    /// `{robot}_dashboard_request_result`, so a bad argument is diagnosable from
    /// Redis rather than reported as an unknown command.
    pub fn parse(name: &str, arg: &str) -> Result<Self, String> {
        let name = name.trim().to_lowercase();
        let arg = arg.trim();

        let require_arg = |what: &str| -> Result<String, String> {
            if arg.is_empty() {
                Err(format!("'{}' needs an argument: {}", name, what))
            } else {
                Ok(arg.to_string())
            }
        };

        let cmd = match name.as_str() {
            "stop" => DashboardCommand::Stop,
            "pause" => DashboardCommand::Pause,
            "play" => DashboardCommand::Play,
            "resume" => DashboardCommand::Resume,
            "power_on" => DashboardCommand::PowerOn,
            "power_off" => DashboardCommand::PowerOff,
            "brake_release" => DashboardCommand::BrakeRelease,
            // `reset_protective_stop` is the name the old enum used; keep it as an
            // alias so anything already written against it keeps working.
            "unlock_protective_stop" | "reset_protective_stop" => {
                DashboardCommand::UnlockProtectiveStop
            }
            "close_safety_popup" => DashboardCommand::CloseSafetyPopup,
            "close_popup" => DashboardCommand::ClosePopup,
            "restart_safety" => DashboardCommand::RestartSafety,
            "load" => DashboardCommand::Load(require_arg("a program filename")?),
            "load_installation" => {
                DashboardCommand::LoadInstallation(require_arg("an installation filename")?)
            }
            "set_operational_mode" => {
                let mode = OperationalMode::parse(arg);
                if mode.wire().is_none() {
                    return Err(format!(
                        "'set_operational_mode' needs 'manual' or 'automatic', got '{}'",
                        arg
                    ));
                }
                DashboardCommand::SetOperationalMode(mode)
            }
            "get_operational_mode" => DashboardCommand::GetOperationalMode,
            "clear_operational_mode" => DashboardCommand::ClearOperationalMode,
            "robot_mode" => DashboardCommand::RobotMode,
            "safety_status" => DashboardCommand::SafetyStatus,
            "safety_mode" => DashboardCommand::SafetyMode,
            "program_state" => DashboardCommand::ProgramState,
            "is_program_running" => DashboardCommand::IsProgramRunning,
            "is_program_saved" => DashboardCommand::IsProgramSaved,
            "is_in_remote_control" => DashboardCommand::IsInRemoteControl,
            "get_loaded_program" => DashboardCommand::GetLoadedProgram,
            "get_robot_model" => DashboardCommand::GetRobotModel,
            "get_serial_number" => DashboardCommand::GetSerialNumber,
            "polyscope_version" => DashboardCommand::PolyscopeVersion,
            "version" => DashboardCommand::Version,
            // No argument means the type the manual defaults to, `system`.
            "generate_flight_report" => {
                DashboardCommand::GenerateFlightReport(FlightReportType::parse(arg))
            }
            "generate_support_file" => DashboardCommand::GenerateSupportFile(require_arg(
                "a directory inside the programs directory",
            )?),
            "popup" => DashboardCommand::Popup(arg.to_string()),
            "add_to_log" => DashboardCommand::AddToLog(arg.to_string()),
            "shutdown" => DashboardCommand::Shutdown,
            _ => return Err(format!("unknown dashboard command '{}'", name)),
        };
        Ok(cmd)
    }

    /// The line to write on the socket, without the trailing newline.
    pub fn wire(&self) -> String {
        match self {
            DashboardCommand::Stop => "stop".to_string(),
            DashboardCommand::Pause => "pause".to_string(),
            // `Resume` is `play` plus a confirmation step; the wire command is the
            // same one.
            DashboardCommand::Play | DashboardCommand::Resume => "play".to_string(),
            DashboardCommand::PowerOn => "power on".to_string(),
            DashboardCommand::PowerOff => "power off".to_string(),
            DashboardCommand::BrakeRelease => "brake release".to_string(),
            DashboardCommand::UnlockProtectiveStop => "unlock protective stop".to_string(),
            DashboardCommand::CloseSafetyPopup => "close safety popup".to_string(),
            DashboardCommand::ClosePopup => "close popup".to_string(),
            DashboardCommand::RestartSafety => "restart safety".to_string(),
            DashboardCommand::Load(p) => format!("load {}", p),
            DashboardCommand::LoadInstallation(p) => format!("load installation {}", p),
            // `parse` refuses any mode without a wire spelling, so this cannot be
            // reached with `None`/`Unknown`.
            DashboardCommand::SetOperationalMode(m) => {
                format!("set operational mode {}", m.wire().unwrap_or("manual"))
            }
            DashboardCommand::GetOperationalMode => "get operational mode".to_string(),
            DashboardCommand::ClearOperationalMode => "clear operational mode".to_string(),
            DashboardCommand::RobotMode => "robotmode".to_string(),
            DashboardCommand::SafetyStatus => "safetystatus".to_string(),
            DashboardCommand::SafetyMode => "safetymode".to_string(),
            DashboardCommand::ProgramState => "programState".to_string(),
            DashboardCommand::IsProgramRunning => "running".to_string(),
            DashboardCommand::IsProgramSaved => "isProgramSaved".to_string(),
            DashboardCommand::IsInRemoteControl => "is in remote control".to_string(),
            DashboardCommand::GetLoadedProgram => "get loaded program".to_string(),
            DashboardCommand::GetRobotModel => "get robot model".to_string(),
            DashboardCommand::GetSerialNumber => "get serial number".to_string(),
            DashboardCommand::PolyscopeVersion => "PolyscopeVersion".to_string(),
            DashboardCommand::Version => "version".to_string(),
            DashboardCommand::GenerateFlightReport(t) => {
                format!("generate flight report {}", t.wire())
            }
            DashboardCommand::GenerateSupportFile(d) => format!("generate support file {}", d),
            DashboardCommand::Popup(t) => format!("popup {}", t),
            DashboardCommand::AddToLog(t) => format!("addToLog {}", t),
            DashboardCommand::Shutdown => "shutdown".to_string(),
        }
    }

    /// Substring of the reply that means the command took effect.
    ///
    /// `None` marks a query: there is no fixed reply to match, so any reply at all
    /// is a success and the reply itself is the answer.
    pub fn expect(&self) -> Option<&'static str> {
        match self {
            DashboardCommand::Stop => Some("Stopped"),
            DashboardCommand::Pause => Some("Pausing program"),
            DashboardCommand::Play | DashboardCommand::Resume => Some("Starting program"),
            DashboardCommand::PowerOn => Some("Powering on"),
            DashboardCommand::PowerOff => Some("Powering off"),
            DashboardCommand::BrakeRelease => Some("Brake releasing"),
            DashboardCommand::UnlockProtectiveStop => Some("Protective stop releasing"),
            DashboardCommand::CloseSafetyPopup => Some("closing safety popup"),
            DashboardCommand::ClosePopup => Some("closing popup"),
            DashboardCommand::RestartSafety => Some("Restarting safety"),
            DashboardCommand::Load(_) => Some("Loading program"),
            DashboardCommand::LoadInstallation(_) => Some("Loading installation"),
            DashboardCommand::SetOperationalMode(_) => Some("Setting operational mode"),
            DashboardCommand::ClearOperationalMode => {
                Some("no longer controlled by Dashboard Server")
            }
            DashboardCommand::GenerateSupportFile(_) => Some("Completed successfully"),
            DashboardCommand::Popup(_) => Some("showing popup"),
            DashboardCommand::AddToLog(_) => Some("Added log message"),
            DashboardCommand::Shutdown => Some("Shutting down"),
            // Queries - the reply is the payload, there is nothing to match. So is
            // `generate flight report`, which answers with the report id.
            DashboardCommand::GetOperationalMode
            | DashboardCommand::RobotMode
            | DashboardCommand::SafetyStatus
            | DashboardCommand::SafetyMode
            | DashboardCommand::ProgramState
            | DashboardCommand::IsProgramRunning
            | DashboardCommand::IsProgramSaved
            | DashboardCommand::IsInRemoteControl
            | DashboardCommand::GetLoadedProgram
            | DashboardCommand::GetRobotModel
            | DashboardCommand::GetSerialNumber
            | DashboardCommand::PolyscopeVersion
            | DashboardCommand::Version
            | DashboardCommand::GenerateFlightReport(_) => None,
        }
    }

    /// How long the controller gets to answer this command.
    ///
    /// A single flat timeout for every command is wrong in both directions: 2 s
    /// drops the socket on a `load`, and 10 minutes would hide a dead controller
    /// behind a `robotmode` query. The values come from the "Description" column of
    /// the manual, which says which commands do not return until their work is done.
    pub fn reply_timeout(&self) -> Duration {
        match self {
            DashboardCommand::Load(_) | DashboardCommand::LoadInstallation(_) => LOAD_TIMEOUT,
            DashboardCommand::GenerateFlightReport(_) => FLIGHT_REPORT_TIMEOUT,
            DashboardCommand::GenerateSupportFile(_) => SUPPORT_FILE_TIMEOUT,
            DashboardCommand::Stop
            | DashboardCommand::Pause
            | DashboardCommand::Play
            | DashboardCommand::Resume
            | DashboardCommand::PowerOn
            | DashboardCommand::PowerOff
            | DashboardCommand::BrakeRelease
            | DashboardCommand::UnlockProtectiveStop
            | DashboardCommand::RestartSafety
            | DashboardCommand::SetOperationalMode(_)
            | DashboardCommand::ClearOperationalMode
            | DashboardCommand::Shutdown => ACTION_TIMEOUT,
            _ => QUERY_TIMEOUT,
        }
    }

    /// Whether the manual marks this command "Only Remote Control".
    ///
    /// Used to *annotate* a refusal, not to pre-reject one: the controller's own
    /// "Failed to execute: <command>" is the truthful answer, and this only explains
    /// why. Pre-rejecting would also block these commands on a robot where the
    /// Remote Control feature is simply not enabled, which is a different problem
    /// with a different fix.
    pub fn requires_remote_control(&self) -> bool {
        matches!(
            self,
            DashboardCommand::Stop
                | DashboardCommand::Pause
                | DashboardCommand::Play
                | DashboardCommand::Resume
                | DashboardCommand::PowerOn
                | DashboardCommand::PowerOff
                | DashboardCommand::BrakeRelease
                | DashboardCommand::UnlockProtectiveStop
                | DashboardCommand::CloseSafetyPopup
                | DashboardCommand::RestartSafety
                | DashboardCommand::Load(_)
                | DashboardCommand::LoadInstallation(_)
        )
    }

    /// Type the controller's reply, for the commands whose reply carries a value.
    ///
    /// `None` means there is nothing to type - an action's acknowledgement, or a
    /// free-form answer like a flight report id.
    pub fn parse_reply(&self, response: &str) -> Option<DashboardValue> {
        match self {
            DashboardCommand::RobotMode => {
                Some(DashboardValue::RobotMode(RobotMode::parse(response)))
            }
            DashboardCommand::SafetyMode => {
                Some(DashboardValue::SafetyMode(SafetyMode::parse(response)))
            }
            DashboardCommand::SafetyStatus => {
                Some(DashboardValue::SafetyStatus(SafetyStatus::parse(response)))
            }
            DashboardCommand::ProgramState => {
                let (state, program) = ProgramState::parse(response);
                Some(DashboardValue::ProgramState { state, program })
            }
            DashboardCommand::GetOperationalMode => Some(DashboardValue::OperationalMode(
                OperationalMode::parse(response),
            )),
            DashboardCommand::IsProgramRunning
            | DashboardCommand::IsProgramSaved
            | DashboardCommand::IsInRemoteControl => {
                Some(DashboardValue::Flag(parse_flag(response)))
            }
            // "Loaded program: <path>" or "No program loaded" - strip the label,
            // keep whatever the controller said.
            DashboardCommand::GetLoadedProgram => {
                Some(DashboardValue::Text(payload_of(response).to_string()))
            }
            DashboardCommand::GetRobotModel
            | DashboardCommand::GetSerialNumber
            | DashboardCommand::PolyscopeVersion
            | DashboardCommand::Version => {
                Some(DashboardValue::Text(response.trim().to_string()))
            }
            _ => None,
        }
    }
}

/// The typed answer to a query command.
#[derive(Clone, PartialEq, Debug)]
pub enum DashboardValue {
    RobotMode(RobotMode),
    SafetyMode(SafetyMode),
    SafetyStatus(SafetyStatus),
    ProgramState {
        state: ProgramState,
        program: Option<String>,
    },
    OperationalMode(OperationalMode),
    /// `running`, `is in remote control`, `isProgramSaved`.
    Flag(bool),
    /// Serial number, robot model, version, loaded program.
    Text(String),
}

impl DashboardValue {
    /// The boolean behind a [`DashboardValue::Flag`], if this is one.
    pub fn as_flag(&self) -> Option<bool> {
        match self {
            DashboardValue::Flag(v) => Some(*v),
            _ => None,
        }
    }

    /// The string behind a [`DashboardValue::Text`], if this is one.
    pub fn as_text(&self) -> Option<&str> {
        match self {
            DashboardValue::Text(v) => Some(v),
            _ => None,
        }
    }
}

/// The outcome of one dashboard command.
///
/// `response` is the controller's reply line, verbatim and trimmed, and is what
/// reaches `{robot}_dashboard_request_result`. For a query that line *is* the
/// answer; for an action it is kept so a failure can say what the controller
/// actually said instead of just `false`. `value` is the same reply typed, for
/// callers inside the driver that need to branch on it.
#[derive(Clone, Debug)]
pub struct DashboardReply {
    pub success: bool,
    pub response: String,
    pub value: Option<DashboardValue>,
}

impl DashboardReply {
    pub fn ok(response: impl Into<String>) -> Self {
        DashboardReply { success: true, response: response.into(), value: None }
    }

    pub fn fail(response: impl Into<String>) -> Self {
        DashboardReply { success: false, response: response.into(), value: None }
    }

    /// Attach the typed reply.
    pub fn with_value(mut self, value: Option<DashboardValue>) -> Self {
        self.value = value;
        self
    }
}

/// What the dashboard connection polls on every keepalive tick.
///
/// None of this is available from the realtime stream: offset 1052 does not carry
/// the stopped/playing/paused enum, and Remote Control and the operational mode are
/// not in the packet at all.
#[derive(Clone, PartialEq, Debug)]
pub struct DashboardStatus {
    pub remote_control: bool,
    pub program_state: ProgramState,
    pub program_name: Option<String>,
    pub program_running: bool,
    pub operational_mode: OperationalMode,
}

impl Default for DashboardStatus {
    fn default() -> Self {
        DashboardStatus {
            remote_control: false,
            program_state: ProgramState::Unknown,
            program_name: None,
            program_running: false,
            operational_mode: OperationalMode::Unknown,
        }
    }
}

/// Fixed facts about the controller, read once per connection rather than on every
/// keepalive tick.
#[derive(Clone, PartialEq, Debug, Default)]
pub struct DashboardIdentity {
    pub robot_model: String,
    pub serial_number: String,
    pub polyscope_version: String,
}
