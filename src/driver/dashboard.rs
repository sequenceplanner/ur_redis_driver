use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{Instant, MissedTickBehavior, interval, timeout};

use crate::driver::realtime_reader::connect_loop;
use crate::{
    DashboardCommand, DashboardIdentity, DashboardReply, DashboardStatus, DashboardValue,
    DriverState, OperationalMode, ProgramState, lock_driver_state, safety_mode_name,
};

/// How long the controller gets to send its greeting.
///
/// Every *command* takes its own deadline from `DashboardCommand::reply_timeout()`
/// instead - `load` is allowed 30 s and `generate support file` ten minutes, and a
/// single flat ceiling used to drop the socket on both.
const DASHBOARD_GREETING_TIMEOUT: Duration = Duration::from_secs(2);

/// How often the idle socket is exercised to notice a dead connection and to
/// refresh the status the realtime stream cannot supply.
const DASHBOARD_KEEPALIVE: Duration = Duration::from_secs(2);

/// Pause between a dropped dashboard socket and the next connect attempt.
const DASHBOARD_RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// PolyScope refuses to release a protective stop while the robot is still
/// settling. This is the documented minimum wait after the stop is raised.
const PROTECTIVE_STOP_SETTLE: Duration = Duration::from_millis(500);

/// How long to wait for the safety mode to actually return to NORMAL after the
/// controller accepts `unlock protective stop`.
const PROTECTIVE_STOP_RELEASE_TIMEOUT: Duration = Duration::from_secs(3);

/// How long to wait for the program state to read `PAUSED` after `pause`.
const PAUSE_CONFIRM_TIMEOUT: Duration = Duration::from_secs(1);

/// How long to wait for a program to be running again after `play`.
///
/// Short on purpose: when `play` cannot resume an injected script it does not fail
/// slowly, it simply never starts one, and every millisecond spent here is a
/// millisecond the robot stays still before the re-issue fallback takes over.
const RESUME_CONFIRM_TIMEOUT: Duration = Duration::from_millis(500);

/// Gap between polls while confirming a pause or a resume.
const CONFIRM_POLL_INTERVAL: Duration = Duration::from_millis(100);

type DashboardRequest = (DashboardCommand, oneshot::Sender<DashboardReply>);

/// Long-lived client for the UR Dashboard Server on port 29999.
///
/// Serves commands arriving on `recv` and keeps the dashboard-sourced half of
/// `DriverState` current: `dashboard_connected`, `remote_control`, `program_state`,
/// `program_running`, `operational_mode` and `identity`.
///
/// This function does not return `Err` for anything a robot can do to it - a
/// refused connection, a dropped socket, a controller reboot. It is joined with
/// `tokio::try_join!` in `main`, so an `Err` here tears down the realtime reader,
/// the command server and the state publisher along with it. A dashboard that is
/// merely unreachable must not do that; it reconnects instead, and every command
/// that arrives meanwhile is answered with a failure so no caller is left hanging.
pub async fn dashboard(
    mut recv: mpsc::Receiver<DashboardRequest>,
    ur_dashboard_address: String,
    driver_state: Arc<Mutex<DriverState>>,
    log_target: String,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let stream = connect_loop(&ur_dashboard_address).await;
        let mut stream = BufReader::new(stream);

        match read_greeting(&mut stream).await {
            Ok(greeting) => {
                log::info!(target: &log_target, "Dashboard server connected: {}", greeting);
            }
            Err(e) => {
                log::warn!(
                    target: &log_target,
                    "Not a UR Dashboard Server at {} ({}). Retrying.",
                    ur_dashboard_address, e
                );
                tokio::time::sleep(DASHBOARD_RECONNECT_DELAY).await;
                continue;
            }
        }

        // Model, serial and PolyScope version do not change while the controller is
        // up, so they are read once here rather than on every keepalive tick. An
        // `Err` means the socket is already unusable, which is worth reconnecting
        // for before serving any command on it.
        match read_identity(&mut stream).await {
            Ok(identity) => {
                log::info!(
                    target: &log_target,
                    "Controller is a {} (serial {}), {}.",
                    identity.robot_model, identity.serial_number, identity.polyscope_version
                );
                lock_driver_state(&driver_state).identity = Some(identity);
            }
            Err(e) => {
                log::warn!(target: &log_target, "Could not read the controller identity: {}. Reconnecting.", e);
                tokio::time::sleep(DASHBOARD_RECONNECT_DELAY).await;
                continue;
            }
        }

        lock_driver_state(&driver_state).dashboard_connected = true;

        let closed = serve(&mut stream, &mut recv, &driver_state, &log_target).await;

        {
            let mut ds = lock_driver_state(&driver_state);
            ds.dashboard_connected = false;
            ds.remote_control = false;
            ds.identity = None;
        }

        if closed {
            log::info!(target: &log_target, "Dashboard command channel closed, stopping.");
            return Ok(());
        }

        log::warn!(target: &log_target, "Dashboard connection lost, reconnecting.");
        tokio::time::sleep(DASHBOARD_RECONNECT_DELAY).await;
    }
}

/// Serve commands until the socket fails or the command channel closes.
///
/// Returns `true` if the channel closed (a real shutdown) and `false` if the socket
/// needs reconnecting.
async fn serve(
    stream: &mut BufReader<TcpStream>,
    recv: &mut mpsc::Receiver<DashboardRequest>,
    driver_state: &Arc<Mutex<DriverState>>,
    log_target: &str,
) -> bool {
    let mut keepalive = interval(DASHBOARD_KEEPALIVE);
    keepalive.set_missed_tick_behavior(MissedTickBehavior::Delay);
    // `interval` fires immediately on the first tick, which is what we want here:
    // it seeds the status right after connecting instead of two seconds later.

    loop {
        tokio::select! {
            request = recv.recv() => {
                let Some((cmd, reply)) = request else {
                    return true;
                };

                log::debug!(target: log_target, "Dashboard command: {:?}", cmd);

                // An exhaustive `match` on the composites, so a command that needs
                // more than one round trip cannot quietly fall into the generic
                // path the way an `if cmd == ...` chain allowed.
                let result = match &cmd {
                    DashboardCommand::UnlockProtectiveStop => {
                        reset_protective_stop(stream, driver_state, log_target).await
                    }
                    DashboardCommand::Pause => {
                        pause_motion(stream, driver_state, log_target).await
                    }
                    DashboardCommand::Resume => {
                        resume_motion(stream, driver_state, log_target).await
                    }
                    _ => send_command(stream, &cmd).await,
                };

                match result {
                    Ok(mut dashboard_reply) => {
                        if !dashboard_reply.success {
                            // Local control is the single most common reason a
                            // dashboard action is refused, and "Failed to execute:
                            // play" does not say so. Annotate rather than
                            // pre-reject: the controller's own verdict stays the
                            // one being reported.
                            if cmd.requires_remote_control()
                                && !lock_driver_state(driver_state).remote_control
                            {
                                dashboard_reply
                                    .response
                                    .push_str(" (robot is in Local control)");
                            }
                            log::warn!(
                                target: log_target,
                                "Dashboard command {:?} did not take effect: {}",
                                cmd, dashboard_reply.response
                            );
                        }
                        let _ = reply.send(dashboard_reply);
                    }
                    Err(e) => {
                        // Answer before reconnecting. The caller is blocked on this
                        // oneshot and dropping it without a value would strand it
                        // until its own timeout fires.
                        let _ = reply.send(DashboardReply::fail(format!(
                            "dashboard socket error: {}", e
                        )));
                        return false;
                    }
                }
            }

            _ = keepalive.tick() => {
                // The realtime stream carries safety and robot mode, but not a
                // usable program state - RT packet 1052 does not hold the
                // stopped/playing/paused enum (see `DriverState::program_state_raw`)
                // - and it cannot report Remote Control or the operational mode at
                // all. Those come from here, which doubles as the liveness check on
                // an idle socket.
                match poll_status(stream).await {
                    Ok(status) => {
                        let mut ds = lock_driver_state(driver_state);
                        ds.remote_control = status.remote_control;
                        ds.program_state = status.program_state.as_str().to_string();
                        ds.program_running = status.program_running;
                        ds.operational_mode = status.operational_mode.as_str().to_string();
                    }
                    Err(e) => {
                        log::warn!(target: log_target, "Dashboard keepalive failed: {}", e);
                        return false;
                    }
                }
            }
        }
    }
}

/// Refresh the status the realtime stream cannot supply.
async fn poll_status(stream: &mut BufReader<TcpStream>) -> Result<DashboardStatus, String> {
    let remote = send_command(stream, &DashboardCommand::IsInRemoteControl).await?;
    let state = send_command(stream, &DashboardCommand::ProgramState).await?;
    let running = send_command(stream, &DashboardCommand::IsProgramRunning).await?;
    let operational = send_command(stream, &DashboardCommand::GetOperationalMode).await?;

    let (program_state, program_name) = match state.value {
        Some(DashboardValue::ProgramState { state, program }) => (state, program),
        _ => (ProgramState::Unknown, None),
    };

    Ok(DashboardStatus {
        remote_control: remote.value.and_then(|v| v.as_flag()).unwrap_or(false),
        program_state,
        program_name,
        program_running: running.value.and_then(|v| v.as_flag()).unwrap_or(false),
        operational_mode: match operational.value {
            Some(DashboardValue::OperationalMode(mode)) => mode,
            _ => OperationalMode::Unknown,
        },
    })
}

/// Read the facts about the controller that do not change while it is up.
async fn read_identity(
    stream: &mut BufReader<TcpStream>,
) -> Result<DashboardIdentity, String> {
    // `PolyscopeVersion` rather than `version`: the latter is PolyScope 5.13.0 and
    // later only, and this has to work on every controller the driver supports.
    let model = send_command(stream, &DashboardCommand::GetRobotModel).await?;
    let serial = send_command(stream, &DashboardCommand::GetSerialNumber).await?;
    let version = send_command(stream, &DashboardCommand::PolyscopeVersion).await?;

    Ok(DashboardIdentity {
        robot_model: model.response,
        serial_number: serial.response,
        polyscope_version: version.response,
    })
}

/// Read and validate the banner the dashboard server sends on connect.
async fn read_greeting(stream: &mut BufReader<TcpStream>) -> Result<String, String> {
    let mut line = String::new();
    match timeout(DASHBOARD_GREETING_TIMEOUT, stream.read_line(&mut line)).await {
        Ok(Ok(0)) => Err("connection closed before greeting".to_string()),
        Ok(Ok(_)) => {
            let line = line.trim().to_string();
            if line.contains("Universal Robots Dashboard Server") {
                Ok(line)
            } else {
                Err(format!("unexpected greeting: {}", line))
            }
        }
        Ok(Err(e)) => Err(e.to_string()),
        Err(_) => Err("timed out waiting for greeting".to_string()),
    }
}

/// Write one command and match its reply.
///
/// `Err` means the socket is unusable and the caller should reconnect. `Ok` with
/// `success: false` means the controller answered but refused the command - a
/// normal outcome (wrong mode, not in remote control) that must not drop the
/// connection.
async fn send_command(
    stream: &mut BufReader<TcpStream>,
    cmd: &DashboardCommand,
) -> Result<DashboardReply, String> {
    let line = format!("{}\n", cmd.wire());
    stream
        .get_mut()
        .write_all(line.as_bytes())
        .await
        .map_err(|e| e.to_string())?;
    stream.get_mut().flush().await.map_err(|e| e.to_string())?;

    let mut response = String::new();
    // Per command, not one ceiling for all of them: `load` does not answer until
    // the program and its installation have loaded, and `generate support file` can
    // take ten minutes.
    match timeout(cmd.reply_timeout(), stream.read_line(&mut response)).await {
        Ok(Ok(0)) => return Err("connection closed by controller".to_string()),
        Ok(Ok(_)) => {}
        Ok(Err(e)) => return Err(e.to_string()),
        Err(_) => {
            return Err(format!(
                "timed out after {:?} waiting for reply to '{}'",
                cmd.reply_timeout(),
                cmd.wire()
            ));
        }
    }

    let response = response.trim().to_string();
    let value = cmd.parse_reply(&response);

    match cmd.expect() {
        // A query has no fixed reply to match against - the reply is the answer.
        None => Ok(DashboardReply::ok(response).with_value(value)),
        Some(expected) => {
            let success = response.contains(expected);
            Ok(DashboardReply { success, response, value })
        }
    }
}

/// Read the program state, discarding anything that is not a program state.
async fn program_state(
    stream: &mut BufReader<TcpStream>,
) -> Result<ProgramState, String> {
    let reply = send_command(stream, &DashboardCommand::ProgramState).await?;
    match reply.value {
        Some(DashboardValue::ProgramState { state, .. }) => Ok(state),
        _ => Ok(ProgramState::Unknown),
    }
}

/// Hold the robot where it is.
///
/// `pause` suspends the running program, which includes a script this driver
/// injected over port 30003 - the controller counts one as a running program even
/// though `programState` reports on the *pendant* program. The confirmation
/// therefore has to come off this socket: realtime offset 1052 does not carry the
/// stopped/playing/paused enum.
async fn pause_motion(
    stream: &mut BufReader<TcpStream>,
    driver_state: &Arc<Mutex<DriverState>>,
    log_target: &str,
) -> Result<DashboardReply, String> {
    let pause = send_command(stream, &DashboardCommand::Pause).await?;
    if !pause.success {
        return Ok(pause);
    }

    let deadline = Instant::now() + PAUSE_CONFIRM_TIMEOUT;
    let mut last;
    loop {
        last = program_state(stream).await?;
        if last == ProgramState::Paused {
            lock_driver_state(driver_state).motion_paused = true;
            log::info!(target: log_target, "Motion paused.");
            return Ok(DashboardReply::ok(ProgramState::Paused.as_str()));
        }
        if Instant::now() >= deadline {
            break;
        }
        tokio::time::sleep(CONFIRM_POLL_INTERVAL).await;
    }

    // The controller accepted `pause` but never reported `PAUSED`. Latch anyway:
    // the acknowledgement is the controller's own word that it is decelerating, and
    // leaving the latch off would let `command_server` start a new move into a
    // robot that is coming to a stop.
    lock_driver_state(driver_state).motion_paused = true;
    log::warn!(
        target: log_target,
        "Pause acknowledged but the program state is {}; holding motion anyway.",
        last.as_str()
    );
    Ok(DashboardReply::ok(format!(
        "pause acknowledged but the program state is {}",
        last.as_str()
    )))
}

/// Release a pause and get the robot moving again.
///
/// Two paths. `play` resumes the paused program, which is all that is needed when
/// the controller treats the injected script as resumable. When it does not - `play`
/// starts the *loaded pendant program*, not an interface script - nothing comes back
/// running, and the live goal is told to re-issue itself instead. Which path ran is
/// reported in the reply, because it is the one thing about pause/resume that
/// depends on the controller rather than on this driver.
async fn resume_motion(
    stream: &mut BufReader<TcpStream>,
    driver_state: &Arc<Mutex<DriverState>>,
    log_target: &str,
) -> Result<DashboardReply, String> {
    let (paused, resume_sender) = {
        let ds = lock_driver_state(driver_state);
        (ds.motion_paused, ds.resume_sender.clone())
    };

    if !paused {
        return Ok(DashboardReply::fail("motion is not paused"));
    }

    let play = send_command(stream, &DashboardCommand::Play).await?;

    // A refused `play` still goes on to the fallback rather than returning here:
    // refusing to resume an injected script is exactly the case the fallback exists
    // for.
    if play.success {
        let deadline = Instant::now() + RESUME_CONFIRM_TIMEOUT;
        loop {
            let running = send_command(stream, &DashboardCommand::IsProgramRunning).await?;
            let running = running.value.and_then(|v| v.as_flag()).unwrap_or(false);
            if running && program_state(stream).await? == ProgramState::Playing {
                lock_driver_state(driver_state).motion_paused = false;
                log::info!(target: log_target, "Motion resumed by play.");
                return Ok(DashboardReply::ok("resumed"));
            }
            if Instant::now() >= deadline {
                break;
            }
            tokio::time::sleep(CONFIRM_POLL_INTERVAL).await;
        }
    }

    // Clear the latch either way. Leaving it set on a failed resume would park the
    // driver rejecting every motion request with "motion is paused" and no way back
    // except a restart.
    lock_driver_state(driver_state).motion_paused = false;

    let Some(resume_sender) = resume_sender else {
        log::warn!(
            target: log_target,
            "Play did not resume a program and there is no live goal to re-issue."
        );
        return Ok(DashboardReply::fail(format!(
            "play did not resume a program and no goal is live ({})",
            play.response
        )));
    };

    if let Err(e) = resume_sender.try_send(()) {
        return Ok(DashboardReply::fail(format!(
            "play did not resume a program and the goal could not be signalled: {}",
            e
        )));
    }

    log::info!(
        target: log_target,
        "Play did not resume the injected script, re-issuing the live goal."
    );
    Ok(DashboardReply::ok(
        "play did not resume, re-issuing the goal",
    ))
}

/// Clear a protective stop.
///
/// A bare `unlock protective stop` is not enough on an e-Series controller: a
/// safety popup is usually covering the screen, and PolyScope refuses to release
/// while the robot is still settling. So this closes the popup, waits out the
/// settle, unlocks, and then confirms against the safety mode coming off the
/// realtime stream rather than trusting the acknowledgement - the controller
/// answers "Protective stop releasing" before it has actually released, and can
/// still fail if whatever caused the stop is still present.
async fn reset_protective_stop(
    stream: &mut BufReader<TcpStream>,
    driver_state: &Arc<Mutex<DriverState>>,
    log_target: &str,
) -> Result<DashboardReply, String> {
    let safety_mode = lock_driver_state(driver_state).safety_mode;
    if safety_mode != 3 {
        return Ok(DashboardReply::fail(format!(
            "not in a protective stop, safety mode is {}",
            safety_mode_name(safety_mode)
        )));
    }

    // Best effort: there may be no popup to close, and "no popup" is not a failure.
    if let Err(e) = send_command(stream, &DashboardCommand::CloseSafetyPopup).await {
        return Err(e);
    }

    tokio::time::sleep(PROTECTIVE_STOP_SETTLE).await;

    let unlock = send_command(stream, &DashboardCommand::UnlockProtectiveStop).await?;
    if !unlock.success {
        return Ok(DashboardReply::fail(format!(
            "controller refused to unlock: {}",
            unlock.response
        )));
    }

    // Poll the realtime stream, not the socket. It already carries safety mode at
    // 125 Hz, so this costs nothing and reports the state the rest of the driver
    // actually acts on.
    let deadline = Instant::now() + PROTECTIVE_STOP_RELEASE_TIMEOUT;
    loop {
        let mode = lock_driver_state(driver_state).safety_mode;
        if mode == 1 || mode == 2 {
            log::info!(target: log_target, "Protective stop released, safety mode is {}.", safety_mode_name(mode));
            return Ok(DashboardReply::ok(safety_mode_name(mode)));
        }
        if Instant::now() >= deadline {
            return Ok(DashboardReply::fail(format!(
                "unlock acknowledged but safety mode is still {}",
                safety_mode_name(mode)
            )));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}
