use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{MissedTickBehavior, interval, timeout};

use crate::driver::realtime_reader::connect_loop;
use crate::{DashboardCommand, DashboardReply, DriverState, lock_driver_state, safety_mode_name};

/// How long the controller gets to answer one dashboard command.
const DASHBOARD_REPLY_TIMEOUT: Duration = Duration::from_secs(2);

/// How often the idle socket is exercised to notice a dead connection and to
/// refresh the Remote Control flag.
const DASHBOARD_KEEPALIVE: Duration = Duration::from_secs(2);

/// Pause between a dropped dashboard socket and the next connect attempt.
const DASHBOARD_RECONNECT_DELAY: Duration = Duration::from_secs(1);

/// PolyScope refuses to release a protective stop while the robot is still
/// settling. This is the documented minimum wait after the stop is raised.
const PROTECTIVE_STOP_SETTLE: Duration = Duration::from_millis(500);

/// How long to wait for the safety mode to actually return to NORMAL after the
/// controller accepts `unlock protective stop`.
const PROTECTIVE_STOP_RELEASE_TIMEOUT: Duration = Duration::from_secs(3);

type DashboardRequest = (DashboardCommand, oneshot::Sender<DashboardReply>);

/// Long-lived client for the UR Dashboard Server on port 29999.
///
/// Serves commands arriving on `recv` and keeps `DriverState.dashboard_connected` /
/// `remote_control` current.
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

        lock_driver_state(&driver_state).dashboard_connected = true;

        let closed = serve(&mut stream, &mut recv, &driver_state, &log_target).await;

        lock_driver_state(&driver_state).dashboard_connected = false;
        lock_driver_state(&driver_state).remote_control = false;

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
    // it seeds `remote_control` right after connecting instead of two seconds later.

    loop {
        tokio::select! {
            request = recv.recv() => {
                let Some((cmd, reply)) = request else {
                    return true;
                };

                log::debug!(target: log_target, "Dashboard command: {:?}", cmd);

                let result = if cmd == DashboardCommand::UnlockProtectiveStop {
                    reset_protective_stop(stream, driver_state, log_target).await
                } else {
                    send_command(stream, &cmd).await
                };

                match result {
                    Ok(dashboard_reply) => {
                        if !dashboard_reply.success {
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
                // - and it cannot report Remote Control at all. Those two come from
                // here, which doubles as the liveness check on an idle socket.
                match poll_status(stream).await {
                    Ok((remote_control, program_state, program_running)) => {
                        let mut ds = lock_driver_state(driver_state);
                        ds.remote_control = remote_control;
                        ds.program_state = program_state;
                        ds.program_running = program_running;
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
async fn poll_status(stream: &mut BufReader<TcpStream>) -> Result<(bool, String, bool), String> {
    let remote = send_command(stream, &DashboardCommand::IsInRemoteControl).await?;
    let state = send_command(stream, &DashboardCommand::ProgramState).await?;
    let running = send_command(stream, &DashboardCommand::IsProgramRunning).await?;

    // `programState` answers "STOPPED <program name>"; only the state is wanted.
    let program_state = state
        .response
        .split_whitespace()
        .next()
        .unwrap_or("UNKNOWN")
        .to_string();

    // `running` answers "Program running: true".
    let program_running = running
        .response
        .rsplit(':')
        .next()
        .map(|v| v.trim().eq_ignore_ascii_case("true"))
        .unwrap_or(false);

    Ok((
        remote.response.trim().eq_ignore_ascii_case("true"),
        program_state,
        program_running,
    ))
}

/// Read and validate the banner the dashboard server sends on connect.
async fn read_greeting(stream: &mut BufReader<TcpStream>) -> Result<String, String> {
    let mut line = String::new();
    match timeout(DASHBOARD_REPLY_TIMEOUT, stream.read_line(&mut line)).await {
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
    match timeout(DASHBOARD_REPLY_TIMEOUT, stream.read_line(&mut response)).await {
        Ok(Ok(0)) => return Err("connection closed by controller".to_string()),
        Ok(Ok(_)) => {}
        Ok(Err(e)) => return Err(e.to_string()),
        Err(_) => return Err(format!("timed out waiting for reply to '{}'", cmd.wire())),
    }

    let response = response.trim().to_string();

    match cmd.expect() {
        // A query has no fixed reply to match against - the reply is the answer.
        None => Ok(DashboardReply::ok(response)),
        Some(expected) => {
            let success = response.contains(expected);
            Ok(DashboardReply { success, response })
        }
    }
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
    let deadline = tokio::time::Instant::now() + PROTECTIVE_STOP_RELEASE_TIMEOUT;
    loop {
        let mode = lock_driver_state(driver_state).safety_mode;
        if mode == 1 || mode == 2 {
            log::info!(target: log_target, "Protective stop released, safety mode is {}.", safety_mode_name(mode));
            return Ok(DashboardReply::ok(safety_mode_name(mode)));
        }
        if tokio::time::Instant::now() >= deadline {
            return Ok(DashboardReply::fail(format!(
                "unlock acknowledged but safety mode is still {}",
                safety_mode_name(mode)
            )));
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
}
