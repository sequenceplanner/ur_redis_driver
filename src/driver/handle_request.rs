use micro_sp::{ActionRequestState, SPConnection, StateManager, ToSPValue};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::timeout;

use crate::interfaces::command_server::{
    fail_request, publish_script_feedback, publish_script_result,
};
use crate::{
    DashboardCommand, DashboardReply, DriverState, RobotCommand, ScriptRequest,
    generate_core_script_from_template, generate_ur_script, lock_driver_state,
};

/// How long the script gets to dial back and complete the handshake.
const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(5000);

/// How long to wait for a killed script's verdict before re-issuing.
///
/// The verdict has to be consumed on the *old* goal channel: `socket_server` sends
/// `false` when the script socket closes, and if the new channels are already
/// installed that `false` fails the resumed goal instead of the dead one.
const REISSUE_DRAIN_TIMEOUT: Duration = Duration::from_secs(2);

enum ResultType {
    ABORTED,
    CANCELED,
    SUCCEDED,
}

/// Why the wait on a running script ended.
enum Outcome {
    /// The script reported a verdict, or its socket closed.
    Goal(Result<bool, oneshot::error::RecvError>),
    Cancel,
    Resume,
}

/// Run one motion goal to a terminal state.
///
/// The body is a loop rather than a straight line because a goal can outlive the
/// script that is executing it: when a dashboard `play` does not resume an injected
/// script, the dashboard task signals `resume_receiver` and this re-renders the
/// remainder of the motion and sends it as a second script under the *same* goal id
/// and the same Redis request. From the caller's side the request just keeps
/// executing and then succeeds.
pub async fn handle_request(
    ur_address: String,
    robot_name: String,
    host_address: String,
    driver_state: Arc<Mutex<DriverState>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<DashboardReply>)>,
    req: ScriptRequest,
    mut cancel_receiver: mpsc::Receiver<()>,
    mut resume_receiver: mpsc::Receiver<()>,
    templates: Arc<tera::Tera>,
    mut con: SPConnection,
) -> Result<(), Box<dyn std::error::Error>> {
    let key = |suffix: &str| format!("{robot_name}_{suffix}");
    let log_target = "ur_redis_driver";

    let ScriptRequest { uuid, mut script, mut command } = req;

    let (result_success, result_type, result_message) = 'attempts: loop {
        let (goal_sender, mut goal_receiver) = oneshot::channel::<bool>();
        let (handshake_sender, handshake_receiver) = oneshot::channel::<bool>();
        let (feedback_sender, mut feedback_receiver) = mpsc::channel::<String>(5);

        log::debug!(target: log_target, "Making a new connection to the robot for goal {}.", uuid);
        let mut write_stream = match TcpStream::connect(&ur_address).await {
            Ok(write_stream) => write_stream,
            Err(e) => {
                break (
                    false,
                    ResultType::ABORTED,
                    format!("could not connect to the realtime port for writing: {}", e),
                );
            }
        };

        let script_to_write = generate_ur_script(&script, &host_address);

        {
            let mut ds = lock_driver_state(&driver_state);
            ds.goal_id = Some(uuid.clone());
            ds.goal_sender = Some(goal_sender);
            ds.handshake_sender = Some(handshake_sender);
            ds.feedback_sender = Some(feedback_sender);
            // `goal_id` is the guard again from here on.
            ds.reissuing = false;
        }

        log::debug!(target: log_target, "Writing the script for goal {}:\n{}", uuid, script_to_write);
        // Not `?`: this is inside the attempt loop, and returning early would skip
        // the teardown below and leave `goal_id` set, which rejects every later
        // request with "a goal is already running".
        if let Err(e) = write_stream.write_all(script_to_write.as_bytes()).await {
            break (
                false,
                ResultType::ABORTED,
                format!("could not write the script to the robot: {}", e),
            );
        }
        if let Err(e) = write_stream.flush().await {
            break (
                false,
                ResultType::ABORTED,
                format!("could not write the script to the robot: {}", e),
            );
        }

        match timeout(HANDSHAKE_TIMEOUT, handshake_receiver).await {
            Ok(Ok(true)) => {}
            Ok(_) | Err(_) => {
                break (
                    false,
                    ResultType::ABORTED,
                    "the script never completed the handshake".to_string(),
                );
            }
        }

        // Wait on the script, publishing its feedback as it arrives. `feedback_open`
        // keeps the closed channel out of the select: `recv` on a dropped sender
        // returns `None` immediately and forever, which would spin this loop.
        let mut feedback_open = true;
        let outcome = loop {
            tokio::select! {
                res = &mut goal_receiver => break Outcome::Goal(res),
                msg = feedback_receiver.recv(), if feedback_open => {
                    match msg {
                        Some(msg) => {
                            publish_script_feedback(&mut con, &robot_name, &uuid, &msg).await
                        }
                        None => feedback_open = false,
                    }
                }
                _ = cancel_receiver.recv() => break Outcome::Cancel,
                _ = resume_receiver.recv() => break Outcome::Resume,
            }
        };

        match outcome {
            // The goal channel carries the script's own verdict: `true` for the "ok"
            // line, `false` for "error" or for the socket closing without a result
            // (a dashboard `stop`, an abort on the pendant). Mapping every resolved
            // value to SUCCEDED - as this once did - reported a failed or externally
            // stopped move as `succeeded` in Redis, which is worse than reporting
            // nothing at all.
            Outcome::Goal(Ok(true)) => {
                break (true, ResultType::SUCCEDED, "succeeded".to_string());
            }
            Outcome::Goal(Ok(false)) => {
                break (
                    false,
                    ResultType::ABORTED,
                    "aborted before reaching the goal".to_string(),
                );
            }
            Outcome::Goal(Err(_)) => {
                break (
                    false,
                    ResultType::ABORTED,
                    "aborted before reaching the goal".to_string(),
                );
            }

            Outcome::Cancel => {
                log::info!(target: log_target, "Got a cancel request for goal {}.", uuid);
                let stop_reply = stop_script(&dashboard_commands, &uuid, log_target);

                tokio::select! {
                    res = stop_reply => match res {
                        Some(reply) => break (reply.success, ResultType::CANCELED, "cancelled".to_string()),
                        None => break (
                            false,
                            ResultType::ABORTED,
                            "aborted before reaching the goal".to_string(),
                        ),
                    },
                    // The move can finish while the stop is still in flight.
                    res = &mut goal_receiver => match res {
                        Ok(true) => break (true, ResultType::SUCCEDED, "succeeded".to_string()),
                        _ => break (
                            false,
                            ResultType::ABORTED,
                            "aborted before reaching the goal".to_string(),
                        ),
                    },
                }
            }

            Outcome::Resume => {
                log::info!(
                    target: log_target,
                    "Re-issuing goal {} because play did not resume the paused script.",
                    uuid
                );

                // Decide what is left to run *before* touching the robot, so an
                // unresumable command is reported without first killing its script
                // for nothing.
                let completed = lock_driver_state(&driver_state).waypoints_completed;
                let remaining = match remaining_command(&command, completed) {
                    Ok(remaining) => remaining,
                    Err(reason) => break (false, ResultType::ABORTED, reason),
                };

                let next_script = match generate_core_script_from_template(
                    &robot_name,
                    remaining.clone(),
                    &templates,
                ) {
                    Ok(next_script) => next_script,
                    Err(e) => {
                        break (
                            false,
                            ResultType::ABORTED,
                            format!("could not re-render the remaining motion: {}", e),
                        );
                    }
                };

                // Claim the goal across the gap. `socket_server` clears `goal_id`
                // the moment the killed script's socket closes, and without this a
                // request arriving in between would be admitted on top of the
                // resume.
                lock_driver_state(&driver_state).reissuing = true;

                // Kill the paused script. Sending a new one to port 30003 would
                // replace it anyway, but doing it explicitly means the close is
                // observed here rather than racing the next attempt's handshake.
                let _ = stop_script(&dashboard_commands, &uuid, log_target).await;

                // Consume the dead script's verdict on the old channel. A timeout is
                // not fatal: the worst case is that the verdict arrives after the new
                // channels are installed, and the next attempt then reports an abort
                // that the operator can retry.
                if timeout(REISSUE_DRAIN_TIMEOUT, &mut goal_receiver).await.is_err() {
                    log::warn!(
                        target: log_target,
                        "The paused script for goal {} did not report after the stop.",
                        uuid
                    );
                }

                command = remaining;
                script = next_script;
                lock_driver_state(&driver_state).waypoints_completed = 0;
                {
                    let mut ds = lock_driver_state(&driver_state);
                    ds.active_command = Some(command.clone());
                }
                continue 'attempts;
            }
        }
    };

    let request_state;
    match result_type {
        ResultType::ABORTED => {
            log::error!(target: log_target, "Goal aborted, result is: '{}'.", result_success);
            request_state = ActionRequestState::Failed.to_string();
        }
        ResultType::CANCELED => {
            log::warn!(target: log_target, "Goal cancelled, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
        }
        ResultType::SUCCEDED => {
            log::info!(target: log_target, "Goal succeeded, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
        }
    }

    if request_state == ActionRequestState::Succeeded.to_string() {
        publish_script_result(&mut con, &robot_name, &result_message, true).await;
        StateManager::set_sp_value(&mut con, &key("request_state"), &request_state.to_spvalue())
            .await;
        // A successful run clears the consecutive-failure streak. The total counter
        // is cumulative and is never reset.
        StateManager::set_sp_value(&mut con, &key("subsequent_fail_counter"), &0.to_spvalue())
            .await;
    } else {
        // Route execution failures through the same helper as the admission-control
        // rejections, so both kinds of failure land in `request_result` the same way
        // and both move the counters. A supervisor watching
        // `subsequent_fail_counter` wants a move that aborted on the robot to count
        // just as much as one this driver refused to start.
        fail_request(&mut con, &robot_name, &result_message, log_target).await;
    }

    {
        let mut ds = lock_driver_state(&driver_state);
        ds.goal_id = None;
        ds.goal_sender = None;
        ds.handshake_sender = None;
        ds.feedback_sender = None;
        ds.cancel_sender = None;
        ds.resume_sender = None;
        ds.active_command = None;
        ds.waypoints_completed = 0;
        ds.reissuing = false;
        // The hold cannot outlive the goal it was holding. Leaving it set would park
        // `command_server` rejecting every later request with "motion is paused".
        ds.motion_paused = false;
    }

    Ok(())
}

/// Ask the dashboard to kill the running script, and wait for its reply.
///
/// `None` means the dashboard task never answered - it is mid-reconnect, or its
/// channel is full. The script may then still be running, which is why the caller
/// treats that as an abort rather than a clean cancel.
async fn stop_script(
    dashboard_commands: &mpsc::Sender<(DashboardCommand, oneshot::Sender<DashboardReply>)>,
    uuid: &str,
    log_target: &str,
) -> Option<DashboardReply> {
    let (sender, receiver) = oneshot::channel();

    // `try_send` used to be followed by `.expect`, which turned a momentarily full
    // 10-slot channel - or a dashboard task that is mid-reconnect - into a panic
    // that poisoned the shared `DriverState` mutex for every other task.
    if let Err(e) = dashboard_commands.try_send((DashboardCommand::Stop, sender)) {
        log::error!(
            target: log_target,
            "Could not send the dashboard stop for goal {}: {}. The script may still be running.",
            uuid, e
        );
        return None;
    }

    receiver.await.ok()
}

/// The part of a command still left to run after `completed` waypoints.
///
/// `Err` carries a reason meant for `request_result`: refusing to resume is better
/// than resuming into the wrong motion.
fn remaining_command(command: &RobotCommand, completed: usize) -> Result<RobotCommand, String> {
    // A relative move applies its offset to the TCP pose *at the time the script
    // runs*. Re-issuing it from the paused pose would travel the full offset a
    // second time, so the robot would end up past its goal.
    if command.command_type.ends_with("_relative") {
        return Err(format!(
            "cannot resume '{}': re-issuing a relative-pose move would apply the offset a second time from the paused pose. Cancel and submit a new request instead",
            command.command_type
        ));
    }

    // A single move to an absolute target in the base frame is correct to re-send
    // verbatim: the script recomputes IK from wherever the robot now is.
    if command.waypoints.is_empty() {
        return Ok(command.clone());
    }

    if completed >= command.waypoints.len() {
        return Err(format!(
            "cannot resume '{}': all {} waypoints were already reached",
            command.command_type,
            command.waypoints.len()
        ));
    }

    let mut remaining = command.clone();
    remaining.waypoints.drain(..completed);
    Ok(remaining)
}
