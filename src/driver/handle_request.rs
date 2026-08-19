use futures::FutureExt;
use futures::future::{self, Either};
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
    DashboardCommand, DashboardReply, DriverState, ScriptRequest, generate_ur_script,
    lock_driver_state,
};

pub async fn handle_request(
    ur_address: String,
    robot_name: String,
    host_address: String,
    driver_state: Arc<Mutex<DriverState>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<DashboardReply>)>,
    req: ScriptRequest,
    mut cancel_receiver: mpsc::Receiver<()>,
    mut con: SPConnection,
) -> Result<(), Box<dyn std::error::Error>> {
    let key = |suffix: &str| format!("{robot_name}_{suffix}");
    let (goal_sender, goal_receiver) = oneshot::channel::<bool>();
    let log_target = "ur_redis_driver";

    println!("making a new connection to the driver.");
    let conn = TcpStream::connect(&ur_address).await;
    let mut write_stream = match conn {
        Ok(write_stream) => write_stream,
        Err(_) => {
            println!("could not connect to realtime port for writing");
            return Err("oh no".into());
        }
    };

    let script_to_write = generate_ur_script(&req.script, &host_address);
    let (handshake_sender, handshake_receiver) = oneshot::channel::<bool>();
    let (feedback_sender, mut feedback_receiver) = tokio::sync::mpsc::channel(5);

    {
        let mut ds = lock_driver_state(&driver_state);
        ds.goal_id = Some(req.uuid.clone());
        ds.goal_sender = Some(goal_sender);
        ds.handshake_sender = Some(handshake_sender);
        ds.feedback_sender = Some(feedback_sender);
    }

    println!("writing data to driver\n{}", script_to_write);
    write_stream.write_all(script_to_write.as_bytes()).await?;
    write_stream.flush().await?;

    enum ResultType {
        ABORTED,
        CANCELED,
        SUCCEDED,
    }

    let (result_success, result_type) =
        match timeout(Duration::from_millis(5000), handshake_receiver).await {
            Ok(Ok(true)) => {
                println!("HANDSHAKE OK, PERFORM NOMINAL");

                let req_uuid = req.uuid.clone();
                // Its own handle, because `con` below is borrowed mutably for the
                // terminal state write. `SPConnection` is a cheap-to-clone
                // multiplexed handle, so this is not a second socket.
                let mut feedback_con = con.clone();
                let feedback_robot_name = robot_name.clone();
                let publish_feedback_fut = async move {
                    loop {
                        match feedback_receiver.recv().await {
                            Some(msg) => {
                                publish_script_feedback(
                                    &mut feedback_con,
                                    &feedback_robot_name,
                                    &req_uuid,
                                    &msg,
                                )
                                .await
                            }
                            None => return Result::<(), Box<dyn std::error::Error>>::Ok(()),
                        }
                    }
                };

                let nominal =
                    Box::pin(async { futures::join!(goal_receiver, publish_feedback_fut) }).fuse();

                match future::select(nominal, cancel_receiver.recv().boxed()).await {
                    Either::Left(((res, _), _cancel_stream)) => {
                        // The goal channel carries the script's own verdict: `true`
                        // for the "ok" line, `false` for "error" or for the socket
                        // closing without a result (a dashboard `stop`, an abort on
                        // the pendant). Mapping every resolved value to SUCCEDED -
                        // as this did - reported a failed or externally stopped move
                        // as `succeeded` in Redis, which is worse than reporting
                        // nothing at all.
                        match res {
                            Ok(true) => {
                                println!("goal completed successfully.");
                                (true, ResultType::SUCCEDED)
                            }
                            Ok(false) => {
                                println!("goal ended without success, abort.");
                                (false, ResultType::ABORTED)
                            }
                            Err(_) => {
                                println!("future appears canceled, abort.");
                                (false, ResultType::ABORTED)
                            }
                        }
                    }
                    Either::Right((_cancel_req, nominal)) => {
                        println!("got cancel request: {}", req.uuid);
                        let (sender, ds_cancel_receiver) = oneshot::channel();
                        // `try_send` used to be followed by `.expect`, which turned a
                        // momentarily full 10-slot channel - or a dashboard task
                        // that is mid-reconnect - into a panic that poisoned the
                        // shared `DriverState` mutex for every other task.
                        if let Err(e) = dashboard_commands.try_send((DashboardCommand::Stop, sender))
                        {
                            log::error!(
                                target: &log_target,
                                "Could not send the dashboard stop for goal {}: {}. The script may still be running.",
                                req.uuid, e
                            );
                        }

                        match future::select(ds_cancel_receiver, nominal).await {
                            Either::Left((res, _nominal)) => {
                                match res {
                                    Ok(reply) => (reply.success, ResultType::CANCELED),
                                    Err(_) => {
                                        println!("cancel dashboard future appears canceled");
                                        (false, ResultType::ABORTED)
                                    }
                                }
                            }
                            Either::Right(((res, _), _ds_cancel_receiver)) => {
                                // Same verdict mapping as the nominal path above.
                                match res {
                                    Ok(true) => {
                                        println!("goal completed before cancel took effect.");
                                        (true, ResultType::SUCCEDED)
                                    }
                                    Ok(false) => {
                                        println!("goal ended without success before cancel.");
                                        (false, ResultType::ABORTED)
                                    }
                                    Err(_) => {
                                        println!("finished executing but future is canceled.");
                                        (false, ResultType::ABORTED)
                                    }
                                }
                            }
                        }
                    }
                }
            }
            Ok(_) | Err(_) => {
                println!("HANDSHAKE FAILURE OR TIMEOUT, ABORTING");
                (false, ResultType::ABORTED)
            }
        };

    let request_state;
    let result_message;
    match result_type {
        ResultType::ABORTED => {
            log::error!(target: &log_target, "Goal aborted, result is: '{}'.", result_success);
            request_state = ActionRequestState::Failed.to_string();
            result_message = "aborted before reaching the goal".to_string();
        }
        ResultType::CANCELED => {
            log::warn!(target: &log_target, "Goal cancelled, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
            result_message = "cancelled".to_string();
        }
        ResultType::SUCCEDED => {
            log::info!(target: &log_target, "Goal succeeded, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
            result_message = "succeeded".to_string();
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
        fail_request(&mut con, &robot_name, &result_message, &log_target).await;
    }

    {
        let mut ds = lock_driver_state(&driver_state);
        ds.goal_id = None;
        ds.goal_sender = None;
        ds.handshake_sender = None;
        ds.feedback_sender = None;
        ds.cancel_sender = None;
    }

    Ok(())
}
