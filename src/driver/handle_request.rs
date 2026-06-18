use futures::FutureExt;
use futures::future::{self, Either};
use micro_sp::{ActionRequestState, StateManager, ToSPValue};
use redis::aio::MultiplexedConnection;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::net::TcpStream;
use tokio::sync::{mpsc, oneshot};
use tokio::time::timeout;

use crate::interfaces::command_server::{publish_script_feedback, publish_script_result};
use crate::{DashboardCommand, DriverState, ScriptRequest, generate_ur_script};

pub async fn handle_request(
    ur_address: String,
    robot_name: String,
    host_address: String,
    driver_state: Arc<Mutex<DriverState>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
    req: ScriptRequest,
    mut cancel_receiver: mpsc::Receiver<()>,
    mut con: MultiplexedConnection,
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
        let mut ds = driver_state.lock().unwrap();
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
                let publish_feedback_fut = async {
                    loop {
                        match feedback_receiver.recv().await {
                            Some(msg) => publish_script_feedback(&req_uuid, &msg),
                            None => return Result::<(), Box<dyn std::error::Error>>::Ok(()),
                        }
                    }
                };

                let nominal =
                    Box::pin(async { futures::join!(goal_receiver, publish_feedback_fut) }).fuse();

                match future::select(nominal, cancel_receiver.recv().boxed()).await {
                    Either::Left(((res, _), _cancel_stream)) => {
                        if let Ok(ok) = res {
                            println!("goal completed. result: {}", ok);
                            (ok, ResultType::SUCCEDED)
                        } else {
                            println!("future appears canceled, abort.");
                            (false, ResultType::ABORTED)
                        }
                    }
                    Either::Right((_cancel_req, nominal)) => {
                        println!("got cancel request: {}", req.uuid);
                        let (sender, ds_cancel_receiver) = oneshot::channel();
                        dashboard_commands
                            .try_send((DashboardCommand::Stop, sender))
                            .expect("could not send dashboard stop");

                        match future::select(ds_cancel_receiver, nominal).await {
                            Either::Left((res, _nominal)) => {
                                if let Ok(ok) = res {
                                    (ok, ResultType::CANCELED)
                                } else {
                                    println!("cancel dashboard future appears canceled");
                                    (false, ResultType::ABORTED)
                                }
                            }
                            Either::Right(((res, _), _ds_cancel_receiver)) => {
                                if let Ok(ok) = res {
                                    println!("goal completed before cancel. result: {}", ok);
                                    (ok, ResultType::SUCCEDED)
                                } else {
                                    println!("finished executing but future is canceled.");
                                    (false, ResultType::ABORTED)
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
    match result_type {
        ResultType::ABORTED => {
            log::error!(target: &log_target, "Goal aborted, result is: '{}'.", result_success);
            request_state = ActionRequestState::Failed.to_string();
        }
        ResultType::CANCELED => {
            log::warn!(target: &log_target, "Goal cancelled, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
        }
        ResultType::SUCCEDED => {
            log::info!(target: &log_target, "Goal succeeded, result is: '{}'.", result_success);
            request_state = ActionRequestState::Succeeded.to_string();
        }
    }

    StateManager::set_sp_value(&mut con, &key("request_state"), &request_state.to_spvalue()).await;

    {
        let mut ds = driver_state.lock().unwrap();
        ds.goal_id = None;
        ds.goal_sender = None;
        ds.handshake_sender = None;
        ds.feedback_sender = None;
        ds.cancel_sender = None;
    }

    Ok(())
}
