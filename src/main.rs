use futures::future::{self, Either};
use futures::stream::StreamExt;
use futures::{FutureExt, SinkExt};
use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;
use tokio::time::timeout;
use tokio_util::codec::{Framed, LinesCodec};
use tokio_util::task::LocalPoolHandle;
use ur_redis_driver::RobotCommand;



#[derive(Clone, PartialEq, Debug)]
pub enum DashboardCommand {
    Stop,
    ResetProtectiveStop,
}

pub struct ScriptRequest {
    pub uuid: String,
    pub script: String,
}

/// TODO: Implement this to yield new dashboard commands from your custom interface.
async fn wait_for_dashboard_command() -> Option<DashboardCommand> {
    // Example: Read from a custom channel or API
    // tokio::time::sleep(Duration::from_secs(1)).await;
    // None
    std::future::pending().await // blocks forever by default
}

/// TODO: Implement this to yield new script requests from your custom interface.
async fn wait_for_script_request() -> Option<ScriptRequest> {
    // Example: Read from a custom channel or API
    std::future::pending().await
}

/// TODO: Implement this to publish/send joint states to your custom system.
async fn publish_joint_states(joints: &[f64], speeds: &[f64]) {
    println!("Joints: {:?}", joints);
}

/// TODO: Implement this to publish/send robot measured states to your custom system.
async fn publish_measured_state(
    robot_state: i32,
    program_state: i32,
    forces: &[f64],
    inputs: u32,
    outputs: u32,
) {
    // println!("Robot State: {}, Forces: {:?}", robot_state, forces);
}

/// TODO: Implement this to handle script execution feedback (e.g., standard output/errors).
fn publish_script_feedback(uuid: &str, feedback: &str) {
    println!("Script [{}] Feedback: {}", uuid, feedback);
}

/// TODO: Implement this to handle script completion results.
fn publish_script_result(uuid: &str, success: bool) {
    println!("Script [{}] Result: {}", uuid, success);
}

// ============================================================================
// DRIVER IMPLEMENTATION
// ============================================================================

struct DriverState {
    running: bool,
    connected: bool,
    goal_id: Option<String>,
    goal_sender: Option<oneshot::Sender<bool>>,
    handshake_sender: Option<oneshot::Sender<bool>>,
    feedback_sender: Option<mpsc::Sender<String>>,
    robot_state: i32,
    program_state: i32,
    joint_values: Vec<f64>,
    joint_speeds: Vec<f64>,
    digital_inputs: u32,
    digital_outputs: u32,
    forces: Vec<f64>,
}

impl DriverState {
    fn new() -> Self {
        DriverState {
            running: true,
            connected: false,
            goal_id: None,
            goal_sender: None,
            handshake_sender: None,
            feedback_sender: None,
            robot_state: 0,
            program_state: 0,
            joint_values: vec![],
            joint_speeds: vec![],
            digital_inputs: 0,
            digital_outputs: 0,
            forces: vec![],
        }
    }
}

async fn handle_dashboard_commands_loop(
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        if let Some(cmd) = wait_for_dashboard_command().await {
            println!("Got dashboard command request: {:?}", cmd);
            let (sender, future) = oneshot::channel();
            if dashboard_commands.try_send((cmd, sender)).is_ok() {
                let _ok = future.await.unwrap_or(false);
                // Handle reply to your interface if needed
            }
        } else {
            break;
        }
    }
    Ok(())
}



async fn handle_request(
    ur_address: String,
    host_address: String,
    driver_state: Arc<Mutex<DriverState>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
    req: ScriptRequest,
    mut cancel_receiver: mpsc::Receiver<()>,
) -> Result<(), Box<dyn std::error::Error>> {
    let (goal_sender, goal_receiver) = oneshot::channel::<bool>();

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

    publish_script_result(&req.uuid, result_success);

    {
        let mut ds = driver_state.lock().unwrap();
        ds.goal_id = None;
        ds.goal_sender = None;
        ds.handshake_sender = None;
        ds.feedback_sender = None;
    }

    Ok(())
}

async fn script_request_server(
    ur_address: String,
    local_addr: watch::Receiver<Option<SocketAddr>>,
    driver_state: Arc<Mutex<DriverState>>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
) -> Result<(), Box<dyn std::error::Error>> {
    let local_pool = LocalPoolHandle::new(1);

    loop {
        if let Some(req) = wait_for_script_request().await {
            let local_addr = local_addr.borrow().clone();

            if local_addr.is_none() || !driver_state.lock().unwrap().connected {
                println!(
                    "Not connected to robot yet, rejecting request: {}",
                    req.uuid
                );
                publish_script_result(&req.uuid, false);
                continue;
            }
            let local_addr_str = local_addr.unwrap().ip().to_string();

            if driver_state.lock().unwrap().robot_state != 1 {
                println!("Robot not in normal mode, rejecting request: {}", req.uuid);
                publish_script_result(&req.uuid, false);
                continue;
            }

            if driver_state.lock().unwrap().goal_id.is_some() {
                println!(
                    "Already have an active goal, rejecting request: {}",
                    req.uuid
                );
                publish_script_result(&req.uuid, false);
                continue;
            }

            println!("Accepting goal request with goal id: {}", req.uuid);

            // Note: If you want cancellation, you must hook `cancel_sender` up to your custom interface.
            let (_cancel_sender, cancel_receiver) = mpsc::channel(1);

            let task_ur_address = ur_address.clone();
            let task_dashboard_commands = dashboard_commands.clone();
            let task_driver_state = driver_state.clone();

            local_pool.spawn_pinned(move || async {
                let result = handle_request(
                    task_ur_address,
                    local_addr_str,
                    task_driver_state,
                    task_dashboard_commands,
                    req,
                    cancel_receiver,
                )
                .await;

                if let Err(e) = result {
                    println!("Error while handing goal: {}", e);
                }
            });
        } else {
            break;
        }
    }
    Ok(())
}

fn read_f64(slice: &[u8]) -> f64 {
    let mut bytes = [0u8; 8];
    bytes.copy_from_slice(slice);
    f64::from_be_bytes(bytes)
}

async fn connect_loop(address: &str) -> TcpStream {
    loop {
        let ret = TcpStream::connect(address).await;
        match ret {
            Ok(s) => {
                let local_address = s.local_addr().expect("could net get local address");
                let peer_address = s.peer_addr().expect("could net get local address");
                println!(
                    "connected to: {} with host ip {}",
                    peer_address, local_address
                );
                return s;
            }
            Err(e) => {
                println!("could not connect to realtime at {}: {}", address, e);
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

async fn socket_server(
    driver_state: Arc<Mutex<DriverState>>,
    mut local_addr: watch::Receiver<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut addr = None;
    while addr.is_none() {
        local_addr.changed().await?;
        addr = local_addr.borrow().clone();
    }

    let mut addr = addr.unwrap();
    addr.set_port(UR_DRIVER_SOCKET_PORT);

    println!("Starting socket server at {}", addr);

    let listener = TcpListener::bind(&addr).await?;
    loop {
        let (stream, addr) = listener.accept().await?;
        println!("New connection: {}", addr);

        let (goal_id, handshake_sender, feedback_sender) = {
            let mut ds = driver_state.lock().unwrap();
            if ds.handshake_sender.is_none() || ds.goal_id.is_none() || ds.feedback_sender.is_none()
            {
                println!("SHOULD NOT HAPPEN, DROPPING STREAM");
                continue;
            }
            (
                ds.goal_id.clone().unwrap(),
                ds.handshake_sender.take().unwrap(),
                ds.feedback_sender.clone().unwrap(),
            )
        };

        let mut lines = Framed::new(stream, LinesCodec::new());
        lines.send(&goal_id).await?;

        let line = lines.next().await;
        match line {
            Some(Ok(s)) if s == goal_id => {
                println!("got GO with correct GOAL ID, start UR script.");
                let _ = handshake_sender.send(true);
            }
            _ => {
                println!("got GO with incorrect GOAL ID, SHOULD NOT HAPPEN");
                let _ = handshake_sender.send(false);
            }
        }

        let _ = feedback_sender
            .send("Handshake complete, script should be running.".to_string())
            .await;

        loop {
            match lines.next().await {
                Some(Ok(s)) if s == "ok" => {
                    println!("got OK, we are done.");
                    let mut ds = driver_state.lock().unwrap();
                    if let Some(goal_sender) = ds.goal_sender.take() {
                        let _ = goal_sender.send(true);
                    }
                }
                Some(Ok(s)) if s == "error" => {
                    println!("got ERROR, we are done.");
                    let mut ds = driver_state.lock().unwrap();
                    if let Some(goal_sender) = ds.goal_sender.take() {
                        let _ = goal_sender.send(false);
                    }
                }
                Some(Ok(s)) => {
                    println!("got {}, sending as feedback", s);
                    let _ = feedback_sender.send(s).await;
                }
                _ => {
                    println!("Socket connection closed, dropping feedback sender.");
                    let mut ds = driver_state.lock().unwrap();
                    ds.feedback_sender = None;
                    break;
                }
            };
        }
    }
}

async fn realtime_reader(
    driver_state: Arc<Mutex<DriverState>>,
    ur_address: String,
    override_host_address: Option<String>,
    local_addr_sender: watch::Sender<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut size_bytes = [0u8; 4];
    let mut stream = connect_loop(&ur_address).await;

    let local_addr = if let Some(s) = &override_host_address {
        SocketAddr::from_str(&format!("{}:0", s))?
    } else {
        stream.local_addr()?
    };
    local_addr_sender.send(Some(local_addr))?;
    driver_state.lock().unwrap().connected = true;

    loop {
        let ret = timeout(
            Duration::from_millis(1000),
            stream.read_exact(&mut size_bytes),
        )
        .await;
        if ret.is_err() {
            driver_state.lock().unwrap().connected = false;
            println!("timeout on read, reconnecting... ");
            stream = connect_loop(&ur_address).await;
            driver_state.lock().unwrap().connected = true;
            continue;
        } else if let Ok(Err(e)) = ret {
            println!("unexpected read error: {}", e);
            return Err("oh no".into());
        }

        let msg_size = u32::from_be_bytes(size_bytes) as usize;
        let mut buf: Vec<u8> = Vec::new();
        buf.resize(msg_size - 4, 0);
        stream.read_exact(&mut buf).await?;

        if msg_size == 1220 {
            let mut joints = vec![];
            let mut speeds = vec![];
            for i in 0..6 {
                joints.push(read_f64(&buf[248 + i * 8..248 + i * 8 + 8]));
                speeds.push(read_f64(&buf[296 + i * 8..296 + i * 8 + 8]));
            }

            let mut forces = vec![];
            for i in 0..6 {
                forces.push(read_f64(&buf[536 + i * 8..536 + i * 8 + 8]));
            }

            let digital_inputs = read_f64(&buf[680..688]) as u32;
            let robot_state = read_f64(&buf[808..816]) as i32;
            let digital_outputs = read_f64(&buf[1040..1048]) as u32;
            let program_state = read_f64(&buf[1048..1056]) as i32;

            {
                let mut ds = driver_state.lock().unwrap();
                ds.joint_values = joints;
                ds.joint_speeds = speeds;
                ds.forces = forces;
                ds.robot_state = robot_state;
                ds.program_state = program_state;
                ds.digital_inputs = digital_inputs;
                ds.digital_outputs = digital_outputs;
            }

            if robot_state != 1 {
                let mut ds = driver_state.lock().unwrap();
                if let Some(goal_sender) = ds.goal_sender.take() {
                    println!("aborting due to protective stop");
                    let _ = goal_sender.send(false);
                }
                ds.feedback_sender = None;
                ds.goal_id = None;
            }
        } else {
            println!("got unknown frame length: {}", msg_size);
        }
    }
}

async fn state_publisher(
    driver_state: Arc<Mutex<DriverState>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;

        let (joints, speeds, state, prog_state, forces, inputs, outputs) = {
            let ds = driver_state.lock().unwrap();
            (
                ds.joint_values.clone(),
                ds.joint_speeds.clone(),
                ds.robot_state,
                ds.program_state,
                ds.forces.clone(),
                ds.digital_inputs,
                ds.digital_outputs,
            )
        };

        // Call the boilerplate hook functions
        publish_joint_states(&joints, &speeds).await;
        publish_measured_state(state, prog_state, &forces, inputs, outputs).await;
    }
}

async fn dashboard(
    mut recv: tokio::sync::mpsc::Receiver<(DashboardCommand, oneshot::Sender<bool>)>,
    ur_address: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let stream = connect_loop(&ur_address).await;
    let mut stream = BufReader::new(stream);

    let mut line = String::new();
    stream.read_line(&mut line).await?;
    if !line.contains("Connected: Universal Robots Dashboard Server") {
        return Err("oh no".into());
    }

    stream.write_all(b"get robot model\n").await?;
    stream.flush().await?;
    let mut robot_model = String::new();
    stream.read_line(&mut robot_model).await?;
    println!("robot model: {}", robot_model);

    loop {
        if let Some((cmd, channel)) = recv.recv().await {
            println!("dashboard writer got command {:?}", cmd);

            let (command, expected_response) = match cmd {
                DashboardCommand::Stop => ("stop\n", "Stopped"),
                DashboardCommand::ResetProtectiveStop => {
                    ("unlock protective stop\n", "Protective stop releasing")
                }
            };

            stream.write_all(command.as_bytes()).await?;
            stream.flush().await?;

            let mut response = String::new();
            stream.read_line(&mut response).await?;

            let success = response.contains(expected_response);
            if !success {
                println!(
                    "failed to execute command via dashboard: {}, expected: {}",
                    response, expected_response
                );
            }
            let _ = channel.send(success);
        } else {
            break;
        }
    }
    Ok(())
}

async fn run(
    ur_ip: &str,
    override_host_ip: Option<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    let ur_dashboard_address = format!("{}:29999", ur_ip);
    let ur_address = format!("{}:30003", ur_ip);

    let (tx_dashboard, rx_dashboard) = mpsc::channel(10);
    let shared_state = Arc::new(Mutex::new(DriverState::new()));
    let (local_addr_sender, local_addr_receiver) = watch::channel(None);

    let dashboard_task = handle_dashboard_commands_loop(tx_dashboard.clone());
    let action_task = script_request_server(
        ur_address.clone(),
        local_addr_receiver.clone(),
        shared_state.clone(),
        tx_dashboard.clone(),
    );

    let realtime_task = realtime_reader(
        shared_state.clone(),
        ur_address,
        override_host_ip,
        local_addr_sender,
    );

    let state_publisher_task = state_publisher(shared_state.clone());
    let socket_server_task = socket_server(shared_state.clone(), local_addr_receiver);
    let dashboard_connection = dashboard(rx_dashboard, ur_dashboard_address);

    let ret = tokio::try_join!(
        action_task,
        realtime_task,
        socket_server_task,
        dashboard_connection,
        state_publisher_task,
        dashboard_task,
    );

    if let Err(e) = ret {
        shared_state.lock().unwrap().running = false;
        return Err(e.into());
    }
    Ok(())
}

#[tokio::main]
async fn main() {
    let robot_ip = "0.0.0.0"; // Define or parameterize this
    loop {
        if let Err(e) = run(robot_ip, None).await {
            println!("fatal error: {}", e);
            tokio::time::sleep(Duration::from_secs(2)).await;
        }
    }
}

// GEMINI interpretation for implementing functions:
// use tokio::time::interval;

// pub async fn wait_for_script_request(
//     robot_id: &str,
//     connection_manager: &Arc<ConnectionManager>,
// ) -> Option<ScriptRequest> {
//     let log_target = "ur_driver_script_handler".to_string();
//     let mut interval = interval(Duration::from_millis(50));

//     // Connect to Redis
//     let mut con = connection_manager.get_connection().await;

//     // Define the specific keys we need to watch for script execution
//     let keys: Vec<String> = vec![
//         format!("{robot_id}_script_request_trigger"),
//         format!("{robot_id}_script_request_state"),
//         format!("{robot_id}_script_content"),
//         format!("{robot_id}_script_uuid"),
//     ];

//     loop {
//         interval.tick().await;

//         if connection_manager.check_redis_health("ur_driver").await.is_err() {
//             continue;
//         }

//         let Some(state) = StateManager::get_state_for_keys(&mut con, &keys, &log_target).await else {
//             continue;
//         };

//         let trigger = state.get_bool_or_default_to_false(
//             &format!("{robot_id}_script_request_trigger"),
//             &log_target
//         );
//         let request_state = state.get_string_or_default_to_unknown(
//             &format!("{robot_id}_script_request_state"),
//             &log_target
//         );

//         // Assuming ServiceRequestState::Initial evaluates to "Initial"
//         if trigger && request_state == "Initial" {
//             let script_content = state.get_string_or_default_to_unknown(
//                 &format!("{robot_id}_script_content"),
//                 &log_target
//             );

//             // Extract UUID or generate a fallback if none was provided
//             let mut uuid = state.get_string_or_default_to_unknown(
//                 &format!("{robot_id}_script_uuid"),
//                 &log_target
//             );
//             if uuid == "Unknown" || uuid.is_empty() {
//                 uuid = uuid::Uuid::new_v4().to_string(); // Optional: requires the `uuid` crate
//             }

//             log::info!(target: &log_target, "Accepted script request: {}", uuid);

//             // 1. Acknowledge the request in Redis so we don't process it twice
//             let mut new_state = state.clone();

//             // Assuming your state struct has a setter like this:
//             // (Adjust based on how your StateManager handles updates)
//             new_state.set_string(
//                 &format!("{robot_id}_script_request_state"),
//                 "Executing" // or ServiceRequestState::Executing.to_string()
//             );

//             StateManager::set_state(&mut con, &state.get_diff_partial_state(&new_state)).await;

//             // 2. Return the request to the driver to execute on the physical robot
//             return Some(ScriptRequest {
//                 uuid,
//                 script: script_content,
//             });
//         }
//     }
// }

// pub async fn publish_script_result(
//     robot_id: &str,
//     uuid: &str,
//     success: bool,
//     connection_manager: &Arc<ConnectionManager>
// ) {
//     let log_target = "ur_driver_script_result".to_string();
//     let mut con = connection_manager.get_connection().await;

//     let state_key = format!("{robot_id}_script_request_state");

//     // Fetch the current state to perform a diff update
//     if let Some(state) = StateManager::get_state_for_keys(&mut con, &[state_key.clone()], &log_target).await {
//         let mut new_state = state.clone();

//         let end_state = if success { "Completed" } else { "Failed" };

//         new_state.set_string(&state_key, end_state);
//         new_state.set_bool(&format!("{robot_id}_script_request_trigger"), false); // Optional: reset trigger

//         StateManager::set_state(&mut con, &state.get_diff_partial_state(&new_state)).await;
//         log::info!(target: &log_target, "Script [{}] finished. State set to: {}", uuid, end_state);
//     }
// }

// async fn script_request_server(
//     ur_address: String,
//     robot_id: String,
//     connection_manager: Arc<ConnectionManager>,
//     local_addr: watch::Receiver<Option<SocketAddr>>,
//     driver_state: Arc<Mutex<DriverState>>,
//     dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
// ) -> Result<(), Box<dyn std::error::Error>> {
//     let local_pool = LocalPoolHandle::new(1);

//     loop {
//         // Now passing the Redis dependencies into the wait function
//         if let Some(req) = wait_for_script_request(&robot_id, &connection_manager).await {
//             let local_addr = local_addr.borrow().clone();

//             if local_addr.is_none() || !driver_state.lock().unwrap().connected {
//                 println!("Not connected to robot yet, rejecting request: {}", req.uuid);
//                 publish_script_result(&robot_id, &req.uuid, false, &connection_manager).await;
//                 continue;
//             }

//             // ... (Rest of the validation logic remains the same) ...

//             let task_ur_address = ur_address.clone();
//             let task_dashboard_commands = dashboard_commands.clone();
//             let task_driver_state = driver_state.clone();
//             let task_conn_manager = connection_manager.clone();
//             let task_robot_id = robot_id.clone();

//             local_pool.spawn_pinned(move || async move {
//                 let result = handle_request(
//                     task_ur_address,
//                     local_addr_str,
//                     task_driver_state,
//                     task_dashboard_commands,
//                     req.clone(), // Clone req to use uuid in result
//                     cancel_receiver,
//                 ).await;

//                 // Result of execution: Determine if handle_request succeeded
//                 let success = result.is_ok(); // You might want deeper validation based on your handle_request return
//                 publish_script_result(&task_robot_id, &req.uuid, success, &task_conn_manager).await;

//                 if let Err(e) = result {
//                     println!("Error while handing goal: {}", e);
//                 }
//             });
//         } else {
//             break; // Break if wait_for_script_request intentionally returns None to kill the loop
//         }
//     }
//     Ok(())
// }
