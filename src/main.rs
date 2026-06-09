use ur_redis_driver::driver::dashboard::{dashboard, handle_dashboard_commands_loop};
use ur_redis_driver::driver::handle_request::script_request_server;
use ur_redis_driver::driver::socker_server::socket_server;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::{mpsc, watch};
use ur_redis_driver::{DriverState, realtime_reader, state_publisher};

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