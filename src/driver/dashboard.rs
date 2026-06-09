use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::sync::{mpsc, oneshot};

use crate::DashboardCommand;
use crate::driver::realtime_reader::connect_loop;

/// TODO: Implement this to yield new dashboard commands from your custom interface.
pub async fn wait_for_dashboard_command() -> Option<DashboardCommand> {
    // Example: Read from a custom channel or API
    // tokio::time::sleep(Duration::from_secs(1)).await;
    // None
    std::future::pending().await // blocks forever by default
}

pub async fn handle_dashboard_commands_loop(
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<bool>)>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        if let Some(cmd) = wait_for_dashboard_command().await {
            println!("Got dashboard command request: {:?}", cmd);
            let (sender, future) = oneshot::channel();
            if dashboard_commands.try_send((cmd, sender)).is_ok() {
                let _ok = future.await.unwrap_or(false);
                // Handle reply to interface if needed
            }
        } else {
            break;
        }
    }
    Ok(())
}

pub async fn dashboard(
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
