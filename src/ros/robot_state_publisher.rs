use std::error::Error;
use tokio::process::Command;
use tokio::task::JoinHandle;

// Instead of the below, broadcast the robot TF here and set the urdf as the robot_descitption parameter, i.e. our own robot state publisher
// The ros part shouold be a feature so that we can turn off ros if we don't need it
pub async fn robot_state_publisher(urdf: &str, namespace: &str) -> Result<JoinHandle<()>, Box<dyn Error>> {
    tokio::time::sleep(std::time::Duration::from_millis(2000)).await;
    let urdf_owned = urdf.to_string();
    let namespace_owned = namespace.to_string();
    let child_future = async move {
        let mut child = Command::new("ros2")
            .arg("run")
            .arg("robot_state_publisher")
            .arg("robot_state_publisher")
            .arg("--ros-args")
            .arg("-r")
            .arg(format!("__ns:=/{}", namespace_owned))
            .arg("-p")
            .arg(format!("robot_description:={}", urdf_owned))
            .spawn()
            .expect("Failed to spawn robot_state_publisher process");

        match child.wait().await {
            Ok(status) => {
                if !status.success() {
                    eprintln!("robot_state_publisher exited with code {:?}", status.code());
                } else {
                    println!("robot_state_publisher exited successfully");
                }
            }
            Err(e) => {
                eprintln!("Error waiting for child: {e}");
            }
        }
    };

    let handle = tokio::spawn(child_future);

    Ok(handle)
}