use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::watch;
use tokio::time::timeout;

use crate::DriverState;

pub async fn realtime_reader(
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
        // handle outer timeout error
        if let Err(_) = ret {
            {
                // We no longer kill the goal here since we know the state of execution
                // via the socket communication.
                let mut ds = driver_state.lock().unwrap();
                ds.connected = false;
            }
            println!("timeout on read, reconnecting... ");
            stream = connect_loop(&ur_address).await;
            driver_state.lock().unwrap().connected = true;

            continue;
        } else if let Ok(ret) = ret {
            if let Err(e) = ret {
                println!("unexpected read error: {}", e);
                return Err("oh no".into());
            }
        }

        let msg_size = u32::from_be_bytes(size_bytes) as usize;

        // need to subtract the 4 we already read (msg_size)
        let mut buf: Vec<u8> = Vec::new();
        buf.resize(msg_size - 4, 0);
        stream.read_exact(&mut buf).await?;

        if msg_size != 1220 {
            println!("got unkown frame length: {}", msg_size);
        }
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
                // robot has entered protective or emergency stop. If
                // there is an active goal, abort it.  we are
                // finished. succeed and remove the action goal
                // handle.
                let mut ds = driver_state.lock().unwrap();
                if let Some(goal_sender) = ds.goal_sender.take() {
                    println!("aborting due to protective stop");
                    let _ = goal_sender.send(false);
                }
                ds.feedback_sender = None;
                ds.goal_id = None;
            }
        }
    }
}

pub async fn connect_loop(address: &str) -> TcpStream {
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

fn read_f64(slice: &[u8]) -> f64 {
    let mut bytes = [0u8; 8];
    bytes.copy_from_slice(slice);
    f64::from_be_bytes(bytes)
}
