use std::net::SocketAddr;
use std::str::FromStr;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::io::AsyncReadExt;
use tokio::net::TcpStream;
use tokio::sync::watch;
use tokio::time::timeout;

use crate::{DriverState, lock_driver_state, safety_mode_aborts_goal, safety_mode_name};

/// Size of the RT frame this parser understands. The field offsets below are only
/// valid for this layout.
const RT_FRAME_SIZE: usize = 1220;

/// Upper bound on a frame length before it is treated as a desynced stream rather
/// than a real message. Generous next to `RT_FRAME_SIZE` so a newer controller
/// sending a longer frame is skipped rather than rejected outright.
const RT_MAX_FRAME_SIZE: usize = 8192;

const RT_READ_TIMEOUT: Duration = Duration::from_millis(1000);

/// Read the UR realtime interface (port 30003) into `DriverState`.
///
/// Offsets below are into `buf`, which already has the 4-byte length prefix
/// stripped, so each is the documented RT packet offset minus 4.
pub async fn realtime_reader(
    driver_state: Arc<Mutex<DriverState>>,
    ur_address: String,
    override_host_address: Option<String>,
    local_addr_sender: watch::Sender<Option<SocketAddr>>,
    log_target: String,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut size_bytes = [0u8; 4];
    let mut stream = connect_loop(&ur_address).await;

    let local_addr = if let Some(s) = &override_host_address {
        SocketAddr::from_str(&format!("{}:0", s))?
    } else {
        stream.local_addr()?
    };
    local_addr_sender.send(Some(local_addr))?;
    lock_driver_state(&driver_state).connected = true;

    loop {
        // Any failure to get a whole frame - timeout, error, or a length that makes
        // no sense - is handled the same way: drop the socket and reconnect. A read
        // error used to end the task, which collapsed `try_join!` and restarted the
        // entire driver for what is usually just a controller reboot.
        let frame = match read_frame(&mut stream, &mut size_bytes).await {
            Ok(frame) => frame,
            Err(e) => {
                log::warn!(target: &log_target, "Realtime read failed ({}), reconnecting.", e);
                lock_driver_state(&driver_state).connected = false;
                stream = connect_loop(&ur_address).await;
                lock_driver_state(&driver_state).connected = true;
                continue;
            }
        };

        let Some(buf) = frame else {
            // A frame we can parse the length of but not the contents of. Skipping
            // keeps the stream in sync, unlike reconnecting.
            continue;
        };

        let mut joints = Vec::with_capacity(6);
        let mut speeds = Vec::with_capacity(6);
        for i in 0..6 {
            joints.push(read_f64(&buf[248 + i * 8..248 + i * 8 + 8])); // packet 252, q actual
            speeds.push(read_f64(&buf[296 + i * 8..296 + i * 8 + 8])); // packet 300, qd actual
        }

        let mut tcp_pose = Vec::with_capacity(6);
        let mut forces = Vec::with_capacity(6);
        for i in 0..6 {
            tcp_pose.push(read_f64(&buf[440 + i * 8..440 + i * 8 + 8])); // packet 444, tool vector actual
            forces.push(read_f64(&buf[536 + i * 8..536 + i * 8 + 8])); // packet 540, TCP force
        }

        let digital_inputs = read_f64(&buf[680..688]) as u32; // packet 684
        let robot_mode = read_f64(&buf[752..760]) as i32; // packet 756
        // Packet 812 is Safety Mode. This was previously read as `robot_state`,
        // which conflated it with Robot Mode at packet 756 above.
        let safety_mode = read_f64(&buf[808..816]) as i32; // packet 812
        let speed_scaling = read_f64(&buf[936..944]); // packet 940
        let digital_outputs = read_f64(&buf[1040..1048]) as u32; // packet 1044
        // Packet 1052 is not the dashboard's stopped/playing/paused enum; see the
        // note on `DriverState::program_state_raw`. Kept for diagnostics.
        let program_state_raw = read_f64(&buf[1048..1056]) as i32; // packet 1052

        {
            let mut ds = lock_driver_state(&driver_state);
            ds.joint_values = joints;
            ds.joint_speeds = speeds;
            ds.tcp_pose = tcp_pose;
            ds.forces = forces;
            ds.robot_mode = robot_mode;
            ds.safety_mode = safety_mode;
            ds.speed_scaling = speed_scaling;
            ds.program_state_raw = program_state_raw;
            ds.digital_inputs = digital_inputs;
            ds.digital_outputs = digital_outputs;
        }

        // A safety stop means the script cannot finish, so an active goal is failed
        // here rather than left to time out. REDUCED is excluded on purpose: it is
        // a normal operating mode inside a reduced-speed zone, and the old
        // `safety_mode != 1` test aborted every goal that entered one.
        if safety_mode_aborts_goal(safety_mode) {
            let mut ds = lock_driver_state(&driver_state);
            if let Some(goal_sender) = ds.goal_sender.take() {
                log::warn!(
                    target: &log_target,
                    "Aborting active goal, safety mode is {}.",
                    safety_mode_name(safety_mode)
                );
                let _ = goal_sender.send(false);
            }
            ds.feedback_sender = None;
            ds.goal_id = None;
        }
    }
}

/// Read one length-prefixed RT frame.
///
/// `Err` means the socket is unusable. `Ok(None)` means the frame was read and
/// discarded because this parser does not understand its layout.
async fn read_frame(
    stream: &mut TcpStream,
    size_bytes: &mut [u8; 4],
) -> Result<Option<Vec<u8>>, String> {
    match timeout(RT_READ_TIMEOUT, stream.read_exact(size_bytes)).await {
        Ok(Ok(_)) => {}
        Ok(Err(e)) => return Err(e.to_string()),
        Err(_) => return Err("timed out reading frame length".to_string()),
    }

    let msg_size = u32::from_be_bytes(*size_bytes) as usize;

    // `msg_size` counts the 4 length bytes already consumed. Without this check a
    // short frame underflows the subtraction below and panics, and an absurd one
    // asks for a huge allocation.
    if msg_size <= 4 || msg_size > RT_MAX_FRAME_SIZE {
        return Err(format!("implausible frame length: {}", msg_size));
    }

    let mut buf = vec![0u8; msg_size - 4];
    // The body needs its own timeout too. `read_exact` without one leaves the
    // reader parked forever on a half-open connection.
    match timeout(RT_READ_TIMEOUT, stream.read_exact(&mut buf)).await {
        Ok(Ok(_)) => {}
        Ok(Err(e)) => return Err(e.to_string()),
        Err(_) => return Err("timed out reading frame body".to_string()),
    }

    if msg_size != RT_FRAME_SIZE {
        return Ok(None);
    }

    Ok(Some(buf))
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
