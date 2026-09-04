use futures::stream::StreamExt;
use futures::SinkExt;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;
use tokio::sync::watch;
use tokio_util::codec::{Framed, LinesCodec};

use crate::{DriverState, lock_driver_state, ur_driver_socket_port};

/// Prefix of the progress lines the trajectory templates send.
///
/// Must match `templates/trajectory_*.script`.
const WAYPOINT_REACHED_PREFIX: &str = "waypoint_reached ";

pub async fn socket_server(
    driver_state: Arc<Mutex<DriverState>>,
    mut local_addr: watch::Receiver<Option<SocketAddr>>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut addr = None;
    while addr.is_none() {
        local_addr.changed().await?;
        addr = local_addr.borrow().clone();
    }

    let mut addr = addr.unwrap();
    let port = ur_driver_socket_port();
    addr.set_port(port);

    println!("Starting socket server at {}", addr);

    // The most likely way a second driver on one host fails to start, and a bare
    // "Address already in use (os error 98)" from main's restart loop says nothing
    // about which port or how to change it.
    let listener = TcpListener::bind(&addr).await.map_err(|e| {
        format!(
            "could not bind the driver socket server on {addr}: {e}.              Is another ur_redis_driver already using port {port}?              Set UR_DRIVER_SOCKET_PORT to give this one its own."
        )
    })?;
    loop {
        // A failed accept is per-connection (the peer went away mid-handshake, the
        // fd table is momentarily full) and says nothing about the listener. `?`
        // here used to end the task and restart the whole driver.
        let (stream, addr) = match listener.accept().await {
            Ok(accepted) => accepted,
            Err(e) => {
                println!("Failed to accept a script connection: {}", e);
                continue;
            }
        };
        println!("New connection: {}", addr);

        let (goal_id, handshake_sender, feedback_sender) = {
            let mut ds = lock_driver_state(&driver_state);
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
                    let mut ds = lock_driver_state(&driver_state);
                    if let Some(goal_sender) = ds.goal_sender.take() {
                        let _ = goal_sender.send(true);
                    }
                }
                Some(Ok(s)) if s == "error" => {
                    println!("got ERROR, we are done.");
                    let mut ds = lock_driver_state(&driver_state);
                    if let Some(goal_sender) = ds.goal_sender.take() {
                        let _ = goal_sender.send(false);
                    }
                }
                // Trajectory progress, not feedback. The templates emit one of
                // these per waypoint so a paused blended trajectory can resume
                // where it stopped instead of driving back through the path it
                // already covered. With blending the line fires as the controller
                // hands over to the next move rather than at a full stop, which is
                // exactly the boundary a resume wants.
                Some(Ok(s)) if s.starts_with(WAYPOINT_REACHED_PREFIX) => {
                    let index = s[WAYPOINT_REACHED_PREFIX.len()..].trim().parse::<usize>();
                    match index {
                        Ok(index) => {
                            // The script reports the index it reached, so the count
                            // of finished waypoints is one higher.
                            lock_driver_state(&driver_state).waypoints_completed = index + 1;
                        }
                        Err(_) => {
                            println!("could not read a waypoint index out of '{}'", s);
                        }
                    }
                }
                Some(Ok(s)) => {
                    println!("got {}, sending as feedback", s);
                    let _ = feedback_sender.send(s).await;
                }
                _ => {
                    // The script's socket closing without an "ok"/"error" line means
                    // the script ended without reporting - it was killed by a
                    // dashboard `stop`, or the program was aborted on the pendant.
                    //
                    // Resolving the goal here is what makes `stop` usable: leaving
                    // it unresolved parks `handle_request` on `goal_receiver`
                    // forever, so `goal_id` stays `Some` and *every* later request
                    // is rejected with "a goal is already running" until the driver
                    // is restarted.
                    let mut ds = lock_driver_state(&driver_state);
                    match ds.goal_sender.take() {
                        Some(goal_sender) => {
                            println!("Script socket closed with no result, failing the active goal.");
                            let _ = goal_sender.send(false);
                        }
                        None => {
                            println!("Socket connection closed, dropping feedback sender.");
                        }
                    }
                    ds.feedback_sender = None;
                    ds.goal_id = None;
                    break;
                }
            };
        }
    }
}