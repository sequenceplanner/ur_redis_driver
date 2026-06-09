use futures::stream::StreamExt;
use futures::SinkExt;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use tokio::net::TcpListener;
use tokio::sync::watch;
use tokio_util::codec::{Framed, LinesCodec};

use crate::{DriverState, UR_DRIVER_SOCKET_PORT};

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