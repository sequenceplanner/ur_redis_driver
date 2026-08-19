use std::sync::Arc;
use std::time::Duration;

use micro_sp::*;
use tokio::sync::{mpsc, oneshot};
use tokio::time::{MissedTickBehavior, interval, timeout};

use crate::{DashboardCommand, DashboardReply, state_bool_or, state_string_or};

/// Poll period for the dashboard request keys.
///
/// Deliberately much slower than the motion loop's 5 ms: dashboard commands are
/// operator-scale events, not a control signal, and this loop blocks on a socket
/// round trip once triggered.
const DASHBOARD_POLL_INTERVAL_MS: u64 = 50;

/// Ceiling on one dashboard command, including the reconnect the dashboard task
/// may do underneath. Longer than any individual step in `reset_protective_stop`.
const DASHBOARD_COMMAND_TIMEOUT: Duration = Duration::from_secs(10);

/// Redis-facing half of the dashboard interface.
///
/// Mirrors `command_server`'s trigger/state handshake on the `_dashboard_*` keys:
/// write `{robot}_dashboard_command` (and `_dashboard_command_arg` when the command
/// takes one), then set `{robot}_dashboard_request_trigger` to true.
/// `{robot}_dashboard_request_state` walks `initial -> executing -> succeeded|failed`
/// and `{robot}_dashboard_request_result` carries the controller's reply.
///
/// This is its own task rather than a branch inside `command_server` because it
/// awaits the controller's answer. Folding it in would let a ten-second dashboard
/// command stall the 5 ms loop that services motion requests and cancellations.
pub async fn dashboard_server(
    robot_name: &str,
    connection_manager: &Arc<ConnectionManager>,
    dashboard_commands: mpsc::Sender<(DashboardCommand, oneshot::Sender<DashboardReply>)>,
) -> Result<(), Box<dyn std::error::Error>> {
    let log_target = format!("{robot_name}_dashboard_server");

    let mut ticker = interval(Duration::from_millis(DASHBOARD_POLL_INTERVAL_MS));
    ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

    // One long-lived handle for the whole task. `SPConnection` is multiplexed and
    // self-healing, so it stays valid across Redis reconnects.
    let mut con = connection_manager.get_connection().await;

    let key = |suffix: &str| format!("{robot_name}_{suffix}");

    let trigger_key = vec![key("dashboard_request_trigger")];
    let request_keys: Vec<String> = [
        "dashboard_request_trigger",
        "dashboard_request_state",
        "dashboard_command",
        "dashboard_command_arg",
    ]
    .iter()
    .map(|s| key(s))
    .collect();

    loop {
        ticker.tick().await;

        // Idle tick reads one key, not four.
        let Some(flags) =
            StateManager::get_state_for_keys(&mut con, &trigger_key, &log_target).await
        else {
            continue;
        };
        if !state_bool_or(&flags, &key("dashboard_request_trigger"), false, &log_target) {
            continue;
        }

        let Some(state) =
            StateManager::get_state_for_keys(&mut con, &request_keys, &log_target).await
        else {
            continue;
        };

        // Consume the edge immediately, so a command cannot be run twice if the
        // work below outlasts a tick.
        StateManager::set_sp_value(&mut con, &key("dashboard_request_trigger"), &false.to_spvalue())
            .await;

        let request_state = state_string_or(
            &state,
            &key("dashboard_request_state"),
            &ActionRequestState::UNKNOWN.to_string(),
            &log_target,
        );
        if request_state != ActionRequestState::Initial.to_string() {
            log::warn!(
                target: &log_target,
                "Dashboard request triggered while in state '{}', ignoring. Reset it to 'initial' first.",
                request_state
            );
            continue;
        }

        let command_name =
            state_string_or(&state, &key("dashboard_command"), "UNKNOWN", &log_target);
        let command_arg =
            state_string_or(&state, &key("dashboard_command_arg"), "", &log_target);

        let Some(command) = DashboardCommand::parse(&command_name, &command_arg) else {
            log::error!(target: &log_target, "Unknown dashboard command: '{}'.", command_name);
            finish(
                &mut con,
                robot_name,
                false,
                &format!("unknown dashboard command '{}'", command_name),
            )
            .await;
            continue;
        };

        StateManager::set_sp_value(
            &mut con,
            &key("dashboard_request_state"),
            &ActionRequestState::Executing.to_string().to_spvalue(),
        )
        .await;

        log::info!(target: &log_target, "Executing dashboard command '{}'.", command_name);

        let (reply_sender, reply_receiver) = oneshot::channel();

        // `send` rather than `try_send`: a full channel means the dashboard task is
        // busy, not that the command is invalid, and waiting a moment is better
        // than failing the request. The timeout below still bounds the wait.
        if dashboard_commands.send((command, reply_sender)).await.is_err() {
            finish(&mut con, robot_name, false, "dashboard task is not running").await;
            continue;
        }

        let (success, response) = match timeout(DASHBOARD_COMMAND_TIMEOUT, reply_receiver).await {
            Ok(Ok(reply)) => (reply.success, reply.response),
            Ok(Err(_)) => (false, "dashboard task dropped the request".to_string()),
            Err(_) => (false, "timed out waiting for the controller".to_string()),
        };

        log::info!(
            target: &log_target,
            "Dashboard command '{}' finished: success={}, response='{}'.",
            command_name, success, response
        );

        finish(&mut con, robot_name, success, &response).await;
    }
}

/// Write the terminal state and result for a dashboard request.
async fn finish(con: &mut SPConnection, robot_name: &str, success: bool, response: &str) {
    let request_state = if success {
        ActionRequestState::Succeeded.to_string()
    } else {
        ActionRequestState::Failed.to_string()
    };

    StateManager::set_sp_value(
        con,
        &format!("{robot_name}_dashboard_request_result"),
        &response.to_spvalue(),
    )
    .await;
    StateManager::set_sp_value(
        con,
        &format!("{robot_name}_dashboard_request_state"),
        &request_state.to_spvalue(),
    )
    .await;
}
