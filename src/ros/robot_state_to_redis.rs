use ordered_float::OrderedFloat;
use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;
use std::sync::Arc;
use std::time::SystemTime;

use futures::{Stream, StreamExt};
use micro_sp::*;

fn tf_to_sp_tf(tf: TransformStamped) -> SPTransformStamped {
    SPTransformStamped {
        active_transform: true,
        enable_transform: true,
        time_stamp: SystemTime::now(),
        parent_frame_id: tf.header.frame_id,
        child_frame_id: tf.child_frame_id,
        transform: SPTransform {
            translation: SPTranslation {
                x: OrderedFloat(tf.transform.translation.x),
                y: OrderedFloat(tf.transform.translation.y),
                z: OrderedFloat(tf.transform.translation.z),
            },
            rotation: SPRotation {
                x: OrderedFloat(tf.transform.rotation.x),
                y: OrderedFloat(tf.transform.rotation.y),
                z: OrderedFloat(tf.transform.rotation.z),
                w: OrderedFloat(tf.transform.rotation.w),
            },
        },
        metadata: MapOrUnknown::UNKNOWN,
    }
}

pub async fn robot_state_to_redis(
    robot_name: &str,
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    connection_manager: &Arc<ConnectionManager>,
) -> Result<(), Box<dyn std::error::Error>> {

    let initial = vec![
        ("base_link", "base_link_inertia"),
        ("base_link_inertia", "shoulder_link"),
        ("shoulder_link", "upper_arm_link"),
        ("upper_arm_link", "forearm_link"),
        ("forearm_link", "wrist_1_link"),
        ("wrist_1_link", "wrist_2_link"),
        ("wrist_2_link", "wrist_3_link"),
        ("wrist_3_link", "flange"),
        ("wrist_3_link", "ft_frame"),
        ("flange", "tool0"),
    ];

    let mut con = connection_manager.get_connection().await;
    TransformsManager::insert_transforms(
        &mut con,
        &initial
            .iter()
            .map(|t| SPTransformStamped {
                active_transform: true,
                enable_transform: true,
                time_stamp: SystemTime::now(),
                parent_frame_id: t.0.to_string(),
                child_frame_id: t.1.to_string(),
                transform: SPTransform::default(),
                metadata: MapOrUnknown::UNKNOWN,
            })
            .collect::<Vec<SPTransformStamped>>(),
    )
    .await?;

    let update_interval = Duration::from_millis(ROBOT_STATE_UPDATE_INTERVAL_MS);
    let mut last_processed_time = Instant::now();

    loop {
        match subscriber.next().await {
            Some(message) => {

                let now = Instant::now();
                if now.duration_since(last_processed_time) < update_interval {
                    continue; 
                }

                last_processed_time = now;

                if let Err(_) = connection_manager
                    .check_redis_health(&&format!("{robot_name}_action_client"))
                    .await
                {
                    continue;
                }
                let links_to_move = [
                    "base_link_inertia",
                    "shoulder_link",
                    "upper_arm_link",
                    "forearm_link",
                    "upper_arm_link",
                    "wrist_1_link",
                    "wrist_2_link",
                    "wrist_3_link",
                    "flange",
                    "ft_frame",
                    "tool0",
                ];

                for tf in &message.transforms {
                    if links_to_move.contains(&tf.child_frame_id.as_str()) {
                        TransformsManager::move_transform(
                            &mut con,
                            &tf.child_frame_id,
                            tf_to_sp_tf(tf.clone()).transform,
                        )
                        .await?;
                    }
                }
            }
            None => {
                r2r::log_error!(
                    &format!("{robot_name}_tf_to_redis"),
                    "TF subscriber did not get the message?"
                );
            }
        }
    }
}
