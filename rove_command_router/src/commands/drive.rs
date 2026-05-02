/// Drive commands — translate setpoints into per-node ODrive commands.
///
/// Follows the same state-tracking pattern as test_tracks.py:
/// - `axis_state=8` (ClosedLoopControl) sent only on first command or after reset
/// - `control_mode` / `input_mode` sent only when they change
/// - setpoint (`input_vel` or `input_torque`) sent every cycle
use anyhow::Result;
use async_trait::async_trait;

use crate::config::DriveMode;

use super::{Command, CommandContext, NodeDriveState};

const AXIS_CLOSED_LOOP: u32 = 8;
const CTRL_VELOCITY: u32 = 2;
const CTRL_TORQUE: u32 = 1;
const INPUT_PASSTHROUGH: u32 = 1;

/// Send absolute setpoints to the tracks.
/// In velocity mode: values are rev/s.  In torque mode: values are Nm.
/// Right-side nodes are sign-flipped for mirrored motor mounting.
pub struct DriveCommand {
    pub left: f32,
    pub right: f32,
}

#[async_trait]
impl Command for DriveCommand {
    async fn execute(&self, ctx: &CommandContext) -> Result<()> {
        let mut states = ctx.node_drive_state.lock().unwrap();

        let (ctrl_mode, input_key) = match ctx.drive_mode {
            DriveMode::Velocity => (CTRL_VELOCITY, "input_vel"),
            DriveMode::Torque => (CTRL_TORQUE, "input_torque"),
        };

        for &nid in ctx.left_nodes.iter().chain(&ctx.right_nodes) {
            let is_left = ctx.left_nodes.contains(&nid);
            let setpoint = if is_left {
                self.left
            } else {
                -self.right
            };

            let nds = states.entry(nid).or_insert_with(NodeDriveState::default);
            let mut cmd = serde_json::Map::new();

            // axis_state → only send when it changes (or first time)
            if nds.axis_state != Some(AXIS_CLOSED_LOOP) {
                cmd.insert("axis_state".into(), serde_json::json!(AXIS_CLOSED_LOOP));
                nds.axis_state = Some(AXIS_CLOSED_LOOP);
            }

            // control_mode + input_mode → only send when they change
            if nds.control_mode != Some(ctrl_mode)
                || nds.input_mode != Some(INPUT_PASSTHROUGH)
            {
                cmd.insert("control_mode".into(), serde_json::json!(ctrl_mode));
                cmd.insert("input_mode".into(), serde_json::json!(INPUT_PASSTHROUGH));
                nds.control_mode = Some(ctrl_mode);
                nds.input_mode = Some(INPUT_PASSTHROUGH);
            }

            // setpoint → every cycle
            let val = (setpoint * 10000.0).round() / 10000.0;
            cmd.insert(input_key.into(), serde_json::json!(val));

            ctx.api
                .send_drive_udp(nid, &serde_json::Value::Object(cmd));
        }
        Ok(())
    }
}

/// Drive from normalized inputs (-1..1), scaled by `max_velocity` or `max_torque`
/// depending on the configured drive mode.
pub struct NormalizedDriveCommand {
    pub left_norm: f32,
    pub right_norm: f32,
}

#[async_trait]
impl Command for NormalizedDriveCommand {
    async fn execute(&self, ctx: &CommandContext) -> Result<()> {
        let scale = match ctx.drive_mode {
            DriveMode::Velocity => ctx.max_velocity,
            DriveMode::Torque => ctx.max_torque,
        };
        let left = self.left_norm * scale;
        let right = self.right_norm * scale;
        DriveCommand { left, right }.execute(ctx).await
    }
}
