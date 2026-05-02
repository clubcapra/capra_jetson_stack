/// Command design pattern — every action the router can perform is a
/// `Command` object with an `execute` method.
pub mod drive;
pub mod estop;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use anyhow::Result;
use async_trait::async_trait;

use crate::api_client::ApiClient;
use crate::config::DriveMode;
use crate::robot_state::RobotState;

/// Per-node CAN state tracker — mirrors the approach in test_tracks.py.
///
/// The ODrive requires axis_state and controller mode to be set before
/// setpoints take effect.  We track what was last sent so we only emit
/// transition frames when something actually changes, avoiding redundant
/// CAN traffic and unwanted state-machine side-effects on the drive.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NodeDriveState {
    pub axis_state: Option<u32>,
    pub control_mode: Option<u32>,
    pub input_mode: Option<u32>,
}

/// Shared context passed to every command on execution.
pub struct CommandContext {
    pub api: Arc<ApiClient>,
    pub state: Arc<RobotState>,
    pub left_nodes: Vec<u8>,
    pub right_nodes: Vec<u8>,
    pub drive_mode: DriveMode,
    pub max_velocity: f32,
    pub max_torque: f32,
    /// Per-node tracking: only send axis_state / control_mode when they change.
    pub node_drive_state: Mutex<HashMap<u8, NodeDriveState>>,
}

impl CommandContext {
    /// Reset all per-node tracking (after estop or clear_errors).
    /// Next drive command will re-send axis_state and control_mode.
    pub fn reset_drive_state(&self) {
        self.node_drive_state.lock().unwrap().clear();
    }
}

/// Base trait for the command design pattern.
#[async_trait]
pub trait Command: Send + Sync {
    async fn execute(&self, ctx: &CommandContext) -> Result<()>;
}
