/// Strategy design pattern — autonomy algorithms implement [`Strategy`] and
/// are auto-assigned a UDP endpoint by the router.
///
/// Add a new strategy:
/// 1. Create `src/strategies/my_algo.rs` implementing [`Strategy`].
/// 2. Add `pub mod my_algo;` here.
/// 3. Instantiate it in [`register_all`].
///
/// The router assigns UDP port `base_port + index` in registration order.
pub mod point_turn;
pub mod square_pattern;

use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use async_trait::async_trait;
use tokio_util::sync::CancellationToken;

use crate::commands::drive::DriveCommand;
use crate::commands::{Command, CommandContext};

// ── Strategy trait ──────────────────────────────────────────────────────────

/// Base trait for the strategy design pattern.
#[async_trait]
pub trait Strategy: Send + Sync + 'static {
    fn name(&self) -> &str;
    fn description(&self) -> &str;

    /// Execute the algorithm.  Use `ctx.drive()` and `ctx.sleep()` which are
    /// cancellation-aware — they return `Err(Cancelled)` when the strategy is
    /// stopped by an operator override or estop.
    async fn run(&self, ctx: StrategyContext) -> Result<()>;
}

// ── Strategy context ────────────────────────────────────────────────────────

/// Passed to a running strategy — provides state access and cancellable helpers.
pub struct StrategyContext {
    pub cmd_ctx: Arc<CommandContext>,
    pub params: serde_json::Value,
    pub cancel: CancellationToken,
}

#[derive(Debug, thiserror::Error)]
#[error("strategy cancelled")]
pub struct Cancelled;

impl StrategyContext {
    /// Send a drive command.  Returns `Err` if the strategy was cancelled.
    pub async fn drive(&self, left: f32, right: f32) -> Result<()> {
        if self.cancel.is_cancelled() {
            anyhow::bail!(Cancelled);
        }
        DriveCommand { left, right }
            .execute(&self.cmd_ctx)
            .await
    }

    /// Cancellable sleep.
    pub async fn sleep(&self, duration: Duration) -> Result<()> {
        tokio::select! {
            _ = tokio::time::sleep(duration) => Ok(()),
            _ = self.cancel.cancelled() => Err(Cancelled.into()),
        }
    }

    /// Zero all drives.
    pub async fn stop_drive(&self) -> Result<()> {
        DriveCommand {
            left: 0.0,
            right: 0.0,
        }
        .execute(&self.cmd_ctx)
        .await
    }

    /// Read a param with a default.
    pub fn param_f32(&self, key: &str, default: f32) -> f32 {
        self.params
            .get(key)
            .and_then(|v| v.as_f64())
            .map(|v| v as f32)
            .unwrap_or(default)
    }

    pub fn param_u32(&self, key: &str, default: u32) -> u32 {
        self.params
            .get(key)
            .and_then(|v| v.as_u64())
            .map(|v| v as u32)
            .unwrap_or(default)
    }
}

// ── Registry ────────────────────────────────────────────────────────────────

/// Return all available strategies.  Add new strategies here.
pub fn register_all() -> Vec<Box<dyn Strategy>> {
    vec![
        Box::new(square_pattern::SquarePattern),
        Box::new(point_turn::PointTurn),
    ]
}
