/// Emergency-stop and error-clearing commands.
use anyhow::Result;
use async_trait::async_trait;

use super::{Command, CommandContext};

/// Immediately e-stop every drive node via the HTTP API.
/// Resets per-node drive state so the next command re-sends axis_state + mode.
pub struct EstopCommand;

#[async_trait]
impl Command for EstopCommand {
    async fn execute(&self, ctx: &CommandContext) -> Result<()> {
        tracing::warn!("ESTOP — sending to all nodes");
        ctx.reset_drive_state();
        ctx.api.estop_all().await
    }
}

/// Clear errors on every drive node.
/// Resets per-node drive state so the next command re-sends axis_state + mode.
pub struct ClearErrorsCommand;

#[async_trait]
impl Command for ClearErrorsCommand {
    async fn execute(&self, ctx: &CommandContext) -> Result<()> {
        tracing::info!("clearing errors on all nodes");
        ctx.reset_drive_state();
        ctx.api.clear_errors_all().await
    }
}
