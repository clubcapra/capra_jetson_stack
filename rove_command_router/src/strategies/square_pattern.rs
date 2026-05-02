/// Example strategy: drive in a square pattern.
///
/// UDP payload: `{"action": "start", "params": {"speed": 1.0, "side_seconds": 3.0, "laps": 1}}`
use std::time::Duration;

use anyhow::Result;
use async_trait::async_trait;

use super::{Strategy, StrategyContext};

pub struct SquarePattern;

#[async_trait]
impl Strategy for SquarePattern {
    fn name(&self) -> &str {
        "square_pattern"
    }
    fn description(&self) -> &str {
        "Drive in a square — params: speed, side_seconds, turn_seconds, laps"
    }

    async fn run(&self, ctx: StrategyContext) -> Result<()> {
        let speed = ctx.param_f32("speed", 1.0);
        let side_s = ctx.param_f32("side_seconds", 3.0);
        let turn_s = ctx.param_f32("turn_seconds", 1.0);
        let laps = ctx.param_u32("laps", 1);

        tracing::info!(speed, side_s, turn_s, laps, "square_pattern started");

        for lap in 0..laps {
            for side in 0..4u32 {
                tracing::info!(lap = lap + 1, side = side + 1, "forward");
                ctx.drive(speed, speed).await?;
                ctx.sleep(Duration::from_secs_f32(side_s)).await?;

                tracing::info!(lap = lap + 1, side = side + 1, "turning");
                ctx.drive(speed, -speed).await?;
                ctx.sleep(Duration::from_secs_f32(turn_s)).await?;
            }
        }

        ctx.stop_drive().await?;
        tracing::info!("square_pattern complete");
        Ok(())
    }
}
