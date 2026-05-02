/// Strategy: rotate in place by a given number of degrees.
///
/// UDP payload: `{"action": "start", "params": {"degrees": 90, "speed": 1.0}}`
use std::time::Duration;

use anyhow::Result;
use async_trait::async_trait;

use super::{Strategy, StrategyContext};

/// Rough calibration: seconds per 360° at 1.0 rev/s counter-rotation.
/// Tune for your track width and wheel diameter.
const SECONDS_PER_360: f32 = 4.0;

pub struct PointTurn;

#[async_trait]
impl Strategy for PointTurn {
    fn name(&self) -> &str {
        "point_turn"
    }
    fn description(&self) -> &str {
        "Rotate in place — params: degrees, speed"
    }

    async fn run(&self, ctx: StrategyContext) -> Result<()> {
        let degrees = ctx.param_f32("degrees", 90.0);
        let speed = ctx.param_f32("speed", 1.0).abs();

        let duration = (degrees.abs() / 360.0) * SECONDS_PER_360 / speed;
        let dir = if degrees >= 0.0 { 1.0 } else { -1.0 };

        tracing::info!(degrees, speed, duration, "point_turn started");

        ctx.drive(speed * dir, -speed * dir).await?;
        ctx.sleep(Duration::from_secs_f32(duration)).await?;
        ctx.stop_drive().await?;

        tracing::info!("point_turn complete");
        Ok(())
    }
}
