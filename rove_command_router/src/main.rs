mod api_client;
mod commands;
mod config;
mod proto;
mod robot_state;
mod router;
mod strategies;

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "command_router=info".into()),
        )
        .init();

    let cfg_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("config.toml")
        });

    let cfg = config::load(&cfg_path)?;
    tracing::info!("API target: {}:{}", cfg.api.host, cfg.api.http_port);

    let api = Arc::new(api_client::ApiClient::new(&cfg.api.host, cfg.api.http_port)?);

    // Discover ODrive nodes
    tracing::info!("discovering nodes...");
    let nodes = api.discover().await?;
    if nodes.is_empty() {
        tracing::error!("no ODrive nodes found — is the API running?");
        std::process::exit(1);
    }
    for (nid, info) in nodes.iter() {
        let side = if cfg.robot.left_nodes.contains(nid) {
            "left"
        } else {
            "right"
        };
        tracing::info!(
            "  odrive_{}  {:>5}  cmd_port={}",
            nid,
            side,
            info.command_port
        );
    }

    let state = Arc::new(robot_state::RobotState::new(
        api.clone(),
        Duration::from_millis(100),
    ));
    let router = router::CommandRouter::new(&cfg, api, state.clone());

    tracing::info!("");
    tokio::select! {
        _ = state.run() => {},
        res = router.run() => {
            if let Err(e) = res {
                tracing::error!(error = %e, "router failed");
            }
        },
    }

    Ok(())
}
