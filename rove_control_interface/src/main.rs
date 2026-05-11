use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Result;
use clap::Parser;
use tokio::sync::{mpsc, watch};
use tracing::info;

mod config;
mod ik_link;
mod kinova;
mod proto;
mod state;
mod teleop;

use config::Config;
use ik_link::{run_joint_pusher, run_kinova_pusher, run_velocs_listener, TwistSender};
use kinova::{run_data_subscriber, CommandSender};
use state::ArmSnapshot;
use teleop::{run_control_listener, run_telemetry_pusher, PeerTracker};

#[derive(Parser, Debug)]
#[command(name = "rove_control_interface", about = "Glue between teleop, rove_mvp_engine and the kinova_arm api")]
struct Cli {
    /// Path to config.toml.
    #[arg(short, long, default_value = "config.toml")]
    config: PathBuf,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "rove_control_interface=info,info".into()),
        )
        .init();

    let cli = Cli::parse();
    let cfg = Config::load(&cli.config)?;
    info!(?cfg, "loaded config");

    // Latest kinova arm snapshot — broadcast to whoever needs it.
    let (snap_tx, snap_rx) = watch::channel(ArmSnapshot::default());

    // IK engine output → kinova command pusher.
    let (cmd_tx, cmd_rx) = mpsc::channel(8);

    let twist = Arc::new(TwistSender::connect(&cfg.ik_engine).await?);
    let kinova_cmd = CommandSender::connect(&cfg.kinova).await?;
    let peer = Arc::new(PeerTracker::default());

    // ---- spawn tasks ----
    {
        let kinova_cfg = cfg.kinova.clone();
        let snap_tx = snap_tx.clone();
        tokio::spawn(async move {
            if let Err(e) = run_data_subscriber(&kinova_cfg, snap_tx).await {
                tracing::error!(target: "kinova", "data subscriber crashed: {e}");
            }
        });
    }
    let _ = snap_tx; // keep the watch alive even if the subscriber bounces.

    {
        let ik_cfg = cfg.ik_engine.clone();
        let joint_map = cfg.joint_map.clone();
        let snap_rx = snap_rx.clone();
        tokio::spawn(async move {
            if let Err(e) = run_joint_pusher(&ik_cfg, joint_map, snap_rx).await {
                tracing::error!(target: "ik", "joint pusher crashed: {e}");
            }
        });
    }
    {
        let ik_cfg = cfg.ik_engine.clone();
        let joint_map = cfg.joint_map.clone();
        tokio::spawn(async move {
            if let Err(e) = run_velocs_listener(&ik_cfg, joint_map, cmd_tx).await {
                tracing::error!(target: "ik", "velocs listener crashed: {e}");
            }
        });
    }
    {
        let pusher = kinova_cmd.clone();
        let rate = cfg.kinova.command_rate_hz;
        tokio::spawn(async move {
            if let Err(e) = run_kinova_pusher(pusher, cmd_rx, rate).await {
                tracing::error!(target: "kinova", "command pusher crashed: {e}");
            }
        });
    }
    {
        let drainer = kinova_cmd.clone();
        tokio::spawn(async move { drainer.drain_acks().await });
    }
    {
        let teleop_cfg = cfg.teleop.clone();
        let twist = twist.clone();
        let peer = peer.clone();
        tokio::spawn(async move {
            if let Err(e) = run_control_listener(&teleop_cfg, twist, peer).await {
                tracing::error!(target: "teleop", "control listener crashed: {e}");
            }
        });
    }
    {
        let teleop_cfg = cfg.teleop.clone();
        let snap_rx = snap_rx.clone();
        let peer = peer.clone();
        tokio::spawn(async move {
            if let Err(e) = run_telemetry_pusher(&teleop_cfg, snap_rx, peer).await {
                tracing::error!(target: "teleop", "telemetry pusher crashed: {e}");
            }
        });
    }

    info!("rove_control_interface ready");
    tokio::signal::ctrl_c().await?;
    info!("shutdown");
    Ok(())
}
