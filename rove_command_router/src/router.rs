/// Command Router — the autonomy layer.
///
/// Implements the **router design pattern**: incoming UDP packets are dispatched
/// to the correct handler based on which endpoint received them.
///
/// - Estop endpoint   → e-stop all drives, cancel any active strategy.
/// - Control endpoint → operator override: cancel strategy, forward drive commands.
/// - Strategy endpoints (auto-generated) → start / stop autonomy algorithms.
use std::sync::Arc;

use anyhow::Result;
use prost::Message;
use serde_json::Value;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

use crate::commands::drive::{DriveCommand, NormalizedDriveCommand};
use crate::commands::estop::{ClearErrorsCommand, EstopCommand};
use crate::commands::{Command, CommandContext};
use crate::config::Config;
use crate::proto::RoveControl;
use crate::robot_state::RobotState;
use crate::strategies::{self, Cancelled, Strategy, StrategyContext};

// ── Active strategy tracking ────────────────────────────────────────────────

struct ActiveStrategy {
    name: String,
    cancel: CancellationToken,
    handle: JoinHandle<()>,
}

// ── Router ──────────────────────────────────────────────────────────────────

pub struct CommandRouter {
    cmd_ctx: Arc<CommandContext>,
    active: Arc<Mutex<Option<ActiveStrategy>>>,
    estopped: Arc<std::sync::atomic::AtomicBool>,
    strategies: Vec<Arc<dyn Strategy>>,
    estop_port: u16,
    control_port: u16,
    strategy_base_port: u16,
}

impl CommandRouter {
    pub fn new(config: &Config, api: Arc<crate::api_client::ApiClient>, state: Arc<RobotState>) -> Self {
        let cmd_ctx = Arc::new(CommandContext {
            api,
            state,
            left_nodes: config.robot.left_nodes.clone(),
            right_nodes: config.robot.right_nodes.clone(),
            drive_mode: config.robot.drive_mode,
            max_velocity: config.robot.max_velocity,
            max_torque: config.robot.max_torque,
            node_drive_state: std::sync::Mutex::new(std::collections::HashMap::new()),
        });

        let strategies: Vec<Arc<dyn Strategy>> = strategies::register_all()
            .into_iter()
            .map(|b| Arc::from(b))
            .collect();

        Self {
            cmd_ctx,
            active: Arc::new(Mutex::new(None)),
            estopped: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            strategies,
            estop_port: config.endpoints.estop_port,
            control_port: config.endpoints.control_port,
            strategy_base_port: config.strategies.base_port,
        }
    }

    pub async fn run(self) -> Result<()> {
        let router = Arc::new(self);

        // Bind estop endpoint
        let sock = UdpSocket::bind(("0.0.0.0", router.estop_port)).await?;
        tracing::info!("ESTOP endpoint     → UDP :{}", router.estop_port);
        let r = router.clone();
        tokio::spawn(async move { recv_loop(sock, move |data, addr| r.clone().on_estop(data, addr)).await });

        // Bind control endpoint
        let sock = UdpSocket::bind(("0.0.0.0", router.control_port)).await?;
        tracing::info!("CONTROL endpoint   → UDP :{}", router.control_port);
        let r = router.clone();
        tokio::spawn(async move { recv_loop(sock, move |data, addr| r.clone().on_control(data, addr)).await });

        // Bind one endpoint per strategy
        for (i, strat) in router.strategies.iter().enumerate() {
            let port = router.strategy_base_port + i as u16;
            let name = strat.name().to_string();
            let desc = strat.description();
            tracing::info!("STRATEGY {:<20} → UDP :{}  {}", name, port, desc);

            let sock = UdpSocket::bind(("0.0.0.0", port)).await?;
            let r = router.clone();
            let n = name.clone();
            tokio::spawn(async move {
                recv_loop(sock, move |data, addr| {
                    let r = r.clone();
                    let n = n.clone();
                    async move { r.on_strategy(&n, data, addr).await }
                })
                .await
            });
        }

        if router.strategies.is_empty() {
            tracing::info!("(no strategies registered)");
        }

        tracing::info!("command router ready");
        // Block forever
        std::future::pending::<()>().await;
        Ok(())
    }

    // ── Estop handler ───────────────────────────────────────────────────

    async fn on_estop(self: Arc<Self>, _data: Vec<u8>, addr: std::net::SocketAddr) {
        tracing::warn!(%addr, "ESTOP received");
        self.estopped
            .store(true, std::sync::atomic::Ordering::SeqCst);

        self.cancel_active().await;

        if let Err(e) = EstopCommand.execute(&self.cmd_ctx).await {
            tracing::error!(error = %e, "estop command failed");
        }
    }

    // ── Control handler (operator override) ─────────────────────────────

    async fn on_control(self: Arc<Self>, data: Vec<u8>, addr: std::net::SocketAddr) {
        tracing::debug!(%addr, len = data.len(), "control packet received");

        if self.estopped.load(std::sync::atomic::Ordering::SeqCst) {
            // Check if this is a clear_errors message
            if let Ok(obj) = serde_json::from_slice::<Value>(&data) {
                if obj.get("clear_errors").and_then(|v| v.as_bool()).unwrap_or(false) {
                    self.estopped
                        .store(false, std::sync::atomic::Ordering::SeqCst);
                    if let Err(e) = ClearErrorsCommand.execute(&self.cmd_ctx).await {
                        tracing::error!(error = %e, "clear_errors failed");
                    }
                    return;
                }
            }
            tracing::debug!("control ignored — estopped");
            return;
        }

        // Cancel any active strategy — operator takes over
        {
            let active = self.active.lock().await;
            if active.is_some() {
                drop(active);
                tracing::info!("operator override — cancelling active strategy");
                self.cancel_active().await;
            }
        }

        // Parse and execute
        match parse_control(&data) {
            Some(cmd) => {
                if let Err(e) = cmd.execute(&self.cmd_ctx).await {
                    tracing::warn!(error = %e, "drive command failed");
                }
            }
            None => {
                tracing::warn!(
                    len = data.len(),
                    first_bytes = ?&data[..data.len().min(16)],
                    "control packet could not be parsed as proto or JSON"
                );
            }
        }
    }

    // ── Strategy handler ────────────────────────────────────────────────

    async fn on_strategy(self: Arc<Self>, name: &str, data: Vec<u8>, _addr: std::net::SocketAddr) {
        if self.estopped.load(std::sync::atomic::Ordering::SeqCst) {
            tracing::warn!("strategy command ignored — estopped");
            return;
        }

        let msg: Value = match serde_json::from_slice(&data) {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!(error = %e, "bad JSON on strategy endpoint");
                return;
            }
        };

        let action = msg
            .get("action")
            .and_then(|v| v.as_str())
            .unwrap_or("start");

        match action {
            "stop" => {
                let active = self.active.lock().await;
                if active.as_ref().map(|a| a.name.as_str()) == Some(name) {
                    drop(active);
                    tracing::info!(strategy = name, "stopping");
                    self.cancel_active().await;
                }
            }
            "start" => {
                // Cancel previous
                self.cancel_active().await;

                // Find strategy index
                let idx = self.strategies.iter().position(|s| s.name() == name);
                let idx = match idx {
                    Some(i) => i,
                    None => {
                        tracing::error!(strategy = name, "not found");
                        return;
                    }
                };

                let params = msg.get("params").cloned().unwrap_or(Value::Object(Default::default()));
                let cancel = CancellationToken::new();
                let ctx = StrategyContext {
                    cmd_ctx: self.cmd_ctx.clone(),
                    params,
                    cancel: cancel.clone(),
                };

                let strat = self.strategies[idx].clone();
                let strategy_name = name.to_string();
                let cmd_ctx = self.cmd_ctx.clone();
                let active = self.active.clone();

                let handle = tokio::spawn(async move {
                    tracing::info!(strategy = strat.name(), "started");
                    match strat.run(ctx).await {
                        Ok(()) => tracing::info!(strategy = strat.name(), "completed"),
                        Err(e) if e.is::<Cancelled>() => {
                            tracing::info!(strategy = strat.name(), "cancelled")
                        }
                        Err(e) => tracing::error!(strategy = strat.name(), error = %e, "crashed"),
                    }

                    // Zero drives on exit
                    let _ = DriveCommand {
                        left: 0.0,
                        right: 0.0,
                    }
                    .execute(&cmd_ctx)
                    .await;

                    // Clear active
                    let sname = strat.name().to_string();
                    let mut guard = active.lock().await;
                    if guard.as_ref().map(|a| a.name.as_str()) == Some(sname.as_str()) {
                        *guard = None;
                    }
                });

                *self.active.lock().await = Some(ActiveStrategy {
                    name: strategy_name,
                    cancel,
                    handle,
                });
            }
            other => {
                tracing::warn!(action = other, "unknown strategy action");
            }
        }
    }

    // ── Helpers ─────────────────────────────────────────────────────────

    async fn cancel_active(&self) {
        let mut guard = self.active.lock().await;
        if let Some(active) = guard.take() {
            active.cancel.cancel();
            let _ = active.handle.await;
        }
    }
}

// ── Control packet parser ───────────────────────────────────────────────────

fn parse_control(data: &[u8]) -> Option<Box<dyn Command>> {
    // Try JSON first (starts with '{')
    if data.first() == Some(&b'{') {
        tracing::debug!("control: detected JSON");
        return parse_json_control(data);
    }

    // Try protobuf
    match RoveControl::decode(data.as_ref()) {
        Ok(msg) => {
            tracing::debug!(
                has_tracks = msg.tracks.is_some(),
                has_flippers = msg.flippers.is_some(),
                has_ovis = msg.ovis.is_some(),
                timestamp_us = msg.timestamp_us,
                "control: decoded RoveControl proto"
            );
            if let Some(ref tracks) = msg.tracks {
                tracing::info!(
                    left_vel = tracks.left_vel,
                    right_vel = tracks.right_vel,
                    "control: tracks command"
                );
                return Some(Box::new(NormalizedDriveCommand {
                    left_norm: tracks.left_vel,
                    right_norm: tracks.right_vel,
                }));
            }
        }
        Err(e) => {
            tracing::debug!(error = %e, len = data.len(), "control: protobuf decode failed");
        }
    }

    // Fallback: try JSON anyway
    parse_json_control(data)
}

fn parse_json_control(data: &[u8]) -> Option<Box<dyn Command>> {
    let obj: Value = serde_json::from_slice(data).ok()?;

    // Normalized (-1..1)
    if let (Some(l), Some(r)) = (
        obj.get("left_vel").and_then(|v| v.as_f64()),
        obj.get("right_vel").and_then(|v| v.as_f64()),
    ) {
        return Some(Box::new(NormalizedDriveCommand {
            left_norm: l as f32,
            right_norm: r as f32,
        }));
    }

    // Absolute velocity
    if let (Some(l), Some(r)) = (
        obj.get("left").and_then(|v| v.as_f64()),
        obj.get("right").and_then(|v| v.as_f64()),
    ) {
        return Some(Box::new(DriveCommand {
            left: l as f32,
            right: r as f32,
        }));
    }

    tracing::warn!("unrecognised control payload");
    None
}

// ── UDP receive loop ────────────────────────────────────────────────────────

async fn recv_loop<F, Fut>(sock: UdpSocket, handler: F)
where
    F: Fn(Vec<u8>, std::net::SocketAddr) -> Fut + Send + 'static,
    Fut: std::future::Future<Output = ()> + Send,
{
    let mut buf = [0u8; 4096];
    loop {
        match sock.recv_from(&mut buf).await {
            Ok((len, addr)) => {
                handler(buf[..len].to_vec(), addr).await;
            }
            Err(e) => {
                tracing::warn!(error = %e, "UDP recv error");
            }
        }
    }
}
