//! Bridge to rove_mvp_engine.
//!
//! - JointState is pushed at the kinova data cadence so the engine always
//!   has a fresh `q`.
//! - Twists are forwarded synchronously when teleop sends a non-zero Ovis.
//! - JointCommand replies are received on a bound socket and pushed to the
//!   `cmd_tx` channel for the kinova command sender to act on.

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, watch};
use tracing::{debug, info, warn};

/// How often (max) the periodic "what am I commanding" summaries log.
const SUMMARY_INTERVAL: Duration = Duration::from_secs(1);

use crate::config::{IkEngineCfg, JointMapEntry};
use crate::proto::{self, IkMode, JointState, NamedFloat, Orientation, Twist, Vector3};
use crate::state::ArmSnapshot;

const DEG_TO_RAD: f32 = std::f32::consts::PI / 180.0;
const RAD_TO_DEG: f32 = 180.0 / std::f32::consts::PI;

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

pub fn ik_mode(s: &str) -> u32 {
    match s.to_ascii_lowercase().as_str() {
        "position_ik" => IkMode::PositionIk as u32,
        _ => IkMode::ResolvedRate as u32,
    }
}

/// Each frame: one JointState packet to the engine's joints port carrying
/// the latest sensor readings translated into the IK chain's q frame.
pub async fn run_joint_pusher(
    cfg: &IkEngineCfg,
    joint_map: Vec<JointMapEntry>,
    mut snapshot_rx: watch::Receiver<ArmSnapshot>,
) -> Result<()> {
    let dst: SocketAddr = format!("{}:{}", cfg.host, cfg.joints_port).parse()?;
    let sock = UdpSocket::bind("0.0.0.0:0").await?;
    sock.connect(dst).await?;
    info!(target: "ik", "joint pusher → {dst}");

    let mut packets: u64 = 0;
    while snapshot_rx.changed().await.is_ok() {
        let snap = snapshot_rx.borrow_and_update().clone();
        let mut js = JointState {
            joints: Vec::with_capacity(joint_map.len()),
            t_us: now_us(),
        };
        for entry in &joint_map {
            let idx = (entry.kinova_idx as usize).saturating_sub(1);
            if idx >= 6 {
                continue;
            }
            // The IK chain (URDF) is modelled with q=0 at hardware home, so
            // subtract the kinova's reported home angle before sending. This
            // keeps the engine's FK / collision queries in the same frame
            // the URDF was built in — otherwise every solve sees the arm
            // displaced by ~home_rad and collisions don't trip correctly.
            // `invert` flips the rotation direction for joints whose URDF
            // axis points opposite to the kinova firmware's report.
            let sign: f32 = if entry.invert { -1.0 } else { 1.0 };
            let q_deg = sign * (snap.joint_pos_deg[idx] - entry.home_deg);
            let value_rad = q_deg * DEG_TO_RAD;
            js.joints.push(NamedFloat {
                name: entry.ik_id.clone(),
                value: value_rad,
            });
        }
        let bytes = proto::encode_joint_state(&js);
        if let Err(e) = sock.send(&bytes).await {
            warn!(target: "ik", "joint state send: {e}");
        } else {
            packets += 1;
            if matches!(packets, 1 | 100 | 1000 | 10000) {
                debug!(target: "ik", "joint state #{packets} sent");
            }
        }
    }
    Ok(())
}

/// One Twist per teleop frame on a connected UDP socket. The engine replies
/// on the velocs port (handled by `run_velocs_listener`).
pub struct TwistSender {
    sock: Arc<UdpSocket>,
    mode: u32,
}

impl TwistSender {
    pub async fn connect(cfg: &IkEngineCfg) -> Result<Self> {
        let dst: SocketAddr = format!("{}:{}", cfg.host, cfg.twist_port).parse()?;
        let sock = UdpSocket::bind("0.0.0.0:0").await?;
        sock.connect(dst).await?;
        info!(target: "ik", "twist sender → {dst} mode={}", cfg.mode);
        Ok(Self {
            sock: Arc::new(sock),
            mode: ik_mode(&cfg.mode),
        })
    }

    pub async fn send(&self, ovis: &proto::OvisTwist) -> Result<()> {
        let twist = Twist {
            orientation: Orientation {
                yaw: ovis.orientation.yaw,
                pitch: ovis.orientation.pitch,
                roll: ovis.orientation.roll,
            },
            position: Vector3 {
                x: ovis.position.x,
                y: ovis.position.y,
                z: ovis.position.z,
            },
            mode: self.mode,
            t_us: now_us(),
        };
        let bytes = proto::encode_twist(&twist);
        self.sock.send(&bytes).await?;
        Ok(())
    }
}

/// Joint vector emitted by the engine, mapped back into kinova arm space
/// (deg/s for revolute joints). Index 0..5 = kinova joint_1..joint_6;
/// joints not in the IK chain stay at zero.
#[derive(Debug, Clone, Default)]
#[allow(dead_code)]
pub struct VelocsCommand {
    pub vel_deg_s: [f32; 6],
    // mode/residual/converged are surfaced so logs and future failsafe
    // logic have first-class access without re-deriving them from the
    // wire packet.
    pub mode: u32,
    pub residual: f32,
    pub converged: bool,
}

pub async fn run_velocs_listener(
    cfg: &IkEngineCfg,
    joint_map: Vec<JointMapEntry>,
    cmd_tx: mpsc::Sender<VelocsCommand>,
) -> Result<()> {
    let bind: SocketAddr = format!("{}:{}", cfg.velocs_listen_host, cfg.velocs_listen_port).parse()?;
    let sock = UdpSocket::bind(bind).await?;
    info!(target: "ik", "velocs listener bound at {bind}");
    let mut rx = vec![0u8; 8192];

    let mut last_summary = Instant::now() - SUMMARY_INTERVAL;
    let mut frames_since_summary: u32 = 0;
    let mut peak_speed_deg_s: f32 = 0.0;

    loop {
        let (n, _from) = sock.recv_from(&mut rx).await?;
        let cmd = match proto::decode_joint_command(&rx[..n]) {
            Ok(c) => c,
            Err(e) => {
                warn!(target: "ik", "joint command decode failed ({e}); {n} byte frame");
                continue;
            }
        };

        let mut out = VelocsCommand {
            mode: cmd.mode,
            residual: cmd.residual,
            converged: cmd.converged,
            ..Default::default()
        };

        // RESOLVED_RATE → joint velocities directly. POSITION_IK → joint
        // *positions*; treat as a no-op for now (we'd need the latest
        // sensor q to derive a velocity). Log instead of acting.
        if cmd.mode != IkMode::ResolvedRate as u32 {
            debug!(target: "ik", "velocs in POSITION_IK mode — not forwarding velocities (residual={:.4})", cmd.residual);
            continue;
        }

        for nf in &cmd.joints {
            if let Some(entry) = joint_map.iter().find(|e| e.ik_id == nf.name) {
                let idx = (entry.kinova_idx as usize).saturating_sub(1);
                if idx < 6 {
                    let sign: f32 = if entry.invert { -1.0 } else { 1.0 };
                    out.vel_deg_s[idx] = sign * nf.value * RAD_TO_DEG;
                }
            }
        }

        // Track the largest joint speed seen in this summary window so the
        // logged "peak" line tells the operator how aggressive the IK got.
        for v in &out.vel_deg_s {
            let a = v.abs();
            if a > peak_speed_deg_s {
                peak_speed_deg_s = a;
            }
        }
        frames_since_summary += 1;
        if last_summary.elapsed() >= SUMMARY_INTERVAL {
            info!(
                target: "ik",
                "IK out @ {} Hz | J1..J6 = [{:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}] deg/s  peak={:.2}  residual={:.4}",
                frames_since_summary,
                out.vel_deg_s[0], out.vel_deg_s[1], out.vel_deg_s[2],
                out.vel_deg_s[3], out.vel_deg_s[4], out.vel_deg_s[5],
                peak_speed_deg_s, cmd.residual,
            );
            last_summary = Instant::now();
            frames_since_summary = 0;
            peak_speed_deg_s = 0.0;
        }

        // Bounded channel; drop oldest if full so the kinova worker isn't
        // ever waiting on stale commands.
        if cmd_tx.try_send(out).is_err() {
            // Channel full — replace by recv-and-resend pattern is overkill;
            // a single dropped frame is preferable to unbounded queue growth.
            tokio::task::yield_now().await;
        }
    }
}

/// Drive the kinova command sender at a steady cadence, picking up the
/// latest VelocsCommand each tick. If no command has arrived recently the
/// arm is sent zero velocities (matching the kinova driver's "no input =
/// stop" expectation).
pub async fn run_kinova_pusher(
    sender: crate::kinova::CommandSender,
    mut cmd_rx: mpsc::Receiver<VelocsCommand>,
    rate_hz: f64,
) -> Result<()> {
    let period = Duration::from_secs_f64((1.0 / rate_hz).max(0.005));
    let mut tick = tokio::time::interval(period);
    let mut latest = VelocsCommand::default();
    let mut age_ticks: u32 = 0;
    let mut last_summary = Instant::now() - SUMMARY_INTERVAL;
    let mut sent_since_summary: u32 = 0;

    info!(target: "kinova", "command pusher @ {:.1} Hz", rate_hz);
    loop {
        tokio::select! {
            _ = tick.tick() => {
                if age_ticks > (rate_hz as u32).max(1) {
                    if latest.vel_deg_s.iter().any(|v| v.abs() > 0.001) {
                        info!(target: "kinova", "IK silent for >1s — zeroing arm command");
                    }
                    latest = VelocsCommand::default();
                }
                if let Err(e) = sender.send_velocity(latest.vel_deg_s).await {
                    warn!(target: "kinova", "vel command send: {e}");
                } else {
                    sent_since_summary += 1;
                }
                age_ticks = age_ticks.saturating_add(1);

                if last_summary.elapsed() >= SUMMARY_INTERVAL {
                    let active = latest.vel_deg_s.iter().any(|v| v.abs() > 0.001);
                    info!(
                        target: "kinova",
                        "arm cmd ({} pkts/s, {}): J1..J6 = [{:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}, {:+6.2}] deg/s",
                        sent_since_summary,
                        if active { "MOVING" } else { "idle" },
                        latest.vel_deg_s[0], latest.vel_deg_s[1], latest.vel_deg_s[2],
                        latest.vel_deg_s[3], latest.vel_deg_s[4], latest.vel_deg_s[5],
                    );
                    last_summary = Instant::now();
                    sent_since_summary = 0;
                }
            }
            Some(cmd) = cmd_rx.recv() => {
                latest = cmd;
                age_ticks = 0;
            }
        }
    }
}
