//! Teleop UDP link: receive RoveControl, push RoveTelemetry.
//!
//! The telemetry destination address is locked to whichever peer most
//! recently sent us a RoveControl packet — the steam deck routes UDP via
//! whatever interface it likes, so reflecting `recv_from`'s source IP is
//! more robust than hard-coding one.

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use tokio::net::UdpSocket;
use tokio::sync::watch;
use tracing::{info, warn};

const TWIST_SUMMARY_INTERVAL: Duration = Duration::from_secs(1);

use crate::config::TeleopCfg;
use crate::ik_link::TwistSender;
use crate::proto::{self, DriveNodeState, OvisTelemetry, RoveTelemetry};
use crate::state::ArmSnapshot;

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

#[derive(Default)]
pub struct PeerTracker {
    last: std::sync::Mutex<Option<SocketAddr>>,
}

impl PeerTracker {
    pub fn update(&self, addr: SocketAddr) {
        let mut g = self.last.lock().unwrap();
        *g = Some(addr);
    }
    pub fn get(&self) -> Option<SocketAddr> {
        *self.last.lock().unwrap()
    }
}

/// Minimum change in any twist / tracks component before we re-emit the
/// "twist rx" line. Otherwise a 50 Hz stream of nearly-identical packets
/// floods the log with no useful signal.
const TWIST_LOG_EPSILON: f32 = 0.02;
/// Cap on how long we'll suppress identical-looking packets — even if
/// nothing moved, we still want a line every ~2 s confirming the link.
const TWIST_LOG_MAX_GAP: Duration = Duration::from_secs(2);

#[derive(Default, Clone, Copy)]
struct TwistSnapshot {
    px: f32, py: f32, pz: f32,
    oy: f32, op: f32, or: f32,
    tl: f32, tr: f32,
    fl: i32, fr: i32, rl: i32, rr: i32,
    grip: bool,
}

impl TwistSnapshot {
    fn from(rc: &proto::RoveControl) -> Self {
        Self {
            px: rc.ovis.position.x, py: rc.ovis.position.y, pz: rc.ovis.position.z,
            oy: rc.ovis.orientation.yaw, op: rc.ovis.orientation.pitch, or: rc.ovis.orientation.roll,
            tl: rc.tracks.left_vel, tr: rc.tracks.right_vel,
            fl: rc.flippers.fl, fr: rc.flippers.fr, rl: rc.flippers.rl, rr: rc.flippers.rr,
            grip: rc.gripper.open_state,
        }
    }

    fn meaningfully_different(&self, other: &Self) -> bool {
        let d = |a: f32, b: f32| (a - b).abs() > TWIST_LOG_EPSILON;
        d(self.px, other.px) || d(self.py, other.py) || d(self.pz, other.pz)
            || d(self.oy, other.oy) || d(self.op, other.op) || d(self.or, other.or)
            || d(self.tl, other.tl) || d(self.tr, other.tr)
            || self.fl != other.fl || self.fr != other.fr
            || self.rl != other.rl || self.rr != other.rr
            || self.grip != other.grip
    }
}

pub async fn run_control_listener(
    cfg: &TeleopCfg,
    twist: Arc<TwistSender>,
    peer: Arc<PeerTracker>,
) -> Result<()> {
    let bind: SocketAddr = format!("{}:{}", cfg.listen_host, cfg.listen_port).parse()?;
    let sock = UdpSocket::bind(bind).await?;
    info!(target: "teleop", "RoveControl listener bound at {bind}");
    let mut buf = vec![0u8; 4096];
    let mut received: u64 = 0;
    let mut last_summary = Instant::now() - TWIST_SUMMARY_INTERVAL;
    let mut rx_since_summary: u32 = 0;
    let mut peak_lin: f32 = 0.0;
    let mut peak_ang: f32 = 0.0;
    let mut last_peer: Option<SocketAddr> = None;
    let mut last_logged_snapshot: Option<TwistSnapshot> = None;
    let mut last_logged_at = Instant::now() - TWIST_LOG_MAX_GAP;

    loop {
        let (n, from) = sock.recv_from(&mut buf).await?;
        let rc = match proto::decode_rove_control(&buf[..n]) {
            Ok(rc) => rc,
            Err(e) => {
                warn!(target: "teleop", "RoveControl decode failed ({e}); {n} byte frame from {from}");
                continue;
            }
        };
        peer.update(from);
        received += 1;
        rx_since_summary += 1;

        let p = &rc.ovis.position;
        let o = &rc.ovis.orientation;
        let lin_mag = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
        let ang_mag = (o.yaw * o.yaw + o.pitch * o.pitch + o.roll * o.roll).sqrt();
        if lin_mag > peak_lin {
            peak_lin = lin_mag;
        }
        if ang_mag > peak_ang {
            peak_ang = ang_mag;
        }

        // Log "twist rx" only when the command actually changes enough to
        // matter (per-component delta > epsilon), or when more than the
        // max-gap has elapsed so the operator knows the link is still
        // live. This turns a 50 Hz wall of near-identical lines into one
        // line per real user gesture.
        let snap = TwistSnapshot::from(&rc);
        let any_motion = lin_mag > 1e-3
            || ang_mag > 1e-3
            || snap.tl.abs() > 1e-3
            || snap.tr.abs() > 1e-3
            || snap.fl != 0 || snap.fr != 0 || snap.rl != 0 || snap.rr != 0
            || snap.grip;
        let changed = last_logged_snapshot
            .as_ref()
            .map(|prev| snap.meaningfully_different(prev))
            .unwrap_or(true);
        let stale = last_logged_at.elapsed() >= TWIST_LOG_MAX_GAP;
        if (any_motion && changed) || stale {
            info!(
                target: "teleop",
                "twist rx: pos=({:+.3},{:+.3},{:+.3}) ori=({:+.3},{:+.3},{:+.3}) | tracks L={:+.2} R={:+.2} | flips fl={} fr={} rl={} rr={} grip={}",
                snap.px, snap.py, snap.pz, snap.oy, snap.op, snap.or,
                snap.tl, snap.tr,
                snap.fl, snap.fr, snap.rl, snap.rr,
                if snap.grip { "OPEN" } else { "CLOSED" },
            );
            last_logged_snapshot = Some(snap);
            last_logged_at = Instant::now();
        }

        if last_summary.elapsed() >= TWIST_SUMMARY_INTERVAL {
            info!(
                target: "teleop",
                "RoveControl rx: {} pkts/s from {} (total {}) | peak twist lin={:.3} ang={:.3}",
                rx_since_summary,
                last_peer.map(|a| a.to_string()).unwrap_or_else(|| from.to_string()),
                received, peak_lin, peak_ang,
            );
            last_summary = Instant::now();
            rx_since_summary = 0;
            peak_lin = 0.0;
            peak_ang = 0.0;
        }
        last_peer = Some(from);

        // Always forward an Ovis Twist — even at zero — so the engine emits
        // a JointCommand the kinova pusher can read each frame. (For
        // RESOLVED_RATE, an all-zero twist yields all-zero q_dot.)
        if let Err(e) = twist.send(&rc.ovis).await {
            warn!(target: "ik", "twist forward: {e}");
        }
    }
}

pub async fn run_telemetry_pusher(
    cfg: &TeleopCfg,
    snapshot_rx: watch::Receiver<ArmSnapshot>,
    peer: Arc<PeerTracker>,
) -> Result<()> {
    let sock = Arc::new(UdpSocket::bind("0.0.0.0:0").await?);
    let static_dst: Option<SocketAddr> = if cfg.telemetry_host.trim().is_empty() {
        None
    } else {
        Some(format!("{}:{}", cfg.telemetry_host, cfg.telemetry_port).parse()?)
    };
    let period = Duration::from_secs_f64((1.0 / cfg.telemetry_rate_hz).max(0.005));
    let mut tick = tokio::time::interval(period);
    info!(target: "teleop", "telemetry pusher @ {:.1} Hz", cfg.telemetry_rate_hz);

    loop {
        tick.tick().await;
        let snap = snapshot_rx.borrow().clone();
        let dst = if let Some(d) = static_dst {
            Some(d)
        } else {
            peer.get().map(|p| SocketAddr::new(p.ip(), cfg.telemetry_port))
        };
        let Some(dst) = dst else { continue };
        let telemetry = build_telemetry(&snap);
        let bytes = proto::encode_rove_telemetry(&telemetry);
        if let Err(e) = sock.send_to(&bytes, dst).await {
            warn!(target: "teleop", "telemetry send to {dst}: {e}");
        }
    }
}

fn build_telemetry(snap: &ArmSnapshot) -> RoveTelemetry {
    let act = |idx: usize| DriveNodeState {
        node_id: (idx + 1) as u32,
        node_state: 1, // 1 == operational; arm api doesn't expose a richer state right now
        node_temp_c: snap.joint_temp_c[idx],
        motor_temp_c: snap.joint_temp_c[idx],
        motor_amp: snap.joint_current_a[idx],
        motor_pos: snap.joint_pos_deg[idx],
        active_errors: 0,
        latched_errors: 0,
    };
    RoveTelemetry {
        ovis: OvisTelemetry {
            act_1: act(0),
            act_2: act(1),
            act_3: act(2),
            act_4: act(3),
            act_5: act(4),
            act_6: act(5),
        },
        timestamp_us: now_us(),
        machine_state: 0,
    }
}
