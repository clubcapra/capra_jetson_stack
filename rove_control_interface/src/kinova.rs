//! Talks to the Kinova arm via the rove_sensor_api UDP protocol.
//!
//! Wire format is a 4-byte header (version, msg_type, u16 LE seq) followed
//! by a JSON payload. We Subscribe on the data port and stream Command
//! packets to the command port.

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, Result};
use serde_json::Value;
use tokio::net::UdpSocket;
use tokio::sync::watch;
use tokio::time::interval;
use tracing::{debug, info, warn};

use crate::config::KinovaCfg;
use crate::state::ArmSnapshot;

const PROTO_VERSION: u8 = 1;
const MSG_SUBSCRIBE: u8 = 0x01;
const MSG_DATA: u8 = 0x03;
const MSG_SUB_ACK: u8 = 0x04;
const MSG_COMMAND: u8 = 0x10;
const MSG_CMD_ACK: u8 = 0x11;
const MSG_ERROR: u8 = 0xFF;

fn frame(msg_type: u8, seq: u16, payload: &[u8]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + payload.len());
    buf.push(PROTO_VERSION);
    buf.push(msg_type);
    buf.extend_from_slice(&seq.to_le_bytes());
    buf.extend_from_slice(payload);
    buf
}

fn parse(buf: &[u8]) -> Option<(u8, u16, &[u8])> {
    if buf.len() < 4 || buf[0] != PROTO_VERSION {
        return None;
    }
    let seq = u16::from_le_bytes([buf[2], buf[3]]);
    Some((buf[1], seq, &buf[4..]))
}

/// Spawn the data subscriber loop. Updates `snapshot_tx` on each Data frame.
pub async fn run_data_subscriber(
    cfg: &KinovaCfg,
    snapshot_tx: watch::Sender<ArmSnapshot>,
) -> Result<()> {
    let api_addr: SocketAddr = format!("{}:{}", cfg.api_host, cfg.data_port).parse()?;
    let sock = UdpSocket::bind("0.0.0.0:0").await?;
    sock.connect(api_addr).await?;
    let seq = AtomicU16::new(1);

    // Keep re-subscribing periodically — the api drops subscribers it can't
    // reach for too long, and our client port isn't pinned across restarts.
    let mut resub = interval(Duration::from_secs(5));
    let payload = serde_json::to_vec(&serde_json::json!({
        "interval_ms": cfg.subscribe_interval_ms,
    }))?;

    let send_subscribe = || -> Vec<u8> {
        let s = seq.fetch_add(1, Ordering::Relaxed);
        frame(MSG_SUBSCRIBE, s, &payload)
    };
    sock.send(&send_subscribe()).await?;
    info!(target: "kinova", "subscribed to {} every {} ms", api_addr, cfg.subscribe_interval_ms);

    let mut rx_buf = vec![0u8; 4096];
    loop {
        tokio::select! {
            _ = resub.tick() => {
                if let Err(e) = sock.send(&send_subscribe()).await {
                    warn!(target: "kinova", "re-subscribe send failed: {e}");
                }
            }
            r = sock.recv(&mut rx_buf) => {
                match r {
                    Ok(n) => handle_packet(&rx_buf[..n], &snapshot_tx),
                    Err(e) => {
                        warn!(target: "kinova", "recv failed: {e}");
                        tokio::time::sleep(Duration::from_millis(200)).await;
                    }
                }
            }
        }
    }
}

fn handle_packet(buf: &[u8], snapshot_tx: &watch::Sender<ArmSnapshot>) {
    let Some((msg_type, _seq, payload)) = parse(buf) else {
        debug!(target: "kinova", "drop {} byte non-protocol packet", buf.len());
        return;
    };
    match msg_type {
        MSG_DATA => match serde_json::from_slice::<Value>(payload) {
            Ok(v) => {
                if let Some(snap) = ArmSnapshot::from_kinova_json(&v) {
                    let _ = snapshot_tx.send(snap);
                } else {
                    debug!(target: "kinova", "data missing joint fields: {v}");
                }
            }
            Err(e) => warn!(target: "kinova", "data json parse failed: {e}"),
        },
        MSG_SUB_ACK => debug!(target: "kinova", "subscribe ack"),
        MSG_ERROR => warn!(target: "kinova", "api error: {}", String::from_utf8_lossy(payload)),
        other => debug!(target: "kinova", "unexpected msg_type 0x{other:02x}"),
    }
}

/// Sender for command packets. Cheap to clone.
#[derive(Clone)]
pub struct CommandSender {
    sock: Arc<UdpSocket>,
    seq: Arc<AtomicU16>,
}

impl CommandSender {
    pub async fn connect(cfg: &KinovaCfg) -> Result<Self> {
        let api_addr: SocketAddr = format!("{}:{}", cfg.api_host, cfg.command_port)
            .parse()
            .map_err(|e| anyhow!("bad api addr: {e}"))?;
        let sock = UdpSocket::bind("0.0.0.0:0").await?;
        sock.connect(api_addr).await?;
        Ok(Self {
            sock: Arc::new(sock),
            seq: Arc::new(AtomicU16::new(1)),
        })
    }

    pub async fn send_velocity(&self, vel_deg_s: [f32; 6]) -> Result<()> {
        let mut payload = serde_json::Map::with_capacity(6);
        for (i, v) in vel_deg_s.iter().enumerate() {
            payload.insert(format!("joint_{}_vel", i + 1), serde_json::json!(v));
        }
        self.send_json(&Value::Object(payload)).await
    }

    pub async fn send_json(&self, value: &Value) -> Result<()> {
        let payload = serde_json::to_vec(value)?;
        let s = self.seq.fetch_add(1, Ordering::Relaxed);
        let frame = frame(MSG_COMMAND, s, &payload);
        self.sock.send(&frame).await?;
        Ok(())
    }

    pub async fn drain_acks(&self) {
        // Reads and discards CommandAck frames so the OS rx buffer doesn't
        // fill. Run this as a background task.
        let mut buf = vec![0u8; 1024];
        loop {
            match self.sock.recv(&mut buf).await {
                Ok(n) => {
                    if let Some((mt, _seq, p)) = parse(&buf[..n]) {
                        if mt == MSG_ERROR {
                            warn!(target: "kinova", "cmd error: {}", String::from_utf8_lossy(p));
                        } else if mt != MSG_CMD_ACK {
                            debug!(target: "kinova", "cmd port got msg_type 0x{mt:02x}");
                        }
                    }
                }
                Err(e) => {
                    warn!(target: "kinova", "cmd recv: {e}");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }
    }
}
