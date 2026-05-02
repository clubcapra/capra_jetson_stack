/// Thin async client for the capra-rove-interface Rust API.
///
/// HTTP (via `ureq` in `spawn_blocking`) for discovery / estop / state.
/// Raw UDP for streaming drive commands — same wire format as test_tracks.py.
use std::collections::HashMap;
use std::net::UdpSocket;
use std::sync::Mutex;

use anyhow::{anyhow, Result};
use serde_json::Value;

const PROTO_VERSION: u8 = 0x01;
const MSG_COMMAND: u8 = 0x10;

#[derive(Debug, Clone)]
pub struct NodeInfo {
    pub node_id: u8,
    pub command_port: u16,
    pub data_port: u16,
}

pub struct ApiClient {
    host: String,
    base_url: String,
    udp_sock: UdpSocket,
    seqs: Mutex<HashMap<u8, u16>>,
    nodes: Mutex<HashMap<u8, NodeInfo>>,
}

impl ApiClient {
    pub fn new(host: &str, http_port: u16) -> Result<Self> {
        let udp_sock = UdpSocket::bind("0.0.0.0:0")?;
        udp_sock.set_nonblocking(false)?;
        Ok(Self {
            host: host.to_string(),
            base_url: format!("http://{}:{}", host, http_port),
            udp_sock,
            seqs: Mutex::new(HashMap::new()),
            nodes: Mutex::new(HashMap::new()),
        })
    }

    // ── Discovery ───────────────────────────────────────────────────────

    pub async fn discover(&self) -> Result<HashMap<u8, NodeInfo>> {
        let data = self.http_get("/discover").await?;
        let sensors = data
            .get("sensors")
            .and_then(|s| s.as_array())
            .ok_or_else(|| anyhow!("bad discover response"))?;

        let mut nodes = HashMap::new();
        for s in sensors {
            let sid = s.get("id").and_then(|v| v.as_str()).unwrap_or("");
            if let Some(nid_str) = sid.strip_prefix("odrive_") {
                if let Ok(nid) = nid_str.parse::<u8>() {
                    let cmd_port = s["command_port"].as_u64().unwrap_or(0) as u16;
                    let data_port = s["data_port"].as_u64().unwrap_or(0) as u16;
                    nodes.insert(
                        nid,
                        NodeInfo {
                            node_id: nid,
                            command_port: cmd_port,
                            data_port,
                        },
                    );
                }
            }
        }

        *self.nodes.lock().unwrap() = nodes.clone();
        Ok(nodes)
    }

    pub fn node_ids(&self) -> Vec<u8> {
        self.nodes.lock().unwrap().keys().copied().collect()
    }

    // ── Telemetry ───────────────────────────────────────────────────────

    pub async fn get_data(&self, node_id: u8) -> Result<Value> {
        self.http_get(&format!("/odrive_{}/data", node_id)).await
    }

    // ── Drive command (UDP) ─────────────────────────────────────────────

    pub fn send_drive_udp(&self, node_id: u8, cmd: &Value) {
        let port = match self.nodes.lock().unwrap().get(&node_id) {
            Some(info) => info.command_port,
            None => return,
        };

        let mut seqs = self.seqs.lock().unwrap();
        let seq = seqs.entry(node_id).or_insert(0);

        let payload = serde_json::to_vec(cmd).unwrap_or_default();
        let mut buf = Vec::with_capacity(4 + payload.len());
        buf.push(PROTO_VERSION);
        buf.push(MSG_COMMAND);
        buf.extend_from_slice(&seq.to_le_bytes());
        buf.extend_from_slice(&payload);

        let addr = format!("{}:{}", self.host, port);
        let _ = self.udp_sock.send_to(&buf, &addr);
        *seq = seq.wrapping_add(1);
    }

    // ── E-Stop (HTTP) ───────────────────────────────────────────────────

    pub async fn estop(&self, node_id: u8) -> Result<()> {
        self.http_post(&format!("/odrive_{}/estop", node_id), None)
            .await?;
        Ok(())
    }

    pub async fn estop_all(&self) -> Result<()> {
        let ids = self.node_ids();
        for nid in ids {
            if let Err(e) = self.estop(nid).await {
                tracing::warn!(node_id = nid, error = %e, "estop failed");
            }
        }
        Ok(())
    }

    // ── Clear errors (HTTP) ─────────────────────────────────────────────

    pub async fn clear_errors(&self, node_id: u8) -> Result<()> {
        self.http_post(
            &format!("/odrive_{}/command", node_id),
            Some(serde_json::json!({"clear_errors": true})),
        )
        .await?;
        Ok(())
    }

    pub async fn clear_errors_all(&self) -> Result<()> {
        let ids = self.node_ids();
        for nid in ids {
            if let Err(e) = self.clear_errors(nid).await {
                tracing::warn!(node_id = nid, error = %e, "clear_errors failed");
            }
        }
        Ok(())
    }

    // ── HTTP helpers ────────────────────────────────────────────────────

    async fn http_get(&self, path: &str) -> Result<Value> {
        let url = format!("{}{}", self.base_url, path);
        tokio::task::spawn_blocking(move || -> Result<Value> {
            let resp = ureq::get(&url)
                .timeout(std::time::Duration::from_secs(3))
                .call()
                .map_err(|e| anyhow!("GET {}: {}", url, e))?;
            let body = resp.into_string()?;
            Ok(serde_json::from_str(&body)?)
        })
        .await?
    }

    async fn http_post(&self, path: &str, body: Option<Value>) -> Result<Value> {
        let url = format!("{}{}", self.base_url, path);
        tokio::task::spawn_blocking(move || -> Result<Value> {
            let req = ureq::post(&url).timeout(std::time::Duration::from_secs(3));
            let resp = match body {
                Some(val) => req
                    .set("Content-Type", "application/json")
                    .send_string(&serde_json::to_string(&val)?)
                    .map_err(|e| anyhow!("POST {}: {}", url, e))?,
                None => req
                    .send_bytes(&[])
                    .map_err(|e| anyhow!("POST {}: {}", url, e))?,
            };
            let text = resp.into_string()?;
            Ok(serde_json::from_str(&text).unwrap_or(Value::Null))
        })
        .await?
    }
}
