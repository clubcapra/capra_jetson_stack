/// Periodically polls the Rust API for every ODrive node and exposes the
/// latest snapshot to the rest of the command router.
use std::collections::HashMap;
use std::sync::{Arc, RwLock};
use std::time::Duration;

use serde_json::Value;

use crate::api_client::ApiClient;

pub struct RobotState {
    api: Arc<ApiClient>,
    data: RwLock<HashMap<u8, Value>>,
    interval: Duration,
}

impl RobotState {
    pub fn new(api: Arc<ApiClient>, interval: Duration) -> Self {
        Self {
            api,
            data: RwLock::new(HashMap::new()),
            interval,
        }
    }

    /// Latest telemetry for a single node.
    pub fn node(&self, node_id: u8) -> Option<Value> {
        self.data.read().unwrap().get(&node_id).cloned()
    }

    /// Snapshot of all nodes.
    pub fn all(&self) -> HashMap<u8, Value> {
        self.data.read().unwrap().clone()
    }

    /// Background polling task — runs forever.
    pub async fn run(&self) {
        tracing::info!("state poller started ({} ms)", self.interval.as_millis());
        loop {
            self.poll_once().await;
            tokio::time::sleep(self.interval).await;
        }
    }

    async fn poll_once(&self) {
        let ids = self.api.node_ids();
        let mut snapshot = HashMap::new();
        for nid in ids {
            match self.api.get_data(nid).await {
                Ok(v) if !v.is_null() => {
                    snapshot.insert(nid, v);
                }
                _ => {}
            }
        }
        if !snapshot.is_empty() {
            *self.data.write().unwrap() = snapshot;
        }
    }
}
