use serde::Deserialize;
use std::error::Error;
use std::fs;
use std::net::UdpSocket;
use std::time::Duration;

use crate::models::input::Input;
use crate::models::output::Output;

#[derive(Deserialize)]
struct ConfigTable {
    #[serde(default)]
    config: Vec<GlobalSettings>,
    #[serde(default)]
    inputs: Vec<RawInput>,
    #[serde(default)]
    outputs: Vec<RawOutput>,
}

#[derive(Deserialize)]
struct GlobalSettings {
    path_to_protobuf: String,
    #[serde(default = "default_emit_interval_ms")]
    emit_interval_ms: u64,
    #[serde(default = "default_staleness_ms")]
    staleness_ms: u64,
}

fn default_emit_interval_ms() -> u64 {
    10
}
fn default_staleness_ms() -> u64 {
    500
}

#[derive(Deserialize)]
struct RawInput {
    address: String,
    priority: u16,
    protobuf: String,
}

#[derive(Deserialize)]
struct RawOutput {
    address: String,
    protobuf: String,
}

pub struct AppConfig {
    pub path_to_protobuf: String,
    pub emit_interval: Duration,
    pub staleness: Duration,
    pub inputs: Vec<Input>,
    pub outputs: Vec<Output>,
}

impl AppConfig {
    pub fn new(file_path: &str) -> Result<Self, Box<dyn Error>> {
        let content = fs::read_to_string(file_path)?;
        let raw: ConfigTable = toml::from_str(&content)?;

        let (path_to_protobuf, emit_interval_ms, staleness_ms) = raw
            .config
            .into_iter()
            .next()
            .map(|c| (c.path_to_protobuf, c.emit_interval_ms, c.staleness_ms))
            .unwrap_or_else(|| {
                (
                    String::new(),
                    default_emit_interval_ms(),
                    default_staleness_ms(),
                )
            });

        let mut inputs = Vec::with_capacity(raw.inputs.len());
        for r in raw.inputs {
            inputs.push(bind_input(r)?);
        }
        // Lowest priority number first: priority=1 beats priority=3.
        // Index 0 is therefore the most important input.
        inputs.sort_by(|a, b| a.priority().cmp(&b.priority()));

        let mut outputs = Vec::with_capacity(raw.outputs.len());
        for r in raw.outputs {
            outputs.push(bind_output(r)?);
        }

        Ok(Self {
            path_to_protobuf,
            emit_interval: Duration::from_millis(emit_interval_ms),
            staleness: Duration::from_millis(staleness_ms),
            inputs,
            outputs,
        })
    }
}

fn bind_input(raw: RawInput) -> Result<Input, Box<dyn Error>> {
    let socket = UdpSocket::bind(&raw.address)
        .map_err(|e| format!("failed to bind input {}: {}", raw.address, e))?;
    Ok(Input {
        socket,
        priority: raw.priority,
        protobuf: raw.protobuf,
    })
}

fn bind_output(raw: RawOutput) -> Result<Output, Box<dyn Error>> {
    let socket = UdpSocket::bind("0.0.0.0:0")
        .map_err(|e| format!("failed to bind output ephemeral socket: {}", e))?;
    socket
        .connect(&raw.address)
        .map_err(|e| format!("failed to connect output to {}: {}", raw.address, e))?;
    Ok(Output {
        socket,
        protobuf: raw.protobuf,
    })
}