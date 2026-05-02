use anyhow::{Context, Result};
use serde::Deserialize;
use std::path::Path;

/// ODrive control mode for the tracks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DriveMode {
    Velocity,
    Torque,
}

#[derive(Debug, Deserialize)]
pub struct Config {
    pub api: ApiConfig,
    pub endpoints: EndpointConfig,
    pub strategies: StrategyConfig,
    pub robot: RobotConfig,
}

#[derive(Debug, Deserialize)]
pub struct ApiConfig {
    pub host: String,
    pub http_port: u16,
}

#[derive(Debug, Deserialize)]
pub struct EndpointConfig {
    pub estop_port: u16,
    pub control_port: u16,
}

#[derive(Debug, Deserialize)]
pub struct StrategyConfig {
    pub base_port: u16,
}

#[derive(Debug, Deserialize)]
pub struct RobotConfig {
    pub left_nodes: Vec<u8>,
    pub right_nodes: Vec<u8>,
    pub drive_mode: DriveMode,
    pub max_velocity: f32,
    pub max_torque: f32,
    pub command_interval_ms: u64,
}

impl Config {
    pub fn all_nodes(&self) -> Vec<u8> {
        self.robot
            .left_nodes
            .iter()
            .chain(&self.robot.right_nodes)
            .copied()
            .collect()
    }
}

pub fn load(path: &Path) -> Result<Config> {
    let content =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    toml::from_str(&content).with_context(|| format!("parsing {}", path.display()))
}
