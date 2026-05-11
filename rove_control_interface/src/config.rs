use std::path::Path;

use anyhow::Result;
use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct Config {
    pub teleop: TeleopCfg,
    pub ik_engine: IkEngineCfg,
    pub kinova: KinovaCfg,
    pub joint_map: Vec<JointMapEntry>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct TeleopCfg {
    pub listen_host: String,
    pub listen_port: u16,
    pub telemetry_host: String,
    pub telemetry_port: u16,
    pub telemetry_rate_hz: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct IkEngineCfg {
    pub host: String,
    pub joints_port: u16,
    pub twist_port: u16,
    pub velocs_listen_host: String,
    pub velocs_listen_port: u16,
    pub mode: String,
}

#[derive(Debug, Deserialize, Clone)]
pub struct KinovaCfg {
    pub api_host: String,
    pub data_port: u16,
    pub command_port: u16,
    pub subscribe_interval_ms: u32,
    pub command_rate_hz: f64,
}

#[derive(Debug, Deserialize, Clone)]
pub struct JointMapEntry {
    pub ik_id: String,
    pub kinova_idx: u8,
    // The kinova-reported angle (deg) when the arm is at its physical
    // home. The wrapper subtracts this from every JointState it pushes
    // to the IK engine so the engine's q-frame is centred on the URDF
    // home — keeps FK and collisions sane.
    pub home_deg: f32,
    // Flip the rotation direction between the kinova and the IK chain.
    // Applied symmetrically: negates both the sensor→engine joint
    // position and the engine→kinova joint velocity, so a joint whose
    // URDF axis points opposite to the firmware's reports correctly.
    #[serde(default)]
    pub invert: bool,
}

impl Config {
    pub fn load(path: &Path) -> Result<Self> {
        let txt = std::fs::read_to_string(path)?;
        Ok(toml::from_str(&txt)?)
    }
}
