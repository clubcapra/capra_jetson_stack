use prost::Message;
use std::collections::HashMap;

use crate::proto::telemetry::{RoveControl, RoveTelemetry};

pub type Validator = fn(&[u8]) -> Option<u64>;

pub fn registry() -> HashMap<&'static str, Validator> {
    let mut m: HashMap<&'static str, Validator> = HashMap::new();
    m.insert("RoveControl.proto", validate_rove_control);
    m.insert("RoveTelemetry.proto", validate_rove_telemetry);
    m
}

fn validate_rove_control(bytes: &[u8]) -> Option<u64> {
    RoveControl::decode(bytes).ok().map(|m| m.timestamp_us)
}

fn validate_rove_telemetry(bytes: &[u8]) -> Option<u64> {
    RoveTelemetry::decode(bytes).ok().map(|m| m.timestamp_us)
}