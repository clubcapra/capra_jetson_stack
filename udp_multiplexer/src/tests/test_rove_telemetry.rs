//! End-to-end test for the multiplexer against RoveTelemetry messages.
//!
//! Every field of the payload is a deterministic function of the sender's
//! priority. The decoder reconstructs the expected payload for each candidate
//! priority and compares; a match identifies the sender with high confidence,
//! and any partial/mixed/cross-contaminated packet fails to match any
//! candidate.
//!
//! NOTE: DriveNodeState in this schema carries only `motor_pos` (a single
//! float position in degrees), not a full JointState. Telemetry is
//! position-only — control keeps the full vel/pos/torque triplet elsewhere.
//!
//! Run with: `cargo run --bin test_rove_telemetry`

use std::env;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use prost::Message;

use udp_multiplexer::proto::telemetry::{
    Battery, DriveNodeState, Icm40609, OdrivesTelemetry, Orientation, OvisTelemetry,
    Position, RoveTelemetry, Vector3, Vn300,
};
use udp_multiplexer::test_support::run_suite;

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

fn field(priority: u16, field_id: u16) -> f32 {
    0.1 * priority as f32 + 0.001 * field_id as f32
}

fn field_double(priority: u16, field_id: u16) -> f64 {
    0.1 * priority as f64 + 0.001 * field_id as f64
}

/// Build a DriveNodeState where every field is derived from priority.
/// `node_id` identifies the node's position in the telemetry struct;
/// `base` offsets all other fields so each node has unique numeric values.
fn drive_node(priority: u16, node_id: u32, base: u16) -> DriveNodeState {
    DriveNodeState {
        node_id,
        node_state: 7 + priority as u32,
        node_temp_c: 40.0 + field(priority, base),
        motor_temp_c: 48.0 + field(priority, base + 1),
        motor_amp: 8.0 + field(priority, base + 2),
        active_errors: 0,
        latched_errors: 0,
        motor_pos: field(priority, base + 3) * 100.0, // scale into degrees-ish
    }
}

fn vec3(priority: u16, base: u16) -> Vector3 {
    Vector3 {
        x: field(priority, base),
        y: field(priority, base + 1),
        z: field(priority, base + 2),
    }
}

/// Build the full expected payload for a given priority.
fn build_expected(priority: u16) -> RoveTelemetry {
    RoveTelemetry {
        odrives: Some(OdrivesTelemetry {
            node_1: Some(drive_node(priority, 1, 100)),
            node_2: Some(drive_node(priority, 2, 110)),
            node_3: Some(drive_node(priority, 3, 120)),
            node_4: Some(drive_node(priority, 4, 130)),
            node_5: Some(drive_node(priority, 5, 140)),
            node_6: Some(drive_node(priority, 6, 150)),
            node_7: Some(drive_node(priority, 7, 160)),
            node_8: Some(drive_node(priority, 8, 170)),
        }),
        ovis: Some(OvisTelemetry {
            act_1: Some(drive_node(priority, 11, 200)),
            act_2: Some(drive_node(priority, 12, 210)),
            act_3: Some(drive_node(priority, 13, 220)),
            act_4: Some(drive_node(priority, 14, 230)),
            act_5: Some(drive_node(priority, 15, 240)),
            act_6: Some(drive_node(priority, 16, 250)),
        }),
        vn300: Some(Vn300 {
            position: Some(Position {
                lat: 45.5 + field_double(priority, 300),
                lon: -73.5 - field_double(priority, 301),
                alt: 50.0 + field(priority, 302),
            }),
            orientation: Some(Orientation {
                yaw: 85.0 + field(priority, 310),
                pitch: field(priority, 311),
                roll: field(priority, 312),
            }),
            velocity: Some(vec3(priority, 320)),
            accel: Some(Vector3 {
                x: field(priority, 330),
                y: field(priority, 331),
                z: 9.78 + field(priority, 332),
            }),
            gyro: Some(vec3(priority, 340)),
        }),
        livox_1_icm40609: Some(Icm40609 {
            accel: Some(Vector3 {
                x: field(priority, 400),
                y: field(priority, 401),
                z: 9.80 + field(priority, 402),
            }),
            gyro: Some(vec3(priority, 410)),
            temp_c: 41.0 + field(priority, 420),
        }),
        livox_2_icm40609: Some(Icm40609 {
            accel: Some(Vector3 {
                x: field(priority, 500),
                y: field(priority, 501),
                z: 9.79 + field(priority, 502),
            }),
            gyro: Some(vec3(priority, 510)),
            temp_c: 41.0 + field(priority, 520),
        }),
        battery: Some(Battery {
            volt: 24.0 + field(priority, 600),
            amp: 12.0 + field(priority, 601),
            temp_c: 28.0 + field(priority, 602),
        }),
        timestamp_us: 0,
    }
}

fn build_packet_for(priority: u16) -> Vec<u8> {
    let mut msg = build_expected(priority);
    msg.timestamp_us = now_us();
    msg.encode_to_vec()
}

fn identify_sender(bytes: &[u8]) -> Option<u16> {
    let received = RoveTelemetry::decode(bytes).ok()?;
    for candidate in 1..=20u16 {
        let mut expected = build_expected(candidate);
        expected.timestamp_us = received.timestamp_us;
        if expected == received {
            return Some(candidate);
        }
    }
    None
}

fn main() {
    let config_path = env::args()
        .nth(1)
        .unwrap_or_else(|| "test_rove_telemetry.toml".to_string());

    let build_packet = Arc::new(|priority: u16, _left: f32, _right: f32| -> Vec<u8> {
        build_packet_for(priority)
    });

    let decode_priority = Arc::new(|bytes: &[u8]| -> Option<u16> { identify_sender(bytes) });

    let code = run_suite(
        "RoveTelemetry",
        &config_path,
        build_packet,
        decode_priority,
    );
    std::process::exit(code);
}