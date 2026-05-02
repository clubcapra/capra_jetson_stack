//! End-to-end test for the multiplexer against RoveControl messages.
//!
//! Each sender emits a fully-populated payload where **every field** derives
//! deterministically from its priority number. This means:
//!   - Packets from prio=1 and prio=2 differ in every field, not just a marker.
//!   - The decoder reconstructs the expected payload from any candidate
//!     priority and checks all fields match. If the multiplexer ever forwarded
//!     mixed bytes, a partial packet, or a packet from the wrong input, the
//!     decoder would reject it.
//!
//! Run with: `cargo run --bin test_rove_control`

use std::env;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use prost::Message;

use udp_multiplexer::proto::telemetry::{
    Flippers, JointState, Ovis, RoveControl, Tracks,
};
use udp_multiplexer::test_support::run_suite;

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

/// Derive a realistic-looking float value from the priority and a field index.
/// The formula is deterministic and distinct per priority, so every field
/// becomes a per-input signature.
fn field(priority: u16, field_id: u16) -> f32 {
    // Keep numbers in plausible ranges while making them unique per priority.
    // priority=1, field_id=0 -> 0.10; priority=2, field_id=0 -> 0.20, etc.
    // The 0.001 * field_id spread ensures every field is different from the others.
    0.1 * priority as f32 + 0.001 * field_id as f32
}

fn joint_from(priority: u16, base_id: u16) -> JointState {
    JointState {
        vel: field(priority, base_id),
        pos_deg: field(priority, base_id + 1) * 100.0, // scale to degrees-ish
        torque: field(priority, base_id + 2) * 10.0,
    }
}

/// Build the full expected payload for a given priority.
/// Both the sender and the verifier call this — the decoder rebuilds the
/// expected struct for each candidate priority and compares.
fn build_expected(priority: u16) -> RoveControl {
    RoveControl {
        tracks: Some(Tracks {
            left_vel: field(priority, 0),
            right_vel: field(priority, 1),
        }),
        flippers: Some(Flippers {
            fl: Some(joint_from(priority, 10)),
            fr: Some(joint_from(priority, 20)),
            rl: Some(joint_from(priority, 30)),
            rr: Some(joint_from(priority, 40)),
        }),
        ovis: Some(Ovis {
            act_1: Some(joint_from(priority, 50)),
            act_2: Some(joint_from(priority, 60)),
            act_3: Some(joint_from(priority, 70)),
            act_4: Some(joint_from(priority, 80)),
            act_5: Some(joint_from(priority, 90)),
            act_6: Some(joint_from(priority, 100)),
        }),
        // timestamp_us is the one field that legitimately varies per packet
        timestamp_us: 0,
    }
}

fn build_packet_for(priority: u16) -> Vec<u8> {
    let mut msg = build_expected(priority);
    msg.timestamp_us = now_us();
    msg.encode_to_vec()
}

/// Decode a packet and figure out which priority sent it by reconstructing
/// the expected payload for each candidate and checking all fields match
/// except timestamp_us.
///
/// Returns None if the packet doesn't match any known sender — meaning the
/// bytes were corrupted, truncated, or came from an unexpected source.
fn identify_sender(bytes: &[u8]) -> Option<u16> {
    let received = RoveControl::decode(bytes).ok()?;

    // Try each candidate priority. The test only uses 1 and 2, but the loop
    // trivially extends if you add more inputs to the config.
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
        .unwrap_or_else(|| "test_rove_control.toml".to_string());

    let build_packet = Arc::new(|priority: u16, _left: f32, _right: f32| -> Vec<u8> {
        build_packet_for(priority)
    });

    let decode_priority = Arc::new(|bytes: &[u8]| -> Option<u16> { identify_sender(bytes) });

    let code = run_suite(
        "RoveControl",
        &config_path,
        build_packet,
        decode_priority,
    );
    std::process::exit(code);
}