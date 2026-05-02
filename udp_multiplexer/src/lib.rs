//! Core of the multiplexer, exposed as a library so both the `udp_multiplexer`
//! binary and the test binaries can drive it without duplicating logic.

pub mod helpers {
    pub mod app_config;
    pub mod proto_registry;
}
pub mod models {
    pub mod input;
    pub mod output;
}
pub mod proto;
pub mod test_support;

use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use helpers::app_config::AppConfig;
use helpers::proto_registry::{registry, Validator};
use models::input::Input;
use models::output::Output;

struct Incoming {
    input_index: usize,
    priority: u16,
    timestamp_us: u64,
    bytes: Vec<u8>,
}

struct LatestPacket {
    priority: u16,
    timestamp_us: u64,
    bytes: Vec<u8>,
    received_at: Instant,
}

pub fn run_multiplexer(app: AppConfig) {
    if app.inputs.is_empty() {
        eprintln!("No inputs configured — nothing to do.");
        return;
    }

    let reg = registry();
    let mut validators: Vec<Validator> = Vec::with_capacity(app.inputs.len());
    for input in &app.inputs {
        match reg.get(input.protobuf()) {
            Some(v) => validators.push(*v),
            None => panic!("no validator registered for proto '{}'", input.protobuf()),
        }
    }

    let (tx, rx) = mpsc::channel::<Incoming>();
    for ((idx, input), validator) in app.inputs.into_iter().enumerate().zip(validators) {
        let tx = tx.clone();
        thread::spawn(move || listener(idx, input, validator, tx));
    }
    drop(tx);
    selector(app.outputs, rx, app.emit_interval, app.staleness);
}

fn listener(input_index: usize, input: Input, validate: Validator, tx: mpsc::Sender<Incoming>) {
    let mut buf = [0u8; 65_535];
    let local = input
        .socket()
        .local_addr()
        .map(|a| a.to_string())
        .unwrap_or_else(|_| "<unknown>".to_string());
    let priority = input.priority();
    let proto_name = input.protobuf().to_string();

    loop {
        match input.socket().recv_from(&mut buf) {
            Ok((len, src)) => {
                let slice = &buf[..len];
                match validate(slice) {
                    Some(timestamp_us) => {
                        let incoming = Incoming {
                            input_index,
                            priority,
                            timestamp_us,
                            bytes: slice.to_vec(),
                        };
                        if tx.send(incoming).is_err() {
                            break;
                        }
                    }
                    None => {
                        eprintln!(
                            "[{} prio={} proto={}] DROP: {} bytes from {} failed to decode",
                            local, priority, proto_name, len, src
                        );
                    }
                }
            }
            Err(e) => {
                eprintln!("[{}] recv_from error: {} — stopping listener", local, e);
                break;
            }
        }
    }
}

fn selector(
    outputs: Vec<Output>,
    rx: mpsc::Receiver<Incoming>,
    emit_interval: Duration,
    staleness: Duration,
) {
    let mut latest: Vec<Option<LatestPacket>> = Vec::new();
    let mut last_emitted: Option<(usize, u64)> = None;
    let mut next_tick = Instant::now() + emit_interval;

    loop {
        let now = Instant::now();
        let wait = next_tick.saturating_duration_since(now);
        match rx.recv_timeout(wait) {
            Ok(msg) => ingest(&mut latest, msg),
            Err(mpsc::RecvTimeoutError::Timeout) => {
                if let Some((idx, winner)) = pick_winner(&latest, staleness) {
                    let key = (idx, winner.timestamp_us);
                    if last_emitted != Some(key) {
                        fan_out(&outputs, idx, winner);
                        last_emitted = Some(key);
                    }
                }
                next_tick += emit_interval;
                let now = Instant::now();
                if next_tick < now {
                    next_tick = now + emit_interval;
                }
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => return,
        }
    }
}

fn ingest(latest: &mut Vec<Option<LatestPacket>>, msg: Incoming) {
    if msg.input_index >= latest.len() {
        latest.resize_with(msg.input_index + 1, || None);
    }
    let accept = match &latest[msg.input_index] {
        Some(prev) => msg.timestamp_us > prev.timestamp_us,
        None => true,
    };
    if !accept {
        return;
    }
    latest[msg.input_index] = Some(LatestPacket {
        priority: msg.priority,
        timestamp_us: msg.timestamp_us,
        bytes: msg.bytes,
        received_at: Instant::now(),
    });
}

fn pick_winner(
    latest: &[Option<LatestPacket>],
    staleness: Duration,
) -> Option<(usize, &LatestPacket)> {
    let now = Instant::now();
    latest
        .iter()
        .enumerate()
        .filter_map(|(i, opt)| opt.as_ref().map(|p| (i, p)))
        .filter(|(_, p)| now.duration_since(p.received_at) <= staleness)
        .min_by(|(_, a), (_, b)| {
            a.priority
                .cmp(&b.priority)
                .then_with(|| b.timestamp_us.cmp(&a.timestamp_us))
        })
}

fn fan_out(outputs: &[Output], winner_idx: usize, winner: &LatestPacket) {
    for out in outputs {
        match out.socket().send(&winner.bytes) {
            Ok(n) => println!(
                "FORWARD input#{} prio={} ts={} -> {} ({} bytes)",
                winner_idx,
                winner.priority,
                winner.timestamp_us,
                out.address()
                    .map(|a| a.to_string())
                    .unwrap_or_else(|_| "<unknown>".to_string()),
                n
            ),
            Err(e) => eprintln!("send to output {:?} failed: {}", out.address().ok(), e),
        }
    }
}