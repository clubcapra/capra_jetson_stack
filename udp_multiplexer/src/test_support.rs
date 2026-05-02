//! Shared end-to-end scenario runner used by the per-proto test binaries.
//!
//! Each proto-specific binary supplies two closures:
//!   - `build_packet(priority, left_vel, right_vel) -> Vec<u8>` — encode a
//!     valid wire payload that embeds `left_vel` (so the test can tell who
//!     sent it) and a fresh `timestamp_us`.
//!   - `decode_priority(&[u8]) -> Option<u16>` — decode a wire payload and
//!     return the priority the sender encoded into it, or None on decode
//!     failure (in which case `bad` is incremented).
//!
//! The scenarios are identical regardless of proto type: priority wins,
//! junk is dropped, staleness triggers failover. Identical behavior is the
//! point — any proto should produce the same verdicts through the same
//! multiplexer.

use std::collections::VecDeque;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::helpers::app_config::AppConfig;
use crate::run_multiplexer;

pub type PacketBuilder = Arc<dyn Fn(u16, f32, f32) -> Vec<u8> + Send + Sync + 'static>;
pub type PriorityDecoder = Arc<dyn Fn(&[u8]) -> Option<u16> + Send + Sync + 'static>;

#[derive(Clone)]
struct Received {
    priority: Option<u16>,
    at: Instant,
    decoded_ok: bool,
}

#[derive(Debug, Default)]
pub struct Counts {
    pub prio1: u64,
    pub prio2: u64,
    pub other: u64,
    pub bad: u64,
}

fn pf(b: bool) -> &'static str {
    if b {
        "PASS"
    } else {
        "FAIL"
    }
}

/// Run the three-scenario test suite against `config_path`.
/// Returns process exit code (0 = all pass, 1 = any failure).
pub fn run_suite(
    suite_name: &str,
    config_path: &str,
    build_packet: PacketBuilder,
    decode_priority: PriorityDecoder,
) -> i32 {
    let app = match AppConfig::new(config_path) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("failed to load '{}': {}", config_path, e);
            return 2;
        }
    };

    let input_entries: Vec<(String, u16)> = app
        .inputs
        .iter()
        .map(|i| (i.socket().local_addr().unwrap().to_string(), i.priority()))
        .collect();
    let output_addrs: Vec<String> = app
        .outputs
        .iter()
        .map(|o| o.address().unwrap().to_string())
        .collect();
    let emit_interval = app.emit_interval;
    let staleness = app.staleness;

    let has_prio = |p: u16| input_entries.iter().any(|(_, pr)| *pr == p);
    if !has_prio(1) || !has_prio(2) {
        eprintln!("'{}' requires inputs with priority 1 and 2", config_path);
        return 2;
    }
    if output_addrs.is_empty() {
        eprintln!("'{}' requires at least one output", config_path);
        return 2;
    }

    let stop = Arc::new(AtomicBool::new(false));
    let mut listener_handles = Vec::new();
    let mut logs = Vec::new();
    for addr in &output_addrs {
        let (h, log) = spawn_output_listener(addr, Arc::clone(&stop), Arc::clone(&decode_priority));
        listener_handles.push(h);
        logs.push(log);
    }

    let mux_handle = thread::spawn(move || run_multiplexer(app));
    thread::sleep(Duration::from_millis(200));

    println!("=== {} ===", suite_name);
    println!("config: {}", config_path);
    println!(
        "emit_interval={:?} staleness={:?} inputs={} outputs={}",
        emit_interval,
        staleness,
        input_entries.len(),
        output_addrs.len()
    );

    let addr_of = |want: u16| -> String {
        input_entries
            .iter()
            .find(|(_, p)| *p == want)
            .map(|(a, _)| a.clone())
            .unwrap()
    };

    // --- Scenario A: priority wins ---
    println!("\n[A] priority: prio=1 & prio=2 stream; outputs must see only prio=1");
    let a_start = Instant::now();
    let a_duration = Duration::from_millis(1500);
    let s1 = spawn_sender(
        addr_of(1),
        1,
        50.0,
        a_duration,
        Arc::clone(&stop),
        false,
        Arc::clone(&build_packet),
    );
    let s2 = spawn_sender(
        addr_of(2),
        2,
        50.0,
        a_duration,
        Arc::clone(&stop),
        false,
        Arc::clone(&build_packet),
    );
    s1.join().unwrap();
    s2.join().unwrap();
    thread::sleep(Duration::from_millis(100));
    let a_counts = tally(&logs, a_start, Instant::now());
    let pass_priority = a_counts.prio1 > 0 && a_counts.prio2 == 0;
    println!("  {:?}  [{}]", a_counts, pf(pass_priority));
    thread::sleep(staleness + Duration::from_millis(100));

    // --- Scenario B: junk dropped ---
    println!("\n[B] junk: 1-in-5 garbage in prio=1 stream; outputs must see 0 decode failures");
    let b_start = Instant::now();
    let sb = spawn_sender(
        addr_of(1),
        1,
        50.0,
        Duration::from_millis(1500),
        Arc::clone(&stop),
        true,
        Arc::clone(&build_packet),
    );
    sb.join().unwrap();
    thread::sleep(Duration::from_millis(100));
    let b_counts = tally(&logs, b_start, Instant::now());
    let pass_junk = b_counts.bad == 0 && b_counts.prio1 > 0;
    println!("  {:?}  [{}]", b_counts, pf(pass_junk));
    thread::sleep(staleness + Duration::from_millis(100));

    // --- Scenario C: staleness failover ---
    println!("\n[C] staleness: prio=1 stops mid-run; prio=2 must take over after staleness");
    let c_start = Instant::now();
    let prio1_window = Duration::from_millis(800);
    let total_c = prio1_window + staleness + Duration::from_millis(700);
    let sc1 = spawn_sender(
        addr_of(1),
        1,
        50.0,
        prio1_window,
        Arc::clone(&stop),
        false,
        Arc::clone(&build_packet),
    );
    let sc2 = spawn_sender(
        addr_of(2),
        2,
        50.0,
        total_c,
        Arc::clone(&stop),
        false,
        Arc::clone(&build_packet),
    );
    let prio1_end = c_start + prio1_window;
    sc1.join().unwrap();
    sc2.join().unwrap();
    thread::sleep(Duration::from_millis(100));
    let c_early = tally(&logs, c_start, prio1_end);
    let c_late = tally(
        &logs,
        prio1_end + staleness + Duration::from_millis(150),
        Instant::now(),
    );
    let pass_staleness =
        c_early.prio1 > 0 && c_early.prio2 == 0 && c_late.prio2 > 0 && c_late.prio1 == 0;
    println!("  early: {:?}", c_early);
    println!("  late : {:?}  [{}]", c_late, pf(pass_staleness));

    stop.store(true, Ordering::Relaxed);
    let _ = mux_handle;
    for h in listener_handles {
        let _ = h.join();
    }

    println!("\n=== verdict [{}] ===", suite_name);
    println!("  priority  : {}", pf(pass_priority));
    println!("  junk      : {}", pf(pass_junk));
    println!("  staleness : {}", pf(pass_staleness));
    let ok = pass_priority && pass_junk && pass_staleness;
    println!("\n{}\n", if ok { "OK" } else { "FAIL" });
    if ok { 0 } else { 1 }
}

fn spawn_output_listener(
    addr: &str,
    stop: Arc<AtomicBool>,
    decode_priority: PriorityDecoder,
) -> (thread::JoinHandle<()>, Arc<Mutex<VecDeque<Received>>>) {
    let sock = UdpSocket::bind(addr)
        .unwrap_or_else(|e| panic!("test listener bind {} failed: {}", addr, e));
    sock.set_read_timeout(Some(Duration::from_millis(100))).ok();

    let log: Arc<Mutex<VecDeque<Received>>> = Arc::new(Mutex::new(VecDeque::new()));
    let log_w = Arc::clone(&log);

    let h = thread::spawn(move || {
        let mut buf = [0u8; 65_535];
        while !stop.load(Ordering::Relaxed) {
            match sock.recv_from(&mut buf) {
                Ok((len, _)) => {
                    let entry = match decode_priority(&buf[..len]) {
                        Some(p) => Received {
                            priority: Some(p),
                            at: Instant::now(),
                            decoded_ok: true,
                        },
                        None => Received {
                            priority: None,
                            at: Instant::now(),
                            decoded_ok: false,
                        },
                    };
                    log_w.lock().unwrap().push_back(entry);
                }
                Err(_) => {}
            }
        }
    });
    (h, log)
}

fn spawn_sender(
    addr: String,
    priority: u16,
    rate_hz: f64,
    dur: Duration,
    stop: Arc<AtomicBool>,
    mix_junk: bool,
    build_packet: PacketBuilder,
) -> thread::JoinHandle<()> {
    let deadline = Instant::now() + dur;
    thread::spawn(move || {
        let sock = UdpSocket::bind("0.0.0.0:0").expect("sender bind");
        sock.connect(&addr).expect("sender connect");
        let left = priority as f32 * 0.1;
        let right = -(priority as f32) * 0.1;
        let period = Duration::from_secs_f64(1.0 / rate_hz);

        let mut i: u64 = 0;
        let mut next = Instant::now();
        while !stop.load(Ordering::Relaxed) && Instant::now() < deadline {
            let payload = if mix_junk && i % 5 == 0 {
                b"GARBAGE_BYTES_NOT_A_VALID_PROTO_MESSAGE".to_vec()
            } else {
                build_packet(priority, left, right)
            };
            let _ = sock.send(&payload);
            i += 1;
            next += period;
            let now = Instant::now();
            if next > now {
                thread::sleep(next - now);
            } else {
                next = now;
            }
        }
    })
}

fn tally(
    logs: &[Arc<Mutex<VecDeque<Received>>>],
    from: Instant,
    to: Instant,
) -> Counts {
    let mut c = Counts::default();
    for log in logs {
        let log = log.lock().unwrap();
        for e in log.iter() {
            if e.at < from || e.at > to {
                continue;
            }
            if !e.decoded_ok {
                c.bad += 1;
                continue;
            }
            match e.priority {
                Some(1) => c.prio1 += 1,
                Some(2) => c.prio2 += 1,
                _ => c.other += 1,
            }
        }
    }
    c
}

/// Convenience: decode the `left_vel` encoding used by senders and map to priority.
pub fn priority_from_left(left: f32) -> Option<u16> {
    let n = (left * 10.0).round();
    if (1.0..=20.0).contains(&n) && (n - (left * 10.0)).abs() < 0.01 {
        Some(n as u16)
    } else {
        None
    }
}