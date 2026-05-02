use std::env;

use udp_multiplexer::helpers::app_config::AppConfig;
use udp_multiplexer::run_multiplexer;

fn resolve_config_path() -> String {
    let mut args = env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--config" => return args.next().expect("--config requires a path"),
            "-h" | "--help" => {
                println!("usage: udp_multiplexer [--config PATH]");
                println!("  env: ROVE_CONFIG=PATH overrides the default");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {}", other);
                std::process::exit(2);
            }
        }
    }
    env::var("ROVE_CONFIG").unwrap_or_else(|_| "config.toml".to_string())
}

fn main() {
    let config_path = resolve_config_path();

    let app = match AppConfig::new(&config_path) {
        Ok(c) => c,
        Err(e) => {
            eprintln!(
                "CRITICAL: Failed to load configuration from '{}': {}",
                config_path, e
            );
            std::process::exit(1);
        }
    };

    println!("Using config: {}", config_path);
    println!("Scanning Protobuf path: {}", app.path_to_protobuf);
    println!(
        "Loaded {} input(s) and {} output(s). emit_interval={:?}, staleness={:?}",
        app.inputs.len(),
        app.outputs.len(),
        app.emit_interval,
        app.staleness,
    );
    for input in &app.inputs {
        if let Ok(addr) = input.socket().local_addr() {
            println!(
                "Input listening on {} [priority: {}, proto: {}]",
                addr,
                input.priority(),
                input.protobuf()
            );
        }
    }
    for output in &app.outputs {
        if let Ok(addr) = output.address() {
            println!("Output ready -> {} [proto: {}]", addr, output.protobuf());
        }
    }

    run_multiplexer(app);
}