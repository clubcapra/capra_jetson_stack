fn main() {
    prost_build::Config::new()
        .compile_protos(
            &[
                "src/proto/RoveControl.proto",
                "src/proto/RoveTelemetry.proto",
            ],
            &["src/"],
        )
        .expect("failed to compile protobufs");
}