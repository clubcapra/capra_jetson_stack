# rove_control_interface

Rust glue between the Steam Deck teleop, the [`rove_mvp_engine`](../rove_mvp_engine)
IK solver, and the [`rove_sensor_api`](../../capra_roboguard/rove_sensor_api)
kinova driver.

```
            RoveControl                      JointState (q)
 teleop ─────────────▶ rove_control ─────▶ rove_mvp_engine ──┐
            (UDP 7000)   interface         (UDP 9501/9502)   │
                                                             │
                                              JointCommand   │
                                                  (UDP 9503) ▼
                            kinova arm  ◀──────  rove_control_interface
                            (deg/s vel)            (api JSON over UDP)
                                                             │
                          RoveTelemetry  ◀───────────────────┤
                            (UDP 7001)
```

This binary subscribes to `kinova_arm` data on port 5002, forwards every
fresh joint frame as a `JointState` to the IK engine on port 9501, and
forwards the Ovis Twist field of each `RoveControl` packet to port 9502.
The engine's `JointCommand` reply (resolved-rate joint velocities) is
re-emitted as a kinova `joint_*_vel` command on port 5003. Telemetry is
mirrored back to whatever address last sent us a control packet, so the
operator UI sees joint positions / currents in near real time.

## Run

```sh
cargo run --release -- --config config.toml
```

The IK engine and the rove_sensor_api can be on the same host (default) or
elsewhere — both endpoints are configurable in [config.toml](config.toml).

## Joint mapping

The IK chain has five movable joints; the kinova arm has six. The
configured `[[joint_map]]` entries pair each chain joint with a kinova
slot (1-indexed) and record the home pose (deg) the arm reads at its
firmware home — this matches the offsets baked into the IK engine's
[chain.json](../rove_mvp_engine/data/chain.json) so the engine sees q=0
when the arm is at home. Joint 6 is held at zero velocity (no IK control).

## Wire formats in one place

| Link | Direction | Encoding |
|---|---|---|
| teleop ↔ this | UDP 7000/7001 | `RoveControl` / `RoveTelemetry` Protobuf |
| this → engine | UDP 9501 | `JointState` Protobuf (chain entity ids, rad) |
| this → engine | UDP 9502 | `Twist` Protobuf (normalized [-1, 1]) |
| engine → this | UDP 9503 | `JointCommand` Protobuf (rad/s for resolved-rate) |
| this ↔ kinova api | UDP 5002/5003 | rove_sensor_api 4-byte header + JSON |

All Protobuf encoding/decoding is hand-rolled in
[src/proto.rs](src/proto.rs) — wire-compatible with `protoc`-generated
code on either side, no codegen step.
