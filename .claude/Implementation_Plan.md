# rove autonomy stack — implementation plan (draft)

## Approach

The spec already lays out 13 phases. This plan reframes them around **risk gates** and **exit criteria** — what has to be proved before the next thing depends on it — rather than a flat task list. The goal is to lock in load-bearing boundaries (Pi UDP API, Python sidecar over iceoryx2, L0 reflex authority) before any code that depends on them gets written, and to fork independent streams once the spine is alive.

---

## M0 — De-risking spikes (do these first, throw the code away)

The architecture has three places where being wrong is expensive. Prove them before committing.

1. **Sidecar round-trip** — iceoryx2 Rust↔Python, joint-state pub at 200 Hz + IK request/response on a real 6DOF URDF via robotics-toolbox. Target: IK p99 < 10 ms, FK publish jitter < 1 ms. **Decision output:** iceoryx2 vs UDS+bincode for the Rust↔Python boundary.
2. **Gaussian-splat budget** — single CUDA pass on Orin: lidar voxelize + camera splat aggregate for one frame. Target: < 100 ms end-to-end. If this misses, the speed-adaptive fallbacks in the spec need tightening before perception is built.
3. **Pi UDP API** — bidirectional latency, packet loss under load, command-then-telemetry round trip. Target: 50 Hz steady, < 5 ms one-way.

**Gate:** Sidecar architecture is go/no-go here. If 50 Hz teleop IK can't be hit, redesign teleop arm control before writing more code (Cartesian-impedance fallback, or move IK into Rust with a lighter solver).

---

## M1 — Hardware spine + L0 (safety floor)

Nothing commands motors until L0 owns the wire.

- Pi UDP I/O, Jetson direct sensor I/O (read-only at first)
- System Blackboard, iceoryx2 channels
- Logging infrastructure
- ReflexEngine: rule registry, action types, 200 Hz loop
- Rules: hardware limits, orientation, stuck detection, thermal, electrical
- `HardwareWriter` private to the reflex module (compiler enforces)

**Exit:** Robot can be teleop'd at the lowest level (raw velocity commands) with L0 clipping to safe state. Bench tests prove pitch/roll cut, velocity clip, joint limits clip. No service or mission code yet.

---

## M2 — World model alive

- Python sidecar: load URDF, expose iceoryx2 interface
- Joint-state forwarding Rust → sidecar
- Sidecar publishes link transforms, sensor poses, directional minimums (empty env initially), body envelope
- `SpatialEngine` in Rust: cache + async client
- L0 wires up proximity rules against directional minimums

**Exit:** Bare 3D viewer (or tooling) shows URDF tracking live joint states. `SpatialEngine.solve_ik()` works from Rust at 50 Hz.

---

After M2, three streams can fork in parallel:

```
M2 ──┬── M3  Position
     ├── M4  Perception          (M3+M4 → M5 Locomotion)
     └── M6  Mission framework    (pure software, no hw deps)
```

---

## M3 — Position truth

Wheel + IMU dead reckoning → add VN300 GNSS → integrity monitor + state machine → lidar odometry → drift budget. Publishes to blackboard and forwards fused pose to sidecar.

**Exit:** 30-min field run; GNSS state transitions logged correctly; drift estimate matches ground truth within calibrated band.

## M4 — Perception

Lidar → voxel pipeline → push deltas to sidecar (collision queries against live env work) → camera ingestion + timestamp alignment → Gaussian-splat color → speed-adaptive subsampling.

**Exit:** 10 Hz voxel deltas flowing; sidecar answers `check_clearance` against real environment; viewer renders colored voxels.

## M5 — Locomotion (gated on M3 + M4)

`MotionService.drive_toward` → path planning via sidecar → clearance checking → `BodyAwarenessService` reconfiguration search → `Reconfigure` sub-mission + orchestrator integration → flipper coordination.

**Exit:** Robot drives 20 m through a narrow passage, returns `NEEDS_RECONFIGURE`, orchestrator inserts arm-stow, mission resumes after replan.

## M6 — Mission framework (parallel to M3–M5)

Mission trait, Status, CancelToken → composites (Sequence/Selector/Loop/Parallel + policies) → decorators → tree runner with cancellation propagation + auto-escalation → RON loader + schema validation → mission registry + `run_id` tracking.

**Exit:** A toy 3-leaf compound runs end-to-end with cooperative cancel and auto-escalation to abort. RON loader rejects malformed trees with clear errors.

---

## M7 — Mission layer

PreconditionEngine + PreemptionEngine with default rule set → Orchestrator tick loop → CommsMonitor + SafePointService → leaf missions (GoTo, HoldPosition, Arm/Flipper/Gripper primitives, Retreat, MarkPOI, SetSafePoint) → compound missions from RON (Sentinel, Waypoint, ReturnHome, BacktrackComm, Reset, Calibrate).

**Exit:** Sentinel patrols A↔B; injecting a battery-critical condition correctly preempts to ReturnHome via the priority-80 rule; failure handler runs when GoTo fails.

## M8 — Vision (parallel-able from end of M4)

VisionService consumer for RT-DETR + NvDCF stream → 2D→3D fusion via sidecar + lidar → cross-camera dedup → velocity estimation, occlusion timeout → vision-dependent missions: Follow, Intercept, Track, Inspect, PickUp.

**Exit:** Follow mission tracks a person across camera handoff with a stable unified `object_id`.

## M9 — Operator interface

Command Router UDP + mode state machine → protobuf schemas (versioned with firmware) → autonomous command handlers → teleop handler (50 Hz IK loop via sidecar) → mode transitions with cooperative cancel + auto-escalation → telemetry stream out.

**Exit:** Steam Deck drives the robot in teleop and starts/cancels autonomous missions; mode switch is clean; telemetry streams consistently.

## M10 — Streaming + replay + 3D viewer

StreamService publishes full state to comms layer (delta encoding lives below us) → RecordingService writes to disk → 3D viewer (live + replay + comparison). **Tech stack decision needed before this milestone — not before.**

## M11 — Field calibration + polish

Tune every `_TBD_` parameter (motor temps, current spike thresholds, RSSI bands, drift budgets, voxel resolution) → long-duration mission tests → fun missions (Dance, HelloWorld, Worm).

---

## Decisions needed early

| Decision | When | Notes |
|---|---|---|
| iceoryx2 vs UDS for sidecar | End of M0 | Spec already names UDS+bincode as fallback |
| Protobuf schema versioning policy | Before M9 schema work | Locks operator wire format |
| Log format + storage layout | M1 | Touches every service |
| Voxel resolution | After M4 profiling | 0.10 m is a starting default |
| 3D viewer tech stack | Before M10 | Three.js / wgpu / Bevy / Godot — defer |

## Top risks (and what to do about them)

1. **Sidecar latency miss** — biggest single risk. Mitigation: M0 spike, with a fallback plan (Cartesian impedance for teleop arm, or in-Rust IK).
2. **iceoryx2 Python bindings immature** — falls back to UDS+bincode per spec; the spike confirms which we ship.
3. **Gaussian-splat compute budget on Orin** — speed-adaptive policy is in the spec, but the M0 spike has to validate the worst case before M4 builds on it.
4. **GNSS spoofing false positives in real RF noise** — only tunable in field; budget calibration time in M11.
5. **Reconfigure correctness** — deeply tangled with orchestrator state machine; treat as the hardest test case in M5/M7 and write replay-based regression cases against recorded logs.

---

## Workspace layout

Single Cargo workspace at the root of `capra_jetson_stack`. One Python project for the sidecar. Crate boundaries follow the architecture's authority layers — boundaries are load-bearing, not cosmetic.

```
capra_jetson_stack/
├── Cargo.toml                      # workspace root
├── rust-toolchain.toml             # pinned toolchain
│
├── crates/
│   ├── rove-types/                 # shared types: Pose, Coordinate, JointStates, etc.
│   ├── rove-proto/                 # generated protobuf types (operator wire format)
│   ├── rove-blackboard/            # System Blackboard (RwLock-backed registry)
│   ├── rove-policy/                # rule-registry pattern reused by L0/L2/L3
│   │
│   ├── rove-hw/                    # Pi UDP I/O + Jetson sensor I/O
│   │                               #   HardwareWriter is pub(crate) — only rove-reflex
│   │                               #   re-exports a submit interface
│   ├── rove-reflex/                # L0 ReflexEngine; owns HardwareWriter
│   │                               #   pub fn submit(cmd) — only entry point to motors
│   │
│   ├── rove-spatial/               # SpatialEngine: iceoryx2 client + cache
│   ├── rove-position/              # PositionService + GNSS integrity monitor
│   ├── rove-perception/            # MapService: CUDA Gaussian-splat pipeline
│   ├── rove-vision/                # VisionService: RT-DETR/NvDCF consumer + 3D fusion
│   ├── rove-comms/                 # CommsMonitor (tri-state link)
│   ├── rove-safepoints/            # SafePointService
│   ├── rove-motion/                # MotionService + BodyAwarenessService
│   ├── rove-arm/                   # ArmService (75 Hz loop)
│   ├── rove-gripper/               # GripperService (200 Hz loop)
│   │
│   ├── rove-mission/               # Mission trait, composites, decorators, RON loader
│   ├── rove-precondition/          # L2 PreconditionEngine + default types
│   ├── rove-orchestrator/          # L3 PreemptionEngine + tree runner + Reconfigure logic
│   │
│   ├── rove-router/                # Command Router (UDP listener, mode SM, telemetry out)
│   ├── rove-stream/                # StreamService
│   ├── rove-record/                # RecordingService
│   │
│   └── rove-bin/                   # main binary; wires services, owns the runtime
│
├── sidecar/                        # Python project (uv or poetry)
│   ├── pyproject.toml
│   ├── src/sidecar/
│   │   ├── __main__.py             # entry point, iceoryx2 setup
│   │   ├── world.py                # URDF + voxel env wrapper around robotics-toolbox
│   │   ├── ik.py                   # damped-least-squares wrapper
│   │   ├── planner.py              # body path + arm trajectory
│   │   ├── directional.py          # directional-minimum publisher (200 Hz consumer)
│   │   └── ipc.py                  # iceoryx2 (or UDS fallback) bindings
│   └── tests/
│
├── urdf/
│   ├── rove.urdf.xacro             # source URDF
│   ├── rove.urdf                   # generated, hash-stamped
│   └── meshes/
│
├── proto/
│   ├── operator.proto              # operator wire format (versioned)
│   └── iceoryx_frames.proto        # shared-memory frame schemas (sidecar boundary)
│
├── config/
│   ├── missions/                   # compound mission trees (RON, hot-reloadable)
│   │   ├── sentinel.ron
│   │   ├── return_home.ron
│   │   └── ...
│   ├── rules/
│   │   ├── reflex.ron              # L0 default rules
│   │   └── preemption.ron          # L3 default rules
│   └── thresholds.ron              # calibration values (one source of truth)
│
├── tests/
│   ├── integration/                # cross-crate scenarios
│   ├── replay/                     # recorded missions used as regression fixtures
│   └── spikes/                     # M0 spike code, kept for reference
│
├── tools/
│   ├── viewer/                     # off-robot 3D viewer (separate crate, M10)
│   └── log-inspector/              # replay log dump utility
│
└── .claude/                        # spec + plan
    ├── SAR_Mission_Spec.md
    ├── SAR_Software_Architecture.md
    └── Implementation_Plan.md
```

### Crate dependency rules

The dep graph encodes the authority model. CI should fail PRs that violate it.

```
rove-bin
   ├── rove-router → rove-orchestrator → rove-mission → rove-precondition
   │                                               ↓
   │                                          (services)
   │                                               ↓
   │                                         rove-reflex
   │                                               ↓
   │                                          rove-hw
   ├── rove-stream / rove-record → (read-only on services + blackboard)
   └── rove-spatial → (iceoryx2 to sidecar)

Forbidden edges (enforced by `cargo deny` or a workspace lint):
- Anything → rove-hw (only rove-reflex may depend on it)
- Missions / services → raw blackboard writes (read-only view exposed in rove-mission)
- L1 services → rove-orchestrator (services must not know who's calling them)
```

### Why this split

- `rove-hw` private to `rove-reflex` is the compiler-level enforcement the spec calls for. There's no way for a mission or service to grab the hardware writer because it isn't re-exported.
- `rove-policy` shared by L0/L2/L3 means rule registration code is one implementation, not three.
- `rove-types` and `rove-proto` separate so wire format can version independently of internal types.
- `rove-bin` is intentionally thin: build the services, hand them refs, run loops. All logic lives in the leaf crates.

---

## M0 spikes — concrete scope

Each spike is a standalone repo (or `tests/spikes/<name>/`) that proves one thing. Not production code. Time-boxed to a few days each.

### Spike 1 — Sidecar round-trip

**Goal:** Prove iceoryx2 Rust↔Python (or UDS fallback) hits the latency budget on the actual Orin.

**Build:**
- Minimal Python process that loads a 6DOF URDF via robotics-toolbox, exposes one iceoryx2 service: `solve_ik(target_pose) → joint_states`. Also subscribes to `joint_states` and publishes `link_transforms`.
- Minimal Rust binary that publishes joint_states at 200 Hz (synthetic), subscribes to link_transforms, and fires `solve_ik` requests at 50 Hz.
- Histogram p50/p95/p99 on:
  - End-to-end IK round-trip latency
  - link_transforms publish-to-receive latency
  - Joint-state publish-to-receive lag in the sidecar

**Decision:** If iceoryx2 Python bindings can't hit p99 < 10 ms or are unstable, swap in UDS+bincode and re-measure. Document which one ships.

**Side product:** Repeatable benchmark harness — keep it in `tests/spikes/sidecar_latency/` for regression checks once the real sidecar is built.

### Spike 2 — Gaussian-splat budget

**Goal:** Validate the perception loop math fits in the 10 Hz budget on a 32GB Orin under realistic load.

**Build:**
- Synthetic input: one frame of Livox-ish point cloud (~24k points) + 8 camera frames at 1080p.
- CUDA pass: voxelize lidar → 0.10 m grid → frustum cull per camera → splat aggregate weighted color per voxel.
- Measure end-to-end wall time + GPU occupancy + memory footprint.
- Repeat at 1, 2, 4, 8 cameras to find where it falls over.

**Decision:** Confirm/adjust voxel resolution, splat-spread curves, and the speed-adaptive subsampling thresholds in the spec.

**Side product:** Reusable CUDA kernels for `rove-perception`.

### Spike 3 — Pi UDP API

**Goal:** Prove the Jetson↔Pi UDP link is reliable enough at 50 Hz with full payload, and characterize the failure modes.

**Build:**
- Echo loopback on the Pi: receive a command frame, immediately reply with a synthetic telemetry frame.
- Jetson client: send commands at 50 Hz, log packet loss, RTT, jitter.
- Soak test: 1 hour at full rate. Run with the switch saturated by another stream to test contention.
- Inject network conditions (`tc qdisc` / `netem`) to see what happens at 1%, 5%, 10% loss — the spike output informs how aggressive L0's hardware-link timeout should be.

**Decision:** Fix the wire schema (compact, fixed-size where possible). Set the hardware-link watchdog timeout. Decide whether telemetry needs sequence numbers or per-channel timestamps.

**Side product:** The wire-format header used by `rove-hw` in M1.

---

## Definition of "done" per milestone

A milestone is done when **all three** are true:

1. **Exit criterion** is demonstrably met (bench or field test, not "it compiles").
2. **Replay log** of the demonstration is committed to `tests/replay/`.
3. **Regression test** runs that log against current code and asserts the same outcome.

This means by M5 we already have a small library of replay regressions. By M11 we have dozens — and any change that breaks Reconfigure or fallback ordering surfaces in CI, not in the field.

---

## What I'd build first if I had one day

Not the spikes — those are days each. If I had one day to derisk *the riskiest assumption*:

A standalone Python process loading the actual rove URDF in robotics-toolbox, doing 1000 IK solves on a benchmark and printing p50/p95/p99. That's the single number that makes or breaks the sidecar architecture. If it's already 8 ms in pure Python before any IPC overhead, the architecture as written doesn't survive — and we'd want to know that before committing to anything else.
