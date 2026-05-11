# SAR Robot — Software Architecture

_How the autonomy layer is built._

---

## Architecture Overview

Five-tier system with strict authority boundaries. Each layer has one job and one direction of authority.

```
┌─────────────────────────────────────────────────────────┐
│  Operator (Steam Deck)                                  │
│  Sends mission commands, teleop control states, E-stop  │
│  Receives telemetry, mission status, alerts             │
└────────────────────────┬────────────────────────────────┘
                         │ UDP (Microhard / LoRa)
┌────────────────────────┼────────────────────────────────┐
│  Command Router                                         │
│  Mode state machine (autonomous / teleop)               │
│  Validates and routes operator input                    │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  L3 — Orchestrator (PreemptionEngine)                   │
│  Owns the running tree, ticks it, preempts, watchdogs,  │
│  inserts Reconfigure sub-missions, runs fallbacks       │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  L2 — Preconditions (PreconditionEngine)                │
│  Declarative metadata per mission node                  │
│  Checked at start, monitored during execution           │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  Missions (Leaf + Compound)                             │
│  Pure intent — call services, never hardware            │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  L1 — Services                                          │
│  Python Sidecar (Robotics Toolbox — world model),       │
│  SpatialEngine, MapService, PositionService,            │
│  MotionService, BodyAwarenessService, ArmService,       │
│  GripperService, VisionService, CommsMonitor,           │
│  SafePointService, StreamService, RecordingService      │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  L0 — Safety Reflex (ReflexEngine)                      │
│  Runs at sensor rate, final authority on every          │
│  command, missions and operator CANNOT override         │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────┐
│  Hardware                                               │
│  Pi (UDP): ODrive, 6DOF, Gripper, Battery, PMCI         │
│  Jetson (Direct): VN300, Livox, IMUs, Cameras           │
└─────────────────────────────────────────────────────────┘
```

**Principle:** Missions are intent. Services are capability. Safety reflex is reality. The operator commands intent; the safety reflex has final say.

### Tick Flow

```
Pi API @ 50 Hz                              # heartbeat
  → CommandRouter.tick()                    # process operator UDP, route by mode
      → Autonomous mode:
      → Orchestrator.tick()                 # L3
          → Services.tick()                 # L1 — update world model
          → PreemptionEngine.evaluate()     # L3 — check rules
          → PreconditionEngine.check()      # L2 — invariants
          → ActiveMission.tick(cancel_token) # tree tick
              → Service.action()            # capability
                  → SpatialEngine queries   # via iceoryx2 to Python sidecar
                  → SafetyReflex.write()    # L0
                      → Hardware
      → Teleop mode:
      → TeleopHandler.tick()                # apply control state
          → ArmService.solve_ik(target)     # via Python sidecar
          → SafetyReflex.write()            # L0
              → Hardware
  → StreamService.tick()                    # publish state to comms layer
  → RecordingService.tick()                 # write log frame
```

L0 ReflexEngine runs in its own loop at 200 Hz, independent of the main tick.

---

## Runtime & Concurrency

The autonomy layer runs in Rust. The single exception is the Python sidecar that wraps Peter Corke's [robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python). The sidecar is the canonical world model — URDF, voxel environment, FK, IK, path planning, collision checking all live there.

### Loops

Different layers have different rate requirements. The refactor must respect this — not everything ticks together.

| Loop | Rate | Components | Notes |
|---|---|---|---|
| Reflex loop | 200 Hz | ReflexEngine (L0) | Independent thread. Reads telemetry directly, writes hardware. Cannot block on anything else. Matches IMU/VN300 rate. |
| Position loop | 200 Hz | PositionService Kalman fusion | Matches IMU rate, the highest-frequency input to the fusion. |
| Mission tick | 50 Hz | CommandRouter, Orchestrator, Missions, MotionService, BodyAwarenessService, PreemptionEngine, PreconditionEngine | Matches Pi API publish rate. 20ms tick budget. |
| Arm control | 75 Hz | ArmService control loop | Driven by arm hardware rate. Interpolates between mission-set targets. |
| Gripper control | 200 Hz | GripperService | Matches gripper hardware rate. |
| Perception loop | 10 Hz | MapService voxel pipeline (Gaussian splatting) | Matches Livox native rate. Pushes voxel deltas to Python sidecar. |
| Vision pipeline | ~20 fps | RT-DETR + NvDCF (external) → VisionService consumer | External pipeline; VisionService just consumes the stream. |
| Stream loop | Configurable | StreamService, RecordingService | Independent. Bandwidth-adaptive throttling lives in the comms layer below. |

### Cross-Loop Communication

Loops never make synchronous calls across boundaries. They communicate through three mechanisms:

1. **iceoryx2 shared memory** — primary IPC, both Rust↔Rust and Rust↔Python. Sub-microsecond latency for local pub/sub. Used for high-frequency state (joint states, voxel deltas, sensor poses, directional minimums).
2. **iceoryx2 request/response** — used for query-shaped operations (IK solve, path plan, collision check) that cross to the Python sidecar. Each call has a configurable timeout (default 50 ms); on timeout, the call fails and the caller handles the failure.
3. **System Blackboard** — Rust-side shared state with read-write locks. Used for mission-rate state visible across services (position_state, comms_status, reflex_flags, safe_points).

### Rust ↔ Python Sidecar Boundary

Robotics Toolbox runs in a Python sidecar process. iceoryx2 is the IPC.

**The sidecar publishes (continuous):**

- Link transforms / sensor poses for current joint state — Rust services consume via shared memory
- Directional minimums (nearest occupied voxel in each cardinal direction + above + below) — L0 ReflexEngine reads at 200 Hz
- Body collision envelope at current config — for fast L0 + reflex use

**The sidecar serves (request/response):**

- IK solve (used at 50 Hz in teleop, on-demand in autonomous)
- Path planning for body navigation (on-demand)
- Trajectory smoothing for arm motion (on-demand)
- Collision check for hypothetical configs (used by BodyAwarenessService for reconfiguration search)

**The sidecar consumes (subscriptions):**

- Joint state updates from Rust (continuous, published as the Pi reports them)
- Voxel deltas from MapService (10 Hz, atomic environment updates)

If the Python bindings for iceoryx2 are immature at implementation time, the fallback is a Unix domain socket with a binary protocol (bincode) for the Rust↔Python boundary while keeping iceoryx2 for Rust↔Rust.

### What L0 reads

L0 runs faster than the main tick, so it cannot do full collision queries inline. It reads:

- Raw telemetry (ODrive, 6DOF, gripper, VN300, battery) directly from hardware queues
- Pre-computed directional minimums published by the Python sidecar to a shared memory channel
- Reflex flag state shared with L3 via the system blackboard

L0 never makes synchronous service calls. The directional minimums it reads are slightly stale (up to 100 ms in the worst case from the 10 Hz perception loop), which is acceptable because L0 actions are conservative and fail-safe — clipping and derating, never aggressive maneuvers.

---

## Command Router

Entry point for the autonomy layer. Receives operator UDP traffic from the Steam Deck, validates and routes by mode, publishes telemetry back. The Command Router is the only thing the operator talks to directly.

### Responsibilities

- Maintain the **mode state machine**: autonomous / teleop
- Handle mode transitions cleanly (cancel active mission before teleop, release teleop state before autonomous)
- Validate incoming commands (schema, mission existence, parameter types) before dispatching
- Issue `run_id` for accepted missions, track active runs
- Publish telemetry stream to the operator
- Forward E-stop directly to L0 (bypasses tree)

### Mode State Machine

```
        ┌──────────────┐    StartMission     ┌──────────────┐
        │              │ ──────────────────► │              │
        │   TELEOP     │                     │  AUTONOMOUS  │
        │              │ ◄────────────────── │              │
        └──────────────┘    SwitchToTeleop   └──────────────┘
                            (after cancel)
```

- **TELEOP → AUTONOMOUS**: Releases teleop control state (clears stored joystick/end-effector targets). Validates and starts the requested mission.
- **AUTONOMOUS → TELEOP**: Sends `cancel` to the active mission. Waits for the tree to return CANCELLED (or auto-escalates to abort after timeout). Then accepts teleop control state.
- **E-stop is always immediate** regardless of mode — bypasses both paths and goes directly to L0.

### Operator Commands

Schema versioned with the firmware. Binary protobuf over UDP.

```
StartMission { mission_id, params, failure_handler_override?, run_id }
  → MissionAccepted { run_id }
  → MissionRejected { reason }

CancelMission { run_id }
  → MissionCancelling { run_id }    # acknowledged, cleanup running
  → MissionEnded { run_id, status: CANCELLED }

AbortMission { run_id }
  → MissionEnded { run_id, status: ABORTED }

SetSafePoint { label }
  → SafePointAcknowledged { label }

ClearSafePoint { label }
  → SafePointAcknowledged { label }

SwitchToTeleop {}
  → ModeChanged { mode: TELEOP }

TeleopControl {                      # high-rate, no acknowledgment
  track_velocity_left, track_velocity_right,
  flipper_pos_fl, flipper_pos_fr, flipper_pos_bl, flipper_pos_br,
  gripper_state,
  arm_end_effector_pose,
  camera_subscription
}

EStop {}                             # direct to L0
  → AllMotionStopped {}
```

### Telemetry Stream Out

Continuous publish of state to the operator UDP listener. Comms layer below the autonomy stack handles delta encoding, compression, and bandwidth-mode adaptation between Microhard and LoRa.

The Command Router does not know about LoRa vs Microhard — it publishes full state at full rate. The comms layer decides what fits on the wire.

### Mission Run Tracking

Active mission runs are tracked in a small registry: `{ run_id → { mission_type, start_time, current_state, failure_handler } }`. When a mission ends, the entry is retired and the final status is sent to the operator. New `StartMission` while a run is already active returns `MissionRejected(reason: "mission already active")` unless the running mission is the operator's known idle/fallback mission.

---

## Policy Engine

L0, L2, and L3 use the same pattern: evaluate conditions against state, pick the highest-priority match, execute an action. Behavior is driven by registries of declarative rules — not hardcoded if/else chains. Adding a new constraint, sensor, or failure mode means registering a new rule.

### Rule Structure

```rust
struct Rule {
    id:        String,
    condition: Box<dyn Fn(&State) -> bool>,
    priority:  i32,
    action:    Action,
    escalate:  Option<EscalationTarget>,
    metadata:  HashMap<String, Value>,
    enabled:   bool,
}
```

### Rule Evaluation

Each engine tick:

1. Evaluate all enabled rules against current state
2. Collect rules whose condition returns `true`
3. Sort by priority (descending)
4. Execute the highest-priority action
5. For rules with `escalate`, notify the target layer

Multiple rules can fire simultaneously. Priority resolves conflicts. Tie-breaking is by registration order.

### Rule Registration

```rust
// At startup — from config file or code
engine.register(Rule {
    id: "motor_temp_derate".into(),
    condition: Box::new(|s| s.odrive.any_motor_temp() > thresholds::MOTOR_TEMP_DERATE),
    priority: 50,
    action: Action::DerateVelocity { factor_fn: Box::new(temp_derate_curve) },
    escalate: Some(EscalationTarget::WarnOperator("Motor temp elevated".into())),
    metadata: HashMap::new(),
    enabled: true,
});

// At runtime — dynamically
engine.register(/* new rule */);

// Disable without removing
engine.disable("motor_temp_derate");
```

### Three Engines, Same Pattern

| Engine | Layer | Tick rate | Evaluates against | Actions |
|---|---|---|---|---|
| ReflexEngine | L0 | 200 Hz | Raw telemetry + directional minimums | Clip, derate, freeze, stop |
| PreconditionEngine | L2 | On mission start + each tick | Blackboard + telemetry | Gate or abort |
| PreemptionEngine | L3 | 50 Hz | Blackboard + telemetry | Abort, fallback, warn, restrict |

---

## L0 — Safety Reflex (ReflexEngine)

Runs at 200 Hz in its own thread. Sits between services and hardware. Final authority on every command. Not a mission — a filter.

**Structural enforcement:** Missions and services do not hold a reference to the hardware writer. They submit commands to the ReflexEngine, which is the only thing that talks to the hardware UDP channel. The compiler enforces this — `HardwareWriter` is private to the reflex module.

L0 acts immediately and locally — clip, derate, freeze, stop. It never makes mission decisions but flags conditions upward via the system blackboard so L3 can react.

### Reflex Rule Extensions

L0 rules extend the base Rule with hardware-specific fields:

```rust
struct ReflexRule {
    base: Rule,
    stateful:    bool,        // needs ring buffer (e.g., stuck detection)
    buffer_size: usize,        // cycles for stateful rules
}
```

### Reflex Action Types

```
ReflexAction enum:
    Clip { channel, value }              // cap a value
    Derate { channel, factor_fn }        // progressive reduction by curve
    Freeze { channel }                    // lock current value, reject new
    Stop { channel }                      // zero specific channel
    KillAll                               // zero everything
    HoldForce                             // maintain current force
```

### Default Reflex Rules

Rules loaded from configuration at startup. Each row is a registered rule.

#### Thermal

Derating is preferred over hard cutoff — a 120kg robot stopping dead on a slope is its own problem.

| Source | Telemetry | Warn | Derate | Critical |
|---|---|---|---|---|
| Track motor temps | ODrive | Flag to L3 | Cap velocity progressively | Lock motor, escalate to L3 |
| Flipper motor temps | ODrive | Flag to L3 | Cap flipper speed | Lock flipper, escalate to L3 |
| Arm joint temps | 6DOF | Flag to L3 | Cap joint speeds | Lock joint, escalate to L3 |
| Battery temp | Battery | Flag to L3 | Derate all outputs | Kill non-essential via PMCI, escalate to L3 |

**Configuration (starting defaults — calibrate from field testing):**

| Parameter | Value | Description |
|---|---|---|
| motor_temp_warn | 70°C | Warning threshold |
| motor_temp_derate | 80°C | Begin derating |
| motor_temp_critical | 95°C | Lock motor |
| joint_temp_warn | 60°C | Arm joint warning |
| joint_temp_derate | 70°C | Begin derating joint speed |
| joint_temp_critical | 85°C | Lock joint |
| battery_temp_warn | 45°C | Battery warning |
| battery_temp_derate | 55°C | Begin derating all |
| battery_temp_critical | 60°C | Kill non-essential |

#### Electrical

| Condition | Telemetry | L0 Action | Escalate to L3 |
|---|---|---|---|
| Battery low (warn) | Battery | — | Warn operator |
| Battery low (derate) | Battery | Derate all outputs | Warn operator |
| Battery critical | Battery | Derate all outputs | Force ReturnHome |
| Current spike | ODrive | Cap torque output | — |
| Cell imbalance | Battery | — | Flag |

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| battery_warn_percent | 30% | Warning threshold |
| battery_derate_percent | 20% | Begin derating |
| battery_critical_percent | 10% | Force ReturnHome |
| max_current_spike | _per-motor TBD_ | Torque cap trigger |

#### Stuck Detection

High effort with no result. Uses a ring buffer of N consecutive cycles — not single-cycle checks. Motors pull hard briefly during normal operation (starting on slope, flipper hitting stair edge).

| Condition | Detection Signal | L0 Action | Escalate to L3 |
|---|---|---|---|
| Tracks stuck | High ODrive current + near-zero velocity + VN300 confirms no acceleration, N cycles | Kill drive | Mission handles failure |
| Arm stuck | High joint torque + near-zero velocity, N cycles | Freeze joint | Mission handles failure |
| Gripper stuck | Force climbing + no position change | Hold current force | Mission handles failure |

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| stuck_current_threshold | _per-motor TBD_ | High effort threshold |
| stuck_velocity_threshold | 0.05 m/s (tracks), 0.1 rad/s (joints) | Not-moving threshold |
| stuck_cycle_count | 20 cycles (100 ms at 200 Hz) | Consecutive cycles before triggering |
| gripper_force_delta_threshold | 5 N/cycle | Force increase rate indicating binding |

#### Orientation

| Condition | Telemetry | L0 Action | Escalate to L3 |
|---|---|---|---|
| Pitch beyond threshold | VN300 pitch | Kill drive | Abort mission |
| Roll beyond threshold | VN300 roll | Kill drive | Abort mission |
| Angular rate too high (tumbling) | VN300 angular rates | Kill everything | Abort mission |
| Sustained pitch + no velocity | VN300 pitch + ODrive velocity | Kill drive | Mission handles failure |

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| max_pitch | 35° | Kill drive threshold |
| max_roll | 35° | Kill drive threshold |
| max_angular_rate | 90°/s | Kill everything threshold |
| incline_stuck_pitch_min | 15° | Minimum pitch for incline-stuck check |

#### Proximity

L0 needs fast directional distance checks, not full voxel queries. The Python sidecar publishes pre-computed directional minimums (nearest occupied voxel in each cardinal direction + above + below) at perception loop rate to a shared memory channel. L0 reads these at 200 Hz; values are slightly stale (up to ~100 ms) which is acceptable for fail-safe clipping.

Full 3D collision checking lives in the Python sidecar and is queried by L1 services (MotionService, BodyAwarenessService, ArmService) — not by L0.

| Condition | Source | L0 Action | Escalate to L3 |
|---|---|---|---|
| Obstacle at hard limit (front) | Directional min front | Hard stop forward | — |
| Obstacle at hard limit (rear) | Directional min rear | Hard stop reverse | — |
| Obstacle at hard limit (side) | Directional min left/right | Block turn toward | — |
| Obstacle distance closing fast | Directional min rate of change | Proportional deceleration | — |
| Below clearance too small | Directional min below | Stop drive | — |
| Overhead clearance too small | Directional min above | Stop drive | — |

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| min_front_distance | 0.4 m | Hard stop front |
| min_rear_distance | 0.4 m | Hard stop rear |
| min_side_distance | 0.3 m | Block turn threshold |
| min_below_distance | 0.05 m | Chassis clearance minimum |
| min_above_distance | 0.1 m | Overhead clearance minimum |
| proximity_decel_start | 1.5 m | Begin proportional deceleration |
| proximity_decel_rate | linear | Deceleration curve factor |

#### Hardware Limits (Always Active)

| Limit | L0 Action |
|---|---|
| Max velocity | Clip to absolute cap |
| Max torque | Clip motor torque commands |
| Arm joint limits | Clip per-joint to URDF min/max angles |
| Gripper force limit | Cap force output |

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| max_velocity | 8.5 m/s | Absolute velocity cap |
| max_torque | _per-motor TBD_ | Absolute torque cap |
| arm_joint_limits | from URDF | Per-joint min/max angles |
| max_gripper_force | _TBD_ | Gripper force cap |

### L0 Escalation Summary

| Condition | L0 Action (immediate) | L3 Action (escalated) |
|---|---|---|
| Motor temp high | Derate velocity | Warn operator |
| Motor temp critical | Lock motor | Abort mission |
| Battery temp high | Derate all | Warn operator |
| Battery temp critical | Kill non-essential via PMCI | Abort mission |
| Battery low | Derate | Warn operator |
| Battery critical | Derate | Force ReturnHome |
| Current spike | Cap torque | — |
| Tracks stuck | Kill drive | Mission handles failure |
| Arm stuck | Freeze joint | Mission handles failure |
| Gripper stuck | Hold force | Mission handles failure |
| Pitch/roll exceeded | Kill drive | Abort mission |
| Tumbling | Kill everything | Abort mission |
| Incline stuck | Kill drive | Mission handles failure |
| Proximity hard limit | Hard stop in that direction | — |
| Proximity closing fast | Proportional deceleration | — |
| Below/Above clearance | Stop drive | — |

L0 applies in both teleop and autonomous mode. The operator cannot override safety limits.

---

## L1 — Services

Services consume telemetry, build a world model, and expose high-level actions. Missions call services — never hardware, never raw telemetry.

### Service Dependency Graph

```
Python Sidecar (Robotics Toolbox)   — canonical world model: URDF + voxel environment
  ↑ joint state subscription
  ↑ voxel delta subscription
  ↓ publishes link transforms, sensor poses, directional minimums, body envelope

SpatialEngine (Rust)                 — transport + cache: subscribes to sidecar publishes,
  ↓ exposes Rust API for other services        forwards request/response queries

MapService                           — Gaussian splat pipeline on Orin GPU
  → pushes voxel deltas to Python sidecar via iceoryx2

PositionService                      — Kalman fusion, GNSS integrity monitor
  → publishes position_state to blackboard, joint state to sidecar

CommsMonitor                         — link health, tri-state
SafePointService                     — maintains safe point registry
MotionService                        — drive_toward via SpatialEngine path queries
BodyAwarenessService                 — clearance + reconfiguration via SpatialEngine
ArmService                           — arm motion via SpatialEngine IK + trajectory
GripperService                       — gripper telemetry + commands
VisionService                        — consumes RT-DETR + NvDCF, fuses with lidar
StreamService                        — publishes deltas to comms layer
RecordingService                     — writes streams to disk for replay
```

The Python sidecar is the canonical world model. Every spatial-reasoning query (FK, IK, path planning, collision) goes through it. The Rust side is transport, caching, and consumption.

---

### Python Sidecar (Robotics Toolbox)

The world model. Holds the URDF and the live voxel environment. Wraps Peter Corke's [robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python). Communicates with Rust via iceoryx2.

**Responsibilities:**

- Load URDF as `ERobot.URDF` at startup
- Maintain the live voxel environment (updated by MapService voxel deltas)
- Compute FK for all links and sensor mounts on every joint state update
- Solve IK for arm targeting (damped least squares — handles unreachable targets gracefully)
- Plan body paths through the voxel environment
- Plan and smooth arm trajectories
- Check collisions: self-collision and environment-collision
- Maintain a joint state ring buffer (last 500 ms) for timestamp-aligned FK queries
- Compute and publish directional minimums for L0 ReflexEngine

**Subscribes (continuous):**

- Joint states from Rust (50–200 Hz depending on source — Pi telemetry gives 50 Hz arm + flippers, gripper at 200 Hz)
- Voxel deltas from MapService (10 Hz, at perception loop rate)

**Publishes (continuous, shared memory):**

- Link transforms / sensor poses for current joint state
- Directional minimums (nearest occupied voxel in 6 directions)
- Body collision envelope at current config

**Serves (request/response):**

- `solve_ik(target_pose, current_config?) → JointStates | NoSolution`
- `plan_body_path(from, to, avoid_zones?) → Path | NoPath`
- `plan_arm_trajectory(start_config, goal_config) → Trajectory | NoPath`
- `smooth_arm_trajectory(waypoints) → Trajectory`
- `check_self_collision(config) → bool`
- `check_environment_collision(config) → bool`
- `check_clearance(path_segment, config) → ClearanceResult`
- `compute_collision_envelope(config) → CollisionEnvelope`
- `get_collision_meshes(config) → List[Mesh]`
- `query_voxel_region(bbox) → List[Voxel]`
- `query_ray(origin, direction) → RayResult`
- `fk_at_timestamp(timestamp, sensor_id) → Pose` (uses ring buffer for past times)

**Performance targets:**

- IK solve: <10 ms per call (50 Hz teleop budget)
- Collision check: <5 ms per call
- Body path planning: <100 ms per call (infrequent, called when missions need a new plan)
- FK + sensor poses: continuous publish at joint state rate

**Failure modes:**

- IK unsolvable → returns no-solution + best-effort partial config (damped least squares output)
- Path planning timeout → returns no-path; caller's mission handles FAILURE
- Sidecar process death → Rust side detects via missing publishes within 100 ms, marks `sidecar_alive: false` on blackboard, ReflexEngine adds new rule that prevents driving without a fresh sidecar
- IPC timeout (default 50 ms per request) → call fails, caller handles

---

### SpatialEngine (Rust)

Rust-side transport and cache for the Python sidecar. Other Rust services see only the SpatialEngine — they don't know about the sidecar.

**Holds:**

- Local cache of latest sensor poses (refreshed by sidecar publishes via shared memory)
- Local cache of directional minimums (refreshed by sidecar publishes)
- Local cache of body collision envelope
- iceoryx2 publisher for joint states going TO the sidecar
- iceoryx2 subscriber for FK results FROM the sidecar
- iceoryx2 request/response client for query operations

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Refresh local cache from sidecar publishes | — |
| `get_sensor_pose(sensor_id)` | Cached world-frame pose of a sensor | `Pose` |
| `get_all_sensor_poses()` | All sensor transforms at once | `HashMap<SensorId, Pose>` |
| `get_link_pose(link_name)` | Cached world-frame pose of any link | `Pose` |
| `get_joint_states()` | Current joint positions | `JointStates` |
| `get_directional_minimums()` | Cached directional clearances | `DirectionalDistances` |
| `get_collision_envelope()` | Cached envelope at current config | `CollisionEnvelope` |
| `solve_ik(target_pose) → Future<Result>` | Async request to sidecar | `Future<JointStates>` |
| `plan_body_path(from, to) → Future<Result>` | Async request to sidecar | `Future<Path>` |
| `check_clearance(path_segment, config) → Future<Result>` | Async request to sidecar | `Future<ClearanceResult>` |
| `check_self_collision(config) → Future<Result>` | Async request to sidecar | `Future<bool>` |
| `query_voxel_region(bbox) → Future<Result>` | Async request to sidecar | `Future<List<Voxel>>` |
| `is_sidecar_alive()` | Health check | `bool` |

**Caching strategy:**

- FK (sensor poses, link poses) — cached locally, refreshed at sidecar publish rate (matches joint state input rate)
- Directional minimums — cached, refreshed at perception loop rate
- IK / planning / collision — never cached, always round-trip via request/response
- Voxel region queries — never cached (environment changes)

**Sensor Mount Registry (from URDF):**

| Sensor | Mount | Moves with |
|---|---|---|
| Livox Mid-360 #1 | Chassis | Robot body only |
| Livox Mid-360 #2 | Chassis | Robot body only |
| Camera #1–#4 | Chassis | Robot body only |
| Camera #5–#6 | Arm links | Arm joint states |
| Camera #7 | Gripper / end-effector | Arm + gripper |
| Camera #8 | _TBD mount_ | _TBD_ |
| VN300 | Chassis | Robot body only |
| Livox IMUs | Chassis | Robot body only |

---

### MapService

Owns the perception pipeline. Transforms raw sensor data into colored voxels and feeds them to the Python sidecar. Runs on the Jetson Orin GPU using CUDA. Does NOT hold the voxel map — the sidecar does. MapService is the ingestion pipeline.

**Inputs:**

| Source | Data | Rate |
|---|---|---|
| Livox Mid-360 #1 | 3D point cloud | 10 Hz |
| Livox Mid-360 #2 | 3D point cloud | 10 Hz |
| Cameras #1–#8 | RGB images (RTSP) | ~30 fps |
| SpatialEngine | Sensor-to-world transforms (timestamp-aligned via FK ring buffer) | Continuous |
| PositionService | Robot world-frame position + heading | 200 Hz |

**Pipeline (per tick at 10 Hz):**

```
1. For each Livox scan with timestamp T_lidar:
    Get robot pose at T_lidar from SpatialEngine (timestamp-aligned FK)
    Transform points from lidar frame → world frame
    Voxelize occupancy

2. For each camera frame at T_camera within alignment window of T_lidar:
    Get camera-frame pose at T_camera from SpatialEngine
    Get robot velocity at T_camera and T_lidar
    Compute motion-compensated transform: maps camera frame to where it would
      have been at T_lidar
    Project camera image rays into world space

3. Gaussian Splat Aggregation (CUDA kernels):
    For each pixel that projects onto a 3D point in voxel space:
      Generate a 3D Gaussian splat — color contribution to nearby voxels,
      weighted by distance from splat center
    Splat spread is a function of:
      - Robot velocity at observation (faster = wider Gaussian)
      - Distance from camera (farther = wider Gaussian)
      - Camera resolution
    Multiple cameras + frames contribute splats to the same voxel; weighted sum
    Frustum culling per camera before splatting (only voxels in FOV)
    Tile-based GPU aggregation

4. Occupancy + decay:
    Lidar ray-tracing for free-space marking
    Age out stale voxels not seen recently (configurable timeout)
    Mark voxels beyond map radius for removal

5. Push delta to Python sidecar via iceoryx2:
    Only changed voxels (additions, removals, color updates) — never full map
```

**Voxel structure (in MapService output, before sidecar ingestion):**

```rust
struct Voxel {
    position:           Vec3,
    occupied:           bool,
    color_sum:          Vec3,         // weighted sum, color = sum / weight
    weight_sum:         f32,
    observation_count:  u32,
    last_seen:          Timestamp,
    confidence:         f32,
    semantic_label:     Option<String>,  // from VisionService when applicable
}
```

**Speed-adaptive policy:**

At high robot velocity, motion blur and timestamp misalignment dominate. MapService adapts:

| Robot velocity | Camera subsampling | Splat spread multiplier |
|---|---|---|
| < 1 m/s | Full resolution, every frame | 1.0× |
| 1–3 m/s | Full resolution, every frame | 1.5× |
| 3–6 m/s | 1/2 resolution OR every-other-frame | 2.0× |
| > 6 m/s | 1/4 resolution OR every-third-frame | 3.0× |

At very high velocity, MapService can drop color updates entirely for that frame and only update occupancy from lidar — better to have geometry without color than wrong colors.

**Map frames:**

| GNSS status | Map frame | Behavior |
|---|---|---|
| TRUSTED | Global (UTM or lat/lon/alt) | Map is globally referenced |
| REJECTED / ABSENT | Local odometry frame | Anchored to `last_gnss_trusted` position |

When GNSS transitions from TRUSTED → REJECTED, the map continues in local frame. When GNSS is re-validated, the local map is re-anchored.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Run perception pipeline, push voxel delta to sidecar | — |
| `get_coverage()` | How much of the explored area is mapped | `CoverageStats` |
| `get_map_frame()` | Current reference frame | `FrameInfo` |

All voxel queries (region, ray, occupancy) go through SpatialEngine → sidecar.

**Configuration (starting defaults — calibrate from field testing):**

| Parameter | Value | Description |
|---|---|---|
| voxel_resolution | 0.10 m | Voxel size — coarser at first, refine after profiling |
| map_radius | 50 m | Active map radius from robot |
| map_height | 10 m | Vertical extent (5 m above, 5 m below) |
| max_voxels | 4M | Memory budget at 0.10 m — ~125 MB voxel storage |
| voxel_decay_time | 60 s | Stale voxel removal |
| alignment_window | 50 ms | Camera-lidar timestamp tolerance |
| frustum_cull | enabled | Skip voxels outside camera FOV |
| free_space_raytracing | enabled | Lidar rays clear voxels they pass through |
| update_rate | 10 Hz | Pipeline frequency |

**Compute target:** Jetson AGX Orin 32GB. CUDA kernels for splat aggregation, Tensor Cores where applicable for fused multiply-add. Run on CUDA streams independent of the vision inference stream so they don't serialize.

---

### PositionService

Owns position estimation, GNSS integrity monitoring, multi-source fusion. Single source of truth for "where is the robot." Runs at 200 Hz (matches IMU rate).

**Sensor Sources (4 independent):**

| Source | What it gives | Drift behavior | Spoofable |
|---|---|---|---|
| GNSS (VN300) | Global position, heading | None when honest | Yes — external spoofing, multipath |
| Fused IMU (Kalman: VN300 + 2× Livox IMU) | Orientation, angular rates, acceleration | Gyro bias drift over time | No |
| Wheel odometry (ODrive encoders) | Distance traveled, differential heading | Slip on loose terrain | No |
| Lidar odometry (2× Livox Mid-360 scan matching) | Frame-to-frame motion estimate | Best drift resistance of relative sources | No |

Three of four sources are unspoofable. Cross-validation of all four detects GNSS spoofing reliably.

#### GNSS Integrity Monitor

Maintains a dead-reckoning position (wheel + IMU + lidar odometry) in parallel with GNSS at all times. If they diverge, GNSS is lying.

| Detection | Signal | Threshold |
|---|---|---|
| Position jump | GNSS location change impossible given encoder velocity × elapsed time | 2 m / tick |
| Velocity mismatch | GNSS velocity vs encoder velocity | 1.0 m/s divergence |
| Heading mismatch | GNSS vs fused IMU vs differential track heading — GNSS disagrees with both others | 10° |
| Gradual drift (slow spoofing) | GNSS slowly diverges from dead reckoning over time | 0.5 m/min |
| Signal quality | Fix degrades (RTK → float → standalone → none), HDOP spikes | HDOP > 5 |

**Configuration values are starting defaults — calibrate from field testing.**

#### GNSS State Machine

```
TRUSTED ↔ SUSPECT → REJECTED → ABSENT
   ↑          ↓          ↓
   └──────────┘          ↓
   (re-validated)        ↓
   ↑                     ↓
   └─────────────────────┘
   (signal returns + passes integrity check over N cycles)
```

- `TRUSTED` — GNSS and dead reckoning agree. GNSS used in fusion.
- `SUSPECT` — Divergence detected, soft threshold. GNSS weight reduced. Alert operator.
- `REJECTED` — Hard threshold exceeded or spoofing pattern confirmed. GNSS excluded.
- `ABSENT` — No GNSS signal (underground, jammed).

Re-validation requires convergence over N cycles (default 100, ~0.5 s at 200 Hz) — single matching tick is not enough since spoofing can briefly align.

#### Position Fusion

GNSS TRUSTED — fuse all four sources, GNSS dominates for global accuracy. SUSPECT — reduce GNSS weight. REJECTED/ABSENT — wheel odometry + fused IMU + lidar odometry only.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Read all sources, integrity check, update fused position | — |
| `get_position()` | Best fused position estimate | `PositionEstimate` |
| `get_gnss_status()` | Current GNSS state | `TRUSTED / SUSPECT / REJECTED / ABSENT` |
| `get_confidence()` | Position confidence (degrades without GNSS) | `f32 (0.0–1.0)` |
| `get_drift_estimate()` | Accumulated error since last GNSS-trusted fix | `f32 (meters)` |
| `get_heading()` | Fused heading | `f32 (degrees)` |
| `get_velocity()` | Fused velocity estimate | `Velocity` |

```rust
struct PositionEstimate {
    coordinate:      Coordinate,
    confidence:      f32,
    drift_estimate:  f32,
    gnss_status:     GnssStatus,
    timestamp:       Timestamp,
    source_breakdown: SourceContributions,
}
```

**Joint State Forwarding:** PositionService publishes the fused robot pose to the Python sidecar via iceoryx2 alongside Pi joint state telemetry. The sidecar uses these to update the URDF base link transform.

#### Drift Budget

| Drift estimate | System behavior |
|---|---|
| < 2 m | Normal operations, all missions available |
| 2–10 m | Warn operator, reduce GoTo precision expectations |
| 10–25 m | Block new long-range GoTo (>100m), prefer relative navigation |
| > 25 m | Only relative missions allowed, recommend ReturnHome to `last_gnss_trusted` |

---

### CommsMonitor

Watches link state, maintains tri-state quality metric. Publishes to blackboard.

**Inputs:** Comms link telemetry (RSSI, packet loss, latency, last operator heartbeat).

**States:**

| State | Meaning | Description |
|---|---|---|
| HEALTHY | Microhard or equivalent high-bandwidth link active | Full operations, video, full telemetry |
| DEGRADED | LoRa or fallback link only | Commands and minimal telemetry only, no video |
| LOST | No link at all | Robot is on its own |

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Sample link state, update `comms_status` on blackboard | — |
| `get_state()` | Current tri-state | `CommsState` |
| `time_since_last_heartbeat()` | Seconds since last operator packet | `f32` |
| `get_signal_quality()` | Quality metric | `f32 (0.0–1.0)` |

**Blackboard writes:** `comms_status: { state, rssi, packet_loss, latency, last_heartbeat_age, quality }`

**State transition rules:**

- HEALTHY → DEGRADED: high-bandwidth link unavailable but fallback link active
- DEGRADED → LOST: no heartbeat from operator within `heartbeat_timeout` (default 5 s)
- LOST → DEGRADED: heartbeat returns
- DEGRADED → HEALTHY: high-bandwidth link returns AND quality stable for N seconds

Hysteresis on transitions prevents thrashing.

**Configuration (starting defaults):**

| Parameter | Value | Description |
|---|---|---|
| heartbeat_timeout | 5 s | Seconds before declaring LOST |
| min_rssi_healthy | _TBD per radio_ | RSSI threshold for HEALTHY |
| min_rssi_degraded | _TBD per radio_ | RSSI floor for DEGRADED |
| stable_recovery_window | 3 s | Quality must hold this long to upgrade state |

---

### SafePointService

Owns the safe point registry. Auto-updates from PositionService and CommsMonitor state. Operator-set points come from `SetSafePoint` mission via the Command Router.

**Inputs (auto-update sources):**

| Source | Triggers update of |
|---|---|
| PositionService (TRUSTED GNSS) | `last_gnss_trusted` |
| PositionService (stable orientation, clear surroundings, high confidence) | `last_stable` |
| CommsMonitor (HEALTHY) | `last_comms` |
| VisionService (model running) | `last_vision` |
| CommandRouter (on deploy) | `origin` (once) |
| `SetSafePoint` mission (operator) | `operator[]` |

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Sample sources, update auto-managed points on blackboard | — |
| `add_operator_point(label)` | Record current position as named operator point | `SafePoint` |
| `remove_operator_point(label)` | Remove a named operator point | `bool` |
| `resolve(strategy, candidates, current_position)` | Pick best safe point per strategy | `SafePoint` |
| `get_all()` | Full registry | `SafePointRegistry` |

**Resolution strategies (registered, extensible):**

- `specific(key)` — go to a named safe point key
- `nearest(candidates)` — geographically nearest from candidate list
- `freshest(candidates)` — most recently updated
- `best(candidates)` — score by distance × age × confidence

New strategies can be registered without changing existing fallback rules.

**Stable-ground detection (for `last_stable`):** Pitch and roll within thresholds, all flippers below an extension threshold, no active proximity reflex flags, position confidence above a threshold. Sampled every 5 s; overwrites the previous `last_stable` only if the new candidate is meaningfully different (avoid thrashing on small motions).

**Blackboard writes:** `safe_points` (full registry, see Blackboard section)

---

### MotionService

Path planning orchestration, flipper coordination, drive command generation. Uses SpatialEngine (which calls the Python sidecar) for path planning and clearance, PositionService for position, BodyAwarenessService for reconfiguration decisions.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Update local planning state, append to path_history | — |
| `drive_toward(coordinate)` | Plan via sidecar, drive toward target. Returns `NEEDS_RECONFIGURE` if body config doesn't fit. | `RUNNING / SUCCESS / FAILURE / NEEDS_RECONFIGURE(target_config)` |
| `stop()` | Zero velocity, brake | — |
| `reverse(distance)` | Drive backward along path_history | `RUNNING / SUCCESS / FAILURE` |
| `orbit(center, radius)` | Circular path around a point | `RUNNING / SUCCESS / FAILURE` |
| `set_teleop_velocities(left, right, flippers)` | Direct velocity commands in teleop mode | — |

**Internal concerns (invisible to missions):**

- Position from `PositionService.get_position()`
- Path planning via `SpatialEngine.plan_body_path()` → sidecar
- Clearance from `SpatialEngine.check_clearance()` → sidecar
- If clearance fails → `BodyAwarenessService.suggest_reconfiguration()` → return `NEEDS_RECONFIGURE` to orchestrator (not handled internally)
- Flipper coordination based on pitch/roll/terrain
- AvoidZone geofences from mission blackboard
- Stair/gap traversal
- Arrival threshold adjusted by `PositionService.get_drift_estimate()`
- Maintains `path_history` on system blackboard — appends each significant pose change for Retreat and BacktrackComm

**Drive-through-narrow-space sequence (internal):**

```
drive_toward(target):
    current_pos = position_service.get_position()
    path = spatial_engine.plan_body_path(current_pos, target).await
    if path is None:
        return FAILURE

    next_segment = path[0]
    clearance = spatial_engine.check_clearance(next_segment, current_config).await

    if clearance.fits:
        execute_drive(next_segment)
        return RUNNING

    # Doesn't fit — ask BodyAwarenessService for a reconfig
    suggestion = body_awareness.suggest_reconfiguration(next_segment)
    if suggestion is not None:
        return NEEDS_RECONFIGURE(suggestion.config)

    # No reconfig works — replan
    avoid = blocking_voxels_to_geometry(clearance.blocking_voxels)
    replan = spatial_engine.plan_body_path(current_pos, target, extra_avoid=avoid).await
    if replan is None:
        return FAILURE
    path = replan
    return RUNNING
```

The orchestrator sees `NEEDS_RECONFIGURE` and inserts a Reconfigure sub-mission. See L3 — Reconfiguration Handling.

---

### BodyAwarenessService

Thin query layer on top of SpatialEngine. Answers "does the robot fit?" and "what should I reconfigure to fit?" Does not own geometry or the voxel map — the Python sidecar does.

**Dependencies:**

| Service | What it provides |
|---|---|
| SpatialEngine | `check_clearance()`, `check_self_collision()`, `get_collision_envelope()` (all routed to Python sidecar) |

**How it works:**

1. Query `SpatialEngine.check_clearance(path_segment, current_config)` → sidecar
2. If doesn't fit → iterate known configs, query `SpatialEngine.check_clearance(path_segment, candidate_config)` for each → sidecar
3. Return best reconfiguration option or None

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Refresh cached body_state from SpatialEngine, update blackboard | — |
| `check_clearance(path_segment) → Future<Result>` | Can the robot traverse this segment in current config? | `Future<ClearanceResult>` |
| `check_clearance_at(path_segment, config) → Future<Result>` | Can it traverse in a hypothetical config? | `Future<ClearanceResult>` |
| `suggest_reconfiguration(path_segment) → Future<Result>` | What config changes would let the robot fit? | `Future<Option<ReconfigSuggestion>>` |
| `get_collision_envelope()` | Current 3D bounding volume (cached from sidecar) | `CollisionEnvelope` |
| `get_config()` | Current body configuration | `BodyConfig` |

**Data types:**

```rust
struct ClearanceResult {
    fits: bool,
    min_clearance: f32,
    blocking_voxels: Vec<VoxelId>,
    can_reconfigure: bool,
    suggested_config: Option<BodyConfig>,
}

struct ReconfigSuggestion {
    config: BodyConfig,
    actions: Vec<ReconfigAction>,        // ["arm_stow", "flippers_flat"]
    estimated_clearance: f32,
    reconfiguration_time: f32,           // seconds
}

struct BodyConfig {
    flipper_angles: [f32; 4],            // fl, fr, bl, br
    arm_joint_states: [f32; 6],
    gripper_position: f32,
}
```

**Known configurations (precomputed from URDF):**

| Config | Description | Use case |
|---|---|---|
| `travel` | Arm stowed, flippers flat, gripper closed | Minimum profile |
| `home` | Arm home, flippers default | Default working profile |
| `compact` | Arm stowed, flippers tucked inward | Narrowest width — doorways |
| `low` | Arm stowed, flippers flat | Lowest height — crawlspaces/overhangs |
| `climb` | Arm stowed, flippers extended | Stair/obstacle configuration |

Reconfiguration search checks known configs first (fast lookup) before any general search.

---

### ArmService

Arm motion execution and joint interpolation. Uses SpatialEngine (→ Python sidecar) for IK and trajectory planning.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Run 75 Hz arm control loop, interpolate toward latest target | — |
| `move_to_joint_state(joints) → Future` | Plan trajectory via sidecar, execute | `RUNNING / SUCCESS / FAILURE` |
| `stow() → Future` | Move to stow configuration | `RUNNING / SUCCESS / FAILURE` |
| `home() → Future` | Move to home configuration | `RUNNING / SUCCESS / FAILURE` |
| `reach_toward(position) → Future` | IK via sidecar → trajectory → execute | `RUNNING / SUCCESS / FAILURE` |
| `set_teleop_target(end_effector_pose)` | Set end-effector target for teleop loop | — |
| `is_stowed()` | Check if arm is in stow position | `bool` |

**Teleop loop (internal):** When in teleop mode, ArmService runs at 75 Hz reading the latest end-effector target, calling `SpatialEngine.solve_ik(target)` (which round-trips to the sidecar in <10 ms), and commanding the resulting joint states. The sidecar's damped least squares IK provides best-effort solutions when the target is outside the workspace — arm follows operator intent up to the boundary.

**Internal concerns:**

- Self-collision checking via `SpatialEngine.check_self_collision()` before any commanded joint state
- Trajectory smoothing via sidecar for autonomous arm moves
- Force/torque limits enforced by L0 ReflexEngine

---

### GripperService

Gripper command execution at 200 Hz.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Run 200 Hz gripper control loop | — |
| `open()` | Open gripper | `RUNNING / SUCCESS / FAILURE` |
| `close(force_limit)` | Close until force threshold | `RUNNING / SUCCESS / FAILURE` |
| `is_gripping()` | Check if holding something | `bool` |
| `set_teleop_state(state)` | open/close command in teleop | — |

---

### VisionService

Wraps the always-running vision pipeline. Receives 2D detections + tracking from upstream, fuses with lidar to produce 3D-positioned tracked objects.

**Vision Pipeline (external, runs as separate processes):**

```
8 cameras (RTSP)
  → Per-camera RT-DETR inference (~20 fps each)
  → Per-camera NvDCF tracker (assigns persistent track IDs within each camera)
  → WebSocket stream of {camera_id, track_id, bbox, class, confidence, timestamp}
  → VisionService consumer
```

**VisionService Responsibilities:**

- Subscribe to the WebSocket detection stream
- For each detection, fuse with lidar to produce 3D position:
  - Project the 2D bbox to a 3D viewing frustum (using camera intrinsics + sensor pose at detection timestamp from SpatialEngine)
  - Query lidar points within the frustum at matching timestamp (motion-compensated)
  - Cluster the lidar points; the densest cluster's centroid gives 3D position
  - Cluster lateral extent gives object size
- Cross-camera deduplication: same physical object seen by multiple cameras gets a unified `object_id`. Use 3D position proximity + classification matching, with hysteresis to avoid flickering.
- Track persistence: when an upstream tracker drops a track, keep the unified object marked as "occluded" with a configurable timeout. Re-associate if it reappears within the window with a consistent 3D position.
- Velocity estimation: maintain a short history per unified object (last N positions + timestamps), compute velocity as a low-pass-filtered finite difference.

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Process detection stream, update active object list | — |
| `get_object(object_id)` | Get tracked object by unified ID | `Option<Object>` |
| `get_objects_by_type(type)` | Filter by classification | `Vec<Object>` |
| `get_nearest(type)` | Nearest object of a type | `Option<Object>` |

```rust
struct Object {
    id:               String,        // unified across cameras
    type_:            String,        // person, car, drone, robot, poi
    position:         Coordinate,    // world frame
    size:             Vec3,          // bbox extents
    distance:         f32,
    velocity:         Velocity,
    camera_feed_ids:  Vec<String>,   // which cameras currently see it
    confidence:       f32,
    last_seen:        Timestamp,
    is_occluded:      bool,          // upstream tracker lost it but still in window
}
```

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| occlusion_timeout | 2 s | How long to keep an occluded object before removing |
| dedup_distance_threshold | 1.0 m | Same physical object if within this distance + same class |
| velocity_filter_window | 5 frames | History size for velocity estimation |

---

### StreamService

Publishes full state at full rate to the comms layer below the autonomy stack. Does NOT handle bandwidth adaptation, delta encoding, or LoRa fallback — those live in the comms layer.

**Subscribes:**

- SpatialEngine: voxel deltas (via Python sidecar publishes), robot pose (joint states), sensor poses
- VisionService: identified objects
- Blackboard: poi_map, safe_points, path_history, avoid_zones, comms_status, position_state, reflex_flags, thermal_state, battery_state, body_state
- Orchestrator: active mission tree state and transitions
- ReflexEngine: reflex flag changes
- Camera streams (selectable per camera)

**Publishes:**

| Stream | Source | Content | Default rate |
|---|---|---|---|
| voxel_delta | sidecar via SpatialEngine | New, changed, removed voxels (colored) | 5–10 Hz |
| robot_pose | SpatialEngine | Full joint states for URDF rendering | 50 Hz |
| identified_objects | VisionService | Object list with ID, type, position, bbox, distance | Every change |
| poi_map | Blackboard | All POI pins with tags | On change |
| safe_points | Blackboard | Safe point registry | On change |
| path_history | Blackboard | Breadcrumb trail | Throttled |
| avoid_zones | Blackboard | No-go geometries | On change |
| active_mission | Orchestrator | Mission tree, state, precondition status | On change |
| reflex_flags | ReflexEngine | Active reflexes, derate levels | On change |
| position_state | PositionService | Fused position, confidence, drift, GNSS status | 50 Hz |
| comms_status | CommsMonitor | Tri-state link status | On change |
| battery_state | Blackboard | Voltage, percentage, temp | 1 Hz |
| thermal_state | Blackboard | Temps + derate levels | 1 Hz |
| camera_feeds | Cameras | Compressed video (selectable) | Configurable |

**Exposes:**

| Method | Description | Returns |
|---|---|---|
| `tick()` | Collect updates, publish frames to comms layer | — |
| `subscribe(stream_id, rate)` | Comms layer subscribes to specific streams | — |
| `get_stream_stats()` | Per-stream rates and queue depths for diagnostics | `StreamStats` |

The comms layer below subscribes to whichever streams it wants, then handles delta encoding, compression, throttling, and bandwidth-mode selection. StreamService stays oblivious to all of that.

---

### RecordingService

Writes streams to disk for replay. Runs alongside StreamService, subscribes to the same streams at full rate (not throttled).

**Records:**

- All StreamService streams at full rate
- Raw telemetry snapshots
- Mission tree state transitions
- Preemption / reflex events
- GNSS integrity state transitions
- Sidecar request/response events with timing (for performance profiling)

**Log format:**

```rust
struct MissionLog {
    header: LogHeader {
        robot_id:     String,
        mission_id:   String,
        start_time:   Timestamp,
        urdf_hash:    String,
        firmware_version: String,
    },
    frames: Vec<Frame>,
}

struct Frame {
    timestamp:           u64,
    sequence:            u64,
    voxel_delta:         Bytes,
    robot_joint_states:  JointStates,
    position_state:      PositionEstimate,
    identified_objects:  Vec<Object>,
    poi_map:             Vec<POI>,
    reflex_flags:        ReflexFlags,
    mission_state:       MissionState,
    operator_command:    Option<OperatorCommand>,
    sidecar_metrics:     SidecarMetrics,
}
```

**Configuration:**

| Parameter | Value | Description |
|---|---|---|
| log_directory | _TBD_ | Where to write log files |
| max_log_size | 10 GB | Rotate after this size |
| rotation_strategy | per_mission | New file per mission run |


---

## L2 — Preconditions (PreconditionEngine)

Declarative metadata on each mission node. Uses the policy engine pattern — precondition types are registered, not hardcoded.

### Extensible Precondition Types

```rust
struct PreconditionType<V> {
    id:        &'static str,
    evaluate:  Box<dyn Fn(&V, &State) -> bool>,
    default:   Option<V>,
}

// Register a default type
precondition_engine.register_type(PreconditionType {
    id: "min_battery",
    evaluate: Box::new(|value: &f32, state: &State| state.battery.percent >= *value),
    default: None,
});

// Register a new type at runtime — e.g., new sensor
precondition_engine.register_type(PreconditionType {
    id: "requires_daylight",
    evaluate: Box::new(|value: &f32, state: &State|
        state.ambient_light.lux >= *value),
    default: None,
});
```

Adding a new sensor or constraint = registering a new type. No existing missions change. Mission-level precondition values are checked on start; `requires_*` boolean fields are also checked each tick as invariants.

### Default Precondition Types

| Type ID | Value type | Evaluation |
|---|---|---|
| `min_battery` | f32 | `state.battery.percent >= value` |
| `requires_comms` | enum {HEALTHY, DEGRADED} | `state.comms.state >= value` |
| `requires_arm_stowed` | bool | `state.body_state.is_arm_stowed == value` |
| `min_position_confidence` | f32 | `state.position.confidence >= value` |
| `max_drift` | f32 | `state.position.drift_estimate <= value` |
| `max_pitch` | f32 | `state.position.pitch.abs() <= value` |
| `max_runtime` | f32 | `mission.runtime <= value` |

See Mission Specification for the per-mission precondition table and failure handlers.

### Behavior

- **Pre-start check**: all preconditions evaluated against current state when `StartMission` is received. If any fail, `MissionRejected` returned with reason.
- **Runtime invariant check**: `requires_*` and `min_*` / `max_*` re-evaluated each tick by PreemptionEngine as a `precondition_violated` rule. Violation triggers preemption with the mission's failure handler.

---

## Mission Framework

The behavior tree primitives, lifecycle, and configuration format.

### Mission Trait

Every mission — leaf, compound, internal primitive — implements:

```rust
enum Status {
    Running,
    Success,
    Failure { reason: String },
    Cancelled,
    NeedsReconfigure { target_config: BodyConfig },  // MotionService only
}

trait Mission: Send {
    fn tick(&mut self,
            state: &State,
            mission_blackboard: &mut Blackboard,
            cancel_token: &CancelToken) -> Status;

    fn preconditions(&self) -> &[Precondition];
    fn failure_handler(&self) -> Option<&MissionDescriptor>;
    fn id(&self) -> &str;
}
```

### Cancellation

Two cancellation modes:

**`abort` (hard kill):**
- Orchestrator stops ticking the tree immediately
- L0 ReflexEngine clamps motors to safe state
- No leaf cleanup
- Fastest path, used for E-stop, tumbling detection, comms-critical preemptions

**`cancel` (cooperative):**
- Cancel token propagates down the tree
- Each leaf checks `cancel_token.is_cancelled()` at decision points and runs its cleanup path before returning `CANCELLED`
- Compound nodes propagate the token to running children; wait for child to return CANCELLED, then return CANCELLED themselves
- Used for operator cancel, mission timeout, lower-priority preemptions

**Auto-escalation:** If a `cancel` doesn't return within `cancel_timeout` (default 5 s), the orchestrator escalates to `abort`. This guarantees fallback missions always get to run, even if a leaf is stuck in cleanup.

```rust
struct CancelToken {
    state: AtomicState,  // RUNNING / CANCEL_REQUESTED / ABORT_REQUESTED
}

impl CancelToken {
    fn is_cancelled(&self) -> bool { /* ... */ }
    fn is_aborted(&self) -> bool { /* ... */ }
}
```

### Composite Nodes

| Composite | Children policy | Description |
|---|---|---|
| `Sequence` | Tick first child, advance on SUCCESS, fail on first FAILURE | Standard sequential execution |
| `Selector` | Tick first child, advance on FAILURE, succeed on first SUCCESS | Standard fallback execution |
| `Loop` | Tick child, restart on completion, until cancel or condition | Repeating behavior |
| `Parallel` | See policy below | Multiple children simultaneously |

### Parallel Composite — Configurable Policy

Parallel takes a per-instance policy parameter:

| Policy | Success condition | Failure condition |
|---|---|---|
| `require_all` | All children SUCCESS | First child FAILURE → cancel siblings, return FAILURE |
| `require_any` | First child SUCCESS → cancel siblings, return SUCCESS | All children FAILURE → return FAILURE |
| `fail_fast` | All children SUCCESS | First child FAILURE → cancel siblings, return FAILURE (synonym for require_all) |
| `succeed_fast` | First child SUCCESS → cancel siblings, return SUCCESS | All children FAILURE → return FAILURE (synonym for require_any) |

Example: `SurveyArea = Parallel(policy=require_all, HoldPosition, CameraSweep, ListenPing)` — all three must complete cleanly.

### Decorators

Wrap a single child mission with modified behavior.

| Decorator | Behavior |
|---|---|
| `AvoidZone(geometry)` | Writes geometry to mission blackboard, child mission sees it via path planner. Cleaned up automatically when mission ends. |
| `Timeout(seconds)` | Cancels child after timeout, returns FAILURE. |
| `Retry(max_attempts)` | Retries child on FAILURE up to max_attempts times. |

Decorators are first-class composite types — registered like other composites and usable in config.

### Tree Definition

**Primitives are defined in code.** Every leaf mission and composite type is a Rust type with strict signatures.

**Compound mission trees are defined in RON config files.** Schema-validated at load time.

Example config:

```ron
Mission(
    id: "Sentinel",
    params: [
        Param(name: "coordinate_a", type: Coordinate),
        Param(name: "coordinate_b", type: Coordinate),
    ],
    preconditions: {
        "min_battery": 30,
        "requires_arm_stowed": true,
        "min_position_confidence": 0.5,
        "max_drift": 25,
    },
    failure_handler: Some("HoldPosition"),
    tree: Loop(
        Sequence([
            Leaf("GoTo", { "coordinate": $coordinate_a }),
            Leaf("GoTo", { "coordinate": $coordinate_b }),
        ])
    ),
)
```

**Validation pass at load time:**

- Do referenced primitives exist in the registry?
- Do parameter types match what each primitive expects?
- Do referenced safe point keys exist?
- Are all precondition types registered?
- Is the failure handler a valid mission?

Bad trees fail to load with clear errors. They never crash mid-mission.

**Loading paths:**

- Built-in trees registered at startup, compiled into the firmware
- Custom trees loaded from a missions directory, hot-reloadable on operator command
- Both available to the operator through the same mission catalog

### Blackboards

Two blackboards. Different lifetimes, different scopes.

**System blackboard** — global, lives for the duration of the robot session. Owned by services, read by anyone.

| Key | Writer | Description |
|---|---|---|
| position_state | PositionService | Fused position, confidence, drift, GNSS status |
| comms_status | CommsMonitor | Tri-state link status |
| reflex_flags | ReflexEngine | Active reflexes |
| thermal_state | ReflexEngine | Per-motor and battery temps + derate levels |
| battery_state | ReflexEngine | Voltage, percentage, temp, cell health |
| body_state | BodyAwarenessService | Current config, collision envelope (cached from sidecar) |
| safe_points | SafePointService | Full registry |
| path_history | MotionService | Breadcrumb trail of positions |
| poi_map | MarkPOI mission, Explore mission | Persistent across missions |
| active_objects | VisionService | Currently tracked objects |
| sidecar_alive | SpatialEngine | Health of Python sidecar process |

**Mission blackboard** — scoped to a single mission tree's lifetime. Created when orchestrator starts the mission, destroyed when the mission returns SUCCESS / FAILURE / CANCELLED.

Used for:

- Tree-internal state (Explore tracks visited waypoints, PickUp tracks approach phase)
- Decorator-set values that should auto-clean (AvoidZone writes here)
- Intermediate data shared between leaves of a compound mission

**Access pattern:** Each leaf gets `(state, mission_blackboard, cancel_token)` on tick. State includes a read-only view of the system blackboard plus a telemetry snapshot. Mission blackboard is read-write, scoped to the running tree.

**Sub-mission scope:** When a compound mission invokes a child compound (e.g., Sentinel → Sequence → GoTo), they share the same mission blackboard, not nested. Naming convention: namespace keys with the primitive's name (e.g., `explore.visited_waypoints`, `pickup.approach_phase`) to avoid collisions.

**System keys are read-only from missions.** Missions cannot write to position_state, reflex_flags, etc. Those are owned by their respective services.

---

## L3 — Orchestrator (PreemptionEngine)

Owns the running tree, ticks it, handles preemption, watchdogs, fallback policies, and the Reconfigure sub-mission insertion logic. All preemption behavior driven by the PreemptionEngine — a policy rule registry.

**Responsibilities:**

- Start missions after PreconditionEngine check
- Tick the active behavior tree each cycle (50 Hz)
- Tick the PreemptionEngine each cycle
- Insert Reconfigure sub-missions when MotionService returns NEEDS_RECONFIGURE
- Run failure handlers when missions return FAILURE
- Run fallback behaviors selected by preemption rules
- Watchdog: kill hung missions past their `max_runtime`
- Auto-escalate stuck `cancel` to `abort` after timeout

### Reconfiguration Handling

When a leaf returns `NEEDS_RECONFIGURE(target_config)`:

1. Orchestrator pauses the active mission tree (saves state, stops ticking)
2. Inserts a synthetic `Reconfigure(target_config)` sub-mission
3. Reconfigure runs as a regular mission with its own preconditions and preemption rules
4. While Reconfigure runs, all standard preemption rules apply — battery critical, comms lost, operator cancel all preempt normally
5. Operator sees `Mission paused, Reconfiguring (stow arm, flatten flippers)` in the active_mission stream
6. On Reconfigure SUCCESS:
   - Original mission resumes
   - Replans from scratch via `SpatialEngine.plan_body_path()` (world may have changed during the 5+ s reconfiguration)
7. On Reconfigure FAILURE: original mission inherits the failure — its failure handler runs
8. On Reconfigure CANCELLED: same as a regular cancellation propagates up

### Default Preemption Rules

Rules loaded from configuration. Add, remove, or re-prioritize without code changes.

```ron
preemption_rules: [
    // Priority 100 — Operator
    Rule(
        id: "operator_estop",
        condition: "state.operator.estop == true",
        priority: 100,
        action: HardStop,
        escalate: None,  // L0 handles directly
    ),
    Rule(
        id: "operator_cancel",
        condition: "state.operator.cancel_requested(active_mission.run_id)",
        priority: 99,
        action: CancelMission,
        escalate: None,
    ),
    Rule(
        id: "operator_abort",
        condition: "state.operator.abort_requested(active_mission.run_id)",
        priority: 99,
        action: HardStop,
        escalate: None,
    ),

    // Priority 90 — Physics
    Rule(
        id: "tumbling",
        condition: "state.reflex_flags.tumbling == true",
        priority: 90,
        action: AllStop,
        escalate: Some(AbortMission { reason: "Tumbling detected" }),
    ),
    Rule(
        id: "pitch_roll_exceeded",
        condition: "state.reflex_flags.pitch_exceeded || state.reflex_flags.roll_exceeded",
        priority: 85,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Orientation exceeded" }),
    ),

    // Priority 80 — Power
    Rule(
        id: "battery_critical",
        condition: "state.battery.percent < thresholds.battery_critical",
        priority: 80,
        action: FallbackToSafePoint {
            resolve: "nearest",
            candidates: ["operator", "last_stable"],
        },
        escalate: Some(AbortMission { reason: "Battery critical" }),
    ),

    // Priority 70 — Thermal
    Rule(
        id: "thermal_critical",
        condition: "state.reflex_flags.any_motor_critical || state.reflex_flags.battery_temp_critical",
        priority: 70,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Thermal critical" }),
    ),

    // Priority 60 — Comms
    Rule(
        id: "comms_lost",
        condition: "state.comms.state == LOST && active_mission.preconditions.requires_comms != None",
        priority: 60,
        action: FallbackToSafePoint {
            resolve: "specific",
            key: "last_comms",
            navigate: "global_or_retrace",
        },
        escalate: Some(AbortMission { reason: "Comms lost" }),
    ),
    Rule(
        id: "comms_degraded_for_healthy_mission",
        condition: "state.comms.state == DEGRADED && active_mission.preconditions.requires_comms == HEALTHY",
        priority: 58,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Mission requires healthy comms" }),
    ),

    // Priority 55 — GNSS
    Rule(
        id: "gnss_rejected",
        condition: "state.position.gnss_status in [REJECTED, ABSENT]
                    && active_mission.preconditions.min_position_confidence
                       > state.position.confidence",
        priority: 55,
        action: FallbackToSafePoint {
            resolve: "specific",
            key: "last_gnss_trusted",
            navigate: "path_retrace_only",
        },
        escalate: Some(AbortMission { reason: "GNSS denied, position degraded" }),
    ),
    Rule(
        id: "gnss_suspect",
        condition: "state.position.gnss_status == SUSPECT",
        priority: 30,
        action: WarnOperator { message: "GNSS suspect — reduced weight" },
        escalate: None,
    ),
    Rule(
        id: "drift_exceeded_mission",
        condition: "state.position.drift_estimate > active_mission.preconditions.max_drift",
        priority: 50,
        action: FallbackToSafePoint {
            resolve: "specific",
            key: "last_gnss_trusted",
            navigate: "path_retrace_only",
        },
        escalate: Some(AbortMission { reason: "Drift budget exceeded" }),
    ),
    Rule(
        id: "drift_exceeded_global",
        condition: "state.position.drift_estimate > thresholds.max_global_drift",
        priority: 45,
        action: RestrictMissions { allow: "relative_only" },
        escalate: Some(WarnOperator { message: "Drift too high, global nav restricted" }),
    ),

    // Priority 40 — Sidecar health
    Rule(
        id: "sidecar_dead",
        condition: "state.sidecar_alive == false",
        priority: 88,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Python sidecar unresponsive" }),
    ),

    // Priority 40 — Watchdog
    Rule(
        id: "mission_timeout",
        condition: "active_mission.runtime > active_mission.preconditions.max_runtime",
        priority: 40,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Mission timeout" }),
    ),

    // Priority 35 — Precondition violation
    Rule(
        id: "precondition_violated",
        condition: "precondition_engine.check(active_mission, state) == false",
        priority: 35,
        action: HoldPosition,
        escalate: Some(AbortMission { reason: "Precondition invariant violated" }),
    ),
]
```

### Action Types

Registered and extensible.

```rust
enum Action {
    HardStop,                              // L0 immediate stop, bypass tree
    AllStop,                               // L0 kill everything
    HoldPosition,                          // Start HoldPosition mission
    CancelMission,                         // Cooperative cancel of active tree
    FallbackToSafePoint {
        resolve:   ResolutionStrategy,
        candidates: Option<Vec<String>>,    // for nearest/freshest/best
        key:       Option<String>,           // for specific
        navigate:  NavigationMode,
    },
    WarnOperator { message: String },       // Push notification, no mission change
    RestrictMissions { allow: MissionSet }, // Update allowed mission set
    AbortMission { reason: String },        // Kill current tree (escalation)
}

enum ResolutionStrategy {
    Specific,           // resolve by key
    Nearest,            // by distance from current position
    Freshest,           // most recently updated
    Best,               // distance × age × confidence
}

enum NavigationMode {
    GlobalOrRetrace,    // Plan global path, fall back to retrace if planning fails
    PathRetraceOnly,    // Follow path_history breadcrumbs only — no global planning
}
```

### Fallback Priority Summary

Derived from rule priorities — not hardcoded, just the result of evaluating the default ruleset.

| Priority | Condition | Action |
|---|---|---|
| 100 | Operator E-stop | Hard stop |
| 99 | Operator Cancel/Abort | Cancel or hard stop |
| 90 | Tumbling | All stop |
| 88 | Sidecar dead | HoldPosition |
| 85 | Pitch/roll exceeded | HoldPosition |
| 80 | Battery critical | GoTo(nearest safe point) |
| 70 | Thermal critical | HoldPosition |
| 60 | Comms lost | GoTo(`last_comms`) |
| 58 | Comms degraded for healthy-required mission | HoldPosition |
| 55 | GNSS rejected | GoTo(`last_gnss_trusted`) via retrace |
| 50 | Drift exceeds mission limit | GoTo(`last_gnss_trusted`) via retrace |
| 45 | Drift exceeds global limit | Restrict to relative missions |
| 40 | Mission timeout | HoldPosition |
| 35 | Precondition violated | HoldPosition |
| 30 | GNSS suspect | Warn operator |

### Mission Failure (Not Preempted)

When a mission tree returns `FAILURE` (genuinely failed, not preempted, not cancelled), the orchestrator:

1. Logs the failure with reason
2. Notifies the operator with `MissionEnded(run_id, status: FAILURE, reason)`
3. Looks up the mission's `failure_handler`:
   - If declared → instantiates and starts the failure handler as a new mission run with its own run_id
   - If not declared → starts `HoldPosition` as the universal fallback
4. The failure handler runs through the same orchestrator with its own preconditions and preemption rules

Operators can override the failure handler per-invocation via `StartMission(failure_handler_override)`.


---

## Mission Integration

Missions are thin. All complexity lives in services and policy engines.

**Example: GoTo (leaf, code-defined):**

```rust
struct GoTo {
    target: Coordinate,
    motion: Arc<MotionService>,
}

impl GoTo {
    const PRECONDITIONS: &'static [Precondition] = &[
        Precondition::MinBattery(20.0),
        Precondition::MinPositionConfidence(0.5),
        Precondition::MaxDrift(25.0),
        Precondition::RequiresArmStowed(true),
    ];
}

impl Mission for GoTo {
    fn tick(&mut self, _state: &State, _bb: &mut Blackboard, cancel: &CancelToken) -> Status {
        if cancel.is_cancelled() {
            self.motion.stop();
            return Status::Cancelled;
        }

        match self.motion.drive_toward(self.target) {
            DriveStatus::Running => Status::Running,
            DriveStatus::Success => Status::Success,
            DriveStatus::Failure(reason) => Status::Failure { reason },
            DriveStatus::NeedsReconfigure(config) => Status::NeedsReconfigure { target_config: config },
        }
    }

    fn preconditions(&self) -> &[Precondition] { Self::PRECONDITIONS }
    fn failure_handler(&self) -> Option<&MissionDescriptor> {
        Some(&MissionDescriptor::HoldPosition)
    }
    fn id(&self) -> &str { "GoTo" }
}
```

GoTo is essentially one line of logic. Path planning, collision avoidance, flipper coordination, body reconfiguration, GNSS integrity, safety limits — all handled by the layers below.

**Example: Follow (leaf, code-defined):**

```rust
struct Follow {
    object_id:        String,
    camera_feed_id:   String,
    follow_distance:  f32,
    motion:           Arc<MotionService>,
    vision:           Arc<VisionService>,
}

impl Mission for Follow {
    fn tick(&mut self, _state: &State, _bb: &mut Blackboard, cancel: &CancelToken) -> Status {
        if cancel.is_cancelled() {
            self.motion.stop();
            return Status::Cancelled;
        }

        match self.vision.get_object(&self.object_id) {
            None => Status::Failure { reason: "Object lost".into() },
            Some(obj) => {
                let target = compute_follow_position(obj.position, self.follow_distance);
                match self.motion.drive_toward(target) {
                    DriveStatus::Running => Status::Running,
                    DriveStatus::NeedsReconfigure(config) => Status::NeedsReconfigure { target_config: config },
                    other => other.into(),
                }
            }
        }
    }
    // ...
}
```

**Example: Sentinel (compound, RON config):**

```ron
Mission(
    id: "Sentinel",
    params: [
        Param(name: "coordinate_a", type: Coordinate),
        Param(name: "coordinate_b", type: Coordinate),
    ],
    preconditions: {
        "min_battery": 30,
        "requires_arm_stowed": true,
        "min_position_confidence": 0.5,
        "max_drift": 25,
    },
    failure_handler: Some("HoldPosition"),
    tree: Loop(
        Sequence([
            Leaf("GoTo", { "coordinate": $coordinate_a }),
            Leaf("GoTo", { "coordinate": $coordinate_b }),
        ])
    ),
)
```

---

## 3D Debug/Replay

The autonomy layer runs headless on the robot. All 3D visualization lives off-robot. The Steam Deck is the operator interface for commanding; a separate operator-station 3D viewer renders the scene from StreamService output.

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Robot (Jetson AGX Orin)                                    │
│                                                             │
│  Python Sidecar ───┐                                        │
│  VisionService  ───┤                                        │
│  Blackboard     ───┼── StreamService ── Comms layer ──────► │
│  ReflexEngine   ───┤                                        │
│  Orchestrator   ───┘                                        │
│  RecordingService ──→ Disk (log files)                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ via Microhard / LoRa
                              │ (delta-encoded by comms layer)
                    ┌─────────┼───────────┐
                    │  Operator Station   │
                    │                     │
                    │  Steam Deck         │  ← commands the robot
                    │   (control UI)      │
                    │                     │
                    │  3D Viewer          │  ← renders the world
                    │   (live + replay)   │
                    └─────────────────────┘
```

### 3D Viewer Modes

| Mode | Description |
|---|---|
| Live | Connected to robot via comms layer. Real-time 3D view. Operator can orbit/zoom the camera independently of the robot's pose. |
| Replay | Load a log file from RecordingService. Scrub through timeline. Play/pause/speed control. |
| Comparison | Side-by-side: live + recording, or two recordings, for debugging. |

### Renders

| Layer | Source | Visualization |
|---|---|---|
| Voxel map | voxel_delta stream | Colored 3D voxels — the environment as the robot sees it (Gaussian-splat aggregated) |
| Robot model | robot_pose + URDF | Animated URDF model showing flipper angles, arm position, gripper state |
| Identified objects | identified_objects stream | Colored bounding boxes with labels (person=red, car=blue, drone=yellow, etc.) |
| POIs | poi_map stream | Labeled pins with tag-type icons |
| Safe points | safe_points stream | Distinct markers (separate from POI icons) with labels |
| Path history | path_history stream | Trail showing where the robot has been |
| Avoid zones | avoid_zones stream | Semi-transparent red volumes |
| Planned path | active_mission stream | Green line showing current planned route |
| Sensor FOVs | URDF sensor mounts (computed by sidecar) | Optional wireframe cones showing camera/lidar coverage |
| Status overlay | reflex_flags, position_state, comms_status | HUD: battery, comms tri-state, GNSS status, active reflexes, drift estimate |

### Read-Only

The 3D viewer does NOT send commands. It is purely a situational awareness and debugging tool. Commands come from the Steam Deck only.

### Tech Stack

_TBD — options: Three.js/WebGL in browser, native Rust viewer with wgpu, or Bevy/Godot game engine. Decision deferred until viewer implementation begins._

---

## Implementation Order

The existing system is being completely replaced. There is no migration path to maintain. The order below is a suggested buildable progression — engineers can branch and parallelize once the foundation is in place.

### Phase 1 — Foundation

1. UDP hardware interface to Pi (read motor telemetry, send motor commands)
2. Direct sensor I/O on Jetson (VN300, Livox lidars, Livox IMUs, IP cameras)
3. System Blackboard infrastructure (locks, snapshots)
4. iceoryx2 pub/sub setup, basic channels
5. Logging infrastructure

### Phase 2 — Safety First

6. L0 ReflexEngine framework (rule registry, action types)
7. Hardware-limit reflex rules (max velocity, max torque, joint limits, gripper force)
8. Orientation reflex rules (pitch, roll, angular rate)
9. Stuck detection rules (ring buffers, thresholds)
10. Thermal and electrical reflex rules

L0 must be in place before anything else commands hardware.

### Phase 3 — World Model

11. Python sidecar process: load URDF via Robotics Toolbox, expose iceoryx2 interface
12. Joint state forwarding from Rust to sidecar
13. Sidecar publishes link transforms, sensor poses (continuous)
14. SpatialEngine in Rust: cache layer, async request/response client to sidecar
15. Sidecar publishes directional minimums for L0 (initially with empty environment)

### Phase 4 — Position

16. PositionService basic fusion (wheel + IMU dead reckoning, no GNSS yet)
17. Add VN300 GNSS to fusion
18. GNSS integrity monitor + state machine (TRUSTED/SUSPECT/REJECTED/ABSENT)
19. Lidar odometry from Livox scan matching
20. Drift budget tracking and blackboard publish

### Phase 5 — Perception

21. MapService basic pipeline: lidar → voxels → push to sidecar (no color yet)
22. Voxel decay, free-space ray-tracing, occupancy publishes
23. Sidecar can now answer collision queries against real environment
24. Camera ingestion + sensor pose timestamp alignment
25. Gaussian splat color aggregation on GPU
26. Speed-adaptive subsampling and splat spread

### Phase 6 — Locomotion

27. MotionService basic drive_toward (straight-line, no obstacle avoidance)
28. Path planning via sidecar (`plan_body_path`)
29. Clearance checking via sidecar
30. BodyAwarenessService reconfiguration search
31. Reconfigure mission and orchestrator handling of NEEDS_RECONFIGURE
32. Flipper coordination logic

### Phase 7 — Mission Framework

33. Mission trait, Status enum, CancelToken
34. Composite primitives: Sequence, Selector, Loop, Parallel (with policies)
35. Decorators: AvoidZone, Timeout, Retry
36. Behavior tree runner with cancellation propagation and auto-escalation
37. RON config loader and validator
38. Mission registry and run_id tracking

### Phase 8 — Mission Layer

39. PreconditionEngine with default types
40. PreemptionEngine with default rule set
41. Orchestrator: tick loop, mission start/cancel/abort, failure handler invocation
42. CommsMonitor (tri-state)
43. SafePointService (auto-update + operator-set)
44. Implement leaf missions: GoTo, HoldPosition, ArmStow, ArmHome, FlippersHome, GripperOpen, GripperClose, Retreat, MarkPOI, SetSafePoint, etc.
45. Implement compound missions from RON: Sentinel, Waypoint, ReturnHome, BacktrackComm, Reset, Calibrate, etc.

### Phase 9 — Vision Integration

46. VisionService consumer for RT-DETR + NvDCF WebSocket stream
47. 2D-to-3D fusion via lidar + sidecar
48. Cross-camera deduplication
49. Velocity estimation and track persistence
50. Implement vision-dependent missions: Follow, Intercept, Track, Inspect, PickUp

### Phase 10 — Operator Interface

51. Command Router: UDP listener, mode state machine
52. Protobuf schema definitions, validation
53. Autonomous mode command handling (StartMission, CancelMission, AbortMission, etc.)
54. Teleop mode handler: subscribe to teleop control messages, run 50 Hz IK loop via sidecar, route to motion/arm/gripper services
55. Mode transitions with cooperative cancel
56. Telemetry stream out

### Phase 11 — Streaming & Replay

57. StreamService: subscribe to all sources, publish full streams to comms layer
58. RecordingService: write streams to disk
59. Comms layer integration (delta encoding, bandwidth-mode adaptation — separate from autonomy stack)

### Phase 12 — 3D Viewer

60. 3D viewer (off-robot, tech stack TBD)
61. Live mode + replay mode + comparison mode

### Phase 13 — Polish

62. Fun missions (Dance, HelloWorld, Worm)
63. Field calibration of all `_TBD_` parameters
64. Performance profiling and tuning of sidecar IPC
65. Long-duration mission testing

This is a sketch, not a strict order. Phases 4–6 can parallelize once Phase 3 is stable. Phase 9 can start as soon as Phase 5 has the camera pose pipeline working.
