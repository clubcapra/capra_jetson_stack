# SAR Robot — Mission Specification

_What the robot does._

---

## Robot

Search and Rescue tracked robot. 120kg. Four flippers, articulated 6DOF arm with gripper, two tracks powered by drums. Jetson AGX Orin (32GB) + Raspberry Pi connected over physical switch. URDF model used by the Python Robotics Toolbox sidecar for full-body kinematics, IK, path planning, and collision checking against the live voxel map.

**Operator hardware:** Steam Deck connected via Microhard (2.4 GHz) and LoRa fallback. Steam Deck talks to the Command Router over UDP.

**Sensors:**

- VN300 (IMU/INS/GNSS) — 200 Hz
- 2x additional IMUs (fused with VN300 via Kalman filter) — 200 Hz
- 2x Livox Mid-360 lidars — 10 Hz
- 8x IP cameras (RTSP, identical make/model, positions defined in URDF)
- Battery monitoring, PMCI

**Perception:** Per-camera RT-DETR detection (~20 fps) with NvDCF tracking provides persistent track IDs. VisionService fuses 2D bounding boxes with lidar depth to produce 3D-positioned, classified, tracked objects (military personnel, cars, drones, robots, POI).

**Locomotion:** Stairs and gap traversal are handled implicitly during driving — not discrete missions. The robot's 3D collision envelope (URDF + current joint states) is checked against the voxel map by the Python Robotics Toolbox sidecar before traversal. If the current body configuration doesn't fit, the orchestrator inserts a visible Reconfigure sub-mission (stow arm, flatten flippers, etc.) before resuming.

**Top speed:** ~30 km/h (8.3 m/s). Autonomous missions typically run at 10–20 km/h.

---

## Concepts

**Missions** are automated behaviors triggered by the operator. Each mission returns `RUNNING`, `SUCCESS`, `FAILURE`, or `CANCELLED`. Missions never read raw telemetry or write hardware commands directly; they call services.

**Leaf missions** are atomic primitives. **Compound missions** compose leaf missions into behavior trees using composite nodes (Sequence, Parallel, Loop, Selector). Compound mission trees are defined in RON config files and loaded at startup; primitives are implemented in code.

**Safe points** are a registry of known-good positions (last comms, last GNSS trusted, last stable ground, operator-designated rally points). Fallback behaviors navigate to contextually relevant safe points — not always home.

**Preconditions** are declarative metadata per mission, checked at start and monitored during execution. Extensible: new precondition types can be registered without modifying existing missions.

**Failure handlers** are per-mission policies for what to do when the mission fails (not preempted, not cancelled — genuinely failed). Each operator mission declares its own failure handler.

**Cancellation** has two modes: `cancel` (cooperative — leaves run cleanup before returning) and `abort` (hard — orchestrator stops the tick immediately, L0 clamps to safe state). Operator-initiated stops are usually `cancel`; safety preemptions are usually `abort`.

**Operating modes:**

- **Autonomous** — missions run, command router executes mission tree, operator monitors and can cancel/abort
- **Teleop** — operator drives directly via Steam Deck inputs, no missions running, L0 reflexes still apply

---

## Operator Interface

### Transport

UDP, both directions, between Steam Deck and Command Router. Binary protobuf-encoded messages. Schema versioned with the firmware.

The comms layer below the autonomy stack handles encoding, delta compression, and bandwidth adaptation between Microhard and LoRa. The autonomy layer publishes full state; the comms layer decides what fits on the wire.

### Autonomous Mode

Operator sends mission commands. Command Router validates, runs preconditions, returns acceptance.

| Command | Description |
|---|---|
| `StartMission(mission_id, params, failure_handler_override?)` | Start a named mission. Returns `MissionAccepted(run_id)` or `MissionRejected(reason)`. |
| `CancelMission(run_id)` | Cooperative cancel — mission leaves run cleanup before returning. |
| `AbortMission(run_id)` | Hard abort — tick stops immediately, L0 clamps. |
| `SetSafePoint(label)` | Mark current position as named operator safe point. |
| `ClearSafePoint(label)` | Remove operator safe point. |
| `SwitchToTeleop` | Cancel active mission, hand control to teleop after cleanup. |

`run_id` is a unique handle for the mission run. Mission status streams back continuously while the mission runs. When the mission ends, the final status is sent and the `run_id` is retired.

### Teleop Mode

Operator drives directly via Steam Deck inputs. Steam Deck sends a teleop control protobuf at high rate; Command Router forwards to services subject to L0 reflexes.

**Teleop control schema:**

| Field | Type | Range | Description |
|---|---|---|---|
| `track_velocity_left` | float | -1.0 to 1.0 | Left drum normalized velocity |
| `track_velocity_right` | float | -1.0 to 1.0 | Right drum normalized velocity |
| `flipper_pos_fl` | float | 0.0 to 1.0 | Front-left flipper position |
| `flipper_pos_fr` | float | 0.0 to 1.0 | Front-right flipper position |
| `flipper_pos_bl` | float | 0.0 to 1.0 | Back-left flipper position |
| `flipper_pos_br` | float | 0.0 to 1.0 | Back-right flipper position |
| `gripper_state` | enum | `open` / `close` | Gripper command |
| `arm_end_effector_pose` | Pose | — | Target pose for end-effector (position + orientation) |
| `camera_subscription` | string or null | — | Camera ID to stream, or null |

The arm is controlled by end-effector pose only. The Python Robotics Toolbox sidecar solves IK in real-time using damped least squares — the arm follows the operator's intent smoothly and degrades gracefully at the workspace boundary (best-effort solution when target is unreachable).

L0 reflexes apply equally in teleop. The operator cannot override safety — pushing the joystick toward a wall still results in the velocity being clipped before it reaches the motors.

### Mode Switching

- **Autonomous → Teleop**: operator sends `SwitchToTeleop`. Active mission gets `cancel`. After cleanup completes (or auto-escalation timeout fires), control hands to teleop inputs.
- **Teleop → Autonomous**: operator sends `StartMission`. Teleop control state is released, mission begins.

### Camera Streams

Cameras are IP cameras on the robot's network. The Steam Deck subscribes directly via RTSP — the Command Router does not proxy video. The teleop control message's `camera_subscription` field tells the Command Router which camera the operator wants active for routing, but the actual video stream is direct camera → operator.

**Analog VTX fallback:** A separate analog video transmitter on the Pi streams one camera independently of the IP network. Operator-side switchable. Out of scope for the autonomy layer — handled entirely by the Pi — but documented here as part of the redundancy story.

### Telemetry Stream

Command Router streams telemetry continuously to the Steam Deck UDP listener:

- Position, heading, velocity
- Comms state (HEALTHY / DEGRADED / LOST)
- Battery, thermal, reflex flags
- Active mission state and progress (when in autonomous mode)
- Identified objects with classifications
- Safe points, POIs
- Mission tree state changes

The comms layer below handles delta encoding and degrades gracefully on LoRa.

---

## Hardware Interface

The autonomy stack interfaces with hardware through two paths: a UDP link between the Jetson and the Pi (motors, gripper) and direct sensor input on the Jetson (cameras, lidars, IMUs).

### Pi (UDP) — Commands & Motor Telemetry

**Commands (Write):**

| Channel | Description |
|---|---|
| ODrive | Track motor commands (left/right drum velocity, torque), flipper commands |
| 6DOF joint states | Arm joint positions, velocities, torques (6 joints) |
| Gripper | Position, speed, force |

Pi API publishes at 50 Hz; sensors manage their own input rates.

**Telemetry (Read):**

| Channel | Description | Rate |
|---|---|---|
| ODrive | Motor temps, current draw, encoder positions, velocities, faults | 50 Hz |
| 6DOF | Joint positions, velocities, torques, temps | 75 Hz (arm-driven) |
| Gripper | Position, speed, force | 200 Hz |
| Battery | Voltage, current, charge level, cell health | 50 Hz |
| PMCI | Power management and control interface data | 50 Hz |

### Jetson (Direct) — Sensors & Perception Inputs

| Channel | Description | Rate |
|---|---|---|
| VN300 | IMU/INS — orientation, angular rates, acceleration, GNSS position, heading | 200 Hz |
| Livox Mid-360 (x2) | 3D point clouds, lidar odometry source | 10 Hz |
| Livox IMUs | Acceleration, angular rates (fused with VN300 via Kalman filter) | 200 Hz |
| Cameras (x8) | RGB image streams (RTSP) | ~30 fps |
| Vision pipeline | Per-camera RT-DETR + NvDCF tracker — identified objects with stable track IDs | ~20 fps |

---

## Safe Points

The robot maintains a registry of known-good positions, updated automatically and manually.

| Type | Updated | Description |
|---|---|---|
| `origin` | Once on deploy | Deployment start position |
| `last_comms` | Auto — while comms HEALTHY | Last position with full link quality |
| `last_gnss_trusted` | Auto — while GNSS validated | Last position with trusted GNSS fix |
| `last_stable` | Auto — periodically on flat, clear ground | Last position safe to park |
| `last_vision` | Auto — while vision pipeline running | Last position with working vision |
| `operator` | Manual — multiple allowed | Operator-designated rally points, extraction zones |

Each safe point carries: coordinate, position confidence, drift estimate, timestamp. Stale safe points are factored into fallback resolution — distance × age × confidence determine which point is "best" for a given trigger.

---

## Mission Preconditions & Failure Handlers

| Mission | min_battery | requires_comms | requires_arm_stowed | min_position_confidence | max_drift | max_pitch | failure_handler |
|---|---|---|---|---|---|---|---|
| GoTo | 20% | — | ✓ | 0.5 | 25m | — | HoldPosition |
| Follow | 30% | DEGRADED | ✓ | — | — | — | HoldPosition |
| Intercept | 30% | DEGRADED | ✓ | — | — | — | HoldPosition |
| Track | 15% | DEGRADED | — | — | — | — | HoldPosition |
| Explore | 40% | — | ✓ | 0.7 | 10m | — | ReturnHome(target=last_stable) |
| Sentinel | 30% | — | ✓ | 0.5 | 25m | — | HoldPosition |
| ReturnHome | — | — | ✓ | 0.3 | — | — | HoldPosition |
| BacktrackComm | — | — | ✓ | — | — | — | HoldPosition |
| Orbit | 25% | — | ✓ | 0.5 | 10m | — | HoldPosition |
| Retreat | 15% | — | ✓ | — | — | — | HoldPosition |
| PickUp | 20% | DEGRADED | — | — | — | 20° | Sequence(GripperOpen, ArmStow) |
| Inspect | 20% | HEALTHY | — | — | — | 20° | ArmStow |
| RelayPosition | 25% | — | ✓ | 0.5 | 25m | — | HoldPosition |

**Notes:**

- `requires_comms` accepts `HEALTHY` (full bandwidth) or `DEGRADED` (any link, including LoRa). Default for `requires_comms: true` shorthand is `DEGRADED`.
- Inspect uses `HEALTHY` because the operator reviews the camera feed before deciding next action — needs full bandwidth.
- ReturnHome and BacktrackComm have relaxed position requirements — emergency fallbacks must run in degraded conditions.
- Retreat uses `path_history` (relative) — no GNSS or position confidence required.
- Follow / Intercept / Track use vision-relative positioning — global accuracy not required.
- Failure handlers are themselves missions, run with their own preconditions and preemption rules.
- Operators can override the failure handler per-invocation via `StartMission`.

Values are starting defaults — calibrate from field testing.

---

## Operator Missions

### Leaf Missions

#### GoTo

Go to a given coordinate.

| Param | Type | Description |
|---|---|---|
| coordinate | coordinate | Target destination |

**Success:** Arrival within threshold (threshold widens with drift estimate)
**Failure:** Unreachable, stuck timeout

---

#### Follow

Follow a specified object in view.

| Param | Type | Description |
|---|---|---|
| object_id | string | Target from vision pipeline (NvDCF persistent track ID) |
| camera_feed_id | string | Camera feed tracking the object |
| follow_distance | float | Desired distance to maintain (default 3m) |

**Success:** Continuous — runs until cancelled
**Failure:** Object lost from vision pipeline beyond timeout

---

#### Intercept

Intercept a moving object based on velocity.

| Param | Type | Description |
|---|---|---|
| object_id | string | Target from vision pipeline |
| camera_feed_id | string | Camera feed tracking the object |

**Success:** Within intercept threshold of target
**Failure:** Object lost, unreachable

---

#### Track

Lock camera on a target without moving the robot. Arm camera follows the object.

| Param | Type | Description |
|---|---|---|
| object_id | string | Target from vision pipeline |
| camera_feed_id | string | Camera feed to use |

**Success:** Continuous — runs until cancelled
**Failure:** Object lost from pipeline

---

#### HoldPosition

Full stop, hold ground.

_No parameters._

**Success:** Continuous — runs until cancelled
**Failure:** Drift beyond threshold

---

#### ListenPing

Motors off, passive audio listening.

| Param | Type | Description |
|---|---|---|
| duration_seconds | float | How long to listen |

**Success:** Duration elapsed or signal detected
**Failure:** Timeout with no detection

---

#### MarkPOI

Drop a labeled pin at current position.

| Param | Type | Description |
|---|---|---|
| tag_type | string | Label (commonly from vision pipeline classification) |

**Success:** Immediate — POI carries position confidence
**Failure:** None

---

#### SetSafePoint

Operator manually designates the current position as a named operator safe point.

| Param | Type | Description |
|---|---|---|
| label | string | Human-readable label |

**Success:** Immediate
**Failure:** None

---

#### DisplayStatus

Visually show the robot's status.

_No parameters._

**Success:** Immediate
**Failure:** Display hardware fault

---

#### SignalOperator

Robot draws attention to itself (victim-facing — "help is here").

| Param | Type | Description |
|---|---|---|
| signal_type | enum | `visual`, `audio`, `both` |

**Success:** Continuous — runs until cancelled
**Failure:** Hardware fault

---

#### ArmStow

Tuck the arm into compact travel position.

_No parameters._

**Success:** All joints at stow configuration
**Failure:** Joint fault, timeout

---

#### ArmHome

Set the arm to home/default position.

_No parameters._

**Success:** All joints at home configuration
**Failure:** Joint fault, timeout

---

#### FlippersHome

Set all flippers to default position.

_No parameters._

**Success:** All flippers at default angles
**Failure:** Flipper fault, timeout

---

#### GripperOpen

Open the gripper.

_No parameters._

**Success:** Gripper at open position
**Failure:** Gripper fault

---

#### GripperClose

Close the gripper.

| Param | Type | Description |
|---|---|---|
| force | float | Optional grip force limit |

**Success:** Closed or force threshold met
**Failure:** Gripper fault

---

#### Retreat

Back up a set distance along traveled path.

| Param | Type | Description |
|---|---|---|
| distance | float | Distance in meters |

**Success:** Distance covered
**Failure:** Rear obstacle, stuck

---

#### Orbit

Circle a point of interest.

| Param | Type | Description |
|---|---|---|
| coordinate | coordinate | Center point |
| radius | float | Orbit radius in meters |

**Success:** Continuous — runs until cancelled
**Failure:** Obstacle blocks orbit path

---

### Compound Missions

Defined in RON config files, composed from leaf missions and internal primitives.

#### Sentinel

Patrol between point A and point B.

| Param | Type | Description |
|---|---|---|
| coordinate_a | coordinate | First patrol waypoint |
| coordinate_b | coordinate | Second patrol waypoint |

**Tree:** `Loop(Sequence(GoTo(A), GoTo(B)))`

---

#### Waypoint

Follow an ordered route.

| Param | Type | Description |
|---|---|---|
| coordinate_list | coordinate[] | Ordered list of waypoints |

**Tree:** `Sequence(GoTo(c1), GoTo(c2), ..., GoTo(cN))`

---

#### ReturnHome

Navigate back to a safe point. Defaults to origin.

| Param | Type | Description |
|---|---|---|
| target | string (optional) | Safe point key. Default `origin`. Options: `origin`, `last_comms`, `last_stable`, `last_gnss_trusted`, or operator label. |

**Tree:** `GoTo(safe_points[target])` with retrace fallback if global planning fails

---

#### BacktrackComm

Go back toward the last known comms point until communication is regained.

_No parameters._

**Tree:** `GoTo(safe_points.last_comms)` with comms check each tick — stops as soon as link returns to HEALTHY

---

#### Explore

Explore the given perimeter to create a map of it with POIs.

| Param | Type | Description |
|---|---|---|
| coordinates_geometry | geometry | Perimeter to explore |

**Tree:** `Sequence(GenerateCoveragePath(perimeter), Waypoint(path))` with `MarkPOI` triggered by vision pipeline events

---

#### SurveyArea

Wait, look, and listen from current position.

_No parameters._

**Tree:** `Parallel(policy=require_all, HoldPosition, CameraSweep, ListenPing)`

---

#### Inspect

Move arm camera close to a vision-detected object for detail.

| Param | Type | Description |
|---|---|---|
| object_id | string | Target from vision pipeline |

**Tree:** `Sequence(ArmReachToward(object), Capture, ArmStow)`

---

#### PickUp

Grasp a target object identified by the vision pipeline.

| Param | Type | Description |
|---|---|---|
| object_id | string | Target from vision pipeline |

**Tree:** `Sequence(ArmReachToward(object), GripperOpen, ArmFinalApproach, GripperClose, ArmStow)`

---

#### Drop

Release current payload.

_No parameters._

**Tree:** `Sequence(ArmReachForward, GripperOpen, ArmStow)`

---

#### Calibrate

Run calibration sequence for flippers and arm.

_No parameters._

**Tree:** `Sequence(FlippersCalibrate, ArmCalibrate, GripperCalibrate)`

---

#### Reset

Set the robot to home state.

_No parameters._

**Tree:** `Parallel(policy=require_all, FlippersHome, ArmHome, GripperOpen)`

---

#### RelayPosition

Move to a coordinate and hold as a communications relay.

| Param | Type | Description |
|---|---|---|
| coordinate | coordinate | Relay position |

**Tree:** `Sequence(GoTo(coordinate), HoldPosition)` with comms health monitored

---

### Decorators

Modifiers that wrap other missions.

#### AvoidZone

Define avoidance zones for wrapped missions.

| Param | Type | Description |
|---|---|---|
| coordinates_geometry | geometry | No-go region |

**Behavior:** Writes the geometry to the mission-scoped blackboard. Path planning respects it for the duration of the wrapped mission. Cleaned up automatically when the mission ends.

---

### Fun

No tactical purpose. Compound missions built from motion primitives.

#### Dance

Pre-programmed dance showcase. _No parameters._

#### HelloWorld

Wave with the arm. _No parameters._

#### Worm

Do the worm. _No parameters._

---

## Internal Primitives

These share the leaf mission interface (`tick() → RUNNING/SUCCESS/FAILURE/CANCELLED`) but are not in the operator command catalog. They exist only as building blocks for compound missions and are implemented as thin wrappers around service method calls.

To promote an internal primitive to operator-callable, move its entry to the Operator Missions section.

### CameraSweep

Sweep the arm camera through a configured arc. Used by `SurveyArea`.

**Calls:** `ArmService` joint trajectory through preset sweep waypoints; `VisionService` accumulates detections during sweep
**Success:** Sweep complete
**Failure:** Joint fault

---

### ArmReachToward

Move arm end-effector toward a 3D point. Used by `Inspect`, `PickUp`, `Track`.

| Param | Type | Description |
|---|---|---|
| target_position | Vec3 | World-frame 3D point |

**Calls:** `ArmService.reach_toward(target_position)` — IK via Python Robotics Toolbox sidecar
**Success:** End-effector within tolerance of target
**Failure:** Unreachable (IK no solution), collision, joint fault

---

### ArmFinalApproach

Final slow approach during pickup. Used by `PickUp`.

**Calls:** `ArmService` Cartesian linear motion toward grasp pose, slower than reach_toward
**Success:** Grasp pose reached
**Failure:** Collision, joint fault, force limit hit

---

### ArmReachForward

Move arm to a forward extension pose for dropping payload. Used by `Drop`.

**Calls:** `ArmService.move_to_joint_state(forward_drop_config)`
**Success:** Joints at forward configuration
**Failure:** Joint fault, timeout

---

### Capture

Capture a single image from the arm camera. Used by `Inspect`.

| Param | Type | Description |
|---|---|---|
| camera_feed_id | string | Which camera (typically arm-mounted) |

**Calls:** Direct image capture via VisionService passthrough, recorded with current arm pose
**Success:** Image captured and tagged with pose + timestamp
**Failure:** Camera fault

---

### FlippersCalibrate

Range-of-motion sweep and encoder zeroing for all four flippers.

**Calls:** Direct ODrive commands via MotionService calibration mode
**Success:** All flippers calibrated, encoders zeroed
**Failure:** Flipper fault, timeout

---

### ArmCalibrate

Range-of-motion sweep and encoder zeroing for arm joints.

**Calls:** Direct 6DOF commands via ArmService calibration mode
**Success:** All arm joints calibrated
**Failure:** Joint fault, timeout

---

### GripperCalibrate

Range-of-motion sweep for gripper.

**Calls:** Direct gripper commands via GripperService calibration mode
**Success:** Gripper calibrated
**Failure:** Gripper fault, timeout

---

### Reconfigure

Inserted by the orchestrator (not operator-callable) when MotionService returns `NEEDS_RECONFIGURE` mid-drive. Brings the robot into a target body configuration so the active mission can resume.

| Param | Type | Description |
|---|---|---|
| target_config | BodyConfig | Desired flipper angles, arm joint states, gripper position |

**Tree:** `Sequence` of joint moves required to reach target_config
**Success:** Robot in target configuration
**Failure:** Joint fault, collision during reconfiguration, timeout
**Cancellation:** Cooperative cancel returns arm/flippers to safe intermediate before returning CANCELLED. Hard abort lets L0 clamp.

The active mission tree is paused while Reconfigure runs. On Reconfigure SUCCESS, the active mission resumes (with replanning, since the world may have changed). On Reconfigure FAILURE, the active mission inherits the failure.
