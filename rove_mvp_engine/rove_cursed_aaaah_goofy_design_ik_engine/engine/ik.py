"""IK math for the exported engine — self-contained, numpy-only.

Loaded chain spec (chain.json) describes the kinematic chain in
forge-engine canonical form:

  {
    "base": "<entity_id>",
    "tip":  "<entity_id>",
    "tip_offset": [[4x4]],          // tip-link → TCP, identity by default
    "joints": [
      {
        "id":        "<entity_id>",
        "name":      "joint_1",
        "type":      "revolute" | "prismatic" | "fixed",
        "axis":      [x, y, z],     // unit vector in joint local frame
        "lower":     -1.57,         // rad or m
        "upper":      1.57,
        "velocity":   2.0,          // rad/s or m/s, used for clamp + scaling
        "inverted":   false,        // true → joint rotates opposite the URDF axis
        "pre_xform":  [[4x4]],      // parent link → joint origin (fixed)
        "post_xform": [[4x4]]       // joint → child link origin (fixed)
      },
      ...
    ]
  }

`inverted` is consumed by both the engine's FK/Jacobian (so the
solver matches the editor's behavior) and the morpher (which flips
the sign of the per-joint velocity the engine emits before forwarding
it to the real arm). There is no position-offset constant — IK is
done in URDF-native joint coordinates throughout.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class JointSpec:
    id: str
    name: str
    type: str
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float
    inverted: bool
    pre_xform: np.ndarray
    post_xform: np.ndarray

    @property
    def movable(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic")

    @property
    def sign(self) -> float:
        return -1.0 if self.inverted else 1.0


@dataclass
class Chain:
    base: str
    tip: str
    joints: list[JointSpec]
    tip_offset: np.ndarray

    @property
    def movable(self) -> list[JointSpec]:
        return [j for j in self.joints if j.movable]


@dataclass
class Profile:
    """Tuned IK parameters from the editor's `Train IK…` step.

    Only the fields the engine actually uses for resolved-rate +
    position IK. Score / metadata fields are kept for traceability
    but ignored at runtime.
    """
    mode: str = "pose_locked"           # default for POSITION_IK requests
    damping: float = 0.05               # DLS damping (lambda)
    rest_pose_gain: float = 0.3         # null-space pull toward rest_pose
    max_iter: int = 60                  # POSITION_IK iterations
    orientation_weight: float = 5.0
    joint_weight_strength: float = 0.0
    max_dq_step: float = 0.05           # rad per joint per iter (POSITION_IK)
    max_pos_step: float = 0.05          # m per iter (POSITION_IK)
    max_total_dq_step: float | None = 0.10
    orientation_secondary_gain: float = 0.5
    # Velocity scaling for the normalized [-1, 1] twist input.
    max_lin_vel: float = 0.25           # m/s when |position.{x,y,z}| = 1
    max_ang_vel: float = 1.0            # rad/s when |orientation.{...}| = 1
    rest_pose: dict = field(default_factory=dict)  # joint_id → value (rad/m)


# ---- (de)serialization ----

def load_chain(path: Path) -> Chain:
    data = json.loads(Path(path).read_text())
    joints = []
    for j in data["joints"]:
        joints.append(JointSpec(
            id=j["id"],
            name=j.get("name", j["id"]),
            type=j["type"],
            axis=np.asarray(j["axis"], dtype=np.float64),
            lower=float(j.get("lower", 0.0)),
            upper=float(j.get("upper", 0.0)),
            velocity=float(j.get("velocity", 1.0) or 1.0),
            inverted=bool(j.get("inverted", False)),
            pre_xform=np.asarray(j["pre_xform"], dtype=np.float64),
            post_xform=np.asarray(j["post_xform"], dtype=np.float64),
        ))
    return Chain(
        base=data["base"],
        tip=data["tip"],
        joints=joints,
        tip_offset=np.asarray(
            data.get("tip_offset", np.eye(4).tolist()), dtype=np.float64
        ),
    )


def load_profile(path: Path) -> Profile:
    data = json.loads(Path(path).read_text())
    p = Profile()
    for k, v in data.items():
        if hasattr(p, k):
            setattr(p, k, v)
    return p


# ---- transforms ----

def _axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation. axis must be unit-length."""
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    a = axis / n
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _joint_xform(j: JointSpec, q: float) -> np.ndarray:
    """Variable transform applied AT the joint axis.

    `q` is the slider-space value (radians for revolute, meters for
    prismatic) — what the IK works in. FK applies *only* the direction
    flip: `actual = sign * q`. `offset` lives in chain.json for the
    morpher to convert between real and slider space, but never enters
    FK — adding it would deform the model and break IK convergence
    when calibration is set.
    """
    actual = j.sign * q
    T = np.eye(4)
    if j.type in ("revolute", "continuous"):
        T[:3, :3] = _axis_angle_to_R(j.axis, actual)
    elif j.type == "prismatic":
        n = np.linalg.norm(j.axis) or 1.0
        T[:3, 3] = (j.axis / n) * actual
    return T


def fk(chain: Chain, q: dict[str, float]) -> tuple[np.ndarray, list[np.ndarray]]:
    """Forward kinematics along the chain.

    Returns:
        T_tip: 4x4 world transform of the tip (after tip_offset)
        T_at_joints: list of 4x4 world transforms AT each joint axis
                     (i.e., after pre_xform but before joint actuation),
                     in chain order. Used by the Jacobian builder.
    """
    T = np.eye(4)
    T_at_joints: list[np.ndarray] = []
    for j in chain.joints:
        T = T @ j.pre_xform
        T_at_joints.append(T.copy())
        T = T @ _joint_xform(j, q.get(j.id, 0.0))
        T = T @ j.post_xform
    T_tip = T @ chain.tip_offset
    return T_tip, T_at_joints


def jacobian(chain: Chain, q: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
    """Geometric Jacobian for the *movable* joints, evaluated at q.

    Returns:
        J: 6×n matrix (rows = [vx, vy, vz, wx, wy, wz], cols = movable joints)
        T_tip: same as `fk()` for caller convenience.
    """
    T_tip, T_at_joints = fk(chain, q)
    p_tip = T_tip[:3, 3]
    movable = chain.movable
    n = len(movable)
    J = np.zeros((6, n))
    # We need T_at_joint for each *movable* joint specifically. Build an
    # index map: chain.joints index → column in J (only movable joints).
    col = 0
    for i, j in enumerate(chain.joints):
        if not j.movable:
            continue
        T_aj = T_at_joints[i]
        axis_world = T_aj[:3, :3] @ j.axis
        n_axis = np.linalg.norm(axis_world)
        if n_axis > 1e-12:
            axis_world = axis_world / n_axis
        if j.type == "prismatic":
            J[:3, col] = axis_world
            J[3:, col] = 0.0
        else:  # revolute / continuous
            origin = T_aj[:3, 3]
            J[:3, col] = np.cross(axis_world, p_tip - origin)
            J[3:, col] = axis_world
        # d(end_effector)/d(slider) = sign · d(end_effector)/d(URDF_angle).
        # For inverted joints, every row of this column flips.
        if j.sign != 1.0:
            J[:, col] *= j.sign
        col += 1
    return J, T_tip


# ---- resolved-rate IK ----

def resolved_rate(
    chain: Chain,
    q: dict[str, float],
    twist: np.ndarray,
    profile: Profile,
) -> dict[str, float]:
    """Compute joint velocities for a desired Cartesian twist.

    twist is a 6-vector [vx, vy, vz, wx, wy, wz] in the world frame,
    already scaled to physical units (m/s, rad/s).

    Returns {joint_id → q_dot}, clamped per-joint by the chain's
    velocity limits and ramped down near joint position limits to
    avoid running into a hard stop at speed.
    """
    J, _ = jacobian(chain, q)
    movable = chain.movable
    # Damped pseudoinverse: q_dot = Jᵀ (J Jᵀ + λ²I)⁻¹ v
    lam_sq = profile.damping ** 2
    JJT = J @ J.T
    A = JJT + lam_sq * np.eye(6)
    try:
        v = np.linalg.solve(A, twist)
    except np.linalg.LinAlgError:
        v = np.linalg.lstsq(A, twist, rcond=None)[0]
    q_dot = J.T @ v

    # Per-joint clamps + soft limit avoidance.
    for col, j in enumerate(movable):
        # Limit avoidance: scale q_dot toward the safe direction inside
        # a 5% margin from each end-stop. Linear ramp.
        cur = q.get(j.id, 0.0)
        span = max(j.upper - j.lower, 1e-9)
        margin = 0.05 * span
        if q_dot[col] > 0 and cur > j.upper - margin:
            scale = max(0.0, (j.upper - cur) / margin)
            q_dot[col] *= scale
        elif q_dot[col] < 0 and cur < j.lower + margin:
            scale = max(0.0, (cur - j.lower) / margin)
            q_dot[col] *= scale
        # Hard cap at the joint's velocity limit.
        if j.velocity > 0 and abs(q_dot[col]) > j.velocity:
            q_dot[col] = math.copysign(j.velocity, q_dot[col])

    return {j.id: float(q_dot[col]) for col, j in enumerate(movable)}


# ---- position IK (DLS, iterative) ----

def _R_to_axis_angle(R: np.ndarray) -> np.ndarray:
    """log map on SO(3): returns 3-vector ω with ||ω|| = angle, direction = axis."""
    cos_t = max(-1.0, min(1.0, (np.trace(R) - 1.0) * 0.5))
    theta = math.acos(cos_t)
    if theta < 1e-9:
        return np.zeros(3)
    if abs(math.pi - theta) < 1e-6:
        # Near-π: numerically careful branch
        eig = (R + np.eye(3)) * 0.5
        # Pick the column with largest diagonal entry as the axis.
        i = int(np.argmax(np.diag(eig)))
        ax = np.sqrt(np.maximum(eig[:, i], 0.0))
        if ax[i] != 0:
            ax = ax * np.sign(eig[i, i])
        return ax * theta
    inv = 0.5 / math.sin(theta)
    return np.array([
        (R[2, 1] - R[1, 2]) * inv,
        (R[0, 2] - R[2, 0]) * inv,
        (R[1, 0] - R[0, 1]) * inv,
    ]) * theta


@dataclass
class IKResult:
    q: dict[str, float]
    iterations: int
    residual: float
    converged: bool


def position_ik(
    chain: Chain,
    q_init: dict[str, float],
    target_pos: np.ndarray,
    target_R: np.ndarray | None,
    profile: Profile,
    *,
    tol: float = 1e-3,
) -> IKResult:
    """Solve for joint positions that put the tip at target_pos / target_R.

    Damped least squares with optional null-space rest-pose pull.
    Mode follows `profile.mode`:
       "pose_locked": position+orientation in primary task (6 rows)
       "pos_primary": position primary (3 rows), orientation secondary
                      via null-space projector
    """
    movable = chain.movable
    n = len(movable)
    q = {j.id: q_init.get(j.id, 0.0) for j in movable}

    pose_locked = profile.mode == "pose_locked" and target_R is not None
    has_rot = target_R is not None
    rest = profile.rest_pose or {}
    lam_sq = max(profile.damping, 1e-6) ** 2

    res = float("inf")
    converged = False
    it = 0
    for it in range(profile.max_iter):
        J6, T_tip = jacobian(chain, q)
        p_tip = T_tip[:3, 3]
        R_tip = T_tip[:3, :3]
        e_pos = target_pos - p_tip
        e_rot = (
            _R_to_axis_angle(target_R @ R_tip.T)
            if has_rot else np.zeros(3)
        )
        # Step-size cap on per-iter Cartesian motion (prevents jumps).
        np_e_pos = np.linalg.norm(e_pos)
        if np_e_pos > profile.max_pos_step:
            e_pos = e_pos * (profile.max_pos_step / np_e_pos)
        np_e_rot = np.linalg.norm(e_rot)
        if np_e_rot > 0.30:
            e_rot = e_rot * (0.30 / np_e_rot)

        if pose_locked:
            err = np.concatenate([e_pos, profile.orientation_weight * e_rot])
            J = J6.copy()
            J[3:] *= profile.orientation_weight
            A = J @ J.T + lam_sq * np.eye(6)
            v = np.linalg.solve(A, err)
            dq = J.T @ v
        else:
            J = J6[:3]
            err = e_pos
            A = J @ J.T + lam_sq * np.eye(3)
            v = np.linalg.solve(A, err)
            dq = J.T @ v
            # Null-space secondaries: orientation pull (if has_rot) and
            # rest-pose pull. Project secondary delta through (I − J⁺J).
            Jp = J.T @ np.linalg.solve(A, np.eye(3))  # damped pseudoinverse
            null_proj = np.eye(n) - Jp @ J
            secondary = np.zeros(n)
            if has_rot:
                # Map orientation error back into joint space using full J.
                Aw = J6[3:] @ J6[3:].T + lam_sq * np.eye(3)
                dq_rot = J6[3:].T @ np.linalg.solve(Aw, e_rot)
                secondary += profile.orientation_secondary_gain * dq_rot
            if rest:
                pull = np.array([
                    (rest.get(j.id, q[j.id]) - q[j.id]) * profile.rest_pose_gain
                    for j in movable
                ])
                secondary += pull
            dq = dq + null_proj @ secondary

        # Clamp per-joint dq.
        if profile.max_dq_step > 0:
            mx = float(np.max(np.abs(dq))) if dq.size else 0.0
            if mx > profile.max_dq_step:
                dq = dq * (profile.max_dq_step / mx)
        # Clamp total ||dq||_∞ across the call (across iterations) — we
        # approximate "across the call" as "across this iteration".
        if profile.max_total_dq_step is not None:
            mx = float(np.max(np.abs(dq))) if dq.size else 0.0
            if mx > profile.max_total_dq_step:
                dq = dq * (profile.max_total_dq_step / mx)

        # Apply with joint-limit clamp.
        for col, j in enumerate(movable):
            new = q[j.id] + float(dq[col])
            if j.upper > j.lower:
                new = max(j.lower, min(j.upper, new))
            q[j.id] = new

        res = float(np.linalg.norm(e_pos))
        if has_rot:
            res = math.sqrt(res * res + np.linalg.norm(e_rot) ** 2)
        if res < tol:
            converged = True
            break

    return IKResult(q=q, iterations=it + 1, residual=res, converged=converged)
