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
it to the real arm). The project's home pose is baked into pre_xform
at export time (see IKEngineExporter), so q=0 is the home pose and IK
operates in "delta-from-home" coordinates throughout the engine — no
runtime offset application is needed in this file.
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
    joint_limit_avoidance: float = 0.5  # cubic penalty pulling away from limits
    max_dq_step: float = 0.05           # rad per joint per iter (POSITION_IK)
    max_pos_step: float = 0.05          # m per iter (POSITION_IK)
    max_rot_step: float = 0.30          # rad per iter on orientation error
    max_total_dq_step: float | None = 0.10
    orientation_secondary_gain: float = 0.5
    step: float = 1.0                   # global step-size multiplier
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

    `q` is the home-relative joint value (radians for revolute, meters
    for prismatic) — what the IK works in. FK applies *only* the
    direction flip: `actual = sign * q`. The project's home pose is
    statically baked into pre_xform at export, so q=0 means home; the
    morpher's encoder→engine conversion (encoder − home_deg) keeps the
    real arm aligned with this convention.
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
    tol: float = 1e-4,
    tcp_offset_local: np.ndarray | None = None,
    collision_world: "object | None" = None,
    chain_q_to_urdf: dict[str, str] | None = None,
) -> IKResult:
    """Faithful clone of forge's solve_position_ik.

    Uses SVD-based truncated pseudo-inverse with adaptive damping (gives
    well-behaved motion near singularities), Liégeois-style null-space
    projection for secondary objectives (rest-pose pull + cubic joint-
    limit avoidance), and task-priority orientation in pos_primary mode.
    Per-iteration step caps on Cartesian error AND a per-call cap on
    cumulative joint motion (`max_total_dq_step`), with residuals
    recomputed after the cumulative clamp so the caller's HUD reflects
    the actually-achieved error.

    The two solvers must stay in sync — if you fix something here, mirror
    it in `forge/backend/forgebot/core/kinematics/inverse.py`.
    """
    movable = chain.movable
    n = len(movable)
    if n == 0:
        return IKResult(q=dict(q_init), iterations=0, residual=0.0, converged=True)

    q: dict[str, float] = {j.id: float(q_init.get(j.id, 0.0)) for j in movable}
    q_initial: dict[str, float] = dict(q)

    has_rot = target_R is not None
    pose_mode = has_rot and profile.mode != "pos_primary"
    rest = profile.rest_pose or {}
    rows = 6 if pose_mode else 3
    damp_sq = max(profile.damping, 1e-6) ** 2
    sv_threshold = 0.05
    step = profile.step

    # Tapered joint weights: joints closer to the base get smaller weights
    # (move more readily) than joints near the tip — matches the editor's
    # joint_weight_strength behaviour.
    alpha = max(0.0, min(1.0, profile.joint_weight_strength))
    tapered = np.array(
        [max(1.0, 50.0 * (0.4 ** depth)) for depth in range(n)],
        dtype=float,
    )
    weights = (1.0 - alpha) * np.ones(n) + alpha * tapered
    w_inv = 1.0 / weights
    s_w = np.sqrt(w_inv)

    last_residual = float("inf")
    converged = False
    it = 0
    tcp = (
        np.asarray(tcp_offset_local, dtype=np.float64)
        if tcp_offset_local is not None
        else None
    )

    # Collision baseline: capture the pairs already colliding at the
    # starting q (e.g., chassis self-overlaps in the URDF). The iteration
    # only breaks if a *new* pair appears — mirrors solve_position_ik.
    def _q_to_urdf(q_local: dict[str, float]) -> dict[str, float]:
        if not chain_q_to_urdf:
            return {}
        return {chain_q_to_urdf[jid]: v for jid, v in q_local.items() if jid in chain_q_to_urdf}

    baseline_pairs: frozenset = frozenset()
    if collision_world is not None:
        try:
            baseline_pairs = frozenset(
                tuple(sorted((p.a, p.b)))
                for p in collision_world.check(_q_to_urdf(q))
            )
        except Exception:  # noqa: BLE001
            collision_world = None
    for it in range(profile.max_iter):
        J6, T_tip = jacobian(chain, q)
        ee_R = T_tip[:3, :3]
        # If a TCP offset is given, the position task targets the TCP (tip
        # link origin + R_tip @ tcp_offset). The Jacobian must reflect this:
        # the position rows are built from cross(axis_world, ee_pos − joint_pos)
        # already inside `jacobian()` *using the tip-origin point*. We add
        # the offset contribution: d(ee_pos)/dq = J_pos + skew(axis_world) @
        # (R_tip @ tcp) for revolute joints, which simplifies to recomputing
        # J_pos with the TCP point. The cleanest way is to add the correction
        # term to the existing J_pos rows: cross(axis_world, R_tip @ tcp).
        if tcp is not None:
            tcp_world = ee_R @ tcp
            ee_pos = T_tip[:3, 3] + tcp_world
            # axis_world for each joint is recoverable from J_rot rows (J6[3:])
            # — that's already axis_world (× sign for inverted). Append the
            # offset cross product to the position rows.
            J6 = J6.copy()
            for i in range(J6.shape[1]):
                axis_w = J6[3:, i]  # already sign-flipped if inverted
                J6[:3, i] = J6[:3, i] + np.cross(axis_w, tcp_world)
        else:
            ee_pos = T_tip[:3, 3]
        pos_err = target_pos - ee_pos
        rot_err_raw = (
            _R_to_axis_angle(target_R @ ee_R.T) if has_rot else np.zeros(3)
        )

        unscaled = np.concatenate([pos_err, rot_err_raw]) if pose_mode else pos_err
        residual = float(np.linalg.norm(unscaled))
        last_residual = residual
        if residual < tol and (
            profile.mode != "pos_primary" or np.linalg.norm(rot_err_raw) < tol
        ):
            converged = True
            break

        # Cap Cartesian step magnitudes to keep linearization valid.
        pos_norm = float(np.linalg.norm(pos_err))
        if pos_norm > profile.max_pos_step:
            pos_err = pos_err * (profile.max_pos_step / pos_norm)
        rot_norm = float(np.linalg.norm(rot_err_raw))
        if rot_norm > profile.max_rot_step:
            rot_err_raw = rot_err_raw * (profile.max_rot_step / rot_norm)

        if pose_mode:
            err = np.concatenate([pos_err, profile.orientation_weight * rot_err_raw])
            J = J6.copy()
            J[3:] *= profile.orientation_weight
        else:
            err = pos_err
            J = J6[:3].copy()
        J_rot = J6[3:].copy()  # unweighted rotation rows for task-priority block

        # SVD-based damped pseudo-inverse with adaptive damping near small
        # singular values — keeps `dq` bounded across singularities instead
        # of blowing up like a plain DLS.
        J_w = J * s_w
        try:
            U, sigma, Vt = np.linalg.svd(J_w, full_matrices=False)
        except np.linalg.LinAlgError:
            break
        lam_sq = damp_sq + np.maximum(0.0, sv_threshold * sv_threshold - sigma * sigma)
        sigma_inv = sigma / (sigma * sigma + lam_sq)
        u_primary = Vt.T @ (sigma_inv * (U.T @ err))
        dq = s_w * u_primary

        # Secondary objectives (Liégeois null-space projection):
        #   - rest-pose pull
        #   - cubic joint-limit avoidance
        secondary = np.zeros(n)
        q_vec = np.array([q[j.id] for j in movable])
        if rest and profile.rest_pose_gain > 0.0:
            rest_vec = np.array([float(rest.get(j.id, q[j.id])) for j in movable])
            secondary = secondary + profile.rest_pose_gain * (rest_vec - q_vec)
        if profile.joint_limit_avoidance > 0.0:
            for i, j in enumerate(movable):
                if j.upper <= j.lower:
                    continue
                center = 0.5 * (j.lower + j.upper)
                half = 0.5 * (j.upper - j.lower)
                norm = (q_vec[i] - center) / half
                secondary[i] -= profile.joint_limit_avoidance * (norm ** 3) * half
        if np.any(secondary != 0):
            secondary_u = secondary / s_w
            eff = sigma > sv_threshold
            if np.any(eff):
                V_eff_t = Vt[eff]
                proj_u = secondary_u - V_eff_t.T @ (V_eff_t @ secondary_u)
            else:
                proj_u = secondary_u
            dq = dq + s_w * proj_u

        # Task-priority rotation in pos_primary mode: solve for the dq that
        # reduces orientation error *within* the null space of the position
        # task. Distinct from Liégeois secondaries because for a Cartesian
        # rotation task, projection-then-solve gives an exact damped Newton
        # step, not a slow gradient.
        if (
            profile.mode == "pos_primary"
            and has_rot
            and profile.orientation_secondary_gain > 0.0
        ):
            eff = sigma > sv_threshold
            if np.any(eff):
                V_eff = Vt[eff].T
                N_w = np.eye(n) - V_eff @ V_eff.T
            else:
                N_w = np.eye(n)
            J_rot_w = J_rot * s_w
            J_rot_proj = J_rot_w @ N_w
            try:
                U_r, sig_r, Vt_r = np.linalg.svd(J_rot_proj, full_matrices=False)
            except np.linalg.LinAlgError:
                pass
            else:
                lam_r_sq = damp_sq + np.maximum(
                    0.0, sv_threshold * sv_threshold - sig_r * sig_r
                )
                sig_r_inv = sig_r / (sig_r * sig_r + lam_r_sq)
                u_rot = Vt_r.T @ (sig_r_inv * (U_r.T @ rot_err_raw))
                dq_rot = s_w * u_rot
                dq = dq + profile.orientation_secondary_gain * dq_rot

        dq = step * dq

        # Per-iter clamp.
        dq_inf = float(np.max(np.abs(dq))) if dq.size else 0.0
        if dq_inf > profile.max_dq_step:
            dq = dq * (profile.max_dq_step / dq_inf)

        # Apply with joint-position-limit clamp.
        candidate_q = dict(q)
        for i, j in enumerate(movable):
            new_val = q[j.id] + float(dq[i])
            if j.upper > j.lower:
                new_val = max(j.lower, min(j.upper, new_val))
            candidate_q[j.id] = new_val

        # Collision check (editor parity): if the candidate q introduces a
        # NEW colliding pair (one not already in the baseline), reject this
        # iteration and stop — q stays at the previous step.
        if collision_world is not None:
            try:
                new_pairs = frozenset(
                    tuple(sorted((p.a, p.b)))
                    for p in collision_world.check(_q_to_urdf(candidate_q))
                )
            except Exception:  # noqa: BLE001
                new_pairs = baseline_pairs
            if new_pairs - baseline_pairs:
                break

        # Per-call cap on cumulative joint motion across iterations.
        # Editor parity: when triggered, clamp candidate_q back to within the
        # cap, recompute residuals at the clamped pose, and stop iterating.
        if profile.max_total_dq_step is not None and profile.max_total_dq_step > 0:
            diff_inf = max(
                (abs(candidate_q[jid] - q_initial[jid]) for jid in candidate_q),
                default=0.0,
            )
            if diff_inf > profile.max_total_dq_step:
                scale = profile.max_total_dq_step / diff_inf
                for jid in candidate_q:
                    candidate_q[jid] = q_initial[jid] + scale * (candidate_q[jid] - q_initial[jid])
                q = candidate_q
                # `jacobian()` returns (J, T_tip) — we want the latter for FK.
                _J_final, T_final = jacobian(chain, q)
                ee_pos_final = T_final[:3, 3]
                if tcp is not None:
                    ee_pos_final = ee_pos_final + T_final[:3, :3] @ tcp
                pos_err_final = target_pos - ee_pos_final
                pos_r = float(np.linalg.norm(pos_err_final))
                if has_rot:
                    rot_err_final = _R_to_axis_angle(target_R @ T_final[:3, :3].T)
                    last_residual = math.sqrt(
                        pos_r * pos_r + float(np.linalg.norm(rot_err_final)) ** 2
                    )
                else:
                    last_residual = pos_r
                break

        q = candidate_q

    return IKResult(
        q=q, iterations=it + 1, residual=last_residual, converged=converged
    )
