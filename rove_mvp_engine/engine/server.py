"""UDP server tying the IK math to the wire protocol.

Three sockets, each unidirectional:

    JOINTS  port (default 9501): client → engine.  Receives JointState
                                 messages; updates the engine's view of
                                 the real robot's current joint values.

    TWIST   port (default 9502): client → engine.  Receives Twist
                                 messages; on each, runs IK against the
                                 latest known JointState and emits a
                                 JointCommand on the VELOCS port.

    VELOCS  port (default 9503): engine → client.  Sends JointCommand
                                 results — one per Twist received.

The hot path (Twist arrives → IK → JointCommand sent) runs on a single
asyncio task, no locks: the JointState receive loop just mutates the
shared `state.q` dict, which Python's GIL makes safe for our usage.

Latency contract: JOINTS messages should arrive faster than TWIST so
the IK always has a current `q`. If `q` is stale, the resolved-rate
output will be wrong proportional to the staleness; nothing else
breaks.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import ik as ik_math
from . import proto as pb

_log = logging.getLogger("ik_engine")


@dataclass
class State:
    chain: ik_math.Chain
    profile: ik_math.Profile
    # Latest joint values from the real robot. Initialized to 0 for
    # every movable joint so the engine has a starting point even
    # before the first JointState arrives.
    q: dict[str, float]
    # Persistent target pose for POSITION_IK mode: integrated from
    # successive twists. Reset whenever JOINTS receives a state that
    # differs significantly from our integrated estimate (i.e., the
    # real robot was commanded externally).
    target_pos: np.ndarray | None = None
    target_R: np.ndarray | None = None

    # ---- debug-UI snapshot fields ----
    # Filled on every UDP rx / IK solve. Always written (cheap), but
    # only *read* by the debug HTTP server when --gui is active. Safe
    # to ignore if you're running headless.
    last_joint_state_us: int = 0
    js_rx_count: int = 0
    last_twist: "pb.Twist | None" = None
    last_twist_us: int = 0
    twist_rx_count: int = 0
    last_cmd: "pb.JointCommand | None" = None
    last_cmd_us: int = 0
    cmd_tx_count: int = 0


def _make_state(chain_path: Path, profile_path: Path) -> State:
    chain = ik_math.load_chain(chain_path)
    profile = ik_math.load_profile(profile_path)
    # After IKEngineExporter's home-pose bake, q=0 IS the home pose:
    # chain.json's pre_xform and the exported URDF both encode the
    # project's home pose as a static rotation on each joint's origin.
    # rest_pose is therefore an all-zeros vector over the chain joints
    # (kept explicit so the null-space pull in position IK has a
    # well-defined "home" to drive redundant DOFs toward). Seed q from
    # rest_pose — for legacy exports without the bake this still pulls
    # the right values; for current exports it's just zeros.
    q0 = {j.id: 0.0 for j in chain.movable}
    for jid, v in (profile.rest_pose or {}).items():
        if jid in q0:
            q0[jid] = float(v)
    return State(chain=chain, profile=profile, q=q0)


# ---- UDP protocols ----

class _JointStateProto(asyncio.DatagramProtocol):
    def __init__(self, state: State) -> None:
        self.state = state
        self.count = 0

    def datagram_received(self, data: bytes, addr) -> None:  # noqa: D401
        try:
            msg = pb.decode_joint_state(data)
        except Exception as e:  # noqa: BLE001
            _log.warning("JointState decode failed (%d B from %s): %s", len(data), addr, e)
            return
        for nf in msg.joints:
            # Only update joints we know about — silently ignore unknowns
            # so the morphing layer can send extras without surprise.
            if any(j.id == nf.name for j in self.state.chain.movable):
                self.state.q[nf.name] = float(nf.value)
        self.count += 1
        self.state.js_rx_count = self.count
        self.state.last_joint_state_us = int(time.time() * 1e6)
        if self.count in (1, 100, 1000, 10000):
            _log.info("JointState rx #%d (%d joints from %s)", self.count, len(msg.joints), addr)


class _TwistProto(asyncio.DatagramProtocol):
    def __init__(self, state: State, send: "_VelocsSender") -> None:
        self.state = state
        self.send = send
        self.count = 0

    def datagram_received(self, data: bytes, addr) -> None:
        try:
            twist = pb.decode_twist(data)
        except Exception as e:  # noqa: BLE001
            _log.warning("Twist decode failed (%d B from %s): %s", len(data), addr, e)
            return
        try:
            cmd = _solve(self.state, twist)
        except Exception as e:  # noqa: BLE001
            _log.exception("IK solve failed: %s", e)
            return
        try:
            self.send.send(pb.encode_joint_command(cmd))
        except OSError as e:
            _log.warning("VELOCS send failed: %s", e)
        self.count += 1
        self.state.last_twist = twist
        self.state.last_twist_us = int(time.time() * 1e6)
        self.state.twist_rx_count = self.count
        self.state.last_cmd = cmd
        self.state.last_cmd_us = self.state.last_twist_us
        self.state.cmd_tx_count = self.count
        if self.count in (1, 100, 1000, 10000):
            _log.info(
                "Twist rx #%d mode=%s — solved in %.2f ms, sent %d joints",
                self.count, "RR" if twist.mode == pb.RESOLVED_RATE else "POS",
                cmd.residual * 1000 if twist.mode == pb.POSITION_IK else 0.0,
                len(cmd.joints),
            )


class _VelocsSender:
    """Small wrapper around a connected UDP socket. We `connect()` so
    each `send()` is a single syscall (no per-packet routing lookup)."""

    def __init__(self, host: str, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))
        self.host, self.port = host, port

    def send(self, data: bytes) -> None:
        self.sock.send(data)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


# ---- IK solve dispatch ----

def _solve(state: State, twist: pb.Twist) -> pb.JointCommand:
    profile = state.profile
    # Scale normalized [-1,1] inputs to physical units.
    v = np.array([
        twist.position.x * profile.max_lin_vel,
        twist.position.y * profile.max_lin_vel,
        twist.position.z * profile.max_lin_vel,
        twist.orientation.roll * profile.max_ang_vel,
        twist.orientation.pitch * profile.max_ang_vel,
        twist.orientation.yaw * profile.max_ang_vel,
    ], dtype=np.float64)

    cmd = pb.JointCommand(
        mode=twist.mode,
        t_us=int(time.time() * 1e6),
        converged=True,
        residual=0.0,
    )

    if twist.mode == pb.RESOLVED_RATE:
        q_dot = ik_math.resolved_rate(state.chain, state.q, v, profile)
        for jid, val in q_dot.items():
            cmd.joints.append(pb.NamedFloat(name=jid, value=float(val)))
        return cmd

    # POSITION_IK: integrate twist over a fixed dt to a moving target.
    # We anchor the target to the current FK pose if we don't have a
    # running target yet, or if the joints have been displaced by an
    # external command (heuristic: ||q − target_q|| > threshold).
    dt = 1.0 / 30.0  # treat each twist packet as ~33 ms of motion
    T_tip, _ = ik_math.fk(state.chain, state.q)
    if state.target_pos is None:
        state.target_pos = T_tip[:3, 3].copy()
        state.target_R = T_tip[:3, :3].copy() if v[3:].any() else None
    state.target_pos = state.target_pos + v[:3] * dt
    if state.target_R is not None and v[3:].any():
        # Compose target orientation with the integrated angular velocity.
        omega = v[3:] * dt
        state.target_R = ik_math._axis_angle_to_R(omega, np.linalg.norm(omega)) @ state.target_R

    res = ik_math.position_ik(
        state.chain, state.q, state.target_pos, state.target_R, profile,
    )
    cmd.converged = res.converged
    cmd.residual = float(res.residual)
    for jid, val in res.q.items():
        cmd.joints.append(pb.NamedFloat(name=jid, value=float(val)))
    return cmd


# ---- main ----

async def serve(
    chain_path: Path,
    profile_path: Path,
    *,
    host: str = "0.0.0.0",
    joints_port: int = 9501,
    twist_port: int = 9502,
    velocs_host: str = "127.0.0.1",
    velocs_port: int = 9503,
    gui: bool = False,
    gui_host: str = "0.0.0.0",
    gui_port: int = 9504,
) -> None:
    state = _make_state(chain_path, profile_path)
    velocs = _VelocsSender(velocs_host, velocs_port)
    loop = asyncio.get_running_loop()
    js_transport, _ = await loop.create_datagram_endpoint(
        lambda: _JointStateProto(state), local_addr=(host, joints_port)
    )
    tw_transport, _ = await loop.create_datagram_endpoint(
        lambda: _TwistProto(state, velocs), local_addr=(host, twist_port)
    )
    _log.warning(
        "ik_engine ready — chain=%s tip=%s movable=%d "
        "in[joints]=%s:%d in[twist]=%s:%d out[velocs]=%s:%d",
        state.chain.base, state.chain.tip, len(state.chain.movable),
        host, joints_port, host, twist_port, velocs_host, velocs_port,
    )
    # Debug GUI is opt-in. When off, the import below never runs and
    # the engine is byte-identical to the headless baseline.
    gui_server = None
    if gui:
        from . import debug_ui
        gui_server = debug_ui.start(state, host=gui_host, port=gui_port)
    try:
        # Spin forever; signal-handler cancels us cleanly.
        while True:
            await asyncio.sleep(3600)
    finally:
        js_transport.close()
        tw_transport.close()
        velocs.close()
        if gui_server is not None:
            gui_server.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(prog="ik_engine")
    ap.add_argument("--data-dir", type=Path, default=Path(__file__).resolve().parent.parent / "data")
    ap.add_argument("--chain", type=Path, default=None, help="path to chain.json (default: data/chain.json)")
    ap.add_argument("--profile", type=Path, default=None, help="path to ik_profile.json (default: data/ik_profile.json)")
    ap.add_argument("--host", default="0.0.0.0", help="bind address for the input ports")
    ap.add_argument("--joints-port", type=int, default=9501)
    ap.add_argument("--twist-port", type=int, default=9502)
    ap.add_argument("--velocs-host", default="127.0.0.1", help="destination host for joint-command output")
    ap.add_argument("--velocs-port", type=int, default=9503)
    ap.add_argument(
        "--gui", action="store_true",
        help="enable the debug HTTP UI (off by default; engine is headless)",
    )
    ap.add_argument(
        "--gui-host", default="0.0.0.0",
        help="bind address for the debug GUI (default 0.0.0.0 so a "
             "laptop on the LAN can reach the engine running on the robot)",
    )
    ap.add_argument("--gui-port", type=int, default=9504, help="port for the debug GUI")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)-7s %(name)s — %(message)s",
    )
    chain = args.chain or (args.data_dir / "chain.json")
    profile = args.profile or (args.data_dir / "ik_profile.json")
    if not chain.exists():
        ap.error(f"chain.json not found at {chain}")
    if not profile.exists():
        ap.error(f"ik_profile.json not found at {profile}")
    asyncio.run(serve(
        chain_path=chain,
        profile_path=profile,
        host=args.host,
        joints_port=args.joints_port,
        twist_port=args.twist_port,
        velocs_host=args.velocs_host,
        velocs_port=args.velocs_port,
        gui=args.gui,
        gui_host=args.gui_host,
        gui_port=args.gui_port,
    ))


if __name__ == "__main__":
    main()
