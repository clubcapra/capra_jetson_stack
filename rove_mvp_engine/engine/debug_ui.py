"""Optional debug UI for the exported IK engine.

A small HTTP server (stdlib only, runs in a daemon thread) that serves
a single HTML page plus a `/state` JSON endpoint. The page polls
`/state` at ~30 Hz and renders the engine's current view of the world:

  * the latest JointState received from the real arm (per-joint bars
    inside their limits)
  * the latest Twist received (per-axis bars in [-1, 1])
  * the latest JointCommand emitted (q_dot for RESOLVED_RATE, q for
    POSITION_IK)
  * the end-effector pose computed by FK at the current q
  * freshness of each input (last packet age in ms)
  * IK solver convergence + residual for the latest POSITION_IK solve

The UI is strictly opt-in (`--gui` on the CLI). When disabled, none
of this module runs and the engine is byte-identical to before.
"""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np

from . import ik as ik_math


def _local_ipv4s() -> list[str]:
    """Best-effort list of this machine's non-loopback IPv4 addresses.

    Used to print useful URLs in the startup log when --gui-host is
    0.0.0.0. Quietly returns [] if the OS can't tell us — we'll fall
    back to whatever the user passed.
    """
    ips: set[str] = set()
    # The "connect to a public IP" trick picks the routing-interface
    # address without sending a packet.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 1))
            ips.add(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    # Hostname resolution catches additional interfaces.
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None):
            ip = info[4][0]
            if ":" not in ip and not ip.startswith("127."):
                ips.add(ip)
    except (socket.gaierror, OSError):
        pass
    return sorted(ips)

_log = logging.getLogger("ik_engine.gui")
_STATIC_DIR = Path(__file__).resolve().parent / "static"
# The exported bundle layout is:  <root>/engine/{server,debug_ui,…}.py
# and  <root>/data/{robot.urdf, meshes/, …}. Both dirs are reachable
# from this module via parent.parent.
_DATA_DIR = Path(__file__).resolve().parent.parent / "data"

# Map static file extensions to Content-Type. Limited to what the
# debug UI actually needs — no general-purpose static server here.
_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".urdf": "application/xml; charset=utf-8",
    ".xml": "application/xml; charset=utf-8",
    ".stl": "application/octet-stream",
    ".obj": "text/plain; charset=utf-8",
    ".mtl": "text/plain; charset=utf-8",
    ".dae": "application/xml; charset=utf-8",
    ".gltf": "model/gltf+json",
    ".glb": "model/gltf-binary",
    ".ply": "application/octet-stream",
}


def _rotmat_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """ZYX Euler (yaw, pitch, roll) from a 3x3 rotation matrix.

    Returns (roll, pitch, yaw) in radians. Handles the gimbal-lock
    cases at pitch = ±π/2.
    """
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return (roll, pitch, yaw)


def _snapshot(state: Any) -> dict:
    """Build the JSON the UI consumes. Tolerant of partial state — if
    we don't have a JointState yet, FK still runs against the zero q
    so the user sees the chain in its URDF home pose."""
    chain = state.chain
    try:
        T_tip, _ = ik_math.fk(chain, state.q)
        ee_pos = [float(v) for v in T_tip[:3, 3]]
        ee_rpy = list(_rotmat_to_rpy(T_tip[:3, :3]))
    except Exception:  # noqa: BLE001
        ee_pos = [0.0, 0.0, 0.0]
        ee_rpy = [0.0, 0.0, 0.0]

    joints = [
        {
            "id": j.id,
            "name": j.name,
            "type": j.type,
            "inverted": bool(j.inverted),
            "lower": float(j.lower),
            "upper": float(j.upper),
            "velocity": float(j.velocity),
            "q": float(state.q.get(j.id, 0.0)),
        }
        for j in chain.movable
    ]

    twist = None
    last_twist = getattr(state, "last_twist", None)
    if last_twist is not None:
        twist = {
            "position": {
                "x": last_twist.position.x,
                "y": last_twist.position.y,
                "z": last_twist.position.z,
            },
            "orientation": {
                "roll": last_twist.orientation.roll,
                "pitch": last_twist.orientation.pitch,
                "yaw": last_twist.orientation.yaw,
            },
            "mode": int(last_twist.mode),
            "t_us": int(last_twist.t_us),
        }

    cmd = None
    last_cmd = getattr(state, "last_cmd", None)
    if last_cmd is not None:
        cmd = {
            "joints": [
                {"name": j.name, "value": j.value} for j in last_cmd.joints
            ],
            "mode": int(last_cmd.mode),
            "converged": bool(last_cmd.converged),
            "residual": float(last_cmd.residual),
            "t_us": int(last_cmd.t_us),
        }

    return {
        "chain": {
            "base": chain.base,
            "tip": chain.tip,
            "movable_count": len(chain.movable),
        },
        "joints": joints,
        "twist": twist,
        "cmd": cmd,
        "ee_pos": ee_pos,
        "ee_rpy": ee_rpy,
        "ts": {
            "last_joint_state_us": int(getattr(state, "last_joint_state_us", 0)),
            "last_twist_us": int(getattr(state, "last_twist_us", 0)),
            "last_cmd_us": int(getattr(state, "last_cmd_us", 0)),
        },
        "stats": {
            "js_rx": int(getattr(state, "js_rx_count", 0)),
            "twist_rx": int(getattr(state, "twist_rx_count", 0)),
            "cmd_tx": int(getattr(state, "cmd_tx_count", 0)),
        },
    }


def _make_handler(state: Any):
    class _Handler(BaseHTTPRequestHandler):
        # Silence per-request stdout — we have our own logging.
        def log_message(self, *args, **kwargs):  # noqa: D401, ARG002
            return

        def do_GET(self) -> None:  # noqa: D401, N802
            path = self.path.split("?", 1)[0]
            if path == "/state":
                self._write_json(_snapshot(state))
                return
            if path.startswith("/data/"):
                # Robot URDF + meshes + textures (everything the URDF
                # loader needs in the browser). Path resolves under
                # <bundle_root>/data/.
                self._serve_file(path[len("/data/"):], _DATA_DIR)
                return
            if path == "/" or path == "":
                path = "/index.html"
            self._serve_file(path.lstrip("/"), _STATIC_DIR)

        def _serve_file(self, rel: str, root: Path) -> None:
            # Clamp to `root` (prevent traversal via "../" etc.).
            full = (root / rel).resolve()
            try:
                full.relative_to(root.resolve())
            except ValueError:
                self.send_error(403)
                return
            if not full.is_file():
                self.send_error(404)
                return
            ct = _CONTENT_TYPES.get(full.suffix.lower(), "application/octet-stream")
            data = full.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            # Permissive CORS so the URDF + mesh loaders don't choke when
            # the page is served from a different origin during dev.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

        def _write_json(self, payload: dict) -> None:
            data = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

    return _Handler


def start(state: Any, host: str = "0.0.0.0", port: int = 9504) -> ThreadingHTTPServer:
    """Spin up the debug HTTP server in a daemon thread and return the
    server handle so the caller can `shutdown()` it on exit."""
    handler = _make_handler(state)
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(
        target=server.serve_forever, daemon=True, name="ik-engine-gui"
    )
    thread.start()
    # When binding to 0.0.0.0 the literal address isn't a URL anyone
    # can dial — list the reachable IPv4s so the user knows what to
    # type from their laptop. Falls back to whatever they passed if
    # detection fails.
    if host in ("0.0.0.0", "::", ""):
        candidates = _local_ipv4s() or ["127.0.0.1"]
        urls = ", ".join(f"http://{ip}:{port}" for ip in candidates)
    else:
        urls = f"http://{host}:{port}"
    _log.warning(
        "debug GUI listening on %s:%d → %s  (chain: %s → %s, %d movable joints)",
        host, port, urls, state.chain.base, state.chain.tip, len(state.chain.movable),
    )
    return server
