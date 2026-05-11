"""Collision world for the IK engine — exact clone of the editor's
two-tier mesh check, adapted to URDF data instead of a forgebot Project.

Pipeline mirrors `forge/.../validation/collision.py`:

  1. On startup, parse `data/robot.urdf`. Build a kinematic graph
     (links + joints with origins/axes), load every collision mesh
     via trimesh, derive a convex hull per link.
  2. Register full meshes in one `trimesh.collision.CollisionManager`
     (narrow phase) and hulls in another (broad phase).
  3. Per query, do URDF FK using the engine's chain q values (chain
     joints get their q; everything else stays at its URDF zero) and
     push the resulting world transforms to both managers.
  4. Broad-phase finds candidate pairs; narrow-phase confirms with
     full-mesh FCL.collide. Adjacent link pairs (parent/child via the
     same joint) are skipped.

If `trimesh` / `python-fcl` aren't importable, `try_build_world` returns
None and the engine runs without collision checks (degrade gracefully).
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

_log = logging.getLogger("ik_engine.collision")


# ---- URDF parsing ------------------------------------------------------


@dataclass
class _URDFCollision:
    """One <collision> element from URDF, in link-local frame."""

    geom_kind: str  # "mesh" | "box" | "sphere" | "cylinder"
    mesh_filename: str | None = None  # e.g. "meshes/Base.glb"
    box_size: tuple[float, float, float] | None = None
    sphere_radius: float | None = None
    cyl_radius: float | None = None
    cyl_length: float | None = None
    origin: np.ndarray = field(default_factory=lambda: np.eye(4))


@dataclass
class _URDFLink:
    name: str
    collisions: list[_URDFCollision] = field(default_factory=list)


@dataclass
class _URDFJoint:
    name: str
    type: str  # revolute|continuous|prismatic|fixed
    parent: str
    child: str
    origin: np.ndarray  # 4x4 parent→joint static xform
    axis: np.ndarray  # (3,) in joint-local frame


def _origin_from_xml(elem: ET.Element | None) -> np.ndarray:
    if elem is None:
        return np.eye(4)
    xyz_str = elem.get("xyz", "0 0 0").split()
    rpy_str = elem.get("rpy", "0 0 0").split()
    x, y, z = (float(v) for v in xyz_str)
    r, p, yw = (float(v) for v in rpy_str)
    # ZYX intrinsic / XYZ extrinsic = Rz(yw) Ry(p) Rx(r) (URDF convention)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(yw), math.sin(yw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (x, y, z)
    return T


def _parse_urdf(urdf_path: Path) -> tuple[dict[str, _URDFLink], dict[str, _URDFJoint]]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links: dict[str, _URDFLink] = {}
    joints: dict[str, _URDFJoint] = {}

    for link_elem in root.findall("link"):
        name = link_elem.get("name", "")
        if not name:
            continue
        link = _URDFLink(name=name)
        for col in link_elem.findall("collision"):
            origin = _origin_from_xml(col.find("origin"))
            geom = col.find("geometry")
            if geom is None:
                continue
            mesh_el = geom.find("mesh")
            box_el = geom.find("box")
            sph_el = geom.find("sphere")
            cyl_el = geom.find("cylinder")
            c: _URDFCollision | None = None
            if mesh_el is not None:
                c = _URDFCollision(
                    geom_kind="mesh",
                    mesh_filename=mesh_el.get("filename"),
                    origin=origin,
                )
            elif box_el is not None:
                sx, sy, sz = (float(v) for v in box_el.get("size", "1 1 1").split())
                c = _URDFCollision(geom_kind="box", box_size=(sx, sy, sz), origin=origin)
            elif sph_el is not None:
                c = _URDFCollision(
                    geom_kind="sphere",
                    sphere_radius=float(sph_el.get("radius", "0.5")),
                    origin=origin,
                )
            elif cyl_el is not None:
                c = _URDFCollision(
                    geom_kind="cylinder",
                    cyl_radius=float(cyl_el.get("radius", "0.5")),
                    cyl_length=float(cyl_el.get("length", "1.0")),
                    origin=origin,
                )
            if c is not None:
                link.collisions.append(c)
        links[name] = link

    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name", "")
        jtype = joint_elem.get("type", "fixed")
        parent_el = joint_elem.find("parent")
        child_el = joint_elem.find("child")
        if parent_el is None or child_el is None:
            continue
        axis_el = joint_elem.find("axis")
        axis = (
            np.array([float(v) for v in axis_el.get("xyz", "0 0 1").split()], dtype=np.float64)
            if axis_el is not None
            else np.array([0.0, 0.0, 1.0])
        )
        joints[name] = _URDFJoint(
            name=name,
            type=jtype,
            parent=parent_el.get("link", ""),
            child=child_el.get("link", ""),
            origin=_origin_from_xml(joint_elem.find("origin")),
            axis=axis,
        )
    return links, joints


# ---- FK over the URDF tree --------------------------------------------


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.eye(3)
    a = axis / n
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _joint_xform(j: _URDFJoint, q: float) -> np.ndarray:
    T = np.eye(4)
    if j.type in ("revolute", "continuous"):
        T[:3, :3] = _axis_angle(j.axis, q)
    elif j.type == "prismatic":
        n = float(np.linalg.norm(j.axis)) or 1.0
        T[:3, 3] = (j.axis / n) * q
    return T


def _urdf_fk(
    links: dict[str, _URDFLink],
    joints: dict[str, _URDFJoint],
    q: dict[str, float],
) -> dict[str, np.ndarray]:
    """Walk the URDF kinematic tree from each root and return per-link
    world transforms. Joints not in q stay at zero."""
    # Adjacency: child_link → parent_joint
    parent_joint: dict[str, str] = {}
    for jname, j in joints.items():
        parent_joint[j.child] = jname
    roots = [name for name in links if name not in parent_joint]

    worlds: dict[str, np.ndarray] = {}

    def visit(link_name: str, T_parent: np.ndarray) -> None:
        if link_name in worlds:
            return
        worlds[link_name] = T_parent
        for jname, j in joints.items():
            if j.parent == link_name:
                T_joint = T_parent @ j.origin @ _joint_xform(j, q.get(jname, 0.0))
                visit(j.child, T_joint)

    for r in roots:
        visit(r, np.eye(4))
    return worlds


# ---- Mesh loading + collision world -----------------------------------


def _load_mesh(path: Path) -> Any | None:
    try:
        import trimesh  # type: ignore[import-untyped]
    except ImportError:
        return None
    try:
        loaded = trimesh.load(path, force="mesh", process=False)
    except Exception as e:  # noqa: BLE001
        _log.warning("mesh load failed: %s (%s)", path, e)
        return None
    if loaded is None or not hasattr(loaded, "vertices"):
        return None
    if len(loaded.vertices) == 0 or len(getattr(loaded, "faces", [])) == 0:
        return None
    return loaded


def _geom_to_mesh(c: _URDFCollision, meshes_dir: Path) -> Any | None:
    try:
        import trimesh  # type: ignore[import-untyped]
    except ImportError:
        return None
    mesh: Any | None = None
    if c.geom_kind == "mesh" and c.mesh_filename:
        mesh = _load_mesh(meshes_dir.parent / c.mesh_filename)
    elif c.geom_kind == "box" and c.box_size is not None:
        mesh = trimesh.creation.box(extents=list(c.box_size))
    elif c.geom_kind == "sphere" and c.sphere_radius is not None:
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=c.sphere_radius)
    elif c.geom_kind == "cylinder" and c.cyl_radius is not None:
        mesh = trimesh.creation.cylinder(
            radius=c.cyl_radius, height=c.cyl_length or 1.0, sections=24
        )
    if mesh is None:
        return None
    if not np.allclose(c.origin, np.eye(4)):
        mesh = mesh.copy()
        mesh.apply_transform(c.origin)
    return mesh


def _to_convex_hull(mesh: Any) -> Any:
    try:
        hull = mesh.convex_hull
    except Exception:
        return mesh
    if hull is None or len(getattr(hull, "vertices", [])) < 4:
        return mesh
    return hull


@dataclass(frozen=True)
class CollisionPair:
    a: str
    b: str


class CollisionWorld:
    """Two-tier mesh collision checker (hull broad-phase, full-mesh narrow)
    over the full URDF kinematic tree. Hot path is `check(q)`."""

    def __init__(
        self,
        urdf_path: Path,
        broad_mgr: Any,
        narrow_mgr: Any,
        links: dict[str, _URDFLink],
        joints: dict[str, _URDFJoint],
        registered: set[str],
        adjacency: set[tuple[str, str]],
    ) -> None:
        self.urdf_path = urdf_path
        self.broad_mgr = broad_mgr
        self.narrow_mgr = narrow_mgr
        self.links = links
        self.joints = joints
        self.registered = registered
        self.adjacency = adjacency

    def check(self, q: dict[str, float], *, skip_adjacent: bool = True) -> list[CollisionPair]:
        worlds = _urdf_fk(self.links, self.joints, q)
        for name in self.registered:
            T = worlds.get(name)
            if T is None:
                continue
            try:
                self.broad_mgr.set_transform(name, T)
                self.narrow_mgr.set_transform(name, T)
            except KeyError:
                pass

        is_collision, names = self.broad_mgr.in_collision_internal(return_names=True)
        if not is_collision:
            return []
        out: list[CollisionPair] = []
        seen: set[tuple[str, str]] = set()
        for pair in names:
            a, b = sorted(pair)
            if a == b:
                continue
            key = (a, b)
            if key in seen:
                continue
            if skip_adjacent and key in self.adjacency:
                continue
            seen.add(key)
            if not self._meshes_collide(a, b):
                continue
            out.append(CollisionPair(a=a, b=b))
        return out

    def _meshes_collide(self, a: str, b: str) -> bool:
        try:
            import fcl  # type: ignore[import-untyped]
        except ImportError:
            return True
        obj_a = self.narrow_mgr._objs.get(a, {}).get("obj")
        obj_b = self.narrow_mgr._objs.get(b, {}).get("obj")
        if obj_a is None or obj_b is None:
            return True
        request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=False)
        result = fcl.CollisionResult()
        fcl.collide(obj_a, obj_b, request, result)
        return bool(result.is_collision)


def try_build_world(urdf_path: Path) -> CollisionWorld | None:
    """Build the collision world from a URDF. Returns None and logs a
    warning if trimesh/fcl aren't installed, so the engine starts cleanly
    on bare boxes."""
    try:
        import trimesh  # type: ignore[import-untyped]
        from trimesh.collision import CollisionManager  # type: ignore[import-untyped]
    except ImportError:
        _log.warning(
            "trimesh / python-fcl not importable — collision checks disabled. "
            "Install them on the runtime host to enable respect_collisions."
        )
        return None

    if not urdf_path.is_file():
        _log.warning("URDF not found at %s — collision checks disabled", urdf_path)
        return None

    links, joints = _parse_urdf(urdf_path)
    meshes_dir = urdf_path.parent / "meshes"

    broad_mgr = CollisionManager()
    narrow_mgr = CollisionManager()
    registered: set[str] = set()

    for name, link in links.items():
        parts: list[Any] = []
        for c in link.collisions:
            m = _geom_to_mesh(c, meshes_dir)
            if m is not None and len(m.vertices) > 0:
                parts.append(m)
        if not parts:
            continue
        full = parts[0] if len(parts) == 1 else trimesh.util.concatenate(parts)
        if full is None or len(getattr(full, "vertices", [])) == 0:
            continue
        hull = _to_convex_hull(full)
        broad_mgr.add_object(name, hull, transform=np.eye(4))
        narrow_mgr.add_object(name, full, transform=np.eye(4))
        registered.add(name)

    # Adjacency: any parent_link, child_link pair sharing a joint is skipped.
    adjacency: set[tuple[str, str]] = set()
    for j in joints.values():
        if j.parent and j.child:
            adjacency.add(tuple(sorted((j.parent, j.child))))  # type: ignore[arg-type]

    _log.info(
        "collision world built: %d collidable links, %d adjacency-skip pairs",
        len(registered), len(adjacency),
    )
    return CollisionWorld(
        urdf_path=urdf_path,
        broad_mgr=broad_mgr,
        narrow_mgr=narrow_mgr,
        links=links,
        joints=joints,
        registered=registered,
        adjacency=adjacency,
    )
