"""Pure-Python protobuf codec for the IK engine wire format.

We don't depend on the `protobuf` Python package because:
  * The engine is meant to drop into any environment with just numpy,
    no codegen step (`protoc`) required.
  * Our schema is small (5 message types, all primitive-typed), so a
    hand-rolled codec is ~150 lines and trivial to audit.
  * Callers can still use the bundled .proto + their own codegen
    on their side — the wire bytes are standard protobuf.

If you want full protobuf-library support instead, run:

    python -m grpc_tools.protoc -I engine --python_out=engine engine/messages.proto

then replace this module's `encode_*` / `decode_*` calls with the
generated `_pb2` accessors. Wire-compatible.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

# Protobuf wire types we use. (Ref: https://protobuf.dev/programming-guides/encoding/)
_WT_VARINT = 0       # int32, int64, uint32, uint64, bool, enum
_WT_I64 = 1          # fixed64, sfixed64, double — unused
_WT_LEN = 2          # string, bytes, embedded messages, packed repeated
_WT_I32 = 5          # fixed32, sfixed32, float

# Mode enum values must match messages.proto.
RESOLVED_RATE = 0
POSITION_IK = 1


# ---- low-level codec ----

def _encode_varint(n: int) -> bytes:
    if n < 0:
        # Two's complement for negative ints — only relevant for sint*; we
        # don't use signed varints, but be explicit so misuse fails loudly.
        raise ValueError("negative varint not supported (use sint or fixed)")
    out = bytearray()
    while n > 0x7F:
        out.append((n & 0x7F) | 0x80)
        n >>= 7
    out.append(n & 0x7F)
    return bytes(out)


def _decode_varint(buf: bytes, pos: int) -> tuple[int, int]:
    n = 0
    shift = 0
    while True:
        if pos >= len(buf):
            raise ValueError("truncated varint")
        b = buf[pos]
        pos += 1
        n |= (b & 0x7F) << shift
        if not (b & 0x80):
            return n, pos
        shift += 7
        if shift > 63:
            raise ValueError("varint too long")


def _encode_tag(field_num: int, wire_type: int) -> bytes:
    return _encode_varint((field_num << 3) | wire_type)


def _encode_float(field_num: int, v: float) -> bytes:
    return _encode_tag(field_num, _WT_I32) + struct.pack("<f", v)


def _encode_uint64(field_num: int, v: int) -> bytes:
    return _encode_tag(field_num, _WT_VARINT) + _encode_varint(v)


def _encode_enum(field_num: int, v: int) -> bytes:
    return _encode_tag(field_num, _WT_VARINT) + _encode_varint(v)


def _encode_bool(field_num: int, v: bool) -> bytes:
    return _encode_tag(field_num, _WT_VARINT) + _encode_varint(1 if v else 0)


def _encode_string(field_num: int, s: str) -> bytes:
    data = s.encode("utf-8")
    return _encode_tag(field_num, _WT_LEN) + _encode_varint(len(data)) + data


def _encode_msg(field_num: int, msg_bytes: bytes) -> bytes:
    return _encode_tag(field_num, _WT_LEN) + _encode_varint(len(msg_bytes)) + msg_bytes


def _skip_field(buf: bytes, pos: int, wire_type: int) -> int:
    """Advance past an unknown field (proto3 forward-compat)."""
    if wire_type == _WT_VARINT:
        _, pos = _decode_varint(buf, pos)
    elif wire_type == _WT_LEN:
        ln, pos = _decode_varint(buf, pos)
        pos += ln
    elif wire_type == _WT_I32:
        pos += 4
    elif wire_type == _WT_I64:
        pos += 8
    else:
        raise ValueError(f"unknown wire type {wire_type}")
    return pos


# ---- message dataclasses (mirror messages.proto) ----

@dataclass
class Orientation:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class NamedFloat:
    name: str = ""
    value: float = 0.0


@dataclass
class JointState:
    joints: list[NamedFloat] = field(default_factory=list)
    t_us: int = 0


@dataclass
class Twist:
    orientation: Orientation = field(default_factory=Orientation)
    position: Vector3 = field(default_factory=Vector3)
    mode: int = RESOLVED_RATE
    t_us: int = 0


@dataclass
class JointCommand:
    joints: list[NamedFloat] = field(default_factory=list)
    mode: int = RESOLVED_RATE
    t_us: int = 0
    converged: bool = False
    residual: float = 0.0


# ---- per-message codecs ----

def encode_orientation(o: Orientation) -> bytes:
    return _encode_float(1, o.yaw) + _encode_float(2, o.pitch) + _encode_float(3, o.roll)


def encode_vector3(v: Vector3) -> bytes:
    return _encode_float(1, v.x) + _encode_float(2, v.y) + _encode_float(3, v.z)


def encode_named_float(nf: NamedFloat) -> bytes:
    return _encode_string(1, nf.name) + _encode_float(2, nf.value)


def encode_joint_state(js: JointState) -> bytes:
    out = b"".join(_encode_msg(1, encode_named_float(j)) for j in js.joints)
    out += _encode_uint64(2, js.t_us)
    return out


def encode_twist(t: Twist) -> bytes:
    out = _encode_msg(1, encode_orientation(t.orientation))
    out += _encode_msg(2, encode_vector3(t.position))
    out += _encode_enum(3, t.mode)
    out += _encode_uint64(4, t.t_us)
    return out


def encode_joint_command(jc: JointCommand) -> bytes:
    out = b"".join(_encode_msg(1, encode_named_float(j)) for j in jc.joints)
    out += _encode_enum(2, jc.mode)
    out += _encode_uint64(3, jc.t_us)
    out += _encode_bool(4, jc.converged)
    out += _encode_float(5, jc.residual)
    return out


def _decode_orientation(buf: bytes) -> Orientation:
    o = Orientation()
    pos = 0
    while pos < len(buf):
        tag, pos = _decode_varint(buf, pos)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == _WT_I32:
            o.yaw, = struct.unpack_from("<f", buf, pos); pos += 4
        elif fn == 2 and wt == _WT_I32:
            o.pitch, = struct.unpack_from("<f", buf, pos); pos += 4
        elif fn == 3 and wt == _WT_I32:
            o.roll, = struct.unpack_from("<f", buf, pos); pos += 4
        else:
            pos = _skip_field(buf, pos, wt)
    return o


def _decode_vector3(buf: bytes) -> Vector3:
    v = Vector3()
    pos = 0
    while pos < len(buf):
        tag, pos = _decode_varint(buf, pos)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == _WT_I32:
            v.x, = struct.unpack_from("<f", buf, pos); pos += 4
        elif fn == 2 and wt == _WT_I32:
            v.y, = struct.unpack_from("<f", buf, pos); pos += 4
        elif fn == 3 and wt == _WT_I32:
            v.z, = struct.unpack_from("<f", buf, pos); pos += 4
        else:
            pos = _skip_field(buf, pos, wt)
    return v


def _decode_named_float(buf: bytes) -> NamedFloat:
    nf = NamedFloat()
    pos = 0
    while pos < len(buf):
        tag, pos = _decode_varint(buf, pos)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == _WT_LEN:
            ln, pos = _decode_varint(buf, pos)
            nf.name = buf[pos:pos + ln].decode("utf-8")
            pos += ln
        elif fn == 2 and wt == _WT_I32:
            nf.value, = struct.unpack_from("<f", buf, pos); pos += 4
        else:
            pos = _skip_field(buf, pos, wt)
    return nf


def decode_joint_state(buf: bytes) -> JointState:
    js = JointState()
    pos = 0
    while pos < len(buf):
        tag, pos = _decode_varint(buf, pos)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == _WT_LEN:
            ln, pos = _decode_varint(buf, pos)
            js.joints.append(_decode_named_float(buf[pos:pos + ln]))
            pos += ln
        elif fn == 2 and wt == _WT_VARINT:
            js.t_us, pos = _decode_varint(buf, pos)
        else:
            pos = _skip_field(buf, pos, wt)
    return js


def decode_twist(buf: bytes) -> Twist:
    t = Twist()
    pos = 0
    while pos < len(buf):
        tag, pos = _decode_varint(buf, pos)
        fn, wt = tag >> 3, tag & 7
        if fn == 1 and wt == _WT_LEN:
            ln, pos = _decode_varint(buf, pos)
            t.orientation = _decode_orientation(buf[pos:pos + ln])
            pos += ln
        elif fn == 2 and wt == _WT_LEN:
            ln, pos = _decode_varint(buf, pos)
            t.position = _decode_vector3(buf[pos:pos + ln])
            pos += ln
        elif fn == 3 and wt == _WT_VARINT:
            t.mode, pos = _decode_varint(buf, pos)
        elif fn == 4 and wt == _WT_VARINT:
            t.t_us, pos = _decode_varint(buf, pos)
        else:
            pos = _skip_field(buf, pos, wt)
    return t
