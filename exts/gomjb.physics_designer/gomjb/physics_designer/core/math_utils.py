"""Pure math helpers built on ``pxr.Gf``.

Conventions follow USD/Gf: row vectors, ``p_world = p_local * M`` (``M.Transform(p)``),
and a composed transform ``A * B`` means "apply A, then B".
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple, Union

from pxr import Gf

VecLike = Union[Gf.Vec3d, Gf.Vec3f, Sequence[float]]

AXIS_VECTORS = {
    "X": Gf.Vec3d(1, 0, 0),
    "Y": Gf.Vec3d(0, 1, 0),
    "Z": Gf.Vec3d(0, 0, 1),
}
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}
EPS = 1e-9


def vec3d(v: VecLike) -> Gf.Vec3d:
    return Gf.Vec3d(float(v[0]), float(v[1]), float(v[2]))


def quantize(value: float, step: float, offset: float = 0.0) -> float:
    """Round ``value`` to the nearest multiple of ``step`` (shifted by ``offset``)."""
    if step <= 0:
        return value
    return round((value - offset) / step) * step + offset


def quantize_vec(v: VecLike, step: float, offset: VecLike = (0.0, 0.0, 0.0), axes: str = "XYZ") -> Gf.Vec3d:
    out = vec3d(v)
    for axis in axes.upper():
        i = AXIS_INDEX[axis]
        out[i] = quantize(out[i], step, float(offset[i]))
    return out


def axis_vector(axis: Union[str, VecLike]) -> Gf.Vec3d:
    """Return a unit vector for an axis token ("X", "-Y", ...) or an arbitrary vector."""
    if isinstance(axis, str):
        sign = -1.0 if axis.startswith("-") else 1.0
        return AXIS_VECTORS[axis.lstrip("+-").upper()] * sign
    v = vec3d(axis)
    length = v.GetLength()
    if length < EPS:
        raise ValueError("Axis vector must be non-zero")
    return v / length


def rotation_from_to(src: VecLike, dst: VecLike) -> Gf.Rotation:
    """Shortest rotation that maps direction ``src`` onto direction ``dst``."""
    a = axis_vector(src)
    b = axis_vector(dst)
    dot = max(-1.0, min(1.0, Gf.Dot(a, b)))
    if dot > 1.0 - 1e-12:
        return Gf.Rotation(Gf.Vec3d(1, 0, 0), 0.0)
    if dot < -1.0 + 1e-12:
        # 180 degrees: pick any axis perpendicular to ``a``
        perp = Gf.Cross(a, Gf.Vec3d(1, 0, 0))
        if perp.GetLength() < 1e-6:
            perp = Gf.Cross(a, Gf.Vec3d(0, 1, 0))
        return Gf.Rotation(perp.GetNormalized(), 180.0)
    return Gf.Rotation(a, b)


def frame_from_axis(primary_axis: str, direction: VecLike) -> Gf.Rotation:
    """Rotation whose local ``primary_axis`` (X/Y/Z) points along world ``direction``."""
    return rotation_from_to(AXIS_VECTORS[primary_axis.upper()], direction)


def make_transform(
    translation: VecLike = (0, 0, 0),
    rotation: Union[Gf.Rotation, Gf.Quatd, Gf.Quatf, None] = None,
    scale: VecLike = (1, 1, 1),
) -> Gf.Matrix4d:
    """Compose scale -> rotate -> translate into a single matrix."""
    t = Gf.Transform()
    t.SetScale(vec3d(scale))
    if rotation is not None:
        t.SetRotation(to_rotation(rotation))
    t.SetTranslation(vec3d(translation))
    return t.GetMatrix()


def to_rotation(rot: Union[Gf.Rotation, Gf.Quatd, Gf.Quatf, Gf.Quaternion]) -> Gf.Rotation:
    if isinstance(rot, Gf.Rotation):
        return rot
    if isinstance(rot, (Gf.Quatd, Gf.Quatf)):
        return Gf.Rotation(Gf.Quatd(rot.GetReal(), Gf.Vec3d(rot.GetImaginary())))
    return Gf.Rotation(rot)


def to_quatd(rot: Union[Gf.Rotation, Gf.Quatd, Gf.Quatf]) -> Gf.Quatd:
    if isinstance(rot, Gf.Quatd):
        return rot
    if isinstance(rot, Gf.Quatf):
        return Gf.Quatd(rot.GetReal(), Gf.Vec3d(rot.GetImaginary()))
    return to_rotation(rot).GetQuat()


def decompose(matrix: Gf.Matrix4d) -> Tuple[Gf.Vec3d, Gf.Quatd, Gf.Vec3d]:
    """Split a matrix into (translation, rotation quaternion, scale). Shear is dropped."""
    t = Gf.Transform(matrix)
    return Gf.Vec3d(t.GetTranslation()), t.GetRotation().GetQuat(), Gf.Vec3d(t.GetScale())


def rigid_part(matrix: Gf.Matrix4d) -> Gf.Matrix4d:
    """Matrix with scale/shear removed (rotation + translation only)."""
    return Gf.Matrix4d(matrix).RemoveScaleShear()


def rotation_of(matrix: Gf.Matrix4d) -> Gf.Rotation:
    return rigid_part(matrix).ExtractRotation()


def euler_xyz_from_quat(q: Union[Gf.Quatd, Gf.Quatf]) -> Gf.Vec3d:
    """XYZ euler angles in degrees, matching ``xformOp:rotateXYZ`` (X applied first)."""
    m = Gf.Matrix4d().SetRotate(to_rotation(q))
    sb = max(-1.0, min(1.0, -m[0][2]))
    b = math.asin(sb)
    if abs(sb) < 1.0 - 1e-9:
        a = math.atan2(m[1][2], m[2][2])
        c = math.atan2(m[0][1], m[0][0])
    else:  # gimbal lock: fold the Z rotation into X
        c = 0.0
        a = math.atan2(m[1][0] / sb, m[1][1])
    return Gf.Vec3d(math.degrees(a), math.degrees(b), math.degrees(c))


def quat_from_euler_xyz(angles_deg: VecLike) -> Gf.Quatd:
    rx = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(angles_deg[0]))
    ry = Gf.Rotation(Gf.Vec3d(0, 1, 0), float(angles_deg[1]))
    rz = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(angles_deg[2]))
    return (rx * ry * rz).GetQuat()


def snap_rotation(q: Union[Gf.Quatd, Gf.Quatf], step_deg: float) -> Gf.Quatd:
    """Quantize the XYZ euler angles of ``q`` to multiples of ``step_deg``."""
    if step_deg <= 0:
        return to_quatd(q)
    e = euler_xyz_from_quat(q)
    return quat_from_euler_xyz([quantize(e[i], step_deg) for i in range(3)])


def rotate_about_matrix(axis: VecLike, angle_deg: float, pivot: VecLike) -> Gf.Matrix4d:
    """World-space matrix rotating ``angle_deg`` around ``axis`` passing through ``pivot``."""
    p = vec3d(pivot)
    to_origin = Gf.Matrix4d().SetTranslate(-p)
    rot = Gf.Matrix4d().SetRotate(Gf.Rotation(axis_vector(axis), angle_deg))
    back = Gf.Matrix4d().SetTranslate(p)
    return to_origin * rot * back


def is_close(a: VecLike, b: VecLike, tol: float = 1e-6) -> bool:
    return (vec3d(a) - vec3d(b)).GetLength() <= tol


def quat_angle_between(a: Union[Gf.Quatd, Gf.Quatf], b: Union[Gf.Quatd, Gf.Quatf]) -> float:
    """Angle in degrees between two orientations (0..180)."""
    qa, qb = to_quatd(a).GetNormalized(), to_quatd(b).GetNormalized()
    dot = abs(qa.GetReal() * qb.GetReal() + Gf.Dot(qa.GetImaginary(), qb.GetImaginary()))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def centroid(points: Iterable[VecLike]) -> Gf.Vec3d:
    pts = [vec3d(p) for p in points]
    if not pts:
        return Gf.Vec3d(0, 0, 0)
    acc = Gf.Vec3d(0, 0, 0)
    for p in pts:
        acc += p
    return acc / len(pts)
