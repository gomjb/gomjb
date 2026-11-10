"""World-space transform editing for USD prims.

Every function works on plain ``pxr`` objects so it runs both inside Isaac Sim and in
an offline ``usd-core`` Python environment (used by the unit tests).
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Union

from pxr import Gf, Sdf, Usd, UsdGeom

from . import math_utils as mu

PrimLike = Union[Usd.Prim, str, Sdf.Path]

TIME = Usd.TimeCode.Default()


# ---------------------------------------------------------------------------------------
# Lookup helpers
# ---------------------------------------------------------------------------------------
def get_prim(stage: Usd.Stage, prim: PrimLike) -> Usd.Prim:
    if isinstance(prim, Usd.Prim):
        return prim
    p = stage.GetPrimAtPath(Sdf.Path(str(prim)))
    if not p or not p.IsValid():
        raise ValueError(f"Prim not found: {prim}")
    return p


def world_matrix(prim: Usd.Prim, time: Usd.TimeCode = TIME) -> Gf.Matrix4d:
    return Gf.Matrix4d(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time)) if prim.IsA(
        UsdGeom.Xformable
    ) else _parent_world(prim, time)


def _parent_world(prim: Usd.Prim, time: Usd.TimeCode = TIME) -> Gf.Matrix4d:
    parent = prim.GetParent()
    while parent and parent.IsValid() and not parent.IsPseudoRoot():
        if parent.IsA(UsdGeom.Xformable):
            return Gf.Matrix4d(UsdGeom.Xformable(parent).ComputeLocalToWorldTransform(time))
        parent = parent.GetParent()
    return Gf.Matrix4d(1.0)


def parent_world_matrix(prim: Usd.Prim, time: Usd.TimeCode = TIME) -> Gf.Matrix4d:
    xf = UsdGeom.Xformable(prim)
    if xf and xf.GetResetXformStack():
        return Gf.Matrix4d(1.0)
    return _parent_world(prim, time)


def world_position(prim: Usd.Prim) -> Gf.Vec3d:
    return world_matrix(prim).ExtractTranslation()


def world_rotation(prim: Usd.Prim) -> Gf.Quatd:
    return mu.rotation_of(world_matrix(prim)).GetQuat()


def world_bbox(prim: Usd.Prim, purposes: Optional[Sequence[str]] = None) -> Gf.Range3d:
    """Axis aligned world bounding box (empty range if the prim has no geometry)."""
    purposes = purposes or [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    cache = UsdGeom.BBoxCache(TIME, purposes, useExtentsHint=False)
    return cache.ComputeWorldBound(prim).ComputeAlignedRange()


def combined_bbox(prims: Iterable[Usd.Prim]) -> Gf.Range3d:
    rng = Gf.Range3d()
    for p in prims:
        b = world_bbox(p)
        if not b.IsEmpty():
            rng.UnionWith(b)
    return rng


def bbox_center(prim: Usd.Prim) -> Gf.Vec3d:
    b = world_bbox(prim)
    return world_position(prim) if b.IsEmpty() else b.GetMidpoint()


def bbox_face_point(rng: Gf.Range3d, axis: str) -> Gf.Vec3d:
    """Center of the bbox face pointing along ``axis`` ("+X", "-Z", ...)."""
    sign = -1 if axis.startswith("-") else 1
    i = mu.AXIS_INDEX[axis.lstrip("+-").upper()]
    p = Gf.Vec3d(rng.GetMidpoint())
    p[i] = rng.GetMax()[i] if sign > 0 else rng.GetMin()[i]
    return p


# ---------------------------------------------------------------------------------------
# Writing transforms
# ---------------------------------------------------------------------------------------
_STANDARD_OPS = ("xformOp:translate", "xformOp:orient", "xformOp:scale")


def _existing_precision(prim: Usd.Prim, name: str) -> Optional[UsdGeom.XformOp.Precision]:
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        return None
    op = UsdGeom.XformOp(attr)
    return op.GetPrecision() if op else None


def _matrices_close(a: Gf.Matrix4d, b: Gf.Matrix4d, tol: float = 1e-9) -> bool:
    scale = max(1.0, max(abs(a[i][j]) for i in range(4) for j in range(4)))
    return all(abs(a[i][j] - b[i][j]) <= tol * scale for i in range(4) for j in range(4))


def set_local_matrix(prim: Usd.Prim, local: Gf.Matrix4d) -> None:
    """Author ``local`` on ``prim`` as a translate / orient / scale op stack.

    Existing standard ops keep their precision (float vs double) so Isaac Sim assets that
    use ``float3``/``quatf`` stay consistent.  Any other op stack is replaced by an
    equivalent TRS stack.  If the matrix contains shear (e.g. a rotated child under a
    non-uniformly scaled parent) a single ``xformOp:transform`` is authored instead so the
    world pose stays exact.
    """
    xf = UsdGeom.Xformable(prim)
    if not xf:
        raise ValueError(f"{prim.GetPath()} is not Xformable")
    local = Gf.Matrix4d(local)
    t, q, s = mu.decompose(local)
    exact_trs = _matrices_close(mu.make_transform(t, q, s), local, 1e-7)

    old_names = [op.GetOpName() for op in xf.GetOrderedXformOps()]
    reset = xf.GetResetXformStack()
    xf.ClearXformOpOrder()
    if reset:
        xf.SetResetXformStack(True)
    keep = set(_STANDARD_OPS) if exact_trs else {"xformOp:transform"}
    for name in old_names:  # drop stale op attributes so the prim stays tidy
        if name not in keep and prim.GetAttribute(name):
            prim.RemoveProperty(name)

    if not exact_trs:
        if prim.GetAttribute("xformOp:transform"):
            prim.RemoveProperty("xformOp:transform")
        xf.AddTransformOp().Set(local)
        return

    double = UsdGeom.XformOp.PrecisionDouble
    t_op = xf.AddTranslateOp(_existing_precision(prim, "xformOp:translate") or double)
    r_op = xf.AddOrientOp(_existing_precision(prim, "xformOp:orient") or double)
    s_op = xf.AddScaleOp(_existing_precision(prim, "xformOp:scale") or double)

    if t_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        t_op.Set(Gf.Vec3f(t))
    else:
        t_op.Set(t)
    if r_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        r_op.Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    elif r_op.GetPrecision() == UsdGeom.XformOp.PrecisionHalf:
        r_op.Set(Gf.Quath(q.GetReal(), Gf.Vec3h(q.GetImaginary())))
    else:
        r_op.Set(q)
    if s_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        s_op.Set(Gf.Vec3f(s))
    else:
        s_op.Set(s)


def set_world_matrix(prim: Usd.Prim, world: Gf.Matrix4d) -> None:
    local = Gf.Matrix4d(world) * parent_world_matrix(prim).GetInverse()
    set_local_matrix(prim, local)


def set_world_position(prim: Usd.Prim, position: mu.VecLike) -> None:
    m = world_matrix(prim)
    m.SetTranslateOnly(mu.vec3d(position))
    set_world_matrix(prim, m)


def set_world_pose(prim: Usd.Prim, position: mu.VecLike, rotation) -> None:
    """Set world position and orientation while keeping the prim's world scale."""
    _, _, scale = mu.decompose(world_matrix(prim))
    set_world_matrix(prim, mu.make_transform(position, rotation, scale))


def apply_world_delta(prim: Usd.Prim, delta: Gf.Matrix4d) -> None:
    """Pre-transform the prim in world space: ``W' = W * delta``."""
    set_world_matrix(prim, world_matrix(prim) * delta)


# ---------------------------------------------------------------------------------------
# High level move / rotate operations
# ---------------------------------------------------------------------------------------
def translate(prims: Iterable[Usd.Prim], offset: mu.VecLike, space: str = "world") -> None:
    """Move prims by ``offset``. ``space`` is "world" or "local" (prim's own axes)."""
    off = mu.vec3d(offset)
    for prim in _top_level(prims):
        if space == "local":
            rot = mu.rotation_of(world_matrix(prim))
            world_off = rot.TransformDir(off)
        else:
            world_off = off
        apply_world_delta(prim, Gf.Matrix4d().SetTranslate(world_off))


def rotate(
    prims: Iterable[Usd.Prim],
    axis: Union[str, mu.VecLike],
    angle_deg: float,
    pivot: str = "self",
    pivot_point: Optional[mu.VecLike] = None,
    space: str = "world",
) -> None:
    """Rotate prims around an axis.

    ``pivot``: "self" (each prim's origin), "bbox" (each prim's bbox center),
    "group" (common bbox center of all prims) or "point" (``pivot_point``).
    """
    prims = _top_level(prims)
    group_center = combined_bbox(prims).GetMidpoint() if pivot == "group" else None
    for prim in prims:
        if pivot == "self":
            c = world_position(prim)
        elif pivot == "bbox":
            c = bbox_center(prim)
        elif pivot == "group":
            c = group_center
        elif pivot == "point":
            if pivot_point is None:
                raise ValueError("pivot_point required for pivot='point'")
            c = mu.vec3d(pivot_point)
        else:
            raise ValueError(f"Unknown pivot {pivot}")
        ax = mu.axis_vector(axis)
        if space == "local":
            ax = mu.rotation_of(world_matrix(prim)).TransformDir(ax)
        apply_world_delta(prim, mu.rotate_about_matrix(ax, angle_deg, c))


def align(prims: Sequence[Usd.Prim], axis: str, mode: str = "center", reference: str = "first") -> None:
    """Align bboxes along a world axis.

    ``mode``: "min", "center", "max". ``reference``: "first" (first selected prim),
    "last" or "group" (combined bbox of all prims).
    """
    prims = list(_top_level(prims))
    if len(prims) < 2 and reference != "group":
        return
    i = mu.AXIS_INDEX[axis.upper()]

    def key(rng: Gf.Range3d) -> float:
        if mode == "min":
            return rng.GetMin()[i]
        if mode == "max":
            return rng.GetMax()[i]
        return rng.GetMidpoint()[i]

    if reference == "first":
        target = key(world_bbox(prims[0]))
        movers = prims[1:]
    elif reference == "last":
        target = key(world_bbox(prims[-1]))
        movers = prims[:-1]
    else:
        target = key(combined_bbox(prims))
        movers = prims
    for prim in movers:
        b = world_bbox(prim)
        if b.IsEmpty():
            continue
        d = Gf.Vec3d(0, 0, 0)
        d[i] = target - key(b)
        apply_world_delta(prim, Gf.Matrix4d().SetTranslate(d))


def distribute(prims: Sequence[Usd.Prim], axis: str, gap: Optional[float] = None) -> None:
    """Distribute prims along ``axis``.

    With ``gap`` = None the bbox centers are spaced evenly between the outermost prims;
    otherwise prims are packed in their current order with ``gap`` between bboxes.
    """
    prims = list(_top_level(prims))
    if len(prims) < 2:
        return
    i = mu.AXIS_INDEX[axis.upper()]
    boxes = {p.GetPath(): world_bbox(p) for p in prims}
    ordered = sorted(prims, key=lambda p: boxes[p.GetPath()].GetMidpoint()[i])
    if gap is None:
        if len(ordered) < 3:
            return
        start = boxes[ordered[0].GetPath()].GetMidpoint()[i]
        end = boxes[ordered[-1].GetPath()].GetMidpoint()[i]
        step = (end - start) / (len(ordered) - 1)
        for n, prim in enumerate(ordered[1:-1], start=1):
            d = Gf.Vec3d(0, 0, 0)
            d[i] = start + step * n - boxes[prim.GetPath()].GetMidpoint()[i]
            apply_world_delta(prim, Gf.Matrix4d().SetTranslate(d))
    else:
        cursor = boxes[ordered[0].GetPath()].GetMax()[i]
        for prim in ordered[1:]:
            b = boxes[prim.GetPath()]
            d = Gf.Vec3d(0, 0, 0)
            d[i] = cursor + gap - b.GetMin()[i]
            apply_world_delta(prim, Gf.Matrix4d().SetTranslate(d))
            cursor = b.GetMax()[i] + d[i]


def drop_to(prims: Iterable[Usd.Prim], height: float = 0.0, up_axis: str = "Z",
            onto: Optional[Sequence[Usd.Prim]] = None) -> None:
    """Move prims along ``up_axis`` so that their bbox bottom rests on ``height``,
    or on top of the highest overlapping bbox in ``onto`` when given."""
    i = mu.AXIS_INDEX[up_axis.upper()]
    others = list(onto or [])
    for prim in _top_level(prims):
        b = world_bbox(prim)
        if b.IsEmpty():
            continue
        floor = height
        for o in others:
            if o.GetPath() == prim.GetPath():
                continue
            ob = world_bbox(o)
            if ob.IsEmpty() or not _overlap_2d(b, ob, i):
                continue
            floor = max(floor, ob.GetMax()[i])
        d = Gf.Vec3d(0, 0, 0)
        d[i] = floor - b.GetMin()[i]
        apply_world_delta(prim, Gf.Matrix4d().SetTranslate(d))


def copy_pose(source: Usd.Prim, targets: Iterable[Usd.Prim], position: bool = True, rotation: bool = True) -> None:
    sm = world_matrix(source)
    for prim in _top_level(targets):
        if prim.GetPath() == source.GetPath():
            continue
        tm = world_matrix(prim)
        t, q, s = mu.decompose(tm)
        st, sq, _ = mu.decompose(sm)
        set_world_matrix(prim, mu.make_transform(st if position else t, sq if rotation else q, s))


def _overlap_2d(a: Gf.Range3d, b: Gf.Range3d, up: int) -> bool:
    for k in range(3):
        if k == up:
            continue
        if a.GetMax()[k] < b.GetMin()[k] or b.GetMax()[k] < a.GetMin()[k]:
            return False
    return True


def _top_level(prims: Iterable[Usd.Prim]) -> List[Usd.Prim]:
    """Drop prims whose ancestor is also in the list (moving both would double-move)."""
    prims = [p for p in prims if p and p.IsValid()]
    paths = {p.GetPath() for p in prims}
    result = []
    for p in prims:
        parent = p.GetPath().GetParentPath()
        nested = False
        while parent != Sdf.Path.absoluteRootPath and not parent.isEmpty:
            if parent in paths:
                nested = True
                break
            parent = parent.GetParentPath()
        if not nested:
            result.append(p)
    return result
