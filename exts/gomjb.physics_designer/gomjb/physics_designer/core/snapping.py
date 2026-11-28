"""Object snapping: grid / angle snap, snap-to-target, face mating and anchor connection."""
from __future__ import annotations

from typing import Iterable, List, Optional, Sequence, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom

from . import math_utils as mu
from . import transform as tf
from .stage_utils import unique_child_path

ANCHOR_ATTR = "designer:anchor"
ANCHOR_KIND_ATTR = "designer:anchorKind"


# ---------------------------------------------------------------------------------------
# Grid / angle snapping
# ---------------------------------------------------------------------------------------
def snap_to_grid(
    prims: Iterable[Usd.Prim],
    step: float,
    axes: str = "XYZ",
    origin: mu.VecLike = (0, 0, 0),
    use_bbox: bool = False,
) -> None:
    """Quantize world position (pivot, or bbox minimum corner with ``use_bbox``)."""
    for prim in tf._top_level(prims):
        if use_bbox:
            b = tf.world_bbox(prim)
            ref = b.GetMin() if not b.IsEmpty() else tf.world_position(prim)
        else:
            ref = tf.world_position(prim)
        snapped = mu.quantize_vec(ref, step, origin, axes)
        tf.apply_world_delta(prim, Gf.Matrix4d().SetTranslate(snapped - mu.vec3d(ref)))


def snap_rotation(prims: Iterable[Usd.Prim], step_deg: float) -> None:
    """Quantize world orientation (XYZ euler) to multiples of ``step_deg``."""
    for prim in tf._top_level(prims):
        q = tf.world_rotation(prim)
        tf.set_world_pose(prim, tf.world_position(prim), mu.snap_rotation(q, step_deg))


def snap_value_position(position: mu.VecLike, step: float, axes: str = "XYZ") -> Gf.Vec3d:
    return mu.quantize_vec(position, step, (0, 0, 0), axes)


# ---------------------------------------------------------------------------------------
# Snap to another object
# ---------------------------------------------------------------------------------------
def snap_to_target(moving: Sequence[Usd.Prim], target: Usd.Prim, mode: str = "pivot",
                   offset: mu.VecLike = (0, 0, 0)) -> None:
    """Move ``moving`` prims so that their pivot / bbox center coincides with the target's.

    ``mode``: "pivot" (origin to origin) or "bbox" (bbox center to bbox center).
    """
    if mode == "bbox":
        dst = tf.bbox_center(target)
    else:
        dst = tf.world_position(target)
    dst = dst + mu.vec3d(offset)
    for prim in tf._top_level(moving):
        src = tf.bbox_center(prim) if mode == "bbox" else tf.world_position(prim)
        tf.apply_world_delta(prim, Gf.Matrix4d().SetTranslate(dst - src))


def bbox_snap_points(rng: Gf.Range3d) -> List[Gf.Vec3d]:
    """27 characteristic points of a box: corners, edge midpoints, face centers, center."""
    lo, hi, mid = rng.GetMin(), rng.GetMax(), rng.GetMidpoint()
    vals = [(lo[i], mid[i], hi[i]) for i in range(3)]
    return [Gf.Vec3d(x, y, z) for x in vals[0] for y in vals[1] for z in vals[2]]


def mesh_world_points(prim: Usd.Prim, max_points: int = 200000) -> List[Gf.Vec3d]:
    """World-space vertices of every Mesh under ``prim`` (capped at ``max_points``)."""
    out: List[Gf.Vec3d] = []
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        m = tf.world_matrix(p)
        for pt in pts:
            out.append(m.Transform(Gf.Vec3d(pt)))
            if len(out) >= max_points:
                return out
    return out


def nearest_point(query: mu.VecLike, candidates: Iterable[mu.VecLike]) -> Tuple[Optional[Gf.Vec3d], float]:
    q = mu.vec3d(query)
    best, best_d = None, float("inf")
    for c in candidates:
        d = (mu.vec3d(c) - q).GetLength()
        if d < best_d:
            best, best_d = mu.vec3d(c), d
    return best, best_d


def snap_to_nearest_point(moving: Usd.Prim, target: Usd.Prim, source: str = "pivot",
                          candidates: str = "bbox", max_distance: Optional[float] = None) -> Optional[Gf.Vec3d]:
    """Snap a point of ``moving`` to the closest characteristic point of ``target``.

    ``source``: "pivot" or "bbox" (closest of moving's 27 bbox points is used).
    ``candidates``: "bbox" (target's 27 bbox points), "vertex" (mesh vertices) or "pivot".
    Returns the target point used, or None if nothing was within ``max_distance``.
    """
    if candidates == "vertex":
        cand = mesh_world_points(target)
    elif candidates == "pivot":
        cand = [tf.world_position(target)]
    else:
        cand = bbox_snap_points(tf.world_bbox(target))
    if not cand:
        return None

    if source == "bbox":
        srcs = bbox_snap_points(tf.world_bbox(moving))
    else:
        srcs = [tf.world_position(moving)]

    best = None
    for s in srcs:
        p, d = nearest_point(s, cand)
        if p is not None and (best is None or d < best[2]):
            best = (s, p, d)
    if best is None or (max_distance is not None and best[2] > max_distance):
        return None
    tf.apply_world_delta(moving, Gf.Matrix4d().SetTranslate(best[1] - best[0]))
    return best[1]


_OPPOSITE = {"+X": "-X", "-X": "+X", "+Y": "-Y", "-Y": "+Y", "+Z": "-Z", "-Z": "+Z"}


def mate_faces(moving: Usd.Prim, target: Usd.Prim, target_face: str = "+Z", gap: float = 0.0,
               center: bool = True) -> None:
    """Place ``moving`` so that its bbox face opposite to ``target_face`` touches the
    target's ``target_face`` (e.g. "+Z" = stack on top). With ``center`` the two bboxes are
    also centered on the remaining axes."""
    face = target_face if target_face[0] in "+-" else "+" + target_face
    tb, mb = tf.world_bbox(target), tf.world_bbox(moving)
    if tb.IsEmpty() or mb.IsEmpty():
        raise ValueError("Both prims need geometry to mate faces")
    i = mu.AXIS_INDEX[face[1].upper()]
    sign = 1.0 if face[0] == "+" else -1.0
    t_pt = tf.bbox_face_point(tb, face)
    m_pt = tf.bbox_face_point(mb, _OPPOSITE[face])
    delta = Gf.Vec3d(0, 0, 0)
    delta[i] = t_pt[i] - m_pt[i] + sign * gap
    if center:
        for k in range(3):
            if k != i:
                delta[k] = tb.GetMidpoint()[k] - mb.GetMidpoint()[k]
    tf.apply_world_delta(moving, Gf.Matrix4d().SetTranslate(delta))


# ---------------------------------------------------------------------------------------
# Anchors (mount points)
# ---------------------------------------------------------------------------------------
def create_anchor(stage: Usd.Stage, parent: Usd.Prim, name: str = "Anchor",
                  world_position: Optional[mu.VecLike] = None,
                  world_rotation=None, kind: str = "generic",
                  marker_radius: float = 0.01) -> Usd.Prim:
    """Create an anchor Xform under ``parent`` placed at a world position/orientation.

    Anchors are named snap/joint points: "connect anchors" moves one body so two anchors
    coincide, and the joint tool can create joints exactly at an anchor frame. A small
    guide-purpose sphere marks it in the viewport (it is never a collider).
    """
    path = unique_child_path(stage, parent.GetPath(), name)
    xf = UsdGeom.Xform.Define(stage, path)
    prim = xf.GetPrim()
    prim.CreateAttribute(ANCHOR_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
    prim.CreateAttribute(ANCHOR_KIND_ATTR, Sdf.ValueTypeNames.Token, custom=True).Set(kind)
    pos = tf.world_position(parent) if world_position is None else mu.vec3d(world_position)
    rot = Gf.Quatd(1, 0, 0, 0) if world_rotation is None else mu.to_quatd(world_rotation)
    # Anchors live in parent space but must not inherit parent scale in their frame.
    parent_w = tf.world_matrix(parent)
    local = mu.make_transform(pos, rot) * parent_w.GetInverse()
    tf.set_local_matrix(prim, local)
    marker = UsdGeom.Sphere.Define(stage, path.AppendChild("marker"))
    marker.CreateRadiusAttr(marker_radius)
    marker.CreatePurposeAttr(UsdGeom.Tokens.guide)
    marker.CreateDisplayColorAttr([Gf.Vec3f(1.0, 0.55, 0.0)])
    return prim


def is_anchor(prim: Usd.Prim) -> bool:
    attr = prim.GetAttribute(ANCHOR_ATTR)
    return bool(attr and attr.Get())


def find_anchors(root: Usd.Prim) -> List[Usd.Prim]:
    return [p for p in Usd.PrimRange(root) if is_anchor(p)]


def anchor_owner(anchor: Usd.Prim) -> Usd.Prim:
    """The prim that moves when an anchor is connected: nearest rigid-body ancestor,
    otherwise the anchor's direct parent."""
    from pxr import UsdPhysics  # local import keeps module importable for docs tools

    p = anchor.GetParent()
    while p and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.ArticulationRootAPI):
            return p
        p = p.GetParent()
    return anchor.GetParent()


def anchor_frame(anchor: Usd.Prim) -> Gf.Matrix4d:
    return mu.rigid_part(tf.world_matrix(anchor))


def connect_anchors(moving_anchor: Usd.Prim, target_anchor: Usd.Prim, align_rotation: bool = True,
                    flip: bool = False, owner: Optional[Usd.Prim] = None) -> Usd.Prim:
    """Move the owner of ``moving_anchor`` so the anchor coincides with ``target_anchor``.

    ``align_rotation`` matches orientations as well; ``flip`` turns the moving anchor 180°
    about its local X axis first so the two anchor Z axes face each other (connector mating).
    Returns the prim that was moved.
    """
    owner = owner or anchor_owner(moving_anchor)
    a = anchor_frame(moving_anchor)
    t = anchor_frame(target_anchor)
    if not align_rotation:
        t = Gf.Matrix4d(a).SetTranslateOnly(t.ExtractTranslation())
    elif flip:
        t = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)) * t
    rel = tf.world_matrix(owner) * a.GetInverse()
    tf.set_world_matrix(owner, rel * t)
    return owner
