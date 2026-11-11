"""RecurDyn-style joint creation: pick the base body, the action body, a point and a direction.

Joint types (RecurDyn names), creation methods and their pick order follow RecurDyn Professional
(Joint > Modeling Options):

* the **Base** body is the joint's reference (USD body0, the articulation parent), the **Action**
  body moves relative to it (body1);
* the **Point** is the joint origin; the joint **Z axis** is the rotation axis of a revolute and
  the sliding axis of a translational joint. A **Direction** pick (a second point, or a global /
  marker axis) sets it; methods without a direction use the working-plane normal;
* "Point" methods pick only the point: the clicked body is the action body and the base is the
  other body at that point (the smallest one whose bounds contain it), else the ground;
* clicking anything that is not a rigid body (ground plane, static geometry, empty space) picks
  the ground.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Tuple

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import snapping as sn
from . import transform as tf

JOINT_TYPES: Dict[str, str] = {  # RecurDyn name -> USD joint type
    "Fixed": "fixed",
    "Revolute": "revolute",
    "Translational": "prismatic",
    "Spherical": "spherical",
}
METHODS: Dict[str, Tuple[str, ...]] = {  # the first one is the default
    "Fixed": ("Body, Body, Point", "Point"),
    "Revolute": ("Body, Body, Point", "Body, Body, Point, Direction", "Point", "Point, Direction"),
    "Translational": ("Body, Body, Point, Direction", "Point, Direction"),  # the axis must be picked
    "Spherical": ("Body, Body, Point", "Point"),
}
STEPS: Dict[str, Tuple[str, ...]] = {
    "Body, Body, Point": ("base", "action", "point"),
    "Body, Body, Point, Direction": ("base", "action", "point", "direction"),
    "Point": ("point",),
    "Point, Direction": ("point", "direction"),
}
WORKING_PLANES = {"XY": (0.0, 0.0, 1.0), "YZ": (1.0, 0.0, 0.0), "ZX": (0.0, 1.0, 0.0)}  # plane -> normal
PROMPTS = {
    "base": "click the BASE body (ground: anything that is not a body)",
    "action": "click the ACTION body",
    "point": "click the joint POINT",
    "direction": "click a second point for the Z axis, or choose an axis",
}
SNAP_MODES = ("Key point", "Surface")


def body_of(prim: Optional[Usd.Prim]) -> Optional[Usd.Prim]:
    """The rigid body a picked prim belongs to (nearest RigidBodyAPI ancestor); None = ground."""
    while prim is not None and prim.IsValid() and not prim.IsPseudoRoot():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim
        prim = prim.GetParent()
    return None


def key_points(prim: Usd.Prim) -> List[Tuple[Gf.Vec3d, Optional[Usd.Prim]]]:
    """Candidate points around a pick: the anchors (markers) of the picked body, then corners,
    edge midpoints, face centers and center of the picked geometry's own (oriented) bounds."""
    body = body_of(prim)
    out = [(tf.world_position(a), a) for a in sn.find_anchors(body)] if body is not None else []
    local = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeUntransformedBound(prim)
    rng = local.GetRange()
    if not rng.IsEmpty():
        m = local.GetMatrix() * tf.world_matrix(prim)  # bound frame -> prim -> world (row vectors)
        out += [(m.Transform(p), None) for p in sn.bbox_snap_points(rng)]
    return out


MARKER_REACH = 0.15  # a click within this fraction of the geometry's size takes the marker (pin center)


def snap(prim: Usd.Prim, hit: mu.VecLike, mode: str = "Key point") -> Tuple[Gf.Vec3d, Optional[Usd.Prim]]:
    """(point, anchor or None): the hit itself ("Surface"), else the nearest marker when the click
    is near one (a click on a link next to its pin means the pin center), else the nearest key point."""
    hit = mu.vec3d(hit)
    if mode == "Surface" or prim is None:
        return hit, None
    candidates = key_points(prim)
    if not candidates:
        return hit, None

    def dist(c):
        return (c[0] - hit).GetLength()

    markers = [c for c in candidates if c[1] is not None]
    size = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeUntransformedBound(
        prim).GetRange().GetSize().GetLength()
    if markers and dist(min(markers, key=dist)) <= MARKER_REACH * size:
        return min(markers, key=dist)
    return min(candidates, key=dist)


def _base_at(stage: Usd.Stage, point: Gf.Vec3d, action: Optional[Usd.Prim]) -> Optional[Usd.Prim]:
    """The smallest other rigid body whose bounds contain ``point`` (the second body of a
    "Point" pick), else the ground."""
    best, best_vol = None, float("inf")
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or (action is not None and prim.GetPath() == action.GetPath()):
            continue
        rng = tf.world_bbox(prim)
        if rng.IsEmpty() or not rng.Contains(point):
            continue
        size = rng.GetSize()
        vol = size[0] * size[1] * size[2]
        if vol < best_vol:
            best, best_vol = prim, vol
    return best


@dataclass
class JointPicker:
    """State of one joint command: what has been picked so far and what comes next."""

    kind: str = "Revolute"
    method: str = ""
    working_plane: str = "XY"
    picks: Dict[str, object] = field(default_factory=dict)  # step -> body (or None = ground) / point
    anchor: Optional[Usd.Prim] = None  # the marker the point snapped to, if any

    def __post_init__(self):
        if self.kind not in JOINT_TYPES:
            raise ValueError(f"unknown joint type {self.kind!r}")
        self.method = self.method or METHODS[self.kind][0]
        if self.method not in METHODS[self.kind]:
            raise ValueError(f"{self.kind} joints are created with {', '.join(METHODS[self.kind])}")

    @property
    def steps(self) -> Tuple[str, ...]:
        return STEPS[self.method]

    @property
    def step(self) -> Optional[str]:
        """The next pick, None when complete."""
        return next((s for s in self.steps if s not in self.picks), None)

    def prompt(self) -> str:
        step = self.step
        head = f"{self.kind} ({self.method})"
        return f"{head}: {PROMPTS[step]}" if step else f"{head}: complete"

    # -- picks ---------------------------------------------------------------------------
    def pick(self, prim: Optional[Usd.Prim], point: Optional[mu.VecLike] = None,
             anchor: Optional[Usd.Prim] = None) -> None:
        """Feed one viewport pick: the clicked prim (None = empty space) and the 3-D point on it
        (already snapped, see :func:`snap`); ``anchor`` = the marker the point snapped to."""
        step = self.step
        if step is None:
            raise ValueError("the joint is already complete")
        if step in ("base", "action"):
            body = body_of(prim)
            other = self.picks.get("base") if step == "action" else None
            if body is not None and other is not None and body.GetPath() == other.GetPath():
                raise ValueError("base and action must be different bodies")
            if step == "action" and body is None and self.picks.get("base") is None:
                raise ValueError("base and action cannot both be the ground")
            self.picks[step] = body
            return
        if point is None:
            raise ValueError(f"{step}: a point is needed")
        p = mu.vec3d(point)
        if step == "point":
            self.picks["point"] = p
            self.anchor = anchor
            if "base" not in self.steps:  # a "Point" method: the bodies come from the point
                action = body_of(prim)
                if action is None:
                    raise ValueError("click the point on a body")
                self.picks["action"] = action
                self.picks["base"] = _base_at(action.GetStage(), p, action)
            return
        self.set_direction(p - self.picks["point"])

    def set_direction(self, direction: mu.VecLike) -> None:
        """The joint Z axis (a second point minus the first, a global axis or a marker axis)."""
        d = mu.vec3d(direction)
        if d.GetLength() < 1e-12:
            raise ValueError("the direction points must differ")
        self.picks["direction"] = d.GetNormalized()

    def marker_axis(self) -> Optional[Gf.Vec3d]:
        """Z axis of the marker the point snapped to (RecurDyn: "clicking one of the axes of the marker")."""
        if self.anchor is None:
            return None
        return Gf.Rotation(tf.world_rotation(self.anchor)).TransformDir(Gf.Vec3d(0, 0, 1))

    # -- result --------------------------------------------------------------------------
    @property
    def base(self) -> Optional[Usd.Prim]:
        return self.picks.get("base")

    @property
    def action(self) -> Optional[Usd.Prim]:
        return self.picks.get("action")

    def axis(self) -> Gf.Vec3d:
        return self.picks.get("direction") or Gf.Vec3d(*WORKING_PLANES[self.working_plane])

    def frame(self) -> Tuple[Gf.Vec3d, Gf.Quatd]:
        """World joint frame: origin at the point, Z along :meth:`axis`."""
        return self.picks["point"], jt.frame_for_world_axis(self.axis(), "Z")

    def create(self, stage: Usd.Stage, properties: Optional[jt.JointSpec] = None,
               parent_path=None) -> Usd.Prim:
        """Author the joint; ``properties`` carries limits, drive, break and collision settings
        (stage units); its type and axis are set here (RecurDyn: the joint axis is Z)."""
        if self.step is not None:
            raise ValueError(f"not complete: {self.prompt()}")
        spec = replace(properties or jt.JointSpec(), joint_type=JOINT_TYPES[self.kind], axis="Z")
        pos, rot = self.frame()
        return jt.create_joint(stage, spec, self.base, self.action, pos, rot, parent_path)
