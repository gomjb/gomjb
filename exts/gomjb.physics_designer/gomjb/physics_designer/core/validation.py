"""Static checks that catch the most common physics-setup mistakes before pressing Play."""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from pxr import Usd, UsdGeom, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import transform as tf
from .stage_utils import is_guide

ERROR, WARNING, INFO = "error", "warning", "info"


@dataclass
class Issue:
    severity: str
    path: str
    message: str
    fix: Optional[str] = None  # identifier of an automatic fix, see :func:`apply_fix`

    def __str__(self) -> str:
        return f"[{self.severity.upper()}] {self.path}: {self.message}"


def _rigid_ancestor(prim: Usd.Prim) -> Optional[Usd.Prim]:
    p = prim.GetParent()
    while p and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
        p = p.GetParent()
    return None


def _has_collider(prim: Usd.Prim) -> bool:
    return any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(prim))


def validate(stage: Usd.Stage, pos_tol: float = 1e-3, rot_tol_deg: float = 0.5) -> List[Issue]:
    issues: List[Issue] = []
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not scenes:
        issues.append(Issue(WARNING, "/", "No PhysicsScene on the stage (Isaac Sim creates a default one, "
                                          "but gravity/solver settings are then implicit).", fix="add_scene"))

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            anc = _rigid_ancestor(prim)
            xf = UsdGeom.Xformable(prim)
            if anc is not None and not (xf and xf.GetResetXformStack()):
                issues.append(Issue(ERROR, path, f"Nested rigid body under {anc.GetPath()}: PhysX ignores or "
                                                 "mis-handles it. Remove one RigidBodyAPI or restructure."))
            if not _has_collider(prim):
                mass_attr = UsdPhysics.MassAPI(prim).GetMassAttr() if prim.HasAPI(UsdPhysics.MassAPI) else None
                has_mass = bool(mass_attr and (mass_attr.Get() or 0) > 0)
                if has_mass:
                    issues.append(Issue(INFO, path, "Rigid body without collider (explicit mass set): it will "
                                                    "pass through other objects.", fix="add_collider"))
                else:
                    issues.append(Issue(WARNING, path, "Rigid body without collider or explicit mass: PhysX uses "
                                                       "a default mass and it falls through everything.",
                                        fix="add_collider"))
            _, _, scale = mu.decompose(tf.world_matrix(prim))
            if max(abs(scale[i] - scale[j]) for i in range(3) for j in range(3)) > 1e-6:
                issues.append(Issue(INFO, path, f"Non-uniform world scale {tuple(round(s, 4) for s in scale)} on a "
                                                "rigid body: prefer scaling the child geometry instead."))
        if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.IsA(UsdGeom.Mesh):
            body = prim if prim.HasAPI(UsdPhysics.RigidBodyAPI) else _rigid_ancestor(prim)
            approx = None
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                approx = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            dynamic = body is not None and not UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()
            if dynamic and approx in (None, "none"):
                issues.append(Issue(WARNING, path, "Triangle-mesh collider on a dynamic body; PhysX falls back to "
                                                   "a convex hull. Set an explicit approximation.",
                                    fix="convex_hull"))
        if prim.HasAPI(UsdPhysics.CollisionAPI) and is_guide(prim):
            issues.append(Issue(INFO, path, "Collider on a guide-purpose prim (invisible in renders)."))
        if prim.IsA(UsdPhysics.Joint):
            issues.extend(_check_joint(prim, pos_tol, rot_tol_deg))
    return issues


def _check_joint(joint: Usd.Prim, pos_tol: float, rot_tol: float) -> List[Issue]:
    out: List[Issue] = []
    path = str(joint.GetPath())
    j = UsdPhysics.Joint(joint)
    stage = joint.GetStage()
    for label, rel in (("body0", j.GetBody0Rel()), ("body1", j.GetBody1Rel())):
        for t in (rel.GetTargets() if rel else []):
            target = stage.GetPrimAtPath(t)
            if not target:
                out.append(Issue(ERROR, path, f"{label} target {t} does not exist."))
            elif not target.HasAPI(UsdPhysics.RigidBodyAPI) and _rigid_ancestor(target) is None:
                if not target.HasAPI(UsdPhysics.CollisionAPI):
                    out.append(Issue(WARNING, path, f"{label} {t} is not a rigid body: the joint attaches to a "
                                                    "static frame there."))
    b0, b1 = jt.joint_bodies(joint)
    if b0 is None and b1 is None:
        out.append(Issue(ERROR, path, "Joint has no bodies."))
        return out
    try:
        dp, da = jt.joint_frame_error(joint)
    except Exception as exc:  # pragma: no cover - defensive, malformed data
        out.append(Issue(ERROR, path, f"Could not evaluate joint frames: {exc}"))
        return out
    if dp > pos_tol or da > rot_tol:
        out.append(Issue(ERROR, path, f"Joint frames disagree by {dp:.4g} units / {da:.3g} deg: bodies will jump "
                                      "when simulation starts.", fix="realign_joint"))
    if joint.IsA(UsdPhysics.RevoluteJoint) or joint.IsA(UsdPhysics.PrismaticJoint):
        typed = UsdPhysics.RevoluteJoint(joint) if joint.IsA(UsdPhysics.RevoluteJoint) else UsdPhysics.PrismaticJoint(joint)
        lo, hi = typed.GetLowerLimitAttr().Get(), typed.GetUpperLimitAttr().Get()
        if lo is not None and hi is not None and lo > hi:
            out.append(Issue(WARNING, path, f"Lower limit {lo} > upper limit {hi}."))
    return out


def apply_fix(stage: Usd.Stage, issue: Issue) -> bool:
    """Apply the automatic fix attached to ``issue``. Returns True if something changed."""
    from . import physics as ph

    prim = stage.GetPrimAtPath(issue.path) if issue.path != "/" else None
    if issue.fix == "add_scene":
        ph.ensure_physics_scene(stage)
        return True
    if prim is None:
        return False
    if issue.fix == "realign_joint":
        jt.realign_joint(prim, keep="body0")
        return True
    if issue.fix == "add_collider":
        return bool(ph.apply_collision([prim], approximation="convexHull"))
    if issue.fix == "convex_hull":
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("convexHull")
        return True
    return False


def format_report(issues: List[Issue]) -> str:
    if not issues:
        return "No issues found."
    order = {ERROR: 0, WARNING: 1, INFO: 2}
    return "\n".join(str(i) for i in sorted(issues, key=lambda i: (order[i.severity], i.path)))
