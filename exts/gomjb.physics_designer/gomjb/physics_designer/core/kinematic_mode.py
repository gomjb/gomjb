"""How a mechanism moves in the Isaac Sim simulation, stored on its root prim.

* ``"physics"`` (default) — PhysX solves the mechanism: its joints are simulated, its drives
  track the joint motions (:mod:`motion`), loop closures are bushings or rigid joints.
* ``"table"`` / ``"cosim"`` — the exact solver (Exudyn, :mod:`kinematics`) moves it: every body
  of the mechanism becomes a kinematic body that follows the position analysis, from a motion
  table built before the simulation or solved at every physics step (co-simulation). PhysX
  only simulates what touches the mechanism, the split Process Simulate makes between device
  kinematics and physics.

Switching to a kinematic mode makes the mechanism's bodies kinematic, disables the joints that
act only inside it (a joint to an outside dynamic body stays: that body follows the mechanism
physically) and removes the articulation root (PhysX articulation links cannot be kinematic).
Each change is marked (``designer:kinematics:*``) so that switching back restores the model
exactly, and so that the exact solver still reads the mechanism as designed
(:func:`designed_dynamic`, :func:`designed_enabled`). The mode lives on the root prim, so it
travels with the model (and with cloned copies of it).
"""
from __future__ import annotations

import hashlib
from typing import Dict, List, Optional, Tuple

import numpy as np
from pxr import Gf, Sdf, Usd, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import transform as tf

MODES = ("physics", "table", "cosim")
KINEMATIC_MODES = ("table", "cosim")
MODE_ATTR = "designer:kinematics:mode"
TOLERANCE_ATTR = "designer:kinematics:tableTolerance"  # m
TABLE_TOLERANCE = 1.0e-4  # m: interpolation error a motion table is refined to (see kinematics.build_table)
_MADE_KINEMATIC = "designer:kinematics:madeKinematic"  # on bodies
_DISABLED = "designer:kinematics:disabledJoint"  # on joints
_ARTICULATION = "designer:kinematics:articulationRoot"  # on the root


def _marked(prim: Usd.Prim, name: str) -> bool:
    attr = prim.GetAttribute(name)
    return bool(attr and attr.HasAuthoredValue())


def _mark(prim: Usd.Prim, name: str, was_authored: bool = True) -> None:
    """Record a change: the marker's presence says it was made, its value whether the changed
    attribute had an authored value before (restored as such, or removed again)."""
    prim.CreateAttribute(name, Sdf.ValueTypeNames.Bool, custom=True).Set(bool(was_authored))


def _override(prim: Usd.Prim, attr: Usd.Attribute, value: bool, marker: str) -> None:
    """Set the bool ``attr`` to ``value`` unless it already has it, recording the change."""
    if bool(attr.Get()) == value:
        return
    _mark(prim, marker, attr.HasAuthoredValue())
    attr.Set(value)


def _undo(prim: Usd.Prim, attr_name: str, value: bool, marker: str) -> None:
    """Undo :func:`_override`: the previous authored value (the opposite of ``value``) or none."""
    if prim.GetAttribute(marker).Get():
        prim.GetAttribute(attr_name).Set(not value)
    else:
        prim.RemoveProperty(attr_name)
    prim.RemoveProperty(marker)


def get_mode(root: Usd.Prim) -> str:
    attr = root.GetAttribute(MODE_ATTR)
    return str(attr.Get()) if attr and attr.HasAuthoredValue() else "physics"


def get_tolerance(root: Usd.Prim) -> float:
    """Motion table interpolation tolerance [m]."""
    attr = root.GetAttribute(TOLERANCE_ATTR)
    return float(attr.Get()) if attr and attr.HasAuthoredValue() else TABLE_TOLERANCE


def designed_dynamic(prim: Usd.Prim) -> bool:
    """A dynamic rigid body as designed (a body made kinematic by a kinematic mode counts)."""
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return False
    api = UsdPhysics.RigidBodyAPI(prim)
    if api.GetRigidBodyEnabledAttr().Get() is False:
        return False
    return not api.GetKinematicEnabledAttr().Get() or _marked(prim, _MADE_KINEMATIC)


def designed_enabled(joint: Usd.Prim) -> bool:
    """The joint is enabled as designed (disabled by a kinematic mode counts as enabled)."""
    return UsdPhysics.Joint(joint).GetJointEnabledAttr().Get() is not False or _marked(joint, _DISABLED)


def mechanism_root_of(prim: Usd.Prim) -> Optional[Usd.Prim]:
    """The prim itself or its nearest ancestor that is a mechanism root: an articulation root, or
    a root whose articulation a kinematic mode removed."""
    p = prim
    while p and not p.IsPseudoRoot():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI) or p.GetAttribute(MODE_ATTR):
            return p
        p = p.GetParent()
    return None


def is_kinematic_root(prim: Usd.Prim) -> bool:
    return get_mode(prim) in KINEMATIC_MODES


def _scan(stage: Usd.Stage) -> Tuple[List[Usd.Prim], List[Usd.Prim]]:
    """(mechanism roots in a kinematic mode, joints outside all of them) in one traversal that
    does not enter the mechanisms (nothing inside one is looked at)."""
    roots, joints = [], []
    it = iter(Usd.PrimRange.Stage(stage))
    for prim in it:
        if is_kinematic_root(prim):
            roots.append(prim)
            it.PruneChildren()
        elif prim.IsA(UsdPhysics.Joint):
            joints.append(prim)
    return roots, joints


def kinematic_roots(stage: Usd.Stage) -> List[Usd.Prim]:
    """Mechanism roots in a kinematic mode."""
    return _scan(stage)[0]


def _canon(value) -> bytes:
    """A value as bytes, real numbers (scalars, vectors, matrices, quaternions) rounded to 1e-9:
    frames computed at another place carry round-off."""
    if isinstance(value, (Gf.Quatf, Gf.Quatd, Gf.Quath)):
        value = [value.GetReal(), *value.GetImaginary()]
    if isinstance(value, (float, list, Gf.Vec3f, Gf.Vec3d, Gf.Matrix4d)):
        return (np.round(np.asarray(value, dtype=float), 9) + 0.0).tobytes()  # + 0.0: no -0.0
    return repr(value).encode()


def _hash_joint(h, joint: Usd.Prim, base: Sdf.Path, name: str) -> None:
    h.update(f"{name}:{joint.GetTypeName()}".encode())
    # a ground side's frame is in world coordinates, so it differs between placed copies; the exact
    # solver anchors such a joint on its body side (mbd), so it is not part of the identity
    j = UsdPhysics.Joint(joint)
    ground = {i for i, rel in enumerate((j.GetBody0Rel(), j.GetBody1Rel())) if not rel.GetTargets()}
    skip = {f"physics:local{k}{i}" for i in ground for k in ("Pos", "Rot")}
    for attr in joint.GetAuthoredAttributes():
        if attr.GetName() not in skip:
            h.update(attr.GetName().encode())
            h.update(_canon(attr.Get()))
    for rel in joint.GetAuthoredRelationships():
        h.update(f"{rel.GetName()}>{[str(t.MakeRelativePath(base)) for t in rel.GetTargets()]}".encode())


def _fingerprint(root: Usd.Prim, outside: List[Usd.Prim]) -> str:
    """What a mechanism's kinematics depends on, wherever its root is: the bodies' frames relative
    to the root, every authored property of its joints — under the root, and ``outside`` it
    (relationships relative to the root): joint frames, drives, limits, motions — and its mode.
    Copies of one mechanism (cloned environments, duplicates) share it; a copy edited afterwards
    does not."""
    base = root.GetPath()
    root_inv = mu.rigid_part(tf.world_matrix(root)).GetInverse()
    h = hashlib.sha1()
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            h.update(str(prim.GetPath().MakeRelativePath(base)).encode())
            h.update(_canon(tf.world_matrix(prim) * root_inv))
        elif prim.IsA(UsdPhysics.Joint):
            _hash_joint(h, prim, base, str(prim.GetPath().MakeRelativePath(base)))
    for joint in sorted(outside, key=lambda p: p.GetName()):
        _hash_joint(h, joint, base, "<outside>/" + joint.GetName())
    h.update(f"{get_mode(root)}:{get_tolerance(root)}".encode())
    return h.hexdigest()


def _attached(roots: List[Usd.Prim], joints: List[Usd.Prim]) -> Dict[Sdf.Path, List[Usd.Prim]]:
    """Root path -> the joints (outside every root) that act on a body under it."""
    by_path = {r.GetPath() for r in roots}

    def root_above(path: Sdf.Path) -> Optional[Sdf.Path]:
        while not path.isEmpty and path != Sdf.Path.absoluteRootPath:
            if path in by_path:
                return path
            path = path.GetParentPath()
        return None

    out: Dict[Sdf.Path, List[Usd.Prim]] = {}
    for joint in joints:
        for r in {root_above(b.GetPath()) for b in jt.joint_bodies(joint) if b is not None} - {None}:
            out.setdefault(r, []).append(joint)
    return out


def fingerprint(root: Usd.Prim) -> str:
    """The identity of one mechanism's kinematics (as :func:`kinematic_groups` computes it)."""
    stage = root.GetStage()
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and not p.GetPath().HasPrefix(root.GetPath())]
    return _fingerprint(root, _attached([root], joints).get(root.GetPath(), []))


def _composition(root: Usd.Prim) -> Tuple[Tuple[str, str], ...]:
    """The specs a prim is composed of. Roots composed of the same specs are the same mechanism
    whatever their own path: their subtrees are composed of the same specs' namespace children,
    and an opinion on anything inside one of them would add a spec at its root (a spec needs its
    parents). Copies made by inheriting a source (Isaac Sim's cloner, the default) share it."""
    return tuple((s.layer.identifier, str(s.path)) for s in root.GetPrimStack())


def kinematic_groups(stage: Usd.Stage) -> List[Tuple[str, List[Usd.Prim]]]:
    """Mechanism roots in a kinematic mode, grouped into copies of one mechanism, with the group's
    :func:`fingerprint`: one motion table (cached under it) and one kinematic driver serve a whole
    group. Roots composed of the same specs group at once (:func:`_composition`, no property is
    read); the groups left are merged by fingerprint, computed once per group (copies made by
    copying specs, or built alike). A root with joints outside it (the Joint tool's /World/Joints)
    takes no shortcut: its fingerprint includes them."""
    roots, outside = _scan(stage)
    attached = _attached(roots, outside)
    composed: Dict[object, List[Usd.Prim]] = {}
    for r in roots:
        composed.setdefault(str(r.GetPath()) if r.GetPath() in attached else _composition(r), []).append(r)
    groups: Dict[str, List[Usd.Prim]] = {}
    for members in composed.values():
        groups.setdefault(_fingerprint(members[0], attached.get(members[0].GetPath(), [])), []).extend(members)
    return list(groups.items())


def mechanism_bodies(root: Usd.Prim) -> List[Usd.Prim]:
    return [p for p in Usd.PrimRange(root) if designed_dynamic(p)]


def _inner_joints(root: Usd.Prim, bodies: List[Usd.Prim]) -> List[Usd.Prim]:
    """Joints acting only inside the mechanism (between its bodies, or one of them and the world)."""
    inside = {str(b.GetPath()) for b in bodies}
    out = []
    for prim in root.GetStage().Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        ends = [str(b.GetPath()) for b in jt.joint_bodies(prim) if b is not None]
        if ends and all(e in inside for e in ends):
            out.append(prim)
    return out


def edit_paths(root: Usd.Prim) -> List[str]:
    """Prims :func:`set_mode` may change: the root's subtree and the inner joints outside it (the
    Joint tool puts joints under /World/Joints) — what an undo snapshot has to cover."""
    out = [str(root.GetPath())]
    for joint in _inner_joints(root, mechanism_bodies(root)):
        if not joint.GetPath().HasPrefix(root.GetPath()):
            out.append(str(joint.GetPath()))
    return out


def set_mode(root: Usd.Prim, mode: str, tolerance: Optional[float] = None) -> None:
    """Switch the mechanism under ``root`` to ``mode`` (see the module docstring); ``tolerance``
    [m] is the interpolation error a motion table is refined to."""
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if mode == "physics":
        _restore(root)
        return
    if get_mode(root) == "physics":
        bodies = mechanism_bodies(root)
        if not bodies:
            raise ValueError(f"no dynamic rigid bodies under {root.GetPath()}")
        for body in bodies:
            _override(body, UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr(), True, _MADE_KINEMATIC)
        for joint in _inner_joints(root, bodies):
            _override(joint, UsdPhysics.Joint(joint).GetJointEnabledAttr(), False, _DISABLED)
        if root.HasAPI(UsdPhysics.ArticulationRootAPI):
            root.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            _mark(root, _ARTICULATION)
    root.CreateAttribute(MODE_ATTR, Sdf.ValueTypeNames.Token, custom=True).Set(mode)
    if tolerance is not None:
        if not tolerance > 0:
            raise ValueError("the table tolerance must be > 0")
        root.CreateAttribute(TOLERANCE_ATTR, Sdf.ValueTypeNames.Double, custom=True).Set(float(tolerance))


def _restore(root: Usd.Prim) -> None:
    """Undo a kinematic mode: exactly what :func:`set_mode` changed."""
    stage = root.GetStage()
    for prim in Usd.PrimRange(root):
        if _marked(prim, _MADE_KINEMATIC):
            _undo(prim, "physics:kinematicEnabled", True, _MADE_KINEMATIC)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and _marked(prim, _DISABLED):
            _undo(prim, "physics:jointEnabled", False, _DISABLED)
    if _marked(root, _ARTICULATION):
        UsdPhysics.ArticulationRootAPI.Apply(root)
        # RemoveAPI also authored a "delete" of the schema, which Apply leaves in place
        spec = stage.GetEditTarget().GetPrimSpecForScenePath(root.GetPath())
        op = spec.GetInfo("apiSchemas") if spec else None
        if op is not None and "PhysicsArticulationRootAPI" in op.deletedItems:
            op.deletedItems = [s for s in op.deletedItems if s != "PhysicsArticulationRootAPI"]
            spec.SetInfo("apiSchemas", op)
        root.RemoveProperty(_ARTICULATION)
    for name in (MODE_ATTR, TOLERANCE_ATTR):
        if root.GetAttribute(name):
            root.RemoveProperty(name)
