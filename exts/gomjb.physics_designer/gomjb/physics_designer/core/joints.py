"""Physics joint authoring with correct ``localPos/localRot`` frames.

The hardest part of joints in USD Physics is that each body stores the joint frame in its
own local space.  Everything here starts from a single *world* joint frame and derives both
local frames from it, so the joint is consistent at creation time (no snapping of bodies
when simulation starts).
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple, Union

from pxr import Gf, Sdf, Usd, UsdPhysics

from . import math_utils as mu
from . import transform as tf
from . import units as un
from .stage_utils import default_root, ensure_scope, unique_child_path

JOINT_TYPES = ("fixed", "revolute", "prismatic", "spherical", "distance", "d6")
D6_DOFS = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")
LOCATIONS = ("body0", "body1", "midpoint", "body0_bbox", "body1_bbox", "anchor", "custom")

_SCHEMAS = {
    "fixed": UsdPhysics.FixedJoint,
    "revolute": UsdPhysics.RevoluteJoint,
    "prismatic": UsdPhysics.PrismaticJoint,
    "spherical": UsdPhysics.SphericalJoint,
    "distance": UsdPhysics.DistanceJoint,
    "d6": UsdPhysics.Joint,
}


@dataclass
class DriveSpec:
    """Joint drive (motor / PD controller). Angular values are in degrees."""

    drive_type: str = "force"  # "force" or "acceleration"
    stiffness: float = 0.0
    damping: float = 0.0
    max_force: Optional[float] = None
    target_position: Optional[float] = None
    target_velocity: Optional[float] = None


@dataclass
class JointSpec:
    joint_type: str = "revolute"
    name: Optional[str] = None
    axis: str = "X"  # joint axis in the joint frame (revolute / prismatic / spherical)
    lower: Optional[float] = None
    upper: Optional[float] = None
    drive: Optional[DriveSpec] = None
    break_force: Optional[float] = None
    break_torque: Optional[float] = None
    collision_enabled: bool = False
    exclude_from_articulation: bool = False
    # spherical: cone limits (deg); distance: min/max distance
    cone0: Optional[float] = None
    cone1: Optional[float] = None
    min_distance: Optional[float] = None
    max_distance: Optional[float] = None
    # d6: dof -> "free" | "locked" | (low, high); drives per dof
    d6: Dict[str, Union[str, Tuple[float, float]]] = field(default_factory=dict)
    d6_drives: Dict[str, DriveSpec] = field(default_factory=dict)


def _from_si(stage: Usd.Stage, value: Optional[float], quantity: str) -> Optional[float]:
    return None if value is None else value / un.si_per_stage_unit(stage, quantity)


def drive_from_si(stage: Usd.Stage, drive: DriveSpec, linear: bool) -> DriveSpec:
    """Copy of a drive written in SI (N/m or N*m/deg, ...; targets in m or deg) in stage units."""
    def q(lin: str, ang: str) -> str:
        return lin if linear else ang

    return replace(drive,
                   stiffness=_from_si(stage, drive.stiffness, q("linear_stiffness", "angular_stiffness")),
                   damping=_from_si(stage, drive.damping, q("linear_damping", "angular_damping")),
                   max_force=_from_si(stage, drive.max_force, q("force", "torque")),
                   target_position=_from_si(stage, drive.target_position, q("length", "angle")),
                   target_velocity=_from_si(stage, drive.target_velocity, q("velocity", "angular_velocity")))


def spec_from_si(spec: JointSpec, stage: Usd.Stage) -> JointSpec:
    """Copy of ``spec`` written in SI (m, N, N*m, N/m, N*m/deg, ...; angles in deg) in the stage's
    units (metersPerUnit / kilogramsPerUnit), which is how :func:`create_joint` takes it."""
    linear = spec.joint_type == "prismatic"
    limit = "length" if linear else "angle"
    return replace(
        spec,
        lower=_from_si(stage, spec.lower, limit), upper=_from_si(stage, spec.upper, limit),
        drive=drive_from_si(stage, spec.drive, linear) if spec.drive else None,
        break_force=_from_si(stage, spec.break_force, "force"),
        break_torque=_from_si(stage, spec.break_torque, "torque"),
        min_distance=_from_si(stage, spec.min_distance, "length"),
        max_distance=_from_si(stage, spec.max_distance, "length"),
        d6={dof: lim if isinstance(lim, str) else tuple(
            _from_si(stage, v, "length" if dof.startswith("trans") else "angle") for v in lim)
            for dof, lim in spec.d6.items()},
        d6_drives={dof: drive_from_si(stage, d, dof.startswith("trans")) for dof, d in spec.d6_drives.items()})


# ---------------------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------------------
def frame_for_world_axis(world_direction: mu.VecLike, joint_axis: str = "X",
                         base_rotation=None) -> Gf.Quatd:
    """Joint-frame orientation whose local ``joint_axis`` points along ``world_direction``.

    When ``base_rotation`` is given (e.g. a body orientation) the result is the minimal
    rotation away from it, which keeps the other two axes intuitive.
    """
    base = mu.to_rotation(base_rotation) if base_rotation is not None else Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
    current = base.TransformDir(mu.AXIS_VECTORS[joint_axis.upper()])
    return (base * mu.rotation_from_to(current, world_direction)).GetQuat()


def resolve_location(stage: Usd.Stage, location: str, body0: Optional[Usd.Prim], body1: Optional[Usd.Prim],
                     anchor: Optional[Usd.Prim] = None, custom: Optional[mu.VecLike] = None) -> Gf.Vec3d:
    """World position for one of :data:`LOCATIONS`."""
    if location == "custom":
        if custom is None:
            raise ValueError("custom position required")
        return mu.vec3d(custom)
    if location == "anchor":
        if anchor is None:
            raise ValueError("anchor prim required")
        return tf.world_position(anchor)
    if location == "body0":
        return tf.world_position(_need(body0, "body0"))
    if location == "body1":
        return tf.world_position(_need(body1, "body1"))
    if location == "body0_bbox":
        return tf.bbox_center(_need(body0, "body0"))
    if location == "body1_bbox":
        return tf.bbox_center(_need(body1, "body1"))
    if location == "midpoint":
        pts = [tf.world_position(b) for b in (body0, body1) if b is not None]
        return mu.centroid(pts)
    raise ValueError(f"Unknown joint location '{location}'")


def _need(prim, label):
    if prim is None:
        raise ValueError(f"{label} is required for this location")
    return prim


def local_frame(body: Optional[Usd.Prim], world_pos: mu.VecLike, world_rot) -> Tuple[Gf.Vec3f, Gf.Quatf]:
    """Express a world joint frame in ``body``'s local space (or world if ``body`` is None)."""
    wp = mu.vec3d(world_pos)
    wr = mu.to_rotation(world_rot)
    if body is None:
        q = wr.GetQuat()
        return Gf.Vec3f(wp), Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))
    bm = tf.world_matrix(body)
    lp = bm.GetInverse().Transform(wp)
    rel = Gf.Matrix4d().SetRotate(wr) * mu.rigid_part(bm).GetInverse()
    q = rel.ExtractRotationQuat().GetNormalized()
    return Gf.Vec3f(lp), Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary()))


def world_frame(body: Optional[Usd.Prim], local_pos, local_rot) -> Tuple[Gf.Vec3d, Gf.Quatd]:
    """Inverse of :func:`local_frame`."""
    lp = Gf.Vec3d(local_pos) if local_pos is not None else Gf.Vec3d(0, 0, 0)
    lr = mu.to_rotation(local_rot) if local_rot is not None else Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
    if body is None:
        return lp, lr.GetQuat()
    bm = tf.world_matrix(body)
    wp = bm.Transform(lp)
    wr = (Gf.Matrix4d().SetRotate(lr) * mu.rigid_part(bm)).ExtractRotationQuat()
    return wp, wr


def joint_bodies(joint: Usd.Prim) -> Tuple[Optional[Usd.Prim], Optional[Usd.Prim]]:
    j = UsdPhysics.Joint(joint)
    stage = joint.GetStage()

    def first(rel):
        targets = rel.GetTargets() if rel else []
        if not targets:
            return None
        p = stage.GetPrimAtPath(targets[0])
        return p if p and p.IsValid() else None

    return first(j.GetBody0Rel()), first(j.GetBody1Rel())


def joint_world_frames(joint: Usd.Prim):
    """((pos0, rot0), (pos1, rot1)) — the joint frame as seen from each body, in world."""
    j = UsdPhysics.Joint(joint)
    b0, b1 = joint_bodies(joint)
    f0 = world_frame(b0, j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get())
    f1 = world_frame(b1, j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())
    return f0, f1


def joint_frame_error(joint: Usd.Prim) -> Tuple[float, float]:
    """(position error, angle error in degrees) between the two body-side frames."""
    (p0, r0), (p1, r1) = joint_world_frames(joint)
    return (p0 - p1).GetLength(), mu.quat_angle_between(r0, r1)


def realign_joint(joint: Usd.Prim, keep: str = "body0") -> None:
    """Recompute one side's local frame so both sides agree (``keep`` = side to trust)."""
    j = UsdPhysics.Joint(joint)
    b0, b1 = joint_bodies(joint)
    (p0, r0), (p1, r1) = joint_world_frames(joint)
    if keep == "body0":
        lp, lr = local_frame(b1, p0, r0)
        j.CreateLocalPos1Attr().Set(lp)
        j.CreateLocalRot1Attr().Set(lr)
    else:
        lp, lr = local_frame(b0, p1, r1)
        j.CreateLocalPos0Attr().Set(lp)
        j.CreateLocalRot0Attr().Set(lr)


# ---------------------------------------------------------------------------------------
# Joint creation
# ---------------------------------------------------------------------------------------
def default_joint_parent(stage: Usd.Stage) -> Sdf.Path:
    return default_root(stage).AppendChild("Joints")


def create_joint(
    stage: Usd.Stage,
    spec: JointSpec,
    body0: Optional[Usd.Prim],
    body1: Optional[Usd.Prim],
    world_pos: mu.VecLike,
    world_rot=None,
    parent_path: Optional[Union[str, Sdf.Path]] = None,
) -> Usd.Prim:
    """Create a joint between ``body0`` and ``body1`` (either may be None = world)."""
    if spec.joint_type not in _SCHEMAS:
        raise ValueError(f"Unknown joint type '{spec.joint_type}', expected one of {JOINT_TYPES}")
    if body0 is None and body1 is None:
        raise ValueError("A joint needs at least one body")
    if body0 is not None and body1 is not None and body0.GetPath() == body1.GetPath():
        raise ValueError("body0 and body1 must be different prims")
    world_rot = Gf.Quatd(1, 0, 0, 0) if world_rot is None else world_rot

    parent = Sdf.Path(str(parent_path)) if parent_path else default_joint_parent(stage)
    if not stage.GetPrimAtPath(parent):
        ensure_scope(stage, str(parent))
    name = spec.name or _auto_name(spec.joint_type, body0, body1)
    path = unique_child_path(stage, parent, name)

    joint = _SCHEMAS[spec.joint_type].Define(stage, path)
    prim = joint.GetPrim()
    if body0 is not None:
        joint.CreateBody0Rel().SetTargets([body0.GetPath()])
    if body1 is not None:
        joint.CreateBody1Rel().SetTargets([body1.GetPath()])

    lp0, lr0 = local_frame(body0, world_pos, world_rot)
    lp1, lr1 = local_frame(body1, world_pos, world_rot)
    joint.CreateLocalPos0Attr().Set(lp0)
    joint.CreateLocalRot0Attr().Set(lr0)
    joint.CreateLocalPos1Attr().Set(lp1)
    joint.CreateLocalRot1Attr().Set(lr1)

    joint.CreateCollisionEnabledAttr().Set(bool(spec.collision_enabled))
    if spec.exclude_from_articulation:
        joint.CreateExcludeFromArticulationAttr().Set(True)
    if spec.break_force is not None:
        joint.CreateBreakForceAttr().Set(float(spec.break_force))
    if spec.break_torque is not None:
        joint.CreateBreakTorqueAttr().Set(float(spec.break_torque))

    _apply_type_specifics(prim, spec)
    return prim


def create_joint_at(
    stage: Usd.Stage,
    spec: JointSpec,
    body0: Optional[Usd.Prim],
    body1: Optional[Usd.Prim],
    location: str = "body1",
    world_axis: Union[str, mu.VecLike] = "Z",
    anchor: Optional[Usd.Prim] = None,
    custom_position: Optional[mu.VecLike] = None,
    parent_path=None,
) -> Usd.Prim:
    """Convenience wrapper: pick the frame from a location keyword and a world axis.

    ``world_axis`` may also be "anchor" to use the anchor's orientation as the joint frame.
    """
    pos = resolve_location(stage, location, body0, body1, anchor, custom_position)
    if isinstance(world_axis, str) and world_axis == "anchor":
        if anchor is None:
            raise ValueError("world_axis='anchor' needs an anchor prim")
        rot = tf.world_rotation(anchor)
    else:
        rot = frame_for_world_axis(mu.axis_vector(world_axis), spec.axis)
    return create_joint(stage, spec, body0, body1, pos, rot, parent_path)


def set_drive(prim: Usd.Prim, drive_name: str, drive: DriveSpec) -> UsdPhysics.DriveAPI:
    api = UsdPhysics.DriveAPI.Apply(prim, drive_name)
    api.CreateTypeAttr().Set(drive.drive_type)
    api.CreateStiffnessAttr().Set(float(drive.stiffness))
    api.CreateDampingAttr().Set(float(drive.damping))
    if drive.max_force is not None:
        api.CreateMaxForceAttr().Set(float(drive.max_force))
    if drive.target_position is not None:
        api.CreateTargetPositionAttr().Set(float(drive.target_position))
    if drive.target_velocity is not None:
        api.CreateTargetVelocityAttr().Set(float(drive.target_velocity))
    return api


def set_limits(prim: Usd.Prim, lower: Optional[float], upper: Optional[float]) -> None:
    """Set lower/upper limits on a revolute or prismatic joint (None = unlimited)."""
    if prim.IsA(UsdPhysics.RevoluteJoint):
        j = UsdPhysics.RevoluteJoint(prim)
    elif prim.IsA(UsdPhysics.PrismaticJoint):
        j = UsdPhysics.PrismaticJoint(prim)
    else:
        raise ValueError("Limits apply to revolute / prismatic joints only")
    j.CreateLowerLimitAttr().Set(float("-inf") if lower is None else float(lower))
    j.CreateUpperLimitAttr().Set(float("inf") if upper is None else float(upper))


def _apply_type_specifics(prim: Usd.Prim, spec: JointSpec) -> None:
    t = spec.joint_type
    axis = spec.axis.upper()
    if t in ("revolute", "prismatic"):
        j = UsdPhysics.RevoluteJoint(prim) if t == "revolute" else UsdPhysics.PrismaticJoint(prim)
        j.CreateAxisAttr().Set(axis)
        if spec.lower is not None or spec.upper is not None:
            set_limits(prim, spec.lower, spec.upper)
        if spec.drive is not None:
            set_drive(prim, "angular" if t == "revolute" else "linear", spec.drive)
    elif t == "spherical":
        j = UsdPhysics.SphericalJoint(prim)
        j.CreateAxisAttr().Set(axis)
        if spec.cone0 is not None:
            j.CreateConeAngle0LimitAttr().Set(float(spec.cone0))
        if spec.cone1 is not None:
            j.CreateConeAngle1LimitAttr().Set(float(spec.cone1))
    elif t == "distance":
        j = UsdPhysics.DistanceJoint(prim)
        if spec.min_distance is not None:
            j.CreateMinDistanceAttr().Set(float(spec.min_distance))
        if spec.max_distance is not None:
            j.CreateMaxDistanceAttr().Set(float(spec.max_distance))
    elif t == "d6":
        for dof in D6_DOFS:
            mode = spec.d6.get(dof, "free")
            if mode == "free":
                continue
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            if mode == "locked":
                # PhysX convention: low > high means the DOF is locked
                lim.CreateLowAttr().Set(1.0)
                lim.CreateHighAttr().Set(-1.0)
            else:
                lo, hi = mode
                lim.CreateLowAttr().Set(float(lo))
                lim.CreateHighAttr().Set(float(hi))
        for dof, drive in spec.d6_drives.items():
            set_drive(prim, dof, drive)


# ---------------------------------------------------------------------------------------
# Bushings
# ---------------------------------------------------------------------------------------
BUSHING_ATTR = "designer:bushing"


def joint_free_axes(prim: Usd.Prim) -> List[str]:
    """Free DOFs of a joint as D6 names in the joint frame (a revolute about Y -> ["rotY"])."""
    if prim.IsA(UsdPhysics.FixedJoint):
        return []
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        rev = prim.IsA(UsdPhysics.RevoluteJoint)
        typed = UsdPhysics.RevoluteJoint(prim) if rev else UsdPhysics.PrismaticJoint(prim)
        return [("rot" if rev else "trans") + (typed.GetAxisAttr().Get() or "X").upper()]
    if prim.IsA(UsdPhysics.SphericalJoint):
        return ["rotX", "rotY", "rotZ"]
    if prim.IsA(UsdPhysics.DistanceJoint):
        raise ValueError(f"{prim.GetPath()}: a distance joint has no bushing equivalent")
    free = []
    for dof in D6_DOFS:  # D6: every axis that is not locked
        locked = False
        if prim.HasAPI(UsdPhysics.LimitAPI, dof):
            lim = UsdPhysics.LimitAPI(prim, dof)
            lo, hi = lim.GetLowAttr().Get(), lim.GetHighAttr().Get()
            locked = lo is not None and hi is not None and lo > hi
        if not locked:
            free.append(dof)
    return free


def bushing_spec(free: Sequence[str] = ("rotZ",), linear_stiffness: float = 1.0e7,
                 angular_stiffness: float = 2.0e3, linear_damping: Optional[float] = None,
                 angular_damping: Optional[float] = None, name: Optional[str] = None,
                 collision_enabled: bool = False) -> JointSpec:
    """A bushing in place of a joint: a D6 spring-damper, stiff on the axes the joint would
    constrain and zero on its ``free`` axes (the Adams / RecurDyn bushing, and the soft loop
    closure MuJoCo uses).  As a soft constraint it keeps PhysX GPU dynamics stable where a
    rigid loop closure diverges, and redundant load paths split by stiffness.

    Angular stiffness is per degree (PhysX drive convention).  Values are in the caller's
    units: stage units for :func:`create_joint`, SI when passed through :func:`spec_from_si`
    (template builders).  Damping defaults to stiffness / 1000 (a 1 ms time constant).
    """
    unknown = set(free) - set(D6_DOFS)
    if unknown:
        raise ValueError(f"unknown bushing axes {sorted(unknown)}")
    drives = {}
    for dof in D6_DOFS:
        if dof in free:
            continue
        linear = dof.startswith("trans")
        k = float(linear_stiffness if linear else angular_stiffness)
        c = linear_damping if linear else angular_damping
        drives[dof] = DriveSpec(stiffness=k, damping=k * 1e-3 if c is None else float(c), target_position=0.0)
    return JointSpec("d6", name=name, collision_enabled=collision_enabled,
                     exclude_from_articulation=True, d6_drives=drives)


def mark_bushing(prim: Usd.Prim) -> Usd.Prim:
    prim.CreateAttribute(BUSHING_ATTR, Sdf.ValueTypeNames.Bool, custom=True).Set(True)
    return prim


def is_bushing(prim: Usd.Prim) -> bool:
    attr = prim.GetAttribute(BUSHING_ATTR)
    return bool(attr and attr.Get())


def convert_to_bushing(joint: Usd.Prim, linear_stiffness: float = 1.0e7, angular_stiffness: float = 2.0e3,
                       linear_damping: Optional[float] = None, angular_damping: Optional[float] = None) -> Usd.Prim:
    """Replace ``joint`` in place by a bushing with the same bodies, frame and free axes.

    Stage units, angular stiffness per degree.  Only undriven joints convert (a drive or motion
    is an actuator, which a bushing cannot carry); a damping-only drive is dropped.  A tree
    joint of an articulation is refused: cutting it apart would change the mechanism — convert
    loop-closing (``excludeFromArticulation``) joints, which is where bushings help.
    """
    if not joint.IsA(UsdPhysics.Joint):
        raise ValueError(f"{joint.GetPath()} is not a joint")
    if is_bushing(joint):
        return joint
    for dof in ("linear", "angular") + D6_DOFS:
        if joint.HasAPI(UsdPhysics.DriveAPI, dof) and                 float(UsdPhysics.DriveAPI(joint, dof).GetStiffnessAttr().Get() or 0.0) > 0.0:
            raise ValueError(f"{joint.GetPath()} is driven: convert only undriven joints")
    if any(a.GetName().startswith("designer:motion") for a in joint.GetAuthoredAttributes()):
        raise ValueError(f"{joint.GetPath()} carries a motion: convert only undriven joints")
    free = joint_free_axes(joint)
    j = UsdPhysics.Joint(joint)
    b0, b1 = joint_bodies(joint)
    if not j.GetExcludeFromArticulationAttr().Get():
        from . import physics as ph

        if any(b is not None and ph.articulation_root_of(b) is not None for b in (b0, b1)):
            raise ValueError(f"{joint.GetPath()} is an articulation tree joint; a bushing here would cut "
                             "the articulation apart. Convert loop-closing joints (excludeFromArticulation).")
    (p0, r0), _ = joint_world_frames(joint)
    collision = bool(j.GetCollisionEnabledAttr().Get())
    stage = joint.GetStage()
    path = joint.GetPath()
    stage.RemovePrim(path)
    spec = bushing_spec(free, linear_stiffness, angular_stiffness, linear_damping, angular_damping,
                        name=path.name, collision_enabled=collision)
    return mark_bushing(create_joint(stage, spec, b0, b1, p0, r0, parent_path=path.GetParentPath()))


def _auto_name(joint_type: str, body0, body1) -> str:
    n0 = body0.GetName() if body0 is not None else "World"
    n1 = body1.GetName() if body1 is not None else "World"
    return f"{joint_type.capitalize()}Joint_{n0}_{n1}"


def list_joints(stage: Usd.Stage) -> List[Usd.Prim]:
    return [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
