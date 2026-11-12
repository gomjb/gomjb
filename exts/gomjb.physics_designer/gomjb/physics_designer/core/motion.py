"""Joint motions, RecurDyn's "Include Motion": the relative motion of a revolute or translational joint
prescribed by an expression of TIME, as a displacement, velocity or acceleration.

* Revolute joints in degrees (deg, deg/s, deg/s^2); translational joints in the length unit of
  the unit system the motion was written in, which is stored with it. Velocity motions take an
  initial displacement, acceleration motions an initial displacement and velocity.
* The motion lives on the joint prim (``designer:motion:*``), so it travels with the model.
* In the PhysX simulation the joint's position drive tracks it (targets set every physics step by
  :mod:`motion_runtime`); a joint without a drive gets a stiff tracking drive. The exact solver
  prescribes it exactly (:meth:`mbd.MbdModel.simulate`).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

from pxr import Sdf, Usd, UsdPhysics

from . import joints as jt
from . import units as un
from .expressions import compile_expression

MOTION_TYPES = ("displacement", "velocity", "acceleration")
PREFIX = "designer:motion:"
# tracking drive for joints without one (SI): stiffness, damping
TRACKING = {"linear": (1.0e7, 1.0e5), "angular": (1.0e4, 1.0e2)}  # N/m, N*s/m / N*m/deg, N*m*s/deg
STEP_LIMIT = 1.0e-3  # s, integration sub-step of velocity / acceleration motions


@dataclass
class Motion:
    type: str = "displacement"
    expression: str = "0"
    units: str = "MKS"  # unit system of the lengths (translational joints)
    initial_displacement: float = 0.0
    initial_velocity: float = 0.0


def drive_name(joint: Usd.Prim) -> str:
    return "linear" if joint.IsA(UsdPhysics.PrismaticJoint) else "angular"


def _check_joint(joint: Usd.Prim) -> None:
    if not (joint.IsA(UsdPhysics.RevoluteJoint) or joint.IsA(UsdPhysics.PrismaticJoint)):
        raise ValueError(f"{joint.GetName()}: motions apply to revolute and translational joints")


def get_motion(joint: Usd.Prim) -> Optional[Motion]:
    attr = joint.GetAttribute(PREFIX + "type")
    if not attr or not attr.HasAuthoredValue():
        return None

    def get(name, default):
        a = joint.GetAttribute(PREFIX + name)
        return a.Get() if a and a.HasAuthoredValue() else default

    return Motion(str(attr.Get()), get("expression", "0"), get("units", "MKS"),
                  float(get("initialDisplacement", 0.0)), float(get("initialVelocity", 0.0)))


def set_motion(joint: Usd.Prim, motion: Motion) -> None:
    """Store ``motion`` on ``joint`` and make sure a position drive can track it."""
    _check_joint(joint)
    if motion.type not in MOTION_TYPES:
        raise ValueError(f"motion type must be one of {MOTION_TYPES}")
    un.system(motion.units)
    compile_expression(motion.expression)  # reject a bad expression before anything is written
    for name, typ, value in (("type", Sdf.ValueTypeNames.Token, motion.type),
                             ("expression", Sdf.ValueTypeNames.String, motion.expression),
                             ("units", Sdf.ValueTypeNames.Token, motion.units),
                             ("initialDisplacement", Sdf.ValueTypeNames.Double, float(motion.initial_displacement)),
                             ("initialVelocity", Sdf.ValueTypeNames.Double, float(motion.initial_velocity))):
        joint.CreateAttribute(PREFIX + name, typ, custom=True).Set(value)
    name = drive_name(joint)
    api = UsdPhysics.DriveAPI(joint, name)
    if not joint.HasAPI(UsdPhysics.DriveAPI, name) or not (api.GetStiffnessAttr().Get() or 0.0) > 0.0:
        k, c = TRACKING[name]
        drive = jt.drive_from_si(joint.GetStage(), jt.DriveSpec(stiffness=k, damping=c), name == "linear")
        jt.set_drive(joint, name, drive)
        joint.CreateAttribute(PREFIX + "trackingDrive", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
    apply_targets(joint, Profile(motion, name == "linear"), 0.0)


def clear_motion(joint: Usd.Prim) -> None:
    """Remove the motion (and the tracking drive it added, if any)."""
    tracking = joint.GetAttribute(PREFIX + "trackingDrive")
    if tracking and tracking.Get():
        name = drive_name(joint)
        joint.RemoveAPI(UsdPhysics.DriveAPI, name)
        for prop in list(joint.GetPropertyNames()):
            if prop.startswith(f"drive:{name}:"):
                joint.RemoveProperty(prop)
    for prop in list(joint.GetPropertyNames()):
        if prop.startswith(PREFIX):
            joint.RemoveProperty(prop)


class Profile:
    """A motion's displacement (m or deg) and velocity at time t; velocity and acceleration motions
    are integrated (RK4, sub-steps <= STEP_LIMIT), incrementally for increasing times."""

    def __init__(self, motion: Motion, linear: bool):
        self.motion = motion
        self.f = compile_expression(motion.expression)
        self.k = un.si_per_unit(un.system(motion.units), "length") if linear else 1.0
        self._reset()

    def _reset(self) -> None:
        self._t, self._x, self._v = 0.0, self.k * self.motion.initial_displacement, self.k * self.motion.initial_velocity

    def _deriv(self, t: float, x: float, v: float) -> Tuple[float, float]:
        if self.motion.type == "velocity":
            return self.k * self.f(t), 0.0
        return v, self.k * self.f(t)  # acceleration

    def _advance(self, t: float) -> None:
        if t < self._t:
            self._reset()
        while self._t < t - 1e-15:
            h = min(STEP_LIMIT, t - self._t)
            t0, x, v = self._t, self._x, self._v
            k1 = self._deriv(t0, x, v)
            k2 = self._deriv(t0 + h / 2, x + h / 2 * k1[0], v + h / 2 * k1[1])
            k3 = self._deriv(t0 + h / 2, x + h / 2 * k2[0], v + h / 2 * k2[1])
            k4 = self._deriv(t0 + h, x + h * k3[0], v + h * k3[1])
            self._x = x + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
            self._v = v + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
            self._t = t0 + h

    def value(self, t: float) -> float:
        """Displacement at ``t`` in m (translational) or deg (revolute)."""
        if self.motion.type == "displacement":
            return self.k * self.f(t)
        self._advance(t)
        return self._x

    def velocity(self, t: float) -> float:
        if self.motion.type == "displacement":
            h = 1e-5
            return self.k * (self.f(t + h) - self.f(max(t - h, 0.0))) / (t + h - max(t - h, 0.0))
        if self.motion.type == "velocity":
            return self.k * self.f(t)
        self._advance(t)
        return self._v


def apply_targets(joint: Usd.Prim, profile: Profile, t: float) -> None:
    """Drive targets of ``joint`` at time ``t`` (stage units for lengths, deg for angles)."""
    name = drive_name(joint)
    scale = 1.0 / un.si_per_stage_unit(joint.GetStage(), "length") if name == "linear" else 1.0
    api = UsdPhysics.DriveAPI(joint, name)
    api.CreateTargetPositionAttr().Set(float(profile.value(t) * scale))
    api.CreateTargetVelocityAttr().Set(float(profile.velocity(t) * scale))


def profiles(stage: Usd.Stage, skip: Optional[Callable[[Usd.Prim], bool]] = None) -> Dict[str, Profile]:
    """Joint path -> profile for every joint with a motion, not looking inside prims ``skip`` accepts."""
    out = {}
    it = iter(Usd.PrimRange.Stage(stage))
    for prim in it:
        if skip is not None and skip(prim):
            it.PruneChildren()
            continue
        if prim.IsA(UsdPhysics.Joint):
            motion = get_motion(prim)
            if motion is not None:
                out[str(prim.GetPath())] = Profile(motion, prim.IsA(UsdPhysics.PrismaticJoint))
    return out
