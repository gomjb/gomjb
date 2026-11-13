"""Automatic bushings: joints that become stiff spring-dampers with a sized stiffness.

A bushing replaces a rigid joint by a 6-DOF spring-damper (a D6 joint whose locked axes are
drives, see :func:`joints.bushing_spec`): stiff on the axes the joint constrains, zero on its free
axes.  Rigid loop closures are what iterative solvers converge worst on (PhysX GPU diverged on a
closed-loop articulation in Isaac Sim 6.1) and what makes reactions over redundant load paths
non-unique; a bushing turns them into a soft constraint, as MuJoCo does for every constraint and
Adams / RecurDyn recommend for over-constrained models.

**Stiffness sizing** (:func:`auto_stiffness`), from the mechanism the joint belongs to:

* load ``W = M·g``, with ``M`` the mass of the dynamic bodies connected to the joint;
* linear ``k = W / δ`` — the whole weight deflects the bushing by ``δ`` (default 0.1 mm);
* angular ``k_θ = W·L / θ`` — the weight at the mechanism's size ``L`` (its bounding-box
  diagonal) twists it by ``θ`` (default 0.01°);
* damping ``c = 2ζ·√(k·m_eff)`` on each axis (default ζ = 1, critical), ``m_eff`` the reduced mass
  of the two bodies (their reduced principal inertia for rotations): the bushing's own vibration
  dies out instead of ringing.

So the bushing deflects no more than the tolerance under the full weight — stiff enough to act as
the joint, yet a spring rather than a hard constraint.  PhysX drives are implicit, so these
stiffnesses are stable at any time step; :attr:`Stiffness.omega_dt` reports how far above the step
rate the bushing's natural frequency sits.

**When a joint becomes a bushing** (:data:`MODES`):

* ``"auto"`` — only when it closes a kinematic loop (its two bodies are already connected through
  other joints or both reach the ground) and it is not driven.  Tree joints stay exact joints,
  articulations keep their reduced coordinates and only the loop closures are soft.
* ``"bushing"`` — every undriven joint.
* ``"rigid"`` — never (exact joints, as before).

Driven joints always stay joints: a drive or motion is an actuator, which a bushing cannot carry.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set

from pxr import Gf, Usd, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import transform as tf
from . import units as un
from .stage_utils import si_factors

MODES = ("auto", "bushing", "rigid")
GROUND = "<ground>"
G = 9.81


@dataclass
class Stiffness:
    """A bushing's stiffness and damping, in SI (angular per degree, the PhysX drive convention)."""

    linear: float  # N/m
    angular: float  # N*m/deg
    linear_damping: float  # N*s/m
    angular_damping: float  # N*m*s/deg
    mass: float = 0.0  # kg of the mechanism the load is taken from
    size: float = 0.0  # m, its bounding-box diagonal
    omega_dt: float = 0.0  # linear natural frequency x physics time step

    def in_stage_units(self, stage: Usd.Stage) -> "Stiffness":
        s = lambda q: un.si_per_stage_unit(stage, q)  # noqa: E731
        return Stiffness(self.linear / s("linear_stiffness"), self.angular / s("angular_stiffness"),
                         self.linear_damping / s("linear_damping"), self.angular_damping / s("angular_damping"),
                         self.mass, self.size, self.omega_dt)

    def describe(self) -> str:
        return (f"k = {self.linear:.3g} N/m, {self.angular:.3g} N*m/deg; c = {self.linear_damping:.3g} N*s/m, "
                f"{self.angular_damping:.3g} N*m*s/deg (mechanism {self.mass:.3g} kg, {self.size:.3g} m; "
                f"omega*dt = {self.omega_dt:.3g})")


# ---------------------------------------------------------------------------------------
# Joint graph
# ---------------------------------------------------------------------------------------
def _is_dynamic(prim: Optional[Usd.Prim]) -> bool:
    if prim is None or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return False
    api = UsdPhysics.RigidBodyAPI(prim)
    return api.GetRigidBodyEnabledAttr().Get() is not False and not api.GetKinematicEnabledAttr().Get()


def _node(prim: Optional[Usd.Prim]) -> str:
    """Graph node of a joint side: a dynamic body, or the ground (world, static, kinematic)."""
    return str(prim.GetPath()) if _is_dynamic(prim) else GROUND


class _Graph:
    def __init__(self, stage: Usd.Stage, skip: Sequence[Usd.Prim] = ()):
        skip_paths = {p.GetPath() for p in skip}
        self.parent: Dict[str, str] = {}
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint) or prim.GetPath() in skip_paths:
                continue
            if UsdPhysics.Joint(prim).GetJointEnabledAttr().Get() is False:
                continue
            b0, b1 = jt.joint_bodies(prim)
            self.union(_node(b0), _node(b1))

    def find(self, n: str) -> str:
        self.parent.setdefault(n, n)
        while self.parent[n] != n:
            self.parent[n] = self.parent[self.parent[n]]
            n = self.parent[n]
        return n

    def union(self, a: str, b: str) -> None:
        self.parent[self.find(a)] = self.find(b)

    def component(self, n: str) -> Set[str]:
        root = self.find(n)
        return {m for m in list(self.parent) if self.find(m) == root}


def closes_loop(stage: Usd.Stage, body0: Optional[Usd.Prim], body1: Optional[Usd.Prim],
                ignore: Sequence[Usd.Prim] = ()) -> bool:
    """Whether a joint between ``body0`` and ``body1`` closes a kinematic loop: the two sides are
    already connected through other joints (``ignore``: joints left out, e.g. the joint itself).
    Two grounded sides count, since the ground is one body."""
    n0, n1 = _node(body0), _node(body1)
    if n0 == n1:
        return n0 == GROUND  # ground to ground: nothing to close
    g = _Graph(stage, ignore)
    return g.find(n0) == g.find(n1)


def mechanism_bodies(stage: Usd.Stage, body0: Optional[Usd.Prim], body1: Optional[Usd.Prim],
                     ignore: Sequence[Usd.Prim] = ()) -> List[Usd.Prim]:
    """Dynamic bodies connected (through joints, not through the ground) to either side."""
    nodes: Set[str] = set()
    for b in (body0, body1):
        n = _node(b)
        if n != GROUND:
            nodes |= _component_no_ground(stage, n, ignore)
    return [stage.GetPrimAtPath(n) for n in sorted(nodes)]


def _component_no_ground(stage: Usd.Stage, start: str, ignore: Sequence[Usd.Prim]) -> Set[str]:
    """Bodies reachable from ``start`` through joints without passing through the ground (the
    ground joins every grounded mechanism on the stage, which must not pool their masses)."""
    skip = {p.GetPath() for p in ignore}
    adj: Dict[str, Set[str]] = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and prim.GetPath() not in skip:
            b0, b1 = jt.joint_bodies(prim)
            n0, n1 = _node(b0), _node(b1)
            if GROUND not in (n0, n1):
                adj.setdefault(n0, set()).add(n1)
                adj.setdefault(n1, set()).add(n0)
    seen, todo = {start}, [start]
    while todo:
        for m in adj.get(todo.pop(), ()):
            if m not in seen:
                seen.add(m)
                todo.append(m)
    return seen


# ---------------------------------------------------------------------------------------
# Stiffness sizing
# ---------------------------------------------------------------------------------------
def _mass_si(prim: Usd.Prim) -> float:
    if prim.HasAPI(UsdPhysics.MassAPI):
        m = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        if m and m > 0:
            return float(m) * si_factors(prim.GetStage())["mass"]
    return 0.0


def _inertia_si(prim: Usd.Prim, mass: float, size: float) -> float:
    """Largest principal inertia (kg*m^2): authored, else a solid sphere of the body's bbox."""
    f = si_factors(prim.GetStage())
    if prim.HasAPI(UsdPhysics.MassAPI):
        diag = UsdPhysics.MassAPI(prim).GetDiagonalInertiaAttr().Get()
        if diag is not None and any(diag):
            return float(max(diag)) * f["mass"] * f["length"] ** 2
    box = tf.world_bbox(prim)
    r = (box.GetSize().GetLength() / 2 * f["length"]) if not box.IsEmpty() else size / 4
    return 0.4 * mass * r * r


def _reduced(a: float, b: Optional[float]) -> float:
    if b is None:  # the other side is the ground
        return a
    return a * b / (a + b) if a + b > 0 else 0.0


def _time_step(stage: Usd.Stage) -> float:
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
            if attr and attr.Get():
                return 1.0 / float(attr.Get())
    return 1.0 / 60.0  # PhysX default


def auto_stiffness(stage: Usd.Stage, body0: Optional[Usd.Prim], body1: Optional[Usd.Prim],
                   deflection: float = 1.0e-4, angle_deflection: float = 0.01, damping_ratio: float = 1.0,
                   ignore: Sequence[Usd.Prim] = ()) -> Stiffness:
    """Bushing stiffness for a joint between ``body0`` and ``body1`` (SI; see the module docs).

    ``deflection`` (m) and ``angle_deflection`` (deg): what the mechanism's full weight may
    deflect the bushing by.  ``ignore``: joints left out of the mechanism graph.
    """
    if deflection <= 0 or angle_deflection <= 0:
        raise ValueError("deflection tolerances must be positive")
    bodies = mechanism_bodies(stage, body0, body1, ignore)
    mass = sum(_mass_si(b) for b in bodies) or 1.0
    length = si_factors(stage)["length"]
    box = tf.combined_bbox(bodies) if bodies else Gf.Range3d()
    size = box.GetSize().GetLength() * length if not box.IsEmpty() else 1.0
    size = size or 1.0

    weight = mass * G
    k_lin = weight / deflection
    k_rad = weight * size / math.radians(angle_deflection)  # N*m/rad

    sides = [b if _is_dynamic(b) else None for b in (body0, body1)]
    masses = [(_mass_si(b) or mass) if b is not None else None for b in sides]
    inertias = [_inertia_si(b, m, size) if b is not None else None for b, m in zip(sides, masses)]
    dyn = [i for i, b in enumerate(sides) if b is not None]
    if not dyn:
        m_eff, i_eff = mass, 0.4 * mass * (size / 2) ** 2
    elif len(dyn) == 1:
        m_eff, i_eff = masses[dyn[0]], inertias[dyn[0]]
    else:
        m_eff, i_eff = _reduced(masses[0], masses[1]), _reduced(inertias[0], inertias[1])
    c_lin = 2.0 * damping_ratio * math.sqrt(k_lin * m_eff)
    c_rad = 2.0 * damping_ratio * math.sqrt(k_rad * max(i_eff, 1e-12))
    omega_dt = math.sqrt(k_lin / m_eff) * _time_step(stage) if m_eff > 0 else 0.0
    deg = math.radians(1.0)  # per-radian -> per-degree
    return Stiffness(k_lin, k_rad * deg, c_lin, c_rad * deg, mass, size, omega_dt)


# ---------------------------------------------------------------------------------------
# Creation
# ---------------------------------------------------------------------------------------
def spec_free_axes(spec: jt.JointSpec) -> List[str]:
    """Free DOFs (D6 names) of the joint ``spec`` describes."""
    t, axis = spec.joint_type, spec.axis.upper()
    if t == "fixed":
        return []
    if t == "revolute":
        return [f"rot{axis}"]
    if t == "prismatic":
        return [f"trans{axis}"]
    if t == "spherical":
        return ["rotX", "rotY", "rotZ"]
    if t == "d6":
        return [d for d in jt.D6_DOFS if spec.d6.get(d, "free") != "locked"]
    raise ValueError(f"a {t} joint has no bushing equivalent")


def _driven(spec: jt.JointSpec) -> bool:
    if spec.drive is not None and (spec.drive.stiffness or 0) > 0:
        return True
    return any((d.stiffness or 0) > 0 for d in spec.d6_drives.values())


def should_be_bushing(stage: Usd.Stage, spec: jt.JointSpec, body0, body1, mode: str = "auto") -> bool:
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if mode == "rigid" or _driven(spec) or spec.joint_type == "distance":
        return False
    if spec.joint_type == "d6" and spec.d6_drives:
        return False
    if any(v not in ("free", "locked") for v in spec.d6.values()):
        return False  # ranged D6 limits are limits, not a bushing
    if mode == "bushing":
        return True
    return closes_loop(stage, body0, body1)


def create_joint(stage: Usd.Stage, spec: jt.JointSpec, body0: Optional[Usd.Prim], body1: Optional[Usd.Prim],
                 world_pos: mu.VecLike, world_rot=None, parent_path=None, mode: str = "auto",
                 deflection: float = 1.0e-4, angle_deflection: float = 0.01,
                 damping_ratio: float = 1.0) -> Usd.Prim:
    """:func:`joints.create_joint`, made a bushing with :func:`auto_stiffness` when ``mode`` says
    so (``spec`` in stage units, tolerances in SI). The prim records the sizing in
    ``designer:bushing:*`` attributes."""
    if not should_be_bushing(stage, spec, body0, body1, mode):
        return jt.create_joint(stage, spec, body0, body1, world_pos, world_rot, parent_path)
    k = auto_stiffness(stage, body0, body1, deflection, angle_deflection, damping_ratio)
    ks = k.in_stage_units(stage)
    name = spec.name or jt._auto_name(spec.joint_type, body0, body1).replace("Joint_", "Bushing_", 1)
    bspec = jt.bushing_spec(spec_free_axes(spec), ks.linear, ks.angular, ks.linear_damping, ks.angular_damping,
                            name=name, collision_enabled=spec.collision_enabled)
    bspec.break_force, bspec.break_torque = spec.break_force, spec.break_torque
    prim = jt.mark_bushing(jt.create_joint(stage, bspec, body0, body1, world_pos, world_rot, parent_path))
    _record(prim, spec.joint_type, k)
    return prim


def convert(joint: Usd.Prim, deflection: float = 1.0e-4, angle_deflection: float = 0.01,
            damping_ratio: float = 1.0) -> Usd.Prim:
    """Replace an existing undriven joint by an auto-sized bushing (see :func:`joints.convert_to_bushing`)."""
    stage = joint.GetStage()
    b0, b1 = jt.joint_bodies(joint)
    kind = joint.GetTypeName()
    k = auto_stiffness(stage, b0, b1, deflection, angle_deflection, damping_ratio, ignore=[joint])
    ks = k.in_stage_units(stage)
    prim = jt.convert_to_bushing(joint, ks.linear, ks.angular, ks.linear_damping, ks.angular_damping)
    _record(prim, kind, k)
    return prim


def _record(prim: Usd.Prim, replaces: str, k: Stiffness) -> None:
    from pxr import Sdf

    prim.CreateAttribute("designer:bushing:replaces", Sdf.ValueTypeNames.String, custom=True).Set(replaces)
    prim.CreateAttribute("designer:bushing:sizing", Sdf.ValueTypeNames.String, custom=True).Set(k.describe())
