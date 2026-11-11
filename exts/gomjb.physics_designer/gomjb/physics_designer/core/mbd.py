"""Exact constraint-based multibody analysis of a designer model (Exudyn backend).

The USD model is turned into a differential-algebraic system in absolute coordinates, as Adams
does: rigid bodies with their explicit mass properties, every joint an exact constraint
(Lagrange multipliers = joint reactions, loop-closing joints included), and every *driven* axis
a prescribed motion ("motion" in RecurDyn) whose multiplier is the exact actuator force.

* Bodies: prims with RigidBodyAPI (+ MassAPI mass / centerOfMass / diagonalInertia, as the
  Modeler authors them). Kinematic bodies and non-rigid joint targets are ground.
* Joints: fixed / revolute / prismatic / spherical / D6 (locked DOFs) as Exudyn generic joints,
  built from the joint's world frame (body0 side) at the authored pose.
* Drives with stiffness > 0 become prescribed coordinates at their target; damping-only drives
  and joint limits are not part of the analysis (listed in :attr:`MbdModel.notes`).
* Redundant constraints (e.g. revolute joints closing planar loops, or two sides of a scissor
  lift sharing a platform) make the reactions non-unique (Wojtyra 2005; García de Jalón &
  Gutiérrez-López 2013). Two treatments, see :class:`MbdModel`: ``"release"`` detects them
  numerically and releases one equation at a time — loop-closing joints first — keeping the
  mobility, like Adams / MotionSolve, and unlike Adams / Simscape reports what was released;
  ``"bushings"`` replaces undriven joints by stiff bushings so the split follows stiffness.

Inputs and outputs are SI (m, deg, N, N*m); the model itself runs in stage units.
Channel names match :mod:`recording`: ``pos:<joint>`` / ``force:<joint>`` for driven and free
joint axes, ``joint_force:<joint>:x|y|z`` and ``joint_torque:...`` for full reactions in the
joint frame. Forces are those the joint applies to its body1 (same sign as PhysX).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import transform as tf
from .results import Recording
from .stage_utils import si_factors

AXIS_NAMES = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")
TORQUE_SIGN = np.array([1.0, -1.0, -1.0])  # Exudyn TorqueLocal -> torque applied to body1 (see values())


@dataclass
class MbdJoint:
    name: str
    path: str
    kind: str
    index: int  # Exudyn object number
    constrained: List[int]
    free_axes: List[int]  # translation 0..2 / rotation 3..5 of the joint frame
    drives: Dict[int, float] = field(default_factory=dict)  # axis -> target (stage units / rad)
    loop: bool = False
    j0: np.ndarray = field(default_factory=lambda: np.eye(3))  # joint frame axes in marker 0's body frame
    body0: int = -1  # Exudyn object of marker 0's body, -1 = ground
    bushing: bool = False  # modelled as a stiff spring-damper instead of a constraint


REDUNDANCY_MODES = ("release", "bushings")


class MbdModel:
    """``redundancy``:

    * ``"release"`` — exact joints; dependent equations are found numerically and released (the
      Adams / MotionSolve / Simscape practice). Totals are exact, the split over redundant load
      paths is one admissible solution, and :attr:`released` lists what was released.
    * ``"bushings"`` — every joint that is not driven becomes a stiff 6-DOF bushing (zero stiffness
      on its free axes): the load split then follows stiffness like the real structure, which is the
      fix Adams / Altair recommend for asymmetric results. ``bushing_stiffness`` = (N/m, N*m/rad).
    """

    def __init__(self, stage: Usd.Stage, root_path: str, redundancy: str = "release",
                 bushing_stiffness: Tuple[float, float] = (1.0e8, 1.0e7)):
        import exudyn as exu
        from exudyn.utilities import MarkerBodyRigid, ObjectGround, ObjectJointGeneric, RigidBodyInertia

        if redundancy not in REDUNDANCY_MODES:
            raise ValueError(f"redundancy must be one of {REDUNDANCY_MODES}")
        self._exu = exu
        self.stage = stage
        self.root_path = str(root_path)
        self.redundancy = redundancy
        self.si = si_factors(stage)
        # SI stiffness -> stage units (force / length, torque / rad)
        self._k = (bushing_stiffness[0] * self.si["length"] / self.si["force"],
                   bushing_stiffness[1] / self.si["torque"])
        self.notes: List[str] = []
        self.released: List[Tuple[str, str]] = []
        self.joints: Dict[str, MbdJoint] = {}
        self.bodies: Dict[str, int] = {}

        self.SC = exu.SystemContainer()
        mbs = self.mbs = self.SC.AddSystem()
        self._ground = mbs.AddObject(ObjectGround())
        gravity = _gravity(stage)

        root = stage.GetPrimAtPath(self.root_path)
        if not root:
            raise ValueError(f"{self.root_path} does not exist")
        for prim in Usd.PrimRange(root):
            if not _is_dynamic_body(prim):
                continue
            self.bodies[str(prim.GetPath())] = mbs.CreateRigidBody(
                name=prim.GetName(), inertia=_inertia(prim, RigidBodyInertia), gravity=list(gravity),
                referencePosition=list(tf.world_position(prim)),
                referenceRotationMatrix=_rot_matrix(mu.rotation_of(tf.world_matrix(prim))))
        if not self.bodies:
            raise ValueError(f"no dynamic rigid bodies under {self.root_path}")

        # joints anywhere on the stage (the Joint tool puts them under /World/Joints) that act on these bodies
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint) and any(
                    b is not None and str(b.GetPath()) in self.bodies for b in jt.joint_bodies(prim)):
                self._add_joint(prim, MarkerBodyRigid, ObjectJointGeneric)
        mbs.Assemble()
        dof = mbs.ComputeSystemDegreeOfFreedom()
        self.dof = int(dof["degreeOfFreedom"])  # bushings: DOF held by springs count as free here
        self.redundant = int(dof["redundantConstraints"])
        if self.redundant:
            self._release_redundant()

    # -- model building ------------------------------------------------------------------
    def _side(self, body: Optional[Usd.Prim]) -> Tuple[int, Gf.Matrix4d]:
        """(Exudyn body, its world frame); world, kinematic and static targets are the ground."""
        if body is not None and str(body.GetPath()) in self.bodies:
            return self.bodies[str(body.GetPath())], mu.rigid_part(tf.world_matrix(body))
        return self._ground, Gf.Matrix4d(1)

    def _add_joint(self, prim: Usd.Prim, MarkerBodyRigid, ObjectJointGeneric) -> None:
        j = UsdPhysics.Joint(prim)
        name = prim.GetName()
        if j.GetJointEnabledAttr().Get() is False:
            self.notes.append(f"{name}: disabled joint skipped")
            return
        b0, b1 = jt.joint_bodies(prim)
        (p0, r0), _ = jt.joint_world_frames(prim)
        frame = mu.make_transform(p0, r0)
        constrained, free = _constrained_axes(prim)
        drives = {}
        for axis, drive_name in _drive_axes(prim, free):
            api = UsdPhysics.DriveAPI(prim, drive_name)
            if float(api.GetStiffnessAttr().Get() or 0.0) > 0.0:
                target = float(api.GetTargetPositionAttr().Get() or 0.0)
                drives[axis] = math.radians(target) if axis >= 3 else target
                constrained[axis] = 1
            else:
                self.notes.append(f"{name}: damping-only drive ignored")
        if _has_limits(prim):
            self.notes.append(f"{name}: joint limits not enforced")
        markers, rotations, indices = [], [], []
        for body in (b0, b1):
            index, body_frame = self._side(body)
            local = frame * body_frame.GetInverse()
            markers.append(self.mbs.AddMarker(MarkerBodyRigid(bodyNumber=index,
                                                              localPosition=list(local.ExtractTranslation()))))
            rotations.append(_rot_matrix(mu.rotation_of(local)))
            indices.append(index)
        mj = MbdJoint(name, str(prim.GetPath()), prim.GetTypeName(), -1, list(constrained),
                      [a for a in free if a not in drives], drives, bool(j.GetExcludeFromArticulationAttr().Get()),
                      rotations[0], -1 if indices[0] == self._ground else int(indices[0]))
        if self.redundancy == "bushings" and not drives:
            from exudyn.utilities import ObjectConnectorRigidBodySpringDamper

            k = np.diag([self._k[0] if c and a < 3 else self._k[1] if c else 0.0
                         for a, c in enumerate(constrained)])
            mj.bushing = True
            mj.index = int(self.mbs.AddObject(ObjectConnectorRigidBodySpringDamper(
                name=name, markerNumbers=markers, stiffness=k, damping=k * 1e-3,
                rotationMarker0=rotations[0], rotationMarker1=rotations[1], useIntrinsicFormulation=True,
                visualization={"show": False, "drawSize": -1.0, "color": [-1, -1, -1, -1]})))
            self.joints[name] = mj
            return
        mj.index = int(self.mbs.AddObject(ObjectJointGeneric(
            name=name, markerNumbers=markers, constrainedAxes=list(constrained),
            rotationMarker0=rotations[0], rotationMarker1=rotations[1],
            offsetUserFunctionParameters=_j0_values(drives),
            offsetUserFunction=lambda mbs_, t, item, p, _mj=mj: _exudyn_offset(mbs_, _mj, p),
            visualization={"show": False, "axesRadius": 0.0, "axesLength": 0.0, "color": [-1, -1, -1, -1]})))
        self.joints[name] = mj

    def _release_redundant(self) -> None:
        """Release dependent constraint equations until none is left, keeping the mobility."""
        mbs = self.mbs
        target_dof = self.dof
        candidates = sorted((m for m in self.joints.values() if not m.bushing),
                            key=lambda m: (not m.loop, list(self.joints).index(m.name)))
        for mj in candidates:
            for axis in _release_order(mj):
                if self.redundant == 0:
                    return
                if not mj.constrained[axis] or axis in mj.drives:
                    continue
                mj.constrained[axis] = 0
                mbs.SetObjectParameter(mj.index, "constrainedAxes", list(mj.constrained))
                mbs.Assemble()
                res = mbs.ComputeSystemDegreeOfFreedom()
                if res["redundantConstraints"] < self.redundant and res["degreeOfFreedom"] == target_dof:
                    self.redundant = int(res["redundantConstraints"])
                    self.released.append((mj.name, AXIS_NAMES[axis]))
                else:
                    mj.constrained[axis] = 1
                    mbs.SetObjectParameter(mj.index, "constrainedAxes", list(mj.constrained))
        mbs.Assemble()
        if self.redundant:
            self.notes.append(f"{self.redundant} redundant constraint(s) could not be released")

    # -- drives -------------------------------------------------------------------------
    @property
    def drive_names(self) -> List[str]:
        return [n for n, m in self.joints.items() if m.drives]

    def set_drive(self, name: str, value_si: float) -> None:
        """Set a driven joint's target: metres for a linear axis, degrees for a rotation."""
        mj = self.joints[name]
        axis = next(iter(mj.drives))
        mj.drives[axis] = math.radians(value_si) if axis >= 3 else value_si / self.si["length"]
        self.mbs.SetObjectParameter(mj.index, "offsetUserFunctionParameters", _j0_values(mj.drives))

    def drive_value(self, name: str) -> float:
        mj = self.joints[name]
        axis, value = next(iter(mj.drives.items()))
        return math.degrees(value) if axis >= 3 else value * self.si["length"]

    # -- solving ------------------------------------------------------------------------
    def _settings(self):
        ss = self._exu.SimulationSettings()
        ss.solution.file.write = False
        ss.solution.sensors.writePeriod = 1e9
        ss.staticSolver.verboseMode = 0
        ss.timeIntegration.verboseMode = 0
        ss.staticSolver.newton.relativeTolerance = 1e-10
        ss.staticSolver.newton.absoluteTolerance = 1e-12
        ss.linearSolver.solverType = self._exu.LinearSolverType.EigenDense
        return ss

    def solve_static(self, steps: int = 1) -> Dict[str, float]:
        """Static equilibrium with the current drive targets. ``steps`` > 1 reaches the targets
        through intermediate configurations (continuation), for large moves."""
        finals = {n: self.drive_value(n) for n in self.drive_names}
        solved = getattr(self, "_last_target", {})
        starts = {n: solved.get(n, 0.0) for n in finals}  # 0 = the authored pose
        for k in range(1, steps + 1):
            for n in finals:
                self.set_drive(n, starts[n] + (finals[n] - starts[n]) * k / steps)
            self._static_step()
        return self.values()

    def _static_step(self) -> None:
        exu = self._exu
        mbs = self.mbs
        mbs.SolveStatic(self._settings())
        current = mbs.systemData.GetODE2Coordinates(exu.ConfigurationType.Current)
        mbs.systemData.SetODE2Coordinates(current, exu.ConfigurationType.Initial)  # continuation
        self._last_target = {n: self.drive_value(n) for n in self.drive_names}

    def static_solution(self, steps: int = 10) -> Recording:
        """:meth:`solve_static` at the current drive targets as a one-sample recording."""
        rec = self._recording(f"{self.root_path.rsplit('/', 1)[-1]}_static")
        rec.x_label = "solution"
        rec.add_sample(0.0, self.solve_static(steps))
        return rec

    def static_sweep(self, drives: Sequence[str], values_si: Sequence[float], substeps: int = 5) -> Recording:
        """Static solutions along a stroke of ``drives`` moved together (e.g. the paired actuators
        of a two-sided lift); ``values_si`` in m or deg, x = drive value. Other drives hold."""
        units = {"deg" if next(iter(self.joints[d].drives)) >= 3 else "m" for d in drives}
        if len(units) != 1:
            raise ValueError("drives swept together must all be linear or all angular")
        rec = self._recording(f"{self.root_path.rsplit('/', 1)[-1]}_static")
        rec.x_label = f"{', '.join(drives)} [{units.pop()}]"
        starts = {d: self.drive_value(d) for d in drives}
        for v in values_si:
            for k in range(1, substeps + 1):
                for d in drives:
                    self.set_drive(d, starts[d] + (v - starts[d]) * k / substeps)
                self._static_step()
            starts = dict.fromkeys(drives, v)
            rec.add_sample(v, self.values())
        return rec

    def simulate(self, end_time: float, profiles: Dict[str, Callable[[float], float]], step: float = 1e-3,
                 sample: float = 1.0 / 60.0) -> Recording:
        """Dynamic analysis (implicit generalized-alpha, index 3) with drive profiles f(t) [m or deg]."""
        exu = self._exu
        mbs = self.mbs
        for name in profiles:
            mj = self.joints[name]
            axis = next(iter(mj.drives))
            scale = math.radians(1.0) if axis >= 3 else 1.0 / self.si["length"]
            fn = profiles[name]
            others = dict(mj.drives)

            def offset(mbs_, t, item, p, _mj=mj, _axis=axis, _scale=scale, _fn=fn, _others=others):
                return _exudyn_offset(mbs_, _mj, _j0_values({**_others, _axis: _fn(t) * _scale}))

            mbs.SetObjectParameter(mj.index, "offsetUserFunction", offset)
        rec = self._recording(f"{self.root_path.rsplit('/', 1)[-1]}_dynamic")
        state = {"next": 0.0}

        def sampler(mbs_, t):
            if t + 1e-12 >= state["next"]:
                rec.add_sample(t, self.values())
                state["next"] += sample
            return True

        mbs.SetPreStepUserFunction(sampler)
        mbs.Assemble()
        ss = self._settings()
        ss.timeIntegration.endTime = end_time
        ss.timeIntegration.numberOfSteps = max(1, int(round(end_time / step)))
        ss.timeIntegration.generalizedAlpha.spectralRadius = 0.8
        ss.timeIntegration.generalizedAlpha.useIndex2Constraints = False
        ss.timeIntegration.generalizedAlpha.useNewmark = False
        mbs.SolveDynamic(ss, solverType=exu.DynamicSolverType.GeneralizedAlpha)
        rec.add_sample(end_time, self.values())
        return rec

    # -- results ------------------------------------------------------------------------
    def _recording(self, name: str) -> Recording:
        rec = Recording(name)
        for n, mj in self.joints.items():
            for axis in list(mj.drives) + mj.free_axes:
                rot = axis >= 3
                rec.units[f"pos:{n}"] = "deg" if rot else "m"
                rec.units[f"force:{n}"] = "N*m" if rot else "N"
            for c in "xyz":
                rec.units[f"joint_force:{n}:{c}"] = "N"
                rec.units[f"joint_torque:{n}:{c}"] = "N*m"
        return rec

    def values(self) -> Dict[str, float]:
        exu = self._exu
        mbs = self.mbs
        out: Dict[str, float] = {}
        f_si, t_si, l_si = self.si["force"], self.si["torque"], self.si["length"]
        for n, mj in self.joints.items():
            # In J0: ForceLocal is what the joint applies to marker 1's body; TorqueLocal holds the
            # rotation-constraint multipliers, whose y and z components carry the opposite sign
            # (measured on revolute joints about each J0 axis against static moment balance).
            # Bushings (spring-dampers) report the force and torque on marker 0's body: negate.
            force = np.asarray(mbs.GetObjectOutput(mj.index, exu.OutputVariableType.ForceLocal)) * f_si
            torque = np.asarray(mbs.GetObjectOutput(mj.index, exu.OutputVariableType.TorqueLocal)) * t_si
            if mj.bushing:
                force, torque = -force, -torque
            else:
                torque = torque * TORQUE_SIGN
            for c, fc, tc in zip("xyz", force, torque):
                out[f"joint_force:{n}:{c}"] = float(fc)
                out[f"joint_torque:{n}:{c}"] = float(tc)
            axes = list(mj.drives) + mj.free_axes
            if len(axes) == 1:  # a single-DOF joint: position and axial force/torque
                axis = axes[0]
                if axis >= 3:
                    rot = mbs.GetObjectOutput(mj.index, exu.OutputVariableType.Rotation)
                    out[f"pos:{n}"] = math.degrees(float(rot[axis - 3]))
                    out[f"force:{n}"] = float(torque[axis - 3])
                else:
                    disp = mbs.GetObjectOutput(mj.index, exu.OutputVariableType.DisplacementLocal)
                    out[f"pos:{n}"] = float(disp[axis]) * l_si
                    out[f"force:{n}"] = float(force[axis])
        return out

    def report(self) -> str:
        bushings = [n for n, m in self.joints.items() if m.bushing]
        lines = [f"bodies {len(self.bodies)}, joints {len(self.joints)}, drives: "
                 f"{', '.join(self.drive_names) or 'none'}, redundancy handling: {self.redundancy}"]
        if bushings:
            lines.append(f"{len(bushings)} joint(s) as stiff bushings (k = {self._k[0] * self.si['force'] / self.si['length']:.3g}"
                         f" N/m, {self._k[1] * self.si['torque']:.3g} N*m/rad): load split follows stiffness")
        else:
            lines.append(f"mobility with drives locked: {self.dof}")
        if self.released:
            lines.append("released redundant equations: " + ", ".join(f"{j}.{a}" for j, a in self.released))
            lines.append("-> reactions over redundant load paths are not unique (totals are exact); "
                         "use redundancy='bushings' for a stiffness-based split")
        lines += self.notes
        return "\n".join(lines)


# ---------------------------------------------------------------------------------------
def _j0_values(values: Dict[int, float]) -> List[float]:
    """6-vector of drive values in the joint frame (J0): translations, then rotations."""
    out = [0.0] * 6
    for axis, value in values.items():
        out[axis] = value
    return out


def _exudyn_offset(mbs, mj: "MbdJoint", j0_values) -> List[float]:
    """Offset vector in Exudyn's convention for joint-frame drive values.

    Exudyn 1.13 applies rotation offsets about the joint (J0) axes but translation offsets in
    *global* coordinates (measured: with marker 0's body yawed 90 deg, a J0 offset moved the
    slider along the global axis of the same index), so translations are rotated from J0 into
    the global frame with marker 0's body orientation at this instant."""
    import exudyn as exu

    p = np.asarray(j0_values, dtype=float)
    rot = np.eye(3)
    if mj.body0 >= 0:
        rot = np.asarray(mbs.GetObjectOutputBody(mj.body0, exu.OutputVariableType.RotationMatrix,
                                                 localPosition=[0.0, 0.0, 0.0])).reshape(3, 3)
    return list(rot @ mj.j0 @ p[:3]) + list(p[3:])


def _is_dynamic_body(prim: Usd.Prim) -> bool:
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return False
    api = UsdPhysics.RigidBodyAPI(prim)
    return api.GetRigidBodyEnabledAttr().Get() is not False and not api.GetKinematicEnabledAttr().Get()


def _gravity(stage: Usd.Stage) -> Gf.Vec3d:
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            scene = UsdPhysics.Scene(prim)
            d = Gf.Vec3d(scene.GetGravityDirectionAttr().Get() or Gf.Vec3f(0))
            m = scene.GetGravityMagnitudeAttr().Get()
            if d.GetLength() > 0 and m is not None and not math.isinf(m) and m >= 0:
                return d.GetNormalized() * float(m)
    up = mu.AXIS_VECTORS[UsdGeom.GetStageUpAxis(stage) or "Z"]
    return -up * (9.81 / si_factors(stage)["length"])


def _inertia(prim: Usd.Prim, RigidBodyInertia):
    api = UsdPhysics.MassAPI(prim)
    mass, diag = api.GetMassAttr().Get(), api.GetDiagonalInertiaAttr().Get()
    if not prim.HasAPI(UsdPhysics.MassAPI) or not mass or diag is None or not any(diag):
        raise ValueError(f"{prim.GetPath()} needs explicit mass properties (create it with the Modeler or "
                         "author MassAPI mass and diagonalInertia)")
    _, _, scale = mu.decompose(tf.world_matrix(prim))
    if any(abs(s - 1.0) > 1e-6 for s in scale):
        raise ValueError(f"{prim.GetPath()} is scaled; put the scale on its geometry, not on the body")
    com = api.GetCenterOfMassAttr().Get()
    com = [0.0, 0.0, 0.0] if com is None or any(math.isinf(c) for c in com) else list(com)
    axes = api.GetPrincipalAxesAttr().Get()
    R = _rot_matrix(Gf.Rotation(Gf.Quatd(axes.GetReal(), Gf.Vec3d(axes.GetImaginary())))) if axes else np.eye(3)
    tensor = R @ np.diag(list(diag)) @ R.T
    return RigidBodyInertia(mass=float(mass), inertiaTensor=tensor, com=np.array(com), inertiaTensorAtCOM=True)


def _rot_matrix(rot: Gf.Rotation) -> np.ndarray:
    """3x3 matrix mapping local to global column vectors (Exudyn convention)."""
    m = Gf.Matrix4d().SetRotate(rot)
    return np.array([[m[c][r] for c in range(3)] for r in range(3)])  # Gf is row-vector: transpose


def _constrained_axes(prim: Usd.Prim) -> Tuple[List[int], List[int]]:
    """(constrained flags, free single-DOF axes) in the joint frame."""
    if prim.IsA(UsdPhysics.FixedJoint):
        return [1] * 6, []
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        typed = UsdPhysics.RevoluteJoint(prim) if prim.IsA(UsdPhysics.RevoluteJoint) else UsdPhysics.PrismaticJoint(prim)
        axis = "XYZ".index(typed.GetAxisAttr().Get() or "X")
        free = axis + 3 if prim.IsA(UsdPhysics.RevoluteJoint) else axis
        flags = [1] * 6
        flags[free] = 0
        return flags, [free]
    if prim.IsA(UsdPhysics.SphericalJoint):
        return [1, 1, 1, 0, 0, 0], []
    if prim.IsA(UsdPhysics.DistanceJoint):
        raise ValueError(f"{prim.GetPath()}: distance joints are not supported by the exact solver yet")
    flags = [0] * 6  # D6: locked DOFs only
    for i, dof in enumerate(AXIS_NAMES):
        if prim.HasAPI(UsdPhysics.LimitAPI, dof):
            lim = UsdPhysics.LimitAPI(prim, dof)
            lo, hi = lim.GetLowAttr().Get(), lim.GetHighAttr().Get()
            if lo is not None and hi is not None and lo > hi:
                flags[i] = 1
    return flags, [i for i in range(6) if not flags[i]]


def _drive_axes(prim: Usd.Prim, free: List[int]):
    if prim.IsA(UsdPhysics.RevoluteJoint) and free:
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            yield free[0], "angular"
    elif prim.IsA(UsdPhysics.PrismaticJoint) and free:
        if prim.HasAPI(UsdPhysics.DriveAPI, "linear"):
            yield free[0], "linear"
    elif not (prim.IsA(UsdPhysics.FixedJoint) or prim.IsA(UsdPhysics.SphericalJoint)):
        for i in free:
            if prim.HasAPI(UsdPhysics.DriveAPI, AXIS_NAMES[i]):
                yield i, AXIS_NAMES[i]


def _has_limits(prim: Usd.Prim) -> bool:
    for schema in (UsdPhysics.RevoluteJoint, UsdPhysics.PrismaticJoint):
        if prim.IsA(schema):
            typed = schema(prim)
            lo, hi = typed.GetLowerLimitAttr().Get(), typed.GetUpperLimitAttr().Get()
            return any(v is not None and not math.isinf(v) for v in (lo, hi))
    return False


def _release_order(mj: MbdJoint) -> List[int]:
    """Equations to try first: translation along / rotations across a single free axis (the ones
    planar loops make redundant), then the rest."""
    order = []
    if len(mj.free_axes) + len(mj.drives) == 1:
        axis = (mj.free_axes + list(mj.drives))[0] % 3
        order = [axis] + [3 + i for i in range(3) if i != axis]
    return order + [a for a in range(6) if a not in order]
