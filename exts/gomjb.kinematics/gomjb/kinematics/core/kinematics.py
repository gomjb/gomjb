"""Mechanism kinematics from the exact solver, for kinematic mechanisms in the Isaac Sim
simulation (:mod:`kinematic_mode`). The position analysis of :class:`mbd.MbdModel` places every
body at given drive values with the loops closed exactly; PhysX then moves the bodies there as
kinematic bodies and simulates only what touches them.

Two sources of the body frames, the mechanism's mode:

* :class:`MotionTable` (``"table"``) — the position analysis along the drive stroke, once before
  the simulation: stroke value -> every body's frame, interpolated while simulating (no solver
  in the loop; one table serves any number of copies of the mechanism, on the GPU). A table
  covers one stroke: the driven joints with a motion must share that motion (e.g. the two
  actuators of a lift), and its range is their joint limits; values beyond it are clamped.
  The stroke points are refined where the kinematics bends until interpolating between them
  is within a tolerance (:func:`build_table`) — near a singular pose (a scissor folding flat)
  the bodies move fast for little stroke, and an even spacing would be far off there. Tables are
  kept on disk under the mechanism's kinematic identity (:func:`cached_table`), so a model is
  tabulated once and every later run — another process too — reads it.
* :class:`CoSimulation` (``"cosim"``) — the position analysis at the drive values of every
  physics step: exact at any drive values and for any number of independent motions, one CPU
  solve per step.

Drive values come from the joints' motions (:mod:`motion`); drives without a motion hold their
targets. Frames are root-local (relative to the mechanism root's rigid frame) in stage units,
``x y z qx qy qz qw`` — the layout of PhysX kinematic targets.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import asdict, astuple, dataclass
from typing import List, Tuple

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import math_utils as mu
from gomjb.physics_designer.core import motion as mo
from gomjb.physics_designer.core import transform as tf

from . import kinematic_mode as km
from .mbd import MbdModel

INITIAL_POINTS = 17  # even stroke points a table starts from before refining
MAX_POINTS = 4097  # refinement stops here (a singular pose inside the stroke)


def kinematic_model(stage: Usd.Stage, root_path: str) -> MbdModel:
    """The exact model of a mechanism for its kinematics: exact joints (bushings as the joints
    they replace, redundant equations released), every DOF driven."""
    model = MbdModel(stage, root_path, redundancy="release", rigid_bushings=True)
    if model.dof:
        raise ValueError(f"{root_path}: {model.dof} DOF without a drive - a kinematic mechanism needs a drive "
                         "(stiffness > 0) on every free axis, so that the drives fix its pose")
    return model


class _Frames:
    """The model's body frames at its current solution, root-local, stage units."""

    def __init__(self, model: MbdModel):
        root = model.stage.GetPrimAtPath(model.root_path)
        self._root_inv = mu.rigid_part(tf.world_matrix(root)).GetInverse()
        self._to_stage = 1.0 / model.si["length"]
        self._model = model
        self.bodies = [p[len(model.root_path) + 1:] for p in model.bodies]

    def __call__(self) -> np.ndarray:
        poses = self._model.body_poses()
        out = np.empty((len(self.bodies), 7))
        for i, name in enumerate(self.bodies):
            p, q = poses[name]
            world = Gf.Matrix4d().SetRotate(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
            world.SetTranslateOnly(Gf.Vec3d(*p) * self._to_stage)
            local = world * self._root_inv
            r = local.ExtractRotationQuat()
            out[i, :3] = local.ExtractTranslation()
            out[i, 3:6] = r.GetImaginary()
            out[i, 6] = r.GetReal()
        return out


def _hemisphere(ref: np.ndarray, frames: np.ndarray) -> np.ndarray:
    """``frames`` with each quaternion on the side of ``ref``'s (q and -q are one orientation),
    so that interpolating between them takes the short way."""
    out = frames.copy()
    out[np.einsum("bi,bi->b", ref[:, 3:], frames[:, 3:]) < 0, 3:] *= -1.0
    return out


def _lerp(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    out = a + s * (b - a)
    out[..., 3:] /= np.linalg.norm(out[..., 3:], axis=-1, keepdims=True)
    return out


def interpolate(stroke: np.ndarray, frames: np.ndarray, value: float) -> np.ndarray:
    """Frames (bodies, 7) at stroke ``value`` from a table (``stroke`` increasing, ``frames``
    (points, bodies, 7)): linear in position, normalized linear in orientation, clamped."""
    value = min(max(value, stroke[0]), stroke[-1])
    k = min(max(int(np.searchsorted(stroke, value, side="right")) - 1, 0), len(stroke) - 2)
    return _lerp(frames[k], frames[k + 1], (value - stroke[k]) / (stroke[k + 1] - stroke[k]))


@dataclass
class MotionTable:
    drives: List[str]  # driven joints that follow the stroke
    motion: mo.Motion  # their shared motion
    linear: bool  # stroke in m (True) or deg
    stroke: np.ndarray  # (points,) increasing stroke values (SI)
    bodies: List[str]  # body paths relative to the mechanism root
    frames: np.ndarray  # (points, bodies, 7): root-local frames, stage units, x y z qx qy qz qw
    error: Tuple[float, float]  # largest measured interpolation error: position [m], angle [deg]
    tolerance: float  # m, what the table was refined to (error[0] + radius * error[1] in rad)

    def profile(self) -> mo.Profile:
        """Stroke value of time (the drives' motion)."""
        return mo.Profile(self.motion, self.linear)

    def at(self, value: float) -> np.ndarray:
        return interpolate(self.stroke, self.frames, value)


def _stroke(model: MbdModel) -> Tuple[List[str], mo.Motion, bool]:
    profiles = model.motion_profiles()
    if not profiles:
        raise ValueError("no driven joint has a motion: give the driving joint(s) a motion (Include Motion)")
    drives = list(profiles)
    motions = [mo.get_motion(model.stage.GetPrimAtPath(model.joints[d].path)) for d in drives]
    units = {model.drive_unit(d) for d in drives}
    if len({astuple(m) for m in motions}) > 1 or len(units) > 1:
        raise ValueError(f"a motion table covers one stroke, but the motions of {', '.join(drives)} differ - "
                         "use co-simulation")
    return drives, motions[0], units.pop() == "m"


def _limits(model: MbdModel, drives: List[str], linear: bool) -> Tuple[float, float]:
    """Common range of the drives' joint limits (SI)."""
    lo, hi = -math.inf, math.inf
    scale = model.si["length"] if linear else 1.0
    for d in drives:
        prim = model.stage.GetPrimAtPath(model.joints[d].path)
        typed = UsdPhysics.PrismaticJoint(prim) if linear else UsdPhysics.RevoluteJoint(prim)
        a, b = typed.GetLowerLimitAttr().Get(), typed.GetUpperLimitAttr().Get()
        if a is None or b is None or math.isinf(a) or math.isinf(b):
            raise ValueError(f"{d}: set its joint limits - they are the range of the motion table")
        lo, hi = max(lo, a * scale), min(hi, b * scale)
    if not lo < hi:
        raise ValueError(f"the joint limits of {', '.join(drives)} do not overlap")
    return lo, hi


def build_table(model: MbdModel, tolerance: float = km.TABLE_TOLERANCE) -> MotionTable:
    """Position analysis along the stroke (the drives' joint limits): :data:`INITIAL_POINTS` even
    points, then every interval whose mid-point the interpolation misses by more than
    ``tolerance`` [m] is halved, until all are within it (or :data:`MAX_POINTS`). The error is
    measured at a body's origin plus its rotation error at the mechanism's radius, against the
    exact solution at the mid-point (solved from the interval's start). An accepted interval
    keeps its mid-point too, so the table's error is at most :attr:`MotionTable.error`."""
    drives, motion, linear = _stroke(model)
    lo, hi = _limits(model, drives, linear)
    frames = _Frames(model)
    length = model.si["length"]
    radius = max(float(np.linalg.norm(frames()[:, :3], axis=1).max()) * length, 1e-9)  # m, at the authored pose

    def solve(v: float):
        model.solve_position(dict.fromkeys(drives, float(v)))
        return float(v), frames(), model.solution()

    grid = np.linspace(lo, hi, INITIAL_POINTS)
    i0 = int(np.argmin(np.abs(grid - model.drive_value(drives[0]))))
    for v in grid[i0::-1]:  # continuation from the current pose to the lower end of the stroke
        solve(v)
    points = [solve(v) for v in grid]
    for k in range(1, len(points)):
        points[k] = (points[k][0], _hemisphere(points[k - 1][1], points[k][1]), points[k][2])
    open_ = [True] * (len(points) - 1)  # intervals still to check
    worst = missed = (0.0, 0.0)
    while any(open_) and len(points) < MAX_POINTS:
        refined, still, missed = [points[0]], [], (0.0, 0.0)
        for (a, b), check in zip(zip(points, points[1:]), open_):
            if not check:
                refined.append(b)
                still.append(False)
                continue
            model.restore(a[2])
            v, f, sol = solve(0.5 * (a[0] + b[0]))
            f = _hemisphere(a[1], f)
            guess = _lerp(a[1], b[1], 0.5)
            dp = float(np.linalg.norm(guess[:, :3] - f[:, :3], axis=1).max()) * length
            da = float(np.degrees(2.0 * np.arccos(np.clip(np.abs(np.einsum("bi,bi->b", guess[:, 3:], f[:, 3:])),
                                                           0.0, 1.0))).max())
            ok = dp + radius * math.radians(da) <= tolerance
            if ok:
                worst = (max(worst[0], dp), max(worst[1], da))
            else:
                missed = (max(missed[0], dp), max(missed[1], da))
            refined += [(v, f, sol), b]
            still += [not ok, not ok]
        points, open_ = refined, still
    if any(open_):  # stopped at MAX_POINTS: the error is what the last refinement still missed
        worst = (max(worst[0], missed[0]), max(worst[1], missed[1]))
    return MotionTable(drives, motion, linear, np.array([p[0] for p in points]), frames.bodies,
                       np.array([p[1] for p in points]), worst, float(tolerance))


TABLE_FORMAT = 1  # raise when build_table or the saved layout changes: older cached tables are not used


def table_key(stage: Usd.Stage, fingerprint: str) -> str:
    """Cache key of a mechanism's motion table: its kinematic identity
    (:func:`kinematic_mode.fingerprint`), the stage's scale (frames are in stage units) and the
    table format."""
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    return hashlib.sha1(f"{fingerprint}:{mpu!r}:{TABLE_FORMAT}".encode()).hexdigest()


def save_table(table: MotionTable, path: str) -> None:
    """Write ``table`` (.npz); atomically, so a process reading the cache never sees half a file."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "wb") as f:
        np.savez(f, drives=np.array(table.drives), motion=np.array(json.dumps(asdict(table.motion))),
                 linear=table.linear, stroke=table.stroke, bodies=np.array(table.bodies), frames=table.frames,
                 error=np.array(table.error), tolerance=table.tolerance)
    os.replace(tmp, path)


def load_table(path: str) -> MotionTable:
    with np.load(path, allow_pickle=False) as d:
        return MotionTable([str(s) for s in d["drives"]], mo.Motion(**json.loads(str(d["motion"]))),
                           bool(d["linear"]), d["stroke"], [str(s) for s in d["bodies"]], d["frames"],
                           (float(d["error"][0]), float(d["error"][1])), float(d["tolerance"]))


def cached_table(stage: Usd.Stage, root_path: str, fingerprint: str, folder: str) -> Tuple[MotionTable, bool]:
    """(the mechanism's motion table, whether it came from ``folder``): built and stored there
    under :func:`table_key` the first time, read back afterwards — by any process."""
    path = os.path.join(folder, table_key(stage, fingerprint) + ".npz")
    if os.path.exists(path):
        return load_table(path), True
    table = build_table(kinematic_model(stage, root_path), km.get_tolerance(stage.GetPrimAtPath(root_path)))
    os.makedirs(folder, exist_ok=True)
    save_table(table, path)
    return table, False


class CoSimulation:
    """The position analysis at every call: frames at time ``t`` of the joints' motions."""

    def __init__(self, model: MbdModel):
        self.model = model
        self._profiles = model.motion_profiles()
        self._frames = _Frames(model)
        self.bodies = self._frames.bodies
        if self._profiles:  # reach the motions' start through intermediate poses
            model.solve_position({n: f(0.0) for n, f in self._profiles.items()}, substeps=10)

    def frames(self, t: float) -> np.ndarray:
        if self._profiles:
            self.model.solve_position({n: f(t) for n, f in self._profiles.items()})
        return self._frames()
