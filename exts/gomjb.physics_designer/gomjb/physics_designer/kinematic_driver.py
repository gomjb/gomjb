"""Kinematic mechanisms in the running simulation (Isaac Sim runtime): before every physics step
the bodies of a mechanism in a kinematic mode (:mod:`core.kinematic_mode`) are moved to the frames
of its kinematics (:mod:`core.kinematics`) through the tensor API (``omni.physics.tensors``) on
the simulation device:

* CPU pipeline — kinematic targets at the end of the step (the pattern of the PhysX samples):
  PhysX moves the bodies there and derives their velocity.
* GPU pipeline — PhysX 110.3 does not implement kinematic targets there ("GPU Rigid Body View
  kinematic target setting not implemented"), so the bodies are placed at the frame of the step's
  start and given the velocity that reaches the next one (centre of mass, finite difference).

Either way PhysX knows each body's velocity, so what rests on the mechanism is carried by
contact and friction. One Warp kernel looks the stroke up in the motion table (or takes the
co-simulation's frames) and composes the frames with the root of every copy of the mechanism:
N cloned mechanisms cost one table and one launch per step.
"""
from __future__ import annotations

import time
from typing import List, Optional, Sequence

import numpy as np
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd, UsdPhysics

from .core import kinematic_mode as km
from .core import kinematics as kn
from .core import math_utils as mu
from .core import transform as tf


@wp.func
def _lookup(stroke: wp.array(dtype=wp.float32), frames: wp.array3d(dtype=wp.float32), b: int, value: float):
    """Table frame of body b at a stroke value: binary search, linear position, normalized linear
    orientation, clamped to the table's range."""
    n = stroke.shape[0]
    k = int(0)
    s = float(0.0)
    if n > 1:
        v = wp.clamp(value, stroke[0], stroke[n - 1])
        lo = int(0)
        hi = n - 1
        while hi - lo > 1:
            mid = (lo + hi) >> 1
            if stroke[mid] <= v:
                lo = mid
            else:
                hi = mid
        k = lo
        s = (v - stroke[k]) / (stroke[k + 1] - stroke[k])
    k1 = wp.min(k + 1, n - 1)
    p = wp.vec3(frames[k, b, 0], frames[k, b, 1], frames[k, b, 2]) * (1.0 - s) \
        + wp.vec3(frames[k1, b, 0], frames[k1, b, 1], frames[k1, b, 2]) * s
    q = wp.normalize(wp.quat(frames[k, b, 3], frames[k, b, 4], frames[k, b, 5], frames[k, b, 6]) * (1.0 - s)
                     + wp.quat(frames[k1, b, 3], frames[k1, b, 4], frames[k1, b, 5], frames[k1, b, 6]) * s)
    return wp.transform(p, q)


@wp.kernel
def _states(stroke: wp.array(dtype=wp.float32), frames: wp.array3d(dtype=wp.float32),
            now: wp.array(dtype=wp.float32), after: wp.array(dtype=wp.float32), inv_dt: float,
            roots: wp.array(dtype=wp.transform), com: wp.array(dtype=wp.vec3), slots: wp.array2d(dtype=wp.int32),
            pose_now: wp.array2d(dtype=wp.float32), pose_next: wp.array2d(dtype=wp.float32),
            velocity: wp.array2d(dtype=wp.float32)):
    """Copy i, body b: world frames at the stroke values now[i] and after[i] (the start and the end
    of the step) and the velocity between them (linear at the centre of mass, angular in rad/s),
    written to view row slots[i, b] (x y z qx qy qz qw / vx vy vz wx wy wz)."""
    i, b = wp.tid()
    a = wp.transform_multiply(roots[i], _lookup(stroke, frames, b, now[i]))
    c = wp.transform_multiply(roots[i], _lookup(stroke, frames, b, after[i]))
    j = slots[i, b]
    pa = wp.transform_get_translation(a)
    qa = wp.transform_get_rotation(a)
    pc = wp.transform_get_translation(c)
    qc = wp.transform_get_rotation(c)
    for k in range(3):
        pose_now[j, k] = pa[k]
        pose_next[j, k] = pc[k]
    for k in range(4):
        pose_now[j, 3 + k] = qa[k]
        pose_next[j, 3 + k] = qc[k]
    v = (wp.transform_point(c, com[b]) - wp.transform_point(a, com[b])) * inv_dt
    d = qc * wp.quat_inverse(qa)
    if d[3] < 0.0:
        d = -d
    axis = wp.vec3(d[0], d[1], d[2])
    sin_half = wp.length(axis)
    w = wp.vec3(0.0)
    if sin_half > 1.0e-12:
        w = axis * (2.0 * wp.atan2(sin_half, d[3]) / sin_half * inv_dt)
    for k in range(3):
        velocity[j, k] = v[k]
        velocity[j, 3 + k] = w[k]


def _frame(m) -> wp.transform:
    """World frame of a prim (its rigid part, stage units)."""
    m = mu.rigid_part(m)
    q = m.ExtractRotationQuat()
    return wp.transform(wp.vec3(*m.ExtractTranslation()), wp.quat(*q.GetImaginary(), q.GetReal()))


def _com(body: Usd.Prim) -> List[float]:
    c = UsdPhysics.MassAPI(body).GetCenterOfMassAttr().Get() if body.HasAPI(UsdPhysics.MassAPI) else None
    return [0.0, 0.0, 0.0] if c is None or any(abs(v) == float("inf") for v in c) else [float(v) for v in c]


class KinematicDriver:
    """Moves the bodies of the mechanism under ``root_path`` — and of every copy of it whose root
    is in ``instances`` (cloned environments: the same body paths below their root) — to its
    kinematics. Created once the physics simulation view exists (after Play)."""

    def __init__(self, stage: Usd.Stage, root_path: str, instances: Optional[Sequence[str]] = None):
        root = stage.GetPrimAtPath(root_path)
        self.root_path = str(root_path)
        self.mode = km.get_mode(root)
        if self.mode not in km.KINEMATIC_MODES:
            raise ValueError(f"{root_path} is not in a kinematic mode")
        t0 = time.perf_counter()
        model = kn.kinematic_model(stage, self.root_path)
        self.table: Optional[kn.MotionTable] = None
        self._cosim: Optional[kn.CoSimulation] = None
        if self.mode == "table":
            self.table = kn.build_table(model, km.get_tolerance(root))
            self._profile = self.table.profile()
            self.bodies = self.table.bodies
            stroke, frames = self.table.stroke, self.table.frames
        else:  # two rows, the frames at the step's start and end: stroke 0 -> 1
            self._cosim = kn.CoSimulation(model)
            self._profile = None
            self.bodies = self._cosim.bodies
            stroke, frames = np.array([0.0, 1.0]), np.repeat(self._cosim.frames(0.0)[None], 2, axis=0)
        self.build_time = time.perf_counter() - t0

        instances = [str(p) for p in (instances or [self.root_path])]
        paths = [f"{inst}/{b}" for inst in instances for b in self.bodies]
        self._view = SimulationManager.get_physics_simulation_view().create_rigid_body_view(paths)
        rows = {str(p): i for i, p in enumerate(self._view.prim_paths)}
        missing = [p for p in paths if p not in rows]
        if missing:
            raise ValueError(f"not rigid bodies in the simulation: {', '.join(missing[:3])}")
        self._device = dev = SimulationManager.get_device()
        self._gpu = dev.is_cuda
        n = len(instances)
        self._stroke = wp.array(np.asarray(stroke, np.float32), dtype=wp.float32, device=dev)
        self._frames = wp.array(np.asarray(frames, np.float32), dtype=wp.float32, device=dev)
        self._now = wp.zeros(n, dtype=wp.float32, device=dev)
        self._after = wp.zeros(n, dtype=wp.float32, device=dev)
        if self._cosim is not None:
            self._after.fill_(1.0)
        self._roots = wp.array([_frame(tf.world_matrix(stage.GetPrimAtPath(p))) for p in instances],
                               dtype=wp.transform, device=dev)
        self._com = wp.array(np.asarray([_com(stage.GetPrimAtPath(f"{self.root_path}/{b}")) for b in self.bodies],
                                        np.float32), dtype=wp.vec3, device=dev)
        self._slots = wp.array(np.asarray([[rows[f"{inst}/{b}"] for b in self.bodies] for inst in instances],
                                          np.int32), dtype=wp.int32, device=dev)
        self._pose_now = wp.zeros((len(paths), 7), dtype=wp.float32, device=dev)
        self._pose_next = wp.zeros((len(paths), 7), dtype=wp.float32, device=dev)
        self._velocity = wp.zeros((len(paths), 6), dtype=wp.float32, device=dev)
        self._rows = wp.array(np.arange(len(paths), dtype=np.uint32), dtype=wp.uint32, device=dev)
        self._dim = (n, len(self.bodies))
        self._next_frames: Optional[np.ndarray] = None  # co-simulation: last step's end = this step's start

    def report(self) -> str:
        name = self.root_path.rsplit("/", 1)[-1]
        if self._dim[0] > 1:
            name += f" x {self._dim[0]} copies"
        how = "velocity + pose (GPU)" if self._gpu else "kinematic targets (CPU)"
        if self.table is not None:
            t = self.table
            return (f"{name}: motion table of {', '.join(t.drives)}, {len(t.stroke)} points, interpolation error "
                    f"{t.error[0] * 1e3:.3g} mm / {t.error[1]:.3g} deg (tolerance {t.tolerance * 1e3:.3g} mm), "
                    f"built in {self.build_time:.1f} s; {how}")
        return f"{name}: co-simulation, position analysis every physics step; {how}"

    def apply(self, t: float, dt: float) -> None:
        """Move the bodies over the coming physics step, from time ``t`` to ``t + dt``."""
        if self._profile is not None:
            self._now.fill_(float(self._profile.value(t)))
            self._after.fill_(float(self._profile.value(t + dt)))
        else:
            start = self._next_frames if self._next_frames is not None else self._cosim.frames(t)
            self._next_frames = self._cosim.frames(t + dt)
            self._frames.assign(np.asarray(np.stack([start, self._next_frames]), np.float32))
        wp.launch(_states, dim=self._dim, device=self._device,
                  inputs=[self._stroke, self._frames, self._now, self._after, 1.0 / dt, self._roots, self._com,
                          self._slots, self._pose_now, self._pose_next, self._velocity])
        if self._gpu:
            self._view.set_transforms(self._pose_now, self._rows)
            self._view.set_velocities(self._velocity, self._rows)
        else:
            self._view.set_kinematic_targets(self._pose_next, self._rows)
