"""Record an articulation's joint states and forces after every physics step (Isaac Sim 6).

Uses the ``isaacsim.core.experimental`` tensor API, i.e. the simulated state — not authored USD.
Values are converted to SI with the stage's metersPerUnit / kilogramsPerUnit.
Channels (positions in deg / m, forces in N·m / N):

* ``pos:<dof>``                  joint position
* ``force:<dof>``                joint force projected on the DOF axis (drive / actuator force)
* ``link_force:<link>:x|y|z``    6D reaction of the link's incoming joint, force part
* ``link_torque:<link>:x|y|z``   ... torque part (PhysX: child joint frame)

Joints excluded from the articulation (loop closures) are maximal-coordinate constraints whose
reactions the tensor API does not expose.
"""
from __future__ import annotations

import math
from typing import Optional

import carb
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager

from .core.results import Recording
from .core.stage_utils import si_factors


class ArticulationRecorder:
    def __init__(self, root_path: str, name: Optional[str] = None, decimation: int = 1):
        self.root_path = str(root_path)
        self.recording = Recording(name or self.root_path.rsplit("/", 1)[-1])
        self.decimation = max(1, int(decimation))
        self._art: Optional[Articulation] = None
        self._uid: Optional[int] = None
        self._time = 0.0
        self._steps = 0
        self._rotational = []
        self.unavailable: list = []

    @property
    def active(self) -> bool:
        return self._uid is not None

    def start(self) -> None:
        self.stop()
        # PhysX reports in stage units (e.g. kg*cm/s^2 on a centimetre stage): record SI
        self._si = si_factors(omni.usd.get_context().get_stage())
        self._art = Articulation(self.root_path)
        self._rotational = [str(t).endswith("Rotation") for t in self._art.dof_types]
        for name, rot in zip(self._art.dof_names, self._rotational):
            self.recording.units[f"pos:{name}"] = "deg" if rot else "m"
            self.recording.units[f"force:{name}"] = "N*m" if rot else "N"
        for link in self._art.link_names:
            for c in "xyz":
                self.recording.units[f"link_force:{link}:{c}"] = "N"
                self.recording.units[f"link_torque:{link}:{c}"] = "N*m"
        self._uid = SimulationManager.register_callback(self._on_step, event=SimulationEvent.PHYSICS_POST_STEP)

    def stop(self) -> None:
        if self._uid is not None:
            SimulationManager.deregister_callback(self._uid)
        self._uid = None

    def _read(self, name: str, fn):
        """Call a tensor getter; a quantity the active engine does not provide (e.g. joint
        forces on Newton) is reported once and then skipped."""
        if name in self.unavailable:
            return None
        try:
            return fn()
        except Exception as exc:  # backend without this quantity
            self.unavailable.append(name)
            carb.log_warn(f"[Physics Designer] recorder: '{name}' not available from this physics engine ({exc})")
            return None

    def _on_step(self, dt: float, context) -> None:
        self._time += dt
        self._steps += 1
        if self._steps % self.decimation:
            return
        art = self._art
        values = {}
        names = art.dof_names
        pos = self._read("dof positions", lambda: art.get_dof_positions().numpy()[0])
        si = self._si
        if pos is not None:
            for name, rot, p in zip(names, self._rotational, pos):
                values[f"pos:{name}"] = math.degrees(float(p)) if rot else float(p) * si["length"]
        frc = self._read("dof forces", lambda: art.get_dof_projected_joint_forces().numpy()[0])
        if frc is not None:
            for name, rot, f in zip(names, self._rotational, frc):
                values[f"force:{name}"] = float(f) * (si["torque"] if rot else si["force"])
        link = self._read("link forces", lambda: [a.numpy()[0] for a in art.get_link_incoming_joint_force()])
        if link is not None:
            for lname, fv, tv in zip(art.link_names, link[0], link[1]):
                for c, fc, tc in zip("xyz", fv, tv):
                    values[f"link_force:{lname}:{c}"] = float(fc) * si["force"]
                    values[f"link_torque:{lname}:{c}"] = float(tc) * si["torque"]
        self.recording.add_sample(self._time, values)
