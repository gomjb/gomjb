"""Joint motions in the running simulation (Isaac Sim runtime). Before every physics step:

* the drive targets of each simulated joint with a motion (core.motion) are set to the motion at
  the end of the step (mechanisms in the "physics" mode);
* the bodies of each mechanism in a kinematic mode get their kinematic targets from the exact
  solver's kinematics (:mod:`kinematic_driver`).

PhysX reads drive targets authored in USD during the simulation. Newton does not: with Newton a
motion acts only through the targets it had when the simulation started.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import carb
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from pxr import UsdPhysics

from .core import kinematic_mode as km
from .core import motion as mo


class MotionRuntime:
    """The step callback is registered on Play and removed on Stop, like the result recorder: a
    subscription made earlier does not survive a new stage."""

    def __init__(self):
        self._profiles: Optional[Dict[str, mo.Profile]] = None  # read on the first step after Play
        self._drivers: Optional[List] = None  # kinematic mechanisms, built on the first step after Stop
        self.reports: List[str] = []  # what the kinematic drivers did on the last start
        self._uid = None
        self._time = 0.0  # simulated time since Play (the recorder counts the same way)
        self._stopped = True  # Play after Stop starts at TIME = 0; Play after Pause resumes
        events = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._subs = [
            events.create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.PLAY), self._on_play,
                                                      name="gomjb.physics_designer.motion.play"),
            events.create_subscription_to_pop_by_type(int(omni.timeline.TimelineEventType.STOP), self._on_stop,
                                                      name="gomjb.physics_designer.motion.stop"),
        ]

    def destroy(self) -> None:
        self._unregister()
        self._subs = []

    def _unregister(self) -> None:
        if self._uid is not None:
            SimulationManager.deregister_callback(self._uid)
            self._uid = None

    def _on_play(self, *args) -> None:
        self._profiles = None  # motions may have changed while stopped or paused
        if self._stopped:
            self._time, self._stopped, self._drivers = 0.0, False, None
        if self._uid is None:
            self._uid = SimulationManager.register_callback(self._on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def _on_stop(self, *args) -> None:
        """Back to the motions' targets at TIME = 0, as the bodies return to their start pose."""
        self._unregister()
        self._profiles, self._drivers, self._stopped = None, None, True
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            for path, profile in mo.profiles(stage, skip=km.is_kinematic_root).items():
                mo.apply_targets(stage.GetPrimAtPath(path), profile, 0.0)

    def _start_drivers(self, stage) -> List:
        from .kinematic_driver import KinematicDriver

        drivers, self.reports = [], []
        for fingerprint, group in km.kinematic_groups(stage):  # copies of one mechanism share a table and a driver
            root = group[0]
            try:
                driver = KinematicDriver(stage, str(root.GetPath()), [str(r.GetPath()) for r in group], fingerprint)
            except Exception as exc:  # the mechanism then stays where it is: say why
                self.reports.append(f"{root.GetName()}: {exc}")
                carb.log_error(f"[Physics Designer] kinematic mechanism {root.GetPath()}: {exc}")
                continue
            drivers.append(driver)
            self.reports.append(driver.report())
            carb.log_info(f"[Physics Designer] {driver.report()}")
        return drivers

    def _on_pre_step(self, dt, *args) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        if self._profiles is None:
            # joints of kinematic mechanisms are disabled: the kinematic driver plays their motions
            self._profiles = {path: p for path, p in mo.profiles(stage, skip=km.is_kinematic_root).items()
                              if UsdPhysics.Joint(stage.GetPrimAtPath(path)).GetJointEnabledAttr().Get() is not False}
        if self._drivers is None:
            self._drivers = self._start_drivers(stage)
        self._time += dt  # targets for the end of the coming step
        for path, profile in self._profiles.items():
            joint = stage.GetPrimAtPath(path)
            if joint:
                mo.apply_targets(joint, profile, self._time)
        for driver in list(self._drivers):
            try:
                driver.apply(self._time - dt, dt)
            except Exception as exc:  # e.g. a co-simulation solve beyond the mechanism's range
                self._drivers.remove(driver)
                self.reports.append(f"{driver.root_path.rsplit('/', 1)[-1]} stopped at {self._time:.3f} s: {exc}")
                carb.log_error(f"[Physics Designer] kinematic mechanism {driver.root_path} stopped: {exc}")
