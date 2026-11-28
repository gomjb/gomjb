"""Joint motions in the running simulation (Isaac Sim runtime): before every physics step the drive
targets of each simulated joint with a motion (core.motion) are set to the motion at the end of
the step.

Other extensions take part in the same step through :meth:`MotionRuntime.add_participant`, on the
same time base (simulated time since Play): gomjb.kinematics moves its kinematic mechanisms this
way. A participant is an object with

* ``skips(prim) -> bool`` — prims whose subtree it moves itself (their motions are not played as
  drive targets, and the scan for motions does not enter them);
* ``start(stage)`` — on the first physics step after Play from Stop;
* ``step(t, dt)`` — before every physics step, from time ``t`` to ``t + dt``;
* ``stop()`` — on Stop.

PhysX reads drive targets authored in USD during the simulation. Newton does not: with Newton a
motion acts only through the targets it had when the simulation started.
"""
from __future__ import annotations

from typing import Dict, List, Optional

import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from pxr import UsdPhysics

from .core import motion as mo


class MotionRuntime:
    """The step callback is registered on Play and removed on Stop, like the result recorder: a
    subscription made earlier does not survive a new stage."""

    def __init__(self):
        self._profiles: Optional[Dict[str, mo.Profile]] = None  # read on the first step after Play
        self._participants: List = []
        self._started = False  # participants started since the last Stop
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

    def add_participant(self, participant) -> None:
        self._participants.append(participant)

    def remove_participant(self, participant) -> None:
        if participant in self._participants:
            self._participants.remove(participant)

    def _skips(self, prim) -> bool:
        return any(p.skips(prim) for p in self._participants)

    def _unregister(self) -> None:
        if self._uid is not None:
            SimulationManager.deregister_callback(self._uid)
            self._uid = None

    def _on_play(self, *args) -> None:
        self._profiles = None  # motions may have changed while stopped or paused
        if self._stopped:
            self._time, self._stopped, self._started = 0.0, False, False
        if self._uid is None:
            self._uid = SimulationManager.register_callback(self._on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)

    def _on_stop(self, *args) -> None:
        """Back to the motions' targets at TIME = 0, as the bodies return to their start pose."""
        self._unregister()
        self._profiles, self._stopped, self._started = None, True, False
        for p in self._participants:
            p.stop()
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            for path, profile in mo.profiles(stage, skip=self._skips).items():
                mo.apply_targets(stage.GetPrimAtPath(path), profile, 0.0)

    def _on_pre_step(self, dt, *args) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        if self._profiles is None:  # a disabled joint is not simulated: its motion has no drive to set
            self._profiles = {path: p for path, p in mo.profiles(stage, skip=self._skips).items()
                              if UsdPhysics.Joint(stage.GetPrimAtPath(path)).GetJointEnabledAttr().Get() is not False}
        if not self._started:
            for p in self._participants:
                p.start(stage)
            self._started = True
        t0 = self._time
        self._time += dt  # targets for the end of the coming step
        for path, profile in self._profiles.items():
            joint = stage.GetPrimAtPath(path)
            if joint:
                mo.apply_targets(joint, profile, self._time)
        for p in self._participants:
            p.step(t0, dt)
