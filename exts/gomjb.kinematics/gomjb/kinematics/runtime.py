"""Kinematic mechanisms in the running simulation (Isaac Sim runtime): a participant of the Physics
Designer's motion runtime (``gomjb.physics_designer.motion_runtime``), so they share its time base.

On the first physics step after Play every group of copies of a mechanism in a kinematic mode
gets one :class:`~.driver.KinematicDriver`; before every step the drivers move their bodies.
"""
from __future__ import annotations

from typing import List, Optional

import carb

from .core import kinematic_mode as km


class KinematicRuntime:
    def __init__(self):
        self._drivers: Optional[List] = None
        self.reports: List[str] = []  # what the drivers did on the last start

    def skips(self, prim) -> bool:
        """The joints inside a kinematic mechanism are disabled: the driver plays their motions."""
        return km.is_kinematic_root(prim)

    def start(self, stage) -> None:
        from .driver import KinematicDriver

        self._drivers, self.reports = [], []
        for fingerprint, group in km.kinematic_groups(stage):  # copies of one mechanism share a table and a driver
            root = group[0]
            try:
                driver = KinematicDriver(stage, str(root.GetPath()), [str(r.GetPath()) for r in group], fingerprint)
            except Exception as exc:  # the mechanism then stays where it is: say why
                self.reports.append(f"{root.GetName()}: {exc}")
                carb.log_error(f"[Kinematics] kinematic mechanism {root.GetPath()}: {exc}")
                continue
            self._drivers.append(driver)
            self.reports.append(driver.report())
            carb.log_info(f"[Kinematics] {driver.report()}")

    def step(self, t: float, dt: float) -> None:
        for driver in list(self._drivers or []):
            try:
                driver.apply(t, dt)
            except Exception as exc:  # e.g. a co-simulation solve beyond the mechanism's range
                self._drivers.remove(driver)
                self.reports.append(f"{driver.root_path.rsplit('/', 1)[-1]} stopped at {t + dt:.3f} s: {exc}")
                carb.log_error(f"[Kinematics] kinematic mechanism {driver.root_path} stopped: {exc}")

    def stop(self) -> None:
        self._drivers = None

    @property
    def drivers(self) -> List:
        return list(self._drivers or [])
