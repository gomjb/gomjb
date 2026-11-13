"""Analysis section: record an articulation while simulating; plot channels, export CSV.

The exact solver (Exudyn) and kinematic mechanisms are the gomjb.kinematics extension's
"Kinematics" window, which shows its results in the same :class:`~.result_view.ResultView`."""
from __future__ import annotations

from typing import Optional

import omni.ui as ui
from pxr import UsdPhysics

from ..core import physics as ph
from .result_view import ResultView
from .widgets import StringInput, button_row, hint, section


class AnalysisSection:
    def __init__(self, window):
        self._w = window
        self._recorder = None
        self.results = ResultView(window.units, window._status)

    def build(self):
        self.frame = section("Analysis")
        with self.frame:
            with ui.VStack(spacing=3):
                hint("Record: start recording, then Play - every physics step stores joint positions, DOF "
                     "(actuator) forces and joint reactions. Loop-closing joints are not reported; Newton "
                     "reports positions only. Results show in the window's unit system.")
                self._root = StringInput("Mechanism", "", "Articulation root to record")
                button_row([("Use selection", self.use_selection), ("Start recording", self.start),
                            ("Stop", self.stop)])
                self.results.build()

    def _mechanism(self) -> Optional[str]:
        path = self._root.value.strip()
        stage = self._w._stage()
        if not path or stage is None or not stage.GetPrimAtPath(path):
            self._w._status("Analysis: set the mechanism (Use selection).", error=True)
            return None
        return path

    def use_selection(self):
        for prim in self._w._selection():
            root = ph.articulation_root_of(prim)
            if root is not None:
                self._root.model.set_value(str(root.GetPath()))
                self._w._status(f"Analysis target: {root.GetPath()}")
                return
        self._w._status("Analysis: select a prim inside an articulation.", error=True)

    def start(self):
        path = self._mechanism()
        if path is None:
            return
        if not self._w._stage().GetPrimAtPath(path).HasAPI(UsdPhysics.ArticulationRootAPI):
            self._w._status(f"Recording: {path} is no articulation root (a kinematic mechanism has none: "
                            "its motion is the exact solver's).", error=True)
            return
        from ..recording import ArticulationRecorder  # Isaac Sim runtime only

        self.stop(refresh=False)
        self._recorder = ArticulationRecorder(path)
        self._recorder.start()
        self.results.result, self.results.report = self._recorder.recording, ""
        self._w._status(f"Recording {path}: press Play to simulate.")

    def stop(self, refresh: bool = True):
        if self._recorder is None:
            return
        self._recorder.stop()
        if refresh:
            n = len(self._recorder.recording)
            missing = f" (not available: {', '.join(self._recorder.unavailable)})" if self._recorder.unavailable else ""
            self._w._status(f"Recorded {n} samples{missing}.")
            self.results.refresh()

    @property
    def recording(self):
        return self.results.result
