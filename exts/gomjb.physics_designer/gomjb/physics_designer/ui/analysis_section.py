"""Analysis section: record an articulation while simulating, plot channels, export CSV."""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import omni.ui as ui

from ..core import physics as ph
from .widgets import ComboInput, StringInput, button_row, hint, section

PLOTS = ["DOF forces", "DOF positions", "Joint reaction |F|"]

# Categorical series colors (dark-mode steps, fixed order), validated on the dark plot surface #252525:
# lightness band, chroma, adjacent CVD dE >= 8.4, normal-vision dE >= 19.3, contrast >= 3:1.
SERIES_COLORS = [[0x39, 0x87, 0xE5], [0xD9, 0x59, 0x26], [0x19, 0x9E, 0x70], [0xC9, 0x85, 0x00],
                 [0xD5, 0x51, 0x81], [0x00, 0x83, 0x00], [0x90, 0x85, 0xE9], [0xE6, 0x67, 0x67]]


def _axes(xs: List[float], ys: List[List[float]]) -> dict:
    """Axis limits fitted to the data (5 % margin on y)."""
    vals = [v for y in ys for v in y if not math.isnan(v)]
    lo, hi = min(vals), max(vals)
    pad = 0.05 * (hi - lo) if hi > lo else max(1.0, abs(hi) * 0.05)
    return {"x_min": xs[0], "x_max": max(xs[-1], xs[0] + 1e-6), "y_min": lo - pad, "y_max": hi + pad}


class AnalysisSection:
    def __init__(self, window):
        self._w = window
        self._recorder = None
        self._plot = None
        self._plot_spec: dict = {}
        self._slots: Dict[str, int] = {}
        self._summary_model = ui.SimpleStringModel("")

    def build(self):
        self.frame = section("Analysis")
        with self.frame:
            with ui.VStack(spacing=3):
                hint("Start recording, then Play: every physics step stores joint positions, DOF (actuator) "
                     "forces and joint reactions in SI units. Loop-closing joints are not reported; Newton "
                     "reports positions only.")
                self._root = StringInput("Articulation", "")
                button_row([("Use selection", self.use_selection), ("Start recording", self.start),
                            ("Stop", self.stop)])
                self._kind = ComboInput("Plot", PLOTS, on_changed=lambda v: self.refresh_plot())
                self._filter = StringInput("Name filter", "", "Only channels containing this text, e.g. Actuator")
                ui.StringField(model=self._summary_model, multiline=True, read_only=True, height=60)
                self._plot_frame = ui.Frame(height=0)
                self._plot_frame.set_build_fn(self._build_plot)
                button_row([("Refresh plot", self.refresh_plot), ("Export CSV", self.export_csv)])
                self._csv = StringInput("CSV file", "", "Empty: Documents/PhysicsDesigner/<name>_<time>.csv")

    def _build_plot(self):
        """(Re)create the plot from ``_plot_spec``.  XYPlot is rebuilt rather than updated in place:
        it only auto-ranges on its first build, and set_plot_colors fails when the series count grows."""
        from isaacsim.gui.components import XYPlot

        self._plot = XYPlot("Result", x_label="time [s]", y_label="", plot_height=8, show_legend=True,
                            **self._plot_spec)

    # -- recording ---------------------------------------------------------------------
    def use_selection(self):
        for prim in self._w._selection():
            root = ph.articulation_root_of(prim)
            if root is not None:
                self._root.model.set_value(str(root.GetPath()))
                self._w._status(f"Analysis target: {root.GetPath()}")
                return
        self._w._status("Analysis: select a prim inside an articulation.", error=True)

    def start(self):
        path = self._root.value.strip()
        stage = self._w._stage()
        if not path or stage is None or not stage.GetPrimAtPath(path):
            self._w._status("Analysis: set the articulation (Use selection).", error=True)
            return
        from ..recording import ArticulationRecorder  # Isaac Sim runtime only

        self.stop(refresh=False)
        self._recorder = ArticulationRecorder(path)
        self._recorder.start()
        self._w._status(f"Recording {path}: press Play to simulate.")

    def stop(self, refresh: bool = True):
        if self._recorder is None:
            return
        self._recorder.stop()
        if refresh:
            n = len(self._recorder.recording)
            missing = f" (not available: {', '.join(self._recorder.unavailable)})" if self._recorder.unavailable else ""
            self._w._status(f"Recorded {n} samples{missing}.")
            self.refresh_plot()

    @property
    def recording(self):
        return self._recorder.recording if self._recorder is not None else None

    # -- plotting / export ---------------------------------------------------------------
    def series(self) -> Tuple[List[str], List[List[float]], List[str]]:
        """(names, y-series, units) of the selected plot kind and name filter."""
        rec = self.recording
        if rec is None or not len(rec):
            return [], [], []
        text = self._filter.value.strip()
        kind = self._kind.value
        names, ys, units = [], [], []
        if kind in ("DOF forces", "DOF positions"):
            prefix = "force:" if kind == "DOF forces" else "pos:"
            for ch in rec.names(prefix):
                if text in ch:
                    names.append(ch[len(prefix):])
                    ys.append(rec.channels[ch])
                    units.append(rec.units.get(ch, ""))
        else:
            links = sorted({ch.split(":")[1] for ch in rec.names("link_force:")})
            for link in links:
                if text not in link:
                    continue
                comps = [rec.channels[f"link_force:{link}:{c}"] for c in "xyz"]
                ys.append([math.sqrt(x * x + y * y + z * z) for x, y, z in zip(*comps)])
                names.append(link)
                units.append("N")
        return names, ys, units

    def _colors(self, names: List[str]) -> List[List[int]]:
        """A series keeps its color while it stays plotted (filters do not repaint survivors);
        slots are handed out in first-plotted order and recycled only when the set changes."""
        if any(n not in self._slots for n in names) and len(set(self._slots) | set(names)) > len(SERIES_COLORS):
            self._slots = {}
        for n in names:
            if n not in self._slots:
                self._slots[n] = len(self._slots)
        return [SERIES_COLORS[self._slots[n]] for n in names]

    def refresh_plot(self):
        names, ys, units = self.series()
        rec = self.recording
        if not names:
            self._summary_model.set_value("No data (record a simulation first).")
            return
        note = ""
        if len(names) > len(SERIES_COLORS):  # never cycle colors: narrow the selection instead
            note = f"Showing {len(SERIES_COLORS)} of {len(names)} series - narrow the name filter.\n"
            names, ys, units = (v[:len(SERIES_COLORS)] for v in (names, ys, units))
        self._plot_spec = dict(x_data=[list(rec.time)] * len(ys), y_data=[list(y) for y in ys], legends=names,
                               plot_colors=self._colors(names), **_axes(rec.time, ys))
        self._plot_frame.rebuild()
        lines = [note] if note else []
        for name, y, unit in zip(names, ys, units):
            vals = [v for v in y if not math.isnan(v)]
            if vals:
                lines.append(f"{name}: min {min(vals):.4g}  max {max(vals):.4g}  final {vals[-1]:.4g} {unit}")
        self._summary_model.set_value("\n".join(lines))

    def export_csv(self) -> Optional[str]:
        rec = self.recording
        if rec is None or not len(rec):
            self._w._status("Export: nothing recorded yet.", error=True)
            return None
        path = self._csv.value.strip()
        if not path:
            folder = os.path.join(os.path.expanduser("~"), "Documents", "PhysicsDesigner")
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"{rec.name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        rec.to_csv(path)
        self._w._status(f"Exported {len(rec)} samples, {len(rec.channels)} channels to {path}")
        return path
