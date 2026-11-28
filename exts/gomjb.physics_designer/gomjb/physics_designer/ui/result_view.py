"""Result view: plot and summary of a :class:`~..core.results.Recording` - a live PhysX recording,
or another tool's solution (the gomjb.kinematics exact solver) - and CSV export, in the window's
unit system (the recording itself stays SI)."""
from __future__ import annotations

import math
import os
import textwrap
import time
from typing import Callable, Dict, List, Optional, Tuple

import omni.ui as ui

from ..core import units as un
from .widgets import ComboInput, StringInput, Units, button_row

PLOTS = ["DOF forces", "DOF positions", "Joint reaction |F|"]
# PhysX records the incoming joint force per link, the exact solver the reaction per joint
REACTION_PREFIXES = ("link_force:", "joint_force:")
SUMMARY_WIDTH = 68  # characters per summary line (the field does not wrap)

# Categorical series colors (dark-mode steps, fixed order), validated on the dark plot surface #252525:
# lightness band, chroma, adjacent CVD dE >= 8.4, normal-vision dE >= 19.3, contrast >= 3:1.
SERIES_COLORS = [[0x39, 0x87, 0xE5], [0xD9, 0x59, 0x26], [0x19, 0x9E, 0x70], [0xC9, 0x85, 0x00],
                 [0xD5, 0x51, 0x81], [0x00, 0x83, 0x00], [0x90, 0x85, 0xE9], [0xE6, 0x67, 0x67]]


def _axes(xs: List[float], ys: List[List[float]]) -> dict:
    """Axis limits fitted to the data (5 % margin on y)."""
    vals = [v for y in ys for v in y if not math.isnan(v)]
    lo, hi = min(vals), max(vals)
    pad = 0.05 * (hi - lo) if hi > lo else max(1.0, abs(hi) * 0.05)
    return {"x_min": min(xs), "x_max": max(max(xs), min(xs) + 1e-6), "y_min": lo - pad, "y_max": hi + pad}


class ResultView:
    """``status(text, error=False)`` reports to the owning window's status line."""

    def __init__(self, units: Units, status: Callable[..., None]):
        self._units = units
        self._status = status
        self.result = None  # the Recording on display
        self.report = ""  # text shown below the per-series summary (e.g. a solver report)
        self._plot = None
        self._plot_spec: dict = {}
        self._slots: Dict[str, int] = {}
        self._summary_model = ui.SimpleStringModel("")
        self._built = False
        units.add_listener(self._on_units)

    def build(self) -> None:
        self._kind = ComboInput("Plot", PLOTS, on_changed=lambda v: self.refresh())
        self._filter = StringInput("Name filter", "", "Only channels containing this text, e.g. Actuator")
        ui.StringField(model=self._summary_model, multiline=True, read_only=True, height=90)
        # XYPlot is wider than the window: clip it, or every row of the section widens and the unit
        # columns of the inputs above end up outside the window
        self._plot_frame = ui.Frame(height=0, horizontal_clipping=True)
        self._plot_frame.set_build_fn(self._build_plot)
        button_row([("Refresh plot", self.refresh), ("Export CSV", self.export_csv)])
        self._csv = StringInput("CSV file", "", "Empty: Documents/PhysicsDesigner/<name>_<time>.csv")
        self._built = True

    def _build_plot(self):
        """(Re)create the plot from ``_plot_spec``.  XYPlot is rebuilt rather than updated in place:
        it only auto-ranges on its first build, and set_plot_colors fails when the series count grows."""
        from isaacsim.gui.components import XYPlot

        self._plot = XYPlot("Result", y_label="", plot_height=8, show_legend=True, **self._plot_spec)

    def show(self, rec, report: str = "") -> None:
        self.result, self.report = rec, report
        self.refresh()

    def displayed(self):
        """The result in the window's unit system."""
        return un.convert_recording(self.result, self._units.system) if self.result is not None else None

    def _on_units(self) -> None:
        if self._built and self.result is not None:
            self.refresh()

    def series(self, rec) -> Tuple[List[str], List[List[float]], List[str]]:
        """(names, y-series, units) of the selected plot kind and name filter."""
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
            for prefix in REACTION_PREFIXES:
                for item in sorted({ch.split(":")[1] for ch in rec.names(prefix)}):
                    if text not in item:
                        continue
                    comps = [rec.channels[f"{prefix}{item}:{c}"] for c in "xyz"]
                    ys.append([math.sqrt(x * x + y * y + z * z) for x, y, z in zip(*comps)])
                    names.append(item)
                    units.append(rec.units.get(f"{prefix}{item}:x", ""))
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

    def refresh(self) -> None:
        """Plot and summary: the per-series results first, then the report wrapped below."""
        rec = self.displayed()
        names, ys, units = self.series(rec)
        report = [w for line in self.report.splitlines() for w in textwrap.wrap(line, SUMMARY_WIDTH, subsequent_indent="  ")]
        if not names:
            self._summary_model.set_value("\n".join(["No data (record a simulation or run the exact solver)."] + report))
            return
        lines = []
        if len(names) > len(SERIES_COLORS):  # never cycle colors: narrow the selection instead
            lines.append(f"Showing {len(SERIES_COLORS)} of {len(names)} series - narrow the name filter.")
            names, ys, units = (v[:len(SERIES_COLORS)] for v in (names, ys, units))
        # XYPlot interpolates between consecutive x values and assumes they increase: a sweep can run downwards
        order = sorted(range(len(rec.time)), key=rec.time.__getitem__)
        xs = [rec.time[i] for i in order]
        self._plot_spec = dict(x_data=[xs] * len(ys), y_data=[[y[i] for i in order] for y in ys], legends=names,
                               plot_colors=self._colors(names), x_label=rec.x_label, **_axes(xs, ys))
        self._plot_frame.rebuild()
        for name, y, unit in zip(names, ys, units):
            vals = [v for v in y if not math.isnan(v)]
            if vals:
                lines.append(f"{name}: min {min(vals):.4g}  max {max(vals):.4g}  final {vals[-1]:.4g} {unit}")
        self._summary_model.set_value("\n".join(lines + report))

    def export_csv(self) -> Optional[str]:
        """CSV in the window's unit system (each column header names its unit)."""
        rec = self.displayed()
        if rec is None or not len(rec):
            self._status("Export: no results yet.", error=True)
            return None
        path = self._csv.value.strip()
        if not path:
            folder = os.path.join(os.path.expanduser("~"), "Documents", "PhysicsDesigner")
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"{rec.name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        rec.to_csv(path)
        self._status(f"Exported {len(rec)} samples, {len(rec.channels)} channels to {path}")
        return path
