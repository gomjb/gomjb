"""Analysis section: record an articulation while simulating, or solve the model exactly (Exudyn);
plot channels, export CSV."""
from __future__ import annotations

import math
import os
import textwrap
import time
from typing import Dict, List, Optional, Tuple

import omni.ui as ui

from ..core import physics as ph
from ..core import units as un
from .widgets import ComboInput, FloatInput, IntInput, StringInput, button_row, hint, section

PLOTS = ["DOF forces", "DOF positions", "Joint reaction |F|"]
REDUNDANCY = ["release", "bushings"]
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


class AnalysisSection:
    def __init__(self, window):
        self._w = window
        self._recorder = None
        self._result = None  # the Recording on display: a live recording or an exact solution
        self._report = ""
        self._plot = None
        self._plot_spec: dict = {}
        self._slots: Dict[str, int] = {}
        self._summary_model = ui.SimpleStringModel("")
        self._sweep_from = self._sweep_to = None
        window.units.add_listener(self._on_units)

    def build(self):
        self.frame = section("Analysis")
        with self.frame:
            with ui.VStack(spacing=3):
                hint("Record: start recording, then Play - every physics step stores joint positions, DOF "
                     "(actuator) forces and joint reactions. Loop-closing joints are not reported; Newton "
                     "reports positions only. Results show in the window's unit system.")
                self._root = StringInput("Mechanism", "", "Articulation root to record; for the exact "
                                         "solver any prim that holds the bodies")
                button_row([("Use selection", self.use_selection), ("Start recording", self.start),
                            ("Stop", self.stop)])
                hint("Exact solver (Exudyn): every joint an exact constraint, loop closures included; "
                     "driven axes are prescribed motions whose multipliers are the actuator forces. "
                     "Solve static equilibrium at the drive targets, or sweep drives along a stroke.")
                self._redundancy = ComboInput(
                    "Redundant", REDUNDANCY,
                    tooltip="release: dependent equations are released and listed (Adams / MotionSolve "
                            "practice) - totals exact, split over redundant paths arbitrary.\n"
                            "bushings: undriven joints become stiff bushings - split follows stiffness.")
                self._drive = StringInput("Sweep drives", "", "Driven joints moved together, comma separated; "
                                          "empty = all drives (e.g. both actuators of a lift)")
                tip = "Drive target: a length for linear drives, degrees for rotational ones"
                self._sweep_from = FloatInput("From", 0.0, tip, unit="")
                self._sweep_to = FloatInput("To", 0.1, tip, unit="")
                self._sweep_points = IntInput("Points", 11, min_value=2)
                self._on_units()
                button_row([("Solve static", self.solve_static), ("Sweep drive", self.sweep)])
                self._kind = ComboInput("Plot", PLOTS, on_changed=lambda v: self.refresh_plot())
                self._filter = StringInput("Name filter", "", "Only channels containing this text, e.g. Actuator")
                ui.StringField(model=self._summary_model, multiline=True, read_only=True, height=90)
                # XYPlot is wider than the window: clip it, or every row of the section widens and the
                # unit column of From / To ends up outside the window
                self._plot_frame = ui.Frame(height=0, horizontal_clipping=True)
                self._plot_frame.set_build_fn(self._build_plot)
                button_row([("Refresh plot", self.refresh_plot), ("Export CSV", self.export_csv)])
                self._csv = StringInput("CSV file", "", "Empty: Documents/PhysicsDesigner/<name>_<time>.csv")

    def _build_plot(self):
        """(Re)create the plot from ``_plot_spec``.  XYPlot is rebuilt rather than updated in place:
        it only auto-ranges on its first build, and set_plot_colors fails when the series count grows."""
        from isaacsim.gui.components import XYPlot

        self._plot = XYPlot("Result", y_label="", plot_height=8, show_legend=True, **self._plot_spec)

    def _mechanism(self) -> Optional[str]:
        path = self._root.value.strip()
        stage = self._w._stage()
        if not path or stage is None or not stage.GetPrimAtPath(path):
            self._w._status("Analysis: set the mechanism (Use selection).", error=True)
            return None
        return path

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
        path = self._mechanism()
        if path is None:
            return
        from ..recording import ArticulationRecorder  # Isaac Sim runtime only

        self.stop(refresh=False)
        self._recorder = ArticulationRecorder(path)
        self._recorder.start()
        self._result, self._report = self._recorder.recording, ""
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
        return self._result

    # -- exact solver --------------------------------------------------------------------
    def _model(self):
        path = self._mechanism()
        if path is None:
            return None
        from ..core.mbd import MbdModel  # needs exudyn ([python.pipapi])

        model = MbdModel(self._w._stage(), path, redundancy=self._redundancy.value)
        if model.redundancy == "release" and model.dof > 0:
            raise ValueError(f"{model.dof} free DOF with the drives locked - static equilibrium needs "
                             "a drive (stiffness > 0) on every free axis")
        return model

    def _solve(self, fn) -> None:
        try:
            model = self._model()
            if model is None:
                return
            rec, msg = fn(model)
        except Exception as exc:  # solver / model errors go to the status line, not the console only
            self._w._status(f"Exact solver: {exc}", error=True)
            return
        self.stop(refresh=False)
        self._result, self._report = rec, model.report()
        self.refresh_plot()
        self._w._status(msg)

    def solve_static(self):
        self._solve(lambda model: (model.static_solution(), f"Static equilibrium of {model.root_path} solved."))

    def sweep(self):
        def run(model):
            names = [n.strip() for n in self._drive.value.split(",") if n.strip()] or model.drive_names
            unknown = [n for n in names if n not in model.drive_names]
            if not names or unknown:
                raise ValueError(f"no driven joint {', '.join(unknown)} (drives: {', '.join(model.drive_names) or 'none'})")
            kinds = {model.drive_unit(name) for name in names}
            if len(kinds) != 1:
                raise ValueError("drives swept together must all be linear or all angular")
            us = self._w.units.system  # From / To: lengths in the unit system, angles in deg
            to_si = (lambda v: un.to_si(v, us, "length")) if kinds == {"m"} else (lambda v: v)
            n = self._sweep_points.value
            a, b = to_si(self._sweep_from.value), to_si(self._sweep_to.value)
            for name in names:  # reach the start of the stroke; drives not swept hold their targets
                model.set_drive(name, a)
            model.solve_static(steps=10)
            rec = model.static_sweep(names, [a + (b - a) * k / (n - 1) for k in range(n)])
            return rec, f"Swept {', '.join(names)} over {n} points."

        self._solve(run)

    # -- plotting / export ---------------------------------------------------------------
    def displayed(self):
        """The result in the window's unit system (the recording itself stays SI)."""
        return un.convert_recording(self._result, self._w.units.system) if self._result is not None else None

    def _on_units(self) -> None:
        if self._sweep_from is None:  # not built yet
            return
        unit = f"{self._w.units.label('length')} | deg"
        self._sweep_from.unit_label.text = self._sweep_to.unit_label.text = unit
        if self._result is not None:
            self.refresh_plot()

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

    def refresh_plot(self):
        """Plot and summary: the per-series results first, then the solver report wrapped below."""
        rec = self.displayed()
        names, ys, units = self.series(rec)
        report = [w for line in self._report.splitlines() for w in textwrap.wrap(line, SUMMARY_WIDTH, subsequent_indent="  ")]
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
            self._w._status("Export: no results yet.", error=True)
            return None
        path = self._csv.value.strip()
        if not path:
            folder = os.path.join(os.path.expanduser("~"), "Documents", "PhysicsDesigner")
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, f"{rec.name}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        rec.to_csv(path)
        self._w._status(f"Exported {len(rec)} samples, {len(rec.channels)} channels to {path}")
        return path
