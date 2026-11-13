"""The "Kinematics" window: how a mechanism moves when simulated (physics / motion table /
co-simulation) and the exact solver (Exudyn) - static equilibrium, drive sweeps, dynamic
analysis - with the Physics Designer's result view (plot, summary, CSV)."""
from __future__ import annotations

import time
from typing import List, Optional

import carb
import carb.settings
import omni.ui as ui
import omni.usd
from pxr import Usd

from gomjb.physics_designer.commands import run_edit
from gomjb.physics_designer.core import units as un
from gomjb.physics_designer.core.stage_utils import prims_from_paths
from gomjb.physics_designer.ui.result_view import ResultView
from gomjb.physics_designer.ui.widgets import (ComboInput, FloatInput, IntInput, StringInput, Units, button_row, hint,
                                               section)
from gomjb.physics_designer.ui.window import UNITS_SETTING

from ..core import kinematic_mode as km

WINDOW_TITLE = "Kinematics"
REDUNDANCY = ["release", "bushings"]


class KinematicsWindow(ui.Window):
    def __init__(self, title: str = WINDOW_TITLE, **kwargs):
        super().__init__(title, width=440, height=720, **kwargs)
        self._status_model = ui.SimpleStringModel("Ready.")
        saved = carb.settings.get_settings().get(UNITS_SETTING)  # the Physics Designer's unit system
        self.units = Units(saved if saved in un.SYSTEMS else "MKS")
        self.results = ResultView(self.units, self._status)
        self._sweep_from = self._sweep_to = None
        self.units.add_listener(self._on_units)
        self.frame.set_build_fn(self._build)

    # -- helpers ---------------------------------------------------------------------------
    def _stage(self) -> Optional[Usd.Stage]:
        return omni.usd.get_context().get_stage()

    def _selection(self) -> List[Usd.Prim]:
        stage = self._stage()
        if stage is None:
            return []
        return prims_from_paths(stage, omni.usd.get_context().get_selection().get_selected_prim_paths())

    def _status(self, text: str, error: bool = False) -> None:
        self._status_model.set_value(text)
        (carb.log_error if error else carb.log_info)(f"[Kinematics] {text}")

    def set_unit_system(self, name: str) -> None:
        self.units.set_system(name)
        carb.settings.get_settings().set(UNITS_SETTING, name)
        us = self.units.system
        self._status(f"Units: {name} ({us.length}, {us.mass}, {us.force}, s, deg).")

    @property
    def recording(self):
        """The result on display (an exact solution)."""
        return self.results.result

    # -- layout ----------------------------------------------------------------------------
    def _build(self):
        with ui.VStack(spacing=4):
            self._unit_combo = ComboInput(
                "Unit system", list(un.SYSTEMS), default=list(un.SYSTEMS).index(self.units.system.name),
                tooltip="Units of every value in this window (time in s, angles in deg), shared with the "
                        "Physics Designer window. The stage keeps its own scale.",
                on_changed=self.set_unit_system)
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6, height=0):
                    self._build_mechanism()
                    self._build_kinematics()
                    self._build_solver()
                    with section("Results", collapsed=False):
                        with ui.VStack(spacing=3):
                            self.results.build()
            with ui.HStack(height=20):
                ui.StringField(model=self._status_model, read_only=True)

    def _build_mechanism(self):
        with section("Mechanism", collapsed=False):
            with ui.VStack(spacing=3):
                self._root = StringInput("Mechanism", "", "Root prim of the mechanism: its articulation root, "
                                         "or any prim that holds its bodies")
                button_row([("Use selection", self.use_selection)])

    def _build_kinematics(self):
        with section("Kinematics", collapsed=False):
            with ui.VStack(spacing=3):
                hint("How the mechanism moves when simulated. physics - PhysX solves its joints and drives. "
                     "table / cosim - the exact solver moves its bodies (kinematic, loops closed exactly) along "
                     "the joint motions and PhysX simulates only what touches them: a motion table is solved "
                     "along the drives' stroke (one stroke, range = joint limits), co-simulation solves every "
                     "physics step (any motions).")
                self._kin_mode = ComboInput(
                    "Kinematics", km.MODES,
                    tooltip="physics: joints simulated by PhysX.\n"
                            "table: motion table from the exact solver, interpolated (GPU-friendly, one stroke).\n"
                            "cosim: exact solver every physics step (CPU, any motions).")
                self._kin_tol = self.units.float("Table tolerance", km.TABLE_TOLERANCE, "length",
                                                 "Largest interpolation error of the motion table: points are "
                                                 "added where the kinematics bends until it is met")
                button_row([("Apply kinematics", self.apply_kinematics,
                             "Store the mode on the mechanism root: bodies kinematic, inner joints disabled "
                             "(undoable; 'physics' restores the model). A motion table is built here.")])

    def _build_solver(self):
        with section("Exact solver", collapsed=False):
            with ui.VStack(spacing=3):
                hint("Exact solver (Exudyn): every joint an exact constraint, loop closures included; driven axes "
                     "are prescribed motions whose multipliers are the actuator forces. Solve static equilibrium "
                     "at the drive targets, sweep drives along a stroke, or simulate the joint motions.")
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
                self._end_time = FloatInput("End time [s]", 2.0, "Dynamic analysis length", step=0.1)
                button_row([("Simulate", self.simulate,
                             "Dynamic analysis (generalized-alpha): joints with a motion follow it")])

    def _on_units(self) -> None:
        if self._sweep_from is None:  # not built yet
            return
        unit = f"{self.units.label('length')} | deg"
        self._sweep_from.unit_label.text = self._sweep_to.unit_label.text = unit

    # -- mechanism ---------------------------------------------------------------------------
    def _mechanism(self) -> Optional[str]:
        path = self._root.value.strip()
        stage = self._stage()
        if not path or stage is None or not stage.GetPrimAtPath(path):
            self._status("Set the mechanism (Use selection).", error=True)
            return None
        return path

    def use_selection(self):
        for prim in self._selection():
            root = km.mechanism_root_of(prim)
            if root is not None:
                self._root.model.set_value(str(root.GetPath()))
                self._status(f"Mechanism: {root.GetPath()} (kinematics: {km.get_mode(root)})")
                return
        self._status("Select a prim inside a mechanism (articulation).", error=True)

    # -- kinematics ---------------------------------------------------------------------------
    def apply_kinematics(self):
        path = self._mechanism()
        if path is None:
            return
        mode = self._kin_mode.value
        tol = self._kin_tol.si
        if mode in km.KINEMATIC_MODES:  # check what the exact solver needs before touching the model
            try:
                from ..core import kinematics as kn  # needs exudyn ([python.pipapi])

                model = kn.kinematic_model(self._stage(), path)
                if not model.motion_profiles():
                    raise ValueError("no driven joint has a motion (Include Motion): nothing would move")
            except Exception as exc:
                self._status(f"Kinematics: {exc}", error=True)
                return
        try:
            run_edit(lambda s: km.set_mode(s.GetPrimAtPath(path), mode, tol if mode == "table" else None),
                     modify_paths=km.edit_paths(self._stage().GetPrimAtPath(path)),
                     label=f"Kinematics: {mode}")
        except Exception as exc:
            self._status(f"Kinematics: {exc}", error=True)
            return
        what = {"physics": "PhysX simulates its joints", "cosim": "exact solver every physics step"}.get(mode)
        if mode == "table":  # build it now (into the cache Play reads): its size and error show here
            from ..cache import table_cache
            from ..core import kinematics as kn

            stage = self._stage()
            t0 = time.perf_counter()
            try:
                table, cached = kn.cached_table(stage, path, km.fingerprint(stage.GetPrimAtPath(path)), table_cache())
            except Exception as exc:  # the mode stays; Play reports the same error
                self._status(f"Kinematics 'table': motion table failed: {exc}", error=True)
                return
            what = (f"motion table {len(table.stroke)} points, error {table.error[0] * 1e3:.3g} mm / "
                    f"{table.error[1]:.3g} deg, {'from the cache' if cached else 'built'} in "
                    f"{time.perf_counter() - t0:.1f} s")
        self._status(f"{path}: kinematics '{mode}' - {what}.")

    # -- exact solver ---------------------------------------------------------------------------
    def _model(self, static: bool):
        path = self._mechanism()
        if path is None:
            return None
        from ..core.mbd import MbdModel  # needs exudyn ([python.pipapi])

        model = MbdModel(self._stage(), path, redundancy=self._redundancy.value)
        if static and model.redundancy == "release" and model.dof > 0 and not model.has_bushings:
            raise ValueError(f"{model.dof} free DOF with the drives locked - static equilibrium needs "
                             "a drive (stiffness > 0) on every free axis")
        return model

    def _solve(self, fn, static: bool = True) -> None:
        try:
            model = self._model(static)
            if model is None:
                return
            rec, msg = fn(model)
        except Exception as exc:  # solver / model errors go to the status line, not the console only
            self._status(f"Exact solver: {exc}", error=True)
            return
        self.results.show(rec, model.report())
        self._status(msg)

    def simulate(self):
        """Dynamic analysis to End time; joints with a motion follow it, other drives hold."""
        end = self._end_time.value
        if end <= 0:
            self._status("Exact solver: End time must be > 0.", error=True)
            return

        def run(model):
            n = len(model.motion_profiles())
            rec = model.simulate(end, step=min(1e-3, end / 100), sample=end / 200)
            return rec, f"Simulated {end:g} s, {n} motion(s)."

        self._solve(run, static=False)

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
            us = self.units.system  # From / To: lengths in the unit system, angles in deg
            to_si = (lambda v: un.to_si(v, us, "length")) if kinds == {"m"} else (lambda v: v)
            n = self._sweep_points.value
            a, b = to_si(self._sweep_from.value), to_si(self._sweep_to.value)
            for name in names:  # reach the start of the stroke; drives not swept hold their targets
                model.set_drive(name, a)
            model.solve_static(steps=10)
            rec = model.static_sweep(names, [a + (b - a) * k / (n - 1) for k in range(n)])
            return rec, f"Swept {', '.join(names)} over {n} points."

        self._solve(run)
