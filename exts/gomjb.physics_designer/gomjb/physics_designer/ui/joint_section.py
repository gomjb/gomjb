"""Joints section, RecurDyn style (see core.joint_builder): choose the joint type, then click base
body -> action body -> point (-> direction) in the viewport; the joint is created at the last pick."""
from __future__ import annotations

from typing import Optional

import omni.ui as ui
from pxr import Gf, Usd, UsdPhysics

from ..core import joint_builder as jb
from ..core import joints as jt
from ..core import motion as mo
from ..core import snapping as sn
from ..core import transform as tf
from ..core import units as un
from .widgets import BoolInput, ComboInput, FloatInput, StringInput, button_row, hint, section

PARENTS = ["<root>/Joints", "base body", "action body"]
MOTION_LABELS = {"Displacement (time)": "displacement", "Velocity (time)": "velocity",
                 "Acceleration (time)": "acceleration"}
_SELECTED = {"Button:selected": {"background_color": 0xFF7A5A2A}}


class JointSection:
    def __init__(self, window):
        self._w = window
        self.kind = "Revolute"
        self.method = jb.METHODS[self.kind][0]  # the creation method of the next command
        self.picker: Optional[jb.JointPicker] = None
        self._viewport = None
        self._type_buttons = {}
        self._prompt_model = ui.SimpleStringModel("Choose a joint type to start.")
        self._picks_model = ui.SimpleStringModel("")

    # -- layout ---------------------------------------------------------------------------
    def build(self):
        u = self._w.units
        with section("Joints"):
            with ui.VStack(spacing=3):
                hint("RecurDyn style: choose the joint type, then click in the viewport - the BASE body, the "
                     "ACTION body, the joint POINT (and a second point for the DIRECTION). The joint Z axis is "
                     "the rotation / sliding axis; without a direction it is the working-plane normal. A click "
                     "on anything that is not a rigid body picks the ground.")
                with ui.HStack(height=26, spacing=3):
                    for kind in jb.JOINT_TYPES:
                        self._type_buttons[kind] = ui.Button(kind, clicked_fn=lambda k=kind: self.start(k),
                                                             style=_SELECTED)
                self._method_frame = ui.Frame(height=0)
                self._method_frame.set_build_fn(self._build_method)
                self._plane = ComboInput("Working plane", list(jb.WORKING_PLANES),
                                         tooltip="Its normal is the joint axis when the method has no direction")
                self._snap = ComboInput("Point snap", list(jb.SNAP_MODES),
                                        tooltip="Key point: the nearest marker (anchor, e.g. a link pin) of the "
                                                "body, or corner / edge middle / face center of the clicked "
                                                "geometry. Surface: the clicked point itself.")
                ui.StringField(model=self._prompt_model, read_only=True, height=22,
                               style={"color": 0xFF6FD3FF})
                ui.StringField(model=self._picks_model, multiline=True, read_only=True, height=58)
                button_row([("+X", lambda: self.direction((1, 0, 0))), ("+Y", lambda: self.direction((0, 1, 0))),
                            ("+Z", lambda: self.direction((0, 0, 1))),
                            ("Marker Z", self.marker_direction, "Z axis of the marker the point snapped to")])
                button_row([("Pick from selection", self.use_selection,
                             "Feed the selected prim (body steps) or anchor (point steps) instead of a click"),
                            ("Cancel", self.cancel)])
                ui.Separator(height=6)
                hint("Properties of the next joint (RecurDyn joint page). Limits and drive follow the joint type: "
                     "deg for revolute, lengths for translational.")
                self._limit = BoolInput("Limits", False)
                self._lower = u.float("Lower", -90.0, self._q("length", "angle"), step=1.0)
                self._upper = u.float("Upper", 90.0, self._q("length", "angle"), step=1.0)
                self._drive = BoolInput("Drive", False)
                self._drive_type = ComboInput("Drive type", ["force", "acceleration"])
                self._stiff = u.float("Stiffness", 1000.0, self._q("linear_stiffness", "angular_stiffness"), step=10.0)
                self._damp = u.float("Damping", 100.0, self._q("linear_damping", "angular_damping"), step=1.0)
                self._target = u.float("Target position", 0.0, self._q("length", "angle"))
                self._tvel = u.float("Target velocity", 0.0, self._q("velocity", "angular_velocity"))
                self._maxf = u.float("Max force (0=inf)", 0.0, self._q("force", "torque"))
                self._break = u.float("Break force (0=never)", 0.0, "force")
                self._break_t = u.float("Break torque (0=never)", 0.0, "torque")
                self._collide = BoolInput("Collide bodies", False)
                self._excl = BoolInput("Exclude from artic.", False, "A loop-closing joint of an articulation")
                self._parent = ComboInput("Joint prim under", PARENTS)
                ui.Separator(height=6)
                hint("Motion of the selected revolute / translational joints (RecurDyn 'Include Motion'): the "
                     "joint's relative displacement follows an expression of TIME - deg for revolute joints, "
                     "the unit system's length for translational ones. STEP(TIME, 0, 0, 1, 90) goes 0 -> 90 in "
                     "1 s. Functions: STEP SIN COS TAN ASIN ACOS ATAN ATAN2 ABS SQRT EXP LOG MIN MAX SIGN MOD, PI.")
                self._motion_on = BoolInput("Include Motion", True, "Unchecked: Apply removes the motion")
                self._motion_type = ComboInput("Motion", list(MOTION_LABELS))
                self._motion_expr = StringInput("Expression", "STEP(TIME, 0, 0, 1, 90)")
                self._motion_x0 = FloatInput("Initial displacement", 0.0, "Velocity and acceleration motions")
                self._motion_v0 = FloatInput("Initial velocity", 0.0, "Acceleration motions")
                button_row([("Load motion", self.load_motion, "From the selected joint"),
                            ("Apply motion", self.apply_motion, "To the selected joints")])
                ui.Separator(height=6)
                button_row([("Realign selected joints", self.realign_selected,
                             "Move the action-side frame back onto the base-side frame")])
        self._refresh()

    def _build_method(self):
        methods = list(jb.METHODS[self.kind])
        self._method = ComboInput("Creation method", methods, default=methods.index(self.method),
                                  on_changed=lambda m: self.start(self.kind, m))

    def _q(self, linear: str, angular: str):
        return lambda: linear if self.kind == "Translational" else angular

    # -- the joint command ---------------------------------------------------------------
    def start(self, kind: str, method: Optional[str] = None) -> None:
        """Begin a joint command (RecurDyn: click the joint icon); clicks in the viewport feed it."""
        changed = kind != self.kind
        self.kind = kind
        self.method = method or (jb.METHODS[kind][0] if changed else self.method)
        self.picker = jb.JointPicker(kind, self.method, self._plane.value)
        if changed:
            self._w.units.refresh_labels()  # limits / drive switch between deg and lengths
            self._method_frame.rebuild()  # this type's creation methods
        self._start_viewport()
        self._refresh()

    def _start_viewport(self) -> None:
        if self._viewport is None:
            try:
                from .viewport_pick import ViewportPicker  # Kit viewport only
            except ImportError:
                return  # no viewport (tests): picks come from pick() / use_selection()
            self._viewport = ViewportPicker(self.on_viewport_pick)
        try:
            self._viewport.start()
        except RuntimeError as exc:  # no viewport (e.g. a headless app): the selection still works
            self._w._status(f"Joint: {exc}; use 'Pick from selection'.")

    def cancel(self) -> None:
        self.picker = None
        if self._viewport is not None:
            self._viewport.stop()
        self._refresh()

    def on_viewport_pick(self, path: str, world) -> None:
        if self.picker is None:
            return
        stage = self._w._stage()
        prim = stage.GetPrimAtPath(path) if stage is not None and path else None
        prim = prim if prim is not None and prim.IsValid() else None
        if self.picker.step in ("point", "direction"):
            if world is None:
                self._w._status("Joint: click on geometry to pick a point.", error=True)
                return
            point, anchor = jb.snap(prim, world, self._snap.value) if prim is not None else (Gf.Vec3d(*world), None)
            self.pick(prim, point, anchor)
        else:
            self.pick(prim)

    def pick(self, prim: Optional[Usd.Prim], point=None, anchor: Optional[Usd.Prim] = None) -> None:
        """One pick of the running command (a viewport click, or the selection)."""
        if self.picker is None:
            self._w._status("Joint: choose a joint type first.", error=True)
            return
        try:
            self.picker.pick(prim, point, anchor)
        except ValueError as exc:
            self._w._status(f"Joint: {exc}", error=True)
        self._advance()

    def direction(self, axis) -> None:
        if self.picker is None or self.picker.step != "direction":
            self._w._status("Joint: no direction is being picked.", error=True)
            return
        self.picker.set_direction(axis)
        self._advance()

    def marker_direction(self) -> None:
        axis = self.picker.marker_axis() if self.picker is not None else None
        if axis is None:
            self._w._status("Joint: the point did not snap to a marker.", error=True)
            return
        self.direction(axis)

    def use_selection(self) -> None:
        if self.picker is None:
            self._w._status("Joint: choose a joint type first.", error=True)
            return
        prims = self._w._selection()
        prim = prims[-1] if prims else None
        if self.picker.step in ("point", "direction"):
            if prim is None:
                self._w._status("Joint: select an anchor or a prim for the point.", error=True)
                return
            anchor = prim if sn.is_anchor(prim) else None
            self.pick(prim, tf.world_position(prim), anchor)
        else:
            self.pick(prim)

    def _advance(self) -> None:
        if self.picker is not None and self.picker.step is None:
            self._create()
        self._refresh()

    def _create(self) -> None:
        picker, kind = self.picker, self.kind
        picker.working_plane = self._plane.value
        parent = self._parent.value

        def op(stage, prims):
            body = picker.base if parent == "base body" else picker.action if parent == "action body" else None
            return picker.create(stage, self.properties(stage), body.GetPath() if body is not None else None)

        joint = self._w._run(f"Create {kind} joint", op, modify=lambda p: [], select_result=True)
        self.cancel()
        if joint is not None:
            self._prompt_model.set_value(f"Created {joint.GetPath()}. Choose a joint type for the next one.")

    def properties(self, stage: Usd.Stage) -> jt.JointSpec:
        """Limits, drive, break and collision settings of the next joint, in stage units."""
        spec = jt.JointSpec(collision_enabled=self._collide.value, exclude_from_articulation=self._excl.value,
                            break_force=self._break.core(stage) or None,
                            break_torque=self._break_t.core(stage) or None)
        lower, upper = self._lower.core(stage), self._upper.core(stage)
        if self._limit.value:
            if self.kind in ("Revolute", "Translational"):
                spec.lower, spec.upper = lower, upper
            elif self.kind == "Spherical":
                spec.cone0 = spec.cone1 = abs(upper)
        if self._drive.value and self.kind in ("Revolute", "Translational"):
            spec.drive = jt.DriveSpec(self._drive_type.value, self._stiff.core(stage), self._damp.core(stage),
                                      self._maxf.core(stage) or None, self._target.core(stage),
                                      self._tvel.core(stage))
        return spec

    # -- motion -------------------------------------------------------------------------
    def apply_motion(self) -> None:
        include = self._motion_on.value
        motion = mo.Motion(MOTION_LABELS[self._motion_type.value], self._motion_expr.value.strip(),
                           self._w.units.system.name, self._motion_x0.value, self._motion_v0.value)

        def op(stage, prims):
            joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
            if not joints:
                raise ValueError("select revolute / translational joints")
            for j in joints:
                if include:
                    mo.set_motion(j, motion)
                else:
                    mo.clear_motion(j)

        self._w._run("Apply motion" if include else "Remove motion", op, min_sel=1)

    def load_motion(self) -> None:
        joint = next((p for p in self._w._selection() if p.IsA(UsdPhysics.Joint)), None)
        if joint is None:
            self._w._status("Motion: select a joint.", error=True)
            return
        motion = mo.get_motion(joint)
        self._motion_on.model.set_value(motion is not None)
        if motion is None:
            self._w._status(f"{joint.GetName()} has no motion.")
            return
        labels = list(MOTION_LABELS)
        self._motion_type.widget.model.get_item_value_model().set_value(
            labels.index(next(k for k, v in MOTION_LABELS.items() if v == motion.type)))
        self._motion_expr.model.set_value(motion.expression)
        self._motion_x0.value, self._motion_v0.value = motion.initial_displacement, motion.initial_velocity
        note = "" if motion.units == self._w.units.system.name else \
            f" Its lengths are in {motion.units}: switch the unit system to {motion.units} to edit it."
        self._w._status(f"Loaded the motion of {joint.GetName()}.{note}")

    def realign_selected(self) -> None:
        def op(stage, prims):
            joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
            if not joints:
                raise ValueError("select joint prims")
            for j in joints:
                jt.realign_joint(j, keep="body0")

        self._w._run("Realign joints", op, min_sel=1)

    # -- display --------------------------------------------------------------------------
    def _refresh(self) -> None:
        for kind, button in self._type_buttons.items():
            button.selected = self.picker is not None and kind == self.kind
        if self.picker is None:
            if not self._prompt_model.as_string.startswith("Created"):
                self._prompt_model.set_value("Choose a joint type to start.")
            self._picks_model.set_value("")
            return
        self._prompt_model.set_value(self.picker.prompt())
        picks, lines = self.picker.picks, []
        for step in ("base", "action"):  # picked, or inferred from the point ("Point" methods)
            if step in picks:
                lines.append(f"{step.capitalize()}: {picks[step].GetName() if picks[step] is not None else 'ground'}")
        if "point" in picks:
            u, stage = self._w.units, self._w._stage()
            xyz = ", ".join(f"{un.from_core(v, u.system, 'length', stage):.4g}" for v in picks["point"])
            at = f" at {self.picker.anchor.GetName()}" if self.picker.anchor is not None else ""
            lines.append(f"Point: ({xyz}) {u.label('length')}{at}")
        if "direction" in picks:
            lines.append("Direction: (" + ", ".join(f"{v:.3f}" for v in picks["direction"]) + ")")
        self._picks_model.set_value("\n".join(lines))
