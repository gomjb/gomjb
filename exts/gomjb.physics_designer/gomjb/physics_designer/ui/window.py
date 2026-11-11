"""Main "Physics Designer" window."""
from __future__ import annotations

import inspect
from typing import Callable, Dict, List, Optional, Sequence

import carb
import carb.settings
import omni.ui as ui
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from ..commands import run_edit
from ..core import joints as jt
from ..core import math_utils as mu
from ..core import mechanisms as me
from ..core import physics as ph
from ..core import snapping as sn
from ..core import transform as tf
from ..core import units as un
from ..core import validation as va
from ..core.stage_utils import prims_from_paths
from .analysis_section import AnalysisSection
from .live_snap import LiveSnap
from .modeler_section import ModelerSection
from .widgets import (BoolInput, ComboInput, FloatInput, IntInput, StringInput, UnitFloat, Units, UnitVec3,
                      Vec3Input, button_row, hint, section)

WINDOW_TITLE = "Physics Designer"
UNITS_SETTING = "/persistent/exts/gomjb.physics_designer/unitSystem"
_LINEAR_JOINTS = ("prismatic", "distance")

_AXIS_ITEMS = ["X", "Y", "Z", "-X", "-Y", "-Z"]
_FACE_ITEMS = ["+Z", "-Z", "+X", "-X", "+Y", "-Y"]
_JOINT_LOCATIONS = {
    "Body1 origin": "body1",
    "Body0 origin": "body0",
    "Midpoint": "midpoint",
    "Body1 bbox center": "body1_bbox",
    "Body0 bbox center": "body0_bbox",
    "Selected anchor": "anchor",
    "Custom (world)": "custom",
}
_JOINT_AXES = ["Z", "Y", "X", "-Z", "-Y", "-X", "Anchor frame", "Body0 local X", "Body0 local Y",
               "Body0 local Z", "Custom (world)"]
_ANCHOR_LOCATIONS = ["Pivot", "BBox center", "+X face", "-X face", "+Y face", "-Y face", "+Z face", "-Z face",
                     "Custom (world)"]


class PhysicsDesignerWindow(ui.Window):
    def __init__(self, title: str = WINDOW_TITLE, **kwargs):
        super().__init__(title, width=440, height=820, **kwargs)
        self._live_snap = LiveSnap(self._live_snap_params)
        self._status_model = ui.SimpleStringModel("Ready.")
        self._selection_model = ui.SimpleStringModel("")
        self._validation_model = ui.SimpleStringModel("Press 'Validate stage' to check the physics setup.")
        self._issues: List[va.Issue] = []
        self._template_inputs: Dict[str, object] = {}
        saved = carb.settings.get_settings().get(UNITS_SETTING)
        self.units = Units(saved if saved in un.SYSTEMS else "MKS")
        self.modeler = ModelerSection(self)
        self.analysis = AnalysisSection(self)
        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="gomjb.physics_designer.window"
        )
        self.frame.set_build_fn(self._build)
        self._refresh_selection()

    def destroy(self):
        self._live_snap.stop()
        self.analysis.stop(refresh=False)
        self._stage_sub = None
        super().destroy()

    # =================================================================================
    # Helpers
    # =================================================================================
    def _stage(self) -> Optional[Usd.Stage]:
        return omni.usd.get_context().get_stage()

    def _selection(self) -> List[Usd.Prim]:
        stage = self._stage()
        if stage is None:
            return []
        return prims_from_paths(stage, omni.usd.get_context().get_selection().get_selected_prim_paths())

    def _status(self, text: str, error: bool = False) -> None:
        self._status_model.set_value(text)
        (carb.log_error if error else carb.log_info)(f"[Physics Designer] {text}")

    def _run(self, label: str, fn: Callable[[Usd.Stage, List[Usd.Prim]], object], min_sel: int = 0,
             modify: Optional[Callable[[List[Usd.Prim]], Sequence[str]]] = None, select_result: bool = False):
        """Run an undoable edit on the current selection and report the outcome."""
        stage = self._stage()
        if stage is None:
            self._status("No stage open.", error=True)
            return None
        prims = self._selection()
        if len(prims) < min_sel:
            self._status(f"{label}: select at least {min_sel} prim(s).", error=True)
            return None
        paths = list(modify(prims)) if modify else [str(p.GetPath()) for p in prims]
        try:
            result = run_edit(lambda s: fn(s, prims), modify_paths=paths, label=f"Physics Designer: {label}")
        except Exception as exc:  # report every failure in the status line
            self._status(f"{label} failed: {exc}", error=True)
            return None
        if select_result and result:
            created = result if isinstance(result, (list, tuple)) else [result]
            omni.usd.get_context().get_selection().set_selected_prim_paths(
                [str(p.GetPath()) if isinstance(p, Usd.Prim) else str(p) for p in created], True
            )
        self._status(f"{label}: done.")
        return result

    def _on_stage_event(self, event) -> None:
        etype = event.type
        if etype == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._refresh_selection()
        elif etype == int(omni.usd.StageEventType.OPENED):
            if self._live_snap.active:
                self._live_snap.start()
            self._refresh_selection()
        elif etype == int(omni.usd.StageEventType.CLOSING):
            self._live_snap.stop()

    def _refresh_selection(self) -> None:
        prims = self._selection()
        if not prims:
            self._selection_model.set_value("Selection: (none)")
            return
        names = []
        for i, p in enumerate(prims[:4]):
            tag = " [anchor]" if sn.is_anchor(p) else (" [rigid]" if p.HasAPI(UsdPhysics.RigidBodyAPI) else "")
            names.append(f"{i}: {p.GetName()}{tag}")
        more = f"  (+{len(prims) - 4})" if len(prims) > 4 else ""
        self._selection_model.set_value("Selection  " + "   ".join(names) + more)

    # =================================================================================
    # Layout
    # =================================================================================
    def set_unit_system(self, name: str) -> None:
        """Show and read every value in ``name`` (MKS, MMKS, ...); the stage itself is unchanged."""
        self.units.set_system(name)
        carb.settings.get_settings().set(UNITS_SETTING, name)
        us = self.units.system
        self._status(f"Units: {name} ({us.length}, {us.mass}, {us.force}, s, deg).")

    def _build(self):
        with ui.VStack(spacing=4):
            self._unit_combo = ComboInput(
                "Unit system", list(un.SYSTEMS), default=list(un.SYSTEMS).index(self.units.system.name),
                tooltip="Units of every value in this window (time in s, angles in deg). The stage keeps its "
                        "own scale (metersPerUnit); values are converted.",
                on_changed=self.set_unit_system)
            with ui.HStack(height=20):
                ui.StringField(model=self._selection_model, read_only=True)
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                with ui.VStack(spacing=6, height=0):
                    self.modeler.build()
                    self._build_move()
                    self._build_snap()
                    self._build_anchor()
                    self._build_joint()
                    self._build_physics()
                    self._build_templates()
                    self._build_validate()
                    self.analysis.build()
            with ui.HStack(height=20):
                ui.StringField(model=self._status_model, read_only=True)

    # ---------------------------------------------------------------------------------
    # Move / rotate / align
    # ---------------------------------------------------------------------------------
    def _build_move(self):
        with section("Move & Rotate"):
            with ui.VStack(spacing=3):
                self._move_step = self.units.float("Move step", 0.1, "length", "Distance per nudge")
                self._move_space = ComboInput("Space", ["world", "local"])
                for axis in "XYZ":
                    button_row([
                        (f"{axis} -", lambda a=axis: self._nudge(a, -1)),
                        (f"{axis} +", lambda a=axis: self._nudge(a, +1)),
                    ])
                ui.Spacer(height=4)
                self._rot_step = FloatInput("Rotate step (deg)", 15.0, step=1.0)
                self._rot_pivot = ComboInput("Pivot", ["self", "bbox", "group"],
                                             tooltip="self = prim origin, bbox = own bbox center, "
                                                     "group = center of the whole selection")
                button_row([(f"R{a} {s}", lambda a=a, s=s: self._rotate(a, 1 if s == "+" else -1))
                            for a in "XYZ" for s in "-+"], height=22, spacing=2)
                ui.Spacer(height=4)
                self._abs_pos = self.units.vec3("World position", (0.0, 0.0, 0.0), "length")
                button_row([("Get from selection", self._get_position), ("Apply to selection", self._set_position)])
                ui.Spacer(height=4)
                self._align_axis = ComboInput("Align / Distribute", ["X", "Y", "Z"], default=2)
                self._align_mode = ComboInput("Align mode", ["min", "center", "max"], default=0)
                self._align_ref = ComboInput("Align to", ["first", "last", "group"])
                self._dist_gap = self.units.float("Distribute gap", -1.0, "length", "< 0: even spacing of centers, "
                                                                                    ">= 0: pack with this gap")
                button_row([("Align", self._align), ("Distribute", self._distribute)])
                self._drop_height = self.units.float("Ground height", 0.0, "length")
                button_row([
                    ("Drop to ground", lambda: self._drop(False), "Bbox bottom onto ground height"),
                    ("Drop onto objects", lambda: self._drop(True), "Rest on the highest overlapping prim"),
                    ("Copy pose", self._copy_pose, "Copy the first selected prim's pose to the others"),
                ])

    def _nudge(self, axis: str, sign: int):
        self._run(f"Move {axis}", lambda s, p: tf.translate(
            p, mu.AXIS_VECTORS[axis] * (sign * self._move_step.core(s)), self._move_space.value), min_sel=1)

    def _rotate(self, axis: str, sign: int):
        ang = sign * self._rot_step.value
        self._run(f"Rotate {axis}", lambda s, p: tf.rotate(p, axis, ang, pivot=self._rot_pivot.value,
                                                           space=self._move_space.value), min_sel=1)

    def _get_position(self):
        prims = self._selection()
        if prims:
            self._abs_pos.set_core(self._stage(), tuple(tf.world_position(prims[0])))

    def _set_position(self):
        self._run("Set position", lambda s, p: [tf.set_world_position(x, self._abs_pos.core(s))
                                                for x in tf._top_level(p)], min_sel=1)

    def _align(self):
        self._run("Align", lambda s, p: tf.align(p, self._align_axis.value, self._align_mode.value,
                                                 self._align_ref.value), min_sel=2)

    def _distribute(self):
        self._run("Distribute", lambda s, p: tf.distribute(
            p, self._align_axis.value, None if self._dist_gap.value < 0 else self._dist_gap.core(s)), min_sel=2)

    def _drop(self, onto: bool):
        def op(stage, prims):
            others = [x for x in stage.Traverse() if x.HasAPI(UsdPhysics.CollisionAPI)] if onto else None
            tf.drop_to(prims, self._drop_height.core(stage), up_axis=str(UsdGeom.GetStageUpAxis(stage)),
                       onto=others)

        self._run("Drop", op, min_sel=1)

    def _copy_pose(self):
        self._run("Copy pose", lambda s, p: tf.copy_pose(p[0], p[1:]), min_sel=2,
                  modify=lambda p: [str(x.GetPath()) for x in p[1:]])

    # ---------------------------------------------------------------------------------
    # Snap
    # ---------------------------------------------------------------------------------
    def _build_snap(self):
        with section("Snap"):
            with ui.VStack(spacing=3):
                self._grid = self.units.float("Grid size", 0.05, "length")
                self._angle = FloatInput("Angle step (deg)", 15.0, step=1.0)
                button_row([
                    ("Pivot to grid", lambda: self._run("Grid snap", lambda s, p: sn.snap_to_grid(p, self._grid.core(s)),
                                                       min_sel=1)),
                    ("BBox corner to grid", lambda: self._run(
                        "Grid snap (bbox)", lambda s, p: sn.snap_to_grid(p, self._grid.core(s), use_bbox=True),
                        min_sel=1)),
                    ("Rotation to step", lambda: self._run(
                        "Angle snap", lambda s, p: sn.snap_rotation(p, self._angle.value), min_sel=1)),
                ])
                self._live_pos = BoolInput("Live grid snap", False, "Snap while dragging with the gizmo",
                                           on_changed=lambda v: self._toggle_live())
                self._live_rot = BoolInput("Live angle snap", False, on_changed=lambda v: self._toggle_live())
                ui.Separator(height=6)
                hint("Snap to target: the LAST selected prim is the target, all others move.")
                self._snap_mode = ComboInput("Match", ["pivot", "bbox"])
                self._snap_src = ComboInput("Nearest: from", ["pivot", "bbox"],
                                            tooltip="pivot = moving origin, bbox = closest of 27 bbox points")
                self._snap_cand = ComboInput("Nearest: to", ["bbox", "vertex", "pivot"],
                                             tooltip="bbox = 27 points (corners/edges/faces/center), "
                                                     "vertex = mesh vertices")
                self._snap_max = self.units.float("Max distance", 0.0, "length", "0 = unlimited")
                button_row([("Snap to target", self._snap_target), ("Snap to nearest point", self._snap_nearest)])
                ui.Separator(height=6)
                self._mate_face = ComboInput("Mate on target face", _FACE_ITEMS)
                self._mate_gap = self.units.float("Gap", 0.0, "length", step=0.001)
                self._mate_center = BoolInput("Center on face", True)
                button_row([("Mate faces / Stack", self._mate)])

    def _live_snap_params(self) -> dict:
        stage = self._stage()
        return {"step": self._grid.core(stage) if stage is not None else self._grid.si, "angle": self._angle.value,
                "position": self._live_pos.value, "rotation": self._live_rot.value}

    def _toggle_live(self):
        if self._live_pos.value or self._live_rot.value:
            self._live_snap.start()
            self._status("Live snap on.")
        else:
            self._live_snap.stop()
            self._status("Live snap off.")

    def _movers(self, prims):
        return [str(p.GetPath()) for p in prims[:-1]]

    def _snap_target(self):
        self._run("Snap to target", lambda s, p: sn.snap_to_target(p[:-1], p[-1], self._snap_mode.value),
                  min_sel=2, modify=self._movers)

    def _snap_nearest(self):
        def op(stage, prims):
            md = self._snap_max.core(stage) or None
            hits = [sn.snap_to_nearest_point(m, prims[-1], self._snap_src.value, self._snap_cand.value, md)
                    for m in prims[:-1]]
            if not any(h is not None for h in hits):
                raise RuntimeError("no snap point within max distance")

        self._run("Snap to nearest", op, min_sel=2, modify=self._movers)

    def _mate(self):
        self._run("Mate faces", lambda s, p: [sn.mate_faces(m, p[-1], self._mate_face.value, self._mate_gap.core(s),
                                                            self._mate_center.value) for m in p[:-1]],
                  min_sel=2, modify=self._movers)

    # ---------------------------------------------------------------------------------
    # Anchors
    # ---------------------------------------------------------------------------------
    def _build_anchor(self):
        with section("Anchors"):
            with ui.VStack(spacing=3):
                hint("Anchors are named frames on a part (holes, pins, mounting faces). Connect two anchors to "
                     "assemble parts, then create a joint exactly at the anchor.")
                self._anchor_name = StringInput("Name", "Anchor")
                self._anchor_kind = ComboInput("Kind", ["generic", "pin", "socket", "mount", "tcp"])
                self._anchor_loc = ComboInput("Location", _ANCHOR_LOCATIONS)
                self._anchor_custom = self.units.vec3("Custom position", (0.0, 0.0, 0.0), "length")
                button_row([("Create anchor on selection", self._create_anchor)])
                ui.Separator(height=6)
                hint("Connect: select the MOVING anchor first, then the TARGET anchor. The moving anchor's owner "
                     "(nearest rigid body, otherwise its parent) is moved.")
                self._conn_rot = BoolInput("Align orientation", True)
                self._conn_flip = BoolInput("Flip (Z faces Z)", False, "Rotate 180 deg about X so Z axes oppose")
                self._conn_joint = ComboInput("Then create joint", ["none", "fixed", "revolute", "prismatic",
                                                                    "spherical"])
                button_row([("Connect anchors", self._connect_anchors)])

    def _create_anchor(self):
        loc = self._anchor_loc.value

        def op(stage, prims):
            created = []
            for prim in prims:
                rot = None
                if loc == "Pivot":
                    pos = tf.world_position(prim)
                elif loc == "BBox center":
                    pos = tf.bbox_center(prim)
                elif loc == "Custom (world)":
                    pos = self._anchor_custom.core(stage)
                else:
                    face = loc.split()[0]
                    pos = tf.bbox_face_point(tf.world_bbox(prim), face)
                    rot = mu.rotation_from_to((0, 0, 1), mu.axis_vector(face)).GetQuat()
                created.append(sn.create_anchor(stage, prim, self._anchor_name.value, pos, rot,
                                                kind=self._anchor_kind.value))
            return created

        self._run("Create anchor", op, min_sel=1, modify=lambda p: [], select_result=True)

    def _connect_anchors(self):
        jtype = self._conn_joint.value

        def op(stage, prims):
            a, b = prims[0], prims[1]
            if not (sn.is_anchor(a) and sn.is_anchor(b)):
                raise ValueError("select two anchors")
            owner = sn.connect_anchors(a, b, self._conn_rot.value, self._conn_flip.value)
            if jtype != "none":
                spec = jt.JointSpec(jtype, axis="Z")
                jt.create_joint_at(stage, spec, sn.anchor_owner(b), owner, location="anchor", world_axis="anchor",
                                   anchor=b)
            return owner

        def modify(prims):
            return [str(sn.anchor_owner(prims[0]).GetPath())] if prims and sn.is_anchor(prims[0]) else []

        self._run("Connect anchors", op, min_sel=2, modify=modify)

    # ---------------------------------------------------------------------------------
    # Joints
    # ---------------------------------------------------------------------------------
    def _build_joint(self):
        with section("Joints"):
            with ui.VStack(spacing=3):
                hint("Select body0 then body1 (one prim = joint to world). Selecting two anchors creates the "
                     "joint between their owners at the first anchor; pick World axis 'Anchor frame' to use the "
                     "anchor orientation (Axis token = anchor axis, usually Z).")
                self._j_type = ComboInput("Type", list(jt.JOINT_TYPES), default=1,
                                          on_changed=lambda v: self.units.refresh_labels())
                self._j_loc = ComboInput("Location", list(_JOINT_LOCATIONS))
                self._j_custom = self.units.vec3("Custom position", (0.0, 0.0, 0.0), "length")
                self._j_axis = ComboInput("World axis", _JOINT_AXES)
                self._j_custom_axis = Vec3Input("Custom axis", (0, 0, 1))
                self._j_token = ComboInput("Axis token", ["X", "Y", "Z"],
                                           tooltip="Joint-frame axis that is mapped onto the world axis "
                                                   "(with 'Anchor frame' the anchor's own axis)")
                self._j_limit = BoolInput("Limits", False)
                self._j_lower = self.units.float("Lower", -90.0, self._jq("length", "angle"), step=1.0)
                self._j_upper = self.units.float("Upper", 90.0, self._jq("length", "angle"), step=1.0)
                self._j_drive = BoolInput("Drive", False)
                self._j_drive_type = ComboInput("Drive type", ["force", "acceleration"])
                self._j_stiff = self.units.float("Stiffness", 1000.0, self._jq("linear_stiffness", "angular_stiffness"),
                                                 step=10.0)
                self._j_damp = self.units.float("Damping", 100.0, self._jq("linear_damping", "angular_damping"),
                                                step=1.0)
                self._j_target = self.units.float("Target position", 0.0, self._jq("length", "angle"))
                self._j_tvel = self.units.float("Target velocity", 0.0, self._jq("velocity", "angular_velocity"))
                self._j_maxf = self.units.float("Max force (0=inf)", 0.0, self._jq("force", "torque"))
                self._j_break = self.units.float("Break force (0=never)", 0.0, "force")
                self._j_break_t = self.units.float("Break torque (0=never)", 0.0, "torque")
                self._j_collide = BoolInput("Collide bodies", False)
                self._j_excl = BoolInput("Exclude from artic.", False)
                self._j_parent = ComboInput("Joint prim under", ["<root>/Joints", "body0", "body1"])
                self._j_auto_rb = BoolInput("Add RigidBody to bodies", True,
                                            "Apply RigidBodyAPI + colliders to bodies that lack it")
                button_row([("Create joint", self._create_joint),
                            ("Realign selected joints", self._realign_joints)])

    def _jq(self, linear: str, angular: str):
        """Quantity of a joint input: ``linear`` for prismatic / distance joints, else ``angular``."""
        return lambda: linear if self._j_type.value in _LINEAR_JOINTS else angular

    def _joint_spec(self, stage: Usd.Stage) -> jt.JointSpec:
        jtype = self._j_type.value
        spec = jt.JointSpec(jtype, axis=self._j_token.value, collision_enabled=self._j_collide.value,
                            exclude_from_articulation=self._j_excl.value,
                            break_force=self._j_break.core(stage) or None,
                            break_torque=self._j_break_t.core(stage) or None)
        lower, upper = self._j_lower.core(stage), self._j_upper.core(stage)
        if self._j_limit.value:
            if jtype in ("revolute", "prismatic"):
                spec.lower, spec.upper = lower, upper
            elif jtype == "spherical":
                spec.cone0 = spec.cone1 = abs(upper)
            elif jtype == "distance":
                spec.min_distance, spec.max_distance = max(0.0, lower), upper
        if self._j_drive.value:
            drive = jt.DriveSpec(self._j_drive_type.value, self._j_stiff.core(stage), self._j_damp.core(stage),
                                 self._j_maxf.core(stage) or None, self._j_target.core(stage),
                                 self._j_tvel.core(stage))
            if jtype in ("revolute", "prismatic"):
                spec.drive = drive
            elif jtype == "d6":
                spec.d6_drives = {"rotX": drive}
        return spec

    def _joint_bodies(self, prims):
        anchor = next((p for p in prims if sn.is_anchor(p)), None)
        if len(prims) >= 2 and sn.is_anchor(prims[0]) and sn.is_anchor(prims[1]):
            return sn.anchor_owner(prims[0]), sn.anchor_owner(prims[1]), prims[0], True
        bodies = [p for p in prims if not sn.is_anchor(p)]
        if len(bodies) == 1:
            return None, bodies[0], anchor, False
        if len(bodies) >= 2:
            return bodies[0], bodies[1], anchor, False
        raise ValueError("select one or two bodies")

    def _create_joint(self):
        def modify(prims):
            if not self._j_auto_rb.value:
                return []
            try:
                b0, b1, _, _ = self._joint_bodies(prims)
            except ValueError:
                return []
            return [str(b.GetPath()) for b in (b0, b1) if b is not None]

        def op(stage, prims):
            b0, b1, anchor, anchor_pair = self._joint_bodies(prims)
            if self._j_auto_rb.value:
                missing = [b for b in (b0, b1) if b is not None and not b.HasAPI(UsdPhysics.RigidBodyAPI)]
                if missing:
                    ph.apply_rigid_body(missing)
            spec = self._joint_spec(stage)
            loc = "anchor" if anchor_pair else _JOINT_LOCATIONS[self._j_loc.value]
            axis_item = self._j_axis.value
            if axis_item == "Anchor frame":
                world_axis = "anchor"
            elif axis_item == "Custom (world)":
                world_axis = self._j_custom_axis.value
            elif axis_item.startswith("Body0 local"):
                ref = b0 if b0 is not None else b1
                world_axis = tuple(mu.rotation_of(tf.world_matrix(ref)).TransformDir(
                    mu.AXIS_VECTORS[axis_item[-1]]))
            else:
                world_axis = axis_item
            parent = None
            if self._j_parent.value == "body0" and b0 is not None:
                parent = b0.GetPath()
            elif self._j_parent.value == "body1" and b1 is not None:
                parent = b1.GetPath()
            return jt.create_joint_at(stage, spec, b0, b1, location=loc, world_axis=world_axis, anchor=anchor,
                                      custom_position=self._j_custom.core(stage), parent_path=parent)

        self._run("Create joint", op, min_sel=1, modify=modify, select_result=True)

    def _realign_joints(self):
        def op(stage, prims):
            joints = [p for p in prims if p.IsA(UsdPhysics.Joint)]
            if not joints:
                raise ValueError("select joint prims")
            for j in joints:
                jt.realign_joint(j, keep="body0")

        self._run("Realign joints", op, min_sel=1)

    # ---------------------------------------------------------------------------------
    # Physics properties
    # ---------------------------------------------------------------------------------
    def _build_physics(self):
        with section("Physics Properties"):
            with ui.VStack(spacing=3):
                self._approx = ComboInput("Collider approx.", list(ph.APPROXIMATIONS), default=1)
                button_row([
                    ("Rigid body", lambda: self._run("Rigid body", lambda s, p: ph.apply_rigid_body(
                        p, approximation=self._approx.value), min_sel=1)),
                    ("Kinematic", lambda: self._run("Kinematic body", lambda s, p: ph.apply_rigid_body(
                        p, kinematic=True, approximation=self._approx.value), min_sel=1)),
                    ("Collider only", lambda: self._run("Collider", lambda s, p: ph.apply_collision(
                        p, self._approx.value), min_sel=1)),
                ])
                self._mass = self.units.float("Mass (0=auto)", 0.0, "mass", step=0.1)
                self._density = self.units.float("Density (0=auto)", 0.0, "density", step=10.0)
                button_row([("Set mass / density", lambda: self._run("Mass", lambda s, p: ph.set_mass(
                    p, self._mass.core(s) or None, self._density.core(s) or None), min_sel=1))])
                ui.Separator(height=6)
                self._sf = FloatInput("Static friction", 0.5)
                self._df = FloatInput("Dynamic friction", 0.5)
                self._rest = FloatInput("Restitution", 0.0)
                button_row([("Create + bind physics material", self._material)])
                ui.Separator(height=6)
                button_row([
                    ("Articulation root", lambda: self._run("Articulation root", lambda s, p: [
                        ph.apply_articulation_root(x) for x in p], min_sel=1)),
                    ("Filter pair collision", lambda: self._run("Filter collisions", lambda s, p: ph.filter_collisions(
                        p[0], p[1]), min_sel=2, modify=lambda p: [str(p[0].GetPath())])),
                ])
                button_row([
                    ("Physics scene", lambda: self._run("Physics scene", lambda s, p: ph.ensure_physics_scene(s),
                                                        modify=lambda p: [])),
                    ("Ground plane", lambda: self._run("Ground plane", lambda s, p: ph.create_ground_plane(s),
                                                       modify=lambda p: [])),
                    ("Remove physics", lambda: self._run("Remove physics", lambda s, p: ph.remove_physics(p),
                                                         min_sel=1)),
                ])

    def _material(self):
        def op(stage, prims):
            mat = ph.create_physics_material(stage, static_friction=self._sf.value,
                                             dynamic_friction=self._df.value, restitution=self._rest.value)
            ph.bind_physics_material(prims, mat)
            return mat

        self._run("Physics material", op, min_sel=1)

    # ---------------------------------------------------------------------------------
    # Templates
    # ---------------------------------------------------------------------------------
    def _build_templates(self):
        with section("Mechanism Templates"):
            with ui.VStack(spacing=3):
                self._tpl = ComboInput("Template", list(me.TEMPLATES), on_changed=lambda v: self._tpl_frame.rebuild())
                self._tpl_frame = ui.Frame(height=0)
                self._tpl_frame.set_build_fn(self._build_template_params)
                button_row([("Build template", self._build_template)])

    def _build_template_params(self):
        """One input per builder parameter; annotated quantities (core.units) show in the unit system.
        Defaults are the builders' metre-based values."""
        fn = me.TEMPLATES[self._tpl.value]
        self._template_inputs = {}
        with ui.VStack(spacing=2, height=0):
            for name, param in inspect.signature(fn).parameters.items():
                if name in ("stage", "parent"):
                    continue
                d = param.default
                q = un.annotation_quantity(param.annotation)
                label = name.replace("_", " ")
                if isinstance(d, bool):
                    self._template_inputs[name] = ("bool", BoolInput(label, d))
                elif isinstance(d, int):
                    self._template_inputs[name] = ("int", IntInput(label, d, min_value=1))
                elif isinstance(d, float):
                    self._template_inputs[name] = ("float", self.units.float(label, d, q) if q else FloatInput(label, d))
                elif isinstance(d, str):
                    self._template_inputs[name] = ("str", StringInput(label, d))
                elif isinstance(d, tuple) and len(d) in (2, 3):
                    vals = tuple(float(v) for v in d) + (0.0,) * (3 - len(d))
                    widget = self.units.vec3(label, vals, q) if q else Vec3Input(label, vals)
                    self._template_inputs[name] = (f"vec{len(d)}", widget)
                elif d is None:
                    self._template_inputs[name] = ("optional", self.units.float(label + " (0=none)", 0.0, q))
            doc = (inspect.getdoc(fn) or "").split("\n\n")[0]
            hint(doc)

    def _build_template(self):
        fn = me.TEMPLATES[self._tpl.value]

        def op(stage, prims):
            kwargs = {}
            for name, (kind, widget) in self._template_inputs.items():
                v = widget.core(stage) if isinstance(widget, (UnitFloat, UnitVec3)) else widget.value
                if kind == "vec2":
                    v = tuple(v[:2])
                elif kind == "optional":
                    v = v or None
                kwargs[name] = v
            res = fn(stage, **kwargs)
            return [res["root"][0]]

        self._run(f"Build {self._tpl.value}", op, modify=lambda p: [], select_result=True)

    # ---------------------------------------------------------------------------------
    # Validation
    # ---------------------------------------------------------------------------------
    def _build_validate(self):
        with section("Validate"):
            with ui.VStack(spacing=3):
                button_row([("Validate stage", self._validate), ("Auto-fix fixable issues", self._autofix)])
                ui.StringField(model=self._validation_model, multiline=True, read_only=True, height=160)

    def _validate(self):
        stage = self._stage()
        if stage is None:
            return
        self._issues = va.validate(stage)
        self._validation_model.set_value(va.format_report(self._issues))
        n_err = sum(i.severity == va.ERROR for i in self._issues)
        self._status(f"Validation: {len(self._issues)} issue(s), {n_err} error(s).")

    def _autofix(self):
        if not self._issues:
            self._validate()
        fixable = [i for i in self._issues if i.fix]
        if not fixable:
            self._status("Nothing to auto-fix.")
            return
        paths = sorted({i.path for i in fixable if i.path != "/"})
        self._run("Auto-fix", lambda s, p: [va.apply_fix(s, i) for i in fixable], modify=lambda p: paths)
        self._validate()
