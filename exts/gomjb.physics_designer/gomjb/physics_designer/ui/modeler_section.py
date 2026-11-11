"""Modeler section: create and edit physics-ready primitive bodies."""
from __future__ import annotations

from typing import List

import omni.ui as ui
from pxr import UsdPhysics

from ..core import math_utils as mu
from ..core import modeling as md
from ..core import snapping as sn
from ..core import transform as tf
from ..core.stage_utils import default_root, unique_child_path
from .widgets import ComboInput, FloatInput, StringInput, Vec3Input, button_row, hint, section

DEFAULT_DIMS = {
    "box": (0.5, 0.3, 0.2),
    "cylinder": (0.1, 0.5),
    "sphere": (0.15,),
    "capsule": (0.05, 0.4),
    "cone": (0.15, 0.3),
    "link": (0.8, 0.06, 0.02),
}
PLACEMENTS = ["Position below", "On top of selection", "At selected anchor"]


class ModelerSection:
    """Builds into the Physics Designer window; edits run through the window's undoable ``_run``."""

    def __init__(self, window):
        self._w = window
        self._dim_inputs: List[FloatInput] = []

    def build(self):
        self.frame = section("Modeler", collapsed=False)
        with self.frame:
            with ui.VStack(spacing=3):
                hint("Creates a rigid body (frame + geometry + material + exact mass properties). "
                     "A link gets pin anchors PinA / PinB at both ends for joints.")
                self._shape = ComboInput("Shape", list(md.SHAPES), on_changed=lambda v: self._dims_frame.rebuild())
                self._dims_frame = ui.Frame(height=0)
                self._dims_frame.set_build_fn(self._build_dims)
                self._name = StringInput("Name (optional)", "")
                self._material = ComboInput("Material", list(md.MATERIALS))
                self._density = FloatInput("Density (0=material)", 0.0, "kg/m^3", step=10.0)
                self._mass = FloatInput("Mass (0=from density)", 0.0, "kg; inertia follows the geometry", step=0.1)
                self._type = ComboInput("Body type", list(md.BODY_TYPES),
                                        tooltip="static = collider only, visual = no physics")
                self._place = ComboInput("Placement", PLACEMENTS,
                                         tooltip="At selected anchor: a link puts its PinA on the anchor")
                self._pos = Vec3Input("Position")
                self._rot = Vec3Input("Rotation XYZ (deg)")
                button_row([("Create body", self.create)])
                button_row([("Load from selected", self.load_selected),
                            ("Apply to selected", self.apply_selected, "New dimensions / material / mass")])

    def _build_dims(self):
        shape = self._shape.value
        self._dim_inputs = []
        with ui.VStack(spacing=2, height=0):
            for label, value in zip(md.SHAPES[shape], DEFAULT_DIMS[shape]):
                self._dim_inputs.append(FloatInput(label.capitalize(), value, step=0.005))

    def _dims(self):
        return [d.value for d in self._dim_inputs]

    # -- actions --------------------------------------------------------------------
    def create(self):
        shape, dims = self._shape.value, self._dims()
        placement = self._place.value

        def op(stage, prims):
            name = self._name.value.strip() or shape.capitalize()
            path = unique_child_path(stage, default_root(stage), name)
            body = md.create_body(stage, path, shape, dims, material=self._material.value,
                                  density=self._density.value or None, mass=self._mass.value or None,
                                  body_type=self._type.value)
            tf.set_world_pose(body, self._pos.value, mu.quat_from_euler_xyz(self._rot.value))
            if placement == "On top of selection":
                if not prims:
                    raise ValueError("select the prim to place the body on")
                sn.mate_faces(body, prims[-1], "+Z")
            elif placement == "At selected anchor":
                anchor = next((p for p in prims if sn.is_anchor(p)), None)
                if anchor is None:
                    raise ValueError("select an anchor")
                if shape == "link":
                    sn.connect_anchors(body.GetChild("PinA"), anchor, align_rotation=True, owner=body)
                else:
                    tf.set_world_position(body, tf.world_position(anchor))
            return body

        body = self._w._run(f"Create {shape}", op, modify=lambda p: [], select_result=True)
        if body is not None and body.HasAPI(UsdPhysics.MassAPI):
            props = md.body_mass_properties(body, si=True)
            self._w._status(f"Created {body.GetPath()}: mass {props.mass:.3f} kg, "
                            f"I = ({props.inertia[0]:.4g}, {props.inertia[1]:.4g}, {props.inertia[2]:.4g}) kg*m^2")

    def load_selected(self):
        prims = [p for p in self._w._selection() if md.is_designer_body(p)]
        if not prims:
            self._w._status("Load: select a body created by the modeler.", error=True)
            return
        p = md.body_params(prims[0])
        self._shape.widget.model.get_item_value_model().set_value(self._shape.items.index(p["shape"]))
        self._dims_frame.rebuild()
        for field, value in zip(self._dim_inputs, p["dims"]):
            field.value = value
        self._material.widget.model.get_item_value_model().set_value(self._material.items.index(p["material"]))
        self._density.value = p["density"] or 0.0
        self._mass.value = p["mass"] or 0.0
        self._w._status(f"Loaded parameters of {prims[0].GetName()}.")

    def apply_selected(self):
        def op(stage, prims):
            bodies = [p for p in prims if md.is_designer_body(p)]
            if not bodies:
                raise ValueError("select bodies created by the modeler")
            for body in bodies:
                same_shape = md.body_params(body)["shape"] == self._shape.value
                md.update_body(body, dims=self._dims() if same_shape else None, material=self._material.value,
                               density=self._density.value, mass=self._mass.value)
            return bodies

        self._w._run("Apply body parameters", op, min_sel=1)
