"""Parametric mechanism templates built from primitive rigid bodies and joints.

Every builder creates a self-contained hierarchy::

    <root>            Xform placed at ``origin``
      <Body>...       rigid bodies (Xform + child geometry, body scale = 1)
      Joints/...      joints between them

Dimensions are in stage units (Isaac Sim default: meters), angles in degrees, and layouts
are authored Z-up in the root's local frame (the root is rotated for Y-up stages).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import joints as jt
from . import math_utils as mu
from . import physics as ph
from . import transform as tf
from .snapping import create_anchor
from .stage_utils import default_root, stage_up_axis, unique_child_path

STEEL = Gf.Vec3f(0.62, 0.64, 0.68)
ORANGE = Gf.Vec3f(0.95, 0.55, 0.1)
BLUE = Gf.Vec3f(0.2, 0.45, 0.85)
DARK = Gf.Vec3f(0.25, 0.25, 0.28)


class _Builder:
    """Shared plumbing: root frame, bodies and joints expressed in root-local coordinates."""

    def __init__(self, stage: Usd.Stage, name: str, origin, parent: Optional[Sdf.Path] = None,
                 direction: Optional[mu.VecLike] = None):
        self.stage = stage
        parent = Sdf.Path(str(parent)) if parent else default_root(stage)
        self.root_path = unique_child_path(stage, parent, name)
        root = UsdGeom.Xform.Define(stage, self.root_path).GetPrim()
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 0)
        if stage_up_axis(stage) == "Y":
            rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)  # local Z-up -> world Y-up
        if direction is not None:
            rot = mu.rotation_from_to((0, 0, 1), direction)
        tf.set_world_matrix(root, mu.make_transform(origin, rot))
        self.root = root
        self.result: Dict[str, List[str]] = {"root": [str(self.root_path)], "bodies": [], "joints": [], "anchors": []}

    # -- frames -----------------------------------------------------------------------
    @property
    def root_world(self) -> Gf.Matrix4d:
        return tf.world_matrix(self.root)

    def wpos(self, p) -> Gf.Vec3d:
        return self.root_world.Transform(mu.vec3d(p))

    def wrot(self, local_rot=None) -> Gf.Quatd:
        local = Gf.Matrix4d().SetRotate(mu.to_rotation(local_rot)) if local_rot is not None else Gf.Matrix4d(1)
        return mu.rigid_part(local * self.root_world).ExtractRotationQuat()

    # -- bodies ------------------------------------------------------------------------
    def body(self, name: str, pos, shape: str, dims, rot=None, color=STEEL, mass: Optional[float] = None,
             collider: bool = True, dynamic: bool = True, geom_offset=(0, 0, 0)) -> Usd.Prim:
        """Rigid body Xform with one child shape.

        ``shape``: "box" (dims = sx, sy, sz), "cylinder" (dims = radius, height; axis Z) or
        "capsule" (dims = radius, height; axis Z).  ``geom_offset`` shifts the shape inside
        the body (so the body origin can sit on a pivot).
        """
        path = unique_child_path(self.stage, self.root_path, name)
        prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
        tf.set_local_matrix(prim, mu.make_transform(pos, rot))
        gpath = path.AppendChild("geom")
        if shape == "box":
            g = UsdGeom.Cube.Define(self.stage, gpath)
            g.CreateSizeAttr(1.0)
            gx = UsdGeom.Xformable(g)
            gx.AddTranslateOp().Set(mu.vec3d(geom_offset))
            gx.AddScaleOp().Set(mu.vec3d(dims))
            g.CreateExtentAttr([-Gf.Vec3f(0.5), Gf.Vec3f(0.5)])
        elif shape in ("cylinder", "capsule"):
            g = (UsdGeom.Cylinder if shape == "cylinder" else UsdGeom.Capsule).Define(self.stage, gpath)
            g.CreateRadiusAttr(float(dims[0]))
            g.CreateHeightAttr(float(dims[1]))
            g.CreateAxisAttr("Z")
            UsdGeom.Xformable(g).AddTranslateOp().Set(mu.vec3d(geom_offset))
            r, h = float(dims[0]), float(dims[1])
            half_h = h / 2 + (r if shape == "capsule" else 0)
            g.CreateExtentAttr([Gf.Vec3f(-r, -r, -half_h), Gf.Vec3f(r, r, half_h)])
        else:
            raise ValueError(f"Unknown shape {shape}")
        g.CreateDisplayColorAttr([color])
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if not dynamic:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        if collider:
            ph.apply_collision([prim])
        if mass is not None:
            ph.set_mass([prim], mass=mass)
        self.result["bodies"].append(str(path))
        return prim

    # -- joints ------------------------------------------------------------------------
    def joint(self, spec: jt.JointSpec, b0: Optional[Usd.Prim], b1: Optional[Usd.Prim], local_pos,
              local_rot=None) -> Usd.Prim:
        j = jt.create_joint(self.stage, spec, b0, b1, self.wpos(local_pos), self.wrot(local_rot),
                            parent_path=self.root_path.AppendChild("Joints"))
        self.result["joints"].append(str(j.GetPath()))
        return j

    def anchor(self, parent: Usd.Prim, name: str, local_pos, local_rot=None, kind="mount") -> Usd.Prim:
        a = create_anchor(self.stage, parent, name, self.wpos(local_pos), self.wrot(local_rot), kind=kind)
        self.result["anchors"].append(str(a.GetPath()))
        return a


# ---------------------------------------------------------------------------------------
# Templates
# ---------------------------------------------------------------------------------------
def build_pendulum_chain(stage: Usd.Stage, name: str = "PendulumChain", origin=(0, 0, 2.0), links: int = 5,
                         link_length: float = 0.25, link_radius: float = 0.02, link_mass: float = 0.5,
                         initial_angle: float = 30.0, damping: float = 0.0, articulation: bool = False,
                         parent=None) -> Dict[str, List[str]]:
    """Chain of capsule links hanging from a world pivot, revolute about local Y."""
    b = _Builder(stage, name, origin, parent)
    swing = Gf.Rotation(Gf.Vec3d(0, 1, 0), initial_angle)
    down = swing.TransformDir(Gf.Vec3d(0, 0, -1))
    prev = None
    capsule_h = max(link_length - 2 * link_radius, 1e-4)
    for i in range(links):
        top = down * (link_length * i)
        link = b.body(f"Link_{i:02d}", top, "capsule", (link_radius, capsule_h), rot=swing,
                      color=ORANGE if i % 2 == 0 else BLUE, mass=link_mass,
                      geom_offset=(0, 0, -link_length / 2))
        spec = jt.JointSpec("revolute", name=f"Hinge_{i:02d}", axis="Y")
        if damping > 0:
            spec.drive = jt.DriveSpec(stiffness=0.0, damping=damping)
        b.joint(spec, prev, link, top)
        prev = link
    if articulation:
        ph.apply_articulation_root(b.root)
    return b.result


def build_hydraulic_cylinder(stage: Usd.Stage, name: str = "HydraulicCylinder", origin=(0, 0, 0),
                             direction=(0, 0, 1), retracted_length: float = 0.6, stroke: float = 0.4,
                             bore: float = 0.08, rod_diameter: float = 0.04, barrel_mass: float = 8.0,
                             rod_mass: float = 3.0, stiffness: float = 2.0e4, damping: float = 2.0e3,
                             max_force: Optional[float] = None, fixed_base: bool = True,
                             parent=None) -> Dict[str, List[str]]:
    """Barrel + rod connected by a driven prismatic joint (stroke limits, PD drive).

    Anchors ``MountBase`` (barrel end) and ``MountRod`` (rod eye) let you attach the
    cylinder to other bodies with "Connect anchors" and revolute clevis joints.
    """
    lb = float(retracted_length)
    stroke = min(float(stroke), 0.9 * lb)
    tip_out = 0.1 * lb
    b = _Builder(stage, name, origin, parent, direction=direction)
    barrel = b.body("Barrel", (0, 0, 0), "cylinder", (bore / 2, lb), color=DARK, mass=barrel_mass,
                    geom_offset=(0, 0, lb / 2))
    rod_len = 0.95 * lb + tip_out
    tip = lb + tip_out
    rod = b.body("Rod", (0, 0, tip), "cylinder", (rod_diameter / 2, rod_len), color=STEEL, mass=rod_mass,
                 geom_offset=(0, 0, -rod_len / 2))
    if fixed_base:
        b.joint(jt.JointSpec("fixed", name="BaseFixed"), None, barrel, (0, 0, 0))
    spec = jt.JointSpec("prismatic", name="Stroke", axis="Z", lower=0.0, upper=stroke,
                        drive=jt.DriveSpec(stiffness=stiffness, damping=damping, max_force=max_force,
                                           target_position=0.0))
    b.joint(spec, barrel, rod, (0, 0, tip))
    b.anchor(barrel, "MountBase", (0, 0, 0))
    b.anchor(rod, "MountRod", (0, 0, tip))
    return b.result


def build_hinged_door(stage: Usd.Stage, name: str = "HingedDoor", origin=(0, 0, 0), width: float = 0.9,
                      height: float = 2.0, thickness: float = 0.04, door_mass: float = 20.0,
                      open_limit: float = 110.0, damping: float = 5.0, parent=None) -> Dict[str, List[str]]:
    """Door frame post (fixed to world) and a panel on a limited revolute hinge (Z axis)."""
    b = _Builder(stage, name, origin, parent)
    post = b.body("Post", (0, 0, height / 2), "box", (0.06, 0.06, height), color=DARK, mass=10.0)
    door = b.body("Panel", (0.03, 0, height / 2), "box", (width, thickness, height), color=BLUE,
                  mass=door_mass, geom_offset=(width / 2, 0, 0))
    b.joint(jt.JointSpec("fixed", name="PostFixed"), None, post, (0, 0, 0))
    spec = jt.JointSpec("revolute", name="Hinge", axis="Z", lower=0.0, upper=open_limit,
                        drive=jt.DriveSpec(stiffness=0.0, damping=damping) if damping > 0 else None)
    b.joint(spec, post, door, (0.03, 0, height / 2))
    return b.result


def build_linear_slider(stage: Usd.Stage, name: str = "LinearSlider", origin=(0, 0, 0), rail_length: float = 1.0,
                        carriage_size=(0.15, 0.12, 0.06), carriage_mass: float = 2.0,
                        stiffness: float = 0.0, damping: float = 50.0, parent=None) -> Dict[str, List[str]]:
    """Rail fixed to world and a carriage on a limited prismatic joint along X."""
    b = _Builder(stage, name, origin, parent)
    rail_h = 0.04
    rail = b.body("Rail", (0, 0, rail_h / 2), "box", (rail_length, 0.05, rail_h), color=DARK, mass=5.0)
    cz = rail_h + carriage_size[2] / 2 + 0.002
    carriage = b.body("Carriage", (0, 0, cz), "box", carriage_size, color=ORANGE, mass=carriage_mass)
    b.joint(jt.JointSpec("fixed", name="RailFixed"), None, rail, (0, 0, 0))
    travel = max(0.0, rail_length / 2 - carriage_size[0] / 2)
    spec = jt.JointSpec("prismatic", name="Slide", axis="X", lower=-travel, upper=travel,
                        drive=jt.DriveSpec(stiffness=stiffness, damping=damping, target_position=0.0))
    b.joint(spec, rail, carriage, (0, 0, cz))
    return b.result


def build_scissor_lift(stage: Usd.Stage, name: str = "ScissorLift", origin=(0, 0, 0), stages: int = 3,
                       arm_length: float = 1.2, angle: float = 25.0, width: float = 0.8,
                       arm_section=(0.06, 0.03), sides: int = 2, arm_mass: float = 5.0,
                       platform_mass: float = 20.0, base_mass: float = 60.0, arm_collision: bool = False,
                       drive_stiffness: float = 1.0e5, drive_damping: float = 1.0e4,
                       parent=None) -> Dict[str, List[str]]:
    """Multi-stage scissor lift (closed kinematic loops → maximal-coordinate joints).

    Layout in the root frame: arms move in the XZ plane, pins rotate about Y.

    * ``A`` arms rise from -X to +X, ``B`` arms from +X to -X, crossing at a center pin.
    * Bottom: ``A0`` is pinned to the base at -X; ``B0`` is pinned to a slider that runs on
      the base along X (driven prismatic joint = actuator).
    * Top: ``B[n-1]`` is pinned to the platform at -X; ``A[n-1]`` rides a platform slider.

    Driving the bottom slider's prismatic target toward negative values raises the lift.
    """
    if stages < 1:
        raise ValueError("stages must be >= 1")
    th = math.radians(angle)
    L = float(arm_length)
    w = L / 2 * math.cos(th)
    h = L * math.sin(th)
    a, t = float(arm_section[0]), float(arm_section[1])
    base_t = 0.05
    z0 = base_t + a  # height of the bottom pins
    span = 2 * w + 0.2
    deck_w = width + 0.1

    b = _Builder(stage, name, origin, parent)
    base = b.body("Base", (0, 0, base_t / 2), "box", (span, deck_w, base_t), color=DARK, mass=base_mass)
    z_top = z0 + stages * h
    platform = b.body("Platform", (0, 0, z_top + a + 0.02), "box", (span, deck_w, 0.04), color=ORANGE,
                      mass=platform_mass)
    b.joint(jt.JointSpec("fixed", name="BaseFixed"), None, base, (0, 0, 0))

    side_list = [0.0] if sides == 1 else [-1.0, 1.0]
    rot_a = Gf.Rotation(Gf.Vec3d(0, 1, 0), -angle)  # local X -> (cos, 0, sin)
    rot_b = Gf.Rotation(Gf.Vec3d(0, 1, 0), angle)   # local X -> (cos, 0, -sin)
    pin = lambda nm: jt.JointSpec("revolute", name=nm, axis="Y")  # noqa: E731

    # slider travel (displacement from the initial pose, along X)
    x_raised = L / 2 * math.cos(math.radians(75.0))
    x_folded = L / 2 * math.cos(math.radians(4.0))
    lower, upper = x_raised - w, x_folded - w

    for si, s in enumerate(side_list):
        tag = "" if sides == 1 else ("_L" if s < 0 else "_R")
        y_out = s * width / 2
        y_in = s * (width / 2 - t) if sides != 1 else t
        y_pin = (y_out + y_in) / 2
        arms_a, arms_b = [], []
        for i in range(stages):
            zc = z0 + i * h + h / 2
            arms_a.append(b.body(f"ArmA{tag}_{i}", (0, y_out, zc), "box", (L, t, a), rot=rot_a, color=STEEL,
                                 mass=arm_mass, collider=arm_collision))
            arms_b.append(b.body(f"ArmB{tag}_{i}", (0, y_in, zc), "box", (L, t, a), rot=rot_b, color=BLUE,
                                 mass=arm_mass, collider=arm_collision))
            b.joint(pin(f"Center{tag}_{i}"), arms_a[i], arms_b[i], (0, y_pin, zc))
        for i in range(stages - 1):
            zj = z0 + (i + 1) * h
            b.joint(pin(f"LinkPlus{tag}_{i}"), arms_a[i], arms_b[i + 1], (w, y_pin, zj))
            b.joint(pin(f"LinkMinus{tag}_{i}"), arms_b[i], arms_a[i + 1], (-w, y_pin, zj))

        # bottom: fixed pin + driven slider
        b.joint(pin(f"BasePin{tag}"), base, arms_a[0], (-w, y_pin, z0))
        slider_b = b.body(f"BaseSlider{tag}", (w, y_pin, z0), "box", (0.08, t, a), color=DARK, mass=1.0,
                          collider=False)
        b.joint(pin(f"BaseSliderPin{tag}"), slider_b, arms_b[0], (w, y_pin, z0))
        b.joint(jt.JointSpec("prismatic", name=f"Actuator{tag}", axis="X", lower=lower, upper=upper,
                             drive=jt.DriveSpec(stiffness=drive_stiffness, damping=drive_damping,
                                                target_position=0.0)),
                base, slider_b, (w, y_pin, z0))

        # top: fixed pin + free slider
        b.joint(pin(f"PlatformPin{tag}"), platform, arms_b[-1], (-w, y_pin, z_top))
        slider_t = b.body(f"TopSlider{tag}", (w, y_pin, z_top), "box", (0.08, t, a), color=DARK, mass=1.0,
                          collider=False)
        b.joint(pin(f"TopSliderPin{tag}"), slider_t, arms_a[-1], (w, y_pin, z_top))
        b.joint(jt.JointSpec("prismatic", name=f"TopSlide{tag}", axis="X", lower=lower, upper=upper),
                platform, slider_t, (w, y_pin, z_top))
    return b.result


def scissor_height(arm_length: float, angle: float, stages: int) -> float:
    """Pin-to-pin height of a scissor stack for a given arm angle (deg)."""
    return stages * arm_length * math.sin(math.radians(angle))


def scissor_slider_offset_for_height(arm_length: float, angle0: float, stages: int, height: float) -> float:
    """Prismatic target that raises a lift built at ``angle0`` to the pin height ``height``."""
    s = max(-1.0, min(1.0, height / (stages * arm_length)))
    angle = math.degrees(math.asin(s))
    return arm_length / 2 * (math.cos(math.radians(angle)) - math.cos(math.radians(angle0)))


TEMPLATES = {
    "Pendulum chain": build_pendulum_chain,
    "Hydraulic cylinder": build_hydraulic_cylinder,
    "Hinged door": build_hinged_door,
    "Linear slider": build_linear_slider,
    "Scissor lift": build_scissor_lift,
}
