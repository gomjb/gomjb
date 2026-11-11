"""Parametric mechanism templates built from primitive rigid bodies and joints.

Every builder creates a self-contained hierarchy::

    <root>            Xform placed at ``origin``, ArticulationRootAPI
      <Body>...       rigid bodies (Xform + child geometry, body scale = 1)
      Joints/...      joints between them (body0 = parent, body1 = child)

Each mechanism is one fixed-base articulation, the convention of Isaac Sim robot assets:
PhysX solves it with exact reduced-coordinate joints and Newton only accepts joints that
belong to an articulation.  Loop-closing joints are ``excludeFromArticulation``.

Dimensions and drive parameters are in stage units (Isaac Sim default: meters), masses in kg,
angles in degrees; parameter annotations (:mod:`units`) name each quantity for the window.
Layouts are authored Z-up in the root's local frame (the root is rotated for Y-up stages).
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional

from pxr import Gf, Sdf, Usd, UsdGeom

from . import joints as jt
from . import math_utils as mu
from . import modeling as md
from . import physics as ph
from . import transform as tf
from .snapping import create_anchor
from .stage_utils import default_root, stage_up_axis, unique_child_path
from .units import Angle, AngularDamping, Force, Length, Lengths, LinearDamping, LinearStiffness, Mass

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
        ph.apply_articulation_root(root)
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
             collider: bool = True, geom_offset=(0, 0, 0)) -> Usd.Prim:
        """Modeler body (see :func:`modeling.create_body`) placed in the root frame.

        ``geom_offset`` shifts the shape inside the body so the body origin can sit on a pivot.
        """
        path = unique_child_path(self.stage, self.root_path, name)
        prim = md.create_body(self.stage, path, shape, dims, mass=mass, collision=collider, color=color,
                              offset=geom_offset)
        tf.set_local_matrix(prim, mu.make_transform(pos, rot))
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
def build_pendulum_chain(stage: Usd.Stage, name: str = "PendulumChain", origin: Lengths = (0, 0, 2.0),
                         links: int = 5, link_length: Length = 0.25, link_radius: Length = 0.02,
                         link_mass: Mass = 0.5, initial_angle: Angle = 30.0, damping: AngularDamping = 0.0,
                         parent=None) -> Dict[str, List[str]]:
    """Chain of capsule links hanging from a support fixed to the world, revolute about local Y."""
    b = _Builder(stage, name, origin, parent)
    swing = Gf.Rotation(Gf.Vec3d(0, 1, 0), initial_angle)
    down = swing.TransformDir(Gf.Vec3d(0, 0, -1))
    # PhysX articulations attach to the world through a fixed joint only: hang the chain from a
    # fixed support link so the first hinge is an articulation DOF (with readable forces)
    prev = b.body("Support", (0, 0, 0), "box", (0.08, 0.08, 0.04), color=DARK, mass=1.0, collider=False,
                  geom_offset=(0, 0, 0.02 + link_radius))
    b.joint(jt.JointSpec("fixed", name="SupportFixed"), None, prev, (0, 0, 0))
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
    return b.result


def build_hydraulic_cylinder(stage: Usd.Stage, name: str = "HydraulicCylinder", origin: Lengths = (0, 0, 0),
                             direction=(0, 0, 1), retracted_length: Length = 0.6, stroke: Length = 0.4,
                             bore: Length = 0.08, rod_diameter: Length = 0.04, barrel_mass: Mass = 8.0,
                             rod_mass: Mass = 3.0, stiffness: LinearStiffness = 2.0e4,
                             damping: LinearDamping = 2.0e3, max_force: Optional[Force] = None, fixed_base: bool = True,
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


def build_hinged_door(stage: Usd.Stage, name: str = "HingedDoor", origin: Lengths = (0, 0, 0),
                      width: Length = 0.9, height: Length = 2.0, thickness: Length = 0.04, door_mass: Mass = 20.0,
                      open_limit: Angle = 110.0, damping: AngularDamping = 5.0, parent=None) -> Dict[str, List[str]]:
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


def build_linear_slider(stage: Usd.Stage, name: str = "LinearSlider", origin: Lengths = (0, 0, 0),
                        rail_length: Length = 1.0, carriage_size: Lengths = (0.15, 0.12, 0.06),
                        carriage_mass: Mass = 2.0, stiffness: LinearStiffness = 0.0, damping: LinearDamping = 50.0,
                        parent=None) -> Dict[str, List[str]]:
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


def build_scissor_lift(stage: Usd.Stage, name: str = "ScissorLift", origin: Lengths = (0, 0, 0), stages: int = 3,
                       arm_length: Length = 1.2, angle: Angle = 25.0, width: Length = 0.8,
                       arm_section: Lengths = (0.06, 0.03), sides: int = 2, arm_mass: Mass = 5.0,
                       platform_mass: Mass = 20.0, base_mass: Mass = 60.0, arm_collision: bool = False,
                       sag_tolerance: Length = 0.002, parent=None) -> Dict[str, List[str]]:
    """Multi-stage scissor lift.

    Layout in the root frame: arms move in the XZ plane, pins rotate about Y.

    * ``A`` arms rise from -X to +X, ``B`` arms from +X to -X, crossing at a center pin.
    * Bottom: ``A0`` is pinned to the base at -X; ``B0`` is pinned to a slider that runs on
      the base along X (driven prismatic joint = actuator).
    * Top: ``B[n-1]`` is pinned to the platform at -X; ``A[n-1]`` rides a platform slider.

    Driving the bottom slider's prismatic target toward negative values raises the lift
    (see :func:`scissor_slider_offset_for_height`).

    A scissor is a chain of closed loops with a large motion ratio, which plain
    (maximal-coordinate) joints cannot hold under load.  The lift is therefore one fixed-base
    articulation (exact reduced-coordinate joints, actuator in the tree) and only the
    loop-closing joints are ``excludeFromArticulation`` — the standard PhysX pattern for loops.
    The actuator drive is sized from the lifted weight: stiffness keeps the static platform
    sag under ``sag_tolerance`` (stage units) and damping is critical for the lifted mass
    reflected onto the slider.
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

    def pin(nm: str, loop: bool = False) -> jt.JointSpec:
        return jt.JointSpec("revolute", name=nm, axis="Y", exclude_from_articulation=loop)

    # slider travel along X, measured from the initial pose (fixed pin to slider span = L·cos)
    lower = scissor_slider_offset(L, angle, 75.0)
    upper = scissor_slider_offset(L, angle, 4.0)

    # actuator drive sized from the static load and the scissor's motion ratio
    n_sides = len(side_list)
    ratio = stages / math.tan(th)  # platform rise per unit of slider travel
    gravity = 9.81 / float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    load = scissor_actuator_force(L, angle, stages, platform_mass, arm_mass, SCISSOR_SLIDER_MASS, n_sides, gravity)
    stiffness = load * ratio / sag_tolerance  # slider deflection = sag / ratio
    lifted = load / (gravity * ratio) * n_sides  # weight-equivalent mass seen by the platform
    damping = 2.0 * math.sqrt(stiffness * lifted * ratio ** 2 / n_sides)

    for si, s in enumerate(side_list):
        tag = "" if sides == 1 else ("_L" if s < 0 else "_R")
        y_out = s * width / 2
        y_in = s * (width / 2 - t) if sides != 1 else t
        y_pin = (y_out + y_in) / 2
        arms_a, arms_b = [], []
        for i in range(stages):
            zc = z0 + i * h + h / 2
            # links: pin distance L along local X, width a in the lift plane, thickness t; pin axis Y
            arms_a.append(b.body(f"ArmA{tag}_{i}", (0, y_out, zc), "link", (L, a, t), rot=rot_a, color=STEEL,
                                 mass=arm_mass, collider=arm_collision))
            arms_b.append(b.body(f"ArmB{tag}_{i}", (0, y_in, zc), "link", (L, a, t), rot=rot_b, color=BLUE,
                                 mass=arm_mass, collider=arm_collision))
            # articulation tree (body0 = parent, body1 = child, as Newton requires):
            # base → slider → B0 → A0 → links upward; centers above stage 0 close loops
            b.joint(pin(f"Center{tag}_{i}", loop=i > 0), arms_b[i], arms_a[i], (0, y_pin, zc))
        for i in range(stages - 1):
            zj = z0 + (i + 1) * h
            b.joint(pin(f"LinkPlus{tag}_{i}"), arms_a[i], arms_b[i + 1], (w, y_pin, zj))
            b.joint(pin(f"LinkMinus{tag}_{i}"), arms_b[i], arms_a[i + 1], (-w, y_pin, zj))

        # bottom: fixed pin (closes the base loop) + driven slider
        b.joint(pin(f"BasePin{tag}", loop=True), base, arms_a[0], (-w, y_pin, z0))
        slider_b = b.body(f"BaseSlider{tag}", (w, y_pin, z0), "box", (0.08, t, a), color=DARK,
                          mass=SCISSOR_SLIDER_MASS, collider=False)
        b.joint(pin(f"BaseSliderPin{tag}"), slider_b, arms_b[0], (w, y_pin, z0))
        b.joint(jt.JointSpec("prismatic", name=f"Actuator{tag}", axis="X", lower=lower, upper=upper,
                             drive=jt.DriveSpec(stiffness=stiffness, damping=damping, target_position=0.0)),
                base, slider_b, (w, y_pin, z0))

        # top: fixed pin + free slider (the platform joins the tree through the first side only)
        b.joint(pin(f"PlatformPin{tag}", loop=si > 0), arms_b[-1], platform, (-w, y_pin, z_top))
        slider_t = b.body(f"TopSlider{tag}", (w, y_pin, z_top), "box", (0.08, t, a), color=DARK,
                          mass=SCISSOR_SLIDER_MASS, collider=False)
        # loops close on revolute pins only: MuJoCo (Newton) maps them to "connect" constraints
        # but has no equality constraint for a prismatic loop closure
        b.joint(jt.JointSpec("prismatic", name=f"TopSlide{tag}", axis="X", lower=lower, upper=upper),
                platform, slider_t, (w, y_pin, z_top))
        b.joint(pin(f"TopSliderPin{tag}", loop=True), arms_a[-1], slider_t, (w, y_pin, z_top))
    return b.result


SCISSOR_SLIDER_MASS = 1.0


def scissor_actuator_force(arm_length: float, angle: float, stages: int, platform_mass: float, arm_mass: float,
                           slider_mass: float = SCISSOR_SLIDER_MASS, sides: int = 2, gravity: float = 9.81) -> float:
    """Static actuator force (per side) holding a scissor lift at arm ``angle`` (deg).

    Virtual work: ``F·dx = Σ m·g·dz`` with slider travel ``x = L·cos θ`` and pin height
    ``H = n·L·sin θ``.  The platform and top sliders rise with ``H``; the arms of stage ``k``
    rise with ``(k + ½)/n·H``, so all arms of one side weigh in as ``n·arm_mass``.  The
    actuator pushes the slider toward the fixed pin (negative X) with this magnitude.
    """
    n_sides = 1 if sides == 1 else 2
    weight = platform_mass + n_sides * (slider_mass + stages * arm_mass)
    return weight * gravity * stages / math.tan(math.radians(angle)) / n_sides


def scissor_height(arm_length: float, angle: float, stages: int) -> float:
    """Pin-to-pin height of a scissor stack for a given arm angle (deg)."""
    return stages * arm_length * math.sin(math.radians(angle))


def scissor_slider_offset(arm_length: float, angle0: float, angle: float) -> float:
    """Slider travel (along X) that turns a lift built at ``angle0`` to arm ``angle`` (deg)."""
    return arm_length * (math.cos(math.radians(angle)) - math.cos(math.radians(angle0)))


def scissor_slider_offset_for_height(arm_length: float, angle0: float, stages: int, height: float) -> float:
    """Prismatic target that raises a lift built at ``angle0`` to the pin height ``height``."""
    s = max(-1.0, min(1.0, height / (stages * arm_length)))
    return scissor_slider_offset(arm_length, angle0, math.degrees(math.asin(s)))


TEMPLATES = {
    "Pendulum chain": build_pendulum_chain,
    "Hydraulic cylinder": build_hydraulic_cylinder,
    "Hinged door": build_hinged_door,
    "Linear slider": build_linear_slider,
    "Scissor lift": build_scissor_lift,
}
