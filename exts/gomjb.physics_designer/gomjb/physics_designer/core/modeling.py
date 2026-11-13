"""Primitive body modeler: physics-ready rigid bodies with analytic mass properties.

Follows the body conventions of multibody tools (RecurDyn / Adams): a body is a frame with
geometry, a material and *explicit* mass properties, and a "link" is a bar with rounded ends
and a pin marker (anchor) at each end.

USD layout of a body::

    <Body>            Xform: RigidBodyAPI + MassAPI (mass, centerOfMass, diagonalInertia),
                      designer:* parameters so the body can be edited later
      geom            implicit gprim (Cube / Cylinder / Sphere / Capsule / Cone), CollisionAPI
      endA, endB      link only: rounded ends (Cylinder)
      PinA, PinB      link only: anchors on the pin axes

The body frame carries no scale, so joint frames and anchors stay rigid; sizes live on the
geometry.  Mass properties are computed from the geometry (not left to the physics engine),
which also gives collider-less bodies a correct inertia.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from . import math_utils as mu
from . import transform as tf
from .snapping import create_anchor, is_anchor
from .stage_utils import si_factors

SHAPES: Dict[str, Tuple[str, ...]] = {
    "box": ("size x", "size y", "size z"),
    "cylinder": ("radius", "height"),
    "sphere": ("radius",),
    "capsule": ("radius", "height"),
    "cone": ("radius", "height"),
    "link": ("pin distance", "width", "thickness"),
}
BODY_TYPES = ("dynamic", "kinematic", "static", "visual")

# name: (density kg/m^3, display color)
MATERIALS: Dict[str, Tuple[float, Gf.Vec3f]] = {
    "steel": (7850.0, Gf.Vec3f(0.62, 0.64, 0.68)),
    "stainless": (8000.0, Gf.Vec3f(0.78, 0.79, 0.80)),
    "aluminum": (2700.0, Gf.Vec3f(0.80, 0.82, 0.86)),
    "cast_iron": (7200.0, Gf.Vec3f(0.35, 0.35, 0.37)),
    "brass": (8500.0, Gf.Vec3f(0.80, 0.65, 0.30)),
    "abs_plastic": (1050.0, Gf.Vec3f(0.95, 0.55, 0.10)),
    "wood": (600.0, Gf.Vec3f(0.62, 0.45, 0.28)),
    "rubber": (1100.0, Gf.Vec3f(0.12, 0.12, 0.12)),
}

ATTR_SHAPE = "designer:shape"
ATTR_DIMS = "designer:dims"
ATTR_MATERIAL = "designer:material"
ATTR_DENSITY = "designer:density"
ATTR_MASS = "designer:mass"  # 0 = from density
ATTR_OFFSET = "designer:geomOffset"
ATTR_TYPE = "designer:bodyType"

_GEOM_CHILDREN = ("geom", "endA", "endB")


@dataclass
class MassProps:
    """Mass properties about the centre of mass, principal axes = body axes."""

    mass: float
    com: Gf.Vec3d
    inertia: Gf.Vec3d  # Ixx, Iyy, Izz

    def scaled_to(self, mass: float) -> "MassProps":
        k = mass / self.mass
        return MassProps(mass, Gf.Vec3d(self.com), Gf.Vec3d(self.inertia) * k)


# ---------------------------------------------------------------------------------------
# Mass properties (geometry frame; cylinders, capsules and cones along Z, links along X)
# ---------------------------------------------------------------------------------------
def _check(shape: str, dims: Sequence[float]) -> Tuple[float, ...]:
    if shape not in SHAPES:
        raise ValueError(f"Unknown shape '{shape}', expected one of {list(SHAPES)}")
    need = len(SHAPES[shape])
    if len(dims) < need:
        raise ValueError(f"{shape} needs {need} dimensions {SHAPES[shape]}")
    d = tuple(float(v) for v in dims[:need])
    if any(v <= 0 for v in d):
        raise ValueError(f"{shape} dimensions must be positive, got {d}")
    return d


def mass_properties(shape: str, dims: Sequence[float], density: float) -> MassProps:
    """Analytic mass, centre of mass and principal inertia of a homogeneous primitive."""
    d = _check(shape, dims)
    rho = float(density)
    if shape == "box":
        a, b, c = d
        m = rho * a * b * c
        return MassProps(m, Gf.Vec3d(0), Gf.Vec3d(m * (b * b + c * c), m * (a * a + c * c), m * (a * a + b * b)) / 12)
    if shape == "sphere":
        r = d[0]
        m = rho * 4.0 / 3.0 * math.pi * r ** 3
        i = 0.4 * m * r * r
        return MassProps(m, Gf.Vec3d(0), Gf.Vec3d(i, i, i))
    if shape == "cylinder":
        r, h = d
        m = rho * math.pi * r * r * h
        ix = m * (3 * r * r + h * h) / 12
        return MassProps(m, Gf.Vec3d(0), Gf.Vec3d(ix, ix, m * r * r / 2))
    if shape == "cone":  # USD Cone: base at -h/2, apex at +h/2
        r, h = d
        m = rho * math.pi * r * r * h / 3
        ix = m * (3.0 / 20.0 * r * r + 3.0 / 80.0 * h * h)
        return MassProps(m, Gf.Vec3d(0, 0, -h / 4), Gf.Vec3d(ix, ix, 0.3 * m * r * r))
    if shape == "capsule":  # USD Capsule: ``height`` is the cylindrical part
        r, h = d
        mc = rho * math.pi * r * r * h
        mh = rho * 2.0 / 3.0 * math.pi * r ** 3  # one hemisphere
        dz = 3.0 * r / 8.0  # hemisphere COM from its flat face
        ix = mc * (3 * r * r + h * h) / 12 + 2 * mh * (0.4 * r * r - dz * dz + (h / 2 + dz) ** 2)
        iz = mc * r * r / 2 + 2 * mh * 0.4 * r * r
        return MassProps(mc + 2 * mh, Gf.Vec3d(0), Gf.Vec3d(ix, ix, iz))
    # link: stadium of pin distance L (X) and width w (Z), thickness t (Y); pins on the Y axis
    L, w, t = d
    r = w / 2
    mb = rho * L * w * t
    mh = rho * math.pi * r * r * t / 2  # one half-disc end
    dc = 4 * r / (3 * math.pi)  # half-disc COM from its flat edge
    off = L / 2 + dc
    ix = mb * (t * t + w * w) / 12 + 2 * mh * (r * r / 4 + t * t / 12)
    iy = mb * (L * L + w * w) / 12 + 2 * mh * (r * r / 2 - dc * dc + off * off)
    iz = mb * (L * L + t * t) / 12 + 2 * mh * (r * r / 4 + t * t / 12 - dc * dc + off * off)
    return MassProps(mb + 2 * mh, Gf.Vec3d(0), Gf.Vec3d(ix, iy, iz))


def link_pin_positions(dims: Sequence[float]) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    L = _check("link", dims)[0]
    return Gf.Vec3d(-L / 2, 0, 0), Gf.Vec3d(L / 2, 0, 0)


# ---------------------------------------------------------------------------------------
# Bodies
# ---------------------------------------------------------------------------------------
def create_body(stage: Usd.Stage, path, shape: str, dims: Sequence[float], *, material: str = "steel",
                density: Optional[float] = None, mass: Optional[float] = None, body_type: str = "dynamic",
                collision: bool = True, color: Optional[Gf.Vec3f] = None,
                offset: mu.VecLike = (0, 0, 0)) -> Usd.Prim:
    """Create a body at ``path`` (identity pose — place it with :mod:`transform`).

    ``dims`` are in stage units; ``density`` (kg/m^3) and ``mass`` (kg) are SI and converted
    with the stage's metersPerUnit / kilogramsPerUnit, so the authored MassAPI values are in
    stage units as USD Physics expects.
    ``density`` overrides the material's; ``mass`` fixes the mass and scales the inertia of
    the uniform-density geometry to it.  ``offset`` shifts the geometry inside the body frame
    (e.g. to put the body origin on a pivot).  ``body_type``: dynamic, kinematic, static
    (collider only) or visual (no physics).  ``collision=False`` keeps a dynamic body
    collider-less (it still has correct mass properties).
    """
    _check(shape, dims)
    if material not in MATERIALS:
        raise ValueError(f"Unknown material '{material}', expected one of {list(MATERIALS)}")
    if body_type not in BODY_TYPES:
        raise ValueError(f"body_type must be one of {BODY_TYPES}")
    prim = UsdGeom.Xform.Define(stage, Sdf.Path(str(path))).GetPrim()
    _attr(prim, ATTR_SHAPE, Sdf.ValueTypeNames.Token, shape)
    _attr(prim, ATTR_MATERIAL, Sdf.ValueTypeNames.Token, material)
    _attr(prim, ATTR_DENSITY, Sdf.ValueTypeNames.Double, float(density or 0.0))
    _attr(prim, ATTR_MASS, Sdf.ValueTypeNames.Double, float(mass or 0.0))
    _attr(prim, ATTR_OFFSET, Sdf.ValueTypeNames.Double3, Gf.Vec3d(*mu.vec3d(offset)))
    _attr(prim, ATTR_TYPE, Sdf.ValueTypeNames.Token, body_type)
    if body_type in ("dynamic", "kinematic"):
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(body_type == "kinematic")
    _build(prim, list(dims), color, collision and body_type != "visual")
    return prim


def is_designer_body(prim: Usd.Prim) -> bool:
    return bool(prim and prim.GetAttribute(ATTR_SHAPE) and prim.GetAttribute(ATTR_SHAPE).Get())


def body_params(prim: Usd.Prim) -> dict:
    """The stored modeling parameters of a body created by :func:`create_body`."""
    if not is_designer_body(prim):
        raise ValueError(f"{prim.GetPath()} was not created by the modeler")
    return {
        "shape": prim.GetAttribute(ATTR_SHAPE).Get(),
        "dims": list(prim.GetAttribute(ATTR_DIMS).Get()),
        "material": prim.GetAttribute(ATTR_MATERIAL).Get(),
        "density": prim.GetAttribute(ATTR_DENSITY).Get() or None,
        "mass": prim.GetAttribute(ATTR_MASS).Get() or None,
        "body_type": prim.GetAttribute(ATTR_TYPE).Get(),
    }


def update_body(prim: Usd.Prim, dims: Optional[Sequence[float]] = None, material: Optional[str] = None,
                density: Optional[float] = None, mass: Optional[float] = None) -> MassProps:
    """Change dimensions / material / mass of a modeler body; geometry, mass properties and
    link pin anchors follow.  ``density`` / ``mass`` of 0 mean "use the material density".

    Joints keep their body-local frames: after moving link pins, re-align attached joints.
    """
    p = body_params(prim)
    if material is not None:
        if material not in MATERIALS:
            raise ValueError(f"Unknown material '{material}'")
        prim.GetAttribute(ATTR_MATERIAL).Set(material)
    if density is not None:
        prim.GetAttribute(ATTR_DENSITY).Set(float(density))
    if mass is not None:
        prim.GetAttribute(ATTR_MASS).Set(float(mass))
    colliding = any(c.HasAPI(UsdPhysics.CollisionAPI) for c in prim.GetChildren() if c.GetName() in _GEOM_CHILDREN)
    color = None if material is not None else _current_color(prim)
    for name in _GEOM_CHILDREN:
        if prim.GetChild(name):
            prim.GetStage().RemovePrim(prim.GetPath().AppendChild(name))
    return _build(prim, list(dims) if dims is not None else p["dims"], color, colliding)


def body_mass_properties(prim: Usd.Prim, si: bool = False) -> MassProps:
    """Mass properties as authored on the body (body frame, stage units; ``si``: kg, m, kg*m^2)."""
    api = UsdPhysics.MassAPI(prim)
    props = MassProps(float(api.GetMassAttr().Get()), Gf.Vec3d(api.GetCenterOfMassAttr().Get()),
                      Gf.Vec3d(api.GetDiagonalInertiaAttr().Get()))
    if not si:
        return props
    f = si_factors(prim.GetStage())
    return MassProps(props.mass * f["mass"], props.com * f["length"], props.inertia * f["mass"] * f["length"] ** 2)


# ---------------------------------------------------------------------------------------
def _attr(prim, name, type_name, value):
    prim.CreateAttribute(name, type_name, custom=True).Set(value)


def _current_color(prim: Usd.Prim) -> Optional[Gf.Vec3f]:
    geom = prim.GetChild("geom")
    if geom:
        colors = UsdGeom.Gprim(geom).GetDisplayColorAttr().Get()
        if colors:
            return Gf.Vec3f(colors[0])
    return None


def _build(prim: Usd.Prim, dims, color, collision: bool) -> MassProps:
    """(Re)author geometry, collision and mass properties from the stored parameters."""
    stage = prim.GetStage()
    shape = prim.GetAttribute(ATTR_SHAPE).Get()
    d = _check(shape, dims)
    prim.CreateAttribute(ATTR_DIMS, Sdf.ValueTypeNames.DoubleArray, custom=True).Set(list(d))
    material = prim.GetAttribute(ATTR_MATERIAL).Get()
    density_si = prim.GetAttribute(ATTR_DENSITY).Get() or MATERIALS[material][0]  # kg/m^3
    density = density_si / si_factors(stage)["density"]  # stage mass units per stage length unit^3
    offset = Gf.Vec3d(prim.GetAttribute(ATTR_OFFSET).Get())
    color = color if color is not None else MATERIALS[material][1]
    base = prim.GetPath()

    def place(gprim, translate, scale=None):
        x = UsdGeom.Xformable(gprim)
        x.AddTranslateOp().Set(Gf.Vec3d(translate))
        if scale is not None:
            x.AddScaleOp().Set(Gf.Vec3d(scale))
        gprim.CreateDisplayColorAttr([color])
        return gprim.GetPrim()

    geoms = []
    if shape == "box":
        g = UsdGeom.Cube.Define(stage, base.AppendChild("geom"))
        g.CreateSizeAttr(1.0)
        g.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        geoms.append(place(g, offset, d))
    elif shape == "sphere":
        g = UsdGeom.Sphere.Define(stage, base.AppendChild("geom"))
        g.CreateRadiusAttr(d[0])
        g.CreateExtentAttr([Gf.Vec3f(-d[0]), Gf.Vec3f(d[0])])
        geoms.append(place(g, offset))
    elif shape in ("cylinder", "capsule", "cone"):
        r, h = d
        schema = {"cylinder": UsdGeom.Cylinder, "capsule": UsdGeom.Capsule, "cone": UsdGeom.Cone}[shape]
        g = schema.Define(stage, base.AppendChild("geom"))
        g.CreateRadiusAttr(r)
        g.CreateHeightAttr(h)
        g.CreateAxisAttr("Z")
        half = h / 2 + (r if shape == "capsule" else 0)
        g.CreateExtentAttr([Gf.Vec3f(-r, -r, -half), Gf.Vec3f(r, r, half)])
        geoms.append(place(g, offset))
    else:  # link
        L, w, t = d
        g = UsdGeom.Cube.Define(stage, base.AppendChild("geom"))
        g.CreateSizeAttr(1.0)
        g.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
        geoms.append(place(g, offset, (L, t, w)))
        for name, x in (("endA", -L / 2), ("endB", L / 2)):
            c = UsdGeom.Cylinder.Define(stage, base.AppendChild(name))
            c.CreateRadiusAttr(w / 2)
            c.CreateHeightAttr(t)
            c.CreateAxisAttr("Y")
            c.CreateExtentAttr([Gf.Vec3f(-w / 2, -t / 2, -w / 2), Gf.Vec3f(w / 2, t / 2, w / 2)])
            geoms.append(place(c, offset + Gf.Vec3d(x, 0, 0)))
        _place_link_pins(prim, d, offset)

    if collision:
        for gp in geoms:
            UsdPhysics.CollisionAPI.Apply(gp)

    props = mass_properties(shape, d, density)
    fixed = prim.GetAttribute(ATTR_MASS).Get()  # kg
    if fixed:
        props = props.scaled_to(float(fixed) / si_factors(stage)["mass"])
    props = MassProps(props.mass, props.com + offset, props.inertia)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr().Set(float(props.mass))
        api.CreateCenterOfMassAttr().Set(Gf.Vec3f(props.com))
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(props.inertia))
        api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
    return props


def _place_link_pins(prim: Usd.Prim, dims, offset: Gf.Vec3d) -> None:
    """Anchors on both pin axes; their Z axis is the pin axis (body Y) for revolute joints."""
    pin_rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)  # local Z -> body Y
    body_world = tf.world_matrix(prim)
    for name, local in zip(("PinA", "PinB"), link_pin_positions(dims)):
        frame = mu.make_transform(local + offset, pin_rot) * mu.rigid_part(body_world)
        anchor = prim.GetChild(name)
        if anchor and is_anchor(anchor):
            tf.set_world_matrix(anchor, frame)
        else:
            create_anchor(prim.GetStage(), prim, name, frame.ExtractTranslation(),
                          mu.rotation_of(frame).GetQuat(), kind="pin", marker_radius=dims[1] * 0.15)
