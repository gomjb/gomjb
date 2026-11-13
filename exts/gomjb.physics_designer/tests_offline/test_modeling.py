import math

import pytest
from pxr import Gf, UsdPhysics

from gomjb.physics_designer.core import math_utils as mu
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import snapping as sn
from gomjb.physics_designer.core import transform as tf

# Independent check of the analytic formulas: point-sample each solid on a grid.
INSIDE = {
    "box": lambda d, x, y, z: abs(x) <= d[0] / 2 and abs(y) <= d[1] / 2 and abs(z) <= d[2] / 2,
    "sphere": lambda d, x, y, z: x * x + y * y + z * z <= d[0] ** 2,
    "cylinder": lambda d, x, y, z: x * x + y * y <= d[0] ** 2 and abs(z) <= d[1] / 2,
    "cone": lambda d, x, y, z: abs(z) <= d[1] / 2 and math.hypot(x, y) <= d[0] * (d[1] / 2 - z) / d[1],
    "capsule": lambda d, x, y, z: (x * x + y * y + max(0.0, abs(z) - d[1] / 2) ** 2) <= d[0] ** 2,
    "link": lambda d, x, y, z: abs(y) <= d[2] / 2 and (max(0.0, abs(x) - d[0] / 2) ** 2 + z * z) <= (d[1] / 2) ** 2,
}
BOUNDS = {
    "box": lambda d: (d[0] / 2, d[1] / 2, d[2] / 2),
    "sphere": lambda d: (d[0],) * 3,
    "cylinder": lambda d: (d[0], d[0], d[1] / 2),
    "cone": lambda d: (d[0], d[0], d[1] / 2),
    "capsule": lambda d: (d[0], d[0], d[1] / 2 + d[0]),
    "link": lambda d: (d[0] / 2 + d[1] / 2, d[2] / 2, d[1] / 2),
}
DIMS = {"box": (0.4, 0.2, 0.1), "sphere": (0.15,), "cylinder": (0.1, 0.5), "cone": (0.2, 0.3),
        "capsule": (0.05, 0.3), "link": (0.6, 0.08, 0.02)}


def numeric_props(shape, d, n=48):
    bx, by, bz = BOUNDS[shape](d)
    hx, hy, hz = 2 * bx / n, 2 * by / n, 2 * bz / n
    dv = hx * hy * hz
    m = sx = sy = sz = sxx = syy = szz = 0.0
    inside = INSIDE[shape]
    for i in range(n):
        x = -bx + (i + 0.5) * hx
        for j in range(n):
            y = -by + (j + 0.5) * hy
            for k in range(n):
                z = -bz + (k + 0.5) * hz
                if inside(d, x, y, z):
                    m += dv
                    sx += x * dv
                    sy += y * dv
                    sz += z * dv
                    sxx += x * x * dv
                    syy += y * y * dv
                    szz += z * z * dv
    cx, cy, cz = sx / m, sy / m, sz / m
    ixx = (syy - m * cy * cy) + (szz - m * cz * cz)
    iyy = (sxx - m * cx * cx) + (szz - m * cz * cz)
    izz = (sxx - m * cx * cx) + (syy - m * cy * cy)
    return m, (cx, cy, cz), (ixx, iyy, izz)


@pytest.mark.parametrize("shape", list(md.SHAPES))
def test_mass_properties_match_numeric_integration(shape):
    d = DIMS[shape]
    props = md.mass_properties(shape, d, density=1.0)
    m, com, inertia = numeric_props(shape, d)
    assert props.mass == pytest.approx(m, rel=0.02)
    assert tuple(props.com) == pytest.approx(com, abs=0.01 * max(d))
    assert tuple(props.inertia) == pytest.approx(inertia, rel=0.03)


def test_density_and_material(stage):
    body = md.create_body(stage, "/World/Plate", "box", (1.0, 0.5, 0.02), material="aluminum")
    props = md.body_mass_properties(body)
    assert props.mass == pytest.approx(2700 * 1.0 * 0.5 * 0.02)
    md.update_body(body, density=1000.0)
    assert md.body_mass_properties(body).mass == pytest.approx(10.0)
    md.update_body(body, density=0.0, material="steel")
    assert md.body_mass_properties(body).mass == pytest.approx(7850 * 0.01)


def test_fixed_mass_scales_inertia(stage):
    body = md.create_body(stage, "/World/Rod", "cylinder", (0.02, 1.0), mass=3.0, offset=(0, 0, -0.5))
    props = md.body_mass_properties(body)
    assert props.mass == pytest.approx(3.0)
    assert props.inertia[0] == pytest.approx(3.0 * (3 * 0.02 ** 2 + 1.0) / 12, rel=1e-5)
    assert tuple(props.com) == pytest.approx((0, 0, -0.5))  # follows the geometry offset


@pytest.mark.parametrize("body_type,rigid,kinematic,collider", [
    ("dynamic", True, False, True), ("kinematic", True, True, True),
    ("static", False, None, True), ("visual", False, None, False)])
def test_body_types(stage, body_type, rigid, kinematic, collider):
    body = md.create_body(stage, "/World/B", "sphere", (0.1,), body_type=body_type)
    assert body.HasAPI(UsdPhysics.RigidBodyAPI) == rigid
    if rigid:
        assert UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get() == kinematic
        assert body.HasAPI(UsdPhysics.MassAPI)
    assert body.GetChild("geom").HasAPI(UsdPhysics.CollisionAPI) == collider


def test_link_pins_follow_resize_and_pose(stage):
    link = md.create_body(stage, "/World/Arm", "link", (0.8, 0.06, 0.02))
    tf.set_world_pose(link, (1, 2, 3), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
    pin_a, pin_b = link.GetChild("PinA"), link.GetChild("PinB")
    assert sn.is_anchor(pin_a) and sn.is_anchor(pin_b)
    assert mu.is_close(tf.world_position(pin_a), (1, 2 - 0.4, 3), 1e-9)
    assert mu.is_close(tf.world_position(pin_b), (1, 2 + 0.4, 3), 1e-9)
    # anchor Z = pin axis = body Y, which the 90 deg yaw turns into world -X
    z_axis = mu.rotation_of(tf.world_matrix(pin_a)).TransformDir(Gf.Vec3d(0, 0, 1))
    assert mu.is_close(z_axis, (-1, 0, 0), 1e-9)
    m0 = md.body_mass_properties(link).mass
    md.update_body(link, dims=(1.2, 0.06, 0.02))
    assert mu.is_close(tf.world_position(link.GetChild("PinB")), (1, 2 + 0.6, 3), 1e-9)
    assert link.GetChild("PinB").GetPath() == pin_b.GetPath()  # anchors are kept, not recreated
    assert md.body_mass_properties(link).mass > m0
    assert md.body_params(link)["dims"] == pytest.approx([1.2, 0.06, 0.02])


def test_invalid_input(stage):
    with pytest.raises(ValueError):
        md.create_body(stage, "/World/X", "torus", (1, 1))
    with pytest.raises(ValueError):
        md.create_body(stage, "/World/X", "box", (1, 0, 1))
    with pytest.raises(ValueError):
        md.create_body(stage, "/World/X", "box", (1, 1, 1), material="unobtainium")


def test_centimetre_stage_gives_the_same_physical_body(stage):
    """dims are stage units, density/mass are SI: a 50x30x20 cm steel box weighs 235.5 kg either way."""
    from pxr import Usd, UsdGeom
    from gomjb.physics_designer.core.stage_utils import si_factors

    cm = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(cm, 0.01)
    assert si_factors(cm) == pytest.approx({"length": 0.01, "mass": 1.0, "force": 0.01, "torque": 1e-4,
                                            "density": 1e6})
    box_cm = md.create_body(cm, "/Box", "box", (50, 30, 20))
    box_m = md.create_body(stage, "/World/Box", "box", (0.5, 0.3, 0.2))
    a, b = md.body_mass_properties(box_cm, si=True), md.body_mass_properties(box_m, si=True)
    assert a.mass == pytest.approx(235.5) and b.mass == pytest.approx(235.5)
    assert tuple(a.inertia) == pytest.approx(tuple(b.inertia))
    # authored values are stage units: kg*cm^2 on the centimetre stage
    assert md.body_mass_properties(box_cm).inertia[0] == pytest.approx(b.inertia[0] * 1e4)
    fixed = md.create_body(cm, "/Rod", "cylinder", (2, 100), mass=3.0)
    assert md.body_mass_properties(fixed, si=True).mass == pytest.approx(3.0)
