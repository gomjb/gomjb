"""Templates take SI and have the same physical size on any stage scale (fixed internal sizes too)."""
import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import mechanisms as me
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import transform as tf
from gomjb.physics_designer.core import units as un


def _stage(meters_per_unit):
    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, meters_per_unit)
    s.SetDefaultPrim(UsdGeom.Xform.Define(s, "/World").GetPrim())
    ph.ensure_physics_scene(s)
    return s


def _si(stage, value, quantity):
    return value * un.si_per_stage_unit(stage, quantity)


@pytest.mark.parametrize("template", list(me.TEMPLATES))
def test_template_is_the_same_on_a_centimetre_stage(template):
    m, cm = _stage(1.0), _stage(0.01)
    res_m, res_cm = (me.TEMPLATES[template](s, origin=(0.3, -0.2, 0.1)) for s in (m, cm))
    assert res_m["bodies"] == res_cm["bodies"] and res_m["joints"] == res_cm["joints"]
    for path in res_m["bodies"]:
        a, b = m.GetPrimAtPath(path), cm.GetPrimAtPath(path)
        assert list(tf.world_position(b)) == pytest.approx([100 * v for v in tf.world_position(a)], abs=1e-6)
        assert md.body_params(b)["dims"] == pytest.approx([100 * v for v in md.body_params(a)["dims"]])
        pa, pb = md.body_mass_properties(a, si=True), md.body_mass_properties(b, si=True)
        assert pb.mass == pytest.approx(pa.mass) and list(pb.inertia) == pytest.approx(list(pa.inertia), rel=1e-5)
    for path in res_m["joints"]:
        a, b = m.GetPrimAtPath(path), cm.GetPrimAtPath(path)
        ja, jb = UsdPhysics.Joint(a), UsdPhysics.Joint(b)
        assert list(jb.GetLocalPos0Attr().Get()) == pytest.approx([100 * v for v in ja.GetLocalPos0Attr().Get()],
                                                                  abs=1e-4)
        linear = a.IsA(UsdPhysics.PrismaticJoint)
        if linear or a.IsA(UsdPhysics.RevoluteJoint):
            schema = UsdPhysics.PrismaticJoint if linear else UsdPhysics.RevoluteJoint
            for get in ("GetLowerLimitAttr", "GetUpperLimitAttr"):
                va, vb = getattr(schema(a), get)().Get(), getattr(schema(b), get)().Get()
                if va is not None:
                    q = "length" if linear else "angle"
                    assert _si(cm, vb, q) == pytest.approx(_si(m, va, q), rel=1e-5)
        drive = "linear" if linear else "angular"
        if a.HasAPI(UsdPhysics.DriveAPI, drive):
            da, db = UsdPhysics.DriveAPI(a, drive), UsdPhysics.DriveAPI(b, drive)
            for get, lin, ang in (("GetStiffnessAttr", "linear_stiffness", "angular_stiffness"),
                                  ("GetDampingAttr", "linear_damping", "angular_damping")):
                q = lin if linear else ang
                va, vb = getattr(da, get)().Get(), getattr(db, get)().Get()
                assert _si(cm, vb, q) == pytest.approx(_si(m, va, q), rel=1e-5), f"{path} {get}"


def test_template_parameters_are_si_on_a_millimetre_stage():
    mm = _stage(0.001)
    res = me.build_linear_slider(mm, rail_length=1.0)
    rail = mm.GetPrimAtPath(res["bodies"][0])
    assert md.body_params(rail)["dims"][0] == pytest.approx(1000.0)  # 1 m rail = 1000 stage units
