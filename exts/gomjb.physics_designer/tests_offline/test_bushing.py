"""Bushings in place of joints: spec, loop detection, stiffness sizing, conversion, both solvers."""
import math

import pytest
from pxr import Gf, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import bushing as bu
from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import mechanisms as me
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import transform as tf
from gomjb.physics_designer.core import units as un

G = 9.81


@pytest.fixture
def rig(stage):
    """World -fixed- Base (10 kg) -revolute- Link (1 kg); a free Box (2 kg) not connected."""
    ph.ensure_physics_scene(stage)
    UsdGeom.Xform.Define(stage, "/World/Mech")
    base = md.create_body(stage, "/World/Mech/Base", "box", (1.0, 0.4, 0.1), mass=10.0)
    link = md.create_body(stage, "/World/Mech/Link", "link", (0.8, 0.06, 0.02), mass=1.0)
    tf.set_world_position(link, (0.4, 0, 0.2))
    box = md.create_body(stage, "/World/Box", "box", (0.2, 0.2, 0.2), mass=2.0)
    tf.set_world_position(box, (3, 0, 0))
    jt.create_joint(stage, jt.JointSpec("fixed", name="BaseFixed"), None, base, (0, 0, 0))
    jt.create_joint(stage, jt.JointSpec("revolute", name="Hinge", axis="Y"), base, link, (0, 0, 0.2))
    return stage, base, link, box


def _drives(prim):
    return {d: UsdPhysics.DriveAPI(prim, d) for d in jt.D6_DOFS if prim.HasAPI(UsdPhysics.DriveAPI, d)}


def test_bushing_spec_frees_only_the_joint_axes():
    spec = jt.bushing_spec(["rotY"], 1e6, 500.0)
    assert spec.joint_type == "d6" and spec.exclude_from_articulation
    assert set(spec.d6_drives) == set(jt.D6_DOFS) - {"rotY"}
    assert spec.d6_drives["transX"].stiffness == 1e6 and spec.d6_drives["rotZ"].stiffness == 500.0
    assert spec.d6_drives["transX"].damping == pytest.approx(1e3)  # default: stiffness / 1000
    with pytest.raises(ValueError):
        jt.bushing_spec(["rotW"])


def test_free_axes_of_every_joint_type():
    assert bu.spec_free_axes(jt.JointSpec("fixed")) == []
    assert bu.spec_free_axes(jt.JointSpec("revolute", axis="Z")) == ["rotZ"]
    assert bu.spec_free_axes(jt.JointSpec("prismatic", axis="X")) == ["transX"]
    assert bu.spec_free_axes(jt.JointSpec("spherical")) == ["rotX", "rotY", "rotZ"]
    with pytest.raises(ValueError):
        bu.spec_free_axes(jt.JointSpec("distance"))


def test_loop_detection(rig):
    stage, base, link, box = rig
    assert bu.closes_loop(stage, base, link)  # a second base-link joint closes a loop
    assert bu.closes_loop(stage, None, link)  # link is grounded through Base: a world joint closes one
    assert not bu.closes_loop(stage, link, box)  # Box is not connected yet
    assert not bu.closes_loop(stage, None, box)
    hinge = stage.GetPrimAtPath("/World/Joints/Hinge")
    assert not bu.closes_loop(stage, base, link, ignore=[hinge])  # without itself the hinge is a tree joint


def test_auto_stiffness_formula(rig):
    stage, base, link, _ = rig
    k = bu.auto_stiffness(stage, base, link, deflection=1e-4, angle_deflection=0.01, damping_ratio=1.0)
    M = 11.0  # Base + Link: the mechanism; the unconnected Box does not count
    assert k.mass == pytest.approx(M)
    assert k.linear == pytest.approx(M * G / 1e-4)
    size = tf.combined_bbox([base, link]).GetSize().GetLength()
    assert k.size == pytest.approx(size)
    assert k.angular == pytest.approx(M * G * size / math.radians(0.01) * math.radians(1.0))  # per degree
    m_eff = 10.0 * 1.0 / 11.0
    assert k.linear_damping == pytest.approx(2 * math.sqrt(k.linear * m_eff))  # critical
    assert k.omega_dt == pytest.approx(math.sqrt(k.linear / m_eff) / 60.0)
    stiffer = bu.auto_stiffness(stage, base, link, deflection=1e-5)
    assert stiffer.linear == pytest.approx(10 * k.linear)


def test_auto_stiffness_is_unit_independent(stage):
    """The same mechanism on a metre and a centimetre stage gets the same SI stiffness."""
    from pxr import Usd

    out = []
    for mpu in (1.0, 0.01):
        s = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(s, mpu)
        s.SetDefaultPrim(UsdGeom.Xform.Define(s, "/World").GetPrim())
        res = me.build_scissor_lift(s, stages=2, sides=1, angle=30.0)
        pin = s.GetPrimAtPath(res["root"][0] + "/Joints/BasePin")
        assert jt.is_bushing(pin)
        out.append((UsdPhysics.DriveAPI(pin, "transX").GetStiffnessAttr().Get()
                    * un.si_per_stage_unit(s, "linear_stiffness"),
                    UsdPhysics.DriveAPI(pin, "rotX").GetStiffnessAttr().Get()
                    * un.si_per_stage_unit(s, "angular_stiffness")))
    assert out[1][0] == pytest.approx(out[0][0], rel=1e-6)
    assert out[1][1] == pytest.approx(out[0][1], rel=1e-6)


def test_create_joint_modes(rig):
    stage, base, link, box = rig
    rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()  # joint Z along world -Y
    rev = jt.JointSpec("revolute", axis="Z")
    # auto: a loop closure becomes a bushing with the same frame, free about the joint axis
    b = bu.create_joint(stage, rev, base, link, (0.8, 0, 0.2), rot)
    assert jt.is_bushing(b) and b.IsA(UsdPhysics.Joint) and not b.IsA(UsdPhysics.RevoluteJoint)
    assert set(_drives(b)) == set(jt.D6_DOFS) - {"rotZ"}
    assert UsdPhysics.Joint(b).GetExcludeFromArticulationAttr().Get()
    assert jt.joint_frame_error(b)[0] < 1e-6
    assert b.GetAttribute("designer:bushing:replaces").Get() == "revolute"
    assert "N/m" in b.GetAttribute("designer:bushing:sizing").Get()
    # auto: a tree joint stays a joint
    t = bu.create_joint(stage, rev, link, box, (1.0, 0, 0.2), rot)
    assert t.IsA(UsdPhysics.RevoluteJoint) and not jt.is_bushing(t)
    # a driven loop closure is an actuator: it stays a joint
    driven = jt.JointSpec("revolute", axis="Z", drive=jt.DriveSpec(stiffness=100.0))
    assert bu.create_joint(stage, driven, base, link, (0.8, 0, 0.2), rot).IsA(UsdPhysics.RevoluteJoint)
    # bushing mode: every undriven joint; rigid mode: none
    assert jt.is_bushing(bu.create_joint(stage, jt.JointSpec("fixed"), None, box, (3, 0, 0), mode="bushing"))
    assert not jt.is_bushing(bu.create_joint(stage, rev, base, link, (0.8, 0, 0.2), rot, mode="rigid"))
    with pytest.raises(ValueError):
        bu.create_joint(stage, rev, base, link, (0, 0, 0), mode="soft")


def test_convert_existing_joints(rig):
    stage, base, link, _ = rig
    loop = jt.create_joint(stage, jt.JointSpec("revolute", name="Loop", axis="Y"), base, link, (0.8, 0, 0.2))
    (p0, r0), _ = jt.joint_world_frames(loop)
    path = loop.GetPath()
    b = bu.convert(loop)
    assert b.GetPath() == path and jt.is_bushing(b)  # same path, so references to it survive
    assert "rotY" not in _drives(b) and len(_drives(b)) == 5
    (q0, s0), _ = jt.joint_world_frames(b)
    assert (q0 - p0).GetLength() < 1e-6
    assert [p.GetPath() for p in jt.joint_bodies(b)] == [base.GetPath(), link.GetPath()]
    assert bu.convert(b).GetPath() == path  # already a bushing: unchanged
    driven = jt.create_joint(stage, jt.JointSpec("revolute", axis="Y", drive=jt.DriveSpec(stiffness=5.0)),
                             base, link, (0, 0, 0.2))
    with pytest.raises(ValueError, match="driven"):
        bu.convert(driven)


def test_articulation_tree_joint_is_not_converted(stage):
    res = me.build_pendulum_chain(stage, links=2)
    hinge = stage.GetPrimAtPath(res["root"][0] + "/Joints/Hinge_01")
    with pytest.raises(ValueError, match="articulation tree joint"):
        bu.convert(hinge)


def test_scissor_bushings_physx_shape(stage):
    """Default scissor: loop closures are bushings, the tree and the actuators stay joints."""
    res = me.build_scissor_lift(stage, stages=2, sides=2)
    for p in res["joints"]:
        prim = stage.GetPrimAtPath(p)
        loop = bool(UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Get())
        assert jt.is_bushing(prim) == loop, p
        if "Actuator" in p:
            assert prim.IsA(UsdPhysics.PrismaticJoint) and not jt.is_bushing(prim)


def test_bushing_error_scales_with_its_compliance(stage):
    """The force error of a soft loop closure is its deflection: 10x stiffer, 10x smaller."""
    pytest.importorskip("exudyn")
    from pxr import Usd

    from gomjb.physics_designer.core.mbd import MbdModel

    errors = []
    for defl in (1e-4, 1e-5):
        s = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
        s.SetDefaultPrim(UsdGeom.Xform.Define(s, "/World").GetPrim())
        ph.ensure_physics_scene(s)
        res = me.build_scissor_lift(s, stages=3, sides=1, angle=30.0, loop_closure="rigid")
        for p in res["joints"]:
            if UsdPhysics.Joint(s.GetPrimAtPath(p)).GetExcludeFromArticulationAttr().Get():
                bu.convert(s.GetPrimAtPath(p), deflection=defl)
        model = MbdModel(s, res["root"][0])
        model.set_drive("Actuator", me.scissor_slider_offset(1.2, 30.0, 45.0))
        force = -model.solve_static(steps=5)["force:Actuator"]
        errors.append(force / me.scissor_actuator_force(1.2, 45.0, 3, 20.0, 5.0, sides=1) - 1)
    assert 0 < errors[0] < 1e-3
    assert errors[1] == pytest.approx(errors[0] / 10, rel=0.05)


@pytest.mark.parametrize("sides", [1, 2])
def test_exact_solver_reads_usd_bushings(stage, sides):
    """The exact solver maps a designer bushing to the same spring-damper: no redundant
    constraints left and the total actuator force within the bushing deflection of the analytic
    answer. (The left / right split is not symmetric: only the loop closures are soft while the
    articulation-tree pins stay rigid, and the two sides close different loops.)"""
    pytest.importorskip("exudyn")
    from gomjb.physics_designer.core.mbd import MbdModel

    ph.ensure_physics_scene(stage)
    res = me.build_scissor_lift(stage, stages=3, sides=sides, angle=30.0)
    model = MbdModel(stage, res["root"][0])
    assert model.has_bushings and model.redundant == 0 and not model.released
    assert "bushing joints" in model.report()
    for deg in (40.0, 55.0):
        target = me.scissor_slider_offset(1.2, 30.0, deg)
        for d in model.drive_names:
            model.set_drive(d, target)
        v = model.solve_static(steps=5)
        forces = [-v[f"force:{d}"] for d in model.drive_names]
        expected = me.scissor_actuator_force(1.2, deg, 3, 20.0, 5.0, sides=sides)
        assert sum(forces) == pytest.approx(expected * len(forces), rel=1e-3)
        assert all(f > 0 for f in forces)  # both actuators push toward the fixed pins
