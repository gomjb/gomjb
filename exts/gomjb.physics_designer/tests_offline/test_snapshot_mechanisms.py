
import math

import pytest
from pxr import UsdPhysics

from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import mechanisms as me
from gomjb.physics_designer.core import snapshot as ss
from gomjb.physics_designer.core import transform as tf
from gomjb.physics_designer.core import validation as va

from conftest import make_cube


def test_snapshot_undo_redo(stage):
    a = make_cube(stage, "/World/A")
    b = make_cube(stage, "/World/B", pos=(2, 0, 0))
    make_cube(stage, "/World/Z", pos=(9, 0, 0))
    layer = stage.GetRootLayer()
    before = layer.ExportToString()

    snap = ss.EditSnapshot(layer, ["/World/A", "/World/B"])
    tf.translate([a, b], (0, 0, 5))
    UsdPhysics.RigidBodyAPI.Apply(a)
    joint = jt.create_joint(stage, jt.JointSpec("fixed"), a, b, (1, 0, 5))
    snap.finish()
    after = layer.ExportToString()
    assert any(str(p) == "/World/Joints" for p in snap.created_paths)

    snap.undo()
    assert layer.ExportToString() == before
    assert not stage.GetPrimAtPath(joint.GetPath())
    assert [c.GetName() for c in stage.GetPrimAtPath("/World").GetChildren()] == ["A", "B", "Z"]

    snap.redo()
    assert layer.ExportToString() == after
    snap.undo()
    assert layer.ExportToString() == before


@pytest.mark.parametrize("name", list(me.TEMPLATES))
def test_templates_have_consistent_joints(stage, name):
    result = me.TEMPLATES[name](stage)
    assert result["bodies"] and result["joints"]
    for jp in result["joints"]:
        dp, da = jt.joint_frame_error(stage.GetPrimAtPath(jp))
        assert dp < 1e-5 and da < 1e-3, jp
    errors = [i for i in va.validate(stage) if i.severity == va.ERROR or "articulation" in i.message]
    assert not errors, va.format_report(errors)


def test_scissor_geometry(stage):
    res = me.build_scissor_lift(stage, stages=2, arm_length=1.0, angle=30, sides=1)
    assert len(res["joints"]) == 1 + 2 * 1 + 2 * 1 + 3 + 3  # fixed + centers + links + bottom + top
    platform = stage.GetPrimAtPath(res["root"][0] + "/Platform")
    expected_pin_height = me.scissor_height(1.0, 30, 2)
    assert expected_pin_height == pytest.approx(1.0)
    pin = stage.GetPrimAtPath(res["root"][0] + "/Joints/PlatformPin")
    (p0, _), _ = jt.joint_world_frames(pin)
    base_pin = stage.GetPrimAtPath(res["root"][0] + "/Joints/BasePin")
    (q0, _), _ = jt.joint_world_frames(base_pin)
    assert p0[2] - q0[2] == pytest.approx(expected_pin_height)
    assert platform
    target = me.scissor_slider_offset_for_height(1.0, 30, 2, 1.4)
    assert target < 0  # slider moves toward the fixed pin to raise
    assert me.scissor_slider_offset_for_height(1.0, 30, 2, 1.0) == pytest.approx(0.0, abs=1e-12)
    # the slider sits one full span (L·cos) away from the fixed base pin
    slider_pin = stage.GetPrimAtPath(res["root"][0] + "/Joints/BaseSliderPin")
    (s0, _), _ = jt.joint_world_frames(slider_pin)
    span = s0[0] - q0[0]
    assert span == pytest.approx(math.cos(math.radians(30)))
    assert span + me.scissor_slider_offset(1.0, 30, 50) == pytest.approx(math.cos(math.radians(50)))


@pytest.mark.parametrize("sides", [1, 2])
def test_scissor_articulation_is_a_parent_child_tree(stage, sides):
    """Non-excluded joints must form a tree rooted at the world that reaches every body once,
    with body0 = parent and body1 = child on every tree joint (Newton rejects reversed joints)."""
    res = me.build_scissor_lift(stage, stages=3, sides=sides)
    assert stage.GetPrimAtPath(res["root"][0]).HasAPI(UsdPhysics.ArticulationRootAPI)
    edges = []
    for jp in res["joints"]:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(jp))
        if joint.GetExcludeFromArticulationAttr().Get():
            continue
        b0, b1 = (str(r.GetTargets()[0]) if r.GetTargets() else "world"
                  for r in (joint.GetBody0Rel(), joint.GetBody1Rel()))
        edges.append((jp, b0, b1))
    assert len(edges) == len(res["bodies"])  # a tree over world + bodies has exactly |bodies| edges
    depth = {"world": 0}
    frontier = ["world"]
    while frontier:
        nxt = []
        for node in frontier:
            for jp, b0, b1 in edges:
                if node in (b0, b1):
                    other = b1 if node == b0 else b0
                    if other not in depth:
                        assert other == b1, f"{jp}: body0 must be the parent"
                        depth[other] = depth[node] + 1
                        nxt.append(other)
        frontier = nxt
    assert set(depth) == set(res["bodies"]) | {"world"}
    # Newton's MuJoCo solver closes loops with point constraints: only revolute loop joints map
    for jp in res["joints"]:
        prim = stage.GetPrimAtPath(jp)
        if UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Get():
            assert prim.IsA(UsdPhysics.RevoluteJoint), f"{jp}: loop closure must be revolute"


def test_cylinder_direction(stage):
    res = me.build_hydraulic_cylinder(stage, origin=(1, 0, 0), direction=(1, 0, 0), retracted_length=0.5)
    rod = stage.GetPrimAtPath(res["root"][0] + "/Rod")
    assert tuple(tf.world_position(rod)) == pytest.approx((1 + 0.55, 0, 0))
    stroke = UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(res["joints"][-1]))
    assert stroke.GetUpperLimitAttr().Get() == pytest.approx(0.4)
    assert len(res["anchors"]) == 2


def test_y_up_stage(stage):
    from pxr import UsdGeom
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    res = me.build_linear_slider(stage)
    carriage = stage.GetPrimAtPath(res["root"][0] + "/Carriage")
    assert tf.world_position(carriage)[1] > 0
    assert abs(tf.world_position(carriage)[2]) < 1e-9
