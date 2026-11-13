"""Joint motions (RecurDyn "Include Motion"): expressions, profiles, storage (the exact solver's run of them: gomjb.kinematics)."""
import math

import pytest
from pxr import UsdGeom, UsdPhysics

from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import motion as mo
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import transform as tf
from gomjb.physics_designer.core.expressions import compile_expression, step


def test_expressions():
    f = compile_expression("STEP(TIME, 0, 0, 1, 90)")
    assert (f(0), f(0.5), f(1), f(2)) == (0, 45, 90, 90)
    assert step(0.25, 0, 0, 1, 1) == pytest.approx(0.15625)  # a^2 (3 - 2a)
    assert compile_expression("30*sin(2*pi*time)")(0.25) == pytest.approx(30)  # case insensitive
    assert compile_expression("2 + 3 * TIME ** 2")(2) == 14
    for bad in ("__import__('os')", "TIME.real", "'a'", "open", "lambda: 1", "STEP(TIME)", "1/0", "[1]"):
        with pytest.raises(ValueError):
            compile_expression(bad)


def test_profiles_integrate_velocity_and_acceleration():
    disp = mo.Profile(mo.Motion("displacement", "STEP(TIME, 0, 0, 1, 90)"), linear=False)
    assert disp.value(1.0) == 90 and disp.velocity(0.5) == pytest.approx(135, rel=1e-6)  # 1.5 * 90 / s
    vel = mo.Profile(mo.Motion("velocity", "90", initial_displacement=10), linear=False)
    assert vel.value(0.0) == 10 and vel.value(1.0) == pytest.approx(100) and vel.velocity(0.3) == 90
    acc = mo.Profile(mo.Motion("acceleration", "2", initial_displacement=1, initial_velocity=3), linear=False)
    assert acc.value(2.0) == pytest.approx(1 + 3 * 2 + 0.5 * 2 * 4) and acc.velocity(2.0) == pytest.approx(7)
    assert acc.value(1.0) == pytest.approx(1 + 3 + 1)  # going back in time restarts the integration
    lin = mo.Profile(mo.Motion("displacement", "100", units="MMKS"), linear=True)
    assert lin.value(0.0) == pytest.approx(0.1)  # 100 mm in metres


@pytest.fixture
def hinge(stage):
    """A 2 kg, 1 m link hinged to the world at its left end, horizontal along +X; hinge axis Y."""
    ph.ensure_physics_scene(stage)
    root = UsdGeom.Xform.Define(stage, "/World/Arm").GetPrim()
    ph.apply_articulation_root(root)
    link = md.create_body(stage, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (0.5, 0, 0))
    joint = jt.create_joint_at(stage, jt.JointSpec("revolute", name="Hinge", axis="Z"), None, link,
                               location="custom", custom_position=(0, 0, 0), world_axis="Y",
                               parent_path="/World/Arm/Joints")
    return stage, link, joint


def test_set_get_clear_motion_with_a_tracking_drive(hinge):
    stage, link, joint = hinge
    mo.set_motion(joint, mo.Motion("displacement", "STEP(TIME, 0, 15, 1, 60)"))
    assert mo.get_motion(joint) == mo.Motion("displacement", "STEP(TIME, 0, 15, 1, 60)", "MKS", 0.0, 0.0)
    drive = UsdPhysics.DriveAPI(joint, "angular")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(mo.TRACKING["angular"][0])  # metre stage: SI numbers
    assert drive.GetTargetPositionAttr().Get() == pytest.approx(15)  # the motion at TIME = 0
    with pytest.raises(ValueError):
        mo.set_motion(joint, mo.Motion("displacement", "import os"))
    mo.clear_motion(joint)
    assert mo.get_motion(joint) is None and not joint.HasAPI(UsdPhysics.DriveAPI, "angular")
    fixed = jt.create_joint(stage, jt.JointSpec("fixed"), None, link, (0, 0, 0))
    with pytest.raises(ValueError, match="revolute and translational"):
        mo.set_motion(fixed, mo.Motion())


def test_translational_motion_in_millimetres(stage):
    ph.ensure_physics_scene(stage)
    block = md.create_body(stage, "/World/Block", "box", (0.1, 0.1, 0.1), mass=1.0)
    joint = jt.create_joint_at(stage, jt.JointSpec("prismatic", axis="Z"), None, block, location="body1",
                               world_axis="X")
    mo.set_motion(joint, mo.Motion("displacement", "50", units="MMKS"))
    assert UsdPhysics.DriveAPI(joint, "linear").GetTargetPositionAttr().Get() == pytest.approx(0.05)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)  # the same motion on a centimetre stage
    mo.apply_targets(joint, mo.Profile(mo.get_motion(joint), True), 0.0)
    assert UsdPhysics.DriveAPI(joint, "linear").GetTargetPositionAttr().Get() == pytest.approx(5.0)
