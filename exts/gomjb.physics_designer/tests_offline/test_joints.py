import pytest
from pxr import Gf, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import math_utils as mu
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import snapping as sn
from gomjb.physics_designer.core import transform as tf

from conftest import make_cube


@pytest.fixture
def bodies(stage):
    parent = UsdGeom.Xform.Define(stage, "/World/Rig")
    parent.AddRotateXOp().Set(20)
    a = make_cube(stage, "/World/Rig/A", pos=(0, 0, 1), scale=(2, 0.5, 0.3), rot_z=30)
    b = make_cube(stage, "/World/Rig/B", pos=(2, 1, 1.5), scale=(0.7, 0.7, 3), rot_z=-60)
    ph.apply_rigid_body([a, b])
    return a, b


@pytest.mark.parametrize("jtype", jt.JOINT_TYPES)
def test_frames_consistent_for_every_type(stage, bodies, jtype):
    a, b = bodies
    rot = jt.frame_for_world_axis((1, 2, 3), "X")
    joint = jt.create_joint(stage, jt.JointSpec(jtype), a, b, (1, 0.5, 1.2), rot)
    dp, da = jt.joint_frame_error(joint)
    assert dp < 1e-5 and da < 1e-3
    (p0, r0), _ = jt.joint_world_frames(joint)
    assert mu.is_close(p0, (1, 0.5, 1.2), 1e-5)
    assert mu.quat_angle_between(r0, rot) < 1e-3
    assert str(joint.GetPath()).startswith("/World/Joints/")


def test_world_joint_and_axis(stage, bodies):
    _, b = bodies
    spec = jt.JointSpec("revolute", axis="Z", lower=-45, upper=90,
                        drive=jt.DriveSpec(stiffness=100, damping=10, target_position=15))
    joint = jt.create_joint_at(stage, spec, None, b, location="body1_bbox", world_axis=(0, 1, 0))
    rj = UsdPhysics.RevoluteJoint(joint)
    assert rj.GetAxisAttr().Get() == "Z"
    assert rj.GetLowerLimitAttr().Get() == -45 and rj.GetUpperLimitAttr().Get() == 90
    drive = UsdPhysics.DriveAPI(joint, "angular")
    assert drive.GetStiffnessAttr().Get() == 100 and drive.GetTargetPositionAttr().Get() == 15
    (p0, r0), _ = jt.joint_world_frames(joint)
    axis_world = Gf.Rotation(r0).TransformDir(Gf.Vec3d(0, 0, 1))
    assert mu.is_close(axis_world, (0, 1, 0), 1e-5)
    assert mu.is_close(p0, tf.bbox_center(b), 1e-5)
    assert not rj.GetBody0Rel().GetTargets()


def test_joint_at_anchor_uses_anchor_frame(stage, bodies):
    a, b = bodies
    anchor = sn.create_anchor(stage, a, "Pin", world_position=(0.5, 0.2, 1.1),
                              world_rotation=Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat())
    joint = jt.create_joint_at(stage, jt.JointSpec("revolute", axis="Z"), a, b, location="anchor",
                               world_axis="anchor", anchor=anchor)
    (p0, r0), _ = jt.joint_world_frames(joint)
    assert mu.is_close(p0, (0.5, 0.2, 1.1), 1e-5)
    assert mu.is_close(Gf.Rotation(r0).TransformDir(Gf.Vec3d(0, 0, 1)), (0, -1, 0), 1e-5)


def test_d6_and_misc(stage, bodies):
    a, b = bodies
    spec = jt.JointSpec("d6", d6={"transX": (-0.1, 0.1), "transY": "locked", "transZ": "locked",
                                  "rotX": "locked", "rotY": "locked"},
                        d6_drives={"rotZ": jt.DriveSpec(damping=3)}, break_force=1000)
    joint = jt.create_joint(stage, spec, a, b, (0, 0, 0))
    assert UsdPhysics.LimitAPI(joint, "transY").GetLowAttr().Get() > UsdPhysics.LimitAPI(joint, "transY").GetHighAttr().Get()
    assert UsdPhysics.LimitAPI(joint, "transX").GetHighAttr().Get() == pytest.approx(0.1)
    assert not joint.HasAPI(UsdPhysics.LimitAPI, "rotZ")
    assert UsdPhysics.DriveAPI(joint, "rotZ").GetDampingAttr().Get() == 3
    assert UsdPhysics.Joint(joint).GetBreakForceAttr().Get() == 1000
    with pytest.raises(ValueError):
        jt.create_joint(stage, jt.JointSpec("revolute"), a, a, (0, 0, 0))
    with pytest.raises(ValueError):
        jt.create_joint(stage, jt.JointSpec("bogus"), a, b, (0, 0, 0))


def test_realign_after_body_moved(stage, bodies):
    a, b = bodies
    joint = jt.create_joint(stage, jt.JointSpec("fixed"), a, b, (1, 1, 1))
    tf.translate([b], (0.3, 0, 0))
    assert jt.joint_frame_error(joint)[0] == pytest.approx(0.3, rel=1e-4)
    jt.realign_joint(joint, keep="body0")
    assert jt.joint_frame_error(joint)[0] < 1e-5
