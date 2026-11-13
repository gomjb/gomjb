import pytest
from pxr import Gf, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import math_utils as mu
from gomjb.physics_designer.core import snapping as sn
from gomjb.physics_designer.core import transform as tf

from conftest import make_cube


def test_grid_and_rotation_snap(stage):
    c = make_cube(stage, "/World/C", pos=(0.26, 1.12, 0.49), rot_z=47)
    sn.snap_to_grid([c], 0.25)
    assert tuple(tf.world_position(c)) == pytest.approx((0.25, 1.0, 0.5))
    sn.snap_rotation([c], 15)
    assert mu.quat_angle_between(tf.world_rotation(c), mu.quat_from_euler_xyz((0, 0, 45))) < 1e-6


def test_grid_snap_bbox_corner(stage):
    c = make_cube(stage, "/World/C", pos=(0.1, 0.1, 0.1))
    sn.snap_to_grid([c], 1.0, use_bbox=True)
    assert tuple(tf.world_bbox(c).GetMin()) == pytest.approx((0, 0, 0))


def test_mate_faces_stack(stage):
    base = make_cube(stage, "/World/Base", pos=(1, 1, 0), scale=(4, 4, 1))
    top = make_cube(stage, "/World/Top", pos=(-3, 7, 9), size=0.5)
    sn.mate_faces(top, base, "+Z", gap=0.01)
    bb = tf.world_bbox(top)
    assert bb.GetMin()[2] == pytest.approx(0.51)
    assert bb.GetMidpoint()[0] == pytest.approx(1)
    sn.mate_faces(top, base, "-X", center=False)
    assert tf.world_bbox(top).GetMax()[0] == pytest.approx(-1)


def test_snap_to_nearest_bbox_point(stage):
    a = make_cube(stage, "/World/A", pos=(0, 0, 0))
    b = make_cube(stage, "/World/B", pos=(1.1, 0.05, 0.4), size=0.2)
    target = sn.snap_to_nearest_point(b, a, source="pivot", candidates="bbox")
    assert tuple(target) == pytest.approx((0.5, 0, 0.5))
    assert tuple(tf.world_position(b)) == pytest.approx((0.5, 0, 0.5))
    assert sn.snap_to_nearest_point(b, a, max_distance=1e-9, candidates="pivot") is None


def test_vertex_snap(stage):
    mesh = UsdGeom.Mesh.Define(stage, "/World/M")
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    mesh.AddTranslateOp().Set((0, 0, 2))
    c = make_cube(stage, "/World/C", pos=(0.9, 0.2, 2.3), size=0.1)
    sn.snap_to_nearest_point(c, mesh.GetPrim(), candidates="vertex")
    assert tuple(tf.world_position(c)) == pytest.approx((1, 0, 2))


def test_connect_anchors(stage):
    a = make_cube(stage, "/World/A", pos=(0, 0, 0), scale=(2, 2, 2))
    b = make_cube(stage, "/World/B", pos=(5, 5, 0), rot_z=30)
    rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90).GetQuat()
    anchor_a = sn.create_anchor(stage, a, "Socket", world_position=(0, 0, 1), world_rotation=rot)
    anchor_b = sn.create_anchor(stage, b, "Plug", world_position=(5, 5, -0.5))
    assert sn.is_anchor(anchor_a) and len(sn.find_anchors(stage.GetPseudoRoot())) == 2
    # anchor frames are scale free even under a scaled parent
    _, _, s = mu.decompose(tf.world_matrix(anchor_a))
    assert tuple(s) == pytest.approx((1, 1, 1))
    moved = sn.connect_anchors(anchor_b, anchor_a, align_rotation=True)
    assert moved.GetPath() == b.GetPath()
    assert mu.is_close(tf.world_position(anchor_b), (0, 0, 1), 1e-6)
    assert mu.quat_angle_between(tf.world_rotation(anchor_b), rot) < 1e-5


def test_connect_anchors_flip_and_owner(stage):
    body = UsdGeom.Xform.Define(stage, "/World/Body").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    part = make_cube(stage, "/World/Body/Part", pos=(0, 0, 1))
    anchor = sn.create_anchor(stage, part, "Tip", world_position=(0, 0, 1.5))
    target_body = make_cube(stage, "/World/T", pos=(3, 0, 0))
    target = sn.create_anchor(stage, target_body, "Hole", world_position=(3, 0, 0.5))
    assert sn.anchor_owner(anchor).GetPath() == body.GetPath()
    sn.connect_anchors(anchor, target, flip=True)
    assert mu.is_close(tf.world_position(anchor), (3, 0, 0.5), 1e-6)
    z = mu.rotation_of(tf.world_matrix(anchor)).TransformDir(Gf.Vec3d(0, 0, 1))
    assert mu.is_close(z, (0, 0, -1), 1e-6)
