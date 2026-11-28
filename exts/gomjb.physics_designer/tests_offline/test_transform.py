import pytest
from pxr import Gf, UsdGeom

from gomjb.physics_designer.core import math_utils as mu
from gomjb.physics_designer.core import transform as tf

from conftest import make_cube


def test_set_world_position_under_transformed_parent(stage):
    parent = UsdGeom.Xform.Define(stage, "/World/P")
    parent.AddTranslateOp().Set((5, 0, 0))
    parent.AddRotateZOp().Set(90)
    parent.AddScaleOp().Set((2, 2, 2))
    cube = make_cube(stage, "/World/P/C")
    tf.set_world_position(cube, (1, 2, 3))
    assert mu.is_close(tf.world_position(cube), (1, 2, 3))
    # op stack normalised to translate/orient/scale
    names = [op.GetOpName() for op in UsdGeom.Xformable(cube).GetOrderedXformOps()]
    assert names == ["xformOp:translate", "xformOp:orient", "xformOp:scale"]


def test_float_precision_preserved(stage):
    xf = UsdGeom.Xform.Define(stage, "/World/F")
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1, 1, 1))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(1))
    xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(1, 1, 1))
    tf.translate([xf.GetPrim()], (1, 0, 0))
    ops = xf.GetOrderedXformOps()
    assert all(op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat for op in ops)
    assert tuple(ops[0].Get()) == pytest.approx((2, 1, 1))


def test_translate_local_space(stage):
    cube = make_cube(stage, "/World/C", rot_z=90)
    tf.translate([cube], (1, 0, 0), space="local")
    assert mu.is_close(tf.world_position(cube), (0, 1, 0), 1e-9)


def test_rotate_about_group_center(stage):
    a = make_cube(stage, "/World/A", pos=(-1, 0, 0))
    b = make_cube(stage, "/World/B", pos=(1, 0, 0))
    tf.rotate([a, b], "Z", 90, pivot="group")
    assert mu.is_close(tf.world_position(a), (0, -1, 0), 1e-9)
    assert mu.is_close(tf.world_position(b), (0, 1, 0), 1e-9)


def test_nested_selection_moves_once(stage):
    parent = make_cube(stage, "/World/A")
    child = make_cube(stage, "/World/A/B", pos=(0, 0, 2))
    tf.translate([parent, child], (1, 0, 0))
    assert mu.is_close(tf.world_position(child), (1, 0, 2))


def test_align_and_distribute(stage):
    a = make_cube(stage, "/World/A", pos=(0, 0, 0))
    b = make_cube(stage, "/World/B", pos=(3, 2, 5), size=2)
    c = make_cube(stage, "/World/C", pos=(10, -1, 1))
    tf.align([a, b, c], "Z", "min")
    for p in (b, c):
        assert tf.world_bbox(p).GetMin()[2] == pytest.approx(-0.5)
    tf.distribute([a, b, c], "X")
    assert tf.bbox_center(b)[0] == pytest.approx(5.0)
    tf.distribute([a, b, c], "X", gap=0.5)
    assert tf.world_bbox(b).GetMin()[0] == pytest.approx(1.0)
    assert tf.world_bbox(c).GetMin()[0] == pytest.approx(tf.world_bbox(b).GetMax()[0] + 0.5)


def test_drop_to_ground_and_onto(stage):
    table = make_cube(stage, "/World/Table", pos=(0, 0, 0.5), scale=(2, 2, 1))
    box = make_cube(stage, "/World/Box", pos=(0.2, 0, 5), size=0.5)
    far = make_cube(stage, "/World/Far", pos=(10, 0, 5), size=0.5)
    tf.drop_to([box, far], 0.0, onto=[table])
    assert tf.world_bbox(box).GetMin()[2] == pytest.approx(1.0)
    assert tf.world_bbox(far).GetMin()[2] == pytest.approx(0.0)


def test_copy_pose(stage):
    src = make_cube(stage, "/World/S", pos=(1, 2, 3), rot_z=45)
    dst = make_cube(stage, "/World/D", scale=(2, 2, 2))
    tf.copy_pose(src, [dst])
    t, q, s = mu.decompose(tf.world_matrix(dst))
    assert mu.is_close(t, (1, 2, 3))
    assert mu.quat_angle_between(q, tf.world_rotation(src)) < 1e-6
    assert tuple(s) == pytest.approx((2, 2, 2))


def test_rotated_child_under_nonuniform_parent_is_exact(stage):
    parent = UsdGeom.Xform.Define(stage, "/World/P")
    parent.AddRotateZOp().Set(30)
    parent.AddScaleOp().Set((3, 1, 0.5))
    child = UsdGeom.Xform.Define(stage, "/World/P/C").GetPrim()
    target = mu.make_transform((1, 2, 3), Gf.Rotation(Gf.Vec3d(1, 1, 0), 40))
    tf.set_world_matrix(child, target)
    m = tf.world_matrix(child)
    assert all(abs(m[i][j] - target[i][j]) < 1e-9 for i in range(4) for j in range(4))
    names = [op.GetOpName() for op in UsdGeom.Xformable(child).GetOrderedXformOps()]
    assert names == ["xformOp:transform"]
    # moving back to a plain pose under an identity parent restores a TRS stack
    parent.ClearXformOpOrder()
    tf.set_world_pose(child, (0, 0, 0), Gf.Quatd(1, 0, 0, 0))
    names = [op.GetOpName() for op in UsdGeom.Xformable(child).GetOrderedXformOps()]
    assert names == ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    assert not child.GetAttribute("xformOp:transform")
