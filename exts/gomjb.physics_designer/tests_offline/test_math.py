import math

import pytest
from pxr import Gf, Usd, UsdGeom

from gomjb.physics_designer.core import math_utils as mu


def test_quantize():
    assert mu.quantize(0.26, 0.25) == pytest.approx(0.25)
    assert mu.quantize(-0.13, 0.25) == pytest.approx(-0.25)
    assert mu.quantize(1.0, 0.0) == 1.0
    v = mu.quantize_vec((0.26, 0.74, 3.3), 0.5, axes="XY")
    assert tuple(v) == pytest.approx((0.5, 0.5, 3.3))


def test_euler_matches_usd_rotate_xyz():
    stage = Usd.Stage.CreateInMemory()
    xf = UsdGeom.Xform.Define(stage, "/a")
    angles = (30.0, -20.0, 75.0)
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*angles))
    m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    q = mu.quat_from_euler_xyz(angles)
    assert mu.quat_angle_between(q, m.ExtractRotationQuat()) < 1e-6
    assert tuple(mu.euler_xyz_from_quat(q)) == pytest.approx(angles, abs=1e-6)


def test_snap_rotation():
    q = mu.quat_from_euler_xyz((0, 0, 47.0))
    snapped = mu.snap_rotation(q, 15.0)
    assert mu.quat_angle_between(snapped, mu.quat_from_euler_xyz((0, 0, 45.0))) < 1e-6


@pytest.mark.parametrize("dst", [(0, 0, 1), (0, 1, 0), (-1, 0, 0), (1, 1, 1), (1, 0, 0)])
def test_rotation_from_to(dst):
    r = mu.rotation_from_to((1, 0, 0), dst)
    out = r.TransformDir(Gf.Vec3d(1, 0, 0))
    assert mu.is_close(out, mu.axis_vector(dst), 1e-9)


def test_rotate_about_matrix_keeps_pivot():
    m = mu.rotate_about_matrix("Z", 90, (1, 0, 0))
    assert mu.is_close(m.Transform(Gf.Vec3d(1, 0, 0)), (1, 0, 0))
    assert mu.is_close(m.Transform(Gf.Vec3d(2, 0, 0)), (1, 1, 0))


def test_decompose_roundtrip():
    m = mu.make_transform((1, 2, 3), Gf.Rotation(Gf.Vec3d(0, 1, 0), 33), (2, 3, 4))
    t, q, s = mu.decompose(m)
    assert tuple(t) == pytest.approx((1, 2, 3))
    assert tuple(s) == pytest.approx((2, 3, 4))
    assert mu.quat_angle_between(q, Gf.Rotation(Gf.Vec3d(0, 1, 0), 33).GetQuat()) < 1e-6
    assert math.isclose(mu.quat_angle_between(Gf.Quatd(1, 0, 0, 0), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat()), 90)


def test_euler_gimbal_lock_roundtrip():
    for angles in [(10.0, 90.0, 0.0), (0.0, -90.0, 0.0), (179.0, 0.0, -179.0)]:
        q = mu.quat_from_euler_xyz(angles)
        q2 = mu.quat_from_euler_xyz(mu.euler_xyz_from_quat(q))
        assert mu.quat_angle_between(q, q2) < 1e-6
