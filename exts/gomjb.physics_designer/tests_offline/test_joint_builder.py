"""RecurDyn-style joint creation: pick order, frames, ground, inferred base, snapping."""
import pytest
from pxr import Gf, UsdGeom, UsdPhysics

from gomjb.physics_designer.core import joint_builder as jb
from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import snapping as sn
from gomjb.physics_designer.core import transform as tf


@pytest.fixture
def scene(stage):
    ph.ensure_physics_scene(stage)
    base = md.create_body(stage, "/World/Base", "box", (1.0, 0.4, 0.1), mass=10.0)
    tf.set_world_position(base, (0, 0, 0.05))
    link = md.create_body(stage, "/World/Link", "link", (0.8, 0.06, 0.02), mass=1.0)
    tf.set_world_pose(link, (0.2, 0, 0.3), Gf.Rotation(Gf.Vec3d(0, 1, 0), -30).GetQuat())
    ground = ph.create_ground_plane(stage, size=10)
    return stage, base, link, ground


def geom(body):
    return next(c for c in body.GetChildren() if c.IsA(UsdGeom.Gprim))


def pin(link, name):
    return next(a for a in sn.find_anchors(link) if a.GetName() == name)


def test_revolute_body_body_point_uses_the_working_plane_normal(scene):
    stage, base, link, _ = scene
    p = jb.JointPicker("Revolute")
    assert p.method == "Body, Body, Point" and "BASE" in p.prompt()
    p.pick(geom(base))  # a geometry child picks its body
    assert "ACTION" in p.prompt()
    p.pick(geom(link))
    pin_a = tf.world_position(pin(link, "PinA"))
    point, anchor = jb.snap(geom(link), pin_a + Gf.Vec3d(0.004, 0, 0.003))  # a click next to the pin
    assert anchor.GetName() == "PinA" and (point - pin_a).GetLength() < 1e-9
    p.pick(geom(link), point, anchor)
    assert p.step is None and "complete" in p.prompt()
    j = p.create(stage)
    assert j.IsA(UsdPhysics.RevoluteJoint) and UsdPhysics.RevoluteJoint(j).GetAxisAttr().Get() == "Z"
    b0, b1 = jt.joint_bodies(j)
    assert (b0.GetPath(), b1.GetPath()) == (base.GetPath(), link.GetPath())  # base = body0, action = body1
    (p0, r0), (p1, r1) = jt.joint_world_frames(j)
    assert (p0 - pin_a).GetLength() < 1e-6 and (p1 - pin_a).GetLength() < 1e-6
    z = Gf.Rotation(r0).TransformDir(Gf.Vec3d(0, 0, 1))
    assert (z - Gf.Vec3d(0, 0, 1)).GetLength() < 1e-6  # working plane XY -> axis Z


def test_working_plane_and_marker_and_global_directions(scene):
    stage, base, link, _ = scene
    p = jb.JointPicker("Revolute", working_plane="ZX")
    p.pick(geom(base))
    p.pick(geom(link))
    p.pick(geom(link), tf.world_position(pin(link, "PinB")), pin(link, "PinB"))
    assert (p.axis() - Gf.Vec3d(0, 1, 0)).GetLength() < 1e-12  # ZX plane -> Y axis
    q = jb.JointPicker("Revolute", "Body, Body, Point, Direction")
    q.pick(geom(base))
    q.pick(geom(link))
    q.pick(geom(link), tf.world_position(pin(link, "PinA")), pin(link, "PinA"))
    assert "Z axis" in q.prompt()
    q.set_direction(q.marker_axis())  # the pin's own Z axis (the link's Y axis)
    z = Gf.Rotation(q.frame()[1]).TransformDir(Gf.Vec3d(0, 0, 1))
    assert (z - Gf.Rotation(tf.world_rotation(link)).TransformDir(Gf.Vec3d(0, 1, 0))).GetLength() < 1e-6


def test_translational_needs_a_direction_from_two_points(scene):
    stage, base, link, _ = scene
    with pytest.raises(ValueError):
        jb.JointPicker("Translational", "Body, Body, Point")
    p = jb.JointPicker("Translational")
    assert p.method == "Body, Body, Point, Direction"
    p.pick(geom(base))
    p.pick(geom(link))
    p.pick(geom(link), (0.2, 0, 0.3))
    p.pick(geom(link), (0.2, 0, 0.8))  # second point straight above: slide along +Z
    j = p.create(stage, jt.JointSpec(lower=-0.1, upper=0.2))
    assert j.IsA(UsdPhysics.PrismaticJoint)
    assert UsdPhysics.PrismaticJoint(j).GetUpperLimitAttr().Get() == pytest.approx(0.2)
    z = Gf.Rotation(jt.joint_world_frames(j)[0][1]).TransformDir(Gf.Vec3d(0, 0, 1))
    assert (z - Gf.Vec3d(0, 0, 1)).GetLength() < 1e-6


def test_ground_picks_and_errors(scene):
    stage, base, link, ground = scene
    p = jb.JointPicker("Revolute")
    p.pick(ground.GetChildren()[0])  # the ground plane is not a body: ground
    assert p.base is None
    with pytest.raises(ValueError, match="both be the ground"):
        p.pick(ground)
    p.pick(geom(link))
    p.pick(geom(link), tf.world_position(link))
    j = p.create(stage)
    b0, b1 = jt.joint_bodies(j)
    assert b0 is None and b1.GetPath() == link.GetPath()
    q = jb.JointPicker("Fixed")
    q.pick(geom(link))
    with pytest.raises(ValueError, match="different bodies"):
        q.pick(geom(link))


def test_point_method_infers_the_base_from_the_point(scene):
    stage, base, link, _ = scene
    tf.set_world_pose(link, (0.2, 0, 0.08), Gf.Rotation(Gf.Vec3d(0, 1, 0), 0).GetQuat())  # lying on the base
    p = jb.JointPicker("Revolute", "Point")
    p.pick(geom(link), (0.0, 0.0, 0.08))
    assert p.action.GetPath() == link.GetPath() and p.base.GetPath() == base.GetPath()
    q = jb.JointPicker("Revolute", "Point")
    q.pick(geom(link), (0.55, 0.0, 0.08))  # beyond the base: the second body is the ground
    assert q.base is None


def test_a_click_near_a_pin_takes_the_pin_even_if_a_corner_is_closer(scene):
    """Measured in Isaac Sim: a click on a link's top edge 6 cm from PinA was nearer a bbox corner."""
    stage, base, link, _ = scene
    tf.set_world_pose(link, (0.1, 0, 0.3), Gf.Rotation(Gf.Vec3d(1, 0, 0), 0).GetQuat())
    pin_a = tf.world_position(pin(link, "PinA"))
    hit = pin_a + Gf.Vec3d(0.06, -0.005, 0.04)
    point, anchor = jb.snap(geom(link), hit)
    assert anchor is not None and anchor.GetName() == "PinA" and (point - pin_a).GetLength() < 1e-9
    far = tf.world_position(link) + Gf.Vec3d(0.0, -0.005, 0.04)  # mid-link: no pin within reach
    assert jb.snap(geom(link), far)[1] is None


def test_key_points_follow_the_rotated_geometry(scene):
    stage, base, link, _ = scene
    box = md.create_body(stage, "/World/Tilted", "box", (0.4, 0.2, 0.2), mass=1.0)
    tf.set_world_pose(box, (1, 1, 1), Gf.Rotation(Gf.Vec3d(0, 0, 1), 45).GetQuat())
    corner_dir = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45).TransformDir(Gf.Vec3d(0.2, 0, 0))
    face_center = Gf.Vec3d(1, 1, 1) + corner_dir  # the +X face center, rotated with the box
    point, anchor = jb.snap(geom(box), face_center + Gf.Vec3d(0, 0, 0.01))
    assert anchor is None and (point - face_center).GetLength() < 1e-6
    assert jb.snap(geom(box), face_center + Gf.Vec3d(0, 0, 0.01), "Surface")[0][2] == pytest.approx(1.01)
