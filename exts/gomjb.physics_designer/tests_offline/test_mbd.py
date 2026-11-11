"""Exact solver (Exudyn) against problems with known answers."""
import math

import pytest

pytest.importorskip("exudyn")

from pxr import Gf, UsdGeom  # noqa: E402

from gomjb.physics_designer.core import joints as jt  # noqa: E402
from gomjb.physics_designer.core import mechanisms as me  # noqa: E402
from gomjb.physics_designer.core import modeling as md  # noqa: E402
from gomjb.physics_designer.core import physics as ph  # noqa: E402
from gomjb.physics_designer.core import transform as tf  # noqa: E402
from gomjb.physics_designer.core.mbd import MbdModel  # noqa: E402

G = 9.81


@pytest.fixture
def scene(stage):
    ph.ensure_physics_scene(stage)
    return stage


def _mechanism(stage, name):
    root = UsdGeom.Xform.Define(stage, f"/World/{name}").GetPrim()
    ph.apply_articulation_root(root)
    return root


def test_vertical_prismatic_and_horizontal_link_reactions(scene):
    _mechanism(scene, "Lift")
    block = md.create_body(scene, "/World/Lift/Block", "box", (0.2, 0.2, 0.2), mass=10.0)
    tf.set_world_position(block, (0, 0, 1))
    jt.create_joint(scene, jt.JointSpec("prismatic", name="Slide", axis="Z",
                                        drive=jt.DriveSpec(stiffness=1e6, damping=1e3)),
                    None, block, (0, 0, 1), parent_path="/World/Lift/Joints")
    _mechanism(scene, "Arm")
    link = md.create_body(scene, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (3.5, 0, 1))
    jt.create_joint(scene, jt.JointSpec("revolute", name="Hinge", axis="Y",
                                        drive=jt.DriveSpec(stiffness=1e6, damping=1e3)),
                    None, link, (3, 0, 1), parent_path="/World/Arm/Joints")

    lift = MbdModel(scene, "/World/Lift")
    v = lift.solve_static()
    assert v["force:Slide"] == pytest.approx(10 * G, rel=1e-6)  # joint pushes the block up (+Z); USD gravity is float32
    arm = MbdModel(scene, "/World/Arm")
    v = arm.solve_static()
    assert v["force:Hinge"] == pytest.approx(-2 * G * 0.5, rel=1e-6)  # same sign as PhysX: -9.81 N*m
    assert v["joint_force:Hinge:z"] == pytest.approx(2 * G, rel=1e-6)


def _fixed_base(stage, rot: Gf.Rotation, pos=(0, 0, 0)):
    base = md.create_body(stage, "/World/M/Base", "box", (0.2, 0.2, 0.2), mass=3.0)
    tf.set_world_pose(base, pos, rot.GetQuat())
    jt.create_joint(stage, jt.JointSpec("fixed", name="Fix"), None, base, pos, parent_path="/World/M/Joints")
    return base


BASE_ROTATIONS = [Gf.Rotation(Gf.Vec3d(0, 0, 1), 0), Gf.Rotation(Gf.Vec3d(0, 0, 1), 90),
                  Gf.Rotation(Gf.Vec3d(0, 0, 1), 30) * Gf.Rotation(Gf.Vec3d(1, 0, 0), 25)]


@pytest.mark.parametrize("base_rot", BASE_ROTATIONS)
@pytest.mark.parametrize("token", "XYZ")
def test_driven_revolute_torque_for_every_joint_axis(scene, base_rot, token):
    """Joint frame axis X/Y/Z mapped onto world Y, under a rotated base: holding torque and angle."""
    _mechanism(scene, "M")
    base = _fixed_base(scene, base_rot)
    link = md.create_body(scene, "/World/M/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (0.5, 0, 0))
    spec = jt.JointSpec("revolute", name="Hinge", axis=token, drive=jt.DriveSpec(stiffness=1e6))
    jt.create_joint_at(scene, spec, base, link, location="custom", custom_position=(0, 0, 0), world_axis="Y",
                       parent_path="/World/M/Joints")
    model = MbdModel(scene, "/World/M")
    for target in (0.0, 50.0):
        model.set_drive("Hinge", target)
        v = model.solve_static(steps=5)
        assert v["pos:Hinge"] == pytest.approx(target, abs=1e-6)
        assert v["force:Hinge"] == pytest.approx(-2 * G * 0.5 * math.cos(math.radians(target)), abs=1e-5)


@pytest.mark.parametrize("base_rot", BASE_ROTATIONS[:2])
@pytest.mark.parametrize("token", "XYZ")
def test_driven_prismatic_moves_along_its_axis_under_a_rotated_base(scene, base_rot, token):
    import exudyn as exu

    _mechanism(scene, "M")
    base = _fixed_base(scene, base_rot, (0, 0, 1))
    block = md.create_body(scene, "/World/M/Block", "box", (0.2, 0.2, 0.2), mass=10.0)
    tf.set_world_position(block, (0.5, 0.2, 1.3))
    spec = jt.JointSpec("prismatic", name="Slide", axis=token, drive=jt.DriveSpec(stiffness=1e6))
    jt.create_joint_at(scene, spec, base, block, location="custom", custom_position=(0.5, 0.2, 1.3),
                       world_axis="Z", parent_path="/World/M/Joints")
    model = MbdModel(scene, "/World/M")
    model.set_drive("Slide", 0.3)
    v = model.solve_static(steps=3)
    node = model.mbs.GetObject(model.bodies["/World/M/Block"])["nodeNumber"]
    assert list(model.mbs.GetNodeOutput(node, exu.OutputVariableType.Position)) == pytest.approx([0.5, 0.2, 1.6], abs=1e-6)  # float32 USD poses
    assert v["pos:Slide"] == pytest.approx(0.3, abs=1e-9)
    assert v["force:Slide"] == pytest.approx(10 * G, rel=1e-6)


def test_prismatic_on_a_moving_arm(scene):
    """The slide's offset follows its (rotating) base body: block at 0.8 m along a 40 deg arm."""
    import exudyn as exu

    _mechanism(scene, "M")
    arm = md.create_body(scene, "/World/M/Arm", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(arm, (0.5, 0, 0))
    jt.create_joint_at(scene, jt.JointSpec("revolute", name="Hinge", axis="X", drive=jt.DriveSpec(stiffness=1e6)),
                       None, arm, location="custom", custom_position=(0, 0, 0), world_axis="Y",
                       parent_path="/World/M/Joints")
    block = md.create_body(scene, "/World/M/Block", "box", (0.1, 0.1, 0.1), mass=1.0)
    tf.set_world_position(block, (0.5, 0, 0))
    jt.create_joint_at(scene, jt.JointSpec("prismatic", name="Slide", axis="Z", drive=jt.DriveSpec(stiffness=1e6)),
                       arm, block, location="custom", custom_position=(0.5, 0, 0), world_axis="X",
                       parent_path="/World/M/Joints")
    model = MbdModel(scene, "/World/M")
    model.set_drive("Hinge", 40.0)
    model.set_drive("Slide", 0.3)
    v = model.solve_static(steps=8)
    a = math.radians(40.0)
    node = model.mbs.GetObject(model.bodies["/World/M/Block"])["nodeNumber"]
    pos = list(model.mbs.GetNodeOutput(node, exu.OutputVariableType.Position))
    assert pos == pytest.approx([0.8 * math.cos(a), 0.0, -0.8 * math.sin(a)], abs=1e-6)
    assert v["force:Slide"] == pytest.approx(-1.0 * G * math.sin(a), rel=1e-6)  # holds the block up the incline


@pytest.mark.parametrize("stages", [1, 2, 3])
def test_single_side_scissor_actuator_force_is_exact(scene, stages):
    res = me.build_scissor_lift(scene, stages=stages, sides=1, angle=30.0, loop_closure="rigid")
    model = MbdModel(scene, res["root"][0])
    assert model.dof == 0 and model.drive_names == ["Actuator"]
    angles = [35.0, 45.0, 60.0]  # clear of the 90 deg singular (fully upright) pose
    targets = [me.scissor_slider_offset(1.2, 30.0, a) for a in angles]
    rec = model.static_sweep(["Actuator"], targets)
    for theta, target, force, pos in zip(angles, targets, rec.channels["force:Actuator"], rec.channels["pos:Actuator"]):
        assert pos == pytest.approx(target, abs=1e-9)
        expected = me.scissor_actuator_force(1.2, theta, stages, 20.0, 5.0, sides=1)
        assert -force == pytest.approx(expected, rel=1e-6)  # pushes the slider toward the fixed pin (-X)


def test_two_sided_scissor_total_and_redundancy_report(scene):
    res = me.build_scissor_lift(scene, stages=3, sides=2, angle=20.0, loop_closure="rigid")
    model = MbdModel(scene, res["root"][0])
    assert model.dof == 0 and model.redundant == 0 and model.released
    assert "not unique" in model.report()
    target = me.scissor_slider_offset_for_height(1.2, 20.0, 3, 2.2)
    model.set_drive("Actuator_L", target)
    model.set_drive("Actuator_R", target)
    v = model.solve_static(steps=10)
    total = -(v["force:Actuator_L"] + v["force:Actuator_R"])
    theta = math.degrees(math.asin(2.2 / 3.6))
    assert total == pytest.approx(2 * me.scissor_actuator_force(1.2, theta, 3, 20.0, 5.0, sides=2), rel=1e-6)
    from gomjb.physics_designer.core.mbd import AXIS_NAMES

    for joint, axis in model.released:  # released equations carry no reaction (as in MotionSolve)
        a = AXIS_NAMES.index(axis)
        assert v[f"{'joint_force' if a < 3 else 'joint_torque'}:{joint}:{'xyz'[a % 3]}"] == 0.0


def test_bushings_give_a_symmetric_split_with_the_same_total(scene):
    """Adams / Altair practice for redundant load paths: stiff bushings instead of exact joints."""
    res = me.build_scissor_lift(scene, stages=3, sides=2, angle=20.0, loop_closure="rigid")
    model = MbdModel(scene, res["root"][0], redundancy="bushings")
    assert not model.released and "bushings" in model.report()
    target = me.scissor_slider_offset_for_height(1.2, 20.0, 3, 2.2)
    model.set_drive("Actuator_L", target)
    model.set_drive("Actuator_R", target)
    v = model.solve_static(steps=10)
    left, right = -v["force:Actuator_L"], -v["force:Actuator_R"]
    theta = math.degrees(math.asin(2.2 / 3.6))
    total = 2 * me.scissor_actuator_force(1.2, theta, 3, 20.0, 5.0, sides=2)
    # bushings deflect (F/k at each of the pins in series), which changes the arm angle slightly
    assert left + right == pytest.approx(total, rel=1e-3)
    assert left == pytest.approx(right, rel=1e-3)  # symmetric structure, symmetric load
    # re-solving an equilibrium (unchanged targets) must converge at once: sweeps start that way
    again = model.solve_static()
    assert again["force:Actuator_L"] == pytest.approx(v["force:Actuator_L"], rel=1e-9)
    assert len(model.static_sweep(["Actuator_L", "Actuator_R"], [target, 0.0])) == 2
    # the released-equation model gives the same total but an arbitrary split
    exact = MbdModel(scene, res["root"][0])
    exact.set_drive("Actuator_L", target)
    exact.set_drive("Actuator_R", target)
    e = exact.solve_static(steps=10)
    assert -(e["force:Actuator_L"] + e["force:Actuator_R"]) == pytest.approx(total, rel=1e-6)


def test_bushing_reaction_sign_matches_joints(scene):
    _mechanism(scene, "Arm")
    link = md.create_body(scene, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (0.5, 0, 0))
    jt.create_joint(scene, jt.JointSpec("fixed", name="Fix"), None, link, (0, 0, 0),
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat(), parent_path="/World/Arm/Joints")
    for mode in ("release", "bushings"):
        v = MbdModel(scene, "/World/Arm", redundancy=mode).solve_static()
        # joint frame yawed 90 deg: world +Z force and world -Y torque on the link, expressed in J0
        assert v["joint_force:Fix:z"] == pytest.approx(2 * G, rel=1e-5), mode
        assert v["joint_torque:Fix:x"] == pytest.approx(-G, rel=1e-5), mode


def test_ground_reaction_carries_the_whole_weight(scene):
    res = me.build_scissor_lift(scene, stages=2, sides=1, angle=35.0, loop_closure="rigid")
    model = MbdModel(scene, res["root"][0])
    v = model.solve_static()
    weight = sum(md.body_mass_properties(scene.GetPrimAtPath(p)).mass for p in res["bodies"]) * G
    assert v["joint_force:BaseFixed:z"] == pytest.approx(weight, rel=1e-6)
    assert abs(v["joint_force:BaseFixed:x"]) < 1e-6 * weight


def test_compound_pendulum_period(scene):
    """Free link swinging from a pin: T = 2*pi*sqrt(I_pivot / (m g d)) for small amplitude."""
    _mechanism(scene, "Pendulum")
    L, m = 0.8, 1.5
    link = md.create_body(scene, "/World/Pendulum/Link", "link", (L, 0.04, 0.02), mass=m)
    amplitude = 2.0  # deg
    a = math.radians(amplitude)
    tf.set_world_pose(link, (L / 2 * math.sin(a), 0, -L / 2 * math.cos(a)),
                      Gf.Rotation(Gf.Vec3d(0, 1, 0), 90 - amplitude).GetQuat())
    jt.create_joint(scene, jt.JointSpec("revolute", name="Pin", axis="Y"), None, link, (0, 0, 0),
                    parent_path="/World/Pendulum/Joints")
    props = md.body_mass_properties(link)
    i_pivot = props.inertia[1] + m * (L / 2) ** 2
    expected = 2 * math.pi * math.sqrt(i_pivot / (m * G * L / 2))
    model = MbdModel(scene, "/World/Pendulum")
    rec = model.simulate(end_time=2.5 * expected, profiles={}, step=2e-4, sample=1e-3)
    # the joint angle is 0 at the released (authored) pose, so the swing is centred on -amplitude
    angle = [v - (max(rec.channels["pos:Pin"]) + min(rec.channels["pos:Pin"])) / 2 for v in rec.channels["pos:Pin"]]
    assert max(angle) == pytest.approx(amplitude, rel=1e-2)
    crossings = [rec.time[k] for k in range(1, len(angle)) if angle[k - 1] < 0 <= angle[k] or angle[k - 1] > 0 >= angle[k]]
    period = 2 * (crossings[-1] - crossings[0]) / (len(crossings) - 1)
    assert period == pytest.approx(expected, rel=2e-3)
