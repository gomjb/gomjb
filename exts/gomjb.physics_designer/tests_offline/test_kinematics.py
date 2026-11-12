"""Kinematic mechanisms: the mode switch on the USD model, and the exact solver's kinematics
(motion table, co-simulation) against the scissor lift's closed-form kinematics."""
import math

import numpy as np
import pytest

pytest.importorskip("exudyn")

from pxr import Gf, UsdPhysics  # noqa: E402

from gomjb.physics_designer.core import kinematic_mode as km  # noqa: E402
from gomjb.physics_designer.core import kinematics as kn  # noqa: E402
from gomjb.physics_designer.core import mechanisms as me  # noqa: E402
from gomjb.physics_designer.core import motion as mo  # noqa: E402
from gomjb.physics_designer.core import physics as ph  # noqa: E402
from gomjb.physics_designer.core.mbd import MbdModel  # noqa: E402

L, A0, STAGES = 1.2, 25.0, 3


@pytest.fixture
def lift(stage):
    """Two-sided lift with the default bushing loop closures; both actuators share one motion."""
    ph.ensure_physics_scene(stage)
    res = me.build_scissor_lift(stage, stages=STAGES, arm_length=L, angle=A0)
    for side in ("L", "R"):
        mo.set_motion(stage.GetPrimAtPath(f"{res['root'][0]}/Joints/Actuator_{side}"),
                      mo.Motion("displacement", "-0.2 * TIME"))
    return stage, res["root"][0]


def _angle(stroke):
    """Arm angle (deg) at slider travel ``stroke`` (m) of a lift built at A0."""
    return math.degrees(math.acos(math.cos(math.radians(A0)) + stroke / L))


def _rise(stroke):
    return STAGES * L * (math.sin(math.radians(_angle(stroke))) - math.sin(math.radians(A0)))


def _rot(frame):
    return Gf.Rotation(Gf.Quatd(frame[6], Gf.Vec3d(*frame[3:6])))


def _turn(a, b):
    """Angle (deg) between two frames' orientations."""
    return (_rot(a).GetInverse() * _rot(b)).GetAngle()


def test_mode_switch_round_trips_the_model(lift):
    stage, root_path = lift
    root = stage.GetPrimAtPath(root_path)
    before = stage.GetRootLayer().ExportToString()
    km.set_mode(root, "table", tolerance=5e-5)
    assert km.get_mode(root) == "table" and km.get_tolerance(root) == 5e-5
    assert not root.HasAPI(UsdPhysics.ArticulationRootAPI)
    assert km.mechanism_root_of(stage.GetPrimAtPath(root_path + "/Platform")) == root
    for body in km.mechanism_bodies(root):
        assert UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get()
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    assert joints and all(UsdPhysics.Joint(j).GetJointEnabledAttr().Get() is False for j in joints)
    assert km.kinematic_roots(stage) == [root]
    km.set_mode(root, "cosim")  # between kinematic modes only the mode changes
    assert km.get_mode(root) == "cosim"
    km.set_mode(root, "physics")
    assert stage.GetRootLayer().ExportToString() == before


def test_kinematic_mode_keeps_an_outside_joint(lift):
    """A payload fixed to the platform stays jointed: it follows the kinematic platform physically."""
    from gomjb.physics_designer.core import joints as jt
    from gomjb.physics_designer.core import modeling as md

    stage, root_path = lift
    box = md.create_body(stage, "/World/Payload", "box", (0.2, 0.2, 0.2), mass=1.0)
    joint = jt.create_joint(stage, jt.JointSpec("fixed", name="PayloadFix"),
                            stage.GetPrimAtPath(root_path + "/Platform"), box, (0, 0, 2.0))
    km.set_mode(stage.GetPrimAtPath(root_path), "table")
    assert UsdPhysics.Joint(joint).GetJointEnabledAttr().Get() is not False
    assert not UsdPhysics.RigidBodyAPI(box).GetKinematicEnabledAttr().Get()


def test_kinematic_mechanism_validates_without_joint_warnings(lift):
    """Its joints are disabled (not simulated), so the articulation checks do not apply."""
    from gomjb.physics_designer.core import validation as va

    stage, root_path = lift
    km.set_mode(stage.GetPrimAtPath(root_path), "table")
    assert not [i for i in va.validate(stage) if i.severity != va.INFO and "oint" in i.message]
    assert km.edit_paths(stage.GetPrimAtPath(root_path)) == [root_path]  # its joints live under the root


def test_copies_of_a_mechanism_share_one_group(stage):
    """Copies anywhere (and turned) group together; a different or edited copy does not."""
    from gomjb.physics_designer.core import math_utils as mu
    from gomjb.physics_designer.core import transform as tf

    ph.ensure_physics_scene(stage)
    roots = [me.build_scissor_lift(stage, name=n, stages=2, origin=o)["root"][0]
             for n, o in (("A", (0, 0, 0)), ("B", (5, 3, 0)), ("C", (0, 0, 0)))]
    # C turned and moved after it was built, as a cloner places copies: its ground anchor is stale
    tf.set_world_matrix(stage.GetPrimAtPath(roots[2]),
                        mu.make_transform((0, 6, 1), Gf.Rotation(Gf.Vec3d(0, 0, 1), 30.0)))
    roots.append(me.build_scissor_lift(stage, name="D", stages=2, arm_length=1.0, origin=(9, 0, 0))["root"][0])
    for r in roots:
        for side in "LR":
            mo.set_motion(stage.GetPrimAtPath(f"{r}/Joints/Actuator_{side}"), mo.Motion("displacement", "-0.1*TIME"))
        km.set_mode(stage.GetPrimAtPath(r), "table")

    def groups():
        return sorted(sorted(str(p.GetName()) for p in g) for g in km.kinematic_groups(stage))

    assert groups() == [["A", "B", "C"], ["D"]]
    # the moved copy's kinematics: the same root-local frames as the original's
    a, c = (kn.CoSimulation(kn.kinematic_model(stage, r)) for r in (roots[0], roots[2]))
    for t in (0.0, 1.0):
        assert np.abs(a.frames(t) - c.frames(t)).max() < 1e-9
    UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(roots[1] + "/Joints/Actuator_L")).GetUpperLimitAttr().Set(0.05)
    assert groups() == [["A", "C"], ["B"], ["D"]]


def test_exact_solver_reads_a_kinematic_mechanism_as_designed(lift):
    stage, root_path = lift
    a = MbdModel(stage, root_path, rigid_bushings=True)
    km.set_mode(stage.GetPrimAtPath(root_path), "cosim")
    b = MbdModel(stage, root_path, rigid_bushings=True)
    assert (list(a.bodies), list(a.joints), a.dof, a.drive_names) == (list(b.bodies), list(b.joints), b.dof,
                                                                      b.drive_names)
    assert "disabled joints skipped" not in b.notes


def test_motion_table_follows_the_scissor_kinematics(lift):
    stage, root_path = lift
    tol = 1e-4
    table = kn.build_table(kn.kinematic_model(stage, root_path), tolerance=tol)
    assert table.drives == ["Actuator_L", "Actuator_R"] and table.linear
    # range = the actuators' joint limits (75 deg .. 4 deg arm angle)
    assert table.stroke[0] == pytest.approx(me.scissor_slider_offset(L, A0, 75.0))
    assert table.stroke[-1] == pytest.approx(me.scissor_slider_offset(L, A0, 4.0))
    assert np.all(np.diff(table.stroke) > 0)
    dp, da = table.error
    assert 0 < dp <= tol and da < 0.01
    # refined where the scissor folds flat: the spacing at the 4 deg end is far finer than at 75 deg
    assert np.diff(table.stroke)[-1] < 0.1 * np.diff(table.stroke)[0]
    rest = table.at(0.0)
    names = table.bodies
    platform = names.index("Platform")
    arms = [i for i, n in enumerate(names) if n.startswith("Arm")]
    # anywhere along the stroke, not only at table points: the closed-form scissor kinematics
    for stroke in np.linspace(table.stroke[0], table.stroke[-1], 37)[1:-1]:
        f = table.at(stroke)
        assert f[platform, 2] - rest[platform, 2] == pytest.approx(_rise(stroke), abs=tol)
        assert _turn(f[platform], rest[platform]) < 0.01  # the deck stays level
        for i in arms:
            assert _turn(f[i], rest[i]) == pytest.approx(abs(_angle(stroke) - A0), abs=0.01)


def test_motion_table_reproduces_the_authored_pose(lift):
    """At the authored stroke the table gives the bodies' authored frames (root-local)."""
    from gomjb.physics_designer.core import math_utils as mu
    from gomjb.physics_designer.core import transform as tf

    stage, root_path = lift
    table = kn.build_table(kn.kinematic_model(stage, root_path), tolerance=1e-3)
    frames = table.at(0.0)
    root_inv = mu.rigid_part(tf.world_matrix(stage.GetPrimAtPath(root_path))).GetInverse()
    for name, f in zip(table.bodies, frames):
        local = tf.world_matrix(stage.GetPrimAtPath(f"{root_path}/{name}")) * root_inv
        assert list(f[:3]) == pytest.approx(list(local.ExtractTranslation()), abs=table.tolerance)


def test_cosimulation_solves_every_step(lift):
    stage, root_path = lift
    cosim = kn.CoSimulation(kn.kinematic_model(stage, root_path))
    platform = cosim.bodies.index("Platform")
    z0 = cosim.frames(0.0)[platform, 2]
    for t in (0.25, 0.5, 1.0, 2.0):  # motion: slider travel -0.2 m/s * t
        assert cosim.frames(t)[platform, 2] - z0 == pytest.approx(_rise(-0.2 * t), abs=1e-7)


def test_motion_table_needs_one_stroke_with_limits(lift):
    stage, root_path = lift
    joints = root_path + "/Joints"
    mo.set_motion(stage.GetPrimAtPath(joints + "/Actuator_R"), mo.Motion("displacement", "-0.1 * TIME"))
    with pytest.raises(ValueError, match="use co-simulation"):
        kn.build_table(kn.kinematic_model(stage, root_path))
    for side in ("L", "R"):
        mo.set_motion(stage.GetPrimAtPath(f"{joints}/Actuator_{side}"), mo.Motion("displacement", "-0.2 * TIME"))
    UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(joints + "/Actuator_L")).GetLowerLimitAttr().Set(-math.inf)
    with pytest.raises(ValueError, match="joint limits"):
        kn.build_table(kn.kinematic_model(stage, root_path))


def test_kinematics_needs_every_dof_driven(stage):
    ph.ensure_physics_scene(stage)
    res = me.build_pendulum_chain(stage)  # free (damped) pendulums
    with pytest.raises(ValueError, match="without a drive"):
        kn.kinematic_model(stage, res["root"][0])


def test_exact_solver_rejects_mimic_joints(lift):
    stage, root_path = lift
    stage.GetPrimAtPath(root_path + "/Joints/Center_L_0").AddAppliedSchema("NewtonMimicAPI")
    with pytest.raises(ValueError, match="mimic"):
        MbdModel(stage, root_path)
