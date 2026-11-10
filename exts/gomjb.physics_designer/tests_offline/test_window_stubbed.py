"""Drive the real window callbacks with stubbed Kit modules on a usd-core stage."""
import sys

import pytest
from pxr import UsdGeom, UsdPhysics

import kit_stubs


@pytest.fixture
def env(stage):
    ctx, commands = kit_stubs.install()
    for mod in [m for m in sys.modules if m.startswith("gomjb.physics_designer")]:
        del sys.modules[mod]
    from gomjb.physics_designer import commands as dcmd
    from gomjb.physics_designer.ui.window import PhysicsDesignerWindow

    dcmd.register()
    ctx.stage = stage
    for name, x in (("A", 0.0), ("B", 2.0)):
        cube = UsdGeom.Cube.Define(stage, f"/World/{name}")
        UsdGeom.Xformable(cube).AddTranslateOp().Set((x, 0.03, 0))
    kit_stubs._Widget.registry.clear()
    win = PhysicsDesignerWindow()
    yield win, ctx, commands
    for mod in [m for m in list(sys.modules) if m.startswith("gomjb.physics_designer") or m.startswith("omni")
                or m == "carb"]:
        del sys.modules[mod]


def click(text):
    btn = [w for w in kit_stubs._Widget.registry if isinstance(w, kit_stubs.Button) and w.args[0] == text]
    assert btn, f"no button {text!r}"
    btn[-1].kwargs["clicked_fn"]()


def status(win):
    return win._status_model.as_string


def select(ctx, *paths):
    ctx.selection.paths = list(paths)


def test_every_button_runs_without_crashing(env):
    win, ctx, _ = env
    select(ctx, "/World/A", "/World/B")
    skip = {"Build template"}
    for w in list(kit_stubs._Widget.registry):
        if isinstance(w, kit_stubs.Button) and w.args[0] not in skip:
            select(ctx, "/World/A", "/World/B")
            w.kwargs["clicked_fn"]()
            assert "failed" not in status(win) or any(
                k in status(win) for k in ("select two anchors", "select joint prims", "no snap point")
            ), f"{w.args[0]}: {status(win)}"


def test_nudge_undo(env):
    win, ctx, commands = env
    from gomjb.physics_designer.core import transform as tf

    select(ctx, "/World/A")
    click("Z +")
    a = ctx.stage.GetPrimAtPath("/World/A")
    assert tf.world_position(a)[2] == pytest.approx(0.1)
    commands.undo()
    assert tf.world_position(a)[2] == pytest.approx(0.0)


def test_joint_button_creates_rigid_bodies_and_joint(env):
    win, ctx, commands = env
    select(ctx, "/World/A", "/World/B")
    win._j_loc.widget.model.select(list(win._j_loc.items).index("Midpoint"))
    win._j_limit.model.set_value(True)
    win._j_drive.model.set_value(True)
    click("Create joint")
    assert "done" in status(win), status(win)
    joint_path = ctx.selection.paths[0]
    joint = ctx.stage.GetPrimAtPath(joint_path)
    assert joint.IsA(UsdPhysics.RevoluteJoint)
    assert UsdPhysics.RevoluteJoint(joint).GetLowerLimitAttr().Get() == -90
    assert joint.HasAPI(UsdPhysics.DriveAPI, "angular")
    assert ctx.stage.GetPrimAtPath("/World/A").HasAPI(UsdPhysics.RigidBodyAPI)
    commands.undo()
    assert not ctx.stage.GetPrimAtPath(joint_path)
    assert not ctx.stage.GetPrimAtPath("/World/A").HasAPI(UsdPhysics.RigidBodyAPI)


def test_anchor_connect_and_joint(env):
    win, ctx, _ = env
    from gomjb.physics_designer.core import math_utils as mu
    from gomjb.physics_designer.core import transform as tf

    win._anchor_loc.widget.model.select(_index(win._anchor_loc, "+X face"))
    select(ctx, "/World/A")
    click("Create anchor on selection")
    socket = ctx.selection.paths[0]
    win._anchor_loc.widget.model.select(_index(win._anchor_loc, "-X face"))
    select(ctx, "/World/B")
    click("Create anchor on selection")
    plug = ctx.selection.paths[0]
    win._conn_flip.model.set_value(True)
    win._conn_joint.widget.model.select(_index(win._conn_joint, "revolute"))
    select(ctx, plug, socket)
    click("Connect anchors")
    assert "done" in status(win), status(win)
    b = ctx.stage.GetPrimAtPath("/World/B")
    assert mu.is_close(tf.world_bbox(b).GetMin(), (1.0, -0.97, -1.0), 1e-6)  # default Cube size is 2
    joints = [p for p in ctx.stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    assert len(joints) == 1


def test_templates_build_from_form(env):
    win, ctx, commands = env
    for i, name in enumerate(win._tpl.items):
        win._tpl.widget.model.select(i)
        click("Build template")
        assert "done" in status(win), (name, status(win))
    n = len(list(ctx.stage.Traverse()))
    for _ in win._tpl.items:
        commands.undo()
    assert len(list(ctx.stage.Traverse())) < n
    assert ctx.stage.GetPrimAtPath("/World/A")


def test_validate_and_autofix(env):
    win, ctx, _ = env
    click("Validate stage")
    assert "PhysicsScene" in win._validation_model.as_string
    click("Auto-fix fixable issues")
    assert any(p.IsA(UsdPhysics.Scene) for p in ctx.stage.Traverse())


def _index(combo, item):
    return list(combo.items).index(item)


def test_live_snap_quantizes_selected(env):
    win, ctx, _ = env
    from gomjb.physics_designer.core import math_utils as mu
    from gomjb.physics_designer.core import transform as tf
    from gomjb.physics_designer.ui.live_snap import LiveSnap

    a = ctx.stage.GetPrimAtPath("/World/A")
    tf.set_world_pose(a, (0.123, 0.26, -0.04), mu.quat_from_euler_xyz((0, 0, 50)))
    LiveSnap._snap(a, {"step": 0.05, "angle": 15.0, "position": True, "rotation": True})
    assert tuple(tf.world_position(a)) == pytest.approx((0.1, 0.25, -0.05))
    assert mu.quat_angle_between(tf.world_rotation(a), mu.quat_from_euler_xyz((0, 0, 45))) < 1e-6


def test_designer_edit_context_manager(env):
    win, ctx, commands = env
    from gomjb.physics_designer.commands import designer_edit
    from gomjb.physics_designer.core import joints, physics, snapping, transform

    base = ctx.stage.GetPrimAtPath("/World/A")
    arm = ctx.stage.GetPrimAtPath("/World/B")
    with designer_edit(["/World/A", "/World/B"], "Assemble arm"):
        physics.apply_rigid_body([base, arm])
        snapping.mate_faces(arm, base, "+Z")
        spec = joints.JointSpec("revolute", axis="X", lower=-45, upper=45,
                                drive=joints.DriveSpec(stiffness=1e4, damping=1e2))
        j = joints.create_joint_at(ctx.stage, spec, base, arm, location="body1_bbox", world_axis="Y")
    assert transform.world_bbox(arm).GetMin()[2] == pytest.approx(1.0)
    commands.undo()
    assert not ctx.stage.GetPrimAtPath(j.GetPath())
    assert transform.world_position(arm)[2] == pytest.approx(0.0)
    commands.redo()
    assert ctx.stage.GetPrimAtPath(j.GetPath())

    with pytest.raises(RuntimeError):
        with designer_edit(["/World/A"]):
            transform.translate([base], (9, 0, 0))
            raise RuntimeError("abort")
    assert transform.world_position(base)[0] == pytest.approx(0.0)
