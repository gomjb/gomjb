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
                or m.startswith("isaacsim") or m == "carb"]:
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
                k in status(win) for k in ("select two anchors", "select joint prims", "no snap point",
                                           "select bodies created by the modeler")
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


def _combo(combo, item):
    combo.widget.model.select(list(combo.items).index(item))


def test_modeler_link_on_anchor_and_undo(env):
    win, ctx, commands = env
    from gomjb.physics_designer.core import math_utils as mu
    from gomjb.physics_designer.core import modeling as md
    from gomjb.physics_designer.core import transform as tf

    _combo(win._anchor_loc, "+X face")
    select(ctx, "/World/A")
    click("Create anchor on selection")
    anchor = ctx.stage.GetPrimAtPath(ctx.selection.paths[0])
    m = win.modeler
    _combo(m._shape, "link")
    _combo(m._place, "At selected anchor")
    _combo(m._material, "aluminum")
    select(ctx, str(anchor.GetPath()))
    click("Create body")
    assert "mass" in status(win), status(win)
    link = ctx.stage.GetPrimAtPath(ctx.selection.paths[0])
    assert md.body_params(link)["shape"] == "link"
    assert mu.is_close(tf.world_position(link.GetChild("PinA")), tf.world_position(anchor), 1e-9)
    commands.undo()
    assert not ctx.stage.GetPrimAtPath(link.GetPath())


def test_modeler_load_and_apply(env):
    win, ctx, _ = env
    from gomjb.physics_designer.core import modeling as md

    m = win.modeler
    _combo(m._shape, "box")
    click("Create body")
    box = ctx.selection.paths[0]
    mass0 = md.body_mass_properties(ctx.stage.GetPrimAtPath(box)).mass
    _combo(m._shape, "sphere")  # load must switch the form back to the body's shape
    click("Load from selected")
    assert m._shape.value == "box" and len(m._dim_inputs) == 3
    m._dim_inputs[0].value = m._dim_inputs[0].value * 2
    click("Apply to selected")
    assert md.body_mass_properties(ctx.stage.GetPrimAtPath(box)).mass == pytest.approx(2 * mass0)


def test_analysis_plot_summary_and_csv(env, tmp_path):
    win, ctx, _ = env
    from gomjb.physics_designer.core.results import Recording

    rec = Recording("lift")
    for i in range(4):
        rec.add_sample(i / 60, {"pos:Actuator_L": -0.01 * i, "force:Actuator_L": -1000.0 - i,
                                "force:Hinge": 2.0, "link_force:Platform:x": 3.0,
                                "link_force:Platform:y": 0.0, "link_force:Platform:z": 4.0})
    rec.units["force:Actuator_L"] = "N"
    win.analysis._result = rec
    win.analysis._filter.model.set_value("Actuator")
    win.analysis.refresh_plot()
    plot = win.analysis._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Actuator_L"] and plot.y_data[0][-1] == -1003.0
    assert plot.y_min < -1003.0 and plot.y_max > -1000.0 and (plot.x_min, plot.x_max) == (0.0, 3 / 60)
    assert "Actuator_L: min -1003  max -1000  final -1003 N" in win.analysis._summary_model.as_string
    from gomjb.physics_designer.ui.analysis_section import SERIES_COLORS

    assert plot.colors == [SERIES_COLORS[0]]
    win.analysis._filter.model.set_value("")
    win.analysis.refresh_plot()
    plot = win.analysis._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Actuator_L", "Hinge"] and plot.colors == SERIES_COLORS[:2]
    win.analysis._filter.model.set_value("Hinge")
    win.analysis.refresh_plot()
    plot = win.analysis._plot  # the plot is rebuilt on every refresh
    assert plot.colors == [SERIES_COLORS[1]]  # the survivor keeps its color
    _combo(win.analysis._kind, "Joint reaction |F|")
    win.analysis._filter.model.set_value("")
    win.analysis.refresh_plot()
    plot = win.analysis._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Platform"] and plot.y_data[0][0] == pytest.approx(5.0)
    win.analysis._csv.model.set_value(str(tmp_path / "out.csv"))
    click("Export CSV")
    assert (tmp_path / "out.csv").read_text(encoding="utf-8").startswith("time [s],pos:Actuator_L")


def test_exact_solver_sweep_and_bushings(env, tmp_path):
    import math

    win, ctx, _ = env
    from gomjb.physics_designer.core import mechanisms as me
    from gomjb.physics_designer.core import physics as ph

    UsdGeom.SetStageMetersPerUnit(ctx.stage, 1.0)
    ph.ensure_physics_scene(ctx.stage)
    res = me.build_scissor_lift(ctx.stage, stages=1, sides=1, angle=30.0)
    a = win.analysis
    a._root.model.set_value(res["root"][0])
    targets = [me.scissor_slider_offset(1.2, 30.0, deg) for deg in (40.0, 50.0)]
    a._sweep_from.value, a._sweep_to.value = targets
    a._sweep_points.model.set_value(2)
    a._filter.model.set_value("Actuator")
    click("Sweep drive")
    assert "Swept Actuator over 2 points" in status(win)
    plot = a._plot
    assert plot.legends == ["Actuator"] and a.recording.x_label == "Actuator [m]"
    # this stroke runs downwards (raising the lift); XYPlot only draws increasing x
    assert targets[1] < targets[0] and plot.x_data[0] == sorted(targets)
    assert plot.y_data[0] == a.recording.channels["force:Actuator"][::-1]
    assert a._summary_model.as_string.startswith("Actuator: min")  # results first, report below
    for deg, force in zip((40.0, 50.0), a.recording.channels["force:Actuator"]):
        assert -force == pytest.approx(me.scissor_actuator_force(1.2, deg, 1, 20.0, 5.0, sides=1), rel=1e-6)
    assert "redundancy handling: release" in a._summary_model.as_string
    a._csv.model.set_value(str(tmp_path / "sweep.csv"))
    click("Export CSV")
    assert (tmp_path / "sweep.csv").read_text(encoding="utf-8").startswith("Actuator [m],")

    _combo(a._redundancy, "bushings")
    click("Solve static")
    assert "solved" in status(win) and "as stiff bushings" in a._summary_model.as_string
    _combo(a._kind, "Joint reaction |F|")
    a._filter.model.set_value("BaseFixed")
    a.refresh_plot()
    weight = sum(md_mass(ctx.stage, p) for p in res["bodies"]) * 9.81
    assert a._plot.y_data[0][0] == pytest.approx(weight, rel=1e-4)
    assert math.isfinite(a._plot.y_max)


def md_mass(stage, path):
    from gomjb.physics_designer.core import modeling as md

    return md.body_mass_properties(stage.GetPrimAtPath(path)).mass
