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
                                           "select bodies created by the modeler",
                                           "select revolute / translational joints")
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


def test_recurdyn_joint_base_action_point(env):
    """Revolute (Body, Body, Point): click base, action, point; the joint appears at the last pick."""
    win, ctx, commands = env
    from gomjb.physics_designer.core import physics as ph
    from gomjb.physics_designer.core import joints as jt

    a, b = (ctx.stage.GetPrimAtPath(p) for p in ("/World/A", "/World/B"))
    ph.apply_rigid_body([a, b])
    j = win.joints
    click("Revolute")
    assert "Revolute (Body, Body, Point): click the BASE body" in j._prompt_model.as_string
    j.on_viewport_pick("/World/A", (0.0, 0.03, 0.5))  # a viewport click on A
    assert "ACTION" in j._prompt_model.as_string and "Base: A" in j._picks_model.as_string
    j.on_viewport_pick("/World/B", (2.0, 0.03, 0.5))
    j._limit.model.set_value(True)
    j._drive.model.set_value(True)
    j._snap.widget.model.select(list(j._snap.items).index("Surface"))
    j.on_viewport_pick("/World/B", (1.0, 0.03, 0.0))  # the point: the joint is created now
    joint_path = ctx.selection.paths[0]
    joint = ctx.stage.GetPrimAtPath(joint_path)
    assert joint.IsA(UsdPhysics.RevoluteJoint) and UsdPhysics.RevoluteJoint(joint).GetAxisAttr().Get() == "Z"
    assert [p.GetPath() for p in jt.joint_bodies(joint)] == [a.GetPath(), b.GetPath()]  # base, action
    assert list(jt.joint_world_frames(joint)[0][0]) == pytest.approx([1.0, 0.03, 0.0])
    assert UsdPhysics.RevoluteJoint(joint).GetLowerLimitAttr().Get() == -90
    assert joint.HasAPI(UsdPhysics.DriveAPI, "angular")
    assert j.picker is None and "Created" in j._prompt_model.as_string
    commands.undo()
    assert not ctx.stage.GetPrimAtPath(joint_path)


def test_joint_tool_makes_a_loop_closure_a_bushing(env):
    """Auto joint model: A and B are both fixed to the world, so a revolute between them closes a
    loop and is created as an auto-sized bushing; 'Rigid joint' keeps a revolute; undo works."""
    win, ctx, commands = env
    from gomjb.physics_designer.core import joints as jt
    from gomjb.physics_designer.core import physics as ph

    a, b = (ctx.stage.GetPrimAtPath(p) for p in ("/World/A", "/World/B"))
    ph.apply_rigid_body([a, b])
    for body in (a, b):
        jt.create_joint(ctx.stage, jt.JointSpec("fixed"), None, body, tf_pos(body))
    j = win.joints
    assert j._model.value.startswith("Auto")

    def revolute():
        click("Revolute")
        j._snap.widget.model.select(list(j._snap.items).index("Surface"))
        j.on_viewport_pick("/World/A", (0.0, 0.03, 0.5))
        j.on_viewport_pick("/World/B", (2.0, 0.03, 0.5))
        j.on_viewport_pick("/World/B", (1.0, 0.03, 0.0))
        return ctx.stage.GetPrimAtPath(ctx.selection.paths[0])

    joint = revolute()
    assert jt.is_bushing(joint) and "bushing" in j._prompt_model.as_string
    assert "closes a loop: bushing, k =" in status(win)
    assert not joint.HasAPI(UsdPhysics.DriveAPI, "rotZ") and joint.HasAPI(UsdPhysics.DriveAPI, "transX")
    path = joint.GetPath()
    commands.undo()
    assert not ctx.stage.GetPrimAtPath(path)

    j._model.widget.model.select(list(j._model.items).index("Rigid joint"))
    rigid = revolute()
    assert rigid.IsA(UsdPhysics.RevoluteJoint) and not jt.is_bushing(rigid)
    # convert it afterwards from the selection; undo restores the revolute
    select(ctx, str(rigid.GetPath()))
    click("Convert selected to bushing")
    assert "done" in status(win), status(win)
    assert jt.is_bushing(ctx.stage.GetPrimAtPath(rigid.GetPath()))
    commands.undo()
    assert ctx.stage.GetPrimAtPath(rigid.GetPath()).IsA(UsdPhysics.RevoluteJoint)


def tf_pos(prim):
    from gomjb.physics_designer.core import transform as tf

    return tf.world_position(prim)


def test_motion_apply_load_remove(env):
    """Include Motion on a joint from the window (the exact solver's run of it: gomjb.kinematics)."""
    win, ctx, commands = env
    from gomjb.physics_designer.core import joints as jt
    from gomjb.physics_designer.core import modeling as md
    from gomjb.physics_designer.core import motion as mo
    from gomjb.physics_designer.core import physics as ph
    from gomjb.physics_designer.core import transform as tf

    stage = ctx.stage
    ph.ensure_physics_scene(stage)
    root = UsdGeom.Xform.Define(stage, "/World/Arm").GetPrim()
    ph.apply_articulation_root(root)
    link = md.create_body(stage, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (0.5, 0, 0))
    joint = jt.create_joint_at(stage, jt.JointSpec("revolute", name="Hinge", axis="Z"), None, link,
                               location="custom", custom_position=(0, 0, 0), world_axis="Y",
                               parent_path="/World/Arm/Joints")
    j = win.joints
    _combo(j._motion_type, "Displacement (time)")
    j._motion_expr.model.set_value("STEP(TIME, 0, 0, 0.5, 45)")
    select(ctx, str(joint.GetPath()))
    click("Apply motion")
    assert mo.get_motion(joint).expression == "STEP(TIME, 0, 0, 0.5, 45)", status(win)
    j._motion_expr.model.set_value("")
    click("Load motion")
    assert j._motion_expr.value == "STEP(TIME, 0, 0, 0.5, 45)" and j._motion_on.value

    j._motion_on.model.set_value(False)
    click("Apply motion")
    assert mo.get_motion(joint) is None
    commands.undo()
    assert mo.get_motion(joint) is not None  # undo brings the motion back


def test_recurdyn_translational_with_direction_and_ground(env):
    """Translational (Body, Body, Point, Direction) from the ground; the axis from the +Z button."""
    win, ctx, _ = env
    from gomjb.physics_designer.core import physics as ph
    from gomjb.physics_designer.core import joints as jt

    ph.apply_rigid_body([ctx.stage.GetPrimAtPath("/World/B")])
    j = win.joints
    click("Translational")
    assert j.picker.method == "Body, Body, Point, Direction"
    j.on_viewport_pick("", None)  # empty space: the ground
    j.on_viewport_pick("/World/B", (2.0, 0.03, 0.0))
    select(ctx, "/World/B")
    click("Pick from selection")  # the point from the selection: B's origin
    assert "Z axis" in j._prompt_model.as_string and "Base: ground" in j._picks_model.as_string
    click("+Z")
    joint = ctx.stage.GetPrimAtPath(ctx.selection.paths[0])
    assert joint.IsA(UsdPhysics.PrismaticJoint)
    b0, b1 = jt.joint_bodies(joint)
    assert b0 is None and b1.GetPath() == "/World/B"
    rot = jt.joint_world_frames(joint)[0][1]
    from pxr import Gf

    assert list(Gf.Rotation(rot).TransformDir(Gf.Vec3d(0, 0, 1))) == pytest.approx([0, 0, 1])


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
    win.analysis.results.result = rec
    win.analysis.results._filter.model.set_value("Actuator")
    win.analysis.results.refresh()
    plot = win.analysis.results._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Actuator_L"] and plot.y_data[0][-1] == -1003.0
    assert plot.y_min < -1003.0 and plot.y_max > -1000.0 and (plot.x_min, plot.x_max) == (0.0, 3 / 60)
    assert "Actuator_L: min -1003  max -1000  final -1003 N" in win.analysis.results._summary_model.as_string
    from gomjb.physics_designer.ui.result_view import SERIES_COLORS

    assert plot.colors == [SERIES_COLORS[0]]
    win.analysis.results._filter.model.set_value("")
    win.analysis.results.refresh()
    plot = win.analysis.results._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Actuator_L", "Hinge"] and plot.colors == SERIES_COLORS[:2]
    win.analysis.results._filter.model.set_value("Hinge")
    win.analysis.results.refresh()
    plot = win.analysis.results._plot  # the plot is rebuilt on every refresh
    assert plot.colors == [SERIES_COLORS[1]]  # the survivor keeps its color
    _combo(win.analysis.results._kind, "Joint reaction |F|")
    win.analysis.results._filter.model.set_value("")
    win.analysis.results.refresh()
    plot = win.analysis.results._plot  # the plot is rebuilt on every refresh
    assert plot.legends == ["Platform"] and plot.y_data[0][0] == pytest.approx(5.0)
    win.analysis.results._csv.model.set_value(str(tmp_path / "out.csv"))
    click("Export CSV")
    assert (tmp_path / "out.csv").read_text(encoding="utf-8").startswith("time [s],pos:Actuator_L")


def test_unit_system_mmks_everywhere(env, tmp_path):
    """MMKS: inputs show mm and keep their physical value; the stage (metres) gets the same geometry."""
    win, ctx, _ = env
    from gomjb.physics_designer.core import mechanisms as me
    from gomjb.physics_designer.core import modeling as md
    from gomjb.physics_designer.core import physics as ph
    from gomjb.physics_designer.core import transform as tf

    assert win._move_step.unit_label.text == "m" and win._move_step.value == pytest.approx(0.1)
    _combo(win._unit_combo, "MMKS")
    assert "Units: MMKS (mm, kg, N, s, deg)" in status(win)
    assert kit_stubs.sys.modules["carb.settings"].get_settings().get(
        "/persistent/exts/gomjb.physics_designer/unitSystem") == "MMKS"
    assert win._move_step.unit_label.text == "mm" and win._move_step.value == pytest.approx(100.0)
    select(ctx, "/World/A")
    click("X +")
    assert tf.world_position(ctx.stage.GetPrimAtPath("/World/A"))[0] == pytest.approx(0.1)  # 100 mm
    click("Get from selection")
    assert win._abs_pos.value[0] == pytest.approx(100.0)

    # joint inputs follow the joint type: degrees for revolute, lengths for translational
    j = win.joints
    assert j._lower.unit_label.text == "deg" and j._stiff.unit_label.text == "N*mm/deg"
    click("Translational")
    assert j._lower.unit_label.text == "mm" and j._stiff.unit_label.text == "N/mm"
    assert j._stiff.value == pytest.approx(1.0)  # the SI number stays: 1000 N*m/deg -> 1000 N/m = 1 N/mm
    click("Revolute")
    assert j._stiff.value == pytest.approx(1.0e6) and j._stiff.unit_label.text == "N*mm/deg"
    j.cancel()

    # modeler: a 500 x 300 x 200 mm box is 0.5 x 0.3 x 0.2 m on the stage
    m = win.modeler
    assert [d.value for d in m._dim_inputs] == pytest.approx([500.0, 300.0, 200.0])
    m._density.value = 7.85e-6  # kg/mm^3 = steel
    select(ctx)
    m.create()
    box = ctx.stage.GetPrimAtPath(ctx.selection.paths[0])
    assert md.body_params(box)["dims"] == pytest.approx([0.5, 0.3, 0.2])
    assert md.body_mass_properties(box, si=True).mass == pytest.approx(7850 * 0.5 * 0.3 * 0.2)
    assert "kg*mm^2" in status(win)

    # templates: the builders' metre defaults show in mm and build the same lift
    _combo(win._tpl, "Scissor lift")
    win._tpl_frame.rebuild()
    kind, arm = win._template_inputs["arm_length"]
    assert arm.unit_label.text == "mm" and arm.value == pytest.approx(1200.0)
    assert win._template_inputs["arm_mass"][1].unit_label.text == "kg"

    # back to MKS: the same physical values
    _combo(win._unit_combo, "MKS")
    assert win._move_step.value == pytest.approx(0.1) and win._abs_pos.value[0] == pytest.approx(0.1)


def test_unit_system_on_a_centimetre_stage(env):
    """Display units are independent of the stage scale: 100 mm on a cm stage is 10 stage units."""
    win, ctx, _ = env
    from gomjb.physics_designer.core import transform as tf

    UsdGeom.SetStageMetersPerUnit(ctx.stage, 0.01)
    _combo(win._unit_combo, "MMKS")
    select(ctx, "/World/A")
    click("Z +")
    assert tf.world_position(ctx.stage.GetPrimAtPath("/World/A"))[2] == pytest.approx(10.0)
