"""Drive the real Kinematics window callbacks with the Physics Designer's stubbed Kit modules on a
usd-core stage."""
import math
import pathlib
import sys

import pytest

pytest.importorskip("exudyn")

from pxr import UsdGeom  # noqa: E402

import kit_stubs  # noqa: E402  (gomjb.physics_designer/tests_offline, see conftest)

UI_DIR = pathlib.Path(__file__).resolve().parents[1] / "gomjb" / "kinematics" / "ui"


@pytest.fixture
def env(stage):
    ctx, commands = kit_stubs.install()
    for mod in [m for m in sys.modules if m.startswith("gomjb.")]:
        del sys.modules[mod]
    from gomjb.kinematics.ui.window import KinematicsWindow
    from gomjb.physics_designer import commands as dcmd

    dcmd.register()
    ctx.stage = stage
    kit_stubs._Widget.registry.clear()
    win = KinematicsWindow()
    yield win, ctx, commands
    for mod in [m for m in list(sys.modules) if m.startswith("gomjb.") or m.startswith("omni")
                or m.startswith("isaacsim") or m == "carb"]:
        del sys.modules[mod]


def click(text):
    btn = [w for w in kit_stubs._Widget.registry if isinstance(w, kit_stubs.Button) and w.args[0] == text]
    assert btn, f"no button {text!r}"
    btn[-1].kwargs["clicked_fn"]()


def status(win):
    return win._status_model.as_string


def _combo(combo, item):
    combo.widget.model.select(list(combo.items).index(item))


def _lift(stage, **kw):
    from gomjb.physics_designer.core import mechanisms as me
    from gomjb.physics_designer.core import physics as ph

    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    ph.ensure_physics_scene(stage)
    return me.build_scissor_lift(stage, angle=30.0, **kw)


def test_ui_sources_are_ascii():
    """Isaac Sim's default UI font has no Hangul (or arrow / degree) glyphs: they render as '?'."""
    for path in UI_DIR.glob("*.py"):
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            assert line.isascii(), f"{path.name}:{n}: {line.strip()}"


def test_every_button_runs_without_crashing(env):
    win, ctx, _ = env
    for w in list(kit_stubs._Widget.registry):
        if isinstance(w, kit_stubs.Button):
            w.kwargs["clicked_fn"]()
            assert "Set the mechanism" in status(win) or "Select a prim" in status(win) \
                or "no results" in status(win), f"{w.args[0]}: {status(win)}"


def test_exact_solver_sweep_and_bushings(env, tmp_path):
    from gomjb.physics_designer.core import mechanisms as me
    from gomjb.physics_designer.core import modeling as md

    win, ctx, _ = env
    res = _lift(ctx.stage, stages=1, sides=1, loop_closure="rigid")
    win._root.model.set_value(res["root"][0])
    targets = [me.scissor_slider_offset(1.2, 30.0, deg) for deg in (40.0, 50.0)]
    win._sweep_from.value, win._sweep_to.value = targets
    win._sweep_points.model.set_value(2)
    r = win.results
    r._filter.model.set_value("Actuator")
    click("Sweep drive")
    assert "Swept Actuator over 2 points" in status(win)
    assert r._plot.legends == ["Actuator"] and win.recording.x_label == "Actuator [m]"
    # this stroke runs downwards (raising the lift); XYPlot only draws increasing x
    assert targets[1] < targets[0] and r._plot.x_data[0] == sorted(targets)
    assert r._plot.y_data[0] == win.recording.channels["force:Actuator"][::-1]
    assert r._summary_model.as_string.startswith("Actuator: min")  # results first, report below
    for deg, force in zip((40.0, 50.0), win.recording.channels["force:Actuator"]):
        assert -force == pytest.approx(me.scissor_actuator_force(1.2, deg, 1, 20.0, 5.0, sides=1), rel=1e-6)
    assert "redundancy handling: release" in r._summary_model.as_string
    r._csv.model.set_value(str(tmp_path / "sweep.csv"))
    click("Export CSV")
    assert (tmp_path / "sweep.csv").read_text(encoding="utf-8").startswith("Actuator [m],")

    _combo(win._redundancy, "bushings")
    click("Solve static")
    assert "solved" in status(win) and "as stiff bushings" in r._summary_model.as_string
    _combo(r._kind, "Joint reaction |F|")
    r._filter.model.set_value("BaseFixed")
    r.refresh()
    weight = sum(md.body_mass_properties(ctx.stage.GetPrimAtPath(p)).mass for p in res["bodies"]) * 9.81
    assert r._plot.y_data[0][0] == pytest.approx(weight, rel=1e-4)
    assert math.isfinite(r._plot.y_max)


def test_simulate_follows_a_joint_motion(env):
    from gomjb.physics_designer.core import joints as jt
    from gomjb.physics_designer.core import modeling as md
    from gomjb.physics_designer.core import motion as mo
    from gomjb.physics_designer.core import physics as ph
    from gomjb.physics_designer.core import transform as tf

    win, ctx, _ = env
    stage = ctx.stage
    ph.ensure_physics_scene(stage)
    root = UsdGeom.Xform.Define(stage, "/World/Arm").GetPrim()
    ph.apply_articulation_root(root)
    link = md.create_body(stage, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
    tf.set_world_position(link, (0.5, 0, 0))
    joint = jt.create_joint_at(stage, jt.JointSpec("revolute", name="Hinge", axis="Z"), None, link,
                               location="custom", custom_position=(0, 0, 0), world_axis="Y",
                               parent_path="/World/Arm/Joints")
    mo.set_motion(joint, mo.Motion("displacement", "STEP(TIME, 0, 0, 0.5, 45)"))
    ctx.selection.paths = [str(link.GetPath())]
    click("Use selection")
    assert win._root.value == "/World/Arm"
    win._end_time.value = 1.0
    win.results._filter.model.set_value("Hinge")
    click("Simulate")
    assert "Simulated 1 s, 1 motion(s)." in status(win), status(win)
    rec = win.recording
    assert rec.x_name == "time" and abs(rec.channels["pos:Hinge"][-1]) == pytest.approx(45.0, abs=1e-6)
    assert abs(rec.channels["force:Hinge"][-1]) == pytest.approx(2 * 9.81 * 0.5 * math.cos(math.radians(45)),
                                                                  rel=1e-3)


def test_sweep_in_millimetres(env, tmp_path):
    """MMKS: From / To in mm, the result stays SI and shows in mm."""
    from gomjb.physics_designer.core import mechanisms as me

    win, ctx, _ = env
    _combo(win._unit_combo, "MMKS")
    assert win._sweep_from.unit_label.text == "mm | deg"
    res = _lift(ctx.stage, stages=1, sides=1, loop_closure="rigid")
    win._root.model.set_value(res["root"][0])
    targets = [me.scissor_slider_offset(1.2, 30.0, deg) for deg in (40.0, 50.0)]
    win._sweep_from.value, win._sweep_to.value = (1000.0 * t for t in targets)
    win._sweep_points.model.set_value(2)
    win.results._filter.model.set_value("Actuator")
    click("Sweep drive")
    assert win.recording.x_unit == "m" and win.recording.time == pytest.approx(targets)
    assert win.results._plot.x_data[0] == pytest.approx(sorted(1000.0 * t for t in targets))
    for deg, force in zip((40.0, 50.0), win.recording.channels["force:Actuator"]):
        assert -force == pytest.approx(me.scissor_actuator_force(1.2, deg, 1, 20.0, 5.0, sides=1), rel=1e-6)
    win.results._csv.model.set_value(str(tmp_path / "mm.csv"))
    click("Export CSV")
    assert (tmp_path / "mm.csv").read_text(encoding="utf-8").startswith("Actuator [mm],")


def test_apply_kinematics_builds_the_table_and_undoes(env, tmp_path, monkeypatch):
    from gomjb.kinematics import cache
    from gomjb.kinematics.core import kinematic_mode as km
    from gomjb.physics_designer.core import motion as mo

    win, ctx, commands = env
    monkeypatch.setattr(cache, "table_cache", lambda: str(tmp_path))
    res = _lift(ctx.stage, stages=2)
    root = res["root"][0]
    for side in "LR":
        mo.set_motion(ctx.stage.GetPrimAtPath(f"{root}/Joints/Actuator_{side}"), mo.Motion("displacement", "-0.1*TIME"))
    before = ctx.stage.GetRootLayer().ExportToString()
    win._root.model.set_value(root)
    _combo(win._kin_mode, "table")
    click("Apply kinematics")
    assert "kinematics 'table' - motion table" in status(win) and "built" in status(win), status(win)
    assert km.get_mode(ctx.stage.GetPrimAtPath(root)) == "table" and len(list(tmp_path.iterdir())) == 1
    click("Apply kinematics")  # the same model again: read from the cache
    assert "from the cache" in status(win)
    _combo(win._kin_mode, "physics")
    click("Apply kinematics")
    assert ctx.stage.GetRootLayer().ExportToString() == before
    commands.undo()
    assert km.get_mode(ctx.stage.GetPrimAtPath(root)) == "table"  # undo brings the kinematic mode back
