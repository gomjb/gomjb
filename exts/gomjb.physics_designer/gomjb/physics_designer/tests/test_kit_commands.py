"""In-Kit tests (run with the Kit test runner, e.g. ``./isaac-sim.sh --enable omni.kit.test
--/exts/omni.kit.test/testExtsToRun/0=gomjb.physics_designer``). The Kit-independent core is
covered more thoroughly by ``tests_offline`` (plain pytest + usd-core)."""
import omni.kit.commands
import omni.kit.test
import omni.kit.undo
import omni.usd
from pxr import UsdGeom, UsdPhysics

from ..commands import run_edit
from ..core import joints as jt
from ..core import mechanisms as me
from ..core import transform as tf


class TestPhysicsDesignerCommands(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(self.stage, "/World")
        for name, x in (("A", 0.0), ("B", 2.0)):
            cube = UsdGeom.Cube.Define(self.stage, f"/World/{name}")
            UsdGeom.Xformable(cube).AddTranslateOp().Set((x, 0, 0))

    async def test_move_undo_redo(self):
        a = self.stage.GetPrimAtPath("/World/A")
        run_edit(lambda s: tf.translate([a], (0, 0, 1)), ["/World/A"], "move")
        self.assertAlmostEqual(tf.world_position(a)[2], 1.0)
        omni.kit.undo.undo()
        self.assertAlmostEqual(tf.world_position(a)[2], 0.0)
        omni.kit.undo.redo()
        self.assertAlmostEqual(tf.world_position(a)[2], 1.0)

    async def test_joint_undo(self):
        a = self.stage.GetPrimAtPath("/World/A")
        b = self.stage.GetPrimAtPath("/World/B")
        joint = run_edit(lambda s: jt.create_joint_at(s, jt.JointSpec("revolute"), a, b, "midpoint", "Z"))
        path = joint.GetPath()
        self.assertTrue(self.stage.GetPrimAtPath(path).IsA(UsdPhysics.RevoluteJoint))
        omni.kit.undo.undo()
        self.assertFalse(self.stage.GetPrimAtPath(path))

    async def test_template_and_error_rollback(self):
        res = run_edit(lambda s: me.build_scissor_lift(s, stages=2))
        self.assertTrue(self.stage.GetPrimAtPath(res["root"][0]))
        a = self.stage.GetPrimAtPath("/World/A")

        def failing(stage):
            tf.translate([a], (5, 0, 0))
            raise RuntimeError("boom")

        with self.assertRaises(RuntimeError):
            run_edit(failing, ["/World/A"])
        self.assertAlmostEqual(tf.world_position(a)[0], 0.0)
