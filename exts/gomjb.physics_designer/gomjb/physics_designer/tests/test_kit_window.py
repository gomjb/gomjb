"""Build the real omni.ui window and drive its callbacks inside Kit."""
import omni.kit.app
import omni.kit.test
import omni.kit.undo
import omni.usd
from pxr import UsdGeom, UsdPhysics

from ..ui.window import PhysicsDesignerWindow


class TestPhysicsDesignerWindow(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)  # Isaac Sim default (the bare test app uses cm)
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, "/World").GetPrim())
        for name, x in (("A", 0.0), ("B", 2.0)):
            UsdGeom.Xformable(UsdGeom.Cube.Define(self.stage, f"/World/{name}")).AddTranslateOp().Set((x, 0, 0))
        # omni.ui builds a window's frame lazily, on the first frame it is drawn
        self.window = PhysicsDesignerWindow("Physics Designer Test")
        self.window.visible = True
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
            if hasattr(self.window, "_tpl"):
                break

    async def tearDown(self):
        self.window.destroy()
        self.window = None

    def _select(self, *paths):
        omni.usd.get_context().get_selection().set_selected_prim_paths(list(paths), True)

    async def test_window_builds_every_section(self):
        # template parameter fields live in a collapsed section and build when it is first expanded
        for attr in ("_move_step", "_grid", "_anchor_name", "_j_type", "_approx", "_tpl", "_validation_model"):
            self.assertTrue(hasattr(self.window, attr), attr)

    async def test_create_joint_from_selection(self):
        self._select("/World/A", "/World/B")
        self.window._create_joint()
        joints = [p for p in self.stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
        self.assertEqual(len(joints), 1, self.window._status_model.as_string)
        self.assertTrue(self.stage.GetPrimAtPath("/World/A").HasAPI(UsdPhysics.RigidBodyAPI))
        omni.kit.undo.undo()
        self.assertFalse([p for p in self.stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)])

    async def test_every_template_builds_and_validates(self):
        for i, name in enumerate(self.window._tpl.items):
            self.window._tpl.widget.model.get_item_value_model().set_value(i)
            await omni.kit.app.get_app().next_update_async()
            self.window._build_template()
            self.assertIn("done", self.window._status_model.as_string, name)
        self.window._validate()
        report = self.window._validation_model.as_string
        self.assertNotIn("[ERROR]", report)

    async def test_modeler_creates_link_with_mass_properties(self):
        from ..core import modeling as md

        m = self.window.modeler
        m._shape.widget.model.get_item_value_model().set_value(list(m._shape.items).index("link"))
        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
        m.create()
        body = self.stage.GetPrimAtPath(omni.usd.get_context().get_selection().get_selected_prim_paths()[0])
        self.assertEqual(md.body_params(body)["shape"], "link")
        self.assertGreater(md.body_mass_properties(body).mass, 0.0)
        self.assertTrue(body.GetChild("PinA") and body.GetChild("PinB"))

    async def test_analysis_records_actuator_forces(self):
        import math

        import omni.timeline

        from ..commands import run_edit
        from ..core import mechanisms as me
        from ..core import physics as ph

        res = run_edit(lambda s: (ph.ensure_physics_scene(s), me.build_scissor_lift(s, stages=2, angle=30.0))[-1])
        analysis = self.window.analysis
        analysis._root.model.set_value(res["root"][0])
        analysis.start()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(180):
            await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        analysis.stop()
        rec = analysis.recording
        self.assertGreater(len(rec), 60)
        pos = rec.channels["pos:Actuator_L"][-1]
        theta = math.degrees(math.acos(math.cos(math.radians(30.0)) + pos / 1.2))
        total = abs(rec.channels["force:Actuator_L"][-1]) + abs(rec.channels["force:Actuator_R"][-1])
        expected = 2 * me.scissor_actuator_force(1.2, theta, 2, 20.0, 5.0, sides=2)
        self.assertAlmostEqual(total / expected, 1.0, delta=0.02)
        self.assertIn("Actuator_L", analysis._summary_model.as_string)
