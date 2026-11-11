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
