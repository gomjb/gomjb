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
        for attr in ("_move_step", "_grid", "_anchor_name", "_approx", "_tpl", "_validation_model"):
            self.assertTrue(hasattr(self.window, attr), attr)
        self.assertEqual(set(self.window.joints._type_buttons), {"Fixed", "Revolute", "Translational", "Spherical"})

    async def test_recurdyn_joint_base_action_point(self):
        """Revolute (Body, Body, Point): base A, action B, point; the joint is created at the last pick."""
        from ..core import physics as ph

        a, b = (self.stage.GetPrimAtPath(p) for p in ("/World/A", "/World/B"))
        ph.apply_rigid_body([a, b])
        joints = self.window.joints
        joints.start("Revolute")
        joints.pick(a)
        joints.pick(b)
        joints.pick(b, (1.0, 0.0, 0.0))
        found = [p for p in self.stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
        self.assertEqual(len(found), 1, self.window._status_model.as_string)
        rel = UsdPhysics.Joint(found[0])
        self.assertEqual([rel.GetBody0Rel().GetTargets()[0], rel.GetBody1Rel().GetTargets()[0]], [a.GetPath(), b.GetPath()])
        self.assertIsNone(joints.picker)
        omni.kit.undo.undo()
        self.assertFalse([p for p in self.stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)])

    async def test_motion_drives_the_physx_simulation(self):
        """A revolute joint with a STEP motion 0 -> 45 deg in 0.5 s: PhysX follows it (motion runtime)."""
        import omni.timeline

        from ..commands import run_edit
        from ..core import joints as jt
        from ..core import modeling as md
        from ..core import motion as mo
        from ..core import physics as ph
        from ..core import transform as tf

        def build(stage):
            ph.ensure_physics_scene(stage)
            root = UsdGeom.Xform.Define(stage, "/World/Arm").GetPrim()
            ph.apply_articulation_root(root)
            base = md.create_body(stage, "/World/Arm/Base", "box", (0.2, 0.2, 0.2), mass=5.0)
            tf.set_world_position(base, (0, 0, 1))
            jt.create_joint(stage, jt.JointSpec("fixed", name="BaseFixed"), None, base, (0, 0, 1),
                            parent_path="/World/Arm/Joints")
            link = md.create_body(stage, "/World/Arm/Link", "link", (1.0, 0.05, 0.02), mass=2.0)
            tf.set_world_position(link, (0.5, -0.15, 1))
            hinge = jt.create_joint_at(stage, jt.JointSpec("revolute", name="Hinge", axis="Z"), base, link,
                                       location="custom", custom_position=(0, -0.15, 1), world_axis="Y",
                                       parent_path="/World/Arm/Joints")
            mo.set_motion(hinge, mo.Motion("displacement", "STEP(TIME, 0, 0, 0.5, 45)"))

        run_edit(build)
        analysis = self.window.analysis
        analysis._root.model.set_value("/World/Arm")
        analysis.start()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(150):
            await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        analysis.stop()
        rec = analysis.recording
        self.assertGreater(rec.time[-1], 1.0)
        self.assertAlmostEqual(abs(rec.at(rec.time[-1], "pos:Hinge")), 45.0, delta=0.5)
        self.assertAlmostEqual(abs(rec.at(0.25, "pos:Hinge")), 22.5, delta=2.0)  # mid-STEP
        await omni.kit.app.get_app().next_update_async()
        hinge = self.stage.GetPrimAtPath("/World/Arm/Joints/Hinge")
        self.assertAlmostEqual(UsdPhysics.DriveAPI(hinge, "angular").GetTargetPositionAttr().Get(), 0.0)  # Stop

    async def test_scissor_stroke_ends_on_its_joint_limit(self):
        """The actuators are driven past the lift's range: the actuator joint limits (no arm contact)
        hold it at the 75 deg end of the stroke, still and at the exact kinematic height."""
        import math

        import omni.timeline

        from ..commands import run_edit
        from ..core import mechanisms as me
        from ..core import motion as mo
        from ..core import physics as ph

        lower = me.scissor_slider_offset(1.2, 30.0, 75.0)  # the actuators' lower limit (m)

        def build(stage):
            ph.ensure_physics_scene(stage)
            root = me.build_scissor_lift(stage, stages=2, angle=30.0)["root"][0]
            for side in "LR":
                mo.set_motion(stage.GetPrimAtPath(f"{root}/Joints/Actuator_{side}"),
                              mo.Motion("displacement", f"STEP(TIME, 0, 0, 2, {lower - 0.15})"))
            return root

        root = run_edit(build)
        analysis = self.window.analysis
        analysis._root.model.set_value(root)
        analysis.start()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(240):  # STEP over 2 s past the limit, then hold
            await omni.kit.app.get_app().next_update_async()
        timeline.stop()
        analysis.stop()
        rec = analysis.recording
        end = rec.time[-1]
        self.assertGreater(end, 3.0)
        for side in "LR":
            pos = [rec.at(t, f"pos:Actuator_{side}") for t in (end - 0.5, end)]
            self.assertTrue(all(math.isfinite(p) for p in pos), pos)
            self.assertAlmostEqual(pos[-1], lower, delta=0.002)  # stopped by the limit, not the drive target
            self.assertAlmostEqual(pos[0], pos[1], delta=5e-4)  # resting, not chattering on the stop

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
        self.assertIn("Actuator_L", analysis.results._summary_model.as_string)
