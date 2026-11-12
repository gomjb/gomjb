"""The Kinematics window and kinematic mechanisms inside Kit (Isaac Sim 6.1, PhysX CPU)."""
import math

import omni.kit.app
import omni.kit.test
import omni.timeline
import omni.usd
from pxr import UsdGeom

from gomjb.physics_designer.commands import run_edit
from gomjb.physics_designer.core import mechanisms as me
from gomjb.physics_designer.core import modeling as md
from gomjb.physics_designer.core import motion as mo
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import transform as tf

from ..core import kinematic_mode as km
from ..extension import get_kinematic_runtime
from ..ui.window import KinematicsWindow


class TestKinematics(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)  # Isaac Sim default (the bare test app uses cm)
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, "/World").GetPrim())
        # omni.ui builds a window's frame lazily, on the first frame it is drawn
        self.window = KinematicsWindow("Kinematics Test")
        self.window.visible = True
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()
            if hasattr(self.window, "_kin_mode"):
                break

    async def tearDown(self):
        self.window.destroy()
        self.window = None

    def _status(self) -> str:
        return self.window._status_model.as_string

    async def test_exact_solver_sweep(self):
        """The exact solver ([python.pipapi] exudyn) against the virtual-work answer."""
        res = run_edit(lambda s: (ph.ensure_physics_scene(s),
                                  me.build_scissor_lift(s, stages=2, angle=30.0, loop_closure="rigid"))[-1])
        w = self.window
        w._root.model.set_value(res["root"][0])
        w.results._filter.model.set_value("Actuator")
        angles = (40.0, 55.0)
        w._sweep_from.value, w._sweep_to.value = [me.scissor_slider_offset(1.2, 30.0, a) for a in angles]
        w._sweep_points.model.set_value(2)
        w.sweep()  # empty drive field: both actuators together
        rec = w.recording
        self.assertEqual(len(rec), 2, self._status())
        self.assertIn("released redundant equations", w.results._summary_model.as_string)
        for k, deg in enumerate(angles):
            total = -(rec.channels["force:Actuator_L"][k] + rec.channels["force:Actuator_R"][k])
            expected = 2 * me.scissor_actuator_force(1.2, deg, 2, 20.0, 5.0, sides=2)
            self.assertAlmostEqual(total / expected, 1.0, delta=1e-6)

    async def _kinematic_lift(self, mode: str):
        """A lift (bushing loop closures) raised by one motion on both actuators, a box on its deck:
        in a kinematic mode the exact solver moves the lift and PhysX carries the box with it."""
        from isaacsim.core.experimental.prims import RigidPrim

        def build(s):
            ph.ensure_physics_scene(s)
            root = me.build_scissor_lift(s, stages=2, angle=30.0)["root"][0]
            for side in "LR":
                mo.set_motion(s.GetPrimAtPath(f"{root}/Joints/Actuator_{side}"),
                              mo.Motion("displacement", "STEP(TIME, 0, 0, 1, -0.3)"))
            deck = tf.world_position(s.GetPrimAtPath(root + "/Platform"))
            box = md.create_body(s, "/World/Box", "box", (0.2, 0.2, 0.2), mass=2.0)
            tf.set_world_position(box, (deck[0], deck[1], deck[2] + 0.02 + 0.1 + 0.002))
            return root

        root = run_edit(build)
        w = self.window
        w._root.model.set_value(root)
        w._kin_mode.widget.model.get_item_value_model().set_value(km.MODES.index(mode))
        w.apply_kinematics()
        self.assertIn(f"kinematics '{mode}'", self._status())
        self.assertEqual(km.get_mode(self.stage.GetPrimAtPath(root)), mode)
        box = self.stage.GetPrimAtPath("/World/Box")
        z_box = tf.world_position(box)[2]
        platform = RigidPrim(root + "/Platform")
        z_deck = float(platform.get_world_poses()[0].numpy()[0, 2])
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(150):  # STEP over 1 s, then hold
            await omni.kit.app.get_app().next_update_async()
        deck = float(platform.get_world_poses()[0].numpy()[0, 2])
        lifted = tf.world_position(box)[2]
        timeline.stop()
        reports = get_kinematic_runtime().reports
        self.assertTrue(any(("motion table" if mode == "table" else "co-simulation") in r for r in reports), reports)
        a = math.acos(math.cos(math.radians(30.0)) - 0.3 / 1.2)
        rise = 2 * 1.2 * (math.sin(a) - math.sin(math.radians(30.0)))
        self.assertAlmostEqual(deck - z_deck, rise, delta=1e-3)  # the exact kinematics
        self.assertAlmostEqual(lifted - z_box, rise, delta=0.01)  # the box rode up on the deck
        await omni.kit.app.get_app().next_update_async()
        w._kin_mode.widget.model.get_item_value_model().set_value(km.MODES.index("physics"))
        w.apply_kinematics()
        self.assertEqual(km.get_mode(self.stage.GetPrimAtPath(root)), "physics")

    async def test_kinematic_lift_from_motion_table(self):
        await self._kinematic_lift("table")

    async def test_kinematic_lift_by_cosimulation(self):
        await self._kinematic_lift("cosim")
