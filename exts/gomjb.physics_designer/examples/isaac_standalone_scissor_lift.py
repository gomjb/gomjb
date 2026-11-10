"""Isaac Sim standalone script: build a scissor lift and raise it by driving the actuator.

    ./python.sh <repo>/exts/gomjb.physics_designer/examples/isaac_standalone_scissor_lift.py

Works with Isaac Sim 4.x / 5.x (uses only SimulationApp, omni.usd, omni.timeline and USD).
"""
import os
import sys

try:
    from isaacsim import SimulationApp  # Isaac Sim >= 4.0
except ImportError:  # pragma: no cover - older releases
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": "--headless" in sys.argv})

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom, UsdPhysics  # noqa: E402

from gomjb.physics_designer.core import mechanisms, physics, transform, validation  # noqa: E402

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

physics.ensure_physics_scene(stage)
physics.create_ground_plane(stage)
ARM, ANGLE, STAGES = 1.2, 20.0, 3
lift = mechanisms.build_scissor_lift(stage, stages=STAGES, arm_length=ARM, angle=ANGLE)
print(validation.format_report(validation.validate(stage)))

actuators = [stage.GetPrimAtPath(p) for p in lift["joints"] if "/Actuator" in p]
platform = stage.GetPrimAtPath(lift["root"][0] + "/Platform")

timeline = omni.timeline.get_timeline_interface()
timeline.play()
target_height = 2.2
final = mechanisms.scissor_slider_offset_for_height(ARM, ANGLE, STAGES, target_height)
steps = 600
for i in range(steps + 240):
    alpha = min(1.0, i / steps)
    for joint in actuators:
        UsdPhysics.DriveAPI(joint, "linear").GetTargetPositionAttr().Set(final * alpha)
    simulation_app.update()
    if i % 120 == 0:
        print(f"step {i:4d}  platform z = {transform.world_position(platform)[2]:.3f}")

timeline.stop()
simulation_app.close()
