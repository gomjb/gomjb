"""Isaac Sim standalone script: build a scissor lift and raise it to a target height.

    python.bat <repo>/exts/gomjb.physics_designer/examples/isaac_standalone_scissor_lift.py [--headless]

Targets Isaac Sim 6 (Kit 110): runtime control and readback go through the
``isaacsim.core.experimental`` tensor API — authored USD values are not the simulated state.
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--height", type=float, default=2.2, help="target pin height of the lift (m)")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.experimental.prims import Articulation, RigidPrim  # noqa: E402
from pxr import UsdGeom  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from gomjb.physics_designer.core import mechanisms, physics, validation  # noqa: E402

stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

physics.ensure_physics_scene(stage)
physics.create_ground_plane(stage)
ARM, ANGLE, STAGES = 1.2, 20.0, 3
lift = mechanisms.build_scissor_lift(stage, stages=STAGES, arm_length=ARM, angle=ANGLE)
print(validation.format_report(validation.validate(stage)))

robot = Articulation(lift["root"][0])
platform = RigidPrim(lift["root"][0] + "/Platform")
actuators = [i for i, name in enumerate(robot.dof_names) if name.startswith("Actuator")]
target = mechanisms.scissor_slider_offset_for_height(ARM, ANGLE, STAGES, args.height)
print(f"actuator target for {args.height:.2f} m: {target:.4f} m")

app_utils.play()
simulation_app.update()
ramp = 600
for i in range(ramp + 240):
    alpha = min(1.0, i / ramp)
    robot.set_dof_position_targets([[target * alpha] * len(actuators)], dof_indices=actuators)
    simulation_app.update()
    if i % 120 == 0:
        z = platform.get_world_poses()[0].numpy()[0, 2]
        print(f"step {i:4d}  platform z = {z:.3f} m")

app_utils.stop()
simulation_app.close()
