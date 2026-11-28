"""Build a ready-to-simulate USD scene without Isaac Sim (only ``pip install usd-core``).

    python examples/offline_build_scene.py out.usda

Open ``out.usda`` in Isaac Sim and press Play: the scissor lift, hydraulic cylinder,
pendulum chain, door and slider are all wired with consistent joints.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from pxr import Usd, UsdGeom  # noqa: E402

from gomjb.physics_designer.core import mechanisms, physics, validation  # noqa: E402


def main(path: str) -> None:
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

    physics.ensure_physics_scene(stage)
    physics.create_ground_plane(stage, size=20.0)
    mechanisms.build_scissor_lift(stage, origin=(0, 0, 0), stages=3)
    mechanisms.build_hydraulic_cylinder(stage, origin=(2.5, 0, 0.3), direction=(1, 0, 1))
    mechanisms.build_pendulum_chain(stage, origin=(-2.5, 0, 2.0), links=6)
    mechanisms.build_hinged_door(stage, origin=(0, 2.5, 0))
    mechanisms.build_linear_slider(stage, origin=(0, -2.5, 0))

    print(validation.format_report(validation.validate(stage)))
    stage.GetRootLayer().Save()
    print(f"saved {path}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "physics_designer_demo.usda")
