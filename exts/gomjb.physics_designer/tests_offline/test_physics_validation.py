from pxr import UsdGeom, UsdPhysics, UsdShade

from gomjb.physics_designer.core import joints as jt
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import snapping as sn
from gomjb.physics_designer.core import transform as tf
from gomjb.physics_designer.core import validation as va

from conftest import make_cube


def test_rigid_body_collision_material(stage):
    root = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
    mesh = UsdGeom.Mesh.Define(stage, "/World/Robot/Mesh").GetPrim()
    cube = make_cube(stage, "/World/Robot/Cube")
    anchor = sn.create_anchor(stage, root, "A")
    ph.apply_rigid_body([root], approximation="convexDecomposition")
    assert root.HasAPI(UsdPhysics.RigidBodyAPI)
    assert mesh.HasAPI(UsdPhysics.CollisionAPI) and cube.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.MeshCollisionAPI(mesh).GetApproximationAttr().Get() == "convexDecomposition"
    # anchor marker is never a collider
    assert not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse() if p.GetPath().HasPrefix(anchor.GetPath()))
    mat = ph.create_physics_material(stage, static_friction=0.9, restitution=0.2)
    ph.bind_physics_material([root], mat)
    bound, _ = UsdShade.MaterialBindingAPI(cube).ComputeBoundMaterial("physics")
    assert bound.GetPrim().GetPath() == mat.GetPath()
    ph.set_mass([root], mass=12.5)
    assert UsdPhysics.MassAPI(root).GetMassAttr().Get() == 12.5
    n = ph.remove_physics([root])
    assert n > 0 and not root.HasAPI(UsdPhysics.RigidBodyAPI) and not cube.HasAPI(UsdPhysics.CollisionAPI)


def test_scene_and_ground(stage):
    scene = ph.ensure_physics_scene(stage)
    assert ph.ensure_physics_scene(stage).GetPath() == scene.GetPath()
    assert tuple(UsdPhysics.Scene(scene).GetGravityDirectionAttr().Get()) == (0, 0, -1)
    ground = ph.create_ground_plane(stage, height=0.0)
    assert abs(tf.world_bbox(ground).GetMax()[2]) < 1e-9


def test_validation_detects_and_fixes(stage):
    a = make_cube(stage, "/World/A")
    b = make_cube(stage, "/World/A/B", pos=(0, 0, 2))
    ph.apply_rigid_body([a])
    UsdPhysics.RigidBodyAPI.Apply(b)  # nested rigid body
    c = make_cube(stage, "/World/C", pos=(3, 0, 0))
    UsdPhysics.RigidBodyAPI.Apply(c)  # no collider
    joint = jt.create_joint(stage, jt.JointSpec("revolute"), a, c, (1.5, 0, 0))
    tf.translate([c], (0, 1, 0))
    mesh = UsdGeom.Mesh.Define(stage, "/World/C/M").GetPrim()
    UsdPhysics.CollisionAPI.Apply(mesh)
    issues = va.validate(stage)
    msgs = va.format_report(issues)
    assert "Nested rigid body" in msgs
    assert "No PhysicsScene" in msgs
    assert "Triangle-mesh collider" in msgs
    assert "outside any articulation" in msgs
    frame_issue = [i for i in issues if i.fix == "realign_joint"]
    assert frame_issue and frame_issue[0].path == str(joint.GetPath())
    for i in issues:
        va.apply_fix(stage, i)
    remaining = va.validate(stage)
    assert not [i for i in remaining if i.fix in ("realign_joint", "add_scene", "convex_hull")]


def test_created_scene_runs_on_cpu(stage):
    scene = ph.ensure_physics_scene(stage)
    # usd-core has no PhysxSchema registered, so check the authored list op itself
    assert "PhysxSceneAPI" in scene.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
    assert not ph.uses_gpu_dynamics(scene)
    assert scene.GetAttribute("physxScene:broadphaseType").Get() == "MBP"


def test_gpu_dynamics_with_closed_loop_articulation_is_flagged(stage):
    from gomjb.physics_designer.core import mechanisms as me

    me.build_scissor_lift(stage)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()  # PhysX default: GPU dynamics
    assert ph.uses_gpu_dynamics(scene)
    gpu = [i for i in va.validate(stage) if i.fix == "cpu_dynamics"]
    assert len(gpu) == 1 and gpu[0].path == "/World/PhysicsScene"
    va.apply_fix(stage, gpu[0])
    assert not [i for i in va.validate(stage) if i.fix == "cpu_dynamics"]


def test_open_chain_articulation_on_gpu_is_not_flagged(stage):
    from gomjb.physics_designer.core import mechanisms as me

    me.build_pendulum_chain(stage)
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    assert not [i for i in va.validate(stage) if i.fix == "cpu_dynamics"]
