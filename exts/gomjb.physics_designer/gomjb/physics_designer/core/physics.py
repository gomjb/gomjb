"""Rigid bodies, colliders, mass, physics materials, articulations and scene setup."""
from __future__ import annotations

from typing import Iterable, List, Optional, Sequence

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from . import transform as tf
from .snapping import is_anchor
from .stage_utils import default_root, is_guide, stage_up_axis, unique_child_path

APPROXIMATIONS = (
    "none",  # triangle mesh (static only in PhysX)
    "convexHull",
    "convexDecomposition",
    "meshSimplification",
    "boundingCube",
    "boundingSphere",
)
_IMPLICIT_SHAPES = (UsdGeom.Cube, UsdGeom.Sphere, UsdGeom.Capsule, UsdGeom.Cylinder, UsdGeom.Cone, UsdGeom.Plane)


def _geometry_prims(root: Usd.Prim) -> List[Usd.Prim]:
    """Collidable geometry under ``root`` (skips guide prims and anchor markers)."""
    out = []
    it = iter(Usd.PrimRange(root))
    for prim in it:
        if is_anchor(prim) or is_guide(prim):
            it.PruneChildren()
            continue
        if prim.IsA(UsdGeom.Mesh) or any(prim.IsA(s) for s in _IMPLICIT_SHAPES):
            out.append(prim)
    return out


# ---------------------------------------------------------------------------------------
# Rigid bodies
# ---------------------------------------------------------------------------------------
def apply_rigid_body(prims: Iterable[Usd.Prim], kinematic: bool = False, enabled: bool = True,
                     add_colliders: bool = True, approximation: str = "convexHull") -> List[Usd.Prim]:
    out = []
    for prim in tf._top_level(prims):
        api = UsdPhysics.RigidBodyAPI.Apply(prim)
        api.CreateRigidBodyEnabledAttr().Set(bool(enabled))
        api.CreateKinematicEnabledAttr().Set(bool(kinematic))
        if add_colliders:
            apply_collision([prim], approximation=approximation)
        out.append(prim)
    return out


def set_kinematic(prims: Iterable[Usd.Prim], kinematic: bool) -> None:
    for prim in prims:
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(bool(kinematic))


def set_velocity(prim: Usd.Prim, linear=(0, 0, 0), angular=(0, 0, 0)) -> None:
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    api.CreateVelocityAttr().Set(Gf.Vec3f(*linear))
    api.CreateAngularVelocityAttr().Set(Gf.Vec3f(*angular))


# ---------------------------------------------------------------------------------------
# Collision
# ---------------------------------------------------------------------------------------
def apply_collision(prims: Iterable[Usd.Prim], approximation: str = "convexHull",
                    recursive: bool = True) -> List[Usd.Prim]:
    """Add CollisionAPI to geometry (``recursive``: every geometry prim in the subtree).

    Meshes also get MeshCollisionAPI with the requested approximation.  Use "none" only for
    static geometry: PhysX does not support triangle meshes on dynamic bodies.
    """
    if approximation not in APPROXIMATIONS:
        raise ValueError(f"approximation must be one of {APPROXIMATIONS}")
    targets: List[Usd.Prim] = []
    for prim in prims:
        targets.extend(_geometry_prims(prim) if recursive else [prim])
    for g in targets:
        UsdPhysics.CollisionAPI.Apply(g).CreateCollisionEnabledAttr().Set(True)
        if g.IsA(UsdGeom.Mesh):
            UsdPhysics.MeshCollisionAPI.Apply(g).CreateApproximationAttr().Set(approximation)
    return targets


def set_collision_enabled(prims: Iterable[Usd.Prim], enabled: bool) -> None:
    for prim in prims:
        for g in _geometry_prims(prim):
            if g.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(g).CreateCollisionEnabledAttr().Set(bool(enabled))


def filter_collisions(prim_a: Usd.Prim, prim_b: Usd.Prim) -> None:
    """Disable collisions between two specific bodies/colliders (FilteredPairsAPI)."""
    UsdPhysics.FilteredPairsAPI.Apply(prim_a).CreateFilteredPairsRel().AddTarget(prim_b.GetPath())


def create_collision_group(stage: Usd.Stage, name: str, members: Sequence[Usd.Prim],
                           filtered_groups: Sequence[Sdf.Path] = (), parent: Optional[Sdf.Path] = None,
                           ) -> Usd.Prim:
    parent = parent or default_root(stage).AppendChild("CollisionGroups")
    if not stage.GetPrimAtPath(parent):
        UsdGeom.Scope.Define(stage, parent)
    group = UsdPhysics.CollisionGroup.Define(stage, unique_child_path(stage, parent, name))
    col = group.CreateCollidersCollectionAPI()
    for m in members:
        col.CreateIncludesRel().AddTarget(m.GetPath())
    rel = group.CreateFilteredGroupsRel()
    for g in filtered_groups:
        rel.AddTarget(g)
    return group.GetPrim()


# ---------------------------------------------------------------------------------------
# Mass
# ---------------------------------------------------------------------------------------
def set_mass(prims: Iterable[Usd.Prim], mass: Optional[float] = None, density: Optional[float] = None,
             center_of_mass=None, diagonal_inertia=None) -> None:
    """MassAPI. Give either ``mass`` (kg) or ``density`` (kg / stage-unit^3)."""
    for prim in prims:
        api = UsdPhysics.MassAPI.Apply(prim)
        if mass is not None:
            api.CreateMassAttr().Set(float(mass))
        if density is not None:
            api.CreateDensityAttr().Set(float(density))
        if center_of_mass is not None:
            api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*center_of_mass))
        if diagonal_inertia is not None:
            api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*diagonal_inertia))


# ---------------------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------------------
def create_physics_material(stage: Usd.Stage, name: str = "PhysicsMaterial", static_friction: float = 0.5,
                            dynamic_friction: float = 0.5, restitution: float = 0.0,
                            density: Optional[float] = None, parent: Optional[Sdf.Path] = None) -> Usd.Prim:
    parent = parent or default_root(stage).AppendChild("PhysicsMaterials")
    if not stage.GetPrimAtPath(parent):
        UsdGeom.Scope.Define(stage, parent)
    mat = UsdShade.Material.Define(stage, unique_child_path(stage, parent, name))
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(float(static_friction))
    api.CreateDynamicFrictionAttr().Set(float(dynamic_friction))
    api.CreateRestitutionAttr().Set(float(restitution))
    if density is not None:
        api.CreateDensityAttr().Set(float(density))
    return mat.GetPrim()


def bind_physics_material(prims: Iterable[Usd.Prim], material: Usd.Prim, recursive: bool = True) -> None:
    """Bind with the "physics" purpose so render materials are untouched."""
    mat = UsdShade.Material(material)
    for prim in prims:
        targets = _geometry_prims(prim) if recursive else [prim]
        for g in targets or [prim]:
            UsdShade.MaterialBindingAPI.Apply(g).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics"
            )


# ---------------------------------------------------------------------------------------
# Articulations / scene
# ---------------------------------------------------------------------------------------
def apply_articulation_root(prim: Usd.Prim) -> None:
    UsdPhysics.ArticulationRootAPI.Apply(prim)


def ensure_physics_scene(stage: Usd.Stage, path: Optional[str] = None, gravity: float = 9.81) -> Usd.Prim:
    """Return the first PhysicsScene or create one with gravity along -up (in stage units).

    A created scene runs PhysX on the CPU (see :func:`set_cpu_dynamics`).
    """
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            return prim
    path = path or str(default_root(stage).AppendChild("PhysicsScene"))
    scene = UsdPhysics.Scene.Define(stage, path)
    up = stage_up_axis(stage)
    direction = Gf.Vec3f(0, 0, -1) if up == "Z" else Gf.Vec3f(0, -1, 0)
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    scene.CreateGravityDirectionAttr().Set(direction)
    scene.CreateGravityMagnitudeAttr().Set(gravity / mpu)
    set_cpu_dynamics(scene.GetPrim())
    return scene.GetPrim()


GPU_DYNAMICS_ATTR = "physxScene:enableGPUDynamics"  # PhysxSchema fallback: True


def set_cpu_dynamics(scene: Usd.Prim) -> None:
    """Run PhysX on the CPU (CPU dynamics + MBP broadphase).

    Mechanism scenes are small but constraint-heavy; in Isaac Sim 6.1 the GPU solver diverged
    when a closed-loop articulation (scissor lift) shared the scene with other articulations,
    while the CPU solver stays exact.  Authored as plain PhysxSceneAPI attributes so it also
    works with ``usd-core``, which ships without ``PhysxSchema``.
    """
    scene.AddAppliedSchema("PhysxSceneAPI")
    scene.CreateAttribute(GPU_DYNAMICS_ATTR, Sdf.ValueTypeNames.Bool).Set(False)
    scene.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set("MBP")


def uses_gpu_dynamics(scene: Usd.Prim) -> bool:
    attr = scene.GetAttribute(GPU_DYNAMICS_ATTR)
    return True if not attr or attr.Get() is None else bool(attr.Get())


def create_ground_plane(stage: Usd.Stage, path: Optional[str] = None, size: float = 50.0,
                        height: float = 0.0, material: Optional[Usd.Prim] = None) -> Usd.Prim:
    """Static box collider acting as a ground plane (thin cube, top face at ``height``)."""
    path = path or str(unique_child_path(stage, default_root(stage), "GroundPlane"))
    root = UsdGeom.Xform.Define(stage, path)
    cube = UsdGeom.Cube.Define(stage, Sdf.Path(path).AppendChild("Collider"))
    cube.CreateSizeAttr(1.0)
    up = stage_up_axis(stage)
    thickness = size * 0.01
    if up == "Z":
        scale, pos = Gf.Vec3d(size, size, thickness), Gf.Vec3d(0, 0, height - thickness / 2)
    else:
        scale, pos = Gf.Vec3d(size, thickness, size), Gf.Vec3d(0, height - thickness / 2, 0)
    cube_xf = UsdGeom.Xformable(cube)
    cube_xf.AddTranslateOp().Set(pos)
    cube_xf.AddScaleOp().Set(scale)
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.37, 0.4)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if material is not None:
        bind_physics_material([cube.GetPrim()], material, recursive=False)
    return root.GetPrim()


_PHYSICS_APIS = (
    "PhysicsRigidBodyAPI",
    "PhysicsCollisionAPI",
    "PhysicsMeshCollisionAPI",
    "PhysicsMassAPI",
    "PhysicsArticulationRootAPI",
    "PhysicsFilteredPairsAPI",
    "PhysxRigidBodyAPI",
    "PhysxCollisionAPI",
    "PhysxConvexHullCollisionAPI",
    "PhysxConvexDecompositionCollisionAPI",
    "PhysxArticulationAPI",
)


def remove_physics(prims: Iterable[Usd.Prim], recursive: bool = True) -> int:
    """Remove physics API schemas (and their ``physics:`` / ``physx`` attributes)."""
    count = 0
    for root in prims:
        targets = list(Usd.PrimRange(root)) if recursive else [root]
        for prim in targets:
            schemas = [s for s in prim.GetAppliedSchemas() if s.split(":")[0] in _PHYSICS_APIS]
            for s in schemas:
                prim.RemoveAppliedSchema(s)
                count += 1
            for attr in prim.GetAuthoredAttributes():
                n = attr.GetName()
                if n.startswith("physics:") or n.startswith("physx"):
                    prim.RemoveProperty(n)
            for rel in prim.GetAuthoredRelationships():
                n = rel.GetName()
                if n.startswith("physics:") or (n == "material:binding:physics"):
                    prim.RemoveProperty(n)
    return count
