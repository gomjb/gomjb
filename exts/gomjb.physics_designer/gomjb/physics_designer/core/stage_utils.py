"""Small stage helpers shared by the core modules."""
from __future__ import annotations

from typing import Iterable, List, Optional

from pxr import Sdf, Tf, Usd, UsdGeom


def unique_child_path(stage: Usd.Stage, parent: Sdf.Path, name: str) -> Sdf.Path:
    """``parent/name`` or ``parent/name_01``... whichever is still free."""
    name = Tf.MakeValidIdentifier(name or "Prim")
    parent = Sdf.Path(str(parent))
    candidate = parent.AppendChild(name)
    n = 1
    while stage.GetPrimAtPath(candidate):
        candidate = parent.AppendChild(f"{name}_{n:02d}")
        n += 1
    return candidate


def default_root(stage: Usd.Stage) -> Sdf.Path:
    """The default prim path, or ``/World`` (created as Xform if missing)."""
    dp = stage.GetDefaultPrim()
    if dp and dp.IsValid():
        return dp.GetPath()
    world = stage.GetPrimAtPath("/World")
    if not world:
        world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(world)
    return world.GetPath()


def ensure_scope(stage: Usd.Stage, path: str) -> Usd.Prim:
    prim = stage.GetPrimAtPath(path)
    if not prim:
        prim = UsdGeom.Scope.Define(stage, path).GetPrim()
    return prim


def prims_from_paths(stage: Usd.Stage, paths: Iterable[str]) -> List[Usd.Prim]:
    out = []
    for p in paths:
        prim = stage.GetPrimAtPath(Sdf.Path(str(p)))
        if prim and prim.IsValid():
            out.append(prim)
    return out


def is_guide(prim: Usd.Prim) -> bool:
    img = UsdGeom.Imageable(prim)
    if not img:
        return False
    attr = img.GetPurposeAttr()
    return bool(attr and attr.Get() == UsdGeom.Tokens.guide)


def stage_up_axis(stage: Usd.Stage) -> str:
    return str(UsdGeom.GetStageUpAxis(stage) or "Z")


def meters_per_unit(stage: Usd.Stage) -> float:
    return float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)


def kilograms_per_unit(stage: Usd.Stage) -> float:
    from pxr import UsdPhysics

    return float(UsdPhysics.GetStageKilogramsPerUnit(stage) or 1.0)


def si_factors(stage: Usd.Stage) -> dict:
    """Multipliers that turn stage-unit physics quantities into SI (m, kg, N, N*m, kg/m^3)."""
    m, kg = meters_per_unit(stage), kilograms_per_unit(stage)
    return {"length": m, "mass": kg, "force": kg * m, "torque": kg * m * m, "density": kg / m ** 3}


def find_first(stage: Usd.Stage, predicate) -> Optional[Usd.Prim]:
    for prim in stage.Traverse():
        if predicate(prim):
            return prim
    return None
