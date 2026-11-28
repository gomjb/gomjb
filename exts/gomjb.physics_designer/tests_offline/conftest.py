import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pxr import Usd, UsdGeom  # noqa: E402


@pytest.fixture
def stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    world = UsdGeom.Xform.Define(s, "/World").GetPrim()
    s.SetDefaultPrim(world)
    return s


def make_cube(stage, path, pos=(0, 0, 0), size=1.0, scale=None, rot_z=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    cube.CreateExtentAttr([(-size / 2,) * 3, (size / 2,) * 3])
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(pos)
    if rot_z is not None:
        xf.AddRotateZOp().Set(rot_z)
    if scale is not None:
        xf.AddScaleOp().Set(scale)
    return cube.GetPrim()
