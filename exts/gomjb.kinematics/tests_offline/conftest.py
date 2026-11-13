import os
import sys

import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, ".."))
MODELER = os.path.abspath(os.path.join(ROOT, "..", "gomjb.physics_designer"))
# both extensions share the "gomjb" namespace package; the modeler's Kit stand-ins drive the window
for path in (ROOT, MODELER, os.path.join(MODELER, "tests_offline")):
    sys.path.insert(0, path)

from pxr import Usd, UsdGeom  # noqa: E402


@pytest.fixture
def stage():
    s = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(s, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    world = UsdGeom.Xform.Define(s, "/World").GetPrim()
    s.SetDefaultPrim(world)
    return s
