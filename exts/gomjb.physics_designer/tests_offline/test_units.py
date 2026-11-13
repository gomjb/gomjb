"""Unit systems: labels, SI factors, stage conversion, recordings and template annotations."""
import inspect

import pytest
from pxr import UsdGeom, UsdPhysics

from gomjb.physics_designer.core import mechanisms as me
from gomjb.physics_designer.core import physics as ph
from gomjb.physics_designer.core import units as un
from gomjb.physics_designer.core.results import Recording

MMKS, CGS, MTS, IPS = (un.system(n) for n in ("MMKS", "CGS", "MTS", "IPS"))


def test_labels_follow_the_system():
    assert [un.label(MMKS, q) for q in ("length", "torque", "density", "angular_stiffness", "velocity")] == \
        ["mm", "N*mm", "kg/mm^3", "N*mm/deg", "mm/s"]
    assert un.label(MTS, "force") == "kN" and un.label(CGS, "mass") == "g" and un.label(IPS, "force") == "lbf"
    assert un.label(MMKS, "angle") == "deg"


def test_si_factors():
    assert un.to_si(7.85e-6, MMKS, "density") == pytest.approx(7850.0)  # steel
    assert un.to_si(1.0, MMKS, "torque") == pytest.approx(1e-3)
    assert un.to_si(1.0, MMKS, "linear_stiffness") == pytest.approx(1e3)  # N/mm -> N/m
    assert un.to_si(1.0, MTS, "force") == 1e3 and un.to_si(1.0, MTS, "mass") == 1e3
    assert un.to_si(1.0, CGS, "force") == pytest.approx(1e-5)
    assert un.to_si(1.0, IPS, "force") == pytest.approx(4.4482216)
    assert un.to_si(30.0, MMKS, "angle") == 30.0
    for us in un.SYSTEMS.values():
        for q in un.QUANTITIES:
            assert un.from_si(un.to_si(1.234, us, q), us, q) == pytest.approx(1.234)


def test_core_values_use_stage_units_except_mass_and_density(stage):
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)  # a centimetre stage
    UsdPhysics.SetStageKilogramsPerUnit(stage, 0.001)  # in grams
    assert un.to_core(100.0, MMKS, "length", stage) == pytest.approx(10.0)  # 100 mm = 10 cm
    assert un.to_core(2.0, MMKS, "mass", stage) == pytest.approx(2.0)  # the core takes kg
    assert un.to_core(1.0, MMKS, "force", stage) == pytest.approx(1.0 / (0.001 * 0.01))  # g*cm/s^2
    assert un.from_core(10.0, MMKS, "length", stage) == pytest.approx(100.0)


def test_set_mass_takes_si(stage):
    UsdPhysics.SetStageKilogramsPerUnit(stage, 0.001)
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)
    prim = UsdGeom.Xform.Define(stage, "/World/Body").GetPrim()
    ph.set_mass([prim], mass=2.0, density=1000.0)
    api = UsdPhysics.MassAPI(prim)
    assert api.GetMassAttr().Get() == pytest.approx(2000.0)  # grams
    assert api.GetDensityAttr().Get() == pytest.approx(1.0)  # g/cm^3


def test_convert_recording():
    rec = Recording("sweep", x_name="Actuator", x_unit="m")
    rec.units.update({"force:Actuator": "N", "pos:Hinge": "deg", "joint_torque:Pin:x": "N*m"})
    rec.add_sample(-0.1, {"force:Actuator": -2000.0, "pos:Hinge": 30.0, "joint_torque:Pin:x": 5.0})
    out = un.convert_recording(rec, MTS)
    assert out.channels["force:Actuator"] == [-2.0] and out.units["force:Actuator"] == "kN"
    assert out.channels["pos:Hinge"] == [30.0] and out.units["pos:Hinge"] == "deg"
    assert out.channels["joint_torque:Pin:x"] == [pytest.approx(0.005)] and out.units["joint_torque:Pin:x"] == "kN*m"
    mm = un.convert_recording(rec, MMKS)
    assert mm.time == [pytest.approx(-100.0)] and mm.x_label == "Actuator [mm]"
    assert rec.channels["force:Actuator"] == [-2000.0] and rec.x_unit == "m"  # the original stays SI


def test_every_template_parameter_has_a_quantity():
    """The window converts template parameters by their annotation: none may be left unitless by mistake."""
    unitless = {"name", "parent", "stage", "direction"}  # direction is a unit vector
    for name, fn in me.TEMPLATES.items():
        for pname, param in inspect.signature(fn).parameters.items():
            if pname in unitless or isinstance(param.default, (bool, int, str)) and not isinstance(param.default, float):
                continue
            assert un.annotation_quantity(param.annotation), f"{name}.{pname} has no unit annotation"
