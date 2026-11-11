"""Unit systems for input and display, as in RecurDyn / Adams / Ansys Motion (MKS, MMKS, CGS, IPS).

A unit system names the length, mass and force units the designer shows and reads; time is
always seconds and angles degrees. It does not touch the stage: USD keeps its own scale
(``metersPerUnit`` / ``kilogramsPerUnit``) and every value is converted on the way in and out.
Like those tools the systems need not be coherent (MMKS = mm, kg, N, s).

Quantities are products of length, mass and force powers, so one table gives each unit
label and its factors. The core API takes masses and densities in SI (kg, kg/m^3) and
everything else in stage units; :func:`to_core` / :func:`from_core` follow that convention.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Dict, Optional, Tuple

from pxr import Usd

from .results import Recording
from .stage_utils import si_factors


@dataclass(frozen=True)
class UnitSystem:
    name: str
    length: str
    length_si: float  # metres per unit
    mass: str
    mass_si: float  # kilograms per unit
    force: str
    force_si: float  # newtons per unit


SYSTEMS: Dict[str, UnitSystem] = {s.name: s for s in (
    UnitSystem("MKS", "m", 1.0, "kg", 1.0, "N", 1.0),
    UnitSystem("MMKS", "mm", 1e-3, "kg", 1.0, "N", 1.0),
    UnitSystem("CGS", "cm", 1e-2, "g", 1e-3, "dyn", 1e-5),
    UnitSystem("MTS", "m", 1.0, "t", 1e3, "kN", 1e3),
    UnitSystem("IPS", "in", 0.0254, "lbm", 0.45359237, "lbf", 4.4482216152605),
)}

# quantity -> (length power, mass power, force power, label); time stays in s, angles in deg
QUANTITIES: Dict[str, Tuple[int, int, int, str]] = {
    "length": (1, 0, 0, "{L}"),
    "mass": (0, 1, 0, "{M}"),
    "force": (0, 0, 1, "{F}"),
    "torque": (1, 0, 1, "{F}*{L}"),
    "density": (-3, 1, 0, "{M}/{L}^3"),
    "inertia": (2, 1, 0, "{M}*{L}^2"),
    "velocity": (1, 0, 0, "{L}/s"),
    "linear_stiffness": (-1, 0, 1, "{F}/{L}"),
    "linear_damping": (-1, 0, 1, "{F}*s/{L}"),
    "angle": (0, 0, 0, "deg"),
    "angular_velocity": (0, 0, 0, "deg/s"),
    "angular_stiffness": (1, 0, 1, "{F}*{L}/deg"),  # PhysX angular drives are per degree
    "angular_damping": (1, 0, 1, "{F}*{L}*s/deg"),
}
CORE_SI = {"mass", "density"}  # quantities the core API takes in SI; the rest in stage units

# Parameter annotations of the template builders (mechanisms.py): the window shows and converts them
Length = float
Lengths = tuple  # a point or a size, every component a length
Mass = float
Force = float
Angle = float
LinearStiffness = float
LinearDamping = float
AngularDamping = float
ANNOTATIONS = {"Length": "length", "Lengths": "length", "Mass": "mass", "Force": "force", "Angle": "angle",
               "LinearStiffness": "linear_stiffness", "LinearDamping": "linear_damping",
               "AngularDamping": "angular_damping"}


def annotation_quantity(annotation) -> Optional[str]:
    """Quantity named by a parameter annotation (a string under postponed evaluation), e.g.
    ``"Optional[Force]"`` -> ``"force"``."""
    text = annotation if isinstance(annotation, str) else getattr(annotation, "__name__", "")
    for alias, quantity in ANNOTATIONS.items():
        if text == alias or text == f"Optional[{alias}]":
            return quantity
    return None

# unit labels of SI result channels (recording / exact solver) -> quantity
SI_LABELS = {"m": "length", "N": "force", "N*m": "torque", "kg": "mass"}


def system(name: str) -> UnitSystem:
    return SYSTEMS[name]


def label(us: UnitSystem, quantity: str) -> str:
    return QUANTITIES[quantity][3].format(L=us.length, M=us.mass, F=us.force)


def _factor(length: float, mass: float, force: float, quantity: str) -> float:
    a, b, c, _ = QUANTITIES[quantity]
    return length ** a * mass ** b * force ** c


def si_per_unit(us: UnitSystem, quantity: str) -> float:
    """SI value of one display unit of ``quantity``."""
    return _factor(us.length_si, us.mass_si, us.force_si, quantity)


def si_per_stage_unit(stage: Usd.Stage, quantity: str) -> float:
    f = si_factors(stage)
    return _factor(f["length"], f["mass"], f["force"], quantity)


def to_si(value: float, us: UnitSystem, quantity: str) -> float:
    return value * si_per_unit(us, quantity)


def from_si(value: float, us: UnitSystem, quantity: str) -> float:
    return value / si_per_unit(us, quantity)


def to_core(value: float, us: UnitSystem, quantity: str, stage: Usd.Stage) -> float:
    """Display value -> what the core API takes (SI for mass / density, else stage units)."""
    si = to_si(value, us, quantity)
    return si if quantity in CORE_SI else si / si_per_stage_unit(stage, quantity)


def from_core(value: float, us: UnitSystem, quantity: str, stage: Usd.Stage) -> float:
    si = value if quantity in CORE_SI else value * si_per_stage_unit(stage, quantity)
    return from_si(si, us, quantity)


def convert_recording(rec: Recording, us: UnitSystem) -> Recording:
    """Copy of an SI recording in ``us`` (channels whose unit is m, N, N*m or kg; deg and s stay)."""
    out = replace(rec, time=list(rec.time), channels={}, units={})
    if rec.x_unit in SI_LABELS:
        q = SI_LABELS[rec.x_unit]
        out.time = [from_si(t, us, q) for t in rec.time]
        out.x_unit = label(us, q)
    for key, series in rec.channels.items():
        unit = rec.units.get(key, "")
        q = SI_LABELS.get(unit)
        out.channels[key] = [from_si(v, us, q) for v in series] if q else list(series)
        out.units[key] = label(us, q) if q else unit
    return out
