"""Tiny omni.ui form helpers so the window code reads like a form description."""
from __future__ import annotations

import weakref
from typing import Callable, List, Optional, Sequence, Union

import omni.ui as ui
from pxr import Usd

from ..core import units as un

LABEL_W = 118
UNIT_W = 78  # unit column right of a value ("N*mm*s/deg")
ROW_H = 22

Quantity = Union[str, Callable[[], str]]  # a quantity name, or a function of the form state (joint type)


def _label(text: str, tooltip: str = ""):
    return ui.Label(text, width=LABEL_W, tooltip=tooltip or text)


def _unit_label():
    return ui.Label("", width=UNIT_W, style={"color": 0xFF9A9A9A, "margin_width": 4})


class FloatInput:
    """``unit``: text of a unit column right of the value (None = no column)."""

    def __init__(self, label: str, value: float = 0.0, tooltip: str = "", step: float = 0.01,
                 unit: Optional[str] = None):
        self.unit_label = None
        with ui.HStack(height=ROW_H):
            self.label = _label(label, tooltip)
            self.model = ui.SimpleFloatModel(float(value))
            self.drag = ui.FloatDrag(model=self.model, step=step)
            if unit is not None:
                self.unit_label = _unit_label()
                self.unit_label.text = unit

    @property
    def value(self) -> float:
        return self.model.as_float

    @value.setter
    def value(self, v: float) -> None:
        self.model.set_value(float(v))


class IntInput:
    def __init__(self, label: str, value: int = 0, tooltip: str = "", min_value: int = 0):
        with ui.HStack(height=ROW_H):
            _label(label, tooltip)
            self.model = ui.SimpleIntModel(int(value))
            ui.IntDrag(model=self.model, min=min_value, max=10000)

    @property
    def value(self) -> int:
        return self.model.as_int


class StringInput:
    def __init__(self, label: str, value: str = "", tooltip: str = ""):
        with ui.HStack(height=ROW_H):
            _label(label, tooltip)
            self.model = ui.SimpleStringModel(value)
            ui.StringField(model=self.model)

    @property
    def value(self) -> str:
        return self.model.as_string


class BoolInput:
    def __init__(self, label: str, value: bool = False, tooltip: str = "",
                 on_changed: Optional[Callable[[bool], None]] = None):
        with ui.HStack(height=ROW_H):
            _label(label, tooltip)
            self.model = ui.SimpleBoolModel(bool(value))
            ui.CheckBox(model=self.model, width=20)
            ui.Spacer()
        if on_changed is not None:
            self.model.add_value_changed_fn(lambda m: on_changed(m.as_bool))

    @property
    def value(self) -> bool:
        return self.model.as_bool


class Vec3Input:
    def __init__(self, label: str, value=(0.0, 0.0, 0.0), tooltip: str = "", step: float = 0.01,
                 unit: Optional[str] = None):
        self.models = [ui.SimpleFloatModel(float(v)) for v in value]
        self.drags = []
        self.unit_label = None
        with ui.HStack(height=ROW_H, spacing=2):
            self.label = _label(label, tooltip)
            for axis, m in zip("XYZ", self.models):
                ui.Label(axis, width=10, alignment=ui.Alignment.CENTER)
                self.drags.append(ui.FloatDrag(model=m, step=step))
            if unit is not None:
                self.unit_label = _unit_label()
                self.unit_label.text = unit

    @property
    def value(self):
        return tuple(m.as_float for m in self.models)

    @value.setter
    def value(self, v) -> None:
        for m, x in zip(self.models, v):
            m.set_value(float(x))


# ---------------------------------------------------------------------------------------
# Unit-aware inputs
# ---------------------------------------------------------------------------------------
class Units:
    """The window's unit system. Inputs made here show their value in it, label the unit and keep
    the physical value when the system changes; :meth:`core` hands values to the core API."""

    def __init__(self, name: str):
        self.system = un.system(name)
        self._fields = weakref.WeakSet()  # rebuilt frames drop their old fields
        self._listeners: List[Callable[[], None]] = []

    def float(self, label: str, value_si: float, quantity: Quantity, tooltip: str = "",
              step: float = 0.01) -> "UnitFloat":
        return UnitFloat(self, label, value_si, quantity, tooltip, step)

    def vec3(self, label: str, value_si, quantity: Quantity, tooltip: str = "", step: float = 0.01) -> "UnitVec3":
        return UnitVec3(self, label, value_si, quantity, tooltip, step)

    def label(self, quantity: str) -> str:
        return un.label(self.system, quantity)

    def add_listener(self, fn: Callable[[], None]) -> None:
        """``fn()`` after every change of unit system (labels or displays outside the fields)."""
        self._listeners.append(fn)

    def set_system(self, name: str) -> None:
        old, self.system = self.system, un.system(name)
        for f in list(self._fields):
            f._convert(old)
        self.refresh_labels()

    def refresh_labels(self) -> None:
        """Relabel the inputs, e.g. after a joint type change turned degrees into lengths."""
        for f in list(self._fields):
            f._relabel()
        for fn in self._listeners:
            fn()


class _UnitField:
    def _setup(self, units: Units, label: str, quantity: Quantity, step_si: float) -> None:
        self._units, self._base, self._quantity, self._step_si = units, label, quantity, step_si
        self._shown = self.quantity  # the quantity the displayed number is in
        units._fields.add(self)
        self._relabel()

    def _requantify(self, values):
        """``values`` after a change of quantity (a joint type change turned N*m/deg into N/m): the
        SI number is kept, as the inputs behaved before unit systems (1000 N*m/deg -> 1000 N/m)."""
        old, self._shown = self._shown, self.quantity
        if old == self._shown:
            return values
        us = self._units.system
        return [un.from_si(un.to_si(v, us, old), us, self._shown) for v in values]

    @property
    def quantity(self) -> str:
        return self._quantity() if callable(self._quantity) else self._quantity

    @property
    def unit(self) -> str:
        return self._units.label(self.quantity)

    def _step(self) -> float:
        return self._step_si / un.si_per_unit(self._units.system, self.quantity)

    def _from_si(self, v: float) -> float:
        return un.from_si(v, self._units.system, self.quantity)

    def _to_si(self, v: float) -> float:
        return un.to_si(v, self._units.system, self.quantity)


class UnitFloat(FloatInput, _UnitField):
    def __init__(self, units: Units, label: str, value_si: float, quantity: Quantity, tooltip: str = "",
                 step: float = 0.01):
        FloatInput.__init__(self, label, 0.0, tooltip, step, unit="")
        self._setup(units, label, quantity, step)
        self.si = value_si

    @property
    def si(self) -> float:
        return self._to_si(self.value)

    @si.setter
    def si(self, v: float) -> None:
        self.value = self._from_si(v)

    def core(self, stage: Usd.Stage) -> float:
        """The value as the core API takes it (SI for mass / density, else stage units)."""
        return un.to_core(self.value, self._units.system, self.quantity, stage)

    def set_core(self, stage: Usd.Stage, v: float) -> None:
        self.value = un.from_core(v, self._units.system, self.quantity, stage)

    def _relabel(self) -> None:
        self.value = self._requantify([self.value])[0]
        self.unit_label.text = self.unit
        self.drag.step = self._step()

    def _convert(self, old: un.UnitSystem) -> None:
        self.value = self._from_si(un.to_si(self.value, old, self.quantity))


class UnitVec3(Vec3Input, _UnitField):
    def __init__(self, units: Units, label: str, value_si, quantity: Quantity, tooltip: str = "",
                 step: float = 0.01):
        Vec3Input.__init__(self, label, (0.0, 0.0, 0.0), tooltip, step, unit="")
        self._setup(units, label, quantity, step)
        self.value = tuple(self._from_si(v) for v in value_si)

    def core(self, stage: Usd.Stage):
        return tuple(un.to_core(v, self._units.system, self.quantity, stage) for v in self.value)

    def set_core(self, stage: Usd.Stage, v) -> None:
        self.value = tuple(un.from_core(x, self._units.system, self.quantity, stage) for x in v)

    def _relabel(self) -> None:
        self.value = tuple(self._requantify(list(self.value)))
        self.unit_label.text = self.unit
        for d in self.drags:
            d.step = self._step()

    def _convert(self, old: un.UnitSystem) -> None:
        self.value = tuple(self._from_si(un.to_si(v, old, self.quantity)) for v in self.value)


class ComboInput:
    def __init__(self, label: str, items: Sequence[str], default: int = 0, tooltip: str = "",
                 on_changed: Optional[Callable[[str], None]] = None):
        self.items = list(items)
        with ui.HStack(height=ROW_H):
            _label(label, tooltip)
            self.widget = ui.ComboBox(default, *self.items)
        if on_changed is not None:
            self.widget.model.add_item_changed_fn(lambda m, i: on_changed(self.value))

    @property
    def index(self) -> int:
        return self.widget.model.get_item_value_model().as_int

    @property
    def value(self) -> str:
        return self.items[self.index]


def button_row(buttons: Sequence[tuple], height: int = 24, spacing: int = 3) -> None:
    """``buttons`` = [(text, callback, tooltip?), ...] laid out in one row."""
    with ui.HStack(height=height, spacing=spacing):
        for entry in buttons:
            text, fn = entry[0], entry[1]
            tip = entry[2] if len(entry) > 2 else ""
            ui.Button(text, clicked_fn=fn, tooltip=tip)


def section(title: str, collapsed: bool = True):
    return ui.CollapsableFrame(title, collapsed=collapsed, height=0)


def hint(text: str) -> None:
    ui.Label(text, word_wrap=True, height=0, style={"color": 0xFF9A9A9A, "font_size": 13})
