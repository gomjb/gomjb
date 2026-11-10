"""Tiny omni.ui form helpers so the window code reads like a form description."""
from __future__ import annotations

from typing import Callable, Optional, Sequence

import omni.ui as ui

LABEL_W = 118
ROW_H = 22


def _label(text: str, tooltip: str = "") -> None:
    ui.Label(text, width=LABEL_W, tooltip=tooltip or text)


class FloatInput:
    def __init__(self, label: str, value: float = 0.0, tooltip: str = "", step: float = 0.01):
        with ui.HStack(height=ROW_H):
            _label(label, tooltip)
            self.model = ui.SimpleFloatModel(float(value))
            ui.FloatDrag(model=self.model, step=step)

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
    def __init__(self, label: str, value=(0.0, 0.0, 0.0), tooltip: str = ""):
        self.models = [ui.SimpleFloatModel(float(v)) for v in value]
        with ui.HStack(height=ROW_H, spacing=2):
            _label(label, tooltip)
            for axis, m in zip("XYZ", self.models):
                ui.Label(axis, width=10, alignment=ui.Alignment.CENTER)
                ui.FloatDrag(model=m, step=0.01)

    @property
    def value(self):
        return tuple(m.as_float for m in self.models)

    @value.setter
    def value(self, v) -> None:
        for m, x in zip(self.models, v):
            m.set_value(float(x))


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
