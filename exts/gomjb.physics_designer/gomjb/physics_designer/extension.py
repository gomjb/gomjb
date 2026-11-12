"""Kit extension entry point: registers commands, the Tools menu entry and the window."""
from __future__ import annotations

from typing import Optional

import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from . import commands

MENU_PATH = "Tools"
_instance: Optional["PhysicsDesignerExtension"] = None


def get_window():
    """The open Physics Designer window (for scripting), or None."""
    return _instance._window if _instance is not None else None


def get_motion_runtime():
    """The runtime that plays joint motions and kinematic mechanisms, or None."""
    return _instance._motions if _instance is not None else None


class PhysicsDesignerExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        global _instance
        _instance = self
        self._window = None
        commands.register()
        from .motion_runtime import MotionRuntime  # Isaac Sim runtime

        self._motions = MotionRuntime()
        self._menu_items = [MenuItemDescription(name="Physics Designer", onclick_fn=self.toggle_window)]
        add_menu_items(self._menu_items, MENU_PATH)
        ui.Workspace.set_show_window_fn("Physics Designer", lambda visible: self._show(visible))
        self._show(True)

    def toggle_window(self, *_):
        self._show(not (self._window and self._window.visible))

    def _show(self, visible: bool):
        if visible and self._window is None:
            from .ui import PhysicsDesignerWindow

            self._window = PhysicsDesignerWindow()
        if self._window is not None:
            self._window.visible = visible

    def on_shutdown(self):
        global _instance
        remove_menu_items(self._menu_items, MENU_PATH)
        ui.Workspace.set_show_window_fn("Physics Designer", None)
        if self._window is not None:
            self._window.destroy()
            self._window = None
        self._motions.destroy()
        commands.unregister()
        _instance = None
