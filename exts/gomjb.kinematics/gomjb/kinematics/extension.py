"""Kit extension entry point: joins the Physics Designer's motion runtime, Tools menu entry, window."""
from __future__ import annotations

from typing import Optional

import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

MENU_PATH = "Tools"
WINDOW_TITLE = "Kinematics"
_instance: Optional["KinematicsExtension"] = None


def get_window():
    """The open Kinematics window (for scripting), or None."""
    return _instance._window if _instance is not None else None


def get_kinematic_runtime():
    """The runtime that moves kinematic mechanisms while simulating, or None."""
    return _instance.runtime if _instance is not None else None


class KinematicsExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        global _instance
        _instance = self
        self._window = None
        from gomjb.physics_designer.extension import get_motion_runtime

        from .runtime import KinematicRuntime

        self.runtime = KinematicRuntime()
        get_motion_runtime().add_participant(self.runtime)
        self._menu_items = [MenuItemDescription(name=WINDOW_TITLE, onclick_fn=self.toggle_window)]
        add_menu_items(self._menu_items, MENU_PATH)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, lambda visible: self._show(visible))

    def toggle_window(self, *_):
        self._show(not (self._window and self._window.visible))

    def _show(self, visible: bool):
        if visible and self._window is None:
            from .ui import KinematicsWindow

            self._window = KinematicsWindow()
        if self._window is not None:
            self._window.visible = visible

    def on_shutdown(self):
        global _instance
        from gomjb.physics_designer.extension import get_motion_runtime

        runtime = get_motion_runtime()
        if runtime is not None:
            runtime.remove_participant(self.runtime)
        remove_menu_items(self._menu_items, MENU_PATH)
        ui.Workspace.set_show_window_fn(WINDOW_TITLE, None)
        if self._window is not None:
            self._window.destroy()
            self._window = None
        _instance = None
