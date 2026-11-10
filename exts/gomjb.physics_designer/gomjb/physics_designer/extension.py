"""Kit extension entry point: registers commands, the Tools menu entry and the window."""
from __future__ import annotations

import omni.ext
import omni.ui as ui

from . import commands

MENU_PATH = "Tools"


class PhysicsDesignerExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._window = None
        commands.register()
        self._menu_items = []
        try:
            from omni.kit.menu.utils import MenuItemDescription, add_menu_items

            self._menu_items = [MenuItemDescription(name="Physics Designer", onclick_fn=self.toggle_window)]
            add_menu_items(self._menu_items, MENU_PATH)
        except ImportError:  # headless / minimal apps
            pass
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
        if self._menu_items:
            from omni.kit.menu.utils import remove_menu_items

            remove_menu_items(self._menu_items, MENU_PATH)
            self._menu_items = []
        ui.Workspace.set_show_window_fn("Physics Designer", None)
        if self._window is not None:
            self._window.destroy()
            self._window = None
        commands.unregister()
