"""Viewport picking: while active, a left click reports the clicked prim and the 3-D point under the
cursor. It follows Kit's own pattern (omni.graph.ui_nodes PickingManipulator): a gesture on an
``sc.Screen`` in a scene view over the viewport, and ``viewport_api.request_query`` for the hit.
The gesture never blocks the viewport's own ones, so the click still selects and the camera
still navigates (a drag is not a click)."""
from __future__ import annotations

from typing import Callable, Optional, Sequence

from omni.ui import scene as sc

FRAME_NAME = "gomjb.physics_designer.pick"
OnPick = Callable[[str, Optional[Sequence[float]]], None]  # (prim path or "", world point or None)


class _NeverPrevented(sc.GestureManager):
    def can_be_prevented(self, gesture):
        return False


class _PickClick(sc.ClickGesture):
    def __init__(self, viewport_api, on_pick: OnPick):
        super().__init__(mouse_button=0, manager=_NeverPrevented())
        self._api = viewport_api
        self._on_pick = on_pick

    def on_ended(self):
        pos, _ = self._api.map_ndc_to_texture(self.sender.gesture_payload.mouse)
        if pos is None or not all(0.0 <= v <= 1.0 for v in pos):
            return  # not over the rendered image
        width, height = self._api.resolution
        pixel = (int(pos[0] * width), int((1.0 - pos[1]) * height))
        self._api.request_query(pixel, lambda path, world, *args: self._on_pick(path or "", world))


class ViewportPicker:
    def __init__(self, on_pick: OnPick):
        self._on_pick = on_pick
        self._window = None
        self._scene_view = None

    @property
    def active(self) -> bool:
        return self._scene_view is not None

    def start(self) -> None:
        from omni.kit.viewport.utility import get_active_viewport_window

        if self.active:
            return
        window = get_active_viewport_window()
        if window is None:
            raise RuntimeError("no viewport window")
        self._window = window
        with window.get_frame(FRAME_NAME):
            self._scene_view = sc.SceneView(child_windows_input=0)
        window.viewport_api.add_scene_view(self._scene_view)
        with self._scene_view.scene:
            sc.Screen(gesture=[_PickClick(window.viewport_api, self._on_pick)])

    def stop(self) -> None:
        if not self.active:
            return
        self._window.viewport_api.remove_scene_view(self._scene_view)
        self._scene_view.scene.clear()
        self._scene_view = None
        self._window.get_frame(FRAME_NAME).clear()
        self._window = None
