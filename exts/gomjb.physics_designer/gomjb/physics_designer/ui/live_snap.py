"""Live grid / angle snapping while objects are dragged with the viewport gizmo.

Listens to ``Usd.Notice.ObjectsChanged`` for transform changes on the selected prims and,
on the next app update, quantizes their world position (and optionally orientation).  The
gizmo keeps writing its own unsnapped value each frame, so the effect is a stepped drag.
"""
from __future__ import annotations

from typing import Callable, Optional, Set

import carb
import omni.kit.app
import omni.usd
from pxr import Sdf, Tf, Usd

from ..core import math_utils as mu
from ..core import transform as tf


class LiveSnap:
    def __init__(self, get_params: Callable[[], dict]):
        """``get_params`` returns {"step": float, "angle": float, "position": bool, "rotation": bool}."""
        self._get_params = get_params
        self._listener: Optional[Tf.Listener] = None
        self._update_sub = None
        self._pending: Set[Sdf.Path] = set()
        self._applying = False
        self._stage: Optional[Usd.Stage] = None

    @property
    def active(self) -> bool:
        return self._listener is not None

    def start(self) -> None:
        self.stop()
        self._stage = omni.usd.get_context().get_stage()
        if self._stage is None:
            return
        self._listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, self._stage)
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="gomjb.physics_designer.live_snap")
        )

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.Revoke()
        self._listener = None
        self._update_sub = None
        self._pending.clear()
        self._stage = None

    # ---------------------------------------------------------------------------------
    def _on_objects_changed(self, notice: Usd.Notice.ObjectsChanged, sender: Usd.Stage) -> None:
        if self._applying:
            return
        selected = set(omni.usd.get_context().get_selection().get_selected_prim_paths())
        if not selected:
            return
        for path in notice.GetChangedInfoOnlyPaths():
            if path.IsPropertyPath() and path.name.startswith("xformOp"):
                prim_path = path.GetPrimPath()
                if str(prim_path) in selected:
                    self._pending.add(prim_path)

    def _on_update(self, _event) -> None:
        if not self._pending or self._stage is None:
            return
        params = self._get_params()
        pending, self._pending = self._pending, set()
        self._applying = True
        try:
            # No Sdf.ChangeBlock here: the snap reads composed transforms between writes.
            for path in sorted(pending, key=lambda p: p.pathElementCount):
                prim = self._stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    self._snap(prim, params)
        except Exception as exc:  # never break the update loop
            carb.log_warn(f"[Physics Designer] live snap failed: {exc}")
        finally:
            self._applying = False

    @staticmethod
    def _snap(prim: Usd.Prim, params: dict) -> None:
        pos = tf.world_position(prim)
        rot = tf.world_rotation(prim)
        new_pos = mu.quantize_vec(pos, params["step"]) if params.get("position", True) else pos
        new_rot = mu.snap_rotation(rot, params["angle"]) if params.get("rotation", False) else rot
        if mu.is_close(pos, new_pos, 1e-9) and mu.quat_angle_between(rot, new_rot) < 1e-6:
            return
        if params.get("rotation", False):
            tf.set_world_pose(prim, new_pos, new_rot)
        else:
            tf.set_world_position(prim, new_pos)
