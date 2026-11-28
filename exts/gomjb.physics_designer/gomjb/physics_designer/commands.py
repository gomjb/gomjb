"""Undoable Kit commands wrapping the core operations.

Any core operation can be made undoable::

    from gomjb.physics_designer.commands import run_edit
    run_edit(lambda stage: transform.translate(prims, (0, 0, 1)), modify_paths=paths, label="Lift")

or, from the Script Editor::

    with designer_edit(["/World/Cube"], "My edit") as stage:
        ...  # any USD authoring on ``stage`` becomes one Ctrl+Z step
"""
from __future__ import annotations

import contextlib
from typing import Callable, Iterable, List, Optional

import omni.kit.commands
import omni.usd
from pxr import Usd

from .core.snapshot import EditSnapshot


class PhysicsDesignerEditCommand(omni.kit.commands.Command):
    """Runs ``fn(stage)`` once; undo/redo replay a path-scoped USD snapshot."""

    def __init__(self, fn: Callable[[Usd.Stage], object], modify_paths: Iterable[str] = (),
                 label: str = "Physics Designer edit", usd_context_name: str = ""):
        self._fn = fn
        self._paths = [str(p) for p in modify_paths]
        self._label = label
        self._ctx = usd_context_name
        self._snapshot: Optional[EditSnapshot] = None
        self._result = None
        self._error: Optional[BaseException] = None

    def do(self):
        stage = omni.usd.get_context(self._ctx).get_stage()
        if self._snapshot is not None:  # redo
            self._snapshot.redo()
            return self._result
        layer = stage.GetEditTarget().GetLayer()
        self._snapshot = EditSnapshot(layer, self._paths)
        try:
            self._result = self._fn(stage)
        except BaseException as exc:  # roll back partial edits, surface the error to the caller
            self._snapshot.finish()
            self._snapshot.undo()
            self._error = exc
            raise
        self._snapshot.finish()
        return self._result

    def undo(self):
        if self._snapshot is not None:
            self._snapshot.undo()


class PhysicsDesignerRecordedEditCommand(omni.kit.commands.Command):
    """Pushes an already-applied :class:`EditSnapshot` onto the undo stack."""

    def __init__(self, snapshot: EditSnapshot, label: str = "Scripted edit"):
        self._snapshot = snapshot
        self._label = label
        self._applied = True  # the edit already happened when the command is executed

    def do(self):
        if not self._applied:
            self._snapshot.redo()
        self._applied = True

    def undo(self):
        self._snapshot.undo()
        self._applied = False


COMMANDS = [PhysicsDesignerEditCommand, PhysicsDesignerRecordedEditCommand]


def register() -> None:
    for cmd in COMMANDS:
        omni.kit.commands.register(cmd)


def unregister() -> None:
    for cmd in COMMANDS:
        omni.kit.commands.unregister(cmd)


def run_edit(fn: Callable[[Usd.Stage], object], modify_paths: Iterable[str] = (),
             label: str = "Physics Designer edit"):
    """Execute ``fn`` as one undoable step and return its result (re-raises errors)."""
    errors: List[BaseException] = []

    def guarded(stage):
        try:
            return fn(stage)
        except BaseException as exc:
            errors.append(exc)
            raise

    result = omni.kit.commands.execute(
        "PhysicsDesignerEditCommand", fn=guarded, modify_paths=list(modify_paths), label=label
    )
    if errors:
        raise errors[0]
    # Kit returns (success, result) in recent versions, plain result in older ones.
    if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], bool):
        result = result[1]
    return result


@contextlib.contextmanager
def designer_edit(modify_paths: Iterable[str] = (), label: str = "Scripted edit"):
    """Context manager form of :func:`run_edit` for interactive scripting.

    The body's USD edits are captured and pushed onto Kit's undo stack as one step.
    """
    stage = omni.usd.get_context().get_stage()
    snap = EditSnapshot(stage.GetEditTarget().GetLayer(), modify_paths)
    try:
        yield stage
    except BaseException:
        snap.finish()
        snap.undo()
        raise
    snap.finish()
    omni.kit.commands.execute("PhysicsDesignerRecordedEditCommand", snapshot=snap, label=label)
