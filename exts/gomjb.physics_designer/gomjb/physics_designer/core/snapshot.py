"""Lightweight, path-scoped undo snapshots for USD layers.

Kit's undo stack only understands ``omni.kit.commands``.  Designer operations are ordinary
Python functions that author USD directly, so each command wraps them in an
:class:`EditSnapshot`:

* the prim specs that will be *modified* (declared paths) are copied into an anonymous
  backup layer before the edit, and copied back on undo (``Sdf.CopySpec`` overwrites the
  spec in place, so sibling order is preserved);
* prim specs *created* by the edit are found by diffing the layer's prim paths and are
  removed on undo.

This keeps undo cheap even for stages with heavy meshes in the root layer, because only the
touched subtrees are copied.
"""
from __future__ import annotations

from typing import Iterable, List, Optional, Set

from pxr import Sdf


def _prim_paths(layer: Sdf.Layer) -> Set[Sdf.Path]:
    paths: Set[Sdf.Path] = set()

    def visit(p: Sdf.Path) -> None:
        if p.IsPrimPath():
            paths.add(p)

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    return paths


def _minimal_roots(paths: Iterable[Sdf.Path]) -> List[Sdf.Path]:
    ordered = sorted(paths, key=lambda p: p.pathElementCount)
    roots: List[Sdf.Path] = []
    for p in ordered:
        if not any(p.HasPrefix(r) for r in roots):
            roots.append(p)
    return roots


def _remove_spec(layer: Sdf.Layer, path: Sdf.Path) -> None:
    spec = layer.GetPrimAtPath(path)
    if not spec:
        return
    parent = spec.nameParent
    if parent:
        del parent.nameChildren[spec.name]
    else:  # pragma: no cover - root level prims always have the pseudo-root as parent
        del layer.pseudoRoot.nameChildren[spec.name]


class EditSnapshot:
    """Capture → edit → (undo) restore → (redo) re-apply."""

    def __init__(self, layer: Sdf.Layer, modify_paths: Iterable[str] = ()):
        self.layer = layer
        self.modify_paths = _minimal_roots({Sdf.Path(str(p)) for p in modify_paths if p})
        self._before = self._capture()
        self._before_prims = _prim_paths(layer)
        self._after: Optional[Sdf.Layer] = None
        self._after_roots: List[Sdf.Path] = []
        self._created: List[Sdf.Path] = []

    # -- capture helpers ------------------------------------------------------------
    def _capture(self, extra: Iterable[Sdf.Path] = ()) -> Sdf.Layer:
        backup = Sdf.Layer.CreateAnonymous(".designer_undo.usda")
        for path in list(self.modify_paths) + list(extra):
            if self.layer.GetPrimAtPath(path):
                if not path.GetParentPath().IsAbsoluteRootPath():
                    Sdf.CreatePrimInLayer(backup, path.GetParentPath())
                Sdf.CopySpec(self.layer, path, backup, path)
        return backup

    def finish(self) -> None:
        """Call after the edit to record what was created and the post-edit state (redo)."""
        created = _prim_paths(self.layer) - self._before_prims
        self._created = _minimal_roots(created)
        self._after_roots = _minimal_roots(set(self.modify_paths) | set(self._created))
        self._after = Sdf.Layer.CreateAnonymous(".designer_redo.usda")
        for path in self._after_roots:
            if self.layer.GetPrimAtPath(path):
                if not path.GetParentPath().IsAbsoluteRootPath():
                    Sdf.CreatePrimInLayer(self._after, path.GetParentPath())
                Sdf.CopySpec(self.layer, path, self._after, path)

    @property
    def created_paths(self) -> List[Sdf.Path]:
        return list(self._created)

    # -- undo / redo ----------------------------------------------------------------
    def undo(self) -> None:
        with Sdf.ChangeBlock():
            for path in sorted(self._created, key=lambda p: -p.pathElementCount):
                _remove_spec(self.layer, path)
            for path in self.modify_paths:
                self._restore_from(self._before, path)

    def redo(self) -> None:
        if self._after is None:
            return
        with Sdf.ChangeBlock():
            for path in self._after_roots:
                self._restore_from(self._after, path)

    def _restore_from(self, source: Sdf.Layer, path: Sdf.Path) -> None:
        if source.GetPrimAtPath(path):
            if not path.GetParentPath().IsAbsoluteRootPath():
                Sdf.CreatePrimInLayer(self.layer, path.GetParentPath())
            Sdf.CopySpec(source, path, self.layer, path)
        else:
            _remove_spec(self.layer, path)
