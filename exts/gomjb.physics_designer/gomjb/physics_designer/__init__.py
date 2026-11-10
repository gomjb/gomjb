"""Physics Designer extension for NVIDIA Isaac Sim.

The ``core`` package only needs ``pxr`` and can be scripted from Isaac Sim's Script Editor,
standalone Python (``usd-core``) or unit tests.  The Kit extension (UI, undo commands) is
loaded only when running inside Omniverse Kit.
"""
from . import core  # noqa: F401

try:  # Kit runtime only
    import omni.ext  # noqa: F401

    from .extension import PhysicsDesignerExtension  # noqa: F401
except ImportError:  # pragma: no cover - offline usage
    pass
