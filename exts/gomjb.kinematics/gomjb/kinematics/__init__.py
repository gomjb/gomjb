"""Kinematics tool for NVIDIA Isaac Sim: the exact multibody solver (Exudyn) on the USD model the
Physics Designer makes, and kinematic mechanisms — moved by a motion table or a co-simulation
with the exact solver — in the running simulation.

The ``core`` package needs ``pxr``, numpy, ``gomjb.physics_designer.core`` and, for solving,
``exudyn``; it runs in Isaac Sim's Script Editor, standalone Python or unit tests. The Kit
extension (window, runtime) is loaded only inside Omniverse Kit.
"""
from . import core  # noqa: F401

try:  # Kit runtime only
    import omni.ext  # noqa: F401

    from .extension import KinematicsExtension  # noqa: F401
except ImportError:  # pragma: no cover - offline usage
    pass
