# Changelog

## [0.1.0] - 2026-11-10
### Added
- Move & rotate tools: nudges in world/local space, rotate about origin / bbox / group, absolute
  world position, align, distribute, drop to ground / onto objects, copy pose.
- Snapping: grid & angle snap, bbox-corner snap, snap to target pivot/bbox, nearest bbox point or
  mesh vertex, face mating / stacking, live gizmo snap (experimental).
- Anchors (named mount frames), anchor-to-anchor connection with optional joint creation.
- Joint authoring for fixed, revolute, prismatic, spherical, distance and D6 joints with limits,
  drives and break forces; local frames derived from one world frame (no start-up jumps).
- Physics properties: rigid / kinematic bodies, colliders with approximation, mass & density,
  physics materials, articulation root, collision filtering, scene and ground plane, removal.
- Parametric mechanism templates: pendulum chain, hydraulic cylinder, hinged door, linear slider,
  multi-stage scissor lift.
- Stage validation with auto-fixes (joint frame realign, missing colliders, mesh approximation,
  physics scene).
- Path-scoped undo/redo for every operation (`PhysicsDesignerEditCommand`).
