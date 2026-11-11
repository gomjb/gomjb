# Changelog

## [0.3.0] - 2026-11-11
### Added
- Modeler: box / cylinder / sphere / capsule / cone / link bodies with material density presets,
  analytic mass, centre of mass and principal inertia (checked against numeric integration),
  body types (dynamic, kinematic, static, visual), placement on a prim or an anchor, and editable
  parameters. Links carry PinA / PinB anchors.
- Analysis: per-physics-step recording of articulation joint positions, DOF forces and joint
  reactions (SI units), plot with summary, CSV export. Verified against exact answers
  (0.00 % on single bodies, within 0.4 % on the 3-stage scissor lift).
- `mechanisms.scissor_actuator_force()` static solution; scissor drive sizing uses it.
- Validation: non-fixed joint from the world into an articulation.
- `extension.get_window()` for scripting.

### Changed
- Templates build their bodies with the modeler (explicit inertia); scissor arms are links.
- Pendulum chain hangs from a support link fixed to the world (PhysX articulations attach to
  the world with a fixed joint only).

### Fixed
- Modeler masses and recorded forces respect metersPerUnit / kilogramsPerUnit.

## [0.2.0] - 2026-11-11
Verified in Isaac Sim 6.1.0 (Kit 110, Windows) with PhysX and Newton.

### Fixed
- Scissor lift collapsed under its own weight: the closed loops are now one fixed-base
  articulation with only the loop-closing revolute pins `excludeFromArticulation`.
- Scissor slider kinematics used half the fixed-pin-to-slider span: travel limits and
  `scissor_slider_offset_for_height()` were off by a factor of two.
- Joints are authored parent-to-child (body0 = parent) as Newton requires.
- Window text rendered as `?` (Isaac Sim's UI font has no Hangul / arrow glyphs): UI is ASCII.
- Standalone example read the authored USD pose during simulation; it now uses the
  `isaacsim.core.experimental` tensor API.

### Changed
- Every template is an articulation (ArticulationRootAPI on its root); the pendulum's
  `articulation` option is gone.
- Scissor actuator drive is sized from the lifted weight: `sag_tolerance` replaces
  `drive_stiffness` / `drive_damping`.
- A Physics Scene created by the designer runs PhysX on the CPU (the GPU solver diverged
  with a closed-loop articulation next to other articulations).

### Added
- Validation: joints between bodies outside any articulation (rejected by Newton) and GPU
  dynamics with a closed-loop articulation (auto-fix: CPU dynamics).
- Kit tests that build the real omni.ui window and drive joint / template creation.

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
