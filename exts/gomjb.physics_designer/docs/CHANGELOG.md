# Changelog

## [0.9.0] - 2026-11-12
### Added
- Motion table cache (`kinematics.cached_table`, `save_table` / `load_table`): tables are stored in
  Kit's cache (`${omni_cache}/gomjb.physics_designer/motion_tables`) under the mechanism's
  kinematic fingerprint, the stage's scale and the table format. *Apply kinematics* builds the
  table and shows its size, error and time; Play reads it, as does a batch run of the same model.

### Changed
- Grouping copies of a mechanism: roots composed of the same USD specs (Isaac Sim's cloner by
  default inherits the source environment) group without reading a property. Other copies are
  compared by fingerprint, computed once per group. A fingerprint now includes the mechanism's
  joints outside its root. `kinematic_groups` returns the fingerprint with each group.
- Stage scans for kinematic roots and joint motions no longer enter kinematic mechanisms. The
  physics scene is found breadth first (`physics.find_physics_scene`).
- Exact solver: joints to dynamic bodies outside the mechanism (a payload fixed to it) are left
  out (listed in the notes) instead of fixing the mechanism to the ground.
- First step after Play with 4096 cloned lifts: 52 s -> 7.7 s with the cached table (1.3 s of it
  in the designer, the rest PhysX starting the scene); 27.9 s the first time.

## [0.8.0] - 2026-11-12
### Added
- Kinematic mechanisms (`core.kinematic_mode`): a mechanism's *Kinematics* is `physics` (PhysX
  simulates its joints, as before), `table` or `cosim` — the exact solver moves its bodies as
  kinematic bodies and PhysX simulates only what touches them (Process Simulate's split of device
  kinematics and physics). Switching makes the bodies kinematic, disables the joints inside the
  mechanism and removes its articulation root; every change is marked and `physics` restores the
  model exactly. Analysis section: *Kinematics*, *Table tolerance*, *Apply kinematics* (undoable).
- `core.kinematics`: motion table — position analysis along the drives' stroke (their joint
  limits), refined where the kinematics bends until interpolation is within a tolerance (default
  0.1 mm, rotation error weighted by the mechanism's radius) — and co-simulation — position
  analysis at every physics step, any number of independent motions.
- `kinematic_driver`: one Warp kernel per step for all copies of a mechanism; CPU pipeline through
  kinematic targets, GPU pipeline through pose + centre-of-mass velocity (PhysX 110.3 has no GPU
  kinematic targets). The motion runtime groups copies of a mechanism (cloned environments,
  duplicates — equal root-relative bodies and joints) under one table and one driver.
- `MbdModel`: `body_poses()`, `solve_position()`, `solution()` / `restore()`; `rigid_bushings` solves
  designer bushings as the joints they replace.

### Changed
- Exact solver: a joint to the ground is anchored on its body side, so a mechanism moved or cloned
  after its world joint was made is solved where it stands; non-converging static solutions raise
  instead of returning the last iterate; unsupported joint types (distance, PhysX gear / rack and
  pinion) and mimic joints are rejected instead of silently skipped.
- Validation skips disabled joints (not simulated).

## [0.7.0] - 2026-11-12
### Added
- Bushings in place of joints (`core.bushing`, `joints.bushing_spec` / `convert_to_bushing`): a D6
  spring-damper, stiff on the axes the joint locks and zero on its free axes, marked
  `designer:bushing` with its sizing recorded on the prim.
- Automatic stiffness: linear `k = M*g/delta` (delta 0.1 mm), angular `k = M*g*L/theta` (0.01 deg),
  critical damping on the reduced mass / inertia, from the mechanism the joint belongs to.
- Joint model in the Joints section: Auto (an undriven loop-closing joint becomes a bushing), Bushing
  (every undriven joint), Rigid joint; tolerances and damping ratio; *Convert selected to bushing*.
- Exact solver maps designer bushings to the same spring-damper (`MbdModel.has_bushings`).
- Validation: a rigid loop closure inside an articulation gets an auto-fix to a bushing.
- Kit test for the bushing scissor next to another articulation on PhysX GPU dynamics.

### Changed
- `build_scissor_lift(loop_closure="bushing")` by default; `"rigid"` keeps revolute loop closures
  (needed for Newton, whose MuJoCo solver closes loops with revolute joints only).
- The GPU closed-loop warning ignores bushing loop closures.

## [0.6.0] - 2026-11-11
### Changed
- Joints are created the RecurDyn way: a joint type button (Fixed, Revolute, Translational,
  Spherical), a creation method (Body, Body, Point[, Direction] / Point[, Direction]) and clicks in
  the viewport: base body, action body, point, direction. The joint Z axis is the rotation /
  sliding axis; without a direction it is the working-plane normal. A click near a marker (a link
  pin) snaps to it, otherwise to the clicked geometry's corners / edge middles / face centers; a
  click on anything that is not a rigid body picks the ground (`core.joint_builder`).
- The former selection + location + world-axis joint form is removed; joint properties (limits,
  drive, break force / torque, collision, articulation exclusion) apply to the next joint.

### Added
- Viewport picking (`ui.viewport_pick`): a click gesture over the viewport and
  `viewport_api.request_query` for the clicked prim and surface point (Kit's own picking pattern).
- Joint motions, RecurDyn's "Include Motion" (`core.motion`): displacement, velocity or
  acceleration of a revolute (deg) or translational joint as an expression of TIME
  (`core.expressions`: STEP, SIN, ... in a whitelisted language), stored on the joint prim with
  the unit system it was written in. Joints without a drive get a stiff tracking drive.
- Motions in the PhysX simulation (`motion_runtime`): drive targets set before every physics step
  from Play, back to TIME = 0 on Stop.
- Exact solver *Simulate* (dynamic analysis to an end time): joints with a motion follow it
  exactly (`MbdModel.simulate` uses the motions by default).

## [0.5.0] - 2026-11-11
### Added
- Unit system selector (MKS, MMKS, CGS, MTS, IPS; time s, angles deg) for every input and result in
  the window: labels carry the unit, values keep their physical meaning when the system changes,
  the choice persists. The stage keeps its own scale; values are converted (`core.units`).
- Joint limit and drive inputs take the joint type's quantity (deg / length, N*mm/deg / N/mm, ...);
  separate break force and break torque inputs.
- Template parameters carry unit annotations (`Length`, `Mass`, `LinearStiffness`, ...).

### Changed
- Template builders take SI (m, kg, N/m, N*m*s/deg, ...) and convert every length, joint limit and
  drive value to the stage's units, fixed internal sizes included: a template has the same size
  on a metre, centimetre or millimetre stage (`joints.spec_from_si` converts a joint spec).
- `physics.set_mass` takes kg and kg/m^3 and authors them in stage units (it wrote the raw value).
- Exact-solver results stay SI in the recording; plot, summary and CSV show the unit system.
- `Recording.x_name` / `x_unit` replace the `x_label` field (`x_label` is now derived).

## [0.4.0] - 2026-11-11
### Added
- Exact solver (`core.mbd`, Exudyn 1.13.6 through `[python.pipapi]`): the USD model as an exact
  constraint system — loop-closing joints included, drives as prescribed motions whose
  multipliers are the actuator forces. Static equilibrium, drive sweeps (several drives together)
  and dynamic runs (generalized-alpha); full 6-axis reactions per joint. Matches the analytic
  answers to < 1e-6 (prismatic, rotated revolute axes, 1–3 stage scissor lifts).
- Redundant constraints: `release` (dependent equations released, loop joints first, mobility
  kept, and listed — Adams / MotionSolve practice) or `bushings` (undriven joints as stiff
  bushings, so the split over redundant load paths follows stiffness — the Adams / Altair fix).
- Analysis section: Exact solver controls (redundancy, sweep drives / range / points, Solve static,
  Sweep drive); exact results share the plot, summary and CSV.
- README 2.10: redundancy handling in Adams, Simscape, MotionSolve, Abaqus, RecurDyn, Simcenter 3D
  Motion, Exudyn and the Ajou University group's solver — only what official documentation,
  original papers or source code confirm.

### Fixed
- `bushings` sweeps failed ("SolveStatic terminated due to errors"): re-solving an equilibrium could
  not meet the 1e-12 absolute residual tolerance with stiff bushings; the tolerance is now 1e-9 of
  the model weight.
- Sweeps running towards smaller drive values drew no curve (XYPlot needs increasing x): plotted
  samples are ordered by x.

### Changed
- `Recording.x_label` names the abscissa (time, or the drive value of a sweep) in plots and CSV.
- Analysis summary lists the results first and the solver report below, wrapped; solver notes are
  grouped per kind ("joint limits not enforced: Actuator_L, ...").
- Analysis target field is "Mechanism".

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
