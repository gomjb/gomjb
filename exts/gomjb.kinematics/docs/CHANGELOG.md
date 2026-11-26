# Changelog

## [0.1.3] - 2026-11-26
### Added
- Flipped-assembly check (`AssemblyFlip`): the enforced rotation equations of a joint also hold with
  its axes anti-parallel (Adams documents the same for its JOINTs), so one large static step could
  converge to the pose turned 180 deg and return it as a valid solution (measured: 0 -> 170 deg in
  one step gave -10 deg). After every solved configuration, static and every dynamic step, an
  enforced locked rotation axis or a driven one more than 90 deg from its target (motion values
  during dynamics) stops the analysis and names the joint. Continuation reaches the same target.

### Fixed
- A static step that did not converge raised Exudyn 1.13's `SolverError` ("SolveStatic terminated
  due to errors") instead of the solver's own message with the drive targets.

## [0.1.2] - 2026-11-26
### Added
- Redundant violation check (RecurDyn's check of the same name, error C0276): a released redundant
  equation is no longer enforced, so incompatible redundant constraints (non-parallel pin axes,
  misaligned joints) used to hide their gap in it and the solution looked valid. Every solved
  configuration - static, sweep, position analysis (motion tables), every dynamic step - measures
  the released equations and raises `RedundantViolation` beyond `redundant_tolerance`
  (default 1e-6 m, 0.001 deg). `MbdModel.max_violation` and the report show the largest value.
  Measured: compatible models stay at 1e-15 m; a 0.1 deg tilt of one loop pin of a two-sided
  3-stage scissor hides a 96 mm gap at 45 deg.
- README 2.2: RecurDyn's redundant-constraint handling from FunctionBay's documentation.

### Fixed
- Joint rotations read from Exudyn's Tait-Bryan output picked the wrong one of its two equivalent
  branches once a free rotation passed 90 deg: a revolute driven to 120 deg read 60 deg, and the
  satisfied equations of a scissor's crossing arms read 180 deg off. The branch that keeps the
  joint's locked axes smallest is used now (`pos:` channels and the violation check).

## [0.1.1] - 2026-11-26
### Added
- Choosing a closed-loop mechanism in `physics` mode on GPU dynamics (Use selection), or applying
  `physics` to it, notes in the status line that PhysX diverges on its loops there (Isaac Sim 6.1)
  and advises `table` / `cosim`. Needs gomjb.physics_designer 0.10.3 (`validation.closed_loop_roots`).
- README 2.3: GPU dynamics with closed-loop mechanisms.

## [0.1.0] - 2026-11-12
### Added
- Split from gomjb.physics_designer 0.9.0 (its history covers what came before). It depends on that
  extension for the model, the undo commands, the motion runtime and the result view.
- `core.mbd` (exact solver, Exudyn), `core.kinematic_mode`, `core.kinematics` (adaptive motion
  tables with a disk cache, co-simulation), `driver` (one Warp kernel for all copies of a
  mechanism) and `runtime` (a participant of the Physics Designer's motion runtime).
- "Kinematics" window (Tools > Kinematics) with these sections: Mechanism, Kinematics (mode, table
  tolerance, Apply kinematics), Exact solver (static, sweep, simulate, redundancy handling) and
  Results.
- The motion table cache moved to `${omni_cache}/gomjb.kinematics/motion_tables`.
