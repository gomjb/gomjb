# Changelog

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
