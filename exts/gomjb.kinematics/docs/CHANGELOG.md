# Changelog

## [0.1.5] - 2026-11-26
### Fixed
- Joint torques: Exudyn's `TorqueLocal` holds the multipliers of the rotation constraint equations,
  whose order and signs depend on how many rotations are constrained (`CObjectJointGeneric`: three
  -> z0.y1, z0.x1, x0.y1; two -> f0.l at the locked axes; one -> f1_0.f2_1). The fixed sign table
  `TORQUE_SIGN` was right for three only: when redundancy release freed rotations of a driven pin
  (every four-bar and slider-crank) the drive torque had the wrong sign, and undriven revolutes
  reported permuted torque axes. The torque is now rebuilt as `sum_i lambda_i dg_i/dtheta_k` from
  the same equations (`MbdModel._joint_reaction`).
- Joint forces of a joint with released translations and a driven rotation are turned back from the
  offset-rotated frame into J0.
- Redundant violation false positive: a pin whose free axis is Rxyz's middle axis (y) is in gimbal
  lock at +-90 deg, where Exudyn's `Rotation` gives an arbitrary x / z pair (-52 / -52 deg on a
  satisfied crank pin), so `check_redundant` stopped a valid four-bar at 90 deg. A joint with one
  rotational DOF is now measured without Euler angles: the twist about that axis (swing-twist) and
  the tilt of the axis about each locked axis (`_joint_rotation`; `_euler_rotation` otherwise).
- Joint angles are continuous past +-180 deg: a driven axis around its target, a free axis around
  its last value (`restore()` resets that reference).

### Added
- `tests_offline/test_models.py`: four-bar crank-rocker (full turn, plane also turned about Z),
  slider-crank (full turn), a pin driven 0 -> 350 deg, scissor lifts built folded and raised past a
  90 deg crossing angle, a door hinge carrying its moment on the locked axes, and the released
  equations of a crank pin in gimbal lock, against closed-form geometry and virtual work. Max errors:
  angles 4e-6 deg, torques 7e-7 N*m, scissor actuator force 2e-7 relative.

## [0.1.4] - 2026-11-26
### Added
- Joint limits are enforced by the exact solver as stops on the joint coordinate, Adams' BISTOP
  (`bistop`): no force inside the range, beyond it `k * penetration^2.2` (Adams: an exponent of 1.0
  is a corner) with damping ramped in over the penetration depth (Adams STEP), sized so the
  mechanism's whole weight presses the stop in by `limit_penetration` (0.1 mm / 0.01 deg) and
  critically damped there. Revolute limits use Exudyn's torsional spring-damper with continuous
  rotation, prismatic ones its linear spring-damper; no contact geometry. Before, limits were not
  enforced (listed as "joint limits not enforced").
- A drive target beyond its joint's limits is refused: a prescribed motion cannot be stopped
  (Adams does not allow BISTOP in a MOTION).

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
