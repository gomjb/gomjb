"""The exact solver on classic mechanisms with closed-form answers.

Every model lies in the world XZ plane (Z up) with pins about world +Y, so a planar angle theta
(measured from +X toward +Z) is a rotation of -theta about +Y: a joint's ``pos`` is minus the
change of its relative planar angle. Positions are checked against the mechanism's geometry, and
the driving torque against virtual work, ``T = dV/dphi`` with V the gravitational potential and phi
the drive's rotation about +Y (the sign convention of the single-link tests in test_mbd.py).

The sweeps run the crank through a full turn, so every pin's relative rotation passes +-90 deg
(where Exudyn's Rxyz output switches branch, issue #108) and +-180 deg (where it wraps).
"""
import math

import pytest

pytest.importorskip("exudyn")

from pxr import Gf, UsdGeom  # noqa: E402

from gomjb.physics_designer.core import joints as jt  # noqa: E402
from gomjb.physics_designer.core import mechanisms as me  # noqa: E402
from gomjb.physics_designer.core import modeling as md  # noqa: E402
from gomjb.physics_designer.core import physics as ph  # noqa: E402
from gomjb.physics_designer.core import transform as tf  # noqa: E402
from gomjb.kinematics.core.mbd import MbdModel  # noqa: E402

G = 9.81
ROOT = "/World/M"


# ---------------------------------------------------------------------------------------
# Model building
# ---------------------------------------------------------------------------------------
def _root(stage, yaw: float = 0.0):
    """Mechanism root; ``yaw`` (deg about Z) turns the whole mechanism, which gravity ignores."""
    ph.ensure_physics_scene(stage)
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    ph.apply_articulation_root(root)
    return root


def _planar(p, yaw: float = 0.0) -> Gf.Vec3d:
    """Planar point (x, z) -> world, the plane turned by ``yaw`` about Z."""
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return Gf.Vec3d(p[0] * c, p[0] * s, p[1])


def _link(stage, name, p0, p1, mass, yaw=0.0):
    """A link body from planar point p0 to p1 (pins at its ends, centre of mass in the middle)."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dz)
    body = md.create_body(stage, f"{ROOT}/{name}", "link", (length, 0.04, 0.02), mass=mass)
    rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), -math.degrees(math.atan2(dz, dx))) * Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw)
    tf.set_world_pose(body, _planar(((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2), yaw), rot.GetQuat())
    return body


def _pin(stage, name, b0, b1, p, yaw=0.0, driven=False):
    axis = Gf.Vec3d(-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)), 0)  # the plane's normal
    spec = jt.JointSpec("revolute", name=name, axis="Y", drive=jt.DriveSpec(stiffness=1e6) if driven else None)
    return jt.create_joint_at(stage, spec, b0, b1, location="custom", custom_position=_planar(p, yaw),
                              world_axis=tuple(axis), parent_path=f"{ROOT}/Joints")


def _deg(a: float) -> float:
    return math.degrees(a)


def _unwrap(values):
    out = [values[0]]
    for v in values[1:]:
        out.append(out[-1] + (v - out[-1] + 180.0) % 360.0 - 180.0)
    return out


# ---------------------------------------------------------------------------------------
# Closed-form kinematics
# ---------------------------------------------------------------------------------------
def fourbar(a, b, c, d, th2, prev_c=None):
    """Crank a at A=(0,0), coupler b, rocker c at D=(d,0): (B, C) for crank angle th2 (rad).
    Of the two assemblies, the one nearest ``prev_c`` (default: C above the ground link)."""
    B = (a * math.cos(th2), a * math.sin(th2))
    ex, ez = d - B[0], 0.0 - B[1]
    e = math.hypot(ex, ez)
    ux, uz = ex / e, ez / e
    along = (b * b - c * c + e * e) / (2 * e)
    h = math.sqrt(max(b * b - along * along, 0.0))
    P = (B[0] + along * ux, B[1] + along * uz)
    cands = [(P[0] - h * uz, P[1] + h * ux), (P[0] + h * uz, P[1] - h * ux)]
    if prev_c is None:
        C = max(cands, key=lambda q: q[1])
    else:
        C = min(cands, key=lambda q: (q[0] - prev_c[0]) ** 2 + (q[1] - prev_c[1]) ** 2)
    return B, C


def slider_x(r, l, th):
    return r * math.cos(th) + math.sqrt(l * l - (r * math.sin(th)) ** 2)


def _angle(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def _dV(fn, th, h=1e-6):
    return (fn(th + h) - fn(th - h)) / (2 * h)


# ---------------------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------------------
FOURBAR = dict(a=0.3, b=1.0, c=0.8, d=1.0, ma=1.0, mb=2.0, mc=1.5)  # Grashof crank-rocker
TH0 = math.radians(60.0)


def build_fourbar(stage, yaw=0.0):
    a, b, c, d = (FOURBAR[k] for k in "abcd")
    _root(stage, yaw)
    B, C = fourbar(a, b, c, d, TH0)
    A, D = (0.0, 0.0), (d, 0.0)
    crank = _link(stage, "Crank", A, B, FOURBAR["ma"], yaw)
    coupler = _link(stage, "Coupler", B, C, FOURBAR["mb"], yaw)
    rocker = _link(stage, "Rocker", D, C, FOURBAR["mc"], yaw)
    _pin(stage, "PinA", None, crank, A, yaw, driven=True)
    _pin(stage, "PinB", crank, coupler, B, yaw)
    _pin(stage, "PinC", coupler, rocker, C, yaw)
    _pin(stage, "PinD", None, rocker, D, yaw)


def fourbar_answer(th2, prev_c=None):
    a, b, c, d = (FOURBAR[k] for k in "abcd")
    B, C = fourbar(a, b, c, d, th2, prev_c)
    D = (d, 0.0)
    th3, th4 = _angle(B, C), _angle(D, C)
    return B, C, th3, th4


def fourbar_potential(th2, prev_c):
    B, C, _, _ = fourbar_answer(th2, prev_c)
    A, D = (0.0, 0.0), (FOURBAR["d"], 0.0)
    z = [(A[1] + B[1]) / 2, (B[1] + C[1]) / 2, (D[1] + C[1]) / 2]
    return G * sum(m * zi for m, zi in zip((FOURBAR["ma"], FOURBAR["mb"], FOURBAR["mc"]), z))


SLIDER = dict(r=0.25, l=0.8, mr=1.0, ml=2.0, ms=3.0)


def build_slider_crank(stage):
    r, l = SLIDER["r"], SLIDER["l"]
    _root(stage)
    B = (r * math.cos(TH0), r * math.sin(TH0))
    S = (slider_x(r, l, TH0), 0.0)
    crank = _link(stage, "Crank", (0, 0), B, SLIDER["mr"])
    rod = _link(stage, "Rod", B, S, SLIDER["ml"])
    block = md.create_body(stage, f"{ROOT}/Slider", "box", (0.12, 0.08, 0.06), mass=SLIDER["ms"])
    tf.set_world_position(block, _planar(S))
    _pin(stage, "PinA", None, crank, (0, 0), driven=True)
    _pin(stage, "PinB", crank, rod, B)
    _pin(stage, "PinS", rod, block, S)
    jt.create_joint_at(stage, jt.JointSpec("prismatic", name="Slide", axis="X"), None, block, location="custom",
                       custom_position=_planar(S), world_axis="X", parent_path=f"{ROOT}/Joints")


def slider_potential(th):
    r = SLIDER["r"]
    zb = r * math.sin(th)
    return G * (SLIDER["mr"] * zb / 2 + SLIDER["ml"] * zb / 2)  # the slider stays at z = 0


# ---------------------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------------------
CRANK_TURN = [10.0 * k for k in range(37)]  # deg, a full turn of the crank


def sweep(model, drive, planar_degs, substeps=2):
    """Static sweep of ``drive`` to planar crank angles (deg from the built pose)."""
    return model.static_sweep([drive], [-p for p in planar_degs], substeps)


@pytest.mark.parametrize("yaw", [0.0, 37.0])
def test_fourbar_full_turn_positions_and_torque(stage, yaw):
    """Grashof crank-rocker, crank driven through 360 deg: the rocker follows Freudenstein's
    geometry, the free pins read their relative angles continuously past +-90 / +-180 deg, and
    the crank torque is the virtual-work torque. Turning the plane about Z changes nothing."""
    build_fourbar(stage, yaw)
    model = MbdModel(stage, ROOT)
    assert model.drive_names == ["PinA"]
    rec = sweep(model, "PinA", CRANK_TURN)
    _, C0, th3_0, th4_0 = fourbar_answer(TH0)
    prev = C0
    exp = {"PinB": [], "PinC": [], "PinD": [], "torque": []}
    for p in CRANK_TURN:
        th2 = TH0 + math.radians(p)
        _, C, th3, th4 = fourbar_answer(th2, prev)
        exp["PinB"].append(-_deg((th3 - th2) - (th3_0 - TH0)))
        exp["PinC"].append(-_deg((th4 - th3) - (th4_0 - th3_0)))
        exp["PinD"].append(-_deg(th4 - th4_0))
        prev_c = prev
        exp["torque"].append(-_dV(lambda t: fourbar_potential(t, prev_c), th2))  # T = dV/dphi = -dV/dth
        prev = C
    for pin in ("PinB", "PinC", "PinD"):
        got = rec.channels[f"pos:{pin}"]
        want = _unwrap([((v + 180.0) % 360.0) - 180.0 for v in exp[pin]])  # atan2 wraps; the joint does not
        assert got == pytest.approx(want, abs=1e-5), pin  # float32 USD poses
    assert rec.channels["pos:PinA"] == pytest.approx([-p for p in CRANK_TURN], abs=1e-6)
    assert rec.channels["force:PinA"] == pytest.approx(exp["torque"], abs=1e-5)
    # the crank-coupler pin turns once relative to the crank: its reading runs through 360 deg
    assert max(rec.channels["pos:PinB"]) - min(rec.channels["pos:PinB"]) > 300.0


def test_slider_crank_full_turn(stage):
    """Slider-crank, crank through 360 deg: slider stroke x(th) = r cos th + sqrt(l^2 - r^2 sin^2 th)
    and the virtual-work crank torque (the slider does no work against gravity)."""
    build_slider_crank(stage)
    model = MbdModel(stage, ROOT)
    rec = sweep(model, "PinA", CRANK_TURN)
    r, l = SLIDER["r"], SLIDER["l"]
    x0 = slider_x(r, l, TH0)
    want_x = [slider_x(r, l, TH0 + math.radians(p)) - x0 for p in CRANK_TURN]
    assert rec.channels["pos:Slide"] == pytest.approx(want_x, abs=1e-7)
    want_t = [-_dV(slider_potential, TH0 + math.radians(p)) for p in CRANK_TURN]
    assert rec.channels["force:PinA"] == pytest.approx(want_t, abs=1e-5)
    # the rod-slider pin swings within +-asin(r/l) of the horizontal: no branch issue there
    phi0 = math.asin(r * math.sin(TH0) / l)
    # body0 = rod, body1 = slider: the relative planar angle is -theta_rod, so pos = +d(theta_rod)
    want_s = [_deg(-math.asin(r * math.sin(TH0 + math.radians(p)) / l) + phi0) for p in CRANK_TURN]
    assert rec.channels["pos:PinS"] == pytest.approx(want_s, abs=1e-5)


def test_driven_pin_reads_its_target_through_a_full_turn(stage):
    """A single link driven 0 -> 350 deg about +Y: the angle reads the target past +-90 and
    +-180 deg (Exudyn's raw output switches branch and wraps), the torque is -m g r cos."""
    _root(stage)
    link = _link(stage, "Arm", (0, 0), (1.0, 0), 2.0)
    _pin(stage, "Hinge", None, link, (0, 0), driven=True)
    model = MbdModel(stage, ROOT)
    targets = [10.0 * k for k in range(36)]
    rec = model.static_sweep(["Hinge"], targets, 2)
    assert rec.channels["pos:Hinge"] == pytest.approx(targets, abs=1e-6)
    assert rec.channels["force:Hinge"] == pytest.approx(
        [-2 * G * 0.5 * math.cos(math.radians(t)) for t in targets], abs=1e-5)


@pytest.mark.parametrize("stages", [1, 3])
def test_scissor_built_folded_reads_crossing_angle_past_90(stage, stages):
    """A scissor built folded (10 deg) and raised to 70 deg: the crossing arms turn 2*(a - 10)
    deg relative to each other, past 90 deg; the actuator force matches virtual work."""
    ph.ensure_physics_scene(stage)
    res = me.build_scissor_lift(stage, stages=stages, sides=1, angle=10.0, loop_closure="rigid")
    model = MbdModel(stage, res["root"][0])
    angles = [20.0, 40.0, 50.0, 56.0, 62.0, 70.0]
    rec = model.static_sweep(["Actuator"], [me.scissor_slider_offset(1.2, 10.0, a) for a in angles], 5)
    for k, a in enumerate(angles):
        for i in range(stages):
            assert rec.channels[f"pos:Center_{i}"][k] == pytest.approx(-2 * (a - 10.0), abs=1e-5)  # float32 poses
        expected = me.scissor_actuator_force(1.2, a, stages, 20.0, 5.0, sides=1)
        assert -rec.channels["force:Actuator"][k] == pytest.approx(expected, rel=1e-6)


def test_free_angle_reference_resets_on_restore(stage):
    """After restore() jumps to another configuration, a free pin is read on its own branch."""
    build_fourbar(stage)
    model = MbdModel(stage, ROOT)
    start = model.solution()
    rec = sweep(model, "PinA", [0.0, 90.0, 180.0])
    model.restore(start)
    model.set_drive("PinA", 0.0)
    again = model.solve_static()
    assert again["pos:PinD"] == pytest.approx(rec.channels["pos:PinD"][0], abs=1e-6)


@pytest.mark.parametrize("token", "XYZ")
def test_revolute_hinge_carries_the_moment_on_its_locked_axes(stage, token):
    """A door on a vertical revolute hinge (two rotations constrained), held about the hinge
    axis by a torsional bushing: the hinge carries the panel's gravity moment, -r x F, on its
    locked axes - whichever joint-frame axis is the hinge axis (Exudyn orders and signs the
    multipliers of a revolute differently from a fixed joint)."""
    _root(stage)
    m, rx = 4.0, 0.45
    panel = md.create_body(stage, f"{ROOT}/Panel", "box", (0.9, 0.04, 1.0), mass=m)
    tf.set_world_position(panel, (rx, 0, 0))
    hinge = jt.create_joint_at(stage, jt.JointSpec("revolute", name="Hinge", axis=token), None, panel,
                               location="custom", custom_position=(0, 0, 0), world_axis="Z",
                               parent_path=f"{ROOT}/Joints")
    spring = jt.bushing_spec([d for d in jt.D6_DOFS if d != f"rot{token}"], angular_stiffness=50.0)
    spring.axis = token  # its frame maps the same axis onto world Z as the hinge's
    jt.mark_bushing(jt.create_joint_at(stage, spring, None, panel, location="custom", custom_position=(0, 0, 0),
                                       world_axis="Z", parent_path=f"{ROOT}/Joints"))
    model = MbdModel(stage, ROOT)
    v = model.solve_static()
    (_, r0), _ = jt.joint_world_frames(hinge)
    rot = Gf.Matrix4d().SetRotate(Gf.Rotation(r0)).ExtractRotationMatrix()
    world = Gf.Vec3d(0, -rx * m * G, 0)  # -r x F with r = (rx, 0, 0), F = (0, 0, -m g)
    local = rot * world  # Gf row vectors: world -> J0 components
    got = [v[f"joint_torque:Hinge:{c}"] for c in "xyz"]
    assert got == pytest.approx(list(local), abs=1e-6)
    assert v[f"joint_force:Hinge:{'xyz'['XYZ'.index(token)]}"] == pytest.approx(m * G, rel=1e-6)  # along the axis


@pytest.mark.parametrize("yaw", [0.0, 37.0])
def test_released_equations_read_zero_in_gimbal_lock(stage, yaw):
    """A crank pin (free axis = Rxyz's middle axis y) at exactly 90 / 270 deg: Exudyn's Rotation
    output is in gimbal lock and gives an arbitrary x / z pair (about -52 / -52 deg here), which
    the redundant violation check read as a released equation 52 deg off and stopped a valid
    mechanism. The pin's locked axes are measured as the tilt of its axis: zero."""
    build_fourbar(stage, yaw)
    model = MbdModel(stage, ROOT)
    assert ("PinA", "rotX") in model.released and ("PinA", "rotZ") in model.released
    for p in (90.0, 270.0):
        model.solve_position({"PinA": -p}, 9)
        for _, axis, value in model.redundant_violation():
            assert abs(value) < 1e-6, (p, axis, value)
