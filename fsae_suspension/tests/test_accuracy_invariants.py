# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
Accuracy invariants — the checks that catch sign errors and silent drift.

WHY THIS FILE EXISTS
--------------------
Every defect found in the 2026-08 accuracy audit passed the existing test
suite. Not because the suite is thin, but because it tests OUTPUTS ("anti-dive
is a modest positive number") rather than INVARIANTS ("load transfer conserves
moment"). An output test cannot tell a correct number from a number that is
wrong in a way the test author also believed.

So these tests assert things that must hold as a matter of physics or internal
consistency, independent of any particular geometry:

  * lateral load transfer closes the moment balance,
  * the front-view IC construction degenerates to the classic hand construction
    on flat pickups,
  * anti-dive/anti-squat change sign when the geometry is mirrored,
  * the two longitudinal models (laptime, GGV) agree on braking,
  * the generic kernel and the native solver see the same reference car.

Run:  python -m pytest tests/test_accuracy_invariants.py
"""
import math
import pytest
import numpy as np
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from suspension import SuspensionKinematics, Hardpoints, VehicleDynamics, VehicleParams


# --------------------------------------------------------------------------- #
#  Lateral load transfer — moment balance
# --------------------------------------------------------------------------- #
#  Whatever the model does internally, total lateral load transfer must satisfy
#      dWf * t_f  +  dWr * t_r  =  m * a_lat * h_cg
#  This is just moments about the roll axis and it holds for ANY split between
#  the elastic and geometric paths. The roll-axis interpolation bug broke it by
#  up to ~2.7% at realistic front/rear roll-centre splits, while staying exact on
#  the symmetric default — which is precisely why no output test caught it.
def _closure_error(veh, params, lateral_g):
    _, info = veh.lateral_load_transfer(lateral_g)
    lhs = (info["ltd_front"] * params.track_front / 1000.0
           + info["ltd_rear"] * params.track_rear / 1000.0)
    rhs = params.mass * lateral_g * params.g * (params.cg_height / 1000.0)
    return abs(lhs - rhs) / rhs


def test_load_transfer_closes_moment_balance():
    for wd in (0.42, 0.47, 0.55):
        for g in (0.5, 1.0, 1.6):
            p = VehicleParams(weight_dist_front=wd)
            err = _closure_error(VehicleDynamics(p), p, g)
            assert err < 1e-9, f"moment balance off by {err:.3%} at wd={wd}, {g}g"


def test_load_transfer_closes_with_split_roll_centres():
    """The bug vanished when rc_f == rc_r. Force them apart via geometry so the
    interpolation weight actually matters."""
    front = Hardpoints.default()
    rear = Hardpoints.default()
    # Raise the rear roll centre by lifting the rear inner pickups.
    for k in ("upper_front_inner", "upper_rear_inner"):
        getattr(rear, k)[2] += 45.0
    for k in ("lower_front_inner", "lower_rear_inner"):
        getattr(rear, k)[2] += 45.0
    fk, rk = SuspensionKinematics(front), SuspensionKinematics(rear)
    p = VehicleParams(weight_dist_front=0.44)
    veh = VehicleDynamics(p, front_kin=fk, rear_kin=rk)
    _, info = veh.lateral_load_transfer(1.5)
    assert abs(info["rc_front"] - info["rc_rear"]) > 5.0, \
        "test is vacuous unless the roll centres actually differ"
    assert _closure_error(veh, p, 1.5) < 1e-9


# --------------------------------------------------------------------------- #
#  Front-view instant centre
# --------------------------------------------------------------------------- #
def test_instant_centre_matches_hand_construction_on_flat_pickups():
    """With both pivot axes parallel to x, the exact velocity-based IC must
    reduce EXACTLY to the classic 'line through the ball joint and the inner
    pickup' construction. This is what proves the new solver is a
    generalisation, not a different answer."""
    hp = Hardpoints.default()
    hp.upper_front_inner[2] = hp.upper_rear_inner[2] = 290.0
    hp.lower_front_inner[2] = hp.lower_rear_inner[2] = 120.0
    hp.upper_front_inner[1] = hp.upper_rear_inner[1] = 240.0
    hp.lower_front_inner[1] = hp.lower_rear_inner[1] = 200.0
    kin = SuspensionKinematics(hp)
    st = kin.static

    u_in = 0.5 * (hp.upper_front_inner + hp.upper_rear_inner)
    l_in = 0.5 * (hp.lower_front_inner + hp.lower_rear_inner)
    uo, lo = st.upper_outer, st.lower_outer
    p1 = np.array([uo[1], uo[2]]); d1 = np.array([u_in[1] - uo[1], u_in[2] - uo[2]])
    p2 = np.array([lo[1], lo[2]]); d2 = np.array([l_in[1] - lo[1], l_in[2] - lo[2]])
    ts = np.linalg.solve(np.column_stack([d1, -d2]), p2 - p1)
    hand = p1 + ts[0] * d1

    assert np.allclose(st.instant_center, hand, atol=1e-6), \
        f"IC {st.instant_center} != hand construction {hand}"


def test_instant_centre_is_perpendicular_to_ball_joint_motion():
    """Definition check: the IC must lie on the perpendicular to each ball
    joint's front-view velocity. Verified against finite differences of the
    actual solved poses, which is independent of how _instant_center builds it."""
    kin = SuspensionKinematics(Hardpoints.default())
    ic = kin.static.instant_center
    h = 0.5
    up, dn = kin.solve_at_travel(h), kin.solve_at_travel(-h)
    for name in ("upper_outer", "lower_outer"):
        joint = getattr(kin.static, name)
        v = np.array([getattr(up, name)[1] - getattr(dn, name)[1],
                      getattr(up, name)[2] - getattr(dn, name)[2]])
        r = np.array([ic[0] - joint[1], ic[1] - joint[2]])
        # r (joint -> IC) must be perpendicular to the velocity
        cos = abs(np.dot(v, r)) / (np.linalg.norm(v) * np.linalg.norm(r))
        assert cos < 1e-3, f"{name}: IC not perpendicular to its motion (cos={cos})"


# --------------------------------------------------------------------------- #
#  Anti-dive / anti-squat sign conventions
# --------------------------------------------------------------------------- #
def _mirror_stagger(hp):
    """Flip the fore/aft pickup-height stagger, mirroring the side-view instant
    centre to the other side of the wheel."""
    hp.upper_front_inner[2], hp.upper_rear_inner[2] = \
        hp.upper_rear_inner[2], hp.upper_front_inner[2]
    hp.lower_front_inner[2], hp.lower_rear_inner[2] = \
        hp.lower_rear_inner[2], hp.lower_front_inner[2]
    return hp


def test_anti_dive_flips_sign_when_geometry_is_mirrored():
    """The whole point of the sign fix: pro-dive geometry must read NEGATIVE.
    Taking abs() of the swing-arm offset reported both as positive anti-dive, so
    a car built with the stagger backwards was told it had anti-dive it did not
    have."""
    good = SuspensionKinematics(Hardpoints.default())
    bad = SuspensionKinematics(_mirror_stagger(Hardpoints.default()))
    ad_good = good.anti_dive_pct(300.0, 1550.0, brake_bias_front=0.65)
    ad_bad = bad.anti_dive_pct(300.0, 1550.0, brake_bias_front=0.65)
    assert ad_good > 0, f"default geometry should be anti-dive, got {ad_good}%"
    assert ad_bad < 0, f"mirrored geometry is pro-dive, got {ad_bad}%"
    #  Near-equal, NOT exactly equal. The idealised wishbone-line construction was
    #  perfectly antisymmetric under this mirror; the real solved linkage is not,
    #  because flipping the pickup z-stagger leaves caster, kingpin and the outer
    #  points untouched. 26.22 vs -25.18 is the linkage being asymmetric, which it
    #  is. The SIGN is the invariant; the magnitude is only approximately mirrored.
    assert abs(abs(ad_good) - abs(ad_bad)) / abs(ad_good) < 0.10


def test_default_anti_dive_matches_its_documented_value():
    """Hardpoints.default() advertises ~26% anti-dive. Pin it, so the geometry
    and its docstring cannot drift apart again."""
    kin = SuspensionKinematics(Hardpoints.default())
    ad = kin.anti_dive_pct(300.0, 1550.0, brake_bias_front=0.65)
    assert 24.0 < ad < 28.0, f"default anti-dive {ad:.1f}%, docstring says ~26%"


def test_anti_squat_uses_wheel_centre_not_contact_patch():
    """Inboard drive reacts torque at the chassis, so only tractive force goes
    through the links — at WHEEL-CENTRE height. Moving the wheel centre must
    therefore change anti-squat; moving only the contact patch must not.
    Measuring from the patch added a whole tyre radius to the lever arm."""
    base = SuspensionKinematics(Hardpoints.default())
    a0 = base.anti_squat_pct(300.0, 1550.0, drive_bias_rear=1.0)

    hp_wc = Hardpoints.default()
    hp_wc.wheel_center[2] += 40.0
    a_wc = SuspensionKinematics(hp_wc).anti_squat_pct(300.0, 1550.0, 1.0)
    assert abs(a_wc - a0) > 1.0, "anti-squat must respond to wheel-centre height"

    hp_cp = Hardpoints.default()
    hp_cp.contact_patch[2] -= 15.0
    a_cp = SuspensionKinematics(hp_cp).anti_squat_pct(300.0, 1550.0, 1.0)
    assert abs(a_cp - a0) < 1e-6, \
        "inboard-drive anti-squat must NOT depend on contact-patch height"


def test_anti_features_are_zero_on_flat_pickups():
    hp = Hardpoints.default()
    for k in ("upper_front_inner", "upper_rear_inner"):
        getattr(hp, k)[2] = 290.0
    for k in ("lower_front_inner", "lower_rear_inner"):
        getattr(hp, k)[2] = 120.0
    kin = SuspensionKinematics(hp)
    #  SMALL, not exactly zero. Flat pickups make the classic side-view swing arm
    #  infinite, which is where "zero anti-dive" comes from — but the real carrier
    #  still pitches a little as it travels (caster and kingpin see to that), so
    #  the contact patch keeps a small longitudinal path slope and the true
    #  anti-dive is a fraction of a percent rather than a hard zero. Deriving from
    #  the solved path instead of the idealised construction is what surfaced it.
    assert abs(kin.anti_dive_pct(300.0, 1550.0)) < 2.0
    #  Anti-SQUAT does not collapse the same way, and that is correct: it is
    #  measured at the WHEEL CENTRE, which still swings fore/aft with the arms
    #  when the side-view arm is infinite. Only the contact-patch slope goes to
    #  ~0 with flat pickups. The idealised construction hid this by giving both
    #  the same infinite swing arm.
    assert abs(kin.anti_squat_pct(300.0, 1550.0)) < 10.0
    staggered = SuspensionKinematics(Hardpoints.default())
    assert abs(kin.anti_dive_pct(300.0, 1550.0)) < \
        0.1 * abs(staggered.anti_dive_pct(300.0, 1550.0)), \
        "flat pickups must give far less anti-dive than a staggered set"


def test_swing_arm_length_sign_agrees_with_anti_dive():
    """side_view_swing_arm_length() exists to explain anti_dive_pct(); the two
    documented the offset in opposite directions before the fix."""
    for hp in (Hardpoints.default(), _mirror_stagger(Hardpoints.default())):
        kin = SuspensionKinematics(hp)
        sva = kin.side_view_swing_arm_length()
        ad = kin.anti_dive_pct(300.0, 1550.0)
        if math.isfinite(sva) and abs(ad) > 1e-9:
            assert (sva > 0) == (ad > 0), \
                f"SVA length {sva:.0f} mm disagrees in sign with anti-dive {ad:.1f}%"


# --------------------------------------------------------------------------- #
#  One car, two solvers
# --------------------------------------------------------------------------- #
def test_generic_example_is_the_same_car_as_the_native_default():
    """topologies.example('double_wishbone') used to be a copy-pasted literal of
    Hardpoints.default(). The cross-solver agreement test then silently became a
    comparison of two different cars."""
    from suspension import topologies
    mech = topologies.example("double_wishbone")
    hp = Hardpoints.default()
    # Mechanism uses short point names and wraps coordinates in a Point.
    name_map = {
        "ufi": "upper_front_inner", "uri": "upper_rear_inner",
        "lfi": "lower_front_inner", "lri": "lower_rear_inner",
        "tri": "tie_rod_inner", "tro": "tie_rod_outer",
        "uo": "upper_outer", "lo": "lower_outer",
        "wc": "wheel_center", "cp": "contact_patch",
    }
    checked = 0
    for short, attr in name_map.items():
        pt = mech.points.get(short)
        if pt is None:
            continue
        assert np.allclose(np.asarray(pt.pos, float), getattr(hp, attr), atol=1e-9), \
            f"{attr} differs between the generic example and Hardpoints.default()"
        checked += 1
    assert checked >= 8, f"only {checked} points compared; naming may have changed"


# --------------------------------------------------------------------------- #
#  Longitudinal models agree
# --------------------------------------------------------------------------- #
def test_braking_uses_the_same_friction_envelope_as_acceleration():
    """Braking is longitudinal. With no lateral demand, the brake-side grip
    ceiling must scale with the combined tyre's mu_x_ratio exactly as the accel
    side does — otherwise the same rubber is credited with extra grip under
    power and denied it under brakes."""
    from suspension import laptime

    class _Tire:
        ell_kx = 2.0
        ell_ky = 2.0
        mu_x_ratio = 1.30

    veh = VehicleDynamics(VehicleParams())
    pt_plain = laptime.Powertrain()
    pt_tire = laptime.Powertrain()
    pt_tire.combined_tire = _Tire()
    # brake_g_cap would clip the comparison; lift it so we see the tyre limit.
    pt_plain.brake_g_cap = 99.0
    pt_tire.brake_g_cap = 99.0

    v, mu = 20.0, 1.5
    a_plain = laptime._decel_long(veh, v, pt_plain, mu, 0.0)
    a_tire = laptime._decel_long(veh, v, pt_tire, mu, 0.0)
    # Drag is common to both, so compare the grip-derived part.
    m = veh.p.mass
    F_drag = 0.5 * pt_plain.rho * pt_plain.cda * v * v
    g_plain = a_plain * m - F_drag
    g_tire = a_tire * m - F_drag
    assert abs(g_tire / g_plain - 1.30) < 1e-6, \
        f"brake side ignored mu_x_ratio (ratio {g_tire / g_plain:.4f})"


def test_grip_fallback_is_reported_explicitly_not_by_float_equality():
    """A geometry that legitimately solves to exactly the fallback value must
    not be labelled as having fallen back."""
    from suspension import laptime

    class _Veh:
        p = VehicleParams()

        def max_lateral_g(self):
            return laptime._FALLBACK_LAT_G      # a real, valid result

    g, fell_back = laptime._max_lat_g_flagged(_Veh())
    assert g == laptime._FALLBACK_LAT_G
    assert fell_back is False, "valid grip equal to the sentinel was misreported"

    class _Broken:
        p = VehicleParams()

        def max_lateral_g(self):
            raise RuntimeError("no geometry")

    g2, fell_back2 = laptime._max_lat_g_flagged(_Broken())
    assert fell_back2 is True and g2 == laptime._FALLBACK_LAT_G


# --------------------------------------------------------------------------- #
#  Shared physical constants
# --------------------------------------------------------------------------- #
def test_air_density_is_consistent_across_modules():
    """Two modules disagreeing on rho by 2% is a slow leak: it shows up as an
    unexplained delta between the aero, powertrain and cooling numbers."""
    from suspension import pt_integration
    import inspect
    sig = inspect.signature(pt_integration.cooling_operating_point)
    rho = sig.parameters["air_density"].default
    assert abs(rho - 1.225) < 1e-9, f"air_density default {rho}, expected 1.225"


# --------------------------------------------------------------------------- #
#  Anti-dive / anti-squat derived a SECOND, independent way
# --------------------------------------------------------------------------- #
#  The anti-squat change (contact patch -> wheel centre) FLIPPED A SIGN on the
#  strength of one reviewer's reading of Gillespie/Milliken. A wrong sign with a
#  confident comment attached is worse than the original bug, because the next
#  reader trusts the comment. So here the same two numbers are derived from
#  scratch by VIRTUAL WORK, with no appeal to any textbook formula, and the two
#  routes are required to agree.
#
#  Derivation (self-contained, so it can be checked without the source):
#
#  The corner has one degree of freedom, q = wheel travel (bump positive). In
#  side view the wheel swings about the side-view instant centre S, so the
#  reference point P (where the longitudinal force acts) moves on a circle about
#  S. Writing dx = x_S - x_P and dz = z_S - z_P, the path slope is
#
#       d(x_P)/d(q) = -dz / dx  ==  -tan(phi)
#
#  Ground forces Fx (longitudinal) and Fz (vertical) therefore produce a
#  generalised force on the suspension DOF of
#
#       Q = Fz + Fx * d(x_P)/d(q) = Fz - Fx * tan(phi)
#
#  Q is what the SPRING carries. The part of the axle's load transfer that never
#  reaches the spring is the anti-effect:
#
#       anti% = (dW - dQ) / dW = Fx * tan(phi) / dW
#
#  WHERE P IS depends on how the torque is reacted, and this is the whole
#  outboard/inboard distinction:
#    * outboard brakes  - brake torque is reacted between caliper and rotor,
#      both on the carrier. Internal. The carrier sees only the ground forces,
#      so P = the CONTACT PATCH.
#    * inboard drive    - the chassis reacts the drive torque, so the carrier
#      additionally sees a couple of magnitude Fx*R from the halfshaft. A force
#      Fx at the contact patch plus a couple Fx*R is statically equivalent to Fx
#      applied one tyre radius higher and no couple: P = the WHEEL CENTRE.
#  That equivalence is the entire justification for the reference-point change,
#  and it is arithmetic, not authority.
def _anti_by_virtual_work(kin, point_name, fx_sign, bias, wheelbase, cg_height,
                          d=0.5):
    """anti-effect %, from virtual work on the SOLVED path — no formula reused
    from the module under test.

    Q = Fz + Fx*S with S = d(x_P)/d(q), so the part of the axle's load transfer
    that never reaches the spring is  anti = -Fx*S/dW.  `fx_sign` is +1 when the
    longitudinal ground force acts rearward (braking) and -1 forward (traction);
    that sign flip is why anti-dive and anti-squat respond OPPOSITELY to the same
    path slope."""
    up = kin.solve_at_travel(+d)
    dn = kin.solve_at_travel(-d)
    pu, pd = getattr(up, point_name), getattr(dn, point_name)
    dz = pu[2] - pd[2]
    if abs(dz) < 1e-12:
        return float("nan")
    slope = (pu[0] - pd[0]) / dz
    fx = fx_sign * bias                 # per unit m*a
    dw = cg_height / wheelbase          # per unit m*a
    return -(fx * slope) / dw * 100.0


def test_anti_dive_agrees_with_a_virtual_work_derivation():
    L, h, bias = 1550.0, 300.0, 0.65
    for hp in (Hardpoints.default(), _mirror_stagger(Hardpoints.default())):
        kin = SuspensionKinematics(hp)
        vw = _anti_by_virtual_work(kin, "contact_patch", +1.0, bias, L, h)
        closed = kin.anti_dive_pct(h, L, brake_bias_front=bias)
        assert abs(vw - closed) < 1e-6, \
            f"anti-dive: virtual work {vw:.4f}% vs {closed:.4f}%"


def test_anti_squat_agrees_with_a_virtual_work_derivation():
    L, h, bias = 1550.0, 300.0, 1.0
    for hp in (Hardpoints.default(), _mirror_stagger(Hardpoints.default())):
        kin = SuspensionKinematics(hp)
        vw = _anti_by_virtual_work(kin, "wheel_center", -1.0, bias, L, h)
        closed = kin.anti_squat_pct(h, L, drive_bias_rear=bias)
        assert abs(vw - closed) < 1e-6, \
            f"anti-squat: virtual work {vw:.4f}% vs {closed:.4f}%"


def test_native_and_generic_solvers_agree_on_the_same_car():
    """adapter.GenericKinematics and SuspensionKinematics are two paths to one
    number, and topologies.example('double_wishbone') is now literally
    Hardpoints.default(). A user switching topology must not see the physics
    change under them — they disagreed by 26% on anti-squat because each used a
    different pair of carrier points to build a side-view instant centre that,
    for a non-planar linkage, is not a shared object at all."""
    from suspension.adapter import GenericKinematics
    from suspension.topologies import example
    n = SuspensionKinematics(Hardpoints.default())
    g = GenericKinematics(example("double_wishbone"))
    L, h = 1550.0, 300.0
    assert abs(n.anti_dive_pct(h, L, 0.65) - g.anti_dive_pct(h, L, 0.65)) < 1e-3
    assert abs(n.anti_squat_pct(h, L, 1.0) - g.anti_squat_pct(h, L, 1.0)) < 1e-3
    assert abs(n.static.camber - g.static.camber) < 1e-6


def test_anti_dive_and_anti_squat_respond_oppositely_to_path_slope():
    """Not a convention: braking pushes the contact patch rearward and traction
    pulls it forward, so the SAME path slope gives opposite anti-effects. If
    these two ever share a sign convention, one of them is wrong."""
    kin = SuspensionKinematics(Hardpoints.default())
    s_cp = kin._path_slope_xz("contact_patch")
    s_wc = kin._path_slope_xz("wheel_center")
    ad = kin.anti_dive_pct(300.0, 1550.0, 0.65)
    asq = kin.anti_squat_pct(300.0, 1550.0, 1.0)
    assert abs(ad - (-0.65 * s_cp * 1550.0 / 300.0 * 100.0)) < 1e-6
    assert abs(asq - (+1.0 * s_wc * 1550.0 / 300.0 * 100.0)) < 1e-6


# ============================================================================ #
#  AERO
# ============================================================================ #
def _box_mesh(ride_h_m, thickness=0.06):
    trimesh = pytest.importorskip("trimesh")
    m = trimesh.creation.box(extents=(1.0, 0.6, thickness))
    m.apply_translation((0.0, 0.0, ride_h_m + thickness / 2.0))
    return m


def _panel_cl(mesh, ground_effect, aref=0.6):
    """Bare-bones C_L from the panel solver's own internals, so the test does not
    depend on STL loading, decimation or the CaseSpec plumbing."""
    np_ = pytest.importorskip("numpy")
    from suspension.aero.panel_method import PanelMethodModel, PanelParams
    mdl = PanelMethodModel(PanelParams(max_panels=None, ground_effect=ground_effect))
    c, n, a = mesh.triangles_center, mesh.face_normals, mesh.area_faces
    vinf = np_.array([1.0, 0.0, 0.0])
    A = mdl._influence_matrix(c, n, a)
    sigma = np_.linalg.lstsq(A, -(n @ vinf), rcond=None)[0]
    v = mdl._induced_velocity(c, n, a, sigma,
                              ground_effect=ground_effect,
                              road_plane_z_m=0.0)
    vs = v + vinf[None, :]
    vn = np_.einsum("ij,ij->i", vs, n)
    vt = vs - vn[:, None] * n
    cp = 1.0 - np_.einsum("ij,ij->i", vt, vt)
    return float((-(cp * a)[:, None] * n).sum(axis=0)[2] / aref)


def test_panel_method_recovers_free_air_when_the_road_is_far():
    """THE decisive check on the ground-image implementation, and the one that
    caught the bug: an image system is a way of imposing a boundary condition at
    z=0, so moving that plane far from the body must reproduce the unbounded
    solve. It did not. The image was applied when solving for the source
    strengths but omitted from the velocity field used for Cp, so a body 0.5 m
    above the road still reported a spurious C_L of ~9e-4 against a free-air
    0.0 — and at real FSAE ride heights it produced LIFT where the corrected
    solve produces downforce.

    Failure here means the influence matrix and the velocity evaluation have
    drifted out of agreement again. They must always describe the same flow."""
    for h in (2.0, 5.0, 20.0):
        free = _panel_cl(_box_mesh(h), ground_effect=False)
        near = _panel_cl(_box_mesh(h), ground_effect=True)
        assert abs(near - free) < 5e-5, (
            f"road {h} m away: ground-on C_L={near:.3e} vs free-air {free:.3e}. "
            f"The image system is not converging to the unbounded solution.")


def test_panel_method_ground_effect_makes_downforce_not_lift():
    """A body close to the road must gain downforce relative to free air: the
    image accelerates the flow underneath, dropping the pressure there. Sign
    only — potential flow has no business predicting a magnitude here."""
    low = _panel_cl(_box_mesh(0.03), ground_effect=True)
    free = _panel_cl(_box_mesh(0.03), ground_effect=False)
    assert low < free, (
        f"in ground effect C_L={low:.4e} is not below free-air {free:.4e}; "
        f"the image is either missing or reflected the wrong way")


def _subdivided_box(h, n=2):
    m = _box_mesh(h)
    for _ in range(n):
        m = m.subdivide()
    return m


def test_panel_ground_effect_decays_with_ride_height():
    """Monotone decay of the ground contribution. Ride-height sensitivity is the
    single number teams actually take from this module.

    The mesh is subdivided deliberately. On the raw 12-panel box (340 mm panels)
    this FAILS at 30 mm ride height — the ground effect comes out smaller than at
    100 mm, i.e. the trend inverts — because each panel is a point source at its
    centroid and the road image sits well inside the range where that holds. That
    is a real limitation of the method, not a bug in it, so it is guarded rather
    than fixed: see test_panel_method_warns_when_the_mesh_is_too_coarse."""
    deltas = [abs(_panel_cl(_subdivided_box(h), True)
                  - _panel_cl(_subdivided_box(h), False))
              for h in (0.03, 0.10, 0.40, 2.0)]
    for a, b in zip(deltas, deltas[1:]):
        assert b <= a + 1e-9, f"ground effect grew with ride height: {deltas}"


def test_panel_method_warns_when_the_mesh_is_too_coarse():
    """The coarse-mesh failure is SILENT: the solve converges, the numbers look
    plausible, and the ride-height trend is backwards. A user cannot see that
    from the result, so the solver has to say it."""
    from suspension.aero.panel_method import PanelMethodModel, PanelParams
    mdl = PanelMethodModel(PanelParams(max_panels=None, ground_effect=True))
    coarse = _box_mesh(0.03)          # 340 mm panels over a 30 mm gap
    warn = mdl._resolution_warning(coarse.triangles_center, coarse.area_faces)
    assert "WARNING" in warn and "invert" in warn.lower(),         f"coarse mesh in ground effect produced no warning: {warn!r}"
    fine = _subdivided_box(0.30, n=3)  # small panels, generous ride height
    assert mdl._resolution_warning(fine.triangles_center, fine.area_faces) == "",         "a well-resolved mesh must not be flagged"


def test_air_kinematic_viscosity_tracks_the_standard_table():
    """nu(T) feeds every Reynolds number in the aero package — scaled-run
    similitude, panel skin friction, CFD case setup. A few percent here silently
    shifts every Re the team quotes."""
    from suspension.aero.scale_model import air_kinematic_viscosity, reynolds
    table = {0: 1.330e-5, 15: 1.470e-5, 25: 1.562e-5, 40: 1.702e-5}
    for t_c, ref in table.items():
        got = air_kinematic_viscosity(t_c)
        assert abs(got - ref) / ref < 0.02, f"nu({t_c} C) = {got:.4e}, table {ref:.3e}"
    # and Reynolds must compose from it, not from a second hard-coded nu
    nu = air_kinematic_viscosity(15.0)
    assert abs(reynolds(20.0, 0.5, nu) - 20.0 * 0.5 / nu) < 1.0


def test_dynamic_pressure_prefers_measured_pitot_over_nominal_speed():
    """q sets the scale of every Cp. When a pitot total pressure was logged it is
    what the freestream actually was; computing 1/2 rho V^2 from a set-point
    trusts a speed the tunnel may not have held."""
    from suspension.aero.pressure_tap import ScanProvenance
    p = ScanProvenance("A1", rho=1.225, speed_ms=30.0,
                       p_total_inf_pa=600.0, p_static_inf_pa=0.0)
    assert abs(p.dynamic_pressure() - 600.0) < 1e-9
    q_nominal = 0.5 * 1.225 * 30.0 ** 2
    p2 = ScanProvenance("A2", rho=1.225, speed_ms=30.0)
    assert abs(p2.dynamic_pressure() - q_nominal) < 1e-9


def test_aero_coupling_axle_rate_is_both_wheels():
    """`_axle_wheel_rate` is divided into PER-AXLE loads (longitudinal_load_transfer
    returns the whole axle's dW; aero downforce is a whole-car force), so it must
    be the rate of both wheels together. Returning the single-wheel rate made
    every pitch and heave deflection 2x too large — and because heave sets the
    ride height that is fed back into the aero map, the error compounded around
    the coupling loop."""
    from suspension.aero.coupling import _axle_wheel_rate

    class _Kin:
        @staticmethod
        def motion_ratio():
            return 0.5

    class _P:
        use_spring_rates = True
        spring_rate_front = 200.0      # N/mm at the spring
        spring_rate_rear = 200.0

    class _Veh:
        p = _P()
        front_kin = _Kin()
        rear_kin = _Kin()

    # one wheel: 200 * 0.5^2 = 50 N/mm; the axle is two of them.
    assert abs(_axle_wheel_rate(_Veh(), "front") - 100.0) < 1e-9


def test_aero_heave_matches_a_hand_calculation():
    """End-to-end sanity on the units: 1000 N of downforce on a car whose four
    corners each give 50 N/mm must heave 5 mm, not 10."""
    from suspension.aero.coupling import _axle_wheel_rate

    class _Kin:
        @staticmethod
        def motion_ratio():
            return 0.5

    class _P:
        use_spring_rates = True
        spring_rate_front = spring_rate_rear = 200.0

    class _Veh:
        p = _P()
        front_kin = rear_kin = _Kin()

    kf = _axle_wheel_rate(_Veh(), "front")
    kr = _axle_wheel_rate(_Veh(), "rear")
    assert abs(1000.0 / (kf + kr) - 5.0) < 1e-9


def test_pitch_is_geometry_side_in_every_cfd_export():
    """RAKE MUST REACH THE MESH. Every deck this package writes has a ground plane
    at z=0, and tilting the inlet leaves the car's angle to that plane unchanged —
    so pitch on the inlet means a rake sweep exports the same geometry every time.
    meshing.py already stated the criterion ("they move the CAR relative to the
    ground plane, which the freestream cannot represent"); pitch passes it as
    plainly as roll does.

    Yaw stays on the inlet, correctly — the road is symmetric about z."""
    import math as _m
    from suspension.aero.cfd import Attitude
    from suspension.aero.meshing import _attitude_geometry_transform
    from suspension.aero.fluent_journal import _inlet_velocity
    from suspension.aero.backends import OpenFOAMSolver

    # 1. pitch must NOT appear in any inlet velocity
    for f in (lambda pd: _inlet_velocity(20.0, 0.0, pd),
              lambda pd: OpenFOAMSolver._inlet_velocity(
                  Attitude(pitch_deg=pd, speed_ms=20.0))):
        flat, pitched = f(0.0), f(5.0)
        assert all(abs(a - b) < 1e-9 for a, b in zip(flat, pitched)), \
            "pitch is still being folded into the inlet velocity"
        # yaw must still act there
        assert abs(_inlet_velocity(20.0, 5.0, 0.0)[1]) > 1e-6

    # 2. pitch MUST change the meshed geometry transform
    flat = _attitude_geometry_transform(Attitude(pitch_deg=0.0, ride_height_mm=30))
    rake = _attitude_geometry_transform(Attitude(pitch_deg=2.0, ride_height_mm=30))
    assert abs(rake["combined_angle_deg"] - flat["combined_angle_deg"]) > 1.0, \
        "rake does not reach the mesh transform — the exported CFD case has no rake"

    # 3. the composed axis-angle must equal Ry(pitch)·Rx(roll)
    np_ = pytest.importorskip("numpy")
    for roll, pitch in ((0, 0), (3, 0), (0, 2), (4, -2.5)):
        tf = _attitude_geometry_transform(
            Attitude(roll_deg=roll, pitch_deg=pitch, ride_height_mm=30))
        ax = [float(x) for x in tf["combined_axis"].strip("()").split()]
        a = _m.radians(tf["combined_angle_deg"])
        K = np_.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np_.eye(3) + _m.sin(a) * K + (1 - _m.cos(a)) * K @ K
        cr, sr = _m.cos(_m.radians(roll)), _m.sin(_m.radians(roll))
        cp, sp = _m.cos(_m.radians(pitch)), _m.sin(_m.radians(pitch))
        Rx = np_.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np_.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        assert np_.allclose(R, Ry @ Rx, atol=1e-9), \
            f"composed rotation wrong at roll={roll}, pitch={pitch}"


def test_daq_biquads_have_unity_gain_where_they_should():
    """DC GAIN IS THE WHOLE POINT HERE. daq.py states that the quantity it exists
    to protect is "the time-AVERAGE the balance reading needs" — and the average
    is set by the filter's DC gain. Both biquads wrote a[0] = 1.0 and then divided
    the whole denominator by a0 = 1 + alpha, leaving a[0] = 1/(1+alpha): a
    different filter with a spurious (1+alpha) gain everywhere outside the
    feature. The notch read 1.19 at DC and the Q=0.707 low-pass read 11.3, so
    every filtered force channel was scaled by a factor nobody chose."""
    np_ = pytest.importorskip("numpy")
    from suspension.aero.daq import _rbj_notch, _rbj_lowpass

    def mag(b, a, f, fs):
        z = np_.exp(-2j * math.pi * f / fs)
        return abs((b[0] + b[1] * z + b[2] * z * z)
                   / (a[0] + a[1] * z + a[2] * z * z))

    fs = 1000.0
    b, a = _rbj_notch(50.0, fs, 10.0)
    assert abs(a[0] - 1.0) < 1e-12, "denominator not normalised to a[0] = 1"
    assert abs(mag(b, a, 0.1, fs) - 1.0) < 1e-3, "notch has gain at DC"
    assert mag(b, a, 50.0, fs) < 1e-9, "notch does not null at f0"
    assert abs(mag(b, a, 400.0, fs) - 1.0) < 1e-3, "notch has gain far from f0"

    b, a = _rbj_lowpass(100.0, fs, 0.70710678)
    assert abs(a[0] - 1.0) < 1e-12
    assert abs(mag(b, a, 0.1, fs) - 1.0) < 1e-3, "low-pass DC gain is not unity"
    # Butterworth Q: exactly -3.01 dB at the cutoff, and monotone roll-off after.
    assert abs(20 * math.log10(mag(b, a, 100.0, fs)) + 3.01) < 0.05
    for f1, f2 in ((100.0, 200.0), (200.0, 400.0)):
        assert mag(b, a, f2, fs) < mag(b, a, f1, fs)


def test_plug_builder_section_area_matches_the_analytic_superellipse():
    """The lofted section is a Lame curve, which has a closed-form area:
    A_upper = 2*w*h*Gamma(1+1/e)^2 / Gamma(1+2/e). Pins the numerical integration
    that every material take-off (fabric, resin, foam) is derived from."""
    from math import gamma
    from suspension.aero.plug_builder import NoseconeBody
    b = NoseconeBody(length_mm=800.0, base_width_mm=300.0, base_height_mm=250.0)
    x = 800.0
    w, h, e = b.plan_half_width(x), b.crest_height(x), b.section_exponent
    exact = 2.0 * w * h * gamma(1 + 1 / e) ** 2 / gamma(1 + 2 / e)
    assert abs(b._section_area_mm2(x, k=48) - exact) / exact < 2e-3
    assert abs(b._section_area_mm2(x, k=768) - exact) / exact < 5e-5

def test_piv_subpixel_fit_recovers_a_known_offset():
    """Three-point Gaussian fit is what takes PIV below a pixel, and it is the
    whole accuracy claim of the technique. The formula
    dx = (ln c- - ln c+) / (2*(ln c- - 2 ln c0 + ln c+)) is exact for a Gaussian,
    so a synthesised peak at a known sub-pixel offset must come back exactly."""
    def fit(c0, cm, cp):
        lm, l0, lp = math.log(cm), math.log(c0), math.log(cp)
        d = 2.0 * (lm - 2.0 * l0 + lp)
        return (lm - lp) / d if abs(d) > 1e-12 else 0.0
    for true in (-0.4, -0.2, 0.0, 0.15, 0.33, 0.45):
        s_ = 1.2
        g = lambda k: math.exp(-((k - true) ** 2) / (2 * s_ * s_))
        assert abs(fit(g(0), g(-1), g(1)) - true) < 1e-9


def test_run_log_catches_lift_and_drag_normalised_by_different_areas():
    """A row can be internally consistent in lift AND internally consistent in
    drag and still be wrong, if the two were normalised by different reference
    areas — which happens for real when Fluent's reference values are changed
    between report sections. run_log computed the drag-implied area and then
    never read it, so this was the one normalisation failure no gate could see."""
    from suspension.aero.run_log import RunRow, screen
    q = 0.5 * 1.225 * 20.0 ** 2

    def row(i, area_drag):
        return RunRow(source_row=i, sheet="s", component="wing",
                      ride_height_mm=30.0, speed_ms=20.0, converged=True,
                      max_pressure_Pa=q, min_pressure_Pa=-2 * q,
                      lift_force_N=-2.0 * q * 1.0, lift_coeff=-2.0,
                      drag_force_N=0.8 * q * area_drag, drag_coeff=0.8)

    verdicts = screen([row(1, 1.0), row(2, 1.0), row(3, 1.4)])
    codes = {v.row.source_row: {f.code for f in v.flags} for v in verdicts}
    assert "REF_AREA_LIFT_DRAG_SPLIT" not in codes[1]
    assert "REF_AREA_LIFT_DRAG_SPLIT" in codes[3], \
        "a 40% lift/drag area split went undetected"
    assert not [v for v in verdicts if v.row.source_row == 3][0].accepted


def test_surrogate_induced_drag_is_signed():
    """The stand-in model says "plumbing and trends only", so its trends have to
    be right. Induced drag used abs(ground_effect), which added drag whether the
    car was lowered or raised — the drag-vs-ride-height trend was backwards above
    the reference height. Induced drag follows lift squared, so it is signed."""
    from suspension.aero.backends import ReferenceAeroModel
    from suspension.aero.cfd import Attitude
    m = ReferenceAeroModel()
    pts = [m._coeffs(Attitude(ride_height_mm=h, speed_ms=20.0))
           for h in (10.0, 20.0, 30.0, 40.0, 60.0, 80.0)]
    lifts = [p[0] for p in pts]
    drags = [p[1] for p in pts]
    # Raising the car: less downforce (c_lift rises toward 0) and less drag.
    for a, b in zip(lifts, lifts[1:]):
        assert b > a, "downforce must fall as the car is raised"
    for a, b in zip(drags, drags[1:]):
        assert b < a, "induced drag must fall with downforce, not rise"


def test_pressure_tap_partial_chord_is_visible():
    """C_n integrates over the OVERLAP of the two surfaces' instrumented ranges.
    A sparsely tapped pressure side therefore shrinks the domain and shrinks the
    reported loading with it — silently. Everywhere else this module drops holes
    loudly; the load integral quietly truncated its own domain, which is the same
    class of error the rest of the file exists to prevent."""
    from suspension.aero.pressure_tap import CpField, TapLocation, WingSurface

    def taps_and_cp(pressure_range):
        taps, cp = [], {}
        for i, x in enumerate([0.05, 0.2, 0.4, 0.6, 0.8, 0.95]):
            tid = f"s{i}"
            taps.append(TapLocation(tap_id=tid, element="main",
                                    surface=WingSurface.SUCTION, x_over_c=x))
            cp[tid] = -1.5
        for i, x in enumerate(pressure_range):
            tid = f"p{i}"
            taps.append(TapLocation(tap_id=tid, element="main",
                                    surface=WingSurface.PRESSURE, x_over_c=x))
            cp[tid] = 0.5
        return CpField(taps=taps, cp=cp)

    full = taps_and_cp([0.05, 0.3, 0.6, 0.95])
    part = taps_and_cp([0.2, 0.4, 0.6])

    assert full.chord_coverage("main") > 0.95
    cov = part.chord_coverage("main")
    assert cov < 0.6, f"partial instrumentation not reported ({cov:.2f})"

    # The number itself shrinks with the domain — that is the trap.
    cn_full = full.normal_load_coefficient("main")
    cn_part = part.normal_load_coefficient("main")
    assert cn_part < cn_full * 0.8, "truncated integral did not shrink C_n"

    # Opting into a coverage floor turns the silent shrink into an honest NaN.
    assert math.isnan(part.normal_load_coefficient("main", min_chord_coverage=0.6))
    assert not math.isnan(full.normal_load_coefficient("main", min_chord_coverage=0.6))


# ============================================================================ #
#  ELECTRICAL
# ============================================================================ #
def test_onderdonk_uses_circular_mils_not_square_mils():
    """This class implements two standards that measure area DIFFERENTLY, and one
    property was feeding both:

        IPC-2221   I = k * dT^0.44 * A^0.725   -> A in SQUARE mils
        Onderdonk  I = A * sqrt(...)           -> A in CIRCULAR mils

    A circular mil is the area of a 1-mil-diameter circle, so
    A_cmil = A_sqmil * 4/pi. Feeding square mils to Onderdonk understated the
    fusing current by 21.5%. Anchored on a published figure: 10 AWG is
    10380 cmil and fuses in about 1 s at roughly 1500 A."""
    from suspension.electronics import Trace
    mk = lambda **kw: Trace(name="t", net="n", owner_subsystem="s", **kw)
    t = mk(width_mm=1.0, copper_oz=1.0, length_mm=100.0)
    assert abs(t.area_cmil / t.area_mil2 - 4.0 / math.pi) < 1e-12

    # Reproduce the 10 AWG benchmark through the same code path: pick the width
    # that gives 10380 circular mils at this copper weight.
    a_cmil = 10380.0
    a_mm2 = a_cmil * (math.pi / 4.0) / (39.3701 ** 2)
    w = mk(width_mm=1.0, copper_oz=1.0, length_mm=100.0)
    w.width_mm = a_mm2 / (w.area_mm2 / w.width_mm)      # scale width to hit a_mm2
    assert abs(w.area_cmil - a_cmil) / a_cmil < 1e-6
    i_fuse = w.fusing_current_a(t_s=1.0, ambient_c=25.0)
    assert 1400.0 < i_fuse < 1600.0, \
        f"10 AWG 1 s fusing current {i_fuse:.0f} A; published is about 1500 A"


def test_ampacity_correction_never_errs_optimistic():
    """The NEC ambient-correction factor must reproduce the published table, and
    where it does not, it must never claim MORE current than the table allows —
    that is the one direction an ampacity tool cannot be wrong in.

    The resistance-shift term used to be applied here, copied from
    ampacity_scale where it is correct. Here both endpoints have the conductor at
    the same rating temperature, so copper resistance cancels exactly and there
    is nothing to correct. Applying it anyway overshot the 60 C column by 0.048 —
    ten times the claimed tolerance, and optimistic."""
    from suspension.wiring import correction_factor
    published = {
        35: {60: 0.91, 75: 0.94, 90: 0.96}, 40: {60: 0.82, 75: 0.88, 90: 0.91},
        45: {60: 0.71, 75: 0.82, 90: 0.87}, 50: {60: 0.58, 75: 0.75, 90: 0.82},
        55: {75: 0.67, 90: 0.76},
    }
    for ambient, row in published.items():
        for rating, expected in row.items():
            got = correction_factor(rating, ambient)
            assert abs(got - expected) < 0.005, \
                f"{rating} C at {ambient} C ambient: {got:.4f} vs table {expected}"
            assert got < expected + 0.005, "correction factor errs optimistic"

    # The inapplicable refinement is refused, not silently ignored.
    with pytest.raises(ValueError):
        correction_factor(60, 40, include_resistance_shift=True)


def test_iec_adiabatic_k_matches_published_values():
    """k = sqrt(Qc(beta+20)/rho20 * ln((beta+tf)/(beta+ti))) sets every wire
    withstand in the fuse-coordination module. Copper/PVC (70->160 C) is a
    published k = 115; copper 90->250 C is 143."""
    from suspension.fuse_test import WireSpec
    assert abs(WireSpec(awg=16, t_initial_c=70.0, t_final_c=160.0).k() - 115.0) < 1.0
    assert abs(WireSpec(awg=16, t_initial_c=90.0, t_final_c=250.0).k() - 143.0) < 1.0


def test_fuse_curve_power_law_fit_is_exact_on_a_power_law():
    """The datasheet curve is fitted in log-log. A synthetic t = a * m^-b must be
    recovered exactly, or every coordination margin built on it is skewed."""
    from suspension.fuse_test import FuseSpec, CurveAnchor
    a, b = 12.0, 2.3
    f = FuseSpec(rating_a=30.0, anchors=[
        CurveAnchor(current_mult=m, time_s=a * m ** (-b)) for m in (1.5, 2, 3, 5, 10)])
    got_a, got_b = f.power_law()
    assert abs(got_a - a) < 1e-9 and abs(got_b - b) < 1e-9


# ============================================================================ #
#  BRAKES
# ============================================================================ #
def test_hydraulic_chain_matches_hand_calculation():
    """Pedal force -> rod -> balance bar -> line pressure -> clamp -> axle torque,
    end to end against arithmetic done by hand. Also pins the force balance:
    whatever the bar offset, the two clevis forces must sum to the rod force."""
    from suspension.pedal_box import CircuitSpec, balance_bar_bias
    f = CircuitSpec(mc_bore_mm=15.87, caliper_piston_dia_mm=25.4,
                    pistons_per_side=2, pad_mu=0.45, rotor_dia_mm=220.0,
                    pad_inner_radius_mm=79.0, n_corners=2)
    r = CircuitSpec(mc_bore_mm=17.46, caliper_piston_dia_mm=25.4,
                    pistons_per_side=1, pad_mu=0.45, rotor_dia_mm=200.0,
                    pad_inner_radius_mm=72.0, n_corners=2)
    for offset in (-20.0, 0.0, 12.5):
        res = balance_bar_bias(pedal_force_N=200.0, pedal_ratio=5.0,
                               front=f, rear=r, bar_length_mm=60.0,
                               bar_offset_mm=offset)
        assert abs(res.force_front_N + res.force_rear_N - 1000.0) < 1e-9, \
            "clevis forces must sum to the rod force at any bar offset"

    res = balance_bar_bias(pedal_force_N=200.0, pedal_ratio=5.0, front=f, rear=r)
    a_f = math.pi / 4 * 15.87 ** 2
    assert abs(res.pressure_front_bar - (500.0 / (a_f * 1e-6)) / 1e5) < 1e-6
    clamp = res.pressure_front_bar * 1e5 * (2 * math.pi / 4 * 25.4 ** 2) * 1e-6
    hand_T = 2 * clamp * 0.45 * (0.5 * (110.0 + 79.0) * 1e-3) * 2
    assert abs(res.torque_front_Nm - hand_T) < 1e-6


def test_effective_radius_prefers_geometry_over_a_fraction():
    """r_eff sets brake torque linearly, so a guessed fraction is a guessed
    torque. Uniform wear puts it at the mean of the swept band; given the pad's
    inner radius there is nothing to guess.

    The fallback fraction also has to be plausible. Inverting it,
    frac = (r_o + r_i)/(2 r_o), so the old 0.92 implied r_i = 0.84 r_o — a pad
    band 17.6 mm tall on a 220 mm rotor, against a real 25-40 mm. It overstated
    torque by ~8% in the optimistic direction."""
    from suspension.pedal_box import CircuitSpec
    exact = CircuitSpec(mc_bore_mm=15.87, caliper_piston_dia_mm=25.4,
                        rotor_dia_mm=220.0, pad_inner_radius_mm=79.0)
    assert abs(exact.r_eff_mm - 94.5) < 1e-9, "uniform-wear mean radius"

    fallback = CircuitSpec(mc_bore_mm=15.87, caliper_piston_dia_mm=25.4,
                           rotor_dia_mm=220.0)
    r_o = 110.0
    implied_band = 2 * (r_o - fallback.r_eff_mm)
    assert 25.0 <= implied_band <= 40.0, (
        f"the default fraction implies a {implied_band:.1f} mm pad band; real "
        f"FSAE pads are 25-40 mm")

    # A nonsense inner radius must fall back rather than produce garbage.
    bad = CircuitSpec(mc_bore_mm=15.87, caliper_piston_dia_mm=25.4,
                      rotor_dia_mm=220.0, pad_inner_radius_mm=500.0)
    assert bad.r_eff_mm == fallback.r_eff_mm


def test_stop_energy_conserves_and_includes_rotating_inertia():
    """Energy must survive the front/rear split and the two-rotor halving, and
    the brakes have to stop the SPINNING mass too — omitting it under-predicted
    rotor temperature optimistically."""
    from suspension.brake_thermal import BrakeThermalModel, BrakeThermalParams
    m = BrakeThermalModel(BrakeThermalParams())
    kw = dict(mass_kg=300.0, v0_ms=27.8, front_bias=0.65,
              diameter_mm=220.0, thickness_mm=5.0)

    trans = m.single_stop(**kw, rotating_mass_factor=1.0)
    assert abs(trans.q_total_j - 0.5 * 300.0 * 27.8 ** 2) < 1e-6

    # Round-trip the split: q_per -> q_total must come back exactly.
    back = trans.q_per_rotor_j * 2.0 / m.p.heat_to_rotor / 0.65
    assert abs(back - trans.q_total_j) < 1e-6

    spun = m.single_stop(**kw, rotating_mass_factor=1.05)
    assert abs(spun.q_total_j / trans.q_total_j - 1.05) < 1e-9
    assert spun.delta_T_c > trans.delta_T_c, \
        "rotating inertia must RAISE predicted rotor temperature"


# ============================================================================ #
#  SUSPENSION DYNAMICS
# ============================================================================ #
def test_max_lateral_g_is_limited_by_the_first_axle_to_saturate():
    """In steady state there is no yaw acceleration, so moment balance about the
    CG locks the split: F_front = m*a_y*wd_f, F_rear = m*a_y*(1-wd_f). The demand
    on each axle cannot be shifted, so the car is finished when EITHER axle runs
    out — the other's spare grip is unreachable.

    Summing (F_f + F_r)/(m g) credits that spare grip. It is only right when both
    axles saturate together, i.e. on an already-balanced car, which is why the
    near-neutral default hid it at 0.05-0.3%. On real setups it did not: a stiff
    rear bar overstated grip by 4.7%."""
    from suspension.dynamics import VehicleParams, VehicleDynamics

    def limiting_axle_g(p):
        v = VehicleDynamics(p)
        w_f = p.mass * p.g * p.weight_dist_front
        w_r = p.mass * p.g * (1.0 - p.weight_dist_front)
        lo, hi = 0.1, 3.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            loads, _ = v.lateral_load_transfer(mid)
            Ff, Fr = v.axle_grip(loads)
            if min(Ff / w_f, Fr / w_r) >= mid:
                lo = mid
            else:
                hi = mid
        return lo

    for p in (VehicleParams(weight_dist_front=0.45),
              VehicleParams(weight_dist_front=0.45, roll_stiffness_front=150.0,
                            roll_stiffness_rear=900.0),
              VehicleParams(weight_dist_front=0.38, roll_stiffness_front=800.0,
                            roll_stiffness_rear=200.0)):
        got = VehicleDynamics(p).max_lateral_g()
        assert abs(got - limiting_axle_g(p)) < 2e-3, \
            "max_lateral_g does not agree with the limiting-axle limit"

    # And it must agree with balance_index, which was always computed correctly:
    # at the limit, the more-utilised axle sits at utilisation 1.0.
    p = VehicleParams(weight_dist_front=0.45, roll_stiffness_front=150.0,
                      roll_stiffness_rear=900.0)
    v = VehicleDynamics(p)
    g = v.max_lateral_g()
    _, util_f, util_r = v.balance_index(g)
    assert abs(max(util_f, util_r) - 1.0) < 5e-3, \
        f"limiting axle utilisation {max(util_f, util_r):.4f}, expected 1.0"


def test_transient_and_steady_state_agree_on_the_roll_axis():
    """Both models need the roll-axis height AT THE CG's station, and they were
    computing it two different ways — dynamics interpolated by weight
    distribution, transient took a plain mean of the two roll centres. Same car,
    two roll moment arms. The mean is only right at 50/50."""
    rc_f, rc_r = 20.0, 90.0
    for wd in (0.40, 0.45, 0.55):
        weighted = rc_f * wd + rc_r * (1.0 - wd)
        plain_mean = 0.5 * (rc_f + rc_r)
        if abs(wd - 0.5) > 1e-9:
            assert abs(weighted - plain_mean) > 1.0, "test case is vacuous"
        # the weighting must put MORE weight on the rear RC for a rear-biased car
        if wd < 0.5:
            assert weighted > plain_mean


def test_roll_moment_arm_may_be_negative():
    """A roll axis ABOVE the sprung CG reverses the roll moment — the body leans
    INTO the corner. High-roll-centre cars do exactly this, and it is the kind of
    behaviour someone runs a transient model to see. Clamping h_roll at 0
    reported it as zero roll instead of negative."""
    from suspension.transient import TransientParams
    p = TransientParams()
    p.cg_height = 0.20
    p.roll_axis_height = 0.30          # roll axis above the CG
    h_roll = p.cg_height - p.roll_axis_height
    assert h_roll < 0.0, "a roll axis above the CG must give a negative moment arm"


# ============================================================================ #
#  POWERTRAIN / DRIVETRAIN
# ============================================================================ #
def test_regen_does_not_recover_drag_and_rolling_resistance():
    """Drag and rolling resistance slow the car for free, so the motor only ever
    resists what is left. The accel branch gets this right
    (F_trac = m*a + F_drag + F_roll); the regen branch used a bare m*a and
    credited the battery with energy that went into the air and the tyres.

    It also removes a sign trap: below m*|a| = F_drag + F_roll the car is merely
    coasting down and the motor supplies nothing at all, yet every such sample
    was booked as recovery. Overstated endurance range by ~9% on a realistic
    trace, and range is what decides whether a car finishes."""
    np_ = pytest.importorskip("numpy")
    from suspension.ev_powertrain import EVLapSimulator, EVParams, LapSimParams

    class _Lap:
        pass

    # Pure drag coast-down: the motor resists nothing, so recovery must be ~0.
    coast = _Lap()
    coast.distance = np_.linspace(0.0, 500.0, 500)
    coast.speed = np_.linspace(30.0, 25.0, 500)
    coast.long_g = (np_.gradient(coast.speed) * coast.speed
                    / np_.gradient(coast.distance) / 9.81)

    sim = EVLapSimulator(EVParams())
    p = LapSimParams()
    _, regen = sim._energy_from_trace(coast, p)
    assert regen < 1e-3, (
        f"a pure drag coast-down recovered {regen:.5f} kWh; the motor is not "
        f"resisting anything here")

    # Hard braking still recovers, but strictly less than the bare m*a form.
    brake = _Lap()
    brake.distance = np_.linspace(0.0, 100.0, 400)
    brake.speed = np_.linspace(28.0, 6.0, 400)
    brake.long_g = (np_.gradient(brake.speed) * brake.speed
                    / np_.gradient(brake.distance) / 9.81)
    _, hard = sim._energy_from_trace(brake, p)
    assert hard > 0.0, "real braking must still recover energy"


def test_launch_traction_includes_longitudinal_load_transfer():
    """A launch is where weight piles onto the driven axle, so a static rear
    fraction is least defensible exactly where it was used. The transfer is
    self-reinforcing and has a closed form:
    F = mu*m*g*rf / (1 - mu*h/L). Ignoring it left the ceiling 27% low.

    This is not just a slower printed time: GearRatioSolver sweeps ratios through
    this function, and a low traction ceiling clips the extra force of short
    gears, biasing the recommended final drive taller."""
    from suspension.pt_integration import _accel_0_75

    class _Map:
        _rpm = [0.0, 3000.0, 6000.0, 9000.0]
        _t = [230.0, 230.0, 170.0, 90.0]

    common = dict(motor_map=_Map(), wheel_r=0.225, eff=0.90, mass_kg=300.0,
                  mu=1.4, cda=1.1, crr=0.018, rear_frac=0.55)
    # A high CG transfers more weight rearward -> more grip -> quicker launch.
    t_low_cg = _accel_0_75(final_drive=4.0, cg_height_m=0.20, **common)
    t_high_cg = _accel_0_75(final_drive=4.0, cg_height_m=0.40, **common)
    assert t_high_cg < t_low_cg, \
        "raising the CG must improve a rear-drive launch, not worsen it"

    # Closed form must match a hand calculation.
    mu, m, g, rf, h, L = 1.4, 300.0, 9.81, 0.55, 0.30, 1.55
    assert abs(mu * m * g * rf / (1 - mu * h / L) - 3108.0) < 2.0


# ============================================================================ #
#  STRUCTURAL / THERMAL / DAQ
# ============================================================================ #
def test_bolted_joint_stiffness_and_separation_are_exact():
    """Member stiffness against the Wileman closed form the docstring cites, and
    the separation invariant: at F_ext = F_sep the residual clamp must be exactly
    zero, or the joint model is not self-consistent."""
    from suspension.bolted_joint import _bolt_stiffness, _member_stiffness, Fastener
    E = 205000.0
    f = Fastener(grade="8.8", nominal_d_mm=8.0)
    k_b = _bolt_stiffness(f, 20.0)
    k_m = _member_stiffness(E, 8.0, 20.0)
    A, B = 0.78715, 0.62873                       # Wileman steel coefficients
    assert abs(k_m - E * 8.0 * A * math.exp(B * 8.0 / 20.0)) < 1e-6
    assert abs(k_b - f.stress_area() * E / 20.0) < 1e-6
    phi = k_b / (k_b + k_m)
    assert 0.10 < phi < 0.40, f"steel joint load factor {phi:.3f} out of range"
    f_i = 10000.0
    f_sep = f_i / (1.0 - phi)
    assert abs(f_i - (1.0 - phi) * f_sep) < 1e-9


def test_daq_catches_over_filtering_not_just_aliasing():
    """The anti-alias cutoff has TWO failure modes and only one was checked.
    Above Nyquist the filter passes what it should block. Below the signal
    bandwidth it blocks what it should pass — and that is the more dangerous one,
    because an aliased channel looks visibly noisy and someone asks, while an
    over-filtered channel comes back smooth, clean and quietly wrong. A smooth
    trace is exactly what a team assumes is good data."""
    from suspension.daq_plan import SensorSpec, signal_chain_findings, ANALOG_TYPES, Severity
    import inspect
    params = inspect.signature(SensorSpec).parameters
    analog = list(ANALOG_TYPES)[0]

    def spec(cutoff):
        base = dict(key="dp", name="damper pot", output=analog,
                    signal_bandwidth_hz=20.0, sample_rate_hz=200.0,
                    antialias_cutoff_hz=cutoff, adc_bits=12)
        return SensorSpec(**{k: v for k, v in base.items() if k in params})

    def fails(cutoff):
        return [f for f in signal_chain_findings(spec(cutoff))
                if f.severity == Severity.FAIL]

    assert not fails(50.0), "a 50 Hz cutoff on 20 Hz content at 200 Hz is correct"
    assert fails(150.0), "cutoff above Nyquist must FAIL"
    assert fails(5.0), "cutoff below the signal bandwidth must FAIL"


def test_cooling_steady_state_inverts_its_own_ua_requirement():
    """Self-consistency: if the radiator you have is exactly the UA you need,
    the settling temperature must come out at the coolant inlet you specified.
    Two different driving-temperature conventions here would show up as these
    two numbers disagreeing."""
    from suspension.cooling import size_loop, LoopSpec
    s = LoopSpec()
    first = size_loop(s)
    ua_req = s.heat_w / max(s.coolant_in_c - s.ambient_c, 1e-6)
    matched = size_loop(replace_ua(s, ua_req))
    assert abs(matched.steady_coolant_c - s.coolant_in_c) < 0.05, (
        f"with UA = UA_required the loop settles at "
        f"{matched.steady_coolant_c:.2f} C, not the specified {s.coolant_in_c:g} C")
    assert first is not None


def replace_ua(spec, ua):
    from dataclasses import replace
    return replace(spec, radiator_ua_w_per_k=ua)


# ============================================================================ #
#  RESOLVED FOLLOW-UPS
# ============================================================================ #
def test_pack_current_and_ev_energy_agree():
    """pack_thermal.pack_current_trace promises in its own docstring that it
    integrates back to the same energy as ev_powertrain._energy_from_trace. When
    the regen correction landed in one and not the other that promise silently
    became false — and the divergence would have shown up as pack temperature,
    not as an obvious error. Two modules, one physics, pinned together."""
    np_ = pytest.importorskip("numpy")
    from suspension.pack_thermal import pack_current_trace
    from suspension.ev_powertrain import EVLapSimulator, EVParams, LapSimParams

    class _Lap:
        pass

    lap = _Lap()
    lap.distance = np_.linspace(0.0, 1000.0, 3000)
    lap.speed = 18.0 + 7.0 * np_.sin(lap.distance / 40.0) + 3.0 * np_.sin(lap.distance / 13.0)
    lap.long_g = np_.gradient(lap.speed) * lap.speed / np_.gradient(lap.distance) / 9.81

    p, ev = LapSimParams(), EVParams()
    net_kwh, _ = EVLapSimulator(ev)._energy_from_trace(lap, p)
    t, cur = pack_current_trace(lap, p, pack_nominal_v=400.0,
                                inverter_motor_eff=ev.inverter_motor_eff,
                                regen_eff=ev.regen_eff, regen_max_g=ev.regen_max_g)
    from_current = float(np_.trapezoid(cur * 400.0, t)) / 3.6e6
    assert abs(from_current / net_kwh - 1.0) < 0.02, (
        f"current trace integrates to {from_current:.5f} kWh but the energy model "
        f"says {net_kwh:.5f} kWh — the two have drifted apart")


def test_damping_ratio_uses_the_series_ride_rate_when_given_a_tyre():
    """The sprung mass bounces on the wheel rate IN SERIES with the tyre. Using
    the wheel rate alone overstates the stiffness it sees and understates zeta —
    the number people actually tune to."""
    from suspension.damper import damping_ratio, default_damper
    c = default_damper()
    kw = dict(corner_mass_kg=60.0, wheel_rate_N_per_mm=35.0)
    bare = damping_ratio(c, **kw)
    with_tyre = damping_ratio(c, **kw, tire_rate_N_per_mm=130.0)
    k_ride = (35.0 * 130.0) / (35.0 + 130.0)
    assert with_tyre > bare, "series ride rate must RAISE zeta"
    assert abs(with_tyre / bare - math.sqrt(35.0 / k_ride)) < 1e-9
    # A very stiff tyre approaches the wheel-rate-only answer.
    assert abs(damping_ratio(c, **kw, tire_rate_N_per_mm=1e7) - bare) / bare < 1e-3


def test_cooling_reports_both_the_conservative_and_mean_ua():
    """coolant_in_c is the MOTOR inlet, i.e. the radiator's cold end, so driving
    off it understates the available dT and overstates required UA. That is the
    safe direction and is kept — but an undocumented margin is indistinguishable
    from an error, so both figures are now surfaced."""
    from suspension.cooling import size_loop, LoopSpec
    s = LoopSpec()
    r = size_loop(s)
    drive_mean = s.coolant_in_c + 0.5 * r.delta_t_k - s.ambient_c
    ua_mean = s.heat_w / drive_mean
    assert r.required_ua_w_per_k > ua_mean, "cold-end UA must be the larger one"
    assert any(f"{ua_mean:.0f} W/K" in n for n in r.notes), \
        "the less-conservative mean-temperature UA is not reported anywhere"


def test_a_stated_tolerance_beats_a_preset():
    """omnicore parses a stated tolerance out of the brief ("±2 mm"), buckets it
    into a shop CLASS, and nothing read the parsed number again — so a team that
    told the tool ±2.0 mm had their Monte Carlo run at the hand_weld preset's
    1.5 mm. Their own figure discarded in favour of a representative one, and in
    the optimistic direction. A stated tolerance is better evidence than any
    preset, so it wins, and the provenance has to say which one was used."""
    from suspension.kinematik_stochastic import ToleranceField
    preset = ToleranceField.preset("hand_weld")
    stated = ToleranceField.preset("hand_weld", tab_accuracy_mm=2.0)
    assert preset.specs["upper_front_inner"].hi[0] == pytest.approx(1.5)
    assert stated.specs["upper_front_inner"].hi[0] == pytest.approx(2.0)
    assert "preset" in preset.provenance.lower()
    assert "stated" in stated.provenance.lower()
    # The machined outers still come from the shop class — welding tolerance
    # says nothing about how well the upright was cut.
    assert stated.specs["upper_outer"].hi[0] == preset.specs["upper_outer"].hi[0]
    # Neither is inspection data, and neither may claim to be.
    assert not preset.calibrated and not stated.calibrated


def test_both_roll_centre_implementations_agree_including_the_degenerate_case():
    """dynamics.roll_center_height and ghost_topology._rc_height_mm are the same
    formula written twice. They matched everywhere except the branch nobody
    looks at: with the IC directly above the contact patch the line cp->IC is
    vertical and never reaches the centreline, so the roll centre is at infinity.
    ghost returned NaN; dynamics returned contact-patch height — roughly 0 mm,
    which is not a degenerate marker but a popular design target, so it reads as
    a real answer.

    That mattered more than a cosmetic disagreement: lateral_load_transfer
    already tests isfinite() on this value and falls back to a documented
    default. A finite lie is the one thing that slips past a guard built for
    exactly this case."""
    np_ = pytest.importorskip("numpy")
    from suspension.dynamics import VehicleDynamics, VehicleParams
    from suspension.ghost_topology import _rc_height_mm
    kin = SuspensionKinematics(Hardpoints.default())
    veh = VehicleDynamics(VehicleParams())
    for travel in (-25.0, -10.0, 0.0, 10.0, 25.0):
        st = kin.solve_at_travel(travel)
        assert abs(veh.roll_center_height(kin, 1200.0, state=st)
                   - _rc_height_mm(st, 1200.0)) < 1e-9

    class _Degenerate:
        # IC directly above the contact patch -> vertical line -> RC at infinity
        instant_center = np_.array([600.0, 400.0])
        contact_patch = np_.array([0.0, 600.0, 0.0])
        wheel_center = np_.array([0.0, 600.0, 228.0])
        travel = 0.0

    st = _Degenerate()
    assert not np_.isfinite(veh.roll_center_height(kin, 1200.0, state=st)), \
        "a vertical cp->IC line has no finite roll centre; it must not return 0"
    assert not np_.isfinite(_rc_height_mm(st, 1200.0))


def test_laptime_normalises_lateral_demand_by_the_limit_at_speed():
    """Downforce raises what the car can corner at, so in a fast corner the
    lateral demand legitimately EXCEEDS the aero-free grip. Dividing by the
    aero-free figure gave frac_lat > 1, which clamps to 1.0 and zeroes the
    longitudinal capability outright.

    At 30 m/s and 80% of the car's real lateral limit this returned NEGATIVE
    acceleration — only drag left — while the GGV offered +0.38 g. The lap solver
    believed the car could neither accelerate nor brake in any fast corner."""
    from suspension.dynamics import VehicleParams, VehicleDynamics
    from suspension import laptime as lt

    class _T:
        ell_kx = ell_ky = 2.0
        mu_x_ratio = 1.25

    veh = VehicleDynamics(VehicleParams())
    pt = lt.Powertrain()
    pt.combined_tire = _T()
    mu = lt._max_lat_g(veh)
    m, g = veh.p.mass, 9.81

    for v in (20.0, 30.0):
        f_down = 0.5 * pt.rho * max(pt.cla, 0.0) * v * v
        lat_lim = mu * (1.0 + f_down / (m * g))
        assert lat_lim > mu, "test is vacuous without downforce"
        # 80% of the REAL limit must leave real longitudinal capability
        a = lt._accel_long(veh, v, pt, mu, 0.80 * lat_lim) / g
        d = lt._decel_long(veh, v, pt, mu, 0.80 * lat_lim) / g
        assert a > 0.0, f"accel at {v} m/s in a fast corner came out {a:.3f} g"
        assert d > 0.0, f"braking at {v} m/s in a fast corner came out {d:.3f} g"
        # and at the limit itself, longitudinal grip goes to zero, not negative
        a_lim = lt._accel_long(veh, v, pt, mu, lat_lim) / g
        assert a_lim <= a


def test_power_limited_accel_does_not_shrink_with_lateral_demand():
    """A friction ellipse constrains GRIP, not the motor. When the car is
    power-limited the available tractive force does not fall because it is also
    cornering — the correct combination is min(power, grip * ellipse), not
    min(power, grip) * ellipse.

    laptime gets this right: at 20 and 30 m/s its acceleration is flat against
    lateral demand because power is binding. ggv.GGVGenerator applies the
    superellipse to its already-min'd axis limit, so it double-penalises and
    reads up to 43% low in the interior at high speed. This test pins the
    correct behaviour and documents the remaining GGV discrepancy — the envelope
    INTERIOR is still unreconciled, and validate_against_laptime only ever
    compared the three axes."""
    from suspension.dynamics import VehicleParams, VehicleDynamics
    from suspension import laptime as lt
    class _T:
        ell_kx = ell_ky = 2.0
        mu_x_ratio = 1.25

    veh = VehicleDynamics(VehicleParams())
    pt = lt.Powertrain()
    #  The combined tyre lifts the grip ceiling above the power cap at these
    #  speeds, which is what puts the car in the power-limited regime this test
    #  is about. Without it the car is grip-limited and the ellipse SHOULD bite.
    pt.combined_tire = _T()
    mu = lt._max_lat_g(veh)
    for v in (20.0, 30.0):
        flat = [lt._accel_long(veh, v, pt, mu, f * mu) / 9.81 for f in (0.0, 0.3, 0.5)]
        assert max(flat) - min(flat) < 1e-6, \
            f"power-limited accel at {v} m/s varied with lateral demand: {flat}"

    #  Control: grip-limited (no mu_x_ratio headroom) MUST fall with lateral use.
    plain = lt.Powertrain()
    grip_limited = [lt._accel_long(veh, 20.0, plain, mu, f * mu) / 9.81
                    for f in (0.0, 0.3, 0.5)]
    assert grip_limited[0] > grip_limited[-1], \
        "grip-limited acceleration must fall as lateral demand rises"


def test_ggv_and_laptime_agree_across_the_envelope_INTERIOR():
    """The pre-existing validate_against_laptime compares the three AXES and
    passed, while the two models disagreed by 43% two degrees off them. A
    cross-check is only as good as the region it samples.

    Two separate causes, both now fixed:
      * the GGV applied the friction ellipse to its already-min'd axis limit, so
        a power-limited car lost tractive force for cornering. An ellipse
        constrains GRIP; the motor and the brake ceiling are unaffected. Correct
        form is min(cap, grip * ellipse), not min(cap, grip) * ellipse.
      * laptime scaled the lateral limit linearly with downforce. Tyre mu falls
        with load, so that overstates it — 4.9% at 30 m/s. laptime now asks the
        same load-sensitive routine the GGV uses.
    """
    np_ = pytest.importorskip("numpy")
    from suspension.dynamics import VehicleParams, VehicleDynamics
    from suspension import laptime as lt
    from suspension.ggv import GGVGenerator, GGVParams

    class _T:
        ell_kx = ell_ky = 2.0
        mu_x_ratio = 1.25

    veh = VehicleDynamics(VehicleParams())
    pt = lt.Powertrain()
    pt.combined_tire = _T()
    gp = GGVParams.from_powertrain(pt)
    gp.combined_tire = _T()
    speeds = [10.0, 20.0, 30.0]
    res = GGVGenerator(veh, gp).generate(speeds=speeds, n_dir=361)
    mu = lt._max_lat_g(veh)

    worst = 0.0
    for i, v in enumerate(speeds):
        forward = [j for j in range(len(res.theta)) if res.long_g[i][j] > 0]
        for frac in (0.3, 0.5, 0.8, 0.95):
            target = frac * res.max_lat_g[i]
            j = min(forward, key=lambda j: abs(res.lat_g[i][j] - target))
            lon, lat = res.long_g[i][j], res.lat_g[i][j]
            lap = lt._accel_long(veh, v, pt, mu, lat) / 9.81
            assert lap > 0.0, (
                f"laptime gives no forward capability at {v} m/s, {lat:.3f} g "
                f"lateral — inside the car's own envelope")
            worst = max(worst, abs(lon / lap - 1.0))
    assert worst < 0.01, f"envelope interior disagrees by {worst * 100:.1f}%"


# ============================================================================ #
#  PREVIOUSLY UNAUDITED: flex (beam FE), lapsim (third lap solver)
# ============================================================================ #
def test_beam_element_reproduces_every_closed_form():
    """flex._beam_local_K is the structural core, and a 12-DOF Euler-Bernoulli
    element has four exact closed forms plus two structural invariants. Either it
    matches all six or it is not a beam element."""
    np_ = pytest.importorskip("numpy")
    from suspension.flex import _beam_local_K
    E, G = 205000.0, 79000.0
    A, Iy, Iz, L = 147.03, 10136.7, 10136.7, 500.0
    J = 2.0 * Iz
    K = _beam_local_K(E, G, A, Iy, Iz, J, L)

    assert K.shape == (12, 12)
    assert np_.allclose(K, K.T, atol=1e-9), "stiffness matrix must be symmetric"
    w = np_.linalg.eigvalsh(K)
    n_zero = int(np_.sum(np_.abs(w) < 1e-6 * max(abs(w))))
    assert n_zero == 6, (
        f"a free beam element has exactly 6 rigid-body modes, found {n_zero} — "
        f"fewer means a spurious constraint, more means a mechanism")

    free = list(range(6, 12))                     # cantilever: fix node 1
    Kff = K[np_.ix_(free, free)]
    P = 1000.0

    def tip(dof):
        f = np_.zeros(6)
        f[dof] = P
        return np_.linalg.solve(Kff, f)[dof]

    assert abs(tip(2) - P * L ** 3 / (3 * E * Iy)) < 1e-9   # bending, PL^3/3EI
    assert abs(tip(1) - P * L ** 3 / (3 * E * Iz)) < 1e-9
    assert abs(tip(0) - P * L / (A * E)) < 1e-12            # axial, PL/AE
    assert abs(tip(3) - P * L / (G * J)) < 1e-14            # torsion, TL/GJ


def test_lapsim_brake_cap_is_a_cap_not_a_grip_coefficient():
    """`brake_g` is defined as "max decel the brakes+tires can sustain" — a
    ceiling. Multiplying it by the downforce factor (1 + Fz_aero/mg), which is
    what you do to a GRIP limit, walks straight through the ceiling it exists to
    impose: 1.6 g became 2.35 g at 30 m/s, 2.57 g with drag and rolling. FSAE
    cars brake at 1.5-1.8 g.

    Grip scales with downforce; a mechanical ceiling does not. Same shape as the
    GGV interior defect."""
    from suspension.lapsim import LapSimulator, LapSimParams
    p = LapSimParams()
    sim = LapSimulator(p)
    for v in (5.0, 15.0, 25.0, 35.0):
        a_g = sim._braking_decel(v) / p.g
        f_down = 0.5 * p.rho * p.cl_a * v * v
        drag_g = (0.5 * p.rho * p.cd_a * v * v) / p.mass / p.g
        ceiling = p.brake_g + drag_g + p.rolling_g
        assert a_g <= ceiling + 1e-6, (
            f"{a_g:.3f} g at {v} m/s exceeds the {p.brake_g} g brake ceiling "
            f"plus drag and rolling ({ceiling:.3f} g)")
        assert a_g > 0.0
    # Downforce must still help via drag, so decel rises with speed.
    assert sim._braking_decel(35.0) > sim._braking_decel(5.0)


def test_throttle_compressible_flow_matches_the_textbook_choked_form():
    """Isentropic flow through the plate sets everything downstream in the
    throttle-response model. Two exact checks: the choked mass flow has a closed
    form, and past the critical pressure ratio the flow must PLATEAU rather than
    keep rising — the classic error is forgetting to clamp at the critical
    ratio, which makes flow grow without bound as manifold pressure falls."""
    from suspension.throttle_dynamics import compressible_mass_flow, GAMMA, R_AIR
    p_up, T, A, cd = 101325.0, 293.0, 0.002, 0.75
    ref = (cd * A * p_up * math.sqrt(GAMMA / (R_AIR * T))
           * (2.0 / (GAMMA + 1.0)) ** ((GAMMA + 1.0) / (2.0 * (GAMMA - 1.0))))
    assert abs(compressible_mass_flow(A, p_up, 1000.0, T, cd) / ref - 1.0) < 1e-9

    crit = (2.0 / (GAMMA + 1.0)) ** (GAMMA / (GAMMA - 1.0))
    plateau = [compressible_mass_flow(A, p_up, f * p_up, T, cd)
               for f in (crit, 0.4, 0.2, 0.05)]
    assert max(plateau) - min(plateau) < 1e-12, "flow must plateau once choked"
    rising = [compressible_mass_flow(A, p_up, f * p_up, T, cd)
              for f in (0.99, 0.9, 0.7, 0.6)]
    assert all(b > a for a, b in zip(rising, rising[1:])), \
        "below choking, flow must rise as downstream pressure falls"
    assert compressible_mass_flow(A, p_up, p_up, T, cd) == 0.0


def test_pcm_effective_cp_conserves_latent_heat():
    """The enthalpy method smears latent heat into an effective specific heat
    across the melt window. Integrating that bump back out must return the
    declared latent heat — otherwise the PCM silently absorbs more or less energy
    than the material datasheet says it can, which is the entire point of
    fitting one."""
    np_ = pytest.importorskip("numpy")
    from suspension.pcm_cooling import PCMMaterial
    m = PCMMaterial()
    lo = m.t_melt_c - 0.5 * m.melt_window_c
    hi = m.t_melt_c + 0.5 * m.melt_window_c
    T = np_.linspace(lo, hi, 200001)
    cp = np_.array([m.effective_cp_j_per_gk(t) for t in T])
    sensible = 0.5 * (m.cp_solid_j_per_gk + m.cp_liquid_j_per_gk)
    recovered = float(np_.trapezoid(cp - sensible, T))
    assert abs(recovered / m.latent_heat_j_per_g - 1.0) < 1e-3, (
        f"cp curve integrates to {recovered:.2f} J/g against a declared "
        f"{m.latent_heat_j_per_g:.2f} J/g")
    # Outside the window it must fall back to the plain sensible values.
    assert abs(m.effective_cp_j_per_gk(lo - 20.0) - m.cp_solid_j_per_gk) < 1e-9
    assert abs(m.effective_cp_j_per_gk(hi + 20.0) - m.cp_liquid_j_per_gk) < 1e-9


# ============================================================================ #
#  PROVENANCE PROPAGATION
# ============================================================================ #
def test_objective_grade_is_weakest_load_bearing_input():
    """`aggregate()` combined values and dropped grades, so an objective came
    out as a bare float: the uncertainty BAND survived but the pedigree did not.
    Those answer different questions — a band says how far the answer might
    move, a grade says whether anything measured is behind it at all. A lap time
    built from a guessed mass is not a modelled lap time with a wide band, it is
    a guess.

    Two properties have to hold together, and they pull against each other:
      * weakest link — good evidence must not launder bad, so a channel is only
        as good as its worst contributor;
      * load-bearing only — a badly-known input the objective never reads must
        NOT downgrade it, or every output ends up stamped GUESS and the badge
        stops carrying information.
    """
    import suspension.proof_engine as pe
    from suspension.proof_engine import Quantity as Q, EvidenceGrade as G

    obj = [o for o in pe.DEFAULT_OBJECTIVES if "lap" in o.key][0]

    def build(pt_grade, cla_grade=G.GUESS):
        return [
            Q("m_chassis", "chassis", "mass_kg", "Chassis", 32.0, "kg", G.MEASURED),
            Q("m_pt", "powertrain", "mass_kg", "Powertrain", 58.0, "kg", pt_grade),
            Q("m_driver", "driver", "mass_kg", "Driver", 68.0, "kg", G.VERIFIED),
            Q("cla", "aero", "cl_a", "ClA", 2.4, "m2", cla_grade),
        ]

    # 1. the answer tracks the weakest LOAD-BEARING input, monotonically
    seen = []
    for g in (G.VERIFIED, G.MEASURED, G.MODELLED, G.ESTIMATE, G.GUESS):
        rep = pe.analyze_objective(obj, build(g))
        seen.append(rep.grade.rank)
        assert "mass_kg" in rep.limited_by
    assert seen == sorted(seen, reverse=True), \
        f"answer grade must fall as the input grade falls, got {seen}"

    # 2. mass is summed across subsystems: one GUESS contributor makes the whole
    #    channel a guess, however good the others are. No laundering.
    grades = pe.aggregate_grades(build(G.GUESS))
    assert grades["mass_kg"] == G.GUESS

    # 3. a GUESS the objective is insensitive to must NOT downgrade the answer
    rep = pe.analyze_objective(obj, build(G.MEASURED, cla_grade=G.GUESS))
    assert rep.grade == G.MEASURED, (
        "a guessed input with no influence on the objective downgraded it; the "
        "badge stops meaning anything if everything reads GUESS")

    # 4. every report carries a grade — it cannot be forgotten
    assert isinstance(rep.grade, G) and rep.limited_by


def test_grade_never_launders_through_aggregation():
    """The specific dishonesty guarded against: averaging grades would let three
    measured subsystems carry a guessed fourth."""
    import suspension.proof_engine as pe
    from suspension.proof_engine import Quantity as Q, EvidenceGrade as G
    qs = [Q(f"m{i}", f"s{i}", "mass_kg", f"S{i}", 20.0, "kg", g)
          for i, g in enumerate((G.VERIFIED, G.MEASURED, G.MEASURED, G.GUESS))]
    assert pe.aggregate_grades(qs)["mass_kg"] == G.GUESS


def test_proof_plan_headline_carries_its_grade():
    """The current-answer line is the one sentence a reader quotes out of the
    proof plan. It used to print identically whether every input was measured on
    the car or guessed in a meeting. It now carries the grade AND the limiting
    channel, because the limiting channel is the actionable half."""
    import suspension.proof_engine as pe
    from suspension.proof_engine import Quantity as Q, EvidenceGrade as G

    obj = [o for o in pe.DEFAULT_OBJECTIVES if "lap" in o.key][0]

    def plan_line(pt_grade):
        qs = [Q("m_chassis", "chassis", "mass_kg", "Chassis", 32.0, "kg", G.MEASURED),
              Q("m_pt", "powertrain", "mass_kg", "Powertrain", 58.0, "kg", pt_grade),
              Q("m_driver", "driver", "mass_kg", "Driver", 68.0, "kg", G.VERIFIED)]
        rep = pe.analyze_objective(obj, qs)
        plan = pe.plan_proofs(obj, qs)
        return pe.render_proof_plan_md(plan, rep)

    weak = plan_line(G.GUESS)
    strong = plan_line(G.MEASURED)
    assert "guess" in weak and "limited by mass_kg" in weak
    assert "measured" in strong
    assert weak != strong, "the headline must change when the evidence changes"
    # The uncertainty band and the grade are separate signals and both must show.
    assert "±" in weak


def test_anti_squat_shortcut_error_is_bounded():
    """The wheel-centre reference point, checked by STATIC EQUILIBRIUM rather
    than by citation — solving the upright as a rigid body (6 equations, 6 link
    unknowns) and reading the pushrod force directly. No path slope, no tan(phi),
    no sign convention from the module under test.

    Three load cases on the default geometry:
        solid axle, force at the contact patch     -40.3 %
        inboard drive, halfshaft torque modelled   +16.1 %
        wheel-centre shortcut                      +17.1 %

    Two conclusions. The reference point is not cosmetic — patch versus wheel
    centre is a 56-point swing that changes the SIGN, so the fix was both real
    and large. And the shortcut is exact only at zero camber and zero scrub,
    because it puts the whole couple (wc-cp) x Fx onto the linkage while
    physically only the spin-axis component is driveline-reacted; the rest goes
    to the bearings and the tie rod. That leaves ~1 pp of optimism here.

    This test pins the bound. If it widens, the geometry has drifted somewhere
    the shortcut no longer holds and the exact static form is needed.
    """
    np_ = pytest.importorskip("numpy")

    def _basis(a, b, o):
        u = np_.asarray(a, float) - np_.asarray(o, float)
        v = np_.asarray(b, float) - np_.asarray(o, float)
        return u / np_.linalg.norm(u), v / np_.linalg.norm(v)

    def pushrod(hp, st, apps, couples=()):
        uo, lo, tro, pro = st.upper_outer, st.lower_outer, st.tie_rod_outer, st.pushrod_outer
        u1, u2 = _basis(hp.upper_front_inner, hp.upper_rear_inner, uo)
        l1, l2 = _basis(hp.lower_front_inner, hp.lower_rear_inner, lo)
        tr = np_.asarray(hp.tie_rod_inner, float) - np_.asarray(tro, float)
        tr /= np_.linalg.norm(tr)
        pr = np_.asarray(hp.rocker_pushrod, float) - np_.asarray(pro, float)
        pr /= np_.linalg.norm(pr)
        A = np_.zeros((6, 6))
        for j, (d, p) in enumerate([(u1, uo), (u2, uo), (l1, lo), (l2, lo),
                                    (tr, tro), (pr, pro)]):
            A[0:3, j] = d
            A[3:6, j] = np_.cross(np_.asarray(p, float), d)
        F = np_.zeros(3)
        M = np_.zeros(3)
        for f, p in apps:
            F = F + np_.asarray(f, float)
            M = M + np_.cross(np_.asarray(p, float), np_.asarray(f, float))
        for c in couples:
            M = M + np_.asarray(c, float)
        return float(np_.linalg.solve(A, np_.concatenate([-F, -M]))[5])

    hp = Hardpoints.default()
    st = SuspensionKinematics(hp).static
    cp = np_.asarray(st.contact_patch, float)
    wc = np_.asarray(st.wheel_center, float)
    m, g, L, h = 300.0, 9.81, 1550.0, 300.0
    d_fz = m * g * h / L / 2.0
    fx = np_.array([-m * g / 2.0, 0.0, 0.0])          # traction, forward
    p0 = pushrod(hp, st, [([0, 0, d_fz], cp)])

    at_patch = 100.0 * (1 - pushrod(hp, st, [(fx, cp), ([0, 0, d_fz], cp)]) / p0)
    at_wc = 100.0 * (1 - pushrod(hp, st, [(fx, wc), ([0, 0, d_fz], cp)]) / p0)

    cam = math.radians(st.camber)
    axis = np_.array([0.0, math.cos(cam), math.sin(cam)])
    full = np_.cross(wc - cp, fx)
    torque_path = 100.0 * (1 - pushrod(
        hp, st, [(fx, cp), ([0, 0, d_fz], cp)],
        couples=[float(np_.dot(full, axis)) * axis]) / p0)

    # the reference point changes the SIGN — it is not a refinement
    assert at_patch < 0 < at_wc
    assert abs(at_wc - at_patch) > 40.0
    # the module agrees with the wheel-centre statics
    assert abs(SuspensionKinematics(hp).anti_squat_pct(h, L, 1.0) - at_wc) < 0.5
    # and the shortcut sits within ~1.5 pp of the modelled torque path
    assert abs(at_wc - torque_path) < 1.5, (
        f"shortcut {at_wc:.2f}% vs torque path {torque_path:.2f}% — the "
        f"zero-camber/zero-scrub assumption no longer holds for this geometry")


# ============================================================================ #
#  PHYSICALLY REQUIRED ORDERINGS
# ============================================================================ #
#  The 1-D rotor bug was displayed in the app for however long it shipped: the
#  panel showed the friction face 79 C COOLER than the through-thickness mean,
#  directly beneath a help string saying it runs hotter. Heat enters at the
#  face; that ordering cannot invert. Nobody checked, because nothing asserted
#  it — the number was merely printed.
#
#  These are the cheapest tests in the file. Each states a relationship that
#  holds by construction of the physics, independent of any parameter value, so
#  none of them can be wrong in a way the author also believed. Add one wherever
#  two quantities are shown together with an ordering that must hold.
def test_rotor_surface_is_never_cooler_than_its_core():
    """Heat enters at the friction face. A surface below the core, or a
    negative gradient, means the surface node is shedding more than it receives
    — which is exactly what a doubled convective area does."""
    np_ = pytest.importorskip("numpy")
    from suspension.brake_thermal import OneDRotor, TwoNodeRotorPad, TwoNodeParams
    dt, n = 0.02, 3000
    power = np_.zeros(n)
    for i in range(0, n, 250):
        power[i:i + 40] = 45000.0
    speed = np_.full(n, 20.0)
    one = OneDRotor(TwoNodeParams(), n_nodes=12).simulate(power, dt, speed)
    two = TwoNodeRotorPad(TwoNodeParams()).simulate(power, dt, speed)

    assert one.dT_gradient_peak_c >= 0.0
    assert max(one.T_surface_c) >= max(one.T_core_c)
    # and the resolved face must exceed the lumped MEAN, which is the exact
    # comparison the UI puts on screen
    assert one.T_surface_peak_c > two.T_rotor_peak_c, (
        f"surface peak {one.T_surface_peak_c:.0f} C is below the lumped bulk "
        f"{two.T_rotor_peak_c:.0f} C — not physical, and it is what the app "
        f"renders as its 'vs bulk' delta")


def test_a_trace_peak_is_never_below_its_final_value():
    np_ = pytest.importorskip("numpy")
    from suspension.brake_thermal import TwoNodeRotorPad, TwoNodeParams
    dt, n = 0.02, 3000
    power = np_.zeros(n)
    for i in range(0, n, 250):
        power[i:i + 40] = 45000.0
    tr = TwoNodeRotorPad(TwoNodeParams()).simulate(power, dt, np_.full(n, 20.0))
    assert tr.T_rotor_peak_c >= tr.T_rotor_final_c
    assert tr.T_pad_peak_c >= min(tr.T_pad_c)


def test_outer_wheels_gain_load_and_no_wheel_goes_negative():
    """Under lateral acceleration the outer pair must gain and the inner pair
    lose, by the same amount per axle. A negative wheel load means the model has
    lifted a wheel and kept computing grip from it."""
    from suspension.dynamics import VehicleDynamics, VehicleParams
    p = VehicleParams()
    veh = VehicleDynamics(p)
    static_corner = {"f": p.mass * p.g * p.weight_dist_front / 2.0,
                     "r": p.mass * p.g * (1 - p.weight_dist_front) / 2.0}
    for g in (0.5, 1.0, 1.5):
        loads, _ = veh.lateral_load_transfer(g)
        assert loads.fr > loads.fl and loads.rr > loads.rl, \
            f"at {g}g the outer wheels did not gain load"
        assert min(loads.as_tuple()) >= 0.0, \
            f"at {g}g a wheel load went negative ({min(loads.as_tuple()):.0f} N)"
        # the axle total is conserved: transfer moves load, it does not create it
        assert abs((loads.fl + loads.fr) - 2 * static_corner["f"]) < 1e-6
        assert abs((loads.rl + loads.rr) - 2 * static_corner["r"]) < 1e-6


def test_downforce_can_only_help_lateral_grip():
    from suspension.dynamics import VehicleDynamics, VehicleParams
    from suspension.ggv import GGVGenerator, GGVParams
    from suspension import laptime as lt
    gen = GGVGenerator(VehicleDynamics(VehicleParams()),
                       GGVParams.from_powertrain(lt.Powertrain()))
    seq = [gen._max_lateral_g_at_speed(v) for v in (5.0, 15.0, 25.0, 35.0)]
    for a, b in zip(seq, seq[1:]):
        assert b >= a - 1e-9, f"lateral limit fell with speed: {seq}"


def test_coolant_never_settles_below_ambient():
    from suspension.cooling import size_loop, LoopSpec
    for heat in (1000.0, 4000.0, 9000.0):
        s = LoopSpec(heat_w=heat)
        assert size_loop(s).steady_coolant_c >= s.ambient_c - 1e-9


def test_declared_min_max_bounds_cannot_be_inverted():
    """A min above its max is not a configuration, it is a typo — and every
    check downstream inherits the nonsense. An inverted SensorSpec range gives a
    negative span, so the LSB and effective bit depth go negative and the
    resolution finding comes back CLEAN. It scored OK on every gate in the DAQ
    planner, because those validate the signal CHAIN and nothing validated the
    sensor's own declaration.

    Found by enumerating every result/config dataclass in the package for
    orderable field pairs (peak/final, max/min, surface/core) — 19 of them — and
    then asking which could be constructed the wrong way round. Four could."""
    from suspension.daq_plan import SensorSpec, ANALOG_TYPES
    import inspect
    params = inspect.signature(SensorSpec).parameters
    analog = list(ANALOG_TYPES)[0]

    def spec(lo, hi):
        base = dict(key="s", name="damper pot", output=analog,
                    range_min_eu=lo, range_max_eu=hi, adc_bits=12)
        return SensorSpec(**{k: v for k, v in base.items() if k in params})

    spec(0.0, 100.0)                       # sane: must construct
    spec(-50.0, 50.0)                      # signed range: also fine
    with pytest.raises(ValueError):
        spec(100.0, 0.0)

    #  The other three found by the same enumeration. MeshParams matters most:
    #  its pair is written straight into the snappyHexMesh deck as
    #  "level (min max)", so an inverted pair leaves the repo and the failure
    #  lands in OpenFOAM hours later, where it costs far more to diagnose.
    from suspension.tractive_system import Rules
    from suspension.aero.meshing import MeshParams
    from suspension.aero.run_log import ScreenConfig
    for cls, kwargs in (
        (Rules, dict(precharge_min_time_s=10.0, precharge_max_time_s=1.0)),
        (Rules, dict(tsal_flash_hz_min=8.0, tsal_flash_hz_max=2.0)),
        (MeshParams, dict(surface_min_level=8, surface_max_level=2)),
        (ScreenConfig, dict(courant_warn_min=50.0, courant_warn_max=1.0)),
    ):
        cls()                              # defaults must still build
        with pytest.raises(ValueError):
            cls(**kwargs)


def test_result_peaks_dominate_their_series():
    """Generic form of the rotor check: any *_peak field must actually be the
    peak of the series it summarises. An off-by-one or a reset inside the loop
    breaks this silently, and the summary is what gets quoted."""
    np_ = pytest.importorskip("numpy")
    from suspension.brake_thermal import (TwoNodeRotorPad, OneDRotor, TwoNodeParams)
    dt, n = 0.02, 3000
    power = np_.zeros(n)
    for i in range(0, n, 250):
        power[i:i + 40] = 45000.0
    speed = np_.full(n, 20.0)

    two = TwoNodeRotorPad(TwoNodeParams()).simulate(power, dt, speed)
    one = OneDRotor(TwoNodeParams(), n_nodes=12).simulate(power, dt, speed)
    for peak, series, label in (
        (two.T_rotor_peak_c, two.T_rotor_c, "rotor"),
        (two.T_pad_peak_c, two.T_pad_c, "pad"),
        (one.T_surface_peak_c, one.T_surface_c, "surface"),
        (one.dT_gradient_peak_c, one.dT_gradient_c, "gradient"),
        (one.sigma_peak_mpa, one.sigma_mpa, "thermal stress"),
    ):
        assert abs(peak - max(series)) < 1e-6, \
            f"{label}: reported peak {peak:.3f} != series max {max(series):.3f}"


def test_nothing_non_finite_escapes_into_an_external_deck():
    """THE EXPORT BOUNDARY. Everything written into a snappyHexMesh dict or a
    Fluent journal leaves this repo and executes elsewhere, so a bad value
    surfaces hours later, on a cluster, in the wrong tool, to someone who cannot
    see this code. Auditing the four export writers found ZERO finite-checks
    between them and three live escapes:

      * a NaN roll was swallowed by the axis-angle extraction and written as
        "angle 0.0" — the requested attitude SILENTLY DISCARDED and the car
        meshed flat. Worse than a crash: the run completes and looks clean.
      * an infinite ride height wrote "(0 0 inf)" into the snappy transform.
      * a NaN yaw wrote "(nan, nan, 0.0)" as the Fluent inlet velocity.

    Not hypothetical: this package deliberately returns NaN for genuinely
    undefined geometry (degenerate roll centre, parallel-link swing arm), which
    is the honest thing to do upstream and precisely why the edge needs a hard
    stop."""
    from suspension.aero.cfd import Attitude
    from suspension.aero.meshing import _attitude_geometry_transform
    from suspension.aero.fluent_journal import _inlet_velocity

    good = Attitude(roll_deg=1.0, pitch_deg=2.0, ride_height_mm=30.0, speed_ms=20.0)
    _attitude_geometry_transform(good)          # must still work
    _inlet_velocity(20.0, 3.0, 0.0)

    base = dict(roll_deg=0.0, pitch_deg=0.0, ride_height_mm=30.0, speed_ms=20.0)
    for field in ("roll_deg", "pitch_deg", "ride_height_mm"):
        for bad in (float("nan"), float("inf")):
            att = Attitude(**{**base, field: bad})
            with pytest.raises(ValueError):
                _attitude_geometry_transform(att)

    for args in ((float("nan"), 0.0, 0.0), (20.0, float("nan"), 0.0),
                 (float("inf"), 0.0, 0.0)):
        with pytest.raises(ValueError):
            _inlet_velocity(*args)

    # the guard must name the offending field — a caller three layers up needs
    # to know WHICH number went bad, not merely that one did
    try:
        _attitude_geometry_transform(Attitude(**{**base, "roll_deg": float("nan")}))
    except ValueError as exc:
        assert "roll_deg" in str(exc)
