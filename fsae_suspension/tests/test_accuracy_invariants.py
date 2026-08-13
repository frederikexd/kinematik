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
    assert abs(ad_good + ad_bad) < 1.0, "mirroring should flip sign, not magnitude"


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
    assert abs(kin.anti_dive_pct(300.0, 1550.0)) < 1e-6
    assert abs(kin.anti_squat_pct(300.0, 1550.0)) < 1e-6


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
