"""suspension/tirefit.py: the fit recovers a tire it was given data from (to a fraction of a newton), aligns a
flipped sign convention, filters unloaded and non-finite points, weights by load, and reports its statistics; the
coefficient set and bounds it fits within are pinned."""
import math

import numpy as np
import pytest

from suspension import tirefit as tf
from suspension.tiremodel import PacejkaLateral, _DEFAULT_FSAE_COEFFS

TRUE = {**_DEFAULT_FSAE_COEFFS, "PCY1": 1.5, "PDY1": 1.7, "PDY2": -0.3, "PDY3": 1.0, "PKY1": -30.0, "PKY2": 1.8,
        "PEY1": -0.4, "PVY3": 0.1}


def _data(n_sa=21, loads=(500.0, 900.0, 1300.0), ias=(0.0, 0.035)):
    t = PacejkaLateral(TRUE, FNOMIN=900.0)
    SA, FZ, IA = (np.array(v).ravel() for v in np.meshgrid(np.radians(np.linspace(-12, 12, n_sa)), loads, ias))
    return SA, FZ, IA, t.fy(SA, FZ, IA)


def test_the_fitted_set_and_its_bounds():
    assert tf._FIT_KEYS == ["PCY1", "PDY1", "PDY2", "PDY3", "PEY1", "PEY2", "PEY3", "PEY4", "PKY1", "PKY2", "PKY3",
                            "PHY1", "PHY2", "PHY3", "PVY1", "PVY2", "PVY3", "PVY4"]
    assert tf._BOUNDS == {"PCY1": (1.0, 2.0), "PDY1": (0.5, 3.0), "PDY2": (-2.0, 0.5), "PDY3": (-10.0, 10.0),
                          "PEY1": (-5.0, 1.0), "PEY2": (-2.0, 2.0), "PEY3": (-2.0, 2.0), "PEY4": (-20.0, 20.0),
                          "PKY1": (-80.0, -5.0), "PKY2": (0.5, 5.0), "PKY3": (-2.0, 2.0), "PHY1": (-0.1, 0.1),
                          "PHY2": (-0.1, 0.1), "PHY3": (-0.2, 0.2), "PVY1": (-0.5, 0.5), "PVY2": (-0.5, 0.5),
                          "PVY3": (-1.0, 1.0), "PVY4": (-1.0, 1.0)}
    t = tf._build_tire(np.arange(18.0), 800.0)
    assert t.FNOMIN == 800.0 and [t.coeffs[k] for k in tf._FIT_KEYS] == list(np.arange(18.0))
    assert set(t.coeffs) == set(_DEFAULT_FSAE_COEFFS)
    seed = PacejkaLateral(dict(_DEFAULT_FSAE_COEFFS), FNOMIN=700.0)
    a = math.radians(1.0)
    assert tf._seed_zero_slope(700.0) == pytest.approx((seed.fy(a, 700.0) - seed.fy(-a, 700.0)) / (2 * a))


def test_the_fit_recovers_the_tire_that_made_the_data(capsys):
    SA, FZ, IA, FY = _data()
    r = tf.fit_mf52_lateral(SA, FZ, FY, IA, fnomin=900.0, verbose=True)
    assert r["n"] == len(SA) and r["FNOMIN"] == 900.0 and r["rmse_N"] < 2.0 and r["r2"] > 0.99999
    fit = PacejkaLateral(r["coeffs"], FNOMIN=900.0)
    assert np.abs(fit.fy(SA, FZ, IA) - FY).max() < 10.0
    out = capsys.readouterr().out
    assert out == f"  fit on {len(SA)} pts | FNOMIN=900 N | RMSE={r['rmse_N']:.1f} N | R^2={r['r2']:.4f}\n"
    pred = fit.fy(SA, FZ, IA)
    err = pred - FY
    assert r["rmse_N"] == pytest.approx(math.sqrt(np.mean(err ** 2)))
    assert r["r2"] == pytest.approx(1 - np.sum(err ** 2) / np.sum((FY - FY.mean()) ** 2))
    assert set(r) == {"coeffs", "FNOMIN", "rmse_N", "r2", "n"}


def test_a_flipped_sign_convention_and_the_default_nominal_load():
    SA, FZ, IA, FY = _data(ias=(0.0,))
    r = tf.fit_mf52_lateral(SA, -FZ, -FY)                   # loads given negative, force sign flipped
    assert r["FNOMIN"] == pytest.approx(float(np.median(FZ))) and r["rmse_N"] < 2.0
    fit = PacejkaLateral(r["coeffs"], FNOMIN=r["FNOMIN"])
    assert np.abs(fit.fy(SA, FZ) - FY).max() < 10.0         # back in the model's own convention
    unweighted = tf.fit_mf52_lateral(SA, FZ, FY, weight_by_load=False, fnomin=900.0)
    assert unweighted["rmse_N"] < 2.0


def test_unloaded_and_non_finite_points_are_dropped():
    SA, FZ, IA, FY = _data()
    SA2 = np.append(SA, [0.1, np.nan, 0.1, 0.1])
    FZ2 = np.append(FZ, [40.0, 900.0, 900.0, 50.0])
    FY2 = np.append(FY, [5000.0, 0.0, np.inf, -3000.0])
    IA2 = np.append(IA, [0.0, 0.0, 0.0, 0.0])
    r = tf.fit_mf52_lateral(SA2, FZ2, FY2, IA2, fnomin=900.0)
    assert r["n"] == len(SA) and r["rmse_N"] < 2.0
    bad = np.append(IA, [np.nan])
    r = tf.fit_mf52_lateral(np.append(SA, 0.1), np.append(FZ, 900.0), np.append(FY, 1.0), bad, fnomin=900.0)
    assert r["n"] == len(SA)
    with pytest.raises(ValueError) as e:
        tf.fit_mf52_lateral(SA[:49], FZ[:49], FY[:49])
    assert str(e.value) == ("Only 49 usable points — need at least ~50 spanning a range of slip and load to fit a "
                            "Magic Formula.")
    assert tf.fit_mf52_lateral(SA[:50], FZ[:50], FY[:50], IA[:50], fnomin=900.0)["n"] == 50


def test_a_constant_force_has_no_r2():
    SA = np.radians(np.linspace(-5, 5, 60))
    r = tf.fit_mf52_lateral(SA, np.full(60, 900.0), np.full(60, 25.0))
    assert math.isnan(r["r2"])


def test_the_ttc_channel_wrapper():
    SA, FZ, IA, FY = _data()
    r = tf.fit_from_ttc_channels({"SA": np.degrees(SA), "FZ": FZ, "FY": FY, "IA": np.degrees(IA)}, fnomin=900.0)
    assert r["rmse_N"] < 2.0
    SA, FZ, IA, FY = _data(ias=(0.0,))
    assert tf.fit_from_ttc_channels({"SA": np.degrees(SA), "FZ": FZ, "FY": FY}, fnomin=900.0)["rmse_N"] < 2.0
    for missing in ("FY", "FZ", "SA"):
        ch = {"SA": [0.0], "FZ": [1.0], "FY": [1.0]}
        del ch[missing]
        with pytest.raises(ValueError) as e:
            tf.fit_from_ttc_channels(ch)
        assert str(e.value) == "Need FY, FZ and SA channels to fit a lateral model."


def test_load_weighting_changes_a_noisy_fit():
    SA, FZ, IA, FY = _data(ias=(0.0,))
    noise = np.random.default_rng(4).normal(0.0, 40.0, len(FY)) * (900.0 / FZ)       # noisier at light load
    a = tf.fit_mf52_lateral(SA, FZ, FY + noise, fnomin=900.0)
    b = tf.fit_mf52_lateral(SA, FZ, FY + noise, fnomin=900.0, weight_by_load=False)
    assert a["coeffs"] != b["coeffs"]
    heavy = FZ == 1300.0
    fa, fb = (PacejkaLateral(r["coeffs"], FNOMIN=900.0).fy(SA, FZ) for r in (a, b))
    assert np.sqrt(np.mean((fa - FY)[heavy] ** 2)) < np.sqrt(np.mean((fb - FY)[heavy] ** 2))


def test_the_fit_statistics_on_noisy_data_and_the_defaults(capsys):
    SA, FZ, IA, FY = _data(ias=(0.0,))
    noisy = FY + np.random.default_rng(1).normal(0.0, 30.0, len(FY))
    r = tf.fit_mf52_lateral(SA, FZ, noisy, fnomin=900.0)
    err = PacejkaLateral(r["coeffs"], FNOMIN=900.0).fy(SA, FZ) - noisy
    assert r["rmse_N"] == pytest.approx(math.sqrt(np.mean(err ** 2)), rel=1e-9)
    assert r["r2"] == pytest.approx(1 - np.sum(err ** 2) / np.sum((noisy - noisy.mean()) ** 2), rel=1e-9)
    assert 0.9 < r["r2"] < 1.0 and capsys.readouterr().out == ""
    flat = tf.fit_mf52_lateral(np.radians(np.linspace(-5, 5, 60)), np.full(60, 900.0),
                               25.0 + 0.01 * np.sin(np.arange(60)))           # total variance under 1 N^2
    assert not math.isnan(flat["r2"])
    edge = tf.fit_mf52_lateral(np.append(SA, 0.05), np.append(FZ, 50.5), np.append(FY, 40.0), fnomin=900.0)
    assert edge["n"] == len(SA) + 1
    assert tf.fit_from_ttc_channels({"SA": np.degrees(SA), "FZ": FZ, "FY": FY}, fnomin=777.0)["FNOMIN"] == 777.0


# ---- the channel wrapper's guards: exact boundary and exact words ----
_SA = np.linspace(-12, 12, 60)
_CH = {"SA": _SA, "FZ": np.full(60, 1000.0), "FY": -1500 * np.tanh(_SA / 4)}


def _says(exc_info, text):
    assert str(exc_info.value) == text


def test_channels_need_eight_finite_samples_each():
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({k: v[:7] for k, v in _CH.items()})
    _says(e, "tire data channel FY: needs at least 8 value(s), got 7")
    with pytest.raises(ValueError) as e:                         # eight pass the channel check; the fit wants ~50
        tf.fit_from_ttc_channels({k: v[:8] for k, v in _CH.items()})
    _says(e, "Only 8 usable points — need at least ~50 spanning a range of slip and load to fit a Magic Formula.")
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "FZ": np.where(_SA == _SA[3], np.inf, _CH["FZ"])})
    _says(e, "tire data channel FZ: 1 of 60 value(s) are not finite numbers (nan or inf)")


def test_channels_must_have_one_length():
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "FZ": _CH["FZ"][:59]})
    _says(e, "tire data channels must be the same length, got {'FY': 60, 'FZ': 59, 'SA': 60}")
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "IA": np.zeros(30)})
    _says(e, "tire data channels must be the same length, got {'FY': 60, 'FZ': 60, 'SA': 60, 'IA': 30}")
    assert tf.fit_from_ttc_channels({**_CH, "IA": np.zeros(60)})["n"] == 60


def test_a_fit_that_is_not_finite_is_refused():
    with pytest.raises(ValueError) as e:                         # constant force: finite rmse, no r2
        tf.fit_from_ttc_channels({**_CH, "FY": np.full(60, 25.0)})
    _says(e, "the tire fit did not converge to finite numbers: the data may be constant, far out of range, or in the "
             "wrong units")


def test_too_little_spread_for_the_fit_is_a_value_error(monkeypatch):
    def _short(*a, **k):
        raise IndexError("index 0 is out of bounds")
    monkeypatch.setattr(tf, "fit_mf52_lateral", _short)
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels(_CH)
    _says(e, "the tire data does not cover enough loads and slip angles to fit a lateral model")


def test_every_channel_is_checked_by_name():
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "SA": np.where(_SA == _SA[5], np.nan, _SA)})
    _says(e, "tire data channel SA: 1 of 60 value(s) are not finite numbers (nan or inf)")
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "IA": np.full(60, np.inf)})
    _says(e, "tire data channel IA: 60 of 60 value(s) are not finite numbers (nan or inf)")
    with pytest.raises(ValueError) as e:
        tf.fit_from_ttc_channels({**_CH, "SA": _SA[:50]})
    _says(e, "tire data channels must be the same length, got {'FY': 60, 'FZ': 60, 'SA': 50}")
