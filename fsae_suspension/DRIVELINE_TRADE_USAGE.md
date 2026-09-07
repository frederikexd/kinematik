# Driveline & component trade study — usage

Two new modules that let KinematiK answer a question it previously could not:
**"should we buy this specific part?"**

* `suspension/driveline.py` — a differential model, and the derivation of
  `LapSimParams.drive_grip_frac` from real corner-exit physics instead of a
  typed-in guess.
* `suspension/trade.py` — an A/B/C purchase trade study with an explicit
  refusal rule.

---

## Why these exist

`worthwhile.py` answers *"does the car we are actually building still score?"*
by comparing a paper car to a reconciled car. It cannot compare **option A to
option B**, and until now nothing in the toolkit could tell two differentials
apart at all:

* `LapSimParams.drive_grip_frac` is a single hand-typed scalar standing in for
  the entire rear-axle traction limit. Every differential in the world produced
  byte-identical lap times.
* There was no torque-bias, preload, or per-wheel-drive-torque model anywhere in
  the repo (`grep -ri "torque bias\|differential" suspension/*.py` on the
  physics modules returns nothing).
* There was no monetary cost model outside `omnicore`'s parametric sweep, so a
  price tag had nowhere to go.

## What `driveline.py` does

Solves the rear axle at a corner exit:

1. per-wheel vertical load from the real `dynamics.lateral_load_transfer`
   solver (roll-stiffness split + geometric/roll-centre component), plus the
   rear share of downforce;
2. per-wheel force capacity from whichever grip model is attached (Pacejka
   MF5.2 when a tyre is loaded, the linear placeholder otherwise);
3. a friction-circle split between the cornering already being done and the
   longitudinal capacity that remains;
4. the differential's torque-split decision:

   | type | total axle torque |
   |---|---|
   | open | `2 × T_inside` |
   | LSD  | `T_inside + min(T_out_cap, TBR × T_inside + preload)` |
   | spool | `T_inside + T_outside` |

   The LSD form degenerates **exactly** to the open diff at `TBR = 1` and
   **exactly** to the spool as `TBR → ∞`. Both limits are unit-tested.
5. the result expressed as `drive_grip_frac` using lapsim's own definition, so
   it drops straight into `LapSimParams` with no change to the sim.

```python
from suspension.dynamics import VehicleParams, VehicleDynamics
from suspension import driveline as dl

veh  = VehicleDynamics(VehicleParams(mass=280.0))
spec = dl.catalog("tre_mk2_center")          # override any field you have data for
r    = dl.axle_traction(veh, spec, dl.ExitCondition(lateral_g_frac=0.70))

r.drive_grip_frac   # -> feed to LapSimParams
r.lock_ratio        # 0 = open, -> 1 = fully locked
r.notes             # conditions where the comparison is meaningless
```

The corner is specified as a **fraction of the car's own lateral limit**, not an
absolute g. An absolute value silently means different things on a grippy car and
a slippery one, and near the limit it pins longitudinal capacity at zero — where
every differential scores identically by construction.

## What `trade.py` does

Runs the real lap sim on each option, scores the times through the FSAE points
curves, and sweeps the parameters the answer is actually sensitive to: `mu`,
torque bias ratio, preload, the lock/yaw penalty coefficient, and which corner
exit you evaluate. Every option sees the **same draw**, so shared nuisance
parameters cancel and what survives is the difference between the parts.

Two gates before a number is printed:

1. **Sign stability** — does the 90% band stay on one side of zero?
2. **Practical floor** — is the median bigger than the difference a real driver
   would bury? (default 5 points; set it from your own logged run-to-run spread)

Fail either and no winner is named and **no $/point is printed**. This mirrors
the hard rule in `worthwhile.py` — refusing to average away a contradiction —
applied to a purchasing decision.

It also runs a **paired candidate-vs-candidate** test, which is usually the
decision that matters. Beating a bad baseline is easy; being separable from the
cheaper option on the shortlist is the actual question.

```python
from suspension import trade, driveline as dl

verdict = trade.compare(
    options=[trade.Option("ATB $2,600", dl.catalog("tre_mk2_center")),
             trade.Option("Spool",      dl.catalog("spool", mass_kg=2.6,
                                                   cost_usd=250.0))],
    baseline=trade.Option("Open", dl.catalog("open", mass_kg=3.4,
                                             cost_usd=400.0)),
    base_params=..., sim_params=..., practical_floor_points=5.0)

verdict.verdict_text
verdict.pairwise[("ATB $2,600", "Spool")]["separable"]
```

## Three currencies, never blended

* **points** — from the sim, with a band
* **dollars** — purchase *plus the parts the listing omits*
* **team hours** — `extra_fab_hours`. "No sprocket adapter" is not a discount,
  it is a work order.

Nothing converts hours into points. If your team is time-limited rather than
cash-limited, that column decides, and only you know which constraint binds.
The FSAE Cost event is the one place dollars become points — and
`cost_event_points()` returns `None` unless you supply the year's Cmin/Cmax,
because inventing the scale would invent the answer.

## Run the worked example

```bash
python demo_diff_worthwhile.py
python -m pytest tests/test_driveline_trade.py -q
```

## Known limitations — read before quoting a number

* Friction **circle**, not a measured combined-slip ellipse.
* Lateral force is shared between the rear tyres in proportion to capacity.
* One corner-exit condition is compressed into a single sim scalar.
* The lock/yaw penalty is a **lap-averaged coefficient**, not a solved yaw
  balance. Its band deliberately starts at zero so "no penalty at all" is inside
  the space explored.
* `CATALOG["tre_mk2_*"]` carries the vendor's published mass and price. **The
  torque bias ratio and preload are class estimates, not TRE figures** —
  override them from the TRE ATB information PDF before quoting anything.
* Nothing here models tyre scrub in tight radii, driver workload, mid-corner
  turn-in behaviour, or endurance tyre wear. Those are exactly the grounds a
  locking diff is usually chosen on, and the tool says so rather than pretending
  its silence is a verdict.

## Bug found in the existing code

`worthwhile._score_against_reference` scores both cars against the paper car's
times. `lapsim.event_points` clamps at maximum for any time *faster* than
`best_time`, so a reconciled car that comes in **lighter** than the paper
baseline scores exactly 0 delta rather than a gain — the comparison can only
ever return ≤ 0. It is harmless in `worthwhile.py`'s intended use (reconciled
cars are heavier), but it silently caps any good news.

`trade._field_reference` avoids this by taking per-event `Tmin` as the fastest
option in the field, which is also what FSAE itself does. The same one-line fix
would work in `worthwhile.py` — left unmade here so as not to change tested
behaviour without a decision.
