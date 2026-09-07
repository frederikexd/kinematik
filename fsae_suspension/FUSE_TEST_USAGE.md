# Fuse time-current coordination & bench test — usage

`suspension/fuse_test.py`, surfaced as the **⏱ Time-current & bench test** tab
inside the existing Fusebox panel (`ui/fusebox.py`).

---

## Why this exists

`fusebox.py` models an overload path as a series race in one load multiplier:
each element gets a failure current with a mean and a spread, and the audit
reports which one probably goes first. That is the right model for a slow
overload and it has **no clock**:

```
grep -ri "i2t\|time.current\|blow\|melting" suspension/*.py     # nothing
```

A dead short is not decided by current magnitude. Both the fuse and the wire
survive enormous currents for short enough times; what settles it is the
**energy** each absorbs first. So the toolkit could tell you the fuse was rated
below the wire and still not tell you the wire would cook first at 400 A.

The second half of the module exists because the bench rig that measures a real
fuse curve is easy to build and easy to build *wrong*.

---

## The wire curve is derived, not looked up

IEC 60949 adiabatic: `I²t = k²S²`, with

```
k = sqrt( qc·(β+20)/ρ₂₀ · ln((β + t_final) / (β + t_initial)) )
```

The derivation reproduces the published IEC 60364-5-54 table 43.1 values to
within a rounding digit — copper PVC **114.8** vs 115, copper PVC-90 **99.7** vs
100, copper XLPE **142.9** vs 143, copper rubber **140.7** vs 141, aluminium PVC
**76.1** vs 76. All five are pinned in the tests.

Deriving beats looking up because automotive insulation (TXL/GXL, SXL, PTFE)
isn't in the standard's table, but its datasheet does state the two temperatures
the formula needs. So those get a real k instead of a nearest-neighbour guess.

---

## The fuse curve is declared, never invented

You supply two anchor points read off the datasheet's log-log plot, and the
module fits `t = a·(I/I_rated)^-b` through them. A fuse with no anchors gets no
curve, and `coordinate()` **refuses** rather than estimating — a protected range
computed from an invented characteristic is indistinguishable in presentation
from a real one.

### The short-circuit region is energy-limited, not extrapolated

This is the correction that matters most. A blade fuse fitted between 2× and 10×
commonly returns **b ≈ 3.9**, and extrapolating `I^-3.9` out to a dead-short
current predicts clearing times orders of magnitude shorter than the fuse can
deliver — an error in the **dangerous** direction, because it makes the
protection look fast enough when it isn't.

Above the fastest declared anchor the model switches to constant I²t (`t ∝ I^-2`),
which is what a fuse physically does once the element melts faster than heat can
leave it. On a worked example the reported margin at 600 A fell from a fictional
**1530×** to a realistic **116×**.

---

## Worked example

15 A blade fuse on 18 AWG TXL, accumulator capable of ~900 A:

```python
from suspension import fuse_test as ft

wire = ft.WireSpec(label="18 AWG TXL", awg=18, insulation="TXL / GXL 125")
fuse = ft.FuseSpec(label="15 A blade", rating_a=15.0,
                   anchors=[ft.CurveAnchor(2.0, 5.0),      # off the datasheet
                            ft.CurveAnchor(10.0, 0.01)])

r = ft.coordinate(fuse, wire, prospective_fault_a=900.0)
```

```
OK      Above 150 A the fuse is energy-limited, so both curves fall as I^-2
        and never cross again: 225 A²s let-through against 10,293 A²s
        withstand. The wire survives any fault current, however large.
OK      At 900 A the fuse clears in 0.3 ms, 46x inside the wire's 12.7 ms limit.
WARN    The 19 A crossover lies outside the 2x-10x range your anchors cover.
```

---

## The rig

The proof-of-concept rig — operator watches, presses a key, `millis()` times it
— has a **281 ms** detection path and can resolve nothing shorter than **800 ms**
to 10%. Against a four-level test plan:

| Rig | Resolves | Levels usable |
|---|---|---|
| keypress + `millis()` | ≥ 800 ms | **2 of 4** |
| shunt + ADC threshold + `micros()` | ≥ 0.5 ms | **4 of 4** |

The two it loses are the 5× and 10× points — the short-circuit end, which is the
only part of the curve that decides whether the harness survives. A rig that can
only take the slow points measures the region that matters least.

`emit_arduino_sketch(plan)` generates replacement firmware with the four faults
designed out: threshold detection off a shunt instead of a keypress, `micros()`
instead of `millis()`, no blocking serial between the marks, and on-rig
median/spread statistics so nobody transcribes a single reading as the answer.
The ADC threshold is derived from the shunt value and amplifier gain, so the
constant in the sketch corresponds to a stated current rather than to a number
someone tuned until it looked right.

---

## Destructive tests need a budget — sized on your parts, not a guess

`samples_needed()` inverts the log-normal confidence interval on a median. The
count is **quadratic** in the unit-to-unit scatter, which makes that scatter the
single most load-bearing number in the plan — so the module refuses to leave it
assumed.

`PRIOR_LOG_SCATTER = 0.30` is a placeholder and is labelled as one. A plan built
on it emits a `MISSING` finding saying exactly that, and quotes what the count
becomes if your parts scatter half again as widely.

**The two-stage fix.** Run a pilot at one current, and the module measures the
scatter from your own fuses:

```python
pilot = [ft.Measurement(30.0, measured_times_s)]
r = ft.refine_from_pilot(pilot)
plan = ft.build_test_plan(fuse, rig, log_scatter=r["scatter"])
plan.sized_on_measurement()      # True
```

`fit_measurements()` also returns a `.scatter` that drops straight back into
`build_test_plan()`, so each session sizes the next one.

| Pilot | Result |
|---|---|
| 9 fuses, tight parts (σ≈0.12) | **5** per level, not 11 — six fewer per level |
| 9 fuses, prior-like (σ≈0.19) | 13 per level |
| 9 fuses, wide parts (σ≈0.58) | ⚠ far more than 11 — the prior would have under-sized it |

**Sizing uses the upper confidence bound on σ, not the point estimate**, because
σ enters squared: a pilot that happened to come out tight would otherwise licence
a test too small to deliver the precision it promised. The bound uses a
Wilson–Hilferty χ² approximation, verified against published quantiles (−0.22% at
10 dof, erring low and therefore conservative below that).

**And it refuses when the pilot is too thin.** A 3-fuse pilot gives σ to only
±50%, and the rigorous upper bound then demands *hundreds* per level — correct
statistics, useless advice. Below 8 degrees of freedom the module says so, falls
back to the point estimate, and tells you how many more fuses reach a pilot worth
acting on (`recommended_pilot_n()` → 9).

---

## The hard rule

A measurement whose uncertainty is dominated by the instrument is **rejected**
from the fit, not averaged into it. A rig with 250 ms of detection latency
reporting a 47 ms blow time has measured its operator, and folding such readings
into a regression launders them into something shaped like a curve.

```python
ft.fit_measurements(measurements, fuse, ft.Instrument())
# FAIL  150 A: median 10.0 ms against instrument uncertainty of ±80.0 ms
#       (800%). Rejected from the fit — this reading is the rig, not the fuse.
```

`fit_measurements` also flags parts that come back **out of family** against the
declared curve. Slower than published invalidates every coordination result
computed from that curve; counterfeit or mislabelled blade fuses are common
enough that this is the expected explanation, not an exotic one.

---

## Tests

`tests/test_fuse_test.py` — 76 tests, including the five IEC anchors, the
continuity of the power-law→I²t handover, and the refusal paths.

```bash
python -m pytest tests/test_fuse_test.py -q
```
