# KinematiK against NREL FASTSim

FASTSim (Future Automotive Systems Technology Simulator) is the U.S. Department of Energy's vehicle energy model,
written and maintained by the National Renewable Energy Laboratory (NREL), free and open source (Apache 2.0), and
validated by NREL against Argonne dynamometer data. This benchmark is the second of three tools, alongside the
GT-SUITE papers ([GT_SUITE_BENCHMARK.md](GT_SUITE_BENCHMARK.md)) and Simulink.

The GT-SUITE papers leave inputs out (motor maps, BSFC maps, rolling resistance), so those comparisons carry
assumption bands. FASTSim states every input, so here there is nothing to assume: KinematiK is given exactly
FASTSim's vehicle and exactly FASTSim's speed trace, and the question is only whether the physics agree.

```
python scripts/fastsim_benchmark.py            # grade; FASTSim itself is not needed, its runs are stored
pip install fastsim==3.1.0
python scripts/fastsim_benchmark.py extract    # rerun FASTSim and rewrite benchmarks/fastsim
```

## What is compared

| | FASTSim 3.1.0 | KinematiK |
|---|---|---|
| Vehicles | Renault Zoe ZE50, Nissan Leaf 30 kWh, Tesla Model 3 RWD (EV); Ford Fusion 2012, Chrysler Pacifica 2026 (petrol) | the same parameters, read from FASTSim's vehicle files |
| Cycles | UDDS, HWFET, US06, NEDC (the GT-SUITE benchmark's pinned cycle files) | FASTSim's achieved speed on each |
| EV quantity | energy at the battery terminals, Wh/km | `ev_electrical_check.traction_power_w` with FASTSim's motor map (`motor_eff_map`), gearing efficiency, regeneration cap and auxiliary load |
| ICE quantity | fuel energy, Wh/km | `ice_powertrain.fuel_for_lap` with FASTSim's engine map (`PowerEfficiencyMap`), gearing efficiency and auxiliary load (`aux_power_w`); standing still, the engine idles driving the auxiliaries |

Graded: battery or fuel energy per km, within the 5 % tolerance of the GT-SUITE benchmark. Reported: tractive
energy alone (road load: mass, rolling resistance, drag, wheel inertia), and for the petrol cars a Willans line fitted
to FASTSim's map with `fit_willans`.

## Results

20 of 20 within 5 %. Mean absolute error 0.02 %, bias +0.02 %, worst +0.04 %.

| Vehicle | Cycle | Tractive, KinematiK vs FASTSim | Energy, KinematiK | FASTSim | Error |
|---|---|---|---|---|---|
| Zoe | UDDS / HWFET / US06 / NEDC | +0.02 / +0.03 / +0.02 / +0.03 % | 113.2 / 136.3 / 189.9 / 127.9 | 113.2 / 136.3 / 189.9 / 127.9 Wh/km | +0.04 / +0.03 / +0.02 / +0.03 % |
| Leaf | UDDS / HWFET / US06 / NEDC | +0.02 / +0.03 / +0.01 / +0.02 % | 99.7 / 127.2 / 175.1 / 116.1 | 99.6 / 127.2 / 175.0 / 116.1 Wh/km | +0.04 / +0.03 / +0.02 / +0.03 % |
| Model 3 | UDDS / HWFET / US06 / NEDC | +0.02 / +0.03 / +0.02 / +0.03 % | 91.2 / 97.9 / 133.3 / 97.2 | 91.1 / 97.8 / 133.3 / 97.1 Wh/km | +0.04 / +0.04 / +0.02 / +0.04 % |
| Fusion | UDDS / HWFET / US06 / NEDC | +0.02 / +0.03 / +0.01 / +0.02 % | 609.1 / 445.8 / 686.0 / 599.9 | 609.1 / 445.7 / 686.0 / 599.9 Wh/km | −0.00 / +0.02 / +0.01 / +0.01 % |
| Pacifica | UDDS / HWFET / US06 / NEDC | +0.02 / +0.03 / +0.01 / +0.02 % | 941.9 / 610.7 / 981.8 / 910.0 | 942.0 / 610.6 / 981.7 / 909.9 Wh/km | −0.01 / +0.02 / +0.01 / +0.01 % |

The residual, a few hundredths of a percent, is already there in the tractive energy, before any motor or engine
model: it comes from how each tool steps along the speed trace, and it is the same size on every vehicle and cycle.

### What the Willans line costs (reported, not graded)

A Willans line fitted to FASTSim's engine map with `fit_willans` (least squares over the map's points, at one engine
speed) gives −26.1 / −5.8 / +3.9 / −25.3 % on the Fusion and −22.1 / −4.5 / +0.0 / −22.4 % on the Pacifica
(UDDS / HWFET / US06 / NEDC). The line follows the map at high load and misses it at light load, where FASTSim's
generic map falls to 10 % efficiency or less. US06 and the highway cycle, the ones nearest a race lap, are within 6 %;
the stop-and-go city cycles are not. With an engine map (`PowerEfficiencyMap`) the error is 0.02 % or less on every
cycle. For a Formula Student engine, a dyno sweep fitted with `fit_willans` remains the recommended route (it was
graded against Argonne's measured Blazer at 3.8 % mean error, GT_SUITE_BENCHMARK.md); where a full BSFC table is
available, `PowerEfficiencyMap` uses it as measured.

## How each vehicle is set up

- **Zoe, Fusion:** as bundled.
- **Leaf, Model 3:** FASTSim's cabin, HVAC and battery thermal models are switched off. KinematiK has no cabin model,
  and with HVAC on the comparison would be about the climate control, not the powertrain. The 250 W base auxiliary
  load stays on both sides.
- **Pacifica:** stop-start and deceleration fuel cut-off are switched off: a race engine runs throughout. With them on,
  FASTSim's UDDS fuel falls 10 % (848 against 942 Wh/km), which is the stop-start, not the engine.
- **Chevrolet Bolt EV:** left out. Its battery efficiency is tabulated against temperature, so FASTSim cannot run it
  with the battery thermal model off.

## Facts about FASTSim 3.1.0 this depends on

These were found by reproducing FASTSim's own step-by-step output, and `extract` checks the second on every run:

1. Motor and engine efficiency are read against output power as a fraction of the rating, linearly (Zoe, Fusion,
   Pacifica) or held from each point to the next (Leaf, Model 3: "Step-Lower"). KinematiK reads tables linearly, so
   a step map is passed in as a staircase whose steps end a hair before the next point; the test suite checks that
   the staircase reads exactly like the step map.
2. Regeneration uses the motor's efficiency at zero load, whatever the regenerated power. KinematiK's `regen_eff`
   is therefore the gearing efficiency times that value; the regeneration cap is the motor rating at the motor
   shaft.
3. The engine burns `max(P / eta(P), idle fuel)` while it runs; idle fuel is 0 W on the Fusion and 13 kW on the
   Pacifica. `PowerEfficiencyMap` has the same floor (`idle_fuel_w`).
4. Air density is FASTSim's constant 1.1728 kg/m³, and wheel inertia enters as `1 + n J / (r² m)`.

## Provenance

- FASTSim 3.1.0, pinned: `extract` refuses any other version. Each vehicle's parameters, as FASTSim states them, are
  in `benchmarks/fastsim/fastsim_runs.json` with the SHA-256 of FASTSim's own vehicle record.
- FASTSim's achieved speed on every run is stored (`benchmarks/fastsim/<vehicle>_<cycle>.csv`), so grading needs no
  FASTSim install. Where FASTSim could not follow the trace (US06 on the Fusion, Pacifica and Leaf, by up to 0.9 m/s),
  KinematiK is given the speed FASTSim actually drove.
- Every stored file and the four cycle files are listed with their SHA-256 in `benchmarks/fastsim/MANIFEST.sha256`;
  the script refuses to grade when one no longer matches.
- Tests: `tests/test_fastsim_benchmark.py` (manifest, every row within tolerance, mean error under 0.2 %, the road-load
  layer within 0.1 %, the staircase conversion).

FASTSim: National Renewable Energy Laboratory, FASTSim, https://github.com/NREL/fastsim, Apache License 2.0. The
stored runs are FASTSim output.

## What this benchmark does not cover

FASTSim is an energy model, so this benchmark checks energy. It does not check launches or lap times (KinematiK's
`simulate_launch` is graded against Argonne's measured i3 launches in GT_SUITE_BENCHMARK.md), battery temperature,
or anything at Formula Student scale: FASTSim's vehicles are road cars. A Formula Student-sized vehicle file run
through FASTSim would be a code-to-code check at the right scale, without measured data behind either side.
