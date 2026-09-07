# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================

"""
demo_pedal_box.py — the pedal box that does not fit, worked end to end.

This is the Summer 2026 Meeting #5 problem, in software:

    "We need to shorten the blue distance. We also need to account for the lines
     coming out the rear of the MCs. Distance needed to be cut is 50-100 mm
     (max 4 in). Can shorten pushrod but not nearly enough. Any ideas?"

The answer is not one idea, it is a priced menu -- and the reason the question is
hard is that the obvious fixes are paid for somewhere else. So this demo runs the
three coupled checks together:

    1. STACK-UP     where the installed length actually goes, segment by segment,
                    including the line exit behind the cylinders that a CAD
                    envelope check routinely omits.
    2. THE MENU     every lever that buys X-length, each with the millimetres it
                    returns and what it costs in pedal effort or pedal travel,
                    then the cheapest combination that clears the deficit.
    3. THE BILL     what those fixes do to balance-bar bias authority and to
                    pedal travel -- because a shorter box with a pedal on the
                    floor, or a bias the bar cannot reach, is not a solution.

Run:  python demo_pedal_box.py
"""

from suspension import pedal_box as pb


def rule(title):
    print("\n" + "=" * 78)
    print(f"  {title}")
    print("=" * 78)


# --------------------------------------------------------------------------- #
#  The car, as the deck describes it: dual master cylinders on a balance bar,
#  220 mm front rotors, and roughly 290 mm of bay to live in.
# --------------------------------------------------------------------------- #
AVAILABLE_MM = 290.0
PEDAL_RATIO = 5.0
PEDAL_LEVER_MM = 90.0
PEDAL_FORCE_N = 500.0
TARGET_BIAS = 0.65

front = pb.CircuitSpec(mc_bore_mm=15.875, caliper_piston_dia_mm=30.0,
                       pistons_per_side=2, opposed=True, pad_mu=0.45,
                       rotor_dia_mm=220.0, n_corners=2)
rear = pb.CircuitSpec(mc_bore_mm=17.5, caliper_piston_dia_mm=25.0,
                      pistons_per_side=1, opposed=True, pad_mu=0.45,
                      rotor_dia_mm=200.0, n_corners=2)


# =========================================================================== #
rule("1. STACK-UP — where is the length?")

stack = pb.stack_up(available_mm=AVAILABLE_MM,
                    pedal_lever_mm=PEDAL_LEVER_MM,
                    pedal_ratio=PEDAL_RATIO,
                    pedal_rest_angle_deg=25.0,
                    mc_family="standard racing",
                    mc_outlet="straight fitting + hardline bend",
                    pushrod_mm=55.0)
print(stack.summary())

print("\nFindings:")
for f in stack.findings:
    print(f"  [{f.severity.value.upper():7s}] {f.message}")


# =========================================================================== #
rule(f"2. THE MENU — {stack.deficit_mm:.0f} mm to find, every lever priced")

opts = pb.shorten_options(stack, pedal_lever_mm=PEDAL_LEVER_MM,
                          pedal_ratio=PEDAL_RATIO, mc_bore_mm=front.mc_bore_mm)

print("\nEverything available, cheapest first:\n")
rank_label = {0: "free adjustment", 1: "parts, no penalty",
              2: "trades force/travel", 3: "re-layout"}
for o in opts:
    flag = "" if o.feasible else "   [NOT FEASIBLE]"
    print(f"  {o.gain_mm:+6.1f} mm  [{rank_label[o.cost_rank]:>19s}]  "
          f"{o.name}{flag}")

print("\n\nCheapest combination that clears the deficit "
      "(nothing above 'trades force/travel'):\n")
plan = pb.plan_shortening(stack, opts, max_cost_rank=2)
print(plan.summary())

print("\nFindings:")
for f in plan.findings:
    print(f"  [{f.severity.value.upper():7s}] {f.message}")


# =========================================================================== #
rule("3a. THE BILL — can the bar still reach the bias?")

auth = pb.bias_authority(pedal_force_N=PEDAL_FORCE_N, pedal_ratio=PEDAL_RATIO,
                         front=front, rear=rear, bar_length_mm=60.0,
                         target_bias=TARGET_BIAS)
print(auth.summary())
print("\nFindings:")
for f in auth.findings:
    print(f"  [{f.severity.value.upper():7s}] {f.message}")

# What a bore change does to that -- the honest fix when the bar runs out of trim.
print("\nRe-centring the bar by changing the FRONT bore:\n")
print(f"  {'front bore':>12s}  {'band':>16s}  {'centred':>9s}  "
      f"{'offset for 65%':>15s}")
for bore in (14.3, 15.875, 17.5, 19.05, 20.6):
    f2 = pb.CircuitSpec(**{**front.__dict__, "mc_bore_mm": bore})
    a2 = pb.bias_authority(pedal_force_N=PEDAL_FORCE_N, pedal_ratio=PEDAL_RATIO,
                           front=f2, rear=rear, bar_length_mm=60.0,
                           target_bias=TARGET_BIAS)
    off = (f"{a2.offset_for_target_mm:+.2f} mm" if a2.target_reachable
           else "unreachable")
    print(f"  {bore:9.3f} mm  {a2.bias_min*100:6.1f}-{a2.bias_max*100:5.1f}%  "
          f"{a2.bias_at_centre*100:8.1f}%  {off:>15s}")


# =========================================================================== #
rule("3b. THE BILL — does the pedal still stop before the floor?")

bar = pb.balance_bar_bias(pedal_force_N=PEDAL_FORCE_N, pedal_ratio=PEDAL_RATIO,
                          front=front, rear=rear, bar_length_mm=60.0,
                          bar_offset_mm=(auth.offset_for_target_mm
                                         if auth.target_reachable else 0.0))
print(f"Front circuit runs at {bar.pressure_front_bar:.0f} bar, "
      f"rear at {bar.pressure_rear_bar:.0f} bar.\n")

travel = pb.pedal_travel(circuit=front, line_pressure_bar=bar.pressure_front_bar,
                         pedal_ratio=PEDAL_RATIO, hose_length_m=0.6,
                         hardline_length_m=1.8)
print(travel.summary())

print("\nNow price the two shortening levers that cost travel:\n")
print(f"  {'change':>34s}  {'travel at pad':>14s}  {'stroke':>9s}  {'verdict':>8s}")
baseline = travel
print(f"  {'baseline (ratio 5.0, 15.9 mm bore)':>34s}  "
      f"{baseline.pedal_travel_mm:11.0f} mm  "
      f"{baseline.mc_stroke_mm:6.1f} mm  {baseline.verdict:>8s}")

hi_ratio = pb.pedal_travel(circuit=front,
                           line_pressure_bar=bar.pressure_front_bar,
                           pedal_ratio=6.5, hose_length_m=0.6,
                           hardline_length_m=1.8)
print(f"  {'raise pedal ratio to 6.5':>34s}  "
      f"{hi_ratio.pedal_travel_mm:11.0f} mm  "
      f"{hi_ratio.mc_stroke_mm:6.1f} mm  {hi_ratio.verdict:>8s}")

big_bore = pb.CircuitSpec(**{**front.__dict__, "mc_bore_mm": 19.05})
bore_up = pb.pedal_travel(circuit=big_bore,
                          line_pressure_bar=bar.pressure_front_bar,
                          pedal_ratio=PEDAL_RATIO, hose_length_m=0.6,
                          hardline_length_m=1.8)
print(f"  {'go up to a 19.05 mm bore':>34s}  "
      f"{bore_up.pedal_travel_mm:11.0f} mm  "
      f"{bore_up.mc_stroke_mm:6.1f} mm  {bore_up.verdict:>8s}")

print("\n  -> Raising the ratio buys length AND cuts pedal effort, but it is the "
      "\n     most expensive thing you can do to travel. Going up in bore buys "
      "\n     length AND travel, and is paid for in effort. They pull opposite "
      "\n     ways, which is exactly why they have to be chosen together.")


# =========================================================================== #
rule("4. THE COUPLED VERDICT")

study = pb.study(available_mm=AVAILABLE_MM, front=front, rear=rear,
                 pedal_force_N=PEDAL_FORCE_N, pedal_ratio=PEDAL_RATIO,
                 pedal_lever_mm=PEDAL_LEVER_MM, target_bias=TARGET_BIAS)
print(study.summary())

if study.findings:
    print("\nCross-cutting findings — the ones no single check can see:")
    for f in study.findings:
        print(f"  [{f.severity.value.upper():7s}] {f.message}")

print("\n" + "-" * 78)
p = pb.provenance()
print(f"SAFE:        {p['safe']}")
print(f"PROVISIONAL: {p['provisional']}")
print(f"NOTE:        {p['note']}")
