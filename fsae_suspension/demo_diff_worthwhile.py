#!/usr/bin/env python3
# ============================================================================
#  KinematiK — demo: is the TRE Mk2 Quaife ATB chain-drive diff worth buying?
#
#  Evaluates the actual listed part (Center Drive, NO sprocket adapter,
#  $2,600 / $3,025 with the Mk2.5 external preload adjuster, 8.9 lb) against an
#  open diff and a spool, using the new driveline + trade modules.
#
#  Run:  python demo_diff_worthwhile.py
# ============================================================================

from suspension.dynamics import VehicleParams
from suspension import lapsim
from suspension import driveline as dl
from suspension import trade


def main():
    # ---- the car everything is evaluated on ---------------------------- #
    # mass here EXCLUDES the differential; each option adds its own.
    base = VehicleParams(
        mass=276.0,            # car + driver, without the diff
        cg_height=290.0,
        wheelbase=1550.0, track_front=1200.0, track_rear=1180.0,
        weight_dist_front=0.46,
        roll_stiffness_front=340.0, roll_stiffness_rear=300.0,
    )
    sim = lapsim.LapSimParams(power_w=55_000.0, mass=base.mass,
                              cl_a=2.2, cd_a=1.1, brake_g=1.6)
    cond = dl.ExitCondition(lateral_g_frac=0.70, speed_ms=14.0,
                            r_wheel_m=0.225, cl_a=sim.cl_a)

    # ---- the options ---------------------------------------------------- #
    # An open diff still needs a housing, chain drive and sprocket carrier —
    # that is what mass_offset_kg and the fabrication fields carry.
    open_diff = trade.Option(
        label="Open diff (team-built)",
        spec=dl.catalog("open", mass_kg=3.4, cost_usd=400.0,
                        extra_fab_hours=60.0, extra_parts_usd=350.0,
                        sprocket_adapter_included=False,
                        source="ESTIMATE — replace with your own build data."),
        notes="Cheapest in dollars, most expensive in hours and risk.")

    spool = trade.Option(
        label="Spool (solid axle)",
        spec=dl.catalog("spool", mass_kg=2.6, cost_usd=250.0,
                        extra_fab_hours=45.0, extra_parts_usd=300.0,
                        source="ESTIMATE — replace with your own build data."),
        notes="Best pure traction number; pays for it in yaw on corner exit.")

    tre = trade.Option(
        label="TRE Mk2 ATB — Center Drive, no adapter ($2,600)",
        spec=dl.catalog("tre_mk2_center"),
        notes="Vendor mass and price; TBR and preload are CLASS ESTIMATES.")

    tre_adj = trade.Option(
        label="TRE Mk2 ATB + Mk2.5 external adjuster ($3,025)",
        spec=dl.catalog("tre_mk2_center_adj"),
        notes="Same physics at nominal preload — the value is tunability, "
              "which a fixed-condition lap sim cannot see.")

    # ---- the trade ------------------------------------------------------ #
    verdict = trade.compare(
        options=[tre, tre_adj, spool], baseline=open_diff,
        base_params=base, sim_params=sim, cond=cond,
        unc=trade.Uncertainty(n_draws=300))

    print("=" * 78)
    print("  DIFFERENTIAL PURCHASE TRADE — baseline:", verdict.baseline)
    print("=" * 78)
    for r in verdict.results:
        print(f"\n{r.label}")
        if r.delta_points_median is None:
            print("   no usable result")
            continue
        print(f"   Δ points vs baseline : {r.delta_points_median:+6.1f}   "
              f"(90% band {r.delta_points_p05:+.1f} … {r.delta_points_p95:+.1f})")
        print(f"   sign stable          : {'YES' if r.sign_stable else 'NO — not separable'}")
        acq = f"${r.acquisition_usd:,.0f}" if r.acquisition_usd else "n/a"
        upp = f"${r.usd_per_point:,.0f}" if r.usd_per_point else "WITHHELD"
        print(f"   acquisition          : {acq}")
        print(f"   team fabrication     : {r.fab_hours:.0f} h")
        print(f"   $ per point          : {upp}")
        n = r.nominal
        if n:
            print(f"   nominal drive_grip_frac {n['drive_grip_frac']:.3f}   "
                  f"lock ratio {n['lock_ratio']:.2f}   "
                  f"rear Fz in/out {n['fz_inside_n']:.0f}/{n['fz_outside_n']:.0f} N")
        for note in r.notes:
            print(f"   ! {note}")

    print("\n" + "-" * 78)
    print("VERDICT")
    print("-" * 78)
    print(verdict.verdict_text)
    print(f"\nModel resolution floor across the sweep: "
          f"{verdict.resolution_points:.1f} points.")

    # ---- the Cost event, honestly --------------------------------------- #
    d = trade.cost_event_delta(car_cost_usd=22_000.0,
                               part_delta_usd=2_600.0 - 400.0,
                               min_cost_usd=None, max_cost_usd=None)
    print("\nFSAE Cost event impact:",
          "WITHHELD — pass this year's Cmin/Cmax to score it."
          if d is None else f"{d:+.2f} points")

    print("\nHARD RULE:", trade.PROVENANCE["hard_rule"])


if __name__ == "__main__":
    main()
