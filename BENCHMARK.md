# Benchmarks

Every benchmark test in the repository, listed by role and by the feature it checks. [VALIDATION.md](VALIDATION.md)
summarises what the checks found; this file is the index of the tests themselves, so a team lead can see what stands
behind the tools their subteam uses.

A test is listed here only if it compares KinematiK with something outside its own code:

| Kind | Meaning |
|---|---|
| **Another tool** | A separate program solves the same problem from the same inputs |
| **Published** | Published data, a standard's table or a manufacturer's documented value |
| **Closed form** | A textbook formula or exact solution the numerical result must reproduce |
| **Known answer** | A synthetic input with a planted answer the method must recover |
| **Cross-check** | Two methods in the repository that share no solving code must agree (a linearisation against the full solver, two file parsers) |

Robustness, determinism and "never raises" tests are not benchmarks and are left out, and so are the reproducibility
re-runs and the recomputation of the paper's 499 numbers, which VALIDATION.md covers. A feature with nothing in the
five kinds says so: it has been checked against its own tests only. Paths are relative to `fsae_suspension/`; test
names are `tests/<file>.py::<test>`. Run any of them with `pytest <path>`.

A tab that several roles use is listed in full under the first role that uses it and referred to after that.

**Roles:** [Shared by every role](#shared-by-every-role) · [Suspension / Dynamics](#suspension--dynamics) ·
[Aerodynamics](#aerodynamics) · [Powertrain / Drivetrain](#powertrain--drivetrain) · [Electrics](#electrics) ·
[Cooling](#cooling) · [Data Acquisition](#data-acquisition) · [Brakes](#brakes) · [Chassis / Frame](#chassis--frame) ·
[Cost / Business](#cost--business) · [Power-user tabs](#power-user-tabs)

## Shared by every role

3D Model, Integration, Frames & Datums, Registry, Documentation, Lead Notes, Weight & Handover, Validation, Proof
Planner, Saboteur, Phantom Car, Earshot, Fusebox, Analytics.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Frames & Datums | Frame conversions and rotation senses checked against exact identities (round trips, right-hand rule, SAE J670 vs ISO 8855 signs) | Closed form | `python3 coordinate_frames.py` |
| Validation: strain-gauge rig | Bridge output exact, shunt calibration and its monitor | Closed form | `tests/test_setup_checks.py::test_bridge_exact_and_shunt_monitor` |
| | Thermal fit reproduces its closed form | Closed form | `tests/test_setup_checks.py::test_thermal_fit_matches_the_closed_form` |
| | GUM uncertainty budget reproduces the supplement's S7.4 reference row | Published | `tests/test_setup_checks.py::test_gum_budget_s74_reference_row` |
| | Transducer dynamic error matches the source paper; Euler buckling and shunt values | Published, closed form | `tests/test_transducer.py::test_dynamic_error_matches_the_paper`, `::test_euler_and_shunt` |
| Proof Planner | No benchmark: checked against its own tests only | | `tests/test_proof_engine.py` |
| Saboteur | A deck corrupted with a known mutation is named by its fingerprint | Known answer | `tests/test_saboteur.py::test_fingerprint_names_the_injected_corruption` |
| Phantom Car | Normal-distribution helpers match the standard normal | Closed form | `tests/test_phantom_car.py::test_helper_math_matches_the_standard_normal` |
| Earshot | z-values against textbook tables; run-order drift bias exact; tilt-test angle term by hand | Closed form | `tests/test_earshot.py::test_z_helpers_match_textbook`, `::test_ordering_bias_exact`, `::test_tilt_angle_term_hand_check` |
| Fusebox | Two-element failure order against Φ((μⱼ−μᵢ)/√(σᵢ²+σⱼ²)); fixes solve that formula exactly | Closed form | `tests/test_fusebox.py::test_pairwise_matches_napkin_formula`, `::test_prescriptions_solve_pairwise_exactly` |
| 3D Model, Integration, Registry, Documentation, Lead Notes, Weight & Handover, Analytics | No physics to benchmark: bookkeeping, roll-ups and storage, covered by their own tests | | |

## Suspension / Dynamics

Kinematics, Roll & Load Transfer, Compliance, FlexGen, Tire & Grip, ThermicPatch, SimulForge, Stochastic Inversion,
InverseGenesis, InverseGenesis-FullCar, MorphMesh, OmniCore, Setup Optimiser, Track Testing.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Kinematics | Adams View 2026.1: the paper's three corners, camber, toe, caster, kingpin inclination and wheel-centre path at every 1 mm from −25 to +25 mm. Largest difference 0.000027° and 0.000027 mm against limits of 0.005–0.01° and 0.05 mm set before the run | Another tool | `scripts/adams_view_benchmark.py`, `tests/test_adams_view_benchmark.py`, [benchmarks/adams_view](fsae_suspension/benchmarks/adams_view/README.md) |
| | Lotus Suspension Analysis on two published corners (Arevalo et al. 2018): camber and toe in bump, camber in roll and steer, roll-centre height | Another tool | `scripts/lotus_benchmark.py`, `tests/test_lotus_benchmark.py` |
| | Applied Racing Dynamics on a front and a rear corner, static and ±20 mm | Another tool | Manual comparison, PHYSICS.md §32.4 |
| | Independent rigid-body solve (SciPy `fsolve`, no shared code) on three corners | Another tool | `scripts/independent_kinematics.py`, `tests/test_review_items.py` |
| | Parallel equal-length arms give zero camber gain | Closed form | `tests/test_kinematics.py::test_parallel_equal_arms_zero_camber_gain` |
| | Instant centre against the hand construction | Closed form | `tests/test_accuracy_invariants.py::test_instant_centre_matches_hand_construction_on_flat_pickups` |
| | Anti-dive and anti-squat against a virtual-work derivation | Closed form | `tests/test_accuracy_invariants.py::test_anti_dive_agrees_with_a_virtual_work_derivation`, `::test_anti_squat_agrees_with_a_virtual_work_derivation` |
| | Pushrod force from statics equals the virtual-work result | Closed form | `tests/test_load_case_physics.py::test_pushrod_force_equals_virtual_work` |
| | The generic topology kernel reproduces the double-wishbone solver | Cross-check | `tests/test_topology.py::test_generic_matches_native_double_wishbone` |
| Roll & Load Transfer | Load transfer closes the moment balance, also with split roll centres; wheel loads sum to the car's weight | Closed form | `tests/test_accuracy_invariants.py::test_load_transfer_closes_moment_balance`, `::test_load_transfer_closes_with_split_roll_centres`, `tests/test_kinematics.py::test_load_conservation` |
| Compliance | Cantilever tip stiffness 3EI/L³; two beams in series EA/L; tube section against the annulus formula | Closed form | `tests/test_compliance.py::test_cantilever_tip_stiffness_3EI_over_L3`, `::test_guyan_series_two_beams_axial`, `::test_tube_section_matches_annulus_formula` |
| | Beam element: tip deflection PL³/3EI, axial PL/EA, twist TL/GJ | Closed form | `tests/test_load_case_physics.py::test_beam_element_closed_forms`, `tests/test_accuracy_invariants.py::test_beam_element_reproduces_every_closed_form` |
| FlexGen | Small deflection against Euler–Bernoulli, fixed-free and fixed-guided; buckling against the Euler column | Closed form | `tests/test_flexgen.py::test_small_deflection_matches_euler_bernoulli_fixed_free`, `::test_small_deflection_matches_euler_bernoulli_fixed_guided`, `::test_buckling_matches_euler_column` |
| Tire & Grip | Tire fitter recovers a known Magic Formula tire, also with the sign flipped | Known answer | `tests/test_tiremodel.py::test_fitter_recovers_known_tire`, `::test_fitter_handles_flipped_sign` |
| | Pneumatic trail and carcass share recovered from synthetic measurements | Known answer | `tests/test_measurement_fits.py::test_pneumatic_trail_recovered`, `::test_carcass_share_recovered_and_consistent_with_rate_model` |
| | Lateral equilibrium at the design case | Closed form | `tests/test_tire_utilisation.py::test_lateral_equilibrium_and_near_limit_at_design_case` |
| | Inflation gas heats at constant volume | Closed form | `tests/test_energy_rotor_gas_physics.py::test_inflation_gas_heats_at_constant_volume` |
| | Not benchmarked: the default tire is a synthetic set, not a TTC fit | | |
| ThermicPatch | No benchmark: flagged uncalibrated until fed measured temperatures | | `tests/test_thermic_patch.py` |
| SimulForge | Bus voltage obeys V = V_oc − I·R at every step | Closed form | `tests/test_simulforge.py::test_bus_algebraic_constraint_holds` |
| Stochastic Inversion | Sensitivity matrix predicts a direct solve; linear yield agrees with full nonlinear solves | Cross-check | `tests/test_kinematik_stochastic.py::test_sensitivity_matches_direct_solve`, `::test_full_mode_agrees_with_linear` |
| | A planted weld pull is restored by the alignment prescription | Known answer | `tests/test_kinematik_stochastic.py::test_prescription_restores_injected_error` |
| InverseGenesis | Known-answer recovery: hidden geometries with up to six moved pickups, recovered from their curves | Known answer | `scripts/method_benchmarks.py`, `tests/test_method_benchmarks.py::test_known_answer_recovery_meets_the_curves_not_the_pickups` |
| | Target curves agree with the forward solver | Cross-check | `tests/test_inverse_genesis.py::test_camber_curve_agrees_with_direct_solver` |
| InverseGenesis-FullCar | No benchmark beyond InverseGenesis | | `tests/test_inverse_genesis_fullcar.py` |
| MorphMesh | No benchmark: the plate element's symmetry and rigid-body modes are checked, not a reference solution | | `tests/test_morphmesh.py` |
| OmniCore | Hand-worked Pareto cases: strict domination and a trade-off that keeps both | Known answer | `tests/test_omnicore.py::TestPareto::test_hand_case_strict_domination`, `::test_hand_case_tradeoff_keeps_both` |
| Setup Optimiser | No benchmark: checked against its own tests only | | `tests/test_tiremodel.py::test_optimiser_never_regresses` |
| Track Testing | Skidpad time consistent with its closed form | Closed form | `tests/test_laptime.py::test_skidpad_closed_form_consistency` |
| | Not benchmarked against another lap simulator: the ARD lap-time study is pending | | |

## Aerodynamics

Aerodynamics, Track Testing, Setup Optimiser.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Panel method | Sphere surface pressure against potential flow: Cp minimum −1.248 against −1.250 | Closed form | `scripts/aero_closed_form.py`, `tests/test_aero_closed_form.py::test_sphere_surface_pressure_matches_potential_flow` |
| Vortex lattice | Rectangular AR 8 wing against lifting-line theory and an independent lattice; grid convergence. Open: reads 4% under the independent lattice | Closed form, another tool | `tests/test_aero_closed_form.py::test_wing_lift_against_lifting_line_and_an_independent_lattice`, `::test_wing_lift_is_grid_converged` |
| Air properties | Kinematic viscosity follows the standard table | Published | `tests/test_accuracy_invariants.py::test_air_kinematic_viscosity_tracks_the_standard_table` |
| Scale model | Reynolds number by the textbook formula; scaled geometry recovers full size exactly | Closed form | `tests/test_scale_model.py::test_reynolds_is_textbook`, `::test_scale_geometry_recovers_full_size_exactly` |
| Pressure taps | Volts to Cp exact; a known offset gives that RMSE | Known answer | `tests/test_pressure_tap.py::test_volts_to_cp_is_exact`, `::test_known_offset_gives_that_rmse` |
| PIV | Cross-correlation recovers a known velocity; sub-pixel fit recovers a known offset | Known answer | `tests/test_piv.py::test_cross_correlation_recovers_known_velocity`, `tests/test_accuracy_invariants.py::test_piv_subpixel_fit_recovers_a_known_offset` |
| Virtual wind tunnel | Ride heights to attitude and back is exact | Closed form | `tests/test_windtunnel.py::test_ride_heights_to_attitude_roundtrip_is_exact` |
| Plug builder | Section area against the closed-form superellipse area | Closed form | `tests/test_accuracy_invariants.py::test_plug_builder_section_area_matches_the_analytic_superellipse` |
| | Not benchmarked: no wind tunnel, CFD or track correlation. Both solvers are inviscid; compare designs, do not quote absolute downforce | | |
| Track Testing, Setup Optimiser | See [Suspension / Dynamics](#suspension--dynamics) | | |

## Powertrain / Drivetrain

Powertrain (EV / ICE switch), Data Acquisition, InverseGenesis-FullCar, Track Testing, Setup
Optimiser, DFMEA.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| EV: energy | Pack current at constant speed matches the force balance; cruise power is (rolling + aero force) × speed | Closed form | `tests/test_energy_rotor_gas_physics.py::test_pack_current_at_constant_speed_matches_the_force_balance`, `tests/test_power_draw.py::test_steady_cruise_power_is_force_times_velocity` |
| | Pack current does not depend on gearing; wheel-speed constant against the unit conversion | Closed form | `tests/test_power_draw.py::test_pack_current_does_not_depend_on_gearing`, `::test_wheel_rpm_constant_matches_mph_conversion` |
| Torque distribution | Force and yaw balance hold; an open differential splits equally and a limited-slip one respects its bias ratio | Closed form | `tests/test_torque_distribution.py::test_force_and_yaw_balance_hold`, `::test_open_split_is_equal_and_lsd_respects_its_bias_ratio` |
| Throttle | Compressible flow through the plate against the textbook choked form | Closed form | `tests/test_accuracy_invariants.py::test_throttle_compressible_flow_matches_the_textbook_choked_form` |
| | Not benchmarked: no dyno or measured energy trace | | |
| ICE: intake restrictor | Choked mass flow against the isentropic closed form (0.0745 kg/s through 20 mm at Cd 1); flow scales with throat area | Closed form | `tests/test_ice_powertrain.py::test_choked_flow_matches_the_isentropic_closed_form`, `::test_choked_flow_scales_with_throat_area` |
| | Power ceiling is air ÷ stoichiometric AFR × LHV × BTE; a capped curve never exceeds it at any rpm | Closed form | `tests/test_ice_powertrain.py::test_power_ceiling_is_air_over_afr_times_lhv_times_bte`, `::test_restricted_curve_never_exceeds_the_ceiling` |
| | Restrictor diameters and CO2 factors are the rule values | Published | `tests/test_ice_powertrain.py::test_rule_diameters_and_co2_factors` |
| ICE: gearbox | Upshift speed on a linear torque curve against v* = a / (b c (G_g + G_g+1)); a flat curve runs each gear to redline | Closed form | `tests/test_ice_powertrain.py::test_shift_speed_matches_the_closed_form_on_a_linear_torque_curve`, `::test_flat_torque_runs_each_gear_to_redline` |
| ICE: launch and 75 m | With no losses, the 75 m time is √(2x/a) for a = T G / (r m); rotating inertia reflects through G²; shifts land on the gearbox's shift speeds | Closed form | `tests/test_ice_powertrain.py::test_launch_matches_constant_acceleration_with_no_losses`, `::test_rotating_inertia_reflects_through_the_square_of_the_ratio`, `::test_launch_shifts_where_the_gearbox_says_and_finishes` |
| ICE: fuel and CO2 | Fuel at constant speed from the force balance; coasting burns only the overrun flow; endurance fuel and tank margin | Closed form | `tests/test_ice_powertrain.py::test_fuel_at_constant_speed_matches_the_force_balance`, `::test_coasting_burns_only_the_overrun_flow`, `::test_endurance_fuel_and_tank_margin` |
| ICE: engine braking | Overrun force T G / r and the effective brake balance | Closed form | `tests/test_ice_powertrain.py::test_engine_braking_force_and_balance` |
| | Not benchmarked: the torque curve is declared (no engine model), BTE is one number for the whole map, and nothing has been run against GT-POWER, OptimumLap or a dyno yet ([plan](docs/POWERTRAIN_BENCHMARK_PLAN.md)) | | |
| Data Acquisition | See [Data Acquisition](#data-acquisition) | | |
| InverseGenesis-FullCar, Track Testing, Setup Optimiser | See [Suspension / Dynamics](#suspension--dynamics) | | |
| DFMEA | No physics to benchmark: risk-priority arithmetic, covered by its own tests | | `tests/test_dfmea.py` |

## Electrics

Accumulator, Powertrain, Track Testing, Electronics (PCB), Data Acquisition, Tractive Safety, DFMEA, SimulForge,
OmniCore.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Accumulator: BMS link | BQ796xx frames, addressing, cell LSB and the balancing frame against TI's documentation | Published | `tests/test_bq796xx.py::test_broadcast_write_matches_documentation`, `::test_reverse_addressing_matches_ti_section_9_2`, `::test_cell_lsb_matches_documentation`, `::test_multibyte_write_matches_the_published_balancing_frame` |
| Accumulator: pack thermal | No benchmark: flagged synthesized until calibrated against a measured pack | | `tests/test_pack_thermal.py` |
| Electronics (PCB) | Wire fusing constant k against the published IEC adiabatic values; Onderdonk in circular mils | Published | `tests/test_accuracy_invariants.py::test_iec_adiabatic_k_matches_published_values`, `::test_onderdonk_uses_circular_mils_not_square_mils` |
| | Ampacity derating reproduces every published NEC correction factor and the THHN table | Published | `tests/test_express.py::test_the_derivation_reproduces_every_published_nec_factor`, `::test_thhn_at_the_nec_basis_recovers_the_table_exactly`, `tests/test_accuracy_invariants.py::test_ampacity_correction_never_errs_optimistic` |
| | Fuse-curve fit is exact on a power law | Known answer | `tests/test_accuracy_invariants.py::test_fuse_curve_power_law_fit_is_exact_on_a_power_law` |
| | PCB Doctor: the same board in KiCad and Altium formats gives one diagnosis | Cross-check | `tests/test_pcb_doctor.py::TestAltiumParser::test_same_board_two_formats_one_diagnosis` |
| | Not benchmarked: trace temperature rise is checked for its trend, not against the IPC-2152 charts | | `tests/test_electronics.py::test_ipc_temp_rise_increases_with_current` |
| Harness | AWG areas against the gauge table; copper mass exact; clearance exact | Published, closed form | `tests/test_harness.py::test_awg_area_known_and_interpolated`, `::test_copper_mass_exact`, `::test_clearance_is_exact_not_sampled` |
| Tractive Safety | Precharge time constant and energy against the RC solution | Closed form | `tests/test_tractive_and_pcm.py::test_precharge_time_constant_and_energy` |
| | Phase-change buffer energy is mass × latent heat; the enthalpy method conserves latent heat | Closed form | `tests/test_tractive_and_pcm.py::test_pcm_buffer_energy_is_mass_times_latent`, `tests/test_accuracy_invariants.py::test_pcm_effective_cp_conserves_latent_heat` |
| Powertrain | See [Powertrain / Drivetrain](#powertrain--drivetrain) | | |
| Data Acquisition | See [Data Acquisition](#data-acquisition) | | |
| Track Testing, SimulForge, OmniCore | See [Suspension / Dynamics](#suspension--dynamics) | | |
| DFMEA | See [Powertrain / Drivetrain](#powertrain--drivetrain) | | |

## Cooling

Powertrain, Data Acquisition, DFMEA, Tractive Safety.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Coolant heat rejection | Heat rejection equals ṁ·c_p·ΔT | Closed form | `tests/test_daq_plan.py::TestDeltaT::test_heat_rejection_matches_m_dot_cp_delta_t` |
| Radiator sizing | No benchmark: the steady state is checked for self-consistency only, not against a measured radiator | | `tests/test_accuracy_invariants.py::test_cooling_steady_state_inverts_its_own_ua_requirement` |
| Engine heat to coolant (ICE) | No benchmark: a declared share of the fuel energy, fed to the same loop sizing | | `tests/test_ice_powertrain.py::test_cooling_uses_the_engine_heat` |
| Powertrain, DFMEA | See [Powertrain / Drivetrain](#powertrain--drivetrain) | | |
| Tractive Safety | See [Electrics](#electrics) | | |
| Data Acquisition | See [Data Acquisition](#data-acquisition) | | |

## Data Acquisition

Data Acquisition, Electronics (PCB), DFMEA.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| CAN bus plan | Worst case of an 8-byte standard frame is the published 135 bits; bus load is the exact sum of frame bits × rate | Published, closed form | `tests/test_daq_plan.py::TestCanFrameBits::test_standard_8_byte_worst_case_is_135_bits`, `tests/test_daq_plan.py::TestBusLoad::test_load_is_the_exact_sum_of_frame_bits_times_rate` |
| Sampling | An anti-alias filter above Nyquist is refused | Closed form | `tests/test_daq_plan.py::TestSignalChain::test_filter_above_nyquist_defeats_its_own_purpose` |
| Road input | Road class recovered from a synthetic ISO 8608 profile | Known answer | `tests/test_measurement_fits.py::test_road_class_recovered_from_a_synthetic_iso_profile` |
| Electronics (PCB) | See [Electrics](#electrics) | | |
| DFMEA | See [Powertrain / Drivetrain](#powertrain--drivetrain) | | |

## Brakes

Brakes, Tire & Grip, ThermicPatch, Track Testing.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Hydraulics | Pedal force to axle torque against a hand calculation | Closed form | `tests/test_accuracy_invariants.py::test_hydraulic_chain_matches_hand_calculation` |
| Pedal box | Effort scales with bore squared; lever statics; line volume against a hand calculation | Closed form | `tests/test_pedal_box.py::test_bore_increase_reports_the_square_law_on_effort`, `::test_moving_the_pivot_toward_the_front_raises_front_force`, `::test_line_volume_matches_a_hand_calculation` |
| Stopping energy | Energy conserved across the axle split, with rotating inertia; rotor spins with the wheel | Closed form | `tests/test_accuracy_invariants.py::test_stop_energy_conserves_and_includes_rotating_inertia`, `tests/test_energy_rotor_gas_physics.py::test_rotor_spins_with_the_wheel` |
| Bolts | Joint stiffness and separation against the Wileman closed form | Closed form | `tests/test_accuracy_invariants.py::test_bolted_joint_stiffness_and_separation_are_exact` |
| Fade | Fade law recovered from synthetic data | Known answer | `tests/test_measurement_fits.py::test_fade_law_recovered` |
| Throttle and pedal return | Frictionless return against the analytical quarter period | Closed form | `tests/test_throttle_snap.py::test_frictionless_matches_analytical_quarter_period` |
| | Not benchmarked: rotor temperature has no measured rotor behind it; the cooling map is flagged synthesized | | `tests/test_brakes.py::test_cooling_map_is_synthesized_and_uncorrelated` |
| Tire & Grip, ThermicPatch, Track Testing | See [Suspension / Dynamics](#suspension--dynamics) | | |

## Chassis / Frame

Team Fit, Compliance, FlexGen, MorphMesh, OmniCore.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Frame Planner | Tube buckling scales as the Euler column | Closed form | `tests/test_tubeframe.py::test_euler_buckling_scaling` |
| Bracket FoS | No benchmark against FEA: hand-calculation modes (net section, tear-out, bending), checked against their own tests | | `tests/test_bracket_fos.py` |
| Team Fit | No physics to benchmark | | |
| Compliance, FlexGen, MorphMesh, OmniCore | See [Suspension / Dynamics](#suspension--dynamics) | | |

## Cost / Business

Cost & BOM.

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Cost & BOM | No physics to benchmark. Cost-event points are withheld until the year's reference values are entered | | `tests/test_driveline_trade.py::test_cost_event_points_are_withheld_without_the_years_references` |

## Power-user tabs

Tabs outside every role's list, shown under "All tabs".

| Feature | Benchmark | Kind | Re-run |
|---|---|---|---|
| Phantom Envelope | Vectorised clearances match the scalar distance; capsule boundary points lie on the skin | Closed form | `tests/test_phantom_envelope.py::test_point_segment_batch_matches_scalar`, `::test_single_capsule_boundary_points_lie_on_skin` |
| Ghost Topology | Member critical load against the Euler formula | Closed form | `tests/test_ghost_topology.py::test_euler_pcr_matches_formula` |

## Adding a benchmark

Put the reference values in the test, state where they came from in its docstring, and set the pass limit before the
comparison is run. Then add a row here under every role whose tab uses the feature, and a line in VALIDATION.md if it
changes what has been checked.
