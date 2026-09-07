# KinematiK: Physics & Mathematics Reference

This document covers every physics calculation in KinematiK. Each section maps directly to source code in `fsae_suspension/suspension/` and is verified by the test suite.

If you find an error in the math or physics, [open an issue](https://github.com/frederikexd/kinematik/issues). Nobody has yet.

---

## Coordinate System

SAE vehicle axes, millimetres throughout:

| Axis | Direction |
|------|-----------|
| x | Rearward positive |
| y | Right positive (toward driver's right) |
| z | Upward positive |

A single corner is modelled. Left/right symmetry is handled by mirroring y.

---

## 1. Suspension Kinematics (`kinematics.py`)

### 1.1 Constraint Solver

The kinematic state at any wheel travel is found by enforcing **rigid-link constraints** on the double-wishbone linkage.

**Unknowns** (9): lower outer ball joint `lo` (3), upper outer ball joint `uo` (3), tie-rod outer `tro` (3).

**Constraints** (9 independent equations from 10):

| # | Constraint | Equation |
|---|-----------|----------|
| 1 | Lower outer on lower-front sphere | `‖lo − lfi‖ = L_lower_f` |
| 2 | Lower outer on lower-rear sphere | `‖lo − lri‖ = L_lower_r` |
| 3 | Upper outer on upper-front sphere | `‖uo − ufi‖ = L_upper_f` |
| 4 | Upper outer on upper-rear sphere | `‖uo − uri‖ = L_upper_r` |
| 5 | Upright rigid length | `‖uo − lo‖ = L_upright` |
| 6 | Tie-rod outer rigid to lower outer | `‖tro − lo‖ = L_tro_lo` |
| 7 | Tie-rod outer rigid to upper outer | `‖tro − uo‖ = L_tro_uo` |
| 8 | Tie-rod length to inner pickup | `‖tro − tri‖ = L_tie` |
| 9 | Travel drive | `lo[z] = lo_static[z] + travel` |

Solved by **Levenberg-Marquardt least squares** (`scipy.optimize.least_squares`, method `"lm"`, tolerances `xtol = ftol = 1e-12`). Accepted when `max|residual| < 0.1 mm`. Warm-started from the previous travel step to stay on the correct configuration branch.

### 1.2 Camber

```
camber = −arctan( s[z] / |s[y]| )   [degrees]
```

where `s` is the unit wheel spin axis vector. Negative = top inboard (racing convention).

### 1.3 Toe (Bump Steer)

```
toe = arctan( s[x] / |s[y]| )   [degrees]
```

Positive = toe-out.

### 1.4 Caster & King-Pin Inclination

King-pin axis vector: `kp = uo − lo`

```
caster = arctan( kp[x] / kp[z] )    [degrees]   positive = top rearward
KPI    = arctan( −kp[y] / kp[z] )   [degrees]   positive = top inboard
```

### 1.5 Scrub Radius

King-pin axis extended to the ground plane (z = 0):

```
t      = −lo[z] / kp̂[z]
ground = lo + t · kp̂
scrub  = cp[y] − ground[y]   [mm]
```

### 1.6 Front-View Instant Centre & Roll-Centre Height

**Exact velocity construction** (not the midpoint approximation). Each ball joint rotates about its wishbone's chassis pivot axis. Its velocity is:

```
v = axis × (joint − point_on_axis)
```

Projected into the y-z plane, the front-view IC is where the perpendiculars to the two projected velocities meet. Parallel arms → IC at infinity → zero roll-centre contribution.

**Why not the midpoint construction:** the textbook method draws each wishbone as the line from its ball joint to the midpoint of its two chassis pickups. This is only correct when the wishbone pivot axis is parallel to x. With staggered z-heights (which anti-dive geometry requires), the midpoint line is no longer perpendicular to the ball joint's velocity. On the default geometry this error displaced the IC by ~96 mm and the roll centre by ~1.3 mm.

**Roll-centre height** is the z-coordinate of the line from the IC to the contact patch at ground level.

### 1.7 Side-View Instant Centre (SVIC)

Same velocity construction in the x-z plane:

```
v = axis × (joint − point_on_axis)
Project to (vx, vz)
Perpendicular direction: (−vz, vx)
Intersect two perpendiculars in x-z
```

Parallel → infinite swing arm → zero anti-dive. On the default geometry the classic wishbone-line construction gave 1908.6 mm swing-arm length; the velocity construction gives 2050.4 mm — a 26% error on anti-squat.

### 1.8 Anti-Dive

**Reference point: contact patch** (outboard front brakes).

Side-view path slope by central difference on the converged solver (δ = 0.5 mm, verified: 2 mm / 0.5 mm / 0.1 mm steps agree to 7 significant figures):

```
S = Δx_cp / Δz_cp

tan(φ) = −S    (signed: positive when SVIC is rearward of contact patch)

anti-dive% = tan(φ) × (wheelbase / CG_height) × brake_bias_front × 100
```

**Sign convention:** positive only when SVIC is rearward of the front contact patch and above ground. An SVIC forward of the patch is pro-dive and returns negative. The old bug took |offset|, which reported pro-dive geometry as positive anti-dive.

**Reference:** Gillespie §5; Milliken & Milliken §17.

### 1.9 Anti-Squat

**Reference point: wheel centre** (inboard drive — chassis reacts the torque, tractive force passes through the linkage at wheel-centre height).

```
tan(φ) = S_wc    (wheel-centre path slope — sign opposite to anti-dive)

anti-squat% = tan(φ) × (wheelbase / CG_height) × drive_bias_rear × 100
```

**Accuracy of the wheel-centre shortcut** (verified by static equilibrium):

| Method | Anti-squat |
|--------|-----------|
| Solid axle, force at contact patch | −40.3% |
| Inboard drive, half-shaft torque modelled (exact) | +16.1% |
| Wheel-centre shortcut (this implementation) | +17.1% |

The shortcut is right to ~1 percentage point. Patch vs wheel centre is a 56-point swing that changes sign. Residual is quantified and pinned by `test_anti_squat_shortcut_error_is_bounded`.

### 1.10 Motion Ratio

**With pushrod/rocker geometry** (real calculation):

```
‖pushrod_outer(θ) − rocker_pushrod(θ)‖ = L_pushrod    (solved by Brent's method)
spring_length(θ) = ‖rocker_spring(θ) − spring_inner‖
MR = d(wheel_travel) / d(spring_length)               (numerical differentiation)
```

**Without rocker** (direct-acting fallback): velocity of the ball-joint attachment point projected onto the spring axis. Labelled explicitly in all outputs.

---

## 2. Vehicle Dynamics (`dynamics.py`)

### 2.1 Lateral Load Transfer

Total lateral load transfer at axle in a steady-state corner at lateral acceleration `a_y`:

```
ΔFz_total = m · a_y · h_cg / track

ΔFz_geometric = (RC_height / track) · m · a_y   (geometric / roll-centre component)
ΔFz_elastic   = ΔFz_total − ΔFz_geometric        (through springs/ARB)
```

The elastic share is split front/rear by roll stiffness ratio:

```
ΔFz_front = ΔFz_elastic · K_front / (K_front + K_rear)  +  ΔFz_geometric_front
ΔFz_rear  = ΔFz_elastic · K_rear  / (K_front + K_rear)  +  ΔFz_geometric_rear
```

**When spring rates and motion ratio are supplied**, axle roll stiffness is derived rather than input directly:

```
wheel_rate = spring_rate × MR²
K_axle = wheel_rate × track² / 2   +   ARB_rate
```

This is the whole point of the rocker geometry: a quoted spring rate only maps to a wheel/roll rate through the motion ratio.

### 2.2 Per-Corner Vertical Loads

```
Fz_FL = W_front/2 − ΔFz_front   (outside loaded in a right-hand corner)
Fz_FR = W_front/2 + ΔFz_front
Fz_RL = W_rear/2  − ΔFz_rear
Fz_RR = W_rear/2  + ΔFz_rear
```

### 2.3 Longitudinal Load Transfer

Under braking/acceleration at longitudinal acceleration `a_x`:

```
ΔFz_longitudinal = m · a_x · h_cg / wheelbase
```

Front axle gains load under braking, rear gains under acceleration.

### 2.4 Roll-Centre Height

Roll-centre height is derived from the front-view instant centre at each corner:

```
RC_height = IC[z] × track / (track − 2 · IC[y])     (where IC is the instant centre [y,z])
```

Migration across travel is reported explicitly — a single static RC height is honest only when the team understands how much it moves.

---

## 3. Tyre Model (`tiremodel.py`)

### 3.1 Pacejka Magic Formula MF5.2 — Pure Lateral

Standard MF5.2 pure-lateral force equations (Pacejka, *Tyre and Vehicle Dynamics*). The equations are textbook and open-source. Coefficients are tire-specific, loaded from a private file that is gitignored and never committed.

```
dfz = (Fz − FNOMIN) / FNOMIN          normalised load increment
g   = γ · LGAY                         scaled camber (rad)

Cy  = PCY1 · LCY                       shape factor
μy  = Fz · (PDY1 + PDY2·dfz) · (1 − PDY3·g²) · LMUY      peak factor
Dy  = μy                                peak value
Ky  = PKY1·FNOMIN·sin(2·arctan(Fz/(PKY2·FNOMIN·LKY))) · (1−PKY3·|g|) · LKY
                                        cornering stiffness
Ey  = (PEY1 + PEY2·dfz) · (1 − (PEY3 + PEY4·g)·sgn(α))   curvature factor
SHy = (PHY1 + PHY2·dfz) · LHY + PHY3·g   horizontal shift
SVy = Fz·((PVY1 + PVY2·dfz)·LVY + (PVY3 + PVY4·dfz)·g)·LMUY·LVY
                                        vertical shift
αy  = α + SHy                          shifted slip angle
By  = Ky / (Cy·Dy)                     stiffness factor
Fy  = Dy · sin(Cy · arctan(By·αy − Ey·(By·αy − arctan(By·αy)))) + SVy
```

**Key fix:** `LGAY` (camber scaling factor) is now applied to all camber-dependent terms (Dy, Ey, Ky, SHy, SVy) consistently. Previously it was applied to SVy alone, so it scaled camber thrust while leaving camber's effect on peak mu, cornering stiffness, and horizontal shift untouched — which made the factor mean something different from what the standard defines.

### 3.2 Combined Slip (Friction Ellipse)

```
F_combined = √(Fx² + Fy²) ≤ μ · Fz

Fx_available = F_cap · √(1 − (Fy / F_cap)²)
```

Standard elliptic combined-slip approximation. Flagged uncalibrated until drive/brake TTC data is supplied.

### 3.3 Tyre Relaxation Length

```
dFy/dt = (V / σ) · (Fy_ss − Fy)
```

where σ is the relaxation length (typically 0.3–0.5 m for FSAE tyres) and Fy_ss is the Magic Formula steady-state force. Used in the transient solver.

---

## 4. GGV Diagram (`ggv.py`)

The g-g-V envelope: for each forward speed V, the boundary in (longitudinal g, lateral g) space that the car can sustain in steady state.

```
Lateral limit:    a_y_max = Fy_max(Fz_loaded) / m
                  where Fz_loaded includes lateral load transfer from VehicleDynamics

Traction limit:   a_x_max = min(P / (m·V), F_traction_cap / m)
                  F_traction = drive_grip_frac · μ · (m·g + Fz_aero)

Brake limit:      a_x_min = −(μ_front·Fz_front + μ_rear·Fz_rear) / m

Combined:         friction ellipse — √((ax/ax_max)² + (ay/ay_max)²) ≤ 1
```

Design levers that move the envelope: CG height, roll-centre height, wheel rate, dynamic camber gain, weight distribution, aero ClA/CdA.

---

## 5. Lap Simulation (`lapsim.py`, `laptime.py`)

Quasi-steady-state (QSS) point-mass simulation on a track defined as a sequence of straights and constant-radius arcs.

### 5.1 Corner Speed Limit

```
v_corner = √(a_y_max · R)

a_y_max from VehicleDynamics lateral grip at that speed (load-sensitive Pacejka)
```

### 5.2 Three-Pass Speed Profile

```
Pass 1 — corner limits:   v[i] ≤ v_corner[i] for all arc segments
Pass 2 — forward (accel): v[i+1] = min(v_corner[i+1], √(v[i]² + 2·a_x_max·ds))
Pass 3 — backward (brake): v[i] = min(v[i], √(v[i+1]² + 2·|a_x_min|·ds))
```

### 5.3 Lap Time Integration

```
dt[i] = ds[i] / v[i]
t_lap  = Σ dt[i]
```

### 5.4 Motor Map (Real Torque Curve)

When a motor map is supplied (shaft torque vs RPM from a dyno or datasheet):

```
ω_motor    = v / r_wheel × final_drive          [rad/s]
rpm_motor  = ω_motor × 60 / (2π)
T_wheel    = T_motor(rpm) × final_drive × η_drivetrain
F_traction = T_wheel / r_wheel
```

Falls back to flat-power approximation (`P_max / v`) when no curve is supplied.

---

## 6. Transient Dynamics (`transient.py`)

Explicit RK4 time-step solver (1 ms fixed step) for the unsteady behaviour QSS discards.

**State vector:** longitudinal velocity u, lateral velocity v, yaw rate r, roll angle φ, pitch angle θ, four unsprung vertical positions z_i, four lagged slip angles α_i.

**Differential block (RK4):**

```
ṁu = Fx/m + r·v
ṁv = Fy/m − r·u
ṙ  = Mz / Iz
φ̈  = (M_roll_net) / Iroll
θ̈  = (M_pitch_net) / Ipitch
z̈_i = (Fz_tyre_i − Fz_spring_i − Fz_damper_i) / m_unsprung
α̇_i = (V/σ) · (α_ss_i − α_i)     (tyre relaxation)
```

**Algebraic block (explicit, evaluated each step):**

```
Fz_i = static_load ± lateral_transfer ± longitudinal_transfer ± aero
α_ss = arctan((v ± r·a) / u)     (steady-state slip angle at each corner)
Fy_i = Pacejka(α_i, Fz_i, γ_i)
Fx_i = demand, friction-ellipse limited: |Fx| ≤ √(F_cap_i² − Fy_i²)
```

Captures: turn-in lag, snap-oversteer onset, pitch/dive through brake-to-throttle, kerb strikes and wheel lift.

---

## 7. Differential & Traction Model (`driveline.py`)

### 7.1 Per-Wheel Traction Capacity

```
Fz_i     = vertical load from VehicleDynamics (includes lateral load transfer)
F_cap_i  = μ(Fz_i) · Fz_i                   (load-sensitive grip)
λ        = a_y / a_y_max                      (lateral utilisation)
Fx_cap_i = F_cap_i · √(1 − λ²)              (friction circle)
T_cap_i  = Fx_cap_i · r_wheel
```

### 7.2 Differential Torque Split

```
Open:   T_total = 2 · T_cap_inside
LSD:    T_out   = min(T_cap_out, TBR · T_cap_in + preload)
        T_total = T_cap_in + T_out
Spool:  T_total = T_cap_in + T_cap_out
```

TBR is the manufacturer's published torque bias ratio. LSD degenerates correctly: TBR = 1, preload = 0 gives the open diff; TBR → ∞ gives the spool.

### 7.3 Mapping to Lap Sim

```
drive_grip_frac = F_x_available / (μ · (m·g + Fz_aero))
```

This replaces the hand-typed scalar in `LapSimParams` with a physically derived quantity. A spool vs open diff now produces a different lap time rather than an identical one.

---

## 8. Compliant Kinematics (`compliance.py`)

Couples the rigid kinematic solver with member compliance in a loop:

```
1. Rigid solve → corner geometry
2. Load-path solve → axial force in each link
3. Compliance: δL = F / k_axial    (k = E·A/L for tube, or condensed FEA body)
4. Feed length changes back into solver → re-solve
5. Iterate to convergence (typically 2–4 steps)
```

**Outputs:**

```
compliance_toe    = toe(compliant) − toe(rigid)      [deg]  "compliance steer"
compliance_camber = camber(compliant) − camber(rigid) [deg]
```

Member stiffness sources: analytic (E·A/L for a steel tube), direct k input, or a condensed FEA flex body.

---

## 9. Damper Model (`damper.py`)

Bilinear-digressive force-velocity law, independent bump and rebound:

```
For |v| ≤ v_knee:   F = c_low · v
For |v| > v_knee:   F = c_low · v_knee + c_high · (|v| − v_knee)   [signed for bump/reb]
```

Calibrated from dyno data via `from_dyno_points` (least-squares fit). Flagged `is_calibrated = False` and `source = "representative"` until real dyno data is loaded.

**Critical damping ratio:**

```
ζ = c_effective / (2 · √(k_wheel · m_corner))
```

where `k_wheel = spring_rate × MR²` and `m_corner = total_mass / 4`.

---

## 10. Aero — Panel Method (`aero/panel_method.py`)

3D source-panel boundary-element method on the real STL geometry.

**Flow tangency constraint** (one equation per panel):

```
A · σ = −V∞ · n̂
```

where A is the panel influence matrix, σ the source strengths, V∞ the onset flow, n̂ the panel normals.

**Ground effect:** image of every panel reflected through z = 0 (road plane), so the road is an exact streamline. Ground effect emerges from the physics, not a tuned gain.

**Surface pressure:**

```
Cp = 1 − (V_surface / V∞)²    (Bernoulli, inviscid)
```

**Forces:**

```
C_L, C_D_pressure = surface integral of Cp · n̂
C_D_friction      = flat-plate turbulent skin-friction estimate (Cf × wetted area)
C_D_total         = C_D_pressure + C_D_friction
```

Fidelity label: `POTENTIAL`. Captures: ground effect, attached-flow pressure field, downforce trend with rake/ride height. Does not capture: viscous separation, turbulent wake, stall. Trust deltas between geometries more than absolute levels.

---

## 11. Aero — Vortex Lattice Method (`aero/vortex_lattice.py`)

Horseshoe-vortex lattice over the mean camber surface for isolated lifting surfaces (wings).

**Kutta condition:** 1/4–3/4 rule — bound vortex at quarter-chord, control point at three-quarter-chord.

**Circulation solve:**

```
A · Γ = −V∞ · n̂
```

**Forces:**

```
L  = ρ · V∞ · Σ(Γ_i · Δy_i)     (Kutta–Joukowski)
Di = ρ · V∞ · Σ(Γ_i · w_i · Δy_i)   (induced drag from own downwash)
```

**Ground effect:** image vortices reflected through z = 0 with reversed circulation.

**Validated against closed form** (rectangular AR = 8 wing):
- C_L within 6% of lifting-line theory (expected — rectangular planform ≠ elliptic loading)
- Span efficiency e = 1.004 (induced drag = theoretical)
- Grid-converged to < 0.5% between 30×8 and 40×10 panels
- C_L rises monotonically 0.411 (free air) → 0.931 (h/c = 0.15)

Does not capture: profile drag, viscous separation, stall. Induced drag only.

---

## 12. Brake Thermal Model (`brakes.py`)

Explicit lumped-capacitance thermal network: friction ring → hat → hub → caliper/fluid branch.

**Heat input:** `P_brake = F_brake · v` deposited into the friction ring on every braking sample.

**Thermal network (forward-Euler with adaptive sub-steps):**

```
C_ring · dT_ring/dt = P_brake − h_c · A_ring · (T_ring − T_ambient) − k_hat · (T_ring − T_hat)
C_hat  · dT_hat/dt  = k_hat · (T_ring − T_hat) − k_hub · (T_hat − T_hub)
...
```

**Convective coefficient** `h_c(speed)` from a CFD map (or analytic surrogate until calibrated).

**Design loop:** thin the friction ring until `T_peak < T_pad_limit` AND `T_fluid < T_boil`. Returns lightest rotor that passes both limits, or reports that none in the search does.

---

## 13. Structural Analysis

### 13.1 Bolted Joint (`bolted_joint.py`) — VDI 2230

```
F_preload = T_assembly / (k · d)              (assembly torque to clamp force)
Φ         = k_bolt / (k_bolt + k_member)      (joint stiffness ratio, bolt's share)
F_bolt_max = F_preload + Φ · F_external       (peak bolt tensile force)
F_sep      = F_preload / (1 − Φ)             (joint separation load)
σ_bolt     = F_bolt_max / A_stress            (bolt tensile stress)
```

**Reference:** VDI 2230 Part 1; Shigley *Mechanical Engineering Design*.

### 13.2 Bracket Factor of Safety (`bracket_fos.py`) — Shigley

Four failure modes, all closed-form:

```
Direct tension/shear:  σ = F / A,   τ = V / A
Bending (governing for most tab brackets):
    σ = M · c / I    where M = F · lever_arm, I = b·h³/12
Bearing at bolt hole:  σ_bearing = P / (d · t)
Fillet weld throat:    τ_weld = F / (0.707 · leg · weld_length)

FoS = σ_yield / σ_governing    (yield, not ultimate — per team's standing rule: FoS ≥ 1.5)
```

Flags `screening_only = True` — does not capture stress concentration, weld-toe HAZ, or 3D stiffness effects.

### 13.3 Tube Frame (`tubeframe.py`)

**Triangulation audit:** frame as node/tube graph. Reports: non-triangulated nodes, quadrilateral bays without diagonals, mid-span tube junctions (load-path interruptions), and whether a triangulated load path exists between two named nodes.

**Tube equivalency** (FSAE rules compliance):

```
EI_candidate ≥ EI_baseline    (bending stiffness)
σ_yield_candidate ≥ σ_yield_baseline × Z_candidate / Z_baseline    (bending strength)
wall ≥ wall_minimum
```

**Panel fastener pitch:**

```
F_fastener = p_load · pitch²        (pressure × tributary area per fastener)
deflection  = 5 · p_load · a⁴ / (384 · E · I)    (simply-supported plate between fasteners)
```

---

## 14. Test Coverage

Every formula in this document is pinned by the test suite. Key tests:

| Test | What it pins |
|------|-------------|
| `test_flat_pickups_give_zero_anti_dive` | Parallel arms → SVIC at infinity → zero anti-dive |
| `test_anti_squat_shortcut_error_is_bounded` | Wheel-centre shortcut within 1 pp of exact |
| `test_instant_center_staggered_pickups` | Velocity IC vs midpoint IC diverge correctly when staggered |
| `test_motion_ratio_with_rocker` | Full pushrod/rocker chain vs direct-acting |
| `test_kinematics_*` | Camber, toe, caster, KPI, scrub radius |
| `test_pacejka_*` | MF5.2 force law, mu_peak, LGAY scaling |
| `test_lateral_load_transfer_*` | Load transfer split, per-corner Fz |
| `test_roll_center_*` | RC height, RC migration |
| `test_compliance_*` | Compliance steer and camber |
| `test_damper_*` | Bilinear force law, damping ratio |
| `test_vortex_lattice_*` | CL vs lifting-line, span efficiency, ground effect |
| `test_bolted_joint_*` | VDI 2230 preload, separation, stiffness ratio |
| `test_bracket_fos_*` | FoS on yield for all four failure modes |

**1578 tests. All passing. CI is green.**

---

## References

- Gillespie, T.D. — *Fundamentals of Vehicle Dynamics*, SAE International
- Milliken, W.F. & Milliken, D.L. — *Race Car Vehicle Dynamics*, SAE International
- Dixon, J.C. — *Tires, Suspension and Handling*, SAE International
- Pacejka, H.B. — *Tyre and Vehicle Dynamics*, Elsevier
- VDI 2230 Part 1 — Systematic Calculation of Highly Stressed Bolted Joints
- Shigley, J.E. — *Mechanical Engineering Design*, McGraw-Hill
- AWS D1.1 — Structural Welding Code
