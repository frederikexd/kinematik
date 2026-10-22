# KinematiK: Physics & Mathematics Reference

This document covers the physics and mathematics behind every feature of KinematiK, across every discipline it models: suspension kinematics and synthesis, vehicle dynamics and lap time, tyres, aerodynamics, brakes and pedal box, powertrain and energy, battery and cooling thermal, electrical systems, data acquisition, structures, measurement and uncertainty. Each section names the source module in `fsae_suspension/suspension/` (or `fsae_suspension/scripts/`, `fsae_suspension/powertrain/`) that implements it. Section 32 sets out how the physics is verified, and the external validation it has had so far.

Modules that only move data (the app, accounts, reports, file import and export, plotting) contain no physics and are not listed. A model labelled *representative* or *uncalibrated* in its module returns `synthesized=True` or `calibrated=False` on its outputs until it is fitted to measurements.

If you find an error in the math or physics, [open an issue](https://github.com/frederikexd/kinematik/issues). The errors found so far, and the corrections they forced, are listed in §32.3.

---

## Coordinate System

SAE vehicle axes, millimetres throughout:

| Axis | Direction |
|------|-----------|
| x | Rearward positive |
| y | Right positive (toward driver's right) |
| z | Upward positive |

A single corner is modelled: a right-hand corner, so +y is outboard. Left/right symmetry is handled by mirroring y; the left wheel at rack travel +x is the mirrored right wheel at −x.

Vehicle-level functions use their own stated sign: in `target_derivation.corner_loads` and `tire_utilisation`, `a_lat_g > 0` is a left turn (load to the right-hand tyres) and `a_long_g > 0` is braking; in `torque_distribution`, x is forward of the CG and y is to the left. Each function's docstring states its frame.

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

Solved by **Levenberg-Marquardt least squares** (`scipy.optimize.least_squares`, method `"lm"`, tolerances `xtol = ftol = 1e-12`). Accepted when `max|residual| < 0.1 mm`. `sweep` solves static first and marches outward in both directions, warm-starting each step from the previous pose so the solver stays on the correct configuration branch.

Steering is a rack displacement of the tie-rod inner pickup along y (`pickup_deltas={"tie_rod_inner": (0, x, 0)}`), with the tie-rod length held.

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

The king-pin axis is extended to the **road**, the plane through the contact patch at this travel:

```
kp̂     = (uo − lo) / ‖uo − lo‖
t      = (cp[z] − lo[z]) / kp̂[z]
ground = lo + t · kp̂
scrub  = cp[y] − ground[y]   [mm]
```

In the body-fixed corner frame the road moves with the wheel: at +20 mm of bump it sits 20 mm above z = 0. Intersecting the axis with z = 0 instead adds `tan(KPI) × travel` to the scrub (2.3 mm at 20 mm of bump on `front_v11_joints50`). At static the two planes coincide. The generic adapter (`adapter.py`) uses the same road plane, through the wheel-centre reference it carries, and the mechanical trail of §16.1 is taken on the same plane. Pinned by `test_scrub_radius_is_measured_on_the_road`.

### 1.6 Front-View Instant Centre & Roll-Centre Height

**Exact velocity construction** (not the midpoint approximation). Each ball joint rotates about its wishbone's chassis pivot axis `a` (front inner pickup to rear inner pickup). Its velocity is:

```
v = a × (joint − p_front_inner)
```

Projected into the y-z plane, the front-view IC lies on the line through each joint perpendicular to its projected velocity `(v_y, v_z)`, with direction `(−v_z, v_y)`. The two lines meet at the IC. Parallel lines → IC at infinity → NaN, not zero.

**Roll-centre height** (`dynamics.roll_center_height`, `ghost_topology._rc_height_mm`, `robustness.hand_roll_centre`): the line from the contact patch through the IC, intersected with the centreline y = 0:

```
m        = (IC_z − cp_z) / (IC_y − cp_y)
RC_height = cp_z − m · cp_y
```

When `IC_y = cp_y` the line is vertical and never reaches the centreline: the roll centre is at infinity and the function returns NaN. (It used to return the contact-patch height, which reads as a legitimate 0 mm design target.)

**Why not the midpoint construction:** the textbook method draws each wishbone as the line from its ball joint to the midpoint of its two chassis pickups. This is only correct when the wishbone pivot axis is parallel to x. With staggered z-heights (which anti-dive geometry requires), the midpoint line is no longer perpendicular to the ball joint's velocity. On the default geometry this error displaced the IC by ~96 mm and the roll centre by ~1.3 mm. On a staggered FSAE front (`front_v11_joints50`) it puts the roll centre 3.8 mm low (52.26 against 56.09 mm).

**Hand check:** the construction above, done by hand from a corner's hardpoints (pivot axes, joint velocities, the two front-view lines and their intersection), reproduces KinematiK's IC and roll-centre height to within 1e-12 mm.

### 1.7 Side-View Instant Centre (SVIC)

Same velocity construction in the x-z plane:

```
v = a × (joint − p_front_inner)
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

Every anti value is the same function of a side-view path slope, `anti = slope × (L/h) × (brake or drive share) × 100`; recomputing it from the solved path by central difference over ±0.5 mm agrees with these functions to better than 1e-9 points.

**Reference:** Gillespie §5; Milliken & Milliken §17.

### 1.9 Anti-Squat and Anti-Lift

**Anti-squat reference point: wheel centre** (inboard drive — chassis reacts the torque, tractive force passes through the linkage at wheel-centre height).

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

The shortcut is right to ~1 percentage point. Patch vs wheel centre is a 56-point swing that changes sign. The residual comes from the camber and scrub terms of the couple `(wc − cp) × Fx`, which the wheel bearings and tie rod take; it is quantified and pinned by `test_anti_squat_shortcut_error_is_bounded`.

**Anti-lift** (`anti_lift_pct`, rear under braking). Braking unloads the rear, so the sign is opposite to anti-dive:

```
outboard rear brakes:  anti-lift% = +brake_bias_rear × S_cp × (L/h) × 100      (contact-patch path)
inboard rear brakes:   anti-lift% = anti-squat% × brake_bias_rear / drive_bias_rear   (wheel-centre path)
```

Inboard brakes, like an inboard drive, react their torque at the chassis, so only the braking force passes through the links, at wheel-centre height.

### 1.10 Motion Ratio

**Convention:** `MR = spring (damper) travel / wheel travel`, so `k_wheel = k_spring × MR²`.

**With pushrod/rocker geometry** (real calculation):

```
‖pushrod_outer(θ) − rocker_pushrod(θ)‖ = L_pushrod    (solved by Brent's method)
spring_length(θ) = ‖rocker_spring(θ) − spring_inner‖
MR = |Δ spring_length / Δ z_wheel_centre|            (central difference over ±5 mm of wheel travel)
```

**Without rocker** (direct-acting fallback): the lower-ball-joint displacement projected onto the spring axis, over the same wheel-centre travel. Labelled explicitly in all outputs (`motion_ratio_is_real()`).

### 1.11 Scrub Rate and the Force-Based Roll Centre (`scripts/force_based_rc.py`)

Let the wheel rise `dz`, body held; the contact patch moves sideways `dy = q dz`, with `q = dy_cp/dz_cp` the scrub rate (corner frame, y outboard). By virtual work, of the patch loads `Fz` and `Fy`:

```
spring path sees:            Fz + q · Fy  (+ q_x · Fx)
links carry into the body:   V = −q · Fy   (positive up)
```

`V` is the geometric (link-borne) part of the lateral load transfer; summed over an axle it is the **jacking force**. The **force-based roll centre** is the height at which the axle's lateral force acts on the body, the moment of the link-borne forces about the ground centre point over the axle's lateral force (vehicle frame, outer and inner patches at `y_o`, `y_i`):

```
h_FRC = −( y_o · V_o + y_i · V_i ) / ( F_y,o + F_y,i )
geometric transfer = (F_y,o + F_y,i) · h_FRC / track
```

It equals the geometric roll centre only when both wheels share one scrub rate. In roll they differ, and the force-based one is what transfers load. The geometric intercept from the same rate is `RC = cp_z + q · cp_y`, the CP-IC line of §1.6.

Built-in proofs: `q` from the patch motion equals the slope of the CP-IC line (within 0.2% front, 1.3% rear, a 3D motion against its projection); the pushrod force from `loadpath` equals the virtual-work prediction; the link-borne force from `loadpath`'s member forces equals `−q Fy` to 1e-6 N.

---

## 2. Vehicle Dynamics (`dynamics.py`)

### 2.1 Lateral Load Transfer

Steady-state corner at lateral acceleration `a_y`, axle weight fractions `w_f` and `1 − w_f`, roll-centre heights `RC_f`, `RC_r`:

```
h_ra    = RC_f · w_f + RC_r · (1 − w_f)          roll axis height at the CG station
M_roll  = m · a_y · (h_cg − h_ra)                roll moment about the roll axis
φ       = M_roll / (K_f + K_r)                   roll angle

ΔFz_elastic,i  = (K_i / (K_f + K_r)) · M_roll / track_i        through springs and bars
ΔFz_geometric,i = m_i · a_y · RC_i / track_i                    through the links (m_f = m·w_f, m_r = m·(1 − w_f))
ΔFz_i = ΔFz_elastic,i + ΔFz_geometric,i
```

Summed over both axles, `ΔFz_f·t_f + ΔFz_r·t_r = m·a_y·h_cg` for any pair of roll centres (pinned by `test_load_transfer_closes_moment_balance` and `test_load_transfer_closes_with_split_roll_centres`). When a roll centre cannot be solved, a documented default is used and flagged.

**When spring rates and motion ratio are supplied**, axle roll stiffness is derived rather than input directly:

```
k_wheel = k_spring × MR²                               [N/mm]
K_axle  = k_wheel · track² / 2 · (π/180) / 1000  +  K_ARB     [N·m/deg; track in mm]
```

This is the whole point of the rocker geometry: a quoted spring rate only maps to a wheel/roll rate through the motion ratio.

### 2.2 Per-Corner Vertical Loads

```
Fz_FL = W_front/2 − ΔFz_front   (outside loaded in a right-hand corner)
Fz_FR = W_front/2 + ΔFz_front
Fz_RL = W_rear/2  − ΔFz_rear
Fz_RR = W_rear/2  + ΔFz_rear
```

The design-case loads with bank, grade, downforce and steer warp are in §14.7.

### 2.3 Longitudinal Load Transfer

Under braking/acceleration at longitudinal acceleration `a_x`:

```
ΔFz_longitudinal = m · a_x · h_cg / wheelbase      (axle total; half per wheel)
```

Front axle gains load under braking, rear gains under acceleration.

### 2.4 Roll-Centre Height

From the front-view instant centre at each corner, by the construction of §1.6:

```
RC_height = cp_z − cp_y · (IC_z − cp_z) / (IC_y − cp_y)
```

Migration across travel is reported explicitly (`roll_center_migration`) — a single static RC height is honest only when the team understands how much it moves. The inverse synthesis can bound migration as a least-squares slope over travel in the body frame (§15.3); the manifest record reports it against the ground as well.

---

## 3. Tyre Model (`tiremodel.py`)

### 3.1 Pacejka Magic Formula MF5.2 — Pure Lateral

Standard MF5.2 pure-lateral force equations (Pacejka, *Tyre and Vehicle Dynamics*). The equations are textbook and open-source. Coefficients are tire-specific, loaded from a private file that is gitignored and never committed; the default set is a synthetic, hand-tuned one, flagged as such.

```
dfz = (Fz − FNOMIN) / FNOMIN          normalised load increment
g   = γ · LGAY                         scaled camber (rad)

Cy  = PCY1 · LCY                                           shape factor
μy  = (PDY1 + PDY2·dfz) · (1 − PDY3·g²) · LMUY             peak friction
Dy  = μy · Fz                                              peak value
SHy = (PHY1 + PHY2·dfz) · LHY + PHY3·g                     horizontal shift
αy  = α + SHy                                              shifted slip angle
Ey  = (PEY1 + PEY2·dfz) · (1 − (PEY3 + PEY4·g)·sgn(αy)) · LEY,   capped at Ey ≤ 1
Ky  = PKY1·FNOMIN · sin(2·arctan(Fz / (PKY2·FNOMIN·LFZO))) · (1 − PKY3·|g|) · LKY
                                                           cornering stiffness
By  = Ky / (Cy·Dy)                                         stiffness factor
SVy = Fz · ((PVY1 + PVY2·dfz)·LVY + (PVY3 + PVY4·dfz)·g)   vertical shift
Fy  = Dy · sin(Cy · arctan(By·αy − Ey·(By·αy − arctan(By·αy)))) + SVy
```

**Fixes pinned by the tests:** `LGAY` scales camber before it enters every camber-dependent term (Dy, Ey, Ky, SHy, SVy), not SVy alone; the curvature term keys off the sign of the **shifted** slip `αy`, not the raw `α`, so the asymmetry break sits in the right place when `SHy ≠ 0`.

**Peak friction** `mu_peak(Fz, γ) = max|Fy| / Fz` over a slip sweep of ±15° (121 points), widened to ±30° when the peak lands on the edge of the sweep.

### 3.2 Combined Slip (Friction Ellipse)

```
F_combined = √(Fx² + Fy²) ≤ μ · Fz

Fx_available = F_cap · √(1 − (Fy / F_cap)²)
```

Standard elliptic combined-slip approximation. Flagged uncalibrated until drive/brake TTC data is supplied. The tyre-use and torque-distribution models (§19) scale lateral force by the ellipse for each tyre's own Fx, with the longitudinal-to-lateral friction ratio declared.

### 3.3 Tyre Relaxation Length

```
dα/dt = (|V| / σ) · (α_ss − α)
```

where σ is the relaxation length (typically 0.3–0.5 m for FSAE tyres; 0.47 m by default) and α_ss the steady-state slip. Used in the transient solver (§6) and the steering-response model (§16.6).

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

Design levers that move the envelope: CG height, roll-centre height, wheel rate, dynamic camber gain, weight distribution, aero ClA/CdA. The per-tyre quasi-steady envelope, with the limiting tyre in each direction, is §19.1.

---

## 5. Lap Simulation (`lapsim.py`, `laptime.py`)

Quasi-steady-state (QSS) point-mass simulation on a track defined as a sequence of straights and constant-radius arcs.

### 5.1 Corner Speed Limit

```
v_corner = √(a_y_max · R)

a_y_max from VehicleDynamics lateral grip at that speed (load-sensitive Pacejka),
iterated because the grip depends on the speed through downforce
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

Explicit RK4 time-step solver (1 ms fixed base step, optional sub-steps for the stiff tyre-vertical mode) for the unsteady behaviour QSS discards. The model is an index-1 DAE whose algebraic block is an explicit function of the state, so no implicit solve is needed per step.

**State vector (24):** body velocities u, v and yaw rate r; global X, Y and heading ψ; sprung heave z_s and its rate; roll φ and its rate; pitch θ and its rate; four unsprung vertical positions z_u,i and their rates; four lagged slip angles α_i.

**Differential block (RK4):**

```
u̇   = Fx/m + r·v
v̇   = Fy/m − r·u
ṙ   = Mz / Iz
Ẋ, Ẏ, ψ̇ from (u, v, r) and ψ
z̈_s = Σ ΔF_suspension / m_s             (forces measured from static, so the static balance cancels)
φ̈   = M_roll_net / I_roll
θ̈   = M_pitch_net / I_pitch
z̈_u,i = (Fz_tyre,i − Fz_spring,i − Fz_damper,i) / m_unsprung
α̇_i = (|u| / σ) · (α_ss,i − α_i)     (tyre relaxation)
```

**Algebraic block (explicit, evaluated each step):**

```
Fz_i = static_load ± lateral_transfer ± longitudinal_transfer ± aero
α_ss = arctan((v ± r·a) / u)     (steady-state slip angle at each corner)
Fy_i = Pacejka(α_i, Fz_i, γ_i)
Fx_i = demand, friction-ellipse limited: |Fx| ≤ √(F_cap_i² − Fy_i²)
```

Captures: turn-in lag, snap-oversteer onset, pitch/dive through brake-to-throttle, kerb strikes and wheel lift. Does not model wheel-spin states (longitudinal slip ratio), tyre temperature, or the closed kinematic loops with multipliers; those are flagged in the module.

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
Open:   T_out   = min(T_cap_out, T_cap_in + preload)
LSD:    T_out   = min(T_cap_out, TBR · T_cap_in + preload)
        T_total = T_cap_in + T_out
Spool:  T_total = T_cap_in + T_cap_out
```

TBR is the manufacturer's published torque bias ratio. With zero preload the open diff gives `T_total = 2 · T_cap_inside`. LSD degenerates correctly: TBR = 1, preload = 0 gives the open diff; TBR → ∞ gives the spool. The quasi-steady comparison of open, clutch LSD, spool and torque vectoring through whole corners is §19.2.

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
2. Load-path solve → axial force in each link (§17.1)
3. Compliance: δL = F / k_axial    (k = E·A/L for tube, or condensed FEA body)
4. Feed length changes back into solver → re-solve
5. Iterate to convergence (typically 2–4 steps)
```

**Outputs:**

```
compliance_toe    = toe(compliant) − toe(rigid)      [deg]  "compliance steer"
compliance_camber = camber(compliant) − camber(rigid) [deg]
```

Member stiffness sources: analytic (E·A/L for a steel tube), direct k input, or a condensed FEA flex body. Stiffness at the chassis pickups in any direction, joint and tab stiffness in series, and free play are §18.

---

## 9. Damper Model (`damper.py`)

Bilinear-digressive force-velocity law, independent bump and rebound:

```
For |v| ≤ v_knee:   F = c_low · v
For |v| > v_knee:   F = c_low · v_knee + c_high · (|v| − v_knee)   [signed for bump/reb]
```

Calibrated from dyno data via `from_dyno_points` (least-squares slopes through the origin and through the knee force). Flagged `is_calibrated = False` and `source = "representative"` until real dyno data is loaded.

**Damping ratio** (`damping_ratio`), on the low-speed slope that governs the body modes:

```
c_wheel = c_shaft,low · MR²                          (MR = shaft travel / wheel travel)
k_ride  = k_wheel · k_tyre / (k_wheel + k_tyre)       when a tyre rate is given, else k_wheel
ζ       = c_wheel / (2 · √(k_ride · m_corner))
```

`m_corner` is the corner mass the caller passes (the sprung corner mass), not `total_mass / 4`. Leaving the tyre out overstates the stiffness the mass sees and understates ζ (35 N/mm on a 130 N/mm tyre: ζ about 11% low).

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

**Checked against closed form** (sphere in free air, ground effect and circulation off; `scripts/aero_closed_form.py`): surface Cp against `1 − (9/4) sin²θ` has minimum −1.248 (analytic −1.250), mean |ΔCp| 0.009 and largest |ΔCp| 0.028 on 1,280 panels; 0.007 and 0.024 on 5,120.

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

**Checked against closed form** (flat rectangular AR = 8 wing, 4°, free air, 40 × 10 panels; `scripts/aero_closed_form.py`, `tests/test_aero_closed_form.py`):
- C_L 11% under lifting-line theory and 9% under Helmbold's lifting-surface estimate (rectangular planform ≠ elliptic loading accounts for part of this)
- C_L 4% under an independent flat-plate lattice on the same grid; the gap is not yet explained (open item)
- Span efficiency e = 0.92
- Grid-converged to 0.2% between 30×8 and 40×10 panels
- C_L rises 2.4× from free air to h/c = 0.15

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

The frame beam model used for torsion and pickup stiffness is §18.5; link buckling and rocker bearings are §17.

---

## 14. Target Derivation (`target_derivation.py`)

Targets are derived from vehicle requirements rather than declared, so the chain can be re-run when an input changes. Units: mm, kg, deg, g, Hz, N. The default declared vehicle (`Vehicle`): 300 kg, h = 280 mm, L = 1630 mm, t = 1210 mm, jounce budget 17.5 mm, sprung corner masses 60.1 / 66.1 kg, front weight fraction 0.48. The declared tyre (`Tire`): camber optimum −1.83°, peak μ 1.55, camber sensitivity 0.36%/deg. Tyres are rigid (no vertical rate) unless a rate is declared.

### 14.1 Design Accelerations

The design accelerations scale with peak μ from the reference μ_ref = 1.55:

```
a_lat      = 1.5  · μ / 1.55   [g]
a_combined = 1.06 · μ / 1.55   [g]   (1.5 / √2, each way)
a_straight = 1.06 · μ / 1.55   [g]
```

### 14.2 Roll and Outer-Wheel Bump

```
φ      = RG · a_y                          roll angle (deg), RG the roll gradient (deg/g)
z_roll = (t/2) · tan φ                     outside-wheel bump from roll (mm)
φ_tyre = m g a_y h / (k_t · t²)            extra roll to the road from tyre deflection (rad), when a tyre rate k_t is declared
```

At RG = 0.78 deg/g and 1.5g: φ = 1.17°, z_roll = 12.4 mm. With a tyre rate, each axle's roll stiffness is the springs-plus-bars value in series with the tyres' `k_t t²/2` (`series_roll_stiffness`).

### 14.3 Camber Gain and Static Camber

Camber to the road at the design case is `γ_static + g·z + φ`. Holding the loaded outside tyre at its optimum gives the gain and the static camber together:

```
g          = (γ_opt − φ − γ_static) / z          [deg/mm]   camber_gain_needed
γ_static   = γ_opt − φ − g · z                   [deg]      static_camber_needed (an alignment, no hardpoint moves)
error      = γ_static + g·z + φ − γ_opt                     loaded_camber_error
```

At −2.5° static the gain is −0.040 deg/mm; at the declared −1.5° it would be −0.12 deg/mm.

### 14.4 Ride-Frequency Floor and Anti Floors

Under a longitudinal acceleration the spring must absorb the part of the load transfer the anti-geometry does not, inside the jounce budget left after roll and downforce:

```
ΔFz   = m · g · a_x · h / (2L)                          per-wheel transfer (N)
room  = b − z_aero − z_roll(a_y)                         jounce budget left (mm)
k_min = (1 − A) · ΔFz / room
f_min = (1 / 2π) · √(k_min / m_s)
```

with A the anti fraction and m_s the sprung corner mass. Inverting it gives the smallest anti that lets a declared ride frequency hold the manoeuvre inside the budget:

```
A_min = 1 − (2π f)² · m_s · room / ΔFz                  anti_floor_pct
```

On `front_v11_final` and `rear_v11_final` (anti-dive 68.9% at 73% bias, anti-squat 46.7%, every case at 1.06g): front 1.42 Hz straight and 2.00 Hz combined, rear 1.77 and 2.50 Hz (`test_target_derivation.py`).

**Derived ride frequency** (`ride_frequency_derived`): the combined-case floor evaluated with peak μ raised by a 10% allowance while the tyre is unmeasured. A frequency above its floor costs grip over bumps and buys no travel.

**Roll gradient** (`roll_gradient_ceiling`, `roll_gradient_for_friction`): with the springs fixed by the ride frequencies, the largest roll gradient (bisection) at which the combined case still fits the budget at a given μ.

**Springs-only roll gradient** (`springs_only_roll_gradient`):

```
k_wheel = (2π f)² · m_s                          per corner
K_axle  = k_wheel · t² / 2                       (N·m/deg after unit conversion)
M/g     = m · g · (h − h_ra),   h_ra = RC_f + (RC_r − RC_f) · (1 − w_f)
RG      = (M/g) / (K_f + K_r)
```

At 2.6 Hz front and 3.0 Hz rear: 205 and 300 N·m/deg against 686 N·m per g, 1.36 deg/g; the bars carry the rest of the 0.78 deg/g target.

**Aero heave** (`aero_heave_mm`): `z = ½ ρ ClA v² · axle_share / 2 / k_wheel`, taken out of the jounce budget before roll and pitch (6.9 mm front at ClA 2.6 m², 60 km/h, 16.0 N/mm).

### 14.5 Lateral Load-Transfer Distribution

```
front share = [ w_f · RC_f + s_f · (h − h_ra) ] / h          front_lltd_share
s_f         = (target · h − w_f · RC_f) / (h − h_ra)         stiffness_share_for_lltd
```

with s_f the front roll-stiffness share. Differentiating this share model with respect to each roll centre gives the resolution of the roll-centre band:

```
d(share)/d(RC_f) = w_f (1 − s_f) / h          0.082 points per mm
d(share)/d(RC_r) = −s_f (1 − w_f) / h         −0.097 points per mm
```

so ±18 mm is ±1.5 points at the front and ±1.7 at the rear. `share_per_mm_front_rc` gives a variant that charges the geometric transfer to the sprung masses only, `(m_s,front − s_f w_f m_s) / (m h)`, 0.068 points per mm; it is used to size roll-centre changes in `robustness.balance_under_tire_change`.

### 14.6 Neutral Balance from Load Sensitivity

The synthetic tyre's peak μ falls with load (`mu_of_load`): μ(Fz) = 1.66 − 0.0002 (Fz − 550), 1.66 at 550 N to 1.44 at 1650 N. With the lateral transfer `LT = m g a_y h / t` split by the front share s:

```
C_f = Σ_{±} μ(W_f ± s·LT) · (W_f ± s·LT) / (2 W_f)
C_r = Σ_{±} μ(W_r ± (1−s)·LT) · (W_r ± (1−s)·LT) / (2 W_r)
capacity ratio = C_f / C_r        (< 1: understeer)
```

The **neutral share** is where the ratio is 1 (bisection): 51.1% at 1.5g. The 53% target makes the front saturate first by 0.7% of axle capacity, a controllability choice.

**Tyre heat** (`robustness.balance_under_tire_change`): the same balance with one axle's μ scaled; it reports the new neutral share and the front roll-centre (hardpoint) or front-bar (set-up) change that would restore the margin.

### 14.7 Design Corner Loads (`corner_loads`, `steer_warp`)

Each vertical term is computed separately: static, longitudinal and lateral transfer, bank and grade, downforce and steer warp. For a bank β (outside of the turn raised) and grade (uphill > 0):

```
normal = m (g cos β cos(grade) + a_y sin β)
lat    = m (a_y cos β − g sin β)
lon    = m a_x − m g sin(grade)
static_i  = normal · (w_f or 1 − w_f) / 2
Δlon      = lon · h / L / 2                   (+ front, − rear)
Δlat      = lat · h / t,  split by the front share (front ± s·Δlat, rear ± (1−s)·Δlat)
aero_i    = downforce · (aero front share or rear share) / 2
```

**Steer warp:** at fixed wheel travel a steered wheel's patch moves vertically through caster and kingpin inclination (to first order `sin δ · (s sin(caster) + t sin(KPI))`), plus, when the pushrod rides on the upright, the spring compression steering puts in, as wheel travel over the motion ratio. The difference across the axle is a roll input `φ = (dz_R − dz_L)/t_f` resisted by the front and rear roll stiffnesses in series:

```
M_warp = φ · K_f K_r / (K_f + K_r)
ΔF_front pair = M_warp / t_f,    ΔF_rear pair = M_warp / t_r      (diagonal: steering left loads FL and RR)
```

At 1.5g on the flat the outer front carries 1248 N (FL, FR, RL, RR: 165, 1248, 285, 1245 N).

### 14.8 Acceptance Bands

| Channel | Band | Basis |
|---------|------|-------|
| Toe | ±0.08° | 1% of the 8.1° peak slip angle |
| Camber | ±0.30° | at most 0.11% of peak grip at the synthetic sensitivity |
| Roll-centre height | ±18 mm | ±1.5 points of front load-transfer share at the front, ±1.7 at the rear (§14.5) |

### 14.9 Sensitivity to the Tyre Inputs (`tire_sensitivity`)

Each tyre input is moved by ±25% and the camber sensitivity from half to five times, the design accelerations scaling with peak μ. For each case and corner it reports the loaded camber error, the static camber that restores the optimum, the grip that error costs, and whether the corner's anti-geometry still clears the ride-frequency floor.

---

## 15. Inverse Synthesis (`inverse_genesis.py`, `genesis_walls.py`, `genesis_seed.py`)

### 15.1 Design Vector and Residual

The design vector x is the displacement of the chosen pickups from a seed geometry, each held inside a legal box (`LegalVolume`). The forward map (`curves_of`) solves §1.1 at each travel station (and, for a steered corner, each rack position) and returns camber, toe, roll-centre height and scrub. Each channel c has a target T_c(t) and band B_c(t), and the residual is normalised by the band:

```
r_i = ( y_c(t_i; x) − T_c(t_i) ) / B_c(t_i)
acceptable  ⇔  max_i |r_i| ≤ 1
```

Acceptance is a feasibility test, not a weighted sum: there are no arbitrary weights and no Pareto front.

### 15.2 Levenberg-Damped Gauss-Newton (`genesis_solve`)

```
J_ij  = ( r_i(x + h e_j) − r_i(x − h e_j) ) / 2h        h = 0.25 mm; a probe that fails to solve stops the search (a cliff, not a number)
(JᵀJ + λ · diag(JᵀJ)) Δx = −Jᵀ r
x_try = clamp(x + Δx)                                   into the legal boxes
```

A trial step that crosses any wall (§15.3) or fails to solve is **refused, not penalised**: λ ← 10λ and the step is retried, up to 8 times per iteration. A step is accepted when the cost `rᵀr` falls or every station is in band; then λ ← max(λ/3, 1e-6). The solve stops when every station is in band, after 30 iterations, or when no legal step lowers the cost. It is deterministic.

A start that already violates a wall is refused and reported, never repaired. The search runs from six starts, the seed and five deterministic samples inside the boxes. It is local: a NO_FIT means none of the starts found a fit, not that none exists.

### 15.3 Walls

All are absolute and evaluated on every trial geometry:

- **Keep-outs**: frame tubes as capsules with a probe radius and clearance; signed distance must stay positive.
- **Solved-property bounds** (`PropertyBound`, `SolvedPropertyBounds`), evaluated at the static state: caster, KPI, static scrub, mechanical trail, anti-dive, anti-squat, anti-lift, roll-centre migration (least-squares slope over travel, body frame), compliance camber, toe, caster and KPI under a declared cornering case and toe under a declared braking case (§18), joint-lash toe and camber deadband (§18.3), toe from a declared frame twist, the largest member force over the wheel load, the steering ratio, effort margin and lock (§16.4), joint swing and misalignment at lock, and Ackermann at lock (§16.5).
- **Wheel envelope** (`WheelEnvelope`): the rim and tyre rebuilt at every state from the solved camber and toe, as an annulus from the rim bead to the tyre radius tapering from flange width to section width; no link, swept as a tube of its radius, may enter it, and every outboard pickup and its rod-end housing must sit inside the upright envelope, at every station and both steering locks.
- **Node attachment** (`NodeAttachment`): chosen chassis pickups within a declared bracket reach of a frame node.
- **Joint misalignment** (`joint_misalignment`): each rod end's swing measured in the frame of the body its housing is fixed to (Kabsch alignment of that body between states), against its rating.
- **Vehicle walls** (`genesis_walls`): straight-line deceleration at the declared brake bias on the declared tyre (`BrakeDecel`, bisection on §19.1); the lowest link buckling or yield factor under the shock cases (`LinkStructure`, §17.4); the rocker bearing static safety factor (`RockerBearingWall`, §17.5).

### 15.4 Start Search (`genesis_seed`)

When every drawn start is refused, a deterministic pattern search over the same pickups and boxes minimises

```
cost = max_i |r_i| + Σ wall penalties
```

(a violated property bound adds its violation over a tenth of the bound, capped at 10; wheel clearance below target adds the shortfall over the target; keep-out and spacing breaches add their depth over 10 mm). Its result is only a start: InverseGenesis is re-run from it with every wall absolute, and that run is the manifest.

### 15.5 Build Yield and Verdicts (`build_yield`, `_verify_yield_full`)

The tolerance field perturbs every linkage point on uniform per-axis bounds (jig-weld preset: ±0.5 mm at chassis tabs, ±0.15 mm at machined points). The candidate's own fit residual is added to the propagated scatter, because the fit has already spent part of the band:

```
δ_k ~ U(field),  k = 1..N
pass_k = max_i | r_fit,i + (J δ_k)_i | ≤ 1
yield  = (1/N) Σ pass_k                                first order
```

A subsample is re-solved through the full kinematics (perturbed geometries that fail to solve count as fails) and the linear and full pass/fail decisions must agree on at least 98% of builds. A worst-case linear bound, `W = max_i( |r_fit,i| + Σ_j |J_ij| h_j )` with h_j the field half-widths, gives 100% yield whenever W ≤ 1; the reported yield is the sampled one above.

| Verdict | Yield |
|---------|-------|
| RESILIENT | ≥ 95% |
| TEMPERED | ≥ 80% |
| KNIFE_EDGE | < 80%, or the sensitivity probes fail to solve |
| NO_FIT | no start reaches every band |

The winner is the **highest-yield** hit (ties by fit), not the best fit; the yield the best fit forfeits is reported as the resilience premium.

### 15.6 Reproducibility (`genesis_repro`)

A manifest holds the seed hardpoints at full precision, targets, bands, boxes, tolerance field, every wall and the search constants, hashed (SHA-256) as one input set. An edited manifest is refused; a re-run must match the recorded output to 1e-6 mm in every coordinate and 1e-9 in every fit and yield.

---

## 16. Steering (`steering_feel.py`, `inverse_genesis.SteerEffort`, `dynamic_ackermann.py`, `steering_response.py`)

### 16.1 Mechanical Trail

Ground-level trail from the kingpin intercept with the road plane through the contact patch (§1.5), x rearward:

```
t        = (cp[z] − lo[z]) / (uo − lo)[z]
ground   = lo + t · (uo − lo)
t_mech   = cp[x] − ground[x]                  (inverse_genesis.mechanical_trail_mm)
t_mech  ≈ R · tan(caster) − spindle offset    (steering_feel.mechanical_trail_mm)
```

### 16.2 Pneumatic Trail (cosine law)

```
t_p(α) = t0 · cos( (π/2) · α / α0 )
t0     = l / 6,   l = Fz / (p · w)          brush theory, p = 83 kPa, w = 180 mm
```

α0 is the slip of peak lateral force at that load, so the trail falls through zero at the force peak and goes negative past it. At the outer wheel's 1248 N, t0 = 13.9 mm. When an outer-wheel t0 is given, the inner wheel's is scaled by load.

### 16.3 Aligning Torque and Limit Feel

```
M(α) = |Fy(α)| · ( t_mech + t_p(α) )          per wheel, about the steering axis
M_axle(α) = M_outer(α) + M_inner(α)           both wheels at the same slip
```

`limit_feel` reports the peak torque and its slip, the torque at the limit `M(α0) = |Fy,peak| · t_mech`, the warning margin `α0 − α(M_max)`, and the lightening `1 − M(α0)/M_max`.

### 16.4 Steering Ratio Window (`SteerEffort`, `ratio_window`)

```
r_k     = (360 / c) / |dδ/dx|                       kinematic ratio at centre; c = rack travel per wheel turn (mm/rev),
                                                    dδ/dx road-wheel steer per mm of rack by central difference on the linkage
r_eff   = max_α M_axle(α) / T_max                   ratio the effort limit needs (worst over slip, not at the limit)
effort:  r_k − r_eff ≥ 0
lock:    360 · x_lock / c ≤ 120°                    x_lock: rack travel at which the inner (or mean) wheel reaches lock, by bisection
```

The worst torque comes before the limit because the pneumatic trail peaks before the force does. `trail_limit_mm` finds, by bisection, the zero-slip trail at which the window closes (22.4 mm on `front_v11_final`).

### 16.5 Ackermann (`dynamic_ackermann.py`)

```
cot δ_i,geo = cot δ_o − t_k / L                     geometric split (t_k kingpin track)
Ackermann%  = 100 · (δ_i − δ_o) / (δ_i,geo − δ_o)   100 geometric, 0 parallel
dynamic target: δ_i − δ_o = (δ_i,geo − δ_o,geo) + (α*_i − α*_o)
```

with α* each tyre's peak-force slip at its load. `axle_force_at_split` prices a fixed split as front axle force lost against the optimum, the maximum over the common sideslip.

### 16.6 Steering Response (`steering_response.py`)

Linear single-track car, unknowns v, r, Fyf, Fyr per frequency, steer δ = 1:

```
m (s v + V r) = Fyf + Fyr
Iz s r        = a Fyf − b Fyr
s Fyf = (V/σ) ( Cf (δ − e_f G Fyf − (v + a r)/V) − Fyf )
s Fyr = (V/σ) ( Cr (−(v − b r)/V) − Fyr )
a_y   = (Fyf + Fyr) / m
lag   = −phase / (360 f)
```

e_f is compliance steer (rad per N of front axle force) and G the joint-stiction describing function, `(k/k_eff) e^{−jφ}`. Lateral acceleration beyond V²/R is §19.4.

### 16.7 Upright Stiffness for the Caliper Torque (`robustness.upright_brake_requirement`)

```
T_caliper = bias_f · m · g · a_x · R / 2           per front wheel (N·m)
k_upright ≥ T_caliper / Δcaster_limit               (0.1° by default)
```

---

## 17. Corner Statics and Structure (`loadpath.py`, `robustness.py`, `actuation.py`, `halfshaft.py`)

### 17.1 Two-Force-Member Statics (`solve_member_forces`)

Every link is a pin-jointed two-force member, so the upright is held by six axial force lines (UF, UR, LF, LR, TR, PR) and six equilibrium equations give every force:

```
Σ_k T_k û_k + F = 0
Σ_k (p_k − cp) × T_k û_k + M_applied = 0
```

T > 0 is tension; û runs from the member's outboard point toward its inboard point. F is the contact-patch load (Fx, Fy, Fz) on the wheel; M_applied (`_applied_moment`) collects the aligning torque Mz, a longitudinal force reacted by the shaft (`Fx_wc`, applied at the wheel centre for inboard drive or inboard brakes), and any extra hub forces and moments. Without a pushrod a vertical strut proxy is substituted and flagged; a near-singular equilibrium matrix is flagged.

### 17.2 Two-Body Statics (`_solve_wishbone_pushrod`)

When the pushrod acts on a wishbone rather than the upright, the upright is held by the other wishbone's two legs, the tie rod and the carrying wishbone's ball joint (any direction): six unknowns on the upright. That wishbone's moment about its own pivot axis then fixes the pushrod force, and the force across each of its legs and the arm to the mount are reported so the leg bending can be screened (§17.4).

### 17.3 Load Cases (`genesis_repro.vehicle_load_cases`, `robustness.shock_cases`)

Compliance and structure cases, all at the outer wheel:

```
cornering:     Fz = static + share · m g a_y h / t,   Fy = −a_y · Fz   (toward the turn centre)
               Mz = −|Fy| · t0 · cos(π α / 2 α0)       the declared tyre's own aligning moment at that Fy (zero past peak)
braking 1.5g:  Fz = static ± m g a_x h / L / 2         (+ front, − rear),   Fx = bias · m g a_x / 2
combined 1.06g, 3g vertical bump, kerb 2g + 1g, and traction cases at the driven axle
```

Shock cases (`shock_cases`): braking 1.8g on the dynamic vertical load; a 3g curb strike with a 0.3 kick longitudinally and laterally, outboard and, as a separate case, inboard; the curb strike during the 1.8g stop (front) or during a 1.06g traction exit (rear). Rear braking and traction forces with inboard brakes and drive act at the wheel centre (`Fx_wc`).

### 17.4 Link Margins (`member_margins`)

```
Compression:  P_cr = π² E I / L²                         (pinned ends)
              σ_peak = (P/A) · [ 1 + (e c / r²) / (1 − P/P_cr) ],   e = crook · L  (L/1000)
Tension:      factor = σ_y A / F
Leg bending:  σ += (M / Z) · 1 / (1 − P/P_cr)             for a leg that carries the pushrod mount
buckling factor = P_cr / P,   yield factor = σ_y / σ_peak      (the LinkStructure wall takes the lower of the two over every link and case)
```

### 17.5 Rocker Bearings (`actuation.rocker_bearing_loads`)

For each case: the pushrod force acts at the rocker's pushrod point; the spring force follows from moment balance about the rocker axis k̂; the pivot carries the rest, `R = −(F_p + F_s)`, split into axial `F_a = |R·k̂|` and radial `F_r`. The moment about axes normal to k̂ is a couple M over the bearing spacing s:

```
F_r,bearing = F_r / 2 + M / s
P0 = max( F_r,bearing, X0 F_r,bearing + Y0 F_a )
s0 = C0 / P0                                          (required ≥ 2.0)
```

### 17.6 Actuation Synthesis (`actuation.py`)

Actuation is a second stage: the pushrod rides on the lower wishbone (or an upright bracket) and only reads its motion, so it cannot change camber, toe or roll centre. Targets are derived:

```
MR_max    = u · S / T                  usable stroke fraction u (0.80), damper stroke S, total wheel travel T
MR_static = MR_max · (1 − margin)      margin 0.07
spread    = (MR_max,travel − MR_min,travel) / MR_static ≤ 0.10      so k_wheel ∝ MR² stays near constant
```

plus bounds on the out-of-plane angle of the pushrod and spring to the rocker plane, the transmission angles away from toggle, damper top-out and bottom-out, clearances and node reach, each a hinge residual in units of its tolerance, solved by bounded least squares.

### 17.7 Halfshaft Loads on the Upright (`halfshaft.py`)

```
tripod plunge friction:  F_ax = μ T / r          (both signs, opposing the plunge; μ declared)
CV secondary couple:     M = T · tan(β / 2)      about the normal to the plane of the two shafts
```

---

## 18. Compliance, Frame and Joints (`elastokinematics.py`, `compliance_margin.py`, `frame_fe.py`, `joint_wear.py`, `setup_checks.py`)

### 18.1 Pickup Stiffness and the Elastokinematic Solve

Each chassis pickup has a stiffness (`StiffnessField`): a declared scalar, a 3×3 matrix in the corner frame (for example condensed from the frame model of §18.5), or built from pull tests, with its provenance recorded. A link in tension pulls its pickup toward the upright:

```
d_pickup = K⁻¹ · ( −T û )
```

`compliance_jacobian` is the linear map from pickup displacement to camber, toe, caster and KPI, by central differences with link lengths held. `solve_elastokinematic` applies the wheel load in steps; within each step it re-solves the member forces on the geometry deflected so far, turns them into pickup deflections and link length changes, and iterates to a fixed point. It returns the non-linear result beside the Jacobian prediction, so the adequacy of a linear correction is measured each time.

### 18.2 Joints, Tabs and Arms in Series (`compliance_margin.py`)

One declared stiffness per link end, along the link; two ends in series, and in series with any declared tab stiffness:

```
k_ends    = k_end / 2
k_tab,eff = 1 / (1/k_tab + 1/k_ends)
```

`required_end_stiffness` finds, by bisection, the end stiffness below which each compliance wall (cornering and braking toe) breaks: the number a joint, tab and arm design must exceed and a pull test measures.

### 18.3 Free Play (`lash_deadband`)

```
deadband_c = Σ_m | ∂c / ∂L_m | · clearance_m        every joint at the end of its play in the adverse direction
```

### 18.4 Joint Wear (`joint_wear.py`) — Archard

```
h = K · p · s,   p = F / (d w),   s = r θ           wear depth per cycle
clearance growth = 2h per cycle                     (the load reverses)
```

K is the liner's specific wear rate (mm³/(N·m)), declared; `fit_wear_rate` fits it from clearances measured at intervals. The growth feeds §18.3 to give the distance at which wear consumes the toe band.

### 18.5 Frame Beam Model (`frame_fe.py`)

Two-node Euler-Bernoulli space-frame elements (12 DOF: axial, torsion, two bending planes) with circular tube sections and rigid joints, built from the STEP tube axes (ends within `merge_mm` share a node, T-joints get a node on the receiving tube, bends become chords). `K u = f` with fixed DOF; `condensed_compliance` is the flexibility at chosen DOF by unit loads; `torsional_rate` holds one axle's nodes and twists the other by a couple (N·m/deg); `attach` adds a pickup node joined to frame nodes so its stiffness is derived rather than declared. No joint, tab or gusset flexibility, ovalisation or panels: every stiffness is an upper bound on the built frame's.

### 18.6 Thermal Fit (`setup_checks.thermal_fit`)

```
span growth = (α_upright − α_pin) · L · ΔT
bore change = (α_upright − α_pin) · d · ΔT        positive: the clearance opens
slot margin = slot clearance / span growth
```

---

## 19. Tyre Use, Yaw Moment and Torque Distribution (`tire_utilisation.py`, `torque_distribution.py`)

### 19.1 Instantaneous Tyre Utilisation (`tire_state`, `limit_envelope`)

```
u      = √( (Fx / (k_x μ Fz))² + (Fy / (μ Fz))² )     μ the load-sensitive peak μ, k_x = μx/μy (declared 1.0)
margin = 1 − u
spare  = |F| · (1/u − 1)
```

Forces at a quasi-steady (a_x, a_y): Fz from §14.7; braking split by the declared front bias, equal left and right; traction at the rear only, equal split; lateral by steady yaw balance (front axle carries w_f of m·a_y), within an axle a common slip angle solved on the tyre model (grid to 30°, then bisection), each tyre's Fy scaled by `√(1 − (Fx / (k_x μ Fz))²)`. An axle that cannot carry its share at any slip makes the state infeasible. `limit_envelope` bisects, in each direction from braking (0°) through cornering (90°) to traction (180°), for the largest acceleration before the first tyre reaches u = 1 (tolerance 1e-4 for the peak-μ sweep).

### 19.2 Torque Distribution and Vectoring (`torque_distribution.py`)

Force and yaw balance at every instant:

```
Fy_f + Fy_r = m a_y
a Fy_f − b Fy_r + Mz,x = Iz ṙ
Mz,x = (Fx,out − Fx,in) · t / 2             yaw moment of unequal rear forces
```

Rear forces written `F_R/2 ± d`:

| Option | d |
|--------|---|
| open | 0 |
| clutch LSD, TBR k (2 or 3) | in [F_R/2 · (1−k)/(1+k), 0] under drive |
| spool | −c_x Fz t κ / 2 (declared slip stiffness), capped at each tyre's grip |
| torque vectoring | free, each wheel within a declared motor force, drive and regeneration |

Drag, rolling resistance and the rearward component of the front lateral force at the steer angle enter the longitudinal balance. Each state takes the split that minimises the largest utilisation (scan, then golden section). `simulate_corner` runs a straight, clothoid, arc and mirrored exit by forward and backward passes at the largest tangential acceleration the allocation allows.

### 19.3 Yaw Moment Budget (`yaw_budget`)

The twelve causes of yaw moment about the CG, the lateral force, longitudinal force and aligning torque of each tyre i at (x_i, y_i), x forward and y left:

```
N = Σ_i ( x_i Fy,i − y_i Fx,i + Mz,i ),     Mz,i = −Fy,i · t_p(α_i)   (§16.2)
```

The eight force terms must equal Iz ṙ: zero in a steady corner, positive to build yaw rate, negative to unwind it.

### 19.4 Lateral Acceleration Beyond V²/R

```
a_y = V · (r + β̇),    β = v / V
```

Only at constant sideslip is all of it V r = V²/R; whenever sideslip changes, V β̇ is lateral acceleration without rotation.

---

## 20. Measurement and Uncertainty (`transducer.py`, `calibration.py`, `strain_thermal.py`)

### 20.1 Strain-Gauged Link (`transducer.py`)

```
ε_axial   = F / (E A)
ε_bending = M c / (E I)
γ_torsion = T c / (G J),      G = E / (2(1 + ν))
V_o / V_i = (GF/4)(ε1 − ε2 + ε3 − ε4)  =  (GF/4) · 2(1 + ν) · ε      full Poisson bridge
V_max     = 2 n √(R A P')                                           self-heating excitation limit
lead loss = 2 R_lead / (R_bridge + 2 R_lead)                         without remote sense
dynamic amplitude error = 1 / (1 − r²) − 1,   r = f / f_n             lightly damped member
```

`rejection_monte_carlo` estimates bending and torsion leakage for a gauge layout under bonding tolerances.

### 20.2 Calibration Reduction (`calibration.certificate_reduction`, `calibration.calibrate`)

**Certificate reduction** (the acceptance definition), from the mean increasing and decreasing response at each load over the runs or orientations:

```
A(F)     = mean increasing response − its value at zero               span response
B(F)     = m₁ F² + m₂ F                                               second-order polynomial through zero, least squares on A
NL(F)    = 100 (B − A) / A(F_max)                                     non-linearity, % of span
H(F)     = 100 (mean increasing − mean decreasing) / A(F_max)         hysteresis, % of span
F(y)     = m₁' y² + m₂' y                                             inverse polynomial used in service
```

Acceptance: non-linearity and hysteresis each ≤ 0.25% FS, reproducibility between orientations ≤ 0.10% FS. `loading_schedule` generates the loading cycle itself: three preloads, five equal steps up and down with 30 s holds, and one preload before each further orientation.

**Straight-line reduction** (`calibrate`), used for the uncertainty estimate: a best-fit line through every ascending and descending point gives sensitivity and zero. Non-linearity is the largest deviation of the ascending mean from the line, hysteresis the largest ascending-descending difference at one load, repeatability the largest spread between runs at one load and direction, each in % of full scale. The standard uncertainty combines them as rectangular distributions with the reference cell:

```
u = √( (NL/√3)² + (H/(2√3))² + (Rp/(2√3))² + (u_ref · FS)² ),     U = 2u   (k = 2)
```

`bending_rejection`, `zero_drift`, `temperature_coeff` and `installation_offset` reduce the other pre-registered checks.

### 20.3 Radiant Gradient on a Gauged Tube (`strain_thermal.py`)

Thin ring of radius r, wall t, conductivity k, absorbing `a q max(cos θ, 0)` on one side, convection h on both faces:

```
T1  = (a q / 2) / (2h + k t / r²)          first-harmonic amplitude; hot-to-cold difference ≈ 2 T1
τ   = ρ c t / (2h + k t / r²)
apparent strain = (gauge compensation mismatch) × (temperature difference between arms)
```

Two sensors on a diameter read the full amplitude only when aligned with the source (cos φ otherwise); three at 120° resolve amplitude and direction (`first_harmonic_from_three`).

### 20.4 GUM Budget

Every term is combined in quadrature after conversion to a standard uncertainty (Type B rectangular: half-width / √3), and expanded at k = 2. The budget is a pre-calibration estimate, every term Type B, to be replaced by measured values once the links are calibrated.

---

## 21. Aerodynamics Beyond the Solvers (`aero/`)

### 21.1 Underfloor Channel (`aero/underfloor.py`)

A floor makes downforce as a duct, not a wing: the gap converges to a throat, the flow speeds up, and the diffuser recovers the pressure. Quasi-1D incompressible continuity plus Bernoulli on the gap area A(x):

```
A_flow  = min( A_exit, 1.15 · A_inlet )            the base wake sets the exit flow; the inlet may over-capture by 15%
v/V∞    = A_flow / A(x)
Cp(x)   = C_d · (1 − (A_flow / A(x))²)             up to the throat (C_d = 0.75, a loss factor on the suction)
Cp(x)   = Cp_throat · (1 − η · ξ)                  aft of the throat, ξ from 0 at the throat to 1 at the exit, η = 0.80
L       = Σ Cp · q · w(x) · Δx,    q = ½ ρ V²,    C_L = L / (q · A_ref)
```

A constant-area duct gives Cp = 0 at any C_d. Because the inlet and throat both shrink as the car is lowered, their ratio, and so the suction, saturates instead of diverging as thin-surface theory does. The diffuser is screened by its equivalent conical half-angle, `atan((r_exit − r_throat) / length)` with r = √(A/π); above 7° the attached-flow recovery is flagged as optimistic. No spanwise variation, edge vortices or yaw. This module has no dedicated test yet.

### 21.2 Aero Map and Lap-Sim Coupling (`aero/aeromap.py`, `aero/coupling.py`, `aero/backends.py`)

The map stores converged coefficient results over attitude (roll, pitch, yaw, ride height, speed), interpolates multilinearly on the axes that varied, clamps to the swept envelope and returns no value for a channel nobody solved. The lap sim works in area-coefficients:

```
F = ½ ρ (C·A) v²,      cl_a = −C_L · A,      cd_a = C_D · A        (C_L negative = downforce)
```

Attitude comes from the same vehicle model: roll from §2.1; pitch from the longitudinal transfer through the wheel rates, less the anti share, `θ = atan((δ_r − δ_f)/L)` with `δ_f = (ΔW/k_f)(1 − anti-dive)` under braking; ride height from the downforce over the ride rate.

The analytic reference model (`ReferenceAeroModel`, labelled `POTENTIAL`, uncorrelated) is a trends-only surrogate: `C_L = C_L0 + k_yaw|ψ| + clamp(−k_h (h_ref − h), ±0.6)`, induced drag `0.15 (C_L² − C_L0²)/|C_L0|`, side force proportional to yaw, and front balance shifting with pitch.

### 21.3 Wind Tunnel, Pressure Taps, Balance and PIV (`aero/windtunnel.py`, `aero/pressure_tap.py`, `aero/daq.py`, `aero/piv.py`)

- **Like-for-like correlation:** the CFD sweep is run at the tunnel's exact front and rear ride heights and speed; per point `error% = 100 (C_CFD − C_tunnel)/|C_tunnel|`, and the map RMS of the C_L error is held to a declared tolerance (5% per point, 4% RMS by default).
- **Pressure taps:** `p = s (V − V0)`, `q = ½ρV²` (or the measured pitot q), `Cp = (p − p∞)/q`. Sectional normal force `C_n = ∫ (Cp_pressure − Cp_suction) d(x/c)` over the chord both surfaces cover (trapezoidal). Stall is read from the suction surface: the recovery slope dCp/d(x/c) aft of the suction peak; a flat plateau below the declared slope marks separation. CFD against taps by `RMSE = √(mean (Cp_CFD − Cp_tap)²)` at the same points.
- **Six-component balance:** `loads = C · (V − V_zero)` with C the 6×6 interaction matrix; an uncalibrated or ill-conditioned C, or a railed bridge, gives NaN rather than a raw channel. Fan blade-pass tones and structural resonances are notched before averaging, with the removed variance reported.
- **PIV:** each interrogation window of the two frames is cross-correlated; the integer peak is refined by a three-point Gaussian fit, `δ = (ln c₋ − ln c₊) / (2 (ln c₋ − 2 ln c₀ + ln c₊))`; velocity = pixel shift × magnification / Δt. Seeding is checked by the Stokes number `St = τ_p / τ_f` and the quarter-window rule on the expected shift `U Δt / magnification`.

### 21.4 Scale Models (`aero/scale_model.py`)

```
ν(T)   = μ(T) / ρ(T),   μ = 1.716e-5 (T/273.15)^1.5 (273.15 + 110.4)/(T + 110.4)  (Sutherland),   ρ = 101325 / (287.05 T)
Re     = V L / ν
V_model = V_full / scale                                  Reynolds match
```

When the tunnel cannot reach the matched speed the residual Re mismatch is reported. Build tolerances become coefficient uncertainty by first-order sensitivities (chord 1:1 on C_L and 0.5:1 on C_D, camber/leading edge 3:1 and 2:1, span 1:1 and 0.3:1, surface waviness 0.2:1 and 2.5:1, each as a fraction of the scaled dimension), combined root-sum-square.

---

## 22. Brakes, Pedal Box and Throttle (`brake_bias.py`, `brake_thermal.py`, `pedal_box.py`, `throttle_return.py`, `throttle_dynamics.py`)

### 22.1 Brake Bias from the Hydraulics (`brake_bias.py`)

```
P_ax  = F_pedal · ratio · bar_ax / A_mc,ax           line pressure per circuit
N_ax  = P_ax · A_piston,ax                           clamp force, one side of an opposed caliper
T_ax  = n_cal · 2 · μ_ax(T) · N_ax · r_eff,ax        two pad faces
bias  = T_f / (T_f + T_r)
```

μ_ax(T) is the nominal pad friction times the fade law of each axle's rotor temperature, so the bias is followed through a stint. Caliper deflection, hose swell and fluid expansion change pedal travel, not bias, because each circuit's pressure at equilibrium is still pedal force over master-cylinder area.

### 22.2 Balance Bar and Pedal Travel (`pedal_box.py`)

```
F_front = F_rod · b / L,   F_rear = F_rod · a / L     a, b: pivot to front/rear clevis, a + b = L
r_eff   = (r_o + r_i) / 2                              swept pad band
```

`bias_authority` sweeps the bar through its adjustment to give the reachable bias band and the change per turn. **Pedal travel** adds every consumer of fluid and converts it to travel at the pad:

```
V = V_knockback + V_pad + (caliper + hose + hardline compliance) · P + V_fluid · P / K + V_air · (1 − 1/(1 + P))
stroke = V / A_mc,      travel = (stroke + free play) · pedal ratio
```

(K the fluid bulk modulus; trapped air compressed isothermally from 1 bar.) Travel past the available pedal stroke, or master-cylinder stroke past its limit, fails. `stack_up` breaks the installed length into named segments and prices each shortening lever in pedal force or travel.

### 22.3 Rotor Thermal (`brake_thermal.py`)

```
single stop:   Q = ½ m v₀² · (1 + Σ I_i / (r² m)),   ΔT = Q · bias · η_rotor / 2 / (m_rotor c_p)
endurance:     P̄ = h A (T − T∞) + ε σ A (T⁴ − T∞⁴)        solved for T by Newton
transient:     C_r dT_r/dt = q_rotor − q_cond − h A (T_r − T∞) − q_rad;   C_p dT_p/dt = q_pad + q_cond − ½ h A_pad (T_p − T∞)
```

The convection coefficient combines a rotating-disc and a cross-flow correlation, `h = √(h_rot² + h_x²) · duct gain` (floor 12 W/m²K), with `Nu_r = 0.015 Re_r^0.8`, `Re_r = ω r² / ν` with the rotor turning with the wheel, `ω = v / R_wheel` (228 mm by default), and `Nu_x = 0.037 Re_x^0.8 Pr^(1/3)` on the rotor diameter. The design loop of §12 thins the ring until the pad and fluid limits bind.

### 22.4 Throttle Return Redundancy (`throttle_return.py`)

Spring rates are measured (`k = F/x`, `k = ΔF/Δx`, `k_θ = T/θ`) or estimated from geometry, `k = G d⁴ / (8 D³ N_a)`. With the worst single spring removed, the net closing torque (springs less friction, stiction and cable drag) must stay positive over the whole travel, with margin. The snap-closed model integrates the plate's rotation under the spring torque, friction and an aerodynamic torque `T_aero = C q A_plate r_plate f(θ)`, with `I_plate ≈ ⅓ m r²` plus pedal and cable mass reflected to the shaft, through any backlash band. The brake pedal is screened at 2000 N against FoS ≥ 1.5 on yield.

### 22.5 Throttle Plate and Plenum Dynamics (`throttle_dynamics.py`, `throttle_flutter_cosim.py`)

```
ṁ = C_d A(θ) p_up / √(R T_up) · √( 2γ/(γ−1) · ( pr^(2/γ) − pr^((γ+1)/γ) ) ),   pr = max(p_down/p_up, (2/(γ+1))^(γ/(γ−1)))   (choked below the critical ratio)
dp/dt = (R T / V_plenum) (ṁ_in − ṁ_out)
A(θ) ∝ 1 − cos θ  (normalised to the bore at 90°, plus a leak area)
```

Plate angle, rate and manifold pressure are integrated together, the vacuum acting back on the plate as a torque. **Flutter screen:** `I θ̈ + (c_struct + c_aero(V)) θ̇ + k_θ θ = 0`, `c_aero(V) = c_aero,ref (V/V_ref)²`; negative total damping is flutter. The aerodynamic damping derivative is the in-phase component of the moment in a forced-oscillation CFD case (`throttle_flutter_cosim`); without one, only the structural stability is reported.

---

## 23. Tyre Thermal, Fitting and Field Tests (`tire_thermal.py`, `thermic_patch.py`, `tirefit.py`, `tire_field.py`, `degradation.py`)

### 23.1 Lumped Tyre Thermal Network (`tire_thermal.py`)

Tread band(s), carcass and inflation gas, explicit Euler:

```
Q_fric = |Fy · V_x tan α| + |Fx · V_x κ|,     Q_roll = C_rr Fz V_x
C_tread dT_t/dt = Q_gen − G_tc (T_t − T_c) − h(V) A (T_t − T∞) − G_track (T_t − T_track)   (track term only while loaded)
C_carc  dT_c/dt = G_tc (T̄_t − T_c) − G_cg (T_c − T_g) − h(V) A_c (T_c − T∞)
m_gas c_v dT_g/dt = G_cg (T_c − T_g),     c_v = 718 J/kg K (the cavity is a fixed volume); p_hot from the ideal gas
μ_scale(T) = max(1 − s |T − T_opt|, floor),   s colder or hotter side
```

Uncalibrated until fitted: every output carries `synthesized=True`.

### 23.2 Flash Heat Along a Transient (`thermic_patch.py`)

A post-processor on a solved transient (§6): a three-node ladder (surface, grip core, carcass) takes the frictional flash heat, and the core temperature sets grip through a parabolic window, `μ/μ_opt = clip(1 − ((T − T_opt)/w)², floor, 1)`, with a narrower half-width w on the hot side. Sub-stepped for explicit stability.

### 23.3 Fitting and Field Tests (`tirefit.py`, `tire_field.py`, `measurement_fits.py`)

- **MF5.2 fit:** bounded non-linear least squares of the §3.1 Fy equation against (α, Fz, γ, Fy), seeded from the default set, so the fitted model is exactly the one evaluated.
- **Skidpad friction:** `μ = m a_y / (m g + ½ ρ ClA v²)`, with `a_y = v²/R` and `v = 2πR/t` from a lap time (`correlation`).
- **Camber optimum:** a parabola through lap time at three or more static cambers; its minimum.
- **Tyre rate:** the slope of load against deflection on the car's own tyre; rate at pressure `k(p) = k0 (s + (1 − s) p/p0)` with the carcass share s fitted from two or more pressures.
- **Pneumatic trail:** t0 and α0 of `t_p = t0 cos(πα/2α0)` fitted to Mz and Fy against slip (§16.2).

### 23.4 Degradation over a Run (`degradation.py`)

Three layers on the same kinematic solver: thermal expansion of the chassis pickups, `Δx = α ΔT L` along the vector from a thermal datum, with `T(t) = T∞ + (T_soak − T∞)(1 − e^{−t/τ})` and an optional decay `e^{−d/ℓ}` with distance; structural compliance through §8; and an empirical tyre grip-temperature law, flagged as a placeholder until fitted.

---

## 24. Ride and Damping (`contact_patch.py`, `damper_synthesis.py`)

### 24.1 Quarter Car over an ISO 8608 Road (`contact_patch.py`)

Sprung corner mass m_s, unsprung m_u, wheel rate k, damper c, tyre k_t (and c_t), road input z_r:

```
ΔFz      = k_t (z_r − z_u) + c_t (ż_r − ż_u)
G_d(n)   = G_d(n0) (n/n0)^−2,   n0 = 0.1 cycle/m,   G_d(n0) = 16e-6 (A), 64e-6 (B), 256e-6 (C) m³
G(f)     = G_d(f/V) / V
σ²_ΔFz   = ∫ |H(f)|² G(f) df,       result reported as σ(ΔFz) / Fz_static
f_hop    = √((k + k_t)/m_u) / 2π
P(|v_shaft| < v_knee) = erf( v_knee / (σ_v √2) )
```

Under braking or traction the anti-geometry carries `F_link = κ ΔFz` straight into the body, `κ = tan θ · μ_x`, `tan θ = (anti/100) h / (L · share)`; κ = 0 is the classical quarter car. Body acceleration RMS uses the same frequency response.

### 24.2 Damper Synthesis (`damper_synthesis.py`)

- **High-speed slope:** the damping ratio at the minimum of σ(ΔFz) on the design road (grip optimum).
- **Low-speed slope:** a body-control ratio (0.65 to 0.7) for the body modes.
- **Knee:** a declared multiple of the RMS suspension velocity below 4 Hz on the design road.
- **Front/rear split:** the transient front share of the damper roll moment, `c_f t_f² / (c_f t_f² + c_r t_r²)`, matched to the steady share (53%) by `front_rate_for_share`.

Rebound is a declared multiple of bump; wheel rates go to the shaft through MR² (§9). The result is flagged as synthesised until dynoed.

---

## 25. EV and Combustion Powertrain and Energy (`ev_powertrain.py`, `power_draw.py`, `ev_electrical_check.py`, `powertrain/engine.py`, `simulforge.py`, `ice_powertrain.py`)

### 25.1 Tractive Power, Current and Pack Limits (`power_draw.py`, `ev_electrical_check.py`)

```
F       = C_rr m g + ½ ρ CdA v² + m a
P_wheel = F v,        P_elec = P_wheel / η
P       = (V_oc − I R) I   →   I = (V_oc − √(V_oc² − 4 R P)) / (2R)      (low-current root)
R_pack  = S · R_cell / P_par,     P_max = V_oc² / (4 R_pack)
I_cell  = I_pack / P_par,         joule heat = I² R
gear:   ω_motor = ω_wheel · reduction,   F_t = T / r_pitch,   F_r = F_t tan(pressure angle)
```

The electrical check flags peak pack current against the fuse, peak cell current against its rated C, and lap energy against usable pack energy, and returns the fastest lap the current limit allows. `power_draw` also audits the team workbook (inverted gear ratio, current proportional to voltage, efficiency multiplied rather than divided).

### 25.2 Architecture and Energy over an Event (`ev_powertrain.py`)

Each architecture (single motor and diff, two motors, four-motor vectoring) runs the QSS lap of §5 with its own mass and a declared drive-grip fraction. Energy along the lap:

```
F = m a + F_drag + F_roll                  the force the motor must supply, from the force balance
F > 0 (driving, including constant speed):  E_trac += F · ds / η_inv,motor
F < 0 (resisting):                          E_regen += min(−F, a_regen,max m g) · ds · η_regen
E_lap = E_trac − E_regen,     laps until empty = E_usable / E_lap
```

A shortfall is priced as a planning-grade derate (`0.30 × lap time × energy shortfall fraction`).

### 25.3 Launch and Gear Ratio (`powertrain/engine.py`)

```
rpm      = v/r · G · 60/2π
T        = min( curve(rpm), k_t · i_max )
F_drive  = T G η / r
N_driven = m g w_driven + m a h / L               static share plus load transfer onto the driven (rear) axle
F        = min( F_drive, μ N_driven )
a        = (F − ½ ρ CdA v² − C_rr m g) / (k_rot m)
v ← v + a dt,  x ← x + v dt                         semi-implicit Euler
```

`optimize_gear_ratio` sweeps the final drive for minimum time over a distance.

### 25.4 Actuator on the Bus (`simulforge.py`)

An electrically driven active anti-roll actuator solved with the bus it draws from:

```
M_cmd = clip( −(k_p φ + k_d φ̇), ±M_max )
V_app = clip( R i_ref + K_e ω, ±duty · V_bus )
L di/dt = V_app − R i − K_e ω                      exact exponential update over the step
V_bus = V_oc − I_total R_total                     brown-out latch below a threshold
M = K_t · gear · η · i
```

Degradation (contact resistance, pack resistance, authority loss) enters as electrical parameters and shows up as lost roll authority.

### 25.5 Combustion Powertrain (`ice_powertrain.py`)

The combustion counterpart of §25.1 to §25.3. The torque curve is declared (dyno or engine code); the module does
not model the engine's breathing.

```
restrictor (choked):  ṁ_air = C_d A p₀ √(γ/(R T₀)) (2/(γ+1))^((γ+1)/(2(γ−1)))      (owner: throttle_dynamics, §22)
                      20 mm, 101.325 kPa, 298 K, C_d = 1:  ṁ_air = 0.0745 kg/s
power ceiling:        P_max = ṁ_air / AFR_stoich · LHV · BTE                   (rich fuel beyond stoichiometric does not burn)
gear g:               G_g = primary · ratio_g · final,   rpm = v/r · G_g · 60/2π,   F_g = T(rpm) G_g η / r
upshift g → g+1:      F_g(v) = F_g+1(v), or the redline speed of g if g still pulls harder there
                      (linear T = a − b·rpm:  v* = a / (b c (G_g + G_g+1)),  c = 60/(2π r))
rotating inertia:     m_eff = m + (J_wheels + J_engine G_g² η) / r²             (J_engine only while coupled)
launch:               engine held at launch rpm while the clutch slips; locks when rpm(v, gear 1) = launch rpm;
                      drive cut for t_shift at each upshift; traction and integration as §25.3
QSS lap:              §5 solver on the best-gear envelope, force scaled by m / m_eff so a = F / m_eff;
                      each upshift adds Δt ≈ min(t_s, L a t_s / v²) over the L metres before the next braking point
fuel:                 P_engine = (m_eff a + ½ ρ CdA v² + C_rr m g) v / η  (capped at the envelope),
                      ṁ_fuel = P_engine / (BTE · LHV) on the throttle, a declared overrun flow off it
CO2:                  litres × 2.31 kg/L (gasoline) or 1.65 kg/L (E85);  EV: kWh × 0.65 kg/kWh
coolant heat:         Q = share × ṁ_fuel · LHV  (share declared, about a quarter to a third at load)
engine braking:       F_eb = T_overrun G_g / r at the rear; mechanical brakes supply m a − F_eb split by the bias,
                      so the effective front share = bias (m a − F_eb) / (m a)
```

The feasibility gate checks the restrictor diameter against the rule (20.0 mm gasoline, 19.0 mm E85), the curve against
the restrictor's power ceiling, endurance fuel against the tank with a 10% reserve, fuel use against the 26 L/100 km
limit of the 2024 efficiency proposal, the fuel pump at 1.2 × peak demand, injector duty at 85%, top-gear redline
against the lap's top speed and the cooling loop of §26. An undeclared input is MISSING, never a pass. Brake thermal
efficiency is one number for the whole map, so fuel is a planning estimate until a BSFC map replaces it.

---

## 26. Battery and Cooling Thermal (`pack_thermal.py`, `pcm_cooling.py`, `cooling.py`, `powertrain/engine.py`)

### 26.1 Pack Thermal (`pack_thermal.py`)

Current against time from the QSS lap, on the same force balance as §25.2 (`dt = ds / v̄`, `F = m a + F_drag + F_roll`, `P_batt = F v / η` when F > 0 and regenerated current when F < 0, `I = P_batt / V_pack`, optionally with resistive sag). Each cell on the layout grid:

```
C_cell dT/dt = I_cell² R(T)  +  Σ_neighbours k_cc (T_n − T)  +  h(x,y) A (T_air − T),       I_cell = I_pack / P_par
```

h varies with position from the declared fans, so moving a fan moves the answer. Uncalibrated until fitted.

### 26.2 Phase-Change Buffer (`pcm_cooling.py`)

Effective heat capacity over a melt window around T_melt: `c_p,eff = c_p + L / ΔT_window`; the latent buffer per cell is `m_PCM · L`.

### 26.3 Coolant Loop and Rig (`cooling.py`)

```
Q          = (P_shaft / (η_motor η_inv) − P_shaft) · coolant fraction
ṁ          = ρ V̇,     ΔT = Q / (ṁ c_p)
UA_req     = Q / (T_coolant,in − T∞)              conservative (cold end); also against the mean core temperature
T_steady   = T∞ + Q / UA
t_limit    = m_loop c_p (T_max − T_in) / Q          no radiator
T_boil     ≈ T_boil,1bar + 20 °C per bar of cap pressure
(u_Q/Q)²   = (u_V̇/V̇)² + (u_ΔT/ΔT)² + (u_cp/cp)²,    two independent sensors: u_ΔT = √2 · u_sensor
```

### 26.4 Cooling Network (`powertrain/engine.py`)

```
1/√f  = −1.8 log10[ (ε/D/3.7)^1.11 + 6.9/Re ]             Haaland
ΔP    = (f L/D + Σζ) · ½ ρ v²                             Darcy–Weisbach with minor losses
Y-branch: K_branch = k0 + (1 − A_in/A_b)² (expansion) or 0.5 (1 − A_b/A_in) (contraction), on the inlet head
pump:  ΔP(Q) = Δp0 (1 − (Q/Q_max)²),   operating point where pump and system curves meet
radiator (cross-flow, both unmixed): NTU = UA/C_min, C_r = C_min/C_max,
       ε = 1 − exp[ NTU^0.22 / C_r · (exp(−C_r NTU^0.78) − 1) ],   Q̇ = ε C_min (T_cool,in − T_air)
C_th dT/dt = Q̇_gen(t) − Q̇_reject(T, v(t)),     C_th = ρ V c_p (coolant) + m_metal c_p,al
```

---

## 27. Electrical Systems (`wiring.py`, `fuse_test.py`, `fusebox.py`, `electronics.py`, `pcb_doctor.py`, `harness.py`, `tractive_system.py`, `bq796xx.py`, `rules_fsae.py`)

### 27.1 Wire Ampacity (`wiring.py`)

Base ampacities are the NEC 2017 table. Since the heat shed is proportional to the rise and the heat is I²R:

```
ambient correction = √( (T_rating − T_ambient) / (T_rating − 30 °C) )
bundling: 1.00 (≤3), 0.80 (4–6), 0.70 (7–9), 0.50 (10–20), 0.45 (21–30), 0.40 (31–40), 0.35 (>40)
```

The two multiply; duty is judged on the RMS current.

### 27.2 Fuse and Wire Coordination (`fuse_test.py`)

```
wire withstand:   I² t = k² S²,   k = √( q_c (β + 20)/ρ_20 · ln((β + θ_f)/(β + θ_i)) )    adiabatic; reproduces the IEC 60364-5-54 values
fuse curve:       t = a (I/I_r)^−b,   flattening to constant I²t at high current
crossover:        I^(2−b) = k² S² / (a I_r^b)
```

On the bench: pooled log scatter `s_p² = Σ(n_i − 1) s_i² / Σ(n_i − 1)`, samples needed `n = (z σ_ln / ln(1 + p))²`, relative uncertainty of σ ≈ 1/√(2 dof), timing quantisation resolution/√12.

### 27.3 Which Element Fails First (`fusebox.py`)

Each element on a load path has a capacity with mean μ_i (its FoS at the path's reference load) and σ_i from its evidence grade (§31.1):

```
P(i before j)   = Φ( (μ_j − μ_i) / √(σ_i² + σ_j²) )
P(i fails first) = ∫ φ_i(x) Π_{j≠i} (1 − Φ_j(x)) dx         (trapezoidal quadrature)
```

### 27.4 PCB Copper and Signal Integrity (`electronics.py`, `pcb_doctor.py`)

```
R = ρ_Cu(T) L / A,   ρ_Cu(T) = ρ_20 (1 + α (T − 20))
IPC-2221:   I = k ΔT^0.44 A^0.725   (A in square mils; k = 0.048 external, 0.024 internal)
Onderdonk:  I = A_cmil √( log10( (T_m − T_a)/(234 + T_a) + 1 ) / (33 t) ),   T_m = 1083 °C,   A_cmil = A_sqmil · 4/π
microstrip: Z0 = 87/√(ε_r + 1.41) · ln( 5.98 h / (0.8 w + t) )
diff pair:  Z_diff ≈ 2 Z0 (1 − 0.48 e^(−0.96 s/h))
```

plus a geometric crosstalk budget against switching aggressors. `pcb_doctor` applies the same checks to imported KiCad and Altium boards.

### 27.5 Harness (`harness.py`)

Each conductor is a 3D polyline in car coordinates, checked for clearance against the same keep-outs as the suspension.

- **Bend radius at a vertex:** `R = min(leg_in, leg_out)/2 · tan((180° − turn)/2)`, against the minimum formed bend radius, a multiplier × the outside diameter.
- **Cut length:** path length plus bend allowance, service loops and strip stock.
- **Resistance and mass:** from AWG area.
- **Span sag:** geometric slack `δ ≈ c √(3(s − c)/(8c))`; elastic sag `δ = q L⁴ / (384 EI)` (clamped–clamped), with EI bounded between solid and stranded conductor.
- **First natural frequency:** `f₁ = (22.373 / 2π L²) √(EI/μ)`, with a resonant amplitude `Q n δ` when it overlaps the excitation band.

### 27.6 Tractive-System Safety (`tractive_system.py`, `rules_fsae.py`)

```
precharge:   τ = R C,   V_cap(t) = V (1 − e^(−t/τ)),   t(frac) = −τ ln(1 − frac)
             I_peak = V/R,   P_R,peak = V²/R,   E_R = ½ C V² (independent of R)
```

The shutdown chain (TSAL, BSPD, AMS, IMD) is a series of normally-closed nodes, each naming what trips it. Logged power and voltage are checked the way the energy meter checks them: a violation is the limit exceeded continuously for 100 ms or on a 500 ms moving average (EV.3.4.1, 2027 draft, flagged non-binding).

### 27.7 BMS Daisy-Chain Polling (`bq796xx.py`)

```
t_cycle = (request + response bytes) · t_byte / usable fraction + ADC settle + parse,     max poll rate = 1 / t_cycle
```

CRC-16 (polynomial 0x8005, reflected 0xA001), seeded 0xFFFF, pinned against the published example frames.

---

## 28. Data Acquisition and Sensing (`daq_plan.py`, `gnss.py`, `correlation.py`)

### 28.1 CAN Bus (`daq_plan.py`)

```
frame bits (standard ID) = 47 + 8 · DLC,   + ⌊(34 + 8 · DLC − 1)/4⌋ worst-case stuff bits
bus load = Σ_i bits_i · rate_i / bitrate
response time:  w = B + C_m + Σ_{j higher priority} ⌈(w + τ_bit) · rate_j⌉ C_j,   iterated to a fixed point;
                B = the longest lower-priority frame; unschedulable if w exceeds the period
```

### 28.2 Sampling (`daq_plan.py`)

- **Sample rate:** at least 2× the signal bandwidth (floor), 5× practical, 10× comfortable, with the anti-alias cutoff below fs/2. Aliasing is flagged as unrecoverable.
- **ADC resolution:** span / 2^bits in engineering units.
- **Temperature-difference budget:** `σ_ΔT = √(σ_in² + σ_out²)` for independent sensors, smaller for a matched pair.

### 28.3 IMU Placement and Yaw Limits (`gnss.py`)

```
a_measured = a_cg + α × r + ω × (ω × r)
err_x = −α y − ω² x,    err_y = α x − ω² y            (ω, α about z; r the IMU offset from the CG)
I_zz ≈ m a b                                          (dynamic index one)
α_max = μ g / L                                       from M_z = μ m g (b/L) a
```

### 28.4 Correlation with Track Data (`correlation.py`)

```
skidpad:   v = √(a_y R),   t = 2πR / v
trace:     RMSE, mean bias (sim fast or slow), peak-speed error, R² of predicted against measured speed
```

Nothing is fitted; the gap and its sign are reported.

---

## 29. Structures and Mechanisms Beyond the Corner (`flex.py`, `flexgen.py`, `generic_compliance.py`, `joints.py`, `permanent_set.py`, `printed_parts.py`, `morphmesh.py`, `ghost_topology.py`, `phantom_envelope.py`, `topology.py`, `chassis.py`, `mountpoints.py`, `interfaces.py`)

### 29.1 Flexible Bodies (`flex.py`)

`k = E A / L` for a tube link, or a finite-element part condensed to its interface nodes (Guyan):

```
K_red = K_mm − K_ms K_ss⁻¹ K_sm
```

which is exact for the quasi-static case; modal content is not fabricated.

### 29.2 Flexure Joints (`flexgen.py`)

A pseudo-rigid-body discretised elastica: n rigid segments of length ℓ = L/n joined by torsional springs `k_i = E I(s_i)/ℓ` at midpoint stations. Equilibrium by Newton on the exact gradient of

```
Π(φ) = Σ ½ k_i (φ_i − φ_{i−1})² − F · u_tip(φ) − M φ_tip
C    = J H⁻¹ Jᵀ   (tip compliance, J = ∂u_tip/∂φ)
P_cr from H(P) = K − P · diag(ℓ) becoming singular → π² E I / (K L)²
```

n = 16 is within 1% of the analytic small-deflection limits. The strain energy per mm of bump gives the wheel rate the flexure adds, `d²U/dz²`.

### 29.3 Compliance of Any Architecture (`generic_compliance.py`)

For each link, nudge its held length and re-solve the whole mechanism to get `b_i = ∂(contact patch)/∂L_i` and `r_i = ∂(carrier rotation)/∂L_i`. By virtual work the member force is `λ_i = F · b_i`, its stretch `λ_i / k_i`, and the wheel moves by `Σ b_i ΔL_i`; the held lengths are updated and the mechanism re-solved.

### 29.4 Non-Linear Joints (`joints.py`)

```
series:      δ_total(T) = δ_link(T) + δ_joint,in(T) + δ_joint,out(T)
laws:        F = k δ;   F = k₁ δ + k₃ δ³;   bilinear with a knee;   free-play dead band;   tabulated
loss/cycle:  E = π c ω X² (viscous),   E = π η k_sec X² (structural),   c_eq = c + η k / ω
thread fatigue (VDI 2230): σ_ASV = 0.85 (150/d + 45),   rolled after heat treatment σ_ASG = (2 − F_Sm/F_0.2) σ_ASV
```

Stiction enters as a describing function (secant stiffness and a phase lag, §16.6).

### 29.5 Permanent Set (`permanent_set.py`)

A fiber model of the section: plane sections `ε = ε₀ + κ y`, each fiber `σ = σ_res + E (ε − ε_p)` clipped to ±σ_y (elastic–perfectly plastic). Loading finds the curvature that carries the moment with zero axial force at each station and records plastic strain; unloading is elastic (a 2×2 solve). Permanent curvature integrated along the cantilever gives the residual tip deflection. Weld residual stress is declared and self-equilibrated; strain hardening is neglected.

### 29.6 Printed Parts (`printed_parts.py`)

```
allowable = σ_datasheet × k_orientation × k_temperature × k_creep × k_moisture
k_orientation = z-factor across layers (0.40–0.60), 1 in-plane
k_temperature = 1 below (HDT − 60 °C), falling linearly to 0.2 at HDT, 0 above
k_creep       = 0.75 sustained, 0.55 sustained within 90 °C of HDT
k_moisture    = 0.95 dried and sealed, 0.80 otherwise (hygroscopic materials)
hoop stress   = p d / (2t),  axial half
```

### 29.7 Generative Brackets (`morphmesh.py`)

Density-based SIMP on a 2D plate: `E_e = E_min + x^3 (E_0 − E_min)`, density filter of radius r_min, Heaviside projection `x̄ = (tanh βη + tanh β(x − η)) / (tanh βη + tanh β(1 − η))` with β continuation, and an optimality-criteria update under a volume budget. The load set is the fan of member-force directions from a transient audit, grouped by exposure; the minimum rib and the heat-affected zone come from the declared shop tolerance.

### 29.8 Deformed Geometry under a Transient (`ghost_topology.py`, `phantom_envelope.py`)

At each instant of a transient the compliant corner (§8) is solved under that instant's loads. The tyre force responds to the deflection, `dFy = (∂Fy/∂γ) Δγ + (∂Fy/∂toe) Δtoe`, with a loop gain `g = (dFy₂ − dFy₁)/dFy₁` and converged force `Fy + dFy₁/(1 − g)`. Members are screened by pinned Euler buckling and yield. `phantom_envelope` sweeps each member as a capsule (tube radius) over travel, steer and the compliant states, and reports where the loaded envelope leaves the rigid one.

### 29.9 Multibody Kernel and Packaging (`topology.py`, `chassis.py`, `mountpoints.py`, `interfaces.py`)

- **Multibody kernel:** any suspension (MacPherson, multilink, trailing arm, solid axle, twist beam, truck steering) is a list of points, bodies and constraints (distance, revolute, planar, in-line) driven by wheel travel and rack, solved by Levenberg-Marquardt like §1.1.
- **Clearance:** the minimum distance from the swept links to the chassis mesh over travel and steer.
- **Mount points:** the signed distance from each point to the keep-out boxes.
- **Mass roll-up:** `CG = Σ m_i r_i / Σ m_i`.

---

## 30. Build Tolerance (`kinematik_stochastic.py`)

Per-point, per-axis tolerance boxes, which may be asymmetric (a weld pulls toward the bead), sampled uniformly (the default) or as a normal truncated at the box with σ = span/4. Metrics are propagated to first order through the Jacobian J (or by full solves), giving yield as in §15.5. With an asymmetric field the expected as-built error is `E[Δm] = J E[δ] ≠ 0`, and the **robust nudge** shifts the nominal by the band-weighted least-squares solution of `J x ≈ −J μ`, re-centring the build cloud in its bands. A symmetric field admits no such gain, and the module says so. Adjusters (shims, threads) are quantised.

---

## 31. Uncertainty, Tests and Decisions (`proof_engine.py`, `phantom_car.py`, `earshot.py`, `saboteur.py`, `trade.py`, `measurement_fits.py`, `setup.py`, `arch_synth.py`)

### 31.1 Evidence Grades (`proof_engine.py`)

Every declared number carries a relative uncertainty from its evidence grade, inflated linearly with age and capped at the guess level:

| Grade | Base relative uncertainty | Doubles after |
|-------|---------------------------|---------------|
| guess | 40% | never |
| estimate | 20% | 365 days |
| modelled | 10% | 270 days |
| measured | 3% | 180 days |
| verified | 1% | 180 days |

```
u(age) = min( u_base (1 + age / half-life), 0.40 )
```

These are propagated through the objective models. A simulation result outside the prediction ± 3u is reported as discrepant: either the run or the ledger is wrong.

### 31.2 Stacked Margins and Mutation Testing (`phantom_car.py`, `saboteur.py`)

Each hedge on a shared number is expressed as `z_i = (f_i − 1) |value| / σ` and combined as a first-order reliability index `β = √(Σ z_i²)`: the statistical rarity of the car the margins actually defend. `saboteur` injects known input errors (unit slips, imperial into SI, frame flips, dropped or double-counted roll-up terms) and reports those that move the objective by less than its own 3σ band.

### 31.3 Will the Test Answer the Question? (`earshot.py`)

```
A/B laps:      n per configuration = 2 ((z_{1−α/2} + z_power) σ / δ)²,    MDE(n) = (z_{1−α/2} + z_power) σ √(2/n)
               miss probability = Φ( δ / (σ √(2/n)) − z_{1−α/2} )
ordering bias: drift × difference in mean lap index (AABB biased; ABBA cancels linear drift)
tilt CG:       h = ΔW L / (W tan θ),  σ from the angle term σ_θ/(sin θ cos θ) and the scale term σ_ΔW/ΔW
coast-down:    F = A + B v²,   B = m (a_h − a_l) / (v_h² − v_l²),   CdA = 2B/ρ
corner scales: σ_total = √4 × pad resolution
```

### 31.4 Trade Studies, Sensitivity and Architecture (`trade.py`, `setup.py`, `arch_synth.py`, `inverse_genesis_fullcar.py`)

- **Trade studies:** sweep the parameters a component comparison is sensitive to (μ scale, torque bias ratio, preload, locked-axle penalty, exit lateral g), and declare no winner unless the sign of the points delta holds across the band.
- **Setup sensitivity:** finite differences of limit grip and balance per knob; optimisation is a transparent grid or coordinate search that holds balance in a window.
- **Architecture synthesis:** NSGA-II over discrete and continuous variables together, with kinematic objectives from the real solver and mass and points as labelled parametric estimates.
- **Full-car synthesis:** a staged deterministic search over pack configuration, drive architecture and final drive against a rule matrix and the event-points model.

---

## 32. Verification of the Physics

### 32.1 Physics Proofs (`tests/test_load_case_physics.py`, `tests/test_accuracy_invariants.py`)

Each check uses a principle the code under test does not:

| Check | Principle |
|-------|-----------|
| Pushrod force from `loadpath` | Virtual work: `T · dL/dz` equals the load power on the wheel (patch force on the wheel's material point, shaft force at the wheel centre, aligning torque on its rotation) |
| Forces on the chassis | Global equilibrium of the corner, forces and moments |
| Cornering load case | Vehicle equilibrium (tyre lateral forces toward the turn centre) and `Mz` following the tyre |
| Link-borne vertical force | Virtual work, `V = −q Fy`, against `loadpath` member forces (§1.11) |
| Scrub rate | Patch motion against the front-view instant-centre construction |
| Beam element | Closed forms `PL³/3EI`, `PL/EA`, `TL/GJ` |
| Frame model | Six rigid-body modes and no mechanism; flexibility symmetric (Maxwell-Betti) and positive definite |
| Free-frame solve | Loads balanced by inertia: no reaction at the pinning nodes |
| Load transfer | Moment balance `ΔFz_f t_f + ΔFz_r t_r = m a_y h`, with split roll centres |
| Anti-dive, anti-squat | Re-derived from virtual work without production code |
| Instant centre | Perpendicular to the ball joints' motion; matches the hand construction |

### 32.2 Benchmarks

- **Lotus Suspension Analysis** (`scripts/lotus_benchmark.py`, `tests/test_lotus_benchmark.py`): camber and toe in bump, roll and steer, and roll centre, against a published corner the code was not tuned to. It checks the forward kinematics, not the synthesis.
- **Independent kinematics** (`scripts/independent_kinematics.py`): the upright as one rigid body (six unknowns, SciPy `fsolve`), camber and toe read off the spin axis, the roll centre off the patch's path; agrees with KinematiK's camber and toe within 1e-12° on `front_v11_final`, `rear_v11_final` and `front_v11_joints50`.
- **Known-answer recovery** (`test_inverse_solve_recovers_hidden_geometry`): hidden geometries recovered from their curves alone.

### 32.3 Corrections the Checks Forced

| Error | Correction |
|-------|-----------|
| Lateral load cases put the outer wheel's ground force outboard (`Fy > 0`, `Mz > 0`) | It points to the turn centre: `Fy < 0`, `Mz = −t_p · abs(Fy) < 0`; shock cases kick inboard as well as outboard; rear braking unloads the rear |
| `loadpath` resolved the load as if the pushrod acted on the upright when it acted on the wishbone (pushrod force 35% low against virtual work) | Two-body statics (§17.2) and leg bending in `member_margins` |
| Scrub radius measured on z = 0 at every travel | Measured on the road plane through the contact patch (§1.5) |
| Midpoint instant-centre construction | Exact velocity construction (§1.6, §1.7) |
| `abs()` on the anti-dive offset | Signed slope (§1.8) |
| Anti-squat from the contact patch with inboard drive | Wheel-centre path (§1.9) |
| Degenerate roll centre returned as contact-patch height | NaN (§1.6) |
| `LGAY` on SVy only; curvature sign on raw α | `LGAY` on every camber term; sign of shifted slip (§3.1) |
| Damping ratio on wheel rate alone | Ride rate with the tyre in series (§9) |

### 32.4 External Validation

**Independent re-runs (SIT Racing, Elbee Racing).** SIT Formula Racing team (Singapore Institute of Technology), and Jonathan Iglesias of the Elbee Racing suspension team each re-ran `front_v11_final` from the published code (manifest `2e7072ce2268`, its record before the load-case correction; the corrected record `2d35663ecd27` returns the same pickups). `scripts/verify_run.py` reported each re-run identical to the recorded outputs: every coordinate to 1e-6 mm, every fit and yield to 1e-9. This shows the synthesis reproduces outside the author's environment. It is a reproducibility check, not a physical test of the car.

**Applied Racing Dynamics (ARD).** `front_v11_joints50` (front) and `rear_v11_final` (rear) were entered in ARD's suspension model and compared at static and at ±20 mm of travel. Differences are against ARD's display precision.

| Quantity | Front: KinematiK / ARD | Rear: KinematiK / ARD |
|----------|------------------------|-----------------------|
| Caster, static (°) | 3.656 / 3.66 | 2.862 / 2.86 |
| KPI, static (°) | 7.313 / 7.31 | 8.601 / 8.60 |
| Scrub, static (mm) | 6.0 / 6.00 | 13.625 / 13.62 |
| Roll-centre height, static (mm) | 56.09 / 56.35 | 41.85 / 41.90 |
| Motion ratio (spring) | 0.80 / 0.80 | 0.821 / 0.82 |
| Camber change, 20 mm droop (°) | +0.672 / +0.67 | +0.551 / +0.56 |
| Camber change, 20 mm bump (°) | −0.804 / −0.79 | −0.598 / −0.59 |
| Caster at ±20 mm (°) | within 0.01 | within 0.003 (tyre on the ground) |
| KPI at ±20 mm (°) | within 0.012 | within 0.003 |
| Scrub at ±20 mm, on the road (mm) | 6.03, 5.96 / 6.02, 6.00 | 13.66, 13.53 / 13.68, 13.58 |
| Anti-squat (%) | — | 50.2 / 50.29 |

Every difference that exceeded its limit was traced to a definition, each checked with an independent solve:

- **Toe change and steer angle** (0.04 to 0.15°). ARD's kinematics page evaluates the spin axis at 0° static camber. With 0° the independent solve reproduces ARD (front −0.007 and +0.073°, rear −0.133 and +0.150°); with the static camber the corner carries (−2.5°, −2.6°) it reproduces KinematiK.
- **Roll-centre height over travel.** ARD measures from the road, KinematiK from the static ground line in the body frame. The constant 0.26 mm offset at static carries through ±20 mm.
- **Rear caster and KPI over travel.** ARD heaves the body with the tyre on the ground, so the wheel centre moves 20.34 and 19.61 mm for a 20 mm heave. At that state the independent solve gives ARD's values.
- **Scrub over travel.** The comparison found that KinematiK took scrub on z = 0 at every travel. This was corrected to the road plane (§1.5, §32.3), after which it agrees.

Not compared: camber in steer (ARD's definition is unconfirmed), and anti-dive and anti-lift (ARD's brake page was not set to this car's bias and inboard rear brakes).

**ARD lap time.** A load-transfer-balance study in ARD's lap simulator (front share 49, 53 and 57%) is pending. 

---

**Lotus Suspension Analysis Benchmark**

The source is C. Arevalo, A. Medina and J. Valladolid, *Kinematic and dynamic study of the suspension system of an electric single seater competition Formula Student*, Ingenius 20 (2018) 96–107 (open access). It lists the hardpoints of two double-wishbone corners and plots their kinematics from Lotus Suspension Analysis. KinematiK solves the same hardpoints and is compared against the plotted values (read from the figures, so the Lotus figures carry reading error).

**Set-up.** Source frame to KinematiK: `x_K = −(x − x_axle)` with the axles at x ≈ 160 and x ≈ 1760 mm, y unchanged, `z_K = z − (310 − 247.65)` mm, which puts the ground under the 310 mm wheel centre of a 19.5 in tyre. As in Lotus, bump is wheel-centre travel. The corners are named here by their source x position, because the source's front/rear labels are inconsistent between its tables.

| Quantity | Lotus | KinematiK | Largest difference |
|----------|-------|-----------|--------------------|
| Camber, bump ±30 mm | figure 5 | table below | 0.009° |
| Toe, bump ±30 mm | figure 8 | table below | 0.0003° |
| Camber, body roll ±3° | figure 9 | table below | 0.008° |
| Camber, steer ±31.5° (tyre on the ground) | figure 11 | table below | 0.050° |
| Roll-centre height, bump ±30 mm | figure 7 | table below | 2.1 mm (x ≈ 160), 3.3 mm (x ≈ 1760) |
| Caster magnitude, static (x ≈ 160 / x ≈ 1760) | 7.0° / 9.36° | 7.00° / 9.36° | < 0.01° |
| KPI, static (x ≈ 160 / x ≈ 1760) | 8.13° / 0° | 8.13° / 0.00° | exact |
| Track (x ≈ 160 / x ≈ 1760) | 1200 / 1180 mm | 1200 / 1180 mm | exact |

**Bump** (degrees; toe-in positive):

| Wheel travel (mm) | Camber x≈160, Lotus / KinematiK | Camber x≈1760, Lotus / KinematiK | Toe x≈160, Lotus / KinematiK | Toe x≈1760, Lotus / KinematiK |
|---|---|---|---|---|
| −30 | +1.414 / +1.416 | +0.964 / +0.964 | +0.0062 / +0.0062 | +0.1388 / +0.1391 |
| −20 | +0.964 / +0.967 | +0.660 / +0.660 | +0.0007 / +0.0007 | +0.0734 / +0.0731 |
| −10 | +0.492 / +0.495 | +0.340 / +0.339 | −0.0015 / −0.0015 | +0.0270 / +0.0269 |
| +10 | −0.524 / −0.520 | −0.356 / −0.358 | +0.0054 / +0.0054 | −0.0078 / −0.0079 |
| +20 | −1.065 / −1.066 | −0.731 / −0.735 | +0.0150 / +0.0151 | +0.0033 / +0.0031 |
| +30 | −1.631 / −1.640 | −1.129 / −1.133 | +0.0291 / +0.0293 | +0.0327 / +0.0329 |

**Camber to ground against body roll** (degrees):

| Roll (°) | x≈160, Lotus / KinematiK | x≈1760, Lotus / KinematiK |
|---|---|---|
| −3 | +1.276 / +1.274 | +1.827 / +1.829 |
| −2 | +0.879 / +0.881 | +1.240 / +1.242 |
| −1 | +0.449 / +0.455 | +0.630 / +0.631 |
| +1 | −0.476 / −0.482 | −0.646 / −0.651 |
| +2 | −0.988 / −0.990 | −1.315 / −1.321 |
| +3 | −1.512 / −1.520 | −2.004 / −2.008 |

**Camber against steer**, x ≈ 160 corner (degrees). KinematiK is solved with the tyre held on the ground, as Lotus does. `camber_in_steer` as shipped holds the ball joint at ride height instead, which reads 0.06 to 0.19° larger in magnitude.

| Steer (°) | Lotus | KinematiK, tyre on the ground | Difference | KinematiK, ball joint held |
|---|---|---|---|---|
| −31.5 | +5.507 | +5.553 | +0.046 | +5.740 |
| −30 | +5.146 | +5.162 | +0.016 | +5.342 |
| −20 | +2.920 | +2.970 | +0.050 | +3.093 |
| −10 | +1.302 | +1.306 | +0.004 | +1.368 |
| +10 | −1.051 | −1.050 | +0.001 | −1.112 |
| +20 | −1.925 | −1.906 | +0.019 | −2.029 |
| +30 | −2.647 | −2.605 | +0.042 | −2.784 |
| +31.5 | −2.744 | −2.697 | +0.047 | −2.884 |

**Roll-centre height above ground in bump** (mm), tyre radius 230 mm (the radius that reproduces the source's static roll centre):

| Wheel travel (mm) | x≈160, Lotus / KinematiK | x≈1760, Lotus / KinematiK |
|---|---|---|
| −30 | 90.9 / 90.0 | 110.2 / 113.5 |
| −20 | 75.8 / 74.7 | 95.3 / 97.5 |
| −10 | 60.8 / 59.6 | 80.5 / 81.8 |
| 0 | 46.0 / 44.6 | 66.1 / 66.2 |
| +10 | 31.4 / 29.9 | 51.8 / 50.9 |
| +20 | 17.0 / 15.3 | 37.8 / 35.7 |
| +30 | 3.0 / 0.9 | 23.8 / 20.7 |

Camber and toe agree to within the precision of reading the source's plots. The roll-centre differences are larger: they depend on the tire radius the source used (unstated) and on reading a steep curve off a figure. The source publishes no anti-geometry. Every anti value in KinematiK is the same function of the solved side-view path slope (§1.8), which this benchmark checks through camber and toe.

---

## 33. Test Coverage

The formulas in this document are pinned by the test suite (`fsae_suspension/tests/`); the exception is the underfloor channel (§21.1), which has no dedicated test yet. Key tests and files:

| Test | What it pins |
|------|-------------|
| `test_flat_pickups_give_zero_anti_dive` | Parallel arms → SVIC at infinity → zero anti-dive |
| `test_anti_squat_shortcut_error_is_bounded` | Wheel-centre shortcut within 1 pp of exact |
| `test_scrub_radius_is_measured_on_the_road` | Scrub on the road plane at every travel |
| `test_accuracy_invariants.py` | Load-transfer moment balance, instant centre, anti signs, virtual-work anti, native vs generic solver |
| `test_kinematics.py`, `test_motion_ratio_antidive.py` | Camber, toe, caster, KPI, scrub, motion ratio |
| `test_tiremodel.py` | MF5.2 force law, load sensitivity, mu_peak, fitter |
| `test_compliance.py`, `test_elastokinematics.py` | Compliance steer and camber, pickup stiffness, elastokinematic solve |
| `test_target_derivation.py` | Ride-frequency floor, camber chain, floors, shares, sensitivity |
| `test_inverse_genesis*.py`, `test_genesis_walls.py`, `test_genesis_repro.py` | Residual, walls as refusals, wheel envelope, property bounds, yield, manifests |
| `test_load_case_physics.py` | The physics proofs of §32.1 |
| `test_lotus_benchmark.py`, `test_final_pair.py` | External benchmark; every wall and quoted value on the final corners |
| `test_robustness.py`, `test_actuation.py`, `test_steering_margin.py` | Margins, shock cases, actuation, steering window |
| `test_tire_utilisation.py`, `test_torque_distribution.py` | Utilisation, envelope, torque split, yaw budget |
| `test_transducer.py` | Strains, bridge output, excitation limit, dynamic error, calibration certificate |
| `test_damper*.py`, `test_bolted_joint.py`, `test_bracket_fos.py` | Damper law, VDI 2230, bracket FoS |
| `test_scale_model.py`, `test_pressure_tap.py`, `test_piv.py`, `test_windtunnel.py`, `test_daq.py` | Similitude, Cp reduction and stall, PIV, tunnel correlation, balance decoupling |
| `test_pedal_box.py`, `test_throttle_return.py`, `test_throttle_dynamics.py` | Balance bar, pedal travel, return-spring redundancy, plenum and flutter |
| `test_tire_thermal.py`, `test_thermic_patch.py`, `test_tire_field.py`, `test_degradation.py`, `test_measurement_fits.py` | Tyre thermal network, flash heat, field tests, degradation, fits |
| `test_ev_powertrain.py`, `test_power_draw.py`, `test_pack_thermal.py`, `test_simulforge.py` | Architecture and energy, power draw, pack thermal, actuator on the bus |
| `test_ice_powertrain.py` | Choked restrictor flow and power ceiling, shift points, clutch launch, fuel by force balance, endurance fuel, engine braking, the ICE gate |
| `test_electronics.py`, `test_harness.py`, `test_fuse_test.py`, `test_fusebox.py`, `test_bq796xx.py` | PCB copper and impedance, harness, fuse coordination, failure order, BMS polling |
| `test_daq_plan.py`, `test_gnss.py`, `test_correlation.py` | CAN frames and bus load, sampling, IMU lever arm, track correlation |
| `test_flexgen.py`, `test_generic_compliance.py`, `test_morphmesh.py`, `test_ghost_topology.py`, `test_phantom_envelope.py`, `test_topology.py` | Flexures, generic compliance, SIMP, deformed geometry, envelopes, multibody kernel |
| `test_kinematik_stochastic.py`, `test_proof_engine.py`, `test_phantom_car.py`, `test_earshot.py`, `test_arch_synth.py` | Tolerance and robust nudge, evidence grades, stacked margins, test power, architecture search |

About 3,500 tests; `pytest --collect-only -q` gives the current count.

---

## References

- Gillespie, T.D. — *Fundamentals of Vehicle Dynamics*, SAE International
- Milliken, W.F. & Milliken, D.L. — *Race Car Vehicle Dynamics*, SAE International
- Dixon, J.C. — *Tires, Suspension and Handling*, SAE International
- Pacejka, H.B. — *Tyre and Vehicle Dynamics*, Elsevier
- Archard, J.F. — *Contact and Rubbing of Flat Surfaces*, Journal of Applied Physics, 1953
- VDI 2230 Part 1 — Systematic Calculation of Highly Stressed Bolted Joints
- Shigley, J.E. — *Mechanical Engineering Design*, McGraw-Hill
- AWS D1.1 — Structural Welding Code
- BS 8422:2003 — Force measurement: strain gauge load cells
- JCGM 100:2008 — Evaluation of measurement data: Guide to the expression of uncertainty in measurement (GUM)
- ISO 8608 — Mechanical vibration: road surface profiles
- Haaland, S.E. — *Simple and Explicit Formulas for the Friction Factor in Turbulent Pipe Flow*, J. Fluids Eng., 1983
- Incropera, F.P. & DeWitt, D.P. — *Fundamentals of Heat and Mass Transfer*, Wiley (ε-NTU, convection correlations)
- Heywood, J.B. — *Internal Combustion Engine Fundamentals*, McGraw-Hill (compressible orifice flow)
- IPC-2221 / IPC-2152 — Printed board design; current-carrying capacity
- IEC 60364-5-54 and NEC 2017 — Conductor short-circuit withstand; ampacity tables
- Davis, R.I. et al. — *Controller Area Network (CAN) Schedulability Analysis*, Real-Time Systems, 2007
- Howell, L.L. — *Compliant Mechanisms*, Wiley (pseudo-rigid-body model)
- Andreassen, E., Clausen, A., Schevenels, M., Lazarov, B.S. & Sigmund, O. — *Efficient topology optimization in MATLAB using 88 lines of code*, Struct. Multidisc. Optim., 2011
- Deb, K. et al. — *A fast and elitist multiobjective genetic algorithm: NSGA-II*, IEEE Trans. Evol. Comput., 2002
