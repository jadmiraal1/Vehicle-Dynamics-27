# Model status

What each part of the toolchain models, how far it has been checked, and what
is still open. Fidelity tiers: **T0** point mass with one friction coefficient;
**T1** axle-level quasi-steady-state (per-tire vertical loads, load-sensitive
tire, two axles); T2 (four-wheel with suspension kinematics) and T3 (transient)
are not built.

Status key: **Working** - runs, checked by `vd_selftest`/`vd_golden`;
**Provisional** - runs, but rests on unmeasured inputs or an open question;
**Needs data** - cannot be trusted until a measurement exists.

Numbers quoted below are for TR27 as configured on the date of this file and
move whenever the config does; re-run the script for current values.

## Summary

| Feature | Files | Model | Status |
|---|---|---|---|
| Car definition | `cars/`, `vehicle_params.m`, `util/vd_derive.m`, `util/vd_set.m` | inputs + derived values, "what if?" via `vd_set` | Working |
| Tire model | `tire/`, `models/mu_of_load.m`, `tire_forces.m`, `tire_camber.m` | per-load Magic Formula (lateral), camber factors, friction ellipse | Provisional |
| Load transfer | `models/load_transfer.m`, `road_loads.m` | rigid body, LLTD as an input | Working (simplified) |
| Lateral limit | `models/axle_grip.m`, `ay_limit.m` | T1 | Working |
| Longitudinal limits | `models/ax_limit.m`, `ax_combined.m`, `gg_envelope.m` | T1 (RWD only) / T0 | Working |
| Lap simulation | `lapsim/lap_sim.m`, `corner_speed.m` | point-mass QSS path solver on T1 limits | Working |
| 75 m acceleration | `lapsim/accel_time.m` | time integration on T1 traction + motor cap | Working |
| Handling | `models/bicycle_model.m`, `understeer_at.m` | linear 2-DOF bicycle | Working (sub-limit only) |
| Energy | `lapsim/lap_duty_cycle.m`, `run_energy_strategy.m` | QSS energy with motor loss map | Provisional |
| Cooling | `targets/run_cooling_targets.m` | duty-cycle heat + eps-NTU sizing | Needs data |
| Points model | `util/fsae_points.m`, `util/fsae_rules.m` | FSAE 2026 D.9-D.13 scoring | Working |
| Steering / aligning moment | `targets/aligning_moment.m` | binned TTC Mz data | Provisional |
| Tracks | `tracks/` | digitized centrelines | Provisional |
| Suspension kinematics, transient, tire thermal | - | not built | - |

## Lap simulation - what kind of model is it?

**Neither a pure bicycle model nor a four-wheel vehicle model: a point-mass
path solver whose acceleration limits come from a four-tire, two-axle load
model.**

- **Trajectory (point mass).** The car is a point following the track
  centreline at the curvature in the CSV. Speed is the only state; there is no
  yaw, no steering angle, no slip angle and no racing-line optimisation. The
  solver sets a speed ceiling at every node from the cornering limit, then runs
  a forward (accelerate) and a backward (brake) pass and keeps the lower speed.
  A closed track repeats the passes until start and end speeds agree (two
  passes suffice); an open track starts from rest.
- **Limits (four tire loads, two axles).** At each speed the limits are solved,
  not assumed:
  - *lateral* (`axle_grip`): static axle loads from weight and downforce (centre
    of pressure), lateral load transfer split between axles by LLTD, giving four
    tire loads; each tire's grip from the load-sensitive tire model (with camber
    if set); the front and rear axles must each carry their share of the
    lateral force from the yaw-moment balance (front `b/L`, rear `a/L`). The
    lateral limit is where the first axle runs out - a bicycle-style force
    balance on four-wheel loads.
  - *longitudinal* (`ax_combined`): same loads plus longitudinal transfer
    `m*ax*h/L`; drive on the rear axle only; ideal brake bias; on each axle the
    longitudinal force left after its lateral share follows the measured
    friction ellipse (exponent ~1.8, a lower-bound estimate).
  - *motor*: torque cap below base speed, constant pack power above (through a
    lumped driveline efficiency), rev limit as a hard speed ceiling.
- **Not modelled:** yaw dynamics and transient balance, the racing line,
  driver, tire temperature and wear, track grip changes, the 0.3 m / 6 m
  staging roll-in before the timing line (accel / autocross), aero balance
  change with ride height or pitch.

The default tiers are `grip_model = 'axle'` and `long_model = 'combined'`.
Setting `grip_model = 'pointmass'` gives the constant-mu T0 limit for comparison.

## Feature notes

### Car definition
- TR27 is fully specified in `cars/config_TR27.m`; values are design targets or
  TR26 carry-over until measured. 29 parameters are flagged PROVISIONAL,
  [verify] or MEASURE (see `params_report`).
- **TR27 aero is set to the target package** (ClA 4.0, CdA = ClA/2.5) rather
  than the measured TR26 aero (ClA 1.30). The aero and aero-vs-gear studies
  compare against the same target and so show almost no difference until the
  config holds a different, measured value.
- `config_TR26.m` is largely TR27 carry-over marked `[verify]`; a TR26
  calibration is not meaningful until those are replaced with TR26 records.

### Tire model
- Lateral: 4-parameter Magic Formula per 50-250 lbf load bin (median curve,
  rolling samples only, 9-13 psi, |IA| < 1.5 deg), peak mu linear in load,
  cornering stiffness quadratic in load. Camber: peak, stiffness and thrust
  factors from the 0/2/4 deg sweeps (no negative-inclination data; symmetric
  tire assumed). Longitudinal: only the 18 in LC0 has drive/brake data, at one
  load; its mu_x/mu_y ratio is transferred to the design tire.
- **Belt-to-track scaling `mu_derate = 0.67` is uncalibrated** - every grip
  number scales with it. Calibrate against skidpad and braking data.
- **The cornering limit runs on extrapolated tire data.** The design tire's data
  ends at ~246 lbf and the donor tires at ~249 lbf, so above the edge mu
  continues the fitted line ('central' and 'low' coincide). At the cornering
  limit the outer front tire carries ~330 lbf at skidpad speed and ~440 lbf at
  28 m/s (downforce): a third to 80% beyond any data. Camber factors are
  clamped at 250 lbf.
- Pressure, temperature, relaxation length, combined-slip Magic Formula and an
  aligning-moment model are not built.
- The committed `tire_coeffs_TR27.mat` must be rebuilt (`build_tire_coeffs`,
  needs TTC_Data and the Optimization Toolbox) whenever `tire/` changes.

### Load transfer and axle grip
- Rigid-body quasi-static transfer. LLTD is a single input splitting the
  lateral roll moment between axles; no roll centres, no geometric/elastic
  split, no unsprung mass term, no suspension compliance.
- Drag acting above the ground (pitch moment) and wheel-inertia reaction
  torques are not included in the longitudinal transfer.

### Longitudinal limits
- The axle tiers model rear-wheel drive only (they stop with an error for any
  other `p.drive`).
- Braking assumes ideal brake bias - an upper bound a fixed-bias car matches at
  one deceleration only. `run_load_transfer_targets` reports the ideal bias.

### Handling
- Linear bicycle model at static axle loads, constant cornering stiffness, no
  downforce or lateral load transfer: valid sub-limit (~0.4 g) for sign and
  trend. TR27 currently reads slightly oversteering (negative K).
- Yaw inertia uses a provisional dynamic index (DI = 0.75).

### Energy and power
- Wheel work from the QSS speed trace; motor losses from a fit to the EMRAX 228
  efficiency map (the published map is for the HV winding; the datasheet states
  the windings differ only in voltage and current); flat inverter and chain
  efficiencies (PROVISIONAL); no regen (TR27 decision).
- The thrust limit uses the lumped `eta_dt` while the energy accounting uses
  chain x inverter x motor map; they differ by a few percent.
- The endurance power cap that fits the usable pack (with margin) comes from
  `run_energy_strategy`; the usable window and margin are choices.

### Cooling
- Heat from the same duty cycle; radiator and fan requirement by eps-NTU.
- Radiator size, UA, coolant flow, fan operating point and ram-air capture are
  unmeasured, so the study issues a requirement, not a verdict.

### Points model and studies
- Scoring follows FSAE Rules 2026 D.9-D.13 against the real 2026 results in
  `organization/comp_benchmarks_2026.csv`, with an 8% time haircut on the QSS
  times (calibrated to 2026 results). The best efficiency factor in the field
  (`ef_max` = 0.60) is an estimate.
- Studies that issue targets: load transfer, g-g, lap, handling, stability,
  balance (LLTD and CoP bands), weight distribution, aero, energy strategy,
  final drive (+ aero sensitivity), pack size, camber, cooling, steering.
  Each prints an Assumes line; read it before quoting.
- The weight-distribution floor (44% front) is a placeholder for a transient
  stability result.

### Tracks
- Endurance (closed, 964 m) and autocross (open, 674 m) were digitized from
  the 2026 course maps. **Slalom cone spacings are unverified**; slalom
  curvature scales with 1/spacing^2.

## Not built (planned in order of value)

1. Suspension kinematics (roll centres, camber gain) feeding the existing
   camber model and a derived LLTD.
2. Calibration of `mu_derate`, `lambda_Ca` and the haircut against measured
   skidpad, accel and lap data.
3. Transient (yaw/roll) model for limit balance, damping and rearward weight
   limits.
4. Tire pressure and temperature effects; aligning-moment model.
