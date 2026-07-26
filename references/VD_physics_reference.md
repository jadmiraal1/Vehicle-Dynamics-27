# Vehicle Dynamics — Concept-Tier Physics Reference

A from-first-principles walkthrough of every model in the toolchain, in build order.
Each section gives the governing equations, the derivation, what the script computes,
and the assumptions baked in. SI units throughout (kg, m, N, s, rad); accelerations
quoted in g where noted. Companion to `vehicle_params.m`, `load_transfer.m`,
`run_load_transfer_targets.m`, `ttc_fit.m`, `gg_envelope.m`, `run_gg_targets.m`,
`corner_speed.m`, `lap_sim.m`, `load_track.m`, `run_lap_targets.m`.
The scripts keep only label comments; the reasoning lives here.

---

## 0. Conventions

- **Axes:** x forward, y left, z up. `a_x > 0` accelerates, `a_x < 0` brakes; `a_y` is the
  lateral (cornering) acceleration magnitude.
- **Loads:** `F_z` is vertical tire load. On a tire it acts down; TTC data reports it
  negative, but every grip quantity is a ratio so the sign cancels.
- **Mass split:** `χ_f` = static front mass fraction (`mass_dist_f`). Front axle carries
  `χ_f`, rear carries `1 − χ_f`.
- **Single source of truth:** every number below traces to `vehicle_params()`. Derived
  quantities are computed, never hand-typed.

---

## 1. Statics — `vehicle_params.m`

A car at rest (or constant velocity) is a rigid beam on two supports. Total weight
`W = m g` acts down at the CG; the axles push up with `W_f`, `W_r`.

**Vertical force balance**

    W_f + W_r = m g

**Moment balance** about the rear contact patch (weight acts a distance `b` ahead of it,
the front reaction a distance `L` ahead):

    W_f · L = W · b      ⇒   W_f = W · (b / L)

and about the front patch, `W_r = W · (a / L)`. Each axle carries the weight fraction set
by the CG's distance from the *opposite* axle.

**Going from the measured split to the CG.** You weigh the front fraction `χ_f` on scales,
so by definition `W_f = W χ_f`. Equating to `W b/L`:

    b = L · χ_f                 (CG → rear axle)
    a = L − b = L · (1 − χ_f)   (CG → front axle)
    W_f^static = m g · χ_f
    W_r^static = m g · (1 − χ_f)

With `χ_f = 0.47`, `b = 0.47 L` and `a = 0.53 L`: a front-light car sits with its CG nearer
the rear axle, so the gap to the *front* axle is the larger one.

**Assumptions:** rigid body, flat ground, left–right symmetric (per-wheel load = half the
axle load), no aero (static).

---

## 2. Quasi-static load transfer — `load_transfer.m`

When the car accelerates, an inertial (d'Alembert) force `m a` acts at the CG, a height `h`
above the contact patches. That force times its height is a moment the tires must react by
**redistributing vertical load** between the supports that straddle the axis of the moment.
Same equation twice, with the support spacing in the denominator:

**Longitudinal** (supports = the two axles, spacing `L`):

    ΔW_long = m · a_x · h / L

Load moves onto the rear under acceleration, onto the front under braking. Combined with
the static loads:

    W_f = W_f^static − ΔW_long
    W_r = W_r^static + ΔW_long

**Lateral** (supports = left/right wheels, spacing = track `t`):

    ΔW_lat = m · a_y · h / t

The outer wheels gain `ΔW_lat`, the inner lose it. The script uses mean track
`t = ½(t_f + t_r)`.

The two are the *same* relation: `ΔW = m a h / (support spacing)`. Tall CG bad; long
wheelbase / wide track good.

**Concept-tier flag — the front/rear lateral split.** The total lateral transfer is split
between axles by static axle load (`χ_f`). The *true* split is governed by roll-stiffness
distribution and roll-centre heights, which is the single biggest handling-balance knob and
waits for the mid-tier kinematics model.

**Assumptions:** quasi-static (no transient roll/pitch build-up), one lumped CG (no separate
sprung/unsprung), no aero, no jacking.

---

## 3. Load-transfer design targets — `run_load_transfer_targets.m`

All four are built on the **static rollover threshold** — the lateral g at which the inside
wheels lift, found by setting the lateral transfer equal to half the axle load:

    a_roll [g] = (t / 2) / h

and the rule that the car must **slide before it tips**, with a safety factor `SF` over the
grip limit `μ`:

    a_roll ≥ SF · μ

**T-CGH — CG height ceiling.** Solve the rollover rule for height:

    h_max = (t/2) / (SF · μ)

**T-TRK — track floor.** Solve the same rule for track:

    t_min = 2 · h · SF · μ

These are one constraint viewed two ways. Note both *tighten* as `μ` rises — more grip means
you reach higher lateral g, so you need a lower CG / wider track to keep the same tip margin.

**T-BB — brake bias starting point.** The ideal front bias matches the front share of
vertical load at the braking limit. Routing through `load_transfer` at `a_x = −D`:

    bias_f = W_f / (W_f + W_r) = χ_f + μ · h / L

**T-MS — mass invariance.** Because transfer *and* grip both scale with `m`, the load-transfer
fractions (rollover, track, bias) are **mass-independent**. Mass moves only absolute loads:

    d(ΔW_lat)/dm = μ g h / t   [N per kg]

so the real mass *target* is deferred to the lap sim, where mass changes lap time.

**Assumptions:** rigid-body rollover (no suspension compliance, no tire deflection), no aero,
grip is a single scalar `μ`.

---

## 4. Tire grip from TTC — `ttc_fit.m`

Grip is a force ratio, which is why the lab data's unit system is irrelevant:

    μ = |F_y| / |F_z|

The script turns Calspan TTC sweeps (Round 8 for the LC0; Round 9 for R20/R20_18x60/GY) into the design grip:

1. **Isolate pure cornering** — keep samples with near-zero longitudinal force,
   `|F_x / F_z| < 0.1`, so combined-slip points don't pollute the lateral peak.
2. **Read at the design load** — the car's mean per-corner load `F_z = m g / 4 ≈ 183 lbf`,
   in a ±12% band. This matters because **tire μ falls as load rises** (load sensitivity), so
   grip must be read at the load the tire actually carries, not at the light loads where it
   looks best.
3. **Take the peak** — the 99th percentile of `|F_y/F_z|` in the band (99th, not the max, to
   reject sensor spikes). Raw peak ≈ 2.60 for the LC0.
4. **Derate to the track** — multiply by `k = 0.67`:

       μ_design = k · μ_raw = 1.74

   The lab belt (3M grit, fresh single tire, controlled temperature) is far grippier than
   competition asphalt; the derate (target #12, "grip scaling factor") scrubs it to reality.
5. **Windows** — repeat the peak sliced by camber and by pressure to map where grip is best
   (operating-window targets #9/#10).
6. **Longitudinal μ** — same extraction from the set's only drive/brake file (18in LC0),
   with the pure-slip filter flipped: near-zero *slip angle* instead of near-zero `F_x`,
   and a wider ±30% load band because drive/brake data is sparser. Used as a cross-tire
   proxy until per-tire drive/brake data exists.

**What this deliberately omits:** the *shape* of the force-vs-slip curve. Peak μ is one
scalar; the full **Pacejka / Magic Formula** model gives `F_y(α, F_z, γ)` including
cornering stiffness `C_α = ∂F_y/∂α|₀`. That curve shape is what the steady-state bicycle
model needs, so Pacejka is the **mid-tier** tire model — fitted when a consumer needs the
curve, not just the peak.

---

## 5. The g-g-V envelope — `gg_envelope.m`

The envelope is the set of accelerations the point mass can reach at a given speed `v`. It is
speed-dependent because aero scales with `v²`.

**Speed-dependent loads**

    Downforce DF = ½ ρ ClA v²        ⇒  Normal load  N(v) = m g + DF
    Drag        = ½ ρ CdA v²
    Rolling res. F_rr = C_rr · N
    Effective mass m_eff = k_rot · m   (translate the car AND spin up wheels + motor rotor)

**Lateral** — the tire friction circle scaled by downforce (steady cornering doesn't spin the
wheels up, so `m_eff` does *not* apply):

    a_y,max = μ_y · N / (m g)   [g]

**Longitudinal — braking** (all four tires, drag and rolling resistance help):

    a_x,brake = (μ_x · N + Drag + F_rr) / (m_eff g)   [g]

**Longitudinal — acceleration** is the richest part. Two ceilings, take the lower:

*Traction ceiling.* Only the driven axle makes thrust, and for RWD/FWD the driven-axle load
itself shifts with `a_x` (load transfer), so the limit is implicit and solved in closed form.
With usable grip `kμ = k_trac · μ_x` (launch utilisation — you can't deploy peak μ from a
standstill without slip control):

    AWD:  a_tr = (kμ·N − Drag − F_rr) / m_eff
    RWD:  a_tr = (kμ·N_r0 − Drag − F_rr) / (m_eff − kμ·m·h/L),   N_r0 = (1−χ_f)·N
    FWD:  a_tr = (kμ·N_f0 − Drag − F_rr) / (m_eff + kμ·m·h/L),   N_f0 =   χ_f ·N

The `−kμ m h/L` term in the RWD denominator is the rearward load transfer *helping* traction;
for FWD the sign flips and transfer *hurts*. This is why a single-motor RWD car launches at
~1.0 g while an AWD car (all four tires, full `N`) launches near `μ`.

*Power ceiling.* The motor delivers constant torque up to its base speed, constant power
above it (the 80 kW rules cap). At the wheels:

    F_motor = min( T_max · gear_ratio · η / R_e ,  η · P_max / v )
    a_pw    = (F_motor − Drag − F_rr) / m_eff

    a_x,accel = min(a_tr, a_pw)

At low speed traction limits; above the crossover (~18 m/s for this car) power limits, and
thrust decays as `P/v` — same power spread over more meters per second.

**Friction ellipse** (combined cornering + long.), used by the lap sim:

    (a_x / a_x,lim)² + (a_y / a_y,max)² ≤ 1

**Assumptions:** point mass (no individual wheel loads, no combined-slip tire curve), aero
splits front/rear like static weight (no aero-balance model yet), brakes assumed sized to
lock, `k_rot`/`k_trac`/`η` are provisional scalars pending CAD inertia + a motor map.

---

## 6. Performance events — `run_gg_targets.m`

**T-SKID — skidpad.** Steady cornering on a circle of path radius `R = 9.125 m`. Set the
required lateral accel equal to the available grip, `v²/R = a_y,max(v) g`. Because `N` is
linear in `v²` this is closed-form:

    v² = μ_y g / ( 1/R − μ_y · ½ ρ ClA / m )
    a_y = v² / (R g)   [g],     t_skid = 2πR / v

**T-ACC — 75 m acceleration.** March a small velocity step and accumulate distance and time
from the available accel at each speed:

    dt = dv / a_x,accel(v),   dx = v dt + ½ a dt²,   sum until x = 75 m

Traction-limited off the line, power-limited past the crossover.

**T-MS2 — mass sensitivity.** Finite difference: re-run the accel event at `m + 10 kg` and
report `Δt / Δm` (s/kg). This is the **acceleration** sensitivity; the full-lap value (many
accel/brake zones) needs the track lap sim.

**T-GG / T-PWR** report the envelope corners and where power becomes the binding limit.

---

## 7. QSS lap solver — `corner_speed.m`, `lap_sim.m`, `load_track.m`, `run_lap_targets.m`

A quasi-steady-state (QSS) lap sim treats a lap as a sequence of points along the racing
line, each with an arc length `s` and curvature `κ = 1/R` (from `load_track.m`), and asks
at every point: what is the fastest speed consistent with the g-g-V envelope?

**Corner-speed ceiling — `corner_speed.m`.** Steady cornering at curvature `κ` demands
`a_y = v²κ`; the lateral limit supplies `a_y,max(v)·g`. Because downforce makes capability
rise with speed, the equation `v²κ = a_y,max(v)·g` has a unique crossing, found by bisection.
Straights (`κ ≈ 0`) return the rev-limited top speed.

**Lateral limit source — `ay_limit.m` (the model switch).** As of the Jul 2026 upgrade the
lateral limit `a_y,max(v)` no longer comes straight from `gg_envelope`. Both `corner_speed`
and `lap_sim` call `ay_limit(p, v)`, a single evaluator selected by `p.grip_model`:

- `'axle'` (default) → `axle_grip(p,v).ay_lim_g`, the **load-sensitive** per-axle limit
  (§11): transferring load onto the outer tire *loses* grip because μ falls with load, so
  `a_y,max ≈ 0.88–0.90·μ_y·g`, not `μ_y·g`. This is the realistic mid-tier ceiling.
- `'pointmass'` → `gg_envelope(p,v).ay`, the old constant-μ envelope (`≈ μ_y·g`), kept only
  to reproduce the earlier optimistic lap for a before/after comparison.

This upgrade is **lateral-only**: the longitudinal edges (`a_x,accel`, `a_x,brake`) still come
from the point-mass `gg_envelope`, and the friction ellipse then couples them against the new
`a_y,max`. Effect (validated in Python and MATLAB): the endurance lap goes from 65.7 s
(point mass) to 68.7 s (axle) — the point mass was **~4.4 % optimistic** — with the whole gap
coming from corner-entry/apex/exit; straights are unchanged. `lap_report` prints both times.

**Speed trace — `lap_sim.m`.** Three steps:

1. **Ceiling pass** — every point gets its corner-speed limit `v_lim(s)`.
2. **Forward pass (accelerate)** — march forward; at each point, grip already spent on
   cornering shrinks what remains for thrust via the friction ellipse,

       frac = √(1 − (a_y,used / a_y,max)²),   a_x = a_x,accel · frac

   then `v² = v² + 2·a_x·ds`, capped at the next point's ceiling.
3. **Backward pass (brake)** — same march in reverse using `a_x,brake`, so the trace slows
   down *early enough* before each corner. Taking the min of both passes yields a trace
   that respects the envelope everywhere.

Closed loops iterate 3× so the start/finish speeds settle. Lap time is `Σ ds / v_mid`.
This forward/backward structure is the standard QSS method (look up: *quasi-steady-state
lap time simulation*); the same idea transplanted into Simulink with real dynamics is the
transient sim end-goal.

**Targets — `run_lap_targets.m`.** Re-derives accel/skidpad through the lap solver as a
consistency check against `run_gg_targets`, adds lap times on digitized tracks and the
full-lap mass sensitivity T-MS3 (finite difference, +10 kg, with `k_rot` recomputed since
rotating inertia stays fixed while mass grows).

---

## 8. Magic Formula tire fit — `pacejka_fit.m`

Everything before this section used one number per tire (peak μ). The Magic Formula
(Pacejka) replaces it with the whole force curve:

    F_y = D sin( C atan( Bα − E(Bα − atan Bα) ) )

An empirical shape, no physics: `D` = peak force (μ·F_z), `B·C·D` = slope at α=0
(**cornering stiffness** C_α, the quantity the bicycle model consumes), `C` sets the
post-peak falloff, `E` the knee sharpness. Fitted per load bin, then load dependence:
`C_α(F_z)` as a quadratic, and **μ linear in load** (Pacejka's p_Dy1/p_Dy2 form —
better conditioned than fitting the force D and dividing by F_z), with
D(F_z) = μ(F_z)·F_z derived.

**Fitting method.** Raw TTC point clouds contain warmup segments, sweep-junction
artifacts (a visible glitch at +5–7° SA), and hysteresis loops; a naive least-squares
over raw samples gets dragged by them (C pins at its bound, D inflates ~40%). The fix:
mirror the negative-α branch onto the positive one (kills plysteer/conicity offsets),
take per-half-degree **medians**, and fit the ~23-point curve. R² goes from ~0.87 to
~0.999.

**MF peak vs the ttc_fit percentile.** `ttc_fit`'s 99th percentile reads the *upper
envelope* of the scatter; the MF `D` reads the *median* curve. With the identified-bin
trend the MF design-load peak is ≈2.36 vs the 2.60 percentile (~9% lower). The
percentile number is the optimistic edge of the same data — a choice to make
consciously when the two feed different consumers, and the size of the planned
percentile→curve re-issue.

**Known caveat — unidentifiable peaks at high load.** The slip angle of peak force
grows with load; at the heaviest bins it moves past the 12° sweep limit, and a curve
that is still rising at the last data point cannot pin down `D` (for C→1 the MF is
near-monotonic, so C and D trade off freely — the fitted peak biases high, producing
an unphysical μ spike at 250 lbf for LC0/R20_18x60). Fix: bins whose fitted curve still
rises >2% between 10° and 12° are flagged (`peak_in_sweep = false`, `*` in the table,
hollow markers in `tire_report`) and excluded from the μ(F_z) trend. Design-load μ
comes from the identified-bin trend: LC0 ≈ 2.36 at 183 lbf (vs 2.60 percentile, ~9%
lower). Notable finding once the artifact is removed: the LC0's load sensitivity is
~2× steeper than the other three candidates.

**Two belt-to-track scalings, not one.** Peak grip is surface-dominated → harsh scaling
(λ_μy = `mu_derate` = 0.67, fitted at skidpad). Cornering stiffness is carcass-dominated
→ gentle scaling (λ_Kyα = `lambda_Ca` ≈ 0.9 provisional, fitted from steering-response /
understeer-gradient tests). Scaling the whole curve by 0.67 would understate stiffness
by a third and corrupt bicycle-model predictions. These are the Pacejka standard scaling
factors LMUY / LKY.

**Cross-tire longitudinal transfer.** Only the 18in LC0 (Hoosier 18.0x6.0-10, Round 6) has
drive/brake data — note the transfer to the 16x7.5 crosses diameter, section width, and rim
width. The 18in never gets a full lateral sweep either (SA is *held* at ~0/−3/−6°), so its
lateral **peak is never measured** — only its value at 6°. The design tire's MF curve supplies
the shape correction.

**The shape correction must be load-matched.** This is where the original hand-calc went wrong.
It used 0.88 — the fraction of peak reached at 6° at the **design** load, 183 lbf — but the
18in lateral data sits at **~245 lbf**. Tire curves flatten as load rises, and at 245 lbf the
correct factor is **0.852**. You must evaluate the correction at the load the data actually
lives at.

With that fixed, and with μ_x read off the fitted MF curves rather than a percentile:

    μ_x(18in) = ½(drive 2.467 + brake 2.311) = 2.389
    μ_y(18in) peak = 2.102 / 0.852            = 2.469
    anisotropy = 2.389 / 2.469                = 0.968

So the construction is **mildly longitudinally-weak**, not isotropic. Applied to the 16in:
`mu_x_raw = 2.336 × 0.968 = 2.261` (the old percentile chain gave 2.602 × 1.008 = 2.622).
The *ratio*, not the absolute value, is assumed to transfer across the construction family.

**This is now computed, not asserted.** `mu_anisotropy = 1.008` previously existed only as a
literal in `vehicle_params.m` with a comment describing a hand calculation — nothing in the
codebase could reproduce or check it. `build_tire_coeffs.m` now derives it from the fits and
stamps it into `tire_coeffs.mat`; see §8a.

---

### 8a. Percentile → curve: the grip-basis transition (Jul 2026)

Design grip used to come from `ttc_fit`'s **99th-percentile** of |F_y/F_z| over the samples in
the design-load band. That reads the **upper envelope of a noisy point cloud**. The Magic
Formula peak reads the **fitted median curve**. These answer different questions, and the
percentile is the wrong one for a design value: it is a near-best-case sample, and it inherits
the noise it was meant to reject.

The whole toolchain moved to the curve basis in one deliberate re-issue:

| quantity | percentile | curve | Δ |
|---|---|---|---|
| `mu_y_raw` (LC0 @183 lbf) | 2.602 | **2.336** | −10.2% |
| `mu_anisotropy` | 1.008 | **0.968** | −4.0% |
| `mu_x_raw` | 2.622 | **2.261** | −13.8% |
| derated μ_y (λ=0.67) | 1.743 | **1.565** | |
| derated μ_x | 1.757 | **1.515** | |

**Honest limit of the new number.** The LC0's lateral peak is *not reached* inside the ±12°
TTC sweep at high load — the 250 lbf bin fails the `peak_in_sweep` test and is excluded from
the μ(F_z) trend for exactly that reason. μ_y at the design load therefore still rests on an MF
**extrapolation beyond the data**. Trust it to a few percent, not better. This is a limitation
of the *test*, not of the fit: the sweep does not go far enough.

**Two consequences that matter more than the numbers.**

1. **The CG-height / track-width rollover conflict dissolved** — margin 1.22× → 1.36× against a
   1.3× target. Not because the car got safer. The rollover margin is `(t/2)/(h·μ)`, which
   scales as **1/μ**: a car that grips less *slides before it tips*. The conflict was partly an
   artifact of an over-optimistic grip number. It **re-binds if λ_μy > 0.70**, and λ_μy is
   currently an assumed 0.67 that gets **fit at skidpad** — that is only 4.7% of headroom.
   Packaging should not spend this margin until the skidpad number exists.

2. **The cross-tire ranking changed.** Percentile: GY > R20_18x60 > R20 > LC0. Curve: **R20_18x60 >
   GY > R20 > LC0**. The chosen LC0 is last on both bases, but the gap to the leader widens
   from ~5% to ~10%. Target #8 (tire choice) deserves the lap-sim comparison it has been
   waiting for.

**Longitudinal curve fit (single-load).** The drive/brake torque sweeps at zero slip
angle exist only at ~250 lbf. Slip ratio requires a frozen free-rolling radius (the RE
channel is defined as V/ω per sample, so instantaneous RE gives SR ≡ 0). Curve-based
peaks: drive μ_x ≈ 2.46, brake ≈ 2.32 (asymmetric tire), both below the percentile read
— same envelope-vs-median-curve gap as the lateral side. Longitudinal slip stiffness
K_x is a new deliverable (traction-control targets, Simulink wheelspin). Design values
remain percentile-based pending the one planned percentile→curve transition.

**Aligning moment and pneumatic trail.** The contact patch's lateral force acts a
distance behind the wheel center — the *pneumatic trail* t_p — producing the aligning
moment M_Z = F_y·t_p that tries to straighten the wheel. Trail exists because the
patch's force distribution is triangular: rubber deflects progressively through the
patch, so the resultant sits rearward. As slip angle grows and the rear of the patch
slides, the resultant moves forward — trail *collapses* (LC0: ~1.3 in at 1–2° to
~0.2 in near the limit) and M_Z peaks around 4° then dies even as F_y keeps rising.
This is the mechanism of steering feel: effort goes light *before* the front axle
peaks, which is the driver's grip warning. Consumers: steering effort/rack force
(#54), the Milliken Moment Method, and any steering-feel or driver-model work.

**Friction-envelope exponent — the ellipse assumption audited.** The held-SA (3°/6°)
combined sweeps trace real (F_x, F_y) points. Fitting |f_x|ⁿ + |f_y|ⁿ = 1 to the
max-over-slices envelope gives n ≈ 1.8 in both drive and brake quadrants — a *lower
bound*, since slip angles beyond 6° (where the lateral peak lives) weren't swept.
True n plausibly 1.9–2.2. Conclusion: the n = 2 friction ellipse used by `lap_sim`
and the g-g plots is consistent with the data; notably there is NO support for the
common "boxier than elliptical" (n > 2) assumption for this tire.

---

## 9. Why concept-tier numbers are optimistic (the fidelity ladder)

Point-mass concept models share a systematic bias: they assume the car can use **peak grip,
everywhere, instantly**, and they ignore loss mechanisms. Each rung of fidelity you add tends
to *remove* capability, so predicted times generally **increase toward reality** as the model
grows. Examples already seen and still to come:

| Effect added | Direction | Where it enters |
|---|---|---|
| Rotating inertia (`k_rot`) | slower accel & braking | g-g (done) |
| Realistic driveline η | slower accel | g-g (done) |
| Launch traction utilisation (`k_trac`) | slower accel | g-g (done) |
| RWD vs AWD (driven-axle limit) | slower accel | g-g (done) |
| Tire **load sensitivity** on the loaded outer tire | lower skidpad/lat g | mid-tier |
| Lateral load transfer reducing effective axle μ | lower skidpad/lat g | mid-tier |
| Combined-slip friction ellipse (real, not circular) | smaller envelope | mid-tier |
| Thermal/pressure drift, tire wear | lower μ | late-tier |
| Motor torque-speed curve below peak power | slower accel | mid-tier (#42) |

**Worked example — skidpad.** The concept model uses a single `μ_y = 1.565` at the 183 lbf
design load. But in a corner load transfers to the outer tires, which then sit near ~260 lbf,
where the fitted `μ(F_z)` trend gives roughly `μ ≈ 1.50` derated. The lightly loaded inner
tires cannot make up the difference — they lose grip faster than the outers gain it, because
μ falls with load. So the *effective axle* μ is below the design-load value, and the honest
skidpad number is **lower than the 1.62 g the point mass reports**. That gap is exactly what
the mid-tier axle-grip model exists to close. Quote 1.62 g as a **ceiling**, not a prediction.

---

## 10. Bicycle model — `bicycle_model.m`, `run_handling_targets.m`

> **Note (Jul 2026):** this section was rewritten during the refactor. The equations were
> re-derived from `bicycle_model.m` and verified against it line by line, but the prose is
> not the original author's.

The linear two-axle (single-track) model collapses each axle to one tire on the centreline.
Valid only in the **linear tire range** — small slip angles, well below the friction limit —
so it says nothing about grip. It is a *balance and response* model, not a lap-time model.

**Axle cornering stiffness.** Each axle's stiffness is twice the per-tire `C_α` at that axle's
static per-tire load, scaled belt→track:

    C_f = 2 · C_α(F_z,f) · λ_Kyα ,   C_r = 2 · C_α(F_z,r) · λ_Kyα

with `C_α(F_z)` read from the **promoted artifact** (`p.Ca_coef`, from `tire_coeffs.mat`), not
from a live re-fit. Note this uses **λ_Kyα ≈ 0.90**, not λ_μy = 0.67 — stiffness is
carcass-dominated, grip is surface-dominated (§8). Merging them would understate stiffness by
a third and corrupt every number below.

**Understeer gradient.** From the steady-state relation `δ = L/R + K·a_y`:

    K = W_f/C_f − W_r/C_r        [rad/g]

- `K > 0` understeer, `K < 0` oversteer, `K = 0` neutral.
- **TR26: K ≈ −0.08 deg/g — effectively neutral.** This is a *small difference of two large
  numbers*, so it is ill-conditioned: **trust the sign, not the decimals.** It will move once
  the mid-tier roll-stiffness model lands, and it is not a design target yet.

**Stability speed.** For `K < 0` (oversteer) there is a **critical speed** above which the car
is open-loop unstable; for `K > 0` a **characteristic speed** of peak yaw response:

    v_crit = sqrt(g·L / −K)   (K<0)        v_char = sqrt(g·L / K)   (K>0)

TR26's v_crit lands ~3.4× v_max — far outside the operating envelope, so the near-neutral
balance is not a stability problem.

**Yaw-rate gain.** Steady-state `r/δ`:

    r/δ = v / (L + K·v²/g)

This is also the **reference model for torque vectoring** (#56/#58): it is the yaw rate the
controller should target for a given steer input and speed.

**Transient response.** With states `[β; r]` (sideslip, yaw rate), from
`m·v·(β̇ + r) = F_yf + F_yr` and `I_zz·ṙ = a·F_yf − b·F_yr`:

    A(v) = [ −(C_f + C_r)/(m·v)          −1 + (b·C_r − a·C_f)/(m·v²) ]
           [ (b·C_r − a·C_f)/I_zz        −(a²·C_f + b²·C_r)/(I_zz·v) ]

The eigenvalues give the yaw mode: `τ = 1/min|Re(λ)|` and an equivalent damping ratio.
TR26 is **overdamped** (real eigenvalues) with τ ≈ 50–120 ms across the speed range — the car
settles quickly and does not oscillate in yaw.

**Caveats.** Linear tires (no saturation, so nothing here survives at the limit); static axle
loads (no load transfer, no roll); rigid chassis; `I_zz` from a provisional dynamic index
DI = 0.75 (#6) — which affects τ and ζ but **not** K, v_crit or the yaw gain, all of which are
steady-state and inertia-free. The MMM / yaw-moment diagram (roadmap 4) is what extends
balance analysis to the nonlinear range.

## 11. Mid-tier axle-grip model — `axle_grip.m`, `run_balance_targets.m`

**What it adds over the point mass.** The point-mass g-g envelope uses one scalar grip:
`ay_max = mu_y * N_total / (m g)`. But the tire fit says mu **falls with load**
(`mu_coef`, the pDy1/pDy2 load-sensitivity line from section 8). In a corner, load transfers
from the inside tires to the outside tires, and because mu(Fz) is a falling line, the outside
tire gains less grip than the inside tire loses:

$$F_{y,axle}^{max} = \mu(F_{z,out})F_{z,out} + \mu(F_{z,in})F_{z,in} \;<\; \mu(F_{z,axle}/2)\,F_{z,axle}$$

This concavity is why real cornering limits sit below `mu_y * g` and why track width, CG
height and roll stiffness distribution matter at all. Look up: **lateral load transfer
sensitivity** (Milliken RCVD ch. 18).

**The two demands.** Steady-state moment balance about the CG forces each axle to supply a
fixed share of the total lateral force: `Fy_f = m a_y b/L` (the front's share equals the front
static weight fraction) and `Fy_r = m a_y a/L`. The lateral limit is the largest `a_y` at
which BOTH axles can still meet their share. Whichever axle runs out first sets the **limit
balance**: front-limited = terminal understeer (stable, the car pushes wide); rear-limited =
terminal oversteer (the rear lets go — snap).

**LLTD, the one chassis knob.** The total transfer moment `m a_y h` splits between the axles
in proportion to their roll stiffnesses (plus geometry we ignore at this tier). The front
fraction is the **lateral load transfer distribution** (LLTD). Shifting LLTD forward loads
the outside-front harder → front axle loses relatively more grip → limit balance moves toward
understeer. Peak total grip occurs where both axles saturate together (the **neutral point**,
LLTD ≈ 0.57 for TR26 today); the design recommendation sits just front of neutral, buying
stability for ≤2% grip. Below neutral the car gives its best number and bites the driver.

**Aero CoP (target #38).** Downforce enters axle loads through `aero_df_front`. Because
downforce grows with v² while weight does not, the limit balance can FLIP with speed: a CoP
too far rearward keeps the rear planted (fine) until the demand share overtakes it — a CoP
too far forward starves the rear at speed → high-speed limit oversteer. The band printed by
`run_balance_targets` is the CoP range that stays front-limited at 95% of v_max at the
target ClA while giving up ≤2% of the best high-speed grip.

**Validation.** The mid-tier skidpad (~5.08 s) and the point-mass skidpad (4.50 s) bracket
the real 2026 best (4.782 s): load sensitivity accounts for most of the QSS haircut that was
previously an empirical fudge. That is the fidelity ladder (section 9) doing its job.

**Caveats.** Single-knob LLTD — no roll centers, no unsprung/geometric split, no camber
(fitted at IA = 0), no compliance. CoP fixed with speed, though a real undertray's CoP moves
with ride height and pitch. Steady state only — the transient model (Simulink) inherits
these axle capacities as its saturation limits.

---

## 12. Weight-distribution target — `run_wdist_targets.m`

Static front/rear weight split (`mass_dist_f`, target #2). The model sweeps it through the
mid-tier chain and reports four things per split: lateral grip (`axle_grip`), launch traction
(`gg_envelope`, RWD), straight-line braking (per-axle, load-sensitive, ideal bias), and the
LLTD needed to balance the limit.

**The governing result is that steady-state metrics cannot find the optimum.** For this car:

- **Lateral grip is flat** — ~1% across 38–52% front, front-limited throughout. Weight split
  is not a grip lever.
- **Braking mildly improves rearward** — braking transfers load forward, so a rear-static car
  is *more* balanced under the brakes (`ΔW_long = m·a_x·h/L` unloads the rear, and a rear-heavy
  static car ends nearer 50/50 at the decel limit, where load-sensitive total grip peaks).
- **Launch traction strongly favours rearward** — RWD tractive force scales with the driven
  (rear) axle load; ~+24% from 50% → 38% front.
- **LLTD balances the limit across the whole range** — the roll-stiffness split has enough
  authority to make any of these front-limited.

So every steady-state axis favours rearward or is indifferent, and the sweep bottoms out at
its rearward edge. **This is a blind spot, not an answer.** What actually caps rearward bias is
*transient* yaw stability — turn-in response, trail-brake rotation, snap-oversteer margin —
governed by the yaw-plane dynamics of §10, not by any steady-state limit. A car that is
perfectly balanced at the steady-state limit can still be undriveable in transients if the CG
is too far back (reduced static margin → the car wants to rotate).

**Design rule.** Weight distribution is a packaging decision made once and hard to change; LLTD,
aero balance and brake bias are per-session knobs. So set the weight split for what the knobs
*cannot* fix — traction, packaging, and the transient-stability floor — and trim handling
balance with LLTD afterward. Do not spend the weight split chasing steady-state balance.

The target is therefore issued as "rearmost that transient stability allows", with a provisional
front floor from FSAE convention (~44%) until the transient model (roadmap #5) or track data
sets the real limit. The current 40% front is accel-optimal but below that floor and flagged for
validation.

---

## 13. Design notes relocated from code headers (Jul 2026 cleanup)

Code headers were cut to 1-3 lines; the mechanism notes they carried live here.

**mu_of_load piecewise law.** Below the design tire's data edge (`Fz_fit_max`), mu(Fz) is the
measured linear fit. Above it, a donor-informed slope extracted from the pooled 18in tires'
normalized load-sensitivity shape (cross-tire agreement ~1%, which justifies the transfer;
blind linear extrapolation would be far too pessimistic). `p.tire_hiload = 'low'` forces the
blind-linear extension; the band between the two modes is the honest extrapolation
uncertainty. Physical floor mu = 0.1 with a warning.

**Friction-ellipse exponents.** Measured from the 18in LC0 held-SA combined sweeps:
n_drive = 1.78, n_brake = 1.82 (vs the classic circle n = 2). The envelope SHAPE is a
construction property that transfers cross-tire, like the mu_x/mu_y anisotropy. The measured
n is a lower bound (SA only swept to ~6 deg), so it is the conservative edge.

**Belt-to-track scalings.** Peak grip is surface-dominated: lambda_muY = 0.67, fit at
skidpad. Cornering stiffness is structural: lambda_KyA = 0.90 (literature ~0.92-0.96,
belt reads high). Pneumatic trail is the RATIO of two structural stiffnesses
(aligning/cornering), so it is ~surface-invariant: lambda_t = 1.0. Never merge these.

**ay_limit / ay_lut.** One lateral-limit source for corner_speed and lap_sim, switched by
`p.grip_model` ('axle' = load-sensitive default, 'pointmass' = old optimistic comparison).
Because ay(v) varies only through downforce (~v^2, smooth and monotone), a 120-point lookup
per lap is exact to ~1e-4 g and avoids the nested-bisection cost (~35k axle_grip calls to
~120 per lap sim).

**Caster / T-CAS.** Steering torque per tire = Fy(alpha) x (mechanical + pneumatic trail).
Pneumatic trail collapses toward the limit, so with zero caster the wheel goes light exactly
at the edge; constant mechanical trail restores limit feel. The 3-6 deg recommendation is a
cited practice window (kept below peak pneumatic trail so it complements, not overpowers,
self-centering - no power steering); the steering team finalizes against its effort budget
once the ratio exists. No invented feel threshold selects the value.

**Understeer gradient under longitudinal transfer.** Ca(Fz) is concave, so the needed-slip
ratio W/Ca RISES with load: braking loads the front -> understeer; power loads the rear ->
oversteer (normal RWD, managed by diff/throttle/LLTD, not by static weight split). The
linear model cannot see trail-brake rotation - that is a friction-circle effect at the limit
requiring combined-slip per-axle grip. K is ill-conditioned: trust sign and trend.

**Rev limit is voltage-governed.** The Emrax 228's 5500 rpm rating is reached at 470 Vdc;
at this pack (~299 V nominal) back-EMF limits the motor to ~4500 rpm, giving ~24.7 m/s at
3.82:1. More top speed requires pack voltage, not a higher software limit.
