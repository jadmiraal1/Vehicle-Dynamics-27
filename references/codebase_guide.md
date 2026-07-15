# Codebase guide

The deep documentation: design philosophy, the generated-artifact safety
pattern, per-file reference, data provenance, conventions, and roadmap.
For a quick orientation see the README; for physics derivations see
VD_physics_reference.md (cited below as sec N).

---

## Philosophy

1. **Targets are decisions**, not plots: "design to X because the model shows Y."
2. **One source of truth**: every number traces to `vehicle_params.m`. No
   constant may live only inside a script — and no *model output* may live in
   `vehicle_params` as a hand-typed literal either (see below).
3. **Never block a subteam**: issue a provisional number from literature or
   hand-calc, refine when the model exists, validate against test data.
   Status pipeline: Provisional → Delivered → Validated (see the tracker).
4. **Fidelity tiers**: concept (point mass, single mu) → mid (load-sensitive
   axle grip, roll stiffness distribution) → transient (Simulink). Each tier's
   numbers are optimistic; each added effect usually removes capability (§9).
5. **Refine on consumer demand**: a model gets more detail only when something
   downstream needs it.

## Generated inputs, not typed inputs (Jul 2026)

Grip used to reach the car by hand-copy: `ttc_fit` printed a number, you typed
it into `vehicle_params`. Two things were wrong with that. The same number
lived in two places and could silently disagree — and `mu_anisotropy = 1.008`
had **no code behind it at all**, it was a hand calculation preserved in a
comment, unreproducible and uncheckable.

Now grip is a **generated artifact**:

```
build_tire_coeffs   ->   tire_coeffs.mat   ->   vehicle_params (loads it)
   (run deliberately)      (checked in)          (never hand-types grip)
```

`build_tire_coeffs` is the *only* producer of design grip. `vehicle_params`
only consumes. The two are decoupled on purpose: re-fitting the tire does
**not** silently change the car — you must run the build, deliberately, and
then re-issue the grip-derived targets.

That safety is enforced, not merely intended. `tire_coeffs.mat` stores a hash
of every input it was built from (the fit code + the TTC files). `vd_selftest`
recomputes that hash and **fails loudly** if the artifact has gone stale, and
also greps `vehicle_params.m` to make sure nobody has re-introduced a hard-coded
`mu`. You cannot silently drift, and you cannot silently forget to rebuild.

Note what stayed hand-typed, correctly: `mu_derate` (λ_μy) and `lambda_Ca`
(λ_Kyα) are belt→track **design decisions**, not fit outputs. They belong in
the parameter file.

## Data flow

```
TTC_Data/*.mat  (Calspan tire tests, US units)
   |         \
 ttc_fit    pacejka_fit          tracks/*.png
(screening)  (DESIGN GRIP)            |
   |            |                digitize_track.py  (scale AUTO-CALIBRATED
   +-----+------+                     |              from the paddock grid)
         |                            v
   build_tire_coeffs            track_*.csv  --> load_track
         |  (deliberate promotion,          \        |
         v   hashes its inputs)              \       |
   tire_coeffs.mat  ---------.                \      |
                              v                \     |
                       vehicle_params.m  <-- single source of truth
                              |                      |
   +--> load_transfer --> run_load_transfer_targets  |   (T-CGH, T-TRK, T-BB)
   +--> gg_envelope ----> run_gg_targets             |   (T-SKID, T-ACC, T-GG)
   |        |                                        |
   |        +--> corner_speed --> lap_sim --> run_lap_targets   (T-LAP, T-NRG)
   |                                  ^              |
   |                                  +--------------+
   +--> bicycle_model --> run_handling_targets  (T-USG, T-VCR, T-YRG, T-YAW)
              ^ [axle Ca from tire_coeffs Ca(Fz) x lambda_Ca]

 vd_selftest checks: (0) artifact freshness  (1) formula wiring  (2) data anchors
```

---

## File reference

### `vehicle_params.m` — the parameter file (§1)
`p = vehicle_params()`, or `vehicle_params('bootstrap')` for grip-free access
(used only by the tire fits, to break the circular dependency). **Three tiers:**

- **inputs** — measured or decided, each with a source comment.
- **loaded** — produced by a model, read from a generated artifact, *never*
  hand-typed here. Currently: `mu_y_raw`, `mu_anisotropy`, `mu_coef`,
  `Ca_coef`, all from `tire_coeffs.mat`.
- **derived** — computed from the two above: `mu_x_raw`, `mu_y`, `mu_x`,
  `k_rot`, `Izz`, `P_max`, `v_max`, static axle loads.

Key parameters and provenance:
- `mu_y_raw` — **loaded**, curve basis, ≈2.336 (was a hand-typed 2.602)
- `mu_anisotropy` — **loaded**, ≈0.968, *computed* by `build_tire_coeffs`
  (was a hand-typed 1.008 with no code behind it)
- `mu_derate = 0.67` — λ_μy grip scaling, belt→track; a **decision**, stays
  here; **fit at skidpad**. Also gates the rollover margin — see #3/#5.
- `lambda_Ca = 0.90` — λ_Kyα stiffness scaling; **fit from steering response**
- `Crr = 0.014` — measured from TTC free-rolling FX
- `b_driveline, Tc_driveline` — TR26 roll test spin-down fit (no-load, prov.)
- pack (`V_pack_nom/max, I_pack_max, E_pack_Wh`) — powertrain, Jul 2026;
  `P_max = min(rules 80 kW, V_nom*I_max) = 62.7 kW` — **the pack, not the
  rules, limits power**
- `T_motor_max = 220` Nm — Emrax 228 datasheet, confirmed by inverter testing
- `I_wheel = 0.217` — CAD (lb·in² converted; includes tire), `I_rotor` datasheet
- `DI = 0.75` — dynamic index for Izz estimate, provisional pending CAD (#6)

### `params_report.m` — parameter export for humans **(new)**
`params_report()`. One-way dump of `vehicle_params()` into
`organization/vehicle_params_report.xlsx` (name, value, tier, provisional
flag, provenance note parsed from the source comments). For subteam leads
who will never open MATLAB. Generated, never edited — regenerate after any
parameter change.

### `build_tire_coeffs.m` — the tire→car promotion step **(new)**
`T = build_tire_coeffs()`. Runs `pacejka_fit`, evaluates the design tire at the
design corner load, computes the μ_x/μ_y anisotropy, and writes
`tire_coeffs.mat` stamped with a hash of every input it read. **Run
deliberately**, only when the TTC data or the fit changes — then re-issue the
grip targets. This is the only producer of design grip.

The anisotropy is the interesting part. The 18in LC0 is the only tire with
drive/brake data, but it never gets a full lateral sweep (SA is *held* at
0/−3/−6°), so its lateral peak is never measured — only its value at 6°. The
design tire's MF curve supplies the shape correction. The old hand-calc applied
the shape factor at the *design* load (0.88 @ 183 lbf) to data taken at
*245 lbf*; tire curves flatten with load, and the load-matched factor is 0.852.
That, plus the median-curve basis, is the whole 1.008 → 0.968 move.

### `vd_hash.m` — reproducible content hash **(new)**
MD5 over a file list, defined simply enough to reproduce in Python (the
definition is in the header). Used by `build_tire_coeffs` to stamp the artifact
and by `vd_selftest` to detect staleness.

### `ttc_fit.m` — tire **screening** from TTC data (§4)
`R = ttc_fit()`. Peak lateral mu per candidate tire at the design corner load
(±12% band), 99th-percentile method, camber and pressure windows, longitudinal
mu from the 18in drive/brake file. **No longer a source of design values** —
the percentile reads the upper envelope of a noisy cloud and ran ~10% high in
μ_y, ~16% high in μ_x. Kept for what percentiles are good at: comparing tires
on equal terms and locating the camber/pressure windows. `vd_selftest` still
anchors on its LC0 output (2.602 ± 0.03) as a check that the TTC *data path*
hasn't moved. Writes `ttc_fit.png`.

### `pacejka_fit.m` — Magic Formula curve fit, all tires (§8)
`R = pacejka_fit()`. Fits FY = D·sin(C·atan(Bα − E(Bα − atan Bα))) per load
bin for **every candidate tire** (`R.LC0`, `R.R20`, `R.R20_18x60`, `R.GY`); the
design tire (`p.tire_data_prefix`) is mirrored at top level for downstream
code. Method: symmetrized per-half-degree *median* curve (robust to sweep
artifacts — raw-point fitting fails on this data), then `D(Fz)` and `Ca(Fz)`
quadratics per tire. R² ≥ 0.989 per bin. Consumers: `run_handling_targets`
(cornering stiffness); eventually the Simulink tire. `R.eval(alpha_deg,
Fz_lbf)` is a callable design tire. Notes: the MF peak reads the median
curve, ~5–7% below ttc_fit's percentile (upper envelope) — a conscious
methodological difference (§8); cross-tire comparison shows the GY is both
the grippiest *and* the stiffest (Ca ~174 vs LC0 ~133 lbf/deg at 250 lbf).
Writes no figures — `tire_report` is the presentation layer.

### `tire_report.m` — presentation figure suite
`tire_report()`. Publication-grade tire figures into `plots/`: per-tire MF
fits vs data, the 3D F_Y(α, F_Z) surface for the design tire (fit
interpolated between measured load bins), aligning moment M_Z + pneumatic
trail (steering-feel/MMM inputs, targets #54), and the cross-tire load
dependence comparison, and the combined corner/drive friction cloud
(tire + vehicle axes, superellipse overlays — the visual form of the
envelope-exponent audit). Separation of concerns: `pacejka_fit` fits,
`tire_report` presents.

### `load_transfer.m` — quasi-static weight transfer (§2)
`LT = load_transfer(p, ay, ax)`, accelerations in [g]. dW = m·a·h_cg/(L or
track). Lateral split between axles by static load — placeholder until the
mid-tier roll-stiffness model. Consumer: `run_load_transfer_targets`.

### `run_load_transfer_targets.m` — chassis targets (§3)
`out = run_load_transfer_targets()`. Four decisions: CG-height ceiling and
track-width floor (slide-before-tip, SF=1.3 over grip), brake bias starting
point (dynamic front load fraction at limit decel), and the proof that mass
moves none of them (fractions are mass-invariant). Writes
`load_transfer_targets.png`.

### `gg_envelope.m` — point-mass g-g-V capability (§5)
`GG = gg_envelope(p, v)`, v scalar or vector, outputs in [g]. Three edges:
lateral μ_y·N/(m·g); braking (all tires + drag + rolling + driveline drag
assist, no power limit); acceleration = min(traction, motor). Traction couples
with longitudinal transfer (closed form per drive layout); motor = torque cap
below base speed, η·P/v above. All physics in m/s², one ÷g conversion at the
interface. THE car model consumed by everything downstream.

### `run_gg_targets.m` — performance targets (§6)
`out = run_gg_targets()`. Skidpad (closed form), 75 m accel (forward Euler in
velocity steps — a lap sim in miniature), envelope cardinal points, grip→power
crossover, accel mass sensitivity (+10 kg finite difference). Writes
`gg_envelope.png`.

### `corner_speed.m` — cornering speed ceiling (§7)
`vc = corner_speed(p, kappa)`. Bisection on v²κ ≤ a_y,max(v)·g (unique
crossing because downforce raises capability with speed). Straights return
the rev-limited v_max.

### `lap_sim.m` — QSS lap solver (§7)
`[v, t, E] = lap_sim(p, s, kappa, v0, closed)`. Three speed caps per point —
cornering ceiling, reachable-from-behind (forward pass), stoppable-from-ahead
(backward pass) — final trace is the min everywhere; braking points *emerge*
at cap intersections. Friction ellipse scales longitudinal capability by
lateral usage. Time is an output (Σ ds/v), never a state — the defining QSS
property. Optional third output: energy accounting (drive Wh at wheel and
accumulator, braking Wh as the regen upper bound).

### `load_track.m` — track CSV reader (§7)
`[s, kappa, x, y, prov] = load_track(fname)`. Columns s_m, x_m, y_m,
kappa_1perm, behind a `#` provenance header. Header length is **detected**, not
assumed (it used to be hard-coded to 1 line). `prov` returns the source PNG,
the calibrated scale, and whether the slalom spacings were verified — so a run
script can state exactly which map produced its lap time.
`track_representative.csv` is a synthetic fallback loop.

### `tracks/digitize_track.py` — course map PNG → track CSV
`python digitize_track.py --all`. Extracts the red course line, skeletonizes,
orders it into a path, spline-smooths, takes curvature, and injects slalom
weave where dashed cone markings sit off the centreline. Per-track config lives
in one `TRACKS` dict at the top.

**Scale is auto-calibrated** from the paddock grid (peak-find on the grid
lines). It used to be a hard-coded `GRID_PX = 11.6`; the sheets actually measure
**12.00 px**, a 3.4% distance error that inflated *every* lap time and energy
number — endurance was 999 m, it is really 964 m. Auto-calibration means that
class of bug cannot recur. The spline is also fitted in **pixel** space, not
metres, because `splprep`'s smoothing parameter is in the units of its input —
fitting in metres made the amount of smoothing depend on the scale factor.

⚠️ **The slalom cone spacings are still UNVERIFIED.** The 1024-px sheet exports
are too coarse to read the annotations (at 9× zoom the autocross note is legible
only as "33'→30'" *or possibly* "35'→50'"), and the previous values were carried
over from an *earlier year's sheet*. Since `k_peak = A·(π/d)²`, 44 ft vs 31 ft
is a **2× difference in slalom curvature**, and slaloms are a large share of
autocross time. **A higher-resolution map export is the blocker on a
trustworthy autocross lap time.**

### `run_lap_targets.m` — whole-lap targets (§7)
`out = run_lap_targets()`. Accel (rev-limited), skidpad via the solver,
lap times per available track, top speed, energy per lap (T-NRG: drive energy
+ regen bound + 22 km endurance projection — currently ~9.1 kWh no-regen vs
6.27 kWh pack: regen + derating are critical path), full-lap mass sensitivity.
Saves a speed-colored `lap_map_<track>.png` per track — the visual check that
the intended course loaded.

### `run_energy_strategy.m` — endurance feasibility sweep
`out = run_energy_strategy()`. The "how do we finish endurance" table
(targets #43/#44/#65): endurance power cap × regen scenario → 22 km net
energy vs usable pack with margin, plus the lap-time cost of each cap.
Prints T-STRAT decisions (regen required; recommended VCU cap; BMS SOC
window note) and writes `plots/energy_strategy.png` (feasibility frontier).
Scenario constants (regen capture/RT, usable fraction, margin) are named
at the top — re-run after the pack load test and regen implementation.

### `run_aero_targets.m` — aero package targets
`out = run_aero_targets()`. Targets #36/#37/#38/#40: sweeps added downforce
along achievable package lines (drag and mass follow ClA), scores every
point against the REAL FSAE 2026 Michigan benchmarks
(`organization/comp_benchmarks_2026.csv`), and re-solves the endurance
power cap per point so drag pays its true energy price. Key finding: points
are monotonic in ClA to the model edge — the deliverable is a target band
(ClA 4.0, band 3.5–4.5) plus exchange rates (pts per m² and per kg) and a
device L/D floor, not an interior optimum. Prints T-CLA/T-CDA/T-XR/T-LDF/
T-BAL, writes `plots/aero_targets.png`. The hand-to-aero sheet is
`organization/aero_component_targets.xlsx`. Runtime: several minutes.

### `lap_report.m` — lap presentation figures
`lap_report('track_endurance.csv')`. Two figures into `plots/`: the lap
dashboard (speed-colored map, speed trace colored by longitudinal g, and
achieved g-g points against the envelope — the "sim works, car uses its
grip" slide) and the endurance energy budget (no-regen demand vs
regen-scenario demand vs pack usable/nominal — the regen/derating case in
one chart; scenario constants are named at the top of the file).

### `lap_replay.m` — lap animation
`lap_replay('track_<name>.csv', speedup)`. Real-time moving point mass on the
speed-colored track, with a live g-g panel: the envelope at the current speed
plus the instantaneous (ax, ay) point and its trail. Sanity-check tool: the
dot should ride the envelope boundary through corners and braking zones
(limit driving) and sit inside it only where power, not grip, is the limit.

### `bicycle_model.m` — two-axle handling model (§10)
`B = bicycle_model(p, Ca_axle_f, Ca_axle_r, v_sweep)`. The model proper:
understeer gradient K, critical/characteristic speed, yaw-rate gain
v/(L + Kv²/g), and the 2-state [β; r] yaw-plane matrix with its eigen
quantities (τ, ζ vs speed). Pure function, no printing — reused by the
future QSS bicycle lap sim and the Simulink work.

### `axle_grip.m` — mid-tier lateral limit (§11)
`out = axle_grip(p, v)`. Per-axle saturation with lateral load transfer and
tire load sensitivity — the physics the point mass cannot see: transferring
load loses total grip because mu(Fz) falls with load. Solves the limit ay at
speed v, reports the limiting axle (front-limited = stable, rear-limited =
snap), per-tire loads, and wheel lift. Knobs: `p.LLTD` (roll stiffness
split), `p.aero_df_front` (CoP). Pure function; its axle capacities become
the saturation limits of the Simulink transient model.

### `run_balance_targets.m` — chassis/aero balance driver (§11)
`out = run_balance_targets()`. Prints T-SKD2 (load-sensitive skidpad — the
mid-tier and point-mass predictions bracket the real 2026 best, validating
the fidelity ladder), T-LLTD (neutral point + recommended front-of-neutral
band), and T-BAL (#38: the aero CoP band that stays front-limited at 95%
v_max at target ClA). Writes `plots/balance_targets.png`.

### `run_handling_targets.m` — handling targets driver (§10)
`out = run_handling_targets()`. Thin driver in the run_* pattern: builds axle
stiffnesses from **`p.Ca_coef`** (the promoted artifact) at static loads ×
λ_Kyα, calls `bicycle_model`, prints T-USG/T-VCR/T-YRG/T-YAW (TR26: K ≈ −0.08
deg/g, effectively neutral — ill-conditioned, trust the sign not the decimals;
v_crit ~3.4× v_max; overdamped yaw, τ ~50–120 ms). The gain formula is also the
torque-vectoring reference model. Writes `handling_response.png`.

⚠️ This used to call `pacejka_fit()` **live at runtime**, which quietly defeated
the artifact pattern: grip was decoupled from the car but *stiffness was not*.
Editing the fit would move K with no rebuild and no staleness warning. Both now
come from `tire_coeffs.mat`, so the staleness gate covers K, yaw gain and yaw
response too.

### `vd_selftest.m` — regression harness
`vd_selftest`. **Run after every edit.** Three layers:

0. **Artifact integrity.** Re-hashes the fit code + promotion math + TTC data
   and compares against the hash stored in `tire_coeffs.mat`. If they differ,
   the car is running on grip that no longer matches the fit — it fails loudly
   and names the targets to re-issue. It also greps `vehicle_params.m` to catch
   anyone re-introducing a hard-coded `mu`.
1. **Formula wiring.** Recomputes each target's formula independently from the
   parameter file and compares against script output, so the checks survive
   parameter changes.
2. **Data anchors.** `ttc_fit`'s LC0 percentile (2.602 ± 0.03 — a check that the
   TTC *data path* hasn't moved), the curve-basis `mu_y_raw` the car actually
   runs on (2.336 ± 0.03), and the computed anisotropy (0.968 ± 0.02).

Extend it when adding a new derived quantity or script.

---

## Data

- `TTC_Data/*.mat` — FSAE TTC (Calspan). US units: lbf, psi, mph, °F; SAE
  signs (FZ < 0 under load). `*raw*` files are unprocessed — skipped by loaders.

  **Naming: `<compound>_<size>`.** The five datasets are a **compound × size
  factorial**, and the old names hid it completely — `R20` and `BigR20` were the
  *same R20 compound* in two different casings, which was impossible to tell from
  the name. Same for `LC0` and `18inLC0`.

  |  | 16×7.5-10 (8" rim) | 18.0×6.0-10 (7" rim) |
  |---|---|---|
  | **LC0** | `LC0_16x75` (Hoosier 43075) | `LC0_18x60` (Hoosier 41100) |
  | **R20** | `R20_16x75` (Hoosier 43075) | `R20_18x60` (Hoosier 43100) |

  plus `GY_18x65` (Goodyear D0571, 18.0×6.5-10, 7" rim).

  Read the grid: **`LC0_16x75` vs `R20_16x75` is a clean COMPOUND A/B** — identical
  part number 43075, identical casing, only the rubber differs. **`R20_16x75` vs
  `R20_18x60` is a clean SIZE A/B** — same R20 compound, different casing. That is
  what the naming is for; you should never again have to open a `.mat` to find out
  how two candidates differ.

  ⚠️ **Only `LC0_18x60` has longitudinal data** (testid `Drive/Brake/Combined`,
  TTC Round 6). All four *cornering candidates* are lateral-only — verified by
  force, not by the label: |FX/FZ| at the 99th percentile is **2.67** for
  `LC0_18x60` versus **0.13–0.24** for the rest, which is just rolling resistance.
  (The `SR` channel in the cornering files is a junk placeholder — median 0.9999 —
  do not trust it.) This is *why* μ_x has to be transferred cross-tire: your actual
  candidates have no drive/brake data at all. The transfer holds compound constant
  (LC0 → LC0) and varies casing, which is the best available option.

  ⚠️ **Calspan's own `tireid` disagrees on `R20_18x60`.** The field inside those
  files reads `"Hoosier 43100 18.0x6.0-10 R20, 7 inch rim"`. The raw TTC files are
  the provenance record and are **never edited**, so the disagreement stays visible
  rather than papered over. Disambiguate by part number: **43100** is the 18in R20,
  **43075** is the 16in casing (both LC0 and R20), **41100** is the 18in LC0.

- `tracks/` — course map PNGs, digitized `track_*.csv` centrelines, and
  `digitize_track.py` (map PNG → CSV; scale **auto-calibrated** from the 25 ft
  paddock grid). Endurance (964 m, closed) and autocross (674 m, open) are both
  digitized. Slalom cone spacings remain **unverified** — see the digitizer
  section above.
- `plots/` — all script-generated figures land here.
- `organization/` — target catalog + spec request spreadsheets.
- `references/` — physics reference doc + component datasheets.
- `tests/` — `vd_selftest.m` (add `tests/` to the MATLAB path, or run from
  the repo root with `addpath tests`).
- `old_python_lap_sim/` — the TR26 Python reference implementation this
  MATLAB chain was ported from; kept for cross-checking.

## Conventions

- SI inside functions; accelerations in **[g] across interfaces**
  (`gg_envelope` outputs, `load_transfer` inputs). Convert once, at the
  boundary, never twice.
- TTC work stays in US units (mu is unit-free); the design load is converted
  once (`N_PER_LBF = 4.44822`).
- Axes: x forward, y left, z up. FZ negative under load in TTC data.
- Naming: `*_raw` pre-scaling; λs are belt→track scalings (λ_μy = grip,
  λ_Kyα = stiffness — different physics, different tests, never merge them).
- **Model outputs are generated, not typed.** Anything a model produces reaches
  the car through a build step and an artifact, never through a hand-copy.

## Verification practice

1. `vd_selftest` after every change (wiring + artifact freshness).
2. Independent numeric replication for new models (this codebase's pattern:
   a Python/scipy re-implementation must reproduce MATLAB outputs before a
   model is trusted).
3. Empirical anchors from test days convert Provisional → Validated:
   skidpad → λ_μy; constant-radius steer sweep → K and λ_Kyα; coast-down →
   Crr + aero drag (driveline part already measured by the roll test);
   accel runs + pack logs → k_trac, η; endurance logs → energy model.

## Roadmap (build order, each unlocked by a consumer)

0. ~~**Percentile → curve transition**~~ **DONE, Jul 2026.** Design grip now
   reads the fitted Magic Formula median curve, not ttc_fit's 99th-percentile
   upper envelope. `mu_y_raw` 2.602 → **2.336** (−10.2%), `mu_x_raw` 2.623 →
   **2.261** (−13.8%), `mu_anisotropy` 1.008 → **0.968** — the last of these now
   *computed* by `build_tire_coeffs` rather than hand-derived. All grip-derived
   targets re-issued together. **Two consequences worth knowing:**
   - The **CG-height / track-width rollover conflict dissolved** (margin
     1.22× → 1.36×, target 1.3×). Not because the car got safer — because
     rollover margin scales as 1/μ and the car makes less grip than the
     percentile claimed. It **re-binds if λ_μy fits out above 0.70** (only
     4.7% headroom over the assumed 0.67), and λ_μy is fit at *skidpad*.
     Do not let packaging spend that margin yet.
   - The **cross-tire ranking changed**. On the curve basis the R20_18x60 leads
     (raw μ_y 2.569) and the chosen LC0 is *last* of the four (2.336) — a 10%
     gap, up from 5% on the percentile basis. Target #8 is now the
     highest-value open item.
1. **Mid-tier axle grip**: effective axle mu/Ca under lateral load transfer
   via the MF load sensitivity → real skidpad estimate, roll-stiffness
   distribution as a balance knob, track-width optimum becomes sweepable.
2. **Energy/regen strategy**: power-cap sweep vs lap time + regen model —
   closes the endurance energy gap with powertrain. **Critical path.**
3. ~~Combined slip~~ **done as a validation**: the 18in held-SA sweeps give
   envelope exponent n ≈ 1.8 (lower bound, true ~1.9–2.2) — the n=2 friction
   ellipse is retained, now data-checked rather than assumed (§8).
4. **MMM / yaw moment diagram** → trim and stability across the full g range.
5. **Transient Simulink model**: bicycle model + relaxation-length tires +
   driver model; the QSS chain stays as its validation overlay and fast
   what-if tool.
