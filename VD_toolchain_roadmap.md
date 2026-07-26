# VD Concept-Tier Toolchain Roadmap — from tools to a sweep campaign (TR26)

**Purpose.** Move the VD toolchain from "targets issued one script at a time" to a
place where we can run *trustworthy* parameter sweeps across every design variable and
read the decisions off the curves. The plan is deliberately staged: finish and validate
the tools to a defined fidelity, then run one systematic sweep campaign — not the other
way around. Sweeping on half-built or uncalibrated tools produces sharp curves centred on
the wrong point.

This doc is the plan of record. It is a living document: update the inventory and the
gap status as tools land. Companion references: `VD_physics_reference.md` (the physics),
`README.md` (how to run), `organization/VD_target_catalog.xlsx` (the target tracker).

---

## 0. Two governing principles

**1. Fidelity is per-decision, not maximal — define "done enough."** Each decision needs
a *minimum* tool fidelity, above which extra modelling buys nothing for that call. The QSS
lap sim plus the linear bicycle model is enough for almost everything; the big nonlinear
transient sim is reserved for the handful of decisions that actually need it (weight
distribution / LLTD stability limit). Section 2 sets the bar per decision so we know when
to stop building and start sweeping.

**2. Precision is not accuracy.** A sweep is internally precise but only as *accurate* as
the constants feeding it. Ten of the inputs are still flagged PROVISIONAL — `mu_derate`
(0.67), `lambda_Ca` (0.90), `LLTD` (0.60), `aero_df_front` (0.40), `k_trac` (0.90),
`eta_dt` (0.88), `R_pack`, `DI`, and the two driveline-drag terms. Sweep on those unpinned
and every curve is confidently wrong. "Developing the tools" therefore includes
*calibrating the inputs against data* (Section 4), not just adding model fidelity.

---

## 1. Where the toolchain stands today

| Layer | Tool(s) | Fidelity tier | Status |
|---|---|---|---|
| Single source of truth | `vehicle_params.m` | inputs / loaded / derived | done |
| Tire fit | `pacejka_fit`, `ttc_fit`, `build_tire_coeffs`, `mu_of_load`, `tire_coeffs.mat` | Magic-Formula, curve-based, load-extrapolated | mature |
| Static load transfer | `load_transfer`, `run_load_transfer_targets` | quasi-static | done |
| Point-mass envelope | `gg_envelope` | constant-μ g-g-V | done |
| Mid-tier lateral | `axle_grip` | per-axle, load-sensitive, LLTD | done |
| Lap sim | `ay_limit`, `corner_speed`, `lap_sim`, `load_track` | QSS; **lateral** load-sensitive, **longitudinal** still point-mass | **partial — gap #1** |
| Handling / stability | `bicycle_model`, `run_handling_targets` | linear single-track (K, yaw gain, transient mode) | built, **not swept / not integrated — gap #2** |
| Steering | `aligning_moment` | tire self-aligning moment (T-MZ, T-CAS) | done (needs steering ratio from subteam to close effort) |
| Balance / weight dist | `run_wdist_targets`, `run_balance_targets` | steady-state | done (steady-state only) |
| Targets suite | `run_{aero,energy,gg,lap,load_transfer,handling,wdist}_targets` | mixed | done |
| Integrity | `vd_selftest`, `vd_hash` | staleness + formula-wiring regression | done |

The chain is well-developed on the *force/grip* side and on *steady-state* handling. The
two structural gaps are (a) the lap sim's longitudinal edges are still point-mass, and
(b) there is no swept, integrated *stability* output — which is exactly the metric the
weight-distribution and LLTD decisions are waiting on.

---

## 2. Fidelity bar per decision (what "done" means, and do we have it)

| Decision (target #) | Minimum fidelity needed | Available now? | Stop criterion |
|---|---|---|---|
| Total mass (#1) | QSS lap s/kg, any grip model | Yes (more honest post-upgrade) | s/kg stable to model changes |
| Weight distribution (#2) | Steady-state balance **+ linear stability** (static margin, K, yaw damping) | Steady-state yes; **stability = gap #2** | stability-vs-χ available & calibrated |
| CG height (#3) | Load transfer + rollover + lap sensitivity | Yes | — |
| LLTD (#12/#66) | axle_grip limit-balance **+ yaw damping** | Partial (needs gap #2) | gap #2 |
| Aero ClA (#36) | QSS lap (downforce vs lap time) | Yes | — |
| Aero CdA (#37) | QSS lap + energy model | Yes | — |
| Aero balance / CoP (#38) | Steady-state balance + stability | Partial (needs gap #2) | gap #2 |
| Gear ratio (#41) | Lap sim **longitudinal** (accel vs top speed) | Needs **gap #1** for trust | longitudinal load-sensitive done |
| Motor / power cap (#42/#43) | Lap sim + energy (traction zones) | Needs **gap #1** | gap #1 |
| Brake bias (#48-49) | Per-axle load-sensitive braking, ideal bias | Yes (`brake_axle_limit`) | — |
| Tire choice (#8) | Tire model + lap compare | Yes | — |
| Caster / steering (#20/#54) | `aligning_moment` transfer | Yes (effort needs subteam ratio) | ratio available |

Reading: most decisions are already at or near their required fidelity. The cluster that
is genuinely blocked — weight distribution, LLTD, aero balance — is blocked on the *same*
missing capability (swept linear stability), which is why gap #2 is high-leverage.

---

## 3. Tool gaps to close, in order

**Gap #1 — Finish the lap-sim force model (longitudinal per-axle).**
We upgraded the lateral edge to `axle_grip`; the accel/brake edges in `gg_envelope` are
still point-mass constant-μ. Make them load-sensitive: RWD traction off the rear-axle load
through `mu_of_load`, braking as the 4-tire load-sensitive ideal-bias limit (the
`brake_axle_limit` logic already exists in `run_wdist_targets`). This makes the most-used
tool fully load-sensitive, so mass / gearing / braking / power sweeps become trustworthy.
Small increment, same pattern as the lateral upgrade. *Acceptance:* new selftest checks
(longitudinal axle < point-mass at load, continuity at the data edge) + Python cross-check;
lap time shifts in the expected direction on accel-limited zones.

**Gap #2 — Make stability a computed, swept output.**
Wire the bicycle model so understeer gradient K, static margin (CG-to-neutral-steer
distance / wheelbase), critical/characteristic speed, and yaw damping ζ and response time
fall out for *any* parameter set — then expose them to the sweep harness. This is what
unlocks weight distribution, LLTD, and aero balance. Linear / sub-limit, which is the
correct concept-tier tool for *stability and response* (it will not capture nonlinear
limit snap — that is the transient sim, deferred). *Acceptance:* stability metrics vs χ and
vs LLTD reproduce hand-calc at one point; signs and trends match theory.

**Gap #3 — One sweep / DOE harness.**
Generalise the `p2 = p; p2.x = …; run chain` pattern the `run_*_targets` scripts already
use into a single engine: give it `{variable, range}` (1-D) or a pair (2-D map), it runs
the model chain per point and emits (i) a sensitivity table, (ii) tornado charts around the
baseline, and (iii) 1-D line / 2-D contour plots. Must re-derive dependent params (a, b,
Wf/Wr, k_rot) on each perturbation exactly as `run_wdist_targets` does, or the sweep lies.
*Acceptance:* reproduces the existing `run_wdist_targets` χ-sweep numbers when pointed at
`mass_dist_f`; one 2-D map (χ × LLTD) renders.

**Gap #4 — Calibrate against data (runs in parallel).**
Back-test the chain against last year's car (`organization/LastYear_Spec_Request.xlsx`),
the competition benchmarks (`organization/comp_benchmarks_2026.csv`), and the event results
(`references/fsae_2026_mi5_results.pdf`); use any skidpad / accel test numbers to pin the
PROVISIONAL constants (`mu_derate`, `LLTD`, `aero_df_front`, `k_trac`, `eta_dt`). *Acceptance:*
the model reproduces last year's measured skidpad, 75 m accel, and endurance lap within a
stated tolerance (target ≤ ~5%). Until this passes, sweep *outputs* are directional, not
absolute — flag them that way.

**Deferred — Nonlinear transient sim (roadmap #5).**
The gold standard for the rearward-bias / snap-oversteer limit. Large effort. *Trigger to
build:* only if the gap-#2 linear stability study is inconclusive for the weight-dist / LLTD
call, or track data contradicts the QSS lap sim. Do not build pre-emptively.

---

## 4. Design-variable catalogue for the campaign

Baseline = current `vehicle_params`. Ranges are provisional; tighten during calibration.

| Variable | `p` field | Baseline | Sweep range | Informs (target #) | Primary outputs |
|---|---|---|---|---|---|
| Total mass | `m` (via m_car/driver) | 333.8 kg | −40 … +20 kg | #1 | lap time, s/kg |
| Front mass frac | `mass_dist_f` | 0.40 | 0.38 – 0.52 | #2 | ay, accel, brake, K, static margin, lap |
| CG height | `h_cg` | 0.279 m | 0.24 – 0.33 m | #3 | load transfer, rollover, lap |
| LLTD | `LLTD` | 0.60 | 0.35 – 0.72 | #12/#66 | limit balance, yaw damping |
| Downforce | `ClA` | 1.301 m² | 0.9 – 1.8 m² | #36 | lap, corner speed |
| Drag | `CdA` | 0.953 m² | 0.7 – 1.2 m² | #37 | lap, energy/lap |
| Aero balance | `aero_df_front` | 0.40 | 0.30 – 0.45 | #38 | balance, high-speed stability |
| Final drive | `gear_ratio` | 3.82 | 3.0 – 4.5 | #41 | 75 m accel, top speed, lap |
| Power cap | `P_max` / deploy | 62.7 kW | 25 – 63 kW | #42/#43 | lap, energy/lap |
| Grip derate | `mu_derate` | 0.67 | 0.60 – 0.75 | calibration | everything (sensitivity of the whole model) |

**Priority 2-D maps:** χ × LLTD (the balance map — where limit balance and yaw damping
trade), ClA × aero_df_front (downforce vs where it acts), gear × power-cap (accel vs energy).

---

## 5. Sequence & milestones

1. **M1 — Lap sim complete.** Close gap #1. Lap sim fully load-sensitive; mass / gear /
   power / brake sweeps trustworthy.
2. **M2 — Stability online.** Close gap #2. K, static margin, yaw damping computable and
   swept; weight-dist / LLTD / aero-balance decisions unblocked.
3. **M3 — Sweep harness.** Close gap #3. One engine drives 1-D sweeps, 2-D maps, tornado
   sensitivity across the Section 4 catalogue.
4. **M4 — Calibrated.** Close gap #4. Provisional constants pinned; model reproduces
   last-year measured events within tolerance. Sweep outputs become *absolute*, not just
   directional.
5. **M5 — Campaign.** Run the full sweep set, produce sensitivity/tornado summaries, and
   issue final (non-provisional) targets from the curves. Because all tools share one
   tire/axle model, every output moves consistently under a given perturbation.

Guardrails carried throughout: keep `vd_selftest` green after every change (staleness +
formula wiring), validate each new model numerically before trusting it, and keep every
emitted number tagged with its fidelity tier and whether its inputs are calibrated.

---

## 6. Output-metric definitions (glossary for the sweeps)

*Lap time* — Σ ds / v_mid from the QSS trace. *s/kg* — finite-difference lap-time
sensitivity to mass, k_rot recomputed. *ay_lim* — lateral limit [g] from `axle_grip`.
*Understeer gradient K* — steer-vs-lateral-accel slope [deg/g]; sign gives under/oversteer.
*Static margin* — CG-to-neutral-steer distance as a fraction of wheelbase; the "how much
rear bias is safe" number. *Yaw damping ζ, response time τ* — transient yaw mode from the
bicycle model. *Accel / brake g* — longitudinal limits (load-sensitive after gap #1).
*Energy/lap* — longitudinal work integrated along the trace; regen as an upper bound.
