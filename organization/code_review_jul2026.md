# Code review — TR26 VD toolchain (Jul 13, 2026)

Scope: all 23 MATLAB files + the track digitizer, cross-file compatibility,
numeric verification, data/artifact integrity. Reviewed without a MATLAB
runtime, so every check that is runnable outside MATLAB was run here; the
one remaining gate is `vd_selftest` run natively (instructions at bottom).

## Summary

The codebase is internally consistent: every interface a script consumes was
verified against the file that provides it, every derived number recomputes
from its inputs, the tire artifact is fresh against its source hash, and the
MATLAB linter passes all 23 files with zero findings. Two documented-not-fixed
maintainability items below; no correctness defects found.

## Verified (passing)

- **Lint**: miss_hit `mh_lint`, 23 files, no findings.
- **Artifact staleness gate**: `tire_coeffs.mat` src_hash recomputed
  independently in Python via the documented `vd_hash` definition — match
  (fresh). Design load 184.061 lbf/corner matches params exactly.
- **Derived-parameter chain** (Python mirror): mu_y 1.5626, mu_x 1.5122,
  k_rot 1.1110, P_max 62.748 kW, v_max 24.67 m/s, Izz 146.6 kg·m²,
  brake bias 67.0% F, CG ceiling 0.297 m, light-driver rollover margin 1.36
  (binding case, ≥1.3 OK), axle_grip anchor 1.426 g @ 12 m/s. All PASS.
- **Interfaces**: `lap_sim(p,s,kappa,v0,closed)` + E-struct fields
  (`drive_acc_Wh`, `brake_wheel_Wh`) match all three consumers;
  `corner_speed(p,kappa)` handles kappa=0 and v0=0 (no div-by-zero:
  `max(v,1e-3)` in gg_envelope); `axle_grip` output fields match
  run_balance_targets and vd_selftest; benchmarks CSV parser skips `#` and
  non-numeric rows correctly.
- **Data**: both track CSVs parse, lengths 964/674 m, curvature clip
  1/4.5 m honored exactly; scenario constants (regen 0.50×0.65, usable 0.90,
  margin 0.90) identical across the three files that state them.
- **Guardrails**: no hand-typed grip in vehicle_params (artifact-loaded);
  selftest covers the new axle_grip rows.

## Findings

| # | Where | Finding | Severity | Status |
|---|-------|---------|----------|--------|
| 1 | run_energy_strategy vs run_aero_targets | Endurance basis differs: rules 22.0 km (22.8 laps) vs real-event 22 laps (21.2 km), ~4% energy delta | 🟡 Med | Documented in code (deliberate: deployment plans for rules distance; scoring uses real-event basis) |
| 2 | run_energy_strategy / lap_report / run_aero_targets | Regen/usable/margin constants duplicated in 3 files (currently identical) | 🟡 Med | Open — recommend promoting to a `p.scenario` struct in vehicle_params when regen implementation pins real values |
| 3 | lap_sim | `arrayfun(corner_speed)` = 60 gg calls/point (~58k/lap); fine at n≈1000, will matter for optimization sweeps | 🟢 Low | Note only — vectorize when it hurts |
| 4 | axle_grip | mu(Fz) clamp extends to 450 lbf, above the 350 lbf fitted ceiling (conservative falling slope, documented) | 🟢 Low | Acceptable; revisit if aero loads push corners past ~400 lbf |
| 5 | repo | TTC_Data is under the FSAE TTC license (no redistribution) and references/ holds vendor datasheets | 🔴 For GitHub | Both excluded from git; repo must be **private** |

## Known model-tier caveats (documented in code + physics reference, not defects)

Point-mass optimism (bracketed by mid-tier, §9/§11); K ill-conditioning
(trust sign); single-knob LLTD; CoP fixed with speed; QSS everywhere;
regen scenario and EF_MAX are assumptions pending implementation.

## The remaining gate (run in MATLAB)

```matlab
addpath tests; vd_selftest        % 17 wiring checks + 4 data anchors
run_balance_targets;              % expect: skidpad ~5.08 s, neutral LLTD 0.57
run_aero_targets;                 % expect: baseline ~395 pts, target ~424
```

A fresh clone WITHOUT TTC_Data/ will fail the staleness gate **by design**
(grip must match data). Copy TTC_Data from the team drive before running.

Verdict: **Approve** — pending the native vd_selftest run above.
