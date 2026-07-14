# Triton Racing — Vehicle Dynamics

MATLAB toolchain for UCSD Triton Racing (FSAE EV). It turns tire test data,
digitized course maps, and vehicle parameters into **design targets** for the
other subteams — grip, chassis, powertrain, energy strategy, and aero numbers,
all scored against the real FSAE 2026 Michigan results. The chain builds
toward a transient lap simulation in Simulink.

## Quick start

```matlab
vd_selftest                    % run after ANY edit (addpath tests)
run_load_transfer_targets;     % CG ceiling, track floor, brake bias
run_gg_targets;                % skidpad, accel, g-g envelope
run_lap_targets;               % lap times + energy per track
run_handling_targets;          % understeer gradient, yaw response
run_balance_targets;           % LLTD + aero balance band
run_energy_strategy;           % endurance power cap / regen table
run_aero_targets;              % ClA/CdA targets vs 2026 points
```

Every script is standalone and writes its figures to `plots/`.

**Fresh clone?** `TTC_Data/` is not in the repo (FSAE TTC license — never
commit it). Copy it from the team drive first, or `vd_selftest` will fail its
staleness gate — that failure is by design: the car's grip must provably
match the tire data it came from.

## The one rule

Model outputs are **generated, never typed**. Design grip reaches the car
only through `build_tire_coeffs → tire_coeffs.mat → vehicle_params`, and
`vd_selftest` fails loudly if the artifact goes stale or someone hand-types a
mu. Run `build_tire_coeffs` deliberately, then re-issue the affected targets.

## Where things live

| Path | What |
|---|---|
| `vehicle_params.m` | Single source of truth for every number |
| `*_fit.m`, `build_tire_coeffs.m` | Tire fitting → the grip artifact |
| nouns (`gg_envelope`, `lap_sim`, `bicycle_model`, `axle_grip`, ...) | Pure models |
| `run_*.m` | Target drivers — print decisions, save figures |
| `tests/vd_selftest.m` | Regression gate: artifact freshness + formula wiring |
| `tracks/` | Course PNGs + digitizer + track CSVs |
| `organization/` | Target catalog, benchmarks, decision matrices, code review |
| `references/` | The deep docs (below) |
| `plots/` | All generated figures |

## Documentation

- **[Codebase guide](references/codebase_guide.md)** — philosophy, per-file
  reference, data provenance, conventions, roadmap. Start here to work on
  the code.
- **[Physics reference](references/VD_physics_reference.md)** — every
  equation derived, §1–§11. Start here to understand the models.
- **[Pipeline diagram](references/code_pipeline.mermaid)** — the data flow
  at a glance (renders on GitHub).
- **Targets tracker** — `organization/VD_target_catalog.xlsx`: every number
  issued to a subteam, with status (Provisional → Delivered → Validated).
