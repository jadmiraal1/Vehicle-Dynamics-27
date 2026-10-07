# TR27 Vehicle Dynamics Toolchain

MATLAB models for the Triton Racing FSAE EV: tire characterization from TTC
data, load transfer, grip and acceleration limits, a quasi-steady-state lap
simulator, a linear handling model, energy and cooling studies. The scripts in
`targets/` turn these into the numbers other subteams design against.

What each model is, how far it can be trusted and what is still open:
**[docs/STATUS.md](docs/STATUS.md)**. How to work in the repo:
**[CONTRIBUTING.md](CONTRIBUTING.md)**.

## Requirements

- **MATLAB R2019b or newer.**
- **Optimization Toolbox, for the tire fit only.** Everything downstream of the
  tire artifact runs on base MATLAB. `build_tire_coeffs` refuses to run without
  the toolbox: its `fminsearch` fallback gives a different tire (cornering
  stiffness ~30% low). Without the toolbox, use the committed artifact.
- **TTC tire data** (licensed to the team - never commit or share it). Copy
  `TTC_Data/` from the team drive into the repo root. Only the tire fit,
  `tire_report` and `aligning_moment` need it.
- Python 3 for the CI checks (`tests/*.py`, standard library only) and, if you
  re-digitize a track, the packages in `tracks/requirements.txt`.

## First-time setup

```matlab
vd_setup              % adds all folders to the path (once per session)
vd_selftest           % must end ALL PASS before you trust any number
```

If `vd_selftest` reports a stale tire artifact or a moved design load, it
names the fix - usually `build_tire_coeffs` (needs `TTC_Data/` and the
Optimization Toolbox), then `vd_selftest` again.

## Layout

    vd_car.m               which car is active - one line, edit to switch
    cars/config_<CAR>.m    every property of one car - the file you normally edit
    vehicle_params.m       builds the params struct p (constants, model switches)
    tire_coeffs_<CAR>.mat  generated tire model (never hand-edit; rebuild it)
    util/        shared helpers: vd_set, vd_derive, vd_const, fsae_rules, scoring,
                 vd_row / vd_warn (printed output)
    tire/        tire fitting and the artifact build (needs TTC_Data/)
    models/      physics at one operating point (grip, accel, braking, handling)
    lapsim/      lap solver, 75 m integrator, duty cycle, lap reports
    targets/     runnable studies (run_*) that issue targets
    tests/       vd_selftest, vd_golden, ci_checks.py, ci_staleness.py, refs/
    tracks/      digitized courses (CSV) and the digitizer
    docs/        model status
    references/  pipeline diagram (tracked); datasheets and internal docs (local only)
    TTC_Data/    tire test data (local only)    plots/  generated figures (local only)

### Where does a number come from?

| kind | lives in |
|---|---|
| measured or decided about *this car* | `cars/config_<CAR>.m` |
| fixed by the *competition rules* | `util/fsae_rules.m` |
| fitted from *tire data* | `tire/`, ends up in `tire_coeffs_<CAR>.mat` |
| pure arithmetic on the inputs | `util/vd_derive.m` |
| computed at *one operating point* | `models/` |
| computed over a *lap* | `lapsim/` |
| a *study result* (target) | `targets/run_*.m` |

## Everyday workflow

1. Edit `cars/config_<CAR>.m` (mass, geometry, aero, tire, pack). For a one-off
   "what if?", edit nothing and use `vd_set`:
   `out = run_lap_targets(vd_set(vehicle_params(), 'm_car', 240))`.
2. Run the script that answers your question (table below).
3. After any code change run `vd_selftest`, `vd_golden` and
   `python3 tests/ci_checks.py`.

Rebuild `tire_coeffs_<CAR>.mat` when the tire-fit code, the TTC data or the
car's mass changes; `vd_selftest` refuses to run on a stale artifact.

## What to run

| script | answers | writes |
|---|---|---|
| `run_load_transfer_targets` | CG height ceiling, track floor, ideal brake bias | load_transfer_targets.png |
| `run_gg_targets` | point-mass skidpad, 75 m, braking; g-g-V surface | gg_envelope.png, gg_surface.png |
| `run_lap_targets` | 75 m, skidpad, top speed, lap times, endurance energy, s/kg | lap_map_*.png |
| `run_handling_targets` | understeer gradient, critical/characteristic speed, yaw response | handling_response.png |
| `run_stability_targets` | understeer gradient vs braking/power and weight split | stability_targets.png |
| `run_balance_targets` | load-sensitive skidpad, LLTD band, aero CoP band | balance_targets.png |
| `run_wdist_targets` | weight-distribution sweep | wdist_targets.png |
| `run_aero_targets` | downforce/drag/L-D targets in competition points | aero_targets.png |
| `run_energy_strategy` | endurance power cap vs energy, pack current | energy_strategy.png |
| `run_gear_targets` | final-drive ratio study | gear_freeze_*.png |
| `run_aero_gear_sensitivity` | how the aero package moves the best final drive | gear_aero_sensitivity.png |
| `run_pack_targets` | accumulator series count | pack_targets.png |
| `run_camber_targets` | what camber is worth: grip, balance, skidpad | camber_targets.png |
| `run_cooling_targets` | powertrain heat, pack current, radiator and fan requirement | cooling_*.png |
| `aligning_moment` | steering torque and caster chart (needs TTC_Data) | caster_target.png, aligning_moment.png |
| `tire_report` | tire-fit figures (needs TTC_Data) | tire_*.png |
| `lap_report` | lap dashboard and energy budget | lap_dashboard_*.png, energy_budget.png |
| `lap_replay` | animated lap | - |
| `params_report` | every parameter with source and status | organization/vehicle_params_report_<CAR>.xlsx |

Each `run_*` script prints its results as one line per number (what it is, the
value with units, and OK / TOO HIGH / ... where there is a requirement), then an
**Assumes:** line stating what the numbers rest on. Read it before quoting a
number. A tire warning (for example, loads beyond the tire data) prints once per
run, above the results.

## Testing

Three layers, cheapest first. CI runs all of them on every push.

```matlab
vd_selftest      % is the physics wired up correctly?  (~65 checks)
vd_golden        % did any number this repo produces change?
```
```bash
python3 tests/ci_checks.py      % have docs, constants and code drifted apart?
python3 tests/ci_staleness.py   % was the tire artifact rebuilt when its sources changed?
```

- **`vd_selftest`** checks relationships: that each formula recomputed by hand
  matches the code, that zero camber reproduces the no-camber model exactly, and
  that the tire artifact matches its inputs (skipped, and said so, without
  `TTC_Data/`).
- **`vd_golden`** runs every target, flattens every number into
  `tests/refs/golden_<CAR>.tsv` and reports what moved. When a change moves a
  number on purpose, read the list, run `vd_golden('bless')` and commit the
  `.tsv` with the code.
- **`tests/ci_checks.py`** needs no MATLAB: every target in this README, every
  target states its assumptions, constants have one home, and similar.

CI has no `TTC_Data/`, so it cannot rebuild or verify the tire fit. After
editing anything in `tire/`, run `build_tire_coeffs` and `vd_selftest` on a
machine with the data before pushing.

## Model switches

Set in `vehicle_params.m`; override per study with `vd_set`.

    p.grip_model = 'axle'       lateral limit: 'axle' (load transfer + load-sensitive
                                tire, default) or 'pointmass' (constant mu, optimistic)
    p.long_model = 'combined'   longitudinal: 'combined' (per-axle friction ellipse,
                                default), 'axle' or 'pointmass'
    p.tire_hiload = 'central'   tire grip above the tested load range: 'central'
                                or 'low' (pessimistic bracket)

Quote numbers with the model they came from.

## Troubleshooting

- **"tire artifact is stale"** - the fit code, the TTC data or the car mass
  changed since the last build. Run `build_tire_coeffs`, then `vd_selftest`.
- **"Undefined function ..."** - run `vd_setup` first in each MATLAB session.
- **A g value differs between two scripts** - they use different model tiers on
  purpose (see Model switches and each script's Assumes line).
- **Editing files outside MATLAB** - reload the file in the MATLAB editor before
  saving, or the editor's copy overwrites your change.

## Notes

- `plots/`, the team spreadsheets and the datasheets in `references/`
  are not version-controlled; figures are reproduced by the scripts.
- Code headers refer to sections of `VD_physics_reference.md` (derivations and
  design notes), kept on the team drive.
