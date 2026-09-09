# TR27 Vehicle Dynamics Toolchain

MATLAB models for the Triton Racing FSAE EV: tire characterization from TTC
data, load transfer, grip and acceleration limits, a quasi-steady-state lap
simulator, and a linear handling model. The scripts in `targets/` turn these
into the numbers other subteams design against.

## Requirements

- MATLAB (R2016b or newer)
- **Optimization Toolbox, for the tire fit only.** Everything downstream of the
  artifact runs on base MATLAB. `build_tire_coeffs` does not fail without the
  toolbox - it falls back to `fminsearch`, and it produces DIFFERENT NUMBERS
  (cornering stiffness ~32% low, anisotropy 1.15 vs 1.00). If you do not have
  the toolbox, do not rebuild the artifact; use the committed one.
- The TTC tire data (licensed - not in this repo). Copy `TTC_Data/` from the
  team drive into the repo root before the first run.

## First-time setup

```matlab
vd_setup              % adds all folders to the path (run once per session)
build_tire_coeffs     % fits the tires, writes tire_coeffs_<CAR>.mat
vd_selftest           % must end ALL PASS before you trust any number
```

If `vd_selftest` fails with a "stale artifact" or "design load moved" message,
it tells you exactly what to run. Do that and re-run it.

## Layout

    vd_car.m            WHICH car is active - one line, edit to switch
    cars/config_<CAR>.m ALL car properties - THIS is the file you normally edit
    vehicle_params.m    universal constants + derived values - do NOT edit
    tire_coeffs_<CAR>.mat  generated tire data (do not hand-edit; rebuild instead)
    vd_setup.m          path setup       vd_root.m   repo-root helper
    util/       shared helpers (vd_set, vd_derive, constants)
    tire/       tire fitting and the artifact build
    models/     physics evaluators (grip, accel, braking, handling)
    lapsim/     the lap solver and its reports
    targets/    runnable target scripts (run_*)
    tests/      vd_selftest, vd_golden, ci_checks.py, refs/
    TTC_Data/   tire test data (local only)     tracks/   digitized courses
    plots/      generated figures (local only, reproduced by the scripts)

### Where does a number come from?

Ask what KIND of number it is, then look in exactly one place:

| kind | lives in |
|---|---|
| measured or decided about *this car* | `cars/config_<CAR>.m` |
| fitted from *tire data* | `tire/`, ends up in `tire_coeffs_<CAR>.mat` |
| pure arithmetic on the inputs | `util/vd_derive.m` |
| computed at *one operating point* | `models/` |
| computed over a *lap* | `lapsim/` |
| an *issued target* | `targets/run_*.m` |

## Everyday workflow

1. Edit `cars/config_<CAR>.m` (mass, geometry, aero, tire, pack). Model-fidelity
   switches live in `vehicle_params.m`. For a one-off "what if?", do not edit any
   file - use `vd_set`:  `out = run_lap_targets(vd_set(p, 'm_car', 240))`.
2. Run the script that answers your question (table below).
3. Run `vd_selftest` **and** `vd_golden` after any code change.

Rebuild `tire_coeffs_<CAR>.mat` (then re-run `vd_selftest`) only when you change
the tire-fit code, the TTC data, or the car's mass - the selftest will refuse
to run stale and will say so.

## What to run

| script | answers | writes |
|---|---|---|
| `run_load_transfer_targets` | CG ceiling, track floor, brake bias | load_transfer_targets.png |
| `run_gg_targets` | skidpad / 75 m / braking capability, g-g-V surface | gg_envelope.png, gg_surface.png |
| `run_lap_targets` | lap times per track, seconds per kg | lap figures |
| `run_handling_targets` | understeer gradient, stability speed, yaw response | handling_response.png |
| `run_stability_targets` | balance vs braking/power and weight split | stability_targets.png |
| `run_balance_targets` | load-sensitive skidpad, LLTD and CoP bands | - |
| `run_wdist_targets` | weight-distribution sweep | wdist_targets.png |
| `run_aero_targets` | downforce/drag/points trade study | aero_targets.png |
| `run_energy_strategy` | endurance power cap and energy budget | energy_strategy.png |
| `run_gear_targets` | final-drive study + freeze evidence | gear_freeze_*.png |
| `run_camber_targets` | what camber is worth: grip, balance, skidpad | camber_targets.png |
| `run_pack_targets` | accumulator sizing and the freeze evidence | pack_targets.png |
| `run_aero_gear_sensitivity` | how the gear choice moves with downforce | aero_gear_*.png |
| `aligning_moment` | steering torque and caster chart | caster_target.png, aligning_moment.png |
| `tire_report` | tire fit figures incl. stiffness/loaded radius | tire_*.png |
| `lap_report` | lap dashboard and energy budget figures | lap_dashboard_*.png |
| `params_report` | snapshot of vehicle_params to a spreadsheet | organization/vehicle_params_report.xlsx |

Every script prints a caveat line at the end of its output stating the
assumptions behind its numbers. Read it before quoting anything.

## Testing

Three layers, cheapest first. Run all three before pushing.

```matlab
vd_selftest      % is the physics still wired up correctly?  (~60 checks)
vd_golden        % did ANY number this repo produces change?
```
```bash
python3 tests/ci_checks.py    % have the docs and the code drifted apart?
```

**`vd_selftest`** checks relationships - that `mu_y` really is `mu_y_raw * derate`,
that camber at zero degrees reproduces the pre-camber model exactly, that the
tire artifact is not stale. It needs `TTC_Data/` for the staleness gate; without
it, it says so and skips that one check rather than pretending.

**`vd_golden`** is blunter: it runs every target, flattens every number into
`tests/refs/golden_<CAR>.tsv`, and tells you what moved. The file is TEXT and
sorted, so a pull request diff shows you *which* number changed and by how much.
When a change moves a number on purpose: read the list, then `vd_golden('bless')`
and commit the updated `.tsv` **in the same commit as the code**. Blessing
without reading the list makes the whole thing a rubber stamp.

**`tests/ci_checks.py`** needs no MATLAB and no tire data. It catches the failure
mode that has cost this repo the most time: two copies of the same fact drifting
apart - a target missing from the README, a caveat line that stopped being
printed, the staleness gate's input list disagreeing between its two homes.

### Continuous integration

`.github/workflows/ci.yml` runs all of the above on every push and pull request.
The repo is public, so GitHub-hosted runners and the MATLAB action's license are
both free - no license server, no self-hosted runner.

CI cannot see `TTC_Data/` (licensed, gitignored), so it cannot rebuild the tire
artifact or verify the staleness gate. It covers everything downstream of the
artifact, which is tracked. Two consequences:

- **Tire-fit changes are only verified locally.** After editing anything in
  `tire/`, run `build_tire_coeffs` and `vd_selftest` on a machine with the data
  before pushing. CI enforces the bookkeeping (if a hashed file changed, the
  artifact must have changed too) but cannot check the numbers.
- **CI installs the Optimization Toolbox** so it takes the same code path you do.

## Model switches (in vehicle_params)

    p.grip_model = 'axle'       lateral limit: 'axle' (load-sensitive, default)
                                or 'pointmass' (optimistic, comparison only)
    p.long_model = 'combined'   longitudinal: 'combined' (per-axle friction
                                ellipse, default), 'axle', or 'pointmass'

Higher tiers are slower to run and closer to reality. Quote numbers with the
tier they came from.

## Troubleshooting

- **"tire artifact is stale"** - the fit code, TTC data, or mass changed
  since the last build. Run `build_tire_coeffs`, then `vd_selftest`.
- **"Undefined function ..."** - you skipped `vd_setup` this session.
- **A g-value looks different in two places** - the tiers differ on purpose;
  check which model produced each (see Model switches).
- **Editing files outside MATLAB** - if a file changed on disk, right-click ->
  Reload in the MATLAB editor before saving, or your buffer overwrites it.

## Notes

- `plots/` and the target trackers are not version-controlled; figures are
  reproduced by the scripts, and the trackers live on the team drive.
- Detailed derivations and design notes are internal documents - ask the VD
  lead if you need them.
