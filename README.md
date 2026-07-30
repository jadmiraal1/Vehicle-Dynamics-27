# TR27 Vehicle Dynamics Toolchain

MATLAB models for the Triton Racing FSAE EV: tire characterization from TTC
data, load transfer, grip and acceleration limits, a quasi-steady-state lap
simulator, and a linear handling model. The scripts in `targets/` turn these
into the numbers other subteams design against.

## Requirements

- MATLAB (R2016b or newer; no toolboxes required)
- The TTC tire data (licensed - not in this repo). Copy `TTC_Data/` from the
  team drive into the repo root before the first run.

## First-time setup

```matlab
vd_setup              % adds all folders to the path (run once per session)
build_tire_coeffs     % fits the tires, writes tire_coeffs.mat
vd_selftest           % must end ALL PASS before you trust any number
```

If `vd_selftest` fails with a "stale artifact" or "design load moved" message,
it tells you exactly what to run. Do that and re-run it.

## Layout

    vehicle_params.m    all car inputs - the only file you normally edit
    tire_coeffs.mat     generated tire data (do not hand-edit; rebuild instead)
    vd_setup.m          path setup       vd_root.m   repo-root helper
    tire/       tire fitting and the artifact build
    models/     physics evaluators (grip, accel, braking, handling)
    lapsim/     the lap solver and its reports
    targets/    runnable target scripts (run_*)
    tests/      vd_selftest
    TTC_Data/   tire test data (local only)     tracks/   digitized courses
    plots/      generated figures (local only, reproduced by the scripts)

## Everyday workflow

1. Edit `vehicle_params.m` (mass, geometry, aero, model switches).
2. Run the script that answers your question (table below).
3. Run `vd_selftest` after any code change.

Rebuild `tire_coeffs.mat` (then re-run `vd_selftest`) only when you change
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
| `aligning_moment` | steering torque and caster chart | caster_target.png, aligning_moment.png |
| `tire_report` | tire fit figures incl. stiffness/loaded radius | tire_*.png |
| `lap_report` | lap dashboard and energy budget figures | lap_dashboard_*.png |
| `params_report` | snapshot of vehicle_params to a spreadsheet | organization/vehicle_params_report.xlsx |

Every script prints a caveat line at the end of its output stating the
assumptions behind its numbers. Read it before quoting anything.

## Model switches (in vehicle_params)

    p.grip_model = 'axle'       lateral limit: 'axle' (load-sensitive, default)
                                or 'pointmass' (optimistic, comparison only)
    p.long_model = 'combined'   longitudinal: 'combined' (per-axle friction
                                ellipse, default), 'axle', or 'pointmass'

Higher tiers are slower to run and closer to reality. Quote numbers with the
tier they came from.

## Troubleshooting

- **"tire_coeffs.mat is stale"** - the fit code, TTC data, or mass changed
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
