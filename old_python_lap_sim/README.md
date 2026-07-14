# TR26 FSAE EV Lap Simulator

Point-mass quasi-steady-state lap sim for Triton Racing TR26. Validated Hoosier
16x7.5-10 LC0 tire model, real FSAE Michigan 2026 track digitization, slalom
injection, endurance energy/strategy, and a competition points model — all in
one file, `tr26_sim.py`.

## Setup

```bash
pip install numpy scipy matplotlib scikit-image pillow
```

Folder layout (the script makes `output/` automatically):

```
tr26_sim.py
maps/
  endurance.png      # the red-line endurance course map
  autocross.png      # the red-line autocross course map
output/              # CSVs, figures, GIF land here
```

## Usage

```bash
python tr26_sim.py tracks      # digitize maps -> output/track_*.csv + tracks.png
python tr26_sim.py events      # accel / skidpad / autocross / endurance times
python tr26_sim.py strategy    # power-derate x regen energy sweep vs pack
python tr26_sim.py points      # full points model + design sensitivities
python tr26_sim.py animate     # animated endurance lap replay GIF
python tr26_sim.py all         # tracks -> events -> strategy -> points
```

Run `tracks` once first — the other commands read the CSVs it writes.

### Overriding placeholders (no file editing needed)

```bash
python tr26_sim.py points --mass-lb 650 --pack-kwh 5.5 --power-kw 80 --tire-F 0.66
python tr26_sim.py strategy --mass-lb 550 --pack-kwh 7.5 --no-regen
```

| Flag | Meaning | Default |
|------|---------|---------|
| `--mass-lb` | total mass with driver [lb] | 650 |
| `--pack-kwh` | usable accumulator energy [kWh] | 5.5 |
| `--power-kw` | sprint power cap [kW] | 80 |
| `--tire-F` | track grip factor (calibrate from skidpad) | 0.66 |
| `--no-regen` | disable regen | regen on |

## The numbers you must replace

Everything marked `PLACEHOLDER` in the `Vehicle` dataclass needs a measured
value before predictions are competition-grade. In priority order:

1. **mass** — get the real curb + driver weight (swings every result).
2. **pack usable kWh** — controls the endurance DNF cliff; the single most
   important number for EV strategy.
3. **regen capability** (`regen_frac`, `regen_eff`, `regen_pcap`) — confirm with
   powertrain whether regen is implemented and its rear-axle limit.
4. **rev_limit_rpm** — confirm whether the ~67 mph ceiling is a software limit.
5. **wheelbase / cg_h / rear_wt_frac** — affects launch traction only.

## Calibration (do once, after first test day)

The whole sim is only as honest as `tire_F` and the CFD `aero_corr` factor.

1. Run skidpad on TR26. Set `--tire-F` until simulated skidpad time matches the
   measured time. (Grip dominates everything — this is the highest-leverage
   calibration.)
2. Sanity-check the accel time against a real run.
3. If you can, set up a small cone course, photograph/GPS it, digitize, and
   confirm predicted vs actual lap time end to end.

After calibration, every Michigan prediction carries a defensible error bar.

## Known limitations (state these in design review)

- **Point mass**: no handling balance, no per-wheel transfer in corners. Upgrade
  to a bicycle model only if vehicle dynamics needs balance answers.
- **FX tire fit unverified**: no 16" drive/brake data existed in the TTC set;
  longitudinal grip (braking, traction-limited launch) is plausible but
  uncalibrated.
- **Track scale** from a 50 ft paddock grid at image resolution; smoothing was
  anchored to the known ~1 km lap length.
- **Racing line** follows the centerline (QSS assumption); a real driver uses
  track width. Affects all configs ~equally, so design *decisions* are robust.
- **Efficiency score** normalization is approximated (true formula depends on
  the full competitor field).
- **Reference competitor** in the points model is an assumption — tune it in
  `reference_competitor()`. Relative scores and sensitivities are robust to it
  because both cars run through the same model.

## Tire model provenance

`_lc0_fy_lb` / `_lc0_fx_lb` are a teammate's Pacejka-style fit to Calspan TTC
data for the Hoosier 16x7.5-10 LC0. The lateral (FY) fit was validated at
R²=0.985 vs the raw belt data. `F` is the belt-to-track grip derate. FZ must be
**negative** (SAE convention).
