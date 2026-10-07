# Golden-value baselines

`golden_<CAR>.tsv` holds every number the toolchain produces for one car, as a
sorted, tab-separated text file. `tests/vd_golden.m` writes and compares it.
Being text and sorted, a pull-request diff shows **which** number moved and by
**how much**.

## When `vd_golden` reports drift

A failure is not a bug report: it says a number moved, which is sometimes the
point of the change.

1. `vd_golden` and read the list of what moved
2. every line should be a change you meant to make - if not, stop and explain it
3. `vd_golden('bless')`
4. commit the code **and** the `.tsv` in the same commit

Blessing without reading the list makes the baseline useless.

## Where to bless

Bless on a machine with the **Optimization Toolbox**. The tire artifact is
fitted with `lsqcurvefit`; without the toolbox the fit takes a different code
path and gives a different tire, so a baseline blessed there would not match CI.

`vd_golden` covers everything downstream of the committed tire artifact. It does
not cover anything that needs `TTC_Data/` (the fits themselves and
`aligning_moment`); `vd_selftest` checks those on a machine with the data.
