# Golden-value baselines

`golden_<CAR>.tsv` is every number this toolchain produces, for one car, as a
sorted tab-separated text file. `tests/vd_golden.m` writes and compares it.

**There is no baseline in this folder yet.** Generate one:

```matlab
vd_setup
vd_golden('bless')     % on a machine WITH the Optimization Toolbox
```

then commit the `.tsv`. Until you do, the golden step of CI will fail and tell
you this.

## Why text, and why sorted

So that a pull request diff shows you **which** number moved and **by how
much**. A `.mat` reference would only tell you that something changed, which is
the least useful half of the answer.

## The rule

A failure here is not a bug report - it says a number moved, and sometimes that
is the point of the change. So:

1. `vd_golden` -> read the list of what moved
2. every line should be a change you meant to make
3. `vd_golden('bless')`
4. commit the code **and** the `.tsv` in the same commit

Step 2 is the only one that matters. Blessing without reading turns this into a
rubber stamp, which is worse than having nothing - it will make you confident
about a change nobody checked.

## Generate it on a machine with the Optimization Toolbox

Without it, `fit_mf` falls back to `fminsearch` and the tire artifact comes out
different (cornering stiffness ~32% low). A baseline blessed on a machine
without the toolbox does not describe the same model as CI runs.
