#!/usr/bin/env python3
"""Staleness bookkeeping the CI can enforce WITHOUT the tire data.

Run:  python3 tests/ci_staleness.py [base-ref]

WHY THIS IS SEPARATE FROM THE REAL GATE
---------------------------------------
vd_selftest's hash gate is the real protection: it hashes the fit code AND the
TTC files and refuses to run when the artifact does not match. CI cannot do that
- TTC_Data/ is licensed and gitignored, so a hosted runner has no data and the
hash is not comparable.

What CI CAN check is the bookkeeping half: if someone changed a file that the
artifact is built from, the artifact must have been rebuilt in the same change.
That catches the common mistake ("edited the fit, forgot to rebuild") without
needing a single byte of tire data. It does NOT catch a change to the data
itself, and it does not verify any number - say so, do not oversell it.

Exit code 0 = fine, 1 = someone changed the fit without rebuilding.
"""

import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def git(*args):
    return subprocess.run(["git"] + list(args), cwd=ROOT,
                          capture_output=True, text=True)


def active_car():
    with open(os.path.join(ROOT, "vd_car.m"), encoding="utf-8") as f:
        m = re.search(r"name\s*=\s*'([A-Za-z0-9_]+)'", f.read())
    return m.group(1) if m else None


def hashed_sources():
    """The list build_tire_coeffs feeds to vd_hash, minus the TTC files."""
    path = os.path.join(ROOT, "tire", "build_tire_coeffs.m")
    with open(path, encoding="utf-8") as f:
        src = f.read()
    m = re.search(r"function files = tire_src_files\(here\)(.*?)\nend", src, re.S)
    if not m:
        return []
    return ["tire/" + n for n in re.findall(r"'([A-Za-z0-9_]+\.m)'", m.group(1))]


def pick_base(argv):
    if len(argv) > 1 and argv[1]:
        return argv[1]
    for env in ("BASE_REF", "GITHUB_BASE_REF"):
        v = os.environ.get(env)
        if v:
            return v if git("rev-parse", "--verify", v).returncode == 0 else f"origin/{v}"
    for cand in ("origin/main", "HEAD~1"):
        if git("rev-parse", "--verify", cand).returncode == 0:
            return cand
    return None


def main():
    car = active_car()
    if car is None:
        print("could not read vd_car.m - skipping"); return 0
    artifact = f"tire_coeffs_{car}.mat"

    base = pick_base(sys.argv)
    print("=" * 62)
    print(f"ARTIFACT STALENESS BOOKKEEPING  (car {car})")
    print("=" * 62)
    if base is None:
        print("  NOTE  no base ref to compare against - skipping.")
        return 0

    r = git("diff", "--name-only", f"{base}...HEAD")
    if r.returncode != 0:
        r = git("diff", "--name-only", base, "HEAD")
    if r.returncode != 0:
        print(f"  NOTE  git diff against {base} failed - skipping.")
        return 0

    changed = set(l.strip() for l in r.stdout.splitlines() if l.strip())
    print(f"  base {base}: {len(changed)} file(s) changed")

    sources = hashed_sources()
    touched = sorted(changed & set(sources))
    artifact_rebuilt = artifact in changed

    if not touched:
        print("  PASS  no hashed tire source changed")
    elif artifact_rebuilt:
        print(f"  PASS  {', '.join(touched)} changed AND {artifact} was rebuilt")
    else:
        print(f"  FAIL  changed: {', '.join(touched)}")
        print(f"        but {artifact} was NOT rebuilt in this change.")
        print()
        print("  The tire artifact is generated from those files. Ship them out of")
        print("  step and every clone runs on grip that no longer matches the fit,")
        print("  which vd_selftest will only catch on a machine that has the data.")
        print()
        print("  On a machine with TTC_Data/:")
        print("      build_tire_coeffs")
        print("      vd_selftest")
        print("      vd_golden            % read what moved, then bless it")
        print(f"      git add {artifact} tests/refs/")
        return 1

    print()
    print("  NOTE  this only checks that the artifact was rebuilt. It cannot")
    print("        verify any tire number, and it cannot see a change to the TTC")
    print("        data itself - that is vd_selftest's hash gate, run locally.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
