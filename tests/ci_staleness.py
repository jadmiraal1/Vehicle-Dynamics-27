#!/usr/bin/env python3
"""Tire-artifact bookkeeping that CI can check without the tire data.

Run:  python3 tests/ci_staleness.py [base-ref]

vd_selftest's hash check is the real protection: it hashes the fit code and
the TTC files and refuses a stale artifact. CI has no TTC data (licensed,
gitignored), so it checks only the bookkeeping: if a file the artifact is
built from changed, the artifact must have been rebuilt in the same change.
It cannot see a change to the data itself and verifies no numbers.

Exit code 0 = fine, 1 = a hashed source changed without a rebuild,
2 = the hashed-source list could not be read.
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
    """The code files tire/tire_src_files.m feeds to vd_hash (TTC files excluded).

    Returns None if the list cannot be read, so a refactor of that file fails
    loudly instead of silently turning this check off."""
    path = os.path.join(ROOT, "tire", "tire_src_files.m")
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        src = f.read()
    code = "\n".join(l.split("%", 1)[0] for l in src.splitlines())   # drop comments
    names = re.findall(r"fullfile\(root,\s*'tire',\s*'([A-Za-z0-9_]+\.m)'\)", code)
    return ["tire/" + n for n in names] or None


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
    if sources is None:
        print("  FAIL  could not read the hashed-source list from tire/tire_src_files.m")
        return 2
    print(f"  hashed sources: {', '.join(sources)}")
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
        print("  The tire artifact is generated from those files. Out of step, every")
        print("  clone runs on grip that no longer matches the fit, and only a")
        print("  machine with the data would notice.")
        print()
        print("  On a machine with TTC_Data/:")
        print("      build_tire_coeffs")
        print("      vd_selftest")
        print("      vd_golden            % read what moved, then bless it")
        print(f"      git add {artifact} tests/refs/")
        return 1

    print()
    print("  NOTE  this checks only that the artifact was rebuilt; tire numbers and")
    print("        TTC data changes are checked by vd_selftest, run locally.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
