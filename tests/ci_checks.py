#!/usr/bin/env python3
"""Repo consistency checks that need no MATLAB and no tire data.

Run:  python3 tests/ci_checks.py          (from the repo root)

WHY THIS EXISTS
---------------
Most of what has gone wrong in this repo was not a wrong equation. It was two
copies of the same fact drifting apart: a doc describing a previous version of
the code, a constant with two homes, a helper duplicated with the SAME NAME and
OPPOSITE meaning in two files. None of that needs MATLAB to catch, and none of
it needs the licensed TTC data - which is why these checks run in seconds on
every push while the MATLAB job is still installing.

Each check below corresponds to a specific thing that has actually drifted.
Adding a check is cheap; do it the next time you find a duplicate by hand.

Exit code 0 = all pass, 1 = something drifted.
"""

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FAILURES = []
NOTES = []


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8", errors="replace") as f:
        return f.read()


def exists(rel):
    return os.path.exists(os.path.join(ROOT, rel))


def listdir(rel, suffix=""):
    d = os.path.join(ROOT, rel)
    if not os.path.isdir(d):
        return []
    return sorted(n for n in os.listdir(d) if n.endswith(suffix))


def fail(check, msg):
    FAILURES.append((check, msg))


def all_m_files():
    out = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames
                       if d not in (".git", "_to_delete", "plots", "TTC_Data")]
        for n in filenames:
            if n.endswith(".m"):
                out.append(os.path.relpath(os.path.join(dirpath, n), ROOT))
    return sorted(out)


# --------------------------------------------------------------------------
def check_targets_in_readme():
    """README promises a table of every target. It has been missing three."""
    readme = read("README.md")
    for n in listdir("targets", ".m"):
        stem = n[:-2]
        if not stem.startswith("run_"):
            continue
        if stem not in readme:
            fail("targets-in-README",
                 f"targets/{n} is not mentioned in README.md. Add it to the "
                 f"script table, or the next person will not know it exists.")


def check_caveat_lines():
    """README: 'Every script prints a caveat line.' Make that mechanically true."""
    for n in listdir("targets", ".m"):
        if not n.startswith("run_"):
            continue
        src = read(f"targets/{n}")
        printed = [l for l in src.splitlines()
                   if "fprintf" in l and "caveat" in l.lower()]
        if not printed:
            fail("caveat-line",
                 f"targets/{n} never prints a line containing 'Caveat'. Every "
                 f"issued number needs its warning label - that is what you say "
                 f"when someone asks how sure you are.")


def check_artifact_filename():
    """The artifact has been tire_coeffs_<CAR>.mat since the cars/ refactor.

    A bare 'tire_coeffs.mat' in an error message sends a stuck newcomer to ls
    for a file that does not exist."""
    pat = re.compile(r"tire_coeffs\.mat")
    for rel in all_m_files() + ["README.md", "references/code_pipeline.mermaid"]:
        if not exists(rel):
            continue
        for i, line in enumerate(read(rel).splitlines(), 1):
            if pat.search(line):
                fail("artifact-filename",
                     f"{rel}:{i} says 'tire_coeffs.mat'. The file is "
                     f"tire_coeffs_<CAR>.mat.")


def check_tire_src_files_agree():
    """The staleness gate's input list lives in TWO files.

    build_tire_coeffs.m and vd_selftest.m each define a local tire_src_files().
    They take 'here' with OPPOSITE meanings (tire/ vs repo root) and are kept in
    sync by a comment. If they ever disagree, the gate silently stops covering a
    file and nothing says so."""
    def names(rel):
        src = read(rel)
        m = re.search(r"function files = tire_src_files\(here\)(.*?)\nend",
                      src, re.S)
        if not m:
            fail("tire-src-files", f"{rel} has no tire_src_files() to compare.")
            return None
        return sorted(set(re.findall(r"'([A-Za-z0-9_]+\.m)'", m.group(1))))

    a = names("tire/build_tire_coeffs.m")
    b = names("tests/vd_selftest.m")
    if a is None or b is None:
        return
    if a != b:
        fail("tire-src-files",
             "tire_src_files() lists different files in build_tire_coeffs.m "
             f"({a}) and vd_selftest.m ({b}). The staleness gate is only as "
             "good as the shorter list.")


def check_schema_version_agrees():
    """vehicle_params refuses an artifact of the wrong schema. The number it
    expects and the number build_tire_coeffs stamps must match, or a fresh
    clone cannot load its own committed artifact."""
    vp = read("vehicle_params.m")
    bt = read("tire/build_tire_coeffs.m")
    a = re.search(r"SCHEMA_EXPECTED\s*=\s*(\d+)", vp)
    b = re.search(r"T\.schema_version\s*=\s*(\d+)", bt)
    if not a or not b:
        fail("schema-version", "could not find SCHEMA_EXPECTED / T.schema_version.")
        return
    if a.group(1) != b.group(1):
        fail("schema-version",
             f"vehicle_params expects schema v{a.group(1)} but "
             f"build_tire_coeffs stamps v{b.group(1)}. Every clone would refuse "
             f"the committed artifact.")


def check_function_names():
    """A MATLAB file whose function name differs from its filename is callable
    only by the filename - the mismatch is invisible until someone reads it."""
    decl = re.compile(
        r"^\s*function\s+(?:\[[^\]]*\]\s*=\s*|[A-Za-z_]\w*\s*=\s*)?"
        r"([A-Za-z_]\w*)\s*(?:\(|$)")
    for rel in all_m_files():
        stem = os.path.basename(rel)[:-2]
        for line in read(rel).splitlines():
            m = decl.match(line)
            if m:
                if m.group(1) != stem:
                    fail("function-name",
                         f"{rel} declares function '{m.group(1)}'.")
                break


def check_python_requirements():
    """README says MATLAB is the only requirement. digitize_track.py disagrees."""
    pys = [n for n in listdir("tracks", ".py")]
    if pys and not exists("tracks/requirements.txt"):
        fail("python-requirements",
             f"tracks/ has Python ({', '.join(pys)}) but no requirements.txt. "
             f"A fresh clone cannot re-run the track digitiser.")


def check_golden_covers_targets():
    """Coverage rots quietly: a new target gets written and nothing tests it.

    vd_golden lists the targets it runs. Every run_* must be in that list or in
    the explicit exclusion note beside it."""
    if not exists("tests/vd_golden.m"):
        NOTES.append("tests/vd_golden.m absent - skipping coverage check.")
        return
    src = read("tests/vd_golden.m")
    m = re.search(r"TARGETS = \{(.*?)\};", src, re.S)
    if not m:
        fail("golden-coverage", "vd_golden.m has no TARGETS list to check.")
        return
    listed = set(re.findall(r"'([A-Za-z0-9_]+)'", m.group(1)))
    excluded = set(re.findall(r"%\s*.*?\b(aligning_moment|params_report)\b", src))
    for n in listdir("targets", ".m"):
        stem = n[:-2]
        if not stem.startswith("run_"):
            continue
        if stem not in listed and stem not in excluded:
            fail("golden-coverage",
                 f"targets/{n} is not in vd_golden's TARGETS list, so no "
                 f"baseline covers it. Add it, or say in a comment why not.")


def check_readme_points_at_cars():
    """The pre-refactor README told people to edit the one file whose header
    says 'never edit this file'."""
    readme = read("README.md")
    if "cars/" not in readme or "vd_car" not in readme:
        fail("readme-cars",
             "README.md does not mention both 'cars/' and 'vd_car'. A newcomer "
             "following it will edit vehicle_params.m, find no m_car, and stop.")


# --------------------------------------------------------------------------
CHECKS = [
    ("targets listed in README", check_targets_in_readme),
    ("every target prints a caveat", check_caveat_lines),
    ("artifact filename is current", check_artifact_filename),
    ("staleness gate input lists agree", check_tire_src_files_agree),
    ("artifact schema versions agree", check_schema_version_agrees),
    ("function name == filename", check_function_names),
    ("python deps declared", check_python_requirements),
    ("golden baseline covers every target", check_golden_covers_targets),
    ("README points at cars/", check_readme_points_at_cars),
]


def main():
    print("=" * 62)
    print("REPO CONSISTENCY CHECKS  (no MATLAB, no TTC data)")
    print("=" * 62)
    for label, fn in CHECKS:
        before = len(FAILURES)
        fn()
        n = len(FAILURES) - before
        print(f"  {'FAIL' if n else 'PASS'}  {label}" + (f"  ({n})" if n else ""))
    for note in NOTES:
        print(f"  NOTE  {note}")
    if FAILURES:
        print("\n" + "-" * 62)
        for check, msg in FAILURES:
            print(f"[{check}] {msg}")
        print("-" * 62)
        print(f"{len(FAILURES)} problem(s).")
        return 1
    print("\nAll consistency checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
