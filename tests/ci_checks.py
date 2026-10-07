#!/usr/bin/env python3
"""Repo consistency checks that need no MATLAB and no tire data.

Run:  python3 tests/ci_checks.py          (from the repo root)

Each check guards against two copies of the same fact drifting apart: docs
vs code, a constant with two homes, a helper defined twice. They run in
seconds on every push. Add a check whenever you find a duplicate by hand.

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
    """Every targets/run_*.m appears in the README script table."""
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
    """Every run_* target prints an 'Assumes:' line."""
    for n in listdir("targets", ".m"):
        if not n.startswith("run_"):
            continue
        src = read(f"targets/{n}")
        printed = [l for l in src.splitlines()
                   if "fprintf" in l and "Assumes:" in l]
        if not printed:
            fail("assumes-line",
                 f"targets/{n} never prints a line starting 'Assumes:'. Every "
                 f"result needs a statement of what it assumes.")


def check_no_tracker_ids():
    """No internal tracker references (T-CGH, #41, ...) in code or docs."""
    t_code = re.compile(r"\bT-[A-Z]{2,}[0-9]*\b")
    hash_n = re.compile(r"(?<![\w&])#[0-9]+\b")
    files = all_m_files() + ["README.md", "CONTRIBUTING.md", "docs/STATUS.md",
                             "tests/refs/README.md", "references/code_pipeline.mermaid"]
    for rel in files:
        if not exists(rel):
            continue
        for i, line in enumerate(read(rel).splitlines(), 1):
            m = t_code.search(line) or hash_n.search(line)
            if m:
                fail("tracker-id",
                     f"{rel}:{i} refers to '{m.group(0)}', an internal tracker item. "
                     f"Describe the quantity in words instead.")


def check_artifact_filename():
    """The artifact is tire_coeffs_<CAR>.mat; no file may name 'tire_coeffs.mat'."""
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
    """The staleness check's input list is defined once, in tire/tire_src_files.m."""
    defs = []
    for rel in all_m_files():
        for i, line in enumerate(read(rel).splitlines(), 1):
            if re.match(r"\s*function\s+.*\btire_src_files\s*\(", line):
                defs.append(f"{rel}:{i}")
    if defs != ["tire/tire_src_files.m:1"]:
        fail("tire-src-files",
             "tire_src_files() must be defined exactly once, in "
             f"tire/tire_src_files.m. Found: {defs or 'nowhere'}.")


def check_no_bare_lbf_constant():
    """The lbf conversion lives only in util/vd_const.m (hot loops use p.N_PER_LBF)."""
    for rel in all_m_files():
        if rel.endswith("vd_const.m"):
            continue
        for i, line in enumerate(read(rel).splitlines(), 1):
            if "4.44822" in line:
                fail("lbf-constant",
                     f"{rel}:{i} writes 4.44822 out by hand. Use vd_const(), "
                     f"or p.N_PER_LBF if it is in a hot loop.")


def check_one_accel_integrator():
    """One 75 m integrator: lapsim/accel_time.m."""
    hits = []
    for rel in all_m_files():
        if rel == "lapsim/accel_time.m":
            continue
        for i, line in enumerate(read(rel).splitlines(), 1):
            if re.match(r"\s*function\s+.*\b(accel_time|accel_event)\s*\(", line):
                hits.append(f"{rel}:{i}")
    if hits:
        fail("accel-integrator",
             "the 75 m accel integrator lives in lapsim/accel_time.m only. "
             f"Found another definition at: {', '.join(hits)}.")


def check_schema_version_agrees():
    """The schema version vehicle_params expects equals the one build_tire_coeffs writes."""
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
    """Each MATLAB file's function name matches its file name."""
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
    """Python code in tracks/ declares its dependencies."""
    pys = [n for n in listdir("tracks", ".py")]
    if pys and not exists("tracks/requirements.txt"):
        fail("python-requirements",
             f"tracks/ has Python ({', '.join(pys)}) but no requirements.txt. "
             f"A fresh clone cannot re-run the track digitiser.")


def check_golden_covers_targets():
    """Every run_* target is in vd_golden's TARGETS list or its exclusion note."""
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
    """The README points newcomers at cars/ and vd_car, not vehicle_params.m."""
    readme = read("README.md")
    if "cars/" not in readme or "vd_car" not in readme:
        fail("readme-cars",
             "README.md does not mention both 'cars/' and 'vd_car'. A newcomer "
             "following it will edit vehicle_params.m, find no m_car, and stop.")


def check_no_bare_rule_constants():
    """Competition constants live only in util/fsae_rules.m."""
    pats = [r"(?<![\d.])9\.125(?![\d])", r"(?<![\w.])80e3(?![\w])"]
    for rel in all_m_files():
        if rel.endswith("fsae_rules.m"):
            continue
        for i, line in enumerate(read(rel).splitlines(), 1):
            code = line.split("%", 1)[0]
            for pat in pats:
                if re.search(pat, code):
                    fail("rule-constant",
                         f"{rel}:{i} writes a competition constant by hand. "
                         f"Use fsae_rules().")


def check_staleness_list_readable():
    """ci_staleness.py can read the hashed-source list (else that check is off)."""
    path = os.path.join(ROOT, "tire", "tire_src_files.m")
    if not os.path.exists(path):
        fail("staleness-list", "tire/tire_src_files.m is missing.")
        return
    names = re.findall(r"fullfile\(root,\s*'tire',\s*'([A-Za-z0-9_]+\.m)'\)", read("tire/tire_src_files.m"))
    if not names:
        fail("staleness-list",
             "tests/ci_staleness.py cannot find the tire/*.m entries in "
             "tire/tire_src_files.m, so the CI staleness check would be off. "
             "Keep the fullfile(root, 'tire', 'name.m') form or update both files.")


# --------------------------------------------------------------------------
CHECKS = [
    ("targets listed in README", check_targets_in_readme),
    ("every target states its assumptions", check_caveat_lines),
    ("no internal tracker references", check_no_tracker_ids),
    ("artifact filename is current", check_artifact_filename),
    ("staleness gate has one input list", check_tire_src_files_agree),
    ("lbf conversion has one home", check_no_bare_lbf_constant),
    ("one 75 m accel integrator", check_one_accel_integrator),
    ("rule constants have one home", check_no_bare_rule_constants),
    ("staleness list is readable", check_staleness_list_readable),
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
