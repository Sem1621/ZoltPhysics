#!/usr/bin/env python3
"""Instruction budget of the Phase 10 regression guard (`zig build perf-budget`).

Bit exactness cannot see a lost speedup, and wall time is too noisy for CI. So this script counts instructions:

    python3 tools/perf_budget.py <ZoltPerformanceTest> <JoltPerformanceTest> [scene ...]
    python3 tools/perf_budget.py --self-test     # check the verdicts on made-up counts

`zig build perf-budget` builds ZoltPerformanceTest with ReleaseFast for x86-64-v3 and the C++ PerformanceTest of
`zig build jolt-performance-test`, and runs this script from the repository root (the Ragdoll scene reads Assets/).
It runs short single threaded runs (-t=1) of both under valgrind's callgrind. Only the instructions (Ir) of the
physics updates count: --toggle-collect on PhysicsSystem::Update and on the scene update (UpdateTest, which moves
the characters of CharacterVirtual). The check fails when Zolt/C++ of a scene is over its budget, when a Zolt-only
scene is over its absolute budget, or when the two binaries give different hashes. It also fails when a count is
below half of its measurement: then callgrind did not find the toggle functions (for example when LLVM inlined
PhysicsSystem.update into main, or a name changed), and the count proves nothing.

Two scenes are Zolt only. Valgrind 3.22 stops the C++ binary with SIGILL at a VEX vmovq that it does not know: in
the Ragdoll scene, and in BodyInterface::AddImpulse after the first character contact of CharacterVirtual.

Measured on top of Phase 11 WS, the SIMD contact solver (d60324b; Zig 0.16.0, valgrind 3.22.0, x86-64-v3):

    Scene             Steps   Zolt Ir        C++ Ir         Zolt/C++   Budget
    ConvexVsMesh        200   1,981,091,844  2,038,825,594  0.972      1.07
    Pyramid              60   4,180,344,176  4,354,343,582  0.960      1.06
    HighSpeed            60   1,999,039,429  2,041,903,413  0.979      1.08
    CharacterVirtual     60     578,784,382  (Zolt only)               0.64e9
    Ragdoll              15   1,681,867,165  (Zolt only)               1.85e9

The budgets are the measurement plus about 10%. Before Phase 10 the ratios were 17 (ConvexVsMesh) and 5.4 (Pyramid).
After Phase 10 (W3, 4035b3c) Pyramid was 0.983; the SIMD contact solver of Phase 11 made it 0.960. SCENES keeps the
measured counts too. Re-measure and update the budgets and the counts after a Zig, LLVM or valgrind upgrade, after a
Jolt upgrade, and after a change that makes Zolt faster on purpose. A change that gives up speed on purpose updates
them in the same commit and says why. The callgrind runs take about 30 seconds on 4 cores (they run in parallel).
The budgets are for the default build options (single precision, no debug renderer).
"""
import collections
import concurrent.futures
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

Scene = collections.namedtuple("Scene", [
    "name", "quality", "steps",
    "ratio_budget",  # The budget of Zolt/C++, or None
    "abs_budget",  # The budget of Zolt's instructions in a Zolt-only scene, or None
    "zolt_ir", "jolt_ir",  # The measured counts (None: not run)
])
SCENES = [
    Scene("ConvexVsMesh", "LinearCast", 200, 1.07, None, 1_981_091_844, 2_038_825_594),
    Scene("Pyramid", "LinearCast", 60, 1.06, None, 4_180_344_176, 4_354_343_582),
    Scene("HighSpeed", "LinearCast", 60, 1.08, None, 1_999_039_429, 2_041_903_413),
    Scene("CharacterVirtual", "Discrete", 60, None, 0.64e9, 578_784_382, None),
    Scene("Ragdoll", "LinearCast", 15, None, 1.85e9, 1_681_867_165, None),
]
# A count below this part of its measurement fails: callgrind did not find the toggle functions
MIN_PART = 0.5

# The functions whose instructions count (callgrind --toggle-collect patterns, `*` is a wildcard)
TOGGLE = {
    "zolt": ["Physics.PhysicsSystem.PhysicsSystem.update", "*updateTestThunk"],
    "jolt": ["JPH::PhysicsSystem::Update(*", "*::UpdateTest(*"],
}
SIDE_NAMES = {"zolt": "Zolt", "jolt": "C++"}

COLLECTED_RE = re.compile(r"^==\d+== Collected : (\d+)$", re.M)


def run(side, binary, scene, quality, steps, out_dir):
    """Run one scene under callgrind. Returns (instructions, hash, error)."""
    cmd = ["valgrind", "--tool=callgrind", f"--callgrind-out-file={out_dir}/{side}_{scene}.out"]
    cmd += [f"--toggle-collect={t}" for t in TOGGLE[side]]
    cmd += [binary, f"-q={quality}", "-t=1", f"-s={scene}", f"-i={steps}"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    m = COLLECTED_RE.search(p.stderr)
    # The result line of the run: "<quality>, <threads>, <steps / s>, <hash>"
    h = re.search(rf"^{quality}, 1, [^,]+, (0x[0-9a-f]+)$", p.stdout, re.M)
    if p.returncode != 0 or not m or not h:
        tail = "\n".join((p.stdout + p.stderr).splitlines()[-15:])
        return None, None, f"{side} {scene} failed (exit code {p.returncode}):\n{tail}"
    return int(m.group(1)), h.group(1), None


def evaluate(scenes, results):
    """Compare the counts with the budgets. `results` maps (side, scene name) to (instructions, hash, error).
    Returns (table rows, failures)."""
    rows, failed = [], []
    for s in scenes:
        z_ir, z_hash, z_err = results[("zolt", s.name)]
        j_ir, j_hash, j_err = results.get(("jolt", s.name), (None, None, None))
        errors = [e for e in (z_err, j_err) if e]
        if errors:
            failed += errors
            rows.append(f"{s.name:<18}{s.steps:>6}  run failed")
            continue
        # A count far below its measurement means that callgrind did not find the toggle functions
        low = []
        for side, ir, measured in (("zolt", z_ir, s.zolt_ir), ("jolt", j_ir, s.jolt_ir)):
            if ir is not None and measured is not None and ir < MIN_PART * measured:
                low.append(f"{SIDE_NAMES[side]} {s.name}: callgrind counted {ir:,} instructions, less than "
                           f"{MIN_PART:.0%} of the {measured:,} measured. It probably did not find "
                           f"{' or '.join(TOGGLE[side])} (inlined or renamed? check the names with nm). If "
                           f"{SIDE_NAMES[side]} got this much faster on purpose, update the counts in SCENES.")
        if s.ratio_budget is not None:
            ratio = z_ir / j_ir if j_ir else float("inf")
            ok = ratio <= s.ratio_budget and z_hash == j_hash and not low
            result = "ok" if ok else "COUNT TOO LOW" if low else "OVER BUDGET" if z_hash == j_hash else "HASH MISMATCH"
            rows.append(f"{s.name:<18}{s.steps:>6}{z_ir:>16,}{j_ir:>16,}{ratio:>10.3f}{s.ratio_budget:>10.2f}  "
                        f"{result}")
            failed += low
            if not low and ratio > s.ratio_budget:
                failed.append(f"{s.name}: Zolt/C++ = {ratio:.3f} is over the budget {s.ratio_budget:.2f}")
            if z_hash != j_hash:
                failed.append(f"{s.name}: Zolt's hash {z_hash} is not the C++ hash {j_hash}")
        else:
            ok = z_ir <= s.abs_budget and not low
            result = "ok" if ok else "COUNT TOO LOW" if low else "OVER BUDGET"
            rows.append(f"{s.name:<18}{s.steps:>6}{z_ir:>16,}{'(Zolt only)':>16}{'':>10}{s.abs_budget:>10.3g}  "
                        f"{result}")
            failed += low
            if z_ir > s.abs_budget:
                failed.append(f"{s.name}: Zolt's {z_ir:,} instructions are over the budget {s.abs_budget:.3g}")
    return rows, failed


def self_test():
    def results(**changes):
        """The measured counts with equal hashes, with some (side, scene) entries changed"""
        r = {}
        for s in SCENES:
            r[("zolt", s.name)] = (s.zolt_ir, "0x1", None)
            if s.jolt_ir is not None:
                r[("jolt", s.name)] = (s.jolt_ir, "0x1", None)
        for key, value in changes.items():
            side, name = key.split("_", 1)
            r[(side, name)] = value
        return r

    rows, failed = evaluate(SCENES, results())
    assert failed == [] and all(row.endswith("ok") for row in rows), (rows, failed)
    # No instructions counted: the toggle function is missing. This passed before (ratio 0).
    rows, failed = evaluate(SCENES, results(zolt_Pyramid=(0, "0x1", None)))
    assert len(failed) == 1 and "Zolt Pyramid: callgrind counted 0" in failed[0], failed
    assert "Physics.PhysicsSystem.PhysicsSystem.update" in failed[0] and rows[1].endswith("COUNT TOO LOW"), rows
    rows, failed = evaluate(SCENES, results(zolt_Ragdoll=(0, "0x1", None)))
    assert len(failed) == 1 and "Zolt Ragdoll" in failed[0], failed
    # The same on the C++ side: no division by zero
    rows, failed = evaluate(SCENES, results(jolt_HighSpeed=(0, "0x1", None)))
    assert len(failed) == 1 and "C++ HighSpeed" in failed[0] and "JPH::PhysicsSystem::Update(*" in failed[0], failed
    # Over the budget, and different hashes
    rows, failed = evaluate(SCENES, results(zolt_ConvexVsMesh=(35_000_000_000, "0x1", None),
                                            zolt_CharacterVirtual=(700_000_000, "0x1", None),
                                            zolt_HighSpeed=(1_998_814_903, "0x2", None)))
    assert len(failed) == 3 and "ConvexVsMesh: Zolt/C++ = 17.167" in failed[0], failed
    assert "HighSpeed: Zolt's hash 0x2" in failed[1] and "CharacterVirtual: Zolt's 700,000,000" in failed[2], failed
    # A run that failed
    rows, failed = evaluate(SCENES, results(zolt_Pyramid=(None, None, "zolt Pyramid failed (exit code 1)")))
    assert failed == ["zolt Pyramid failed (exit code 1)"] and rows[1].endswith("run failed"), (rows, failed)
    print("perf_budget.py self-test passed")
    return 0


def main(args):
    if args == ["--self-test"]:
        return self_test()
    if len(args) < 2:
        print(__doc__)
        return 2
    zolt, jolt = os.path.abspath(args[0]), os.path.abspath(args[1])
    only = set(args[2:])
    unknown = only - {s.name for s in SCENES}
    if unknown:
        print(f"perf-budget: unknown scenes {sorted(unknown)}, the scenes are {[s.name for s in SCENES]}")
        return 2
    if shutil.which("valgrind") is None:
        print("perf-budget: valgrind is not installed (apt install valgrind)")
        return 1
    scenes = [s for s in SCENES if not only or s.name in only]
    start = time.time()
    with tempfile.TemporaryDirectory(prefix="perf-budget-") as out_dir:
        jobs = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 1) as pool:
            for s in scenes:
                jobs[("zolt", s.name)] = pool.submit(run, "zolt", zolt, s.name, s.quality, s.steps, out_dir)
                if s.ratio_budget is not None:
                    jobs[("jolt", s.name)] = pool.submit(run, "jolt", jolt, s.name, s.quality, s.steps, out_dir)
        results = {k: f.result() for k, f in jobs.items()}

    rows, failed = evaluate(scenes, results)
    print(f"{'Scene':<18}{'Steps':>6}{'Zolt Ir':>16}{'C++ Ir':>16}{'Zolt/C++':>10}{'Budget':>10}  Result")
    for row in rows:
        print(row)
    print(f"perf-budget: {len(jobs)} callgrind runs in {time.time() - start:.0f} s")
    if failed:
        print("perf-budget FAILED:")
        for f in failed:
            print("  " + f)
        print("Find the cause with callgrind_annotate, or with tools/check_codegen.py for a large fill or copy. "
              "Change a budget in tools/perf_budget.py only on purpose, and say why in the commit.")
        return 1
    print("perf-budget passed")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
