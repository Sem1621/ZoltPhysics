#!/usr/bin/env python3
"""Instruction budget of the Phase 10 regression guard (`zig build perf-budget`).

Bit exactness cannot see a lost speedup, and wall time is too noisy for CI. So this script counts instructions:

    python3 tools/perf_budget.py <ZoltPerformanceTest> <JoltPerformanceTest> [scene ...]

`zig build perf-budget` builds ZoltPerformanceTest with ReleaseFast for x86-64-v3 and the C++ PerformanceTest of
`zig build jolt-performance-test`, and runs this script from the repository root (the Ragdoll scene reads Assets/).
It runs short single threaded runs (-t=1) of both under valgrind's callgrind. Only the instructions (Ir) of the
physics updates count: --toggle-collect on PhysicsSystem::Update and on the scene update (UpdateTest, which moves
the characters of CharacterVirtual). The check fails when Zolt/C++ of a scene is over its budget, when a Zolt-only
scene is over its absolute budget, or when the two binaries give different hashes.

Two scenes are Zolt only. Valgrind 3.22 stops the C++ binary with SIGILL at a VEX vmovq that it does not know: in
the Ragdoll scene, and in BodyInterface::AddImpulse after the first character contact of CharacterVirtual.

Measured on top of Phase 10 W3 (4035b3c; Zig 0.16.0, valgrind 3.22.0, x86-64-v3):

    Scene             Steps   Zolt Ir        C++ Ir         Zolt/C++   Budget
    ConvexVsMesh        200   1,980,847,565  2,038,825,594  0.972      1.07
    Pyramid              60   4,280,522,043  4,354,343,582  0.983      1.08
    HighSpeed            60   1,998,814,903  2,041,903,413  0.979      1.08
    CharacterVirtual     60     578,779,034  (Zolt only)               0.64e9
    Ragdoll              15   1,681,813,228  (Zolt only)               1.85e9

The budgets are the measurement plus about 10%. Before Phase 10 the ratios were 17 (ConvexVsMesh) and 5.4
(Pyramid). Re-measure and update the budgets after a Zig, LLVM or valgrind upgrade, and after a change that makes
Zolt faster on purpose. A change that gives up speed on purpose updates them in the same commit and says why. The
callgrind runs take about 30 seconds on 4 cores (they run in parallel). The budgets are for the default build options
(single precision, no debug renderer).
"""
import concurrent.futures
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

# (scene, motion quality, physics steps, budget of Zolt/C++ or None, absolute budget of Zolt's Ir or None)
SCENES = [
    ("ConvexVsMesh", "LinearCast", 200, 1.07, None),
    ("Pyramid", "LinearCast", 60, 1.08, None),
    ("HighSpeed", "LinearCast", 60, 1.08, None),
    ("CharacterVirtual", "Discrete", 60, None, 0.64e9),
    ("Ragdoll", "LinearCast", 15, None, 1.85e9),
]

# The functions whose instructions count (callgrind --toggle-collect patterns, `*` is a wildcard)
TOGGLE = {
    "zolt": ["Physics.PhysicsSystem.PhysicsSystem.update", "*updateTestThunk"],
    "jolt": ["JPH::PhysicsSystem::Update(*", "*::UpdateTest(*"],
}

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


def main(args):
    if len(args) < 2:
        print(__doc__)
        return 2
    zolt, jolt = os.path.abspath(args[0]), os.path.abspath(args[1])
    only = set(args[2:])
    unknown = only - {s[0] for s in SCENES}
    if unknown:
        print(f"perf-budget: unknown scenes {sorted(unknown)}, the scenes are {[s[0] for s in SCENES]}")
        return 2
    if shutil.which("valgrind") is None:
        print("perf-budget: valgrind is not installed (apt install valgrind)")
        return 1
    scenes = [s for s in SCENES if not only or s[0] in only]
    start = time.time()
    with tempfile.TemporaryDirectory(prefix="perf-budget-") as out_dir:
        jobs = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 1) as pool:
            for scene, quality, steps, ratio_budget, _ in scenes:
                jobs[("zolt", scene)] = pool.submit(run, "zolt", zolt, scene, quality, steps, out_dir)
                if ratio_budget is not None:
                    jobs[("jolt", scene)] = pool.submit(run, "jolt", jolt, scene, quality, steps, out_dir)
        results = {k: f.result() for k, f in jobs.items()}

    failed = []
    print(f"{'Scene':<18}{'Steps':>6}{'Zolt Ir':>16}{'C++ Ir':>16}{'Zolt/C++':>10}{'Budget':>10}  Result")
    for scene, quality, steps, ratio_budget, abs_budget in scenes:
        z_ir, z_hash, z_err = results[("zolt", scene)]
        j_ir, j_hash, j_err = results.get(("jolt", scene), (None, None, None))
        for err in (z_err, j_err):
            if err:
                failed.append(err)
        if z_err or j_err:
            print(f"{scene:<18}{steps:>6}  run failed")
            continue
        if ratio_budget is not None:
            ratio = z_ir / j_ir
            ok = ratio <= ratio_budget and z_hash == j_hash
            print(f"{scene:<18}{steps:>6}{z_ir:>16,}{j_ir:>16,}{ratio:>10.3f}{ratio_budget:>10.2f}  "
                  f"{'ok' if ok else 'OVER BUDGET' if z_hash == j_hash else 'HASH MISMATCH'}")
            if ratio > ratio_budget:
                failed.append(f"{scene}: Zolt/C++ = {ratio:.3f} is over the budget {ratio_budget:.2f}")
            if z_hash != j_hash:
                failed.append(f"{scene}: Zolt's hash {z_hash} is not the C++ hash {j_hash}")
        else:
            ok = z_ir <= abs_budget
            print(f"{scene:<18}{steps:>6}{z_ir:>16,}{'(Zolt only)':>16}{'':>10}{abs_budget:>10.3g}  "
                  f"{'ok' if ok else 'OVER BUDGET'}")
            if not ok:
                failed.append(f"{scene}: Zolt's {z_ir:,} instructions are over the budget {abs_budget:.3g}")
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
