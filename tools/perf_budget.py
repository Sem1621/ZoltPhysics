#!/usr/bin/env python3
"""Instruction budget of the Phase 10 regression guard (`zig build perf-budget`).

Bit exactness cannot see a lost speedup, and wall time is too noisy for CI. So this script counts instructions:

    python3 tools/perf_budget.py <ZoltPerformanceTest> <JoltPerformanceTest> [scene ...]
    python3 tools/perf_budget.py --self-test     # check the verdicts on made-up counts
    python3 tools/perf_budget.py --cache-sim [--keep DIR] [--jobs] [--steps SCENE=N ...] [--host-caches]
                                 <Zolt> <Jolt> [scene ...]

The --cache-sim mode measures instead of judging (Phase 13): callgrind with its cache and branch simulation, both
sides, the events of the collected part (Ir, cache misses, branch mispredictions) and an estimate of the cycles
(CEst = Ir + 10 (Bcm + Bim) + 10 (I1mr + D1mr + D1mw) + 100 (ILmr + DLmr + DLmw)), Zolt / C++. --jobs adds the
inclusive Ir and CEst of each PhysicsSystem job (Zolt's jobX paired with Jolt's JobX), --keep keeps the callgrind
files (with --dump-instr=yes, for callgrind_annotate per line), --steps changes the number of steps of a scene (e.g.
Ragdoll=80 or ConvexVsMesh=500, where the large islands form). It gives no verdict and exits 0 when the runs worked.
The simulated caches have a fixed geometry, CACHE_GEOMETRY (the machine of the first Phase 13 measurements: I1 32 KB,
D1 48 KB, LL 264 MB), because callgrind otherwise takes the geometry of the host, and a cloud machine can change its
host between two runs: the cache events of two hosts do not compare. --host-caches uses the host's geometry instead.

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

Measured on top of Phase 14, the SIMD LaneGroups and the character pair data (fc36075; Zig 0.16.0, valgrind 3.22.0,
x86-64-v3):

    Scene             Steps   Zolt Ir        C++ Ir         Zolt/C++   Budget
    ConvexVsMesh        200   1,980,768,617  2,038,825,594  0.972      1.07
    Pyramid              60   3,769,125,044  4,354,343,582  0.866      0.95
    HighSpeed            60   1,998,757,196  2,041,903,417  0.979      1.08
    CharacterVirtual     60     343,950,960  (Zolt only)               0.38e9
    Ragdoll              15   1,681,309,184  (Zolt only)               1.85e9

The budgets are the measurement plus about 10%. Before Phase 10 the ratios were 17 (ConvexVsMesh) and 5.4 (Pyramid).
After Phase 10 (W3, 4035b3c) Pyramid was 0.983; the SIMD contact solver of Phase 11 made it 0.960 (its contact solve
went from 1,144M to 1,027M instructions, SimulationArchitecture.md section 10), the large island sorts of Phase 12
0.953, the SIMD plans of Phase 13 0.929 (its budget went from 1.05 to 1.02), the SIMD LaneGroups of Phase 14 0.866
(budget 0.95). Phase 12's character bounds caches took CharacterVirtual from 578,784,382 to 373,906,184 (Roadmap Phase
12), Phase 14's character pair data to 343,950,960 (budget 0.41e9 to 0.38e9).
SCENES keeps the measured counts too. Re-measure and update the budgets and the counts after a Zig, LLVM or valgrind
upgrade, after a Jolt upgrade, and after a change that makes Zolt faster on purpose. A change that gives up speed on
purpose updates them in the same commit and says why. The callgrind runs take about 30 seconds on 4 cores (they run in
parallel).
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
    Scene("ConvexVsMesh", "LinearCast", 200, 1.07, None, 1_980_768_617, 2_038_825_594),
    Scene("Pyramid", "LinearCast", 60, 0.95, None, 3_769_125_044, 4_354_343_582),
    Scene("HighSpeed", "LinearCast", 60, 1.08, None, 1_998_757_196, 2_041_903_417),
    Scene("CharacterVirtual", "Discrete", 60, None, 0.38e9, 343_950_960, None),
    Scene("Ragdoll", "LinearCast", 15, None, 1.85e9, 1_681_309_184, None),
]
# A count below this part of its measurement fails: callgrind did not find the toggle functions
MIN_PART = 0.5

# The functions whose instructions count (callgrind --toggle-collect patterns, `*` is a wildcard)
TOGGLE = {
    "zolt": ["Physics.PhysicsSystem.PhysicsSystem.update", "*updateTestThunk"],
    "jolt": ["JPH::PhysicsSystem::Update(*", "*::UpdateTest(*"],
}
SIDE_NAMES = {"zolt": "Zolt", "jolt": "C++"}

COLLECTED_RE = re.compile(r"^==\d+== Collected : (\d+)(?: \d+)*$", re.M)

# The events of callgrind with --cache-sim=yes --branch-sim=yes (cycle_estimate derives KCachegrind's CEst from them)
EVENTS = ["Ir", "Dr", "Dw", "I1mr", "D1mr", "D1mw", "ILmr", "DLmr", "DLmw", "Bc", "Bcm", "Bi", "Bim"]
# The simulated caches of --cache-sim (size, associativity, line size): the 4 core Xeon of the first Phase 13
# measurements, so that cache events compare across hosts
CACHE_GEOMETRY = ["--I1=32768,8,64", "--D1=49152,12,64", "--LL=276824064,33,64"]


def cycle_estimate(e):
    """CEst of a dict of events: instructions plus 10 per branch misprediction and L1 miss, 100 per LL miss"""
    g = lambda k: e.get(k, 0)
    return (g("Ir") + 10 * (g("Bcm") + g("Bim")) + 10 * (g("I1mr") + g("D1mr") + g("D1mw")) +
            100 * (g("ILmr") + g("DLmr") + g("DLmw")))


def read_totals(path):
    """The events and their totals of a callgrind file (its "events:" and "totals:" or "summary:" lines)"""
    names, values = None, None
    with open(path) as f:
        for line in f:
            if line.startswith("events:"):
                names = line.split()[1:]
            elif line.startswith("totals:") or (line.startswith("summary:") and values is None):
                values = [int(v) for v in line.split()[1:]]
    if names is None or values is None:
        raise ValueError(f"{path}: no events or totals line")
    return dict(zip(names, values + [0] * (len(names) - len(values))))


# The jobs of PhysicsSystem: Zolt's jobX and Jolt's JobX, as addr2line names a function (not a function inside one)
JOB_RE = {
    "zolt": re.compile(r"Physics\.PhysicsSystem\.PhysicsSystem\.job([A-Z][A-Za-z]+)"),
    "jolt": re.compile(r"JPH::PhysicsSystem::Job([A-Z][A-Za-z]+)\([^()]*\)(?: const)?"),
}
NOT_IN_A_JOB = "(not in a job)"


def read_costs(path):
    """The events, the self cost per instruction and the inclusive cost of the calls per call site of a callgrind file
    written with --dump-instr=yes: (event names, {address: costs}, {address: costs})"""
    events = positions = last = None
    own, calls = {}, {}
    call_line = False
    with open(path) as f:
        for line in f:
            if not line.strip() or line[0] == "#":
                continue
            if line.startswith("events:"):
                events = line.split()[1:]
            elif line.startswith("positions:"):
                positions = line.split()[1:]
            elif line.startswith("calls="):
                # The target position of a call does not change the last position; the next cost line is the call's
                call_line = True
            elif line[0] in "0123456789+-*":
                if positions is None or positions[0] != "instr":
                    raise ValueError(f"{path}: no instruction positions (run callgrind with --dump-instr=yes)")
                parts = line.split()
                n = len(positions)
                # Subposition compression: "*" is the last value, "+N" and "-N" are relative to it
                pos = [last[i] if t == "*" else last[i] + int(t) if t[0] in "+-" else int(t, 0)
                       for i, t in enumerate(parts[:n])]
                last = pos
                cost = [int(v) for v in parts[n:]] + [0] * (len(events) - len(parts[n:]))
                d = calls if call_line else own
                old = d.get(pos[0])
                d[pos[0]] = cost if old is None else [a + b for a, b in zip(old, cost)]
                call_line = False
    return events, own, calls


def read_inline_chains(lines):
    """The functions at each address (innermost inlined function first) of `addr2line -f -i -C -a` output lines"""
    chains, address = {}, None
    i = 0
    while i < len(lines):
        if re.fullmatch(r"0x[0-9a-f]{16}", lines[i]):
            address = int(lines[i], 16)
            chains[address] = []
            i += 1
        else:
            chains[address].append(lines[i])
            i += 2  # function, then file:line
    return chains


def attribute_jobs(events, own, calls, chains, side):
    """Inclusive events per job: the self cost of its instructions (its own and the ones inlined into it, wherever it
    was inlined) plus the inclusive cost of the calls made from them. The rest of the collected part is NOT_IN_A_JOB."""
    jobs = {}

    def job_of(address):
        for name in chains.get(address, []):
            m = JOB_RE[side].fullmatch(name)
            if m:
                return m.group(1)
        return NOT_IN_A_JOB

    for costs, include in ((own, True), (calls, False)):
        for address, cost in costs.items():
            name = job_of(address)
            if name == NOT_IN_A_JOB and not include:
                continue  # A call outside the jobs: its callees' self costs are counted where they are
            acc = jobs.setdefault(name, [0] * len(events))
            for i, v in enumerate(cost):
                acc[i] += v
    # NOT_IN_A_JOB is the total minus the jobs (the calls from outside the jobs include the jobs)
    total = [sum(c[i] for c in own.values()) for i in range(len(events))]
    in_jobs = [sum(v[i] for k, v in jobs.items() if k != NOT_IN_A_JOB) for i in range(len(events))]
    jobs[NOT_IN_A_JOB] = [t - j for t, j in zip(total, in_jobs)]
    return {k: dict(zip(events, v)) for k, v in jobs.items()}


def job_costs(path, binary, side):
    """Inclusive events per job of a callgrind file written with --dump-instr=yes"""
    events, own, calls = read_costs(path)
    addresses = "\n".join(f"{a:#x}" for a in sorted(set(own) | set(calls)))
    p = subprocess.run(["addr2line", "-e", binary, "-f", "-i", "-C", "-a"], input=addresses, capture_output=True,
                       text=True, check=True)
    return attribute_jobs(events, own, calls, read_inline_chains(p.stdout.splitlines()), side)


def run(side, binary, scene, quality, steps, out_dir, extra=()):
    """Run one scene under callgrind. Returns (instructions, hash, error). `extra`: more callgrind options."""
    cmd = ["valgrind", "--tool=callgrind", f"--callgrind-out-file={out_dir}/{side}_{scene}.out", *extra]
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
    # The --cache-sim helpers: the cycle estimate, the totals of a callgrind file, the summary line of valgrind
    e = {"Ir": 1000, "Bcm": 2, "Bim": 1, "I1mr": 3, "D1mr": 4, "D1mw": 5, "ILmr": 1, "DLmr": 2, "DLmw": 3}
    assert cycle_estimate(e) == 1000 + 10 * 3 + 10 * 12 + 100 * 6, cycle_estimate(e)
    assert cycle_estimate({"Ir": 7}) == 7
    # valgrind takes a cache only if its number of sets is a power of two
    for g in CACHE_GEOMETRY:
        size, assoc, line = (int(v) for v in g.split("=")[1].split(","))
        sets = size // (assoc * line)
        assert sets * assoc * line == size and sets & (sets - 1) == 0, g
    with tempfile.TemporaryDirectory(prefix="perf-budget-test-") as d:
        path = os.path.join(d, "t.out")
        with open(path, "w") as f:
            f.write("version: 1\ncreator: callgrind-3.22.0\nevents: Ir Dr Dw I1mr D1mr D1mw ILmr DLmr DLmw Bc Bcm Bi Bim\n"
                    "fn=(1) main\n0 1 2 3\nsummary: 9 9 9\ntotals: 100 40 20 3 4 5 1 2 3 30 6 2 1\n")
        t = read_totals(path)
        assert t["Ir"] == 100 and t["Bim"] == 1 and t["DLmw"] == 3 and len(t) == len(EVENTS), t
        with open(path, "w") as f:
            f.write("events: Ir\nsummary: 42\n")
        assert read_totals(path) == {"Ir": 42}
    m = COLLECTED_RE.search("==12== Events    : Ir Dr\n==12== Collected : 4148175816 1200 3400\n")
    assert m and m.group(1) == "4148175816", m
    m = COLLECTED_RE.search("==12== Collected : 1980760533\n")
    assert m and m.group(1) == "1980760533", m
    lines = cache_report(SCENES[:1], {("zolt", "ConvexVsMesh"): {"Ir": 2000}, ("jolt", "ConvexVsMesh"): {"Ir": 1000}},
                         {("zolt", "ConvexVsMesh"): {"SolveVelocityConstraints": {"Ir": 30}},
                          ("jolt", "ConvexVsMesh"): {"SolveVelocityConstraints": {"Ir": 20}, "Old": {"Ir": 5}}})
    assert any(l.startswith("  Ir ") and l.endswith("2.000") for l in lines), lines
    assert any("SolveVelocityConstraints" in l and "1.500" in l for l in lines), lines
    assert any("Old" in l and "(C++ only)" in l for l in lines), lines
    # The jobs: names as addr2line prints them, a function inside a job is not the job
    assert JOB_RE["zolt"].fullmatch("Physics.PhysicsSystem.PhysicsSystem.jobFinalizeIslands").group(1) == \
        "FinalizeIslands"
    assert not JOB_RE["zolt"].fullmatch("Physics.PhysicsSystem.PhysicsSystem.jobFinalizeIslands.Inner.f")
    assert JOB_RE["jolt"].fullmatch("JPH::PhysicsSystem::JobFinalizeIslands(JPH::PhysicsUpdateContext*)").group(1) == \
        "FinalizeIslands"
    assert not JOB_RE["jolt"].fullmatch("JPH::PhysicsSystem::JobX(JPH::PhysicsUpdateContext*)::$_0::operator()() const")
    assert not JOB_RE["jolt"].fullmatch("std::__1::__function::__func<JPH::PhysicsSystem::Update(float, int, "
                                        "JPH::TempAllocator*, JPH::JobSystem*)::$_3, void ()>::operator()()")
    # A callgrind file with --dump-instr=yes: compressed positions, a call (its target position does not move the last
    # position), costs that end early
    with tempfile.TemporaryDirectory(prefix="perf-budget-test-") as d:
        path = os.path.join(d, "t.out")
        with open(path, "w") as f:
            f.write("positions: instr line\nevents: Ir Dr\nfn=(1) a\n0x100 10 5 1\n+4 * 3\ncalls=2 0x900 -3\n"
                    "* * 50 7\nfn=(2) b\n0x200 20 9 2\n-8 +1 4\n* * 1 1\n")
        events, own, calls = read_costs(path)
        assert events == ["Ir", "Dr"] and own == {0x100: [5, 1], 0x104: [3, 0], 0x200: [9, 2], 0x1f8: [5, 1]}, own
        assert calls == {0x104: [50, 7]}, calls
        with open(path, "w") as f:
            f.write("positions: line\nevents: Ir\n10 5\n")
        try:
            read_costs(path)
            assert False, "no instruction positions must fail"
        except ValueError:
            pass
    # addr2line output: two addresses, the first with an inlined job
    chains = read_inline_chains(["0x0000000000000104", "Physics.PhysicsSystem.PhysicsSystem.jobApplyGravity",
                                 "/z/PhysicsSystem.zig:10", "Core.JobSystem.JobSystem.JobFunction.init__anon_1.gen.invoke",
                                 "/z/JobSystem.zig:20", "0x0000000000000200", "b", "/z/b.zig:1"])
    assert chains == {0x104: ["Physics.PhysicsSystem.PhysicsSystem.jobApplyGravity",
                              "Core.JobSystem.JobSystem.JobFunction.init__anon_1.gen.invoke"], 0x200: ["b"]}, chains
    # The job gets its own instructions and its call; b (called from the job) is in the call's cost, not twice
    jobs = attribute_jobs(["Ir", "Dr"], {0x100: [5, 1], 0x104: [3, 0], 0x200: [50, 7]}, {0x104: [50, 7]},
                          {0x104: chains[0x104], 0x100: ["a"], 0x200: ["b"]}, "zolt")
    assert jobs == {"ApplyGravity": {"Ir": 53, "Dr": 7}, NOT_IN_A_JOB: {"Ir": 5, "Dr": 1}}, jobs
    print("perf_budget.py self-test passed")
    return 0


def cache_report(scenes, events, jobs):
    """The lines of the --cache-sim report. `events` maps (side, scene name) to a dict of events (or None), `jobs` maps
    (side, scene name) to the events per job (or None)."""
    lines = []
    for s in scenes:
        z, j = events.get(("zolt", s.name)), events.get(("jolt", s.name))
        lines.append(f"{s.name} ({s.steps} steps)")
        lines.append(f"  {'Event':<6}{'Zolt':>18}{'C++':>18}{'Zolt/C++':>10}")
        for name in EVENTS + ["CEst"]:
            zv = cycle_estimate(z) if name == "CEst" and z else z.get(name, 0) if z else None
            jv = cycle_estimate(j) if name == "CEst" and j else j.get(name, 0) if j else None
            ratio = f"{zv / jv:>10.3f}" if zv is not None and jv else ""
            lines.append(f"  {name:<6}{zv if zv is None else format(zv, ','):>18}"
                         f"{'' if jv is None else format(jv, ','):>18}{ratio}")
        zj, jj = jobs.get(("zolt", s.name)), jobs.get(("jolt", s.name))
        if zj is not None:
            lines.append(f"  {'Job (inclusive)':<28}{'Zolt Ir':>15}{'C++ Ir':>15}{'Ir ratio':>9}{'Zolt CEst':>15}"
                         f"{'C++ CEst':>15}{'CEst ratio':>11}")
            for name in sorted(zj, key=lambda n: -zj[n]["Ir"]):
                ze, je = zj[name], (jj or {}).get(name)
                row = f"  {name:<28}{ze['Ir']:>15,}"
                row += f"{je['Ir']:>15,}{ze['Ir'] / je['Ir']:>9.3f}" if je and je["Ir"] else f"{'':>15}{'':>9}"
                zc = cycle_estimate(ze)
                row += f"{zc:>15,}"
                if je and je["Ir"]:
                    jc = cycle_estimate(je)
                    row += f"{jc:>15,}{zc / jc:>11.3f}"
                lines.append(row)
            for name in sorted(set(jj or {}) - set(zj)):
                lines.append(f"  {name:<28}{'':>15}{jj[name]['Ir']:>15,}  (C++ only)")
    return lines


def cache_sim(zolt, jolt, scenes, keep, with_jobs, host_caches=False):
    """The --cache-sim mode: measure both sides, no verdict"""
    extra = ["--cache-sim=yes", "--branch-sim=yes"] + ([] if host_caches else CACHE_GEOMETRY)
    extra += ["--dump-instr=yes"] if keep or with_jobs else []
    start = time.time()
    out_dir = keep or tempfile.mkdtemp(prefix="perf-budget-cache-")
    os.makedirs(out_dir, exist_ok=True)
    try:
        futures = {}
        with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count() or 1) as pool:
            for s in scenes:
                futures[("zolt", s.name)] = pool.submit(run, "zolt", zolt, s.name, s.quality, s.steps, out_dir, extra)
                if s.ratio_budget is not None:
                    futures[("jolt", s.name)] = pool.submit(run, "jolt", jolt, s.name, s.quality, s.steps, out_dir,
                                                            extra)
        failed, events, jobs = [], {}, {}
        for (side, name), f in futures.items():
            ir, h, err = f.result()
            if err:
                failed.append(err)
                continue
            path = os.path.join(out_dir, f"{side}_{name}.out")
            events[(side, name)] = read_totals(path)
            if with_jobs:
                jobs[(side, name)] = job_costs(path, zolt if side == "zolt" else jolt, side)
        for s in scenes:
            z, j = futures.get(("zolt", s.name)), futures.get(("jolt", s.name))
            if z and j and not z.result()[2] and not j.result()[2] and z.result()[1] != j.result()[1]:
                failed.append(f"{s.name}: Zolt's hash {z.result()[1]} is not the C++ hash {j.result()[1]}")
        for line in cache_report(scenes, events, jobs):
            print(line)
        print(f"perf-budget --cache-sim: {len(futures)} callgrind runs in {time.time() - start:.0f} s"
              + (f", the files are in {out_dir}" if keep else ""))
        if failed:
            print("perf-budget --cache-sim FAILED:")
            for f in failed:
                print("  " + f)
            return 1
        return 0
    finally:
        if not keep:
            shutil.rmtree(out_dir, ignore_errors=True)


def main(args):
    if args == ["--self-test"]:
        return self_test()
    # The options of the --cache-sim mode
    mode_cache, keep, with_jobs, host_caches, steps, rest = False, None, False, False, {}, []
    it = iter(args)
    for a in it:
        if a == "--cache-sim":
            mode_cache = True
        elif a == "--host-caches":
            host_caches = True
        elif a == "--keep":
            keep = os.path.abspath(next(it, ""))
        elif a == "--jobs":
            with_jobs = True
        elif a == "--steps":
            scene, _, n = next(it, "").partition("=")
            if not n.isdigit():
                print("perf-budget: --steps takes SCENE=N")
                return 2
            steps[scene] = int(n)
        else:
            rest.append(a)
    if (keep or with_jobs or steps or host_caches) and not mode_cache:
        print("perf-budget: --keep, --jobs, --steps and --host-caches are options of --cache-sim")
        return 2
    args = rest
    if len(args) < 2:
        print(__doc__)
        return 2
    zolt, jolt = os.path.abspath(args[0]), os.path.abspath(args[1])
    only = set(args[2:])
    unknown = (only | set(steps)) - {s.name for s in SCENES}
    if unknown:
        print(f"perf-budget: unknown scenes {sorted(unknown)}, the scenes are {[s.name for s in SCENES]}")
        return 2
    if shutil.which("valgrind") is None:
        print("perf-budget: valgrind is not installed (apt install valgrind)")
        return 1
    scenes = [s._replace(steps=steps.get(s.name, s.steps)) for s in SCENES if not only or s.name in only]
    if mode_cache:
        return cache_sim(zolt, jolt, scenes, keep, with_jobs, host_caches)
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
