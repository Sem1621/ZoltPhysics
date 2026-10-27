#!/usr/bin/env python3
"""Steps per second of several PerformanceTest binaries in interleaved rounds (Roadmap Phase 13).

    python3 tools/wall_rounds.py [--rounds N] [--threads 1,2,4] [--scenes A,B] [--steps N] [--json FILE]
                                 NAME=BINARY[,ARG...] NAME=BINARY[,ARG...] ...
    python3 tools/wall_rounds.py --report FILE [--ref NAME]   # the tables of an earlier --json FILE, without running
    python3 tools/wall_rounds.py --self-test

Wall clock is noisy, so a speedup is measured as the median of several rounds, and every round runs every variant
once per scene and thread count, one after the other (the variants in a different order each round), so a slow minute
of the machine hits all variants alike. A variant is a name, a binary (ZoltPerformanceTest, or the C++ PerformanceTest
of `zig build jolt-performance-test`) and extra arguments after commas, e.g. `spin30=<dir>/bin/ZoltPerformanceTest,
-spin_us=30`. Build the Zolt binaries like the C++ reference, e.g. `zig build -Doptimize=ReleaseFast -Dcpu=x86_64_v3
-p <dir>` (a plain `zig build` installs a Debug build). The defaults: 7 rounds, 1, 2 and 4 threads, the five scenes of
Jolt's PerformanceTest (LinearCast, and Discrete for CharacterVirtual as in Jolt's hash of that scene), 500 steps (the
binaries' own default).

The first table has per scene and thread count the median steps per second of every variant, the ratio of its median
to the first variant's, the spread of its rounds ((max - min) / median) and the median CPU time (user + system) of a
run, the cost of a spin. The second table pairs the runs of a round: per variant after the first, the median of the
per-round ratios to the first variant, their quartiles, the rounds the variant won, and the p value of a two-sided sign
test (ties left out). Phase 13 judged its changes by these paired ratios: a slow minute of the machine hits both runs of
a pair alike. The script fails when a run fails or when two runs of one scene print different hashes: all variants must
compute the same bits at every thread count (class A). `--json` writes every run (with its round), and `--report` prints
the tables of such a file again (`--ref NAME`: the ratios to that variant instead of the first). Run it on a quiet
machine. It needs only the standard library.
"""

import json
import math
import os
import re
import statistics
import subprocess
import sys

SCENES = ["Ragdoll", "ConvexVsMesh", "HighSpeed", "CharacterVirtual", "Pyramid"]
QUALITY = {"CharacterVirtual": "Discrete"}  # the others: LinearCast
RESULT_RE = re.compile(r"^(Discrete|LinearCast), (\d+), ([0-9.]+), (0x[0-9a-f]+)$", re.M)


def parse_variant(text):
    """NAME=BINARY[,ARG...] -> (name, binary, [args])"""
    name, sep, rest = text.partition("=")
    if not sep or not name or not rest:
        raise ValueError(f"a variant is NAME=BINARY[,ARG...], not {text!r}")
    binary, *args = rest.split(",")
    return name, binary, args


def parse_result(output):
    """(steps per second, hash) of the result line of a run, or None"""
    m = RESULT_RE.search(output)
    return (float(m.group(3)), m.group(4)) if m else None


def run_once(binary, args, scene, threads, steps):
    """One run: (steps per second, hash, CPU seconds, error)"""
    cmd = [binary, f"-q={QUALITY.get(scene, 'LinearCast')}", f"-t={threads}", f"-s={scene}"]
    cmd += [f"-i={steps}"] if steps else []
    cmd += args
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    out = p.stdout.read()
    p.stdout.close()
    _, status, usage = os.wait4(p.pid, 0)
    p.returncode = os.waitstatus_to_exitcode(status)
    r = parse_result(out)
    if p.returncode != 0 or r is None:
        tail = "\n".join(out.splitlines()[-10:])
        return None, None, None, f"{' '.join(cmd)} failed (exit code {p.returncode}):\n{tail}"
    return r[0], r[1], usage.ru_utime + usage.ru_stime, None


def schedule(variants, scenes, threads, rounds):
    """The runs in their order: (round, scene, threads, variant index); the variants rotate from round to round"""
    order = []
    n = len(variants)
    for r in range(rounds):
        for scene in scenes:
            for t in threads:
                for i in range(n):
                    order.append((r, scene, t, (i + r) % n))
    return order


def summarize(variants, scenes, threads, runs):
    """The table lines and the failures of the runs [{variant, scene, threads, steps_per_s, hash, cpu_s, error}]"""
    failed = [r["error"] for r in runs if r["error"]]
    for scene in scenes:
        hashes = sorted({r["hash"] for r in runs if r["scene"] == scene and not r["error"]})
        if len(hashes) > 1:
            by = {}
            for r in runs:
                if r["scene"] == scene and not r["error"]:
                    by.setdefault(r["hash"], set()).add(f"{r['variant']} -t={r['threads']}")
            failed.append(f"{scene}: different hashes: " + "; ".join(f"{h} ({', '.join(sorted(by[h]))})" for h in hashes))
    names = [v[0] for v in variants]
    lines = [f"{'Scene':<17}{'Threads':>7}  " + "".join(f"{n:>32}" for n in names)]
    lines.append(f"{'':<17}{'':>7}  " + "".join(f"{'steps/s':>11}{'ratio':>7}{'spread':>7}{'cpu':>7}" for _ in names))
    for scene in scenes:
        for t in threads:
            medians = []
            cells = []
            for n in names:
                rs = [r for r in runs if r["variant"] == n and r["scene"] == scene and r["threads"] == t and not r["error"]]
                if not rs:
                    medians.append(None)
                    cells.append(f"{'-':>32}")
                    continue
                sps = [r["steps_per_s"] for r in rs]
                med = statistics.median(sps)
                medians.append(med)
                base = medians[0]
                ratio = med / base if base else float("nan")
                spread = (max(sps) - min(sps)) / med if med else float("nan")
                cpu = statistics.median([r["cpu_s"] for r in rs])
                cells.append(f"{med:>11.1f}{ratio:>7.3f}{spread:>7.1%}{cpu:>6.2f}s")
            lines.append(f"{scene:<17}{t:>7}  " + "".join(cells))
    return lines, failed


def sign_test(wins, n):
    """The p value of a two-sided sign test: `wins` of `n` rounds (ties left out) against a fair coin"""
    if n == 0:
        return 1.0
    k = min(wins, n - wins)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def paired(variants, scenes, threads, runs):
    """The table lines of the paired per-round ratios of every variant after the first to the first one"""
    names = [v[0] for v in variants]
    if len(names) < 2:
        return []
    by = {(r["variant"], r["scene"], r["threads"], r.get("round")): r["steps_per_s"] for r in runs if not r["error"]}
    lines = [f"{'Scene':<17}{'Threads':>7}  " + "".join(f"{n + ' / ' + names[0]:>36}" for n in names[1:])]
    lines.append(f"{'':<17}{'':>7}  " + "".join(f"{'ratio':>7}{'q1..q3':>15}{'wins':>8}{'p':>6}" for _ in names[1:]))
    for scene in scenes:
        for t in threads:
            cells = []
            for n in names[1:]:
                ratios = [by[(n, scene, t, rd)] / by[(names[0], scene, t, rd)] for (v, s, th, rd) in by
                          if v == n and s == scene and th == t and rd is not None and (names[0], scene, t, rd) in by]
                if not ratios:
                    cells.append(f"{'-':>36}")
                    continue
                q1, q3 = (statistics.quantiles(ratios, n=4, method="inclusive")[::2] if len(ratios) > 1
                          else (ratios[0], ratios[0]))
                wins = sum(1 for x in ratios if x > 1)
                p = sign_test(wins, sum(1 for x in ratios if x != 1))
                won = f"{wins}/{len(ratios)}"
                cells.append(f"{statistics.median(ratios):>7.3f}{q1:>8.3f}..{q3:<5.3f}{won:>8}{p:>6.3f}")
            lines.append(f"{scene:<17}{t:>7}  " + "".join(cells))
    return lines


def report(variants, runs, rounds):
    """Print both tables of the runs (scenes and thread counts as they occur); 1 if a run failed or hashes differ"""
    scenes = [s for s in SCENES if any(r["scene"] == s for r in runs)]
    threads = sorted({r["threads"] for r in runs})
    lines, failed = summarize(variants, scenes, threads, runs)
    print(f"{rounds} rounds, median steps per second, ratio to {variants[0][0]}, spread (max - min) / median, "
          "median CPU time of a run")
    for line in lines:
        print(line)
    pl = paired(variants, scenes, threads, runs)
    if pl:
        print(f"Paired per-round ratios to {variants[0][0]}: median, quartiles, rounds won, two-sided sign test p")
        for line in pl:
            print(line)
    if failed:
        print("wall_rounds FAILED:")
        for f in failed:
            print("  " + f)
        return 1
    return 0


def self_test():
    assert parse_variant("base=/x/ZoltPerformanceTest") == ("base", "/x/ZoltPerformanceTest", [])
    assert parse_variant("spin=/x/Z,-spin_us=30,-p") == ("spin", "/x/Z", ["-spin_us=30", "-p"])
    for bad in ["/x/Z", "=/x/Z", "a="]:
        try:
            parse_variant(bad)
            raise AssertionError(f"{bad!r} must fail")
        except ValueError:
            pass
    out = "Running scene: Pyramid\nMotion Quality, Thread Count, Steps / Second, Hash\nLinearCast, 4, 112.400000, 0x74d0118836ac0892\n"
    assert parse_result(out) == (112.4, "0x74d0118836ac0892")
    assert parse_result("Discrete, 1, 536.5, 0x1\n") == (536.5, "0x1") and parse_result("no result") is None
    order = schedule([("a",), ("b",), ("c",)], ["S"], [1, 4], 2)
    assert [o[3] for o in order] == [0, 1, 2, 0, 1, 2, 1, 2, 0, 1, 2, 0], order
    assert len(order) == 12 and order[0][:3] == (0, "S", 1) and order[-1][:3] == (1, "S", 4)
    variants = [("cpp", "c", []), ("zolt", "z", [])]
    run = lambda v, sps, h="0x1", t=1, err=None, rd=None: {"round": rd, "variant": v, "scene": "Pyramid", "threads": t,
                                                           "steps_per_s": sps, "hash": h, "cpu_s": 2.0, "error": err}
    runs = [run("cpp", 100.0, rd=0), run("cpp", 110.0, rd=1), run("cpp", 90.0, rd=2), run("zolt", 120.0, rd=0),
            run("zolt", 130.0, rd=1), run("zolt", 110.0, rd=2)]
    lines, failed = summarize(variants, ["Pyramid"], [1], runs)
    assert failed == [] and "100.0  1.000" in lines[2] and "120.0  1.200" in lines[2] and "20.0%" in lines[2], lines
    # A variant that computes other bits, at one thread count only, fails
    lines, failed = summarize(variants, ["Pyramid"], [1, 4], runs + [run("zolt", 200.0, h="0x2", t=4)])
    assert len(failed) == 1 and "Pyramid: different hashes: 0x1 (cpp -t=1, zolt -t=1); 0x2 (zolt -t=4)" == failed[0], failed
    assert lines[3].split()[2] == "-", lines  # no C++ run at 4 threads
    lines, failed = summarize(variants, ["Pyramid"], [1], runs + [run("zolt", None, h=None, err="z failed")])
    assert failed == ["z failed"], failed
    # Paired ratios: 1.2, 130/110 = 1.182, 110/90 = 1.222 -> median 1.2, 3 of 3 rounds won, p = 0.25
    pl = paired(variants, ["Pyramid"], [1], runs)
    assert pl[2].split() == ["Pyramid", "1", "1.200", "1.191..1.211", "3/3", "0.250"], pl
    assert sign_test(12, 15) == 2 * (1 + 15 + 105 + 455) / 2 ** 15 and round(sign_test(12, 15), 3) == 0.035
    assert sign_test(15, 15) == 2 / 2 ** 15 and sign_test(7, 14) == 1.0 and sign_test(0, 0) == 1.0
    # A round without its pair is left out, and so is a failed run
    pl = paired(variants, ["Pyramid"], [1], runs + [run("zolt", 500.0, rd=3), run("cpp", None, h=None, err="x", rd=4),
                                                     run("zolt", 1.0, rd=4)])
    assert pl[2].split()[2:] == ["1.200", "1.191..1.211", "3/3", "0.250"], pl
    assert paired(variants[:1], ["Pyramid"], [1], runs) == []
    # --report reads the file that --json wrote
    import contextlib
    import io
    import tempfile
    with tempfile.TemporaryDirectory(prefix="wall-rounds-test-") as d:
        path = os.path.join(d, "runs.json")
        with open(path, "w") as f:
            json.dump({"variants": variants, "runs": runs}, f)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            assert main(["--report", path]) == 0
        assert "Paired per-round ratios to cpp" in out.getvalue() and "3/3" in out.getvalue(), out.getvalue()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            assert main(["--report", path, "--ref", "zolt"]) == 0
            assert main(["--report", path, "--ref", "nope"]) == 2
        assert "Paired per-round ratios to zolt" in out.getvalue() and "0/3" in out.getvalue(), out.getvalue()
    print("wall_rounds self-test passed")
    return 0


def main(args):
    if args == ["--self-test"]:
        return self_test()
    if args[:1] == ["--report"] and (len(args) == 2 or (len(args) == 4 and args[2] == "--ref")):
        with open(args[1]) as f:
            d = json.load(f)
        variants = [tuple(v) for v in d["variants"]]
        if len(args) == 4:
            ref = [v for v in variants if v[0] == args[3]]
            if not ref:
                print(f"wall_rounds: no variant {args[3]!r} in {args[1]}")
                return 2
            variants = ref + [v for v in variants if v[0] != args[3]]
        rounds = len({r.get("round") for r in d["runs"]})
        return report(variants, d["runs"], rounds)
    rounds, threads, scenes, steps, json_path, variants = 7, [1, 2, 4], SCENES, None, None, []
    it = iter(args)
    try:
        for a in it:
            if a == "--rounds":
                rounds = int(next(it))
            elif a == "--threads":
                threads = [int(t) for t in next(it).split(",")]
            elif a == "--scenes":
                scenes = next(it).split(",")
            elif a == "--steps":
                steps = int(next(it))
            elif a == "--json":
                json_path = next(it)
            else:
                variants.append(parse_variant(a))
    except (StopIteration, ValueError) as e:
        print(f"wall_rounds: {e or 'an option lacks its value'}\n{__doc__}")
        return 2
    unknown = set(scenes) - set(SCENES)
    if not variants or unknown:
        print(f"wall_rounds: unknown scenes {sorted(unknown)}" if unknown else __doc__)
        return 2
    if len({v[0] for v in variants}) != len(variants):
        print("wall_rounds: two variants have the same name")
        return 2
    runs = []
    order = schedule(variants, scenes, threads, rounds)
    for k, (r, scene, t, vi) in enumerate(order):
        name, binary, extra = variants[vi]
        sps, h, cpu, err = run_once(binary, extra, scene, t, steps)
        runs.append({"round": r, "variant": name, "scene": scene, "threads": t, "steps_per_s": sps, "hash": h,
                     "cpu_s": cpu, "error": err})
        print(f"[{k + 1}/{len(order)}] round {r + 1} {scene} -t={t} {name}: " + (err.splitlines()[0] if err else f"{sps:.1f} steps/s {h}"),
              file=sys.stderr, flush=True)
    if json_path:
        with open(json_path, "w") as f:
            json.dump({"variants": variants, "runs": runs}, f, indent=1)
    return report(variants, runs, rounds)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
