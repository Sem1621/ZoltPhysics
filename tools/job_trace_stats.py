#!/usr/bin/env python3
"""Statistics of a job profile (`zig build performance-test -Djob_profile=true -- ... -p`, Zolt/Core/JobProfiler.zig).

    python3 tools/job_trace_stats.py <scene>_<tag>.json [--steps N]
    python3 tools/job_trace_stats.py --self-test

ZoltPerformanceTest -p writes the Chrome trace of the jobs of a run. This script prints where the time of the job
threads went, the numbers that decide whether the thread pool or the island scheduling is worth changing (Roadmap
Phase 12):

- Per thread: busy (job bodies), the part of busy that jobs spent waiting for work that other jobs hold (`wait`
  spans: the yield loops of the solve jobs and of the soft body simulation), and idle (waiting for jobs or for a
  barrier). All in ms and in percent of the wall time (the first event to the last).
- The idle spans by length (< 5, 5-20, 20-100, 100-500, >= 500 us): their number and their time. Short idle spans
  are the ones a spin before the futex sleep could save.
- Wakeups: idle spans per worker thread and per step. The number of steps is `--steps` (the collision steps of the
  run), or the number of FinalizeIslands jobs (one per collision step).
- The `part` spans by name (the preparation of islands, the final batch of a split island): total, count and
  percent of the wall time.

Times come from the trace (microseconds). The script only reads the file and needs only the standard library.
"""

import json
import sys

IDLE_BUCKETS_US = [5.0, 20.0, 100.0, 500.0]  # upper bounds of the buckets, the last bucket is >= 500 us


def bucket_names():
    names = []
    lower = 0.0
    for upper in IDLE_BUCKETS_US:
        names.append("< %g us" % upper if lower == 0.0 else "%g-%g us" % (lower, upper))
        lower = upper
    names.append(">= %g us" % lower)
    return names


def bucket_index(dur_us):
    for i, upper in enumerate(IDLE_BUCKETS_US):
        if dur_us < upper:
            return i
    return len(IDLE_BUCKETS_US)


def analyze(trace, steps=None):
    """The statistics of a parsed trace (the JSON object) as a dict"""
    events = trace["traceEvents"]
    names = {}
    for e in events:
        if e.get("ph") == "M" and e.get("name") == "thread_name":
            names[e["tid"]] = e["args"]["name"]
    spans = [e for e in events if e.get("ph") == "X"]
    if not spans:
        raise ValueError("the trace has no spans")
    start = min(e["ts"] for e in spans)
    end = max(e["ts"] + e["dur"] for e in spans)
    wall = end - start

    threads = {}
    buckets = [[0, 0.0] for _ in range(len(IDLE_BUCKETS_US) + 1)]
    parts = {}
    num_finalize = 0
    for e in spans:
        tid = e["tid"]
        t = threads.setdefault(tid, {"name": names.get(tid, "tid %d" % tid), "busy": 0.0, "wait": 0.0, "idle": 0.0, "jobs": 0, "idle_spans": 0})
        cat = e.get("cat")
        dur = e["dur"]
        if cat == "job":
            t["busy"] += dur
            t["jobs"] += 1
            if e["name"] == "FinalizeIslands":
                num_finalize += 1
        elif cat == "idle":
            t["idle"] += dur
            t["idle_spans"] += 1
            b = buckets[bucket_index(dur)]
            b[0] += 1
            b[1] += dur
        elif cat == "wait":
            t["wait"] += dur
        elif cat == "part":
            p = parts.setdefault(e["name"], [0, 0.0])
            p[0] += 1
            p[1] += dur
    num_steps = steps if steps else num_finalize
    workers = [t for t in threads.values() if t["name"].startswith("Worker")]
    return {
        "wall_us": wall,
        "threads": [threads[tid] for tid in sorted(threads)],
        "idle_buckets": list(zip(bucket_names(), buckets)),
        "parts": parts,
        "num_steps": num_steps,
        "wakeups_per_worker_per_step": (sum(t["idle_spans"] for t in workers) / len(workers) / num_steps) if workers and num_steps else None,
    }


def report(stats):
    wall = stats["wall_us"]
    pct = lambda us: 100.0 * us / wall if wall > 0 else 0.0
    lines = ["Wall %.3f ms, %s steps" % (wall / 1000.0, stats["num_steps"] or "unknown")]
    lines.append("%-12s %10s %7s %10s %7s %10s %7s %8s" % ("Thread", "busy ms", "%", "wait ms", "%", "idle ms", "%", "jobs"))
    for t in stats["threads"]:
        lines.append("%-12s %10.3f %6.1f%% %10.3f %6.1f%% %10.3f %6.1f%% %8d" % (t["name"], t["busy"] / 1000.0, pct(t["busy"]), t["wait"] / 1000.0, pct(t["wait"]), t["idle"] / 1000.0, pct(t["idle"]), t["jobs"]))
    num_threads = len(stats["threads"])
    lines.append("Idle spans by length (all threads; time in percent of the wall time of all threads):")
    for name, (count, total) in stats["idle_buckets"]:
        lines.append("  %-12s %8d spans %10.3f ms %6.2f%%" % (name, count, total / 1000.0, pct(total) / num_threads if num_threads else 0.0))
    if stats["wakeups_per_worker_per_step"] is not None:
        lines.append("Wakeups: %.2f idle spans per worker thread per step" % stats["wakeups_per_worker_per_step"])
    if stats["parts"]:
        lines.append("Parts of jobs (percent of the wall time):")
        for name, (count, total) in sorted(stats["parts"].items()):
            per_step = " (%.1f per step)" % (count / stats["num_steps"]) if stats["num_steps"] else ""
            lines.append("  %-32s %8d%s %10.3f ms %6.2f%%" % (name, count, per_step, total / 1000.0, pct(total)))
    return "\n".join(lines)


def self_test():
    trace = {"traceEvents": [
        {"name": "thread_name", "ph": "M", "pid": 1, "tid": 0, "args": {"name": "Worker 0"}},
        {"name": "thread_name", "ph": "M", "pid": 1, "tid": 1, "args": {"name": "Thread 0"}},
        {"name": "FinalizeIslands", "cat": "job", "ph": "X", "pid": 1, "tid": 0, "ts": 0.0, "dur": 10.0},
        {"name": "Idle: wait for jobs", "cat": "idle", "ph": "X", "pid": 1, "tid": 0, "ts": 10.0, "dur": 3.0},
        {"name": "SolveVelocityConstraints", "cat": "job", "ph": "X", "pid": 1, "tid": 0, "ts": 13.0, "dur": 50.0},
        {"name": "Wait: split island batches", "cat": "wait", "ph": "X", "pid": 1, "tid": 0, "ts": 20.0, "dur": 7.0},
        {"name": "Prepare island (split)", "cat": "part", "ph": "X", "pid": 1, "tid": 0, "ts": 14.0, "dur": 4.0},
        {"name": "Idle: wait for jobs", "cat": "idle", "ph": "X", "pid": 1, "tid": 0, "ts": 63.0, "dur": 37.0},
        {"name": "Idle: wait for barrier", "cat": "idle", "ph": "X", "pid": 1, "tid": 1, "ts": 0.0, "dur": 600.0},
    ]}
    s = analyze(trace)
    assert s["wall_us"] == 600.0, s
    w = s["threads"][0]
    assert (w["name"], w["busy"], w["wait"], w["idle"], w["jobs"], w["idle_spans"]) == ("Worker 0", 60.0, 7.0, 40.0, 2, 2), w
    counts = [c for _, (c, _t) in s["idle_buckets"]]
    assert counts == [1, 0, 1, 0, 1], counts  # 3 us, 37 us, 600 us
    assert s["num_steps"] == 1 and s["wakeups_per_worker_per_step"] == 2.0, s
    assert s["parts"] == {"Prepare island (split)": [1, 4.0]}, s["parts"]
    assert analyze(trace, steps=4)["wakeups_per_worker_per_step"] == 0.5
    text = report(s)
    assert "Worker 0" in text and "< 5 us" in text and ">= 500 us" in text and "Prepare island (split)" in text, text
    try:
        analyze({"traceEvents": []})
        raise AssertionError("an empty trace must fail")
    except ValueError:
        pass
    print("job_trace_stats self-test passed")


def main(argv):
    if argv[1:] == ["--self-test"]:
        self_test()
        return 0
    args = argv[1:]
    steps = None
    if "--steps" in args:
        i = args.index("--steps")
        steps = int(args[i + 1])
        del args[i:i + 2]
    if len(args) != 1:
        print(__doc__.strip().split("\n\n")[1], file=sys.stderr)
        return 2
    with open(args[0]) as f:
        trace = json.load(f)
    print(report(analyze(trace, steps)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
