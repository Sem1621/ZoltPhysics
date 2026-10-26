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
- The idle spans by length (< 5, 5-20, 20-50, 50-100, 100-500, >= 500 us): their number and their time. Short idle
  spans are the ones a spin before the futex sleep could save; the share of the spans under 50 us is printed on its
  own line (the rule of Roadmap Phase 12 for a spin).
- Wakeups: idle spans per worker thread and per step. The number of steps is `--steps` (the collision steps of the
  run), or the number of FinalizeIslands jobs (one per collision step).
- The `part` spans by name (the preparation of islands, the final batch of a split island): total, count and
  percent of the wall time.

Times come from the trace (microseconds). When the ring buffer of a thread was full, the trace lacks its oldest events
(JobProfiler writes their number as "overwritten" in the thread's metadata). The statistics then cover only the time
from the newest first event of those threads on, where every thread has all its events: spans that start earlier are
cut at that time and not counted as spans, and the steps are the FinalizeIslands jobs in that time (`--steps` would
count the whole run). The script says so in its output. It only reads the file and needs only the standard library.
"""

import json
import sys

IDLE_BUCKETS_US = [5.0, 20.0, 50.0, 100.0, 500.0]  # upper bounds of the buckets, the last bucket is >= 500 us
SHORT_IDLE_US = 50.0  # idle spans shorter than this are the ones a spin before the futex sleep could save


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
    overwritten = {}
    for e in events:
        if e.get("ph") == "M" and e.get("name") == "thread_name":
            names[e["tid"]] = e["args"]["name"]
            overwritten[e["tid"]] = e["args"].get("overwritten", 0)
    spans = [e for e in events if e.get("ph") == "X"]
    if not spans:
        raise ValueError("the trace has no spans")
    first = {}
    for e in spans:
        first[e["tid"]] = min(first.get(e["tid"], e["ts"]), e["ts"])
    # Threads that lost their oldest events: count from the newest first event of those threads on
    lossy = sorted(tid for tid, n in overwritten.items() if n and tid in first)
    start = max(first[tid] for tid in lossy) if lossy else min(first.values())
    end = max(e["ts"] + e["dur"] for e in spans)
    if end <= start:
        raise ValueError("no time is left after the overwritten events")
    wall = end - start

    threads = {}
    buckets = [[0, 0.0] for _ in range(len(IDLE_BUCKETS_US) + 1)]
    parts = {}
    short_idle = 0.0
    num_finalize = 0
    for e in spans:
        ts = e["ts"]
        dur = e["dur"]
        whole = ts >= start  # the span lies in the counted time; otherwise only its part after `start` counts
        if not whole:
            dur = ts + dur - start
            if dur <= 0:
                continue
        tid = e["tid"]
        t = threads.setdefault(tid, {"name": names.get(tid, "tid %d" % tid), "busy": 0.0, "wait": 0.0, "idle": 0.0, "jobs": 0, "idle_spans": 0})
        cat = e.get("cat")
        if cat == "job":
            t["busy"] += dur
            if whole:
                t["jobs"] += 1
                if e["name"] == "FinalizeIslands":
                    num_finalize += 1
        elif cat == "idle":
            t["idle"] += dur
            if whole:
                t["idle_spans"] += 1
                b = buckets[bucket_index(dur)]
                b[0] += 1
                b[1] += dur
                if dur < SHORT_IDLE_US:
                    short_idle += dur
        elif cat == "wait":
            t["wait"] += dur
        elif cat == "part":
            if whole:
                p = parts.setdefault(e["name"], [0, 0.0])
                p[0] += 1
                p[1] += dur
    num_steps = steps if steps and not lossy else num_finalize
    workers = [t for t in threads.values() if t["name"].startswith("Worker")]
    return {
        "wall_us": wall,
        "threads": [threads[tid] for tid in sorted(threads)],
        "idle_buckets": list(zip(bucket_names(), buckets)),
        "short_idle_us": short_idle,
        "parts": parts,
        "num_steps": num_steps,
        "steps_ignored": bool(steps) and bool(lossy),
        "overwritten": {names.get(tid, "tid %d" % tid): overwritten[tid] for tid in lossy},
        "wakeups_per_worker_per_step": (sum(t["idle_spans"] for t in workers) / len(workers) / num_steps) if workers and num_steps else None,
    }


def report(stats):
    wall = stats["wall_us"]
    pct = lambda us: 100.0 * us / wall if wall > 0 else 0.0
    lines = []
    if stats["overwritten"]:
        lines.append("Note: the ring buffers of %s were full (%d oldest events not in the trace); the statistics cover the last %.3f ms, where every thread has all its events" % (
            ", ".join(sorted(stats["overwritten"])), sum(stats["overwritten"].values()), wall / 1000.0))
        if stats["steps_ignored"]:
            lines.append("Note: --steps counts the whole run, the steps are the FinalizeIslands jobs of that time")
    lines.append("Wall %.3f ms, %s steps" % (wall / 1000.0, stats["num_steps"] or "unknown"))
    lines.append("%-12s %10s %7s %10s %7s %10s %7s %8s" % ("Thread", "busy ms", "%", "wait ms", "%", "idle ms", "%", "jobs"))
    for t in stats["threads"]:
        lines.append("%-12s %10.3f %6.1f%% %10.3f %6.1f%% %10.3f %6.1f%% %8d" % (t["name"], t["busy"] / 1000.0, pct(t["busy"]), t["wait"] / 1000.0, pct(t["wait"]), t["idle"] / 1000.0, pct(t["idle"]), t["jobs"]))
    num_threads = len(stats["threads"])
    lines.append("Idle spans by length (all threads; time in percent of the wall time of all threads):")
    for name, (count, total) in stats["idle_buckets"]:
        lines.append("  %-12s %8d spans %10.3f ms %6.2f%%" % (name, count, total / 1000.0, pct(total) / num_threads if num_threads else 0.0))
    lines.append("Idle spans < %g us: %.2f%% of the wall time of all threads" % (SHORT_IDLE_US, pct(stats["short_idle_us"]) / num_threads if num_threads else 0.0))
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
    assert counts == [1, 0, 1, 0, 0, 1], counts  # 3 us, 37 us, 600 us
    assert s["short_idle_us"] == 40.0, s  # 3 + 37 us
    assert s["num_steps"] == 1 and s["wakeups_per_worker_per_step"] == 2.0, s
    assert s["parts"] == {"Prepare island (split)": [1, 4.0]}, s["parts"]
    assert s["overwritten"] == {} and not s["steps_ignored"], s
    assert analyze(trace, steps=4)["wakeups_per_worker_per_step"] == 0.5
    text = report(s)
    assert "Worker 0" in text and "< 5 us" in text and "20-50 us" in text and ">= 500 us" in text and "Prepare island (split)" in text, text
    assert "Idle spans < 50 us: 3.33% of the wall time of all threads" in text and "Note:" not in text, text

    # Worker 0 lost its oldest events: its first event left is at 13 us, so only the time from 13 us on counts
    lossy = {"traceEvents": [dict(e, args={"name": "Worker 0", "overwritten": 2}) if e.get("tid") == 0 and e["ph"] == "M" else e
                             for e in trace["traceEvents"] if not (e.get("tid") == 0 and e["ph"] == "X" and e["ts"] < 13.0)]}
    s = analyze(lossy, steps=4)
    assert s["wall_us"] == 587.0, s
    w, other = s["threads"]
    assert (w["busy"], w["wait"], w["idle"], w["jobs"], w["idle_spans"]) == (50.0, 7.0, 37.0, 1, 1), w
    assert (other["idle"], other["idle_spans"]) == (587.0, 0), other  # the barrier wait started before 13 us: cut, not a span
    assert s["overwritten"] == {"Worker 0": 2} and s["steps_ignored"] and s["num_steps"] == 0, s
    assert s["wakeups_per_worker_per_step"] is None, s  # no FinalizeIslands in the counted time
    text = report(s)
    assert "Note: the ring buffers of Worker 0 were full (2 oldest events not in the trace)" in text and "--steps counts the whole run" in text, text
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
