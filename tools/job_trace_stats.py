#!/usr/bin/env python3
"""Statistics of a job profile (`zig build performance-test -Djob_profile=true -- ... -p`, Zolt/Core/JobProfiler.zig).

    python3 tools/job_trace_stats.py <scene>_<tag>.json [--steps N] [--by-job] [--under NAME]
    python3 tools/job_trace_stats.py --self-test

ZoltPerformanceTest -p writes the Chrome trace of the jobs of a run. This script prints where the time of the job
threads went, the numbers that decide whether the thread pool or the island scheduling is worth changing (Roadmap
Phase 12):

- Per thread: busy (job bodies), the part of busy that jobs spent waiting for work that other jobs hold (`wait`
  spans: the yield loops of the solve jobs and of the soft body simulation), and idle (waiting for jobs or for a
  barrier). All in ms and in percent of the wall time (the first event to the last).
- The idle spans by length (< 5, 5-20, 20-50, 50-100, 100-500, >= 500 us): their number and their time. Short idle
  spans are the ones a spin before the futex sleep could save; the share of the spans under 50 us is printed on its
  own line (the rule of Roadmap Phase 12 for a spin), split by the name of the idle span (waiting for jobs in the
  worker loop, or for a barrier on the thread that called WaitForJobs).
- Wakeups: idle spans per worker thread and per step. The number of steps is `--steps` (the collision steps of the
  run), or the number of FinalizeIslands jobs (one per collision step).
- The `part` spans by name (the preparation of islands, the final batch of a split island): total, count and
  percent of the wall time.
- `--by-job`: the busy time of every job name, its count and mean, in percent of the time of all threads.
- `--under NAME`: what the other threads do while one thread is in a span called NAME (Roadmap Phase 13, the final
  batch of a split island): their idle and wait time inside those spans, in percent of the time of all threads, and
  the upper bound of the work a parallel version of the span could move to them: per span the smaller of that time
  and (k - 1) / k of the span (k threads; an even split leaves 1 / k on the thread that runs it).

Times come from the trace (microseconds). When the ring buffer of a thread was full, the trace lacks its oldest events
(JobProfiler writes their number as "overwritten" in the thread's metadata). The statistics then cover only the time
from the latest of the earliest ends of those threads' remaining events on, where every thread has all its events: a
thread records an event when it ends, so every overwritten event ended before that time, also the wait and part spans
inside a job that is still in the trace. Spans that start earlier are cut at that time and not counted as spans, and the steps are the FinalizeIslands jobs in that time (`--steps` would
count the whole run). The script says so in its output. It only reads the file and needs only the standard library.
"""

import bisect
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


def overlap(intervals, begin, end):
    """The time of the sorted, disjoint `intervals` [(start, end)] inside [begin, end)"""
    i = bisect.bisect_right(intervals, (begin, float("inf"))) - 1
    total = 0.0
    for a, b in intervals[max(i, 0):]:
        if a >= end:
            break
        total += max(0.0, min(b, end) - max(a, begin))
    return total


def analyze(trace, steps=None, under=None):
    """The statistics of a parsed trace (the JSON object) as a dict. `under`: the name of the spans of --under."""
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
    first_end = {}
    for e in spans:
        first[e["tid"]] = min(first.get(e["tid"], e["ts"]), e["ts"])
        first_end[e["tid"]] = min(first_end.get(e["tid"], e["ts"] + e["dur"]), e["ts"] + e["dur"])
    # Threads that lost their oldest events: a thread records an event when it ends, so it lost only events that ended
    # before the earliest end of its remaining ones. Count from the latest such end of those threads on.
    lossy = sorted(tid for tid, n in overwritten.items() if n and tid in first)
    start = max(first_end[tid] for tid in lossy) if lossy else min(first.values())
    end = max(e["ts"] + e["dur"] for e in spans)
    if end <= start:
        raise ValueError("no time is left after the overwritten events")
    wall = end - start

    threads = {}
    buckets = [[0, 0.0] for _ in range(len(IDLE_BUCKETS_US) + 1)]
    parts = {}
    jobs = {}
    short_idle = 0.0
    short_idle_by_name = {}
    free = {}  # per thread: the idle and wait intervals (cut at start), for --under
    under_spans = []
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
        if under is not None and e["name"] == under and whole:
            under_spans.append((tid, ts, dur))
        if cat == "job":
            t["busy"] += dur
            j = jobs.setdefault(e["name"], [0, 0.0])
            j[1] += dur
            if whole:
                t["jobs"] += 1
                j[0] += 1
                if e["name"] == "FinalizeIslands":
                    num_finalize += 1
        elif cat == "idle":
            t["idle"] += dur
            free.setdefault(tid, []).append((ts + e["dur"] - dur, ts + e["dur"]))
            if whole:
                t["idle_spans"] += 1
                b = buckets[bucket_index(dur)]
                b[0] += 1
                b[1] += dur
                if dur < SHORT_IDLE_US:
                    short_idle += dur
                    short_idle_by_name[e["name"]] = short_idle_by_name.get(e["name"], 0.0) + dur
        elif cat == "wait":
            t["wait"] += dur
            free.setdefault(tid, []).append((ts + e["dur"] - dur, ts + e["dur"]))
        elif cat == "part":
            if whole:
                p = parts.setdefault(e["name"], [0, 0.0])
                p[0] += 1
                p[1] += dur
    num_steps = steps if steps and not lossy else num_finalize
    workers = [t for t in threads.values() if t["name"].startswith("Worker")]
    under_stats = None
    if under is not None:
        for v in free.values():
            v.sort()
        k = len(threads)
        span_us = other_free_us = movable_us = 0.0
        for tid, ts, dur in under_spans:
            o = sum(overlap(v, ts, ts + dur) for other, v in free.items() if other != tid)
            span_us += dur
            other_free_us += o
            movable_us += min(o, (k - 1) / k * dur)
        under_stats = {"name": under, "count": len(under_spans), "span_us": span_us, "other_free_us": other_free_us,
                       "movable_us": movable_us}
    return {
        "wall_us": wall,
        "threads": [threads[tid] for tid in sorted(threads)],
        "idle_buckets": list(zip(bucket_names(), buckets)),
        "short_idle_us": short_idle,
        "short_idle_by_name": short_idle_by_name,
        "parts": parts,
        "jobs": jobs,
        "under": under_stats,
        "num_steps": num_steps,
        "steps_ignored": bool(steps) and bool(lossy),
        "overwritten": {names.get(tid, "tid %d" % tid): overwritten[tid] for tid in lossy},
        "wakeups_per_worker_per_step": (sum(t["idle_spans"] for t in workers) / len(workers) / num_steps) if workers and num_steps else None,
    }


def report(stats, by_job=False):
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
    for name, total in sorted(stats["short_idle_by_name"].items()):
        lines.append("  %-30s %6.2f%%" % (name, pct(total) / num_threads if num_threads else 0.0))
    if stats["wakeups_per_worker_per_step"] is not None:
        lines.append("Wakeups: %.2f idle spans per worker thread per step" % stats["wakeups_per_worker_per_step"])
    if stats["parts"]:
        lines.append("Parts of jobs (percent of the wall time):")
        for name, (count, total) in sorted(stats["parts"].items()):
            per_step = " (%.1f per step)" % (count / stats["num_steps"]) if stats["num_steps"] else ""
            lines.append("  %-32s %8d%s %10.3f ms %6.2f%%" % (name, count, per_step, total / 1000.0, pct(total)))
    if by_job and stats["jobs"]:
        lines.append("Jobs by busy time (percent of the wall time of all threads):")
        for name, (count, total) in sorted(stats["jobs"].items(), key=lambda kv: -kv[1][1]):
            mean = " %9.1f us mean" % (total / count) if count else ""
            lines.append("  %-32s %8d %10.3f ms %6.2f%%%s" % (name, count, total / 1000.0, pct(total) / num_threads if num_threads else 0.0, mean))
    u = stats["under"]
    if u is not None:
        all_threads = lambda us: pct(us) / num_threads if num_threads else 0.0
        lines.append("Under \"%s\": %d spans, %.3f ms (%.2f%% of the wall time)" % (u["name"], u["count"], u["span_us"] / 1000.0, pct(u["span_us"])))
        lines.append("  other threads idle + wait inside: %.3f ms, %.2f%% of the wall time of all threads" % (u["other_free_us"] / 1000.0, all_threads(u["other_free_us"])))
        lines.append("  work a parallel version could move (upper bound): %.3f ms, %.2f%% of the wall time of all threads" % (u["movable_us"] / 1000.0, all_threads(u["movable_us"])))
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
    assert s["short_idle_by_name"] == {"Idle: wait for jobs": 40.0}, s["short_idle_by_name"]
    assert s["jobs"] == {"FinalizeIslands": [1, 10.0], "SolveVelocityConstraints": [1, 50.0]}, s["jobs"]
    assert s["under"] is None and "Under" not in text and "Jobs by busy time" not in text, text
    text = report(s, by_job=True)
    assert "Jobs by busy time" in text and "SolveVelocityConstraints" in text and "50.0 us mean" in text, text
    # --under: Thread 0 waits for the barrier the whole time, so it is free under every span of Worker 0. The
    # prepare span (4 us, 2 threads) can move at most half of itself; the wait span is Worker 0's own.
    s = analyze(trace, under="Prepare island (split)")
    assert s["under"] == {"name": "Prepare island (split)", "count": 1, "span_us": 4.0, "other_free_us": 4.0, "movable_us": 2.0}, s["under"]
    text = report(s)
    assert 'Under "Prepare island (split)": 1 spans' in text and "(upper bound): 0.002 ms" in text, text
    # Free time of the others only counts inside the span: Worker 0 is idle in [10, 13) and [63, 100) and waits in [20, 27)
    s = analyze(dict(trace, traceEvents=trace["traceEvents"] + [
        {"name": "Final batch", "cat": "part", "ph": "X", "pid": 1, "tid": 1, "ts": 5.0, "dur": 60.0}]), under="Final batch")
    assert s["under"]["other_free_us"] == 3.0 + 7.0 + 2.0 and s["under"]["movable_us"] == 12.0, s["under"]
    assert overlap([(0.0, 2.0), (4.0, 8.0), (9.0, 10.0)], 1.0, 5.0) == 2.0
    assert overlap([(0.0, 2.0)], 3.0, 5.0) == 0.0 and overlap([], 0.0, 1.0) == 0.0

    # Worker 0 lost its 2 oldest events (by end: FinalizeIslands and the idle span up to 13 us). Its earliest end left
    # is that of the prepare part at 18 us, so only the time from 18 us on counts: the job that started at 13 us is
    # cut, and so is the prepare part
    def lose(n):
        """The trace with the n events of Worker 0 that ended first overwritten"""
        ends = sorted(e["ts"] + e["dur"] for e in trace["traceEvents"] if e.get("tid") == 0 and e["ph"] == "X")
        return {"traceEvents": [dict(e, args={"name": "Worker 0", "overwritten": n}) if e.get("tid") == 0 and e["ph"] == "M" else e
                                for e in trace["traceEvents"] if not (e.get("tid") == 0 and e["ph"] == "X" and e["ts"] + e["dur"] <= ends[n - 1])]}
    lossy = lose(2)
    s = analyze(lossy, steps=4)
    assert s["wall_us"] == 582.0, s
    w, other = s["threads"]
    assert (w["busy"], w["wait"], w["idle"], w["jobs"], w["idle_spans"]) == (45.0, 7.0, 37.0, 0, 1), w
    assert s["parts"] == {}, s["parts"]
    assert (other["idle"], other["idle_spans"]) == (582.0, 0), other  # the barrier wait started before 18 us: cut, not a span
    assert s["overwritten"] == {"Worker 0": 2} and s["steps_ignored"] and s["num_steps"] == 0, s
    assert s["wakeups_per_worker_per_step"] is None, s  # no FinalizeIslands in the counted time
    text = report(s)
    assert "Note: the ring buffers of Worker 0 were full (2 oldest events not in the trace)" in text and "--steps counts the whole run" in text, text
    # Worker 0 also lost the prepare part and the wait inside the job it kept: the counted time starts at the end of
    # that job (63 us), so the lost wait does not show up as busy time
    s = analyze(lose(4))
    w = s["threads"][0]
    assert s["wall_us"] == 537.0 and (w["busy"], w["wait"], w["idle"], w["jobs"], w["idle_spans"]) == (0.0, 0.0, 37.0, 0, 1), (s["wall_us"], w)
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
    under = None
    if "--steps" in args:
        i = args.index("--steps")
        steps = int(args[i + 1])
        del args[i:i + 2]
    if "--under" in args:
        i = args.index("--under")
        under = args[i + 1]
        del args[i:i + 2]
    by_job = "--by-job" in args
    if by_job:
        args.remove("--by-job")
    if len(args) != 1:
        print(__doc__.strip().split("\n\n")[1], file=sys.stderr)
        return 2
    with open(args[0]) as f:
        trace = json.load(f)
    print(report(analyze(trace, steps, under), by_job))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
