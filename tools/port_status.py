#!/usr/bin/env python3
"""Report the status of the Jolt -> Zolt port.

Every Zig file in Zolt/ and ZoltTests/ starts with a header like:

    //! Port of: Jolt/Math/Vec3.h, Jolt/Math/Vec3.inl, Jolt/Math/Vec3.cpp
    //! Status: complete

This script matches those headers against the C++ files in Jolt/ and UnitTests/ and prints a
summary per directory. It also lints the port: headers that reference missing C++ files, Zig
files without a header, and library files that are not registered in Zolt/zolt.zig.

Usage:
    python3 tools/port_status.py            # print summary + lint warnings
    python3 tools/port_status.py --write    # also regenerate Docs/Zolt/Progress.md
    python3 tools/port_status.py --check    # exit 1 if Progress.md is stale or lint fails (CI)
    python3 tools/port_status.py --next     # list unported C++ units whose dependencies are all ported
"""
import argparse
import os
import re
import signal
import subprocess
import sys
from collections import defaultdict

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
PROGRESS_MD = os.path.join(ROOT, "Docs", "Zolt", "Progress.md")
CPP_EXTENSIONS = (".h", ".inl", ".cpp")
STATUSES = ("complete", "partial", "stub")

# Directories and files that are intentionally postponed (see Docs/Zolt/Roadmap.md). They are listed
# with status "deferred" and do not count towards the progress percentage until a Zig file ports them.
DEFERRED = {
    "Jolt/Compute": "GPU compute backends (DX12/Vulkan/Metal), needed for GPU hair only",
    "Jolt/Shaders": "GPU shaders for Jolt/Compute",
    "Jolt/Renderer": "debug renderer interface, ported with the debug renderer phase",
    "Jolt/Core/Profiler": "developer tooling, JPH_PROFILE is compiled out in the deterministic Distribution configuration",
    "Jolt/Core/RTTI": "serialization support, ported with ObjectStream (Phase 8)",
    "Jolt/Core/Factory": "serialization support, ported with ObjectStream (Phase 8)",
    "Jolt/Core/StreamUtils": "serialization support, ported with ObjectStream (Phase 8)",
}


# C++ files that have no Zig counterpart because Zig provides the functionality natively.
# Keep the reason short, it is shown in Progress.md. A unit listed here is still reported with its
# real status if a Zig file claims to port it.
NOT_APPLICABLE = {
    "Jolt/Jolt": "umbrella header, the module root is Zolt/zolt.zig",
    "Jolt/Core/IncludeWindows": "Windows headers",
    "Jolt/Core/LSANSuppressions": "sanitizer configuration",
    "Jolt/Core/ARMNeon": "NEON intrinsic helpers, Zolt uses @Vector",
    "Jolt/Core/RISCVVector": "RVV intrinsic helpers, Zolt uses @Vector",
    "Jolt/Core/Memory": "global allocation hooks, Zolt passes std.mem.Allocator explicitly",
    "Jolt/Core/STLAllocator": "STL allocator adapter, Zolt uses std.mem.Allocator",
    "Jolt/Core/STLAlignedAllocator": "STL allocator adapter, Zolt uses std.mem.Allocator",
    "Jolt/Core/NonCopyable": "Zig has no copy constructors",
    "Jolt/Core/ScopeExit": "Zig `defer`",
    "Jolt/Core/IssueReporting": "std.debug.assert and std.log",
    "Jolt/Core/Result": "Zig error unions",
    "Jolt/Core/Array": "std.ArrayList",
    "Jolt/Core/UnorderedMapFwd": "forward declarations",
    "Jolt/Core/UnorderedSetFwd": "forward declarations",
    "Jolt/Core/FPException": "Zolt never enables floating point exceptions",
    "Jolt/Math/MathTypes": "forward declarations and argument type aliases",
    "UnitTests/Core/ArrayTest": "tests Jolt's Array, Zolt uses std.ArrayList",
    "UnitTests/Core/ScopeExitTest": "tests ScopeExit, Zig has `defer`",
    "UnitTests/doctest": "test framework, Zolt uses Zig's test runner",
}


def rel(path):
    return os.path.relpath(path, ROOT).replace(os.sep, "/")


def count_lines(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return sum(1 for line in f if line.strip())


def cpp_units(base_dir):
    """Group C++ files by directory + stem: {unit: [files]}, e.g. Jolt/Math/Vec3 -> [.h, .inl, .cpp]"""
    units = defaultdict(list)
    for dirpath, _, filenames in os.walk(os.path.join(ROOT, base_dir)):
        for name in filenames:
            if name.endswith(CPP_EXTENSIONS):
                path = rel(os.path.join(dirpath, name))
                units[os.path.splitext(path)[0]].append(path)
    return {unit: sorted(files) for unit, files in units.items()}


HEADER_PORT = re.compile(r"^//!\s*Port of:\s*(.*)$")
HEADER_STATUS = re.compile(r"^//!\s*Status:\s*(\w+)")
HEADER_ADDITION = re.compile(r"^//!\s*Zolt addition")
TEST_DECL = re.compile(r"^\s*test\s+\"", re.MULTILINE)


def read_zig_headers(base_dir):
    """{zig_file: (ported_cpp_files, status)} for every .zig file under base_dir"""
    result = {}
    for dirpath, _, filenames in os.walk(os.path.join(ROOT, base_dir)):
        for name in sorted(filenames):
            if not name.endswith(".zig"):
                continue
            path = os.path.join(dirpath, name)
            ports, status = None, None
            with open(path, encoding="utf-8") as f:
                source = f.read()
            for line in source.splitlines():
                if not line.startswith("//!"):
                    break
                m = HEADER_PORT.match(line.strip())
                if m:
                    ports = [p.strip() for p in m.group(1).split(",") if p.strip()]
                if HEADER_ADDITION.match(line.strip()):
                    ports = []  # Zolt specific file without a C++ counterpart
                m = HEADER_STATUS.match(line.strip())
                if m:
                    status = m.group(1).lower()
            # A test file without an explicit status is complete once it contains tests
            if status is None and base_dir == "ZoltTests":
                status = "complete" if TEST_DECL.search(source) else "stub"
            result[rel(path)] = (ports, status)
    return result


def unit_dir(unit):
    return unit.rsplit("/", 1)[0]


def collect():
    lint = []
    jolt_units = cpp_units("Jolt")
    test_units = cpp_units("UnitTests")
    lib_headers = read_zig_headers("Zolt")
    test_headers = read_zig_headers("ZoltTests")

    with open(os.path.join(ROOT, "Zolt", "zolt.zig"), encoding="utf-8") as f:
        root_source = f.read()
    with open(os.path.join(ROOT, "ZoltTests", "unit_tests.zig"), encoding="utf-8") as f:
        tests_root_source = f.read()

    # Map every C++ file to the status of the Zig file that ports it
    cpp_status = {}
    cpp_zig = {}
    for headers, registry, registry_name, prefix in (
        (lib_headers, root_source, "Zolt/zolt.zig", "Zolt/"),
        (test_headers, tests_root_source, "ZoltTests/unit_tests.zig", "ZoltTests/"),
    ):
        for zig_file, (ports, status) in headers.items():
            if zig_file in ("Zolt/zolt.zig", "ZoltTests/unit_tests.zig"):
                continue
            if ports is None:
                lint.append(f"{zig_file}: missing '//! Port of:' header")
                continue
            if status is None and prefix == "Zolt/":
                lint.append(f"{zig_file}: missing '//! Status:' header")
                status = "partial"
            if status is not None and status not in STATUSES:
                lint.append(f"{zig_file}: unknown status '{status}' (use {', '.join(STATUSES)})")
            import_path = zig_file[len(prefix):]
            if f'@import("{import_path}")' not in registry:
                lint.append(f"{zig_file}: not registered in {registry_name}")
            for cpp in ports:
                if cpp.startswith(("Jolt/", "UnitTests/")):
                    if not os.path.exists(os.path.join(ROOT, cpp)):
                        lint.append(f"{zig_file}: ports '{cpp}', which does not exist")
                    cpp_status[cpp] = status or "complete"
                    cpp_zig[cpp] = zig_file

    # Zig 0.16 (LLVM) can pass a runtime bool to a C function with garbage in bits 1..7, which the C++ side
    # reads as true: the parity C ABI must take integers instead (see the guide's Tests section)
    extern_bool = re.compile(r"\bextern fn \w+\([^)]*:\s*bool\s*[,)]")
    for dirpath, _, filenames in os.walk(os.path.join(ROOT, "ZoltParity")):
        for name in sorted(filenames):
            if name.endswith(".zig"):
                path = os.path.join(dirpath, name)
                with open(path, encoding="utf-8") as f:
                    for line_no, line in enumerate(f, 1):
                        if extern_bool.search(line):
                            rel = os.path.relpath(path, ROOT).replace(os.sep, "/")
                            lint.append(f"{rel}:{line_no}: bool parameter in an extern fn, pass c_int instead")

    def unit_status(files):
        statuses = [cpp_status.get(f) for f in files]
        if all(s == "complete" for s in statuses):
            return "complete"
        if all(s is None for s in statuses):
            return "todo"
        if any(s in ("complete", "partial") for s in statuses):
            return "partial"
        return "stub"

    def build_rows(units):
        rows = []
        for unit, files in sorted(units.items()):
            lines = sum(count_lines(os.path.join(ROOT, f)) for f in files)
            zig = sorted({cpp_zig[f] for f in files if f in cpp_zig})
            status = unit_status(files)
            if status == "todo" and unit in NOT_APPLICABLE:
                status = "n/a"
            elif status == "todo" and deferred_reason(unit):
                status = "deferred"
            rows.append({"unit": unit, "files": files, "lines": lines, "status": status, "zig": zig})
        return rows

    return build_rows(jolt_units), build_rows(test_units), lint


def deferred_reason(unit):
    for prefix, reason in DEFERRED.items():
        if unit == prefix or unit.startswith(prefix + "/"):
            return reason
    return None


def group_key(unit):
    """Group by the first two path components below Jolt/, e.g. Jolt/Physics/Collision"""
    parts = unit.split("/")
    if len(parts) <= 2:
        return parts[0]
    if parts[1] == "Physics" and len(parts) > 3:
        return "/".join(parts[:3])
    return "/".join(parts[:2])


WEIGHT = {"complete": 1.0, "partial": 0.5, "stub": 0.1, "todo": 0.0}
ICON = {"complete": "✅", "partial": "🟡", "stub": "⚪", "todo": "❌", "n/a": "➖", "deferred": "⏸"}


def counted(rows):
    """Rows that count towards the progress percentage"""
    return [r for r in rows if r["status"] not in ("n/a", "deferred")]


def group_deferred_reason(items):
    """Reason when every unit of a group is deferred (e.g. a whole directory), else None"""
    reasons = [deferred_reason(r["unit"]) for r in items]
    return reasons[0] if all(reasons) and all(r["status"] == "deferred" for r in items) else None


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[group_key(row["unit"])].append(row)
    summary = []
    for group, items in sorted(groups.items()):
        total = sum(r["lines"] for r in counted(items))
        done = sum(r["lines"] * WEIGHT[r["status"]] for r in counted(items))
        counts = {s: sum(1 for r in items if r["status"] == s) for s in ICON}
        summary.append((group, items, total, done, counts))
    return summary


def percent(done, total):
    return 100.0 * done / total if total else 100.0


def render_markdown(lib_rows, test_rows):
    out = []
    out.append("# Zolt Port Progress")
    out.append("")
    out.append("<!-- Generated by tools/port_status.py --write. Do not edit by hand. -->")
    out.append("")
    out.append("Status comes from the `//! Port of:` / `//! Status:` headers of the Zig files. Percentages are")
    out.append("weighted by non-blank C++ lines (complete = 100%, partial = 50%, stub = 10%). ➖ marks C++ files")
    out.append("that need no port because Zig covers them natively (see `NOT_APPLICABLE` in the script), ⏸ files")
    out.append("that are postponed to a later phase (see `DEFERRED` in the script); neither counts towards the percentage.")
    out.append("")

    in_scope = [r for r in counted(lib_rows) if not deferred_reason(r["unit"])]
    total = sum(r["lines"] for r in in_scope)
    done = sum(r["lines"] * WEIGHT[r["status"]] for r in in_scope)
    test_total = sum(r["lines"] for r in counted(test_rows))
    test_done = sum(r["lines"] * WEIGHT[r["status"]] for r in counted(test_rows))
    out.append(f"**Library: {percent(done, total):.1f}%** of {total} lines in scope · "
               f"**Unit tests: {percent(test_done, test_total):.1f}%** of {test_total} lines")
    out.append("")

    for title, rows in (("Library (Jolt/ → Zolt/)", lib_rows), ("Unit tests (UnitTests/ → ZoltTests/)", test_rows)):
        out.append(f"## {title}")
        out.append("")
        out.append("| Directory | Progress | ✅ | 🟡 | ⚪ | ❌ | ➖ | ⏸ | C++ lines |")
        out.append("|-----------|---------:|---:|---:|---:|---:|---:|---:|----------:|")
        summary = summarize(rows)
        for group, items, g_total, g_done, counts in summary:
            reason = group_deferred_reason(items)
            progress = "deferred" if reason else f"{percent(g_done, g_total):.0f}%"
            out.append(f"| `{group}` | {progress} | {counts['complete']} | {counts['partial']} | "
                       f"{counts['stub']} | {counts['todo']} | {counts['n/a']} | {counts['deferred']} | {g_total} |")
        out.append("")
        for group, items, g_total, g_done, counts in summary:
            reason = group_deferred_reason(items)
            label = f"{group} — deferred: {reason}" if reason else f"{group} — {percent(g_done, g_total):.0f}%"
            out.append(f"<details><summary>{label}</summary>")
            out.append("")
            out.append("| C++ | Lines | Status | Zig |")
            out.append("|-----|------:|--------|-----|")
            for r in items:
                cpp = ", ".join(f"`{os.path.basename(f)}`" for f in r["files"])
                zig = ", ".join(f"`{z}`" for z in r["zig"])
                if r["status"] == "n/a":
                    zig = NOT_APPLICABLE[r["unit"]]
                elif r["status"] == "deferred":
                    zig = deferred_reason(r["unit"])
                out.append(f"| {cpp} | {r['lines']} | {ICON[r['status']]} {r['status']} | {zig} |")
            out.append("")
            out.append("</details>")
            out.append("")
    return "\n".join(out).rstrip() + "\n"


INCLUDE = re.compile(r'^\s*#\s*include\s*<(Jolt/[^>]+)>', re.MULTILINE)


def unit_dependencies(rows):
    """{unit: set of units it includes}, based on the #include <Jolt/...> lines of its files"""
    known = {r["unit"] for r in rows}
    deps = {}
    for r in rows:
        unit_deps = set()
        for f in r["files"]:
            with open(os.path.join(ROOT, f), encoding="utf-8", errors="replace") as fh:
                for include in INCLUDE.findall(fh.read()):
                    dep = os.path.splitext(include)[0]
                    if dep != r["unit"] and dep in known:
                        unit_deps.add(dep)
        deps[r["unit"]] = unit_deps
    return deps


# Units that every Jolt file includes but that have no real porting work (or are replaced by Zig builtins)
IMPLICIT_DEPENDENCIES = {"Jolt/Jolt", "Jolt/Core/Core", "Jolt/Core/IssueReporting", "Jolt/Core/Memory",
                         "Jolt/Core/STLAllocator", "Jolt/Core/Array", "Jolt/Core/Profiler",
                         "Jolt/Core/NonCopyable", "Jolt/Math/MathTypes", "Jolt/Math/Real"}


def print_next(lib_rows):
    status = {r["unit"]: r["status"] for r in lib_rows}
    lines = {r["unit"]: r["lines"] for r in lib_rows}
    deps = unit_dependencies(lib_rows)
    ready, blocked = [], []
    for unit, unit_status_value in sorted(status.items()):
        if unit_status_value != "todo":
            continue
        # Dependencies on deferred units (e.g. the debug renderer) don't block: that code is skipped for now
        missing = sorted(d for d in deps[unit]
                         if status.get(d) not in ("complete", "partial", "n/a")
                         and d not in IMPLICIT_DEPENDENCIES and not deferred_reason(d))
        (blocked if missing else ready).append((unit, missing))
    print(f"Ready to port ({len(ready)} units, all Jolt includes already ported):")
    for unit, _ in sorted(ready, key=lambda u: (group_key(u[0]), lines[u[0]])):
        print(f"  {unit:<64} {lines[unit]:>5} lines")
    print(f"Blocked: {len(blocked)} units (run with --next --verbose to see what blocks them)")
    return blocked


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", action="store_true", help="regenerate Docs/Zolt/Progress.md")
    ap.add_argument("--check", action="store_true", help="fail if Progress.md is stale or lint finds problems")
    ap.add_argument("--next", action="store_true", help="list unported units whose dependencies are ported")
    ap.add_argument("--verbose", action="store_true", help="with --next: also list blocked units")
    args = ap.parse_args()

    lib_rows, test_rows, lint = collect()
    if args.next:
        blocked = print_next(lib_rows)
        if args.verbose:
            for unit, missing in blocked:
                print(f"  {unit}: needs {', '.join(m.removeprefix('Jolt/') for m in missing)}")
        return
    markdown = render_markdown(lib_rows, test_rows)

    for title, rows in (("Library", lib_rows), ("Unit tests", test_rows)):
        print(f"{title}:")
        for group, items, total, done, counts in summarize(rows):
            reason = group_deferred_reason(items)
            progress = "deferred" if reason else f"{percent(done, total):5.1f}%"
            print(f"  {group:<32} {progress:>8}  ({counts['complete']} complete, {counts['partial']} partial, "
                  f"{counts['stub']} stub, {counts['todo']} todo, {counts['n/a']} n/a, {counts['deferred']} deferred, {total} lines)")
    for message in lint:
        print(f"lint: {message}", file=sys.stderr)

    if args.write:
        os.makedirs(os.path.dirname(PROGRESS_MD), exist_ok=True)
        with open(PROGRESS_MD, "w", encoding="utf-8") as f:
            f.write(markdown)
        print(f"Wrote {rel(PROGRESS_MD)}")

    if args.check:
        stale = not os.path.exists(PROGRESS_MD) or open(PROGRESS_MD, encoding="utf-8").read() != markdown
        if stale:
            print("Docs/Zolt/Progress.md is stale, run: python3 tools/port_status.py --write", file=sys.stderr)
        untidy = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "tidy_registry.py"), "--check"]).returncode != 0
        if stale or lint or untidy:
            sys.exit(1)


if __name__ == "__main__":
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)  # Allow piping into head
    main()
