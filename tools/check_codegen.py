#!/usr/bin/env python3
"""Codegen check of the Phase 10 regression guard (`zig build codegen-check`).

Zolt once spent 80-96% of its instructions in hidden zero-fills of large stack buffers, while every parity test and
hash passed (Docs/Zolt/Roadmap.md, Phase 10). Bit exactness cannot see such a bug, so this script reads the machine
code instead. `zig build codegen-check` builds ZoltPerformanceTest with ReleaseFast for x86-64-v3 and runs:

    python3 tools/check_codegen.py <ZoltPerformanceTest binary>
    python3 tools/check_codegen.py --self-test     # check the parser on a small made-up listing

It reads `objdump -d -l` of the binary and checks every function of the library: the names that start with one of
PREFIXES. The check fails when such a function
1. calls memset, memcpy or memmove (compiler_rt or libc) with a constant size over 512 bytes (FILL_COPY_LIMIT): a
   fill or a copy of a whole large buffer. `rep stos` / `rep movs` with a constant count count too;
2. copies more than 256 bytes (RODATA_LIMIT) from read-only data, such as an `__anon_*` image: the comptime value of
   `.empty`, `.{}` or a field default of a large struct;
3. runs the byte loop of `std.mem.swap` (instructions whose debug line is in std/mem.zig `swap`). Use `Core.swap`.
   Without debug line info this sub-check is skipped, and the script says so.
The size comes from the last constant moved into %rdx (or %rcx for `rep`) before the call in the same function. The
source of a copy comes from %rsi. The scan is linear, so it can be fooled by branches, but in practice it finds the
real sites.

ALLOW lists the sites that may stay: constraint and body creation, teardown, and one copy per update. An entry is a
regular expression on the function name, the kinds of access (fill, copy, copy from read-only data) and the largest
size. It never names an address, so it survives code changes. The script prints the allowed sites, and offenders as
`function: size, call site`. Only the code that ZoltPerformanceTest links is checked.

How to fix an offender: construct the value in place (`var x: T = undefined;` and then `x.clear()` or
`x.init(...)`), give no field default to a large field, and do not return a large value by value
(Docs/Zolt/PortingGuide.md, "Large values"). Add an entry to ALLOW only for code that does not run per body, per
contact or per query.
"""
import os
import re
import subprocess
import sys

# The repository root: the call sites are printed relative to it
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")) + os.sep
# The library functions that are checked (Zolt's top-level namespaces). ObjectStream, Renderer, Compute, Shaders and
# TriangleSplitter are setup, debug or hair code and are left out.
PREFIXES = ("Physics.", "Geometry.", "AABBTree.", "Math.", "Core.", "Skeleton.")

# A constant memset / memcpy / memmove larger than this fails
FILL_COPY_LIMIT = 512
# A copy from read-only data larger than this fails
RODATA_LIMIT = 256

# The sites that may stay: (regular expression on the function name, what it may do, largest size in bytes, why).
# "fill" is a memset, "copy" a memcpy / memmove, "rodata" a copy from read-only data.
ALLOW = [
    # Constraint creation builds the constraint parts in place from their default values
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.PathConstraint\.PathConstraintSettings,\.createConstraint\)\.",
     {"fill"}, 0x260, "PathConstraint creation"),
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.SwingTwistConstraint\.SwingTwistConstraintSettings,"
     r"\.createConstraint\)\.", {"fill", "copy"}, 0x3c0, "SwingTwistConstraint creation"),
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.SixDOFConstraint\.SixDOFConstraintSettings,\.createConstraint\)\.",
     {"copy"}, 0x740, "SixDOFConstraint creation"),
    # Body creation copies the default Body (0x140 bytes, 0x180 in double precision)
    (r"^Physics\.Body\.BodyManager\.BodyManager\.allocateBody$", {"rodata"}, 0x180, "body creation"),
    # Once per update: the default PhysicsUpdateContext.Step (Jolt's mSteps.resize runs the Step constructor). About
    # 0.01% of the instructions of an update.
    (r"^Physics\.PhysicsSystem\.PhysicsSystem\.update$", {"rodata"}, 0x1ca8, "the steps of an update"),
    # Teardown resets the members to their default values
    (r"^Physics\.PhysicsSystem\.PhysicsSystem\.deinit$", {"rodata"}, 0x500, "PhysicsSystem teardown"),
    (r"^Physics\.Constraints\.ContactConstraintManager\.ContactConstraintManager\.deinit$", {"rodata"}, 0x128,
     "ContactConstraintManager teardown"),
    (r"^Core\.JobSystemThreadPool\.JobSystemThreadPool\.deinit$", {"rodata"}, 0x21c0, "JobSystemThreadPool teardown"),
]

FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
INLINED_RE = re.compile(r"^(\S.*)\(\):$")
LINE_RE = re.compile(r"^(\S+):(\d+)(?: \(discriminator \d+\))?$")
INSN_RE = re.compile(r"^\s+([0-9a-f]+):\s+(.*)$")
CALL_RE = re.compile(r"^(?:call|jmp)\w*\s+([0-9a-f]+) <([^>+]+)(?:\+0x[0-9a-f]+)?>")
MEM_FUNC_RE = re.compile(r"mem(set|cpy|move)")
REP_RE = re.compile(r"^rep\w*\s+(stos|movs)([bwlqd]?)\s*(?:%(\w+),)?")
IMM_RE = re.compile(r"^\$0x([0-9a-f]+)$")
COMMENT_ADDR_RE = re.compile(r"#\s*([0-9a-f]+)")
BYTE_STORE_RE = re.compile(r"^mov[b]?\s+%(?:[abcd]l|[sd]il|[sb]pl|r\d+b),.*\(")
# Instructions that write %rdx / %rcx without naming them as the last operand
IMPLICIT_RDX = re.compile(r"^(cqto|cltd|cwtd|cqo|cdq|rdtsc|rdtscp|rdpid|cpuid|mul|div|idiv|xgetbv)\b")
IMPLICIT_RCX = re.compile(r"^(cpuid|rdtscp|syscall|rep|repz|repnz|loop)\b")
NO_WRITE = re.compile(r"^(cmp|test|bt|ucomis|comis|vucomis|vcomis|push|vptest|ptest)")
REG_NAMES = {
    "rdx": ("rdx", "edx", "dx", "dl", "dh"),
    "rcx": ("rcx", "ecx", "cx", "cl", "ch"),
    "rsi": ("rsi", "esi", "si", "sil"),
}


def run(cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def read_only_ranges(binary, objdump):
    """The address ranges of the read-only data sections"""
    ranges = []
    for line in run([objdump, "-h", binary]).splitlines():
        f = line.split()
        if len(f) >= 4 and f[0].isdigit() and (f[1].startswith(".rodata") or f[1].startswith(".data.rel.ro")):
            start = int(f[3], 16)
            ranges.append((start, start + int(f[2], 16)))
    return ranges


def symbols(binary, nm):
    """The sorted (address, name) list of the data symbols"""
    result = []
    for line in run([nm, "-n", "--defined-only", binary]).splitlines():
        f = line.split(None, 2)
        if len(f) == 3 and f[1] in "rRdDbB":
            result.append((int(f[0], 16), f[2]))
    return result


def symbol_at(syms, address):
    lo, hi = 0, len(syms)
    while lo < hi:
        mid = (lo + hi) // 2
        if syms[mid][0] <= address:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        return None
    base, name = syms[lo - 1]
    return name if base == address else f"{name}+0x{address - base:x}"


def swap_lines(path):
    """The line range of `pub fn swap` in std/mem.zig, or None"""
    try:
        with open(path) as f:
            lines = f.read().splitlines()
    except OSError:
        return None
    for i, line in enumerate(lines):
        if line.startswith("pub fn swap("):
            for j in range(i + 1, len(lines)):
                if lines[j] == "}":
                    return (i + 1, j + 1)
    return None


def operands(text):
    """Split `mnemonic a,b` into the mnemonic and the operands (commas inside parentheses stay)"""
    text = text.split("#")[0].strip()
    parts = text.split(None, 1)
    if len(parts) < 2:
        return parts[0] if parts else "", []
    ops, depth, cur = [], 0, ""
    for ch in parts[1]:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            ops.append(cur.strip())
            cur = ""
        else:
            cur += ch
    ops.append(cur.strip())
    return parts[0], ops


def allowed(func, kind, size):
    for pattern, kinds, limit, _ in ALLOW:
        if re.search(pattern, func) and kind in kinds and size <= limit:
            return pattern
    return None


def scan(listing, ro_ranges, syms, swap_range_of):
    """Scan an `objdump -d -l` listing. Returns (offenders, allowed sites, number of functions, has line info).

    Each offender / allowed site is (function, size, what, call site)."""
    offenders, allowed_sites = [], []
    func = None
    checked = False
    funcs = 0
    line_info = False
    regs = {}
    where = "?"
    inlined = None
    last_own_line = "?"
    swaps = {}

    def in_ro(address):
        return any(lo <= address < hi for lo, hi in ro_ranges)

    def report(kind, size, what, site):
        entry = (func, size, what, site, kind)
        if allowed(func, kind, size):
            allowed_sites.append(entry)
        else:
            offenders.append(entry)

    def check_copy(kind, size, src, site):
        if size is None:
            return
        if kind != "memset" and src is not None and in_ro(src) and size > RODATA_LIMIT:
            name = symbol_at(syms, src) or f"0x{src:x}"
            report("rodata", size, f"copy from read-only {name}", site)
        elif size > FILL_COPY_LIMIT:
            report("fill" if kind == "memset" else "copy", size, kind, site)

    for line in listing.splitlines():
        m = FUNC_RE.match(line)
        if m:
            func = m.group(2)
            checked = func.startswith(PREFIXES)
            funcs += checked
            regs = {}
            inlined = None
            where = last_own_line = "?"
            continue
        if not checked:
            continue
        m = LINE_RE.match(line)
        if m:
            line_info = True
            where = f"{m.group(1).removeprefix(ROOT)}:{m.group(2)}"
            rng = swap_range_of(m.group(1)) if m.group(1).endswith("std/mem.zig") else None
            in_swap = rng is not None and rng[0] <= int(m.group(2)) <= rng[1]
            if not in_swap and "/lib/std/" not in m.group(1):
                last_own_line = where
            regs["in_swap"] = in_swap
            continue
        m = INLINED_RE.match(line)
        if m:
            inlined = m.group(1)
            continue
        m = INSN_RE.match(line)
        if not m:
            continue
        address, text = m.group(1), m.group(2).strip()
        site = f"{where} ({inlined + ', ' if inlined and inlined != func else ''}0x{address})"
        if regs.get("in_swap"):
            s = swaps.setdefault(func, [0, 0, last_own_line, address])
            s[0] += 1
            s[1] += bool(BYTE_STORE_RE.match(text))
        m = CALL_RE.match(text)
        if m:
            k = MEM_FUNC_RE.search(m.group(2))
            if k:
                check_copy("mem" + k.group(1), regs.get("rdx"), regs.get("rsi"), site)
            regs = {"in_swap": regs.get("in_swap")}
            continue
        m = REP_RE.match(text)
        if m:
            # `rep stos %rax,%es:(%rdi)` (the width is the register) or `rep movsq %ds:(%rsi),%es:(%rdi)`
            count = regs.get("rcx")
            if count is not None:
                if m.group(1) == "stos":
                    width = {"al": 1, "ax": 2, "eax": 4, "rax": 8}.get(m.group(3), 1)
                else:
                    width = {"b": 1, "w": 2, "l": 4, "d": 4, "q": 8}.get(m.group(2), 1)
                check_copy("memset" if m.group(1) == "stos" else "memcpy", count * width, regs.get("rsi"), site)
            regs.pop("rcx", None)
            continue
        mnemonic, ops = operands(text)
        if mnemonic.startswith("call"):
            regs = {"in_swap": regs.get("in_swap")}
            continue
        if IMPLICIT_RDX.match(mnemonic):
            regs.pop("rdx", None)
        if IMPLICIT_RCX.match(mnemonic):
            regs.pop("rcx", None)
        if not ops or NO_WRITE.match(mnemonic):
            continue
        dest = ops[-1].lstrip("%")
        for reg, names in REG_NAMES.items():
            if dest not in names:
                continue
            value = None
            imm = IMM_RE.match(ops[0]) if len(ops) == 2 else None
            if mnemonic.startswith("mov") and imm:
                value = int(imm.group(1), 16)
            elif mnemonic.startswith("xor") and len(ops) == 2 and ops[0] == ops[1]:
                value = 0
            elif mnemonic.startswith("lea") and "(%rip)" in ops[0]:
                c = COMMENT_ADDR_RE.search(text)
                value = int(c.group(1), 16) if c else None
            regs[reg] = value
    for f, (count, byte_stores, own, address) in swaps.items():
        offenders.append((f, None, f"std.mem.swap ({count} instructions, {byte_stores} byte stores)",
                          f"{own} (0x{address})", "swap"))
    return offenders, allowed_sites, funcs, line_info


def check(binary):
    objdump = os.environ.get("OBJDUMP", "objdump")
    nm = os.environ.get("NM", "nm")
    listing = run([objdump, "-d", "-l", "--no-show-raw-insn", binary])
    cache = {}

    def swap_range_of(path):
        if path not in cache:
            cache[path] = swap_lines(path)
        return cache[path]

    offenders, allowed_sites, funcs, line_info = scan(listing, read_only_ranges(binary, objdump), symbols(binary, nm),
                                                      swap_range_of)
    print(f"codegen-check: {funcs} library functions ({', '.join(PREFIXES)}), limits: fill / copy {FILL_COPY_LIMIT} B, "
          f"copy from read-only data {RODATA_LIMIT} B")
    if not line_info:
        print("codegen-check: the binary has no debug line info, the std.mem.swap check is skipped")
    elif not any(cache.values()):
        print("codegen-check: std/mem.zig `swap` not found, the std.mem.swap check is skipped")
    for f, size, what, site, _ in allowed_sites:
        print(f"  allowed: {f}: {size} (0x{size:x}) {what}, {site}")
    used = {allowed(f, kind, size) for f, size, _, _, kind in allowed_sites}
    for pattern, _, _, why in ALLOW:
        if pattern not in used:
            print(f"  note: the allow-list entry for {why} matched nothing")
    if offenders:
        print(f"codegen-check FAILED: {len(offenders)} large fills, copies or byte swaps in library code:")
        for f, size, what, site, _ in offenders:
            size_text = f"{size} (0x{size:x}) {what}" if size is not None else what
            print(f"  {f}: {size_text}, {site}")
        print("Construct large values in place and use Core.swap (Docs/Zolt/PortingGuide.md, \"Large values\"). "
              "Only setup-time code may go into ALLOW in tools/check_codegen.py.")
        return 1
    print("codegen-check passed")
    return 0


SELF_TEST_LISTING = """
0000000001000000 <Geometry.EPAConvexHullBuilder.EPAConvexHullBuilder.addPoint>:
Geometry.EPAConvexHullBuilder.EPAConvexHullBuilder.addPoint():
/z/Zolt/Geometry/EPAConvexHullBuilder.zig:423
 1000000:\tlea    0x10(%rsp),%rdi
 1000005:\tmov    $0x808,%edx
 100000a:\txor    %esi,%esi
 100000c:\tcall   1100000 <compiler_rt.memset>
 1000011:\tmov    $0x200,%edx
 1000016:\tcall   1100010 <compiler_rt.memcpy.memcpyFast>
 100001b:\tmov    %rax,%rdx
 100001e:\tcall   1100000 <compiler_rt.memset>
/z/lib/std/mem.zig:3
 1000023:\tmov    %al,(%rcx)
/z/Zolt/Geometry/EPAConvexHullBuilder.zig:430
 1000025:\tmov    $0x9000,%ecx
 100002a:\trep stos %rax,%es:(%rdi)

0000000001000100 <Physics.PhysicsSystem.PhysicsSystem.deinit>:
/z/Zolt/Physics/PhysicsSystem.zig:466
 1000100:\tmov    $0x500,%edx
 1000105:\tmov    $0x2000,%esi
 100010a:\tcall   1100010 <compiler_rt.memcpy.memcpyFast>

0000000001000200 <Physics.Foo.bar>:
/z/Zolt/Physics/Foo.zig:1
 1000200:\tmov    $0x120,%edx
 1000205:\tlea    -0x1000(%rip),%rsi        # 2100 <__anon_7>
 100020c:\tcall   1100010 <compiler_rt.memcpy.memcpyFast>

0000000001000300 <PerformanceTest.main>:
 1000300:\tmov    $0x100000,%edx
 1000305:\tcall   1100000 <compiler_rt.memset>
"""


def self_test():
    std_swap = {"/z/lib/std/mem.zig": (2, 5)}
    offenders, allowed_sites, funcs, line_info = scan(SELF_TEST_LISTING, [(0x2000, 0x3000)],
                                                      [(0x2000, "__anon_6"), (0x2100, "__anon_7")],
                                                      lambda p: std_swap.get(p))
    got = sorted((f.rsplit(".", 1)[-1], size or 0, what.split(" (")[0]) for f, size, what, _, _ in offenders)
    want = sorted([("addPoint", 0x808, "memset"), ("addPoint", 0x9000 * 8, "memset"),
                   ("addPoint", 0, "std.mem.swap"), ("bar", 0x120, "copy from read-only __anon_7")])
    assert got == want, got
    assert [(f.rsplit(".", 1)[-1], s) for f, s, _, _, _ in allowed_sites] == [("deinit", 0x500)], allowed_sites
    assert funcs == 3 and line_info
    print("check_codegen.py self-test passed")
    return 0


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        sys.exit(self_test())
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(check(sys.argv[1]))
