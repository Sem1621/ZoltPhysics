#!/usr/bin/env python3
"""Codegen check of the Phase 10 regression guard (`zig build codegen-check`).

Zolt once spent 80-96% of its instructions in hidden zero-fills of large stack buffers, while every parity test and
hash passed (Docs/Zolt/Roadmap.md, Phase 10). Bit exactness cannot see such a bug, so this script reads the machine
code instead. `zig build codegen-check` builds ZoltPerformanceTest with ReleaseFast for x86-64-v3 and runs:

    python3 tools/check_codegen.py [--fma] <ZoltPerformanceTest binary>
    python3 tools/check_codegen.py --self-test     # check the parser on a small made-up listing

It reads `objdump -d -l --inlines` of the binary and checks the code of the library: the functions whose names start
with one of PREFIXES, and the library code that LLVM inlined into other functions (an instruction counts when a
function of its inline chain starts with one of PREFIXES). The check fails when library code
1. calls memset, memcpy or memmove (compiler_rt or libc) with a constant size over 512 bytes (FILL_COPY_LIMIT): a
   fill or a copy of a whole large buffer. `rep stos` / `rep movs` with a constant count count too;
2. copies more than 256 bytes (RODATA_LIMIT) from read-only data, such as an `__anon_*` image: the comptime value of
   `.empty`, `.{}` or a field default of a large struct;
3. runs the byte loop of `std.mem.swap` (instructions whose debug line is in std/mem.zig `swap`), or calls a
   `mem.swap` that LLVM did not inline. Use `Core.swap`;
4. holds a fused multiply-add instruction (vfmadd*, vfmsub*, vfnmadd*, vfnmsub*). Without -Dfma Zolt has none: Jolt
   built with CROSS_PLATFORM_DETERMINISTIC never fuses. With -Dfma (`--fma`) only Jolt's 8 JPH_USE_FMADD sites may
   fuse: every FMA instruction must come from one of FMA_SITES (by its inline chain), and at least one must exist.
Without debug line info, the check of inlined library code and of the inlined `std.mem.swap` is skipped, and the
script says so. The size comes from the last constant moved into %rdx (or %rcx for `rep`) before the call in the same
function. The source of a copy comes from %rsi. The scan is linear, so it can be fooled by branches, but in practice
it finds the real sites.

The check also fails when the scan cannot prove anything: when the binary is not x86-64 (the scanner reads x86-64
code only), when it finds fewer than MIN_FUNCS library functions or not SENTINEL (a stripped binary, or names that
changed), or when no ALLOW entry matches (the scanner no longer sees the calls).

ALLOW lists the sites that may stay: constraint, body and system creation, teardown, debug drawing, and one copy per
update. An entry is a regular expression on a function name (the function itself or a function of its inline
chain), the kinds of access (fill, copy, copy from read-only data) and the largest size. It never names an address,
so it survives code changes. The script prints the allowed sites, and offenders as `function: size, call site`. Only
the code that ZoltPerformanceTest links is checked.

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
# A scan that finds fewer library functions than this, or not SENTINEL, proves nothing (Phase 10: 1336 functions).
# SENTINEL is also the function whose instructions perf-budget counts.
MIN_FUNCS = 600
SENTINEL = "Physics.PhysicsSystem.PhysicsSystem.update"
# The scanner reads x86-64 code only
FORMAT = "elf64-x86-64"
# The functions of Jolt's 8 JPH_USE_FMADD sites (Zolt/Core/Core.zig use_fmadd): with --fma, every FMA instruction of
# the library must come from one of them (the function itself or a function of its inline chain)
FMA_SITES = re.compile(r"^Math\.(Vec3|Vec4|DVec3|Quat|Math)\.(\w+\.)?"
                       r"(fusedMultiplyAdd|differenceOfProducts|crossPrecise|mul|multiplyImaginary)$")

# The sites that may stay: (regular expression on a function name, what it may do, largest size in bytes, why).
# "fill" is a memset, "copy" a memcpy / memmove, "rodata" a copy from read-only data. The name is the function that
# holds the code, or a function of its inline chain.
ALLOW = [
    # Constraint creation builds the constraint parts in place from their default values
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.PathConstraint\.PathConstraintSettings,\.createConstraint\)\.",
     {"fill"}, 0x260, "PathConstraint creation"),
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.SwingTwistConstraint\.SwingTwistConstraintSettings,"
     r"\.createConstraint\)\.", {"fill", "copy"}, 0x3c0, "SwingTwistConstraint creation"),
    # (0x750 bytes with -Ddebug_renderer)
    (r"^Core\.Virtual\.Thunk\(Physics\.Constraints\.SixDOFConstraint\.SixDOFConstraintSettings,\.createConstraint\)\.",
     {"copy"}, 0x750, "SixDOFConstraint creation"),
    # Body creation copies the default Body (0x140 bytes, 0x180 in double precision)
    (r"^Physics\.Body\.BodyManager\.BodyManager\.allocateBody$", {"rodata"}, 0x180, "body creation"),
    # Once per update: the default PhysicsUpdateContext.Step (Jolt's mSteps.resize runs the Step constructor). About
    # 0.01% of the instructions of an update.
    (r"^Physics\.PhysicsSystem\.PhysicsSystem\.update$", {"rodata"}, 0x1ca8, "the steps of an update"),
    # Construction and teardown set the members to their default values
    (r"^Physics\.PhysicsSystem\.PhysicsSystem\.(init|deinit)$", {"rodata"}, 0x500, "PhysicsSystem init and deinit"),
    (r"^Physics\.Constraints\.ContactConstraintManager\.ContactConstraintManager\.(init|deinit)$", {"rodata"}, 0x128,
     "ContactConstraintManager init and deinit"),
    (r"^Core\.JobSystemThreadPool\.JobSystemThreadPool\.deinit$", {"rodata"}, 0x21c0, "JobSystemThreadPool teardown"),
    # Job system creation: the job list of each barrier
    (r"^Core\.JobSystemWithBarrier\.JobSystemWithBarrier\.init$", {"fill"}, 0x4001, "JobSystemWithBarrier creation"),
    # A random generator (std::mt19937) is created by value when a scene or hair settings are set up
    (r"^Core\.Mt19937\.init$", {"copy"}, 0x9c4, "Mt19937 creation"),
    # Debug drawing (only with -Ddebug_renderer, not timed). ConvexShape keys its supporting face cache with a
    # StaticArray(Vec3, 32).
    (r"[.,]draw\w*(\)\.call\d+)?$", {"fill", "copy", "rodata"}, 0x1000, "debug drawing"),
]

FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
FORMAT_RE = re.compile(r"\sfile format (\S+)$")
INLINED_RE = re.compile(r"^(\S.*)\(\):$")
# `inlined by <file>:<line> (<function>)`: the call site of the inlined function, the innermost first
INLINED_BY_RE = re.compile(r"^inlined by (\S+):(\d+)(?: \(discriminator \d+\))? \((.+)\)$")
LINE_RE = re.compile(r"^(\S+):(\d+)(?: \(discriminator \d+\))?$")
INSN_RE = re.compile(r"^\s+([0-9a-f]+):\s+(.*)$")
CALL_RE = re.compile(r"^(?:call|jmp)\w*\s+([0-9a-f]+) <([^>+]+)(?:\+0x[0-9a-f]+)?>")
MEM_FUNC_RE = re.compile(r"mem(set|cpy|move)")
REP_RE = re.compile(r"^rep\w*\s+(stos|movs)([bwlqd]?)\s*(?:%(\w+),)?")
IMM_RE = re.compile(r"^\$0x([0-9a-f]+)$")
COMMENT_ADDR_RE = re.compile(r"#\s*([0-9a-f]+)")
FMA_RE = re.compile(r"^vfn?m(add|sub)")
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


def allowed(names, kind, size):
    """The ALLOW entry that covers a site whose code belongs to the functions `names`, or None"""
    for pattern, kinds, limit, _ in ALLOW:
        if kind in kinds and size is not None and size <= limit and any(re.search(pattern, n) for n in names):
            return pattern
    return None


class Scan:
    """The result of a scan"""

    def __init__(self):
        self.format = None  # The file format that objdump prints
        self.offenders = []  # (function, size or None, what, call site, kind)
        self.allowed_sites = []  # (function, size, what, call site, kind, ALLOW pattern)
        self.funcs = set()  # The library functions (the names that start with one of PREFIXES)
        self.hosts = set()  # The other functions that hold inlined library code
        self.line_info = False
        self.fma = []  # The FMA instructions of the library code: (function, names of its inline chain, site, mnemonic)


def scan(lines, ro_ranges, syms, swap_range_of):
    """Scan the lines of an `objdump -d -l [--inlines]` listing. Returns a Scan."""
    result = Scan()
    func = None
    func_is_lib = False
    regs = {}
    where = "?"
    inlined = None
    chain = []  # The `inlined by` lines of the next instruction: (file, line, function)
    in_swap = False
    last_own_line = "?"
    swaps = {}

    def in_ro(address):
        return any(lo <= address < hi for lo, hi in ro_ranges)

    def report(owner, names, kind, size, what, site):
        pattern = allowed(names, kind, size)
        if pattern:
            result.allowed_sites.append((owner, size, what, site, kind, pattern))
        else:
            result.offenders.append((owner, size, what, site, kind))

    def check_copy(owner, names, kind, size, src, site):
        if size is None:
            return
        if kind != "memset" and src is not None and in_ro(src) and size > RODATA_LIMIT:
            name = symbol_at(syms, src) or f"0x{src:x}"
            report(owner, names, "rodata", size, f"copy from read-only {name}", site)
        elif size > FILL_COPY_LIMIT:
            report(owner, names, "fill" if kind == "memset" else "copy", size, kind, site)

    for line in lines:
        line = line.rstrip("\n")
        m = FUNC_RE.match(line)
        if m:
            func = m.group(2)
            func_is_lib = func.startswith(PREFIXES)
            if func_is_lib:
                result.funcs.add(func)
            regs = {}
            inlined = None
            chain = []
            in_swap = False
            where = last_own_line = "?"
            continue
        if func is None:
            m = FORMAT_RE.search(line)
            if m:
                result.format = m.group(1)
            continue
        m = INLINED_BY_RE.match(line)
        if m:
            chain.append((m.group(1), m.group(2), m.group(3)))
            continue
        m = LINE_RE.match(line)
        if m:
            result.line_info = True
            where = f"{m.group(1).removeprefix(ROOT)}:{m.group(2)}"
            rng = swap_range_of(m.group(1)) if m.group(1).endswith("std/mem.zig") else None
            in_swap = rng is not None and rng[0] <= int(m.group(2)) <= rng[1]
            if not in_swap and "/lib/std/" not in m.group(1):
                last_own_line = where
            chain = []
            continue
        m = INLINED_RE.match(line)
        if m:
            inlined = m.group(1)
            chain = []
            continue
        m = INSN_RE.match(line)
        if not m:
            continue
        address, text = m.group(1), m.group(2).strip()
        # The functions this instruction belongs to, the innermost first. It is library code when one of them is.
        names = [inlined or func] + [c[2] for c in chain] + [func]
        callers, chain = chain, []
        lib_names = [n for n in names if n.startswith(PREFIXES)]
        if lib_names:
            # Name the function itself, or the outermost library function that LLVM inlined into another function
            owner = func if func_is_lib else lib_names[-1]
            if not func_is_lib:
                result.hosts.add(func)
            notes = [n for n in (inlined,) if n and n != owner]
            if owner != func:
                notes.append(f"inlined into {func}")
            site = f"{where} ({''.join(n + ', ' for n in notes)}0x{address})"
            if in_swap:
                # The call site of std.mem.swap: the first caller outside the standard library
                own = next((f"{f.removeprefix(ROOT)}:{n}" for f, n, _ in callers if "/lib/std/" not in f),
                           last_own_line)
                s = swaps.setdefault(owner, [0, 0, own, address])
                s[0] += 1
                s[1] += bool(BYTE_STORE_RE.match(text))
        if lib_names and FMA_RE.match(text):
            result.fma.append((owner, names, site, text.split()[0]))
        m = CALL_RE.match(text)
        if m:
            if lib_names:
                k = MEM_FUNC_RE.search(m.group(2))
                if k:
                    check_copy(owner, names, "mem" + k.group(1), regs.get("rdx"), regs.get("rsi"), site)
                elif m.group(2).startswith("mem.swap"):
                    # A std.mem.swap that LLVM did not inline
                    result.offenders.append((owner, None, f"call of std.mem.swap {m.group(2)}", site, "swap"))
            regs = {}
            continue
        m = REP_RE.match(text)
        if m:
            # `rep stos %rax,%es:(%rdi)` (the width is the register) or `rep movsq %ds:(%rsi),%es:(%rdi)`
            count = regs.get("rcx")
            if count is not None and lib_names:
                if m.group(1) == "stos":
                    width = {"al": 1, "ax": 2, "eax": 4, "rax": 8}.get(m.group(3), 1)
                else:
                    width = {"b": 1, "w": 2, "l": 4, "d": 4, "q": 8}.get(m.group(2), 1)
                check_copy(owner, names, "memset" if m.group(1) == "stos" else "memcpy", count * width,
                           regs.get("rsi"), site)
            regs.pop("rcx", None)
            continue
        mnemonic, ops = operands(text)
        if mnemonic.startswith("call"):
            regs = {}
            continue
        if IMPLICIT_RDX.match(mnemonic):
            regs.pop("rdx", None)
        if IMPLICIT_RCX.match(mnemonic):
            regs.pop("rcx", None)
        if not ops or NO_WRITE.match(mnemonic):
            continue
        dest = ops[-1].lstrip("%")
        for reg, names_of_reg in REG_NAMES.items():
            if dest not in names_of_reg:
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
        result.offenders.append((f, None, f"std.mem.swap ({count} instructions, {byte_stores} byte stores)",
                                 f"{own} (0x{address})", "swap"))
    return result


def problems(result):
    """The reasons why a scan proves nothing (an empty list when it is valid)"""
    if result.format != FORMAT:
        return [f"the binary is {result.format or 'of an unknown format'}, but the scanner reads {FORMAT} code only"]
    found = []
    if len(result.funcs) < MIN_FUNCS or SENTINEL not in result.funcs:
        found.append(f"the scan found {len(result.funcs)} library functions (at least {MIN_FUNCS} expected) and "
                     f"{'not ' if SENTINEL not in result.funcs else ''}{SENTINEL}. Is the binary stripped, did the "
                     f"names change (check them with nm), or was {SENTINEL} inlined? perf-budget needs it too.")
    if ALLOW and not result.allowed_sites:
        found.append("no ALLOW entry matched: the scanner probably no longer sees the memset / memcpy calls")
    return found


def fma_offenders(result, fma):
    """The FMA instructions that are not allowed: all of them without -Dfma, else those outside FMA_SITES"""
    if not fma:
        return [(f, site, mnemonic) for f, _, site, mnemonic in result.fma]
    if not result.line_info:
        # Without the inline chains the site of an inlined FMA is unknown
        return []
    return [(f, site, mnemonic) for f, names, site, mnemonic in result.fma if not any(FMA_SITES.match(n) for n in names)]


def check(binary, fma=False):
    objdump = os.environ.get("OBJDUMP", "objdump")
    nm = os.environ.get("NM", "nm")
    cache = {}

    def swap_range_of(path):
        if path not in cache:
            cache[path] = swap_lines(path)
        return cache[path]

    ro_ranges, syms = read_only_ranges(binary, objdump), symbols(binary, nm)
    # The listing is large (about 200 MB with the inline chains): read it line by line
    with subprocess.Popen([objdump, "-d", "-l", "--inlines", "--no-show-raw-insn", binary], stdout=subprocess.PIPE,
                          text=True) as proc:
        result = scan(proc.stdout, ro_ranges, syms, swap_range_of)
    if proc.returncode != 0:
        print(f"codegen-check: {objdump} failed with exit code {proc.returncode}")
        return 1
    print(f"codegen-check: {len(result.funcs)} library functions ({', '.join(PREFIXES)}) and the library code "
          f"inlined into {len(result.hosts)} other functions, limits: fill / copy {FILL_COPY_LIMIT} B, copy from "
          f"read-only data {RODATA_LIMIT} B")
    if not result.line_info:
        print("codegen-check: the binary has no debug line info, so the check of the library code inlined into "
              "other functions and of std.mem.swap is skipped")
    elif not any(cache.values()):
        print("codegen-check: std/mem.zig `swap` not found, the std.mem.swap check is skipped")
    for f, size, what, site, _, _ in result.allowed_sites:
        print(f"  allowed: {f}: {size} (0x{size:x}) {what}, {site}")
    used = {pattern for _, _, _, _, _, pattern in result.allowed_sites}
    for pattern, _, _, why in ALLOW:
        if pattern not in used:
            print(f"  note: the allow-list entry for {why} matched nothing")
    failed = problems(result)
    if fma and not result.fma:
        failed.append("--fma: the library has no FMA instruction, so Jolt's JPH_USE_FMADD sites did not fuse "
                      "(was the binary built with -Dfma=true?)")
    if failed:
        print("codegen-check FAILED: the scan proves nothing:")
        for p in failed:
            print("  " + p)
        return 1
    bad_fma = fma_offenders(result, fma)
    if fma:
        print(f"codegen-check: {len(result.fma)} FMA instructions in the library code (-Dfma), "
              f"{len(result.fma) - len(bad_fma)} of them from Jolt's JPH_USE_FMADD sites"
              + ("" if result.line_info else " (no debug line info: their sites are not checked)"))
    if bad_fma:
        print(f"codegen-check FAILED: {len(bad_fma)} FMA instructions in library code"
              + (" outside Jolt's JPH_USE_FMADD sites:" if fma else " (none may exist without -Dfma):"))
        for f, site, mnemonic in bad_fma[:20]:
            print(f"  {f}: {mnemonic}, {site}")
        print("Never use @mulAdd outside Jolt's 8 JPH_USE_FMADD sites, and only behind Core.use_fmadd "
              "(Docs/Zolt/PortingGuide.md). Zig does not contract a * b + c by itself.")
    if result.offenders:
        print(f"codegen-check FAILED: {len(result.offenders)} large fills, copies or byte swaps in library code:")
        for f, size, what, site, _ in result.offenders:
            size_text = f"{size} (0x{size:x}) {what}" if size is not None else what
            print(f"  {f}: {size_text}, {site}")
        print("Construct large values in place and use Core.swap (Docs/Zolt/PortingGuide.md, \"Large values\"). "
              "Only setup-time code may go into ALLOW in tools/check_codegen.py.")
        return 1
    if bad_fma:
        return 1
    print("codegen-check passed")
    return 0


SELF_TEST_LISTING = """
zolt:     file format elf64-x86-64


Disassembly of section .text:

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
 1000211:\tcall   1100020 <mem.swap__anon_123>

0000000001000300 <PerformanceTest.main>:
PerformanceTest.main():
/z/ZoltPerformanceTest/PerformanceTest.zig:500
 1000300:\tmov    $0x100000,%edx
 1000305:\tcall   1100000 <compiler_rt.memset>
Core.JobSystemWithBarrier.JobSystemWithBarrier.BarrierImpl.init():
/z/Zolt/Core/JobSystemWithBarrier.zig:40
inlined by /z/Zolt/Core/JobSystemWithBarrier.zig:93 (Core.JobSystemWithBarrier.JobSystemWithBarrier.init)
inlined by /z/ZoltPerformanceTest/PerformanceTest.zig:510 (PerformanceTest.main)
 100030a:\tmov    $0x4001,%edx
inlined by /z/Zolt/Core/JobSystemWithBarrier.zig:93 (Core.JobSystemWithBarrier.JobSystemWithBarrier.init)
inlined by /z/ZoltPerformanceTest/PerformanceTest.zig:510 (PerformanceTest.main)
 100030f:\tcall   1100000 <compiler_rt.memset>

0000000001000400 <PerformanceTestScene.updateTestThunk>:
PerformanceTestScene.updateTestThunk():
/z/ZoltPerformanceTest/PerformanceTestScene.zig:106
 1000400:\tmov    $0x900,%edx
Physics.Character.CharacterVirtual.CharacterVirtual.extendedUpdate():
/z/Zolt/Physics/Character.zig:1700
inlined by /z/ZoltPerformanceTest/PerformanceTestScene.zig:107 (PerformanceTestScene.updateTestThunk)
 1000405:\tmov    $0x900,%edx
inlined by /z/ZoltPerformanceTest/PerformanceTestScene.zig:107 (PerformanceTestScene.updateTestThunk)
 100040a:\tcall   1100000 <compiler_rt.memset>
mem.swap__anon_9():
/z/lib/std/mem.zig:4
inlined by /z/Zolt/Physics/Character.zig:1710 (Physics.Character.CharacterVirtual.CharacterVirtual.moveShape)
inlined by /z/Zolt/Physics/Character.zig:1701 (Physics.Character.CharacterVirtual.CharacterVirtual.extendedUpdate)
inlined by /z/ZoltPerformanceTest/PerformanceTestScene.zig:107 (PerformanceTestScene.updateTestThunk)
 100040f:\tmov    %dl,(%rax)

0000000001000500 <Physics.Body.Body.updateRotation>:
/z/Zolt/Physics/Body/Body.zig:300
 1000500:\tvmulps %xmm1,%xmm0,%xmm0
Math.Quat.Quat.mul():
/z/Zolt/Math/Quat.zig:354
inlined by /z/Zolt/Physics/Body/Body.zig:301 (Physics.Body.Body.updateRotation)
 1000504:\tvfmadd213ps %xmm2,%xmm1,%xmm0
Physics.Body.Body.updateRotation():
/z/Zolt/Physics/Body/Body.zig:302
 1000509:\tvfnmadd231ps %xmm2,%xmm1,%xmm0
"""


def self_test():
    std_swap = {"/z/lib/std/mem.zig": (2, 5)}
    r = scan(SELF_TEST_LISTING.splitlines(), [(0x2000, 0x3000)], [(0x2000, "__anon_6"), (0x2100, "__anon_7")],
             lambda p: std_swap.get(p))
    got = sorted((f.rsplit(".", 1)[-1], size or 0, what.split(" (")[0]) for f, size, what, _, _ in r.offenders)
    want = sorted([("addPoint", 0x808, "memset"), ("addPoint", 0x9000 * 8, "memset"), ("addPoint", 0, "std.mem.swap"),
                   ("bar", 0x120, "copy from read-only __anon_7"),
                   ("bar", 0, "call of std.mem.swap mem.swap__anon_123"),
                   # Library code that LLVM inlined into a function of the test program
                   ("extendedUpdate", 0x900, "memset"), ("extendedUpdate", 0, "std.mem.swap")])
    assert got == want, got
    sites = {(f.rsplit(".", 1)[-1], kind): site for f, _, _, site, kind in r.offenders}
    assert sites[("extendedUpdate", "fill")] == \
        "/z/Zolt/Physics/Character.zig:1700 (inlined into PerformanceTestScene.updateTestThunk, " \
        "0x100040a)", sites
    # The site of an inlined std.mem.swap is its caller in the library
    assert sites[("extendedUpdate", "swap")] == "/z/Zolt/Physics/Character.zig:1710 (0x100040f)", sites
    # Allowed by the function name of the inline chain, not by the function that holds the code
    allowed_sites = [(f.rsplit(".", 1)[-1], s) for f, s, _, _, _, _ in r.allowed_sites]
    assert allowed_sites == [("deinit", 0x500), ("init", 0x4001)], allowed_sites
    # Two FMA instructions in the library: without -Dfma both fail, with -Dfma only the one outside Quat.mul fails
    assert [(f.rsplit(".", 1)[-1], m) for f, _, _, m in r.fma] == [("updateRotation", "vfmadd213ps"),
                                                                    ("updateRotation", "vfnmadd231ps")], r.fma
    assert len(fma_offenders(r, False)) == 2
    assert [m for _, _, m in fma_offenders(r, True)] == ["vfnmadd231ps"], fma_offenders(r, True)
    assert len(r.funcs) == 4 and r.hosts == {"PerformanceTest.main", "PerformanceTestScene.updateTestThunk"}
    assert r.line_info and r.format == FORMAT
    # A scan that proves nothing fails: too few library functions and no SENTINEL here
    assert len(problems(r)) == 1 and SENTINEL in problems(r)[0], problems(r)
    r.funcs |= {SENTINEL} | {f"Physics.F{i}" for i in range(MIN_FUNCS)}
    assert problems(r) == [], problems(r)
    r.allowed_sites = []
    assert len(problems(r)) == 1 and "ALLOW" in problems(r)[0], problems(r)
    # Another architecture: the x86-64 patterns would find nothing, so it fails
    aarch64 = scan(["zolt:     file format elf64-littleaarch64", "", "0000000001000000 <Physics.Foo.bar>:",
                    " 1000000:\tbl\t1100000 <memset>"], [], [], lambda p: None)
    assert aarch64.offenders == [] and "elf64-littleaarch64" in problems(aarch64)[0], problems(aarch64)
    print("check_codegen.py self-test passed")
    return 0


if __name__ == "__main__":
    if sys.argv[1:] == ["--self-test"]:
        sys.exit(self_test())
    argv = sys.argv[1:]
    use_fma = argv[:1] == ["--fma"]
    if use_fma:
        argv = argv[1:]
    if len(argv) != 1:
        print(__doc__)
        sys.exit(2)
    sys.exit(check(argv[0], use_fma))
