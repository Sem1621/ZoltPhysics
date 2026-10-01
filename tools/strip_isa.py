#!/usr/bin/env python3
"""Strip ISA-specific preprocessor branches from a Jolt C++ file.

Jolt's math code has one branch per instruction set (SSE/AVX/NEON/RVV/...) plus a
portable scalar fallback. The fallback defines the canonical semantics, and that is
what Zolt ports (expressed with Zig @Vector ops). This tool resolves every #if/#elif
whose outcome is decidable when all ISA macros are treated as undefined, and leaves
every other conditional (JPH_DOUBLE_PRECISION, JPH_DEBUG_RENDERER, ...) untouched.

Usage:
    python3 tools/strip_isa.py Jolt/Math/Vec4.inl            # print stripped file
    python3 tools/strip_isa.py -D JPH_FLOATING_POINT_EXCEPTIONS_ENABLED Jolt/Math/Vec3.h
    python3 tools/strip_isa.py -U JPH_DEBUG_RENDERER Jolt/Physics/Body/Body.h
"""
import argparse
import re
import sys

# Treated as undefined unless overridden with -D
ISA_MACROS = {
    "JPH_USE_SSE", "JPH_USE_SSE4_1", "JPH_USE_SSE4_2", "JPH_USE_AVX", "JPH_USE_AVX2",
    "JPH_USE_AVX512", "JPH_USE_NEON", "JPH_USE_RVV", "JPH_USE_F16C", "JPH_USE_FMADD",
    "JPH_USE_LZCNT", "JPH_USE_TZCNT", "JPH_COMPILER_MSVC", "JPH_COMPILER_MINGW",
    "JPH_PLATFORM_WASM", "JPH_CPU_ARM", "JPH_CPU_X86", "JPH_CPU_WASM", "JPH_CPU_E2K",
    "JPH_CPU_RISCV", "JPH_CPU_PPC", "JPH_CPU_LOONGARCH",
}

TOKEN_RE = re.compile(r"\s*(defined|\|\||&&|!|\(|\)|[A-Za-z_][A-Za-z_0-9]*|\d+|==|!=|>=|<=|>|<|.)")


class Unknown(Exception):
    pass


def evaluate(expr, defined, undefined):
    """Return True/False if decidable, None otherwise (tri-state)."""
    tokens = [t for t in TOKEN_RE.findall(expr.split("//")[0]) if t.strip()]
    pos = 0

    def peek():
        return tokens[pos] if pos < len(tokens) else None

    def take():
        nonlocal pos
        tok = tokens[pos]
        pos += 1
        return tok

    def primary():
        tok = take()
        if tok == "!":
            v = primary()
            return None if v is None else (not v)
        if tok == "(":
            v = or_expr()
            if peek() == ")":
                take()
            return v
        if tok == "defined":
            paren = peek() == "("
            if paren:
                take()
            name = take()
            if paren and peek() == ")":
                take()
            if name in defined:
                return True
            if name in undefined:
                return False
            return None
        if tok in undefined:
            return False
        # Comparisons / arbitrary identifiers / numbers: undecidable
        while peek() in ("==", "!=", ">=", "<=", ">", "<"):
            take()
            take()
        return None

    def and_expr():
        v = primary()
        while peek() == "&&":
            take()
            r = primary()
            if v is False or r is False:
                v = False
            elif v is None or r is None:
                v = None
            else:
                v = True
        return v

    def or_expr():
        v = and_expr()
        while peek() == "||":
            take()
            r = and_expr()
            if v is True or r is True:
                v = True
            elif v is None or r is None:
                v = None
            else:
                v = False
        return v

    try:
        return or_expr()
    except IndexError:
        return None


DIRECTIVE_RE = re.compile(r"^(\s*)#\s*(if|ifdef|ifndef|elif|else|endif)\b(.*)$")


def process(lines, defined, undefined):
    out = []
    # Stack frames: dict(state, emitting, any_true, emitted_directive, parent_emitting)
    stack = []

    def emitting():
        return all(f["emitting"] for f in stack)

    for line in lines:
        m = DIRECTIVE_RE.match(line)
        if not m:
            if emitting():
                out.append(line)
            continue
        indent, kind, rest = m.groups()
        if kind in ("if", "ifdef", "ifndef"):
            parent = emitting()
            if kind == "if":
                cond = evaluate(rest, defined, undefined)
            else:
                name = rest.strip().split()[0]
                cond = True if name in defined else False if name in undefined else None
                if kind == "ifndef" and cond is not None:
                    cond = not cond
            frame = {"decided": cond is True, "emitting": cond is not False, "open": cond is None}
            stack.append(frame)
            if parent and cond is None:
                out.append(line)
        elif kind == "elif":
            frame = stack[-1]
            if frame["decided"]:
                frame["emitting"] = False
                continue
            cond = evaluate(rest, defined, undefined)
            parent = all(f["emitting"] for f in stack[:-1])
            if cond is True:
                frame["decided"] = True
                frame["emitting"] = True
                if frame["open"] and parent:
                    out.append(f"{indent}#else\n")
            elif cond is False:
                frame["emitting"] = False
            else:
                frame["emitting"] = True
                if parent:
                    out.append(line if frame["open"] else f"{indent}#if{rest}\n")
                frame["open"] = True
        elif kind == "else":
            frame = stack[-1]
            parent = all(f["emitting"] for f in stack[:-1])
            if frame["decided"]:
                frame["emitting"] = False
            else:
                frame["emitting"] = True
                if frame["open"] and parent:
                    out.append(line)
        elif kind == "endif":
            frame = stack.pop()
            if frame["open"] and emitting():
                out.append(line)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("-D", action="append", default=[], help="treat macro as defined")
    ap.add_argument("-U", action="append", default=[], help="treat macro as undefined")
    ap.add_argument("--keep-blank", action="store_true", help="keep consecutive blank lines")
    args = ap.parse_args()
    defined = set(args.D)
    undefined = (ISA_MACROS | set(args.U)) - defined
    with open(args.file, encoding="utf-8") as f:
        lines = f.readlines()
    out = process(lines, defined, undefined)
    prev_blank = False
    for line in out:
        blank = not line.strip()
        if blank and prev_blank and not args.keep_blank:
            continue
        prev_blank = blank
        sys.stdout.write(line)


if __name__ == "__main__":
    main()
