#!/usr/bin/env python3
"""Generate and check the RTTI declarations of Zolt (Docs/Zolt/SerializationArchitecture.md, S2, S5 and S18).

Jolt declares the serializable classes in the sCreateRTTI blocks of its .cpp files:

    JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SphereShapeSettings)
    {
        JPH_ADD_BASE_CLASS(SphereShapeSettings, ConvexShapeSettings)
        JPH_ADD_ATTRIBUTE(SphereShapeSettings, mRadius)
    }

Zolt declares the same class next to its fields, with `pub const rtti = RTTI.declare(@This(), .{ ... })`.

Usage:
    python3 tools/gen_serializable.py --list     # print Jolt's classes (kind, bases, number of attributes)
    python3 tools/gen_serializable.py --write    # add a pending declaration to every class that has none
    python3 tools/gen_serializable.py --check    # compare the Zolt declarations with Jolt's blocks
    python3 tools/gen_serializable.py --check --complete   # also fail while a declaration is pending (CI)

`--write` gives a class without a declaration Jolt's name, kind and bases, and the marker
`pub const serialization_pending = true;` (S18). A group then adds the attributes and removes the marker.

`--check` compares every declaration with Jolt's block: the name, the kind (from the IMPLEMENT macro), the bases (in
order) and the attributes (in order: the stream name, alias or not, enum or not, and the member path of an alias). A
pending declaration is only checked for its name, kind and bases. port_status.py --check runs this check with
`--complete`: Phase 8 is done, so a pending declaration fails CI.

The classes of Phase 9 (Physics/SoftBody, Physics/Hair) are skipped (S17). So is the attribute "mSoftBodies" of
PhysicsScene: Zolt declares it in Phase 9 (S17).
"""
import argparse
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# Phase 9 (SerializationArchitecture.md S17)
EXCLUDED_DIRS = ("Jolt/Physics/SoftBody/", "Jolt/Physics/Hair/")

# Attributes of Phase 9 classes in the classes of Phase 8 (class, stream name): Zolt declares them in Phase 9 (S17)
EXCLUDED_ATTRIBUTES = {("PhysicsScene", "mSoftBodies")}

# Classes that Jolt declares outside the class: Zolt declares them in this file (TypeDeclarations.zig)
OUTSIDE_FILE = "Zolt/ObjectStream/TypeDeclarations.zig"

KIND = {
    "NON_VIRTUAL": "non_virtual",
    "OUTSIDE_CLASS": "non_virtual",
    "VIRTUAL": "virtual",
    "VIRTUAL_BASE": "virtual",
    "ABSTRACT": "abstract",
    "ABSTRACT_BASE": "abstract",
}

# The attribute macros and the Zolt function that each one becomes (`.attrAs` and `.custom` are `.attr`)
ATTRIBUTE_MACROS = {
    "ATTRIBUTE": "attr",
    "ENUM_ATTRIBUTE": "enumAttr",
    "ATTRIBUTE_WITH_ALIAS": "alias",
    "ENUM_ATTRIBUTE_WITH_ALIAS": "enumAlias",
}
ZOLT_ATTRIBUTE = {"attr": "attr", "attrAs": "attr", "custom": "attr", "enumAttr": "enumAttr", "alias": "alias", "enumAlias": "enumAlias"}

PENDING_LINE = "pub const serialization_pending = true;"
PENDING_COMMENT = "/// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)"


def rel(path):
    return os.path.relpath(path, ROOT).replace(os.sep, "/")


def read(path):
    with open(os.path.join(ROOT, path), encoding="utf-8") as f:
        return f.read()


class JoltClass:
    def __init__(self, cpp, name, implement, body):
        self.cpp = cpp
        self.name = name
        self.implement = implement
        self.kind = KIND[implement]
        self.short = name.split("::")[-1]
        self.outer = name.split("::")[:-1]
        self.bases = re.findall(r"JPH_ADD_BASE_CLASS\(\s*[\w:]+\s*,\s*([\w:]+)\s*\)", body)
        self.attributes = []
        for macro, args in re.findall(r"JPH_ADD_(ENUM_ATTRIBUTE_WITH_ALIAS|ATTRIBUTE_WITH_ALIAS|ENUM_ATTRIBUTE|ATTRIBUTE)\(([^)]*)\)", body):
            parts = [p.strip() for p in args.split(",")]
            function = ATTRIBUTE_MACROS[macro]
            member = parts[1].split(".")
            stream_name = parts[2].strip('"') if function in ("alias", "enumAlias") else parts[1]
            if (name, stream_name) in EXCLUDED_ATTRIBUTES:
                continue
            self.attributes.append((function, stream_name, member))
        self.declare = None
        if implement == "OUTSIDE_CLASS":
            self.zig = OUTSIDE_FILE
        else:
            self.zig = "Zolt/" + cpp[len("Jolt/"):-len(".cpp")] + ".zig"


def jolt_classes():
    """Every sCreateRTTI block of Jolt (without Phase 9), in file order"""
    block = re.compile(r"^JPH_IMPLEMENT_SERIALIZABLE_(\w+)\(([^)]*)\)\s*\n\{(.*?)^\}", re.S | re.M)
    declare = re.compile(r"JPH_DECLARE_SERIALIZABLE_(\w+)\(\s*JPH_EXPORT\s*,\s*(\w+)\s*\)")
    classes = []
    for dirpath, _, filenames in sorted(os.walk(os.path.join(ROOT, "Jolt"))):
        for name in sorted(filenames):
            if not name.endswith(".cpp"):
                continue
            cpp = rel(os.path.join(dirpath, name))
            if cpp.startswith(EXCLUDED_DIRS):
                continue
            source = read(cpp)
            found = [JoltClass(cpp, m.group(2).strip(), m.group(1), m.group(3)) for m in block.finditer(source)]
            if not found:
                continue
            header = cpp[:-len(".cpp")] + ".h"
            declared = dict((short, macro) for macro, short in declare.findall(read(header))) if os.path.exists(os.path.join(ROOT, header)) else {}
            for c in found:
                c.declare = declared.get(c.short)
            classes.extend(found)
    return classes


def strip_comments(text):
    """Remove `//` comments (the declarations hold no string with `//`)"""
    return "\n".join(line.split("//", 1)[0] for line in text.split("\n"))


def matching_paren(text, start):
    """Index of the parenthesis that closes the one at `start`"""
    depth = 0
    in_string = False
    i = start
    while i < len(text):
        c = text[i]
        if in_string:
            if c == "\\":
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
        elif c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("unbalanced parentheses")


def field(text, key):
    """The value of `.key = value` in a declaration (up to the comma at depth 0), None when missing"""
    m = re.search(r"\.%s\s*=\s*" % key, text)
    if not m:
        return None
    i = m.end()
    if text[i] == '"':
        return text[i + 1:text.index('"', i + 1)]
    j = i
    depth = 0
    while j < len(text):
        c = text[j]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            if depth == 0:
                break
            depth -= 1
        elif c == "," and depth == 0:
            break
        j += 1
    return text[i:j].strip()


class ZoltDeclaration:
    def __init__(self, path, line, text, pending):
        self.path = path
        self.line = line
        body = strip_comments(text)
        self.name = field(body, "name")
        kind = field(body, "kind")
        self.kind = kind.lstrip(".") if kind else None
        bases = field(body, "bases")
        self.bases = [b.strip().split(".")[-1] for b in bases.strip("&.{} \n").split(",") if b.strip()] if bases else []
        self.renamed = field(body, "renamed") == "true"
        self.attributes = []
        attributes = field(body, "attributes")
        if attributes:
            for function, name, member in re.findall(r"\.(attr|attrAs|custom|enumAttr|alias|enumAlias)\(\s*\"([^\"]*)\"\s*,\s*(\.\{[^}]*\}|\.?[\w.]+)", attributes):
                path = [p.strip().lstrip(".") for p in member.strip(".{} ").split(",")] if function != "custom" else []
                self.attributes.append((ZOLT_ATTRIBUTE[function], name, path, function))
        self.pending = pending


def zolt_declarations():
    """Every `RTTI.declare(` call in Zolt/ with the pending marker that follows it"""
    declarations = []
    for dirpath, _, filenames in sorted(os.walk(os.path.join(ROOT, "Zolt"))):
        for name in sorted(filenames):
            if not name.endswith(".zig"):
                continue
            path = rel(os.path.join(dirpath, name))
            source = read(path)
            for m in re.finditer(r"RTTI\.declare\(", source):
                if source[:m.start()].rstrip().endswith(("fn declare", "//")) or "\n" not in source[:m.start()]:
                    continue
                line_start = source.rfind("\n", 0, m.start()) + 1
                if source[line_start:m.start()].lstrip().startswith(("//", "///")):
                    continue
                end = matching_paren(source, m.end() - 1)
                rest = source[end + 1:].split("\n")[1:]
                following = [line.strip() for line in rest if line.strip() and not line.strip().startswith("//")]
                pending = bool(following) and following[0] == PENDING_LINE
                line = source.count("\n", 0, m.start()) + 1
                declarations.append(ZoltDeclaration(path, line, source[m.end():end], pending))
    return declarations


def same_name(jolt, zig):
    """SerializableAttribute.zig's sameName: without the `m` prefix, the underscores and the case"""
    if len(jolt) > 1 and jolt[0] == "m" and jolt[1].isupper():
        jolt = jolt[1:]
    return jolt.replace("_", "").lower() == zig.replace("_", "").lower()


def compare(jolt, decl):
    """The differences between Jolt's block and a Zolt declaration"""
    problems = []
    where = f"{decl.path}:{decl.line}: {jolt.name}"
    if decl.path != jolt.zig:
        problems.append(f"{where}: declared in {decl.path}, expected in {jolt.zig} (the port of {jolt.cpp})")
    if decl.kind != jolt.kind:
        problems.append(f"{where}: kind .{decl.kind}, Jolt's JPH_IMPLEMENT_SERIALIZABLE_{jolt.implement} is .{jolt.kind}")
    jolt_bases = [b.split("::")[-1] for b in jolt.bases]
    if decl.bases != jolt_bases:
        problems.append(f"{where}: bases {decl.bases}, Jolt's JPH_ADD_BASE_CLASS are {jolt_bases}")
    if decl.renamed:
        problems.append(f"{where}: `.renamed = true` (JPH_RENAME_CLASS is unused in Jolt)")
    if decl.pending:
        return problems
    zolt_attributes = [(a[0], a[1]) for a in decl.attributes]
    jolt_attributes = [(a[0], a[1]) for a in jolt.attributes]
    if zolt_attributes != jolt_attributes:
        problems.append(f"{where}: attributes differ from Jolt's sCreateRTTI")
        for i in range(max(len(zolt_attributes), len(jolt_attributes))):
            z = zolt_attributes[i] if i < len(zolt_attributes) else None
            j = jolt_attributes[i] if i < len(jolt_attributes) else None
            if z != j:
                z_text = f".{z[0]}(\"{z[1]}\")" if z else "(none)"
                j_text = f".{j[0]}(\"{j[1]}\")" if j else "(none)"
                problems.append(f"    attribute {i}: Zolt {z_text}, Jolt {j_text}")
        return problems
    for (function, name, path, zolt_function), (_, _, member) in zip(decl.attributes, jolt.attributes):
        if zolt_function == "custom":
            continue
        if len(path) != len(member) or not all(same_name(j, z) for j, z in zip(member, path)):
            problems.append(f"{where}: attribute \"{name}\" is the member .{{{', '.join(path)}}}, Jolt's member is {'.'.join(member)}")
    return problems


def check(require_complete):
    classes = jolt_classes()
    declarations = zolt_declarations()
    by_name = {}
    for d in declarations:
        by_name.setdefault(d.name, []).append(d)
    problems = []
    pending = []
    declared = 0
    for c in classes:
        found = by_name.get(c.name, [])
        if not found:
            problems.append(f"{c.zig}: {c.name} has no declaration (run: python3 tools/gen_serializable.py --write)")
            continue
        if len(found) > 1:
            problems.append(f"{c.name} is declared {len(found)} times: " + ", ".join(f"{d.path}:{d.line}" for d in found))
        problems.extend(compare(c, found[0]))
        declared += 1
        if found[0].pending:
            pending.append(c.name)

    # Every marker belongs to a declaration of a Jolt class
    known = {c.name for c in classes}
    marked = sum(1 for d in declarations if d.pending and d.name in known)
    for dirpath, _, filenames in sorted(os.walk(os.path.join(ROOT, "Zolt"))):
        for name in sorted(filenames):
            if name.endswith(".zig"):
                path = rel(os.path.join(dirpath, name))
                count = sum(1 for line in read(path).split("\n") if line.strip() == PENDING_LINE)
                marked -= count
                if count and not any(d.path == path and d.pending for d in declarations):
                    problems.append(f"{path}: `{PENDING_LINE}` does not follow a declaration")
    if marked != 0 and not problems:
        problems.append("a `serialization_pending` marker does not directly follow its declaration")
    if require_complete and pending:
        problems.append(f"{len(pending)} declarations are pending: " + ", ".join(pending))

    for p in problems:
        print(f"gen_serializable: {p}", file=sys.stderr)
    print(f"gen_serializable: {len(classes)} classes, {declared - len(pending)} complete, {len(pending)} pending, {len(classes) - declared} missing")
    return not problems


def find_struct(lines, c):
    """Index of the line that opens the struct of class `c` (nested classes inside their outer struct)"""
    indent = ""
    start = 0
    end = len(lines)
    for part in c.outer + [c.short]:
        pattern = re.compile(r"^%spub const %s = (?:extern )?struct \{$" % (re.escape(indent), re.escape(part)))
        index = next((i for i in range(start, end) if pattern.match(lines[i])), None)
        if index is None:
            return None
        start = index + 1
        end = next((i for i in range(start, len(lines)) if lines[i] == indent + "};"), len(lines))
        indent += "    "
    return start - 1


def import_path(zig, target):
    """Relative import path from file `zig` to `target` (both relative to the repository root)"""
    return os.path.relpath(os.path.join(ROOT, target), os.path.dirname(os.path.join(ROOT, zig))).replace(os.sep, "/")


def declares(source, identifier):
    return re.search(r"^\s*(?:pub )?const %s\b" % re.escape(identifier), source, re.M) is not None


def add_import(lines, zig, identifier, target, decl):
    """Add `const identifier = @import(...).decl;` after the import of std"""
    text = f'const {identifier} = @import("{import_path(zig, target)}").{decl};'
    at = next((i for i, line in enumerate(lines) if line.startswith('const std = @import("std");')), None)
    if at is None:
        at = next(i for i, line in enumerate(lines) if not line.startswith("//!") and line.strip()) - 1
    # After the Allocator alias and the RTTI import when they follow std
    while at + 1 < len(lines) and lines[at + 1].startswith(("const Allocator = ", "const RTTI = ")):
        at += 1
    lines.insert(at + 1, text)


def declaration_lines(c, indent):
    declare = f"JPH_DECLARE_SERIALIZABLE_{c.declare} / " if c.declare else ""
    out = [
        f"{indent}/// {declare}JPH_IMPLEMENT_SERIALIZABLE_{c.implement}({c.name})",
        f"{indent}pub const rtti = RTTI.declare(@This(), .{{",
        f'{indent}    .name = "{c.name}",',
        f"{indent}    .kind = .{c.kind},",
    ]
    bases = [b.split("::")[-1] for b in c.bases]
    if len(bases) == 1:
        out.append(f"{indent}    .bases = &.{{{bases[0]}}},")
    elif bases:
        out.append(f"{indent}    .bases = &.{{ {', '.join(bases)} }},")
    out.append(f"{indent}}});")
    out.append(f"{indent}{PENDING_COMMENT}")
    out.append(f"{indent}{PENDING_LINE}")
    return out


def write():
    classes = jolt_classes()
    existing = {d.name for d in zolt_declarations()}
    by_file = {}
    for c in classes:
        if c.name not in existing:
            by_file.setdefault(c.zig, []).append(c)
    failed = False
    for zig, todo in sorted(by_file.items()):
        lines = read(zig).split("\n")
        for c in todo:
            index = find_struct(lines, c)
            if index is None:
                print(f"gen_serializable: {zig}: no struct for {c.name}", file=sys.stderr)
                failed = True
                continue
            indent = re.match(r"^(\s*)", lines[index]).group(1) + "    "
            # Replace the TODO line of the declaration (Jolt's DECLARE macro) right after the struct opening
            first = index + 1
            if lines[first].strip().startswith("// TODO(serialization): JPH_DECLARE_SERIALIZABLE"):
                del lines[first]
            new = declaration_lines(c, indent)
            if lines[first].strip():
                new.append("")
            lines[first:first] = new
        source = "\n".join(lines)
        added = []
        if not declares(source, "RTTI"):
            add_import(lines, zig, "RTTI", "Zolt/Core/RTTI.zig", "RTTI")
            added.append("RTTI")
        for c in todo:
            for base in c.bases:
                base = base.split("::")[-1]
                if base == "SerializableObject" and not declares("\n".join(lines), base):
                    add_import(lines, zig, base, "Zolt/ObjectStream/SerializableObject.zig", base)
                    added.append(base)
                elif not declares("\n".join(lines), base):
                    print(f"gen_serializable: {zig}: {c.name} has the base {base}, which the file does not declare", file=sys.stderr)
                    failed = True
        with open(os.path.join(ROOT, zig), "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        print(f"{zig}: {', '.join(c.name for c in todo)}" + (f" (imports {', '.join(added)})" if added else ""))
    return not failed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="print Jolt's serializable classes")
    ap.add_argument("--write", action="store_true", help="add a pending declaration to every class that has none")
    ap.add_argument("--check", action="store_true", help="compare the Zolt declarations with Jolt's blocks")
    ap.add_argument("--complete", action="store_true", help="with --check: fail while a declaration is pending")
    args = ap.parse_args()
    if args.list:
        for c in jolt_classes():
            print(f"{c.name:<45} .{c.kind:<12} bases={','.join(c.bases) or '-':<30} {len(c.attributes):>2} attributes  {c.zig}")
    if args.write and not write():
        sys.exit(1)
    if args.check and not check(args.complete):
        sys.exit(1)
    if not (args.list or args.write or args.check):
        ap.print_help()


if __name__ == "__main__":
    main()
