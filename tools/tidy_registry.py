#!/usr/bin/env python3
"""Sort and deduplicate the registry lists of the port.

`Zolt/zolt.zig` (re-exports and `source_files`) and `ZoltTests/unit_tests.zig` (test imports) are
the only files that parallel ports both modify. Git merges them with the `union` driver (see
.gitattributes), which keeps every added line but can duplicate lines and break the ordering.
Run this after merging branches:

    python3 tools/tidy_registry.py          # rewrite the files in place
    python3 tools/tidy_registry.py --check  # exit 1 if a file is not tidy (used by port_status.py --check)

Rules: within each block of consecutive registry lines (a block ends at a blank line or a comment),
lines are deduplicated and sorted by imported path (case insensitive), then by the line itself.
"""
import argparse
import os
import re
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
FILES = ["Zolt/zolt.zig", "ZoltTests/unit_tests.zig"]
REGISTRY_LINE = re.compile(r'^\s*(pub const \w+ = )?(_ = )?@import\("([^"]+)"\)')


def sort_key(line):
    m = REGISTRY_LINE.match(line)
    return (m.group(3).lower(), line.strip().lower())


def tidy(source):
    lines = source.split("\n")
    out = []
    block = []

    def flush():
        seen = set()
        unique = []
        for line in block:
            if line.strip() not in seen:
                seen.add(line.strip())
                unique.append(line)
        out.extend(sorted(unique, key=sort_key))
        block.clear()

    for line in lines:
        if REGISTRY_LINE.match(line):
            block.append(line)
        else:
            flush()
            out.append(line)
    flush()
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only check, exit 1 if a file would change")
    args = ap.parse_args()

    untidy = []
    for rel_path in FILES:
        path = os.path.join(ROOT, rel_path)
        with open(path, encoding="utf-8") as f:
            source = f.read()
        result = tidy(source)
        if result != source:
            untidy.append(rel_path)
            if not args.check:
                with open(path, "w", encoding="utf-8") as f:
                    f.write(result)
                print(f"Tidied {rel_path}")
    if args.check and untidy:
        for rel_path in untidy:
            print(f"{rel_path} has unsorted or duplicate registry lines, run: python3 tools/tidy_registry.py", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
