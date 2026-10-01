# ZoltPhysics

This repository ports the Jolt Physics engine from C++ to **Zig 0.16** ("Zolt"). It is a fork of
JoltPhysics: the C++ sources stay in place as the reference, the Zig port lives next to them.

## Layout

| Path                 | What                                                                  |
|----------------------|-----------------------------------------------------------------------|
| `Jolt/`              | C++ reference implementation (do not modify, except when syncing with upstream) |
| `UnitTests/`         | C++ unit tests (reference for `ZoltTests/`)                           |
| `Zolt/`              | The Zig port, mirrors `Jolt/` file by file. Module root: `Zolt/zolt.zig` |
| `ZoltTests/`         | Port of `UnitTests/`, root: `ZoltTests/unit_tests.zig`                |
| `Docs/Zolt/`         | Porting docs: **read `PortingGuide.md` before writing any Zig code**  |
| `tools/`             | Porting helpers (`strip_isa.py`, `port_status.py`)                    |

## Commands

```sh
zig build test                              # all tests (library inline tests + ported unit tests)
zig build test -Ddouble_precision=true      # same with JPH_DOUBLE_PRECISION semantics
zig build test -Dtest-filter=TestVec3Cross  # run matching tests only
zig build check                             # compile only (fast)
zig fmt Zolt ZoltTests build.zig            # format (CI checks this)
python3 tools/strip_isa.py -D JPH_CROSS_PLATFORM_DETERMINISTIC Jolt/Math/Vec3.inl   # C++ without SIMD branches
python3 tools/port_status.py --write        # regenerate Docs/Zolt/Progress.md
```

If `zig` is missing (fresh cloud container), the SessionStart hook installs it; manually:
`bash tools/install_zig.sh`.

## Rules (details in Docs/Zolt/PortingGuide.md)

- **Bit exact port.** Zolt must produce the same results as Jolt built with
  `CROSS_PLATFORM_DETERMINISTIC=ON`. Never reorder floating point operations, never use FMA
  (`@mulAdd`), `std.math` trig or `@min`/`@max` on floats (use `Math/Math.zig` helpers).
- One Zig file per Jolt header (`.h` + `.inl` + `.cpp` merged), same directory and name.
  Every file starts with `//! Port of: <C++ files>` and `//! Status: complete|partial|stub`.
- Register new files in `source_files` in `Zolt/zolt.zig` (re-export public types there) and
  new test files in `ZoltTests/unit_tests.zig`.
- Naming: `GetFoo` → `getFoo`, `sFoo` → `foo`, `mFoo` → `foo`, `inFoo` → `foo`,
  `EFoo::Bar` → `Foo.bar`. Operators become methods (`add`, `sub`, `mul`, `mulScalar`, `eql`, ...).
- Keep Jolt's doc comments and implementation comments.
- Explicit `std.mem.Allocator` everywhere, no globals. Blocking sync primitives need `std.Io`.
- Port the matching Jolt unit tests with the same names and values.
- Before finishing: `zig build test`, `zig build test -Ddouble_precision=true`,
  `zig fmt --check Zolt ZoltTests build.zig`, `python3 tools/port_status.py --write`.

## Plan and status

- `Docs/Zolt/Roadmap.md`: phases in dependency order, milestones, the determinism acceptance test.
- `Docs/Zolt/Progress.md`: generated per-file status (do not edit by hand).
- The port tracks Jolt v5.6.1 at commit `5830c34` (see Roadmap for syncing with upstream).
