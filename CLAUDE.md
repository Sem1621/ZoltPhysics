# ZoltPhysics

This repository ports the Jolt Physics engine from C++ to **Zig 0.16** ("Zolt"). It is a fork of
JoltPhysics: the C++ sources stay in place as the reference, the Zig port lives next to them.

## Layout

| Path                 | What                                                                  |
|----------------------|-----------------------------------------------------------------------|
| `Jolt/`              | C++ reference implementation (do not modify, except when syncing with upstream) |
| `UnitTests/`         | C++ unit tests (reference for `ZoltTests/`)                           |
| `Zolt/`              | The Zig port, mirrors `Jolt/` file by file. Module root: `Zolt/zolt.zig` |
| `ZoltTests/`         | Port of all of `UnitTests/` (milestone M10), root: `ZoltTests/unit_tests.zig` |
| `ZoltParity/`        | Bit-for-bit comparison of Zolt with the C++ library (`zig build parity`) |
| `ZoltHelloWorld/`    | Port of `HelloWorld/HelloWorld.cpp` (milestone M5)                      |
| `ZoltPerformanceTest/` | Port of `PerformanceTest/` with Jolt's determinism hashes (milestones M6, M8 and M9); `-r` records Jolt's `.jor` debug renderer stream, only in a build with `-Ddebug_renderer=true` (without it `-r` is ignored) |
| `Docs/Zolt/`         | Porting docs: **read `PortingGuide.md` before writing any Zig code**  |
| `tools/`             | Porting helpers (`strip_isa.py`, `port_status.py`, `gen_serializable.py`, `gen_jolt_sources.py`, `tidy_registry.py`, `install_zig.sh`) |

## Commands

```sh
zig build test                              # all tests (library inline tests + ported unit tests)
zig build test -Ddouble_precision=true      # same with JPH_DOUBLE_PRECISION semantics
zig build test -Ddebug_renderer=true        # same with JPH_DEBUG_RENDERER: the draw code is compiled in
zig build test -Dtest-filter=TestVec3Cross  # run matching tests only
zig build check                             # compile only (fast)
zig build check -Ddebug_renderer=true       # compile the draw code too (it is type checked only with the option)
zig build parity-check                      # compile every parity test and the C++ reference without running them
zig build parity-check -Ddebug_renderer=true  # the same with the draw code and a C++ reference with JPH_DEBUG_RENDERER
zig build parity                            # compare with the C++ Jolt library bit for bit (first build ~4 min)
zig build parity -Ddebug_renderer=true      # the same with the draw code (files with JPH_DEBUG_RENDERER code)
zig build parity -Dreference_asserts=true -Dtest-filter='.SceneTest("<scene>"['  # a new scene vs an asserting Jolt build
zig build hello-world-check                 # C++ HelloWorld and its port must print the same bytes
zig build performance-test -Doptimize=ReleaseFast -- -q=LinearCast -t=max -s=Pyramid -validate_hash=0x74d0118836ac0892
zig build performance-test -Doptimize=ReleaseFast -- -q=LinearCast -t=max -s=Ragdoll -validate_hash=0x3be6ea76c07a2ac8  # M9 (double: 0x8dd6bc501859a228)
zig build performance-test -Doptimize=ReleaseFast -- -s=Pyramid -t=1 -phase_times  # also print the time of the scene and the physics updates
zig build performance-test -Doptimize=ReleaseFast -Djob_profile=true -- -s=Pyramid -t=2 -p  # job timeline: <scene>_<tag>.json (Chrome trace) + busy/idle per thread
zig build test -Djob_profile=true           # tests with the job profiler compiled in (Zolt/Core/JobProfiler.zig, off by default: zero cost)
zig build jolt-performance-test -- <args>   # the C++ PerformanceTest, built like the parity reference (the oracle)
zig build debug-renderer-check -Ddebug_renderer=true  # the .jor bytes of PerformanceTest -r and ZoltPerformanceTest -r must be equal
zig build codegen-check                     # fail on a large fill or copy or a std.mem.swap in the library code (ReleaseFast, x86-64)
zig build perf-budget                       # callgrind instructions of the physics updates vs C++, fail over budget (needs valgrind)
zig fmt Zolt ZoltTests ZoltParity ZoltHelloWorld ZoltPerformanceTest build.zig build.zig.zon  # format (CI checks this)
python3 tools/strip_isa.py -D JPH_CROSS_PLATFORM_DETERMINISTIC Jolt/Math/Vec3.inl   # C++ without SIMD branches
python3 tools/port_status.py --write        # regenerate Docs/Zolt/Progress.md
python3 tools/port_status.py --next         # what can be ported now (dependencies done)
python3 tools/port_status.py --check        # CI: Progress.md is up to date, valid headers, no pending declaration, M10 holds, no TODO(debug_renderer)
python3 tools/port_status.py --check-m10    # milestone M10: every UnitTests/ unit and TEST_CASE is ported
python3 tools/gen_serializable.py --check [--complete]  # the RTTI declarations against Jolt's sCreateRTTI blocks
python3 tools/tidy_registry.py              # sort/dedupe registry files after merging branches
python3 tools/tidy_registry.py --check      # fail if a registry file is not tidy (part of port_status.py --check)
```

If `zig` is missing (fresh cloud container), the SessionStart hook installs it; manually:
`bash tools/install_zig.sh`.

## Rules (details in Docs/Zolt/PortingGuide.md)

- **Bit exact port.** Zolt must produce the same results as Jolt built with
  `CROSS_PLATFORM_DETERMINISTIC=ON`. Never reorder floating point operations, never use FMA
  (`@mulAdd`), `std.math` trig or `@min`/`@max` on floats (use `Math/Math.zig` helpers).
- **No hidden fills.** In code that runs per step, body, contact or query, no field default, `.empty` or `.{}` for a
  value over 256 bytes: build it in place. Use `Core.swap`, not `std.mem.swap` (PortingGuide.md, "Large values").
  `zig build codegen-check` checks it.
- One Zig file per Jolt header (`.h` + `.inl` + `.cpp` merged), same directory and name.
  Every file starts with `//! Port of: <C++ files>` and `//! Status: complete|partial|stub`.
- Register new files in `source_files` in `Zolt/zolt.zig` (re-export public types there) and
  new test files in `ZoltTests/unit_tests.zig`.
- Naming: `GetFoo` → `getFoo`, `sFoo` → `foo`, `mFoo` → `foo`, `inFoo` → `foo`,
  `EFoo::Bar` → `Foo.bar`. Operators become methods (`add`, `sub`, `mul`, `mulScalar`, `eql`, ...).
- Keep Jolt's doc comments and implementation comments.
- Explicit `std.mem.Allocator` everywhere, no globals. Blocking sync primitives need `std.Io`.
  The exceptions are Jolt's own globals:
  - the static member variables that must stay global (`sFoo` → `pub var foo`);
  - the module-level variables of `Zolt/Physics/DeterminismLog.zig`: `writer` and `base_path`
    (Jolt's `sLog` and `sBasePath`), the stream state and the log file. They exist only with
    `-Ddeterminism_log=true`. Without it they are `void`;
  - the assert handler of `Core/IssueReporting.zig` (Jolt's `AssertFailed`). It exists only in
    builds with asserts (Debug and ReleaseSafe). It is the one global that Phase 9 adds
    (RemainingFeaturesArchitecture.md R8).
- `JPH_DEBUG_RENDERER` code is ported under the `-Ddebug_renderer` gates
  (RemainingFeaturesArchitecture.md R2). Never leave a `TODO(debug_renderer)`.
- Port the matching Jolt unit tests with the same names and values, and add parity tests
  (`ZoltParity/`) for every function that computes numbers.
- Before finishing: `zig build test`, `zig build test -Ddouble_precision=true`,
  `zig build check -Ddebug_renderer=true`, `zig build parity-check` (also with `-Ddebug_renderer=true`),
  `zig build parity` (also with `-Ddebug_renderer=true` for files with `JPH_DEBUG_RENDERER` code),
  `zig fmt --check Zolt ZoltTests ZoltParity ZoltHelloWorld ZoltPerformanceTest build.zig build.zig.zon`,
  `python3 tools/port_status.py --write`, then `--check`.

## Plan and status

- `Docs/Zolt/Roadmap.md`: phases in dependency order, milestones, the determinism acceptance test.
- `Docs/Zolt/Progress.md`: generated per-file status (do not edit by hand).
- Phases 0 to 9 are done (milestone M10: all of `UnitTests/` is ported). Only the GPU compute
  backends and the profiler stay deferred.
- Phase 10 made Zolt about as fast as C++ in instructions without changing a result (class A). Work that changes
  results follows the `-Djolt_compat` policy in the Roadmap's Phase 10 section.
- The port tracks Jolt v5.6.1 at commit `5830c34` (see Roadmap for syncing with upstream).
