//! Build script for Zolt, a Zig port of Jolt Physics.
//!
//! Steps:
//!   zig build              Build the library (installs docs-less static lib for C-ABI consumers later)
//!   zig build test         Run all tests (inline library tests + ported Jolt unit tests)
//!   zig build check        Compile everything without running (fast feedback / ZLS build-on-save)
//!   zig build docs         Emit autodoc HTML into zig-out/docs
//!   zig build parity       Compare Zolt bit for bit with the C++ Jolt library (compiled from Jolt/ with Zig's C++ compiler)
//!   zig build parity-check Compile the parity tests and the C++ reference without running them
//!   zig build performance-test -- <args>   Run ZoltPerformanceTest (the port of PerformanceTest/, see its -h)
//!   zig build hello-world  Run ZoltHelloWorld (the port of HelloWorld/HelloWorld.cpp, milestone M5)
//!   zig build hello-world-check  Run the C++ HelloWorld and its port, fail unless their stdout is identical (part of parity)
//!   zig build jolt-performance-test -- <args>   Run the C++ PerformanceTest built like the parity reference (the M9 oracle)
//!   zig build debug-renderer-check -Ddebug_renderer=true   Record the determinism scenes with the C++ PerformanceTest and
//!                          ZoltPerformanceTest (-r), fail unless the .jor bytes are equal
//!   zig build codegen-check  Build ZoltPerformanceTest (ReleaseFast, x86-64-v3) and fail when the library code fills or
//!                          copies a large buffer or swaps byte by byte (tools/check_codegen.py)
//!   zig build perf-budget [-- <scene>...]  Count the instructions of the physics updates of ZoltPerformanceTest and
//!                          the C++ PerformanceTest with valgrind, fail when Zolt is over its budget (tools/perf_budget.py)
//!
//! Options (mirror the Jolt CMake options that change semantics):
//!   -Ddouble_precision=true     JPH_DOUBLE_PRECISION: Real = f64, RVec3 = DVec3, RMat44 = DMat44
//!   -Dobject_layer_bits=16|32   JPH_OBJECT_LAYER_BITS
//!   -Ddeterminism_log=true      JPH_ENABLE_DETERMINISM_LOG: Zolt/Physics/DeterminismLog.zig exists at run time (the
//!                               `test` step always also runs the DeterminismLog unit tests with it enabled)
//!   -Ddebug_renderer=true       JPH_DEBUG_RENDERER: compile the debug renderer hooks into the library (the draw vtable
//!                               entries, the draw functions, the debug-only members and parameters). Off by default:
//!                               then every hook is `void`. The C++ reference of `zig build parity` gets the define too
//!                               (Docs/Zolt/RemainingFeaturesArchitecture.md R1).
//!   -Dtest-filter=<substring>   Only run tests whose name contains the substring (repeatable)
//!   -Dreference_asserts=true    Build the C++ reference of `zig build parity` with JPH_ENABLE_ASSERTS: the simulation
//!                               parity fails a scene whose C++ side hits a Jolt assert. The release_only scenes and
//!                               the release only parity tests compare the counts of both sides instead (R8).
//!   -Dupdate_timeout=<seconds>  The simulation parity aborts the parity process when one PhysicsSystem::Update of either
//!                               side runs longer (the update watchdog, default 300; 0: off, e.g. under a debugger)
//!   -Duse_llvm=true|false       Use the LLVM backend (default: true). Zig 0.16's self-hosted x86_64 backend,
//!                               which Zig uses by default for Debug builds, miscompiles some @Vector code.
//!   -Djob_profile=true          Record a timeline of the jobs and the idle time of every job thread
//!                               (Zolt/Core/JobProfiler.zig, `ZoltPerformanceTest -p`). Off by default: then every hook
//!                               is `void` and costs nothing. Results do not change.
//!   -Dsimd_contact_solver=false Solve the contacts of the parallel splits one at a time, like Jolt (the default true
//!                               solves 8 at a time with SIMD, Zolt/Physics/Constraints/ContactConstraintSimd.zig).
//!                               Results do not change: an A/B switch.
const std = @import("std");
const jolt_sources = @import("ZoltParity/jolt_sources.zig");
const parity_reference_sources = @import("ZoltParity/reference_sources.zig");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const double_precision = b.option(bool, "double_precision", "Use f64 for world space positions (JPH_DOUBLE_PRECISION)") orelse false;
    const object_layer_bits = b.option(u8, "object_layer_bits", "Number of bits in ObjectLayer, 16 or 32 (JPH_OBJECT_LAYER_BITS)") orelse 16;
    const determinism_log = b.option(bool, "determinism_log", "Log the simulation state to find determinism issues (JPH_ENABLE_DETERMINISM_LOG)") orelse false;
    const debug_renderer = b.option(bool, "debug_renderer", "Compile the debug renderer hooks into the library and the C++ reference (JPH_DEBUG_RENDERER)") orelse false;
    const job_profile = b.option(bool, "job_profile", "Record a timeline of the jobs and the idle time of the job threads (Zolt/Core/JobProfiler.zig)") orelse false;
    const simd_contact_solver = b.option(bool, "simd_contact_solver", "Solve the contacts of the parallel splits 8 at a time with SIMD (default true, results do not change)") orelse true;
    const test_filters = b.option([]const []const u8, "test-filter", "Only run tests whose name contains this substring") orelse &.{};
    // Zig 0.16's self-hosted x86_64 backend (the default for Debug builds) miscompiles some vector operations
    // (e.g. @bitCast of @Vector(4, bool)), so default to LLVM. Use -Duse_llvm=false for faster Debug compiles.
    const use_llvm = b.option(bool, "use_llvm", "Use the LLVM backend (default true, the self-hosted backend miscompiles some @Vector code)") orelse true;

    if (object_layer_bits != 16 and object_layer_bits != 32) {
        std.debug.panic("object_layer_bits must be 16 or 32, got {d}", .{object_layer_bits});
    }

    // Compile time configuration, available in the library as @import("zolt_options")
    const options = b.addOptions();
    options.addOption(bool, "double_precision", double_precision);
    options.addOption(u8, "object_layer_bits", object_layer_bits);
    options.addOption(bool, "determinism_log", determinism_log);
    options.addOption(bool, "debug_renderer", debug_renderer);
    options.addOption(bool, "job_profile", job_profile);
    options.addOption(bool, "simd_contact_solver", simd_contact_solver);
    const options_module = options.createModule();

    // User shape registrations, default material and serializable classes (Zolt/RegisterTypes.zig): an empty module by
    // default. An application replaces it with `zolt_module.addImport("zolt_user_types", my_module)`, where
    // `my_module` may declare `registrations`, `default_material` and `serializable_types` (and import "zolt" itself).
    const user_types_source = b.addWriteFiles().add("zolt_user_types.zig",
        \\//! Default (empty) `zolt_user_types` module: no user shapes, Jolt's default material, no user serializable classes.
        \\pub const registrations = .{};
        \\
    );
    const user_types_module = b.createModule(.{
        .root_source_file = user_types_source,
        .target = target,
        .optimize = optimize,
    });

    // The library module. Consumers use it via `b.dependency("ZoltPhysics", ...).module("zolt")`.
    const zolt = b.addModule("zolt", .{
        .root_source_file = b.path("Zolt/zolt.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt_options", .module = options_module },
            .{ .name = "zolt_user_types", .module = user_types_module },
        },
    });

    // Static library artifact (mostly useful to verify that everything compiles and for future C-ABI work)
    const lib = b.addLibrary(.{
        .name = "zolt",
        .linkage = .static,
        .root_module = zolt,
        .use_llvm = use_llvm,
    });
    b.installArtifact(lib);

    // Inline tests that live next to the code inside Zolt/. They run on their own instance of the library module whose
    // `zolt_user_types` registers the test shapes of the shape core (Zolt/Physics/Collision/Shape/TestShapes.zig) and
    // the test classes of the factory through the user hook, exactly like an application would (a module import
    // cycle). The library and the unit tests use the default (empty) module, the parity tests have their own.
    const test_user_types_module = b.createModule(.{
        .root_source_file = b.addWriteFiles().add("zolt_user_types.zig",
            \\//! `zolt_user_types` of the inline tests: the test shapes, the test material and the test constraint of Zolt
            \\//! (only declared in tests).
            \\const zolt = @import("zolt");
            \\pub const registrations = zolt.test_shapes.registrations;
            \\pub const serializable_types = zolt.test_shapes.serializable_types ++ zolt.test_constraints.serializable_types;
            \\
        ),
        .target = target,
        .optimize = optimize,
    });
    const zolt_for_tests = b.createModule(.{
        .root_source_file = b.path("Zolt/zolt.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt_options", .module = options_module },
            .{ .name = "zolt_user_types", .module = test_user_types_module },
        },
    });
    test_user_types_module.addImport("zolt", zolt_for_tests);
    const lib_tests = b.addTest(.{
        .name = "zolt-tests",
        .root_module = zolt_for_tests,
        .filters = test_filters,
        .use_llvm = use_llvm,
    });

    // Port of Jolt's UnitTests/ directory. These only use the public API through @import("zolt").
    const unit_tests = b.addTest(.{
        .name = "zolt-unit-tests",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltTests/unit_tests.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt", .module = zolt },
            },
        }),
        .filters = test_filters,
        .use_llvm = use_llvm,
    });

    const test_step = b.step("test", "Run all tests");
    test_step.dependOn(&b.addRunArtifact(lib_tests).step);
    test_step.dependOn(&b.addRunArtifact(unit_tests).step);

    // The unit tests once more on a library built with the determinism log enabled (-Ddeterminism_log=true), so that
    // the log text of a simulation is checked by every test run. Without -Dtest-filter only the DeterminismLog tests
    // run (the others are compiled out by the filter); with filters the given tests run in this configuration too.
    var det_log_tests_opt: ?*std.Build.Step.Compile = null;
    if (!determinism_log) {
        const det_log_options = b.addOptions();
        det_log_options.addOption(bool, "double_precision", double_precision);
        det_log_options.addOption(u8, "object_layer_bits", object_layer_bits);
        det_log_options.addOption(bool, "determinism_log", true);
        det_log_options.addOption(bool, "debug_renderer", debug_renderer);
        det_log_options.addOption(bool, "job_profile", job_profile);
        det_log_options.addOption(bool, "simd_contact_solver", simd_contact_solver);
        const zolt_det_log = b.createModule(.{
            .root_source_file = b.path("Zolt/zolt.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt_options", .module = det_log_options.createModule() },
                .{ .name = "zolt_user_types", .module = user_types_module },
            },
        });
        const det_log_tests = b.addTest(.{
            .name = "zolt-unit-tests-determinism-log",
            .root_module = b.createModule(.{
                .root_source_file = b.path("ZoltTests/unit_tests.zig"),
                .target = target,
                .optimize = optimize,
                .imports = &.{
                    .{ .name = "zolt", .module = zolt_det_log },
                },
            }),
            .filters = if (test_filters.len != 0) test_filters else &.{"DeterminismLog"},
            .use_llvm = use_llvm,
        });
        test_step.dependOn(&b.addRunArtifact(det_log_tests).step);
        det_log_tests_opt = det_log_tests;
    }

    // Compile-only step, used for fast feedback and by ZLS (build-on-save)
    const check_step = b.step("check", "Compile the library and tests without running them");
    check_step.dependOn(&lib.step);
    check_step.dependOn(&lib_tests.step);
    check_step.dependOn(&unit_tests.step);
    if (det_log_tests_opt) |t| check_step.dependOn(&t.step);

    // Port of PerformanceTest/ (milestone M6): installed with the library, `zig build performance-test -- <arguments>`
    // runs it, e.g. `zig build performance-test -Doptimize=ReleaseFast -- -q=LinearCast -t=max -s=Pyramid
    // -validate_hash=0x74d0118836ac0892` (the hashes of .github/workflows/determinism_check.yml)
    const performance_test = b.addExecutable(.{
        .name = "ZoltPerformanceTest",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltPerformanceTest/PerformanceTest.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt", .module = zolt },
            },
        }),
        .use_llvm = use_llvm,
    });
    b.installArtifact(performance_test);
    const run_performance_test = b.addRunArtifact(performance_test);
    if (b.args) |args| run_performance_test.addArgs(args);
    const performance_test_step = b.step("performance-test", "Run the port of PerformanceTest (arguments after --)");
    performance_test_step.dependOn(&run_performance_test.step);
    check_step.dependOn(&performance_test.step);

    // The inline tests of ZoltPerformanceTest/ (the Ragdoll scene reads Assets/, relative to the repository root)
    const performance_test_tests = b.addTest(.{
        .name = "zolt-performance-test-tests",
        .root_module = performance_test.root_module,
        .filters = test_filters,
        .use_llvm = use_llvm,
    });
    const run_performance_test_tests = b.addRunArtifact(performance_test_tests);
    run_performance_test_tests.setCwd(b.path("."));
    test_step.dependOn(&run_performance_test_tests.step);
    check_step.dependOn(&performance_test_tests.step);

    const docs_step = b.step("docs", "Generate API documentation into zig-out/docs");
    docs_step.dependOn(&b.addInstallDirectory(.{
        .source_dir = lib.getEmittedDocs(),
        .install_dir = .prefix,
        .install_subdir = "docs",
    }).step);

    // Parity tests: the C++ library is the reference. It is compiled in the configuration that Zolt
    // follows (JPH_CROSS_PLATFORM_DETERMINISTIC, no FP contraction, no asserts / profiler, and no debug renderer unless
    // -Ddebug_renderer=true, like the Distribution build used for Jolt's determinism hashes). -Dreference_asserts=true adds
    // JPH_ENABLE_ASSERTS: a failed Jolt assert is printed and counted (ZoltParity/ParityHooks.h), and the simulation
    // parity fails a scene whose C++ side hit one (the check that scenes keep away from Jolt's asserts). The release_only
    // scenes and the release only parity tests (ParityHooks.ReleaseAsserts) do not fail on them: they compare the count
    // with Zolt's (Docs/Zolt/RemainingFeaturesArchitecture.md R8).
    const reference_asserts = b.option(bool, "reference_asserts", "Build the C++ reference of the parity tests with JPH_ENABLE_ASSERTS (scenes that hit a Jolt assert fail, release only tests compare the assert counts)") orelse false;
    // JPH_OBJECT_STREAM: Jolt's CMake option ENABLE_OBJECT_STREAM (ON by default, also in the build of the determinism
    // hashes). It adds the attribute tables to the RTTI, makes Factory::Register also register the attribute types and
    // compiles ObjectStream/*.cpp, which Phase 8 compares with Zolt (Docs/Zolt/SerializationArchitecture.md S1). Every
    // C++ translation unit of the parity build (Jolt, the references, HelloWorld, PerformanceTest) must agree on it: it
    // changes the RTTI layout and JPH_VERSION_ID.
    var cpp_flags: std.ArrayList([]const u8) = .empty;
    cpp_flags.appendSlice(b.allocator, &.{ "-std=c++17", "-ffp-contract=off", "-DJPH_CROSS_PLATFORM_DETERMINISTIC", "-DJPH_OBJECT_STREAM" }) catch @panic("OOM");
    if (double_precision) cpp_flags.append(b.allocator, "-DJPH_DOUBLE_PRECISION") catch @panic("OOM");
    cpp_flags.append(b.allocator, b.fmt("-DJPH_OBJECT_LAYER_BITS={d}", .{object_layer_bits})) catch @panic("OOM");
    if (reference_asserts) cpp_flags.append(b.allocator, "-DJPH_ENABLE_ASSERTS") catch @panic("OOM");
    // JPH_USE_CPU_COMPUTE: Jolt's CMake option USE_CPU_COMPUTE (ON by default). It compiles the CPU compute backend
    // (Compute/CPU/*.cpp) and the HLSL shaders as C++ (Shaders/HairWrapper.cpp, TestComputeWrapper.cpp), which Phase 9
    // compares with Zolt (Docs/Zolt/RemainingFeaturesArchitecture.md R1). It is not a JPH_VERSION_ID feature bit; only
    // the compute, shader and hair translation units preprocess differently with it, so no other result changes.
    cpp_flags.append(b.allocator, "-DJPH_USE_CPU_COMPUTE") catch @panic("OOM");
    // JPH_DEBUG_RENDERER follows -Ddebug_renderer (Zolt's Core.debug_renderer). It is bit 6 of JPH_VERSION_ID and it
    // changes Jolt's results that the parity sees (Constraint::GetConstraintSettings copies mDrawConstraintSize, the
    // configuration string), so Zolt and every C++ translation unit (Jolt, the references, HelloWorld, PerformanceTest)
    // must agree on it.
    if (debug_renderer) cpp_flags.append(b.allocator, "-DJPH_DEBUG_RENDERER") catch @panic("OOM");

    // On x86_64 the reference is pinned to x86-64-v3 (SSE4.2/AVX/AVX2/F16C/LZCNT/TZCNT, no AVX-512), which
    // matches the default ISA options of Jolt's CMake build, so parity results don't depend on the host CPU.
    const reference_target = if (target.result.cpu.arch == .x86_64) b.resolveTargetQuery(.{
        .cpu_arch = .x86_64,
        .os_tag = target.result.os.tag,
        .abi = target.result.abi,
        .cpu_model = .{ .explicit = &std.Target.x86.cpu.x86_64_v3 },
    }) else target;

    const jolt_cpp = b.addLibrary(.{
        .name = "jolt-cpp",
        .linkage = .static,
        .root_module = b.createModule(.{
            .target = reference_target,
            // The reference is always optimized: results don't depend on it and it keeps the step fast
            .optimize = .ReleaseFast,
            .link_libcpp = true,
            .sanitize_c = .off,
        }),
    });
    jolt_cpp.root_module.addIncludePath(b.path("."));
    jolt_cpp.root_module.addCSourceFiles(.{ .root = b.path("Jolt"), .files = &jolt_sources.files, .flags = cpp_flags.items });
    jolt_cpp.root_module.addCSourceFiles(.{ .root = b.path("ZoltParity"), .files = &parity_reference_sources.files, .flags = cpp_flags.items });
    // The C files of the reference (the libm pin, Docs/Zolt/RemainingFeaturesArchitecture.md R13): C flags, no FP
    // contraction like the C++ files
    jolt_cpp.root_module.addCSourceFiles(.{ .root = b.path("ZoltParity"), .files = &parity_reference_sources.c_files, .flags = &.{ "-std=c11", "-ffp-contract=off" } });

    // The parity tests run on their own instance of the library module too: its `zolt_user_types` registers the collide
    // / cast functions of the parity test shape (ZoltParity/Physics/ShapeCoreUserTypes.zig), which the C++ reference
    // registers as well, so the dispatch tables, the reversed functions and the TransformedShape entry points are
    // compared through the user hook. The parity tests import that module as "parity_user_types".
    const parity_user_types_module = b.createModule(.{
        .root_source_file = b.path("ZoltParity/Physics/ShapeCoreUserTypes.zig"),
        .target = target,
        .optimize = optimize,
    });
    const zolt_for_parity = b.createModule(.{
        .root_source_file = b.path("Zolt/zolt.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt_options", .module = options_module },
            .{ .name = "zolt_user_types", .module = parity_user_types_module },
        },
    });
    parity_user_types_module.addImport("zolt", zolt_for_parity);

    // The layers of the unit tests (ZoltTests/Layers.zig), used by the simulation parity tests whose C++ side includes
    // UnitTests/Layers.h. The parity tests import it as "test_layers".
    const parity_layers_module = b.createModule(.{
        .root_source_file = b.path("ZoltTests/Layers.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt", .module = zolt_for_parity },
        },
    });

    // The classes of the ObjectStream unit test (ZoltTests/ObjectStream/ObjectStreamTest.zig), whose streams the
    // ObjectStream parity compares with the C++ library's. The parity tests import it as "objectstream_test".
    const parity_objectstream_test_module = b.createModule(.{
        .root_source_file = b.path("ZoltTests/ObjectStream/ObjectStreamTest.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt", .module = zolt_for_parity },
        },
    });

    // ZoltPerformanceTest's RagdollScene (milestone M9), which the parity test perf_ragdoll runs against
    // PerformanceTest/RagdollScene.h (ZoltParity/Physics/PerfRagdollParity.zig). The parity tests import it as
    // "perf_ragdoll_scene".
    const parity_perf_ragdoll_module = b.createModule(.{
        .root_source_file = b.path("ZoltPerformanceTest/RagdollScene.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt", .module = zolt_for_parity },
        },
    });

    // The update watchdog of the simulation parity (ZoltParity/Physics/SimulationParity.zig, "Hangs" in its header). The
    // parity tests import it as "parity_options".
    const update_timeout = b.option(u32, "update_timeout", "Seconds that one PhysicsSystem::Update of the simulation parity may run before the parity process aborts (default 300, 0: no watchdog)") orelse 300;
    const parity_options = b.addOptions();
    parity_options.addOption(u32, "update_timeout", update_timeout);

    const parity_tests = b.addTest(.{
        .name = "zolt-parity",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltParity/parity.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt", .module = zolt_for_parity },
                .{ .name = "parity_user_types", .module = parity_user_types_module },
                .{ .name = "test_layers", .module = parity_layers_module },
                .{ .name = "objectstream_test", .module = parity_objectstream_test_module },
                .{ .name = "perf_ragdoll_scene", .module = parity_perf_ragdoll_module },
                .{ .name = "parity_options", .module = parity_options.createModule() },
            },
        }),
        .filters = test_filters,
        .use_llvm = use_llvm,
    });
    parity_tests.root_module.linkLibrary(jolt_cpp);

    const parity_step = b.step("parity", "Compare Zolt bit for bit with the C++ Jolt library");
    const run_parity_tests = b.addRunArtifact(parity_tests);
    run_parity_tests.setCwd(b.path(".")); // The ObjectStream parity reads Assets/ (relative to the repository root)
    parity_step.dependOn(&run_parity_tests.step);

    // Compile-only step of the parity tests: builds the C++ reference and every parity test without running them. A
    // filtered parity run analyzes only the tests that it runs, so this catches compile errors in the others.
    const parity_check_step = b.step("parity-check", "Compile the parity tests and the C++ reference without running them");
    parity_check_step.dependOn(&parity_tests.step);

    // Port of HelloWorld/HelloWorld.cpp (milestone M5): installed with the library, `zig build hello-world` runs it.
    const hello_world = b.addExecutable(.{
        .name = "ZoltHelloWorld",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltHelloWorld/HelloWorld.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt", .module = zolt },
            },
        }),
        .use_llvm = use_llvm,
    });
    b.installArtifact(hello_world);
    check_step.dependOn(&hello_world.step);
    const hello_world_step = b.step("hello-world", "Run the port of HelloWorld/HelloWorld.cpp");
    hello_world_step.dependOn(&b.addRunArtifact(hello_world).step);

    // `zig build hello-world-check`: the C++ HelloWorld, built against the C++ Jolt library of the parity tests (same
    // flags), and the Zig port must print the same bytes (ZoltHelloWorld/CompareOutput.zig compares them). The parity
    // step includes the check.
    const hello_world_cpp = b.addExecutable(.{
        .name = "JoltHelloWorld",
        .root_module = b.createModule(.{
            .target = reference_target,
            .optimize = .ReleaseFast,
            .link_libcpp = true,
            .sanitize_c = .off,
        }),
    });
    hello_world_cpp.root_module.addIncludePath(b.path("."));
    hello_world_cpp.root_module.addCSourceFile(.{ .file = b.path("HelloWorld/HelloWorld.cpp"), .flags = cpp_flags.items });
    hello_world_cpp.root_module.linkLibrary(jolt_cpp);
    const compare_output = b.addExecutable(.{
        .name = "CompareOutput",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltHelloWorld/CompareOutput.zig"),
            .target = b.graph.host,
            .optimize = optimize,
        }),
        .use_llvm = use_llvm,
    });
    const run_compare_output = b.addRunArtifact(compare_output);
    run_compare_output.addFileArg(b.addRunArtifact(hello_world_cpp).captureStdOut(.{}));
    run_compare_output.addFileArg(b.addRunArtifact(hello_world).captureStdOut(.{}));
    const hello_world_check_step = b.step("hello-world-check", "Fail unless HelloWorld/HelloWorld.cpp and its Zig port print the same bytes");
    hello_world_check_step.dependOn(&run_compare_output.step);
    parity_step.dependOn(&run_compare_output.step);

    // `zig build jolt-performance-test -- <arguments>`: the C++ PerformanceTest (PerformanceTest/PerformanceTest.cpp),
    // built against the C++ library of the parity tests (same flags, with JPH_OBJECT_STREAM, so it has the Ragdoll scene).
    // It is the oracle of milestone M9: it must give the hashes of determinism_check.yml, and its short runs
    // (-i=<updates>) give the expected values of ZoltPerformanceTest's inline test.
    const jolt_performance_test = b.addExecutable(.{
        .name = "JoltPerformanceTest",
        .root_module = b.createModule(.{
            .target = reference_target,
            .optimize = .ReleaseFast,
            .link_libcpp = true,
            .sanitize_c = .off,
        }),
    });
    jolt_performance_test.root_module.addIncludePath(b.path("."));
    jolt_performance_test.root_module.addCSourceFile(.{ .file = b.path("PerformanceTest/PerformanceTest.cpp"), .flags = cpp_flags.items });
    jolt_performance_test.root_module.linkLibrary(jolt_cpp);
    const run_jolt_performance_test = b.addRunArtifact(jolt_performance_test);
    run_jolt_performance_test.setCwd(b.path("."));
    if (b.args) |args| run_jolt_performance_test.addArgs(args);
    const jolt_performance_test_step = b.step("jolt-performance-test", "Run the C++ PerformanceTest built like the parity reference (arguments after --)");
    jolt_performance_test_step.dependOn(&run_jolt_performance_test.step);

    // `zig build debug-renderer-check -Ddebug_renderer=true` (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, ".jor
    // bytes"): the C++ PerformanceTest and ZoltPerformanceTest record the 5 scenes of the determinism check with
    // `-r -i=20`, and ZoltParity/Renderer/DebugRendererCheck.zig fails unless the .jor bytes are equal. Zolt's
    // DebugRendererPlayback also reads every .jor of the C++ side.
    const debug_renderer_check = b.addExecutable(.{
        .name = "DebugRendererCheck",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltParity/Renderer/DebugRendererCheck.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zolt", .module = zolt },
            },
        }),
        .use_llvm = use_llvm,
    });
    check_step.dependOn(&debug_renderer_check.step);
    const run_debug_renderer_check = b.addRunArtifact(debug_renderer_check);
    run_debug_renderer_check.addArtifactArg(jolt_performance_test);
    run_debug_renderer_check.addArtifactArg(performance_test);
    _ = run_debug_renderer_check.addOutputDirectoryArg("debug-renderer-check");
    run_debug_renderer_check.stdio = .inherit;
    const debug_renderer_check_step = b.step("debug-renderer-check", "Fail unless the C++ PerformanceTest and ZoltPerformanceTest record the same .jor bytes (needs -Ddebug_renderer=true)");
    debug_renderer_check_step.dependOn(&run_debug_renderer_check.step);

    // The regression guard of Phase 10 (Docs/Zolt/Roadmap.md). Bit exactness cannot see a lost speedup, so two steps
    // check the machine code and the instruction counts. Both use ZoltPerformanceTest built like the benchmarks:
    // ReleaseFast with LLVM for the target of the C++ reference (x86-64-v3 on x86_64), whatever -Doptimize says.
    const bench_user_types_module = b.createModule(.{
        .root_source_file = user_types_source,
        .target = reference_target,
        .optimize = .ReleaseFast,
    });
    const zolt_bench = b.createModule(.{
        .root_source_file = b.path("Zolt/zolt.zig"),
        .target = reference_target,
        .optimize = .ReleaseFast,
        .imports = &.{
            .{ .name = "zolt_options", .module = options_module },
            .{ .name = "zolt_user_types", .module = bench_user_types_module },
        },
    });
    const bench_performance_test = b.addExecutable(.{
        .name = "ZoltPerformanceTestBench",
        .root_module = b.createModule(.{
            .root_source_file = b.path("ZoltPerformanceTest/PerformanceTest.zig"),
            .target = reference_target,
            .optimize = .ReleaseFast,
            .imports = &.{
                .{ .name = "zolt", .module = zolt_bench },
            },
        }),
        .use_llvm = true,
    });

    // `zig build codegen-check`: tools/check_codegen.py reads `objdump -d -l --inlines` of the binary. It fails when
    // library code (also where LLVM inlined it into the test program) calls memset / memcpy with a large constant
    // size, copies a large constant from read-only data, or swaps with std.mem.swap, unless an allow-list entry of the
    // script covers it. It needs GNU objdump and fails on a binary that is not x86-64: it reads x86-64 code only.
    const run_codegen_check = b.addSystemCommand(&.{"python3"});
    run_codegen_check.addFileArg(b.path("tools/check_codegen.py"));
    run_codegen_check.addArtifactArg(bench_performance_test);
    const codegen_check_step = b.step("codegen-check", "Fail when the library code of ZoltPerformanceTest (ReleaseFast) fills or copies a large buffer or swaps byte by byte");
    codegen_check_step.dependOn(&run_codegen_check.step);

    // `zig build perf-budget`: tools/perf_budget.py counts the instructions of the physics updates of short runs with
    // valgrind (callgrind) in ZoltPerformanceTest and the C++ PerformanceTest of `jolt-performance-test`, and fails when
    // Zolt is over the budget of the script. It needs valgrind and takes a few minutes.
    const run_perf_budget = b.addSystemCommand(&.{"python3"});
    run_perf_budget.addFileArg(b.path("tools/perf_budget.py"));
    run_perf_budget.addArtifactArg(bench_performance_test);
    run_perf_budget.addArtifactArg(jolt_performance_test);
    run_perf_budget.setCwd(b.path(".")); // The Ragdoll scene reads Assets/
    run_perf_budget.stdio = .inherit;
    if (b.args) |args| run_perf_budget.addArgs(args); // Only these scenes: `zig build perf-budget -- Pyramid`
    const perf_budget_step = b.step("perf-budget", "Fail when the physics updates of ZoltPerformanceTest take more instructions than their budget (needs valgrind)");
    perf_budget_step.dependOn(&run_perf_budget.step);
}
