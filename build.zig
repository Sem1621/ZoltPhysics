//! Build script for Zolt, a Zig port of Jolt Physics.
//!
//! Steps:
//!   zig build              Build the library (installs docs-less static lib for C-ABI consumers later)
//!   zig build test         Run all tests (inline library tests + ported Jolt unit tests)
//!   zig build check        Compile everything without running (fast feedback / ZLS build-on-save)
//!   zig build docs         Emit autodoc HTML into zig-out/docs
//!
//! Options (mirror the Jolt CMake options that change semantics):
//!   -Ddouble_precision=true     JPH_DOUBLE_PRECISION: Real = f64, RVec3 = DVec3, RMat44 = DMat44
//!   -Dobject_layer_bits=16|32   JPH_OBJECT_LAYER_BITS
//!   -Dtest-filter=<substring>   Only run tests whose name contains the substring (repeatable)
//!   -Duse_llvm=true|false       Use the LLVM backend (default: true). Zig 0.16's self-hosted x86_64 backend,
//!                               which Zig uses by default for Debug builds, miscompiles some @Vector code.
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const double_precision = b.option(bool, "double_precision", "Use f64 for world space positions (JPH_DOUBLE_PRECISION)") orelse false;
    const object_layer_bits = b.option(u8, "object_layer_bits", "Number of bits in ObjectLayer, 16 or 32 (JPH_OBJECT_LAYER_BITS)") orelse 16;
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
    const options_module = options.createModule();

    // The library module. Consumers use it via `b.dependency("ZoltPhysics", ...).module("zolt")`.
    const zolt = b.addModule("zolt", .{
        .root_source_file = b.path("Zolt/zolt.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "zolt_options", .module = options_module },
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

    // Inline tests that live next to the code inside Zolt/
    const lib_tests = b.addTest(.{
        .name = "zolt-tests",
        .root_module = zolt,
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

    // Compile-only step, used for fast feedback and by ZLS (build-on-save)
    const check_step = b.step("check", "Compile the library and tests without running them");
    check_step.dependOn(&lib.step);
    check_step.dependOn(&lib_tests.step);
    check_step.dependOn(&unit_tests.step);

    const docs_step = b.step("docs", "Generate API documentation into zig-out/docs");
    docs_step.dependOn(&b.addInstallDirectory(.{
        .source_dir = lib.getEmittedDocs(),
        .install_dir = .prefix,
        .install_subdir = "docs",
    }).step);
}
