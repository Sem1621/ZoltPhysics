//! Port of: Jolt/ConfigurationString.h
//! Status: complete
//!
//! The string is built at compile time from the Zig target and the build options, using Jolt's vocabulary.
//! Zolt doesn't use intrinsics: the instruction sets listed are the target features that the compiler may use
//! for @Vector code. Some of Jolt's flags never apply to Zolt: FMADD (never used, see the guide's floating point
//! rules), FP exceptions, debug renderer, profiler, ObjectStream (not ported yet), C++ RTTI / exceptions and
//! shared library. Zolt always follows JPH_CROSS_PLATFORM_DETERMINISTIC.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("Core/Core.zig");

/// Construct a string that lists the most important configuration settings
pub fn getConfigurationString() [:0]const u8 {
    return configuration_string;
}

const configuration_string: [:0]const u8 = (if (Core.double_precision) "Double" else "Single") ++ " precision " ++
    cpu_string ++
    (if (@bitSizeOf(usize) == 64) "64-bit " else if (@bitSizeOf(usize) == 32) "32-bit " else "") ++
    "with instructions: " ++
    instructions_string ++
    "(Cross Platform Deterministic) " ++
    (if (Core.object_layer_bits == 32) "(32-bit ObjectLayer) " else "(16-bit ObjectLayer) ") ++
    (if (Core.enable_asserts) "(Assertions) " else "") ++
    (if (builtin.mode == .Debug) "(Debug) " else "");

/// Name of the CPU architecture (JPH_CPU_*)
const cpu_string = switch (builtin.cpu.arch) {
    .x86, .x86_64 => "x86 ",
    .arm, .armeb, .thumb, .thumbeb, .aarch64, .aarch64_be => "ARM ",
    .riscv32, .riscv32be, .riscv64, .riscv64be => "RISC-V ",
    .powerpc, .powerpcle, .powerpc64, .powerpc64le => "PowerPC " ++ (if (builtin.cpu.arch.endian() == .big) "(Big Endian) " else "(Little Endian) "),
    .loongarch32, .loongarch64 => "LoongArch ",
    .wasm32, .wasm64 => "WASM ",
    else => @tagName(builtin.cpu.arch) ++ " ", // Not supported by Jolt
};

fn hasX86Feature(comptime feature: std.Target.x86.Feature) bool {
    return std.Target.x86.featureSetHas(builtin.cpu.features, feature);
}

/// Instruction sets that are enabled (JPH_USE_*), in Jolt's order
const instructions_string = switch (builtin.cpu.arch) {
    .x86, .x86_64 => blk: {
        // Same derivation as Core.h: each instruction set implies the ones below it
        const use_avx512 = hasX86Feature(.avx512f) and hasX86Feature(.avx512vl) and hasX86Feature(.avx512dq);
        const use_avx2 = hasX86Feature(.avx2) or use_avx512;
        const use_avx = hasX86Feature(.avx) or use_avx2;
        const use_sse4_2 = hasX86Feature(.sse4_2) or use_avx;
        const use_sse4_1 = hasX86Feature(.sse4_1) or use_sse4_2;
        const use_f16c = hasX86Feature(.f16c) or use_avx2;
        const use_lzcnt = hasX86Feature(.lzcnt) or use_avx2;
        const use_tzcnt = hasX86Feature(.bmi) or use_avx2;
        break :blk "SSE2 " ++
            (if (use_sse4_1) "SSE4.1 " else "") ++
            (if (use_sse4_2) "SSE4.2 " else "") ++
            (if (use_avx) "AVX " else "") ++
            (if (use_avx2) "AVX2 " else "") ++
            (if (use_avx512) "AVX512 " else "") ++
            (if (use_f16c) "F16C " else "") ++
            (if (use_lzcnt) "LZCNT " else "") ++
            (if (use_tzcnt) "TZCNT " else "");
    },
    .aarch64, .aarch64_be => "NEON ",
    .riscv32, .riscv32be, .riscv64, .riscv64be => if (std.Target.riscv.featureSetHas(builtin.cpu.features, .v)) "RVV " else "",
    .wasm32, .wasm64 => if (std.Target.wasm.featureSetHas(builtin.cpu.features, .simd128)) "SSE2 SSE4.1 SSE4.2 " else "",
    else => "",
};

test "getConfigurationString" {
    const s = getConfigurationString();
    try std.testing.expect(std.mem.startsWith(u8, s, if (Core.double_precision) "Double precision " else "Single precision "));
    try std.testing.expect(std.mem.indexOf(u8, s, "(Cross Platform Deterministic) ") != null);
    try std.testing.expect(std.mem.indexOf(u8, s, if (Core.object_layer_bits == 32) "(32-bit ObjectLayer) " else "(16-bit ObjectLayer) ") != null);
    if (builtin.cpu.arch == .x86_64)
        try std.testing.expect(std.mem.indexOf(u8, s, "x86 64-bit with instructions: SSE2 ") != null);
    try std.testing.expectEqual(@as(u8, 0), s.ptr[s.len]);
}
