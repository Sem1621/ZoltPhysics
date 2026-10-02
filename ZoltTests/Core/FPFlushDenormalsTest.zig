//! Port of: UnitTests/Core/FPFlushDenormalsTest.cpp

const std = @import("std");
const builtin = @import("builtin");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const FPFlushDenormals = zolt.FPFlushDenormals;

/// Like the C++ test: not supported on WASM, RISC-V, PowerPC and LoongArch
const supported = switch (builtin.cpu.arch) {
    .wasm32, .wasm64, .riscv32, .riscv64, .powerpc, .powerpcle, .powerpc64, .powerpc64le, .loongarch32, .loongarch64 => false,
    else => true,
};

// Implemented as a global atomic so the compiler can't optimize it to a constant
var test_flt_min: std.atomic.Value(f32) = .init(zolt.math.flt_min);

// The result of each multiplication is stored in an atomic too, so that the multiplication can't be moved out of the
// scope of FPFlushDenormals (the inline assembly that changes the control word only orders memory operations)
var test_result: std.atomic.Value(f32) = .init(0.0);

fn multiplyFltMin() f32 {
    test_result.store(test_flt_min.load(.seq_cst) * 0.1, .seq_cst);
    return test_result.load(.seq_cst);
}

test "TestFlushDenormals" {
    if (!supported) return error.SkipZigTest;

    // By default flush denormals should be off
    {
        const value = multiplyFltMin();
        try fw.expect(value > 0.0);
    }

    // Turn flush denormal on
    {
        const flush_denormals = FPFlushDenormals.init();
        defer flush_denormals.deinit();

        const value = multiplyFltMin();
        try fw.expect(value == 0.0);
    }

    // Check if state was properly restored
    {
        const value = multiplyFltMin();
        try fw.expect(value > 0.0);
    }

    // Update TestFltMin to prevent the compiler from optimizing away TestFltMin and replace all calculations above with 0
    test_flt_min.store(1.0, .seq_cst);
}
