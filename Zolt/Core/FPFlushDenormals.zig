//! Port of: Jolt/Core/FPFlushDenormals.h
//! Status: complete
//!
//! Usage (the C++ object on the stack becomes init() + defer deinit()):
//!     const flush_denormals = FPFlushDenormals.init();
//!     defer flush_denormals.deinit();

const std = @import("std");
const builtin = @import("builtin");
const FPControlWord = @import("FPControlWord.zig").FPControlWord;
const Word = @import("FPControlWord.zig").Word;

/// MXCSR flush to zero bit (_MM_FLUSH_ZERO_ON)
pub const mm_flush_zero_on: u32 = 0x8000;

/// MXCSR flush to zero mask (_MM_FLUSH_ZERO_MASK)
pub const mm_flush_zero_mask: u32 = 0x8000;

/// Flush denormals to zero bit (FP_FZ, of the ARM FPCR / FPSCR register)
pub const fp_fz: u64 = 1 << 24;

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
pub const FPFlushDenormals = switch (builtin.cpu.arch) {
    .x86, .x86_64 => if (std.Target.x86.featureSetHas(builtin.cpu.features, .sse))
        FPControlWord(mm_flush_zero_on, mm_flush_zero_mask)
    else
        NotSupported,
    .aarch64, .aarch64_be, .arm, .armeb, .thumb, .thumbeb => FPControlWord(@as(Word, @intCast(fp_fz)), @as(Word, @intCast(fp_fz))),
    else => NotSupported, // WASM, RISC-V, PowerPC, LoongArch
};

/// Not supported: does nothing (an empty class in C++)
const NotSupported = struct {
    pub fn init() NotSupported {
        return .{};
    }

    pub fn deinit(self: *const NotSupported) void {
        _ = self;
    }
};
