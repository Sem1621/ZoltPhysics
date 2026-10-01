//! Port of: Jolt/Core/FPControlWord.h
//! Status: complete
//!
//! The C++ class sets the control word in its constructor and restores it in its destructor. In Zig:
//!     const control_word = FPControlWord(value, mask).init();
//!     defer control_word.deinit();
//! Supported like in Jolt: x86 with SSE (MXCSR, via stmxcsr / ldmxcsr), 64-bit ARM (FPCR, via mrs / msr) and
//! 32-bit ARM with a VFP unit (FPSCR, via vmrs / vmsr). Jolt doesn't support the other CPUs (WASM, RISC-V,
//! PowerPC, LoongArch), there init() / deinit() do nothing.

const std = @import("std");
const builtin = @import("builtin");

/// How the floating point control word is accessed on the target CPU
const Backend = enum {
    /// MXCSR register (_mm_getcsr / _mm_setcsr)
    sse,
    /// FPCR register of 64-bit ARM
    fpcr,
    /// FPSCR register of 32-bit ARM
    fpscr,
    /// Not supported
    none,
};

const backend: Backend = switch (builtin.cpu.arch) {
    .x86, .x86_64 => if (std.Target.x86.featureSetHas(builtin.cpu.features, .sse)) .sse else .none,
    .aarch64, .aarch64_be => .fpcr,
    .arm, .armeb, .thumb, .thumbeb => if (std.Target.arm.featureSetHas(builtin.cpu.features, .vfp2sp)) .fpscr else .none,
    else => .none,
};

/// Type of the control word (and of the Value / Mask template parameters): uint for SSE, uint64 for 64-bit ARM, uint32 for 32-bit ARM
pub const Word = switch (backend) {
    .fpcr => u64,
    else => u32,
};

/// Helper class that needs to be put on the stack to update the state of the floating point control word.
/// This state is kept per thread.
pub fn FPControlWord(comptime value: Word, comptime mask: Word) type {
    return struct {
        const Self = @This();

        prev_state: Word,

        /// Save the current state and update the bits in `mask` to `value` (constructor)
        pub fn init() Self {
            const prev_state = readControlWord();
            writeControlWord((prev_state & ~mask) | value);
            return .{ .prev_state = prev_state };
        }

        /// Restore the bits in `mask` to the state saved by init() (destructor)
        pub fn deinit(self: *const Self) void {
            writeControlWord((readControlWord() & ~mask) | (self.prev_state & mask));
        }
    };
}

// Note on the x86 inline assembly: memory operands ("m" / "*m" constraints) are not handled consistently by Zig's
// LLVM and self-hosted backends, so stmxcsr / ldmxcsr get the address of the value in a fixed register instead.

/// Read the floating point control word
fn readControlWord() Word {
    switch (backend) {
        .sse => {
            // _mm_getcsr()
            var csr: u32 = 0;
            if (builtin.cpu.arch == .x86_64)
                asm volatile ("stmxcsr (%%rax)"
                    :
                    : [csr] "{rax}" (&csr),
                    : .{ .memory = true })
            else
                asm volatile ("stmxcsr (%%eax)"
                    :
                    : [csr] "{eax}" (&csr),
                    : .{ .memory = true });
            return csr;
        },
        .fpcr => {
            return asm volatile ("mrs %[val], fpcr"
                : [val] "=r" (-> u64),
            );
        },
        .fpscr => {
            return asm volatile ("vmrs %[val], fpscr"
                : [val] "=r" (-> u32),
            );
        },
        .none => return 0,
    }
}

/// Write the floating point control word
fn writeControlWord(control_word: Word) void {
    switch (backend) {
        .sse => {
            // _mm_setcsr(control_word)
            const csr: u32 = control_word;
            if (builtin.cpu.arch == .x86_64)
                asm volatile ("ldmxcsr (%%rax)"
                    :
                    : [csr] "{rax}" (&csr),
                    : .{ .memory = true, .mxcsr = true })
            else
                asm volatile ("ldmxcsr (%%eax)"
                    :
                    : [csr] "{eax}" (&csr),
                    : .{ .memory = true, .mxcsr = true });
        },
        .fpcr => {
            asm volatile ("msr fpcr, %[val]"
                :
                : [val] "r" (control_word),
                : .{ .memory = true, .fpcr = true });
        },
        .fpscr => {
            asm volatile ("vmsr fpscr, %[val]"
                :
                : [val] "r" (control_word),
                : .{ .memory = true });
        },
        .none => {},
    }
}

test "FPControlWord restores only the masked bits" {
    if (backend == .none) return error.SkipZigTest;

    // Use the flush to zero bit (MXCSR bit 15 / FPCR and FPSCR bit 24)
    const bit: Word = if (backend == .sse) 0x8000 else 1 << 24;

    // Ignore the sticky exception flags in the lower 6 bits of MXCSR (FPCR has no status bits)
    const control_bits: Word = if (backend == .sse) ~@as(Word, 0x3f) else ~@as(Word, 0);

    const before = readControlWord() & control_bits;
    {
        const control_word = FPControlWord(bit, bit).init();
        defer control_word.deinit();
        try std.testing.expect(readControlWord() & bit == bit);
        try std.testing.expectEqual(before & ~bit, readControlWord() & control_bits & ~bit);
    }
    try std.testing.expectEqual(before, readControlWord() & control_bits);

    // Only the masked bits are restored: a change of another bit while the guard is active is kept
    const other_bit: Word = if (backend == .sse) 0x2000 else 1 << 22; // Rounding mode bit
    {
        const control_word = FPControlWord(bit, bit).init();
        defer control_word.deinit();
        writeControlWord(readControlWord() ^ other_bit);
    }
    try std.testing.expectEqual(before ^ other_bit, readControlWord() & control_bits);
    writeControlWord(readControlWord() ^ other_bit);
    try std.testing.expectEqual(before, readControlWord() & control_bits);
}
