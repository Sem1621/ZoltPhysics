//! Port of: Jolt/Core/Core.h
//! Status: partial
//!
//! Most of Core.h is platform/compiler detection, which Zig provides through `builtin`, and
//! macros that have a native Zig equivalent (see Docs/Zolt/PortingGuide.md). Only the
//! configuration values that ported code depends on are kept here.

const std = @import("std");
const builtin = @import("builtin");
const options = @import("zolt_options");

/// Version of Jolt that this port tracks (JPH_VERSION_MAJOR / MINOR / PATCH)
pub const version_major = 5;
pub const version_minor = 6;
pub const version_patch = 1;

/// Use f64 for world space positions (JPH_DOUBLE_PRECISION), set with `-Ddouble_precision=true`
pub const double_precision: bool = options.double_precision;

/// Use Jolt's fused multiply add sites (JPH_USE_FMADD), set with `-Dfma=true`. Off by default. Jolt has 8 such sites:
/// Vec4, Vec3 and DVec3 `fusedMultiplyAdd`, the scalar, Vec4 and Vec3 `differenceOfProducts` (and so
/// Vec3.crossPrecise), Quat.mul and Quat.multiplyImaginary. With the option they use @mulAdd where Jolt calls an FMA
/// intrinsic or std::fma. No other code may use @mulAdd. Results change: the C++ reference of the parity tests gets
/// -DJPH_USE_FMADD, so the parity proves this mode too. It needs a CPU with FMA (see the check below), because Zig's
/// software fma is not correctly rounded.
pub const use_fmadd: bool = options.fma;

comptime {
    if (use_fmadd) {
        switch (builtin.cpu.arch) {
            .x86, .x86_64 => if (!std.Target.x86.featureSetHas(builtin.cpu.features, .fma))
                @compileError("-Dfma=true needs a CPU with FMA (x86-64: -Dcpu=x86_64_v3 or native)"),
            .aarch64, .aarch64_be => {}, // Every aarch64 CPU has FMA
            else => @compileError("-Dfma=true needs a CPU with FMA (x86-64: -Dcpu=x86_64_v3 or native; aarch64)"),
        }
    }
}

/// Number of bits in an ObjectLayer (JPH_OBJECT_LAYER_BITS), set with `-Dobject_layer_bits=16|32`
pub const object_layer_bits: u8 = options.object_layer_bits;

/// Log the state of the simulation to find determinism issues (JPH_ENABLE_DETERMINISM_LOG, see
/// Physics/DeterminismLog.zig), set with `-Ddeterminism_log=true`
pub const determinism_log: bool = options.determinism_log;

/// Compile the debug renderer hooks into the library (JPH_DEBUG_RENDERER), set with `-Ddebug_renderer=true`. Without it
/// every hook is `void`: the draw vtable entries, the draw functions, the debug-only members and parameters
/// (Renderer/DebugRenderer.zig, Docs/Zolt/RemainingFeaturesArchitecture.md R2). The compiler then proves that drawing
/// cannot change a result. The parity reference gets the define with the option (R1).
pub const debug_renderer: bool = options.debug_renderer;

/// Record a timeline of the jobs and of the idle time of the job threads (Zolt, no Jolt option; see
/// Core/JobProfiler.zig), set with `-Djob_profile=true`. Without it every hook is `void`. Results do not change.
pub const job_profile: bool = options.job_profile;

/// Solve the contacts of a batch of a parallel split 8 at a time with SIMD (Zolt, no Jolt option; see
/// Physics/Constraints/ContactConstraintSimd.zig), on by default, off with `-Dsimd_contact_solver=false` for A/B tests.
/// Results do not change.
pub const simd_contact_solver: bool = options.simd_contact_solver;

/// Cache line size, used to avoid false sharing (JPH_CACHE_LINE_SIZE)
pub const cache_line_size = 64;

/// Alignment of Vec3 / Vec4 / UVec4 / Quat / Mat44 (JPH_VECTOR_ALIGNMENT)
pub const vector_alignment = 16;

/// Alignment of DVec3 / DMat44 (JPH_DVECTOR_ALIGNMENT)
pub const dvector_alignment = 32;

/// Default memory allocation alignment (JPH_DEFAULT_ALLOCATE_ALIGNMENT, which is
/// `__STDCPP_DEFAULT_NEW_ALIGNMENT__` = 16 on the 64 bit platforms Jolt supports). Zolt allocates through
/// `std.mem.Allocator`, which always takes an explicit alignment, so this only feeds the
/// `needs_aligned_allocate` constants that are kept for reference.
pub const default_allocate_alignment = 16;

/// True when asserts are evaluated (JPH_ENABLE_ASSERTS). In Zig this follows the optimize mode:
/// enabled in Debug and ReleaseSafe. Use it to guard code that only exists to feed asserts
/// (the equivalent of JPH_IF_ENABLE_ASSERTS).
pub const enable_asserts = std.debug.runtime_safety;

/// JPH_ASSERT at a site that Jolt's release build continues after, for the asserts that a test expects to fail
/// (Core/IssueReporting.zig, Docs/Zolt/RemainingFeaturesArchitecture.md R8). Re-exported here, so that a converted assert
/// stays one line: `if (Core.enable_asserts) Core.assertRecoverable(ok, "message", @src());`
pub const assertRecoverable = @import("IssueReporting.zig").assertRecoverable;

/// JPH_ASSERT(false, message) on a path that Jolt's release build continues on (see `assertRecoverable`)
pub const assertFailed = @import("IssueReporting.zig").assertFailed;

/// Swap the values behind `a` and `b` (std::swap). Use this instead of `std.mem.swap`. In Zig 0.16 `std.mem.swap`
/// swaps byte by byte. That costs more instructions, and a value that is read back right after the byte stores stalls
/// the store forwarding. This version copies the whole value through a temporary. `a` and `b` may be the same pointer.
pub inline fn swap(comptime T: type, a: *T, b: *T) void {
    const tmp = a.*;
    a.* = b.*;
    b.* = tmp;
}

test "configuration" {
    try std.testing.expect(object_layer_bits == 16 or object_layer_bits == 32);
}

test "swap gives the same result as std.mem.swap" {
    // No padding bytes, so the bytes compare equal
    const S = extern struct { a: u32, b: f32, c: [3]u16, d: u16, e: u64 };
    var prng = std.Random.DefaultPrng.init(0x5eed);
    const random = prng.random();
    for (0..1000) |_| {
        // Scalars, a vector, an array and a struct
        inline for (.{ u8, u32, i64, f32, f64, usize, @Vector(4, f32), [7]u8, S }) |T| {
            var a: T = undefined;
            var b: T = undefined;
            random.bytes(std.mem.asBytes(&a));
            random.bytes(std.mem.asBytes(&b));
            var a_ref = a;
            var b_ref = b;
            swap(T, &a, &b);
            std.mem.swap(T, &a_ref, &b_ref);
            try std.testing.expectEqualSlices(u8, std.mem.asBytes(&a_ref), std.mem.asBytes(&a));
            try std.testing.expectEqualSlices(u8, std.mem.asBytes(&b_ref), std.mem.asBytes(&b));
        }

        // Two elements of the same array, also the same element twice
        var items: [8]u32 = undefined;
        random.bytes(std.mem.asBytes(&items));
        var items_ref = items;
        const i = random.uintLessThan(usize, items.len);
        const j = random.uintLessThan(usize, items.len);
        swap(u32, &items[i], &items[j]);
        if (i != j)
            std.mem.swap(u32, &items_ref[i], &items_ref[j]);
        try std.testing.expectEqualSlices(u32, &items_ref, &items);
    }
}
