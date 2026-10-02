//! Port of: Jolt/Core/Core.h
//! Status: partial
//!
//! Most of Core.h is platform/compiler detection, which Zig provides through `builtin`, and
//! macros that have a native Zig equivalent (see Docs/Zolt/PortingGuide.md). Only the
//! configuration values that ported code depends on are kept here.

const std = @import("std");
const options = @import("zolt_options");

/// Version of Jolt that this port tracks (JPH_VERSION_MAJOR / MINOR / PATCH)
pub const version_major = 5;
pub const version_minor = 6;
pub const version_patch = 1;

/// Use f64 for world space positions (JPH_DOUBLE_PRECISION), set with `-Ddouble_precision=true`
pub const double_precision: bool = options.double_precision;

/// Number of bits in an ObjectLayer (JPH_OBJECT_LAYER_BITS), set with `-Dobject_layer_bits=16|32`
pub const object_layer_bits: u8 = options.object_layer_bits;

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

test "configuration" {
    try std.testing.expect(object_layer_bits == 16 or object_layer_bits == 32);
}
