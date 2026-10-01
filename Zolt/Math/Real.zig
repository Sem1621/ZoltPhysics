//! Port of: Jolt/Math/Real.h
//! Status: complete
//!
//! The 'real' types switch between float and double precision with `Core.double_precision`
//! (`-Ddouble_precision=true`, JPH_DOUBLE_PRECISION).
//!
//! - `RVec3Arg` / `RMat44Arg` are not needed: Zig decides itself how to pass parameters, use `RVec3` / `RMat44`.
//! - The `_r` literal operator (`using namespace JPH::literals; 1.0_r`) is not needed either: a float
//!   literal coerces to `Real` (`const x: Real = 1.0;`, `@as(Real, 1.0)`).

const std = @import("std");
const Core = @import("../Core/Core.zig");
const Double3 = @import("Double3.zig").Double3;
const DMat44 = @import("DMat44.zig").DMat44;
const DVec3 = @import("DVec3.zig").DVec3;
const Float3 = @import("Float3.zig").Float3;
const Mat44 = @import("Mat44.zig").Mat44;
const Vec3 = @import("Vec3.zig").Vec3;

/// Real is double (JPH_DOUBLE_PRECISION) or float
pub const Real = if (Core.double_precision) f64 else f32;

/// Real3 is Double3 (JPH_DOUBLE_PRECISION) or Float3
pub const Real3 = if (Core.double_precision) Double3 else Float3;

/// RVec3 is DVec3 (JPH_DOUBLE_PRECISION) or Vec3
pub const RVec3 = if (Core.double_precision) DVec3 else Vec3;

/// RMat44 is DMat44 (JPH_DOUBLE_PRECISION) or Mat44
pub const RMat44 = if (Core.double_precision) DMat44 else Mat44;

/// Alignment of RVec3 (JPH_RVECTOR_ALIGNMENT)
pub const rvector_alignment = if (Core.double_precision) Core.dvector_alignment else Core.vector_alignment;

test "Real types follow double_precision" {
    try std.testing.expect(@alignOf(RVec3) == rvector_alignment);
    const x: Real = 1.5; // Float literals coerce to Real, no _r operator needed
    const v = RVec3.init(x, 2.0, 3.0);
    try std.testing.expectEqual(x, v.getX());
    try std.testing.expect(RMat44.translation(v).getTranslation().eql(v));
    try std.testing.expect(RMat44.identity().toMat44().eql(Mat44.identity()));
    try std.testing.expectEqual(@sizeOf(Real) * 3, @sizeOf(Real3));
}
