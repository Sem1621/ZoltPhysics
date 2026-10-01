//! Port of: UnitTests/UnitTestFramework.h, UnitTests/UnitTestFramework.cpp
//! Status: partial
//! Missing: ExpectAssert (Zig asserts panic, tests that rely on it are skipped)
//!
//! Helpers shared by the ported unit tests. Import with:
//!     const fw = @import("../UnitTestFramework.zig");

const std = @import("std");
const zolt = @import("zolt");

/// Define the exact random number generator we want to use across platforms for consistency
/// (UnitTestRandom, std::mt19937 in Jolt)
pub const UnitTestRandom = zolt.Mt19937;

pub const expect = std.testing.expect;
pub const expectEqual = std.testing.expectEqual;

pub const ApproxOptions = struct { tolerance: f64 = 1.0e-6 };

/// CHECK_APPROX_EQUAL(lhs, rhs, tolerance)
///
/// Supports f32, f64, Float2, Quat (q and -q are considered equal) and every type with an
/// `isClose(other, .{ .max_dist_sq = ... })` method (Vec3, Vec4, Mat44, DVec3, DMat44, ...).
pub fn checkApproxEqual(lhs: anytype, rhs: @TypeOf(lhs), opts: ApproxOptions) !void {
    if (!isApproxEqual(lhs, rhs, opts)) {
        std.debug.print("checkApproxEqual failed (tolerance {d}):\n  lhs: {any}\n  rhs: {any}\n", .{ opts.tolerance, lhs, rhs });
        return error.TestExpectedApproxEqual;
    }
}

/// Same comparison as `checkApproxEqual`, returns the result instead of failing the test
pub fn isApproxEqual(lhs: anytype, rhs: @TypeOf(lhs), opts: ApproxOptions) bool {
    const T = @TypeOf(lhs);
    return switch (@typeInfo(T)) {
        .float => @abs(rhs - lhs) <= @as(T, @floatCast(opts.tolerance)),
        .comptime_float => @abs(rhs - lhs) <= opts.tolerance,
        else => isCloseValue(lhs, rhs, opts.tolerance),
    };
}

fn isCloseValue(lhs: anytype, rhs: @TypeOf(lhs), tolerance: f64) bool {
    const T = @TypeOf(lhs);
    if (T == zolt.Float2) {
        const dx = lhs.x - rhs.x;
        const dy = lhs.y - rhs.y;
        const tol: f32 = @floatCast(tolerance);
        return dx * dx + dy * dy < tol * tol;
    }
    if (!@hasDecl(T, "isClose")) @compileError("checkApproxEqual: " ++ @typeName(T) ++ " has no isClose method");

    // isClose takes the max squared distance in the precision of the type
    const max_dist_sq = tolerance * tolerance;
    if (@hasDecl(zolt, "Quat") and T == zolt.Quat) {
        // A quaternion and its negation represent the same rotation
        const tol_sq: f32 = @floatCast(max_dist_sq);
        return lhs.isClose(rhs, .{ .max_dist_sq = tol_sq }) or lhs.isClose(rhs.negate(), .{ .max_dist_sq = tol_sq });
    }
    const Opts = @typeInfo(@TypeOf(T.isClose)).@"fn".params[2].type.?;
    const DistType = @FieldType(Opts, "max_dist_sq");
    return lhs.isClose(rhs, .{ .max_dist_sq = @as(DistType, @floatCast(max_dist_sq)) });
}

test "checkApproxEqual" {
    try checkApproxEqual(@as(f32, 1.0), 1.0000001, .{});
    try expect(!isApproxEqual(@as(f32, 1.0), 1.1, .{}));
    try checkApproxEqual(zolt.Vec3.init(1, 2, 3), zolt.Vec3.init(1, 2, 3.0000001), .{});
    try checkApproxEqual(zolt.Vec4.init(1, 2, 3, 4), zolt.Vec4.init(1, 2, 3, 4.01), .{ .tolerance = 0.1 });
}
