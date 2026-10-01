//! Port of: Jolt/Math/FindRoot.h
//! Status: complete

const std = @import("std");
const math = @import("Math.zig");

/// Result of `findRoot` (the return value and the outX1 / outX2 out parameters of FindRoot)
pub fn FindRootResult(comptime T: type) type {
    return struct {
        /// The number of roots. When 0, `x1` and `x2` are undefined.
        num_roots: i32,
        x1: T,
        x2: T,
    };
}

/// Find the roots of \f$inA \: x^2 + inB \: x + inC = 0\f$.
/// @return The number of roots, actual roots in outX1 and outX2.
/// If number of roots returned is 1 then outX1 == outX2.
pub fn findRoot(comptime T: type, a: T, b: T, c: T) FindRootResult(T) {
    if (T != f32 and T != f64) @compileError("findRoot: T must be f32 or f64");

    // Check if this is a linear equation
    if (a == 0) {
        // Check if this is a constant equation
        if (b == 0) {
            if (c == 0)
                return .{ .num_roots = 1, .x1 = 0.0, .x2 = 0.0 }; // Actually infinitely many solutions

            return .{ .num_roots = 0, .x1 = undefined, .x2 = undefined };
        }

        // Linear equation with 1 solution
        const x = -c / b;
        return .{ .num_roots = 1, .x1 = x, .x2 = x };
    }

    // See Numerical Recipes in C, Chapter 5.6 Quadratic and Cubic Equations
    // Note that Jolt only has a float version of DifferenceOfProducts, so for T = double the arguments are converted to float.
    const det: T = if (T == f32)
        math.differenceOfProducts(b, b, 4 * a, c)
    else
        math.differenceOfProducts(@floatCast(b), @floatCast(b), @floatCast(4 * a), @floatCast(c));
    if (det < 0)
        return .{ .num_roots = 0, .x1 = undefined, .x2 = undefined };
    const q = (b + math.sign(b) * math.sqrt(det)) / -2;
    const x1 = q / a;
    if (q == 0)
        return .{ .num_roots = 1, .x1 = x1, .x2 = x1 };
    return .{ .num_roots = 2, .x1 = x1, .x2 = c / q };
}

test "findRoot" {
    // x^2 - 3x + 2 = (x - 1) * (x - 2)
    const r2 = findRoot(f32, 1.0, -3.0, 2.0);
    try std.testing.expectEqual(@as(i32, 2), r2.num_roots);
    try std.testing.expectEqual(@as(f32, 2.0), r2.x1);
    try std.testing.expectEqual(@as(f32, 1.0), r2.x2);

    // x^2 - 2x + 1 = (x - 1)^2
    const r1 = findRoot(f32, 1.0, -2.0, 1.0);
    try std.testing.expectEqual(@as(i32, 2), r1.num_roots); // q != 0 so both (equal) roots are returned
    try std.testing.expectEqual(@as(f32, 1.0), r1.x1);
    try std.testing.expectEqual(@as(f32, 1.0), r1.x2);

    // x^2 = 0, q == 0
    const r0 = findRoot(f32, 1.0, 0.0, 0.0);
    try std.testing.expectEqual(@as(i32, 1), r0.num_roots);
    try std.testing.expectEqual(@as(f32, 0.0), r0.x1);
    try std.testing.expectEqual(@as(f32, 0.0), r0.x2);

    // x^2 + 1 = 0 has no real roots
    try std.testing.expectEqual(@as(i32, 0), findRoot(f32, 1.0, 0.0, 1.0).num_roots);

    // Linear: 2x + 4 = 0
    const lin = findRoot(f32, 0.0, 2.0, 4.0);
    try std.testing.expectEqual(@as(i32, 1), lin.num_roots);
    try std.testing.expectEqual(@as(f32, -2.0), lin.x1);
    try std.testing.expectEqual(@as(f32, -2.0), lin.x2);

    // Constant: 0 = 0 and 1 = 0
    const all = findRoot(f32, 0.0, 0.0, 0.0);
    try std.testing.expectEqual(@as(i32, 1), all.num_roots);
    try std.testing.expectEqual(@as(f32, 0.0), all.x1);
    try std.testing.expectEqual(@as(i32, 0), findRoot(f32, 0.0, 0.0, 1.0).num_roots);

    // Double precision: 2x^2 + 2x - 4 = 2 * (x - 1) * (x + 2)
    const rd = findRoot(f64, 2.0, 2.0, -4.0);
    try std.testing.expectEqual(@as(i32, 2), rd.num_roots);
    try std.testing.expectEqual(@as(f64, -2.0), rd.x1);
    try std.testing.expectEqual(@as(f64, 1.0), rd.x2);
}
