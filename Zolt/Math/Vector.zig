//! Port of: Jolt/Math/Vector.h
//! Status: complete
//!
//! Compound assignment operators (`*=`, `/=`, `+=`, `-=`) are expressed with the binary methods
//! (`v = v.add(other)`), which perform the same floating point operations.

const std = @import("std");
const math = @import("Math.zig");

/// Templatized vector class
pub fn Vector(comptime rows: u32) type {
    return extern struct {
        const Self = @This();

        /// Number of rows (the `Rows` template parameter), usable at comptime
        pub const row_count: u32 = rows;

        f32s: [rows]f32,

        /// Dimensions
        pub fn getRows(self: Self) u32 {
            _ = self;
            return rows;
        }

        /// Vector with all zeros
        pub fn setZero(self: *Self) void {
            for (0..rows) |r|
                self.f32s[r] = 0.0;
        }

        pub fn zero() Self {
            var v: Self = undefined;
            v.setZero();
            return v;
        }

        /// Copy a (part) of another vector into this vector.
        /// `v` can be any vector type with a `getComponent(index)` method (Vector, Vec3, Vec4, ...).
        pub fn copyPart(self: *Self, v: anytype, source_row: u32, num_rows: u32, dest_row: u32) void {
            for (0..num_rows) |r_usize| {
                const r: u32 = @intCast(r_usize);
                self.f32s[dest_row + r] = v.getComponent(source_row + r);
            }
        }

        /// Get float component by index (operator [] const)
        pub fn getComponent(self: Self, coordinate: u32) f32 {
            std.debug.assert(coordinate < rows);
            return self.f32s[coordinate];
        }

        /// Set float component by index (operator [])
        pub fn setComponent(self: *Self, coordinate: u32, v: f32) void {
            std.debug.assert(coordinate < rows);
            self.f32s[coordinate] = v;
        }

        /// Comparison (operator ==, use !eql for operator !=)
        pub fn eql(self: Self, v2: Self) bool {
            for (0..rows) |r|
                if (self.f32s[r] != v2.f32s[r])
                    return false;
            return true;
        }

        /// Test if vector consists of all zeros
        pub fn isZero(self: Self) bool {
            for (0..rows) |r|
                if (self.f32s[r] != 0.0)
                    return false;
            return true;
        }

        /// Test if vector is near zero
        pub fn isNearZero(self: Self, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
            return self.lengthSq() <= opts.max_dist_sq;
        }

        /// Test if two vectors are close to each other
        pub fn isClose(self: Self, v2: Self, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
            return v2.sub(self).lengthSq() <= opts.max_dist_sq;
        }

        /// Multiply vector with float (operator * (float) and operator * (float, Vector))
        pub fn mulScalar(self: Self, v2: f32) Self {
            var v: Self = undefined;
            for (0..rows) |r|
                v.f32s[r] = self.f32s[r] * v2;
            return v;
        }

        /// Divide vector by float (operator / (float))
        pub fn divScalar(self: Self, v2: f32) Self {
            var v: Self = undefined;
            for (0..rows) |r|
                v.f32s[r] = self.f32s[r] / v2;
            return v;
        }

        /// Add two float vectors (component wise) (operator +)
        pub fn add(self: Self, v2: Self) Self {
            var v: Self = undefined;
            for (0..rows) |r|
                v.f32s[r] = self.f32s[r] + v2.f32s[r];
            return v;
        }

        /// Negate (operator - ()). Like Jolt, this flips the sign of each component (0 becomes -0),
        /// unlike Vec3 / Vec4 which compute 0 - v.
        pub fn negate(self: Self) Self {
            var v: Self = undefined;
            for (0..rows) |r|
                v.f32s[r] = -self.f32s[r];
            return v;
        }

        /// Subtract two float vectors (component wise) (operator -)
        pub fn sub(self: Self, v2: Self) Self {
            var v: Self = undefined;
            for (0..rows) |r|
                v.f32s[r] = self.f32s[r] - v2.f32s[r];
            return v;
        }

        /// Dot product
        pub fn dot(self: Self, v2: Self) f32 {
            var result: f32 = 0.0;
            for (0..rows) |r|
                result += self.f32s[r] * v2.f32s[r];
            return result;
        }

        /// Squared length of vector
        pub fn lengthSq(self: Self) f32 {
            return self.dot(self);
        }

        /// Length of vector
        pub fn length(self: Self) f32 {
            return math.sqrt(self.lengthSq());
        }

        /// Test if length^2 of this vector is within the range [1 - inTolerance, 1 + inTolerance]
        pub fn isNormalized(self: Self, opts: struct { tolerance_sq: f32 = 1.0e-6 }) bool {
            return @abs(self.lengthSq() - 1.0) <= opts.tolerance_sq;
        }

        /// Normalize vector
        pub fn normalized(self: Self) Self {
            return self.divScalar(self.length());
        }

        /// To String
        pub fn format(self: Self, writer: *std.Io.Writer) std.Io.Writer.Error!void {
            try writer.writeAll("[");
            for (0..rows - 1) |i|
                try writer.print("{d}, ", .{self.f32s[i]});
            try writer.print("{d}]", .{self.f32s[rows - 1]});
        }
    };
}

test "Vector size and layout" {
    // Jolt reinterprets a Vector<3> as a Float3, so the layout must be exactly the array of floats
    try std.testing.expectEqual(3 * @sizeOf(f32), @sizeOf(Vector(3)));
    try std.testing.expectEqual(@alignOf(f32), @alignOf(Vector(3)));
}

test "Vector functions" {
    // Generic functions are only type checked when instantiated, so call every function here
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;
    const Vec3 = @import("Vec3.zig").Vec3;

    const V = Vector(4);
    var v = V.zero();
    try expectEqual(4, V.row_count);
    try expectEqual(4, v.getRows());
    try expect(v.isZero());
    try expect(v.isNearZero(.{}));

    // Copy from another Vector and from a Vec3
    v.copyPart(Vector(2){ .f32s = .{ 3, 4 } }, 0, 2, 1);
    try expect(v.eql(.{ .f32s = .{ 0, 3, 4, 0 } }));
    var v3 = Vector(3).zero();
    v3.copyPart(Vec3.init(1, 2, 3), 1, 2, 0);
    try expect(v3.eql(.{ .f32s = .{ 2, 3, 0 } }));

    try expect(!v.isZero());
    try expect(!v.isNearZero(.{ .max_dist_sq = 1.0 }));
    try expectEqual(25.0, v.dot(v));
    try expectEqual(25.0, v.lengthSq());
    try expectEqual(5.0, v.length());

    const n = v.normalized();
    try expect(n.eql(v.divScalar(5.0)));
    try expect(n.isNormalized(.{}));
    try expect(!v.isNormalized(.{ .tolerance_sq = 1.0 }));
    try expect(n.isClose(.{ .f32s = .{ 0, 0.6, 0.8, 0 } }, .{}));
    try expect(!n.isClose(.{ .f32s = .{ 0, 0.6, 0.8, 0.1 } }, .{ .max_dist_sq = 1.0e-3 }));
    try expect(v.sub(v).isZero());
    try expect(v.add(v).eql(v.mulScalar(2.0)));

    // Negate flips the sign bit, also for zero
    const neg = v.negate();
    try expect(neg.eql(.{ .f32s = .{ -0.0, -3, -4, -0.0 } }));
    try expect(std.math.signbit(neg.f32s[0]));

    try expectEqual(-3.0, neg.getComponent(1));
    v.setComponent(3, 7);
    try expectEqual(7.0, v.f32s[3]);
}
