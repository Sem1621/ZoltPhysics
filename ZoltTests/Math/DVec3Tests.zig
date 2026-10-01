//! Port of: UnitTests/Math/DVec3Tests.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const DVec3 = zolt.DVec3;
const Double3 = zolt.Double3;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

fn bitsOf(v: f64) u64 {
    return @bitCast(v);
}

fn doubleFromBits(v: u64) f64 {
    return @bitCast(v);
}

test "TestDVec3Zero" {
    const v = DVec3.zero();

    try expectEqual(0, v.getX());
    try expectEqual(0, v.getY());
    try expectEqual(0, v.getZ());
}

test "TestDVec3Axis" {
    try expect(DVec3.axisX().eql(DVec3.init(1, 0, 0)));
    try expect(DVec3.axisY().eql(DVec3.init(0, 1, 0)));
    try expect(DVec3.axisZ().eql(DVec3.init(0, 0, 1)));
}

test "TestVec3NaN" {
    var v = DVec3.nan();

    try expect(std.math.isNan(v.getX()));
    try expect(std.math.isNan(v.getY()));
    try expect(std.math.isNan(v.getZ()));
    try expect(v.isNaN());

    v.setComponent(0, 0);
    try expect(v.isNaN());
    v.setComponent(1, 0);
    try expect(v.isNaN());
    v.setComponent(2, 0);
    try expect(!v.isNaN());
}

test "TestDVec3ConstructComponents" {
    var v = DVec3.init(1, 2, 3);

    // Test component access
    try expectEqual(1, v.getX());
    try expectEqual(2, v.getY());
    try expectEqual(3, v.getZ());

    // Test component access by [] operators
    try expectEqual(1, v.getComponent(0));
    try expectEqual(2, v.getComponent(1));
    try expectEqual(3, v.getComponent(2));

    // Test == and != operators
    try expect(v.eql(DVec3.init(1, 2, 3)));
    try expect(!v.eql(DVec3.init(1, 2, 4)));

    // Set the components
    v.setComponent(0, 4);
    v.setComponent(1, 5);
    v.setComponent(2, 6);
    try expect(v.eql(DVec3.init(4, 5, 6)));

    // Set the components again
    v.setX(7);
    v.setY(8);
    v.setZ(9);
    try expect(v.eql(DVec3.init(7, 8, 9)));

    // Set all components
    v.set(10, 11, 12);
    try expect(v.eql(DVec3.init(10, 11, 12)));
}

test "TestVec4ToDVec3" {
    try expect(DVec3.fromVec4(Vec4.init(1, 3, 5, 7)).eql(DVec3.init(1, 3, 5)));
}

test "TestDVec3Replicate" {
    try expect(DVec3.replicate(2).eql(DVec3.init(2, 2, 2)));
}

test "TestDVec3ToVec3" {
    try expect(DVec3.init(1, 3, 5).toVec3().eql(Vec3.init(1, 3, 5)));

    // Check rounding up and down
    try expect(DVec3.init(2.0, 0x1.0000000000001p1, -0x1.0000000000001p1).toVec3RoundUp().eql(Vec3.init(2.0, 0x1.000002p1, -2.0)));
    try expect(DVec3.init(2.0, 0x1.0000000000001p1, -0x1.0000000000001p1).toVec3RoundDown().eql(Vec3.init(2.0, 2.0, -0x1.000002p1)));
}

test "TestVec3MinMax" {
    const v1 = DVec3.init(1, 5, 3);
    const v2 = DVec3.init(4, 2, 6);

    try expect(DVec3.min(v1, v2).eql(DVec3.init(1, 2, 3)));
    try expect(DVec3.max(v1, v2).eql(DVec3.init(4, 5, 6)));
}

test "TestDVec3Clamp" {
    const v1 = DVec3.init(1, 2, 3);
    const v2 = DVec3.init(4, 5, 6);
    const v = DVec3.init(-1, 3, 7);

    try expect(DVec3.clamp(v, v1, v2).eql(DVec3.init(1, 3, 6)));
}

test "TestDVec3Trues" {
    const t = DVec3.true_value;
    const f = DVec3.false_value;

    try expectEqual(0b0000, DVec3.init(f, f, f).getTrues());
    try expectEqual(0b0001, DVec3.init(t, f, f).getTrues());
    try expectEqual(0b0010, DVec3.init(f, t, f).getTrues());
    try expectEqual(0b0011, DVec3.init(t, t, f).getTrues());
    try expectEqual(0b0100, DVec3.init(f, f, t).getTrues());
    try expectEqual(0b0101, DVec3.init(t, f, t).getTrues());
    try expectEqual(0b0110, DVec3.init(f, t, t).getTrues());
    try expectEqual(0b0111, DVec3.init(t, t, t).getTrues());

    try expect(!DVec3.init(f, f, f).testAnyTrue());
    try expect(DVec3.init(t, f, f).testAnyTrue());
    try expect(DVec3.init(f, t, f).testAnyTrue());
    try expect(DVec3.init(t, t, f).testAnyTrue());
    try expect(DVec3.init(f, f, t).testAnyTrue());
    try expect(DVec3.init(t, f, t).testAnyTrue());
    try expect(DVec3.init(f, t, t).testAnyTrue());
    try expect(DVec3.init(t, t, t).testAnyTrue());

    try expect(!DVec3.init(f, f, f).testAllTrue());
    try expect(!DVec3.init(t, f, f).testAllTrue());
    try expect(!DVec3.init(f, t, f).testAllTrue());
    try expect(!DVec3.init(t, t, f).testAllTrue());
    try expect(!DVec3.init(f, f, t).testAllTrue());
    try expect(!DVec3.init(t, f, t).testAllTrue());
    try expect(!DVec3.init(f, t, t).testAllTrue());
    try expect(DVec3.init(t, t, t).testAllTrue());
}

test "TestDVec3Comparisons" {
    try expectEqual(0b101, DVec3.equals(DVec3.init(1, 2, 3), DVec3.init(1, 4, 3)).getTrues()); // Can't directly check if equal to (true, false, true) because true = -NaN and -NaN != -NaN
    try expectEqual(0b010, DVec3.less(DVec3.init(1, 2, 4), DVec3.init(1, 4, 3)).getTrues());
    try expectEqual(0b011, DVec3.lessOrEqual(DVec3.init(1, 2, 4), DVec3.init(1, 4, 3)).getTrues());
    try expectEqual(0b100, DVec3.greater(DVec3.init(1, 2, 4), DVec3.init(1, 4, 3)).getTrues());
    try expectEqual(0b101, DVec3.greaterOrEqual(DVec3.init(1, 2, 4), DVec3.init(1, 4, 3)).getTrues());
}

test "TestDVec3FMA" {
    try expect(DVec3.fusedMultiplyAdd(DVec3.init(1, 2, 3), DVec3.init(4, 5, 6), DVec3.init(7, 8, 9)).eql(DVec3.init(1 * 4 + 7, 2 * 5 + 8, 3 * 6 + 9)));
}

test "TestDVec3Select" {
    const true2 = doubleFromBits(@as(u64, 1) << 63);
    const false2 = doubleFromBits(~@as(u64, 0) >> 1);
    const t = DVec3.true_value;
    const f = DVec3.false_value;

    try expect(DVec3.select(DVec3.init(1, 2, 3), DVec3.init(4, 5, 6), DVec3.init(t, f, t)).eql(DVec3.init(4, 2, 6)));
    try expect(DVec3.select(DVec3.init(1, 2, 3), DVec3.init(4, 5, 6), DVec3.init(f, t, f)).eql(DVec3.init(1, 5, 3)));
    try expect(DVec3.select(DVec3.init(1, 2, 3), DVec3.init(4, 5, 6), DVec3.init(true2, false2, true2)).eql(DVec3.init(4, 2, 6)));
    try expect(DVec3.select(DVec3.init(1, 2, 3), DVec3.init(4, 5, 6), DVec3.init(false2, true2, false2)).eql(DVec3.init(1, 5, 3)));
}

test "TestDVec3BitOps" {
    // Test all bit permutations
    const v1 = DVec3.init(doubleFromBits(0b0011), doubleFromBits(0b00110), doubleFromBits(0b001100));
    const v2 = DVec3.init(doubleFromBits(0b0101), doubleFromBits(0b01010), doubleFromBits(0b010100));

    try expect(DVec3.bitOr(v1, v2).eql(DVec3.init(doubleFromBits(0b0111), doubleFromBits(0b01110), doubleFromBits(0b011100))));
    try expect(DVec3.bitXor(v1, v2).eql(DVec3.init(doubleFromBits(0b0110), doubleFromBits(0b01100), doubleFromBits(0b011000))));
    try expect(DVec3.bitAnd(v1, v2).eql(DVec3.init(doubleFromBits(0b0001), doubleFromBits(0b00010), doubleFromBits(0b000100))));
}

test "TestDVec3Close" {
    try expect(DVec3.init(1, 2, 3).isClose(DVec3.init(1.001, 2.001, 3.001), .{ .max_dist_sq = 1.0e-4 }));
    try expect(!DVec3.init(1, 2, 3).isClose(DVec3.init(1.001, 2.001, 3.001), .{ .max_dist_sq = 1.0e-6 }));

    try expect(DVec3.init(1.001, 0, 0).isNormalized(.{ .tolerance = 1.0e-2 }));
    try expect(!DVec3.init(0, 1.001, 0).isNormalized(.{ .tolerance = 1.0e-4 }));

    try expect(DVec3.init(-1.0e-7, 1.0e-7, 1.0e-8).isNearZero(.{ .max_dist_sq = 1.0e-12 }));
    try expect(!DVec3.init(-1.0e-7, 1.0e-7, -1.0e-5).isNearZero(.{ .max_dist_sq = 1.0e-12 }));
}

test "TestDVec3Operators" {
    try expect(DVec3.init(1, 2, 3).negate().eql(DVec3.init(-1, -2, -3)));

    const neg_zero = DVec3.zero().negate();
    try expect(neg_zero.eql(DVec3.zero()));

    // When cross platform deterministic, we want to make sure that -0 is represented as 0
    try expectEqual(0, bitsOf(neg_zero.getX()));
    try expectEqual(0, bitsOf(neg_zero.getY()));
    try expectEqual(0, bitsOf(neg_zero.getZ()));

    try expect(DVec3.init(1, 2, 3).addVec3(Vec3.init(4, 5, 6)).eql(DVec3.init(5, 7, 9)));
    try expect(DVec3.init(1, 2, 3).subVec3(Vec3.init(6, 5, 4)).eql(DVec3.init(-5, -3, -1)));

    try expect(DVec3.init(1, 2, 3).add(DVec3.init(4, 5, 6)).eql(DVec3.init(5, 7, 9)));
    try expect(DVec3.init(1, 2, 3).sub(DVec3.init(6, 5, 4)).eql(DVec3.init(-5, -3, -1)));

    try expect(DVec3.init(1, 2, 3).mul(DVec3.init(4, 5, 6)).eql(DVec3.init(4, 10, 18)));
    try expect(DVec3.init(1, 2, 3).mulScalar(2).eql(DVec3.init(2, 4, 6)));
    try expect(DVec3.init(1, 2, 3).mulScalar(4).eql(DVec3.init(4, 8, 12))); // 4 * DVec3(1, 2, 3)

    try expect(DVec3.init(1, 2, 3).divScalar(2).eql(DVec3.init(0.5, 1.0, 1.5)));
    try expect(DVec3.init(1, 2, 3).div(DVec3.init(2, 8, 24)).eql(DVec3.init(0.5, 0.25, 0.125)));

    var v = DVec3.init(1, 2, 3);
    v = v.mul(DVec3.init(4, 5, 6));
    try expect(v.eql(DVec3.init(4, 10, 18)));
    v = v.mulScalar(2);
    try expect(v.eql(DVec3.init(8, 20, 36)));
    v = v.divScalar(2);
    try expect(v.eql(DVec3.init(4, 10, 18)));
    v = v.add(DVec3.init(1, 2, 3));
    try expect(v.eql(DVec3.init(5, 12, 21)));
    v = v.sub(DVec3.init(1, 2, 3));
    try expect(v.eql(DVec3.init(4, 10, 18)));
    v = v.addVec3(Vec3.init(1, 2, 3));
    try expect(v.eql(DVec3.init(5, 12, 21)));
    v = v.subVec3(Vec3.init(1, 2, 3));
    try expect(v.eql(DVec3.init(4, 10, 18)));

    try expect(DVec3.init(2, 4, 8).reciprocal().eql(DVec3.init(0.5, 0.25, 0.125)));
}

test "TestDVec3Abs" {
    try expect(DVec3.init(1, -2, 3).abs().eql(DVec3.init(1, 2, 3)));
    try expect(DVec3.init(-1, 2, -3).abs().eql(DVec3.init(1, 2, 3)));
}

test "TestDVec3Dot" {
    try expectEqual(@as(f64, 2 * 5 + 3 * 6 + 4 * 7), DVec3.init(2, 3, 4).dot(DVec3.init(5, 6, 7)));
}

test "TestDVec3Length" {
    try expectEqual(@as(f64, 4 + 9 + 16), DVec3.init(2, 3, 4).lengthSq());
    try expectEqual(zolt.math.sqrt(@as(f64, 4 + 9 + 16)), DVec3.init(2, 3, 4).length());
}

test "TestDVec3Sqrt" {
    try expect(DVec3.init(13, 15, 17).sqrt().eql(DVec3.init(zolt.math.sqrt(@as(f64, 13.0)), zolt.math.sqrt(@as(f64, 15.0)), zolt.math.sqrt(@as(f64, 17.0)))));
}

test "TestDVec3Equals" {
    try expect(!DVec3.init(13, 15, 17).eql(DVec3.init(13, 15, 19)));
    try expect(DVec3.init(13, 15, 17).eql(DVec3.init(13, 15, 17)));
    try expect(!DVec3.init(13, 15, 17).eql(DVec3.init(13, 15, 19)));
}

test "TestDVec3LoadStoreDouble3Unsafe" {
    const d4 = [4]f64{ 1, 2, 3, 4 };
    const d3: *const Double3 = @ptrCast(&d4);
    const v = DVec3.loadDouble3Unsafe(d3);
    const v2 = DVec3.init(1, 2, 3);
    try expect(v.eql(v2));

    var d3_out: Double3 = undefined;
    DVec3.init(1, 2, 3).storeDouble3(&d3_out);
    try expect(d3.eql(d3_out));
}

test "TestDVec3StoreDouble3EdgeCases" {
    // Negative, zero, and large values to stress SIMD lane handling
    {
        const v = DVec3.init(-1.23456789, 0.0, 1.23456789e10);
        var out: Double3 = undefined;
        v.storeDouble3(&out);
        try expectEqual(v.getX(), out.x);
        try expectEqual(v.getY(), out.y);
        try expectEqual(v.getZ(), out.z);
    }

    // Negative zero can expose SIMD mask bugs
    {
        const v = DVec3.init(-0.0, -0.0, -0.0);
        var out: Double3 = undefined;
        v.storeDouble3(&out);
        try expectEqual(bitsOf(v.getX()), bitsOf(out.x));
        try expectEqual(bitsOf(v.getY()), bitsOf(out.y));
        try expectEqual(bitsOf(v.getZ()), bitsOf(out.z));
    }
}

test "TestDVec3Cross" {
    try expect(DVec3.init(1, 0, 0).cross(DVec3.init(0, 1, 0)).eql(DVec3.init(0, 0, 1)));
    try expect(DVec3.init(0, 1, 0).cross(DVec3.init(1, 0, 0)).eql(DVec3.init(0, 0, -1)));
    try expect(DVec3.init(0, 1, 0).cross(DVec3.init(0, 0, 1)).eql(DVec3.init(1, 0, 0)));
    try expect(DVec3.init(0, 0, 1).cross(DVec3.init(0, 1, 0)).eql(DVec3.init(-1, 0, 0)));
    try expect(DVec3.init(0, 0, 1).cross(DVec3.init(1, 0, 0)).eql(DVec3.init(0, 1, 0)));
    try expect(DVec3.init(1, 0, 0).cross(DVec3.init(0, 0, 1)).eql(DVec3.init(0, -1, 0)));
}

test "TestDVec3Normalize" {
    try expect(DVec3.init(3, 2, 1).normalized().eql(DVec3.init(3, 2, 1).divScalar(zolt.math.sqrt(@as(f64, 9.0 + 4.0 + 1.0)))));
}

test "TestDVec3Sign" {
    const true_min = std.math.floatTrueMin(f64);
    const inf = std.math.inf(f64);

    try expect(DVec3.init(1.2345, -6.7891, 0).getSign().eql(DVec3.init(1, -1, 1)));
    try expect(DVec3.init(0, 2.3456, -7.8912).getSign().eql(DVec3.init(1, 1, -1)));
    try expect(DVec3.init(-0.0, 0.0, -0.0).getSign().eql(DVec3.init(-1, 1, -1)));
    try expect(DVec3.init(1.0, -1.0, 1.0).getSign().eql(DVec3.init(1, -1, 1)));
    try expect(DVec3.init(true_min, -true_min, true_min).getSign().eql(DVec3.init(1, -1, 1))); // Denormal number
    try expect(DVec3.init(inf, -inf, inf).getSign().eql(DVec3.init(1, -1, 1)));
}

test "TestDVec3ConvertToString" {
    const v = DVec3.init(1, 2, 3);
    var buffer: [64]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3", try std.fmt.bufPrint(&buffer, "{f}", .{v}));
}
