//! Port of: UnitTests/Math/Vec3Tests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Float4 = zolt.Float4;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// std::uniform_real_distribution<float>(min, max) on UnitTestRandom
fn uniformFloat(random: *fw.UnitTestRandom, min: f32, max: f32) f32 {
    return fw.UniformFloatDistribution.init(min, max).next(random);
}

test "TestVec3ConstructComponents" {
    var v = Vec3.init(1, 2, 3);

    // Test component access
    try fw.expectEqual(1, v.getX());
    try fw.expectEqual(2, v.getY());
    try fw.expectEqual(3, v.getZ());

    // Test component access by [] operators
    try fw.expectEqual(1, v.getComponent(0));
    try fw.expectEqual(2, v.getComponent(1));
    try fw.expectEqual(3, v.getComponent(2));

    // Test == and != operators
    try fw.expect(v.eql(Vec3.init(1, 2, 3)));
    try fw.expect(!v.eql(Vec3.init(1, 2, 4)));

    // Set the components
    v.setComponent(0, 4);
    v.setComponent(1, 5);
    v.setComponent(2, 6);
    try fw.expect(v.eql(Vec3.init(4, 5, 6)));

    // Set the components
    v.setX(7);
    v.setY(8);
    v.setZ(9);
    try fw.expect(v.eql(Vec3.init(7, 8, 9)));

    // Set all components
    v.set(10, 11, 12);
    try fw.expect(v.eql(Vec3.init(10, 11, 12)));
}

test "TestVec3LoadStoreFloat3" {
    const f4 = [_]f32{ 1, 2, 3, 4 }; // Extra element since we read one too many in sLoadFloat3Unsafe
    const f3: *const Float3 = @ptrCast(&f4);
    try fw.expect(Vec3.fromFloat3(f3.*).eql(Vec3.init(1, 2, 3)));
    try fw.expect(Vec3.loadFloat3Unsafe(f3).eql(Vec3.init(1, 2, 3)));

    var f3_out: Float3 = undefined;
    Vec3.init(1, 2, 3).storeFloat3(&f3_out);
    try fw.expect(f3.eql(f3_out));
}

test "TestVec3ConstructVec4" {
    const v4 = Vec4.init(1, 2, 3, 4);
    try fw.expect(Vec3.fromVec4(v4).eql(Vec3.init(1, 2, 3)));
}

test "TestVec3Zero" {
    const v = Vec3.zero();

    try fw.expectEqual(0, v.getX());
    try fw.expectEqual(0, v.getY());
    try fw.expectEqual(0, v.getZ());
}

test "TestVec3NaN" {
    var v = Vec3.nan();

    try fw.expect(std.math.isNan(v.getX()));
    try fw.expect(std.math.isNan(v.getY()));
    try fw.expect(std.math.isNan(v.getZ()));
    try fw.expect(v.isNaN());

    v.setComponent(0, 0);
    try fw.expect(v.isNaN());
    v.setComponent(1, 0);
    try fw.expect(v.isNaN());
    v.setComponent(2, 0);
    try fw.expect(!v.isNaN());
}

test "TestVec3Replicate" {
    try fw.expect(Vec3.replicate(2).eql(Vec3.init(2, 2, 2)));
}

test "TestVec3MinMax" {
    const v1 = Vec3.init(1, 5, 3);
    const v2 = Vec3.init(4, 2, 6);
    const v3 = Vec3.init(6, 4, 2);

    try fw.expect(Vec3.min(v1, v2).eql(Vec3.init(1, 2, 3)));
    try fw.expect(Vec3.max(v1, v2).eql(Vec3.init(4, 5, 6)));

    try fw.expectEqual(1, v1.reduceMin());
    try fw.expectEqual(5, v1.reduceMax());
    try fw.expectEqual(2, v2.reduceMin());
    try fw.expectEqual(6, v2.reduceMax());

    try fw.expectEqual(0, v1.getLowestComponentIndex());
    try fw.expectEqual(1, v1.getHighestComponentIndex());
    try fw.expectEqual(1, v2.getLowestComponentIndex());
    try fw.expectEqual(2, v2.getHighestComponentIndex());
    try fw.expectEqual(2, v3.getLowestComponentIndex());
    try fw.expectEqual(0, v3.getHighestComponentIndex());
}

test "TestVec3Clamp" {
    const v1 = Vec3.init(1, 2, 3);
    const v2 = Vec3.init(4, 5, 6);
    const v = Vec3.init(-1, 3, 7);

    try fw.expect(Vec3.clamp(v, v1, v2).eql(Vec3.init(1, 3, 6)));
}

test "TestVec3Comparisons" {
    try fw.expect(Vec3.equals(Vec3.init(1, 2, 3), Vec3.init(1, 4, 3)).eql(UVec4.init(0xffffffff, 0, 0xffffffff, 0xffffffff))); // W is always Z for comparisons
    try fw.expect(Vec3.less(Vec3.init(1, 2, 4), Vec3.init(1, 4, 3)).eql(UVec4.init(0, 0xffffffff, 0, 0)));
    try fw.expect(Vec3.lessOrEqual(Vec3.init(1, 2, 4), Vec3.init(1, 4, 3)).eql(UVec4.init(0xffffffff, 0xffffffff, 0, 0)));
    try fw.expect(Vec3.greater(Vec3.init(1, 2, 4), Vec3.init(1, 4, 3)).eql(UVec4.init(0, 0, 0xffffffff, 0xffffffff)));
    try fw.expect(Vec3.greaterOrEqual(Vec3.init(1, 2, 4), Vec3.init(1, 4, 3)).eql(UVec4.init(0xffffffff, 0, 0xffffffff, 0xffffffff)));
}

test "TestVec3FMA" {
    try fw.expect(Vec3.fusedMultiplyAdd(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9)).eql(Vec3.init(1 * 4 + 7, 2 * 5 + 8, 3 * 6 + 9)));
}

test "TestVec3Select" {
    try fw.expect(Vec3.select(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), UVec4.init(0x80000000, 0, 0x80000000, 0)).eql(Vec3.init(4, 2, 6)));
    try fw.expect(Vec3.select(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), UVec4.init(0, 0x80000000, 0, 0x80000000)).eql(Vec3.init(1, 5, 3)));
    try fw.expect(Vec3.select(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), UVec4.init(0xffffffff, 0x7fffffff, 0xffffffff, 0x7fffffff)).eql(Vec3.init(4, 2, 6)));
    try fw.expect(Vec3.select(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), UVec4.init(0x7fffffff, 0xffffffff, 0x7fffffff, 0xffffffff)).eql(Vec3.init(1, 5, 3)));
}

test "TestVec3BitOps" {
    // Test all bit permutations
    const v1 = Vec3.fromVec4(UVec4.init(0b0011, 0b00110, 0b001100, 0).reinterpretAsFloat());
    const v2 = Vec3.fromVec4(UVec4.init(0b0101, 0b01010, 0b010100, 0).reinterpretAsFloat());

    try fw.expect(Vec3.bitOr(v1, v2).eql(Vec3.fromVec4(UVec4.init(0b0111, 0b01110, 0b011100, 0).reinterpretAsFloat())));
    try fw.expect(Vec3.bitXor(v1, v2).eql(Vec3.fromVec4(UVec4.init(0b0110, 0b01100, 0b011000, 0).reinterpretAsFloat())));
    try fw.expect(Vec3.bitAnd(v1, v2).eql(Vec3.fromVec4(UVec4.init(0b0001, 0b00010, 0b000100, 0).reinterpretAsFloat())));
}

test "TestVec3Close" {
    try fw.expect(Vec3.init(1, 2, 3).isClose(Vec3.init(1.001, 2.001, 3.001), .{ .max_dist_sq = 1.0e-4 }));
    try fw.expect(!Vec3.init(1, 2, 3).isClose(Vec3.init(1.001, 2.001, 3.001), .{ .max_dist_sq = 1.0e-6 }));

    try fw.expect(Vec3.init(1.001, 0, 0).isNormalized(.{ .tolerance = 1.0e-2 }));
    try fw.expect(!Vec3.init(0, 1.001, 0).isNormalized(.{ .tolerance = 1.0e-4 }));

    try fw.expect(Vec3.init(-1.0e-7, 1.0e-7, 1.0e-8).isNearZero(.{}));
    try fw.expect(!Vec3.init(-1.0e-7, 1.0e-7, -1.0e-5).isNearZero(.{}));
}

test "TestVec3Operators" {
    try fw.expect(Vec3.init(1, 2, 3).negate().eql(Vec3.init(-1, -2, -3)));

    const neg_zero = Vec3.zero().negate();
    try fw.expect(neg_zero.eql(Vec3.zero()));

    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // When cross platform deterministic, we want to make sure that -0 is represented as 0
    const neg_zero_bin = neg_zero.reinterpretAsInt();
    try fw.expectEqual(0, neg_zero_bin.getX());
    try fw.expectEqual(0, neg_zero_bin.getY());
    try fw.expectEqual(0, neg_zero_bin.getZ());

    try fw.expect(Vec3.init(1, 2, 3).add(Vec3.init(4, 5, 6)).eql(Vec3.init(5, 7, 9)));
    try fw.expect(Vec3.init(1, 2, 3).sub(Vec3.init(6, 5, 4)).eql(Vec3.init(-5, -3, -1)));

    try fw.expect(Vec3.init(1, 2, 3).mul(Vec3.init(4, 5, 6)).eql(Vec3.init(4, 10, 18)));
    try fw.expect(Vec3.init(1, 2, 3).mulScalar(2).eql(Vec3.init(2, 4, 6)));
    try fw.expect(Vec3.init(1, 2, 3).mulScalar(4).eql(Vec3.init(4, 8, 12))); // 4 * Vec3(1, 2, 3)

    try fw.expect(Vec3.init(1, 2, 3).divScalar(2).eql(Vec3.init(0.5, 1.0, 1.5)));
    try fw.expect(Vec3.init(1, 2, 3).div(Vec3.init(2, 8, 24)).eql(Vec3.init(0.5, 0.25, 0.125)));

    var v = Vec3.init(1, 2, 3);
    v = v.mul(Vec3.init(4, 5, 6));
    try fw.expect(v.eql(Vec3.init(4, 10, 18)));
    v = v.mulScalar(2);
    try fw.expect(v.eql(Vec3.init(8, 20, 36)));
    v = v.divScalar(2);
    try fw.expect(v.eql(Vec3.init(4, 10, 18)));
    v = v.add(Vec3.init(1, 2, 3));
    try fw.expect(v.eql(Vec3.init(5, 12, 21)));
    v = v.sub(Vec3.init(1, 2, 3));
    try fw.expect(v.eql(Vec3.init(4, 10, 18)));

    try fw.expect(Vec3.init(2, 4, 8).reciprocal().eql(Vec3.init(0.5, 0.25, 0.125)));
}

test "TestVec3Swizzle" {
    const v = Vec3.init(1, 2, 3);

    try fw.expect(v.splatX().eql(Vec4.replicate(1)));
    try fw.expect(v.splatY().eql(Vec4.replicate(2)));
    try fw.expect(v.splatZ().eql(Vec4.replicate(3)));

    try fw.expect(v.swizzle(.x, .x, .x).eql(Vec3.init(1, 1, 1)));
    try fw.expect(v.swizzle(.x, .x, .y).eql(Vec3.init(1, 1, 2)));
    try fw.expect(v.swizzle(.x, .x, .z).eql(Vec3.init(1, 1, 3)));
    try fw.expect(v.swizzle(.x, .y, .x).eql(Vec3.init(1, 2, 1)));
    try fw.expect(v.swizzle(.x, .y, .y).eql(Vec3.init(1, 2, 2)));
    try fw.expect(v.swizzle(.x, .y, .z).eql(Vec3.init(1, 2, 3)));
    try fw.expect(v.swizzle(.x, .z, .x).eql(Vec3.init(1, 3, 1)));
    try fw.expect(v.swizzle(.x, .z, .y).eql(Vec3.init(1, 3, 2)));
    try fw.expect(v.swizzle(.x, .z, .z).eql(Vec3.init(1, 3, 3)));

    try fw.expect(v.swizzle(.y, .x, .x).eql(Vec3.init(2, 1, 1)));
    try fw.expect(v.swizzle(.y, .x, .y).eql(Vec3.init(2, 1, 2)));
    try fw.expect(v.swizzle(.y, .x, .z).eql(Vec3.init(2, 1, 3)));
    try fw.expect(v.swizzle(.y, .y, .x).eql(Vec3.init(2, 2, 1)));
    try fw.expect(v.swizzle(.y, .y, .y).eql(Vec3.init(2, 2, 2)));
    try fw.expect(v.swizzle(.y, .y, .z).eql(Vec3.init(2, 2, 3)));
    try fw.expect(v.swizzle(.y, .z, .x).eql(Vec3.init(2, 3, 1)));
    try fw.expect(v.swizzle(.y, .z, .y).eql(Vec3.init(2, 3, 2)));
    try fw.expect(v.swizzle(.y, .z, .z).eql(Vec3.init(2, 3, 3)));

    try fw.expect(v.swizzle(.z, .x, .x).eql(Vec3.init(3, 1, 1)));
    try fw.expect(v.swizzle(.z, .x, .y).eql(Vec3.init(3, 1, 2)));
    try fw.expect(v.swizzle(.z, .x, .z).eql(Vec3.init(3, 1, 3)));
    try fw.expect(v.swizzle(.z, .y, .x).eql(Vec3.init(3, 2, 1)));
    try fw.expect(v.swizzle(.z, .y, .y).eql(Vec3.init(3, 2, 2)));
    try fw.expect(v.swizzle(.z, .y, .z).eql(Vec3.init(3, 2, 3)));
    try fw.expect(v.swizzle(.z, .z, .x).eql(Vec3.init(3, 3, 1)));
    try fw.expect(v.swizzle(.z, .z, .y).eql(Vec3.init(3, 3, 2)));
    try fw.expect(v.swizzle(.z, .z, .z).eql(Vec3.init(3, 3, 3)));
}

test "TestVec3Abs" {
    try fw.expect(Vec3.init(1, -2, 3).abs().eql(Vec3.init(1, 2, 3)));
    try fw.expect(Vec3.init(-1, 2, -3).abs().eql(Vec3.init(1, 2, 3)));
}

test "TestVec3ReduceSum" {
    try fw.expectEqual(111, Vec3.init(1, 10, 100).reduceSum());
    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // Test handling of -0.0f
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, 0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, 0.0, 0.0).reduceSum())));
}

test "TestVec3Dot" {
    try fw.expectEqual(@as(f32, 1 * 4 + 2 * 5 + 3 * 6), Vec3.init(1, 2, 3).dot(Vec3.init(4, 5, 6)));
    try fw.expect(Vec3.init(1, 2, 3).dotV(Vec3.init(4, 5, 6)).eql(Vec3.replicate(1 * 4 + 2 * 5 + 3 * 6)));
    try fw.expect(Vec3.init(1, 2, 3).dotV4(Vec3.init(4, 5, 6)).eql(Vec4.replicate(1 * 4 + 2 * 5 + 3 * 6)));
    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // Test handling of -0.0f
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, -0.0, -0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, -0.0, 0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, 0.0, -0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(-0.0, 0.0, 0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, -0.0, -0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, -0.0, 0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, 0.0, -0.0).dot(Vec3.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec3.init(0.0, 0.0, 0.0).dot(Vec3.replicate(1.0)))));
}

test "TestVec3Length" {
    try fw.expectEqual(@as(f32, 1 + 4 + 9), Vec3.init(1, 2, 3).lengthSq());
    try fw.expectEqual(zolt.math.sqrt(@as(f32, 1 + 4 + 9)), Vec3.init(1, 2, 3).length());
}

test "TestVec3Sqrt" {
    try fw.checkApproxEqual(Vec3.init(13, 15, 17).sqrt(), Vec3.init(zolt.math.sqrt(@as(f32, 13.0)), zolt.math.sqrt(@as(f32, 15.0)), zolt.math.sqrt(@as(f32, 17.0))), .{});
}

test "TestVec3Cross" {
    try fw.expect(Vec3.init(1, 0, 0).cross(Vec3.init(0, 1, 0)).eql(Vec3.init(0, 0, 1)));
    try fw.expect(Vec3.init(0, 1, 0).cross(Vec3.init(1, 0, 0)).eql(Vec3.init(0, 0, -1)));
    try fw.expect(Vec3.init(0, 1, 0).cross(Vec3.init(0, 0, 1)).eql(Vec3.init(1, 0, 0)));
    try fw.expect(Vec3.init(0, 0, 1).cross(Vec3.init(0, 1, 0)).eql(Vec3.init(-1, 0, 0)));
    try fw.expect(Vec3.init(0, 0, 1).cross(Vec3.init(1, 0, 0)).eql(Vec3.init(0, 1, 0)));
    try fw.expect(Vec3.init(1, 0, 0).cross(Vec3.init(0, 0, 1)).eql(Vec3.init(0, -1, 0)));
}

test "TestVec3Normalize" {
    try fw.expect(Vec3.init(3, 2, 1).normalized().eql(Vec3.init(3, 2, 1).divScalar(zolt.math.sqrt(@as(f32, 9.0 + 4.0 + 1.0)))));
    try fw.expect(Vec3.init(3, 2, 1).normalizedOr(Vec3.init(1, 2, 3)).eql(Vec3.init(3, 2, 1).divScalar(zolt.math.sqrt(@as(f32, 9.0 + 4.0 + 1.0)))));
    try fw.expect(Vec3.zero().normalizedOr(Vec3.init(1, 2, 3)).eql(Vec3.init(1, 2, 3)));
    try fw.expect(Vec3.init(0.999 * zolt.math.sqrt(zolt.math.flt_min), 0, 0).normalizedOr(Vec3.init(1, 2, 3)).eql(Vec3.init(1, 2, 3))); // A vector that has a squared length that is denormal should also be treated as zero
    try fw.checkApproxEqual(Vec3.init(1.001 * zolt.math.sqrt(zolt.math.flt_min), 0, 0).normalizedOr(Vec3.init(1, 2, 3)), Vec3.init(1, 0, 0), .{}); // A value that is just above being denormal should work normally
}

test "TestVec3Cast" {
    try fw.expect(UVec4.equals(Vec3.init(1, 2, 3).toInt(), UVec4.init(1, 2, 3, 0)).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(1, 2, 3).reinterpretAsInt(), UVec4.init(0x3f800000, 0x40000000, 0x40400000, 0)).testAllXYZTrue());
}

test "TestVec3NormalizedPerpendicular" {
    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    // uniform_real_distribution<float> one_to_ten(1.0f, 10.0f);
    for (0..100) |_| {
        var v = Vec3.random(&random);
        try fw.expect(v.isNormalized(.{}));
        v = v.mulScalar(uniformFloat(&random, 1.0, 10.0));

        const p = v.getNormalizedPerpendicular();

        try fw.expect(p.isNormalized(.{}));
        try fw.expect(@abs(v.dot(p)) < 1.0e-6);
    }

    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // Check treatment of -0.0f
    try fw.expect(UVec4.equals(Vec3.init(0.0, 0.0, 2.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 1.0, 0.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(0.0, -0.0, 2.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 1.0, 0.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(-0.0, 0.0, 2.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 1.0, 0.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(-0.0, -0.0, 2.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 1.0, 0.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(0.0, 2.0, 0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(0.0, 2.0, -0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, -0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(-0.0, 2.0, 0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(-0.0, 2.0, -0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, -0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(2.0, 0.0, 0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(2.0, 0.0, -0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(-0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(2.0, -0.0, 0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
    try fw.expect(UVec4.equals(Vec3.init(2.0, -0.0, -0.0).getNormalizedPerpendicular().reinterpretAsInt(), Vec3.init(-0.0, 0.0, -1.0).reinterpretAsInt()).testAllXYZTrue());
}

test "TestVec3Sign" {
    const flt_true_min = std.math.floatTrueMin(f32);
    const inf = std.math.inf(f32);

    try fw.expect(Vec3.init(1.2345, -6.7891, 0).getSign().eql(Vec3.init(1, -1, 1)));
    try fw.expect(Vec3.init(0, 2.3456, -7.8912).getSign().eql(Vec3.init(1, 1, -1)));
    try fw.expect(Vec3.init(-0.0, 0.0, -0.0).getSign().eql(Vec3.init(-1, 1, -1)));
    try fw.expect(Vec3.init(1.0, -1.0, 1.0).getSign().eql(Vec3.init(1, -1, 1)));
    try fw.expect(Vec3.init(flt_true_min, -flt_true_min, flt_true_min).getSign().eql(Vec3.init(1, -1, 1))); // Denormal number
    try fw.expect(Vec3.init(inf, -inf, inf).getSign().eql(Vec3.init(1, -1, 1)));
}

test "TestVec3FlipSign" {
    const v = Vec3.init(1, 2, 3);
    try fw.expect(v.flipSign(-1, 1, 1).eql(Vec3.init(-1, 2, 3)));
    try fw.expect(v.flipSign(1, -1, 1).eql(Vec3.init(1, -2, 3)));
    try fw.expect(v.flipSign(1, 1, -1).eql(Vec3.init(1, 2, -3)));
}

// In Jolt this test only exists when JPH_FLOATING_POINT_EXCEPTIONS_ENABLED is defined. Zolt does not port that
// mode, but its Vec3 always keeps W equal to Z (see Zolt/Math/Vec3.zig), so the test applies unconditionally.
test "TestVec3SyncW" {
    {
        // Check that W equals Z
        const v = Vec3.init(1, 2, 3);
        try fw.expect(Vec4.fromVec3(v).eql(Vec4.init(1, 2, 3, 3)));
    }

    {
        // Check that setting individual components syncs W and Z
        var v: Vec3 = undefined;
        v.setComponent(2, 3);
        v.setComponent(1, 2);
        v.setComponent(0, 1);
        try fw.expect(v.eql(Vec3.init(1, 2, 3)));
        try fw.expect(Vec4.fromVec3(v).eql(Vec4.init(1, 2, 3, 3)));
    }

    {
        // Check that W and Z are still synced after a simple addition
        try fw.expect(Vec4.fromVec3(Vec3.init(1, 2, 3).add(Vec3.init(4, 5, 6))).eql(Vec4.init(5, 7, 9, 9)));
    }

    {
        // Test that casting a Vec4 to Vec3 syncs W and Z
        try fw.expect(Vec4.fromVec3(Vec3.fromVec4(Vec4.init(1, 2, 3, 4))).eql(Vec4.init(1, 2, 3, 3)));
    }

    {
        // Test that loading from Float3 syncs W and Z
        try fw.expect(Vec4.fromVec3(Vec3.fromFloat3(Float3.init(1, 2, 3))).eql(Vec4.init(1, 2, 3, 3)));
    }

    {
        // Test that loading unsafe from Float3 syncs W and Z
        const v = Float4.init(1, 2, 3, 4);
        try fw.expect(Vec4.fromVec3(Vec3.loadFloat3Unsafe(@ptrCast(&v))).eql(Vec4.init(1, 2, 3, 3)));
    }

    {
        // Test swizzle syncs W and Z
        try fw.expect(Vec4.fromVec3(Vec3.init(1, 2, 3).swizzle(.z, .y, .x)).eql(Vec4.init(3, 2, 1, 1)));
    }

    {
        // Test cross product syncs W and Z
        try fw.expect(Vec4.fromVec3(Vec3.init(1, 0, 0).cross(Vec3.init(0, 1, 0))).eql(Vec4.init(0, 0, 1, 1)));
        try fw.expect(Vec4.fromVec3(Vec3.init(0, 1, 0).cross(Vec3.init(0, 0, 1))).eql(Vec4.init(1, 0, 0, 0)));
    }
}

test "TestVec3ConvertToString" {
    const v = Vec3.init(1, 2, 3);
    var buf: [64]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}

test "TestVec3CompressUnitVector" {
    // We want these to be preserved exactly
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisX().compressUnitVector()).eql(Vec3.axisX()));
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisY().compressUnitVector()).eql(Vec3.axisY()));
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisZ().compressUnitVector()).eql(Vec3.axisZ()));
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisX().negate().compressUnitVector()).eql(Vec3.axisX().negate()));
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisY().negate().compressUnitVector()).eql(Vec3.axisY().negate()));
    try fw.expect(Vec3.decompressUnitVector(Vec3.axisZ().negate().compressUnitVector()).eql(Vec3.axisZ().negate()));

    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    for (0..1000) |_| {
        const v = Vec3.random(&random);
        const compressed = v.compressUnitVector();
        const decompressed = Vec3.decompressUnitVector(compressed);
        const diff = decompressed.sub(v).length();
        try fw.expect(diff < 1.0e-4);
    }
}

test "TestDifferenceOfProducts" {
    const a = Vec3.init(33962.035, 33962.0351, 33962.0352);
    const b = Vec3.init(-30438.8, -30438.801, -30438.802);
    const c = Vec3.init(41563.4, 41563.401, 41563.402);
    const d = Vec3.init(-24871.969, -24871.970, -24871.971);
    const result = Vec3.differenceOfProducts(a, b, c, d);
    const expected = DVec3.fromVec3(a).mul(DVec3.fromVec3(b)).sub(DVec3.fromVec3(c).mul(DVec3.fromVec3(d)));
    try fw.expect(expected.eql(DVec3.init(-75.165603637695312, 103.16904449462891, 36.836944580078125)));
    // JPH_USE_FMADD is never defined in Zolt (FMA is not used, see PortingGuide section 8), so only the #else branch applies:
    // CHECK(result == Vec3(expected));
    try fw.expect(result.eql(Vec3.init(-128.0, 64.0, 0.0))); // The products are in the order of 10^9, so the subtraction causes a large loss of precision and we get a very different result. This is expected when fused multiply add instructions are not available.
}
