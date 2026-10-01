//! Port of: UnitTests/Math/Vec4Tests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const Float4 = zolt.Float4;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const pi = zolt.math.pi;

/// std::uniform_real_distribution<float>(min, max) on UnitTestRandom
fn uniformFloat(random: *fw.UnitTestRandom, min: f32, max: f32) f32 {
    return fw.UniformFloatDistribution.init(min, max).next(random);
}

test "TestVec4Construct" {
    const v = Vec4.init(1, 2, 3, 4);

    // Test component access
    try fw.expectEqual(1, v.getX());
    try fw.expectEqual(2, v.getY());
    try fw.expectEqual(3, v.getZ());
    try fw.expectEqual(4, v.getW());

    // Test component access by [] operators
    try fw.expectEqual(1, v.getComponent(0));
    try fw.expectEqual(2, v.getComponent(1));
    try fw.expectEqual(3, v.getComponent(2));
    try fw.expectEqual(4, v.getComponent(3));

    // Test == and != operators
    try fw.expect(v.eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(!v.eql(Vec4.init(1, 2, 4, 3)));
}

test "TestVec4LoadStoreFloat4" {
    const f4: Float4 align(16) = .init(1, 2, 3, 4);
    try fw.expect(Vec4.loadFloat4(&f4).eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(Vec4.loadFloat4Aligned(&f4).eql(Vec4.init(1, 2, 3, 4)));

    var f4_out: Float4 = undefined;
    Vec4.init(1, 2, 3, 4).storeFloat4(&f4_out);
    try fw.expectEqual(1, f4_out.getComponent(0));
    try fw.expectEqual(2, f4_out.getComponent(1));
    try fw.expectEqual(3, f4_out.getComponent(2));
    try fw.expectEqual(4, f4_out.getComponent(3));

    const sf = [_]f32{ 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 4, 0 };
    try fw.expect(Vec4.gatherFloat4(2 * @sizeOf(f32), &sf, UVec4.init(1, 3, 8, 9)).eql(Vec4.init(1, 2, 3, 4)));
}

test "TestVec4ConstructVec3" {
    const v3 = Vec3.init(1, 2, 3);
    try fw.expect(Vec4.fromVec3W(v3, 4).eql(Vec4.init(1, 2, 3, 4)));
}

test "TestVec4Zero" {
    const v = Vec4.zero();

    try fw.expectEqual(0, v.getX());
    try fw.expectEqual(0, v.getY());
    try fw.expectEqual(0, v.getZ());
    try fw.expectEqual(0, v.getW());
}

test "TestVec4NaN" {
    var v = Vec4.nan();

    try fw.expect(std.math.isNan(v.getX()));
    try fw.expect(std.math.isNan(v.getY()));
    try fw.expect(std.math.isNan(v.getZ()));
    try fw.expect(std.math.isNan(v.getW()));
    try fw.expect(v.isNaN());

    v.setX(0);
    try fw.expect(v.isNaN());
    v.setY(0);
    try fw.expect(v.isNaN());
    v.setZ(0);
    try fw.expect(v.isNaN());
    v.setW(0);
    try fw.expect(!v.isNaN());
}

test "TestVec4Replicate" {
    try fw.expect(Vec4.replicate(2).eql(Vec4.init(2, 2, 2, 2)));
}

test "TestVec4MinMax" {
    const v1 = Vec4.init(1, 6, 3, 8);
    const v2 = Vec4.init(5, 2, 7, 4);
    const v3 = Vec4.init(5, 7, 2, 4);
    const v4 = Vec4.init(7, 5, 4, 2);

    try fw.expect(Vec4.min(v1, v2).eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(Vec4.max(v1, v2).eql(Vec4.init(5, 6, 7, 8)));

    try fw.expectEqual(1, v1.reduceMin());
    try fw.expectEqual(8, v1.reduceMax());
    try fw.expectEqual(2, v2.reduceMin());
    try fw.expectEqual(7, v2.reduceMax());

    try fw.expectEqual(0, v1.getLowestComponentIndex());
    try fw.expectEqual(3, v1.getHighestComponentIndex());
    try fw.expectEqual(1, v2.getLowestComponentIndex());
    try fw.expectEqual(2, v2.getHighestComponentIndex());
    try fw.expectEqual(2, v3.getLowestComponentIndex());
    try fw.expectEqual(1, v3.getHighestComponentIndex());
    try fw.expectEqual(3, v4.getLowestComponentIndex());
    try fw.expectEqual(0, v4.getHighestComponentIndex());
}

test "TestVec4ReduceSum" {
    const v1 = Vec4.init(1, 6, 3, 8);
    const v2 = Vec4.init(-5, 2, -7, 4);

    try fw.expectEqual(1 + 6 + 3 + 8, v1.reduceSum());
    try fw.expectEqual(-5 + 2 - 7 + 4, v2.reduceSum());
    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // Test handling of -0.0f
    try fw.expectEqual(@as(u32, 0x80000000), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, 0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, 0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, 0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, -0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, -0.0, 0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, 0.0, -0.0).reduceSum())));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, 0.0, 0.0).reduceSum())));
}

test "TestVec4Clamp" {
    const v1 = Vec4.init(1, 2, 3, 4);
    const v2 = Vec4.init(5, 6, 7, 8);
    const v = Vec4.init(-1, 3, 9, -11);

    try fw.expect(Vec4.clamp(v, v1, v2).eql(Vec4.init(1, 3, 7, 4)));
}

test "TestVec4Comparisons" {
    try fw.expect(Vec4.equals(Vec4.init(1, 2, 3, 4), Vec4.init(2, 1, 3, 4)).eql(UVec4.init(0, 0, 0xffffffff, 0xffffffff)));
    try fw.expect(Vec4.less(Vec4.init(1, 2, 3, 4), Vec4.init(2, 1, 3, 4)).eql(UVec4.init(0xffffffff, 0, 0, 0)));
    try fw.expect(Vec4.lessOrEqual(Vec4.init(1, 2, 3, 4), Vec4.init(2, 1, 3, 4)).eql(UVec4.init(0xffffffff, 0, 0xffffffff, 0xffffffff)));
    try fw.expect(Vec4.greater(Vec4.init(1, 2, 3, 4), Vec4.init(2, 1, 3, 4)).eql(UVec4.init(0, 0xffffffff, 0, 0)));
    try fw.expect(Vec4.greaterOrEqual(Vec4.init(1, 2, 3, 4), Vec4.init(2, 1, 3, 4)).eql(UVec4.init(0, 0xffffffff, 0xffffffff, 0xffffffff)));
}

test "TestVec4FMA" {
    try fw.expect(Vec4.fusedMultiplyAdd(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12)).eql(Vec4.init(1 * 5 + 9, 2 * 6 + 10, 3 * 7 + 11, 4 * 8 + 12)));
}

test "TestVec4Select" {
    try fw.expect(Vec4.select(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), UVec4.init(0x80000000, 0, 0x80000000, 0)).eql(Vec4.init(5, 2, 7, 4)));
    try fw.expect(Vec4.select(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), UVec4.init(0, 0x80000000, 0, 0x80000000)).eql(Vec4.init(1, 6, 3, 8)));
    try fw.expect(Vec4.select(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), UVec4.init(0xffffffff, 0x7fffffff, 0xffffffff, 0x7fffffff)).eql(Vec4.init(5, 2, 7, 4)));
    try fw.expect(Vec4.select(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), UVec4.init(0x7fffffff, 0xffffffff, 0x7fffffff, 0xffffffff)).eql(Vec4.init(1, 6, 3, 8)));
}

test "TestVec4BitOps" {
    // Test all bit permutations
    const v1 = UVec4.init(0b0011, 0b00110, 0b001100, 0b0011000).reinterpretAsFloat();
    const v2 = UVec4.init(0b0101, 0b01010, 0b010100, 0b0101000).reinterpretAsFloat();

    try fw.expect(Vec4.bitOr(v1, v2).eql(UVec4.init(0b0111, 0b01110, 0b011100, 0b0111000).reinterpretAsFloat()));
    try fw.expect(Vec4.bitXor(v1, v2).eql(UVec4.init(0b0110, 0b01100, 0b011000, 0b0110000).reinterpretAsFloat()));
    try fw.expect(Vec4.bitAnd(v1, v2).eql(UVec4.init(0b0001, 0b00010, 0b000100, 0b0001000).reinterpretAsFloat()));
}

test "TestVec4Close" {
    try fw.expect(Vec4.init(1, 2, 3, 4).isClose(Vec4.init(1.001, 2.001, 3.001, 4.001), .{ .max_dist_sq = 1.0e-4 }));
    try fw.expect(!Vec4.init(1, 2, 3, 4).isClose(Vec4.init(1.001, 2.001, 3.001, 4.001), .{ .max_dist_sq = 1.0e-6 }));

    try fw.expect(Vec4.init(1.001, 0, 0, 0).isNormalized(.{ .tolerance = 1.0e-2 }));
    try fw.expect(!Vec4.init(0, 1.001, 0, 0).isNormalized(.{ .tolerance = 1.0e-4 }));

    try fw.expect(Vec4.init(-1.0e-7, 1.0e-7, 1.0e-8, -1.0e-8).isNearZero(.{}));
    try fw.expect(!Vec4.init(-1.0e-7, 1.0e-7, -1.0e-5, 1.0e-5).isNearZero(.{}));
}

test "TestVec4Operators" {
    try fw.expect(Vec4.init(1, 2, 3, 4).negate().eql(Vec4.init(-1, -2, -3, -4)));

    const neg_zero = Vec4.zero().negate();
    try fw.expect(neg_zero.eql(Vec4.zero()));

    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // When cross platform deterministic, we want to make sure that -0 is represented as 0
    const neg_zero_bin = neg_zero.reinterpretAsInt();
    try fw.expectEqual(0, neg_zero_bin.getX());
    try fw.expectEqual(0, neg_zero_bin.getY());
    try fw.expectEqual(0, neg_zero_bin.getZ());
    try fw.expectEqual(0, neg_zero_bin.getW());

    try fw.expect(Vec4.init(1, 2, 3, 4).add(Vec4.init(5, 6, 7, 8)).eql(Vec4.init(6, 8, 10, 12)));
    try fw.expect(Vec4.init(1, 2, 3, 4).sub(Vec4.init(8, 7, 6, 5)).eql(Vec4.init(-7, -5, -3, -1)));

    try fw.expect(Vec4.init(1, 2, 3, 4).mul(Vec4.init(5, 6, 7, 8)).eql(Vec4.init(5, 12, 21, 32)));
    try fw.expect(Vec4.init(1, 2, 3, 4).mulScalar(2).eql(Vec4.init(2, 4, 6, 8)));
    try fw.expect(Vec4.init(1, 2, 3, 4).mulScalar(4).eql(Vec4.init(4, 8, 12, 16))); // 4 * Vec4(1, 2, 3, 4)

    try fw.expect(Vec4.init(1, 2, 3, 4).divScalar(2).eql(Vec4.init(0.5, 1.0, 1.5, 2.0)));
    try fw.expect(Vec4.init(1, 2, 3, 4).div(Vec4.init(2, 8, 24, 64)).eql(Vec4.init(0.5, 0.25, 0.125, 0.0625)));

    var v = Vec4.init(1, 2, 3, 4);
    v = v.mul(Vec4.init(5, 6, 7, 8));
    try fw.expect(v.eql(Vec4.init(5, 12, 21, 32)));
    v = v.mulScalar(2);
    try fw.expect(v.eql(Vec4.init(10, 24, 42, 64)));
    v = v.divScalar(2);
    try fw.expect(v.eql(Vec4.init(5, 12, 21, 32)));
    v = v.add(Vec4.init(1, 2, 3, 4));
    try fw.expect(v.eql(Vec4.init(6, 14, 24, 36)));

    try fw.expect(Vec4.init(2, 4, 8, 16).reciprocal().eql(Vec4.init(0.5, 0.25, 0.125, 0.0625)));
}

test "TestVec4Swizzle" {
    const v = Vec4.init(1, 2, 3, 4);

    try fw.expect(v.splatX().eql(Vec4.replicate(1)));
    try fw.expect(v.splatY().eql(Vec4.replicate(2)));
    try fw.expect(v.splatZ().eql(Vec4.replicate(3)));
    try fw.expect(v.splatW().eql(Vec4.replicate(4)));

    try fw.expect(v.splatX3().eql(Vec3.replicate(1)));
    try fw.expect(v.splatY3().eql(Vec3.replicate(2)));
    try fw.expect(v.splatZ3().eql(Vec3.replicate(3)));
    try fw.expect(v.splatW3().eql(Vec3.replicate(4)));

    try fw.expect(v.swizzle(.x, .x, .x, .x).eql(Vec4.init(1, 1, 1, 1)));
    try fw.expect(v.swizzle(.x, .x, .x, .y).eql(Vec4.init(1, 1, 1, 2)));
    try fw.expect(v.swizzle(.x, .x, .x, .z).eql(Vec4.init(1, 1, 1, 3)));
    try fw.expect(v.swizzle(.x, .x, .x, .w).eql(Vec4.init(1, 1, 1, 4)));
    try fw.expect(v.swizzle(.x, .x, .y, .x).eql(Vec4.init(1, 1, 2, 1)));
    try fw.expect(v.swizzle(.x, .x, .y, .y).eql(Vec4.init(1, 1, 2, 2)));
    try fw.expect(v.swizzle(.x, .x, .y, .z).eql(Vec4.init(1, 1, 2, 3)));
    try fw.expect(v.swizzle(.x, .x, .y, .w).eql(Vec4.init(1, 1, 2, 4)));
    try fw.expect(v.swizzle(.x, .x, .z, .x).eql(Vec4.init(1, 1, 3, 1)));
    try fw.expect(v.swizzle(.x, .x, .z, .y).eql(Vec4.init(1, 1, 3, 2)));
    try fw.expect(v.swizzle(.x, .x, .z, .z).eql(Vec4.init(1, 1, 3, 3)));
    try fw.expect(v.swizzle(.x, .x, .z, .w).eql(Vec4.init(1, 1, 3, 4)));
    try fw.expect(v.swizzle(.x, .x, .w, .x).eql(Vec4.init(1, 1, 4, 1)));
    try fw.expect(v.swizzle(.x, .x, .w, .y).eql(Vec4.init(1, 1, 4, 2)));
    try fw.expect(v.swizzle(.x, .x, .w, .z).eql(Vec4.init(1, 1, 4, 3)));
    try fw.expect(v.swizzle(.x, .x, .w, .w).eql(Vec4.init(1, 1, 4, 4)));
    try fw.expect(v.swizzle(.x, .y, .x, .x).eql(Vec4.init(1, 2, 1, 1)));
    try fw.expect(v.swizzle(.x, .y, .x, .y).eql(Vec4.init(1, 2, 1, 2)));
    try fw.expect(v.swizzle(.x, .y, .x, .z).eql(Vec4.init(1, 2, 1, 3)));
    try fw.expect(v.swizzle(.x, .y, .x, .w).eql(Vec4.init(1, 2, 1, 4)));
    try fw.expect(v.swizzle(.x, .y, .y, .x).eql(Vec4.init(1, 2, 2, 1)));
    try fw.expect(v.swizzle(.x, .y, .y, .y).eql(Vec4.init(1, 2, 2, 2)));
    try fw.expect(v.swizzle(.x, .y, .y, .z).eql(Vec4.init(1, 2, 2, 3)));
    try fw.expect(v.swizzle(.x, .y, .y, .w).eql(Vec4.init(1, 2, 2, 4)));
    try fw.expect(v.swizzle(.x, .y, .z, .x).eql(Vec4.init(1, 2, 3, 1)));
    try fw.expect(v.swizzle(.x, .y, .z, .y).eql(Vec4.init(1, 2, 3, 2)));
    try fw.expect(v.swizzle(.x, .y, .z, .z).eql(Vec4.init(1, 2, 3, 3)));
    try fw.expect(v.swizzle(.x, .y, .z, .w).eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(v.swizzle(.x, .y, .w, .x).eql(Vec4.init(1, 2, 4, 1)));
    try fw.expect(v.swizzle(.x, .y, .w, .y).eql(Vec4.init(1, 2, 4, 2)));
    try fw.expect(v.swizzle(.x, .y, .w, .z).eql(Vec4.init(1, 2, 4, 3)));
    try fw.expect(v.swizzle(.x, .y, .w, .w).eql(Vec4.init(1, 2, 4, 4)));
    try fw.expect(v.swizzle(.x, .z, .x, .x).eql(Vec4.init(1, 3, 1, 1)));
    try fw.expect(v.swizzle(.x, .z, .x, .y).eql(Vec4.init(1, 3, 1, 2)));
    try fw.expect(v.swizzle(.x, .z, .x, .z).eql(Vec4.init(1, 3, 1, 3)));
    try fw.expect(v.swizzle(.x, .z, .x, .w).eql(Vec4.init(1, 3, 1, 4)));
    try fw.expect(v.swizzle(.x, .z, .y, .x).eql(Vec4.init(1, 3, 2, 1)));
    try fw.expect(v.swizzle(.x, .z, .y, .y).eql(Vec4.init(1, 3, 2, 2)));
    try fw.expect(v.swizzle(.x, .z, .y, .z).eql(Vec4.init(1, 3, 2, 3)));
    try fw.expect(v.swizzle(.x, .z, .y, .w).eql(Vec4.init(1, 3, 2, 4)));
    try fw.expect(v.swizzle(.x, .z, .z, .x).eql(Vec4.init(1, 3, 3, 1)));
    try fw.expect(v.swizzle(.x, .z, .z, .y).eql(Vec4.init(1, 3, 3, 2)));
    try fw.expect(v.swizzle(.x, .z, .z, .z).eql(Vec4.init(1, 3, 3, 3)));
    try fw.expect(v.swizzle(.x, .z, .z, .w).eql(Vec4.init(1, 3, 3, 4)));
    try fw.expect(v.swizzle(.x, .z, .w, .x).eql(Vec4.init(1, 3, 4, 1)));
    try fw.expect(v.swizzle(.x, .z, .w, .y).eql(Vec4.init(1, 3, 4, 2)));
    try fw.expect(v.swizzle(.x, .z, .w, .z).eql(Vec4.init(1, 3, 4, 3)));
    try fw.expect(v.swizzle(.x, .z, .w, .w).eql(Vec4.init(1, 3, 4, 4)));
    try fw.expect(v.swizzle(.x, .w, .x, .x).eql(Vec4.init(1, 4, 1, 1)));
    try fw.expect(v.swizzle(.x, .w, .x, .y).eql(Vec4.init(1, 4, 1, 2)));
    try fw.expect(v.swizzle(.x, .w, .x, .z).eql(Vec4.init(1, 4, 1, 3)));
    try fw.expect(v.swizzle(.x, .w, .x, .w).eql(Vec4.init(1, 4, 1, 4)));
    try fw.expect(v.swizzle(.x, .w, .y, .x).eql(Vec4.init(1, 4, 2, 1)));
    try fw.expect(v.swizzle(.x, .w, .y, .y).eql(Vec4.init(1, 4, 2, 2)));
    try fw.expect(v.swizzle(.x, .w, .y, .z).eql(Vec4.init(1, 4, 2, 3)));
    try fw.expect(v.swizzle(.x, .w, .y, .w).eql(Vec4.init(1, 4, 2, 4)));
    try fw.expect(v.swizzle(.x, .w, .z, .x).eql(Vec4.init(1, 4, 3, 1)));
    try fw.expect(v.swizzle(.x, .w, .z, .y).eql(Vec4.init(1, 4, 3, 2)));
    try fw.expect(v.swizzle(.x, .w, .z, .z).eql(Vec4.init(1, 4, 3, 3)));
    try fw.expect(v.swizzle(.x, .w, .z, .w).eql(Vec4.init(1, 4, 3, 4)));
    try fw.expect(v.swizzle(.x, .w, .w, .x).eql(Vec4.init(1, 4, 4, 1)));
    try fw.expect(v.swizzle(.x, .w, .w, .y).eql(Vec4.init(1, 4, 4, 2)));
    try fw.expect(v.swizzle(.x, .w, .w, .z).eql(Vec4.init(1, 4, 4, 3)));
    try fw.expect(v.swizzle(.x, .w, .w, .w).eql(Vec4.init(1, 4, 4, 4)));

    try fw.expect(v.swizzle(.y, .x, .x, .x).eql(Vec4.init(2, 1, 1, 1)));
    try fw.expect(v.swizzle(.y, .x, .x, .y).eql(Vec4.init(2, 1, 1, 2)));
    try fw.expect(v.swizzle(.y, .x, .x, .z).eql(Vec4.init(2, 1, 1, 3)));
    try fw.expect(v.swizzle(.y, .x, .x, .w).eql(Vec4.init(2, 1, 1, 4)));
    try fw.expect(v.swizzle(.y, .x, .y, .x).eql(Vec4.init(2, 1, 2, 1)));
    try fw.expect(v.swizzle(.y, .x, .y, .y).eql(Vec4.init(2, 1, 2, 2)));
    try fw.expect(v.swizzle(.y, .x, .y, .z).eql(Vec4.init(2, 1, 2, 3)));
    try fw.expect(v.swizzle(.y, .x, .y, .w).eql(Vec4.init(2, 1, 2, 4)));
    try fw.expect(v.swizzle(.y, .x, .z, .x).eql(Vec4.init(2, 1, 3, 1)));
    try fw.expect(v.swizzle(.y, .x, .z, .y).eql(Vec4.init(2, 1, 3, 2)));
    try fw.expect(v.swizzle(.y, .x, .z, .z).eql(Vec4.init(2, 1, 3, 3)));
    try fw.expect(v.swizzle(.y, .x, .z, .w).eql(Vec4.init(2, 1, 3, 4)));
    try fw.expect(v.swizzle(.y, .x, .w, .x).eql(Vec4.init(2, 1, 4, 1)));
    try fw.expect(v.swizzle(.y, .x, .w, .y).eql(Vec4.init(2, 1, 4, 2)));
    try fw.expect(v.swizzle(.y, .x, .w, .z).eql(Vec4.init(2, 1, 4, 3)));
    try fw.expect(v.swizzle(.y, .x, .w, .w).eql(Vec4.init(2, 1, 4, 4)));
    try fw.expect(v.swizzle(.y, .y, .x, .x).eql(Vec4.init(2, 2, 1, 1)));
    try fw.expect(v.swizzle(.y, .y, .x, .y).eql(Vec4.init(2, 2, 1, 2)));
    try fw.expect(v.swizzle(.y, .y, .x, .z).eql(Vec4.init(2, 2, 1, 3)));
    try fw.expect(v.swizzle(.y, .y, .x, .w).eql(Vec4.init(2, 2, 1, 4)));
    try fw.expect(v.swizzle(.y, .y, .y, .x).eql(Vec4.init(2, 2, 2, 1)));
    try fw.expect(v.swizzle(.y, .y, .y, .y).eql(Vec4.init(2, 2, 2, 2)));
    try fw.expect(v.swizzle(.y, .y, .y, .z).eql(Vec4.init(2, 2, 2, 3)));
    try fw.expect(v.swizzle(.y, .y, .y, .w).eql(Vec4.init(2, 2, 2, 4)));
    try fw.expect(v.swizzle(.y, .y, .z, .x).eql(Vec4.init(2, 2, 3, 1)));
    try fw.expect(v.swizzle(.y, .y, .z, .y).eql(Vec4.init(2, 2, 3, 2)));
    try fw.expect(v.swizzle(.y, .y, .z, .z).eql(Vec4.init(2, 2, 3, 3)));
    try fw.expect(v.swizzle(.y, .y, .z, .w).eql(Vec4.init(2, 2, 3, 4)));
    try fw.expect(v.swizzle(.y, .y, .w, .x).eql(Vec4.init(2, 2, 4, 1)));
    try fw.expect(v.swizzle(.y, .y, .w, .y).eql(Vec4.init(2, 2, 4, 2)));
    try fw.expect(v.swizzle(.y, .y, .w, .z).eql(Vec4.init(2, 2, 4, 3)));
    try fw.expect(v.swizzle(.y, .y, .w, .w).eql(Vec4.init(2, 2, 4, 4)));
    try fw.expect(v.swizzle(.y, .z, .x, .x).eql(Vec4.init(2, 3, 1, 1)));
    try fw.expect(v.swizzle(.y, .z, .x, .y).eql(Vec4.init(2, 3, 1, 2)));
    try fw.expect(v.swizzle(.y, .z, .x, .z).eql(Vec4.init(2, 3, 1, 3)));
    try fw.expect(v.swizzle(.y, .z, .x, .w).eql(Vec4.init(2, 3, 1, 4)));
    try fw.expect(v.swizzle(.y, .z, .y, .x).eql(Vec4.init(2, 3, 2, 1)));
    try fw.expect(v.swizzle(.y, .z, .y, .y).eql(Vec4.init(2, 3, 2, 2)));
    try fw.expect(v.swizzle(.y, .z, .y, .z).eql(Vec4.init(2, 3, 2, 3)));
    try fw.expect(v.swizzle(.y, .z, .y, .w).eql(Vec4.init(2, 3, 2, 4)));
    try fw.expect(v.swizzle(.y, .z, .z, .x).eql(Vec4.init(2, 3, 3, 1)));
    try fw.expect(v.swizzle(.y, .z, .z, .y).eql(Vec4.init(2, 3, 3, 2)));
    try fw.expect(v.swizzle(.y, .z, .z, .z).eql(Vec4.init(2, 3, 3, 3)));
    try fw.expect(v.swizzle(.y, .z, .z, .w).eql(Vec4.init(2, 3, 3, 4)));
    try fw.expect(v.swizzle(.y, .z, .w, .x).eql(Vec4.init(2, 3, 4, 1)));
    try fw.expect(v.swizzle(.y, .z, .w, .y).eql(Vec4.init(2, 3, 4, 2)));
    try fw.expect(v.swizzle(.y, .z, .w, .z).eql(Vec4.init(2, 3, 4, 3)));
    try fw.expect(v.swizzle(.y, .z, .w, .w).eql(Vec4.init(2, 3, 4, 4)));
    try fw.expect(v.swizzle(.y, .w, .x, .x).eql(Vec4.init(2, 4, 1, 1)));
    try fw.expect(v.swizzle(.y, .w, .x, .y).eql(Vec4.init(2, 4, 1, 2)));
    try fw.expect(v.swizzle(.y, .w, .x, .z).eql(Vec4.init(2, 4, 1, 3)));
    try fw.expect(v.swizzle(.y, .w, .x, .w).eql(Vec4.init(2, 4, 1, 4)));
    try fw.expect(v.swizzle(.y, .w, .y, .x).eql(Vec4.init(2, 4, 2, 1)));
    try fw.expect(v.swizzle(.y, .w, .y, .y).eql(Vec4.init(2, 4, 2, 2)));
    try fw.expect(v.swizzle(.y, .w, .y, .z).eql(Vec4.init(2, 4, 2, 3)));
    try fw.expect(v.swizzle(.y, .w, .y, .w).eql(Vec4.init(2, 4, 2, 4)));
    try fw.expect(v.swizzle(.y, .w, .z, .x).eql(Vec4.init(2, 4, 3, 1)));
    try fw.expect(v.swizzle(.y, .w, .z, .y).eql(Vec4.init(2, 4, 3, 2)));
    try fw.expect(v.swizzle(.y, .w, .z, .z).eql(Vec4.init(2, 4, 3, 3)));
    try fw.expect(v.swizzle(.y, .w, .z, .w).eql(Vec4.init(2, 4, 3, 4)));
    try fw.expect(v.swizzle(.y, .w, .w, .x).eql(Vec4.init(2, 4, 4, 1)));
    try fw.expect(v.swizzle(.y, .w, .w, .y).eql(Vec4.init(2, 4, 4, 2)));
    try fw.expect(v.swizzle(.y, .w, .w, .z).eql(Vec4.init(2, 4, 4, 3)));
    try fw.expect(v.swizzle(.y, .w, .w, .w).eql(Vec4.init(2, 4, 4, 4)));

    try fw.expect(v.swizzle(.z, .x, .x, .x).eql(Vec4.init(3, 1, 1, 1)));
    try fw.expect(v.swizzle(.z, .x, .x, .y).eql(Vec4.init(3, 1, 1, 2)));
    try fw.expect(v.swizzle(.z, .x, .x, .z).eql(Vec4.init(3, 1, 1, 3)));
    try fw.expect(v.swizzle(.z, .x, .x, .w).eql(Vec4.init(3, 1, 1, 4)));
    try fw.expect(v.swizzle(.z, .x, .y, .x).eql(Vec4.init(3, 1, 2, 1)));
    try fw.expect(v.swizzle(.z, .x, .y, .y).eql(Vec4.init(3, 1, 2, 2)));
    try fw.expect(v.swizzle(.z, .x, .y, .z).eql(Vec4.init(3, 1, 2, 3)));
    try fw.expect(v.swizzle(.z, .x, .y, .w).eql(Vec4.init(3, 1, 2, 4)));
    try fw.expect(v.swizzle(.z, .x, .z, .x).eql(Vec4.init(3, 1, 3, 1)));
    try fw.expect(v.swizzle(.z, .x, .z, .y).eql(Vec4.init(3, 1, 3, 2)));
    try fw.expect(v.swizzle(.z, .x, .z, .z).eql(Vec4.init(3, 1, 3, 3)));
    try fw.expect(v.swizzle(.z, .x, .z, .w).eql(Vec4.init(3, 1, 3, 4)));
    try fw.expect(v.swizzle(.z, .x, .w, .x).eql(Vec4.init(3, 1, 4, 1)));
    try fw.expect(v.swizzle(.z, .x, .w, .y).eql(Vec4.init(3, 1, 4, 2)));
    try fw.expect(v.swizzle(.z, .x, .w, .z).eql(Vec4.init(3, 1, 4, 3)));
    try fw.expect(v.swizzle(.z, .x, .w, .w).eql(Vec4.init(3, 1, 4, 4)));
    try fw.expect(v.swizzle(.z, .y, .x, .x).eql(Vec4.init(3, 2, 1, 1)));
    try fw.expect(v.swizzle(.z, .y, .x, .y).eql(Vec4.init(3, 2, 1, 2)));
    try fw.expect(v.swizzle(.z, .y, .x, .z).eql(Vec4.init(3, 2, 1, 3)));
    try fw.expect(v.swizzle(.z, .y, .x, .w).eql(Vec4.init(3, 2, 1, 4)));
    try fw.expect(v.swizzle(.z, .y, .y, .x).eql(Vec4.init(3, 2, 2, 1)));
    try fw.expect(v.swizzle(.z, .y, .y, .y).eql(Vec4.init(3, 2, 2, 2)));
    try fw.expect(v.swizzle(.z, .y, .y, .z).eql(Vec4.init(3, 2, 2, 3)));
    try fw.expect(v.swizzle(.z, .y, .y, .w).eql(Vec4.init(3, 2, 2, 4)));
    try fw.expect(v.swizzle(.z, .y, .z, .x).eql(Vec4.init(3, 2, 3, 1)));
    try fw.expect(v.swizzle(.z, .y, .z, .y).eql(Vec4.init(3, 2, 3, 2)));
    try fw.expect(v.swizzle(.z, .y, .z, .z).eql(Vec4.init(3, 2, 3, 3)));
    try fw.expect(v.swizzle(.z, .y, .z, .w).eql(Vec4.init(3, 2, 3, 4)));
    try fw.expect(v.swizzle(.z, .y, .w, .x).eql(Vec4.init(3, 2, 4, 1)));
    try fw.expect(v.swizzle(.z, .y, .w, .y).eql(Vec4.init(3, 2, 4, 2)));
    try fw.expect(v.swizzle(.z, .y, .w, .z).eql(Vec4.init(3, 2, 4, 3)));
    try fw.expect(v.swizzle(.z, .y, .w, .w).eql(Vec4.init(3, 2, 4, 4)));
    try fw.expect(v.swizzle(.z, .z, .x, .x).eql(Vec4.init(3, 3, 1, 1)));
    try fw.expect(v.swizzle(.z, .z, .x, .y).eql(Vec4.init(3, 3, 1, 2)));
    try fw.expect(v.swizzle(.z, .z, .x, .z).eql(Vec4.init(3, 3, 1, 3)));
    try fw.expect(v.swizzle(.z, .z, .x, .w).eql(Vec4.init(3, 3, 1, 4)));
    try fw.expect(v.swizzle(.z, .z, .y, .x).eql(Vec4.init(3, 3, 2, 1)));
    try fw.expect(v.swizzle(.z, .z, .y, .y).eql(Vec4.init(3, 3, 2, 2)));
    try fw.expect(v.swizzle(.z, .z, .y, .z).eql(Vec4.init(3, 3, 2, 3)));
    try fw.expect(v.swizzle(.z, .z, .y, .w).eql(Vec4.init(3, 3, 2, 4)));
    try fw.expect(v.swizzle(.z, .z, .z, .x).eql(Vec4.init(3, 3, 3, 1)));
    try fw.expect(v.swizzle(.z, .z, .z, .y).eql(Vec4.init(3, 3, 3, 2)));
    try fw.expect(v.swizzle(.z, .z, .z, .z).eql(Vec4.init(3, 3, 3, 3)));
    try fw.expect(v.swizzle(.z, .z, .z, .w).eql(Vec4.init(3, 3, 3, 4)));
    try fw.expect(v.swizzle(.z, .z, .w, .x).eql(Vec4.init(3, 3, 4, 1)));
    try fw.expect(v.swizzle(.z, .z, .w, .y).eql(Vec4.init(3, 3, 4, 2)));
    try fw.expect(v.swizzle(.z, .z, .w, .z).eql(Vec4.init(3, 3, 4, 3)));
    try fw.expect(v.swizzle(.z, .z, .w, .w).eql(Vec4.init(3, 3, 4, 4)));
    try fw.expect(v.swizzle(.z, .w, .x, .x).eql(Vec4.init(3, 4, 1, 1)));
    try fw.expect(v.swizzle(.z, .w, .x, .y).eql(Vec4.init(3, 4, 1, 2)));
    try fw.expect(v.swizzle(.z, .w, .x, .z).eql(Vec4.init(3, 4, 1, 3)));
    try fw.expect(v.swizzle(.z, .w, .x, .w).eql(Vec4.init(3, 4, 1, 4)));
    try fw.expect(v.swizzle(.z, .w, .y, .x).eql(Vec4.init(3, 4, 2, 1)));
    try fw.expect(v.swizzle(.z, .w, .y, .y).eql(Vec4.init(3, 4, 2, 2)));
    try fw.expect(v.swizzle(.z, .w, .y, .z).eql(Vec4.init(3, 4, 2, 3)));
    try fw.expect(v.swizzle(.z, .w, .y, .w).eql(Vec4.init(3, 4, 2, 4)));
    try fw.expect(v.swizzle(.z, .w, .z, .x).eql(Vec4.init(3, 4, 3, 1)));
    try fw.expect(v.swizzle(.z, .w, .z, .y).eql(Vec4.init(3, 4, 3, 2)));
    try fw.expect(v.swizzle(.z, .w, .z, .z).eql(Vec4.init(3, 4, 3, 3)));
    try fw.expect(v.swizzle(.z, .w, .z, .w).eql(Vec4.init(3, 4, 3, 4)));
    try fw.expect(v.swizzle(.z, .w, .w, .x).eql(Vec4.init(3, 4, 4, 1)));
    try fw.expect(v.swizzle(.z, .w, .w, .y).eql(Vec4.init(3, 4, 4, 2)));
    try fw.expect(v.swizzle(.z, .w, .w, .z).eql(Vec4.init(3, 4, 4, 3)));
    try fw.expect(v.swizzle(.z, .w, .w, .w).eql(Vec4.init(3, 4, 4, 4)));

    try fw.expect(v.swizzle(.w, .x, .x, .x).eql(Vec4.init(4, 1, 1, 1)));
    try fw.expect(v.swizzle(.w, .x, .x, .y).eql(Vec4.init(4, 1, 1, 2)));
    try fw.expect(v.swizzle(.w, .x, .x, .z).eql(Vec4.init(4, 1, 1, 3)));
    try fw.expect(v.swizzle(.w, .x, .x, .w).eql(Vec4.init(4, 1, 1, 4)));
    try fw.expect(v.swizzle(.w, .x, .y, .x).eql(Vec4.init(4, 1, 2, 1)));
    try fw.expect(v.swizzle(.w, .x, .y, .y).eql(Vec4.init(4, 1, 2, 2)));
    try fw.expect(v.swizzle(.w, .x, .y, .z).eql(Vec4.init(4, 1, 2, 3)));
    try fw.expect(v.swizzle(.w, .x, .y, .w).eql(Vec4.init(4, 1, 2, 4)));
    try fw.expect(v.swizzle(.w, .x, .z, .x).eql(Vec4.init(4, 1, 3, 1)));
    try fw.expect(v.swizzle(.w, .x, .z, .y).eql(Vec4.init(4, 1, 3, 2)));
    try fw.expect(v.swizzle(.w, .x, .z, .z).eql(Vec4.init(4, 1, 3, 3)));
    try fw.expect(v.swizzle(.w, .x, .z, .w).eql(Vec4.init(4, 1, 3, 4)));
    try fw.expect(v.swizzle(.w, .x, .w, .x).eql(Vec4.init(4, 1, 4, 1)));
    try fw.expect(v.swizzle(.w, .x, .w, .y).eql(Vec4.init(4, 1, 4, 2)));
    try fw.expect(v.swizzle(.w, .x, .w, .z).eql(Vec4.init(4, 1, 4, 3)));
    try fw.expect(v.swizzle(.w, .x, .w, .w).eql(Vec4.init(4, 1, 4, 4)));
    try fw.expect(v.swizzle(.w, .y, .x, .x).eql(Vec4.init(4, 2, 1, 1)));
    try fw.expect(v.swizzle(.w, .y, .x, .y).eql(Vec4.init(4, 2, 1, 2)));
    try fw.expect(v.swizzle(.w, .y, .x, .z).eql(Vec4.init(4, 2, 1, 3)));
    try fw.expect(v.swizzle(.w, .y, .x, .w).eql(Vec4.init(4, 2, 1, 4)));
    try fw.expect(v.swizzle(.w, .y, .y, .x).eql(Vec4.init(4, 2, 2, 1)));
    try fw.expect(v.swizzle(.w, .y, .y, .y).eql(Vec4.init(4, 2, 2, 2)));
    try fw.expect(v.swizzle(.w, .y, .y, .z).eql(Vec4.init(4, 2, 2, 3)));
    try fw.expect(v.swizzle(.w, .y, .y, .w).eql(Vec4.init(4, 2, 2, 4)));
    try fw.expect(v.swizzle(.w, .y, .z, .x).eql(Vec4.init(4, 2, 3, 1)));
    try fw.expect(v.swizzle(.w, .y, .z, .y).eql(Vec4.init(4, 2, 3, 2)));
    try fw.expect(v.swizzle(.w, .y, .z, .z).eql(Vec4.init(4, 2, 3, 3)));
    try fw.expect(v.swizzle(.w, .y, .z, .w).eql(Vec4.init(4, 2, 3, 4)));
    try fw.expect(v.swizzle(.w, .y, .w, .x).eql(Vec4.init(4, 2, 4, 1)));
    try fw.expect(v.swizzle(.w, .y, .w, .y).eql(Vec4.init(4, 2, 4, 2)));
    try fw.expect(v.swizzle(.w, .y, .w, .z).eql(Vec4.init(4, 2, 4, 3)));
    try fw.expect(v.swizzle(.w, .y, .w, .w).eql(Vec4.init(4, 2, 4, 4)));
    try fw.expect(v.swizzle(.w, .z, .x, .x).eql(Vec4.init(4, 3, 1, 1)));
    try fw.expect(v.swizzle(.w, .z, .x, .y).eql(Vec4.init(4, 3, 1, 2)));
    try fw.expect(v.swizzle(.w, .z, .x, .z).eql(Vec4.init(4, 3, 1, 3)));
    try fw.expect(v.swizzle(.w, .z, .x, .w).eql(Vec4.init(4, 3, 1, 4)));
    try fw.expect(v.swizzle(.w, .z, .y, .x).eql(Vec4.init(4, 3, 2, 1)));
    try fw.expect(v.swizzle(.w, .z, .y, .y).eql(Vec4.init(4, 3, 2, 2)));
    try fw.expect(v.swizzle(.w, .z, .y, .z).eql(Vec4.init(4, 3, 2, 3)));
    try fw.expect(v.swizzle(.w, .z, .y, .w).eql(Vec4.init(4, 3, 2, 4)));
    try fw.expect(v.swizzle(.w, .z, .z, .x).eql(Vec4.init(4, 3, 3, 1)));
    try fw.expect(v.swizzle(.w, .z, .z, .y).eql(Vec4.init(4, 3, 3, 2)));
    try fw.expect(v.swizzle(.w, .z, .z, .z).eql(Vec4.init(4, 3, 3, 3)));
    try fw.expect(v.swizzle(.w, .z, .z, .w).eql(Vec4.init(4, 3, 3, 4)));
    try fw.expect(v.swizzle(.w, .z, .w, .x).eql(Vec4.init(4, 3, 4, 1)));
    try fw.expect(v.swizzle(.w, .z, .w, .y).eql(Vec4.init(4, 3, 4, 2)));
    try fw.expect(v.swizzle(.w, .z, .w, .z).eql(Vec4.init(4, 3, 4, 3)));
    try fw.expect(v.swizzle(.w, .z, .w, .w).eql(Vec4.init(4, 3, 4, 4)));
    try fw.expect(v.swizzle(.w, .w, .x, .x).eql(Vec4.init(4, 4, 1, 1)));
    try fw.expect(v.swizzle(.w, .w, .x, .y).eql(Vec4.init(4, 4, 1, 2)));
    try fw.expect(v.swizzle(.w, .w, .x, .z).eql(Vec4.init(4, 4, 1, 3)));
    try fw.expect(v.swizzle(.w, .w, .x, .w).eql(Vec4.init(4, 4, 1, 4)));
    try fw.expect(v.swizzle(.w, .w, .y, .x).eql(Vec4.init(4, 4, 2, 1)));
    try fw.expect(v.swizzle(.w, .w, .y, .y).eql(Vec4.init(4, 4, 2, 2)));
    try fw.expect(v.swizzle(.w, .w, .y, .z).eql(Vec4.init(4, 4, 2, 3)));
    try fw.expect(v.swizzle(.w, .w, .y, .w).eql(Vec4.init(4, 4, 2, 4)));
    try fw.expect(v.swizzle(.w, .w, .z, .x).eql(Vec4.init(4, 4, 3, 1)));
    try fw.expect(v.swizzle(.w, .w, .z, .y).eql(Vec4.init(4, 4, 3, 2)));
    try fw.expect(v.swizzle(.w, .w, .z, .z).eql(Vec4.init(4, 4, 3, 3)));
    try fw.expect(v.swizzle(.w, .w, .z, .w).eql(Vec4.init(4, 4, 3, 4)));
    try fw.expect(v.swizzle(.w, .w, .w, .x).eql(Vec4.init(4, 4, 4, 1)));
    try fw.expect(v.swizzle(.w, .w, .w, .y).eql(Vec4.init(4, 4, 4, 2)));
    try fw.expect(v.swizzle(.w, .w, .w, .z).eql(Vec4.init(4, 4, 4, 3)));
    try fw.expect(v.swizzle(.w, .w, .w, .w).eql(Vec4.init(4, 4, 4, 4)));
}

test "TestVec4Abs" {
    try fw.expect(Vec4.init(1, -2, 3, -4).abs().eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(Vec4.init(-1, 2, -3, 4).abs().eql(Vec4.init(1, 2, 3, 4)));
}

test "TestVec4Dot" {
    try fw.expectEqual(@as(f32, 1 * 5 + 2 * 6 + 3 * 7 + 4 * 8), Vec4.init(1, 2, 3, 4).dot(Vec4.init(5, 6, 7, 8)));
    try fw.expect(Vec4.init(1, 2, 3, 4).dotV(Vec4.init(5, 6, 7, 8)).eql(Vec4.replicate(1 * 5 + 2 * 6 + 3 * 7 + 4 * 8)));
    // JPH_CROSS_PLATFORM_DETERMINISTIC is always enabled in Zolt
    // Test handling of -0.0f
    try fw.expectEqual(@as(u32, 0x80000000), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, -0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, -0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, 0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, -0.0, 0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, -0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, -0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, 0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(-0.0, 0.0, 0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, -0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, -0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, 0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, -0.0, 0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, -0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, -0.0, 0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, 0.0, -0.0).dot(Vec4.replicate(1.0)))));
    try fw.expectEqual(@as(u32, 0), @as(u32, @bitCast(Vec4.init(0.0, 0.0, 0.0, 0.0).dot(Vec4.replicate(1.0)))));
}

test "TestVec4Length" {
    try fw.expectEqual(@as(f32, 1 + 4 + 9 + 16), Vec4.init(1, 2, 3, 4).lengthSq());
    try fw.expectEqual(zolt.math.sqrt(@as(f32, 1 + 4 + 9 + 16)), Vec4.init(1, 2, 3, 4).length());
}

test "TestVec4Sqrt" {
    try fw.checkApproxEqual(Vec4.init(13, 15, 17, 19).sqrt(), Vec4.init(zolt.math.sqrt(@as(f32, 13.0)), zolt.math.sqrt(@as(f32, 15.0)), zolt.math.sqrt(@as(f32, 17.0)), zolt.math.sqrt(@as(f32, 19.0))), .{});
}

test "TestVec4Normalize" {
    try fw.expect(Vec4.init(1, 2, 3, 4).normalized().eql(Vec4.init(1, 2, 3, 4).divScalar(zolt.math.sqrt(@as(f32, 30.0)))));
}

test "TestVec4Cast" {
    try fw.expect(Vec4.init(1, 2, 3, 4).toInt().eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(Vec4.init(1, 2, 3, 4).reinterpretAsInt().eql(UVec4.init(0x3f800000, 0x40000000, 0x40400000, 0x40800000)));
}

test "TestVec4Sign" {
    const flt_true_min = std.math.floatTrueMin(f32);
    const inf = std.math.inf(f32);

    try fw.expect(Vec4.init(1.2345, -6.7891, 0, 1).getSign().eql(Vec4.init(1, -1, 1, 1)));
    try fw.expect(Vec4.init(0, 2.3456, -7.8912, -1).getSign().eql(Vec4.init(1, 1, -1, -1)));
    try fw.expect(Vec4.init(-0.0, 0.0, -0.0, 0.0).getSign().eql(Vec4.init(-1, 1, -1, 1)));
    try fw.expect(Vec4.init(1.0, -1.0, 1.0, -1.0).getSign().eql(Vec4.init(1, -1, 1, -1)));
    try fw.expect(Vec4.init(flt_true_min, -flt_true_min, flt_true_min, -flt_true_min).getSign().eql(Vec4.init(1, -1, 1, -1))); // Denormal number
    try fw.expect(Vec4.init(inf, -inf, inf, -inf).getSign().eql(Vec4.init(1, -1, 1, -1)));
}

test "TestVec4FlipSign" {
    const v = Vec4.init(1, 2, 3, 4);
    try fw.expect(v.flipSign(-1, 1, 1, 1).eql(Vec4.init(-1, 2, 3, 4)));
    try fw.expect(v.flipSign(1, -1, 1, 1).eql(Vec4.init(1, -2, 3, 4)));
    try fw.expect(v.flipSign(1, 1, -1, 1).eql(Vec4.init(1, 2, -3, 4)));
    try fw.expect(v.flipSign(1, 1, 1, -1).eql(Vec4.init(1, 2, 3, -4)));
}

test "TestVec4SignBit" {
    try fw.expectEqual(0b1010, Vec4.init(2, -3, 4, -5).getSignBits());
    try fw.expectEqual(0b0101, Vec4.init(-2, 3, -4, 5).getSignBits());
}

test "TestVec4Sort" {
    var i: u32 = 0;
    while (i < 4) : (i += 1) {
        var j: u32 = 0;
        while (j < 4) : (j += 1) {
            if (i != j) {
                var k: u32 = 0;
                while (k < 4) : (k += 1) {
                    if (i != k and j != k) {
                        var l: u32 = 0;
                        while (l < 4) : (l += 1) {
                            if (i != l and j != l and k != l) {
                                var v1 = Vec4.init(@floatFromInt(i), @floatFromInt(j), @floatFromInt(k), @floatFromInt(l));
                                var v2 = v1;
                                var idx1 = UVec4.init(i + 4, j + 4, k + 4, l + 4);
                                var idx2 = idx1;
                                Vec4.sort4(&v1, &idx1);
                                Vec4.sort4Reverse(&v2, &idx2);
                                var m: u32 = 0;
                                while (m < 4) : (m += 1) {
                                    try fw.expectEqual(@as(f32, @floatFromInt(m)), v1.getComponent(m));
                                    try fw.expectEqual(@as(f32, @floatFromInt(3 - m)), v2.getComponent(m));
                                    try fw.expectEqual(m + 4, idx1.getComponent(m));
                                    try fw.expectEqual(3 - m + 4, idx2.getComponent(m));
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

test "TestVec4SinCos" {
    // Check edge cases
    const edge = Vec4.init(0, 0.5 * pi, pi, -0.5 * pi).sinCos();
    try fw.expect(edge.sin.isClose(Vec4.init(0, 1, 0, -1), .{ .max_dist_sq = 1.0e-7 }));
    try fw.expect(edge.cos.isClose(Vec4.init(1, 0, -1, 0), .{ .max_dist_sq = 1.0e-7 }));

    var ms: f64 = 0.0;
    var mc: f64 = 0.0;

    var x: f32 = -100.0 * pi;
    while (x < 100.0 * pi) : (x += 1.0e-3) {
        // Create a vector with intermediate values
        const xv = Vec4.replicate(x).add(Vec4.init(0.0e-4, 2.5e-4, 5.0e-4, 7.5e-4));

        // Calculate sin and cos
        const sc = xv.sinCos();

        var i: u32 = 0;
        while (i < 4) : (i += 1) {
            // Check accuracy of sin
            const s1 = std.math.sin(@as(f64, xv.getComponent(i)));
            const s2: f64 = sc.sin.getComponent(i);
            const ds = @abs(s2 - s1);
            ms = zolt.math.max(ms, ds);

            // Check accuracy of cos
            const c1 = std.math.cos(@as(f64, xv.getComponent(i)));
            const c2: f64 = sc.cos.getComponent(i);
            const dc = @abs(c2 - c1);
            mc = zolt.math.max(mc, dc);
        }
    }

    try fw.expect(ms < 1.0e-7);
    try fw.expect(mc < 1.0e-7);
}

test "TestVec4Tan" {
    // Check edge cases
    try fw.expect(Vec4.replicate(0.0).tan().eql(Vec4.zero()));
    try fw.expect(Vec4.replicate(0.5 * pi - 1.0e-6).tan().getX() > 1.0e6);
    try fw.expect(Vec4.replicate(0.5 * pi + 1.0e-6).tan().getX() < -1.0e6);

    var mt: f64 = 0.0;

    var x: f32 = -100.0 * pi;
    while (x < 100.0 * pi) : (x += 1.0e-3) {
        // Create a vector with intermediate values
        const xv = Vec4.replicate(x).add(Vec4.init(0.0e-4, 2.5e-4, 5.0e-4, 7.5e-4));

        // Calculate tan
        const vt = xv.tan();

        var i: u32 = 0;
        while (i < 4) : (i += 1) {
            // Check accuracy of tan
            const t1 = std.math.tan(@as(f64, xv.getComponent(i)));
            const t2: f64 = vt.getComponent(i);
            const dt = @abs(t2 - t1);
            mt = zolt.math.max(mt, dt) / zolt.math.max(@as(f64, 1.0), @abs(t1)); // Take relative error
        }
    }

    try fw.expect(mt < 1.5e-7);
}

test "TestVec4ASin" {
    // Check edge cases
    try fw.expect(Vec4.replicate(0.0).asin().eql(Vec4.zero()));
    try fw.expect(Vec4.one().asin().eql(Vec4.replicate(0.5 * pi)));
    try fw.expect(Vec4.replicate(-1.0).asin().eql(Vec4.replicate(-0.5 * pi)));

    var ma: f64 = 0.0;

    var x: f32 = -1.0;
    while (x <= 1.0) : (x += 1.0e-3) {
        // Create a vector with intermediate values
        const xv = Vec4.min(Vec4.replicate(x).add(Vec4.init(0.0e-4, 2.5e-4, 5.0e-4, 7.5e-4)), Vec4.one());

        // Calculate asin
        const va = xv.asin();

        var i: u32 = 0;
        while (i < 4) : (i += 1) {
            // Check accuracy of asin
            const a1 = std.math.asin(@as(f64, xv.getComponent(i)));
            const a2: f64 = va.getComponent(i);
            const da = @abs(a2 - a1);
            ma = zolt.math.max(ma, da);
        }
    }

    try fw.expect(ma < 2.0e-7);

    // Check that inputs are clamped as promised
    try fw.expect(Vec4.replicate(-1.1).asin().eql(Vec4.replicate(-0.5 * pi)));
    try fw.expect(Vec4.replicate(1.1).asin().eql(Vec4.replicate(0.5 * pi)));
}

test "TestVec4ACos" {
    // Check edge cases
    try fw.expect(Vec4.replicate(0.0).acos().eql(Vec4.replicate(0.5 * pi)));
    try fw.expect(Vec4.one().acos().eql(Vec4.zero()));
    try fw.expect(Vec4.replicate(-1.0).acos().eql(Vec4.replicate(pi)));

    var ma: f64 = 0.0;

    var x: f32 = -1.0;
    while (x <= 1.0) : (x += 1.0e-3) {
        // Create a vector with intermediate values
        const xv = Vec4.min(Vec4.replicate(x).add(Vec4.init(0.0e-4, 2.5e-4, 5.0e-4, 7.5e-4)), Vec4.one());

        // Calculate acos
        const va = xv.acos();

        var i: u32 = 0;
        while (i < 4) : (i += 1) {
            // Check accuracy of acos
            const a1 = std.math.acos(@as(f64, xv.getComponent(i)));
            const a2: f64 = va.getComponent(i);
            const da = @abs(a2 - a1);
            ma = zolt.math.max(ma, da);
        }
    }

    try fw.expect(ma < 3.5e-7);

    // Check that inputs are clamped as promised
    try fw.expect(Vec4.replicate(-1.1).acos().eql(Vec4.replicate(pi)));
    try fw.expect(Vec4.replicate(1.1).acos().eql(Vec4.zero()));
}

test "TestVec4ATan" {
    // Check edge cases
    try fw.expect(Vec4.replicate(0.0).atan().eql(Vec4.zero()));
    try fw.expect(Vec4.replicate(zolt.math.flt_max).atan().eql(Vec4.replicate(0.5 * pi)));
    try fw.expect(Vec4.replicate(-zolt.math.flt_max).atan().eql(Vec4.replicate(-0.5 * pi)));

    var ma: f64 = 0.0;

    var x: f32 = -100.0;
    while (x < 100.0) : (x += 1.0e-3) {
        // Create a vector with intermediate values
        const xv = Vec4.replicate(x).add(Vec4.init(0.0e-4, 2.5e-4, 5.0e-4, 7.5e-4));

        // Calculate atan
        const va = xv.atan();

        var i: u32 = 0;
        while (i < 4) : (i += 1) {
            // Check accuracy of atan
            const a1 = std.math.atan(@as(f64, xv.getComponent(i)));
            const a2: f64 = va.getComponent(i);
            const da = @abs(a2 - a1);
            ma = zolt.math.max(ma, da);
        }
    }

    try fw.expect(ma < 1.5e-7);
}

test "TestVec4ATan2" {
    var ma: f64 = 0.0;

    // Test the axis
    try fw.expect(Vec4.atan2(Vec4.zero(), Vec4.replicate(10.0)).eql(Vec4.zero()));
    try fw.expect(Vec4.atan2(Vec4.zero(), Vec4.replicate(-10.0)).eql(Vec4.replicate(pi)));
    try fw.expect(Vec4.atan2(Vec4.replicate(10.0), Vec4.zero()).eql(Vec4.replicate(0.5 * pi)));
    try fw.expect(Vec4.atan2(Vec4.replicate(-10.0), Vec4.zero()).eql(Vec4.replicate(-0.5 * pi)));

    // Test the 4 quadrants
    try fw.expect(Vec4.atan2(Vec4.replicate(10.0), Vec4.replicate(10.0)).eql(Vec4.replicate(0.25 * pi)));
    try fw.expect(Vec4.atan2(Vec4.replicate(10.0), Vec4.replicate(-10.0)).eql(Vec4.replicate(0.75 * pi)));
    try fw.expect(Vec4.atan2(Vec4.replicate(-10.0), Vec4.replicate(-10.0)).eql(Vec4.replicate(-0.75 * pi)));
    try fw.expect(Vec4.atan2(Vec4.replicate(-10.0), Vec4.replicate(10.0)).eql(Vec4.replicate(-0.25 * pi)));

    var y: f32 = -5.0;
    while (y < 5.0) : (y += 1.0e-2) {
        // Create a vector with intermediate values
        const yv = Vec4.replicate(y).add(Vec4.init(0.0e-3, 2.5e-3, 5.0e-3, 7.5e-3));

        var x: f32 = -5.0;
        while (x < 5.0) : (x += 1.0e-2) {
            // Create a vector with intermediate values
            const xv = Vec4.replicate(x).add(Vec4.init(0.0e-3, 2.5e-3, 5.0e-3, 7.5e-3));

            // Calculate atan
            const va = Vec4.atan2(yv, xv);

            var i: u32 = 0;
            while (i < 4) : (i += 1) {
                // Check accuracy of atan
                const a1 = std.math.atan2(@as(f64, yv.getComponent(i)), @as(f64, xv.getComponent(i)));
                const a2: f64 = va.getComponent(i);
                const da = @abs(a2 - a1);
                ma = zolt.math.max(ma, da);
            }
        }
    }

    try fw.expect(ma < 3.0e-7);
}

test "TestVec4ConvertToString" {
    const v = Vec4.init(1, 2, 3, 4);
    var buf: [64]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}

test "TestVec4CompressUnitVector" {
    // We want these to be preserved exactly
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(1, 0, 0, 0).compressUnitVector()).eql(Vec4.init(1, 0, 0, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, 1, 0, 0).compressUnitVector()).eql(Vec4.init(0, 1, 0, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, 0, 1, 0).compressUnitVector()).eql(Vec4.init(0, 0, 1, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, 0, 0, 1).compressUnitVector()).eql(Vec4.init(0, 0, 0, 1)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(-1, 0, 0, 0).compressUnitVector()).eql(Vec4.init(-1, 0, 0, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, -1, 0, 0).compressUnitVector()).eql(Vec4.init(0, -1, 0, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, 0, -1, 0).compressUnitVector()).eql(Vec4.init(0, 0, -1, 0)));
    try fw.expect(Vec4.decompressUnitVector(Vec4.init(0, 0, 0, -1).compressUnitVector()).eql(Vec4.init(0, 0, 0, -1)));

    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    for (0..1000) |_| {
        // std::uniform_real_distribution<float> scale(-1.0f, 1.0f);
        const v = Vec4.init(uniformFloat(&random, -1.0, 1.0), uniformFloat(&random, -1.0, 1.0), uniformFloat(&random, -1.0, 1.0), uniformFloat(&random, -1.0, 1.0)).normalized();
        const compressed = v.compressUnitVector();
        const decompressed = Vec4.decompressUnitVector(compressed);
        const diff = decompressed.sub(v).length();
        try fw.expect(diff < 5.0e-3);
    }
}

test "TestDifferenceOfProducts" {
    const a = Vec4.init(33962.035, 33962.0351, 33962.0352, 33962.0353);
    const b = Vec4.init(-30438.8, -30438.801, -30438.802, -30438.803);
    const c = Vec4.init(41563.4, 41563.401, 41563.402, 41563.403);
    const d = Vec4.init(-24871.969, -24871.970, -24871.971, -24871.972);
    const result = Vec4.differenceOfProducts(a, b, c, d);
    const expected = [4]f64{
        @as(f64, a.getX()) * @as(f64, b.getX()) - @as(f64, c.getX()) * @as(f64, d.getX()),
        @as(f64, a.getY()) * @as(f64, b.getY()) - @as(f64, c.getY()) * @as(f64, d.getY()),
        @as(f64, a.getZ()) * @as(f64, b.getZ()) - @as(f64, c.getZ()) * @as(f64, d.getZ()),
        @as(f64, a.getW()) * @as(f64, b.getW()) - @as(f64, c.getW()) * @as(f64, d.getW()),
    };
    try fw.expectEqual(-75.165603637695312, expected[0]);
    try fw.expectEqual(103.16904449462891, expected[1]);
    try fw.expectEqual(36.836944580078125, expected[2]);
    try fw.expectEqual(118.01546478271484, expected[3]);
    // JPH_USE_FMADD is never defined in Zolt (FMA is not used, see PortingGuide section 8), so only the #else branch applies:
    // CHECK(result == Vec4(float(expected[0]), float(expected[1]), float(expected[2]), float(expected[3])));
    try fw.expect(result.eql(Vec4.init(-128.0, 64.0, 0.0, 64.0))); // The products are in the order of 10^9, so the subtraction causes a large loss of precision and we get a very different result. This is expected when fused multiply add instructions are not available.
}
