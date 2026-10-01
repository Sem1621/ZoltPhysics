//! Port of: UnitTests/Math/MathTests.cpp
//! Status: partial
//! Missing: TestFindRoot (needs Jolt/Math/FindRoot.h, which has no Zolt port yet)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const checkApproxEqual = fw.checkApproxEqual;
const math = zolt.math;

test "TestCountTrailingZeros" {
    try expectEqual(@as(u32, 32), math.countTrailingZeros(0));
    for (0..32) |i|
        try expectEqual(@as(u32, @intCast(i)), math.countTrailingZeros(@as(u32, 1) << @intCast(i)));
}

test "TestCountLeadingZeros" {
    try expectEqual(@as(u32, 32), math.countLeadingZeros(0));
    for (0..32) |i|
        try expectEqual(@as(u32, @intCast(31 - i)), math.countLeadingZeros(@as(u32, 1) << @intCast(i)));
}

test "TestCountBits" {
    try expectEqual(@as(u32, 0), math.countBits(0));
    try expectEqual(@as(u32, 1), math.countBits(0b10000000000000000000000000000000));
    try expectEqual(@as(u32, 1), math.countBits(0b00000000000000000000000000000001));
    try expectEqual(@as(u32, 2), math.countBits(0b10000000000000001000000000000000));
    try expectEqual(@as(u32, 2), math.countBits(0b00000000000000010000000000000001));
    try expectEqual(@as(u32, 4), math.countBits(0b10000000100000001000000010000000));
    try expectEqual(@as(u32, 4), math.countBits(0b00000001000000010000000100000001));
    try expectEqual(@as(u32, 8), math.countBits(0b10001000100010001000100010001000));
    try expectEqual(@as(u32, 8), math.countBits(0b00010001000100010001000100010001));
    try expectEqual(@as(u32, 16), math.countBits(0b10101010101010101010101010101010));
    try expectEqual(@as(u32, 16), math.countBits(0b01010101010101010101010101010101));
    try expectEqual(@as(u32, 32), math.countBits(0b11111111111111111111111111111111));
}

test "TestNextPowerOf2" {
    try expectEqual(@as(u32, 1), math.getNextPowerOf2(0));

    for (0..31) |i| {
        const pow = @as(u32, 1) << @intCast(i);
        if (pow > 2)
            try expectEqual(pow, math.getNextPowerOf2(pow - 1));
        try expectEqual(pow, math.getNextPowerOf2(pow));
        try expectEqual(pow << 1, math.getNextPowerOf2(pow + 1));
    }

    try expectEqual(@as(u32, 0x8000000), math.getNextPowerOf2(0x8000000 - 1));
    try expectEqual(@as(u32, 0x8000000), math.getNextPowerOf2(0x8000000));
}

test "TestCenterAngleAroundZero" {
    var i: i32 = 0;
    while (i < 10) : (i += 2) {
        const fi: f32 = @floatFromInt(i);
        try checkApproxEqual(math.centerAngleAroundZero(fi * math.pi), 0, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero((0.5 + fi) * math.pi), 0.5 * math.pi, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero((1.5 + fi) * math.pi), -0.5 * math.pi, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero(-(0.5 + fi) * math.pi), -0.5 * math.pi, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero(-(1.5 + fi) * math.pi), 0.5 * math.pi, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero(-(0.99 + fi) * math.pi), -0.99 * math.pi, .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(math.centerAngleAroundZero((0.99 + fi) * math.pi), 0.99 * math.pi, .{ .tolerance = 1.0e-5 });
    }
}

test "TestIsPowerOf2" {
    for (0..63) |_|
        try expect(math.isPowerOf2(@as(u64, 1) << 1));
    try expect(!math.isPowerOf2(@as(i32, -2)));
    try expect(!math.isPowerOf2(@as(i32, 0)));
    try expect(!math.isPowerOf2(@as(i32, 3)));
    try expect(!math.isPowerOf2(@as(i32, 5)));
    try expect(!math.isPowerOf2(@as(i32, 15)));
    try expect(!math.isPowerOf2(@as(i32, 17)));
    try expect(!math.isPowerOf2(@as(i32, 65535)));
    try expect(!math.isPowerOf2(@as(i32, 65537)));
}

test "TestFindRoot" {
    // Not ported yet: depends on Jolt/Math/FindRoot.h, which has no Zolt port yet
    return error.SkipZigTest;
}

test "TestDifferenceOfProducts" {
    // Runtime variables to try to prevent the compiler from doing the calculation at compile time ('volatile' in Jolt)
    var a: f32 = 33962.035;
    var b: f32 = -30438.8;
    var c: f32 = 41563.4;
    var d: f32 = -24871.969;
    _ = .{ &a, &b, &c, &d };
    const result = math.differenceOfProducts(a, b, c, d);
    const expected = @as(f64, a) * @as(f64, b) - @as(f64, c) * @as(f64, d);
    try expectEqual(@as(f64, -75.165603637695312), expected);
    // Zolt never uses fused multiply add instructions (JPH_USE_FMADD is not compatible with JPH_CROSS_PLATFORM_DETERMINISTIC)
    try expectEqual(@as(f32, -128.0), result); // The products are in the order of 10^9, so the subtraction causes a large loss of precision and we get a very different result. This is expected when fused multiply add instructions are not available.
}

test "TestMulRSqrtApproximate" {
    var y: f32 = math.flt_min;
    while (y < 1.0e10) : (y *= 1.5) {
        const result = math.mulRSqrtApproximate(5.0, y);
        const expected = 5.0 / @sqrt(y);
        try checkApproxEqual(expected, result, .{ .tolerance = expected * 3.0e-7 }); // about 22 bits of precision
    }
}
