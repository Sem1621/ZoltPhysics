//! Port of: UnitTests/Math/TrigonometryTests.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const math = zolt.math;
const trigonometry = zolt.trigonometry;

test "TestACosApproximate" {
    // Check error over entire range [-1, 1]
    var i: i32 = -1000;
    while (i <= 1000) : (i += 1) {
        const x = @as(f32, @floatFromInt(i)) / 1000.0;
        const acos1 = std.math.acos(x); // Reference value
        const acos2 = trigonometry.acosApproximate(x);
        try fw.checkApproxEqual(acos1, acos2, .{ .tolerance = 4.3e-3 });
    }

    // Check edge cases for exact matches
    try fw.expect(trigonometry.acosApproximate(1.0) == 0.0);
    try fw.expect(trigonometry.acosApproximate(1.0e-12) == math.pi / 2.0);
    try fw.expect(trigonometry.acosApproximate(-1.0e-12) == math.pi / 2.0);
    try fw.expect(trigonometry.acosApproximate(-1.0) == math.pi);
}
