//! Port of: UnitTests/Math/VectorTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const expect = fw.expect;
const expectEqual = fw.expectEqual;

const Vector = zolt.Vector;

test "TestVectorEquals" {
    var v1: Vector(3) = undefined;
    v1.setComponent(0, 1);
    v1.setComponent(1, 2);
    v1.setComponent(2, 3);

    var v2: Vector(3) = undefined;
    v2.setComponent(0, 1);
    v2.setComponent(1, 2);
    v2.setComponent(2, 3);

    var v3: Vector(3) = undefined;
    v3.setComponent(0, 1);
    v3.setComponent(1, 5);
    v3.setComponent(2, 3);

    try expect(v1.eql(v2));
    try expect(!!v1.eql(v2)); // !(v1 != v2)
    try expect(!v1.eql(v3)); // v1 != v3
    try expect(!v1.eql(v3));
}

test "TestVectorStream" {
    var v1: Vector(3) = undefined;
    v1.setComponent(0, 1);
    v1.setComponent(1, 2);
    v1.setComponent(2, 3);

    var buf: [64]u8 = undefined;
    const str = try std.fmt.bufPrint(&buf, "{f}", .{v1});
    try std.testing.expectEqualStrings("[1, 2, 3]", str);
}

test "TestVectorMultiplyFloat" {
    var v: Vector(5) = undefined;
    v.setComponent(0, 1);
    v.setComponent(1, 2);
    v.setComponent(2, 3);
    v.setComponent(3, 4);
    v.setComponent(4, 5);
    const v2 = v.mulScalar(2);
    try expectEqual(2.0, v2.getComponent(0));
    try expectEqual(4.0, v2.getComponent(1));
    try expectEqual(6.0, v2.getComponent(2));
    try expectEqual(8.0, v2.getComponent(3));
    try expectEqual(10.0, v2.getComponent(4));
}

test "TestVectorAdd" {
    var v1 = Vector(5).zero();
    var v2 = Vector(5).zero();
    v1.setComponent(0, 1);
    v2.setComponent(0, 2);
    v1.setComponent(4, 5);
    const v3 = v1.add(v2);
    try expectEqual(3.0, v3.getComponent(0));
    try expectEqual(0.0, v3.getComponent(1));
    try expectEqual(0.0, v3.getComponent(2));
    try expectEqual(0.0, v3.getComponent(3));
    try expectEqual(5.0, v3.getComponent(4));
}

test "TestVectorNegate" {
    var v: Vector(5) = undefined;
    v.setComponent(0, 1);
    v.setComponent(1, 2);
    v.setComponent(2, 3);
    v.setComponent(3, 4);
    v.setComponent(4, 5);
    const v2 = v.negate();
    try expectEqual(-1.0, v2.getComponent(0));
    try expectEqual(-2.0, v2.getComponent(1));
    try expectEqual(-3.0, v2.getComponent(2));
    try expectEqual(-4.0, v2.getComponent(3));
    try expectEqual(-5.0, v2.getComponent(4));
}

test "TestVectorLength" {
    var v: Vector(5) = undefined;
    v.setComponent(0, 1);
    v.setComponent(1, 2);
    v.setComponent(2, 3);
    v.setComponent(3, 4);
    v.setComponent(4, 5);
    try expectEqual(@as(f32, 1 + 4 + 9 + 16 + 25), v.lengthSq());
}
