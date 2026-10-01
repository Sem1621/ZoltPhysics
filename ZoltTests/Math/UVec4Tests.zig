//! Port of: UnitTests/Math/UVec4Tests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const UVec4 = zolt.UVec4;
const Vec4 = zolt.Vec4;

test "TestUVec4Construct" {
    const v = UVec4.init(1, 2, 3, 4);

    try fw.expectEqual(1, v.getX());
    try fw.expectEqual(2, v.getY());
    try fw.expectEqual(3, v.getZ());
    try fw.expectEqual(4, v.getW());

    try fw.expectEqual(1, v.getComponent(0));
    try fw.expectEqual(2, v.getComponent(1));
    try fw.expectEqual(3, v.getComponent(2));
    try fw.expectEqual(4, v.getComponent(3));

    // Test == and != operators
    try fw.expect(v.eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(!v.eql(UVec4.init(1, 2, 4, 3)));
}

test "TestUVec4Components" {
    var v = UVec4.init(1, 2, 3, 4);
    v.setX(5);
    v.setY(6);
    v.setZ(7);
    v.setW(8);
    try fw.expect(v.eql(UVec4.init(5, 6, 7, 8)));
}

test "TestUVec4LoadStoreInt4" {
    const int4: [4]u32 align(16) = .{ 1, 2, 3, 4 }; // i4 in the C++ test, which is a primitive type in Zig
    try fw.expect(UVec4.loadInt(&int4[0]).eql(UVec4.init(1, 0, 0, 0)));
    try fw.expect(UVec4.loadInt4(&int4).eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(UVec4.loadInt4Aligned(&int4).eql(UVec4.init(1, 2, 3, 4)));

    var i4_out1: [4]u32 = undefined;
    UVec4.init(1, 2, 3, 4).storeInt4(&i4_out1);
    try fw.expectEqual(1, i4_out1[0]);
    try fw.expectEqual(2, i4_out1[1]);
    try fw.expectEqual(3, i4_out1[2]);
    try fw.expectEqual(4, i4_out1[3]);

    var i4_out2: [4]u32 align(16) = undefined;
    UVec4.init(1, 2, 3, 4).storeInt4Aligned(&i4_out2);
    try fw.expectEqual(1, i4_out2[0]);
    try fw.expectEqual(2, i4_out2[1]);
    try fw.expectEqual(3, i4_out2[2]);
    try fw.expectEqual(4, i4_out2[3]);

    const si = [_]u32{ 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 4, 0 };
    try fw.expect(UVec4.gatherInt4(2 * @sizeOf(u32), &si, UVec4.init(1, 3, 8, 9)).eql(UVec4.init(1, 2, 3, 4)));
}

test "TestUVec4Zero" {
    const v = UVec4.zero();

    try fw.expectEqual(0, v.getX());
    try fw.expectEqual(0, v.getY());
    try fw.expectEqual(0, v.getZ());
    try fw.expectEqual(0, v.getW());
}

test "TestUVec4Replicate" {
    try fw.expect(UVec4.replicate(2).eql(UVec4.init(2, 2, 2, 2)));
}

test "TestUVec4MinMax" {
    const v1 = UVec4.init(1, 6, 3, 8);
    const v2 = UVec4.init(5, 2, 7, 4);

    try fw.expect(UVec4.min(v1, v2).eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(UVec4.max(v1, v2).eql(UVec4.init(5, 6, 7, 8)));
}

test "TestUVec4Comparisons" {
    try fw.expect(UVec4.equals(UVec4.init(1, 2, 3, 4), UVec4.init(2, 1, 3, 4)).eql(UVec4.init(0, 0, 0xffffffff, 0xffffffff)));

    try fw.expectEqual(0b0000, UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).getTrues());
    try fw.expectEqual(0b0001, UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).getTrues());
    try fw.expectEqual(0b0010, UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).getTrues());
    try fw.expectEqual(0b0011, UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).getTrues());
    try fw.expectEqual(0b0100, UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).getTrues());
    try fw.expectEqual(0b0101, UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).getTrues());
    try fw.expectEqual(0b0110, UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).getTrues());
    try fw.expectEqual(0b0111, UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).getTrues());
    try fw.expectEqual(0b1000, UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).getTrues());
    try fw.expectEqual(0b1001, UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).getTrues());
    try fw.expectEqual(0b1010, UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).getTrues());
    try fw.expectEqual(0b1011, UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).getTrues());
    try fw.expectEqual(0b1100, UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).getTrues());
    try fw.expectEqual(0b1101, UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).getTrues());
    try fw.expectEqual(0b1110, UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).getTrues());
    try fw.expectEqual(0b1111, UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).getTrues());

    try fw.expectEqual(0, UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).countTrues());
    try fw.expectEqual(1, UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).countTrues());
    try fw.expectEqual(1, UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).countTrues());
    try fw.expectEqual(2, UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).countTrues());
    try fw.expectEqual(1, UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).countTrues());
    try fw.expectEqual(2, UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).countTrues());
    try fw.expectEqual(2, UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).countTrues());
    try fw.expectEqual(3, UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).countTrues());
    try fw.expectEqual(1, UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).countTrues());
    try fw.expectEqual(2, UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).countTrues());
    try fw.expectEqual(2, UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).countTrues());
    try fw.expectEqual(3, UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).countTrues());
    try fw.expectEqual(2, UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).countTrues());
    try fw.expectEqual(3, UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).countTrues());
    try fw.expectEqual(3, UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).countTrues());
    try fw.expectEqual(4, UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).countTrues());

    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).testAllTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).testAllTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).testAllTrue());

    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).testAllXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).testAllXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).testAllXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).testAllXYZTrue());

    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).testAnyTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).testAnyTrue());

    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000).testAnyXYZTrue());
    try fw.expect(!UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff).testAnyXYZTrue());
    try fw.expect(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff).testAnyXYZTrue());
}

test "TestUVec4Select" {
    try fw.expect(UVec4.select(UVec4.init(1, 2, 3, 4), UVec4.init(5, 6, 7, 8), UVec4.init(0x80000000, 0, 0x80000000, 0)).eql(UVec4.init(5, 2, 7, 4)));
    try fw.expect(UVec4.select(UVec4.init(1, 2, 3, 4), UVec4.init(5, 6, 7, 8), UVec4.init(0, 0x80000000, 0, 0x80000000)).eql(UVec4.init(1, 6, 3, 8)));
    try fw.expect(UVec4.select(UVec4.init(1, 2, 3, 4), UVec4.init(5, 6, 7, 8), UVec4.init(0xffffffff, 0x7fffffff, 0xffffffff, 0x7fffffff)).eql(UVec4.init(5, 2, 7, 4)));
    try fw.expect(UVec4.select(UVec4.init(1, 2, 3, 4), UVec4.init(5, 6, 7, 8), UVec4.init(0x7fffffff, 0xffffffff, 0x7fffffff, 0xffffffff)).eql(UVec4.init(1, 6, 3, 8)));
}

test "TestUVec4BitOps" {
    // Test all bit permutations
    const v1 = UVec4.init(0b0011, 0b00110, 0b001100, 0b0011000);
    const v2 = UVec4.init(0b0101, 0b01010, 0b010100, 0b0101000);

    try fw.expect(UVec4.bitOr(v1, v2).eql(UVec4.init(0b0111, 0b01110, 0b011100, 0b0111000)));
    try fw.expect(UVec4.bitXor(v1, v2).eql(UVec4.init(0b0110, 0b01100, 0b011000, 0b0110000)));
    try fw.expect(UVec4.bitAnd(v1, v2).eql(UVec4.init(0b0001, 0b00010, 0b000100, 0b0001000)));

    try fw.expect(UVec4.bitNot(v1).eql(UVec4.init(0xfffffffc, 0xfffffff9, 0xfffffff3, 0xffffffe7)));
    try fw.expect(UVec4.bitNot(v2).eql(UVec4.init(0xfffffffa, 0xfffffff5, 0xffffffeb, 0xffffffd7)));

    try fw.expect(UVec4.init(0x80000000, 0x40000000, 0x20000000, 0x10000000).logicalShiftRight(1).eql(UVec4.init(0x40000000, 0x20000000, 0x10000000, 0x08000000)));
    try fw.expect(UVec4.init(0x80000000, 0x40000000, 0x20000000, 0x10000000).arithmeticShiftRight(1).eql(UVec4.init(0xC0000000, 0x20000000, 0x10000000, 0x08000000)));
    try fw.expect(UVec4.init(0x40000000, 0x20000000, 0x10000000, 0x08000001).logicalShiftLeft(1).eql(UVec4.init(0x80000000, 0x40000000, 0x20000000, 0x10000002)));
}

test "TestUVec4Operators" {
    try fw.expect(UVec4.init(1, 2, 3, 4).add(UVec4.init(5, 6, 7, 8)).eql(UVec4.init(6, 8, 10, 12)));

    try fw.expect(UVec4.init(5, 6, 7, 8).sub(UVec4.init(4, 3, 2, 1)).eql(UVec4.init(1, 3, 5, 7)));

    try fw.expect(UVec4.init(1, 2, 3, 4).mul(UVec4.init(5, 6, 7, 8)).eql(UVec4.init(1 * 5, 2 * 6, 3 * 7, 4 * 8)));

    var v = UVec4.init(1, 2, 3, 4);
    v = v.add(UVec4.init(5, 6, 7, 8));
    try fw.expect(v.eql(UVec4.init(6, 8, 10, 12)));
    v = v.sub(UVec4.init(4, 3, 2, 1));
    try fw.expect(v.eql(UVec4.init(2, 5, 8, 11)));
}

test "TestUVec4Swizzle" {
    const v = UVec4.init(1, 2, 3, 4);

    try fw.expect(v.splatX().eql(UVec4.replicate(1)));
    try fw.expect(v.splatY().eql(UVec4.replicate(2)));
    try fw.expect(v.splatZ().eql(UVec4.replicate(3)));
    try fw.expect(v.splatW().eql(UVec4.replicate(4)));

    try fw.expect(v.swizzle(.x, .x, .x, .x).eql(UVec4.init(1, 1, 1, 1)));
    try fw.expect(v.swizzle(.x, .x, .x, .y).eql(UVec4.init(1, 1, 1, 2)));
    try fw.expect(v.swizzle(.x, .x, .x, .z).eql(UVec4.init(1, 1, 1, 3)));
    try fw.expect(v.swizzle(.x, .x, .x, .w).eql(UVec4.init(1, 1, 1, 4)));
    try fw.expect(v.swizzle(.x, .x, .y, .x).eql(UVec4.init(1, 1, 2, 1)));
    try fw.expect(v.swizzle(.x, .x, .y, .y).eql(UVec4.init(1, 1, 2, 2)));
    try fw.expect(v.swizzle(.x, .x, .y, .z).eql(UVec4.init(1, 1, 2, 3)));
    try fw.expect(v.swizzle(.x, .x, .y, .w).eql(UVec4.init(1, 1, 2, 4)));
    try fw.expect(v.swizzle(.x, .x, .z, .x).eql(UVec4.init(1, 1, 3, 1)));
    try fw.expect(v.swizzle(.x, .x, .z, .y).eql(UVec4.init(1, 1, 3, 2)));
    try fw.expect(v.swizzle(.x, .x, .z, .z).eql(UVec4.init(1, 1, 3, 3)));
    try fw.expect(v.swizzle(.x, .x, .z, .w).eql(UVec4.init(1, 1, 3, 4)));
    try fw.expect(v.swizzle(.x, .x, .w, .x).eql(UVec4.init(1, 1, 4, 1)));
    try fw.expect(v.swizzle(.x, .x, .w, .y).eql(UVec4.init(1, 1, 4, 2)));
    try fw.expect(v.swizzle(.x, .x, .w, .z).eql(UVec4.init(1, 1, 4, 3)));
    try fw.expect(v.swizzle(.x, .x, .w, .w).eql(UVec4.init(1, 1, 4, 4)));
    try fw.expect(v.swizzle(.x, .y, .x, .x).eql(UVec4.init(1, 2, 1, 1)));
    try fw.expect(v.swizzle(.x, .y, .x, .y).eql(UVec4.init(1, 2, 1, 2)));
    try fw.expect(v.swizzle(.x, .y, .x, .z).eql(UVec4.init(1, 2, 1, 3)));
    try fw.expect(v.swizzle(.x, .y, .x, .w).eql(UVec4.init(1, 2, 1, 4)));
    try fw.expect(v.swizzle(.x, .y, .y, .x).eql(UVec4.init(1, 2, 2, 1)));
    try fw.expect(v.swizzle(.x, .y, .y, .y).eql(UVec4.init(1, 2, 2, 2)));
    try fw.expect(v.swizzle(.x, .y, .y, .z).eql(UVec4.init(1, 2, 2, 3)));
    try fw.expect(v.swizzle(.x, .y, .y, .w).eql(UVec4.init(1, 2, 2, 4)));
    try fw.expect(v.swizzle(.x, .y, .z, .x).eql(UVec4.init(1, 2, 3, 1)));
    try fw.expect(v.swizzle(.x, .y, .z, .y).eql(UVec4.init(1, 2, 3, 2)));
    try fw.expect(v.swizzle(.x, .y, .z, .z).eql(UVec4.init(1, 2, 3, 3)));
    try fw.expect(v.swizzle(.x, .y, .z, .w).eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(v.swizzle(.x, .y, .w, .x).eql(UVec4.init(1, 2, 4, 1)));
    try fw.expect(v.swizzle(.x, .y, .w, .y).eql(UVec4.init(1, 2, 4, 2)));
    try fw.expect(v.swizzle(.x, .y, .w, .z).eql(UVec4.init(1, 2, 4, 3)));
    try fw.expect(v.swizzle(.x, .y, .w, .w).eql(UVec4.init(1, 2, 4, 4)));
    try fw.expect(v.swizzle(.x, .z, .x, .x).eql(UVec4.init(1, 3, 1, 1)));
    try fw.expect(v.swizzle(.x, .z, .x, .y).eql(UVec4.init(1, 3, 1, 2)));
    try fw.expect(v.swizzle(.x, .z, .x, .z).eql(UVec4.init(1, 3, 1, 3)));
    try fw.expect(v.swizzle(.x, .z, .x, .w).eql(UVec4.init(1, 3, 1, 4)));
    try fw.expect(v.swizzle(.x, .z, .y, .x).eql(UVec4.init(1, 3, 2, 1)));
    try fw.expect(v.swizzle(.x, .z, .y, .y).eql(UVec4.init(1, 3, 2, 2)));
    try fw.expect(v.swizzle(.x, .z, .y, .z).eql(UVec4.init(1, 3, 2, 3)));
    try fw.expect(v.swizzle(.x, .z, .y, .w).eql(UVec4.init(1, 3, 2, 4)));
    try fw.expect(v.swizzle(.x, .z, .z, .x).eql(UVec4.init(1, 3, 3, 1)));
    try fw.expect(v.swizzle(.x, .z, .z, .y).eql(UVec4.init(1, 3, 3, 2)));
    try fw.expect(v.swizzle(.x, .z, .z, .z).eql(UVec4.init(1, 3, 3, 3)));
    try fw.expect(v.swizzle(.x, .z, .z, .w).eql(UVec4.init(1, 3, 3, 4)));
    try fw.expect(v.swizzle(.x, .z, .w, .x).eql(UVec4.init(1, 3, 4, 1)));
    try fw.expect(v.swizzle(.x, .z, .w, .y).eql(UVec4.init(1, 3, 4, 2)));
    try fw.expect(v.swizzle(.x, .z, .w, .z).eql(UVec4.init(1, 3, 4, 3)));
    try fw.expect(v.swizzle(.x, .z, .w, .w).eql(UVec4.init(1, 3, 4, 4)));
    try fw.expect(v.swizzle(.x, .w, .x, .x).eql(UVec4.init(1, 4, 1, 1)));
    try fw.expect(v.swizzle(.x, .w, .x, .y).eql(UVec4.init(1, 4, 1, 2)));
    try fw.expect(v.swizzle(.x, .w, .x, .z).eql(UVec4.init(1, 4, 1, 3)));
    try fw.expect(v.swizzle(.x, .w, .x, .w).eql(UVec4.init(1, 4, 1, 4)));
    try fw.expect(v.swizzle(.x, .w, .y, .x).eql(UVec4.init(1, 4, 2, 1)));
    try fw.expect(v.swizzle(.x, .w, .y, .y).eql(UVec4.init(1, 4, 2, 2)));
    try fw.expect(v.swizzle(.x, .w, .y, .z).eql(UVec4.init(1, 4, 2, 3)));
    try fw.expect(v.swizzle(.x, .w, .y, .w).eql(UVec4.init(1, 4, 2, 4)));
    try fw.expect(v.swizzle(.x, .w, .z, .x).eql(UVec4.init(1, 4, 3, 1)));
    try fw.expect(v.swizzle(.x, .w, .z, .y).eql(UVec4.init(1, 4, 3, 2)));
    try fw.expect(v.swizzle(.x, .w, .z, .z).eql(UVec4.init(1, 4, 3, 3)));
    try fw.expect(v.swizzle(.x, .w, .z, .w).eql(UVec4.init(1, 4, 3, 4)));
    try fw.expect(v.swizzle(.x, .w, .w, .x).eql(UVec4.init(1, 4, 4, 1)));
    try fw.expect(v.swizzle(.x, .w, .w, .y).eql(UVec4.init(1, 4, 4, 2)));
    try fw.expect(v.swizzle(.x, .w, .w, .z).eql(UVec4.init(1, 4, 4, 3)));
    try fw.expect(v.swizzle(.x, .w, .w, .w).eql(UVec4.init(1, 4, 4, 4)));

    try fw.expect(v.swizzle(.y, .x, .x, .x).eql(UVec4.init(2, 1, 1, 1)));
    try fw.expect(v.swizzle(.y, .x, .x, .y).eql(UVec4.init(2, 1, 1, 2)));
    try fw.expect(v.swizzle(.y, .x, .x, .z).eql(UVec4.init(2, 1, 1, 3)));
    try fw.expect(v.swizzle(.y, .x, .x, .w).eql(UVec4.init(2, 1, 1, 4)));
    try fw.expect(v.swizzle(.y, .x, .y, .x).eql(UVec4.init(2, 1, 2, 1)));
    try fw.expect(v.swizzle(.y, .x, .y, .y).eql(UVec4.init(2, 1, 2, 2)));
    try fw.expect(v.swizzle(.y, .x, .y, .z).eql(UVec4.init(2, 1, 2, 3)));
    try fw.expect(v.swizzle(.y, .x, .y, .w).eql(UVec4.init(2, 1, 2, 4)));
    try fw.expect(v.swizzle(.y, .x, .z, .x).eql(UVec4.init(2, 1, 3, 1)));
    try fw.expect(v.swizzle(.y, .x, .z, .y).eql(UVec4.init(2, 1, 3, 2)));
    try fw.expect(v.swizzle(.y, .x, .z, .z).eql(UVec4.init(2, 1, 3, 3)));
    try fw.expect(v.swizzle(.y, .x, .z, .w).eql(UVec4.init(2, 1, 3, 4)));
    try fw.expect(v.swizzle(.y, .x, .w, .x).eql(UVec4.init(2, 1, 4, 1)));
    try fw.expect(v.swizzle(.y, .x, .w, .y).eql(UVec4.init(2, 1, 4, 2)));
    try fw.expect(v.swizzle(.y, .x, .w, .z).eql(UVec4.init(2, 1, 4, 3)));
    try fw.expect(v.swizzle(.y, .x, .w, .w).eql(UVec4.init(2, 1, 4, 4)));
    try fw.expect(v.swizzle(.y, .y, .x, .x).eql(UVec4.init(2, 2, 1, 1)));
    try fw.expect(v.swizzle(.y, .y, .x, .y).eql(UVec4.init(2, 2, 1, 2)));
    try fw.expect(v.swizzle(.y, .y, .x, .z).eql(UVec4.init(2, 2, 1, 3)));
    try fw.expect(v.swizzle(.y, .y, .x, .w).eql(UVec4.init(2, 2, 1, 4)));
    try fw.expect(v.swizzle(.y, .y, .y, .x).eql(UVec4.init(2, 2, 2, 1)));
    try fw.expect(v.swizzle(.y, .y, .y, .y).eql(UVec4.init(2, 2, 2, 2)));
    try fw.expect(v.swizzle(.y, .y, .y, .z).eql(UVec4.init(2, 2, 2, 3)));
    try fw.expect(v.swizzle(.y, .y, .y, .w).eql(UVec4.init(2, 2, 2, 4)));
    try fw.expect(v.swizzle(.y, .y, .z, .x).eql(UVec4.init(2, 2, 3, 1)));
    try fw.expect(v.swizzle(.y, .y, .z, .y).eql(UVec4.init(2, 2, 3, 2)));
    try fw.expect(v.swizzle(.y, .y, .z, .z).eql(UVec4.init(2, 2, 3, 3)));
    try fw.expect(v.swizzle(.y, .y, .z, .w).eql(UVec4.init(2, 2, 3, 4)));
    try fw.expect(v.swizzle(.y, .y, .w, .x).eql(UVec4.init(2, 2, 4, 1)));
    try fw.expect(v.swizzle(.y, .y, .w, .y).eql(UVec4.init(2, 2, 4, 2)));
    try fw.expect(v.swizzle(.y, .y, .w, .z).eql(UVec4.init(2, 2, 4, 3)));
    try fw.expect(v.swizzle(.y, .y, .w, .w).eql(UVec4.init(2, 2, 4, 4)));
    try fw.expect(v.swizzle(.y, .z, .x, .x).eql(UVec4.init(2, 3, 1, 1)));
    try fw.expect(v.swizzle(.y, .z, .x, .y).eql(UVec4.init(2, 3, 1, 2)));
    try fw.expect(v.swizzle(.y, .z, .x, .z).eql(UVec4.init(2, 3, 1, 3)));
    try fw.expect(v.swizzle(.y, .z, .x, .w).eql(UVec4.init(2, 3, 1, 4)));
    try fw.expect(v.swizzle(.y, .z, .y, .x).eql(UVec4.init(2, 3, 2, 1)));
    try fw.expect(v.swizzle(.y, .z, .y, .y).eql(UVec4.init(2, 3, 2, 2)));
    try fw.expect(v.swizzle(.y, .z, .y, .z).eql(UVec4.init(2, 3, 2, 3)));
    try fw.expect(v.swizzle(.y, .z, .y, .w).eql(UVec4.init(2, 3, 2, 4)));
    try fw.expect(v.swizzle(.y, .z, .z, .x).eql(UVec4.init(2, 3, 3, 1)));
    try fw.expect(v.swizzle(.y, .z, .z, .y).eql(UVec4.init(2, 3, 3, 2)));
    try fw.expect(v.swizzle(.y, .z, .z, .z).eql(UVec4.init(2, 3, 3, 3)));
    try fw.expect(v.swizzle(.y, .z, .z, .w).eql(UVec4.init(2, 3, 3, 4)));
    try fw.expect(v.swizzle(.y, .z, .w, .x).eql(UVec4.init(2, 3, 4, 1)));
    try fw.expect(v.swizzle(.y, .z, .w, .y).eql(UVec4.init(2, 3, 4, 2)));
    try fw.expect(v.swizzle(.y, .z, .w, .z).eql(UVec4.init(2, 3, 4, 3)));
    try fw.expect(v.swizzle(.y, .z, .w, .w).eql(UVec4.init(2, 3, 4, 4)));
    try fw.expect(v.swizzle(.y, .w, .x, .x).eql(UVec4.init(2, 4, 1, 1)));
    try fw.expect(v.swizzle(.y, .w, .x, .y).eql(UVec4.init(2, 4, 1, 2)));
    try fw.expect(v.swizzle(.y, .w, .x, .z).eql(UVec4.init(2, 4, 1, 3)));
    try fw.expect(v.swizzle(.y, .w, .x, .w).eql(UVec4.init(2, 4, 1, 4)));
    try fw.expect(v.swizzle(.y, .w, .y, .x).eql(UVec4.init(2, 4, 2, 1)));
    try fw.expect(v.swizzle(.y, .w, .y, .y).eql(UVec4.init(2, 4, 2, 2)));
    try fw.expect(v.swizzle(.y, .w, .y, .z).eql(UVec4.init(2, 4, 2, 3)));
    try fw.expect(v.swizzle(.y, .w, .y, .w).eql(UVec4.init(2, 4, 2, 4)));
    try fw.expect(v.swizzle(.y, .w, .z, .x).eql(UVec4.init(2, 4, 3, 1)));
    try fw.expect(v.swizzle(.y, .w, .z, .y).eql(UVec4.init(2, 4, 3, 2)));
    try fw.expect(v.swizzle(.y, .w, .z, .z).eql(UVec4.init(2, 4, 3, 3)));
    try fw.expect(v.swizzle(.y, .w, .z, .w).eql(UVec4.init(2, 4, 3, 4)));
    try fw.expect(v.swizzle(.y, .w, .w, .x).eql(UVec4.init(2, 4, 4, 1)));
    try fw.expect(v.swizzle(.y, .w, .w, .y).eql(UVec4.init(2, 4, 4, 2)));
    try fw.expect(v.swizzle(.y, .w, .w, .z).eql(UVec4.init(2, 4, 4, 3)));
    try fw.expect(v.swizzle(.y, .w, .w, .w).eql(UVec4.init(2, 4, 4, 4)));

    try fw.expect(v.swizzle(.z, .x, .x, .x).eql(UVec4.init(3, 1, 1, 1)));
    try fw.expect(v.swizzle(.z, .x, .x, .y).eql(UVec4.init(3, 1, 1, 2)));
    try fw.expect(v.swizzle(.z, .x, .x, .z).eql(UVec4.init(3, 1, 1, 3)));
    try fw.expect(v.swizzle(.z, .x, .x, .w).eql(UVec4.init(3, 1, 1, 4)));
    try fw.expect(v.swizzle(.z, .x, .y, .x).eql(UVec4.init(3, 1, 2, 1)));
    try fw.expect(v.swizzle(.z, .x, .y, .y).eql(UVec4.init(3, 1, 2, 2)));
    try fw.expect(v.swizzle(.z, .x, .y, .z).eql(UVec4.init(3, 1, 2, 3)));
    try fw.expect(v.swizzle(.z, .x, .y, .w).eql(UVec4.init(3, 1, 2, 4)));
    try fw.expect(v.swizzle(.z, .x, .z, .x).eql(UVec4.init(3, 1, 3, 1)));
    try fw.expect(v.swizzle(.z, .x, .z, .y).eql(UVec4.init(3, 1, 3, 2)));
    try fw.expect(v.swizzle(.z, .x, .z, .z).eql(UVec4.init(3, 1, 3, 3)));
    try fw.expect(v.swizzle(.z, .x, .z, .w).eql(UVec4.init(3, 1, 3, 4)));
    try fw.expect(v.swizzle(.z, .x, .w, .x).eql(UVec4.init(3, 1, 4, 1)));
    try fw.expect(v.swizzle(.z, .x, .w, .y).eql(UVec4.init(3, 1, 4, 2)));
    try fw.expect(v.swizzle(.z, .x, .w, .z).eql(UVec4.init(3, 1, 4, 3)));
    try fw.expect(v.swizzle(.z, .x, .w, .w).eql(UVec4.init(3, 1, 4, 4)));
    try fw.expect(v.swizzle(.z, .y, .x, .x).eql(UVec4.init(3, 2, 1, 1)));
    try fw.expect(v.swizzle(.z, .y, .x, .y).eql(UVec4.init(3, 2, 1, 2)));
    try fw.expect(v.swizzle(.z, .y, .x, .z).eql(UVec4.init(3, 2, 1, 3)));
    try fw.expect(v.swizzle(.z, .y, .x, .w).eql(UVec4.init(3, 2, 1, 4)));
    try fw.expect(v.swizzle(.z, .y, .y, .x).eql(UVec4.init(3, 2, 2, 1)));
    try fw.expect(v.swizzle(.z, .y, .y, .y).eql(UVec4.init(3, 2, 2, 2)));
    try fw.expect(v.swizzle(.z, .y, .y, .z).eql(UVec4.init(3, 2, 2, 3)));
    try fw.expect(v.swizzle(.z, .y, .y, .w).eql(UVec4.init(3, 2, 2, 4)));
    try fw.expect(v.swizzle(.z, .y, .z, .x).eql(UVec4.init(3, 2, 3, 1)));
    try fw.expect(v.swizzle(.z, .y, .z, .y).eql(UVec4.init(3, 2, 3, 2)));
    try fw.expect(v.swizzle(.z, .y, .z, .z).eql(UVec4.init(3, 2, 3, 3)));
    try fw.expect(v.swizzle(.z, .y, .z, .w).eql(UVec4.init(3, 2, 3, 4)));
    try fw.expect(v.swizzle(.z, .y, .w, .x).eql(UVec4.init(3, 2, 4, 1)));
    try fw.expect(v.swizzle(.z, .y, .w, .y).eql(UVec4.init(3, 2, 4, 2)));
    try fw.expect(v.swizzle(.z, .y, .w, .z).eql(UVec4.init(3, 2, 4, 3)));
    try fw.expect(v.swizzle(.z, .y, .w, .w).eql(UVec4.init(3, 2, 4, 4)));
    try fw.expect(v.swizzle(.z, .z, .x, .x).eql(UVec4.init(3, 3, 1, 1)));
    try fw.expect(v.swizzle(.z, .z, .x, .y).eql(UVec4.init(3, 3, 1, 2)));
    try fw.expect(v.swizzle(.z, .z, .x, .z).eql(UVec4.init(3, 3, 1, 3)));
    try fw.expect(v.swizzle(.z, .z, .x, .w).eql(UVec4.init(3, 3, 1, 4)));
    try fw.expect(v.swizzle(.z, .z, .y, .x).eql(UVec4.init(3, 3, 2, 1)));
    try fw.expect(v.swizzle(.z, .z, .y, .y).eql(UVec4.init(3, 3, 2, 2)));
    try fw.expect(v.swizzle(.z, .z, .y, .z).eql(UVec4.init(3, 3, 2, 3)));
    try fw.expect(v.swizzle(.z, .z, .y, .w).eql(UVec4.init(3, 3, 2, 4)));
    try fw.expect(v.swizzle(.z, .z, .z, .x).eql(UVec4.init(3, 3, 3, 1)));
    try fw.expect(v.swizzle(.z, .z, .z, .y).eql(UVec4.init(3, 3, 3, 2)));
    try fw.expect(v.swizzle(.z, .z, .z, .z).eql(UVec4.init(3, 3, 3, 3)));
    try fw.expect(v.swizzle(.z, .z, .z, .w).eql(UVec4.init(3, 3, 3, 4)));
    try fw.expect(v.swizzle(.z, .z, .w, .x).eql(UVec4.init(3, 3, 4, 1)));
    try fw.expect(v.swizzle(.z, .z, .w, .y).eql(UVec4.init(3, 3, 4, 2)));
    try fw.expect(v.swizzle(.z, .z, .w, .z).eql(UVec4.init(3, 3, 4, 3)));
    try fw.expect(v.swizzle(.z, .z, .w, .w).eql(UVec4.init(3, 3, 4, 4)));
    try fw.expect(v.swizzle(.z, .w, .x, .x).eql(UVec4.init(3, 4, 1, 1)));
    try fw.expect(v.swizzle(.z, .w, .x, .y).eql(UVec4.init(3, 4, 1, 2)));
    try fw.expect(v.swizzle(.z, .w, .x, .z).eql(UVec4.init(3, 4, 1, 3)));
    try fw.expect(v.swizzle(.z, .w, .x, .w).eql(UVec4.init(3, 4, 1, 4)));
    try fw.expect(v.swizzle(.z, .w, .y, .x).eql(UVec4.init(3, 4, 2, 1)));
    try fw.expect(v.swizzle(.z, .w, .y, .y).eql(UVec4.init(3, 4, 2, 2)));
    try fw.expect(v.swizzle(.z, .w, .y, .z).eql(UVec4.init(3, 4, 2, 3)));
    try fw.expect(v.swizzle(.z, .w, .y, .w).eql(UVec4.init(3, 4, 2, 4)));
    try fw.expect(v.swizzle(.z, .w, .z, .x).eql(UVec4.init(3, 4, 3, 1)));
    try fw.expect(v.swizzle(.z, .w, .z, .y).eql(UVec4.init(3, 4, 3, 2)));
    try fw.expect(v.swizzle(.z, .w, .z, .z).eql(UVec4.init(3, 4, 3, 3)));
    try fw.expect(v.swizzle(.z, .w, .z, .w).eql(UVec4.init(3, 4, 3, 4)));
    try fw.expect(v.swizzle(.z, .w, .w, .x).eql(UVec4.init(3, 4, 4, 1)));
    try fw.expect(v.swizzle(.z, .w, .w, .y).eql(UVec4.init(3, 4, 4, 2)));
    try fw.expect(v.swizzle(.z, .w, .w, .z).eql(UVec4.init(3, 4, 4, 3)));
    try fw.expect(v.swizzle(.z, .w, .w, .w).eql(UVec4.init(3, 4, 4, 4)));

    try fw.expect(v.swizzle(.w, .x, .x, .x).eql(UVec4.init(4, 1, 1, 1)));
    try fw.expect(v.swizzle(.w, .x, .x, .y).eql(UVec4.init(4, 1, 1, 2)));
    try fw.expect(v.swizzle(.w, .x, .x, .z).eql(UVec4.init(4, 1, 1, 3)));
    try fw.expect(v.swizzle(.w, .x, .x, .w).eql(UVec4.init(4, 1, 1, 4)));
    try fw.expect(v.swizzle(.w, .x, .y, .x).eql(UVec4.init(4, 1, 2, 1)));
    try fw.expect(v.swizzle(.w, .x, .y, .y).eql(UVec4.init(4, 1, 2, 2)));
    try fw.expect(v.swizzle(.w, .x, .y, .z).eql(UVec4.init(4, 1, 2, 3)));
    try fw.expect(v.swizzle(.w, .x, .y, .w).eql(UVec4.init(4, 1, 2, 4)));
    try fw.expect(v.swizzle(.w, .x, .z, .x).eql(UVec4.init(4, 1, 3, 1)));
    try fw.expect(v.swizzle(.w, .x, .z, .y).eql(UVec4.init(4, 1, 3, 2)));
    try fw.expect(v.swizzle(.w, .x, .z, .z).eql(UVec4.init(4, 1, 3, 3)));
    try fw.expect(v.swizzle(.w, .x, .z, .w).eql(UVec4.init(4, 1, 3, 4)));
    try fw.expect(v.swizzle(.w, .x, .w, .x).eql(UVec4.init(4, 1, 4, 1)));
    try fw.expect(v.swizzle(.w, .x, .w, .y).eql(UVec4.init(4, 1, 4, 2)));
    try fw.expect(v.swizzle(.w, .x, .w, .z).eql(UVec4.init(4, 1, 4, 3)));
    try fw.expect(v.swizzle(.w, .x, .w, .w).eql(UVec4.init(4, 1, 4, 4)));
    try fw.expect(v.swizzle(.w, .y, .x, .x).eql(UVec4.init(4, 2, 1, 1)));
    try fw.expect(v.swizzle(.w, .y, .x, .y).eql(UVec4.init(4, 2, 1, 2)));
    try fw.expect(v.swizzle(.w, .y, .x, .z).eql(UVec4.init(4, 2, 1, 3)));
    try fw.expect(v.swizzle(.w, .y, .x, .w).eql(UVec4.init(4, 2, 1, 4)));
    try fw.expect(v.swizzle(.w, .y, .y, .x).eql(UVec4.init(4, 2, 2, 1)));
    try fw.expect(v.swizzle(.w, .y, .y, .y).eql(UVec4.init(4, 2, 2, 2)));
    try fw.expect(v.swizzle(.w, .y, .y, .z).eql(UVec4.init(4, 2, 2, 3)));
    try fw.expect(v.swizzle(.w, .y, .y, .w).eql(UVec4.init(4, 2, 2, 4)));
    try fw.expect(v.swizzle(.w, .y, .z, .x).eql(UVec4.init(4, 2, 3, 1)));
    try fw.expect(v.swizzle(.w, .y, .z, .y).eql(UVec4.init(4, 2, 3, 2)));
    try fw.expect(v.swizzle(.w, .y, .z, .z).eql(UVec4.init(4, 2, 3, 3)));
    try fw.expect(v.swizzle(.w, .y, .z, .w).eql(UVec4.init(4, 2, 3, 4)));
    try fw.expect(v.swizzle(.w, .y, .w, .x).eql(UVec4.init(4, 2, 4, 1)));
    try fw.expect(v.swizzle(.w, .y, .w, .y).eql(UVec4.init(4, 2, 4, 2)));
    try fw.expect(v.swizzle(.w, .y, .w, .z).eql(UVec4.init(4, 2, 4, 3)));
    try fw.expect(v.swizzle(.w, .y, .w, .w).eql(UVec4.init(4, 2, 4, 4)));
    try fw.expect(v.swizzle(.w, .z, .x, .x).eql(UVec4.init(4, 3, 1, 1)));
    try fw.expect(v.swizzle(.w, .z, .x, .y).eql(UVec4.init(4, 3, 1, 2)));
    try fw.expect(v.swizzle(.w, .z, .x, .z).eql(UVec4.init(4, 3, 1, 3)));
    try fw.expect(v.swizzle(.w, .z, .x, .w).eql(UVec4.init(4, 3, 1, 4)));
    try fw.expect(v.swizzle(.w, .z, .y, .x).eql(UVec4.init(4, 3, 2, 1)));
    try fw.expect(v.swizzle(.w, .z, .y, .y).eql(UVec4.init(4, 3, 2, 2)));
    try fw.expect(v.swizzle(.w, .z, .y, .z).eql(UVec4.init(4, 3, 2, 3)));
    try fw.expect(v.swizzle(.w, .z, .y, .w).eql(UVec4.init(4, 3, 2, 4)));
    try fw.expect(v.swizzle(.w, .z, .z, .x).eql(UVec4.init(4, 3, 3, 1)));
    try fw.expect(v.swizzle(.w, .z, .z, .y).eql(UVec4.init(4, 3, 3, 2)));
    try fw.expect(v.swizzle(.w, .z, .z, .z).eql(UVec4.init(4, 3, 3, 3)));
    try fw.expect(v.swizzle(.w, .z, .z, .w).eql(UVec4.init(4, 3, 3, 4)));
    try fw.expect(v.swizzle(.w, .z, .w, .x).eql(UVec4.init(4, 3, 4, 1)));
    try fw.expect(v.swizzle(.w, .z, .w, .y).eql(UVec4.init(4, 3, 4, 2)));
    try fw.expect(v.swizzle(.w, .z, .w, .z).eql(UVec4.init(4, 3, 4, 3)));
    try fw.expect(v.swizzle(.w, .z, .w, .w).eql(UVec4.init(4, 3, 4, 4)));
    try fw.expect(v.swizzle(.w, .w, .x, .x).eql(UVec4.init(4, 4, 1, 1)));
    try fw.expect(v.swizzle(.w, .w, .x, .y).eql(UVec4.init(4, 4, 1, 2)));
    try fw.expect(v.swizzle(.w, .w, .x, .z).eql(UVec4.init(4, 4, 1, 3)));
    try fw.expect(v.swizzle(.w, .w, .x, .w).eql(UVec4.init(4, 4, 1, 4)));
    try fw.expect(v.swizzle(.w, .w, .y, .x).eql(UVec4.init(4, 4, 2, 1)));
    try fw.expect(v.swizzle(.w, .w, .y, .y).eql(UVec4.init(4, 4, 2, 2)));
    try fw.expect(v.swizzle(.w, .w, .y, .z).eql(UVec4.init(4, 4, 2, 3)));
    try fw.expect(v.swizzle(.w, .w, .y, .w).eql(UVec4.init(4, 4, 2, 4)));
    try fw.expect(v.swizzle(.w, .w, .z, .x).eql(UVec4.init(4, 4, 3, 1)));
    try fw.expect(v.swizzle(.w, .w, .z, .y).eql(UVec4.init(4, 4, 3, 2)));
    try fw.expect(v.swizzle(.w, .w, .z, .z).eql(UVec4.init(4, 4, 3, 3)));
    try fw.expect(v.swizzle(.w, .w, .z, .w).eql(UVec4.init(4, 4, 3, 4)));
    try fw.expect(v.swizzle(.w, .w, .w, .x).eql(UVec4.init(4, 4, 4, 1)));
    try fw.expect(v.swizzle(.w, .w, .w, .y).eql(UVec4.init(4, 4, 4, 2)));
    try fw.expect(v.swizzle(.w, .w, .w, .z).eql(UVec4.init(4, 4, 4, 3)));
    try fw.expect(v.swizzle(.w, .w, .w, .w).eql(UVec4.init(4, 4, 4, 4)));
}

test "TestUVec4Dot" {
    try fw.expectEqual(1 * 5 + 2 * 6 + 3 * 7 + 4 * 8, UVec4.init(1, 2, 3, 4).dot(UVec4.init(5, 6, 7, 8)));
    try fw.expect(UVec4.init(1, 2, 3, 4).dotV(UVec4.init(5, 6, 7, 8)).eql(UVec4.replicate(1 * 5 + 2 * 6 + 3 * 7 + 4 * 8)));
}

test "TestUVec4Cast" {
    try fw.expect(UVec4.init(1, 2, 3, 4).toFloat().eql(Vec4.init(1, 2, 3, 4)));
    try fw.expect(UVec4.init(0x3f800000, 0x40000000, 0x40400000, 0x40800000).reinterpretAsFloat().eql(Vec4.init(1, 2, 3, 4)));
}

test "TestUVec4ExtractUInt16" {
    const data = [_]u32{ 0x0b020a01, 0x0d040c03, 0x0b060a05, 0x0d080c07 };
    const vector = UVec4.loadInt4(&data);

    try fw.expect(vector.expand4Uint16Lo().eql(UVec4.init(0x0a01, 0x0b02, 0x0c03, 0x0d04)));
    try fw.expect(vector.expand4Uint16Hi().eql(UVec4.init(0x0a05, 0x0b06, 0x0c07, 0x0d08)));
}

test "TestUVec4ExtractBytes" {
    const data = [_]u32{ 0x14131211, 0x24232221, 0x34333231, 0x44434241 };
    const vector = UVec4.loadInt4(&data);

    try fw.expect(vector.expand4Byte0().eql(UVec4.init(0x11, 0x12, 0x13, 0x14)));
    try fw.expect(vector.expand4Byte4().eql(UVec4.init(0x21, 0x22, 0x23, 0x24)));
    try fw.expect(vector.expand4Byte8().eql(UVec4.init(0x31, 0x32, 0x33, 0x34)));
    try fw.expect(vector.expand4Byte12().eql(UVec4.init(0x41, 0x42, 0x43, 0x44)));
}

test "TestUVec4ShiftComponents" {
    const v = UVec4.init(1, 2, 3, 4);

    try fw.expect(v.shiftComponents4Minus(4).eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(v.shiftComponents4Minus(3).eql(UVec4.init(2, 3, 4, 0)));
    try fw.expect(v.shiftComponents4Minus(2).eql(UVec4.init(3, 4, 0, 0)));
    try fw.expect(v.shiftComponents4Minus(1).eql(UVec4.init(4, 0, 0, 0)));
    try fw.expect(v.shiftComponents4Minus(0).eql(UVec4.init(0, 0, 0, 0)));
}

test "TestUVec4Sort4True" {
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0x00000000, 0x00000000, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(4, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(2, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 2, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(3, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 3, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(2, 3, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0x00000000), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 2, 3, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0x00000000, 0x00000000, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(4, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0x00000000, 0x00000000, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0xffffffff, 0x00000000, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(2, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0xffffffff, 0x00000000, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 2, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0x00000000, 0xffffffff, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(3, 4, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0x00000000, 0xffffffff, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 3, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0x00000000, 0xffffffff, 0xffffffff, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(2, 3, 4, 4)));
    try fw.expect(UVec4.sort4True(UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff), UVec4.init(1, 2, 3, 4)).eql(UVec4.init(1, 2, 3, 4)));
}

test "TestUVec4ConvertToString" {
    const v = UVec4.init(1, 2, 3, 4);
    var buf: [64]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4", try std.fmt.bufPrint(&buf, "{f}", .{v}));
}
