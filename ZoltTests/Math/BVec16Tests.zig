//! Port of: UnitTests/Math/BVec16Tests.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const BVec16 = zolt.BVec16;

test "TestBVec16Construct" {
    var v = BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

    try expectEqual(1, v.getComponent(0));
    try expectEqual(2, v.getComponent(1));
    try expectEqual(3, v.getComponent(2));
    try expectEqual(4, v.getComponent(3));
    try expectEqual(5, v.getComponent(4));
    try expectEqual(6, v.getComponent(5));
    try expectEqual(7, v.getComponent(6));
    try expectEqual(8, v.getComponent(7));
    try expectEqual(9, v.getComponent(8));
    try expectEqual(10, v.getComponent(9));
    try expectEqual(11, v.getComponent(10));
    try expectEqual(12, v.getComponent(11));
    try expectEqual(13, v.getComponent(12));
    try expectEqual(14, v.getComponent(13));
    try expectEqual(15, v.getComponent(14));
    try expectEqual(16, v.getComponent(15));

    // Test == and != operators
    try expect(v.eql(BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)));
    try expect(!v.eql(BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 10, 9, 11, 12, 13, 14, 15, 16)));

    // Check element modification
    try expectEqual(16, v.getComponent(15)); // Check const operator
    v.setComponent(15, 17);
    try expectEqual(17, v.getComponent(15));
    try expect(v.eql(BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17)));
}

test "TestBVec16LoadByte16" {
    const u16_values = [16]u8{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 };
    try expect(BVec16.loadByte16(&u16_values).eql(BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16)));
}

test "TestBVec16Zero" {
    const v = BVec16.zero();

    for (0..16) |i|
        try expectEqual(0, v.getComponent(@intCast(i)));
}

test "TestBVec16Replicate" {
    try expect(BVec16.replicate(2).eql(BVec16.init(2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2)));
}

test "TestBVec16Comparisons" {
    const eq = BVec16.equals(BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16), BVec16.init(6, 7, 3, 4, 5, 6, 7, 5, 9, 10, 11, 12, 13, 14, 15, 13));
    try expectEqual(0b0111111101111100, eq.getTrues());
    try expect(eq.testAnyTrue());
    try expect(!eq.testAllTrue());
}

test "TestBVec16BitOps" {
    // Test all bit permutations
    const v1 = BVec16.init(0b011, 0b0110, 0b01100, 0b011000, 0b0110000, 0b01100000, 0b011, 0b0110, 0b01100, 0b011000, 0b0110000, 0b01100000, 0b011, 0b0110, 0b01100, 0b011000);
    const v2 = BVec16.init(0b101, 0b1010, 0b10100, 0b101000, 0b1010000, 0b10100000, 0b101, 0b1010, 0b10100, 0b101000, 0b1010000, 0b10100000, 0b101, 0b1010, 0b10100, 0b101000);

    try expect(BVec16.bitOr(v1, v2).eql(BVec16.init(0b111, 0b1110, 0b11100, 0b111000, 0b1110000, 0b11100000, 0b111, 0b1110, 0b11100, 0b111000, 0b1110000, 0b11100000, 0b111, 0b1110, 0b11100, 0b111000)));
    try expect(BVec16.bitXor(v1, v2).eql(BVec16.init(0b110, 0b1100, 0b11000, 0b110000, 0b1100000, 0b11000000, 0b110, 0b1100, 0b11000, 0b110000, 0b1100000, 0b11000000, 0b110, 0b1100, 0b11000, 0b110000)));
    try expect(BVec16.bitAnd(v1, v2).eql(BVec16.init(0b001, 0b0010, 0b00100, 0b001000, 0b0010000, 0b00100000, 0b001, 0b0010, 0b00100, 0b001000, 0b0010000, 0b00100000, 0b001, 0b0010, 0b00100, 0b001000)));

    try expect(BVec16.bitNot(v1).eql(BVec16.init(0b11111100, 0b11111001, 0b11110011, 0b11100111, 0b11001111, 0b10011111, 0b11111100, 0b11111001, 0b11110011, 0b11100111, 0b11001111, 0b10011111, 0b11111100, 0b11111001, 0b11110011, 0b11100111)));
    try expect(BVec16.bitNot(v2).eql(BVec16.init(0b11111010, 0b11110101, 0b11101011, 0b11010111, 0b10101111, 0b01011111, 0b11111010, 0b11110101, 0b11101011, 0b11010111, 0b10101111, 0b01011111, 0b11111010, 0b11110101, 0b11101011, 0b11010111)));
}

test "TestBVec16ToString" {
    const v = BVec16.init(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

    var buffer: [128]u8 = undefined;
    try std.testing.expectEqualStrings("1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16", try std.fmt.bufPrint(&buffer, "{f}", .{v}));
}
