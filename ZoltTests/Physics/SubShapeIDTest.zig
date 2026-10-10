//! Port of: UnitTests/Physics/SubShapeIDTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;

const SSPair = struct {
    value: u32,
    num_bits: u32,
};

// Helper function that pushes sub shape ID's on the creator and checks that they come out again
fn testPushPop(pairs: []const SSPair) !void {
    // Push all id's on the creator
    var creator: SubShapeIDCreator = .{};
    var total_bits: u32 = 0;
    for (pairs) |p| {
        creator = creator.pushID(p.value, p.num_bits);
        total_bits += p.num_bits;
    }
    try fw.expectEqual(total_bits, creator.getNumBitsWritten());

    // Now pop all parts
    var id = creator.getID();
    for (pairs) |p| {
        // There should be data (note there is a possibility of a false positive if the bit pattern is all 1's)
        try fw.expect(!id.isEmpty());

        // Pop the part
        const popped = id.popID(p.num_bits);

        // Check value
        try fw.expectEqual(p.value, popped.id);

        // Continue with the remainder
        id = popped.remainder;
    }

    try fw.expect(id.isEmpty());
}

test "SubShapeIDTest" {
    // Test storing some values
    try testPushPop(&.{ .{ .value = 0b110101010, .num_bits = 9 }, .{ .value = 0b0101010101, .num_bits = 10 }, .{ .value = 0b10110101010, .num_bits = 11 } });

    // Test storing some values with a different pattern
    try testPushPop(&.{ .{ .value = 0b001010101, .num_bits = 9 }, .{ .value = 0b1010101010, .num_bits = 10 }, .{ .value = 0b01001010101, .num_bits = 11 } });

    // Test storing up to 32 bits
    try testPushPop(&.{ .{ .value = 0b10, .num_bits = 2 }, .{ .value = 0b1110101010, .num_bits = 10 }, .{ .value = 0b0101010101, .num_bits = 10 }, .{ .value = 0b1010101010, .num_bits = 10 } });

    // Test storing up to 32 bits with a different pattern
    try testPushPop(&.{ .{ .value = 0b0001010101, .num_bits = 10 }, .{ .value = 0b1010101010, .num_bits = 10 }, .{ .value = 0b0101010101, .num_bits = 10 }, .{ .value = 0b01, .num_bits = 2 } });

    // Test storing 0 bits
    try testPushPop(&.{ .{ .value = 0b10, .num_bits = 2 }, .{ .value = 0b1110101010, .num_bits = 10 }, .{ .value = 0, .num_bits = 0 }, .{ .value = 0b0101010101, .num_bits = 10 }, .{ .value = 0, .num_bits = 0 }, .{ .value = 0b1010101010, .num_bits = 10 } });

    // Test 32 bits at once
    try testPushPop(&.{.{ .value = 0b10101010101010101010101010101010, .num_bits = 32 }});

    // Test 32 bits at once with a different pattern
    try testPushPop(&.{.{ .value = 0b01010101010101010101010101010101, .num_bits = 32 }});
}
