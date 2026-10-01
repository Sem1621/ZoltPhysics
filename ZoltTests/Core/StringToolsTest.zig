//! Port of: UnitTests/Core/StringToolsTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expectEqualStrings = std.testing.expectEqualStrings;

/// Free an Array<String> (the strings are owned by the allocator)
fn freeStrings(allocator: std.mem.Allocator, vector: *std.ArrayList([]u8)) void {
    for (vector.items) |s|
        allocator.free(s);
    vector.deinit(allocator);
}

test "TestStringFormat" {
    const allocator = std.testing.allocator;
    const s = try zolt.stringFormat(allocator, "Test: {d}", .{1234});
    defer allocator.free(s);
    try expectEqualStrings("Test: 1234", s);
}

test "TestConvertToString" {
    const allocator = std.testing.allocator;

    const s1 = try zolt.convertToString(allocator, 1234);
    defer allocator.free(s1);
    try expectEqualStrings("1234", s1);

    const s2 = try zolt.convertToString(allocator, -1);
    defer allocator.free(s2);
    try expectEqualStrings("-1", s2);

    const s3 = try zolt.convertToString(allocator, @as(u64, 0x7fffffffffffffff));
    defer allocator.free(s3);
    try expectEqualStrings("9223372036854775807", s3);
}

test "StringReplace" {
    const allocator = std.testing.allocator;
    var value = try allocator.dupe(u8, "Hello this si si a test");
    defer allocator.free(value);
    try zolt.stringReplace(allocator, &value, "si", "is");
    try expectEqualStrings("Hello this is is a test", value);
    try zolt.stringReplace(allocator, &value, "is is", "is");
    try expectEqualStrings("Hello this is a test", value);
    try zolt.stringReplace(allocator, &value, "Hello", "Bye");
    try expectEqualStrings("Bye this is a test", value);
    try zolt.stringReplace(allocator, &value, "a test", "complete");
    try expectEqualStrings("Bye this is complete", value);
}

test "StringToVector" {
    const allocator = std.testing.allocator;
    var value: std.ArrayList([]u8) = .empty;
    defer freeStrings(allocator, &value);

    try zolt.stringToVector(allocator, "", &value, .{});
    try fw.expect(value.items.len == 0);

    try zolt.stringToVector(allocator, "a,b,c", &value, .{});
    try expectEqualStrings("a", value.items[0]);
    try expectEqualStrings("b", value.items[1]);
    try expectEqualStrings("c", value.items[2]);

    try zolt.stringToVector(allocator, "a,.b,.c,", &value, .{ .delimiter = "." });
    try expectEqualStrings("a,", value.items[0]);
    try expectEqualStrings("b,", value.items[1]);
    try expectEqualStrings("c,", value.items[2]);
}

test "VectorToString" {
    const allocator = std.testing.allocator;
    var input: []const []const u8 = &.{};

    const value1 = try zolt.vectorToString(allocator, input, .{});
    defer allocator.free(value1);
    try fw.expect(value1.len == 0);

    input = &.{ "a", "b", "c" };
    const value2 = try zolt.vectorToString(allocator, input, .{});
    defer allocator.free(value2);
    try expectEqualStrings("a,b,c", value2);

    const value3 = try zolt.vectorToString(allocator, input, .{ .delimiter = ", " });
    defer allocator.free(value3);
    try expectEqualStrings("a, b, c", value3);
}

test "ToLower" {
    const allocator = std.testing.allocator;
    const s = try zolt.toLower(allocator, "123 HeLlO!");
    defer allocator.free(s);
    try expectEqualStrings("123 hello!", s);
}

test "NibbleToBinary" {
    const nibbleToBinary = zolt.nibbleToBinary;
    try expectEqualStrings("0000", nibbleToBinary(0b0000));
    try expectEqualStrings("0001", nibbleToBinary(0b0001));
    try expectEqualStrings("0010", nibbleToBinary(0b0010));
    try expectEqualStrings("0011", nibbleToBinary(0b0011));
    try expectEqualStrings("0100", nibbleToBinary(0b0100));
    try expectEqualStrings("0101", nibbleToBinary(0b0101));
    try expectEqualStrings("0110", nibbleToBinary(0b0110));
    try expectEqualStrings("0111", nibbleToBinary(0b0111));
    try expectEqualStrings("1000", nibbleToBinary(0b1000));
    try expectEqualStrings("1001", nibbleToBinary(0b1001));
    try expectEqualStrings("1010", nibbleToBinary(0b1010));
    try expectEqualStrings("1011", nibbleToBinary(0b1011));
    try expectEqualStrings("1100", nibbleToBinary(0b1100));
    try expectEqualStrings("1101", nibbleToBinary(0b1101));
    try expectEqualStrings("1110", nibbleToBinary(0b1110));
    try expectEqualStrings("1111", nibbleToBinary(0b1111));

    try expectEqualStrings("0000", nibbleToBinary(0xfffffff0));
}
