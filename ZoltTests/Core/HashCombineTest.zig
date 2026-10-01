//! Port of: UnitTests/Core/HashCombineTest.cpp

const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const HashCombine = zolt.HashCombine;

test "TestHashBytes" {
    try fw.expectEqual(@as(u64, 2733878766136413408), HashCombine.hashBytes("This is a test"));
}

test "TestHashString" {
    try fw.expectEqual(@as(u64, 2733878766136413408), HashCombine.hashString("This is a test"));
}

test "TestHashStruct" {
    // Hash<const char *>, Hash<std::string_view> and Hash<String> all map to hashing the characters
    const char_test: []const u8 = "This is a test";
    try fw.expectEqual(@as(u64, 2733878766136413408), HashCombine.hash(char_test));

    try fw.expectEqual(@as(u64, 2733878766136413408), HashCombine.hash("This is a test"));
}

test "TestHashCombine" {
    const val1: i32 = 0;
    const val1_hash = HashCombine.hash(val1);
    const val2: i32 = 1;
    const val2_hash = HashCombine.hash(val2);

    // Check non-commutative
    var seed1 = val1_hash;
    HashCombine.hashCombine(&seed1, val2);
    var seed2 = val2_hash;
    HashCombine.hashCombine(&seed2, val1);
    try fw.expect(seed1 != seed2);

    // Check that adding a 0 changes the hash
    var seed3 = val1_hash;
    HashCombine.hashCombine(&seed3, val1);
    try fw.expect(seed3 != val1_hash);
}
