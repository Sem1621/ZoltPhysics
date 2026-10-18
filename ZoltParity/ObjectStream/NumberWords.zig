//! The words of P6 (SerializationArchitecture.md section 5, S12): integers, floats, bools and identifiers as a text
//! object stream may hold them, valid or not. ObjectStreamParity.zig reads every word with C++'s ObjectStreamTextIn
//! (`IStringStream >> T`, `strtoul` for identifiers) and with Zolt's reader.
//!
//! The words avoid D14: a float word whose value is exactly a subnormal of the read type (glibc reads it, Zolt does not).
//! A hexadecimal word is never a subnormal of the read type. A decimal word has at most 25 significant digits, and an
//! exact subnormal needs many more (105 for the smallest float).

const std = @import("std");

const Allocator = std.mem.Allocator;

/// The word types of jolt_os_read_word (ObjectStreamReference.cpp)
pub const WordType = enum(c_int) { uint8, uint16, int, uint32, uint64, float, double, bool, identifier, count };

/// Words for `word_type`: fixed edge cases, then random words until there are `count`
pub fn generate(arena: Allocator, r: std.Random, word_type: WordType, count: usize) ![]const []const u8 {
    var words: std.ArrayList([]const u8) = .empty;
    const fixed: []const []const u8 = switch (word_type) {
        .uint8, .uint16, .int, .uint32, .uint64, .count => &integer_edges,
        .float, .double => &float_edges,
        .bool => &bool_edges,
        .identifier => &identifier_edges,
    };
    for (fixed) |w| try words.append(arena, w);
    // Every fixed word also with leading white space, a trailing word and another case
    for (fixed) |w| {
        try words.append(arena, try std.fmt.allocPrint(arena, " \t\r\n{s}", .{w}));
        try words.append(arena, try std.fmt.allocPrint(arena, "{s} 7", .{w}));
        const upper = try arena.dupe(u8, w);
        for (upper) |*c| c.* = std.ascii.toUpper(c.*);
        try words.append(arena, upper);
    }
    while (words.items.len < count) {
        const word = switch (word_type) {
            .uint8, .uint16, .int, .uint32, .uint64, .count => try randomInteger(arena, r),
            .float => try randomFloatWord(arena, r, f32),
            .double => try randomFloatWord(arena, r, f64),
            .bool => try randomBool(arena, r),
            .identifier => try randomIdentifier(arena, r),
        };
        if (word) |w| try words.append(arena, w);
    }
    return words.items;
}

const integer_edges = [_][]const u8{
    "",                      " ",                     "0",                       "-0",                             "+0",
    "00000",                 "1",                     "-1",                      "+1",                             "255",
    "256",                   "-255",                  "-256",                    "65535",                          "65536",
    "-65536",                "2147483647",            "2147483648",              "-2147483648",                    "-2147483649",
    "4294967295",            "4294967296",            "-4294967295",             "-4294967296",                    "9223372036854775807",
    "9223372036854775808",   "-9223372036854775808",  "-9223372036854775809",    "18446744073709551615",           "18446744073709551616",
    "-18446744073709551615", "-18446744073709551616", "99999999999999999999999", "000000000000000000000000000001", "12abc",
    "12.5",                  "12e3",                  "0x10",                    "0X1F",                           "-",
    "+",                     "+-1",                   "-+1",                     "--1",                            "++1",
    "abc",                   ".5",                    "1,000",                   "1_000",                          "\x00",
    "7\x00",                 "\xff",
    "٣",
};

const float_edges = [_][]const u8{
    "",                               "0",                                            "-0",                      "+0",                      "0.0",
    "-0.0",                           "0.000000",                                     "-0.000000",               "0e0",                     "0e-400",
    "0x0p0",                          "-0x0p0",                                       "1",                       "-1",                      ".5",
    "5.",                             "-.5",                                          "+2.5",                    "1.2.3",                   "1..2",
    "1.5abc",                         "3zz",                                          "1e",                      "1e+",                     "1e-",
    ".",                              "-.",                                           ".e1",                     "e1",                      "+",
    "-",                              "--1",                                          "+-1",                     "0x",                      "0x.",
    "0xp1",                           "0x1p",                                         "0x1.8p3",                 "0X1P-126",                "0x1.fffffep+127",
    "0x1.fffffffffffffp+1023",        "0x1p-1022",                                    "0x1.0000000000001p0",     "0x1g",                    "0x.8",
    "inf",                            "-inf",                                         "nan",                     "-nan",                    "infinity",
    "1e999",                          "-1e999",                                       "1e-999",                  "-1e-999",                 "1e38",
    "1e39",                           "3.40282347e+38",                               "3.40282357e+38",          "3.4028235e38",            "3.4028235677973366e38",
    "3.4028235677973362e38",          "1.17549435e-38",                               "1.17549430e-38",          "1.17549421e-38",          "1.1754943e-38",
    "1.1754942e-38",                  "1.40129846e-45",                               "7.0e-46",                 "1e-50",                   "4.9406564584124654e-324",
    "2.4703282292062327e-324",        "2.2250738585072014e-308",                      "2.2250738585072011e-308", "2.2250738585072009e-308", "1.7976931348623157e308",
    "1.7976931348623158e308",         "1.7976931348623159e308",                       "1e309",                   "0.1",                     "0.30000000000000004",
    "123456789012345678901234567890", "0.000000000000000000000000000000000000000001", "1e+0038",                 "1E5",                     "1e5x",
    "1.e5",                           "1e5.5",                                        "1e+-5",                   "12abc",                   "0.1f",
    // Just below the smallest normal number: rounded to it, or tiny (strtof / strtod ERANGE)
    "1.17549429e-38",                 "1.17549431e-38",                               "1.175494315e-38",         "1.17549434e-38",          "2.2250738585072012e-308",
    "2.2250738585072013e-308",        "2.22507385850720135e-308",
};

const bool_edges = [_][]const u8{ "", "true", "false", "TRUE", "FALSE", "True", "fAlSe", "truex", "fals", "t", "f", "1", "0", "yes", "no", "true\x00", " false" };

const identifier_edges = [_][]const u8{
    "",                 "0",                 "00000000",          "00000001",           "FFFFFFFF",
    "ffffffff",         "100000000",         "100000001",         "0x10",               "0X10",
    "0x",               "0xg",               "0x0x1",             "-1",                 "+1",
    "-0x10",            "+0x10",             "--1",               "zz",                 "1Fz",
    "FFFFFFFFFFFFFFFF", "10000000000000000", "-FFFFFFFFFFFFFFFF", "-10000000000000000", "0000000000000000000000001",
    "DEADBEEF",         "deadbeefcafe",      " 1A",               "1A 2B",              "g",
};

fn randomDigits(arena: Allocator, r: std.Random, min: usize, max: usize) ![]u8 {
    const digits = try arena.alloc(u8, r.intRangeAtMost(usize, min, max));
    for (digits) |*d| d.* = '0' + r.uintLessThan(u8, 10);
    return digits;
}

fn randomInteger(arena: Allocator, r: std.Random) !?[]const u8 {
    const signs = [_][]const u8{ "", "", "", "-", "+", "+-", "--" };
    const suffixes = [_][]const u8{ "", "", "", "", "abc", ".5", "e3", "x", "\x00", "-1" };
    const sign = signs[r.uintLessThan(usize, signs.len)];
    const suffix = suffixes[r.uintLessThan(usize, suffixes.len)];
    switch (r.uintLessThan(u32, 4)) {
        // A random number of any width
        0 => return try std.fmt.allocPrint(arena, "{s}{d}{s}", .{ sign, r.int(u64) >> r.int(u6), suffix }),
        // Near a power of two
        1 => {
            const p = @as(u128, 1) << r.intRangeAtMost(u7, 7, 64);
            const v = p + r.intRangeAtMost(u128, 0, 2) -% 1;
            return try std.fmt.allocPrint(arena, "{s}{d}{s}", .{ sign, v, suffix });
        },
        // Many digits, maybe with leading zeros
        2 => return try std.fmt.allocPrint(arena, "{s}{s}{s}", .{ sign, try randomDigits(arena, r, 1, 30), suffix }),
        else => return try std.fmt.allocPrint(arena, "{s}{d}{s}", .{ sign, r.uintLessThan(u32, 300), suffix }),
    }
}

fn randomFloatWord(arena: Allocator, r: std.Random, comptime T: type) !?[]const u8 {
    const Bits = std.meta.Int(.unsigned, @bitSizeOf(T));
    const signs = [_][]const u8{ "", "", "", "-", "+" };
    const sign = signs[r.uintLessThan(usize, signs.len)];
    switch (r.uintLessThan(u32, 6)) {
        // A random value of the type (also subnormals, which fail), printed exactly
        0, 1 => {
            const value: T = @bitCast(r.int(Bits));
            if (!std.math.isFinite(value)) return null;
            return try std.fmt.allocPrint(arena, "{e}", .{value});
        },
        // A random value of the other width (rounding)
        2 => {
            const Other = if (T == f32) f64 else f32;
            const value: Other = @bitCast(r.int(std.meta.Int(.unsigned, @bitSizeOf(Other))));
            if (!std.math.isFinite(value)) return null;
            return try std.fmt.allocPrint(arena, "{e}", .{value});
        },
        // A decimal mantissa with a random point and exponent
        3 => {
            const digits = try randomDigits(arena, r, 1, 25);
            const point = r.uintLessThan(usize, digits.len + 1);
            const exponent = r.intRangeAtMost(i32, -330, 330);
            const e = if (r.boolean()) "e" else "E";
            return try std.fmt.allocPrint(arena, "{s}{s}.{s}{s}{d}", .{ sign, digits[0..point], digits[point..], e, exponent });
        },
        // Hexadecimal, never a subnormal of T (D14)
        4 => {
            const mantissa = r.int(u32);
            const exponent = r.intRangeAtMost(i32, -1100, 1100);
            const word = try std.fmt.allocPrint(arena, "{s}0x{x}.{x}p{d}", .{ sign, mantissa >> 16, mantissa & 0xffff, exponent });
            const value = std.fmt.parseFloat(f128, word) catch return word;
            if (value != 0 and @abs(value) < std.math.floatMin(T)) return null;
            return word;
        },
        // A short decimal with a garbage suffix
        else => {
            const suffixes = [_][]const u8{ "", "abc", ".", ".5", "e", "e+", "x", "p1", "f", "-" };
            return try std.fmt.allocPrint(arena, "{s}{d}.{d}{s}", .{ sign, r.uintLessThan(u32, 1000), r.uintLessThan(u32, 1000), suffixes[r.uintLessThan(usize, suffixes.len)] });
        },
    }
}

fn randomBool(arena: Allocator, r: std.Random) !?[]const u8 {
    const words = [_][]const u8{ "true", "false", "tru", "falsee", "1", "0" };
    const word = try arena.dupe(u8, words[r.uintLessThan(usize, words.len)]);
    for (word) |*c| {
        if (r.boolean()) c.* = std.ascii.toUpper(c.*);
    }
    return word;
}

fn randomIdentifier(arena: Allocator, r: std.Random) !?[]const u8 {
    const prefixes = [_][]const u8{ "", "", "", "0x", "0X", "-", "+", "-0x" };
    const suffixes = [_][]const u8{ "", "", "", "z", "g1", ".5", " 1" };
    const prefix = prefixes[r.uintLessThan(usize, prefixes.len)];
    const suffix = suffixes[r.uintLessThan(usize, suffixes.len)];
    const value = r.int(u64) >> r.int(u6);
    switch (r.uintLessThan(u32, 3)) {
        0 => return try std.fmt.allocPrint(arena, "{s}{X:0>8}{s}", .{ prefix, value, suffix }),
        1 => return try std.fmt.allocPrint(arena, "{s}{x}{s}", .{ prefix, value, suffix }),
        // More than 64 bits
        else => return try std.fmt.allocPrint(arena, "{s}{x}{x:0>16}{s}", .{ prefix, r.uintLessThan(u32, 0x100), value, suffix }),
    }
}
