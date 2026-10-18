//! Port of: Jolt/ObjectStream/ObjectStreamTextIn.h, Jolt/ObjectStream/ObjectStreamTextIn.cpp
//! Status: complete
//!
//! The text format of ObjectStreamIn, a member of its format union (ObjectStreamIn.zig). Words are separated by C
//! `isspace` characters (CR, LF, tab, ...); data type words and bools are case insensitive.
//!
//! Numbers: Jolt reads a word with `IStringStream >> value`. The reference platform of `zig build parity` is libc++
//! (with glibc's strtof / strtod), and Zolt follows its num_get (SerializationArchitecture.md S12):
//! - an integer is an optional sign and the decimal digits that follow, the rest of the word is ignored ("12abc" is 12);
//!   an unsigned value is negated modulo 2^n ("-1" is 4294967295 for a uint32); out of range fails; uint8 and uint16
//!   are read as uint32 and truncated;
//! - a float collects the characters of num_get's stage 2 ("0123456789abcdefABCDEFxX+-pP" and one '.'), which must
//!   then form a whole decimal or hexadecimal number ("1.5abc" fails); the result is rounded once, directly to f32 /
//!   f64, and an overflow, a subnormal result or a nonzero number that rounds to zero fails (strtof's ERANGE). So a
//!   subnormal that Jolt writes cannot be read back, on this platform also in Jolt;
//! - an identifier is `strtoul(word, 16)` truncated to 32 bits; only a 64 bit overflow fails (Jolt also tests a stale
//!   errno, divergence D7).
//! The token parity test of the foundation (P6) pins these rules against the C++ library.
//!
//! Reads that fail return error.InvalidObjectStream (Jolt: `return false`); the end of the stream is such a failure, a
//! failing reader is error.ReadFailed.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Reader = std.Io.Reader;
const Float3 = @import("../Math/Float3.zig").Float3;
const Float4 = @import("../Math/Float4.zig").Float4;
const Double3 = @import("../Math/Double3.zig").Double3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Quat = @import("../Math/Quat.zig").Quat;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const DMat44 = @import("../Math/DMat44.zig").DMat44;
const ObjectStream = @import("ObjectStream.zig");
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const Identifier = ObjectStream.Identifier;

const log = std.log.scoped(.zolt);

pub const Error = error{ OutOfMemory, ReadFailed, InvalidObjectStream };

/// Implementation of ObjectStream text input stream
pub const ObjectStreamTextIn = struct {
    reader: *Reader,
    /// The last word read (reused buffer, owned by the stream's allocator)
    word: std.ArrayList(u8) = .empty,

    pub fn deinit(self: *ObjectStreamTextIn, allocator: Allocator) void {
        self.word.deinit(allocator);
    }

    /// The next character, null at the end of the stream (ReadChar)
    fn readChar(self: *ObjectStreamTextIn) Error!?u8 {
        return self.reader.takeByte() catch |err| switch (err) {
            error.EndOfStream => null,
            error.ReadFailed => error.ReadFailed,
        };
    }

    /// Read a word (ReadWord): skip white space, then read up to the next white space, which is consumed. The result is
    /// valid until the next read.
    fn readWord(self: *ObjectStreamTextIn, allocator: Allocator) Error![]const u8 {
        self.word.clearRetainingCapacity();

        // Skip whitespace
        var c = while (true) {
            const c = try self.readChar() orelse return error.InvalidObjectStream;
            if (!std.ascii.isWhitespace(c)) break c;
        };

        // Read word
        while (true) {
            try self.word.append(allocator, c);
            c = try self.readChar() orelse break;
            if (std.ascii.isWhitespace(c)) break;
        }
        return self.word.items;
    }

    pub fn readDataType(self: *ObjectStreamTextIn, allocator: Allocator) Error!OSDataType {
        const token = try self.readWord(allocator);
        const table = [_]struct { []const u8, OSDataType }{
            .{ "declare", .declare },   .{ "object", .object },   .{ "instance", .instance }, .{ "pointer", .pointer },
            .{ "array", .array },       .{ "uint8", .t_uint8 },   .{ "uint16", .t_uint16 },   .{ "int", .t_int },
            .{ "uint32", .t_uint32 },   .{ "uint64", .t_uint64 }, .{ "float", .t_float },     .{ "double", .t_double },
            .{ "bool", .t_bool },       .{ "string", .t_string }, .{ "float3", .t_float3 },   .{ "float4", .t_float4 },
            .{ "double3", .t_double3 }, .{ "vec3", .t_vec3 },     .{ "dvec3", .t_dvec3 },     .{ "vec4", .t_vec4 },
            .{ "uvec4", .t_uvec4 },     .{ "quat", .t_quat },     .{ "mat44", .t_mat44 },     .{ "dmat44", .t_dmat44 },
        };
        for (table) |entry| {
            if (std.ascii.eqlIgnoreCase(token, entry[0])) return entry[1];
        }
        log.info("ObjectStreamTextIn: Found unknown data type.", .{});
        return error.InvalidObjectStream;
    }

    /// A class or attribute name, valid until the next read
    pub fn readName(self: *ObjectStreamTextIn, allocator: Allocator) Error![]const u8 {
        return self.readWord(allocator);
    }

    /// An identifier: `(uint32)strtoul(word, nullptr, 16)` (Jolt writes %08X)
    pub fn readIdentifier(self: *ObjectStreamTextIn, allocator: Allocator) Error!Identifier {
        const value = parseIdentifier(try self.readWord(allocator)) orelse return error.InvalidObjectStream;
        return @truncate(value);
    }

    pub fn readCount(self: *ObjectStreamTextIn, allocator: Allocator) Error!u32 {
        var count: u32 = undefined;
        try self.readPrimitive(allocator, u32, &count);
        return count;
    }

    pub fn readPrimitive(self: *ObjectStreamTextIn, allocator: Allocator, comptime T: type, value: *T) Error!void {
        switch (T) {
            // uint8 and uint16 are read as uint32 and truncated
            u8, u16 => value.* = @truncate(try parseUnsigned(u32, try self.readWord(allocator))),
            u32, u64 => value.* = try parseUnsigned(T, try self.readWord(allocator)),
            i32 => value.* = try parseInt(try self.readWord(allocator)),
            f32, f64 => value.* = try parseFloat(T, try self.readWord(allocator)),
            bool => {
                const token = try self.readWord(allocator);
                if (std.ascii.eqlIgnoreCase(token, "true")) value.* = true else if (std.ascii.eqlIgnoreCase(token, "false")) value.* = false else return error.InvalidObjectStream;
            },
            Float3 => value.* = .init(try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32)),
            Double3 => value.* = .init(try self.read(allocator, f64), try self.read(allocator, f64), try self.read(allocator, f64)),
            Float4 => value.* = .init(try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32)),
            Vec3 => value.* = .init(try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32)),
            DVec3 => value.* = .init(try self.read(allocator, f64), try self.read(allocator, f64), try self.read(allocator, f64)),
            Vec4 => value.* = .init(try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32)),
            UVec4 => value.* = .init(try self.read(allocator, u32), try self.read(allocator, u32), try self.read(allocator, u32), try self.read(allocator, u32)),
            Quat => value.* = .init(try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32), try self.read(allocator, f32)),
            Mat44 => value.* = .init(try self.read(allocator, Vec4), try self.read(allocator, Vec4), try self.read(allocator, Vec4), try self.read(allocator, Vec4)),
            DMat44 => value.* = .init(try self.read(allocator, Vec4), try self.read(allocator, Vec4), try self.read(allocator, Vec4), try self.read(allocator, DVec3)),
            else => @compileError("not a primitive: " ++ @typeName(T)),
        }
    }

    fn read(self: *ObjectStreamTextIn, allocator: Allocator, comptime T: type) Error!T {
        var value: T = undefined;
        try self.readPrimitive(allocator, T, &value);
        return value;
    }

    /// A string in double quotes (ReadPrimitiveData(String &)), owned by `allocator`: raw newlines and tabs are dropped,
    /// `\\`, `\n`, `\t` and `\"` are decoded, any other escaped character is dropped
    pub fn readString(self: *ObjectStreamTextIn, allocator: Allocator) Error![]u8 {
        // Skip whitespace
        const first = while (true) {
            const c = try self.readChar() orelse return error.InvalidObjectStream;
            if (!std.ascii.isWhitespace(c)) break c;
        };

        // Check if it is a opening quote
        if (first != '"') return error.InvalidObjectStream;

        // Read string and interpret special characters
        var result: std.ArrayList(u8) = .empty;
        errdefer result.deinit(allocator);
        var escaped = false;
        while (true) {
            const c = try self.readChar() orelse return error.InvalidObjectStream;
            switch (c) {
                '\n', '\t' => {},
                '\\' => if (escaped) {
                    try result.append(allocator, '\\');
                    escaped = false;
                } else {
                    escaped = true;
                },
                'n', 't' => if (escaped) {
                    try result.append(allocator, if (c == 'n') '\n' else '\t');
                    escaped = false;
                } else try result.append(allocator, c),
                '"' => if (escaped) {
                    try result.append(allocator, '"');
                    escaped = false;
                } else {
                    // Found closing double quote
                    return result.toOwnedSlice(allocator);
                },
                else => if (escaped) {
                    escaped = false;
                } else try result.append(allocator, c),
            }
        }
    }
};

/// The characters of an integer that num_get's stage 2 collects (base 10): an optional sign, then decimal digits
fn integerPrefix(word: []const u8) []const u8 {
    var i: usize = 0;
    if (i < word.len and (word[i] == '+' or word[i] == '-')) i += 1;
    while (i < word.len and std.ascii.isDigit(word[i])) i += 1;
    return word[0..i];
}

/// The digits after an optional '+' as a u64 (strtoull on what stage 2 collected), null without digits or on overflow
fn parseDigits(digits_in: []const u8) ?u64 {
    const digits = if (digits_in.len > 0 and digits_in[0] == '+') digits_in[1..] else digits_in;
    if (digits.len == 0) return null;
    var value: u64 = 0;
    for (digits) |c| {
        value = std.math.mul(u64, value, 10) catch return null;
        value = std.math.add(u64, value, c - '0') catch return null;
    }
    return value;
}

/// `istringstream >> T` for an unsigned integer (libc++ __num_get_unsigned_integral)
fn parseUnsigned(comptime T: type, word: []const u8) Error!T {
    const collected = integerPrefix(word);
    const negate = collected.len > 0 and collected[0] == '-';
    const value = parseDigits(if (negate) collected[1..] else collected) orelse return error.InvalidObjectStream;
    if (value > std.math.maxInt(T)) return error.InvalidObjectStream;
    const result: T = @intCast(value);
    return if (negate) 0 -% result else result;
}

/// `istringstream >> int` (read as a long, then range checked)
fn parseInt(word: []const u8) Error!i32 {
    const collected = integerPrefix(word);
    const negative = collected.len > 0 and collected[0] == '-';
    const magnitude = parseDigits(if (negative) collected[1..] else collected) orelse return error.InvalidObjectStream;
    if (negative) {
        if (magnitude > 1 << 31) return error.InvalidObjectStream;
        return @intCast(-@as(i64, @intCast(magnitude)));
    }
    if (magnitude > std.math.maxInt(i32)) return error.InvalidObjectStream;
    return @intCast(magnitude);
}

/// The characters of a float that num_get's stage 2 collects (libc++ __stage2_float_loop)
fn floatPrefix(word: []const u8) []const u8 {
    const atoms = "0123456789abcdefABCDEFxX+-pP";
    var in_units = true;
    var exp: u8 = 'E';
    var i: usize = 0;
    while (i < word.len) : (i += 1) {
        const c = word[i];
        if (c == '.') {
            if (!in_units) break;
            in_units = false;
            continue;
        }
        if (std.mem.indexOfScalar(u8, atoms, c) == null) break;
        if (c == '-' or c == '+') {
            if (i == 0 or std.ascii.toUpper(word[i - 1]) == std.ascii.toUpper(exp)) continue;
            break;
        }
        if (c == 'x' or c == 'X') {
            exp = 'P';
        } else if (std.ascii.toUpper(c) == exp) {
            exp = std.ascii.toLower(exp);
            in_units = false;
        }
    }
    return word[0..i];
}

/// `istringstream >> T` for a float (libc++ __num_get_float: strtof / strtod must use every collected character, and
/// fail on ERANGE)
fn parseFloat(comptime T: type, word: []const u8) Error!T {
    const collected = floatPrefix(word);
    // Zig's parseFloat accepts the same decimal and hexadecimal numbers as strtof for this alphabet (no '_', 'i', 'n')
    const value = std.fmt.parseFloat(T, collected) catch return error.InvalidObjectStream;

    // ERANGE: overflow to infinity, underflow to a subnormal or to zero
    if (std.math.isInf(value)) return error.InvalidObjectStream;
    if (value != 0 and @abs(value) < std.math.floatMin(T)) return error.InvalidObjectStream;
    if (value == 0 and hasNonzeroMantissa(collected)) return error.InvalidObjectStream;
    return value;
}

/// True when the mantissa of a number (the part before the exponent) has a nonzero digit
fn hasNonzeroMantissa(number: []const u8) bool {
    const hex = std.mem.indexOfAny(u8, number, "xX") != null;
    const start = if (hex) std.mem.indexOfAny(u8, number, "xX").? + 1 else 0;
    for (number[start..]) |c| {
        if (hex and (c == 'p' or c == 'P')) break;
        if (!hex and (c == 'e' or c == 'E')) break;
        if (std.ascii.isHex(c) and c != '0') return true;
    }
    return false;
}

/// `strtoul(word, nullptr, 16)` (64 bit unsigned long): an optional sign and "0x", then hexadecimal digits up to the
/// first other character; null on overflow (ERANGE)
fn parseIdentifier(word: []const u8) ?u64 {
    var i: usize = 0;
    var negate = false;
    if (i < word.len and (word[i] == '+' or word[i] == '-')) {
        negate = word[i] == '-';
        i += 1;
    }
    // A "0x" prefix (without a hexadecimal digit after it the value is the 0 before the 'x')
    if (i + 2 < word.len and word[i] == '0' and (word[i + 1] == 'x' or word[i + 1] == 'X') and std.ascii.isHex(word[i + 2]))
        i += 2;
    var value: u64 = 0;
    while (i < word.len and std.ascii.isHex(word[i])) : (i += 1) {
        const digit = std.fmt.charToDigit(word[i], 16) catch unreachable;
        value = std.math.mul(u64, value, 16) catch return null;
        value = std.math.add(u64, value, digit) catch return null;
    }
    return if (negate) 0 -% value else value;
}

test "ObjectStreamTextIn: number words like the reference platform (libc++ istream)" {
    const testing = std.testing;
    // Integers: a prefix is enough, unsigned values are negated, out of range fails
    try testing.expectEqual(@as(u32, 12), try parseUnsigned(u32, "12abc"));
    try testing.expectEqual(@as(u32, 4294967295), try parseUnsigned(u32, "-1"));
    try testing.expectEqual(@as(u32, 1), try parseUnsigned(u32, "-4294967295"));
    try testing.expectEqual(@as(u32, 0), try parseUnsigned(u32, "0x10"));
    try testing.expectEqual(@as(u32, 5), try parseUnsigned(u32, "+5"));
    try testing.expectError(error.InvalidObjectStream, parseUnsigned(u32, "4294967296"));
    try testing.expectError(error.InvalidObjectStream, parseUnsigned(u32, "-"));
    try testing.expectError(error.InvalidObjectStream, parseUnsigned(u32, "+-1"));
    try testing.expectError(error.InvalidObjectStream, parseUnsigned(u32, "abc"));
    try testing.expectEqual(@as(u64, 0xffffffffffffffff), try parseUnsigned(u64, "-1"));
    try testing.expectEqual(@as(i32, -2147483648), try parseInt("-2147483648"));
    try testing.expectEqual(@as(i32, 7), try parseInt("7.9"));
    try testing.expectError(error.InvalidObjectStream, parseInt("2147483648"));

    // Floats: the whole stage 2 prefix must be a number, rounded once, no ERANGE
    try testing.expectEqual(@as(f32, 0.5), try parseFloat(f32, ".5"));
    try testing.expectEqual(@as(f32, 5), try parseFloat(f32, "5."));
    try testing.expectEqual(@as(f32, 2.5), try parseFloat(f32, "+2.5"));
    try testing.expectEqual(@as(f32, 1.2), try parseFloat(f32, "1.2.3"));
    try testing.expectEqual(@as(f32, 16), try parseFloat(f32, "0x10"));
    try testing.expectEqual(@as(f32, 3), try parseFloat(f32, "3zz"));
    try testing.expect(std.math.signbit(try parseFloat(f32, "-0")));
    try testing.expect(std.math.signbit(try parseFloat(f32, "-0.000000"))); // dead_pose1.tof
    try testing.expectEqual(@as(u32, 0x7f7fffff), @as(u32, @bitCast(try parseFloat(f32, "3.40282347e+38"))));
    try testing.expectEqual(@as(u32, 0x800000), @as(u32, @bitCast(try parseFloat(f32, "1.17549435e-38"))));
    try testing.expectEqual(@as(f64, 2.2204460492503131e-16), try parseFloat(f64, "2.2204460492503131e-16"));
    for ([_][]const u8{ "1.5abc", "1.40129846e-45", "1e-40", "1.17549421e-38", "1e39", "3.40282357e+38", "inf", "nan", "-nan", "1e", "1e+", "--1", "." }) |w|
        try testing.expectError(error.InvalidObjectStream, parseFloat(f32, w));
    try testing.expectError(error.InvalidObjectStream, parseFloat(f64, "4.9406564584124654e-324"));
    try testing.expectError(error.InvalidObjectStream, parseFloat(f64, "1e309"));

    // Identifiers: strtoul(word, 16)
    try testing.expectEqual(@as(?u64, 0x1a), parseIdentifier("0000001A"));
    try testing.expectEqual(@as(?u64, 0x100000001), parseIdentifier("100000001")); // Truncated to 00000001 by the caller
    try testing.expectEqual(@as(?u64, 0), parseIdentifier("zz"));
    try testing.expectEqual(@as(?u64, 0x10), parseIdentifier("0x10"));
    try testing.expectEqual(@as(?u64, 0xffffffffffffffff), parseIdentifier("-1"));
    try testing.expectEqual(@as(?u64, null), parseIdentifier("10000000000000000"));
}
