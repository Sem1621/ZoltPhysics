//! Port of: Jolt/Core/StringTools.h, Jolt/Core/StringTools.cpp
//! Status: complete
//!
//! Jolt's `String` is an owned `[]u8` allocated with the `allocator` passed to the function, `string_view`
//! is `[]const u8` and `Array<String>` is `std.ArrayList([]u8)` whose elements are owned by the same allocator.
//! `StringFormat` takes a Zig format string (`std.fmt` syntax) instead of a printf format string.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Writer = std.Io.Writer;

/// Size of the internal buffer of `stringFormat`, including the terminating zero of the C++ version
const string_format_buffer_size = 1024;

/// Create a formatted text string for debugging purposes.
/// Note that this function has an internal buffer of 1024 characters, so long strings will be trimmed.
/// `fmt` uses the `std.fmt` syntax (e.g. "Test: {d}"). Caller owns the returned memory.
pub fn stringFormat(allocator: Allocator, comptime fmt: []const u8, args: anytype) Allocator.Error![]u8 {
    // vsnprintf writes at most sizeof(buffer) - 1 characters followed by a zero
    var buffer: [string_format_buffer_size - 1]u8 = undefined;

    // Format the string. A fixed writer fills the buffer completely before it fails, so a failure means
    // that the string was trimmed to the size of the buffer.
    var writer: Writer = .fixed(&buffer);
    writer.print(fmt, args) catch {};

    return allocator.dupe(u8, writer.buffered());
}

/// Convert type to string (ConvertToString), formats `value` like `std::ostringstream << value`:
/// - integers in decimal, bools as 1 / 0, enums as their integer value
/// - floats like printf("%g") (std::ostream's default precision of 6 significant digits), f32 is converted
///   to f64 first like the C++ stream does
/// - strings (`[]const u8`, string literals) as is
/// - types with a `format` method (the Zolt version of `operator <<`) through `{f}`
///
/// Unlike C++ (where uint8 is an unsigned char), u8 / i8 are printed as numbers. Caller owns the returned memory.
pub fn convertToString(allocator: Allocator, value: anytype) Allocator.Error![]u8 {
    var out: Writer.Allocating = .init(allocator);
    defer out.deinit();
    writeValue(&out.writer, value) catch return error.OutOfMemory; // The allocating writer only fails when out of memory
    return out.toOwnedSlice();
}

/// Writes `value` like `std::ostream << value` (see convertToString)
fn writeValue(writer: *Writer, value: anytype) Writer.Error!void {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .int, .comptime_int => return writer.print("{d}", .{value}),
        .float => return writeFloatGeneral(writer, @floatCast(value)),
        .comptime_float => return writeFloatGeneral(writer, @as(f64, value)),
        .bool => return writer.writeAll(if (value) "1" else "0"),
        .pointer => |ptr| {
            if (ptr.size == .slice and ptr.child == u8) return writer.writeAll(value);
            if (ptr.size == .one) switch (@typeInfo(ptr.child)) {
                // String literal, e.g. *const [5:0]u8
                .array => |arr| if (arr.child == u8) return writer.writeAll(value),
                else => {},
            };
            if (ptr.size == .many and ptr.child == u8 and ptr.sentinel() == @as(u8, 0)) return writer.writeAll(std.mem.span(value));
            @compileError("convertToString: unsupported pointer type " ++ @typeName(T));
        },
        .@"struct", .@"union", .@"enum" => {
            if (@hasDecl(T, "format")) return writer.print("{f}", .{value});
            if (@typeInfo(T) == .@"enum") return writer.print("{d}", .{@intFromEnum(value)});
            @compileError("convertToString: " ++ @typeName(T) ++ " has no format method");
        },
        else => @compileError("convertToString: unsupported type " ++ @typeName(T)),
    }
}

/// Number of significant digits that std::ostream uses for floating point numbers by default
const default_float_precision = 6;

/// Integer type that holds the numerator and denominator of any finite double scaled to `default_float_precision`
/// digits before the decimal point (at most 2^53 * 10^329 resp. 2^1074, see roundToSignificantDigits)
const BigInt = u1280;

/// A positive finite double rounded to `default_float_precision` significant digits: `digits * 10^(exponent - precision + 1)`
const RoundedDecimal = struct {
    /// The significant digits, in [10^(precision - 1), 10^precision)
    digits: u64,
    /// Decimal exponent of the first digit
    exponent: i32,
};

fn powerOf10(comptime T: type, n: u32) T {
    var result: T = 1;
    for (0..n) |_| result *= 10;
    return result;
}

/// Round a positive finite double to `default_float_precision` significant digits.
/// This uses exact integer arithmetic and rounds ties to even, like glibc's printf (in the default rounding mode).
fn roundToSignificantDigits(value: f64) RoundedDecimal {
    std.debug.assert(value > 0 and std.math.isFinite(value));
    const precision = default_float_precision;

    // value = mantissa * 2^binary_exponent
    const bits: u64 = @bitCast(value);
    const biased_exponent: i32 = @intCast(bits >> 52);
    const fraction = bits & ((@as(u64, 1) << 52) - 1);
    const mantissa: u64 = if (biased_exponent == 0) fraction else fraction | (@as(u64, 1) << 52);
    const binary_exponent: i32 = (if (biased_exponent == 0) 1 else biased_exponent) - 1075;

    // Estimate the decimal exponent from the binary exponent (log10(2) ~ 1233 / 4096), corrected below
    const log2_value: i32 = binary_exponent + 63 - @as(i32, @clz(mantissa));
    var exponent: i32 = @divFloor(log2_value * 1233, 4096);

    const lower_bound = powerOf10(u64, precision - 1);
    const upper_bound = powerOf10(u64, precision);
    while (true) {
        // Compute value * 10^scale as numerator / denominator
        const scale = precision - 1 - exponent;
        var numerator: BigInt = mantissa;
        var denominator: BigInt = 1;
        if (binary_exponent >= 0)
            numerator <<= @intCast(binary_exponent)
        else
            denominator <<= @intCast(-binary_exponent);
        if (scale >= 0)
            numerator *= powerOf10(BigInt, @intCast(scale))
        else
            denominator *= powerOf10(BigInt, @intCast(-scale));

        const quotient = numerator / denominator;
        if (quotient >= upper_bound) {
            exponent += 1;
            continue;
        }
        if (quotient < lower_bound) {
            exponent -= 1;
            continue;
        }

        // Round to nearest, ties to even
        var digits: u64 = @intCast(quotient);
        const twice_remainder = (numerator % denominator) * 2;
        if (twice_remainder > denominator or (twice_remainder == denominator and (digits & 1) != 0))
            digits += 1;
        if (digits == upper_bound) {
            digits = lower_bound;
            exponent += 1;
        }
        return .{ .digits = digits, .exponent = exponent };
    }
}

/// Write a double like printf("%g", value) does: `default_float_precision` significant digits, fixed notation
/// when the exponent is in [-4, precision), scientific notation otherwise, trailing zeros removed.
fn writeFloatGeneral(writer: *Writer, value: f64) Writer.Error!void {
    const precision = default_float_precision;

    if (std.math.isNan(value))
        return writer.writeAll(if (std.math.signbit(value)) "-nan" else "nan");
    if (std.math.signbit(value))
        try writer.writeByte('-');
    const abs_value = @abs(value);
    if (std.math.isInf(abs_value))
        return writer.writeAll("inf");
    if (abs_value == 0)
        return writer.writeAll("0");

    const rounded = roundToSignificantDigits(abs_value);
    var digits_buffer: [precision]u8 = undefined;
    _ = std.fmt.printInt(&digits_buffer, rounded.digits, 10, .lower, .{ .width = precision, .fill = '0' });

    // Number of digits without trailing zeros
    var num_digits: usize = precision;
    while (num_digits > 1 and digits_buffer[num_digits - 1] == '0')
        num_digits -= 1;

    const exponent = rounded.exponent;
    if (exponent < -4 or exponent >= precision) {
        // Scientific notation: d.ddddde+XX (at least 2 exponent digits)
        try writer.writeByte(digits_buffer[0]);
        if (num_digits > 1) {
            try writer.writeByte('.');
            try writer.writeAll(digits_buffer[1..num_digits]);
        }
        try writer.writeByte('e');
        try writer.writeByte(if (exponent < 0) '-' else '+');
        const abs_exponent = @abs(exponent);
        if (abs_exponent < 10)
            try writer.writeByte('0');
        try writer.print("{d}", .{abs_exponent});
    } else if (exponent >= 0) {
        // Fixed notation with exponent + 1 digits before the decimal point
        const num_integer_digits: usize = @intCast(exponent + 1);
        try writer.writeAll(digits_buffer[0..num_integer_digits]);
        if (num_digits > num_integer_digits) {
            try writer.writeByte('.');
            try writer.writeAll(digits_buffer[num_integer_digits..num_digits]);
        }
    } else {
        // Fixed notation with leading zeros after the decimal point
        try writer.writeAll("0.");
        try writer.splatByteAll('0', @intCast(-exponent - 1));
        try writer.writeAll(digits_buffer[0..num_digits]);
    }
}

/// Replace substring with other string. `string` is owned by `allocator` and is reallocated when its length changes.
pub fn stringReplace(allocator: Allocator, string: *[]u8, search: []const u8, replace: []const u8) Allocator.Error!void {
    // An empty search string would loop forever in the C++ version
    std.debug.assert(search.len > 0);

    var index: usize = 0;
    while (true) {
        index = std.mem.indexOfPos(u8, string.*, index, search) orelse break;

        // string.replace(index, search.size(), replace)
        if (search.len == replace.len) {
            @memcpy(string.*[index..][0..replace.len], replace);
        } else {
            const old = string.*;
            const new = try allocator.alloc(u8, old.len - search.len + replace.len);
            @memcpy(new[0..index], old[0..index]);
            @memcpy(new[index..][0..replace.len], replace);
            @memcpy(new[index + replace.len ..], old[index + search.len ..]);
            allocator.free(old);
            string.* = new;
        }

        index += replace.len;
    }
}

/// Free the strings in `vector` and clear it (Array<String>::clear), the capacity is retained
fn clearStringVector(allocator: Allocator, vector: *std.ArrayList([]u8)) void {
    for (vector.items) |s|
        allocator.free(s);
    vector.clearRetainingCapacity();
}

/// Convert a delimited string to an array of strings.
/// The strings that are added to `vector` are owned by `allocator` (like the strings that are already in it when `clear_vector` is true).
pub fn stringToVector(allocator: Allocator, string: []const u8, vector: *std.ArrayList([]u8), opts: struct { delimiter: []const u8 = ",", clear_vector: bool = true }) Allocator.Error!void {
    std.debug.assert(opts.delimiter.len > 0);

    // Ensure vector empty
    if (opts.clear_vector)
        clearStringVector(allocator, vector);

    // No string? no elements
    if (string.len == 0)
        return;

    // Start with initial string
    var s = string;

    // Add to vector while we have a delimiter
    while (s.len != 0) {
        const i = std.mem.indexOf(u8, s, opts.delimiter) orelse break;
        try appendString(allocator, vector, s[0..i]);
        s = s[i + opts.delimiter.len ..];
    }

    // Add final element
    try appendString(allocator, vector, s);
}

/// Append a copy of `string` to `vector`
fn appendString(allocator: Allocator, vector: *std.ArrayList([]u8), string: []const u8) Allocator.Error!void {
    const copy = try allocator.dupe(u8, string);
    errdefer allocator.free(copy);
    try vector.append(allocator, copy);
}

/// Convert an array strings to a delimited string. Caller owns the returned memory.
pub fn vectorToString(allocator: Allocator, vector: []const []const u8, opts: struct { delimiter: []const u8 = "," }) Allocator.Error![]u8 {
    var out: std.ArrayList(u8) = .empty;
    errdefer out.deinit(allocator);

    for (vector) |s| {
        // Add delimiter if not first element
        if (out.items.len != 0)
            try out.appendSlice(allocator, opts.delimiter);

        // Add element
        try out.appendSlice(allocator, s);
    }

    return out.toOwnedSlice(allocator);
}

/// Convert a string to lower case (with the "C" locale, like tolower). Caller owns the returned memory.
pub fn toLower(allocator: Allocator, string: []const u8) Allocator.Error![]u8 {
    const out = try allocator.alloc(u8, string.len);
    for (out, string) |*o, c|
        o.* = std.ascii.toLower(c);
    return out;
}

/// Converts the lower 4 bits of inNibble to a string that represents the number in binary format
pub fn nibbleToBinary(nibble: u32) [:0]const u8 {
    const nibbles = [_][:0]const u8{ "0000", "0001", "0010", "0011", "0100", "0101", "0110", "0111", "1000", "1001", "1010", "1011", "1100", "1101", "1110", "1111" };
    return nibbles[nibble & 0xf];
}

test "stringFormat trims to the size of the internal buffer" {
    const allocator = std.testing.allocator;
    const long = "x" ** 2000;
    const s = try stringFormat(allocator, "{s}", .{long});
    defer allocator.free(s);
    try std.testing.expectEqual(@as(usize, 1023), s.len);
    try std.testing.expectEqualStrings(long[0..1023], s);
}

fn expectConverted(expected: []const u8, value: anytype) !void {
    const s = try convertToString(std.testing.allocator, value);
    defer std.testing.allocator.free(s);
    try std.testing.expectEqualStrings(expected, s);
}

test "convertToString formats like std::ostream" {
    // Floats use %g with 6 significant digits (values checked against glibc printf)
    try expectConverted("0", @as(f32, 0.0));
    try expectConverted("-0", @as(f32, -0.0));
    try expectConverted("1.5", @as(f32, 1.5));
    try expectConverted("0.1", @as(f32, 0.1));
    try expectConverted("3.14159", @as(f64, 3.14159265358979));
    try expectConverted("1e+10", @as(f64, 1.0e10));
    try expectConverted("1.23457e+08", @as(f64, 123456789.0));
    try expectConverted("1e-05", @as(f64, 1.0e-5));
    try expectConverted("0.0001", @as(f64, 0.0001));
    try expectConverted("0.000123457", @as(f64, 0.000123456789));
    try expectConverted("100000", @as(f64, 100000.0));
    try expectConverted("1e+06", @as(f64, 1000000.0));
    try expectConverted("1.23456e+06", @as(f64, 1234565.0)); // Tie, rounds to even
    try expectConverted("1e+06", @as(f64, 999999.5)); // Tie, rounds to even and carries into the exponent
    try expectConverted("1e+100", @as(f64, 1.0e100));
    try expectConverted("4.94066e-324", @as(f64, 4.9406564584124654e-324));
    try expectConverted("1.79769e+308", @as(f64, std.math.floatMax(f64)));
    try expectConverted("-2.5", -2.5);
    try expectConverted("inf", std.math.inf(f32));
    try expectConverted("-inf", -std.math.inf(f64));
    try expectConverted("nan", std.math.nan(f64));

    // Other types
    try expectConverted("1", true);
    try expectConverted("0", false);
    try expectConverted("hello", "hello");
    try expectConverted("hello", @as([]const u8, "hello"));
    try expectConverted("hello", @as([*:0]const u8, "hello"));
    try expectConverted("200", @as(u8, 200));
    const E = enum(u8) { a = 3, b = 7 };
    try expectConverted("7", E.b);
    const Formattable = struct {
        v: i32,
        pub fn format(self: @This(), writer: *Writer) Writer.Error!void {
            try writer.print("<{d}>", .{self.v});
        }
    };
    try expectConverted("<5>", Formattable{ .v = 5 });
}

test "stringReplace edge cases" {
    const allocator = std.testing.allocator;
    var s = try allocator.dupe(u8, "aaa");
    defer allocator.free(s);

    // Replacement contains the search string: continues after the replacement
    try stringReplace(allocator, &s, "a", "aa");
    try std.testing.expectEqualStrings("aaaaaa", s);
    try stringReplace(allocator, &s, "aa", "b");
    try std.testing.expectEqualStrings("bbb", s);
    try stringReplace(allocator, &s, "b", "");
    try std.testing.expectEqualStrings("", s);
}

test "stringToVector / vectorToString edge cases" {
    const allocator = std.testing.allocator;
    var vector: std.ArrayList([]u8) = .empty;
    defer {
        clearStringVector(allocator, &vector);
        vector.deinit(allocator);
    }

    // Trailing delimiter gives an empty last element
    try stringToVector(allocator, "a,", &vector, .{});
    try std.testing.expectEqual(@as(usize, 2), vector.items.len);
    try std.testing.expectEqualStrings("a", vector.items[0]);
    try std.testing.expectEqualStrings("", vector.items[1]);

    // Append without clearing, multi character delimiter
    try stringToVector(allocator, "b::c", &vector, .{ .delimiter = "::", .clear_vector = false });
    try std.testing.expectEqual(@as(usize, 4), vector.items.len);
    try std.testing.expectEqualStrings("b", vector.items[2]);
    try std.testing.expectEqualStrings("c", vector.items[3]);

    // A delimiter is only added after a non empty prefix (like the C++ version)
    const s = try vectorToString(allocator, &.{ "", "a", "", "b" }, .{});
    defer allocator.free(s);
    try std.testing.expectEqualStrings("a,,b", s);

    // Owned strings can be passed directly
    const s2 = try vectorToString(allocator, vector.items, .{ .delimiter = "-" });
    defer allocator.free(s2);
    try std.testing.expectEqualStrings("a--b-c", s2);
}
