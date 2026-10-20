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
/// - floats like printf("%g") (std::ostream's default precision of 6 significant digits, see writeFloatPrecision), f32
///   is converted to f64 first like the C++ stream does
/// - strings (`[]const u8`, string literals) as is
/// - types with a `format` method (the Zolt version of `operator <<`) through `{f}`, so they print whatever that
///   method writes. The Math types (Vec3, Vec4, Mat44, ...) print their components like their C++ `operator <<`
///   (writeFloatGeneral, e.g. "1e+10, 1.23457e+08, 0.1").
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

// ---------------------------------------------------------------------------------------------------------------------
// The C++ compatible float formatter (a Zolt addition, SimulationArchitecture section 7: text equality for M5)
//
// `std::ostream << value` prints a float or a double (libstdc++ and libc++ convert a float to double first) with the C
// library's printf("%.*g") and the stream's precision, 6 by default:
// - the value is rounded to `precision` significant digits, exactly and with ties to even (glibc and musl in the default
//   rounding mode);
// - when the decimal exponent X of the rounded value is in [-4, precision) it is printed in fixed notation with
//   precision - 1 - X digits after the decimal point, otherwise in scientific notation (d.ddddde+XX, at least two
//   exponent digits);
// - trailing zeros after the decimal point are removed, and the decimal point when no digit follows it;
// - -0 prints "-0", the infinities "inf" / "-inf" and NaN "nan" / "-nan" (by its sign bit, like glibc and musl).
// ZoltParity/Core/CoreParity.zig compares it with the C library's snprintf("%.*g") (every float and double exponent,
// random bit patterns, the rounding boundaries of every decimal exponent, every precision) and with std::ostream.

/// Number of significant digits that std::ostream uses for floating point numbers by default (its precision)
pub const default_float_precision = 6;

/// The largest precision that `writeFloatPrecision` supports (17 significant digits identify every double)
pub const max_float_precision = 17;

/// Integer type that holds the numerator and the denominator of any finite double scaled to at most
/// `max_float_precision` digits before the decimal point (see roundToSignificantDigits: the numerator is below
/// 2^53 * 10^342 < 2^1190 or 2^1024, the denominator below 2^1074 or 10^309)
const BigInt = u1280;

/// 10^n for n in [0, 32)
const small_powers_of_10: [32]u128 = blk: {
    var table: [32]u128 = undefined;
    var p: u128 = 1;
    for (&table) |*entry| {
        entry.* = p;
        p *%= 10;
    }
    break :blk table;
};

/// 10^(32 * n) for n in [0, 12) (roundToSignificantDigits needs at most 10^342)
const large_powers_of_10: [12]BigInt = blk: {
    var table: [12]BigInt = undefined;
    var p: BigInt = 1;
    for (&table) |*entry| {
        entry.* = p;
        p *= small_powers_of_10[31] * 10; // 10^32
    }
    break :blk table;
};

/// 10^n as a BigInt (n < 384)
fn powerOf10(n: u32) BigInt {
    return large_powers_of_10[n / 32] * small_powers_of_10[n % 32];
}

/// A positive finite double rounded to `precision` significant digits: `digits * 10^(exponent - precision + 1)`
const RoundedDecimal = struct {
    /// The significant digits, in [10^(precision - 1), 10^precision)
    digits: u64,
    /// Decimal exponent of the first digit
    exponent: i32,
};

/// Round a positive finite double to `precision` significant digits (1 to max_float_precision).
/// This uses exact integer arithmetic and rounds ties to even, like the C library's printf (in the default rounding
/// mode).
fn roundToSignificantDigits(value: f64, precision: u32) RoundedDecimal {
    std.debug.assert(value > 0 and std.math.isFinite(value));
    std.debug.assert(precision >= 1 and precision <= max_float_precision);

    // value = mantissa * 2^binary_exponent
    const bits: u64 = @bitCast(value);
    const biased_exponent: i32 = @intCast(bits >> 52);
    const fraction = bits & ((@as(u64, 1) << 52) - 1);
    const mantissa: u64 = if (biased_exponent == 0) fraction else fraction | (@as(u64, 1) << 52);
    const binary_exponent: i32 = (if (biased_exponent == 0) 1 else biased_exponent) - 1075;

    // Estimate the decimal exponent from the binary exponent (log10(2) ~ 1233 / 4096), corrected below
    const log2_value: i32 = binary_exponent + 63 - @as(i32, @clz(mantissa));
    var exponent: i32 = @divFloor(log2_value * 1233, 4096);

    const lower_bound: u64 = @intCast(small_powers_of_10[precision - 1]);
    const upper_bound: u64 = @intCast(small_powers_of_10[precision]);
    while (true) {
        // Compute value * 10^scale as numerator / denominator
        const scale: i32 = @as(i32, @intCast(precision)) - 1 - exponent;
        var numerator: BigInt = mantissa;
        var denominator: BigInt = 1;
        if (binary_exponent >= 0)
            numerator <<= @intCast(binary_exponent)
        else
            denominator <<= @intCast(-binary_exponent);
        if (scale >= 0)
            numerator *= powerOf10(@intCast(scale))
        else
            denominator *= powerOf10(@intCast(-scale));

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
        const twice_remainder = (numerator - quotient * denominator) * 2;
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
/// This is how `std::ostream << value` prints a float or a double (pass a float converted to f64), the `format` methods
/// of the Math types use it to print like their C++ `operator <<`.
pub fn writeFloatGeneral(writer: *Writer, value: f64) Writer.Error!void {
    return writeFloatPrecision(writer, value, default_float_precision);
}

/// Write a double like printf("%.*g", precision, value) does, i.e. like a std::ostream with `precision`
/// (std::setprecision) and the default float field. A precision of 0 is treated as 1 (like printf), at most
/// `max_float_precision`.
pub fn writeFloatPrecision(writer: *Writer, value: f64, precision_in: u32) Writer.Error!void {
    std.debug.assert(precision_in <= max_float_precision);
    const precision: u32 = if (precision_in == 0) 1 else precision_in;

    if (std.math.isNan(value))
        return writer.writeAll(if (std.math.signbit(value)) "-nan" else "nan");
    if (std.math.signbit(value))
        try writer.writeByte('-');
    const abs_value = @abs(value);
    if (std.math.isInf(abs_value))
        return writer.writeAll("inf");
    if (abs_value == 0)
        return writer.writeAll("0");

    const rounded = roundToSignificantDigits(abs_value, precision);
    var digits_buffer: [max_float_precision]u8 = undefined;
    const digits = digits_buffer[0..precision];
    _ = std.fmt.printInt(digits, rounded.digits, 10, .lower, .{ .width = precision, .fill = '0' });

    // Number of digits without trailing zeros
    var num_digits: usize = precision;
    while (num_digits > 1 and digits[num_digits - 1] == '0')
        num_digits -= 1;

    const exponent = rounded.exponent;
    if (exponent < -4 or exponent >= precision) {
        // Scientific notation: d.ddddde+XX (at least 2 exponent digits)
        try writer.writeByte(digits[0]);
        if (num_digits > 1) {
            try writer.writeByte('.');
            try writer.writeAll(digits[1..num_digits]);
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
        try writer.writeAll(digits[0..num_integer_digits]);
        if (num_digits > num_integer_digits) {
            try writer.writeByte('.');
            try writer.writeAll(digits[num_integer_digits..num_digits]);
        }
    } else {
        // Fixed notation with leading zeros after the decimal point
        try writer.writeAll("0.");
        try writer.splatByteAll('0', @intCast(-exponent - 1));
        try writer.writeAll(digits[0..num_digits]);
    }
}

/// A float that prints like `std::ostream << value` (see writeFloatPrecision), for `{f}` in a format string:
/// `writer.print("x = {f}", .{cppFloat(x)})` is `cout << "x = " << x`
pub const CppFloat = struct {
    value: f64,
    /// The precision of the stream (std::setprecision)
    precision: u32 = default_float_precision,

    pub fn format(self: CppFloat, writer: *Writer) Writer.Error!void {
        return writeFloatPrecision(writer, self.value, self.precision);
    }
};

/// Wrap a float for `{f}` with std::ostream's default precision (an f32 is converted to f64 like the C++ stream does)
pub fn cppFloat(value: anytype) CppFloat {
    return .{ .value = switch (@typeInfo(@TypeOf(value))) {
        .float => @floatCast(value),
        .comptime_float => value,
        else => @compileError("cppFloat: expected a float, got " ++ @typeName(@TypeOf(value))),
    } };
}

// ---------------------------------------------------------------------------------------------------------------------
// The C++ compatible fixed notation formatter (a Zolt addition, RemainingFeaturesArchitecture.md R7: the draw texts of
// the debug renderer)
//
// Jolt's draw texts use printf with "%.0f", "%.1f", "%.2f" and "%.3f" (a float argument is converted to double). glibc
// and musl print the exact decimal expansion of the double:
// - the value is rounded to `decimals` digits after the decimal point, exactly and with ties to even (in the default
//   rounding mode): printf("%.0f", 2.5) is "2" and printf("%.2f", 0.125) is "0.12";
// - the integer part has all its digits ("0" when it is 0), there is no exponent;
// - the decimal point is left out when `decimals` is 0;
// - the sign comes from the sign bit, also when the rounded value is 0 ("-0.00" for -0.001 and -0);
// - the infinities print "inf" / "-inf" and NaN "nan" / "-nan" (by its sign bit, like glibc and musl).
// std.fmt rounds differently on ties, so draw sites use this instead. ZoltParity/Core/CoreParity.zig compares it with the
// C library's snprintf("%.*f").

/// The largest number of decimals that `writeFloatFixed` supports (BigInt holds every double times 10^decimals)
pub const max_fixed_decimals = 60;

/// Write a double like printf("%.*f", decimals, value) does (see above). Use it where Jolt formats a number with
/// "%.<decimals>f".
pub fn writeFloatFixed(writer: *Writer, value: f64, decimals: u32) Writer.Error!void {
    std.debug.assert(decimals <= max_fixed_decimals);

    if (std.math.isNan(value))
        return writer.writeAll(if (std.math.signbit(value)) "-nan" else "nan");
    if (std.math.signbit(value))
        try writer.writeByte('-');
    const abs_value = @abs(value);
    if (std.math.isInf(abs_value))
        return writer.writeAll("inf");

    // value = mantissa * 2^binary_exponent
    const bits: u64 = @bitCast(abs_value);
    const biased_exponent: i32 = @intCast(bits >> 52);
    const fraction = bits & ((@as(u64, 1) << 52) - 1);
    const mantissa: u64 = if (biased_exponent == 0) fraction else fraction | (@as(u64, 1) << 52);
    const binary_exponent: i32 = (if (biased_exponent == 0) 1 else biased_exponent) - 1075;

    // value * 10^decimals rounded to an integer, exact and with ties to even
    const scaled: BigInt = @as(BigInt, mantissa) * powerOf10(decimals);
    var rounded: BigInt = undefined;
    if (binary_exponent >= 0) {
        rounded = scaled << @intCast(binary_exponent);
    } else {
        const shift: std.math.Log2Int(BigInt) = @intCast(-binary_exponent);
        rounded = scaled >> shift;
        const remainder = scaled - (rounded << shift);
        const half = @as(BigInt, 1) << (shift - 1);
        if (remainder > half or (remainder == half and (rounded & 1) != 0))
            rounded += 1;
    }

    // The decimal digits of the rounded value (a double below 2^1024 has at most 309 digits before the decimal point)
    var digits_buffer: [309 + max_fixed_decimals + 1]u8 = undefined;
    const digits = digits_buffer[0..std.fmt.printInt(&digits_buffer, rounded, 10, .lower, .{})];

    if (digits.len > decimals) {
        // Integer part, then the decimals
        const num_integer_digits = digits.len - decimals;
        try writer.writeAll(digits[0..num_integer_digits]);
        if (decimals > 0) {
            try writer.writeByte('.');
            try writer.writeAll(digits[num_integer_digits..]);
        }
    } else {
        // Integer part 0, leading zeros after the decimal point
        try writer.writeAll("0.");
        try writer.splatByteAll('0', decimals - digits.len);
        try writer.writeAll(digits);
    }
}

/// A float that prints like printf("%.*f", decimals, value) (see writeFloatFixed), for `{f}` in a format string:
/// `stringFormat(allocator, "{f}", .{cppFixed(value, 2)})` is Jolt's `StringFormat("%.2f", value)`
pub const CppFixed = struct {
    value: f64,
    /// The number of digits after the decimal point (the precision of "%.*f")
    decimals: u32,

    pub fn format(self: CppFixed, writer: *Writer) Writer.Error!void {
        return writeFloatFixed(writer, self.value, self.decimals);
    }
};

/// Wrap a float for `{f}` in fixed notation with `decimals` digits after the decimal point (an f32 is converted to f64
/// like a printf argument)
pub fn cppFixed(value: anytype, decimals: u32) CppFixed {
    return .{ .value = switch (@typeInfo(@TypeOf(value))) {
        .float => @floatCast(value),
        .comptime_float => value,
        else => @compileError("cppFixed: expected a float, got " ++ @typeName(@TypeOf(value))),
    }, .decimals = decimals };
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
    try expectConverted("-nan", -std.math.nan(f64));

    // Rounding boundaries (checked against glibc printf, the exhaustive comparison is in ZoltParity/Core/CoreParity.zig)
    try expectConverted("10", @as(f64, 9.999995)); // Just above the tie (9.99999500000000016...), carries into the exponent
    try expectConverted("9.99999", @as(f64, 9.999994999999999));
    try expectConverted("123456", @as(f64, 123456.5)); // Tie, rounds to even
    try expectConverted("123458", @as(f64, 123457.5)); // Tie, rounds to even
    try expectConverted("1.23457e+06", @as(f64, 1234567.0));
    try expectConverted("0.0001", @as(f64, 0.0001)); // Fixed notation down to exponent -4
    try expectConverted("9.99999e-05", @as(f64, 0.0000999999));
    try expectConverted("0.0001", @as(f64, 0.00009999995)); // Rounds up into exponent -4 (the rounded exponent decides)
    try expectConverted("1e-05", @as(f64, 1.0e-5));
    try expectConverted("999999", @as(f64, 999999.0));
    try expectConverted("1.4013e-45", @as(f32, 1.0e-45)); // Smallest float denormal (1.40129846e-45) converted to double
    try expectConverted("0.1", @as(f32, 0.1)); // 0.100000001490116... as a double
    try expectConverted("3.40282e+38", std.math.floatMax(f32));
}

fn expectFormatted(expected: []const u8, value: f64, precision: u32) !void {
    var buffer: [64]u8 = undefined;
    const s = try std.fmt.bufPrint(&buffer, "{f}", .{CppFloat{ .value = value, .precision = precision }});
    try std.testing.expectEqualStrings(expected, s);
}

test "writeFloatPrecision formats like printf(\"%.*g\")" {
    // Values checked against glibc printf
    try expectFormatted("0.1", 0.15, 1); // 0.1499999999999999944...: rounds the exact value, not the shortest one
    try expectFormatted("0.2", 0.25, 1); // Tie, rounds to even
    try expectFormatted("0.3", 0.35, 1); // 0.34999999999999997779...
    try expectFormatted("2", 2.5, 0); // Precision 0 is 1, tie rounds to even
    try expectFormatted("4", 3.5, 1);
    try expectFormatted("1e+01", 9.5, 1);
    try expectFormatted("0.30000000000000004", @as(f64, 0.1) + @as(f64, 0.2), 17); // f64 addition (comptime_float 0.1 + 0.2 is 0.3)
    try expectFormatted("0.10000000000000001", 0.1, 17);
    try expectFormatted("1.7976931348623157e+308", std.math.floatMax(f64), 17);
    try expectFormatted("4.9406564584124654e-324", 4.9406564584124654e-324, 17);
    try expectFormatted("1.2345678901230001e+20", 123456789012300000000.0, 17);
    try expectFormatted("100000", 100000.0, 6);
    try expectFormatted("1e+05", 100000.0, 5);
    try expectFormatted("12345.7", 12345.67, 6);
    try expectFormatted("-0", -0.0, 3);
    try expectFormatted("-inf", -std.math.inf(f64), 17);

    // cppFloat converts f32 to f64
    var buffer: [32]u8 = undefined;
    try std.testing.expectEqualStrings("1.1, -2e-07", try std.fmt.bufPrint(&buffer, "{f}, {f}", .{ cppFloat(@as(f32, 1.1)), cppFloat(-2.0e-7) }));

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

fn expectFixed(expected: []const u8, value: f64, decimals: u32) !void {
    var buffer: [512]u8 = undefined;
    const s = try std.fmt.bufPrint(&buffer, "{f}", .{CppFixed{ .value = value, .decimals = decimals }});
    try std.testing.expectEqualStrings(expected, s);
}

test "writeFloatFixed formats like printf(\"%.*f\")" {
    // Values checked against glibc printf
    try expectFixed("0", 0.0, 0);
    try expectFixed("-0", -0.0, 0);
    try expectFixed("0.000", 0.0, 3);
    try expectFixed("-0.00", -0.0, 2);
    try expectFixed("-0.00", -0.001, 2); // Rounds to 0, keeps the sign
    try expectFixed("2", 2.5, 0); // Tie, rounds to even
    try expectFixed("4", 3.5, 0);
    try expectFixed("0", 0.5, 0);
    try expectFixed("2", 1.5, 0);
    try expectFixed("0.12", 0.125, 2); // Tie, rounds to even
    try expectFixed("0.38", 0.375, 2);
    try expectFixed("0.1", 0.15, 1); // 0.1499999999999999944...: rounds the exact value
    try expectFixed("0.2", 0.25, 1); // Exact tie, rounds to even
    try expectFixed("1.000", 0.9995, 3); // 0.99950000000000005507...: just above the tie
    try expectFixed("0.999", 0.99949999999999994, 3);
    try expectFixed("123456789.000", 123456789.0, 3);
    try expectFixed("100000002537764290115403776.0", @as(f32, 1.0e26), 1); // The exact value of the float
    try expectFixed("1000000000000000019884624838656.00", 1.0e30, 2);
    try expectFixed("0.000", 1.0e-30, 3);
    try expectFixed("0.001", 0.0005000000000000001, 3);
    try expectFixed("0.001", 0.0005, 3); // 0.000500000000000000010408...: just above the tie
    try expectFixed("-9.81", @as(f32, -9.81), 2);
    try expectFixed("inf", std.math.inf(f64), 2);
    try expectFixed("-inf", -std.math.inf(f64), 0);
    try expectFixed("nan", std.math.nan(f64), 1);
    try expectFixed("-nan", -std.math.nan(f64), 3);

    // cppFixed converts f32 to f64
    var buffer: [64]u8 = undefined;
    try std.testing.expectEqualStrings("RPM: 1000, 0.1", try std.fmt.bufPrint(&buffer, "RPM: {f}, {f}", .{ cppFixed(@as(f32, 999.5), 0), cppFixed(@as(f32, 0.1), 1) }));
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
