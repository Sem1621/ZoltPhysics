//! Port of: Jolt/Physics/DeterminismLog.h, Jolt/Physics/DeterminismLog.cpp
//! Status: complete
//!
//! SimulationArchitecture D11:
//! - Jolt's `JPH_ENABLE_DETERMINISM_LOG` (a commented out `#define`) is the build option `-Ddeterminism_log=true`,
//!   `DeterminismLog.enabled` (off by default). Without it nothing of the log exists at run time: `writer`,
//!   `base_path` and the stream state are `void`, and the call sites compile to nothing.
//! - `DeterminismLog::sLog` (a singleton `std::ofstream`) is the module level `writer`, because the `JPH_DET_LOG` call
//!   sites in MotionProperties have no system pointer. `open(io)` / `close()` are `JPH_DET_LOG_OPEN()` /
//!   `JPH_DET_LOG_CLOSE()` (the file `detlog.txt` in `base_path` or the current directory); an application may also
//!   point `writer` at any `std.Io.Writer`.
//! - `JPH_DET_LOG(a << b << c)` is `if (DeterminismLog.enabled) DeterminismLog.log(.{ a, b, c });`, which writes the
//!   values and a '\n'. The text is Jolt's, value by value (the `operator <<` overloads):
//!   - strings (`[]const u8`, string literals) switch the stream to decimal (`std::dec`) and are written as is;
//!   - `i32` is C++ `int`: written in the current base without padding (decimal after a string, hexadecimal after a
//!     hex value, a negative value then as its two's complement);
//!   - `u32` / `BodyID` (index and sequence number) / `SubShapeID` / `f32` (its bits) are 8 hex digits, `u64` 16 hex
//!     digits (lower case, padded with '0', `std::hex` stays set);
//!   - `Vec3` / `Float3` are 3 floats, `Vec4` and `Quat` (`GetXYZW`) 4 floats, `DVec3` 3 doubles of 16 hex digits,
//!     separated by a space; `Mat44` its 4 columns, `DMat44` 3 columns and the translation, separated by `" "`
//!     strings (which switch to decimal like Jolt's).
//!   Jolt has no overload for `double`, `bool` or `uint8`: they are compile errors here too.
//! - Like Jolt's log it is not thread safe and only meaningful with a single threaded job system.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const Float3 = @import("../Math/Float3.zig").Float3;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const DMat44 = @import("../Math/DMat44.zig").DMat44;
const Quat = @import("../Math/Quat.zig").Quat;
const BodyID = @import("Body/BodyID.zig").BodyID;
const SubShapeID = @import("Collision/Shape/SubShapeID.zig").SubShapeID;

const log_scope = std.log.scoped(.zolt);

/// JPH_ENABLE_DETERMINISM_LOG: the log exists (build option `-Ddeterminism_log=true`)
pub const enabled: bool = Core.determinism_log;

/// When base path is set, use it as the base path for the log file, otherwise use the current working directory
/// (DeterminismLog::sBasePath)
pub var base_path: if (enabled) ?[]const u8 else void = if (enabled) null else {};

/// Where the log goes (the stream of DeterminismLog::sLog): set by `open`, or by the application. Nothing is logged
/// while it is null. `void` when the log is disabled.
pub var writer: if (enabled) ?*std.Io.Writer else void = if (enabled) null else {};

/// The base flag of the stream (`std::hex` / `std::dec` are sticky in C++)
var state: if (enabled) Format else void = if (enabled) .{} else {};

/// The file that `open` created
const LogFile = struct {
    io: std.Io,
    file: std.Io.File,
    file_writer: std.Io.File.Writer,
    buffer: [4096]u8,
};

var log_file: if (enabled) ?LogFile else void = if (enabled) null else {};

/// Open the log file `detlog.txt` (JPH_DET_LOG_OPEN, DeterminismLog::Open): in `base_path` when it is set, otherwise
/// in the current working directory. It is opened in binary mode and truncated. Does nothing when the log is
/// disabled.
pub fn open(io: std.Io) std.Io.File.OpenError!void {
    if (enabled) {
        // Determine file name (Jolt: a char[1024] filled with snprintf, longer names are truncated)
        var file_name_buffer: [1024]u8 = undefined;
        const file_name: []const u8 = if (base_path) |path|
            std.fmt.bufPrint(&file_name_buffer, "{s}/detlog.txt", .{path}) catch file_name_buffer[0 .. file_name_buffer.len - 1] // snprintf truncates and keeps the terminator
        else
            "detlog.txt";
        log_scope.info("Opening determinism log file: {s}", .{file_name});

        // Open the file (binary because we don't want a difference between Unix and Windows line endings)
        close();
        const file = try std.Io.Dir.cwd().createFile(io, file_name, .{});
        log_file = .{ .io = io, .file = file, .file_writer = undefined, .buffer = undefined };
        const lf = &log_file.?;
        lf.file_writer = lf.file.writer(io, &lf.buffer);
        writer = &lf.file_writer.interface;
        state = .{};
    }
}

/// Close the log file (JPH_DET_LOG_CLOSE, DeterminismLog::Close). Does nothing when the log is disabled or `open`
/// was not called.
pub fn close() void {
    if (enabled) {
        if (log_file) |*lf| {
            lf.file_writer.interface.flush() catch {}; // Like an ofstream, write errors are not reported
            lf.file.close(lf.io);
            log_file = null;
            writer = null;
        }
    }
}

/// JPH_DET_LOG(...): write the values of the tuple `values` (Jolt's `<<` chain) and a newline to `writer`.
/// Only call it under `if (DeterminismLog.enabled)`.
pub fn log(values: anytype) void {
    if (!enabled) @compileError("DeterminismLog.log: the determinism log is disabled, call it under `if (DeterminismLog.enabled)`");
    if (writer) |w| {
        // Like an ofstream, write errors are not reported
        state.write(w, values) catch {};
        w.writeByte('\n') catch {};
    }
}

/// The text formatting of DeterminismLog's `operator <<` overloads, with the stream's base flag (also used by the
/// tests when the log is disabled)
pub const Format = struct {
    /// The stream is in hexadecimal mode (`std::hex`), otherwise decimal (`std::dec`, the initial state)
    hex: bool = false,

    /// Write the values of the tuple `values` in order
    pub fn write(self: *Format, w: *std.Io.Writer, values: anytype) std.Io.Writer.Error!void {
        inline for (values) |value|
            try self.writeValue(w, value);
    }

    /// One `operator <<`
    pub fn writeValue(self: *Format, w: *std.Io.Writer, value: anytype) std.Io.Writer.Error!void {
        const T = @TypeOf(value);
        if (comptime isString(T)) {
            // operator << (const char *) / (const String &)
            self.hex = false;
            try w.writeAll(value);
        } else if (T == i32) {
            // operator << (int): the current base, no width
            if (self.hex)
                try w.print("{x}", .{@as(u32, @bitCast(value))})
            else
                try w.print("{d}", .{value});
        } else if (T == u32) {
            try self.writeHex32(w, value);
        } else if (T == u64) {
            try self.writeHex64(w, value);
        } else if (T == f32) {
            try self.writeHex32(w, @bitCast(value));
        } else if (T == BodyID) {
            try self.writeHex32(w, value.getIndexAndSequenceNumber());
        } else if (T == SubShapeID) {
            try self.writeHex32(w, value.getValue());
        } else if (T == Vec3) {
            try self.writeFloats(w, &.{ value.getX(), value.getY(), value.getZ() });
        } else if (T == Float3) {
            try self.writeFloats(w, &.{ value.x, value.y, value.z });
        } else if (T == Vec4) {
            try self.writeFloats(w, &.{ value.getX(), value.getY(), value.getZ(), value.getW() });
        } else if (T == Quat) {
            try self.writeValue(w, value.getXYZW());
        } else if (T == DVec3) {
            try self.writeHex64(w, @bitCast(value.getX()));
            try w.writeByte(' ');
            try self.writeHex64(w, @bitCast(value.getY()));
            try w.writeByte(' ');
            try self.writeHex64(w, @bitCast(value.getZ()));
        } else if (T == Mat44) {
            try self.write(w, .{ value.getColumn4(0), " ", value.getColumn4(1), " ", value.getColumn4(2), " ", value.getColumn4(3) });
        } else if (T == DMat44) {
            try self.write(w, .{ value.getColumn4(0), " ", value.getColumn4(1), " ", value.getColumn4(2), " ", value.getTranslation() });
        } else {
            @compileError("DeterminismLog: no operator << for " ++ @typeName(T));
        }
    }

    /// std::hex << std::setw(8) << value (the fill is '0')
    fn writeHex32(self: *Format, w: *std.Io.Writer, value: u32) std.Io.Writer.Error!void {
        self.hex = true;
        try w.print("{x:0>8}", .{value});
    }

    /// std::hex << std::setw(16) << value (the fill is '0')
    fn writeHex64(self: *Format, w: *std.Io.Writer, value: u64) std.Io.Writer.Error!void {
        self.hex = true;
        try w.print("{x:0>16}", .{value});
    }

    /// The components of a Vec3 / Vec4 / Float3: 8 hex digits each, separated by a space (written directly to the
    /// stream in Jolt, so the base stays hexadecimal)
    fn writeFloats(self: *Format, w: *std.Io.Writer, components: []const f32) std.Io.Writer.Error!void {
        for (components, 0..) |c, i| {
            if (i != 0) try w.writeByte(' ');
            try self.writeHex32(w, @bitCast(c));
        }
    }

    fn isString(comptime T: type) bool {
        return switch (@typeInfo(T)) {
            .pointer => |p| switch (p.size) {
                .slice => p.child == u8,
                .one => switch (@typeInfo(p.child)) {
                    .array => |a| a.child == u8,
                    else => false,
                },
                else => false,
            },
            else => false,
        };
    }
};

/// The text of `values` as `log` writes it (without the newline), for the tests
fn formatForTest(buffer: []u8, format: *Format, values: anytype) ![]const u8 {
    var w: std.Io.Writer = .fixed(buffer);
    try format.write(&w, values);
    return w.buffered();
}

test "DeterminismLog: Jolt's text for every operator <<" {
    const expectEqualStrings = std.testing.expectEqualStrings;
    var buffer: [512]u8 = undefined;
    var format: Format = .{};

    // Strings are written as is; an int after a string is decimal, without width
    try expectEqualStrings("PhysicsSystem::Update: dt: 3c888889 steps: 1", try formatForTest(&buffer, &format, .{ "PhysicsSystem::Update: dt: ", @as(f32, 1.0 / 60.0), " steps: ", @as(i32, 1) }));
    try std.testing.expect(!format.hex);
    try expectEqualStrings("-12", try formatForTest(&buffer, &format, .{@as(i32, -12)}));

    // After a hex value the stream stays hexadecimal: an int is then written in hex (two's complement), until a string
    // switches back to decimal
    try expectEqualStrings("0000000a1b 27 ffffffff", try formatForTest(&buffer, &format, .{ @as(u32, 10), @as(i32, 27), " ", @as(i32, 27), " ", @as(u32, 0xffffffff) }));
    try std.testing.expect(format.hex);
    try expectEqualStrings("fffffff4", try formatForTest(&buffer, &format, .{@as(i32, -12)}));
    try expectEqualStrings(" 12", try formatForTest(&buffer, &format, .{ " ", @as(i32, 12) }));
    try std.testing.expect(!format.hex);

    // uint64, float bits, BodyID and SubShapeID
    try expectEqualStrings("0123456789abcdef 00000000 80000000 3f800000 7fc00000", try formatForTest(&buffer, &format, .{ @as(u64, 0x0123456789abcdef), " ", @as(f32, 0.0), " ", @as(f32, -0.0), " ", @as(f32, 1.0), " ", std.math.nan(f32) }));
    try expectEqualStrings("id: 00800005 sub: fffffffe", try formatForTest(&buffer, &format, .{ "id: ", BodyID.fromIndexAndSequenceNumber(5, 1), " sub: ", SubShapeID{ .value = 0xfffffffe } }));

    // Vectors
    try expectEqualStrings("v: 3f800000 40000000 c0400000", try formatForTest(&buffer, &format, .{ "v: ", Vec3.init(1, 2, -3) }));
    try expectEqualStrings("3f800000 40000000 40400000 40800000", try formatForTest(&buffer, &format, .{Vec4.init(1, 2, 3, 4)}));
    try expectEqualStrings("3f000000 00000000 bf800000", try formatForTest(&buffer, &format, .{Float3.init(0.5, 0, -1)}));
    try expectEqualStrings("3ff0000000000000 c000000000000000 0000000000000000", try formatForTest(&buffer, &format, .{DVec3.init(1, -2, 0)}));
    try expectEqualStrings("00000000 00000000 00000000 3f800000", try formatForTest(&buffer, &format, .{Quat.identity()}));

    // Matrices: the columns separated by " " strings, DMat44 ends with the double translation
    try expectEqualStrings("3f800000 00000000 00000000 00000000 00000000 3f800000 00000000 00000000 00000000 00000000 3f800000 00000000 3f800000 40000000 40400000 3f800000", try formatForTest(&buffer, &format, .{Mat44.translation(Vec3.init(1, 2, 3))}));
    try expectEqualStrings("3f800000 00000000 00000000 00000000 00000000 3f800000 00000000 00000000 00000000 00000000 3f800000 00000000 3ff0000000000000 4000000000000000 4008000000000000", try formatForTest(&buffer, &format, .{DMat44.translation(DVec3.init(1, 2, 3))}));
    try std.testing.expect(format.hex);
}

test "DeterminismLog: the log only exists with the build option" {
    if (enabled) {
        try std.testing.expect(@TypeOf(writer) == ?*std.Io.Writer);
    } else {
        try std.testing.expect(@TypeOf(writer) == void and @TypeOf(base_path) == void);
        // Opening and closing a disabled log does nothing
        try open(std.testing.io);
        close();
    }
}
