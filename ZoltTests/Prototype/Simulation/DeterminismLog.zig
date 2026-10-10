//! Phase 5 architecture prototype: Jolt/Physics/DeterminismLog.h (Docs/Zolt/SimulationArchitecture.md D11).
//!
//! JPH_DET_LOG(a << b << c) becomes `if (DeterminismLog.enabled) DeterminismLog.log(.{ a, b, c });`, which compiles to
//! nothing when the log is disabled (the default). F2 turns `enabled` into the build option `-Ddeterminism_log`.
//! The output is Jolt's exact text (std::hex with fill '0': floats as their bits, BodyIDs as 8 hex digits), so a Zolt
//! log and a C++ log of a single threaded run can be diffed line by line. The writer is the one module-level `var` of
//! Phase 5, and it only exists when the option is on (Jolt's global sLog; MotionProperties logs without a system
//! pointer). Like Jolt's, the log is only meaningful single threaded.

const std = @import("std");
const zolt = @import("zolt");
const BodyID = zolt.BodyID;
const SubShapeID = zolt.SubShapeID;
const Vec3 = zolt.Vec3;
const DVec3 = zolt.DVec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;

/// JPH_ENABLE_DETERMINISM_LOG (F2: a build option, default off)
pub const enabled = false;

/// DeterminismLog::sLog: where the log goes (set by the application). `void` when the log is disabled.
pub var writer: if (enabled) ?*std.Io.Writer else void = if (enabled) null else {};

/// JPH_DET_LOG(...): writes the values and a newline
pub fn log(args: anytype) void {
    if (!enabled) return;
    const w = writer orelse return;
    write(w, args) catch {};
}

/// Format the values of a JPH_DET_LOG line (`operator <<` of DeterminismLog for each value)
pub fn write(w: *std.Io.Writer, args: anytype) std.Io.Writer.Error!void {
    inline for (args) |value| try writeValue(w, value);
    try w.writeByte('\n');
}

fn writeValue(w: *std.Io.Writer, value: anytype) std.Io.Writer.Error!void {
    const T = @TypeOf(value);
    if (T == BodyID) {
        try w.print("{x:0>8}", .{value.getIndexAndSequenceNumber()});
    } else if (T == SubShapeID) {
        try w.print("{x:0>8}", .{value.getValue()});
    } else if (T == f32) {
        try w.print("{x:0>8}", .{@as(u32, @bitCast(value))});
    } else if (T == f64) {
        try w.print("{x:0>16}", .{@as(u64, @bitCast(value))});
    } else if (T == u32) {
        try w.print("{x:0>8}", .{value});
    } else if (T == u64) {
        try w.print("{x:0>16}", .{value});
    } else if (T == i32) {
        try w.print("{d}", .{value});
    } else if (T == Vec3) {
        try w.print("{x:0>8} {x:0>8} {x:0>8}", .{ @as(u32, @bitCast(value.getX())), @as(u32, @bitCast(value.getY())), @as(u32, @bitCast(value.getZ())) });
    } else if (T == DVec3) {
        try w.print("{x:0>16} {x:0>16} {x:0>16}", .{ @as(u64, @bitCast(value.getX())), @as(u64, @bitCast(value.getY())), @as(u64, @bitCast(value.getZ())) });
    } else if (T == Vec4) {
        try w.print("{x:0>8} {x:0>8} {x:0>8} {x:0>8}", .{ @as(u32, @bitCast(value.getX())), @as(u32, @bitCast(value.getY())), @as(u32, @bitCast(value.getZ())), @as(u32, @bitCast(value.getW())) });
    } else if (T == Quat) {
        try writeValue(w, value.getXYZW());
    } else {
        // String literals and slices (`operator << (const char *)`)
        try w.writeAll(value);
    }
}
