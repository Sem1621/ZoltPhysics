//! Port of: Jolt/ObjectStream/ObjectStreamBinaryOut.h, Jolt/ObjectStream/ObjectStreamBinaryOut.cpp
//! Status: complete
//!
//! The binary format of ObjectStreamOut: little endian (Jolt writes native bytes, every supported platform is little
//! endian), no padding. Vec3 / DVec3 are 3 components (12 / 24 bytes), DMat44 is 3 Vec4 columns and a DVec3 (72
//! bytes), bool is 1 byte. Strings (names and String values) share one table: the first time a string is written as
//! its length and bytes, afterwards as its id (0x80000000, 0x80000001, ...); the empty string is length 0 and never
//! enters the table.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Writer = std.Io.Writer;
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

pub const Error = Allocator.Error || Writer.Error;

/// Implementation of ObjectStream binary output stream
pub const ObjectStreamBinaryOut = struct {
    writer: *Writer,
    /// Strings that have been written, keys owned by the stream's allocator
    string_table: std.StringHashMapUnmanaged(u32) = .empty,
    next_string_id: u32 = 0x80000000,

    /// Constructor: writes the header "BOS 1.00"
    pub fn init(writer: *Writer) Writer.Error!ObjectStreamBinaryOut {
        try writer.print("BOS{d:>2}.{d:0>2}", .{ ObjectStream.version, ObjectStream.revision });
        return .{ .writer = writer };
    }

    pub fn deinit(self: *ObjectStreamBinaryOut, allocator: Allocator) void {
        var it = self.string_table.keyIterator();
        while (it.next()) |key| allocator.free(key.*);
        self.string_table.deinit(allocator);
    }

    pub fn writeDataType(self: *ObjectStreamBinaryOut, data_type: OSDataType) Writer.Error!void {
        try self.writer.writeInt(u32, @intFromEnum(data_type), .little);
    }

    pub fn writeIdentifier(self: *ObjectStreamBinaryOut, identifier: Identifier) Writer.Error!void {
        try self.writer.writeInt(u32, identifier, .little);
    }

    pub fn writeCount(self: *ObjectStreamBinaryOut, count: u32) Writer.Error!void {
        try self.writer.writeInt(u32, count, .little);
    }

    pub fn writePrimitive(self: *ObjectStreamBinaryOut, comptime T: type, value: T) Writer.Error!void {
        const w = self.writer;
        switch (T) {
            u8, u16, i32, u32, u64 => try w.writeInt(T, value, .little),
            f32 => try w.writeInt(u32, @bitCast(value), .little),
            f64 => try w.writeInt(u64, @bitCast(value), .little),
            bool => try w.writeByte(@intFromBool(value)),
            Float3, Double3 => inline for (.{ value.x, value.y, value.z }) |c| try self.writePrimitive(@TypeOf(c), c),
            Float4 => inline for (.{ value.x, value.y, value.z, value.w }) |c| try self.writePrimitive(f32, c),
            // Vec3 / DVec3 without their W component
            Vec3, DVec3 => inline for (.{ value.getX(), value.getY(), value.getZ() }) |c| try self.writePrimitive(@TypeOf(c), c),
            Vec4, UVec4, Quat => inline for (.{ value.getX(), value.getY(), value.getZ(), value.getW() }) |c| try self.writePrimitive(@TypeOf(c), c),
            Mat44 => for (0..4) |i| try self.writePrimitive(Vec4, value.getColumn4(@intCast(i))),
            DMat44 => {
                for (0..3) |i| try self.writePrimitive(Vec4, value.getColumn4(@intCast(i)));
                try self.writePrimitive(DVec3, value.getTranslation());
            },
            else => @compileError("not a primitive: " ++ @typeName(T)),
        }
    }

    /// WritePrimitiveData(const String &), also used for names
    pub fn writeString(self: *ObjectStreamBinaryOut, allocator: Allocator, value: []const u8) Error!void {
        // Empty strings are trivial
        if (value.len == 0) return self.writeCount(0);

        // Check if we've already written this string
        if (self.string_table.get(value)) |id| return self.writeCount(id);

        // Insert string in table
        const key = try allocator.dupe(u8, value);
        errdefer allocator.free(key);
        try self.string_table.putNoClobber(allocator, key, self.next_string_id);
        self.next_string_id += 1;

        // Write string
        const len: u32 = @intCast(@min(value.len, 0x7fffffff));
        try self.writeCount(len);
        try self.writer.writeAll(value[0..len]);
    }
};
