//! Port of: Jolt/ObjectStream/ObjectStreamBinaryIn.h, Jolt/ObjectStream/ObjectStreamBinaryIn.cpp
//! Status: complete
//!
//! The binary format of ObjectStreamIn (see ObjectStreamBinaryOut.zig), a member of its format union. Like Jolt, a
//! string is cut at its first NUL character (Jolt assigns the `char *` to a String), and the string table stores the
//! cut string. Differences on malformed input only:
//! - a bool byte other than 0 or 1 is true (Jolt reads it into a C++ bool, undefined; D5);
//! - a string is read in chunks, so a corrupt length runs into the end of the stream (Jolt: JPH_STACK_ALLOC(len + 1),
//!   a stack overflow for a large length; D6);
//! - an unknown string id or a string that ends early is error.InvalidObjectStream (Jolt: false).

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

pub const Error = error{ OutOfMemory, ReadFailed, InvalidObjectStream };

/// Implementation of ObjectStream binary input stream
pub const ObjectStreamBinaryIn = struct {
    reader: *Reader,
    /// Strings read so far by id (owned by the stream's allocator)
    string_table: std.AutoHashMapUnmanaged(u32, []u8) = .empty,
    next_string_id: u32 = 0x80000000,

    pub fn deinit(self: *ObjectStreamBinaryIn, allocator: Allocator) void {
        var it = self.string_table.valueIterator();
        while (it.next()) |s| allocator.free(s.*);
        self.string_table.deinit(allocator);
    }

    fn readBytes(self: *ObjectStreamBinaryIn, buffer: []u8) Error!void {
        self.reader.readSliceAll(buffer) catch |err| return switch (err) {
            error.EndOfStream => error.InvalidObjectStream,
            error.ReadFailed => error.ReadFailed,
        };
    }

    fn readU32(self: *ObjectStreamBinaryIn) Error!u32 {
        var bytes: [4]u8 = undefined;
        try self.readBytes(&bytes);
        return std.mem.readInt(u32, &bytes, .little);
    }

    pub fn readDataType(self: *ObjectStreamBinaryIn, allocator: Allocator) Error!OSDataType {
        _ = allocator;
        return @enumFromInt(try self.readU32());
    }

    pub fn readIdentifier(self: *ObjectStreamBinaryIn, allocator: Allocator) Error!Identifier {
        _ = allocator;
        return self.readU32();
    }

    pub fn readCount(self: *ObjectStreamBinaryIn, allocator: Allocator) Error!u32 {
        _ = allocator;
        return self.readU32();
    }

    pub fn readPrimitive(self: *ObjectStreamBinaryIn, allocator: Allocator, comptime T: type, value: *T) Error!void {
        switch (T) {
            u8, u16, i32, u32, u64 => {
                var bytes: [@sizeOf(T)]u8 = undefined;
                try self.readBytes(&bytes);
                value.* = std.mem.readInt(T, &bytes, .little);
            },
            f32 => value.* = @bitCast(try self.read(allocator, u32)),
            f64 => value.* = @bitCast(try self.read(allocator, u64)),
            bool => value.* = (try self.read(allocator, u8)) != 0,
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

    fn read(self: *ObjectStreamBinaryIn, allocator: Allocator, comptime T: type) Error!T {
        var value: T = undefined;
        try self.readPrimitive(allocator, T, &value);
        return value;
    }

    /// A string from the string table or the stream (ReadPrimitiveData(String &)); the result is owned by the table
    /// (valid until deinit)
    fn readTableString(self: *ObjectStreamBinaryIn, allocator: Allocator) Error![]const u8 {
        // Read length or ID of string
        const len = try self.readU32();

        // Check empty string
        if (len == 0) return "";

        // Check if it is an ID in the string table
        if (len & 0x80000000 != 0) return self.string_table.get(len) orelse error.InvalidObjectStream;

        // Read the string (in chunks, so that a corrupt length runs into the end of the stream, not out of memory)
        var data: std.ArrayList(u8) = .empty;
        defer data.deinit(allocator);
        var remaining: usize = len;
        while (remaining > 0) {
            const chunk = @min(remaining, 4096);
            try data.ensureUnusedCapacity(allocator, chunk);
            const old_len = data.items.len;
            data.items.len += chunk;
            try self.readBytes(data.items[old_len..]);
            remaining -= chunk;
        }

        // Jolt assigns the `char *`: the string ends at the first NUL
        const string = try allocator.dupe(u8, std.mem.sliceTo(data.items, 0));
        errdefer allocator.free(string);

        // Insert string in table
        try self.string_table.putNoClobber(allocator, self.next_string_id, string);
        self.next_string_id += 1;
        return string;
    }

    /// A class or attribute name (valid until the stream is closed)
    pub fn readName(self: *ObjectStreamBinaryIn, allocator: Allocator) Error![]const u8 {
        return self.readTableString(allocator);
    }

    /// A String value, owned by `allocator`
    pub fn readString(self: *ObjectStreamBinaryIn, allocator: Allocator) Error![]u8 {
        return allocator.dupe(u8, try self.readTableString(allocator));
    }
};
