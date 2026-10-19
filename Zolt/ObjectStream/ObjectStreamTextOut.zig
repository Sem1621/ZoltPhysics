//! Port of: Jolt/ObjectStream/ObjectStreamTextOut.h, Jolt/ObjectStream/ObjectStreamTextOut.cpp
//! Status: complete
//!
//! The text format of ObjectStreamOut, a member of its format union (ObjectStreamOut.zig). Floats are written like
//! `std::ostream` with precision 9 (doubles 17), that is printf("%.9g") / ("%.17g") (StringTools.writeFloatPrecision);
//! integers like `std::to_string`; identifiers as %08X; lines end in "\r\n" on every platform, two spaces per
//! indentation level.

const std = @import("std");
const Writer = std.Io.Writer;
const writeFloatPrecision = @import("../Core/StringTools.zig").writeFloatPrecision;
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

/// Implementation of ObjectStream text output stream
pub const ObjectStreamTextOut = struct {
    writer: *Writer,
    indentation: u32 = 0,

    /// Constructor: writes the header "TOS 1.00" (StringFormat("TOS%2d.%02d", sVersion, sRevision))
    pub fn init(writer: *Writer) Writer.Error!ObjectStreamTextOut {
        try writer.print("TOS{d:>2}.{d:0>2}", .{ ObjectStream.version, ObjectStream.revision });
        return .{ .writer = writer };
    }

    pub fn writeDataType(self: *ObjectStreamTextOut, data_type: OSDataType) Writer.Error!void {
        try self.writer.writeAll(switch (data_type) {
            .declare => "declare ",
            .object => "object ",
            .instance => "instance ",
            .pointer => "pointer ",
            .array => "array ",
            .t_uint8 => "uint8",
            .t_uint16 => "uint16",
            .t_int => "int",
            .t_uint32 => "uint32",
            .t_uint64 => "uint64",
            .t_float => "float",
            .t_double => "double",
            .t_bool => "bool",
            .t_string => "string",
            .t_float3 => "float3",
            .t_float4 => "float4",
            .t_double3 => "double3",
            .t_vec3 => "vec3",
            .t_dvec3 => "dvec3",
            .t_vec4 => "vec4",
            .t_uvec4 => "uvec4",
            .t_quat => "quat",
            .t_mat44 => "mat44",
            .t_dmat44 => "dmat44",
            .invalid, _ => unreachable, // Only the writer chooses data types (Jolt: JPH_ASSERT(false))
        });
    }

    pub fn writeName(self: *ObjectStreamTextOut, name: []const u8) Writer.Error!void {
        try self.writer.writeAll(name);
        try self.writer.writeByte(' ');
    }

    pub fn writeIdentifier(self: *ObjectStreamTextOut, identifier: Identifier) Writer.Error!void {
        try self.writer.print("{X:0>8}", .{identifier});
    }

    pub fn writeCount(self: *ObjectStreamTextOut, count: u32) Writer.Error!void {
        try self.writer.print("{d}", .{count});
    }

    pub fn writePrimitive(self: *ObjectStreamTextOut, comptime T: type, value: T) Writer.Error!void {
        const w = self.writer;
        switch (T) {
            u8, u16, i32, u32, u64 => try w.print("{d}", .{value}), // std::to_string
            f32 => try writeFloatPrecision(w, value, 9),
            f64 => try writeFloatPrecision(w, value, 17),
            bool => try w.writeAll(if (value) "true" else "false"),
            Float3, Double3 => try self.writeComponents(.{ value.x, value.y, value.z }),
            Float4 => try self.writeComponents(.{ value.x, value.y, value.z, value.w }),
            Vec3, DVec3 => try self.writeComponents(.{ value.getX(), value.getY(), value.getZ() }),
            Vec4, UVec4, Quat => try self.writeComponents(.{ value.getX(), value.getY(), value.getZ(), value.getW() }),
            Mat44 => try self.writeComponents(.{ value.getColumn4(0), value.getColumn4(1), value.getColumn4(2), value.getColumn4(3) }),
            DMat44 => try self.writeComponents(.{ value.getColumn4(0), value.getColumn4(1), value.getColumn4(2), value.getTranslation() }),
            else => @compileError("not a primitive: " ++ @typeName(T)),
        }
    }

    /// Multi component values: the components separated by single spaces
    fn writeComponents(self: *ObjectStreamTextOut, components: anytype) Writer.Error!void {
        inline for (components, 0..) |c, i| {
            if (i != 0) try self.writer.writeByte(' ');
            try self.writePrimitive(@TypeOf(c), c);
        }
    }

    /// A string in double quotes, escaping `\`, newline, tab and `"` (the StringReplace calls of Jolt, in one pass)
    pub fn writeString(self: *ObjectStreamTextOut, value: []const u8) Writer.Error!void {
        try self.writer.writeByte('"');
        for (value) |c| {
            switch (c) {
                '\\' => try self.writer.writeAll("\\\\"),
                '\n' => try self.writer.writeAll("\\n"),
                '\t' => try self.writer.writeAll("\\t"),
                '"' => try self.writer.writeAll("\\\""),
                else => try self.writer.writeByte(c),
            }
        }
        try self.writer.writeByte('"');
    }

    pub fn hintNextItem(self: *ObjectStreamTextOut) Writer.Error!void {
        try self.writer.writeAll("\r\n");
        for (0..self.indentation) |_| try self.writer.writeAll("  ");
    }

    pub fn hintIndentUp(self: *ObjectStreamTextOut) void {
        self.indentation += 1;
    }

    pub fn hintIndentDown(self: *ObjectStreamTextOut) void {
        self.indentation -= 1;
    }
};
