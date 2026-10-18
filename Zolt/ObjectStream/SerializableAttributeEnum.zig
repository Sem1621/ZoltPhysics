//! Port of: Jolt/ObjectStream/SerializableAttributeEnum.h
//! Status: complete
//!
//! AddSerializableAttributeEnum<MemberType> (JPH_ADD_ENUM_ATTRIBUTE): the member is written as `uint32(value)` and read
//! as `static_cast<MemberType>(uint32)`, whatever its underlying type. Zolt accepts an enum, a packed struct of bit
//! flags (EAllowedDOFs) and an integer (ObjectLayer); the underlying type must be Jolt's (8, 16 or 32 bits, a compile
//! error otherwise, so a plain Zig `enum` must get Jolt's `enum(i32)`).
//!
//! Reading an unnamed value (SerializationArchitecture.md S13): a non-exhaustive enum, a packed struct and an integer
//! store every value like the C++ enum (truncated to the underlying type). An exhaustive Zig enum cannot hold an unnamed
//! value: the member keeps its value and the read goes on (Jolt stores the invalid value, divergence D3). This follows
//! the binary state of BodyCreationSettings.

const std = @import("std");
const RTTI = @import("../Core/RTTI.zig").RTTI;
const serializable_attribute = @import("SerializableAttribute.zig");
const OSDataType = serializable_attribute.OSDataType;
const MemberType = serializable_attribute.MemberType;
const ObjectStreamIn = @import("ObjectStreamIn.zig").ObjectStreamIn;
const ObjectStreamOut = @import("ObjectStreamOut.zig").ObjectStreamOut;

const log = std.log.scoped(.zolt);

/// The MemberType of an enum attribute of type M
pub fn memberType(comptime M: type, comptime where: []const u8) *const MemberType {
    const Int = switch (@typeInfo(M)) {
        .@"enum" => |e| e.tag_type,
        .@"struct" => |s| if (s.layout == .@"packed") s.backing_integer.? else @compileError(where ++ ": .enumAttr needs an enum, a packed struct or an integer"),
        .int => M,
        else => @compileError(where ++ ": .enumAttr needs an enum, a packed struct or an integer"),
    };
    const bits = @typeInfo(Int).int.bits;
    if (bits != 8 and bits != 16 and bits != 32)
        @compileError(where ++ ": give the enum Jolt's underlying type (enum(u8), enum(u16), enum(i32) or enum(u32)), it has " ++ @typeName(Int));
    const Unsigned = std.meta.Int(.unsigned, bits);
    return &struct {
        const value: MemberType = .{
            .array_depth = 0,
            .data_type = .t_uint32,
            .class_rtti = null,
            .primitive_rtti = null, // Enums have no RTTI
            .read = read,
            .write = write,
        };

        fn read(in: *ObjectStreamIn, member_erased: *anyopaque, source: OSDataType) ObjectStreamIn.Error!void {
            std.debug.assert(source == .t_uint32);
            const member: *M = @ptrCast(@alignCast(member_erased));
            var temporary: u32 = undefined;
            try in.readPrimitive(u32, &temporary);
            // static_cast<MemberType>(temporary): truncated to the underlying type
            const int: Int = @bitCast(@as(Unsigned, @truncate(temporary)));
            switch (@typeInfo(M)) {
                .@"enum" => |e| if (e.is_exhaustive) {
                    if (std.enums.fromInt(M, int)) |v| {
                        member.* = v;
                    } else {
                        // Zolt: an exhaustive enum keeps its value (D3)
                        log.info("ObjectStreamIn: {d} is not a valid {s}, the attribute keeps its value.", .{ temporary, @typeName(M) });
                    }
                } else {
                    member.* = @enumFromInt(int);
                },
                .@"struct" => member.* = @bitCast(int),
                else => member.* = int,
            }
        }

        fn write(out: *ObjectStreamOut, member_erased: *const anyopaque) ObjectStreamOut.Error!void {
            const member: *const M = @ptrCast(@alignCast(member_erased));
            const int: Int = switch (@typeInfo(M)) {
                .@"enum" => @intFromEnum(member.*),
                .@"struct" => @bitCast(member.*),
                else => member.*,
            };
            // uint32(value): a signed underlying type is sign extended, like the C++ conversion
            const temporary: u32 = if (@typeInfo(Int).int.signedness == .signed) @bitCast(@as(i32, int)) else int;
            try out.hintNextItem();
            try out.writePrimitive(u32, temporary);
        }
    }.value;
}
