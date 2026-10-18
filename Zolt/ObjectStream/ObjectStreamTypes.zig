//! Port of: Jolt/ObjectStream/ObjectStreamTypes.h
//! Status: complete
//!
//! The list of primitive types (Jolt expands JPH_DECLARE_PRIMITIVE over it three times: the EOSDataType values, the
//! OS* function declarations and their definitions). Note: order is important, an enum is created and its value is
//! stored in a binary stream (OSDataType in SerializableAttribute.zig has the same order).
//!
//! `String` is a slice: `[]const u8`, or `[]u8` for an owned string (both are the String primitive).

const OSDataType = @import("SerializableAttribute.zig").OSDataType;
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

pub const Primitive = struct {
    /// The Zig type
    type: type,
    /// The name of its RTTI (TypeDeclarations.cpp)
    name: []const u8,
    /// T_name
    data_type: OSDataType,
};

/// JPH_DECLARE_PRIMITIVE, in the order of ObjectStreamTypes.h
pub const primitives = [_]Primitive{
    .{ .type = u8, .name = "uint8", .data_type = .t_uint8 },
    .{ .type = u16, .name = "uint16", .data_type = .t_uint16 },
    .{ .type = i32, .name = "int", .data_type = .t_int },
    .{ .type = u32, .name = "uint32", .data_type = .t_uint32 },
    .{ .type = u64, .name = "uint64", .data_type = .t_uint64 },
    .{ .type = f32, .name = "float", .data_type = .t_float },
    .{ .type = bool, .name = "bool", .data_type = .t_bool },
    .{ .type = []const u8, .name = "String", .data_type = .t_string },
    .{ .type = Float3, .name = "Float3", .data_type = .t_float3 },
    .{ .type = Vec3, .name = "Vec3", .data_type = .t_vec3 },
    .{ .type = Vec4, .name = "Vec4", .data_type = .t_vec4 },
    .{ .type = Quat, .name = "Quat", .data_type = .t_quat },
    .{ .type = Mat44, .name = "Mat44", .data_type = .t_mat44 },
    .{ .type = f64, .name = "double", .data_type = .t_double },
    .{ .type = DVec3, .name = "DVec3", .data_type = .t_dvec3 },
    .{ .type = DMat44, .name = "DMat44", .data_type = .t_dmat44 },
    .{ .type = Double3, .name = "Double3", .data_type = .t_double3 },
    .{ .type = Float4, .name = "Float4", .data_type = .t_float4 },
    .{ .type = UVec4, .name = "UVec4", .data_type = .t_uvec4 },
};

/// The Zig type of a primitive data type
pub fn PrimitiveType(comptime data_type: OSDataType) type {
    for (primitives) |p| {
        if (p.data_type == data_type) return p.type;
    }
    @compileError("not a primitive data type");
}

comptime {
    for (primitives, 0..) |p, i| {
        if (@intFromEnum(p.data_type) != @intFromEnum(OSDataType.t_uint8) + i) @compileError("ObjectStreamTypes: order of " ++ p.name);
    }
}
