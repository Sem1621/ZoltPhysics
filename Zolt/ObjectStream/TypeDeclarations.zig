//! Port of: Jolt/ObjectStream/TypeDeclarations.h, Jolt/ObjectStream/TypeDeclarations.cpp
//! Status: complete
//!
//! The RTTI of the primitives (JPH_IMPLEMENT_RTTI_OUTSIDE_CLASS) and of the classes that Jolt declares outside the
//! class (JPH_IMPLEMENT_SERIALIZABLE_OUTSIDE_CLASS: Color, AABox, Triangle, IndexedTriangleNoMaterial, IndexedTriangle
//! and Plane). `RTTI.of(T)` looks here when T has no `rtti` declaration. Only this file declares classes from the
//! outside, so that the Math, Geometry and Core types do not import the ObjectStream; every other class of the library
//! and of the user declares `pub const rtti` itself.
//!
//! Zolt: the members that C++ leaves indeterminate in a default constructed outside class (`new Color`, `mIdx` of
//! `new IndexedTriangle`) are zero (Lifetime.zig, divergence D9).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const primitives = @import("ObjectStreamTypes.zig").primitives;
const Color = @import("../Core/Color.zig").Color;
const AABox = @import("../Geometry/AABox.zig").AABox;
const Triangle = @import("../Geometry/Triangle.zig").Triangle;
const IndexedTriangleNoMaterial = @import("../Geometry/IndexedTriangle.zig").IndexedTriangleNoMaterial;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
const Plane = @import("../Geometry/Plane.zig").Plane;

/// The RTTIs of all primitives (JPH_IMPLEMENT_RTTI_OUTSIDE_CLASS(uint8) ...), in the order of ObjectStreamTypes.h.
/// ObjectStreamOut never declares them.
pub const primitive_rtti: [primitives.len]*const RTTI = blk: {
    var result: [primitives.len]*const RTTI = undefined;
    for (primitives, 0..) |p, i| result[i] = primitiveRTTI(p.type, p.name);
    break :blk result;
};

/// The stream type of primitive T (null for other types, also for strings, which have their own codec)
pub fn primitiveDataType(comptime T: type) ?OSDataType {
    for (primitives) |p| {
        if (p.type == T and p.data_type != .t_string) return p.data_type;
    }
    return null;
}

/// The RTTI of a primitive: no bases, no attributes, creatable
fn primitiveRTTI(comptime T: type, comptime name: []const u8) *const RTTI {
    const Value = if (T == []const u8) []u8 else T;
    return &struct {
        const value: RTTI = .{
            .name = name,
            .hash = RTTI.hashName(name),
            .kind = .primitive,
            .bases = &.{},
            .attributes = &.{},
            .create = create,
            .destroy = destroy,
            .ref_count = null,
        };
        fn create(allocator: Allocator) Allocator.Error!*anyopaque {
            const object = try allocator.create(Value);
            object.* = std.mem.zeroes(Value);
            return @ptrCast(object);
        }
        fn destroy(object: *anyopaque, allocator: Allocator) void {
            const self: *Value = @ptrCast(@alignCast(object));
            if (Value == []u8) allocator.free(self.*);
            allocator.destroy(self);
        }
    }.value;
}

/// JPH_IMPLEMENT_SERIALIZABLE_OUTSIDE_CLASS
const outside_class = .{
    .{ Color, RTTI.declare(Color, .{
        .name = "Color",
        .kind = .non_virtual,
        .attributes = &.{ .attr("r", .r), .attr("g", .g), .attr("b", .b), .attr("a", .a) },
    }) },
    .{ AABox, RTTI.declare(AABox, .{
        .name = "AABox",
        .kind = .non_virtual,
        .attributes = &.{ .attr("mMin", .min), .attr("mMax", .max) },
    }) },
    .{ Triangle, RTTI.declare(Triangle, .{
        .name = "Triangle",
        .kind = .non_virtual,
        .attributes = &.{ .attr("mV", .v), .attr("mMaterialIndex", .material_index), .attr("mUserData", .user_data) },
    }) },
    .{ IndexedTriangleNoMaterial, RTTI.declare(IndexedTriangleNoMaterial, .{
        .name = "IndexedTriangleNoMaterial",
        .kind = .non_virtual,
        .attributes = &.{.attr("mIdx", .idx)},
    }) },
    .{
        IndexedTriangle,
        RTTI.declare(IndexedTriangle, .{
            .name = "IndexedTriangle",
            .kind = .non_virtual,
            .bases = &.{IndexedTriangleNoMaterial}, // A flattened prefix (Zolt's IndexedTriangle starts with the fields of IndexedTriangleNoMaterial)
            .attributes = &.{ .attr("mMaterialIndex", .material_index), .attr("mUserData", .user_data) },
        }),
    },
    .{ Plane, RTTI.declare(Plane, .{
        .name = "Plane",
        .kind = .non_virtual,
        .attributes = &.{.attr("mNormalAndConstant", .normal_and_constant)},
    }) },
};

/// The RTTI of a primitive or of a class declared here (GetRTTIOfType of these types), null for other types
pub fn find(comptime T: type) ?*const RTTI {
    const Key = if (T == []u8) []const u8 else T;
    for (primitives, 0..) |p, i| {
        if (p.type == Key) return primitive_rtti[i];
    }
    for (outside_class) |entry| {
        if (entry[0] == T) return entry[1];
    }
    return null;
}
