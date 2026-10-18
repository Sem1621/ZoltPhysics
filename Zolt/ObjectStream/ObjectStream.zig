//! Port of: Jolt/ObjectStream/ObjectStream.h, Jolt/ObjectStream/ObjectStream.cpp
//! Status: complete
//!
//! - `ObjectStream::EStreamType` is `StreamType`, `sVersion` / `sRevision` / `sNullIdentifier` are constants here.
//! - `IObjectStreamIn` / `IObjectStreamOut` disappear: the attribute functions take the concrete `*ObjectStreamIn` /
//!   `*ObjectStreamOut`, whose text and binary formats are a tagged union (a closed set of final classes).
//! - The OSIsType / OSReadData / OSWriteData / OSWriteDataType overloads (JPH_DECLARE_PRIMITIVE, the templates for
//!   Array, StaticArray, C arrays, Ref and RefConst) are the codecs below, chosen at compile time from the Zig member
//!   type by `Codec`. The class codec (JPH_IMPLEMENT_SERIALIZATION_FUNCTIONS) is in SerializableObject.zig, the enum
//!   codec in SerializableAttributeEnum.zig.
//!
//! | Zig member type                                              | Stream type                  | Jolt                    |
//! |--------------------------------------------------------------|------------------------------|-------------------------|
//! | u8 u16 i32 u32 u64 f32 f64 bool Float3 Float4 Double3 Vec3 DVec3 Vec4 UVec4 Quat Mat44 DMat44 | the primitive | JPH_DECLARE_PRIMITIVE |
//! | `[]u8` / `[]const u8` (owned by the object's allocator)       | string                       | String                  |
//! | a class with an RTTI (by value)                              | instance C (its attributes)  | C                       |
//! | `Ref(C)` / `RefConst(C)`                                      | instance C (an identifier!)  | Ref<C> / RefConst<C>    |
//! | `?*C` / `?*const C`                                           | pointer C (an identifier)    | C *                     |
//! | `std.ArrayList(E)`, `StaticArray(E, N)`, `[N]E`                | array E                      | Array<E>, StaticArray<E, N>, E[N] |
//!
//! Jolt declares a Ref as `instance C` although its data is an identifier, so a reader cannot tell a Ref from a by
//! value instance without the member's type (SkipAttributeData misreads a Ref that it has to skip; reproduced).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const virtual = @import("../Core/Virtual.zig");
const Lifetime = @import("../Core/Lifetime.zig");
const Reference = @import("../Core/Reference.zig");
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const ObjectStreamIn = @import("ObjectStreamIn.zig").ObjectStreamIn;
const ObjectStreamOut = @import("ObjectStreamOut.zig").ObjectStreamOut;
const TypeDeclarations = @import("TypeDeclarations.zig");
const getPrimitiveTypeOfType = @import("GetPrimitiveTypeOfType.zig").getPrimitiveTypeOfType;
const InstanceCodec = @import("SerializableObject.zig").InstanceCodec;

/// Stream type (EStreamType)
pub const StreamType = enum { text, binary };

/// Identifier for objects
pub const Identifier = u32;

pub const version = 1;
pub const revision = 0;
pub const null_identifier: Identifier = 0;

/// The codec of members of type M (the OS* overloads that C++ overload resolution picks for M): a struct with
/// `array_depth`, `data_type`, `class_rtti`, `primitive_rtti`, `read(in, *M, source)` and `write(out, *const M)`.
/// `declared` is the class of `.attrAs` (Jolt's member points to it, Zolt's to one of its base classes).
pub fn Codec(comptime M: type, comptime declared: ?type, comptime where: []const u8) type {
    if (comptime TypeDeclarations.primitiveDataType(M) != null) return PrimitiveCodec(M);
    if (comptime isString(M)) return StringCodec(M);
    if (comptime ArrayListElement(M)) |E| return ArrayCodec(M, E, .dynamic, 0, declared, where);
    if (comptime StaticArrayElement(M)) |E| return ArrayCodec(M, E, .static, M.capacity, declared, where);
    if (comptime RefTarget(M)) |Target| return ReferenceCodec(M, Target, .instance, declared);
    switch (@typeInfo(M)) {
        .array => |a| return ArrayCodec(M, a.child, .fixed, a.len, declared, where),
        .optional => |o| if (@typeInfo(o.child) == .pointer and @typeInfo(o.child).pointer.size == .one)
            return ReferenceCodec(M, @typeInfo(o.child).pointer.child, .pointer, declared),
        .@"enum" => @compileError(where ++ " is an enum: use .enumAttr (JPH_ADD_ENUM_ATTRIBUTE)"),
        .@"struct" => |s| if (s.layout == .@"packed") @compileError(where ++ " is a packed struct (bit flags): use .enumAttr (JPH_ADD_ENUM_ATTRIBUTE)"),
        else => {},
    }
    if (declared != null) @compileError(where ++ ": .attrAs is only for references and pointers");
    return InstanceCodec(M);
}

pub fn isString(comptime M: type) bool {
    return M == []u8 or M == []const u8;
}

/// The Ref(C) / RefConst(C) target C, null for other types
pub fn RefTarget(comptime M: type) ?type {
    if (@typeInfo(M) != .@"struct" or !@hasField(M, "ptr")) return null;
    const P = @FieldType(M, "ptr");
    if (@typeInfo(P) != .optional or @typeInfo(@typeInfo(P).optional.child) != .pointer) return null;
    const C = @typeInfo(@typeInfo(P).optional.child).pointer.child;
    if (@typeInfo(C) != .@"struct") return null;
    return if (M == Reference.Ref(C) or M == Reference.RefConst(C)) C else null;
}

/// The element type of std.ArrayList(E), null for other types
pub fn ArrayListElement(comptime M: type) ?type {
    if (@typeInfo(M) != .@"struct" or !@hasField(M, "items") or !@hasField(M, "capacity")) return null;
    const E = @typeInfo(@FieldType(M, "items")).pointer.child;
    return if (M == std.ArrayList(E)) E else null;
}

/// The element type of StaticArray(E, N), null for other types
pub fn StaticArrayElement(comptime M: type) ?type {
    if (@typeInfo(M) != .@"struct" or !@hasField(M, "buffer") or !@hasDecl(M, "capacity")) return null;
    const E = @typeInfo(@FieldType(M, "buffer")).array.child;
    return if (M == StaticArray(E, M.capacity)) E else null;
}

/// JPH_DECLARE_PRIMITIVE: OSIsType is `array_depth == 0 && data_type == T_name`, OSWriteData is HintNextItem +
/// WritePrimitiveData
fn PrimitiveCodec(comptime M: type) type {
    return struct {
        pub const array_depth: u32 = 0;
        pub const data_type: OSDataType = TypeDeclarations.primitiveDataType(M).?;
        pub const class_rtti: ?*const RTTI = null;
        pub const primitive_rtti: ?*const RTTI = getPrimitiveTypeOfType(M, null);

        pub fn read(in: *ObjectStreamIn, member: *M, source: OSDataType) ObjectStreamIn.Error!void {
            if (source == data_type) return in.readPrimitive(M, member);
            // The conversions of ReadClassData (a stream written with the other precision)
            if (M == DVec3 and source == .t_vec3) {
                // Vec3 to DVec3
                var tmp: Vec3 = undefined;
                try in.readPrimitive(Vec3, &tmp);
                member.* = DVec3.fromVec3(tmp);
            } else if (M == Vec3 and source == .t_dvec3) {
                // DVec3 to Vec3
                var tmp: DVec3 = undefined;
                try in.readPrimitive(DVec3, &tmp);
                member.* = tmp.toVec3();
            } else unreachable; // ReadRTTI only maps convertible types
        }

        pub fn write(out: *ObjectStreamOut, member: *const M) ObjectStreamOut.Error!void {
            try out.hintNextItem();
            try out.writePrimitive(M, member.*);
        }
    };
}

/// JPH_DECLARE_PRIMITIVE(String): the member owns its string, allocated with the stream's allocator (the allocator of
/// every object the stream creates)
fn StringCodec(comptime M: type) type {
    return struct {
        pub const array_depth: u32 = 0;
        pub const data_type: OSDataType = .t_string;
        pub const class_rtti: ?*const RTTI = null;
        pub const primitive_rtti: ?*const RTTI = getPrimitiveTypeOfType(M, null);

        pub fn read(in: *ObjectStreamIn, member: *M, source: OSDataType) ObjectStreamIn.Error!void {
            std.debug.assert(source == data_type);
            const value = try in.readString();
            in.allocator.free(member.*);
            member.* = value;
        }

        pub fn write(out: *ObjectStreamOut, member: *const M) ObjectStreamOut.Error!void {
            try out.hintNextItem();
            try out.writeString(member.*);
        }
    };
}

/// Ref(C) / RefConst(C) (Jolt: declared as `instance C`, the data is an identifier) and `?*C` (the `C *` functions of
/// JPH_IMPLEMENT_SERIALIZATION_FUNCTIONS: `pointer C`)
fn ReferenceCodec(comptime M: type, comptime Target: type, comptime kind: OSDataType, comptime declared: ?type) type {
    const Declared = declared orelse Target;
    // A type level check (the RTTI of a class that refers to itself is not built yet)
    if (!virtual.isDerivedFrom(Declared, Target))
        @compileError(@typeName(Declared) ++ " (.attrAs) does not derive from " ++ @typeName(Target) ++ ", the class of the member");
    // `*Target` or `*const Target`, the pointer that the member holds
    const Ptr = @typeInfo(if (kind == .pointer) M else @FieldType(M, "ptr")).optional.child;
    return struct {
        pub const array_depth: u32 = 0;
        pub const data_type: OSDataType = kind;
        pub const class_rtti: ?*const RTTI = RTTI.of(Declared);
        pub const primitive_rtti: ?*const RTTI = getPrimitiveTypeOfType(Declared, null);

        /// Link resolution: point the member at `object` (an object of class `object_rtti`, which is kind of
        /// Declared) or at nothing. A Ref adds a reference (Jolt: `++refcount`), a raw pointer does not.
        fn store(member_erased: *anyopaque, object: ?*anyopaque, object_rtti: *const RTTI) void {
            const member: *M = @ptrCast(@alignCast(member_erased));
            const target: ?Ptr = if (object) |o| @ptrCast(@alignCast(object_rtti.castTo(o, RTTI.of(Target)).?)) else null;
            if (kind == .pointer) member.* = target else member.set(target);
        }

        pub fn read(in: *ObjectStreamIn, member: *M, source: OSDataType) ObjectStreamIn.Error!void {
            std.debug.assert(source == data_type);
            return in.readPointerData(RTTI.of(Declared), @ptrCast(member), store, kind == .instance);
        }

        pub fn write(out: *ObjectStreamOut, member: *const M) ObjectStreamOut.Error!void {
            const ptr: ?*const Target = if (kind == .pointer) member.* else member.ptr;
            if (ptr) |p| {
                // The dynamic class and the address of the most derived object (Jolt: GetRTTI(p), p)
                try out.writePointerData(RTTI.getRTTI(Target, p), RTTI.mostDerived(Target, p));
            } else try out.writePointerData(null, null);
        }
    };
}

const ArrayShape = enum { dynamic, static, fixed };

/// Array<E> (`std.ArrayList(E)`), StaticArray<E, N> and E[N]
fn ArrayCodec(comptime M: type, comptime E: type, comptime shape: ArrayShape, comptime n: usize, comptime declared: ?type, comptime where: []const u8) type {
    const C = Codec(E, declared, where);
    return struct {
        pub const array_depth: u32 = C.array_depth + 1;
        pub const data_type: OSDataType = C.data_type;
        pub const class_rtti: ?*const RTTI = C.class_rtti;
        pub const primitive_rtti: ?*const RTTI = C.primitive_rtti;

        pub fn read(in: *ObjectStreamIn, member: *M, source: OSDataType) ObjectStreamIn.Error!void {
            std.debug.assert(source == data_type);

            // Read array length
            const count = try in.readCount();

            // Read array items. All elements exist before the first one is read (Jolt's clear() + resize()), so that
            // the links into them stay valid.
            const items: []E = switch (shape) {
                .dynamic => blk: {
                    for (member.items) |*item| Lifetime.destruct(E, in.allocator, item);
                    member.clearRetainingCapacity();
                    try member.ensureTotalCapacityPrecise(in.allocator, count);
                    for (0..count) |_| member.appendAssumeCapacity(try Lifetime.defaultValue(E, in.allocator));
                    break :blk member.items;
                },
                .static => blk: {
                    // Check if we can fit this many elements
                    if (count > n) return error.InvalidObjectStream;
                    for (member.slice()) |*item| Lifetime.destruct(E, in.allocator, item);
                    member.clear();
                    member.resize(count);
                    for (member.slice()) |*item| item.* = try Lifetime.defaultValue(E, in.allocator);
                    break :blk member.slice();
                },
                .fixed => blk: {
                    if (count != n) return error.InvalidObjectStream;
                    break :blk member;
                },
            };
            for (items) |*item| try C.read(in, item, C.data_type);
        }

        pub fn write(out: *ObjectStreamOut, member: *const M) ObjectStreamOut.Error!void {
            const items: []const E = switch (shape) {
                .dynamic => member.items,
                .static => member.constSlice(),
                .fixed => member,
            };

            // Write size of array
            try out.hintNextItem();
            try out.writeCount(@intCast(items.len));

            // Write data in array
            out.hintIndentUp();
            for (items) |*item| try C.write(out, item);
            out.hintIndentDown();
        }
    };
}
