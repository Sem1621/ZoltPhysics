//! Port of: Jolt/ObjectStream/SerializableAttribute.h
//! Status: complete
//!
//! - `EOSDataType` is `OSDataType`: its values are the binary token values, so the order is load bearing.
//! - Jolt's five function pointers per attribute (GetMemberPrimitiveType, IsType, ReadData, WriteData, WriteDataType)
//!   become one `MemberType` per member type, generated at compile time from the Zig type of the member (see
//!   SerializableAttributeTyped.zig and SerializableAttributeEnum.zig). `isType` and `writeDataType` are data driven.
//! - The JPH_ADD_ATTRIBUTE* macros are the `AttributeDecl` decl literals that `RTTI.declare` takes:
//!
//! | Jolt                                              | Zolt                                                             |
//! |---------------------------------------------------|------------------------------------------------------------------|
//! | JPH_ADD_ATTRIBUTE(C, mFoo)                        | `.attr("mFoo", .foo)` (the name is checked against the field)    |
//! | JPH_ADD_ATTRIBUTE_WITH_ALIAS(C, mA.mB, "mX")      | `.alias("mX", .{ .a, .b })` (a path of fields)                   |
//! | JPH_ADD_ENUM_ATTRIBUTE(C, mFoo)                   | `.enumAttr("mFoo", .foo)`                                        |
//! | JPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(C, mA.mB, "mX") | `.enumAlias("mX", .{ .a, .b })`                                  |
//! | a `Ref<Derived>` that Zolt holds as `Ref(Base)`   | `.attrAs("mFoo", .foo, Derived)` (Zolt)                          |
//! | a member that Zolt stores in another form         | `.custom("mFoo", Accessor)` (Zolt, BodyCreationSettings.mShape)  |

const std = @import("std");
const RTTI = @import("../Core/RTTI.zig").RTTI;
const ObjectStreamIn = @import("ObjectStreamIn.zig").ObjectStreamIn;
const ObjectStreamOut = @import("ObjectStreamOut.zig").ObjectStreamOut;
const typed = @import("SerializableAttributeTyped.zig");

/// Data type (EOSDataType)
pub const OSDataType = enum(u32) {
    // Control codes
    /// Used to declare the attributes of a new object type
    declare,
    /// Start of a new object
    object,
    /// Used in attribute declaration, indicates that an object is an instanced attribute (no pointer)
    instance,
    /// Used in attribute declaration, indicates that an object is a pointer attribute
    pointer,
    /// Used in attribute declaration, indicates that this is an array of objects
    array,

    // Basic types (primitives, JPH_DECLARE_PRIMITIVE in the order of ObjectStreamTypes.h)
    t_uint8,
    t_uint16,
    t_int,
    t_uint32,
    t_uint64,
    t_float,
    t_bool,
    t_string,
    t_float3,
    t_vec3,
    t_vec4,
    t_quat,
    t_mat44,
    t_double,
    t_dvec3,
    t_dmat44,
    t_double3,
    t_float4,
    t_uvec4,

    // Error values for read functions
    /// Next token on the stream was not a valid data type
    invalid,

    /// Any other value (a binary stream can hold any uint32, Jolt casts it to EOSDataType)
    _,
};

comptime {
    // The binary token values (the order of ObjectStreamTypes.h)
    std.debug.assert(@intFromEnum(OSDataType.t_uint8) == 5);
    std.debug.assert(@intFromEnum(OSDataType.t_float3) == 13);
    std.debug.assert(@intFromEnum(OSDataType.t_double) == 18);
    std.debug.assert(@intFromEnum(OSDataType.t_uvec4) == 23);
    std.debug.assert(@intFromEnum(OSDataType.invalid) == 24);
}

/// Attributes are members of classes that need to be serialized.
pub const SerializableAttribute = struct {
    /// Name of the attribute
    name: []const u8,
    /// Offset of the member relative to the class (for a custom attribute: 0, the accessor gets the class)
    member_offset: usize,
    /// Size of the member (0 for a custom attribute), for the overlap check of RTTI.declare
    member_size: usize,
    /// How the member is read and written
    member_type: *const MemberType,

    /// Construct from other attribute with base class offset
    pub fn withBaseOffset(self: SerializableAttribute, base_offset: usize) SerializableAttribute {
        var result = self;
        result.member_offset += base_offset;
        return result;
    }

    pub fn getName(self: *const SerializableAttribute) []const u8 {
        return self.name;
    }

    /// In case this attribute contains an RTTI type, return it (note that a Array<sometype> will return the rtti of
    /// sometype)
    pub fn getMemberPrimitiveType(self: *const SerializableAttribute) ?*const RTTI {
        return self.member_type.primitive_rtti;
    }

    /// Check if this attribute is of a specific type (OSIsType)
    pub fn isType(self: *const SerializableAttribute, array_depth: u32, data_type: OSDataType, class_name: []const u8) bool {
        const m = self.member_type;
        if (array_depth != m.array_depth or data_type != m.data_type) return false;
        return switch (data_type) {
            .instance, .pointer => std.mem.eql(u8, class_name, m.class_rtti.?.name),
            else => true, // Primitives and enums ignore the class name
        };
    }

    /// Read the data for this attribute into attribute containing class `object`. `source` is the type in the stream:
    /// the member's type, or Vec3 / DVec3 for the conversion of ReadClassData.
    pub fn readData(self: *const SerializableAttribute, in: *ObjectStreamIn, object: *anyopaque, source: OSDataType) ObjectStreamIn.Error!void {
        return self.member_type.read(in, @ptrFromInt(@intFromPtr(object) + self.member_offset), source);
    }

    /// Write the data for this attribute from attribute containing class `object`
    pub fn writeData(self: *const SerializableAttribute, out: *ObjectStreamOut, object: *const anyopaque) ObjectStreamOut.Error!void {
        return self.member_type.write(out, @ptrFromInt(@intFromPtr(object) + self.member_offset));
    }

    /// Write the data type of this attribute to a stream (OSWriteDataType)
    pub fn writeDataType(self: *const SerializableAttribute, out: *ObjectStreamOut) ObjectStreamOut.Error!void {
        for (0..self.member_type.array_depth) |_| try out.writeDataType(.array);
        try out.writeDataType(self.member_type.data_type);
        switch (self.member_type.data_type) {
            .instance, .pointer => try out.writeName(self.member_type.class_rtti.?.name),
            else => {},
        }
    }
};

/// The serialization functions of one member type (Jolt's function pointers of SerializableAttribute)
pub const MemberType = struct {
    /// Number of `array` prefixes of the declared type (OSIsType's inArrayDepth)
    array_depth: u32,
    /// Declared type of the element (OSWriteDataType)
    data_type: OSDataType,
    /// The class of an `instance` / `pointer` element
    class_rtti: ?*const RTTI,
    /// GetMemberPrimitiveType: the RTTI of the element (null for enums)
    primitive_rtti: ?*const RTTI,
    /// OSReadData (of the member, or of the class for a custom attribute)
    read: *const fn (in: *ObjectStreamIn, member: *anyopaque, source: OSDataType) ObjectStreamIn.Error!void,
    /// OSWriteData
    write: *const fn (out: *ObjectStreamOut, member: *const anyopaque) ObjectStreamOut.Error!void,
};

/// One JPH_ADD_ATTRIBUTE line of a declaration (`RTTI.Declaration.attributes`), made with the decl literals `.attr`,
/// `.enumAttr`, `.alias`, `.enumAlias`, `.attrAs` and `.custom`
pub const AttributeDecl = struct {
    /// Name in the stream (the member name, or the alias)
    name: []const u8,
    /// Field names from the class to the member (Jolt's `mSpringSettings.mFrequency` is `.{ .spring_settings, .frequency }`)
    path: []const []const u8,
    /// JPH_ADD_ENUM_ATTRIBUTE: stored as uint32
    enumeration: bool = false,
    /// JPH_ADD_ATTRIBUTE_WITH_ALIAS: the name is not checked against the member name
    is_alias: bool = false,
    /// Zolt: the class that Jolt's member points to when Zolt stores a base class (Jolt's
    /// `Ref<TwoBodyConstraintSettings>` is a `Ref(ConstraintSettings)` in Zolt): the stream declares `declared`, a link
    /// accepts only objects of `declared`
    declared: ?type = null,
    /// Zolt: the member is not a field (Zolt's layout differs from Jolt's). `accessor` declares `Member`, the type the
    /// stream sees, and `get(self: *const Class) Member` (what is written, a reference is borrowed). A reference member
    /// (Ref / RefConst) declares `set(self: *Class, target: ?*T) void`, called at link resolution like the store into a
    /// Ref field (a null identifier calls nothing, the member keeps its default); any other member declares
    /// `prepare(self: *Class) *Member`, the storage it is read into.
    accessor: ?type = null,

    /// JPH_ADD_ATTRIBUTE(Class, member): `member` is a field (`.radius`) or a path of fields (`.{ .spring_settings,
    /// .frequency }`); `name` must be the member's name in Jolt's spelling ("mRadius" for `radius`)
    pub fn attr(comptime name: []const u8, comptime member: anytype) AttributeDecl {
        return .{ .name = name, .path = toPath(member) };
    }

    /// JPH_ADD_ENUM_ATTRIBUTE(Class, member)
    pub fn enumAttr(comptime name: []const u8, comptime member: anytype) AttributeDecl {
        return .{ .name = name, .path = toPath(member), .enumeration = true };
    }

    /// JPH_ADD_ATTRIBUTE_WITH_ALIAS(Class, member, name)
    pub fn alias(comptime name: []const u8, comptime member: anytype) AttributeDecl {
        return .{ .name = name, .path = toPath(member), .is_alias = true };
    }

    /// JPH_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(Class, member, name)
    pub fn enumAlias(comptime name: []const u8, comptime member: anytype) AttributeDecl {
        return .{ .name = name, .path = toPath(member), .enumeration = true, .is_alias = true };
    }

    /// JPH_ADD_ATTRIBUTE of a `Ref<Declared>` / `Declared *` member that Zolt stores as a reference to a base class of
    /// `Declared` (or arrays of them)
    pub fn attrAs(comptime name: []const u8, comptime member: anytype, comptime Declared: type) AttributeDecl {
        return .{ .name = name, .path = toPath(member), .declared = Declared };
    }

    /// JPH_ADD_ATTRIBUTE of a member that Zolt keeps in another form (BodyCreationSettings' mShape / mShapePtr are the
    /// union `shape`), see `accessor`
    pub fn custom(comptime name: []const u8, comptime Accessor: type) AttributeDecl {
        return .{ .name = name, .path = &.{}, .accessor = Accessor };
    }

    fn toPath(comptime member: anytype) []const []const u8 {
        const M = @TypeOf(member);
        if (M == @TypeOf(.enum_literal)) return &.{@tagName(member)};
        if (@typeInfo(M) == .@"struct" and @typeInfo(M).@"struct".is_tuple) {
            var path: []const []const u8 = &.{};
            for (member) |m| {
                if (@TypeOf(m) != @TypeOf(.enum_literal)) @compileError("an attribute path is a tuple of field names (enum literals)");
                path = path ++ &[_][]const u8{@tagName(m)};
            }
            return path;
        }
        @compileError("an attribute member is a field name (`.radius`) or a tuple of field names (`.{ .spring_settings, .frequency }`)");
    }
};

/// Build the attribute `decl` of class `T` (AddSerializableAttributeTyped / AddSerializableAttributeEnum): resolve the
/// member, check the name, pick the member type
pub fn build(comptime T: type, comptime decl: AttributeDecl) SerializableAttribute {
    if (decl.accessor) |Accessor|
        return .{ .name = decl.name, .member_offset = 0, .member_size = 0, .member_type = typed.customMemberType(T, Accessor) };
    var M: type = T;
    var offset: usize = 0;
    for (decl.path) |field| {
        if (@typeInfo(M) != .@"struct" or !@hasField(M, field))
            @compileError(@typeName(T) ++ ": attribute " ++ decl.name ++ " names the member '" ++ field ++ "', which " ++ @typeName(M) ++ " does not have");
        offset += @offsetOf(M, field);
        M = @FieldType(M, field);
    }
    const last = decl.path[decl.path.len - 1];
    if (!decl.is_alias and !sameName(decl.name, last))
        @compileError(@typeName(T) ++ ": attribute " ++ decl.name ++ " is not the name of the member '" ++ last ++ "' (use .alias for JPH_ADD_ATTRIBUTE_WITH_ALIAS)");
    checkDefault(T, decl, last);
    const where = @typeName(T) ++ "." ++ decl.name;
    return .{
        .name = decl.name,
        .member_offset = offset,
        .member_size = @sizeOf(M),
        .member_type = if (decl.enumeration)
            @import("SerializableAttributeEnum.zig").memberType(M, where)
        else
            typed.memberType(M, decl.declared, where),
    };
}

/// Jolt's member name ("mHalfExtent", "mGroupID", "r") and the Zig field name ("half_extent", "group_id", "r") are the
/// same name: without the "m" prefix, the underscores and the case
fn sameName(comptime jolt: []const u8, comptime zig: []const u8) bool {
    const stripped = if (jolt.len > 1 and jolt[0] == 'm' and std.ascii.isUpper(jolt[1])) jolt[1..] else jolt;
    return std.mem.eql(u8, normalize(stripped), normalize(zig));
}

fn normalize(comptime name: []const u8) []const u8 {
    var result: []const u8 = "";
    for (name) |c| {
        if (c != '_') result = result ++ &[_]u8{std.ascii.toLower(c)};
    }
    return result;
}

/// Members that the reader frees must start empty: a String is freed with the object's allocator when it is read
fn checkDefault(comptime T: type, comptime decl: AttributeDecl, comptime field_name: []const u8) void {
    var Owner: type = T;
    for (decl.path[0 .. decl.path.len - 1]) |f| Owner = @FieldType(Owner, f);
    const field = for (@typeInfo(Owner).@"struct".fields) |f| {
        if (std.mem.eql(u8, f.name, field_name)) break f;
    } else unreachable;
    if (field.type == []u8 or field.type == []const u8) {
        const default = field.defaultValue() orelse
            @compileError(@typeName(Owner) ++ "." ++ field_name ++ " (attribute " ++ decl.name ++ ") needs a default value (an empty string)");
        if (default.len != 0)
            @compileError(@typeName(Owner) ++ "." ++ field_name ++ " (attribute " ++ decl.name ++ ") must default to an empty string: ObjectStreamIn frees the old string with the object's allocator");
    }
}
