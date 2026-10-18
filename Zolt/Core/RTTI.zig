//! Port of: Jolt/Core/RTTI.h, Jolt/Core/RTTI.cpp
//! Status: partial
//! Missing: the `rtti` data entry in the root vtables of ShapeSettings, PhysicsMaterial, GroupFilter,
//! ConstraintSettings, PathConstraintPath, VehicleControllerSettings and WheelSettings (Phase 8 foundation, see
//! Docs/Zolt/SerializationArchitecture.md S3).
//!
//! Jolt builds one `RTTI` object per class at startup (a function local static that `sCreateRTTI` fills with
//! JPH_ADD_BASE_CLASS / JPH_ADD_ATTRIBUTE). Zolt builds the same record at compile time from a declaration next to the
//! fields and keeps it as an immutable constant, `T.rtti: *const RTTI` (JPH_RTTI(T)):
//!
//! ```zig
//! pub const SphereShapeSettings = struct {
//!     /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SphereShapeSettings)
//!     pub const rtti = RTTI.declare(@This(), .{
//!         .name = "SphereShapeSettings",
//!         .kind = .virtual,
//!         .bases = &.{ConvexShapeSettings},
//!         .attributes = &.{.attr("mRadius", .radius)},
//!     });
//!     ...
//! ```
//!
//! | Jolt                                                 | Zolt                                                              |
//! |------------------------------------------------------|-------------------------------------------------------------------|
//! | JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL / _OUTSIDE_CLASS | `.kind = .non_virtual` (GetRTTI is the static RTTI, creatable)  |
//! | JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL / _VIRTUAL_BASE   | `.kind = .virtual` (GetRTTI reads the `rtti` vtable entry, creatable) |
//! | JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT / _ABSTRACT_BASE | `.kind = .abstract` (not creatable)                               |
//! | JPH_IMPLEMENT_RTTI_OUTSIDE_CLASS (TypeDeclarations)  | a declaration in ObjectStream/TypeDeclarations.zig                |
//! | JPH_ADD_BASE_CLASS(C, B)                             | `B` in `.bases` (the field of type B, or a flattened prefix of C) |
//! | JPH_ADD_ATTRIBUTE / _ENUM_ / _WITH_ALIAS             | `.attr` / `.enumAttr` / `.alias` / `.enumAlias` in `.attributes`  |
//! | JPH_RTTI(C)                                          | `RTTI.of(C)` (= `C.rtti`)                                         |
//! | GetRTTI(ptr), IsKindOf, DynamicCast                  | `RTTI.getRTTI(C, ptr)`, `rtti.isKindOf`, `RTTI.dynamicCast`       |
//! | StaticCast                                           | `virtual.upcast` / `virtual.downcast`                             |
//! | RTTI::CreateObject / DestructObject                  | `rtti.createObject(allocator)` / `rtti.destructObject(object, allocator)` |
//! | JPH_RENAME_CLASS (unused in Jolt)                    | `.renamed = true` and the new name                                |
//!
//! Differences from Jolt:
//! - The creatability comes from the `kind`, which is Jolt's IMPLEMENT macro (not the DECLARE macro): PhysicsMaterial,
//!   ConstraintSettings and WheelSettings are `.virtual`, DecoratedShapeSettings is `.abstract`.
//! - The base offsets are the real Zig offsets (Zig may reorder fields, `base` need not be at offset 0). `castTo` adds
//!   them like Jolt's CastTo; Jolt's ObjectStreamIn assumes offset 0, Zolt's casts.
//! - RTTI identity is the address of the constant, like Jolt's `operator ==`. Every RTTI of a factory has its own name
//!   (a compile error otherwise), so no two RTTIs have the same contents.
//!
//! Compile time checks (each a readable @compileError): the name is the class's name, the kind fits the class (a
//! virtual class has the `rtti` vtable entry, a creatable class that owns an allocator declares `createDefault`), every
//! base is a base sub object or a flattened prefix, every attribute names a real member of a serializable type, the
//! attribute name is the member's name unless it is an alias, `.attr` is not used on an enum (JPH_ADD_ENUM_ATTRIBUTE),
//! a String member defaults to "", at most 32 attributes and 4 bases (Jolt's StaticArrays), no attribute name twice,
//! no two attributes in the same memory.

const std = @import("std");
const Allocator = std.mem.Allocator;
const virtual = @import("Virtual.zig");
const HashCombine = @import("HashCombine.zig");
const RefCount = @import("Reference.zig").RefCount;
const Lifetime = @import("Lifetime.zig");
const serializable_attribute = @import("../ObjectStream/SerializableAttribute.zig");
const SerializableAttribute = serializable_attribute.SerializableAttribute;
const AttributeDecl = serializable_attribute.AttributeDecl;
const TypeDeclarations = @import("../ObjectStream/TypeDeclarations.zig");

/// Light weight runtime type information system (compile time constants, see the file comment)
pub const RTTI = struct {
    /// Class name (Jolt's: the stringified macro argument, "Outer::Inner" for nested classes)
    name: []const u8,
    /// GetHash(): HashString(name) folded to 32 bits, computed at compile time
    hash: u32,
    /// How the class was declared
    kind: Kind,
    /// Base classes (JPH_ADD_BASE_CLASS) with the offset of their sub object
    bases: []const BaseClass,
    /// The attributes: the flattened attributes of the bases (in base order), then the class's own (Jolt's order)
    attributes: []const SerializableAttribute,
    /// `new T` (null: abstract). Objects are owned by `allocator`; classes that keep an allocator (pattern A roots,
    /// RefTargets) keep this one. Reference counted objects start with reference count 0.
    create: ?*const fn (allocator: Allocator) Allocator.Error!*anyopaque,
    /// `delete (T *)object` (null: abstract). A value class is freed with `allocator` (the allocator that created it), a
    /// class that keeps its allocator uses its own.
    destroy: ?*const fn (object: *anyopaque, allocator: Allocator) void,
    /// The reference count of an object of a reference counted class (Jolt: sInternalGetRefCountOffset), null for other
    /// classes
    ref_count: ?*const fn (object: *anyopaque) *const RefCount,

    pub const Kind = enum {
        /// A primitive of ObjectStreamTypes.h (JPH_IMPLEMENT_RTTI_OUTSIDE_CLASS in TypeDeclarations.cpp)
        primitive,
        /// JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL / _OUTSIDE_CLASS: GetRTTI(ptr) is the static RTTI
        non_virtual,
        /// JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL / _VIRTUAL_BASE: GetRTTI(ptr) reads the `rtti` entry of the vtable
        virtual,
        /// JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT / _ABSTRACT_BASE: virtual, not creatable
        abstract,
    };

    /// Base class information
    pub const BaseClass = struct {
        rtti: *const RTTI,
        /// Offset of the base sub object in the derived class (JPH_BASE_CLASS_OFFSET)
        offset: usize,
    };

    /// The body of JPH_IMPLEMENT_SERIALIZABLE_*: what `declare` turns into an RTTI
    pub const Declaration = struct {
        /// Jolt's class name: the stream name and the hash input ("RagdollSettings::Part" for nested classes). Checked
        /// against the Zig type name ("::" is "."), unless `renamed` (JPH_RENAME_CLASS).
        name: []const u8,
        kind: Kind,
        /// JPH_ADD_BASE_CLASS, in Jolt's order
        bases: []const type = &.{},
        /// JPH_ADD_ATTRIBUTE / JPH_ADD_ENUM_ATTRIBUTE (and _WITH_ALIAS), in Jolt's order
        attributes: []const AttributeDecl = &.{},
        /// The stream name differs from the Zig name on purpose (JPH_RENAME_CLASS)
        renamed: bool = false,
    };

    /// Jolt's limits: StaticArray<SerializableAttribute, 32> and StaticArray<BaseClass, 4>
    pub const max_attributes = 32;
    pub const max_bases = 4;

    /// JPH_DECLARE_SERIALIZABLE_* + JPH_IMPLEMENT_SERIALIZABLE_*: the RTTI of class `T`. Declare it once per class, as
    /// `pub const rtti = RTTI.declare(@This(), .{ ... })`.
    pub fn declare(comptime T: type, comptime decl: Declaration) *const RTTI {
        return &struct {
            const value: RTTI = build(T, decl);
        }.value;
    }

    /// JPH_RTTI(T): the RTTI of a class (its `rtti` declaration), of a class declared outside (TypeDeclarations) or of a
    /// primitive
    pub fn of(comptime T: type) *const RTTI {
        return comptime lookup(T);
    }

    fn lookup(comptime T: type) *const RTTI {
        switch (@typeInfo(T)) {
            .@"struct", .@"enum", .@"union", .@"opaque" => if (@hasDecl(T, "rtti")) {
                if (@TypeOf(T.rtti) != *const RTTI)
                    @compileError(@typeName(T) ++ ".rtti must be `RTTI.declare(@This(), .{ ... })`");
                return T.rtti;
            },
            else => {},
        }
        return TypeDeclarations.find(T) orelse
            @compileError(@typeName(T) ++ " is not serializable: declare `pub const rtti = RTTI.declare(@This(), .{ ... })` (JPH_DECLARE_SERIALIZABLE_*)");
    }

    /// GetRTTI(object): the RTTI of the most derived class for a virtual class (the `rtti` entry of its vtable), the
    /// static RTTI otherwise
    pub fn getRTTI(comptime T: type, object: *const T) *const RTTI {
        const static = comptime of(T);
        if (comptime static.kind == .virtual or static.kind == .abstract) {
            if (comptime VTableRoot(T)) |Root| return virtual.upcast(Root, object).vtable.rtti;
        }
        return static;
    }

    /// The address of the most derived object (the object of `getRTTI(T, object)`) that contains `object`. Zolt
    /// identifies objects by this address (Jolt by the `T *`, the same address for Jolt's classes).
    pub fn mostDerived(comptime T: type, object: *const T) *anyopaque {
        const dynamic = getRTTI(T, object);
        // The `rtti` vtable entry of a class derived from T is a class derived from T (unless a class declares the RTTI
        // of an unrelated class, a programming error that would make the pointer arithmetic wrong)
        const offset = dynamic.baseOffset(of(T)) orelse std.debug.panic("{s} is not a {s}: wrong `rtti` declaration", .{ dynamic.name, of(T).name });
        return @ptrFromInt(@intFromPtr(object) - offset);
    }

    /// DynamicCast<To>(object): null in, null out; null when the object is not a To
    pub fn dynamicCast(comptime To: type, comptime From: type, object: ?*From) ?*To {
        const o = object orelse return null;
        const result = getRTTI(From, o).castTo(mostDerived(From, o), of(To)) orelse return null;
        return @ptrCast(@alignCast(result));
    }

    /// IsType(object, rtti): true when the object is exactly of class `rtti` (and for null, like Jolt)
    pub fn isType(comptime T: type, object: ?*const T, rtti: *const RTTI) bool {
        const o = object orelse return true;
        return getRTTI(T, o) == rtti;
    }

    /// IsKindOf(object, rtti): true when the object is of class `rtti` or derived from it (and for null, like Jolt)
    pub fn isKindOfObject(comptime T: type, object: ?*const T, rtti: *const RTTI) bool {
        const o = object orelse return true;
        return getRTTI(T, o).isKindOf(rtti);
    }

    /// GetHash(): `HashString(name)` folded to 32 bits (Jolt's diffusion step)
    pub fn hashName(name: []const u8) u32 {
        // Perform diffusion step to get from 64 to 32 bits (see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function)
        const hash = HashCombine.hashString(name);
        return @truncate(hash ^ (hash >> 32));
    }

    pub fn getName(self: *const RTTI) []const u8 {
        return self.name;
    }

    pub fn getHash(self: *const RTTI) u32 {
        return self.hash;
    }

    /// Check if this class can be constructed
    pub fn isAbstract(self: *const RTTI) bool {
        return self.create == null;
    }

    pub fn getBaseClassCount(self: *const RTTI) usize {
        return self.bases.len;
    }

    pub fn getBaseClass(self: *const RTTI, index: usize) *const RTTI {
        return self.bases[index].rtti;
    }

    pub fn getAttributeCount(self: *const RTTI) usize {
        return self.attributes.len;
    }

    pub fn getAttribute(self: *const RTTI, index: usize) *const SerializableAttribute {
        return &self.attributes[index];
    }

    /// The offset of the `to` sub object in an object of this class (depth first over the bases, the first match like
    /// RTTI::CastTo), null when this class does not derive from `to`
    pub fn baseOffset(self: *const RTTI, to: *const RTTI) ?usize {
        if (self == to) return 0;
        for (self.bases) |base| {
            if (base.rtti.baseOffset(to)) |offset| return base.offset + offset;
        }
        return null;
    }

    /// Test if this class is derived from class of type `other`
    pub fn isKindOf(self: *const RTTI, other: *const RTTI) bool {
        return self.baseOffset(other) != null;
    }

    /// Cast `object` of this type to object of type `to`, returns null if the cast is unsuccessful
    pub fn castTo(self: *const RTTI, object: *anyopaque, to: *const RTTI) ?*anyopaque {
        const offset = self.baseOffset(to) orelse return null;
        return @ptrFromInt(@intFromPtr(object) + offset);
    }

    /// Create an object of this type (returns null if the object is abstract)
    pub fn createObject(self: *const RTTI, allocator: Allocator) Allocator.Error!?*anyopaque {
        const create = self.create orelse return null;
        return try create(allocator);
    }

    /// Destruct object of this type (an object that createObject returned, with reference count 0)
    pub fn destructObject(self: *const RTTI, object: *anyopaque, allocator: Allocator) void {
        self.destroy.?(object, allocator);
    }
};

/// The pattern A root class of T (the class with the `vtable` field at the end of the `base` chain), null for classes
/// without virtual functions
pub fn VTableRoot(comptime T: type) ?type {
    var L = T;
    while (true) {
        if (@typeInfo(L) != .@"struct") return null;
        if (@hasField(L, "vtable")) return L;
        if (!@hasField(L, "base")) return null;
        L = virtual.Parent(L);
    }
}

/// True when objects of T free themselves with their own allocator (a field `allocator` in T or in its `base` chain:
/// pattern A roots and RefTargets without virtual functions such as Skeleton)
fn ownsAllocator(comptime T: type) bool {
    var L = T;
    while (true) {
        if (@typeInfo(L) != .@"struct") return false;
        if (@hasField(L, "allocator")) return @FieldType(L, "allocator") == Allocator;
        if (!@hasField(L, "base")) return false;
        L = virtual.Parent(L);
    }
}

/// The class in T's `base` chain that holds `ref_count: RefCount`, null when T is not reference counted
fn RefCountHolder(comptime T: type) ?type {
    var L = T;
    while (true) {
        if (@typeInfo(L) != .@"struct") return null;
        if (@hasField(L, "ref_count") and @FieldType(L, "ref_count") == RefCount) return L;
        if (!@hasField(L, "base")) return null;
        L = virtual.Parent(L);
    }
}

fn build(comptime T: type, comptime decl: RTTI.Declaration) RTTI {
    @setEvalBranchQuota(1_000_000);
    checkName(T, decl);
    checkKind(T, decl.kind);
    const bases = buildBases(T, decl.bases);
    return .{
        .name = decl.name,
        .hash = RTTI.hashName(decl.name),
        .kind = decl.kind,
        .bases = bases,
        .attributes = buildAttributes(T, decl, bases),
        .create = if (decl.kind == .abstract) null else createFn(T),
        .destroy = if (decl.kind == .abstract) null else destroyFn(T),
        .ref_count = refCountFn(T),
    };
}

/// The name with "::" replaced by "." must be the end of the Zig type name (`RagdollSettings::Part` is the nested
/// `RagdollSettings.Part`), so that a copy pasted declaration cannot give a class the name of another
fn checkName(comptime T: type, comptime decl: RTTI.Declaration) void {
    if (decl.name.len == 0) @compileError(@typeName(T) ++ ": the RTTI name cannot be empty");
    if (decl.renamed) return;
    var dotted: []const u8 = "";
    var i: usize = 0;
    while (i < decl.name.len) : (i += 1) {
        if (i + 1 < decl.name.len and decl.name[i] == ':' and decl.name[i + 1] == ':') {
            dotted = dotted ++ ".";
            i += 1;
        } else dotted = dotted ++ decl.name[i .. i + 1];
    }
    const full = @typeName(T);
    if (!(std.mem.eql(u8, full, dotted) or std.mem.endsWith(u8, full, "." ++ dotted)))
        @compileError(@typeName(T) ++ ": RTTI name \"" ++ decl.name ++ "\" is not the name of the class (use `.renamed = true` for JPH_RENAME_CLASS)");
}

fn checkKind(comptime T: type, comptime kind: RTTI.Kind) void {
    const root = VTableRoot(T);
    switch (kind) {
        .primitive => @compileError(@typeName(T) ++ ": `.primitive` is reserved for TypeDeclarations"),
        .non_virtual => if (root != null)
            @compileError(@typeName(T) ++ " has virtual functions (a vtable): declare it `.virtual` or `.abstract`, a non virtual RTTI would write derived objects as " ++ @typeName(T)),
        .virtual, .abstract => if (root) |Root| {
            if (!@hasField(Root.VTable, "rtti") or @FieldType(Root.VTable, "rtti") != *const RTTI)
                @compileError(@typeName(T) ++ " is virtual: its root " ++ @typeName(Root) ++ ".VTable needs the data entry `rtti: *const RTTI` (GetRTTI)");
        } else if (kind == .virtual) {
            @compileError(@typeName(T) ++ " is declared `.virtual` but has no vtable (pattern A): declare it `.non_virtual`");
        },
    }
    if (kind == .virtual or kind == .non_virtual) {
        if (ownsAllocator(T)) {
            const ok = @hasDecl(T, "createDefault") and @TypeOf(T.createDefault) == fn (Allocator) Allocator.Error!*T;
            if (!ok) @compileError(@typeName(T) ++ " is creatable (JPH_IMPLEMENT_SERIALIZABLE_" ++ (if (kind == .virtual) "VIRTUAL" else "NON_VIRTUAL") ++ "): declare `pub fn createDefault(allocator: Allocator) Allocator.Error!*" ++ @typeName(T) ++ "` (new " ++ @typeName(T) ++ ", reference count 0)");
            if (root == null and !(@hasDecl(T, "deinit") and @TypeOf(T.deinit) == fn (*T) void))
                @compileError(@typeName(T) ++ " owns its allocator: declare `pub fn deinit(self: *" ++ @typeName(T) ++ ") void` (its destructor)");
        }
    }
}

fn buildBases(comptime T: type, comptime bases: []const type) []const RTTI.BaseClass {
    if (bases.len > RTTI.max_bases) @compileError(@typeName(T) ++ ": Jolt's RTTI holds at most 4 base classes");
    var result: [bases.len]RTTI.BaseClass = undefined;
    for (bases, 0..) |Base, i| result[i] = .{ .rtti = RTTI.of(Base), .offset = baseOffsetOf(T, Base) };
    const final = result;
    return &final;
}

/// JPH_BASE_CLASS_OFFSET(T, Base): the unique field of type Base (pattern A `base`, the second base of a class with two
/// bases), otherwise Base is a flattened prefix of T (virtual.checkPrefix, same field offsets) at offset 0; an empty
/// Base (SerializableObject) is always a flattened prefix
fn baseOffsetOf(comptime T: type, comptime Base: type) usize {
    var found: ?usize = null;
    for (@typeInfo(T).@"struct".fields) |field| {
        if (field.type == Base) {
            if (found != null) @compileError(@typeName(T) ++ " has two fields of base class type " ++ @typeName(Base));
            found = @offsetOf(T, field.name);
        }
    }
    if (found) |offset| return offset;
    const base_fields = @typeInfo(Base).@"struct".fields;
    const fields = @typeInfo(T).@"struct".fields;
    const is_prefix = fields.len >= base_fields.len and for (base_fields, fields[0..base_fields.len]) |b, f| {
        if (!std.mem.eql(u8, b.name, f.name) or b.type != f.type) break false;
    } else true;
    if (!is_prefix)
        @compileError(@typeName(T) ++ ": base class " ++ @typeName(Base) ++ " is neither a field of " ++ @typeName(T) ++ " (pattern A `base`) nor a flattened prefix (its fields first)");
    for (base_fields) |field| {
        if (@offsetOf(Base, field.name) != @offsetOf(T, field.name))
            @compileError(@typeName(T) ++ "." ++ field.name ++ " is not at the offset of the flattened base " ++ @typeName(Base) ++ " (make both extern structs)");
    }
    return 0;
}

fn buildAttributes(comptime T: type, comptime decl: RTTI.Declaration, comptime bases: []const RTTI.BaseClass) []const SerializableAttribute {
    var list: []const SerializableAttribute = &.{};
    // AddBaseClass copies the attributes of the base at the moment it is called (Jolt always adds the bases first)
    for (bases) |base| {
        for (base.rtti.attributes) |a| list = list ++ &[_]SerializableAttribute{a.withBaseOffset(base.offset)};
    }
    for (decl.attributes) |a| list = list ++ &[_]SerializableAttribute{serializable_attribute.build(T, a)};
    if (list.len > RTTI.max_attributes)
        @compileError(std.fmt.comptimePrint("{s}: {d} attributes, Jolt's RTTI holds at most {d}", .{ @typeName(T), list.len, RTTI.max_attributes }));
    for (list, 0..) |a, i| {
        for (list[0..i]) |b| {
            if (std.mem.eql(u8, a.name, b.name))
                @compileError(@typeName(T) ++ ": attribute " ++ a.name ++ " is declared twice (a stream can only address the first)");
            // Every member is read at most once per object (the reader keeps link addresses into arrays valid)
            if (a.member_size != 0 and b.member_size != 0 and a.member_offset < b.member_offset + b.member_size and b.member_offset < a.member_offset + a.member_size)
                @compileError(@typeName(T) ++ ": attributes " ++ b.name ++ " and " ++ a.name ++ " share memory");
        }
    }
    const final = list[0..list.len].*;
    return &final;
}

fn createFn(comptime T: type) *const fn (Allocator) Allocator.Error!*anyopaque {
    return &struct {
        fn f(allocator: Allocator) Allocator.Error!*anyopaque {
            if (comptime ownsAllocator(T)) return try T.createDefault(allocator);
            const object = try allocator.create(T);
            errdefer allocator.destroy(object);
            object.* = try Lifetime.defaultValue(T, allocator);
            return object;
        }
    }.f;
}

fn destroyFn(comptime T: type) *const fn (*anyopaque, Allocator) void {
    return &struct {
        fn f(object: *anyopaque, allocator: Allocator) void {
            const self: *T = @ptrCast(@alignCast(object));
            if (comptime VTableRoot(T)) |Root| {
                // delete through the virtual destructor: the destructor chain and the free with the root's allocator
                const root: *Root = virtual.upcast(Root, self);
                root.vtable.destroy(root);
            } else if (comptime ownsAllocator(T)) {
                // A RefTarget without virtual functions (Skeleton, RagdollSettings, PhysicsScene)
                const owner = self.allocator;
                self.deinit();
                owner.destroy(self);
            } else {
                Lifetime.destruct(T, allocator, self);
                allocator.destroy(self);
            }
        }
    }.f;
}

fn refCountFn(comptime T: type) ?*const fn (*anyopaque) *const RefCount {
    const Holder = RefCountHolder(T) orelse return null;
    return &struct {
        fn f(object: *anyopaque) *const RefCount {
            const self: *T = @ptrCast(@alignCast(object));
            return &virtual.upcast(Holder, self).ref_count;
        }
    }.f;
}

test "RTTI: hashes, bases with real offsets, casts, create / destroy" {
    const testing = std.testing;
    // Jolt's RTTI::GetHash values (fact sheet: RegisterTypes in the C++ library)
    try testing.expectEqual(@as(u32, 0x66d6453c), RTTI.hashName("MotorSettings"));
    try testing.expectEqual(@as(u32, 0xbe3406ad), RTTI.hashName("SpringSettings"));
    try testing.expectEqual(@as(u32, 0x3828ae15), RTTI.hashName("RagdollSettings::Part"));
    try testing.expectEqual(@as(u32, 0xa7d2746c), RTTI.of(@import("Color.zig").Color).hash);
    try testing.expectEqual(@as(u32, 0x8c5fd1d4), RTTI.of([]const u8).hash); // String

    const A = struct {
        pub const rtti = RTTI.declare(@This(), .{ .name = "A", .kind = .non_virtual, .attributes = &.{.attr("mA", .a)} });
        a: u32 = 1,
    };
    const B = struct {
        pub const rtti = RTTI.declare(@This(), .{ .name = "B", .kind = .non_virtual, .attributes = &.{.attr("mB", .b)} });
        b: f32 = 2,
    };
    const C = struct {
        pub const rtti = RTTI.declare(@This(), .{ .name = "C", .kind = .non_virtual, .bases = &.{ A, B }, .attributes = &.{.attr("mC", .c)} });
        c: u8 = 3,
        base: A = .{},
        base2: B = .{},
    };
    const c_rtti = RTTI.of(C);
    try testing.expectEqualStrings("mA", c_rtti.attributes[0].name);
    try testing.expectEqualStrings("mB", c_rtti.attributes[1].name);
    try testing.expectEqualStrings("mC", c_rtti.attributes[2].name);
    try testing.expectEqual(@offsetOf(C, "base2") + @offsetOf(B, "b"), c_rtti.attributes[1].member_offset);
    try testing.expect(c_rtti.isKindOf(RTTI.of(A)) and c_rtti.isKindOf(RTTI.of(B)) and !RTTI.of(A).isKindOf(c_rtti));

    const object: *C = @ptrCast(@alignCast((try c_rtti.createObject(testing.allocator)).?));
    defer c_rtti.destructObject(object, testing.allocator);
    try testing.expectEqual(@as(?*anyopaque, &object.base2), c_rtti.castTo(object, RTTI.of(B)));
    try testing.expectEqual(@as(?*B, &object.base2), RTTI.dynamicCast(B, C, object));
    try testing.expectEqual(@as(?*A, null), RTTI.dynamicCast(A, B, &object.base2)); // B is not virtual: its static RTTI
    try testing.expectEqual(@as(u8, 3), object.c);
}
