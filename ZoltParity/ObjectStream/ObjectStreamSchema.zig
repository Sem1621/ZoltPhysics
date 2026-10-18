//! The class model of the ObjectStream parity harness (ObjectStreamParity.zig, SerializationArchitecture.md section 5).
//!
//! - The C++ side: every class of the C++ Factory after RegisterTypes, with its schema line, its parsed attributes, how
//!   each attribute stores its data (ObjectStreamReference.cpp probes it) and the IsKindOf matrix.
//! - The Zolt side: the class in the Zolt factory, the `serialization_pending` marker (S18) and the values of the enum
//!   attributes, found at compile time from the Zig types that the `zolt` module exports.
//!
//! Nothing here is written for one class. A class that a group declares, or whose marker a group removes, is picked up
//! by every layer without a change here.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const OSDataType = zolt.OSDataType;

pub extern fn jolt_os_class_count() c_int;
pub extern fn jolt_os_class_name(index: c_int) c_int;
pub extern fn jolt_os_result(index: c_int, size: *usize) [*]const u8;
pub extern fn jolt_os_class_schema(name: [*:0]const u8) c_int;
pub extern fn jolt_os_is_kind_of(name: [*:0]const u8, base: [*:0]const u8) c_int;
pub extern fn jolt_os_probe_attribute(name: [*:0]const u8, index: c_int, element: *c_int, counts: *u64) c_int;
pub extern fn jolt_os_default_write(name: [*:0]const u8) c_int;
pub extern fn jolt_os_resave(data: [*]const u8, size: usize, name: [*:0]const u8) c_int;
pub extern fn jolt_os_read_word(word: [*]const u8, size: usize, word_type: c_int, bits: *u64) c_int;
pub extern fn jolt_os_asset_binary_state(data: [*]const u8, size: usize, name: [*:0]const u8) c_int;
pub extern fn jolt_os_unit_test_write(binary: c_int) c_int;
pub extern fn jolt_os_unit_test_resave(data: [*]const u8, size: usize, binary: c_int, ref_count: *u32, casts_ok: *c_int) c_int;
pub extern fn jolt_os_quarantine_check() c_int;

/// The bytes of a C++ result slot (valid until the next C++ call)
pub fn cppResult(index: c_int) []const u8 {
    var size: usize = 0;
    const data = jolt_os_result(index, &size);
    return data[0..size];
}

/// The result of a C++ call that runs in the allocation quarantine (P2 to P7). -5 means that the quarantine was lost
/// during the call: its hooks were replaced, or a block was freed twice. That is a harness error.
pub fn quarantined(result: c_int) error{HarnessError}!c_int {
    if (result == -5) {
        std.debug.print("the C++ allocation quarantine was lost during a call (its hooks were replaced or a block was freed twice)\n", .{});
        return error.HarnessError;
    }
    return result;
}

/// Phase 9 (S17): SoftBodyCreationSettings, SoftBodySharedSettings and its 11 nested classes. P1 skips them and drops
/// the attributes of other classes whose class is one of them (PhysicsScene.mSoftBodies); classes that reach them are
/// compared by cross-reading. Phase 9 removes this list.
pub const phase9_classes = [_][]const u8{
    "SoftBodyCreationSettings",
    "SoftBodySharedSettings",
    "SoftBodySharedSettings::Vertex",
    "SoftBodySharedSettings::Face",
    "SoftBodySharedSettings::Edge",
    "SoftBodySharedSettings::RodStretchShear",
    "SoftBodySharedSettings::RodBendTwist",
    "SoftBodySharedSettings::DihedralBend",
    "SoftBodySharedSettings::Volume",
    "SoftBodySharedSettings::InvBind",
    "SoftBodySharedSettings::SkinWeight",
    "SoftBodySharedSettings::Skinned",
    "SoftBodySharedSettings::LRA",
};

/// JPH_IMPLEMENT_SERIALIZABLE_OUTSIDE_CLASS: CreateObject uses the global new, which the allocation hooks of the C++ side
/// do not zero, so P2 skips their defaults (they are compared inside the classes that hold them)
pub const outside_classes = [_][]const u8{ "Color", "AABox", "Triangle", "IndexedTriangleNoMaterial", "IndexedTriangle", "Plane" };

/// The primitives of ObjectStreamTypes.h (their RTTI names)
pub const primitive_names = [_][]const u8{ "uint8", "uint16", "int", "uint32", "uint64", "float", "bool", "String", "Float3", "Vec3", "Vec4", "Quat", "Mat44", "double", "DVec3", "DMat44", "Double3", "Float4", "UVec4" };

fn contains(list: []const []const u8, name: []const u8) bool {
    for (list) |n| {
        if (std.mem.eql(u8, n, name)) return true;
    }
    return false;
}

pub fn isPhase9(name: []const u8) bool {
    return contains(&phase9_classes, name);
}

// The Zolt side ------------------------------------------------------------------------------------------------------

/// The Zolt factory that P1 to P7 compare with the C++ Factory after RegisterTypes: `RegisterTypes.factory` (S8).
pub const has_register_types_factory = @hasDecl(zolt.RegisterTypes, "factory");
pub const factory: *const Factory = if (has_register_types_factory) &zolt.RegisterTypes.factory else &stand_in.factory;

/// TEMPORARY stand-in for `RegisterTypes.factory`, used only while the Phase 8 migration (F2) has not added it: Jolt's
/// RegisterTypes list (RegisterTypes.cpp, without SoftBodyCreationSettings), restricted to the classes that already
/// declare `pub const rtti`. `factory` above switches to RegisterTypes.factory as soon as it exists; delete this then.
const stand_in = struct {
    const names = [_][]const u8{
        "SkeletalAnimation",            "Skeleton",                         "CompoundShapeSettings",           "StaticCompoundShapeSettings",
        "MutableCompoundShapeSettings", "TriangleShapeSettings",            "PlaneShapeSettings",              "SphereShapeSettings",
        "BoxShapeSettings",             "CapsuleShapeSettings",             "TaperedCapsuleShapeSettings",     "CylinderShapeSettings",
        "TaperedCylinderShapeSettings", "ScaledShapeSettings",              "MeshShapeSettings",               "ConvexHullShapeSettings",
        "HeightFieldShapeSettings",     "RotatedTranslatedShapeSettings",   "OffsetCenterOfMassShapeSettings", "EmptyShapeSettings",
        "RagdollSettings",              "PointConstraintSettings",          "SixDOFConstraintSettings",        "SliderConstraintSettings",
        "SwingTwistConstraintSettings", "DistanceConstraintSettings",       "HingeConstraintSettings",         "FixedConstraintSettings",
        "ConeConstraintSettings",       "PathConstraintSettings",           "VehicleConstraintSettings",       "WheeledVehicleControllerSettings",
        "WheelSettingsWV",              "TrackedVehicleControllerSettings", "WheelSettingsTV",                 "MotorcycleControllerSettings",
        "PathConstraintPath",           "PathConstraintPathHermite",        "RackAndPinionConstraintSettings", "GearConstraintSettings",
        "PulleyConstraintSettings",     "MotorSettings",                    "PhysicsScene",                    "PhysicsMaterial",
        "PhysicsMaterialSimple",        "GroupFilter",                      "GroupFilterTable",                "BodyCreationSettings",
    };
    const types = blk: {
        var list: []const type = &.{};
        for (names) |n| {
            if (@hasDecl(zolt, n) and isSerializable(@field(zolt, n))) list = list ++ &[_]type{@field(zolt, n)};
        }
        break :blk list;
    };
    const factory: Factory = .init(types);
};

/// True when T declares its RTTI (`pub const rtti = RTTI.declare(...)`)
pub fn isSerializable(comptime T: type) bool {
    return switch (@typeInfo(T)) {
        .@"struct", .@"union", .@"enum", .@"opaque" => @hasDecl(T, "rtti") and @TypeOf(T.rtti) == *const RTTI,
        else => false,
    };
}

/// The values that an enum attribute (JPH_ADD_ENUM_ATTRIBUTE) may hold: Jolt's named values for an enum, any value of
/// `bits` bits for a packed struct (bit flags) or an integer
pub const EnumAttribute = struct {
    name: []const u8,
    values: []const u32,
    bits: u8,
};

/// What Zolt knows about a class that declares its RTTI
pub const ZoltClass = struct {
    rtti: *const RTTI,
    /// `pub const serialization_pending = true;` (S18): the declaration has no attributes yet
    pending: bool,
    enums: []const EnumAttribute,
};

/// Every class that the `zolt` module exports (and the classes nested in a serializable class, like
/// `RagdollSettings.Part`), found at compile time
pub const zolt_classes: []const ZoltClass = blk: {
    @setEvalBranchQuota(50_000_000);
    var list: []const ZoltClass = &.{};
    for (@typeInfo(zolt).@"struct".decls) |d| {
        if (@TypeOf(@field(zolt, d.name)) != type) continue;
        const T = @field(zolt, d.name);
        if (!isSerializable(T)) continue;
        list = addClass(list, T);
        for (declsOf(T)) |nd| {
            if (@TypeOf(@field(T, nd.name)) != type) continue;
            const N = @field(T, nd.name);
            if (isSerializable(N)) list = addClass(list, N);
        }
    }
    break :blk list;
};

fn declsOf(comptime T: type) []const std.builtin.Type.Declaration {
    return switch (@typeInfo(T)) {
        inline .@"struct", .@"union", .@"enum", .@"opaque" => |info| info.decls,
        else => &.{},
    };
}

/// Add T, unless its RTTI is already there or is the RTTI of another class (a user class that declares the RTTI of its
/// parent, S3): the Zig type name must end in the RTTI name ("::" is ".")
fn addClass(comptime list: []const ZoltClass, comptime T: type) []const ZoltClass {
    for (list) |c| {
        if (c.rtti == T.rtti) return list;
    }
    var dotted: []const u8 = "";
    var i: usize = 0;
    const name = T.rtti.name;
    while (i < name.len) : (i += 1) {
        if (i + 1 < name.len and name[i] == ':' and name[i + 1] == ':') {
            dotted = dotted ++ ".";
            i += 1;
        } else dotted = dotted ++ name[i .. i + 1];
    }
    const full = @typeName(T);
    if (!(std.mem.eql(u8, full, dotted) or std.mem.endsWith(u8, full, "." ++ dotted))) return list;
    return list ++ &[_]ZoltClass{.{ .rtti = T.rtti, .pending = @hasDecl(T, "serialization_pending"), .enums = enumAttributes(T) }};
}

/// The enum attributes of T (no primitive RTTI, declared uint32) with their values, found by the member's offset
fn enumAttributes(comptime T: type) []const EnumAttribute {
    var list: []const EnumAttribute = &.{};
    for (T.rtti.attributes) |a| {
        const m = a.member_type;
        if (m.primitive_rtti != null or m.data_type != .t_uint32 or m.array_depth != 0 or a.member_size == 0) continue;
        const L = enumLeaf(T, a.member_offset, a.member_size) orelse continue;
        list = list ++ &[_]EnumAttribute{enumValues(a.name, L)};
    }
    return list;
}

/// The enum, packed struct or integer member of T at `offset` with `size` bytes
fn enumLeaf(comptime T: type, comptime offset: usize, comptime size: usize) ?type {
    switch (@typeInfo(T)) {
        .@"struct" => |s| {
            if (s.layout == .@"packed") return if (offset == 0 and @sizeOf(T) == size) T else null;
            for (s.fields) |f| {
                if (f.is_comptime) continue;
                const start = @offsetOf(T, f.name);
                if (offset >= start and offset < start + @sizeOf(f.type)) {
                    if (enumLeaf(f.type, offset - start, size)) |L| return L;
                }
            }
            return null;
        },
        .@"enum", .int => return if (offset == 0 and @sizeOf(T) == size) T else null,
        else => return null,
    }
}

fn enumValues(comptime name: []const u8, comptime L: type) EnumAttribute {
    switch (@typeInfo(L)) {
        .@"enum" => |e| {
            var values: [e.fields.len]u32 = undefined;
            // uint32(value): a signed tag is sign extended (S13)
            for (e.fields, 0..) |f, i| values[i] = @truncate(@as(u64, @bitCast(@as(i64, f.value))));
            const final = values;
            return .{ .name = name, .values = &final, .bits = @bitSizeOf(e.tag_type) };
        },
        else => return .{ .name = name, .values = &.{}, .bits = @bitSizeOf(L) },
    }
}

/// The Zolt class of an RTTI (null for primitives, the outside classes and classes that the module does not export)
pub fn zoltClass(rtti: *const RTTI) ?*const ZoltClass {
    for (zolt_classes) |*c| {
        if (c.rtti == rtti) return c;
    }
    return null;
}

pub fn isPending(rtti: *const RTTI) bool {
    const c = zoltClass(rtti) orelse return false;
    return c.pending;
}

/// True when the class or a class in its base chain is pending: its attribute list is not complete
pub fn hasPendingBase(rtti: *const RTTI) bool {
    if (isPending(rtti)) return true;
    for (rtti.bases) |b| {
        if (hasPendingBase(b.rtti)) return true;
    }
    return false;
}

/// The first pending class that `rtti` reaches through its bases and attribute classes (transitively), null when none.
/// A layer skips a class that reaches a pending class (S18): writing it declares that class.
pub fn reachesPending(rtti: *const RTTI) ?*const RTTI {
    var visited: [256]*const RTTI = undefined;
    var count: usize = 0;
    return reachesPendingVisit(rtti, &visited, &count);
}

fn reachesPendingVisit(rtti: *const RTTI, visited: *[256]*const RTTI, count: *usize) ?*const RTTI {
    for (visited[0..count.*]) |v| {
        if (v == rtti) return null;
    }
    visited[count.*] = rtti;
    count.* += 1;
    if (isPending(rtti)) return rtti;
    for (rtti.bases) |b| {
        if (reachesPendingVisit(b.rtti, visited, count)) |p| return p;
    }
    for (rtti.attributes) |*a| {
        if (a.getMemberPrimitiveType()) |t| {
            if (reachesPendingVisit(t, visited, count)) |p| return p;
        }
    }
    return null;
}

// The C++ side --------------------------------------------------------------------------------------------------------

/// How an attribute stores one element (ObjectStreamReference.cpp, jolt_os_probe_attribute)
pub const Element = enum {
    /// Not probed (an abstract class)
    unknown,
    /// A primitive or a string
    primitive,
    /// JPH_ADD_ENUM_ATTRIBUTE: uint32
    enumeration,
    /// A raw pointer: an identifier
    pointer,
    /// A Ref / RefConst: declared `instance X`, the data is an identifier
    reference,
    /// A class by value: declared `instance X`, the data is the class data
    value,
};

pub const Attribute = struct {
    name: []const u8,
    /// The declared type as ObjectStreamTextOut::WriteDataType writes it ("array instance Foo ")
    type_text: []const u8,
    /// GetMemberPrimitiveType (null for enums)
    primitive: ?[]const u8,
    /// Number of `array` prefixes
    depth: u32,
    /// Type of an element: a primitive data type, `.instance` or `.pointer`
    data_type: OSDataType,
    /// The class of an `instance` / `pointer` element
    class_name: []const u8,
    element: Element = .unknown,
    /// Arrays: bit c is set when an array of c elements can be read (C++ probe)
    counts: u64 = 0,
    /// The values of an enum attribute (Zolt)
    enum_values: ?*const EnumAttribute = null,
};

pub const Class = struct {
    name: [:0]const u8,
    /// The schema line of jolt_os_class_schema
    line: []const u8,
    hash: u32,
    abstract: bool,
    bases: []const []const u8,
    attributes: []Attribute,
    phase9: bool,
    outside: bool,
    primitive: bool,
    /// The class in the Zolt factory
    zolt: ?*const RTTI = null,
    /// Zolt: the first pending class that this class reaches, null when none
    waits_for: ?*const RTTI = null,
    /// The C++ class reaches a Phase 9 class (PhysicsScene.mSoftBodies): bytes are compared by cross-reading (S17)
    cross: bool = false,

    /// Zolt declares the class and everything it reaches: every layer compares it
    pub fn ready(self: *const Class) bool {
        return self.zolt != null and self.waits_for == null and !self.phase9;
    }

    /// A class whose objects the generators create and whose defaults P2 compares
    pub fn creatable(self: *const Class) bool {
        return !self.abstract and !self.phase9 and !self.outside and !self.primitive;
    }
};

/// The C++ classes (sorted by name) with the C++ IsKindOf matrix and the Zolt information. Built once per process.
pub const Schema = struct {
    classes: []Class,
    /// kind_of[i * n + j]: classes[i] IsKindOf classes[j]
    kind_of: []bool,

    pub fn find(self: *const Schema, name: []const u8) ?usize {
        for (self.classes, 0..) |*c, i| {
            if (std.mem.eql(u8, c.name, name)) return i;
        }
        return null;
    }

    pub fn isKindOf(self: *const Schema, a: usize, b: usize) bool {
        return self.kind_of[a * self.classes.len + b];
    }

    /// The class of `name` that the C++ side knows, an error when it does not (a harness error)
    pub fn get(self: *const Schema, name: []const u8) error{UnknownClass}!*const Class {
        const i = self.find(name) orelse return error.UnknownClass;
        return &self.classes[i];
    }
};

var schema_arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
var schema_instance: ?Schema = null;

/// The schema of this process (built on first use, never freed)
pub fn get() !*const Schema {
    if (schema_instance == null) schema_instance = try build(schema_arena.allocator());
    return &schema_instance.?;
}

fn build(arena: Allocator) !Schema {
    const n: usize = @intCast(jolt_os_class_count());
    const classes = try arena.alloc(Class, n);
    for (classes, 0..) |*c, i| {
        if (jolt_os_class_name(@intCast(i)) != 0) return error.HarnessError;
        const name = try arena.dupeZ(u8, cppResult(0));
        if (jolt_os_class_schema(name) != 0) return error.HarnessError;
        c.* = try parseLine(arena, name, try arena.dupe(u8, cppResult(0)));
    }

    const kind_of = try arena.alloc(bool, n * n);
    for (classes, 0..) |*a, i| {
        for (classes, 0..) |*b, j| kind_of[i * n + j] = jolt_os_is_kind_of(a.name, b.name) == 1;
    }
    const schema: Schema = .{ .classes = classes, .kind_of = kind_of };

    for (classes) |*c| {
        // How the attributes store their data (every class with objects, also the outside classes as by value members)
        if (!c.abstract and !c.phase9 and !c.primitive) {
            for (c.attributes, 0..) |*a, index| {
                var element: c_int = 0;
                var counts: u64 = 0;
                if (try quarantined(jolt_os_probe_attribute(c.name, @intCast(index), &element, &counts)) != 0) return error.HarnessError;
                a.counts = counts;
                a.element = switch (element) {
                    1 => if (a.data_type == .pointer) .pointer else if (a.primitive == null) .enumeration else .primitive,
                    2 => .reference,
                    3 => .value,
                    else => .unknown,
                };
            }
        }

        // Zolt
        c.zolt = factory.find(c.name);
        if (c.zolt) |rtti| {
            c.waits_for = reachesPending(rtti);
            if (zoltClass(rtti)) |zc| {
                for (c.attributes) |*a| {
                    for (zc.enums) |*e| {
                        if (std.mem.eql(u8, e.name, a.name)) a.enum_values = e;
                    }
                }
            }
        }
    }

    // Classes that reach a Phase 9 class through their bases and attribute classes (C++)
    for (classes, 0..) |*c, i| c.cross = reachesPhase9(&schema, i, 0);
    return schema;
}

fn reachesPhase9(schema: *const Schema, index: usize, depth: u32) bool {
    const c = &schema.classes[index];
    if (c.phase9) return true;
    if (depth > 32) return false;
    for (c.bases) |b| {
        if (schema.find(b)) |j| {
            if (reachesPhase9(schema, j, depth + 1)) return true;
        }
    }
    for (c.attributes) |a| {
        if (a.primitive) |p| {
            if (schema.find(p)) |j| {
                if (j != index and reachesPhase9(schema, j, depth + 1)) return true;
            }
        }
    }
    return false;
}

/// Parse "name hash abstract|creatable bases=[a,b] attributes=[n:type(primitive);...]"
fn parseLine(arena: Allocator, name: [:0]const u8, line: []const u8) !Class {
    var rest = line[name.len + 1 ..];
    const hash = try std.fmt.parseInt(u32, rest[0..8], 16);
    rest = rest[9..];
    const abstract = std.mem.startsWith(u8, rest, "abstract");
    rest = rest[(std.mem.indexOf(u8, rest, "bases=[") orelse return error.HarnessError) + 7 ..];
    const bases_end = std.mem.indexOfScalar(u8, rest, ']') orelse return error.HarnessError;
    var bases: std.ArrayList([]const u8) = .empty;
    if (bases_end > 0) {
        var it = std.mem.splitScalar(u8, rest[0..bases_end], ',');
        while (it.next()) |b| try bases.append(arena, b);
    }
    rest = rest[bases_end..];
    const attributes_start = (std.mem.indexOf(u8, rest, "attributes=[") orelse return error.HarnessError) + 12;
    const attributes_text = rest[attributes_start .. rest.len - 1];
    var attributes: std.ArrayList(Attribute) = .empty;
    if (attributes_text.len > 0) {
        var it = std.mem.splitScalar(u8, attributes_text, ';');
        while (it.next()) |entry| try attributes.append(arena, try parseAttribute(entry));
    }
    return .{
        .name = name,
        .line = line,
        .hash = hash,
        .abstract = abstract,
        .bases = bases.items,
        .attributes = attributes.items,
        .phase9 = isPhase9(name),
        .outside = contains(&outside_classes, name),
        .primitive = contains(&primitive_names, name),
    };
}

/// "name:type(primitive)"
fn parseAttribute(entry: []const u8) !Attribute {
    const colon = std.mem.indexOfScalar(u8, entry, ':') orelse return error.HarnessError;
    const open = std.mem.lastIndexOfScalar(u8, entry, '(') orelse return error.HarnessError;
    const type_text = entry[colon + 1 .. open];
    const primitive = entry[open + 1 .. entry.len - 1];
    var attribute: Attribute = .{
        .name = entry[0..colon],
        .type_text = type_text,
        .primitive = if (std.mem.eql(u8, primitive, "-")) null else primitive,
        .depth = 0,
        .data_type = .invalid,
        .class_name = "",
    };
    var words = std.mem.tokenizeScalar(u8, type_text, ' ');
    while (words.next()) |word| {
        const t = dataTypeOfWord(word) orelse return error.HarnessError;
        switch (t) {
            .array => attribute.depth += 1,
            .instance, .pointer => {
                attribute.data_type = t;
                attribute.class_name = words.next() orelse return error.HarnessError;
                break;
            },
            else => {
                attribute.data_type = t;
                break;
            },
        }
    }
    return attribute;
}

/// The text words of the data types (ObjectStreamTextOut::WriteDataType)
pub const data_type_words = [_]struct { []const u8, OSDataType }{
    .{ "declare", .declare },   .{ "object", .object },   .{ "instance", .instance }, .{ "pointer", .pointer },
    .{ "array", .array },       .{ "uint8", .t_uint8 },   .{ "uint16", .t_uint16 },   .{ "int", .t_int },
    .{ "uint32", .t_uint32 },   .{ "uint64", .t_uint64 }, .{ "float", .t_float },     .{ "double", .t_double },
    .{ "bool", .t_bool },       .{ "string", .t_string }, .{ "float3", .t_float3 },   .{ "float4", .t_float4 },
    .{ "double3", .t_double3 }, .{ "vec3", .t_vec3 },     .{ "dvec3", .t_dvec3 },     .{ "vec4", .t_vec4 },
    .{ "uvec4", .t_uvec4 },     .{ "quat", .t_quat },     .{ "mat44", .t_mat44 },     .{ "dmat44", .t_dmat44 },
};

pub fn dataTypeOfWord(word: []const u8) ?OSDataType {
    for (data_type_words) |e| {
        if (std.mem.eql(u8, e[0], word)) return e[1];
    }
    return null;
}

pub fn wordOfDataType(data_type: OSDataType) []const u8 {
    for (data_type_words) |e| {
        if (e[1] == data_type) return e[0];
    }
    unreachable;
}

/// The data types of the primitives (in the order of ObjectStreamTypes.h)
pub const primitive_data_types = [_]OSDataType{ .t_uint8, .t_uint16, .t_int, .t_uint32, .t_uint64, .t_float, .t_bool, .t_string, .t_float3, .t_vec3, .t_vec4, .t_quat, .t_mat44, .t_double, .t_dvec3, .t_dmat44, .t_double3, .t_float4, .t_uvec4 };
