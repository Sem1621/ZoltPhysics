//! Port of: UnitTests/ObjectStream/ObjectStreamTest.cpp
//!
//! The test classes are `pub`: the ObjectStream parity test (ZoltParity/ObjectStream/ObjectStreamParity.zig) imports
//! this file as the module "objectstream_test" and compares their streams with the C++ library's bytes. This file
//! therefore imports nothing but "std" and "zolt".
//!
//! Zolt layout of the classes (Jolt: `TestSerializable : TestSerializableBase, TestSerializableBase2`):
//! - TestSerializableBase is a pattern A root (Jolt's RefTarget with a virtual destructor, JPH_DECLARE_SERIALIZABLE_
//!   VIRTUAL_BASE) with the `rtti` vtable entry;
//! - TestSerializableBase2 is the second base, the field `base2`. Zolt's pattern A has one `base` chain, so the second
//!   base has no vtable and is declared `.non_virtual` (GetRTTI through a TestSerializableBase2 pointer is static);
//! - `mReference` / `mReferenceConst` are `Ref(TestSerializableBase)` / `RefConst(TestSerializableBase)` declared with
//!   `.attrAs(..., TestSerializable)`: Zolt's Ref needs the class that has addRef / release (the root), the stream
//!   declares Jolt's `instance TestSerializable`.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const ObjectStreamIn = zolt.ObjectStreamIn;
const ObjectStreamOut = zolt.ObjectStreamOut;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RefCount = zolt.RefCount;
const StaticArray = zolt.StaticArray;
const virtual = zolt.virtual;
const Vec3 = zolt.Vec3;
const DVec3 = zolt.DVec3;
const Vec4 = zolt.Vec4;
const UVec4 = zolt.UVec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const DMat44 = zolt.DMat44;
const Float3 = zolt.Float3;
const Float4 = zolt.Float4;
const Double3 = zolt.Double3;

pub const TestEnum = enum(i32) {
    a,
    b,
    c,
};

pub const TestSerializableBase = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL_BASE / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializableBase)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "TestSerializableBase",
        .kind = .virtual,
        .attributes = &.{
            .attr("mUInt8", .uint8),
            .attr("mUInt16", .uint16),
            .attr("mInt", .int),
            .attr("mUInt32", .uint32),
            .attr("mUInt64", .uint64),
            .attr("mFloat", .float),
            .attr("mDouble", .double),
            .attr("mBool", .bool),
            .attr("mFloat3", .float3),
            .attr("mFloat4", .float4),
            .attr("mDouble3", .double3),
            .attr("mQuat", .quat),
            .attr("mVec3", .vec3),
            .attr("mDVec3", .dvec3),
            .attr("mVec4", .vec4),
            .attr("mUVec4", .uvec4),
            .attr("mMat44", .mat44),
            .attr("mDMat44", .dmat44),
            .attr("mString", .string),
        },
    });

    pub const VTable = struct {
        /// JPH_RTTI of the most derived class (GetRTTI)
        rtti: *const RTTI,
        deinit: *const fn (self: *TestSerializableBase) void,
        destroy: *const fn (self: *TestSerializableBase) void,
    };

    pub const overrides = .{};

    vtable: *const VTable,
    ref_count: RefCount = .{},
    allocator: Allocator,

    uint8: u8 = 0,
    uint16: u16 = 0,
    int: i32 = 0,
    uint32: u32 = 0,
    uint64: u64 = 0,
    float: f32 = 0,
    double: f64 = 0,
    bool: bool = false,
    float3: Float3 = .init(0, 0, 0),
    float4: Float4 = .init(0, 0, 0, 0),
    double3: Double3 = .init(0, 0, 0),
    quat: Quat = Quat.identity(),
    vec3: Vec3 = Vec3.zero(),
    dvec3: DVec3 = DVec3.zero(),
    vec4: Vec4 = Vec4.zero(),
    uvec4: UVec4 = UVec4.zero(),
    mat44: Mat44 = Mat44.identity(),
    dmat44: DMat44 = DMat44.identity(),
    /// Owned by `allocator`
    string: []u8 = &.{},

    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestSerializableBase {
        const self = try allocator.create(TestSerializableBase);
        self.* = .{ .vtable = virtual.vtablePtr(VTable, TestSerializableBase), .allocator = allocator };
        return self;
    }

    pub fn destruct(self: *TestSerializableBase) void {
        self.allocator.free(self.string);
    }

    pub fn addRef(self: *const TestSerializableBase) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const TestSerializableBase) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self));
    }

    pub fn getRefCount(self: *const TestSerializableBase) u32 {
        return self.ref_count.get();
    }
};

pub const TestSerializableBase2 = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL_BASE / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializableBase2): non virtual
    /// in Zolt (the second base has no vtable, see the file comment)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "TestSerializableBase2",
        .kind = .non_virtual,
        .attributes = &.{.attr("mBase2", .base2)},
    });

    base2: u32 = 0,
};

pub const TestSerializable = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestSerializable)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "TestSerializable",
        .kind = .virtual,
        .bases = &.{ TestSerializableBase, TestSerializableBase2 },
        .attributes = &.{
            .enumAttr("mEnum", .@"enum"),
            .attr("mIntVector", .int_vector),
            .attr("mBoolVector", .bool_vector),
            .attr("mFloatVector", .float_vector),
            .attr("mArrayOfVector", .array_of_vector),
            .attr("mVectorOfVector", .vector_of_vector),
            .attr("mPointer", .pointer),
            .attrAs("mReference", .reference, TestSerializable),
            .attrAs("mReferenceConst", .reference_const, TestSerializable),
        },
    });

    pub const overrides = .{};

    base: TestSerializableBase,
    base2: TestSerializableBase2 = .{},

    @"enum": TestEnum = .a,
    int_vector: std.ArrayList(i32) = .empty,
    bool_vector: StaticArray(bool, 10) = .empty,
    float_vector: [3]f32 = .{ 0, 0, 0 },
    array_of_vector: [3]std.ArrayList(f32) = @splat(.empty),
    vector_of_vector: std.ArrayList(std.ArrayList(i32)) = .empty,
    pointer: ?*TestSerializable = null,
    reference: Ref(TestSerializableBase) = .empty,
    reference_const: RefConst(TestSerializableBase) = .empty,

    /// new TestSerializable (reference count 0)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestSerializable {
        const self = try allocator.create(TestSerializable);
        self.* = .{ .base = .{ .vtable = virtual.vtablePtr(TestSerializableBase.VTable, TestSerializable), .allocator = allocator } };
        return self;
    }

    pub fn destruct(self: *TestSerializable) void {
        const allocator = self.base.allocator;
        self.int_vector.deinit(allocator);
        for (&self.array_of_vector) |*v| v.deinit(allocator);
        for (self.vector_of_vector.items) |*v| v.deinit(allocator);
        self.vector_of_vector.deinit(allocator);
        self.reference.deinit();
        self.reference_const.deinit();
    }

    /// delete (reference count 0)
    pub fn destroy(self: *TestSerializable) void {
        self.base.vtable.destroy(&self.base);
    }
};

/// CreateTestObject of ObjectStreamTest.cpp
pub fn createTestObject(allocator: Allocator) Allocator.Error!*TestSerializable {
    const test_object = try TestSerializable.createDefault(allocator);
    errdefer test_object.destroy();
    const b = &test_object.base;
    b.uint8 = 0xff;
    b.uint16 = 0xffff;
    b.int = -1;
    b.uint32 = 0xf1f2f3f4;
    b.uint64 = 0xf5f6f7f8f9fafbfc;
    b.float = 0.12345;
    b.double = std.math.floatEps(f64); // DBL_EPSILON
    b.bool = true;
    b.float3 = .init(9, 10, 11);
    b.float4 = .init(11, 9, 10, 12);
    b.double3 = .init(10, 11, 12);
    b.vec3 = .init(6, 7, 8);
    b.dvec3 = .init(7, 8, 9);
    b.vec4 = .init(9, 10, 11, 12);
    b.uvec4 = .init(12, 11, 9, 10);
    b.quat = Quat.rotation(Vec3.axisX(), 0.1234);
    b.mat44 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.4567), .init(13, 14, 15));
    b.dmat44 = DMat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.789), .init(20, 21, 22));
    b.string = try allocator.dupe(u8, "\"test string\"");
    test_object.@"enum" = .b;
    try test_object.int_vector.appendSlice(allocator, &.{ 1, 2, 3, 4, 5 });
    test_object.bool_vector.append(true);
    test_object.bool_vector.append(false);
    test_object.bool_vector.append(true);
    test_object.float_vector = .{ 1.0, 2.0, 3.0 };
    try test_object.array_of_vector[0].appendSlice(allocator, &.{ 1, 2, 3 });
    try test_object.array_of_vector[1].appendSlice(allocator, &.{ 4, 5 });
    try test_object.array_of_vector[2].appendSlice(allocator, &.{ 6, 7, 8, 9 });
    for ([_][]const i32{ &.{ 10, 11 }, &.{ 12, 13, 14 }, &.{ 15, 16, 17, 18 } }) |values| {
        try test_object.vector_of_vector.ensureUnusedCapacity(allocator, 1);
        var list: std.ArrayList(i32) = .empty;
        try list.appendSlice(allocator, values);
        test_object.vector_of_vector.appendAssumeCapacity(list);
    }
    test_object.base2.base2 = 0x9876;

    const test2 = try TestSerializable.createDefault(allocator);
    test2.base.float = 4.5;
    test_object.pointer = test2;
    test_object.reference = .init(&test2.base);
    test_object.reference_const = .init(&test2.base);
    return test_object;
}

/// The factory of the test: Jolt registers TestSerializable after RegisterTypes() (Phase 8 foundation: with
/// RegisterTypes.factory_types)
pub const factory: Factory = .init(&.{TestSerializable});

fn compareObjects(input: *const TestSerializable, output: *const TestSerializable) !void {
    const testing = std.testing;
    const in = &input.base;
    const out = &output.base;
    try testing.expectEqual(in.uint8, out.uint8);
    try testing.expectEqual(in.uint16, out.uint16);
    try testing.expectEqual(in.int, out.int);
    try testing.expectEqual(in.uint32, out.uint32);
    try testing.expectEqual(in.uint64, out.uint64);
    try testing.expectEqual(in.float, out.float);
    try testing.expectEqual(in.double, out.double);
    try testing.expectEqual(in.bool, out.bool);
    try testing.expectEqual(in.float3, out.float3);
    try testing.expectEqual(in.float4, out.float4);
    try testing.expectEqual(in.double3, out.double3);
    try testing.expect(in.quat.eql(out.quat));
    try testing.expect(in.vec3.eql(out.vec3));
    try testing.expect(in.dvec3.eql(out.dvec3));
    try testing.expect(in.vec4.eql(out.vec4));
    try testing.expect(in.uvec4.eql(out.uvec4));
    try testing.expect(in.mat44.eql(out.mat44));
    try testing.expect(in.dmat44.eql(out.dmat44));
    try testing.expectEqualStrings(in.string, out.string);
    try testing.expectEqual(input.@"enum", output.@"enum");
    try testing.expectEqualSlices(i32, input.int_vector.items, output.int_vector.items);
    try testing.expectEqualSlices(bool, input.bool_vector.constSlice(), output.bool_vector.constSlice());

    for (input.float_vector, output.float_vector) |a, b|
        try testing.expectEqual(a, b);

    for (input.array_of_vector, output.array_of_vector) |a, b|
        try testing.expectEqualSlices(f32, a.items, b.items);

    try testing.expectEqual(input.vector_of_vector.items.len, output.vector_of_vector.items.len);
    for (input.vector_of_vector.items, output.vector_of_vector.items) |a, b|
        try testing.expectEqualSlices(i32, a.items, b.items);

    const output_reference: ?*const TestSerializableBase = output.reference.ptr;
    try testing.expectEqual(@as(?*const TestSerializableBase, if (output.pointer) |p| &p.base else null), output_reference);
    try testing.expectEqual(output_reference, output.reference_const.ptr);

    if (input.pointer == null) {
        try testing.expect(output.pointer == null);
    } else {
        try testing.expect(input.pointer != output.pointer);
        try compareObjects(input.pointer.?, output.pointer.?);
        try testing.expectEqual(@as(u32, 2), output.reference.ptr.?.getRefCount());
        try testing.expectEqual(@as(u32, 2), output.reference_const.ptr.?.getRefCount());
    }

    try testing.expectEqual(input.base2.base2, output.base2.base2);
}

fn loadSave(stream_type: zolt.ObjectStream.StreamType, check_casts: bool) !void {
    const allocator = std.testing.allocator;
    const test_object = try createTestObject(allocator);
    defer test_object.destroy();

    var stream: std.Io.Writer.Allocating = .init(allocator);
    defer stream.deinit();
    try ObjectStreamOut.sWriteObject(TestSerializable, allocator, &stream.writer, stream_type, test_object);

    var reader: std.Io.Reader = .fixed(stream.written());
    const test_out = try ObjectStreamIn.sReadObject(TestSerializable, allocator, &reader, .{ .factory = &factory });
    defer test_out.destroy();

    if (check_casts) {
        // Check that DynamicCast returns the right offsets
        try std.testing.expectEqual(@as(?*TestSerializable, test_out), RTTI.dynamicCast(TestSerializable, TestSerializable, test_out));
        try std.testing.expectEqual(@as(?*TestSerializableBase, &test_out.base), RTTI.dynamicCast(TestSerializableBase, TestSerializable, test_out));
        try std.testing.expectEqual(@as(?*TestSerializableBase2, &test_out.base2), RTTI.dynamicCast(TestSerializableBase2, TestSerializable, test_out));
    }

    try compareObjects(test_object, test_out);
}

test "TestObjectStreamLoadSaveText" {
    try loadSave(.text, true);
}

test "TestObjectStreamLoadSaveBinary" {
    try loadSave(.binary, false);
}
