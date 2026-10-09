//! Port of: Jolt/Physics/Collision/Shape/DecoratedShape.h, Jolt/Physics/Collision/Shape/DecoratedShape.cpp
//! Status: complete
//!
//! Architecture (Docs/Zolt/CollisionArchitecture.md):
//! - D1: DecoratedShape is an abstract class that adds no virtual functions, so it uses `Shape.VTable` (built by
//!   `Shape.vtableFor(T)` with the most derived type T, which the constructors take as their first parameter). Its
//!   implementations of Shape's virtual functions are in `DecoratedShape.impl`; a derived class that calls C++'s
//!   `DecoratedShape::Foo()` uses `DecoratedShape.impl.foo(&self.base, ...)`, and a function that DecoratedShape does
//!   not override resolves to `Shape.impl` (e.g. C++ `DecoratedShape::SaveBinaryState` is `Shape::SaveBinaryState`).
//! - D2: the decorated shape owns its child: `inner_shape: RefConst(Shape)`, released in `destruct` (~DecoratedShape).
//! - D3: DecoratedShapeSettings holds the child settings as `Ref(ShapeSettings)` (Jolt: `RefConst<ShapeSettings>`):
//!   creating the child writes the child's cache, so it must be reachable through a mutable pointer (Rule M). The
//!   constructor that takes the settings is `initFromSettings`, it runs in place on the heap object.
//!
//! Renames: the constructors of DecoratedShapeSettings are `initDefault` (default constructor), `init` (const
//! ShapeSettings *) and `initPtr` (const Shape *); the two DecoratedShape(EShapeSubType[, const Shape *]) constructors
//! are one `init(T, allocator, sub_type, inner_shape)` (`inner_shape` null for the first one).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const ReferenceFile = @import("../../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeList = ShapeFile.ShapeList;
const ShapeRefC = ShapeFile.ShapeRefC;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeType = ShapeFile.ShapeType;
const SubShapeID = @import("SubShapeID.zig").SubShapeID;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;

/// Class that constructs a DecoratedShape
pub const DecoratedShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, DecoratedShapeSettings)

    base: ShapeSettings,

    /// Sub shape (either this or inner_shape_ptr needs to be filled up). Jolt: RefConst<ShapeSettings>, a Ref here
    /// because creating the child writes its cache (Rule M).
    inner_shape: Ref(ShapeSettings) = .empty,
    /// Sub shape (either this or inner_shape needs to be filled up)
    inner_shape_ptr: RefConst(Shape) = .empty,

    /// Default constructor for deserialization, called by the derived settings with their most derived type `T`
    pub fn initDefault(comptime T: type, allocator: Allocator) DecoratedShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator) };
    }

    /// Constructor that decorates another shape (DecoratedShapeSettings(const ShapeSettings *)), adds a reference to
    /// `inner_shape`
    pub fn init(comptime T: type, allocator: Allocator, inner_shape: ?*ShapeSettings) DecoratedShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator), .inner_shape = .init(inner_shape) };
    }

    /// Constructor that decorates another shape (DecoratedShapeSettings(const Shape *)), adds a reference to
    /// `inner_shape_ptr`
    pub fn initPtr(comptime T: type, allocator: Allocator, inner_shape_ptr: ?*const Shape) DecoratedShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(T), allocator), .inner_shape_ptr = .init(inner_shape_ptr) };
    }

    /// ~DecoratedShapeSettings
    pub fn destruct(self: *DecoratedShapeSettings) void {
        self.inner_shape.deinit();
        self.inner_shape_ptr.deinit();
    }

    /// Upcast to the root class
    pub fn asShapeSettings(self: *DecoratedShapeSettings) *ShapeSettings {
        return &self.base;
    }
};

/// Base class for shapes that decorate another shape with extra functionality (e.g. scale, translation etc.)
pub const DecoratedShape = struct {
    /// Abstract class: `Shape.isKindOf(DecoratedShape)` / `Shape.cast(DecoratedShape)` check the shape type
    pub const shape_type: ShapeType = .decorated;

    base: Shape,
    inner_shape: RefConst(Shape) = .empty,

    /// Constructor (DecoratedShape(EShapeSubType) when `inner_shape` is null, DecoratedShape(EShapeSubType, const
    /// Shape *) otherwise), called by the derived classes with their most derived type `T`. Adds a reference to
    /// `inner_shape`.
    pub fn init(comptime T: type, allocator: Allocator, sub_type: ShapeSubType, inner_shape: ?*const Shape) DecoratedShape {
        return .{ .base = .init(Shape.vtableFor(T), allocator, .decorated, sub_type), .inner_shape = .init(inner_shape) };
    }

    /// The part of DecoratedShape(EShapeSubType, const DecoratedShapeSettings &, ShapeResult &) that reads the
    /// settings (the shape was constructed with `init(T, allocator, sub_type, null)` before): creates the child shape.
    /// An error is written to `result` (a child error is copied, C++ `outResult = child_result`), the derived class
    /// checks `result.hasError()` afterwards. Allocation failure is returned.
    pub fn initFromSettings(self: *DecoratedShape, settings: *const DecoratedShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base);

        // Check that there's a shape
        if (settings.inner_shape.get() == null and settings.inner_shape_ptr.get() == null) {
            result.setError("Inner shape is null!");
            return;
        }

        if (settings.inner_shape_ptr.get()) |inner_shape_ptr| {
            // Use provided shape
            self.inner_shape.set(inner_shape_ptr);
        } else {
            // Create child shape
            var child_result = try settings.inner_shape.get().?.createShape(allocator);
            defer child_result.deinit();
            if (!child_result.isValid()) {
                result.assign(&child_result);
                return;
            }
            self.inner_shape.set(child_result.getPtr());
        }
    }

    /// ~DecoratedShape
    pub fn destruct(self: *DecoratedShape) void {
        self.inner_shape.deinit();
    }

    /// Upcast to the root class
    pub fn asShape(self: *const DecoratedShape) *const Shape {
        return &self.base;
    }

    /// Upcast to the root class (creation and restore only)
    pub fn asShapeMut(self: *DecoratedShape) *Shape {
        return &self.base;
    }

    /// Access to the decorated inner shape (set by every constructor that is used to create a usable shape, and by
    /// restoreSubShapeState after a restore)
    pub fn getInnerShape(self: *const DecoratedShape) *const Shape {
        return self.inner_shape.get().?;
    }

    /// DecoratedShape's implementations of Shape's virtual functions (C++ `DecoratedShape::Foo`)
    pub const impl = struct {
        // See Shape::MustBeStatic
        pub fn mustBeStatic(self: *const DecoratedShape) bool {
            return self.getInnerShape().mustBeStatic();
        }

        // See Shape::GetCenterOfMass
        pub fn getCenterOfMass(self: *const DecoratedShape) Vec3 {
            return self.getInnerShape().getCenterOfMass();
        }

        // See Shape::GetSubShapeIDBitsRecursive
        pub fn getSubShapeIDBitsRecursive(self: *const DecoratedShape) u32 {
            return self.getInnerShape().getSubShapeIDBitsRecursive();
        }

        // See Shape::GetLeafShape
        pub fn getLeafShape(self: *const DecoratedShape, sub_shape_id: SubShapeID) Shape.LeafShape {
            return self.getInnerShape().getLeafShape(sub_shape_id);
        }

        // See Shape::GetMaterial
        pub fn getMaterial(self: *const DecoratedShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
            return self.getInnerShape().getMaterial(sub_shape_id);
        }

        // See Shape::GetSupportingFace
        pub fn getSupportingFace(self: *const DecoratedShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
            self.getInnerShape().getSupportingFace(sub_shape_id, direction, scale, center_of_mass_transform, out_vertices);
        }

        // See Shape::GetSubShapeUserData
        pub fn getSubShapeUserData(self: *const DecoratedShape, sub_shape_id: SubShapeID) u64 {
            return self.getInnerShape().getSubShapeUserData(sub_shape_id);
        }

        // See Shape::SaveSubShapeState
        pub fn saveSubShapeState(self: *const DecoratedShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
            // outSubShapes.clear() (releases the references)
            for (out_sub_shapes.items) |*s| s.deinit();
            out_sub_shapes.clearRetainingCapacity();
            try out_sub_shapes.ensureUnusedCapacity(allocator, 1);
            out_sub_shapes.appendAssumeCapacity(self.inner_shape.clone());
        }

        // See Shape::RestoreSubShapeState
        pub fn restoreSubShapeState(self: *DecoratedShape, sub_shapes: []const ShapeRefC) void {
            if (Core.enable_asserts) std.debug.assert(sub_shapes.len == 1); // A corrupt stream can violate this (Jolt's release build then reads the first element regardless)
            self.inner_shape.set(sub_shapes[0].get());
        }

        // See Shape::GetStatsRecursive
        pub fn getStatsRecursive(self: *const DecoratedShape, allocator: Allocator, visited_shapes: *Shape.VisitedShapes) Allocator.Error!Shape.Stats {
            // Get own stats
            var stats = try Shape.impl.getStatsRecursive(&self.base, allocator, visited_shapes);

            // Add child stats
            const child_stats = try self.getInnerShape().getStatsRecursive(allocator, visited_shapes);
            stats.size_bytes +%= child_stats.size_bytes;
            stats.num_triangles +%= child_stats.num_triangles;

            return stats;
        }

        // See Shape::IsValidScale
        pub fn isValidScale(self: *const DecoratedShape, scale: Vec3) bool {
            return self.getInnerShape().isValidScale(scale);
        }

        // See Shape::MakeScaleValid
        pub fn makeScaleValid(self: *const DecoratedShape, scale: Vec3) Vec3 {
            return self.getInnerShape().makeScaleValid(scale);
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the test shapes of TestShapes.zig are registered as User1..User3 through zolt_user_types in the inline tests)

const testing = std.testing;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Quat = @import("../../../Math/Quat.zig").Quat;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RefCount = ReferenceFile.RefCount;
const SubShapeIDCreator = @import("SubShapeID.zig").SubShapeIDCreator;
const ScaleHelpers = @import("ScaleHelpers.zig");
const TestShapes = @import("TestShapes.zig");
const TestBoxShape = TestShapes.TestBoxShape;
const TestBoxShapeSettings = TestShapes.TestBoxShapeSettings;
const TestSphereShape = TestShapes.TestSphereShape;
const TestCompoundShape = TestShapes.TestCompoundShape;
const TestMaterial = TestShapes.TestMaterial;

/// Settings of TestDecoratedShape (a concrete class derived from DecoratedShapeSettings)
const TestDecoratedShapeSettings = struct {
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,
    /// Makes the shape fail after the base class constructor succeeded (like a derived class with its own checks)
    fail_after_base: bool = false,

    fn init(allocator: Allocator, inner_shape: ?*ShapeSettings) TestDecoratedShapeSettings {
        return .{ .base = .init(TestDecoratedShapeSettings, allocator, inner_shape) };
    }

    fn initPtr(allocator: Allocator, inner_shape: ?*const Shape) TestDecoratedShapeSettings {
        return .{ .base = .initPtr(TestDecoratedShapeSettings, allocator, inner_shape) };
    }

    fn create(allocator: Allocator, inner_shape: ?*ShapeSettings) Allocator.Error!*TestDecoratedShapeSettings {
        const self = try allocator.create(TestDecoratedShapeSettings);
        self.* = .init(allocator, inner_shape);
        return self;
    }

    fn asShapeSettings(self: *TestDecoratedShapeSettings) *ShapeSettings {
        return self.base.asShapeSettings();
    }

    fn deinit(self: *TestDecoratedShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    pub fn createShape(self: *TestDecoratedShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(TestDecoratedShape, self, allocator);
    }
};

/// A minimal concrete decorated shape (User4): implements the pure virtual functions by passing them on to the inner
/// shape, everything else comes from DecoratedShape.impl and Shape.impl
const TestDecoratedShape = struct {
    pub const shape_sub_type: ShapeSubType = .user4;
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume };

    base: DecoratedShape,

    pub fn initDefault(allocator: Allocator) TestDecoratedShape {
        return .{ .base = .init(TestDecoratedShape, allocator, shape_sub_type, null) };
    }

    fn init(allocator: Allocator, inner_shape: *const Shape) TestDecoratedShape {
        return .{ .base = .init(TestDecoratedShape, allocator, shape_sub_type, inner_shape) };
    }

    fn create(allocator: Allocator, inner_shape: *const Shape) Allocator.Error!*TestDecoratedShape {
        const self = try allocator.create(TestDecoratedShape);
        self.* = .init(allocator, inner_shape);
        return self;
    }

    pub fn initFromSettings(self: *TestDecoratedShape, settings: *const TestDecoratedShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        try self.base.initFromSettings(&settings.base, result, allocator);
        if (result.hasError())
            return;

        if (settings.fail_after_base) {
            result.setError("Derived class error");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    fn asShape(self: *const TestDecoratedShape) *const Shape {
        return self.base.asShape();
    }

    fn asShapeMut(self: *TestDecoratedShape) *Shape {
        return self.base.asShapeMut();
    }

    fn inner(self: *const TestDecoratedShape) *const Shape {
        return self.base.getInnerShape();
    }

    pub fn getLocalBounds(self: *const TestDecoratedShape) AABox {
        return self.inner().getLocalBounds();
    }

    pub fn getInnerRadius(self: *const TestDecoratedShape) f32 {
        return self.inner().getInnerRadius();
    }

    pub fn getMassProperties(self: *const TestDecoratedShape) MassProperties {
        return self.inner().getMassProperties();
    }

    pub fn getSurfaceNormal(self: *const TestDecoratedShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        return self.inner().getSurfaceNormal(sub_shape_id, local_surface_position);
    }

    pub fn getSubmergedVolume(self: *const TestDecoratedShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        return self.inner().getSubmergedVolume(center_of_mass_transform, scale, surface);
    }

    pub fn castRay(self: *const TestDecoratedShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        return self.inner().castRay(ray, sub_shape_id_creator, hit);
    }

    pub fn castRayCollector(self: *const TestDecoratedShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *ShapeFile.CastRayCollector, shape_filter: *const ShapeFilter) void {
        self.inner().castRayCollector(ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collidePoint(self: *const TestDecoratedShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *ShapeFile.CollidePointCollector, shape_filter: *const ShapeFilter) void {
        self.inner().collidePoint(point, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const TestDecoratedShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.inner().collideSoftBodyVertices(center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index);
    }

    pub fn getTrianglesStart(self: *const TestDecoratedShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        self.inner().getTrianglesStart(context, box, position_com, rotation, scale);
    }

    pub fn getTrianglesNext(self: *const TestDecoratedShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        return self.inner().getTrianglesNext(context, max_triangles_requested, out_triangle_vertices, out_materials);
    }

    pub fn getStats(self: *const TestDecoratedShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TestDecoratedShape), 1);
    }

    pub fn getVolume(self: *const TestDecoratedShape) f32 {
        return self.inner().getVolume();
    }
};

fn saveToBuffer(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

test "DecoratedShape: the overrides pass the calls on to the inner shape" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const material = try TestMaterial.create(allocator, 3);
    var material_ref = RefConst(PhysicsMaterial).init(&material.base);
    defer material_ref.deinit();

    // A compound inner shape, so that the sub shape IDs lead somewhere
    const box = try TestBoxShape.create(allocator, Vec3.init(1, 2, 3), .{ .center_of_mass = Vec3.init(0.5, 0.25, 0), .material = &material.base });
    box.asShapeMut().setUserData(17);
    const sphere = try TestSphereShape.create(allocator, 1.5);
    const compound = try TestCompoundShape.create(allocator, box.asShape(), Vec3.init(-3, 0, 0), sphere.asShape(), Vec3.init(3, 0, 0));
    compound.must_be_static = true;

    var decorated = TestDecoratedShape.init(allocator, compound.asShape());
    decorated.asShape().setEmbedded();
    defer decorated.asShapeMut().deinit();
    const shape = decorated.asShape();
    try testing.expectEqual(@as(u32, 1), compound.asShape().getRefCount()); // Held by the decorated shape

    // Type and casts
    try expect(shape.getType() == .decorated and shape.getSubType() == .user4);
    try expect(shape.isKindOf(DecoratedShape) and shape.isKindOf(TestDecoratedShape) and !shape.isKindOf(TestBoxShape));
    try expect(shape.cast(DecoratedShape) == &decorated.base);
    try expect(shape.cast(DecoratedShape).getInnerShape() == compound.asShape());

    // DecoratedShape.impl
    const id_box = SubShapeIDCreator.pushID(.{}, 0, 1).getID();
    const id_sphere = SubShapeIDCreator.pushID(.{}, 1, 1).getID();
    try expect(shape.mustBeStatic());
    compound.must_be_static = false;
    try expect(!shape.mustBeStatic());
    try expect(shape.getCenterOfMass().eql(compound.asShape().getCenterOfMass()));
    try testing.expectEqual(@as(u32, 1), shape.getSubShapeIDBitsRecursive());
    try expect(shape.getLeafShape(id_box).shape == box.asShape());
    try expect(shape.getLeafShape(id_sphere).shape == sphere.asShape());
    try expect(shape.getLeafShape(id_sphere).remainder.isEmpty());
    try expect(shape.getMaterial(id_box) == &material.base);
    try expect(shape.getMaterial(id_sphere) == PhysicsMaterial.default);
    try testing.expectEqual(@as(u64, 17), shape.getSubShapeUserData(id_box));
    try testing.expectEqual(@as(u64, 0), shape.getSubShapeUserData(id_sphere));
    var face: Shape.SupportingFace = .empty;
    var expected_face: Shape.SupportingFace = .empty;
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.25), Vec3.init(1, 2, 3));
    shape.getSupportingFace(id_box, Vec3.init(1, 0.1, 0), Vec3.replicate(2), transform, &face);
    compound.asShape().getSupportingFace(id_box, Vec3.init(1, 0.1, 0), Vec3.replicate(2), transform, &expected_face);
    try testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice(), expected_face.constSlice()) |a, b| try expect(a.eql(b));

    // The inner shape decides about the scale: the test compound only supports uniform scales
    try expect(shape.isValidScale(Vec3.replicate(-2)) and !shape.isValidScale(Vec3.init(1, 2, 3)));
    try expect(shape.makeScaleValid(Vec3.init(1, 2, 3)).eql(compound.asShape().makeScaleValid(Vec3.init(1, 2, 3))));

    // Shape.impl versions (DecoratedShape does not override them) use the virtual functions of the derived class
    try expect(shape.getWorldSpaceBounds(transform, Vec3.one()).eql(compound.asShape().getLocalBounds().transformed(transform)));
    var sub_ts = shape.getSubShapeTransformedShape(id_box, Vec3.init(1, 2, 3), Quat.identity(), Vec3.one());
    defer sub_ts.transformed_shape.deinit();
    try expect(sub_ts.transformed_shape.shape.get() == shape and sub_ts.remainder.isEmpty());

    // The overrides of the test class
    try testing.expectEqual(compound.asShape().getVolume(), shape.getVolume());
    var hit: RayCastResult = .{};
    try expect(shape.castRay(.init(Vec3.init(10, 0, 0), Vec3.init(-20, 0, 0)), .{}, &hit));
    try expect(hit.sub_shape_id2.eql(id_sphere));
}

test "DecoratedShape: construction from settings, Jolt's error texts and reference counts" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // No inner shape (the default constructor and the constructors with null)
    {
        var default_settings: TestDecoratedShapeSettings = .{ .base = .initDefault(TestDecoratedShapeSettings, allocator) };
        defer default_settings.deinit();
        var null_settings = TestDecoratedShapeSettings.init(allocator, null);
        defer null_settings.deinit();
        var null_ptr_settings = TestDecoratedShapeSettings.initPtr(allocator, null);
        defer null_ptr_settings.deinit();
        for ([_]*TestDecoratedShapeSettings{ &default_settings, &null_settings, &null_ptr_settings }) |settings| {
            var result = try settings.asShapeSettings().createShape(allocator);
            defer result.deinit();
            try testing.expectEqualStrings("Inner shape is null!", result.getError());
        }
    }

    // The child error is forwarded
    {
        const bad_child = try TestBoxShapeSettings.create(allocator, Vec3.init(1, -1, 1), null);
        var settings = TestDecoratedShapeSettings.init(allocator, bad_child.asShapeSettings());
        defer settings.deinit();
        try testing.expectEqual(@as(u32, 1), bad_child.base.getRefCount());
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid half extent", result.getError());
    }

    // An error of the derived class after the base class succeeded: the shape (and its reference to the child) is
    // destroyed, the child stays cached in its settings
    {
        const child = try TestBoxShapeSettings.create(allocator, Vec3.one(), null);
        var settings = TestDecoratedShapeSettings.init(allocator, child.asShapeSettings());
        defer settings.deinit();
        settings.fail_after_base = true;
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Derived class error", result.getError());
        try testing.expectEqual(@as(u32, 1), child.base.cached_result.getPtr().?.getRefCount()); // Only the cache
    }

    // Child settings: the child shape is created (and cached by the child settings), user data is copied
    {
        const child = try TestBoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), null);
        var settings = TestDecoratedShapeSettings.init(allocator, child.asShapeSettings());
        defer settings.deinit();
        settings.asShapeSettings().user_data = 99;
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const shape = result.getPtr().?;
        try testing.expectEqual(@as(u64, 99), shape.getUserData());
        const inner = shape.cast(DecoratedShape).getInnerShape();
        try expect(inner == child.base.cached_result.getPtr().?);
        try testing.expectEqual(@as(u32, 2), inner.getRefCount()); // Child cache + decorated shape
        try expect(inner.cast(TestBoxShape).half_extent.eql(Vec3.init(1, 2, 3)));
        try testing.expectEqual(@as(u32, 2), shape.getRefCount()); // Cache + result
    }

    // A concrete shape: inner_shape_ptr wins over inner_shape (both are set here)
    {
        const box = try TestBoxShape.create(allocator, Vec3.one(), .{});
        var box_ref = RefConst(Shape).init(box.asShape());
        defer box_ref.deinit();
        const unused_child = try TestBoxShapeSettings.create(allocator, Vec3.init(-1, 1, 1), null);
        var settings = TestDecoratedShapeSettings.initPtr(allocator, box.asShape());
        defer settings.deinit();
        settings.base.inner_shape.set(unused_child.asShapeSettings());
        try testing.expectEqual(@as(u32, 2), box.asShape().getRefCount()); // box_ref + settings
        {
            var result = try settings.asShapeSettings().createShape(allocator);
            defer result.deinit();
            try expect(result.getPtr().?.cast(DecoratedShape).getInnerShape() == box.asShape());
            try testing.expectEqual(@as(u32, 3), box.asShape().getRefCount()); // + the decorated shape
            try expect(unused_child.base.cached_result.isEmpty()); // Not created
        }
        settings.asShapeSettings().clearCachedResult(); // Destroys the decorated shape
        try testing.expectEqual(@as(u32, 2), box.asShape().getRefCount());
    }

    // Heap settings own their child settings; a heap decorated shape owns its inner shape
    {
        const child = try TestBoxShapeSettings.create(allocator, Vec3.one(), null);
        const settings = try TestDecoratedShapeSettings.create(allocator, child.asShapeSettings());
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        defer settings_ref.deinit();
        try testing.expectEqual(@as(u32, 1), child.base.getRefCount());

        const sphere = try TestSphereShape.create(allocator, 1.0);
        const decorated = try TestDecoratedShape.create(allocator, sphere.asShape());
        var decorated_ref = RefConst(Shape).init(decorated.asShape());
        try testing.expectEqual(@as(u32, 1), sphere.asShape().getRefCount());
        decorated_ref.deinit(); // Releases (and destroys) the sphere too, leak checked
    }
}

test "DecoratedShape: out of memory while creating the inner shape is returned, not cached" {
    const allocator = testing.allocator;

    const child = try TestBoxShapeSettings.create(allocator, Vec3.one(), null);
    var settings = TestDecoratedShapeSettings.init(allocator, child.asShapeSettings());
    defer settings.deinit();

    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var result = settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(settings.base.base.cached_result.isEmpty());
            child.asShapeSettings().clearCachedResult();
            continue;
        };
        defer result.deinit();
        try testing.expect(result.isValid() and !failing.has_induced_failure);
        try testing.expectEqual(@as(usize, 2), fail_index); // The decorated shape and its child
        child.asShapeSettings().clearCachedResult();
        settings.asShapeSettings().clearCachedResult();
        break;
    }
}

test "DecoratedShape: binary state, sub shape state and stats" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const box = try TestBoxShape.create(allocator, Vec3.init(1, 2, 3), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const decorated = try TestDecoratedShape.create(allocator, box.asShape());
    decorated.asShapeMut().setUserData(1234);
    var decorated_ref = RefConst(Shape).init(decorated.asShape());
    defer decorated_ref.deinit();
    const shape = decorated.asShape();

    // DecoratedShape has no binary state of its own: Shape's (sub type, user data)
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(shape, &buffer);
    try testing.expectEqual(@as(usize, 1 + 8), bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.user4), bytes[0]);

    // Sub shape state: the list is cleared (and its references released) first
    var sub_shapes: ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    const sphere = try TestSphereShape.create(allocator, 1.0);
    try sub_shapes.append(allocator, .init(sphere.asShape()));
    try shape.saveSubShapeState(allocator, &sub_shapes); // Destroys the sphere (leak checked)
    try testing.expectEqual(@as(usize, 1), sub_shapes.items.len);
    try expect(sub_shapes.items[0].get() == box.asShape());
    try testing.expectEqual(@as(u32, 3), box.asShape().getRefCount());

    // Restore into a default constructed shape
    var restored = TestDecoratedShape.initDefault(allocator);
    restored.asShape().setEmbedded();
    defer restored.asShapeMut().deinit();
    {
        var reader: std.Io.Reader = .fixed(bytes[1..]); // Shape.restoreFromBinaryState reads the sub type
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        try restored.asShapeMut().restoreBinaryState(in.streamIn());
    }
    try testing.expectEqual(@as(u64, 1234), restored.asShape().getUserData());
    try expect(restored.base.inner_shape.get() == null);
    restored.asShapeMut().restoreSubShapeState(sub_shapes.items);
    try expect(restored.base.getInnerShape() == box.asShape());
    try testing.expectEqual(@as(u32, 4), box.asShape().getRefCount());

    // Stats: own stats + the inner shape's, a shape that was visited before counts 0 bytes
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    const stats = try shape.getStatsRecursive(allocator, &visited);
    try testing.expectEqual(@as(usize, @sizeOf(TestDecoratedShape) + @sizeOf(TestBoxShape)), stats.size_bytes);
    try testing.expectEqual(@as(u32, 1 + 12), stats.num_triangles);
    const restored_stats = try restored.asShape().getStatsRecursive(allocator, &visited);
    try testing.expectEqual(@as(usize, @sizeOf(TestDecoratedShape)), restored_stats.size_bytes); // The box was visited
    try testing.expectEqual(@as(u32, 1 + 12), restored_stats.num_triangles);
    try testing.expectEqual(@as(u32, 3), visited.count());

    // Out of memory in getStatsRecursive and saveSubShapeState is returned
    var visited2: Shape.VisitedShapes = .empty;
    defer visited2.deinit(allocator);
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, shape.getStatsRecursive(failing.allocator(), &visited2));
    var empty_list: ShapeList = .empty;
    try testing.expectError(error.OutOfMemory, shape.saveSubShapeState(failing.allocator(), &empty_list));
    try testing.expectEqual(@as(u32, 4), box.asShape().getRefCount()); // The failed append released its clone

    // The scale helpers of the inner shape (TestBoxShape: Shape.impl, any non zero scale)
    try expect(shape.isValidScale(Vec3.init(1, -2, 3)) and !shape.isValidScale(Vec3.init(1, 0, 3)));
    try expect(shape.makeScaleValid(Vec3.init(0, -2, 3)).eql(Vec3.init(ScaleHelpers.min_scale, -2, 3)));
    try testing.expectEqual(RefCount.embedded, restored.asShape().getRefCount());
}
