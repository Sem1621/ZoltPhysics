//! Port of: Jolt/Physics/Collision/Shape/EmptyShape.h, Jolt/Physics/Collision/Shape/EmptyShape.cpp
//! Status: complete
//!
//! A concrete shape that derives from Shape directly (D1: `base: Shape`, built with `Shape.vtableFor(EmptyShape)`),
//! following the porter template of Docs/Zolt/CollisionArchitecture.md (section 2). The C++ constructors are
//! `initDefault` (`EmptyShape()`, used by restoreFromBinaryState), `init` (`EmptyShape(inCenterOfMass)`),
//! `initFromSettings` (D3) and `create` (`new EmptyShape(inCenterOfMass)`).
//!
//! Like Jolt, EmptyShape does not override SaveBinaryState / RestoreBinaryState: the center of mass is not part of the
//! binary state (a restored EmptyShape has its center of mass at the origin).
//!
//! The lambdas `collide_empty` / `cast_empty` of EmptyShape::sRegister are the functions `collideEmpty` / `castEmpty`.
//! JPH_DEBUG_RENDERER (Draw) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Class that constructs an EmptyShape
pub const EmptyShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, EmptyShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: ShapeSettings,
    /// Determines the center of mass for this shape
    center_of_mass: Vec3 = Vec3.zero(),

    /// Default constructor (EmptyShapeSettings() = default)
    pub fn initDefault(allocator: Allocator) EmptyShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(EmptyShapeSettings), allocator) };
    }

    /// EmptyShapeSettings(inCenterOfMass) (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, center_of_mass: Vec3) EmptyShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(EmptyShapeSettings), allocator), .center_of_mass = center_of_mass };
    }

    /// new EmptyShapeSettings(inCenterOfMass): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, center_of_mass: Vec3) Allocator.Error!*EmptyShapeSettings {
        const self = try allocator.create(EmptyShapeSettings);
        self.* = .init(allocator, center_of_mass);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *EmptyShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *EmptyShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *EmptyShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(EmptyShape, self, allocator);
    }
};

/// An empty shape that has no volume and collides with nothing.
///
/// Possible use cases:
/// - As a placeholder for a shape that will be created later. E.g. if you first need to create a body and only then know what shape it will have.
/// - If you need a kinematic body to attach a constraint to, but you don't want the body to collide with anything.
///
/// Note that, if possible, you should also put your body in an ObjectLayer that doesn't collide with anything.
/// This ensures that collisions will be filtered out at broad phase level instead of at narrow phase level, this is more efficient.
pub const EmptyShape = struct {
    /// Concrete class: `Shape.cast(EmptyShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .empty;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume, .isValidScale };

    base: Shape,
    center_of_mass: Vec3 = Vec3.zero(),

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// EmptyShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) EmptyShape {
        return .{ .base = .init(Shape.vtableFor(EmptyShape), allocator, .empty, shape_sub_type) };
    }

    /// EmptyShape(inCenterOfMass) (on the stack / as a member: `asShape().setEmbedded()` before taking references,
    /// `asShapeMut().deinit()` at the end)
    pub fn init(allocator: Allocator, center_of_mass: Vec3) EmptyShape {
        return .{ .base = .init(Shape.vtableFor(EmptyShape), allocator, .empty, shape_sub_type), .center_of_mass = center_of_mass };
    }

    /// EmptyShape(const EmptyShapeSettings &inSettings, ShapeResult &outResult)
    pub fn initFromSettings(self: *EmptyShape, settings: *const EmptyShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base);
        self.center_of_mass = settings.center_of_mass;

        result.set(.init(self.asShapeMut()));
    }

    /// new EmptyShape(inCenterOfMass) (`new EmptyShape()` is `create(allocator, Vec3.zero())`): reference count 0, put
    /// it in a Ref / RefConst
    pub fn create(allocator: Allocator, center_of_mass: Vec3) Allocator.Error!*EmptyShape {
        const self = try allocator.create(EmptyShape);
        self.* = .init(allocator, center_of_mass);
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const EmptyShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *EmptyShape) *Shape {
        return &self.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See: Shape
    pub fn getCenterOfMass(self: *const EmptyShape) Vec3 {
        return self.center_of_mass;
    }

    pub fn getLocalBounds(self: *const EmptyShape) AABox {
        _ = self;
        return .init(Vec3.zero(), Vec3.zero());
    }

    pub fn getSubShapeIDBitsRecursive(self: *const EmptyShape) u32 {
        _ = self;
        return 0;
    }

    pub fn getInnerRadius(self: *const EmptyShape) f32 {
        _ = self;
        return 0.0;
    }

    pub fn getMassProperties(self: *const EmptyShape) MassProperties {
        _ = self;
        var mass_properties: MassProperties = .{};
        mass_properties.mass = 1.0;
        mass_properties.inertia = Mat44.identity();
        return mass_properties;
    }

    pub fn getMaterial(self: *const EmptyShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        _ = .{ self, sub_shape_id };
        return PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const EmptyShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id, local_surface_position };
        return Vec3.zero();
    }

    pub fn getSubmergedVolume(self: *const EmptyShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        return .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    // TODO(debug_renderer): Draw (JPH_DEBUG_RENDERER, draws a marker at the center of mass)

    pub fn castRay(self: *const EmptyShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        _ = .{ self, ray, sub_shape_id_creator, hit };
        return false;
    }

    pub fn castRayCollector(self: *const EmptyShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Do nothing
        _ = .{ self, ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter };
    }

    pub fn collidePoint(self: *const EmptyShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Do nothing
        _ = .{ self, point, sub_shape_id_creator, collector, shape_filter };
    }

    pub fn collideSoftBodyVertices(self: *const EmptyShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        // Do nothing
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
    }

    pub fn getTrianglesStart(self: *const EmptyShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        // Do nothing
        _ = .{ self, context, box, position_com, rotation, scale };
    }

    pub fn getTrianglesNext(self: *const EmptyShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        return 0;
    }

    pub fn getStats(self: *const EmptyShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(EmptyShape), 0);
    }

    pub fn getVolume(self: *const EmptyShape) f32 {
        _ = self;
        return 0.0;
    }

    pub fn isValidScale(self: *const EmptyShape, scale: Vec3) bool {
        _ = .{ self, scale };
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.empty);
        f.construct = ShapeFunctions.constructor(EmptyShape);
        f.color = Color.black;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.empty, s, collideEmpty);
            r.registerCollideShape(s, .empty, collideEmpty);

            r.registerCastShape(.empty, s, castEmpty);
            r.registerCastShape(s, .empty, castEmpty);
        }
    }

    /// The lambda `collide_empty` of sRegister
    fn collideEmpty(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        // Do Nothing
        _ = .{ shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter };
    }

    /// The lambda `cast_empty` of sRegister
    fn castEmpty(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        // Do nothing
        _ = .{ shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt has no unit tests for EmptyShape, the bit exact comparison with Jolt is in
// ZoltParity/Physics/PlaneEmptyParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const math = @import("../../../Math/Math.zig");
const CollisionDispatch = @import("../CollisionDispatch.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const SphereShape = @import("SphereShape.zig").SphereShape;
const BoxShape = @import("BoxShape.zig").BoxShape;
const PlaneShape = @import("PlaneShape.zig").PlaneShape;
const ScaleHelpers = @import("ScaleHelpers.zig");

fn saveToBuffer(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!ShapeResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return Shape.restoreFromBinaryState(allocator, in.streamIn());
}

test "EmptyShape: settings, cached results and out of memory" {
    const allocator = testing.allocator;

    // Default constructor: center of mass at the origin
    var default_settings = EmptyShapeSettings.initDefault(allocator);
    defer default_settings.deinit();
    var default_result = try default_settings.asShapeSettings().createShape(allocator);
    defer default_result.deinit();
    try testing.expect(default_result.isValid());
    try testing.expect(default_result.getPtr().?.getCenterOfMass().eql(Vec3.zero()));

    // Heap settings: center of mass and user data are passed to the shape, the result is cached
    const settings = try EmptyShapeSettings.create(allocator, Vec3.init(1, 2, 3));
    var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
    defer settings_ref.deinit();
    settings.asShapeSettings().user_data = 42;
    var result = try settings.createShape(allocator);
    defer result.deinit();
    var result2 = try settings.createShape(allocator);
    defer result2.deinit();
    try testing.expect(result.getPtr() == result2.getPtr());
    const empty = result.getPtr().?.cast(EmptyShape);
    try testing.expect(empty.asShape().getCenterOfMass().eql(Vec3.init(1, 2, 3)));
    try testing.expectEqual(@as(u64, 42), empty.asShape().getUserData());
    try testing.expectEqual(ShapeSubType.empty, empty.asShape().getSubType());
    try testing.expect(empty.asShape().getType() == .empty);

    // Out of memory while creating the shape is returned and not cached, a later call succeeds
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var oom_settings = EmptyShapeSettings.init(allocator, Vec3.init(1, 2, 3));
        defer oom_settings.deinit();
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var r = oom_settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(oom_settings.base.cached_result.isEmpty());
            continue;
        };
        defer r.deinit();
        try testing.expect(r.isValid());
        try testing.expectEqual(@as(usize, 1), fail_index); // Only the shape is allocated
        break;
    }
}

test "EmptyShape: properties and queries (collides with nothing)" {
    const allocator = testing.allocator;

    var empty = EmptyShape.init(allocator, Vec3.init(1, 2, 3));
    empty.asShape().setEmbedded();
    defer empty.asShapeMut().deinit();
    const shape = empty.asShape();

    try testing.expect(shape.getCenterOfMass().eql(Vec3.init(1, 2, 3)));
    try testing.expect(shape.getLocalBounds().eql(.init(Vec3.zero(), Vec3.zero())));
    const transform = Mat44.translation(Vec3.init(4, 5, 6));
    try testing.expect(shape.getWorldSpaceBounds(transform, Vec3.replicate(2.0)).eql(.init(Vec3.init(4, 5, 6), Vec3.init(4, 5, 6)))); // Shape's version
    try testing.expect(!shape.mustBeStatic());
    try testing.expectEqual(@as(u32, 0), shape.getSubShapeIDBitsRecursive());
    try testing.expectEqual(@as(f32, 0.0), shape.getInnerRadius());
    const p = shape.getMassProperties();
    try testing.expectEqual(@as(f32, 1.0), p.mass);
    try testing.expect(p.inertia.eql(Mat44.identity()));
    try testing.expect(shape.getMaterial(.{ .value = 5 }) == PhysicsMaterial.default);
    try testing.expect(shape.getSurfaceNormal(.empty, Vec3.init(1, 0, 0)).eql(Vec3.zero()));
    try testing.expectEqual(@as(f32, 0.0), shape.getVolume());
    try testing.expectEqual(@as(usize, @sizeOf(EmptyShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);
    try testing.expect(shape.isValidScale(Vec3.zero())); // Any scale
    try testing.expect(shape.makeScaleValid(Vec3.zero()).eql(Vec3.replicate(ScaleHelpers.min_scale))); // Shape's version
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.axisX(), Vec3.one(), transform, &face);
    try testing.expectEqual(@as(u32, 0), face.len); // Shape's version
    const submerged = shape.getSubmergedVolume(transform, Vec3.one(), Plane.init(Vec3.axisY(), 100.0));
    try testing.expectEqual(@as(f32, 0.0), submerged.total_volume);
    try testing.expectEqual(@as(f32, 0.0), submerged.submerged_volume);
    try testing.expect(submerged.center_of_buoyancy.eql(Vec3.zero()));

    // Ray casts and collide point: nothing
    var hit: RayCastResult = .{};
    try testing.expect(!shape.castRay(.init(Vec3.init(-1, 0, 0), Vec3.init(2, 0, 0)), .{}, &hit));
    try testing.expectEqual(@as(f32, 1.0 + math.flt_epsilon), hit.fraction);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    shape.castRayCollector(.init(Vec3.init(-1, 0, 0), Vec3.init(2, 0, 0)), &.{}, .{}, &hits.base, &.{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.zero(), .{}, &points.base, &.{});
    try testing.expectEqual(@as(usize, 0), points.hits.items.len);

    // Soft body vertices: untouched
    var positions = [_]Vec3{Vec3.zero()};
    var inv_masses = [_]f32{1};
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)};
    var penetrations = [_]f32{-math.flt_max};
    var indices = [_]i32{-1};
    const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));
    shape.collideSoftBodyVertices(Mat44.identity(), Vec3.one(), &vertices, 1, 3);
    try testing.expectEqual(-math.flt_max, penetrations[0]);
    try testing.expectEqual(@as(i32, -1), indices[0]);

    // No triangles
    var context: Shape.GetTrianglesContext = .{};
    shape.getTrianglesStart(&context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
    var triangle_vertices: [3 * 32]Float3 = undefined;
    var materials: [32]*const PhysicsMaterial = undefined;
    try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&context, 32, &triangle_vertices, &materials));
}

test "EmptyShape: collide and cast against every shape do nothing (CollisionDispatch)" {
    const allocator = testing.allocator;

    var empty = EmptyShape.init(allocator, Vec3.zero());
    empty.asShape().setEmbedded();
    defer empty.asShapeMut().deinit();
    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var plane = PlaneShape.init(allocator, Plane.init(Vec3.axisY(), 0.0), .{});
    plane.asShape().setEmbedded();
    defer plane.asShapeMut().deinit();

    // Overlapping shapes in both orders: no hits, no panic of the unsupported functions
    for ([_]*const Shape{ empty.asShape(), sphere.asShape(), box.asShape(), plane.asShape() }) |other| {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(empty.asShape(), other, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &.{}, &collector.base, &.{});
        CollisionDispatch.collideShapeVsShape(other, empty.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &.{}, &collector.base, &.{});
        try testing.expectEqual(@as(usize, 0), collector.hits.items.len);

        var cast_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer cast_collector.deinit();
        const cast_empty = ShapeCast.init(empty.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 5, 0)), Vec3.init(0, -10, 0));
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast_empty, &.{}, other, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast_collector.base);
        const cast_other = ShapeCast.init(other, Vec3.one(), Mat44.translation(Vec3.init(0, 5, 0)), Vec3.init(0, -10, 0));
        CollisionDispatch.castShapeVsShapeWorldSpace(&cast_other, &.{}, empty.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast_collector.base);
        try testing.expectEqual(@as(usize, 0), cast_collector.hits.items.len);
    }

    // sRegister: every pair with an empty shape, registered last (it overrides e.g. the convex vs plane functions)
    const registry = &RegisterTypes.registry;
    for (ShapeFile.all_sub_shape_types) |s| {
        try testing.expect(registry.getCollideShape(.empty, s) == &EmptyShape.collideEmpty);
        try testing.expect(registry.getCollideShape(s, .empty) == &EmptyShape.collideEmpty);
        try testing.expect(registry.getCastShape(.empty, s) == &EmptyShape.castEmpty);
        try testing.expect(registry.getCastShape(s, .empty) == &EmptyShape.castEmpty);
    }
    const functions = ShapeFunctions.get(.empty);
    try testing.expect(functions.construct != null);
    try testing.expect(functions.color.eql(Color.black));
}

test "EmptyShape: binary state (the center of mass is not saved, like Jolt) and out of memory" {
    const allocator = testing.allocator;

    var empty = EmptyShape.init(allocator, Vec3.init(1, 2, 3));
    empty.asShape().setEmbedded();
    defer empty.asShapeMut().deinit();
    empty.asShapeMut().setUserData(9);

    // Sub type and user data (Shape's version)
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(empty.asShape(), &buffer);
    try testing.expectEqual(@as(usize, 1 + 8), bytes.len);
    var result = try restoreFromBuffer(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?.cast(EmptyShape);
    try testing.expectEqual(@as(u64, 9), restored.asShape().getUserData());
    try testing.expect(restored.asShape().getCenterOfMass().eql(Vec3.zero()));

    // A heap shape released through a reference
    var empty_ref = RefConst(Shape).init((try EmptyShape.create(allocator, Vec3.one())).asShape());
    try testing.expect(empty_ref.get().?.getCenterOfMass().eql(Vec3.one()));
    empty_ref.deinit();

    // Every creation path that allocates
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, EmptyShape.create(failing.allocator(), Vec3.zero()));
    try testing.expectError(error.OutOfMemory, EmptyShapeSettings.create(failing.allocator(), Vec3.zero()));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.empty).construct.?(failing.allocator()));
    try testing.expectError(error.OutOfMemory, restoreFromBuffer(failing.allocator(), bytes));
}
