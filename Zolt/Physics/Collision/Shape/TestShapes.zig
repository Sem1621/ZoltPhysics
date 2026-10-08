//! Zolt addition, no Jolt file: test shapes and a test material for the inline tests of the shape core
//! Status: complete
//!
//! Only used by the inline tests: `zolt.test_shapes` is this file in test builds (`builtin.is_test`), and build.zig
//! gives the inline tests a `zolt_user_types` module that registers `registrations` and `material_types` of this file
//! through the D4 user hook (Docs/Zolt/CollisionArchitecture.md), exactly like an application. The library and the
//! unit tests use the default (empty) user types module, the parity tests their own
//! (ZoltParity/Physics/ShapeCoreUserTypes.zig).
//!
//! The shapes derive from Shape directly (the concrete Jolt shapes are stubs while the shape core is ported) and
//! implement the virtual functions in a simple, deterministic way, so that the tests can exercise every Shape
//! dispatcher, the default implementations in `Shape.impl`, the collectors, the dispatch tables (with reversed
//! entries), TransformedShape and the binary state of a shape graph:
//! - `TestBoxShape` (User1): an axis aligned box around the center of mass (with an optional center of mass offset and
//!   material); overrides most virtual functions, keeps the defaults of the others.
//! - `TestSphereShape` (User2): a sphere; uses `Shape.collidePointUsingRayCast` and a uniform scale.
//! - `TestCompoundShape` (User3): two child shapes at offsets (1 sub shape ID bit), owns its children.
//! - `TestMaterial`: a material class registered through `material_types`.
//!
//! Registered collision functions: box vs box, sphere vs box (and the reversed box vs sphere through
//! CollisionDispatch.reversedCollideShape / reversedCastShape), compound vs any and any vs compound (recursing through
//! CollisionDispatch like Jolt's compound shapes).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RaySphere = @import("../../../Geometry/RaySphere.zig");
const math = @import("../../../Math/Math.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const RVec3 = @import("../../../Math/Real.zig").RVec3;
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeFile = @import("../CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeList = ShapeFile.ShapeList;
const ShapeRefC = ShapeFile.ShapeRefC;
const PhysicsMaterialList = ShapeFile.PhysicsMaterialList;
const PhysicsMaterialRefC = ShapeFile.PhysicsMaterialRefC;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const ScaleHelpers = @import("ScaleHelpers.zig");
const GetTrianglesContextFile = @import("GetTrianglesContext.zig");
const GetTrianglesContextVertexList = GetTrianglesContextFile.GetTrianglesContextVertexList;
const GetTrianglesContextMultiVertexList = GetTrianglesContextFile.GetTrianglesContextMultiVertexList;

/// The user registrations of the inline tests (see build.zig)
pub const registrations = .{TestShapeRegistration};

/// The user material types of the inline tests (see build.zig)
pub const material_types = .{TestMaterial};

/// A material class that only exists in the tests (restored through RegisterTypes.material_types)
pub const TestMaterial = struct {
    pub const overrides = .{ .getDebugName, .saveBinaryState, .restoreBinaryState };
    pub const rtti_name = "TestMaterial";

    base: PhysicsMaterial,
    value: u32 = 0,

    /// new TestMaterial(value)
    pub fn create(allocator: Allocator, value: u32) Allocator.Error!*TestMaterial {
        const self = try allocator.create(TestMaterial);
        self.* = .{ .base = .init(TestMaterial, allocator), .value = value };
        return self;
    }

    /// Default constructor on the heap (used by PhysicsMaterial.restoreFromBinaryState)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestMaterial {
        return create(allocator, 0);
    }

    pub fn getDebugName(self: *const TestMaterial) []const u8 {
        _ = self;
        return "TestMaterial";
    }

    pub fn saveBinaryState(self: *const TestMaterial, stream: StreamOut) void {
        PhysicsMaterial.impl.saveBinaryState(&self.base, stream);
        stream.write(self.value);
    }

    pub fn restoreBinaryState(self: *TestMaterial, stream: StreamIn) Allocator.Error!void {
        try PhysicsMaterial.impl.restoreBinaryState(&self.base, stream);
        stream.read(&self.value);
    }
};

/// Call counts of collideSoftBodyVertices (const query: the state lives behind a pointer, Rule M)
pub const SoftBodyRecord = struct {
    calls: u32 = 0,
    num_vertices: u32 = 0,
    colliding_shape_index: i32 = -1,
};

/// The 12 triangles of a unit box (computed at compile time, like Jolt's static vertex lists)
pub const unit_box_triangles: [36]Vec3 = blk: {
    const corners = [8]Vec3{
        Vec3.init(-1, -1, -1), Vec3.init(1, -1, -1), Vec3.init(1, 1, -1), Vec3.init(-1, 1, -1),
        Vec3.init(-1, -1, 1),  Vec3.init(1, -1, 1),  Vec3.init(1, 1, 1),  Vec3.init(-1, 1, 1),
    };
    const indices = [36]u8{ 0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 6, 2, 3, 7, 6, 0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5 };
    var result: [36]Vec3 = undefined;
    for (indices, &result) |i, *v| v.* = corners[i];
    break :blk result;
};

/// The triangles of a half unit sphere (top / bottom part) at detail level 1, computed at compile time
pub const half_unit_sphere_top: StaticArray(Vec3, 48) = blk: {
    @setEvalBranchQuota(100_000);
    var vertices: StaticArray(Vec3, 48) = .empty;
    GetTrianglesContextVertexList.createHalfUnitSphereTop(&vertices, 1) catch unreachable;
    break :blk vertices;
};
pub const half_unit_sphere_bottom: StaticArray(Vec3, 48) = blk: {
    @setEvalBranchQuota(100_000);
    var vertices: StaticArray(Vec3, 48) = .empty;
    GetTrianglesContextVertexList.createHalfUnitSphereBottom(&vertices, 1) catch unreachable;
    break :blk vertices;
};

/// Settings of TestBoxShape (a settings class with a material reference)
pub const TestBoxShapeSettings = struct {
    pub const overrides = .{.createShape};

    base: ShapeSettings,
    half_extent: Vec3 = Vec3.one(),
    material: RefConst(PhysicsMaterial) = .empty,

    /// Constructor
    pub fn init(allocator: Allocator, half_extent: Vec3, material: ?*const PhysicsMaterial) TestBoxShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(TestBoxShapeSettings), allocator), .half_extent = half_extent, .material = .init(material) };
    }

    /// new TestBoxShapeSettings(...): reference count 0
    pub fn create(allocator: Allocator, half_extent: Vec3, material: ?*const PhysicsMaterial) Allocator.Error!*TestBoxShapeSettings {
        const self = try allocator.create(TestBoxShapeSettings);
        self.* = .init(allocator, half_extent, material);
        return self;
    }

    pub fn destruct(self: *TestBoxShapeSettings) void {
        self.material.deinit();
    }

    pub fn asShapeSettings(self: *TestBoxShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *TestBoxShapeSettings) void {
        self.base.deinit();
    }

    pub fn createShape(self: *TestBoxShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(TestBoxShape, self, allocator);
    }
};

/// Settings of TestCompoundShape: holds child settings (Ref(ShapeSettings), creating a child writes its cache)
pub const TestCompoundShapeSettings = struct {
    pub const overrides = .{.createShape};

    pub const ChildSettings = struct {
        settings: Ref(ShapeSettings) = .empty,
        position: Vec3 = Vec3.zero(),
    };

    base: ShapeSettings,
    children: [2]ChildSettings = .{ .{}, .{} },

    /// Constructor, adds a reference to the child settings
    pub fn init(allocator: Allocator, child0: *ShapeSettings, position0: Vec3, child1: *ShapeSettings, position1: Vec3) TestCompoundShapeSettings {
        return .{ .base = .init(ShapeSettings.vtableFor(TestCompoundShapeSettings), allocator), .children = .{ .{ .settings = .init(child0), .position = position0 }, .{ .settings = .init(child1), .position = position1 } } };
    }

    pub fn destruct(self: *TestCompoundShapeSettings) void {
        for (&self.children) |*c| c.settings.deinit();
    }

    pub fn asShapeSettings(self: *TestCompoundShapeSettings) *ShapeSettings {
        return &self.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *TestCompoundShapeSettings) void {
        self.base.deinit();
    }

    pub fn createShape(self: *TestCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(TestCompoundShape, self, allocator);
    }
};

/// A box around its center of mass (User1)
pub const TestBoxShape = struct {
    pub const shape_sub_type: ShapeSubType = .user1;
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .saveMaterialState, .restoreMaterialState, .getStats, .getVolume, .restoreBinaryState };

    base: Shape,
    half_extent: Vec3 = Vec3.one(),
    center_of_mass: Vec3 = Vec3.zero(),
    material: RefConst(PhysicsMaterial) = .empty,
    soft_body_record: ?*SoftBodyRecord = null,

    /// Default constructor (used by restoreFromBinaryState)
    pub fn initDefault(allocator: Allocator) TestBoxShape {
        return .{ .base = .init(Shape.vtableFor(TestBoxShape), allocator, .user1, shape_sub_type) };
    }

    /// Constructor
    pub fn init(allocator: Allocator, half_extent: Vec3, opts: struct { center_of_mass: Vec3 = Vec3.zero(), material: ?*const PhysicsMaterial = null }) TestBoxShape {
        var self = initDefault(allocator);
        self.half_extent = half_extent;
        self.center_of_mass = opts.center_of_mass;
        self.material = .init(opts.material);
        return self;
    }

    /// Constructor from settings (in place, run by ShapeSettings.createCached)
    pub fn initFromSettings(self: *TestBoxShape, settings: *const TestBoxShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        self.base.initFromSettings(&settings.base);

        if (Vec3.lessOrEqual(settings.half_extent, Vec3.zero()).testAnyXYZTrue()) {
            result.setError("Invalid half extent");
            return;
        }
        self.half_extent = settings.half_extent;
        self.material.set(settings.material.get());

        result.set(.init(self.asShapeMut()));
    }

    /// new TestBoxShape(...): reference count 0
    pub fn create(allocator: Allocator, half_extent: Vec3, opts: struct { center_of_mass: Vec3 = Vec3.zero(), material: ?*const PhysicsMaterial = null }) Allocator.Error!*TestBoxShape {
        const self = try allocator.create(TestBoxShape);
        self.* = .init(allocator, half_extent, .{ .center_of_mass = opts.center_of_mass, .material = opts.material });
        return self;
    }

    pub fn destruct(self: *TestBoxShape) void {
        self.material.deinit();
    }

    pub fn asShape(self: *const TestBoxShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *TestBoxShape) *Shape {
        return &self.base;
    }

    fn localBox(self: *const TestBoxShape) AABox {
        return .init(self.half_extent.negate(), self.half_extent);
    }

    pub fn getCenterOfMass(self: *const TestBoxShape) Vec3 {
        return self.center_of_mass;
    }

    pub fn getLocalBounds(self: *const TestBoxShape) AABox {
        return self.localBox();
    }

    pub fn getSubShapeIDBitsRecursive(self: *const TestBoxShape) u32 {
        _ = self;
        return 0;
    }

    pub fn getInnerRadius(self: *const TestBoxShape) f32 {
        return self.half_extent.reduceMin();
    }

    pub fn getMassProperties(self: *const TestBoxShape) MassProperties {
        var p: MassProperties = .{};
        p.setMassAndInertiaOfSolidBox(self.half_extent.mulScalar(2.0), 1000.0);
        return p;
    }

    pub fn getMaterial(self: *const TestBoxShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        std.debug.assert(sub_shape_id.isEmpty());
        return self.material.get() orelse PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const TestBoxShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        std.debug.assert(sub_shape_id.isEmpty());
        const axis = local_surface_position.div(self.half_extent).abs().getHighestComponentIndex();
        var normal = Vec3.zero();
        normal.setComponent(axis, if (local_surface_position.getComponent(axis) < 0.0) -1.0 else 1.0);
        return normal;
    }

    pub fn getSupportingFace(self: *const TestBoxShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        std.debug.assert(sub_shape_id.isEmpty());
        self.localBox().scaled(scale).getSupportingFace(direction, out_vertices) catch unreachable;
        for (out_vertices.slice()) |*v| v.* = center_of_mass_transform.mulVec3(v.*);
    }

    pub fn getSubmergedVolume(self: *const TestBoxShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        const total_volume = self.getVolume() * @abs(scale.getX() * scale.getY() * scale.getZ());
        const center = center_of_mass_transform.getTranslation();
        if (surface.signedDistance(center) < 0.0)
            return .{ .total_volume = total_volume, .submerged_volume = total_volume, .center_of_buoyancy = center };
        return .{ .total_volume = total_volume, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    }

    pub fn castRay(self: *const TestBoxShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const fraction = math.max(RayAABox.rayAABox(ray.origin, .init(ray.direction), self.half_extent.negate(), self.half_extent), 0.0);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    pub fn castRayCollector(self: *const TestBoxShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const min_max = RayAABox.rayAABoxMinMax(ray.origin, .init(ray.direction), self.half_extent.negate(), self.half_extent);
        if (min_max.min > min_max.max or min_max.max < 0.0)
            return;
        const body_id = TransformedShape.getBodyID(collector.getContext());
        const front = math.max(min_max.min, 0.0);
        if (front < collector.getEarlyOutFraction()) {
            collector.addHit(&.{ .body_id = body_id, .fraction = front, .sub_shape_id2 = sub_shape_id_creator.getID() });
            if (collector.shouldEarlyOut()) return;
        }
        if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and min_max.max <= 1.0 and min_max.max < collector.getEarlyOutFraction())
            collector.addHit(&.{ .body_id = body_id, .fraction = min_max.max, .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    pub fn collidePoint(self: *const TestBoxShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;
        if (self.localBox().containsVec3(point))
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    pub fn collideSoftBodyVertices(self: *const TestBoxShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ center_of_mass_transform, scale, vertices };
        if (self.soft_body_record) |r| {
            r.calls += 1;
            r.num_vertices += num_vertices;
            r.colliding_shape_index = colliding_shape_index;
        }
    }

    pub fn getTrianglesStart(self: *const TestBoxShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        context.emplace(GetTrianglesContextVertexList).* = .init(position_com, rotation, scale, Mat44.scaleVec3(self.half_extent), &unit_box_triangles, self.getMaterial(.empty));
    }

    pub fn getTrianglesNext(self: *const TestBoxShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    pub fn saveBinaryState(self: *const TestBoxShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);
        stream.write(self.half_extent);
        stream.write(self.center_of_mass);
    }

    pub fn restoreBinaryState(self: *TestBoxShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);
        stream.read(&self.half_extent);
        stream.read(&self.center_of_mass);
    }

    pub fn saveMaterialState(self: *const TestBoxShape, allocator: Allocator, out_materials: *PhysicsMaterialList) Allocator.Error!void {
        try out_materials.ensureUnusedCapacity(allocator, 1);
        out_materials.appendAssumeCapacity(self.material.clone());
    }

    pub fn restoreMaterialState(self: *TestBoxShape, materials: []const PhysicsMaterialRefC) void {
        std.debug.assert(materials.len == 1);
        self.material.set(materials[0].get());
    }

    pub fn getStats(self: *const TestBoxShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TestBoxShape), 12);
    }

    pub fn getVolume(self: *const TestBoxShape) f32 {
        return 8.0 * self.half_extent.getX() * self.half_extent.getY() * self.half_extent.getZ();
    }
};

/// A sphere (User2), only valid with a uniform scale
pub const TestSphereShape = struct {
    pub const shape_sub_type: ShapeSubType = .user2;
    pub const overrides = .{ .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: Shape,
    radius: f32 = 1.0,

    /// Default constructor (used by restoreFromBinaryState)
    pub fn initDefault(allocator: Allocator) TestSphereShape {
        return .{ .base = .init(Shape.vtableFor(TestSphereShape), allocator, .user2, shape_sub_type) };
    }

    /// Constructor
    pub fn init(allocator: Allocator, radius: f32) TestSphereShape {
        var self = initDefault(allocator);
        self.radius = radius;
        return self;
    }

    /// new TestSphereShape(radius): reference count 0
    pub fn create(allocator: Allocator, radius: f32) Allocator.Error!*TestSphereShape {
        const self = try allocator.create(TestSphereShape);
        self.* = .init(allocator, radius);
        return self;
    }

    pub fn asShape(self: *const TestSphereShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *TestSphereShape) *Shape {
        return &self.base;
    }

    pub fn getLocalBounds(self: *const TestSphereShape) AABox {
        return .fromCenterAndRadius(Vec3.zero(), self.radius);
    }

    pub fn getSubShapeIDBitsRecursive(self: *const TestSphereShape) u32 {
        _ = self;
        return 0;
    }

    pub fn getInnerRadius(self: *const TestSphereShape) f32 {
        return self.radius;
    }

    pub fn getMassProperties(self: *const TestSphereShape) MassProperties {
        var p: MassProperties = .{};
        p.mass = 1000.0 * self.getVolume();
        return p;
    }

    pub fn getMaterial(self: *const TestSphereShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        _ = .{ self, sub_shape_id };
        return PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const TestSphereShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id };
        return local_surface_position.normalizedOr(Vec3.axisY());
    }

    pub fn getSubmergedVolume(self: *const TestSphereShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        const scaled_radius = self.radius * @abs(scale.getX());
        const total_volume = (4.0 / 3.0 * math.pi) * math.cubed(scaled_radius);
        const distance = surface.signedDistance(center_of_mass_transform.getTranslation());
        if (distance >= scaled_radius)
            return .{ .total_volume = total_volume, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
        return .{ .total_volume = total_volume, .submerged_volume = total_volume, .center_of_buoyancy = center_of_mass_transform.getTranslation() };
    }

    pub fn castRay(self: *const TestSphereShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const fraction = RaySphere.raySphere(ray.origin, ray.direction, Vec3.zero(), self.radius);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    pub fn castRayCollector(self: *const TestSphereShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const r = RaySphere.raySphereMinMax(ray.origin, ray.direction, Vec3.zero(), self.radius);
        if (r.num_intersections == 0) return;
        const body_id = TransformedShape.getBodyID(collector.getContext());
        if (r.min_fraction >= 0.0 and r.min_fraction <= 1.0 and r.min_fraction < collector.getEarlyOutFraction()) {
            collector.addHit(&.{ .body_id = body_id, .fraction = r.min_fraction, .sub_shape_id2 = sub_shape_id_creator.getID() });
            if (collector.shouldEarlyOut()) return;
        }
        if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and r.max_fraction >= 0.0 and r.max_fraction <= 1.0 and r.max_fraction < collector.getEarlyOutFraction())
            collector.addHit(&.{ .body_id = body_id, .fraction = r.max_fraction, .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    pub fn collidePoint(self: *const TestSphereShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // The ray cast fallback of Shape (counts the hits of a ray along +Y)
        Shape.collidePointUsingRayCast(self.asShape(), point, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const TestSphereShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
    }

    pub fn getTrianglesStart(self: *const TestSphereShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = box;
        const ctx = context.emplace(GetTrianglesContextMultiVertexList);
        ctx.* = .init(ScaleHelpers.isInsideOut(scale), PhysicsMaterial.default);
        const local_to_world = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale.mulScalar(self.radius)));
        ctx.addPart(local_to_world, half_unit_sphere_top.constSlice());
        ctx.addPart(local_to_world, half_unit_sphere_bottom.constSlice());
    }

    pub fn getTrianglesNext(self: *const TestSphereShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextMultiVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    pub fn saveBinaryState(self: *const TestSphereShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);
        stream.write(self.radius);
    }

    pub fn restoreBinaryState(self: *TestSphereShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);
        stream.read(&self.radius);
    }

    pub fn getStats(self: *const TestSphereShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TestSphereShape), 0);
    }

    pub fn getVolume(self: *const TestSphereShape) f32 {
        return 4.0 / 3.0 * math.pi * math.cubed(self.radius);
    }

    pub fn isValidScale(self: *const TestSphereShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScale(scale.abs());
    }

    pub fn makeScaleValid(self: *const TestSphereShape, scale: Vec3) Vec3 {
        _ = self;
        const abs_scale = scale.abs();
        return scale.getSign().mul(ScaleHelpers.makeUniformScale(abs_scale));
    }
};

/// Two child shapes at offsets (User3), the sub shape ID of a child is its index (1 bit)
pub const TestCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .user3;
    pub const overrides = .{ .mustBeStatic, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getLeafShape, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubShapeUserData, .getSubShapeTransformedShape, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .saveSubShapeState, .restoreSubShapeState, .getStats, .getStatsRecursive, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    pub const num_bits: u32 = 1;

    pub const Child = struct {
        shape: RefConst(Shape) = .empty,
        position: Vec3 = Vec3.zero(),
    };

    base: Shape,
    children: [2]Child = .{ .{}, .{} },
    must_be_static: bool = false,

    /// Default constructor (used by restoreFromBinaryState)
    pub fn initDefault(allocator: Allocator) TestCompoundShape {
        return .{ .base = .init(Shape.vtableFor(TestCompoundShape), allocator, .user3, shape_sub_type) };
    }

    /// Constructor from settings (in place, run by ShapeSettings.createCached): creates the children, a child error is
    /// forwarded (C++ `outResult = child_result`)
    pub fn initFromSettings(self: *TestCompoundShape, settings: *const TestCompoundShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base);

        for (&self.children, &settings.children) |*c, *s| {
            var child_result = try s.settings.get().?.createShape(allocator);
            defer child_result.deinit();
            if (child_result.hasError()) {
                result.assign(&child_result);
                return;
            }
            c.* = .{ .shape = .init(child_result.getPtr()), .position = s.position };
        }

        result.set(.init(self.asShapeMut()));
    }

    /// new TestCompoundShape(...): reference count 0, adds a reference to the children
    pub fn create(allocator: Allocator, child0: *const Shape, position0: Vec3, child1: *const Shape, position1: Vec3) Allocator.Error!*TestCompoundShape {
        const self = try allocator.create(TestCompoundShape);
        self.* = .initDefault(allocator);
        self.children = .{ .{ .shape = .init(child0), .position = position0 }, .{ .shape = .init(child1), .position = position1 } };
        return self;
    }

    pub fn destruct(self: *TestCompoundShape) void {
        for (&self.children) |*c| c.shape.deinit();
    }

    pub fn asShape(self: *const TestCompoundShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *TestCompoundShape) *Shape {
        return &self.base;
    }

    fn child(self: *const TestCompoundShape, index: u32) *const Shape {
        return self.children[index].shape.get().?;
    }

    /// The child a sub shape ID leads to and the remainder
    fn childFromID(sub_shape_id: SubShapeID) struct { index: u32, remainder: SubShapeID } {
        const popped = sub_shape_id.popID(num_bits);
        return .{ .index = popped.id, .remainder = popped.remainder };
    }

    fn childTransform(self: *const TestCompoundShape, index: u32, center_of_mass_transform: Mat44, scale: Vec3) Mat44 {
        return center_of_mass_transform.preTranslated(scale.mul(self.children[index].position));
    }

    pub fn mustBeStatic(self: *const TestCompoundShape) bool {
        return self.must_be_static;
    }

    pub fn getLocalBounds(self: *const TestCompoundShape) AABox {
        var bounds = AABox.empty;
        for (&self.children) |*c| {
            var b = c.shape.get().?.getLocalBounds();
            b.translate(c.position);
            bounds.encapsulate(b);
        }
        return bounds;
    }

    pub fn getSubShapeIDBitsRecursive(self: *const TestCompoundShape) u32 {
        return num_bits + @max(self.child(0).getSubShapeIDBitsRecursive(), self.child(1).getSubShapeIDBitsRecursive());
    }

    pub fn getInnerRadius(self: *const TestCompoundShape) f32 {
        return math.min(self.child(0).getInnerRadius(), self.child(1).getInnerRadius());
    }

    pub fn getMassProperties(self: *const TestCompoundShape) MassProperties {
        var p: MassProperties = .{};
        p.mass = self.child(0).getMassProperties().mass + self.child(1).getMassProperties().mass;
        return p;
    }

    pub fn getLeafShape(self: *const TestCompoundShape, sub_shape_id: SubShapeID) Shape.LeafShape {
        const c = childFromID(sub_shape_id);
        if (c.index >= self.children.len) return .{ .shape = null, .remainder = .empty };
        return self.child(c.index).getLeafShape(c.remainder);
    }

    pub fn getMaterial(self: *const TestCompoundShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        const c = childFromID(sub_shape_id);
        return self.child(c.index).getMaterial(c.remainder);
    }

    pub fn getSurfaceNormal(self: *const TestCompoundShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        const c = childFromID(sub_shape_id);
        return self.child(c.index).getSurfaceNormal(c.remainder, local_surface_position.sub(self.children[c.index].position));
    }

    pub fn getSupportingFace(self: *const TestCompoundShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        const c = childFromID(sub_shape_id);
        self.child(c.index).getSupportingFace(c.remainder, direction, scale, self.childTransform(c.index, center_of_mass_transform, scale), out_vertices);
    }

    pub fn getSubShapeUserData(self: *const TestCompoundShape, sub_shape_id: SubShapeID) u64 {
        const c = childFromID(sub_shape_id);
        return self.child(c.index).getSubShapeUserData(c.remainder);
    }

    pub fn getSubShapeTransformedShape(self: *const TestCompoundShape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) Shape.SubShapeTransformedShape {
        const c = childFromID(sub_shape_id);
        const position = position_com.add(rotation.mulVec3(scale.mul(self.children[c.index].position)));
        var ts = TransformedShape.init(RVec3.fromVec3(position), rotation, self.child(c.index), .invalid, .{});
        ts.setShapeScale(scale);
        return .{ .transformed_shape = ts, .remainder = c.remainder };
    }

    pub fn getSubmergedVolume(self: *const TestCompoundShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        var result: Shape.SubmergedVolume = .{ .total_volume = 0.0, .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
        for (0..self.children.len) |i| {
            const r = self.child(@intCast(i)).getSubmergedVolume(self.childTransform(@intCast(i), center_of_mass_transform, scale), scale, surface);
            result.total_volume += r.total_volume;
            result.submerged_volume += r.submerged_volume;
            result.center_of_buoyancy = result.center_of_buoyancy.add(r.center_of_buoyancy.mulScalar(r.submerged_volume));
        }
        if (result.submerged_volume > 0.0)
            result.center_of_buoyancy = result.center_of_buoyancy.divScalar(result.submerged_volume);
        return result;
    }

    pub fn castRay(self: *const TestCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        var any = false;
        for (0..self.children.len) |i| {
            const local_ray = ray.translated(self.children[i].position.negate());
            if (self.child(@intCast(i)).castRay(local_ray, sub_shape_id_creator.pushID(@intCast(i), num_bits), hit))
                any = true;
        }
        return any;
    }

    pub fn castRayCollector(self: *const TestCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        for (0..self.children.len) |i| {
            if (collector.shouldEarlyOut()) return;
            const local_ray = ray.translated(self.children[i].position.negate());
            self.child(@intCast(i)).castRayCollector(local_ray, ray_cast_settings, sub_shape_id_creator.pushID(@intCast(i), num_bits), collector, shape_filter);
        }
    }

    pub fn collidePoint(self: *const TestCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        for (0..self.children.len) |i|
            self.child(@intCast(i)).collidePoint(point.sub(self.children[i].position), sub_shape_id_creator.pushID(@intCast(i), num_bits), collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const TestCompoundShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        for (0..self.children.len) |i|
            self.child(@intCast(i)).collideSoftBodyVertices(self.childTransform(@intCast(i), center_of_mass_transform, scale), scale, vertices, num_vertices, colliding_shape_index);
    }

    pub fn collectTransformedShapes(self: *const TestCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        for (0..self.children.len) |i| {
            if (collector.shouldEarlyOut()) return;
            const position = position_com.add(rotation.mulVec3(scale.mul(self.children[i].position)));
            self.child(@intCast(i)).collectTransformedShapes(box, position, rotation, scale, sub_shape_id_creator.pushID(@intCast(i), num_bits), collector, shape_filter);
        }
    }

    pub fn transformShape(self: *const TestCompoundShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        for (0..self.children.len) |i|
            self.child(@intCast(i)).transformShape(center_of_mass_transform.preTranslated(self.children[i].position), collector);
    }

    pub fn getTrianglesStart(self: *const TestCompoundShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        // No triangles of its own (a compound returns the triangles of its children, not needed by the tests)
        _ = .{ self, box, position_com, rotation, scale };
        context.emplace(GetTrianglesContextVertexList).* = .init(Vec3.zero(), Quat.identity(), Vec3.one(), Mat44.identity(), &.{}, PhysicsMaterial.default);
    }

    pub fn getTrianglesNext(self: *const TestCompoundShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = self;
        return context.get(GetTrianglesContextVertexList).getTrianglesNext(max_triangles_requested, out_triangle_vertices, out_materials);
    }

    pub fn saveBinaryState(self: *const TestCompoundShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(&self.base, stream);
        for (&self.children) |*c| stream.write(c.position);
        stream.write(@as(u8, @intFromBool(self.must_be_static)));
    }

    pub fn restoreBinaryState(self: *TestCompoundShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(&self.base, stream);
        for (&self.children) |*c| stream.read(&c.position);
        var must_be_static: u8 = 0;
        stream.read(&must_be_static);
        self.must_be_static = must_be_static != 0;
    }

    pub fn saveSubShapeState(self: *const TestCompoundShape, allocator: Allocator, out_sub_shapes: *ShapeList) Allocator.Error!void {
        try out_sub_shapes.ensureUnusedCapacity(allocator, self.children.len);
        for (&self.children) |*c| out_sub_shapes.appendAssumeCapacity(c.shape.clone());
    }

    pub fn restoreSubShapeState(self: *TestCompoundShape, sub_shapes: []const ShapeRefC) void {
        std.debug.assert(sub_shapes.len == self.children.len);
        for (&self.children, sub_shapes) |*c, s| c.shape.set(s.get());
    }

    pub fn getStats(self: *const TestCompoundShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(TestCompoundShape), 0);
    }

    pub fn getStatsRecursive(self: *const TestCompoundShape, allocator: Allocator, visited_shapes: *Shape.VisitedShapes) Allocator.Error!Shape.Stats {
        // Jolt's CompoundShape::GetStatsRecursive: own stats (Shape::GetStatsRecursive), then the children
        var stats = try Shape.impl.getStatsRecursive(self.asShape(), allocator, visited_shapes);
        for (&self.children) |*c| {
            const child_stats = try c.shape.get().?.getStatsRecursive(allocator, visited_shapes);
            stats.size_bytes += child_stats.size_bytes;
            stats.num_triangles += child_stats.num_triangles;
        }
        return stats;
    }

    pub fn getVolume(self: *const TestCompoundShape) f32 {
        return self.child(0).getVolume() + self.child(1).getVolume();
    }

    pub fn isValidScale(self: *const TestCompoundShape, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and ScaleHelpers.isUniformScale(scale);
    }

    pub fn makeScaleValid(self: *const TestCompoundShape, scale: Vec3) Vec3 {
        _ = self;
        return ScaleHelpers.makeUniformScale(ScaleHelpers.makeNonZeroScale(scale));
    }
};

/// Registration of the test shapes (the sRegister of a user shape)
pub const TestShapeRegistration = struct {
    pub fn register(comptime r: *Registry) void {
        const box = r.shapeFunctions(.user1);
        box.construct = ShapeFunctions.constructor(TestBoxShape);
        box.color = Color.green;
        const sphere = r.shapeFunctions(.user2);
        sphere.construct = ShapeFunctions.constructor(TestSphereShape);
        sphere.color = Color.red;
        const compound = r.shapeFunctions(.user3);
        compound.construct = ShapeFunctions.constructor(TestCompoundShape);
        compound.color = Color.orange;

        r.registerCollideShape(.user1, .user1, collideBoxVsBox);
        r.registerCollideShape(.user2, .user1, collideSphereVsBox);
        r.registerCollideShape(.user1, .user2, CollisionDispatch.reversedCollideShape);
        r.registerCastShape(.user2, .user1, castSphereVsBox);
        r.registerCastShape(.user1, .user2, CollisionDispatch.reversedCastShape);

        for ([_]ShapeSubType{ .user1, .user2, .user3 }) |s| {
            r.registerCollideShape(.user3, s, collideCompoundVsShape);
            r.registerCollideShape(s, .user3, collideShapeVsCompound);
        }
    }
};

/// Box vs box: the world space bounds overlap test (the penetration axis is the axis with the smallest overlap)
pub fn collideBoxVsBox(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    _ = .{ collide_shape_settings, shape_filter };
    const b1 = shape1.getWorldSpaceBounds(center_of_mass_transform1, scale1);
    const b2 = shape2.getWorldSpaceBounds(center_of_mass_transform2, scale2);
    if (!b1.overlaps(b2)) return;
    const overlap = Vec3.min(b1.max, b2.max).sub(Vec3.max(b1.min, b2.min));
    const axis = overlap.getLowestComponentIndex();
    const depth = overlap.getComponent(axis);
    var penetration_axis = Vec3.zero();
    penetration_axis.setComponent(axis, if (b2.getCenter().getComponent(axis) >= b1.getCenter().getComponent(axis)) 1.0 else -1.0);
    const on1 = b1.getClosestPoint(b2.getCenter());
    const on2 = on1.sub(penetration_axis.mulScalar(depth));
    const result = CollideShapeResult.init(on1, on2, penetration_axis, depth, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));
    if (-result.penetration_depth < collector.getEarlyOutFraction())
        collector.addHit(&result);
}

/// Sphere vs box (shape1 is the sphere): closest point on the world space box to the sphere center
pub fn collideSphereVsBox(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    _ = shape_filter;
    const sphere = shape1.cast(TestSphereShape);
    const radius = sphere.radius * @abs(scale1.getX());
    const center = center_of_mass_transform1.getTranslation();
    const box = shape2.getWorldSpaceBounds(center_of_mass_transform2, scale2);
    const closest = box.getClosestPoint(center);
    const delta = closest.sub(center);
    const distance_sq = delta.lengthSq();
    if (distance_sq >= radius * radius) return;
    const distance = @sqrt(distance_sq);
    const axis = delta.normalizedOr(Vec3.axisY());
    var result = CollideShapeResult.init(center.add(axis.mulScalar(radius)), closest, axis, radius - distance, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));
    if (collide_shape_settings.collect_faces_mode == .collect_faces) {
        result.shape1_face.append(result.contact_point_on1);
        shape2.getSupportingFace(.empty, center_of_mass_transform2.multiply3x3Transposed(axis.negate()), scale2, center_of_mass_transform2, &result.shape2_face);
    }
    if (-result.penetration_depth < collector.getEarlyOutFraction())
        collector.addHit(&result);
}

/// Sphere cast against a box (the cast is in the local space of the box): a ray cast of the center against the box
/// expanded by the radius
pub fn castSphereVsBox(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    _ = shape_filter;
    const sphere = shape_cast.shape.cast(TestSphereShape);
    const radius = sphere.radius * @abs(shape_cast.scale.getX());
    var box = shape.getLocalBounds().scaled(scale);
    box.expandBy(Vec3.replicate(radius));
    const start = shape_cast.center_of_mass_start.getTranslation();
    const fraction = math.max(RayAABox.rayAABox(start, .init(shape_cast.direction), box.min, box.max), 0.0);
    if (fraction > 1.0 or fraction >= collector.getEarlyOutFraction()) return;
    const center = shape_cast.getPointOnRay(fraction);
    const normal = shape.getSurfaceNormal(.empty, center.div(scale));
    const contact_on2 = center.sub(normal.mulScalar(radius));
    var result = ShapeCastResult.init(fraction, center_of_mass_transform2.mulVec3(contact_on2), center_of_mass_transform2.mulVec3(contact_on2), center_of_mass_transform2.multiply3x3(normal.negate()), false, sub_shape_id_creator1.getID(), sub_shape_id_creator2.getID(), TransformedShape.getBodyID(collector.getContext()));
    if (shape_cast_settings.collect_faces_mode == .collect_faces)
        shape.getSupportingFace(.empty, normal, scale, center_of_mass_transform2, &result.base.shape2_face);
    collector.addHit(&result);
}

/// Compound vs shape: recurses into the children through CollisionDispatch (like Jolt's compound shapes)
pub fn collideCompoundVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    const compound = shape1.cast(TestCompoundShape);
    for (0..compound.children.len) |i| {
        if (collector.shouldEarlyOut()) return;
        const transform1 = compound.childTransform(@intCast(i), center_of_mass_transform1, scale1);
        CollisionDispatch.collideShapeVsShape(compound.child(@intCast(i)), shape2, scale1, scale2, transform1, center_of_mass_transform2, sub_shape_id_creator1.pushID(@intCast(i), TestCompoundShape.num_bits), sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }
}

/// Shape vs compound: recurses into the children through CollisionDispatch (like Jolt's compound shapes)
pub fn collideShapeVsCompound(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    const compound = shape2.cast(TestCompoundShape);
    for (0..compound.children.len) |i| {
        if (collector.shouldEarlyOut()) return;
        const transform2 = compound.childTransform(@intCast(i), center_of_mass_transform2, scale2);
        CollisionDispatch.collideShapeVsShape(shape1, compound.child(@intCast(i)), scale1, scale2, center_of_mass_transform1, transform2, sub_shape_id_creator1, sub_shape_id_creator2.pushID(@intCast(i), TestCompoundShape.num_bits), collide_shape_settings, collector, shape_filter);
    }
}
