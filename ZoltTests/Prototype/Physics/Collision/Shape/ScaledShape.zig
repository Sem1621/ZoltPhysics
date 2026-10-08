//! Port of: Jolt/Physics/Collision/Shape/ScaledShape.h, Jolt/Physics/Collision/Shape/ScaledShape.cpp (prototype)
//! Status: partial
//! Missing: GetSubShapeTransformedShape, GetSubmergedVolume, CollideSoftBodyVertices, JPH_DEBUG_RENDERER
//!
//! A concrete decorated shape: its overrides are top-level functions listed in `overrides`; the virtual functions it
//! does not override come from `DecoratedShape.impl` and `Shape.impl` (found by `virtual.make`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const Core = zolt.Core;
const AABox = zolt.AABox;
const Color = zolt.Color;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const Vec3 = zolt.Vec3;

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
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const DecoratedShapeFile = @import("DecoratedShape.zig");
const DecoratedShape = DecoratedShapeFile.DecoratedShape;
const DecoratedShapeSettings = DecoratedShapeFile.DecoratedShapeSettings;
const ScaleHelpers = @import("ScaleHelpers.zig");
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const MassProperties = @import("../../Body/MassProperties.zig").MassProperties;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Class that constructs a ScaledShape
pub const ScaledShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,
    /// Scale for the shape
    scale: Vec3 = Vec3.one(),

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) ScaledShapeSettings {
        return .{ .base = .init(ScaledShapeSettings, allocator, null) };
    }

    /// Constructor that decorates another shape with a scale (ScaledShapeSettings(const ShapeSettings *, Vec3Arg))
    pub fn init(allocator: Allocator, shape: ?*ShapeSettings, scale: Vec3) ScaledShapeSettings {
        return .{ .base = .init(ScaledShapeSettings, allocator, shape), .scale = scale };
    }

    /// Variant that uses a concrete shape (ScaledShapeSettings(const Shape *, Vec3Arg))
    pub fn initPtr(allocator: Allocator, shape: ?*const Shape, scale: Vec3) ScaledShapeSettings {
        return .{ .base = .initPtr(ScaledShapeSettings, allocator, shape), .scale = scale };
    }

    /// new ScaledShapeSettings(const ShapeSettings *, Vec3Arg)
    pub fn create(allocator: Allocator, shape: ?*ShapeSettings, scale: Vec3) Allocator.Error!*ScaledShapeSettings {
        const self = try allocator.create(ScaledShapeSettings);
        self.* = .init(allocator, shape, scale);
        return self;
    }

    pub fn asShapeSettings(self: *ScaledShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    pub fn deinit(self: *ScaledShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *ScaledShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(ScaledShape, self, allocator);
    }
};

/// A shape that scales a child shape in local space of that shape. The scale can be non-uniform and can even turn it inside out when one or three components of the scale are negative.
pub const ScaledShape = struct {
    pub const shape_sub_type: ShapeSubType = .scaled;

    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSupportingFace, .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .restoreBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid };

    base: DecoratedShape,
    scale: Vec3 = Vec3.one(),

    /// ScaledShape()
    pub fn initDefault(allocator: Allocator) ScaledShape {
        return .{ .base = .init(ScaledShape, allocator, shape_sub_type, null) };
    }

    /// ScaledShape(const ScaledShapeSettings &, ShapeResult &): the base constructor may already have set an error
    pub fn initFromSettings(self: *ScaledShape, settings: *const ScaledShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        try self.base.initFromSettings(&settings.base, result, allocator);
        self.scale = settings.scale;

        if (result.hasError())
            return;

        if (ScaleHelpers.isZeroScale(settings.scale)) {
            result.setError("Can't use zero scale!");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// Constructor that decorates another shape with a scale
    pub fn init(allocator: Allocator, shape: *const Shape, scale: Vec3) ScaledShape {
        std.debug.assert(!ScaleHelpers.isZeroScale(scale));
        return .{ .base = .init(ScaledShape, allocator, shape_sub_type, shape), .scale = scale };
    }

    /// new ScaledShape(const Shape *, Vec3Arg)
    pub fn create(allocator: Allocator, shape: *const Shape, scale: Vec3) Allocator.Error!*ScaledShape {
        const self = try allocator.create(ScaledShape);
        self.* = .init(allocator, shape, scale);
        return self;
    }

    pub fn asShape(self: *const ScaledShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *ScaledShape) *Shape {
        return &self.base.base;
    }

    /// Get the scale
    pub fn getScale(self: *const ScaledShape) Vec3 {
        return self.scale;
    }

    fn inner(self: *const ScaledShape) *const Shape {
        return self.base.getInnerShape();
    }

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const ScaledShape) Vec3 {
        return self.scale.mul(self.inner().getCenterOfMass());
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const ScaledShape) AABox {
        return self.inner().getLocalBounds().scaled(self.scale);
    }

    // See Shape::GetWorldSpaceBounds
    pub fn getWorldSpaceBounds(self: *const ScaledShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        return self.inner().getWorldSpaceBounds(center_of_mass_transform, scale.mul(self.scale));
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const ScaledShape) f32 {
        return self.scale.reduceMin() * self.inner().getInnerRadius();
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const ScaledShape) MassProperties {
        var p = self.inner().getMassProperties();
        p.scale(self.scale);
        return p;
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const ScaledShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        // Transform the surface point to local space and pass the query on
        const normal = self.inner().getSurfaceNormal(sub_shape_id, local_surface_position.div(self.scale));

        // Need to transform the plane normals using inScale
        // Transforming a direction with matrix M is done through multiplying by (M^-1)^T
        // In this case M is a diagonal matrix with the scale vector, so we need to multiply our normal by 1 / scale and renormalize afterwards
        return normal.div(self.scale).normalized();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const ScaledShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        self.inner().getSupportingFace(sub_shape_id, direction, scale.mul(self.scale), center_of_mass_transform, out_vertices);
    }

    // See Shape::CastRay
    pub fn castRay(self: *const ScaledShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const inv_scale = self.scale.reciprocal();
        const scaled_ray = RayCast.init(inv_scale.mul(ray.origin), inv_scale.mul(ray.direction));
        return self.inner().castRay(scaled_ray, sub_shape_id_creator, hit);
    }

    // See Shape::CastRay (collector version)
    pub fn castRayCollector(self: *const ScaledShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const inv_scale = self.scale.reciprocal();
        const scaled_ray = RayCast.init(inv_scale.mul(ray.origin), inv_scale.mul(ray.direction));
        self.inner().castRayCollector(scaled_ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const ScaledShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const inv_scale = self.scale.reciprocal();
        self.inner().collidePoint(inv_scale.mul(point), sub_shape_id_creator, collector, shape_filter);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const ScaledShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        self.inner().collectTransformedShapes(box, position_com, rotation, scale.mul(self.scale), sub_shape_id_creator, collector, shape_filter);
    }

    // See Shape::TransformShape
    pub fn transformShape(self: *const ScaledShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.inner().transformShape(center_of_mass_transform.mul(Mat44.scaleVec3(self.scale)), collector);
    }

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const ScaledShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, context, box, position_com, rotation, scale };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const ScaledShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
        return 0;
    }

    // See Shape::SaveBinaryState: C++ `DecoratedShape::SaveBinaryState` resolves to Shape's version (DecoratedShape does not override it)
    pub fn saveBinaryState(self: *const ScaledShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(self.asShape(), stream);

        stream.write(self.scale);
    }

    // See Shape::RestoreBinaryState
    pub fn restoreBinaryState(self: *ScaledShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(self.asShapeMut(), stream);

        stream.read(&self.scale);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const ScaledShape) Shape.Stats {
        _ = self;
        return .{ .size_bytes = @sizeOf(ScaledShape), .num_triangles = 0 };
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const ScaledShape) f32 {
        return @abs(self.scale.getX() * self.scale.getY() * self.scale.getZ()) * self.inner().getVolume();
    }

    // See Shape::IsValidScale
    pub fn isValidScale(self: *const ScaledShape, scale: Vec3) bool {
        return self.inner().isValidScale(scale.mul(self.scale));
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const ScaledShape, scale: Vec3) Vec3 {
        return self.inner().makeScaleValid(self.scale.mul(scale)).div(self.scale);
    }

    // Helper functions called by CollisionDispatch
    pub fn collideScaledVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(ScaledShape);

        CollisionDispatch.collideShapeVsShape(shape1.inner(), shape2, scale1.mul(shape1.getScale()), scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    pub fn collideShapeVsScaled(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(ScaledShape);

        CollisionDispatch.collideShapeVsShape(shape1, shape2.inner(), scale1, scale2.mul(shape2.getScale()), center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    pub fn castScaledVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const scaled = shape_cast.shape.cast(ScaledShape);

        const scaled_cast = ShapeCast.init(scaled.inner(), shape_cast.scale.mul(scaled.getScale()), shape_cast.center_of_mass_start, shape_cast.direction);
        CollisionDispatch.castShapeVsShapeLocalSpace(&scaled_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    pub fn castShapeVsScaled(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(ScaledShape);

        CollisionDispatch.castShapeVsShapeLocalSpace(shape_cast, shape_cast_settings, shape.inner(), scale.mul(shape.scale), shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    // Register shape functions with the registry (sRegister)
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.scaled);
        f.construct = ShapeFunctions.constructor(ScaledShape);
        f.color = Color.yellow;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.scaled, s, collideScaledVsShape);
            r.registerCollideShape(s, .scaled, collideShapeVsScaled);
            r.registerCastShape(.scaled, s, castScaledVsShape);
            r.registerCastShape(s, .scaled, castShapeVsScaled);
        }
    }
};
