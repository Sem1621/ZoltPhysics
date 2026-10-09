//! Port of: Jolt/Physics/Collision/Shape/ScaledShape.h, Jolt/Physics/Collision/Shape/ScaledShape.cpp
//! Status: complete
//!
//! A concrete decorated shape (Docs/Zolt/CollisionArchitecture.md, section 2 "Decorated shapes"):
//! - `ScaledShapeSettings` / `ScaledShape` embed `DecoratedShapeSettings` / `DecoratedShape` as `base` (D1); `overrides`
//!   lists every C++ `override` of the header in header order. The functions that ScaledShape does not override come
//!   from `DecoratedShape.impl` (and `Shape.impl`).
//! - Constructors: `initDefault` (default constructor), `initFromSettings` (D3, Jolt's error texts), `init` / `create`
//!   (ScaledShape(const Shape *, Vec3Arg), on the stack / `new`). The two settings constructors are `init` (const
//!   ShapeSettings *) and `initPtr` (const Shape *), with `create` / `createPtr` for `new ScaledShapeSettings(...)`.
//! - `sRegister` is `register` (D4): the collide / cast functions with a scaled shape on either side forward to the
//!   inner shape with the scale applied (`collideScaledVsShape`, `collideShapeVsScaled`, `castScaledVsShape`,
//!   `castShapeVsScaled`, private like in Jolt).
//! - JPH_DEBUG_RENDERER (Draw, DrawGetSupportFunction, DrawGetSupportingFace and the `inBaseOffset` parameter of
//!   GetSubmergedVolume) is not ported yet: TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const RVec3 = @import("../../../Math/Real.zig").RVec3;
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
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const DecoratedShapeFile = @import("DecoratedShape.zig");
const DecoratedShape = DecoratedShapeFile.DecoratedShape;
const DecoratedShapeSettings = DecoratedShapeFile.DecoratedShapeSettings;
const ScaleHelpers = @import("ScaleHelpers.zig");
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
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollideSoftBodyVertexIterator = @import("../CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Class that constructs a ScaledShape
pub const ScaledShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, ScaledShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,
    scale: Vec3 = Vec3.one(),

    /// Default constructor for deserialization
    pub fn initDefault(allocator: Allocator) ScaledShapeSettings {
        return .{ .base = .initDefault(ScaledShapeSettings, allocator) };
    }

    /// Constructor that decorates another shape with a scale (settings on the stack: `defer settings.deinit()`)
    pub fn init(allocator: Allocator, shape: ?*ShapeSettings, scale: Vec3) ScaledShapeSettings {
        return .{ .base = .init(ScaledShapeSettings, allocator, shape), .scale = scale };
    }

    /// Variant that uses a concrete shape, which means this object cannot be serialized.
    pub fn initPtr(allocator: Allocator, shape: ?*const Shape, scale: Vec3) ScaledShapeSettings {
        return .{ .base = .initPtr(ScaledShapeSettings, allocator, shape), .scale = scale };
    }

    /// new ScaledShapeSettings(inShape, inScale): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, shape: ?*ShapeSettings, scale: Vec3) Allocator.Error!*ScaledShapeSettings {
        const self = try allocator.create(ScaledShapeSettings);
        self.* = .init(allocator, shape, scale);
        return self;
    }

    /// new ScaledShapeSettings(const Shape *inShape, inScale): reference count 0, freed by the last release()
    pub fn createPtr(allocator: Allocator, shape: ?*const Shape, scale: Vec3) Allocator.Error!*ScaledShapeSettings {
        const self = try allocator.create(ScaledShapeSettings);
        self.* = .initPtr(allocator, shape, scale);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *ScaledShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
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
    /// Concrete class: `Shape.cast(ScaledShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .scaled;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSubShapeTransformedShape, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: DecoratedShape,
    /// Every field has a valid default: a half constructed shape (error in initFromSettings) is destroyed through the destructor chain
    scale: Vec3 = Vec3.one(),

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// ScaledShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) ScaledShape {
        return .{ .base = .init(ScaledShape, allocator, shape_sub_type, null) };
    }

    /// ScaledShape(const ScaledShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body
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

    /// Constructor that decorates another shape with a scale (on the stack / as a member: `asShape().setEmbedded()`
    /// before taking references, `asShapeMut().deinit()` at the end). Adds a reference to `shape`.
    pub fn init(allocator: Allocator, shape: *const Shape, scale: Vec3) ScaledShape {
        std.debug.assert(!ScaleHelpers.isZeroScale(scale));
        return .{ .base = .init(ScaledShape, allocator, shape_sub_type, shape), .scale = scale };
    }

    /// new ScaledShape(inShape, inScale): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, shape: *const Shape, scale: Vec3) Allocator.Error!*ScaledShape {
        const self = try allocator.create(ScaledShape);
        self.* = .init(allocator, shape, scale);
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const ScaledShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *ScaledShape) *Shape {
        return &self.base.base;
    }

    /// mInnerShape (never null after construction or a complete restore)
    fn innerShape(self: *const ScaledShape) *const Shape {
        return self.base.inner_shape.get().?;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Get the scale
    pub fn getScale(self: *const ScaledShape) Vec3 {
        return self.scale;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const ScaledShape) Vec3 {
        return self.scale.mul(self.innerShape().getCenterOfMass());
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const ScaledShape) AABox {
        return self.innerShape().getLocalBounds().scaled(self.scale);
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const ScaledShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        return self.innerShape().getWorldSpaceBounds(center_of_mass_transform, scale.mul(self.scale));
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const ScaledShape) f32 {
        return self.scale.reduceMin() * self.innerShape().getInnerRadius();
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const ScaledShape) MassProperties {
        var p = self.innerShape().getMassProperties();
        p.scale(self.scale);
        return p;
    }

    // See Shape::GetSubShapeTransformedShape
    pub fn getSubShapeTransformedShape(self: *const ScaledShape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) Shape.SubShapeTransformedShape {
        // We don't use any bits in the sub shape ID
        const remainder = sub_shape_id;

        var ts = TransformedShape.init(RVec3.fromVec3(position_com), rotation, self.innerShape(), .invalid, .{});
        ts.setShapeScale(scale.mul(self.scale));
        return .{ .transformed_shape = ts, .remainder = remainder };
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const ScaledShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        // Transform the surface point to local space and pass the query on
        const normal = self.innerShape().getSurfaceNormal(sub_shape_id, local_surface_position.div(self.scale));

        // Need to transform the plane normals using inScale
        // Transforming a direction with matrix M is done through multiplying by (M^-1)^T
        // In this case M is a diagonal matrix with the scale vector, so we need to multiply our normal by 1 / scale and renormalize afterwards
        return normal.div(self.scale).normalized();
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const ScaledShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        self.innerShape().getSupportingFace(sub_shape_id, direction, scale.mul(self.scale), center_of_mass_transform, out_vertices);
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const ScaledShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        return self.innerShape().getSubmergedVolume(center_of_mass_transform, scale.mul(self.scale), surface);
    }

    // TODO(debug_renderer): Draw, DrawGetSupportFunction, DrawGetSupportingFace (pass the call on with inScale * mScale)

    // See Shape::CastRay
    pub fn castRay(self: *const ScaledShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const inv_scale = self.scale.reciprocal();
        const scaled_ray = RayCast.init(inv_scale.mul(ray.origin), inv_scale.mul(ray.direction));
        return self.innerShape().castRay(scaled_ray, sub_shape_id_creator, hit);
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const ScaledShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const inv_scale = self.scale.reciprocal();
        const scaled_ray = RayCast.init(inv_scale.mul(ray.origin), inv_scale.mul(ray.direction));
        return self.innerShape().castRayCollector(scaled_ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const ScaledShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const inv_scale = self.scale.reciprocal();
        self.innerShape().collidePoint(inv_scale.mul(point), sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const ScaledShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.innerShape().collideSoftBodyVertices(center_of_mass_transform, scale.mul(self.scale), vertices, num_vertices, colliding_shape_index);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const ScaledShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        self.innerShape().collectTransformedShapes(box, position_com, rotation, scale.mul(self.scale), sub_shape_id_creator, collector, shape_filter);
    }

    // See Shape::TransformShape
    pub fn transformShape(self: *const ScaledShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.innerShape().transformShape(center_of_mass_transform.mul(Mat44.scaleVec3(self.scale)), collector);
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

    // See Shape: C++ `DecoratedShape::SaveBinaryState(inStream)` is Shape's version (DecoratedShape does not override it)
    pub fn saveBinaryState(self: *const ScaledShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(self.asShape(), stream);

        stream.write(self.scale);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const ScaledShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(ScaledShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const ScaledShape) f32 {
        return @abs(self.scale.getX() * self.scale.getY() * self.scale.getZ()) * self.innerShape().getVolume();
    }

    // See Shape::IsValidScale
    pub fn isValidScale(self: *const ScaledShape, scale: Vec3) bool {
        return self.innerShape().isValidScale(scale.mul(self.scale));
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const ScaledShape, scale: Vec3) Vec3 {
        return self.innerShape().makeScaleValid(self.scale.mul(scale)).div(self.scale);
    }

    // See: Shape::RestoreBinaryState: C++ `DecoratedShape::RestoreBinaryState(inStream)` is Shape's version
    pub fn restoreBinaryState(self: *ScaledShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(self.asShapeMut(), stream);

        stream.read(&self.scale);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
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

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn collideScaledVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const scaled1 = shape1.cast(ScaledShape); // Asserts that the sub type is EShapeSubType::Scaled

        CollisionDispatch.collideShapeVsShape(scaled1.base.getInnerShape().?, shape2, scale1.mul(scaled1.getScale()), scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn collideShapeVsScaled(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const scaled2 = shape2.cast(ScaledShape); // Asserts that the sub type is EShapeSubType::Scaled

        CollisionDispatch.collideShapeVsShape(shape1, scaled2.base.getInnerShape().?, scale1, scale2.mul(scaled2.getScale()), center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn castScaledVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const scaled = shape_cast.shape.cast(ScaledShape); // Asserts that the sub type is EShapeSubType::Scaled

        const scaled_cast = ShapeCast.init(scaled.base.getInnerShape().?, shape_cast.scale.mul(scaled.getScale()), shape_cast.center_of_mass_start, shape_cast.direction);
        CollisionDispatch.castShapeVsShapeLocalSpace(&scaled_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    fn castShapeVsScaled(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const scaled = shape.cast(ScaledShape); // Asserts that the sub type is EShapeSubType::Scaled

        CollisionDispatch.castShapeVsShapeLocalSpace(shape_cast, shape_cast_settings, scaled.innerShape(), scale.mul(scaled.scale), shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/DecoratedParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const math = @import("../../../Math/Math.zig");
const RegisterTypes = @import("../../../RegisterTypes.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const SphereShapeFile = @import("SphereShape.zig");
const SphereShape = SphereShapeFile.SphereShape;
const SphereShapeSettings = SphereShapeFile.SphereShapeSettings;
const BoxShape = @import("BoxShape.zig").BoxShape;
const RotatedTranslatedShape = @import("RotatedTranslatedShape.zig").RotatedTranslatedShape;

/// A filter that rejects one shape (and counts the calls through a pointer, Rule M)
const RejectShapeFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    rejected: ?*const Shape,
    calls: *u32,

    pub fn shouldCollide(self: *const RejectShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = sub_shape_id_of_shape2;
        self.calls.* += 1;
        return shape2 != self.rejected;
    }

    pub fn shouldCollidePair(self: *const RejectShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ sub_shape_id_of_shape1, sub_shape_id_of_shape2 };
        self.calls.* += 1;
        return shape1 != self.rejected and shape2 != self.rejected;
    }
};

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

test "ScaledShape: settings, Jolt's error texts, cached results and out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // Default constructor: unit scale, no inner shape
    {
        var settings = ScaledShapeSettings.initDefault(allocator);
        defer settings.deinit();
        try expect(settings.scale.eql(Vec3.one()));
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Inner shape is null!", result.getError());
    }

    // Zero scale (any component below ScaleHelpers.min_scale)
    for ([_]Vec3{ Vec3.zero(), Vec3.init(1, 0, 1), Vec3.init(1, 1, -1.0e-7) }) |scale| {
        const sphere_settings = try SphereShapeSettings.create(allocator, 1.0, .{});
        var settings = ScaledShapeSettings.init(allocator, sphere_settings.asShapeSettings(), scale);
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Can't use zero scale!", result.getError());
    }

    // An error of the child is forwarded
    {
        const sphere_settings = try SphereShapeSettings.create(allocator, -1.0, .{});
        var settings = ScaledShapeSettings.init(allocator, sphere_settings.asShapeSettings(), Vec3.replicate(2));
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid radius", result.getError());
    }

    // Child settings: the child is created (and cached), the user data is copied, the result is cached
    {
        const sphere_settings = try SphereShapeSettings.create(allocator, 1.5, .{});
        const settings = try ScaledShapeSettings.create(allocator, sphere_settings.asShapeSettings(), Vec3.init(-2, 2, 2));
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        defer settings_ref.deinit();
        settings.asShapeSettings().user_data = 42;
        var result = try settings.createShape(allocator);
        defer result.deinit();
        const scaled = result.getPtr().?.cast(ScaledShape);
        try expect(scaled.getScale().eql(Vec3.init(-2, 2, 2)));
        try testing.expectEqual(@as(u64, 42), scaled.asShape().getUserData());
        try testing.expectEqual(ShapeSubType.scaled, scaled.asShape().getSubType());
        try expect(scaled.base.getInnerShape().? == sphere_settings.base.base.cached_result.getPtr().?);
        var again = try settings.createShape(allocator);
        defer again.deinit();
        try expect(again.getPtr().? == result.getPtr().?);
    }

    // A concrete inner shape (heap settings through createPtr)
    {
        var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 1.0, .{})).asShape());
        defer sphere_ref.deinit();
        const settings = try ScaledShapeSettings.createPtr(allocator, sphere_ref.get(), Vec3.replicate(3));
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        defer settings_ref.deinit();
        var result = try settings.createShape(allocator);
        defer result.deinit();
        try expect(result.getPtr().?.cast(DecoratedShape).getInnerShape() == sphere_ref.get());
        try testing.expectEqual(@as(u32, 3), sphere_ref.get().?.getRefCount()); // sphere_ref, settings, scaled shape
    }

    // Out of memory is returned and not cached: the scaled shape and its child are allocated
    {
        const sphere_settings = try SphereShapeSettings.create(allocator, 1.0, .{});
        var settings = ScaledShapeSettings.init(allocator, sphere_settings.asShapeSettings(), Vec3.replicate(2));
        defer settings.deinit();
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var result = settings.asShapeSettings().createShape(failing.allocator()) catch |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try expect(settings.base.base.cached_result.isEmpty());
                sphere_settings.asShapeSettings().clearCachedResult();
                continue;
            };
            defer result.deinit();
            try expect(result.isValid() and !failing.has_induced_failure);
            try testing.expectEqual(@as(usize, 2), fail_index);
            sphere_settings.asShapeSettings().clearCachedResult();
            settings.asShapeSettings().clearCachedResult();
            break;
        }
    }
}

test "ScaledShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    try testing.expectError(error.OutOfMemory, ScaledShape.create(failing.allocator(), sphere.asShape(), Vec3.replicate(2)));
    try testing.expectError(error.OutOfMemory, ScaledShapeSettings.create(failing.allocator(), null, Vec3.replicate(2)));
    try testing.expectError(error.OutOfMemory, ScaledShapeSettings.createPtr(failing.allocator(), sphere.asShape(), Vec3.replicate(2)));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.scaled).construct.?(failing.allocator()));

    // Concrete inner shape: only the scaled shape is allocated
    var settings = ScaledShapeSettings.initPtr(allocator, sphere.asShape(), Vec3.replicate(2));
    defer settings.deinit();
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var result = settings.asShapeSettings().createShape(f.allocator()) catch continue;
        defer result.deinit();
        try testing.expectEqual(@as(usize, 1), fail_index);
        settings.asShapeSettings().clearCachedResult();
        break;
    }

    // Restore: the shape is the only allocation
    var scaled = ScaledShape.init(allocator, sphere.asShape(), Vec3.replicate(2));
    scaled.asShape().setEmbedded();
    defer scaled.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    try testing.expectError(error.OutOfMemory, restoreFromBuffer(failing.allocator(), saveToBuffer(scaled.asShape(), &buffer)));
}

test "ScaledShape: bounds, center of mass, inner radius, mass properties, volume, stats, sub shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.0 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    box.asShapeMut().setUserData(77);
    var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.identity(), box.asShape());
    rt.asShape().setEmbedded();
    defer rt.asShapeMut().deinit();
    const scale = Vec3.init(2, -3, 0.5);
    var scaled = ScaledShape.init(allocator, rt.asShape(), scale);
    scaled.asShape().setEmbedded();
    defer scaled.asShapeMut().deinit();
    const shape = scaled.asShape();

    try expect(shape.getType() == .decorated and shape.getSubType() == .scaled);
    try expect(shape.getCenterOfMass().eql(Vec3.init(2, -6, 1.5)));
    try expect(shape.getLocalBounds().eql(.init(Vec3.init(-2, -6, -1.5), Vec3.init(2, 6, 1.5))));
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.3), Vec3.init(4, 5, 6));
    try expect(shape.getWorldSpaceBounds(transform, Vec3.replicate(2)).eql(rt.asShape().getWorldSpaceBounds(transform, Vec3.replicate(2).mul(scale))));
    try testing.expectEqual(@as(f32, -3.0), shape.getInnerRadius()); // ReduceMin of the scale times the inner radius
    try testing.expectEqual(@as(f32, 3.0 * 48.0), shape.getVolume());
    var expected_mass = rt.asShape().getMassProperties();
    expected_mass.scale(scale);
    const mass = shape.getMassProperties();
    try testing.expectEqual(expected_mass.mass, mass.mass);
    try expect(mass.inertia.eql(expected_mass.inertia));
    try testing.expectEqual(@as(usize, @sizeOf(ScaledShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    const stats = try shape.getStatsRecursive(allocator, &visited);
    try testing.expectEqual(@as(usize, @sizeOf(ScaledShape) + @sizeOf(RotatedTranslatedShape) + @sizeOf(BoxShape)), stats.size_bytes);
    try testing.expectEqual(box.asShape().getStats().num_triangles, stats.num_triangles);

    // DecoratedShape.impl: the leaf, user data, material and sub shape ID bits of the inner shape
    const id = SubShapeIDCreator.pushID(.{}, 5, 3).getID();
    const leaf = shape.getLeafShape(id);
    try expect(leaf.shape == box.asShape() and leaf.remainder.eql(id));
    try testing.expectEqual(@as(u64, 77), shape.getSubShapeUserData(id));
    try expect(shape.getMaterial(.empty) == PhysicsMaterial.default);
    try testing.expectEqual(@as(u32, 0), shape.getSubShapeIDBitsRecursive());
    try expect(!shape.mustBeStatic());

    // The direct child with its scale
    var sub = shape.getSubShapeTransformedShape(id, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.5), Vec3.replicate(2));
    defer sub.transformed_shape.deinit();
    try expect(sub.remainder.eql(id));
    try expect(sub.transformed_shape.shape.get() == rt.asShape());
    try expect(sub.transformed_shape.getShapeScale().eql(Vec3.init(4, -6, 1)));
    try expect(sub.transformed_shape.shape_position_com.eql(RVec3.init(1, 2, 3)));
    try expect(sub.transformed_shape.shape_rotation.eql(Quat.rotation(Vec3.axisX(), 0.5)));
    try expect(sub.transformed_shape.body_id.eql(.invalid));

    // Surface normal: the point is scaled into the inner shape, the normal scaled back and normalized (the Y scale
    // is negative, so the top face of the box becomes the bottom face)
    try expect(shape.getSurfaceNormal(.empty, Vec3.init(0.5, -6, 1.5)).eql(Vec3.init(0, -1, 0)));
    try expect(shape.getSurfaceNormal(.empty, Vec3.init(2, 0, 0)).eql(Vec3.axisX()));

    // Supporting face: the inner shape with the combined scale
    var face: Shape.SupportingFace = .empty;
    var expected_face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.init(0.1, 1, 0.2), Vec3.replicate(-1), transform, &face);
    rt.asShape().getSupportingFace(.empty, Vec3.init(0.1, 1, 0.2), Vec3.replicate(-1).mul(scale), transform, &expected_face);
    try testing.expectEqual(@as(u32, 4), face.len);
    try expect(face.eql(&expected_face));
}

test "ScaledShape: valid scales (the ScaledShape part of Jolt's TestIsValidScale)" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 2.0, .{})).asShape());
    defer sphere_ref.deinit();
    const sphere = sphere_ref.get().?;

    var scaled_ref = RefConst(Shape).init((try ScaledShape.create(allocator, sphere, Vec3.init(1, 2, 1))).asShape());
    defer scaled_ref.deinit();
    const scaled = scaled_ref.get().?;
    try expect(!scaled.isValidScale(Vec3.zero()));
    try expect(!scaled.isValidScale(Vec3.init(1, 1, 1)));
    try expect(scaled.isValidScale(Vec3.init(1, 0.5, 1)));
    try expect(scaled.isValidScale(Vec3.init(-1, 0.5, 1)));
    try expect(!scaled.isValidScale(Vec3.init(2, 1, 1)));
    try expect(!scaled.isValidScale(Vec3.init(1, 2, 1)));
    try expect(!scaled.isValidScale(Vec3.init(1, 1, 2)));
    try expect(scaled.makeScaleValid(Vec3.init(3, 3, 3)).eql(Vec3.init(4, 2, 4)));
    try expect(scaled.makeScaleValid(Vec3.init(4, 2, 4)).eql(Vec3.init(4, 2, 4)));

    var scaled2_ref = RefConst(Shape).init((try ScaledShape.create(allocator, scaled, Vec3.init(1, 0.5, 1))).asShape());
    defer scaled2_ref.deinit();
    const scaled2 = scaled2_ref.get().?;
    try expect(!scaled2.isValidScale(Vec3.zero()));
    try expect(scaled2.isValidScale(Vec3.init(2, 2, 2)));
    try expect(scaled2.isValidScale(Vec3.init(-1, 1, -1)));
    try expect(!scaled2.isValidScale(Vec3.init(2, 1, 1)));
    try expect(!scaled2.isValidScale(Vec3.init(1, 2, 1)));
    try expect(!scaled2.isValidScale(Vec3.init(1, 1, 2)));
    try expect(scaled2.makeScaleValid(Vec3.init(3, 3, 3)).eql(Vec3.init(3, 3, 3)));
    try expect(scaled2.makeScaleValid(Vec3.init(5, 2, 5)).eql(Vec3.init(4, 4, 4)));
}

test "ScaledShape: ray casts, collide point, shape filters and the collector context" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var scaled = ScaledShape.init(allocator, sphere.asShape(), Vec3.replicate(-2));
    scaled.asShape().setEmbedded();
    defer scaled.asShapeMut().deinit();
    const shape = scaled.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 1, 2);

    // Single hit: the ray is scaled into the inner shape
    var hit: RayCastResult = .{};
    try expect(shape.castRay(.init(Vec3.init(-5, 0, 0), Vec3.init(10, 0, 0)), creator, &hit));
    try testing.expectEqual(@as(f32, 0.3), hit.fraction);
    try expect(hit.sub_shape_id2.eql(creator.getID()));
    try expect(!shape.castRay(.init(Vec3.init(-5, 3, 0), Vec3.init(10, 0, 0)), creator, &hit));

    // Collector: both faces, the body ID of the context
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(12), .{});
    hits.base.setContext(&context);
    var calls: u32 = 0;
    const accept_all: RejectShapeFilter = .{ .rejected = null, .calls = &calls };
    shape.castRayCollector(.init(Vec3.init(-5, 0, 0), Vec3.init(10, 0, 0)), &settings, creator, &hits.base, &accept_all.base);
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.3), hits.hits.items[0].fraction);
    try testing.expectEqual(@as(f32, 0.7), hits.hits.items[1].fraction);
    try expect(hits.hits.items[0].body_id.eql(.init(12)));
    try testing.expectEqual(@as(u32, 2), calls); // The scaled shape and the sphere

    // The filter rejects the scaled shape or the inner shape
    for ([_]*const Shape{ shape, sphere.asShape() }) |rejected| {
        hits.reset();
        const filter: RejectShapeFilter = .{ .rejected = rejected, .calls = &calls };
        shape.castRayCollector(.init(Vec3.init(-5, 0, 0), Vec3.init(10, 0, 0)), &settings, creator, &hits.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    }

    // Collide point
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.init(1.9, 0, 0), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(0, -2.1, 0), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    try expect(points.hits.items[0].sub_shape_id2.eql(creator.getID()));
    const reject_inner: RejectShapeFilter = .{ .rejected = sphere.asShape(), .calls = &calls };
    shape.collidePoint(Vec3.init(1.9, 0, 0), creator, &points.base, &reject_inner.base);
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
}

test "ScaledShape: collide and cast through CollisionDispatch match the inner shape with the scale applied" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var scaled = ScaledShape.init(allocator, sphere.asShape(), Vec3.replicate(2));
    scaled.asShape().setEmbedded();
    defer scaled.asShapeMut().deinit();
    var scaled_box = ScaledShape.init(allocator, box.asShape(), Vec3.init(1, -0.5, 2));
    scaled_box.asShape().setEmbedded();
    defer scaled_box.asShapeMut().deinit();

    // The registered functions (the decorators register after the convex shapes, later registrations win)
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.scaled, .sphere) == &ScaledShape.collideScaledVsShape);
    try expect(r.getCollideShape(.box, .scaled) == &ScaledShape.collideShapeVsScaled);
    try expect(r.getCollideShape(.scaled, .scaled) == &ScaledShape.collideShapeVsScaled); // Within one loop iteration the second write wins
    try expect(r.getCollideShape(.scaled, .rotated_translated) == &ScaledShape.collideScaledVsShape);
    try expect(r.getCollideShape(.offset_center_of_mass, .scaled) == &ScaledShape.collideShapeVsScaled);
    try expect(r.getCastShape(.scaled, .user1) == &ScaledShape.castScaledVsShape);
    try expect(r.getCastShape(.scaled, .scaled) == &ScaledShape.castShapeVsScaled);
    try expect(r.getCastShape(.empty, .scaled) == &ScaledShape.castShapeVsScaled);
    try expect(ShapeFunctions.get(.scaled).color.eql(Color.yellow));
    try expect(ShapeFunctions.get(.scaled).construct != null);

    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    const t1 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.2), Vec3.init(0.5, 1, 0));
    const t2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.4), Vec3.init(2, 0.5, 1));
    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 2);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);
    const Pair = struct { s1: *const Shape, sc1: Vec3, s2: *const Shape, sc2: Vec3, e1: *const Shape, esc1: Vec3, e2: *const Shape, esc2: Vec3 };
    const pairs = [_]Pair{
        .{ .s1 = scaled.asShape(), .sc1 = Vec3.replicate(0.5), .s2 = box.asShape(), .sc2 = Vec3.one(), .e1 = sphere.asShape(), .esc1 = Vec3.one(), .e2 = box.asShape(), .esc2 = Vec3.one() },
        .{ .s1 = box.asShape(), .sc1 = Vec3.one(), .s2 = scaled.asShape(), .sc2 = Vec3.replicate(-1), .e1 = box.asShape(), .esc1 = Vec3.one(), .e2 = sphere.asShape(), .esc2 = Vec3.replicate(-2) },
        .{ .s1 = scaled.asShape(), .sc1 = Vec3.one(), .s2 = scaled_box.asShape(), .sc2 = Vec3.replicate(2), .e1 = sphere.asShape(), .esc1 = Vec3.replicate(2), .e2 = box.asShape(), .esc2 = Vec3.init(2, -1, 4) },
    };
    for (pairs) |p| {
        var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, p.sc1, p.sc2, t1, t2, creator1, creator2, &settings, &hits.base, &.{});
        try hits.checkError();
        var expected = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.collideShapeVsShape(p.e1, p.e2, p.esc1, p.esc2, t1, t2, creator1, creator2, &settings, &expected.base, &.{});
        try expected.checkError();
        try testing.expectEqual(@as(usize, 1), expected.hits.items.len);
        try testing.expectEqual(expected.hits.items.len, hits.hits.items.len);
        const a = &hits.hits.items[0];
        const b = &expected.hits.items[0];
        try expect(a.contact_point_on1.eql(b.contact_point_on1) and a.contact_point_on2.eql(b.contact_point_on2) and a.penetration_axis.eql(b.penetration_axis));
        try testing.expectEqual(b.penetration_depth, a.penetration_depth);
        try expect(a.sub_shape_id1.eql(creator1.getID()) and a.sub_shape_id2.eql(creator2.getID()));
        try expect(a.shape1_face.eql(&b.shape1_face) and a.shape2_face.eql(&b.shape2_face));

        // The pair filter sees the inner shapes too
        var calls: u32 = 0;
        const filter: RejectShapeFilter = .{ .rejected = p.e1, .calls = &calls };
        var filtered = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer filtered.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, p.sc1, p.sc2, t1, t2, creator1, creator2, &settings, &filtered.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), filtered.hits.items.len);
    }

    // Casts with the scaled shape on either side
    const cast_settings: ShapeCastSettings = .{};
    const start = Mat44.translation(Vec3.init(-6, 0.25, 0));
    const CastPair = struct { cast: ShapeCast, s: *const Shape, sc: Vec3, expected_cast: ShapeCast, e: *const Shape, esc: Vec3 };
    const cast_pairs = [_]CastPair{
        .{ .cast = .init(scaled.asShape(), Vec3.replicate(0.5), start, Vec3.init(10, 0, 0)), .s = box.asShape(), .sc = Vec3.one(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = box.asShape(), .esc = Vec3.one() },
        .{ .cast = .init(box.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = scaled.asShape(), .sc = Vec3.replicate(1.5), .expected_cast = .init(box.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = sphere.asShape(), .esc = Vec3.replicate(3) },
    };
    for (cast_pairs) |p| {
        var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.cast, &cast_settings, p.s, p.sc, &.{}, t2, creator1, creator2, &hits.base);
        try hits.checkError();
        var expected = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.expected_cast, &cast_settings, p.e, p.esc, &.{}, t2, creator1, creator2, &expected.base);
        try expected.checkError();
        try testing.expectEqual(@as(usize, 1), expected.hits.items.len);
        try testing.expectEqual(expected.hits.items.len, hits.hits.items.len);
        try testing.expectEqual(expected.hits.items[0].fraction, hits.hits.items[0].fraction);
        try expect(hits.hits.items[0].base.contact_point_on2.eql(expected.hits.items[0].base.contact_point_on2));
        try expect(hits.hits.items[0].base.penetration_axis.eql(expected.hits.items[0].base.penetration_axis));
    }
}

test "ScaledShape: submerged volume, soft body vertices, collect and transform shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const scale = Vec3.init(2, 0.5, -1);
    var scaled = ScaledShape.init(allocator, box.asShape(), scale);
    scaled.asShape().setEmbedded();
    defer scaled.asShapeMut().deinit();
    const shape = scaled.asShape();
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.7), Vec3.init(1, 2, 3));

    // Submerged volume: the inner shape with the combined scale
    const surface = Plane.fromPointAndNormal(Vec3.init(0, 2.5, 0), Vec3.axisY());
    const v = shape.getSubmergedVolume(transform, Vec3.replicate(2), surface);
    const expected_v = box.asShape().getSubmergedVolume(transform, Vec3.replicate(2).mul(scale), surface);
    try testing.expectEqual(expected_v.total_volume, v.total_volume);
    try testing.expectEqual(expected_v.submerged_volume, v.submerged_volume);
    try expect(v.center_of_buoyancy.eql(expected_v.center_of_buoyancy));
    try expect(v.submerged_volume > 0 and v.submerged_volume < v.total_volume);

    // Soft body vertices: the inner shape with the combined scale
    var positions = [_]Vec3{ Vec3.init(1, 2.5, 3), Vec3.init(1.5, 2, 3.5), Vec3.init(10, 2, 3) };
    var inv_masses = [_]f32{ 1, 1, 1 };
    var planes: [2][3]Plane = undefined;
    var penetrations: [2][3]f32 = undefined;
    var indices: [2][3]i32 = undefined;
    for (0..2) |i| {
        planes[i] = @splat(Plane.init(Vec3.zero(), 0.0));
        penetrations[i] = @splat(-math.flt_max);
        indices[i] = @splat(-1);
        const vertices = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[i][0], .{}), .init(&penetrations[i][0], .{}), .init(&indices[i][0], .{}));
        if (i == 0)
            shape.collideSoftBodyVertices(transform, Vec3.one(), &vertices, 3, 5)
        else
            box.asShape().collideSoftBodyVertices(transform, scale, &vertices, 3, 5);
    }
    for (0..3) |i| {
        try testing.expectEqual(penetrations[1][i], penetrations[0][i]);
        try testing.expectEqual(indices[1][i], indices[0][i]);
        try expect(planes[0][i].normal_and_constant.eql(planes[1][i].normal_and_constant));
    }
    try testing.expectEqual(@as(i32, 5), indices[0][0]);

    // Collect transformed shapes: the leaf with the combined scale
    var collected = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collected.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(3), .{});
    collected.base.setContext(&context);
    const creator = SubShapeIDCreator.pushID(.{}, 3, 2);
    shape.collectTransformedShapes(AABox.biggest(), Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.1), Vec3.replicate(3), creator, &collected.base, &.{});
    try collected.checkError();
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);
    const ts = &collected.hits.items[0];
    try expect(ts.shape.get() == box.asShape() and ts.getShapeScale().eql(Vec3.init(6, 1.5, -3)));
    try expect(ts.shape_position_com.eql(RVec3.init(1, 2, 3)) and ts.shape_rotation.eql(Quat.rotation(Vec3.axisZ(), 0.1)));
    try expect(ts.body_id.eql(.init(3)) and ts.sub_shape_id_creator.getID().eql(creator.getID()));
    var calls: u32 = 0;
    const reject: RejectShapeFilter = .{ .rejected = shape, .calls = &calls };
    shape.collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), creator, &collected.base, &reject.base);
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);

    // Transform shape: the scale is applied to the transform, the leaf decomposes it
    var transformed = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer transformed.deinit();
    shape.transformShape(transform, &transformed.base);
    try transformed.checkError();
    try testing.expectEqual(@as(usize, 1), transformed.hits.items.len);
    try expect(transformed.hits.items[0].shape.get() == box.asShape());
    try expect(transformed.hits.items[0].getShapeScale().isClose(scale, .{ .max_dist_sq = 1.0e-10 }));

    // getTrianglesStart / Next assert (Jolt: "Cannot call on non-leaf shapes"), without asserts they return nothing
    if (!Core.enable_asserts) {
        var tri_context: Shape.GetTrianglesContext = .{};
        shape.getTrianglesStart(&tri_context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
        var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
        try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&tri_context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    }
}

test "ScaledShape: binary state, sub shape state, restoreFromBinaryState and saveWithChildren" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 1.25, .{})).asShape());
    defer sphere_ref.deinit();
    const scaled = try ScaledShape.create(allocator, sphere_ref.get().?, Vec3.replicate(-3));
    scaled.asShapeMut().setUserData(0x123456789);
    var scaled_ref = RefConst(Shape).init(scaled.asShape());
    defer scaled_ref.deinit();

    // Sub type, user data, scale
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(scaled.asShape(), &buffer);
    try testing.expectEqual(@as(usize, 1 + 8 + 12), bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.scaled), bytes[0]);

    // Restore: the inner shape comes from the sub shape state
    var result = try restoreFromBuffer(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?.castMut(ScaledShape);
    try expect(restored.getScale().eql(Vec3.replicate(-3)));
    try testing.expectEqual(@as(u64, 0x123456789), restored.asShape().getUserData());
    try expect(restored.base.getInnerShape() == null);
    var sub_shapes: ShapeFile.ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try scaled.asShape().saveSubShapeState(allocator, &sub_shapes);
    try testing.expectEqual(@as(usize, 1), sub_shapes.items.len);
    restored.asShapeMut().restoreSubShapeState(sub_shapes.items);
    try expect(restored.base.getInnerShape() == sphere_ref.get());
    var buffer2: [64]u8 = undefined;
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(restored.asShape(), &buffer2));

    // saveWithChildren / restoreWithChildren
    var graph: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&graph);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try scaled.asShape().saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
    try testing.expectEqual(@as(u32, 2), shape_map.count());
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var id_to_shape: Shape.IDToShapeMap = .empty;
    defer {
        for (id_to_shape.items) |*s| s.deinit();
        id_to_shape.deinit(allocator);
    }
    var id_to_material: Shape.IDToMaterialMap = .empty;
    defer {
        for (id_to_material.items) |*m| m.deinit();
        id_to_material.deinit(allocator);
    }
    var graph_result = try Shape.restoreWithChildren(allocator, in.streamIn(), &id_to_shape, &id_to_material);
    defer graph_result.deinit();
    const graph_scaled = graph_result.getPtr().?.cast(ScaledShape);
    try expect(graph_scaled.getScale().eql(Vec3.replicate(-3)));
    try testing.expectEqual(@as(f32, 1.25), graph_scaled.base.getInnerShape().?.cast(SphereShape).getRadius());
    try testing.expectEqual(scaled.asShape().getVolume(), graph_scaled.asShape().getVolume());
}
