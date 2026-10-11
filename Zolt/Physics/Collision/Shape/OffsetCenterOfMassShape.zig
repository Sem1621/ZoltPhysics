//! Port of: Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h, Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.cpp
//! Status: complete
//!
//! A concrete decorated shape (Docs/Zolt/CollisionArchitecture.md, section 2 "Decorated shapes"):
//! - `OffsetCenterOfMassShapeSettings` / `OffsetCenterOfMassShape` embed `DecoratedShapeSettings` / `DecoratedShape` as
//!   `base` (D1); `overrides` lists every C++ `override` of the header in header order. The functions that
//!   OffsetCenterOfMassShape does not override (IsValidScale, MakeScaleValid, ...) come from `DecoratedShape.impl` (and
//!   `Shape.impl`).
//! - Constructors: `initDefault` (default constructor), `initFromSettings` (D3), `init` / `create`
//!   (OffsetCenterOfMassShape(const Shape *, Vec3Arg), on the stack / `new`). The two settings constructors are `init`
//!   (const ShapeSettings *) and `initPtr` (const Shape *), with `create` / `createPtr` for `new`.
//! - `mOffset` is uninitialized in Jolt for the default constructors of the settings and the shape; it is zero here (the
//!   constructors and the restore set it like in Jolt).
//! - `sRegister` is `register` (D4): the collide / cast functions with an offset center of mass shape on either side
//!   forward to the inner shape with the offset applied, private like in Jolt.
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

/// Class that constructs an OffsetCenterOfMassShape
pub const OffsetCenterOfMassShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, OffsetCenterOfMassShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,
    /// Offset to be applied to the center of mass of the child shape (uninitialized in Jolt)
    offset: Vec3 = Vec3.zero(),

    /// Constructor (the default constructor, for deserialization)
    pub fn initDefault(allocator: Allocator) OffsetCenterOfMassShapeSettings {
        return .{ .base = .initDefault(OffsetCenterOfMassShapeSettings, allocator) };
    }

    /// Construct with shape settings, can be serialized (settings on the stack: `defer settings.deinit()`).
    pub fn init(allocator: Allocator, offset: Vec3, shape: ?*ShapeSettings) OffsetCenterOfMassShapeSettings {
        return .{ .base = .init(OffsetCenterOfMassShapeSettings, allocator, shape), .offset = offset };
    }

    /// Variant that uses a concrete shape, which means this object cannot be serialized.
    pub fn initPtr(allocator: Allocator, offset: Vec3, shape: ?*const Shape) OffsetCenterOfMassShapeSettings {
        return .{ .base = .initPtr(OffsetCenterOfMassShapeSettings, allocator, shape), .offset = offset };
    }

    /// new OffsetCenterOfMassShapeSettings(inOffset, inShape): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, offset: Vec3, shape: ?*ShapeSettings) Allocator.Error!*OffsetCenterOfMassShapeSettings {
        const self = try allocator.create(OffsetCenterOfMassShapeSettings);
        self.* = .init(allocator, offset, shape);
        return self;
    }

    /// new OffsetCenterOfMassShapeSettings(inOffset, const Shape *inShape): reference count 0, freed by the last release()
    pub fn createPtr(allocator: Allocator, offset: Vec3, shape: ?*const Shape) Allocator.Error!*OffsetCenterOfMassShapeSettings {
        const self = try allocator.create(OffsetCenterOfMassShapeSettings);
        self.* = .initPtr(allocator, offset, shape);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *OffsetCenterOfMassShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *OffsetCenterOfMassShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *OffsetCenterOfMassShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(OffsetCenterOfMassShape, self, allocator);
    }
};

/// This shape will shift the center of mass of a child shape, it can e.g. be used to lower the center of mass of an unstable object like a boat to make it stable
pub const OffsetCenterOfMassShape = struct {
    /// Concrete class: `Shape.cast(OffsetCenterOfMassShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .offset_center_of_mass;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSubShapeTransformedShape, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .restoreBinaryState };

    base: DecoratedShape,
    /// Offset of the center of mass
    offset: Vec3 = Vec3.zero(),

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// OffsetCenterOfMassShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) OffsetCenterOfMassShape {
        return .{ .base = .init(OffsetCenterOfMassShape, allocator, shape_sub_type, null) };
    }

    /// OffsetCenterOfMassShape(const OffsetCenterOfMassShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body
    pub fn initFromSettings(self: *OffsetCenterOfMassShape, settings: *const OffsetCenterOfMassShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        try self.base.initFromSettings(&settings.base, result, allocator);
        self.offset = settings.offset;

        if (result.hasError())
            return;

        result.set(.init(self.asShapeMut()));
    }

    /// OffsetCenterOfMassShape(const Shape *inShape, Vec3Arg inOffset) (on the stack / as a member:
    /// `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end). Adds a reference to
    /// `shape`.
    pub fn init(allocator: Allocator, shape: *const Shape, offset: Vec3) OffsetCenterOfMassShape {
        return .{ .base = .init(OffsetCenterOfMassShape, allocator, shape_sub_type, shape), .offset = offset };
    }

    /// new OffsetCenterOfMassShape(inShape, inOffset): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, shape: *const Shape, offset: Vec3) Allocator.Error!*OffsetCenterOfMassShape {
        const self = try allocator.create(OffsetCenterOfMassShape);
        self.* = .init(allocator, shape, offset);
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const OffsetCenterOfMassShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *OffsetCenterOfMassShape) *Shape {
        return &self.base.base;
    }

    /// mInnerShape (never null after construction or a complete restore)
    fn innerShape(self: *const OffsetCenterOfMassShape) *const Shape {
        return self.base.inner_shape.get().?;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Access the offset that is applied to the center of mass
    pub fn getOffset(self: *const OffsetCenterOfMassShape) Vec3 {
        return self.offset;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const OffsetCenterOfMassShape) Vec3 {
        return self.innerShape().getCenterOfMass().add(self.offset);
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const OffsetCenterOfMassShape) AABox {
        var bounds = self.innerShape().getLocalBounds();
        bounds.min = bounds.min.sub(self.offset);
        bounds.max = bounds.max.sub(self.offset);
        return bounds;
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const OffsetCenterOfMassShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        return self.innerShape().getWorldSpaceBounds(center_of_mass_transform.preTranslated(scale.negate().mul(self.offset)), scale);
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const OffsetCenterOfMassShape) f32 {
        return self.innerShape().getInnerRadius();
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const OffsetCenterOfMassShape) MassProperties {
        var mp = self.innerShape().getMassProperties();
        mp.translate(self.offset);
        return mp;
    }

    // See Shape::GetSubShapeTransformedShape
    pub fn getSubShapeTransformedShape(self: *const OffsetCenterOfMassShape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) Shape.SubShapeTransformedShape {
        // We don't use any bits in the sub shape ID
        const remainder = sub_shape_id;

        var ts = TransformedShape.init(RVec3.fromVec3(position_com.sub(rotation.mulVec3(scale.mul(self.offset)))), rotation, self.innerShape(), .invalid, .{});
        ts.setShapeScale(scale);
        return .{ .transformed_shape = ts, .remainder = remainder };
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const OffsetCenterOfMassShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        // Transform surface position to local space and pass call on
        return self.innerShape().getSurfaceNormal(sub_shape_id, local_surface_position.add(self.offset));
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const OffsetCenterOfMassShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        self.innerShape().getSupportingFace(sub_shape_id, direction, scale, center_of_mass_transform.preTranslated(scale.negate().mul(self.offset)), out_vertices);
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const OffsetCenterOfMassShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        return self.innerShape().getSubmergedVolume(center_of_mass_transform.preTranslated(scale.negate().mul(self.offset)), scale, surface);
    }

    // TODO(debug_renderer): Draw, DrawGetSupportFunction, DrawGetSupportingFace (pass the call on with
    // inCenterOfMassTransform.PreTranslated(-inScale * mOffset))

    // See Shape::CastRay
    pub fn castRay(self: *const OffsetCenterOfMassShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Transform the ray to local space
        var local_ray = ray;
        local_ray.origin = local_ray.origin.add(self.offset);

        return self.innerShape().castRay(local_ray, sub_shape_id_creator, hit);
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const OffsetCenterOfMassShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Transform the ray to local space
        var local_ray = ray;
        local_ray.origin = local_ray.origin.add(self.offset);

        return self.innerShape().castRayCollector(local_ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const OffsetCenterOfMassShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Pass the point on to the inner shape in local space
        self.innerShape().collidePoint(point.add(self.offset), sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const OffsetCenterOfMassShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.innerShape().collideSoftBodyVertices(center_of_mass_transform.preTranslated(scale.negate().mul(self.offset)), scale, vertices, num_vertices, colliding_shape_index);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const OffsetCenterOfMassShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        self.innerShape().collectTransformedShapes(box, position_com.sub(rotation.mulVec3(scale.mul(self.offset))), rotation, scale, sub_shape_id_creator, collector, shape_filter);
    }

    // See Shape::TransformShape
    pub fn transformShape(self: *const OffsetCenterOfMassShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.innerShape().transformShape(center_of_mass_transform.preTranslated(self.offset.negate()), collector);
    }

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const OffsetCenterOfMassShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, context, box, position_com, rotation, scale };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const OffsetCenterOfMassShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
        return 0;
    }

    // See Shape: C++ `DecoratedShape::SaveBinaryState(inStream)` is Shape's version (DecoratedShape does not override it)
    pub fn saveBinaryState(self: *const OffsetCenterOfMassShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(self.asShape(), stream);

        stream.write(self.offset);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const OffsetCenterOfMassShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(OffsetCenterOfMassShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const OffsetCenterOfMassShape) f32 {
        return self.innerShape().getVolume();
    }

    // See: Shape::RestoreBinaryState: C++ `DecoratedShape::RestoreBinaryState(inStream)` is Shape's version
    pub fn restoreBinaryState(self: *OffsetCenterOfMassShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(self.asShapeMut(), stream);

        stream.read(&self.offset);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.offset_center_of_mass);
        f.construct = ShapeFunctions.constructor(OffsetCenterOfMassShape);
        f.color = Color.cyan;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.offset_center_of_mass, s, collideOffsetCenterOfMassVsShape);
            r.registerCollideShape(s, .offset_center_of_mass, collideShapeVsOffsetCenterOfMass);
            r.registerCastShape(.offset_center_of_mass, s, castOffsetCenterOfMassVsShape);
            r.registerCastShape(s, .offset_center_of_mass, castShapeVsOffsetCenterOfMass);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn collideOffsetCenterOfMassVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(OffsetCenterOfMassShape); // Asserts that the sub type is EShapeSubType::OffsetCenterOfMass

        CollisionDispatch.collideShapeVsShape(shape1.innerShape(), shape2, scale1, scale2, center_of_mass_transform1.preTranslated(scale1.negate().mul(shape1.offset)), center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn collideShapeVsOffsetCenterOfMass(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(OffsetCenterOfMassShape); // Asserts that the sub type is EShapeSubType::OffsetCenterOfMass

        CollisionDispatch.collideShapeVsShape(shape1, shape2.innerShape(), scale1, scale2, center_of_mass_transform1, center_of_mass_transform2.preTranslated(scale2.negate().mul(shape2.offset)), sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn castOffsetCenterOfMassVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        // Fetch offset center of mass shape from cast shape
        const shape1 = shape_cast.shape.cast(OffsetCenterOfMassShape); // Asserts that the sub type is EShapeSubType::OffsetCenterOfMass

        // Transform the shape cast and update the shape
        const local_shape_cast = ShapeCast.init(shape1.innerShape(), shape_cast.scale, shape_cast.center_of_mass_start.preTranslated(shape_cast.scale.negate().mul(shape1.offset)), shape_cast.direction);

        CollisionDispatch.castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    fn castShapeVsOffsetCenterOfMass(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(OffsetCenterOfMassShape); // Asserts that the sub type is EShapeSubType::OffsetCenterOfMass

        // Transform the shape cast
        const local_shape_cast = shape_cast.postTransformed(Mat44.translation(scale.mul(shape.offset)));

        CollisionDispatch.castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape.innerShape(), scale, shape_filter, center_of_mass_transform2.preTranslated(scale.negate().mul(shape.offset)), sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's OffsetCenterOfMassShapeTests are in ZoltTests/Physics, the bit exact comparison with Jolt in
// ZoltParity/Physics/DecoratedParity.zig)

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
const BoxShapeFile = @import("BoxShape.zig");
const BoxShape = BoxShapeFile.BoxShape;
const BoxShapeSettings = BoxShapeFile.BoxShapeSettings;
const ScaleHelpers = @import("ScaleHelpers.zig");

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

test "OffsetCenterOfMassShape: settings, Jolt's error texts, cached results and out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // Default constructor: no inner shape
    {
        var settings = OffsetCenterOfMassShapeSettings.initDefault(allocator);
        defer settings.deinit();
        try expect(settings.offset.eql(Vec3.zero()));
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Inner shape is null!", result.getError());
    }

    // An error of the child is forwarded
    {
        const box_settings = try BoxShapeSettings.create(allocator, Vec3.one(), .{ .convex_radius = -1 });
        var settings = OffsetCenterOfMassShapeSettings.init(allocator, Vec3.init(1, 2, 3), box_settings.asShapeSettings());
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid convex radius", result.getError());
    }

    // A concrete inner shape (heap settings through createPtr), user data
    {
        var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.one(), .{})).asShape());
        defer box_ref.deinit();
        const settings = try OffsetCenterOfMassShapeSettings.createPtr(allocator, Vec3.init(0, -1, 0), box_ref.get());
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        defer settings_ref.deinit();
        settings.asShapeSettings().user_data = 3;
        var result = try settings.createShape(allocator);
        defer result.deinit();
        const offset = result.getPtr().?.cast(OffsetCenterOfMassShape);
        try expect(offset.getOffset().eql(Vec3.init(0, -1, 0)) and offset.getCenterOfMass().eql(Vec3.init(0, -1, 0)));
        try testing.expectEqual(@as(u64, 3), offset.asShape().getUserData());
        try testing.expectEqual(ShapeSubType.offset_center_of_mass, offset.asShape().getSubType());
        try expect(offset.base.getInnerShape() == box_ref.get());
    }

    // Out of memory is returned and not cached: the offset center of mass shape and its child are allocated
    {
        const sphere_settings = try SphereShapeSettings.create(allocator, 1.0, .{});
        var settings = OffsetCenterOfMassShapeSettings.init(allocator, Vec3.init(1, 2, 3), sphere_settings.asShapeSettings());
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
            try expect(result.getPtr().?.getCenterOfMass().eql(Vec3.init(1, 2, 3)));
            sphere_settings.asShapeSettings().clearCachedResult();
            settings.asShapeSettings().clearCachedResult();
            break;
        }
    }
}

test "OffsetCenterOfMassShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    try testing.expectError(error.OutOfMemory, OffsetCenterOfMassShape.create(failing.allocator(), sphere.asShape(), Vec3.zero()));
    try testing.expectError(error.OutOfMemory, OffsetCenterOfMassShapeSettings.create(failing.allocator(), Vec3.zero(), null));
    try testing.expectError(error.OutOfMemory, OffsetCenterOfMassShapeSettings.createPtr(failing.allocator(), Vec3.zero(), sphere.asShape()));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.offset_center_of_mass).construct.?(failing.allocator()));

    // Concrete inner shape: only the offset center of mass shape is allocated
    var settings = OffsetCenterOfMassShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), sphere.asShape());
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
    var offset = OffsetCenterOfMassShape.init(allocator, sphere.asShape(), Vec3.init(1, 2, 3));
    offset.asShape().setEmbedded();
    defer offset.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    try testing.expectError(error.OutOfMemory, restoreFromBuffer(failing.allocator(), saveToBuffer(offset.asShape(), &buffer)));
}

test "OffsetCenterOfMassShape: bounds, center of mass, inner radius, mass properties, volume, scales, sub shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    box.asShapeMut().setUserData(21);
    const offset_value = Vec3.init(0.5, -1, 2);
    var offset = OffsetCenterOfMassShape.init(allocator, box.asShape(), offset_value);
    offset.asShape().setEmbedded();
    defer offset.asShapeMut().deinit();
    const shape = offset.asShape();

    try expect(shape.getType() == .decorated and shape.getSubType() == .offset_center_of_mass);
    try expect(shape.getCenterOfMass().eql(offset_value));
    try expect(shape.getLocalBounds().eql(.init(Vec3.init(-1.5, -1, -5), Vec3.init(0.5, 3, 1))));
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.3), Vec3.init(4, 5, 6));
    const scale = Vec3.init(1, -2, 0.5);
    try expect(shape.getWorldSpaceBounds(transform, scale).eql(box.asShape().getWorldSpaceBounds(transform.preTranslated(scale.negate().mul(offset_value)), scale)));
    try testing.expectEqual(box.asShape().getInnerRadius(), shape.getInnerRadius());
    try testing.expectEqual(box.asShape().getVolume(), shape.getVolume());
    var expected_mass = box.asShape().getMassProperties();
    expected_mass.translate(offset_value);
    const mass = shape.getMassProperties();
    try testing.expectEqual(expected_mass.mass, mass.mass);
    try expect(mass.inertia.eql(expected_mass.inertia));
    try testing.expectEqual(@as(usize, @sizeOf(OffsetCenterOfMassShape)), shape.getStats().size_bytes);

    // DecoratedShape.impl: the inner shape decides about the scale
    try expect(shape.isValidScale(Vec3.init(1, -2, 3)) and !shape.isValidScale(Vec3.init(1, 0, 3)));
    try expect(shape.makeScaleValid(Vec3.init(0, -2, 3)).eql(Vec3.init(ScaleHelpers.min_scale, -2, 3)));
    const id = SubShapeIDCreator.pushID(.{}, 1, 1).getID();
    try expect(shape.getLeafShape(id).shape == box.asShape());
    try testing.expectEqual(@as(u64, 21), shape.getSubShapeUserData(id));

    // The direct child: the position is moved back by the scaled, rotated offset
    const rotation = Quat.rotation(Vec3.axisX(), 0.5);
    var sub = shape.getSubShapeTransformedShape(id, Vec3.init(1, 2, 3), rotation, scale);
    defer sub.transformed_shape.deinit();
    try expect(sub.remainder.eql(id));
    try expect(sub.transformed_shape.shape.get() == box.asShape());
    try expect(sub.transformed_shape.getShapeScale().eql(scale) and sub.transformed_shape.shape_rotation.eql(rotation));
    try expect(sub.transformed_shape.shape_position_com.eql(RVec3.fromVec3(Vec3.init(1, 2, 3).sub(rotation.mulVec3(scale.mul(offset_value))))));

    // Surface normal: the point is moved into the space of the child
    try expect(shape.getSurfaceNormal(.empty, Vec3.init(0.5, 1, -3)).eql(Vec3.axisX()));

    // Supporting face: the transform of the child
    var face: Shape.SupportingFace = .empty;
    var expected_face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.init(0.1, 1, 0.2), scale, transform, &face);
    box.asShape().getSupportingFace(.empty, Vec3.init(0.1, 1, 0.2), scale, transform.preTranslated(scale.negate().mul(offset_value)), &expected_face);
    try testing.expectEqual(@as(u32, 4), face.len);
    try expect(face.eql(&expected_face));
}

test "OffsetCenterOfMassShape: ray casts, collide point, shape filters and the collector context" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var offset = OffsetCenterOfMassShape.init(allocator, sphere.asShape(), Vec3.init(2, 0, 0));
    offset.asShape().setEmbedded();
    defer offset.asShapeMut().deinit();
    const shape = offset.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 1, 1);

    // Single hit: the sphere is at -offset relative to the center of mass
    var hit: RayCastResult = .{};
    try expect(shape.castRay(.init(Vec3.init(-6, 0, 0), Vec3.init(8, 0, 0)), creator, &hit));
    try testing.expectEqual(@as(f32, 0.375), hit.fraction);
    try expect(!shape.castRay(.init(Vec3.init(-6, 1.5, 0), Vec3.init(8, 0, 0)), creator, &hit));

    // Collector
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(4), .{});
    hits.base.setContext(&context);
    shape.castRayCollector(.init(Vec3.init(-6, 0, 0), Vec3.init(8, 0, 0)), &settings, creator, &hits.base, &.{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.625), hits.hits.items[1].fraction);
    try expect(hits.hits.items[1].body_id.eql(.init(4)) and hits.hits.items[1].sub_shape_id2.eql(creator.getID()));
    var calls: u32 = 0;
    for ([_]*const Shape{ shape, sphere.asShape() }) |rejected| {
        hits.reset();
        const filter: RejectShapeFilter = .{ .rejected = rejected, .calls = &calls };
        shape.castRayCollector(.init(Vec3.init(-6, 0, 0), Vec3.init(8, 0, 0)), &settings, creator, &hits.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    }
    try testing.expectEqual(@as(u32, 3), calls);

    // Collide point
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.init(-2.9, 0, 0), creator, &points.base, &.{});
    shape.collidePoint(Vec3.zero(), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    const reject: RejectShapeFilter = .{ .rejected = shape, .calls = &calls };
    shape.collidePoint(Vec3.init(-2.9, 0, 0), creator, &points.base, &reject.base);
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
}

test "OffsetCenterOfMassShape: collide and cast through CollisionDispatch" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const offset1 = Vec3.init(0.25, 0, 0);
    const offset2 = Vec3.init(0, -0.5, 0.25);
    var offset_sphere = OffsetCenterOfMassShape.init(allocator, sphere.asShape(), offset1);
    offset_sphere.asShape().setEmbedded();
    defer offset_sphere.asShapeMut().deinit();
    var offset_box = OffsetCenterOfMassShape.init(allocator, box.asShape(), offset2);
    offset_box.asShape().setEmbedded();
    defer offset_box.asShapeMut().deinit();

    // The registered functions
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.offset_center_of_mass, .sphere) == &OffsetCenterOfMassShape.collideOffsetCenterOfMassVsShape);
    try expect(r.getCollideShape(.box, .offset_center_of_mass) == &OffsetCenterOfMassShape.collideShapeVsOffsetCenterOfMass);
    try expect(r.getCollideShape(.offset_center_of_mass, .offset_center_of_mass) == &OffsetCenterOfMassShape.collideShapeVsOffsetCenterOfMass); // Within one loop iteration the second write wins
    try expect(r.getCollideShape(.offset_center_of_mass, .rotated_translated) == &OffsetCenterOfMassShape.collideOffsetCenterOfMassVsShape);
    try expect(r.getCollideShape(.rotated_translated, .offset_center_of_mass) == &OffsetCenterOfMassShape.collideShapeVsOffsetCenterOfMass);
    try expect(r.getCastShape(.offset_center_of_mass, .user3) == &OffsetCenterOfMassShape.castOffsetCenterOfMassVsShape);
    try expect(r.getCastShape(.mesh, .offset_center_of_mass) == &OffsetCenterOfMassShape.castShapeVsOffsetCenterOfMass);
    try expect(r.getCastShape(.offset_center_of_mass, .scaled) != &OffsetCenterOfMassShape.castOffsetCenterOfMassVsShape); // ScaledShape registers later
    try expect(ShapeFunctions.get(.offset_center_of_mass).color.eql(Color.cyan));
    try expect(ShapeFunctions.get(.offset_center_of_mass).construct != null);

    // The inner shapes with the transforms moved by the scaled offsets (same operations, bit exact)
    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    const t1 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.2), Vec3.init(0.5, 1, 0));
    const t2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.4), Vec3.init(2, 0.5, 1));
    const scale1 = Vec3.replicate(-1.5);
    const scale2 = Vec3.init(1, 2, -1);
    const t1_inner = t1.preTranslated(scale1.negate().mul(offset1));
    const t2_inner = t2.preTranslated(scale2.negate().mul(offset2));
    const Pair = struct { s1: *const Shape, s2: *const Shape, et1: Mat44, et2: Mat44 };
    const pairs = [_]Pair{
        .{ .s1 = offset_sphere.asShape(), .s2 = box.asShape(), .et1 = t1_inner, .et2 = t2 },
        .{ .s1 = sphere.asShape(), .s2 = offset_box.asShape(), .et1 = t1, .et2 = t2_inner },
        .{ .s1 = offset_sphere.asShape(), .s2 = offset_box.asShape(), .et1 = t1_inner, .et2 = t2_inner },
    };
    for (pairs) |p| {
        var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, scale1, scale2, t1, t2, .{}, .{}, &settings, &hits.base, &.{});
        try hits.checkError();
        var expected = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.collideShapeVsShape(sphere.asShape(), box.asShape(), scale1, scale2, p.et1, p.et2, .{}, .{}, &settings, &expected.base, &.{});
        try expected.checkError();
        try testing.expectEqual(@as(usize, 1), expected.hits.items.len);
        try testing.expectEqual(expected.hits.items.len, hits.hits.items.len);
        const a = &hits.hits.items[0];
        const b = &expected.hits.items[0];
        try expect(a.contact_point_on1.eql(b.contact_point_on1) and a.contact_point_on2.eql(b.contact_point_on2) and a.penetration_axis.eql(b.penetration_axis));
        try testing.expectEqual(b.penetration_depth, a.penetration_depth);
        try expect(a.shape1_face.eql(&b.shape1_face) and a.shape2_face.eql(&b.shape2_face));

        // The filter sees the inner shapes
        var calls: u32 = 0;
        const filter: RejectShapeFilter = .{ .rejected = sphere.asShape(), .calls = &calls };
        var filtered = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer filtered.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, scale1, scale2, t1, t2, .{}, .{}, &settings, &filtered.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), filtered.hits.items.len);
    }

    // Casts with the offset center of mass shape on either side
    const cast_settings: ShapeCastSettings = .{};
    const start = Mat44.translation(Vec3.init(-6, 0.25, 0));
    const CastPair = struct { cast: ShapeCast, s: *const Shape, expected_cast: ShapeCast, e: *const Shape };
    const cast_pairs = [_]CastPair{
        .{ .cast = .init(offset_sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = box.asShape(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start.preTranslated(offset1.negate()), Vec3.init(10, 0, 0)), .e = box.asShape() },
        .{ .cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = offset_box.asShape(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = box.asShape() },
    };
    const t2_cast = Mat44.translation(Vec3.init(2, 0.5, 1));
    for (cast_pairs, 0..) |p, i| {
        var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.cast, &cast_settings, p.s, Vec3.one(), &.{}, t2_cast, .{}, .{}, &hits.base);
        try hits.checkError();
        var expected = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.expected_cast, &cast_settings, p.e, Vec3.one(), &.{}, if (i == 1) t2_cast.preTranslated(offset2.negate()) else t2_cast, .{}, .{}, &expected.base);
        try expected.checkError();
        try testing.expectEqual(@as(usize, 1), expected.hits.items.len);
        try testing.expectEqual(expected.hits.items.len, hits.hits.items.len);
        try testing.expectApproxEqAbs(expected.hits.items[0].fraction, hits.hits.items[0].fraction, 1.0e-6);
        try expect(hits.hits.items[0].base.penetration_axis.normalized().isClose(expected.hits.items[0].base.penetration_axis.normalized(), .{ .max_dist_sq = 1.0e-8 }));
    }
}

test "OffsetCenterOfMassShape: submerged volume, soft body vertices, collect and transform shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const offset_value = Vec3.init(1, 0.5, -0.25);
    var offset = OffsetCenterOfMassShape.init(allocator, box.asShape(), offset_value);
    offset.asShape().setEmbedded();
    defer offset.asShapeMut().deinit();
    const shape = offset.asShape();
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.7), Vec3.init(1, 2, 3));
    const scale = Vec3.init(2, -1, 1);
    const inner_transform = transform.preTranslated(scale.negate().mul(offset_value));

    // Submerged volume
    const surface = Plane.fromPointAndNormal(Vec3.init(0, 2.5, 0), Vec3.axisY());
    const v = shape.getSubmergedVolume(transform, scale, surface);
    const expected_v = box.asShape().getSubmergedVolume(inner_transform, scale, surface);
    try testing.expectEqual(expected_v.total_volume, v.total_volume);
    try testing.expectEqual(expected_v.submerged_volume, v.submerged_volume);
    try expect(v.center_of_buoyancy.eql(expected_v.center_of_buoyancy));

    // Soft body vertices
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
            shape.collideSoftBodyVertices(transform, scale, &vertices, 3, 5)
        else
            box.asShape().collideSoftBodyVertices(inner_transform, scale, &vertices, 3, 5);
    }
    for (0..3) |i| {
        try testing.expectEqual(penetrations[1][i], penetrations[0][i]);
        try testing.expectEqual(indices[1][i], indices[0][i]);
        try expect(planes[0][i].normal_and_constant.eql(planes[1][i].normal_and_constant));
    }

    // Collect transformed shapes: the position of the leaf is moved back by the scaled, rotated offset
    var collected = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collected.deinit();
    const creator = SubShapeIDCreator.pushID(.{}, 3, 2);
    const rotation = Quat.rotation(Vec3.axisZ(), 0.1);
    shape.collectTransformedShapes(AABox.biggest(), Vec3.init(1, 2, 3), rotation, scale, creator, &collected.base, &.{});
    try collected.checkError();
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);
    const ts = &collected.hits.items[0];
    try expect(ts.shape.get() == box.asShape() and ts.getShapeScale().eql(scale) and ts.shape_rotation.eql(rotation));
    try expect(ts.shape_position_com.eql(RVec3.fromVec3(Vec3.init(1, 2, 3).sub(rotation.mulVec3(scale.mul(offset_value))))));
    var calls: u32 = 0;
    const reject: RejectShapeFilter = .{ .rejected = shape, .calls = &calls };
    shape.collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), creator, &collected.base, &reject.base);
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);

    // Transform shape: the leaf gets the transform moved by the (unscaled) offset
    var transformed = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer transformed.deinit();
    shape.transformShape(transform, &transformed.base);
    try transformed.checkError();
    try testing.expectEqual(@as(usize, 1), transformed.hits.items.len);
    try expect(transformed.hits.items[0].shape.get() == box.asShape());
    try expect(transformed.hits.items[0].shape_position_com.toVec3().isClose(transform.preTranslated(offset_value.negate()).getTranslation(), .{ .max_dist_sq = 1.0e-10 }));

    // getTrianglesStart / Next assert (Jolt: "Cannot call on non-leaf shapes"), without asserts they return nothing
    if (!Core.enable_asserts) {
        var tri_context: Shape.GetTrianglesContext = .{};
        shape.getTrianglesStart(&tri_context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
        var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
        try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&tri_context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    }
}

test "OffsetCenterOfMassShape: binary state, sub shape state, restoreFromBinaryState and saveWithChildren" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 0.5, .{})).asShape());
    defer sphere_ref.deinit();
    const offset = try OffsetCenterOfMassShape.create(allocator, sphere_ref.get().?, Vec3.init(1, -2, 3));
    offset.asShapeMut().setUserData(1);
    var offset_ref = RefConst(Shape).init(offset.asShape());
    defer offset_ref.deinit();

    // Sub type, user data, offset
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(offset.asShape(), &buffer);
    try testing.expectEqual(@as(usize, 1 + 8 + 12), bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.offset_center_of_mass), bytes[0]);

    var result = try restoreFromBuffer(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?.castMut(OffsetCenterOfMassShape);
    try expect(restored.getOffset().eql(Vec3.init(1, -2, 3)));
    try testing.expectEqual(@as(u64, 1), restored.asShape().getUserData());
    try expect(restored.base.getInnerShape() == null);
    restored.asShapeMut().restoreSubShapeState(&.{sphere_ref}); // Takes its own reference
    try expect(restored.getCenterOfMass().eql(Vec3.init(1, -2, 3)));
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
    try offset.asShape().saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
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
    const graph_offset = graph_result.getPtr().?.cast(OffsetCenterOfMassShape);
    try expect(graph_offset.getOffset().eql(Vec3.init(1, -2, 3)));
    try expect(graph_offset.asShape().getLocalBounds().eql(offset.asShape().getLocalBounds()));
}
