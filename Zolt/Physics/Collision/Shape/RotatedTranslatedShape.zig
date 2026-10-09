//! Port of: Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h, Jolt/Physics/Collision/Shape/RotatedTranslatedShape.cpp
//! Status: complete
//!
//! A concrete decorated shape (Docs/Zolt/CollisionArchitecture.md, section 2 "Decorated shapes"):
//! - `RotatedTranslatedShapeSettings` / `RotatedTranslatedShape` embed `DecoratedShapeSettings` / `DecoratedShape` as
//!   `base` (D1); `overrides` lists every C++ `override` of the header in header order. The functions that
//!   RotatedTranslatedShape does not override come from `DecoratedShape.impl` (and `Shape.impl`).
//! - Constructors: `initDefault` (default constructor), `initFromSettings` (D3), `init` / `create`
//!   (RotatedTranslatedShape(Vec3Arg, QuatArg, const Shape *), on the stack / `new`). The two settings constructors are
//!   `init` (const ShapeSettings *) and `initPtr` (const Shape *), with `create` / `createPtr` for `new`.
//! - Members that Jolt leaves uninitialized (`mPosition` / `mRotation` of the settings, `mCenterOfMass`, `mRotation` and
//!   `mIsRotationIdentity` of a default constructed shape) are zero / identity here; the constructors and the restore
//!   set them like in Jolt.
//! - `sRegister` is `register` (D4): the collide / cast functions with a rotated translated shape on either side (and
//!   on both sides) forward to the inner shape with the rotation applied, private like in Jolt.
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
const math = @import("../../../Math/Math.zig");
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

/// Class that constructs a RotatedTranslatedShape
pub const RotatedTranslatedShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, RotatedTranslatedShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,
    /// Position of the sub shape (uninitialized in Jolt)
    position: Vec3 = Vec3.zero(),
    /// Rotation of the sub shape (uninitialized in Jolt)
    rotation: Quat = Quat.identity(),

    /// Constructor (the default constructor, for deserialization)
    pub fn initDefault(allocator: Allocator) RotatedTranslatedShapeSettings {
        return .{ .base = .initDefault(RotatedTranslatedShapeSettings, allocator) };
    }

    /// Construct with shape settings, can be serialized (settings on the stack: `defer settings.deinit()`).
    pub fn init(allocator: Allocator, position: Vec3, rotation: Quat, shape: ?*ShapeSettings) RotatedTranslatedShapeSettings {
        return .{ .base = .init(RotatedTranslatedShapeSettings, allocator, shape), .position = position, .rotation = rotation };
    }

    /// Variant that uses a concrete shape, which means this object cannot be serialized.
    pub fn initPtr(allocator: Allocator, position: Vec3, rotation: Quat, shape: ?*const Shape) RotatedTranslatedShapeSettings {
        return .{ .base = .initPtr(RotatedTranslatedShapeSettings, allocator, shape), .position = position, .rotation = rotation };
    }

    /// new RotatedTranslatedShapeSettings(inPosition, inRotation, inShape): reference count 0, freed by the last release()
    pub fn create(allocator: Allocator, position: Vec3, rotation: Quat, shape: ?*ShapeSettings) Allocator.Error!*RotatedTranslatedShapeSettings {
        const self = try allocator.create(RotatedTranslatedShapeSettings);
        self.* = .init(allocator, position, rotation, shape);
        return self;
    }

    /// new RotatedTranslatedShapeSettings(inPosition, inRotation, const Shape *inShape): reference count 0, freed by the last release()
    pub fn createPtr(allocator: Allocator, position: Vec3, rotation: Quat, shape: ?*const Shape) Allocator.Error!*RotatedTranslatedShapeSettings {
        const self = try allocator.create(RotatedTranslatedShapeSettings);
        self.* = .initPtr(allocator, position, rotation, shape);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *RotatedTranslatedShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *RotatedTranslatedShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *RotatedTranslatedShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(RotatedTranslatedShape, self, allocator);
    }
};

/// A rotated translated shape will rotate and translate a child shape.
/// Shifts the child object so that it is centered around the center of mass.
pub const RotatedTranslatedShape = struct {
    /// Concrete class: `Shape.cast(RotatedTranslatedShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .rotated_translated;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .getCenterOfMass, .getLocalBounds, .getWorldSpaceBounds, .getInnerRadius, .getMassProperties, .getSubShapeTransformedShape, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .saveBinaryState, .getStats, .getVolume, .isValidScale, .makeScaleValid, .restoreBinaryState };

    base: DecoratedShape,
    /// If rotation is close to identity (put here because it falls in padding bytes)
    is_rotation_identity: bool = true,
    /// Position of the center of mass
    center_of_mass: Vec3 = Vec3.zero(),
    /// Rotation of the child shape
    rotation: Quat = Quat.identity(),

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// RotatedTranslatedShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by createCached
    pub fn initDefault(allocator: Allocator) RotatedTranslatedShape {
        return .{ .base = .init(RotatedTranslatedShape, allocator, shape_sub_type, null) };
    }

    /// RotatedTranslatedShape(const RotatedTranslatedShapeSettings &inSettings, ShapeResult &outResult): base part first, then the C++ body
    pub fn initFromSettings(self: *RotatedTranslatedShape, settings: *const RotatedTranslatedShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        try self.base.initFromSettings(&settings.base, result, allocator);
        if (result.hasError())
            return;

        // Calculate center of mass position
        self.center_of_mass = settings.position.add(settings.rotation.mulVec3(self.innerShape().getCenterOfMass()));

        // Store rotation (position is always zero because we center around the center of mass)
        self.rotation = settings.rotation;
        self.is_rotation_identity = self.rotation.isClose(Quat.identity(), .{});

        result.set(.init(self.asShapeMut()));
    }

    /// RotatedTranslatedShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape) (on the stack / as a member:
    /// `asShape().setEmbedded()` before taking references, `asShapeMut().deinit()` at the end). Adds a reference to
    /// `shape`.
    pub fn init(allocator: Allocator, position: Vec3, rotation: Quat, shape: *const Shape) RotatedTranslatedShape {
        var self: RotatedTranslatedShape = .{ .base = .init(RotatedTranslatedShape, allocator, shape_sub_type, shape) };

        // Calculate center of mass position
        self.center_of_mass = position.add(rotation.mulVec3(self.innerShape().getCenterOfMass()));

        // Store rotation (position is always zero because we center around the center of mass)
        self.rotation = rotation;
        self.is_rotation_identity = self.rotation.isClose(Quat.identity(), .{});
        return self;
    }

    /// new RotatedTranslatedShape(inPosition, inRotation, inShape): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, position: Vec3, rotation: Quat, shape: *const Shape) Allocator.Error!*RotatedTranslatedShape {
        const self = try allocator.create(RotatedTranslatedShape);
        self.* = .init(allocator, position, rotation, shape);
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const RotatedTranslatedShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *RotatedTranslatedShape) *Shape {
        return &self.base.base;
    }

    /// mInnerShape (never null after construction or a complete restore)
    fn innerShape(self: *const RotatedTranslatedShape) *const Shape {
        return self.base.inner_shape.get().?;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Non virtual functions

    /// Access the rotation that is applied to the inner shape
    pub fn getRotation(self: *const RotatedTranslatedShape) Quat {
        return self.rotation;
    }

    /// Access the translation that has been applied to the inner shape
    pub fn getPosition(self: *const RotatedTranslatedShape) Vec3 {
        return self.center_of_mass.sub(self.rotation.mulVec3(self.innerShape().getCenterOfMass()));
    }

    /// Transform the scale to the local space of the child shape
    pub fn transformScale(self: *const RotatedTranslatedShape, scale: Vec3) Vec3 {
        // We don't need to transform uniform scale or if the rotation is identity
        if (self.is_rotation_identity or ScaleHelpers.isUniformScale(scale))
            return scale;

        return ScaleHelpers.rotateScale(self.rotation, scale);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::GetCenterOfMass
    pub fn getCenterOfMass(self: *const RotatedTranslatedShape) Vec3 {
        return self.center_of_mass;
    }

    // See Shape::GetLocalBounds
    pub fn getLocalBounds(self: *const RotatedTranslatedShape) AABox {
        return self.innerShape().getLocalBounds().transformed(Mat44.rotationQuat(self.rotation));
    }

    // See Shape::GetWorldSpaceBounds (the DMat44 overload, `using Shape::GetWorldSpaceBounds`, is Shape.getWorldSpaceBoundsDMat44)
    pub fn getWorldSpaceBounds(self: *const RotatedTranslatedShape, center_of_mass_transform: Mat44, scale: Vec3) AABox {
        const transform = center_of_mass_transform.mul(Mat44.rotationQuat(self.rotation));
        return self.innerShape().getWorldSpaceBounds(transform, self.transformScale(scale));
    }

    // See Shape::GetInnerRadius
    pub fn getInnerRadius(self: *const RotatedTranslatedShape) f32 {
        return self.innerShape().getInnerRadius();
    }

    // See Shape::GetMassProperties
    pub fn getMassProperties(self: *const RotatedTranslatedShape) MassProperties {
        // Rotate inertia of child into place
        var p = self.innerShape().getMassProperties();
        p.rotate(Mat44.rotationQuat(self.rotation));
        return p;
    }

    // See Shape::GetSubShapeTransformedShape
    pub fn getSubShapeTransformedShape(self: *const RotatedTranslatedShape, sub_shape_id: SubShapeID, position_com: Vec3, rotation: Quat, scale: Vec3) Shape.SubShapeTransformedShape {
        // We don't use any bits in the sub shape ID
        const remainder = sub_shape_id;

        var ts = TransformedShape.init(RVec3.fromVec3(position_com), rotation.mul(self.rotation), self.innerShape(), .invalid, .{});
        ts.setShapeScale(self.transformScale(scale));
        return .{ .transformed_shape = ts, .remainder = remainder };
    }

    // See Shape::GetSurfaceNormal
    pub fn getSurfaceNormal(self: *const RotatedTranslatedShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        // Transform surface position to local space and pass call on
        const transform = Mat44.rotationQuat(self.rotation.conjugated());
        const normal = self.innerShape().getSurfaceNormal(sub_shape_id, transform.mulVec3(local_surface_position));

        // Transform normal to this shape's space
        return transform.multiply3x3Transposed(normal);
    }

    // See Shape::GetSupportingFace
    pub fn getSupportingFace(self: *const RotatedTranslatedShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        const transform = Mat44.rotationQuat(self.rotation);
        self.innerShape().getSupportingFace(sub_shape_id, transform.multiply3x3Transposed(direction), self.transformScale(scale), center_of_mass_transform.mul(transform), out_vertices);
    }

    // See Shape::GetSubmergedVolume
    pub fn getSubmergedVolume(self: *const RotatedTranslatedShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        // Get center of mass transform of child
        const transform = center_of_mass_transform.mul(Mat44.rotationQuat(self.rotation));

        // Recurse to child
        return self.innerShape().getSubmergedVolume(transform, self.transformScale(scale), surface);
    }

    // TODO(debug_renderer): Draw, DrawGetSupportFunction, DrawGetSupportingFace (pass the call on with
    // inCenterOfMassTransform * Mat44::sRotation(mRotation) and TransformScale(inScale))

    // See Shape::CastRay
    pub fn castRay(self: *const RotatedTranslatedShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        // Transform the ray
        const transform = Mat44.rotationQuat(self.rotation.conjugated());
        const local_ray = ray.transformed(transform);

        return self.innerShape().castRay(local_ray, sub_shape_id_creator, hit);
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const RotatedTranslatedShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Transform the ray
        const transform = Mat44.rotationQuat(self.rotation.conjugated());
        const local_ray = ray.transformed(transform);

        return self.innerShape().castRayCollector(local_ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const RotatedTranslatedShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        // Transform the point
        const transform = Mat44.rotationQuat(self.rotation.conjugated());
        self.innerShape().collidePoint(transform.mulVec3(point), sub_shape_id_creator, collector, shape_filter);
    }

    // See: Shape::CollideSoftBodyVertices
    pub fn collideSoftBodyVertices(self: *const RotatedTranslatedShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.innerShape().collideSoftBodyVertices(center_of_mass_transform.mul(Mat44.rotationQuat(self.rotation)), scale, vertices, num_vertices, colliding_shape_index);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const RotatedTranslatedShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        self.innerShape().collectTransformedShapes(box, position_com, rotation.mul(self.rotation), self.transformScale(scale), sub_shape_id_creator, collector, shape_filter);
    }

    // See Shape::TransformShape
    pub fn transformShape(self: *const RotatedTranslatedShape, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        self.innerShape().transformShape(center_of_mass_transform.mul(Mat44.rotationQuat(self.rotation)), collector);
    }

    // See Shape::GetTrianglesStart
    pub fn getTrianglesStart(self: *const RotatedTranslatedShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, context, box, position_com, rotation, scale };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
    }

    // See Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *const RotatedTranslatedShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        if (Core.enable_asserts) @panic("Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!");
        return 0;
    }

    // See Shape: C++ `DecoratedShape::SaveBinaryState(inStream)` is Shape's version (DecoratedShape does not override it)
    pub fn saveBinaryState(self: *const RotatedTranslatedShape, stream: StreamOut) void {
        Shape.impl.saveBinaryState(self.asShape(), stream);

        stream.write(self.center_of_mass);
        stream.write(self.rotation);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const RotatedTranslatedShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(RotatedTranslatedShape), 0);
    }

    // See Shape::GetVolume
    pub fn getVolume(self: *const RotatedTranslatedShape) f32 {
        return self.innerShape().getVolume();
    }

    // See Shape::IsValidScale: C++ `Shape::IsValidScale` is the explicit base call
    pub fn isValidScale(self: *const RotatedTranslatedShape, scale: Vec3) bool {
        if (!Shape.impl.isValidScale(self.asShape(), scale))
            return false;

        if (self.is_rotation_identity or ScaleHelpers.isUniformScale(scale))
            return self.innerShape().isValidScale(scale);

        if (!ScaleHelpers.canScaleBeRotated(self.rotation, scale))
            return false;

        return self.innerShape().isValidScale(ScaleHelpers.rotateScale(self.rotation, scale));
    }

    // See Shape::MakeScaleValid
    pub fn makeScaleValid(self: *const RotatedTranslatedShape, scale: Vec3) Vec3 {
        const s = ScaleHelpers.makeNonZeroScale(scale);

        if (self.is_rotation_identity or ScaleHelpers.isUniformScale(s))
            return self.innerShape().makeScaleValid(s);

        if (ScaleHelpers.canScaleBeRotated(self.rotation, s))
            return ScaleHelpers.rotateScale(self.rotation.conjugated(), self.innerShape().makeScaleValid(ScaleHelpers.rotateScale(self.rotation, s)));

        const abs_uniform_scale = ScaleHelpers.makeUniformScale(s.abs());
        const uniform_scale = s.getSign().mul(abs_uniform_scale);
        if (ScaleHelpers.canScaleBeRotated(self.rotation, uniform_scale))
            return uniform_scale;

        return abs_uniform_scale.mulScalar(math.sign(s.getX()));
    }

    // See: Shape::RestoreBinaryState: C++ `DecoratedShape::RestoreBinaryState(inStream)` is Shape's version
    pub fn restoreBinaryState(self: *RotatedTranslatedShape, stream: StreamIn) Allocator.Error!void {
        try Shape.impl.restoreBinaryState(self.asShapeMut(), stream);

        stream.read(&self.center_of_mass);
        stream.read(&self.rotation);
        self.is_rotation_identity = self.rotation.isClose(Quat.identity(), .{});
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.rotated_translated);
        f.construct = ShapeFunctions.constructor(RotatedTranslatedShape);
        f.color = Color.blue;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.rotated_translated, s, collideRotatedTranslatedVsShape);
            r.registerCollideShape(s, .rotated_translated, collideShapeVsRotatedTranslated);
            r.registerCastShape(.rotated_translated, s, castRotatedTranslatedVsShape);
            r.registerCastShape(s, .rotated_translated, castShapeVsRotatedTranslated);
        }

        r.registerCollideShape(.rotated_translated, .rotated_translated, collideRotatedTranslatedVsRotatedTranslated);
        r.registerCastShape(.rotated_translated, .rotated_translated, castRotatedTranslatedVsRotatedTranslated);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn collideRotatedTranslatedVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated

        // Get world transform of 1
        const transform1 = center_of_mass_transform1.mul(Mat44.rotationQuat(shape1.rotation));

        CollisionDispatch.collideShapeVsShape(shape1.innerShape(), shape2, shape1.transformScale(scale1), scale2, transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn collideShapeVsRotatedTranslated(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated

        // Get world transform of 2
        const transform2 = center_of_mass_transform2.mul(Mat44.rotationQuat(shape2.rotation));

        CollisionDispatch.collideShapeVsShape(shape1, shape2.innerShape(), scale1, shape2.transformScale(scale2), center_of_mass_transform1, transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn collideRotatedTranslatedVsRotatedTranslated(shape1_in: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated
        const shape2 = shape2_in.cast(RotatedTranslatedShape);

        // Get world transform of 1 and 2
        const transform1 = center_of_mass_transform1.mul(Mat44.rotationQuat(shape1.rotation));
        const transform2 = center_of_mass_transform2.mul(Mat44.rotationQuat(shape2.rotation));

        CollisionDispatch.collideShapeVsShape(shape1.innerShape(), shape2.innerShape(), shape1.transformScale(scale1), shape2.transformScale(scale2), transform1, transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
    }

    fn castRotatedTranslatedVsShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        // Fetch rotated translated shape from cast shape
        const shape1 = shape_cast.shape.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated

        // Transform the shape cast and update the shape
        const transform = shape_cast.center_of_mass_start.mul(Mat44.rotationQuat(shape1.rotation));
        const scale1 = shape1.transformScale(shape_cast.scale);
        const local_shape_cast = ShapeCast.init(shape1.innerShape(), scale1, transform, shape_cast.direction);

        CollisionDispatch.castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    fn castShapeVsRotatedTranslated(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated

        // Determine the local transform
        const local_transform = Mat44.rotationQuat(shape.rotation);

        // Transform the shape cast
        const local_shape_cast = shape_cast.postTransformed(local_transform.transposed3x3());

        CollisionDispatch.castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape.innerShape(), shape.transformScale(scale), shape_filter, center_of_mass_transform2.mul(local_transform), sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }

    fn castRotatedTranslatedVsRotatedTranslated(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape1 = shape_cast.shape.cast(RotatedTranslatedShape); // Asserts that the sub type is EShapeSubType::RotatedTranslated
        const shape2 = shape_in.cast(RotatedTranslatedShape);

        // Determine the local transform of shape 2
        const local_transform2 = Mat44.rotationQuat(shape2.rotation);
        const local_transform2_transposed = local_transform2.transposed3x3();

        // Transform the shape cast and update the shape
        const transform = local_transform2_transposed.mul(shape_cast.center_of_mass_start).mul(Mat44.rotationQuat(shape1.rotation));
        const scale1 = shape1.transformScale(shape_cast.scale);
        const local_shape_cast = ShapeCast.init(shape1.innerShape(), scale1, transform, local_transform2_transposed.multiply3x3(shape_cast.direction));

        CollisionDispatch.castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape2.innerShape(), shape2.transformScale(scale), shape_filter, center_of_mass_transform2.mul(local_transform2), sub_shape_id_creator1, sub_shape_id_creator2, collector);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/DecoratedParity.zig)

const testing = std.testing;
const Ref = @import("../../../Core/Reference.zig").Ref;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RegisterTypes = @import("../../../RegisterTypes.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const SphereShapeFile = @import("SphereShape.zig");
const SphereShape = SphereShapeFile.SphereShape;
const SphereShapeSettings = SphereShapeFile.SphereShapeSettings;
const BoxShapeFile = @import("BoxShape.zig");
const BoxShape = BoxShapeFile.BoxShape;
const BoxShapeSettings = BoxShapeFile.BoxShapeSettings;
const ScaledShape = @import("ScaledShape.zig").ScaledShape;
const OffsetCenterOfMassShape = @import("OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;

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

test "RotatedTranslatedShape: settings, Jolt's error texts, cached results and out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // Default constructor: no inner shape
    {
        var settings = RotatedTranslatedShapeSettings.initDefault(allocator);
        defer settings.deinit();
        try expect(settings.position.eql(Vec3.zero()) and settings.rotation.eql(Quat.identity()));
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Inner shape is null!", result.getError());
    }

    // An error of the child is forwarded
    {
        const box_settings = try BoxShapeSettings.create(allocator, Vec3.init(1, -1, 1), .{});
        var settings = RotatedTranslatedShapeSettings.init(allocator, Vec3.init(1, 2, 3), Quat.identity(), box_settings.asShapeSettings());
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid half extent", result.getError());
    }

    // Child settings: the center of mass is the position plus the rotated center of mass of the child
    {
        var box = BoxShape.init(allocator, Vec3.one(), .{});
        box.asShape().setEmbedded();
        defer box.asShapeMut().deinit();
        var offset = OffsetCenterOfMassShape.init(allocator, box.asShape(), Vec3.init(1, 0, 0));
        offset.asShape().setEmbedded();
        defer offset.asShapeMut().deinit();
        const rotation = Quat.rotation(Vec3.axisZ(), 0.5 * math.pi);
        const settings = try RotatedTranslatedShapeSettings.createPtr(allocator, Vec3.init(1, 2, 3), rotation, offset.asShape());
        var settings_ref = Ref(ShapeSettings).init(settings.asShapeSettings());
        defer settings_ref.deinit();
        settings.asShapeSettings().user_data = 5;
        var result = try settings.createShape(allocator);
        defer result.deinit();
        const rt = result.getPtr().?.cast(RotatedTranslatedShape);
        try expect(rt.getCenterOfMass().eql(Vec3.init(1, 2, 3).add(rotation.mulVec3(Vec3.init(1, 0, 0)))));
        try expect(rt.getCenterOfMass().isClose(Vec3.init(1, 3, 3), .{ .max_dist_sq = 1.0e-12 }));
        try expect(rt.getPosition().isClose(Vec3.init(1, 2, 3), .{ .max_dist_sq = 1.0e-12 }));
        try expect(rt.getRotation().eql(rotation) and !rt.is_rotation_identity);
        try testing.expectEqual(@as(u64, 5), rt.asShape().getUserData());
        try testing.expectEqual(ShapeSubType.rotated_translated, rt.asShape().getSubType());
        settings.asShapeSettings().clearCachedResult();
    }

    // Out of memory is returned and not cached: the rotated translated shape and its child are allocated
    {
        const sphere_settings = try SphereShapeSettings.create(allocator, 1.0, .{});
        var settings = RotatedTranslatedShapeSettings.init(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.1), sphere_settings.asShapeSettings());
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
            const rt = result.getPtr().?.cast(RotatedTranslatedShape);
            try expect(rt.getCenterOfMass().eql(Vec3.init(1, 2, 3)) and rt.base.getInnerShape() == sphere_settings.base.base.cached_result.getPtr());
            sphere_settings.asShapeSettings().clearCachedResult();
            settings.asShapeSettings().clearCachedResult();
            break;
        }
    }
}

test "RotatedTranslatedShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    try testing.expectError(error.OutOfMemory, RotatedTranslatedShape.create(failing.allocator(), Vec3.zero(), Quat.identity(), sphere.asShape()));
    try testing.expectError(error.OutOfMemory, RotatedTranslatedShapeSettings.create(failing.allocator(), Vec3.zero(), Quat.identity(), null));
    try testing.expectError(error.OutOfMemory, RotatedTranslatedShapeSettings.createPtr(failing.allocator(), Vec3.zero(), Quat.identity(), sphere.asShape()));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.rotated_translated).construct.?(failing.allocator()));

    // Concrete inner shape: only the rotated translated shape is allocated
    var settings = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.identity(), sphere.asShape());
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
    var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.identity(), sphere.asShape());
    rt.asShape().setEmbedded();
    defer rt.asShapeMut().deinit();
    var buffer: [64]u8 = undefined;
    try testing.expectError(error.OutOfMemory, restoreFromBuffer(failing.allocator(), saveToBuffer(rt.asShape(), &buffer)));
}

test "RotatedTranslatedShape: bounds, center of mass, inner radius, mass properties, volume, stats, sub shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .convex_radius = 0.0 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    box.asShapeMut().setUserData(11);
    const rotation = Quat.rotation(Vec3.axisZ(), 0.5 * math.pi);
    var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), rotation, box.asShape());
    rt.asShape().setEmbedded();
    defer rt.asShapeMut().deinit();
    const shape = rt.asShape();
    const r = Mat44.rotationQuat(rotation);

    try expect(shape.getType() == .decorated and shape.getSubType() == .rotated_translated);
    try expect(shape.getCenterOfMass().eql(Vec3.init(1, 2, 3)));
    try expect(shape.getLocalBounds().eql(box.asShape().getLocalBounds().transformed(r)));
    try expect(shape.getLocalBounds().min.isClose(Vec3.init(-2, -1, -3), .{ .max_dist_sq = 1.0e-12 }));
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.3), Vec3.init(4, 5, 6));
    try expect(shape.getWorldSpaceBounds(transform, Vec3.init(1, 2, 3)).eql(box.asShape().getWorldSpaceBounds(transform.mul(r), rt.transformScale(Vec3.init(1, 2, 3)))));
    try testing.expectEqual(box.asShape().getInnerRadius(), shape.getInnerRadius());
    try testing.expectEqual(box.asShape().getVolume(), shape.getVolume());
    var expected_mass = box.asShape().getMassProperties();
    expected_mass.rotate(r);
    const mass = shape.getMassProperties();
    try testing.expectEqual(expected_mass.mass, mass.mass);
    try expect(mass.inertia.eql(expected_mass.inertia));
    try testing.expectEqual(@as(usize, @sizeOf(RotatedTranslatedShape)), shape.getStats().size_bytes);
    try testing.expectEqual(@as(u32, 0), shape.getStats().num_triangles);

    // TransformScale: uniform scales and the identity rotation pass the scale on, others rotate it
    try expect(rt.transformScale(Vec3.replicate(-2)).eql(Vec3.replicate(-2)));
    try expect(rt.transformScale(Vec3.init(1, 2, 3)).eql(ScaleHelpers.rotateScale(rotation, Vec3.init(1, 2, 3))));
    try expect(rt.transformScale(Vec3.init(1, 2, 3)).isClose(Vec3.init(2, 1, 3), .{ .max_dist_sq = 1.0e-10 }));
    var identity = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.identity(), box.asShape());
    identity.asShape().setEmbedded();
    defer identity.asShapeMut().deinit();
    try expect(identity.is_rotation_identity and identity.transformScale(Vec3.init(1, 2, 3)).eql(Vec3.init(1, 2, 3)));

    // DecoratedShape.impl: the leaf, user data and material of the inner shape
    const id = SubShapeIDCreator.pushID(.{}, 2, 2).getID();
    const leaf = shape.getLeafShape(id);
    try expect(leaf.shape == box.asShape() and leaf.remainder.eql(id));
    try testing.expectEqual(@as(u64, 11), shape.getSubShapeUserData(id));
    try expect(shape.getMaterial(.empty) == PhysicsMaterial.default);

    // The direct child: rotation and scale in the space of the child
    const sub_rotation = Quat.rotation(Vec3.axisX(), 0.5);
    var sub = shape.getSubShapeTransformedShape(id, Vec3.init(1, 2, 3), sub_rotation, Vec3.init(1, 2, 3));
    defer sub.transformed_shape.deinit();
    try expect(sub.remainder.eql(id));
    try expect(sub.transformed_shape.shape.get() == box.asShape());
    try expect(sub.transformed_shape.getShapeScale().eql(rt.transformScale(Vec3.init(1, 2, 3))));
    try expect(sub.transformed_shape.shape_rotation.eql(sub_rotation.mul(rotation)));
    try expect(sub.transformed_shape.shape_position_com.eql(RVec3.init(1, 2, 3)));

    // Surface normal: the +X face of the box is the +Y face of the rotated box
    try expect(shape.getSurfaceNormal(.empty, Vec3.init(0.5, 1, 0)).isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-12 }));
    const local_transform = Mat44.rotationQuat(rotation.conjugated());
    try expect(shape.getSurfaceNormal(.empty, Vec3.init(0.5, 1, 0)).eql(local_transform.multiply3x3Transposed(box.asShape().getSurfaceNormal(.empty, local_transform.mulVec3(Vec3.init(0.5, 1, 0))))));

    // Supporting face: the direction in the space of the child, the transform of the child
    var face: Shape.SupportingFace = .empty;
    var expected_face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, Vec3.init(0.1, 1, 0.2), Vec3.init(1, 2, 3), transform, &face);
    box.asShape().getSupportingFace(.empty, r.multiply3x3Transposed(Vec3.init(0.1, 1, 0.2)), rt.transformScale(Vec3.init(1, 2, 3)), transform.mul(r), &expected_face);
    try testing.expectEqual(@as(u32, 4), face.len);
    try expect(face.eql(&expected_face));
}

test "RotatedTranslatedShape: valid scales (the RotatedTranslatedShape part of Jolt's TestIsValidScale)" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // Test a rotated translated shape that can only be scaled uniformly
    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 2.0, .{})).asShape());
    defer sphere_ref.deinit();
    var rt_settings = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.1 * math.pi), sphere_ref.get());
    defer rt_settings.deinit();
    var rt_result = try rt_settings.asShapeSettings().createShape(allocator);
    defer rt_result.deinit();
    const rt_shape = rt_result.getPtr().?;
    try expect(!rt_shape.isValidScale(Vec3.zero()));
    try expect(rt_shape.isValidScale(Vec3.init(1, 1, 1)));
    try expect(rt_shape.isValidScale(Vec3.init(2, 2, 2)));
    try expect(!rt_shape.isValidScale(Vec3.init(2, 1, 1)));
    try expect(!rt_shape.isValidScale(Vec3.init(1, 2, 1)));
    try expect(!rt_shape.isValidScale(Vec3.init(1, 1, 2)));

    // A box can be scaled in any way: like Jolt's triangle tests (rt_shape3 / rt_shape4) with a box
    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{})).asShape());
    defer box_ref.deinit();

    // Rotations inside the rotated translated of 90 degrees
    var rt3 = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), -0.5 * math.pi), box_ref.get().?);
    rt3.asShape().setEmbedded();
    defer rt3.asShapeMut().deinit();
    try expect(!rt3.asShape().isValidScale(Vec3.zero()));
    try expect(rt3.asShape().isValidScale(Vec3.init(1, 1, 1)));
    try expect(rt3.asShape().isValidScale(Vec3.init(2, 1, 1)));
    try expect(rt3.asShape().isValidScale(Vec3.init(1, 2, 1)));
    try expect(rt3.asShape().isValidScale(Vec3.init(1, 1, 2)));
    try expect(rt3.asShape().makeScaleValid(Vec3.init(2, 1, 1)).isClose(Vec3.init(2, 1, 1), .{ .max_dist_sq = 1.0e-10 }));

    // Non-90 degree rotations, this would cause shearing so is not allowed (we can't express that by passing a diagonal scale vector)
    var rt4 = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), box_ref.get().?);
    rt4.asShape().setEmbedded();
    defer rt4.asShapeMut().deinit();
    try expect(!rt4.asShape().isValidScale(Vec3.zero()));
    try expect(rt4.asShape().isValidScale(Vec3.init(1, 1, 1)));
    try expect(rt4.asShape().isValidScale(Vec3.init(2, 2, 2)));
    try expect(!rt4.asShape().isValidScale(Vec3.init(2, 1, 1)));
    try expect(!rt4.asShape().isValidScale(Vec3.init(1, 2, 1)));
    try expect(rt4.asShape().isValidScale(Vec3.init(1, 1, 2))); // We're rotation around Z, so non-uniform in the Z direction is ok

    // MakeScaleValid: a scale that can be rotated, the uniform scale with the signs of the scale, the uniform scale
    // with the sign of X
    try expect(rt4.asShape().makeScaleValid(Vec3.init(1, 1, 2)).isClose(Vec3.init(1, 1, 2), .{ .max_dist_sq = 1.0e-10 }));
    try expect(rt4.asShape().makeScaleValid(Vec3.init(1, 2, 3)).eql(Vec3.replicate(2)));
    try expect(rt4.asShape().makeScaleValid(Vec3.init(-1, -2, -3)).eql(Vec3.replicate(-2)));
    try expect(rt4.asShape().makeScaleValid(Vec3.init(-1, 2, 3)).eql(Vec3.replicate(-2)));
    try expect(rt4.asShape().makeScaleValid(Vec3.init(1, -2, 3)).eql(Vec3.replicate(2)));
    try expect(rt4.asShape().makeScaleValid(Vec3.init(0, 1, 1)).eql(rt4.asShape().makeScaleValid(Vec3.init(ScaleHelpers.min_scale, 1, 1))));
}

test "RotatedTranslatedShape: ray casts, collide point, shape filters and the collector context" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), box.asShape());
    rt.asShape().setEmbedded();
    defer rt.asShapeMut().deinit();
    const shape = rt.asShape();
    const creator = SubShapeIDCreator.pushID(.{}, 3, 2);

    // Single hit: the rotated box is 2 wide along X
    var hit: RayCastResult = .{};
    try expect(shape.castRay(.init(Vec3.init(-4, 0, 0), Vec3.init(8, 0, 0)), creator, &hit));
    try testing.expectApproxEqAbs(@as(f32, 0.25), hit.fraction, 1.0e-6);
    try expect(hit.sub_shape_id2.eql(creator.getID()));
    try expect(!shape.castRay(.init(Vec3.init(-4, 1.5, 0), Vec3.init(8, 0, 0)), creator, &hit));

    // Collector: both faces, the filter is called for the decorated shape and the box
    var settings: RayCastSettings = .{};
    settings.setBackFaceMode(.collide_with_back_faces);
    var hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(8), .{});
    hits.base.setContext(&context);
    var calls: u32 = 0;
    const accept_all: RejectShapeFilter = .{ .rejected = null, .calls = &calls };
    shape.castRayCollector(.init(Vec3.init(-4, 0, 0), Vec3.init(8, 0, 0)), &settings, creator, &hits.base, &accept_all.base);
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectApproxEqAbs(@as(f32, 0.75), hits.hits.items[1].fraction, 1.0e-6);
    try expect(hits.hits.items[1].body_id.eql(.init(8)));
    try testing.expectEqual(@as(u32, 2), calls);
    for ([_]*const Shape{ shape, box.asShape() }) |rejected| {
        hits.reset();
        const filter: RejectShapeFilter = .{ .rejected = rejected, .calls = &calls };
        shape.castRayCollector(.init(Vec3.init(-4, 0, 0), Vec3.init(8, 0, 0)), &settings, creator, &hits.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    }

    // Collide point: inside the rotated box, outside it
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.collidePoint(Vec3.init(1.9, 0.9, 0), creator, &points.base, &.{});
    shape.collidePoint(Vec3.init(0.9, 1.9, 0), creator, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    const reject: RejectShapeFilter = .{ .rejected = shape, .calls = &calls };
    shape.collidePoint(Vec3.init(1.9, 0.9, 0), creator, &points.base, &reject.base);
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
}

test "RotatedTranslatedShape: collide and cast through CollisionDispatch" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    // With the identity rotation the inner transform is the transform of the decorated shape (bit exact)
    var rt_box = RotatedTranslatedShape.init(allocator, Vec3.init(4, 5, 6), Quat.identity(), box.asShape());
    rt_box.asShape().setEmbedded();
    defer rt_box.asShapeMut().deinit();
    var rt_sphere = RotatedTranslatedShape.init(allocator, Vec3.init(-1, 0, 0), Quat.identity(), sphere.asShape());
    rt_sphere.asShape().setEmbedded();
    defer rt_sphere.asShapeMut().deinit();

    // The registered functions (the decorators that register later win)
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.rotated_translated, .sphere) == &RotatedTranslatedShape.collideRotatedTranslatedVsShape);
    try expect(r.getCollideShape(.box, .rotated_translated) == &RotatedTranslatedShape.collideShapeVsRotatedTranslated);
    try expect(r.getCollideShape(.rotated_translated, .rotated_translated) == &RotatedTranslatedShape.collideRotatedTranslatedVsRotatedTranslated);
    try expect(r.getCastShape(.rotated_translated, .rotated_translated) == &RotatedTranslatedShape.castRotatedTranslatedVsRotatedTranslated);
    try expect(r.getCastShape(.rotated_translated, .user2) == &RotatedTranslatedShape.castRotatedTranslatedVsShape);
    try expect(r.getCastShape(.capsule, .rotated_translated) == &RotatedTranslatedShape.castShapeVsRotatedTranslated);
    try expect(r.getCollideShape(.rotated_translated, .scaled) != &RotatedTranslatedShape.collideRotatedTranslatedVsShape); // ScaledShape registers later
    try expect(r.getCollideShape(.rotated_translated, .offset_center_of_mass) != &RotatedTranslatedShape.collideRotatedTranslatedVsShape); // OffsetCenterOfMassShape registers later
    try expect(r.getCollideShape(.static_compound, .rotated_translated) == &RotatedTranslatedShape.collideShapeVsRotatedTranslated);
    try expect(ShapeFunctions.get(.rotated_translated).color.eql(Color.blue));
    try expect(ShapeFunctions.get(.rotated_translated).construct != null);

    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    const t1 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.2), Vec3.init(0.5, 1, 0));
    const t2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.4), Vec3.init(2, 0.5, 1));
    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 2);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);
    const Pair = struct { s1: *const Shape, s2: *const Shape, e1: *const Shape, e2: *const Shape };
    const pairs = [_]Pair{
        .{ .s1 = rt_sphere.asShape(), .s2 = box.asShape(), .e1 = sphere.asShape(), .e2 = box.asShape() },
        .{ .s1 = sphere.asShape(), .s2 = rt_box.asShape(), .e1 = sphere.asShape(), .e2 = box.asShape() },
        .{ .s1 = rt_sphere.asShape(), .s2 = rt_box.asShape(), .e1 = sphere.asShape(), .e2 = box.asShape() },
    };
    for (pairs) |p| {
        var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, Vec3.one(), Vec3.init(1, 2, 1), t1, t2, creator1, creator2, &settings, &hits.base, &.{});
        try hits.checkError();
        var expected = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.collideShapeVsShape(p.e1, p.e2, Vec3.one(), Vec3.init(1, 2, 1), t1, t2, creator1, creator2, &settings, &expected.base, &.{});
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
        const filter: RejectShapeFilter = .{ .rejected = box.asShape(), .calls = &calls };
        var filtered = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer filtered.deinit();
        CollisionDispatch.collideShapeVsShape(p.s1, p.s2, Vec3.one(), Vec3.init(1, 2, 1), t1, t2, creator1, creator2, &settings, &filtered.base, &filter.base);
        try testing.expectEqual(@as(usize, 0), filtered.hits.items.len);
    }

    // A rotation of 90 degrees around Y: the result is close to the rotated box
    var rt_rotated = RotatedTranslatedShape.init(allocator, Vec3.zero(), Quat.rotation(Vec3.axisY(), 0.5 * math.pi), box.asShape());
    rt_rotated.asShape().setEmbedded();
    defer rt_rotated.asShapeMut().deinit();
    {
        var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.collideShapeVsShape(sphere.asShape(), rt_rotated.asShape(), Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 0, 1.5)), Mat44.identity(), .{}, .{}, &settings, &hits.base, &.{});
        try hits.checkError();
        try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
        try testing.expectApproxEqAbs(@as(f32, 0.5), hits.hits.items[0].penetration_depth, 1.0e-4); // The box is 1 thick along Z
    }

    // Casts with the rotated translated shape on either side and on both sides
    const cast_settings: ShapeCastSettings = .{};
    const start = Mat44.translation(Vec3.init(-6, 0.25, 0));
    const CastPair = struct { cast: ShapeCast, s: *const Shape, expected_cast: ShapeCast, e: *const Shape };
    const cast_pairs = [_]CastPair{
        .{ .cast = .init(rt_sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = box.asShape(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = box.asShape() },
        .{ .cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = rt_box.asShape(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = box.asShape() },
        .{ .cast = .init(rt_sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .s = rt_box.asShape(), .expected_cast = .init(sphere.asShape(), Vec3.one(), start, Vec3.init(10, 0, 0)), .e = box.asShape() },
    };
    for (cast_pairs) |p| {
        var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer hits.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.cast, &cast_settings, p.s, Vec3.one(), &.{}, t2, creator1, creator2, &hits.base);
        try hits.checkError();
        var expected = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer expected.deinit();
        CollisionDispatch.castShapeVsShapeWorldSpace(&p.expected_cast, &cast_settings, p.e, Vec3.one(), &.{}, t2, creator1, creator2, &expected.base);
        try expected.checkError();
        try testing.expectEqual(@as(usize, 1), expected.hits.items.len);
        try testing.expectEqual(expected.hits.items.len, hits.hits.items.len);
        try testing.expectApproxEqAbs(expected.hits.items[0].fraction, hits.hits.items[0].fraction, 1.0e-6);
        try expect(hits.hits.items[0].base.contact_point_on2.isClose(expected.hits.items[0].base.contact_point_on2, .{ .max_dist_sq = 1.0e-10 }));
    }
}

test "RotatedTranslatedShape: submerged volume, soft body vertices, collect and transform shapes" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const rotation = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3);
    var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), rotation, box.asShape());
    rt.asShape().setEmbedded();
    defer rt.asShapeMut().deinit();
    const shape = rt.asShape();
    const r = Mat44.rotationQuat(rotation);
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.7), Vec3.init(1, 2, 3));

    // Submerged volume: the child transform
    const surface = Plane.fromPointAndNormal(Vec3.init(0, 2.5, 0), Vec3.axisY());
    const v = shape.getSubmergedVolume(transform, Vec3.replicate(2), surface);
    const expected_v = box.asShape().getSubmergedVolume(transform.mul(r), Vec3.replicate(2), surface);
    try testing.expectEqual(expected_v.total_volume, v.total_volume);
    try testing.expectEqual(expected_v.submerged_volume, v.submerged_volume);
    try expect(v.center_of_buoyancy.eql(expected_v.center_of_buoyancy));

    // Soft body vertices: the child transform, the scale is passed on unchanged
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
            shape.collideSoftBodyVertices(transform, Vec3.init(1, 2, 3), &vertices, 3, 5)
        else
            box.asShape().collideSoftBodyVertices(transform.mul(r), Vec3.init(1, 2, 3), &vertices, 3, 5);
    }
    for (0..3) |i| {
        try testing.expectEqual(penetrations[1][i], penetrations[0][i]);
        try testing.expectEqual(indices[1][i], indices[0][i]);
        try expect(planes[0][i].normal_and_constant.eql(planes[1][i].normal_and_constant));
    }

    // Collect transformed shapes: the leaf with the combined rotation and the scale in its space
    var collected = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collected.deinit();
    const creator = SubShapeIDCreator.pushID(.{}, 3, 2);
    shape.collectTransformedShapes(AABox.biggest(), Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.1), Vec3.replicate(3), creator, &collected.base, &.{});
    try collected.checkError();
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);
    const ts = &collected.hits.items[0];
    try expect(ts.shape.get() == box.asShape() and ts.getShapeScale().eql(Vec3.replicate(3)));
    try expect(ts.shape_rotation.eql(Quat.rotation(Vec3.axisZ(), 0.1).mul(rotation)));
    try expect(ts.sub_shape_id_creator.getID().eql(creator.getID()));
    var calls: u32 = 0;
    const reject: RejectShapeFilter = .{ .rejected = shape, .calls = &calls };
    shape.collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), creator, &collected.base, &reject.base);
    try testing.expectEqual(@as(usize, 1), collected.hits.items.len);

    // Transform shape: the leaf gets the combined transform
    var transformed = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer transformed.deinit();
    shape.transformShape(transform, &transformed.base);
    try transformed.checkError();
    try testing.expectEqual(@as(usize, 1), transformed.hits.items.len);
    try expect(transformed.hits.items[0].shape.get() == box.asShape());
    try expect(transformed.hits.items[0].shape_rotation.isClose(transform.mul(r).getQuaternion(), .{}));

    // getTrianglesStart / Next assert (Jolt: "Cannot call on non-leaf shapes"), without asserts they return nothing
    if (!Core.enable_asserts) {
        var tri_context: Shape.GetTrianglesContext = .{};
        shape.getTrianglesStart(&tri_context, AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one());
        var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
        try testing.expectEqual(@as(u32, 0), shape.getTrianglesNext(&tri_context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    }
}

test "RotatedTranslatedShape: binary state, sub shape state, restoreFromBinaryState and saveWithChildren" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box_ref = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{})).asShape());
    defer box_ref.deinit();
    const rotation = Quat.rotation(Vec3.axisX(), 0.25);
    const rt = try RotatedTranslatedShape.create(allocator, Vec3.init(1, 2, 3), rotation, box_ref.get().?);
    rt.asShapeMut().setUserData(99);
    var rt_ref = RefConst(Shape).init(rt.asShape());
    defer rt_ref.deinit();

    // Sub type, user data, center of mass, rotation
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(rt.asShape(), &buffer);
    try testing.expectEqual(@as(usize, 1 + 8 + 12 + 16), bytes.len);
    try testing.expectEqual(@intFromEnum(ShapeSubType.rotated_translated), bytes[0]);

    // Restore: is_rotation_identity is recalculated, the inner shape comes from the sub shape state
    var result = try restoreFromBuffer(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?.castMut(RotatedTranslatedShape);
    try expect(restored.getCenterOfMass().eql(Vec3.init(1, 2, 3)) and restored.getRotation().eql(rotation) and !restored.is_rotation_identity);
    try testing.expectEqual(@as(u64, 99), restored.asShape().getUserData());
    try expect(restored.base.getInnerShape() == null);
    restored.asShapeMut().restoreSubShapeState(&.{box_ref}); // Takes its own reference
    try expect(restored.base.getInnerShape() == box_ref.get());
    try expect(restored.getPosition().eql(Vec3.init(1, 2, 3)));
    var buffer2: [64]u8 = undefined;
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(restored.asShape(), &buffer2));

    // An identity rotation is detected again after a restore
    var identity = RotatedTranslatedShape.init(allocator, Vec3.zero(), Quat.identity(), box_ref.get().?);
    identity.asShape().setEmbedded();
    defer identity.asShapeMut().deinit();
    var identity_result = try restoreFromBuffer(allocator, saveToBuffer(identity.asShape(), &buffer));
    defer identity_result.deinit();
    try expect(identity_result.getPtr().?.cast(RotatedTranslatedShape).is_rotation_identity);

    // saveWithChildren / restoreWithChildren
    var graph: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&graph);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try rt.asShape().saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
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
    const graph_rt = graph_result.getPtr().?.cast(RotatedTranslatedShape);
    try expect(graph_rt.getRotation().eql(rotation) and graph_rt.asShape().getLocalBounds().eql(rt.asShape().getLocalBounds()));
}
