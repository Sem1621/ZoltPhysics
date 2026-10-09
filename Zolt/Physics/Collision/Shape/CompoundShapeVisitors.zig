//! Port of: Jolt/Physics/Collision/Shape/CompoundShapeVisitors.h
//! Status: complete
//!
//! The visitors that the compound shapes (StaticCompoundShape, MutableCompoundShape) use for collision detection. In
//! Jolt they are protected nested structs of CompoundShape (`CompoundShape::CastRayVisitor`, ...); here they are the
//! public structs of this file, also reachable as `CompoundShape.CastRayVisitor`, ... (CompoundShape.zig re-exports
//! them).
//!
//! Visitor protocol. Jolt's template walkers (`StaticCompoundShape::WalkTree`, `MutableCompoundShape::WalkSubShapes`)
//! take the visitor as a template parameter; in Zolt they take `visitor: anytype` (static dispatch, porting guide
//! section 6) and call:
//! - `visitor.shouldAbort() bool` (ShouldAbort): true when no better hit can be found;
//! - `visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z)` (TestBounds): tests 4 bounding boxes (structure
//!   of arrays, relative to the center of mass of the compound, unscaled) and returns a `Vec4` (distance along the ray
//!   / cast, the visitors of the casts) or a `UVec4` (true for the boxes that overlap, the other visitors);
//! - `visitor.visitShape(sub_shape, sub_shape_index)` (VisitShape): tests a single sub shape.
//! The walkers of Jolt derive from these visitors to add the functions they need (`struct Visitor : public
//! CastRayVisitor { ... }`); in Zolt such a visitor embeds the visitor of this file as its field `base` and forwards
//! the protocol functions (`self.base.visitShape(...)`), so the derived visitor reads the state as `self.base.hit`,
//! `self.base.collector`, ...
//!
//! C++ reference members become pointers (`const RayCast &mRay` -> `ray: *const RayCast`, `RayCastResult &mHit` ->
//! `hit: *RayCastResult`, `CastRayCollector &mCollector` -> `collector: *CastRayCollector`), value members stay values.
//! The pointers must outlive the visitor, exactly like the references in Jolt. Constructors are `init`, with Jolt's
//! parameter order. `GetIntersectingSubShapesVisitor<BoxType>` is the comptime function
//! `GetIntersectingSubShapesVisitor(BoxType)` (BoxType is AABox or OrientedBox); its output buffer and capacity
//! (`uint *outSubShapeIndices, int inMaxSubShapeIndices`) are a slice.
//!
//! The asserts `JPH_ASSERT(inSubShape.IsValidScale(mScale))` can be violated by valid input (a query with a scale that
//! the compound cannot represent, Jolt's release build then continues with a sheared result), so they are guarded with
//! `Core.enable_asserts` (porting guide section 7).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const AABox4 = @import("../../../Geometry/AABox4.zig");
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const CompoundShape = @import("CompoundShape.zig").CompoundShape;
const SubShape = CompoundShape.SubShape;
const SubShapeIDCreator = @import("SubShapeID.zig").SubShapeIDCreator;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("../CollisionDispatch.zig");

/// CompoundShape::CastRayVisitor
pub const CastRayVisitor = struct {
    inv_direction: RayInvDirection,
    ray: *const RayCast,
    hit: *RayCastResult,
    sub_shape_id_creator: SubShapeIDCreator,
    sub_shape_bits: u32,
    return_value: bool = false,

    /// Constructor
    pub fn init(ray: *const RayCast, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) CastRayVisitor {
        var self: CastRayVisitor = .{
            .inv_direction = undefined,
            .ray = ray,
            .hit = hit,
            .sub_shape_id_creator = sub_shape_id_creator,
            .sub_shape_bits = shape.getSubShapeIDBits(),
        };

        // Determine ray properties of cast
        self.inv_direction.set(ray.direction);
        return self;
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CastRayVisitor) bool {
        return self.hit.fraction <= 0.0;
    }

    /// Test ray against 4 bounding boxes and returns the distance where the ray enters the bounding box
    pub fn testBounds(self: *const CastRayVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) Vec4 {
        return RayAABox.rayAABox4(self.ray.origin, self.inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
    }

    /// Test the ray against a single subshape
    pub fn visitShape(self: *CastRayVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Transform the ray
        const transform = Mat44.inverseRotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());
        const ray = self.ray.transformed(transform);
        if (sub_shape.shape.get().?.castRay(ray, shape2_sub_shape_id, self.hit))
            self.return_value = true;
    }
};

/// CompoundShape::CastRayVisitorCollector
pub const CastRayVisitorCollector = struct {
    inv_direction: RayInvDirection,
    ray: *const RayCast,
    collector: *CastRayCollector,
    sub_shape_id_creator: SubShapeIDCreator,
    sub_shape_bits: u32,
    ray_cast_settings: RayCastSettings,
    shape_filter: *const ShapeFilter,

    /// Constructor
    pub fn init(ray: *const RayCast, ray_cast_settings: *const RayCastSettings, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) CastRayVisitorCollector {
        var self: CastRayVisitorCollector = .{
            .inv_direction = undefined,
            .ray = ray,
            .collector = collector,
            .sub_shape_id_creator = sub_shape_id_creator,
            .sub_shape_bits = shape.getSubShapeIDBits(),
            .ray_cast_settings = ray_cast_settings.*,
            .shape_filter = shape_filter,
        };

        // Determine ray properties of cast
        self.inv_direction.set(ray.direction);
        return self;
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CastRayVisitorCollector) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Test ray against 4 bounding boxes and returns the distance where the ray enters the bounding box
    pub fn testBounds(self: *const CastRayVisitorCollector, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) Vec4 {
        return RayAABox.rayAABox4(self.ray.origin, self.inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
    }

    /// Test the ray against a single subshape
    pub fn visitShape(self: *CastRayVisitorCollector, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Transform the ray
        const transform = Mat44.inverseRotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());
        const ray = self.ray.transformed(transform);
        sub_shape.shape.get().?.castRayCollector(ray, &self.ray_cast_settings, shape2_sub_shape_id, self.collector, self.shape_filter);
    }
};

/// CompoundShape::CollidePointVisitor
pub const CollidePointVisitor = struct {
    point: Vec3,
    sub_shape_id_creator: SubShapeIDCreator,
    collector: *CollidePointCollector,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    /// Constructor
    pub fn init(point: Vec3, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) CollidePointVisitor {
        return .{
            .point = point,
            .sub_shape_id_creator = sub_shape_id_creator,
            .collector = collector,
            .sub_shape_bits = shape.getSubShapeIDBits(),
            .shape_filter = shape_filter,
        };
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CollidePointVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Test if point overlaps with 4 boxes, returns true for the ones that do
    pub fn testBounds(self: *const CollidePointVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
        return AABox4.aabox4VsPoint(self.point, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
    }

    /// Test the point against a single subshape
    pub fn visitShape(self: *CollidePointVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Transform the point
        const transform = Mat44.inverseRotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());
        sub_shape.shape.get().?.collidePoint(transform.mulVec3(self.point), shape2_sub_shape_id, self.collector, self.shape_filter);
    }
};

/// CompoundShape::CastShapeVisitor
pub const CastShapeVisitor = struct {
    inv_direction: RayInvDirection,
    box_center: Vec3,
    box_extent: Vec3,
    scale: Vec3,
    shape_cast: *const ShapeCast,
    shape_cast_settings: *const ShapeCastSettings,
    shape_filter: *const ShapeFilter,
    collector: *CastShapeCollector,
    center_of_mass_transform2: Mat44,
    sub_shape_id_creator1: SubShapeIDCreator,
    sub_shape_id_creator2: SubShapeIDCreator,
    sub_shape_bits: u32,

    /// Constructor
    pub fn init(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const CompoundShape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) CastShapeVisitor {
        var self: CastShapeVisitor = .{
            .inv_direction = undefined,
            .box_center = shape_cast.shape_world_bounds.getCenter(),
            .box_extent = shape_cast.shape_world_bounds.getExtent().add(Vec3.replicate(shape_cast_settings.extra_convex_radius)),
            .scale = scale,
            .shape_cast = shape_cast,
            .shape_cast_settings = shape_cast_settings,
            .shape_filter = shape_filter,
            .collector = collector,
            .center_of_mass_transform2 = center_of_mass_transform2,
            .sub_shape_id_creator1 = sub_shape_id_creator1,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
            .sub_shape_bits = shape.getSubShapeIDBits(),
        };

        // Determine ray properties of cast
        self.inv_direction.set(shape_cast.direction);
        return self;
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CastShapeVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Tests the shape cast against 4 bounding boxes, returns the distance along the shape cast where the shape first enters the bounding box
    pub fn testBounds(self: *const CastShapeVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) Vec4 {
        // Scale the bounding boxes
        var bounds = AABox4.aabox4Scale(self.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

        // Enlarge them by the casted shape's box extents
        AABox4.aabox4EnlargeWithExtent(self.box_extent, &bounds.min_x, &bounds.min_y, &bounds.min_z, &bounds.max_x, &bounds.max_y, &bounds.max_z);

        // Test ray against the bounding boxes
        return RayAABox.rayAABox4(self.box_center, self.inv_direction, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);
    }

    /// Test the cast shape against a single subshape
    pub fn visitShape(self: *CastShapeVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        if (Core.enable_asserts) std.debug.assert(sub_shape.isValidScale(self.scale)); // An invalid scale can violate this, Jolt's release build continues

        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator2.pushID(sub_shape_index, self.sub_shape_bits);

        // Calculate the local transform for this sub shape
        const local_transform = Mat44.rotationTranslation(sub_shape.getRotation(), self.scale.mul(sub_shape.getPositionCOM()));

        // Transform the center of mass of 2
        const center_of_mass_transform2 = self.center_of_mass_transform2.mul(local_transform);

        // Transform the shape cast
        const shape_cast = self.shape_cast.postTransformed(local_transform.inversedRotationTranslation());

        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, self.shape_cast_settings, sub_shape.shape.get().?, sub_shape.transformScale(self.scale), self.shape_filter, center_of_mass_transform2, self.sub_shape_id_creator1, shape2_sub_shape_id, self.collector);
    }
};

/// CompoundShape::CollectTransformedShapesVisitor
pub const CollectTransformedShapesVisitor = struct {
    box: AABox,
    local_box: OrientedBox,
    position_com: Vec3,
    rotation: Quat,
    scale: Vec3,
    sub_shape_id_creator: SubShapeIDCreator,
    collector: *TransformedShapeCollector,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    /// Constructor
    pub fn init(box: AABox, shape: *const CompoundShape, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) CollectTransformedShapesVisitor {
        return .{
            .box = box,
            .local_box = .fromAABox(Mat44.inverseRotationTranslation(rotation, position_com), box),
            .position_com = position_com,
            .rotation = rotation,
            .scale = scale,
            .sub_shape_id_creator = sub_shape_id_creator,
            .collector = collector,
            .sub_shape_bits = shape.getSubShapeIDBits(),
            .shape_filter = shape_filter,
        };
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CollectTransformedShapesVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Tests 4 bounding boxes against the query box, returns true for the ones that collide
    pub fn testBounds(self: *const CollectTransformedShapesVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
        // Scale the bounding boxes of this node
        const bounds = AABox4.aabox4Scale(self.scale, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

        // Test which nodes collide
        return AABox4.aabox4VsOrientedBox(self.local_box, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z, .{});
    }

    /// Collect the transformed sub shapes for a single subshape
    pub fn visitShape(self: *CollectTransformedShapesVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        if (Core.enable_asserts) std.debug.assert(sub_shape.isValidScale(self.scale)); // An invalid scale can violate this, Jolt's release build continues

        // Create ID for sub shape
        const sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Calculate world transform for sub shape
        const position = self.position_com.add(self.rotation.mulVec3(self.scale.mul(sub_shape.getPositionCOM())));
        const rotation = self.rotation.mul(sub_shape.getRotation());

        // Recurse to sub shape
        sub_shape.shape.get().?.collectTransformedShapes(self.box, position, rotation, sub_shape.transformScale(self.scale), sub_shape_id, self.collector, self.shape_filter);
    }
};

/// CompoundShape::CollideCompoundVsShapeVisitor
pub const CollideCompoundVsShapeVisitor = struct {
    collide_shape_settings: *const CollideShapeSettings,
    collector: *CollideShapeCollector,
    shape2: *const Shape,
    scale1: Vec3,
    scale2: Vec3,
    transform1: Mat44,
    transform2: Mat44,
    bounds_of2_in_space_of1: AABox,
    sub_shape_id_creator1: SubShapeIDCreator,
    sub_shape_id_creator2: SubShapeIDCreator,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    /// Constructor
    pub fn init(shape1: *const CompoundShape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) CollideCompoundVsShapeVisitor {
        var self: CollideCompoundVsShapeVisitor = .{
            .collide_shape_settings = collide_shape_settings,
            .collector = collector,
            .shape2 = shape2,
            .scale1 = scale1,
            .scale2 = scale2,
            .transform1 = center_of_mass_transform1,
            .transform2 = center_of_mass_transform2,
            .bounds_of2_in_space_of1 = undefined,
            .sub_shape_id_creator1 = sub_shape_id_creator1,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
            .sub_shape_bits = shape1.getSubShapeIDBits(),
            .shape_filter = shape_filter,
        };

        // Get transform from shape 2 to shape 1
        const transform2_to_1 = center_of_mass_transform1.inversedRotationTranslation().mul(center_of_mass_transform2);

        // Convert bounding box of 2 into space of 1
        self.bounds_of2_in_space_of1 = shape2.getLocalBounds().scaled(scale2).transformed(transform2_to_1);
        self.bounds_of2_in_space_of1.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));
        return self;
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CollideCompoundVsShapeVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Tests the bounds of shape 2 vs 4 bounding boxes, returns true for the ones that intersect
    pub fn testBounds(self: *const CollideCompoundVsShapeVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
        // Scale the bounding boxes
        const bounds = AABox4.aabox4Scale(self.scale1, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

        // Test which boxes collide
        return AABox4.aabox4VsBox(self.bounds_of2_in_space_of1, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);
    }

    /// Test the shape against a single subshape
    pub fn visitShape(self: *CollideCompoundVsShapeVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Get world transform of 1
        const transform1 = self.transform1.mul(sub_shape.getLocalTransformNoScale(self.scale1));

        // Create ID for sub shape
        const shape1_sub_shape_id = self.sub_shape_id_creator1.pushID(sub_shape_index, self.sub_shape_bits);

        CollisionDispatch.collideShapeVsShape(sub_shape.shape.get().?, self.shape2, sub_shape.transformScale(self.scale1), self.scale2, transform1, self.transform2, shape1_sub_shape_id, self.sub_shape_id_creator2, self.collide_shape_settings, self.collector, self.shape_filter);
    }
};

/// CompoundShape::CollideShapeVsCompoundVisitor
pub const CollideShapeVsCompoundVisitor = struct {
    collide_shape_settings: *const CollideShapeSettings,
    collector: *CollideShapeCollector,
    shape1: *const Shape,
    scale1: Vec3,
    scale2: Vec3,
    transform1: Mat44,
    transform2: Mat44,
    bounds_of1_in_space_of2: AABox,
    sub_shape_id_creator1: SubShapeIDCreator,
    sub_shape_id_creator2: SubShapeIDCreator,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    /// Constructor
    pub fn init(shape1: *const Shape, shape2: *const CompoundShape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) CollideShapeVsCompoundVisitor {
        var self: CollideShapeVsCompoundVisitor = .{
            .collide_shape_settings = collide_shape_settings,
            .collector = collector,
            .shape1 = shape1,
            .scale1 = scale1,
            .scale2 = scale2,
            .transform1 = center_of_mass_transform1,
            .transform2 = center_of_mass_transform2,
            .bounds_of1_in_space_of2 = undefined,
            .sub_shape_id_creator1 = sub_shape_id_creator1,
            .sub_shape_id_creator2 = sub_shape_id_creator2,
            .sub_shape_bits = shape2.getSubShapeIDBits(),
            .shape_filter = shape_filter,
        };

        // Get transform from shape 1 to shape 2
        const transform1_to_2 = center_of_mass_transform2.inversedRotationTranslation().mul(center_of_mass_transform1);

        // Convert bounding box of 1 into space of 2
        self.bounds_of1_in_space_of2 = shape1.getLocalBounds().scaled(scale1).transformed(transform1_to_2);
        self.bounds_of1_in_space_of2.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));
        return self;
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CollideShapeVsCompoundVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    /// Tests the bounds of shape 1 vs 4 bounding boxes, returns true for the ones that intersect
    pub fn testBounds(self: *const CollideShapeVsCompoundVisitor, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
        // Scale the bounding boxes
        const bounds = AABox4.aabox4Scale(self.scale2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

        // Test which bounding boxes collide
        return AABox4.aabox4VsBox(self.bounds_of1_in_space_of2, bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);
    }

    /// Test the shape against a single subshape
    pub fn visitShape(self: *CollideShapeVsCompoundVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator2.pushID(sub_shape_index, self.sub_shape_bits);

        // Get world transform of 2
        const transform2 = self.transform2.mul(sub_shape.getLocalTransformNoScale(self.scale2));

        CollisionDispatch.collideShapeVsShape(self.shape1, sub_shape.shape.get().?, self.scale1, sub_shape.transformScale(self.scale2), self.transform1, transform2, self.sub_shape_id_creator1, shape2_sub_shape_id, self.collide_shape_settings, self.collector, self.shape_filter);
    }
};

/// CompoundShape::GetIntersectingSubShapesVisitor<BoxType> (BoxType is AABox or OrientedBox)
pub fn GetIntersectingSubShapesVisitor(comptime BoxType: type) type {
    comptime std.debug.assert(BoxType == AABox or BoxType == OrientedBox);

    return struct {
        const Self = @This();

        box: BoxType,
        /// The output buffer (Jolt's outSubShapeIndices, its length is inMaxSubShapeIndices)
        sub_shape_indices: []u32,
        num_results: u32 = 0,

        /// Constructor
        pub fn init(box: BoxType, out_sub_shape_indices: []u32) Self {
            return .{ .box = box, .sub_shape_indices = out_sub_shape_indices };
        }

        /// Returns true when collision detection should abort because the buffer is full
        pub fn shouldAbort(self: *const Self) bool {
            return self.num_results >= self.sub_shape_indices.len;
        }

        /// Tests the box vs 4 bounding boxes, returns true for the ones that intersect
        pub fn testBounds(self: *const Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
            // Test which bounding boxes collide
            return if (BoxType == AABox)
                AABox4.aabox4VsBox(self.box, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z)
            else
                AABox4.aabox4VsOrientedBox(self.box, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z, .{});
        }

        /// Records a hit
        pub fn visitShape(self: *Self, sub_shape: *const SubShape, sub_shape_index: u32) void {
            _ = sub_shape;
            std.debug.assert(self.num_results < self.sub_shape_indices.len); // Writing beyond the buffer is undefined behavior in Jolt too
            self.sub_shape_indices[self.num_results] = sub_shape_index;
            self.num_results += 1;
        }

        /// Get the number of indices that were found
        pub fn getNumResults(self: *const Self) u32 {
            return self.num_results;
        }
    };
}
