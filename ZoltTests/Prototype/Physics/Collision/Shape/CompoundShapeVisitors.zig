//! Port of: Jolt/Physics/Collision/Shape/CompoundShapeVisitors.h (prototype, reduced)
//! Status: partial
//! Missing: TestBounds of every visitor (needs the StaticCompoundShape / MutableCompoundShape bounding volume
//! hierarchies), GetIntersectingSubShapesVisitor
//!
//! Template visitors (`ioVisitor.VisitShape(...)` called from a template WalkTree) are plain structs used through
//! `anytype` (static dispatch, porting guide section 6). C++ reference members become pointers.

const std = @import("std");
const zolt = @import("zolt");
const AABox = zolt.AABox;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;

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
const RayCast = @import("../RayCast.zig").RayCast;
const RayCastSettings = @import("../RayCast.zig").RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("../CollisionDispatch.zig");

pub const CastRayVisitor = struct {
    ray: RayCast,
    hit: *RayCastResult,
    sub_shape_id_creator: SubShapeIDCreator,
    sub_shape_bits: u32,
    return_value: bool = false,

    pub fn init(ray: RayCast, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) CastRayVisitor {
        return .{ .ray = ray, .hit = hit, .sub_shape_id_creator = sub_shape_id_creator, .sub_shape_bits = shape.getSubShapeIDBits() };
    }

    /// Returns true when collision detection should abort because it's not possible to find a better hit
    pub fn shouldAbort(self: *const CastRayVisitor) bool {
        return self.hit.fraction <= 0.0;
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

pub const CastRayVisitorCollector = struct {
    ray: RayCast,
    collector: *CastRayCollector,
    sub_shape_id_creator: SubShapeIDCreator,
    sub_shape_bits: u32,
    ray_cast_settings: RayCastSettings,
    shape_filter: *const ShapeFilter,

    pub fn init(ray: RayCast, ray_cast_settings: *const RayCastSettings, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) CastRayVisitorCollector {
        return .{ .ray = ray, .collector = collector, .sub_shape_id_creator = sub_shape_id_creator, .sub_shape_bits = shape.getSubShapeIDBits(), .ray_cast_settings = ray_cast_settings.*, .shape_filter = shape_filter };
    }

    pub fn shouldAbort(self: *const CastRayVisitorCollector) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CastRayVisitorCollector, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Transform the ray
        const transform = Mat44.inverseRotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());
        const ray = self.ray.transformed(transform);
        sub_shape.shape.get().?.castRayCollector(ray, &self.ray_cast_settings, shape2_sub_shape_id, self.collector, self.shape_filter);
    }
};

pub const CollidePointVisitor = struct {
    point: Vec3,
    sub_shape_id_creator: SubShapeIDCreator,
    collector: *CollidePointCollector,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    pub fn init(point: Vec3, shape: *const CompoundShape, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) CollidePointVisitor {
        return .{ .point = point, .sub_shape_id_creator = sub_shape_id_creator, .collector = collector, .sub_shape_bits = shape.getSubShapeIDBits(), .shape_filter = shape_filter };
    }

    pub fn shouldAbort(self: *const CollidePointVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CollidePointVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Transform the point
        const transform = Mat44.inverseRotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());
        sub_shape.shape.get().?.collidePoint(transform.mulVec3(self.point), shape2_sub_shape_id, self.collector, self.shape_filter);
    }
};

pub const CastShapeVisitor = struct {
    scale: Vec3,
    shape_cast: *const ShapeCast,
    shape_cast_settings: *const ShapeCastSettings,
    shape_filter: *const ShapeFilter,
    collector: *CastShapeCollector,
    center_of_mass_transform2: Mat44,
    sub_shape_id_creator1: SubShapeIDCreator,
    sub_shape_id_creator2: SubShapeIDCreator,
    sub_shape_bits: u32,

    pub fn init(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const CompoundShape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) CastShapeVisitor {
        return .{ .scale = scale, .shape_cast = shape_cast, .shape_cast_settings = shape_cast_settings, .shape_filter = shape_filter, .collector = collector, .center_of_mass_transform2 = center_of_mass_transform2, .sub_shape_id_creator1 = sub_shape_id_creator1, .sub_shape_id_creator2 = sub_shape_id_creator2, .sub_shape_bits = shape.getSubShapeIDBits() };
    }

    pub fn shouldAbort(self: *const CastShapeVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CastShapeVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        std.debug.assert(sub_shape.isValidScale(self.scale));

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

pub const CollectTransformedShapesVisitor = struct {
    box: AABox,
    position_com: Vec3,
    rotation: Quat,
    scale: Vec3,
    sub_shape_id_creator: SubShapeIDCreator,
    collector: *TransformedShapeCollector,
    sub_shape_bits: u32,
    shape_filter: *const ShapeFilter,

    pub fn init(box: AABox, shape: *const CompoundShape, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) CollectTransformedShapesVisitor {
        return .{ .box = box, .position_com = position_com, .rotation = rotation, .scale = scale, .sub_shape_id_creator = sub_shape_id_creator, .collector = collector, .sub_shape_bits = shape.getSubShapeIDBits(), .shape_filter = shape_filter };
    }

    pub fn shouldAbort(self: *const CollectTransformedShapesVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CollectTransformedShapesVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        std.debug.assert(sub_shape.isValidScale(self.scale));

        // Create ID for sub shape
        const sub_shape_id = self.sub_shape_id_creator.pushID(sub_shape_index, self.sub_shape_bits);

        // Calculate world transform for sub shape
        const position = self.position_com.add(self.rotation.mulVec3(self.scale.mul(sub_shape.getPositionCOM())));
        const rotation = self.rotation.mul(sub_shape.getRotation());

        // Recurse to sub shape
        sub_shape.shape.get().?.collectTransformedShapes(self.box, position, rotation, sub_shape.transformScale(self.scale), sub_shape_id, self.collector, self.shape_filter);
    }
};

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

    pub fn init(shape1: *const CompoundShape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) CollideCompoundVsShapeVisitor {
        // Get transform from shape 2 to shape 1
        const transform2_to_1 = center_of_mass_transform1.inversedRotationTranslation().mul(center_of_mass_transform2);

        // Convert bounding box of 2 into space of 1
        var bounds = shape2.getLocalBounds().scaled(scale2).transformed(transform2_to_1);
        bounds.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));

        return .{ .collide_shape_settings = collide_shape_settings, .collector = collector, .shape2 = shape2, .scale1 = scale1, .scale2 = scale2, .transform1 = center_of_mass_transform1, .transform2 = center_of_mass_transform2, .bounds_of2_in_space_of1 = bounds, .sub_shape_id_creator1 = sub_shape_id_creator1, .sub_shape_id_creator2 = sub_shape_id_creator2, .sub_shape_bits = shape1.getSubShapeIDBits(), .shape_filter = shape_filter };
    }

    pub fn shouldAbort(self: *const CollideCompoundVsShapeVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CollideCompoundVsShapeVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Get world transform of 1
        const transform1 = self.transform1.mul(sub_shape.getLocalTransformNoScale(self.scale1));

        // Create ID for sub shape
        const shape1_sub_shape_id = self.sub_shape_id_creator1.pushID(sub_shape_index, self.sub_shape_bits);

        CollisionDispatch.collideShapeVsShape(sub_shape.shape.get().?, self.shape2, sub_shape.transformScale(self.scale1), self.scale2, transform1, self.transform2, shape1_sub_shape_id, self.sub_shape_id_creator2, self.collide_shape_settings, self.collector, self.shape_filter);
    }
};

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

    pub fn init(shape1: *const Shape, shape2: *const CompoundShape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) CollideShapeVsCompoundVisitor {
        // Get transform from shape 1 to shape 2
        const transform1_to_2 = center_of_mass_transform2.inversedRotationTranslation().mul(center_of_mass_transform1);

        // Convert bounding box of 1 into space of 2
        var bounds = shape1.getLocalBounds().scaled(scale1).transformed(transform1_to_2);
        bounds.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));

        return .{ .collide_shape_settings = collide_shape_settings, .collector = collector, .shape1 = shape1, .scale1 = scale1, .scale2 = scale2, .transform1 = center_of_mass_transform1, .transform2 = center_of_mass_transform2, .bounds_of1_in_space_of2 = bounds, .sub_shape_id_creator1 = sub_shape_id_creator1, .sub_shape_id_creator2 = sub_shape_id_creator2, .sub_shape_bits = shape2.getSubShapeIDBits(), .shape_filter = shape_filter };
    }

    pub fn shouldAbort(self: *const CollideShapeVsCompoundVisitor) bool {
        return self.collector.shouldEarlyOut();
    }

    pub fn visitShape(self: *CollideShapeVsCompoundVisitor, sub_shape: *const SubShape, sub_shape_index: u32) void {
        // Create ID for sub shape
        const shape2_sub_shape_id = self.sub_shape_id_creator2.pushID(sub_shape_index, self.sub_shape_bits);

        // Get world transform of 2
        const transform2 = self.transform2.mul(sub_shape.getLocalTransformNoScale(self.scale2));

        CollisionDispatch.collideShapeVsShape(self.shape1, sub_shape.shape.get().?, self.scale1, sub_shape.transformScale(self.scale2), self.transform1, transform2, self.sub_shape_id_creator1, shape2_sub_shape_id, self.collide_shape_settings, self.collector, self.shape_filter);
    }
};
