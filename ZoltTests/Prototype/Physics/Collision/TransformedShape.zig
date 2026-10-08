//! Port of: Jolt/Physics/Collision/TransformedShape.h, Jolt/Physics/Collision/TransformedShape.cpp (prototype, reduced)
//! Status: partial
//! Missing: CastShape (needs RShapeCast), GetWorldSpaceBounds (needs Shape::GetWorldSpaceBounds(DMat44)), GetSupportingFace, GetWorldSpaceSurfaceNormal, GetMaterial, GetSubShapeUserData, GetWorldTransform
//!
//! D12: a value type that owns a reference to its shape (`RefConst<Shape> mShape`), so a query can keep using it after
//! NarrowPhaseQuery released the body lock while another thread replaces the body's shape. Zig has no copy
//! constructor: `init` adds a reference, `clone()` is the copy constructor, `deinit()` the destructor, a plain `=`
//! moves. Collectors that store TransformedShapes clone them (CollisionCollectorImpl.zig).
//!
//! D7: the query entry points write `body_id2` of the shape filter (a `mutable` member in C++), so they take
//! `opts: struct { shape_filter: ?*ShapeFilter = null }` and use a local default filter when none is given.

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const AABox = zolt.AABox;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;

const BodyID = @import("../Body/BodyID.zig").BodyID;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const SubShapeIDCreator = @import("Shape/SubShapeID.zig").SubShapeIDCreator;
const PhysicsMaterial = @import("PhysicsMaterial.zig").PhysicsMaterial;
const RRayCast = @import("RayCast.zig").RRayCast;
const RayCastSettings = @import("RayCast.zig").RayCastSettings;
const RayCastResult = @import("CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("CollideShape.zig").CollideShapeSettings;
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("CollisionDispatch.zig");

/// `RVec3(Vec3)` (the foundation adds precision independent aliases to Math instead: Vec3.toVec3 / DVec3.fromVec3)
pub fn rvec3FromVec3(v: Vec3) RVec3 {
    return if (Core.double_precision) RVec3.fromVec3(v) else v;
}

/// `Vec3(RVec3)`
pub fn vec3FromRVec3(v: RVec3) Vec3 {
    return if (Core.double_precision) v.toVec3() else v;
}

/// Temporary data structure that contains a shape and a transform.
/// This structure can be obtained from a body (e.g. after a broad phase query) under lock protection.
/// The lock can then be released and collision detection operations can be safely performed since
/// the class takes a reference on the shape and does not use anything from the body anymore.
pub const TransformedShape = struct {
    /// Center of mass world position of the shape
    shape_position_com: RVec3 = RVec3.zero(),
    /// Rotation of the shape
    shape_rotation: Quat = Quat.identity(),
    /// The shape itself
    shape: RefConst(Shape) = .empty,
    /// Not stored as Vec3 to get a nicely packed structure
    shape_scale: Float3 = .{ .x = 1, .y = 1, .z = 1 },
    /// Optional body ID from which this shape comes
    body_id: BodyID = .invalid,
    /// Optional sub shape ID creator for the shape (can be used when expanding compound shapes into multiple transformed shapes)
    sub_shape_id_creator: SubShapeIDCreator = .{},

    comptime {
        if (@sizeOf(usize) == 8) std.debug.assert(@sizeOf(TransformedShape) == if (Core.double_precision) 96 else 64); // Not properly packed
    }

    /// Constructor, adds a reference to `shape`
    pub fn init(position_com: RVec3, rotation: Quat, shape: ?*const Shape, body_id: BodyID, sub_shape_id_creator: SubShapeIDCreator) TransformedShape {
        return .{ .shape_position_com = position_com, .shape_rotation = rotation, .shape = .init(shape), .body_id = body_id, .sub_shape_id_creator = sub_shape_id_creator };
    }

    /// Copy constructor: adds a reference to the shape
    pub fn clone(self: *const TransformedShape) TransformedShape {
        var copy = self.*;
        copy.shape = self.shape.clone();
        return copy;
    }

    /// Destructor: releases the shape
    pub fn deinit(self: *TransformedShape) void {
        self.shape.deinit();
    }

    /// Helper function to return the body id from a transformed shape. If the transformed shape is null an invalid body ID will be returned. (sGetBodyID)
    pub fn getBodyID(ts: ?*const TransformedShape) BodyID {
        return if (ts) |t| t.body_id else .invalid;
    }

    /// Get/set the scale of the shape
    pub fn getShapeScale(self: *const TransformedShape) Vec3 {
        return Vec3.fromFloat3(self.shape_scale);
    }

    pub fn setShapeScale(self: *TransformedShape, scale: Vec3) void {
        scale.storeFloat3(&self.shape_scale);
    }

    /// Calculates the transform for this shapes's center of mass (excluding scale)
    pub fn getCenterOfMassTransform(self: *const TransformedShape) RMat44 {
        return RMat44.rotationTranslation(self.shape_rotation, self.shape_position_com);
    }

    /// Calculates the inverse of the transform for this shape's center of mass (excluding scale)
    pub fn getInverseCenterOfMassTransform(self: *const TransformedShape) RMat44 {
        return RMat44.inverseRotationTranslation(self.shape_rotation, self.shape_position_com);
    }

    /// Cast a ray and find the closest hit. Returns true if it finds a hit. Hits further than hit.fraction will not be considered and in this case hit will remain unmodified (and the function will return false).
    /// Convex objects will be treated as solid (meaning if the ray starts inside, you'll get a hit fraction of 0) and back face hits are returned.
    /// If you want the surface normal of the hit use `getWorldSpaceSurfaceNormal(hit.sub_shape_id2, ray.getPointOnRay(hit.fraction))` on this object.
    pub fn castRay(self: *const TransformedShape, ray: RRayCast, hit: *RayCastResult) bool {
        if (self.shape.get()) |shape| {
            // Transform the ray to local space, note that this drops precision which is possible because we're in local space now
            var local_ray = ray.transformed(self.getInverseCenterOfMassTransform()).toRayCast();

            // Scale the ray
            const inv_scale = self.getShapeScale().reciprocal();
            local_ray.origin = local_ray.origin.mul(inv_scale);
            local_ray.direction = local_ray.direction.mul(inv_scale);

            // Cast the ray on the shape
            const sub_shape_id = self.sub_shape_id_creator;
            if (shape.castRay(local_ray, sub_shape_id, hit)) {
                // Set body ID on the hit result
                hit.body_id = self.body_id;
                return true;
            }
        }
        return false;
    }

    /// Cast a ray, allows collecting multiple hits. Note that this version is more flexible but also slightly slower than the castRay function that returns only a single hit.
    /// If you want the surface normal of the hit use `getWorldSpaceSurfaceNormal(collected sub_shape_id2, ray.getPointOnRay(collected fraction))` on this object.
    pub fn castRayCollector(self: *const TransformedShape, ray: RRayCast, ray_cast_settings: *const RayCastSettings, collector: *CastRayCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            // Transform the ray to local space, note that this drops precision which is possible because we're in local space now
            var local_ray = ray.transformed(self.getInverseCenterOfMassTransform()).toRayCast();

            // Scale the ray
            const inv_scale = self.getShapeScale().reciprocal();
            local_ray.origin = local_ray.origin.mul(inv_scale);
            local_ray.direction = local_ray.direction.mul(inv_scale);

            // Cast the ray on the shape
            const sub_shape_id = self.sub_shape_id_creator;
            shape.castRayCollector(local_ray, ray_cast_settings, sub_shape_id, collector, shape_filter);
        }
    }

    /// Check if point is inside any shapes. For this tests all shapes are treated as if they were solid.
    pub fn collidePoint(self: *const TransformedShape, point: RVec3, collector: *CollidePointCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            // Transform and scale the point to local space
            const local_point = vec3FromRVec3(self.getInverseCenterOfMassTransform().mulRVec3(point)).div(self.getShapeScale());

            // Do point collide on the shape
            const sub_shape_id = self.sub_shape_id_creator;
            shape.collidePoint(local_point, sub_shape_id, collector, shape_filter);
        }
    }

    /// Collide a shape and report any hits to collector
    /// @param shape Shape to test
    /// @param shape_scale Scale in local space of shape
    /// @param center_of_mass_transform Center of mass transform for the shape
    /// @param collide_shape_settings Settings
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. mCenterOfMassTransform.GetTranslation() since floats are most accurate near the origin
    /// @param collector Collector that receives the hits
    pub fn collideShape(self: *const TransformedShape, shape: *const Shape, shape_scale: Vec3, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |my_shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            const sub_shape_id1: SubShapeIDCreator = .{};
            const sub_shape_id2 = self.sub_shape_id_creator;
            const transform1 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate()).toMat44();
            const transform2 = self.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();
            CollisionDispatch.collideShapeVsShape(shape, my_shape, shape_scale, self.getShapeScale(), transform1, transform2, sub_shape_id1, sub_shape_id2, collide_shape_settings, collector, shape_filter);
        }
    }

    /// Collect the leaf transformed shapes of all leaf shapes of this shape
    /// box is specified in world space.
    pub fn collectTransformedShapes(self: *const TransformedShape, box: AABox, collector: *TransformedShapeCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            const MyCollector = struct {
                pub const overrides = .{.addHit};

                base: TransformedShapeCollector,
                collector: *TransformedShapeCollector,
                shape_position_com: RVec3,

                pub fn addHit(c: *@This(), result: *const TransformedShape) void {
                    // Apply the center of mass offset
                    var ts = result.clone();
                    defer ts.deinit();
                    ts.shape_position_com = ts.shape_position_com.add(c.shape_position_com);

                    // Pass hit on to child collector
                    c.collector.addHit(&ts);

                    // Update early out fraction based on child collector
                    c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                }
            };

            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector
            collector.setContext(self);

            // Wrap the collector so we can add the center of mass precision, we do this to avoid losing precision because CollectTransformedShapes uses single precision floats
            var my_collector: MyCollector = .{ .base = .initFrom(MyCollector, collector), .collector = collector, .shape_position_com = self.shape_position_com };

            // Take box to local space for the shape
            var local_box = box;
            local_box.translateRVec3(self.shape_position_com.negate());

            shape.collectTransformedShapes(local_box, Vec3.zero(), self.shape_rotation, self.getShapeScale(), self.sub_shape_id_creator, &my_collector.base, shape_filter);
        }
    }

    /// To start iterating over triangles, call this function first.
    /// To get the actual triangles call getTrianglesNext.
    /// @param context A temporary buffer and should remain untouched until the last call to getTrianglesNext.
    /// @param box The world space bounding in which you want to get the triangles.
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. mShapePositionCOM since floats are most accurate near the origin
    pub fn getTrianglesStart(self: *const TransformedShape, context: *Shape.GetTrianglesContext, box: AABox, base_offset: RVec3) void {
        if (self.shape.get()) |shape| {
            // Take box to local space for the shape
            var local_box = box;
            local_box.translateRVec3(base_offset.negate());

            shape.getTrianglesStart(context, local_box, vec3FromRVec3(self.shape_position_com.sub(base_offset)), self.shape_rotation, self.getShapeScale());
        }
    }

    /// Call this repeatedly to get all triangles in the box.
    pub fn getTrianglesNext(self: *const TransformedShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        if (self.shape.get()) |shape|
            return shape.getTrianglesNext(context, max_triangles_requested, out_triangle_vertices, out_materials);
        return 0;
    }
};
