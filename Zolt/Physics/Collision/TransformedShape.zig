//! Port of: Jolt/Physics/Collision/TransformedShape.h, Jolt/Physics/Collision/TransformedShape.cpp
//! Status: complete
//!
//! D12 (Docs/Zolt/CollisionArchitecture.md): a value type that owns a reference to its shape (`RefConst<Shape>
//! mShape`), so a query can keep using it after NarrowPhaseQuery released the body lock while another thread replaces
//! the body's shape. Zig has no copy constructor: `init` adds a reference, `clone()` is the copy constructor, `deinit()`
//! the destructor, a plain `=` moves. Collectors that store TransformedShapes clone them (CollisionCollectorImpl.zig).
//! The default constructor is `.{}` (no shape).
//!
//! D7: the query entry points write `body_id2` of the shape filter (a `mutable` member in C++), so they take
//! `opts: struct { shape_filter: ?*ShapeFilter = null }` and use a local default filter when none is given (Rule M).
//!
//! Renames: the `CastRay` overloads are `castRay` / `castRayCollector`; `GetSubShapeTransformedShape(id, outRemainder)`
//! returns `Shape.SubShapeTransformedShape{ transformed_shape, remainder }`; the two `SetWorldTransform` overloads are
//! `setWorldTransform(position, rotation, scale)` / `setWorldTransformRMat44(transform)`; `sGetBodyID` is `getBodyID`.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Float3 = @import("../../Math/Float3.zig").Float3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Quat = @import("../../Math/Quat.zig").Quat;
const RealFile = @import("../../Math/Real.zig");
const RMat44 = RealFile.RMat44;
const RVec3 = RealFile.RVec3;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const SubShapeIDFile = @import("Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RRayCast = RayCastFile.RRayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const RShapeCast = ShapeCastFile.RShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;
const CollisionDispatch = @import("CollisionDispatch.zig");

/// Temporary data structure that contains a shape and a transform.
/// This structure can be obtained from a body (e.g. after a broad phase query) under lock protection.
/// The lock can then be released and collision detection operations can be safely performed since
/// the class takes a reference on the shape and does not use anything from the body anymore.
pub const TransformedShape = struct {
    /// Center of mass world position of the shape
    shape_position_com: RVec3 = undefined,
    /// Rotation of the shape
    shape_rotation: Quat = undefined,
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
        std.debug.assert(@alignOf(TransformedShape) == @max(Core.vector_alignment, RealFile.rvector_alignment)); // Not properly aligned
    }

    /// Use the context from Shape
    pub const GetTrianglesContext = Shape.GetTrianglesContext;

    /// Constructor, adds a reference to `shape`
    pub fn init(position_com: RVec3, rotation: Quat, shape: ?*const Shape, body_id: BodyID, opts: struct { sub_shape_id_creator: SubShapeIDCreator = .{} }) TransformedShape {
        return .{ .shape_position_com = position_com, .shape_rotation = rotation, .shape = .init(shape), .body_id = body_id, .sub_shape_id_creator = opts.sub_shape_id_creator };
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

    /// Cast a ray and find the closest hit. Returns true if it finds a hit. Hits further than hit.fraction will not be considered and in this case hit will remain unmodified (and the function will return false).
    /// Convex objects will be treated as solid (meaning if the ray starts inside, you'll get a hit fraction of 0) and back face hits are returned.
    /// If you want the surface normal of the hit use GetWorldSpaceSurfaceNormal(hit.sub_shape_id2, ray.getPointOnRay(hit.fraction)) on this object.
    pub fn castRay(self: *const TransformedShape, ray: RRayCast, hit: *RayCastResult) bool {
        if (self.shape.get()) |shape| {
            // Transform the ray to local space, note that this drops precision which is possible because we're in local space now
            var local_ray: RayCast = ray.transformed(self.getInverseCenterOfMassTransform()).toRayCast();

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

    /// Cast a ray, allows collecting multiple hits. Note that this version is more flexible but also slightly slower than the CastRay function that returns only a single hit.
    /// If you want the surface normal of the hit use GetWorldSpaceSurfaceNormal(collected sub shape ID, ray.getPointOnRay(collected fraction)) on this object.
    pub fn castRayCollector(self: *const TransformedShape, ray: RRayCast, ray_cast_settings: *const RayCastSettings, collector: *CastRayCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            // Transform the ray to local space, note that this drops precision which is possible because we're in local space now
            var local_ray: RayCast = ray.transformed(self.getInverseCenterOfMassTransform()).toRayCast();

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
    /// For a mesh shape, this test will only provide sensible information if the mesh is a closed manifold.
    /// For each shape that collides, collector will receive a hit
    pub fn collidePoint(self: *const TransformedShape, point: RVec3, collector: *CollidePointCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            // Transform and scale the point to local space
            const local_point = self.getInverseCenterOfMassTransform().mulRVec3(point).toVec3().div(self.getShapeScale());

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
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. mShapePositionCOM since floats are most accurate near the origin
    /// @param collector Collector that receives the hits
    /// @param opts.shape_filter Filter that allows you to reject collisions
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

    /// Cast a shape and report any hits to collector
    /// @param shape_cast The shape cast and its position and direction
    /// @param shape_cast_settings Settings for the shape cast
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. mShapePositionCOM or shape_cast.center_of_mass_start.getTranslation() since floats are most accurate near the origin
    /// @param collector Collector that receives the hits
    /// @param opts.shape_filter Filter that allows you to reject collisions
    pub fn castShape(self: *const TransformedShape, shape_cast: *const RShapeCast, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector, opts: struct { shape_filter: ?*ShapeFilter = null }) void {
        if (self.shape.get()) |shape| {
            var default_filter: ShapeFilter = .{};
            const shape_filter = opts.shape_filter orelse &default_filter;

            // Set the context on the collector and filter
            collector.setContext(self);
            shape_filter.body_id2 = self.body_id;

            // Get the shape cast relative to the base offset and convert it to floats
            const local_shape_cast: ShapeCast = shape_cast.postTranslated(base_offset.negate()).toShapeCast();

            // Get center of mass of object we're casting against relative to the base offset and convert it to floats
            const center_of_mass_transform2 = self.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();

            const sub_shape_id1: SubShapeIDCreator = .{};
            const sub_shape_id2 = self.sub_shape_id_creator;
            CollisionDispatch.castShapeVsShapeWorldSpace(&local_shape_cast, shape_cast_settings, shape, self.getShapeScale(), shape_filter, center_of_mass_transform2, sub_shape_id1, sub_shape_id2, collector);
        }
    }

    /// Collect the leaf transformed shapes of all leaf shapes of this shape
    /// box is the world space axis aligned box which leaf shapes should collide with
    pub fn collectTransformedShapes(self: *const TransformedShape, box: AABox, collector: *TransformedShapeCollector, opts: struct { shape_filter: ?*const ShapeFilter = null }) void {
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

            const default_filter: ShapeFilter = .{};
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
    /// To get the actual triangles call GetTrianglesNext.
    /// @param context A temporary buffer and should remain untouched until the last call to GetTrianglesNext.
    /// @param box The world space bounding in which you want to get the triangles.
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. box.getCenter() since floats are most accurate near the origin
    pub fn getTrianglesStart(self: *const TransformedShape, context: *GetTrianglesContext, box: AABox, base_offset: RVec3) void {
        if (self.shape.get()) |shape| {
            // Take box to local space for the shape
            var local_box = box;
            local_box.translateRVec3(base_offset.negate());

            shape.getTrianglesStart(context, local_box, self.shape_position_com.sub(base_offset).toVec3(), self.shape_rotation, self.getShapeScale());
        }
    }

    /// Call this repeatedly to get all triangles in the box.
    /// out_triangle_vertices should be large enough to hold 3 * max_triangles_requested entries
    /// out_materials (if it is not null) should contain max_triangles_requested entries
    /// The function returns the amount of triangles that it found (which will be <= max_triangles_requested), or 0 if there are no more triangles.
    /// Note that the function can return a value < max_triangles_requested and still have more triangles to process (triangles can be returned in blocks)
    /// Note that the function may return triangles outside of the requested box, only coarse culling is performed on the returned triangles
    pub fn getTrianglesNext(self: *const TransformedShape, context: *GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        if (self.shape.get()) |shape|
            return shape.getTrianglesNext(context, max_triangles_requested, out_triangle_vertices, out_materials)
        else
            return 0;
    }

    /// Get the scale of the shape as a Vec3
    pub fn getShapeScale(self: *const TransformedShape) Vec3 {
        return Vec3.loadFloat3Unsafe(&self.shape_scale);
    }

    /// Set the scale of the shape as a Vec3
    pub fn setShapeScale(self: *TransformedShape, scale: Vec3) void {
        scale.storeFloat3(&self.shape_scale);
    }

    /// Calculates the transform for this shape's center of mass (excluding scale)
    pub fn getCenterOfMassTransform(self: *const TransformedShape) RMat44 {
        return RMat44.rotationTranslation(self.shape_rotation, self.shape_position_com);
    }

    /// Calculates the inverse of the transform for this shape's center of mass (excluding scale)
    pub fn getInverseCenterOfMassTransform(self: *const TransformedShape) RMat44 {
        return RMat44.inverseRotationTranslation(self.shape_rotation, self.shape_position_com);
    }

    /// Sets the world transform (including scale) of this transformed shape (not from the center of mass but in the space the shape was created)
    pub fn setWorldTransform(self: *TransformedShape, position: RVec3, rotation: Quat, scale: Vec3) void {
        self.shape_position_com = position.addVec3(rotation.mulVec3(scale.mul(self.shape.get().?.getCenterOfMass())));
        self.shape_rotation = rotation;
        self.setShapeScale(scale);
    }

    /// Sets the world transform (including scale) of this transformed shape (not from the center of mass but in the space the shape was created)
    pub fn setWorldTransformRMat44(self: *TransformedShape, transform: RMat44) void {
        const decomposed = transform.decompose();
        const rot_trans = decomposed.rotation_translation;
        self.setWorldTransform(rot_trans.getTranslation(), rot_trans.getQuaternion(), decomposed.scale);
    }

    /// Calculates the world transform including scale of this shape (not from the center of mass but in the space the shape was created)
    pub fn getWorldTransform(self: *const TransformedShape) RMat44 {
        var transform = RMat44.rotationQuat(self.shape_rotation).preScaled(self.getShapeScale());
        transform.setTranslation(self.shape_position_com.subVec3(transform.multiply3x3(self.shape.get().?.getCenterOfMass())));
        return transform;
    }

    /// Get the world space bounding box for this transformed shape
    pub fn getWorldSpaceBounds(self: *const TransformedShape) AABox {
        return if (self.shape.get()) |shape| shape.getWorldSpaceBoundsRMat44(self.getCenterOfMassTransform(), self.getShapeScale()) else AABox.empty;
    }

    /// Make sub_shape_id relative to mShape. When mSubShapeIDCreator is not empty, this is needed in order to get the correct path to the sub shape.
    pub fn makeSubShapeIDRelativeToShape(self: *const TransformedShape, sub_shape_id: SubShapeID) SubShapeID {
        // Take off the sub shape ID part that comes from mSubShapeIDCreator and validate that it is the same
        const num_bits_written = self.sub_shape_id_creator.getNumBitsWritten();
        const popped = sub_shape_id.popID(num_bits_written);
        if (Core.enable_asserts) {
            const root_id = popped.id;
            const mask: u32 = @truncate((@as(u64, 1) << @intCast(num_bits_written)) - 1); // (1 << num_bits_written) - 1, without overflow for 32 bits
            std.debug.assert(root_id == (self.sub_shape_id_creator.getID().getValue() & mask));
        }
        return popped.remainder;
    }

    /// Get surface normal of a particular sub shape and its world space surface position on this body.
    /// Note: When you have a CollideShapeResult or ShapeCastResult you should use -mPenetrationAxis.Normalized() as contact normal as GetWorldSpaceSurfaceNormal will only return face normals (and not vertex or edge normals).
    pub fn getWorldSpaceSurfaceNormal(self: *const TransformedShape, sub_shape_id: SubShapeID, position: RVec3) Vec3 {
        const inv_com = self.getInverseCenterOfMassTransform();
        const scale = self.getShapeScale(); // See comment at ScaledShape::GetSurfaceNormal for the math behind the scaling of the normal
        return inv_com.multiply3x3Transposed(self.shape.get().?.getSurfaceNormal(self.makeSubShapeIDRelativeToShape(sub_shape_id), inv_com.mulRVec3(position).toVec3().div(scale)).div(scale)).normalized();
    }

    /// Get the vertices of the face that faces direction the most (includes any convex radius). Note that this function can only return faces of
    /// convex shapes or triangles, which is why a sub shape ID to get to that leaf must be provided.
    /// @param sub_shape_id Sub shape ID of target shape
    /// @param direction Direction that the face should be facing (in world space)
    /// @param base_offset The vertices will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. mShapePositionCOM since floats are most accurate near the origin
    /// @param out_vertices Resulting face. Note the returned face can have a single point if the shape doesn't have polygons to return (e.g. because it's a sphere). The face will be returned in world space.
    pub fn getSupportingFace(self: *const TransformedShape, sub_shape_id: SubShapeID, direction: Vec3, base_offset: RVec3, out_vertices: *Shape.SupportingFace) void {
        const com = self.getCenterOfMassTransform().postTranslatedRVec3(base_offset.negate()).toMat44();
        self.shape.get().?.getSupportingFace(self.makeSubShapeIDRelativeToShape(sub_shape_id), com.multiply3x3Transposed(direction), self.getShapeScale(), com, out_vertices);
    }

    /// Get material of a particular sub shape
    pub fn getMaterial(self: *const TransformedShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        return self.shape.get().?.getMaterial(self.makeSubShapeIDRelativeToShape(sub_shape_id));
    }

    /// Get the user data of a particular sub shape
    pub fn getSubShapeUserData(self: *const TransformedShape, sub_shape_id: SubShapeID) u64 {
        return self.shape.get().?.getSubShapeUserData(self.makeSubShapeIDRelativeToShape(sub_shape_id));
    }

    /// Get the direct child sub shape and its transform for a sub shape ID.
    /// @param sub_shape_id Sub shape ID that indicates the path to the leaf shape
    /// @return Direct child sub shape and its transform (note that the body ID and sub shape ID will be invalid; the caller deinits it) and the remainder of the sub shape ID after removing the sub shape
    pub fn getSubShapeTransformedShape(self: *const TransformedShape, sub_shape_id: SubShapeID) Shape.SubShapeTransformedShape {
        var result = self.shape.get().?.getSubShapeTransformedShape(sub_shape_id, Vec3.zero(), self.shape_rotation, self.getShapeScale());
        result.transformed_shape.shape_position_com = result.transformed_shape.shape_position_com.add(self.shape_position_com);
        return result;
    }

    /// Helper function to return the body id from a transformed shape. If the transformed shape is null an invalid body ID will be returned. (sGetBodyID)
    pub fn getBodyID(ts: ?*const TransformedShape) BodyID {
        return if (ts) |t| t.body_id else .invalid;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the test shapes of Shape/TestShapes.zig are registered as User1..User3 through zolt_user_types in the inline tests)

const testing = std.testing;
const Allocator = std.mem.Allocator;
const math = @import("../../Math/Math.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const TestShapes = @import("Shape/TestShapes.zig");
const CollisionCollectorImpl = @import("CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;

fn rvec3(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// Remembers the body ID that the entry point wrote into the filter (C++ `mutable mBodyID2`, Rule M) and counts calls
const RecordingFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    seen_body_id: *BodyID,
    accept: bool = true,

    pub fn shouldCollide(self: *const RecordingFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape2, sub_shape_id_of_shape2 };
        self.seen_body_id.* = self.base.body_id2;
        return self.accept;
    }

    pub fn shouldCollidePair(self: *const RecordingFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape1, sub_shape_id_of_shape1, shape2, sub_shape_id_of_shape2 };
        self.seen_body_id.* = self.base.body_id2;
        return self.accept;
    }
};

/// Calls through a function pointer so that the optimizer cannot see the write of body_id2 (Rule M regression test)
noinline fn castRayThroughPointer(ts: *const TransformedShape, ray: RRayCast, collector: *CastRayCollector, filter: *ShapeFilter) void {
    ts.castRayCollector(ray, &.{}, collector, .{ .shape_filter = filter });
}

test "TransformedShape: value semantics, references, transforms" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const box = try TestShapes.TestBoxShape.create(allocator, Vec3.init(1, 2, 3), .{ .center_of_mass = Vec3.init(1, 0, 0) });
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const shape = box.asShape();

    // init adds a reference, clone is the copy constructor, deinit releases
    const rotation = Quat.rotation(Vec3.axisZ(), 0.5 * math.pi);
    var ts = TransformedShape.init(rvec3(10, 0, 0), rotation, shape, .init(7), .{ .sub_shape_id_creator = SubShapeIDCreator.pushID(.{}, 1, 2) });
    defer ts.deinit();
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
    var copy = ts.clone();
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());
    copy.deinit();
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
    const empty: TransformedShape = .{};
    try expect(empty.shape.get() == null and empty.body_id.isInvalid() and empty.getShapeScale().eql(Vec3.one()));
    try expect(TransformedShape.getBodyID(&ts).eql(.init(7)) and TransformedShape.getBodyID(null).isInvalid());

    // Scale and transforms
    ts.setShapeScale(Vec3.init(2, 2, 2));
    try expect(ts.getShapeScale().eql(Vec3.replicate(2)));
    try expect(ts.getCenterOfMassTransform().eql(RMat44.rotationTranslation(rotation, rvec3(10, 0, 0))));
    try expect(ts.getInverseCenterOfMassTransform().eql(RMat44.inverseRotationTranslation(rotation, rvec3(10, 0, 0))));

    // World transform round trip (the center of mass offset is scaled and rotated)
    ts.setWorldTransform(rvec3(1, 2, 3), Quat.identity(), Vec3.init(2, 3, 4));
    try expect(ts.shape_position_com.eql(rvec3(3, 2, 3)));
    try expect(ts.getWorldTransform().getTranslation().eql(rvec3(1, 2, 3)));
    const world = RMat44.rotationTranslation(rotation, rvec3(-5, 6, 7)).preScaled(Vec3.init(1, 2, 3));
    ts.setWorldTransformRMat44(world);
    try expect(ts.getWorldTransform().isClose(world, .{ .max_dist_sq = 1.0e-10 }));
    try expect(ts.getShapeScale().isClose(Vec3.init(1, 2, 3), .{ .max_dist_sq = 1.0e-10 }));

    // World space bounds (the RMat44 overload of Shape::GetWorldSpaceBounds), empty without a shape
    try expect(ts.getWorldSpaceBounds().eql(shape.getWorldSpaceBoundsRMat44(ts.getCenterOfMassTransform(), ts.getShapeScale())));
    try expect(empty.getWorldSpaceBounds().eql(AABox.empty));

    // Sub shape IDs relative to the shape: the part of sub_shape_id_creator is removed
    const full_id = ts.sub_shape_id_creator.getID();
    try expect(ts.makeSubShapeIDRelativeToShape(full_id).isEmpty());
    try testing.expectEqual(@as(u64, 0), ts.getSubShapeUserData(full_id));
    try expect(ts.getMaterial(full_id) == PhysicsMaterial.default);

    // Surface normal and supporting face in world space
    ts.setWorldTransform(rvec3(0, 0, 0), rotation, Vec3.one());
    const normal = ts.getWorldSpaceSurfaceNormal(full_id, ts.shape_position_com.addVec3(rotation.mulVec3(Vec3.init(1.0, 0.1, 0.1))));
    try expect(normal.isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
    var face: Shape.SupportingFace = .empty;
    ts.getSupportingFace(full_id, Vec3.axisY(), rvec3(0, 0, 0), &face);
    try testing.expectEqual(@as(u32, 4), face.len);

    // Direct child of a compound
    const compound = try TestShapes.TestCompoundShape.create(allocator, shape, Vec3.init(-2, 0, 0), shape, Vec3.init(2, 0, 0));
    var compound_ts = TransformedShape.init(rvec3(100, 0, 0), Quat.identity(), compound.asShape(), .init(3), .{});
    defer compound_ts.deinit();
    var child = compound_ts.getSubShapeTransformedShape(SubShapeIDCreator.pushID(.{}, 1, 1).getID());
    defer child.transformed_shape.deinit();
    try expect(child.transformed_shape.shape.get() == shape);
    try expect(child.transformed_shape.shape_position_com.eql(rvec3(102, 0, 0)));
    try expect(child.remainder.isEmpty());
}

test "TransformedShape: queries set the context and the filter's body ID, scale and transform the inputs" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const box = try TestShapes.TestBoxShape.create(allocator, Vec3.one(), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    var ts = TransformedShape.init(rvec3(10, 0, 0), Quat.identity(), box.asShape(), .init(5), .{ .sub_shape_id_creator = SubShapeIDCreator.pushID(.{}, 1, 1) });
    defer ts.deinit();
    ts.setShapeScale(Vec3.replicate(2));

    // Closest hit: world space ray, body ID on the hit
    var hit: RayCastResult = .{};
    try expect(ts.castRay(.init(rvec3(6, 0, 0), Vec3.init(8, 0, 0)), &hit));
    try testing.expectEqual(@as(f32, 0.25), hit.fraction);
    try expect(hit.body_id.eql(.init(5)) and hit.sub_shape_id2.eql(ts.sub_shape_id_creator.getID()));
    const empty: TransformedShape = .{};
    try expect(!empty.castRay(.init(rvec3(6, 0, 0), Vec3.init(8, 0, 0)), &hit));

    // Collector version: context and the filter's body_id2 (written through a mutable pointer, Rule M)
    var seen: BodyID = .invalid;
    var filter: RecordingFilter = .{ .seen_body_id = &seen };
    var all_hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer all_hits.deinit();
    castRayThroughPointer(&ts, .init(rvec3(6, 0, 0), Vec3.init(8, 0, 0)), &all_hits.base, &filter.base);
    try all_hits.checkError();
    try expect(seen.eql(.init(5)) and filter.base.body_id2.eql(.init(5)));
    try expect(all_hits.base.getContext() == &ts);
    try testing.expectEqual(@as(usize, 1), all_hits.hits.items.len);
    try expect(all_hits.hits.items[0].body_id.eql(.init(5)));
    ts.castRayCollector(.init(rvec3(6, 0, 0), Vec3.init(8, 0, 0)), &.{}, &all_hits.base, .{}); // Default filter
    try testing.expectEqual(@as(usize, 2), all_hits.hits.items.len);

    // Collide point
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    seen = .invalid;
    ts.collidePoint(rvec3(11.5, 0, 0), &points.base, .{ .shape_filter = &filter.base });
    ts.collidePoint(rvec3(12.5, 0, 0), &points.base, .{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);
    try expect(points.hits.items[0].body_id.eql(.init(5)) and seen.eql(.init(5)));

    // Collide shape through the dispatch table (box vs box), relative to a base offset
    var sphere = TestShapes.TestSphereShape.init(allocator, 0.5);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var collide_hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collide_hits.deinit();
    seen = .invalid;
    ts.collideShape(sphere.asShape(), Vec3.one(), RMat44.translation(rvec3(12.25, 0, 0)), &.{}, rvec3(10, 0, 0), &collide_hits.base, .{ .shape_filter = &filter.base });
    try collide_hits.checkError();
    try testing.expectEqual(@as(usize, 1), collide_hits.hits.items.len);
    const c = &collide_hits.hits.items[0];
    try expect(c.contact_point_on2.isClose(Vec3.init(2, 0, 0), .{ .max_dist_sq = 1.0e-10 })); // Relative to the base offset
    try testing.expectEqual(@as(f32, 0.25), c.penetration_depth);
    try expect(c.body_id2.eql(.init(5)) and seen.eql(.init(5)));
    try expect(c.sub_shape_id2.eql(ts.sub_shape_id_creator.getID()));
    filter.accept = false; // A rejecting filter stops the dispatch
    ts.collideShape(sphere.asShape(), Vec3.one(), RMat44.translation(rvec3(12.25, 0, 0)), &.{}, rvec3(10, 0, 0), &collide_hits.base, .{ .shape_filter = &filter.base });
    try testing.expectEqual(@as(usize, 1), collide_hits.hits.items.len);
    filter.accept = true;

    // Cast shape through the dispatch table (sphere vs box)
    var cast_hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast_hits.deinit();
    const cast = RShapeCast.init(sphere.asShape(), Vec3.one(), RMat44.translation(rvec3(4, 0, 0)), Vec3.init(8, 0, 0));
    ts.castShape(&cast, &.{}, rvec3(10, 0, 0), &cast_hits.base, .{});
    try cast_hits.checkError();
    try testing.expectEqual(@as(usize, 1), cast_hits.hits.items.len);
    try expect(@abs(cast_hits.hits.items[0].fraction - 0.4375) < 1.0e-6);
    try expect(cast_hits.hits.items[0].base.body_id2.eql(.init(5)));
    try expect(cast_hits.hits.items[0].base.contact_point_on2.isClose(Vec3.init(-2, 0, 0), .{ .max_dist_sq = 1.0e-10 }));
}

test "TransformedShape: collectTransformedShapes adds the center of mass position, getTriangles" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const sphere = try TestShapes.TestSphereShape.create(allocator, 1.0);
    const box = try TestShapes.TestBoxShape.create(allocator, Vec3.one(), .{});
    const compound = try TestShapes.TestCompoundShape.create(allocator, sphere.asShape(), Vec3.init(-2, 0, 0), box.asShape(), Vec3.init(2, 0, 0));
    var compound_ref = RefConst(Shape).init(compound.asShape());
    defer compound_ref.deinit();
    var ts = TransformedShape.init(rvec3(100, 0, 0), Quat.identity(), compound.asShape(), .init(9), .{});
    defer ts.deinit();

    {
        var leaves = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer leaves.deinit();
        ts.collectTransformedShapes(.init(Vec3.replicate(-1000), Vec3.replicate(1000)), &leaves.base, .{});
        try leaves.checkError();
        try testing.expectEqual(@as(usize, 2), leaves.hits.items.len);
        try expect(leaves.hits.items[0].shape.get() == sphere.asShape() and leaves.hits.items[1].shape.get() == box.asShape());
        try expect(leaves.hits.items[0].shape_position_com.eql(rvec3(98, 0, 0)));
        try expect(leaves.hits.items[1].shape_position_com.eql(rvec3(102, 0, 0)));
        try expect(leaves.hits.items[1].body_id.eql(.init(9)));
        try expect(leaves.hits.items[1].sub_shape_id_creator.getID().eql(SubShapeIDCreator.pushID(.{}, 1, 1).getID()));
        try testing.expectEqual(@as(u32, 2), box.asShape().getRefCount()); // The compound + the clone stored by the collector
    }
    try testing.expectEqual(@as(u32, 1), box.asShape().getRefCount());

    // An any hit collector stops after the first leaf (the wrapper follows the early out fraction)
    var first = CollisionCollectorImpl.AnyHitCollisionCollector(TransformedShapeCollector).init();
    defer first.deinit();
    ts.collectTransformedShapes(.init(Vec3.replicate(-1000), Vec3.replicate(1000)), &first.base, .{});
    try expect(first.hadHit() and first.hit.shape.get() == sphere.asShape());

    // Triangles relative to a base offset
    var box_ts = TransformedShape.init(rvec3(50, 0, 0), Quat.identity(), box.asShape(), .invalid, .{});
    defer box_ts.deinit();
    var context: TransformedShape.GetTrianglesContext = .{};
    box_ts.getTrianglesStart(&context, .init(Vec3.replicate(-1000), Vec3.replicate(1000)), rvec3(49, 0, 0));
    var vertices: [3 * Shape.get_triangles_min_triangles_requested]Float3 = undefined;
    try testing.expectEqual(@as(u32, 12), box_ts.getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, null));
    for (vertices[0..36]) |v| try expect(v.x == 0.0 or v.x == 2.0);
    const empty: TransformedShape = .{};
    try testing.expectEqual(@as(u32, 0), empty.getTrianglesNext(&context, Shape.get_triangles_min_triangles_requested, &vertices, null));
}

test "TransformedShape: concurrent queries on a shared shape" {
    const allocator = testing.allocator;

    const sphere = try TestShapes.TestSphereShape.create(allocator, 1.5);
    const box = try TestShapes.TestBoxShape.create(allocator, Vec3.init(1, 2, 0.5), .{});
    const compound = try TestShapes.TestCompoundShape.create(allocator, sphere.asShape(), Vec3.init(-2, 1, 0), box.asShape(), Vec3.init(2, 0, 0));
    var compound_ref = RefConst(Shape).init(compound.asShape());
    defer compound_ref.deinit();

    const Worker = struct {
        fn run(shape: *const Shape, out_sum: *f32) void {
            var ts = TransformedShape.init(rvec3(0, 0, 0), Quat.identity(), shape, .init(1), .{});
            defer ts.deinit();
            var sum: f32 = 0;
            var i: u32 = 0;
            while (i < 500) : (i += 1) {
                const y = @as(f32, @floatFromInt(i % 50)) * 0.1 - 2.5;
                var hit: RayCastResult = .{};
                if (ts.castRay(.init(rvec3(-10, y, 0), Vec3.init(20, 0.1, 0)), &hit))
                    sum += hit.fraction;
            }
            out_sum.* = sum;
        }
    };

    var expected: f32 = 0;
    Worker.run(compound.asShape(), &expected);
    try testing.expect(expected > 0);
    var sums: [4]f32 = @splat(0);
    var threads: [4]std.Thread = undefined;
    for (&threads, &sums) |*t, *s| t.* = try std.Thread.spawn(.{}, Worker.run, .{ compound.asShape(), s });
    for (threads) |t| t.join();
    for (sums) |s| try testing.expectEqual(expected, s);
    try testing.expectEqual(@as(u32, 1), compound.asShape().getRefCount());
}
