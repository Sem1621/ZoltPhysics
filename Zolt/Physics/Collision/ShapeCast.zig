//! Port of: Jolt/Physics/Collision/ShapeCast.h
//! Status: complete
//!
//! `ShapeCastT<Vec, Mat, ShapeCastType>` is the comptime function `ShapeCastT(Vec, Mat, kind)` like `RayCastT` in
//! RayCast.zig: the third parameter is a `ShapeCastKind` instead of the derived class, which also keeps `ShapeCast`
//! (Vec3 / Mat44) and `RShapeCast` (RVec3 / RMat44) different types in single precision builds. Converting between
//! them is explicit like in C++: `RShapeCast.fromShapeCast(cast)` (explicit RShapeCast(const ShapeCast &)) and
//! `r_cast.toShapeCast()` (explicit operator ShapeCast()); these two only exist on RShapeCast.
//!
//! - The constructors are `initWithBounds` (with the world space bounds) and `init` (computes the bounds with
//!   `Shape::GetWorldSpaceBounds`, the virtual Mat44 version for ShapeCast and the DMat44 version for RShapeCast in
//!   double precision, like C++ overload resolution).
//! - `postTransformed` stores `inTransform * mCenterOfMassStart` in a `Mat44`, so in C++ it only compiles when Mat is
//!   Mat44 (ShapeCast, and RShapeCast in single precision). Zolt declares it the same way: `RShapeCast.postTransformed`
//!   is `void` in double precision.
//! - `mShape` is a raw `const Shape *` in Jolt ("does not assume ownership over the shape"), so it stays `*const Shape`.
//!
//! Value inheritance (D12):
//! - `ShapeCastSettings : CollideSettingsBase` is flattened (never passed as its base), checked by `virtual.checkPrefix`.
//! - `ShapeCastResult : CollideShapeResult` embeds its base as `base`, because Jolt passes a ShapeCastResult where a
//!   `const CollideShapeResult &` is expected (CharacterVirtual::sFillContactProperties, ContactListener in CCD):
//!   such callers pass `&result.base`. Inherited fields are `result.base.penetration_depth`. The default constructor
//!   (`.{}`) leaves `fraction` and `is_back_face_hit` uninitialized like in C++.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const PhysicsSettings = @import("../PhysicsSettings.zig");
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const Shape = @import("Shape/Shape.zig").Shape;
const BackFaceMode = @import("BackFaceMode.zig").BackFaceMode;
const ActiveEdgeMode = @import("ActiveEdgeMode.zig").ActiveEdgeMode;
const CollectFacesMode = @import("CollectFacesMode.zig").CollectFacesMode;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideSettingsBase = CollideShapeFile.CollideSettingsBase;

/// Which shape cast type a ShapeCastT instance is (Jolt's ShapeCastType template parameter)
pub const ShapeCastKind = enum {
    /// ShapeCast: Mat44 start, Vec3 points
    shape_cast,
    /// RShapeCast: RMat44 start, RVec3 points
    r_shape_cast,
};

/// Structure that holds a single shape cast (a shape moving along a linear path in 3d space with no rotation)
pub fn ShapeCastT(comptime Vec: type, comptime Mat: type, comptime kind: ShapeCastKind) type {
    return struct {
        const Self = @This();

        /// Shape that's being cast (cannot be mesh shape). Note that this structure does not assume ownership over the shape for performance reasons.
        shape: *const Shape,
        /// Scale in local space of the shape being cast (scales relative to its center of mass)
        scale: Vec3,
        /// Start position and orientation of the center of mass of the shape (construct using fromWorldTransform if you have a world transform for your shape)
        center_of_mass_start: Mat,
        /// Direction and length of the cast (anything beyond this length will not be reported as a hit)
        direction: Vec3,
        /// Cached shape's world bounds, calculated in constructor
        shape_world_bounds: AABox,

        /// Constructor
        pub fn initWithBounds(shape: *const Shape, scale: Vec3, center_of_mass_start: Mat, direction: Vec3, world_space_bounds: AABox) Self {
            return .{ .shape = shape, .scale = scale, .center_of_mass_start = center_of_mass_start, .direction = direction, .shape_world_bounds = world_space_bounds };
        }

        /// Constructor
        pub fn init(shape: *const Shape, scale: Vec3, center_of_mass_start: Mat, direction: Vec3) Self {
            const bounds = if (Mat == Mat44) shape.getWorldSpaceBounds(center_of_mass_start, scale) else shape.getWorldSpaceBoundsDMat44(center_of_mass_start, scale);
            return initWithBounds(shape, scale, center_of_mass_start, direction, bounds);
        }

        /// Construct a shape cast using a world transform for a shape instead of a center of mass transform
        pub fn fromWorldTransform(shape: *const Shape, scale: Vec3, world_transform: Mat, direction: Vec3) Self {
            return init(shape, scale, world_transform.preTranslated(shape.getCenterOfMass()), direction);
        }

        /// Transform this shape cast using transform. Multiply transform on the left left hand side.
        /// (Only exists when Mat is Mat44, see the file comment.)
        pub const postTransformed = if (Mat == Mat44) postTransformedImpl else {};

        fn postTransformedImpl(self: *const Self, transform: Mat44) Self {
            const start = transform.mul(self.center_of_mass_start);
            const direction = transform.multiply3x3(self.direction);
            return init(self.shape, self.scale, start, direction);
        }

        /// Translate this shape cast by translation.
        pub fn postTranslated(self: *const Self, translation: Vec) Self {
            return init(self.shape, self.scale, self.center_of_mass_start.postTranslatedRVec3(translation), self.direction);
        }

        /// Get point with fraction on ray from center_of_mass_start to center_of_mass_start + direction (0 = start of ray, 1 = end of ray)
        pub fn getPointOnRay(self: *const Self, fraction: f32) Vec {
            return self.center_of_mass_start.getTranslation().addVec3(self.direction.mulScalar(fraction));
        }

        /// Convert from ShapeCast, converts single to double precision (RShapeCast only)
        pub const fromShapeCast = switch (kind) {
            .r_shape_cast => fromShapeCastImpl,
            .shape_cast => {},
        };

        /// Convert to ShapeCast, which implies casting from double precision to single precision (RShapeCast only)
        pub const toShapeCast = switch (kind) {
            .r_shape_cast => toShapeCastImpl,
            .shape_cast => {},
        };

        fn fromShapeCastImpl(cast: *const ShapeCast) Self {
            const start: Mat = if (Mat == Mat44) cast.center_of_mass_start else Mat.fromMat44(cast.center_of_mass_start);
            return initWithBounds(cast.shape, cast.scale, start, cast.direction, cast.shape_world_bounds);
        }

        fn toShapeCastImpl(self: *const Self) ShapeCast {
            return .initWithBounds(self.shape, self.scale, self.center_of_mass_start.toMat44(), self.direction, self.shape_world_bounds);
        }
    };
}

/// Shape cast in local space (single precision)
pub const ShapeCast = ShapeCastT(Vec3, Mat44, .shape_cast);

/// Shape cast in world space (RMat44 start, double precision with `-Ddouble_precision`)
pub const RShapeCast = ShapeCastT(RVec3, RMat44, .r_shape_cast);

/// Settings to be passed with a shape cast
pub const ShapeCastSettings = struct {
    // CollideSettingsBase (flattened)

    /// How active edges (edges that a moving object should bump into) are handled
    active_edge_mode: ActiveEdgeMode = .collide_only_with_active,
    /// If colliding faces should be collected or only the collision point
    collect_faces_mode: CollectFacesMode = .no_faces,
    /// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
    collision_tolerance: f32 = PhysicsSettings.default_collision_tolerance,
    /// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
    penetration_tolerance: f32 = PhysicsSettings.default_penetration_tolerance,
    /// When mActiveEdgeMode is CollideOnlyWithActive a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth.
    active_edge_movement_direction: Vec3 = Vec3.zero(),

    /// When > 0 the query shape is inflated by an extra convex radius, this makes it bigger in every direction by this margin. (unit: meter)
    extra_convex_radius: f32 = 0.0,

    /// How backfacing triangles should be treated (should we report moving from back to front for triangle based shapes, e.g. for MeshShape/HeightFieldShape?)
    back_face_mode_triangles: BackFaceMode = .ignore_back_faces,

    /// How backfacing convex objects should be treated (should we report starting inside an object and moving out?)
    back_face_mode_convex: BackFaceMode = .ignore_back_faces,

    /// Indicates if we want to shrink the shape by the convex radius and then expand it again. This speeds up collision detection and gives a more accurate normal at the cost of a more 'rounded' shape.
    use_shrunken_shape_and_convex_radius: bool = false,

    /// When true, and the shape is intersecting at the beginning of the cast (fraction = 0) then this will calculate the deepest penetration point (costing additional CPU time)
    return_deepest_point: bool = false,

    /// Set the backfacing mode for all shapes
    pub fn setBackFaceMode(self: *ShapeCastSettings, mode: BackFaceMode) void {
        self.back_face_mode_triangles = mode;
        self.back_face_mode_convex = mode;
    }

    comptime {
        virtual.checkPrefix(CollideSettingsBase, ShapeCastSettings);
    }
};

/// Result of a shape cast test
pub const ShapeCastResult = struct {
    /// C++ base class (passed as `&result.base` where Jolt passes it as a CollideShapeResult)
    base: CollideShapeResult = .{},
    /// This is the fraction where the shape hit the other shape: CenterOfMassOnHit = Start + value * (End - Start)
    fraction: f32 = undefined,
    /// True if the shape was hit from the back side
    is_back_face_hit: bool = undefined,

    /// Constructor
    /// @param fraction Fraction at which the cast hit
    /// @param contact_point1 Contact point on shape 1
    /// @param contact_point2 Contact point on shape 2
    /// @param contact_normal_or_penetration_depth Contact normal pointing from shape 1 to 2 or penetration depth vector when the objects are penetrating (also from 1 to 2)
    /// @param back_face_hit If this hit was a back face hit
    /// @param sub_shape_id1 Sub shape id for shape 1
    /// @param sub_shape_id2 Sub shape id for shape 2
    /// @param body_id2 BodyID that was hit
    pub fn init(fraction: f32, contact_point1: Vec3, contact_point2: Vec3, contact_normal_or_penetration_depth: Vec3, back_face_hit: bool, sub_shape_id1: SubShapeID, sub_shape_id2: SubShapeID, body_id2: BodyID) ShapeCastResult {
        return .{
            .base = .init(contact_point1, contact_point2, contact_normal_or_penetration_depth, contact_point2.sub(contact_point1).length(), sub_shape_id1, sub_shape_id2, body_id2),
            .fraction = fraction,
            .is_back_face_hit = back_face_hit,
        };
    }

    /// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. For rays/cast shapes we can just use the collision fraction. The fraction and penetration depth are combined in such a way that deeper hits at fraction 0 go first.
    pub fn getEarlyOutFraction(self: *const ShapeCastResult) f32 {
        return if (self.fraction > 0.0) self.fraction else -self.base.penetration_depth;
    }

    /// Reverses the hit result, swapping contact point 1 with contact point 2 etc.
    /// @param world_space_cast_direction Direction of the shape cast in world space
    pub fn reversed(self: *const ShapeCastResult, world_space_cast_direction: Vec3) ShapeCastResult {
        // Calculate by how much to shift the contact points
        const delta = world_space_cast_direction.mulScalar(self.fraction);

        var result: ShapeCastResult = .{};
        result.base.contact_point_on2 = self.base.contact_point_on1.sub(delta);
        result.base.contact_point_on1 = self.base.contact_point_on2.sub(delta);
        result.base.penetration_axis = self.base.penetration_axis.negate();
        result.base.penetration_depth = self.base.penetration_depth;
        result.base.sub_shape_id2 = self.base.sub_shape_id1;
        result.base.sub_shape_id1 = self.base.sub_shape_id2;
        result.base.body_id2 = self.base.body_id2;
        result.fraction = self.fraction;
        result.is_back_face_hit = self.is_back_face_hit;

        result.base.shape2_face.resize(self.base.shape1_face.len);
        for (self.base.shape1_face.constSlice(), result.base.shape2_face.slice()) |v, *out|
            out.* = v.sub(delta);

        result.base.shape1_face.resize(self.base.shape2_face.len);
        for (self.base.shape2_face.constSlice(), result.base.shape1_face.slice()) |v, *out|
            out.* = v.sub(delta);

        return result;
    }
};

test "ShapeCast / RShapeCast: construction, bounds, transforms and conversions" {
    const testing = std.testing;
    const expect = testing.expect;
    const allocator = testing.allocator;
    const TestShapes = @import("Shape/TestShapes.zig");
    const Quat = @import("../../Math/Quat.zig").Quat;
    const DMat44 = @import("../../Math/DMat44.zig").DMat44;

    var box = TestShapes.TestBoxShape.init(allocator, Vec3.init(1, 2, 3), .{ .center_of_mass = Vec3.init(1, 0, 0) });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    const shape = box.asShape();

    const start = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.25), Vec3.init(1, 2, 3));
    const scale = Vec3.init(2, 1, 1);
    const cast = ShapeCast.init(shape, scale, start, Vec3.init(0, 0, 10));
    try expect(cast.shape == shape and cast.scale.eql(scale) and cast.center_of_mass_start.eql(start));
    try expect(cast.shape_world_bounds.eql(shape.getWorldSpaceBounds(start, scale)));
    try expect(cast.getPointOnRay(0.5).eql(start.getTranslation().add(Vec3.init(0, 0, 5))));

    // From a world transform: the center of mass is applied in local space
    const from_world = ShapeCast.fromWorldTransform(shape, scale, start, Vec3.init(0, 0, 10));
    try expect(from_world.center_of_mass_start.eql(start.preTranslated(Vec3.init(1, 0, 0))));

    // Post transformed / translated recompute the bounds
    const t = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.5), Vec3.init(-1, 0, 4));
    const transformed = cast.postTransformed(t);
    try expect(transformed.center_of_mass_start.eql(t.mul(start)));
    try expect(transformed.direction.eql(t.multiply3x3(cast.direction)));
    try expect(transformed.shape_world_bounds.eql(shape.getWorldSpaceBounds(t.mul(start), scale)));
    const translated = cast.postTranslated(Vec3.init(5, 0, 0));
    try expect(translated.center_of_mass_start.eql(start.postTranslated(Vec3.init(5, 0, 0))));

    // RShapeCast: the bounds use the RMat44 overload of GetWorldSpaceBounds, explicit conversions both ways
    const r_cast = RShapeCast.fromShapeCast(&cast);
    try expect(r_cast.shape_world_bounds.eql(cast.shape_world_bounds));
    const back = r_cast.toShapeCast();
    try expect(back.center_of_mass_start.eql(cast.center_of_mass_start) and back.direction.eql(cast.direction));
    const r_start = RMat44.rotationTranslation(Quat.identity(), RVec3.init(100, 0, 0));
    const r_cast2 = RShapeCast.init(shape, Vec3.one(), r_start, Vec3.init(1, 0, 0));
    try expect(r_cast2.shape_world_bounds.eql(shape.getWorldSpaceBoundsRMat44(r_start, Vec3.one())));
    try expect(r_cast2.getPointOnRay(1.0).eql(RVec3.init(101, 0, 0)));
    try expect(r_cast2.postTranslated(RVec3.init(-100, 0, 0)).center_of_mass_start.getTranslation().eql(RVec3.zero()));
    const r_from_world = RShapeCast.fromWorldTransform(shape, Vec3.one(), r_start, Vec3.init(1, 0, 0));
    try expect(r_from_world.center_of_mass_start.getTranslation().eql(RVec3.init(101, 0, 0)));
    try expect(@TypeOf(ShapeCast.fromShapeCast) == void and @TypeOf(ShapeCast.toShapeCast) == void);
    if (Core.double_precision) {
        try expect(@TypeOf(RShapeCast.postTransformed) == void); // Jolt's RShapeCast::PostTransformed does not compile with DMat44
        _ = DMat44;
    } else {
        try expect(r_cast.postTransformed(t).center_of_mass_start.eql(transformed.center_of_mass_start));
    }
    try expect(ShapeCast != RShapeCast);
}

test "ShapeCastSettings and ShapeCastResult" {
    const testing = std.testing;
    const expect = testing.expect;

    var settings: ShapeCastSettings = .{};
    try expect(settings.back_face_mode_triangles == .ignore_back_faces and settings.back_face_mode_convex == .ignore_back_faces);
    try expect(!settings.use_shrunken_shape_and_convex_radius and !settings.return_deepest_point);
    try testing.expectEqual(@as(f32, 0.0), settings.extra_convex_radius);
    try testing.expectEqual(PhysicsSettings.default_collision_tolerance, settings.collision_tolerance);
    settings.setBackFaceMode(.collide_with_back_faces);
    try expect(settings.back_face_mode_triangles == .collide_with_back_faces and settings.back_face_mode_convex == .collide_with_back_faces);

    // The penetration depth is the distance between the contact points; deeper hits at fraction 0 go first
    var result = ShapeCastResult.init(0.5, Vec3.init(1, 0, 0), Vec3.init(1, 3, 4), Vec3.axisY(), true, .{ .value = 1 }, .{ .value = 2 }, .init(3));
    try testing.expectEqual(@as(f32, 5.0), result.base.penetration_depth);
    try testing.expectEqual(@as(f32, 0.5), result.getEarlyOutFraction());
    result.fraction = 0.0;
    try testing.expectEqual(@as(f32, -5.0), result.getEarlyOutFraction());
    result.fraction = 0.5;

    // Reversed: contact points and faces are shifted by fraction * direction, IDs and faces swapped
    result.base.shape1_face.append(Vec3.init(1, 1, 1));
    result.base.shape2_face.append(Vec3.init(2, 2, 2));
    result.base.shape2_face.append(Vec3.init(3, 3, 3));
    const reversed = result.reversed(Vec3.init(2, 0, 0));
    try expect(reversed.base.contact_point_on1.eql(Vec3.init(0, 3, 4)) and reversed.base.contact_point_on2.eql(Vec3.init(0, 0, 0)));
    try expect(reversed.base.penetration_axis.eql(Vec3.axisY().negate()));
    try testing.expectEqual(result.base.penetration_depth, reversed.base.penetration_depth);
    try expect(reversed.base.sub_shape_id1.getValue() == 2 and reversed.base.sub_shape_id2.getValue() == 1);
    try expect(reversed.base.body_id2.eql(.init(3)) and reversed.is_back_face_hit and reversed.fraction == 0.5);
    try testing.expectEqual(@as(u32, 2), reversed.base.shape1_face.len);
    try expect(reversed.base.shape1_face.get(1).eql(Vec3.init(2, 3, 3)));
    try testing.expectEqual(@as(u32, 1), reversed.base.shape2_face.len);
    try expect(reversed.base.shape2_face.get(0).eql(Vec3.init(0, 1, 1)));
}
