//! Port of: Jolt/Physics/Collision/ShapeCast.h (prototype)
//! Status: partial
//! Missing: RShapeCast (the real port makes ShapeCastT(Vec, Mat) generic like RayCast.zig)
//!
//! Value inheritance (D12):
//! - `ShapeCastSettings : CollideSettingsBase` is flattened (never passed as its base), checked by `virtual.checkPrefix`.
//! - `ShapeCastResult : CollideShapeResult` embeds its base as `base`, because Jolt passes a ShapeCastResult where a
//!   `const CollideShapeResult &` is expected (CharacterVirtual::sFillContactProperties, ContactListener in CCD):
//!   such callers pass `&result.base`. Inherited fields are `result.base.penetration_depth`.
//!
//! `mShape` is a raw `const Shape *` in Jolt ("does not assume ownership over the shape"), so it stays `*const Shape`.

const zolt = @import("zolt");
const Vec3 = zolt.Vec3;
const Mat44 = zolt.Mat44;
const AABox = zolt.AABox;
const virtual = @import("../../Core/Virtual.zig");
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

/// Structure that holds a single shape cast (a shape moving along a linear path in space with no rotation)
pub const ShapeCast = struct {
    /// Shape that's being cast (cannot be mesh shape). Note that this structure does not assume ownership over the shape for performance reasons.
    shape: *const Shape,
    /// Scale in local space of the shape being cast (scales relative to its center of mass)
    scale: Vec3,
    /// Start position and orientation of the center of mass of the shape (construct using fromWorldTransform if you have a world transform for your shape)
    center_of_mass_start: Mat44,
    /// Direction and length of the cast (anything beyond this length will not be reported as a hit)
    direction: Vec3,
    /// Cached shape's world bounds, calculated in constructor
    shape_world_bounds: AABox,

    /// Constructor
    pub fn initWithBounds(shape: *const Shape, scale: Vec3, center_of_mass_start: Mat44, direction: Vec3, world_space_bounds: AABox) ShapeCast {
        return .{ .shape = shape, .scale = scale, .center_of_mass_start = center_of_mass_start, .direction = direction, .shape_world_bounds = world_space_bounds };
    }

    /// Constructor
    pub fn init(shape: *const Shape, scale: Vec3, center_of_mass_start: Mat44, direction: Vec3) ShapeCast {
        return initWithBounds(shape, scale, center_of_mass_start, direction, shape.getWorldSpaceBounds(center_of_mass_start, scale));
    }

    /// Construct a shape cast using a world transform for a shape instead of a center of mass transform
    pub fn fromWorldTransform(shape: *const Shape, scale: Vec3, world_transform: Mat44, direction: Vec3) ShapeCast {
        return init(shape, scale, world_transform.preTranslated(shape.getCenterOfMass()), direction);
    }

    /// Transform this shape cast using transform. Multiply transform on the left left hand side.
    pub fn postTransformed(self: *const ShapeCast, transform: Mat44) ShapeCast {
        const start = transform.mul(self.center_of_mass_start);
        const direction = transform.multiply3x3(self.direction);
        return init(self.shape, self.scale, start, direction);
    }

    /// Translate this shape cast by translation.
    pub fn postTranslated(self: *const ShapeCast, translation: Vec3) ShapeCast {
        return init(self.shape, self.scale, self.center_of_mass_start.postTranslated(translation), self.direction);
    }

    /// Get point with fraction on ray from center_of_mass_start to center_of_mass_start + direction (0 = start of ray, 1 = end of ray)
    pub fn getPointOnRay(self: *const ShapeCast, fraction: f32) Vec3 {
        return self.center_of_mass_start.getTranslation().add(self.direction.mulScalar(fraction));
    }
};

/// Settings to be passed with a shape cast
pub const ShapeCastSettings = struct {
    // CollideSettingsBase (flattened)
    active_edge_mode: ActiveEdgeMode = .collide_only_with_active,
    collect_faces_mode: CollectFacesMode = .no_faces,
    collision_tolerance: f32 = PhysicsSettings.default_collision_tolerance,
    penetration_tolerance: f32 = PhysicsSettings.default_penetration_tolerance,
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
    pub fn init(fraction: f32, contact_point1: Vec3, contact_point2: Vec3, contact_normal_or_penetration_depth: Vec3, back_face_hit: bool, sub_shape_id1: SubShapeID, sub_shape_id2: SubShapeID, body_id2: BodyID) ShapeCastResult {
        return .{
            .base = .init(contact_point1, contact_point2, contact_normal_or_penetration_depth, contact_point2.sub(contact_point1).length(), sub_shape_id1, sub_shape_id2, body_id2),
            .fraction = fraction,
            .is_back_face_hit = back_face_hit,
        };
    }

    /// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. The fraction and penetration depth are combined in such a way that deeper hits at fraction 0 go first.
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
