//! Port of: Jolt/Physics/Collision/CollideShape.h
//! Status: complete
//!
//! `CollideShapeSettings : CollideSettingsBase` (non virtual inheritance of a value type that is never passed as its
//! base) is flattened, see CastResult.zig. `CollideShapeResult` is embedded as `base` by ShapeCastResult because Jolt
//! passes a ShapeCastResult where a `const CollideShapeResult &` is expected (D12).

const zolt = @import("zolt");
const Vec3 = zolt.Vec3;
const StaticArray = zolt.StaticArray;
const virtual = @import("../../Core/Virtual.zig");
const PhysicsSettings = @import("../PhysicsSettings.zig");
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const BackFaceMode = @import("BackFaceMode.zig").BackFaceMode;
const ActiveEdgeMode = @import("ActiveEdgeMode.zig").ActiveEdgeMode;
const CollectFacesMode = @import("CollectFacesMode.zig").CollectFacesMode;

/// Class that contains all information of two colliding shapes
pub const CollideShapeResult = struct {
    pub const Face = StaticArray(Vec3, 32);

    /// Contact point on the surface of shape 1 (in world space or relative to base offset)
    contact_point_on1: Vec3 = undefined,
    /// Contact point on the surface of shape 2 (in world space or relative to base offset). If the penetration depth is 0, this will be the same as mContactPointOn1.
    contact_point_on2: Vec3 = undefined,
    /// Direction to move shape 2 out of collision along the shortest path (magnitude is meaningless, in world space). You can use -mPenetrationAxis.Normalized() as contact normal.
    penetration_axis: Vec3 = undefined,
    /// Penetration depth (move shape 2 by this distance to resolve the collision). If this value is negative, this is a separation distance.
    penetration_depth: f32 = undefined,
    /// Sub shape ID that identifies the face on shape 1
    sub_shape_id1: SubShapeID = .empty,
    /// Sub shape ID that identifies the face on shape 2
    sub_shape_id2: SubShapeID = .empty,
    /// BodyID to which shape 2 belongs to
    body_id2: BodyID = .invalid,
    /// Colliding face on shape 1 (optional result, in world space or relative to base offset)
    shape1_face: Face = .{},
    /// Colliding face on shape 2 (optional result, in world space or relative to base offset)
    shape2_face: Face = .{},

    /// Constructor
    pub fn init(contact_point_on1: Vec3, contact_point_on2: Vec3, penetration_axis: Vec3, penetration_depth: f32, sub_shape_id1: SubShapeID, sub_shape_id2: SubShapeID, body_id2: BodyID) CollideShapeResult {
        return .{
            .contact_point_on1 = contact_point_on1,
            .contact_point_on2 = contact_point_on2,
            .penetration_axis = penetration_axis,
            .penetration_depth = penetration_depth,
            .sub_shape_id1 = sub_shape_id1,
            .sub_shape_id2 = sub_shape_id2,
            .body_id2 = body_id2,
        };
    }

    /// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. We use -penetration depth to get the hit with the biggest penetration depth
    pub fn getEarlyOutFraction(self: *const CollideShapeResult) f32 {
        return -self.penetration_depth;
    }

    /// Reverses the hit result, swapping contact point 1 with contact point 2 etc.
    pub fn reversed(self: *const CollideShapeResult) CollideShapeResult {
        var result: CollideShapeResult = .{};
        result.contact_point_on2 = self.contact_point_on1;
        result.contact_point_on1 = self.contact_point_on2;
        result.penetration_axis = self.penetration_axis.negate();
        result.penetration_depth = self.penetration_depth;
        result.sub_shape_id2 = self.sub_shape_id1;
        result.sub_shape_id1 = self.sub_shape_id2;
        result.body_id2 = self.body_id2;
        result.shape2_face = self.shape1_face;
        result.shape1_face = self.shape2_face;
        return result;
    }
};

/// Settings to be passed with a collision query
pub const CollideSettingsBase = struct {
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
};

/// Settings to be passed with a collision query
pub const CollideShapeSettings = struct {
    // CollideSettingsBase (flattened)
    active_edge_mode: ActiveEdgeMode = .collide_only_with_active,
    collect_faces_mode: CollectFacesMode = .no_faces,
    collision_tolerance: f32 = PhysicsSettings.default_collision_tolerance,
    penetration_tolerance: f32 = PhysicsSettings.default_penetration_tolerance,
    active_edge_movement_direction: Vec3 = Vec3.zero(),

    /// When > 0 contacts in the vicinity of the query shape can be found. All nearest contacts that are not further away than this distance will be found (unit: meter)
    max_separation_distance: f32 = 0.0,
    /// How backfacing triangles should be treated
    back_face_mode: BackFaceMode = .ignore_back_faces,
    /// Max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm (unit: meter^2)
    internal_edge_removal_vertex_tolerance_sq: f32 = PhysicsSettings.default_internal_edge_removal_vertex_tolerance_sq,

    comptime {
        virtual.checkPrefix(CollideSettingsBase, CollideShapeSettings);
    }
};
