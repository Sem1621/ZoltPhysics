//! Port of: Jolt/Physics/Collision/CollideShape.h
//! Status: complete
//!
//! - `CollideShapeResult()` (the default constructor) is `.{}`: the vectors and the penetration depth are
//!   uninitialized like in C++ (`undefined`), the IDs are invalid / empty and the faces empty. The member wise
//!   constructor is `init`. ShapeCastResult (ShapeCast.zig) embeds it as `base`, because Jolt passes a ShapeCastResult
//!   where a `const CollideShapeResult &` is expected (D12).
//! - `CollideShapeSettings : CollideSettingsBase` is flattened (a value type that is never passed as its base, D12):
//!   CollideShapeSettings repeats the fields of CollideSettingsBase first, `virtual.checkPrefix` keeps them in sync.
//!   ShapeCastSettings does the same.

const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const StaticArray = @import("../../Core/StaticArray.zig").StaticArray;
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
    /// Penetration depth (move shape 2 by this distance to resolve the collision). If CollideShapeSettings::mMaxSeparationDistance > 0 this number can be negative to indicate that the objects are separated by -mPenetrationDepth. The contact points are the closest points in that case.
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

    /// When > 0 contacts in the vicinity of the query shape can be found. All nearest contacts that are not further away than this distance will be found.
    /// Note that in this case CollideShapeResult::mPenetrationDepth can become negative to indicate that objects are not overlapping. (unit: meter)
    max_separation_distance: f32 = 0.0,

    /// How backfacing triangles should be treated
    back_face_mode: BackFaceMode = .ignore_back_faces,

    /// Max squared distance to consider a vertex to be the same as another vertex, used by the internal edge removal algorithm to determine if two edges are shared. (unit: meter^2)
    internal_edge_removal_vertex_tolerance_sq: f32 = PhysicsSettings.default_internal_edge_removal_vertex_tolerance_sq,

    comptime {
        virtual.checkPrefix(CollideSettingsBase, CollideShapeSettings);
    }
};

test "CollideShapeResult.reversed and the settings defaults" {
    const std = @import("std");
    const expect = std.testing.expect;

    var r = CollideShapeResult.init(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(0, -1, 0), 0.25, .{ .value = 1 }, .{ .value = 2 }, .init(9));
    r.shape1_face.append(Vec3.init(7, 8, 9));
    try std.testing.expectEqual(@as(f32, -0.25), r.getEarlyOutFraction());

    const rev = r.reversed();
    try expect(rev.contact_point_on1.eql(r.contact_point_on2) and rev.contact_point_on2.eql(r.contact_point_on1));
    try expect(rev.penetration_axis.eql(Vec3.init(0, 1, 0)));
    try std.testing.expectEqual(r.penetration_depth, rev.penetration_depth);
    try expect(rev.sub_shape_id1.eql(r.sub_shape_id2) and rev.sub_shape_id2.eql(r.sub_shape_id1));
    try expect(rev.body_id2.eql(r.body_id2));
    try std.testing.expectEqual(@as(u32, 0), rev.shape1_face.len);
    try std.testing.expectEqual(@as(u32, 1), rev.shape2_face.len);

    const settings: CollideShapeSettings = .{};
    try expect(settings.active_edge_mode == .collide_only_with_active and settings.collect_faces_mode == .no_faces);
    try std.testing.expectEqual(PhysicsSettings.default_collision_tolerance, settings.collision_tolerance);
    try std.testing.expectEqual(PhysicsSettings.default_penetration_tolerance, settings.penetration_tolerance);
    try expect(settings.active_edge_movement_direction.eql(Vec3.zero()));
    try std.testing.expectEqual(@as(f32, 0.0), settings.max_separation_distance);
    try expect(settings.back_face_mode == .ignore_back_faces);
    try std.testing.expectEqual(PhysicsSettings.default_internal_edge_removal_vertex_tolerance_sq, settings.internal_edge_removal_vertex_tolerance_sq);
    const base: CollideSettingsBase = .{};
    try std.testing.expectEqual(base.collision_tolerance, settings.collision_tolerance);
}
