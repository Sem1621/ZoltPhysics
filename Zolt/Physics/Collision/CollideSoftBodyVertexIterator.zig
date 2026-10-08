//! Port of: Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h
//! Status: stub
//!
//! Declared by the shape core (Phase 4, foundation part 2) because `Shape.collideSoftBodyVertices` takes it; the
//! iterator itself is ported with CollideSoftBodyVerticesVsTriangles (Wave A) or SoftBody (Phase 9).

const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const StridedPtrFile = @import("../../Core/StridedPtr.zig");
const StridedPtr = StridedPtrFile.StridedPtr;
const StridedPtrConst = StridedPtrFile.StridedPtrConst;

/// Class that allows iterating over the vertices of a soft body.
/// It tracks the largest penetration and allows storing the resulting collision in a different structure than the soft body vertex itself.
pub const CollideSoftBodyVertexIterator = struct {
    /// Input data
    position: StridedPtrConst(Vec3) = .{},
    inv_mass: StridedPtrConst(f32) = .{},

    /// Output data
    collision_plane: StridedPtr(Plane) = .{},
    largest_penetration: StridedPtr(f32) = .{},
    colliding_shape_index: StridedPtr(i32) = .{},
};
