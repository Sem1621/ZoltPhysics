//! Port of: Jolt/Physics/Collision/Shape/MeshShape.h, Jolt/Physics/Collision/Shape/MeshShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A mesh shape, consisting of triangles.
pub const MeshShape = struct {
    pub const shape_sub_type: ShapeSubType = .mesh;

    /// MeshShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
