//! Port of: Jolt/Physics/SoftBody/SoftBodyShape.h, Jolt/Physics/SoftBody/SoftBodyShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 9 (SoftBody)) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("../Collision/Shape/Shape.zig").ShapeSubType;
const Registry = @import("../Collision/CollisionDispatch.zig").Registry;

/// Shape used exclusively for soft bodies. Adds the ability to perform collision checks against soft bodies.
pub const SoftBodyShape = struct {
    pub const shape_sub_type: ShapeSubType = .soft_body;

    /// SoftBodyShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
