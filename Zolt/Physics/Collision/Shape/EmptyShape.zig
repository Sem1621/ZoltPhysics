//! Port of: Jolt/Physics/Collision/Shape/EmptyShape.h, Jolt/Physics/Collision/Shape/EmptyShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// An empty shape that has no volume and collides with nothing.
pub const EmptyShape = struct {
    pub const shape_sub_type: ShapeSubType = .empty;

    /// EmptyShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
