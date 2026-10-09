//! Port of: Jolt/Physics/Collision/Shape/CompoundShape.h, Jolt/Physics/Collision/Shape/CompoundShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeType = @import("Shape.zig").ShapeType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Base class for a compound shape
pub const CompoundShape = struct {
    pub const shape_type: ShapeType = .compound;

    /// CompoundShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
