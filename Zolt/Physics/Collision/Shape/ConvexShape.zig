//! Port of: Jolt/Physics/Collision/Shape/ConvexShape.h, Jolt/Physics/Collision/Shape/ConvexShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeType = @import("Shape.zig").ShapeType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// Base class for all convex shapes. Defines a virtual interface.
pub const ConvexShape = struct {
    pub const shape_type: ShapeType = .convex;

    /// ConvexShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
