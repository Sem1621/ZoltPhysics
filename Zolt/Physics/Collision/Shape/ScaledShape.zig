//! Port of: Jolt/Physics/Collision/Shape/ScaledShape.h, Jolt/Physics/Collision/Shape/ScaledShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A shape that scales a child shape in local space of that shape. The scale can be non-uniform and can even turn it inside out when one or three components of the scale are negative.
pub const ScaledShape = struct {
    pub const shape_sub_type: ShapeSubType = .scaled;

    /// ScaledShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
