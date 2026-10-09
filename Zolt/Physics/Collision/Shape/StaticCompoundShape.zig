//! Port of: Jolt/Physics/Collision/Shape/StaticCompoundShape.h, Jolt/Physics/Collision/Shape/StaticCompoundShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A compound shape, sub shapes can be rotated and translated. Sub shapes cannot be modified once the shape is constructed.
pub const StaticCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .static_compound;

    /// StaticCompoundShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
