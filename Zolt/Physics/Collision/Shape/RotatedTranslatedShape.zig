//! Port of: Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h, Jolt/Physics/Collision/Shape/RotatedTranslatedShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A rotated translated shape will rotate and translate a child shape.
pub const RotatedTranslatedShape = struct {
    pub const shape_sub_type: ShapeSubType = .rotated_translated;

    /// RotatedTranslatedShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
