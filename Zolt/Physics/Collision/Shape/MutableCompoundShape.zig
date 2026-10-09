//! Port of: Jolt/Physics/Collision/Shape/MutableCompoundShape.h, Jolt/Physics/Collision/Shape/MutableCompoundShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A compound shape, sub shapes can be rotated and translated. This shape is optimized for adding / removing and changing the rotation / translation of sub shapes but is less efficient for querying.
pub const MutableCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .mutable_compound;

    /// MutableCompoundShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
