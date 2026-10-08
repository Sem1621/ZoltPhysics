//! Port of: Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h, Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// This shape will shift the center of mass of a child shape, it can e.g. be used to lower the center of mass of an unstable object like a boat to make it stable
pub const OffsetCenterOfMassShape = struct {
    pub const shape_sub_type: ShapeSubType = .offset_center_of_mass;

    /// OffsetCenterOfMassShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
