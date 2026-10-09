//! Port of: Jolt/Physics/Collision/Shape/TriangleShape.h, Jolt/Physics/Collision/Shape/TriangleShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2) so that RegisterTypes.zig can list every shape of Jolt's
//! registration order; the porter of this shape (Phase 4) replaces the stub and ports sRegister into `register`.

const ShapeSubType = @import("Shape.zig").ShapeSubType;
const Registry = @import("../CollisionDispatch.zig").Registry;

/// A single triangle, not the most efficient way of creating a world filled with triangles but can be used as a query shape for example.
pub const TriangleShape = struct {
    pub const shape_sub_type: ShapeSubType = .triangle;

    /// TriangleShape::sRegister (not ported yet)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};
