//! Port of: Jolt/Physics/Collision/Shape/DecoratedShape.h, Jolt/Physics/Collision/Shape/DecoratedShape.cpp
//! Status: stub
//!
//! Created by the shape core (foundation part 2); the porter of this shape (Phase 4) replaces the stub.
//! DecoratedShape has no sRegister in Jolt, so it is not part of RegisterTypes.zig's registration order.

const ShapeType = @import("Shape.zig").ShapeType;

/// Base class for shapes that decorate another shape with extra functionality (e.g. scale, translation etc.)
pub const DecoratedShape = struct {
    pub const shape_type: ShapeType = .decorated;
};
