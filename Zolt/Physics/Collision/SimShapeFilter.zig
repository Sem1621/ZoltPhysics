//! Port of: Jolt/Physics/Collision/SimShapeFilter.h
//! Status: complete
//!
//! Pattern A (Docs/Zolt/CollisionArchitecture.md D7): `SimShapeFilter{}` is the base class that accepts everything, a
//! user filter embeds `base: SimShapeFilter = .init(@This())` and lists `pub const overrides = .{.shouldCollide}`.
//! The filter is called from multiple threads with a `*const SimShapeFilter`: mutable state of a user filter must
//! live behind a pointer field (Rule M) and be thread safe.
//!
//! `virtual ~SimShapeFilter()`: filters are never destroyed through a base pointer in Jolt, so there is no destructor
//! entry.

const std = @import("std");
const virtual = @import("../../Core/Virtual.zig");
const Body = @import("../Body/Body.zig").Body;
const Shape = @import("Shape/Shape.zig").Shape;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;

/// Filter class used during the simulation (PhysicsSystem::Update) to filter out collisions at shape level
pub const SimShapeFilter = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Filter function to determine if two shapes should collide. Returns true if the filter passes.
        /// This overload is called during the simulation (PhysicsSystem::Update) and must be registered with PhysicsSystem::SetSimShapeFilter.
        /// It is called at each level of the shape hierarchy, so if you have a compound shape with a box, this function will be called twice.
        /// It will not be called on triangles that are part of another shape, i.e a mesh shape will not trigger a callback per triangle.
        /// Note that this function is called from multiple threads and must be thread safe. All properties are read only.
        /// @param body1 1st body that is colliding
        /// @param shape1 1st shape that is colliding
        /// @param sub_shape_id_of_shape1 The sub shape ID that will lead from body1.GetShape() to shape1
        /// @param body2 2nd body that is colliding
        /// @param shape2 2nd shape that is colliding
        /// @param sub_shape_id_of_shape2 The sub shape ID that will lead from body2.GetShape() to shape2
        shouldCollide: *const fn (self: *const SimShapeFilter, body1: *const Body, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool,
    };

    vtable: *const VTable = vtableFor(SimShapeFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) SimShapeFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Filter function to determine if two shapes should collide. Returns true if the filter passes.
    /// This overload is called during the simulation (PhysicsSystem::Update) and must be registered with PhysicsSystem::SetSimShapeFilter.
    /// It is called at each level of the shape hierarchy, so if you have a compound shape with a box, this function will be called twice.
    /// It will not be called on triangles that are part of another shape, i.e a mesh shape will not trigger a callback per triangle.
    /// Note that this function is called from multiple threads and must be thread safe. All properties are read only.
    pub fn shouldCollide(self: *const SimShapeFilter, body1: *const Body, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.vtable.shouldCollide(self, body1, shape1, sub_shape_id_of_shape1, body2, shape2, sub_shape_id_of_shape2);
    }

    /// Implementations of the virtual functions in SimShapeFilter: accept everything
    pub const impl = struct {
        pub fn shouldCollide(self: *const SimShapeFilter, body1: *const Body, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, body1, shape1, sub_shape_id_of_shape1, body2, shape2, sub_shape_id_of_shape2 };
            return true;
        }
    };
};
