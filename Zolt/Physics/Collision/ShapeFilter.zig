//! Port of: Jolt/Physics/Collision/ShapeFilter.h
//! Status: complete
//!
//! Pattern A with the data in the base (`body_id2`), Docs/Zolt/CollisionArchitecture.md D7. `ShapeFilter{}` (or
//! `&.{}`) is Jolt's default "accept everything" filter (`const ShapeFilter &inShapeFilter = { }`): `vtable`
//! defaults to the base class table. A derived filter embeds `base: ShapeFilter = .init(@This())` and lists its
//! `overrides`.
//!
//! Rule M: Jolt's `mutable BodyID mBodyID2` is written through a `const ShapeFilter &`. Zig marks `*const` parameters
//! `readonly` for LLVM, so such a write would be undefined behavior (ReleaseFast drops it). Therefore:
//! - shape queries and CollisionDispatch only read the filter and take `*const ShapeFilter`;
//! - the entry points that write `body_id2` (TransformedShape, NarrowPhaseQuery, the CharacterVirtual queries) take
//!   `opts: struct { shape_filter: ?*ShapeFilter = null }` and use a local `var default_filter: ShapeFilter = .{}`
//!   when none is given (the C++ `= { }` temporary);
//! - a user filter that needs mutable state (C++ `mutable` members) keeps it behind a pointer field.
//!
//! `virtual ~ShapeFilter()`: filters are never destroyed through a base pointer in Jolt, so there is no destructor
//! entry. The two `ShouldCollide` overloads are `shouldCollide` and `shouldCollidePair` (D1).

const std = @import("std");
const virtual = @import("../../Core/Virtual.zig");
const BodyID = @import("../Body/BodyID.zig").BodyID;
const Shape = @import("Shape/Shape.zig").Shape;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;

/// Filter class
pub const ShapeFilter = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Filter function to determine if we should collide with a shape. Returns true if the filter passes.
        /// This overload is called when the query doesn't have a source shape (e.g. ray cast / collide point)
        /// @param shape2 Shape we're colliding against
        /// @param sub_shape_id_of_shape2 The sub shape ID that will lead from the root shape to shape2 (i.e. the shape of body_id2)
        shouldCollide: *const fn (self: *const ShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool,

        /// Filter function to determine if two shapes should collide. Returns true if the filter passes.
        /// This overload is called when querying a shape vs a shape (e.g. collide object / cast object).
        /// It is called at each level of the shape hierarchy, so if you have a compound shape with a box, this function will be called twice.
        /// It will not be called on triangles that are part of another shape, i.e a mesh shape will not trigger a callback per triangle. You can filter out individual triangles in the CollisionCollector::AddHit function by their sub shape ID.
        /// @param shape1 1st shape that is colliding
        /// @param sub_shape_id_of_shape1 The sub shape ID that will lead from the root shape to shape1 (i.e. the shape that is used to collide or cast against shape 2)
        /// @param shape2 2nd shape that is colliding
        /// @param sub_shape_id_of_shape2 The sub shape ID that will lead from the root shape to shape2 (i.e. the shape of body_id2)
        shouldCollidePair: *const fn (self: *const ShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool,
    };

    vtable: *const VTable = vtableFor(ShapeFilter),

    /// Used during NarrowPhase queries and TransformedShape queries. Set to the body ID of shape2 before calling ShouldCollide.
    /// Provides context to the filter to indicate which body is colliding.
    body_id2: BodyID = .invalid,

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) ShapeFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Filter function to determine if we should collide with a shape. Returns true if the filter passes.
    /// This overload is called when the query doesn't have a source shape (e.g. ray cast / collide point)
    pub fn shouldCollide(self: *const ShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.vtable.shouldCollide(self, shape2, sub_shape_id_of_shape2);
    }

    /// Filter function to determine if two shapes should collide. Returns true if the filter passes.
    /// This overload is called when querying a shape vs a shape (e.g. collide object / cast object).
    pub fn shouldCollidePair(self: *const ShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.vtable.shouldCollidePair(self, shape1, sub_shape_id_of_shape1, shape2, sub_shape_id_of_shape2);
    }

    /// Implementations of the virtual functions in ShapeFilter: accept everything
    pub const impl = struct {
        pub fn shouldCollide(self: *const ShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, shape2, sub_shape_id_of_shape2 };
            return true;
        }

        pub fn shouldCollidePair(self: *const ShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, shape1, sub_shape_id_of_shape1, shape2, sub_shape_id_of_shape2 };
            return true;
        }
    };
};

/// Helper class to reverse the order of the shapes in the ShouldCollide function
pub const ReversedShapeFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter,
    filter: *const ShapeFilter,

    /// Constructor
    pub fn init(filter: *const ShapeFilter) ReversedShapeFilter {
        var self: ReversedShapeFilter = .{ .base = .init(ReversedShapeFilter), .filter = filter };
        self.base.body_id2 = filter.body_id2;
        return self;
    }

    pub fn shouldCollide(self: *const ReversedShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.filter.shouldCollide(shape2, sub_shape_id_of_shape2);
    }

    pub fn shouldCollidePair(self: *const ReversedShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.filter.shouldCollidePair(shape2, sub_shape_id_of_shape2, shape1, sub_shape_id_of_shape1);
    }
};

test "ShapeFilter: default accepts everything, derived filters, ReversedShapeFilter" {
    const testing = std.testing;
    const allocator = testing.allocator;
    const TestShapes = @import("Shape/TestShapes.zig");
    const Vec3 = @import("../../Math/Vec3.zig").Vec3;

    var box = TestShapes.TestBoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var sphere = TestShapes.TestSphereShape.init(allocator, 1.0);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    const id1: SubShapeID = .{ .value = 1 };
    const id2: SubShapeID = .{ .value = 2 };

    // The base class (Jolt's `{ }` default argument)
    const default_filter: ShapeFilter = .{};
    try testing.expect(default_filter.shouldCollide(box.asShape(), id1));
    try testing.expect(default_filter.shouldCollidePair(box.asShape(), id1, sphere.asShape(), id2));
    try testing.expect(default_filter.body_id2.isInvalid());

    // A filter that only accepts pairs where shape 1 is the box with ID 1 (and remembers the last pair, Rule M: through a pointer)
    const PairFilter = struct {
        pub const overrides = .{.shouldCollidePair};

        base: ShapeFilter = .init(@This()),
        box: *const Shape,
        last_id2: *SubShapeID,

        pub fn shouldCollidePair(self: *const @This(), shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = shape2;
            self.last_id2.* = sub_shape_id_of_shape2;
            return shape1 == self.box and sub_shape_id_of_shape1.getValue() == 1;
        }
    };
    var last_id2: SubShapeID = .empty;
    var filter: PairFilter = .{ .box = box.asShape(), .last_id2 = &last_id2 };
    filter.base.body_id2 = .init(12);
    try testing.expect(filter.base.shouldCollide(box.asShape(), id1)); // Not overridden: the base class accepts
    try testing.expect(filter.base.shouldCollidePair(box.asShape(), id1, sphere.asShape(), id2));
    try testing.expect(!filter.base.shouldCollidePair(sphere.asShape(), id2, box.asShape(), id1));

    // The reversed filter swaps shape 1 and shape 2 and copies body_id2
    const reversed: ReversedShapeFilter = .init(&filter.base);
    try testing.expect(reversed.base.body_id2.eql(.init(12)));
    try testing.expect(reversed.base.shouldCollidePair(sphere.asShape(), id2, box.asShape(), id1));
    try testing.expect(last_id2.eql(id2));
    try testing.expect(!reversed.base.shouldCollidePair(box.asShape(), id1, sphere.asShape(), id2));
    try testing.expect(reversed.base.shouldCollide(box.asShape(), id1)); // Forwarded to the base class version
}
