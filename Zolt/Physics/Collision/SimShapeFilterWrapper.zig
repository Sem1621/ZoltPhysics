//! Port of: Jolt/Physics/Collision/SimShapeFilterWrapper.h
//! Status: complete
//!
//! `class SimShapeFilterWrapper : private ShapeFilter` embeds its ShapeFilter as `base` (pattern A,
//! CollisionArchitecture D7) and lists the two `ShouldCollide` overrides (`shouldCollidePair` / `shouldCollide`, D1).
//!
//! - Jolt stores `mFinalFilter = inFilter != nullptr? this : &mDefault` in the constructor. That pointer points into the
//!   object itself, so Zolt computes it in `getFilter` instead (neither `mFilter` nor the object's address changes after
//!   construction, so the result is the same) and the wrapper can be returned by value.
//! - The narrow phase writes the body ID of the filter it gets (Jolt: the `mutable mBodyID2` of the returned
//!   `const ShapeFilter &`, through TransformedShape::CastShape in the CCD cast). Rule M: `getFilter` returns a
//!   `*const ShapeFilter` for the queries that only read the filter (CollisionDispatch, `SimCollideBodyVsBody`),
//!   `getFilterMut` (a Zolt addition, the non-const overload) a `*ShapeFilter` for the TransformedShape queries that set
//!   its body ID. `mDefault` (`const ShapeFilter`) is a plain field for the same reason.
//! - The wrapper is only read through `*const ShapeFilter` while the narrow phase runs, so a wrapper per job is
//!   enough (Jolt: a local in ProcessBodyPair, FindCCDContacts and the soft body collision collector).

const std = @import("std");
const Body = @import("../Body/Body.zig").Body;
const Shape = @import("Shape/Shape.zig").Shape;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;
const SimShapeFilter = @import("SimShapeFilter.zig").SimShapeFilter;

/// Helper class to forward ShapeFilter calls to a SimShapeFilter
/// INTERNAL CLASS DO NOT USE!
pub const SimShapeFilterWrapper = struct {
    /// The virtual functions of ShapeFilter this class overrides (C++ `override`, header order)
    pub const overrides = .{ .shouldCollidePair, .shouldCollide };

    /// Private base class ShapeFilter
    base: ShapeFilter = .init(SimShapeFilterWrapper),
    filter: ?*const SimShapeFilter,
    body1: *const Body,
    body2: ?*const Body = null,
    default: ShapeFilter = .{},

    /// Constructor
    pub fn init(filter: ?*const SimShapeFilter, body1: *const Body) SimShapeFilterWrapper {
        // Fall back to an empty filter if no simulation shape filter is set, this reduces the virtual call to 'return true' (see getFilter)
        return .{ .filter = filter, .body1 = body1 };
    }

    /// Forward to the simulation shape filter
    pub fn shouldCollidePair(self: *const SimShapeFilterWrapper, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.filter.?.shouldCollide(self.body1, shape1, sub_shape_id_of_shape1, self.body2.?, shape2, sub_shape_id_of_shape2);
    }

    /// Forward to the simulation shape filter
    pub fn shouldCollide(self: *const SimShapeFilterWrapper, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        return self.filter.?.shouldCollide(self.body1, self.body1.getShape(), .empty, self.body2.?, shape2, sub_shape_id_of_shape2);
    }

    /// Set the body we're colliding against
    pub fn setBody2(self: *SimShapeFilterWrapper, body2: *const Body) void {
        self.body2 = body2;
    }

    /// Returns the actual filter to use for collision detection (Jolt's mFinalFilter)
    pub fn getFilter(self: *const SimShapeFilterWrapper) *const ShapeFilter {
        return if (self.filter != null) &self.base else &self.default;
    }

    /// Returns the actual filter to use for collision detection, for the queries that set its body ID (the TransformedShape
    /// queries, Rule M)
    pub fn getFilterMut(self: *SimShapeFilterWrapper) *ShapeFilter {
        return if (self.filter != null) &self.base else &self.default;
    }
};

test "SimShapeFilterWrapper: forwards to the SimShapeFilter, falls back to the default filter" {
    const testing = std.testing;
    const allocator = testing.allocator;
    const TestShapes = @import("Shape/TestShapes.zig");
    const Vec3 = @import("../../Math/Vec3.zig").Vec3;
    const BodyID = @import("../Body/BodyID.zig").BodyID;

    var box1 = TestShapes.TestBoxShape.init(allocator, Vec3.one(), .{});
    box1.asShape().setEmbedded();
    defer box1.asShapeMut().deinit();
    var box2 = TestShapes.TestBoxShape.init(allocator, Vec3.replicate(2.0), .{});
    box2.asShape().setEmbedded();
    defer box2.asShapeMut().deinit();

    var body1: Body = .{ .id = .init(1) };
    body1.shape = .init(box1.asShape());
    defer body1.shape.deinit();
    var body2: Body = .{ .id = .init(2) };
    body2.shape = .init(box2.asShape());
    defer body2.shape.deinit();

    // Records the arguments of the last call (behind a pointer: the filter is const, Rule M)
    const Call = struct {
        body1: ?*const Body = null,
        shape1: ?*const Shape = null,
        sub_shape_id1: SubShapeID = .empty,
        body2: ?*const Body = null,
        shape2: ?*const Shape = null,
        sub_shape_id2: SubShapeID = .empty,
        count: u32 = 0,
    };
    const MySimShapeFilter = struct {
        pub const overrides = .{.shouldCollide};

        base: SimShapeFilter = .init(@This()),
        call: *Call,

        pub fn shouldCollide(self: *const @This(), b1: *const Body, s1: *const Shape, id1: SubShapeID, b2: *const Body, s2: *const Shape, id2: SubShapeID) bool {
            self.call.* = .{ .body1 = b1, .shape1 = s1, .sub_shape_id1 = id1, .body2 = b2, .shape2 = s2, .sub_shape_id2 = id2, .count = self.call.count + 1 };
            return id2.getValue() != 7;
        }
    };

    // Without a simulation shape filter the default filter is used (accepts everything, never calls a filter)
    {
        var wrapper: SimShapeFilterWrapper = .init(null, &body1);
        wrapper.setBody2(&body2);
        const filter = wrapper.getFilter();
        try testing.expect(filter == &wrapper.default);
        try testing.expect(wrapper.getFilterMut() == &wrapper.default);
        try testing.expect(filter.shouldCollide(box2.asShape(), .{ .value = 7 }));
        try testing.expect(filter.shouldCollidePair(box1.asShape(), .empty, box2.asShape(), .{ .value = 7 }));

        // The narrow phase sets the body ID through the mutable filter
        wrapper.getFilterMut().body_id2 = BodyID.init(2);
        try testing.expect(wrapper.default.body_id2.eql(BodyID.init(2)));
    }

    // With a simulation shape filter both overloads forward to it
    {
        var call: Call = .{};
        const sim_filter: MySimShapeFilter = .{ .call = &call };
        var wrapper: SimShapeFilterWrapper = .init(&sim_filter.base, &body1);
        wrapper.setBody2(&body2);
        const filter = wrapper.getFilter();
        try testing.expect(filter == &wrapper.base);
        try testing.expect(wrapper.getFilterMut() == &wrapper.base);

        // ShouldCollide(shape2, id2): shape 1 is the shape of body 1 with an empty sub shape ID
        try testing.expect(filter.shouldCollide(box2.asShape(), .{ .value = 3 }));
        try testing.expectEqual(@as(u32, 1), call.count);
        try testing.expect(call.body1 == &body1 and call.body2 == &body2);
        try testing.expect(call.shape1 == box1.asShape() and call.shape2 == box2.asShape());
        try testing.expect(call.sub_shape_id1.isEmpty());
        try testing.expectEqual(@as(u32, 3), call.sub_shape_id2.getValue());
        try testing.expect(!filter.shouldCollide(box2.asShape(), .{ .value = 7 }));

        // ShouldCollide(shape1, id1, shape2, id2)
        try testing.expect(filter.shouldCollidePair(box2.asShape(), .{ .value = 5 }, box1.asShape(), .{ .value = 6 }));
        try testing.expectEqual(@as(u32, 3), call.count);
        try testing.expect(call.shape1 == box2.asShape() and call.shape2 == box1.asShape());
        try testing.expectEqual(@as(u32, 5), call.sub_shape_id1.getValue());
        try testing.expectEqual(@as(u32, 6), call.sub_shape_id2.getValue());
        try testing.expect(!filter.shouldCollidePair(box1.asShape(), .empty, box2.asShape(), .{ .value = 7 }));

        // A different body 2
        wrapper.setBody2(&body1);
        _ = wrapper.getFilter().shouldCollide(box1.asShape(), .empty);
        try testing.expect(call.body2 == &body1);
    }
}
