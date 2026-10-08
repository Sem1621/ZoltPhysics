//! Port of: Jolt/Physics/Collision/CollisionDispatch.h, Jolt/Physics/Collision/CollisionDispatch.cpp
//! Status: complete
//!
//! D4 (Docs/Zolt/CollisionArchitecture.md): Jolt's static tables `sCollideShape` / `sCastShape` (filled by
//! RegisterTypes() at startup) and `ShapeFunctions::sRegistry` are one immutable `Registry`, built at compile time by
//! RegisterTypes.zig: `Registry.build` runs `init()` (sInit) and then every shape's `register` (a line by line port of
//! its sRegister) in the order of Jolt/RegisterTypes.cpp, so later registrations override earlier ones exactly like
//! in Jolt. There is no mutable global, no initialization call and nothing to synchronize. The dispatch functions keep
//! Jolt's signatures (`void`, no registry parameter) and read `RegisterTypes.registry`.
//!
//! - `sInit` only fills empty entries in Jolt (registrations made before RegisterTypes() survive it); in Zolt the
//!   tables start empty, so `init` fills every entry with `collideUnsupported` / `castUnsupported` (Jolt's lambdas that
//!   assert "Unsupported shape pair"). User registrations (module `zolt_user_types`) run after Jolt's.
//! - `sRegisterCollideShape` / `sRegisterCastShape` / `ShapeFunctions::sGet` (during registration) are methods of the
//!   registry under construction: `r.registerCollideShape(...)`, `r.registerCastShape(...)`, `r.shapeFunctions(...)`.
//! - JPH_TRACK_NARROWPHASE_STATS: see NarrowPhaseStats.zig (`track_narrowphase_stats`, off).

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const num_sub_shape_types = ShapeFile.num_sub_shape_types;
const SubShapeIDCreator = @import("Shape/SubShapeID.zig").SubShapeIDCreator;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const ShapeCastFile = @import("ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeFilterFile = @import("ShapeFilter.zig");
const ShapeFilter = ShapeFilterFile.ShapeFilter;
const ReversedShapeFilter = ShapeFilterFile.ReversedShapeFilter;
const NarrowPhaseStats = @import("NarrowPhaseStats.zig");
const NarrowPhaseStat = NarrowPhaseStats.NarrowPhaseStat;
const TrackNarrowPhaseStat = NarrowPhaseStats.TrackNarrowPhaseStat;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// Function that collides 2 shapes (see collideShapeVsShape)
pub const CollideShape = *const fn (shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void;

/// Function that casts a shape vs another shape (see castShapeVsShapeLocalSpace)
pub const CastShape = *const fn (shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void;

/// The collision function tables (CollisionDispatch::sCollideShape / sCastShape) and the ShapeFunctions
/// (ShapeFunctions::sRegistry), built at compile time (see RegisterTypes.zig)
pub const Registry = struct {
    collide_shape: [num_sub_shape_types][num_sub_shape_types]CollideShape,
    cast_shape: [num_sub_shape_types][num_sub_shape_types]CastShape,
    shape_functions: [num_sub_shape_types]ShapeFunctions,

    /// Initialize all collision functions with a function that asserts and returns no collision (sInit)
    pub fn init() Registry {
        var r: Registry = undefined;
        for (&r.collide_shape) |*row| @memset(row, &collideUnsupported);
        for (&r.cast_shape) |*row| @memset(row, &castUnsupported);
        @memset(&r.shape_functions, .{});
        return r;
    }

    /// Run sInit and then the registrations in order (every element of `registrations` is a type with
    /// `pub fn register(comptime r: *Registry) void`, the port of its sRegister) and return the final tables
    pub fn build(comptime registrations: anytype) Registry {
        comptime {
            @setEvalBranchQuota(1_000_000);
            var r = init();
            for (registrations) |T| T.register(&r);
            return r;
        }
    }

    /// Register a collide shape function in the collision table (sRegisterCollideShape)
    pub fn registerCollideShape(self: *Registry, type1: ShapeSubType, type2: ShapeSubType, function: CollideShape) void {
        self.collide_shape[@intFromEnum(type1)][@intFromEnum(type2)] = function;
    }

    /// Register a cast shape function in the collision table (sRegisterCastShape)
    pub fn registerCastShape(self: *Registry, type1: ShapeSubType, type2: ShapeSubType, function: CastShape) void {
        self.cast_shape[@intFromEnum(type1)][@intFromEnum(type2)] = function;
    }

    /// Get an entry in the ShapeFunctions registry for a particular sub type (ShapeFunctions::sGet during registration)
    pub fn shapeFunctions(self: *Registry, sub_type: ShapeSubType) *ShapeFunctions {
        return &self.shape_functions[@intFromEnum(sub_type)];
    }

    /// The registered collide function (sCollideShape[type1][type2])
    pub fn getCollideShape(self: *const Registry, type1: ShapeSubType, type2: ShapeSubType) CollideShape {
        return self.collide_shape[@intFromEnum(type1)][@intFromEnum(type2)];
    }

    /// The registered cast function (sCastShape[type1][type2])
    pub fn getCastShape(self: *const Registry, type1: ShapeSubType, type2: ShapeSubType) CastShape {
        return self.cast_shape[@intFromEnum(type1)][@intFromEnum(type2)];
    }
};

/// The function that sInit installs in the collide table: asserts and returns no collision
pub fn collideUnsupported(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    _ = .{ shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter };
    if (Core.enable_asserts) @panic("Unsupported shape pair");
}

/// The function that sInit installs in the cast table: asserts and returns no collision
pub fn castUnsupported(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    _ = .{ shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector };
    if (Core.enable_asserts) @panic("Unsupported shape pair");
}

/// Collide 2 shapes and pass any collision on to collector
/// @param shape1 The first shape
/// @param shape2 The second shape
/// @param scale1 Local space scale of shape 1 (scales relative to its center of mass)
/// @param scale2 Local space scale of shape 2 (scales relative to its center of mass)
/// @param center_of_mass_transform1 Transform to transform center of mass of shape 1 into world space
/// @param center_of_mass_transform2 Transform to transform center of mass of shape 2 into world space
/// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for shape 1
/// @param sub_shape_id_creator2 Class that tracks the current sub shape ID for shape 2
/// @param collide_shape_settings Options for the CollideShape test
/// @param collector The collector that receives the results.
/// @param shape_filter allows selectively disabling collisions between pairs of (sub) shapes (Jolt's default `{ }` is `&.{}`).
pub fn collideShapeVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    var track: if (track_narrowphase_stats) TrackNarrowPhaseStat else void = undefined;
    if (track_narrowphase_stats) track.init(&NarrowPhaseStat.collide_shape[@intFromEnum(shape1.getSubType())][@intFromEnum(shape2.getSubType())]);
    defer if (track_narrowphase_stats) track.deinit();

    // Only test shape if it passes the shape filter
    if (shape_filter.shouldCollidePair(shape1, sub_shape_id_creator1.getID(), shape2, sub_shape_id_creator2.getID()))
        RegisterTypes.registry.getCollideShape(shape1.getSubType(), shape2.getSubType())(shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
}

/// Cast a shape against this shape, passes any hits found to collector.
/// Note: This version takes the shape cast in local space relative to the center of mass of shape, take a look at castShapeVsShapeWorldSpace if you have a shape cast in world space.
/// @param shape_cast_local The shape to cast against the other shape and its start and direction.
/// @param shape_cast_settings Settings for performing the cast
/// @param shape The shape to cast against.
/// @param scale Local space scale for the shape to cast against (scales relative to its center of mass).
/// @param shape_filter allows selectively disabling collisions between pairs of (sub) shapes.
/// @param center_of_mass_transform2 Is the center of mass transform of shape 2 (excluding scale), this is used to provide a transform to the shape cast result so that local hit result quantities can be transformed into world space.
/// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for the casting shape
/// @param sub_shape_id_creator2 Class that tracks the current sub shape ID for the shape we're casting against
/// @param collector The collector that receives the results.
pub fn castShapeVsShapeLocalSpace(shape_cast_local: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    var track: if (track_narrowphase_stats) TrackNarrowPhaseStat else void = undefined;
    if (track_narrowphase_stats) track.init(&NarrowPhaseStat.cast_shape[@intFromEnum(shape_cast_local.shape.getSubType())][@intFromEnum(shape.getSubType())]);
    defer if (track_narrowphase_stats) track.deinit();

    // Only test shape if it passes the shape filter
    if (shape_filter.shouldCollidePair(shape_cast_local.shape, sub_shape_id_creator1.getID(), shape, sub_shape_id_creator2.getID()))
        RegisterTypes.registry.getCastShape(shape_cast_local.shape.getSubType(), shape.getSubType())(shape_cast_local, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
}

/// See: castShapeVsShapeLocalSpace.
/// The only difference is that the shape cast (shape_cast_world) is provided in world space.
/// Note: A shape cast contains the center of mass start of the shape, if you have the world transform of the shape you probably want to construct it using ShapeCast.fromWorldTransform.
pub fn castShapeVsShapeWorldSpace(shape_cast_world: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    const local_shape_cast = shape_cast_world.postTransformed(center_of_mass_transform2.inversedRotationTranslation());
    castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
}

/// An implementation of CollideShape that swaps shape1 and shape2 and swaps the result back, can be registered if the collision function only exists the other way around
pub fn reversedCollideShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    // A collision collector that flips the collision results
    const ReversedCollector = struct {
        pub const overrides = .{.addHit};

        base: CollideShapeCollector,
        collector: *CollideShapeCollector,

        /// Constructor
        fn init(c: *CollideShapeCollector) @This() {
            return .{ .base = .initFrom(@This(), c), .collector = c };
        }

        pub fn addHit(self: *@This(), result: *const CollideShapeResult) void {
            // Add the reversed hit
            const reversed = result.reversed();
            self.collector.addHit(&reversed);

            // If our chained collector updated its early out fraction, we need to follow
            self.base.updateEarlyOutFraction(self.collector.getEarlyOutFraction());
        }
    };

    const reversed_filter: ReversedShapeFilter = .init(shape_filter);
    var reversed_collector: ReversedCollector = .init(collector);
    collideShapeVsShape(shape2, shape1, scale2, scale1, center_of_mass_transform2, center_of_mass_transform1, sub_shape_id_creator2, sub_shape_id_creator1, collide_shape_settings, &reversed_collector.base, &reversed_filter.base);
}

/// An implementation of CastShape that swaps shape1 and shape2 and swaps the result back, can be registered if the collision function only exists the other way around
pub fn reversedCastShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    // A collision collector that flips the collision results
    const ReversedCollector = struct {
        pub const overrides = .{.addHit};

        base: CastShapeCollector,
        collector: *CastShapeCollector,
        world_direction: Vec3,

        /// Constructor
        fn init(c: *CastShapeCollector, world_direction: Vec3) @This() {
            return .{ .base = .initFrom(@This(), c), .collector = c, .world_direction = world_direction };
        }

        pub fn addHit(self: *@This(), result: *const ShapeCastResult) void {
            // Add the reversed hit
            const reversed = result.reversed(self.world_direction);
            self.collector.addHit(&reversed);

            // If our chained collector updated its early out fraction, we need to follow
            self.base.updateEarlyOutFraction(self.collector.getEarlyOutFraction());
        }
    };

    // Reverse the shape cast (shape cast is in local space to shape 2)
    const com_start_inv = shape_cast.center_of_mass_start.inversedRotationTranslation();
    const local_shape_cast: ShapeCast = .init(shape, scale, com_start_inv, com_start_inv.multiply3x3(shape_cast.direction).negate());

    // Calculate the center of mass of shape 1 at start of sweep
    const shape1_com = center_of_mass_transform2.mul(shape_cast.center_of_mass_start);

    // Calculate the world space direction vector of the shape cast
    const world_direction = center_of_mass_transform2.multiply3x3(shape_cast.direction).negate();

    // Forward the cast
    const reversed_filter: ReversedShapeFilter = .init(shape_filter);
    var reversed_collector: ReversedCollector = .init(collector, world_direction);
    castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape_cast.shape, shape_cast.scale, &reversed_filter.base, shape1_com, sub_shape_id_creator2, sub_shape_id_creator1, &reversed_collector.base);
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the test shapes of Shape/TestShapes.zig are registered as User1..User3 through zolt_user_types in the inline tests)

const testing = std.testing;
const Allocator = std.mem.Allocator;
const Color = @import("../../Core/Color.zig").Color;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const Quat = @import("../../Math/Quat.zig").Quat;
const math = @import("../../Math/Math.zig");
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const TestShapes = @import("Shape/TestShapes.zig");
const CollisionCollectorImpl = @import("CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;

/// Records the calls of the pair filter and rejects one pair of sub shape IDs (state behind pointers, Rule M)
const RecordingPairFilter = struct {
    pub const overrides = .{.shouldCollidePair};

    pub const Call = struct { shape1: *const Shape, id1: SubShapeID, shape2: *const Shape, id2: SubShapeID };

    base: ShapeFilter = .init(@This()),
    calls: *std.ArrayList(Call),
    allocator: Allocator,
    reject_id2: ?SubShapeID = null,

    pub fn shouldCollidePair(self: *const RecordingPairFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.calls.append(self.allocator, .{ .shape1 = shape1, .id1 = sub_shape_id_of_shape1, .shape2 = shape2, .id2 = sub_shape_id_of_shape2 }) catch @panic("OOM");
        if (self.reject_id2) |r| return !sub_shape_id_of_shape2.eql(r);
        return true;
    }
};

test "CollisionDispatch: Registry init, build order, overrides and the registered tables" {
    const expect = testing.expect;

    // sInit: every entry is the asserting function, no shape functions
    const empty = comptime Registry.init();
    for (all_types) |t1| {
        for (all_types) |t2| {
            try expect(empty.getCollideShape(t1, t2) == &collideUnsupported);
            try expect(empty.getCastShape(t1, t2) == &castUnsupported);
        }
        try expect(empty.shape_functions[@intFromEnum(t1)].construct == null);
        try expect(empty.shape_functions[@intFromEnum(t1)].color.eql(Color.black));
    }

    // Later registrations override earlier ones
    const First = struct {
        pub fn register(comptime r: *Registry) void {
            r.registerCollideShape(.user4, .user5, reversedCollideShape);
            r.registerCastShape(.user4, .user5, reversedCastShape);
            r.shapeFunctions(.user4).color = Color.red;
        }
    };
    const Second = struct {
        pub fn register(comptime r: *Registry) void {
            r.registerCollideShape(.user4, .user5, TestShapes.collideBoxVsBox);
            r.shapeFunctions(.user4).color = Color.blue;
        }
    };
    const ordered = comptime Registry.build(.{ First, Second });
    try expect(ordered.getCollideShape(.user4, .user5) == &TestShapes.collideBoxVsBox);
    try expect(ordered.getCastShape(.user4, .user5) == &reversedCastShape);
    try expect(ordered.shape_functions[@intFromEnum(ShapeSubType.user4)].color.eql(Color.blue));
    const reverse_order = comptime Registry.build(.{ Second, First });
    try expect(reverse_order.getCollideShape(.user4, .user5) == &reversedCollideShape);

    // The registry of the inline tests: Jolt's registrations (stubs register nothing yet) + the test shapes
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.user1, .user1) == &TestShapes.collideBoxVsBox);
    try expect(r.getCollideShape(.user2, .user1) == &TestShapes.collideSphereVsBox);
    try expect(r.getCollideShape(.user1, .user2) == &reversedCollideShape);
    try expect(r.getCastShape(.user2, .user1) == &TestShapes.castSphereVsBox);
    try expect(r.getCastShape(.user1, .user2) == &reversedCastShape);
    try expect(r.getCollideShape(.user3, .user1) == &TestShapes.collideCompoundVsShape);
    try expect(r.getCollideShape(.user1, .user3) == &TestShapes.collideShapeVsCompound);
    try expect(r.getCollideShape(.user3, .user3) == &TestShapes.collideShapeVsCompound); // Within one loop iteration the second write wins
    try expect(r.getCollideShape(.user2, .user2) == &collideUnsupported);
    try expect(r.getCastShape(.user1, .user1) == &castUnsupported);
    try expect(ShapeFunctions.get(.user3).color.eql(Color.orange));
}

const all_types = ShapeFile.all_sub_shape_types;

test "CollisionDispatch: collide through the table, reversed collisions are the forward results reversed" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = TestShapes.TestSphereShape.init(allocator, 1.0);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = TestShapes.TestBoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    const t_sphere = Mat44.translation(Vec3.init(1.5, 0.25, 0.5));
    const t_box = Mat44.translation(Vec3.init(0, 0, 0.125));
    const creator1 = SubShapeIDCreator.pushID(.{}, 1, 3);
    const creator2 = SubShapeIDCreator.pushID(.{}, 2, 3);

    // Forward: sphere vs box
    var calls: std.ArrayList(RecordingPairFilter.Call) = .empty;
    defer calls.deinit(allocator);
    const filter: RecordingPairFilter = .{ .calls = &calls, .allocator = allocator };
    var forward = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer forward.deinit();
    collideShapeVsShape(sphere.asShape(), box.asShape(), Vec3.one(), Vec3.one(), t_sphere, t_box, creator1, creator2, &settings, &forward.base, &filter.base);
    try expect(forward.hadHit());
    try expect(forward.hit.sub_shape_id1.eql(creator1.getID()) and forward.hit.sub_shape_id2.eql(creator2.getID()));
    try testing.expectEqual(@as(u32, 1), forward.hit.shape1_face.len);
    try testing.expectEqual(@as(u32, 4), forward.hit.shape2_face.len);
    try testing.expectEqual(@as(usize, 1), calls.items.len);
    try expect(calls.items[0].shape1 == sphere.asShape() and calls.items[0].id1.eql(creator1.getID()));

    // Reversed: box vs sphere goes through reversedCollideShape, the filter is called with the shapes swapped back
    calls.clearRetainingCapacity();
    var reversed = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer reversed.deinit();
    collideShapeVsShape(box.asShape(), sphere.asShape(), Vec3.one(), Vec3.one(), t_box, t_sphere, creator2, creator1, &settings, &reversed.base, &filter.base);
    try expect(reversed.hadHit());
    const expected = forward.hit.reversed();
    try expect(reversed.hit.contact_point_on1.eql(expected.contact_point_on1) and reversed.hit.contact_point_on2.eql(expected.contact_point_on2));
    try expect(reversed.hit.penetration_axis.eql(expected.penetration_axis));
    try testing.expectEqual(expected.penetration_depth, reversed.hit.penetration_depth);
    try expect(reversed.hit.sub_shape_id1.eql(creator2.getID()) and reversed.hit.sub_shape_id2.eql(creator1.getID()));
    try expect(reversed.hit.shape1_face.eql(&expected.shape1_face) and reversed.hit.shape2_face.eql(&expected.shape2_face));
    try testing.expectEqual(forward.base.getEarlyOutFraction(), reversed.base.getEarlyOutFraction()); // The wrapper follows the early out fraction
    try testing.expectEqual(@as(usize, 2), calls.items.len); // Box vs sphere (outer call), then sphere vs box through the ReversedShapeFilter
    try expect(calls.items[1].shape1 == box.asShape() and calls.items[1].id1.eql(creator2.getID()));
    try expect(calls.items[1].shape2 == sphere.asShape() and calls.items[1].id2.eql(creator1.getID()));

    // A filter that rejects the pair stops the dispatch
    calls.clearRetainingCapacity();
    const rejecting: RecordingPairFilter = .{ .calls = &calls, .allocator = allocator, .reject_id2 = creator2.getID() };
    var rejected = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer rejected.deinit();
    collideShapeVsShape(sphere.asShape(), box.asShape(), Vec3.one(), Vec3.one(), t_sphere, t_box, creator1, creator2, &settings, &rejected.base, &rejecting.base);
    try expect(!rejected.hadHit());
}

test "CollisionDispatch: casts in world and local space, reversed casts" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = TestShapes.TestSphereShape.init(allocator, 0.5);
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = TestShapes.TestBoxShape.init(allocator, Vec3.one(), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var settings: ShapeCastSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    const t_box = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5 * math.pi), Vec3.init(10, 0, 0));

    // Sphere moving +X into the box (world space cast)
    const cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(6, 0.25, 0)), Vec3.init(4, 0, 0));
    var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer hits.deinit();
    castShapeVsShapeWorldSpace(&cast, &settings, box.asShape(), Vec3.one(), &.{}, t_box, .{}, .{}, &hits.base);
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try expect(@abs(hits.hits.items[0].fraction - 0.625) < 1.0e-5);
    const as_collide_result: *const CollideShapeResult = &hits.hits.items[0].base; // ShapeCastResult passed as its base (D12)
    try expect(as_collide_result.contact_point_on2.isClose(Vec3.init(9, 0.25, 0), .{ .max_dist_sq = 1.0e-8 }));

    // The local space version gets the cast in the space of the box
    var local_hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer local_hits.deinit();
    const local_cast = cast.postTransformed(t_box.inversedRotationTranslation());
    castShapeVsShapeLocalSpace(&local_cast, &settings, box.asShape(), Vec3.one(), &.{}, t_box, .{}, .{}, &local_hits.base);
    try local_hits.checkError();
    try testing.expectEqual(hits.hits.items[0].fraction, local_hits.hits.items[0].fraction);

    // Box moving -X into the sphere: reversedCastShape casts the sphere against the box and reverses the result
    const box_cast = ShapeCast.init(box.asShape(), Vec3.one(), t_box, Vec3.init(-4, 0, 0));
    var reversed = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer reversed.deinit();
    castShapeVsShapeWorldSpace(&box_cast, &settings, sphere.asShape(), Vec3.one(), &.{}, Mat44.translation(Vec3.init(6, 0.25, 0)), .{}, .{}, &reversed.base);
    try reversed.checkError();
    try testing.expectEqual(@as(usize, 1), reversed.hits.items.len);
    const hit = &reversed.hits.items[0];
    try expect(@abs(hit.fraction - 0.625) < 1.0e-5);
    try testing.expectEqual(@as(u32, 4), hit.base.shape1_face.len); // The box face moved to shape 1
    try testing.expectEqual(@as(u32, 0), hit.base.shape2_face.len);
    try expect(hit.base.contact_point_on1.isClose(Vec3.init(9 - 2.5, 0.25, 0), .{ .max_dist_sq = 1.0e-8 })); // Shifted back to the start of the cast
}

test "CollisionDispatch: compound recursion, sub shape IDs and the shape filter at each level" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const sphere = try TestShapes.TestSphereShape.create(allocator, 1.0);
    const box = try TestShapes.TestBoxShape.create(allocator, Vec3.one(), .{});
    const compound = try TestShapes.TestCompoundShape.create(allocator, sphere.asShape(), Vec3.init(-3, 0, 0), box.asShape(), Vec3.init(3, 0, 0));
    var compound_ref = RefConst(Shape).init(compound.asShape());
    defer compound_ref.deinit();
    var query = TestShapes.TestBoxShape.init(allocator, Vec3.replicate(4.5), .{});
    query.asShape().setEmbedded();
    defer query.asShapeMut().deinit();

    // A big box overlaps both children: box vs compound -> box vs sphere (reversed) and box vs box
    var calls: std.ArrayList(RecordingPairFilter.Call) = .empty;
    defer calls.deinit(allocator);
    const filter: RecordingPairFilter = .{ .calls = &calls, .allocator = allocator };
    var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits.deinit();
    collideShapeVsShape(query.asShape(), compound.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &.{}, &hits.base, &filter.base);
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try expect(hits.hits.items[0].sub_shape_id2.eql(SubShapeIDCreator.pushID(.{}, 0, 1).getID()));
    try expect(hits.hits.items[1].sub_shape_id2.eql(SubShapeIDCreator.pushID(.{}, 1, 1).getID()));
    try testing.expectEqual(@as(usize, 4), calls.items.len); // Compound level, sphere (outer + reversed), box

    // Compound vs box: the compound recursion on the other side, rejecting the box child at the second level
    calls.clearRetainingCapacity();
    const rejecting: RecordingPairFilter = .{ .calls = &calls, .allocator = allocator, .reject_id2 = .empty };
    var hits2 = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits2.deinit();
    collideShapeVsShape(compound.asShape(), query.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &.{}, &hits2.base, &rejecting.base);
    try hits2.checkError();
    try testing.expectEqual(@as(usize, 0), hits2.hits.items.len); // Every pair has the empty ID on shape 2
}
