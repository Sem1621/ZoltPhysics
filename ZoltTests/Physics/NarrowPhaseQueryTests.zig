//! Zolt addition, no Jolt file: tests of NarrowPhaseQuery (Jolt has no unit test file for it, its queries are used by
//! the ported CastShapeTests, CollideShapeTests, CollidePointTests, RayShapeTests, ConvexVsTrianglesTest,
//! TransformedShapeTests and MutableCompoundShapeTests)
//! Status: complete
//!
//! - Every query (CastRay with both collector types, CollidePoint, CollideShape, CollideShapeWithInternalEdgeRemoval,
//!   CastShape, CollectTransformedShapes) with each of Jolt's filters (broad phase layer, object layer, body filter
//!   before and after locking the body, shape filter, which sees the body ID set by the query: Rule M), through the
//!   locking and the non locking query, and bodies removed from / added back to the broad phase.
//! - The collector protocol: OnBody / AddHit / OnBodyEnd per body with the context of the body, the early out fraction
//!   of the user's collector stops the broad phase (ForceEarlyOut, the closest hit), the single hit CastRay starts at
//!   the fraction of the hit it is given.
//! - CollideShapeWithInternalEdgeRemoval: the internal edges of a mesh are removed, every allocation of the wrapping
//!   InternalEdgeRemovingCollector and of the user's collector fails in turn.
//! - Multithreaded: readers run every query while the main thread adds, moves and removes bodies through the locking
//!   BodyInterface and updates the physics system (which rebuilds the broad phase trees).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const BroadPhaseLayers = LayersFile.BroadPhaseLayers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyFilter = zolt.BodyFilter;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const CapsuleShape = zolt.CapsuleShape;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const Core = zolt.Core;
const IgnoreSingleBodyFilter = zolt.IgnoreSingleBodyFilter;
const MeshShapeSettings = zolt.MeshShapeSettings;
const NarrowPhaseQuery = zolt.NarrowPhaseQuery;
const Quat = zolt.Quat;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const RefConst = zolt.RefConst;
const RMat44 = zolt.RMat44;
const RRayCast = zolt.RRayCast;
const RShapeCast = zolt.RShapeCast;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const SpecifiedBroadPhaseLayerFilter = zolt.SpecifiedBroadPhaseLayerFilter;
const SpecifiedObjectLayerFilter = zolt.SpecifiedObjectLayerFilter;
const SphereShape = zolt.SphereShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Triangle = zolt.Triangle;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

/// The bodies of the test world, along the X axis
const num_bodies = 5;

/// A world with a body per object layer: a sphere (NON_MOVING), a box (MOVING), a static compound of 2 boxes (MOVING2),
/// a capsule (HQ_DEBRIS, broad phase layer MOVING) and a sphere (LQ_DEBRIS), at x = 0, 3, 6, 9, 12
const World = struct {
    c: PhysicsTestContext,
    ids: [num_bodies]BodyID,

    const positions = [num_bodies]f32{ 0, 3, 6, 9, 12 };
    /// A point inside each body (the center of the compound is between its boxes)
    const inner_points = [num_bodies]f32{ 0, 3, 5.4, 9, 12 };
    const layers = [num_bodies]zolt.ObjectLayer{ Layers.non_moving, Layers.moving, Layers.moving2, Layers.hq_debris, Layers.lq_debris };

    /// In place (the physics system must not move)
    fn init(self: *World) !void {
        try self.c.init(allocator, .{});
        errdefer self.c.deinit();

        var compound = StaticCompoundShapeSettings.init(allocator);
        compound.asShapeSettings().setEmbedded();
        defer compound.deinit();
        {
            var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());
            defer box.deinit();
            try compound.base.addShapePtr(Vec3.init(-0.6, 0, 0), Quat.identity(), box.get(), .{ .user_data = 1 });
            try compound.base.addShapePtr(Vec3.init(0.6, 0, 0), Quat.identity(), box.get(), .{ .user_data = 2 });
        }
        var compound_result = try compound.asShapeSettings().createShape(allocator);
        defer compound_result.deinit();

        var shapes: [num_bodies]RefConst(Shape) = .{
            .init((try SphereShape.create(allocator, 0.5, .{})).asShape()),
            .init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape()),
            .init(compound_result.getPtr().?),
            .init((try CapsuleShape.create(allocator, 0.5, 0.4, .{})).asShape()),
            .init((try SphereShape.create(allocator, 0.5, .{})).asShape()),
        };
        defer for (&shapes) |*s| s.deinit();

        for (&self.ids, 0..) |*id, i| {
            var settings = BodyCreationSettings.initWithShape(shapes[i].get(), RVec3.init(positions[i], 0, 0), Quat.identity(), .static, layers[i]);
            defer settings.deinit();
            settings.user_data = 100 + i;
            id.* = try self.c.getBodyInterface().createAndAddBody(&settings, .dont_activate);
        }
    }

    fn deinit(self: *World) void {
        self.c.deinit();
    }

    fn query(self: *World, locking: bool) *const NarrowPhaseQuery {
        return if (locking) self.c.getSystem().getNarrowPhaseQuery() else self.c.getSystem().getNarrowPhaseQueryNoLock();
    }

    /// The bit of the body in a result mask
    fn bit(self: *const World, id: BodyID) u32 {
        for (self.ids, 0..) |b, i| {
            if (b.eql(id))
                return @as(u32, 1) << @intCast(i);
        }
        return 1 << 31; // Not a body of the world
    }
};

/// A shape filter that rejects every shape of one body: it can only do that when the query sets its body ID
const RejectBodyShapeFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject: BodyID,
    /// The number of calls (behind a pointer: the filter is const in the queries, Rule M)
    num_calls: *u32,

    pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape2, sub_shape_id_of_shape2 };
        self.num_calls.* += 1;
        return !self.base.body_id2.eql(self.reject);
    }

    pub fn shouldCollidePair(self: *const @This(), shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ shape1, sub_shape_id_of_shape1, shape2, sub_shape_id_of_shape2 };
        self.num_calls.* += 1;
        return !self.base.body_id2.eql(self.reject);
    }
};

/// A body filter that rejects a body by its user data once it is locked
const RejectUserDataBodyFilter = struct {
    pub const overrides = .{.shouldCollideLocked};

    base: BodyFilter = .init(@This()),
    reject: u64,

    pub fn shouldCollideLocked(self: *const @This(), body: *const Body) bool {
        return body.getUserData() != self.reject;
    }
};

const QueryKind = enum { ray, ray_collector, point, collide, collide_internal_edge_removal, cast, collect };

const Filters = struct {
    broad_phase_layer_filter: *const zolt.BroadPhaseLayerFilter = &.{},
    object_layer_filter: *const zolt.ObjectLayerFilter = &.{},
    body_filter: *const BodyFilter = &.{},
    shape_filter: ?*ShapeFilter = null,
};

/// Run a query over the whole world and return the mask of the bodies it found
fn runQuery(world: *World, kind: QueryKind, locking: bool, filters: Filters) !u32 {
    const query = world.query(locking);
    const query_filters: NarrowPhaseQuery.QueryFilters = .{ .broad_phase_layer_filter = filters.broad_phase_layer_filter, .object_layer_filter = filters.object_layer_filter, .body_filter = filters.body_filter, .shape_filter = filters.shape_filter };
    var mask: u32 = 0;
    switch (kind) {
        .ray => {
            var hit: RayCastResult = .{};
            if (query.castRay(RRayCast.init(RVec3.init(-3, 0, 0), Vec3.init(18, 0, 0)), &hit, .{ .broad_phase_layer_filter = filters.broad_phase_layer_filter, .object_layer_filter = filters.object_layer_filter, .body_filter = filters.body_filter }))
                mask |= world.bit(hit.body_id);
        },
        .ray_collector => {
            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            const settings: RayCastSettings = .{};
            query.castRayCollector(RRayCast.init(RVec3.init(-3, 0, 0), Vec3.init(18, 0, 0)), &settings, &collector.base, query_filters);
            try collector.checkError();
            for (collector.hits.items) |hit| mask |= world.bit(hit.body_id);
        },
        .point => {
            for (World.inner_points) |x| {
                var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
                defer collector.deinit();
                query.collidePoint(RVec3.init(x + 0.1, 0.1, 0), &collector.base, query_filters);
                try collector.checkError();
                for (collector.hits.items) |hit| mask |= world.bit(hit.body_id);
            }
        },
        .collide, .collide_internal_edge_removal => {
            var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(8, 0.2, 0.2), .{})).asShape());
            defer box.deinit();
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            const settings: CollideShapeSettings = .{};
            if (kind == .collide)
                query.collideShape(box.get().?, Vec3.one(), RMat44.translation(RVec3.init(6, 0, 0)), &settings, RVec3.zero(), &collector.base, query_filters)
            else
                try query.collideShapeWithInternalEdgeRemoval(allocator, box.get().?, Vec3.one(), RMat44.translation(RVec3.init(6, 0, 0)), &settings, RVec3.zero(), &collector.base, query_filters);
            try collector.checkError();
            for (collector.hits.items) |hit| mask |= world.bit(hit.body_id2);
        },
        .cast => {
            var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 0.2, .{})).asShape());
            defer sphere.deinit();
            var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
            defer collector.deinit();
            const settings: ShapeCastSettings = .{};
            const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(-3, 0, 0)), Vec3.init(18, 0, 0));
            query.castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, query_filters);
            try collector.checkError();
            for (collector.hits.items) |hit| mask |= world.bit(hit.base.body_id2);
        },
        .collect => {
            var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
            defer collector.deinit();
            query.collectTransformedShapes(AABox.init(Vec3.init(-2, -2, -2), Vec3.init(14, 2, 2)), &collector.base, .{ .broad_phase_layer_filter = filters.broad_phase_layer_filter, .object_layer_filter = filters.object_layer_filter, .body_filter = filters.body_filter, .shape_filter = if (filters.shape_filter) |f| f else &.{} });
            try collector.checkError();
            for (collector.hits.items) |*hit| mask |= world.bit(hit.body_id);
        },
    }
    return mask;
}

/// Run a query through the locking and the non locking interface (same result) and return the mask
fn runQueryBoth(world: *World, kind: QueryKind, filters: Filters) !u32 {
    const mask = try runQuery(world, kind, true, filters);
    try fw.expectEqual(mask, try runQuery(world, kind, false, filters));
    return mask;
}

test "NarrowPhaseQuery: every query with every filter" {
    var world: World = undefined;
    try world.init();
    defer world.deinit();

    const all: u32 = (1 << num_bodies) - 1;
    for (std.enums.values(QueryKind)) |kind| {
        // The single hit ray cast only reports the closest body (the lowest bit of the mask)
        const expect_mask = struct {
            fn f(k: QueryKind, m: u32) u32 {
                return if (k == .ray) m & (~m +% 1) else m;
            }
        }.f;

        // No filter
        try fw.expectEqual(expect_mask(kind, all), try runQueryBoth(&world, kind, .{}));

        // Broad phase layer filter: MOVING contains the MOVING and the HQ_DEBRIS body
        const bp_filter = SpecifiedBroadPhaseLayerFilter.init(BroadPhaseLayers.moving);
        try fw.expectEqual(expect_mask(kind, 0b01010), try runQueryBoth(&world, kind, .{ .broad_phase_layer_filter = &bp_filter.base }));

        // Object layer filter
        const object_filter = SpecifiedObjectLayerFilter.init(Layers.moving2);
        try fw.expectEqual(expect_mask(kind, 0b00100), try runQueryBoth(&world, kind, .{ .object_layer_filter = &object_filter.base }));
        const object_filter2 = SpecifiedObjectLayerFilter.init(Layers.hq_debris);
        try fw.expectEqual(expect_mask(kind, 0b01000), try runQueryBoth(&world, kind, .{ .broad_phase_layer_filter = &bp_filter.base, .object_layer_filter = &object_filter2.base }));

        // Body filter before locking the body
        const ignore_first = IgnoreSingleBodyFilter.init(world.ids[0]);
        try fw.expectEqual(expect_mask(kind, 0b11110), try runQueryBoth(&world, kind, .{ .body_filter = &ignore_first.base }));

        // Body filter after locking the body
        const reject_user_data: RejectUserDataBodyFilter = .{ .reject = 101 };
        try fw.expectEqual(expect_mask(kind, 0b11101), try runQueryBoth(&world, kind, .{ .body_filter = &reject_user_data.base }));

        // Shape filter: the query sets the body ID of the filter before it calls the filter (CollectTransformedShapes
        // does not, so it cannot reject a body by ID)
        if (kind != .ray) {
            var num_calls: u32 = 0;
            var reject_compound: RejectBodyShapeFilter = .{ .reject = world.ids[2], .num_calls = &num_calls };
            const expected: u32 = if (kind == .collect) all else 0b11011;
            try fw.expectEqual(expected, try runQueryBoth(&world, kind, .{ .shape_filter = &reject_compound.base }));
            try fw.expect(num_calls > 0);
        }
    }

    // CollectTransformedShapes with a shape filter that rejects everything
    {
        const RejectAll = struct {
            pub const overrides = .{.shouldCollide};
            base: ShapeFilter = .init(@This()),
            pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
                _ = .{ self, shape2, sub_shape_id_of_shape2 };
                return false;
            }
        };
        var reject_all: RejectAll = .{};
        try fw.expectEqual(@as(u32, 0), try runQueryBoth(&world, .collect, .{ .shape_filter = &reject_all.base }));
    }

    // A body that is removed from the broad phase is not found, it is found again when it is added back
    const bi = world.c.getBodyInterface();
    bi.removeBody(world.ids[3]);
    for (std.enums.values(QueryKind)) |kind|
        try fw.expectEqual(if (kind == .ray) @as(u32, 0b00001) else 0b10111, try runQueryBoth(&world, kind, .{}));
    try bi.addBody(world.ids[3], .dont_activate);
    for (std.enums.values(QueryKind)) |kind|
        try fw.expectEqual(if (kind == .ray) @as(u32, 0b00001) else all, try runQueryBoth(&world, kind, .{}));

    // A body that moved is found at its new position
    bi.setPosition(world.ids[0], RVec3.init(0, 10, 0), .dont_activate);
    try fw.expectEqual(@as(u32, 0b00010), try runQueryBoth(&world, .ray, .{}));
    try fw.expectEqual(@as(u32, 0b11110), try runQueryBoth(&world, .collide, .{}));

    // The same after the physics update rebuilt the broad phase
    _ = try world.c.simulateSingleStep();
    for (std.enums.values(QueryKind)) |kind|
        try fw.expectEqual(if (kind == .ray) @as(u32, 0b00010) else 0b11110, try runQueryBoth(&world, kind, .{}));
}

test "NarrowPhaseQuery: the single hit CastRay starts at the fraction of the hit" {
    var world: World = undefined;
    try world.init();
    defer world.deinit();
    const query = world.query(true);
    const ray = RRayCast.init(RVec3.init(-3, 0, 0), Vec3.init(18, 0, 0));

    // The closest hit is the sphere at x = 0
    var hit: RayCastResult = .{};
    try fw.expect(query.castRay(ray, &hit, .{}));
    try fw.expect(hit.body_id.eql(world.ids[0]));
    try fw.checkApproxEqual(hit.fraction, 2.5 / 18.0, .{});

    // A hit that is closer than every body stays as it is (Jolt returns whether the fraction is <= 1)
    var close: RayCastResult = .{ .body_id = BodyID.init(77), .fraction = 0.1 };
    try fw.expect(query.castRay(ray, &close, .{}));
    try fw.expect(close.body_id.eql(BodyID.init(77)));
    try fw.expectEqual(@as(f32, 0.1), close.fraction);

    // A hit between the first and the second body only lets the first body through
    var between: RayCastResult = .{ .fraction = 0.25 };
    try fw.expect(query.castRay(ray, &between, .{}));
    try fw.expect(between.body_id.eql(world.ids[0]));

    // Nothing in range: false, the hit is unchanged
    var miss: RayCastResult = .{};
    try fw.expect(!query.castRay(RRayCast.init(RVec3.init(-3, 5, 0), Vec3.init(18, 0, 0)), &miss, .{}));
    try fw.expect(miss.body_id.isInvalid());
}

/// A collector that records the protocol of the narrow phase query: OnBody, AddHit (with the body of the context) and
/// OnBodyEnd, and that optionally stops the query or follows the closest hit
fn LoggingCollector(comptime CollectorType: type) type {
    return struct {
        const Self = @This();
        pub const overrides = .{ .onBody, .onBodyEnd, .addHit };

        pub const Event = union(enum) {
            on_body: BodyID,
            hit: BodyID,
            on_body_end,
        };

        pub const Mode = enum { all, force_early_out, closest };

        base: CollectorType = .init(Self),
        events: [64]Event = undefined,
        num_events: u32 = 0,
        mode: Mode,

        fn add(self: *Self, event: Event) void {
            if (self.num_events < self.events.len) self.events[self.num_events] = event;
            self.num_events += 1;
        }

        pub fn onBody(self: *Self, body: *const Body) void {
            self.add(.{ .on_body = body.getID() });
        }

        pub fn onBodyEnd(self: *Self) void {
            self.add(.on_body_end);
        }

        pub fn addHit(self: *Self, result: *const CollectorType.ResultType) void {
            self.add(.{ .hit = self.base.getContext().?.body_id });
            switch (self.mode) {
                .all => {},
                .force_early_out => self.base.forceEarlyOut(),
                .closest => if (@hasDecl(CollectorType.ResultType, "getEarlyOutFraction")) { // TransformedShape has none (like Jolt's)
                    if (result.getEarlyOutFraction() < self.base.getEarlyOutFraction()) self.base.updateEarlyOutFraction(result.getEarlyOutFraction());
                },
            }
        }

        /// Check the protocol and return the number of bodies
        fn check(self: *const Self) !u32 {
            try fw.expect(self.num_events <= self.events.len);
            var num_bodies_seen: u32 = 0;
            var current: ?BodyID = null;
            for (self.events[0..self.num_events]) |event| {
                switch (event) {
                    .on_body => |id| {
                        try fw.expect(current == null);
                        current = id;
                        num_bodies_seen += 1;
                    },
                    .hit => |id| try fw.expect(current != null and current.?.eql(id)),
                    .on_body_end => {
                        try fw.expect(current != null);
                        current = null;
                    },
                }
            }
            try fw.expect(current == null);
            return num_bodies_seen;
        }

        fn numHits(self: *const Self) u32 {
            var n: u32 = 0;
            for (self.events[0..self.num_events]) |event| n += @intFromBool(event == .hit);
            return n;
        }
    };
}

test "NarrowPhaseQuery: collector protocol and early out" {
    var world: World = undefined;
    try world.init();
    defer world.deinit();
    const query = world.query(true);

    const ray = RRayCast.init(RVec3.init(-3, 0, 0), Vec3.init(18, 0, 0));
    const ray_settings: RayCastSettings = .{};
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 0.2, .{})).asShape());
    defer sphere.deinit();
    const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(-3, 0, 0)), Vec3.init(18, 0, 0));
    const cast_settings: ShapeCastSettings = .{};
    var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(8, 0.2, 0.2), .{})).asShape());
    defer box.deinit();
    const collide_settings: CollideShapeSettings = .{};

    for (std.enums.values(LoggingCollector(CastRayCollector).Mode)) |mode| {
        // Rays: every body, the compound gives a hit per box
        var ray_collector: LoggingCollector(CastRayCollector) = .{ .mode = mode };
        query.castRayCollector(ray, &ray_settings, &ray_collector.base, .{});
        const ray_bodies = try ray_collector.check();

        // Casts
        var cast_collector: LoggingCollector(CastShapeCollector) = .{ .mode = mode };
        query.castShape(&shape_cast, &cast_settings, RVec3.zero(), &cast_collector.base, .{});
        const cast_bodies = try cast_collector.check();

        // Collide shape and collide point (the point is inside the first box of the compound)
        var collide_collector: LoggingCollector(CollideShapeCollector) = .{ .mode = mode };
        query.collideShape(box.get().?, Vec3.one(), RMat44.translation(RVec3.init(6, 0, 0)), &collide_settings, RVec3.zero(), &collide_collector.base, .{});
        const collide_bodies = try collide_collector.check();
        var point_collector: LoggingCollector(CollidePointCollector) = .{ .mode = mode };
        query.collidePoint(RVec3.init(5.5, 0, 0), &point_collector.base, .{});
        try fw.expectEqual(@as(u32, 1), try point_collector.check());

        switch (mode) {
            .all => {
                try fw.expectEqual(@as(u32, num_bodies), ray_bodies);
                try fw.expectEqual(@as(u32, num_bodies + 1), ray_collector.numHits());
                try fw.expectEqual(@as(u32, num_bodies), cast_bodies);
                try fw.expectEqual(@as(u32, num_bodies + 1), cast_collector.numHits());
                try fw.expectEqual(@as(u32, num_bodies), collide_bodies);
                try fw.expectEqual(@as(u32, num_bodies + 1), collide_collector.numHits());
            },
            .force_early_out => {
                // The first hit stops the broad phase
                try fw.expectEqual(@as(u32, 1), ray_bodies);
                try fw.expectEqual(@as(u32, 1), ray_collector.numHits());
                try fw.expectEqual(@as(u32, 1), cast_bodies);
                try fw.expectEqual(@as(u32, 1), cast_collector.numHits());
                try fw.expectEqual(@as(u32, 1), collide_bodies);
                try fw.expectEqual(@as(u32, 1), collide_collector.numHits());
            },
            .closest => {
                // The broad phase visits the closest body first and skips the bodies behind the hit
                try fw.expectEqual(@as(u32, 1), ray_bodies);
                try fw.expect(ray_collector.events[0].on_body.eql(world.ids[0]));
                try fw.expectEqual(@as(u32, 1), cast_bodies);
                try fw.expect(cast_collector.events[0].on_body.eql(world.ids[0]));
                try fw.expectEqual(@as(u32, num_bodies), collide_bodies);
            },
        }
    }

    // CollectTransformedShapes: a transformed shape per leaf shape
    var collect_collector: LoggingCollector(TransformedShapeCollector) = .{ .mode = .all };
    query.collectTransformedShapes(AABox.init(Vec3.init(-2, -2, -2), Vec3.init(14, 2, 2)), &collect_collector.base, .{});
    try fw.expectEqual(@as(u32, num_bodies), try collect_collector.check());
    try fw.expectEqual(@as(u32, num_bodies + 1), collect_collector.numHits());
}

/// A flat grid mesh of n x n cells of size s around the origin
fn createGridMesh(n: u32, s: f32) !*MeshShapeSettings {
    var triangles: std.ArrayList(Triangle) = .empty;
    defer triangles.deinit(allocator);
    const half: f32 = 0.5 * s * @as(f32, @floatFromInt(n));
    for (0..n) |z_index|
        for (0..n) |x_index| {
            const x = s * @as(f32, @floatFromInt(x_index)) - half;
            const z = s * @as(f32, @floatFromInt(z_index)) - half;
            try triangles.append(allocator, Triangle.init(Vec3.init(x, 0, z), Vec3.init(x, 0, z + s), Vec3.init(x + s, 0, z + s), .{}));
            try triangles.append(allocator, Triangle.init(Vec3.init(x, 0, z), Vec3.init(x + s, 0, z + s), Vec3.init(x + s, 0, z), .{}));
        };
    return MeshShapeSettings.create(allocator, triangles.items, .{});
}

test "NarrowPhaseQuery: CollideShapeWithInternalEdgeRemoval removes internal edges, out of memory" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A mesh of 20 x 20 cells: a box that lies on it touches 800 triangles and 441 vertices (more than the local buffers
    // of InternalEdgeRemovingCollector hold: 128 voided features, so it falls back to the heap)
    const mesh = try c.createBodyWithShapeSettings((try createGridMesh(20, 0.5)).asShapeSettings(), RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    // A box that slightly penetrates the mesh and a sphere that rests on a vertex
    var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(5.5, 0.5, 5.5), .{ .convex_radius = 0.0 })).asShape());
    defer box.deinit();
    const box_transform = RMat44.translation(RVec3.init(0, 0.49, 0));

    const Query = struct {
        fn run(a: Allocator, context: *PhysicsTestContext, shape: *const Shape, transform: RMat44, mesh_id: BodyID, num_hits: *usize) !void {
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(a);
            defer collector.deinit();
            const settings: CollideShapeSettings = .{};
            const query_result = context.getSystem().getNarrowPhaseQuery().collideShapeWithInternalEdgeRemoval(a, shape, Vec3.one(), transform, &settings, RVec3.zero(), &collector.base, .{});
            const collector_result = collector.checkError();
            try query_result;
            try collector_result;

            // Every hit is a face hit (the normals of the internal edges and vertices are gone)
            for (collector.hits.items) |hit| {
                if (!hit.body_id2.eql(mesh_id) or !fw.isApproxEqual(hit.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{ .tolerance = 1.0e-4 }))
                    return error.TestUnexpectedResult;
            }
            num_hits.* = collector.hits.items.len;
        }
    };

    var num_hits: usize = 0;
    try Query.run(allocator, &c, box.get().?, box_transform, mesh.getID(), &num_hits);
    try fw.expect(num_hits > 0);

    // Every allocation fails in turn: the error arrives and nothing leaks
    try std.testing.checkAllAllocationFailures(allocator, Query.run, .{ &c, box.get().?, box_transform, mesh.getID(), &num_hits });

    // Without internal edge removal the box hits every triangle it touches, the edges of the triangles give other normals
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        var settings: CollideShapeSettings = .{};
        settings.active_edge_mode = .collide_with_all;
        settings.collect_faces_mode = .collect_faces;
        c.getSystem().getNarrowPhaseQuery().collideShape(box.get().?, Vec3.one(), box_transform, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len >= num_hits);
    }

    // A sphere above a triangle near the edges it shares with 2 other triangles: it touches all 3 (the sphere cuts the
    // plane of the mesh in a circle of radius 0.218 around (0.2, 0.25), the diagonal of the cell is at 0.035, the edge
    // x = 0 at 0.2). The contacts with the 2 neighbors are on internal edges: only the face contact remains.
    {
        var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 0.5, .{})).asShape());
        defer sphere.deinit();
        const sphere_transform = RMat44.translation(RVec3.init(0.2, 0.45, 0.25));
        const settings: CollideShapeSettings = .{};

        var plain = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer plain.deinit();
        var plain_settings: CollideShapeSettings = .{};
        plain_settings.active_edge_mode = .collide_with_all;
        c.getSystem().getNarrowPhaseQuery().collideShape(sphere.get().?, Vec3.one(), sphere_transform, &plain_settings, RVec3.zero(), &plain.base, .{});
        try plain.checkError();
        try fw.expectEqual(@as(usize, 3), plain.hits.items.len);

        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        try c.getSystem().getNarrowPhaseQuery().collideShapeWithInternalEdgeRemoval(allocator, sphere.get().?, Vec3.one(), sphere_transform, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expectEqual(@as(usize, 1), collector.hits.items.len);
        try fw.checkApproxEqual(collector.hits.items[0].penetration_depth, 0.05, .{ .tolerance = 1.0e-5 });
        try fw.checkApproxEqual(collector.hits.items[0].penetration_axis.normalized(), Vec3.init(0, -1, 0), .{ .tolerance = 1.0e-5 });
    }
}

test "NarrowPhaseQuery: multithreaded queries while bodies are added, moved and removed" {
    for (0..3) |round| {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        const bi = c.getBodyInterface();

        // 16 boxes that are always there and 16 spheres between them that come and go
        var fixed: [16]BodyID = undefined;
        var changing: [16]BodyID = undefined;
        {
            var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.4), .{})).asShape());
            defer box.deinit();
            var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 0.3, .{})).asShape());
            defer sphere.deinit();
            for (&fixed, &changing, 0..) |*f, *ch, i| {
                const x: f32 = 2.0 * @as(f32, @floatFromInt(i));
                var settings = BodyCreationSettings.initWithShape(box.get(), RVec3.init(x, 0, 0), Quat.identity(), .static, Layers.non_moving);
                defer settings.deinit();
                f.* = try bi.createAndAddBody(&settings, .dont_activate);
                var settings2 = BodyCreationSettings.initWithShape(sphere.get(), RVec3.init(x + 1, 0, 0), Quat.identity(), .static, Layers.non_moving);
                defer settings2.deinit();
                ch.* = (try bi.createBody(&settings2)).?.getID();
            }
        }

        const Shared = struct {
            c: *PhysicsTestContext,
            fixed: *const [16]BodyID,
            changing: *const [16]BodyID,
            stop: std.atomic.Value(bool) = .init(false),
            failures: std.atomic.Value(u32) = .init(0),
            queries: std.atomic.Value(u32) = .init(0),
            started: std.atomic.Value(u32) = .init(0),

            fn isFixed(s: *const @This(), id: BodyID) bool {
                for (s.fixed) |f| if (f.eql(id)) return true;
                return false;
            }

            fn isKnown(s: *const @This(), id: BodyID) bool {
                if (s.isFixed(id)) return true;
                for (s.changing) |ch| if (ch.eql(id)) return true;
                return false;
            }

            /// Every fixed body exactly once, every other body at most once and only known bodies
            fn checkIDs(s: *const @This(), ids: []const BodyID) bool {
                var num_fixed: u32 = 0;
                for (ids, 0..) |a, i| {
                    if (!s.isKnown(a)) return false;
                    num_fixed += @intFromBool(s.isFixed(a));
                    for (ids[i + 1 ..]) |b| if (a.eql(b)) return false;
                }
                return num_fixed == s.fixed.len;
            }

            fn reader(s: *@This()) void {
                _ = s.started.fetchAdd(1, .release);
                while (!s.stop.load(.acquire)) {
                    const ok = s.runQueries() catch false;
                    if (!ok) _ = s.failures.fetchAdd(1, .monotonic);
                    _ = s.queries.fetchAdd(1, .monotonic);
                }
            }

            /// The queries of a reader (through the locking interface: the main thread modifies the bodies)
            fn runQueries(s: *@This()) !bool {
                const query = s.c.getSystem().getNarrowPhaseQuery();
                var ids: [64]BodyID = undefined;
                var num_ids: usize = 0;

                // A ray along all bodies
                {
                    var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
                    defer collector.deinit();
                    const settings: RayCastSettings = .{};
                    query.castRayCollector(RRayCast.init(RVec3.init(-1, 0, 0), Vec3.init(34, 0, 0)), &settings, &collector.base, .{});
                    try collector.checkError();
                    num_ids = 0;
                    for (collector.hits.items) |hit| {
                        ids[num_ids] = hit.body_id;
                        num_ids += 1;
                    }
                    if (!s.checkIDs(ids[0..num_ids])) return false;
                }

                // The closest hit is always the first fixed box
                {
                    var hit: RayCastResult = .{};
                    if (!query.castRay(RRayCast.init(RVec3.init(-1, 0, 0), Vec3.init(34, 0, 0)), &hit, .{}) or !hit.body_id.eql(s.fixed[0])) return false;
                }

                // A box around all bodies
                {
                    var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(20, 1, 1), .{})).asShape());
                    defer box.deinit();
                    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
                    defer collector.deinit();
                    const settings: CollideShapeSettings = .{};
                    query.collideShape(box.get().?, Vec3.one(), RMat44.translation(RVec3.init(15, 0, 0)), &settings, RVec3.zero(), &collector.base, .{});
                    try collector.checkError();
                    num_ids = 0;
                    for (collector.hits.items) |hit| {
                        ids[num_ids] = hit.body_id2;
                        num_ids += 1;
                    }
                    if (!s.checkIDs(ids[0..num_ids])) return false;

                    // A sphere cast along all bodies
                    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 0.1, .{})).asShape());
                    defer sphere.deinit();
                    var cast_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
                    defer cast_collector.deinit();
                    const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(-1, 0, 0)), Vec3.init(34, 0, 0));
                    const cast_settings: ShapeCastSettings = .{};
                    query.castShape(&shape_cast, &cast_settings, RVec3.zero(), &cast_collector.base, .{});
                    try cast_collector.checkError();
                    num_ids = 0;
                    for (cast_collector.hits.items) |hit| {
                        ids[num_ids] = hit.base.body_id2;
                        num_ids += 1;
                    }
                    if (!s.checkIDs(ids[0..num_ids])) return false;
                }

                // The transformed shapes in a box around all bodies
                {
                    var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
                    defer collector.deinit();
                    query.collectTransformedShapes(AABox.init(Vec3.init(-5, -5, -5), Vec3.init(40, 5, 5)), &collector.base, .{});
                    try collector.checkError();
                    num_ids = 0;
                    for (collector.hits.items) |*hit| {
                        ids[num_ids] = hit.body_id;
                        num_ids += 1;
                    }
                    if (!s.checkIDs(ids[0..num_ids])) return false;
                }

                // A point in each fixed box
                for (s.fixed, 0..) |f, i| {
                    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
                    defer collector.deinit();
                    query.collidePoint(RVec3.init(2.0 * @as(f32, @floatFromInt(i)), 0, 0), &collector.base, .{});
                    try collector.checkError();
                    if (collector.hits.items.len != 1 or !collector.hits.items[0].body_id.eql(f)) return false;
                }
                return true;
            }
        };

        var shared: Shared = .{ .c = &c, .fixed = &fixed, .changing = &changing };
        var readers: [3]std.Thread = undefined;
        for (&readers) |*t| t.* = try std.Thread.spawn(.{}, Shared.reader, .{&shared});

        // This thread adds, moves and removes the changing bodies through the locking body interface and updates the
        // physics system (which rebuilds the broad phase). It runs until the readers ran enough queries.
        while (shared.started.load(.acquire) < readers.len) std.Thread.yield() catch {};
        var prng: std.Random.DefaultPrng = .init(round);
        const random = prng.random();
        var in_world: [16]bool = @splat(false);
        const min_queries = 50;
        var iteration: usize = 0;
        while (iteration < 200 or (shared.queries.load(.monotonic) < min_queries and iteration < 1_000_000)) : (iteration += 1) {
            const i = random.uintLessThan(usize, changing.len);
            if (in_world[i]) {
                if (random.boolean()) {
                    bi.removeBody(changing[i]);
                    in_world[i] = false;
                } else {
                    bi.setPosition(changing[i], RVec3.init(2.0 * @as(f32, @floatFromInt(i)) + 1, 0.5 * random.float(f32) - 0.25, 0), .dont_activate);
                }
            } else {
                try bi.addBody(changing[i], .dont_activate);
                in_world[i] = true;
            }
            if (iteration % 8 == 0)
                _ = try c.simulateSingleStep();
        }
        shared.stop.store(true, .release);
        for (readers) |t| t.join();
        try fw.expectEqual(@as(u32, 0), shared.failures.load(.monotonic));
        try fw.expect(shared.queries.load(.monotonic) >= min_queries);
    }
}
