//! Parity of NarrowPhaseQuery and EstimateCollisionResponse (Phase 5, Wave A7) with the C++ library.
//!
//! NarrowPhaseQuery: the same world on both sides, a PhysicsSystem with the layers of UnitTests/Layers.h and the
//! QuadTree broad phase that holds two bodies of every shape of the catalogue of PairwiseParity.zig (convex shapes,
//! decorators, static and mutable compounds, meshes, height fields, planes, empty shapes) at random positions and
//! rotations on random object layers (static, a few kinematic and dynamic), far from the origin with double precision.
//! Half of the bodies are added one by one, the other half as a batch. Then, in rounds between which bodies are removed,
//! moved, added back and the physics system is updated (which rebuilds the broad phase), every query runs with random
//! inputs through the locking and the non locking interface: CastRay (single hit, with a given starting fraction),
//! CastRay with a collector (back face modes, convex as solid), CollidePoint, CollideShape and
//! CollideShapeWithInternalEdgeRemoval (a random catalogue shape with a valid scale, max separation distance, tolerances,
//! back face mode, active edge mode and movement direction, collect faces, base offset), CastShape (extra convex radius,
//! shrunken shape, deepest point, back face modes, collect faces, active edges) and CollectTransformedShapes. Each query
//! gets random filters (a broad phase layer mask, an object layer mask, a body filter that rejects bodies before and
//! after locking them, a shape filter that rejects sub shapes and sees the body ID the query sets) that log every call,
//! and a random collector: a logging collector that records the protocol (OnBody / AddHit with the body of the context
//! and the early out fraction / OnBodyEnd) while it keeps every hit, follows the closest hit, forces an early out or
//! follows the closest hit per body, or Jolt's AllHit / ClosestHit / AnyHit / ClosestHitPerBody collectors, with a
//! random early out fraction set before the query. Every hit (all fields, faces, the leaf shapes and transforms of
//! CollectTransformedShapes), the final early out fraction and the hash of every filter call are compared bit for bit
//! in collector order.
//!
//! EstimateCollisionResponse: random manifolds (1 to 64 contact points, random normals, base offsets) between body
//! pairs of every motion type combination (random box sizes, positions, rotations, velocities, allowed DOFs, mass and
//! inertia overrides, inertia multipliers), with and without friction and restitution, random minimal restitution
//! velocities and iteration counts (including 0, 1 and a count above INT_MAX, which is negative in Jolt).
//!
//! Inputs that Jolt's debug build asserts on assert in Zolt's safe builds as well (CollisionArchitecture.md section 9),
//! so they only run without asserts (ReleaseFast), where Jolt's release behavior is compared:
//! - query shapes that contain a mesh, a height field or a plane (pairs that Jolt does not support against the meshes,
//!   height fields and planes of the world);
//! - Jolt's AnyHit collector with ray, point and collide queries (the world has mutable compounds with more than one
//!   block of sub shapes: WalkSubShapes only stops the current block when the visitor aborts, a second hit asserts);
//! - Jolt's ClosestHit and ClosestHitPerBody collectors with an early out fraction set before a point or collide query
//!   (not every collide function checks the early out fraction before it adds a hit, the collector then asserts that the
//!   early out fraction only decreases);
//! - a closest hit per body strategy behind CollideShapeWithInternalEdgeRemoval (the InternalEdgeRemovingCollector does
//!   not follow the early out fraction that the chained collector restores in OnBodyEnd, and asserts at the next body).
//! The logging collector never asserts by itself. C ABI wrappers: ZoltParity/Physics/NarrowPhaseQueryReference.cpp.

const std = @import("std");
const builtin = @import("builtin");
const zolt = @import("zolt");
const pw = @import("PairwiseParity.zig");
const TestLayers = @import("test_layers");

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AllowedDOFs = zolt.AllowedDOFs;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyFilter = zolt.BodyFilter;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerFilter = zolt.BroadPhaseLayerFilter;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const ClosestHitPerBodyCollisionCollector = zolt.ClosestHitPerBodyCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollidePointResult = zolt.CollidePointResult;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const ContactManifold = zolt.ContactManifold;
const ContactPoints = zolt.ContactPoints;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const Mat44 = zolt.Mat44;
const MotionType = zolt.MotionType;
const NarrowPhaseQuery = zolt.NarrowPhaseQuery;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const PhysicsSystem = zolt.PhysicsSystem;
const Quat = zolt.Quat;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const RefConst = zolt.RefConst;
const RMat44 = zolt.RMat44;
const RRayCast = zolt.RRayCast;
const RShapeCast = zolt.RShapeCast;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const SubShapeID = zolt.SubShapeID;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const io = std.testing.io;
const Layers = TestLayers.Layers;
const Gen = pw.Gen;
const Stream = pw.Stream;

/// Safe builds assert where Jolt's debug build asserts (see the file header): those inputs only run without asserts
const asserts = zolt.Core.enable_asserts;

/// The C++ reference functions, see NarrowPhaseQueryReference.cpp (the catalogue functions are in PairwiseReference.cpp)
const jolt = struct {
    extern fn pw_catalogue_shape(catalogue: *anyopaque, index: u32) *const anyopaque;
    extern fn npq_struct_sizes(out: *[3]u32) void;
    extern fn npq_world_create(catalogue: ?*anyopaque, bodies: [*]const NpqBody, num_bodies: u32, max_bodies: u32) *anyopaque;
    extern fn npq_world_destroy(world: *anyopaque) void;
    extern fn npq_world_remove(world: *anyopaque, body: u32) void;
    extern fn npq_world_add(world: *anyopaque, body: u32) void;
    extern fn npq_world_move(world: *anyopaque, body: u32, position: *const [3]f64, rotation: *const [4]f32) void;
    extern fn npq_world_update(world: *anyopaque) u32;
    extern fn npq_world_body_id(world: *anyopaque, body: u32) u32;
    extern fn npq_query(world: *anyopaque, catalogue: *anyopaque, num_nodes: u32, query: *const NpqQuery, out: [*]u32, capacity: u32) u32;
    extern fn npq_estimate(world: *anyopaque, input: *const NpqEstimateInput, out: [*]u32, capacity: u32) u32;
};

// Section markers in the streams, must match NarrowPhaseQueryReference.cpp
const marker_query: u32 = 0xD0000001;
const marker_on_body: u32 = 0xD0000002;
const marker_hit: u32 = 0xD0000003;
const marker_on_body_end: u32 = 0xD0000004;
const marker_result: u32 = 0xD0000005;
const marker_filters: u32 = 0xD0000006;
const marker_estimate: u32 = 0xD0000007;

/// Query kinds, must match the enum in NarrowPhaseQueryReference.cpp
const QueryKind = enum(u32) { ray, ray_collector, point, collide, collide_internal_edge_removal, cast, collect };

/// Must match NpqBody in NarrowPhaseQueryReference.cpp
const NpqBody = extern struct {
    position: [3]f64,
    rotation: [4]f32,
    /// Catalogue node of the shape
    node: u32,
    object_layer: u32,
    motion_type: u32,
    user_data: u32,
};

/// Must match NpqQuery in NarrowPhaseQueryReference.cpp
const NpqQuery = extern struct {
    /// Ray origin, the point of CollidePoint, the position (center of mass) of the shape of CollideShape / CastShape
    origin: [3]f64 = .{ 0, 0, 0 },
    base_offset: [3]f64 = .{ 0, 0, 0 },
    kind: QueryKind,
    locking: u32 = 1,
    /// 0..3: LoggingCollector mode, 4 AllHit, 5 ClosestHit, 6 AnyHit, 7 ClosestHitPerBody
    collector: u32 = 0,
    /// Early out fraction set before the query (when it is smaller than the initial one)
    early_out: f32 = math.flt_max,
    /// 0xffffffff: the default filter, otherwise BroadPhaseLayerFilterImpl
    broad_phase_layer_mask: u32 = 0xffffffff,
    /// 0xffffffff: the default filter, otherwise ObjectLayerFilterImpl
    object_layer_mask: u32 = 0xffffffff,
    /// 0: the default filter, otherwise BodyFilterImpl
    body_filter: u32 = 0,
    body_reject_modulus: u32 = 0,
    body_locked_reject_modulus: u32 = 0,
    /// 0: the default filter, otherwise ShapeFilterImpl
    shape_filter: u32 = 0,
    shape_reject_modulus: u32 = 0,
    direction: [3]f32 = .{ 0, 0, 0 },
    ray_fraction: f32 = 1.0 + math.flt_epsilon,
    back_face_mode_triangles: u32 = 0,
    back_face_mode_convex: u32 = 0,
    treat_convex_as_solid: u32 = 1,
    shape: u32 = 0,
    scale: [3]f32 = .{ 1, 1, 1 },
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    max_separation_distance: f32 = 0.0,
    collision_tolerance: f32 = zolt.physics_settings.default_collision_tolerance,
    penetration_tolerance: f32 = zolt.physics_settings.default_penetration_tolerance,
    back_face_mode: u32 = 0,
    active_edge_mode: u32 = 0,
    collect_faces: u32 = 0,
    active_edge_movement_direction: [3]f32 = .{ 0, 0, 0 },
    vertex_tolerance_sq: f32 = zolt.physics_settings.default_internal_edge_removal_vertex_tolerance_sq,
    extra_convex_radius: f32 = 0.0,
    use_shrunken_shape: u32 = 0,
    return_deepest_point: u32 = 0,
    box: [6]f32 = .{ 0, 0, 0, 0, 0, 0 },
};

/// Must match NpqEstimateBody in NarrowPhaseQueryReference.cpp
const NpqEstimateBody = extern struct {
    position: [3]f64,
    rotation: [4]f32,
    half_extent: [3]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    mass: f32,
    /// Diagonal of the inertia (override_mass == 2)
    inertia: [3]f32,
    inertia_rotation: [4]f32,
    inertia_multiplier: f32,
    motion_type: u32,
    allowed_dofs: u32,
    /// EOverrideMassProperties
    override_mass: u32,
};

/// Must match NpqEstimateInput in NarrowPhaseQueryReference.cpp
const NpqEstimateInput = extern struct {
    body1: NpqEstimateBody,
    body2: NpqEstimateBody,
    base_offset: [3]f64,
    normal: [3]f32,
    penetration_depth: f32,
    points1: [ContactPoints.capacity][3]f32,
    points2: [ContactPoints.capacity][3]f32,
    num_points: u32,
    combined_friction: f32,
    combined_restitution: f32,
    min_velocity_for_restitution: f32,
    num_iterations: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// Conversions

fn vec3(a: [3]f32) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn arr4(q: Quat) [4]f32 {
    return .{ q.getX(), q.getY(), q.getZ(), q.getW() };
}

fn rvec3(a: [3]f64) RVec3 {
    return RVec3.init(@floatCast(a[0]), @floatCast(a[1]), @floatCast(a[2]));
}

/// A float vector plus a double offset
fn add64(v: Vec3, offset: [3]f64) [3]f64 {
    return .{ @as(f64, v.getX()) + offset[0], @as(f64, v.getY()) + offset[1], @as(f64, v.getZ()) + offset[2] };
}

// ---------------------------------------------------------------------------------------------------------------------
// The filters, must match the filters of NarrowPhaseQueryReference.cpp

/// A hash of the calls of a filter
const FilterLog = struct {
    calls: u32 = 0,
    hash: u32 = 0x811c9dc5,

    fn add(self: *FilterLog, value: u32) void {
        self.hash = (self.hash ^ value) *% 0x01000193;
        self.calls += 1;
    }

    fn write(self: *const FilterLog, s: *Stream) void {
        s.u(self.calls);
        s.u(self.hash);
    }
};

/// Rejects a pseudo random subset (Reject in C++)
fn reject(modulus: u32, value: u32) bool {
    return modulus != 0 and ((value *% 0x9e3779b1) >> 16) % modulus == 0;
}

/// Accepts the broad phase layers in a bit mask, logs every call (the log is behind a pointer: the filter is const in
/// the queries, Rule M)
const BroadPhaseLayerFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: BroadPhaseLayerFilter = .init(@This()),
    mask: u32,
    log: *FilterLog,

    pub fn shouldCollide(self: *const @This(), layer: BroadPhaseLayer) bool {
        self.log.add(layer.getValue());
        return (self.mask >> @intCast(layer.getValue())) & 1 != 0;
    }
};

/// Accepts the object layers in a bit mask, logs every call
const ObjectLayerFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerFilter = .init(@This()),
    mask: u32,
    log: *FilterLog,

    pub fn shouldCollide(self: *const @This(), layer: ObjectLayer) bool {
        self.log.add(layer);
        return (self.mask >> @intCast(layer)) & 1 != 0;
    }
};

/// Rejects a pseudo random subset of the bodies before and after locking them, logs every call
const BodyFilterImpl = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollideLocked };

    base: BodyFilter = .init(@This()),
    reject_modulus: u32,
    locked_reject_modulus: u32,
    log: *FilterLog,

    pub fn shouldCollide(self: *const @This(), body_id: BodyID) bool {
        self.log.add(1);
        self.log.add(body_id.getIndexAndSequenceNumber());
        return !reject(self.reject_modulus, body_id.getIndexAndSequenceNumber());
    }

    pub fn shouldCollideLocked(self: *const @This(), body: *const Body) bool {
        self.log.add(2);
        self.log.add(body.getID().getIndexAndSequenceNumber());
        self.log.add(@truncate(body.getUserData()));
        return !reject(self.locked_reject_modulus, @truncate(body.getUserData()));
    }
};

/// Rejects a pseudo random subset of the shapes (by sub shape ID and body ID), logs every call with the body ID that the
/// query sets
const ShapeFilterImpl = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject_modulus: u32,
    log: *FilterLog,

    pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        const body = self.base.body_id2.getIndexAndSequenceNumber();
        self.log.add(1);
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        self.log.add(body);
        return !reject(self.reject_modulus, sub_shape_id_of_shape2.getValue() ^ body);
    }

    pub fn shouldCollidePair(self: *const @This(), shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        const body = self.base.body_id2.getIndexAndSequenceNumber();
        self.log.add(2);
        self.log.add(@intFromEnum(shape1.getSubType()));
        self.log.add(sub_shape_id_of_shape1.getValue());
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        self.log.add(body);
        return !reject(self.reject_modulus, sub_shape_id_of_shape1.getValue() *% 0x85ebca6b +% (sub_shape_id_of_shape2.getValue() ^ body));
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The results and the collectors, must match ResultWriter / NpqLoggingCollector / RunWithCollector in C++

const Writer = struct {
    s: *Stream,
    built: *const pw.Built,
    /// The number of results written (statistics of the test, not part of the stream)
    num_results: *u32,

    fn write(w: Writer, result: anytype) void {
        const s = w.s;
        w.num_results.* += 1;
        switch (@TypeOf(result.*)) {
            RayCastResult => {
                s.u(result.body_id.getIndexAndSequenceNumber());
                s.f(result.fraction);
                s.u(result.sub_shape_id2.getValue());
            },
            CollidePointResult => {
                s.u(result.body_id.getIndexAndSequenceNumber());
                s.u(result.sub_shape_id2.getValue());
            },
            CollideShapeResult => {
                s.v3(result.contact_point_on1);
                s.v3(result.contact_point_on2);
                s.v3(result.penetration_axis);
                s.f(result.penetration_depth);
                s.u(result.sub_shape_id1.getValue());
                s.u(result.sub_shape_id2.getValue());
                s.u(result.body_id2.getIndexAndSequenceNumber());
                s.face(&result.shape1_face);
                s.face(&result.shape2_face);
            },
            ShapeCastResult => {
                s.f(result.fraction);
                s.b(result.is_back_face_hit);
                w.write(&result.base);
            },
            TransformedShape => {
                // The shape as the index of the catalogue node (0xffffffff when it is not a node shape)
                var index: u32 = 0xffffffff;
                for (w.built.shapes.items, 0..) |node_shape, i| {
                    if (node_shape.get() == result.shape.get()) {
                        index = @intCast(i);
                        break;
                    }
                }
                s.u(index);
                s.u(@intFromEnum(result.shape.get().?.getSubType()));
                s.r3(result.shape_position_com);
                s.q(result.shape_rotation);
                s.v3(result.getShapeScale());
                s.u(result.body_id.getIndexAndSequenceNumber());
                s.u(result.sub_shape_id_creator.getID().getValue());
                s.u(result.sub_shape_id_creator.getNumBitsWritten());
                s.box(result.getWorldSpaceBounds());
            },
            else => @compileError("no writer for " ++ @typeName(@TypeOf(result.*))),
        }
    }
};

/// The early out fraction of a result (NpqEarlyOut in C++: TransformedShape has none in Jolt)
fn earlyOut(result: anytype) f32 {
    return if (@TypeOf(result.*) == TransformedShape) math.flt_max else result.getEarlyOutFraction();
}

/// A collector that writes the protocol of the query and follows a strategy: 0 keeps every hit, 1 the closest hit, 2
/// forces an early out at the first hit, 3 the closest hit per body
fn LoggingCollector(comptime CollectorType: type) type {
    return struct {
        pub const overrides = .{ .onBody, .onBodyEnd, .addHit };

        base: CollectorType = .init(@This()),
        writer: Writer,
        mode: u32,
        previous_early_out_fraction: f32 = 0.0,

        pub fn onBody(self: *@This(), body: *const Body) void {
            const s = self.writer.s;
            s.u(marker_on_body);
            s.u(body.getID().getIndexAndSequenceNumber());
            s.f(self.base.getEarlyOutFraction());
            self.previous_early_out_fraction = self.base.getEarlyOutFraction();
        }

        pub fn addHit(self: *@This(), result: *const CollectorType.ResultType) void {
            const s = self.writer.s;
            s.u(marker_hit);
            s.u(self.base.getContext().?.body_id.getIndexAndSequenceNumber());
            s.f(self.base.getEarlyOutFraction());
            self.writer.write(result);
            const fraction = earlyOut(result);
            switch (self.mode) {
                1, 3 => if (fraction < self.base.getEarlyOutFraction()) self.base.updateEarlyOutFraction(fraction),
                2 => self.base.forceEarlyOut(),
                else => {},
            }
        }

        pub fn onBodyEnd(self: *@This()) void {
            const s = self.writer.s;
            s.u(marker_on_body_end);
            s.f(self.base.getEarlyOutFraction());
            if (self.mode == 3)
                self.base.resetEarlyOutFraction(.{ .fraction = self.previous_early_out_fraction });
        }
    };
}

fn preset(collector: anytype, early_out: f32) void {
    if (early_out < collector.getEarlyOutFraction())
        collector.updateEarlyOutFraction(early_out);
}

/// RunWithCollector in C++: run `query.run(collector)` with the collector of the query, write the hits and the final
/// early out fraction (Jolt's ClosestHit and ClosestHitPerBody collectors need GetEarlyOutFraction, which TransformedShape
/// does not have: collect queries use AllHit for them, like C++)
fn runWithCollector(comptime CollectorType: type, allocator: Allocator, w: Writer, q: *const NpqQuery, query: anytype) !void {
    const s = w.s;
    const has_early_out = CollectorType.ResultType != TransformedShape;
    switch (q.collector) {
        0...3 => {
            var collector: LoggingCollector(CollectorType) = .{ .writer = w, .mode = q.collector };
            preset(&collector.base, q.early_out);
            try query.run(&collector.base);
            s.u(marker_result);
            s.f(collector.base.getEarlyOutFraction());
        },
        5 => if (has_early_out) {
            var collector = ClosestHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            preset(&collector.base, q.early_out);
            try query.run(&collector.base);
            s.u(marker_result);
            s.b(collector.hadHit());
            if (collector.hadHit()) w.write(&collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        } else try runAllHit(CollectorType, allocator, w, q, query),
        6 => {
            var collector = AnyHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            preset(&collector.base, q.early_out);
            try query.run(&collector.base);
            s.u(marker_result);
            s.b(collector.hadHit());
            if (collector.hadHit()) w.write(&collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        },
        7 => if (has_early_out) {
            var collector = ClosestHitPerBodyCollisionCollector(CollectorType).init(allocator);
            defer collector.deinit();
            preset(&collector.base, q.early_out);
            const result = query.run(&collector.base);
            const collector_result = collector.checkError();
            try result;
            try collector_result;
            s.u(marker_result);
            s.u(@intCast(collector.hits.items.len));
            for (collector.hits.items) |*hit| w.write(hit);
            s.f(collector.base.getEarlyOutFraction());
        } else try runAllHit(CollectorType, allocator, w, q, query),
        else => try runAllHit(CollectorType, allocator, w, q, query),
    }
}

fn runAllHit(comptime CollectorType: type, allocator: Allocator, w: Writer, q: *const NpqQuery, query: anytype) !void {
    const s = w.s;
    var collector = AllHitCollisionCollector(CollectorType).init(allocator);
    defer collector.deinit();
    preset(&collector.base, q.early_out);
    const result = query.run(&collector.base);
    const collector_result = collector.checkError();
    try result;
    try collector_result;
    s.u(marker_result);
    s.u(@intCast(collector.hits.items.len));
    for (collector.hits.items) |*hit| w.write(hit);
    s.f(collector.base.getEarlyOutFraction());
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt world, must match NpqWorld in C++

const ZoltWorld = struct {
    allocator: Allocator,
    temp: TempAllocatorImpl,
    jobs: JobSystemSingleThreaded,
    broad_phase_layer_interface: TestLayers.BPLayerInterfaceImpl,
    object_vs_broad_phase_layer_filter: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: TestLayers.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    ids: std.ArrayList(BodyID) = .empty,

    /// Heap allocated: the physics system must not move
    fn create(allocator: Allocator, built: ?*const pw.Built, bodies: []const NpqBody, max_bodies: u32) !*ZoltWorld {
        const self = try allocator.create(ZoltWorld);
        errdefer allocator.destroy(self);
        self.* = .{ .allocator = allocator, .temp = try .init(allocator, 4 * 1024 * 1024), .jobs = undefined, .broad_phase_layer_interface = .init() };
        errdefer self.temp.deinit();
        self.jobs = try .init(allocator, io, zolt.physics_settings.max_physics_jobs);
        errdefer self.jobs.deinit();
        try self.system.init(allocator, io, max_bodies, 0, 64, 64, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        errdefer self.ids.deinit(allocator);

        // Create every body, add the first half one by one and the second half as a batch
        const bi = self.system.getBodyInterfaceNoLockMut();
        for (bodies) |b| {
            var settings = BodyCreationSettings.initWithShape(built.?.shape(b.node), rvec3(b.position), quat(b.rotation), @enumFromInt(b.motion_type), @intCast(b.object_layer));
            defer settings.deinit();
            settings.user_data = b.user_data;
            const body = (try bi.createBody(&settings)) orelse return error.OutOfBodies;
            try self.ids.append(allocator, body.getID());
        }
        const half = bodies.len / 2;
        for (self.ids.items[0..half]) |id| try bi.addBody(id, .dont_activate);
        if (bodies.len > half) {
            const batch = try allocator.dupe(BodyID, self.ids.items[half..]);
            defer allocator.free(batch);
            const state = try bi.addBodiesPrepare(batch);
            bi.addBodiesFinalize(batch, state, .dont_activate);
        }
        return self;
    }

    fn destroy(self: *ZoltWorld) void {
        const allocator = self.allocator;
        self.system.deinit();
        self.ids.deinit(allocator);
        self.jobs.deinit();
        self.temp.deinit();
        allocator.destroy(self);
    }

    fn update(self: *ZoltWorld) !u32 {
        const errors = try self.system.update(1.0 / 60.0, 1, self.temp.tempAllocator(), self.jobs.jobSystem());
        return @bitCast(errors);
    }
};

/// npq_query
fn zoltQuery(allocator: Allocator, world: *ZoltWorld, built: *const pw.Built, q: *const NpqQuery, s: *Stream, num_results: *u32) !void {
    const query = if (q.locking != 0) world.system.getNarrowPhaseQuery() else world.system.getNarrowPhaseQueryNoLock();
    const w: Writer = .{ .s = s, .built = built, .num_results = num_results };

    // The filters
    var broad_phase_log: FilterLog = .{};
    var object_log: FilterLog = .{};
    var body_log: FilterLog = .{};
    var shape_log: FilterLog = .{};
    const npq_broad_phase_filter: BroadPhaseLayerFilterImpl = .{ .mask = q.broad_phase_layer_mask, .log = &broad_phase_log };
    const default_broad_phase_filter: BroadPhaseLayerFilter = .{};
    const broad_phase_filter: *const BroadPhaseLayerFilter = if (q.broad_phase_layer_mask != 0xffffffff) &npq_broad_phase_filter.base else &default_broad_phase_filter;
    const npq_object_filter: ObjectLayerFilterImpl = .{ .mask = q.object_layer_mask, .log = &object_log };
    const default_object_filter: ObjectLayerFilter = .{};
    const object_filter: *const ObjectLayerFilter = if (q.object_layer_mask != 0xffffffff) &npq_object_filter.base else &default_object_filter;
    const npq_body_filter: BodyFilterImpl = .{ .reject_modulus = q.body_reject_modulus, .locked_reject_modulus = q.body_locked_reject_modulus, .log = &body_log };
    const default_body_filter: BodyFilter = .{};
    const body_filter: *const BodyFilter = if (q.body_filter != 0) &npq_body_filter.base else &default_body_filter;
    var npq_shape_filter: ShapeFilterImpl = .{ .reject_modulus = q.shape_reject_modulus, .log = &shape_log };
    var default_shape_filter: ShapeFilter = .{};
    const shape_filter: *ShapeFilter = if (q.shape_filter != 0) &npq_shape_filter.base else &default_shape_filter;
    const filters: NarrowPhaseQuery.QueryFilters = .{ .broad_phase_layer_filter = broad_phase_filter, .object_layer_filter = object_filter, .body_filter = body_filter, .shape_filter = shape_filter };

    s.u(marker_query);
    s.u(@intFromEnum(q.kind));
    switch (q.kind) {
        .ray => {
            var hit: RayCastResult = .{};
            hit.fraction = q.ray_fraction;
            const had_hit = query.castRay(RRayCast.init(rvec3(q.origin), vec3(q.direction)), &hit, .{ .broad_phase_layer_filter = broad_phase_filter, .object_layer_filter = object_filter, .body_filter = body_filter });
            s.u(marker_result);
            s.b(had_hit);
            w.write(&hit);
            if (hit.body_id.isInvalid()) num_results.* -= 1; // Not a hit (statistics only)
        },
        .ray_collector => {
            var settings: RayCastSettings = .{};
            settings.back_face_mode_triangles = if (q.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
            settings.back_face_mode_convex = if (q.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
            settings.treat_convex_as_solid = q.treat_convex_as_solid != 0;
            const Run = struct {
                query: *const NarrowPhaseQuery,
                ray: RRayCast,
                settings: *const RayCastSettings,
                filters: NarrowPhaseQuery.QueryFilters,
                fn run(c: @This(), collector: *CastRayCollector) !void {
                    c.query.castRayCollector(c.ray, c.settings, collector, c.filters);
                }
            };
            try runWithCollector(CastRayCollector, allocator, w, q, Run{ .query = query, .ray = RRayCast.init(rvec3(q.origin), vec3(q.direction)), .settings = &settings, .filters = filters });
        },
        .point => {
            const Run = struct {
                query: *const NarrowPhaseQuery,
                point: RVec3,
                filters: NarrowPhaseQuery.QueryFilters,
                fn run(c: @This(), collector: *CollidePointCollector) !void {
                    c.query.collidePoint(c.point, collector, c.filters);
                }
            };
            try runWithCollector(CollidePointCollector, allocator, w, q, Run{ .query = query, .point = rvec3(q.origin), .filters = filters });
        },
        .collide, .collide_internal_edge_removal => {
            var settings: CollideShapeSettings = .{};
            settings.max_separation_distance = q.max_separation_distance;
            settings.collision_tolerance = q.collision_tolerance;
            settings.penetration_tolerance = q.penetration_tolerance;
            settings.back_face_mode = if (q.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
            settings.active_edge_mode = if (q.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
            settings.collect_faces_mode = if (q.collect_faces != 0) .collect_faces else .no_faces;
            settings.active_edge_movement_direction = vec3(q.active_edge_movement_direction);
            settings.internal_edge_removal_vertex_tolerance_sq = q.vertex_tolerance_sq;
            const Run = struct {
                allocator: Allocator,
                query: *const NarrowPhaseQuery,
                internal_edge_removal: bool,
                shape: *const Shape,
                scale: Vec3,
                transform: RMat44,
                settings: *const CollideShapeSettings,
                base_offset: RVec3,
                filters: NarrowPhaseQuery.QueryFilters,
                fn run(c: @This(), collector: *CollideShapeCollector) !void {
                    if (!c.internal_edge_removal)
                        c.query.collideShape(c.shape, c.scale, c.transform, c.settings, c.base_offset, collector, c.filters)
                    else
                        try c.query.collideShapeWithInternalEdgeRemoval(c.allocator, c.shape, c.scale, c.transform, c.settings, c.base_offset, collector, c.filters);
                }
            };
            try runWithCollector(CollideShapeCollector, allocator, w, q, Run{
                .allocator = allocator,
                .query = query,
                .internal_edge_removal = q.kind == .collide_internal_edge_removal,
                .shape = built.shape(q.shape),
                .scale = vec3(q.scale),
                .transform = RMat44.rotationTranslation(quat(q.rotation), rvec3(q.origin)),
                .settings = &settings,
                .base_offset = rvec3(q.base_offset),
                .filters = filters,
            });
        },
        .cast => {
            var settings: ShapeCastSettings = .{};
            settings.collision_tolerance = q.collision_tolerance;
            settings.penetration_tolerance = q.penetration_tolerance;
            settings.extra_convex_radius = q.extra_convex_radius;
            settings.back_face_mode_triangles = if (q.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
            settings.back_face_mode_convex = if (q.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
            settings.use_shrunken_shape_and_convex_radius = q.use_shrunken_shape != 0;
            settings.return_deepest_point = q.return_deepest_point != 0;
            settings.collect_faces_mode = if (q.collect_faces != 0) .collect_faces else .no_faces;
            settings.active_edge_mode = if (q.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
            settings.active_edge_movement_direction = vec3(q.active_edge_movement_direction);
            const shape_cast = RShapeCast.init(built.shape(q.shape), vec3(q.scale), RMat44.rotationTranslation(quat(q.rotation), rvec3(q.origin)), vec3(q.direction));
            s.box(shape_cast.shape_world_bounds);
            const Run = struct {
                query: *const NarrowPhaseQuery,
                shape_cast: *const RShapeCast,
                settings: *const ShapeCastSettings,
                base_offset: RVec3,
                filters: NarrowPhaseQuery.QueryFilters,
                fn run(c: @This(), collector: *CastShapeCollector) !void {
                    c.query.castShape(c.shape_cast, c.settings, c.base_offset, collector, c.filters);
                }
            };
            try runWithCollector(CastShapeCollector, allocator, w, q, Run{ .query = query, .shape_cast = &shape_cast, .settings = &settings, .base_offset = rvec3(q.base_offset), .filters = filters });
        },
        .collect => {
            const Run = struct {
                query: *const NarrowPhaseQuery,
                box: AABox,
                filters: NarrowPhaseQuery.CollectFilters,
                fn run(c: @This(), collector: *TransformedShapeCollector) !void {
                    c.query.collectTransformedShapes(c.box, collector, c.filters);
                }
            };
            const collect_filters: NarrowPhaseQuery.CollectFilters = .{ .broad_phase_layer_filter = broad_phase_filter, .object_layer_filter = object_filter, .body_filter = body_filter, .shape_filter = shape_filter };
            try runWithCollector(TransformedShapeCollector, allocator, w, q, Run{ .query = query, .box = AABox.init(vec3(q.box[0..3].*), vec3(q.box[3..6].*)), .filters = collect_filters });
        },
    }

    // The filter calls
    s.u(marker_filters);
    broad_phase_log.write(s);
    object_log.write(s);
    body_log.write(s);
    shape_log.write(s);
}

// ---------------------------------------------------------------------------------------------------------------------
// The NarrowPhaseQuery test

/// The size of the world (the positions of the bodies are in [-world_extent, world_extent]^3 around the world offset)
const world_extent: f32 = 8.0;

/// The world offset: far from the origin in double precision (the broad phase drops precision there, both sides alike)
const world_offset: [3]f64 = if (zolt.Core.double_precision) .{ 1.0e5, -2.0e4, 3.0e5 } else .{ 0, 0, 0 };

/// The object layers of the bodies
const body_layers = [_]ObjectLayer{ Layers.non_moving, Layers.moving, Layers.moving2, Layers.hq_debris, Layers.lq_debris, Layers.sensor };

/// Both worlds and the information that the input generation needs
const Worlds = struct {
    allocator: Allocator,
    built: *pw.Built,
    bodies: std.ArrayList(NpqBody) = .empty,
    in_world: std.ArrayList(bool) = .empty,
    zolt_world: *ZoltWorld = undefined,
    jolt_world: *anyopaque = undefined,
    /// The catalogue nodes that can be the shape of a collide / cast query in this build (see the file header)
    collide_nodes: std.ArrayList(u32) = .empty,
    cast_nodes: std.ArrayList(u32) = .empty,
    /// The world contains a body whose shape has a MutableCompoundShape with more than one block of sub shapes
    multi_block: bool = false,

    fn deinit(self: *Worlds) void {
        self.zolt_world.destroy();
        jolt.npq_world_destroy(self.jolt_world);
        self.bodies.deinit(self.allocator);
        self.in_world.deinit(self.allocator);
        self.collide_nodes.deinit(self.allocator);
        self.cast_nodes.deinit(self.allocator);
    }
};

/// The kinds of catalogue nodes that can be the shape of a kinematic or dynamic body (simple convex shapes)
fn canMove(kind: pw.NodeKind) bool {
    return switch (kind) {
        .sphere, .box, .capsule, .tapered_capsule, .cylinder, .tapered_cylinder => true,
        else => false,
    };
}

fn createWorlds(allocator: Allocator, built: *pw.Built, gen: *Gen, cmp: *pw.Comparison) !Worlds {
    var worlds: Worlds = .{ .allocator = allocator, .built = built };
    errdefer {
        worlds.bodies.deinit(allocator);
        worlds.in_world.deinit(allocator);
        worlds.collide_nodes.deinit(allocator);
        worlds.cast_nodes.deinit(allocator);
    }

    // Two bodies per catalogue entry
    for (built.cat.entries.items) |entry| {
        for (0..2) |i| {
            const kind = built.cat.nodes.items[entry.node].kind;
            const motion_type: MotionType = if (i == 1 and canMove(kind)) (if (gen.chance(50)) .kinematic else .dynamic) else .static;
            try worlds.bodies.append(allocator, .{
                .position = add64(gen.vec(-world_extent, world_extent), world_offset),
                .rotation = arr4(gen.rotation()),
                .node = entry.node,
                .object_layer = body_layers[gen.index(body_layers.len)],
                .motion_type = @intFromEnum(motion_type),
                .user_data = @intCast(worlds.bodies.items.len),
            });
            try worlds.in_world.append(allocator, true);
            worlds.multi_block = worlds.multi_block or built.multi_block.items[entry.node];
        }
    }

    // The query shapes: every entry whose leaves are supported against the leaves of every body (only in safe builds)
    for (built.cat.entries.items) |entry| {
        var collide_ok = true;
        var cast_ok = true;
        if (asserts) {
            for (worlds.bodies.items) |b| {
                collide_ok = collide_ok and built.supported(entry.node, b.node, false);
                cast_ok = cast_ok and built.supported(entry.node, b.node, true);
            }
        }
        if (collide_ok) try worlds.collide_nodes.append(allocator, entry.node);
        if (cast_ok) try worlds.cast_nodes.append(allocator, entry.node);
    }

    const max_bodies: u32 = @intCast(worlds.bodies.items.len + 8);
    worlds.zolt_world = try ZoltWorld.create(allocator, built, worlds.bodies.items, max_bodies);
    errdefer worlds.zolt_world.destroy();
    worlds.jolt_world = jolt.npq_world_create(built.handle, worlds.bodies.items.ptr, @intCast(worlds.bodies.items.len), max_bodies);

    // The body IDs are the same
    var zolt_ids: std.ArrayList(u32) = .empty;
    defer zolt_ids.deinit(allocator);
    var jolt_ids: std.ArrayList(u32) = .empty;
    defer jolt_ids.deinit(allocator);
    for (worlds.zolt_world.ids.items, 0..) |id, i| {
        try zolt_ids.append(allocator, id.getIndexAndSequenceNumber());
        try jolt_ids.append(allocator, jolt.npq_world_body_id(worlds.jolt_world, @intCast(i)));
    }
    cmp.check("body IDs", worlds.bodies.items.len, zolt_ids.items, jolt_ids.items);
    return worlds;
}

/// A random point in the world (or near a body)
fn randomPoint(gen: *Gen, worlds: *const Worlds) [3]f64 {
    if (gen.chance(60)) {
        const b = worlds.bodies.items[gen.index(@intCast(worlds.bodies.items.len))];
        const p = gen.vec(-1.0, 1.0);
        return .{ b.position[0] + p.getX(), b.position[1] + p.getY(), b.position[2] + p.getZ() };
    }
    return add64(gen.vec(-world_extent - 2, world_extent + 2), world_offset);
}

/// A direction from `origin` to a random body (beyond it) or a random direction, of a random length
fn randomDirection(gen: *Gen, worlds: *const Worlds, origin: [3]f64, max_length: f32) [3]f32 {
    if (gen.chance(5))
        return .{ 0, 0, 0 };
    if (gen.chance(60)) {
        const b = worlds.bodies.items[gen.index(@intCast(worlds.bodies.items.len))];
        const delta = Vec3.init(@floatCast(b.position[0] - origin[0]), @floatCast(b.position[1] - origin[1]), @floatCast(b.position[2] - origin[2]));
        if (delta.lengthSq() > 1.0e-4)
            return arr3(delta.mulScalar(gen.float(0.5, 2.0)));
    }
    return arr3(gen.direction().mulScalar(gen.float(0.1, max_length)));
}

/// A base offset for the shape queries: zero, the position of the shape or near it
fn randomBaseOffset(gen: *Gen, position: [3]f64) [3]f64 {
    return switch (gen.index(3)) {
        0 => .{ 0, 0, 0 },
        1 => position,
        else => .{ position[0] + 0.5, position[1] - 0.25, position[2] + 1.0 },
    };
}

/// A random query of a kind with random filters and collector
fn randomQuery(gen: *Gen, worlds: *const Worlds, kind: QueryKind) NpqQuery {
    var q: NpqQuery = .{ .kind = kind };
    q.locking = @intFromBool(gen.chance(50));

    // Filters
    if (gen.chance(30)) q.broad_phase_layer_mask = gen.index(@as(u32, 1) << @intCast(TestLayers.BroadPhaseLayers.num_layers));
    if (gen.chance(30)) q.object_layer_mask = gen.index(@as(u32, 1) << @intCast(Layers.num_layers));
    if (gen.chance(40)) {
        q.body_filter = 1;
        q.body_reject_modulus = if (gen.chance(30)) 0 else 2 + gen.index(6);
        q.body_locked_reject_modulus = if (gen.chance(30)) 0 else 2 + gen.index(6);
    }
    if (kind != .ray and gen.chance(50)) {
        q.shape_filter = 1;
        q.shape_reject_modulus = if (gen.chance(30)) 0 else 2 + gen.index(8);
    }

    // Collector and early out fraction
    q.collector = gen.index(8);
    if (kind == .collect and (q.collector == 5 or q.collector == 7)) q.collector = 4; // TransformedShape has no early out fraction
    if (asserts and q.collector == 6 and kind != .cast and worlds.multi_block) q.collector = 2; // AnyHit asserts on a second hit
    if (asserts and kind == .collide_internal_edge_removal and (q.collector == 3 or q.collector == 7)) q.collector = 1; // Restoring the early out fraction behind InternalEdgeRemovingCollector asserts
    const preset_allowed = !asserts or !((q.collector == 5 or q.collector == 7) and (kind == .point or kind == .collide or kind == .collide_internal_edge_removal));
    if (preset_allowed and gen.chance(30)) {
        q.early_out = switch (kind) {
            .ray, .ray_collector => gen.float(0.05, 1.0),
            .cast => gen.float(-0.3, 1.0),
            else => gen.float(-0.5, 0.5),
        };
    }

    switch (kind) {
        .ray, .ray_collector => {
            q.origin = randomPoint(gen, worlds);
            q.direction = randomDirection(gen, worlds, q.origin, 30.0);
            if (gen.chance(30)) q.ray_fraction = gen.float(0.05, 1.0);
            q.back_face_mode_triangles = @intFromBool(gen.chance(50));
            q.back_face_mode_convex = @intFromBool(gen.chance(50));
            q.treat_convex_as_solid = @intFromBool(gen.chance(70));
        },
        .point => q.origin = randomPoint(gen, worlds),
        .collide, .collide_internal_edge_removal, .cast => {
            const nodes = if (kind == .cast) worlds.cast_nodes.items else worlds.collide_nodes.items;
            q.shape = nodes[gen.index(@intCast(nodes.len))];
            q.scale = arr3(pw.randomScale(gen, worlds.built.shape(q.shape)));
            q.rotation = arr4(gen.rotation());
            q.origin = randomPoint(gen, worlds);
            q.base_offset = randomBaseOffset(gen, q.origin);
            if (gen.chance(30)) q.collision_tolerance = gen.float(1.0e-5, 1.0e-3);
            if (gen.chance(30)) q.penetration_tolerance = gen.float(1.0e-5, 1.0e-3);
            q.back_face_mode = @intFromBool(gen.chance(50));
            q.back_face_mode_triangles = @intFromBool(gen.chance(50));
            q.back_face_mode_convex = @intFromBool(gen.chance(50));
            q.active_edge_mode = @intFromBool(gen.chance(50));
            q.collect_faces = @intFromBool(gen.chance(50));
            if (gen.chance(50)) q.active_edge_movement_direction = arr3(gen.vec(-2, 2));
            if (kind == .cast) {
                q.direction = randomDirection(gen, worlds, q.origin, 15.0);
                if (gen.chance(30)) q.extra_convex_radius = gen.float(0, 0.3);
                q.use_shrunken_shape = @intFromBool(gen.chance(50));
                q.return_deepest_point = @intFromBool(gen.chance(50));
            } else {
                if (gen.chance(40)) q.max_separation_distance = gen.float(0, 0.5);
                if (gen.chance(30)) q.vertex_tolerance_sq = gen.float(1.0e-8, 1.0e-3);
            }
        },
        .collect => {
            const center = gen.vec(-world_extent, world_extent);
            const extent = gen.vec(0.5, 6.0);
            const offset = Vec3.init(@floatCast(world_offset[0]), @floatCast(world_offset[1]), @floatCast(world_offset[2]));
            q.box = arr3(center.sub(extent).add(offset)) ++ arr3(center.add(extent).add(offset));
        },
    }
    return q;
}

/// Run `per_kind` random queries of every kind on both worlds and compare the streams
fn runQueries(allocator: Allocator, worlds: *Worlds, gen: *Gen, buffer: []u32, per_kind: usize, cmp: *pw.Comparison, stats: *[@typeInfo(QueryKind).@"enum".fields.len]usize) !void {
    var s: Stream = .{ .allocator = allocator };
    defer s.deinit();
    for (0..per_kind) |_| {
        for (std.enums.values(QueryKind)) |kind| {
            const q = randomQuery(gen, worlds, kind);
            s.values.clearRetainingCapacity();
            var num_results: u32 = 0;
            try zoltQuery(allocator, worlds.zolt_world, worlds.built, &q, &s, &num_results);
            const jolt_values = pw.joltStream(buffer, struct {
                w: *anyopaque,
                c: *anyopaque,
                n: u32,
                q: *const NpqQuery,
                pub fn run(c: @This(), out: [*]u32, capacity: u32) u32 {
                    return jolt.npq_query(c.w, c.c, c.n, c.q, out, capacity);
                }
            }{ .w = worlds.jolt_world, .c = worlds.built.handle, .n = @intCast(worlds.built.shapes.items.len), .q = &q });
            cmp.check(@tagName(kind), q, s.values.items, jolt_values);

            // Count the queries that found something
            stats[@intFromEnum(kind)] += @intFromBool(num_results > 0);
        }
    }
}

/// Print the number of queries with hits (passing tests must not print, only for tuning)
const print_statistics = false;

test "NarrowPhaseQuery parity: every query on a world of every shape type" {
    const allocator = std.testing.allocator;
    const buffer = try allocator.alloc(u32, pw.stream_capacity);
    defer allocator.free(buffer);

    // The struct layouts are the same on both sides
    var sizes: [3]u32 = undefined;
    jolt.npq_struct_sizes(&sizes);
    try std.testing.expectEqual([3]u32{ @sizeOf(NpqBody), @sizeOf(NpqQuery), @sizeOf(NpqEstimateInput) }, sizes);

    const built = try pw.Built.init(allocator, try pw.makeCatalogue(allocator), buffer);
    defer built.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x6e70715a } };
    var cmp: pw.Comparison = .{ .name = "NarrowPhaseQuery parity" };
    var worlds = try createWorlds(allocator, built, &gen, &cmp);
    defer worlds.deinit();

    const per_kind: usize = if (builtin.mode == .Debug) 600 else 3000;
    var stats: [@typeInfo(QueryKind).@"enum".fields.len]usize = @splat(0);
    const num_bodies: u32 = @intCast(worlds.bodies.items.len);

    // Round 1: the world as it was created
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);

    // Round 2: a quarter of the bodies removed
    for (0..num_bodies / 4) |_| {
        const i = gen.index(num_bodies);
        if (worlds.in_world.items[i]) {
            worlds.zolt_world.system.getBodyInterfaceMut().removeBody(worlds.zolt_world.ids.items[i]);
            jolt.npq_world_remove(worlds.jolt_world, i);
            worlds.in_world.items[i] = false;
        }
    }
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);

    // Round 3: a quarter of the bodies moved (in the broad phase or not)
    for (0..num_bodies / 4) |_| {
        const i = gen.index(num_bodies);
        const b = &worlds.bodies.items[i];
        b.position = add64(gen.vec(-world_extent, world_extent), world_offset);
        b.rotation = arr4(gen.rotation());
        worlds.zolt_world.system.getBodyInterfaceMut().setPositionAndRotation(worlds.zolt_world.ids.items[i], rvec3(b.position), quat(b.rotation), .dont_activate);
        jolt.npq_world_move(worlds.jolt_world, i, &b.position, &b.rotation);
    }
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);

    // Round 4: after an update (the broad phase rebuilds its trees)
    try std.testing.expectEqual(jolt.npq_world_update(worlds.jolt_world), try worlds.zolt_world.update());
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);

    // Round 5: the removed bodies added back, updated again
    for (worlds.in_world.items, 0..) |*in_world, i| {
        if (!in_world.*) {
            try worlds.zolt_world.system.getBodyInterfaceMut().addBody(worlds.zolt_world.ids.items[i], .dont_activate);
            jolt.npq_world_add(worlds.jolt_world, @intCast(i));
            in_world.* = true;
        }
    }
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);
    try std.testing.expectEqual(jolt.npq_world_update(worlds.jolt_world), try worlds.zolt_world.update());
    try runQueries(allocator, &worlds, &gen, buffer, per_kind, &cmp, &stats);

    if (print_statistics)
        std.debug.print("NarrowPhaseQuery parity: {d} queries per kind, with hits: {any}\n", .{ 6 * per_kind, stats });

    // Every kind of query finds something (the inputs reach the narrow phase)
    for (stats, 0..) |n, kind| {
        if (n < per_kind / 2) {
            std.debug.print("NarrowPhaseQuery parity: only {d} {s} queries with hits\n", .{ n, @tagName(@as(QueryKind, @enumFromInt(kind))) });
            return error.TestUnexpectedResult;
        }
    }
    try cmp.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// EstimateCollisionResponse

fn randomEstimateBody(gen: *Gen) NpqEstimateBody {
    const motion_type: MotionType = switch (gen.index(3)) {
        0 => .static,
        1 => .kinematic,
        else => .dynamic,
    };
    const allowed_dofs: u8 = switch (gen.index(8)) {
        0 => @bitCast(AllowedDOFs.plane_2d),
        1 => 0b000111, // Translation only
        2 => 0b111000, // Rotation only
        3 => 0b010101, // Translation X, Z, rotation Y
        else => @bitCast(AllowedDOFs.all),
    };
    return .{
        .position = add64(gen.vec(-3, 3), world_offset),
        .rotation = arr4(gen.rotation()),
        .half_extent = arr3(gen.vec(0.1, 2.0)),
        .linear_velocity = arr3(gen.vec(-5, 5)),
        .angular_velocity = arr3(gen.vec(-3, 3)),
        .mass = gen.float(1, 100),
        .inertia = arr3(gen.vec(0.5, 5)),
        .inertia_rotation = arr4(gen.rotation()),
        .inertia_multiplier = if (gen.chance(70)) 1.0 else gen.float(0.5, 2.0),
        .motion_type = @intFromEnum(motion_type),
        .allowed_dofs = allowed_dofs,
        .override_mass = gen.index(3),
    };
}

fn randomEstimate(gen: *Gen) NpqEstimateInput {
    var in: NpqEstimateInput = undefined;
    in.body1 = randomEstimateBody(gen);
    in.body2 = randomEstimateBody(gen);
    in.base_offset = add64(gen.vec(-2, 2), world_offset);
    const normal = gen.direction();
    in.normal = arr3(normal);
    in.penetration_depth = gen.float(-0.05, 0.1);
    in.num_points = switch (gen.index(10)) {
        0 => 1,
        1 => 2,
        2 => 3 + gen.index(62),
        else => 4,
    };
    for (0..ContactPoints.capacity) |i| {
        const p1 = gen.vec(-2, 2);
        in.points1[i] = arr3(p1);
        in.points2[i] = arr3(p1.add(normal.mulScalar(gen.float(-0.1, 0.1))).add(gen.vec(-0.01, 0.01)));
    }
    in.combined_friction = if (gen.chance(30)) 0.0 else gen.float(0, 1.5);
    in.combined_restitution = if (gen.chance(30)) 0.0 else gen.float(0, 1.0);
    in.min_velocity_for_restitution = if (gen.chance(50)) 1.0 else gen.float(0, 5);
    in.num_iterations = switch (gen.index(20)) {
        0 => 0,
        1 => 1,
        2 => 0x80000000, // A negative int in Jolt: no iterations
        3, 4, 5 => gen.index(30),
        else => 10,
    };
    return in;
}

/// npq_estimate
fn zoltEstimate(world: *ZoltWorld, in: *const NpqEstimateInput, s: *Stream) !void {
    const allocator = world.allocator;
    const bi = world.system.getBodyInterfaceMut();

    // Create the bodies
    var bodies: [2]*Body = undefined;
    for (&bodies, 0..) |*body, i| {
        const b = if (i == 0) &in.body1 else &in.body2;
        var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, vec3(b.half_extent), .{})).asShape(), rvec3(b.position), quat(b.rotation), @enumFromInt(b.motion_type), Layers.moving);
        defer settings.deinit();
        settings.allowed_dofs = @bitCast(@as(u8, @intCast(b.allowed_dofs)));
        settings.override_mass_properties = @enumFromInt(b.override_mass);
        settings.mass_properties_override.mass = b.mass;
        settings.mass_properties_override.inertia = Mat44.rotationQuat(quat(b.inertia_rotation)).mul(Mat44.scaleVec3(vec3(b.inertia))).mul(Mat44.rotationQuat(quat(b.inertia_rotation)).transposed());
        settings.inertia_multiplier = b.inertia_multiplier;
        if (b.motion_type != @intFromEnum(MotionType.static)) {
            settings.linear_velocity = vec3(b.linear_velocity);
            settings.angular_velocity = vec3(b.angular_velocity);
        }
        body.* = (try bi.createBody(&settings)) orelse return error.OutOfBodies;
    }
    defer for (bodies) |body| bi.destroyBody(body.getID());

    // The manifold
    var manifold: ContactManifold = .{};
    manifold.base_offset = rvec3(in.base_offset);
    manifold.world_space_normal = vec3(in.normal);
    manifold.penetration_depth = in.penetration_depth;
    for (0..in.num_points) |i| {
        manifold.relative_contact_points_on1.append(vec3(in.points1[i]));
        manifold.relative_contact_points_on2.append(vec3(in.points2[i]));
    }

    const result = zolt.estimateCollisionResponse(bodies[0], bodies[1], &manifold, in.combined_friction, in.combined_restitution, .{ .min_velocity_for_restitution = in.min_velocity_for_restitution, .num_iterations = in.num_iterations });

    s.u(marker_estimate);
    s.v3(result.linear_velocity1);
    s.v3(result.angular_velocity1);
    s.v3(result.linear_velocity2);
    s.v3(result.angular_velocity2);
    s.v3(result.friction_point);
    s.v3(result.friction_impulse);
    s.f(result.angular_friction_impulse);
    s.u(result.contact_impulse.len);
    for (result.contact_impulse.constSlice()) |impulse| s.f(impulse);

    // The inputs of the estimate that the bodies compute
    for (bodies) |body| {
        s.r3(body.getCenterOfMassPosition());
        if (body.getMotionType() == .dynamic) {
            s.f(body.getMotionProperties().getInverseMass());
            const inv_inertia = body.getInverseInertia();
            for (0..3) |c| s.v3(inv_inertia.getColumn3(@intCast(c)));
        }
    }
}

test "NarrowPhaseQuery parity: EstimateCollisionResponse" {
    const allocator = std.testing.allocator;
    const buffer = try allocator.alloc(u32, 4096);
    defer allocator.free(buffer);

    const zolt_world = try ZoltWorld.create(allocator, null, &.{}, 4);
    defer zolt_world.destroy();
    const jolt_world = jolt.npq_world_create(null, &[_]NpqBody{}, 0, 4);
    defer jolt.npq_world_destroy(jolt_world);

    var gen: Gen = .{ .rng = .{ .state = 0x3e57a7e } };
    var cmp: pw.Comparison = .{ .name = "EstimateCollisionResponse parity" };
    var s: Stream = .{ .allocator = allocator };
    defer s.deinit();
    const iterations: usize = if (builtin.mode == .Debug) 20000 else 100000;
    var num_moving_pairs: usize = 0;
    for (0..iterations) |_| {
        const in = randomEstimate(&gen);
        s.values.clearRetainingCapacity();
        try zoltEstimate(zolt_world, &in, &s);
        const jolt_values = pw.joltStream(buffer, struct {
            w: *anyopaque,
            in: *const NpqEstimateInput,
            pub fn run(c: @This(), out: [*]u32, capacity: u32) u32 {
                return jolt.npq_estimate(c.w, c.in, out, capacity);
            }
        }{ .w = jolt_world, .in = &in });
        cmp.check("estimate", .{ in.num_points, in.num_iterations, in.body1.motion_type, in.body2.motion_type }, s.values.items, jolt_values);
        num_moving_pairs += @intFromBool(in.body1.motion_type == @intFromEnum(MotionType.dynamic) and in.body2.motion_type == @intFromEnum(MotionType.dynamic));
    }
    try std.testing.expect(num_moving_pairs > iterations / 20);
    try cmp.finish();
}
