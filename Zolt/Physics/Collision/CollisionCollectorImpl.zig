//! Port of: Jolt/Physics/Collision/CollisionCollectorImpl.h
//! Status: complete
//!
//! `template <class CollectorType> class XCollisionCollector : public CollectorType` is the comptime function
//! `XCollisionCollector(CollectorType)` whose result embeds `base: CollectorType` (Docs/Zolt/CollisionArchitecture.md,
//! D5). Queries take the base (`&collector.base`).
//! - `init(...)` constructs the collector itself, `initDerived(T, ...)` is the constructor for a class that derives
//!   from it (it builds the vtable of the most derived class T, which inherits these overrides like in C++).
//! - Results that hold references (TransformedShape: `clone` / `deinit`) are copied with `clone()` when stored and
//!   released when overwritten, on `reset()` and on `deinit()` (C++ copy constructor, assignment and destructor).
//!   C++ `ClosestHitCollisionCollector::Reset` keeps `mHit` until it is overwritten; Zolt releases a stored reference
//!   on `reset` (the value of a result without references is kept, like in C++).
//! - Out of memory (D6): `addHit` stays `void`. A collector that cannot store a hit records the error in an
//!   `AllocationErrorLatch` (never overwritten until `reset`), forces an early out so the query stops as soon as
//!   possible, and the owner calls `try collector.checkError()` after the query. In safe builds `reset()` /
//!   `deinit()` assert that a recorded error was observed through `checkError()`, so a forgotten check fails loudly
//!   in tests. For ClosestHitPerBody, `had_hit` stays false when storing the first hit of a body fails, so `onBodyEnd`
//!   does not restore the early out fraction and the query stops.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const math = @import("../../Math/Math.zig");
const Body = @import("../Body/Body.zig").Body;

/// Copy a result: adds a reference when the result owns one (C++ copy constructor)
fn copyResult(comptime R: type, result: *const R) R {
    return if (@hasDecl(R, "clone")) result.clone() else result.*;
}

/// Release a stored result (C++ destructor)
fn releaseResult(comptime R: type, result: *R) void {
    if (@hasDecl(R, "deinit")) result.deinit();
}

/// `[](const ResultType &inLHS, const ResultType &inRHS) { return inLHS.GetEarlyOutFraction() < inRHS.GetEarlyOutFraction(); }`
fn lessThanEarlyOut(comptime R: type) fn (void, R, R) bool {
    return struct {
        fn f(_: void, lhs: R, rhs: R) bool {
            return lhs.getEarlyOutFraction() < rhs.getEarlyOutFraction();
        }
    }.f;
}

/// Records an allocation failure inside `addHit` until the owner observes it (D6, Zolt addition)
pub const AllocationErrorLatch = struct {
    /// The first error that was recorded since the last clear
    err: ?Allocator.Error = null,
    /// False while an error was recorded and not returned by `check` yet
    observed: bool = true,

    /// Record an error (the first one wins)
    pub fn set(self: *AllocationErrorLatch, err: Allocator.Error) void {
        if (self.err == null) {
            self.err = err;
            self.observed = false;
        }
    }

    /// Return the recorded error (marks it as observed)
    pub fn check(self: *AllocationErrorLatch) Allocator.Error!void {
        self.observed = true;
        if (self.err) |err| return err;
    }

    /// Called by reset / deinit: a recorded error must have been observed (checked in safe builds)
    pub fn clear(self: *AllocationErrorLatch) void {
        if (Core.enable_asserts) std.debug.assert(self.observed); // An allocation failed in addHit and nobody called checkError()
        self.* = .{};
    }
};

/// Simple implementation that collects all hits and optionally sorts them on distance
pub fn AllHitCollisionCollector(comptime CollectorType: type) type {
    return struct {
        const Self = @This();

        /// Redeclare ResultType
        pub const ResultType = CollectorType.ResultType;

        pub const overrides = .{ .reset, .addHit };

        base: CollectorType,
        /// Allocator of `hits` (Jolt's Array uses the global allocator)
        allocator: Allocator,
        hits: std.ArrayList(ResultType) = .empty,
        /// Allocation failure in addHit (D6)
        alloc_error: AllocationErrorLatch = .{},

        /// Constructor
        pub fn init(allocator: Allocator) Self {
            return initDerived(Self, allocator);
        }

        /// Constructor for classes that derive from this collector (T is the most derived class)
        pub fn initDerived(comptime T: type, allocator: Allocator) Self {
            return .{ .base = .init(T), .allocator = allocator };
        }

        /// Destructor
        pub fn deinit(self: *Self) void {
            self.alloc_error.clear();
            self.clearHits();
            self.hits.deinit(self.allocator);
        }

        /// Returns error.OutOfMemory if a hit could not be stored (the query was stopped early)
        pub fn checkError(self: *Self) Allocator.Error!void {
            return self.alloc_error.check();
        }

        // See: CollectorType::Reset
        pub fn reset(self: *Self) void {
            CollectorType.impl.reset(&self.base);

            self.clearHits();
            self.alloc_error.clear();
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *Self, result: *const ResultType) void {
            var hit = copyResult(ResultType, result);
            self.hits.append(self.allocator, hit) catch |err| {
                releaseResult(ResultType, &hit);
                self.alloc_error.set(err);
                self.base.forceEarlyOut();
            };
        }

        /// Order hits on closest first
        pub fn sort(self: *Self) void {
            quickSort(ResultType, self.hits.items, {}, lessThanEarlyOut(ResultType));
        }

        /// Check if any hits were collected
        pub fn hadHit(self: *const Self) bool {
            return self.hits.items.len != 0;
        }

        /// mHits.clear()
        fn clearHits(self: *Self) void {
            for (self.hits.items) |*hit| releaseResult(ResultType, hit);
            self.hits.clearRetainingCapacity();
        }
    };
}

/// Simple implementation that collects the closest / deepest hit
pub fn ClosestHitCollisionCollector(comptime CollectorType: type) type {
    return struct {
        const Self = @This();

        /// Redeclare ResultType
        pub const ResultType = CollectorType.ResultType;

        pub const overrides = .{ .reset, .addHit };

        base: CollectorType,
        hit: ResultType = .{},
        had_hit: bool = false,

        /// Constructor
        pub fn init() Self {
            return initDerived(Self);
        }

        /// Constructor for classes that derive from this collector (T is the most derived class)
        pub fn initDerived(comptime T: type) Self {
            return .{ .base = .init(T) };
        }

        /// Destructor (releases a stored reference)
        pub fn deinit(self: *Self) void {
            if (self.had_hit) releaseResult(ResultType, &self.hit);
            self.had_hit = false;
        }

        // See: CollectorType::Reset
        pub fn reset(self: *Self) void {
            CollectorType.impl.reset(&self.base);

            if (self.had_hit) releaseResult(ResultType, &self.hit);
            self.had_hit = false;
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *Self, result: *const ResultType) void {
            const early_out = result.getEarlyOutFraction();
            if (!self.had_hit or early_out < self.hit.getEarlyOutFraction()) {
                // Update early out fraction
                self.base.updateEarlyOutFraction(early_out);

                // Store hit
                const copy = copyResult(ResultType, result); // Copy first: `result` may be the stored hit
                if (self.had_hit) releaseResult(ResultType, &self.hit);
                self.hit = copy;
                self.had_hit = true;
            }
        }

        /// Check if this collector has had a hit
        pub fn hadHit(self: *const Self) bool {
            return self.had_hit;
        }
    };
}

/// Implementation that collects the closest / deepest hit for each body and optionally sorts them on distance
pub fn ClosestHitPerBodyCollisionCollector(comptime CollectorType: type) type {
    return struct {
        const Self = @This();

        /// Redeclare ResultType
        pub const ResultType = CollectorType.ResultType;

        pub const overrides = .{ .reset, .onBody, .addHit, .onBodyEnd };

        base: CollectorType,
        /// Allocator of `hits` (Jolt's Array uses the global allocator)
        allocator: Allocator,
        hits: std.ArrayList(ResultType) = .empty,
        /// Allocation failure in addHit (D6)
        alloc_error: AllocationErrorLatch = .{},

        /// Store early out fraction that was initially configured for the collector
        previous_early_out_fraction: f32 = -math.flt_max,

        /// Flag to indicate if we have a hit for the current body
        had_hit: bool = false,

        /// Constructor
        pub fn init(allocator: Allocator) Self {
            return initDerived(Self, allocator);
        }

        /// Constructor for classes that derive from this collector (T is the most derived class)
        pub fn initDerived(comptime T: type, allocator: Allocator) Self {
            return .{ .base = .init(T), .allocator = allocator };
        }

        /// Destructor
        pub fn deinit(self: *Self) void {
            self.alloc_error.clear();
            self.clearHits();
            self.hits.deinit(self.allocator);
        }

        /// Returns error.OutOfMemory if a hit could not be stored (the query was stopped early)
        pub fn checkError(self: *Self) Allocator.Error!void {
            return self.alloc_error.check();
        }

        // See: CollectorType::Reset
        pub fn reset(self: *Self) void {
            CollectorType.impl.reset(&self.base);

            self.clearHits();
            self.had_hit = false;
            self.alloc_error.clear();
        }

        // See: CollectorType::OnBody
        pub fn onBody(self: *Self, body: *const Body) void {
            _ = body;

            // Store the early out fraction so we can restore it after we've collected all hits for this body
            self.previous_early_out_fraction = self.base.getEarlyOutFraction();
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *Self, result: *const ResultType) void {
            const early_out = result.getEarlyOutFraction();
            if (!self.had_hit or early_out < self.base.getEarlyOutFraction()) {
                // Update early out fraction to avoid spending work on collecting further hits for this body
                self.base.updateEarlyOutFraction(early_out);

                if (!self.had_hit) {
                    // First time we have a hit we append it to the array
                    var hit = copyResult(ResultType, result);
                    self.hits.append(self.allocator, hit) catch |err| {
                        // had_hit stays false, so onBodyEnd does not restore the early out fraction: the query stops
                        releaseResult(ResultType, &hit);
                        self.alloc_error.set(err);
                        self.base.forceEarlyOut();
                        return;
                    };
                    self.had_hit = true;
                } else {
                    // Closer hits will override the previous one
                    const back = &self.hits.items[self.hits.items.len - 1];
                    const copy = copyResult(ResultType, result); // Copy first: `result` may be the stored hit
                    releaseResult(ResultType, back);
                    back.* = copy;
                }
            }
        }

        // See: CollectorType::OnBodyEnd
        pub fn onBodyEnd(self: *Self) void {
            if (self.had_hit) {
                // Reset the early out fraction to the configured value so that we will continue
                // to collect hits at any distance for other bodies
                if (Core.enable_asserts) std.debug.assert(self.previous_early_out_fraction != -math.flt_max); // Check that we got a call to OnBody
                self.base.resetEarlyOutFraction(.{ .fraction = self.previous_early_out_fraction });
                self.had_hit = false;
            }

            // For asserting purposes we reset the stored early out fraction so we can detect that OnBody was called
            if (Core.enable_asserts) self.previous_early_out_fraction = -math.flt_max;
        }

        /// Order hits on closest first
        pub fn sort(self: *Self) void {
            quickSort(ResultType, self.hits.items, {}, lessThanEarlyOut(ResultType));
        }

        /// Check if any hits were collected
        pub fn hadHit(self: *const Self) bool {
            return self.hits.items.len != 0;
        }

        /// mHits.clear()
        fn clearHits(self: *Self) void {
            for (self.hits.items) |*hit| releaseResult(ResultType, hit);
            self.hits.clearRetainingCapacity();
        }
    };
}

/// Simple implementation that collects any hit
pub fn AnyHitCollisionCollector(comptime CollectorType: type) type {
    return struct {
        const Self = @This();

        /// Redeclare ResultType
        pub const ResultType = CollectorType.ResultType;

        pub const overrides = .{ .reset, .addHit };

        base: CollectorType,
        hit: ResultType = .{},
        had_hit: bool = false,

        /// Constructor
        pub fn init() Self {
            return initDerived(Self);
        }

        /// Constructor for classes that derive from this collector (T is the most derived class)
        pub fn initDerived(comptime T: type) Self {
            return .{ .base = .init(T) };
        }

        /// Destructor (releases a stored reference)
        pub fn deinit(self: *Self) void {
            if (self.had_hit) releaseResult(ResultType, &self.hit);
            self.had_hit = false;
        }

        // See: CollectorType::Reset
        pub fn reset(self: *Self) void {
            CollectorType.impl.reset(&self.base);

            if (self.had_hit) releaseResult(ResultType, &self.hit);
            self.had_hit = false;
        }

        // See: CollectorType::AddHit
        pub fn addHit(self: *Self, result: *const ResultType) void {
            // Test that the collector is not collecting more hits after forcing an early out
            // (InternalEdgeRemovingCollector::Flush can add several hits, Jolt's release build then keeps the last one)
            if (Core.enable_asserts) std.debug.assert(!self.had_hit);

            // Abort any further testing
            self.base.forceEarlyOut();

            // Store hit
            const copy = copyResult(ResultType, result); // Copy first: `result` may be the stored hit
            if (self.had_hit) releaseResult(ResultType, &self.hit);
            self.hit = copy;
            self.had_hit = true;
        }

        /// Check if this collector has had a hit
        pub fn hadHit(self: *const Self) bool {
            return self.had_hit;
        }
    };
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const RefCount = @import("../../Core/Reference.zig").RefCount;
const Quat = @import("../../Math/Quat.zig").Quat;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const RayCastResult = @import("CastResult.zig").RayCastResult;
const CollideShapeResult = @import("CollideShape.zig").CollideShapeResult;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;
const TestBoxShape = @import("Shape/TestShapes.zig").TestBoxShape;

fn rayHit(body: u32, fraction: f32) RayCastResult {
    return .{ .body_id = .init(body), .fraction = fraction, .sub_shape_id2 = .{ .value = body } };
}

test "AllHitCollisionCollector: collects, sorts, resets" {
    const allocator = testing.allocator;

    var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer collector.deinit();
    try testing.expect(!collector.hadHit());
    const fractions = [_]f32{ 0.5, 0.25, 0.75, 0.25, 0.0 };
    for (fractions, 0..) |f, i| collector.base.addHit(&rayHit(@intCast(i), f));
    try collector.checkError();
    try testing.expect(collector.hadHit());
    try testing.expectEqual(@as(usize, 5), collector.hits.items.len);
    try testing.expectEqual(CastRayCollector.Traits.initial_early_out_fraction, collector.base.getEarlyOutFraction()); // AllHit never lowers it

    collector.sort();
    var previous: f32 = -1.0;
    for (collector.hits.items) |h| {
        try testing.expect(h.fraction >= previous);
        previous = h.fraction;
    }
    try testing.expectEqual(@as(u32, 4), collector.hits.items[0].body_id.getIndex());

    // Reset (virtual): clears the hits and the early out fraction
    collector.base.forceEarlyOut();
    collector.base.reset();
    try testing.expect(!collector.hadHit());
    try testing.expectEqual(CastRayCollector.Traits.initial_early_out_fraction, collector.base.getEarlyOutFraction());
}

test "ClosestHitCollisionCollector: keeps the closest hit and lowers the early out fraction" {
    var collector = ClosestHitCollisionCollector(CastRayCollector).init();
    defer collector.deinit();
    collector.base.addHit(&rayHit(1, 0.5));
    collector.base.addHit(&rayHit(2, 0.75)); // Further: in Jolt the query would not report it, the collector ignores it
    collector.base.addHit(&rayHit(3, 0.25));
    try testing.expect(collector.hadHit());
    try testing.expectEqual(@as(u32, 3), collector.hit.body_id.getIndex());
    try testing.expectEqual(@as(f32, 0.25), collector.base.getEarlyOutFraction());

    collector.base.reset();
    try testing.expect(!collector.hadHit());
    try testing.expectEqual(CastRayCollector.Traits.initial_early_out_fraction, collector.base.getEarlyOutFraction());
    collector.base.addHit(&rayHit(4, 0.9)); // The first hit after a reset is always taken
    try testing.expectEqual(@as(u32, 4), collector.hit.body_id.getIndex());

    // Collide shape results order on -penetration depth
    var deepest = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer deepest.deinit();
    for ([_]f32{ 0.1, 0.3, 0.2 }) |depth| deepest.base.addHit(&CollideShapeResult.init(Vec3.zero(), Vec3.zero(), Vec3.axisX(), depth, .empty, .empty, .invalid));
    try testing.expectEqual(@as(f32, 0.3), deepest.hit.penetration_depth);
    try testing.expectEqual(@as(f32, -0.3), deepest.base.getEarlyOutFraction());
}

test "ClosestHitPerBodyCollisionCollector: one hit per body, the early out fraction is restored per body" {
    const allocator = testing.allocator;

    var collector = ClosestHitPerBodyCollisionCollector(CastRayCollector).init(allocator);
    defer collector.deinit();
    const bodies = [_]Body{ .{ .id = .init(1) }, .{ .id = .init(2) }, .{ .id = .init(3) } };
    const hits = [_][]const f32{ &.{ 0.5, 0.3, 0.4 }, &.{}, &.{0.8} };
    for (&bodies, hits) |*body, body_hits| {
        collector.base.onBody(body);
        for (body_hits) |f| collector.base.addHit(&rayHit(body.id.getIndex(), f));
        collector.base.onBodyEnd();
        try testing.expectEqual(CastRayCollector.Traits.initial_early_out_fraction, collector.base.getEarlyOutFraction());
    }
    try collector.checkError();
    try testing.expectEqual(@as(usize, 2), collector.hits.items.len);
    try testing.expectEqual(@as(f32, 0.3), collector.hits.items[0].fraction);
    try testing.expectEqual(@as(f32, 0.8), collector.hits.items[1].fraction);

    // Within a body the early out fraction follows the closest hit
    collector.base.onBody(&bodies[0]);
    collector.base.addHit(&rayHit(1, 0.6));
    try testing.expectEqual(@as(f32, 0.6), collector.base.getEarlyOutFraction());
    collector.base.addHit(&rayHit(1, 0.1));
    try testing.expectEqual(@as(f32, 0.1), collector.base.getEarlyOutFraction());
    collector.base.onBodyEnd();
    try testing.expectEqual(@as(usize, 3), collector.hits.items.len);
    try testing.expectEqual(@as(f32, 0.1), collector.hits.items[2].fraction);

    collector.sort();
    try testing.expectEqual(@as(f32, 0.1), collector.hits.items[0].fraction);
    try testing.expectEqual(@as(f32, 0.8), collector.hits.items[2].fraction);

    collector.base.reset();
    try testing.expect(!collector.hadHit());
}

test "AnyHitCollisionCollector: stops at the first hit" {
    var collector = AnyHitCollisionCollector(CastRayCollector).init();
    defer collector.deinit();
    try testing.expect(!collector.base.shouldEarlyOut());
    collector.base.addHit(&rayHit(5, 0.5));
    try testing.expect(collector.hadHit() and collector.base.shouldEarlyOut());
    try testing.expectEqual(@as(u32, 5), collector.hit.body_id.getIndex());
    collector.base.reset();
    try testing.expect(!collector.hadHit() and !collector.base.shouldEarlyOut());
}

test "Collectors of results that hold references (TransformedShape) clone and release them" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    const box = try TestBoxShape.create(allocator, Vec3.one(), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const shape = box.asShape();

    var near = TransformedShape.init(RVec3.zero(), Quat.identity(), shape, .init(1), .{});
    defer near.deinit();
    var far = TransformedShape.init(RVec3.zero(), Quat.identity(), shape, .init(2), .{});
    defer far.deinit();
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());

    {
        var all = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer all.deinit();
        all.base.addHit(&near);
        all.base.addHit(&far);
        try all.checkError();
        try testing.expectEqual(@as(u32, 5), shape.getRefCount());
        all.base.reset();
        try testing.expectEqual(@as(u32, 3), shape.getRefCount());
        all.base.addHit(&near);
    }
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());

    {
        // TransformedShape.getEarlyOutFraction does not exist: the closest / per body collectors store the first hit
        var any = AnyHitCollisionCollector(TransformedShapeCollector).init();
        defer any.deinit();
        any.base.addHit(&near);
        try testing.expectEqual(@as(u32, 4), shape.getRefCount());
        try expect(any.hit.body_id.eql(.init(1)));
        any.base.reset();
        try testing.expectEqual(@as(u32, 3), shape.getRefCount());
        any.base.addHit(&far);
    }
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());
}

test "Collectors: out of memory in addHit is latched, forces an early out and is reported by checkError" {
    const allocator = testing.allocator;

    // The latch keeps the first error until it was observed
    var latch: AllocationErrorLatch = .{};
    try latch.check();
    latch.set(error.OutOfMemory);
    latch.set(error.OutOfMemory);
    try testing.expectError(error.OutOfMemory, latch.check());
    latch.clear();
    try latch.check();

    // AllHit: the hit is dropped, the collector forces an early out
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 1 });
    var all = AllHitCollisionCollector(CastRayCollector).init(failing.allocator());
    defer all.deinit();
    all.base.addHit(&rayHit(1, 0.5)); // Allocates the array (capacity for more than one hit)
    try all.checkError();
    var i: u32 = 0;
    while (!all.base.shouldEarlyOut()) : (i += 1) all.base.addHit(&rayHit(2 + i, 0.5));
    try testing.expectError(error.OutOfMemory, all.checkError());
    try testing.expect(all.hits.items.len >= 1);
    all.base.reset(); // The error was observed: reset clears it
    try all.checkError();
    try testing.expect(!all.base.shouldEarlyOut());

    // ClosestHitPerBody: the forced early out survives onBodyEnd (had_hit stays false)
    var failing2 = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var per_body = ClosestHitPerBodyCollisionCollector(CastRayCollector).init(failing2.allocator());
    defer per_body.deinit();
    const body: Body = .{ .id = .init(3) };
    per_body.base.onBody(&body);
    per_body.base.addHit(&rayHit(3, 0.5));
    per_body.base.onBodyEnd();
    try testing.expect(per_body.base.shouldEarlyOut());
    try testing.expect(!per_body.hadHit());
    try testing.expectError(error.OutOfMemory, per_body.checkError());

    // A collector of references releases the reference of a hit it could not store
    const box = try TestBoxShape.create(allocator, Vec3.one(), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    var ts = TransformedShape.init(RVec3.zero(), Quat.identity(), box.asShape(), .init(1), .{});
    defer ts.deinit();
    var failing3 = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var shapes = AllHitCollisionCollector(TransformedShapeCollector).init(failing3.allocator());
    defer shapes.deinit();
    shapes.base.addHit(&ts);
    try testing.expectError(error.OutOfMemory, shapes.checkError());
    try testing.expectEqual(@as(u32, 2), box.asShape().getRefCount());
}

test "Collectors: a class derived from ClosestHitPerBody inherits its overrides (initDerived)" {
    const allocator = testing.allocator;

    // Jolt's UnitTests derive from ClosestHitPerBodyCollisionCollector (CastShapeTests.cpp): the vtable is the one of
    // the derived class, the overrides of the parent (onBody / onBodyEnd / reset) are inherited
    const MyCollector = struct {
        pub const overrides = .{.addHit};

        base: ClosestHitPerBodyCollisionCollector(CastRayCollector),
        num_add_hit: u32 = 0,

        fn init(a: Allocator) @This() {
            return .{ .base = .initDerived(@This(), a) };
        }

        pub fn addHit(self: *@This(), result: *const RayCastResult) void {
            self.num_add_hit += 1;
            self.base.addHit(result); // C++ ClosestHitPerBodyCollisionCollector::AddHit(inResult)
        }
    };

    var collector = MyCollector.init(allocator);
    defer collector.base.deinit();
    const root: *CastRayCollector = &collector.base.base;
    const body: Body = .{ .id = .init(1) };
    root.onBody(&body);
    root.addHit(&rayHit(1, 0.5));
    root.addHit(&rayHit(1, 0.25));
    root.onBodyEnd();
    try collector.base.checkError();
    try testing.expectEqual(@as(u32, 2), collector.num_add_hit);
    try testing.expectEqual(@as(usize, 1), collector.base.hits.items.len);
    try testing.expectEqual(@as(f32, 0.25), collector.base.hits.items[0].fraction);
    try testing.expectEqual(CastRayCollector.Traits.initial_early_out_fraction, root.getEarlyOutFraction()); // Inherited onBodyEnd

    // The derived AllHit / ClosestHit / AnyHit constructors
    const DerivedAll = struct {
        pub const overrides = .{};
        base: AllHitCollisionCollector(CastRayCollector),
    };
    var derived_all: DerivedAll = .{ .base = .initDerived(DerivedAll, allocator) };
    defer derived_all.base.deinit();
    derived_all.base.base.addHit(&rayHit(1, 0.5));
    try testing.expectEqual(@as(usize, 1), derived_all.base.hits.items.len);
    const DerivedClosest = struct {
        pub const overrides = .{};
        base: ClosestHitCollisionCollector(CastRayCollector),
    };
    var derived_closest: DerivedClosest = .{ .base = .initDerived(DerivedClosest) };
    derived_closest.base.base.addHit(&rayHit(1, 0.5));
    try testing.expect(derived_closest.base.hadHit());
    const DerivedAny = struct {
        pub const overrides = .{};
        base: AnyHitCollisionCollector(CastRayCollector),
    };
    var derived_any: DerivedAny = .{ .base = .initDerived(DerivedAny) };
    derived_any.base.base.addHit(&rayHit(1, 0.5));
    try testing.expect(derived_any.base.hadHit());
}
