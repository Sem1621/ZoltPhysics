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
