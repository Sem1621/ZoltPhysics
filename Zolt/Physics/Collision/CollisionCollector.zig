//! Port of: Jolt/Physics/Collision/CollisionCollector.h
//! Status: complete
//!
//! `CollisionCollector<ResultType, TraitsType>` is the comptime function `CollisionCollector(ResultType, Traits)`, a
//! pattern A root (Docs/Zolt/CollisionArchitecture.md, D5): the vtable, the early out fraction and the context live
//! in the base, so the hot accessors (`getEarlyOutFraction`, `shouldEarlyOut`) are field reads and `addHit` is one
//! virtual call, like in Jolt. Implementations embed it as `base` and declare `pub const overrides = .{ ... }`;
//! queries take `*CastRayCollector` etc. and callers pass `&collector.base`.
//!
//! - `init(T)` is the default constructor of the base class (T is the most derived class, its vtable is used),
//!   `initFrom(T, other)` the explicit constructor that copies the early out fraction and the context of another
//!   collector with the same traits (`CollisionCollector(const CollisionCollector<ResultTypeArg2, TraitsType> &)`).
//! - `virtual ~CollisionCollector()`: collectors are never destroyed through a base pointer in Jolt, so there is no
//!   virtual destructor entry. Implementations that own memory have their own (non virtual) `deinit`.
//! - `addHit` returns `void` like Jolt (D6). Collectors that allocate record an allocation failure, force an early out
//!   and report it through their own `checkError()` (see CollisionCollectorImpl.zig).

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const math = @import("../../Math/Math.zig");
const Body = @import("../Body/Body.zig").Body;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;

/// Traits to use for CastRay
pub const CollisionCollectorTraitsCastRay = struct {
    /// For rays the early out fraction is the fraction along the line to order hits.
    pub const initial_early_out_fraction: f32 = 1.0 + math.flt_epsilon; // Furthest hit: Fraction is 1 + epsilon
    pub const should_early_out_fraction: f32 = 0.0; // Closest hit: Fraction is 0
};

/// Traits to use for CastShape
pub const CollisionCollectorTraitsCastShape = struct {
    /// For rays the early out fraction is the fraction along the line to order hits.
    pub const initial_early_out_fraction: f32 = 1.0 + math.flt_epsilon; // Furthest hit: Fraction is 1 + epsilon
    pub const should_early_out_fraction: f32 = -math.flt_max; // Deepest hit: Penetration is infinite
};

/// Traits to use for CollideShape
pub const CollisionCollectorTraitsCollideShape = struct {
    /// For shape collisions we use -penetration depth to order hits.
    pub const initial_early_out_fraction: f32 = math.flt_max; // Most shallow hit: Separation is infinite
    pub const should_early_out_fraction: f32 = -math.flt_max; // Deepest hit: Penetration is infinite
};

/// Traits to use for CollidePoint
pub const CollisionCollectorTraitsCollidePoint = CollisionCollectorTraitsCollideShape;

/// Virtual interface that allows collecting multiple collision results
pub fn CollisionCollector(comptime ResultTypeArg: type, comptime TraitsType: type) type {
    return struct {
        const Self = @This();

        /// Declare ResultType so that derived classes can use it
        pub const ResultType = ResultTypeArg;

        /// The traits of this collector (TraitsType)
        pub const Traits = TraitsType;

        /// One entry per C++ virtual function, in declaration order
        pub const VTable = struct {
            /// If you want to reuse this collector, call Reset()
            reset: *const fn (self: *Self) void,

            /// When running a query through the NarrowPhaseQuery class, this will be called for every body that is potentially colliding.
            /// It allows collecting additional information needed by the collision collector implementation from the body under lock protection
            /// before AddHit is called (e.g. the user data pointer or the velocity of the body).
            onBody: *const fn (self: *Self, body: *const Body) void,

            /// When running a query through the NarrowPhaseQuery class, this will be called after all AddHit calls have been made for a particular body.
            onBodyEnd: *const fn (self: *Self) void,

            /// This function can be used to set some user data on the collision collector
            setUserData: *const fn (self: *Self, user_data: u64) void,

            /// This function will be called for every hit found, it's up to the application to decide how to store the hit
            addHit: *const fn (self: *Self, result: *const ResultType) void,
        };

        vtable: *const VTable,

        /// The early out fraction determines the fraction below which the collector is still accepting a hit (can be used to reduce the amount of work)
        early_out_fraction: f32 = Traits.initial_early_out_fraction,

        /// Set by the collision detection functions to the current TransformedShape of the body that we're colliding against before calling the AddHit function
        context: ?*const TransformedShape = null,

        /// Default constructor, called by the derived collector with its most derived type `T`
        pub fn init(comptime T: type) Self {
            return .{ .vtable = vtableFor(T) };
        }

        /// Constructor to initialize from another collector (with the same traits, any result type)
        pub fn initFrom(comptime T: type, other: anytype) Self {
            comptime {
                if (@typeInfo(@TypeOf(other)).pointer.child.Traits != Traits) @compileError("initFrom needs a collector with the same traits");
            }
            return .{ .vtable = vtableFor(T), .early_out_fraction = other.getEarlyOutFraction(), .context = other.getContext() };
        }

        /// The vtable of the concrete collector class `T`
        pub fn vtableFor(comptime T: type) *const VTable {
            return virtual.vtablePtr(VTable, T);
        }

        /// If you want to reuse this collector, call Reset()
        pub fn reset(self: *Self) void {
            self.vtable.reset(self);
        }

        /// When running a query through the NarrowPhaseQuery class, this will be called for every body that is potentially colliding.
        /// It allows collecting additional information needed by the collision collector implementation from the body under lock protection
        /// before AddHit is called (e.g. the user data pointer or the velocity of the body).
        pub fn onBody(self: *Self, body: *const Body) void {
            self.vtable.onBody(self, body);
        }

        /// When running a query through the NarrowPhaseQuery class, this will be called after all AddHit calls have been made for a particular body.
        pub fn onBodyEnd(self: *Self) void {
            self.vtable.onBodyEnd(self);
        }

        /// Set by the collision detection functions to the current TransformedShape that we're colliding against before calling the AddHit function.
        /// Note: Only valid during AddHit! For performance reasons, the pointer is not reset after leaving AddHit so the context may point to freed memory.
        pub fn setContext(self: *Self, context: ?*const TransformedShape) void {
            self.context = context;
        }

        pub fn getContext(self: *const Self) ?*const TransformedShape {
            return self.context;
        }

        /// This function can be used to set some user data on the collision collector
        pub fn setUserData(self: *Self, user_data: u64) void {
            self.vtable.setUserData(self, user_data);
        }

        /// This function will be called for every hit found, it's up to the application to decide how to store the hit
        pub fn addHit(self: *Self, result: *const ResultType) void {
            self.vtable.addHit(self, result);
        }

        /// Update the early out fraction (should be lower than before)
        pub fn updateEarlyOutFraction(self: *Self, fraction: f32) void {
            if (Core.enable_asserts) std.debug.assert(fraction <= self.early_out_fraction); // Degenerate (NaN) hits can violate this, Jolt's release build continues
            self.early_out_fraction = fraction;
        }

        /// Reset the early out fraction to a specific value
        pub fn resetEarlyOutFraction(self: *Self, opts: struct { fraction: f32 = Traits.initial_early_out_fraction }) void {
            self.early_out_fraction = opts.fraction;
        }

        /// Force the collision detection algorithm to terminate as soon as possible. Call this from the AddHit function when a satisfying hit is found.
        pub fn forceEarlyOut(self: *Self) void {
            self.early_out_fraction = Traits.should_early_out_fraction;
        }

        /// When true, the collector will no longer accept any additional hits and the collision detection routine should early out as soon as possible
        pub fn shouldEarlyOut(self: *const Self) bool {
            return self.early_out_fraction <= Traits.should_early_out_fraction;
        }

        /// Get the current early out value
        pub fn getEarlyOutFraction(self: *const Self) f32 {
            return self.early_out_fraction;
        }

        /// Get the current early out value but make sure it's bigger than zero, this is used for shape casting as negative values are used for penetration
        pub fn getPositiveEarlyOutFraction(self: *const Self) f32 {
            return math.max(math.flt_min, self.early_out_fraction);
        }

        /// Implementations of the virtual functions in CollisionCollector (AddHit is pure virtual)
        pub const impl = struct {
            pub fn reset(self: *Self) void {
                self.early_out_fraction = Traits.initial_early_out_fraction;
            }

            pub fn onBody(self: *Self, body: *const Body) void {
                // Collects nothing by default
                _ = .{ self, body };
            }

            pub fn onBodyEnd(self: *Self) void {
                // Does nothing by default
                _ = self;
            }

            pub fn setUserData(self: *Self, user_data: u64) void {
                // Does nothing by default
                _ = .{ self, user_data };
            }
        };
    };
}

test "CollisionCollector: base class behavior, traits and initFrom" {
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;
    const RayCastResult = @import("CastResult.zig").RayCastResult;
    const CollidePointResult = @import("CollidePointResult.zig").CollidePointResult;
    const CastRayCollector = CollisionCollector(RayCastResult, CollisionCollectorTraitsCastRay);
    const CollidePointCollector = CollisionCollector(CollidePointResult, CollisionCollectorTraitsCollidePoint);

    // A collector that only implements AddHit (the pure virtual function), everything else is inherited
    const CountingCollector = struct {
        pub const overrides = .{.addHit};

        base: CastRayCollector = .init(@This()),
        count: u32 = 0,

        pub fn addHit(self: *@This(), result: *const RayCastResult) void {
            self.count += 1;
            self.base.updateEarlyOutFraction(result.fraction);
        }
    };

    var collector: CountingCollector = .{};
    const base = &collector.base;
    try expectEqual(CollisionCollectorTraitsCastRay.initial_early_out_fraction, base.getEarlyOutFraction());
    try expect(!base.shouldEarlyOut());
    base.addHit(&.{ .fraction = 0.5 });
    try expectEqual(@as(u32, 1), collector.count);
    try expectEqual(@as(f32, 0.5), base.getEarlyOutFraction());
    try expectEqual(@as(f32, 0.5), base.getPositiveEarlyOutFraction());

    // Default virtual implementations do nothing (onBody, onBodyEnd, setUserData) or reset the fraction
    const body: Body = .{};
    base.onBody(&body);
    base.onBodyEnd();
    base.setUserData(5);
    try expectEqual(@as(f32, 0.5), base.getEarlyOutFraction());
    base.reset();
    try expectEqual(CollisionCollectorTraitsCastRay.initial_early_out_fraction, base.getEarlyOutFraction());

    // Early out
    base.forceEarlyOut();
    try expect(base.shouldEarlyOut());
    try expectEqual(math.flt_min, base.getPositiveEarlyOutFraction());
    base.resetEarlyOutFraction(.{ .fraction = 0.75 });
    try expectEqual(@as(f32, 0.75), base.getEarlyOutFraction());
    base.resetEarlyOutFraction(.{});
    try expectEqual(CollisionCollectorTraitsCastRay.initial_early_out_fraction, base.getEarlyOutFraction());

    // Context and initFrom (copies the fraction and the context, not the vtable)
    const ts: TransformedShape = .{};
    base.setContext(&ts);
    base.updateEarlyOutFraction(0.25);
    const Other = struct {
        pub const overrides = .{.addHit};
        base: CastRayCollector,
        pub fn addHit(self: *@This(), result: *const RayCastResult) void {
            _ = .{ self, result };
        }
    };
    const other: Other = .{ .base = .initFrom(Other, base) };
    try expect(other.base.getContext() == &ts);
    try expectEqual(@as(f32, 0.25), other.base.getEarlyOutFraction());
    try expect(other.base.vtable != base.vtable);

    // Collide point traits are the collide shape traits
    try expect(CollidePointCollector.Traits == CollisionCollectorTraitsCollideShape);
    try expectEqual(math.flt_max, CollisionCollectorTraitsCollidePoint.initial_early_out_fraction);
    try expectEqual(-math.flt_max, CollisionCollectorTraitsCastShape.should_early_out_fraction);
    try expectEqual(@as(f32, 0.0), CollisionCollectorTraitsCastRay.should_early_out_fraction);
}
