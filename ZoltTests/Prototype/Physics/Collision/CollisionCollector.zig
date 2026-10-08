//! Port of: Jolt/Physics/Collision/CollisionCollector.h
//! Status: complete
//!
//! `CollisionCollector<ResultType, TraitsType>` is the comptime function `CollisionCollector(ResultType, Traits)`, a
//! pattern A root (D5): the vtable, the early out fraction and the context live in the base, so the hot accessors
//! (`getEarlyOutFraction`, `shouldEarlyOut`) are field reads and `addHit` is one virtual call, like in Jolt.
//! Implementations embed it as `base`; queries take `*CastRayCollector` etc. and callers pass `&collector.base`.
//!
//! `addHit` returns `void` like Jolt (D6). Collectors that allocate record an allocation failure, force an early out
//! and report it through their own `checkError()` (see CollisionCollectorImpl.zig).

const std = @import("std");
const zolt = @import("zolt");
const math = zolt.math;
const virtual = @import("../../Core/Virtual.zig");
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
        pub const Traits = TraitsType;

        pub const VTable = struct {
            /// If you want to reuse this collector, call reset()
            reset: *const fn (self: *Self) void,
            /// When running a query through the NarrowPhaseQuery class, this will be called for every body that is potentially colliding.
            /// It allows collecting additional information needed by the collision collector implementation from the body under lock protection
            /// before addHit is called (e.g. the user data pointer or the velocity of the body).
            onBody: *const fn (self: *Self, body: *const Body) void,
            /// When running a query through the NarrowPhaseQuery class, this will be called after all addHit calls have been made for a particular body.
            onBodyEnd: *const fn (self: *Self) void,
            /// This function can be used to set some user data on the collision collector
            setUserData: *const fn (self: *Self, user_data: u64) void,
            /// This function will be called for every hit found, it's up to the application to decide how to store the hit
            addHit: *const fn (self: *Self, result: *const ResultType) void,
        };

        vtable: *const VTable,

        /// The early out fraction determines the fraction below which the collector is still accepting a hit (can be used to reduce the amount of work)
        early_out_fraction: f32 = Traits.initial_early_out_fraction,

        /// Set by the collision detection functions to the current TransformedShape of the body that we're colliding against before calling the addHit function
        context: ?*const TransformedShape = null,

        /// Constructor (default constructor of the C++ base), called by the derived collector with its most derived type
        pub fn init(comptime T: type) Self {
            return .{ .vtable = vtableFor(T) };
        }

        /// Constructor to initialize from another collector (copies the early out fraction and the context)
        pub fn initFrom(comptime T: type, other: anytype) Self {
            comptime {
                if (@TypeOf(other.*).Traits != Traits) @compileError("initFrom needs a collector with the same traits");
            }
            return .{ .vtable = vtableFor(T), .early_out_fraction = other.getEarlyOutFraction(), .context = other.getContext() };
        }

        pub fn vtableFor(comptime T: type) *const VTable {
            return &struct {
                const vt = virtual.make(VTable, T);
            }.vt;
        }

        // Virtual dispatchers
        pub fn reset(self: *Self) void {
            self.vtable.reset(self);
        }

        pub fn onBody(self: *Self, body: *const Body) void {
            self.vtable.onBody(self, body);
        }

        pub fn onBodyEnd(self: *Self) void {
            self.vtable.onBodyEnd(self);
        }

        pub fn setUserData(self: *Self, user_data: u64) void {
            self.vtable.setUserData(self, user_data);
        }

        pub fn addHit(self: *Self, result: *const ResultType) void {
            self.vtable.addHit(self, result);
        }

        /// Set by the collision detection functions to the current TransformedShape that we're colliding against before calling the AddHit function.
        /// Note: Only valid during AddHit! For performance reasons, the pointer is not reset after leaving AddHit so the context may point to freed memory.
        pub fn setContext(self: *Self, context: ?*const TransformedShape) void {
            self.context = context;
        }

        pub fn getContext(self: *const Self) ?*const TransformedShape {
            return self.context;
        }

        /// Update the early out fraction (should be lower than before)
        pub fn updateEarlyOutFraction(self: *Self, fraction: f32) void {
            std.debug.assert(fraction <= self.early_out_fraction);
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

        /// Default implementations of the virtual functions (addHit is pure virtual)
        pub const impl = struct {
            pub fn reset(self: *Self) void {
                self.early_out_fraction = Traits.initial_early_out_fraction;
            }

            pub fn onBody(self: *Self, body: *const Body) void {
                // Collects nothing by default
                _ = self;
                _ = body;
            }

            pub fn onBodyEnd(self: *Self) void {
                // Does nothing by default
                _ = self;
            }

            pub fn setUserData(self: *Self, user_data: u64) void {
                // Does nothing by default
                _ = self;
                _ = user_data;
            }
        };
    };
}
