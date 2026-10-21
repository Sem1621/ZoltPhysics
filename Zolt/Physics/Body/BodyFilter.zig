//! Port of: Jolt/Physics/Body/BodyFilter.h
//! Status: complete
//!
//! Pattern A filters (CollisionArchitecture D7), like ShapeFilter: `BodyFilter{}` (or `&.{}`) is Jolt's default
//! filter that accepts every body, a derived filter embeds `base: BodyFilter = .init(@This())` and lists its
//! `overrides`. The virtual functions are const; filters are never destroyed through a base pointer, so there is no
//! destructor entry.
//! - `IgnoreMultipleBodiesFilter` owns its `Array<BodyID>`: `init(allocator)` / `deinit()`, and `reserve` /
//!   `ignoreBody` return `Allocator.Error`.
//! - `IgnoreSingleBodyFilterChained` keeps Jolt's `const BodyFilter &` as a `*const BodyFilter`.
//! - `BodyDrawFilter` (JPH_DEBUG_RENDERER) is a gated declaration (RemainingFeaturesArchitecture.md R2): `void` without
//!   `-Ddebug_renderer`. It is a pattern A class like BodyFilter: `BodyDrawFilter{}` draws every body.

const std = @import("std");
const Allocator = std.mem.Allocator;
const virtual = @import("../../Core/Virtual.zig");
const Core = @import("../../Core/Core.zig");
const BodyID = @import("BodyID.zig").BodyID;
const Body = @import("Body.zig").Body;

/// Class function to filter out bodies, returns true if test should collide with body
pub const BodyFilter = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Filter function. Returns true if we should collide with body_id
        shouldCollide: *const fn (self: *const BodyFilter, body_id: BodyID) bool,

        /// Filter function. Returns true if we should collide with body (this is called after the body is locked and makes it possible to filter based on body members)
        shouldCollideLocked: *const fn (self: *const BodyFilter, body: *const Body) bool,
    };

    vtable: *const VTable = vtableFor(BodyFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) BodyFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Filter function. Returns true if we should collide with body_id
    pub fn shouldCollide(self: *const BodyFilter, body_id: BodyID) bool {
        return self.vtable.shouldCollide(self, body_id);
    }

    /// Filter function. Returns true if we should collide with body (this is called after the body is locked and makes it possible to filter based on body members)
    pub fn shouldCollideLocked(self: *const BodyFilter, body: *const Body) bool {
        return self.vtable.shouldCollideLocked(self, body);
    }

    /// Implementations of the virtual functions in BodyFilter: accept everything
    pub const impl = struct {
        pub fn shouldCollide(self: *const BodyFilter, body_id: BodyID) bool {
            _ = .{ self, body_id };
            return true;
        }

        pub fn shouldCollideLocked(self: *const BodyFilter, body: *const Body) bool {
            _ = .{ self, body };
            return true;
        }
    };
};

/// A simple body filter implementation that ignores a single, specified body
pub const IgnoreSingleBodyFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: BodyFilter,
    body_id: BodyID,

    /// Constructor, pass the body you want to ignore
    pub fn init(body_id: BodyID) IgnoreSingleBodyFilter {
        return .{ .base = .init(IgnoreSingleBodyFilter), .body_id = body_id };
    }

    /// Filter function. Returns true if we should collide with body_id
    pub fn shouldCollide(self: *const IgnoreSingleBodyFilter, body_id: BodyID) bool {
        return !self.body_id.eql(body_id);
    }
};

/// A simple body filter implementation that ignores multiple, specified bodies
pub const IgnoreMultipleBodiesFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: BodyFilter,
    allocator: Allocator,
    body_ids: std.ArrayList(BodyID) = .empty,

    /// Constructor (the allocator owns the list of body IDs)
    pub fn init(allocator: Allocator) IgnoreMultipleBodiesFilter {
        return .{ .base = .init(IgnoreMultipleBodiesFilter), .allocator = allocator };
    }

    /// Destructor
    pub fn deinit(self: *IgnoreMultipleBodiesFilter) void {
        self.body_ids.deinit(self.allocator);
    }

    /// Remove all bodies from the filter
    pub fn clear(self: *IgnoreMultipleBodiesFilter) void {
        self.body_ids.clearRetainingCapacity();
    }

    /// Reserve space for size body ID's
    pub fn reserve(self: *IgnoreMultipleBodiesFilter, size: u32) Allocator.Error!void {
        try self.body_ids.ensureTotalCapacity(self.allocator, size);
    }

    /// Add a body to be ignored
    pub fn ignoreBody(self: *IgnoreMultipleBodiesFilter, body_id: BodyID) Allocator.Error!void {
        try self.body_ids.append(self.allocator, body_id);
    }

    /// Filter function. Returns true if we should collide with body_id
    pub fn shouldCollide(self: *const IgnoreMultipleBodiesFilter, body_id: BodyID) bool {
        for (self.body_ids.items) |id| {
            if (id.eql(body_id))
                return false;
        }
        return true;
    }
};

/// Ignores a single body and chains the filter to another filter
pub const IgnoreSingleBodyFilterChained = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollideLocked };

    base: BodyFilter,
    body_id: BodyID,
    filter: *const BodyFilter,

    /// Constructor
    pub fn init(body_id: BodyID, filter: *const BodyFilter) IgnoreSingleBodyFilterChained {
        return .{ .base = .init(IgnoreSingleBodyFilterChained), .body_id = body_id, .filter = filter };
    }

    /// Filter function. Returns true if we should collide with body_id
    pub fn shouldCollide(self: *const IgnoreSingleBodyFilterChained, body_id: BodyID) bool {
        return !body_id.eql(self.body_id) and self.filter.shouldCollide(body_id);
    }

    /// Filter function. Returns true if we should collide with body (this is called after the body is locked and makes it possible to filter based on body members)
    pub fn shouldCollideLocked(self: *const IgnoreSingleBodyFilterChained, body: *const Body) bool {
        return self.filter.shouldCollideLocked(body);
    }
};

/// Class function to filter out bodies for debug rendering, returns true if body should be rendered
/// (JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)
pub const BodyDrawFilter = if (Core.debug_renderer) BodyDrawFilterClass else void;

/// The class of BodyDrawFilter (only analyzed with -Ddebug_renderer). Jolt's virtual destructor is not ported: filters
/// are never destroyed through a base pointer, like BodyFilter.
const BodyDrawFilterClass = struct {
    /// One entry per C++ virtual function
    pub const VTable = struct {
        /// Filter function. Returns true if inBody should be rendered
        shouldDraw: *const fn (self: *const BodyDrawFilterClass, body: *const Body) bool,
    };

    vtable: *const VTable = vtableFor(BodyDrawFilterClass),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) BodyDrawFilterClass {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Filter function. Returns true if inBody should be rendered
    pub fn shouldDraw(self: *const BodyDrawFilterClass, body: *const Body) bool {
        return self.vtable.shouldDraw(self, body);
    }

    /// Implementation of the virtual function: draw every body
    pub const impl = struct {
        pub fn shouldDraw(self: *const BodyDrawFilterClass, body: *const Body) bool {
            _ = .{ self, body };
            return true;
        }
    };
};

test "BodyFilter: default, ignore single, ignore multiple, chained" {
    const testing = std.testing;
    const id1 = BodyID.init(1);
    const id2 = BodyID.init(2);
    const id3 = BodyID.init(3);
    const body: Body = .{ .id = id2 };

    const default_filter: BodyFilter = .{};
    try testing.expect(default_filter.shouldCollide(id1));
    try testing.expect(default_filter.shouldCollideLocked(&body));

    const single: IgnoreSingleBodyFilter = .init(id1);
    try testing.expect(!single.base.shouldCollide(id1));
    try testing.expect(single.base.shouldCollide(id2));
    try testing.expect(single.base.shouldCollideLocked(&body));

    var multiple: IgnoreMultipleBodiesFilter = .init(testing.allocator);
    defer multiple.deinit();
    try multiple.reserve(4);
    try multiple.ignoreBody(id1);
    try multiple.ignoreBody(id3);
    try testing.expect(!multiple.base.shouldCollide(id1));
    try testing.expect(multiple.base.shouldCollide(id2));
    try testing.expect(!multiple.base.shouldCollide(id3));
    multiple.clear();
    try testing.expect(multiple.base.shouldCollide(id1));

    // A filter that rejects body 2 when locked, chained behind one that ignores body 3
    const RejectLocked = struct {
        pub const overrides = .{.shouldCollideLocked};
        base: BodyFilter = .init(@This()),
        pub fn shouldCollideLocked(self: *const @This(), b: *const Body) bool {
            _ = self;
            return !b.getID().eql(BodyID.init(2));
        }
    };
    const reject: RejectLocked = .{};
    const chained: IgnoreSingleBodyFilterChained = .init(id3, &single.base);
    try testing.expect(!chained.base.shouldCollide(id3));
    try testing.expect(!chained.base.shouldCollide(id1)); // Rejected by the chained filter
    try testing.expect(chained.base.shouldCollide(id2));
    const chained2: IgnoreSingleBodyFilterChained = .init(id3, &reject.base);
    try testing.expect(!chained2.base.shouldCollideLocked(&body));
}
