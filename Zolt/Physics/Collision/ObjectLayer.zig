//! Port of: Jolt/Physics/Collision/ObjectLayer.h
//! Status: complete
//! Not ported: JPH_TRACK_BROADPHASE_STATS (ObjectLayerFilter::GetDescription)
//!
//! `ObjectLayer` is `u16` or `u32` depending on the build option `-Dobject_layer_bits` (JPH_OBJECT_LAYER_BITS).
//!
//! The filters are pattern A classes (Docs/Zolt/CollisionArchitecture.md, D1 and D7): the base class holds the
//! vtable, which defaults to the base class's own table, so `ObjectLayerFilter{}` (or `&.{}` for a
//! `*const ObjectLayerFilter` parameter) is Jolt's default constructed "accept everything" filter. A derived filter
//! embeds `base: ObjectLayerFilter = .init(@This())` and lists its overrides:
//! ```zig
//! const MyFilter = struct {
//!     pub const overrides = .{.shouldCollide};
//!     base: ObjectLayerFilter = .init(@This()),
//!     pub fn shouldCollide(self: *const MyFilter, layer: ObjectLayer) bool { ... }
//! };
//! var filter: MyFilter = .{};
//! query(..., &filter.base);
//! ```
//! The virtual functions are const (Rule M: state a filter must change lives behind a pointer field). The classes are
//! NonCopyable in Jolt and never deleted through a base pointer, so the vtables have no destructor entries. The
//! copy constructor of DefaultObjectLayerFilter is a plain Zig copy.

const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");

/// Layer that objects can be in, determines which other objects it can collide with
pub const ObjectLayer = switch (Core.object_layer_bits) {
    16 => u16,
    32 => u32,
    else => @compileError("JPH_OBJECT_LAYER_BITS must be 16 or 32"),
};

/// Constant value used to indicate an invalid object layer
pub const object_layer_invalid: ObjectLayer = ~@as(ObjectLayer, 0);

/// Filter class for object layers
pub const ObjectLayerFilter = struct {
    pub const VTable = struct {
        /// Function to filter out object layers when doing collision query test (return true to allow testing against objects with this layer)
        shouldCollide: *const fn (self: *const ObjectLayerFilter, layer: ObjectLayer) bool,
    };

    vtable: *const VTable = vtableFor(ObjectLayerFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) ObjectLayerFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Function to filter out object layers when doing collision query test (return true to allow testing against objects with this layer)
    pub fn shouldCollide(self: *const ObjectLayerFilter, layer: ObjectLayer) bool {
        return self.vtable.shouldCollide(self, layer);
    }

    /// Default implementations of the virtual functions
    pub const impl = struct {
        pub fn shouldCollide(self: *const ObjectLayerFilter, layer: ObjectLayer) bool {
            _ = self;
            _ = layer;
            return true;
        }
    };
};

/// Filter class to test if two objects can collide based on their object layer. Used while finding collision pairs.
pub const ObjectLayerPairFilter = struct {
    pub const VTable = struct {
        /// Returns true if two layers can collide
        shouldCollide: *const fn (self: *const ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) bool,
    };

    vtable: *const VTable = vtableFor(ObjectLayerPairFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) ObjectLayerPairFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Returns true if two layers can collide
    pub fn shouldCollide(self: *const ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) bool {
        return self.vtable.shouldCollide(self, layer1, layer2);
    }

    /// Default implementations of the virtual functions
    pub const impl = struct {
        pub fn shouldCollide(self: *const ObjectLayerPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) bool {
            _ = self;
            _ = layer1;
            _ = layer2;
            return true;
        }
    };
};

/// Default filter class that uses the pair filter in combination with a specified layer to filter layers
pub const DefaultObjectLayerFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerFilter = .init(DefaultObjectLayerFilter),
    object_layer_pair_filter: *const ObjectLayerPairFilter,
    layer: ObjectLayer,

    /// Constructor
    pub fn init(object_layer_pair_filter: *const ObjectLayerPairFilter, layer: ObjectLayer) DefaultObjectLayerFilter {
        return .{ .object_layer_pair_filter = object_layer_pair_filter, .layer = layer };
    }

    // See ObjectLayerFilter::ShouldCollide
    pub fn shouldCollide(self: *const DefaultObjectLayerFilter, layer: ObjectLayer) bool {
        return self.object_layer_pair_filter.shouldCollide(self.layer, layer);
    }
};

/// Allows objects from a specific layer only
pub const SpecifiedObjectLayerFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerFilter = .init(SpecifiedObjectLayerFilter),
    layer: ObjectLayer,

    /// Constructor
    pub fn init(layer: ObjectLayer) SpecifiedObjectLayerFilter {
        return .{ .layer = layer };
    }

    // See ObjectLayerFilter::ShouldCollide
    pub fn shouldCollide(self: *const SpecifiedObjectLayerFilter, layer: ObjectLayer) bool {
        return self.layer == layer;
    }
};

test "ObjectLayer filters" {
    const std = @import("std");
    const expect = std.testing.expect;

    try expect(@bitSizeOf(ObjectLayer) == Core.object_layer_bits);
    try expect(object_layer_invalid == std.math.maxInt(ObjectLayer));

    // Default constructed filters accept everything
    const all: ObjectLayerFilter = .{};
    try expect(all.shouldCollide(0) and all.shouldCollide(object_layer_invalid));
    const all_pairs: ObjectLayerPairFilter = .{};
    try expect(all_pairs.shouldCollide(1, 2));

    // A user pair filter: layers collide when they are equal or one of them is 0
    const PairFilter = struct {
        pub const overrides = .{.shouldCollide};
        base: ObjectLayerPairFilter = .init(@This()),

        pub fn shouldCollide(self: *const @This(), layer1: ObjectLayer, layer2: ObjectLayer) bool {
            _ = self;
            return layer1 == layer2 or layer1 == 0 or layer2 == 0;
        }
    };
    const pair_filter: PairFilter = .{};
    const default_filter = DefaultObjectLayerFilter.init(&pair_filter.base, 3);
    const copy = default_filter; // Copy constructor
    const filter: *const ObjectLayerFilter = &copy.base;
    try expect(filter.shouldCollide(3));
    try expect(filter.shouldCollide(0));
    try expect(!filter.shouldCollide(4));

    const specified = SpecifiedObjectLayerFilter.init(5);
    try expect(specified.base.shouldCollide(5));
    try expect(!specified.base.shouldCollide(6));
}
