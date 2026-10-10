//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h
//! Status: complete
//! Not ported: JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (BroadPhaseLayerInterface::GetBroadPhaseLayerName, Zolt has no profiler)
//!
//! `BroadPhaseLayer` is an `extern struct` around a `u8` (explicit conversions only, like Jolt): `init(value)` is the
//! explicit constructor, `getValue()` / `toType()` the explicit conversion back, `eql` / `lessThan` the operators.
//!
//! The interface and filters are pattern A classes, see ObjectLayer.zig for how to derive from them
//! (Docs/Zolt/CollisionArchitecture.md, D7). `BroadPhaseLayerInterface` has only pure virtual functions, so it has
//! no default vtable: an implementation embeds `base: BroadPhaseLayerInterface = .init(@This())`.

const virtual = @import("../../../Core/Virtual.zig");
const ObjectLayer = @import("../ObjectLayer.zig").ObjectLayer;

/// An object layer can be mapped to a broadphase layer. Objects with the same broadphase layer will end up in the same sub structure (usually a tree) of the broadphase.
/// When there are many layers, this reduces the total amount of sub structures the broad phase needs to manage. Usually you want objects that don't collide with each other
/// in different broad phase layers, but there could be exceptions if objects layers only contain a minor amount of objects so it is not beneficial to give each layer its
/// own sub structure in the broadphase.
/// Note: This class requires explicit casting from and to Type to avoid confusion with ObjectLayer
pub const BroadPhaseLayer = extern struct {
    pub const Type = u8;

    value: Type,

    /// Explicit constructor (BroadPhaseLayer(Type))
    pub fn init(value: Type) BroadPhaseLayer {
        return .{ .value = value };
    }

    /// operator ==, `!=` is `!a.eql(b)`
    pub fn eql(self: BroadPhaseLayer, other: BroadPhaseLayer) bool {
        return self.value == other.value;
    }

    /// operator <
    pub fn lessThan(self: BroadPhaseLayer, other: BroadPhaseLayer) bool {
        return self.value < other.value;
    }

    /// explicit operator Type()
    pub fn toType(self: BroadPhaseLayer) Type {
        return self.value;
    }

    pub fn getValue(self: BroadPhaseLayer) Type {
        return self.value;
    }
};

/// Constant value used to indicate an invalid broad phase layer
pub const broad_phase_layer_invalid: BroadPhaseLayer = .init(0xff);

/// Interface that the application should implement to allow mapping object layers to broadphase layers
pub const BroadPhaseLayerInterface = struct {
    pub const VTable = struct {
        /// Return the number of broadphase layers there are
        getNumBroadPhaseLayers: *const fn (self: *const BroadPhaseLayerInterface) u32,

        /// Convert an object layer to the corresponding broadphase layer
        getBroadPhaseLayer: *const fn (self: *const BroadPhaseLayerInterface, layer: ObjectLayer) BroadPhaseLayer,

        // TODO(profiler): GetBroadPhaseLayerName (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
    };

    vtable: *const VTable,

    /// Constructor, called by implementations with their most derived type
    pub fn init(comptime T: type) BroadPhaseLayerInterface {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of implementation T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Return the number of broadphase layers there are
    pub fn getNumBroadPhaseLayers(self: *const BroadPhaseLayerInterface) u32 {
        return self.vtable.getNumBroadPhaseLayers(self);
    }

    /// Convert an object layer to the corresponding broadphase layer
    pub fn getBroadPhaseLayer(self: *const BroadPhaseLayerInterface, layer: ObjectLayer) BroadPhaseLayer {
        return self.vtable.getBroadPhaseLayer(self, layer);
    }

    /// All functions are pure virtual
    pub const impl = struct {};
};

/// Class to test if an object can collide with a broadphase layer. Used while finding collision pairs.
pub const ObjectVsBroadPhaseLayerFilter = struct {
    pub const VTable = struct {
        /// Returns true if an object layer should collide with a broadphase layer
        shouldCollide: *const fn (self: *const ObjectVsBroadPhaseLayerFilter, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool,
    };

    vtable: *const VTable = vtableFor(ObjectVsBroadPhaseLayerFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) ObjectVsBroadPhaseLayerFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Returns true if an object layer should collide with a broadphase layer
    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilter, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        return self.vtable.shouldCollide(self, layer1, layer2);
    }

    /// Default implementations of the virtual functions
    pub const impl = struct {
        pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilter, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
            _ = self;
            _ = layer1;
            _ = layer2;
            return true;
        }
    };
};

/// Filter class for broadphase layers
pub const BroadPhaseLayerFilter = struct {
    pub const VTable = struct {
        /// Function to filter out broadphase layers when doing collision query test (return true to allow testing against objects with this layer)
        shouldCollide: *const fn (self: *const BroadPhaseLayerFilter, layer: BroadPhaseLayer) bool,
    };

    vtable: *const VTable = vtableFor(BroadPhaseLayerFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) BroadPhaseLayerFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// Function to filter out broadphase layers when doing collision query test (return true to allow testing against objects with this layer)
    pub fn shouldCollide(self: *const BroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
        return self.vtable.shouldCollide(self, layer);
    }

    /// Default implementations of the virtual functions
    pub const impl = struct {
        pub fn shouldCollide(self: *const BroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
            _ = self;
            _ = layer;
            return true;
        }
    };
};

/// Default filter class that uses the pair filter in combination with a specified layer to filter layers
pub const DefaultBroadPhaseLayerFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: BroadPhaseLayerFilter = .init(DefaultBroadPhaseLayerFilter),
    object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter,
    layer: ObjectLayer,

    /// Constructor
    pub fn init(object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, layer: ObjectLayer) DefaultBroadPhaseLayerFilter {
        return .{ .object_vs_broad_phase_layer_filter = object_vs_broad_phase_layer_filter, .layer = layer };
    }

    // See BroadPhaseLayerFilter::ShouldCollide
    pub fn shouldCollide(self: *const DefaultBroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
        return self.object_vs_broad_phase_layer_filter.shouldCollide(self.layer, layer);
    }
};

/// Allows objects from a specific broad phase layer only
pub const SpecifiedBroadPhaseLayerFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: BroadPhaseLayerFilter = .init(SpecifiedBroadPhaseLayerFilter),
    layer: BroadPhaseLayer,

    /// Constructor
    pub fn init(layer: BroadPhaseLayer) SpecifiedBroadPhaseLayerFilter {
        return .{ .layer = layer };
    }

    // See BroadPhaseLayerFilter::ShouldCollide
    pub fn shouldCollide(self: *const SpecifiedBroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
        return self.layer.eql(layer);
    }
};

test "BroadPhaseLayer interface and filters" {
    const std = @import("std");
    const expect = std.testing.expect;

    const a = BroadPhaseLayer.init(1);
    try expect(a.eql(.init(1)) and !a.eql(.init(2)));
    try expect(a.lessThan(.init(2)) and !a.lessThan(.init(1)));
    try expect(a.toType() == 1 and a.getValue() == 1);
    try expect(broad_phase_layer_invalid.getValue() == 0xff);

    // An implementation of the interface: object layers 0 and 1 map to broadphase layers 0 and 1
    const LayerInterface = struct {
        pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };
        base: BroadPhaseLayerInterface = .init(@This()),

        pub fn getNumBroadPhaseLayers(self: *const @This()) u32 {
            _ = self;
            return 2;
        }

        pub fn getBroadPhaseLayer(self: *const @This(), layer: ObjectLayer) BroadPhaseLayer {
            _ = self;
            return .init(@intCast(layer));
        }
    };
    const layer_interface: LayerInterface = .{};
    const interface: *const BroadPhaseLayerInterface = &layer_interface.base;
    try expect(interface.getNumBroadPhaseLayers() == 2);
    try expect(interface.getBroadPhaseLayer(1).eql(.init(1)));

    // Default constructed filters accept everything
    const all: BroadPhaseLayerFilter = .{};
    try expect(all.shouldCollide(.init(3)));
    const all_object_vs_broad_phase: ObjectVsBroadPhaseLayerFilter = .{};
    try expect(all_object_vs_broad_phase.shouldCollide(0, .init(3)));

    // A user filter: object layer 0 only collides with broadphase layer 1
    const ObjectVsBroadPhase = struct {
        pub const overrides = .{.shouldCollide};
        base: ObjectVsBroadPhaseLayerFilter = .init(@This()),

        pub fn shouldCollide(self: *const @This(), layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
            _ = self;
            return layer1 != 0 or layer2.eql(.init(1));
        }
    };
    const object_vs_broad_phase: ObjectVsBroadPhase = .{};
    const default_filter = DefaultBroadPhaseLayerFilter.init(&object_vs_broad_phase.base, 0);
    try expect(default_filter.base.shouldCollide(.init(1)));
    try expect(!default_filter.base.shouldCollide(.init(0)));

    const specified = SpecifiedBroadPhaseLayerFilter.init(.init(2));
    try expect(specified.base.shouldCollide(.init(2)));
    try expect(!specified.base.shouldCollide(.init(1)));
}
