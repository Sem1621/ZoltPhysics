//! Port of: PerformanceTest/Layers.h
//! Status: complete
//!
//! The object and broadphase layers of the performance test (two of each, unlike the unit test layers of
//! ZoltTests/Layers.zig). The namespaces `Layers` and `BroadPhaseLayers` are structs with constants
//! (`Layers::NON_MOVING` is `Layers.non_moving`), the filter classes are pattern A implementations (pass `&filter.base`
//! where the base class is expected). `GetBroadPhaseLayerName` exists only with JPH_EXTERNAL_PROFILE /
//! JPH_PROFILE_ENABLED: TODO(profiler).

const std = @import("std");
const zolt = @import("zolt");

const Core = zolt.Core;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;

/// Layer that objects can be in, determines which other objects it can collide with
pub const Layers = struct {
    pub const non_moving: ObjectLayer = 0;
    pub const moving: ObjectLayer = 1;
    pub const num_layers: ObjectLayer = 2;
};

/// Class that determines if two object layers can collide
pub const ObjectLayerPairFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterImpl),

    pub fn shouldCollide(self: *const ObjectLayerPairFilterImpl, object1: ObjectLayer, object2: ObjectLayer) bool {
        _ = self;
        switch (object1) {
            Layers.non_moving => return object2 == Layers.moving, // Non moving only collides with moving
            Layers.moving => return true, // Moving collides with everything
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

/// Broadphase layers
pub const BroadPhaseLayers = struct {
    pub const non_moving: BroadPhaseLayer = .init(0);
    pub const moving: BroadPhaseLayer = .init(1);
    pub const num_layers: u32 = 2;
};

/// BroadPhaseLayerInterface implementation
pub const BPLayerInterfaceImpl = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(BPLayerInterfaceImpl),
    object_to_broad_phase: [Layers.num_layers]BroadPhaseLayer,

    pub fn init() BPLayerInterfaceImpl {
        var self: BPLayerInterfaceImpl = .{ .object_to_broad_phase = undefined };

        // Create a mapping table from object to broad phase layer
        self.object_to_broad_phase[Layers.non_moving] = BroadPhaseLayers.non_moving;
        self.object_to_broad_phase[Layers.moving] = BroadPhaseLayers.moving;
        return self;
    }

    pub fn getNumBroadPhaseLayers(self: *const BPLayerInterfaceImpl) u32 {
        _ = self;
        return BroadPhaseLayers.num_layers;
    }

    pub fn getBroadPhaseLayer(self: *const BPLayerInterfaceImpl, layer: ObjectLayer) BroadPhaseLayer {
        std.debug.assert(layer < Layers.num_layers);
        return self.object_to_broad_phase[layer];
    }

    // TODO(profiler): GetBroadPhaseLayerName (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
};

/// Class that determines if an object layer can collide with a broadphase layer
pub const ObjectVsBroadPhaseLayerFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseLayerFilterImpl),

    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilterImpl, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        _ = self;
        switch (layer1) {
            Layers.non_moving => return layer2.eql(BroadPhaseLayers.moving),
            Layers.moving => return true,
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

test "Layers" {
    const expect = std.testing.expect;

    const pair_filter: ObjectLayerPairFilterImpl = .{};
    const pair: *const ObjectLayerPairFilter = &pair_filter.base;
    try expect(pair.shouldCollide(Layers.moving, Layers.non_moving));
    try expect(pair.shouldCollide(Layers.moving, Layers.moving));
    try expect(pair.shouldCollide(Layers.non_moving, Layers.moving));
    try expect(!pair.shouldCollide(Layers.non_moving, Layers.non_moving));

    const bp_interface = BPLayerInterfaceImpl.init();
    const interface: *const BroadPhaseLayerInterface = &bp_interface.base;
    try std.testing.expectEqual(BroadPhaseLayers.num_layers, interface.getNumBroadPhaseLayers());
    try expect(interface.getBroadPhaseLayer(Layers.non_moving).eql(BroadPhaseLayers.non_moving));
    try expect(interface.getBroadPhaseLayer(Layers.moving).eql(BroadPhaseLayers.moving));

    const bp_filter: ObjectVsBroadPhaseLayerFilterImpl = .{};
    const filter: *const ObjectVsBroadPhaseLayerFilter = &bp_filter.base;
    try expect(filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.moving));
    try expect(!filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.non_moving));
    try expect(filter.shouldCollide(Layers.moving, BroadPhaseLayers.non_moving));
    try expect(filter.shouldCollide(Layers.moving, BroadPhaseLayers.moving));
}
