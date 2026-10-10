//! Port of: UnitTests/Layers.h
//! Status: complete
//!
//! The object and broadphase layers used by the unit tests. The namespaces `Layers` and `BroadPhaseLayers` are structs
//! with constants (`Layers::NON_MOVING` is `Layers.non_moving`), the filter classes are pattern A implementations
//! (pass `&filter.base` where the base class is expected).

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
    pub const unused1: ObjectLayer = 0; // 5 unused values so that broadphase layers values don't match with object layer values (for testing purposes)
    pub const unused2: ObjectLayer = 1;
    pub const unused3: ObjectLayer = 2;
    pub const unused4: ObjectLayer = 3;
    pub const unused5: ObjectLayer = 4;
    pub const non_moving: ObjectLayer = 5;
    pub const moving: ObjectLayer = 6;
    pub const moving2: ObjectLayer = 7; // Another moving layer that acts as MOVING but doesn't collide with MOVING
    pub const hq_debris: ObjectLayer = 8; // High quality debris collides with MOVING and NON_MOVING but not with any debris
    pub const lq_debris: ObjectLayer = 9; // Low quality debris only collides with NON_MOVING
    pub const sensor: ObjectLayer = 10; // Sensors only collide with MOVING objects
    pub const num_layers: ObjectLayer = 11;
};

/// Class that determines if two object layers can collide
pub const ObjectLayerPairFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterImpl),

    pub fn shouldCollide(self: *const ObjectLayerPairFilterImpl, object1: ObjectLayer, object2: ObjectLayer) bool {
        _ = self;
        switch (object1) {
            Layers.unused1, Layers.unused2, Layers.unused3, Layers.unused4, Layers.unused5 => return false,
            Layers.non_moving => return object2 == Layers.moving or object2 == Layers.moving2 or object2 == Layers.hq_debris or object2 == Layers.lq_debris,
            Layers.moving => return object2 == Layers.non_moving or object2 == Layers.moving or object2 == Layers.hq_debris or object2 == Layers.sensor,
            Layers.moving2 => return object2 == Layers.non_moving or object2 == Layers.moving2 or object2 == Layers.hq_debris or object2 == Layers.sensor,
            Layers.hq_debris => return object2 == Layers.non_moving or object2 == Layers.moving or object2 == Layers.moving2,
            Layers.lq_debris => return object2 == Layers.non_moving,
            Layers.sensor => return object2 == Layers.moving or object2 == Layers.moving2,
            else => {
                if (Core.enable_asserts) @panic("invalid object layer");
                return false;
            },
        }
    }
};

/// Broadphase layers
pub const BroadPhaseLayers = struct {
    pub const non_moving: BroadPhaseLayer = .init(0);
    pub const moving: BroadPhaseLayer = .init(1);
    pub const moving2: BroadPhaseLayer = .init(2);
    pub const lq_debris: BroadPhaseLayer = .init(3);
    pub const unused: BroadPhaseLayer = .init(4);
    pub const sensor: BroadPhaseLayer = .init(5);
    pub const num_layers: u32 = 6;
};

/// BroadPhaseLayerInterface implementation
pub const BPLayerInterfaceImpl = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(BPLayerInterfaceImpl),
    object_to_broad_phase: [Layers.num_layers]BroadPhaseLayer,

    pub fn init() BPLayerInterfaceImpl {
        var self: BPLayerInterfaceImpl = .{ .object_to_broad_phase = undefined };

        // Create a mapping table from object to broad phase layer
        self.object_to_broad_phase[Layers.unused1] = BroadPhaseLayers.unused;
        self.object_to_broad_phase[Layers.unused2] = BroadPhaseLayers.unused;
        self.object_to_broad_phase[Layers.unused3] = BroadPhaseLayers.unused;
        self.object_to_broad_phase[Layers.unused4] = BroadPhaseLayers.unused;
        self.object_to_broad_phase[Layers.unused5] = BroadPhaseLayers.unused;
        self.object_to_broad_phase[Layers.non_moving] = BroadPhaseLayers.non_moving;
        self.object_to_broad_phase[Layers.moving] = BroadPhaseLayers.moving;
        self.object_to_broad_phase[Layers.moving2] = BroadPhaseLayers.moving2;
        self.object_to_broad_phase[Layers.hq_debris] = BroadPhaseLayers.moving; // HQ_DEBRIS is also in the MOVING layer as an example on how to map multiple layers onto the same broadphase layer
        self.object_to_broad_phase[Layers.lq_debris] = BroadPhaseLayers.lq_debris;
        self.object_to_broad_phase[Layers.sensor] = BroadPhaseLayers.sensor;
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
            Layers.moving, Layers.hq_debris => return layer2.eql(BroadPhaseLayers.non_moving) or layer2.eql(BroadPhaseLayers.moving) or layer2.eql(BroadPhaseLayers.sensor),
            Layers.moving2 => return layer2.eql(BroadPhaseLayers.non_moving) or layer2.eql(BroadPhaseLayers.moving2) or layer2.eql(BroadPhaseLayers.sensor),
            Layers.lq_debris => return layer2.eql(BroadPhaseLayers.non_moving),
            Layers.sensor => return layer2.eql(BroadPhaseLayers.moving),
            Layers.unused1, Layers.unused2, Layers.unused3, Layers.unused4, Layers.unused5 => return false,
            else => {
                if (Core.enable_asserts) @panic("invalid object layer");
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
    try expect(!pair.shouldCollide(Layers.moving, Layers.moving2));
    try expect(!pair.shouldCollide(Layers.unused3, Layers.moving));

    const bp_interface = BPLayerInterfaceImpl.init();
    const interface: *const BroadPhaseLayerInterface = &bp_interface.base;
    try std.testing.expectEqual(BroadPhaseLayers.num_layers, interface.getNumBroadPhaseLayers());
    try expect(interface.getBroadPhaseLayer(Layers.hq_debris).eql(BroadPhaseLayers.moving));
    try expect(interface.getBroadPhaseLayer(Layers.unused1).eql(BroadPhaseLayers.unused));

    const bp_filter: ObjectVsBroadPhaseLayerFilterImpl = .{};
    const filter: *const ObjectVsBroadPhaseLayerFilter = &bp_filter.base;
    try expect(filter.shouldCollide(Layers.sensor, BroadPhaseLayers.moving));
    try expect(!filter.shouldCollide(Layers.sensor, BroadPhaseLayers.sensor));
}
