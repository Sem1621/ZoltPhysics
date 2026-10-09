//! Port of: Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterMask.h
//! Status: complete
//!
//! A pattern A filter (Docs/Zolt/CollisionArchitecture.md, D7). Jolt's `const BroadPhaseLayerInterfaceMask &` member
//! is a `*const BroadPhaseLayerInterfaceMask`: the interface must outlive the filter. Pass `&filter.base` where an
//! `*const ObjectVsBroadPhaseLayerFilter` is expected.

const std = @import("std");
const ObjectLayer = @import("../ObjectLayer.zig").ObjectLayer;
const BroadPhaseLayer = @import("BroadPhaseLayer.zig").BroadPhaseLayer;
const ObjectVsBroadPhaseLayerFilter = @import("BroadPhaseLayer.zig").ObjectVsBroadPhaseLayerFilter;
const BroadPhaseLayerInterfaceMask = @import("BroadPhaseLayerInterfaceMask.zig").BroadPhaseLayerInterfaceMask;

/// Class that determines if an object layer can collide with a broadphase layer.
/// This implementation works together with BroadPhaseLayerInterfaceMask and ObjectLayerPairFilterMask
pub const ObjectVsBroadPhaseLayerFilterMask = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseLayerFilterMask),
    broad_phase_layer_interface: *const BroadPhaseLayerInterfaceMask,

    /// Constructor
    pub fn init(broad_phase_layer_interface: *const BroadPhaseLayerInterfaceMask) ObjectVsBroadPhaseLayerFilterMask {
        return .{ .broad_phase_layer_interface = broad_phase_layer_interface };
    }

    /// Returns true if an object layer should collide with a broadphase layer
    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilterMask, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        // Just defer to BroadPhaseLayerInterface
        return self.broad_phase_layer_interface.shouldCollide(layer1, layer2);
    }
};

test "ObjectVsBroadPhaseLayerFilterMask" {
    const ObjectLayerPairFilterMask = @import("../ObjectLayerPairFilterMask.zig").ObjectLayerPairFilterMask;
    const expect = std.testing.expect;

    var bp_interface = try BroadPhaseLayerInterfaceMask.init(std.testing.allocator, 2);
    defer bp_interface.deinit();
    bp_interface.configureLayer(.init(0), 0b01, 0);

    const filter = ObjectVsBroadPhaseLayerFilterMask.init(&bp_interface);
    const base: *const ObjectVsBroadPhaseLayerFilter = &filter.base;
    try expect(base.shouldCollide(ObjectLayerPairFilterMask.getObjectLayer(0b10, .{ .mask = 0b01 }), .init(0)));
    try expect(!base.shouldCollide(ObjectLayerPairFilterMask.getObjectLayer(0b10, .{ .mask = 0b10 }), .init(0)));
    try expect(base.shouldCollide(ObjectLayerPairFilterMask.getObjectLayer(0b10, .{ .mask = 0b10 }), .init(1))); // Last layer
}
