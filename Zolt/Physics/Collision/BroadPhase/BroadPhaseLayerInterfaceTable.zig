//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h
//! Status: complete
//! Not ported: JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (mBroadPhaseLayerNames, SetBroadPhaseLayerName, GetBroadPhaseLayerName, Zolt has no profiler)
//!
//! A pattern A implementation of BroadPhaseLayerInterface (Docs/Zolt/CollisionArchitecture.md, D7) that owns its
//! table: `init(allocator, num_object_layers, num_broad_phase_layers)` is the constructor, `deinit()` the destructor
//! (the allocator is stored). Pass `&interface.base` where an `*const BroadPhaseLayerInterface` is expected. Like the
//! C++ class (NonCopyable through its base) it is not copied.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ObjectLayer = @import("../ObjectLayer.zig").ObjectLayer;
const BroadPhaseLayer = @import("BroadPhaseLayer.zig").BroadPhaseLayer;
const BroadPhaseLayerInterface = @import("BroadPhaseLayer.zig").BroadPhaseLayerInterface;

/// BroadPhaseLayerInterface implementation.
/// This defines a mapping between object and broadphase layers.
/// This implementation uses a simple table
pub const BroadPhaseLayerInterfaceTable = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(BroadPhaseLayerInterfaceTable),
    num_broad_phase_layers: u32,
    object_to_broad_phase: std.ArrayList(BroadPhaseLayer) = .empty,
    /// Owns the table
    allocator: Allocator,

    /// Constructor
    pub fn init(allocator: Allocator, num_object_layers: u32, num_broad_phase_layers: u32) Allocator.Error!BroadPhaseLayerInterfaceTable {
        var self: BroadPhaseLayerInterfaceTable = .{ .num_broad_phase_layers = num_broad_phase_layers, .allocator = allocator };
        try self.object_to_broad_phase.appendNTimes(allocator, .init(0), num_object_layers);

        // TODO(profiler): mBroadPhaseLayerNames.resize(inNumBroadPhaseLayers, "Undefined") (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
        return self;
    }

    /// Destructor
    pub fn deinit(self: *BroadPhaseLayerInterfaceTable) void {
        self.object_to_broad_phase.deinit(self.allocator);
    }

    pub fn mapObjectToBroadPhaseLayer(self: *BroadPhaseLayerInterfaceTable, object_layer: ObjectLayer, broad_phase_layer: BroadPhaseLayer) void {
        std.debug.assert(broad_phase_layer.toType() < self.num_broad_phase_layers);
        self.object_to_broad_phase.items[object_layer] = broad_phase_layer;
    }

    pub fn getNumBroadPhaseLayers(self: *const BroadPhaseLayerInterfaceTable) u32 {
        return self.num_broad_phase_layers;
    }

    pub fn getBroadPhaseLayer(self: *const BroadPhaseLayerInterfaceTable, layer: ObjectLayer) BroadPhaseLayer {
        return self.object_to_broad_phase.items[layer];
    }

    // TODO(profiler): SetBroadPhaseLayerName, GetBroadPhaseLayerName (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
};

test "BroadPhaseLayerInterfaceTable" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    var bp_interface = try BroadPhaseLayerInterfaceTable.init(allocator, 4, 3);
    defer bp_interface.deinit();
    const interface: *const BroadPhaseLayerInterface = &bp_interface.base;
    try std.testing.expectEqual(@as(u32, 3), interface.getNumBroadPhaseLayers());

    // Unmapped layers map to broadphase layer 0
    for (0..4) |i| try expect(interface.getBroadPhaseLayer(@intCast(i)).eql(.init(0)));

    bp_interface.mapObjectToBroadPhaseLayer(1, .init(2));
    bp_interface.mapObjectToBroadPhaseLayer(3, .init(1));
    bp_interface.mapObjectToBroadPhaseLayer(3, .init(2)); // The last mapping wins
    try expect(interface.getBroadPhaseLayer(0).eql(.init(0)));
    try expect(interface.getBroadPhaseLayer(1).eql(.init(2)));
    try expect(interface.getBroadPhaseLayer(2).eql(.init(0)));
    try expect(interface.getBroadPhaseLayer(3).eql(.init(2)));
}

test "BroadPhaseLayerInterfaceTable: out of memory" {
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        var bp_interface = BroadPhaseLayerInterfaceTable.init(failing.allocator(), 11, 6) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        bp_interface.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The table
        break;
    }
}
