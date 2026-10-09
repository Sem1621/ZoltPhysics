//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceMask.h
//! Status: complete
//! Not ported: JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (mBroadPhaseLayerNames, SetBroadPhaseLayerName, GetBroadPhaseLayerName, Zolt has no profiler)
//!
//! A pattern A implementation of BroadPhaseLayerInterface (Docs/Zolt/CollisionArchitecture.md, D7) that owns its
//! mapping: `init(allocator, n)` is the constructor, `deinit()` the destructor (the allocator is stored). Pass
//! `&interface.base` where an `*const BroadPhaseLayerInterface` is expected. Like the C++ class (NonCopyable through
//! its base) it is not copied.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ObjectLayer = @import("../ObjectLayer.zig").ObjectLayer;
const ObjectLayerPairFilterMask = @import("../ObjectLayerPairFilterMask.zig").ObjectLayerPairFilterMask;
const BroadPhaseLayer = @import("BroadPhaseLayer.zig").BroadPhaseLayer;
const BroadPhaseLayerInterface = @import("BroadPhaseLayer.zig").BroadPhaseLayerInterface;

/// BroadPhaseLayerInterface implementation.
/// This defines a mapping between object and broadphase layers.
/// This implementation works together with ObjectLayerPairFilterMask and ObjectVsBroadPhaseLayerFilterMask.
/// A broadphase layer is suitable for an object if its group & inGroupsToInclude is not zero and its group & inGroupsToExclude is zero.
/// The broadphase layers are iterated from lowest to highest value and the first one that matches is taken. If none match then it takes the last layer.
pub const BroadPhaseLayerInterfaceMask = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    const Mapping = struct {
        groups_to_include: u32 = 0,
        groups_to_exclude: u32 = ~@as(u32, 0),
    };

    base: BroadPhaseLayerInterface = .init(BroadPhaseLayerInterfaceMask),
    mapping: std.ArrayList(Mapping) = .empty,
    /// Owns the mapping
    allocator: Allocator,

    /// Constructor
    pub fn init(allocator: Allocator, num_broad_phase_layers: u32) Allocator.Error!BroadPhaseLayerInterfaceMask {
        std.debug.assert(num_broad_phase_layers > 0);
        var self: BroadPhaseLayerInterfaceMask = .{ .allocator = allocator };
        try self.mapping.appendNTimes(allocator, .{}, num_broad_phase_layers);

        // TODO(profiler): mBroadPhaseLayerNames.resize(inNumBroadPhaseLayers, "Undefined") (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
        return self;
    }

    /// Destructor
    pub fn deinit(self: *BroadPhaseLayerInterfaceMask) void {
        self.mapping.deinit(self.allocator);
    }

    // Configures a broadphase layer.
    pub fn configureLayer(self: *BroadPhaseLayerInterfaceMask, broad_phase_layer: BroadPhaseLayer, groups_to_include: u32, groups_to_exclude: u32) void {
        std.debug.assert(broad_phase_layer.toType() < @as(u32, @intCast(self.mapping.items.len)));
        const m = &self.mapping.items[broad_phase_layer.toType()];
        m.groups_to_include = groups_to_include;
        m.groups_to_exclude = groups_to_exclude;
    }

    pub fn getNumBroadPhaseLayers(self: *const BroadPhaseLayerInterfaceMask) u32 {
        return @intCast(self.mapping.items.len);
    }

    pub fn getBroadPhaseLayer(self: *const BroadPhaseLayerInterfaceMask, layer: ObjectLayer) BroadPhaseLayer {
        // Try to find the first broadphase layer that matches
        const group = ObjectLayerPairFilterMask.getGroup(layer);
        for (self.mapping.items, 0..) |*m, index|
            if ((group & m.groups_to_include) != 0 and (group & m.groups_to_exclude) == 0)
                return .init(@truncate(index));

        // Fall back to the last broadphase layer
        return .init(@truncate(self.mapping.items.len - 1));
    }

    /// Returns true if an object layer should collide with a broadphase layer, this function is being called from ObjectVsBroadPhaseLayerFilterMask
    pub fn shouldCollide(self: *const BroadPhaseLayerInterfaceMask, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        const mask = ObjectLayerPairFilterMask.getMask(layer1);
        const index: usize = layer2.toType();
        const m = &self.mapping.items[index];
        return index == self.mapping.items.len - 1 // Last layer may collide with anything
        or (m.groups_to_include & mask) != 0; // Mask allows it to collide with objects that could reside in this layer
    }

    // TODO(profiler): SetBroadPhaseLayerName, GetBroadPhaseLayerName (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)
};

test "BroadPhaseLayerInterfaceMask" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    var bp_interface = try BroadPhaseLayerInterfaceMask.init(allocator, 3);
    defer bp_interface.deinit();
    const interface: *const BroadPhaseLayerInterface = &bp_interface.base;
    try std.testing.expectEqual(@as(u32, 3), interface.getNumBroadPhaseLayers());

    // Unconfigured layers never match (include nothing, exclude everything): everything goes to the last layer
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0b1, .{})).eql(.init(2)));

    bp_interface.configureLayer(.init(0), 0b011, 0b100);
    bp_interface.configureLayer(.init(1), 0b100, 0);
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0b001, .{})).eql(.init(0)));
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0b010, .{})).eql(.init(0)));
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0b110, .{})).eql(.init(1))); // Excluded from 0
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0b1000, .{})).eql(.init(2)));
    try expect(interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(0, .{})).eql(.init(2)));

    // The mask decides, the last layer collides with everything
    const layer = ObjectLayerPairFilterMask.getObjectLayer(0b1, .{ .mask = 0b001 });
    try expect(bp_interface.shouldCollide(layer, .init(0)));
    try expect(!bp_interface.shouldCollide(layer, .init(1)));
    try expect(bp_interface.shouldCollide(layer, .init(2)));
    try expect(!bp_interface.shouldCollide(ObjectLayerPairFilterMask.getObjectLayer(0b1, .{ .mask = 0 }), .init(0)));
    try expect(bp_interface.shouldCollide(ObjectLayerPairFilterMask.getObjectLayer(0b1, .{ .mask = 0 }), .init(2)));
}

test "BroadPhaseLayerInterfaceMask: out of memory" {
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        var bp_interface = BroadPhaseLayerInterfaceMask.init(failing.allocator(), 4) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        bp_interface.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The mapping
        break;
    }
}
