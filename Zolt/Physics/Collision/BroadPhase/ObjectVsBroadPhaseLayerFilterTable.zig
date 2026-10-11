//! Port of: Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h
//! Status: complete
//!
//! A pattern A filter (Docs/Zolt/CollisionArchitecture.md, D7) that owns its bit table: `init(allocator, ...)` is the
//! constructor, `deinit()` the destructor (the allocator is stored). Pass `&filter.base` where an
//! `*const ObjectVsBroadPhaseLayerFilter` is expected. Like the C++ class (NonCopyable through its base) it is not
//! copied. The bit index arithmetic is done in `u32` with wrapping operations like Jolt's `uint` arithmetic.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ObjectLayer = @import("../ObjectLayer.zig").ObjectLayer;
const ObjectLayerPairFilter = @import("../ObjectLayer.zig").ObjectLayerPairFilter;
const BroadPhaseLayer = @import("BroadPhaseLayer.zig").BroadPhaseLayer;
const BroadPhaseLayerInterface = @import("BroadPhaseLayer.zig").BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = @import("BroadPhaseLayer.zig").ObjectVsBroadPhaseLayerFilter;

/// Class that determines if an object layer can collide with a broadphase layer.
/// This implementation uses a table and constructs itself from an ObjectLayerPairFilter and a BroadPhaseLayerInterface.
pub const ObjectVsBroadPhaseLayerFilterTable = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseLayerFilterTable),
    /// The total number of broadphase layers
    num_broad_phase_layers: u32,
    /// The table of bits that indicates which layers collide
    table: std.ArrayList(u8) = .empty,
    /// Owns the table
    allocator: Allocator,

    /// Get which bit corresponds to the pair (inLayer1, inLayer2)
    fn getBit(self: *const ObjectVsBroadPhaseLayerFilterTable, layer1: ObjectLayer, layer2: BroadPhaseLayer) u32 {
        // Calculate at which bit the entry for this pair resides
        return @as(u32, layer1) *% self.num_broad_phase_layers +% layer2.toType();
    }

    /// Construct the table
    /// @param inBroadPhaseLayerInterface The broad phase layer interface that maps object layers to broad phase layers
    /// @param inNumBroadPhaseLayers Number of broad phase layers
    /// @param inObjectLayerPairFilter The object layer pair filter that determines which object layers can collide
    /// @param inNumObjectLayers Number of object layers
    pub fn init(allocator: Allocator, broad_phase_layer_interface: *const BroadPhaseLayerInterface, num_broad_phase_layers: u32, object_layer_pair_filter: *const ObjectLayerPairFilter, num_object_layers: u32) Allocator.Error!ObjectVsBroadPhaseLayerFilterTable {
        var self: ObjectVsBroadPhaseLayerFilterTable = .{ .num_broad_phase_layers = num_broad_phase_layers, .allocator = allocator };

        // Resize table and set all entries to false
        try self.table.appendNTimes(allocator, 0, (num_broad_phase_layers *% num_object_layers +% 7) / 8);

        // Loop over all object layer pairs
        var o1: ObjectLayer = 0;
        while (o1 < num_object_layers) : (o1 += 1) {
            var o2: ObjectLayer = 0;
            while (o2 < num_object_layers) : (o2 += 1) {
                // Get the broad phase layer for the second object layer
                const b2 = broad_phase_layer_interface.getBroadPhaseLayer(o2);
                std.debug.assert(b2.toType() < num_broad_phase_layers);

                // If the object layers collide then so should the object and broadphase layer
                if (object_layer_pair_filter.shouldCollide(o1, o2)) {
                    const bit = self.getBit(o1, b2);
                    self.table.items[bit >> 3] |= @as(u8, 1) << @as(u3, @intCast(bit & 0b111));
                }
            }
        }

        return self;
    }

    /// Destructor
    pub fn deinit(self: *ObjectVsBroadPhaseLayerFilterTable) void {
        self.table.deinit(self.allocator);
    }

    /// Returns true if an object layer should collide with a broadphase layer
    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilterTable, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        const bit = self.getBit(layer1, layer2);
        return (self.table.items[bit >> 3] & (@as(u8, 1) << @as(u3, @intCast(bit & 0b111)))) != 0;
    }
};

test "ObjectVsBroadPhaseLayerFilterTable" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;
    const ObjectLayerPairFilterTable = @import("../ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
    const BroadPhaseLayerInterfaceTable = @import("BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;

    // 3 object layers: 0 and 1 in broadphase layer 0, 2 in broadphase layer 1
    var pair_filter = try ObjectLayerPairFilterTable.init(allocator, 3);
    defer pair_filter.deinit();
    pair_filter.enableCollision(0, 2);
    pair_filter.enableCollision(1, 1);
    var bp_interface = try BroadPhaseLayerInterfaceTable.init(allocator, 3, 2);
    defer bp_interface.deinit();
    bp_interface.mapObjectToBroadPhaseLayer(2, .init(1));

    var filter = try ObjectVsBroadPhaseLayerFilterTable.init(allocator, &bp_interface.base, 2, &pair_filter.base, 3);
    defer filter.deinit();
    try std.testing.expectEqual(@as(usize, 1), filter.table.items.len);
    const base: *const ObjectVsBroadPhaseLayerFilter = &filter.base;
    try expect(!base.shouldCollide(0, .init(0)));
    try expect(base.shouldCollide(0, .init(1)));
    try expect(base.shouldCollide(1, .init(0)));
    try expect(!base.shouldCollide(1, .init(1)));
    try expect(base.shouldCollide(2, .init(0)));
    try expect(!base.shouldCollide(2, .init(1)));

    // Out of memory
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var f = ObjectVsBroadPhaseLayerFilterTable.init(failing.allocator(), &bp_interface.base, 2, &pair_filter.base, 3) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        f.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The table
        break;
    }
}
