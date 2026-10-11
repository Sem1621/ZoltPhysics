//! Port of: Jolt/Physics/Collision/ObjectLayerPairFilterTable.h
//! Status: complete
//!
//! A pattern A filter (Docs/Zolt/CollisionArchitecture.md, D7) that owns its bit table: `init(allocator, n)` is the
//! constructor, `deinit()` the destructor (the allocator is stored). Pass `&filter.base` where an
//! `*const ObjectLayerPairFilter` is expected. Like the C++ class (NonCopyable through its base) it is not copied.
//!
//! The bit index arithmetic is done in `u32` with wrapping operations: Jolt computes it in `uint` (32 bit object layers)
//! or in `int` after integer promotion (16 bit object layers, the same value for every layer index whose bit fits in
//! the table).

const std = @import("std");
const Allocator = std.mem.Allocator;
const ObjectLayer = @import("ObjectLayer.zig").ObjectLayer;
const ObjectLayerPairFilter = @import("ObjectLayer.zig").ObjectLayerPairFilter;

/// Filter class to test if two objects can collide based on their object layer. Used while finding collision pairs.
/// This implementation uses a table to determine if two layers can collide.
pub const ObjectLayerPairFilterTable = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterTable),
    /// The number of layers that this table supports
    num_object_layers: u32,
    /// The table of bits that indicates which layers collide
    table: std.ArrayList(u8) = .empty,
    /// Owns the table
    allocator: Allocator,

    /// Get which bit corresponds to the pair (inLayer1, inLayer2)
    fn getBit(self: *const ObjectLayerPairFilterTable, layer1_in: ObjectLayer, layer2_in: ObjectLayer) u32 {
        var layer1 = layer1_in;
        var layer2 = layer2_in;

        // We store the lower left half only, so swap the inputs when trying to access the top right half
        if (layer1 > layer2)
            std.mem.swap(ObjectLayer, &layer1, &layer2);

        std.debug.assert(layer2 < self.num_object_layers);

        // Calculate at which bit the entry for this pair resides
        // We use the fact that a row always starts at inLayer2 * (inLayer2 + 1) / 2
        // (this is the amount of bits needed to store a table of inLayer2 entries)
        const l1: u32 = layer1;
        const l2: u32 = layer2;
        return (l2 *% (l2 +% 1)) / 2 +% l1;
    }

    /// Constructs the table with inNumObjectLayers Layers, initially all layer pairs are disabled
    pub fn init(allocator: Allocator, num_object_layers: u32) Allocator.Error!ObjectLayerPairFilterTable {
        var self: ObjectLayerPairFilterTable = .{ .num_object_layers = num_object_layers, .allocator = allocator };

        // By default nothing collides
        // For the first layer we only need to store 1 bit, for the second 2 bits, for the third 3 bits, etc.
        // We use the formula Sum_i=1^N i = N * (N + 1) / 2 to calculate the size of the table
        const table_size: u32 = (num_object_layers *% (num_object_layers +% 1) / 2 + 7) / 8;
        try self.table.appendNTimes(allocator, 0, table_size);
        return self;
    }

    /// Destructor
    pub fn deinit(self: *ObjectLayerPairFilterTable) void {
        self.table.deinit(self.allocator);
    }

    /// Get the number of object layers
    pub fn getNumObjectLayers(self: *const ObjectLayerPairFilterTable) u32 {
        return self.num_object_layers;
    }

    /// Disable collision between two object layers
    pub fn disableCollision(self: *ObjectLayerPairFilterTable, layer1: ObjectLayer, layer2: ObjectLayer) void {
        const bit = self.getBit(layer1, layer2);
        self.table.items[bit >> 3] &= (0xff ^ (@as(u8, 1) << @as(u3, @intCast(bit & 0b111))));
    }

    /// Enable collision between two object layers
    pub fn enableCollision(self: *ObjectLayerPairFilterTable, layer1: ObjectLayer, layer2: ObjectLayer) void {
        const bit = self.getBit(layer1, layer2);
        self.table.items[bit >> 3] |= @as(u8, 1) << @as(u3, @intCast(bit & 0b111));
    }

    /// Returns true if two layers can collide
    pub fn shouldCollide(self: *const ObjectLayerPairFilterTable, object1: ObjectLayer, object2: ObjectLayer) bool {
        // Test if the bit is set for this group pair
        const bit = self.getBit(object1, object2);
        return (self.table.items[bit >> 3] & (@as(u8, 1) << @as(u3, @intCast(bit & 0b111)))) != 0;
    }
};

test "ObjectLayerPairFilterTable: table layout" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // Table sizes: N * (N + 1) / 2 bits rounded up to bytes
    const sizes = [_][2]u32{ .{ 0, 0 }, .{ 1, 1 }, .{ 3, 1 }, .{ 4, 2 }, .{ 11, 9 }, .{ 100, 632 } };
    for (sizes) |s| {
        var table = try ObjectLayerPairFilterTable.init(allocator, s[0]);
        defer table.deinit();
        try std.testing.expectEqual(s[1], @as(u32, @intCast(table.table.items.len)));
        try std.testing.expectEqual(s[0], table.getNumObjectLayers());
        for (table.table.items) |b| try std.testing.expectEqual(@as(u8, 0), b);
    }

    // The bit of every pair (including a layer with itself) is unique and symmetric
    var table = try ObjectLayerPairFilterTable.init(allocator, 5);
    defer table.deinit();
    var seen: [15]bool = @splat(false);
    for (0..5) |i| for (0..5) |j| {
        const bit = table.getBit(@intCast(i), @intCast(j));
        try std.testing.expectEqual(bit, table.getBit(@intCast(j), @intCast(i)));
        if (i <= j) {
            try expect(!seen[bit]);
            seen[bit] = true;
        }
    };
    for (seen) |s| try expect(s);

    // Through the base class
    const filter: *const ObjectLayerPairFilter = &table.base;
    try expect(!filter.shouldCollide(1, 3));
    table.enableCollision(3, 1);
    try expect(filter.shouldCollide(1, 3) and filter.shouldCollide(3, 1));
    table.enableCollision(4, 4);
    try expect(filter.shouldCollide(4, 4));
    try expect(!filter.shouldCollide(3, 3));
    table.disableCollision(1, 3);
    try expect(!filter.shouldCollide(3, 1));
}

test "ObjectLayerPairFilterTable: out of memory" {
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = fail_index });
        var table = ObjectLayerPairFilterTable.init(failing.allocator(), 10) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        table.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The table
        break;
    }
}
