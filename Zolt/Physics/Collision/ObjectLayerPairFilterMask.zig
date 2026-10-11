//! Port of: Jolt/Physics/Collision/ObjectLayerPairFilterMask.h
//! Status: complete
//!
//! A pattern A filter (Docs/Zolt/CollisionArchitecture.md, D7): `ObjectLayerPairFilterMask{}` is the filter, pass
//! `&filter.base` where an `*const ObjectLayerPairFilter` is expected. `sGetObjectLayer(inGroup, inMask = cMask)` is
//! `getObjectLayer(group, .{ .mask = m })`. The group and mask bits are computed in `u32` like Jolt and truncated to
//! `ObjectLayer` (16 or 32 bits, `-Dobject_layer_bits`).

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const ObjectLayer = @import("ObjectLayer.zig").ObjectLayer;
const ObjectLayerPairFilter = @import("ObjectLayer.zig").ObjectLayerPairFilter;

/// Filter class to test if two objects can collide based on their object layer. Used while finding collision pairs.
/// Uses group bits and mask bits. Two layers can collide if Object1.Group & Object2.Mask is non-zero and Object2.Group & Object1.Mask is non-zero.
/// The behavior is similar to that in e.g. Bullet.
/// This implementation works together with BroadPhaseLayerInterfaceMask and ObjectVsBroadPhaseLayerFilterMask
pub const ObjectLayerPairFilterMask = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterMask),

    /// Number of bits for the group and mask bits
    pub const num_bits: u32 = Core.object_layer_bits / 2;
    pub const mask: u32 = (1 << num_bits) - 1;

    /// Construct an ObjectLayer from a group and mask bits
    pub fn getObjectLayer(group: u32, opts: struct { mask: u32 = mask }) ObjectLayer {
        std.debug.assert((group & ~mask) == 0);
        std.debug.assert((opts.mask & ~mask) == 0);
        return @truncate((group & mask) | (opts.mask << num_bits));
    }

    /// Get the group bits from an ObjectLayer
    pub fn getGroup(object_layer: ObjectLayer) u32 {
        return @as(u32, object_layer) & mask;
    }

    /// Get the mask bits from an ObjectLayer
    pub fn getMask(object_layer: ObjectLayer) u32 {
        return @as(u32, object_layer) >> num_bits;
    }

    /// Returns true if two layers can collide
    pub fn shouldCollide(self: *const ObjectLayerPairFilterMask, object1: ObjectLayer, object2: ObjectLayer) bool {
        _ = self;
        return (getGroup(object1) & getMask(object2)) != 0 and (getGroup(object2) & getMask(object1)) != 0;
    }
};

test "ObjectLayerPairFilterMask" {
    const expect = std.testing.expect;

    try std.testing.expectEqual(@as(u32, Core.object_layer_bits / 2), ObjectLayerPairFilterMask.num_bits);
    try std.testing.expectEqual(@as(u32, (1 << (Core.object_layer_bits / 2)) - 1), ObjectLayerPairFilterMask.mask);

    // Round trip of the group and mask bits, the default mask is everything
    const layer = ObjectLayerPairFilterMask.getObjectLayer(0b101, .{ .mask = 0b110 });
    try std.testing.expectEqual(@as(u32, 0b101), ObjectLayerPairFilterMask.getGroup(layer));
    try std.testing.expectEqual(@as(u32, 0b110), ObjectLayerPairFilterMask.getMask(layer));
    const all = ObjectLayerPairFilterMask.getObjectLayer(0b1, .{});
    try std.testing.expectEqual(ObjectLayerPairFilterMask.mask, ObjectLayerPairFilterMask.getMask(all));
    try std.testing.expectEqual(@as(ObjectLayer, std.math.maxInt(ObjectLayer)), ObjectLayerPairFilterMask.getObjectLayer(ObjectLayerPairFilterMask.mask, .{}));

    // Through the base class
    const filter: ObjectLayerPairFilterMask = .{};
    const base: *const ObjectLayerPairFilter = &filter.base;
    const a = ObjectLayerPairFilterMask.getObjectLayer(0b001, .{ .mask = 0b010 });
    const b = ObjectLayerPairFilterMask.getObjectLayer(0b010, .{ .mask = 0b001 });
    const c = ObjectLayerPairFilterMask.getObjectLayer(0b001, .{ .mask = 0b001 });
    try expect(base.shouldCollide(a, b) and base.shouldCollide(b, a));
    try expect(!base.shouldCollide(c, b) and !base.shouldCollide(b, c)); // b's group is not in c's mask
    try expect(base.shouldCollide(layer, layer)); // 0b101 & 0b110 != 0 both ways
    try expect(!base.shouldCollide(layer, all)); // all's group 0b1 is not in layer's mask 0b110
    try expect(base.shouldCollide(all, all));
}
