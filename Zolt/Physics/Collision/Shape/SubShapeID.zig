//! Port of: Jolt/Physics/Collision/Shape/SubShapeID.h
//! Status: complete
//!
//! Value types (`extern struct`: hashed, streamed and stored as keys, D12).
//! - `SubShapeID()` is the field default (`.empty`, all bits set). The private constructor `SubShapeID(inValue)` and
//!   `PushID` are only used by SubShapeIDCreator in Jolt; Zig has no friends, `pushID` stays private and the field
//!   `value` is accessible like every Zig field (use `setValue` / `getValue`).
//! - `PopID(inBits, outRemainder) -> Type` returns `PopResult{ .id, .remainder }`.
//! - `operator ==` is `eql` (`!=` is `!a.eql(b)`).

const std = @import("std");

/// @brief A sub shape id contains a path to an element (usually a triangle or other primitive type) of a compound shape
///
/// Each sub shape knows how many bits it needs to encode its ID, so knows how many bits to take from the sub shape ID.
///
/// For example:
/// * We have a CompoundShape A with 5 child shapes (identify sub shape using 3 bits AAA)
/// * One of its child shapes is CompoundShape B which has 3 child shapes (identify sub shape using 2 bits BB)
/// * One of its child shapes is MeshShape C which contains enough triangles to need 7 bits to identify a triangle (identify sub shape using 7 bits CCCCCCC, note that MeshShape is block based and sorts triangles spatially, you can't assume that the first triangle will have bit pattern 0000000).
///
/// The bit pattern of the sub shape ID to identify a triangle in MeshShape C will then be CCCCCCCBBAAA.
///
/// A sub shape ID will become invalid when the structure of the shape changes. For example, if a child shape is removed from a compound shape, the sub shape ID will no longer be valid.
/// This can be a problem when caching sub shape IDs from one frame to the next. See comments at ContactListener::OnContactPersisted / OnContactRemoved.
pub const SubShapeID = extern struct {
    /// Underlying storage type
    pub const Type = u32;

    /// Type that is bigger than the underlying storage type for operations that would otherwise overflow
    pub const BiggerType = u64;

    comptime {
        std.debug.assert(@sizeOf(BiggerType) > @sizeOf(Type)); // The calculation below assumes BiggerType is a bigger type than Type
    }

    /// How many bits we can store in this ID
    pub const max_bits: u32 = 8 * @sizeOf(Type);

    /// An empty SubShapeID has all bits set
    pub const empty_value: Type = ~@as(Type, 0);

    /// Constructor: an empty sub shape ID (the default constructor)
    pub const empty: SubShapeID = .{};

    value: Type = empty_value,

    /// Result of `popID`: the id and the remainder (Jolt's outRemainder)
    pub const PopResult = struct {
        id: Type,
        remainder: SubShapeID,
    };

    /// Get the next id in the chain of ids (pops parents before children)
    pub fn popID(self: SubShapeID, bits: u32) PopResult {
        const mask_bits: Type = @truncate((@as(BiggerType, 1) << @intCast(bits)) - 1);
        const fill_bits: Type = @truncate(@as(BiggerType, empty_value) << @intCast(max_bits - bits)); // Fill left side bits with 1 so that if there's no remainder all bits will be set, note that we do this using a BiggerType since on intel 0xffffffff << 32 == 0xffffffff
        const v = self.value & mask_bits;
        const remainder: SubShapeID = .{ .value = @as(Type, @truncate(@as(BiggerType, self.value) >> @intCast(bits))) | fill_bits };
        return .{ .id = v, .remainder = remainder };
    }

    /// Get the value of the path to the sub shape ID
    pub fn getValue(self: SubShapeID) Type {
        return self.value;
    }

    /// Set the value of the sub shape ID (use with care!)
    pub fn setValue(self: *SubShapeID, value: Type) void {
        self.value = value;
    }

    /// Check if there is any bits of subshape ID left.
    /// Note that this is not a 100% guarantee as the subshape ID could consist of all 1 bits. Use for asserts only.
    pub fn isEmpty(self: SubShapeID) bool {
        return self.value == empty_value;
    }

    /// Check equal (operator ==, `!=` is `!a.eql(b)`)
    pub fn eql(self: SubShapeID, other: SubShapeID) bool {
        return self.value == other.value;
    }

    /// Adds an id at a particular position in the chain
    /// (this should really only be called by the SubShapeIDCreator)
    fn pushID(self: *SubShapeID, value: Type, first_bit: u32, bits: u32) void {
        // First clear the bits
        self.value &= ~(@as(Type, @truncate((@as(BiggerType, 1) << @intCast(bits)) - 1)) << @intCast(first_bit));

        // Then set them to the new value
        self.value |= value << @intCast(first_bit);
    }
};

/// A sub shape id creator can be used to create a new sub shape id by recursing through the shape
/// hierarchy and pushing new ID's onto the chain
pub const SubShapeIDCreator = extern struct {
    id: SubShapeID = .empty,
    current_bit: u32 = 0,

    /// Add a new id to the chain of id's and return it
    pub fn pushID(self: SubShapeIDCreator, value: u32, bits: u32) SubShapeIDCreator {
        std.debug.assert(value < (@as(SubShapeID.BiggerType, 1) << @intCast(bits)));
        var copy = self;
        copy.id.pushID(value, self.current_bit, bits);
        copy.current_bit += bits;
        std.debug.assert(copy.current_bit <= SubShapeID.max_bits);
        return copy;
    }

    /// Get the resulting sub shape ID
    pub fn getID(self: SubShapeIDCreator) SubShapeID {
        return self.id;
    }

    /// Get the number of bits that have been written to the sub shape ID so far
    pub fn getNumBitsWritten(self: SubShapeIDCreator) u32 {
        return self.current_bit;
    }
};

test "SubShapeID: push and pop" {
    const creator = (SubShapeIDCreator{}).pushID(5, 3).pushID(1, 1).pushID(0, 0).pushID(300, 9);
    try std.testing.expectEqual(@as(u32, 13), creator.getNumBitsWritten());
    try std.testing.expectEqual(@as(u32, (300 << 4) | (1 << 3) | 5 | (~@as(u32, 0) << 13)), creator.getID().getValue());

    var id = creator.getID();
    try std.testing.expect(!id.isEmpty());
    const a = id.popID(3);
    try std.testing.expectEqual(@as(u32, 5), a.id);
    const b = a.remainder.popID(1);
    try std.testing.expectEqual(@as(u32, 1), b.id);
    const c = b.remainder.popID(9);
    try std.testing.expectEqual(@as(u32, 300), c.id);
    try std.testing.expect(c.remainder.isEmpty());
    try std.testing.expect(c.remainder.eql(.empty));

    id.setValue(42);
    try std.testing.expectEqual(@as(u32, 42), id.getValue());
    try std.testing.expect(!id.eql(.empty));

    // Popping 0 bits returns 0 and keeps the value
    const z = id.popID(0);
    try std.testing.expectEqual(@as(u32, 0), z.id);
    try std.testing.expectEqual(@as(u32, 42), z.remainder.getValue());
}
