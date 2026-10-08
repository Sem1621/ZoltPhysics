//! Port of: Jolt/Physics/Collision/Shape/SubShapeID.h
//! Status: complete
//!
//! Value types (`extern struct`: hashed and stored as keys). `PopID(inBits, outRemainder)` returns a struct.

const std = @import("std");

/// A sub shape id contains a path to an element (usually a triangle or other primitive type) of a compound shape
///
/// Each sub shape knows how many bits it needs to encode its ID, so knows how many bits to take from the sub shape ID.
/// A sub shape ID will become invalid when the structure of the shape changes.
pub const SubShapeID = extern struct {
    /// Underlying storage type
    pub const Type = u32;
    /// Type that is bigger than the underlying storage type for operations that would otherwise overflow
    pub const BiggerType = u64;
    /// How many bits we can store in this ID
    pub const max_bits: u32 = 8 * @sizeOf(Type);
    /// An empty SubShapeID has all bits set (cEmpty)
    const empty_value: Type = ~@as(Type, 0);

    /// Default constructor: an empty sub shape ID
    pub const empty: SubShapeID = .{};

    value: Type = empty_value,

    /// Result of `popID` (the C++ returns the id and writes outRemainder)
    pub const PopResult = struct { id: Type, remainder: SubShapeID };

    /// Get the next id in the chain of ids (pops parents before children)
    pub fn popID(self: SubShapeID, bits: u32) PopResult {
        const mask_bits: Type = @truncate((@as(BiggerType, 1) << @intCast(bits)) - 1);
        const fill_bits: Type = @truncate(@as(BiggerType, empty_value) << @intCast(max_bits - bits)); // Fill left side bits with 1 so that if there's no remainder all bits will be set, note that we do this using a BiggerType since on intel 0xffffffff << 32 == 0xffffffff
        const v = self.value & mask_bits;
        return .{ .id = v, .remainder = .{ .value = @as(Type, @truncate(@as(BiggerType, self.value) >> @intCast(bits))) | fill_bits } };
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

    /// Check equal
    pub fn eql(self: SubShapeID, other: SubShapeID) bool {
        return self.value == other.value;
    }

    /// Adds an id at a particular position in the chain (this should really only be called by the SubShapeIDCreator)
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
