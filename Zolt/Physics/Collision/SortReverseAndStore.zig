//! Port of: Jolt/Physics/Collision/SortReverseAndStore.h
//! Status: complete
//!
//! `float *outValues` (4 values are written) is `out_values: *[4]f32`, `UVec4 &ioIdentifiers` is a `*UVec4`. Used by
//! the AABB tree visitors (MeshShape, HeightFieldShape, the AABBTreeToBuffer tests and the broadphase).

const Float4 = @import("../../Math/Float4.zig").Float4;
const UVec4 = @import("../../Math/UVec4.zig").UVec4;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;

/// This function will sort values from high to low and only keep the ones that are less than inMaxValue
/// @param values_in Values to be sorted
/// @param max_value Values need to be less than this to keep them
/// @param identifiers 4 identifiers that will be sorted in the same way as the values
/// @param out_values The values are stored here from high to low
/// @return The number of values that were kept
pub fn sortReverseAndStore(values_in: Vec4, max_value: f32, identifiers: *UVec4, out_values: *[4]f32) i32 {
    // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
    var values = values_in;
    Vec4.sort4Reverse(&values, identifiers);

    // Count how many results are less than the max value
    const closer = Vec4.less(values, Vec4.replicate(max_value));
    const num_results = closer.countTrues();

    // Shift the values so that only the ones that are less than max are kept
    values = values.reinterpretAsInt().shiftComponents4Minus(num_results).reinterpretAsFloat();
    identifiers.* = identifiers.shiftComponents4Minus(num_results);

    // Store the values
    values.storeFloat4(@ptrCast(out_values));

    return @intCast(num_results);
}

/// Shift the elements so that the identifiers that correspond with the trues in inValue come first
/// @param value Values to test for true or false
/// @param identifiers the identifiers that are shifted, on return they are shifted
/// @return The number of trues
pub fn countAndSortTrues(value: UVec4, identifiers: *UVec4) i32 {
    // Sort the hits
    identifiers.* = UVec4.sort4True(value, identifiers.*);

    // Return the amount of hits
    return @intCast(value.countTrues());
}

test "SortReverseAndStore / CountAndSortTrues" {
    const std = @import("std");

    var identifiers = UVec4.init(0, 1, 2, 3);
    var values: [4]f32 = undefined;
    const n = sortReverseAndStore(Vec4.init(3, 1, 4, 2), 3.5, &identifiers, &values);
    try std.testing.expectEqual(@as(i32, 3), n);
    // Sorted high to low: 4 (2), 3 (0), 2 (3), 1 (1), then shifted so that the 3 values below 3.5 come first
    try std.testing.expectEqualSlices(f32, &.{ 3, 2, 1 }, values[0..3]);
    try std.testing.expectEqual(@as(u32, 0), identifiers.getX());
    try std.testing.expectEqual(@as(u32, 3), identifiers.getY());
    try std.testing.expectEqual(@as(u32, 1), identifiers.getZ());

    var ids = UVec4.init(10, 11, 12, 13);
    const trues = countAndSortTrues(UVec4.init(0, 0xffffffff, 0, 0xffffffff), &ids);
    try std.testing.expectEqual(@as(i32, 2), trues);
    try std.testing.expectEqual(@as(u32, 11), ids.getX());
    try std.testing.expectEqual(@as(u32, 13), ids.getY());
    comptime std.debug.assert(@sizeOf(Float4) == @sizeOf([4]f32) and @alignOf(Float4) <= @alignOf([4]f32));
}
