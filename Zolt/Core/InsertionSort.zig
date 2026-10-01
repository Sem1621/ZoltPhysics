//! Port of: Jolt/Core/InsertionSort.h
//! Status: complete
//!
//! The iterator pair becomes a slice and the comparator follows the std.sort convention:
//! `lessThan(context, a, b)`. Pass `{}` and `std.sort.asc(T)` for the C++ overload without a comparator.

const std = @import("std");

/// Implementation of the insertion sort algorithm.
pub fn insertionSort(
    comptime T: type,
    items: []T,
    context: anytype,
    comptime lessThan: fn (@TypeOf(context), T, T) bool,
) void {
    // Empty arrays don't need to be sorted
    if (items.len == 0)
        return;

    // Start at the second element
    for (1..items.len) |i| {
        // Move this element to a temporary value
        const x = items[i];

        // Check if the element goes before the first element (we can't decrement the iterator before begin so this needs to be a separate branch)
        if (lessThan(context, x, items[0])) {
            // Move all elements to the right to make space for x
            var j = i;
            while (j != 0) : (j -= 1)
                items[j] = items[j - 1];

            // Move x to the first place
            items[0] = x;
        } else {
            // Move elements to the right as long as they are bigger than x
            var j = i;
            while (lessThan(context, x, items[j - 1])) : (j -= 1)
                items[j] = items[j - 1];

            // Move x into place
            items[j] = x;
        }
    }
}
