//! Port of: Jolt/Core/QuickSort.h
//! Status: complete
//!
//! The STL version implementation is not consistent across platforms, which is why Jolt has its own
//! (and why Zolt must use this one, not std.sort, wherever Jolt calls QuickSort: the order of equal
//! elements affects simulation results). The iterator pair becomes a slice and the comparator follows
//! the std.sort convention: `lessThan(context, a, b)`. Pass `{}` and `std.sort.asc(T)` for the C++
//! overload without a comparator.

const std = @import("std");
const insertionSort = @import("InsertionSort.zig").insertionSort;

/// Helper function for QuickSort, will move the pivot element to `middle` (all arguments are indices into items)
fn quickSortMedianOfThree(
    comptime T: type,
    items: []T,
    first: usize,
    middle: usize,
    last: usize,
    context: anytype,
    comptime lessThan: fn (@TypeOf(context), T, T) bool,
) void {
    // This should be guaranteed because we switch over to insertion sort when there's 32 or less elements
    std.debug.assert(first != middle and middle != last);

    if (lessThan(context, items[middle], items[first]))
        std.mem.swap(T, &items[first], &items[middle]);

    if (lessThan(context, items[last], items[first]))
        std.mem.swap(T, &items[first], &items[last]);

    if (lessThan(context, items[last], items[middle]))
        std.mem.swap(T, &items[middle], &items[last]);
}

/// Helper function for QuickSort using the Ninther method, will move the pivot element to `middle` (all arguments are indices into items)
fn quickSortNinther(
    comptime T: type,
    items: []T,
    first: usize,
    middle: usize,
    last: usize,
    context: anytype,
    comptime lessThan: fn (@TypeOf(context), T, T) bool,
) void {
    // Divide the range in 8 equal parts (this means there are 9 points)
    const diff = (last - first) >> 3;
    const two_diff = diff << 1;

    // Median of first 3 points
    const mid1 = first + diff;
    quickSortMedianOfThree(T, items, first, mid1, first + two_diff, context, lessThan);

    // Median of second 3 points
    quickSortMedianOfThree(T, items, middle - diff, middle, middle + diff, context, lessThan);

    // Median of third 3 points
    const mid3 = last - diff;
    quickSortMedianOfThree(T, items, last - two_diff, mid3, last, context, lessThan);

    // Determine the median of the 3 medians
    quickSortMedianOfThree(T, items, mid1, middle, mid3, context, lessThan);
}

/// Implementation of the quick sort algorithm. The STL version implementation is not consistent across platforms.
pub fn quickSort(
    comptime T: type,
    items_in: []T,
    context: anytype,
    comptime lessThan: fn (@TypeOf(context), T, T) bool,
) void {
    // Implementation based on https://en.wikipedia.org/wiki/Quicksort using Hoare's partition scheme

    // Loop so that we only need to do 1 recursive call instead of 2.
    var items = items_in;
    while (true) {
        // If there's less than 2 elements we're done
        const num_elements = items.len;
        if (num_elements < 2)
            return;

        // Fall back to insertion sort if there are too few elements
        if (num_elements <= 32) {
            insertionSort(T, items, context, lessThan);
            return;
        }

        // Determine pivot
        const pivot_index = (num_elements - 1) >> 1;
        quickSortNinther(T, items, 0, pivot_index, num_elements - 1, context, lessThan);
        const pivot = items[pivot_index];

        // Left and right indices
        var i: usize = 0;
        var j: usize = num_elements;

        while (true) {
            // Find the first element that is bigger than the pivot
            while (lessThan(context, items[i], pivot))
                i += 1;

            // Find the last element that is smaller than the pivot
            j -= 1;
            while (lessThan(context, pivot, items[j]))
                j -= 1;

            // If the two indices crossed, we're done
            if (i >= j)
                break;

            // Swap the elements
            std.mem.swap(T, &items[i], &items[j]);

            // Note that the first while loop in this function should
            // have been do i++ while (...) but since we cannot decrement
            // the iterator from begin we left that out, so we need to do
            // it here.
            i += 1;
        }

        // Include the middle element on the left side
        j += 1;

        // Check which partition is smaller
        if (j < num_elements - j) {
            // Left side is smaller, recurse to left first
            quickSort(T, items[0..j], context, lessThan);

            // Loop again with the right side to avoid a call
            items = items[j..];
        } else {
            // Right side is smaller, recurse to right first
            quickSort(T, items[j..], context, lessThan);

            // Loop again with the left side to avoid a call
            items = items[0..j];
        }
    }
}
