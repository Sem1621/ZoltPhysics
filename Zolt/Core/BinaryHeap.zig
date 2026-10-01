//! Port of: Jolt/Core/BinaryHeap.h
//! Status: complete
//!
//! The iterator pair becomes a slice and the predicate follows the std.sort convention:
//! `pred(context, a, b)` returns true if a is less or equal than b.

const std = @import("std");

/// Push a new element into a binary max-heap.
/// items[0 .. len - 1] must be a a valid heap. Element len - 1 will be inserted into the heap. The heap will be items[0 .. len] after this call.
/// pred is a function that returns true if the first element is less or equal than the second element.
/// See: https://en.wikipedia.org/wiki/Binary_heap
pub fn binaryHeapPush(
    comptime T: type,
    items: []T,
    context: anytype,
    comptime pred: fn (@TypeOf(context), T, T) bool,
) void {
    // New heap size
    const count = items.len;
    if (count == 0)
        return;

    // Start from the last element
    var current = count - 1;
    while (current > 0) {
        // Get parent element
        const parent = (current - 1) >> 1;

        // Sort them so that the parent is larger than the child
        if (pred(context, items[parent], items[current])) {
            std.mem.swap(T, &items[parent], &items[current]);
            current = parent;
        } else {
            // When there's no change, we're done
            break;
        }
    }
}

/// Pop an element from a binary max-heap.
/// items must be a valid heap. The largest element will be removed from the heap (moved to the last position).
/// The heap will be items[0 .. len - 1] after this call.
/// pred is a function that returns true if the first element is less or equal than the second element.
/// See: https://en.wikipedia.org/wiki/Binary_heap
pub fn binaryHeapPop(
    comptime T: type,
    items: []T,
    context: anytype,
    comptime pred: fn (@TypeOf(context), T, T) bool,
) void {
    std.debug.assert(items.len > 0);

    // Begin by moving the highest element to the end, this is the popped element
    std.mem.swap(T, &items[items.len - 1], &items[0]);

    // New heap size
    const count = items.len - 1;

    // Start from the root
    var largest: usize = 0;
    while (true) {
        // Get first child
        var child = (largest << 1) + 1;

        // Check if we're beyond the end of the heap, if so the 2nd child is also beyond the end
        if (child >= count)
            break;

        // Remember the largest element from the previous iteration
        const prev_largest = largest;

        // Check if first child is bigger, if so select it
        if (pred(context, items[largest], items[child]))
            largest = child;

        // Switch to the second child
        child += 1;

        // Check if second child is bigger, if so select it
        if (child < count and pred(context, items[largest], items[child]))
            largest = child;

        // If there was no change, we're done
        if (prev_largest == largest)
            break;

        // Swap element
        std.mem.swap(T, &items[prev_largest], &items[largest]);
    }
}
