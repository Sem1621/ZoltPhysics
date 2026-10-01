//! Port of: Jolt/Core/Atomics.h
//! Status: complete
//!
//! `std::atomic<T>` is `std.atomic.Value(T)`, see "Threading" in the porting guide for memory orders.

const std = @import("std");
const AtomicOrder = std.builtin.AtomicOrder;

/// Failure order that C++ derives for a compare_exchange with a single memory order
fn failureOrder(comptime order: AtomicOrder) AtomicOrder {
    return switch (order) {
        .acq_rel => .acquire,
        .release => .monotonic,
        else => order,
    };
}

/// Atomically compute the min(atomic, value) and store it in atomic, returns true if value was updated
pub fn atomicMin(comptime T: type, atomic: *std.atomic.Value(T), value: T, comptime opts: struct { order: AtomicOrder = .seq_cst }) bool {
    var cur_value = atomic.load(.monotonic);
    while (cur_value > value) {
        cur_value = atomic.cmpxchgWeak(cur_value, value, opts.order, comptime failureOrder(opts.order)) orelse return true;
    }
    return false;
}

/// Atomically compute the max(atomic, value) and store it in atomic, returns true if value was updated
pub fn atomicMax(comptime T: type, atomic: *std.atomic.Value(T), value: T, comptime opts: struct { order: AtomicOrder = .seq_cst }) bool {
    var cur_value = atomic.load(.monotonic);
    while (cur_value < value) {
        cur_value = atomic.cmpxchgWeak(cur_value, value, opts.order, comptime failureOrder(opts.order)) orelse return true;
    }
    return false;
}

test "atomicMin / atomicMax" {
    var a: std.atomic.Value(u32) = .init(10);
    try std.testing.expect(!atomicMin(u32, &a, 20, .{}));
    try std.testing.expect(atomicMin(u32, &a, 5, .{ .order = .acq_rel }));
    try std.testing.expectEqual(@as(u32, 5), a.load(.monotonic));
    try std.testing.expect(!atomicMax(u32, &a, 1, .{}));
    try std.testing.expect(atomicMax(u32, &a, 30, .{ .order = .release }));
    try std.testing.expectEqual(@as(u32, 30), a.load(.monotonic));
}
