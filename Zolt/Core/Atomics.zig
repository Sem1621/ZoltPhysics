//! Port of: Jolt/Core/Atomics.h
//! Status: complete
//!
//! `std::atomic<T>` is `std.atomic.Value(T)`, see "Threading" in the porting guide for memory orders.
//! `atomicMin` / `atomicMax` also take floats (QuadTree's `atomic<float>` node bounds): Zig's `@cmpxchgWeak` does not
//! accept floats, so the compare exchange runs on the bits, which is what C++'s `compare_exchange_weak` compares (the
//! object representation).

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

/// compare_exchange_weak: null on success, otherwise the current value. Floats are compared and exchanged as bits
/// (C++ compares the object representation, Zig's @cmpxchgWeak takes no floats).
fn cmpxchgWeak(comptime T: type, atomic: *std.atomic.Value(T), expected_value: T, new_value: T, comptime success_order: AtomicOrder, comptime fail_order: AtomicOrder) ?T {
    if (@typeInfo(T) == .float) {
        const Bits = std.meta.Int(.unsigned, @bitSizeOf(T));
        const bits: *Bits = @ptrCast(&atomic.raw);
        const current = @cmpxchgWeak(Bits, bits, @as(Bits, @bitCast(expected_value)), @as(Bits, @bitCast(new_value)), success_order, fail_order) orelse return null;
        return @bitCast(current);
    }
    return atomic.cmpxchgWeak(expected_value, new_value, success_order, fail_order);
}

/// Atomically compute the min(atomic, value) and store it in atomic, returns true if value was updated
pub fn atomicMin(comptime T: type, atomic: *std.atomic.Value(T), value: T, comptime opts: struct { order: AtomicOrder = .seq_cst }) bool {
    var cur_value = atomic.load(.monotonic);
    while (cur_value > value) {
        cur_value = cmpxchgWeak(T, atomic, cur_value, value, opts.order, comptime failureOrder(opts.order)) orelse return true;
    }
    return false;
}

/// Atomically compute the max(atomic, value) and store it in atomic, returns true if value was updated
pub fn atomicMax(comptime T: type, atomic: *std.atomic.Value(T), value: T, comptime opts: struct { order: AtomicOrder = .seq_cst }) bool {
    var cur_value = atomic.load(.monotonic);
    while (cur_value < value) {
        cur_value = cmpxchgWeak(T, atomic, cur_value, value, opts.order, comptime failureOrder(opts.order)) orelse return true;
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

    // Floats (QuadTree's node bounds)
    var f: std.atomic.Value(f32) = .init(1.5);
    try std.testing.expect(!atomicMin(f32, &f, 2.0, .{}));
    try std.testing.expect(atomicMin(f32, &f, -0.25, .{}));
    try std.testing.expectEqual(@as(f32, -0.25), f.load(.monotonic));
    try std.testing.expect(!atomicMax(f32, &f, -1.0, .{}));
    try std.testing.expect(atomicMax(f32, &f, 1.0e15, .{}));
    try std.testing.expectEqual(@as(f32, 1.0e15), f.load(.monotonic));
    try std.testing.expect(!atomicMax(f32, &f, std.math.nan(f32), .{})); // NaN never compares greater
}
