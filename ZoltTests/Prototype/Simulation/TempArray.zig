//! Phase 5 architecture prototype: `TempArray(T)`, the typed TempAllocator block of
//! Docs/Zolt/SimulationArchitecture.md D5. The foundation (F1) adds it to Zolt/Core/TempAllocator.zig.
//!
//! An empty TempArray means "nothing allocated": the allocation failed, was never attempted (the update was poisoned)
//! or had size 0 (Jolt's Allocate(0) returns nullptr and Free(nullptr, 0) is a no-op). `free` skips empty arrays, so
//! freeing at Jolt's positions is always correct and keeps the TempAllocator's LIFO order after an out of memory error.

const std = @import("std");
const zolt = @import("zolt");
const TempAllocator = zolt.TempAllocator;

pub fn TempArray(comptime T: type) type {
    comptime std.debug.assert(@alignOf(T) <= zolt.rvector_alignment); // TempAllocator blocks are rvector aligned

    return struct {
        const Self = @This();

        items: []T = &.{},

        pub const empty: Self = .{};

        /// Allocate `n` uninitialized items (Jolt: (T *)inTempAllocator->Allocate(n * sizeof(T)))
        pub fn alloc(temp_allocator: TempAllocator, n: usize) TempAllocator.Error!Self {
            const block = try temp_allocator.allocate(@intCast(n * @sizeOf(T))) orelse return .empty;
            const ptr: [*]T = @ptrCast(@alignCast(block));
            return .{ .items = ptr[0..n] };
        }

        /// Free the block (Jolt: inTempAllocator->Free(p, n * sizeof(T))), nothing for an empty array
        pub fn free(self: *Self, temp_allocator: TempAllocator) void {
            if (self.items.len != 0)
                temp_allocator.free(@ptrCast(self.items.ptr), @intCast(self.items.len * @sizeOf(T)));
            self.* = .empty;
        }

        pub fn isEmpty(self: Self) bool {
            return self.items.len == 0;
        }
    };
}
