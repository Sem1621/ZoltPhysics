//! Port of: Jolt/Core/STLLocalAllocator.h
//! Status: complete
//!
//! Jolt's `STLLocalAllocator<T, N>` is the allocator of an `Array<T, STLLocalAllocator<T, N>>`: the first N
//! elements live in a buffer inside the allocator, larger arrays fall back to the heap. Zolt's containers
//! take a `std.mem.Allocator`, so `STLLocalAllocator(T, N)` is a `std.mem.Allocator` implementation:
//! ```zig
//! var local = STLLocalAllocator(NodeID, stack_size).init(gpa); // gpa: the heap to fall back to
//! const allocator = local.allocator();
//! var node_stack: std.ArrayList(NodeID) = .empty;
//! defer node_stack.deinit(allocator);
//! ```
//! - The allocator contains the buffer, so it must stay at the same address while it is in use and it
//!   can't be shared by containers that are copied around (the C++ deletes the copy and move constructors).
//! - Jolt's heap is its global allocator (the `STLAllocator<T>` base class), in Zolt the `backing_allocator`.
//! - The std.mem.Allocator functions follow the C++ functions: `alloc` is `allocate`, `remap` is
//!   `reallocate` (in place in the local buffer when possible, otherwise to the heap, never back into the
//!   local buffer, exactly like the C++), `resize` is the in place part of `reallocate` and `free` is
//!   `deallocate`. Heap blocks are reallocated with the heap's `remap` (Jolt: `Base::reallocate`) or by
//!   allocate + copy + free (`ReallocateImpl`).
//! - The C++ allocator is typed: it only serves T elements. A std.mem.Allocator can get any request, so
//!   requests that are not a whole number of T elements or that need a larger alignment than T go straight
//!   to the heap, like the C++ rebind constructor (an allocator for another type) does.
//! - Elements are moved with memcpy, the `std::is_trivially_copyable` branch of ReallocateImpl (Zig has
//!   no move constructors).
//! - The element based C++ functions (`allocate`, `reallocate`, `deallocate`, `isLocal`) are kept.
//! - Not ported: the STL typedefs (`value_type`, `pointer`, ..., `is_always_equal`,
//!   `propagate_on_container_*`) and `rebind`, which have no meaning for std.mem.Allocator, the
//!   `AllocatorHasReallocate` specialization (`has_reallocate` is a plain constant) and the
//!   `JPH_DISABLE_CUSTOM_ALLOCATOR` fallback to `std::allocator`.

const std = @import("std");
const Alignment = std.mem.Alignment;

/// STL allocator that keeps N elements in a local buffer before falling back to regular allocations
pub fn STLLocalAllocator(comptime T: type, comptime N: usize) type {
    comptime std.debug.assert(@sizeOf(T) > 0);

    return struct {
        const Self = @This();

        /// The number of elements in the local buffer (template parameter N)
        pub const num_local_elements = N;

        /// Always implements a reallocate function as we can often reallocate in place
        pub const has_reallocate = true;

        /// Alignment that the local buffer provides (and that heap allocations of T elements use)
        const element_alignment: Alignment = .of(T);

        /// Local buffer for N elements (mElements)
        elements: [N * @sizeOf(T)]u8 align(@alignOf(T)) = undefined,

        /// Number of elements of the local buffer that are in use (mNumElementsUsed)
        num_elements_used: usize = 0,

        /// The heap to fall back to (Jolt: its global allocator through STLAllocator<T>)
        backing_allocator: std.mem.Allocator,

        /// Constructor
        pub fn init(backing_allocator: std.mem.Allocator) Self {
            return .{ .backing_allocator = backing_allocator };
        }

        /// Constructor used when rebinding to another type (`other` is a `*const STLLocalAllocator(T2, N)`).
        /// This expects the allocator to use the original memory pool from the first allocator,
        /// but in our case we cannot use the local buffer of the original allocator as it has different size and alignment rules.
        /// To solve this we make this allocator fall back to the heap immediately.
        pub fn fromOther(other: anytype) Self {
            comptime std.debug.assert(@TypeOf(other.*).num_local_elements == N);
            return .{ .num_elements_used = N, .backing_allocator = other.backing_allocator };
        }

        /// The std.mem.Allocator interface. `self` must stay at the same address while it is in use.
        pub fn allocator(self: *Self) std.mem.Allocator {
            return .{ .ptr = self, .vtable = &vtable };
        }

        /// Check if pointer is in the local buffer (is_local)
        pub fn isLocal(self: *const Self, pointer: *const anyopaque) bool {
            const address = @intFromPtr(pointer);
            const base = @intFromPtr(&self.elements);
            return address >= base and address - base < N * @sizeOf(T);
        }

        /// Allocate memory for n elements
        pub fn allocate(self: *Self, n: usize) std.mem.Allocator.Error![*]T {
            const len = std.math.mul(usize, n, @sizeOf(T)) catch return error.OutOfMemory;
            const result = self.allocBytes(len, element_alignment, @returnAddress()) orelse return error.OutOfMemory;
            return @ptrCast(@alignCast(result));
        }

        /// Reallocate memory
        pub fn reallocate(self: *Self, old_pointer: ?[*]T, old_size: usize, new_size: usize) std.mem.Allocator.Error![*]T {
            std.debug.assert(new_size > 0); // Reallocating to zero size is implementation dependent, so we don't allow it

            // If there was no previous allocation, we can go through the regular allocate function
            const old = old_pointer orelse return self.allocate(new_size);

            const old_memory = @as([*]u8, @ptrCast(old))[0 .. old_size * @sizeOf(T)];
            const new_len = std.math.mul(usize, new_size, @sizeOf(T)) catch return error.OutOfMemory;
            const result = self.reallocBytes(old_memory, element_alignment, new_len, @returnAddress()) orelse return error.OutOfMemory;
            return @ptrCast(@alignCast(result));
        }

        /// Free memory
        pub fn deallocate(self: *Self, pointer: [*]T, n: usize) void {
            self.freeBytes(@as([*]u8, @ptrCast(pointer))[0 .. n * @sizeOf(T)], element_alignment, @returnAddress());
        }

        /// Allocators are not-stateless, assume if allocator address matches that the allocators are the same
        pub fn eql(self: *const Self, other: *const Self) bool {
            return self == other;
        }

        /// Whether a request for len bytes with alignment is for whole T elements, which the local buffer can serve
        fn isElementRequest(len: usize, alignment: Alignment) bool {
            return len % @sizeOf(T) == 0 and alignment.compare(.lte, element_alignment);
        }

        /// Address of element index of the local buffer (wraps like the C++ pointer arithmetic)
        fn localAddress(self: *const Self, index: usize) usize {
            return @intFromPtr(&self.elements) +% index *% @sizeOf(T);
        }

        /// allocate(inN) in bytes
        fn allocBytes(self: *Self, len: usize, alignment: Alignment, ret_addr: usize) ?[*]u8 {
            // Requests for something else than T elements can't use the local buffer (like a rebound allocator)
            if (!isElementRequest(len, alignment))
                return self.backing_allocator.rawAlloc(len, alignment, ret_addr);
            const n = len / @sizeOf(T);

            // If we allocate more than we have, fall back to the heap
            // (mNumElementsUsed + inN > N, written so that it can't overflow)
            if (n > N - self.num_elements_used)
                return self.backing_allocator.rawAlloc(len, alignment, ret_addr);

            // Allocate from our local buffer
            const result: [*]u8 = @ptrFromInt(self.localAddress(self.num_elements_used));
            self.num_elements_used += n;
            return result;
        }

        /// The in place part of reallocate for a block in the local buffer, returns false if the block can't change size in place
        fn resizeLocal(self: *Self, memory: []u8, alignment: Alignment, new_len: usize) bool {
            if (!isElementRequest(new_len, alignment))
                return false;
            const old_size = memory.len / @sizeOf(T);
            const new_size = new_len / @sizeOf(T);

            // If we happen to have space left, we only need to update our bookkeeping
            // (unsigned arithmetic like the C++, the difference wraps when shrinking)
            if (@intFromPtr(memory.ptr) == self.localAddress(self.num_elements_used -% old_size) and self.num_elements_used -% old_size +% new_size <= N) {
                self.num_elements_used +%= new_size -% old_size;
                return true;
            }
            return false;
        }

        /// reallocate(inOldPointer, inOldSize, inNewSize) in bytes, memory is not empty
        fn reallocBytes(self: *Self, memory: []u8, alignment: Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
            std.debug.assert(new_len > 0); // Reallocating to zero size is implementation dependent, so we don't allow it

            // If the pointer is outside our local buffer, fall back to the heap
            if (!self.isLocal(memory.ptr)) {
                // Jolt: Base::reallocate if the heap allocator has it, otherwise ReallocateImpl
                if (self.backing_allocator.rawRemap(memory, alignment, new_len, ret_addr)) |new_pointer|
                    return new_pointer;
                return self.reallocateImpl(memory, alignment, new_len, ret_addr);
            }

            // If we happen to have space left, we only need to update our bookkeeping
            if (self.resizeLocal(memory, alignment, new_len))
                return memory.ptr;

            // We can't reallocate in place, fall back to the heap
            return self.reallocateImpl(memory, alignment, new_len, ret_addr);
        }

        /// deallocate(inPointer, inN) in bytes
        fn freeBytes(self: *Self, memory: []u8, alignment: Alignment, ret_addr: usize) void {
            // If the pointer is not in our local buffer, fall back to the heap
            if (!self.isLocal(memory.ptr))
                return self.backing_allocator.rawFree(memory, alignment, ret_addr);

            // Else we can only reclaim memory if it was the last allocation
            const n = memory.len / @sizeOf(T);
            if (@intFromPtr(memory.ptr) == self.localAddress(self.num_elements_used -% n))
                self.num_elements_used -= n;
        }

        /// Implements reallocate when the base class doesn't or when we go from local buffer to heap
        fn reallocateImpl(self: *Self, memory: []u8, alignment: Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
            const new_pointer = self.backing_allocator.rawAlloc(new_len, alignment, ret_addr) orelse return null;
            const n = @min(memory.len, new_len);

            // Can use mem copy (Zig types are trivially copyable)
            @memcpy(new_pointer[0..n], memory[0..n]);

            self.freeBytes(memory, alignment, ret_addr);
            return new_pointer;
        }

        const vtable: std.mem.Allocator.VTable = .{
            .alloc = allocFn,
            .resize = resizeFn,
            .remap = remapFn,
            .free = freeFn,
        };

        fn allocFn(ptr: *anyopaque, len: usize, alignment: Alignment, ret_addr: usize) ?[*]u8 {
            const self: *Self = @ptrCast(@alignCast(ptr));
            return self.allocBytes(len, alignment, ret_addr);
        }

        fn resizeFn(ptr: *anyopaque, memory: []u8, alignment: Alignment, new_len: usize, ret_addr: usize) bool {
            const self: *Self = @ptrCast(@alignCast(ptr));
            if (!self.isLocal(memory.ptr))
                return self.backing_allocator.rawResize(memory, alignment, new_len, ret_addr);
            return self.resizeLocal(memory, alignment, new_len);
        }

        fn remapFn(ptr: *anyopaque, memory: []u8, alignment: Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
            const self: *Self = @ptrCast(@alignCast(ptr));
            return self.reallocBytes(memory, alignment, new_len, ret_addr);
        }

        fn freeFn(ptr: *anyopaque, memory: []u8, alignment: Alignment, ret_addr: usize) void {
            const self: *Self = @ptrCast(@alignCast(ptr));
            self.freeBytes(memory, alignment, ret_addr);
        }
    };
}

test "STLLocalAllocator element functions" {
    const Allocator = STLLocalAllocator(u32, 8);
    comptime std.debug.assert(Allocator.has_reallocate);
    var local = Allocator.init(std.testing.allocator);
    try std.testing.expect(local.eql(&local));
    const base = @intFromPtr(&local.elements);

    // Allocations are served from the local buffer while they fit
    const a = try local.allocate(3);
    try std.testing.expectEqual(base, @intFromPtr(a));
    try std.testing.expect(local.isLocal(a));
    const b = try local.allocate(2);
    try std.testing.expectEqual(base + 3 * 4, @intFromPtr(b));
    try std.testing.expectEqual(@as(usize, 5), local.num_elements_used);

    // Doesn't fit anymore: heap
    const c = try local.allocate(4);
    try std.testing.expect(!local.isLocal(c));
    try std.testing.expectEqual(@as(usize, 5), local.num_elements_used);
    local.deallocate(c, 4);

    // The last allocation can grow and shrink in place
    var b2 = try local.reallocate(b, 2, 5);
    try std.testing.expectEqual(@intFromPtr(b), @intFromPtr(b2));
    try std.testing.expectEqual(@as(usize, 8), local.num_elements_used);
    b2 = try local.reallocate(b2, 5, 1);
    try std.testing.expectEqual(@intFromPtr(b), @intFromPtr(b2));
    try std.testing.expectEqual(@as(usize, 4), local.num_elements_used);

    // An allocation that is not the last one moves to the heap when reallocated, even when shrinking.
    // Its space in the local buffer is not reclaimed.
    a[0] = 10;
    a[1] = 11;
    a[2] = 12;
    const a2 = try local.reallocate(a, 3, 2);
    try std.testing.expect(!local.isLocal(a2));
    try std.testing.expectEqual(@as(u32, 10), a2[0]);
    try std.testing.expectEqual(@as(u32, 11), a2[1]);
    try std.testing.expectEqual(@as(usize, 4), local.num_elements_used);

    // Growing the last allocation beyond the local buffer moves it to the heap and reclaims its space
    b2[0] = 20;
    const b3 = try local.reallocate(b2, 1, 6);
    try std.testing.expect(!local.isLocal(b3));
    try std.testing.expectEqual(@as(u32, 20), b3[0]);
    try std.testing.expectEqual(@as(usize, 3), local.num_elements_used);

    // Heap blocks stay on the heap
    const b4 = try local.reallocate(b3, 6, 1);
    try std.testing.expect(!local.isLocal(b4));
    try std.testing.expectEqual(@as(u32, 20), b4[0]);
    local.deallocate(b4, 1);
    local.deallocate(a2, 2);

    // Reallocating null allocates
    const d = try local.reallocate(null, 0, 5);
    try std.testing.expectEqual(base + 3 * 4, @intFromPtr(d));
    try std.testing.expectEqual(@as(usize, 8), local.num_elements_used);
    local.deallocate(d, 5);
    try std.testing.expectEqual(@as(usize, 3), local.num_elements_used);

    // Pointers just outside the buffer are not local
    try std.testing.expect(!local.isLocal(@ptrFromInt(base + 8 * 4)));
    try std.testing.expect(local.isLocal(@ptrFromInt(base + 8 * 4 - 1)));
    try std.testing.expect(!local.isLocal(@ptrFromInt(base - 1)));
}

test "STLLocalAllocator rebind falls back to the heap" {
    var local = STLLocalAllocator(u32, 8).init(std.testing.allocator);
    var rebound = STLLocalAllocator(u16, 8).fromOther(&local);
    try std.testing.expectEqual(@as(usize, 8), rebound.num_elements_used);
    const p = try rebound.allocate(1);
    try std.testing.expect(!rebound.isLocal(p));
    rebound.deallocate(p, 1);
}

test "STLLocalAllocator as std.mem.Allocator" {
    var local = STLLocalAllocator(u64, 64).init(std.testing.allocator);
    const allocator = local.allocator();

    // std.ArrayList with its own growth policy starts in the local buffer, grows in place and moves to the heap
    var list: std.ArrayList(u64) = .empty;
    try list.append(allocator, 1);
    try std.testing.expect(local.isLocal(list.items.ptr));
    try std.testing.expectEqual(list.capacity, local.num_elements_used);
    const first_data = list.items.ptr;
    const first_capacity = list.capacity;
    while (list.items.len <= first_capacity)
        try list.append(allocator, list.items.len + 1);
    try std.testing.expect(list.capacity > first_capacity and list.capacity <= 64);
    try std.testing.expectEqual(first_data, list.items.ptr);
    try std.testing.expectEqual(list.capacity, local.num_elements_used);
    while (list.items.len < 100)
        try list.append(allocator, list.items.len + 1);
    try std.testing.expect(!local.isLocal(list.items.ptr));
    try std.testing.expectEqual(@as(usize, 0), local.num_elements_used);
    for (list.items, 1..) |v, i| try std.testing.expectEqual(@as(u64, i), v);
    list.deinit(allocator);

    // In place resize of the last local block
    const block = try allocator.alloc(u64, 4);
    try std.testing.expect(allocator.resize(block, 64));
    try std.testing.expectEqual(@as(usize, 64), local.num_elements_used);
    const block64: []u64 = block.ptr[0..64];
    try std.testing.expect(!allocator.resize(block64, 65));
    try std.testing.expectEqual(@as(usize, 64), local.num_elements_used);
    const block2 = allocator.remap(block64, 2).?;
    try std.testing.expectEqual(@intFromPtr(block.ptr), @intFromPtr(block2.ptr));
    try std.testing.expectEqual(@as(usize, 2), local.num_elements_used);
    allocator.free(block2);
    try std.testing.expectEqual(@as(usize, 0), local.num_elements_used);

    // Requests for something else than whole elements, or with a larger alignment, use the heap
    const bytes = try allocator.alloc(u8, 3);
    try std.testing.expect(!local.isLocal(bytes.ptr));
    allocator.free(bytes);
    const aligned = try allocator.alignedAlloc(u64, .@"64", 2);
    try std.testing.expect(!local.isLocal(aligned.ptr));
    try std.testing.expect(std.mem.isAligned(@intFromPtr(aligned.ptr), 64));
    allocator.free(aligned);
    try std.testing.expectEqual(@as(usize, 0), local.num_elements_used);

    // Whole elements of a smaller type can use the local buffer
    const halves = try allocator.alloc(u32, 4);
    try std.testing.expect(local.isLocal(halves.ptr));
    try std.testing.expectEqual(@as(usize, 2), local.num_elements_used);
    allocator.free(halves);
    try std.testing.expectEqual(@as(usize, 0), local.num_elements_used);

    // A local block that can't be resized in place can't be resized, but it can be remapped to the heap
    const first = try allocator.alloc(u64, 2);
    const second = try allocator.alloc(u64, 2);
    try std.testing.expect(!allocator.resize(first, 3));
    first[0] = 42;
    const moved = allocator.remap(first, 3).?;
    try std.testing.expect(!local.isLocal(moved.ptr));
    try std.testing.expectEqual(@as(u64, 42), moved[0]);
    allocator.free(moved);
    allocator.free(second);
    try std.testing.expectEqual(@as(usize, 2), local.num_elements_used); // first was not the last block, its space is lost

    // Out of memory of the heap
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 0 });
    var failing_local = STLLocalAllocator(u64, 2).init(failing.allocator());
    try std.testing.expectError(error.OutOfMemory, failing_local.allocate(3));
    const ok = try failing_local.allocate(2);
    try std.testing.expectError(error.OutOfMemory, failing_local.reallocate(ok, 2, 3));
    try std.testing.expectEqual(@as(usize, 2), failing_local.num_elements_used);
    failing_local.deallocate(ok, 2);
}
