//! Port of: Jolt/Core/STLTempAllocator.h
//! Status: complete
//!
//! Jolt's `STLTempAllocator<T>` lets STL containers (`Array<T, STLTempAllocator<T>>`) allocate from a
//! TempAllocator. Zolt's containers take a `std.mem.Allocator`, so the adapter is a `std.mem.Allocator`
//! implementation on top of the TempAllocator implementation:
//! ```zig
//! const gpa = temp_allocator.allocator(); // or STLTempAllocator.init(temp_allocator).allocator()
//! var list: std.ArrayList(T) = .empty;
//! defer list.deinit(gpa);
//! try list.ensureTotalCapacityPrecise(gpa, n); // Jolt: list.reserve(n)
//! ```
//! - std.mem.Allocator counts bytes, so the element type parameter disappears, and with it `rebind`, the
//!   converting constructor and the STL typedefs. `allocate(inN)` / `deallocate(inPointer, inN)` are
//!   `alloc(T, n)` / `free(slice)` of the std.mem.Allocator, which call TempAllocator `allocate` / `free`
//!   with the size in bytes.
//! - Every container allocation is a block of the temp allocator, so blocks must be freed in the reverse
//!   order of allocation.
//! - Jolt's adapter has no reallocate: a growing Array allocates a new block, moves the elements and frees
//!   the old block, which TempAllocatorImpl rejects (wrong order). Jolt code therefore reserves the final
//!   capacity upfront; do the same with `ensureTotalCapacityPrecise` (std's growth policy rounds the
//!   capacity up, which would change the memory usage and the point where the temp allocator runs out of
//!   memory). Through the Zolt `TempAllocator.resize` extension, `resize` / `remap` additionally succeed
//!   when the implementation can resize the block in place (the most recent block of a TempAllocatorImpl).
//! - `alloc` asserts that the requested alignment is at most rvector_alignment (JPH_RVECTOR_ALIGNMENT),
//!   which is all that a TempAllocator guarantees. A size that doesn't fit in a uint fails with out of
//!   memory (the C++ truncates it).

const std = @import("std");
const Core = @import("Core.zig");
const TempAllocator = @import("TempAllocator.zig").TempAllocator;
const TempAllocatorImpl = @import("TempAllocator.zig").TempAllocatorImpl;
const TempAllocatorImplWithMallocFallback = @import("TempAllocator.zig").TempAllocatorImplWithMallocFallback;
const TempAllocatorMalloc = @import("TempAllocator.zig").TempAllocatorMalloc;
const rvector_alignment = @import("../Math/Real.zig").rvector_alignment;

/// STL allocator that wraps around TempAllocator
pub const STLTempAllocator = struct {
    /// Our temp allocator (mAllocator)
    temp_allocator: TempAllocator,

    /// Constructor
    pub fn init(temp_allocator: TempAllocator) STLTempAllocator {
        return .{ .temp_allocator = temp_allocator };
    }

    /// The std.mem.Allocator that allocates (allocate) and frees (deallocate) from the temp allocator
    pub fn allocator(self: STLTempAllocator) std.mem.Allocator {
        return self.temp_allocator.allocator();
    }

    /// Allocators are not-stateless, assume if allocator address matches that the allocators are the same
    pub fn eql(self: STLTempAllocator, other: STLTempAllocator) bool {
        // Jolt compares the addresses of the TempAllocator objects. In Zolt the identity of a TempAllocator is
        // ptr + vtable: an implementation can share its address with a nested one (the TempAllocatorImpl inside
        // a TempAllocatorImplWithMallocFallback), only the vtable tells them apart.
        return self.temp_allocator.ptr == other.temp_allocator.ptr and self.temp_allocator.vtable == other.temp_allocator.vtable;
    }

    /// Get our temp allocator
    pub fn getAllocator(self: STLTempAllocator) TempAllocator {
        return self.temp_allocator;
    }
};

/// The std.mem.Allocator vtable that forwards to TempAllocator implementation T, the std.mem.Allocator's
/// `ptr` is the `*T`. `TempAllocator.init` stores it in the TempAllocator vtable, so that
/// `TempAllocator.allocator()` doesn't need an object that stays at a fixed address.
pub fn stdAllocatorVTable(comptime T: type) std.mem.Allocator.VTable {
    const gen = struct {
        fn alloc(ptr: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
            _ = ret_addr;
            const self: *T = @ptrCast(@alignCast(ptr));

            // A TempAllocator only guarantees JPH_RVECTOR_ALIGNMENT
            std.debug.assert(alignment.compare(.lte, .fromByteUnits(rvector_alignment)));

            // Jolt: mAllocator.Allocate(uint(inN * sizeof(value_type)))
            const size = std.math.cast(u32, len) orelse return null;
            return self.allocate(size) catch null;
        }

        fn resize(ptr: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) bool {
            _ = alignment;
            _ = ret_addr;
            if (comptime @hasDecl(T, "resize")) {
                const self: *T = @ptrCast(@alignCast(ptr));
                const new_size = std.math.cast(u32, new_len) orelse return false;
                return self.resize(memory.ptr, @intCast(memory.len), new_size);
            } else {
                // Like Jolt's STLTempAllocator, which has no reallocate
                return false;
            }
        }

        fn remap(ptr: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
            return if (resize(ptr, memory, alignment, new_len, ret_addr)) memory.ptr else null;
        }

        fn free(ptr: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
            _ = alignment;
            _ = ret_addr;
            const self: *T = @ptrCast(@alignCast(ptr));

            // Jolt: mAllocator.Free(inPointer, uint(inN * sizeof(value_type)))
            self.free(memory.ptr, @intCast(memory.len));
        }
    };
    return .{ .alloc = gen.alloc, .resize = gen.resize, .remap = gen.remap, .free = gen.free };
}

// Size of a block of the temp allocator (JPH_RVECTOR_ALIGNMENT)
const a16: usize = if (Core.double_precision) 32 else 16;

test "STLTempAllocator on TempAllocatorImpl" {
    var impl = try TempAllocatorImpl.init(std.testing.allocator, 1024);
    defer impl.deinit();

    const stl = STLTempAllocator.init(impl.tempAllocator());
    try std.testing.expect(stl.eql(STLTempAllocator.init(impl.tempAllocator())));
    try std.testing.expectEqual(@intFromPtr(&impl), @intFromPtr(stl.getAllocator().ptr));
    const gpa = stl.allocator();

    // Arrays reserve their final size, like Jolt's Array<T, STLTempAllocator<T>> users do
    var a: std.ArrayList(u32) = .empty;
    try a.ensureTotalCapacityPrecise(gpa, 5); // 20 bytes
    try std.testing.expectEqual(@intFromPtr(impl.base), @intFromPtr(a.items.ptr));
    try std.testing.expectEqual(std.mem.alignForward(usize, 20, a16), impl.getUsage()); // AlignUp(20)
    for (0..5) |i| a.appendAssumeCapacity(@intCast(i));

    var b: std.ArrayList(u64) = .empty;
    try b.ensureTotalCapacityPrecise(gpa, 3); // 24 bytes
    try std.testing.expectEqual(@intFromPtr(impl.base) + std.mem.alignForward(usize, 20, a16), @intFromPtr(b.items.ptr));
    try std.testing.expectEqual(std.mem.alignForward(usize, 20, a16) + std.mem.alignForward(usize, 24, a16), impl.getUsage());

    // b is the most recent block, so it can grow in place (Zolt extension)
    for (0..10) |i| try b.append(gpa, i);
    try std.testing.expectEqual(@intFromPtr(impl.base) + std.mem.alignForward(usize, 20, a16), @intFromPtr(b.items.ptr));
    try std.testing.expectEqual(std.mem.alignForward(usize, 20, a16) + std.mem.alignForward(usize, b.capacity * 8, a16), impl.getUsage());

    // a is not the most recent block, so it can't grow in place
    try std.testing.expect(!gpa.resize(a.allocatedSlice(), 6));
    try std.testing.expect(gpa.remap(a.allocatedSlice(), 6) == null);

    // Free in reverse order
    b.deinit(gpa);
    try std.testing.expectEqual(std.mem.alignForward(usize, 20, a16), impl.getUsage());
    for (a.items, 0..) |v, i| try std.testing.expectEqual(@as(u32, @intCast(i)), v);
    a.deinit(gpa);
    try std.testing.expect(impl.isEmpty());

    // Zero sized and out of memory allocations
    const empty = try gpa.alloc(u8, 0);
    gpa.free(empty);
    try std.testing.expectError(error.OutOfMemory, gpa.alloc(u8, 1025));
    if (@sizeOf(usize) > @sizeOf(u32))
        try std.testing.expectError(error.OutOfMemory, gpa.alloc(u8, @as(usize, std.math.maxInt(u32)) + 1)); // Doesn't fit in a uint
    try std.testing.expect(impl.isEmpty());

    // Allocations with the alignment of the temp allocator
    const aligned = try gpa.alignedAlloc(u8, .fromByteUnits(rvector_alignment), 3);
    try std.testing.expect(std.mem.isAligned(@intFromPtr(aligned.ptr), rvector_alignment));
    gpa.free(aligned);
    try std.testing.expect(impl.isEmpty());
}

test "STLTempAllocator on TempAllocatorMalloc" {
    var malloc = TempAllocatorMalloc.init(std.testing.allocator);
    const gpa = STLTempAllocator.init(malloc.tempAllocator()).allocator();

    // Malloc blocks can be grown and freed in any order (std.testing.allocator checks for leaks)
    var a: std.ArrayList(u16) = .empty;
    var b: std.ArrayList(u16) = .empty;
    for (0..100) |i| {
        try a.append(gpa, @intCast(i));
        try b.append(gpa, @intCast(2 * i));
    }
    for (a.items, b.items, 0..) |va, vb, i| {
        try std.testing.expectEqual(@as(u16, @intCast(i)), va);
        try std.testing.expectEqual(@as(u16, @intCast(2 * i)), vb);
    }
    a.deinit(gpa);
    b.deinit(gpa);
}

test "STLTempAllocator on TempAllocatorImplWithMallocFallback" {
    var fallback = try TempAllocatorImplWithMallocFallback.init(std.testing.allocator, @intCast(2 * a16));
    defer fallback.deinit();
    const gpa = fallback.tempAllocator().allocator();

    // The nested TempAllocatorImpl is a different allocator, even if it is at the same address
    const stl = STLTempAllocator.init(fallback.tempAllocator());
    const stl_nested = STLTempAllocator.init(fallback.allocator.tempAllocator());
    try std.testing.expect(stl.eql(STLTempAllocator.init(fallback.tempAllocator())));
    try std.testing.expect(stl_nested.eql(STLTempAllocator.init(fallback.allocator.tempAllocator())));
    try std.testing.expect(!stl.eql(stl_nested));
    try std.testing.expect(!stl_nested.eql(stl));

    const a = try gpa.alloc(u8, 2 * a16); // Fills the fixed block
    try std.testing.expect(fallback.allocator.ownsMemory(a.ptr));
    var b = try gpa.alloc(u8, 1); // From the heap
    try std.testing.expect(!fallback.allocator.ownsMemory(b.ptr));
    b = try gpa.realloc(b, 100);
    gpa.free(b);
    gpa.free(a);
    try std.testing.expect(fallback.allocator.isEmpty());
}
