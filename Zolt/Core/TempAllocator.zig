//! Port of: Jolt/Core/TempAllocator.h
//! Status: complete
//!
//! - `TempAllocator` is a pure interface (porting guide pattern B): a type erased fat pointer that wraps
//!   any `*T` declaring `allocate(size: u32)` and `free(address, size)`, created with
//!   `TempAllocator.init(&impl)` or `impl.tempAllocator()`. Pass it by value where Jolt passes a
//!   `TempAllocator *` / `TempAllocator &`. The owner of the implementation calls its `deinit` (Jolt
//!   deletes it through the virtual destructor).
//! - Jolt's global `Allocate` / `AlignedAllocate` / `Free` become an explicit `std.mem.Allocator`
//!   (`backing_allocator`). It always takes an explicit alignment, so both `needs_aligned_allocate`
//!   branches of the C++ become the same rvector_alignment aligned allocation.
//! - Out of memory: `TempAllocatorImpl.allocate` traces and calls `std::abort()` in Jolt. Zolt returns
//!   `error.OutOfMemory` at exactly the same point (same size rounding, same capacity accounting), so
//!   callers `try` it. `TempAllocatorMalloc` reports a failing `std.mem.Allocator` the same way.
//! - Freeing in the wrong order is a programming error: Zolt panics where Jolt aborts.
//! - `TempAllocator.allocator()` returns a `std.mem.Allocator` for std containers, the equivalent of
//!   Jolt's `STLTempAllocator` (see STLTempAllocator.zig).
//! - Zolt extension (no C++ counterpart): `resize` changes the size of a block without moving it. It
//!   backs `resize` / `remap` of the `std.mem.Allocator` adapter: `TempAllocatorImpl` can do this for
//!   the most recent block (the top of its stack), `TempAllocatorMalloc` when its `std.mem.Allocator` can.
//!   Ported Jolt code never needs it (Jolt's STL adapter has no reallocate either).

const std = @import("std");
const Core = @import("Core.zig");
const rvector_alignment = @import("../Math/Real.zig").rvector_alignment;
const STLTempAllocator = @import("STLTempAllocator.zig");

/// Alignment of all memory returned by a TempAllocator (JPH_RVECTOR_ALIGNMENT)
const block_alignment: std.mem.Alignment = .fromByteUnits(rvector_alignment);

/// Allocator for temporary allocations.
/// This allocator works as a stack: The blocks must always be freed in the reverse order as they are allocated.
/// Note that allocations and frees can take place from different threads, but the order is guaranteed though
/// job dependencies, so it is not needed to use any form of locking.
pub const TempAllocator = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    /// Errors of `allocate`
    pub const Error = std.mem.Allocator.Error;

    /// Memory returned by `allocate`, rvector_alignment (JPH_RVECTOR_ALIGNMENT) byte aligned
    pub const Block = [*]align(rvector_alignment) u8;

    pub const VTable = struct {
        /// Allocates size bytes of memory, returned memory address must be JPH_RVECTOR_ALIGNMENT byte aligned
        allocate: *const fn (ptr: *anyopaque, size: u32) Error!?Block,

        /// Frees size bytes of memory located at address
        free: *const fn (ptr: *anyopaque, address: ?*anyopaque, size: u32) void,

        /// Zolt extension: resize the block at address from old_size to new_size bytes without moving it,
        /// null when the implementation never can (see `TempAllocator.resize`)
        resize: ?*const fn (ptr: *anyopaque, address: *anyopaque, old_size: u32, new_size: u32) bool = null,

        /// The `std.mem.Allocator` interface of the same implementation (see `allocator()` and STLTempAllocator.zig)
        std_allocator: std.mem.Allocator.VTable,
    };

    /// If this allocator needs to fall back to aligned allocations because JPH_RVECTOR_ALIGNMENT is bigger than the platform default
    pub const needs_aligned_allocate = rvector_alignment > Core.default_allocate_alignment;

    /// Wrap any `*T` that declares `allocate(size: u32) Error!?Block` and
    /// `free(address: ?*anyopaque, size: u32) void`, and optionally
    /// `resize(address: *anyopaque, old_size: u32, new_size: u32) bool`
    pub fn init(impl: anytype) TempAllocator {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing `allocate` of the outer struct
            fn allocateThunk(ptr: *anyopaque, size: u32) Error!?Block {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.allocate(size);
            }
            fn freeThunk(ptr: *anyopaque, address: ?*anyopaque, size: u32) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.free(address, size);
            }
            fn resizeThunk(ptr: *anyopaque, address: *anyopaque, old_size: u32, new_size: u32) bool {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.resize(address, old_size, new_size);
            }
            const vtable: VTable = .{
                .allocate = allocateThunk,
                .free = freeThunk,
                .resize = if (@hasDecl(T, "resize")) resizeThunk else null,
                .std_allocator = STLTempAllocator.stdAllocatorVTable(T),
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Allocates size bytes of memory, returned memory address must be JPH_RVECTOR_ALIGNMENT byte aligned.
    /// Returns null when size is 0.
    pub fn allocate(self: TempAllocator, size: u32) Error!?Block {
        return self.vtable.allocate(self.ptr, size);
    }

    /// Frees size bytes of memory located at address
    pub fn free(self: TempAllocator, address: ?*anyopaque, size: u32) void {
        self.vtable.free(self.ptr, address, size);
    }

    /// Zolt extension (no C++ counterpart): change the size of the block at address from old_size to
    /// new_size bytes without moving it. Returns false (and changes nothing) when that is not possible.
    pub fn resize(self: TempAllocator, address: *anyopaque, old_size: u32, new_size: u32) bool {
        const resize_fn = self.vtable.resize orelse return false;
        return resize_fn(self.ptr, address, old_size, new_size);
    }

    /// A `std.mem.Allocator` that allocates from this temp allocator, for std containers
    /// (`Array<T, STLTempAllocator<T>>` in Jolt). The blocks follow the LIFO rules of the temp allocator,
    /// see STLTempAllocator.zig.
    pub fn allocator(self: TempAllocator) std.mem.Allocator {
        return .{ .ptr = self.ptr, .vtable = &self.vtable.std_allocator };
    }
};

/// AlignUp(inSize, JPH_RVECTOR_ALIGNMENT) with inSize a uint: computed in 64 bits and converted back to
/// uint like the C++ template (sizes above 0xfffffff0 wrap around to 0)
fn alignUpSize(size: u32) u32 {
    return @truncate((@as(u64, size) + rvector_alignment - 1) & ~@as(u64, rvector_alignment - 1));
}

/// Default implementation of the temp allocator that allocates a large block through malloc upfront
pub const TempAllocatorImpl = struct {
    /// Base address of the memory block
    base: TempAllocator.Block,

    /// Size of the memory block
    size: usize,

    /// End of currently allocated area
    top: usize = 0,

    /// Allocator that owns the memory block (Jolt uses its global Allocate / AlignedAllocate)
    backing_allocator: std.mem.Allocator,

    /// Constructs the allocator with a maximum allocatable size of size
    pub fn init(backing_allocator: std.mem.Allocator, size: usize) std.mem.Allocator.Error!TempAllocatorImpl {
        // Jolt: AlignedAllocate(inSize, JPH_RVECTOR_ALIGNMENT) when needs_aligned_allocate, otherwise
        // Allocate(inSize) which is aligned to JPH_DEFAULT_ALLOCATE_ALIGNMENT >= JPH_RVECTOR_ALIGNMENT
        const block = try backing_allocator.alignedAlloc(u8, block_alignment, size);
        return .{ .base = block.ptr, .size = size, .backing_allocator = backing_allocator };
    }

    /// Destructor, frees the block
    pub fn deinit(self: *TempAllocatorImpl) void {
        std.debug.assert(self.top == 0);
        self.backing_allocator.free(self.base[0..self.size]);
        self.* = undefined;
    }

    /// The TempAllocator interface of this allocator. `self` must stay at the same address while it is in use.
    pub fn tempAllocator(self: *TempAllocatorImpl) TempAllocator {
        return TempAllocator.init(self);
    }

    /// See: TempAllocator.
    /// Jolt traces "TempAllocator: Out of memory trying to allocate %u bytes" and aborts when the block is
    /// exhausted, Zolt returns error.OutOfMemory (and leaves the allocator unchanged).
    pub fn allocate(self: *TempAllocatorImpl, size: u32) TempAllocator.Error!?TempAllocator.Block {
        if (size == 0) {
            return null;
        } else {
            const new_top = self.top + alignUpSize(size);
            if (new_top > self.size) {
                return error.OutOfMemory;
            }
            const address: TempAllocator.Block = @alignCast(self.base + self.top);
            self.top = new_top;
            return address;
        }
    }

    /// See: TempAllocator.
    /// Jolt traces "TempAllocator: Freeing in the wrong order" and aborts when address is not the most
    /// recent allocation, Zolt panics.
    pub fn free(self: *TempAllocatorImpl, address: ?*anyopaque, size: u32) void {
        if (address == null) {
            std.debug.assert(size == 0);
        } else {
            self.top -%= alignUpSize(size); // Wraps like the C++ size_t when freeing more than was allocated, caught below
            if (@intFromPtr(self.base) +% self.top != @intFromPtr(address)) {
                @panic("TempAllocator: Freeing in the wrong order");
            }
        }
    }

    /// Zolt extension (see TempAllocator.resize): only the most recent allocation (the top of the stack)
    /// can change size, as long as the new size fits in the block
    pub fn resize(self: *TempAllocatorImpl, address: *anyopaque, old_size: u32, new_size: u32) bool {
        const start = self.top -% alignUpSize(old_size);
        if (@intFromPtr(self.base) +% start != @intFromPtr(address))
            return false;
        const new_top = start + alignUpSize(new_size);
        if (new_top > self.size)
            return false;
        self.top = new_top;
        return true;
    }

    /// Check if no allocations have been made
    pub fn isEmpty(self: *const TempAllocatorImpl) bool {
        return self.top == 0;
    }

    /// Get the total size of the fixed buffer
    pub fn getSize(self: *const TempAllocatorImpl) usize {
        return self.size;
    }

    /// Get current usage in bytes of the buffer
    pub fn getUsage(self: *const TempAllocatorImpl) usize {
        return self.top;
    }

    /// Check if an allocation of size can be made in this fixed buffer allocator
    pub fn canAllocate(self: *const TempAllocatorImpl, size: u32) bool {
        return self.top + alignUpSize(size) <= self.size;
    }

    /// Check if memory block at address is owned by this allocator
    pub fn ownsMemory(self: *const TempAllocatorImpl, address: *const anyopaque) bool {
        const base = @intFromPtr(self.base);
        return @intFromPtr(address) >= base and @intFromPtr(address) < base + self.size;
    }
};

/// Implementation of the TempAllocator that just falls back to malloc/free
/// Note: This can be quite slow when running in the debugger as large memory blocks need to be initialized with 0xcd
pub const TempAllocatorMalloc = struct {
    /// Allocator that provides the memory (Jolt uses its global Allocate / AlignedAllocate)
    backing_allocator: std.mem.Allocator,

    pub fn init(backing_allocator: std.mem.Allocator) TempAllocatorMalloc {
        return .{ .backing_allocator = backing_allocator };
    }

    /// The TempAllocator interface of this allocator. `self` must stay at the same address while it is in use.
    pub fn tempAllocator(self: *TempAllocatorMalloc) TempAllocator {
        return TempAllocator.init(self);
    }

    /// See: TempAllocator
    pub fn allocate(self: *TempAllocatorMalloc, size: u32) TempAllocator.Error!?TempAllocator.Block {
        if (size > 0) {
            // Jolt: AlignedAllocate(inSize, JPH_RVECTOR_ALIGNMENT) when needs_aligned_allocate, otherwise
            // Allocate(inSize). Both are rvector_alignment aligned.
            const block = try self.backing_allocator.alignedAlloc(u8, block_alignment, size);
            return block.ptr;
        } else return null;
    }

    /// See: TempAllocator.
    /// Jolt ignores size here, std.mem.Allocator needs it to free the block.
    pub fn free(self: *TempAllocatorMalloc, address: ?*anyopaque, size: u32) void {
        if (address) |a| {
            const block: TempAllocator.Block = @ptrCast(@alignCast(a));
            self.backing_allocator.free(block[0..size]);
        }
    }

    /// Zolt extension (see TempAllocator.resize): forwards to std.mem.Allocator.resize
    pub fn resize(self: *TempAllocatorMalloc, address: *anyopaque, old_size: u32, new_size: u32) bool {
        const block: TempAllocator.Block = @ptrCast(@alignCast(address));
        return self.backing_allocator.resize(block[0..old_size], new_size);
    }
};

/// Implementation of the TempAllocator that tries to allocate from a large preallocated block, but falls back to malloc when it is exhausted
pub const TempAllocatorImplWithMallocFallback = struct {
    allocator: TempAllocatorImpl,
    fallback_allocator: TempAllocatorMalloc,

    /// Constructs the allocator with an initial fixed block if size
    pub fn init(backing_allocator: std.mem.Allocator, size: u32) std.mem.Allocator.Error!TempAllocatorImplWithMallocFallback {
        return .{
            .allocator = try TempAllocatorImpl.init(backing_allocator, size),
            .fallback_allocator = TempAllocatorMalloc.init(backing_allocator),
        };
    }

    /// Destructor, frees the fixed block
    pub fn deinit(self: *TempAllocatorImplWithMallocFallback) void {
        self.allocator.deinit();
        self.* = undefined;
    }

    /// The TempAllocator interface of this allocator. `self` must stay at the same address while it is in use.
    pub fn tempAllocator(self: *TempAllocatorImplWithMallocFallback) TempAllocator {
        return TempAllocator.init(self);
    }

    /// See: TempAllocator
    pub fn allocate(self: *TempAllocatorImplWithMallocFallback, size: u32) TempAllocator.Error!?TempAllocator.Block {
        if (self.allocator.canAllocate(size))
            return self.allocator.allocate(size)
        else
            return self.fallback_allocator.allocate(size);
    }

    /// See: TempAllocator
    pub fn free(self: *TempAllocatorImplWithMallocFallback, address: ?*anyopaque, size: u32) void {
        if (address) |a| {
            if (self.allocator.ownsMemory(a))
                self.allocator.free(a, size)
            else
                self.fallback_allocator.free(a, size);
        } else {
            std.debug.assert(size == 0);
        }
    }

    /// Zolt extension (see TempAllocator.resize): forwards to the allocator that owns the block
    pub fn resize(self: *TempAllocatorImplWithMallocFallback, address: *anyopaque, old_size: u32, new_size: u32) bool {
        if (self.allocator.ownsMemory(address))
            return self.allocator.resize(address, old_size, new_size)
        else
            return self.fallback_allocator.resize(address, old_size, new_size);
    }
};

// Expected values below follow from the C++: every block is rounded up to JPH_RVECTOR_ALIGNMENT
// (16 bytes, 32 with double precision) and the blocks are stacked from the start of the buffer.
const a16: usize = if (Core.double_precision) 32 else 16;

test "TempAllocatorImpl allocation offsets and usage" {
    try std.testing.expectEqual(a16, rvector_alignment);
    try std.testing.expectEqual(Core.double_precision, TempAllocator.needs_aligned_allocate);

    var impl = try TempAllocatorImpl.init(std.testing.allocator, 8 * a16);
    defer impl.deinit();
    const base = @intFromPtr(impl.base);
    try std.testing.expect(std.mem.isAligned(base, rvector_alignment));
    try std.testing.expectEqual(8 * a16, impl.getSize());
    try std.testing.expect(impl.isEmpty());
    try std.testing.expectEqual(@as(usize, 0), impl.getUsage());

    // Zero sized allocations return null and don't use memory
    try std.testing.expectEqual(@as(?TempAllocator.Block, null), try impl.allocate(0));
    try std.testing.expect(impl.isEmpty());

    // 1 byte uses a full alignment unit
    const p1 = (try impl.allocate(1)).?;
    try std.testing.expectEqual(base, @intFromPtr(p1));
    try std.testing.expectEqual(a16, impl.getUsage());
    try std.testing.expect(!impl.isEmpty());

    // Exactly one unit
    const p2 = (try impl.allocate(@intCast(a16))).?;
    try std.testing.expectEqual(base + a16, @intFromPtr(p2));
    try std.testing.expectEqual(2 * a16, impl.getUsage());

    // One byte more than a unit takes two units
    const p3 = (try impl.allocate(@intCast(a16 + 1))).?;
    try std.testing.expectEqual(base + 2 * a16, @intFromPtr(p3));
    try std.testing.expectEqual(4 * a16, impl.getUsage());

    // The memory is usable
    @memset(p3[0 .. a16 + 1], 0xab);

    // Remaining: 4 units
    try std.testing.expect(impl.canAllocate(@intCast(4 * a16)));
    try std.testing.expect(impl.canAllocate(@intCast(3 * a16 + 1)));
    try std.testing.expect(!impl.canAllocate(@intCast(4 * a16 + 1)));
    try std.testing.expect(impl.canAllocate(0));

    // Ownership: the buffer [base, base + size)
    try std.testing.expect(impl.ownsMemory(p1));
    try std.testing.expect(impl.ownsMemory(@ptrFromInt(base + 8 * a16 - 1)));
    try std.testing.expect(!impl.ownsMemory(@ptrFromInt(base + 8 * a16)));
    var not_owned: u32 = 0;
    try std.testing.expect(!impl.ownsMemory(&not_owned));

    // Free in reverse order
    impl.free(p3, @intCast(a16 + 1));
    try std.testing.expectEqual(2 * a16, impl.getUsage());
    impl.free(p2, @intCast(a16));
    try std.testing.expectEqual(a16, impl.getUsage());
    impl.free(null, 0);
    try std.testing.expectEqual(a16, impl.getUsage());
    impl.free(p1, 1);
    try std.testing.expect(impl.isEmpty());

    // After freeing, the same addresses are handed out again
    const p4 = (try impl.allocate(3)).?;
    try std.testing.expectEqual(base, @intFromPtr(p4));
    impl.free(p4, 3);
    try std.testing.expect(impl.isEmpty());
}

test "TempAllocatorImpl out of memory" {
    var impl = try TempAllocatorImpl.init(std.testing.allocator, 100);
    defer impl.deinit();
    const base = @intFromPtr(impl.base);

    // 100 bytes hold 6 blocks of 16 bytes (single precision) or 3 blocks of 32 bytes (double precision)
    const num_blocks = 100 / a16;
    var blocks: [6]TempAllocator.Block = undefined;
    for (0..num_blocks) |i| {
        try std.testing.expect(impl.canAllocate(@intCast(a16)));
        blocks[i] = (try impl.allocate(@intCast(a16))).?;
        try std.testing.expectEqual(base + i * a16, @intFromPtr(blocks[i]));
    }
    try std.testing.expectEqual(num_blocks * a16, impl.getUsage());

    // The remaining 4 bytes can't hold a block, even a 1 byte allocation is rounded up to a full block
    try std.testing.expect(!impl.canAllocate(1));
    try std.testing.expectError(error.OutOfMemory, impl.allocate(1));
    try std.testing.expectEqual(num_blocks * a16, impl.getUsage()); // A failed allocation doesn't change the usage

    // Zero sized allocations still succeed
    try std.testing.expect(impl.canAllocate(0));
    try std.testing.expectEqual(@as(?TempAllocator.Block, null), try impl.allocate(0));

    // Freeing the last block makes room for exactly one block again
    impl.free(blocks[num_blocks - 1], @intCast(a16));
    try std.testing.expectError(error.OutOfMemory, impl.allocate(@intCast(a16 + 1)));
    blocks[num_blocks - 1] = (try impl.allocate(@intCast(a16 - 1))).?;
    try std.testing.expectEqual(base + (num_blocks - 1) * a16, @intFromPtr(blocks[num_blocks - 1]));

    var i = num_blocks;
    while (i > 0) {
        i -= 1;
        impl.free(blocks[i], @intCast(a16));
    }
    try std.testing.expect(impl.isEmpty());

    // A single allocation of the full size (rounded up) does not fit, of the size rounded down it does
    try std.testing.expectError(error.OutOfMemory, impl.allocate(100));
    const all = (try impl.allocate(@intCast(num_blocks * a16))).?;
    try std.testing.expectEqual(num_blocks * a16, impl.getUsage());
    impl.free(all, @intCast(num_blocks * a16));

    // The size is rounded up as a uint like Jolt's AlignUp<uint>: a size that rounds past 2^32 wraps to 0
    try std.testing.expectEqual(@as(u32, 0), alignUpSize(0xffff_ffff));
    try std.testing.expectEqual(@as(u32, @intCast(0x1_0000_0000 - a16)), alignUpSize(@intCast(0x1_0000_0000 - a16)));
    try std.testing.expect(!impl.canAllocate(@intCast(0x1_0000_0000 - a16)));
    try std.testing.expectError(error.OutOfMemory, impl.allocate(@intCast(0x1_0000_0000 - a16)));
}

test "TempAllocatorImpl out of memory for the backing allocator" {
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 0 });
    try std.testing.expectError(error.OutOfMemory, TempAllocatorImpl.init(failing.allocator(), 1024));
    try std.testing.expectError(error.OutOfMemory, TempAllocatorImplWithMallocFallback.init(failing.allocator(), 1024));
}

test "TempAllocatorImpl resize (Zolt extension)" {
    var impl = try TempAllocatorImpl.init(std.testing.allocator, 4 * a16);
    defer impl.deinit();

    const p1 = (try impl.allocate(1)).?;
    const p2 = (try impl.allocate(1)).?;

    // Only the top block can be resized
    try std.testing.expect(!impl.resize(p1, 1, @intCast(2 * a16)));
    try std.testing.expect(impl.resize(p2, 1, @intCast(2 * a16)));
    try std.testing.expectEqual(3 * a16, impl.getUsage());
    try std.testing.expect(impl.resize(p2, @intCast(2 * a16), @intCast(3 * a16)));
    try std.testing.expectEqual(4 * a16, impl.getUsage());

    // It must fit
    try std.testing.expect(!impl.resize(p2, @intCast(3 * a16), @intCast(3 * a16 + 1)));
    try std.testing.expectEqual(4 * a16, impl.getUsage());

    // Shrink
    try std.testing.expect(impl.resize(p2, @intCast(3 * a16), 5));
    try std.testing.expectEqual(2 * a16, impl.getUsage());

    impl.free(p2, 5);
    impl.free(p1, 1);
    try std.testing.expect(impl.isEmpty());
}

test "TempAllocatorMalloc" {
    var malloc = TempAllocatorMalloc.init(std.testing.allocator);
    try std.testing.expectEqual(@as(?TempAllocator.Block, null), try malloc.allocate(0));
    malloc.free(null, 0);

    const p1 = (try malloc.allocate(3)).?;
    const p2 = (try malloc.allocate(1000)).?;
    try std.testing.expect(std.mem.isAligned(@intFromPtr(p1), rvector_alignment));
    try std.testing.expect(std.mem.isAligned(@intFromPtr(p2), rvector_alignment));
    @memset(p2[0..1000], 1);

    // Blocks can be freed in any order, std.testing.allocator checks the sizes and leaks
    malloc.free(p1, 3);
    malloc.free(p2, 1000);

    // Out of memory of the backing allocator is reported
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 0 });
    var failing_malloc = TempAllocatorMalloc.init(failing.allocator());
    try std.testing.expectError(error.OutOfMemory, failing_malloc.allocate(16));
}

test "TempAllocatorImplWithMallocFallback" {
    var allocator = try TempAllocatorImplWithMallocFallback.init(std.testing.allocator, @intCast(4 * a16));
    defer allocator.deinit();
    const impl = &allocator.allocator;

    // Fits in the fixed block
    const p1 = (try allocator.allocate(@intCast(2 * a16))).?;
    try std.testing.expect(impl.ownsMemory(p1));
    try std.testing.expectEqual(2 * a16, impl.getUsage());

    // Doesn't fit, goes to the heap
    const p2 = (try allocator.allocate(@intCast(2 * a16 + 1))).?;
    try std.testing.expect(!impl.ownsMemory(p2));
    try std.testing.expectEqual(2 * a16, impl.getUsage());

    // Fits again in the fixed block
    const p3 = (try allocator.allocate(@intCast(2 * a16))).?;
    try std.testing.expect(impl.ownsMemory(p3));
    try std.testing.expectEqual(4 * a16, impl.getUsage());

    // Zero sized
    try std.testing.expectEqual(@as(?TempAllocator.Block, null), try allocator.allocate(0));
    allocator.free(null, 0);

    // The fixed block is full, so everything goes to the heap now
    const p4 = (try allocator.allocate(1)).?;
    try std.testing.expect(!impl.ownsMemory(p4));

    allocator.free(p4, 1);
    allocator.free(p3, @intCast(2 * a16));
    try std.testing.expectEqual(2 * a16, impl.getUsage());
    allocator.free(p2, @intCast(2 * a16 + 1));
    allocator.free(p1, @intCast(2 * a16));
    try std.testing.expect(impl.isEmpty());
}

test "TempAllocator interface" {
    var impl = try TempAllocatorImpl.init(std.testing.allocator, 4 * a16);
    defer impl.deinit();
    var malloc = TempAllocatorMalloc.init(std.testing.allocator);
    var fallback = try TempAllocatorImplWithMallocFallback.init(std.testing.allocator, @intCast(a16));
    defer fallback.deinit();

    const allocators = [_]TempAllocator{ impl.tempAllocator(), malloc.tempAllocator(), fallback.tempAllocator() };
    for (allocators) |temp_allocator| {
        try std.testing.expectEqual(@as(?TempAllocator.Block, null), try temp_allocator.allocate(0));
        const p1 = (try temp_allocator.allocate(5)).?;
        const p2 = (try temp_allocator.allocate(@intCast(2 * a16))).?;
        try std.testing.expect(std.mem.isAligned(@intFromPtr(p1), rvector_alignment));
        try std.testing.expect(std.mem.isAligned(@intFromPtr(p2), rvector_alignment));
        temp_allocator.free(p2, @intCast(2 * a16));
        temp_allocator.free(p1, 5);
        temp_allocator.free(null, 0);
    }
    try std.testing.expect(impl.isEmpty());
    try std.testing.expect(fallback.allocator.isEmpty());

    // The interface reaches the implementation
    const p = (try allocators[0].allocate(1)).?;
    try std.testing.expectEqual(@intFromPtr(impl.base), @intFromPtr(p));
    try std.testing.expectEqual(a16, impl.getUsage());
    try std.testing.expect(allocators[0].resize(p, 1, @intCast(2 * a16)));
    try std.testing.expectEqual(2 * a16, impl.getUsage());
    allocators[0].free(p, @intCast(2 * a16));
    try std.testing.expect(impl.isEmpty());

    // A user implementation without resize
    const Counting = struct {
        malloc: TempAllocatorMalloc,
        num_allocations: u32 = 0,

        pub fn allocate(self: *@This(), size: u32) TempAllocator.Error!?TempAllocator.Block {
            self.num_allocations += 1;
            return self.malloc.allocate(size);
        }
        pub fn free(self: *@This(), address: ?*anyopaque, size: u32) void {
            self.num_allocations -= 1;
            self.malloc.free(address, size);
        }
    };
    var counting: Counting = .{ .malloc = .init(std.testing.allocator) };
    const temp_allocator = TempAllocator.init(&counting);
    const q = (try temp_allocator.allocate(10)).?;
    try std.testing.expectEqual(@as(u32, 1), counting.num_allocations);
    try std.testing.expect(!temp_allocator.resize(q, 10, 11));
    temp_allocator.free(q, 10);
    try std.testing.expectEqual(@as(u32, 0), counting.num_allocations);

    // Its std.mem.Allocator can't resize in place, so realloc allocates, copies and frees
    const gpa = temp_allocator.allocator();
    var bytes = try gpa.alloc(u8, 4);
    @memcpy(bytes, "abcd");
    try std.testing.expect(!gpa.resize(bytes, 5));
    try std.testing.expect(gpa.remap(bytes, 5) == null);
    bytes = try gpa.realloc(bytes, 8);
    try std.testing.expectEqualSlices(u8, "abcd", bytes[0..4]);
    try std.testing.expectEqual(@as(u32, 1), counting.num_allocations);
    gpa.free(bytes);
    try std.testing.expectEqual(@as(u32, 0), counting.num_allocations);
}
