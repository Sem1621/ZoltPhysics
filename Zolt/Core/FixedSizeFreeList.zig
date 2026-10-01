//! Port of: Jolt/Core/FixedSizeFreeList.h, Jolt/Core/FixedSizeFreeList.inl
//! Status: complete
//!
//! Differences with the C++ version:
//! - `init(allocator, io, max_objects, page_size)` replaces the default constructor + `Init`. The free list allocates
//!   pages lazily inside `constructObject` (which has no allocator parameter in Jolt) and locks its page mutex there,
//!   so it stores the allocator and the io (see "Threading" in the porting guide). `empty` is a default constructed list.
//! - `constructObject(object)` takes the initial value instead of forwarding constructor parameters. It returns
//!   `error.OutOfMemory` when a new page cannot be allocated (Jolt does not check).
//! - Zig has no destructors: when `Object` declares `pub fn deinit(self: *Object) void`, it is called where Jolt
//!   calls `~Object()`, otherwise objects are trivially destructible.
//! - `mNumObjectsAllocated` is a plain uint32 in Jolt that `ConstructObject` reads without holding the page mutex (a
//!   benign race on x86). Zig's memory model makes that undefined behavior, so it is an atomic that is loaded with
//!   acquire and stored with release (plain loads/stores on x86), which also publishes the new page pointer.
//!
//! The page layout, the object index encoding (page = index >> page_shift, slot = index & object_mask), the free list
//! (LIFO, batches are pushed as a whole) and the order in which never used objects are handed out are identical to
//! Jolt, so a single threaded sequence of calls returns the same object indices as the C++ version.

const std = @import("std");
const Core = @import("Core.zig");
const Mutex = @import("Mutex.zig").Mutex;
const math = @import("../Math/Math.zig");

/// Class that allows lock free creation / destruction of objects (unless a new page of objects needs to be allocated)
/// It contains a fixed pool of objects and also allows batching up a lot of objects to be destroyed
/// and doing the actual free in a single atomic operation
///
/// Usage:
/// ```zig
/// var list: FixedSizeFreeList(Job) = try .init(allocator, io, max_jobs, page_size);
/// defer list.deinit();
/// const index = try list.constructObject(.{ ... }); // invalid_object_index when full
/// list.get(index).foo = 1;
/// list.destructObject(index);
/// ```
pub fn FixedSizeFreeList(comptime Object: type) type {
    return struct {
        const Self = @This();

        /// Storage type for an Object
        const ObjectStorage = struct {
            /// The object we're storing
            object: Object,

            /// When the object is freed (or in the process of being freed as a batch) this will contain the next free object
            /// When an object is in use it will contain the object's index in the free list
            next_free_object: std.atomic.Value(u32),
        };

        comptime {
            if (@alignOf(ObjectStorage) != @alignOf(Object))
                @compileError("Object not properly aligned");
        }

        /// Alignment of a page of objects
        const page_alignment: std.mem.Alignment = .fromByteUnits(@max(@alignOf(ObjectStorage), Core.cache_line_size));

        /// A page of page_size objects
        const Page = [*]align(page_alignment.toByteUnits()) ObjectStorage;

        /// Objects that declare `deinit(self: *Object) void` have a destructor
        const has_destructor = switch (@typeInfo(Object)) {
            .@"struct", .@"union", .@"enum", .@"opaque" => @hasDecl(Object, "deinit"),
            else => false,
        };

        /// Invalid index
        pub const invalid_object_index: u32 = 0xffffffff;

        /// Size of an object + bookkeeping for the freelist
        pub const object_storage_size = @sizeOf(ObjectStorage);

        /// A batch of objects that can be destructed
        pub const Batch = struct {
            first_object_index: u32 = invalid_object_index,
            last_object_index: u32 = invalid_object_index,
            num_objects: u32 = 0,
        };

        /// Size (in objects) of a single page
        page_size: u32 = 0,

        /// Number of bits to shift an object index to the right to get the page number
        page_shift: u32 = 0,

        /// Mask to and an object index with to get the page number
        object_mask: u32 = 0,

        /// Total number of pages that are usable
        num_pages: u32 = 0,

        /// Total number of objects that have been allocated (atomic in Zolt, see the top of this file)
        num_objects_allocated: std.atomic.Value(u32) = .init(0),

        /// Array of pages of objects (num_pages entries, the first num_objects_allocated / page_size are allocated)
        pages: ?[*]Page = null,

        /// Allocator for the page table and the pages
        allocator: std.mem.Allocator = undefined,

        /// Io used to lock page_mutex
        io: std.Io = undefined,

        /// Mutex that is used to allocate a new page if the storage runs out
        /// This variable is aligned to the cache line to prevent false sharing with
        /// the constants used to index into the list via `get()`.
        page_mutex: Mutex align(Core.cache_line_size) = .{},

        /// Number of objects that we currently have in the free list / new pages (only when Core.enable_asserts)
        num_free_objects: if (Core.enable_asserts) std.atomic.Value(u32) else void = if (Core.enable_asserts) .init(0) else {},

        /// Simple counter that makes the first free object pointer update with every CAS so that we don't suffer from the ABA problem
        allocation_tag: std.atomic.Value(u32) = .init(0),

        /// Index of first free object, the first 32 bits of an object are used to point to the next free object
        first_free_object_and_tag: std.atomic.Value(u64) = .init(0),

        /// The first free object to use when the free list is empty (may need to allocate a new page)
        first_free_object_in_new_page: std.atomic.Value(u32) = .init(0),

        /// A free list that has not been initialized (default constructor), deinit does nothing
        pub const empty: Self = .{};

        /// Initialize the free list, up to `max_objects` can be allocated (rounded up to a multiple of `page_size`).
        /// `page_size` must be a power of 2. Only the page table is allocated here, pages of `page_size` objects are
        /// allocated by `constructObject` when needed, with `allocator`, while holding a mutex that is locked with `io`.
        pub fn init(allocator: std.mem.Allocator, io: std.Io, max_objects: u32, page_size: u32) error{OutOfMemory}!Self {
            // Check sanity
            std.debug.assert(page_size > 0 and math.isPowerOf2(page_size));

            // Store configuration parameters
            const num_pages = (max_objects +% page_size -% 1) / page_size;

            // Allocate page table
            const pages = try allocator.alloc(Page, num_pages);

            return .{
                .page_size = page_size,
                .page_shift = math.countTrailingZeros(page_size),
                .object_mask = page_size - 1,
                .num_pages = num_pages,
                .num_free_objects = if (Core.enable_asserts) .init(num_pages *% page_size) else {},
                .pages = pages.ptr,
                .allocator = allocator,
                .io = io,

                // We didn't yet use any objects of any page
                .num_objects_allocated = .init(0),
                .first_free_object_in_new_page = .init(0),

                // Start with 1 as the first tag
                .allocation_tag = .init(1),

                // Set first free object (with tag 0)
                .first_free_object_and_tag = .init(invalid_object_index),
            };
        }

        /// Destructor, frees all pages. All objects must have been destructed.
        pub fn deinit(self: *Self) void {
            // Check if we got our Init call
            if (self.pages) |pages| {
                // Ensure everything is freed before the freelist is destructed
                if (Core.enable_asserts) std.debug.assert(self.num_free_objects.load(.monotonic) == self.num_pages *% self.page_size);

                // Free memory for pages
                const num_pages = self.num_objects_allocated.load(.monotonic) / self.page_size;
                for (pages[0..num_pages]) |page|
                    self.allocator.free(page[0..self.page_size]);
                self.allocator.free(pages[0..self.num_pages]);
            }
            self.* = .empty;
        }

        /// Access the object storage given the object index
        fn getStorage(self: *const Self, object_index: u32) *ObjectStorage {
            return &self.pages.?[object_index >> @intCast(self.page_shift)][object_index & self.object_mask];
        }

        /// Call the destructor of an object (~Object())
        fn destroy(object: *Object) void {
            if (has_destructor) object.deinit();
        }

        /// Lockless construct a new object, `object` is its initial value (the C++ version forwards constructor parameters).
        /// Returns the index of the new object, or invalid_object_index when all objects are in use.
        /// Takes the page mutex only when a new page needs to be allocated, fails with error.OutOfMemory if that fails.
        pub fn constructObject(self: *Self, object: Object) error{OutOfMemory}!u32 {
            while (true) {
                // Get first object from the linked list
                const first_free_object_and_tag = self.first_free_object_and_tag.load(.acquire);
                var first_free: u32 = @truncate(first_free_object_and_tag);
                if (first_free == invalid_object_index) {
                    // The free list is empty, we take an object from the page that has never been used before
                    first_free = self.first_free_object_in_new_page.fetchAdd(1, .monotonic);
                    if (first_free >= self.num_objects_allocated.load(.acquire)) {
                        // Allocate new page
                        self.page_mutex.lock(self.io);
                        defer self.page_mutex.unlock(self.io);
                        while (first_free >= self.num_objects_allocated.load(.monotonic)) {
                            const num_objects_allocated = self.num_objects_allocated.load(.monotonic);
                            const next_page = num_objects_allocated / self.page_size;
                            if (next_page == self.num_pages)
                                return invalid_object_index; // Out of space!
                            const page = try self.allocator.alignedAlloc(ObjectStorage, page_alignment, self.page_size);
                            self.pages.?[next_page] = page.ptr;
                            self.num_objects_allocated.store(num_objects_allocated +% self.page_size, .release);
                        }
                    }

                    // Allocation successful
                    if (Core.enable_asserts) _ = self.num_free_objects.fetchSub(1, .monotonic);
                    const storage = self.getStorage(first_free);
                    storage.object = object;
                    storage.next_free_object.store(first_free, .release);
                    return first_free;
                } else {
                    // Load next pointer
                    const new_first_free = self.getStorage(first_free).next_free_object.load(.acquire);

                    // Construct a new first free object tag
                    const new_first_free_object_and_tag = @as(u64, new_first_free) + (@as(u64, self.allocation_tag.fetchAdd(1, .monotonic)) << 32);

                    // Compare and swap
                    if (self.first_free_object_and_tag.cmpxchgWeak(first_free_object_and_tag, new_first_free_object_and_tag, .release, .monotonic) == null) {
                        // Allocation successful
                        if (Core.enable_asserts) _ = self.num_free_objects.fetchSub(1, .monotonic);
                        const storage = self.getStorage(first_free);
                        storage.object = object;
                        storage.next_free_object.store(first_free, .release);
                        return first_free;
                    }
                }
            }
        }

        /// Lockless destruct an object and return it to the free pool
        pub fn destructObject(self: *Self, object_index: u32) void {
            std.debug.assert(object_index != invalid_object_index);

            // Call destructor
            const storage = self.getStorage(object_index);
            destroy(&storage.object);

            // Add to object free list
            while (true) {
                // Get first object from the list
                const first_free_object_and_tag = self.first_free_object_and_tag.load(.acquire);
                const first_free: u32 = @truncate(first_free_object_and_tag);

                // Make it the next pointer of the last object in the batch that is to be freed
                storage.next_free_object.store(first_free, .release);

                // Construct a new first free object tag
                const new_first_free_object_and_tag = @as(u64, object_index) + (@as(u64, self.allocation_tag.fetchAdd(1, .monotonic)) << 32);

                // Compare and swap
                if (self.first_free_object_and_tag.cmpxchgWeak(first_free_object_and_tag, new_first_free_object_and_tag, .release, .monotonic) == null) {
                    // Free successful
                    if (Core.enable_asserts) _ = self.num_free_objects.fetchAdd(1, .monotonic);
                    return;
                }
            }
        }

        /// Lockless destruct an object and return it to the free pool (DestructObject(Object *)).
        /// `object` must be a pointer returned by `get`.
        pub fn destructObjectPtr(self: *Self, object: *Object) void {
            const storage: *ObjectStorage = @alignCast(@fieldParentPtr("object", object));
            const index = storage.next_free_object.load(.monotonic);
            std.debug.assert(index < self.num_objects_allocated.load(.monotonic));
            self.destructObject(index);
        }

        /// Add a object to an existing batch to be destructed.
        /// Adding objects to a batch does not destroy or modify the objects, this will merely link them
        /// so that the entire batch can be returned to the free list in a single atomic operation
        pub fn addObjectToBatch(self: *Self, batch: *Batch, object_index: u32) void {
            std.debug.assert(batch.num_objects != 0xffffffff); // Trying to reuse a batch that has already been freed

            // Reset next index
            const next_free_object = &self.getStorage(object_index).next_free_object;
            if (Core.enable_asserts) std.debug.assert(next_free_object.load(.monotonic) == object_index); // Trying to add a object to the batch that is already in a free list
            next_free_object.store(invalid_object_index, .release);

            // Link object in batch to free
            if (batch.first_object_index == invalid_object_index)
                batch.first_object_index = object_index
            else
                self.getStorage(batch.last_object_index).next_free_object.store(object_index, .release);
            batch.last_object_index = object_index;
            batch.num_objects += 1;
        }

        /// Lockless destruct batch of objects
        pub fn destructObjectBatch(self: *Self, batch: *Batch) void {
            if (batch.first_object_index != invalid_object_index) {
                // Call destructors
                if (has_destructor) {
                    var object_idx = batch.first_object_index;
                    while (true) {
                        const storage = self.getStorage(object_idx);
                        destroy(&storage.object);
                        object_idx = storage.next_free_object.load(.monotonic);
                        if (object_idx == invalid_object_index) break;
                    }
                }

                // Add to objects free list
                const storage = self.getStorage(batch.last_object_index);
                while (true) {
                    // Get first object from the list
                    const first_free_object_and_tag = self.first_free_object_and_tag.load(.acquire);
                    const first_free: u32 = @truncate(first_free_object_and_tag);

                    // Make it the next pointer of the last object in the batch that is to be freed
                    storage.next_free_object.store(first_free, .release);

                    // Construct a new first free object tag
                    const new_first_free_object_and_tag = @as(u64, batch.first_object_index) + (@as(u64, self.allocation_tag.fetchAdd(1, .monotonic)) << 32);

                    // Compare and swap
                    if (self.first_free_object_and_tag.cmpxchgWeak(first_free_object_and_tag, new_first_free_object_and_tag, .release, .monotonic) == null) {
                        // Free successful
                        if (Core.enable_asserts) _ = self.num_free_objects.fetchAdd(batch.num_objects, .monotonic);

                        // Mark the batch as freed
                        if (Core.enable_asserts) batch.num_objects = 0xffffffff;
                        return;
                    }
                }
            }
        }

        /// Access an object by index (both the const and the non const Get of the C++ version).
        pub fn get(self: *const Self, object_index: u32) *Object {
            return &self.getStorage(object_index).object;
        }
    };
}

test "FixedSizeFreeList single threaded" {
    const io = std.testing.io;
    const List = FixedSizeFreeList(u32);
    try std.testing.expectEqual(8, List.object_storage_size);

    var empty: List = .empty;
    empty.deinit();

    // 10 objects with pages of 4 is rounded up to 3 pages
    var list: List = try .init(std.testing.allocator, io, 10, 4);
    defer list.deinit();
    try std.testing.expectEqual(@as(u32, 3), list.num_pages);
    try std.testing.expectEqual(@as(u32, 2), list.page_shift);
    try std.testing.expectEqual(@as(u32, 3), list.object_mask);

    // Objects that were never used are handed out in order, pages are allocated when needed
    for (0..12) |i| {
        const index = try list.constructObject(@intCast(100 + i));
        try std.testing.expectEqual(@as(u32, @intCast(i)), index);
        try std.testing.expectEqual(@as(u32, @intCast(i / 4 + 1)), list.num_objects_allocated.load(.monotonic) / 4);
    }
    try std.testing.expectEqual(List.invalid_object_index, try list.constructObject(0));
    try std.testing.expectEqual(List.invalid_object_index, try list.constructObject(0));
    for (0..12) |i|
        try std.testing.expectEqual(@as(u32, @intCast(100 + i)), list.get(@intCast(i)).*);

    // The free list is LIFO
    list.destructObject(3);
    list.destructObjectPtr(list.get(9));
    try std.testing.expectEqual(@as(u32, 9), try list.constructObject(209));
    try std.testing.expectEqual(@as(u32, 3), try list.constructObject(203));
    try std.testing.expectEqual(List.invalid_object_index, try list.constructObject(0));
    try std.testing.expectEqual(@as(u32, 209), list.get(9).*);
    try std.testing.expectEqual(@as(u32, 203), list.get(3).*);

    // A batch is returned in the order it was built
    var batch: List.Batch = .{};
    list.addObjectToBatch(&batch, 5);
    list.addObjectToBatch(&batch, 7);
    list.addObjectToBatch(&batch, 1);
    try std.testing.expectEqual(@as(u32, 5), batch.first_object_index);
    try std.testing.expectEqual(@as(u32, 1), batch.last_object_index);
    try std.testing.expectEqual(@as(u32, 3), batch.num_objects);
    list.destructObjectBatch(&batch);
    if (Core.enable_asserts) try std.testing.expectEqual(@as(u32, 0xffffffff), batch.num_objects);
    list.destructObject(11);
    try std.testing.expectEqual(@as(u32, 11), try list.constructObject(0));
    try std.testing.expectEqual(@as(u32, 5), try list.constructObject(0));
    try std.testing.expectEqual(@as(u32, 7), try list.constructObject(0));
    try std.testing.expectEqual(@as(u32, 1), try list.constructObject(0));
    try std.testing.expectEqual(List.invalid_object_index, try list.constructObject(0));

    // An empty batch does nothing
    var empty_batch: List.Batch = .{};
    list.destructObjectBatch(&empty_batch);
    try std.testing.expectEqual(List.invalid_object_index, try list.constructObject(0));

    // Free everything in one batch
    var all: List.Batch = .{};
    for (0..12) |i| list.addObjectToBatch(&all, @intCast(i));
    list.destructObjectBatch(&all);
    if (Core.enable_asserts) try std.testing.expectEqual(@as(u32, 12), list.num_free_objects.load(.monotonic));
}

test "FixedSizeFreeList calls deinit as destructor" {
    const io = std.testing.io;

    const Tracked = struct {
        const Self = @This();
        value: u64,
        num_destructed: *u32,

        pub fn deinit(self: *Self) void {
            self.num_destructed.* += 1;
            self.value = 0;
        }
    };

    const List = FixedSizeFreeList(Tracked);
    var num_destructed: u32 = 0;
    var list: List = try .init(std.testing.allocator, io, 64, 16);
    defer list.deinit();

    var indices: [10]u32 = undefined;
    for (&indices, 0..) |*index, i| {
        index.* = try list.constructObject(.{ .value = i, .num_destructed = &num_destructed });
        try std.testing.expectEqual(@as(u64, i), list.get(index.*).value);
    }

    list.destructObject(indices[0]);
    try std.testing.expectEqual(@as(u32, 1), num_destructed);
    list.destructObjectPtr(list.get(indices[1]));
    try std.testing.expectEqual(@as(u32, 2), num_destructed);

    var batch: List.Batch = .{};
    for (indices[2..]) |index| list.addObjectToBatch(&batch, index);
    try std.testing.expectEqual(@as(u32, 2), num_destructed); // Adding to a batch doesn't destruct
    list.destructObjectBatch(&batch);
    try std.testing.expectEqual(@as(u32, 10), num_destructed);
}

test "FixedSizeFreeList large and over aligned objects" {
    const io = std.testing.io;

    // Like QuadTree's Node: 124 bytes, so that an object + bookkeeping is 128 bytes
    const Node = struct {
        bounds: [24]f32,
        children: [4]u32,
        parent: u32,
        is_changed: u32,
        padding: u32 = 0,
    };
    const NodeList = FixedSizeFreeList(Node);
    try std.testing.expectEqual(128, NodeList.object_storage_size);

    var nodes: NodeList = try .init(std.testing.allocator, io, 100, 32);
    defer nodes.deinit();
    for (0..100) |i| {
        const index = try nodes.constructObject(.{ .bounds = @splat(@floatFromInt(i)), .children = @splat(@intCast(i)), .parent = @intCast(i), .is_changed = 1 });
        try std.testing.expectEqual(@as(u32, @intCast(i)), index);
    }
    for (0..100) |i| {
        const node = nodes.get(@intCast(i));
        try std.testing.expectEqual(@as(u32, @intCast(i)), node.parent);
        try std.testing.expectEqual(@as(f32, @floatFromInt(i)), node.bounds[23]);
        nodes.destructObject(@intCast(i));
    }

    // Objects with an alignment of 16 stay aligned
    const Aligned = struct {
        v: @Vector(4, f32),
        id: u32,
    };
    const AlignedList = FixedSizeFreeList(Aligned);
    try std.testing.expectEqual(16, @alignOf(AlignedList.ObjectStorage));
    var aligned: AlignedList = try .init(std.testing.allocator, io, 50, 8);
    defer aligned.deinit();
    for (0..50) |i| {
        const index = try aligned.constructObject(.{ .v = @splat(@floatFromInt(i)), .id = @intCast(i) });
        try std.testing.expect(std.mem.isAligned(@intFromPtr(aligned.get(index)), 16));
        try std.testing.expectEqual(@as(u32, @intCast(i)), aligned.get(index).id);
    }
    for (0..50) |i| aligned.destructObjectPtr(aligned.get(@intCast(i)));
}

/// Stress test: several threads construct and destruct (single and batched) objects at the same time and check that
/// no object is handed out twice and that no object is modified while it is in use.
/// Each thread keeps up to `max_live_per_thread` objects alive. Before the threads start, `num_reserved` objects are
/// taken so that the threads run out of objects when `max_objects - num_reserved < max_live_per_thread`, even when
/// the threads happen to not run at the same time.
fn stressFixedSizeFreeList(max_objects: u32, page_size: u32, max_live_per_thread: u32, num_reserved: u32) !void {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_threads = 4;
    const num_iterations = 200_000;

    const Object = struct {
        value: u64,
        check: u64, // Always ~value
    };
    const List = FixedSizeFreeList(Object);

    const Context = struct {
        list: List,
        in_use: []std.atomic.Value(bool),
        failed: std.atomic.Value(bool) = .init(false),
        num_full: std.atomic.Value(u32) = .init(0),
        num_started: std.atomic.Value(u32) = .init(0),
        max_live: u32,

        fn fail(self: *@This()) void {
            self.failed.store(true, .monotonic);
        }

        /// Check an object that this thread owns and mark it as no longer in use
        fn release(self: *@This(), index: u32, expected_value: u64) void {
            const object = self.list.get(index);
            if (object.value != expected_value or object.check != ~expected_value)
                self.fail();
            if (!self.in_use[index].swap(false, .acq_rel))
                self.fail();
        }

        fn run(self: *@This(), thread_index: u32) void {
            var rng: std.Random.DefaultPrng = .init(thread_index);
            const random = rng.random();
            var live_index: [512]u32 = undefined;
            var live_value: [512]u64 = undefined;
            var num_live: u32 = 0;

            // Wait for all threads to start so that they overlap as much as possible
            _ = self.num_started.fetchAdd(1, .monotonic);
            while (self.num_started.load(.monotonic) < num_threads)
                std.Thread.yield() catch {};

            // Alternate between filling up (mostly constructing) and draining (mostly destructing)
            var filling = true;
            for (0..num_iterations) |iteration| {
                if (filling and num_live >= self.max_live) filling = false;
                if (!filling and num_live <= self.max_live / 4) filling = true;

                const action = random.uintLessThan(u32, 100);
                if (num_live == 0 or (num_live < self.max_live and action < @as(u32, if (filling) 85 else 15))) {
                    // Construct
                    const value = (@as(u64, thread_index) << 32) | iteration;
                    const index = self.list.constructObject(.{ .value = value, .check = ~value }) catch {
                        self.fail();
                        return;
                    };
                    if (index == List.invalid_object_index) {
                        // Out of objects, other threads hold the rest
                        _ = self.num_full.fetchAdd(1, .monotonic);
                        filling = false;
                        continue;
                    }
                    if (index >= self.in_use.len or self.in_use[index].swap(true, .acq_rel)) {
                        self.fail();
                        return;
                    }
                    live_index[num_live] = index;
                    live_value[num_live] = value;
                    num_live += 1;
                } else if (action % 4 == 0 and num_live >= 2) {
                    // Destruct a batch of random objects, small batches while filling
                    var batch: List.Batch = .{};
                    const count = random.intRangeAtMost(u32, 1, if (filling) @min(num_live, 4) else num_live);
                    for (0..count) |_| {
                        const i = random.uintLessThan(u32, num_live);
                        self.release(live_index[i], live_value[i]);
                        self.list.addObjectToBatch(&batch, live_index[i]);
                        num_live -= 1;
                        live_index[i] = live_index[num_live];
                        live_value[i] = live_value[num_live];
                    }
                    if (batch.num_objects != count)
                        self.fail();
                    self.list.destructObjectBatch(&batch);
                } else {
                    // Destruct a single random object, by index or by pointer
                    const i = random.uintLessThan(u32, num_live);
                    const index = live_index[i];
                    self.release(index, live_value[i]);
                    if (action % 2 == 0)
                        self.list.destructObject(index)
                    else
                        self.list.destructObjectPtr(self.list.get(index));
                    num_live -= 1;
                    live_index[i] = live_index[num_live];
                    live_value[i] = live_value[num_live];
                }
            }

            // Free everything that is left
            for (live_index[0..num_live], live_value[0..num_live]) |index, value| {
                self.release(index, value);
                self.list.destructObject(index);
            }
        }
    };

    const allocator = std.testing.allocator;
    var context: Context = .{
        .list = try .init(allocator, io, max_objects, page_size),
        .in_use = try allocator.alloc(std.atomic.Value(bool), max_objects),
        .max_live = max_live_per_thread,
    };
    defer allocator.free(context.in_use);
    defer context.list.deinit();
    @memset(context.in_use, .init(false));

    var reserved: List.Batch = .{};
    for (0..num_reserved) |_| {
        const index = try context.list.constructObject(.{ .value = 0, .check = ~@as(u64, 0) });
        try std.testing.expect(!context.in_use[index].swap(true, .monotonic));
        context.list.addObjectToBatch(&reserved, index);
    }

    var threads: [num_threads]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Context.run, .{ &context, @as(u32, @intCast(i)) });
    for (threads) |t| t.join();

    try std.testing.expect(!context.failed.load(.monotonic));
    if (max_live_per_thread * num_threads + num_reserved <= max_objects)
        try std.testing.expectEqual(@as(u32, 0), context.num_full.load(.monotonic));
    if (max_live_per_thread + num_reserved > max_objects)
        try std.testing.expect(context.num_full.load(.monotonic) > 0);

    // Return the reserved objects
    var reserved_index = reserved.first_object_index;
    while (reserved_index != List.invalid_object_index) : (reserved_index = context.list.getStorage(reserved_index).next_free_object.load(.monotonic))
        try std.testing.expect(context.in_use[reserved_index].swap(false, .monotonic));
    context.list.destructObjectBatch(&reserved);
    for (context.in_use) |*in_use| try std.testing.expect(!in_use.load(.monotonic));
    if (Core.enable_asserts) try std.testing.expectEqual(max_objects, context.list.num_free_objects.load(.monotonic));

    // All objects are free again: max_objects distinct objects can be constructed, then the list is full
    var batch: List.Batch = .{};
    for (0..max_objects) |_| {
        const index = try context.list.constructObject(.{ .value = 0, .check = ~@as(u64, 0) });
        try std.testing.expect(index < max_objects);
        try std.testing.expect(!context.in_use[index].swap(true, .monotonic));
        context.list.addObjectToBatch(&batch, index);
    }
    try std.testing.expectEqual(List.invalid_object_index, try context.list.constructObject(.{ .value = 0, .check = 0 }));
    context.list.destructObjectBatch(&batch);
}

test "FixedSizeFreeList multi threaded" {
    try stressFixedSizeFreeList(1024, 64, 200, 0);
}

test "FixedSizeFreeList multi threaded running out of objects" {
    try stressFixedSizeFreeList(256, 16, 100, 200);
}

test "FixedSizeFreeList multi threaded small pages" {
    // Many pages are allocated while the threads are constructing objects
    try stressFixedSizeFreeList(2048, 4, 400, 0);
}
