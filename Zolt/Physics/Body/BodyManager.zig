//! Port of: Jolt/Physics/Body/BodyManager.h, Jolt/Physics/Body/BodyManager.cpp
//! Status: complete
//! Not ported: ReportSimulationStats (JPH_TRACK_SIMULATION_STATS + JPH_PROFILE_ENABLED, dropped with the profiler)
//!
//! RemainingFeaturesArchitecture.md R16: `allocateSoftBody` is one allocation, like Jolt's `new
//! SoftBodyWithMotionPropertiesAndShape`: the body, its SoftBodyMotionProperties and its embedded SoftBodyShape. It
//! returns `CreateBodyError` (out of memory: the half built soft body is deleted again). `deleteBody` deletes a soft
//! body like Jolt's destructor chain: it releases the body's shape reference, deinits the motion properties (Jolt's
//! `~SoftBodyMotionProperties`: the arrays, the shared settings, the leaf shapes), checks that the embedded shape has no
//! reference left and releases the collision group.
//!
//! SimulationArchitecture D2 / D3:
//! - Initialized in place (`var bm: BodyManager = .empty; try bm.init(allocator, io, ...)`), because the lock
//!   interfaces point into it; it must not move afterwards. `io` is stored at init, every lock user reads
//!   `body_manager.io`.
//! - `mBodies` is `bodies: []BodySlot`, allocated once with `max_bodies` entries (`bodies.len` is GetMaxBodies()), and
//!   `bodies_size` is Jolt's `mBodies.size()`: stored with `.release` under the bodies mutex after the slot is written,
//!   loaded with `.acquire` by the lock-free `tryGetBody` / `getBodies`. A slot is a body pointer or a freed index with
//!   Jolt's encoding (`BodySlot`). ID allocation is literal: a LIFO free list in the freed slots, `+%=` on the `u8`
//!   sequence numbers, `addBodyWithCustomID`'s O(N) unlink.
//! - The C++ `mutable` mutexes live in the heap block `mutable_fields` (Rule M), so `getNumBodies`, `lockAllBodies`,
//!   `getActiveBodies`, `saveState`, ... keep `*const BodyManager` receivers.
//! - `num_active_bodies` uses Jolt's orders (`fetchAdd(1, .release)` after the ID is written, `.acquire` in
//!   getNumActiveBodies); `MotionProperties.index_in_active_bodies` is a `.monotonic` atomic.
//! - The lock order checks (PhysicsLock), `mActiveBodiesLocked` and the per thread GrantActiveBodiesAccess overrides are
//!   debug state typed `void` without asserts.
//! - Errors: `allocateBody` returns `CreateBodyError` (no shape: Jolt crashes; out of memory) and frees the half built
//!   body; `addBody` keeps Jolt's `bool` for "out of IDs". Functions that fill a caller's array take its `allocator`
//!   (`getActiveBodies`, `getBodyIDs`); `saveState` / `restoreState` build temporary arrays with the manager's
//!   allocator. All of them return `Allocator.Error`. `restoreState` reserves its arrays before it restores a body, so
//!   out of memory on a stream written by `saveState` leaves the bodies unchanged (a malformed stream that names a body
//!   more than once can still grow them mid-restore, see `restoreState`). `invalidateContactCacheForBody` returns
//!   `Allocator.Error` (the list of bodies to validate grows) and then clears the flag again, so the flag and the list
//!   stay consistent.
//! - Zolt (D5): an update that runs out of memory inside a job may already have reset the InvalidateContactCache flags
//!   (`validateContactCacheForAllBodies` runs in every ContactRemovedCallbacks job), and the flags are not part of the
//!   saved state. `saveContactCacheInvalidation` copies the list at the start of the update into
//!   `bodies_cache_invalid_at_update_start` (its room is reserved in `invalidateContactCacheForBody`, the update does not
//!   allocate) and `restoreContactCacheInvalidation` puts the flags and the list back when the update failed, so that
//!   the update after restoring a saved state treats the contact cache like the update that failed.
//! - Overloads: the const / non-const `GetBody`, `TryGetBody`, `GetBodies` are `getBody` / `getBodyMut`, `tryGetBody` /
//!   `tryGetBodyMut`, `getBodies` / `getBodiesMut`. Pointer + count parameters are slices.
//! - JPH_DEBUG_RENDERER (`Draw`, `DrawSettings`, `EShapeColor`, `ESoftBodyConstraintColor`): gated declarations
//!   (RemainingFeaturesArchitecture.md R2), `void` without `-Ddebug_renderer`. Jolt's default argument `inBodyFilter` is
//!   `DrawOptions.body_filter`. Draw takes the body mutexes like Jolt.
//!
//! Differences with the C++ version (Roadmap Phase 13, class A: memory placement only, no result changes):
//! - The rigid bodies with motion properties (BodyWithMotionProperties: dynamic and kinematic bodies, and static ones
//!   created with `allow_dynamic_or_kinematic`) live in 64 KB pages (`BodyPool`) instead of one `new
//!   BodyWithMotionProperties` each. A page is allocated, aligned to 64 KB, from the allocator of the BodyManager that
//!   creates the body, when its bodies need it. Its first 64 bytes are a header (a magic number, a spin lock, the
//!   number of bodies in the page, the first slot that was never used, a free list, whether its BodyManager still
//!   exists, the allocator of the page), then come (64 KB - 64) / @sizeOf(BodyWithMotionProperties) slots: on 64-bit
//!   targets 204 of 320 bytes, 170 of 384 bytes in double precision, each one starting a cache line. The pages make the bodies' memory
//!   independent of the user's allocator: Zig's SmpAllocator puts every body in a 512 byte slot, so a scene's bodies
//!   take 1.6 times their size in cache lines and pages (5000 bodies: 2.56 MB against a 2 MB L2), and every 8th body
//!   has the 4 KB offset of another one. (Phase 13's prototype X1 measured the pool together with the update arrays that
//!   QuadTree.zig keeps: HighSpeed and ConvexVsMesh 2-8% more steps per second; the two parts were not measured apart.)
//!   Freed slots are reused: create takes a slot of the page that handed out the last slot (within a page the last one
//!   freed first), then of the next page that has a free slot. Static bodies (a Body without motion properties) and soft bodies
//!   are allocated as before. Only where the bodies live changes: no result depends on a body's address (Jolt orders
//!   and hashes bodies by BodyID; the one use of a body address as data, BodySlot's freed bit, needs only an even
//!   address).
//! - Ownership is Jolt's: its `sDeleteBody` is static and frees every body with the global allocator, so any
//!   BodyManager can destroy any body, and a body that UnassignBodyID(s) hands out ("caller assumes ownership") can be
//!   assigned to another PhysicsSystem or destroyed through it, also after its own PhysicsSystem is gone. Here a page
//!   describes itself: the page of a body is the body's address with the low 16 bits cleared, so the destroy needs no
//!   pool (`BodyPool.destroy`), whatever BodyManager runs it.
//! - Memory behavior that differs from Jolt: a page is freed when its BodyManager is gone and its last body is
//!   destroyed (by the BodyManager's deinit, or by the destroy of the page's last body when a body outlives its
//!   BodyManager), with the page's allocator, which must stay valid until then. A page stays allocated while its
//!   BodyManager exists, also when it holds no body, and while it holds a body that is never destroyed (Jolt leaks that
//!   body). The allocation failures of body creation happen once per page (204 or 170 bodies) instead of once per body,
//!   and when the page table grows: init reserves it for the pages of max_bodies (8 bytes per page), it grows only for
//!   bodies beyond (CreateBodyWithoutID can create more than max_bodies). allocateBody then returns error.OutOfMemory
//!   and the pool is unchanged.
//! - A static body or a soft body that is moved to another PhysicsSystem is freed with that system's allocator (Zolt's
//!   explicit allocators, also before Phase 13): the two systems must use the same allocator for such a move.
//! - `getBodyPoolStats` and `isBodyInPool` (Zolt only) tell the pool's state, for tests and diagnostics.

const std = @import("std");
const builtin = @import("builtin");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const MutexFile = @import("../../Core/Mutex.zig");
const Mutex = MutexFile.Mutex;
const SharedMutex = MutexFile.SharedMutex;
const MutexArray = @import("../../Core/MutexArray.zig").MutexArray;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Color = @import("../../Core/Color.zig").Color;
const StringTools = @import("../../Core/StringTools.zig");
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const ShapeFunctions = @import("../Collision/Shape/Shape.zig").ShapeFunctions;
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;
const MassProperties = @import("MassProperties.zig").MassProperties;
const BodyDrawFilter = @import("BodyFilter.zig").BodyDrawFilter;
const BroadPhaseLayerInterface = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerInterface;
const BodyID = @import("BodyID.zig").BodyID;
const Body = @import("Body.zig").Body;
const BodyTypeFile = @import("BodyType.zig");
const BodyType = BodyTypeFile.BodyType;
const body_type_count = BodyTypeFile.body_type_count;
const MotionQuality = @import("MotionQuality.zig").MotionQuality;
const MotionPropertiesFile = @import("MotionProperties.zig");
const MotionProperties = MotionPropertiesFile.MotionProperties;
const track_simulation_stats = MotionPropertiesFile.track_simulation_stats;
const BodyCreationSettingsFile = @import("BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const BodyActivationListener = @import("BodyActivationListener.zig").BodyActivationListener;
const SoftBodyMotionProperties = @import("../SoftBody/SoftBodyMotionProperties.zig").SoftBodyMotionProperties;
const SoftBodyShape = @import("../SoftBody/SoftBodyShape.zig").SoftBodyShape;
const SoftBodyCreationSettings = @import("../SoftBody/SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
const SoftBodySharedSettings = @import("../SoftBody/SoftBodySharedSettings.zig").SoftBodySharedSettings;
const StateRecorderFile = @import("../StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;
const PhysicsLockFile = @import("../PhysicsLock.zig");
const PhysicsLock = PhysicsLockFile.PhysicsLock;
const UniqueLock = PhysicsLockFile.UniqueLock;

/// Defines how to color soft body constraints (JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)
pub const SoftBodyConstraintColor = if (Core.debug_renderer) enum {
    /// Draw different types of constraints in different colors
    constraint_type,
    /// Draw constraints in the same group in the same color, non-parallel group will be red
    constraint_group,
    /// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
    constraint_order,
} else void;

/// Array of bodies (BodyVector = Array<Body *>): the slots of BodyManager.bodies, a body pointer or a freed index
pub const BodyVector = []BodyManager.BodySlot;

/// Array of body ID's
pub const BodyIDVector = std.ArrayList(BodyID);

/// Helper class that combines a body and its motion properties
pub const BodyWithMotionProperties = struct {
    body: Body = .{},
    motion_properties: MotionProperties = .{},

    // Zolt: the motion properties follow the body like in Jolt (offset 128 in single precision). In double precision
    // Jolt's compiler puts them in the tail padding of Body (offset 144: Body's data ends at 138), which Zig does not
    // reuse (offset 160).
    comptime {
        if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
            std.debug.assert(@offsetOf(BodyWithMotionProperties, "motion_properties") == if (Core.double_precision) 160 else 128);
    }
};

/// Helper class that combines a soft body its motion properties and shape
pub const SoftBodyWithMotionPropertiesAndShape = struct {
    body: Body = .{},
    motion_properties: SoftBodyMotionProperties,
    shape: SoftBodyShape,

    /// Constructor: the motion properties allocate with `allocator` (the body manager's). The shape is marked embedded
    /// by BodyManager.allocateSoftBody, once it points to the motion properties.
    pub fn init(allocator: Allocator) SoftBodyWithMotionPropertiesAndShape {
        return .{ .motion_properties = .{ .allocator = allocator }, .shape = .init(allocator) };
    }
};

/// Class that contains all bodies
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
/// Its functionality is exposed through PhysicsSystem, BodyInterface, BodyLockRead and BodyLockWrite.
pub const BodyManager = struct {
    /// Bodies are protected using an array of mutexes (so a fixed number, not 1 per body). Each bit in this mask indicates a locked mutex.
    pub const MutexMask = u64;

    /// An array of mutexes protecting the bodies in the mBodies array
    const BodyMutexes = MutexArray(SharedMutex);

    /// Bit that indicates a pointer in mBodies is actually the index of the next freed body. We use the lowest bit because we know that Bodies need to be 16 byte aligned so addresses can never end in a 1 bit.
    const is_freed_body: usize = 1;

    /// Amount of bits to shift to get an index to the next freed body
    const freed_body_index_shift = 1;

    /// An entry of the body array (BodyVector = Array<Body *>): Jolt stores a `Body *` that is either a body or a freed
    /// slot `(next_free << cFreedBodyIndexShift) | cIsFreedBody`. Zig pointers can't hold odd values, so the entry is an
    /// integer with typed accessors.
    pub const BodySlot = enum(usize) {
        /// Indicates that there are no more freed body IDs (cBodyIDFreeListEnd)
        free_list_end = ~@as(usize, 0),
        _,

        /// The slot of a body
        pub fn fromBody(body: *Body) BodySlot {
            return @enumFromInt(@intFromPtr(body));
        }

        /// Test if a body pointer is valid (sIsValidBodyPointer)
        pub fn isValidBodyPointer(self: BodySlot) bool {
            return (@intFromEnum(self) & is_freed_body) == 0;
        }

        /// The body (the slot must be valid)
        pub fn getBody(self: BodySlot) *Body {
            std.debug.assert(self.isValidBodyPointer());
            return @ptrFromInt(@intFromEnum(self));
        }

        /// A freed slot that links to the freed slot `index`: `(index << cFreedBodyIndexShift) | cIsFreedBody`
        fn fromFreedIndex(index: usize) BodySlot {
            return @enumFromInt((index << freed_body_index_shift) | is_freed_body);
        }

        /// The index that a freed slot links to (`uintptr_t(slot) >> cFreedBodyIndexShift`)
        fn freedIndex(self: BodySlot) usize {
            return @intFromEnum(self) >> freed_body_index_shift;
        }
    };

    /// Helper struct that counts the number of bodies of each type
    pub const BodyStats = struct {
        /// Total number of bodies in the body manager
        num_bodies: u32 = 0,
        /// Max allowed number of bodies in the body manager (as configured in Init(...))
        max_bodies: u32 = 0,

        /// Number of static bodies
        num_bodies_static: u32 = 0,

        /// Number of dynamic bodies
        num_bodies_dynamic: u32 = 0,
        /// Number of dynamic bodies that are currently active
        num_active_bodies_dynamic: u32 = 0,

        /// Number of kinematic bodies
        num_bodies_kinematic: u32 = 0,
        /// Number of kinematic bodies that are currently active
        num_active_bodies_kinematic: u32 = 0,

        /// Number of soft bodies
        num_soft_bodies: u32 = 0,
        /// Number of soft bodies that are currently active
        num_active_soft_bodies: u32 = 0,
    };

    /// The C++ `mutable` members that const functions lock (Rule M: behind a pointer, never through @constCast)
    const MutableFields = struct {
        /// Protects mBodies array (but not the bodies it points to), mNumBodies and mBodyIDFreeListStart
        bodies_mutex: Mutex = .{},
        /// Mutex that protects the mActiveBodies array
        active_bodies_mutex: Mutex = .{},
        /// Mutex that protects the mBodiesCacheInvalid array
        bodies_cache_invalid_mutex: Mutex = .{},
        /// Zolt (Phase 13): the pages of the rigid bodies with motion properties (allocateBody has a `*const` receiver
        /// like Jolt's)
        body_pool: BodyPool = .{},
    };

    /// Zolt (Phase 13, class A: memory placement only): the memory of the rigid bodies with motion properties, in pages
    /// that describe themselves. Jolt allocates every BodyWithMotionProperties with `new` (glibc packs them 336 bytes
    /// apart). Zig's SmpAllocator rounds the 320 bytes up to its 512 byte size class: a scene's bodies then take 1.6 times
    /// the memory and the pages, and every 8th body has the 4 KB offset and the L1 sets of another one. The pool hands out
    /// slots of exactly @sizeOf(BodyWithMotionProperties), 64 byte aligned (5 cache lines per body, 6 in double
    /// precision), from pages of 64 KB that it allocates, aligned to 64 KB, from the BodyManager's allocator when the
    /// bodies need them.
    ///
    /// A page describes itself: its first 64 bytes are a `Page` header, and the page of a body is the body's address with
    /// the low 16 bits cleared. So any BodyManager can destroy any body, also after the BodyManager that created it is
    /// gone, as in Jolt, whose static sDeleteBody frees every body with the global allocator: `destroy` needs no pool, it
    /// puts the slot on the free list of its page. The header also holds the page's allocator and whether the BodyManager
    /// of the page still exists (`owner_alive`). A page is freed once its BodyManager is gone and it holds no body: by the
    /// pool's deinit, or by the destroy of its last body.
    ///
    /// `create` takes a slot of the current page (the page that handed out the last slot): a freed slot first, the last
    /// one freed first, then a slot that was never used. When the current page is full it takes the next page that has a
    /// free slot (it reads one counter per page), and allocates a new page when all pages are full. The page table is
    /// reserved at init for the pages of max_bodies and grows only beyond (CreateBodyWithoutID can create more bodies).
    ///
    /// Threads: bodies are created (allocateBody, no lock in Jolt) and destroyed (destroyBodies under the bodies mutex,
    /// freeBody without it, also through another BodyManager) from several threads. The pool mutex protects the page
    /// table and the choice of a page for create. The contents of a page (`live`, `next_unused`, `free_list`,
    /// `owner_alive`) change only under the page lock, a spin lock that is held for a few instructions. Lock order: the
    /// pool mutex, then one page lock. Nothing is locked inside a page lock, and the allocator is not called inside one (a
    /// page is freed after its lock was released). Why there is no race:
    /// - create against a destroy on the same page (through any BodyManager): both change the free list and `live` only
    ///   under the page lock. Only the creates of the page's own pool make `live` larger, and they run one at a time
    ///   under the pool mutex. So the `live` that create reads without the page lock is never smaller than the count
    ///   when it then locks the page: a page that looked like it had a free slot has one. While create runs, its
    ///   BodyManager exists (its deinit must not run at the same time as its other functions, as in Jolt), so
    ///   `owner_alive` is true and no destroy frees the page.
    /// - deinit against a destroy on the same page: deinit sets `owner_alive` to false under the page lock and frees the
    ///   page when `live` is 0. destroy decrements `live` under the page lock and frees the page when it reaches 0 while
    ///   `owner_alive` is false. `owner_alive` turns false once, and after that `live` only decreases (only the owner
    ///   creates bodies), so the condition (no owner and no body) becomes true in exactly one critical section and stays
    ///   true. Each party checks it right after its own change in that section, so exactly one party frees the page:
    ///   deinit when the page held no body any more (then no destroy of it can follow), otherwise the destroy that takes
    ///   `live` to 0 (deinit saw a body, and every destroy before it saw a body left or the owner alive).
    /// - two destroys on the same page: the page lock orders them, and only the one that takes `live` to 0 can free the
    ///   page; no body of the page is left to destroy after it.
    /// - the party that frees the page uses it last: every other use happened in an earlier critical section of the
    ///   page lock, which the lock's release and acquire order before the free, and nothing can reach the page any more
    ///   (it holds no body, and no pool has it). A thread that waits for a page lock is the owner (create, deinit, the
    ///   stats) or the destroy of a body that is still counted in `live`, so the page is not freed while it waits.
    const BodyPool = struct {
        /// Size and alignment of a page
        const page_size = 64 * 1024;
        /// The page header takes the first cache line of a page
        const header_size = Core.cache_line_size;
        /// Every slot starts a cache line
        const slot_alignment = Core.cache_line_size;

        const Slot = BodyWithMotionProperties;

        /// Number of slots of a page: 204 of 320 bytes, 170 of 384 bytes in double precision (64-bit targets)
        const slots_per_page: u32 = (page_size - header_size) / @sizeOf(Slot);

        /// The first 8 bytes of a page that a pool allocated ("ZoltBody"; destroy asserts them)
        const page_magic: u64 = 0x5a6f_6c74_426f_6479;

        /// A free slot holds the link of its page's free list
        const FreeSlot = extern struct { next: ?*FreeSlot };

        /// The header at the start of a page
        const Page = extern struct {
            /// `page_magic` while the page is allocated
            magic: u64,
            /// The page lock (a spin lock)
            lock: std.atomic.Value(u32),
            /// Number of slots that hold a body. Changed under the page lock, create reads it without the lock.
            live: std.atomic.Value(u32),
            /// First slot that was never handed out
            next_unused: u32,
            /// False once the BodyManager that allocated the page was deinitialized
            owner_alive: bool,
            /// Freed slots, the last one freed first
            free_list: ?*FreeSlot,
            /// The allocator of the page (the one of its BodyManager), it frees the page
            allocator_ptr: *anyopaque,
            allocator_vtable: *const Allocator.VTable,

            /// The page that holds a slot (of any pool)
            fn ofSlot(slot: *Slot) *Page {
                const page: *Page = @ptrFromInt(@intFromPtr(slot) & ~@as(usize, page_size - 1));
                std.debug.assert(page.magic == page_magic); // The body is a slot of a page
                std.debug.assert((@intFromPtr(slot) - @intFromPtr(page.slots())) % @sizeOf(Slot) == 0);
                return page;
            }

            /// The slots after the header
            fn slots(page: *Page) *align(slot_alignment) [slots_per_page]Slot {
                return @ptrFromInt(@intFromPtr(page) + header_size);
            }

            fn acquire(page: *Page) void {
                while (page.lock.cmpxchgWeak(0, 1, .acquire, .monotonic) != null) {
                    while (page.lock.load(.monotonic) != 0)
                        std.atomic.spinLoopHint();
                }
            }

            fn release(page: *Page) void {
                page.lock.store(0, .release);
            }

            /// A slot for a body (not initialized): a freed slot, else one that was never used. The page must have a free
            /// slot.
            fn take(page: *Page) *Slot {
                page.acquire();
                defer page.release();

                const live = page.live.load(.monotonic);
                std.debug.assert(live < slots_per_page and page.owner_alive);
                page.live.store(live + 1, .monotonic);
                if (page.free_list) |freed| {
                    page.free_list = freed.next;
                    return @ptrCast(@alignCast(freed));
                }
                const slot = &page.slots()[page.next_unused];
                page.next_unused += 1;
                return slot;
            }

            /// Gives the page back to its allocator (no pool has it and it holds no body)
            fn free(page: *Page) void {
                const allocator: Allocator = .{ .ptr = page.allocator_ptr, .vtable = page.allocator_vtable };
                page.magic = 0;
                const memory: []align(page_size) u8 = @as([*]align(page_size) u8, @ptrCast(@alignCast(page)))[0..page_size];
                allocator.free(memory);
            }
        };

        comptime {
            // On 64-bit targets the slots have no padding: a body takes exactly the bytes of BodyWithMotionProperties
            // and starts a cache line. 320 bytes in single precision, 384 in double precision; asserts (MotionProperties'
            // cached types) and the debug renderer do not change the size. (Like Body's size assert, this one holds for
            // 64-bit targets only; elsewhere the slots are @sizeOf(BodyWithMotionProperties) apart. With
            // JPH_TRACK_SIMULATION_STATS, whose 56 bytes make the slots 368 / 448 bytes, it holds in double precision
            // only: the single precision slots are not a multiple of 64 bytes, the pool works, but not every slot
            // starts a cache line.)
            if (@sizeOf(usize) == 8 and (!track_simulation_stats or Core.double_precision)) std.debug.assert(@sizeOf(Slot) % slot_alignment == 0);
            std.debug.assert(@alignOf(Slot) <= slot_alignment);
            std.debug.assert(@sizeOf(Page) <= header_size and header_size % slot_alignment == 0);
            std.debug.assert(@sizeOf(FreeSlot) <= @sizeOf(Slot) and @alignOf(FreeSlot) <= slot_alignment);
            std.debug.assert(slots_per_page > 0);
        }

        /// The pages that this pool allocated, `pages.len` entries are allocated (at init the pages of max_bodies)
        pages: []*Page = &.{},
        num_pages: u32 = 0,
        /// The page that create takes slots from first
        current: u32 = 0,
        /// Protects the page table and `current` (a plain lock without PhysicsLock's lock order checks: only page locks
        /// and the allocator are used inside it)
        mutex: Mutex = .{},

        /// Number of pages for a number of bodies
        fn pagesFor(num_bodies: u32) u32 {
            return num_bodies / slots_per_page + @intFromBool(num_bodies % slots_per_page != 0);
        }

        /// Reserves the page table for the pages of max_bodies (the pages come with the bodies)
        fn init(self: *BodyPool, allocator: Allocator, max_bodies: u32) Allocator.Error!void {
            std.debug.assert(self.pages.len == 0);
            self.pages = try allocator.alloc(*Page, pagesFor(max_bodies));
        }

        /// The BodyManager is deinitialized, after it destroyed its bodies: frees the pages that hold no body. A page that
        /// still holds bodies (unassigned, or moved to another BodyManager) is freed by the destroy of its last body.
        fn deinit(self: *BodyPool, allocator: Allocator) void {
            for (self.pages[0..self.num_pages]) |page| {
                page.acquire();
                page.owner_alive = false;
                const unused = page.live.load(.monotonic) == 0;
                page.release();
                if (unused)
                    page.free();
            }
            allocator.free(self.pages);
            self.* = .{};
        }

        /// A slot for a body (not initialized). Error: a new page or a larger page table could not be allocated, the pool
        /// is unchanged.
        fn create(self: *BodyPool, allocator: Allocator, io: std.Io) Allocator.Error!*Slot {
            self.mutex.lock(io);
            defer self.mutex.unlock(io);

            // The current page, else the next page that has a free slot
            var index = self.current;
            for (0..self.num_pages) |_| {
                const page = self.pages[index];
                if (page.live.load(.monotonic) < slots_per_page) {
                    self.current = index;
                    return page.take();
                }
                index += 1;
                if (index == self.num_pages)
                    index = 0;
            }

            // All pages are full: a new page
            const page = try self.addPage(allocator);
            self.current = self.num_pages - 1;
            return page.take();
        }

        /// Allocates a page and puts it in the page table, which grows when it is full
        fn addPage(self: *BodyPool, allocator: Allocator) Allocator.Error!*Page {
            const memory = try allocator.alignedAlloc(u8, comptime .fromByteUnits(page_size), page_size);
            errdefer allocator.free(memory);
            if (self.num_pages == self.pages.len) {
                // Beyond the pages of max_bodies
                const pages = try allocator.alloc(*Page, @max(2 * self.pages.len, 1));
                @memcpy(pages[0..self.num_pages], self.pages);
                allocator.free(self.pages);
                self.pages = pages;
            }
            const page: *Page = @ptrCast(memory.ptr);
            page.* = .{ .magic = page_magic, .lock = .init(0), .live = .init(0), .next_unused = 0, .owner_alive = true, .free_list = null, .allocator_ptr = allocator.ptr, .allocator_vtable = allocator.vtable };
            self.pages[self.num_pages] = page;
            self.num_pages += 1;
            return page;
        }

        /// Returns the slot of a body to its page, whatever BodyManager created the body and whether it still exists. The
        /// destroy of the last body of a page whose BodyManager is gone frees the page.
        fn destroy(slot: *Slot) void {
            const page = Page.ofSlot(slot);
            slot.* = undefined; // Safe builds poison the slot, like the allocator poisons freed memory
            const freed: *FreeSlot = @ptrCast(@alignCast(slot));

            page.acquire();
            freed.* = .{ .next = page.free_list };
            page.free_list = freed;
            const live = page.live.load(.monotonic) - 1;
            page.live.store(live, .monotonic);
            const unused = live == 0 and !page.owner_alive;
            page.release();
            if (unused)
                page.free(); // The last body of a page whose BodyManager is gone
        }
    };

    /// Zolt (Phase 13): the state of the body pool (BodyPool), for tests and diagnostics
    pub const BodyPoolStats = struct {
        /// Number of pages that the pool allocated
        num_pages: u32,
        /// Number of entries of the page table (reserved at init for the pages of max_bodies)
        page_table_len: u32,
        /// Number of slots in the pages
        num_slots: u32,
        /// Number of slots that hold a body (also the bodies that were moved to another BodyManager)
        num_used: u32,
        /// Number of slots on the free lists of the pages
        num_free: u32,
    };

    /// Allocates the bodies and the arrays (set at init)
    allocator: Allocator = undefined,

    /// Blocking operations (the mutexes) need io (set at init)
    io: std.Io = undefined,

    /// List of pointers to all bodies. Contains invalid pointers for deleted bodies, check with BodySlot.isValidBodyPointer. Note that this array is reserved to the max bodies that is passed in the Init function so that adding bodies will not reallocate the array.
    /// `bodies.len` is Jolt's mBodies.capacity(), `bodies_size` its mBodies.size().
    bodies: []BodySlot = &.{},

    /// Jolt's mBodies.size(): stored with release under the bodies mutex after the slot is written, loaded with acquire by the lock-free readers
    bodies_size: std.atomic.Value(u32) = .init(0),

    /// Current number of allocated bodies
    num_bodies: u32 = 0,

    /// Index of first entry in mBodies that is unused
    body_id_free_list_start: BodySlot = .free_list_end,

    /// The mutable mutexes (allocated at init)
    mutable_fields: ?*MutableFields = null,

    /// An array of mutexes protecting the bodies in the mBodies array
    body_mutexes: BodyMutexes = .empty,

    /// List of next sequence number for a body ID
    body_sequence_numbers: []u8 = &.{},

    /// List of all active dynamic bodies (size is equal to max amount of bodies)
    active_bodies: [body_type_count][]BodyID = @splat(&.{}),

    /// How many bodies there are in the list of active bodies
    num_active_bodies: [body_type_count]std.atomic.Value(u32) = @splat(.init(0)),

    /// How many of the active bodies have continuous collision detection enabled
    num_active_ccd_bodies: u32 = 0,

    /// List of all bodies that should have their cache invalidated
    bodies_cache_invalid: BodyIDVector = .empty,

    /// Zolt (D5): `bodies_cache_invalid` at the start of the current update (saveContactCacheInvalidation). Its capacity
    /// is kept at least the length of `bodies_cache_invalid` by invalidateContactCacheForBody.
    bodies_cache_invalid_at_update_start: BodyIDVector = .empty,

    /// Listener that is notified whenever a body is activated/deactivated
    activation_listener: ?BodyActivationListener = null,

    /// Cached broadphase layer interface
    broad_phase_layer_interface: ?*const BroadPhaseLayerInterface = null,

    /// Debug system that tries to limit changes to active bodies during the PhysicsSystem::Update() (JPH_ENABLE_ASSERTS)
    active_bodies_locked: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {},

    /// Not initialized (the default constructor, call `init`)
    pub const empty: BodyManager = .{};

    /// sOverrideAllowActivation / sOverrideAllowDeactivation: per thread debug state (JPH_ENABLE_ASSERTS), `void` without asserts
    threadlocal var override_allow_activation: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {};
    threadlocal var override_allow_deactivation: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {};

    /// Initialize the manager (Init). In place: the lock interfaces of the PhysicsSystem point to it, it must not move afterwards.
    pub fn init(self: *BodyManager, allocator: Allocator, io: std.Io, max_bodies: u32, num_body_mutexes_in: u32, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        std.debug.assert(self.mutable_fields == null); // Init twice
        self.* = .{ .allocator = allocator, .io = io };

        // The mutable members (Zolt: allocated first, they hold the bodies mutex)
        const mutable_fields = try allocator.create(MutableFields);
        errdefer allocator.destroy(mutable_fields);
        mutable_fields.* = .{};
        self.mutable_fields = mutable_fields;
        errdefer self.mutable_fields = null;

        var lock: UniqueLock(Mutex) = .init(&mutable_fields.bodies_mutex, io, self, .bodies_list);
        defer lock.deinit();

        // Num body mutexes must be a power of two and not bigger than our MutexMask
        const num_body_mutexes = math.clamp(math.getNextPowerOf2(if (num_body_mutexes_in == 0) 2 * @as(u32, @intCast(std.Thread.getCpuCount() catch 0)) else num_body_mutexes_in), 1, @bitSizeOf(MutexMask));

        // Allocate the body mutexes
        self.body_mutexes = try .init(allocator, num_body_mutexes);
        errdefer self.body_mutexes.deinit(allocator);

        // Allocate space for bodies
        self.bodies = try allocator.alloc(BodySlot, max_bodies);
        errdefer {
            allocator.free(self.bodies);
            self.bodies = &.{};
        }

        // Allocate space for active bodies
        var num_allocated: usize = 0;
        errdefer for (self.active_bodies[0..num_allocated]) |*active_bodies| {
            allocator.free(active_bodies.*);
            active_bodies.* = &.{};
        };
        for (&self.active_bodies) |*active_bodies| {
            std.debug.assert(active_bodies.len == 0);
            active_bodies.* = try allocator.alloc(BodyID, max_bodies);
            num_allocated += 1;
        }

        // Allocate space for sequence numbers
        self.body_sequence_numbers = try allocator.alloc(u8, max_bodies);
        errdefer {
            allocator.free(self.body_sequence_numbers);
            self.body_sequence_numbers = &.{};
        }
        @memset(self.body_sequence_numbers, 0);

        // Zolt (Phase 13): the page table of the body pool (the pages come with the bodies)
        try mutable_fields.body_pool.init(allocator, max_bodies);

        // Keep layer interface
        self.broad_phase_layer_interface = layer_interface;
    }

    /// Destructor
    pub fn deinit(self: *BodyManager) void {
        const mutable_fields = self.mutable_fields orelse return; // Not initialized
        {
            var lock: UniqueLock(Mutex) = .init(&mutable_fields.bodies_mutex, self.io, self, .bodies_list);
            defer lock.deinit();

            // Destroy any bodies that are still alive
            for (self.bodies[0..self.bodies_size.raw]) |b| {
                if (b.isValidBodyPointer())
                    self.deleteBody(b.getBody());
            }

            for (&self.active_bodies) |*active_bodies|
                self.allocator.free(active_bodies.*);
        }

        self.bodies_cache_invalid.deinit(self.allocator);
        self.bodies_cache_invalid_at_update_start.deinit(self.allocator);
        self.allocator.free(self.body_sequence_numbers);
        self.allocator.free(self.bodies);
        self.body_mutexes.deinit(self.allocator);
        mutable_fields.body_pool.deinit(self.allocator); // Zolt (Phase 13): after the bodies were deleted, frees the empty pages
        self.allocator.destroy(mutable_fields);
        self.* = .empty;
    }

    /// The mutable members (the manager must be initialized)
    fn mutableFields(self: *const BodyManager) *MutableFields {
        return self.mutable_fields.?;
    }

    /// Gets the current amount of bodies that are in the body manager
    pub fn getNumBodies(self: *const BodyManager) u32 {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        return self.num_bodies;
    }

    /// Gets the max bodies that we can support
    pub fn getMaxBodies(self: *const BodyManager) u32 {
        return @intCast(self.bodies.len);
    }

    /// Get stats about the bodies in the body manager (slow, iterates through all bodies)
    pub fn getBodyStats(self: *const BodyManager) BodyStats {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        var stats: BodyStats = .{};
        stats.num_bodies = self.num_bodies;
        stats.max_bodies = @intCast(self.bodies.len);

        for (self.bodies[0..self.bodies_size.raw]) |slot| {
            if (slot.isValidBodyPointer()) {
                const body = slot.getBody();
                if (body.isSoftBody()) {
                    stats.num_soft_bodies += 1;
                    if (body.isActive())
                        stats.num_active_soft_bodies += 1;
                } else {
                    switch (body.getMotionType()) {
                        .static => stats.num_bodies_static += 1,

                        .dynamic => {
                            stats.num_bodies_dynamic += 1;
                            if (body.isActive())
                                stats.num_active_bodies_dynamic += 1;
                        },

                        .kinematic => {
                            stats.num_bodies_kinematic += 1;
                            if (body.isActive())
                                stats.num_active_bodies_kinematic += 1;
                        },
                    }
                }
            }
        }

        return stats;
    }

    /// Zolt (Phase 13): the state of the body pool (slow, walks the free lists)
    pub fn getBodyPoolStats(self: *const BodyManager) BodyPoolStats {
        const pool = &self.mutableFields().body_pool;
        pool.mutex.lock(self.io);
        defer pool.mutex.unlock(self.io);

        var stats: BodyPoolStats = .{ .num_pages = pool.num_pages, .page_table_len = @intCast(pool.pages.len), .num_slots = pool.num_pages * BodyPool.slots_per_page, .num_used = 0, .num_free = 0 };
        for (pool.pages[0..pool.num_pages]) |page| {
            page.acquire();
            defer page.release();
            stats.num_used += page.live.load(.monotonic);
            var free = page.free_list;
            while (free) |f| : (free = f.next)
                stats.num_free += 1;
        }
        return stats;
    }

    /// Zolt (Phase 13): true when the body's memory is a slot of a page of this BodyManager's body pool (a rigid body with
    /// motion properties that this BodyManager created), false for a body of another BodyManager, a body without motion
    /// properties and a soft body
    pub fn isBodyInPool(self: *const BodyManager, body: *const Body) bool {
        const pool = &self.mutableFields().body_pool;
        pool.mutex.lock(self.io);
        defer pool.mutex.unlock(self.io);

        // Only compares addresses: the memory around a body that is not in a page is not read
        const page_address = @intFromPtr(body) & ~@as(usize, BodyPool.page_size - 1);
        for (pool.pages[0..pool.num_pages]) |page| {
            if (@intFromPtr(page) == page_address)
                return true;
        }
        return false;
    }

    /// Create a body using creation settings. The returned body will not be part of the body manager yet.
    /// Zolt: `error.InvalidShape` when the settings have no (valid) shape (Jolt crashes), the body is freed on error.
    pub fn allocateBody(self: *const BodyManager, body_creation_settings: *const BodyCreationSettings) CreateBodyError!*Body {
        // Fill in basic properties
        var body: *Body = undefined;
        if (body_creation_settings.hasMassProperties()) {
            // Zolt (Phase 13): a slot of a page of the body pool
            const bmp = try self.mutableFields().body_pool.create(self.allocator, self.io);
            bmp.* = .{};
            body = &bmp.body;
            body.motion_properties = &bmp.motion_properties;
        } else {
            body = try self.allocator.create(Body);
            body.* = .{};
        }
        errdefer self.deleteBody(body);
        body.body_type = .rigid_body;
        try body.applyBodyCreationSettings(self.allocator, body_creation_settings, self.broad_phase_layer_interface.?);

        return body;
    }

    /// Create a soft body using creation settings. The returned body will not be part of the body manager yet.
    /// Zolt: `error.OutOfMemory` when the soft body or its arrays cannot be allocated, nothing is left behind then.
    pub fn allocateSoftBody(self: *const BodyManager, soft_body_creation_settings: *const SoftBodyCreationSettings) CreateBodyError!*Body {
        // Fill in basic properties
        const bmp = try self.allocator.create(SoftBodyWithMotionPropertiesAndShape);
        bmp.* = .init(self.allocator);
        const mp = &bmp.motion_properties;
        const shape = &bmp.shape;
        const body = &bmp.body;
        shape.motion_properties = mp;
        shape.asShape().setEmbedded(); // Jolt: the constructor of SoftBodyWithMotionPropertiesAndShape
        body.body_type = .soft_body;
        body.motion_properties = &mp.base;
        body.shape = .init(shape.asShape());
        errdefer self.deleteBody(body);
        try body.applySoftBodyCreationSettings(soft_body_creation_settings, self.broad_phase_layer_interface.?);

        return body;
    }

    /// Free a body that has not been added to the body manager yet (if it has, use DestroyBodies).
    pub fn freeBody(self: *const BodyManager, body: *Body) void {
        std.debug.assert(body.getID().isInvalid()); // This function should only be called on a body that doesn't have an ID yet, use DestroyBody otherwise

        self.deleteBody(body);
    }

    /// Add a body to the body manager, assigning it the next available ID. Returns false if no more IDs are available.
    pub fn addBody(self: *BodyManager, body: *Body) bool {
        // Return error when body was already added
        if (!body.getID().isInvalid())
            return false;

        // Determine next free index
        var idx: u32 = undefined;
        {
            var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
            defer lock.deinit();

            if (self.body_id_free_list_start != .free_list_end) {
                // Pop an item from the freelist
                std.debug.assert((@intFromEnum(self.body_id_free_list_start) & is_freed_body) != 0);
                idx = @intCast(self.body_id_free_list_start.freedIndex());
                std.debug.assert(!self.bodies[idx].isValidBodyPointer());
                self.body_id_free_list_start = self.bodies[idx];
                self.bodies[idx] = .fromBody(body);
            } else {
                const size = self.bodies_size.raw;
                if (size < self.bodies.len) {
                    // Allocate a new entry, note that the array should not actually resize since we've reserved it at init time
                    idx = size;
                    self.bodies[idx] = .fromBody(body);
                    self.bodies_size.store(size + 1, .release); // After the slot: the lock-free readers load the size with acquire
                } else {
                    // Out of bodies
                    return false;
                }
            }

            // Update cached number of bodies
            self.num_bodies += 1;
        }

        // Get next sequence number and assign the ID
        const seq_no = self.getNextSequenceNumber(idx);
        body.id = .fromIndexAndSequenceNumber(idx, seq_no);
        return true;
    }

    /// Add a body to the body manager, assigning it a custom ID. Returns false if the ID is not valid.
    pub fn addBodyWithCustomID(self: *BodyManager, body: *Body, body_id: BodyID) bool {
        // Return error when body was already added
        if (!body.getID().isInvalid())
            return false;

        {
            var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
            defer lock.deinit();

            // Check if index is beyond the max body ID
            const idx = body_id.getIndex();
            if (idx >= self.bodies.len)
                return false; // Return error

            const size = self.bodies_size.raw;
            if (idx < size) {
                // Body array entry has already been allocated, check if there's a free body here
                if (self.bodies[idx].isValidBodyPointer())
                    return false; // Return error

                // Remove the entry from the freelist
                const idx_start = self.body_id_free_list_start.freedIndex();
                if (idx == idx_start) {
                    // First entry, easy to remove, the start of the list is our next
                    self.body_id_free_list_start = self.bodies[idx];
                } else {
                    // Loop over the freelist and find the entry in the freelist pointing to our index
                    // TODO: This is O(N), see if this becomes a performance problem (don't want to put the freed bodies in a double linked list)
                    var cur = idx_start;
                    while (cur != BodySlot.free_list_end.freedIndex()) {
                        const next = self.bodies[cur].freedIndex();
                        if (next == idx) {
                            self.bodies[cur] = self.bodies[idx];
                            break;
                        }
                        cur = next;
                    }
                    std.debug.assert(cur != BodySlot.free_list_end.freedIndex());
                }

                // Put the body in the slot
                self.bodies[idx] = .fromBody(body);
            } else {
                // Ensure that all body IDs up to this body ID have been allocated and added to the free list
                var new_size = size;
                while (idx > new_size) {
                    // Push the id onto the freelist
                    self.bodies[new_size] = self.body_id_free_list_start;
                    self.body_id_free_list_start = .fromFreedIndex(new_size);
                    new_size += 1;
                }

                // Add the element to the list
                self.bodies[idx] = .fromBody(body);
                self.bodies_size.store(idx + 1, .release); // After the slots: the lock-free readers load the size with acquire
            }

            // Update cached number of bodies
            self.num_bodies += 1;

            // A custom ID establishes the current sequence number for this slot.
            // Keep normal allocation from immediately reissuing the same ID after this body is destroyed.
            self.body_sequence_numbers[idx] = body_id.getSequenceNumber();
        }

        // Assign the ID
        body.id = body_id;
        return true;
    }

    /// Remove a list of bodies from the body manager. `out_bodies` (when given) receives the bodies, it must be at least as long as `body_ids`.
    pub fn removeBodies(self: *BodyManager, body_ids: []const BodyID, out_bodies: ?[]*Body) void {
        // Don't take lock if no bodies are to be destroyed
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        // Update cached number of bodies
        std.debug.assert(self.num_bodies >= body_ids.len);
        self.num_bodies -= @intCast(body_ids.len);

        for (body_ids, 0..) |b, i| {
            // Remove body
            const body = self.removeBodyInternal(b);

            // Clear the ID
            body.id = .invalid;

            // Return the body to the caller
            if (out_bodies) |out|
                out[i] = body;
        }

        if (builtin.mode == .Debug and Core.enable_asserts)
            self.validateFreeList();
    }

    /// Remove a set of bodies from the body manager and frees them.
    pub fn destroyBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be destroyed
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        // Update cached number of bodies
        std.debug.assert(self.num_bodies >= body_ids.len);
        self.num_bodies -= @intCast(body_ids.len);

        for (body_ids) |b| {
            // Remove body
            const body = self.removeBodyInternal(b);

            // Free the body
            self.deleteBody(body);
        }

        if (builtin.mode == .Debug and Core.enable_asserts)
            self.validateFreeList();
    }

    /// Activate a list of bodies.
    /// This function should only be called when an exclusive lock for the bodies are held.
    pub fn activateBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be activated
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        if (Core.enable_asserts) std.debug.assert(!self.active_bodies_locked or override_allow_activation);

        for (body_ids) |body_id| {
            if (!body_id.isInvalid()) {
                const body = self.bodies[body_id.getIndex()].getBody();

                std.debug.assert(body.getID().eql(body_id));
                std.debug.assert(body.isInBroadPhase()); // Use BodyInterface::AddBody to add the body first!

                if (!body.isStatic()) {
                    // Reset sleeping timer so that we don't immediately go to sleep again
                    body.resetSleepTimer();

                    // Check if we're sleeping
                    if (body.motion_properties.?.index_in_active_bodies.load(.monotonic) == Body.inactive_index) {
                        self.addBodyToActiveBodies(body);

                        // Call activation listener
                        if (self.activation_listener) |listener|
                            listener.onBodyActivated(body_id, body.getUserData());
                    }
                }
            }
        }
    }

    /// Deactivate a list of bodies.
    /// This function should only be called when an exclusive lock for the bodies are held.
    pub fn deactivateBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be deactivated
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        if (Core.enable_asserts) std.debug.assert(!self.active_bodies_locked or override_allow_deactivation);

        for (body_ids) |body_id| {
            if (!body_id.isInvalid()) {
                const body = self.bodies[body_id.getIndex()].getBody();

                std.debug.assert(body.getID().eql(body_id));
                std.debug.assert(body.isInBroadPhase()); // Use BodyInterface::AddBody to add the body first!

                if (body.motion_properties != null and body.motion_properties.?.index_in_active_bodies.load(.monotonic) != Body.inactive_index) {
                    const mp = body.motion_properties.?;

                    // Remove the body from the active bodies list
                    self.removeBodyFromActiveBodies(body);

                    // Mark this body as no longer active
                    mp.island_index = Body.inactive_index;

                    // Reset simulation stats (JPH_TRACK_SIMULATION_STATS)
                    if (track_simulation_stats) mp.simulation_stats.reset();

                    // Reset velocity
                    mp.linear_velocity = Vec3.zero();
                    mp.angular_velocity = Vec3.zero();

                    // Call activation listener
                    if (self.activation_listener) |listener|
                        listener.onBodyDeactivated(body_id, body.getUserData());
                }
            }
        }
    }

    /// Update the motion quality for a body
    pub fn setMotionQuality(self: *BodyManager, body: *Body, motion_quality: MotionQuality) void {
        const mp = body.getMotionPropertiesUncheckedMut() orelse return;
        if (mp.getMotionQuality() != motion_quality) {
            var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
            defer lock.deinit();

            if (Core.enable_asserts) std.debug.assert(!self.active_bodies_locked);

            const is_active = body.isActive();
            if (is_active and mp.getMotionQuality() == .linear_cast)
                self.num_active_ccd_bodies -= 1;

            mp.motion_quality = motion_quality;

            if (is_active and mp.getMotionQuality() == .linear_cast)
                self.num_active_ccd_bodies += 1;
        }
    }

    /// Get copy of the list of active bodies under protection of a lock. `allocator` owns `out_body_ids`.
    pub fn getActiveBodies(self: *const BodyManager, allocator: Allocator, body_type: BodyType, out_body_ids: *BodyIDVector) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        const active_bodies = self.active_bodies[@intFromEnum(body_type)];
        const num_active_bodies = self.num_active_bodies[@intFromEnum(body_type)].load(.monotonic);
        out_body_ids.clearRetainingCapacity();
        try out_body_ids.appendSlice(allocator, active_bodies[0..num_active_bodies]);
    }

    /// Get the list of active bodies. Note: Not thread safe. The active bodies list can change at any moment.
    pub fn getActiveBodiesUnsafe(self: *const BodyManager, body_type: BodyType) [*]const BodyID {
        return self.active_bodies[@intFromEnum(body_type)].ptr;
    }

    /// Get the number of active bodies.
    pub fn getNumActiveBodies(self: *const BodyManager, body_type: BodyType) u32 {
        return self.num_active_bodies[@intFromEnum(body_type)].load(.acquire);
    }

    /// Get the number of active bodies that are using continuous collision detection
    pub fn getNumActiveCCDBodies(self: *const BodyManager) u32 {
        return self.num_active_ccd_bodies;
    }

    /// Listener that is notified whenever a body is activated/deactivated
    pub fn setBodyActivationListener(self: *BodyManager, listener: ?BodyActivationListener) void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        self.activation_listener = listener;
    }

    pub fn getBodyActivationListener(self: *const BodyManager) ?BodyActivationListener {
        return self.activation_listener;
    }

    /// Check if this is a valid body pointer. When a body is freed the memory that the pointer occupies is reused to store a freelist.
    pub fn isValidBodyPointer(body: BodySlot) bool {
        return body.isValidBodyPointer();
    }

    /// Get all bodies. Note that this can contain invalid body pointers, call isValidBodyPointer to check.
    pub fn getBodies(self: *const BodyManager) []const BodySlot {
        return self.bodies[0..self.bodies_size.load(.acquire)];
    }

    /// Get all bodies. Note that this can contain invalid body pointers, call isValidBodyPointer to check. (non-const overload)
    pub fn getBodiesMut(self: *BodyManager) []BodySlot {
        return self.bodies[0..self.bodies_size.load(.acquire)];
    }

    /// Get all body IDs under the protection of a lock. `allocator` owns `out_bodies`.
    pub fn getBodyIDs(self: *const BodyManager, allocator: Allocator, out_bodies: *BodyIDVector) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        // Reserve space for all bodies
        out_bodies.clearRetainingCapacity();
        try out_bodies.ensureTotalCapacity(allocator, self.num_bodies);

        // Iterate the list and find the bodies that are not null
        for (self.bodies[0..self.bodies_size.raw]) |b| {
            if (b.isValidBodyPointer())
                out_bodies.appendAssumeCapacity(b.getBody().getID());
        }

        // Validate that our reservation was correct
        std.debug.assert(out_bodies.items.len == self.num_bodies);
    }

    /// Access a body (not protected by lock)
    pub fn getBody(self: *const BodyManager, id: BodyID) *const Body {
        return self.bodies[id.getIndex()].getBody();
    }

    /// Access a body (not protected by lock, non-const overload)
    pub fn getBodyMut(self: *BodyManager, id: BodyID) *Body {
        return self.bodies[id.getIndex()].getBody();
    }

    /// Access a body, will return a null if the body ID is no longer valid (not protected by lock)
    pub fn tryGetBody(self: *const BodyManager, id: BodyID) ?*const Body {
        return self.tryGetBodyInternal(id);
    }

    /// Access a body, will return a null if the body ID is no longer valid (not protected by lock, non-const overload)
    pub fn tryGetBodyMut(self: *BodyManager, id: BodyID) ?*Body {
        return self.tryGetBodyInternal(id);
    }

    fn tryGetBodyInternal(self: *const BodyManager, id: BodyID) ?*Body {
        const idx = id.getIndex();
        if (idx >= self.bodies_size.load(.acquire))
            return null;

        const body = self.bodies[idx];
        if (body.isValidBodyPointer() and body.getBody().getID().eql(id))
            return body.getBody();

        return null;
    }

    /// Access the mutex for a single body
    pub fn getMutexForBody(self: *const BodyManager, id: BodyID) *SharedMutex {
        return self.body_mutexes.getMutexByObjectIndex(id.getIndex());
    }

    ///@name Batch body mutex access (do not use directly)
    pub fn getAllBodiesMutexMask(self: *const BodyManager) MutexMask {
        return if (self.body_mutexes.getNumMutexes() == @bitSizeOf(MutexMask)) ~@as(MutexMask, 0) else (@as(MutexMask, 1) << @intCast(self.body_mutexes.getNumMutexes())) - 1;
    }

    pub fn getMutexMask(self: *const BodyManager, bodies: []const BodyID) MutexMask {
        std.debug.assert(@bitSizeOf(MutexMask) >= self.body_mutexes.getNumMutexes()); // MutexMask must have enough bits

        if (bodies.len >= self.body_mutexes.getNumMutexes()) {
            // Just lock everything if there are too many bodies
            return self.getAllBodiesMutexMask();
        } else {
            var mask: MutexMask = 0;
            for (bodies) |b| {
                if (!b.isInvalid()) {
                    const index = self.body_mutexes.getMutexIndex(b.getIndex());
                    mask |= @as(MutexMask, 1) << @intCast(index);
                }
            }
            return mask;
        }
    }

    pub fn lockRead(self: *const BodyManager, mutex_mask: MutexMask) void {
        PhysicsLock.checkLock(self, .per_body);

        var index: u32 = 0;
        var mask = mutex_mask;
        while (mask != 0) : ({
            mask >>= 1;
            index += 1;
        }) {
            if (mask & 1 != 0)
                self.body_mutexes.getMutexByIndex(index).lockShared(self.io);
        }
    }

    pub fn unlockRead(self: *const BodyManager, mutex_mask: MutexMask) void {
        PhysicsLock.checkUnlock(self, .per_body);

        var index: u32 = 0;
        var mask = mutex_mask;
        while (mask != 0) : ({
            mask >>= 1;
            index += 1;
        }) {
            if (mask & 1 != 0)
                self.body_mutexes.getMutexByIndex(index).unlockShared(self.io);
        }
    }

    pub fn lockWrite(self: *const BodyManager, mutex_mask: MutexMask) void {
        PhysicsLock.checkLock(self, .per_body);

        var index: u32 = 0;
        var mask = mutex_mask;
        while (mask != 0) : ({
            mask >>= 1;
            index += 1;
        }) {
            if (mask & 1 != 0)
                self.body_mutexes.getMutexByIndex(index).lock(self.io);
        }
    }

    pub fn unlockWrite(self: *const BodyManager, mutex_mask: MutexMask) void {
        PhysicsLock.checkUnlock(self, .per_body);

        var index: u32 = 0;
        var mask = mutex_mask;
        while (mask != 0) : ({
            mask >>= 1;
            index += 1;
        }) {
            if (mask & 1 != 0)
                self.body_mutexes.getMutexByIndex(index).unlock(self.io);
        }
    }

    /// Lock all bodies. This should only be done during PhysicsSystem::Update().
    pub fn lockAllBodies(self: *const BodyManager) void {
        PhysicsLock.checkLock(self, .per_body);
        self.body_mutexes.lockAll(self.io);

        PhysicsLock.lock(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);
    }

    /// Unlock all bodies. This should only be done during PhysicsSystem::Update().
    pub fn unlockAllBodies(self: *const BodyManager) void {
        PhysicsLock.unlock(&self.mutableFields().bodies_mutex, self.io, self, .bodies_list);

        PhysicsLock.checkUnlock(self, .per_body);
        self.body_mutexes.unlockAll(self.io);
    }

    /// Set the Body::EFlags::InvalidateContactCache flag for the specified body. This means that the collision cache is invalid for any body pair involving that body until the next physics step.
    /// Zolt: on out of memory the flag is cleared again and the error returned (the body stays valid).
    pub fn invalidateContactCacheForBody(self: *BodyManager, body: *Body) Allocator.Error!void {
        // If this is the first time we flip the collision cache invalid flag, we need to add it to an internal list to ensure we reset the flag at the end of the physics update
        if (body.invalidateContactCacheInternal()) {
            // lock_guard: a plain lock, Jolt does no lock order check for this mutex
            const mutex = &self.mutableFields().bodies_cache_invalid_mutex;
            mutex.lock(self.io);
            defer mutex.unlock(self.io);
            // Zolt (D5): room for the copy that an update makes (saveContactCacheInvalidation)
            self.bodies_cache_invalid_at_update_start.ensureTotalCapacity(self.allocator, self.bodies_cache_invalid.items.len + 1) catch |err| {
                body.validateContactCacheInternal();
                return err;
            };
            self.bodies_cache_invalid.append(self.allocator, body.getID()) catch |err| {
                body.validateContactCacheInternal();
                return err;
            };
        }
    }

    /// Reset the Body::EFlags::InvalidateContactCache flag for all bodies. All contact pairs in the contact cache will now by valid again.
    pub fn validateContactCacheForAllBodies(self: *BodyManager) void {
        // lock_guard: a plain lock, Jolt does no lock order check for this mutex
        const mutex = &self.mutableFields().bodies_cache_invalid_mutex;
        mutex.lock(self.io);
        defer mutex.unlock(self.io);

        for (self.bodies_cache_invalid.items) |b| {
            // The body may have been removed between the call to InvalidateContactCacheForBody and this call, so check if it still exists
            if (self.tryGetBodyMut(b)) |body|
                body.validateContactCacheInternal();
        }
        self.bodies_cache_invalid.clearRetainingCapacity();
    }

    /// Zolt (D5): remember the bodies whose contact cache is invalid at the start of an update (called by
    /// PhysicsSystem.update before the first job runs; does not allocate)
    pub fn saveContactCacheInvalidation(self: *BodyManager) void {
        const mutex = &self.mutableFields().bodies_cache_invalid_mutex;
        mutex.lock(self.io);
        defer mutex.unlock(self.io);

        self.bodies_cache_invalid_at_update_start.clearRetainingCapacity();
        self.bodies_cache_invalid_at_update_start.appendSliceAssumeCapacity(self.bodies_cache_invalid.items);
    }

    /// Zolt (D5): after an update that ran out of memory inside a job (called by PhysicsSystem.update with all bodies
    /// locked; does not allocate): the bodies flagged during the update are reset, the bodies that were flagged at its
    /// start (saveContactCacheInvalidation) are flagged again, and the list is the list at the start of the update.
    /// The flags are not part of the saved state, so the update after restoring a state saved before the failed update
    /// treats the contact cache like the failed update did.
    pub fn restoreContactCacheInvalidation(self: *BodyManager) void {
        const mutex = &self.mutableFields().bodies_cache_invalid_mutex;
        mutex.lock(self.io);
        defer mutex.unlock(self.io);

        for (self.bodies_cache_invalid.items) |b| {
            if (self.tryGetBodyMut(b)) |body|
                body.validateContactCacheInternal();
        }
        for (self.bodies_cache_invalid_at_update_start.items) |b| {
            if (self.tryGetBodyMut(b)) |body|
                _ = body.invalidateContactCacheInternal();
        }

        // The list held these entries at the start of the update and never shrinks its capacity
        self.bodies_cache_invalid.clearRetainingCapacity();
        self.bodies_cache_invalid.appendSliceAssumeCapacity(self.bodies_cache_invalid_at_update_start.items);
    }

    /// Return the BroadPhaseLayerInterface that this class has been initialized with
    pub fn getBroadPhaseLayerInterface(self: *const BodyManager) *const BroadPhaseLayerInterface {
        return self.broad_phase_layer_interface.?;
    }

    /// Saving state for replay (Zolt: the list of bodies to save is a temporary array, hence Allocator.Error)
    pub fn saveState(self: *const BodyManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        self.lockAllBodies();
        defer self.unlockAllBodies();

        // Determine which bodies to save
        var bodies: std.ArrayList(*const Body) = .empty;
        defer bodies.deinit(self.allocator);
        try bodies.ensureTotalCapacity(self.allocator, self.num_bodies);
        for (self.bodies[0..self.bodies_size.raw]) |slot| {
            if (slot.isValidBodyPointer()) {
                const b = slot.getBody();
                if (b.isInBroadPhase() and (filter == null or filter.?.shouldSaveBody(b)))
                    bodies.appendAssumeCapacity(b);
            }
        }

        // Write state of bodies
        const num_bodies: u32 = @intCast(bodies.items.len);
        stream.write(num_bodies);
        for (bodies.items) |b| {
            stream.write(b.getID());
            stream.write(b.isActive());
            b.saveState(stream);
        }
    }

    /// Restoring state for replay. Returns false if failed.
    /// Zolt: the lists of bodies to (de)activate are reserved before any body is restored. A stream written by saveState
    /// names every body at most once, so it never needs more and out of memory changes no body. When not validating, a
    /// malformed stream that names a body more than once can make an append allocate after earlier bodies were restored
    /// (Jolt's push_back); out of memory then leaves those bodies restored and the activation unchanged.
    pub fn restoreState(self: *BodyManager, stream: *StateRecorder) Allocator.Error!bool {
        var bodies_to_activate: BodyIDVector = .empty;
        defer bodies_to_activate.deinit(self.allocator);
        var bodies_to_deactivate: BodyIDVector = .empty;
        defer bodies_to_deactivate.deinit(self.allocator);

        {
            self.lockAllBodies();
            defer self.unlockAllBodies();

            if (stream.isValidating()) {
                // Read state of bodies, note this reads it in a way to be consistent with validation
                var old_num_bodies: u32 = 0;
                for (self.bodies[0..self.bodies_size.raw]) |slot| {
                    if (slot.isValidBodyPointer() and slot.getBody().isInBroadPhase())
                        old_num_bodies += 1;
                }
                var num_bodies = old_num_bodies; // Initialize to current value for validation
                stream.read(&num_bodies);
                if (num_bodies != old_num_bodies) {
                    if (Core.enable_asserts) @panic("Cannot handle adding/removing bodies");
                    return false;
                }

                try bodies_to_activate.ensureTotalCapacity(self.allocator, num_bodies);
                try bodies_to_deactivate.ensureTotalCapacity(self.allocator, num_bodies);

                for (self.bodies[0..self.bodies_size.raw]) |slot| {
                    if (slot.isValidBodyPointer() and slot.getBody().isInBroadPhase()) {
                        const b = slot.getBody();
                        var body_id = b.getID(); // Initialize to current value for validation
                        stream.read(&body_id);
                        if (!body_id.eql(b.getID())) {
                            if (Core.enable_asserts) @panic("Cannot handle adding/removing bodies");
                            return false;
                        }
                        var is_active = b.isActive(); // Initialize to current value for validation
                        stream.read(&is_active);
                        if (is_active != b.isActive()) {
                            if (is_active)
                                bodies_to_activate.appendAssumeCapacity(body_id)
                            else
                                bodies_to_deactivate.appendAssumeCapacity(body_id);
                        }
                        b.restoreState(stream);
                    }
                }
            } else {
                // Not validating, we can be a bit more loose, read number of bodies
                var num_bodies: u32 = 0;
                stream.read(&num_bodies);

                // Zolt: at most the number of bodies in the manager can change their activation (a valid ID per body), unless
                // the stream names a body more than once (not written by saveState): then the appends below can allocate
                const max_to_change = @min(num_bodies, self.num_bodies);
                try bodies_to_activate.ensureTotalCapacity(self.allocator, max_to_change);
                try bodies_to_deactivate.ensureTotalCapacity(self.allocator, max_to_change);

                // Iterate over the stored bodies and restore their state
                for (0..num_bodies) |_| {
                    var body_id: BodyID = .invalid;
                    stream.read(&body_id);
                    const b = self.tryGetBodyMut(body_id) orelse {
                        if (Core.enable_asserts) @panic("Restoring state for non-existing body");
                        return false;
                    };
                    var is_active: bool = false;
                    stream.read(&is_active);
                    if (is_active != b.isActive()) {
                        if (is_active)
                            try bodies_to_activate.append(self.allocator, body_id)
                        else
                            try bodies_to_deactivate.append(self.allocator, body_id);
                    }
                    b.restoreState(stream);
                }
            }
        }

        {
            var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
            defer lock.deinit();

            for (bodies_to_activate.items) |body_id| {
                const body = self.tryGetBodyMut(body_id).?;
                self.addBodyToActiveBodies(body);
            }

            for (bodies_to_deactivate.items) |body_id| {
                const body = self.tryGetBodyMut(body_id).?;
                self.removeBodyFromActiveBodies(body);
            }
        }

        return true;
    }

    /// Save the state of a single body for replay
    pub fn saveBodyState(self: *const BodyManager, body: *const Body, stream: *StateRecorder) void {
        _ = self;
        stream.write(body.isActive());

        body.saveState(stream);
    }

    /// Restore the state of a single body for replay
    pub fn restoreBodyState(self: *BodyManager, body: *Body, stream: *StateRecorder) void {
        var is_active = body.isActive();
        stream.read(&is_active);

        body.restoreState(stream);

        if (is_active != body.isActive()) {
            var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
            defer lock.deinit();

            if (Core.enable_asserts) std.debug.assert(!self.active_bodies_locked or override_allow_activation);

            if (is_active)
                self.addBodyToActiveBodies(body)
            else
                self.removeBodyFromActiveBodies(body);
        }
    }

    /// How to color the shapes (JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)
    pub const ShapeColor = if (Core.debug_renderer) enum {
        /// Random color per instance
        instance_color,
        /// Convex = green, scaled = yellow, compound = orange, mesh = red
        shape_type_color,
        /// Static = grey, keyframed = green, dynamic = random color per instance
        motion_type_color,
        /// Static = grey, keyframed = green, dynamic = yellow, sleeping = red
        sleep_color,
        /// Static = grey, active = random color per island, sleeping = light grey
        island_color,
        /// Color as defined by the PhysicsMaterial of the shape
        material_color,
    } else void;

    /// Draw settings (JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)
    ///
    /// Note that there are several debug drawing features that are not exposed through this interface since they use information
    /// that is only available deep inside the simulation update and are mostly there to facilitate debugging Jolt. These options
    /// use DebugRenderer::sInstance to draw.
    /// (Zolt: they draw into the renderer of the physics system, PhysicsSystem.setDebugRenderer, and their flags are the
    /// fields of `DebugRenderer.flags`, RemainingFeaturesArchitecture.md R4.)
    ///
    /// E.g.:
    /// * To draw contact information, use ContactConstraintManager::sDrawContactManifolds.
    /// * To draw when continuous collision detection is used, use PhysicsSystem::sDrawMotionQualityLinearCast.
    /// * To draw what's going on in a CharacterVirtual update, use CharacterVirtual::sDrawConstraints, CharacterVirtual::sDrawWalkStairs and CharacterVirtual::sDrawStickToFloor.
    /// * To draw the volume of water that interacts with a shape, use Shape::sDrawSubmergedVolumes.
    pub const DrawSettings = if (Core.debug_renderer) struct {
        /// Draw the GetSupport() function, used for convex collision detection
        draw_get_support_function: bool = false,
        /// When drawing the support function, also draw which direction mapped to a specific support point
        draw_support_direction: bool = false,
        /// Draw the faces that were found colliding during collision detection
        draw_get_supporting_face: bool = false,
        /// Draw the shapes of all bodies
        draw_shape: bool = true,
        /// When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
        draw_shape_wireframe: bool = false,
        /// Coloring scheme to use for shapes
        draw_shape_color: ShapeColor = .motion_type_color,
        /// Draw a bounding box per body
        draw_bounding_box: bool = false,
        /// Draw the center of mass for each body
        draw_center_of_mass_transform: bool = false,
        /// Draw the world transform (which can be different than the center of mass) for each body
        draw_world_transform: bool = false,
        /// Draw the velocity vector for each body
        draw_velocity: bool = false,
        /// Draw the mass and inertia (as the box equivalent) for each body
        draw_mass_and_inertia: bool = false,
        /// Draw stats regarding the sleeping algorithm of each body
        draw_sleep_stats: bool = false,
        /// Draw the vertices of soft bodies
        draw_soft_body_vertices: bool = false,
        /// Draw the velocities of the vertices of soft bodies
        draw_soft_body_vertex_velocities: bool = false,
        /// Draw the edge constraints of soft bodies
        draw_soft_body_edge_constraints: bool = false,
        /// Draw the bend constraints of soft bodies
        draw_soft_body_bend_constraints: bool = false,
        /// Draw the volume constraints of soft bodies
        draw_soft_body_volume_constraints: bool = false,
        /// Draw the skin constraints of soft bodies
        draw_soft_body_skin_constraints: bool = false,
        /// Draw the LRA constraints of soft bodies
        draw_soft_body_lra_constraints: bool = false,
        /// Draw the rods of soft bodies
        draw_soft_body_rods: bool = false,
        /// Draw the rod states (orientation and angular velocity) of soft bodies
        draw_soft_body_rod_states: bool = false,
        /// Draw the rod bend twist constraints of soft bodies
        draw_soft_body_rod_bend_twist_constraints: bool = false,
        /// Draw the predicted bounds of soft bodies
        draw_soft_body_predicted_bounds: bool = false,
        /// Coloring scheme to use for soft body constraints
        draw_soft_body_constraint_color: SoftBodyConstraintColor = .constraint_type,
    } else void;

    /// The optional parameters of draw (Jolt's default argument; JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)
    pub const DrawOptions = if (Core.debug_renderer) struct {
        /// Only the bodies that pass the filter are drawn (null: every body)
        body_filter: ?*const BodyDrawFilter = null,
    } else void;

    /// Draw the state of the bodies (debugging purposes) (JPH_DEBUG_RENDERER)
    pub const draw = if (Core.debug_renderer) drawImpl else {};
    fn drawImpl(self: *BodyManager, draw_settings: *const DrawSettings, physics_settings: *const PhysicsSettings, renderer: *DebugRenderer, opts: DrawOptions) void {
        self.lockAllBodies();
        defer self.unlockAllBodies();

        for (self.bodies[0..self.bodies_size.raw]) |slot| {
            if (slot.isValidBodyPointer() and slot.getBody().isInBroadPhase() and (opts.body_filter == null or opts.body_filter.?.shouldDraw(slot.getBody()))) {
                const body = slot.getBody();
                std.debug.assert(self.bodies[body.getID().getIndex()] == slot);

                const is_sensor = body.isSensor();

                // Determine drawing mode
                var color: Color = undefined;
                if (is_sensor) {
                    color = Color.yellow;
                } else switch (draw_settings.draw_shape_color) {
                    .instance_color => {
                        // Each instance has own color
                        color = Color.getDistinctColor(@intCast(body.id.getIndex()));
                    },

                    .shape_type_color => color = ShapeFunctions.get(body.getShape().getSubType()).color,

                    .motion_type_color => {
                        // Determine color based on motion type
                        switch (body.motion_type) {
                            .static => color = Color.grey,
                            .kinematic => color = Color.green,
                            .dynamic => color = Color.getDistinctColor(@intCast(body.id.getIndex())),
                        }
                    },

                    .sleep_color => {
                        // Determine color based on motion type
                        switch (body.motion_type) {
                            .static => color = Color.grey,
                            .kinematic => color = if (body.isActive()) Color.green else Color.red,
                            .dynamic => color = if (body.isActive()) Color.yellow else Color.red,
                        }
                    },

                    .island_color => {
                        // Determine color based on motion type
                        switch (body.motion_type) {
                            .static => color = Color.grey,
                            .kinematic, .dynamic => {
                                const idx = body.getMotionProperties().getIslandIndexInternal();
                                color = if (idx != Body.inactive_index) Color.getDistinctColor(@intCast(idx)) else Color.light_grey;
                            },
                        }
                    },

                    .material_color => color = Color.white,
                }

                // Draw the results of GetSupportFunction
                if (draw_settings.draw_get_support_function)
                    body.getShape().drawGetSupportFunction(renderer, body.getCenterOfMassTransform(), Vec3.one(), color, draw_settings.draw_support_direction);

                // Draw the results of GetSupportingFace
                if (draw_settings.draw_get_supporting_face)
                    body.getShape().drawGetSupportingFace(renderer, body.getCenterOfMassTransform(), Vec3.one());

                // Draw the shape
                if (draw_settings.draw_shape)
                    body.getShape().draw(renderer, body.getCenterOfMassTransform(), Vec3.one(), color, draw_settings.draw_shape_color == .material_color, draw_settings.draw_shape_wireframe or is_sensor);

                // Draw bounding box
                if (draw_settings.draw_bounding_box)
                    renderer.drawWireBox(body.bounds, color);

                // Draw center of mass transform
                if (draw_settings.draw_center_of_mass_transform)
                    renderer.drawCoordinateSystem(body.getCenterOfMassTransform(), .{ .size = 0.2 });

                // Draw world transform
                if (draw_settings.draw_world_transform)
                    renderer.drawCoordinateSystem(body.getWorldTransform(), .{ .size = 0.2 });

                // Draw world space linear and angular velocity
                if (draw_settings.draw_velocity) {
                    const pos = body.getCenterOfMassPosition();
                    renderer.drawArrow(pos, pos.addVec3(body.getLinearVelocity()), Color.green, 0.1);
                    renderer.drawArrow(pos, pos.addVec3(body.getAngularVelocity()), Color.red, 0.1);
                }

                if (draw_settings.draw_mass_and_inertia and body.isDynamic()) {
                    const mp = body.getMotionProperties();
                    if (mp.getInverseMass() > 0.0 and !Vec3.equals(mp.getInverseInertiaDiagonal(), Vec3.zero()).testAnyXYZTrue()) {
                        // Invert mass again
                        const mass = 1.0 / mp.getInverseMass();

                        // Invert diagonal again
                        const diagonal = mp.getInverseInertiaDiagonal().reciprocal();

                        // Determine how big of a box has the equivalent inertia
                        const box_size = MassProperties.getEquivalentSolidBoxSize(mass, diagonal);

                        // Draw box with equivalent inertia
                        renderer.drawWireBoxMatrix(body.getCenterOfMassTransform().mulMat44(Mat44.rotationQuat(mp.getInertiaRotation())), AABox.init(box_size.mulScalar(-0.5), box_size.mulScalar(0.5)), Color.orange);

                        // Draw mass
                        // StringFormat("%.2f", ...): its buffer holds 1023 characters
                        var text_buffer: [1023]u8 = undefined;
                        var writer: std.Io.Writer = .fixed(&text_buffer);
                        writer.print("{f}", .{StringTools.cppFixed(mass, 2)}) catch {};
                        renderer.drawText3D(body.getCenterOfMassPosition(), writer.buffered(), .{ .color = Color.orange, .height = 0.2 });
                    }
                }

                if (draw_settings.draw_sleep_stats and body.isDynamic() and body.isActive()) {
                    const mp = body.getMotionProperties();

                    // Draw stats to know which bodies could go to sleep
                    // StringFormat("t: %.1f", ...): its buffer holds 1023 characters
                    var text_buffer: [1023]u8 = undefined;
                    var writer: std.Io.Writer = .fixed(&text_buffer);
                    writer.print("t: {f}", .{StringTools.cppFixed(mp.sleep_test_timer, 1)}) catch {};
                    const g_value = math.clamp(255.0 * mp.sleep_test_timer / physics_settings.time_before_sleep, 0.0, 255.0);
                    // uint8(float): Clamp keeps NaN (a zero timer with a zero mTimeBeforeSleep), which Jolt's x86-64 build
                    // converts with cvttss2si and truncates to 0 (porting guide section 8, rule 12)
                    const g: u8 = if (std.math.isNan(g_value)) 0 else @intFromFloat(g_value);
                    const sleep_color = Color.initRgb(0, 255 - g, g);
                    renderer.drawText3D(body.getCenterOfMassPosition(), writer.buffered(), .{ .color = sleep_color, .height = 0.2 });
                    for (0..3) |i| {
                        const center = mp.sleep_test_spheres[i].getCenter();
                        // JPH_IF_DOUBLE_PRECISION(GetSleepTestOffset() +)
                        renderer.drawMarker(if (Core.double_precision) mp.getSleepTestOffset().addVec3(center) else center, sleep_color, mp.sleep_test_spheres[i].getRadius());
                    }
                }

                if (body.isSoftBody()) {
                    const mp = SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties());
                    const com = body.getCenterOfMassTransform();

                    if (draw_settings.draw_soft_body_vertices)
                        mp.drawVertices(renderer, com);

                    if (draw_settings.draw_soft_body_vertex_velocities)
                        mp.drawVertexVelocities(renderer, com);

                    if (draw_settings.draw_soft_body_edge_constraints)
                        mp.drawEdgeConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_rods)
                        mp.drawRods(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_rod_states)
                        mp.drawRodStates(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_rod_bend_twist_constraints)
                        mp.drawRodBendTwistConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_bend_constraints)
                        mp.drawBendConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_volume_constraints)
                        mp.drawVolumeConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_skin_constraints)
                        mp.drawSkinConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_lra_constraints)
                        mp.drawLRAConstraints(renderer, com, draw_settings.draw_soft_body_constraint_color);

                    if (draw_settings.draw_soft_body_predicted_bounds)
                        mp.drawPredictedBounds(renderer, com);
                }
            }
        }
    }

    /// Lock the active body list, asserts when Activate/DeactivateBody is called. (JPH_ENABLE_ASSERTS: does nothing without asserts)
    pub fn setActiveBodiesLocked(self: *BodyManager, locked: bool) void {
        if (Core.enable_asserts) self.active_bodies_locked = locked;
    }

    /// Per thread override of the locked state, to be used by the PhysicsSystem only! (JPH_ENABLE_ASSERTS: does nothing without asserts)
    /// `var grant: BodyManager.GrantActiveBodiesAccess = .init(true, false); defer grant.deinit();`
    pub const GrantActiveBodiesAccess = struct {
        pub fn init(allow_activation: bool, allow_deactivation: bool) GrantActiveBodiesAccess {
            if (Core.enable_asserts) {
                std.debug.assert(!getOverrideAllowActivation());
                setOverrideAllowActivation(allow_activation);

                std.debug.assert(!getOverrideAllowDeactivation());
                setOverrideAllowDeactivation(allow_deactivation);
            }
            return .{};
        }

        pub fn deinit(self: *GrantActiveBodiesAccess) void {
            _ = self;
            if (Core.enable_asserts) {
                setOverrideAllowActivation(false);
                setOverrideAllowDeactivation(false);
            }
        }
    };

    /// Validate if the cached bounding boxes are correct for all active bodies (JPH_DEBUG)
    pub fn validateActiveBodyBounds(self: *BodyManager) void {
        if (builtin.mode != .Debug) return;

        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        for (0..body_type_count) |body_type| {
            const active = self.active_bodies[body_type][0..self.num_active_bodies[body_type].load(.monotonic)];
            for (active) |id| {
                const body = self.bodies[id.getIndex()].getBody();
                body.validateCachedBounds();
            }
        }
    }

    /// Resets the per body simulation stats (JPH_TRACK_SIMULATION_STATS: only declared when `track_simulation_stats` is on)
    pub const resetSimulationStats = if (track_simulation_stats) resetSimulationStatsImpl else {};

    fn resetSimulationStatsImpl(self: *BodyManager) void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        for (0..body_type_count) |body_type| {
            const active = self.active_bodies[body_type][0..self.num_active_bodies[body_type].load(.monotonic)];
            for (active) |id| {
                const body = self.bodies[id.getIndex()].getBody();
                body.motion_properties.?.simulation_stats.reset();
            }
        }
    }

    fn getOverrideAllowActivation() bool {
        return override_allow_activation;
    }

    fn setOverrideAllowActivation(value: bool) void {
        override_allow_activation = value;
    }

    fn getOverrideAllowDeactivation() bool {
        return override_allow_deactivation;
    }

    fn setOverrideAllowDeactivation(value: bool) void {
        override_allow_deactivation = value;
    }

    /// Increment and get the sequence number of the body (we intentionally overflow the uint8 sequence number)
    fn getNextSequenceNumber(self: *BodyManager, body_index: u32) u8 {
        self.body_sequence_numbers[body_index] +%= 1;
        return self.body_sequence_numbers[body_index];
    }

    /// Add a single body to mActiveBodies, note doesn't lock the active body mutex!
    fn addBodyToActiveBodies(self: *BodyManager, body: *Body) void {
        // Select the correct array to use
        const body_type = @intFromEnum(body.getBodyType());
        const num_active_bodies = &self.num_active_bodies[body_type];
        const active_bodies = self.active_bodies[body_type];

        const mp = body.motion_properties.?;
        const num_active_bodies_val = num_active_bodies.load(.monotonic);
        mp.index_in_active_bodies.store(num_active_bodies_val, .monotonic);
        std.debug.assert(num_active_bodies_val < self.getMaxBodies());
        active_bodies[num_active_bodies_val] = body.getID();
        _ = num_active_bodies.fetchAdd(1, .release); // Increment atomic after setting the body ID so that PhysicsSystem::JobFindCollisions (which doesn't lock the mActiveBodiesMutex) will only read valid IDs

        // Count CCD bodies
        if (mp.getMotionQuality() == .linear_cast)
            self.num_active_ccd_bodies += 1;
    }

    /// Remove a single body from mActiveBodies, note doesn't lock the active body mutex!
    fn removeBodyFromActiveBodies(self: *BodyManager, body: *Body) void {
        // Select the correct array to use
        const body_type = @intFromEnum(body.getBodyType());
        const num_active_bodies = &self.num_active_bodies[body_type];
        const active_bodies = self.active_bodies[body_type];

        const last_body_index = num_active_bodies.load(.monotonic) - 1;
        const mp = body.motion_properties.?;
        const index_in_active_bodies = mp.index_in_active_bodies.load(.monotonic);
        if (index_in_active_bodies != last_body_index) {
            // This is not the last body, use the last body to fill the hole
            const last_body_id = active_bodies[last_body_index];
            active_bodies[index_in_active_bodies] = last_body_id;

            // Update that body's index in the active list
            const last_body = self.bodies[last_body_id.getIndex()].getBody();
            std.debug.assert(last_body.motion_properties.?.index_in_active_bodies.load(.monotonic) == last_body_index);
            last_body.motion_properties.?.index_in_active_bodies.store(index_in_active_bodies, .monotonic);
        }

        // Mark this body as no longer active
        mp.index_in_active_bodies.store(Body.inactive_index, .monotonic);

        // Remove unused element from active bodies list
        _ = num_active_bodies.fetchSub(1, .release);

        // Count CCD bodies
        if (mp.getMotionQuality() == .linear_cast)
            self.num_active_ccd_bodies -= 1;
    }

    /// Helper function to remove a body from the manager
    fn removeBodyInternal(self: *BodyManager, body_id: BodyID) *Body {
        // Get body
        const idx = body_id.getIndex();
        const body = self.bodies[idx].getBody();

        // Validate that it can be removed
        std.debug.assert(body.getID().eql(body_id));
        std.debug.assert(!body.isActive());
        std.debug.assert(!body.isInBroadPhase()); // Use BodyInterface::RemoveBody to remove this body first!

        // Push the id onto the freelist
        self.bodies[idx] = self.body_id_free_list_start;
        self.body_id_free_list_start = .fromFreedIndex(idx);

        return body;
    }

    /// Function to check that the free list is not corrupted (JPH_DEBUG && JPH_ENABLE_ASSERTS)
    fn validateFreeList(self: *const BodyManager) void {
        // Check that the freelist is correct
        var num_freed: usize = 0;
        var start = self.body_id_free_list_start;
        while (start != .free_list_end) : (start = self.bodies[start.freedIndex()]) {
            std.debug.assert((@intFromEnum(start) & is_freed_body) != 0);
            num_freed += 1;
        }
        std.debug.assert(self.num_bodies == self.bodies_size.raw - num_freed);
    }

    /// Helper function to delete a body (which could actually be a BodyWithMotionProperties) (sDeleteBody).
    /// Zolt: also runs Body's destructor (releases the shape and the collision group).
    fn deleteBody(self: *const BodyManager, body: *Body) void {
        if (body.motion_properties != null) {
            if (Core.enable_asserts) body.motion_properties = null;
            if (body.isSoftBody()) {
                body.shape.deinit(); // Release the shape to avoid assertion on shape destruction because of embedded object with refcount > 0
                const soft_body: *SoftBodyWithMotionPropertiesAndShape = @fieldParentPtr("body", body);
                soft_body.motion_properties.deinit(); // ~SoftBodyMotionProperties
                soft_body.shape.asShapeMut().deinit(); // ~SoftBodyShape: asserts that no reference to the embedded shape is left
                body.collision_group.deinit();
                self.allocator.destroy(soft_body);
            } else {
                body.shape.deinit();
                body.collision_group.deinit();
                const bmp: *BodyWithMotionProperties = @fieldParentPtr("body", body);
                // Zolt (Phase 13): back to its page, whatever BodyManager created the body (Jolt's static sDeleteBody)
                BodyPool.destroy(bmp);
            }
        } else {
            body.shape.deinit();
            body.collision_group.deinit();
            self.allocator.destroy(body);
        }
    }
};

test "BodyManager: the contact cache flags of an update that runs out of memory are put back (Zolt, D5)" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;
    const Quat = @import("../../Math/Quat.zig").Quat;
    const RVec3 = @import("../../Math/Real.zig").RVec3;
    const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
    const BroadPhaseLayer = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayer;
    const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;

    const LayerInterface = struct {
        pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };
        base: BroadPhaseLayerInterface = .init(@This()),
        pub fn getNumBroadPhaseLayers(_: *const @This()) u32 {
            return 1;
        }
        pub fn getBroadPhaseLayer(_: *const @This(), _: ObjectLayer) BroadPhaseLayer {
            return .init(0);
        }
    };
    const layers: LayerInterface = .{};
    var bm: BodyManager = .empty;
    try bm.init(allocator, std.testing.io, 4, 0, &layers.base);
    defer bm.deinit();

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, 0);
    defer settings.deinit();
    var bodies: [3]*Body = undefined;
    for (&bodies) |*b| {
        b.* = try bm.allocateBody(&settings);
        try expect(bm.addBody(b.*));
    }

    // Bodies 0 and 1 are flagged before the update
    try bm.invalidateContactCacheForBody(bodies[0]);
    try bm.invalidateContactCacheForBody(bodies[1]);
    bm.saveContactCacheInvalidation();

    // The update resets the flags (JobContactRemovedCallbacks), bodies 2 and 0 are flagged again (e.g. by a step
    // listener of the next collision step) and the update runs out of memory: the flags and the list are put back
    bm.validateContactCacheForAllBodies();
    try expect(!bodies[0].isCollisionCacheInvalid() and !bodies[1].isCollisionCacheInvalid());
    try bm.invalidateContactCacheForBody(bodies[2]);
    try bm.invalidateContactCacheForBody(bodies[0]);
    bm.restoreContactCacheInvalidation();
    try expect(bodies[0].isCollisionCacheInvalid() and bodies[1].isCollisionCacheInvalid() and !bodies[2].isCollisionCacheInvalid());
    try std.testing.expectEqualSlices(BodyID, &.{ bodies[0].getID(), bodies[1].getID() }, bm.bodies_cache_invalid.items);

    // The next update resets them like Jolt
    bm.saveContactCacheInvalidation();
    bm.validateContactCacheForAllBodies();
    for (bodies) |b|
        try expect(!b.isCollisionCacheInvalid());
    try std.testing.expectEqual(@as(usize, 0), bm.bodies_cache_invalid.items.len);
}

/// Shared settings that `SoftBodyMotionProperties.initialize` accepts: 3 vertices, a face, a rod, a skinned vertex and
/// one hand-made update group. Reference count 0: the creation settings hold it.
fn testSoftBodySettings(allocator: Allocator) Allocator.Error!*SoftBodySharedSettings {
    const s = try SoftBodySharedSettings.create(allocator);
    errdefer {
        s.deinit();
        allocator.destroy(s);
    }
    try s.vertices.append(allocator, .init(.{ .x = 1, .y = 0, .z = 0 }, .{}));
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = 1, .z = 0 }, .{ .inv_mass = 0.5 }));
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 1 }, .{ .inv_mass = 2.0 }));
    try s.addFace(.init(0, 1, 2, .{}));
    try s.rod_stretch_shear_constraints.append(allocator, .init(0, 1, .{}));
    try s.skinned_constraints.append(allocator, .init(2, 0.5, 0.25, 1.0));
    try s.update_groups.append(allocator, .{ .edge_end_index = 0, .lra_end_index = 0, .rod_stretch_shear_end_index = 1, .rod_bend_twist_end_index = 0, .dihedral_bend_end_index = 0, .volume_end_index = 0, .skinned_end_index = 1 });
    return s;
}

/// One broad phase layer for every object layer
const TestSoftBodyLayers = struct {
    const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
    const BroadPhaseLayer = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayer;
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };
    base: BroadPhaseLayerInterface = .init(@This()),
    pub fn getNumBroadPhaseLayers(_: *const @This()) u32 {
        return 1;
    }
    pub fn getBroadPhaseLayer(_: *const @This(), _: ObjectLayer) BroadPhaseLayer {
        return .init(0);
    }
};

test "BodyManager: allocateSoftBody is one allocation, deleteBody releases everything (R16)" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;
    const Quat = @import("../../Math/Quat.zig").Quat;
    const RVec3 = @import("../../Math/Real.zig").RVec3;
    const RefCount = @import("../../Core/Reference.zig").RefCount;
    const GroupFilterTable = @import("../Collision/GroupFilterTable.zig").GroupFilterTable;

    const layers: TestSoftBodyLayers = .{};
    var bm: BodyManager = .empty;
    try bm.init(allocator, std.testing.io, 4, 0, &layers.base);
    defer bm.deinit();

    const shared = try testSoftBodySettings(allocator);
    const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 2 });
    var settings: SoftBodyCreationSettings = .init(shared, RVec3.init(1, 2, 3), Quat.rotation(Vec3.axisY(), 0.5), 0);
    defer settings.deinit();
    settings.collision_group = .init(&filter.base, 1, 0);
    settings.user_data = 42;
    settings.friction = 0.75;
    settings.vertex_radius = 0.01;
    try expectEqual(@as(u32, 1), shared.getRefCount());
    try expectEqual(@as(u32, 1), filter.base.getRefCount());

    // The body, the motion properties and the shape are one object
    const body = try bm.allocateSoftBody(&settings);
    try expect(body.isSoftBody() and body.isDynamic());
    const soft_body: *SoftBodyWithMotionPropertiesAndShape = @fieldParentPtr("body", body);
    try expect(body.getMotionPropertiesUnchecked().? == &soft_body.motion_properties.base);
    try expect(body.getShape() == soft_body.shape.asShape());
    try expect(soft_body.shape.motion_properties == &soft_body.motion_properties);
    try expectEqual(@as(u32, RefCount.embedded + 1), body.getShape().getRefCount());
    try expectEqual(@as(u32, 2), shared.getRefCount()); // The motion properties hold the shared settings
    try expectEqual(@as(u32, 2), filter.base.getRefCount()); // The body holds the group filter
    try expectEqual(@as(u64, 42), body.getUserData());
    try expectEqual(@as(f32, 0.75), body.getFriction());
    try expect(body.getRotation().eql(Quat.identity())); // mMakeRotationIdentity: the rotation is in the vertices
    try expect(body.getPosition().eql(RVec3.init(1, 2, 3)));
    try expectEqual(@as(usize, 3), soft_body.motion_properties.getVertices().len);
    try expectEqual(@as(f32, 0.01), soft_body.motion_properties.getVertexRadius());

    // GetSoftBodyCreationSettings: the shared settings and the group filter of the body
    var back = body.getSoftBodyCreationSettings();
    try expect(back.settings.get() == shared and back.collision_group.group_filter.get() == &filter.base);
    try expectEqual(@as(u64, 42), back.user_data);
    back.deinit();

    // A second ApplySoftBodyCreationSettings resizes the arrays and leaks nothing
    const other = try testSoftBodySettings(allocator);
    try other.vertices.append(allocator, .init(.{ .x = 3, .y = 0, .z = 0 }, .{}));
    var other_settings: SoftBodyCreationSettings = .init(other, RVec3.zero(), Quat.identity(), 0);
    defer other_settings.deinit();
    try body.applySoftBodyCreationSettings(&other_settings, &layers.base);
    try expectEqual(@as(usize, 4), soft_body.motion_properties.getVertices().len);
    try expectEqual(@as(u32, 1), shared.getRefCount());
    try expectEqual(@as(u32, 2), other.getRefCount());
    try expectEqual(@as(u32, 1), filter.base.getRefCount()); // The other settings have no group filter

    // Destroy through the body manager: the references are released, nothing leaks (testing allocator)
    try expect(bm.addBody(body));
    bm.destroyBodies(&.{body.getID()});
    try expectEqual(@as(u32, 1), other.getRefCount());

    // freeBody of a soft body without an ID, and a soft body that is still alive when the manager is destroyed
    bm.freeBody(try bm.allocateSoftBody(&settings));
    try expectEqual(@as(u32, 1), shared.getRefCount());
    try expectEqual(@as(u32, 1), filter.base.getRefCount());
    try expect(bm.addBody(try bm.allocateSoftBody(&settings)));
    try expectEqual(@as(u32, 2), shared.getRefCount());
}

test "BodyManager: allocateSoftBody and deleteBody out of memory leak nothing" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            const Quat = @import("../../Math/Quat.zig").Quat;
            const RVec3 = @import("../../Math/Real.zig").RVec3;
            const layers: TestSoftBodyLayers = .{};
            var bm: BodyManager = .empty;
            try bm.init(allocator, std.testing.io, 2, 0, &layers.base);
            defer bm.deinit();
            var settings: SoftBodyCreationSettings = .init(try testSoftBodySettings(allocator), RVec3.zero(), Quat.identity(), 0);
            defer settings.deinit();
            const body = try bm.allocateSoftBody(&settings);
            if (!bm.addBody(body)) {
                bm.freeBody(body);
                return error.TestUnexpectedResult;
            }
            // A second ApplySoftBodyCreationSettings with the same sizes does not allocate (the body is not in the
            // broad phase); the soft body is destroyed with the manager
            try body.applySoftBodyCreationSettings(&settings, &layers.base);
            try std.testing.expectEqual(@as(usize, 3), SoftBodyMotionProperties.fromBaseConst(body.getMotionPropertiesUnchecked().?).getVertices().len);
        }
    }.f, .{});
}
