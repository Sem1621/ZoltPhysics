//! Port of: Jolt/Physics/Body/BodyManager.h, Jolt/Physics/Body/BodyManager.cpp
//! Status: partial
//! Missing: AllocateSoftBody (Phase 9, SimulationArchitecture D1: no soft body can be created before Phase 9)
//! Not ported: ReportSimulationStats (JPH_TRACK_SIMULATION_STATS + JPH_PROFILE_ENABLED, dropped with the profiler)
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
//!   out of memory leaves the bodies unchanged. `invalidateContactCacheForBody` returns `Allocator.Error` (the list of
//!   bodies to validate grows) and then clears the flag again, so the flag and the list stay consistent.
//! - Overloads: the const / non-const `GetBody`, `TryGetBody`, `GetBodies` are `getBody` / `getBodyMut`, `tryGetBody` /
//!   `tryGetBodyMut`, `getBodies` / `getBodiesMut`. Pointer + count parameters are slices.
//! - JPH_DEBUG_RENDERER (`Draw`, `DrawSettings`, `EShapeColor`, `ESoftBodyConstraintColor`): TODO(debug_renderer).

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
const StateRecorderFile = @import("../StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;
const PhysicsLockFile = @import("../PhysicsLock.zig");
const PhysicsLock = PhysicsLockFile.PhysicsLock;
const UniqueLock = PhysicsLockFile.UniqueLock;

/// Array of body ID's
pub const BodyIDVector = std.ArrayList(BodyID);

/// Helper class that combines a body and its motion properties
pub const BodyWithMotionProperties = struct {
    body: Body = .{},
    motion_properties: MotionProperties = .{},
};

/// Helper class that combines a soft body its motion properties and shape (the members are Phase 9 stubs)
pub const SoftBodyWithMotionPropertiesAndShape = struct {
    body: Body = .{},
    motion_properties: SoftBodyMotionProperties = .{},
    shape: SoftBodyShape = .{},
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
        @memset(self.body_sequence_numbers, 0);

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
        self.allocator.free(self.body_sequence_numbers);
        self.allocator.free(self.bodies);
        self.body_mutexes.deinit(self.allocator);
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

    /// Create a body using creation settings. The returned body will not be part of the body manager yet.
    /// Zolt: `error.InvalidShape` when the settings have no (valid) shape (Jolt crashes), the body is freed on error.
    pub fn allocateBody(self: *const BodyManager, body_creation_settings: *const BodyCreationSettings) CreateBodyError!*Body {
        // Fill in basic properties
        var body: *Body = undefined;
        if (body_creation_settings.hasMassProperties()) {
            const bmp = try self.allocator.create(BodyWithMotionProperties);
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
    /// Zolt: the lists of bodies to (de)activate are reserved before any body is restored, so out of memory changes no body.
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

                // Zolt: at most the number of bodies in the manager can change their activation (a valid ID per body)
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

    // TODO(debug_renderer): ESoftBodyConstraintColor, EShapeColor, DrawSettings, Draw

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
                body.collision_group.deinit();
                const soft_body: *SoftBodyWithMotionPropertiesAndShape = @fieldParentPtr("body", body);
                self.allocator.destroy(soft_body);
            } else {
                body.shape.deinit();
                body.collision_group.deinit();
                const bmp: *BodyWithMotionProperties = @fieldParentPtr("body", body);
                self.allocator.destroy(bmp);
            }
        } else {
            body.shape.deinit();
            body.collision_group.deinit();
            self.allocator.destroy(body);
        }
    }
};
