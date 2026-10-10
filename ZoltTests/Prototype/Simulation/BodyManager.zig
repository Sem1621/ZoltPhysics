//! Phase 5 architecture prototype: reduced Jolt/Physics/{PhysicsLock.h, Body/BodyManager.h/.cpp,
//! Body/BodyActivationListener.h, Body/BodyLockInterface.h, Body/BodyLock.h, Body/BodyLockMulti.h}
//! (Docs/Zolt/SimulationArchitecture.md D2, D3, D7).
//!
//! - Jolt's body array with the freed-slot encoding (`BodySlot`: a pointer or `(next << 1) | 1`), preallocated to
//!   max_bodies; `bodies_size` is Jolt's `mBodies.size()` as an atomic (release under the bodies mutex, acquire in the
//!   lock-free tryGetBody). IDs are allocated exactly like Jolt (LIFO free list, uint8 sequence numbers).
//! - Body mutexes: `MutexArray(SharedMutex)`; `io` is stored once in the BodyManager (a blocking type) and every lock
//!   user (BodyLockInterface, BodyInterface, broad phase, queries) reads `body_manager.io`.
//! - Rule M for C++ `mutable` mutexes: they live in the heap block `mutable_fields`, so Jolt's const functions keep
//!   `*const BodyManager` receivers (LockAllBodies, SaveState, GetActiveBodies, ...) without `@constCast`.
//! - PhysicsLock's debug lock order checking: a `threadlocal` table typed `void` when asserts are off.
//! - BodyLockInterface is a closed tagged struct (Jolt's two `final` classes); the RAII locks are init / deinit.

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const Mutex = zolt.Mutex;
const SharedMutex = zolt.SharedMutex;
const MutexArray = zolt.MutexArray;
const BodyID = zolt.BodyID;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const Allocator = std.mem.Allocator;
const BodyFile = @import("Body.zig");
const Body = BodyFile.Body;
const MotionProperties = BodyFile.MotionProperties;
const BodyType = BodyFile.BodyType;
const BodyCreationSettings = BodyFile.BodyCreationSettings;
const body_type_count = BodyFile.body_type_count;
const StateRecorderFile = @import("StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;

// -------------------------------------------------------------------------------------------------------------------
// PhysicsLock.h

/// This is the list of locks used by the physics engine, they need to be locked in a particular order (from top of the list to bottom of the list) in order to prevent deadlocks
pub const PhysicsLockTypes = enum(u32) {
    broad_phase_query = 1 << 0,
    per_body = 1 << 1,
    bodies_list = 1 << 2,
    broad_phase_update = 1 << 3,
    constraints_list = 1 << 4,
    active_bodies_list = 1 << 5,
};

/// A token that indicates the context of a lock (we use 1 per physics system and we use the body manager pointer because it's convenient)
pub const PhysicsLockContext = *const BodyManager;

/// Helpers to safely lock the different mutexes that are part of the physics system while preventing deadlock.
/// The lock order checks exist only when Core.enable_asserts (JPH_ENABLE_ASSERTS); the context and type parameters
/// are then unused and optimized away.
pub const PhysicsLock = struct {
    const LockData = struct {
        locked_mutexes: u32 = 0,
        context: ?PhysicsLockContext = null,
    };

    /// static thread_local LockData sLocks[4]: debug-only per-thread state, typed `void` without asserts so that it
    /// provably cannot influence release results (the one permitted kind of process state)
    threadlocal var locks: if (Core.enable_asserts) [4]LockData else void = if (Core.enable_asserts) @splat(.{}) else {};

    /// Call before taking the lock
    pub fn checkLock(context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        if (Core.enable_asserts) {
            const mutexes = getLockedMutexes(context);
            std.debug.assert(@intFromEnum(lock_type) > mutexes.*); // A lock of same or higher priority was already taken, this can create a deadlock!
            mutexes.* |= @intFromEnum(lock_type);
        }
    }

    /// Call after releasing the lock
    pub fn checkUnlock(context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        if (Core.enable_asserts) {
            const mutexes = getLockedMutexes(context);
            std.debug.assert((mutexes.* & @intFromEnum(lock_type)) != 0); // Mutex was not locked!
            mutexes.* &= ~@intFromEnum(lock_type);
        }
    }

    pub fn lock(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkLock(context, lock_type);
        mutex.lock(io);
    }

    pub fn unlock(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkUnlock(context, lock_type);
        mutex.unlock(io);
    }

    pub fn lockShared(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkLock(context, lock_type);
        mutex.lockShared(io);
    }

    pub fn unlockShared(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkUnlock(context, lock_type);
        mutex.unlockShared(io);
    }

    fn getLockedMutexes(context: PhysicsLockContext) *u32 {
        // If we find a matching context we can use it
        for (&locks) |*l| {
            if (l.context == context)
                return &l.locked_mutexes;
        }

        // Otherwise we look for an entry that is not in use
        for (&locks) |*l| {
            if (l.locked_mutexes == 0) {
                l.context = context;
                return &l.locked_mutexes;
            }
        }

        @panic("Too many physics systems locked at the same time!");
    }
};

/// Helper class that is similar to std::unique_lock: `var lock: UniqueLock(Mutex) = .init(...); defer lock.deinit();`
pub fn UniqueLock(comptime LockType: type) type {
    return struct {
        const Self = @This();
        mutex: *LockType,
        io: std.Io,
        context: PhysicsLockContext,
        lock_type: PhysicsLockTypes,

        pub fn init(mutex: *LockType, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) Self {
            PhysicsLock.lock(mutex, io, context, lock_type);
            return .{ .mutex = mutex, .io = io, .context = context, .lock_type = lock_type };
        }

        pub fn deinit(self: *Self) void {
            PhysicsLock.unlock(self.mutex, self.io, self.context, self.lock_type);
        }
    };
}

// -------------------------------------------------------------------------------------------------------------------
// BodyActivationListener.h

/// A listener class that receives events when a body activates or deactivates.
/// It can be registered with the BodyManager (or PhysicsSystem). Pattern B: both callbacks are pure virtual in Jolt.
/// The callbacks run from worker threads, with the active bodies mutex locked.
pub const BodyActivationListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        onBodyActivated: *const fn (ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void,
        onBodyDeactivated: *const fn (ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void,
    };

    pub fn init(impl: anytype) BodyActivationListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn onBodyActivatedThunk(ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onBodyActivated(body_id, body_user_data);
            }
            fn onBodyDeactivatedThunk(ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onBodyDeactivated(body_id, body_user_data);
            }
            const vtable: VTable = .{ .onBodyActivated = onBodyActivatedThunk, .onBodyDeactivated = onBodyDeactivatedThunk };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    pub fn onBodyActivated(self: BodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.vtable.onBodyActivated(self.ptr, body_id, body_user_data);
    }

    pub fn onBodyDeactivated(self: BodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.vtable.onBodyDeactivated(self.ptr, body_id, body_user_data);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// BodyManager.h / BodyManager.cpp

/// Helper class that combines a body and its motion properties (BodyWithMotionProperties : Body)
const BodyWithMotionProperties = struct {
    body: Body,
    motion_properties: MotionProperties,
};

/// Array of body ID's
pub const BodyIDVector = std.ArrayList(BodyID);

/// Errors of BodyManager.allocateBody / BodyInterface.createBody (Jolt crashes on a missing shape)
pub const CreateBodyError = error{
    /// The settings have no shape (Jolt dereferences null)
    InvalidShape,
} || Allocator.Error;

/// Class that contains all bodies
pub const BodyManager = struct {
    /// Bodies are protected using an array of mutexes (so a fixed number, not 1 per body). Each bit in this mask indicates a locked mutex.
    pub const MutexMask = u64;

    /// An array of mutexes protecting the bodies in the mBodies array
    const BodyMutexes = MutexArray(SharedMutex);

    /// Bit that indicates a pointer in mBodies is actually the index of the next freed body. We use the lowest bit because we know that Bodies need to be 16 byte aligned so addresses can never end in a 1 bit.
    const is_freed_body: usize = 1;

    /// Amount of bits to shift to get an index to the next freed body
    const freed_body_index_shift = 1;

    /// An entry of the body array: Jolt stores a `Body *` that is either a body or a freed slot
    /// `(next_free << cFreedBodyIndexShift) | cIsFreedBody`. Zig pointers can't hold odd values (and @ptrFromInt checks
    /// alignment), so the entry is an integer with typed accessors (BodyVector = Array<Body *>).
    pub const BodySlot = enum(usize) {
        /// Indicates that there are no more freed body IDs (cBodyIDFreeListEnd)
        free_list_end = ~@as(usize, 0),
        _,

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
    };

    /// The C++ `mutable` members that const functions lock (Rule M: behind a pointer, never through @constCast)
    const MutableFields = struct {
        /// Protects mBodies array (but not the bodies it points to), mNumBodies and mBodyIDFreeListStart
        bodies_mutex: Mutex = .{},
        /// Mutex that protects the mActiveBodies array
        active_bodies_mutex: Mutex = .{},
    };

    allocator: Allocator = undefined,
    io: std.Io = undefined,

    /// List of pointers to all bodies. Contains invalid pointers for deleted bodies, check with BodySlot.isValidBodyPointer.
    /// Preallocated to max bodies: `bodies.len` is GetMaxBodies(), `bodies_size` is Jolt's mBodies.size().
    bodies: []BodySlot = &.{},

    /// Jolt's mBodies.size(): written under the bodies mutex, read lock-free by tryGetBody
    bodies_size: std.atomic.Value(u32) = .init(0),

    /// Current number of allocated bodies
    num_bodies: u32 = 0,

    /// Index of first entry in mBodies that is unused
    body_id_free_list_start: BodySlot = .free_list_end,

    mutable_fields: *MutableFields = undefined,

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

    /// Listener that is notified whenever a body is activated/deactivated
    activation_listener: ?BodyActivationListener = null,

    /// The broadphase layer interface that maps object layers to broadphase layers
    broad_phase_layer_interface: *const BroadPhaseLayerInterface = undefined,

    /// Debug system that tries to limit changes to active bodies during the PhysicsSystem::Update() (JPH_ENABLE_ASSERTS)
    active_bodies_locked: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {},

    /// Not initialized (default constructor)
    pub const empty: BodyManager = .{};

    /// Initialize the manager (Init). In place: the BodyLockInterfaces of the PhysicsSystem point to it.
    pub fn init(self: *BodyManager, allocator: Allocator, io: std.Io, max_bodies: u32, num_body_mutexes_in: u32, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        self.* = .{ .allocator = allocator, .io = io, .broad_phase_layer_interface = layer_interface };

        // Num body mutexes must be a power of two and not bigger than our MutexMask
        const requested = if (num_body_mutexes_in == 0) 2 * @as(u32, @intCast(std.Thread.getCpuCount() catch 1)) else num_body_mutexes_in;
        const num_body_mutexes = std.math.clamp(std.math.ceilPowerOfTwoAssert(u32, requested), 1, @bitSizeOf(MutexMask));

        self.mutable_fields = try allocator.create(MutableFields);
        errdefer allocator.destroy(self.mutable_fields);
        self.mutable_fields.* = .{};

        // Allocate the body mutexes
        self.body_mutexes = try .init(allocator, num_body_mutexes);
        errdefer self.body_mutexes.deinit(allocator);

        // Allocate space for bodies
        self.bodies = try allocator.alloc(BodySlot, max_bodies);
        errdefer allocator.free(self.bodies);

        // Allocate space for active bodies
        var num_allocated: usize = 0;
        errdefer for (self.active_bodies[0..num_allocated]) |a| allocator.free(a);
        for (&self.active_bodies) |*active_bodies| {
            active_bodies.* = try allocator.alloc(BodyID, max_bodies);
            num_allocated += 1;
        }

        // Allocate space for sequence numbers
        self.body_sequence_numbers = try allocator.alloc(u8, max_bodies);
        @memset(self.body_sequence_numbers, 0);
    }

    /// Destructor
    pub fn deinit(self: *BodyManager) void {
        if (self.bodies.len == 0) return; // Not initialized
        {
            var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
            defer lock.deinit();

            // Destroy any bodies that are still alive
            for (self.bodies[0..self.bodies_size.raw]) |b| {
                if (b.isValidBodyPointer())
                    self.deleteBody(b.getBody());
            }
        }

        for (self.active_bodies) |active_bodies| self.allocator.free(active_bodies);
        self.allocator.free(self.body_sequence_numbers);
        self.allocator.free(self.bodies);
        self.body_mutexes.deinit(self.allocator);
        self.allocator.destroy(self.mutable_fields);
        self.* = .empty;
    }

    /// Gets the current amount of bodies that are in the body manager
    pub fn getNumBodies(self: *const BodyManager) u32 {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();
        return self.num_bodies;
    }

    /// Gets the max bodies that we can support
    pub fn getMaxBodies(self: *const BodyManager) u32 {
        return @intCast(self.bodies.len);
    }

    /// Create a body using creation settings. The returned body will not be part of the body manager yet.
    /// Zolt: the shape is resolved first, so a failure leaves nothing to undo (Jolt crashes on a missing shape).
    pub fn allocateBody(self: *const BodyManager, settings: *const BodyCreationSettings) CreateBodyError!*Body {
        const shape = (try settings.getShape(self.allocator)) orelse return error.InvalidShape;

        // Fill in basic properties
        var body: *Body = undefined;
        if (settings.hasMassProperties()) {
            const bmp = try self.allocator.create(BodyWithMotionProperties);
            bmp.* = .{ .body = .{ .position = undefined, .rotation = undefined }, .motion_properties = .{} };
            body = &bmp.body;
            body.motion_properties = &bmp.motion_properties;
        } else {
            body = try self.allocator.create(Body);
            body.* = .{ .position = undefined, .rotation = undefined };
        }
        body.body_type = .rigid_body;
        body.shape = .init(shape);
        body.user_data = settings.user_data;
        body.collision_group = settings.collision_group.clone();
        body.motion_type = settings.motion_type;
        if (settings.is_sensor) _ = body.flags.fetchOr(Body.Flags.is_sensor, .monotonic);
        body.object_layer = settings.object_layer;
        body.broad_phase_layer = self.broad_phase_layer_interface.getBroadPhaseLayer(settings.object_layer);
        body.friction = settings.friction;
        body.restitution = settings.restitution;

        // Initialize position (reduced: the prototype's shapes have their center of mass at the origin)
        body.position = settings.position;
        body.rotation = settings.rotation.normalized();

        if (settings.hasMassProperties()) {
            const mp = body.motion_properties.?;
            mp.linear_velocity = settings.linear_velocity;
            mp.angular_velocity = settings.angular_velocity;
            mp.inv_mass = 1.0 / settings.getMassProperties(shape).mass;
            mp.linear_damping = settings.linear_damping;
            mp.angular_damping = settings.angular_damping;
            mp.max_linear_velocity = settings.max_linear_velocity;
            mp.max_angular_velocity = settings.max_angular_velocity;
            mp.gravity_factor = settings.gravity_factor;
            mp.motion_quality = settings.motion_quality;
            mp.allowed_dofs = settings.allowed_dofs;
            mp.allow_sleeping = settings.allow_sleeping;
            if (Core.enable_asserts) mp.cached_motion_type = body.motion_type;
        }

        // Position body
        body.calculateWorldSpaceBoundsInternal();
        return body;
    }

    /// Free a body that has not been added to the body manager yet (if it has, use DestroyBodies).
    pub fn freeBody(self: *const BodyManager, body: *Body) void {
        std.debug.assert(body.getID().isInvalid()); // This function should only be called on a body that doesn't have an ID yet, use DestroyBody otherwise
        self.deleteBody(body);
    }

    /// Helper function to delete a body (which could actually be a BodyWithMotionProperties) (sDeleteBody)
    fn deleteBody(self: *const BodyManager, body: *Body) void {
        body.shape.deinit();
        body.collision_group.deinit();
        if (body.motion_properties != null) {
            std.debug.assert(body.isRigidBody()); // Phase 9: soft bodies are a SoftBodyWithMotionPropertiesAndShape
            body.motion_properties = null;
            const bmp: *BodyWithMotionProperties = @fieldParentPtr("body", body);
            self.allocator.destroy(bmp);
        } else {
            self.allocator.destroy(body);
        }
    }

    /// Increment and get the sequence number of the body (we intentionally overflow the uint8 sequence number)
    fn getNextSequenceNumber(self: *BodyManager, body_index: u32) u8 {
        self.body_sequence_numbers[body_index] +%= 1;
        return self.body_sequence_numbers[body_index];
    }

    /// Add a body to the body manager, assigning it the next available ID. Returns false if no more IDs are available.
    pub fn addBody(self: *BodyManager, body: *Body) bool {
        // Return error when body was already added
        if (!body.getID().isInvalid())
            return false;

        // Determine next free index
        var idx: u32 = undefined;
        {
            var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
            defer lock.deinit();

            if (self.body_id_free_list_start != .free_list_end) {
                // Pop an item from the freelist
                std.debug.assert(@intFromEnum(self.body_id_free_list_start) & is_freed_body != 0);
                idx = @intCast(@intFromEnum(self.body_id_free_list_start) >> freed_body_index_shift);
                std.debug.assert(!self.bodies[idx].isValidBodyPointer());
                self.body_id_free_list_start = self.bodies[idx];
                self.bodies[idx] = .fromBody(body);
            } else {
                const size = self.bodies_size.raw;
                if (size < self.bodies.len) {
                    // Allocate a new entry, note that the array should not actually resize since we've reserved it at init time
                    idx = size;
                    self.bodies[idx] = .fromBody(body);
                    self.bodies_size.store(size + 1, .release); // After the slot: tryGetBody reads without lock
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
            var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
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
                const idx_start = @intFromEnum(self.body_id_free_list_start) >> freed_body_index_shift;
                if (idx == idx_start) {
                    // First entry, easy to remove, the start of the list is our next
                    self.body_id_free_list_start = self.bodies[idx];
                } else {
                    // Loop over the freelist and find the entry in the freelist pointing to our index
                    // TODO: This is O(N), see if this becomes a performance problem (don't want to put the freed bodies in a double linked list)
                    var cur = idx_start;
                    while (cur != @intFromEnum(BodySlot.free_list_end) >> freed_body_index_shift) {
                        const next = @intFromEnum(self.bodies[cur]) >> freed_body_index_shift;
                        if (next == idx) {
                            self.bodies[cur] = self.bodies[idx];
                            break;
                        }
                        cur = next;
                    }
                    std.debug.assert(cur != @intFromEnum(BodySlot.free_list_end) >> freed_body_index_shift);
                }

                // Put the body in the slot
                self.bodies[idx] = .fromBody(body);
            } else {
                // Ensure that all body IDs up to this body ID have been allocated and added to the free list
                var new_size = size;
                while (idx > new_size) {
                    // Push the id onto the freelist
                    self.bodies[new_size] = self.body_id_free_list_start;
                    self.body_id_free_list_start = @enumFromInt((@as(usize, new_size) << freed_body_index_shift) | is_freed_body);
                    new_size += 1;
                }

                // Add the element to the list
                self.bodies[idx] = .fromBody(body);
                self.bodies_size.store(idx + 1, .release);
            }

            // Update cached number of bodies
            self.num_bodies += 1;

            // A custom ID establishes the current sequence number for this slot.
            self.body_sequence_numbers[idx] = body_id.getSequenceNumber();
        }

        // Assign the ID
        body.id = body_id;
        return true;
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
        self.body_id_free_list_start = @enumFromInt((@as(usize, idx) << freed_body_index_shift) | is_freed_body);

        return body;
    }

    /// Remove a set of bodies from the body manager and frees them.
    pub fn destroyBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be destroyed
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
        defer lock.deinit();

        // Update cached number of bodies
        std.debug.assert(self.num_bodies >= body_ids.len);
        self.num_bodies -= @intCast(body_ids.len);

        for (body_ids) |b| {
            // Remove body
            const body = self.removeBodyInternal(b);

            // Clear the ID
            body.id = .invalid;

            // Free the body
            self.deleteBody(body);
        }
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
        const index_in_active_bodies = mp.getIndexInActiveBodiesInternal();
        if (index_in_active_bodies != last_body_index) {
            // This is not the last body, use the last body to fill the hole
            const last_body_id = active_bodies[last_body_index];
            active_bodies[index_in_active_bodies] = last_body_id;

            // Update that body's index in the active list
            const last_body = self.bodies[last_body_id.getIndex()].getBody();
            std.debug.assert(last_body.motion_properties.?.getIndexInActiveBodiesInternal() == last_body_index);
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

    /// Activate a list of bodies. This function should only be called when an exclusive lock for the bodies are held.
    pub fn activateBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be activated
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        for (body_ids) |body_id| {
            if (!body_id.isInvalid()) {
                const body = self.bodies[body_id.getIndex()].getBody();

                std.debug.assert(body.getID().eql(body_id));
                std.debug.assert(body.isInBroadPhase()); // Use BodyInterface::AddBody to add the body first!

                if (!body.isStatic()) {
                    // Reset sleeping timer so that we don't immediately go to sleep again
                    body.resetSleepTimer();

                    // Check if we're sleeping
                    if (body.motion_properties.?.getIndexInActiveBodiesInternal() == Body.inactive_index) {
                        self.addBodyToActiveBodies(body);

                        // Call activation listener
                        if (self.activation_listener) |listener|
                            listener.onBodyActivated(body_id, body.getUserData());
                    }
                }
            }
        }
    }

    /// Deactivate a list of bodies. This function should only be called when an exclusive lock for the bodies are held.
    pub fn deactivateBodies(self: *BodyManager, body_ids: []const BodyID) void {
        // Don't take lock if no bodies are to be deactivated
        if (body_ids.len == 0)
            return;

        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        for (body_ids) |body_id| {
            if (!body_id.isInvalid()) {
                const body = self.bodies[body_id.getIndex()].getBody();

                std.debug.assert(body.getID().eql(body_id));
                std.debug.assert(body.isInBroadPhase()); // Use BodyInterface::AddBody to add the body first!

                if (body.motion_properties != null and body.motion_properties.?.getIndexInActiveBodiesInternal() != Body.inactive_index) {
                    // Remove the body from the active bodies list
                    self.removeBodyFromActiveBodies(body);

                    // Mark this body as no longer active
                    body.motion_properties.?.island_index = Body.inactive_index;

                    // Reset velocity
                    body.motion_properties.?.linear_velocity = .zero();
                    body.motion_properties.?.angular_velocity = .zero();

                    // Call activation listener
                    if (self.activation_listener) |listener|
                        listener.onBodyDeactivated(body_id, body.getUserData());
                }
            }
        }
    }

    /// Get copy of the list of active bodies under protection of a lock (GetActiveBodies, const: Rule M via mutable_fields)
    pub fn getActiveBodies(self: *const BodyManager, body_type: BodyType, out_body_ids: *BodyIDVector) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();

        const active_bodies = self.active_bodies[@intFromEnum(body_type)];
        out_body_ids.clearRetainingCapacity();
        try out_body_ids.appendSlice(self.allocator, active_bodies[0..self.num_active_bodies[@intFromEnum(body_type)].load(.monotonic)]);
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
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.active_bodies_mutex, self.io, self, .active_bodies_list);
        defer lock.deinit();
        self.activation_listener = listener;
    }

    /// Get all bodies. Note that this can contain invalid body pointers, call BodySlot.isValidBodyPointer to check
    pub fn getBodies(self: *const BodyManager) []const BodySlot {
        return self.bodies[0..self.bodies_size.load(.acquire)];
    }

    /// Access a body (not protected by lock)
    pub fn getBody(self: *const BodyManager, id: BodyID) *Body {
        return self.bodies[id.getIndex()].getBody();
    }

    /// Access a body, will return null if the body ID is no longer valid (not protected by lock)
    pub fn tryGetBody(self: *const BodyManager, id: BodyID) ?*Body {
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
        var mask = mutex_mask;
        var index: u32 = 0;
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
        var mask = mutex_mask;
        var index: u32 = 0;
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
        var mask = mutex_mask;
        var index: u32 = 0;
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
        var mask = mutex_mask;
        var index: u32 = 0;
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

        PhysicsLock.lock(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);
    }

    /// Unlock all bodies. This should only be done during PhysicsSystem::Update().
    pub fn unlockAllBodies(self: *const BodyManager) void {
        PhysicsLock.unlock(&self.mutable_fields.bodies_mutex, self.io, self, .bodies_list);

        PhysicsLock.checkUnlock(self, .per_body);
        self.body_mutexes.unlockAll(self.io);
    }

    /// Debug: lock the active bodies list during the update (SetActiveBodiesLocked, JPH_ENABLE_ASSERTS only)
    pub fn setActiveBodiesLocked(self: *BodyManager, locked: bool) void {
        if (Core.enable_asserts) self.active_bodies_locked = locked;
    }

    /// Saving state for replay (allocates the list of bodies to save: Jolt's Array, so Allocator.Error)
    pub fn saveState(self: *const BodyManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        self.lockAllBodies();
        defer self.unlockAllBodies();

        // Determine which bodies to save
        var bodies: std.ArrayList(*const Body) = .empty;
        defer bodies.deinit(self.allocator);
        try bodies.ensureTotalCapacity(self.allocator, self.num_bodies);
        for (self.getBodies()) |b| {
            if (b.isValidBodyPointer() and b.getBody().isInBroadPhase() and (filter == null or filter.?.shouldSaveBody(b.getBody())))
                bodies.appendAssumeCapacity(b.getBody());
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
                for (self.getBodies()) |b| {
                    if (b.isValidBodyPointer() and b.getBody().isInBroadPhase())
                        old_num_bodies += 1;
                }
                var num_bodies = old_num_bodies; // Initialize to current value for validation
                stream.read(&num_bodies);
                if (num_bodies != old_num_bodies) {
                    if (Core.enable_asserts) @panic("Cannot handle adding/removing bodies");
                    return false;
                }

                for (self.getBodies()) |slot| {
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
                                try bodies_to_activate.append(self.allocator, body_id)
                            else
                                try bodies_to_deactivate.append(self.allocator, body_id);
                        }
                        b.restoreState(stream);
                    }
                }
            } else {
                // Not validating, we can be a bit more loose, read number of bodies
                var num_bodies: u32 = 0;
                stream.read(&num_bodies);

                // Iterate over the stored bodies and restore their state
                for (0..num_bodies) |_| {
                    var body_id: BodyID = .invalid;
                    stream.read(&body_id);
                    const b = self.tryGetBody(body_id) orelse {
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
            var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.active_bodies_mutex, self.io, self, .active_bodies_list);
            defer lock.deinit();

            for (bodies_to_activate.items) |body_id|
                self.addBodyToActiveBodies(self.tryGetBody(body_id).?);

            for (bodies_to_deactivate.items) |body_id|
                self.removeBodyFromActiveBodies(self.tryGetBody(body_id).?);
        }

        return !stream.isFailed();
    }
};

// -------------------------------------------------------------------------------------------------------------------
// BodyLockInterface.h: Jolt's base class has exactly two `final` implementations, nothing else derives from it. A closed
// set becomes a tagged struct with an exhaustive switch (no vtable); both C++ class bodies stay side by side.

/// Base class interface for locking a body. Usually you will use BodyLockRead / BodyLockWrite / BodyLockMultiRead / BodyLockMultiWrite instead.
pub const BodyLockInterface = struct {
    /// Redefine MutexMask
    pub const MutexMask = BodyManager.MutexMask;

    /// The C++ class: BodyLockInterfaceNoLock or BodyLockInterfaceLocking
    pub const Kind = enum {
        /// Implementation that performs no locking (assumes the lock has already been taken)
        no_lock,
        /// Implementation that uses the body manager to lock the correct mutex for a body
        locking,
    };

    /// Jolt: BodyManager &mBodyManager (a non-const reference in const functions: state behind a pointer, Rule M)
    body_manager: *BodyManager,
    kind: Kind,

    /// BodyLockInterfaceNoLock(BodyManager &)
    pub fn initNoLock(body_manager: *BodyManager) BodyLockInterface {
        return .{ .body_manager = body_manager, .kind = .no_lock };
    }

    /// BodyLockInterfaceLocking(BodyManager &)
    pub fn initLocking(body_manager: *BodyManager) BodyLockInterface {
        return .{ .body_manager = body_manager, .kind = .locking };
    }

    /// Overloads get suffixes: LockRead(BodyID) / LockRead(MutexMask) -> lockRead / lockReadMask
    pub fn lockRead(self: *const BodyLockInterface, body_id: BodyID) ?*SharedMutex {
        switch (self.kind) {
            .no_lock => return null,
            .locking => {
                const bm = self.body_manager;
                const mutex = bm.getMutexForBody(body_id);
                PhysicsLock.lockShared(mutex, bm.io, bm, .per_body);
                return mutex;
            },
        }
    }

    pub fn unlockRead(self: *const BodyLockInterface, mutex: ?*SharedMutex) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => {
                const bm = self.body_manager;
                PhysicsLock.unlockShared(mutex.?, bm.io, bm, .per_body);
            },
        }
    }

    pub fn lockWrite(self: *const BodyLockInterface, body_id: BodyID) ?*SharedMutex {
        switch (self.kind) {
            .no_lock => return null,
            .locking => {
                const bm = self.body_manager;
                const mutex = bm.getMutexForBody(body_id);
                PhysicsLock.lock(mutex, bm.io, bm, .per_body);
                return mutex;
            },
        }
    }

    pub fn unlockWrite(self: *const BodyLockInterface, mutex: ?*SharedMutex) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => {
                const bm = self.body_manager;
                PhysicsLock.unlock(mutex.?, bm.io, bm, .per_body);
            },
        }
    }

    pub fn getMutexMask(self: *const BodyLockInterface, bodies: []const BodyID) MutexMask {
        return switch (self.kind) {
            .no_lock => 0,
            .locking => self.body_manager.getMutexMask(bodies),
        };
    }

    pub fn lockReadMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => self.body_manager.lockRead(mutex_mask),
        }
    }

    pub fn unlockReadMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => self.body_manager.unlockRead(mutex_mask),
        }
    }

    pub fn lockWriteMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => self.body_manager.lockWrite(mutex_mask),
        }
    }

    pub fn unlockWriteMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {},
            .locking => self.body_manager.unlockWrite(mutex_mask),
        }
    }

    /// Get the mask needed to lock all bodies
    pub fn getAllBodiesMutexMask(self: *const BodyLockInterface) MutexMask {
        return self.body_manager.getAllBodiesMutexMask();
    }

    /// Convert body ID to body
    pub fn tryGetBody(self: *const BodyLockInterface, body_id: BodyID) ?*Body {
        return self.body_manager.tryGetBody(body_id);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// BodyLock.h / BodyLockMulti.h (RAII -> init + `defer lock.deinit()`)

/// Base class for locking bodies for the duration of the scope of this class (do not use directly).
/// `write` selects Jolt's template argument: BodyLockRead hands out `*const Body`, BodyLockWrite `*Body`.
pub fn BodyLockBase(comptime write: bool) type {
    return struct {
        const Self = @This();
        pub const BodyPtr = if (write) *Body else *const Body;

        body_lock_interface: *const BodyLockInterface,
        body_lock_mutex: ?*SharedMutex,
        body: ?BodyPtr,

        /// Constructor will lock the body
        pub fn init(body_lock_interface: *const BodyLockInterface, body_id: BodyID) Self {
            if (body_id.eql(.invalid)) {
                // Invalid body id
                return .{ .body_lock_interface = body_lock_interface, .body_lock_mutex = null, .body = null };
            } else {
                // Get mutex
                const mutex = if (write) body_lock_interface.lockWrite(body_id) else body_lock_interface.lockRead(body_id);

                // Get a reference to the body or nullptr when it is no longer valid
                return .{ .body_lock_interface = body_lock_interface, .body_lock_mutex = mutex, .body = body_lock_interface.tryGetBody(body_id) };
            }
        }

        /// Explicitly release the lock (normally this is done in the destructor)
        pub fn releaseLock(self: *Self) void {
            if (self.body_lock_mutex != null) {
                if (write)
                    self.body_lock_interface.unlockWrite(self.body_lock_mutex)
                else
                    self.body_lock_interface.unlockRead(self.body_lock_mutex);
                self.body_lock_mutex = null;
                self.body = null;
            }
        }

        /// Destructor will unlock the body
        pub fn deinit(self: *Self) void {
            self.releaseLock();
        }

        /// Test if the lock was successful (if the body ID was valid)
        pub fn succeeded(self: *const Self) bool {
            return self.body != null;
        }

        /// Test if the lock was successful (if the body ID was valid) and the body is still in the broad phase
        pub fn succeededAndIsInBroadPhase(self: *const Self) bool {
            return self.body != null and self.body.?.isInBroadPhase();
        }

        /// Access the body
        pub fn getBody(self: *const Self) BodyPtr {
            std.debug.assert(self.body != null); // Should check Succeeded() first
            return self.body.?;
        }
    };
}

/// A body lock takes a body ID and locks the underlying body so that other threads cannot access its members
pub const BodyLockRead = BodyLockBase(false);

/// Specialization that locks a body for writing to. @see BodyLockRead for usage patterns.
pub const BodyLockWrite = BodyLockBase(true);

/// Base class for locking multiple bodies for the duration of the scope of this class (do not use directly)
pub fn BodyLockMultiBase(comptime write: bool) type {
    return struct {
        const Self = @This();
        pub const BodyPtr = if (write) *Body else *const Body;

        body_lock_interface: *const BodyLockInterface,
        mutex_mask: BodyLockInterface.MutexMask,
        body_ids: []const BodyID,

        /// Constructor will lock the bodies
        pub fn init(body_lock_interface: *const BodyLockInterface, body_ids: []const BodyID) Self {
            const self: Self = .{ .body_lock_interface = body_lock_interface, .mutex_mask = body_lock_interface.getMutexMask(body_ids), .body_ids = body_ids };
            if (self.mutex_mask != 0) {
                // Get mutex
                if (write)
                    body_lock_interface.lockWriteMask(self.mutex_mask)
                else
                    body_lock_interface.lockReadMask(self.mutex_mask);
            }
            return self;
        }

        /// Explicitly release the locks on all bodies (normally this is done in the destructor)
        pub fn releaseLocks(self: *Self) void {
            if (self.mutex_mask != 0) {
                if (write)
                    self.body_lock_interface.unlockWriteMask(self.mutex_mask)
                else
                    self.body_lock_interface.unlockReadMask(self.mutex_mask);
                self.mutex_mask = 0;
                self.body_ids = &.{};
            }
        }

        /// Destructor will unlock the bodies
        pub fn deinit(self: *Self) void {
            self.releaseLocks();
        }

        /// Returns the number of bodies that were locked
        pub fn getNumBodies(self: *const Self) usize {
            return self.body_ids.len;
        }

        /// Access the body (returns null if body was not properly locked)
        pub fn getBody(self: *const Self, body_index: usize) ?BodyPtr {
            // Get body ID
            const body_id = self.body_ids[body_index];
            if (body_id.isInvalid())
                return null;

            // Get a reference to the body or nullptr when it is no longer valid
            return self.body_lock_interface.tryGetBody(body_id);
        }
    };
}

/// A multi body lock takes a number of body IDs and locks the underlying bodies so that other threads cannot access its members
pub const BodyLockMultiRead = BodyLockMultiBase(false);

/// Specialization that locks multiple bodies for writing to. @see BodyLockMultiRead for usage patterns.
pub const BodyLockMultiWrite = BodyLockMultiBase(true);
