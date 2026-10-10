//! Phase 5 architecture prototype: reduced Jolt/Physics/{PhysicsSystem.h/.cpp, PhysicsUpdateContext.h,
//! PhysicsStepListener.h, EPhysicsUpdateError.h, Body/BodyInterface.h/.cpp} plus a stand-in for the
//! ContactConstraintManager (Docs/Zolt/SimulationArchitecture.md D4, D5, D6, D7).
//!
//! What is real (the decided architecture):
//! - `update` builds Jolt's complete job graph per collision step with Jolt's dependency counts, creation order and kick
//!   order. Every C++ lambda is an inline `struct { fn call(...) }.call` closure at the lambda's position whose captures
//!   are the lambda's captures. Dynamic jobs (TrySpawnJobFindCollisions, FindCCDContacts) are created inside jobs and
//!   added to the barrier, as in Jolt.
//! - The start gate (D4): the first step's root jobs get one extra dependency that `kickStep` (the code StartNextStep
//!   runs for the later steps) removes after the whole graph exists, so building the graph is failure atomic.
//! - The out of memory latch (D5): a job that is poisoned skips its work and its allocations but always does its frees,
//!   its commits (UpdateFinalize) and its dependency bookkeeping. A failed spawn inside a job is not an error.
//! - Temp memory is TempArray (empty when not allocated), freed at Jolt's positions, so LIFO holds by construction.
//! - IslandBuilder is a near-complete port; job work distribution (atomic batch counters, body pair queues with work
//!   stealing, the active find collision job mask, island claims) is Jolt's.
//!
//! Reduced: one closest hit per body pair (no manifolds, no contact cache, no warm starting of contacts), contact
//! solving is a single normal impulse, CCD casts against the plane y = 0, no sleeping, no large island splitter.

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const math = zolt.math;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const BodyID = zolt.BodyID;
const Color = zolt.Color;
const Mutex = zolt.Mutex;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const StaticArray = zolt.StaticArray;
const TempAllocator = zolt.TempAllocator;
const JobSystem = zolt.JobSystem;
const JobHandle = zolt.JobHandle;
const PhysicsSettings = zolt.PhysicsSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideShapeCollector = zolt.CollideShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const HashCombine = zolt.HashCombine;
const Allocator = std.mem.Allocator;

const BodyFile = @import("Body.zig");
const Body = BodyFile.Body;
const BodyCreationSettings = BodyFile.BodyCreationSettings;
const Activation = BodyFile.Activation;
const BodyManagerFile = @import("BodyManager.zig");
const BodyManager = BodyManagerFile.BodyManager;
const BodyIDVector = BodyManagerFile.BodyIDVector;
const CreateBodyError = BodyManagerFile.CreateBodyError;
const BodyActivationListener = BodyManagerFile.BodyActivationListener;
const BodyLockInterface = BodyManagerFile.BodyLockInterface;
const BodyLockRead = BodyManagerFile.BodyLockRead;
const BodyLockWrite = BodyManagerFile.BodyLockWrite;
const BodyLockMultiWrite = BodyManagerFile.BodyLockMultiWrite;
const BroadPhaseFile = @import("BroadPhase.zig");
const BroadPhase = BroadPhaseFile.BroadPhase;
const PrototypeBroadPhase = BroadPhaseFile.PrototypeBroadPhase;
const BodyPair = BroadPhaseFile.BodyPair;
const BodyPairCollector = BroadPhaseFile.BodyPairCollector;
const ConstraintFile = @import("Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const TwoBodyConstraint = ConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = ConstraintFile.TwoBodyConstraintSettings;
const ConstraintManager = ConstraintFile.ConstraintManager;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const StateRecorderFile = @import("StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderState = StateRecorderFile.StateRecorderState;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;
const TempArray = @import("TempArray.zig").TempArray;
const DeterminismLog = @import("DeterminismLog.zig");

// -------------------------------------------------------------------------------------------------------------------
// EPhysicsUpdateError.h

/// Enum used by PhysicsSystem to report error conditions during the PhysicsSystem::Update call. This is a bit field, multiple errors can trigger in the same update.
/// Jolt's soft errors stay values (the simulation continued); allocation failure is the Zig error of `update` (D5).
pub const PhysicsUpdateError = packed struct(u32) {
    /// The manifold cache is full, this means that the total number of contacts between bodies is too high. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
    manifold_cache_full: bool = false,
    /// The body pair cache is full, this means that too many bodies contacted. Some contacts were ignored. Increase inMaxBodyPairs in PhysicsSystem::Init.
    body_pair_cache_full: bool = false,
    /// The contact constraints buffer is full. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
    contact_constraints_full: bool = false,
    _padding: u29 = 0,

    /// No errors
    pub const none: PhysicsUpdateError = .{};

    pub fn isNone(self: PhysicsUpdateError) bool {
        return @as(u32, @bitCast(self)) == 0;
    }
};

// -------------------------------------------------------------------------------------------------------------------
// Listeners (pattern B, called from worker threads)

/// A contact listener (prototype subset of Zolt/Physics/Collision/ContactListener.zig, which takes the Phase 5 Body)
pub const ContactListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Called whenever a new contact point is detected. Null: Jolt's default (empty) implementation.
        onContactAdded: ?*const fn (ptr: *anyopaque, body1: *const Body, body2: *const Body) void = null,
    };

    pub fn init(impl: anytype) ContactListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn onContactAddedThunk(ptr: *anyopaque, body1: *const Body, body2: *const Body) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onContactAdded(body1, body2);
            }
            const vtable: VTable = .{ .onContactAdded = if (@hasDecl(T, "onContactAdded")) onContactAddedThunk else null };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    pub fn onContactAdded(self: ContactListener, body1: *const Body, body2: *const Body) void {
        if (self.vtable.onContactAdded) |f| f(self.ptr, body1, body2);
    }
};

/// Context information for the step listener
pub const PhysicsStepListenerContext = struct {
    /// Delta time of the current step
    delta_time: f32,
    /// True if this is the first step
    is_first_step: bool,
    /// True if this is the last step
    is_last_step: bool,
    /// The physics system that is being stepped
    physics_system: *PhysicsSystem,
};

/// A listener class that receives a callback before every physics simulation step. Pattern B (OnStep is pure virtual).
pub const PhysicsStepListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Called before every simulation step (received inCollisionSteps times for every PhysicsSystem::Update(...) call)
        /// This is called while all body and constraint mutexes are locked. You can read/write bodies and constraints but not add/remove them.
        /// Multiple listeners can be executed in parallel and it is the responsibility of the listener to avoid race conditions.
        onStep: *const fn (ptr: *anyopaque, context: *const PhysicsStepListenerContext) void,
    };

    pub fn init(impl: anytype) PhysicsStepListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn onStepThunk(ptr: *anyopaque, context: *const PhysicsStepListenerContext) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onStep(context);
            }
            const vtable: VTable = .{ .onStep = onStepThunk };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    pub fn onStep(self: PhysicsStepListener, context: *const PhysicsStepListenerContext) void {
        self.vtable.onStep(self.ptr, context);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// ContactConstraintManager (reduced stand-in: the constraint buffer, the per job ContactAllocator, island links, the
// deterministic sort, listener calls)

pub const ContactConstraint = struct {
    body1: *Body,
    body2: *Body,
    sort_key: u64,
    world_space_normal: Vec3,
    penetration_depth: f32,
    total_lambda: f32 = 0.0,
};

pub const ContactConstraintManager = struct {
    /// Per job state (Jolt: ContactAllocator, a LFHMAllocatorContext plus counters), on the stack of each FindCollisions job
    pub const ContactAllocator = struct {
        num_body_pairs: u32 = 0,
        num_manifolds: u32 = 0,
        errors: PhysicsUpdateError = .{},
    };

    max_constraints: u32 = 0,
    constraints: TempArray(ContactConstraint) = .empty,
    num_constraints: std.atomic.Value(u32) = .init(0),
    contact_listener: ?ContactListener = null,
    update_context: ?*PhysicsUpdateContext = null,
    physics_settings: *const PhysicsSettings = undefined,

    pub fn getContactAllocator(self: *const ContactConstraintManager) ContactAllocator {
        _ = self;
        return .{};
    }

    pub fn getMaxConstraints(self: *const ContactConstraintManager) u32 {
        return self.max_constraints;
    }

    pub fn getNumConstraints(self: *const ContactConstraintManager) u32 {
        return @min(self.num_constraints.load(.monotonic), self.max_constraints);
    }

    /// Prepare for simulation step (allocates the constraint buffer from the temp allocator)
    pub fn prepareConstraintBuffer(self: *ContactConstraintManager, context: *PhysicsUpdateContext) TempAllocator.Error!void {
        // Store context
        self.update_context = context;

        // Allocate temporary constraint buffer
        std.debug.assert(self.constraints.isEmpty());
        self.constraints = try .alloc(context.temp_allocator, self.max_constraints);
        self.num_constraints.store(0, .monotonic);
    }

    /// Restart the constraint buffer for the next collision step
    pub fn recycleConstraintBuffer(self: *ContactConstraintManager) void {
        self.num_constraints.store(0, .monotonic);
    }

    /// Terminate the constraint buffer (an empty buffer was never allocated)
    pub fn finishConstraintBuffer(self: *ContactConstraintManager) void {
        if (self.update_context) |context|
            self.constraints.free(context.temp_allocator);
        self.update_context = null;
    }

    /// The contact cache swap and OnContactRemoved callbacks (reduced: no cache)
    pub fn finalizeContactCacheAndCallContactPointRemovedCallbacks(self: *ContactConstraintManager, expected_num_body_pairs: u32, expected_num_manifolds: u32) void {
        _ = self;
        _ = expected_num_body_pairs;
        _ = expected_num_manifolds;
    }

    /// Add a contact constraint for this frame (called from FindCollisions jobs, the order is not deterministic)
    pub fn addContactConstraint(self: *ContactConstraintManager, contact_allocator: *ContactAllocator, activate_and_link_bodies: *bool, body1: *Body, body2: *Body, normal: Vec3, penetration_depth: f32) void {
        const update_context = self.update_context.?;

        // Reserve space for constraint (CreateConstraint)
        const constraint_idx = self.num_constraints.fetchAdd(1, .monotonic);
        if (constraint_idx >= self.max_constraints) {
            contact_allocator.errors.contact_constraints_full = true;
            return;
        }

        const body1_dynamic = body1.isDynamic();
        const body2_dynamic = body2.isDynamic();

        if (activate_and_link_bodies.*) {
            // Do this only once
            activate_and_link_bodies.* = false;

            // Wake up sleeping bodies
            var body_ids: [2]BodyID = undefined;
            var num_bodies: usize = 0;
            if (body1_dynamic and !body1.isActive()) {
                body_ids[num_bodies] = body1.getID();
                num_bodies += 1;
            }
            if (body2_dynamic and !body2.isActive()) {
                body_ids[num_bodies] = body2.getID();
                num_bodies += 1;
            }
            if (num_bodies > 0)
                update_context.body_manager.activateBodies(body_ids[0..num_bodies]);

            // Link the two bodies only if both are dynamic. If one of them is static or kinematic they don't need to go into
            // the same simulation island as a constraint cannot affect the velocity of a kinematic body.
            if (body1_dynamic and body2_dynamic)
                update_context.island_builder.linkBodies(body1.getIndexInActiveBodiesInternal(), body2.getIndexInActiveBodiesInternal());
        }

        // Link the contact to the first dynamic body
        if (body1_dynamic) {
            update_context.island_builder.linkContact(constraint_idx, body1.getIndexInActiveBodiesInternal());
        } else {
            std.debug.assert(body2_dynamic);
            update_context.island_builder.linkContact(constraint_idx, body2.getIndexInActiveBodiesInternal());
        }

        // Construct constraint (Jolt: the sort key is the hash of the SubShapeIDPair)
        const key = BodyPair.init(body1.getID(), body2.getID());
        self.constraints.items[constraint_idx] = .{ .body1 = body1, .body2 = body2, .sort_key = HashCombine.hashBytes(std.mem.asBytes(&key)), .world_space_normal = normal, .penetration_depth = penetration_depth };

        // Notify listener (from a worker thread)
        if (self.contact_listener) |listener|
            listener.onContactAdded(body1, body2);
    }

    /// Sort contact constraints deterministically (SortContacts: sort key, then body 1 ID, then body 2 ID)
    pub fn sortContacts(self: *const ContactConstraintManager, constraint_idx: []u32) void {
        zolt.quickSort(u32, constraint_idx, self.constraints.items, struct {
            fn lessThan(constraints: []ContactConstraint, lhs_idx: u32, rhs_idx: u32) bool {
                const lhs = &constraints[lhs_idx];
                const rhs = &constraints[rhs_idx];

                // Most of the time the sort key will be different so we sort on that
                if (lhs.sort_key != rhs.sort_key)
                    return lhs.sort_key < rhs.sort_key;

                // If they're equal we use the IDs of body 1 to order
                if (lhs.body1 != rhs.body1)
                    return lhs.body1.getID().lessThan(rhs.body1.getID());

                // If they're still equal we use the IDs of body 2 to order
                if (lhs.body2 != rhs.body2)
                    return lhs.body2.getID().lessThan(rhs.body2.getID());

                std.debug.assert(lhs_idx == rhs_idx); // Hash collision, ordering will be inconsistent
                return false;
            }
        }.lessThan);
    }

    /// Solve velocity constraints (reduced: one normal impulse with a Baumgarte bias, accumulated and clamped)
    pub fn solveVelocityConstraints(self: *const ContactConstraintManager, constraint_idx: []const u32, delta_time: f32) bool {
        var any_impulse_applied = false;
        for (constraint_idx) |idx| {
            const c = &self.constraints.items[idx];
            const inv_m1: f32 = if (c.body1.isDynamic()) c.body1.getMotionProperties().inv_mass else 0.0;
            const inv_m2: f32 = if (c.body2.isDynamic()) c.body2.getMotionProperties().inv_mass else 0.0;
            const inv_effective_mass = inv_m1 + inv_m2;
            if (inv_effective_mass == 0.0)
                continue;
            const relative_velocity = c.world_space_normal.dot(c.body2.getLinearVelocity().sub(c.body1.getLinearVelocity()));
            const bias = self.physics_settings.baumgarte / delta_time * math.max(@as(f32, 0.0), c.penetration_depth - self.physics_settings.penetration_slop);
            const lambda = -(relative_velocity - bias) / inv_effective_mass;
            const new_total = math.max(@as(f32, 0.0), c.total_lambda + lambda);
            const applied = new_total - c.total_lambda;
            c.total_lambda = new_total;
            if (applied != 0.0) any_impulse_applied = true;

            // The normal is the direction along which to move body 2 out of collision. Static bodies are never written.
            if (c.body1.isDynamic()) {
                const mp1 = c.body1.getMotionProperties();
                mp1.linear_velocity = mp1.linear_velocity.sub(c.world_space_normal.mulScalar(applied * inv_m1));
            }
            if (c.body2.isDynamic()) {
                const mp2 = c.body2.getMotionProperties();
                mp2.linear_velocity = mp2.linear_velocity.add(c.world_space_normal.mulScalar(applied * inv_m2));
            }
        }
        return any_impulse_applied;
    }
};

// -------------------------------------------------------------------------------------------------------------------
// PhysicsUpdateContext.h

/// Atomic followed by padding to the next cache line (Jolt: atomic<uint32> + uint8 mPaddingN[JPH_CACHE_LINE_SIZE - 4]).
/// Extern so the padding stays behind the atomic; no align(64): steps live in temp memory, which is 16 (32) byte aligned.
const PaddedAtomicU32 = extern struct {
    value: std.atomic.Value(u32) = .init(0),
    padding: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
};

/// A slot of the body pair queues. Jolt writes and reads a plain BodyPair while other jobs may read the slot before
/// their CAS fails (TSAN suppressed); Zolt stores the 8 bytes as one atomic, which is defined and costs nothing.
const BodyPairSlot = std.atomic.Value(u64);

fn storeBodyPair(slot: *BodyPairSlot, pair: BodyPair) void {
    slot.store(std.mem.bytesToValue(u64, std.mem.asBytes(&pair)), .monotonic);
}

fn loadBodyPair(slot: *const BodyPairSlot) BodyPair {
    const raw = slot.load(.monotonic);
    return std.mem.bytesToValue(BodyPair, std.mem.asBytes(&raw));
}

/// Information used during the PhysicsSystem::Update call (internal)
pub const PhysicsUpdateContext = struct {
    /// Maximum supported amount of concurrent jobs
    pub const max_concurrency = 32;

    pub const JobHandleArray = StaticArray(JobHandle, max_concurrency);

    /// A mask that has as many bits as we can have concurrent jobs
    pub const JobMask = u32;

    pub const BodyPairQueue = extern struct {
        write_idx: std.atomic.Value(u32) = .init(0), // Next index to write in mBodyPair array (need to add thread index * mMaxBodyPairsPerQueue and modulo mMaxBodyPairsPerQueue)
        padding1: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined, // Moved to own cache line to avoid conflicts with consumer jobs
        read_idx: std.atomic.Value(u32) = .init(0), // Next index to read in mBodyPair array (need to add thread index * mMaxBodyPairsPerQueue and modulo mMaxBodyPairsPerQueue)
        padding2: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined, // Moved to own cache line to avoid conflicts with producer/consumer jobs
    };

    pub const BodyPairQueues = StaticArray(BodyPairQueue, max_concurrency);

    /// Structure that contains data needed for each collision step.
    pub const Step = struct {
        /// Contains the information needed to cast a body through the scene to do continuous collision detection
        pub const CCDBody = struct {
            delta_position: Vec3, // Desired position step
            contact_normal: Vec3 = Vec3.zero(), // World space normal of closest hit (only valid if mFractionPlusSlop < 1)
            contact_point_on2: RVec3 = RVec3.zero(), // World space contact point on body 2 of closest hit (only valid if mFractionPlusSlop < 1)
            body_id1: BodyID, // Body 1 (the body that is performing collision detection)
            body_id2: BodyID = .invalid, // Body 2 (the body of the closest hit, only valid if mFractionPlusSlop < 1)
            fraction: f32 = 1.0, // Fraction at which the hit occurred
            fraction_plus_slop: f32 = 1.0, // Fraction at which the hit occurred + extra delta to allow body to penetrate by mMaxPenetration
            linear_cast_threshold_sq: f32, // Maximum allowed squared movement before doing a linear cast (determined by inner radius of shape)
            max_penetration: f32, // Maximum allowed penetration (determined by inner radius of shape)

            pub fn init(body_id1: BodyID, delta_position: Vec3, linear_cast_threshold_sq: f32, max_penetration: f32) CCDBody {
                return .{ .delta_position = delta_position, .body_id1 = body_id1, .linear_cast_threshold_sq = linear_cast_threshold_sq, .max_penetration = max_penetration };
            }
        };

        context: *PhysicsUpdateContext = undefined, // The physics update context

        is_first: bool = false, // If this is the first step
        is_last: bool = false, // If this is the last step

        broad_phase_update_state: BroadPhase.UpdateState = .{}, // Handle returned by Broadphase::UpdatePrepare

        num_active_bodies_at_step_start: u32 = 0, // Number of bodies that were active at the start of the physics update step. Only these bodies will receive gravity (they are the first N in the active body list).

        determine_active_constraint_read_idx: PaddedAtomicU32 = .{}, // Next constraint for determine active constraints
        num_active_constraints: PaddedAtomicU32 = .{}, // Number of constraints in the mActiveConstraints array
        setup_velocity_constraints_read_idx: PaddedAtomicU32 = .{}, // Next constraint for setting up velocity constraints
        step_listener_read_idx: PaddedAtomicU32 = .{}, // Next step listener to call
        apply_gravity_read_idx: PaddedAtomicU32 = .{}, // Next body to apply gravity to
        active_body_read_idx: PaddedAtomicU32 = .{}, // Index of fist active body that has not yet been processed by the broadphase

        body_pair_queues: BodyPairQueues = .empty, // Queues in which to put body pairs that need to be tested by the narrowphase

        max_body_pairs_per_queue: u32 = 0, // Amount of body pairs that we can queue per queue

        active_find_collision_jobs: std.atomic.Value(JobMask) = .init(0), // A bitmask that indicates which jobs are still active

        num_body_pairs: std.atomic.Value(u32) = .init(0), // The number of body pairs found in this step (used to size the contact cache in the next step)
        num_manifolds: std.atomic.Value(u32) = .init(0), // The number of manifolds found in this step (used to size the contact cache in the next step)

        solve_velocity_constraints_next_island: std.atomic.Value(u32) = .init(0), // Next island that needs to be processed for the solve velocity constraints step (doesn't need own cache line since position jobs don't run at same time)
        solve_position_constraints_next_island: std.atomic.Value(u32) = .init(0), // Next island that needs to be processed for the solve position constraints step (doesn't need own cache line since velocity jobs don't run at same time)

        integrate_velocity_read_idx: std.atomic.Value(u32) = .init(0), // Next active body index to take when integrating velocities
        ccd_bodies: TempArray(CCDBody) = .empty, // List of bodies that need to do continuous collision detection (capacity: items.len)
        num_ccd_bodies: std.atomic.Value(u32) = .init(0), // Number of CCD bodies in mCCDBodies
        next_ccd_body: std.atomic.Value(u32) = .init(0), // Next unprocessed body index in mCCDBodies
        active_body_to_ccd_body: TempArray(i32) = .empty, // A mapping between an index in BodyManager::mActiveBodies and the index in mCCDBodies

        // Jobs in order of execution (some run in parallel)
        broad_phase_prepare: JobHandle = .empty, // Prepares the new tree in the background
        step_listeners: JobHandleArray = .empty, // Listeners to notify of the beginning of a physics step
        determine_active_constraints: JobHandleArray = .empty, // Determine which constraints will be active during this step
        apply_gravity: JobHandleArray = .empty, // Update velocities of bodies with gravity
        find_collisions: JobHandleArray = .empty, // Find all collisions between active bodies an the world
        update_broadphase_finalize: JobHandle = .empty, // Swap the newly built tree with the current tree
        setup_velocity_constraints: JobHandleArray = .empty, // Calculate properties for all constraints in the constraint manager
        build_islands_from_constraints: JobHandle = .empty, // Go over all constraints and assign the bodies they're attached to to an island
        finalize_islands: JobHandle = .empty, // Finalize calculation simulation islands
        body_set_island_index: JobHandle = .empty, // Set the current island index on each body (not used by the simulation, only for drawing purposes)
        solve_velocity_constraints: JobHandleArray = .empty, // Solve the constraints in the velocity domain
        pre_integrate_velocity: JobHandle = .empty, // Setup integration of all body positions
        integrate_velocity: JobHandleArray = .empty, // Integrate all body positions
        post_integrate_velocity: JobHandle = .empty, // Finalize integration of all body positions
        resolve_ccd_contacts: JobHandle = .empty, // Updates the positions and velocities for all bodies that need continuous collision detection
        solve_position_constraints: JobHandleArray = .empty, // Solve all constraints in the position domain
        contact_removed_callbacks: JobHandle = .empty, // Calls the contact removed callbacks
        soft_body_prepare: JobHandle = .empty, // Prepares updating the soft bodies
        soft_body_collide: JobHandleArray = .empty, // Finds all colliding shapes for soft bodies (Phase 9)
        soft_body_simulate: JobHandleArray = .empty, // Simulates all particles (Phase 9)
        soft_body_finalize: JobHandle = .empty, // Finalizes the soft body update (Phase 9)
        start_next_step: JobHandle = .empty, // Job that kicks the next step (empty for the last step)

        /// ~Step: releases the job handles
        pub fn deinit(self: *Step) void {
            self.broad_phase_prepare.deinit();
            deinitHandles(&self.step_listeners);
            deinitHandles(&self.determine_active_constraints);
            deinitHandles(&self.apply_gravity);
            deinitHandles(&self.find_collisions);
            self.update_broadphase_finalize.deinit();
            deinitHandles(&self.setup_velocity_constraints);
            self.build_islands_from_constraints.deinit();
            self.finalize_islands.deinit();
            self.body_set_island_index.deinit();
            deinitHandles(&self.solve_velocity_constraints);
            self.pre_integrate_velocity.deinit();
            deinitHandles(&self.integrate_velocity);
            self.post_integrate_velocity.deinit();
            self.resolve_ccd_contacts.deinit();
            deinitHandles(&self.solve_position_constraints);
            self.contact_removed_callbacks.deinit();
            self.soft_body_prepare.deinit();
            deinitHandles(&self.soft_body_collide);
            deinitHandles(&self.soft_body_simulate);
            self.soft_body_finalize.deinit();
            self.start_next_step.deinit();
        }
    };

    /// StaticArray::resize default constructs in Jolt: new handles are empty
    pub fn resizeHandles(handles: *JobHandleArray, n: u32) void {
        std.debug.assert(handles.len == 0);
        handles.resize(n);
        for (handles.slice()) |*h| h.* = .empty;
    }

    fn deinitHandles(handles: *JobHandleArray) void {
        for (handles.slice()) |*h| h.deinit();
        handles.clear();
    }

    physics_system: *PhysicsSystem, // The physics system we belong to
    temp_allocator: TempAllocator, // Temporary allocator used during the update
    job_system: JobSystem, // Job system that processes jobs
    barrier: *JobSystem.Barrier = undefined, // Barrier used to wait for all physics jobs to complete

    step_delta_time: f32 = 0.0, // Delta time for a simulation step (collision step)
    warm_start_impulse_ratio: f32 = 0.0, // Ratio of this step delta time vs last step
    errors: std.atomic.Value(u32) = .init(0), // Errors that occurred during the update, actual type is PhysicsUpdateError

    /// Zolt (D5): set when an allocation failed inside a job and never cleared during the update. Poisoned jobs skip
    /// their work and allocations but keep their frees, commits and dependency bookkeeping; `update` returns
    /// error.OutOfMemory.
    out_of_memory: std.atomic.Value(bool) = .init(false),

    active_constraints: TempArray(*Constraint) = .empty, // Constraints that were active at the start of the physics update step (activating bodies can activate constraints and we need a consistent snapshot). Only these constraints will be resolved.

    body_pairs: TempArray(BodyPairSlot) = .empty, // A list of body pairs found by the broadphase

    body_manager: *BodyManager, // Keeps track of all bodies in the simulation
    island_builder: *IslandBuilder, // Keeps track of connected bodies and builds islands for multithreaded velocity/position update

    steps: TempArray(Step) = .empty, // Jolt: Array<Step, STLTempAllocator<Step>>, resized once, never reallocated

    /// Maximum amount of concurrent jobs on this machine
    pub fn getMaxConcurrency(self: *const PhysicsUpdateContext) u32 {
        return @intCast(@min(max_concurrency, self.job_system.getMaxConcurrency()));
    }

    /// Latch an allocation failure (D5)
    pub fn setOutOfMemory(self: *PhysicsUpdateContext) void {
        self.out_of_memory.store(true, .release);
    }

    /// True after an allocation failure: jobs skip their work
    pub fn isOutOfMemory(self: *const PhysicsUpdateContext) bool {
        return self.out_of_memory.load(.acquire);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// BodyInterface.h (reduced)

/// Class that provides operations on bodies using a body ID. Note that if you need to do multiple operations on a single body, it is more efficient to lock the body once and combine the operations.
/// All quantities are in world space unless otherwise specified.
pub const BodyInterface = struct {
    body_lock_interface: *const BodyLockInterface = undefined,
    body_manager: *BodyManager = undefined,
    broad_phase: *BroadPhase = undefined,

    /// Initialize the interface (should only be called by PhysicsSystem)
    pub fn init(body_lock_interface: *const BodyLockInterface, body_manager: *BodyManager, broad_phase: *BroadPhase) BodyInterface {
        return .{ .body_lock_interface = body_lock_interface, .body_manager = body_manager, .broad_phase = broad_phase };
    }

    /// Create a rigid body. Returns null when out of bodies (Jolt: nullptr); error.InvalidShape when the settings have
    /// no shape, error.OutOfMemory when allocation fails (Zolt).
    pub fn createBody(self: *const BodyInterface, settings: *const BodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Create a body with specified ID. This function can be used if a simulation is to run in sync between clients or if a simulation needs to be restored exactly.
    pub fn createBodyWithID(self: *const BodyInterface, body_id: BodyID, settings: *const BodyCreationSettings) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBodyWithCustomID(body, body_id)) {
            self.body_manager.freeBody(body);
            return null;
        }
        return body;
    }

    /// Destroy a body. Make sure that you remove the body from the physics system using BodyInterface::RemoveBody before calling this function.
    pub fn destroyBody(self: *const BodyInterface, body_id: BodyID) void {
        self.body_manager.destroyBodies((&body_id)[0..1]);
    }

    /// Add body to the physics system. Zolt: the broad phase allocates in AddBodiesPrepare, so Allocator.Error; on
    /// error the body was not added.
    pub fn addBody(self: *const BodyInterface, body_id: BodyID, activation_mode: Activation) Allocator.Error!void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();

            // Add to broadphase
            var id = body_id;
            const add_state = try self.broad_phase.addBodiesPrepare((&id)[0..1]);
            self.broad_phase.addBodiesFinalize((&id)[0..1], add_state);

            // Optionally activate body
            if (activation_mode == .activate and !body.isStatic())
                self.body_manager.activateBodies((&body_id)[0..1]);
        }
    }

    /// Remove body from the physics system.
    pub fn removeBody(self: *const BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeededAndIsInBroadPhase()) {
            const body = lock.getBody();

            // Deactivate body
            if (body.isActive())
                self.body_manager.deactivateBodies((&body_id)[0..1]);

            // Remove from broadphase
            var id = body_id;
            self.broad_phase.removeBodies((&id)[0..1]);
        }
    }

    /// Combines CreateBody and AddBody. Returns an invalid ID when out of bodies (Jolt); on an error nothing is left
    /// behind (Zolt).
    pub fn createAndAddBody(self: *const BodyInterface, settings: *const BodyCreationSettings, activation_mode: Activation) CreateBodyError!BodyID {
        const body = (try self.createBody(settings)) orelse return .invalid; // Out of bodies
        const id = body.getID();
        errdefer self.destroyBody(id);
        try self.addBody(id, activation_mode);
        return id;
    }

    /// Prepare adding multiple bodies (the handle goes to AddBodiesFinalize or AddBodiesAbort)
    pub fn addBodiesPrepare(self: *const BodyInterface, bodies: []BodyID) Allocator.Error!BroadPhase.AddState {
        return self.broad_phase.addBodiesPrepare(bodies);
    }

    /// Finalize adding multiple bodies to the physics system (never fails)
    pub fn addBodiesFinalize(self: *const BodyInterface, bodies: []BodyID, add_state: BroadPhase.AddState, activation_mode: Activation) void {
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, bodies);
        defer lock.deinit();

        // Add to broadphase
        self.broad_phase.addBodiesFinalize(bodies, add_state);

        // Optionally activate bodies
        if (activation_mode == .activate)
            self.body_manager.activateBodies(bodies);
    }

    /// Abort adding multiple bodies to the physics system
    pub fn addBodiesAbort(self: *const BodyInterface, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        self.broad_phase.addBodiesAbort(bodies, add_state);
    }

    /// Check if a body has been added to the physics system.
    pub fn isAdded(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return lock.succeededAndIsInBroadPhase();
    }

    /// Create a two body constraint (null bodies are replaced with Body.fixedToWorld()). Allocates with the body
    /// manager's allocator (Jolt: new); reference count 0, like Jolt.
    pub fn createConstraint(self: *const BodyInterface, settings: *const TwoBodyConstraintSettings, body_id1: BodyID, body_id2: BodyID) Allocator.Error!*TwoBodyConstraint {
        const constraint_bodies = [_]BodyID{ body_id1, body_id2 };
        var lock: BodyLockMultiWrite = .init(self.body_lock_interface, &constraint_bodies);
        defer lock.deinit();

        const body1 = lock.getBody(0);
        const body2 = lock.getBody(1);

        std.debug.assert(body1 != null or body2 != null);
        std.debug.assert(body1 != body2);

        return settings.createConstraint(self.body_manager.allocator, body1 orelse Body.fixedToWorld(), body2 orelse Body.fixedToWorld());
    }

    /// Activate a body
    pub fn activateBody(self: *const BodyInterface, body_id: BodyID) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isActive())
                self.body_manager.activateBodies((&body_id)[0..1]);
        }
    }

    /// Check if this body is active
    pub fn isActive(self: *const BodyInterface, body_id: BodyID) bool {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return lock.succeeded() and lock.getBody().isActive();
    }

    pub fn getPosition(self: *const BodyInterface, body_id: BodyID) RVec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return if (lock.succeeded()) lock.getBody().getPosition() else RVec3.zero();
    }

    pub fn getLinearVelocity(self: *const BodyInterface, body_id: BodyID) Vec3 {
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        return if (lock.succeeded()) lock.getBody().getLinearVelocity() else Vec3.zero();
    }

    pub fn setLinearVelocity(self: *const BodyInterface, body_id: BodyID, linear_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocity(linear_velocity);

                // Activate the body if needed
                if (!body.isActive() and !linear_velocity.isNearZero(.{}))
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// Add velocity to current velocity
    pub fn addLinearVelocity(self: *const BodyInterface, body_id: BodyID, linear_velocity: Vec3) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (!body.isStatic()) {
                body.setLinearVelocity(body.getLinearVelocity().add(linear_velocity));

                // Activate the body if needed
                if (!body.isActive() and !body.getLinearVelocity().isNearZero(.{}))
                    self.body_manager.activateBodies((&body_id)[0..1]);
            }
        }
    }

    /// Add force at center of mass. Jolt's defaulted `EActivation inActivationMode = EActivation::Activate` is an option.
    pub fn addForce(self: *const BodyInterface, body_id: BodyID, force: Vec3, opts: struct { activation: Activation = .activate }) void {
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            if (body.isDynamic() and (opts.activation == .activate or body.isActive())) {
                body.addForce(force);

                if (opts.activation == .activate)
                    self.activateBodyInternal(body);
            }
        }
    }

    fn activateBodyInternal(self: *const BodyInterface, body: *Body) void {
        // Activate body or reset its sleep timer.
        // Note that BodyManager::ActivateBodies also resets the sleep timer internally, but we avoid a mutex lock if the body is already active by calling ResetSleepTimer directly.
        if (!body.isActive())
            self.body_manager.activateBodies((&body.id)[0..1])
        else
            body.resetSleepTimer();
    }

    /// Get the current shape (the caller owns the returned reference: `deinit` it)
    pub fn getShape(self: *const BodyInterface, body_id: BodyID) RefConst(Shape) {
        var shape: RefConst(Shape) = .empty;
        var lock: BodyLockRead = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
        if (lock.succeeded())
            shape.set(lock.getBody().getShape());
        return shape;
    }
};

// -------------------------------------------------------------------------------------------------------------------
// PhysicsSystem.h / PhysicsSystem.cpp

/// The main class for the physics system. It contains all rigid bodies and simulates them.
pub const PhysicsSystem = struct {
    const Self = @This();
    const Step = PhysicsUpdateContext.Step;
    const CCDBody = Step.CCDBody;
    const JobMask = PhysicsUpdateContext.JobMask;
    const ContactAllocator = ContactConstraintManager.ContactAllocator;

    // Batch sizes (PhysicsSystem.h)
    const determine_active_constraints_batch_size = 64;
    const setup_velocity_constraints_batch_size = 256;
    const apply_gravity_batch_size = 64;
    const active_bodies_batch_size = 16;
    const integrate_velocity_batch_size = 64;
    const narrow_phase_batch_size = 16;
    const num_ccd_bodies_per_job = 4;

    // Job colors (Color::sGetDistinctColor(n))
    const color_update_broad_phase_finalize = Color.getDistinctColor(1);
    const color_update_broad_phase_prepare = Color.getDistinctColor(2);
    const color_find_collisions = Color.getDistinctColor(3);
    const color_apply_gravity = Color.getDistinctColor(4);
    const color_setup_velocity_constraints = Color.getDistinctColor(5);
    const color_build_islands_from_constraints = Color.getDistinctColor(6);
    const color_determine_active_constraints = Color.getDistinctColor(7);
    const color_finalize_islands = Color.getDistinctColor(8);
    const color_contact_removed_callbacks = Color.getDistinctColor(9);
    const color_body_set_island_index = Color.getDistinctColor(10);
    const color_start_next_step = Color.getDistinctColor(11);
    const color_solve_velocity_constraints = Color.getDistinctColor(12);
    const color_pre_integrate_velocity = Color.getDistinctColor(13);
    const color_integrate_velocity = Color.getDistinctColor(14);
    const color_post_integrate_velocity = Color.getDistinctColor(15);
    const color_resolve_ccd_contacts = Color.getDistinctColor(16);
    const color_solve_position_constraints = Color.getDistinctColor(17);
    const color_find_ccd_contacts = Color.getDistinctColor(18);
    const color_step_listeners = Color.getDistinctColor(19);
    const color_soft_body_prepare = Color.getDistinctColor(20);

    allocator: Allocator = undefined,
    io: std.Io = undefined,

    /// Broadphase layer filter that decides if two objects can collide
    object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter = undefined,

    /// Object layer filter that decides if two objects can collide
    object_layer_pair_filter: *const ObjectLayerPairFilter = undefined,

    /// Keeps track of all bodies
    body_manager: BodyManager = .empty,

    /// Body locking interfaces (Jolt's two `final` classes, a closed tagged struct)
    body_lock_interface_no_lock: BodyLockInterface = undefined,
    body_lock_interface_locking: BodyLockInterface = undefined,

    /// Body interfaces
    body_interface_no_lock: BodyInterface = .{},
    body_interface_locking: BodyInterface = .{},

    /// Broadphase (Jolt: BroadPhase *mBroadPhase = new BroadPhaseQuadTree)
    broad_phase: *BroadPhase = undefined,

    /// The contact manager resolves all contacts during a simulation step
    contact_manager: ContactConstraintManager = .{},

    /// All non-contact constraints
    constraint_manager: ConstraintManager = .empty,

    /// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
    island_builder: IslandBuilder = .{},

    /// Mutex protecting mStepListeners
    step_listeners_mutex: Mutex = .{},

    /// List of physics step listeners
    step_listeners: std.ArrayList(PhysicsStepListener) = .empty,

    /// Global physics settings
    physics_settings: PhysicsSettings = .{},

    /// Gravity
    gravity: Vec3 = Vec3.init(0, -9.81, 0),

    /// Previous frame's delta time of one sub step to allow scaling previous frame's constraint impulses
    previous_step_delta_time: f32 = 0.0,

    /// Prototype instrumentation (not part of the design): CCD bodies resolved, FindCollisions jobs spawned in jobs
    debug_num_ccd_bodies: std.atomic.Value(u32) = .init(0),
    debug_num_spawned_jobs: std.atomic.Value(u32) = .init(0),

    /// Not initialized (default constructor)
    pub const empty: PhysicsSystem = .{};

    /// Initialize the system (Init). In place: the lock and body interfaces point into it, it must not move afterwards.
    pub fn init(self: *Self, allocator: Allocator, io: std.Io, max_bodies: u32, num_body_mutexes: u32, max_contact_constraints: u32, broad_phase_layer_interface: *const BroadPhaseLayerInterface, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter) Allocator.Error!void {
        self.* = .{ .allocator = allocator, .io = io, .object_vs_broad_phase_layer_filter = object_vs_broad_phase_layer_filter, .object_layer_pair_filter = object_layer_pair_filter };

        // Initialize body manager
        try self.body_manager.init(allocator, io, max_bodies, num_body_mutexes, broad_phase_layer_interface);
        errdefer self.body_manager.deinit();
        self.body_lock_interface_no_lock = .initNoLock(&self.body_manager);
        self.body_lock_interface_locking = .initLocking(&self.body_manager);

        // Create broadphase
        const broad_phase = try PrototypeBroadPhase.create(allocator);
        self.broad_phase = broad_phase.asBroadPhase();
        errdefer self.broad_phase.base.destroy();
        try self.broad_phase.initBroadPhase(&self.body_manager, broad_phase_layer_interface);

        // Init contact constraint manager
        self.contact_manager = .{ .max_constraints = max_contact_constraints, .physics_settings = &self.physics_settings };

        // Init islands builder
        try self.island_builder.init(allocator, max_bodies);
        errdefer self.island_builder.deinit();

        // Init constraint manager
        try self.constraint_manager.init(allocator, io, &self.body_manager);

        // Initialize body interface
        self.body_interface_locking = .init(&self.body_lock_interface_locking, &self.body_manager, self.broad_phase);
        self.body_interface_no_lock = .init(&self.body_lock_interface_no_lock, &self.body_manager, self.broad_phase);
    }

    pub fn deinit(self: *Self) void {
        self.constraint_manager.deinit();
        self.broad_phase.base.destroy(); // delete mBroadPhase
        self.island_builder.deinit();
        self.body_manager.deinit();
        self.step_listeners.deinit(self.allocator);
        self.* = .empty;
    }

    pub fn getBodyInterface(self: *Self) *const BodyInterface {
        return &self.body_interface_locking;
    }
    pub fn getBodyInterfaceNoLock(self: *Self) *const BodyInterface {
        return &self.body_interface_no_lock;
    }
    pub fn getBodyLockInterface(self: *const Self) *const BodyLockInterface {
        return &self.body_lock_interface_locking;
    }
    pub fn getBodyLockInterfaceNoLock(self: *const Self) *const BodyLockInterface {
        return &self.body_lock_interface_no_lock;
    }
    pub fn getBroadPhaseQuery(self: *const Self) *const BroadPhaseFile.BroadPhaseQuery {
        return &self.broad_phase.base;
    }

    pub fn setContactListener(self: *Self, listener: ?ContactListener) void {
        self.contact_manager.contact_listener = listener;
    }

    pub fn setBodyActivationListener(self: *Self, listener: ?BodyActivationListener) void {
        self.body_manager.setBodyActivationListener(listener);
    }

    /// Add constraint to the world
    pub fn addConstraint(self: *Self, constraint: *Constraint) Allocator.Error!void {
        try self.constraint_manager.add((&constraint)[0..1]);
    }

    /// Remove constraint from the world
    pub fn removeConstraint(self: *Self, constraint: *Constraint) void {
        self.constraint_manager.remove((&constraint)[0..1]);
    }

    /// Adds a new step listener
    pub fn addStepListener(self: *Self, listener: PhysicsStepListener) Allocator.Error!void {
        self.step_listeners_mutex.lock(self.io);
        defer self.step_listeners_mutex.unlock(self.io);

        for (self.step_listeners.items) |l| std.debug.assert(l.ptr != listener.ptr);
        try self.step_listeners.append(self.allocator, listener);
    }

    /// Removes a step listener (found by its object, swap removed like Jolt)
    pub fn removeStepListener(self: *Self, listener: PhysicsStepListener) void {
        self.step_listeners_mutex.lock(self.io);
        defer self.step_listeners_mutex.unlock(self.io);

        for (self.step_listeners.items, 0..) |l, i| {
            if (l.ptr == listener.ptr) {
                _ = self.step_listeners.swapRemove(i);
                return;
            }
        }
        std.debug.assert(false);
    }

    /// Simulate the system.
    /// The world steps for a total of inDeltaTime seconds. This is divided in inCollisionSteps iterations.
    /// Each iteration consists of collision detection followed by an integration step.
    /// This function internally spawns jobs using inJobSystem and waits for them to complete, so no jobs will be running when this function returns.
    /// Returns Jolt's soft errors, or error.OutOfMemory when the temp allocator or the allocator ran out (D5): a
    /// failure while building the jobs changes nothing, a failure inside a job leaves the step half done (memory safe,
    /// no locks held, nothing leaked: restore a saved state).
    pub fn update(self: *Self, delta_time: f32, collision_steps: u32, temp_allocator: TempAllocator, job_system: JobSystem) error{OutOfMemory}!PhysicsUpdateError {
        std.debug.assert(collision_steps > 0);
        std.debug.assert(delta_time >= 0.0);

        // Sync point for the broadphase. This will allow it to do clean up operations without having any mutexes locked yet.
        self.broad_phase.frameSync();

        // If there are no active bodies (and no step listener to wake them up) or there's no time delta
        const num_active_rigid_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
        const num_active_soft_bodies = self.body_manager.getNumActiveBodies(.soft_body);
        if ((num_active_rigid_bodies == 0 and num_active_soft_bodies == 0 and self.step_listeners.items.len == 0) or delta_time <= 0.0) {
            self.body_manager.lockAllBodies();

            // Update broadphase
            self.broad_phase.lockModifications();
            const update_state = self.broad_phase.updatePrepare() catch |err| {
                // Zolt: the broad phase is unchanged
                self.broad_phase.unlockModifications();
                self.body_manager.unlockAllBodies();
                return err;
            };
            self.broad_phase.updateFinalize(&update_state);
            self.broad_phase.unlockModifications();

            // If time has passed, call contact removal callbacks from contacts that existed in the previous update
            if (delta_time > 0.0)
                self.contact_manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(0, 0);

            self.body_manager.unlockAllBodies();
            return .none;
        }

        // Calculate ratio between current and previous frame delta time to scale initial constraint forces.
        const step_delta_time = delta_time / @as(f32, @floatFromInt(collision_steps));
        const warm_start_impulse_ratio: f32 = if (self.previous_step_delta_time > 0.0) math.min(step_delta_time / self.previous_step_delta_time, @as(f32, 4.0)) else 0.0;
        const old_previous_step_delta_time = self.previous_step_delta_time; // Zolt: restored when building the jobs fails
        self.previous_step_delta_time = step_delta_time;

        // Create the context used for passing information between jobs
        var context: PhysicsUpdateContext = .{
            .physics_system = self,
            .temp_allocator = temp_allocator,
            .job_system = job_system,
            .body_manager = &self.body_manager,
            .island_builder = &self.island_builder,
            .step_delta_time = step_delta_time,
            .warm_start_impulse_ratio = warm_start_impulse_ratio,
        };
        context.barrier = job_system.createBarrier() orelse @panic("PhysicsSystem: no barrier available (increase max_barriers of the job system)"); // Jolt crashes

        // Lock all bodies for write so that we can freely touch them
        self.step_listeners_mutex.lock(self.io);
        self.body_manager.lockAllBodies();
        self.broad_phase.lockModifications();

        // Build the jobs. Zolt: no job runs before every job exists (start gate), so a failure has no side effects.
        var constraints_locked = false;
        self.buildJobs(&context, collision_steps, &constraints_locked) catch |err| {
            self.finishUpdate(&context, constraints_locked);
            self.previous_step_delta_time = old_previous_step_delta_time;
            return err;
        };

        // Zolt start gate: Jolt starts the first step's root jobs while building, Zolt kicks them now, exactly like
        // StartNextStep kicks the following steps (their roots have 1 dependency in both)
        kickStep(&context.steps.items[0]);

        // Build the list of jobs to wait for
        {
            var handles: StaticArray(JobHandle, @intCast(zolt.physics_settings.max_physics_jobs)) = .empty;
            defer for (handles.slice()) |*h| h.deinit();
            for (context.steps.items) |*step| {
                if (step.broad_phase_prepare.isValid())
                    handles.append(step.broad_phase_prepare.clone());
                for (step.step_listeners.constSlice()) |*h| handles.append(h.clone());
                for (step.determine_active_constraints.constSlice()) |*h| handles.append(h.clone());
                for (step.apply_gravity.constSlice()) |*h| handles.append(h.clone());
                for (step.find_collisions.constSlice()) |*h| handles.append(h.clone());
                if (step.update_broadphase_finalize.isValid())
                    handles.append(step.update_broadphase_finalize.clone());
                for (step.setup_velocity_constraints.constSlice()) |*h| handles.append(h.clone());
                handles.append(step.build_islands_from_constraints.clone());
                handles.append(step.finalize_islands.clone());
                handles.append(step.body_set_island_index.clone());
                for (step.solve_velocity_constraints.constSlice()) |*h| handles.append(h.clone());
                handles.append(step.pre_integrate_velocity.clone());
                for (step.integrate_velocity.constSlice()) |*h| handles.append(h.clone());
                handles.append(step.post_integrate_velocity.clone());
                handles.append(step.resolve_ccd_contacts.clone());
                for (step.solve_position_constraints.constSlice()) |*h| handles.append(h.clone());
                handles.append(step.contact_removed_callbacks.clone());
                if (step.soft_body_prepare.isValid())
                    handles.append(step.soft_body_prepare.clone());
                if (step.start_next_step.isValid())
                    handles.append(step.start_next_step.clone());
            }
            context.barrier.addJobs(handles.constSlice());
        }

        // Wait until all jobs finish
        // Note we don't just wait for the last job. If we would and another job
        // would be scheduled in between there is the possibility of a deadlock.
        // The other job could try to e.g. add/remove a body which would try to
        // lock a body mutex while this thread has already locked the mutex
        job_system.waitForJobs(context.barrier);

        self.finishUpdate(&context, constraints_locked);

        // Zolt: allocation failure inside a job (D5)
        if (context.isOutOfMemory())
            return error.OutOfMemory;

        // Return any errors
        const errors: PhysicsUpdateError = @bitCast(context.errors.load(.acquire));
        if (Core.enable_asserts) std.debug.assert(errors.isNone()); // An error occurred during the physics update, see EPhysicsUpdateError for more information
        return errors;
    }

    /// Kick the root jobs of a step (shared by the Zolt start gate and StartNextStep)
    fn kickStep(step: *Step) void {
        // Kick the jobs of the next step (in the same order as the first step)
        step.broad_phase_prepare.removeDependency(.{});
        if (step.step_listeners.isEmpty()) {
            // Kick the gravity and active constraints jobs immediately
            JobHandle.removeDependenciesStaticArray(&step.apply_gravity, .{});
            JobHandle.removeDependenciesStaticArray(&step.determine_active_constraints, .{});
        } else {
            // Kick the step listeners job first
            JobHandle.removeDependenciesStaticArray(&step.step_listeners, .{});
        }
    }

    /// The tail of Update, also the cleanup of a failed build: Jolt's frees in Jolt's order (TempArrays that were
    /// never allocated are skipped) and the unlocks.
    fn finishUpdate(self: *Self, context: *PhysicsUpdateContext, constraints_locked: bool) void {
        const temp_allocator = context.temp_allocator;

        // We're done with the barrier for this update
        context.job_system.destroyBarrier(context.barrier);

        // Clear the large island splitter (reduced: no splitter)

        // Clear the island builder
        self.island_builder.resetIslands(temp_allocator);

        // Clear the contact manager
        self.contact_manager.finishConstraintBuffer();

        // Free active constraints
        context.active_constraints.free(temp_allocator);

        // Free body pairs
        context.body_pairs.free(temp_allocator);

        // Unlock the broadphase
        self.broad_phase.unlockModifications();

        // Unlock all constraints
        if (constraints_locked)
            self.constraint_manager.unlockAllConstraints();

        // Allow write operations to the active bodies list
        self.body_manager.setActiveBodiesLocked(false);

        // Unlock all bodies
        self.body_manager.unlockAllBodies();

        // Unlock step listeners
        self.step_listeners_mutex.unlock(self.io);

        // ~PhysicsUpdateContext: release the job handles, free the steps (the first temp allocation)
        for (context.steps.items) |*step| step.deinit();
        context.steps.free(temp_allocator);
    }

    /// The "Build Jobs" block of PhysicsSystem::Update, line by line. Every C++ lambda is an inline closure whose
    /// captures are the lambda's: `[&context, &step]` -> `.{ context, step }`, `[&step, i]` -> `.{ step, i }`.
    fn buildJobs(self: *Self, context: *PhysicsUpdateContext, collision_steps: u32, constraints_locked: *bool) Allocator.Error!void {
        const temp_allocator = context.temp_allocator;
        const job_system = context.job_system;
        const num_active_rigid_bodies = self.body_manager.getNumActiveBodies(.rigid_body);

        // context.mSteps.resize(inCollisionSteps) (STLTempAllocator)
        context.steps = try .alloc(temp_allocator, collision_steps);
        for (context.steps.items) |*s| s.* = .{};
        const steps = context.steps.items;

        // Allocate space for body pairs
        std.debug.assert(context.body_pairs.isEmpty());
        context.body_pairs = try .alloc(temp_allocator, @intCast(self.physics_settings.max_in_flight_body_pairs));

        // Get max number of concurrent jobs
        const max_concurrency = context.getMaxConcurrency();

        // Calculate how many step listener jobs we spawn
        const num_step_listener_jobs: u32 = if (self.step_listeners.items.len == 0) 0 else @max(1, @min(@as(u32, @intCast(self.step_listeners.items.len)) / @as(u32, @intCast(self.physics_settings.step_listeners_batch_size)) / @as(u32, @intCast(self.physics_settings.step_listener_batches_per_job)), max_concurrency));

        // Number of gravity jobs depends on the amount of active bodies.
        // Launch max 1 job per batch of active bodies
        // Leave 1 thread for update broadphase prepare and 1 for determine active constraints
        const num_apply_gravity_jobs: u32 = @intCast(@max(1, @min(@divTrunc(@as(i32, @intCast(num_active_rigid_bodies)) + apply_gravity_batch_size - 1, apply_gravity_batch_size), @as(i32, @intCast(max_concurrency)) - 2)));

        // Number of determine active constraints jobs to run depends on number of constraints.
        // Leave 1 thread for update broadphase prepare and 1 for apply gravity
        const num_determine_active_constraints_jobs: u32 = @intCast(@max(1, @min(@divTrunc(@as(i32, @intCast(self.constraint_manager.getNumConstraints())) + determine_active_constraints_batch_size - 1, determine_active_constraints_batch_size), @as(i32, @intCast(max_concurrency)) - 2)));

        // Number of setup velocity constraints jobs to run depends on number of constraints.
        const num_setup_velocity_constraints_jobs: u32 = @max(1, @min((self.constraint_manager.getNumConstraints() + setup_velocity_constraints_batch_size - 1) / setup_velocity_constraints_batch_size, max_concurrency));

        // Number of find collisions jobs to run depends on number of active bodies.
        // Note that when we have more than 1 thread, we always spawn at least 2 find collisions jobs so that the first job can wait for build islands from constraints
        // (which may activate additional bodies that need to be processed) while the second job can start processing collision work.
        const num_find_collisions_jobs: u32 = @max(@as(u32, if (max_concurrency == 1) 1 else 2), @min((num_active_rigid_bodies + active_bodies_batch_size - 1) / active_bodies_batch_size, max_concurrency));

        // Number of integrate velocity jobs depends on number of active bodies.
        const num_integrate_velocity_jobs: u32 = @max(1, @min((num_active_rigid_bodies + integrate_velocity_batch_size - 1) / integrate_velocity_batch_size, max_concurrency));

        // Iterate over collision steps
        for (steps, 0..) |*step, step_idx| {
            const is_first_step = step_idx == 0;
            const is_last_step = step_idx == collision_steps - 1;

            step.context = context;
            step.is_first = is_first_step;
            step.is_last = is_last_step;

            // Create job to do broadphase finalization
            // This job must finish before integrating velocities. Until then the positions will not be updated neither will bodies be added / removed.
            step.update_broadphase_finalize = try job_system.createJob("UpdateBroadPhaseFinalize", color_update_broad_phase_finalize, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Validate that all find collision jobs have stopped
                    std.debug.assert(s.active_find_collision_jobs.load(.monotonic) == 0);

                    // Finalize the broadphase update (Zolt D5: a commit, it also runs when poisoned; a state that was
                    // not prepared is zeroed and does nothing)
                    c.physics_system.broad_phase.updateFinalize(&s.broad_phase_update_state);

                    // Signal that it is done
                    s.pre_integrate_velocity.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_find_collisions_jobs + 2 }); // depends on: find collisions, broadphase prepare update, finish building jobs

            // The immediate jobs below are only immediate for the first step, the all finished job will kick them for the next step
            // Zolt start gate: 1 for the first step too, `update` kicks the first step after building (kickStep)
            const previous_step_dependency_count: u32 = 1;

            // Start job immediately: Start the prepare broadphase
            // Must be done under body lock protection since the order is body locks then broadphase mutex
            // If this is turned around the RemoveBody call will hang since it locks in that order
            step.broad_phase_prepare = try job_system.createJob("UpdateBroadPhasePrepare", color_update_broad_phase_prepare, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Prepare the broadphase update (Zolt D5: allocates, on error the state stays zeroed)
                    if (!c.isOutOfMemory())
                        s.broad_phase_update_state = c.physics_system.broad_phase.updatePrepare() catch blk: {
                            c.setOutOfMemory();
                            break :blk .{};
                        };

                    // Now the finalize can run (if other dependencies are met too)
                    s.update_broadphase_finalize.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = previous_step_dependency_count });

            // This job will find all collisions
            step.body_pair_queues.resize(max_concurrency);
            for (step.body_pair_queues.slice()) |*q| q.* = .{};
            step.max_body_pairs_per_queue = @as(u32, @intCast(self.physics_settings.max_in_flight_body_pairs)) / max_concurrency;
            step.active_find_collision_jobs.store(~@as(JobMask, 0) >> @intCast(@bitSizeOf(JobMask) - num_find_collisions_jobs), .release);
            PhysicsUpdateContext.resizeHandles(&step.find_collisions, num_find_collisions_jobs);
            for (0..num_find_collisions_jobs) |i| {
                // Build islands from constraints may activate additional bodies, so the first job will wait for this to finish in order to not miss any active bodies
                const num_dep_build_islands_from_constraints: u32 = if (i == 0) 1 else 0;
                step.find_collisions.slice()[i] = try job_system.createJob("FindCollisions", color_find_collisions, .init(struct {
                    fn call(s: *Step, job_index: u32) void {
                        // Zolt D5: the job body carries the claim protocol, it checks the latch itself
                        s.context.physics_system.jobFindCollisions(s, job_index);
                    }
                }.call, .{ step, @as(u32, @intCast(i)) }), .{ .num_dependencies = num_apply_gravity_jobs + num_determine_active_constraints_jobs + 1 + num_dep_build_islands_from_constraints }); // depends on: apply gravity, determine active constraints, finish building jobs, build islands from constraints
            }

            if (is_first_step) {
                // Don't allow write operations to the active bodies list
                self.body_manager.setActiveBodiesLocked(true);

                // Store the number of active bodies at the start of the step
                step.num_active_bodies_at_step_start = self.body_manager.getNumActiveBodies(.rigid_body);

                // Lock all constraints
                self.constraint_manager.lockAllConstraints();
                constraints_locked.* = true;

                // Allocate memory for storing the active constraints
                std.debug.assert(context.active_constraints.isEmpty());
                context.active_constraints = try .alloc(temp_allocator, self.constraint_manager.getNumConstraints());

                // Prepare contact buffer
                try self.contact_manager.prepareConstraintBuffer(context);

                // Setup island builder
                try self.island_builder.prepareContactConstraints(self.contact_manager.getMaxConstraints(), temp_allocator);
            }

            // This job applies gravity to all active bodies
            PhysicsUpdateContext.resizeHandles(&step.apply_gravity, num_apply_gravity_jobs);
            for (step.apply_gravity.slice()) |*h|
                h.* = try job_system.createJob("ApplyGravity", color_apply_gravity, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobApplyGravity(c, s);

                        JobHandle.removeDependenciesStaticArray(&s.find_collisions, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = if (num_step_listener_jobs > 0) num_step_listener_jobs else previous_step_dependency_count }); // depends on: step listeners (or previous step if no step listeners)

            // This job will setup velocity constraints for non-collision constraints
            PhysicsUpdateContext.resizeHandles(&step.setup_velocity_constraints, num_setup_velocity_constraints_jobs);
            for (step.setup_velocity_constraints.slice()) |*h|
                h.* = try job_system.createJob("SetupVelocityConstraints", color_setup_velocity_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSetupVelocityConstraints(c.step_delta_time, s);

                        JobHandle.removeDependenciesStaticArray(&s.solve_velocity_constraints, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = num_determine_active_constraints_jobs + 1 }); // depends on: determine active constraints, finish building jobs

            // This job will build islands from constraints
            step.build_islands_from_constraints = try job_system.createJob("BuildIslandsFromConstraints", color_build_islands_from_constraints, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobBuildIslandsFromConstraints(c, s) catch c.setOutOfMemory();

                    s.find_collisions.slice()[0].removeDependency(.{}); // The first collisions job cannot start running until we've finished building islands and activated all bodies
                    s.finalize_islands.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_determine_active_constraints_jobs + 1 }); // depends on: determine active constraints, finish building jobs

            // This job determines active constraints
            PhysicsUpdateContext.resizeHandles(&step.determine_active_constraints, num_determine_active_constraints_jobs);
            for (step.determine_active_constraints.slice()) |*h|
                h.* = try job_system.createJob("DetermineActiveConstraints", color_determine_active_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobDetermineActiveConstraints(s);

                        s.build_islands_from_constraints.removeDependency(.{});

                        // Kick these jobs last as they will use up all CPU cores leaving no space for the previous job, we prefer setup velocity constraints to finish first so we kick it first
                        JobHandle.removeDependenciesStaticArray(&s.setup_velocity_constraints, .{});
                        JobHandle.removeDependenciesStaticArray(&s.find_collisions, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = if (num_step_listener_jobs > 0) num_step_listener_jobs else previous_step_dependency_count }); // depends on: step listeners (or previous step if no step listeners)

            // This job calls the step listeners
            PhysicsUpdateContext.resizeHandles(&step.step_listeners, num_step_listener_jobs);
            for (step.step_listeners.slice()) |*h|
                h.* = try job_system.createJob("StepListeners", color_step_listeners, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        // Call the step listeners
                        if (!c.isOutOfMemory())
                            c.physics_system.jobStepListeners(s);

                        // Kick apply gravity and determine active constraint jobs
                        JobHandle.removeDependenciesStaticArray(&s.apply_gravity, .{});
                        JobHandle.removeDependenciesStaticArray(&s.determine_active_constraints, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = previous_step_dependency_count });

            // Unblock the previous step
            if (!is_first_step)
                steps[step_idx - 1].start_next_step.removeDependency(.{});

            // This job will finalize the simulation islands
            step.finalize_islands = try job_system.createJob("FinalizeIslands", color_finalize_islands, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Validate that all find collision jobs have stopped
                    std.debug.assert(s.active_find_collision_jobs.load(.monotonic) == 0);

                    if (!c.isOutOfMemory())
                        c.physics_system.jobFinalizeIslands(c) catch c.setOutOfMemory();

                    JobHandle.removeDependenciesStaticArray(&s.solve_velocity_constraints, .{});
                    s.body_set_island_index.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_find_collisions_jobs + 2 }); // depends on: find collisions, build islands from constraints, finish building jobs

            // Unblock previous job
            // Note: technically we could release find collisions here but we don't want to because that could make them run before 'setup velocity constraints' which means that job won't have a thread left
            step.build_islands_from_constraints.removeDependency(.{});

            // This job will call the contact removed callbacks
            step.contact_removed_callbacks = try job_system.createJob("ContactRemovedCallbacks", color_contact_removed_callbacks, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobContactRemovedCallbacks(s);

                    if (s.start_next_step.isValid())
                        s.start_next_step.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 1 }); // depends on the find ccd contacts

            // This job will set the island index on each body (only used for debug drawing purposes)
            // It will also delete any bodies that have been destroyed in the last frame
            step.body_set_island_index = try job_system.createJob("BodySetIslandIndex", color_body_set_island_index, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobBodySetIslandIndex();

                    JobHandle.removeDependenciesStaticArray(&s.solve_position_constraints, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: finalize islands, finish building jobs

            // Job to start the next collision step
            if (!is_last_step) {
                const next_step = &steps[step_idx + 1];
                step.start_next_step = try job_system.createJob("StartNextStep", color_start_next_step, .init(struct {
                    fn call(ps: *Self, ns: *Step) void {
                        const c = ns.context;

                        // Store the number of active bodies at the start of the step
                        ns.num_active_bodies_at_step_start = ps.body_manager.getNumActiveBodies(.rigid_body);

                        // Clear the large island splitter (reduced: no splitter)
                        const temp = c.temp_allocator;

                        // Clear the island builder (Zolt D5: frees always run)
                        ps.island_builder.resetIslands(temp);

                        // Setup island builder (Zolt D5: no allocation once poisoned)
                        if (!c.isOutOfMemory())
                            ps.island_builder.prepareContactConstraints(ps.contact_manager.getMaxConstraints(), temp) catch c.setOutOfMemory();

                        // Restart the contact manager
                        ps.contact_manager.recycleConstraintBuffer();

                        // Kick the jobs of the next step (in the same order as the first step)
                        kickStep(ns);
                    }
                }.call, .{ self, next_step }), .{ .num_dependencies = 3 }); // depends on: update soft bodies, contact removed callbacks, finish building the previous step
            }

            // This job will solve the velocity constraints
            PhysicsUpdateContext.resizeHandles(&step.solve_velocity_constraints, max_concurrency);
            for (step.solve_velocity_constraints.slice()) |*h|
                h.* = try job_system.createJob("SolveVelocityConstraints", color_solve_velocity_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSolveVelocityConstraints(c, s);

                        s.pre_integrate_velocity.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = num_setup_velocity_constraints_jobs + 2 }); // depends on: finalize islands, setup velocity constraints, finish building jobs.

            // We prefer setup velocity constraints to finish first so we kick it first
            JobHandle.removeDependenciesStaticArray(&step.setup_velocity_constraints, .{});
            JobHandle.removeDependenciesStaticArray(&step.find_collisions, .{});

            // Finalize islands is a dependency on find collisions so it can go last
            step.finalize_islands.removeDependency(.{});

            // This job will prepare the position update of all active bodies
            step.pre_integrate_velocity = try job_system.createJob("PreIntegrateVelocity", color_pre_integrate_velocity, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    if (!c.isOutOfMemory())
                        c.physics_system.jobPreIntegrateVelocity(c, s) catch c.setOutOfMemory();

                    JobHandle.removeDependenciesStaticArray(&s.integrate_velocity, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 + max_concurrency }); // depends on: broadphase update finalize, solve velocity constraints, finish building jobs.

            // Unblock previous jobs
            step.update_broadphase_finalize.removeDependency(.{});
            JobHandle.removeDependenciesStaticArray(&step.solve_velocity_constraints, .{});

            // This job will update the positions of all active bodies
            PhysicsUpdateContext.resizeHandles(&step.integrate_velocity, num_integrate_velocity_jobs);
            for (step.integrate_velocity.slice()) |*h|
                h.* = try job_system.createJob("IntegrateVelocity", color_integrate_velocity, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobIntegrateVelocity(c, s);

                        s.post_integrate_velocity.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: pre integrate velocity, finish building jobs.

            // Unblock previous job
            step.pre_integrate_velocity.removeDependency(.{});

            // This job will finish the position update of all active bodies
            step.post_integrate_velocity = try job_system.createJob("PostIntegrateVelocity", color_post_integrate_velocity, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: the job body spawns the CCD jobs (bookkeeping), it checks the latch itself
                    c.physics_system.jobPostIntegrateVelocity(c, s);

                    s.resolve_ccd_contacts.removeDependency(.{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = num_integrate_velocity_jobs + 1 }); // depends on: integrate velocity, finish building jobs

            // Unblock previous jobs
            JobHandle.removeDependenciesStaticArray(&step.integrate_velocity, .{});

            // This job will update the positions and velocities for all bodies that need continuous collision detection
            step.resolve_ccd_contacts = try job_system.createJob("ResolveCCDContacts", color_resolve_ccd_contacts, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: the job body frees the CCD buffers, it checks the latch itself
                    c.physics_system.jobResolveCCDContacts(c, s);

                    JobHandle.removeDependenciesStaticArray(&s.solve_position_constraints, .{});
                }
            }.call, .{ context, step }), .{ .num_dependencies = 2 }); // depends on: integrate velocities, detect ccd contacts (added dynamically), finish building jobs.

            // Unblock previous job
            step.post_integrate_velocity.removeDependency(.{});

            // Fixes up drift in positions and updates the broadphase with new body positions
            PhysicsUpdateContext.resizeHandles(&step.solve_position_constraints, max_concurrency);
            for (step.solve_position_constraints.slice()) |*h|
                h.* = try job_system.createJob("SolvePositionConstraints", color_solve_position_constraints, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobSolvePositionConstraints(c, s);

                        // Kick the next step
                        if (s.soft_body_prepare.isValid())
                            s.soft_body_prepare.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = 3 }); // depends on: resolve ccd contacts, body set island index, finish building jobs.

            // Unblock previous jobs.
            step.resolve_ccd_contacts.removeDependency(.{});
            step.body_set_island_index.removeDependency(.{});

            // The soft body prepare job will create other jobs if needed
            step.soft_body_prepare = try job_system.createJob("SoftBodyPrepare", color_soft_body_prepare, .init(struct {
                fn call(c: *PhysicsUpdateContext, s: *Step) void {
                    // Zolt D5: the job body kicks StartNextStep or spawns the soft body jobs, it checks the latch itself
                    c.physics_system.jobSoftBodyPrepare(c, s);
                }
            }.call, .{ context, step }), .{ .num_dependencies = max_concurrency }); // depends on: solve position constraints.

            // Unblock previous jobs
            JobHandle.removeDependenciesStaticArray(&step.solve_position_constraints, .{});
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Job functions (reduced bodies, Jolt's work distribution)

    fn jobStepListeners(self: *Self, step: *Step) void {
        const context: PhysicsStepListenerContext = .{
            .delta_time = step.context.step_delta_time,
            .is_first_step = step.is_first,
            .is_last_step = step.is_last,
            .physics_system = self,
        };

        const batch_size: u32 = @intCast(self.physics_settings.step_listeners_batch_size);
        while (true) {
            // Get the start of a new batch
            const batch = step.step_listener_read_idx.value.fetchAdd(batch_size, .seq_cst);
            if (batch >= self.step_listeners.items.len)
                break;

            // Call the listeners
            const i_end = @min(@as(u32, @intCast(self.step_listeners.items.len)), batch + batch_size);
            for (self.step_listeners.items[batch..i_end]) |listener|
                listener.onStep(&context);
        }
    }

    fn jobDetermineActiveConstraints(self: *const Self, step: *Step) void {
        const num_constraints = self.constraint_manager.getNumConstraints();
        var active_constraints: [determine_active_constraints_batch_size]*Constraint = undefined; // JPH_STACK_ALLOC with a comptime size

        while (true) {
            // Atomically fetch a batch of constraints
            const constraint_idx = step.determine_active_constraint_read_idx.value.fetchAdd(determine_active_constraints_batch_size, .seq_cst);
            if (constraint_idx >= num_constraints)
                break;

            // Calculate the end of the batch
            const constraint_idx_end = @min(num_constraints, constraint_idx + determine_active_constraints_batch_size);

            // Store the active constraints at the start of the step (bodies get activated during the step which in turn may activate constraints leading to an inconsistent shapshot)
            const num_active_constraints = self.constraint_manager.getActiveConstraints(constraint_idx, constraint_idx_end, &active_constraints);

            // Copy the block of active constraints to the global list of active constraints
            if (num_active_constraints > 0) {
                const active_constraint_idx = step.num_active_constraints.value.fetchAdd(num_active_constraints, .seq_cst);
                @memcpy(step.context.active_constraints.items[active_constraint_idx..][0..num_active_constraints], active_constraints[0..num_active_constraints]);
            }
        }
    }

    fn jobApplyGravity(self: *Self, context: *const PhysicsUpdateContext, step: *Step) void {
        // Get list of active bodies that we had at the start of the physics update.
        // Any body that is activated as part of the simulation step does not receive gravity this frame.
        // Note that bodies may be activated during this job but not deactivated, this means that only elements
        // will be added to the array. Since the array is made to not reallocate, this is a safe operation.
        const active_bodies = self.body_manager.getActiveBodiesUnsafe(.rigid_body);
        const num_active_bodies_at_step_start = step.num_active_bodies_at_step_start;

        // Fetch delta time once outside the loop
        const delta_time = context.step_delta_time;

        // Update velocities from forces
        while (true) {
            // Atomically fetch a batch of bodies
            var active_body_idx = step.apply_gravity_read_idx.value.fetchAdd(apply_gravity_batch_size, .seq_cst);
            if (active_body_idx >= num_active_bodies_at_step_start)
                break;

            // Calculate the end of the batch
            const active_body_idx_end = @min(num_active_bodies_at_step_start, active_body_idx + apply_gravity_batch_size);

            // Process the batch
            while (active_body_idx < active_body_idx_end) : (active_body_idx += 1) {
                const body = self.body_manager.getBody(active_bodies[active_body_idx]);
                if (body.isDynamic()) {
                    const mp = body.getMotionProperties();
                    const rotation = body.getRotation();
                    mp.applyForceTorqueAndDragInternal(rotation, self.gravity, delta_time);
                }
            }
        }
    }

    fn jobSetupVelocityConstraints(self: *const Self, delta_time: f32, step: *Step) void {
        _ = self;
        const num_constraints = step.num_active_constraints.value.load(.seq_cst);

        while (true) {
            // Atomically fetch a batch of constraints
            const constraint_idx = step.setup_velocity_constraints_read_idx.value.fetchAdd(setup_velocity_constraints_batch_size, .seq_cst);
            if (constraint_idx >= num_constraints)
                break;

            ConstraintManager.setupVelocityConstraints(step.context.active_constraints.items[constraint_idx..][0..@min(setup_velocity_constraints_batch_size, num_constraints - constraint_idx)], delta_time);
        }
    }

    fn jobBuildIslandsFromConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) TempAllocator.Error!void {
        const num_active_constraints = step.num_active_constraints.value.load(.seq_cst);

        // Prepare the island builder
        try self.island_builder.prepareNonContactConstraints(num_active_constraints, context.temp_allocator);

        // Build the islands
        ConstraintManager.buildIslands(context.active_constraints.items[0..num_active_constraints], &self.island_builder, &self.body_manager);
    }

    /// Spawn an extra FindCollisions job when there is enough work. Zolt (D5): a failed createJob is not an error, the
    /// running jobs do the work (the claim and the dependencies are undone).
    fn trySpawnJobFindCollisions(self: *const Self, step: *Step) void {
        // Get how many jobs we can spawn and check if we can spawn more
        const max_jobs = step.body_pair_queues.len;
        if (@popCount(step.active_find_collision_jobs.load(.monotonic)) >= max_jobs)
            return;

        // Count how many body pairs we have waiting
        var num_body_pairs: u32 = 0;
        for (step.body_pair_queues.constSlice()) |*queue|
            num_body_pairs +%= queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);

        // Count how many active bodies we have waiting
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body) -% step.active_body_read_idx.value.load(.seq_cst);

        // Calculate how many jobs we would like
        const desired_num_jobs = @min((num_body_pairs + narrow_phase_batch_size - 1) / narrow_phase_batch_size + (num_active_bodies +% (active_bodies_batch_size - 1)) / active_bodies_batch_size, max_jobs);

        while (true) {
            // Get the bit mask of active jobs and see if we can spawn more
            const current_active_jobs = step.active_find_collision_jobs.load(.monotonic);
            const job_index: u32 = @ctz(~current_active_jobs);
            if (job_index >= desired_num_jobs)
                break;

            // Try to claim the job index
            const job_mask = @as(JobMask, 1) << @intCast(job_index);
            const prev_value = step.active_find_collision_jobs.fetchOr(job_mask, .acquire);
            if ((prev_value & job_mask) == 0) {
                // Add dependencies from the find collisions job to the next jobs
                step.update_broadphase_finalize.addDependency(.{});
                step.finalize_islands.addDependency(.{});

                // Start the job
                var job = step.context.job_system.createJob("FindCollisions", color_find_collisions, .init(struct {
                    fn call(s: *Step, ji: u32) void {
                        s.context.physics_system.jobFindCollisions(s, ji);
                    }
                }.call, .{ step, job_index }), .{}) catch {
                    // Zolt (D5): not an error, undo the claim and the dependencies, the running jobs do the work
                    _ = step.active_find_collision_jobs.fetchAnd(~job_mask, .release);
                    step.update_broadphase_finalize.removeDependency(.{});
                    step.finalize_islands.removeDependency(.{});
                    return;
                };
                defer job.deinit();
                _ = step.context.physics_system.debug_num_spawned_jobs.fetchAdd(1, .monotonic); // self is const (Rule M): through the context

                // Add the job to the job barrier so the main updating thread can execute the job too
                step.context.barrier.addJob(&job);

                // Spawn only 1 extra job at a time
                return;
            }
        }
    }

    /// Collect the contact allocator's counters into the step (sFinalizeContactAllocator)
    fn finalizeContactAllocator(step: *Step, contact_allocator: *const ContactAllocator) void {
        // Atomically accumulate the number of found manifolds and body pairs
        _ = step.num_body_pairs.fetchAdd(contact_allocator.num_body_pairs, .monotonic);
        _ = step.num_manifolds.fetchAdd(contact_allocator.num_manifolds, .monotonic);

        // Combine update errors
        _ = step.context.errors.fetchOr(@bitCast(contact_allocator.errors), .monotonic);
    }

    /// Zolt (D5): the job always runs its exit protocol. Once poisoned it takes no new batch or pair and leaves through
    /// Jolt's "found nothing" exit, which finalizes the ContactAllocator, clears the mask bit and removes dependencies.
    fn jobFindCollisions(self: *Self, step: *Step, job_index: u32) void {
        const context = step.context;

        // Allocation context for allocating new contact points
        var contact_allocator = self.contact_manager.getContactAllocator();

        // Determine initial queue to read pairs from if no broadphase work can be done
        // (always start looking at results from the next job)
        var read_queue_idx = (job_index + 1) % step.body_pair_queues.len;

        // Allocate space to temporarily store a batch of active bodies
        var active_bodies: [active_bodies_batch_size]BodyID = undefined;

        while (true) {
            // Zolt (D5): no new work after an out of memory error
            const poisoned = context.isOutOfMemory();

            // Check if there are active bodies to be processed
            const active_bodies_read_idx = step.active_body_read_idx.value.load(.seq_cst);
            const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
            if (!poisoned and active_bodies_read_idx < num_active_bodies) {
                // Take a batch of active bodies
                const active_bodies_read_idx_end = @min(num_active_bodies, active_bodies_read_idx + active_bodies_batch_size);
                if (step.active_body_read_idx.value.cmpxchgStrong(active_bodies_read_idx, active_bodies_read_idx_end, .seq_cst, .seq_cst) == null) {
                    // Callback when a new body pair is found
                    const MyBodyPairCallback = struct {
                        pub const overrides = .{.addHit};

                        base: BodyPairCollector,
                        step: *Step,
                        contact_allocator: *ContactAllocator,
                        job_index: u32,

                        // Callback function when a body pair is found
                        pub fn addHit(cb: *@This(), pair: *const BodyPair) void {
                            // Check if we have space in our write queue
                            const queue = &cb.step.body_pair_queues.slice()[cb.job_index];
                            const body_pairs_in_queue = queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);
                            if (body_pairs_in_queue >= cb.step.max_body_pairs_per_queue) {
                                // Buffer full, process the pair now
                                cb.step.context.physics_system.processBodyPair(cb.contact_allocator, pair.*) catch cb.step.context.setOutOfMemory();
                            } else {
                                // Store the pair in our own queue
                                storeBodyPair(&cb.step.context.body_pairs.items[cb.job_index * cb.step.max_body_pairs_per_queue + queue.write_idx.load(.monotonic) % cb.step.max_body_pairs_per_queue], pair.*);
                                _ = queue.write_idx.fetchAdd(1, .seq_cst);
                            }
                        }
                    };
                    var add_pair: MyBodyPairCallback = .{ .base = .init(MyBodyPairCallback), .step = step, .contact_allocator = &contact_allocator, .job_index = job_index };

                    // Copy active bodies to temporary array, broadphase will reorder them
                    const batch_size = active_bodies_read_idx_end - active_bodies_read_idx;
                    @memcpy(active_bodies[0..batch_size], self.body_manager.getActiveBodiesUnsafe(.rigid_body)[active_bodies_read_idx..active_bodies_read_idx_end]);

                    // Find pairs in the broadphase
                    self.broad_phase.findCollidingPairs(active_bodies[0..batch_size], self.physics_settings.speculative_contact_distance, self.object_vs_broad_phase_layer_filter, self.object_layer_pair_filter, &add_pair.base);

                    // Check if we have enough pairs in the buffer to start a new job
                    const queue = &step.body_pair_queues.slice()[job_index];
                    const body_pairs_in_queue = queue.write_idx.load(.seq_cst) -% queue.read_idx.load(.seq_cst);
                    if (body_pairs_in_queue >= narrow_phase_batch_size)
                        self.trySpawnJobFindCollisions(step);
                }
            } else {
                // Lockless loop to get the next body pair from the pairs buffer
                const first_read_queue_idx = read_queue_idx;
                while (true) {
                    const queue = &step.body_pair_queues.slice()[read_queue_idx];

                    // Get the next pair to process
                    const pair_idx = queue.read_idx.load(.seq_cst);

                    // If the pair hasn't been written yet (Zolt: a poisoned job treats every queue as empty)
                    if (poisoned or pair_idx >= queue.write_idx.load(.seq_cst)) {
                        // Go to the next queue
                        read_queue_idx = (read_queue_idx + 1) % step.body_pair_queues.len;

                        // If we're back at the first queue, we've looked at all of them and found nothing
                        if (read_queue_idx == first_read_queue_idx) {
                            // Collect information from the contact allocator and accumulate it in the step.
                            finalizeContactAllocator(step, &contact_allocator);

                            // Mark this job as inactive
                            _ = step.active_find_collision_jobs.fetchAnd(~(@as(JobMask, 1) << @intCast(job_index)), .release);

                            // Trigger the next jobs
                            step.update_broadphase_finalize.removeDependency(.{});
                            step.finalize_islands.removeDependency(.{});
                            return;
                        }

                        // Try again reading from the next queue
                        continue;
                    }

                    // Copy the body pair out of the buffer
                    const bp = loadBodyPair(&context.body_pairs.items[read_queue_idx * step.max_body_pairs_per_queue + pair_idx % step.max_body_pairs_per_queue]);

                    // Mark this pair as taken
                    if (queue.read_idx.cmpxchgStrong(pair_idx, pair_idx + 1, .seq_cst, .seq_cst) == null) {
                        // Process the actual body pair
                        self.processBodyPair(&contact_allocator, bp) catch context.setOutOfMemory();
                        break;
                    }
                }
            }
        }
    }

    /// Process narrow phase for a single body pair (reduced: one closest hit, no manifold, no cache). Zolt: the real
    /// narrow phase allocates (InternalEdgeRemovingCollector heap fallback), so Allocator.Error.
    fn processBodyPair(self: *Self, contact_allocator: *ContactAllocator, body_pair: BodyPair) Allocator.Error!void {
        // Fetch body pair
        var body1 = self.body_manager.getBody(body_pair.body_a);
        var body2 = self.body_manager.getBody(body_pair.body_b);
        std.debug.assert(body1.isActive());

        // Ensure that body1 has the higher motion type (i.e. dynamic trumps kinematic), this ensures that we do the collision detection in the space of a moving body,
        // which avoids accuracy problems when testing a very large static object against a small dynamic object
        // Ensure that body1 id < body2 id when motion types are the same.
        if (@intFromEnum(body1.getMotionType()) < @intFromEnum(body2.getMotionType()) or (body1.getMotionType() == body2.getMotionType() and body_pair.body_b.lessThan(body_pair.body_a)))
            std.mem.swap(*Body, &body1, &body2);

        // Check if the contact points from the previous frame are reusable and if so copy them (reduced: no cache)
        contact_allocator.num_body_pairs += 1;

        // Get transforms relative to body1
        const offset = body1.getCenterOfMassPosition();
        const transform1 = Mat44.rotationQuat(body1.getRotation());
        const transform2 = body2.getCenterOfMassTransform().postTranslatedRVec3(offset.negate()).toMat44();

        var settings: CollideShapeSettings = .{};
        settings.max_separation_distance = self.physics_settings.speculative_contact_distance;

        var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), transform1, transform2, .{}, .{}, &settings, &collector.base, &.{});

        if (collector.had_hit) {
            contact_allocator.num_manifolds += 1;
            const normal = collector.hit.penetration_axis.normalized();
            var activate_and_link_bodies = true;
            self.contact_manager.addContactConstraint(contact_allocator, &activate_and_link_bodies, body1, body2, normal, collector.hit.penetration_depth);
        }
    }

    fn jobFinalizeIslands(self: *Self, context: *PhysicsUpdateContext) TempAllocator.Error!void {
        // Finish collecting the islands, at this point the active body list doesn't change so it's safe to access
        const num_active = self.body_manager.getNumActiveBodies(.rigid_body);
        try self.island_builder.finalize(self.body_manager.getActiveBodiesUnsafe(.rigid_body)[0..num_active], self.contact_manager.getNumConstraints(), context.temp_allocator);

        // Prepare the large island splitter (reduced: no splitter)
    }

    fn jobBodySetIslandIndex(self: *Self) void {
        // Loop through the result and tag all bodies with an island index
        for (0..self.island_builder.getNumIslands()) |island_idx| {
            for (self.island_builder.getBodiesInIsland(@intCast(island_idx))) |body|
                self.body_manager.getBody(body).getMotionProperties().island_index = @intCast(island_idx);
        }
    }

    fn jobSolveVelocityConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        const delta_time = context.step_delta_time;
        const active_constraints = context.active_constraints.items;

        // Only the first step to correct for the delta time difference in the previous update
        const warm_start_impulse_ratio: f32 = if (self.physics_settings.constraint_warm_start) (if (step.is_first) context.warm_start_impulse_ratio else 1.0) else 0.0;

        // Reduced: no large island splitter (Jolt first fetches batches of split islands here)
        var check_islands = true;
        while (true) {
            // If that didn't succeed try to process an island
            if (check_islands) {
                // Next island
                const island_idx = step.solve_velocity_constraints_next_island.fetchAdd(1, .seq_cst);
                if (island_idx >= self.island_builder.getNumIslands()) {
                    // We processed all islands, stop checking islands
                    check_islands = false;
                    continue;
                }

                // Get iterators for this island
                const constraints = self.island_builder.getConstraintsInIsland(island_idx);
                const contacts = self.island_builder.getContactsInIsland(island_idx);

                // If we don't have any contacts or constraints, we know that none of the following islands have any contacts or constraints
                // (because they're sorted by most constraints first). This means we're done.
                if (contacts.len == 0 and constraints.len == 0) {
                    check_islands = false;
                    continue;
                }

                // Sort constraints to give a deterministic simulation
                ConstraintManager.sortConstraints(active_constraints, constraints);

                // Sort contacts to give a deterministic simulation
                self.contact_manager.sortContacts(contacts);

                // We didn't create a split, just run the solver now for this entire island. Begin by warm starting.
                ConstraintManager.warmStartVelocityConstraints(active_constraints, constraints, warm_start_impulse_ratio);

                // Store the number of position steps for later
                self.island_builder.setNumPositionSteps(island_idx, self.physics_settings.num_position_steps);

                // Solve velocity constraints
                for (0..self.physics_settings.num_velocity_steps) |_| {
                    var applied_impulse = ConstraintManager.solveVelocityConstraints(active_constraints, constraints, delta_time);
                    applied_impulse = self.contact_manager.solveVelocityConstraints(contacts, delta_time) or applied_impulse;
                    if (!applied_impulse)
                        break;
                }
            } else {
                // No more work
                break;
            }
        }
    }

    fn jobPreIntegrateVelocity(self: *Self, context: *PhysicsUpdateContext, step: *Step) TempAllocator.Error!void {
        // Reserve enough space for all bodies that may need a cast
        const temp_allocator = context.temp_allocator;
        std.debug.assert(step.ccd_bodies.isEmpty());
        step.ccd_bodies = try .alloc(temp_allocator, self.body_manager.getNumActiveCCDBodies());

        // Initialize the mapping table between active body and CCD body
        std.debug.assert(step.active_body_to_ccd_body.isEmpty());
        step.active_body_to_ccd_body = try .alloc(temp_allocator, self.body_manager.getNumActiveBodies(.rigid_body));

        // Prepare the split island builder for solving the position constraints (reduced: no splitter)
    }

    fn jobIntegrateVelocity(self: *Self, context: *const PhysicsUpdateContext, step: *Step) void {
        const delta_time = context.step_delta_time;
        const active_bodies = self.body_manager.getActiveBodiesUnsafe(.rigid_body);
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);

        while (true) {
            // Atomically fetch a batch of bodies
            var active_body_idx = step.integrate_velocity_read_idx.fetchAdd(integrate_velocity_batch_size, .seq_cst);
            if (active_body_idx >= num_active_bodies)
                break;

            // Calculate the end of the batch
            const active_body_idx_end = @min(num_active_bodies, active_body_idx + integrate_velocity_batch_size);

            // Process the batch
            while (active_body_idx < active_body_idx_end) : (active_body_idx += 1) {
                // Update the positions using an Symplectic Euler step (which integrates using the updated velocity v1' rather
                // than the original velocity v1):
                // x1' = x1 + h * v1'
                // At this point the active bodies list does not change, so it is safe to access the array.
                const body_id = active_bodies[active_body_idx];
                const body = self.body_manager.getBody(body_id);
                const mp = body.getMotionProperties();

                if (DeterminismLog.enabled) DeterminismLog.log(.{ "JobIntegrateVelocity: id: ", body_id, " v: ", body.getLinearVelocity(), " w: ", body.getAngularVelocity() });

                // Clamp velocities (not for kinematic bodies)
                if (body.isDynamic()) {
                    mp.clampLinearVelocity();
                    mp.clampAngularVelocity();
                }

                // Update the rotation of the body according to the angular velocity
                body.addRotationStep(body.getAngularVelocity().mulScalar(delta_time));

                // Get delta position
                const delta_pos = body.getLinearVelocity().mulScalar(delta_time);

                // If the position should be updated (or if it is delayed because of CCD)
                var update_position = true;

                switch (mp.getMotionQuality()) {
                    .discrete => {
                        // No additional collision checking to be done
                    },
                    .linear_cast => {
                        if (body.isDynamic() // Kinematic bodies cannot be stopped
                        and !body.isSensor()) // We don't support CCD sensors
                        {
                            // Determine inner radius (the smallest sphere that fits into the shape)
                            const inner_radius = body.getShape().getInnerRadius();
                            std.debug.assert(inner_radius > 0.0); // The shape has no inner radius, this makes the shape unsuitable for the linear cast motion quality as we cannot move it without risking tunneling.

                            // Measure translation in this step and check if it above the threshold to perform a linear cast
                            const linear_cast_threshold_sq = math.square(self.physics_settings.linear_cast_threshold * inner_radius);
                            if (delta_pos.lengthSq() > linear_cast_threshold_sq) {
                                // This body needs a cast
                                const ccd_body_idx = step.num_ccd_bodies.fetchAdd(1, .seq_cst);
                                std.debug.assert(active_body_idx < step.active_body_to_ccd_body.items.len);
                                step.active_body_to_ccd_body.items[active_body_idx] = @intCast(ccd_body_idx);
                                step.ccd_bodies.items[ccd_body_idx] = .init(body_id, delta_pos, linear_cast_threshold_sq, math.min(self.physics_settings.penetration_slop, self.physics_settings.linear_cast_max_penetration * inner_radius));

                                update_position = false;
                            }
                        }
                    },
                }

                if (update_position) {
                    // Move the body now
                    body.addPositionStep(delta_pos);

                    // We did not create a CCD body
                    step.active_body_to_ccd_body.items[active_body_idx] = -1;
                }
            }
        }
    }

    /// Zolt (D5): the job always runs. Once poisoned it takes the "no CCD bodies" path. A FindCCDContacts job that
    /// cannot be created is not an error: the created jobs process every CCD body (they take bodies until mNextCCDBody
    /// runs out), and when none could be created this job does the work itself.
    fn jobPostIntegrateVelocity(self: *const Self, context: *PhysicsUpdateContext, step: *Step) void {
        // Validate that our reservations were correct
        std.debug.assert(step.num_ccd_bodies.load(.monotonic) <= self.body_manager.getNumActiveCCDBodies());

        const num_ccd_bodies = step.num_ccd_bodies.load(.monotonic);
        if (num_ccd_bodies == 0 or context.isOutOfMemory()) {
            // No continuous collision detection jobs -> kick the next job ourselves
            step.contact_removed_callbacks.removeDependency(.{});
        } else {
            // Run the continuous collision detection jobs
            const num_continuous_collision_jobs: u32 = @min((num_ccd_bodies + num_ccd_bodies_per_job - 1) / num_ccd_bodies_per_job, context.getMaxConcurrency());
            step.resolve_ccd_contacts.addDependency(.{ .count = @intCast(num_continuous_collision_jobs) });
            step.contact_removed_callbacks.addDependency(.{ .count = @intCast(num_continuous_collision_jobs - 1) }); // Already had 1 dependency
            for (0..num_continuous_collision_jobs) |i| {
                var job = context.job_system.createJob("FindCCDContacts", color_find_ccd_contacts, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobFindCCDContacts(c, s) catch c.setOutOfMemory();

                        s.resolve_ccd_contacts.removeDependency(.{});
                        s.contact_removed_callbacks.removeDependency(.{});
                    }
                }.call, .{ context, step }), .{}) catch {
                    // Zolt (D5): not an error, see the function comment
                    if (i == 0)
                        context.physics_system.jobFindCCDContacts(context, step) catch context.setOutOfMemory();
                    const num_missing: i32 = @intCast(num_continuous_collision_jobs - i);
                    step.resolve_ccd_contacts.removeDependency(.{ .count = num_missing });
                    step.contact_removed_callbacks.removeDependency(.{ .count = num_missing });
                    return;
                };
                defer job.deinit();
                context.barrier.addJob(&job);
            }
        }
    }

    /// Find the first hit of the CCD bodies (reduced: the body's inner sphere is cast against the plane y = 0, the top of
    /// the floor, instead of the broad phase + CastShape). Allocator.Error like the real cast (collectors).
    fn jobFindCCDContacts(self: *Self, context: *const PhysicsUpdateContext, step: *Step) Allocator.Error!void {
        _ = context;
        while (true) {
            // Fetch the next body to cast
            const idx = step.next_ccd_body.fetchAdd(1, .seq_cst);
            if (idx >= step.num_ccd_bodies.load(.seq_cst))
                break;
            const ccd_body = &step.ccd_bodies.items[idx];
            const body = self.body_manager.getBody(ccd_body.body_id1);

            const radius = body.getShape().getInnerRadius();
            const start: f32 = @as(f32, @floatCast(body.getCenterOfMassPosition().getY())) - radius;
            const delta = ccd_body.delta_position.getY();
            if (delta < 0.0 and start >= 0.0 and start + delta < 0.0) {
                ccd_body.fraction = start / -delta;
                ccd_body.fraction_plus_slop = ccd_body.fraction;
                ccd_body.contact_normal = Vec3.init(0, 1, 0);
            }
        }
    }

    /// Zolt (D5): the job always runs, the work is skipped when poisoned, the frees at the end always happen
    fn jobResolveCCDContacts(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        const temp_allocator = context.temp_allocator;

        // Check if there's anything to do
        const num_ccd_bodies = step.num_ccd_bodies.load(.monotonic);
        if (num_ccd_bodies > 0 and !context.isOutOfMemory())
            self.resolveCCDContacts(temp_allocator, step, num_ccd_bodies) catch context.setOutOfMemory();

        // Ensure we free the CCD bodies array now, will not call the destructor!
        step.active_body_to_ccd_body.free(temp_allocator);
        step.ccd_bodies.free(temp_allocator);
    }

    fn resolveCCDContacts(self: *Self, temp_allocator: TempAllocator, step: *Step, num_ccd_bodies: u32) TempAllocator.Error!void {
        // Sort on fraction so that we process earliest collisions first
        // This is needed to make the simulation deterministic and also to be able to stop contact processing
        // between body pairs if an earlier hit was found involving the body by another CCD body
        // (if it's body ID < this CCD body's body ID - see filtering logic in CCDBroadPhaseCollector)
        var sorted_ccd_bodies = try TempArray(*CCDBody).alloc(temp_allocator, num_ccd_bodies);
        defer sorted_ccd_bodies.free(temp_allocator); // JPH_SCOPE_EXIT
        for (sorted_ccd_bodies.items, step.ccd_bodies.items[0..num_ccd_bodies]) |*dst, *src| dst.* = src;
        zolt.quickSort(*CCDBody, sorted_ccd_bodies.items, {}, struct {
            fn lessThan(_: void, body1: *CCDBody, body2: *CCDBody) bool {
                if (body1.fraction_plus_slop != body2.fraction_plus_slop)
                    return body1.fraction_plus_slop < body2.fraction_plus_slop;
                return body1.body_id1.lessThan(body2.body_id1);
            }
        }.lessThan);

        // We can collide with bodies that are not active, we track them here so we can activate them in one go at the end (reduced: the plane is static)
        for (sorted_ccd_bodies.items) |ccd_body| {
            const body1 = self.body_manager.getBody(ccd_body.body_id1);

            // If there was a hit (reduced sSolveCCDContact: remove the velocity into the contact normal)
            if (ccd_body.fraction_plus_slop < 1.0) {
                const mp = body1.getMotionProperties();
                const normal_velocity = mp.linear_velocity.dot(ccd_body.contact_normal);
                if (normal_velocity < 0.0)
                    mp.linear_velocity = mp.linear_velocity.sub(ccd_body.contact_normal.mulScalar(normal_velocity));
            }

            // Update body position
            body1.addPositionStep(ccd_body.delta_position.mulScalar(ccd_body.fraction_plus_slop));
        }
        _ = self.debug_num_ccd_bodies.fetchAdd(num_ccd_bodies, .monotonic);
    }

    fn jobContactRemovedCallbacks(self: *Self, step: *const Step) void {
        // Finalize the contact cache (this swaps the read and write versions of the contact cache)
        // Trigger all contact removed callbacks by looking at last step contact points that have not been flagged as reused
        self.contact_manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(step.num_body_pairs.load(.monotonic), step.num_manifolds.load(.monotonic));
    }

    /// Update the bounds of the bodies of an island and notify the broad phase (reduced: no sleeping)
    fn checkSleepAndUpdateBounds(self: *Self, island_index: u32, step: *const Step) void {
        // Get the bodies that belong to this island
        const bodies = self.island_builder.getBodiesInIsland(island_index);

        // Only check sleeping in the last step
        // Also resets force and torque used during the apply gravity phase
        if (step.is_last) {
            for (bodies) |body_id| {
                const body = self.body_manager.getBody(body_id);

                // Update bounding box
                body.calculateWorldSpaceBoundsInternal();

                // Reset force and torque
                body.getMotionProperties().resetForce();
            }
        } else {
            // Update bounding box only for all other steps
            for (bodies) |body_id|
                self.body_manager.getBody(body_id).calculateWorldSpaceBoundsInternal();
        }

        // Notify broadphase of changed objects
        // Note: Shuffles the BodyID's around!!!
        self.broad_phase.notifyBodiesAABBChanged(bodies, .{ .take_lock = false });
    }

    fn jobSolvePositionConstraints(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        const delta_time = context.step_delta_time;
        const baumgarte = self.physics_settings.baumgarte;
        const active_constraints = context.active_constraints.items;

        // Reduced: no large island splitter
        var check_islands = true;
        while (true) {
            if (check_islands) {
                // Next island
                const island_idx = step.solve_position_constraints_next_island.fetchAdd(1, .seq_cst);
                if (island_idx >= self.island_builder.getNumIslands()) {
                    // We processed all islands, stop checking islands
                    check_islands = false;
                    continue;
                }

                // Get iterators for this island
                const constraints = self.island_builder.getConstraintsInIsland(island_idx);
                const contacts = self.island_builder.getContactsInIsland(island_idx);

                // Check if this island needs solving
                const num_items = constraints.len + contacts.len;
                if (num_items > 0) {
                    // Iterate (reduced: contacts have no position solver)
                    for (0..self.island_builder.getNumPositionSteps(island_idx)) |_| {
                        var applied_impulse = false;
                        for (constraints) |idx| {
                            if (active_constraints[idx].solvePositionConstraint(delta_time, baumgarte))
                                applied_impulse = true;
                        }
                        if (!applied_impulse)
                            break;
                    }
                }

                // After solving we will update all bounds and check sleeping
                self.checkSleepAndUpdateBounds(island_idx, step);
            } else {
                // No more work
                break;
            }
        }
    }

    /// Zolt (D5): the job always runs, its "no soft bodies" path kicks the next step
    fn jobSoftBodyPrepare(self: *Self, context: *PhysicsUpdateContext, step: *Step) void {
        {
            // Get the active soft bodies (Zolt: the copy allocates; once poisoned the list stays empty)
            var active_bodies: BodyIDVector = .empty;
            defer active_bodies.deinit(self.allocator);
            if (!context.isOutOfMemory())
                self.body_manager.getActiveBodies(.soft_body, &active_bodies) catch context.setOutOfMemory();

            // Quit if there are no active soft bodies
            if (active_bodies.items.len == 0) {
                // Kick the next step
                if (step.start_next_step.isValid())
                    step.start_next_step.removeDependency(.{});
                return;
            }
        }

        // Phase 9: sort, allocate the SoftBodyUpdateContexts and create the SoftBodyFinalize / Simulate / Collide jobs
        // (no soft body can exist before Phase 9)
        @panic("soft bodies: Phase 9");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // State

    /// Saving state for replay (allocates sorted arrays in BodyManager / ContactConstraintManager: Allocator.Error)
    pub fn saveState(self: *const Self, stream: *StateRecorder, state: StateRecorderState, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        stream.write(state);

        if (state.global) {
            stream.write(self.previous_step_delta_time);
            stream.write(self.gravity);
        }

        if (state.bodies)
            try self.body_manager.saveState(stream, filter);

        // Reduced: Jolt saves the contact cache here (ContactConstraintManager::SaveState)

        if (state.constraints)
            self.constraint_manager.saveState(stream);
    }

    /// Restoring state for replay. Returns false if failed.
    pub fn restoreState(self: *Self, stream: *StateRecorder) Allocator.Error!bool {
        var state: StateRecorderState = .all; // Set this value for validation. If a partial state is saved, validation will not work anyway.
        stream.read(&state);

        if (state.global) {
            stream.read(&self.previous_step_delta_time);
            stream.read(&self.gravity);
        }

        if (state.bodies) {
            if (!try self.body_manager.restoreState(stream))
                return false;

            // Update bounding boxes for all bodies in the broadphase
            if (stream.isLastPart()) {
                var bodies: std.ArrayList(BodyID) = .empty;
                defer bodies.deinit(self.allocator);
                for (self.body_manager.getBodies()) |b| {
                    if (b.isValidBodyPointer() and b.getBody().isInBroadPhase())
                        try bodies.append(self.allocator, b.getBody().getID());
                }
                if (bodies.items.len != 0)
                    self.broad_phase.notifyBodiesAABBChanged(bodies.items, .{});
            }
        }

        if (state.constraints) {
            if (!self.constraint_manager.restoreState(stream))
                return false;
        }

        return true;
    }
};
