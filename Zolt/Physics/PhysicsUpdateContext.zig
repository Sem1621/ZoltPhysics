//! Port of: Jolt/Physics/PhysicsUpdateContext.h, Jolt/Physics/PhysicsUpdateContext.cpp
//! Status: complete
//!
//! SimulationArchitecture D4 / D5:
//! - The context lives on the stack of PhysicsSystem.update; the steps are a `TempArray(Step)` (Jolt:
//!   `Array<Step, STLTempAllocator<Step>>`, resized once and never reallocated), the first temp allocation of the update
//!   and the last one freed. The job closures capture pointers to the context and the steps.
//! - Temp memory is only rvector_alignment aligned, so the padded atomics of Step are extern structs with explicit
//!   padding (`PaddedAtomicU32`, `BodyPairQueue`) instead of `align(Core.cache_line_size)`.
//! - `mBodyPairs` is `body_pairs: TempArray(BodyPairSlot)`: Jolt writes and reads a plain BodyPair while other jobs may
//!   read the slot before their compare-exchange fails (TSAN suppressed in Jolt); Zolt stores the 8 bytes as one
//!   `.monotonic` atomic (`storeBodyPair` / `loadBodyPair`), which is defined and has the same bits.
//! - `mCCDBodies` + `mCCDBodiesCapacity` and `mActiveBodyToCCDBody` + `mNumActiveBodyToCCDBody` are TempArrays (the
//!   capacity is the length), `mSoftBodyUpdateContexts` + `mNumSoftBodies` too.
//! - Zolt addition (D5): the out of memory latch `out_of_memory` with `setOutOfMemory` / `isOutOfMemory`.
//! - The constructor `PhysicsUpdateContext(TempAllocator &)` is the struct literal in PhysicsSystem.update; the
//!   destructor is `deinit` (it releases the job handles of the steps and frees the steps).

const std = @import("std");
const Core = @import("../Core/Core.zig");
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const JobSystemFile = @import("../Core/JobSystem.zig");
const JobSystem = JobSystemFile.JobSystem;
const JobHandle = JobSystem.JobHandle;
const TempAllocatorFile = @import("../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const RVec3 = @import("../Math/Real.zig").RVec3;
const BodyID = @import("Body/BodyID.zig").BodyID;
const BodyPair = @import("Body/BodyPair.zig").BodyPair;
const BodyManager = @import("Body/BodyManager.zig").BodyManager;
const SubShapeID = @import("Collision/Shape/SubShapeID.zig").SubShapeID;
const ContactSettings = @import("Collision/ContactListener.zig").ContactSettings;
const BroadPhase = @import("Collision/BroadPhase/BroadPhase.zig").BroadPhase;
const Constraint = @import("Constraints/Constraint.zig").Constraint;
const SoftBodyUpdateContext = @import("SoftBody/SoftBodyUpdateContext.zig").SoftBodyUpdateContext;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const PhysicsSystem = @import("PhysicsSystem.zig").PhysicsSystem;

/// @cond INTERNAL
/// Internal information used during the PhysicsSystem::Update call
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const PhysicsUpdateContext = struct {
    /// Maximum supported amount of concurrent jobs
    pub const max_concurrency = 32;

    pub const JobHandleArray = StaticArray(JobHandle, max_concurrency);

    pub const BodyPairQueue = extern struct {
        /// Next index to write in mBodyPair array (need to add thread index * mMaxBodyPairsPerQueue and modulo mMaxBodyPairsPerQueue)
        write_idx: std.atomic.Value(u32) = .init(0),
        /// Moved to own cache line to avoid conflicts with consumer jobs
        padding1: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
        /// Next index to read in mBodyPair array (need to add thread index * mMaxBodyPairsPerQueue and modulo mMaxBodyPairsPerQueue)
        read_idx: std.atomic.Value(u32) = .init(0),
        /// Moved to own cache line to avoid conflicts with producer/consumer jobs
        padding2: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
    };

    pub const BodyPairQueues = StaticArray(BodyPairQueue, max_concurrency);

    /// A mask that has as many bits as we can have concurrent jobs
    pub const JobMask = u32;

    comptime {
        std.debug.assert(@bitSizeOf(JobMask) >= max_concurrency);
    }

    /// An atomic<uint32> followed by `uint8 mPaddingN[JPH_CACHE_LINE_SIZE - sizeof(atomic<uint32>)]` (Padding to avoid
    /// sharing cache line with the next atomic). Extern so the padding stays behind the atomic; no align(64): the steps
    /// live in temp memory, which is only rvector_alignment aligned.
    pub const PaddedAtomicU32 = extern struct {
        value: std.atomic.Value(u32) = .init(0),
        padding: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
    };

    /// A slot of the body pair queues (Jolt: BodyPair), see the file comment
    pub const BodyPairSlot = std.atomic.Value(u64);

    /// Write a body pair to a slot of the body pair queues
    pub fn storeBodyPair(slot: *BodyPairSlot, pair: BodyPair) void {
        slot.store(std.mem.bytesToValue(u64, std.mem.asBytes(&pair)), .monotonic);
    }

    /// Read a body pair from a slot of the body pair queues
    pub fn loadBodyPair(slot: *const BodyPairSlot) BodyPair {
        const raw = slot.load(.monotonic);
        return std.mem.bytesToValue(BodyPair, std.mem.asBytes(&raw));
    }

    /// Structure that contains data needed for each collision step.
    pub const Step = struct {
        /// Contains the information needed to cast a body through the scene to do continuous collision detection
        pub const CCDBody = struct {
            /// Desired position step
            delta_position: Vec3,
            /// World space normal of closest hit (only valid if mFractionPlusSlop < 1)
            contact_normal: Vec3 = undefined,
            /// World space contact point on body 2 of closest hit (only valid if mFractionPlusSlop < 1)
            contact_point_on2: RVec3 = undefined,
            /// Body 1 (the body that is performing collision detection)
            body_id1: BodyID,
            /// Body 2 (the body of the closest hit, only valid if mFractionPlusSlop < 1)
            body_id2: BodyID = .invalid,
            /// Sub shape of body 2 that was hit (only valid if mFractionPlusSlop < 1)
            sub_shape_id2: SubShapeID = .{},
            /// Fraction at which the hit occurred
            fraction: f32 = 1.0,
            /// Fraction at which the hit occurred + extra delta to allow body to penetrate by mMaxPenetration
            fraction_plus_slop: f32 = 1.0,
            /// Maximum allowed squared movement before doing a linear cast (determined by inner radius of shape)
            linear_cast_threshold_sq: f32,
            /// Maximum allowed penetration (determined by inner radius of shape)
            max_penetration: f32,
            /// The contact settings for this contact
            contact_settings: ContactSettings = .{},

            pub fn init(body_id1: BodyID, delta_position: Vec3, linear_cast_threshold_sq: f32, max_penetration: f32) CCDBody {
                return .{ .delta_position = delta_position, .body_id1 = body_id1, .linear_cast_threshold_sq = linear_cast_threshold_sq, .max_penetration = max_penetration };
            }
        };

        /// The physics update context
        context: *PhysicsUpdateContext = undefined,

        /// If this is the first step
        is_first: bool = undefined,
        /// If this is the last step
        is_last: bool = undefined,

        /// Handle returned by Broadphase::UpdatePrepare
        broad_phase_update_state: BroadPhase.UpdateState = .{},

        /// Number of bodies that were active at the start of the physics update step. Only these bodies will receive gravity (they are the first N in the active body list).
        num_active_bodies_at_step_start: u32 = undefined,

        /// Next constraint for determine active constraints
        determine_active_constraint_read_idx: PaddedAtomicU32 = .{},
        /// Number of constraints in the mActiveConstraints array
        num_active_constraints: PaddedAtomicU32 = .{},
        /// Next constraint for setting up velocity constraints
        setup_velocity_constraints_read_idx: PaddedAtomicU32 = .{},
        /// Next step listener to call
        step_listener_read_idx: PaddedAtomicU32 = .{},
        /// Next body to apply gravity to
        apply_gravity_read_idx: PaddedAtomicU32 = .{},
        /// Index of fist active body that has not yet been processed by the broadphase
        active_body_read_idx: PaddedAtomicU32 = .{},

        /// Queues in which to put body pairs that need to be tested by the narrowphase
        body_pair_queues: BodyPairQueues = .empty,

        /// Amount of body pairs that we can queue per queue
        max_body_pairs_per_queue: u32 = undefined,

        /// A bitmask that indicates which jobs are still active
        active_find_collision_jobs: std.atomic.Value(JobMask) = .init(0),

        /// The number of body pairs found in this step (used to size the contact cache in the next step)
        num_body_pairs: std.atomic.Value(u32) = .init(0),
        /// The number of manifolds found in this step (used to size the contact cache in the next step)
        num_manifolds: std.atomic.Value(u32) = .init(0),

        /// Next island that needs to be processed for the solve velocity constraints step (doesn't need own cache line since position jobs don't run at same time)
        solve_velocity_constraints_next_island: std.atomic.Value(u32) = .init(0),
        /// Next island that needs to be processed for the solve position constraints step (doesn't need own cache line since velocity jobs don't run at same time)
        solve_position_constraints_next_island: std.atomic.Value(u32) = .init(0),

        /// Next active body index to take when integrating velocities
        integrate_velocity_read_idx: std.atomic.Value(u32) = .init(0),
        /// List of bodies that need to do continuous collision detection (mCCDBodies, the capacity is its length)
        ccd_bodies: TempArray(CCDBody) = .empty,
        /// Number of CCD bodies in mCCDBodies
        num_ccd_bodies: std.atomic.Value(u32) = .init(0),
        /// Next unprocessed body index in mCCDBodies
        next_ccd_body: std.atomic.Value(u32) = .init(0),
        /// A mapping between an index in BodyManager::mActiveBodies and the index in mCCDBodies (the number of indices is its length)
        active_body_to_ccd_body: TempArray(i32) = .empty,

        // Jobs in order of execution (some run in parallel)
        /// Prepares the new tree in the background
        broad_phase_prepare: JobHandle = .empty,
        /// Listeners to notify of the beginning of a physics step
        step_listeners: JobHandleArray = .empty,
        /// Determine which constraints will be active during this step
        determine_active_constraints: JobHandleArray = .empty,
        /// Update velocities of bodies with gravity
        apply_gravity: JobHandleArray = .empty,
        /// Find all collisions between active bodies an the world
        find_collisions: JobHandleArray = .empty,
        /// Swap the newly built tree with the current tree
        update_broadphase_finalize: JobHandle = .empty,
        /// Calculate properties for all constraints in the constraint manager
        setup_velocity_constraints: JobHandleArray = .empty,
        /// Go over all constraints and assign the bodies they're attached to to an island
        build_islands_from_constraints: JobHandle = .empty,
        /// Finalize calculation simulation islands
        finalize_islands: JobHandle = .empty,
        /// Set the current island index on each body (not used by the simulation, only for drawing purposes)
        body_set_island_index: JobHandle = .empty,
        /// Solve the constraints in the velocity domain
        solve_velocity_constraints: JobHandleArray = .empty,
        /// Setup integration of all body positions
        pre_integrate_velocity: JobHandle = .empty,
        /// Integrate all body positions
        integrate_velocity: JobHandleArray = .empty,
        /// Finalize integration of all body positions
        post_integrate_velocity: JobHandle = .empty,
        /// Updates the positions and velocities for all bodies that need continuous collision detection
        resolve_ccd_contacts: JobHandle = .empty,
        /// Solve all constraints in the position domain
        solve_position_constraints: JobHandleArray = .empty,
        /// Calls the contact removed callbacks
        contact_removed_callbacks: JobHandle = .empty,
        /// Prepares updating the soft bodies
        soft_body_prepare: JobHandle = .empty,
        /// Finds all colliding shapes for soft bodies
        soft_body_collide: JobHandleArray = .empty,
        /// Simulates all particles
        soft_body_simulate: JobHandleArray = .empty,
        /// Finalizes the soft body update
        soft_body_finalize: JobHandle = .empty,
        /// Job that kicks the next step (empty for the last step)
        start_next_step: JobHandle = .empty,

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

    /// Jolt: Array<Step, STLTempAllocator<Step>>
    pub const Steps = TempArray(Step);

    /// StaticArray::resize of an array of job handles (Jolt default constructs the new handles: they are empty)
    pub fn resizeHandles(handles: *JobHandleArray, n: u32) void {
        std.debug.assert(handles.isEmpty());
        handles.resize(n);
        for (handles.slice()) |*h|
            h.* = .empty;
    }

    fn deinitHandles(handles: *JobHandleArray) void {
        for (handles.slice()) |*h|
            h.deinit();
        handles.clear();
    }

    /// The physics system we belong to
    physics_system: *PhysicsSystem,
    /// Temporary allocator used during the update
    temp_allocator: TempAllocator,
    /// Job system that processes jobs
    job_system: JobSystem,
    /// Barrier used to wait for all physics jobs to complete
    barrier: *JobSystem.Barrier,

    /// Delta time for a simulation step (collision step)
    step_delta_time: f32,
    /// Ratio of this step delta time vs last step
    warm_start_impulse_ratio: f32,
    /// Errors that occurred during the update, actual type is EPhysicsUpdateError
    errors: std.atomic.Value(u32) = .init(0),

    /// Zolt (D5): set when an allocation failed inside a job and never cleared during the update. Poisoned jobs skip
    /// their work and allocations but keep their frees, commits and dependency bookkeeping; `update` returns
    /// error.OutOfMemory.
    out_of_memory: std.atomic.Value(bool) = .init(false),

    /// Constraints that were active at the start of the physics update step (activating bodies can activate constraints and we need a consistent snapshot). Only these constraints will be resolved.
    active_constraints: TempArray(*Constraint) = .empty,

    /// A list of body pairs found by the broadphase
    body_pairs: TempArray(BodyPairSlot) = .empty,

    /// Keeps track of all bodies in the simulation
    body_manager: *BodyManager,
    /// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
    island_builder: *IslandBuilder,

    steps: Steps = .empty,

    /// Contexts for updating soft bodies (Jolt: mSoftBodyUpdateContexts, mNumSoftBodies is its length)
    soft_body_update_contexts: TempArray(SoftBodyUpdateContext) = .empty,
    /// Next soft body to take when running SoftBodyCollide jobs
    soft_body_to_collide: std.atomic.Value(u32) = .init(0),

    /// Destructor: releases the job handles of the steps and frees the steps (they were the first temp allocation of the
    /// update, so they are freed last)
    pub fn deinit(self: *PhysicsUpdateContext) void {
        std.debug.assert(self.body_pairs.isEmpty());
        std.debug.assert(self.active_constraints.isEmpty());

        for (self.steps.items) |*step|
            step.deinit();
        self.steps.free(self.temp_allocator);
    }

    /// Number of active soft bodies in the simulation (mNumSoftBodies)
    pub fn getNumSoftBodies(self: *const PhysicsUpdateContext) u32 {
        return @intCast(self.soft_body_update_contexts.items.len);
    }

    /// Maximum amount of concurrent jobs on this machine
    pub fn getMaxConcurrency(self: *const PhysicsUpdateContext) u32 {
        const max_concurrency_value: i32 = PhysicsUpdateContext.max_concurrency;
        return @intCast(@min(max_concurrency_value, self.job_system.getMaxConcurrency()));
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
// @endcond

test "PhysicsUpdateContext: padded atomics and body pair slots" {
    // The padding keeps each atomic on its own cache line sized block, also in temp memory
    try std.testing.expectEqual(@as(usize, Core.cache_line_size), @sizeOf(PhysicsUpdateContext.PaddedAtomicU32));
    try std.testing.expectEqual(@as(usize, 2 * Core.cache_line_size), @sizeOf(PhysicsUpdateContext.BodyPairQueue));
    try std.testing.expectEqual(@as(usize, 0), @offsetOf(PhysicsUpdateContext.BodyPairQueue, "write_idx"));
    try std.testing.expectEqual(@as(usize, Core.cache_line_size), @offsetOf(PhysicsUpdateContext.BodyPairQueue, "read_idx"));
    comptime std.debug.assert(@alignOf(PhysicsUpdateContext.Step) <= @import("../Math/Real.zig").rvector_alignment);

    // A body pair survives a round trip through a slot bit for bit
    var slot: PhysicsUpdateContext.BodyPairSlot = .init(0);
    const pair = BodyPair.init(BodyID.init(0x12345), BodyID.init(0x7fff00));
    PhysicsUpdateContext.storeBodyPair(&slot, pair);
    const loaded = PhysicsUpdateContext.loadBodyPair(&slot);
    try std.testing.expect(loaded.eql(pair));
}
