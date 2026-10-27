//! Port of: Jolt/Physics/PhysicsUpdateContext.h, Jolt/Physics/PhysicsUpdateContext.cpp
//! Status: complete
//!
//! SimulationArchitecture D4 / D5:
//! - The context lives on the stack of PhysicsSystem.update; the steps are a `TempArray(Step)` (Jolt:
//!   `Array<Step, STLTempAllocator<Step>>`, resized once and never reallocated), the first temp allocation of the update
//!   and the last one freed. The job closures capture pointers to the context and the steps.
//! - Temp memory is only rvector_alignment aligned, so the padded atomics of Step are extern structs with explicit
//!   padding (`PaddedAtomicU32` in `PaddedAtomics`, `BodyPairQueue`) instead of `align(Core.cache_line_size)`.
//! - `mBodyPairs` is `body_pairs: TempArray(BodyPairSlot)`: Jolt writes and reads a plain BodyPair while other jobs may
//!   read the slot before their compare-exchange fails (TSAN suppressed in Jolt); Zolt stores the 8 bytes as one
//!   `.monotonic` atomic (`storeBodyPair` / `loadBodyPair`), which is defined and has the same bits.
//! - `mCCDBodies` + `mCCDBodiesCapacity` and `mActiveBodyToCCDBody` + `mNumActiveBodyToCCDBody` are TempArrays (the
//!   capacity is the length), `mSoftBodyUpdateContexts` + `mNumSoftBodies` too.
//! - Zolt addition (D5): the out of memory latch `out_of_memory` with `setOutOfMemory` / `isOutOfMemory`.
//! - Zolt addition (Phase 12, class A): `Step.sort_scratch_used`, the bytes of the scratch regions that the sorts of
//!   the large islands of the step use (PhysicsSystem.claimSortScratch).
//! - The constructor `PhysicsUpdateContext(TempAllocator &)` is the struct literal in PhysicsSystem.update; the
//!   destructor is `deinit` (it releases the job handles of the steps and frees the steps).
//!
//! Difference with the C++ version (Phase 13, class A, layout only): the six padded atomics of Step (Jolt's
//! mDetermineActiveConstraintReadIdx + mPadding1 ... mActiveBodyReadIdx + mPadding6) and the body pair queues behind
//! them (Jolt's mBodyPairQueues) are the fields of one extern struct `PaddedAtomics`, in Jolt's order, behind 60 bytes
//! of padding (`step.padded_atomics.active_body_read_idx.value`, `step.bodyPairQueues()`). Jolt declares them one after
//! the other, so each one after the first has only the previous one's padding in front of it, and mActiveBodyReadIdx,
//! the last of the six, sits between mPadding5 and mPadding6. Zig reorders the fields of Step: with six separate padded
//! fields it put active_body_read_idx (a compare-exchange per batch of 16 active bodies in the FindCollisions jobs)
//! first, right behind max_body_pairs_per_queue, which the FindCollisions jobs read for every body pair they queue or
//! take, active_find_collision_jobs and the other unpadded counters. With the six in an extern struct and the queues
//! (a StaticArray) a separate field of Step, it put the queues there instead: the write index of queue 0, an atomic add
//! for every body pair that FindCollisions job 0 queues, 8 bytes behind max_body_pairs_per_queue. In the extern struct
//! every atomic, including the write and read index of each queue, has at least 60 bytes of padding on both sides, so
//! whatever Zig places around the struct, a cache line that holds an atomic holds nothing but the atomic and padding (no
//! job writes the padding). The number of queues in use (Jolt: mBodyPairQueues.size(), which shares the cache line of
//! the write index of queue 0) is `Step.num_body_pair_queues`, outside the struct. Simulation results do not change,
//! but the 60 bytes in front (64 with the alignment of Step) make Step larger than Jolt's: 7,408 bytes instead of 7,344,
//! in both precisions. An update needs 64 * collision_steps bytes more temp memory than Jolt, so a TempAllocatorImpl
//! sized at Jolt's exact peak can make `PhysicsSystem.update` return error.OutOfMemory where Jolt succeeds (a
//! TempAllocatorImplWithMallocFallback takes those bytes from the heap instead). The test "PhysicsUpdateContext: no
//! field of a step shares a cache line with a padded atomic" checks the layout.

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

    /// Zolt (Phase 13, class A, layout only): the six padded atomics of Step in Jolt's order, followed by the body pair
    /// queues (Jolt: mPadding6, then mBodyPairQueues), behind as much padding as Jolt puts behind each atomic. Every
    /// atomic then has at least 60 bytes of padding on both sides, which keeps the cache line of the atomic free of other
    /// data wherever Zig places this struct in Step (see the top of this file).
    pub const PaddedAtomics = extern struct {
        /// Zolt: padding to avoid sharing cache line with the field that Zig places in front of this struct
        padding_before: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
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
        /// Queues in which to put body pairs that need to be tested by the narrowphase (Jolt: mBodyPairQueues, a
        /// StaticArray; the first Step.num_body_pair_queues are in use, see Step.bodyPairQueues)
        body_pair_queues: [max_concurrency]BodyPairQueue = undefined,
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

        /// Zolt (Phase 13): Jolt's mDetermineActiveConstraintReadIdx, mNumActiveConstraints,
        /// mSetupVelocityConstraintsReadIdx, mStepListenerReadIdx, mApplyGravityReadIdx and mActiveBodyReadIdx with their
        /// paddings and the elements of mBodyPairQueues, in one extern struct (see the top of this file)
        padded_atomics: PaddedAtomics = .{},

        /// Zolt (Phase 13): the number of body pair queues in use (Jolt: mBodyPairQueues.size()), kept off the cache line
        /// of the first queue
        num_body_pair_queues: u32 = 0,

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
        /// Zolt addition (Phase 12, class A): the bytes of the two scratch regions that the sorts of the large islands of
        /// this step use (PhysicsSystem.sortScratchRegions, claimSortScratch)
        sort_scratch_used: [2]std.atomic.Value(u32) = .{ .init(0), .init(0) },

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

        /// Queues in which to put body pairs that need to be tested by the narrowphase (Jolt: mBodyPairQueues)
        pub fn bodyPairQueues(self: *Step) []BodyPairQueue {
            return self.padded_atomics.body_pair_queues[0..self.num_body_pair_queues];
        }

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
        std.debug.assert(self.soft_body_update_contexts.isEmpty()); // SoftBodyFinalize (or SoftBodyPrepare) freed them

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
    // Zolt (Phase 13): the six padded atomics of a step and then the body pair queues follow each other in Jolt's order
    // behind 60 bytes of padding
    const PaddedAtomics = PhysicsUpdateContext.PaddedAtomics;
    const queues_size = PhysicsUpdateContext.max_concurrency * 2 * Core.cache_line_size;
    try std.testing.expectEqual(@as(usize, Core.cache_line_size - @sizeOf(u32) + 6 * Core.cache_line_size + queues_size), @sizeOf(PaddedAtomics));
    inline for (.{ "determine_active_constraint_read_idx", "num_active_constraints", "setup_velocity_constraints_read_idx", "step_listener_read_idx", "apply_gravity_read_idx", "active_body_read_idx", "body_pair_queues" }, 0..) |name, i|
        try std.testing.expectEqual(@as(usize, Core.cache_line_size - @sizeOf(u32) + i * Core.cache_line_size), @offsetOf(PaddedAtomics, name));
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

/// Zolt (Phase 13): a range of bytes of a struct that holds data (a field, or the atomic of a padded atomic)
const DataRange = struct {
    begin: usize,
    end: usize,
    name: []const u8,
    is_atomic: bool,
};

/// Zolt (Phase 13): true for the extern structs that hold padded atomics and nothing else: their `std.atomic.Value(u32)`
/// fields are the atomics, all other fields are padding
fn isPaddedAtomicType(comptime T: type) bool {
    return T == PhysicsUpdateContext.PaddedAtomicU32 or T == PhysicsUpdateContext.BodyPairQueue;
}

/// Zolt (Phase 13): true when a value of type `T` holds padded atomics: a padded atomic type, an array of them or a
/// struct with such a field (`PaddedAtomics`, also a StaticArray of `BodyPairQueue` like Jolt's mBodyPairQueues)
fn holdsPaddedAtomics(comptime T: type) bool {
    if (isPaddedAtomicType(T))
        return true;
    return switch (@typeInfo(T)) {
        .array => |info| holdsPaddedAtomics(info.child),
        .@"struct" => |info| for (info.fields) |field| {
            if (holdsPaddedAtomics(field.type))
                break true;
        } else false,
        else => false,
    };
}

/// Zolt (Phase 13): `ranges` followed by the bytes of a `T` at byte `begin` that hold data. A padded atomic holds data
/// only in its atomics. In a struct that holds padded atomics, a byte array whose name starts with "padding"
/// (`PaddedAtomics.padding_before`) holds none; its other fields are walked the same way. Fields of size 0 hold none.
fn appendDataRanges(comptime ranges: []const DataRange, comptime T: type, comptime begin: usize, comptime name: []const u8) []const DataRange {
    comptime {
        if (!holdsPaddedAtomics(T)) {
            if (@sizeOf(T) == 0)
                return ranges;
            return ranges ++ [_]DataRange{.{ .begin = begin, .end = begin + @sizeOf(T), .name = name, .is_atomic = false }};
        }
        var result = ranges;
        switch (@typeInfo(T)) {
            .array => |info| for (0..info.len) |i| {
                result = appendDataRanges(result, info.child, begin + i * @sizeOf(info.child), name ++ std.fmt.comptimePrint("[{d}]", .{i}));
            },
            .@"struct" => |info| for (info.fields) |field| {
                const field_begin = begin + @offsetOf(T, field.name);
                const field_name = name ++ "." ++ field.name;
                if (isPaddedAtomicType(T)) {
                    if (field.type == std.atomic.Value(u32))
                        result = result ++ [_]DataRange{.{ .begin = field_begin, .end = field_begin + @sizeOf(u32), .name = field_name, .is_atomic = true }};
                } else if (!(std.mem.startsWith(u8, field.name, "padding") and field.type == [@sizeOf(field.type)]u8)) {
                    result = appendDataRanges(result, field.type, field_begin, field_name);
                }
            },
            else => unreachable,
        }
        return result;
    }
}

/// Zolt (Phase 13): the bytes of the struct `T` that hold data (see appendDataRanges)
fn dataRanges(comptime T: type) []const DataRange {
    comptime {
        @setEvalBranchQuota(100_000);
        var ranges: []const DataRange = &.{};
        for (@typeInfo(T).@"struct".fields) |field|
            ranges = appendDataRanges(ranges, field.type, @offsetOf(T, field.name), field.name);
        const result = ranges[0..ranges.len].*;
        return &result;
    }
}

/// Zolt (Phase 13): the number of padded atomics in the struct `T` (those of `PaddedAtomicU32` and `BodyPairQueue`, also
/// inside other fields). Fails when a byte of data other than the atomic (another field of `T`, or another padded
/// atomic) can share a cache line with one of them, wherever the `T` is. The atomic is 4 bytes and 4 byte aligned, and
/// so is any cache line start relative to it: a cache line that holds the atomic at byte `a` starts at a - 60, a - 56,
/// ... or a, so the cache lines that can hold it cover the bytes a - 60 to a + 63. Jolt's spacing of its padded atomics
/// passes too: the previous atomic ends at a - 60, the next one starts at a + 64. (Jolt's mBodyPairQueues does not:
/// its size, a StaticArray's mSize, sits right in front of the write index of queue 0.)
fn expectPaddedAtomicsIsolated(comptime T: type) !usize {
    const line = Core.cache_line_size;
    const ranges = comptime dataRanges(T);
    var num_atomics: usize = 0;
    var shared: usize = 0;
    for (ranges) |atomic| {
        if (!atomic.is_atomic)
            continue;
        num_atomics += 1;
        for (ranges) |other| {
            if (other.begin != atomic.begin and other.begin < atomic.begin + line and other.end + (line - @sizeOf(u32)) > atomic.begin) {
                std.debug.print("{s}.{s} (bytes {d}..{d}) can share a cache line with the atomic of {s} (byte {d})\n", .{ @typeName(T), other.name, other.begin, other.end, atomic.name, atomic.begin });
                shared += 1;
            }
        }
    }
    try std.testing.expectEqual(@as(usize, 0), shared);
    return num_atomics;
}

// Zolt (Phase 13): Zig orders the fields of Step itself, so whatever it puts around the padded atomics, the cache line of
// each atomic must hold nothing but the atomic and padding (see the top of this file). Step is the only user of
// PaddedAtomics (the context itself has none).
// Bite proofs: with the queues as a StaticArray field of Step (the layout of the first version of PaddedAtomics) this
// test fails with 12 reports, all at the write index of queue 0: the queue count, max_body_pairs_per_queue,
// active_find_collision_jobs, num_body_pairs, the 7 other unpadded counters and the end of setup_velocity_constraints.
// With `padding_before` of size 0 it fails with 12 reports at determine_active_constraint_read_idx (num_body_pair_queues,
// max_body_pairs_per_queue, the 9 other unpadded counters and the end of setup_velocity_constraints, which Zig places in
// front of `padded_atomics`). With BodyPairQueue.padding1 of size 0 it fails with 64 reports: the write and read index
// of each queue.
test "PhysicsUpdateContext: no field of a step shares a cache line with a padded atomic" {
    // The six padded atomics and the write and read index of every body pair queue
    try std.testing.expectEqual(@as(usize, 6 + 2 * PhysicsUpdateContext.max_concurrency), try expectPaddedAtomicsIsolated(PhysicsUpdateContext.Step));
    try std.testing.expectEqual(@as(usize, 0), try expectPaddedAtomicsIsolated(PhysicsUpdateContext));
}
