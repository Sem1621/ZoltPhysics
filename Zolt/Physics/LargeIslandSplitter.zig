//! Port of: Jolt/Physics/LargeIslandSplitter.h, Jolt/Physics/LargeIslandSplitter.cpp
//! Status: complete
//! Not ported: JPH_LARGE_ISLAND_SPLITTER_DEBUG (a debug define that is commented out in Jolt and only traces the sizes
//! of the splits)
//!
//! SimulationArchitecture D5:
//! - The four buffers of Prepare are `TempArray`s, allocated in Jolt's order (split masks, split indices, constraint
//!   and contact indices, split islands) and freed in Reset in the reverse order. A failed allocation leaves an empty
//!   array that consumed nothing, so `reset` frees exactly what was allocated, also after an out of memory error.
//!   `prepare` returns `TempAllocator.Error` (Jolt aborts in the temp allocator).
//! - The batch protocol (fetchNextBatch, the processing, markBatchProcessed) has no out of memory latch: other
//!   SolveVelocityConstraints / SolvePositionConstraints jobs wait until every claimed batch is marked as processed,
//!   so a job that claimed a batch always processes and marks it. Jolt's memory orders are kept 1:1 (the status word
//!   and the processed item counter, `memory_order_relaxed` is `.monotonic`).
//! - Zolt addition: `resetAfterOutOfMemory` frees like `reset` but skips Reset's check that every large island was
//!   split. When an out of memory error poisoned the update, no SolveVelocityConstraints job may have run, so the
//!   islands that Prepare counted were never split (PhysicsSystem calls it instead of `reset` after an error). It also
//!   sets `num_active_bodies` back to 0 when the allocation of the split masks failed (Reset only does that when it
//!   frees the split masks), so the splitter is in Jolt's constructed state again.
//!
//! Differences with the C++ version (Phase 12, class A): the loops of SplitIsland collect the step overrides of the
//! dynamic bodies of the contacts and of the constraints in a `CalculateSolverSteps.Overrides` (four integers that stay
//! in registers) and apply it to ioStepsCalculator before Finalize, instead of calling the calculator per object. The
//! calculator ends in the same state (CalculateSolverSteps.Overrides).
//!
//! Zolt addition (Phase 13, class A): the SIMD plans of the batches of the parallel splits (ContactConstraintSimd.zig,
//! "The plan of a batch") live in `contact_and_constraints_split_idx`. Its entries are needed only inside splitIsland:
//! they hold the split index of each contact and constraint of the island between the assignment loops and the loops
//! that sort the contacts and constraints into the splits, and nothing reads them afterwards. So after splitIsland the
//! entry at the index of a batch's first contact in `contact_and_constraint_indices` (`batchPlanWord`) belongs to that
//! batch: its warm start writes the plan there, its velocity iterations read it. A plan carries a marker that no split
//! index has, so a word that splitIsland wrote is never taken for a plan. Rules for changes to this file:
//! - Add no reader of `contact_and_constraints_split_idx` after splitIsland (or keep the split indices elsewhere).
//! - The contacts of a batch must stay a function of its split and its first item (the item counter of the status
//!   restarts at 0 for every split and iteration and grows by batch_size): the velocity iterations of a batch must see
//!   the slice that its warm start planned. A future bounds phase or another batch layout has to keep that, or plan
//!   per batch layout.
//! - splitIsland must keep writing every entry of its island's range before startFirstBatch: the buffer comes from
//!   the temp allocator and can hold the plans of the previous collision step.
//!
//! Out parameters and pointers:
//! - `Splits::FetchNextBatch` / `LargeIslandSplitter::FetchNextBatch` write their out parameters into an out struct
//!   (`Splits.SplitBatch` with offsets relative to `contact_and_constraint_indices`, `Batch` with slices of it), only when
//!   the status is `batch_retrieved`, like Jolt. The constraint and contact slices of a `Batch` point into
//!   `contact_and_constraint_indices` at Jolt's offsets (an empty range at offset 0 where Jolt sets begin = end = 0).
//! - `MarkBatchProcessed(..., outLastIteration, outFinalBatch)` keeps the two `*bool` out parameters (the F2 call site
//!   in PhysicsSystem). `GetConstraintsInSplit` / `GetContactsInSplit` return a `BufferRange`.
//! - `SplitIsland` takes the contact manager as `anytype`: anything with Jolt's `GetAffectedBodies`, i.e.
//!   `getAffectedBodies(constraint_offset: u32)` returning a value with the fields `body1: *const Body` and
//!   `body2: *const Body` (`ContactConstraintManager.AffectedBodies`). PhysicsSystem passes its ContactConstraintManager,
//!   the tests pass contacts as body pairs. The steps calculator is Jolt's concrete `*CalculateSolverSteps`.
//! - JPH_DEBUG's validation in SplitIsland ("the splits are not touching the same body") allocates an Array<bool> per
//!   split in Jolt. Zolt reuses the split masks of the bodies of the island instead (they are not needed after the
//!   assignment): the same check without a heap allocation inside a job.
//! - `Splits` is an extern struct with Jolt's layout (552 bytes, checked at comptime and against the C++ sizeof), so the
//!   split islands take Jolt's temp memory. `mNumIterations`, `mNumVelocitySteps` and `mNumPositionSteps` stay `i32`
//!   (Jolt's `int`), assigned from CalculateSolverSteps' unsigned counts with Jolt's two's complement conversion.
//!
//! Use in PhysicsSystem (Jolt's JobSolveVelocityConstraints / JobSolvePositionConstraints):
//! - SolveVelocityConstraints: after ConstraintIdxToConstraintOffset and the two sorts of an island, call
//!   `splitIsland(island_idx, &island_builder, &body_manager, &contact_manager, active_constraints, &steps_calculator)`
//!   with a fresh `CalculateSolverSteps.init(&physics_settings)`; on `false` solve the island yourself with the same
//!   calculator. Batches: `fetchNextBatch(&batch)`, then warm start (`batch.first_iteration`) or solve
//!   `batch.constraints` / `batch.contacts`, then `markBatchProcessed(batch.split_island_index, batch.constraints,
//!   batch.contacts, &last_iteration, &final_batch)`; `getIslandIndex(batch.split_island_index)` for the stats and
//!   CheckSleepAndUpdateBounds.
//! - The out of memory latch (D5) may only be checked when a SolveVelocityConstraints job starts, not before each island
//!   claim: `fetchNextBatch` reports `waiting_for_batch` until every island that Prepare counted is split, so a job that
//!   stops claiming islands while others wait for the split islands would hang them.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../Core/Core.zig");
const TempAllocatorFile = @import("../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const Body = @import("Body/Body.zig").Body;
const BodyManager = @import("Body/BodyManager.zig").BodyManager;
const MotionProperties = @import("Body/MotionProperties.zig").MotionProperties;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const Constraint = @import("Constraints/Constraint.zig").Constraint;
const CalculateSolverSteps = @import("Constraints/CalculateSolverSteps.zig").CalculateSolverSteps;

/// Assigns bodies in large islands to multiple groups that can run in parallel
///
/// This basically implements what is described in: High-Performance Physical Simulations on Next-Generation Architecture with Many Cores by Chen et al.
/// See: http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf section "PARALLELIZATION METHODOLOGY"
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const LargeIslandSplitter = struct {
    const SplitMask = u32;

    pub const num_splits: u32 = @sizeOf(SplitMask) * 8;
    pub const non_parallel_split_idx: u32 = num_splits - 1;
    /// If the number of constraints + contacts in an island is larger than this, we will try to split the island
    pub const large_island_treshold: u32 = 128;

    /// If the number of constraints + contacts in a split is lower than this, we will merge this split into the 'non-parallel split'
    const split_combine_treshold: u32 = 32;
    /// Number of items to process in a constraint batch
    const batch_size: u32 = 16;

    /// Status code for retrieving a batch
    pub const Status = enum {
        /// Work is expected to be available later
        waiting_for_batch,
        /// Work is being returned
        batch_retrieved,
        /// No further work is expected from this
        all_batches_done,
    };

    /// A range [begin, end) of offsets relative to contact_and_constraint_indices (Zolt: the out parameters of
    /// Splits::GetConstraintsInSplit / GetContactsInSplit)
    pub const BufferRange = struct {
        begin: u32,
        end: u32,
    };

    /// Describes a split of constraints and contacts
    pub const Split = extern struct {
        /// Begin of the contact buffer (offset relative to mContactAndConstraintIndices)
        contact_buffer_begin: u32,
        /// End of the contact buffer
        contact_buffer_end: u32,

        /// Begin of the constraint buffer (offset relative to mContactAndConstraintIndices)
        constraint_buffer_begin: u32,
        /// End of the constraint buffer
        constraint_buffer_end: u32,

        pub fn getNumContacts(self: *const Split) u32 {
            return self.contact_buffer_end - self.contact_buffer_begin;
        }

        pub fn getNumConstraints(self: *const Split) u32 {
            return self.constraint_buffer_end - self.constraint_buffer_begin;
        }

        pub fn getNumItems(self: *const Split) u32 {
            return self.getNumContacts() + self.getNumConstraints();
        }
    };

    /// Structure that describes the resulting splits from the large island splitter
    /// (an extern struct with Jolt's layout: the split islands live in temp memory with Jolt's size)
    pub const Splits = extern struct {
        /// Data per split
        splits: [num_splits]Split,
        /// Index of the island that was split
        island_index: u32,
        /// Number of splits that were created (excluding the non-parallel split)
        num_splits: u32,
        /// Number of iterations to do
        num_iterations: i32,
        /// Number of velocity steps to do (cached for 2nd sub step)
        num_velocity_steps: i32,
        /// Number of position steps to do
        num_position_steps: i32,
        /// Status of the split, see EIterationStatus
        status: std.atomic.Value(u64),
        /// Number of items that have been marked as processed
        items_processed: std.atomic.Value(u32),

        // EIterationStatus
        pub const status_iteration_mask: u64 = 0xffff000000000000;
        pub const status_iteration_shift: u6 = 48;
        pub const status_split_mask: u64 = 0x0000ffff00000000;
        pub const status_split_shift: u6 = 32;
        pub const status_item_mask: u64 = 0x00000000ffffffff;

        /// A batch of a split (Zolt: the out parameters of FetchNextBatch, offsets relative to mContactAndConstraintIndices)
        pub const SplitBatch = struct {
            constraints_begin: u32,
            constraints_end: u32,
            contacts_begin: u32,
            contacts_end: u32,
            first_iteration: bool,
            /// Zolt addition: true when the batch comes from a parallel split (split index below non_parallel_split_idx).
            /// No dynamic body appears twice in a parallel split, so the contacts of the batch can be solved in any order
            /// (ContactConstraintSimd.zig).
            parallel: bool,
        };

        pub fn getNumSplits(self: *const Splits) u32 {
            return self.num_splits;
        }

        pub fn getConstraintsInSplit(self: *const Splits, split_index: u32) BufferRange {
            const split = &self.splits[split_index];
            return .{ .begin = split.constraint_buffer_begin, .end = split.constraint_buffer_end };
        }

        pub fn getContactsInSplit(self: *const Splits, split_index: u32) BufferRange {
            const split = &self.splits[split_index];
            return .{ .begin = split.contact_buffer_begin, .end = split.contact_buffer_end };
        }

        /// Reset current status so that no work can be picked up from this split
        pub fn resetStatus(self: *Splits) void {
            self.status.store(status_item_mask, .monotonic);
        }

        /// Make the first batch available to other threads
        pub fn startFirstBatch(self: *Splits) void {
            const split_index: u32 = if (self.num_splits > 0) 0 else non_parallel_split_idx;
            self.status.store(@as(u64, split_index) << status_split_shift, .release);
        }

        /// Fetch the next batch to process
        /// (Zolt: the batch is written to `out_batch` when the status is batch_retrieved)
        pub fn fetchNextBatch(self: *Splits, out_batch: *SplitBatch) Status {
            {
                // First check if we can get a new batch (doing a read to avoid hammering an atomic with an atomic subtract)
                // Note this also avoids overflowing the status counter if we're done but there's still one thread processing items
                const status = self.status.load(.acquire);

                // Check for special value that indicates that the splits are still being built
                // (note we do not check for this condition again below as we reset all splits before kicking off jobs that fetch batches of work)
                if (status == status_item_mask)
                    return .waiting_for_batch;

                // Next check if all items have been processed. Note that we do this after checking if the job can be started
                // as mNumIterations is not initialized until the split is started.
                if (getIteration(status) >= self.num_iterations)
                    return .all_batches_done;

                const item = getItem(status);
                const split_index = getSplit(status);
                if (split_index == non_parallel_split_idx) {
                    // Non parallel split needs to be taken as a single batch, only the thread that takes element 0 will do it
                    if (item != 0)
                        return .waiting_for_batch;
                } else {
                    // Parallel split is split into batches
                    std.debug.assert(split_index < self.num_splits);
                    const split = &self.splits[split_index];
                    if (item >= split.getNumItems())
                        return .waiting_for_batch;
                }
            }

            // Then try to actually get the batch
            const status = self.status.fetchAdd(batch_size, .acquire);
            const iteration = getIteration(status);
            if (iteration >= self.num_iterations)
                return .all_batches_done;

            const split_index = getSplit(status);
            std.debug.assert(split_index < self.num_splits or split_index == non_parallel_split_idx);
            const split = &self.splits[split_index];
            const item_begin = getItem(status);
            if (split_index == non_parallel_split_idx) {
                if (item_begin == 0) {
                    // Non-parallel split always goes as a single batch
                    out_batch.* = .{
                        .constraints_begin = split.constraint_buffer_begin,
                        .constraints_end = split.constraint_buffer_end,
                        .contacts_begin = split.contact_buffer_begin,
                        .contacts_end = split.contact_buffer_end,
                        .first_iteration = iteration == 0,
                        .parallel = false,
                    };
                    return .batch_retrieved;
                } else {
                    // Otherwise we're done with this split
                    return .waiting_for_batch;
                }
            }

            // Parallel split is split into batches
            const num_constraints = split.getNumConstraints();
            const num_contacts = split.getNumContacts();
            const num_items = num_constraints + num_contacts;
            if (item_begin >= num_items)
                return .waiting_for_batch;

            const item_end = @min(item_begin + batch_size, num_items);
            var batch: SplitBatch = undefined;
            if (item_end >= num_constraints) {
                if (item_begin < num_constraints) {
                    // Partially from constraints and partially from contacts
                    batch.constraints_begin = split.constraint_buffer_begin + item_begin;
                    batch.constraints_end = split.constraint_buffer_end;
                } else {
                    // Only contacts
                    batch.constraints_begin = 0;
                    batch.constraints_end = 0;
                }

                batch.contacts_begin = split.contact_buffer_begin + (@max(item_begin, num_constraints) - num_constraints);
                batch.contacts_end = split.contact_buffer_begin + (item_end - num_constraints);
            } else {
                // Only constraints
                batch.constraints_begin = split.constraint_buffer_begin + item_begin;
                batch.constraints_end = split.constraint_buffer_begin + item_end;

                batch.contacts_begin = 0;
                batch.contacts_end = 0;
            }

            batch.first_iteration = iteration == 0;
            batch.parallel = true;
            out_batch.* = batch;
            return .batch_retrieved;
        }

        /// Mark a batch as processed
        pub fn markBatchProcessed(self: *Splits, num_processed: u32, out_last_iteration: *bool, out_final_batch: *bool) void {
            // We fetched this batch, nobody should change the split and or iteration until we mark the last batch as processed so we can safely get the current status
            const status = self.status.load(.monotonic);
            var split_index = getSplit(status);
            std.debug.assert(split_index < self.num_splits or split_index == non_parallel_split_idx);
            const split = &self.splits[split_index];
            const num_items_in_split = split.getNumItems();

            // Determine if this is the last iteration before possibly incrementing it
            var iteration = getIteration(status);
            out_last_iteration.* = iteration == self.num_iterations -% 1;

            // Add the number of items we processed to the total number of items processed
            // Note: This needs to happen after we read the status as other threads may update the status after we mark items as processed
            std.debug.assert(num_processed > 0); // Logic will break if we mark a block of 0 items as processed
            const total_items_processed = self.items_processed.fetchAdd(num_processed, .acq_rel) + num_processed;

            // Check if we're at the end of the split
            if (total_items_processed >= num_items_in_split) {
                std.debug.assert(total_items_processed == num_items_in_split); // Should not overflow, that means we're retiring more items than we should process

                // Set items processed back to 0 for the next split/iteration
                self.items_processed.store(0, .release);

                // Determine next split
                while (true) {
                    if (split_index == non_parallel_split_idx) {
                        // At start of next iteration
                        split_index = 0;
                        iteration += 1;
                    } else {
                        // At start of next split
                        split_index += 1;
                    }

                    // If we're beyond the end of splits, go to the non-parallel split
                    if (split_index >= self.num_splits)
                        split_index = non_parallel_split_idx;

                    if (!(iteration < self.num_iterations and self.splits[split_index].getNumItems() == 0)) // We don't support processing empty splits, skip to the next split in this case
                        break;
                }

                self.status.store((@as(u64, @intCast(iteration)) << status_iteration_shift) | (@as(u64, split_index) << status_split_shift), .release);
            }

            // Track if this is the final batch
            out_final_batch.* = iteration >= self.num_iterations;
        }

        pub fn getIteration(status: u64) i32 {
            return @intCast((status & status_iteration_mask) >> status_iteration_shift);
        }

        pub fn getSplit(status: u64) u32 {
            return @intCast((status & status_split_mask) >> status_split_shift);
        }

        pub fn getItem(status: u64) u32 {
            return @intCast(status & status_item_mask);
        }

        comptime {
            // The layout of Jolt's class (checked against the C++ sizeof by the parity tests)
            std.debug.assert(@sizeOf(Splits) == 552);
            std.debug.assert(@alignOf(Splits) == 8);
            std.debug.assert(@offsetOf(Splits, "status") == 536);
        }
    };

    /// A batch returned by fetchNextBatch (Jolt's out parameters)
    pub const Batch = struct {
        /// Handle that must be provided to markBatchProcessed when complete
        split_island_index: u32,
        /// The constraint indices of the batch (Jolt: outConstraintsBegin / outConstraintsEnd, a slice of
        /// contact_and_constraint_indices)
        constraints: []u32,
        /// The contact constraint offsets of the batch (Jolt: outContactsBegin / outContactsEnd, a slice of
        /// contact_and_constraint_indices)
        contacts: []u32,
        first_iteration: bool,
        /// Zolt addition: true when the batch comes from a parallel split (split index below non_parallel_split_idx). No
        /// dynamic body appears twice in a parallel split, so the contacts of the batch can be solved in any order, for
        /// example 8 at a time by ContactConstraintSimd.zig. The non parallel split shares bodies between its contacts.
        parallel: bool,
    };

    /// Cached number of active bodies
    num_active_bodies: u32 = 0,

    /// Bits that indicate for each body in the BodyManager::mActiveBodies list which split they already belong to
    split_masks: TempArray(SplitMask) = .empty,

    /// Buffer to store the split index per constraint or contact
    contact_and_constraints_split_idx: TempArray(u32) = .empty,
    /// Buffer to store the ordered constraint indices per split
    contact_and_constraint_indices: TempArray(u32) = .empty,
    /// Total size of mContactAndConstraintsSplitIdx and mContactAndConstraintIndices
    contact_and_constraints_size: u32 = 0,
    /// Next element that is free in both buffers
    contact_and_constraints_next_free: std.atomic.Value(u32) = .init(0),

    /// Total number of islands that required splitting
    num_split_islands: u32 = 0,
    /// List of islands that required splitting
    split_islands: TempArray(Splits) = .empty,
    /// Next split island to pick from mSplitIslands
    next_split_island: std.atomic.Value(u32) = .init(0),

    /// Jolt's constructor
    pub const empty: LargeIslandSplitter = .{};

    /// Destructor
    pub fn deinit(self: *LargeIslandSplitter) void {
        std.debug.assert(self.split_masks.isEmpty());
        std.debug.assert(self.contact_and_constraints_split_idx.isEmpty());
        std.debug.assert(self.contact_and_constraint_indices.isEmpty());
        std.debug.assert(self.split_islands.isEmpty());
        self.* = .empty;
    }

    /// Prepare the island splitter by allocating memory
    pub fn prepare(self: *LargeIslandSplitter, island_builder: *const IslandBuilder, num_active_bodies: u32, temp_allocator: TempAllocator) TempAllocator.Error!void {
        // Count the total number of constraints and contacts that we will be putting in splits
        std.debug.assert(self.num_split_islands == 0);
        std.debug.assert(self.contact_and_constraints_size == 0);
        var island: u32 = 0;
        while (island < island_builder.getNumIslands()) : (island += 1) {
            // Get the contacts in this island
            const num_contacts_in_island: u32 = @intCast(island_builder.getContactsInIsland(island).len);

            // Get the constraints in this island
            const num_constraints_in_island: u32 = @intCast(island_builder.getConstraintsInIsland(island).len);

            const island_size = num_contacts_in_island + num_constraints_in_island;
            if (island_size >= large_island_treshold) {
                self.num_split_islands += 1;
                self.contact_and_constraints_size += island_size;
            } else break; // If this island doesn't have enough constraints, the next islands won't either since they're sorted from big to small
        }

        if (self.contact_and_constraints_size > 0) {
            self.num_active_bodies = num_active_bodies;

            // Allocate split mask buffer
            std.debug.assert(self.split_masks.isEmpty());
            self.split_masks = try .alloc(temp_allocator, self.num_active_bodies);

            // Allocate contact and constraint buffer
            std.debug.assert(self.contact_and_constraints_split_idx.isEmpty() and self.contact_and_constraint_indices.isEmpty());
            self.contact_and_constraints_split_idx = try .alloc(temp_allocator, self.contact_and_constraints_size);
            self.contact_and_constraint_indices = try .alloc(temp_allocator, self.contact_and_constraints_size);

            // Allocate island split buffer
            std.debug.assert(self.split_islands.isEmpty());
            self.split_islands = try .alloc(temp_allocator, self.num_split_islands);

            // Prevent any of the splits from being picked up as work
            for (self.split_islands.items) |*s|
                s.resetStatus();
        }
    }

    /// Assign two bodies to a split. Returns the split index.
    pub fn assignSplit(self: *LargeIslandSplitter, body1: *const Body, body2: *const Body) u32 {
        const idx1 = body1.getIndexInActiveBodiesInternal();
        const idx2 = body2.getIndexInActiveBodiesInternal();

        // Test if either index is negative
        if (idx1 == Body.inactive_index or !body1.isDynamic()) {
            // Body 1 is not active or a kinematic body, so we only need to set 1 body
            std.debug.assert(idx2 < self.num_active_bodies);
            const mask = &self.split_masks.items[idx2];
            const split = @min(@as(u32, @ctz(~mask.*)), non_parallel_split_idx);
            mask.* |= @as(SplitMask, 1) << @intCast(split);
            return split;
        } else if (idx2 == Body.inactive_index or !body2.isDynamic()) {
            // Body 2 is not active or a kinematic body, so we only need to set 1 body
            std.debug.assert(idx1 < self.num_active_bodies);
            const mask = &self.split_masks.items[idx1];
            const split = @min(@as(u32, @ctz(~mask.*)), non_parallel_split_idx);
            mask.* |= @as(SplitMask, 1) << @intCast(split);
            return split;
        } else {
            // If both bodies are active, we need to set 2 bodies
            std.debug.assert(idx1 < self.num_active_bodies);
            std.debug.assert(idx2 < self.num_active_bodies);
            const mask1 = &self.split_masks.items[idx1];
            const mask2 = &self.split_masks.items[idx2];
            const split = @min(@as(u32, @ctz((~mask1.*) & (~mask2.*))), non_parallel_split_idx);
            const mask = @as(SplitMask, 1) << @intCast(split);
            mask1.* |= mask;
            mask2.* |= mask;
            return split;
        }
    }

    /// Force a body to be in a non parallel split. Returns the split index.
    pub fn assignToNonParallelSplit(self: *LargeIslandSplitter, body: *const Body) u32 {
        const idx = body.getIndexInActiveBodiesInternal();
        if (idx != Body.inactive_index) {
            std.debug.assert(idx < self.num_active_bodies);
            self.split_masks.items[idx] |= @as(SplitMask, 1) << non_parallel_split_idx;
        }

        return non_parallel_split_idx;
    }

    /// Splits up an island, the created splits will be added to the list of batches and can be fetched with FetchNextBatch. Returns false if the island did not need splitting.
    /// `contact_manager`: the ContactConstraintManager (anything with `getAffectedBodies(constraint_offset)` returning
    /// `.body1` / `.body2`), the contacts of the island are constraint offsets (after ConstraintIdxToConstraintOffset)
    pub fn splitIsland(self: *LargeIslandSplitter, island_index: u32, island_builder: *const IslandBuilder, body_manager: *const BodyManager, contact_manager: anytype, active_constraints: []const *Constraint, steps_calculator: *CalculateSolverSteps) bool {
        // Get the contacts in this island
        const contacts = island_builder.getContactsInIsland(island_index);
        const num_contacts_in_island: u32 = @intCast(contacts.len);

        // Get the constraints in this island
        const constraints = island_builder.getConstraintsInIsland(island_index);
        const num_constraints_in_island: u32 = @intCast(constraints.len);

        // Check if it exceeds the threshold
        const island_size = num_contacts_in_island + num_constraints_in_island;
        if (island_size < large_island_treshold)
            return false;

        // Get bodies in this island
        const island_bodies = island_builder.getBodiesInIsland(island_index);

        // Reset the split mask for all bodies in this island
        const bodies = body_manager.getBodies();
        const split_masks = self.split_masks.items;
        for (island_bodies) |b|
            split_masks[bodies[b.getIndex()].getBody().getIndexInActiveBodiesInternal()] = 0;

        // Count the number of contacts and constraints per split
        var num_contacts_in_split: [num_splits]u32 = @splat(0);
        var num_constraints_in_split: [num_splits]u32 = @splat(0);

        // Zolt (Phase 12, class A): the loops collect the step overrides of the bodies and constraints that Jolt passes to
        // ioStepsCalculator in `overrides` (four integers that stay in registers) and apply them before Finalize, with
        // the same result (CalculateSolverSteps.Overrides)
        var overrides: CalculateSolverSteps.Overrides = .{};

        // Get space to store split indices
        var offset = self.contact_and_constraints_next_free.fetchAdd(island_size, .monotonic);
        const contact_split_idx = self.contact_and_constraints_split_idx.items[offset..][0..num_contacts_in_island];
        const constraint_split_idx = self.contact_and_constraints_split_idx.items[offset + num_contacts_in_island ..][0..num_constraints_in_island];

        // Assign the contacts to a split
        for (contacts, contact_split_idx) |c, *cur_contact_split_idx| {
            const affected_bodies = contact_manager.getAffectedBodies(c);
            const body1: *const Body = affected_bodies.body1;
            const body2: *const Body = affected_bodies.body2;
            const split = self.assignSplit(body1, body2);
            num_contacts_in_split[split] += 1;
            cur_contact_split_idx.* = split;

            if (body1.isDynamic())
                overrides.add(MotionProperties, body1.getMotionPropertiesUnchecked().?);
            if (body2.isDynamic())
                overrides.add(MotionProperties, body2.getMotionPropertiesUnchecked().?);
        }

        // Assign the constraints to a split
        for (constraints, constraint_split_idx) |c, *cur_constraint_split_idx| {
            const constraint: *const Constraint = active_constraints[c];
            const split = constraint.buildIslandSplits(self);
            num_constraints_in_split[split] += 1;
            cur_constraint_split_idx.* = split;

            overrides.add(Constraint, constraint);
        }

        steps_calculator.apply(overrides);
        steps_calculator.finalize();

        // Start with 0 splits
        var split_remap_table: [num_splits]u32 = undefined;
        const new_split_idx = self.next_split_island.fetchAdd(1, .monotonic);
        std.debug.assert(new_split_idx < self.num_split_islands);
        const splits = &self.split_islands.items[new_split_idx];
        splits.island_index = island_index;
        splits.num_splits = 0;
        splits.num_iterations = @bitCast(steps_calculator.getNumVelocitySteps() +% 1); // Iteration 0 is used for warm starting
        splits.num_velocity_steps = @bitCast(steps_calculator.getNumVelocitySteps());
        splits.num_position_steps = @bitCast(steps_calculator.getNumPositionSteps());
        splits.items_processed.store(0, .release);

        // Allocate space to store the sorted constraint and contact indices per split
        // (Zolt: offsets in mContactAndConstraintIndices instead of pointers)
        var constraint_buffer_cur: [num_splits]u32 = undefined;
        var contact_buffer_cur: [num_splits]u32 = undefined;
        var s: u32 = 0;
        while (s < num_splits) : (s += 1) {
            // If this split doesn't contain enough constraints and contacts, we will combine it with the non parallel split
            if (num_constraints_in_split[s] + num_contacts_in_split[s] < split_combine_treshold and s < non_parallel_split_idx) { // The non-parallel split cannot merge into itself
                // Remap it
                split_remap_table[s] = non_parallel_split_idx;

                // Add the counts to the non parallel split
                num_contacts_in_split[non_parallel_split_idx] += num_contacts_in_split[s];
                num_constraints_in_split[non_parallel_split_idx] += num_constraints_in_split[s];
            } else {
                // This split is valid, map it to the next empty slot
                var target_split: u32 = undefined;
                if (s < non_parallel_split_idx) {
                    target_split = splits.num_splits;
                    splits.num_splits += 1;
                } else target_split = non_parallel_split_idx;
                const split = &splits.splits[target_split];
                split_remap_table[s] = target_split;

                // Allocate space for contacts
                split.contact_buffer_begin = offset;
                split.contact_buffer_end = split.contact_buffer_begin + num_contacts_in_split[s];

                // Allocate space for constraints
                split.constraint_buffer_begin = split.contact_buffer_end;
                split.constraint_buffer_end = split.constraint_buffer_begin + num_constraints_in_split[s];

                // Store start for each split
                contact_buffer_cur[target_split] = split.contact_buffer_begin;
                constraint_buffer_cur[target_split] = split.constraint_buffer_begin;

                // Update offset
                offset = split.constraint_buffer_end;
            }
        }

        // Split the contacts
        const indices = self.contact_and_constraint_indices.items;
        for (contacts, contact_split_idx) |c, split_idx| {
            const split = split_remap_table[split_idx];
            indices[contact_buffer_cur[split]] = c;
            contact_buffer_cur[split] += 1;
        }

        // Split the constraints
        for (constraints, constraint_split_idx) |c, split_idx| {
            const split = split_remap_table[split_idx];
            indices[constraint_buffer_cur[split]] = c;
            constraint_buffer_cur[split] += 1;
        }

        if (Core.enable_asserts) {
            s = 0;
            while (s < num_splits) : (s += 1) {
                // If there are no more splits, process the non-parallel split
                if (s >= splits.num_splits)
                    s = non_parallel_split_idx;

                // Check that we wrote all elements
                const split = &splits.splits[s];
                std.debug.assert(contact_buffer_cur[s] == split.contact_buffer_end);
                std.debug.assert(constraint_buffer_cur[s] == split.constraint_buffer_end);
            }

            if (builtin.mode == .Debug) {
                // Validate that the splits are indeed not touching the same body
                // (Zolt: Jolt fills an Array<bool> of mNumActiveBodies entries per split. The split masks of the bodies
                // in this island are not needed anymore, so bit s of a mask marks the body as used by split s: the same
                // check without a heap allocation. Every active dynamic body of a contact in this island is a body of
                // this island, so only masks that were reset above are touched.)
                for (island_bodies) |b|
                    split_masks[bodies[b.getIndex()].getBody().getIndexInActiveBodiesInternal()] = 0;
                s = 0;
                while (s < splits.num_splits) : (s += 1) {
                    const body_used: SplitMask = @as(SplitMask, 1) << @intCast(s);

                    // Validate contacts
                    const split_contacts = splits.getContactsInSplit(s);
                    for (indices[split_contacts.begin..split_contacts.end]) |c| {
                        const affected_bodies = contact_manager.getAffectedBodies(c);
                        const body1: *const Body = affected_bodies.body1;
                        const body2: *const Body = affected_bodies.body2;

                        const idx1 = body1.getIndexInActiveBodiesInternal();
                        if (idx1 != Body.inactive_index and body1.isDynamic()) {
                            std.debug.assert(split_masks[idx1] & body_used == 0);
                            split_masks[idx1] |= body_used;
                        }

                        const idx2 = body2.getIndexInActiveBodiesInternal();
                        if (idx2 != Body.inactive_index and body2.isDynamic()) {
                            std.debug.assert(split_masks[idx2] & body_used == 0);
                            split_masks[idx2] |= body_used;
                        }
                    }
                }
            }
        }

        // Allow other threads to pick up this split island now
        splits.startFirstBatch();
        return true;
    }

    /// Fetch the next batch to process, returns a handle in outSplitIslandIndex that must be provided to MarkBatchProcessed when complete
    /// (Zolt: the batch is returned in `out_batch` when the status is batch_retrieved)
    pub fn fetchNextBatch(self: *LargeIslandSplitter, out_batch: *Batch) Status {
        // We can't be done when all islands haven't been submitted yet
        const num_splits_created = self.next_split_island.load(.acquire);
        var all_done = num_splits_created == self.num_split_islands;

        // Loop over all split islands to find work
        var batch: Splits.SplitBatch = undefined;
        for (self.split_islands.items[0..num_splits_created], 0..) |*s, i| {
            switch (s.fetchNextBatch(&batch)) {
                .all_batches_done => {},

                .waiting_for_batch => all_done = false,

                .batch_retrieved => {
                    const indices = self.contact_and_constraint_indices.items;
                    out_batch.* = .{
                        .split_island_index = @intCast(i),
                        .constraints = indices[batch.constraints_begin..batch.constraints_end],
                        .contacts = indices[batch.contacts_begin..batch.contacts_end],
                        .first_iteration = batch.first_iteration,
                        .parallel = batch.parallel,
                    };
                    return .batch_retrieved;
                },
            }
        }

        return if (all_done) .all_batches_done else .waiting_for_batch;
    }

    /// Mark a batch as processed
    /// (Zolt: `constraints` / `contacts` are the slices of the batch, Jolt's begin / end pointers)
    pub fn markBatchProcessed(self: *LargeIslandSplitter, split_island_index: u32, constraints: []const u32, contacts: []const u32, out_last_iteration: *bool, out_final_batch: *bool) void {
        const num_items_processed: u32 = @intCast(constraints.len + contacts.len);

        std.debug.assert(split_island_index < self.next_split_island.load(.monotonic));
        const splits = &self.split_islands.items[split_island_index];
        splits.markBatchProcessed(num_items_processed, out_last_iteration, out_final_batch);
    }

    /// Zolt addition (Phase 13, class A): the word that holds the SIMD plan of the batch whose contacts are `contacts`
    /// (ContactConstraintSimd.planBatch, not empty): the entry of contact_and_constraints_split_idx at the index of the
    /// batch's first contact in contact_and_constraint_indices. splitIsland writes the entries of its island before the
    /// island's first batch and nothing reads them afterwards (see the top of this file).
    pub fn batchPlanWord(self: *LargeIslandSplitter, contacts: []const u32) *u32 {
        const indices = self.contact_and_constraint_indices.items;
        std.debug.assert(contacts.len > 0 and @intFromPtr(contacts.ptr) >= @intFromPtr(indices.ptr) and @intFromPtr(contacts.ptr + contacts.len) <= @intFromPtr(indices.ptr + indices.len));
        const index = (@intFromPtr(contacts.ptr) - @intFromPtr(indices.ptr)) / @sizeOf(u32);
        return &self.contact_and_constraints_split_idx.items[index];
    }

    /// Get the island index of the island that was split for a particular split island index
    pub fn getIslandIndex(self: *const LargeIslandSplitter, split_island_index: u32) u32 {
        std.debug.assert(split_island_index < self.num_split_islands);
        return self.split_islands.items[split_island_index].island_index;
    }

    /// Prepare the island splitter for iterating over the split islands again for position solving. Marks all batches as startable.
    pub fn prepareForSolvePositions(self: *LargeIslandSplitter) void {
        // (Zolt: the split islands of Prepare, mNumSplitIslands entries)
        for (self.split_islands.items) |*s| {
            // Set the number of iterations to the number of position steps
            s.num_iterations = s.num_position_steps;

            // We can start again from the first batch
            s.startFirstBatch();
        }
    }

    /// Reset the island splitter
    pub fn reset(self: *LargeIslandSplitter, temp_allocator: TempAllocator) void {
        // Everything should have been used
        std.debug.assert(self.contact_and_constraints_next_free.load(.monotonic) == self.contact_and_constraints_size);
        std.debug.assert(self.next_split_island.load(.monotonic) == self.num_split_islands);

        self.freeBuffers(temp_allocator);
    }

    /// Zolt addition (SimulationArchitecture D5): reset the island splitter after an out of memory error poisoned the
    /// update. Frees exactly what Prepare allocated, like `reset`, but without Reset's check that every large island was
    /// split: the SolveVelocityConstraints jobs that split them may have skipped their work.
    pub fn resetAfterOutOfMemory(self: *LargeIslandSplitter, temp_allocator: TempAllocator) void {
        self.freeBuffers(temp_allocator);

        // Prepare sets mNumActiveBodies before it allocates the split masks. When that allocation failed, the frees
        // above did not reset it (Jolt cannot get here: its Reset always ends with mNumActiveBodies == 0)
        self.num_active_bodies = 0;
    }

    /// The frees of Reset in Jolt's order (the reverse of the allocations in Prepare), nothing for an empty TempArray
    fn freeBuffers(self: *LargeIslandSplitter, temp_allocator: TempAllocator) void {
        // Free split islands
        if (self.num_split_islands > 0) {
            self.split_islands.free(temp_allocator);

            self.num_split_islands = 0;
            self.next_split_island.store(0, .monotonic);
        }

        // Free contact and constraint buffers
        if (self.contact_and_constraints_size > 0) {
            self.contact_and_constraint_indices.free(temp_allocator);

            self.contact_and_constraints_split_idx.free(temp_allocator);

            self.contact_and_constraints_size = 0;
            self.contact_and_constraints_next_free.store(0, .monotonic);
        }

        // Free split masks
        if (!self.split_masks.isEmpty()) {
            self.split_masks.free(temp_allocator);

            self.num_active_bodies = 0;
        }
    }
};

/// A split island built by hand for the tests of the batch protocol: `parallel` and `non_parallel` give the number of
/// (constraints, contacts) per split, laid out back to back from offset 0 like SplitIsland does
fn testSplits(parallel: []const [2]u32, non_parallel: [2]u32, num_iterations: i32) LargeIslandSplitter.Splits {
    var splits: LargeIslandSplitter.Splits = undefined;
    splits.island_index = 7;
    splits.num_splits = @intCast(parallel.len);
    splits.num_iterations = num_iterations;
    splits.num_velocity_steps = num_iterations - 1;
    splits.num_position_steps = 0;
    splits.items_processed = .init(0);
    var offset: u32 = 0;
    for (0..parallel.len + 1) |i| {
        const counts = if (i < parallel.len) parallel[i] else non_parallel;
        const split = &splits.splits[if (i < parallel.len) i else LargeIslandSplitter.non_parallel_split_idx];
        split.contact_buffer_begin = offset;
        split.contact_buffer_end = offset + counts[1];
        split.constraint_buffer_begin = split.contact_buffer_end;
        split.constraint_buffer_end = split.constraint_buffer_begin + counts[0];
        offset = split.constraint_buffer_end;
    }
    splits.resetStatus();
    return splits;
}

/// Fetch a batch and compare it with `expected` (constraints begin / end, contacts begin / end), null: waiting.
/// The batch must be parallel unless it is the non parallel split (contacts and constraints at the end of the buffer).
fn expectFetch(splits: *LargeIslandSplitter.Splits, expected: ?[4]u32, first_iteration: bool) !void {
    var batch: LargeIslandSplitter.Splits.SplitBatch = undefined;
    const status = splits.fetchNextBatch(&batch);
    if (expected) |e| {
        try std.testing.expectEqual(LargeIslandSplitter.Status.batch_retrieved, status);
        try std.testing.expectEqual(e, [4]u32{ batch.constraints_begin, batch.constraints_end, batch.contacts_begin, batch.contacts_end });
        try std.testing.expectEqual(first_iteration, batch.first_iteration);
        const non_parallel = &splits.splits[LargeIslandSplitter.non_parallel_split_idx];
        try std.testing.expectEqual(!(e[0] == non_parallel.constraint_buffer_begin and e[2] == non_parallel.contact_buffer_begin), batch.parallel);
    } else try std.testing.expectEqual(LargeIslandSplitter.Status.waiting_for_batch, status);
}

/// Mark `n` items as processed and compare the flags
fn expectMark(splits: *LargeIslandSplitter.Splits, n: u32, last_iteration: bool, final_batch: bool) !void {
    var last: bool = undefined;
    var final: bool = undefined;
    splits.markBatchProcessed(n, &last, &final);
    try std.testing.expectEqual(last_iteration, last);
    try std.testing.expectEqual(final_batch, final);
}

test "LargeIslandSplitter.batchPlanWord is the split index entry of the batch's first contact" {
    var indices: [40]u32 = undefined;
    var split_idx: [40]u32 = undefined;
    var splitter: LargeIslandSplitter = .empty;
    splitter.contact_and_constraint_indices = .{ .items = &indices };
    splitter.contact_and_constraints_split_idx = .{ .items = &split_idx };
    defer {
        splitter.contact_and_constraint_indices = .empty;
        splitter.contact_and_constraints_split_idx = .empty;
    }
    try std.testing.expectEqual(&split_idx[0], splitter.batchPlanWord(indices[0..16]));
    try std.testing.expectEqual(&split_idx[16], splitter.batchPlanWord(indices[16..32]));
    try std.testing.expectEqual(&split_idx[37], splitter.batchPlanWord(indices[37..40]));
}

test "LargeIslandSplitter.Splits status bits" {
    const Splits = LargeIslandSplitter.Splits;
    const status = (@as(u64, 0x1234) << Splits.status_iteration_shift) | (@as(u64, 31) << Splits.status_split_shift) | 0x89abcdef;
    try std.testing.expectEqual(@as(i32, 0x1234), Splits.getIteration(status));
    try std.testing.expectEqual(@as(u32, 31), Splits.getSplit(status));
    try std.testing.expectEqual(@as(u32, 0x89abcdef), Splits.getItem(status));
    try std.testing.expectEqual(@as(u64, 0xffffffffffffffff), Splits.status_iteration_mask | Splits.status_split_mask | Splits.status_item_mask);
    try std.testing.expectEqual(@as(u32, 32), LargeIslandSplitter.num_splits);
    try std.testing.expectEqual(@as(u32, 31), LargeIslandSplitter.non_parallel_split_idx);
}

test "LargeIslandSplitter.Splits batch protocol" {
    // Split 0: 20 constraints + 10 contacts, split 1: empty, split 2: only contacts, split 3: the first batch ends at the
    // last constraint; the non parallel split: 3 constraints + 2 contacts. Layout:
    // split 0 contacts [0, 10) constraints [10, 30), split 1 [30, 30), split 2 contacts [30, 70),
    // split 3 contacts [70, 74) constraints [74, 90), non parallel contacts [90, 92) constraints [92, 95)
    var splits = testSplits(&.{ .{ 20, 10 }, .{ 0, 0 }, .{ 0, 40 }, .{ 16, 4 } }, .{ 3, 2 }, 2);

    // The splits are still being built
    try expectFetch(&splits, null, false);

    splits.startFirstBatch();
    for (0..2) |iteration| {
        const first = iteration == 0;
        const last = iteration == 1;

        // Split 0: two batches can be out at the same time, a third one waits until the split is done
        try expectFetch(&splits, .{ 10, 26, 0, 0 }, first); // Only constraints
        try expectFetch(&splits, .{ 26, 30, 0, 10 }, first); // Partially constraints, partially contacts
        try expectFetch(&splits, null, first);
        try expectMark(&splits, 16, last, false);
        try expectFetch(&splits, null, first);
        try expectMark(&splits, 14, last, false);

        // Split 1 is empty and skipped, split 2 has only contacts
        try expectFetch(&splits, .{ 0, 0, 30, 46 }, first);
        try expectMark(&splits, 16, last, false);
        try expectFetch(&splits, .{ 0, 0, 46, 62 }, first);
        try expectFetch(&splits, .{ 0, 0, 62, 70 }, first);
        try expectMark(&splits, 8, last, false);
        try expectMark(&splits, 16, last, false);

        // Split 3: the first batch ends exactly at the last constraint (an empty contact range at the contacts)
        try expectFetch(&splits, .{ 74, 90, 70, 70 }, first);
        try expectFetch(&splits, .{ 0, 0, 70, 74 }, first);
        try expectMark(&splits, 16, last, false);
        try expectMark(&splits, 4, last, false);

        // The non parallel split is one batch for one thread
        try expectFetch(&splits, .{ 92, 95, 90, 92 }, first);
        try expectFetch(&splits, null, first);
        try expectMark(&splits, 5, last, last);
    }

    var batch: LargeIslandSplitter.Splits.SplitBatch = undefined;
    try std.testing.expectEqual(LargeIslandSplitter.Status.all_batches_done, splits.fetchNextBatch(&batch));
    try std.testing.expectEqual(@as(u32, 0), splits.items_processed.load(.monotonic));

    // No iterations (0 position steps): done right away
    splits.num_iterations = 0;
    splits.startFirstBatch();
    try std.testing.expectEqual(LargeIslandSplitter.Status.all_batches_done, splits.fetchNextBatch(&batch));

    // Only the non parallel split: it starts there and every iteration is one batch
    var non_parallel_only = testSplits(&.{}, .{ 100, 40 }, 3);
    non_parallel_only.startFirstBatch();
    for (0..3) |iteration| {
        try expectFetch(&non_parallel_only, .{ 40, 140, 0, 40 }, iteration == 0);
        try expectMark(&non_parallel_only, 140, iteration == 2, iteration == 2);
    }
    try std.testing.expectEqual(LargeIslandSplitter.Status.all_batches_done, non_parallel_only.fetchNextBatch(&batch));
}
