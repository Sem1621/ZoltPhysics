//! Port of: Jolt/Physics/LargeIslandSplitter.h, Jolt/Physics/LargeIslandSplitter.cpp
//! Status: stub
//! Missing: splitting large islands (Wave A, A5): AssignSplit, AssignToNonParallelSplit, SplitIsland,
//! Splits::FetchNextBatch / MarkBatchProcessed and the batch protocol of FetchNextBatch / MarkBatchProcessed, the
//! split buffers of Prepare / Reset.
//!
//! F2 stub with the final names and signatures of what PhysicsSystem and TwoBodyConstraint use (SimulationArchitecture
//! section 2). Without contacts (A4) an island has at most the non-contact constraints, so on the gravity-only path no
//! island reaches `large_island_treshold`: `prepare` counts the islands like Jolt and finds nothing to split (it panics
//! when it would have to allocate the split buffers), `fetchNextBatch` reports `all_batches_done` for zero split
//! islands, `prepareForSolvePositions` and `reset` have nothing to do. Everything that needs a split island panics with
//! "LargeIslandSplitter: A5".

const std = @import("std");
const TempAllocator = @import("../Core/TempAllocator.zig").TempAllocator;
const Body = @import("Body/Body.zig").Body;
const BodyManager = @import("Body/BodyManager.zig").BodyManager;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const Constraint = @import("Constraints/Constraint.zig").Constraint;
const ContactConstraintManager = @import("Constraints/ContactConstraintManager.zig").ContactConstraintManager;

/// Assigns bodies in large islands to multiple groups that can run in parallel
///
/// This basically implements what is described in: High-Performance Physical Simulations on Next-Generation Architecture with Many Cores by Chen et al.
/// See: http://web.eecs.umich.edu/~msmelyan/papers/physsim_onmanycore_itj.pdf section "PARALLELIZATION METHODOLOGY"
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const LargeIslandSplitter = struct {
    const SplitMask = u32;

    pub const num_splits: u32 = @bitSizeOf(SplitMask);
    pub const non_parallel_split_idx: u32 = num_splits - 1;
    /// If the number of constraints + contacts in an island is larger than this, we will try to split the island
    pub const large_island_treshold: u32 = 128;

    /// Status code for retrieving a batch
    pub const Status = enum {
        /// Work is expected to be available later
        waiting_for_batch,
        /// Work is being returned
        batch_retrieved,
        /// No further work is expected from this
        all_batches_done,
    };

    /// A batch returned by fetchNextBatch (Jolt's out parameters)
    pub const Batch = struct {
        /// Handle that must be provided to markBatchProcessed when complete
        split_island_index: u32,
        constraints: []u32,
        contacts: []u32,
        first_iteration: bool,
    };

    /// Cached number of active bodies
    num_active_bodies: u32 = 0,

    /// Total size of mContactAndConstraintsSplitIdx and mContactAndConstraintIndices
    contact_and_constraints_size: u32 = 0,

    /// Total number of islands that required splitting
    num_split_islands: u32 = 0,

    /// Next split island to pick from mSplitIslands
    next_split_island: std.atomic.Value(u32) = .init(0),

    /// Not used yet (Jolt's constructor)
    pub const empty: LargeIslandSplitter = .{};

    /// Destructor
    pub fn deinit(self: *LargeIslandSplitter) void {
        std.debug.assert(self.contact_and_constraints_size == 0);
        std.debug.assert(self.num_split_islands == 0);
        self.* = .empty;
    }

    /// Prepare the island splitter by allocating memory
    pub fn prepare(self: *LargeIslandSplitter, island_builder: *const IslandBuilder, num_active_bodies: u32, temp_allocator: TempAllocator) TempAllocator.Error!void {
        _ = num_active_bodies;
        _ = temp_allocator;

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

        if (self.contact_and_constraints_size > 0)
            @panic("LargeIslandSplitter.prepare: splitting large islands needs A5");
    }

    /// Assign two bodies to a split. Returns the split index.
    pub fn assignSplit(self: *LargeIslandSplitter, body1: *const Body, body2: *const Body) u32 {
        _ = .{ self, body1, body2 };
        @panic("LargeIslandSplitter.assignSplit: A5");
    }

    /// Force a body to be in a non parallel split. Returns the split index.
    pub fn assignToNonParallelSplit(self: *LargeIslandSplitter, body: *const Body) u32 {
        _ = .{ self, body };
        @panic("LargeIslandSplitter.assignToNonParallelSplit: A5");
    }

    /// Splits up an island, the created splits will be added to the list of batches and can be fetched with FetchNextBatch. Returns false if the island did not need splitting.
    /// `steps_calculator`: pointer to a CalculateSolverSteps
    pub fn splitIsland(self: *LargeIslandSplitter, island_index: u32, island_builder: *const IslandBuilder, body_manager: *const BodyManager, contact_manager: *const ContactConstraintManager, active_constraints: []const *Constraint, steps_calculator: anytype) bool {
        _ = .{ self, body_manager, contact_manager, active_constraints, steps_calculator };

        // Check if it exceeds the threshold
        const island_size = island_builder.getContactsInIsland(island_index).len + island_builder.getConstraintsInIsland(island_index).len;
        if (island_size < large_island_treshold)
            return false;

        @panic("LargeIslandSplitter.splitIsland: A5");
    }

    /// Fetch the next batch to process, returns a handle in outSplitIslandIndex that must be provided to MarkBatchProcessed when complete
    /// (Zolt: the batch is returned in `out_batch` when the status is batch_retrieved)
    pub fn fetchNextBatch(self: *LargeIslandSplitter, out_batch: *Batch) Status {
        _ = out_batch;

        // We can't be done when all islands haven't been submitted yet
        const num_splits_created = self.next_split_island.load(.acquire);
        const all_done = num_splits_created == self.num_split_islands;

        // Loop over all split islands to find work (stub: prepare panics before a split island exists)
        if (num_splits_created > 0)
            @panic("LargeIslandSplitter.fetchNextBatch: A5");

        return if (all_done) .all_batches_done else .waiting_for_batch;
    }

    /// Mark a batch as processed
    pub fn markBatchProcessed(self: *LargeIslandSplitter, split_island_index: u32, constraints: []const u32, contacts: []const u32, out_last_iteration: *bool, out_final_batch: *bool) void {
        _ = .{ self, split_island_index, constraints, contacts, out_last_iteration, out_final_batch };
        @panic("LargeIslandSplitter.markBatchProcessed: A5");
    }

    /// Get the island index of the island that was split for a particular split island index
    pub fn getIslandIndex(self: *const LargeIslandSplitter, split_island_index: u32) u32 {
        std.debug.assert(split_island_index < self.num_split_islands);
        @panic("LargeIslandSplitter.getIslandIndex: A5");
    }

    /// Prepare the island splitter for iterating over the split islands again for position solving. Marks all batches as startable.
    pub fn prepareForSolvePositions(self: *LargeIslandSplitter) void {
        // Stub: prepare panics before a split island exists
        std.debug.assert(self.num_split_islands == 0);
    }

    /// Reset the island splitter
    pub fn reset(self: *LargeIslandSplitter, temp_allocator: TempAllocator) void {
        _ = temp_allocator;

        // Everything should have been used
        std.debug.assert(self.next_split_island.load(.monotonic) == self.num_split_islands);

        // Stub: no split islands, contact and constraint buffers or split masks were allocated
        std.debug.assert(self.num_split_islands == 0);
        std.debug.assert(self.contact_and_constraints_size == 0);
        self.num_active_bodies = 0;
    }
};
