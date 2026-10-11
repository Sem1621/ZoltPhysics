//! Port of: Jolt/Physics/IslandBuilder.h, Jolt/Physics/IslandBuilder.cpp
//! Status: complete
//! Not ported: JPH_VALIDATE_ISLAND_BUILDER (`ValidateIslands`, `LinkValidation`: a debug define that is commented out
//! in Jolt and only traces the links of a failed validation)
//!
//! SimulationArchitecture D4 / D5:
//! - LinkBodies is Jolt's lock-free union-find (compare-exchange on the per body link, AtomicMin path shortening, all
//!   `.monotonic` like Jolt's memory_order_relaxed), run concurrently by the FindCollisions and
//!   BuildIslandsFromConstraints jobs. The resulting components do not depend on the order of the links.
//! - The per step arrays are `TempArray`s: a failed allocation leaves an empty array that consumed nothing, so
//!   `resetIslands` frees exactly what was allocated, in Jolt's order (the reverse of the allocations), also after an
//!   out of memory error. `prepareContactConstraints`, `prepareNonContactConstraints` and `finalize` return
//!   `TempAllocator.Error` (Jolt aborts in the temp allocator).
//! - Zolt addition (D5): BuildBodyIslands restores the body links for the next update. When the update was poisoned
//!   before or during Finalize, `resetIslands` restores them instead (`links_need_reset`).
//! - `GetBodiesInIsland(index, outBegin, outEnd)` returns the slice; `GetConstraintsInIsland` /
//!   `GetContactsInIsland` return the slice, empty where Jolt returns false.
//! - JPH_TRACK_SIMULATION_STATS (`IslandStats`, `GetIslandStats`) follows MotionProperties' comptime switch
//!   `track_simulation_stats` (off, like Jolt's default): `island_stats` is `void` while it is off.

const std = @import("std");
const Allocator = std.mem.Allocator;
const atomicMin = @import("../Core/Atomics.zig").atomicMin;
const TempAllocatorFile = @import("../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const quickSort = @import("../Core/QuickSort.zig").quickSort;
const BodyID = @import("Body/BodyID.zig").BodyID;
const MotionPropertiesFile = @import("Body/MotionProperties.zig");
const MotionProperties = MotionPropertiesFile.MotionProperties;

/// JPH_TRACK_SIMULATION_STATS (the switch of MotionProperties)
const track_simulation_stats = MotionPropertiesFile.track_simulation_stats;

/// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
pub const IslandBuilder = struct {
    /// Per island simulation statistics (JPH_TRACK_SIMULATION_STATS)
    pub const IslandStats = struct {
        velocity_constraint_ticks: std.atomic.Value(u64) = .init(0),
        position_constraint_ticks: std.atomic.Value(u64) = .init(0),
        update_bounds_ticks: std.atomic.Value(u64) = .init(0),
        num_velocity_steps: u8 = 0,
        /// Tracking this a 2nd time since IslandBuilder::mNumPositionSteps is not filled in when there are no constraints or for large islands.
        num_position_steps: u8 = 0,
        is_large_island: bool = false,
    };

    /// Intermediate data structure that for each body keeps track what the lowest index of the body is that it is connected to
    const BodyLink = struct {
        /// An index in mBodyLinks pointing to another body in this island with a lower index than this body
        linked_to: std.atomic.Value(u32),
        /// The island index of this body (filled in during Finalize)
        island_index: u32,
    };

    /// Allocates the body links (Jolt: new BodyLink [])
    allocator: Allocator = undefined,

    // Intermediate data
    /// Maps bodies to the first body in the island
    body_links: []BodyLink = &.{},
    /// Maps constraint index to body index (which maps to island index)
    constraint_links: TempArray(u32) = .empty,
    /// Maps contact constraint index to body index (which maps to island index)
    contact_links: TempArray(u32) = .empty,

    // Final data
    /// Bodies ordered by island
    body_islands: TempArray(BodyID) = .empty,
    /// End index of each body island
    body_island_ends: TempArray(u32) = .empty,

    /// Constraints ordered by island
    constraint_islands: TempArray(u32) = .empty,
    /// End index of each constraint island
    constraint_island_ends: TempArray(u32) = .empty,

    /// Contacts ordered by island
    contact_islands: TempArray(u32) = .empty,
    /// End index of each contact island
    contact_island_ends: TempArray(u32) = .empty,

    /// A list of island indices in order of most constraints first
    islands_sorted: TempArray(u32) = .empty,

    /// Number of position steps for each island
    num_position_steps: TempArray(u8) = .empty,

    /// Per island statistics (JPH_TRACK_SIMULATION_STATS)
    island_stats: if (track_simulation_stats) TempArray(IslandStats) else void = if (track_simulation_stats) .empty else {},

    // Counters
    /// Maximum size of the active bodies list (see BodyManager::mActiveBodies)
    max_active_bodies: u32 = 0,
    /// Number of active bodies passed to
    num_active_bodies: u32 = 0,
    /// Size of the constraint list (see ConstraintManager::mConstraints)
    num_constraints: u32 = 0,
    /// Maximum amount of contacts supported
    max_contacts: u32 = 0,
    /// Size of the contacts list (see ContactConstraintManager::mNumConstraints)
    num_contacts: u32 = 0,
    /// Final number of islands
    num_islands: u32 = 0,

    /// Zolt (D5): set while the links may differ from the identity; BuildBodyIslands restores them, resetIslands does
    /// when an out of memory error stopped the update before
    links_need_reset: bool = false,

    /// Not initialized (Jolt's constructor), call init
    pub const empty: IslandBuilder = .{};

    /// Destructor
    pub fn deinit(self: *IslandBuilder) void {
        std.debug.assert(self.constraint_links.isEmpty());
        std.debug.assert(self.contact_links.isEmpty());
        std.debug.assert(self.body_islands.isEmpty());
        std.debug.assert(self.body_island_ends.isEmpty());
        std.debug.assert(self.constraint_islands.isEmpty());
        std.debug.assert(self.constraint_island_ends.isEmpty());
        std.debug.assert(self.contact_islands.isEmpty());
        std.debug.assert(self.contact_island_ends.isEmpty());
        std.debug.assert(self.islands_sorted.isEmpty());

        if (self.body_links.len > 0)
            self.allocator.free(self.body_links);
        self.* = .empty;
    }

    /// Initialize the island builder with the maximum amount of bodies that could be active
    pub fn init(self: *IslandBuilder, allocator: Allocator, max_active_bodies: u32) Allocator.Error!void {
        // Link each body to itself, BuildBodyIslands() will restore this so that we don't need to do this each step
        std.debug.assert(self.body_links.len == 0);
        const body_links = try allocator.alloc(BodyLink, max_active_bodies);
        for (body_links, 0..) |*l, i|
            l.* = .{ .linked_to = .init(@intCast(i)), .island_index = undefined };

        self.allocator = allocator;
        self.max_active_bodies = max_active_bodies;
        self.body_links = body_links;
    }

    /// Prepare for simulation step by allocating space for the contact constraints
    pub fn prepareContactConstraints(self: *IslandBuilder, max_contacts: u32, temp_allocator: TempAllocator) TempAllocator.Error!void {
        // Need to call Init first
        std.debug.assert(self.body_links.len == self.max_active_bodies);

        // Check that the builder has been reset
        std.debug.assert(self.num_contacts == 0);
        std.debug.assert(self.num_islands == 0);

        // Zolt (D5): from here on resetIslands restores the links unless BuildBodyIslands did
        self.links_need_reset = true;

        // Create contact link buffer, not initialized so each contact needs to be explicitly set
        std.debug.assert(self.contact_links.isEmpty());
        self.contact_links = try .alloc(temp_allocator, max_contacts);
        self.max_contacts = max_contacts;
    }

    /// Prepare for simulation step by allocating space for the non-contact constraints
    pub fn prepareNonContactConstraints(self: *IslandBuilder, num_constraints: u32, temp_allocator: TempAllocator) TempAllocator.Error!void {
        // Need to call Init first
        std.debug.assert(self.body_links.len == self.max_active_bodies);

        // Check that the builder has been reset
        std.debug.assert(self.num_constraints == 0);
        std.debug.assert(self.num_islands == 0);

        // Store number of constraints
        self.num_constraints = num_constraints;

        // Create constraint link buffer, not initialized so each constraint needs to be explicitly set
        std.debug.assert(self.constraint_links.isEmpty());
        self.constraint_links = try .alloc(temp_allocator, num_constraints);
    }

    /// Returns the index of the lowest body in the group
    fn getLowestBodyIndex(self: *const IslandBuilder, active_body_index: u32) u32 {
        var index = active_body_index;
        while (true) {
            const link_to = self.body_links[index].linked_to.load(.monotonic);
            if (link_to == index)
                break;
            index = link_to;
        }
        return index;
    }

    /// Link two bodies by their index in the BodyManager::mActiveBodies list to form islands
    pub fn linkBodies(self: *IslandBuilder, first: u32, second: u32) void {
        // Both need to be active, we don't want to create an island with static objects
        if (first >= self.max_active_bodies or second >= self.max_active_bodies)
            return;

        // Start the algorithm with the two bodies
        var first_link_to = first;
        var second_link_to = second;

        while (true) {
            // Follow the chain until we get to the body with lowest index
            // If the swap compare below fails, we'll keep searching from the lowest index for the new lowest index
            first_link_to = self.getLowestBodyIndex(first_link_to);
            second_link_to = self.getLowestBodyIndex(second_link_to);

            // If the targets are the same, the bodies are already connected
            if (first_link_to != second_link_to) {
                // We always link the highest to the lowest
                if (first_link_to < second_link_to) {
                    // Attempt to link the second to the first
                    // Since we found this body to be at the end of the chain it must point to itself, and if it
                    // doesn't it has been reparented and we need to retry the algorithm
                    // (Jolt's compare_exchange_weak writes the current value into second_link_to on failure)
                    if (self.body_links[second_link_to].linked_to.cmpxchgWeak(second_link_to, first_link_to, .monotonic, .monotonic)) |current| {
                        second_link_to = current;
                        continue;
                    }
                } else {
                    // Attempt to link the first to the second
                    // Since we found this body to be at the end of the chain it must point to itself, and if it
                    // doesn't it has been reparented and we need to retry the algorithm
                    if (self.body_links[first_link_to].linked_to.cmpxchgWeak(first_link_to, second_link_to, .monotonic, .monotonic)) |current| {
                        first_link_to = current;
                        continue;
                    }
                }
            }

            // Linking succeeded!
            // Chains of bodies can become really long, resulting in an O(N) loop to find the lowest body index
            // to prevent this we attempt to update the link of the bodies that were passed in to directly point
            // to the lowest index that we found. If the value became lower than our lowest link, some other
            // thread must have relinked these bodies in the mean time so we won't update the value.
            const lowest_link_to = @min(first_link_to, second_link_to);
            _ = atomicMin(u32, &self.body_links[first].linked_to, lowest_link_to, .{ .order = .monotonic });
            _ = atomicMin(u32, &self.body_links[second].linked_to, lowest_link_to, .{ .order = .monotonic });
            break;
        }
    }

    /// Link a constraint to a body by their index in the BodyManager::mActiveBodies
    pub fn linkConstraint(self: *IslandBuilder, constraint_index: u32, index_in_active_body_list: u32) void {
        std.debug.assert(constraint_index < self.num_constraints);
        std.debug.assert(index_in_active_body_list != MotionProperties.inactive_index); // Body should be active

        self.constraint_links.items[constraint_index] = index_in_active_body_list;
    }

    /// Link a contact to a body by their index in the BodyManager::mActiveBodies
    pub fn linkContact(self: *IslandBuilder, contact_index: u32, index_in_active_body_list: u32) void {
        std.debug.assert(contact_index < self.max_contacts);
        std.debug.assert(index_in_active_body_list != MotionProperties.inactive_index); // Body should be active

        self.contact_links.items[contact_index] = index_in_active_body_list;
    }

    // Helper functions to build various islands

    fn buildBodyIslands(self: *IslandBuilder, active_bodies: []const BodyID, temp_allocator: TempAllocator) TempAllocator.Error!void {
        const num_active_bodies: u32 = @intCast(active_bodies.len);

        // Store the amount of active bodies
        self.num_active_bodies = num_active_bodies;

        // Create output arrays for body ID's, don't call constructors
        std.debug.assert(self.body_islands.isEmpty());
        self.body_islands = try .alloc(temp_allocator, num_active_bodies);

        // Create output array for start index of each island. At this point we don't know how many islands there will be, but we know it cannot be more than inNumActiveBodies.
        // Note: We allocate 1 extra entry because we always increment the count of the next island.
        const body_island_starts_array: TempArray(u32) = try .alloc(temp_allocator, num_active_bodies + 1);
        const body_island_starts = body_island_starts_array.items;

        // First island always starts at 0
        body_island_starts[0] = 0;

        // Calculate island index for all bodies
        std.debug.assert(self.num_islands == 0);
        var i: u32 = 0;
        while (i < num_active_bodies) : (i += 1) {
            const link = &self.body_links[i];
            const s = link.linked_to.load(.monotonic);
            if (s != i) {
                // Links to another body, take island index from other body (this must have been filled in already since we're looping from low to high)
                std.debug.assert(s < i);
                const island_index = self.body_links[s].island_index;
                link.island_index = island_index;

                // Increment the start of the next island
                body_island_starts[island_index + 1] += 1;
            } else {
                // Does not link to other body, this is the start of a new island
                link.island_index = self.num_islands;
                self.num_islands += 1;

                // Set the start of the next island to 1
                body_island_starts[self.num_islands] = 1;
            }
        }

        // Make the start array absolute (so far we only counted)
        var island: u32 = 1;
        while (island < self.num_islands) : (island += 1)
            body_island_starts[island] += body_island_starts[island - 1];

        // Convert the to a linear list grouped by island
        i = 0;
        while (i < num_active_bodies) : (i += 1) {
            const link = &self.body_links[i];

            // Copy the body to the correct location in the array and increment it
            const start = &body_island_starts[link.island_index];
            self.body_islands.items[start.*] = active_bodies[i];
            start.* += 1;

            // Reset linked to field for the next update
            link.linked_to.store(i, .monotonic);
        }
        self.links_need_reset = false;

        // We should now have a full array
        std.debug.assert(self.num_islands == 0 or body_island_starts[self.num_islands - 1] == num_active_bodies);

        // We've incremented all body indices so that they now point at the end instead of the starts
        std.debug.assert(self.body_island_ends.isEmpty());
        self.body_island_ends = body_island_starts_array;
    }

    fn buildConstraintIslands(self: *const IslandBuilder, constraint_to_body: []const u32, out_constraints: *TempArray(u32), out_constraints_end: *TempArray(u32), temp_allocator: TempAllocator) TempAllocator.Error!void {
        const num_constraints: u32 = @intCast(constraint_to_body.len);

        // Check if there's anything to do
        if (num_constraints == 0)
            return;

        // Create output arrays for constraints
        // Note: For the end indices we allocate 1 extra entry so we don't have to do an if in the inner loop
        // (Zolt: stored in the outputs right away, so resetIslands frees what was allocated when the second one fails)
        std.debug.assert(out_constraints.isEmpty());
        out_constraints.* = try .alloc(temp_allocator, num_constraints);
        std.debug.assert(out_constraints_end.isEmpty());
        out_constraints_end.* = try .alloc(temp_allocator, self.num_islands + 1);
        const constraints = out_constraints.items;
        const constraint_ends = out_constraints_end.items;

        // Reset sizes
        for (constraint_ends) |*e|
            e.* = 0;

        // Loop over array and increment start relative position for the next island
        for (constraint_to_body) |body_idx| {
            const next_island_idx = self.body_links[body_idx].island_index + 1;
            std.debug.assert(next_island_idx <= self.num_islands);
            constraint_ends[next_island_idx] += 1;
        }

        // Make start positions absolute
        var island: u32 = 1;
        while (island < self.num_islands) : (island += 1)
            constraint_ends[island] += constraint_ends[island - 1];

        // Loop over array and collect constraints
        for (constraint_to_body, 0..) |body_idx, constraint| {
            const island_idx = self.body_links[body_idx].island_index;
            constraints[constraint_ends[island_idx]] = @intCast(constraint);
            constraint_ends[island_idx] += 1;
        }
    }

    /// Sorts the islands so that the islands with most constraints go first
    fn sortIslands(self: *IslandBuilder, temp_allocator: TempAllocator) TempAllocator.Error!void {
        if (self.num_contacts > 0 or self.num_constraints > 0) {
            // Allocate mapping table
            std.debug.assert(self.islands_sorted.isEmpty());
            self.islands_sorted = try .alloc(temp_allocator, self.num_islands);

            // Initialize index
            for (self.islands_sorted.items, 0..) |*s, island|
                s.* = @intCast(island);

            // Determine the sum of contact constraints / constraints per island
            var num_constraints_array: TempArray(u32) = try .alloc(temp_allocator, self.num_islands);
            const num_constraints = num_constraints_array.items;
            var island: u32 = 1;
            if (self.num_contacts > 0 and self.num_constraints > 0) {
                num_constraints[0] = self.constraint_island_ends.items[0] + self.contact_island_ends.items[0];
                while (island < self.num_islands) : (island += 1)
                    num_constraints[island] = self.constraint_island_ends.items[island] - self.constraint_island_ends.items[island - 1] + self.contact_island_ends.items[island] - self.contact_island_ends.items[island - 1];
            } else if (self.num_contacts > 0) {
                num_constraints[0] = self.contact_island_ends.items[0];
                while (island < self.num_islands) : (island += 1)
                    num_constraints[island] = self.contact_island_ends.items[island] - self.contact_island_ends.items[island - 1];
            } else {
                num_constraints[0] = self.constraint_island_ends.items[0];
                while (island < self.num_islands) : (island += 1)
                    num_constraints[island] = self.constraint_island_ends.items[island] - self.constraint_island_ends.items[island - 1];
            }

            // Sort so the biggest islands go first, this means that the jobs that take longest will be running
            // first which improves the chance that all jobs finish at the same time.
            quickSort(u32, self.islands_sorted.items, @as([]const u32, num_constraints), struct {
                fn lessThan(counts: []const u32, lhs: u32, rhs: u32) bool {
                    return counts[lhs] > counts[rhs];
                }
            }.lessThan);

            num_constraints_array.free(temp_allocator);
        }
    }

    /// Finalize the islands after all bodies have been Link()-ed
    pub fn finalize(self: *IslandBuilder, active_bodies: []const BodyID, num_contacts: u32, temp_allocator: TempAllocator) TempAllocator.Error!void {
        self.num_contacts = num_contacts;

        try self.buildBodyIslands(active_bodies, temp_allocator);
        try self.buildConstraintIslands(self.constraint_links.items[0..self.num_constraints], &self.constraint_islands, &self.constraint_island_ends, temp_allocator);
        try self.buildConstraintIslands(self.contact_links.items[0..self.num_contacts], &self.contact_islands, &self.contact_island_ends, temp_allocator);
        try self.sortIslands(temp_allocator);

        self.num_position_steps = try .alloc(temp_allocator, self.num_islands);

        if (track_simulation_stats) {
            self.island_stats = try .alloc(temp_allocator, self.num_islands);
            for (self.island_stats.items) |*s|
                s.* = .{};
        }
    }

    /// Get the amount of islands formed
    pub fn getNumIslands(self: *const IslandBuilder) u32 {
        return self.num_islands;
    }

    /// Get iterator for a particular island (the bodies of the island)
    pub fn getBodiesInIsland(self: *const IslandBuilder, island_index: u32) []BodyID {
        std.debug.assert(island_index < self.num_islands);
        const sorted_index = if (!self.islands_sorted.isEmpty()) self.islands_sorted.items[island_index] else island_index;
        const begin = if (sorted_index > 0) self.body_island_ends.items[sorted_index - 1] else 0;
        return self.body_islands.items[begin..self.body_island_ends.items[sorted_index]];
    }

    /// Get iterator for a particular island, return false if there are no constraints
    /// (Zolt: the constraint indices of the island, empty where Jolt returns false)
    pub fn getConstraintsInIsland(self: *const IslandBuilder, island_index: u32) []u32 {
        std.debug.assert(island_index < self.num_islands);
        if (self.num_constraints == 0)
            return &.{};

        const sorted_index = self.islands_sorted.items[island_index];
        const begin = if (sorted_index > 0) self.constraint_island_ends.items[sorted_index - 1] else 0;
        return self.constraint_islands.items[begin..self.constraint_island_ends.items[sorted_index]];
    }

    /// (Zolt: the contact indices of the island, empty where Jolt returns false)
    pub fn getContactsInIsland(self: *const IslandBuilder, island_index: u32) []u32 {
        std.debug.assert(island_index < self.num_islands);
        if (self.num_contacts == 0)
            return &.{};

        const sorted_index = self.islands_sorted.items[island_index];
        const begin = if (sorted_index > 0) self.contact_island_ends.items[sorted_index - 1] else 0;
        return self.contact_islands.items[begin..self.contact_island_ends.items[sorted_index]];
    }

    /// The number of position iterations for each island
    pub fn setNumPositionSteps(self: *IslandBuilder, island_index: u32, num_position_steps: u32) void {
        std.debug.assert(island_index < self.num_islands);
        std.debug.assert(num_position_steps < 256);
        self.num_position_steps.items[island_index] = @intCast(num_position_steps);
    }

    pub fn getNumPositionSteps(self: *const IslandBuilder, island_index: u32) u32 {
        std.debug.assert(island_index < self.num_islands);
        return self.num_position_steps.items[island_index];
    }

    /// Tracks simulation stats per island (JPH_TRACK_SIMULATION_STATS: only declared when `track_simulation_stats` is on)
    pub const getIslandStats = if (track_simulation_stats) getIslandStatsImpl else {};

    fn getIslandStatsImpl(self: *IslandBuilder, island_index: u32) *IslandStats {
        return &self.island_stats.items[island_index];
    }

    /// After you're done calling the three functions above, call this function to free associated data
    /// (Zolt: empty arrays are skipped, so this is correct after an out of memory error too)
    pub fn resetIslands(self: *IslandBuilder, temp_allocator: TempAllocator) void {
        if (track_simulation_stats)
            self.island_stats.free(temp_allocator);

        self.num_position_steps.free(temp_allocator);

        self.islands_sorted.free(temp_allocator);

        self.contact_island_ends.free(temp_allocator);
        self.contact_islands.free(temp_allocator);

        self.constraint_island_ends.free(temp_allocator);
        self.constraint_islands.free(temp_allocator);

        self.body_island_ends.free(temp_allocator);
        self.body_islands.free(temp_allocator);

        self.constraint_links.free(temp_allocator);

        self.contact_links.free(temp_allocator);

        // Zolt (D5): a Finalize that did not run to the end did not restore the links
        if (self.links_need_reset) {
            for (self.body_links, 0..) |*l, i|
                l.linked_to.store(@intCast(i), .monotonic);
            self.links_need_reset = false;
        }

        self.num_active_bodies = 0;
        self.num_constraints = 0;
        self.max_contacts = 0;
        self.num_contacts = 0;
        self.num_islands = 0;
    }
};
