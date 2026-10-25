//! Zolt addition, no Jolt file: tests of LargeIslandSplitter (SimulationArchitecture D5)
//! Status: complete
//!
//! Jolt has no unit test for the LargeIslandSplitter. These tests drive the splitter like PhysicsSystem's
//! SolveVelocityConstraints / SolvePositionConstraints jobs do, on bodies of a BodyManager, constraints
//! (`zolt.test_constraints.TestConstraint` and a derived constraint that forces its first body into the non parallel
//! split, like Jolt's VehicleConstraint) and contacts as body pairs (a stand in for ContactConstraintManager's
//! GetAffectedBodies resolves the contact constraint offsets), with the islands of the IslandBuilder:
//! - every constraint and contact against an independent model of the split assignment (the greedy split masks, the
//!   merge of small splits into the non parallel split, the buffer layout) and of the iteration counts
//!   (CalculateSolverSteps over the bodies of the contacts and the constraints);
//! - the batch sequence that FetchNextBatch hands out single threaded (batches of 16 items, the non parallel split in
//!   one batch, empty splits skipped, the first / last iteration and final batch flags) for the velocity and the position
//!   steps;
//! - several threads that split the islands and process the batches concurrently: every item is processed exactly the
//!   right number of times and no dynamic body is in two batches at the same time;
//! - Prepare's ResetStatus (a split island that was claimed but not started yet hands out no batch), Reset,
//!   ResetAfterOutOfMemory and every temp allocation failing in turn.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const BodyTestContext = @import("BodyTestContext.zig").BodyTestContext;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Body = zolt.Body;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const Shape = zolt.Shape;
const Constraint = zolt.Constraint;
const TwoBodyConstraint = zolt.TwoBodyConstraint;
const IslandBuilder = zolt.IslandBuilder;
const LargeIslandSplitter = zolt.LargeIslandSplitter;
const CalculateSolverSteps = zolt.CalculateSolverSteps;
const PhysicsSettings = zolt.PhysicsSettings;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const FailingTempAllocator = zolt.FailingTempAllocator;
const MotionType = zolt.MotionType;
const TestConstraint = zolt.test_constraints.TestConstraint;
const TestConstraintSettings = zolt.test_constraints.TestConstraintSettings;

const Splits = LargeIslandSplitter.Splits;
const non_parallel_split_idx = LargeIslandSplitter.non_parallel_split_idx;
const num_splits = LargeIslandSplitter.num_splits;
const large_island_treshold = LargeIslandSplitter.large_island_treshold;

/// LargeIslandSplitter::cBatchSize and cSplitCombineTreshold (private in Jolt)
const batch_size: u32 = 16;
const split_combine_treshold: u32 = 32;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualSlices = std.testing.expectEqualSlices;

/// A body of a test scene
const BodySpec = struct {
    motion_type: MotionType = .dynamic,
    /// Activate after creation (the constraints and contacts activate a dynamic body that is not active, like Jolt)
    activate: bool = true,
    velocity_steps: u32 = 0,
    position_steps: u32 = 0,
};

/// A constraint of a test scene (indices into the bodies of the scene)
const ConstraintSpec = struct {
    body1: u32,
    body2: u32,
    velocity_steps: u32 = 0,
    position_steps: u32 = 0,
    /// A constraint that forces its first body into the non parallel split (like Jolt's VehicleConstraint)
    non_parallel: bool = false,
};

/// The bodies of a contact constraint (ContactConstraintManager.AffectedBodies)
const ContactPair = struct {
    body1: *const Body,
    body2: *const Body,
};

/// The contact constraint offset of contact i is i * contact_stride (Jolt's contacts are constraint offsets in the
/// constraint buffer of the contact manager when the islands are split)
const contact_stride: u32 = 24;

/// Stand in for ContactConstraintManager in SplitIsland: the contacts are body pairs
const ContactPairs = struct {
    pairs: []const ContactPair,

    /// ContactConstraintManager::GetAffectedBodies
    pub fn getAffectedBodies(self: *const ContactPairs, constraint_offset: u32) ContactPair {
        std.debug.assert(constraint_offset % contact_stride == 0);
        return self.pairs[constraint_offset / contact_stride];
    }
};

/// A test constraint that forces its first body into the non parallel split (Jolt's VehicleConstraint does this)
const NonParallelTestConstraint = struct {
    pub const overrides = .{.buildIslandSplits};

    base: TestConstraint,

    pub fn buildIslandSplits(self: *const NonParallelTestConstraint, splitter: *LargeIslandSplitter) u32 {
        return splitter.assignToNonParallelSplit(self.base.base.body1);
    }
};

/// Bodies, constraints and contacts of a test, with the island builder of the physics update (initialized in place)
const Scene = struct {
    allocator: Allocator = undefined,
    bodies: BodyTestContext = .{},
    shape: RefConst(Shape) = .empty,
    body_list: std.ArrayList(*Body) = .empty,
    settings: TestConstraintSettings = undefined,
    constraint_refs: std.ArrayList(Ref(Constraint)) = .empty,
    active_constraints: std.ArrayList(*Constraint) = .empty,
    /// Per constraint: a NonParallelTestConstraint
    non_parallel: std.ArrayList(bool) = .empty,
    contacts: std.ArrayList(ContactPair) = .empty,
    island_builder: IslandBuilder = .empty,
    physics_settings: PhysicsSettings = .{},

    fn init(self: *Scene, allocator: Allocator, max_bodies: u32) !void {
        self.* = .{ .allocator = allocator };
        try self.bodies.init(allocator, max_bodies, 0);
        errdefer self.bodies.deinit();
        try self.island_builder.init(allocator, max_bodies);
        errdefer self.island_builder.deinit();
        const box = try BoxShape.create(allocator, Vec3.replicate(0.5), .{});
        self.shape = .init(box.asShape());
        self.settings = .init(allocator);
    }

    fn deinit(self: *Scene) void {
        for (self.constraint_refs.items) |*r| r.deinit();
        self.constraint_refs.deinit(self.allocator);
        self.active_constraints.deinit(self.allocator);
        self.non_parallel.deinit(self.allocator);
        self.contacts.deinit(self.allocator);
        self.body_list.deinit(self.allocator);
        self.settings.asSettings().deinit();
        self.island_builder.deinit();
        self.bodies.deinit();
        self.shape.deinit();
    }

    fn addBody(self: *Scene, spec: BodySpec) !u32 {
        var settings = BodyCreationSettings.initWithShape(self.shape.get().?, RVec3.zero(), Quat.identity(), spec.motion_type, if (spec.motion_type == .static) Layers.non_moving else Layers.moving);
        defer settings.deinit();
        settings.num_velocity_steps_override = spec.velocity_steps;
        settings.num_position_steps_override = spec.position_steps;
        try self.body_list.ensureUnusedCapacity(self.allocator, 1);
        const body = (try self.bodies.createAndAddBody(&settings, if (spec.activate) .activate else .dont_activate)) orelse return error.OutOfBodies;
        self.body_list.appendAssumeCapacity(body);
        return @intCast(self.body_list.items.len - 1);
    }

    fn addConstraint(self: *Scene, spec: ConstraintSpec) !void {
        const body1 = self.body_list.items[spec.body1];
        const body2 = self.body_list.items[spec.body2];
        try self.constraint_refs.ensureUnusedCapacity(self.allocator, 1);
        try self.active_constraints.ensureUnusedCapacity(self.allocator, 1);
        try self.non_parallel.ensureUnusedCapacity(self.allocator, 1);
        const two_body: *TwoBodyConstraint = if (spec.non_parallel) blk: {
            const c = try self.allocator.create(NonParallelTestConstraint);
            c.* = .{ .base = .{ .base = .init(NonParallelTestConstraint, self.allocator, body1, body2, &self.settings.base) } };
            break :blk &c.base.base;
        } else try self.settings.base.createConstraint(self.allocator, body1, body2);
        const constraint = &two_body.base;
        constraint.setNumVelocityStepsOverride(spec.velocity_steps);
        constraint.setNumPositionStepsOverride(spec.position_steps);
        self.constraint_refs.appendAssumeCapacity(.init(constraint));
        self.active_constraints.appendAssumeCapacity(constraint);
        self.non_parallel.appendAssumeCapacity(spec.non_parallel);
    }

    fn addContact(self: *Scene, body1: u32, body2: u32) !void {
        try self.contacts.append(self.allocator, .{ .body1 = self.body_list.items[body1], .body2 = self.body_list.items[body2] });
    }

    fn contactPairs(self: *const Scene) ContactPairs {
        return .{ .pairs = self.contacts.items };
    }

    fn bodyManager(self: *Scene) *BodyManager {
        return &self.bodies.body_manager;
    }

    /// The island building of one physics update: the constraints link their bodies with BuildIslands (which activates
    /// them), the contacts like ContactConstraintManager::CreateConstraint (activate the dynamic bodies, link two
    /// dynamic bodies, link the contact to the first dynamic body)
    fn buildIslands(self: *Scene, temp_allocator: TempAllocator) TempAllocator.Error!void {
        const builder = &self.island_builder;
        const body_manager = self.bodyManager();
        try builder.prepareContactConstraints(@intCast(self.contacts.items.len), temp_allocator);
        try builder.prepareNonContactConstraints(@intCast(self.active_constraints.items.len), temp_allocator);
        for (self.active_constraints.items, 0..) |c, i|
            c.buildIslands(@intCast(i), builder, body_manager);
        for (self.contacts.items, 0..) |pair, i| {
            const body1_dynamic = pair.body1.isDynamic();
            const body2_dynamic = pair.body2.isDynamic();
            for ([_]*const Body{ pair.body1, pair.body2 }) |b|
                if (b.isDynamic() and !b.isActive())
                    body_manager.activateBodies(&.{b.getID()});
            if (body1_dynamic and body2_dynamic)
                builder.linkBodies(pair.body1.getIndexInActiveBodiesInternal(), pair.body2.getIndexInActiveBodiesInternal());
            builder.linkContact(@intCast(i), if (body1_dynamic) pair.body1.getIndexInActiveBodiesInternal() else pair.body2.getIndexInActiveBodiesInternal());
        }
        const num_active = body_manager.getNumActiveBodies(.rigid_body);
        try builder.finalize(body_manager.getActiveBodiesUnsafe(.rigid_body)[0..num_active], @intCast(self.contacts.items.len), temp_allocator);
    }

    /// ContactConstraintManager::ConstraintIdxToConstraintOffset for the contacts of an island
    fn contactsToOffsets(self: *Scene, island_index: u32) void {
        for (self.island_builder.getContactsInIsland(island_index)) |*c|
            c.* *= contact_stride;
    }

    /// Prepare + SplitIsland for every island in island order (what one SolveVelocityConstraints job does), returns the
    /// number of islands that were split
    fn prepareAndSplit(self: *Scene, splitter: *LargeIslandSplitter, temp_allocator: TempAllocator) TempAllocator.Error!u32 {
        try self.buildIslands(temp_allocator);
        try splitter.prepare(&self.island_builder, self.bodyManager().getNumActiveBodies(.rigid_body), temp_allocator);
        const pairs = self.contactPairs();
        var num_split: u32 = 0;
        var island: u32 = 0;
        while (island < self.island_builder.getNumIslands()) : (island += 1) {
            self.contactsToOffsets(island);
            var steps_calculator = CalculateSolverSteps.init(&self.physics_settings);
            if (splitter.splitIsland(island, &self.island_builder, self.bodyManager(), &pairs, self.active_constraints.items, &steps_calculator))
                num_split += 1;
        }
        return num_split;
    }

    /// Reset + ResetIslands (the end of the update)
    fn finishUpdate(self: *Scene, splitter: *LargeIslandSplitter, temp_allocator: TempAllocator) void {
        splitter.reset(temp_allocator);
        self.island_builder.resetIslands(temp_allocator);
    }
};

fn isActiveDynamic(body: *const Body) bool {
    return body.isDynamic() and body.getIndexInActiveBodiesInternal() != Body.inactive_index;
}

fn constraintBodies(constraint: *const Constraint) ContactPair {
    const two_body: *const TwoBodyConstraint = zolt.virtual.downcast(TwoBodyConstraint, constraint);
    return .{ .body1 = two_body.body1, .body2 = two_body.body2 };
}

/// Independent model of SplitIsland for one island: the greedy assignment over the split masks, the merge of the small
/// splits and the layout of the buffers, plus the iteration counts
const ModelIsland = struct {
    raw_splits: std.ArrayList(u32) = .empty,
    indices: std.ArrayList(u32) = .empty,
    num_parallel_splits: u32 = 0,
    /// Indexed by final split (0 .. num_parallel_splits - 1 and the non parallel split)
    ranges: [num_splits]LargeIslandSplitter.Split = undefined,
    num_velocity_steps: u32 = 0,
    num_position_steps: u32 = 0,

    fn deinit(self: *ModelIsland, allocator: Allocator) void {
        self.raw_splits.deinit(allocator);
        self.indices.deinit(allocator);
    }
};

/// The step overrides of the objects of an island (model of CalculateSolverSteps)
const ModelSteps = struct {
    max_velocity: u32 = 0,
    max_position: u32 = 0,
    default_velocity: bool = false,
    default_position: bool = false,

    fn add(self: *ModelSteps, velocity_steps: u32, position_steps: u32) void {
        if (velocity_steps == 0) self.default_velocity = true else self.max_velocity = @max(self.max_velocity, velocity_steps);
        if (position_steps == 0) self.default_position = true else self.max_position = @max(self.max_position, position_steps);
    }
};

/// The lowest split that none of the active dynamic bodies is in yet, at most the non parallel split
fn modelAssign(masks: []u32, bodies: ContactPair) u32 {
    var used: u32 = 0;
    for ([_]*const Body{ bodies.body1, bodies.body2 }) |b|
        if (isActiveDynamic(b)) {
            used |= masks[b.getIndexInActiveBodiesInternal()];
        };
    var split: u32 = 0;
    while (split < non_parallel_split_idx and used & (@as(u32, 1) << @intCast(split)) != 0)
        split += 1;
    for ([_]*const Body{ bodies.body1, bodies.body2 }) |b|
        if (isActiveDynamic(b)) {
            masks[b.getIndexInActiveBodiesInternal()] |= @as(u32, 1) << @intCast(split);
        };
    return split;
}

fn modelIsland(allocator: Allocator, scene: *Scene, island_index: u32, buffer_offset: u32) !ModelIsland {
    var model: ModelIsland = .{};
    errdefer model.deinit(allocator);

    const contacts = scene.island_builder.getContactsInIsland(island_index);
    const constraints = scene.island_builder.getConstraintsInIsland(island_index);
    const pairs = scene.contactPairs();

    // The split masks per active body, zero for the bodies of the island
    const masks = try allocator.alloc(u32, scene.bodyManager().getNumActiveBodies(.rigid_body));
    defer allocator.free(masks);
    @memset(masks, 0);

    // Assign every item to the lowest split that none of its active dynamic bodies is in yet (at most the non parallel
    // split), contacts first, then constraints
    try model.raw_splits.ensureTotalCapacity(allocator, contacts.len + constraints.len);
    var steps: ModelSteps = .{};
    for (contacts) |c| {
        const bodies = pairs.getAffectedBodies(c);
        model.raw_splits.appendAssumeCapacity(modelAssign(masks, bodies));
        for ([_]*const Body{ bodies.body1, bodies.body2 }) |b|
            if (b.isDynamic()) {
                const mp = b.getMotionPropertiesUnchecked().?;
                steps.add(mp.getNumVelocityStepsOverride(), mp.getNumPositionStepsOverride());
            };
    }
    for (constraints) |c| {
        const constraint = scene.active_constraints.items[c];
        model.raw_splits.appendAssumeCapacity(if (scene.non_parallel.items[c]) non_parallel_split_idx else modelAssign(masks, constraintBodies(constraint)));
        steps.add(constraint.getNumVelocityStepsOverride(), constraint.getNumPositionStepsOverride());
    }
    const max_velocity = steps.max_velocity;
    const max_position = steps.max_position;
    const default_velocity = steps.default_velocity;
    const default_position = steps.default_position;
    model.num_velocity_steps = if (default_velocity) @max(max_velocity, scene.physics_settings.num_velocity_steps) else max_velocity;
    model.num_position_steps = if (default_position) @max(max_position, scene.physics_settings.num_position_steps) else max_position;

    // Small splits go to the non parallel split, the others are numbered in order
    var counts: [num_splits]u32 = @splat(0);
    for (model.raw_splits.items) |s| counts[s] += 1;
    var final_split: [num_splits]u32 = undefined;
    for (0..num_splits) |s| {
        if (s < non_parallel_split_idx and counts[s] < split_combine_treshold) {
            final_split[s] = non_parallel_split_idx;
        } else if (s < non_parallel_split_idx) {
            final_split[s] = model.num_parallel_splits;
            model.num_parallel_splits += 1;
        } else final_split[s] = non_parallel_split_idx;
    }

    // Layout: the parallel splits in order, then the non parallel split, each its contacts then its constraints in
    // island order
    try model.indices.ensureTotalCapacity(allocator, contacts.len + constraints.len);
    var offset = buffer_offset;
    var f: u32 = 0;
    while (f < num_splits) : (f += 1) {
        if (f >= model.num_parallel_splits and f != non_parallel_split_idx)
            continue;
        const range = &model.ranges[f];
        range.contact_buffer_begin = offset;
        for (contacts, model.raw_splits.items[0..contacts.len]) |c, s|
            if (final_split[s] == f)
                model.indices.appendAssumeCapacity(c);
        range.contact_buffer_end = buffer_offset + @as(u32, @intCast(model.indices.items.len));
        range.constraint_buffer_begin = range.contact_buffer_end;
        for (constraints, model.raw_splits.items[contacts.len..]) |c, s|
            if (final_split[s] == f)
                model.indices.appendAssumeCapacity(c);
        range.constraint_buffer_end = buffer_offset + @as(u32, @intCast(model.indices.items.len));
        offset = range.constraint_buffer_end;
    }
    return model;
}

/// Compare the split islands of the splitter with the model, check that the parallel splits don't share a dynamic
/// body, and that the islands below the threshold were not split
fn checkSplits(allocator: Allocator, scene: *Scene, splitter: *const LargeIslandSplitter) !void {
    var num_large: u32 = 0;
    var total_size: u32 = 0;
    var island: u32 = 0;
    while (island < scene.island_builder.getNumIslands()) : (island += 1) {
        const size = scene.island_builder.getContactsInIsland(island).len + scene.island_builder.getConstraintsInIsland(island).len;
        if (size >= large_island_treshold) {
            num_large += 1;
            total_size += @intCast(size);
        }
    }
    try expectEqual(num_large, splitter.num_split_islands);
    try expectEqual(num_large, splitter.next_split_island.raw);
    try expectEqual(total_size, splitter.contact_and_constraints_size);
    try expectEqual(total_size, splitter.contact_and_constraints_next_free.raw);

    // The split islands in the order they were split (island order, single threaded), each island's buffer after the
    // previous one
    var buffer_offset: u32 = 0;
    var split_island_index: u32 = 0;
    for (splitter.split_islands.items[0..splitter.num_split_islands]) |*splits| {
        try expectEqual(splits.island_index, splitter.getIslandIndex(split_island_index));
        var model = try modelIsland(allocator, scene, splits.island_index, buffer_offset);
        defer model.deinit(allocator);
        const island_size: u32 = @intCast(model.indices.items.len);

        // Raw assignment, layout and contents
        try expectEqualSlices(u32, model.raw_splits.items, splitter.contact_and_constraints_split_idx.items[buffer_offset..][0..island_size]);
        try expectEqual(model.num_parallel_splits, splits.getNumSplits());
        try expectEqualSlices(u32, model.indices.items, splitter.contact_and_constraint_indices.items[buffer_offset..][0..island_size]);
        var f: u32 = 0;
        while (f < num_splits) : (f += 1) {
            if (f >= model.num_parallel_splits and f != non_parallel_split_idx)
                continue;
            try expectEqual(model.ranges[f], splits.splits[f]);
            const constraints = splits.getConstraintsInSplit(f);
            try expectEqual(model.ranges[f].constraint_buffer_begin, constraints.begin);
            try expectEqual(model.ranges[f].constraint_buffer_end, constraints.end);
            const contacts = splits.getContactsInSplit(f);
            try expectEqual(model.ranges[f].contact_buffer_begin, contacts.begin);
            try expectEqual(model.ranges[f].contact_buffer_end, contacts.end);
            if (f != non_parallel_split_idx) {
                // A parallel split has enough items and no dynamic body twice
                try expect(splits.splits[f].getNumItems() >= split_combine_treshold);
                try checkNoSharedBody(allocator, scene, splitter, &splits.splits[f]);
            }
        }

        // Iterations: 1 for the warm start + the velocity steps (set by SplitIsland)
        try expectEqual(@as(i32, @intCast(model.num_velocity_steps)), splits.num_velocity_steps);
        try expectEqual(@as(i32, @intCast(model.num_position_steps)), splits.num_position_steps);
        try expectEqual(@as(i32, @intCast(model.num_velocity_steps + 1)), splits.num_iterations);

        buffer_offset += island_size;
        split_island_index += 1;
    }
}

/// No active dynamic body is used by two items of a parallel split
fn checkNoSharedBody(allocator: Allocator, scene: *Scene, splitter: *const LargeIslandSplitter, split: *const LargeIslandSplitter.Split) !void {
    const used = try allocator.alloc(bool, scene.bodyManager().getNumActiveBodies(.rigid_body));
    defer allocator.free(used);
    @memset(used, false);
    const pairs = scene.contactPairs();
    const indices = splitter.contact_and_constraint_indices.items;
    for (indices[split.contact_buffer_begin..split.contact_buffer_end]) |c|
        try useBodies(used, pairs.getAffectedBodies(c));
    for (indices[split.constraint_buffer_begin..split.constraint_buffer_end]) |c|
        try useBodies(used, constraintBodies(scene.active_constraints.items[c]));
}

fn useBodies(used: []bool, bodies: ContactPair) !void {
    for ([_]*const Body{ bodies.body1, bodies.body2 }) |b|
        if (isActiveDynamic(b)) {
            const idx = b.getIndexInActiveBodiesInternal();
            try expect(!used[idx]);
            used[idx] = true;
        };
}

/// A batch as offsets in contact_and_constraint_indices, with the flags of FetchNextBatch and MarkBatchProcessed
const BatchRecord = struct {
    split_island_index: u32,
    constraints_begin: u32,
    constraints_end: u32,
    contacts_begin: u32,
    contacts_end: u32,
    first_iteration: bool,
    last_iteration: bool,
    final_batch: bool,
    /// Zolt: the batch comes from a parallel split (Batch.parallel)
    parallel: bool,
};

/// Offset of a slice of contact_and_constraint_indices (Jolt's pointers)
fn offsetIn(indices: []const u32, slice: []const u32) u32 {
    return @intCast((@intFromPtr(slice.ptr) - @intFromPtr(indices.ptr)) / @sizeOf(u32));
}

/// Fetch and mark every batch single threaded (one job that runs alone), recording the sequence
fn runBatches(allocator: Allocator, splitter: *LargeIslandSplitter, out: *std.ArrayList(BatchRecord)) !void {
    while (true) {
        var batch: LargeIslandSplitter.Batch = undefined;
        switch (splitter.fetchNextBatch(&batch)) {
            .batch_retrieved => {
                var last_iteration: bool = undefined;
                var final_batch: bool = undefined;
                splitter.markBatchProcessed(batch.split_island_index, batch.constraints, batch.contacts, &last_iteration, &final_batch);
                const indices = splitter.contact_and_constraint_indices.items;
                try out.append(allocator, .{
                    .split_island_index = batch.split_island_index,
                    .constraints_begin = offsetIn(indices, batch.constraints),
                    .constraints_end = offsetIn(indices, batch.constraints) + @as(u32, @intCast(batch.constraints.len)),
                    .contacts_begin = offsetIn(indices, batch.contacts),
                    .contacts_end = offsetIn(indices, batch.contacts) + @as(u32, @intCast(batch.contacts.len)),
                    .first_iteration = batch.first_iteration,
                    .last_iteration = last_iteration,
                    .final_batch = final_batch,
                    .parallel = batch.parallel,
                });
            },

            // Single threaded nothing is in flight: every split island always has work until it is done
            .waiting_for_batch => return error.TestUnexpectedResult,

            .all_batches_done => return,
        }
    }
}

/// The batch sequence of a single threaded run: the split islands in order, every iteration, the parallel splits in
/// order in batches of 16 items (constraints first, then contacts), then the non parallel split in one batch (empty
/// splits are skipped). `num_iterations` per split island.
fn expectedBatches(allocator: Allocator, splitter: *const LargeIslandSplitter, out: *std.ArrayList(BatchRecord)) !void {
    for (splitter.split_islands.items[0..splitter.num_split_islands], 0..) |*splits, split_island_index| {
        const first_record = out.items.len;
        var iteration: i32 = 0;
        while (iteration < splits.num_iterations) : (iteration += 1) {
            var k: u32 = 0;
            while (k <= splits.num_splits) : (k += 1) {
                const split = &splits.splits[if (k < splits.num_splits) k else non_parallel_split_idx];
                var record: BatchRecord = .{
                    .split_island_index = @intCast(split_island_index),
                    .constraints_begin = 0,
                    .constraints_end = 0,
                    .contacts_begin = 0,
                    .contacts_end = 0,
                    .first_iteration = iteration == 0,
                    .last_iteration = iteration == splits.num_iterations - 1,
                    .final_batch = false,
                    .parallel = k != splits.num_splits,
                };
                if (k == splits.num_splits) {
                    if (split.getNumItems() > 0) {
                        record.constraints_begin = split.constraint_buffer_begin;
                        record.constraints_end = split.constraint_buffer_end;
                        record.contacts_begin = split.contact_buffer_begin;
                        record.contacts_end = split.contact_buffer_end;
                        try out.append(allocator, record);
                    }
                    continue;
                }

                // Items [0, num_constraints) are the constraints, then the contacts. Jolt sets begin = end = 0 for a
                // batch without constraints / contacts, except for the contacts of a batch that ends exactly at the
                // last constraint (an empty range at the start of the contacts).
                const nc = split.getNumConstraints();
                const num_items = split.getNumItems();
                var item: u32 = 0;
                while (item < num_items) : (item += batch_size) {
                    const end = @min(item + batch_size, num_items);
                    if (end >= nc) {
                        if (item < nc) {
                            record.constraints_begin = split.constraint_buffer_begin + item;
                            record.constraints_end = split.constraint_buffer_end;
                        } else {
                            record.constraints_begin = 0;
                            record.constraints_end = 0;
                        }
                        record.contacts_begin = split.contact_buffer_begin + (@max(item, nc) - nc);
                        record.contacts_end = split.contact_buffer_begin + (end - nc);
                    } else {
                        record.constraints_begin = split.constraint_buffer_begin + item;
                        record.constraints_end = split.constraint_buffer_begin + end;
                        record.contacts_begin = 0;
                        record.contacts_end = 0;
                    }
                    try out.append(allocator, record);
                }
            }
        }

        // The batch that finishes the last iteration is the final batch
        if (out.items.len > first_record)
            out.items[out.items.len - 1].final_batch = true;
    }
}

/// Run the velocity steps and the position steps single threaded and compare the batch sequences with the model
fn checkBatches(allocator: Allocator, splitter: *LargeIslandSplitter) !void {
    var actual: std.ArrayList(BatchRecord) = .empty;
    defer actual.deinit(allocator);
    var expected: std.ArrayList(BatchRecord) = .empty;
    defer expected.deinit(allocator);

    // Velocity steps (iteration 0 is the warm start)
    try expectedBatches(allocator, splitter, &expected);
    try runBatches(allocator, splitter, &actual);
    try expectEqualSlices(BatchRecord, expected.items, actual.items);
    try expectEqual(LargeIslandSplitter.Status.all_batches_done, fetchStatus(splitter));

    // Position steps
    splitter.prepareForSolvePositions();
    for (splitter.split_islands.items) |*s|
        try expectEqual(s.num_position_steps, s.num_iterations);
    expected.clearRetainingCapacity();
    actual.clearRetainingCapacity();
    try expectedBatches(allocator, splitter, &expected);
    try runBatches(allocator, splitter, &actual);
    try expectEqualSlices(BatchRecord, expected.items, actual.items);
    try expectEqual(LargeIslandSplitter.Status.all_batches_done, fetchStatus(splitter));
}

fn fetchStatus(splitter: *LargeIslandSplitter) LargeIslandSplitter.Status {
    var batch: LargeIslandSplitter.Batch = undefined;
    return splitter.fetchNextBatch(&batch);
}

/// One update single threaded with all checks: split, check against the model, run and check the batches, reset
fn checkUpdate(allocator: Allocator, scene: *Scene, temp: *TempAllocatorImpl) !void {
    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    _ = try scene.prepareAndSplit(&splitter, temp.tempAllocator());
    try checkSplits(allocator, scene, &splitter);
    try checkBatches(allocator, &splitter);
    scene.finishUpdate(&splitter, temp.tempAllocator());
    try expect(temp.isEmpty());
}

/// A chain of `num_bodies` dynamic bodies linked by contacts between neighbors (starting at body `first`)
fn addContactChain(scene: *Scene, num_bodies: u32, spec: BodySpec) !void {
    var previous = try scene.addBody(spec);
    for (1..num_bodies) |_| {
        const b = try scene.addBody(spec);
        try scene.addContact(previous, b);
        previous = b;
    }
}

test "LargeIslandSplitter islands below the threshold are not split" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();

    // Two islands with 127 and 64 contacts
    try addContactChain(&scene, 128, .{});
    try addContactChain(&scene, 65, .{});

    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    try scene.buildIslands(temp.tempAllocator());
    try expectEqual(@as(u32, 2), scene.island_builder.getNumIslands());
    const usage = temp.getUsage();
    try splitter.prepare(&scene.island_builder, scene.bodyManager().getNumActiveBodies(.rigid_body), temp.tempAllocator());
    try expectEqual(usage, temp.getUsage()); // Nothing to split, nothing allocated
    try expectEqual(@as(u32, 0), splitter.num_split_islands);
    const pairs = scene.contactPairs();
    for (0..2) |island| {
        scene.contactsToOffsets(@intCast(island));
        var steps_calculator = CalculateSolverSteps.init(&scene.physics_settings);
        try expect(!splitter.splitIsland(@intCast(island), &scene.island_builder, scene.bodyManager(), &pairs, scene.active_constraints.items, &steps_calculator));
    }
    try expectEqual(LargeIslandSplitter.Status.all_batches_done, fetchStatus(&splitter));
    splitter.prepareForSolvePositions();
    try expectEqual(LargeIslandSplitter.Status.all_batches_done, fetchStatus(&splitter));
    scene.finishUpdate(&splitter, temp.tempAllocator());
    try expect(temp.isEmpty());

    // One more item (a constraint) brings the first island to the threshold: it is split, the second one is not
    try scene.addConstraint(.{ .body1 = 0, .body2 = 2 });
    var splitter2: LargeIslandSplitter = .empty;
    defer splitter2.deinit();
    try expectEqual(@as(u32, 1), try scene.prepareAndSplit(&splitter2, temp.tempAllocator()));
    try expectEqual(@as(u32, large_island_treshold), splitter2.contact_and_constraints_size);
    try checkSplits(allocator, &scene, &splitter2);
    try checkBatches(allocator, &splitter2);
    scene.finishUpdate(&splitter2, temp.tempAllocator());
    try expect(temp.isEmpty());
}

test "LargeIslandSplitter a chain splits into two parallel splits" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();

    // 128 contacts between neighbors alternate between split 0 and 1, the non parallel split stays empty
    try addContactChain(&scene, 129, .{});
    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    try expectEqual(@as(u32, 1), try scene.prepareAndSplit(&splitter, temp.tempAllocator()));
    try checkSplits(allocator, &scene, &splitter);
    const splits = &splitter.split_islands.items[0];
    try expectEqual(@as(u32, 2), splits.getNumSplits());
    try expectEqual(@as(u32, 64), splits.splits[0].getNumContacts());
    try expectEqual(@as(u32, 64), splits.splits[1].getNumContacts());
    try expectEqual(@as(u32, 0), splits.splits[non_parallel_split_idx].getNumItems());
    for (splitter.contact_and_constraints_split_idx.items, 0..) |s, i|
        try expectEqual(@as(u32, @intCast(i % 2)), s);

    // Default steps: 10 velocity steps + the warm start, 2 position steps, 4 batches per split, the empty non parallel
    // split is skipped
    try expectEqual(@as(i32, 11), splits.num_iterations);
    try expectEqual(@as(i32, 2), splits.num_position_steps);
    var actual: std.ArrayList(BatchRecord) = .empty;
    defer actual.deinit(allocator);
    try runBatches(allocator, &splitter, &actual);
    try expectEqual(@as(usize, 11 * 8), actual.items.len);
    try expectEqual(BatchRecord{ .split_island_index = 0, .constraints_begin = 0, .constraints_end = 0, .contacts_begin = 0, .contacts_end = 16, .first_iteration = true, .last_iteration = false, .final_batch = false, .parallel = true }, actual.items[0]);
    try expectEqual(BatchRecord{ .split_island_index = 0, .constraints_begin = 0, .constraints_end = 0, .contacts_begin = 64, .contacts_end = 80, .first_iteration = true, .last_iteration = false, .final_batch = false, .parallel = true }, actual.items[4]);
    try expectEqual(BatchRecord{ .split_island_index = 0, .constraints_begin = 0, .constraints_end = 0, .contacts_begin = 112, .contacts_end = 128, .first_iteration = false, .last_iteration = true, .final_batch = true, .parallel = true }, actual.items[11 * 8 - 1]);
    splitter.prepareForSolvePositions();
    actual.clearRetainingCapacity();
    try runBatches(allocator, &splitter, &actual);
    try expectEqual(@as(usize, 2 * 8), actual.items.len);

    scene.finishUpdate(&splitter, temp.tempAllocator());
    try expect(temp.isEmpty());

    // And again with the full checks
    try checkUpdate(allocator, &scene, &temp);
}

test "LargeIslandSplitter one dynamic body against static bodies, more contacts than splits" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();

    // All bodies static but one: every contact takes the next split of the dynamic body, the 32nd and later ones go
    // to the non parallel split; every split has fewer than 32 items, so everything ends up in the non parallel split
    const dynamic = try scene.addBody(.{ .velocity_steps = 3, .position_steps = 4 });
    for (0..140) |i| {
        const s = try scene.addBody(.{ .motion_type = .static });
        if (i % 2 == 0) try scene.addContact(dynamic, s) else try scene.addContact(s, dynamic);
    }
    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    try expectEqual(@as(u32, 1), try scene.prepareAndSplit(&splitter, temp.tempAllocator()));
    try checkSplits(allocator, &scene, &splitter);
    const splits = &splitter.split_islands.items[0];
    try expectEqual(@as(u32, 0), splits.getNumSplits());
    try expectEqual(@as(u32, 140), splits.splits[non_parallel_split_idx].getNumContacts());
    for (splitter.contact_and_constraints_split_idx.items, 0..) |s, i|
        try expectEqual(@min(@as(u32, @intCast(i)), non_parallel_split_idx), s);

    // The body's overrides: 3 velocity steps (+ warm start), 4 position steps, one batch per iteration
    try expectEqual(@as(i32, 4), splits.num_iterations);
    try expectEqual(@as(i32, 4), splits.num_position_steps);
    try checkBatches(allocator, &splitter);
    scene.finishUpdate(&splitter, temp.tempAllocator());
    try expect(temp.isEmpty());
}

test "LargeIslandSplitter constraints, kinematic and static bodies, non parallel constraints and step overrides" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();

    // A grid of 10 x 10 dynamic bodies, one of them activated by a constraint, one by a contact
    var grid: [10][10]u32 = undefined;
    for (0..10) |y| for (0..10) |x| {
        grid[y][x] = try scene.addBody(.{ .activate = !(x == 3 and y == 3) and !(x == 7 and y == 8), .velocity_steps = if (x == 5) 4 else 0, .position_steps = if (y == 5) 6 else 0 });
    };
    const ground = try scene.addBody(.{ .motion_type = .static });
    const moving_kinematic = try scene.addBody(.{ .motion_type = .kinematic });
    const sleeping_kinematic = try scene.addBody(.{ .motion_type = .kinematic, .activate = false });

    // Contacts to the right and down neighbors, constraints along the diagonals
    for (0..10) |y| for (0..10) |x| {
        if (x + 1 < 10) try scene.addContact(grid[y][x], grid[y][x + 1]);
        if (y + 1 < 10) try scene.addContact(grid[y + 1][x], grid[y][x]);
        if (x + 1 < 10 and y + 1 < 10) try scene.addConstraint(.{ .body1 = grid[y][x], .body2 = grid[y + 1][x + 1], .velocity_steps = if (x == y) 12 else 0, .non_parallel = x == 2 });
    };

    // Static and kinematic bodies: only the dynamic body gets a split
    for (0..10) |x| {
        try scene.addContact(ground, grid[9][x]);
        try scene.addContact(grid[0][x], if (x % 2 == 0) moving_kinematic else sleeping_kinematic);
        try scene.addConstraint(.{ .body1 = if (x % 2 == 0) moving_kinematic else ground, .body2 = grid[x][0], .position_steps = 3 });
    }

    // A non parallel constraint whose first body is static or kinematic
    try scene.addConstraint(.{ .body1 = ground, .body2 = grid[4][4], .non_parallel = true });
    try scene.addConstraint(.{ .body1 = moving_kinematic, .body2 = grid[4][5], .non_parallel = true });

    // Check twice: the masks of the bodies are reset at the start of each island
    try checkUpdate(allocator, &scene, &temp);
    try checkUpdate(allocator, &scene, &temp);

    // The settings decide the default steps
    scene.physics_settings.num_velocity_steps = 20;
    scene.physics_settings.num_position_steps = 1;
    try checkUpdate(allocator, &scene, &temp);
}

/// A random scene: clusters of dynamic bodies with contacts and constraints between them and to a few shared static and
/// kinematic bodies (some of them islands above and some below the threshold)
fn buildRandomScene(scene: *Scene, random: std.Random) !void {
    var shared: [8]u32 = undefined;
    const num_shared = random.intRangeAtMost(usize, 1, shared.len);
    for (shared[0..num_shared]) |*s|
        s.* = try scene.addBody(if (random.boolean()) .{ .motion_type = .static } else .{ .motion_type = .kinematic, .activate = random.boolean() });

    const num_clusters = random.intRangeAtMost(u32, 1, 5);
    for (0..num_clusters) |_| {
        const num_bodies = random.intRangeAtMost(u32, 1, 70);
        const first = scene.body_list.items.len;
        for (0..num_bodies) |_|
            _ = try scene.addBody(.{
                .activate = random.uintLessThan(u32, 8) != 0,
                .velocity_steps = if (random.uintLessThan(u32, 8) == 0) random.intRangeAtMost(u32, 1, 15) else 0,
                .position_steps = if (random.uintLessThan(u32, 8) == 0) random.intRangeAtMost(u32, 1, 15) else 0,
            });
        const num_items = random.intRangeAtMost(u32, 20, 400);
        for (0..num_items) |_| {
            var b1: u32 = @intCast(first + random.uintLessThan(usize, num_bodies));
            var b2: u32 = if (num_bodies > 1 and random.uintLessThan(u32, 4) != 0)
                @intCast(first + random.uintLessThan(usize, num_bodies))
            else
                shared[random.uintLessThan(usize, num_shared)];
            if (b1 == b2) continue;
            if (random.boolean()) std.mem.swap(u32, &b1, &b2);
            if (random.uintLessThan(u32, 3) != 0) {
                try scene.addContact(b1, b2);
            } else {
                try scene.addConstraint(.{
                    .body1 = b1,
                    .body2 = b2,
                    .velocity_steps = if (random.uintLessThan(u32, 6) == 0) random.intRangeAtMost(u32, 1, 15) else 0,
                    .position_steps = if (random.uintLessThan(u32, 6) == 0) random.intRangeAtMost(u32, 1, 15) else 0,
                    .non_parallel = random.uintLessThan(u32, 10) == 0,
                });
            }
        }
    }
}

test "LargeIslandSplitter random islands" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();
    var num_split_islands: u32 = 0;
    for (0..60) |seed| {
        var prng: std.Random.DefaultPrng = .init(seed);
        var scene: Scene = undefined;
        try scene.init(allocator, 1024);
        defer scene.deinit();
        try buildRandomScene(&scene, prng.random());
        scene.physics_settings.num_velocity_steps = prng.random().intRangeAtMost(u32, 0, 12);
        scene.physics_settings.num_position_steps = prng.random().intRangeAtMost(u32, 0, 4);

        var splitter: LargeIslandSplitter = .empty;
        defer splitter.deinit();
        num_split_islands += try scene.prepareAndSplit(&splitter, temp.tempAllocator());
        try checkSplits(allocator, &scene, &splitter);
        try checkBatches(allocator, &splitter);
        scene.finishUpdate(&splitter, temp.tempAllocator());
        try expect(temp.isEmpty());
    }
    try expect(num_split_islands >= 30); // The scenes reach the splitter
}

/// Processes the batches from several threads like the SolveVelocityConstraints / SolvePositionConstraints jobs: the
/// velocity phase claims the islands and splits them while other threads already process batches
const ThreadContext = struct {
    const Phase = enum { velocity, position };

    scene: *Scene,
    splitter: *LargeIslandSplitter,
    pairs: ContactPairs,
    phase: Phase,
    next_island: std.atomic.Value(u32) = .init(0),
    num_islands_not_split: std.atomic.Value(u32) = .init(0),
    /// Per active body: 1 + the thread that processes a batch with this body, 0 when no thread does
    body_owner: []std.atomic.Value(u32),
    /// Per item of contact_and_constraint_indices: how often it was processed, in its first and in its last iteration
    item_count: []std.atomic.Value(u32),
    first_iteration_count: []std.atomic.Value(u32),
    last_iteration_count: []std.atomic.Value(u32),
    /// Per split island: the number of final batches
    final_batches: []std.atomic.Value(u32),
    /// Set when two threads processed a body at the same time
    conflict: std.atomic.Value(bool) = .init(false),
    /// Incremented for every processed batch and claimed island
    progress: std.atomic.Value(u64) = .init(0),
    /// Set when no thread made progress for a long time: a broken protocol fails the test instead of hanging it
    stalled: std.atomic.Value(bool) = .init(false),

    fn worker(self: *ThreadContext, thread_index: u32) void {
        var check_islands = self.phase == .velocity;
        var check_split_islands = true;
        var last_progress: u64 = 0;
        var num_stalls: u32 = 0;
        while (true) {
            // First try to get work from large islands
            if (check_split_islands) {
                var batch: LargeIslandSplitter.Batch = undefined;
                switch (self.splitter.fetchNextBatch(&batch)) {
                    .batch_retrieved => {
                        self.processBatch(thread_index, &batch);
                        continue;
                    },
                    .waiting_for_batch => {},
                    .all_batches_done => check_split_islands = false,
                }
            }

            // If that didn't succeed try to process an island
            if (check_islands) {
                const island = self.next_island.fetchAdd(1, .seq_cst);
                if (island >= self.scene.island_builder.getNumIslands()) {
                    check_islands = false;
                    continue;
                }
                _ = self.progress.fetchAdd(1, .monotonic);
                self.scene.contactsToOffsets(island);
                var steps_calculator = CalculateSolverSteps.init(&self.scene.physics_settings);
                if (!self.splitter.splitIsland(island, &self.scene.island_builder, self.scene.bodyManager(), &self.pairs, self.scene.active_constraints.items, &steps_calculator))
                    _ = self.num_islands_not_split.fetchAdd(1, .monotonic);
            } else if (check_split_islands) {
                // Waiting for other threads: give up when nobody makes progress anymore
                const progress = self.progress.load(.monotonic);
                if (progress != last_progress) {
                    last_progress = progress;
                    num_stalls = 0;
                } else {
                    num_stalls += 1;
                    if (num_stalls > 20_000_000 or self.stalled.load(.monotonic)) {
                        self.stalled.store(true, .monotonic);
                        break;
                    }
                }
                std.Thread.yield() catch {};
            } else break;
        }
    }

    fn processBatch(self: *ThreadContext, thread_index: u32, batch: *const LargeIslandSplitter.Batch) void {
        // Take the dynamic bodies of the batch (the non parallel split may use a body several times)
        const owner = thread_index + 1;
        for (batch.constraints) |c| self.takeBodies(owner, constraintBodies(self.scene.active_constraints.items[c]));
        for (batch.contacts) |c| self.takeBodies(owner, self.pairs.getAffectedBodies(c));

        // "Solve": count every item
        const indices = self.splitter.contact_and_constraint_indices.items;
        for ([_][]const u32{ batch.constraints, batch.contacts }) |items| {
            const begin = offsetIn(indices, items);
            for (begin..begin + items.len) |i| {
                _ = self.item_count[i].fetchAdd(1, .monotonic);
                if (batch.first_iteration) _ = self.first_iteration_count[i].fetchAdd(1, .monotonic);
            }
        }
        std.Thread.yield() catch {}; // Hold the bodies a little longer, so that an overlapping batch would show

        // Release the bodies before marking the batch: the next split can start right after
        for (batch.constraints) |c| self.releaseBodies(owner, constraintBodies(self.scene.active_constraints.items[c]));
        for (batch.contacts) |c| self.releaseBodies(owner, self.pairs.getAffectedBodies(c));

        var last_iteration: bool = undefined;
        var final_batch: bool = undefined;
        self.splitter.markBatchProcessed(batch.split_island_index, batch.constraints, batch.contacts, &last_iteration, &final_batch);
        if (last_iteration) {
            for ([_][]const u32{ batch.constraints, batch.contacts }) |items| {
                const begin = offsetIn(indices, items);
                for (begin..begin + items.len) |i| _ = self.last_iteration_count[i].fetchAdd(1, .monotonic);
            }
        }
        if (final_batch)
            _ = self.final_batches[batch.split_island_index].fetchAdd(1, .monotonic);
        _ = self.progress.fetchAdd(1, .monotonic);
    }

    fn takeBodies(self: *ThreadContext, owner: u32, bodies: ContactPair) void {
        for ([_]*const Body{ bodies.body1, bodies.body2 }) |b| {
            if (!isActiveDynamic(b)) continue;
            if (self.body_owner[b.getIndexInActiveBodiesInternal()].cmpxchgStrong(0, owner, .acq_rel, .acquire)) |current| {
                if (current != owner) self.conflict.store(true, .monotonic);
            }
        }
    }

    fn releaseBodies(self: *ThreadContext, owner: u32, bodies: ContactPair) void {
        for ([_]*const Body{ bodies.body1, bodies.body2 }) |b| {
            if (!isActiveDynamic(b)) continue;
            _ = self.body_owner[b.getIndexInActiveBodiesInternal()].cmpxchgStrong(owner, 0, .acq_rel, .monotonic);
        }
    }

    fn resetCounters(self: *ThreadContext) void {
        for ([_][]std.atomic.Value(u32){ self.body_owner, self.item_count, self.first_iteration_count, self.last_iteration_count, self.final_batches }) |counters|
            for (counters) |*c| c.store(0, .monotonic);
    }

    fn run(self: *ThreadContext, num_threads: u32) !void {
        var threads: [8]std.Thread = undefined;
        for (threads[0..num_threads], 0..) |*t, i|
            t.* = try std.Thread.spawn(.{}, worker, .{ self, @as(u32, @intCast(i)) });
        for (threads[0..num_threads]) |t| t.join();
    }

    /// Every item of every split island processed num_iterations times, once in its first and once in its last
    /// iteration, one final batch per split island
    fn check(self: *ThreadContext) !void {
        try expect(!self.stalled.load(.monotonic));
        try expect(!self.conflict.load(.monotonic));
        for (self.splitter.split_islands.items[0..self.splitter.num_split_islands], 0..) |*splits, s| {
            const num_iterations: u32 = @intCast(splits.num_iterations);
            var k: u32 = 0;
            while (k <= splits.num_splits) : (k += 1) {
                const split = &splits.splits[if (k < splits.num_splits) k else non_parallel_split_idx];
                for ([_][2]u32{ .{ split.contact_buffer_begin, split.contact_buffer_end }, .{ split.constraint_buffer_begin, split.constraint_buffer_end } }) |range| {
                    for (range[0]..range[1]) |i| {
                        try expectEqual(num_iterations, self.item_count[i].load(.monotonic));
                        if (self.phase == .velocity)
                            try expectEqual(@as(u32, 1), self.first_iteration_count[i].load(.monotonic));
                        try expectEqual(@as(u32, @intFromBool(num_iterations > 0)), self.last_iteration_count[i].load(.monotonic));
                    }
                }
            }
            try expectEqual(@as(u32, @intFromBool(num_iterations > 0)), self.final_batches[s].load(.monotonic));
        }
    }
};

test "LargeIslandSplitter batches from several threads" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    for (0..12) |round| {
        var prng: std.Random.DefaultPrng = .init(1000 + round);
        const random = prng.random();
        var scene: Scene = undefined;
        try scene.init(allocator, 1024);
        defer scene.deinit();
        try buildRandomScene(&scene, random);
        // Some large islands for sure: chains with crossing constraints
        for (0..3) |_| {
            const first: u32 = @intCast(scene.body_list.items.len);
            try addContactChain(&scene, 60, .{});
            for (0..90) |_| try scene.addConstraint(.{ .body1 = first + random.uintLessThan(u32, 60), .body2 = first + random.uintLessThan(u32, 60), .non_parallel = random.uintLessThan(u32, 20) == 0 });
        }
        scene.physics_settings.num_position_steps = random.intRangeAtMost(u32, 0, 3);

        var splitter: LargeIslandSplitter = .empty;
        defer splitter.deinit();
        try scene.buildIslands(temp.tempAllocator());
        try splitter.prepare(&scene.island_builder, scene.bodyManager().getNumActiveBodies(.rigid_body), temp.tempAllocator());
        try expect(splitter.num_split_islands >= 3);

        const num_active = scene.bodyManager().getNumActiveBodies(.rigid_body);
        const size = splitter.contact_and_constraints_size;
        const counters = try allocator.alloc(std.atomic.Value(u32), num_active + 3 * size + splitter.num_split_islands);
        defer allocator.free(counters);
        var context: ThreadContext = .{
            .scene = &scene,
            .splitter = &splitter,
            .pairs = scene.contactPairs(),
            .phase = .velocity,
            .body_owner = counters[0..num_active],
            .item_count = counters[num_active..][0..size],
            .first_iteration_count = counters[num_active + size ..][0..size],
            .last_iteration_count = counters[num_active + 2 * size ..][0..size],
            .final_batches = counters[num_active + 3 * size ..],
        };
        context.resetCounters();

        // Velocity steps: the threads split the islands and process the batches
        const num_threads: u32 = @intCast(2 + round % 4);
        try context.run(num_threads);
        try expectEqual(scene.island_builder.getNumIslands(), splitter.num_split_islands + context.num_islands_not_split.load(.monotonic));
        try context.check();

        // The splits do not depend on the order in which the islands were split
        for (splitter.split_islands.items) |*splits| {
            var model = try modelIsland(allocator, &scene, splits.island_index, splits.splits[if (splits.num_splits > 0) 0 else non_parallel_split_idx].contact_buffer_begin);
            defer model.deinit(allocator);
            try expectEqual(model.num_parallel_splits, splits.num_splits);
            try expectEqualSlices(u32, model.indices.items, splitter.contact_and_constraint_indices.items[model.ranges[if (splits.num_splits > 0) 0 else non_parallel_split_idx].contact_buffer_begin..][0..model.indices.items.len]);
        }

        // Position steps
        splitter.prepareForSolvePositions();
        context.phase = .position;
        context.resetCounters();
        try context.run(num_threads);
        try context.check();

        scene.finishUpdate(&splitter, temp.tempAllocator());
        try expect(temp.isEmpty());
    }
}

/// A complete update (islands, split, velocity and position batches, reset), the batch sequence goes to `out`
fn fullUpdate(allocator: Allocator, scene: *Scene, splitter: *LargeIslandSplitter, temp_allocator: TempAllocator, out: *std.ArrayList(BatchRecord)) !void {
    _ = try scene.prepareAndSplit(splitter, temp_allocator);
    try runBatches(allocator, splitter, out);
    splitter.prepareForSolvePositions();
    try runBatches(allocator, splitter, out);
    scene.finishUpdate(splitter, temp_allocator);
}

test "LargeIslandSplitter out of temp memory at every allocation" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();
    var prng: std.Random.DefaultPrng = .init(77);
    try buildRandomScene(&scene, prng.random());
    try addContactChain(&scene, 200, .{});
    try addContactChain(&scene, 150, .{});

    // Reference run, counting the allocations
    var reference: std.ArrayList(BatchRecord) = .empty;
    defer reference.deinit(allocator);
    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    var counting: FailingTempAllocator = .init(temp.tempAllocator(), .{});
    try fullUpdate(allocator, &scene, &splitter, counting.tempAllocator(), &reference);
    try expect(temp.isEmpty());
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= 14); // The island builder's and the 4 of the splitter

    var records: std.ArrayList(BatchRecord) = .empty;
    defer records.deinit(allocator);
    for (0..num_allocations) |fail_index| {
        // The error arrives, the cleanup of the update (the poisoned path) frees everything in the right order
        var failing: FailingTempAllocator = .init(temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        records.clearRetainingCapacity();
        try std.testing.expectError(error.OutOfMemory, fullUpdate(allocator, &scene, &splitter, failing.tempAllocator(), &records));
        try expect(failing.has_induced_failure);
        splitter.resetAfterOutOfMemory(failing.tempAllocator());
        scene.island_builder.resetIslands(failing.tempAllocator());
        try expect(temp.isEmpty());

        // The splitter is in its constructed state again (also when the allocation of the split masks failed after
        // Prepare set the number of active bodies)
        try expectEqual(@as(u32, 0), splitter.num_active_bodies);
        try expectEqual(@as(u32, 0), splitter.num_split_islands);
        try expectEqual(@as(u32, 0), splitter.contact_and_constraints_size);
        try expectEqual(@as(u32, 0), splitter.contact_and_constraints_next_free.raw);
        try expectEqual(@as(u32, 0), splitter.next_split_island.raw);

        // The next update gives the reference results
        records.clearRetainingCapacity();
        try fullUpdate(allocator, &scene, &splitter, temp.tempAllocator(), &records);
        try expectEqualSlices(BatchRecord, reference.items, records.items);
        try expect(temp.isEmpty());
    }
}

test "LargeIslandSplitter resetAfterOutOfMemory when no island was split" {
    const allocator = std.testing.allocator;
    var temp: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp.deinit();
    var scene: Scene = undefined;
    try scene.init(allocator, 1024);
    defer scene.deinit();
    try addContactChain(&scene, 200, .{});
    try addContactChain(&scene, 140, .{});

    // Prepare allocated the buffers for two islands, then an out of memory error elsewhere poisoned the update before a
    // SolveVelocityConstraints job split them
    var splitter: LargeIslandSplitter = .empty;
    defer splitter.deinit();
    try scene.buildIslands(temp.tempAllocator());
    try splitter.prepare(&scene.island_builder, scene.bodyManager().getNumActiveBodies(.rigid_body), temp.tempAllocator());
    try expectEqual(@as(u32, 2), splitter.num_split_islands);
    try expectEqual(@as(u32, 199 + 139), splitter.contact_and_constraints_size);
    try expectEqual(scene.bodyManager().getNumActiveBodies(.rigid_body), splitter.num_active_bodies);
    try expectEqual(LargeIslandSplitter.Status.waiting_for_batch, fetchStatus(&splitter)); // Not every island was submitted

    // Prepare prevents any of the splits from being picked up as work (ResetStatus): a split island that a job claimed
    // in SplitIsland (mNextSplitIsland) but did not start yet hands out no batch
    for (splitter.split_islands.items) |*s| {
        try expectEqual(Splits.status_item_mask, s.status.load(.monotonic));
        var split_batch: Splits.SplitBatch = undefined;
        try expectEqual(LargeIslandSplitter.Status.waiting_for_batch, s.fetchNextBatch(&split_batch));
        try expectEqual(Splits.status_item_mask, s.status.load(.monotonic)); // Only read
    }
    for (0..2) |_| {
        _ = splitter.next_split_island.fetchAdd(1, .monotonic); // Claimed like SplitIsland does, not started
        try expectEqual(LargeIslandSplitter.Status.waiting_for_batch, fetchStatus(&splitter));
    }
    for (splitter.split_islands.items) |*s|
        try expectEqual(Splits.status_item_mask, s.status.load(.monotonic));

    splitter.resetAfterOutOfMemory(temp.tempAllocator());
    scene.island_builder.resetIslands(temp.tempAllocator());
    try expect(temp.isEmpty());
    try expectEqual(@as(u32, 0), splitter.num_active_bodies);
    try expectEqual(@as(u32, 0), splitter.num_split_islands);
    try expectEqual(@as(u32, 0), splitter.contact_and_constraints_size);
    try expectEqual(@as(u32, 0), splitter.next_split_island.raw);

    // The splitter works again
    try checkUpdate(allocator, &scene, &temp);
}
