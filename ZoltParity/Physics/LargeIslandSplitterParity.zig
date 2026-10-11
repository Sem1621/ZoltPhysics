//! Parity tests for LargeIslandSplitter (Phase 5 A5). The same scene is built on both sides (`LISSceneDesc`): box
//! bodies in a BodyManager (static, kinematic and dynamic, active or not, with step overrides), two body constraints
//! (zolt.test_constraints.TestConstraint / Jolt's TwoBodyConstraint::BuildIslandSplits, or a constraint that forces its
//! first body into the non parallel split like VehicleConstraint) and contacts as body pairs (resolved by
//! GetAffectedBodies from the contact constraint offsets). Each update links the islands like Jolt, prepares the
//! splitter, splits every island in island order and then fetches and marks the batches of the velocity and the
//! position steps in an order given by a decision list (up to `max_outstanding` batches out at the same time, marked
//! in any order, like several solver jobs). Both sides log as uint32 words and the logs must be equal:
//! - the temp memory used after Prepare (the sizes of Jolt's buffers, sizeof(Splits)) and after Reset;
//! - the islands (bodies, contact offsets, constraints) and SplitIsland's result per island;
//! - the splitter after Prepare, after splitting, after each phase and after Reset: the counters, which split each
//!   contact and constraint was assigned to (mContactAndConstraintsSplitIdx), the sorted indices per split
//!   (mContactAndConstraintIndices), every split island (island index, number of splits, iterations, velocity and
//!   position steps, the status word, the processed item counter, every split range and size) and GetIslandIndex;
//! - every FetchNextBatch (status, split island, the batch as offsets, first iteration, GetIslandIndex) and every
//!   MarkBatchProcessed (last iteration, final batch).
//! Scenes: hand-picked edge cases (islands just below / at the threshold, a chain with an empty non parallel split, all
//! bodies static but one, more contacts on one body than there are splits, kinematic and static bodies, non parallel
//! constraints, 0 velocity / position steps, one large scene) and random scenes with random interleavings, two updates
//! each. AssignSplit / AssignToNonParallelSplit are also compared directly on random split masks (the masks of all active
//! bodies after every call). Plus the layout of Splits.
//! C ABI wrappers: ZoltParity/Physics/LargeIslandSplitterReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const CalculateSolverSteps = zolt.CalculateSolverSteps;
const Constraint = zolt.Constraint;
const IslandBuilder = zolt.IslandBuilder;
const LargeIslandSplitter = zolt.LargeIslandSplitter;
const MotionType = zolt.MotionType;
const ObjectLayer = zolt.ObjectLayer;
const PhysicsSettings = zolt.PhysicsSettings;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const TwoBodyConstraint = zolt.TwoBodyConstraint;
const Vec3 = zolt.Vec3;
const TestConstraint = zolt.test_constraints.TestConstraint;
const TestConstraintSettings = zolt.test_constraints.TestConstraintSettings;

const io = std.testing.io;

/// The C++ reference functions, see LargeIslandSplitterReference.cpp
const jolt = struct {
    extern fn jolt_lis_run(desc: *const LISSceneDesc, out_log: [*]u32, capacity: u32) u32;
    extern fn jolt_lis_assign(bodies: [*]const LISBodyDesc, num_bodies: u32, masks: [*]const u32, ops: [*]const u32, num_ops: u32, out_log: [*]u32, capacity: u32) u32;
    extern fn jolt_lis_layout(out_values: *[8]u32) void;
};

/// Must match LISBodyDesc in LargeIslandSplitterReference.cpp
const LISBodyDesc = extern struct {
    /// EMotionType
    motion_type: u32 = @intFromEnum(MotionType.dynamic),
    activate: u32 = 1,
    velocity_steps: u32 = 0,
    position_steps: u32 = 0,
};

/// Must match LISConstraintDesc in LargeIslandSplitterReference.cpp
const LISConstraintDesc = extern struct {
    body1: u32,
    body2: u32,
    velocity_steps: u32 = 0,
    position_steps: u32 = 0,
    non_parallel: u32 = 0,
};

/// Must match LISContactDesc in LargeIslandSplitterReference.cpp
const LISContactDesc = extern struct {
    body1: u32,
    body2: u32,
};

/// Must match LISSceneDesc in LargeIslandSplitterReference.cpp
const LISSceneDesc = extern struct {
    bodies: [*]const LISBodyDesc,
    constraints: [*]const LISConstraintDesc,
    contacts: [*]const LISContactDesc,
    decisions: [*]const u32,
    num_bodies: u32,
    num_constraints: u32,
    num_contacts: u32,
    num_decisions: u32,
    num_velocity_steps: u32,
    num_position_steps: u32,
    max_outstanding: u32,
    num_updates: u32,
};

/// Distance between the contact records in the constraint buffer of the contact manager (cLISContactStride)
const contact_stride: u32 = 24;

/// Broad phase layer = object layer % 2 (LISBPLayerInterface)
const ParityLayers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(ParityLayers),

    pub fn getNumBroadPhaseLayers(self: *const ParityLayers) u32 {
        _ = self;
        return 2;
    }

    pub fn getBroadPhaseLayer(self: *const ParityLayers, layer: ObjectLayer) BroadPhaseLayer {
        _ = self;
        return .init(@intCast(layer % 2));
    }
};

/// A test constraint that forces its first body into the non parallel split (LISTestConstraint with mNonParallel)
const NonParallelTestConstraint = struct {
    pub const overrides = .{.buildIslandSplits};

    base: TestConstraint,

    pub fn buildIslandSplits(self: *const NonParallelTestConstraint, splitter: *LargeIslandSplitter) u32 {
        return splitter.assignToNonParallelSplit(self.base.base.body1);
    }
};

/// The bodies of a contact constraint (ContactConstraintManager.AffectedBodies)
const ContactPair = struct {
    body1: *const Body,
    body2: *const Body,
};

/// Stand in for ContactConstraintManager in SplitIsland: contact i is at offset i * contact_stride
const ContactPairs = struct {
    pairs: []const ContactPair,

    pub fn getAffectedBodies(self: *const ContactPairs, constraint_offset: u32) ContactPair {
        return self.pairs[constraint_offset / contact_stride];
    }
};

/// The log of LISLog
const Log = struct {
    allocator: Allocator,
    words: std.ArrayList(u32) = .empty,

    fn deinit(self: *Log) void {
        self.words.deinit(self.allocator);
    }

    fn w(self: *Log, value: u32) void {
        self.words.append(self.allocator, value) catch @panic("OOM");
    }

    fn w64(self: *Log, value: u64) void {
        self.w(@truncate(value));
        self.w(@truncate(value >> 32));
    }

    fn i(self: *Log, value: i32) void {
        self.w(@bitCast(value));
    }

    fn b(self: *Log, value: bool) void {
        self.w(@intFromBool(value));
    }
};

/// LISLogSplits
fn logSplits(log: *Log, splits: *const LargeIslandSplitter.Splits) void {
    log.w(splits.island_index);
    log.w(splits.num_splits);
    log.i(splits.num_iterations);
    log.i(splits.num_velocity_steps);
    log.i(splits.num_position_steps);
    log.w64(splits.status.load(.monotonic));
    log.w(splits.items_processed.load(.monotonic));
    var s: u32 = 0;
    while (s < LargeIslandSplitter.num_splits) : (s += 1) {
        // The parallel splits, then the non parallel split
        if (s >= splits.num_splits)
            s = LargeIslandSplitter.non_parallel_split_idx;

        const split = &splits.splits[s];
        log.w(split.contact_buffer_begin);
        log.w(split.contact_buffer_end);
        log.w(split.constraint_buffer_begin);
        log.w(split.constraint_buffer_end);
        log.w(split.getNumContacts());
        log.w(split.getNumConstraints());
        log.w(split.getNumItems());

        const constraints = splits.getConstraintsInSplit(s);
        log.w(constraints.begin);
        log.w(constraints.end);
        const contacts = splits.getContactsInSplit(s);
        log.w(contacts.begin);
        log.w(contacts.end);
    }
}

/// LISLogSplitter
fn logSplitter(log: *Log, splitter: *const LargeIslandSplitter, with_buffers: bool) void {
    const size = splitter.contact_and_constraints_size;
    log.w(0x5717E000);
    log.w(splitter.num_active_bodies);
    log.w(size);
    log.w(splitter.contact_and_constraints_next_free.load(.monotonic));
    log.w(splitter.num_split_islands);
    log.w(splitter.next_split_island.load(.monotonic));
    if (with_buffers and size > 0) {
        for (splitter.contact_and_constraints_split_idx.items[0..size]) |v| log.w(v);
        for (splitter.contact_and_constraint_indices.items[0..size]) |v| log.w(v);
    }
    const num_created = splitter.next_split_island.load(.monotonic);
    for (splitter.split_islands.items[0..num_created], 0..) |*s, i| {
        logSplits(log, s);
        log.w(splitter.getIslandIndex(@intCast(i)));
    }
}

/// LISWorld on the Zolt side (initialized in place)
const World = struct {
    allocator: Allocator = undefined,
    desc: *const LISSceneDesc = undefined,
    settings: PhysicsSettings = .{},
    layers: ParityLayers = .{},
    body_manager: BodyManager = .empty,
    island_builder: IslandBuilder = .empty,
    splitter: LargeIslandSplitter = .empty,
    temp: TempAllocatorImpl = undefined,
    constraint_settings: TestConstraintSettings = undefined,
    shape: RefConst(Shape) = .empty,
    bodies: std.ArrayList(*Body) = .empty,
    constraint_refs: std.ArrayList(Ref(Constraint)) = .empty,
    active_constraints: std.ArrayList(*Constraint) = .empty,
    pairs: std.ArrayList(ContactPair) = .empty,
    coverage: *Coverage = undefined,

    fn init(self: *World, allocator: Allocator, desc: *const LISSceneDesc, coverage: *Coverage) !void {
        self.* = .{ .allocator = allocator, .desc = desc, .coverage = coverage };
        self.settings.num_velocity_steps = desc.num_velocity_steps;
        self.settings.num_position_steps = desc.num_position_steps;
        try self.body_manager.init(allocator, io, desc.num_bodies + 1, 0, &self.layers.base);
        try self.island_builder.init(allocator, desc.num_bodies + 1);
        self.temp = try .init(allocator, 32 * 1024 * 1024);
        self.constraint_settings = .init(allocator);
        self.shape = .init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());

        // Bodies
        for (desc.bodies[0..desc.num_bodies]) |b| {
            const motion_type: MotionType = @enumFromInt(b.motion_type);
            var settings = BodyCreationSettings.initWithShape(self.shape.get().?, RVec3.zero(), Quat.identity(), motion_type, if (motion_type == .static) 0 else 1);
            defer settings.deinit();
            settings.num_velocity_steps_override = b.velocity_steps;
            settings.num_position_steps_override = b.position_steps;
            const body = try self.body_manager.allocateBody(&settings);
            _ = self.body_manager.addBody(body);
            body.setInBroadPhaseInternal(true);
            if (b.activate != 0 and !body.isStatic())
                self.body_manager.activateBodies(&.{body.getID()});
            try self.bodies.append(allocator, body);
        }

        // Constraints
        for (desc.constraints[0..desc.num_constraints]) |c| {
            const body1 = self.bodies.items[c.body1];
            const body2 = self.bodies.items[c.body2];
            const two_body: *TwoBodyConstraint = if (c.non_parallel != 0) blk: {
                const constraint = try allocator.create(NonParallelTestConstraint);
                constraint.* = .{ .base = .{ .base = .init(NonParallelTestConstraint, allocator, body1, body2, &self.constraint_settings.base) } };
                break :blk &constraint.base.base;
            } else try self.constraint_settings.base.createConstraint(allocator, body1, body2);
            const constraint = &two_body.base;
            constraint.setNumVelocityStepsOverride(c.velocity_steps);
            constraint.setNumPositionStepsOverride(c.position_steps);
            try self.constraint_refs.append(allocator, .init(constraint));
            try self.active_constraints.append(allocator, constraint);
        }

        // Contacts
        for (desc.contacts[0..desc.num_contacts]) |c|
            try self.pairs.append(allocator, .{ .body1 = self.bodies.items[c.body1], .body2 = self.bodies.items[c.body2] });
    }

    fn deinit(self: *World) void {
        for (self.constraint_refs.items) |*r| r.deinit();
        self.constraint_refs.deinit(self.allocator);
        self.active_constraints.deinit(self.allocator);
        self.pairs.deinit(self.allocator);
        self.bodies.deinit(self.allocator);
        self.constraint_settings.asSettings().deinit();
        self.splitter.deinit();
        self.island_builder.deinit();
        self.body_manager.deinit();
        self.shape.deinit();
        self.temp.deinit();
    }

    /// LISWorld::Update
    fn update(self: *World, log: *Log) !void {
        const temp = self.temp.tempAllocator();
        const num_contacts = self.desc.num_contacts;
        const bm = &self.body_manager;

        // Build the islands
        try self.island_builder.prepareContactConstraints(num_contacts, temp);
        try self.island_builder.prepareNonContactConstraints(@intCast(self.active_constraints.items.len), temp);
        for (self.active_constraints.items, 0..) |c, i|
            c.buildIslands(@intCast(i), &self.island_builder, bm);
        for (self.pairs.items, 0..) |pair, i| {
            // Like ContactConstraintManager::CreateConstraint
            const body1_dynamic = pair.body1.isDynamic();
            const body2_dynamic = pair.body2.isDynamic();
            var body_ids: [2]zolt.BodyID = undefined;
            var num_bodies: usize = 0;
            if (body1_dynamic and !pair.body1.isActive()) {
                body_ids[num_bodies] = pair.body1.getID();
                num_bodies += 1;
            }
            if (body2_dynamic and !pair.body2.isActive()) {
                body_ids[num_bodies] = pair.body2.getID();
                num_bodies += 1;
            }
            if (num_bodies > 0)
                bm.activateBodies(body_ids[0..num_bodies]);
            if (body1_dynamic and body2_dynamic)
                self.island_builder.linkBodies(pair.body1.getIndexInActiveBodiesInternal(), pair.body2.getIndexInActiveBodiesInternal());
            if (body1_dynamic)
                self.island_builder.linkContact(@intCast(i), pair.body1.getIndexInActiveBodiesInternal())
            else
                self.island_builder.linkContact(@intCast(i), pair.body2.getIndexInActiveBodiesInternal());
        }
        const num_active = bm.getNumActiveBodies(.rigid_body);
        try self.island_builder.finalize(bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active], num_contacts, temp);

        // Prepare the splitter
        try self.splitter.prepare(&self.island_builder, bm.getNumActiveBodies(.rigid_body), temp);
        log.w(0x9E9A0000);
        log.w(@intCast(self.temp.getUsage()));
        logSplitter(log, &self.splitter, false); // The buffers are not initialized yet

        // The islands, with the contacts converted to offsets, then split them in order
        const contact_pairs: ContactPairs = .{ .pairs = self.pairs.items };
        const num_islands = self.island_builder.getNumIslands();
        log.w(0x15A4D000);
        log.w(num_islands);
        var island: u32 = 0;
        while (island < num_islands) : (island += 1) {
            const island_bodies = self.island_builder.getBodiesInIsland(island);
            log.w(@intCast(island_bodies.len));
            for (island_bodies) |id| log.w(id.getIndexAndSequenceNumber());

            const contacts = self.island_builder.getContactsInIsland(island);
            log.w(@intCast(contacts.len));
            for (contacts) |*c| {
                c.* *= contact_stride;
                log.w(c.*);
            }
            const constraints = self.island_builder.getConstraintsInIsland(island);
            log.w(@intCast(constraints.len));
            for (constraints) |c| log.w(c);

            var steps_calculator = CalculateSolverSteps.init(&self.settings);
            const split = self.splitter.splitIsland(island, &self.island_builder, bm, &contact_pairs, self.active_constraints.items, &steps_calculator);
            log.b(split);
            if (!split and contacts.len + constraints.len > 0)
                self.coverage.not_split += 1;
        }
        logSplitter(log, &self.splitter, true);
        self.countSplits();

        // Velocity steps
        log.w(0x7E10C000);
        try self.runBatches(log);
        logSplitter(log, &self.splitter, true);

        // Position steps
        self.splitter.prepareForSolvePositions();
        log.w(0x9051710A);
        logSplitter(log, &self.splitter, true);
        try self.runBatches(log);
        logSplitter(log, &self.splitter, true);

        // The end of the update
        self.splitter.reset(temp);
        self.island_builder.resetIslands(temp);
        log.w(0xE4D00000);
        log.w(@intCast(self.temp.getUsage()));
        logSplitter(log, &self.splitter, true);
    }

    /// LISWorld::RunBatches
    fn runBatches(self: *World, log: *Log) !void {
        const indices = self.splitter.contact_and_constraint_indices.items;
        var outstanding: std.ArrayList(LargeIslandSplitter.Batch) = .empty;
        defer outstanding.deinit(self.allocator);
        var force_mark = false;
        var decision: u32 = 0;
        var step: u32 = 0;
        while (step < 10000000) : (step += 1) {
            const d = self.desc.decisions[decision % self.desc.num_decisions];
            decision +%= 1;
            const fetch = outstanding.items.len == 0 or (!force_mark and outstanding.items.len < self.desc.max_outstanding and (d & 1) != 0);
            if (fetch) {
                var batch: LargeIslandSplitter.Batch = undefined;
                const status = self.splitter.fetchNextBatch(&batch);
                log.w(0xF0000000 | @as(u32, @intFromEnum(status)));
                if (status == .batch_retrieved) {
                    log.w(batch.split_island_index);
                    log.w(offsetIn(indices, batch.constraints));
                    log.w(offsetIn(indices, batch.constraints) + @as(u32, @intCast(batch.constraints.len)));
                    log.w(offsetIn(indices, batch.contacts));
                    log.w(offsetIn(indices, batch.contacts) + @as(u32, @intCast(batch.contacts.len)));
                    log.b(batch.first_iteration);
                    log.w(self.splitter.getIslandIndex(batch.split_island_index));
                    try outstanding.append(self.allocator, batch);
                    self.countBatch(&batch);
                } else {
                    if (status == .waiting_for_batch) self.coverage.waiting += 1;
                    if (outstanding.items.len == 0)
                        return;
                    force_mark = true;
                }
            } else {
                // Mark one of the outstanding batches as processed
                const which = (d >> 1) % outstanding.items.len;
                if (which != 0) self.coverage.out_of_order_marks += 1;
                const batch = outstanding.orderedRemove(which);
                var last_iteration = false;
                var final_batch = false;
                self.splitter.markBatchProcessed(batch.split_island_index, batch.constraints, batch.contacts, &last_iteration, &final_batch);
                log.w(0xA0000000);
                log.w(batch.split_island_index);
                log.b(last_iteration);
                log.b(final_batch);
                force_mark = false;
            }
        }
        log.w(0xDEADDEAD); // Did not finish
    }

    /// Coverage of the split islands
    fn countSplits(self: *World) void {
        const splitter = &self.splitter;
        const c = self.coverage;
        for (splitter.split_islands.items[0..splitter.next_split_island.raw]) |*s| {
            c.split_islands += 1;
            if (s.num_splits == 0) c.only_non_parallel += 1;
            if (s.splits[LargeIslandSplitter.non_parallel_split_idx].getNumItems() == 0) c.empty_non_parallel += 1;
            if (s.num_position_steps == 0) c.zero_position_steps += 1;
            c.max_parallel_splits = @max(c.max_parallel_splits, s.num_splits);
        }
        for (splitter.contact_and_constraints_split_idx.items[0..splitter.contact_and_constraints_size]) |split| {
            if (split == LargeIslandSplitter.non_parallel_split_idx) c.assigned_non_parallel += 1;
        }
    }

    /// Coverage of the batch shapes of FetchNextBatch
    fn countBatch(self: *World, batch: *const LargeIslandSplitter.Batch) void {
        const c = self.coverage;
        const indices = self.splitter.contact_and_constraint_indices.items;
        c.batches += 1;
        if (batch.constraints.len > 0 and batch.contacts.len > 0) c.mixed_batches += 1;
        if (batch.constraints.len == 0 and offsetIn(indices, batch.constraints) == 0 and batch.contacts.len > 0) c.only_contact_batches += 1;
        if (batch.contacts.len == 0 and offsetIn(indices, batch.contacts) != 0) c.ends_at_last_constraint += 1;
    }
};

/// Offset of a slice of contact_and_constraint_indices (Jolt's pointer difference)
fn offsetIn(indices: []const u32, slice: []const u32) u32 {
    return @intCast((@intFromPtr(slice.ptr) -% @intFromPtr(indices.ptr)) / @sizeOf(u32));
}

/// A scene under construction
const Scene = struct {
    allocator: Allocator,
    bodies: std.ArrayList(LISBodyDesc) = .empty,
    constraints: std.ArrayList(LISConstraintDesc) = .empty,
    contacts: std.ArrayList(LISContactDesc) = .empty,
    decisions: std.ArrayList(u32) = .empty,
    num_velocity_steps: u32 = 10,
    num_position_steps: u32 = 2,
    max_outstanding: u32 = 1,
    num_updates: u32 = 2,

    fn deinit(self: *Scene) void {
        self.bodies.deinit(self.allocator);
        self.constraints.deinit(self.allocator);
        self.contacts.deinit(self.allocator);
        self.decisions.deinit(self.allocator);
    }

    fn body(self: *Scene, desc: LISBodyDesc) u32 {
        self.bodies.append(self.allocator, desc) catch @panic("OOM");
        return @intCast(self.bodies.items.len - 1);
    }

    fn contact(self: *Scene, body1: u32, body2: u32) void {
        self.contacts.append(self.allocator, .{ .body1 = body1, .body2 = body2 }) catch @panic("OOM");
    }

    fn constraint(self: *Scene, desc: LISConstraintDesc) void {
        self.constraints.append(self.allocator, desc) catch @panic("OOM");
    }

    /// A chain of dynamic bodies with contacts between neighbors, returns the first body
    fn chain(self: *Scene, num_bodies: u32, desc: LISBodyDesc) u32 {
        const first = self.body(desc);
        for (1..num_bodies) |_| {
            const b = self.body(desc);
            self.contact(b - 1, b);
        }
        return first;
    }

    fn sceneDesc(self: *Scene) LISSceneDesc {
        if (self.decisions.items.len == 0)
            self.decisions.append(self.allocator, 0) catch @panic("OOM");
        return .{
            .bodies = self.bodies.items.ptr,
            .constraints = self.constraints.items.ptr,
            .contacts = self.contacts.items.ptr,
            .decisions = self.decisions.items.ptr,
            .num_bodies = @intCast(self.bodies.items.len),
            .num_constraints = @intCast(self.constraints.items.len),
            .num_contacts = @intCast(self.contacts.items.len),
            .num_decisions = @intCast(self.decisions.items.len),
            .num_velocity_steps = self.num_velocity_steps,
            .num_position_steps = self.num_position_steps,
            .max_outstanding = self.max_outstanding,
            .num_updates = self.num_updates,
        };
    }
};

/// What the scenes reach (each must be reached by the edge cases or the random scenes)
const Coverage = struct {
    split_islands: u32 = 0,
    not_split: u32 = 0,
    only_non_parallel: u32 = 0,
    empty_non_parallel: u32 = 0,
    zero_position_steps: u32 = 0,
    max_parallel_splits: u32 = 0,
    assigned_non_parallel: u32 = 0,
    batches: u32 = 0,
    mixed_batches: u32 = 0,
    only_contact_batches: u32 = 0,
    ends_at_last_constraint: u32 = 0,
    waiting: u32 = 0,
    out_of_order_marks: u32 = 0,

    fn check(self: *const Coverage, min_hits: u32) !void {
        var ok = true;
        inline for (@typeInfo(Coverage).@"struct".fields) |field| {
            if (@field(self, field.name) < min_hits) {
                std.debug.print("LargeIslandSplitter parity: coverage {s} = {d} < {d}\n", .{ field.name, @field(self, field.name), min_hits });
                ok = false;
            }
        }
        if (!ok) return error.CoverageNotReached;
    }
};

/// Run a scene on both sides and compare the logs, returns false on a mismatch (after printing it)
fn runScene(allocator: Allocator, scene: *Scene, name: []const u8, coverage: *Coverage) !bool {
    const d = scene.sceneDesc();

    var zolt_log: Log = .{ .allocator = allocator };
    defer zolt_log.deinit();
    {
        var world: World = undefined;
        try world.init(allocator, &d, coverage);
        defer world.deinit();
        for (0..d.num_updates) |_|
            try world.update(&zolt_log);
    }

    var capacity: u32 = @intCast(zolt_log.words.items.len + 1024);
    var jolt_words = try allocator.alloc(u32, capacity);
    defer allocator.free(jolt_words);
    var jolt_size = jolt.jolt_lis_run(&d, jolt_words.ptr, capacity);
    if (jolt_size > capacity) {
        capacity = jolt_size;
        jolt_words = try allocator.realloc(jolt_words, capacity);
        jolt_size = jolt.jolt_lis_run(&d, jolt_words.ptr, capacity);
    }
    return compareLogs(name, zolt_log.words.items, jolt_words[0..jolt_size]);
}

/// Compare two logs, print the first difference with some context
fn compareLogs(name: []const u8, zolt_words: []const u32, jolt_words: []const u32) bool {
    const n = @min(zolt_words.len, jolt_words.len);
    var first: ?usize = null;
    for (0..n) |i| {
        if (zolt_words[i] != jolt_words[i]) {
            first = i;
            break;
        }
    }
    if (first == null and zolt_words.len == jolt_words.len)
        return true;
    const at = first orelse n;
    std.debug.print("LargeIslandSplitter parity {s}: logs differ at word {d} (zolt {d} words, jolt {d} words)\n", .{ name, at, zolt_words.len, jolt_words.len });
    const begin = at -| 12;
    for (begin..@min(at + 12, @max(zolt_words.len, jolt_words.len))) |i| {
        const z: ?u32 = if (i < zolt_words.len) zolt_words[i] else null;
        const j: ?u32 = if (i < jolt_words.len) jolt_words[i] else null;
        std.debug.print("  [{d}] zolt {?x} jolt {?x}{s}\n", .{ i, z, j, if (i == at) " <--" else "" });
    }
    return false;
}

test "LargeIslandSplitter parity: layout of Splits" {
    var values: [8]u32 = undefined;
    jolt.jolt_lis_layout(&values);
    const Splits = LargeIslandSplitter.Splits;
    try std.testing.expectEqual([8]u32{
        @sizeOf(Splits),
        @alignOf(Splits),
        @offsetOf(Splits, "status"),
        @offsetOf(Splits, "items_processed"),
        @sizeOf(LargeIslandSplitter.Split),
        LargeIslandSplitter.num_splits,
        LargeIslandSplitter.non_parallel_split_idx,
        LargeIslandSplitter.large_island_treshold,
    }, values);
}

const static_body: LISBodyDesc = .{ .motion_type = @intFromEnum(MotionType.static), .activate = 0 };
const kinematic_body: LISBodyDesc = .{ .motion_type = @intFromEnum(MotionType.kinematic) };
const sleeping_kinematic_body: LISBodyDesc = .{ .motion_type = @intFromEnum(MotionType.kinematic), .activate = 0 };

/// Random decisions for the order of the fetches and marks
fn randomDecisions(scene: *Scene, rng: *fw.Rng, count: usize) void {
    scene.decisions.clearRetainingCapacity();
    for (0..count) |_| scene.decisions.append(scene.allocator, rng.next()) catch @panic("OOM");
}

/// Run a scene with one batch at a time and with several batches out at the same time
fn runSceneInterleaved(allocator: Allocator, scene: *Scene, name: []const u8, rng: *fw.Rng, coverage: *Coverage) !bool {
    scene.max_outstanding = 1;
    randomDecisions(scene, rng, 1);
    var ok = try runScene(allocator, scene, name, coverage);
    scene.max_outstanding = rng.intRange(u32, 2, 8);
    randomDecisions(scene, rng, 997);
    ok = try runScene(allocator, scene, name, coverage) and ok;
    return ok;
}

/// The hand-picked scenes
fn edgeScenes(allocator: Allocator, rng: *fw.Rng, coverage: *Coverage) !u32 {
    var failures: u32 = 0;

    // Islands just below the threshold (127 items), at the threshold (128) and just above (129)
    for ([_]u32{ 127, 128, 129 }) |num_items| {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        _ = scene.chain(num_items + 1, .{});
        _ = scene.chain(num_items, .{});
        const first = scene.chain(65, .{});
        for (0..num_items - 64) |i| scene.constraint(.{ .body1 = first + @as(u32, @intCast(i % 65)), .body2 = first + @as(u32, @intCast((i % 65 + 1 + (i * 7) % 63) % 65)) });
        if (!try runSceneInterleaved(allocator, &scene, "threshold", rng, coverage)) failures += 1;
    }

    // A chain: two parallel splits of 64, the non parallel split stays empty and is skipped
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        _ = scene.chain(129, .{});
        if (!try runSceneInterleaved(allocator, &scene, "chain", rng, coverage)) failures += 1;
    }

    // All bodies static but one: every split has one contact, everything goes to the non parallel split
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        const dynamic = scene.body(.{ .velocity_steps = 3, .position_steps = 5 });
        for (0..150) |i| {
            const s = scene.body(static_body);
            if (i % 2 == 0) scene.contact(dynamic, s) else scene.contact(s, dynamic);
        }
        if (!try runSceneInterleaved(allocator, &scene, "one dynamic body", rng, coverage)) failures += 1;
    }

    // More contacts on one body than there are splits, with enough items to keep many splits parallel
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        const center = scene.body(.{});
        var ring: [40]u32 = undefined;
        for (&ring) |*b| {
            b.* = scene.body(.{});
            scene.contact(center, b.*);
        }
        for (0..40) |round| {
            for (ring, 0..) |b, i| {
                const other = scene.body(.{});
                scene.contact(b, other);
                if (round % 3 == 0) scene.constraint(.{ .body1 = other, .body2 = ring[(i + 1) % ring.len] });
            }
        }
        if (!try runSceneInterleaved(allocator, &scene, "more than the maximum number of splits", rng, coverage)) failures += 1;
    }

    // Kinematic and static bodies, non parallel constraints (also on a static and a kinematic first body), step
    // overrides, sleeping dynamic bodies that the constraints and contacts activate
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        const ground = scene.body(static_body);
        const kinematic = scene.body(kinematic_body);
        const sleeping_kinematic = scene.body(sleeping_kinematic_body);
        var grid: [12][12]u32 = undefined;
        for (0..12) |y| for (0..12) |x| {
            grid[y][x] = scene.body(.{ .activate = @intFromBool((x * 5 + y) % 7 != 0), .velocity_steps = if (x == 4) 6 else 0, .position_steps = if (y == 2) 7 else 0 });
        };
        for (0..12) |y| for (0..12) |x| {
            if (x + 1 < 12) scene.contact(grid[y][x], grid[y][x + 1]);
            if (y + 1 < 12) scene.contact(grid[y + 1][x], grid[y][x]);
            if (x + 1 < 12 and y + 1 < 12) scene.constraint(.{ .body1 = grid[y][x], .body2 = grid[y + 1][x + 1], .velocity_steps = if (x == y) 14 else 0, .non_parallel = @intFromBool(x == 3) });
        };
        for (0..12) |x| {
            scene.contact(ground, grid[11][x]);
            scene.contact(grid[0][x], if (x % 2 == 0) kinematic else sleeping_kinematic);
            scene.constraint(.{ .body1 = if (x % 3 == 0) kinematic else ground, .body2 = grid[x][0], .position_steps = 3 });
        }
        scene.constraint(.{ .body1 = ground, .body2 = grid[5][5], .non_parallel = 1 });
        scene.constraint(.{ .body1 = kinematic, .body2 = grid[5][6], .non_parallel = 1 });
        if (!try runSceneInterleaved(allocator, &scene, "kinematic and static bodies", rng, coverage)) failures += 1;

        // 0 position steps (no position batches) and 0 velocity steps (only the warm start)
        scene.num_velocity_steps = 0;
        scene.num_position_steps = 0;
        for (scene.bodies.items) |*b| b.* = .{ .motion_type = b.motion_type, .activate = b.activate };
        for (scene.constraints.items) |*c| c.* = .{ .body1 = c.body1, .body2 = c.body2, .non_parallel = c.non_parallel };
        if (!try runSceneInterleaved(allocator, &scene, "zero steps", rng, coverage)) failures += 1;
    }

    // No islands at all, and islands without constraints or contacts
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        _ = scene.body(static_body);
        if (!try runSceneInterleaved(allocator, &scene, "no islands", rng, coverage)) failures += 1;
        for (0..5) |_| _ = scene.body(.{});
        if (!try runSceneInterleaved(allocator, &scene, "no constraints", rng, coverage)) failures += 1;
    }

    return failures;
}

/// A random scene: clusters of dynamic bodies with contacts and constraints between them and to a few shared static and
/// kinematic bodies (islands above and below the threshold)
fn randomScene(scene: *Scene, rng: *fw.Rng) void {
    var shared: [8]u32 = undefined;
    const num_shared = rng.intRange(usize, 1, shared.len);
    for (shared[0..num_shared]) |*s|
        s.* = scene.body(switch (rng.intRange(u32, 0, 2)) {
            0 => static_body,
            1 => kinematic_body,
            else => sleeping_kinematic_body,
        });

    // Some scenes without step overrides (so that the settings decide, also 0 position steps)
    const overrides = rng.intRange(u32, 0, 3) != 0;
    const num_clusters = rng.intRange(u32, 1, 6);
    for (0..num_clusters) |_| {
        const num_bodies = rng.intRange(u32, 1, 90);
        const first: u32 = @intCast(scene.bodies.items.len);
        for (0..num_bodies) |_|
            _ = scene.body(.{
                .activate = @intFromBool(rng.intRange(u32, 0, 7) != 0),
                .velocity_steps = if (overrides and rng.intRange(u32, 0, 7) == 0) rng.intRange(u32, 1, 15) else 0,
                .position_steps = if (overrides and rng.intRange(u32, 0, 7) == 0) rng.intRange(u32, 1, 15) else 0,
            });
        const num_items = rng.intRange(u32, 20, 500);
        for (0..num_items) |_| {
            var b1 = first + rng.intRange(u32, 0, num_bodies - 1);
            var b2 = if (num_bodies > 1 and rng.intRange(u32, 0, 3) != 0) first + rng.intRange(u32, 0, num_bodies - 1) else shared[rng.intRange(usize, 0, num_shared - 1)];
            if (b1 == b2) continue;
            if (rng.intRange(u32, 0, 1) == 0) std.mem.swap(u32, &b1, &b2);
            if (rng.intRange(u32, 0, 2) != 0) {
                scene.contact(b1, b2);
            } else {
                scene.constraint(.{
                    .body1 = b1,
                    .body2 = b2,
                    .velocity_steps = if (overrides and rng.intRange(u32, 0, 5) == 0) rng.intRange(u32, 1, 15) else 0,
                    .position_steps = if (overrides and rng.intRange(u32, 0, 5) == 0) rng.intRange(u32, 1, 15) else 0,
                    .non_parallel = @intFromBool(rng.intRange(u32, 0, 9) == 0),
                });
            }
        }
    }
    scene.num_velocity_steps = rng.intRange(u32, 0, 12);
    scene.num_position_steps = rng.intRange(u32, 0, 5);
}

test "LargeIslandSplitter parity: edge cases and random islands" {
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{};
    var coverage: Coverage = .{};
    var failures = try edgeScenes(allocator, &rng, &coverage);

    for (0..250) |i| {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        randomScene(&scene, &rng);
        if (!try runSceneInterleaved(allocator, &scene, "random", &rng, &coverage)) {
            failures += 1;
            std.debug.print("  (random scene {d})\n", .{i});
            if (failures > 5) break;
        }
    }

    // One large scene: many islands, many split islands at the same time
    {
        var scene: Scene = .{ .allocator = allocator };
        defer scene.deinit();
        for (0..12) |_| randomScene(&scene, &rng);
        scene.num_updates = 1;
        scene.max_outstanding = 8;
        randomDecisions(&scene, &rng, 4099);
        if (!try runScene(allocator, &scene, "large", &coverage)) failures += 1;
    }

    try std.testing.expectEqual(@as(u32, 0), failures);
    try coverage.check(5);
    try std.testing.expect(coverage.max_parallel_splits >= 10);
}

test "LargeIslandSplitter parity: AssignSplit and AssignToNonParallelSplit" {
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{ .state = 0x5eed1234 };
    var num_ops_total: usize = 0;
    var mismatches: u32 = 0;
    for (0..50) |world_index| {
        // Bodies of every kind, active or not
        var bodies: [64]LISBodyDesc = undefined;
        for (&bodies) |*b| b.* = switch (rng.intRange(u32, 0, 5)) {
            0 => static_body,
            1 => kinematic_body,
            2 => sleeping_kinematic_body,
            3 => .{ .activate = 0 },
            else => .{},
        };

        // Zolt: a body manager with the bodies, the masks of the active bodies
        var layers: ParityLayers = .{};
        var body_manager: BodyManager = .empty;
        try body_manager.init(allocator, io, bodies.len + 1, 0, &layers.base);
        defer body_manager.deinit();
        var shape: RefConst(Shape) = .init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());
        defer shape.deinit();
        var body_ptrs: [bodies.len]*Body = undefined;
        for (bodies, &body_ptrs) |b, *ptr| {
            const motion_type: MotionType = @enumFromInt(b.motion_type);
            var settings = BodyCreationSettings.initWithShape(shape.get().?, RVec3.zero(), Quat.identity(), motion_type, if (motion_type == .static) 0 else 1);
            defer settings.deinit();
            ptr.* = try body_manager.allocateBody(&settings);
            _ = body_manager.addBody(ptr.*);
            ptr.*.setInBroadPhaseInternal(true);
            if (b.activate != 0 and !ptr.*.isStatic())
                body_manager.activateBodies(&.{ptr.*.getID()});
        }
        const num_active = body_manager.getNumActiveBodies(.rigid_body);
        if (num_active == 0) continue;
        const masks = try allocator.alloc(u32, num_active);
        defer allocator.free(masks);
        for (masks) |*m| m.* = switch (rng.intRange(u32, 0, 5)) {
            0 => 0,
            1 => 0xffffffff,
            2 => 0x7fffffff,
            3 => ~(@as(u32, 1) << @intCast(rng.intRange(u32, 0, 31))),
            else => rng.next() & rng.next(),
        };
        const initial_masks = try allocator.dupe(u32, masks);
        defer allocator.free(initial_masks);

        // Valid calls: AssignSplit needs an active body 2 when body 1 is not an active dynamic body
        var ops: std.ArrayList(u32) = .empty;
        defer ops.deinit(allocator);
        const num_ops = 2000;
        for (0..num_ops) |_| {
            const kind: u32 = @intFromBool(rng.intRange(u32, 0, 4) == 0);
            const b1 = rng.intRange(u32, 0, bodies.len - 1);
            var b2 = rng.intRange(u32, 0, bodies.len - 1);
            if (kind == 0) {
                const body1 = body_ptrs[b1];
                if (!(body1.isDynamic() and body1.getIndexInActiveBodiesInternal() != Body.inactive_index)) {
                    while (body_ptrs[b2].getIndexInActiveBodiesInternal() == Body.inactive_index)
                        b2 = rng.intRange(u32, 0, bodies.len - 1);
                }
            }
            try ops.appendSlice(allocator, &.{ kind, b1, b2 });
        }

        // Zolt
        var zolt_log: Log = .{ .allocator = allocator };
        defer zolt_log.deinit();
        var splitter: LargeIslandSplitter = .empty;
        splitter.num_active_bodies = num_active;
        splitter.split_masks.items = masks;
        var i: usize = 0;
        while (i < ops.items.len) : (i += 3) {
            const op = ops.items[i..][0..3];
            zolt_log.w(if (op[0] == 0) splitter.assignSplit(body_ptrs[op[1]], body_ptrs[op[2]]) else splitter.assignToNonParallelSplit(body_ptrs[op[1]]));
            for (masks) |m| zolt_log.w(m);
        }
        splitter.split_masks = .empty;
        splitter.num_active_bodies = 0;
        splitter.deinit();

        // Jolt
        const capacity: u32 = @intCast(zolt_log.words.items.len + 16);
        const jolt_words = try allocator.alloc(u32, capacity);
        defer allocator.free(jolt_words);
        const jolt_size = jolt.jolt_lis_assign(&bodies, bodies.len, initial_masks.ptr, ops.items.ptr, num_ops, jolt_words.ptr, capacity);
        if (!compareLogs("AssignSplit", zolt_log.words.items, jolt_words[0..jolt_size])) {
            std.debug.print("  (world {d})\n", .{world_index});
            mismatches += 1;
        }
        num_ops_total += num_ops;
    }
    try std.testing.expectEqual(@as(u32, 0), mismatches);
    try std.testing.expect(num_ops_total >= 90_000);
}
