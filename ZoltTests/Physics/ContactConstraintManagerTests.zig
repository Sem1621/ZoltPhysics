//! Zolt addition, no Jolt file: tests of ContactConstraintManager (SimulationArchitecture D10)
//! Status: complete
//!
//! Jolt has no unit test for the ContactConstraintManager (its behavior is covered by ContactListenerTests and
//! PhysicsTests, which need the complete PhysicsSystem of Wave B). These tests drive the manager directly like
//! PhysicsSystem does in one collision step: PrepareConstraintBuffer and the island builder's contact links, then per
//! body pair GetContactsFromCache, AddBodyPair and AddContactConstraint with hand made manifolds, the island
//! finalize, ConstraintIdxToConstraintOffset + SortContacts + WarmStart + SolveVelocity + StoreAppliedImpulses +
//! SolvePosition per island, FinalizeContactCacheAndCallContactPointRemovedCallbacks and FinishConstraintBuffer.
//! They cover every motion type combination (sensors, kinematic vs static), manifolds of 1 to 4 points, the listener
//! overrides of the contact settings, the soft errors (contact constraints / manifold cache / body pair cache full),
//! several frames through the contact cache (persisted, removed and re-found contacts, preserved lambdas), CCD
//! contacts, WereBodiesInContact, SaveState / RestoreState round trips (validation, filters, multiple parts),
//! AddContactConstraint from several threads against a single threaded reference, allocation failures in `init`, and
//! the D10 proof of finalizeContactCacheAfterOutOfMemory: every temp allocation of a frame with contacts fails in
//! turn, the temp allocator ends empty, both caches stay structurally valid, no OnContactRemoved is called, and
//! restoreState followed by the frame gives the reference result.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig");

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const BoxShape = zolt.BoxShape;
const CalculateSolverSteps = zolt.CalculateSolverSteps;
const ContactConstraintManager = zolt.ContactConstraintManager;
const ContactListener = zolt.ContactListener;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const Core = zolt.Core;
const DummyCalculateSolverSteps = zolt.DummyCalculateSolverSteps;
const FailingTempAllocator = zolt.FailingTempAllocator;
const IslandBuilder = zolt.IslandBuilder;
const MotionType = zolt.MotionType;
const Mutex = zolt.Mutex;
const PhysicsSettings = zolt.PhysicsSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsUpdateContext = zolt.PhysicsUpdateContext;
const PhysicsUpdateError = zolt.PhysicsUpdateError;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const StateRecorderFilter = zolt.StateRecorderFilter;
const StateRecorderImpl = zolt.StateRecorderImpl;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDPair = zolt.SubShapeIDPair;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const Vec3 = zolt.Vec3;

const CCM = ContactConstraintManager;
const ContactAllocator = CCM.ContactAllocator;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const expectEqualSlices = std.testing.expectEqualSlices;

const io = std.testing.io;

/// The motion type combinations that can collide (Jolt's dispatch tables have no static vs static)
const motion_type_pairs = [_][2]MotionType{
    .{ .static, .kinematic },
    .{ .static, .dynamic },
    .{ .kinematic, .static },
    .{ .kinematic, .kinematic },
    .{ .kinematic, .dynamic },
    .{ .dynamic, .static },
    .{ .dynamic, .kinematic },
    .{ .dynamic, .dynamic },
};

/// A listener callback as a record
const Record = struct {
    pub const Kind = enum(u8) { added, persisted, removed };

    kind: Kind,
    body1: BodyID,
    body2: BodyID,
    sub_shape_id1: u32,
    sub_shape_id2: u32,
    num_points: u32 = 0,
    normal: Vec3 = Vec3.zero(),
    penetration_depth: f32 = 0.0,

    fn lessThan(_: void, a: Record, b: Record) bool {
        if (a.kind != b.kind) return @intFromEnum(a.kind) < @intFromEnum(b.kind);
        if (!a.body1.eql(b.body1)) return a.body1.lessThan(b.body1);
        if (!a.body2.eql(b.body2)) return a.body2.lessThan(b.body2);
        if (a.sub_shape_id1 != b.sub_shape_id1) return a.sub_shape_id1 < b.sub_shape_id1;
        return a.sub_shape_id2 < b.sub_shape_id2;
    }
};

/// A contact listener that records the callbacks (thread safe) and can override the contact settings
const RecordingListener = struct {
    const max_records = 4096;

    mutex: Mutex = .{},
    records: [max_records]Record = undefined,
    num_records: usize = 0,

    /// Called from OnContactAdded / OnContactPersisted to override the settings (null: keep the defaults)
    modify: ?*const fn (body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void = null,

    fn add(self: *RecordingListener, record: Record) void {
        self.mutex.lock(io);
        defer self.mutex.unlock(io);
        std.debug.assert(self.num_records < max_records);
        self.records[self.num_records] = record;
        self.num_records += 1;
    }

    fn reset(self: *RecordingListener) void {
        self.num_records = 0;
    }

    fn slice(self: *RecordingListener) []Record {
        return self.records[0..self.num_records];
    }

    fn count(self: *RecordingListener, kind: Record.Kind) usize {
        var n: usize = 0;
        for (self.slice()) |r| {
            if (r.kind == kind) n += 1;
        }
        return n;
    }

    fn sort(self: *RecordingListener) void {
        std.mem.sort(Record, self.slice(), {}, Record.lessThan);
    }

    fn manifoldRecord(kind: Record.Kind, body1: *const Body, body2: *const Body, manifold: *const ContactManifold) Record {
        return .{
            .kind = kind,
            .body1 = body1.getID(),
            .body2 = body2.getID(),
            .sub_shape_id1 = manifold.sub_shape_id1.getValue(),
            .sub_shape_id2 = manifold.sub_shape_id2.getValue(),
            .num_points = manifold.relative_contact_points_on1.len,
            .normal = manifold.world_space_normal,
            .penetration_depth = manifold.penetration_depth,
        };
    }

    pub fn onContactAdded(self: *RecordingListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        // Body 1 and 2 will be sorted such that body 1 ID < body 2 ID
        std.debug.assert(body1.getID().lessThan(body2.getID()));
        self.add(manifoldRecord(.added, body1, body2, manifold));
        if (self.modify) |f| f(body1, body2, manifold, settings);
    }

    pub fn onContactPersisted(self: *RecordingListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        std.debug.assert(body1.getID().lessThan(body2.getID()));
        self.add(manifoldRecord(.persisted, body1, body2, manifold));
        if (self.modify) |f| f(body1, body2, manifold, settings);
    }

    pub fn onContactRemoved(self: *RecordingListener, sub_shape_pair: *const SubShapeIDPair) void {
        std.debug.assert(sub_shape_pair.getBody1ID().lessThan(sub_shape_pair.getBody2ID()));
        self.add(.{
            .kind = .removed,
            .body1 = sub_shape_pair.getBody1ID(),
            .body2 = sub_shape_pair.getBody2ID(),
            .sub_shape_id1 = sub_shape_pair.getSubShapeID1().getValue(),
            .sub_shape_id2 = sub_shape_pair.getSubShapeID2().getValue(),
        });
    }
};

/// Options of a test body
const BoxOptions = struct {
    half_extent: Vec3 = Vec3.one(),
    rotation: Quat = Quat.identity(),
    linear_velocity: Vec3 = Vec3.zero(),
    angular_velocity: Vec3 = Vec3.zero(),
    friction: f32 = 0.2,
    restitution: f32 = 0.0,
    is_sensor: bool = false,
    activate: bool = true,
    gravity_factor: f32 = 1.0,
};

/// The parts of a PhysicsSystem that the contact constraint manager uses, driven like PhysicsSystem drives them
const Harness = struct {
    allocator: Allocator,
    layers: Layers.BPLayerInterfaceImpl = .init(),
    body_manager: BodyManager = .empty,
    island_builder: IslandBuilder = .empty,
    /// Only for the gravity (an `empty` system, like a PhysicsSystem before Init)
    system: PhysicsSystem = .empty,
    settings: PhysicsSettings = .{},
    manager: CCM = .empty,
    temp: TempAllocatorImpl = undefined,
    listener: RecordingListener = .{},
    context: PhysicsUpdateContext = undefined,
    contact_allocator: ContactAllocator = undefined,
    temp_allocator: TempAllocator = undefined,

    const Options = struct {
        max_bodies: u32 = 256,
        max_body_pairs: u32 = 256,
        max_contact_constraints: u32 = 256,
        listener: bool = true,
    };

    /// In place: the managers point into the harness
    fn init(self: *Harness, allocator: Allocator, opts: Options) !void {
        self.* = .{ .allocator = allocator };
        self.temp = try .init(allocator, 8 * 1024 * 1024);
        errdefer self.temp.deinit();
        try self.body_manager.init(allocator, io, opts.max_bodies, 0, &self.layers.base);
        errdefer self.body_manager.deinit();
        try self.island_builder.init(allocator, opts.max_bodies);
        errdefer self.island_builder.deinit();
        try self.manager.init(allocator, &self.settings, opts.max_body_pairs, opts.max_contact_constraints);
        if (opts.listener)
            self.manager.setContactListener(ContactListener.init(&self.listener));
    }

    fn deinit(self: *Harness) void {
        // A failed test can leave a step open
        if (!self.manager.constraints.isEmpty())
            self.finishStep();
        self.manager.deinit();
        self.island_builder.deinit();
        var ids: zolt.BodyIDVector = .empty;
        defer ids.deinit(self.allocator);
        self.body_manager.getBodyIDs(self.allocator, &ids) catch unreachable;
        for (ids.items) |id| {
            self.body_manager.deactivateBodies(&.{id});
            self.body_manager.getBodyMut(id).setInBroadPhaseInternal(false);
            self.body_manager.destroyBodies(&.{id});
        }
        self.body_manager.deinit();
        self.temp.deinit();
    }

    /// Create a box body (BodyInterface::CreateAndAddBody without a broad phase)
    fn createBox(self: *Harness, motion_type: MotionType, position: RVec3, opts: BoxOptions) !*Body {
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        settings.setShape((try BoxShape.create(self.allocator, opts.half_extent, .{ .convex_radius = 0.05 })).asShape());
        settings.position = position;
        settings.rotation = opts.rotation;
        settings.motion_type = motion_type;
        settings.object_layer = if (motion_type == .static) Layers.Layers.non_moving else Layers.Layers.moving;
        settings.linear_velocity = opts.linear_velocity;
        settings.angular_velocity = opts.angular_velocity;
        settings.friction = opts.friction;
        settings.restitution = opts.restitution;
        settings.is_sensor = opts.is_sensor;
        settings.gravity_factor = opts.gravity_factor;
        const body = try self.body_manager.allocateBody(&settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return error.OutOfBodies;
        }
        body.setInBroadPhaseInternal(true);
        if (opts.activate and motion_type != .static)
            self.body_manager.activateBodies(&.{body.getID()});
        return body;
    }

    /// The start of a collision step: PrepareConstraintBuffer and the island builder's contact links (PhysicsSystem's
    /// build phase), then the contact allocator of the FindCollisions job
    fn beginStep(self: *Harness, temp_allocator: TempAllocator, delta_time: f32, warm_start_impulse_ratio: f32) TempAllocator.Error!void {
        self.temp_allocator = temp_allocator;
        self.context = .{
            .physics_system = &self.system,
            .temp_allocator = temp_allocator,
            .job_system = undefined,
            .barrier = undefined,
            .body_manager = &self.body_manager,
            .island_builder = &self.island_builder,
            .step_delta_time = delta_time,
            .warm_start_impulse_ratio = warm_start_impulse_ratio,
        };
        try self.manager.prepareConstraintBuffer(&self.context);
        try self.island_builder.prepareContactConstraints(self.manager.getMaxConstraints(), temp_allocator);
        self.contact_allocator = self.manager.getContactAllocator();
    }

    /// ProcessBodyPair without the narrow phase: the contact cache first, then the manifolds
    fn processPair(self: *Harness, contact_allocator: *ContactAllocator, body1: *Body, body2: *Body, manifolds: []const ContactManifold, use_cache: bool) void {
        if (use_cache and self.manager.getContactsFromCache(contact_allocator, body1, body2))
            return;
        const handle = self.manager.addBodyPair(contact_allocator, body1, body2) orelse return; // Out of cache space
        var link_and_activate_bodies = true;
        for (manifolds) |*m|
            self.manager.addContactConstraint(contact_allocator, &link_and_activate_bodies, handle, body1, body2, m);
    }

    /// FinalizeIslands and the contact part of SolveVelocityConstraints / SolvePositionConstraints for every island
    fn solve(self: *Harness, num_velocity_steps: u32, num_position_steps: u32) TempAllocator.Error!void {
        const num_active_bodies = self.body_manager.getNumActiveBodies(.rigid_body);
        try self.island_builder.finalize(self.body_manager.getActiveBodiesUnsafe(.rigid_body)[0..num_active_bodies], self.manager.getNumConstraints(), self.temp_allocator);
        var island: u32 = 0;
        while (island < self.island_builder.getNumIslands()) : (island += 1) {
            const contacts = self.island_builder.getContactsInIsland(island);
            if (contacts.len == 0)
                continue;
            self.manager.constraintIdxToConstraintOffset(contacts);
            self.manager.sortContacts(contacts);
            var steps_calculator = CalculateSolverSteps.init(&self.settings);
            self.manager.warmStartVelocityConstraints(contacts, self.context.warm_start_impulse_ratio, &steps_calculator);
            steps_calculator.finalize();
            for (0..num_velocity_steps) |_| {
                if (!self.manager.solveVelocityConstraints(contacts))
                    break;
            }
            self.manager.storeAppliedImpulses(contacts);
            for (0..num_position_steps) |_| {
                if (!self.manager.solvePositionConstraints(contacts))
                    break;
            }
        }
    }

    /// The end of the step: the ContactRemovedCallbacks job and PhysicsSystem's frees
    fn endStep(self: *Harness) void {
        self.manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(self.contact_allocator.num_body_pairs, self.contact_allocator.num_manifolds);
        self.finishStep();
    }

    fn finishStep(self: *Harness) void {
        self.island_builder.resetIslands(self.temp_allocator);
        self.manager.finishConstraintBuffer();
    }

    /// The contact constraint with index `i` (in creation order)
    fn constraint(self: *Harness, i: u32) *CCM.ContactConstraintBase {
        std.debug.assert(i < self.manager.getNumConstraints());
        const offset = self.manager.constraint_idx_to_offset.items[i];
        return @ptrCast(@alignCast(self.manager.constraints.items.ptr + offset));
    }

    /// The SaveState bytes of the contact cache
    fn saveContacts(self: *Harness, out: *std.ArrayList(u8)) !void {
        var recorder: StateRecorderImpl = .init(self.allocator);
        defer recorder.deinit();
        try self.manager.saveState(recorder.recorder(), null);
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    /// The SaveState bytes of the bodies and the contact cache
    fn saveAll(self: *Harness, recorder: *StateRecorderImpl) !void {
        recorder.clear();
        try self.body_manager.saveState(recorder.recorder(), null);
        try self.manager.saveState(recorder.recorder(), null);
    }

    fn restoreAll(self: *Harness, recorder: *StateRecorderImpl) !void {
        recorder.rewind();
        try expect(try self.body_manager.restoreState(recorder.recorder()));
        try expect(try self.manager.restoreState(recorder.recorder(), null));
    }
};

/// A manifold between body 1 above body 2 (normal from body 1 to body 2 is -Y) with `num_points` points around
/// `center` in the XZ plane, body 1's points `penetration` below body 2's (negative: a speculative contact)
fn makeManifold(base_offset: RVec3, center: RVec3, num_points: u32, penetration: f32, sub_shape_id1: u32, sub_shape_id2: u32) ContactManifold {
    var m: ContactManifold = .{};
    m.base_offset = base_offset;
    m.world_space_normal = Vec3.init(0, -1, 0);
    m.penetration_depth = penetration;
    m.sub_shape_id1.setValue(sub_shape_id1);
    m.sub_shape_id2.setValue(sub_shape_id2);
    const offsets = [4][2]f32{ .{ -0.5, -0.5 }, .{ 0.5, -0.5 }, .{ 0.5, 0.5 }, .{ -0.5, 0.5 } };
    for (0..num_points) |i| {
        const p2 = center.add(RVec3.init(offsets[i][0], 0.0, offsets[i][1]));
        const p1 = p2.add(RVec3.init(0.0, -penetration, 0.0));
        m.relative_contact_points_on1.append(toVec3(p1.sub(base_offset)));
        m.relative_contact_points_on2.append(toVec3(p2.sub(base_offset)));
    }
    return m;
}

fn toVec3(v: RVec3) Vec3 {
    return if (Core.double_precision) v.toVec3() else v;
}

/// The relative velocity of body 2 vs body 1 at a world space point along the -Y normal (> 0: separating)
fn separatingVelocity(body1: *const Body, body2: *const Body, point: RVec3) f32 {
    const v1 = body1.getPointVelocity(point);
    const v2 = body2.getPointVelocity(point);
    return v2.sub(v1).dot(Vec3.init(0, -1, 0));
}

test "ContactConstraintManager: init and deinit, every allocation failing in turn" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            const settings: PhysicsSettings = .{};
            var manager: CCM = .empty;
            try manager.init(allocator, &settings, 100, 50);
            defer manager.deinit();
            try expectEqual(@as(u32, 50), manager.getMaxConstraints());
            try expectEqual(@as(u32, 0), manager.getNumConstraints());
            try expect(!manager.wereBodiesInContact(.init(1), .init(2)));
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, S.run, .{});

    // The minimum of 4 constraints and 4 body pairs
    const settings: PhysicsSettings = .{};
    var manager: CCM = .empty;
    try manager.init(std.testing.allocator, &settings, 0, 0);
    defer manager.deinit();
    try expectEqual(@as(u32, 4), manager.getMaxConstraints());
    try expectEqual(@as(u32, 4 * 40 + 4 * 164), manager.cache[0].lfhm_allocator.object_store_size_bytes);
    try expectEqual(@as(u32, 4), manager.cache[0].cached_manifolds.getMaxBuckets());

    // An empty manager owns nothing
    var empty: CCM = .empty;
    empty.deinit();
}

test "ContactConstraintManager: combine functions, listener and validate" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{ .listener = false });
    defer h.deinit();

    const b1 = try h.createBox(.dynamic, RVec3.init(0, 1, 0), .{ .friction = 0.4, .restitution = 0.3 });
    const b2 = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .friction = 0.9, .restitution = 0.7 });
    const sub: SubShapeID = .{};

    // Defaults: geometric mean and max
    try expectEqual(@sqrt(@as(f32, 0.4) * 0.9), h.manager.getCombineFriction()(b1, sub, b2, sub));
    try expectEqual(@as(f32, 0.7), h.manager.getCombineRestitution()(b1, sub, b2, sub));

    const Custom = struct {
        fn friction(body1: *const Body, _: SubShapeID, body2: *const Body, _: SubShapeID) f32 {
            return body1.getFriction() + body2.getFriction();
        }
        fn restitution(body1: *const Body, _: SubShapeID, body2: *const Body, _: SubShapeID) f32 {
            return body1.getRestitution() * body2.getRestitution();
        }
    };
    h.manager.setCombineFriction(Custom.friction);
    h.manager.setCombineRestitution(Custom.restitution);
    try expect(h.manager.getCombineFriction() == Custom.friction);
    try expect(h.manager.getCombineRestitution() == Custom.restitution);

    // Without a listener every contact is accepted
    try expect(h.manager.getContactListener() == null);
    const result: zolt.CollideShapeResult = .{};
    try expectEqual(zolt.ValidateResult.accept_all_contacts_for_this_body_pair, h.manager.validateContactPoint(b1, b2, RVec3.zero(), &result));

    // A listener decides
    const Validator = struct {
        pub fn onContactValidate(_: *@This(), body1: *const Body, body2: *const Body, base_offset: RVec3, _: *const zolt.CollideShapeResult) zolt.ValidateResult {
            _ = body2;
            return if (base_offset.getX() > 0.0 and body1.isDynamic()) .reject_all_contacts_for_this_body_pair else .accept_contact;
        }
    };
    var validator: Validator = .{};
    h.manager.setContactListener(ContactListener.init(&validator));
    try expect(h.manager.getContactListener().?.ptr == @as(*anyopaque, @ptrCast(&validator)));
    try expectEqual(zolt.ValidateResult.reject_all_contacts_for_this_body_pair, h.manager.validateContactPoint(b1, b2, RVec3.init(1, 0, 0), &result));
    try expectEqual(zolt.ValidateResult.accept_contact, h.manager.validateContactPoint(b1, b2, RVec3.init(-1, 0, 0), &result));

    // The custom combine functions end up in the constraint
    h.manager.setContactListener(null);
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
    const m = makeManifold(b1.getCenterOfMassPosition(), RVec3.init(0, 0, 0), 2, 0.01, 0, 0);
    h.processPair(&h.contact_allocator, b1, b2, (&m)[0..1], true);
    try expectEqual(@as(u32, 1), h.manager.getNumConstraints());
    try expectEqual(@as(f32, 0.4 + 0.9), h.constraint(0).combined_friction);
    try h.solve(10, 2);
    h.endStep();
    try expect(h.temp.isEmpty());
}

/// One collision step between a body of type1 above a body of type2 with a manifold of `num_points` points, the
/// bodies passed to the manager in the given order
fn checkMotionTypePair(allocator: Allocator, type1: MotionType, type2: MotionType, num_points: u32, swap: bool) !void {
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();
    h.system.setGravity(Vec3.init(0, -9.81, 0));

    // Body 1 falls onto body 2 (or body 2 rises into body 1), the contact is penetrating
    const top = try h.createBox(type1, RVec3.init(0.1, 0.99, -0.2), .{ .linear_velocity = if (type1 != .static) Vec3.init(0.3, -2.0, 0.1) else Vec3.zero(), .angular_velocity = if (type1 != .static) Vec3.init(0.0, 0.5, 0.2) else Vec3.zero(), .restitution = 0.5, .friction = 0.6 });
    const bottom = try h.createBox(type2, RVec3.init(0, -1, 0), .{ .linear_velocity = if (type2 != .static) Vec3.init(-0.1, 0.5, 0.0) else Vec3.zero(), .friction = 0.5 });
    try expect(top.getID().lessThan(bottom.getID()));
    const any_dynamic = type1 == .dynamic or type2 == .dynamic;

    const manifold = makeManifold(top.getCenterOfMassPosition(), RVec3.init(0.1, 0.0, -0.2), num_points, 0.01, 3, 5);
    const swapped = manifold.swapShapes();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
    if (swap)
        h.processPair(&h.contact_allocator, bottom, top, (&swapped)[0..1], true)
    else
        h.processPair(&h.contact_allocator, top, bottom, (&manifold)[0..1], true);

    // The listener sees body 1 < body 2 and the manifold in that order
    try expectEqual(@as(usize, 1), h.listener.num_records);
    const r = h.listener.records[0];
    try expectEqual(Record.Kind.added, r.kind);
    try expect(r.body1.eql(top.getID()) and r.body2.eql(bottom.getID()));
    try expectEqual(@as(u32, 3), r.sub_shape_id1);
    try expectEqual(@as(u32, 5), r.sub_shape_id2);
    try expectEqual(num_points, r.num_points);
    try expect(r.normal.isClose(Vec3.init(0, -1, 0), .{}));

    // A constraint exists only when there is a dynamic body
    try expectEqual(@as(u32, if (any_dynamic) 1 else 0), h.manager.getNumConstraints());
    try expectEqual(@as(u32, 1), h.contact_allocator.num_body_pairs);
    try expectEqual(@as(u32, 1), h.contact_allocator.num_manifolds);
    try expect(h.contact_allocator.errors.isNone());
    if (any_dynamic) {
        const c = h.constraint(0);
        try expect(c.body1 == top and c.body2 == bottom);
        try expectEqual(num_points, c.num_contact_points);
        try expect(c.getWorldSpaceNormal().isClose(Vec3.init(0, -1, 0), .{}));
        const t = c.getTangents();
        try expect(@abs(t.tangent1.dot(c.getWorldSpaceNormal())) < 1.0e-6);
        try expect(@abs(t.tangent2.dot(c.getWorldSpaceNormal())) < 1.0e-6);
        try expectEqual(@sqrt(@as(f32, 0.6) * 0.5), c.combined_friction);
        try expectEqual(if (type1 == .dynamic) top.getMotionProperties().getInverseMass() else 0.0, c.inv_mass1);
        try expectEqual(if (type2 == .dynamic) bottom.getMotionProperties().getInverseMass() else 0.0, c.inv_mass2);
        const affected = h.manager.getAffectedBodies(h.manager.constraint_idx_to_offset.items[0]);
        try expect(affected.body1 == top and affected.body2 == bottom);
    }

    // Solve: the bodies no longer approach each other at the contact
    try h.solve(10, 2);
    if (any_dynamic) {
        for (0..num_points) |i| {
            const point = manifold.getWorldSpaceContactPointOn2(@intCast(i));
            try expect(separatingVelocity(top, bottom, point) > -0.02); // The approach speed was 2.5 m/s, the iterations converge to 0
        }
    }
    h.endStep();
    try expect(h.temp.isEmpty());
    try expect(h.manager.wereBodiesInContact(top.getID(), bottom.getID()));
    try expect(h.manager.wereBodiesInContact(bottom.getID(), top.getID()));

    // The cache holds the manifold, with the applied impulses when solved
    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try h.saveContacts(&bytes);
    try expectEqual(@as(usize, 4 + 8 + 24 + 4 + 16 + 2 + 24 + 28 * num_points + 4), bytes.items.len);
    try expectEqual(@as(u32, 1), std.mem.readInt(u32, bytes.items[0..4], .little));
}

test "ContactConstraintManager: AddContactConstraint for every motion type combination and number of points" {
    for (motion_type_pairs) |pair| {
        for (1..5) |n| {
            try checkMotionTypePair(std.testing.allocator, pair[0], pair[1], @intCast(n), false);
            try checkMotionTypePair(std.testing.allocator, pair[0], pair[1], @intCast(n), true);
        }
    }
}

test "ContactConstraintManager: sensors, mass scales and friction from the listener" {
    const allocator = std.testing.allocator;

    const Case = enum { sensor, no_mass, half_mass, no_friction, surface_velocity };
    for ([_]Case{ .sensor, .no_mass, .half_mass, .no_friction, .surface_velocity }) |case| {
        var h: Harness = undefined;
        try h.init(allocator, .{});
        defer h.deinit();

        const top = try h.createBox(.dynamic, RVec3.init(0, 0.99, 0), .{ .linear_velocity = Vec3.init(1, -1, 0), .is_sensor = case == .sensor });
        const bottom = try h.createBox(.dynamic, RVec3.init(0, -1.01, 0), .{});

        const Modifiers = struct {
            fn noMass(_: *const Body, _: *const Body, _: *const ContactManifold, settings: *ContactSettings) void {
                settings.inv_mass_scale1 = 0.0;
                settings.inv_mass_scale2 = 0.0;
            }
            fn halfMass(_: *const Body, _: *const Body, _: *const ContactManifold, settings: *ContactSettings) void {
                settings.inv_mass_scale1 = 0.5;
                settings.inv_inertia_scale1 = 0.25;
                settings.inv_mass_scale2 = 2.0;
                settings.inv_inertia_scale2 = 0.0;
                settings.combined_restitution = 0.8;
            }
            fn noFriction(_: *const Body, _: *const Body, _: *const ContactManifold, settings: *ContactSettings) void {
                settings.combined_friction = 0.0;
            }
            fn surfaceVelocity(_: *const Body, _: *const Body, _: *const ContactManifold, settings: *ContactSettings) void {
                settings.relative_linear_surface_velocity = Vec3.init(2, 0, 0);
                settings.relative_angular_surface_velocity = Vec3.init(0, 1, 0);
            }
        };
        h.listener.modify = switch (case) {
            .sensor => null,
            .no_mass => Modifiers.noMass,
            .half_mass => Modifiers.halfMass,
            .no_friction => Modifiers.noFriction,
            .surface_velocity => Modifiers.surfaceVelocity,
        };

        for (0..3) |frame| {
            h.listener.reset();
            try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, if (frame == 0) 0.0 else 1.0);
            const m = makeManifold(top.getCenterOfMassPosition(), RVec3.init(0, -0.01, 0), 4, 0.01, 0, 0);
            h.processPair(&h.contact_allocator, top, bottom, (&m)[0..1], true);
            try expectEqual(@as(usize, 1), h.listener.num_records);
            try expectEqual(if (frame == 0) Record.Kind.added else Record.Kind.persisted, h.listener.records[0].kind);
            switch (case) {
                .sensor, .no_mass => try expectEqual(@as(u32, 0), h.manager.getNumConstraints()),
                .half_mass => {
                    try expectEqual(@as(u32, 1), h.manager.getNumConstraints());
                    const c = h.constraint(0);
                    try expectEqual(0.5 * top.getMotionProperties().getInverseMass(), c.inv_mass1);
                    try expectEqual(2.0 * bottom.getMotionProperties().getInverseMass(), c.inv_mass2);
                    try expectEqual(@as(f32, 0.25), c.inv_inertia_scale1);
                    try expectEqual(@as(f32, 0.0), c.inv_inertia_scale2);
                },
                .no_friction => {
                    const c = CCM.ContactConstraint(.dynamic, .dynamic).fromBase(h.constraint(0));
                    try expect(!c.friction_constraint1.isActive());
                    try expect(!c.friction_constraint2.isActive());
                    try expect(!c.angular_friction_constraint.isActive());
                },
                .surface_velocity => {
                    const c = CCM.ContactConstraint(.dynamic, .dynamic).fromBase(h.constraint(0));
                    try expect(c.friction_constraint1.isActive());
                    try expect(c.angular_friction_constraint.isActive());
                    try expect(c.friction_constraint1.bias != 0.0 or c.friction_constraint2.bias != 0.0);
                    try expect(c.angular_friction_constraint.bias != 0.0);
                },
            }
            try h.solve(10, 2);
            h.endStep();
            try expect(h.manager.wereBodiesInContact(top.getID(), bottom.getID()));
        }

        // Without a constraint the cached impulses stay zero
        if (case == .sensor or case == .no_mass) {
            const read_cache = &h.manager.cache[h.manager.cache_write_idx ^ 1];
            var it = read_cache.cached_manifolds.begin();
            const kv = it.next().?;
            try expectEqual(@as(f32, 0.0), kv.getValue().getContactPoints()[0].non_penetration_lambda);
            try expectEqual(@as(f32, 0.0), kv.getValue().friction_lambda.f32s[0]);
            try expect(it.next() == null);
        }
    }
}

test "ContactConstraintManager: contacts over several frames (cache hits, lambdas, removed and re-found contacts)" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();
    h.system.setGravity(Vec3.init(0, -9.81, 0));

    const top = try h.createBox(.dynamic, RVec3.init(0, 0.99, 0), .{ .linear_velocity = Vec3.init(0, -1, 0) });
    const bottom = try h.createBox(.static, RVec3.init(0, -1, 0), .{});
    const other = try h.createBox(.dynamic, RVec3.init(5, 0.99, 0), .{ .linear_velocity = Vec3.init(0, -1, 0) });

    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);

    // Frame 1: two manifolds between top and bottom (different sub shapes), one between other and bottom
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
    {
        const ms = [_]ContactManifold{
            makeManifold(top.getCenterOfMassPosition(), RVec3.init(0, 0, 0), 4, 0.01, 1, 0),
            makeManifold(top.getCenterOfMassPosition(), RVec3.init(0.2, 0, 0), 2, 0.01, 2, 0),
        };
        h.processPair(&h.contact_allocator, top, bottom, &ms, true);
        const m = makeManifold(other.getCenterOfMassPosition(), RVec3.init(5, 0, 0), 3, 0.01, 0, 0);
        h.processPair(&h.contact_allocator, bottom, other, (&m)[0..1], true);
    }
    try expectEqual(@as(usize, 3), h.listener.count(.added));
    try expectEqual(@as(u32, 3), h.manager.getNumConstraints());
    try h.solve(10, 2);
    h.endStep();
    try h.saveContacts(&bytes);
    const lambdas_frame1 = blk: {
        const read_cache = &h.manager.cache[h.manager.cache_write_idx ^ 1];
        const key = SubShapeIDPair.init(top.getID(), subShape(1), bottom.getID(), subShape(0));
        const kv = read_cache.findManifold(key, key.getHash()).?;
        break :blk kv.getValueConst().getContactPointsConst()[0].non_penetration_lambda;
    };
    try expect(lambdas_frame1 > 0.0);

    // Frame 2: nothing moved, the body pair cache is used: persisted callbacks with an estimated manifold, constraints
    // with the cached lambdas, no narrow phase needed (the manifolds passed are ignored)
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    h.processPair(&h.contact_allocator, top, bottom, &.{}, true);
    h.processPair(&h.contact_allocator, other, bottom, &.{}, true);
    try expectEqual(@as(usize, 3), h.listener.count(.persisted));
    try expectEqual(@as(u32, 3), h.manager.getNumConstraints());
    for (h.listener.slice()) |r| {
        try expect(r.normal.isClose(Vec3.init(0, -1, 0), .{}));
        try expect(@abs(r.penetration_depth - 0.01) < 1.0e-5);
    }
    {
        // The constraint of the first manifold starts with the cached lambda
        var found = false;
        for (0..h.manager.getNumConstraints()) |i| {
            const c = CCM.ContactConstraint(.dynamic, .static).fromBase(h.constraint(@intCast(i)));
            if (c.base.body1 == top and c.base.num_contact_points == 4) {
                found = true;
                try expectEqual(lambdas_frame1, c.getContactPoints()[0].non_penetration_constraint.getTotalLambda());
            }
        }
        try expect(found);
    }
    // Body 1 reached the cache before, the bodies stay linked to the island of the first dynamic body
    try h.solve(10, 2);
    h.endStep();
    try expect(h.manager.wereBodiesInContact(top.getID(), bottom.getID()));
    try expectEqual(@as(usize, 0), h.listener.count(.removed));

    // Frame 3: top moved (no cache hit), only the first manifold is found again (persisted, lambdas preserved for the
    // points that are close), the second one is removed; other is not processed at all (removed)
    top.addPositionStep(Vec3.init(0.0, 0.0, 0.002));
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    {
        const m = makeManifold(top.getCenterOfMassPosition(), RVec3.init(0.0, 0, 0.002), 4, 0.01, 1, 0);
        h.processPair(&h.contact_allocator, top, bottom, (&m)[0..1], true);
    }
    try expectEqual(@as(usize, 1), h.listener.count(.persisted));
    try expectEqual(@as(u32, 1), h.manager.getNumConstraints());
    {
        const c = CCM.ContactConstraint(.dynamic, .static).fromBase(h.constraint(0));
        try expect(c.getContactPoints()[0].non_penetration_constraint.getTotalLambda() > 0.0); // Close to last frame's point
    }
    try h.solve(10, 2);
    h.endStep();
    h.listener.sort();
    try expectEqual(@as(usize, 2), h.listener.count(.removed));
    try expect(!h.manager.wereBodiesInContact(other.getID(), bottom.getID()));
    try expect(h.manager.wereBodiesInContact(top.getID(), bottom.getID()));

    // Frame 4: a pair without manifolds (the narrow phase found nothing) removes the last contact, the pair is cached
    // without contacts
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    top.addPositionStep(Vec3.init(0.0, 0.5, 0.0));
    h.processPair(&h.contact_allocator, top, bottom, &.{}, true);
    try h.solve(10, 2);
    h.endStep();
    try expectEqual(@as(usize, 1), h.listener.count(.removed));
    try expect(!h.manager.wereBodiesInContact(top.getID(), bottom.getID()));
    try h.saveContacts(&bytes);
    try expectEqual(@as(usize, 4 + 8 + 24 + 4 + 4), bytes.items.len); // 1 body pair without manifolds, no CCD manifold

    // Frame 5: the pair is still cached but has no contacts: the cache "handles" it, nothing is reported
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    try expect(h.manager.getContactsFromCache(&h.contact_allocator, top, bottom));
    try h.solve(10, 2);
    h.endStep();
    try expectEqual(@as(usize, 0), h.listener.num_records);

    // Frame 6: the contact comes back (the cache is invalid since the bodies moved): added again
    top.addPositionStep(Vec3.init(0.0, -0.5, 0.0));
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    {
        const m = makeManifold(top.getCenterOfMassPosition(), RVec3.init(0.0, 0, 0.002), 4, 0.01, 1, 0);
        h.processPair(&h.contact_allocator, top, bottom, (&m)[0..1], true);
    }
    try h.solve(10, 2);
    h.endStep();
    try expectEqual(@as(usize, 1), h.listener.count(.added));
    try expect(h.temp.isEmpty());
}

fn subShape(value: u32) SubShapeID {
    var id: SubShapeID = .{};
    id.setValue(value);
    return id;
}

test "ContactConstraintManager: contact constraints full and cache full soft errors" {
    const allocator = std.testing.allocator;

    // 4 constraints at most: the 5th and 6th pair get no constraint, their manifold is not linked to the body pair
    {
        var h: Harness = undefined;
        try h.init(allocator, .{ .max_contact_constraints = 4 });
        defer h.deinit();
        const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
        var boxes: [6]*Body = undefined;
        for (&boxes, 0..) |*b, i|
            b.* = try h.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{ .linear_velocity = Vec3.init(0, -1, 0) });
        try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
        for (boxes) |b| {
            const m = makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), 2, 0.01, 0, 0);
            h.processPair(&h.contact_allocator, b, floor, (&m)[0..1], true);
        }
        try expectEqual(@as(u32, 4), h.manager.getNumConstraints());
        try expect(h.contact_allocator.errors.contact_constraints_full);
        try expect(!h.contact_allocator.errors.manifold_cache_full);
        try expectEqual(@as(u32, 6), h.contact_allocator.num_manifolds);
        try expectEqual(@as(u64, 6), h.manager.num_constraints_and_next_constraint_offset.load(.monotonic) & 0xffffffff);
        try h.solve(10, 2);
        h.endStep();
        for (boxes, 0..) |b, i|
            try expectEqual(i < 4, h.manager.wereBodiesInContact(b.getID(), floor.getID()));
        try expect(h.temp.isEmpty());
    }

    // Tiny caches: 4 body pairs and 4 constraints share 816 bytes
    {
        var h: Harness = undefined;
        try h.init(allocator, .{ .max_contact_constraints = 4, .max_body_pairs = 4 });
        defer h.deinit();
        const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
        try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
        var num_handles: u32 = 0;
        for (0..8) |i| {
            const b = try h.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{});
            const handle = h.manager.addBodyPair(&h.contact_allocator, b, floor);
            if (handle != null) {
                num_handles += 1;
                var link = true;
                const m = makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), 4, 0.01, 0, 0);
                h.manager.addContactConstraint(&h.contact_allocator, &link, handle, b, floor, &m);
            }
        }
        try expect(h.contact_allocator.errors.manifold_cache_full or h.contact_allocator.errors.body_pair_cache_full);
        try expect(num_handles < 8);
        try expectEqual(num_handles, h.contact_allocator.num_body_pairs);
        try h.solve(10, 2);
        h.endStep();
        try expect(h.temp.isEmpty());

        // Restoring a state that does not fit fails
        var big: Harness = undefined;
        try big.init(allocator, .{});
        defer big.deinit();
        const big_floor = try big.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
        try big.beginStep(big.temp.tempAllocator(), 1.0 / 60.0, 0.0);
        for (0..8) |i| {
            const b = try big.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{});
            const m = makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), 4, 0.01, 0, 0);
            big.processPair(&big.contact_allocator, b, big_floor, (&m)[0..1], true);
        }
        try big.solve(10, 2);
        big.endStep();
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        try big.manager.saveState(recorder.recorder(), null);
        try expect(!try h.manager.restoreState(recorder.recorder(), null));
    }
}

test "ContactConstraintManager: CCD contacts" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();

    const a = try h.createBox(.dynamic, RVec3.init(0, 3, 0), .{ .friction = 0.4, .restitution = 0.2 });
    const b = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .friction = 0.9, .restitution = 0.6 });
    const Modifier = struct {
        fn modify(body1: *const Body, _: *const Body, _: *const ContactManifold, settings: *ContactSettings) void {
            // Body 1 has the lower ID: a
            if (body1.isDynamic())
                settings.inv_mass_scale1 = 0.25;
        }
    };
    h.listener.modify = Modifier.modify;

    // Frame 1: the CCD contact is new, a second report in the same update is persisted
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
    {
        const m = makeManifold(a.getCenterOfMassPosition(), RVec3.init(0, 0, 0), 1, 0.0, 7, 0).swapShapes(); // In the order (b, a)
        var settings: ContactSettings = .{};
        h.manager.onCCDContactAdded(&h.contact_allocator, b, a, &m, &settings); // b has the higher ID, the bodies are swapped
        try expectEqual(@sqrt(@as(f32, 0.4) * 0.9), settings.combined_friction);
        try expectEqual(@as(f32, 0.6), settings.combined_restitution);
        try expect(!settings.is_sensor);
        try expectEqual(@as(f32, 0.25), settings.inv_mass_scale2); // Swapped back
        try expectEqual(@as(f32, 1.0), settings.inv_mass_scale1);
        var settings2: ContactSettings = .{};
        h.manager.onCCDContactAdded(&h.contact_allocator, b, a, &m, &settings2);
    }
    try expectEqual(@as(usize, 1), h.listener.count(.added));
    try expectEqual(@as(usize, 1), h.listener.count(.persisted));
    try expectEqual(@as(u32, 1), h.contact_allocator.num_manifolds);
    try expectEqual(@as(u32, 0), h.contact_allocator.num_body_pairs);
    try h.solve(10, 2);
    h.endStep();

    // The CCD manifold is in the state (its key only) but is not a contact between the bodies
    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try h.saveContacts(&bytes);
    try expectEqual(@as(usize, 4 + 4 + 16), bytes.items.len);
    try expect(!h.manager.wereBodiesInContact(a.getID(), b.getID()));

    // Frame 2: reported again: persisted from the read cache; frame 3: not reported: removed
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    {
        const m = makeManifold(a.getCenterOfMassPosition(), RVec3.init(0, 0, 0), 1, 0.0, 7, 0);
        var settings: ContactSettings = .{};
        h.manager.onCCDContactAdded(&h.contact_allocator, a, b, &m, &settings);
        try expectEqual(@as(f32, 0.25), settings.inv_mass_scale1); // Not swapped
    }
    try h.solve(10, 2);
    h.endStep();
    try expectEqual(@as(usize, 1), h.listener.count(.persisted));
    try expectEqual(@as(usize, 0), h.listener.count(.removed));
    h.listener.reset();
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    try h.solve(10, 2);
    h.endStep();
    try expectEqual(@as(usize, 1), h.listener.count(.removed));
    try expectEqual(@as(u32, 7), h.listener.records[0].sub_shape_id1);

    // Without a listener nothing is cached
    h.manager.setContactListener(null);
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    {
        const m = makeManifold(a.getCenterOfMassPosition(), RVec3.init(0, 0, 0), 1, 0.0, 7, 0);
        var settings: ContactSettings = .{};
        h.manager.onCCDContactAdded(&h.contact_allocator, a, b, &m, &settings);
        try expectEqual(@as(f32, 1.0), settings.inv_mass_scale1);
    }
    try expectEqual(@as(u32, 0), h.contact_allocator.num_manifolds);
    try h.solve(10, 2);
    h.endStep();
}

test "ContactConstraintManager: SortContacts orders on the sort key and the body IDs" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();

    const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
    var boxes: [8]*Body = undefined;
    for (&boxes, 0..) |*b, i|
        b.* = try h.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{ .activate = i % 2 == 0 });
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 0.0);
    var i: usize = boxes.len;
    while (i > 0) {
        i -= 1;
        const b = boxes[i];
        const m = makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), 1, 0.01, @intCast(i), 0);
        h.processPair(&h.contact_allocator, floor, b, (&m)[0..1], true);
    }

    // The sleeping bodies were woken up
    for (boxes) |b|
        try expect(b.isActive());

    var offsets: [8]u32 = undefined;
    for (&offsets, 0..) |*o, j|
        o.* = @intCast(j);
    h.manager.constraintIdxToConstraintOffset(&offsets);
    for (offsets, 0..) |o, j|
        try expectEqual(h.manager.constraint_idx_to_offset.items[j], o);
    h.manager.sortContacts(&offsets);
    for (offsets[0 .. offsets.len - 1], offsets[1..]) |o1, o2| {
        const c1: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(h.manager.constraints.items.ptr + o1));
        const c2: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(h.manager.constraints.items.ptr + o2));
        try expect(c1.sort_key < c2.sort_key);
    }

    // Equal sort keys: the IDs of body 1, then of body 2 decide
    const c0 = h.constraint(0);
    const c1 = h.constraint(1);
    const saved = .{ c0.sort_key, c1.sort_key };
    c0.sort_key = 5;
    c1.sort_key = 5;
    var two = [2]u32{ h.manager.constraint_idx_to_offset.items[0], h.manager.constraint_idx_to_offset.items[1] };
    h.manager.sortContacts(&two);
    const first: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(h.manager.constraints.items.ptr + two[0]));
    const second: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(h.manager.constraints.items.ptr + two[1]));
    try expect(first.body1.getID().lessThan(second.body1.getID()) or (first.body1 == second.body1 and first.body2.getID().lessThan(second.body2.getID())));
    c0.sort_key = saved[0];
    c1.sort_key = saved[1];

    // Warm start with the dummy steps calculator
    const dummy: DummyCalculateSolverSteps = .{};
    h.manager.warmStartVelocityConstraints(&offsets, 1.0, &dummy);
    h.manager.storeAppliedImpulses(&offsets);
    try h.solve(10, 2);
    h.endStep();
    try expect(h.temp.isEmpty());
}

/// A filter that only saves / restores the contacts of one body
const BodyFilter = struct {
    pub const overrides = .{ .shouldSaveContact, .shouldRestoreContact };

    base: StateRecorderFilter = .init(BodyFilter),
    body: BodyID,
    restore: bool = true,

    pub fn shouldSaveContact(self: *const BodyFilter, body1: BodyID, body2: BodyID) bool {
        return body1.eql(self.body) or body2.eql(self.body);
    }

    pub fn shouldRestoreContact(self: *const BodyFilter, body1: BodyID, body2: BodyID) bool {
        return self.restore and (body1.eql(self.body) or body2.eql(self.body));
    }
};

/// Fill the contact cache of a harness with a frame of contacts between `boxes` and `floor` and a CCD contact
fn contactFrame(h: *Harness, floor: *Body, boxes: []const *Body) !void {
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    for (boxes, 0..) |b, i| {
        const ms = [_]ContactManifold{
            makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), @intCast(1 + i % 4), 0.01, 0, 0),
            makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0.1, -1, 0)), 2, 0.02, 1, 0),
        };
        h.processPair(&h.contact_allocator, b, floor, ms[0 .. 1 + i % 2], false);
    }
    const m = makeManifold(boxes[0].getCenterOfMassPosition(), RVec3.init(0, 0, 0), 1, 0.0, 9, 0);
    var settings: ContactSettings = .{};
    h.manager.onCCDContactAdded(&h.contact_allocator, boxes[0], floor, &m, &settings);
    try h.solve(10, 2);
    h.endStep();
}

test "ContactConstraintManager: SaveState / RestoreState round trips, validation, filters and parts" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();
    const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
    var boxes: [5]*Body = undefined;
    for (&boxes, 0..) |*b, i|
        b.* = try h.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{ .linear_velocity = Vec3.init(0, -1, 0) });
    try contactFrame(&h, floor, &boxes);

    var original: std.ArrayList(u8) = .empty;
    defer original.deinit(allocator);
    try h.saveContacts(&original);

    // Restore into a second system with the same bodies: same bytes, same contacts
    var h2: Harness = undefined;
    try h2.init(allocator, .{});
    defer h2.deinit();
    _ = try h2.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
    for (0..boxes.len) |i|
        _ = try h2.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{});
    {
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        recorder.writeBytes(original.items);
        try expect(try h2.manager.restoreState(recorder.recorder(), null));
        try expect(!recorder.isFailed());
        var restored: std.ArrayList(u8) = .empty;
        defer restored.deinit(allocator);
        try h2.saveContacts(&restored);
        try expectEqualSlices(u8, original.items, restored.items);
        for (boxes) |b|
            try expect(h2.manager.wereBodiesInContact(b.getID(), floor.getID()));
    }

    // Validating restore of the current state: no mismatch, same state afterwards
    {
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        recorder.writeBytes(original.items);
        recorder.base.setValidating(true);
        try expect(try h.manager.restoreState(recorder.recorder(), null));
        try expectEqual(@as(u32, 0), recorder.num_mismatches);
        var after: std.ArrayList(u8) = .empty;
        defer after.deinit(allocator);
        try h.saveContacts(&after);
        try expectEqualSlices(u8, original.items, after.items);
    }

    // A save filter keeps the contacts of one body (and its CCD contact)
    {
        const filter: BodyFilter = .{ .body = boxes[0].getID() };
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        try h.manager.saveState(recorder.recorder(), &filter.base);
        try expect(recorder.getData().len < original.items.len);
        var h3: Harness = undefined;
        try h3.init(allocator, .{});
        defer h3.deinit();
        try expect(try h3.manager.restoreState(recorder.recorder(), null));
        try expect(h3.manager.wereBodiesInContact(boxes[0].getID(), floor.getID()));
        try expect(!h3.manager.wereBodiesInContact(boxes[1].getID(), floor.getID()));
    }

    // A restore filter skips the other contacts (the stream is still read to the end)
    {
        const filter: BodyFilter = .{ .body = boxes[2].getID() };
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        recorder.writeBytes(original.items);
        var h3: Harness = undefined;
        try h3.init(allocator, .{});
        defer h3.deinit();
        try expect(try h3.manager.restoreState(recorder.recorder(), &filter.base));
        try expect(!recorder.isEOF());
        var probe: [1]u8 = undefined;
        recorder.readBytes(&probe);
        try expect(recorder.isEOF()); // Everything was consumed
        try expect(h3.manager.wereBodiesInContact(boxes[2].getID(), floor.getID()));
        try expect(!h3.manager.wereBodiesInContact(boxes[0].getID(), floor.getID()));
    }

    // Two parts: the contacts of box 0, then the rest; the cache is finalized after the last part only
    {
        const part1: BodyFilter = .{ .body = boxes[0].getID() };
        var recorder1: StateRecorderImpl = .init(allocator);
        defer recorder1.deinit();
        try h.manager.saveState(recorder1.recorder(), &part1.base);
        const NotBody0 = struct {
            pub const overrides = .{.shouldSaveContact};
            base: StateRecorderFilter = .init(@This()),
            body: BodyID,
            pub fn shouldSaveContact(self: *const @This(), body1: BodyID, body2: BodyID) bool {
                return !(body1.eql(self.body) or body2.eql(self.body));
            }
        };
        const part2: NotBody0 = .{ .body = boxes[0].getID() };
        var recorder2: StateRecorderImpl = .init(allocator);
        defer recorder2.deinit();
        try h.manager.saveState(recorder2.recorder(), &part2.base);

        var h3: Harness = undefined;
        try h3.init(allocator, .{});
        defer h3.deinit();
        recorder1.base.setIsLastPart(false);
        try expect(try h3.manager.restoreState(recorder1.recorder(), null));
        try expect(try h3.manager.restoreState(recorder2.recorder(), null));
        var restored: std.ArrayList(u8) = .empty;
        defer restored.deinit(allocator);
        try h3.saveContacts(&restored);
        try expectEqualSlices(u8, original.items, restored.items);
    }

    // Out of memory while saving (the sorted arrays) and while validating
    {
        const S = struct {
            fn save(a: Allocator, manager: *CCM) !void {
                const old = manager.allocator;
                defer manager.allocator = old;
                manager.allocator = a;
                var recorder: StateRecorderImpl = .init(std.testing.allocator);
                defer recorder.deinit();
                try manager.saveState(recorder.recorder(), null);
            }
        };
        try std.testing.checkAllAllocationFailures(allocator, S.save, .{&h.manager});

        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
        h.manager.allocator = failing.allocator();
        var recorder: StateRecorderImpl = .init(allocator);
        defer recorder.deinit();
        recorder.writeBytes(original.items);
        recorder.base.setValidating(true);
        try std.testing.expectError(error.OutOfMemory, h.manager.restoreState(recorder.recorder(), null));
        h.manager.allocator = allocator;

        // The write cache is empty and usable, the read cache unchanged
        var after: std.ArrayList(u8) = .empty;
        defer after.deinit(allocator);
        try h.saveContacts(&after);
        try expectEqualSlices(u8, original.items, after.items);
        try contactFrame(&h, floor, &boxes);
    }
}

/// A frame with the contact processing on several threads: thread t processes the pairs t, t + n, ... with its own
/// contact allocator, like the FindCollisions jobs
fn threadedFrame(h: *Harness, floor: *Body, boxes: []const *Body, num_threads: usize) !void {
    try h.beginStep(h.temp.tempAllocator(), 1.0 / 60.0, 1.0);
    var allocators: [8]ContactAllocator = undefined;
    const Worker = struct {
        fn run(harness: *Harness, contact_allocator: *ContactAllocator, fl: *Body, bs: []const *Body, first: usize, stride: usize) void {
            var i = first;
            while (i < bs.len) : (i += stride) {
                const b = bs[i];
                const ms = [_]ContactManifold{
                    makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), @intCast(1 + i % 4), 0.01, 0, 0),
                    makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0.3, -1, 0.1)), 2, 0.005, 1, 0),
                };
                // Alternate the body order, the manager swaps
                if (i % 3 == 0)
                    harness.processPair(contact_allocator, fl, b, ms[0 .. 1 + i % 2], true)
                else
                    harness.processPair(contact_allocator, b, fl, ms[0 .. 1 + i % 2], true);
            }
        }
    };
    var threads: [8]std.Thread = undefined;
    for (0..num_threads) |t| {
        allocators[t] = h.manager.getContactAllocator();
        threads[t] = try std.Thread.spawn(.{}, Worker.run, .{ h, &allocators[t], floor, boxes, t, num_threads });
    }
    for (threads[0..num_threads]) |t|
        t.join();

    // sFinalizeContactAllocator
    var num_body_pairs: u32 = 0;
    var num_manifolds: u32 = 0;
    for (allocators[0..num_threads]) |a| {
        num_body_pairs += a.num_body_pairs;
        num_manifolds += a.num_manifolds;
        try expect(a.errors.isNone());
    }
    try h.solve(10, 2);
    h.manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(num_body_pairs, num_manifolds);
    h.finishStep();
}

fn runThreadedScene(allocator: Allocator, num_threads: usize, out_state: *StateRecorderImpl, out_records: *std.ArrayList(Record)) !void {
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();
    h.system.setGravity(Vec3.init(0, -9.81, 0));
    const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(500, 1, 500) });
    var boxes: [48]*Body = undefined;
    for (&boxes, 0..) |*b, i| {
        const x: f32 = @floatFromInt(i % 8);
        const z: f32 = @floatFromInt(i / 8);
        b.* = try h.createBox(.dynamic, RVec3.init(x * 3.0, 0.99, z * 3.0), .{ .linear_velocity = Vec3.init(0.1 * x, -1.0, -0.1 * z), .angular_velocity = Vec3.init(0, 0.2 * z, 0), .friction = 0.1 * x });
    }
    for (0..3) |_| {
        h.listener.reset();
        try threadedFrame(&h, floor, &boxes, num_threads);
        for (h.listener.slice()) |r|
            try out_records.append(allocator, r);
        h.listener.sort();
    }
    try h.saveAll(out_state);
}

test "ContactConstraintManager: AddContactConstraint from several threads gives the single threaded result" {
    const allocator = std.testing.allocator;
    var reference: StateRecorderImpl = .init(allocator);
    defer reference.deinit();
    var reference_records: std.ArrayList(Record) = .empty;
    defer reference_records.deinit(allocator);
    try runThreadedScene(allocator, 1, &reference, &reference_records);
    std.mem.sort(Record, reference_records.items, {}, Record.lessThan);

    for (0..20) |iteration| {
        const num_threads = 2 + iteration % 6;
        var state: StateRecorderImpl = .init(allocator);
        defer state.deinit();
        var records: std.ArrayList(Record) = .empty;
        defer records.deinit(allocator);
        try runThreadedScene(allocator, num_threads, &state, &records);
        try expectEqualSlices(u8, reference.getData(), state.getData());
        std.mem.sort(Record, records.items, {}, Record.lessThan);
        try expectEqual(reference_records.items.len, records.items.len);
        for (reference_records.items, records.items) |a, b| {
            try expectEqual(a.kind, b.kind);
            try expect(a.body1.eql(b.body1) and a.body2.eql(b.body2));
            try expectEqual(a.sub_shape_id1, b.sub_shape_id1);
            try expectEqual(a.num_points, b.num_points);
            try expectEqual(@as(u32, @bitCast(a.penetration_depth)), @as(u32, @bitCast(b.penetration_depth)));
        }
    }
}

/// One frame like PhysicsSystem.update runs it, with D5's error handling: a failure in the build phase (the buffers)
/// changes nothing, a failure inside a "job" (FinalizeIslands) poisons the frame: the solve is skipped and the
/// ContactRemovedCallbacks job calls finalizeContactCacheAfterOutOfMemory
fn oomFrame(h: *Harness, temp_allocator: TempAllocator, floor: *Body, boxes: []const *Body) error{OutOfMemory}!void {
    h.beginStep(temp_allocator, 1.0 / 60.0, 1.0) catch |err| {
        h.finishStep();
        return err;
    };

    // FindCollisions: the first box left the floor (its contact is removed), the others persist from the cache or are
    // found again
    for (boxes[1..], 1..) |b, i| {
        const m = makeManifold(b.getCenterOfMassPosition(), b.getCenterOfMassPosition().add(RVec3.init(0, -1, 0)), @intCast(1 + i % 4), 0.01, 0, 0);
        h.processPair(&h.contact_allocator, b, floor, (&m)[0..1], i % 2 == 0);
    }

    // FinalizeIslands (allocates) and the solver jobs
    var poisoned = false;
    h.solve(10, 2) catch {
        poisoned = true;
    };

    // ContactRemovedCallbacks
    if (poisoned)
        h.manager.finalizeContactCacheAfterOutOfMemory()
    else
        h.manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(h.contact_allocator.num_body_pairs, h.contact_allocator.num_manifolds);

    // finishUpdate
    h.finishStep();
    if (poisoned)
        return error.OutOfMemory;
}

test "ContactConstraintManager: every temp allocation of a frame with contacts fails in turn (finalizeContactCacheAfterOutOfMemory)" {
    const allocator = std.testing.allocator;
    var h: Harness = undefined;
    try h.init(allocator, .{});
    defer h.deinit();
    h.system.setGravity(Vec3.init(0, -9.81, 0));
    const floor = try h.createBox(.static, RVec3.init(0, -1, 0), .{ .half_extent = Vec3.init(50, 1, 50) });
    var boxes: [6]*Body = undefined;
    for (&boxes, 0..) |*b, i|
        b.* = try h.createBox(.dynamic, RVec3.init(@as(f32, @floatFromInt(i)) * 3.0, 0.99, 0), .{ .linear_velocity = Vec3.init(0, -1, 0) });
    try contactFrame(&h, floor, &boxes);

    // The state before the frame and the reference frame
    var snapshot: StateRecorderImpl = .init(allocator);
    defer snapshot.deinit();
    try h.saveAll(&snapshot);
    var snapshot_contacts: std.ArrayList(u8) = .empty;
    defer snapshot_contacts.deinit(allocator);
    try h.saveContacts(&snapshot_contacts);

    var counting: FailingTempAllocator = .init(h.temp.tempAllocator(), .{});
    h.listener.reset();
    try oomFrame(&h, counting.tempAllocator(), floor, &boxes);
    try expect(h.listener.count(.removed) > 0); // The frame removes contacts
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= 4);
    var reference: StateRecorderImpl = .init(allocator);
    defer reference.deinit();
    try h.saveAll(&reference);

    for (0..num_allocations) |fail_index| {
        // Restore the state before the frame
        try h.restoreAll(&snapshot);

        h.listener.reset();
        var failing: FailingTempAllocator = .init(h.temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, oomFrame(&h, failing.tempAllocator(), floor, &boxes));

        // Nothing leaked in the temp allocator, no OnContactRemoved was called
        try expect(h.temp.isEmpty());
        try expectEqual(@as(usize, 0), h.listener.count(.removed));

        // Both caches are structurally valid: the read cache is the one from before the frame (its persisted marks
        // reset), the write cache is empty
        var contacts: std.ArrayList(u8) = .empty;
        defer contacts.deinit(allocator);
        try h.saveContacts(&contacts);
        try expectEqualSlices(u8, snapshot_contacts.items, contacts.items);
        const write_cache = &h.manager.cache[h.manager.cache_write_idx];
        try expectEqual(@as(u32, 0), write_cache.lfhm_allocator.write_offset.load(.monotonic));
        if (Core.enable_asserts) {
            try expectEqual(@as(u32, 0), write_cache.getNumManifolds());
            try expectEqual(@as(u32, 0), write_cache.getNumBodyPairs());
        }
        const read_cache = &h.manager.cache[h.manager.cache_write_idx ^ 1];
        var it = read_cache.cached_manifolds.begin();
        while (it.next()) |kv|
            try expectEqual(@as(u16, 0), kv.getValue().flags.load(.monotonic) & CCM.CachedManifold.Flags.contact_persisted);

        // RestoreState works and the frame gives the reference result
        try h.restoreAll(&snapshot);
        try oomFrame(&h, h.temp.tempAllocator(), floor, &boxes);
        var result: StateRecorderImpl = .init(allocator);
        defer result.deinit();
        try h.saveAll(&result);
        try expectEqualSlices(u8, reference.getData(), result.getData());
    }

    // Continuing without a restore after a poisoned frame works too (the caches are usable)
    try h.restoreAll(&snapshot);
    var failing: FailingTempAllocator = .init(h.temp.tempAllocator(), .{ .fail_index = num_allocations - 1 });
    try std.testing.expectError(error.OutOfMemory, oomFrame(&h, failing.tempAllocator(), floor, &boxes));
    h.listener.reset();
    try oomFrame(&h, h.temp.tempAllocator(), floor, &boxes);
    try expect(h.listener.count(.removed) > 0);
    try expect(h.temp.isEmpty());
}
