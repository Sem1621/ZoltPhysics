//! Zolt addition, no Jolt file: tests of BodyManager, the body locks and the state recorder (Phase 5 F1)
//!
//! Jolt has no unit test file for these classes (its tests go through a PhysicsSystem, Wave C). Adapted from the
//! Phase 5 architecture prototype (SimulationPrototype.zig, removed in F2: "BodyManager ID allocation like Jolt", the
//! lock parts of "body locks, the tagged lock interface and BodyInterface", the body parts of "state recorder save,
//! restore, validation and filter") plus activation, the active body arrays, the listener, multi-threaded locking and
//! out of memory. ZoltParity/Physics/BodyParity.zig compares the IDs, orders and state bytes with the C++ library.

const std = @import("std");
const zolt = @import("zolt");
const BodyTestContextFile = @import("BodyTestContext.zig");
const BodyTestContext = BodyTestContextFile.BodyTestContext;
const Layers = @import("../Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyIDVector = zolt.BodyIDVector;
const BodyLockInterface = zolt.BodyLockInterface;
const BodyLockMultiRead = zolt.BodyLockMultiRead;
const BodyLockMultiWrite = zolt.BodyLockMultiWrite;
const BodyLockRead = zolt.BodyLockRead;
const BodyLockWrite = zolt.BodyLockWrite;
const BodyManager = zolt.BodyManager;
const BoxShape = zolt.BoxShape;
const Core = zolt.Core;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const SphereShape = zolt.SphereShape;
const StateRecorderFilter = zolt.StateRecorderFilter;
const StateRecorderImpl = zolt.StateRecorderImpl;
const Vec3 = zolt.Vec3;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const io = BodyTestContextFile.io;

/// Records the activation callbacks (they run with the active bodies mutex locked)
const ActivationRecorder = struct {
    activated: std.ArrayList(BodyID) = .empty,
    deactivated: std.ArrayList(BodyID) = .empty,
    user_data_sum: u64 = 0,

    fn deinit(self: *ActivationRecorder) void {
        self.activated.deinit(std.testing.allocator);
        self.deactivated.deinit(std.testing.allocator);
    }

    pub fn onBodyActivated(self: *ActivationRecorder, body_id: BodyID, body_user_data: u64) void {
        self.activated.append(std.testing.allocator, body_id) catch @panic("OOM");
        self.user_data_sum += body_user_data;
    }

    pub fn onBodyDeactivated(self: *ActivationRecorder, body_id: BodyID, body_user_data: u64) void {
        self.deactivated.append(std.testing.allocator, body_id) catch @panic("OOM");
        self.user_data_sum += body_user_data;
    }
};

fn activeBodies(bm: *const BodyManager) []const BodyID {
    return bm.getActiveBodiesUnsafe(.rigid_body)[0..bm.getNumActiveBodies(.rigid_body)];
}

test "BodyManager ID allocation like Jolt" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;
    try expectEqual(@as(u32, 4), bm.getMaxBodies());

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();

    // Sequential allocation: index 0..3, sequence number 1
    var ids: [4]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        id.* = (try context.createAndAddBody(&settings, .activate)).?.getID();
        try expectEqual(@as(u32, @intCast(i)), id.getIndex());
        try expectEqual(@as(u8, 1), id.getSequenceNumber());
    }
    try expectEqual(@as(u32, 4), bm.getNumBodies());

    // Out of bodies: AddBody returns false
    const extra = try bm.allocateBody(&settings);
    try expect(!bm.addBody(extra));
    try expect(extra.getID().isInvalid());
    bm.freeBody(extra);

    // A body that already has an ID cannot be added again
    try expect(!bm.addBody(bm.getBodyMut(ids[0])));

    // Freed slots are reused in LIFO order of the free list, with the next sequence number
    context.removeAndDestroyBody(ids[1]);
    context.removeAndDestroyBody(ids[3]);
    try expectEqual(@as(u32, 2), bm.getNumBodies());
    try expect(bm.tryGetBody(ids[1]) == null);
    const reused1 = (try context.createAndAddBody(&settings, .activate)).?.getID();
    try expectEqual(@as(u32, 3), reused1.getIndex());
    try expectEqual(@as(u8, 2), reused1.getSequenceNumber());
    const reused2 = (try context.createAndAddBody(&settings, .activate)).?.getID();
    try expectEqual(@as(u32, 1), reused2.getIndex());
    try expectEqual(@as(u8, 2), reused2.getSequenceNumber());

    // A stale ID no longer locks a body
    const lock_interface = BodyLockInterface.initLocking(bm);
    {
        var lock: BodyLockRead = .init(&lock_interface, ids[1]);
        defer lock.deinit();
        try expect(!lock.succeeded());
    }

    // AddBodyWithCustomID removes the slot from the free list and establishes the sequence number
    context.removeAndDestroyBody(reused2);
    const custom = (try context.createAndAddBodyWithID(.fromIndexAndSequenceNumber(1, 7), &settings, .activate)).?;
    try expectEqual(@as(u8, 7), custom.getID().getSequenceNumber());
    try expect((try context.createAndAddBodyWithID(.fromIndexAndSequenceNumber(1, 8), &settings, .activate)) == null); // Slot in use
    try expect((try context.createAndAddBodyWithID(.fromIndexAndSequenceNumber(4, 1), &settings, .activate)) == null); // Beyond the max bodies
    context.removeAndDestroyBody(custom.getID());
    const after_custom = (try context.createAndAddBody(&settings, .activate)).?.getID();
    try expectEqual(@as(u32, 1), after_custom.getIndex());
    try expectEqual(@as(u8, 8), after_custom.getSequenceNumber());
}

test "BodyManager custom IDs beyond the end of the body array build the free list" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 16, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings.deinit();

    // ID 5 pushes 0..4 onto the free list (4 is on top), ID 2 unlinks an entry in the middle of the list
    _ = (try context.createAndAddBodyWithID(.fromIndexAndSequenceNumber(5, 3), &settings, .dont_activate)).?;
    try expectEqual(@as(usize, 6), bm.getBodies().len);
    _ = (try context.createAndAddBodyWithID(.fromIndexAndSequenceNumber(2, 9), &settings, .dont_activate)).?;
    var order: [4]u32 = undefined;
    for (&order) |*index| index.* = (try context.createAndAddBody(&settings, .dont_activate)).?.getID().getIndex();
    try expectEqual([4]u32{ 4, 3, 1, 0 }, order);
    try expectEqual(@as(u32, 6), (try context.createAndAddBody(&settings, .dont_activate)).?.getID().getIndex());

    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try bm.getBodyIDs(allocator, &ids);
    try expectEqual(@as(usize, 7), ids.items.len);
    for (ids.items, 0..) |id, i| try expectEqual(@as(u32, @intCast(i)), id.getIndex());
    try expectEqual(@as(u8, 9), ids.items[2].getSequenceNumber());

    const stats = bm.getBodyStats();
    try expectEqual(@as(u32, 7), stats.num_bodies);
    try expectEqual(@as(u32, 16), stats.max_bodies);
    try expectEqual(@as(u32, 7), stats.num_bodies_static);
    try expectEqual(@as(u32, 0), stats.num_bodies_dynamic);
}

test "BodyManager activation, the active body arrays and the BodyActivationListener" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 16, 4);
    defer context.deinit();
    const bm = &context.body_manager;

    var recorder: ActivationRecorder = .{};
    defer recorder.deinit();
    bm.setBodyActivationListener(.init(&recorder));
    try expect(bm.getBodyActivationListener() != null);

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();

    var ids: [5]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        settings.user_data = i + 1;
        settings.motion_quality = if (i % 2 == 0) .linear_cast else .discrete;
        settings.linear_velocity = Vec3.init(1, 0, 0);
        id.* = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();
    }
    settings.motion_type = .static;
    settings.motion_quality = .discrete;
    settings.linear_velocity = Vec3.zero();
    const static_id = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();
    try expectEqual(@as(u32, 0), bm.getNumActiveBodies(.rigid_body));

    // Activate in a given order (invalid IDs and static bodies are skipped, active bodies are not added twice)
    bm.activateBodies(&.{ ids[3], BodyID.invalid, ids[1], static_id, ids[4], ids[0], ids[3] });
    try std.testing.expectEqualSlices(BodyID, &.{ ids[3], ids[1], ids[4], ids[0] }, activeBodies(bm));
    for (activeBodies(bm), 0..) |id, i| try expectEqual(@as(u32, @intCast(i)), bm.getBody(id).getIndexInActiveBodiesInternal());
    try expectEqual(@as(u32, 2), bm.getNumActiveCCDBodies()); // ids 4 and 0
    try std.testing.expectEqualSlices(BodyID, &.{ ids[3], ids[1], ids[4], ids[0] }, recorder.activated.items);
    try expectEqual(@as(u64, 4 + 2 + 5 + 1), recorder.user_data_sum);
    try expect(!bm.getBody(static_id).isActive());
    try expectEqual(@as(u32, 0), bm.getNumActiveBodies(.soft_body));

    // Deactivation swaps the last body into the hole, resets the velocity and the island index
    bm.getBodyMut(ids[1]).getMotionPropertiesMut().setIslandIndexInternal(3);
    bm.deactivateBodies(&.{ids[1]});
    try std.testing.expectEqualSlices(BodyID, &.{ ids[3], ids[0], ids[4] }, activeBodies(bm));
    try expectEqual(@as(u32, 1), bm.getBody(ids[0]).getIndexInActiveBodiesInternal());
    try expect(bm.getBody(ids[1]).getLinearVelocity().eql(Vec3.zero()));
    try expectEqual(Body.inactive_index, bm.getBody(ids[1]).getMotionProperties().getIslandIndexInternal());
    try expectEqual(Body.inactive_index, bm.getBody(ids[1]).getIndexInActiveBodiesInternal());
    try std.testing.expectEqualSlices(BodyID, &.{ids[1]}, recorder.deactivated.items);

    // Deactivating an inactive body does nothing, deactivating the last body just shrinks the array
    bm.deactivateBodies(&.{ ids[1], ids[4] });
    try std.testing.expectEqualSlices(BodyID, &.{ ids[3], ids[0] }, activeBodies(bm));
    try expectEqual(@as(u32, 1), bm.getNumActiveCCDBodies());

    // GetActiveBodies copies the list
    var copy: BodyIDVector = .empty;
    defer copy.deinit(allocator);
    try bm.getActiveBodies(allocator, .rigid_body, &copy);
    try std.testing.expectEqualSlices(BodyID, activeBodies(bm), copy.items);
    try bm.getActiveBodies(allocator, .soft_body, &copy);
    try expectEqual(@as(usize, 0), copy.items.len);

    // SetMotionQuality updates the CCD counter of active bodies only
    bm.setMotionQuality(bm.getBodyMut(ids[3]), .linear_cast);
    try expectEqual(@as(u32, 2), bm.getNumActiveCCDBodies());
    bm.setMotionQuality(bm.getBodyMut(ids[0]), .discrete);
    try expectEqual(@as(u32, 1), bm.getNumActiveCCDBodies());
    bm.setMotionQuality(bm.getBodyMut(ids[1]), .discrete); // Inactive
    try expectEqual(@as(u32, 1), bm.getNumActiveCCDBodies());
    bm.setMotionQuality(bm.getBodyMut(static_id), .linear_cast); // No motion properties
    try expectEqual(@as(u32, 1), bm.getNumActiveCCDBodies());

    // The body stats count the active bodies per motion type
    bm.getBodyMut(ids[2]).setMotionType(.kinematic);
    bm.activateBodies(&.{ids[2]});
    const stats = bm.getBodyStats();
    try expectEqual(@as(u32, 6), stats.num_bodies);
    try expectEqual(@as(u32, 1), stats.num_bodies_static);
    try expectEqual(@as(u32, 4), stats.num_bodies_dynamic);
    try expectEqual(@as(u32, 2), stats.num_active_bodies_dynamic);
    try expectEqual(@as(u32, 1), stats.num_bodies_kinematic);
    try expectEqual(@as(u32, 1), stats.num_active_bodies_kinematic);

    // The debug locks of the active body list with the per thread grants (only checked with asserts)
    bm.setActiveBodiesLocked(true);
    {
        var grant: BodyManager.GrantActiveBodiesAccess = .init(true, true);
        defer grant.deinit();
        bm.deactivateBodies(&.{ids[2]});
        bm.activateBodies(&.{ids[2]});
    }
    bm.setActiveBodiesLocked(false);

    // Removing the listener stops the callbacks
    bm.setBodyActivationListener(null);
    bm.deactivateBodies(&.{ids[2]});
    try expectEqual(@as(usize, 3), recorder.deactivated.items.len);

    bm.validateActiveBodyBounds();
}

test "BodyManager contact cache invalidation" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const a = (try context.createAndAddBody(&settings, .activate)).?;
    const b = (try context.createAndAddBody(&settings, .activate)).?;

    try bm.invalidateContactCacheForBody(a);
    try bm.invalidateContactCacheForBody(a);
    try bm.invalidateContactCacheForBody(b);
    try expectEqual(@as(usize, 2), bm.bodies_cache_invalid.items.len);
    try expect(a.isCollisionCacheInvalid() and b.isCollisionCacheInvalid());

    // A body that was destroyed in between is skipped
    const b_id = b.getID();
    context.removeAndDestroyBody(b_id);
    bm.validateContactCacheForAllBodies();
    try expect(!a.isCollisionCacheInvalid());
    try expectEqual(@as(usize, 0), bm.bodies_cache_invalid.items.len);
}

test "BodyManager saveState / restoreState of bodies, validation and filter" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 16, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    const box = try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(1, 2, 3), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var ids: [4]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        settings.position = RVec3.init(@floatFromInt(i), 2, 3);
        settings.linear_velocity = Vec3.init(1, @floatFromInt(i), 0);
        id.* = (try context.createAndAddBody(&settings, if (i < 3) .activate else .dont_activate)).?.getID();
    }
    settings.motion_type = .static;
    _ = (try context.createAndAddBody(&settings, .dont_activate)).?;
    // A body that is not in the "broad phase" is not saved
    const not_added = try bm.allocateBody(&settings);
    try expect(bm.addBody(not_added));

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try bm.saveState(snapshot.recorder(), null);
    try expect(!snapshot.recorder().isFailed());

    // Size: count + per body (ID, active flag, position, rotation, motion properties of the dynamic bodies)
    const real_size: usize = if (Core.double_precision) 24 else 12;
    const motion_size: usize = 4 * 12 + (if (Core.double_precision) 24 else 0) + 3 * 16 + 4 + 1;
    try expectEqual(4 + 5 * (4 + 1 + real_size + 16) + 4 * motion_size, snapshot.getData().len);

    // Change the state: move, change velocities and the activation
    bm.getBodyMut(ids[0]).addPositionStep(Vec3.init(5, 5, 5));
    bm.getBodyMut(ids[1]).setLinearVelocity(Vec3.init(9, 9, 9));
    bm.deactivateBodies(&.{ids[2]});
    bm.activateBodies(&.{ids[3]});
    var changed = StateRecorderImpl.init(allocator);
    defer changed.deinit();
    try bm.saveState(changed.recorder(), null);
    try expect(!changed.isEqual(&snapshot));

    // Restore: the state bytes are the same again, the activation is restored
    snapshot.rewind();
    try expect(try bm.restoreState(snapshot.recorder()));
    var restored = StateRecorderImpl.init(allocator);
    defer restored.deinit();
    try bm.saveState(restored.recorder(), null);
    try expect(restored.isEqual(&snapshot));
    try expect(bm.getBody(ids[2]).isActive());
    try expect(!bm.getBody(ids[3]).isActive());
    try checkBounds(bm.getBody(ids[0]));

    // Validating restore: reading the state we just saved finds no mismatch
    restored.rewind();
    restored.recorder().setValidating(true);
    try expect(try bm.restoreState(restored.recorder()));
    try expectEqual(@as(u32, 0), restored.num_mismatches);

    // A filter that skips the static bodies
    const SkipStatic = struct {
        pub const overrides = .{.shouldSaveBody};
        base: StateRecorderFilter = .init(@This()),
        pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
            _ = self;
            return !body.isStatic();
        }
    };
    const filter: SkipStatic = .{};
    var partial = StateRecorderImpl.init(allocator);
    defer partial.deinit();
    try bm.saveState(partial.recorder(), &filter.base);
    try expectEqual(snapshot.getData().len - (4 + 1 + real_size + 16), partial.getData().len);
    partial.rewind();
    try expect(try bm.restoreState(partial.recorder()));

    // SaveBodyState / RestoreBodyState of a single body
    var single = StateRecorderImpl.init(allocator);
    defer single.deinit();
    bm.saveBodyState(bm.getBody(ids[2]), single.recorder());
    bm.deactivateBodies(&.{ids[2]});
    single.rewind();
    bm.restoreBodyState(bm.getBodyMut(ids[2]), single.recorder());
    try expect(bm.getBody(ids[2]).isActive());

    if (!Core.enable_asserts) {
        // Restoring a state that has a body that no longer exists fails (Jolt asserts in debug builds)
        context.removeAndDestroyBody(ids[1]);
        snapshot.rewind();
        try expect(!try bm.restoreState(snapshot.recorder()));

        // A validating restore with a different number of bodies fails
        snapshot.rewind();
        snapshot.recorder().setValidating(true);
        try expect(!try bm.restoreState(snapshot.recorder()));
    }

    // Take the body that is not in the broad phase out again and free it
    bm.removeBodies(&.{not_added.getID()}, null);
    bm.freeBody(not_added);
}

fn checkBounds(body: *const Body) !void {
    try expect(body.getWorldSpaceBounds().eql(body.getShape().getWorldSpaceBoundsRMat44(body.getCenterOfMassTransform(), Vec3.one())));
}

test "BodyManager removeBodies returns the bodies" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const a = try bm.allocateBody(&settings);
    const b = try bm.allocateBody(&settings);
    try expect(bm.addBody(a) and bm.addBody(b));
    const ids = [_]BodyID{ b.getID(), a.getID() };
    var bodies: [2]*Body = undefined;
    bm.removeBodies(&ids, &bodies);
    try expect(bodies[0] == b and bodies[1] == a);
    try expect(a.getID().isInvalid() and b.getID().isInvalid());
    try expectEqual(@as(u32, 0), bm.getNumBodies());
    bm.removeBodies(&.{}, null); // Nothing to do
    bm.destroyBodies(&.{});

    // The bodies can be added again (AddBody after RemoveBodies, like BodyInterface::CreateBodyWithID)
    try expect(bm.addBody(a));
    try expectEqual(@as(u32, 0), a.getID().getIndex());
    try expectEqual(@as(u8, 2), a.getID().getSequenceNumber());
    bm.freeBody(b);
}

test "body locks and the tagged lock interface" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 64, 8);
    defer context.deinit();
    const bm = &context.body_manager;
    const locking = BodyLockInterface.initLocking(bm);
    const no_lock = BodyLockInterface.initNoLock(bm);

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(1, 2, 3), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const id = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();

    {
        var lock: BodyLockWrite = .init(&locking, id);
        defer lock.deinit();
        try expect(lock.succeeded());
        try expect(lock.succeededAndIsInBroadPhase());
        lock.getBody().setUserData(42);
        // The body's mutex is held exclusively
        try expect(bm.getMutexForBody(id).isLocked());
    }
    try expect(!bm.getMutexForBody(id).isLocked());
    {
        var lock: BodyLockRead = .init(&no_lock, id);
        defer lock.deinit();
        try expectEqual(@as(u64, 42), lock.getBody().getUserData());
        try expect(!bm.getMutexForBody(id).isLocked());
    }
    {
        // Read locks share the mutex, releaseLock unlocks early
        var lock1: BodyLockRead = .init(&locking, id);
        defer lock1.deinit();
        var lock2: BodyLockRead = .init(&no_lock, id);
        defer lock2.deinit();
        try expect(lock1.succeeded() and lock2.succeeded());
        lock1.releaseLock();
        try expect(!lock1.succeeded());
        try expect(!bm.getMutexForBody(id).isLocked());
    }

    // Invalid IDs never lock
    var invalid_lock: BodyLockRead = .init(&locking, .invalid);
    try expect(!invalid_lock.succeeded());
    invalid_lock.deinit();

    // A body that is not in the broad phase
    bm.getBodyMut(id).setInBroadPhaseInternal(false);
    {
        var lock: BodyLockRead = .init(&locking, id);
        defer lock.deinit();
        try expect(lock.succeeded() and !lock.succeededAndIsInBroadPhase());
    }
    bm.getBodyMut(id).setInBroadPhaseInternal(true);

    // Multi body locks: the mask has a bit per mutex, too many bodies lock everything
    var ids: [12]BodyID = undefined;
    ids[0] = id;
    for (ids[1..]) |*i| i.* = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();
    try expectEqual(@as(BodyManager.MutexMask, 0xff), locking.getAllBodiesMutexMask());
    try expectEqual(@as(BodyManager.MutexMask, 0xff), locking.getMutexMask(&ids));
    try expectEqual(@as(BodyManager.MutexMask, 0), no_lock.getMutexMask(&ids));
    const mask = locking.getMutexMask(ids[0..3]);
    var expected_mask: BodyManager.MutexMask = 0;
    for (ids[0..3]) |i| expected_mask |= @as(BodyManager.MutexMask, 1) << @intCast(bm.body_mutexes.getMutexIndex(i.getIndex()));
    try expectEqual(expected_mask, mask);
    try expectEqual(mask, locking.getMutexMask(&.{ ids[0], .invalid, ids[1], ids[2] }));
    {
        const lock_ids = [_]BodyID{ ids[0], .invalid, ids[2] };
        var lock: BodyLockMultiWrite = .init(&locking, &lock_ids);
        defer lock.deinit();
        try expectEqual(@as(u32, 3), lock.getNumBodies());
        try expect(lock.getBody(0).? == bm.getBodyMut(ids[0]));
        try expect(lock.getBody(1) == null);
        lock.getBody(2).?.setUserData(7);
        try expect(bm.getMutexForBody(ids[0]).isLocked());
        lock.releaseLocks();
        try expect(!bm.getMutexForBody(ids[0]).isLocked());
        try expectEqual(@as(u32, 0), lock.getNumBodies());
    }
    {
        var lock: BodyLockMultiRead = .init(&no_lock, &ids);
        defer lock.deinit();
        try expectEqual(@as(u64, 7), lock.getBody(2).?.getUserData());
    }

    // LockAllBodies takes every body mutex and the bodies list
    bm.lockAllBodies();
    for (ids) |i| try expect(bm.getMutexForBody(i).isLocked());
    {
        // The no-lock interface can be used while all bodies are locked
        var lock: BodyLockWrite = .init(&no_lock, ids[3]);
        defer lock.deinit();
        lock.getBody().setUserData(3);
    }
    bm.unlockAllBodies();
    try expect(!bm.getMutexForBody(ids[3]).isLocked());
    try expectEqual(@as(u64, 3), bm.getBody(ids[3]).getUserData());

    // The batch functions of the manager
    bm.lockRead(0b101);
    bm.unlockRead(0b101);
    bm.lockWrite(0b11);
    bm.unlockWrite(0b11);
}

test "body locks from several threads" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 64, 4);
    defer context.deinit();
    const bm = &context.body_manager;
    const locking = BodyLockInterface.initLocking(bm);

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var ids: [16]BodyID = undefined;
    for (&ids) |*id| id.* = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();

    const num_threads = 4;
    const num_iterations = 500;
    const Worker = struct {
        fn run(lock_interface: *const BodyLockInterface, body_ids: []const BodyID, thread_index: usize) void {
            var rng: u32 = @intCast(thread_index * 7919 + 1);
            for (0..num_iterations) |iteration| {
                rng = rng *% 1664525 +% 1013904223;
                const a = body_ids[(rng >> 8) % body_ids.len];
                switch (iteration % 3) {
                    0 => {
                        // Read-modify-write under a write lock: no increment may get lost
                        var lock: BodyLockWrite = .init(lock_interface, a);
                        defer lock.deinit();
                        const body = lock.getBody();
                        body.setUserData(body.getUserData() + 1);
                    },
                    1 => {
                        // Several bodies at once (possibly sharing mutexes)
                        const b = body_ids[(rng >> 16) % body_ids.len];
                        const pair = [_]BodyID{ a, b };
                        var lock: BodyLockMultiWrite = .init(lock_interface, &pair);
                        defer lock.deinit();
                        const body = lock.getBody(0).?;
                        body.setUserData(body.getUserData() + 1);
                    },
                    else => {
                        // Readers
                        var lock: BodyLockRead = .init(lock_interface, a);
                        defer lock.deinit();
                        std.mem.doNotOptimizeAway(lock.getBody().getUserData());
                    },
                }
            }
        }
    };
    var threads: [num_threads]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ &locking, &ids, i });

    // Meanwhile this thread locks everything a few times and checks the bodies
    for (0..20) |_| {
        bm.lockAllBodies();
        for (ids) |id| std.mem.doNotOptimizeAway(bm.getBody(id).getUserData());
        bm.unlockAllBodies();
        std.Thread.yield() catch {};
    }
    for (threads) |t| t.join();

    var total: u64 = 0;
    for (ids) |id| total += bm.getBody(id).getUserData();
    const writes_per_thread = num_iterations - num_iterations / 3; // Iterations 0 and 1 of every 3 write
    try expectEqual(@as(u64, num_threads * writes_per_thread), total);
}

test "BodyManager activation from several threads" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 256, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    const ActivationCounter = struct {
        activated: std.atomic.Value(u32) = .init(0),
        deactivated: std.atomic.Value(u32) = .init(0),
        pub fn onBodyActivated(self: *@This(), body_id: BodyID, body_user_data: u64) void {
            _ = .{ body_id, body_user_data };
            _ = self.activated.fetchAdd(1, .monotonic);
        }
        pub fn onBodyDeactivated(self: *@This(), body_id: BodyID, body_user_data: u64) void {
            _ = .{ body_id, body_user_data };
            _ = self.deactivated.fetchAdd(1, .monotonic);
        }
    };
    var counter: ActivationCounter = .{};
    bm.setBodyActivationListener(.init(&counter));

    const sphere = try SphereShape.create(allocator, 1, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var ids: [128]BodyID = undefined;
    for (&ids) |*id| id.* = (try context.createAndAddBody(&settings, .dont_activate)).?.getID();

    // Every thread activates and deactivates its own bodies (with the body locks held, like BodyInterface)
    const Worker = struct {
        fn run(manager: *BodyManager, body_ids: []const BodyID) void {
            const lock_interface = BodyLockInterface.initLocking(manager);
            for (0..10) |_| {
                for (body_ids) |id| {
                    var lock: BodyLockWrite = .init(&lock_interface, id);
                    defer lock.deinit();
                    manager.activateBodies(&.{id});
                }
                var lock: BodyLockMultiWrite = .init(&lock_interface, body_ids);
                defer lock.deinit();
                manager.deactivateBodies(body_ids[0 .. body_ids.len / 2]);
            }
        }
    };
    var threads: [4]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ bm, ids[i * 32 ..][0..32] });
    for (threads) |t| t.join();

    // Half of every thread's bodies are active, the active body array is consistent
    try expectEqual(@as(u32, 64), bm.getNumActiveBodies(.rigid_body));
    for (activeBodies(bm), 0..) |id, i| try expectEqual(@as(u32, @intCast(i)), bm.getBody(id).getIndexInActiveBodiesInternal());
    try expectEqual(@as(u32, 4 * (32 + 9 * 16)), counter.activated.load(.monotonic));
    try expectEqual(@as(u32, 4 * 10 * 16), counter.deactivated.load(.monotonic));
}

/// Init, create bodies of every kind, add, activate, save, restore, deinit: every allocation fails once
fn createBodies(allocator: Allocator) !void {
    var context: BodyTestContext = undefined;
    try context.init(allocator, 8, 2);
    defer context.deinit();
    const bm = &context.body_manager;

    const box_settings = try zolt.BoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), .{});
    var settings = BodyCreationSettings.initWithSettings(box_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    _ = (try context.createAndAddBody(&settings, .activate)).?;
    settings.motion_type = .static;
    const static = (try context.createAndAddBody(&settings, .dont_activate)).?;
    settings.motion_type = .kinematic;
    _ = (try context.createAndAddBody(&settings, .activate)).?;

    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try bm.getBodyIDs(allocator, &ids);
    try bm.getActiveBodies(allocator, .rigid_body, &ids);
    try bm.invalidateContactCacheForBody(static);
    bm.validateContactCacheForAllBodies();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    try bm.saveState(recorder.recorder(), null);
    if (recorder.recorder().isFailed()) return error.OutOfMemory; // The recorder could not grow
    bm.deactivateBodies(ids.items);
    recorder.rewind();
    if (!try bm.restoreState(recorder.recorder())) return error.TestUnexpectedResult;
}

test "BodyManager out of memory in init, body creation, save and restore" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, createBodies, .{});
}

test "BodyManager init with the number of mutexes from the CPU count" {
    var context: BodyTestContext = undefined;
    try context.init(std.testing.allocator, 0, 0);
    defer context.deinit();
    const n = context.body_manager.body_mutexes.getNumMutexes();
    try expect(n >= 1 and n <= 64 and std.math.isPowerOfTwo(n));
    try expectEqual(@as(u32, 0), context.body_manager.getMaxBodies());

    var context2: BodyTestContext = undefined;
    try context2.init(std.testing.allocator, 1, 100);
    defer context2.deinit();
    try expectEqual(@as(u32, 64), context2.body_manager.body_mutexes.getNumMutexes());
    try expectEqual(~@as(BodyManager.MutexMask, 0), context2.body_manager.getAllBodiesMutexMask());

    var context3: BodyTestContext = undefined;
    try context3.init(std.testing.allocator, 1, 5);
    defer context3.deinit();
    try expectEqual(@as(u32, 8), context3.body_manager.body_mutexes.getNumMutexes());

    // An uninitialized manager can be deinitialized
    var empty: BodyManager = .empty;
    empty.deinit();
}
