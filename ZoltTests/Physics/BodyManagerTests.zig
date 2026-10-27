//! Zolt addition, no Jolt file: tests of BodyManager, the body locks and the state recorder (Phase 5 F1)
//!
//! Jolt has no unit test file for these classes (its tests go through a PhysicsSystem, Wave C). Adapted from the
//! Phase 5 architecture prototype (SimulationPrototype.zig, removed in F2: "BodyManager ID allocation like Jolt", the
//! lock parts of "body locks, the tagged lock interface and BodyInterface", the body parts of "state recorder save,
//! restore, validation and filter") plus activation, the active body arrays, the listener, multi-threaded locking and
//! out of memory. ZoltParity/Physics/BodyParity.zig compares the IDs, orders and state bytes with the C++ library.
//! The tests "BodyManager body pool: ..." check the body pool of Phase 13 (slots, pages, reuse, bodies that move to
//! another BodyManager or outlive theirs, threads, out of memory), with the bite proofs above them.

const std = @import("std");
const builtin = @import("builtin");
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

// ---------------------------------------------------------------------------------------------------------------------
// The body pool (Roadmap Phase 13, class A: memory placement only, see the header of BodyManager.zig). The rigid bodies
// with motion properties live in 64 KB pages, aligned to 64 KB, from the allocator of the BodyManager that creates them:
// a 64 byte header, then 64 byte aligned slots of exactly @sizeOf(BodyWithMotionProperties). Freed slots are reused,
// the last one freed first. A body finds its page from its address, so any BodyManager destroys any body, also after the
// BodyManager that created it is gone (Jolt's ownership); a page is freed when its BodyManager is gone and its last body
// is destroyed.
//
// Bite proofs (each change made the named tests fail in Debug, in both unit test binaries, then it was reverted; 13
// tests: the 12 below and BodyInterfaceTests "a body moves to another PhysicsSystem ..."):
// - no owner flag (destroy frees a page when its last body goes, also while its BodyManager exists): the 12 tests below
//   crash (the BodyManager's next body or its deinit uses the freed page);
// - a page freed while it holds bodies (the pool's deinit frees every page): the 4 tests whose bodies outlive their
//   BodyManager crash ("bodies outlive their BodyManager ...", "a BodyManager is deinitialized while other threads
//   destroy ...", "every allocation ... fails in turn", BodyInterfaceTests "a body moves to another PhysicsSystem
//   ...");
// - no page lock (acquire and release do nothing): the 3 tests that create or destroy bodies on several threads crash
//   or fail, in 3 of 3 runs ("a BodyManager is deinitialized while other threads destroy ..." leaks a page: two
//   destroys of its last bodies lost an update of its count);
// - a wrong mask (the page of a slot is its address with the low 15 bits cleared): 9 tests crash (destroy asserts the
//   page's magic number);
// - pages aligned to 4 KB instead of 64 KB: 12 tests crash or fail (the misaligning allocator, and the testing
//   allocator, whose page allocator aligns large blocks to 4 KB only);
// - a first in first out free list (a freed slot appended at the end of its page's list): "freed slots are reused, the
//   last one freed first";
// - the page table not grown (addPage writes past its end): 8 tests crash;
// - the pool's deinit does not free the pages without bodies: the 12 tests below leak or fail.
// With ThreadSanitizer (a temporary sanitize_thread on the library and unit test modules of build.zig), the 3 threaded
// tests report no data race, and "a BodyManager is deinitialized while other threads destroy ..." reports one when
// deinit clears the owner flag after it releases the page lock, and one when destroy reads the flag after it releases
// the page lock.

/// The size and alignment of a page of the body pool
const page_size = 64 * 1024;

/// The size of a body with motion properties: 320 bytes, 384 in double precision
const body_size = @sizeOf(zolt.BodyWithMotionProperties);

/// The slots of a page, after its 64 byte header: 204, 170 in double precision
const slots_per_page: u32 = (page_size - 64) / body_size;

/// The rigid body with motion properties around a body of the pool
fn bodyWithMotionProperties(body: *Body) *zolt.BodyWithMotionProperties {
    return @fieldParentPtr("body", body);
}

fn bodyAddress(body: *Body) usize {
    return @intFromPtr(bodyWithMotionProperties(body));
}

/// The page of a body of the pool
fn pageOf(body: *Body) usize {
    return bodyAddress(body) & ~@as(usize, page_size - 1);
}

const BodyPoolStats = BodyManager.BodyPoolStats;

/// An allocator whose blocks are aligned to exactly the alignment that is asked for, never to twice that: every block
/// starts `alignment` bytes into a block of the inner allocator that is aligned to twice the alignment. A pool whose
/// pages are less aligned than it assumes then finds the wrong page of a body.
const MisaligningAllocator = struct {
    inner: Allocator,

    fn allocator(self: *MisaligningAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = Allocator.noResize, .remap = Allocator.noRemap, .free = free } };
    }

    fn doubled(alignment: std.mem.Alignment) std.mem.Alignment {
        return .fromByteUnits(2 * alignment.toByteUnits());
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *MisaligningAllocator = @ptrCast(@alignCast(ctx));
        const offset = alignment.toByteUnits();
        const block = self.inner.rawAlloc(len + offset, doubled(alignment), ret_addr) orelse return null;
        return block + offset;
    }

    fn free(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        const self: *MisaligningAllocator = @ptrCast(@alignCast(ctx));
        const offset = alignment.toByteUnits();
        self.inner.rawFree((memory.ptr - offset)[0 .. memory.len + offset], doubled(alignment), ret_addr);
    }
};

/// A thread safe allocator around another one that counts its allocations, its live blocks and its live 64 KB blocks
/// (the pages of the body pool). Two of them are two different allocators: a block that one of them frees but the other
/// one allocated would break both counts.
const CountingAllocator = struct {
    inner: Allocator,
    num_allocations: std.atomic.Value(u32) = .init(0),
    num_blocks: std.atomic.Value(i32) = .init(0),
    num_pages: std.atomic.Value(i32) = .init(0),

    fn allocator(self: *CountingAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = resize, .remap = remap, .free = free } };
    }

    fn isPage(len: usize, alignment: std.mem.Alignment) bool {
        return len == page_size and alignment.toByteUnits() == page_size;
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *CountingAllocator = @ptrCast(@alignCast(ctx));
        const memory = self.inner.rawAlloc(len, alignment, ret_addr) orelse return null;
        _ = self.num_allocations.fetchAdd(1, .monotonic);
        _ = self.num_blocks.fetchAdd(1, .monotonic);
        if (isPage(len, alignment))
            _ = self.num_pages.fetchAdd(1, .monotonic);
        return memory;
    }

    fn resize(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) bool {
        const self: *CountingAllocator = @ptrCast(@alignCast(ctx));
        return self.inner.rawResize(memory, alignment, new_len, ret_addr);
    }

    fn remap(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
        const self: *CountingAllocator = @ptrCast(@alignCast(ctx));
        return self.inner.rawRemap(memory, alignment, new_len, ret_addr);
    }

    fn free(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        const self: *CountingAllocator = @ptrCast(@alignCast(ctx));
        _ = self.num_blocks.fetchSub(1, .monotonic);
        if (isPage(memory.len, alignment))
            _ = self.num_pages.fetchSub(1, .monotonic);
        self.inner.rawFree(memory, alignment, ret_addr);
    }

    fn pages(self: *const CountingAllocator) i32 {
        return self.num_pages.load(.monotonic);
    }
};

/// Settings of a dynamic sphere (a body with motion properties); the caller deinits them
fn dynamicSphere(allocator: Allocator) !BodyCreationSettings {
    const sphere = try SphereShape.create(allocator, 1, .{});
    return BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
}

/// destroyBodies for a body with an ID, freeBody for one without
fn destroyBody(manager: *BodyManager, body: *Body) void {
    if (body.getID().isInvalid())
        manager.freeBody(body)
    else
        manager.destroyBodies(&.{body.getID()});
}

test "BodyManager body pool: 64 byte aligned slots of exactly the body size after a 64 byte header, in 64 KB pages" {
    // The slots have no padding: 5 cache lines per body, 6 in double precision (asserts and the debug renderer do not
    // change the size)
    try expectEqual(@as(usize, if (Core.double_precision) 384 else 320), body_size);
    try expectEqual(@as(u32, if (Core.double_precision) 170 else 204), slots_per_page);

    // The pages come from an allocator that never aligns a block more than it is asked to
    var misaligning: MisaligningAllocator = .{ .inner = std.testing.allocator };
    var context: BodyTestContext = undefined;
    const max_bodies = 2 * slots_per_page;
    try context.init(misaligning.allocator(), max_bodies, 0);
    defer context.deinit();
    const bm = &context.body_manager;
    try expectEqual(BodyPoolStats{ .num_pages = 0, .page_table_len = 2, .num_slots = 0, .num_used = 0, .num_free = 0 }, bm.getBodyPoolStats());

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    // Bodies without an ID (CreateBodyWithoutID): the pages come when the bodies need them, the body beyond max_bodies
    // gets a third page (the page table grows)
    const num_bodies = max_bodies + 1;
    var bodies: [2 * 204 + 1]*Body = undefined;
    var num_created: usize = 0;
    defer for (bodies[0..num_created]) |b| bm.freeBody(b);
    for (bodies[0..num_bodies], 0..) |*b, i| {
        b.* = try bm.allocateBody(&settings);
        num_created += 1;
        const num_pages: u32 = @intCast(i / slots_per_page + 1);
        try expectEqual(BodyPoolStats{ .num_pages = num_pages, .page_table_len = if (num_pages <= 2) 2 else 4, .num_slots = num_pages * slots_per_page, .num_used = @intCast(i + 1), .num_free = 0 }, bm.getBodyPoolStats());
    }

    // The first body of a page starts after the 64 byte header, every other one follows the one before it without a gap,
    // and every body starts a cache line
    for (bodies[0..num_bodies], 0..) |b, i| {
        try expect(bm.isBodyInPool(b));
        try expectEqual(@as(usize, 0), pageOf(b) % page_size);
        try expectEqual(pageOf(bodies[i - i % slots_per_page]), pageOf(b));
        try expectEqual(pageOf(b) + 64 + (i % slots_per_page) * body_size, bodyAddress(b));
        try expectEqual(@as(usize, 0), bodyAddress(b) % 64);
        try expectEqual(@intFromPtr(&bodyWithMotionProperties(b).motion_properties), @intFromPtr(b.getMotionProperties()));
    }
    try expect(pageOf(bodies[0]) != pageOf(bodies[slots_per_page]) and pageOf(bodies[slots_per_page]) != pageOf(bodies[2 * slots_per_page]));

    // The last slot of a page ends inside it
    try expect(64 + slots_per_page * body_size <= page_size and 64 + (slots_per_page + 1) * body_size > page_size);

    // A static body has no motion properties: it is a Body from the allocator, not a slot
    settings.motion_type = .static;
    const static = try bm.allocateBody(&settings);
    try expect(static.getMotionPropertiesUnchecked() == null);
    try expect(!bm.isBodyInPool(static));
    bm.freeBody(static);
    try expectEqual(BodyPoolStats{ .num_pages = 3, .page_table_len = 4, .num_slots = 3 * slots_per_page, .num_used = num_bodies, .num_free = 0 }, bm.getBodyPoolStats());
}

/// `allocator` aligns blocks of 64 KB to 64 KB: 16 blocks straight from it, then the pages of a BodyManager that uses it
/// (3 pages of bodies: the first slot of a page starts 64 bytes after a 64 KB boundary, the other slots follow it)
fn expectPagesAligned(allocator: Allocator) !void {
    var blocks: [16][]align(page_size) u8 = undefined;
    var num_blocks: usize = 0;
    defer for (blocks[0..num_blocks]) |block| allocator.free(block);
    for (&blocks) |*block| {
        block.* = try allocator.alignedAlloc(u8, comptime .fromByteUnits(page_size), page_size);
        num_blocks += 1;
        try expectEqual(@as(usize, 0), @intFromPtr(block.ptr) % page_size);
        block.*[0] = 1;
        block.*[page_size - 1] = 1;
    }

    var context: BodyTestContext = undefined;
    try context.init(allocator, 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    var settings = try dynamicSphere(allocator);
    defer settings.deinit();

    const num_bodies = 2 * slots_per_page + 1;
    var bodies: [2 * 204 + 1]*Body = undefined;
    var num_created: usize = 0;
    defer for (bodies[0..num_created]) |b| bm.freeBody(b);
    for (bodies[0..num_bodies], 0..) |*b, i| {
        b.* = try bm.allocateBody(&settings);
        num_created += 1;
        const first = bodies[i - i % slots_per_page];
        try expectEqual(@as(usize, 0), (bodyAddress(first) - 64) % page_size);
        try expectEqual(bodyAddress(first) + (i % slots_per_page) * body_size, bodyAddress(b.*));
    }
    try expectEqual(@as(u32, 3), bm.getBodyPoolStats().num_pages);
}

test "BodyManager body pool: the SmpAllocator, the page allocator and the testing allocator align the pages to 64 KB" {
    // The pool asks for 64 KB aligned to 64 KB. Zig 0.16's SmpAllocator hands every block over 32 KB to
    // PageAllocator.map, which maps more than asked and unmaps the rest to reach an alignment above the 4 KB page size;
    // the testing allocator (a DebugAllocator) gets large blocks from the page allocator. The C allocator (posix_memalign)
    // needs libc, which the unit tests do not link: ZoltParity/Physics/BodyParity.zig checks it in the parity binary.
    try expectPagesAligned(std.heap.smp_allocator);
    try expectPagesAligned(std.heap.page_allocator);
    try expectPagesAligned(std.testing.allocator);
    if (builtin.link_libc) try expectPagesAligned(std.heap.c_allocator);
}

test "BodyManager body pool: freed slots are reused, the last one freed first" {
    var context: BodyTestContext = undefined;
    try context.init(std.testing.allocator, 16, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    var bodies: [4]*Body = undefined;
    for (&bodies) |*b| b.* = try bm.allocateBody(&settings);
    var addresses: [4]usize = undefined;
    for (bodies, &addresses) |b, *a| a.* = bodyAddress(b);
    try expectEqual(addresses[0] + 3 * body_size, addresses[3]);

    // Free three of them (freeBody: no ID), then create four: the freed slots in reverse order, then a new slot
    bm.freeBody(bodies[1]);
    bm.freeBody(bodies[3]);
    bm.freeBody(bodies[0]);
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 1, .num_free = 3 }, bm.getBodyPoolStats());
    for ([_]usize{ 0, 3, 1 }) |i| {
        bodies[i] = try bm.allocateBody(&settings);
        try expectEqual(addresses[i], bodyAddress(bodies[i]));
    }
    const new = try bm.allocateBody(&settings);
    try expectEqual(addresses[0] + 4 * body_size, bodyAddress(new));
    bm.freeBody(new);
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 4, .num_free = 1 }, bm.getBodyPoolStats());

    // A body with an ID that is destroyed (destroyBodies) gives its slot back too, and the next body gets it
    try expect(bm.addBody(bodies[2]));
    bm.destroyBodies(&.{bodies[2].getID()});
    bodies[2] = try bm.allocateBody(&settings);
    try expectEqual(addresses[2], bodyAddress(bodies[2]));

    for (bodies) |b| bm.freeBody(b);
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 0, .num_free = 5 }, bm.getBodyPoolStats());
}

test "BodyManager body pool: bodies beyond max_bodies get slots too, the page table grows only for them" {
    var counting: std.testing.FailingAllocator = .init(std.testing.allocator, .{});
    var context: BodyTestContext = undefined;
    try context.init(counting.allocator(), 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;
    try expectEqual(BodyPoolStats{ .num_pages = 0, .page_table_len = 1, .num_slots = 0, .num_used = 0, .num_free = 0 }, bm.getBodyPoolStats());

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    // Bodies without an ID, far beyond max_bodies (CreateBodyWithoutID). The first body of a page allocates it, the
    // second and third page also grow the page table (to 2 and 4 entries: an allocation and the free of the old table),
    // the fourth page fits in the table. The other bodies allocate nothing.
    const num_bodies = 3 * slots_per_page + 1;
    var bodies: [3 * 204 + 1]*Body = undefined;
    var num_created: usize = 0;
    defer for (bodies[0..num_created]) |b| bm.freeBody(b);
    for (bodies[0..num_bodies], 0..) |*b, i| {
        const allocations = counting.allocations;
        const deallocations = counting.deallocations;
        b.* = try bm.allocateBody(&settings);
        num_created += 1;
        const new_page = i % slots_per_page == 0;
        const grows = i == slots_per_page or i == 2 * slots_per_page;
        try expectEqual(@as(usize, @intFromBool(new_page)) + @intFromBool(grows), counting.allocations - allocations);
        try expectEqual(@as(usize, @intFromBool(grows)), counting.deallocations - deallocations);
        try expect(bm.isBodyInPool(b.*));
    }
    try expectEqual(BodyPoolStats{ .num_pages = 4, .page_table_len = 4, .num_slots = 4 * slots_per_page, .num_used = num_bodies, .num_free = 0 }, bm.getBodyPoolStats());

    // The pages stay while the BodyManager exists, also without bodies: freeing and creating the bodies again neither
    // frees nor allocates
    const allocations = counting.allocations;
    const deallocations = counting.deallocations;
    for (bodies[0..num_bodies]) |b| bm.freeBody(b);
    num_created = 0;
    try expectEqual(BodyPoolStats{ .num_pages = 4, .page_table_len = 4, .num_slots = 4 * slots_per_page, .num_used = 0, .num_free = num_bodies }, bm.getBodyPoolStats());
    for (bodies[0..num_bodies]) |*b| {
        b.* = try bm.allocateBody(&settings);
        num_created += 1;
    }
    try expectEqual(allocations, counting.allocations);
    try expectEqual(deallocations, counting.deallocations);

    // Bodies with IDs that are still in the manager at deinit are deleted there, the testing allocator checks that
    // nothing leaks
    try expect(bm.addBody(bodies[num_bodies - 2]));
    try expect(bm.addBody(bodies[num_bodies - 1]));
    num_created = num_bodies - 2;
}

test "BodyManager body pool: an unassigned body keeps its slot, gets an ID again, or is destroyed" {
    // UnassignBodyID(s) (removeBodies) and AssignBodyID (addBody) in the BodyManager that created the body: it stays in
    // its slot, and freeBody / destroyBodies give the slot back
    var context: BodyTestContext = undefined;
    try context.init(std.testing.allocator, 4, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    var bodies: [4]*Body = undefined;
    for (&bodies) |*b| {
        b.* = try bm.allocateBody(&settings);
        try expect(bm.addBody(b.*));
    }
    const address = bodyAddress(bodies[0]);

    // Unassigned and assigned again: the same slot, a new ID
    var out: [2]*Body = undefined;
    const old_id = bodies[0].getID();
    bm.removeBodies(&.{old_id}, out[0..1]);
    try expect(out[0] == bodies[0]);
    try expect(bodies[0].getID().isInvalid());
    try expect(bm.addBody(bodies[0]));
    try expect(!bodies[0].getID().eql(old_id));
    try expectEqual(address, bodyAddress(bodies[0]));
    try expect(bm.isBodyInPool(bodies[0]));

    // Unassigned, then destroyed without an ID (DestroyBodyWithoutID), or assigned again and destroyed with its ID: both
    // give the slot back
    bm.removeBodies(&.{ bodies[0].getID(), bodies[1].getID() }, &out);
    try expect(out[0] == bodies[0] and out[1] == bodies[1]);
    bm.freeBody(out[0]);
    try expect(bm.addBody(out[1]));
    bm.destroyBodies(&.{out[1].getID()});
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 2, .num_free = 2 }, bm.getBodyPoolStats());

    // Unassigned with out_bodies null (the caller keeps the pointer) and destroyed later
    bm.removeBodies(&.{bodies[2].getID()}, null);
    try expect(bodies[2].getID().isInvalid());
    try expectEqual(@as(u32, 1), bm.getNumBodies());
    bm.freeBody(bodies[2]);
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 1, .num_free = 3 }, bm.getBodyPoolStats());

    // The last body keeps its ID, deinit deletes it
}

test "BodyManager body pool: a body moves to a BodyManager with another allocator (Jolt's UnassignBodyID / AssignBodyID)" {
    // Jolt's sDeleteBody is static: a body that one PhysicsSystem unassigns can be assigned to another one and destroyed
    // there. The body stays in its slot in a page of A, B only has its ID; when B destroys it, the slot goes back to A's
    // page (B's allocator frees nothing), and A's next body takes it.
    var counting_a: CountingAllocator = .{ .inner = std.testing.allocator };
    var counting_b: CountingAllocator = .{ .inner = std.testing.allocator };
    var context_a: BodyTestContext = undefined;
    try context_a.init(counting_a.allocator(), 4, 0);
    defer context_a.deinit();
    var context_b: BodyTestContext = undefined;
    try context_b.init(counting_b.allocator(), 4, 0);
    defer context_b.deinit();
    const a = &context_a.body_manager;
    const b = &context_b.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    const body = try a.allocateBody(&settings);
    try expect(a.addBody(body));
    const address = bodyAddress(body);
    try expectEqual(@as(i32, 1), counting_a.pages());

    // Unassigned in A (UnassignBodyID), assigned in B (AssignBodyID): B has its ID, A's page has its memory
    var out: [1]*Body = undefined;
    a.removeBodies(&.{body.getID()}, &out);
    try expect(out[0] == body);
    try expect(b.addBody(body));
    try expectEqual(@as(u32, 0), a.getNumBodies());
    try expectEqual(@as(u32, 1), b.getNumBodies());
    try expect(a.isBodyInPool(body) and !b.isBodyInPool(body));
    try expect(b.getBody(body.getID()) == body);

    // And back to A, and to B again
    b.removeBodies(&.{body.getID()}, null);
    try expect(a.addBody(body));
    a.removeBodies(&.{body.getID()}, null);
    try expect(b.addBody(body));

    // B destroys it (DestroyBody): the slot goes back to A's page, B's allocator frees nothing, A's next body takes the
    // slot
    const blocks_b = counting_b.num_blocks.load(.monotonic);
    b.destroyBodies(&.{body.getID()});
    try expectEqual(blocks_b, counting_b.num_blocks.load(.monotonic));
    try expectEqual(@as(i32, 0), counting_b.pages());
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 0, .num_free = 1 }, a.getBodyPoolStats());
    try expectEqual(BodyPoolStats{ .num_pages = 0, .page_table_len = 1, .num_slots = 0, .num_used = 0, .num_free = 0 }, b.getBodyPoolStats());
    const next = try a.allocateBody(&settings);
    try expectEqual(address, bodyAddress(next));

    // A body without an ID that B destroys (DestroyBodyWithoutID) goes back to A's page too
    b.freeBody(next);
    try expectEqual(BodyPoolStats{ .num_pages = 1, .page_table_len = 1, .num_slots = slots_per_page, .num_used = 0, .num_free = 1 }, a.getBodyPoolStats());
    try expectEqual(@as(i32, 1), counting_a.pages());
}

test "BodyManager body pool: bodies outlive their BodyManager, the destroy of the last body of a page frees the page" {
    // As in Jolt, an unassigned body stays valid after its PhysicsSystem is gone, and another one destroys it. A's deinit
    // frees its pages without bodies, a page that still holds bodies is freed by the destroy of its last body, with A's
    // allocator.
    var counting_a: CountingAllocator = .{ .inner = std.testing.allocator };
    var counting_b: CountingAllocator = .{ .inner = std.testing.allocator };
    var context_b: BodyTestContext = undefined;
    try context_b.init(counting_b.allocator(), 4, 0);
    defer context_b.deinit();
    const b = &context_b.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    // A creates 3 pages of bodies. Page 0 keeps one body (unassigned, out_bodies null), page 1 two (one unassigned, one
    // that moves to B with an ID), page 2 none.
    var context_a: BodyTestContext = undefined;
    try context_a.init(counting_a.allocator(), 3 * slots_per_page, 0);
    var a_alive = true;
    defer if (a_alive) context_a.deinit();
    const a = &context_a.body_manager;
    var bodies: [3 * 204]*Body = undefined;
    for (bodies[0 .. 3 * slots_per_page]) |*body| {
        body.* = try a.allocateBody(&settings);
        try expect(a.addBody(body.*));
    }
    try expectEqual(@as(i32, 3), counting_a.pages());
    const kept = [_]*Body{ bodies[5], bodies[slots_per_page], bodies[slots_per_page + 7] };
    a.removeBodies(&.{kept[0].getID()}, null);
    var out: [2]*Body = undefined;
    a.removeBodies(&.{ kept[1].getID(), kept[2].getID() }, &out);
    try expect(out[0] == kept[1] and out[1] == kept[2]);
    try expect(b.addBody(kept[2]));
    for (bodies[0 .. 3 * slots_per_page]) |body| {
        if (body != kept[0] and body != kept[1] and body != kept[2])
            a.destroyBodies(&.{body.getID()});
    }
    try expectEqual(BodyPoolStats{ .num_pages = 3, .page_table_len = 3, .num_slots = 3 * slots_per_page, .num_used = 3, .num_free = 3 * slots_per_page - 3 }, a.getBodyPoolStats());

    // A's deinit frees page 2 and everything else of A; pages 0 and 1 stay
    context_a.deinit();
    a_alive = false;
    try expectEqual(@as(i32, 2), counting_a.pages());
    try expectEqual(@as(i32, 2), counting_a.num_blocks.load(.monotonic));

    // The bodies still work: B simulates nothing here, but reads and writes them
    try expect(b.getBody(kept[2].getID()) == kept[2]);
    kept[0].setUserData(1);
    kept[1].setUserData(2);
    try expectEqual(@as(u64, 1), kept[0].getUserData());

    // B destroys them. Page 1 is freed by its last body, whatever destroys it (freeBody, or destroyBodies of a body with
    // an ID in B), page 0 by its only body. B's allocator frees nothing of them.
    const blocks_b = counting_b.num_blocks.load(.monotonic);
    b.freeBody(kept[1]);
    try expectEqual(@as(i32, 2), counting_a.pages());
    b.destroyBodies(&.{kept[2].getID()});
    try expectEqual(@as(i32, 1), counting_a.pages());
    b.freeBody(kept[0]);
    try expectEqual(@as(i32, 0), counting_a.pages());
    try expectEqual(@as(i32, 0), counting_a.num_blocks.load(.monotonic));
    try expectEqual(blocks_b, counting_b.num_blocks.load(.monotonic));
}

test "BodyManager body pool: bodies are created and destroyed from several threads" {
    // 4 threads create bodies and destroy them again, in a BodyManager with 16 IDs (so a page table of 1 page). Each
    // thread keeps up to 512 bodies alive and gives about half of them an ID. Its sequence is fixed (its own random
    // numbers), and alone it holds at some moment 265-281 bodies, 140-146 of them with IDs when no addBody fails (the
    // maxima of the 4 threads). So in every run, whatever the timing: the pool allocates more than one page (more bodies
    // than the 204 or 170 slots of a page) while other threads create and destroy bodies, the page table grows, and
    // addBody fails (if none failed, a thread would hold more bodies with IDs than there are IDs). Every body carries its
    // owner and serial number in its user data: a slot that was handed out twice would be overwritten by the other
    // thread.
    var context: BodyTestContext = undefined;
    try context.init(std.testing.allocator, 16, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    const num_threads = 4;
    const Worker = struct {
        fn run(manager: *BodyManager, body_settings: *const BodyCreationSettings, thread_index: u64, started: *std.atomic.Value(u32), failed: *std.atomic.Value(bool), add_failures: *std.atomic.Value(u32)) void {
            // Start together, so that the threads overlap
            _ = started.fetchAdd(1, .monotonic);
            while (started.load(.monotonic) < num_threads) std.Thread.yield() catch {};
            work(manager, body_settings, thread_index, add_failures) catch failed.store(true, .monotonic);
        }

        fn work(manager: *BodyManager, body_settings: *const BodyCreationSettings, thread_index: u64, add_failures: *std.atomic.Value(u32)) !void {
            var live: [512]?*Body = @splat(null);
            var user_data: [512]u64 = undefined;
            defer for (live) |l| if (l) |b| destroyBody(manager, b);
            var rng: u32 = @intCast(thread_index * 7919 + 1);
            for (0..3000) |iteration| {
                rng = rng *% 1664525 +% 1013904223;
                const index = (rng >> 8) % live.len;
                if (live[index]) |b| {
                    try expectEqual(user_data[index], b.getUserData()); // Still ours
                    live[index] = null;
                    destroyBody(manager, b);
                } else {
                    const b = try manager.allocateBody(body_settings);
                    try expect(manager.isBodyInPool(b));
                    user_data[index] = thread_index << 32 | iteration;
                    b.setUserData(user_data[index]);
                    live[index] = b;
                    if ((rng >> 16) % 2 == 0 and !manager.addBody(b))
                        _ = add_failures.fetchAdd(1, .monotonic); // Out of IDs: the body stays without an ID
                }
            }
        }
    };
    var started: std.atomic.Value(u32) = .init(0);
    var failed: std.atomic.Value(bool) = .init(false);
    var add_failures: std.atomic.Value(u32) = .init(0);
    var threads: [num_threads]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ bm, &settings, @as(u64, i), &started, &failed, &add_failures });
    for (threads) |t| t.join();
    try expect(!failed.load(.monotonic));

    // The IDs ran out, the pool has more than one page (the page table grew from 1 entry), more slots than a page were
    // used (a used slot goes on a free list when its body is destroyed, a slot that was never used does not), and every
    // body is gone
    try expect(add_failures.load(.monotonic) > 0);
    const stats = bm.getBodyPoolStats();
    try expect(stats.num_pages > 1 and stats.page_table_len >= stats.num_pages);
    try expectEqual(@as(u32, 0), stats.num_used);
    try expect(stats.num_free > slots_per_page);
    try expectEqual(@as(u32, 0), bm.getNumBodies());
}

test "BodyManager body pool: bodies created in one BodyManager are destroyed through another, on several threads" {
    // Two BodyManagers with different allocators. Threads 0 and 2 create their bodies in A, threads 1 and 3 in B. A
    // thread hands bodies to the threads of the other manager through a mailbox (unassigned first, so without an ID),
    // and takes the bodies that come from them: it gives some of them an ID in its own manager, and destroys them through
    // its own manager (destroyBodies or freeBody). So bodies of A's pages are destroyed through B while A creates bodies
    // in the same pages, and the other way around. Every body carries its creator and serial number in its user data,
    // which the thread that holds it checks: a slot that was handed out twice would be overwritten.
    var counting: [2]CountingAllocator = .{ .{ .inner = std.testing.allocator }, .{ .inner = std.testing.allocator } };
    var contexts: [2]BodyTestContext = undefined;
    try contexts[0].init(counting[0].allocator(), 64, 0);
    defer contexts[0].deinit();
    try contexts[1].init(counting[1].allocator(), 64, 0);
    defer contexts[1].deinit();
    const managers = [2]*BodyManager{ &contexts[0].body_manager, &contexts[1].body_manager };

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    const num_threads = 4;
    const mailbox_len = 32;
    const Mailboxes = [2][mailbox_len]std.atomic.Value(?*Body);
    const Counts = struct {
        handed_over: std.atomic.Value(u32) = .init(0),
        taken: std.atomic.Value(u32) = .init(0),
    };
    const Worker = struct {
        fn run(all: [2]*BodyManager, body_settings: *const BodyCreationSettings, thread_index: u64, mailboxes: *Mailboxes, started: *std.atomic.Value(u32), failed: *std.atomic.Value(bool), counts: *Counts) void {
            _ = started.fetchAdd(1, .monotonic);
            while (started.load(.monotonic) < num_threads) std.Thread.yield() catch {};
            work(all, body_settings, thread_index, mailboxes, counts) catch failed.store(true, .monotonic);
        }

        fn work(all: [2]*BodyManager, body_settings: *const BodyCreationSettings, thread_index: u64, mailboxes: *Mailboxes, counts: *Counts) !void {
            const own = thread_index % 2;
            const manager = all[own];
            var live: [256]?*Body = @splat(null);
            var user_data: [256]u64 = undefined;
            defer for (live) |l| if (l) |b| destroyBody(manager, b);
            var rng: u32 = @intCast(thread_index * 104729 + 7);
            for (0..3000) |iteration| {
                rng = rng *% 1664525 +% 1013904223;
                const index = (rng >> 8) % live.len;
                const mailbox_index = (rng >> 20) % mailbox_len;
                switch ((rng >> 4) % 4) {
                    // Create a body in the own manager, or destroy one through it (one that came from the other manager
                    // too)
                    0, 1 => if (live[index]) |b| {
                        try expectEqual(user_data[index], b.getUserData());
                        live[index] = null;
                        destroyBody(manager, b);
                    } else {
                        const b = try manager.allocateBody(body_settings);
                        user_data[index] = thread_index << 32 | iteration;
                        b.setUserData(user_data[index]);
                        live[index] = b;
                        if ((rng >> 16) % 2 == 0)
                            _ = manager.addBody(b);
                    },
                    // Hand a body to the threads of the other manager (unassigned first)
                    2 => if (live[index]) |b| {
                        try expectEqual(user_data[index], b.getUserData());
                        if (!b.getID().isInvalid())
                            manager.removeBodies(&.{b.getID()}, null);
                        if (mailboxes[1 - own][mailbox_index].cmpxchgStrong(null, b, .release, .monotonic) == null) {
                            live[index] = null;
                            _ = counts.handed_over.fetchAdd(1, .monotonic);
                        }
                    },
                    // Take a body that the other manager's threads handed over, maybe give it an ID in the own manager
                    3 => if (live[index] == null) {
                        if (mailboxes[own][mailbox_index].swap(null, .acquire)) |b| {
                            user_data[index] = b.getUserData();
                            live[index] = b;
                            _ = counts.taken.fetchAdd(1, .monotonic);
                            if ((rng >> 16) % 2 == 0)
                                _ = manager.addBody(b);
                        }
                    },
                    else => unreachable,
                }
            }
        }
    };
    var mailboxes: Mailboxes = @splat(@splat(.init(null)));
    var started: std.atomic.Value(u32) = .init(0);
    var failed: std.atomic.Value(bool) = .init(false);
    var counts: Counts = .{};
    var threads: [num_threads]std.Thread = undefined;
    for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Worker.run, .{ managers, &settings, @as(u64, i), &mailboxes, &started, &failed, &counts });
    for (threads) |t| t.join();
    try expect(!failed.load(.monotonic));

    // The bodies that are still in a mailbox are destroyed through the manager they were handed to
    for (&mailboxes, managers) |*mailbox, manager| {
        for (mailbox) |*m| {
            if (m.swap(null, .acquire)) |b| manager.freeBody(b);
        }
    }

    // Bodies went both ways, all are destroyed, and each pool's pages are its own allocator's
    try expect(counts.handed_over.load(.monotonic) > 0 and counts.taken.load(.monotonic) > 0);
    for (managers, &counting) |manager, *c| {
        const stats = manager.getBodyPoolStats();
        try expectEqual(@as(u32, 0), stats.num_used);
        try expectEqual(@as(i32, @intCast(stats.num_pages)), c.pages());
        try expectEqual(@as(u32, 0), manager.getNumBodies());
    }
}

test "BodyManager body pool: a BodyManager is deinitialized while other threads destroy the last bodies of its pages through another" {
    // A's deinit races the destroys of the last bodies of its pages, page by page. For every page, exactly one of them
    // frees it, with A's allocator: never both (the testing allocator reports a double free, the count of A's allocator
    // would go below 0) and never neither (a leak). In each of 200 rounds, A creates bodies (without IDs) in 8 pages and
    // destroys all of them but the first and the last body of every page. Then 3 threads start at the same moment: one
    // deinits A, which goes through A's pages from the first one, and 2 destroy the 16 remaining bodies through B, one
    // body of every page each, from the last page, with a pause after every destroy that grows from round to round (0
    // to 32767 spin loop hints). With short pauses the destroys are done before the deinit gets to the pages (the
    // deinit frees all of them). With longer ones the deinit meets the destroys on a page: the pages before it still
    // hold a body when the deinit passes them (a destroy frees them), the pages after it are empty (the deinit frees
    // them), and on that page the page lock orders the deinit and the two destroys, which also race each other on every
    // page. A temporary probe (which thread freed each page) counted the rounds in which both the deinit and a destroy
    // freed pages: 86 to 94 of 200 in Debug and 108 to 115 in ReleaseSafe with 4 idle cores, 2 to 41 in Debug while
    // other processes took the cores.
    var counting_b: CountingAllocator = .{ .inner = std.testing.allocator };
    var context_b: BodyTestContext = undefined;
    try context_b.init(counting_b.allocator(), 4, 0);
    defer context_b.deinit();

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    const num_rounds = 200;
    const num_pages = 8;
    const num_kept = 2; // The bodies of a page that the threads destroy, one per thread
    const num_tasks = 1 + num_kept; // A's deinit, then the destroys
    const Round = struct {
        context_a: *BodyTestContext,
        b: *BodyManager,
        kept: *const [num_pages][num_kept]*Body,
        /// Spin loop hints between two destroys of a thread
        pause: u32,
        ready: std.atomic.Value(u32) = .init(0),
        go: std.atomic.Value(bool) = .init(false),

        fn run(round: *@This(), task: usize) void {
            _ = round.ready.fetchAdd(1, .monotonic);
            while (!round.go.load(.acquire)) std.atomic.spinLoopHint();
            round.work(task);
        }

        fn work(round: *@This(), task: usize) void {
            if (task == 0) {
                round.context_a.deinit(); // Goes through A's pages from the first one
            } else {
                var page: usize = num_pages;
                while (page > 0) {
                    page -= 1;
                    round.b.freeBody(round.kept[page][task - 1]);
                    for (0..round.pause) |_| std.atomic.spinLoopHint();
                }
            }
        }
    };

    const allocations_b = counting_b.num_allocations.load(.monotonic);
    const bodies = try std.testing.allocator.alloc(*Body, num_pages * slots_per_page);
    defer std.testing.allocator.free(bodies);
    for (0..num_rounds) |round_index| {
        var counting_a: CountingAllocator = .{ .inner = std.testing.allocator };
        var context_a: BodyTestContext = undefined;
        try context_a.init(counting_a.allocator(), 0, 0);
        var a_alive = true;
        defer if (a_alive) context_a.deinit();
        const a = &context_a.body_manager;

        // A fills its pages (in the order of its page table) and keeps the first and the last body of every page
        {
            var num_created: usize = 0;
            errdefer for (bodies[0..num_created]) |body| a.freeBody(body);
            for (bodies) |*body| {
                body.* = try a.allocateBody(&settings);
                num_created += 1;
            }
        }
        var kept: [num_pages][num_kept]*Body = undefined;
        for (bodies, 0..) |body, i| {
            const page = i / slots_per_page;
            switch (i % slots_per_page) {
                0 => kept[page][0] = body,
                slots_per_page - 1 => kept[page][1] = body,
                else => a.freeBody(body),
            }
        }
        var kept_alive = true;
        errdefer if (kept_alive) for (kept) |page| for (page) |body| a.freeBody(body);
        try expectEqual(@as(i32, num_pages), counting_a.pages());
        try expectEqual(BodyPoolStats{ .num_pages = num_pages, .page_table_len = num_pages, .num_slots = num_pages * slots_per_page, .num_used = num_pages * num_kept, .num_free = num_pages * (slots_per_page - num_kept) }, a.getBodyPoolStats());

        // The tasks deinit A and destroy the kept bodies, also when a thread cannot be spawned: the main thread then
        // runs the tasks of the missing threads
        const pause = (@as(u32, 1) << @intCast(round_index % 16)) - 1; // 0 to 32767 spin loop hints
        var round: Round = .{ .context_a = &context_a, .b = &context_b.body_manager, .kept = &kept, .pause = pause };
        a_alive = false;
        kept_alive = false;
        {
            var threads: [num_tasks]std.Thread = undefined;
            var num_spawned: usize = 0;
            errdefer {
                round.go.store(true, .release);
                for (num_spawned..num_tasks) |task| round.work(task);
                for (threads[0..num_spawned]) |t| t.join();
            }
            for (&threads, 0..) |*t, task| {
                t.* = try std.Thread.spawn(.{}, Round.run, .{ &round, task });
                num_spawned += 1;
            }
            while (round.ready.load(.monotonic) < num_tasks) std.Thread.yield() catch {};
            round.go.store(true, .release);
            for (threads) |t| t.join();
        }

        // Every page of A was freed exactly once, with A's allocator, and A freed all its memory
        try expectEqual(@as(i32, 0), counting_a.pages());
        try expectEqual(@as(i32, 0), counting_a.num_blocks.load(.monotonic));
    }

    // B's allocator was not used: it freed none of A's pages
    try expectEqual(allocations_b, counting_b.num_allocations.load(.monotonic));
    try expectEqual(@as(i32, 0), counting_b.pages());
}

test "BodyManager body pool: out of memory at a page allocation or at the growth of the page table leaves the pool unchanged" {
    var failing: std.testing.FailingAllocator = .init(std.testing.allocator, .{});
    var context: BodyTestContext = undefined;
    try context.init(failing.allocator(), slots_per_page, 0);
    defer context.deinit();
    const bm = &context.body_manager;

    var settings = try dynamicSphere(std.testing.allocator);
    defer settings.deinit();

    const num_bodies = 3 * slots_per_page + 1;
    var bodies: [3 * 204 + 1]*Body = undefined;
    var num_created: usize = 0;
    defer for (bodies[0..num_created]) |b| bm.freeBody(b);
    for (bodies[0..num_bodies], 0..) |*b, i| {
        // The first body of a page allocates the page; the second and third page also grow the page table (from 1 to 2
        // and 4 entries), after the page. Fail each of these allocations first: allocateBody fails, the pool stays as it
        // was, and a page that was allocated before the page table failed is freed again.
        const new_page = i % slots_per_page == 0;
        const grows = i == slots_per_page or i == 2 * slots_per_page;
        const stats = bm.getBodyPoolStats();
        if (new_page) {
            for (0..@as(usize, 1) + @intFromBool(grows)) |fail| {
                const deallocations = failing.deallocations;
                failing.fail_index = failing.alloc_index + fail;
                try std.testing.expectError(error.OutOfMemory, bm.allocateBody(&settings));
                failing.fail_index = std.math.maxInt(usize);
                try expectEqual(stats, bm.getBodyPoolStats());
                try expectEqual(fail, failing.deallocations - deallocations);
            }
        }

        // Then it works; the other bodies of a page allocate nothing
        const allocations = failing.allocations;
        b.* = try bm.allocateBody(&settings);
        num_created += 1;
        try expectEqual(@as(usize, @intFromBool(new_page)) + @intFromBool(grows), failing.allocations - allocations);
    }
    try expectEqual(BodyPoolStats{ .num_pages = 4, .page_table_len = 4, .num_slots = 4 * slots_per_page, .num_used = num_bodies, .num_free = 0 }, bm.getBodyPoolStats());
}

/// Init, then bodies through three pages (the page table grows), freed and created again, with IDs, moved to a second
/// BodyManager, destroyed, and the first BodyManager deinitialized before the second one destroys the bodies it got:
/// every allocation fails once (checkAllAllocationFailures), nothing leaks
fn createBodiesInThePool(allocator: Allocator) !void {
    // B is deinitialized after A: it destroys the bodies that moved from A, the last one of a page frees it
    var context_b: BodyTestContext = undefined;
    try context_b.init(allocator, 4, 0);
    defer context_b.deinit();
    const b = &context_b.body_manager;

    var context: BodyTestContext = undefined;
    try context.init(allocator, 300, 2);
    defer context.deinit();
    const bm = &context.body_manager;

    const box_settings = try zolt.BoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), .{});
    var settings = BodyCreationSettings.initWithSettings(box_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();

    // Bodies without an ID: three pages (the page table of 2 pages grows)
    var bodies: [2 * 204 + 10]*Body = undefined;
    const num_bodies = 2 * slots_per_page + 10;
    var num_created: usize = 0;
    defer for (bodies[0..num_created]) |body| bm.freeBody(body);
    for (bodies[0..num_bodies]) |*body| {
        body.* = try bm.allocateBody(&settings);
        num_created += 1;
    }

    // Free every third one and create it again (the free lists)
    var i: usize = 0;
    while (i < num_created) : (i += 3) {
        bm.freeBody(bodies[i]);
        bodies[i] = bm.allocateBody(&settings) catch |err| {
            num_created -= 1; // Keep the created bodies together for the deferred frees
            bodies[i] = bodies[num_created];
            return err;
        };
    }

    // Bodies with IDs, two of them move to B (unassigned in A, an ID in B), some are destroyed, the others deleted by
    // the deinits
    for (0..20) |_| {
        const body = try bm.allocateBody(&settings);
        if (!bm.addBody(body)) {
            bm.freeBody(body);
            return error.TestUnexpectedResult;
        }
    }
    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try bm.getBodyIDs(allocator, &ids);
    var moved: [2]*Body = undefined;
    bm.removeBodies(ids.items[0..2], &moved);
    for (moved) |body| {
        if (!b.addBody(body)) {
            b.freeBody(body);
            return error.TestUnexpectedResult;
        }
    }
    bm.destroyBodies(ids.items[2..10]);
}

test "BodyManager body pool: every allocation of init, of body creation and of a move fails in turn" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, createBodiesInThePool, .{});
}
