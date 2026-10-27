//! Zolt addition, no Jolt file: tests of the BodyInterface (Jolt tests it through PhysicsTests.cpp and friends, which
//! need the whole simulation, Wave C)
//! Status: complete
//!
//! - Every getter returns Jolt's default for an invalid and for a destroyed (stale) body ID.
//! - The setters change the body and activate it like Jolt (only bodies in the broad phase, not for velocities near
//!   zero, not with EActivation::DontActivate), SetPositionAndRotationWhenChanged ignores changes below the tolerance,
//!   SetMotionType(Static) deactivates, SetShape / NotifyShapeChanged keep the world position and update the mass,
//!   ActivateBodiesInAABox activates the bodies in the box, the creation functions with custom IDs and running out of
//!   bodies, Unassign / Assign of body IDs.
//! - Several threads use the locking interface at the same time: every thread works on its own bodies and all of them
//!   on a shared body, the results equal a single threaded run (repeated).
//! - Allocation failures of createAndAddBody (shape creation, the broad phase) and activateBodiesInAABox.
//! - A body moves to another PhysicsSystem with another allocator (UnassignBodyID, AssignBodyID) and outlives the first
//!   one, as in Jolt (Phase 13: its page belongs to the first system, the header of BodyManager.zig).
//! The values are compared bit for bit with Jolt in ZoltParity/Physics/BodyInterfaceParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const RMat44 = zolt.RMat44;
const AABox = zolt.AABox;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyInterface = zolt.BodyInterface;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const OffsetCenterOfMassShape = zolt.OffsetCenterOfMassShape;
const CollisionGroup = zolt.CollisionGroup;
const PhysicsMaterial = zolt.PhysicsMaterial;
const StateRecorderImpl = zolt.StateRecorderImpl;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

fn sphereSettings(allocator: Allocator, position: RVec3, motion_type: zolt.MotionType) !BodyCreationSettings {
    const sphere = try SphereShape.create(allocator, 0.5, .{});
    return BodyCreationSettings.initWithShape(sphere.asShape(), position, Quat.identity(), motion_type, if (motion_type == .static) Layers.non_moving else Layers.moving);
}

test "BodyInterface: getters return Jolt's defaults for invalid and destroyed bodies" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    var settings = try sphereSettings(allocator, RVec3.init(1, 2, 3), .dynamic);
    defer settings.deinit();
    const destroyed = try bi.createAndAddBody(&settings, .activate);
    bi.removeBody(destroyed);
    bi.destroyBody(destroyed);
    const replacement = try bi.createAndAddBody(&settings, .activate); // Reuses the slot with a new sequence number
    try expect(replacement.getIndex() == destroyed.getIndex() and !replacement.eql(destroyed));

    for ([_]BodyID{ .invalid, destroyed }) |id| {
        try expect(bi.getPosition(id).eql(RVec3.zero()));
        try expect(bi.getCenterOfMassPosition(id).eql(RVec3.zero()));
        const pr = bi.getPositionAndRotation(id);
        try expect(pr.position.eql(RVec3.zero()) and pr.rotation.eql(Quat.identity()));
        try expect(bi.getRotation(id).eql(Quat.identity()));
        try expect(bi.getWorldTransform(id).eql(RMat44.identity()));
        try expect(bi.getCenterOfMassTransform(id).eql(RMat44.identity()));
        const v = bi.getLinearAndAngularVelocity(id);
        try expect(v.linear_velocity.eql(Vec3.zero()) and v.angular_velocity.eql(Vec3.zero()));
        try expect(bi.getLinearVelocity(id).eql(Vec3.zero()));
        try expect(bi.getAngularVelocity(id).eql(Vec3.zero()));
        try expect(bi.getPointVelocity(id, RVec3.init(1, 1, 1)).eql(Vec3.zero()));
        try expect(bi.getBodyType(id) == .rigid_body);
        try expect(bi.getMotionType(id) == .static);
        try expect(bi.getMotionQuality(id) == .discrete);
        try expect(bi.getInverseInertia(id).eql(Mat44.identity()));
        try expectEqual(@as(f32, 0), bi.getRestitution(id));
        try expectEqual(@as(f32, 0), bi.getFriction(id));
        try expectEqual(@as(f32, 1), bi.getGravityFactor(id));
        try expectEqual(@as(f32, 500), bi.getMaxLinearVelocity(id));
        try expectEqual(@as(f32, 0.25 * std.math.pi * 60.0), bi.getMaxAngularVelocity(id));
        try expect(bi.getUseManifoldReduction(id));
        try expect(!bi.isSensor(id));
        try expect(bi.getCollisionGroup(id).eql(&CollisionGroup.invalid));
        var ts = bi.getTransformedShape(id);
        defer ts.deinit();
        try expect(ts.shape.get() == null and ts.body_id.isInvalid());
        try expectEqual(@as(u64, 0), bi.getUserData(id));
        try expect(bi.getMaterial(id, .empty) == PhysicsMaterial.default);
        try expectEqual(zolt.object_layer_invalid, bi.getObjectLayer(id));
        try expect(!bi.isActive(id) and !bi.isAdded(id));
        var shape = bi.getShape(id);
        defer shape.deinit();
        try expect(shape.get() == null);
        try expect(!bi.applyBuoyancyImpulse(id, RVec3.zero(), Vec3.axisY(), 1, 0, 0, Vec3.zero(), Vec3.init(0, -10, 0), 0.1, .{}));

        // Setters on a body that cannot be locked do nothing
        bi.setPosition(id, RVec3.init(5, 5, 5), .activate);
        bi.setLinearVelocity(id, Vec3.init(1, 0, 0));
        bi.addForce(id, Vec3.init(1, 0, 0), .{});
        bi.addImpulse(id, Vec3.init(1, 0, 0));
        bi.activateBody(id);
        bi.deactivateBody(id);
        bi.removeBody(id);
        try bi.addBody(id, .activate);
        try bi.invalidateContactCache(id);
    }

    // The replacement was not touched
    try expect(bi.getPosition(replacement).eql(RVec3.init(1, 2, 3)));
    try expect(bi.getLinearVelocity(replacement).eql(Vec3.zero()));
    try expect(bi.isActive(replacement));
}

test "BodyInterface: setters and activation" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .max_bodies = 8 });
    defer c.deinit();
    const bi = c.getBodyInterface();

    var settings = try sphereSettings(allocator, RVec3.init(0, 10, 0), .dynamic);
    defer settings.deinit();
    const id = try bi.createAndAddBody(&settings, .dont_activate);
    try expect(bi.isAdded(id) and !bi.isActive(id));

    // Velocities near zero do not activate, larger ones do; DontActivate is respected by the forces
    bi.setLinearVelocity(id, Vec3.replicate(1.0e-7));
    try expect(!bi.isActive(id));
    bi.addForce(id, Vec3.init(0, 100, 0), .{ .activation = .dont_activate });
    try expect(!bi.isActive(id));
    bi.setLinearAndAngularVelocity(id, Vec3.init(1, 0, 0), Vec3.zero());
    try expect(bi.isActive(id));
    bi.deactivateBody(id);
    try expect(!bi.isActive(id));

    // Velocities are clamped to the maximum
    bi.setMaxLinearVelocity(id, 10);
    bi.setLinearVelocity(id, Vec3.init(100, 0, 0));
    try expectEqual(@as(f32, 10), bi.getLinearVelocity(id).getX());
    bi.addLinearVelocity(id, Vec3.init(100, 0, 0));
    try expectEqual(@as(f32, 10), bi.getLinearVelocity(id).getX());

    // SetPositionAndRotationWhenChanged ignores changes below the tolerance (a rotation change here: the tolerance of
    // RVec3.isClose depends on the precision, 1.0e-24 for doubles)
    bi.deactivateBody(id);
    const p = bi.getPosition(id);
    bi.setPositionAndRotationWhenChanged(id, p, Quat.rotation(Vec3.axisY(), 1.0e-7), .activate);
    try expect(bi.getPosition(id).eql(p) and bi.getRotation(id).eql(Quat.identity()) and !bi.isActive(id));
    bi.setPositionAndRotationWhenChanged(id, p.addVec3(Vec3.init(1, 0, 0)), Quat.identity(), .activate);
    try expect(!bi.getPosition(id).eql(p) and bi.isActive(id));

    // MoveKinematic sets the velocity and activates (a kinematic body)
    var kinematic_settings = try sphereSettings(allocator, RVec3.init(5, 0, 0), .kinematic);
    defer kinematic_settings.deinit();
    const kinematic = try bi.createAndAddBody(&kinematic_settings, .dont_activate);
    bi.moveKinematic(kinematic, RVec3.init(6, 0, 0), Quat.identity(), 0.5);
    try expect(bi.isActive(kinematic));
    try expect(bi.getLinearVelocity(kinematic).isClose(Vec3.init(2, 0, 0), .{}));

    // SetMotionType(Static) deactivates the body, back to dynamic activates it
    var switchable_settings = try sphereSettings(allocator, RVec3.init(-5, 0, 0), .dynamic);
    defer switchable_settings.deinit();
    const switchable = try bi.createAndAddBody(&switchable_settings, .activate);
    bi.setMotionType(switchable, .static, .activate);
    try expect(!bi.isActive(switchable) and bi.getMotionType(switchable) == .static);
    bi.setMotionType(switchable, .dynamic, .activate);
    try expect(bi.isActive(switchable));

    // SetShape keeps the body where it is and updates the mass (and the center of mass)
    const offset_box = try OffsetCenterOfMassShape.create(allocator, (try BoxShape.create(allocator, Vec3.replicate(2), .{})).asShape(), Vec3.init(0, 1, 0));
    const inverse_inertia_before = bi.getInverseInertia(id);
    try bi.setShape(id, offset_box.asShape(), true, .dont_activate);
    try expect(bi.getPosition(id).eql(p.addVec3(Vec3.init(1, 0, 0))));
    try expect(bi.getCenterOfMassPosition(id).isClose(bi.getPosition(id).addVec3(Vec3.init(0, 1, 0)), .{ .max_dist_sq = 1.0e-8 }));
    try expect(!bi.getInverseInertia(id).eql(inverse_inertia_before));
    {
        var shape = bi.getShape(id);
        defer shape.deinit();
        try expect(shape.get().? == offset_box.asShape());
    }

    // NotifyShapeChanged with the old center of mass moves the body so that the world position stays
    try bi.notifyShapeChanged(id, Vec3.zero(), false, .dont_activate);
    try expect(bi.getPosition(id).isClose(p.addVec3(Vec3.init(1, 1, 0)), .{ .max_dist_sq = 1.0e-8 }));

    // ActivateBodiesInAABox activates the bodies in the box only
    bi.deactivateBodies(&.{ id, switchable });
    try bi.activateBodiesInAABox(&AABox.init(Vec3.init(-6, -1, -1), Vec3.init(-4, 1, 1)), &.{}, &.{});
    try expect(bi.isActive(switchable) and !bi.isActive(id));

    // Custom IDs: a free one works, a taken one does not
    const custom = BodyID.fromIndexAndSequenceNumber(7, 3);
    try expect((try bi.createBodyWithID(custom, &settings)).?.getID().eql(custom));
    try expect(try bi.createBodyWithID(custom, &settings) == null);
    try expect(try bi.createBodyWithID(BodyID.fromIndexAndSequenceNumber(8, 0), &settings) == null); // Beyond max bodies

    // Unassign an ID and assign a new one (the body was not added)
    const body = bi.unassignBodyID(custom);
    try expect(body.getID().isInvalid());
    try expect(bi.assignBodyID(body));
    const reassigned = body.getID();
    try expect(!reassigned.isInvalid());
    bi.destroyBody(reassigned);

    // Out of bodies: createBody returns null, createAndAddBody the invalid ID
    var created: u32 = 0;
    while (try bi.createBody(&settings)) |_| created += 1;
    try expect(created > 0);
    try expect((try bi.createAndAddBody(&settings, .activate)).isInvalid());
}

/// The operations of one thread on its own bodies (the same in the single and the multi threaded run)
fn threadWork(bi: *BodyInterface, ids: []const BodyID, shared: BodyID, iterations: u32) void {
    for (0..iterations) |k| {
        const f: f32 = @floatFromInt(k);
        for (ids) |id| {
            bi.setPosition(id, bi.getPosition(id).addVec3(Vec3.init(0.25, 0, 0)), .dont_activate);
            bi.addLinearVelocity(id, Vec3.init(0, 0.5, 0));
            bi.addImpulse(id, Vec3.init(0, 0, 1));
            bi.setFriction(id, f);
            bi.setRotation(id, Quat.rotation(Vec3.axisY(), 0.01 * f), .activate);
            if (k % 3 == 0) bi.deactivateBody(id) else bi.activateBody(id);
            _ = bi.getCenterOfMassTransform(id);
            var ts = bi.getTransformedShape(id);
            ts.deinit();
        }
        bi.addLinearVelocity(shared, Vec3.init(1, 0, 0));
        _ = bi.getLinearVelocity(shared);
        _ = bi.isActive(shared);
    }
}

/// The state of a body that the thread operations change
fn bodyState(allocator: Allocator, bi: *BodyInterface, id: BodyID) ![]u8 {
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    const r = recorder.recorder();
    r.write(bi.getPosition(id));
    r.write(bi.getRotation(id));
    r.write(bi.getLinearVelocity(id));
    r.write(bi.getAngularVelocity(id));
    r.write(bi.getFriction(id));
    r.write(bi.isActive(id));
    return allocator.dupe(u8, recorder.getData());
}

/// Run the thread operations with `num_threads` threads (0: on this thread, one block of bodies after the other) and
/// return the states of the bodies
fn runThreads(allocator: Allocator, num_threads: u32, iterations: u32) !std.ArrayList([]u8) {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    const blocks = 4;
    const per_block = 6;
    var ids: [blocks * per_block]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        var settings = try sphereSettings(allocator, RVec3.init(@floatFromInt(3 * i), 0, 0), .dynamic);
        defer settings.deinit();
        id.* = try bi.createAndAddBody(&settings, .dont_activate);
    }
    var shared_settings = try sphereSettings(allocator, RVec3.init(0, 20, 0), .dynamic);
    defer shared_settings.deinit();
    shared_settings.max_linear_velocity = 1.0e6;
    const shared = try bi.createAndAddBody(&shared_settings, .activate);

    if (num_threads == 0) {
        for (0..blocks) |b| threadWork(bi, ids[b * per_block ..][0..per_block], shared, iterations);
    } else {
        var threads: [blocks]std.Thread = undefined;
        for (&threads, 0..) |*t, b| t.* = try std.Thread.spawn(.{}, threadWork, .{ bi, ids[b * per_block ..][0..per_block], shared, iterations });
        for (threads) |t| t.join();
    }

    var states: std.ArrayList([]u8) = .empty;
    errdefer {
        for (states.items) |s| allocator.free(s);
        states.deinit(allocator);
    }
    for (ids) |id| try states.append(allocator, try bodyState(allocator, bi, id));
    try expectEqual(@as(f32, @floatFromInt(blocks * iterations)), bi.getLinearVelocity(shared).getX()); // Every addition counted
    return states;
}

fn freeStates(allocator: Allocator, states: *std.ArrayList([]u8)) void {
    for (states.items) |s| allocator.free(s);
    states.deinit(allocator);
}

test "BodyInterface: several threads use the locking interface" {
    const allocator = std.testing.allocator;
    const iterations = 200;
    var reference = try runThreads(allocator, 0, iterations);
    defer freeStates(allocator, &reference);

    for (0..10) |_| {
        var states = try runThreads(allocator, 4, iterations);
        defer freeStates(allocator, &states);
        for (states.items, reference.items) |s, r|
            try std.testing.expectEqualSlices(u8, r, s);
    }
}

fn createAndActivate(allocator: Allocator) !void {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    var settings: BodyCreationSettings = .initWithSettings((try SphereShapeSettings.create(allocator, 1, .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const a = try bi.createAndAddBody(&settings, .dont_activate);
    settings.position = RVec3.init(1, 0, 0);
    const b = try bi.createAndAddBody(&settings, .dont_activate);
    try bi.activateBodiesInAABox(&AABox.init(Vec3.replicate(-5), Vec3.replicate(5)), &.{}, &.{});
    try expect(bi.isActive(a) and bi.isActive(b));
    try expectEqual(@as(u32, 2), c.getSystem().getNumBodies());
}

test "BodyInterface: out of memory in createAndAddBody and activateBodiesInAABox" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, createAndActivate, .{});
}

test "BodyInterface: a body moves to another PhysicsSystem with another allocator and outlives the first one" {
    // Jolt: UnassignBodyID hands the body to the caller ("caller assumes ownership"), AssignBodyID gives it an ID in
    // another PhysicsSystem, which simulates it and destroys it, also after the first PhysicsSystem is gone (sDeleteBody
    // is static). Zolt (Phase 13): the body stays in its slot in a page of the first system, and its destroy frees the
    // page (the page's last body) with the first system's allocator (the header of BodyManager.zig).
    var allocator_a: std.testing.FailingAllocator = .init(std.testing.allocator, .{});
    var allocator_b: std.testing.FailingAllocator = .init(std.testing.allocator, .{});
    var b: PhysicsTestContext = undefined;
    try b.init(allocator_b.allocator(), .{});
    defer b.deinit();
    var a: PhysicsTestContext = undefined;
    try a.init(allocator_a.allocator(), .{});
    var a_alive = true;
    defer if (a_alive) a.deinit();

    // A sphere falls in A for a step
    const body = try a.createSphere(RVec3.init(0, 10, 0), 0.5, .dynamic, .discrete, Layers.moving, .{});
    _ = try a.simulateSingleStep();
    const position_a = body.getPosition();
    try expect(position_a.getY() < 10);

    // Removed from A and unassigned (UnassignBodyID), assigned in B (AssignBodyID) and added to B
    const bi_a = a.getBodyInterface();
    const id_a = body.getID();
    bi_a.removeBody(id_a);
    try expect(bi_a.unassignBodyID(id_a) == body);
    try expectEqual(@as(u32, 0), a.getSystem().getNumBodies());
    const bi_b = b.getBodyInterface();
    try expect(bi_b.assignBodyID(body));
    try bi_b.addBody(body.getID(), .activate);
    try expectEqual(@as(u32, 1), b.getSystem().getNumBodies());

    // A is gone, its allocator still holds the body's page (and the body's shape): the body falls on in B
    a.deinit();
    a_alive = false;
    try expect(allocator_a.allocated_bytes - allocator_a.freed_bytes >= 64 * 1024);
    for (0..10) |_| _ = try b.simulateSingleStep();
    try expect(body.getPosition().getY() < position_a.getY());
    try expect(bi_b.isActive(body.getID()));

    // B destroys it: everything of A goes back to A's allocator, B's allocator frees none of it
    const freed_b = allocator_b.freed_bytes;
    bi_b.removeBody(body.getID());
    bi_b.destroyBody(body.getID());
    try expectEqual(allocator_a.allocated_bytes, allocator_a.freed_bytes);
    try expectEqual(freed_b, allocator_b.freed_bytes);
    try expectEqual(@as(u32, 0), b.getSystem().getNumBodies());
}
