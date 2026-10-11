//! Port of: UnitTests/Physics/PhysicsTests.cpp
//! Status: partial
//! Missing: the cases that need contacts, the narrow phase query or the constraint types (Wave C, after Waves A and B):
//! TestPhysicsConstraintUserData (PointConstraint, Phase 6), TestPhysicsCollisionElastic,
//! TestPhysicsCollisionPartiallyElastic, TestPhysicsCollisionElasticDynamic, TestPhysicsCollisionInelastic,
//! TestMinVelocityForRestitution, TestPhysicsPenetrationSlop1, TestPhysicsPenetrationSlop2, TestPhysicsPenetrationSlop3,
//! TestPhysicsOutsideOfSpeculativeContactDistance, TestPhysicsInsideSpeculativeContactDistanceNoRestitution,
//! TestPhysicsInsideSpeculativeContactDistanceWithRestitution, TestPhysicsInsideSpeculativeContactDistanceNoHit,
//! TestPhysicsInsideSpeculativeContactDistanceSensor, TestPhysicsInsideSpeculativeContactDistanceMovingAway,
//! TestPhysicsActivationDeactivation, TestPhysicsActivateDuringStep, TestPhysicsBroadPhaseLayers,
//! TestAddSingleBodies (NarrowPhaseQuery::CastRay, A7), TestOutOfContactConstraints, TestFriction,
//! TestAngularFriction, TestAllowedDOFsVsCollision, TestAllowedDOFsVsSmallRotation, TestSelectiveStateSaveAndRestore,
//! TestMultiPartRestoreState, TestSwitchUseManifoldReduction, TestContactsVsSimulationIslands,
//! TestApplyBodyCreationSettings (they need ProcessBodyPair, REGION R2, and the ContactConstraintManager, A4).
//!
//! The ported cases run on the gravity-only path of PhysicsSystem (F2): no body pair reaches the narrow phase.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const math = zolt.math;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const RMat44 = zolt.RMat44;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyIDVector = zolt.BodyIDVector;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyLockRead = zolt.BodyLockRead;
const BodyLockWrite = zolt.BodyLockWrite;
const BodyLockMultiRead = zolt.BodyLockMultiRead;
const BodyLockMultiWrite = zolt.BodyLockMultiWrite;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShape = zolt.SphereShape;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const AllowedDOFs = zolt.AllowedDOFs;
const UnitTestRandom = fw.UnitTestRandom;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const checkApproxEqual = fw.checkApproxEqual;

/// Gravity vector
const gravity = Vec3.init(0.0, -9.81, 0.0);

/// RVec3 + Vec3 (Jolt's DVec3 operator + (Vec3) in double precision)
fn addR(a: RVec3, b: Vec3) RVec3 {
    return if (Core.double_precision) a.addVec3(b) else a.add(b);
}

/// RVec3(Vec3)
fn toR(v: Vec3) RVec3 {
    return if (Core.double_precision) RVec3.fromVec3(v) else v;
}

/// RMat44 * Mat44
fn mulRMat44(a: RMat44, b: Mat44) RMat44 {
    return if (Core.double_precision) a.mulMat44(b) else a.mul(b);
}

// Test the test framework's helper functions
test "TestPhysicsTestContext" {
    // Test that the Symplectic Euler integrator is close enough to the real value
    const simulation_time: f32 = 2.0;

    // For position: x = x0 + v0 * t + 1/2 * a * t^2
    const initial_pos = RVec3.init(0.0, 10.0, 0.0);
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();
    const simulated_pos = c.predictPosition(initial_pos, Vec3.zero(), gravity, simulation_time);
    const integrated_position = addR(initial_pos, gravity.mulScalar(0.5).mulScalar(math.square(simulation_time)));
    try checkApproxEqual(integrated_position, simulated_pos, .{ .tolerance = 0.2 });

    // For rotation
    const initial_rot = Quat.rotation(Vec3.axisY(), 0.1);
    const angular_acceleration = Vec3.init(0.0, 2.0, 0.0);
    const simulated_rot = c.predictOrientation(initial_rot, Vec3.zero(), angular_acceleration, simulation_time);
    const integrated_acceleration = angular_acceleration.mulScalar(0.5).mulScalar(math.square(simulation_time));
    const integrated_acceleration_len = integrated_acceleration.length();
    const integrated_rot = Quat.rotation(integrated_acceleration.divScalar(integrated_acceleration_len), integrated_acceleration_len).mul(initial_rot);
    try checkApproxEqual(integrated_rot, simulated_rot, .{ .tolerance = 0.02 });
}

test "TestPhysicsBodyLock" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();

    // Check that we cannot lock the invalid body ID
    {
        var lock: BodyLockRead = .init(c.getSystem().getBodyLockInterface(), .invalid);
        defer lock.deinit();
        try expect(!lock.succeeded());
        try expect(!lock.succeededAndIsInBroadPhase());
    }

    var body1_id: BodyID = undefined;
    {
        // Create a box
        const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, 0, Vec3.one(), .{});
        body1_id = body1.getID();
        try expect(body1_id.getIndex() == 0);
        try expect(body1_id.getSequenceNumber() == 1);

        // Create another box
        const body2 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, 0, Vec3.one(), .{});
        const body2_id = body2.getID();
        try expect(body2_id.getIndex() == 1);
        try expect(body2_id.getSequenceNumber() == 1);

        // Check that we can lock the first box
        {
            var lock1: BodyLockRead = .init(c.getSystem().getBodyLockInterface(), body1_id);
            defer lock1.deinit();
            try expect(lock1.succeeded());
            try expect(lock1.succeededAndIsInBroadPhase());

            // Unlock automatically on going out of scope
        }

        // Check that we can lock the first box
        {
            var lock1: BodyLockRead = .init(c.getSystem().getBodyLockInterface(), body1_id);
            defer lock1.deinit();
            try expect(lock1.succeeded());
            try expect(lock1.succeededAndIsInBroadPhase());

            // Release the lock early
            lock1.releaseLock();
            try expect(!lock1.succeeded());
            try expect(!lock1.succeededAndIsInBroadPhase());
        }

        // Remove the first box
        c.getSystem().getBodyInterfaceMut().removeBody(body1_id);

        // Check that we can lock the first box
        {
            var lock1: BodyLockWrite = .init(c.getSystem().getBodyLockInterface(), body1_id);
            defer lock1.deinit();
            try expect(lock1.succeeded());
            try expect(!lock1.succeededAndIsInBroadPhase());
        }

        // Destroy the first box
        c.getSystem().getBodyInterfaceMut().destroyBody(body1_id);

        // Check that we can not lock the body anymore
        {
            var lock1: BodyLockWrite = .init(c.getSystem().getBodyLockInterface(), body1_id);
            defer lock1.deinit();
            try expect(!lock1.succeeded());
            try expect(!lock1.succeededAndIsInBroadPhase());
        }
    }

    // Create another box
    const body3 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, 0, Vec3.one(), .{});
    const body3_id = body3.getID();
    try expect(body3_id.getIndex() == 0); // Check index reused
    try expect(body3_id.getSequenceNumber() == 2); // Check sequence number changed

    // Check that we can lock it
    {
        var lock3: BodyLockRead = .init(c.getSystem().getBodyLockInterface(), body3_id);
        defer lock3.deinit();
        try expect(lock3.succeeded());
        try expect(lock3.succeededAndIsInBroadPhase());
    }

    // Check that we can't lock the old body with the same body index anymore
    {
        var lock1: BodyLockRead = .init(c.getSystem().getBodyLockInterface(), body1_id);
        defer lock1.deinit();
        try expect(!lock1.succeeded());
        try expect(!lock1.succeededAndIsInBroadPhase());
    }
}

test "TestPhysicsBodyLockMulti" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();

    // Check that we cannot lock the invalid body ID
    {
        const bodies = [_]BodyID{ .invalid, .invalid };
        var lock: BodyLockMultiRead = .init(c.getSystem().getBodyLockInterface(), &bodies);
        defer lock.deinit();
        try expect(lock.getBody(0) == null);
        try expect(lock.getBody(1) == null);
    }

    {
        // Create two bodies
        const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, 0, Vec3.one(), .{});
        const body2 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, 0, Vec3.one(), .{});
        const bodies = [_]BodyID{ body1.getID(), body2.getID() };

        {
            // Lock the bodies
            var lock: BodyLockMultiWrite = .init(c.getSystem().getBodyLockInterface(), &bodies);
            defer lock.deinit();
            try expect(lock.getBody(0) == body1);
            try expect(lock.getBody(1) == body2);

            // Unlock automatically on going out of scope
        }

        {
            // Lock the bodies
            var lock: BodyLockMultiWrite = .init(c.getSystem().getBodyLockInterface(), &bodies);
            defer lock.deinit();
            try expect(lock.getNumBodies() == 2);
            try expect(lock.getBody(0) == body1);
            try expect(lock.getBody(1) == body2);

            // Release the locks early
            lock.releaseLocks();
            try expect(lock.getNumBodies() == 0);
        }

        // Destroy body 1
        c.getSystem().getBodyInterfaceMut().removeBody(bodies[0]);
        c.getSystem().getBodyInterfaceMut().destroyBody(bodies[0]);

        {
            // Lock the bodies
            var lock: BodyLockMultiRead = .init(c.getSystem().getBodyLockInterface(), &bodies);
            defer lock.deinit();
            try expect(lock.getBody(0) == null);
            try expect(lock.getBody(1) == body2);
        }
    }
}

test "TestPhysicsBodyID" {
    {
        const body_id = BodyID.init(0);
        try expect(body_id.getIndex() == 0);
        try expect(body_id.getSequenceNumber() == 0);
    }

    {
        const body_id = BodyID.init(~BodyID.broad_phase_bit);
        try expect(body_id.getIndex() == BodyID.max_body_index);
        try expect(body_id.getSequenceNumber() == BodyID.max_sequence_number);
    }
}

test "TestPhysicsBodyIDSequenceNumber" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Create a body and check it's id
    const body0_id = (try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{})).getID();
    try expect(body0_id.eql(.fromIndexAndSequenceNumber(0, 1))); // Body 0, sequence number 1

    // Check that the sequence numbers aren't reused until after 256 iterations
    var seq_no: u32 = 1;
    while (seq_no < 258) : (seq_no += 1) {
        const body1_id = (try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{})).getID();
        try expect(body1_id.eql(.fromIndexAndSequenceNumber(1, @truncate(seq_no)))); // Body 1

        bi.removeBody(body1_id);
        bi.destroyBody(body1_id);
    }

    bi.removeBody(body0_id);
    bi.destroyBody(body0_id);
}

test "TestPhysicsBodyIDOverride" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Dummy creation settings
    var bc = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer bc.deinit();

    // Create a body
    var b1 = (try bi.createBody(&bc)).?;
    try expect(b1.getID().eql(.fromIndexAndSequenceNumber(0, 1)));

    // Create body with same ID and same sequence number
    var b2 = try bi.createBodyWithID(.fromIndexAndSequenceNumber(0, 1), &bc);
    try expect(b2 == null);

    // Create body with same ID and different sequence number
    b2 = try bi.createBodyWithID(.fromIndexAndSequenceNumber(0, 2), &bc);
    try expect(b2 == null);

    // Destroy the first body
    bi.destroyBody(b1.getID());

    // Create a body with a custom ID and a high sequence number
    b1 = (try bi.createBodyWithID(.fromIndexAndSequenceNumber(0, 10), &bc)).?;
    try expect(b1.getID().eql(.fromIndexAndSequenceNumber(0, 10)));

    // Destroy it again
    bi.destroyBody(b1.getID());

    // Create another body in the normal way and check that we reuse the body index but that its sequence number got updated
    b1 = (try bi.createBody(&bc)).?;
    try expect(b1.getID().eql(.fromIndexAndSequenceNumber(0, 11)));

    // Create body with different ID (leave 1 open slot)
    const b2_body = try bi.createBodyWithoutID(&bc); // Using syntax that allows separation of allocation and assigning an ID
    try expect(b2_body.getID().isInvalid());
    _ = bi.assignBodyIDWithID(b2_body, .fromIndexAndSequenceNumber(2, 1));
    try expect(b2_body.getID().eql(.fromIndexAndSequenceNumber(2, 1)));

    // Create another body and check that the open slot is returned
    const b3 = (try bi.createBody(&bc)).?;
    try expect(b3.getID().eql(.fromIndexAndSequenceNumber(1, 1)));

    // Create another body and check that we do not hand out the body with specified ID
    var b4 = (try bi.createBody(&bc)).?;
    try expect(b4.getID().eql(.fromIndexAndSequenceNumber(3, 1)));

    // Delete and recreate body 4
    try expect((try bi.createBodyWithID(.fromIndexAndSequenceNumber(3, 1), &bc)) == null);
    bi.destroyBody(b4.getID());
    b4 = (try bi.createBodyWithID(.fromIndexAndSequenceNumber(3, 1), &bc)).?;
    try expect(b4.getID().eql(.fromIndexAndSequenceNumber(3, 1)));

    // Destroy 1st body
    try expect(bi.unassignBodyID(b1.getID()) == b1); // Use syntax that allows separation of unassigning and deallocation
    try expect(b1.getID().isInvalid());
    bi.destroyBodyWithoutID(b1);

    // Clean up remaining bodies
    bi.destroyBody(b2_body.getID());
    bi.destroyBody(b3.getID());
    bi.destroyBody(b4.getID());

    // Recreate body 1
    b1 = (try bi.createBodyWithID(.fromIndexAndSequenceNumber(0, 1), &bc)).?;
    try expect(b1.getID().eql(.fromIndexAndSequenceNumber(0, 1)));

    // Destroy last body
    bi.destroyBody(b1.getID());
}

test "TestPhysicsBodyUserData" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Create a body and pass user data through the creation settings
    var body_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer body_settings.deinit();
    body_settings.user_data = 0x1234567887654321;
    const body = (try bi.createBody(&body_settings)).?;
    try expect(body.getUserData() == 0x1234567887654321);

    // Change the user data
    body.setUserData(0x5678123443218765);
    try expect(body.getUserData() == 0x5678123443218765);

    // Convert back to body settings
    var body_settings2 = body.getBodyCreationSettings();
    defer body_settings2.deinit();
    try expect(body_settings2.user_data == 0x5678123443218765);
}

test "TestPhysicsPosition" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Translate / rotate the box
    const box_pos = Vec3.init(1, 2, 3);
    const box_rotation = Quat.rotation(Vec3.axisX(), 0.25 * math.pi);

    // Translate / rotate the body
    const body_pos = RVec3.init(4, 5, 6);
    const body_rotation = Quat.rotation(Vec3.axisY(), 0.3 * math.pi);
    const body_transform = RMat44.rotationTranslation(body_rotation, body_pos);
    const com_transform = mulRMat44(body_transform, Mat44.translation(box_pos));

    // Create body
    const rt_settings = try RotatedTranslatedShapeSettings.createPtr(allocator, box_pos, box_rotation, (try BoxShape.create(allocator, Vec3.one(), .{})).asShape());
    var body_settings = BodyCreationSettings.initWithSettings(rt_settings.asShapeSettings(), body_pos, body_rotation, .static, Layers.non_moving);
    defer body_settings.deinit();
    const body = (try bi.createBody(&body_settings)).?;

    // Check that the correct positions / rotations are reported
    try checkApproxEqual(body.getPosition(), body_pos, .{});
    try checkApproxEqual(body.getRotation(), body_rotation, .{});
    try checkApproxEqual(body.getWorldTransform(), body_transform, .{});
    try checkApproxEqual(body.getCenterOfMassPosition(), com_transform.getTranslation(), .{});
    try checkApproxEqual(body.getCenterOfMassTransform(), com_transform, .{});
    try checkApproxEqual(body.getInverseCenterOfMassTransform(), com_transform.inversedRotationTranslation(), .{ .tolerance = 1.0e-5 });
}

test "TestPhysicsOverrideMassAndInertia" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    const density: f32 = 1234.0;
    const box_extent = Vec3.init(2.0, 4.0, 6.0);
    const expected_mass = box_extent.getX() * box_extent.getY() * box_extent.getZ() * density;
    // See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    const squared_extents = Vec3.init(math.square(box_extent.getY()) + math.square(box_extent.getZ()), math.square(box_extent.getX()) + math.square(box_extent.getZ()), math.square(box_extent.getX()) + math.square(box_extent.getY()));
    const expected_inertia_diagonal = squared_extents.mulScalar(expected_mass / 12.0);

    const shape_settings = try BoxShapeSettings.create(allocator, box_extent.mulScalar(0.5), .{});
    shape_settings.base.setDensity(density);

    var body_settings = BodyCreationSettings.initWithSettings(shape_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer body_settings.deinit();

    // Create body as is
    const b1 = (try bi.createBody(&body_settings)).?;
    try checkApproxEqual(b1.getMotionProperties().getInverseMass(), 1.0 / expected_mass, .{});
    try checkApproxEqual(b1.getMotionProperties().getInertiaRotation(), Quat.identity(), .{});
    try checkApproxEqual(b1.getMotionProperties().getInverseInertiaDiagonal(), expected_inertia_diagonal.reciprocal(), .{});

    // Scale the mass and check that the mass and inertia are correct
    const new_mass: f32 = 2.0;
    b1.getMotionPropertiesMut().scaleToMass(new_mass);
    const new_expected_inertia_diagonal = squared_extents.mulScalar(new_mass / 12.0);
    try checkApproxEqual(b1.getMotionProperties().getInverseMass(), 1.0 / new_mass, .{});
    try checkApproxEqual(b1.getMotionProperties().getInertiaRotation(), Quat.identity(), .{});
    try checkApproxEqual(b1.getMotionProperties().getInverseInertiaDiagonal(), new_expected_inertia_diagonal.reciprocal(), .{});

    // Override only the mass
    const overridden_mass: f32 = 13.0;
    const overridden_mass_inertia_diagonal = squared_extents.mulScalar(overridden_mass / 12.0);

    body_settings.override_mass_properties = .calculate_inertia;
    body_settings.mass_properties_override.mass = overridden_mass;
    const b2 = (try bi.createBody(&body_settings)).?;
    try checkApproxEqual(b2.getMotionProperties().getInverseMass(), 1.0 / overridden_mass, .{});
    try checkApproxEqual(b2.getMotionProperties().getInertiaRotation(), Quat.identity(), .{});
    try checkApproxEqual(b2.getMotionProperties().getInverseInertiaDiagonal(), overridden_mass_inertia_diagonal.reciprocal(), .{});

    // Override both the mass and inertia
    const overridden_inertia_diagonal = Vec3.init(3.0, 2.0, 1.0); // From big to small so that MassProperties::DecomposePrincipalMomentsOfInertia returns the same rotation as we put in
    const overridden_inertia_rotation = Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.1 * math.pi);

    body_settings.override_mass_properties = .mass_and_inertia_provided;
    body_settings.mass_properties_override.inertia = Mat44.rotationQuat(overridden_inertia_rotation).mul(Mat44.scaleVec3(overridden_inertia_diagonal)).mul(Mat44.rotationQuat(overridden_inertia_rotation.inversed()));
    const b3 = (try bi.createBody(&body_settings)).?;
    try checkApproxEqual(b3.getMotionProperties().getInverseMass(), 1.0 / overridden_mass, .{});
    try checkApproxEqual(b3.getMotionProperties().getInertiaRotation(), overridden_inertia_rotation, .{});
    try checkApproxEqual(b3.getMotionProperties().getInverseInertiaDiagonal(), overridden_inertia_diagonal.reciprocal(), .{});
}

/// Test a box free falling under gravity
fn testPhysicsFreeFall(context: *PhysicsTestContext) !void {
    const initial_pos = RVec3.init(0.0, 10.0, 0.0);
    const simulation_time: f32 = 2.0;

    // Create box
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    try checkApproxEqual(initial_pos, body.getPosition(), .{});
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{});

    _ = try context.simulate(simulation_time, {});

    // Test resulting velocity (due to gravity)
    try checkApproxEqual(gravity.mulScalar(simulation_time), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    const expected_pos = context.predictPosition(initial_pos, Vec3.zero(), gravity, simulation_time);
    try checkApproxEqual(expected_pos, body.getPosition(), .{});
}

test "TestPhysicsFreeFall" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsFreeFall(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsFreeFall(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsFreeFall(&c4);
}

/// Test acceleration of a box with force applied
fn testPhysicsApplyForce(context: *PhysicsTestContext) !void {
    const initial_pos = RVec3.init(0.0, 10.0, 0.0);
    const acceleration = Vec3.init(2.0, 0.0, 0.0);
    const simulation_time: f32 = 2.0;

    // Create box
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    try checkApproxEqual(initial_pos, body.getPosition(), .{});
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{});

    // Validate mass
    const mass = math.cubed(@as(f32, 2.0)) * 1000.0; // Density * Volume
    try checkApproxEqual(1.0 / mass, body.getMotionProperties().getInverseMass(), .{});

    // Simulate while applying force
    const AddForce = struct {
        body: *Body,
        force: Vec3,

        pub fn call(self: @This()) !void {
            self.body.addForce(self.force);
        }
    };
    _ = try context.simulate(simulation_time, AddForce{ .body = body, .force = acceleration.mulScalar(mass) });

    // Test resulting velocity (due to gravity and applied force)
    try checkApproxEqual(gravity.add(acceleration).mulScalar(simulation_time), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    const expected_pos = context.predictPosition(initial_pos, Vec3.zero(), gravity.add(acceleration), simulation_time);
    try checkApproxEqual(expected_pos, body.getPosition(), .{});
}

test "TestPhysicsApplyForce" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsApplyForce(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsApplyForce(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsApplyForce(&c4);
}

/// Test angular acceleration for a box by applying torque every frame
fn testPhysicsApplyTorque(context: *PhysicsTestContext) !void {
    const initial_pos = RVec3.init(0.0, 10.0, 0.0);
    const angular_acceleration = Vec3.init(0.0, 2.0, 0.0);
    const simulation_time: f32 = 2.0;

    // Create box
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    try checkApproxEqual(Quat.identity(), body.getRotation(), .{});
    try checkApproxEqual(Vec3.zero(), body.getAngularVelocity(), .{});

    // Validate mass and inertia
    const mass = math.cubed(@as(f32, 2.0)) * 1000.0; // Density * Volume
    try checkApproxEqual(1.0 / mass, body.getMotionProperties().getInverseMass(), .{});
    const inertia = mass * 8.0 / 12.0; // See: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    try checkApproxEqual(Mat44.scale(1.0 / inertia), body.getMotionProperties().getLocalSpaceInverseInertia(), .{});

    // Simulate while applying torque
    const AddTorque = struct {
        body: *Body,
        torque: Vec3,

        pub fn call(self: @This()) !void {
            self.body.addTorque(self.torque);
        }
    };
    _ = try context.simulate(simulation_time, AddTorque{ .body = body, .torque = angular_acceleration.mulScalar(inertia) });

    // Get resulting angular velocity
    try checkApproxEqual(angular_acceleration.mulScalar(simulation_time), body.getAngularVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting rotation
    const expected_rot = context.predictOrientation(Quat.identity(), Vec3.zero(), angular_acceleration, simulation_time);
    try checkApproxEqual(expected_rot, body.getRotation(), .{ .tolerance = 1.0e-4 });
}

test "TestPhysicsApplyTorque" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsApplyTorque(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsApplyTorque(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsApplyTorque(&c4);
}

/// Test a box moving kinematically
fn testPhysicsMoveKinematic(context: *PhysicsTestContext) !void {
    const num_steps = 60;

    // Create box
    const body = try context.createBox(RVec3.zero(), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    var random = UnitTestRandom.init(UnitTestRandom.default_seed);
    for (0..10) |_| {
        // Remember old position
        const old_rotation = body.getRotation();
        const old_position = body.getPosition();

        // Move to new spot
        const position = toR(Vec3.random(&random).mulScalar(5.0));
        const rotation = Quat.random(&random);
        context.getBodyInterface().moveKinematic(body.getID(), position, rotation, context.getDeltaTime() * num_steps);

        // Simulate cNumSteps
        for (0..num_steps) |j| {
            _ = try context.simulateSingleStep();

            // Check intermediate position
            const fraction = @as(f32, @floatFromInt(j + 1)) / @as(f32, num_steps);
            const expected_pos = old_position.add(position.sub(old_position).mulScalar(fraction));
            try checkApproxEqual(body.getPosition(), expected_pos, .{ .tolerance = 1.0e-4 });

            // Check intermediate rotation
            const expected_rot = old_rotation.slerp(rotation, fraction);
            try checkApproxEqual(body.getRotation(), expected_rot, .{ .tolerance = 1.0e-5 });
        }

        // Check arrived
        try checkApproxEqual(body.getPosition(), position, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(body.getRotation(), rotation, .{ .tolerance = 1.0e-5 });
    }
}

test "TestPhysicsMoveKinematic" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsMoveKinematic(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsMoveKinematic(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsMoveKinematic(&c4);
}

test "TestMultiplePhysicsSystems" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{});
    defer c1.deinit();
    c1.zeroGravity();
    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{});
    defer c2.deinit();
    c2.zeroGravity();

    const box1_position = RVec3.init(1.0, 2.0, 3.0);
    const box1 = try c1.createBox(box1_position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

    const box2_position = RVec3.init(4.0, 5.0, 6.0);
    const box2 = try c2.createBox(box2_position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

    const box1_velocity = Vec3.init(1.0, 0, 0);
    const box2_velocity = Vec3.init(2.0, 0, 0);
    {
        // This tests if we can lock bodies from multiple physics systems (normally locking 2 bodies at the same time without using BodyLockMultiWrite would trigger an assert)
        var lock1: BodyLockWrite = .init(c1.getSystem().getBodyLockInterface(), box1.getID());
        defer lock1.deinit();
        var lock2: BodyLockWrite = .init(c2.getSystem().getBodyLockInterface(), box2.getID());
        defer lock2.deinit();

        try expect(lock1.getBody().getPosition().eql(box1_position));
        try expect(lock2.getBody().getPosition().eql(box2_position));

        lock1.getBody().setLinearVelocity(box1_velocity);
        lock2.getBody().setLinearVelocity(box2_velocity);
    }

    const time: f32 = 1.0;
    _ = try c1.simulate(time, {});
    _ = try c2.simulate(time, {});

    {
        var lock1: BodyLockRead = .init(c1.getSystem().getBodyLockInterface(), box1.getID());
        defer lock1.deinit();
        var lock2: BodyLockRead = .init(c2.getSystem().getBodyLockInterface(), box2.getID());
        defer lock2.deinit();

        // Check that the bodies in the different systems updated correctly
        try checkApproxEqual(lock1.getBody().getPosition(), addR(box1_position, box1_velocity.mulScalar(time)), .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(lock2.getBody().getPosition(), addR(box2_position, box2_velocity.mulScalar(time)), .{ .tolerance = 1.0e-5 });
    }
}

test "TestOutOfBodies" {
    const allocator = std.testing.allocator;

    // Create a context with space for a single body
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0, .max_bodies = 1 });
    defer c.deinit();

    const bi = c.getBodyInterface();

    // First body
    var settings1 = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings1.deinit();
    const b1 = try bi.createBody(&settings1);
    try expect(b1 != null);

    // Second body should fail
    var settings2 = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings2.deinit();
    var b2 = try bi.createBody(&settings2);
    try expect(b2 == null);

    // Free first body
    bi.destroyBody(b1.?.getID());

    // Second body creation should succeed
    var settings3 = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings3.deinit();
    b2 = try bi.createBody(&settings3);
    try expect(b2 != null);

    // Clean up
    bi.destroyBody(b2.?.getID());
}

test "TestAllowedDOFs" {
    const allocator = std.testing.allocator;
    var allowed_dofs: u32 = 1;
    while (allowed_dofs <= 0b111111) : (allowed_dofs += 1) {
        // Create a context
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        c.zeroGravity();

        // Create box
        const initial_position = RVec3.init(1, 2, 3);
        const initial_rotation = Quat.rotation(Vec3.replicate(math.sqrt(@as(f32, 1.0 / 3.0))), math.degreesToRadians(20.0));
        const box_shape = try BoxShape.create(allocator, Vec3.init(0.3, 0.5, 0.7), .{});
        var box_shape_ref = zolt.RefConst(zolt.Shape).init(box_shape.asShape());
        defer box_shape_ref.deinit();
        var box_settings = BodyCreationSettings.initWithShape(box_shape.asShape(), initial_position, initial_rotation, .dynamic, Layers.moving);
        defer box_settings.deinit();
        box_settings.linear_damping = 0;
        box_settings.angular_damping = 0;
        box_settings.allowed_dofs = @bitCast(@as(u8, @intCast(allowed_dofs)));
        const box = (try c.getBodyInterface().createBody(&box_settings)).?;
        try c.getBodyInterface().addBody(box.getID(), .activate);

        // Apply a force and torque in 3D
        const force = Vec3.init(100000, 110000, 120000);
        box.addForce(force);
        const torque = Vec3.init(13000, 14000, 15000);
        box.addTorque(torque);

        // Simulate
        _ = try c.simulateSingleStep();

        // Cancel components that should not be allowed by the allowed DOFs
        var linear_lock = Vec3.one();
        var angular_lock = Vec3.one();
        var axis: u32 = 0;
        while (axis < 3) : (axis += 1) {
            if ((allowed_dofs & (@as(u32, 1) << @intCast(axis))) == 0)
                linear_lock.setComponent(axis, 0.0);

            if ((allowed_dofs & (@as(u32, 0b1000) << @intCast(axis))) == 0)
                angular_lock.setComponent(axis, 0.0);
        }

        // Check resulting linear velocity
        const mp = box_shape.asShape().getMassProperties();
        const expected_linear_velocity = linear_lock.mul(force.divScalar(mp.mass).mulScalar(c.getDeltaTime()));
        try expect((linear_lock.eql(Vec3.zero()) or expected_linear_velocity.length() > 1.0)); // Just to check that we applied a high enough force
        try checkApproxEqual(box.getLinearVelocity(), expected_linear_velocity, .{});
        const expected_position = addR(initial_position, expected_linear_velocity.mulScalar(c.getDeltaTime()));
        try checkApproxEqual(box.getPosition(), expected_position, .{});

        // Check resulting angular velocity
        var inv_inertia = Mat44.rotationQuat(initial_rotation).mul(mp.inertia.inversed3x3()).mul(Mat44.rotationQuat(initial_rotation.conjugated()));
        inv_inertia = Mat44.scaleVec3(angular_lock).mul(inv_inertia).mul(Mat44.scaleVec3(angular_lock)); // Clear row and column for locked axes
        const expected_angular_velocity = inv_inertia.mulVec3(torque).mulScalar(c.getDeltaTime());
        try expect((angular_lock.eql(Vec3.zero()) or expected_angular_velocity.length() > 1.0)); // Just to check that we applied a high enough torque
        try checkApproxEqual(box.getAngularVelocity(), expected_angular_velocity, .{});
        const expected_angular_velocity_len = expected_angular_velocity.length();
        const expected_rotation = if (expected_angular_velocity_len > 0.0) Quat.rotation(expected_angular_velocity.divScalar(expected_angular_velocity_len), expected_angular_velocity_len * c.getDeltaTime()).mul(initial_rotation) else initial_rotation;
        try checkApproxEqual(box.getRotation(), expected_rotation, .{});
    }
}

test "TestOptimizeBroadPhase" {
    const allocator = std.testing.allocator;
    const max_bodies = 128;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0, .max_bodies = max_bodies });
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Create max number of bodies
    var bodies: BodyIDVector = .empty;
    defer bodies.deinit(allocator);
    var bcs = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.moving);
    defer bcs.deinit();
    for (0..max_bodies) |_| {
        const b = try bi.createBody(&bcs);
        try expect(b != null);
        try bodies.append(allocator, b.?.getID());
    }

    // Repeatedly add and remove bodies
    for (0..10) |_| {
        const add_state = try bi.addBodiesPrepare(bodies.items);
        for (bodies.items) |id|
            try expect(!bi.isAdded(id));
        bi.addBodiesFinalize(bodies.items, add_state, .dont_activate);
        for (bodies.items) |id|
            try expect(bi.isAdded(id));

        bi.removeBodies(bodies.items);
        for (bodies.items) |id|
            try expect(!bi.isAdded(id));

        // Optimize the broad phase to recycle quad tree nodes
        c.getSystem().optimizeBroadPhase();
    }
}
