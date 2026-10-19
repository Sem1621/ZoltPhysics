//! Port of: UnitTests/Physics/PhysicsTests.cpp
//! Status: complete
//!
//! - `RVec3(..., 0.99f, ...)` (a float literal or float arithmetic) and `RVec3(..., 0.99_r, ...)` (a Real literal)
//!   differ in double precision: the float values are computed as f32 and widened (`rvec3f`), like in C++.
//! - The listeners check their contracts inside the callbacks; `checkError` reports a violation after the update
//!   (Jolt's CHECK inside a callback). They are unregistered before they go out of scope.
//! - ExpectAssert cannot be ported (Zig asserts panic): the step of TestOutOfContactConstraints that makes Jolt's final
//!   `JPH_ASSERT(errors == EPhysicsUpdateError::None)` fire only runs without asserts (ReleaseFast).
//! - TestAddSingleBodies: Jolt replaces the Trace function to count the "Stack full" messages of the QuadTree. Zolt
//!   logs through std.log, so the test checks the latch of the warning instead (Jolt: a function static, Zolt: one
//!   per broad phase, only with asserts): it is not set before the queries and set after them.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const BroadPhaseLayers = LayersFile.BroadPhaseLayers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;
const LoggingBodyActivationListener = @import("../LoggingBodyActivationListener.zig").LoggingBodyActivationListener;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const math = zolt.math;
const trigonometry = zolt.trigonometry;
const Real = zolt.Real;
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
const MotionType = zolt.MotionType;
const MotionProperties = zolt.MotionProperties;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShape = zolt.SphereShape;
const StaticCompoundShape = zolt.StaticCompoundShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const SubShapeID = zolt.SubShapeID;
const CollisionGroup = zolt.CollisionGroup;
const GroupFilterTable = zolt.GroupFilterTable;
const AllowedDOFs = zolt.AllowedDOFs;
const PhysicsSettings = zolt.PhysicsSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StateRecorderFilter = zolt.StateRecorderFilter;
const StateRecorderState = zolt.StateRecorderState;
const BroadPhase = zolt.BroadPhase;
const BroadPhaseQuadTree = zolt.BroadPhaseQuadTree;
const RRayCast = zolt.RRayCast;
const CastRayCollector = zolt.CastRayCollector;
const BodyPairCollector = zolt.BodyPairCollector;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const UnitTestRandom = fw.UnitTestRandom;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const checkApproxEqual = fw.checkApproxEqual;

/// Gravity vector
const gravity = Vec3.init(0.0, -9.81, 0.0);

/// RVec3(x, y, z) with float arguments (Jolt computes them as float and widens them in double precision)
fn rvec3f(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// RVec3 - Vec3
fn subR(a: RVec3, b: Vec3) RVec3 {
    return a.subVec3(b);
}

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

test "TestPhysicsConstraintUserData" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create a body
    const body = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{});

    // Create constraint with user data
    var constraint_settings = zolt.PointConstraintSettings.init(allocator);
    defer constraint_settings.asSettings().deinit();
    constraint_settings.base.base.user_data = 0x1234567887654321;
    var constraint = zolt.Ref(zolt.Constraint).init((try constraint_settings.createConstraint(allocator, body, zolt.Body.fixedToWorld())).asConstraint());
    defer constraint.deinit();
    try expect(constraint.get().?.getUserData() == 0x1234567887654321);

    // Change the user data
    constraint.get().?.setUserData(0x5678123443218765);
    try expect(constraint.get().?.getUserData() == 0x5678123443218765);

    // Convert back to constraint settings
    var constraint_settings2 = try constraint.get().?.getConstraintSettings();
    defer constraint_settings2.deinit();
    try expect(constraint_settings2.get().?.user_data == 0x5678123443218765);
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

/// Let a sphere bounce on the floor with restitution = 1
fn testPhysicsCollisionElastic(context: *PhysicsTestContext, comptime gravity_through_add_force: bool) !void {
    const simulation_time: f32 = 1.0;
    const distance_traveled = context.predictPosition(RVec3.zero(), Vec3.zero(), gravity, simulation_time);
    const floor_hit_epsilon: f32 = 1.0e-4; // Apply epsilon so that we're sure that the collision algorithm will find a collision
    const floor_hit_pos = rvec3f(0.0, 1.0 - floor_hit_epsilon, 0.0); // Sphere with radius 1 will hit floor when 1 above the floor
    const initial_pos = floor_hit_pos.sub(distance_traveled);

    // Cancel normal gravity and apply it through AddForce if requested
    if (gravity_through_add_force)
        context.zeroGravity();

    // Create sphere
    _ = try context.createFloor();
    const body = try context.createSphere(initial_pos, 1.0, .dynamic, .discrete, Layers.moving, .{});
    body.setRestitution(1.0);

    // Callback to apply gravity
    const ApplyCustomGravity = struct {
        body: *Body,

        pub fn call(self: @This()) !void {
            if (gravity_through_add_force) {
                try expect(self.body.getAccumulatedForce().eql(Vec3.zero())); // Should have been reset after the step
                self.body.addForce(gravity.divScalar(self.body.getMotionProperties().getInverseMass()));
            }
        }
    };
    const apply_custom_gravity: ApplyCustomGravity = .{ .body = body };

    // Simulate until at floor
    _ = try context.simulate(simulation_time, apply_custom_gravity);
    try checkApproxEqual(floor_hit_pos, body.getPosition(), .{});

    // Assert collision not yet processed
    try checkApproxEqual(gravity.mulScalar(simulation_time), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Simulate one more step to process the collision
    try apply_custom_gravity.call();
    _ = try context.simulateSingleStep();

    // Assert that collision is processed and velocity is reversed (which is required for a fully elastic collision).
    const sub_step_delta_time = context.getStepDeltaTime();
    const remaining_step_time = context.getDeltaTime() - context.getStepDeltaTime();
    const reflected_velocity_after_sub_step = gravity.mulScalar(-simulation_time);
    const reflected_velocity_after_full_step = reflected_velocity_after_sub_step.add(gravity.mulScalar(remaining_step_time));
    try checkApproxEqual(reflected_velocity_after_full_step, body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Body should have bounced back
    const pos_after_bounce_sub_step = addR(floor_hit_pos, reflected_velocity_after_sub_step.mulScalar(sub_step_delta_time));
    const pos_after_bounce_full_step = context.predictPosition(pos_after_bounce_sub_step, reflected_velocity_after_sub_step, gravity, remaining_step_time);
    try checkApproxEqual(pos_after_bounce_full_step, body.getPosition(), .{});

    // Simulate same time minus one step, with a fully elastic body we should reach the initial position again
    const expected_pos = context.predictPosition(pos_after_bounce_full_step, reflected_velocity_after_full_step, gravity, simulation_time - context.getDeltaTime());
    _ = try context.simulate(simulation_time - context.getDeltaTime(), apply_custom_gravity);
    try checkApproxEqual(expected_pos, body.getPosition(), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(expected_pos, initial_pos, .{ .tolerance = 1.0e-5 });

    // If we do one more step, we should be going down again
    const pre_step_pos = body.getPosition();
    try expect(body.getLinearVelocity().getY() > 0.0);
    try apply_custom_gravity.call();
    _ = try context.simulateSingleStep();
    try expect(body.getLinearVelocity().getY() < 1.0e-6);
    try expect(body.getPosition().getY() < pre_step_pos.getY() + 1.0e-6);
}

test "TestPhysicsCollisionElastic" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsCollisionElastic(&c1, false);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsCollisionElastic(&c2, false);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsCollisionElastic(&c4, false);

    var c5: PhysicsTestContext = undefined;
    try c5.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c5.deinit();
    try testPhysicsCollisionElastic(&c5, true);

    var c6: PhysicsTestContext = undefined;
    try c6.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c6.deinit();
    try testPhysicsCollisionElastic(&c6, true);

    var c7: PhysicsTestContext = undefined;
    try c7.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c7.deinit();
    try testPhysicsCollisionElastic(&c7, true);
}

// Let a sphere with restitution 0.9 bounce on the floor
test "TestPhysicsCollisionPartiallyElastic" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();
    _ = try c.createFloor();

    // Create sphere
    const initial_pos = RVec3.init(0, 10, 0);
    const restitution: f32 = 0.9;
    const radius: f32 = 2.0;
    const body = try c.createSphere(initial_pos, radius, .dynamic, .discrete, Layers.moving, .{});
    body.setRestitution(restitution);

    // Simple simulation to compare with the actual simulation
    var pos = initial_pos;
    var vel = Vec3.zero();
    const dt = c.getDeltaTime();
    const penetration_slop = c.getSystem().getPhysicsSettings().penetration_slop;
    for (0..1000) |_| {
        // Simple simulation
        const penetration: Real = radius - pos.getY();
        if (penetration > -penetration_slop and vel.getY() < 0.0)
            vel = vel.mulScalar(-restitution)
        else
            vel = vel.add(gravity.mulScalar(dt));
        pos = addR(pos, vel.mulScalar(dt));

        // Actual step
        _ = try c.simulateSingleStep();

        // Compare simulations
        try checkApproxEqual(pos, body.getPosition(), .{ .tolerance = 1.0e-5 });
        try checkApproxEqual(vel, body.getLinearVelocity(), .{ .tolerance = 1.0e-5 });
    }
}

/// 2 spheres bounce with restitution = 1, tests we don't correct for gravity in a perpendicular direction to gravity
fn testPhysicsCollisionElasticDynamic(context: *PhysicsTestContext) !void {
    // Create spheres
    const sphere1 = try context.createSphere(RVec3.init(-2, 0, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere1.setRestitution(1.0);
    sphere1.setLinearVelocity(Vec3.init(5, 0, 0));

    const sphere2 = try context.createSphere(RVec3.init(2, 0, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere2.setRestitution(1.0);
    sphere2.setLinearVelocity(Vec3.init(-10, 0, 0));

    // Simulate
    const simulation_time: f32 = 1.0;
    _ = try context.simulate(simulation_time, {});

    // Check that velocities match that of a fully elastic collision
    try checkApproxEqual(Vec3.init(-10, 0, 0).add(gravity.mulScalar(simulation_time)), sphere1.getLinearVelocity(), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(Vec3.init(5, 0, 0).add(gravity.mulScalar(simulation_time)), sphere2.getLinearVelocity(), .{ .tolerance = 1.0e-5 });
}

test "TestPhysicsCollisionElasticDynamic" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsCollisionElasticDynamic(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsCollisionElasticDynamic(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsCollisionElasticDynamic(&c4);
}

/// Let a sphere bounce on the floor with restitution = 0
fn testPhysicsCollisionInelastic(context: *PhysicsTestContext) !void {
    const simulation_time: f32 = 1.0;
    const distance_traveled = context.predictPosition(RVec3.zero(), Vec3.zero(), gravity, simulation_time);
    const floor_hit_epsilon: f32 = 1.0e-4; // Apply epsilon so that we're sure that the collision algorithm will find a collision
    const floor_hit_pos = rvec3f(0.0, 1.0 - floor_hit_epsilon, 0.0); // Sphere with radius 1 will hit floor when 1 above the floor
    const initial_pos = floor_hit_pos.sub(distance_traveled);

    // Create sphere
    _ = try context.createFloor();
    const body = try context.createSphere(initial_pos, 1.0, .dynamic, .discrete, Layers.moving, .{});
    body.setRestitution(0.0);

    // Simulate until at floor
    _ = try context.simulate(simulation_time, {});
    try checkApproxEqual(floor_hit_pos, body.getPosition(), .{});

    // Assert collision not yet processed
    try checkApproxEqual(gravity.mulScalar(simulation_time), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Simulate one more step to process the collision
    _ = try context.simulateSingleStep();

    // Assert that all velocity was lost in the collision
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Assert that we're on the floor
    try checkApproxEqual(floor_hit_pos, body.getPosition(), .{ .tolerance = 1.0e-4 });

    // Simulate some more to validate that we remain on the floor
    _ = try context.simulate(simulation_time, {});
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(floor_hit_pos, body.getPosition(), .{ .tolerance = 1.0e-4 });
}

test "TestPhysicsCollisionInelastic" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsCollisionInelastic(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsCollisionInelastic(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsCollisionInelastic(&c4);
}

test "TestMinVelocityForRestitution" {
    for (0..2) |i| {
        // Create a context
        var c: PhysicsTestContext = undefined;
        try c.init(std.testing.allocator, .{});
        defer c.deinit();
        c.zeroGravity();

        const sphere1 = try c.createSphere(RVec3.init(0, -2, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
        sphere1.setRestitution(1.0);
        sphere1.setLinearVelocity(Vec3.init(0, 1, 0));

        const sphere2 = try c.createSphere(RVec3.init(0, 2, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
        sphere2.setRestitution(1.0);
        sphere2.setLinearVelocity(Vec3.zero());

        var expected1: Vec3 = undefined;
        var expected2: Vec3 = undefined;
        var s: PhysicsSettings = c.getSystem().getPhysicsSettings().*;
        if (i == 0) {
            // Make the minimum velocity for restitution bigger than the speed of the sphere
            s.min_velocity_for_restitution = 1.01;

            // Non elastic collision will make both spheres move at half speed
            expected1 = sphere1.getLinearVelocity().mulScalar(0.5);
            expected2 = expected1;
        } else {
            // Make the minimum velocity for restitution smaller than the speed of the sphere
            s.min_velocity_for_restitution = 0.99;

            // Elastic collision will transfer all velocity to sphere 2
            expected1 = Vec3.zero();
            expected2 = sphere1.getLinearVelocity();
        }
        c.getSystem().setPhysicsSettings(&s);

        _ = try c.simulate(2.5, {});

        try checkApproxEqual(sphere1.getLinearVelocity(), expected1, .{});
        try checkApproxEqual(sphere2.getLinearVelocity(), expected2, .{});
    }
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

/// Let box intersect with floor by cPenetrationSlop. It should not move, this is the maximum penetration allowed.
fn testPhysicsPenetrationSlop1(context: *PhysicsTestContext) !void {
    const penetration_slop = context.getSystem().getPhysicsSettings().penetration_slop;
    const simulation_time: f32 = 1.0;
    const initial_pos = rvec3f(0.0, 1.0 - penetration_slop, 0.0);

    // Create box, penetrating with floor
    _ = try context.createFloor();
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Simulate
    _ = try context.simulate(simulation_time, {});

    // Test slop not resolved
    try checkApproxEqual(initial_pos, body.getPosition(), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{});
    try checkApproxEqual(Vec3.zero(), body.getAngularVelocity(), .{});
}

test "TestPhysicsPenetrationSlop1" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsPenetrationSlop1(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsPenetrationSlop1(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsPenetrationSlop1(&c4);
}

/// Let box intersect with floor with more than cPenetrationSlop. It should be resolved by SolvePositionConstraint until interpenetration is cPenetrationSlop.
fn testPhysicsPenetrationSlop2(context: *PhysicsTestContext) !void {
    const penetration_slop = context.getSystem().getPhysicsSettings().penetration_slop;
    const simulation_time: f32 = 1.0;
    const initial_pos = rvec3f(0.0, 1.0 - 2.0 * penetration_slop, 0.0);
    const final_pos = rvec3f(0.0, 1.0 - penetration_slop, 0.0);

    // Create box, penetrating with floor
    _ = try context.createFloor();
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Simulate
    _ = try context.simulate(simulation_time, {});

    // Test resolved until slop
    try checkApproxEqual(final_pos, body.getPosition(), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{});
    try checkApproxEqual(Vec3.zero(), body.getAngularVelocity(), .{});
}

test "TestPhysicsPenetrationSlop2" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsPenetrationSlop2(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsPenetrationSlop2(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsPenetrationSlop2(&c4);
}

/// Let box intersect with floor with less than cPenetrationSlop. Body should not move because SolveVelocityConstraint should reset velocity.
fn testPhysicsPenetrationSlop3(context: *PhysicsTestContext) !void {
    const penetration_slop = context.getSystem().getPhysicsSettings().penetration_slop;
    const simulation_time: f32 = 1.0;
    const initial_pos = rvec3f(0.0, 1.0 - 0.1 * penetration_slop, 0.0);

    // Create box, penetrating with floor
    _ = try context.createFloor();
    const body = try context.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Simulate
    _ = try context.simulate(simulation_time, {});

    // Test body remained static
    try checkApproxEqual(initial_pos, body.getPosition(), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(Vec3.zero(), body.getLinearVelocity(), .{});
    try checkApproxEqual(Vec3.zero(), body.getAngularVelocity(), .{});
}

test "TestPhysicsPenetrationSlop3" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    defer c1.deinit();
    try testPhysicsPenetrationSlop3(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    defer c2.deinit();
    try testPhysicsPenetrationSlop3(&c2);

    var c4: PhysicsTestContext = undefined;
    try c4.init(allocator, .{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
    defer c4.deinit();
    try testPhysicsPenetrationSlop3(&c4);
}

/// A PhysicsTestContext with a LoggingContactListener and a LoggingBodyActivationListener (in place). The listeners
/// are registered by the test (Jolt registers them after creating the context, sometimes after creating bodies) and
/// unregistered in `deinit`.
const ListenerContext = struct {
    c: PhysicsTestContext,
    contact_listener: LoggingContactListener,
    activation_listener: LoggingBodyActivationListener,

    fn init(self: *ListenerContext, opts: PhysicsTestContext.Options) !void {
        const allocator = std.testing.allocator;
        try self.c.init(allocator, opts);
        self.contact_listener = .init(allocator);
        self.activation_listener = .init(allocator);
    }

    fn deinit(self: *ListenerContext) void {
        self.c.getSystem().setContactListener(null);
        self.c.getSystem().setBodyActivationListener(null);
        self.c.deinit();
        self.contact_listener.deinit();
        self.activation_listener.deinit();
    }

    /// c.GetSystem()->SetContactListener(&contact_listener)
    fn registerContactListener(self: *ListenerContext) void {
        self.c.getSystem().setContactListener(.init(&self.contact_listener));
    }

    /// c.GetSystem()->SetBodyActivationListener(&activation_listener)
    fn registerActivationListener(self: *ListenerContext) void {
        self.c.getSystem().setBodyActivationListener(.init(&self.activation_listener));
    }

    /// Report a CHECK that failed inside a callback (or a log entry that could not be stored)
    fn checkListeners(self: *const ListenerContext) !void {
        try self.contact_listener.checkError();
        try self.activation_listener.checkError();
    }

    /// SimulateSingleStep, then the checks made inside the callbacks
    fn simulateSingleStep(self: *ListenerContext) !zolt.PhysicsUpdateError {
        const errors = try self.c.simulateSingleStep();
        try self.checkListeners();
        return errors;
    }

    /// Simulate, then the checks made inside the callbacks
    fn simulate(self: *ListenerContext, total_time: f32, pre_step_callback: anytype) !zolt.PhysicsUpdateError {
        const errors = try self.c.simulate(total_time, pre_step_callback);
        try self.checkListeners();
        return errors;
    }
};

test "TestPhysicsOutsideOfSpeculativeContactDistance" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    c.zeroGravity();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a box and a sphere just outside the speculative contact distance
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const distance_above_floor = 1.1 * speculative_contact_distance;
    const initial_pos_box = rvec3f(0, 1.0 + distance_above_floor, 0.0);
    const initial_pos_sphere = addR(initial_pos_box, Vec3.init(5, 0, 0));

    // Make it move 1 m per step down
    const velocity = Vec3.init(0, -1.0 / c.getDeltaTime(), 0);

    const box = try c.createBox(initial_pos_box, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    box.setLinearVelocity(velocity);

    const sphere = try c.createSphere(initial_pos_sphere, 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere.setLinearVelocity(velocity);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that it is now penetrating the floor (collision should not have been detected as it is a discrete body and there was no collision initially)
    try expect(contact_listener.getEntryCount() == 0);
    try checkApproxEqual(box.getPosition(), addR(initial_pos_box, velocity.mulScalar(c.getDeltaTime())), .{});
    try checkApproxEqual(sphere.getPosition(), addR(initial_pos_sphere, velocity.mulScalar(c.getDeltaTime())), .{});

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that the contacts are detected now
    try expect(contact_listener.getEntryCount() == 4); // 2 validates and 2 contacts
    try expect(contact_listener.contains(.validate, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
}

test "TestPhysicsInsideSpeculativeContactDistanceNoRestitution" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    c.zeroGravity();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a box and a sphere just inside the speculative contact distance
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const distance_above_floor = 0.9 * speculative_contact_distance;
    const initial_pos_box = rvec3f(0, 1.0 + distance_above_floor, 0.0);
    const initial_pos_sphere = addR(initial_pos_box, Vec3.init(5, 0, 0));

    // Make it move 1 m per step down
    const velocity = Vec3.init(0, -1.0 / c.getDeltaTime(), 0);

    const box = try c.createBox(initial_pos_box, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    box.setLinearVelocity(velocity);

    const sphere = try c.createSphere(initial_pos_sphere, 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere.setLinearVelocity(velocity);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that it is now on the floor and that 2 collisions have been detected
    try expect(contact_listener.getEntryCount() == 4); // 2 validates and 2 contacts
    try expect(contact_listener.contains(.validate, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // Velocity should have been reduced to exactly hit the floor in this step
    const expected_velocity = Vec3.init(0, -distance_above_floor / c.getDeltaTime(), 0);

    // Box collision is less accurate than sphere as it hits with 4 corners so there's some floating point precision loss in the calculation
    try checkApproxEqual(box.getPosition(), RVec3.init(0, 1, 0), .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(box.getLinearVelocity(), expected_velocity, .{ .tolerance = 0.05 });
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-2 });

    // Sphere has only 1 contact point so is much more accurate
    try checkApproxEqual(sphere.getPosition(), RVec3.init(5, 1, 0), .{});
    try checkApproxEqual(sphere.getLinearVelocity(), expected_velocity, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(sphere.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-4 });

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that the contacts persisted
    try expect(contact_listener.getEntryCount() >= 2); // 2 persist and possibly 2 validates depending on if the cache got reused
    try expect(contact_listener.contains(.persist, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.persist, sphere.getID(), floor.getID()));

    // Box should have come to rest
    try checkApproxEqual(box.getPosition(), RVec3.init(0, 1, 0), .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(box.getLinearVelocity(), Vec3.zero(), .{ .tolerance = 0.05 });
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-2 });

    // Sphere should have come to rest
    try checkApproxEqual(sphere.getPosition(), RVec3.init(5, 1, 0), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(sphere.getLinearVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(sphere.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-4 });
}

test "TestPhysicsInsideSpeculativeContactDistanceWithRestitution" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    c.zeroGravity();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a box and a sphere just inside the speculative contact distance
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const distance_above_floor = 0.9 * speculative_contact_distance;
    const initial_pos_box = rvec3f(0, 1.0 + distance_above_floor, 0.0);
    const initial_pos_sphere = addR(initial_pos_box, Vec3.init(5, 0, 0));

    // Make it move 1 m per step down
    const velocity = Vec3.init(0, -1.0 / c.getDeltaTime(), 0);

    const box = try c.createBox(initial_pos_box, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    box.setLinearVelocity(velocity);
    box.setRestitution(1.0);

    const sphere = try c.createSphere(initial_pos_sphere, 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere.setLinearVelocity(velocity);
    sphere.setRestitution(1.0);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that it has triggered contact points and has bounced from it's initial position (effectively traveling the extra distance to the floor and back for free)
    try expect(contact_listener.getEntryCount() == 4); // 2 validates and 2 contacts
    try expect(contact_listener.contains(.validate, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // Box collision is less accurate than sphere as it hits with 4 corners so there's some floating point precision loss in the calculation
    try checkApproxEqual(box.getPosition(), subR(initial_pos_box, velocity.mulScalar(c.getDeltaTime())), .{ .tolerance = 0.01 });
    try checkApproxEqual(box.getLinearVelocity(), velocity.negate(), .{ .tolerance = 0.1 });
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 0.02 });

    // Sphere has only 1 contact point so is much more accurate
    try checkApproxEqual(sphere.getPosition(), subR(initial_pos_sphere, velocity.mulScalar(c.getDeltaTime())), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(sphere.getLinearVelocity(), velocity.negate(), .{ .tolerance = 2.0e-4 });
    try checkApproxEqual(sphere.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 2.0e-4 });

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that all contact points are removed
    try expect(contact_listener.getEntryCount() == 2); // 2 removes
    try expect(contact_listener.contains(.remove, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.remove, sphere.getID(), floor.getID()));
}

test "TestPhysicsInsideSpeculativeContactDistanceNoHit" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    floor.setRestitution(1.0);
    c.zeroGravity();

    // Turn off the minimum velocity for restitution, our velocity is lower than the default
    var settings: PhysicsSettings = c.getSystem().getPhysicsSettings().*;
    settings.min_velocity_for_restitution = 0.0;
    c.getSystem().setPhysicsSettings(&settings);

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a sphere inside speculative contact distance from the ground
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const distance_above_floor = 0.9 * speculative_contact_distance;
    const initial_pos_sphere = rvec3f(0, 1.0 + distance_above_floor, 0.0);

    // Make it move slow enough so that it will not touch the floor in 1 time step
    const velocity = Vec3.init(0, -0.9 * distance_above_floor / c.getDeltaTime(), 0);

    const sphere = try c.createSphere(initial_pos_sphere, 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere.setLinearVelocity(velocity);
    sphere.setRestitution(1.0);
    sphere.getMotionPropertiesMut().setLinearDamping(0.0);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that it has triggered contact points from the speculative contacts
    try expect(contact_listener.getEntryCount() == 2);
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // Check that sphere didn't actually change velocity (it hasn't actually interacted with the floor, the speculative contact was not an actual contact)
    try expect(sphere.getLinearVelocity().eql(velocity));

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check again that it triggered contact points
    try expect(contact_listener.getEntryCount() == 2);
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.persist, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // It should have bounced back up and inverted velocity due to restitution being 1
    try checkApproxEqual(sphere.getLinearVelocity().negate(), velocity, .{});
}

test "TestPhysicsInsideSpeculativeContactDistanceSensor" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    c.zeroGravity();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a sphere sensor just inside the speculative contact distance
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const radius: f32 = 1.0;
    const distance_above_floor = 0.9 * speculative_contact_distance;
    const initial_pos_sphere = rvec3f(5, radius + distance_above_floor, 0);

    // Make it move 1 m per step down
    const velocity = Vec3.init(0, -1.0 / c.getDeltaTime(), 0);

    const sphere = try c.createSphere(initial_pos_sphere, radius, .dynamic, .discrete, Layers.moving, .{});
    sphere.setIsSensor(true);
    sphere.setLinearVelocity(velocity);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    try expect(contact_listener.getEntryCount() == 0); // We're inside the speculative contact distance but we're a sensor so we shouldn't trigger any contacts

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that we're now actually intersecting
    try expect(contact_listener.getEntryCount() == 2); // 1 validates and 1 contact
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // Sensor should not be affected by the floor
    try checkApproxEqual(sphere.getPosition(), addR(initial_pos_sphere, velocity.mulScalar(2.0 * c.getDeltaTime())), .{});
    try checkApproxEqual(sphere.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(sphere.getAngularVelocity(), Vec3.zero(), .{});
}

test "TestPhysicsInsideSpeculativeContactDistanceMovingAway" {
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    c.zeroGravity();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a box and a sphere just inside the speculative contact distance
    const speculative_contact_distance = c.getSystem().getPhysicsSettings().speculative_contact_distance;
    const distance_above_floor = 0.9 * speculative_contact_distance;
    const initial_pos_box = rvec3f(0, 1.0 + distance_above_floor, 0.0);
    const initial_pos_sphere = addR(initial_pos_box, Vec3.init(5, 0, 0));

    // Make it move 1 m per step up
    const velocity = Vec3.init(0, 1.0 / c.getDeltaTime(), 0);

    const box = try c.createBox(initial_pos_box, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    box.setLinearVelocity(velocity);
    box.setRestitution(1.0);

    const sphere = try c.createSphere(initial_pos_sphere, 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere.setLinearVelocity(velocity);
    sphere.setRestitution(1.0);

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that it has triggered contact points (note that this is wrong since the object never touched the floor but that's the downside of the speculative contacts -> you'll get an incorrect collision callback)
    try expect(contact_listener.getEntryCount() == 4); // 2 validates and 2 contacts
    try expect(contact_listener.contains(.validate, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.validate, sphere.getID(), floor.getID()));
    try expect(contact_listener.contains(.add, sphere.getID(), floor.getID()));
    contact_listener.clear();

    // Box should have moved unimpeded
    try checkApproxEqual(box.getPosition(), addR(initial_pos_box, velocity.mulScalar(c.getDeltaTime())), .{});
    try checkApproxEqual(box.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{});

    // Sphere should have moved unimpeded
    try checkApproxEqual(sphere.getPosition(), addR(initial_pos_sphere, velocity.mulScalar(c.getDeltaTime())), .{});
    try checkApproxEqual(sphere.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(sphere.getAngularVelocity(), Vec3.zero(), .{});

    // Simulate a step
    _ = try ctx.simulateSingleStep();

    // Check that all contact points are removed
    try expect(contact_listener.getEntryCount() == 2); // 2 removes
    try expect(contact_listener.contains(.remove, box.getID(), floor.getID()));
    try expect(contact_listener.contains(.remove, sphere.getID(), floor.getID()));
}

fn testPhysicsActivationDeactivation(opts: PhysicsTestContext.Options) !void {
    var ctx: ListenerContext = undefined;
    try ctx.init(opts);
    defer ctx.deinit();
    const context = &ctx.c;

    const penetration_slop = context.getSystem().getPhysicsSettings().penetration_slop;

    // Install activation listener
    const activation_listener = &ctx.activation_listener;
    ctx.registerActivationListener();

    // Create floor
    const floor = try context.createBox(RVec3.init(0, -1, 0), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(100, 1, 100), .{});
    try expect(!floor.isActive());

    // Create inactive box
    const box = try context.createBox(RVec3.init(0, 5, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .dont_activate });
    try expect(!box.isActive());
    try expect(activation_listener.getEntryCount() == 0);

    // Box should not activate by itself
    _ = try ctx.simulate(1.0, {});
    try expect(box.getPosition().eql(RVec3.init(0, 5, 0)));
    try expect(!box.isActive());
    try expect(activation_listener.getEntryCount() == 0);

    // Activate the body and validate it is active now
    context.getBodyInterface().activateBody(box.getID());
    try expect(box.isActive());
    try expect(box.getLinearVelocity().isNearZero(.{}));
    try expect(activation_listener.getEntryCount() == 1);
    try expect(activation_listener.contains(.activated, box.getID()));
    activation_listener.clear();

    // Do a single step and check that the body is still active and has gained some velocity
    _ = try ctx.simulateSingleStep();
    try expect(box.isActive());
    try expect(activation_listener.getEntryCount() == 0);
    try expect(!box.getLinearVelocity().isNearZero(.{}));

    // Simulate 5 seconds and check it has settled on the floor and is no longer active
    _ = try ctx.simulate(5.0, {});
    try checkApproxEqual(box.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = 1.1 * penetration_slop });
    try checkApproxEqual(box.getLinearVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{});
    try expect(!box.isActive());
    try expect(activation_listener.getEntryCount() == 1);
    try expect(activation_listener.contains(.deactivated, box.getID()));
}

test "TestPhysicsActivationDeactivation" {
    try testPhysicsActivationDeactivation(.{ .delta_time = 1.0 / 60.0, .collision_steps = 1 });
    try testPhysicsActivationDeactivation(.{ .delta_time = 2.0 / 60.0, .collision_steps = 2 });
    try testPhysicsActivationDeactivation(.{ .delta_time = 4.0 / 60.0, .collision_steps = 4 });
}

/// A test that checks that a row of penetrating boxes will all activate and handle collision in 1 frame so that active bodies cannot tunnel through inactive bodies
fn testPhysicsActivateDuringStep(reverse_create: bool) !void {
    const allocator = std.testing.allocator;
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const context = &ctx.c;

    const penetration_slop = context.getSystem().getPhysicsSettings().penetration_slop;
    const num_bodies = 10;
    const box_extent: f32 = 0.5;

    const bi = context.getBodyInterface();

    const activation_listener = &ctx.activation_listener;
    ctx.registerActivationListener();

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create a row of penetrating boxes. Since some of the algorithms rely on body index, we create them normally and reversed to test both cases
    var body_ids: BodyIDVector = .empty;
    defer body_ids.deinit(allocator);
    if (reverse_create) {
        var i: i32 = num_bodies - 1;
        while (i >= 0) : (i -= 1)
            try body_ids.insert(allocator, 0, (try context.createBox(rvec3f(@as(f32, @floatFromInt(i)) * (2.0 * box_extent - penetration_slop), 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{ .activation = .dont_activate })).getID());
    } else {
        for (0..num_bodies) |i|
            try body_ids.append(allocator, (try context.createBox(rvec3f(@as(f32, @floatFromInt(i)) * (2.0 * box_extent - penetration_slop), 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .dont_activate })).getID());
    }

    // Test that nothing is active yet
    try expect(activation_listener.getEntryCount() == 0);
    try expect(contact_listener.getEntryCount() == 0);
    for (body_ids.items) |id|
        try expect(!bi.isActive(id));

    // Activate the left most box and give it a velocity that is high enough to make it tunnel through the second box in a single step
    bi.setLinearVelocity(body_ids.items[0], Vec3.init(500, 0, 0));

    // Test that only the left most box is active
    try expect(activation_listener.getEntryCount() == 1);
    try expect(contact_listener.getEntryCount() == 0);
    try expect(bi.isActive(body_ids.items[0]));
    try expect(activation_listener.contains(.activated, body_ids.items[0]));
    for (1..num_bodies) |i|
        try expect(!bi.isActive(body_ids.items[i]));
    activation_listener.clear();

    // Step the world
    _ = try ctx.simulateSingleStep();

    // Other bodies should now be awake and each body should only collide with its neighbor
    try expect(activation_listener.getEntryCount() == num_bodies - 1);
    try expect(contact_listener.getEntryCount() == 2 * (num_bodies - 1));

    for (0..num_bodies) |i| {
        const id = body_ids.items[i];

        // Check body is active
        try expect(bi.isActive(id));

        // Check that body moved to the right
        try expect(bi.getPosition(id).getX() > @as(f32, @floatFromInt(i)) * (2.0 * box_extent - penetration_slop));
    }

    for (1..num_bodies) |i| {
        const id1 = body_ids.items[i - 1];
        const id2 = body_ids.items[i];

        // Check that we received activation events for each body
        try expect(activation_listener.contains(.activated, id2));

        // Check that we received a validate and an add for each body pair
        const validate = contact_listener.find(.validate, id1, id2);
        try expect(validate != null);
        const add = contact_listener.find(.add, id1, id2);
        try expect(add != null);
        try expect(add.? > validate.?);

        // Check that bodies did not tunnel through each other
        try expect(bi.getPosition(id1).getX() < bi.getPosition(id2).getX());
    }
}

test "TestPhysicsActivateDuringStep" {
    try testPhysicsActivateDuringStep(false);
    try testPhysicsActivateDuringStep(true);
}

test "TestPhysicsBroadPhaseLayers" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Reduce slop
    var settings: PhysicsSettings = c.getSystem().getPhysicsSettings().*;
    settings.penetration_slop = 0.0;
    c.getSystem().setPhysicsSettings(&settings);

    // Create static floor
    _ = try c.createFloor();

    // Create MOVING boxes
    const moving1 = try c.createBox(RVec3.init(0, 1, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .activate });
    const moving2 = try c.createBox(RVec3.init(0, 2, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .activate });

    // Create HQ_DEBRIS boxes
    const hq_debris1 = try c.createBox(RVec3.init(0, 3, 0), Quat.identity(), .dynamic, .discrete, Layers.hq_debris, Vec3.replicate(0.5), .{ .activation = .activate });
    const hq_debris2 = try c.createBox(RVec3.init(0, 4, 0), Quat.identity(), .dynamic, .discrete, Layers.hq_debris, Vec3.replicate(0.5), .{ .activation = .activate });

    // Create LQ_DEBRIS boxes
    const lq_debris1 = try c.createBox(RVec3.init(0, 5, 0), Quat.identity(), .dynamic, .discrete, Layers.lq_debris, Vec3.replicate(0.5), .{ .activation = .activate });
    const lq_debris2 = try c.createBox(RVec3.init(0, 6, 0), Quat.identity(), .dynamic, .discrete, Layers.lq_debris, Vec3.replicate(0.5), .{ .activation = .activate });

    // Check layers
    try expect(moving1.getObjectLayer() == Layers.moving);
    try expect(moving2.getObjectLayer() == Layers.moving);
    try expect(hq_debris1.getObjectLayer() == Layers.hq_debris);
    try expect(hq_debris2.getObjectLayer() == Layers.hq_debris);
    try expect(lq_debris1.getObjectLayer() == Layers.lq_debris);
    try expect(lq_debris2.getObjectLayer() == Layers.lq_debris);
    try expect(moving1.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(moving2.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(hq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(hq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(lq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(lq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));

    // Simulate the boxes falling
    _ = try c.simulate(5.0, {});

    // Everything should sleep
    try expect(!moving1.isActive());
    try expect(!moving2.isActive());
    try expect(!hq_debris1.isActive());
    try expect(!hq_debris2.isActive());
    try expect(!lq_debris1.isActive());
    try expect(!lq_debris2.isActive());

    // MOVING boxes should have stacked
    const slop: f32 = 0.02;
    try checkApproxEqual(moving1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(moving2.getPosition(), rvec3f(0, 1.5, 0), .{ .tolerance = slop });

    // HQ_DEBRIS boxes should have stacked on MOVING boxes but don't collide with each other
    try checkApproxEqual(hq_debris1.getPosition(), rvec3f(0, 2.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(hq_debris2.getPosition(), rvec3f(0, 2.5, 0), .{ .tolerance = slop });

    // LQ_DEBRIS should have fallen through all but the floor
    try checkApproxEqual(lq_debris1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(lq_debris2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });

    // Now change HQ_DEBRIS to LQ_DEBRIS
    try bi.setObjectLayer(hq_debris1.getID(), Layers.lq_debris);
    try bi.setObjectLayer(hq_debris2.getID(), Layers.lq_debris);
    bi.activateBody(hq_debris1.getID());
    bi.activateBody(hq_debris2.getID());

    // Check layers
    try expect(moving1.getObjectLayer() == Layers.moving);
    try expect(moving2.getObjectLayer() == Layers.moving);
    try expect(hq_debris1.getObjectLayer() == Layers.lq_debris);
    try expect(hq_debris2.getObjectLayer() == Layers.lq_debris);
    try expect(lq_debris1.getObjectLayer() == Layers.lq_debris);
    try expect(lq_debris2.getObjectLayer() == Layers.lq_debris);
    try expect(moving1.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(moving2.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(hq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(hq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(lq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(lq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));

    // Simulate again
    _ = try c.simulate(5.0, {});

    // Everything should sleep
    try expect(!moving1.isActive());
    try expect(!moving2.isActive());
    try expect(!hq_debris1.isActive());
    try expect(!hq_debris2.isActive());
    try expect(!lq_debris1.isActive());
    try expect(!lq_debris2.isActive());

    // MOVING boxes should have stacked
    try checkApproxEqual(moving1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(moving2.getPosition(), rvec3f(0, 1.5, 0), .{ .tolerance = slop });

    // HQ_DEBRIS (now LQ_DEBRIS) boxes have fallen through all but the floor
    try checkApproxEqual(hq_debris1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(hq_debris2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });

    // LQ_DEBRIS should have fallen through all but the floor
    try checkApproxEqual(lq_debris1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(lq_debris2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });

    // Now change MOVING to HQ_DEBRIS (this doesn't change the broadphase layer so avoids adding/removing bodies)
    try bi.setObjectLayer(moving1.getID(), Layers.hq_debris);
    try bi.setObjectLayer(moving2.getID(), Layers.hq_debris);
    bi.activateBody(moving1.getID());
    bi.activateBody(moving2.getID());

    // Check layers
    try expect(moving1.getObjectLayer() == Layers.hq_debris);
    try expect(moving2.getObjectLayer() == Layers.hq_debris);
    try expect(hq_debris1.getObjectLayer() == Layers.lq_debris);
    try expect(hq_debris2.getObjectLayer() == Layers.lq_debris);
    try expect(lq_debris1.getObjectLayer() == Layers.lq_debris);
    try expect(lq_debris2.getObjectLayer() == Layers.lq_debris);
    try expect(moving1.getBroadPhaseLayer().eql(BroadPhaseLayers.moving)); // Broadphase layer didn't change
    try expect(moving2.getBroadPhaseLayer().eql(BroadPhaseLayers.moving));
    try expect(hq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(hq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(lq_debris1.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));
    try expect(lq_debris2.getBroadPhaseLayer().eql(BroadPhaseLayers.lq_debris));

    // Simulate again
    _ = try c.simulate(5.0, {});

    // Everything should sleep
    try expect(!moving1.isActive());
    try expect(!moving2.isActive());
    try expect(!hq_debris1.isActive());
    try expect(!hq_debris2.isActive());
    try expect(!lq_debris1.isActive());
    try expect(!lq_debris2.isActive());

    // MOVING boxes now also fall through
    try checkApproxEqual(moving1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(moving2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });

    // HQ_DEBRIS (now LQ_DEBRIS) boxes have fallen through all but the floor
    try checkApproxEqual(hq_debris1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(hq_debris2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });

    // LQ_DEBRIS should have fallen through all but the floor
    try checkApproxEqual(lq_debris1.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
    try checkApproxEqual(lq_debris2.getPosition(), rvec3f(0, 0.5, 0), .{ .tolerance = slop });
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

test "TestAddSingleBodies" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0 });
    defer c.deinit();

    const bi = c.getBodyInterface();
    const sys = c.getSystem();

    const max_bodies = 128;

    // Add individual bodies in a way that will create an inefficient broad phase and will trigger a warning on query
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 1.0, .{})).asShape());
    defer sphere.deinit();
    var bcs = BodyCreationSettings.initWithShape(sphere.get(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer bcs.deinit();
    _ = try bi.createAndAddBody(&bcs, .activate); // Leave this body
    for (0..10) |repeat| {
        // Create cMaxBodies - 1 bodies
        var body_ids: BodyIDVector = .empty;
        defer body_ids.deinit(allocator);
        for (0..max_bodies - 1) |_|
            try body_ids.append(allocator, try bi.createAndAddBody(&bcs, .activate));

        // In all but the last iteration, remove the bodies again
        if (repeat < 9) {
            for (body_ids.items) |id| {
                bi.deactivateBody(id);
                bi.removeBody(id);
                bi.destroyBody(id);
            }
        }
    }

    // Zolt: instead of overriding the trace function to count how many times we get a "Stack full" message, check the
    // latch of the warning (only with asserts): it has not been reported yet
    const quad_tree = zolt.virtual.downcast(BroadPhaseQuadTree, sys.broad_phase.?);
    if (Core.enable_asserts)
        try expect(!quad_tree.mutable_fields.triggered_report.load(.seq_cst));

    // Cast a ray
    var ray_collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer ray_collector.deinit();
    sys.getNarrowPhaseQuery().castRayCollector(RRayCast.init(RVec3.init(-2, 0, 0), Vec3.init(2, 0, 0)), &.{}, &ray_collector.base, .{});
    try ray_collector.checkError();

    // Find colliding pairs
    var active_bodies: BodyIDVector = .empty;
    defer active_bodies.deinit(allocator);
    try sys.getActiveBodies(allocator, .rigid_body, &active_bodies);
    var body_pair_collector = AllHitCollisionCollector(BodyPairCollector).init(allocator);
    defer body_pair_collector.deinit();
    try zolt.virtual.downcast(BroadPhase, sys.getBroadPhaseQuery()).findCollidingPairs(active_bodies.items, 0.0, sys.getObjectVsBroadPhaseLayerFilter(), sys.getObjectLayerPairFilter(), &body_pair_collector.base);
    try body_pair_collector.checkError();

    // Assert that we got a "Stack full" message when asserts are enabled (Zolt: the latch is set; without asserts the
    // warning does not exist)
    if (Core.enable_asserts)
        try expect(quad_tree.mutable_fields.triggered_report.load(.seq_cst));

    // Assert that we hit all bodies
    try expect(ray_collector.hits.items.len == max_bodies);
    try expect(body_pair_collector.hits.items.len == max_bodies * (max_bodies - 1) / 2);
}

test "TestOutOfContactConstraints" {
    // Create a context with space for 8 constraints
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0, .max_bodies = 1024, .max_body_pairs = 4096, .max_contact_constraints = 8 });
    defer c.deinit();

    _ = try c.createFloor();

    // The first 8 boxes should be fine
    for (0..8) |i|
        _ = try c.createBox(RVec3.init(3.0 * @as(Real, @floatFromInt(i)), 0.9, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

    // Step
    var errors = try c.simulateSingleStep();
    try expect(errors.isNone());

    // Adding one more box should introduce an error
    _ = try c.createBox(RVec3.init(24.0, 0.9, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

    // Step
    {
        var expect_assert: fw.ExpectAssert = undefined;
        expect_assert.init(1); // JPH_IF_ENABLE_ASSERTS(ExpectAssert expect_assert(1);)
        errdefer expect_assert.restore();
        errors = try c.simulateSingleStep();
        try expect_assert.finish();
    }
    try expect(!errors.bitAnd(.{ .contact_constraints_full = true }).isNone());
}

test "TestFriction" {
    const friction_floor: f32 = 0.9;
    const friction_box: f32 = 0.8;
    const combined_friction = math.sqrt(friction_floor * friction_box);

    var angle: f32 = 0;
    while (angle < 360.0) : (angle += 30.0) {
        // Create a context with space for 8 constraints
        const allocator = std.testing.allocator;
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0, .max_bodies = 1024, .max_body_pairs = 4096, .max_contact_constraints = 8 });
        defer c.deinit();

        // Create floor
        const floor = try c.createFloor();
        floor.setFriction(friction_floor);

        // Create box with a velocity that will make it slide over the floor (making sure it intersects a little bit initially)
        var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), RVec3.init(0, 0.999, 0), Quat.identity(), .dynamic, Layers.moving);
        defer box_settings.deinit();
        box_settings.friction = friction_box;
        box_settings.linear_damping = 0;
        box_settings.linear_velocity = Vec3.init(trigonometry.sin(math.degreesToRadians(angle)), 0, trigonometry.cos(math.degreesToRadians(angle))).mulScalar(20.0);
        const box = (try c.getBodyInterface().createBody(&box_settings)).?;
        try c.getBodyInterface().addBody(box.getID(), .activate);

        // We know that the friction force equals the normal force times the friction coefficient
        const friction_acceleration = combined_friction * c.getSystem().getGravity().length();

        // Simulate
        var velocity = box_settings.linear_velocity;
        var position = box_settings.position;
        for (0..60) |_| {
            _ = try c.simulateSingleStep();

            // Integrate our own simulation
            velocity = velocity.sub(velocity.normalized().mulScalar(friction_acceleration).mulScalar(c.getDeltaTime()));
            position = addR(position, velocity.mulScalar(c.getDeltaTime()));
        }

        // Note that the result is not very accurate so we need quite a high tolerance
        try checkApproxEqual(box.getCenterOfMassPosition(), position, .{ .tolerance = 1.0e-2 });
        try checkApproxEqual(box.getRotation(), box_settings.rotation, .{ .tolerance = 1.0e-2 });
        try checkApproxEqual(box.getLinearVelocity(), velocity, .{ .tolerance = 2.0e-2 });
        try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-2 });
    }
}

test "TestAngularFriction" {
    const friction_floor: f32 = 0.2;
    const friction_box: f32 = 0.3;
    const combined_friction = math.sqrt(friction_floor * friction_box);

    // Create a context with space for 8 constraints
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0, .max_bodies = 1024, .max_body_pairs = 4096, .max_contact_constraints = 8 });
    defer c.deinit();

    // Create floor
    const floor = try c.createFloor();
    floor.setFriction(friction_floor);

    // Create box with an angular velocity that will make it rotate on the floor (making sure it intersects a little bit initially)
    const initial_position = RVec3.init(10, 0.999, 10);
    var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), initial_position, Quat.identity(), .dynamic, Layers.moving);
    defer box_settings.deinit();
    box_settings.friction = friction_box;
    box_settings.angular_damping = 0;
    box_settings.angular_velocity = Vec3.init(0, 10, 0);
    const box = (try c.getBodyInterface().createBody(&box_settings)).?;
    try c.getBodyInterface().addBody(box.getID(), .activate);

    // Force that the box exerts on the floor
    const contact_force = c.getSystem().getGravity().length() / box.getMotionProperties().getInverseMass();

    // Assuming we apply friction at each contact point (the corners of the box). The angular friction torque will be this:
    const distance_to_contact_point = math.sqrt(@as(f32, 2.0));
    const friction_torque = combined_friction * distance_to_contact_point * contact_force;

    // And then we can calculate the acceleration
    const friction_angular_acceleration = Vec3.init(0, box.getInverseInertia().get(2, 2) * friction_torque, 0);

    // Simulate
    var angular_velocity = box_settings.angular_velocity;
    var rotation = box_settings.rotation;
    for (0..60) |_| {
        _ = try c.simulateSingleStep();

        // Integrate our own simulation
        angular_velocity = angular_velocity.sub(friction_angular_acceleration.mulScalar(c.getDeltaTime()));
        const angular_velocity_len = angular_velocity.length();
        rotation = Quat.rotation(angular_velocity.divScalar(angular_velocity_len), angular_velocity_len * c.getDeltaTime()).mul(rotation).normalized();
    }

    try expect(angular_velocity.getY() >= 0.0); // Check that we are slowing down but haven't reversed direction yet
    try expect(angular_velocity.getY() < 5.0); // Check that we lost significant speed

    // Note that the result is not very accurate so we need quite a high tolerance
    try checkApproxEqual(box.getCenterOfMassPosition(), initial_position, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(box.getRotation(), rotation, .{ .tolerance = 2.0e-3 });
    try checkApproxEqual(box.getLinearVelocity(), Vec3.zero(), .{ .tolerance = 4.0e-4 });
    try checkApproxEqual(box.getAngularVelocity(), angular_velocity, .{ .tolerance = 5.0e-3 });
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

test "TestAllowedDOFsVsCollision" {
    const allocator = std.testing.allocator;
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;
    const floor = try c.createFloor();
    floor.setFriction(1.0);

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create box that can only rotate around Y that intersects with the floor
    const initial_position = rvec3f(0, 0.99, 0);
    var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), initial_position, Quat.identity(), .dynamic, Layers.moving);
    defer box_settings.deinit();
    box_settings.allowed_dofs = .{ .rotation_y = true };
    box_settings.angular_damping = 0.0; // No damping to make the calculation for expected angular velocity simple
    box_settings.override_mass_properties = .calculate_inertia;
    box_settings.mass_properties_override.mass = 1.0;
    box_settings.friction = 1.0; // High friction so that if the collision is processed, we'll slow down the rotation
    const body = (try c.getBodyInterface().createBody(&box_settings)).?;
    try c.getBodyInterface().addBody(body.getID(), .activate);

    // Make the box rotate around Y
    const torque = Vec3.init(0, 100.0, 0);
    body.addTorque(torque);

    // Simulate a step, this will make the box collide with the floor but should not result in the floor stopping the body
    // but will cause the effective mass of the contact to become infinite so is a test if we are properly ignoring the contact in this case
    _ = try ctx.simulateSingleStep();

    // Check that we did detect the collision
    try expect(contact_listener.contains(.add, floor.getID(), body.getID()));

    // Check that we have the correct angular velocity
    const expected_angular_velocity = torque.mulScalar(c.getDeltaTime()).mulScalar(body.getInverseInertia().get(1, 1));
    try checkApproxEqual(body.getAngularVelocity(), expected_angular_velocity, .{});
    try expect(body.getLinearVelocity().eql(Vec3.zero()));
    try expect(body.getPosition().eql(initial_position));
}

test "TestAllowedDOFsVsSmallRotation" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const floor = try c.createFloor();
    floor.setFriction(1.0);

    const penetration_slop = c.getSystem().getPhysicsSettings().penetration_slop;
    const initial_pos = RVec3.init(0, 0.5 - @as(Real, penetration_slop), 0);

    // Introduce a slight rotation around the X axis (which has been frozen by EAllowedDOFs).
    // This creates a very tiny (denormalized float) effective mass for the angular friction component.
    // Inverting this effective mass will make it INF and cause a FP exception if not handled properly.
    var bcs = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), initial_pos, Quat.init(1.0e-20, 0, 0, 1.0), .dynamic, Layers.moving);
    defer bcs.deinit();
    bcs.allowed_dofs = AllowedDOFs.plane_2d;
    bcs.linear_velocity = Vec3.init(0, -1, 0);
    bcs.friction = 1.0;
    const id = try c.getBodyInterface().createAndAddBody(&bcs, .activate);

    _ = try c.simulateSingleStep();

    // Check not moved
    try checkApproxEqual(c.getBodyInterface().getPosition(id), initial_pos, .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(c.getBodyInterface().getLinearVelocity(id), Vec3.zero(), .{ .tolerance = 1.0e-3 });
}

test "TestSelectiveStateSaveAndRestore" {
    const allocator = std.testing.allocator;

    const MyFilter = struct {
        pub const overrides = .{ .shouldSaveBody, .shouldSaveContact };

        base: StateRecorderFilter = .init(@This()),
        ignore_bodies: std.ArrayList(BodyID) = .empty,

        fn shouldSaveBodyID(self: *const @This(), body_id: BodyID) bool {
            for (self.ignore_bodies.items) |id|
                if (id.eql(body_id)) return false;
            return true;
        }

        pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
            return self.shouldSaveBodyID(body.getID());
        }

        pub fn shouldSaveContact(self: *const @This(), body1: BodyID, body2: BodyID) bool {
            return self.shouldSaveBodyID(body1) and self.shouldSaveBodyID(body2);
        }
    };

    for (0..2) |mode| {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        const sys = c.getSystem();

        const grav = sys.getGravity();
        const upside_down_gravity = grav.negate();

        // Create the ground.
        const ground = try c.createFloor();

        // Create two sets of bodies that each overlap
        const box1 = try c.createBox(RVec3.init(0, 1, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
        const sphere1 = try c.createSphere(rvec3f(0, 1, 0.1), 1.0, .dynamic, .discrete, Layers.moving, .{ .activation = .activate });

        const box2 = try c.createBox(RVec3.init(5, 1, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
        const sphere2 = try c.createSphere(rvec3f(5, 1, 0.1), 1.0, .dynamic, .discrete, Layers.moving, .{ .activation = .activate });

        // Store the absolute initial state, that will be used for the final test.
        var absolute_initial_state = StateRecorderImpl.init(allocator);
        defer absolute_initial_state.deinit();
        try sys.saveState(absolute_initial_state.recorder(), .{});

        var state_to_save = StateRecorderState.all;
        var filter: MyFilter = .{};
        defer filter.ignore_bodies.deinit(allocator);
        if (mode == 1) {
            // Don't save the global state
            state_to_save = StateRecorderState.all.bitXor(.{ .global = true });

            // Don't save some bodies
            try filter.ignore_bodies.append(allocator, ground.getID());
            try filter.ignore_bodies.append(allocator, box2.getID());
            try filter.ignore_bodies.append(allocator, sphere2.getID());
        }

        // Store the initial transform.
        const initial_box1_transform = box1.getWorldTransform();
        const initial_sphere1_transform = sphere1.getWorldTransform();
        const initial_box2_transform = box2.getWorldTransform();
        const initial_sphere2_transform = sphere2.getWorldTransform();

        // Save partial state
        var initial_state = StateRecorderImpl.init(allocator);
        defer initial_state.deinit();
        try sys.saveState(initial_state.recorder(), .{ .state = state_to_save, .filter = &filter.base });

        // Simulate for 2 seconds
        _ = try c.simulate(2.0, {});

        // The bodies should have moved and come to rest
        const intermediate_box1_transform = box1.getWorldTransform();
        const intermediate_sphere1_transform = sphere1.getWorldTransform();
        const intermediate_box2_transform = box2.getWorldTransform();
        const intermediate_sphere2_transform = sphere2.getWorldTransform();
        try expect(!intermediate_box1_transform.eql(initial_box1_transform));
        try expect(!intermediate_sphere1_transform.eql(initial_sphere1_transform));
        try expect(!intermediate_box2_transform.eql(initial_box2_transform));
        try expect(!intermediate_sphere2_transform.eql(initial_sphere2_transform));
        try expect(!box1.isActive());
        try expect(!sphere1.isActive());
        try expect(!box2.isActive());
        try expect(!sphere2.isActive());

        // Save the intermediate state.
        var intermediate_state = StateRecorderImpl.init(allocator);
        defer intermediate_state.deinit();
        try sys.saveState(intermediate_state.recorder(), .{ .state = state_to_save, .filter = &filter.base });

        // Change the gravity.
        sys.setGravity(upside_down_gravity);

        // Restore the initial state.
        _ = try sys.restoreState(initial_state.recorder(), .{});

        // Make sure the state is properly set back to the initial state.
        try expect(box1.getWorldTransform().eql(initial_box1_transform));
        try expect(sphere1.getWorldTransform().eql(initial_sphere1_transform));
        try expect(box1.isActive());
        try expect(sphere1.isActive());
        if (mode == 0) {
            // Make sure the gravity is restored.
            try expect(sys.getGravity().eql(grav));

            // The second set of bodies should have been restored as well
            try expect(box2.getWorldTransform().eql(initial_box2_transform));
            try expect(sphere2.getWorldTransform().eql(initial_sphere2_transform));
            try expect(box2.isActive());
            try expect(sphere2.isActive());
        } else {
            // Make sure the gravity is NOT restored.
            try expect(sys.getGravity().eql(upside_down_gravity));
            sys.setGravity(grav);

            // The second set of bodies should NOT have been restored
            try expect(box2.getWorldTransform().eql(intermediate_box2_transform));
            try expect(sphere2.getWorldTransform().eql(intermediate_sphere2_transform));
            try expect(!box2.isActive());
            try expect(!sphere2.isActive());

            // Apply a velocity to the second set of bodies to make sure they are active again
            c.getBodyInterface().setLinearVelocity(box2.getID(), Vec3.init(0, 0, 0.1));
            c.getBodyInterface().setLinearVelocity(sphere2.getID(), Vec3.init(0, 0, 0.1));
        }

        // Simulate for 2 seconds - again
        _ = try c.simulate(2.0, {});

        // The first set of bodies have been saved and should have returned to the same positions again
        try expect(box1.getWorldTransform().eql(intermediate_box1_transform));
        try expect(sphere1.getWorldTransform().eql(intermediate_sphere1_transform));
        try expect(!box1.isActive());
        try expect(!sphere1.isActive());
        if (mode == 0) {
            // The second set of bodies have been saved and should have returned to the same positions again
            try expect(box2.getWorldTransform().eql(intermediate_box2_transform));
            try expect(sphere2.getWorldTransform().eql(intermediate_sphere2_transform));
            try expect(!box2.isActive());
            try expect(!sphere2.isActive());
        } else {
            // The second set of bodies have not been saved and should have moved on
            try expect(!box2.getWorldTransform().eql(intermediate_box2_transform));
            try expect(!sphere2.getWorldTransform().eql(intermediate_sphere2_transform));
            try expect(!box2.isActive());
            try expect(sphere2.isActive()); // The sphere keeps rolling
        }

        // Save the final state
        var final_state = StateRecorderImpl.init(allocator);
        defer final_state.deinit();
        try sys.saveState(final_state.recorder(), .{ .state = state_to_save, .filter = &filter.base });

        // Compare the states to make sure they are the same
        try expect(final_state.isEqual(&intermediate_state));

        // Now restore the absolute initial state and make sure all the
        // bodies are being active and ready to be processed again
        _ = try sys.restoreState(absolute_initial_state.recorder(), .{});

        try expect(box1.getWorldTransform().eql(initial_box1_transform));
        try expect(sphere1.getWorldTransform().eql(initial_sphere1_transform));
        try expect(box2.getWorldTransform().eql(initial_box2_transform));
        try expect(sphere2.getWorldTransform().eql(initial_sphere2_transform));
        try expect(box1.isActive());
        try expect(sphere1.isActive());
        try expect(box2.isActive());
        try expect(sphere2.isActive());

        // Save the state of a single body
        var single_body = StateRecorderImpl.init(allocator);
        defer single_body.deinit();
        sys.saveBodyState(box2, single_body.recorder());

        // Simulate for 2 seconds - again
        _ = try c.simulate(2.0, {});

        // We should have reached the same state as before
        try expect(box1.getWorldTransform().eql(intermediate_box1_transform));
        try expect(sphere1.getWorldTransform().eql(intermediate_sphere1_transform));
        try expect(box2.getWorldTransform().eql(intermediate_box2_transform));
        try expect(sphere2.getWorldTransform().eql(intermediate_sphere2_transform));
        try expect(!box1.isActive());
        try expect(!sphere1.isActive());
        try expect(!box2.isActive());
        try expect(!sphere2.isActive());

        // Restore the single body
        sys.restoreBodyState(box2, single_body.recorder());

        // Only that body should have been restored
        try expect(box1.getWorldTransform().eql(intermediate_box1_transform));
        try expect(sphere1.getWorldTransform().eql(intermediate_sphere1_transform));
        try expect(box2.getWorldTransform().eql(initial_box2_transform));
        try expect(sphere2.getWorldTransform().eql(intermediate_sphere2_transform));
        try expect(!box1.isActive());
        try expect(!sphere1.isActive());
        try expect(box2.isActive());
        try expect(!sphere2.isActive());
    }
}

test "TestMultiPartRestoreState" {
    const allocator = std.testing.allocator;

    const MyFilter = struct {
        pub const overrides = .{ .shouldSaveBody, .shouldSaveContact };

        /// Jolt's `mutable int` counters (Rule M: behind a pointer, the filter is used through `*const`)
        const Counters = struct {
            num_bodies: i32 = 0,
            num_contacts: i32 = 0,
        };

        base: StateRecorderFilter = .init(@This()),
        stored_bodies: []const BodyID,
        counters: *Counters,

        fn shouldSaveBodyID(self: *const @This(), body_id: BodyID) bool {
            for (self.stored_bodies) |id|
                if (id.eql(body_id)) return true;
            return false;
        }

        pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
            if (self.shouldSaveBodyID(body.getID())) {
                self.counters.num_bodies += 1;
                return true;
            }
            return false;
        }

        pub fn shouldSaveContact(self: *const @This(), body1: BodyID, body2: BodyID) bool {
            if (self.shouldSaveBodyID(body1) or self.shouldSaveBodyID(body2)) {
                self.counters.num_contacts += 1;
                return true;
            }
            return false;
        }
    };

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const sys = c.getSystem();
    _ = try c.createFloor();

    // Create 1st set of moving bodies
    const num_moving1 = 10;
    var moving1: std.ArrayList(BodyID) = .empty;
    defer moving1.deinit(allocator);
    for (0..num_moving1) |i| {
        const fi: f32 = @floatFromInt(i);
        try moving1.append(allocator, (try c.createSphere(rvec3f(0, 2.0 + 2.0 * fi, 0.01 * fi), 1.0, .dynamic, .discrete, Layers.moving, .{ .activation = .activate })).getID());
    }

    // Create 2nd set of moving bodies, note that although the bodies overlap with the 1st set, they don't collide because of their layer.
    // We need to create disjoint sets for restoring in parts to work.
    const num_moving2 = 12;
    var moving2: std.ArrayList(BodyID) = .empty;
    defer moving2.deinit(allocator);
    for (0..num_moving2) |i| {
        const fi: f32 = @floatFromInt(i);
        try moving2.append(allocator, (try c.createSphere(rvec3f(1.0, 2.0 + 2.0 * fi, 0.01 * fi), 1.0, .dynamic, .discrete, Layers.moving2, .{ .activation = .activate })).getID());
    }

    // Simulate for a short while to get some contacts
    _ = try c.simulate(2.0, {});

    // Save full snapshot
    var initial_state = StateRecorderImpl.init(allocator);
    defer initial_state.deinit();
    try sys.saveState(initial_state.recorder(), .{});

    // Save everything relating to 1st set of bodies
    var counters1: MyFilter.Counters = .{};
    const filter1: MyFilter = .{ .stored_bodies = moving1.items, .counters = &counters1 };
    var state1 = StateRecorderImpl.init(allocator);
    defer state1.deinit();
    try sys.saveState(state1.recorder(), .{ .state = .all, .filter = &filter1.base });
    try expect(counters1.num_bodies == num_moving1);
    try expect(counters1.num_contacts > num_moving1 / 2); // Many bodies should be in contact now, if not we're not testing contact restoring
    try expect(state1.getDataSize() < initial_state.getDataSize()); // Should be smaller than the full state

    // Save everything relating to 2nd set of bodies
    var counters2: MyFilter.Counters = .{};
    const filter2: MyFilter = .{ .stored_bodies = moving2.items, .counters = &counters2 };
    var state2 = StateRecorderImpl.init(allocator);
    defer state2.deinit();
    try sys.saveState(state2.recorder(), .{ .state = StateRecorderState.bitOr(.{ .bodies = true }, .{ .contacts = true }), .filter = &filter2.base });
    try expect(counters2.num_bodies == num_moving2);
    try expect(counters2.num_contacts > num_moving2 / 2);
    try expect(state2.getDataSize() < initial_state.getDataSize());

    // Simulate for 2 seconds
    _ = try c.simulate(2.0, {});

    // Save result
    var final_state = StateRecorderImpl.init(allocator);
    defer final_state.deinit();
    try sys.saveState(final_state.recorder(), .{});

    // Restore the initial state in parts
    state1.recorder().setIsLastPart(false);
    _ = try sys.restoreState(state1.recorder(), .{});
    _ = try sys.restoreState(state2.recorder(), .{});

    // Verify we're back to the first state
    var verify1 = StateRecorderImpl.init(allocator);
    defer verify1.deinit();
    try sys.saveState(verify1.recorder(), .{});
    try expect(initial_state.isEqual(&verify1));

    // Simulate for 2 seconds again
    _ = try c.simulate(2.0, {});

    // Check we end up in the final state again
    var verify2 = StateRecorderImpl.init(allocator);
    defer verify2.deinit();
    try sys.saveState(verify2.recorder(), .{});
    try expect(final_state.isEqual(&verify2));
}

// This tests that when switching UseManifoldReduction on/off we get the correct contact callbacks
test "TestSwitchUseManifoldReduction" {
    const allocator = std.testing.allocator;
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;

    // Install listener
    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create floor
    const floor = try c.createFloor();

    // Create a compound with 4 boxes
    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(2), .{})).asShape());
    defer box_shape.deinit();
    const compound_settings = try StaticCompoundShapeSettings.create(allocator);
    var shape_settings = Ref(ShapeSettings).init(compound_settings.asShapeSettings());
    defer shape_settings.deinit();
    try compound_settings.base.addShapePtr(Vec3.init(5, 0, 0), Quat.identity(), box_shape.get(), .{});
    try compound_settings.base.addShapePtr(Vec3.init(-5, 0, 0), Quat.identity(), box_shape.get(), .{});
    try compound_settings.base.addShapePtr(Vec3.init(0, 0, 5), Quat.identity(), box_shape.get(), .{});
    try compound_settings.base.addShapePtr(Vec3.init(0, 0, -5), Quat.identity(), box_shape.get(), .{});
    var result = try shape_settings.get().?.createShape(allocator);
    defer result.deinit();
    var compound_shape_ref = RefConst(Shape).init(result.getPtr());
    defer compound_shape_ref.deinit();
    const compound_shape = compound_shape_ref.get().?.cast(StaticCompoundShape);
    const sub_shape_ids = [_]SubShapeID{
        compound_shape.base.getSubShapeIDFromIndex(0, .{}).getID(),
        compound_shape.base.getSubShapeIDFromIndex(1, .{}).getID(),
        compound_shape.base.getSubShapeIDFromIndex(2, .{}).getID(),
        compound_shape.base.getSubShapeIDFromIndex(3, .{}).getID(),
    };

    // Embed body a little bit into the floor so we immediately get contact callbacks
    var body_settings = BodyCreationSettings.initWithShape(compound_shape_ref.get(), RVec3.init(0, 1.99, 0), Quat.identity(), .dynamic, Layers.moving);
    defer body_settings.deinit();
    body_settings.use_manifold_reduction = true;
    const body_id = try c.getBodyInterface().createAndAddBody(&body_settings, .activate);

    // Trigger contact callbacks
    _ = try ctx.simulateSingleStep();

    // Since manifold reduction is on and the contacts will be coplanar we should only get 1 contact with the floor
    // Note that which sub shape ID we get is deterministic but not guaranteed to be a particular value, sub_shape_ids[3] is the one it currently returns!!
    try expect(contact_listener.getEntryCount() == 5); // 4x validate + 1x add
    try expect(contact_listener.containsSubShape(.add, floor.getID(), .empty, body_id, sub_shape_ids[3]));
    contact_listener.clear();

    // Now disable manifold reduction
    try c.getBodyInterface().setUseManifoldReduction(body_id, false);

    // Trigger contact callbacks
    _ = try ctx.simulateSingleStep();

    // Now manifold reduction is off so we should get collisions with each of the sub shapes
    try expect(contact_listener.getEntryCount() == 8); // 4x validate + 1x persist + 3x add
    try expect(contact_listener.containsSubShape(.persist, floor.getID(), .empty, body_id, sub_shape_ids[3]));
    try expect(contact_listener.containsSubShape(.add, floor.getID(), .empty, body_id, sub_shape_ids[0]));
    try expect(contact_listener.containsSubShape(.add, floor.getID(), .empty, body_id, sub_shape_ids[1]));
    try expect(contact_listener.containsSubShape(.add, floor.getID(), .empty, body_id, sub_shape_ids[2]));
    contact_listener.clear();

    // Now enable manifold reduction again
    try c.getBodyInterface().setUseManifoldReduction(body_id, true);

    // Trigger contact callbacks
    _ = try ctx.simulateSingleStep();

    // We should be back to the first state now where we only have 1 contact
    try expect(contact_listener.getEntryCount() == 8); // 4x validate + 1x persist + 3x remove
    try expect(contact_listener.containsSubShape(.persist, floor.getID(), .empty, body_id, sub_shape_ids[3]));
    try expect(contact_listener.containsSubShape(.remove, floor.getID(), .empty, body_id, sub_shape_ids[0]));
    try expect(contact_listener.containsSubShape(.remove, floor.getID(), .empty, body_id, sub_shape_ids[1]));
    try expect(contact_listener.containsSubShape(.remove, floor.getID(), .empty, body_id, sub_shape_ids[2]));
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
        try c.getSystem().optimizeBroadPhase();
    }
}

// Tests that colliding dynamic bodies are put in the same simulation island and that kinematic and static bodies are not
test "TestContactsVsSimulationIslands" {
    const motion_types = [_]MotionType{ .static, .kinematic, .dynamic };
    for (motion_types) |m1| {
        for (motion_types) |m2| {
            for (motion_types) |m3| {
                // Create 3 boxes with different motion types that are all touching each other
                var c: PhysicsTestContext = undefined;
                try c.init(std.testing.allocator, .{});
                defer c.deinit();
                const b1 = try c.createBox(RVec3.init(0, 0, 0), Quat.identity(), m1, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
                const b2 = try c.createBox(rvec3f(1.99, 0, 0), Quat.identity(), m2, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
                const b3 = try c.createBox(rvec3f(3.98, 0, 0), Quat.identity(), m3, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

                // Simulate a step to make sure we properly create simulation islands with different motion types in them
                _ = try c.simulateSingleStep();

                const get_island_index = struct {
                    fn call(body: *const Body) u32 {
                        return if (body.isStatic()) MotionProperties.inactive_index else body.getMotionProperties().getIslandIndexInternal();
                    }
                }.call;

                const island_index1 = get_island_index(b1);
                const island_index2 = get_island_index(b2);
                const island_index3 = get_island_index(b3);

                if (m1 == .dynamic and m2 == .dynamic) {
                    // Two dynamic bodies should always be in the same island
                    try expect(island_index1 != MotionProperties.inactive_index);
                    try expect(island_index1 == island_index2);
                } else if (m1 == .dynamic or m1 == .kinematic) {
                    // A kinematic body or a dynamic body that is touching a kinematic/static body should be in an island of its own
                    try expect(island_index1 != MotionProperties.inactive_index);
                    try expect(island_index1 != island_index2);
                    try expect(island_index1 != island_index3);
                } else {
                    // Static bodies should not be in an island
                    try expect(island_index1 == MotionProperties.inactive_index);
                }

                if (m3 == .dynamic and m2 == .dynamic) {
                    // Two dynamic bodies should always be in the same island
                    try expect(island_index3 != MotionProperties.inactive_index);
                    try expect(island_index2 == island_index3);
                } else if (m3 == .dynamic or m3 == .kinematic) {
                    // A kinematic body or a dynamic body that is touching a kinematic/static body should be in an island of its own
                    try expect(island_index3 != MotionProperties.inactive_index);
                    try expect(island_index3 != island_index2);
                    try expect(island_index3 != island_index1);
                } else {
                    // Static bodies should not be in an island
                    try expect(island_index3 == MotionProperties.inactive_index);
                }
            }
        }
    }
}

// Tests applying body creation settings after the body has been created
test "TestApplyBodyCreationSettings" {
    const allocator = std.testing.allocator;
    var ctx: ListenerContext = undefined;
    try ctx.init(.{});
    defer ctx.deinit();
    const c = &ctx.c;

    const contact_listener = &ctx.contact_listener;
    ctx.registerContactListener();

    // Create box intersecting with the floor and take one step
    const floor_id = (try c.createFloor()).getID();
    const box = try c.createBox(RVec3.init(0, 0.99, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
    const box_id = box.getID();
    _ = try ctx.simulateSingleStep();

    // We should have a contact with the floor
    try expect(contact_listener.containsSubShape(.add, floor_id, .empty, box_id, .empty));
    contact_listener.clear();

    // Remove the body from the system
    const bi = c.getBodyInterface();
    bi.deactivateBody(box_id);
    bi.removeBody(box_id);

    // Contact removals are not processed until the next step
    try expect(contact_listener.getEntryCount() == 0);

    // Now change the body using ApplyBodyCreationSettings, use non default parameters for everything
    var bcs = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.init(1.0, 0.99, 0), Quat.rotation(Vec3.axisX(), 0.1), .kinematic, Layers.moving);
    defer bcs.deinit();
    bcs.object_layer = Layers.moving2;
    bcs.collision_group = CollisionGroup.init((try GroupFilterTable.create(allocator, .{ .num_sub_groups = 10 })).asGroupFilter(), 1, 2);
    bcs.linear_velocity = Vec3.init(-0.1, -0.2, -0.3);
    bcs.angular_velocity = Vec3.init(0.1, 0.2, 0.3);
    bcs.user_data = 1234;
    bcs.allow_dynamic_or_kinematic = true;
    bcs.is_sensor = true;
    bcs.collide_kinematic_vs_non_dynamic = true;
    bcs.use_manifold_reduction = false;
    bcs.apply_gyroscopic_force = true;
    bcs.motion_quality = .linear_cast;
    bcs.enhanced_internal_edge_removal = true;
    bcs.allow_sleeping = false;
    bcs.friction = 0.3;
    bcs.restitution = 0.4;
    bcs.linear_damping = 0.01;
    bcs.angular_damping = 0.02;
    bcs.max_linear_velocity = 100.0;
    bcs.max_angular_velocity = math.pi * 60.0;
    bcs.gravity_factor = 0.5;
    bcs.num_velocity_steps_override = 1;
    bcs.num_position_steps_override = 3;
    bcs.override_mass_properties = .mass_and_inertia_provided;
    bcs.mass_properties_override.mass = 4.0;
    bcs.mass_properties_override.inertia = Mat44.scale(8.0);
    try box.applyBodyCreationSettings(allocator, &bcs, c.getSystem().getBroadPhaseLayerInterface());

    // Check that the settings have been applied
    var bcs_check = box.getBodyCreationSettings();
    defer bcs_check.deinit();
    try expect(bcs_check.eql(&bcs));

    // Add the body back to the system
    try bi.addBody(box_id, .activate);

    _ = try ctx.simulateSingleStep();

    // We should still have a contact with the floor (we didn't invalidate the contact, so the system doesn't know we changed shape)
    try expect(contact_listener.containsSubShape(.persist, floor_id, .empty, box_id, .empty));
    contact_listener.clear();

    // Remove and destroy the body
    bi.deactivateBody(box_id);
    bi.removeBody(box_id);
    bi.destroyBody(box_id);

    _ = try ctx.simulateSingleStep();

    // We should receive a contact removal event
    try expect(contact_listener.containsSubShape(.remove, floor_id, .empty, box_id, .empty));
    contact_listener.clear();
}
