//! Zolt addition, no Jolt file: tests of MotionProperties, Body and BodyCreationSettings (Phase 5 F1)
//!
//! Jolt has no unit test file for these classes (its body tests need a PhysicsSystem, Wave C). These tests check the
//! behavior with hand-computed values; ZoltParity/Physics/BodyParity.zig compares the bits with the C++ library. Some
//! are adapted from the Phase 5 architecture prototype (ZoltTests/Prototype/SimulationPrototype.zig).

const std = @import("std");
const zolt = @import("zolt");
const BodyTestContext = @import("BodyTestContext.zig").BodyTestContext;
const Layers = @import("../Layers.zig").Layers;
const checkApproxEqual = @import("../UnitTestFramework.zig").checkApproxEqual;

const Allocator = std.mem.Allocator;
const AllowedDOFs = zolt.AllowedDOFs;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const Core = zolt.Core;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const MotionProperties = zolt.MotionProperties;
const OffsetCenterOfMassShape = zolt.OffsetCenterOfMassShape;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

/// Motion properties of a dynamic rigid body (the debug-only cached types satisfy the asserts)
fn dynamicMotionProperties() MotionProperties {
    var mp: MotionProperties = .{};
    if (Core.enable_asserts) {
        mp.cached_body_type = .rigid_body;
        mp.cached_motion_type = .dynamic;
    }
    mp.max_linear_velocity = 500.0;
    mp.max_angular_velocity = 100.0;
    mp.linear_damping = 0.0;
    mp.angular_damping = 0.0;
    mp.gravity_factor = 1.0;
    mp.motion_quality = .discrete;
    mp.allow_sleeping = true;
    return mp;
}

/// Mass 2 with a rotated diagonal inertia tensor (1, 2, 3)
fn testMassProperties() MassProperties {
    const rotation = Mat44.rotationQuat(Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.7));
    return .{ .mass = 2.0, .inertia = rotation.mul(Mat44.scaleVec3(Vec3.init(1, 2, 3))).mul(rotation.transposed()) };
}

test "MotionProperties SetMassProperties with every AllowedDOFs combination" {
    const mass_properties = testMassProperties();
    const principal = mass_properties.decomposePrincipalMomentsOfInertia().?;
    const body_rotation = Quat.rotation(Vec3.init(-1, 0.5, 2).normalized(), 1.3);

    for (1..64) |bits| {
        const dofs = AllowedDOFs.fromInt(@intCast(bits));
        var mp = dynamicMotionProperties();
        mp.setMassProperties(dofs, &mass_properties);
        try expect(mp.getAllowedDOFs().eql(dofs));

        // Mass
        const translation_allowed = (bits & 0b111) != 0;
        try expectEqual(@as(f32, if (translation_allowed) 0.5 else 0.0), mp.getInverseMass());

        // Inertia
        const rotation_allowed = (bits >> 3) != 0;
        if (rotation_allowed) {
            try expect(mp.getInverseInertiaDiagonal().eql(principal.diagonal.reciprocal()));
            try expect(mp.getInertiaRotation().eql(principal.rotation.getQuaternion()));
            try checkApproxEqual(mp.getLocalSpaceInverseInertia(), mass_properties.inertia.inversed3x3(), .{ .tolerance = 1.0e-5 });
        } else {
            try expect(mp.getInverseInertiaDiagonal().eql(Vec3.zero()));
            try expect(mp.getInertiaRotation().eql(Quat.identity()));
        }

        // The DOF masks and the locked vectors
        const v = Vec3.init(1, 2, 3);
        const linear = mp.lockTranslation(v);
        const angular = mp.lockAngular(v);
        try expectEqual(@as(f32, if (dofs.translation_x) 1 else 0), linear.getX());
        try expectEqual(@as(f32, if (dofs.translation_y) 2 else 0), linear.getY());
        try expectEqual(@as(f32, if (dofs.translation_z) 3 else 0), linear.getZ());
        try expectEqual(@as(f32, if (dofs.rotation_x) 1 else 0), angular.getX());
        try expectEqual(@as(f32, if (dofs.rotation_y) 2 else 0), angular.getY());
        try expectEqual(@as(f32, if (dofs.rotation_z) 3 else 0), angular.getZ());
        try expectEqual(@as(u32, if (dofs.translation_x) 0xffffffff else 0), mp.getLinearDOFsMask().getX());
        try expectEqual(@as(u32, if (dofs.rotation_z) 0xffffffff else 0), mp.getAngularDOFsMask().getZ());

        // Rows and columns of the locked rotation axes are zero
        const inv_inertia = mp.getInverseInertiaForRotation(Mat44.rotationQuat(body_rotation));
        const allowed = [3]bool{ dofs.rotation_x, dofs.rotation_y, dofs.rotation_z };
        for (0..3) |r| {
            for (0..3) |c| {
                if (!allowed[r] or !allowed[c])
                    try expectEqual(@as(f32, 0), inv_inertia.get(@intCast(r), @intCast(c)));
            }
        }
        const w = mp.multiplyWorldSpaceInverseInertiaByVector(body_rotation, Vec3.init(0.3, -0.4, 0.5));
        for (0..3) |i| {
            if (!allowed[i])
                try expectEqual(@as(f32, 0), w.getComponent(@intCast(i)));
        }
        if (rotation_allowed) {
            // The masked world space inverse inertia is the same as multiplying with the masked matrix
            try checkApproxEqual(w, inv_inertia.multiply3x3(Vec3.bitAnd(Vec3.init(0.3, -0.4, 0.5), Vec3.fromVec4(mp.getAngularDOFsMask().reinterpretAsFloat()))), .{ .tolerance = 1.0e-5 });
        }
    }

    // A tensor that cannot be decomposed (all zero): the inertia of a sphere with radius 1
    var mp = dynamicMotionProperties();
    const zero_inertia: MassProperties = .{ .mass = 4.0, .inertia = Mat44.zero() };
    mp.setMassProperties(.all, &zero_inertia);
    try expect(mp.getInverseInertiaDiagonal().eql(Vec3.replicate(2.5 * 0.25)));
    try expect(mp.getInertiaRotation().eql(Quat.identity()));

    // ScaleToMass scales the inverse inertia with the mass ratio
    mp.setMassProperties(.all, &mass_properties);
    const diagonal = mp.getInverseInertiaDiagonal();
    mp.scaleToMass(4.0);
    try expectEqual(@as(f32, 0.25), mp.getInverseMass());
    try expect(mp.getInverseInertiaDiagonal().eql(diagonal.mulScalar(0.25 / 0.5)));

    // SetInverseMass / SetInverseInertia
    mp.setInverseMass(0.125);
    try expectEqual(@as(f32, 0.125), mp.getInverseMassUnchecked());
    mp.setInverseInertia(Vec3.init(1, 2, 3), Quat.identity());
    try checkApproxEqual(mp.getLocalSpaceInverseInertiaUnchecked(), Mat44.scaleVec3(Vec3.init(1, 2, 3)), .{});
}

test "MotionProperties forces, torques, damping and velocity clamping" {
    var mp = dynamicMotionProperties();
    mp.setInverseMass(0.5);
    mp.setInverseInertia(Vec3.replicate(1.0), Quat.identity());
    mp.setLinearDamping(0.1);
    mp.setAngularDamping(0.2);
    mp.setGravityFactor(2.0);
    mp.setLinearVelocity(Vec3.init(1, 0, 0));
    mp.setAngularVelocity(Vec3.init(0, 0, 1));
    mp.force = .{ .x = 2, .y = 0, .z = 0 };
    mp.torque = .{ .x = 0, .y = 4, .z = 0 };
    try expect(mp.getAccumulatedForce().eql(Vec3.init(2, 0, 0)));
    try expect(mp.getAccumulatedTorque().eql(Vec3.init(0, 4, 0)));

    // v = (v + dt * (gravity factor * g + inv mass * F)) * (1 - linear damping * dt), w = (w + dt * I^-1 T) * (1 - angular damping * dt)
    mp.applyForceTorqueAndDragInternal(Quat.identity(), Vec3.init(0, -10, 0), 0.5);
    try checkApproxEqual(mp.getLinearVelocity(), Vec3.init(1.5 * 0.95, -10 * 0.95, 0), .{});
    try checkApproxEqual(mp.getAngularVelocity(), Vec3.init(0, 2 * 0.9, 1 * 0.9), .{});

    // Forces are not reset by the integration, ResetForce / ResetTorque / ResetMotion do that
    mp.resetForce();
    try expect(mp.getAccumulatedForce().eql(Vec3.zero()));
    try expect(mp.getAccumulatedTorque().eql(Vec3.init(0, 4, 0)));
    mp.resetTorque();
    try expect(mp.getAccumulatedTorque().eql(Vec3.zero()));

    // Damping never makes the velocity flip sign (max(0, 1 - c dt))
    mp.setLinearDamping(10.0);
    mp.applyForceTorqueAndDragInternal(Quat.identity(), Vec3.zero(), 0.5);
    try expect(mp.getLinearVelocity().eql(Vec3.zero()));

    // Clamping
    mp.setMaxLinearVelocity(10.0);
    mp.setMaxAngularVelocity(2.0);
    mp.setLinearVelocityClamped(Vec3.init(30, 40, 0));
    try checkApproxEqual(mp.getLinearVelocity(), Vec3.init(6, 8, 0), .{});
    mp.setAngularVelocityClamped(Vec3.init(0, 3, 4));
    try checkApproxEqual(mp.getAngularVelocity(), Vec3.init(0, 1.2, 1.6), .{});
    mp.setLinearVelocityClamped(Vec3.init(1, 2, 3)); // Below the maximum: unchanged
    try expect(mp.getLinearVelocity().eql(Vec3.init(1, 2, 3)));
    try expectEqual(@as(f32, 10.0), mp.getMaxLinearVelocity());
    try expectEqual(@as(f32, 2.0), mp.getMaxAngularVelocity());

    // Locked axes are respected by the setters and the velocity steps
    const plane_mass: MassProperties = testMassProperties();
    mp.setMassProperties(.plane_2d, &plane_mass);
    mp.setLinearVelocityClamped(Vec3.init(1, 2, 3));
    try expect(mp.getLinearVelocity().eql(Vec3.init(1, 2, 0)));
    mp.addLinearVelocityStep(Vec3.init(1, 1, 1));
    try expect(mp.getLinearVelocity().eql(Vec3.init(2, 3, 0)));
    mp.subLinearVelocityStep(Vec3.init(1, 1, 1));
    try expect(mp.getLinearVelocity().eql(Vec3.init(1, 2, 0)));
    mp.setAngularVelocityClamped(Vec3.init(1, 1, 1));
    try expect(mp.getAngularVelocity().eql(Vec3.init(0, 0, 1)));
    mp.addAngularVelocityStep(Vec3.init(0, 0, 0.5)); // ApplyAngularVelocityStep does not lock (Jolt)
    try expect(mp.getAngularVelocity().eql(Vec3.init(0, 0, 1.5)));
    mp.subAngularVelocityStep(Vec3.init(0, 0, 0.5));
    try expect(mp.getAngularVelocity().eql(Vec3.init(0, 0, 1)));

    // Point velocity: v + w x r
    try checkApproxEqual(mp.getPointVelocityCOM(Vec3.init(1, 0, 0)), Vec3.init(1, 3, 0), .{});

    // MoveKinematic: the velocities that move by the delta in the delta time
    mp.setMassProperties(.all, &plane_mass);
    mp.moveKinematic(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.1), 0.5);
    try checkApproxEqual(mp.getLinearVelocity(), Vec3.init(2, 4, 6), .{});
    try checkApproxEqual(mp.getAngularVelocity(), Vec3.init(0, 0, 0.2), .{ .tolerance = 1.0e-5 });

    mp.resetMotion();
    try expect(mp.getLinearVelocity().eql(Vec3.zero()) and mp.getAngularVelocity().eql(Vec3.zero()));

    // The solver step overrides
    mp.setNumVelocityStepsOverride(12);
    mp.setNumPositionStepsOverride(255);
    try expectEqual(@as(u32, 12), mp.getNumVelocityStepsOverride());
    try expectEqual(@as(u32, 255), mp.getNumPositionStepsOverride());
    mp.setIslandIndexInternal(7);
    try expectEqual(@as(u32, 7), mp.getIslandIndexInternal());
    try expectEqual(MotionProperties.inactive_index, mp.getIndexInActiveBodiesInternal());
}

test "MotionProperties gyroscopic force keeps the angular momentum" {
    var mp = dynamicMotionProperties();
    mp.setInverseInertia(Vec3.init(1, 0.5, 0.25), Quat.rotation(Vec3.axisY(), 0.3));
    mp.setAngularVelocity(Vec3.init(1, 2, 3));
    const body_rotation = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.4);

    // The magnitude of the local angular momentum is preserved by the step
    const inertia_to_world = body_rotation.mul(mp.getInertiaRotation());
    const momentum = Vec3.init(1, 2, 4).mul(inertia_to_world.inverseRotate(mp.getAngularVelocity()));
    mp.applyGyroscopicForceInternal(body_rotation, 0.1);
    const new_momentum = Vec3.init(1, 2, 4).mul(inertia_to_world.inverseRotate(mp.getAngularVelocity()));
    try checkApproxEqual(new_momentum.length(), momentum.length(), .{ .tolerance = 1.0e-4 });
    try expect(!new_momentum.isClose(momentum, .{}));

    // An axis without inertia (inverse inertia 0) and no angular velocity stays at zero
    mp.setInverseInertia(Vec3.init(1, 0, 1), Quat.identity());
    mp.setAngularVelocity(Vec3.zero());
    mp.applyGyroscopicForceInternal(Quat.identity(), 0.1);
    try expect(mp.getAngularVelocity().eql(Vec3.zero()));
}

test "MotionProperties sleep test state and state recorder bytes" {
    var mp = dynamicMotionProperties();
    const points = [3]RVec3{ RVec3.init(1, 2, 3), RVec3.init(2, 2, 3), RVec3.init(1, 3, 3) };
    mp.resetSleepTestSpheres(&points);
    try expectEqual(@as(f32, 0), mp.sleep_test_timer);
    for (mp.sleep_test_spheres) |sphere| try expectEqual(@as(f32, 0), sphere.getRadius());
    try expectEqual(zolt.CanSleep.cannot_sleep, mp.accumulateSleepTime(0.25, 0.5));
    try expectEqual(zolt.CanSleep.can_sleep, mp.accumulateSleepTime(0.25, 0.5));
    mp.resetSleepTestTimer();
    try expectEqual(zolt.CanSleep.cannot_sleep, mp.accumulateSleepTime(0.25, 0.5));

    // SaveState writes only the state that changes at runtime: velocities, force, torque, (offset,) spheres, timer, allow sleeping
    mp.setLinearVelocity(Vec3.init(1, 2, 3));
    var recorder = StateRecorderImpl.init(std.testing.allocator);
    defer recorder.deinit();
    mp.saveState(recorder.recorder());
    const expected_size: usize = 4 * 12 + (if (Core.double_precision) 24 else 0) + 3 * 16 + 4 + 1;
    try expectEqual(expected_size, recorder.getData().len);
    try expect(std.mem.eql(u8, recorder.getData()[0..12], std.mem.asBytes(&[3]f32{ 1, 2, 3 })));

    // Restore into other motion properties, then validate against the same state
    var restored = dynamicMotionProperties();
    restored.restoreState(recorder.recorder());
    try expect(restored.getLinearVelocity().eql(Vec3.init(1, 2, 3)));
    try expectEqual(mp.sleep_test_timer, restored.sleep_test_timer);
    recorder.rewind();
    recorder.base.setValidating(true);
    restored.restoreState(recorder.recorder());
    try expectEqual(@as(u32, 0), recorder.num_mismatches);
    try expect(!recorder.base.isFailed());
}

test "Body layout, getters and setters" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 8, 0);
    defer context.deinit();

    // A box with its center of mass at (0.5, 0, 0) relative to the shape origin
    const box = try BoxShape.create(allocator, Vec3.init(1, 2, 3), .{});
    const offset_box = try OffsetCenterOfMassShape.create(allocator, box.asShape(), Vec3.init(0.5, 0, 0));
    var settings = BodyCreationSettings.initWithShape(offset_box.asShape(), RVec3.init(10, 20, 30), Quat.rotation(Vec3.axisZ(), 0.5 * std.math.pi), .dynamic, Layers.moving);
    defer settings.deinit();
    settings.user_data = 0x1234;
    settings.friction = 0.7;
    settings.restitution = 0.3;
    settings.max_angular_velocity = 1000.0;

    const body = (try context.createAndAddBody(&settings, .dont_activate)).?;
    try expect(body.isRigidBody() and !body.isSoftBody());
    try expect(body.isDynamic() and !body.isStatic() and !body.isKinematic());
    try expect(body.canBeKinematicOrDynamic());
    try expect(body.isInBroadPhase() and !body.isActive());
    try expectEqual(@as(u64, 0x1234), body.getUserData());
    try expectEqual(@as(f32, 0.7), body.getFriction());
    try expectEqual(@as(f32, 0.3), body.getRestitution());
    try expectEqual(Layers.moving, body.getObjectLayer());
    try expect(body.getBroadPhaseLayer().eql(context.layer_interface.base.getBroadPhaseLayer(Layers.moving)));

    // Positions: the center of mass is rotated by 90 degrees around Z, so it is at (10, 20.5, 30)
    try checkApproxEqual(body.getCenterOfMassPosition(), RVec3.init(10, 20.5, 30), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getPosition(), RVec3.init(10, 20, 30), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getWorldTransform().getTranslation(), RVec3.init(10, 20, 30), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getCenterOfMassTransform().getTranslation(), RVec3.init(10, 20.5, 30), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getInverseCenterOfMassTransform().mul(body.getCenterOfMassTransform()).getTranslation(), RVec3.zero(), .{ .tolerance = 1.0e-5 });
    try expect(body.getWorldSpaceBounds().containsRVec3(body.getCenterOfMassPosition()));
    try checkApproxEqual(body.getWorldSpaceBounds().getSize(), Vec3.init(4, 2, 6), .{ .tolerance = 1.0e-4 });

    // Surface normal of the face at +X (local) is +Y in world space
    try checkApproxEqual(body.getWorldSpaceSurfaceNormal(.empty, RVec3.init(10, 21.5, 30)), Vec3.init(0, 1, 0), .{ .tolerance = 1.0e-5 });

    // Flags
    body.setIsSensor(true);
    body.setCollideKinematicVsNonDynamic(true);
    body.setUseManifoldReduction(false);
    body.setApplyGyroscopicForce(true);
    body.setEnhancedInternalEdgeRemoval(true);
    try expect(body.isSensor() and body.getCollideKinematicVsNonDynamic() and !body.getUseManifoldReduction() and body.getApplyGyroscopicForce() and body.getEnhancedInternalEdgeRemoval());
    try expect(body.isInBroadPhase());
    body.setIsSensor(false);
    try expect(!body.isSensor() and body.isInBroadPhase());
    try expect(!body.getUseManifoldReductionWithBody(Body.fixedToWorld()));
    try expect(body.getEnhancedInternalEdgeRemovalWithBody(Body.fixedToWorld()));
    try expect(!body.isCollisionCacheInvalid());
    try expect(body.invalidateContactCacheInternal());
    try expect(!body.invalidateContactCacheInternal());
    try expect(body.isCollisionCacheInvalid());
    body.validateContactCacheInternal();
    try expect(!body.isCollisionCacheInvalid());

    // Forces and impulses: inverse mass of a 2x4x6 box with density 1000
    const inv_mass = 1.0 / (2.0 * 4.0 * 6.0 * 1000.0);
    try checkApproxEqual(body.getMotionProperties().getInverseMass(), inv_mass, .{ .tolerance = 1.0e-9 });
    body.addForce(Vec3.init(1, 0, 0));
    body.addForceAtPosition(Vec3.init(0, 1, 0), body.getCenterOfMassPosition().addVec3(Vec3.init(1, 0, 0)));
    try expect(body.getAccumulatedForce().eql(Vec3.init(1, 1, 0)));
    try checkApproxEqual(body.getAccumulatedTorque(), Vec3.init(0, 0, 1), .{});
    body.addTorque(Vec3.init(0, 0, 1));
    try checkApproxEqual(body.getAccumulatedTorque(), Vec3.init(0, 0, 2), .{});
    body.resetForce();
    body.resetTorque();
    try expect(body.getAccumulatedForce().eql(Vec3.zero()) and body.getAccumulatedTorque().eql(Vec3.zero()));
    body.addImpulse(Vec3.init(48000, 0, 0));
    try checkApproxEqual(body.getLinearVelocity(), Vec3.init(1, 0, 0), .{ .tolerance = 1.0e-5 });
    body.addAngularImpulse(Vec3.init(0, 0, 1000));
    try expect(body.getAngularVelocity().getZ() > 0);
    try checkApproxEqual(body.getAngularVelocity(), body.getInverseInertia().multiply3x3(Vec3.init(0, 0, 1000)), .{ .tolerance = 1.0e-5 });
    body.resetMotion();
    body.addImpulseAtPosition(Vec3.init(0, 4800, 0), body.getCenterOfMassPosition().addVec3(Vec3.init(1, 0, 0)));
    try checkApproxEqual(body.getLinearVelocity(), Vec3.init(0, 0.1, 0), .{ .tolerance = 1.0e-5 });
    try expect(body.getAngularVelocity().getZ() > 0);
    try checkApproxEqual(body.getPointVelocity(body.getCenterOfMassPosition()), body.getLinearVelocity(), .{});
    body.setLinearVelocity(Vec3.init(1, 2, 3));
    body.setAngularVelocity(Vec3.init(0.1, 0.2, 0.3));
    try expect(body.getLinearVelocity().eql(Vec3.init(1, 2, 3)));
    body.setLinearVelocityClamped(Vec3.init(1000, 0, 0));
    try checkApproxEqual(body.getLinearVelocity(), Vec3.init(500, 0, 0), .{ .tolerance = 1.0e-3 });
    body.setAngularVelocityClamped(Vec3.init(0, 0, 2000));
    try checkApproxEqual(body.getAngularVelocity(), Vec3.init(0, 0, 1000), .{ .tolerance = 1.0e-3 });

    // Position steps
    body.resetMotion();
    const com = body.getCenterOfMassPosition();
    body.addPositionStep(Vec3.init(1, 2, 3));
    try checkApproxEqual(body.getCenterOfMassPosition(), com.addVec3(Vec3.init(1, 2, 3)), .{ .tolerance = 1.0e-5 });
    body.subPositionStep(Vec3.init(1, 2, 3));
    const rotation = body.getRotation();
    body.addRotationStep(Vec3.init(0, 0, 0.25));
    try checkApproxEqual(body.getRotation(), Quat.rotation(Vec3.axisZ(), 0.25).mul(rotation), .{ .tolerance = 1.0e-5 });
    body.subRotationStep(Vec3.init(0, 0, 0.25));
    try checkApproxEqual(body.getRotation(), rotation, .{ .tolerance = 1.0e-5 });
    body.addRotationStep(Vec3.init(1.0e-7, 0, 0)); // Below the threshold: no change
    try expect(body.getRotation().eql(rotation) or body.getRotation().isClose(rotation, .{}));

    // MoveKinematic with a kinematic body
    body.setMotionType(.kinematic);
    try expect(body.isKinematic());
    body.moveKinematic(RVec3.init(12, 20, 30), Quat.rotation(Vec3.axisZ(), 0.5 * std.math.pi), 0.5);
    try checkApproxEqual(body.getLinearVelocity(), Vec3.init(4, 0, 0), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(body.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-4 });

    // Back to static: the motion is stopped
    body.setMotionType(.static);
    try expect(body.getLinearVelocity().eql(Vec3.zero()));
    try expect(body.getMotionPropertiesUnchecked().?.getLinearVelocity().eql(Vec3.zero()));
    body.setMotionType(.dynamic);

    // SetPositionAndRotationInternal places the shape origin
    body.setPositionAndRotationInternal(RVec3.init(1, 1, 1), Quat.identity(), .{});
    try checkApproxEqual(body.getCenterOfMassPosition(), RVec3.init(1.5, 1, 1), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getPosition(), RVec3.init(1, 1, 1), .{ .tolerance = 1.0e-5 });

    // SetShapeInternal keeps the world position of the shape origin and recalculates the mass
    const sphere = try SphereShape.create(allocator, 2.0, .{});
    body.setShapeInternal(sphere.asShape(), true);
    try checkApproxEqual(body.getCenterOfMassPosition(), RVec3.init(1, 1, 1), .{ .tolerance = 1.0e-5 });
    try checkApproxEqual(body.getMotionProperties().getInverseMass(), 1.0 / sphere.asShape().getMassProperties().mass, .{ .tolerance = 1.0e-9 });
    try checkApproxEqual(body.getWorldSpaceBounds().getCenter(), Vec3.init(1, 1, 1), .{ .tolerance = 1.0e-5 });

    // The transformed shape refers to the shape and the body
    var transformed = body.getTransformedShape();
    defer transformed.deinit();
    try expect(transformed.shape.get().? == sphere.asShape());
    try expect(transformed.body_id.eql(body.getID()));

    // The collision group is a copy
    var group = zolt.CollisionGroup.init(null, 3, 4);
    body.setCollisionGroup(&group);
    try expectEqual(@as(u32, 3), body.getCollisionGroup().getGroupID());
    body.getCollisionGroupMut().setSubGroupID(5);
    try expectEqual(@as(u32, 5), body.getCollisionGroup().getSubGroupID());
}

test "Body sleep state and buoyancy" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 8, 0);
    defer context.deinit();

    const box = try BoxShape.create(allocator, Vec3.init(1, 0.5, 2), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(0, 0, 0), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const body = (try context.createAndAddBody(&settings, .activate)).?;
    try expect(body.isActive());

    // A body that doesn't move can sleep after the time before sleep
    try expectEqual(zolt.CanSleep.cannot_sleep, body.updateSleepStateInternal(0.25, 0.01, 0.5));
    try expectEqual(zolt.CanSleep.can_sleep, body.updateSleepStateInternal(0.25, 0.01, 0.5));

    // Moving more than the max movement resets the test
    body.addPositionStep(Vec3.init(0.1, 0, 0));
    try expectEqual(zolt.CanSleep.cannot_sleep, body.updateSleepStateInternal(0.25, 0.01, 0.5));
    try expectEqual(@as(f32, 0), body.getMotionProperties().sleep_test_timer);

    // Sensors and bodies that don't allow sleeping never sleep
    body.setAllowSleeping(false);
    try expect(!body.getAllowSleeping());
    try expectEqual(zolt.CanSleep.cannot_sleep, body.updateSleepStateInternal(10.0, 1.0, 0.5));
    body.setAllowSleeping(true);
    body.setIsSensor(true);
    try expectEqual(zolt.CanSleep.cannot_sleep, body.updateSleepStateInternal(10.0, 1.0, 0.5));
    body.setIsSensor(false);

    // Half submerged in water: the buoyancy pushes up
    body.setPositionAndRotationInternal(RVec3.zero(), Quat.identity(), .{});
    const volume = body.getSubmergedVolume(RVec3.zero(), Vec3.axisY());
    try checkApproxEqual(volume.total_volume, 8.0, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(volume.submerged_volume, 4.0, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(volume.relative_center_of_buoyancy, Vec3.init(0, -0.25, 0), .{ .tolerance = 1.0e-5 });
    try expect(body.applyBuoyancyImpulse(RVec3.zero(), Vec3.axisY(), 2.0, 0.5, 0.01, Vec3.zero(), Vec3.init(0, -10, 0), 1.0 / 60.0));
    try expect(body.getLinearVelocity().getY() > 0);

    // Out of the water: nothing happens
    body.resetMotion();
    try expect(!body.applyBuoyancyImpulse(RVec3.init(0, -10, 0), Vec3.axisY(), 2.0, 0.5, 0.01, Vec3.zero(), Vec3.init(0, -10, 0), 1.0 / 60.0));
    try expect(body.getLinearVelocity().eql(Vec3.zero()));
}

test "Body.findCollidingPairsCanCollide" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 8, 0);
    defer context.deinit();

    const sphere = try SphereShape.create(allocator, 1.0, .{});
    var settings = BodyCreationSettings.initWithShape(sphere.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const a = (try context.createAndAddBody(&settings, .activate)).?;
    const b = (try context.createAndAddBody(&settings, .activate)).?;
    const sleeping = (try context.createAndAddBody(&settings, .dont_activate)).?;
    settings.motion_type = .static;
    const static = (try context.createAndAddBody(&settings, .dont_activate)).?;
    settings.motion_type = .kinematic;
    const kinematic = (try context.createAndAddBody(&settings, .activate)).?;

    // A active body collides with bodies later in the active list, sleeping and static bodies, never with itself
    try expect(Body.findCollidingPairsCanCollide(a, b));
    try expect(!Body.findCollidingPairsCanCollide(b, a));
    try expect(Body.findCollidingPairsCanCollide(a, sleeping));
    try expect(Body.findCollidingPairsCanCollide(a, static));
    try expect(Body.findCollidingPairsCanCollide(kinematic, sleeping));

    // Kinematic vs static only with CollideKinematicVsNonDynamic, kinematic vs a sensor
    try expect(!Body.findCollidingPairsCanCollide(kinematic, static));
    kinematic.setCollideKinematicVsNonDynamic(true);
    try expect(Body.findCollidingPairsCanCollide(kinematic, static));
    kinematic.setCollideKinematicVsNonDynamic(false);
    static.setIsSensor(true);
    try expect(Body.findCollidingPairsCanCollide(kinematic, static));
    static.setIsSensor(false);

    // The collision group filters
    var group_filter = try zolt.GroupFilterTable.create(allocator, .{ .num_sub_groups = 2 });
    var group_filter_ref = zolt.Ref(zolt.GroupFilter).init(group_filter.asGroupFilterMut());
    defer group_filter_ref.deinit();
    group_filter.disableCollision(0, 1);
    var g1 = zolt.CollisionGroup.init(group_filter.asGroupFilter(), 0, 0);
    defer g1.deinit();
    var g2 = zolt.CollisionGroup.init(group_filter.asGroupFilter(), 0, 1);
    defer g2.deinit();
    a.setCollisionGroup(&g1);
    static.setCollisionGroup(&g2);
    try expect(!Body.findCollidingPairsCanCollide(a, static));
}

test "BodyCreationSettings shape union (Rule M) and CreateBodyError" {
    const allocator = std.testing.allocator;

    // A const BodyCreationSettings converts the settings on demand: the cache is written through the Ref (Rule M)
    const box_settings = try BoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), .{});
    var box_settings_ref = zolt.Ref(zolt.ShapeSettings).init(box_settings.asShapeSettings()); // Keep it alive after convertShapeSettings
    defer box_settings_ref.deinit();
    var settings = BodyCreationSettings.initWithSettings(box_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const const_settings: *const BodyCreationSettings = &settings;
    const shape1 = (try const_settings.getShape(allocator)).?;
    const shape2 = (try const_settings.getShape(allocator)).?;
    try expect(shape1 == shape2); // Cached in the ShapeSettings
    try expect(!box_settings.base.base.cached_result.isEmpty());
    try expect(const_settings.getShapeSettings().? == box_settings.asShapeSettings());

    // The mass properties for every mode
    settings.inertia_multiplier = 2.0;
    const calculated = try const_settings.getMassProperties(allocator);
    try checkApproxEqual(calculated.mass, 2.0 * 4.0 * 6.0 * 1000.0, .{ .tolerance = 1.0e-2 });
    try checkApproxEqual(calculated.inertia.getColumn4(0).getX(), 2.0 * shape1.getMassProperties().inertia.getColumn4(0).getX(), .{ .tolerance = 1.0 });
    try expectEqual(@as(f32, 1.0), calculated.inertia.get(3, 3));
    settings.override_mass_properties = .calculate_inertia;
    settings.mass_properties_override.mass = 10.0;
    const scaled = try const_settings.getMassProperties(allocator);
    try expectEqual(@as(f32, 10.0), scaled.mass);
    settings.override_mass_properties = .mass_and_inertia_provided;
    settings.mass_properties_override.inertia = Mat44.scaleVec3(Vec3.init(1, 2, 3));
    const provided = try const_settings.getMassProperties(allocator);
    try expect(provided.eql(&settings.mass_properties_override));

    // ConvertShapeSettings releases the settings and keeps the shape
    var result = try settings.convertShapeSettings(allocator);
    defer result.deinit();
    try expect(result.isValid());
    try expect(settings.getShapeSettings() == null);
    try expect((try const_settings.getShape(allocator)).? == shape1);
    var result2 = try settings.convertShapeSettings(allocator); // Already converted: returns the shape
    defer result2.deinit();
    try expect(result2.getPtr().? == shape1);

    // Equality compares the shape pointers
    var copy = settings.clone();
    defer copy.deinit();
    try expect(copy.eql(&settings));
    copy.setShapeSettings(box_settings.asShapeSettings());
    try expect(!copy.eql(&settings));
    copy.setShape(shape1);
    try expect(copy.eql(&settings));

    // No shape at all: Jolt's text from convertShapeSettings, null from getShape, error.InvalidShape from allocateBody
    var empty: BodyCreationSettings = .{};
    defer empty.deinit();
    var no_shape = try empty.convertShapeSettings(allocator);
    defer no_shape.deinit();
    try expect(no_shape.hasError());
    try std.testing.expectEqualStrings("No shape present!", no_shape.getError());
    try expect((try empty.getShape(allocator)) == null);
    try std.testing.expectError(error.InvalidShape, empty.getMassProperties(allocator));
    empty.override_mass_properties = .mass_and_inertia_provided; // Doesn't need the shape
    _ = try empty.getMassProperties(allocator);

    var context: BodyTestContext = undefined;
    try context.init(allocator, 4, 0);
    defer context.deinit();
    try std.testing.expectError(error.InvalidShape, context.body_manager.allocateBody(&empty));
    try expectEqual(@as(u32, 0), context.body_manager.getNumBodies());
}

test "Body.getBodyCreationSettings round trip" {
    const allocator = std.testing.allocator;
    var context: BodyTestContext = undefined;
    try context.init(allocator, 8, 0);
    defer context.deinit();

    const sphere_settings = try SphereShapeSettings.create(allocator, 0.5, .{});
    var settings = BodyCreationSettings.initWithSettings(sphere_settings.asShapeSettings(), RVec3.init(1, 2, 3), Quat.rotation(Vec3.axisX(), 0.3), .dynamic, Layers.moving);
    defer settings.deinit();
    settings.linear_velocity = Vec3.init(1, 0, 0);
    settings.angular_velocity = Vec3.init(0, 1, 0);
    settings.allowed_dofs = AllowedDOFs.all.bitXor(.{ .rotation_y = true });
    settings.motion_quality = .linear_cast;
    settings.gravity_factor = 0.5;
    settings.num_velocity_steps_override = 3;
    settings.is_sensor = true;
    settings.angular_velocity = Vec3.zero();
    const body = (try context.createAndAddBody(&settings, .activate)).?;

    // The settings that recreate this body (mass and inertia provided)
    var recreated = body.getBodyCreationSettings();
    defer recreated.deinit();
    try expect(recreated.override_mass_properties == .mass_and_inertia_provided);
    try expect(recreated.allowed_dofs.eql(settings.allowed_dofs));
    try expect(recreated.motion_quality == .linear_cast);
    try expect(recreated.is_sensor);
    try expectEqual(@as(u32, 3), recreated.num_velocity_steps_override);
    try expect((try recreated.getShape(allocator)).? == body.getShape());
    const body2 = (try context.createAndAddBody(&recreated, .activate)).?;
    try checkApproxEqual(body2.getPosition(), body.getPosition(), .{ .tolerance = 1.0e-5 });
    try expect(body2.getRotation().eql(body.getRotation()));
    try checkApproxEqual(body2.getMotionProperties().getInverseMass(), body.getMotionProperties().getInverseMass(), .{ .tolerance = 1.0e-9 });
    try checkApproxEqual(body2.getMotionProperties().getLocalSpaceInverseInertia(), body.getMotionProperties().getLocalSpaceInverseInertia(), .{ .tolerance = 1.0e-4 });
    try expect(body2.getLinearVelocity().eql(body.getLinearVelocity()));

    // A static body without motion properties
    settings.motion_type = .static;
    settings.is_sensor = false;
    const static = (try context.createAndAddBody(&settings, .dont_activate)).?;
    try expect(!static.canBeKinematicOrDynamic());
    var static_settings = static.getBodyCreationSettings();
    defer static_settings.deinit();
    try expectEqual(std.math.floatMax(f32), static_settings.mass_properties_override.mass);
    try expect(!static_settings.allow_dynamic_or_kinematic);
}
