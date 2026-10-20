//! Port of: UnitTests/Physics/SoftBodyTests.cpp
//! Status: complete
//!
//! The soft body jobs of PhysicsSystem.update end to end (JobSoftBodyPrepare, Collide, Simulate and Finalize) and the
//! response of a linear cast body against a soft body face (RemainingFeaturesArchitecture.md R17, R18). The values are
//! Jolt's. The six systems of TestLinearCastSphereVsSoftBody share one cube, like Jolt's `RefConst` made outside the
//! loop.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;
const LoggingSoftBodyContactListener = @import("../LoggingSoftBodyContactListener.zig").LoggingSoftBodyContactListener;

const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const MotionQuality = zolt.MotionQuality;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const SoftBodyCreationSettings = zolt.SoftBodyCreationSettings;
const SoftBodyMotionProperties = zolt.SoftBodyMotionProperties;
const SoftBodySharedSettings = zolt.SoftBodySharedSettings;
const SphereShape = zolt.SphereShape;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

/// static_cast<SoftBodyMotionProperties *>(body.GetMotionProperties())
fn softBodyMotionProperties(body: *Body) *SoftBodyMotionProperties {
    return SoftBodyMotionProperties.fromBase(body.getMotionPropertiesMut());
}

test "TestBendConstraint" {
    // Possible values for x3
    const x3_values = [_]Float3{
        .{ .x = 0, .y = 0, .z = 1 }, // forming flat plane
        .{ .x = 0, .y = 0, .z = -1 }, // overlapping
        .{ .x = 0, .y = 1, .z = 0 }, // 90 degrees concave
        .{ .x = 0, .y = -1, .z = 0 }, // 90 degrees convex
        .{ .x = 0, .y = 1, .z = 1 }, // 45 degrees concave
        .{ .x = 0, .y = -1, .z = -1 }, // 135 degrees convex
    };

    for (x3_values) |x3| {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        const s = c.getSystem();
        const bi = s.getBodyInterfaceMut();

        // Create settings
        var shared_settings_ref: Ref(SoftBodySharedSettings) = .init(try SoftBodySharedSettings.create(allocator));
        defer shared_settings_ref.deinit();
        const shared_settings = shared_settings_ref.get().?;

        // Create two triangles with a shared edge, x3 = free, the rest is locked
        //    x2
        // e1/  \e3
        //  /    \
        // x0----x1
        //  \ e0 /
        // e2\  /e4
        //    x3
        var v: SoftBodySharedSettings.Vertex = .{};
        v.position = .{ .x = -1, .y = 0, .z = 0 };
        v.inv_mass = 0;
        try shared_settings.vertices.append(allocator, v);
        v.position = .{ .x = 1, .y = 0, .z = 0 };
        try shared_settings.vertices.append(allocator, v);
        v.position = .{ .x = 0, .y = 0, .z = -1 };
        try shared_settings.vertices.append(allocator, v);
        v.position = x3;
        v.inv_mass = 1;
        try shared_settings.vertices.append(allocator, v);

        // Create the 2 triangles
        try shared_settings.addFace(.init(0, 1, 2, .{}));
        try shared_settings.addFace(.init(0, 3, 1, .{}));

        // Create edge and dihedral constraints
        var va: SoftBodySharedSettings.VertexAttributes = .{};
        va.shear_compliance = math.flt_max;
        va.bend_compliance = 0;
        try shared_settings.createConstraints((&va)[0..1], .{ .bend_type = .dihedral });

        // Optimize the settings
        try shared_settings.optimize();

        // Create the soft body
        var sb_settings = SoftBodyCreationSettings.init(shared_settings, RVec3.zero(), Quat.identity(), Layers.moving);
        defer sb_settings.deinit();
        sb_settings.gravity_factor = 0.0;
        sb_settings.allow_sleeping = false;
        sb_settings.update_position = false;
        const body = (try bi.createSoftBody(&sb_settings)).?;
        try bi.addBody(body.getID(), .activate);
        const mp = softBodyMotionProperties(body);

        // Test 4 angles to see if there are singularities (the dot product between the triangles has the same value for 2 configurations)
        for ([_]f32{ 0.0, 90.0, 180.0, 270.0 }) |angle| {
            // Perturb x3
            const perturbed_x3 = Vec3.fromFloat3(x3);
            mp.getVertexMut(3).position = Mat44.rotationX(math.degreesToRadians(angle)).mulVec3(perturbed_x3).mulScalar(0.5);

            // Simulate
            _ = try c.simulate(0.25, {});

            // Should return to the original position
            try checkApproxEqual(mp.getVertex(3).position, Vec3.fromFloat3(x3), .{ .tolerance = 1.0e-3 });
        }
    }
}

// Test that applying a force to a soft body and rigid body of the same mass has the same effect
test "TestApplyForce" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const s = c.getSystem();
    const bi = s.getBodyInterfaceMut();

    // Soft body cube
    var cube = try SoftBodySharedSettings.createCube(allocator, 6, 0.2);
    defer cube.deinit();
    var sb_box_settings = SoftBodyCreationSettings.init(cube.get(), RVec3.zero(), Quat.identity(), Layers.moving);
    defer sb_box_settings.deinit();
    sb_box_settings.gravity_factor = 0.0;
    sb_box_settings.linear_damping = 0.0;
    const sb_box = (try bi.createSoftBody(&sb_box_settings)).?;
    const sb_id = sb_box.getID();
    try bi.addBody(sb_id, .activate);
    const mass: f32 = 216; // 6 * 6 * 6 * 1 kg
    try checkApproxEqual(sb_box.getMotionProperties().getInverseMass(), 1.0 / mass, .{});

    // Rigid body cube of same size and mass
    const rb_box_pos = RVec3.init(0, 2, 0);
    var rb_box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), rb_box_pos, Quat.identity(), .dynamic, Layers.moving);
    defer rb_box_settings.deinit();
    rb_box_settings.gravity_factor = 0.0;
    rb_box_settings.linear_damping = 0.0;
    rb_box_settings.override_mass_properties = .calculate_inertia;
    rb_box_settings.mass_properties_override.mass = mass;
    const rb_box = (try bi.createBody(&rb_box_settings)).?;
    const rb_id = rb_box.getID();
    try bi.addBody(rb_id, .activate);

    // Simulate for 3 seconds while applying the same force
    const num_steps = 180;
    const force = Vec3.init(10000.0, 0, 0);
    for (0..num_steps) |_| {
        bi.addForce(sb_id, force, .{ .activation = .activate });
        bi.addForce(rb_id, force, .{ .activation = .activate });
        _ = try c.simulateSingleStep();
    }

    // Check that the rigid body moved as expected
    const total_time = @as(f32, num_steps) * c.getStepDeltaTime();
    const acceleration = force.divScalar(mass);
    const expected_pos = c.predictPosition(rb_box_pos, Vec3.zero(), acceleration, total_time);
    try checkApproxEqual(rb_box.getPosition(), expected_pos, .{});
    const expected_vel = acceleration.mulScalar(total_time);
    try checkApproxEqual(rb_box.getLinearVelocity(), expected_vel, .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(rb_box.getAngularVelocity(), Vec3.zero(), .{});

    // Check that the soft body moved within 1% of that
    const expected_pos_sb = expected_pos.sub(rb_box_pos);
    try checkApproxEqual(sb_box.getPosition(), expected_pos_sb, .{ .tolerance = 0.01 * expected_pos_sb.length() });
    try checkApproxEqual(sb_box.getLinearVelocity(), expected_vel, .{ .tolerance = 2.0e-3 });
    try checkApproxEqual(sb_box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 0.01 });
}

// Test that a rigid body sphere with high speed tunnels through a soft body cube when using discrete motion quality,
// but collides and transfers momentum correctly when using linear cast motion quality. Test that we can affect
// this behavior with the soft body contact listener.
test "TestLinearCastSphereVsSoftBody" {
    const delta_time: f32 = 1.0 / 60.0;
    const sphere_mass: f32 = 200.0;
    const soft_body_mass: f32 = 6.0 * 6.0 * 6.0; // * 1 kg

    // Sphere starts at x = 2 and moves by -5 in one time step, towards the soft body cube at the origin (spans [-0.5, 0.5])
    // Velocity is high enough to tunnel through the cube in one simulation step
    const sphere_start_pos = RVec3.init(2.0, 0, 0);
    const sphere_velocity = Vec3.init(-5.0 / delta_time, 0, 0);

    // Expensive to make, create outside of loop
    var cube: RefConst(SoftBodySharedSettings) = blk: {
        var created = try SoftBodySharedSettings.createCube(allocator, 6, 0.2);
        defer created.deinit();
        break :blk .init(created.get());
    };
    defer cube.deinit();

    const Test = enum {
        no_contact_listener,
        with_contact_listener,
        with_rejecting_contact_listener,
    };

    for ([_]Test{ .no_contact_listener, .with_contact_listener, .with_rejecting_contact_listener }) |t| {
        for ([_]MotionQuality{ .discrete, .linear_cast }) |motion_quality| {
            var c: PhysicsTestContext = undefined;
            try c.init(allocator, .{ .delta_time = delta_time, .collision_steps = 1 });
            defer c.deinit();
            c.zeroGravity();
            const bi = c.getBodyInterface();

            // Create rigid body sphere approaching the cube
            var sphere_settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), sphere_start_pos, Quat.identity(), .dynamic, Layers.moving);
            defer sphere_settings.deinit();
            sphere_settings.motion_quality = motion_quality;
            sphere_settings.override_mass_properties = .calculate_inertia;
            sphere_settings.mass_properties_override.mass = sphere_mass;
            sphere_settings.linear_velocity = sphere_velocity;
            sphere_settings.linear_damping = 0.0;
            const sphere = try c.createBody(&sphere_settings, .activate);
            try checkApproxEqual(sphere.getMotionProperties().getInverseMass(), 1.0 / sphere_mass, .{});

            // Create soft body cube at origin (size 1 with 5 subdivisions)
            var sb_settings = SoftBodyCreationSettings.init(cube.get(), RVec3.zero(), Quat.identity(), Layers.moving);
            defer sb_settings.deinit();
            const soft_body = (try bi.createSoftBody(&sb_settings)).?;
            try bi.addBody(soft_body.getID(), .activate);
            try checkApproxEqual(soft_body.getMotionProperties().getInverseMass(), 1.0 / soft_body_mass, .{});

            var listener: LoggingContactListener = .init(allocator);
            defer listener.deinit();
            c.getSystem().setContactListener(.init(&listener));

            // Optionally install a soft body contact listener
            var soft_body_listener: LoggingSoftBodyContactListener = .init(allocator);
            defer soft_body_listener.deinit();
            if (t != .no_contact_listener)
                c.getSystem().setSoftBodyContactListener(.init(&soft_body_listener));
            if (t == .with_rejecting_contact_listener)
                soft_body_listener.setValidateValueToReturn(.reject_contact);

            // Simulate one step
            _ = try c.simulateSingleStep();
            try listener.checkError();
            try soft_body_listener.checkError();

            // We shouldn't be receiving any contacts on the normal contact listener
            try expect(listener.getEntryCount() == 0);

            if (motion_quality == .discrete or t == .with_rejecting_contact_listener) {
                // With discrete motion  quality / rejected contacts the sphere should have tunneled through the soft body without any collision
                try checkApproxEqual(sphere.getPosition(), sphere_start_pos.addVec3(sphere_velocity.mulScalar(delta_time)), .{});
                try checkApproxEqual(sphere.getLinearVelocity(), sphere_velocity, .{});
                try expect(soft_body.getLinearVelocity().eql(Vec3.zero()));

                // No contacts should have been detected
                if (motion_quality == .discrete) {
                    try expect(soft_body_listener.getEntryCount() == 0);
                } else {
                    try expect(soft_body_listener.contains(.validate, soft_body.getID(), sphere.getID())); // Should have gotten a validate that we rejected
                    try expect(!soft_body_listener.contains(.add, soft_body.getID(), sphere.getID()));
                }
            } else { // LinearCast
                // With linear cast motion quality the sphere should have collided with the soft body and stopped there (we do time stealing)
                try checkApproxEqual(sphere.getPosition(), RVec3.init(1.5, 0, 0), .{ .tolerance = 1.001 * c.getSystem().getPhysicsSettings().penetration_slop });

                // The soft body should have gained velocity in the direction the sphere was traveling
                try expect(soft_body.getLinearVelocity().getX() < 0.0);

                // Check that momentum is conserved (GetLinearVelocity returns the center of mass velocity of the soft body)
                // Note that because we apply the collision impulse to a face, some rotational velocity will be introduced and we lose some linear momentum
                const initial_momentum = sphere_mass * sphere_velocity.getX();
                const final_momentum = sphere_mass * sphere.getLinearVelocity().getX() + soft_body_mass * soft_body.getLinearVelocity().getX();
                try checkApproxEqual(final_momentum, initial_momentum, .{ .tolerance = 0.05 * @abs(initial_momentum) });

                if (t == .no_contact_listener) {
                    try expect(soft_body_listener.getEntryCount() == 0);
                } else {
                    // We should have received one contact for the soft body
                    try expect(soft_body_listener.contains(.validate, soft_body.getID(), sphere.getID()));
                    try expect(soft_body_listener.contains(.add, soft_body.getID(), sphere.getID()));
                }
            }
        }
    }
}
