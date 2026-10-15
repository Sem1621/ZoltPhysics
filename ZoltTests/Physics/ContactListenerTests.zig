//! Port of: UnitTests/Physics/ContactListenerTests.cpp
//! Status: complete
//!
//! - The callbacks cannot fail a test: Jolt's CHECK (and JPH_ASSERT) inside a callback is remembered by the listener
//!   and reported after the update (`LoggingContactListener.checkError`, `checkError` of the local listeners).
//! - The `ContactListenerImpl` classes are local structs registered with `ContactListener.init(&listener)` (pattern B),
//!   `PhysicsSystem *` / `Body &` members are pointers.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;

const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const MotionQuality = zolt.MotionQuality;
const MotionType = zolt.MotionType;
const PhysicsSystem = zolt.PhysicsSystem;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Real = zolt.Real;
const Ref = zolt.Ref;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDPair = zolt.SubShapeIDPair;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

// Gravity vector
const gravity = Vec3.init(0.0, -9.81, 0.0);

const LogEntry = LoggingContactListener.LogEntry;

/// c.Simulate(time) with the CHECKs that the listener made inside its callbacks
fn simulate(c: *PhysicsTestContext, listener: *const LoggingContactListener, total_time: f32) !void {
    _ = try c.simulate(total_time, {});
    try listener.checkError();
}

/// c.SimulateSingleStep() with the CHECKs that the listener made inside its callbacks
fn simulateSingleStep(c: *PhysicsTestContext, listener: *const LoggingContactListener) !void {
    _ = try c.simulateSingleStep();
    try listener.checkError();
}

/// Square(cPenetrationSlop) as the max squared distance of RVec3::IsClose (float, promoted to Real)
fn slopSq(penetration_slop: f32) Real {
    return math.square(penetration_slop);
}

// Let a sphere bounce on the floor with restitution = 1
test "TestContactListenerElastic" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    const simulation_time: f32 = 1.0;
    const distance_traveled = c.predictPosition(RVec3.zero(), Vec3.zero(), gravity, simulation_time);
    const floor_hit_epsilon: f32 = 1.0e-4; // Apply epsilon so that we're sure that the collision algorithm will find a collision
    const floor_hit_pos = RVec3.init(0.0, 1.0 - floor_hit_epsilon, 0.0); // Sphere with radius 1 will hit floor when 1 above the floor
    const initial_pos = floor_hit_pos.sub(distance_traveled);
    const penetration_slop = c.getSystem().getPhysicsSettings().penetration_slop;

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Create sphere
    const floor = try c.createFloor();
    const body = try c.createSphere(initial_pos, 1.0, .dynamic, .discrete, Layers.moving, .{});
    body.setRestitution(1.0);
    try expect(floor.getID().lessThan(body.getID()));

    // Simulate until at floor
    try simulate(&c, &listener, simulation_time);

    // Assert collision not yet processed
    try expect(listener.getEntryCount() == 0);

    // Simulate one more step to process the collision
    try simulate(&c, &listener, c.getDeltaTime());

    // We expect a validate and a contact point added message
    try expect(listener.getEntryCount() == 2);
    {
        // Check validate callback
        const validate = listener.getEntry(0);
        try expect(validate.type == .validate);
        try expect(validate.body1.eql(body.getID())); // Dynamic body should always be the 1st
        try expect(validate.body2.eql(floor.getID()));

        // Check add contact callback
        const add_contact = listener.getEntry(1);
        try expect(add_contact.type == .add);
        try expect(add_contact.body1.eql(floor.getID())); // Lowest ID should be first
        try expect(add_contact.manifold.sub_shape_id1.getValue() == SubShapeID.empty.getValue()); // Floor doesn't have any sub shapes
        try expect(add_contact.body2.eql(body.getID())); // Highest ID should be second
        try expect(add_contact.manifold.sub_shape_id2.getValue() == SubShapeID.empty.getValue()); // Sphere doesn't have any sub shapes
        try checkApproxEqual(add_contact.manifold.world_space_normal, Vec3.axisY(), .{}); // Normal should move body 2 out of collision
        try expect(add_contact.manifold.relative_contact_points_on1.len == 1);
        try expect(add_contact.manifold.relative_contact_points_on2.len == 1);
        try expect(add_contact.manifold.getWorldSpaceContactPointOn1(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
        try expect(add_contact.manifold.getWorldSpaceContactPointOn2(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
    }
    listener.clear();

    // Simulate same time, with a fully elastic body we should reach the initial position again
    try simulate(&c, &listener, simulation_time);

    // We should only have a remove contact point
    try expect(listener.getEntryCount() == 1);
    {
        // Check remove contact callback
        const remove = listener.getEntry(0);
        try expect(remove.type == .remove);
        try expect(remove.body1.eql(floor.getID())); // Lowest ID should be first
        try expect(remove.body2.eql(body.getID())); // Highest ID should be second
    }
}

/// The CHECKs of an added / persisted contact between the floor and the sphere (TestContactListenerInelastic)
fn checkFloorContact(entry: *const LogEntry, floor: *const Body, body: *const Body) !void {
    try expect(entry.body1.eql(floor.getID())); // Lowest ID first
    try expect(entry.manifold.sub_shape_id1.getValue() == SubShapeID.empty.getValue()); // Floor doesn't have any sub shapes
    try expect(entry.body2.eql(body.getID())); // Highest ID second
    try expect(entry.manifold.sub_shape_id2.getValue() == SubShapeID.empty.getValue()); // Sphere doesn't have any sub shapes
    try checkApproxEqual(entry.manifold.world_space_normal, Vec3.axisY(), .{}); // Normal should move body 2 out of collision
    try expect(entry.manifold.relative_contact_points_on1.len == 1);
    try expect(entry.manifold.relative_contact_points_on2.len == 1);
}

// Let a sphere fall on the floor with restitution = 0, then give it horizontal velocity, then take it away from the floor
test "TestContactListenerInelastic" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    const simulation_time: f32 = 1.0;
    const distance_traveled = c.predictPosition(RVec3.zero(), Vec3.zero(), gravity, simulation_time);
    const floor_hit_epsilon: f32 = 1.0e-4; // Apply epsilon so that we're sure that the collision algorithm will find a collision
    const floor_hit_pos = RVec3.init(0.0, 1.0 - floor_hit_epsilon, 0.0); // Sphere with radius 1 will hit floor when 1 above the floor
    const initial_pos = floor_hit_pos.sub(distance_traveled);
    const penetration_slop = c.getSystem().getPhysicsSettings().penetration_slop;

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Create sphere
    const floor = try c.createFloor();
    const body = try c.createSphere(initial_pos, 1.0, .dynamic, .discrete, Layers.moving, .{});
    body.setRestitution(0.0);
    body.setAllowSleeping(false);
    try expect(floor.getID().lessThan(body.getID()));

    // Simulate until at floor
    try simulate(&c, &listener, simulation_time);

    // Assert collision not yet processed
    try expect(listener.getEntryCount() == 0);

    // Simulate one more step to process the collision
    try simulate(&c, &listener, c.getDeltaTime());
    try checkApproxEqual(body.getPosition(), floor_hit_pos, .{ .tolerance = penetration_slop });

    // We expect a validate and a contact point added message
    try expect(listener.getEntryCount() == 2);
    {
        // Check validate callback
        const validate = listener.getEntry(0);
        try expect(validate.type == .validate);
        try expect(validate.body1.eql(body.getID())); // Dynamic body should always be the 1st
        try expect(validate.body2.eql(floor.getID()));

        // Check add contact callback
        const add_contact = listener.getEntry(1);
        try expect(add_contact.type == .add);
        try checkFloorContact(add_contact, floor, body);
        try expect(add_contact.manifold.getWorldSpaceContactPointOn1(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
        try expect(add_contact.manifold.getWorldSpaceContactPointOn2(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
    }
    listener.clear();

    // Simulate 10 steps
    try simulate(&c, &listener, 10 * c.getDeltaTime());
    try checkApproxEqual(body.getPosition(), floor_hit_pos, .{ .tolerance = penetration_slop });

    // We're not moving, we should have persisted contacts only
    try expect(listener.getEntryCount() == 10);
    for (0..listener.getEntryCount()) |i| {
        // Check persist callback
        const persist_contact = listener.getEntry(i);
        try expect(persist_contact.type == .persist);
        try checkFloorContact(persist_contact, floor, body);
        try expect(persist_contact.manifold.getWorldSpaceContactPointOn1(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
        try expect(persist_contact.manifold.getWorldSpaceContactPointOn2(0).isClose(RVec3.zero(), .{ .max_dist_sq = slopSq(penetration_slop) }));
    }
    listener.clear();

    // Make the body able to go to sleep
    body.setAllowSleeping(true);

    // Let the body go to sleep
    try simulate(&c, &listener, 1.0);
    try checkApproxEqual(body.getPosition(), floor_hit_pos, .{ .tolerance = penetration_slop });

    // Check it went to sleep and that we received a contact removal callback
    try expect(!body.isActive());
    try expect(listener.getEntryCount() > 0);
    for (0..listener.getEntryCount()) |i| {
        // Check persist / removed callbacks
        const entry = listener.getEntry(i);
        try expect(entry.body1.eql(floor.getID()));
        try expect(entry.body2.eql(body.getID()));
        try expect(entry.type == if (i == listener.getEntryCount() - 1) LoggingContactListener.Type.remove else LoggingContactListener.Type.persist); // The last entry should remove the contact as the body went to sleep
    }
    listener.clear();

    // Wake the body up again
    c.getBodyInterface().activateBody(body.getID());
    try expect(body.isActive());

    // Simulate 1 time step to detect the collision with the floor again
    try simulateSingleStep(&c, &listener);

    // Check that the contact got readded
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, floor.getID(), body.getID()));
    try expect(listener.contains(.add, floor.getID(), body.getID()));
    listener.clear();

    // Prevent body from going to sleep again
    body.setAllowSleeping(false);

    // Make the sphere move horizontal
    body.setLinearVelocity(Vec3.axisX());

    // Simulate 10 steps
    try simulate(&c, &listener, 10 * c.getDeltaTime());

    // We should have 10 persisted contacts events
    var validate: i32 = 0;
    var persisted: i32 = 0;
    for (0..listener.getEntryCount()) |i| {
        const entry = listener.getEntry(i);
        switch (entry.type) {
            .validate => validate += 1,

            .persist => {
                // Check persist callback
                try checkFloorContact(entry, floor, body);
                try expect(@abs(entry.manifold.getWorldSpaceContactPointOn1(0).getY()) < penetration_slop);
                try expect(@abs(entry.manifold.getWorldSpaceContactPointOn2(0).getY()) < penetration_slop);
                persisted += 1;
            },

            .add, .remove => try expect(false), // Unexpected event
        }
    }
    try expect(validate <= 10); // We may receive extra validate callbacks when the object is moving
    try expect(persisted == 10);
    listener.clear();

    // Move the sphere away from the floor
    c.getBodyInterface().setPosition(body.getID(), initial_pos, .activate);

    // Simulate 10 steps
    try simulate(&c, &listener, 10 * c.getDeltaTime());

    // We should only have a remove contact point
    try expect(listener.getEntryCount() == 1);
    {
        // Check remove contact callback
        const remove = listener.getEntry(0);
        try expect(remove.type == .remove);
        try expect(remove.body1.eql(floor.getID())); // Lowest ID first
        try expect(remove.body2.eql(body.getID())); // Highest ID second
    }
}

test "TestWereBodiesInContact" {
    var sign: i32 = -1;
    while (sign <= 1) : (sign += 2) {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        const s = c.getSystem();
        const bi = c.getBodyInterface();

        const floor = try c.createFloor();

        // Two spheres at a distance so that when one sphere leaves the floor the body can still be touching the floor with the other sphere
        const compound_settings = try StaticCompoundShapeSettings.create(allocator);
        var compound_shape = Ref(ShapeSettings).init(compound_settings.asShapeSettings());
        defer compound_shape.deinit();
        try compound_settings.base.addShapePtr(Vec3.init(-2, 0, 0), Quat.identity(), (try SphereShape.create(allocator, 1, .{})).asShape(), .{});
        try compound_settings.base.addShapePtr(Vec3.init(2, 0, 0), Quat.identity(), (try SphereShape.create(allocator, 1, .{})).asShape(), .{});
        var bcs = BodyCreationSettings.initWithSettings(compound_shape.get(), RVec3.init(0, 0.999, 0), Quat.identity(), .dynamic, Layers.moving);
        defer bcs.deinit();
        const body = (try bi.createBody(&bcs)) orelse return error.OutOfBodies;
        try bi.addBody(body.getID(), .activate);

        const ContactListenerImpl = struct {
            system: *const PhysicsSystem,

            added: i32 = 0,

            removed: i32 = 0,
            was_in_contact: bool = false,

            /// Zolt: a CHECK inside a callback failed
            check_failed: bool = false,

            pub fn onContactAdded(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                _ = .{ body1, body2, manifold, settings };
                self.added += 1;
            }

            pub fn onContactRemoved(self: *@This(), sub_shape_pair: *const SubShapeIDPair) void {
                self.removed += 1;
                self.was_in_contact = self.system.wereBodiesInContact(sub_shape_pair.getBody1ID(), sub_shape_pair.getBody2ID());
                if (self.was_in_contact != self.system.wereBodiesInContact(sub_shape_pair.getBody2ID(), sub_shape_pair.getBody1ID())) // Returned value should be the same regardless of order
                    self.check_failed = true;
            }

            fn getAddCount(self: *const @This()) i32 {
                return self.added - self.removed;
            }

            fn reset(self: *@This()) void {
                self.added = 0;
                self.removed = 0;
                self.was_in_contact = false;
            }
        };

        // Set listener
        var listener: ContactListenerImpl = .{ .system = s };
        s.setContactListener(.init(&listener));

        // If the simulation hasn't run yet, we can't be in contact
        try expect(!s.wereBodiesInContact(floor.getID(), body.getID()));

        // Step the simulation to allow detecting the contact
        _ = try c.simulateSingleStep();

        // Should be in contact now
        try expect(s.wereBodiesInContact(floor.getID(), body.getID()));
        try expect(s.wereBodiesInContact(body.getID(), floor.getID()));
        try expect(listener.getAddCount() == 1);
        listener.reset();

        // Impulse on one side
        bi.addImpulseAtPosition(body.getID(), Vec3.init(0, 10000, 0), RVec3.init(@floatFromInt(-sign * 2), 0, 0));
        _ = try c.simulateSingleStep(); // One step to detach from the ground (but starts penetrating so will not send a remove callback)
        try expect(listener.getAddCount() == 0);
        _ = try c.simulateSingleStep(); // One step to get contact remove callback

        // Should still be in contact
        // Note that we may get a remove and an add callback because manifold reduction has combined the collision with both spheres into 1 contact manifold.
        // At that point it has to select one of the sub shapes for the contact and if that sub shape no longer collides we get a remove for this sub shape and then an add callback for the other sub shape.
        try expect(s.wereBodiesInContact(floor.getID(), body.getID()));
        try expect(s.wereBodiesInContact(body.getID(), floor.getID()));
        try expect(listener.getAddCount() == 0);
        try expect(listener.removed == 0 or listener.was_in_contact);
        listener.reset();

        // Impulse on the other side
        bi.addImpulseAtPosition(body.getID(), Vec3.init(0, 10000, 0), RVec3.init(@floatFromInt(sign * 2), 0, 0));
        _ = try c.simulateSingleStep(); // One step to detach from the ground (but starts penetrating so will not send a remove callback)
        try expect(listener.getAddCount() == 0);
        _ = try c.simulateSingleStep(); // One step to get contact remove callback

        // Should no longer be in contact
        try expect(!s.wereBodiesInContact(floor.getID(), body.getID()));
        try expect(!s.wereBodiesInContact(body.getID(), floor.getID()));
        try expect(listener.getAddCount() == -1);
        try expect(listener.removed == 1 and !listener.was_in_contact);

        try expect(!listener.check_failed);
    }
}

test "TestSurfaceVelocity" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    const floor = try c.createBox(RVec3.init(0, -1, 0), Quat.rotation(Vec3.axisY(), math.degreesToRadians(10.0)), .static, .discrete, Layers.non_moving, Vec3.init(100.0, 1.0, 100.0), .{});
    floor.setFriction(1.0);

    for (0..2) |iteration| {
        const box = try c.createBox(RVec3.init(0, 0.999, 0), Quat.rotation(Vec3.axisY(), math.degreesToRadians(30.0)), .dynamic, .discrete, Layers.moving, Vec3.one(), .{});
        box.setFriction(1.0);

        // Contact listener sets a constant surface velocity
        const ContactListenerImpl = struct {
            floor: *const Body,
            box: *const Body,
            local_space_linear_velocity: Vec3 = undefined,
            local_space_angular_velocity: Vec3 = undefined,

            /// Zolt: the JPH_ASSERT inside the callback failed
            check_failed: bool = false,

            pub fn onContactAdded(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                _ = manifold;

                // Ensure that the body order is as expected
                if (!(body1.getID().eql(self.floor.getID()) or body2.getID().eql(self.box.getID())))
                    self.check_failed = true;

                // Calculate the relative surface velocity
                settings.relative_linear_surface_velocity = body1.getRotation().mulVec3(self.local_space_linear_velocity).negate();
                settings.relative_angular_surface_velocity = body1.getRotation().mulVec3(self.local_space_angular_velocity).negate();
            }

            pub fn onContactPersisted(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                self.onContactAdded(body1, body2, manifold, settings);
            }
        };

        // Set listener
        var listener: ContactListenerImpl = .{ .floor = floor, .box = box };
        c.getSystem().setContactListener(.init(&listener));

        // Set linear velocity or angular velocity depending on the iteration
        listener.local_space_linear_velocity = if (iteration == 0) Vec3.init(0, 0, -2.0) else Vec3.zero();
        listener.local_space_angular_velocity = if (iteration == 0) Vec3.zero() else Vec3.init(0, math.degreesToRadians(30.0), 0);

        // Simulate
        _ = try c.simulate(5.0, {});
        try expect(!listener.check_failed);

        // Check that the box is moving with the correct linear/angular velocity
        try checkApproxEqual(box.getLinearVelocity(), floor.getRotation().mulVec3(listener.local_space_linear_velocity), .{ .tolerance = 0.005 });
        try checkApproxEqual(box.getAngularVelocity(), floor.getRotation().mulVec3(listener.local_space_angular_velocity), .{ .tolerance = 2.0e-4 });
    }
}

fn getInvMassScale(body: *const Body) f32 {
    const ud = body.getUserData();
    const index = (if ((ud & 1) != 0) (ud >> 1) else (ud >> 3)) & 0b11;
    const mass_overrides = [_]f32{ 1.0, 0.0, 0.5, 2.0 };
    return mass_overrides[@intCast(index)];
}

const motion_types = [_]MotionType{ .static, .kinematic, .dynamic };

test "TestMassOverride" {
    for (motion_types) |m1|
        for (motion_types) |m2|
            if (m1 != .static or m2 != .static) {
                for (0..16) |i| {
                    var c: PhysicsTestContext = undefined;
                    try c.init(allocator, .{});
                    defer c.deinit();
                    c.zeroGravity();

                    const initial_velocity1: f32 = if (m1 != .static) 3.0 else 0.0;
                    const initial_velocity2: f32 = if (m2 != .static) -4.0 else 0.0;

                    // Create two spheres on a collision course
                    var bcs = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), m1, if (m1 != .static) Layers.moving else Layers.non_moving);
                    defer bcs.deinit();
                    bcs.override_mass_properties = .calculate_inertia;
                    bcs.mass_properties_override.mass = 1.0;
                    bcs.restitution = 1.0;
                    bcs.linear_damping = 0.0;
                    bcs.position = RVec3.init(-2, 0, 0);
                    bcs.linear_velocity = Vec3.init(initial_velocity1, 0, 0);
                    bcs.user_data = @as(u64, i) << 1;
                    const body1 = (try c.getBodyInterface().createBody(&bcs)) orelse return error.OutOfBodies;
                    try c.getBodyInterface().addBody(body1.getID(), .activate);

                    bcs.motion_type = m2;
                    bcs.object_layer = if (m2 != .static) Layers.moving else Layers.non_moving;
                    bcs.mass_properties_override.mass = 2.0;
                    bcs.position = RVec3.init(2, 0, 0);
                    bcs.linear_velocity = Vec3.init(initial_velocity2, 0, 0);
                    bcs.user_data += 1;
                    const body2 = (try c.getBodyInterface().createBody(&bcs)) orelse return error.OutOfBodies;
                    try c.getBodyInterface().addBody(body2.getID(), .activate);

                    // Contact listener that modifies mass
                    const ContactListenerImpl = struct {
                        pub fn onContactAdded(self: *@This(), body_a: *const Body, body_b: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                            _ = .{ self, manifold };

                            // Override the mass of body 1
                            const scale1 = getInvMassScale(body_a);
                            settings.inv_mass_scale1 = scale1;
                            settings.inv_inertia_scale1 = scale1;

                            // Override the mass of body 2
                            const scale2 = getInvMassScale(body_b);
                            settings.inv_mass_scale2 = scale2;
                            settings.inv_inertia_scale2 = scale2;
                        }

                        pub fn onContactPersisted(self: *@This(), body_a: *const Body, body_b: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                            self.onContactAdded(body_a, body_b, manifold, settings);
                        }
                    };

                    // Set listener
                    var listener: ContactListenerImpl = .{};
                    c.getSystem().setContactListener(.init(&listener));

                    // Simulate
                    _ = try c.simulate(1.0, {});

                    // Calculate resulting inverse mass
                    const inv_m1: f32 = if (body1.getMotionType() == .dynamic) getInvMassScale(body1) * body1.getMotionProperties().getInverseMass() else 0.0;
                    const inv_m2: f32 = if (body2.getMotionType() == .dynamic) getInvMassScale(body2) * body2.getMotionProperties().getInverseMass() else 0.0;

                    var v1: f32 = undefined;
                    var v2: f32 = undefined;
                    if (inv_m1 == 0.0 and inv_m2 == 0.0) {
                        // If both bodies became kinematic they will pass through each other
                        v1 = initial_velocity1;
                        v2 = initial_velocity2;
                    } else {
                        // Calculate resulting velocity using conservation of momentum and energy
                        // See: https://en.wikipedia.org/wiki/Elastic_collision where m1 = 1 / inv_m1 and m2 = 1 / inv_m2
                        v1 = (2.0 * inv_m1 * initial_velocity2 + (inv_m2 - inv_m1) * initial_velocity1) / (inv_m1 + inv_m2);
                        v2 = (2.0 * inv_m2 * initial_velocity1 + (inv_m1 - inv_m2) * initial_velocity2) / (inv_m1 + inv_m2);
                    }

                    // Check that the spheres move according to their overridden masses
                    try checkApproxEqual(body1.getLinearVelocity(), Vec3.init(v1, 0, 0), .{});
                    try checkApproxEqual(body2.getLinearVelocity(), Vec3.init(v2, 0, 0), .{});
                }
            };
}

test "TestInfiniteMassOverride" {
    for ([_]bool{ false, true }) |do_swap|
        for ([_]MotionQuality{ .discrete, .linear_cast }) |quality| {
            // A contact listener that makes a body have infinite mass
            const ContactListenerImpl = struct {
                body_id: BodyID,

                pub fn onContactAdded(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                    _ = manifold;
                    if (self.body_id.eql(body1.getID())) {
                        settings.inv_inertia_scale1 = 0.0;
                        settings.inv_mass_scale1 = 0.0;
                    } else if (self.body_id.eql(body2.getID())) {
                        settings.inv_inertia_scale2 = 0.0;
                        settings.inv_mass_scale2 = 0.0;
                    }
                }

                pub fn onContactPersisted(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                    self.onContactAdded(body1, body2, manifold, settings);
                }
            };

            var c: PhysicsTestContext = undefined;
            try c.init(allocator, .{});
            defer c.deinit();
            c.zeroGravity();

            // Create a box
            const initial_box_pos = RVec3.init(0, 2, 0);
            var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(2), .{})).asShape(), initial_box_pos, Quat.identity(), .dynamic, Layers.moving);
            defer box_settings.deinit();
            box_settings.override_mass_properties = .calculate_inertia;
            box_settings.mass_properties_override.mass = 1.0;

            // Create a sphere
            var sphere_settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 2, .{})).asShape(), RVec3.init(30, 2, 0), Quat.identity(), .dynamic, Layers.moving);
            defer sphere_settings.deinit();
            sphere_settings.linear_velocity = Vec3.init(-100, 0, 0);
            sphere_settings.override_mass_properties = .calculate_inertia;
            sphere_settings.mass_properties_override.mass = 10.0;
            sphere_settings.restitution = 0.1;
            sphere_settings.linear_damping = 0.0;
            sphere_settings.motion_quality = quality;

            var box_id: BodyID = undefined;
            var sphere_id: BodyID = undefined;
            if (do_swap) {
                // Swap the bodies so that the contact listener will receive the bodies in the opposite order
                sphere_id = try c.getBodyInterface().createAndAddBody(&sphere_settings, .activate);
                box_id = try c.getBodyInterface().createAndAddBody(&box_settings, .activate);
            } else {
                box_id = try c.getBodyInterface().createAndAddBody(&box_settings, .activate);
                sphere_id = try c.getBodyInterface().createAndAddBody(&sphere_settings, .activate);
            }

            // Add listener that will make the box have infinite mass
            var listener: ContactListenerImpl = .{ .body_id = box_id };
            c.getSystem().setContactListener(.init(&listener));

            // Simulate
            const simulation_time: f32 = 0.3;
            _ = try c.simulate(simulation_time, {});

            // Check that the box didn't move
            const bi = c.getBodyInterface();
            try expect(bi.getPosition(box_id).eql(initial_box_pos));
            try expect(bi.getLinearVelocity(box_id).eql(Vec3.zero()));
            try expect(bi.getAngularVelocity(box_id).eql(Vec3.zero()));

            // Check that the sphere bounced off the box
            try checkApproxEqual(bi.getLinearVelocity(sphere_id), sphere_settings.linear_velocity.negate().mulScalar(sphere_settings.restitution), .{});
        };
}

test "TestCollideKinematicVsNonDynamic" {
    for (motion_types) |m1|
        for (0..2) |allow1|
            for (0..2) |active1|
                for (motion_types) |m2|
                    for (0..2) |allow2|
                        for (0..2) |active2|
                            if ((m1 != .static and active1 != 0) or (m2 != .static and active2 != 0)) {
                                var c: PhysicsTestContext = undefined;
                                try c.init(allocator, .{});
                                defer c.deinit();
                                c.zeroGravity();

                                const initial_velocity1 = Vec3.init(if (m1 != .static and active1 != 0) 1.0 else 0.0, 0, 0);
                                const initial_velocity2 = Vec3.init(if (m2 != .static and active2 != 0) -1.0 else 0.0, 0, 0);

                                // Create two spheres that are colliding initially
                                var bcs = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 1.0, .{})).asShape(), RVec3.zero(), Quat.identity(), m1, if (m1 != .static) Layers.moving else Layers.non_moving);
                                defer bcs.deinit();
                                bcs.position = RVec3.init(-0.5, 0, 0);
                                bcs.linear_velocity = initial_velocity1;
                                bcs.collide_kinematic_vs_non_dynamic = allow1 != 0;
                                const body1 = (try c.getBodyInterface().createBody(&bcs)) orelse return error.OutOfBodies;
                                try c.getBodyInterface().addBody(body1.getID(), if (active1 != 0) .activate else .dont_activate);

                                bcs.motion_type = m2;
                                bcs.object_layer = if (m2 != .static) Layers.moving else Layers.non_moving;
                                bcs.position = RVec3.init(0.5, 0, 0);
                                bcs.linear_velocity = initial_velocity2;
                                bcs.collide_kinematic_vs_non_dynamic = allow2 != 0;
                                const body2 = (try c.getBodyInterface().createBody(&bcs)) orelse return error.OutOfBodies;
                                try c.getBodyInterface().addBody(body2.getID(), if (active2 != 0) .activate else .dont_activate);

                                // Set listener
                                var listener: LoggingContactListener = .init(allocator);
                                defer listener.deinit();
                                c.getSystem().setContactListener(.init(&listener));

                                // Step
                                try simulateSingleStep(&c, &listener);

                                if ((allow1 != 0 or allow2 != 0) // In this case we always get a callback
                                or (m1 == .dynamic or m2 == .dynamic)) // Otherwise we only get a callback when one of the bodies is dynamic
                                {
                                    // Check that we received a callback
                                    try expect(listener.getEntryCount() == 2);
                                    try expect(listener.contains(.validate, body1.getID(), body2.getID()));
                                    try expect(listener.contains(.add, body1.getID(), body2.getID()));
                                } else {
                                    // No collision events should have been received
                                    try expect(listener.getEntryCount() == 0);
                                }

                                // Velocities should only change if the body is dynamic
                                if (m1 == .dynamic) {
                                    try expect(!body1.getLinearVelocity().eql(initial_velocity1));
                                    try expect(body1.isActive());
                                } else try expect(body1.getLinearVelocity().eql(initial_velocity1));

                                if (m2 == .dynamic) {
                                    try expect(!body2.getLinearVelocity().eql(initial_velocity2));
                                    try expect(body2.isActive());
                                } else try expect(body2.getLinearVelocity().eql(initial_velocity2));
                            };
}

// Test that an update with zero delta time doesn't generate contact callbacks
test "TestZeroDeltaTime" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Create a sphere that intersects with the floor
    const floor = try c.createFloor();
    const body1 = try c.createSphere(RVec3.zero(), 1.0, .dynamic, .discrete, Layers.moving, .{});

    // Step with zero delta time
    _ = try c.getSystem().update(0.0, 1, c.getTempAllocator(), c.getJobSystem());
    try listener.checkError();

    // No callbacks should trigger when delta time is zero
    try expect(listener.getEntryCount() == 0);

    // Simulate for 1 step
    try simulateSingleStep(&c, &listener);

    // We expect a validate and a contact point added message
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, floor.getID(), body1.getID()));
    try expect(listener.contains(.add, floor.getID(), body1.getID()));
    listener.clear();

    // Create a 2nd sphere that intersects with the floor
    const body2 = try c.createSphere(RVec3.init(4, 0, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});

    // Step with zero delta time
    _ = try c.getSystem().update(0.0, 1, c.getTempAllocator(), c.getJobSystem());
    try listener.checkError();

    // No callbacks should trigger when delta time is zero
    try expect(listener.getEntryCount() == 0);

    // Simulate for 1 step
    try simulateSingleStep(&c, &listener);

    // We expect callbacks for both bodies now
    try expect(listener.getEntryCount() == 4);
    try expect(listener.contains(.validate, floor.getID(), body1.getID()));
    try expect(listener.contains(.persist, floor.getID(), body1.getID()));
    try expect(listener.contains(.validate, floor.getID(), body2.getID()));
    try expect(listener.contains(.add, floor.getID(), body2.getID()));
}
