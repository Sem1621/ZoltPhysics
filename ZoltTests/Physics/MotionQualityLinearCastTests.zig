//! Port of: UnitTests/Physics/MotionQualityLinearCastTests.cpp
//! Status: complete
//!
//! The linear cast motion quality end to end: JobFindCCDContacts, sSolveCCDContact and JobResolveCCDContacts (REGION R4
//! of PhysicsSystem.zig). The listeners check their contracts inside the callbacks; `checkError` reports a violation
//! after the update (Jolt's CHECK inside the callback).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;
const LoggingBodyActivationListener = @import("../LoggingBodyActivationListener.zig").LoggingBodyActivationListener;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;

const box_extent: f32 = 0.5;
const frequency: f32 = 60.0;
const velocity = Vec3.init(2.0 * frequency, 0, 0); // High enough velocity to step 2 meters in a single simulation step
const pos1 = RVec3.init(-1, 0, 0);
const pos2 = RVec3.init(1, 0, 0);

/// RVec3 + Vec3
fn addR(a: RVec3, b: Vec3) RVec3 {
    return a.addVec3(b);
}

/// RVec3 - Vec3
fn subR(a: RVec3, b: Vec3) RVec3 {
    return a.subVec3(b);
}

const LinearCastContext = struct {
    c: PhysicsTestContext,
    listener: LoggingContactListener,
    activation: LoggingBodyActivationListener,

    /// PhysicsTestContext c(1.0f / cFrequency, 1); c.ZeroGravity(); plus the listeners (in place)
    fn init(self: *LinearCastContext, register_activation_listener: bool) !void {
        const allocator = std.testing.allocator;
        try self.c.init(allocator, .{ .delta_time = 1.0 / frequency, .collision_steps = 1 });
        self.c.zeroGravity();

        // Register listener
        self.listener = .init(allocator);
        self.c.getSystem().setContactListener(.init(&self.listener));
        self.activation = .init(allocator);
        if (register_activation_listener)
            self.c.getSystem().setBodyActivationListener(.init(&self.activation));
    }

    fn deinit(self: *LinearCastContext) void {
        self.c.deinit();
        self.listener.deinit();
        self.activation.deinit();
    }

    fn penetrationSlop(self: *LinearCastContext) f32 {
        return self.c.getSystem().getPhysicsSettings().penetration_slop;
    }

    fn simulateSingleStep(self: *LinearCastContext) !void {
        _ = try self.c.simulateSingleStep();
        try self.listener.checkError();
        try self.activation.checkError();
    }
};

// Two boxes colliding in the center, each has enough velocity to tunnel though in 1 step
test "TestDiscreteBoxVsDiscreteBox" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(false);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    // Test that the inner radius of the box makes sense (used internally by linear cast)
    try checkApproxEqual(box1.getShape().getInnerRadius(), box_extent, .{});

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{});
    box2.setLinearVelocity(velocity.negate());

    try ctx.simulateSingleStep();

    // No collisions should be reported and the bodies should have moved according to their velocity (tunneling through each other)
    try expect(listener.getEntryCount() == 0);
    try checkApproxEqual(box1.getPosition(), addR(pos1, velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box1.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), subR(pos2, velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box2.getLinearVelocity(), velocity.negate(), .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}

// Two boxes colliding in the center, each has enough velocity to step over the other in 1 step, restitution = 1
test "TestLinearCastBoxVsLinearCastBoxElastic" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(false);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;

    const penetration_slop = ctx.penetrationSlop();

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);
    box1.setRestitution(1.0);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box2.setLinearVelocity(velocity.negate());
    box2.setRestitution(1.0);

    try ctx.simulateSingleStep();

    // The bodies should have collided and the velocities reversed
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), RVec3.init(-box_extent, 0, 0), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), velocity.negate(), .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), RVec3.init(box_extent, 0, 0), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box2.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});

    listener.clear();
    try ctx.simulateSingleStep();

    // In the second step the bodies should have moved away, but since they were initially overlapping we should have a contact persist callback
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.persist, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), subR(RVec3.init(-box_extent, 0, 0), velocity.divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), velocity.negate(), .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), addR(RVec3.init(box_extent, 0, 0), velocity.divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box2.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});

    listener.clear();
    try ctx.simulateSingleStep();

    // In the third step the bodies have separated and a contact remove callback should have been received
    try expect(listener.getEntryCount() == 1);
    try expect(listener.contains(.remove, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), subR(RVec3.init(-box_extent, 0, 0), velocity.mulScalar(2.0).divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), velocity.negate(), .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), addR(RVec3.init(box_extent, 0, 0), velocity.mulScalar(2.0).divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box2.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}

// Two boxes colliding in the center, each has enough velocity to step over the other in 1 step, restitution = 0
test "TestLinearCastBoxVsLinearCastBoxInelastic" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(false);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;

    const penetration_slop = ctx.penetrationSlop();

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box2.setLinearVelocity(velocity.negate());

    try ctx.simulateSingleStep();

    // The bodies should have collided and both are stopped
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), RVec3.init(-box_extent, 0, 0), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), RVec3.init(box_extent, 0, 0), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box2.getLinearVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});

    // The bodies should persist to contact as they are not moving
    for (0..10) |i| {
        listener.clear();
        try ctx.simulateSingleStep();

        if (i == 0) {
            // Only in the first step we will receive a validate callback since after this step the contact cache will be used
            try expect(listener.getEntryCount() == 2);
            try expect(listener.contains(.validate, box1.getID(), box2.getID()));
        } else try expect(listener.getEntryCount() == 1);
        try expect(listener.contains(.persist, box1.getID(), box2.getID()));
        try checkApproxEqual(box1.getPosition(), RVec3.init(-box_extent, 0, 0), .{ .tolerance = penetration_slop });
        try checkApproxEqual(box1.getLinearVelocity(), Vec3.zero(), .{});
        try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
        try checkApproxEqual(box2.getPosition(), RVec3.init(box_extent, 0, 0), .{ .tolerance = penetration_slop });
        try checkApproxEqual(box2.getLinearVelocity(), Vec3.zero(), .{});
        try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
    }
}

// Two boxes colliding in the center, linear cast vs inactive linear cast
test "TestLinearCastBoxVsInactiveLinearCastBox" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(true);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;
    const activation = &ctx.activation;

    const penetration_slop = ctx.penetrationSlop();

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{ .activation = .dont_activate });
    try expect(!box2.isActive());

    try ctx.simulateSingleStep();

    // The bodies should have collided and body 2 should be activated, have velocity, but not moved in this step
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    const new_velocity = velocity.mulScalar(0.5);
    try checkApproxEqual(box1.getPosition(), subR(pos2, Vec3.init(2.0 * box_extent, 0, 0)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), pos2, .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
    try expect(box2.isActive());
    try expect(activation.contains(.activated, box2.getID()));

    listener.clear();
    try ctx.simulateSingleStep();

    // In the next step body 2 should have started to move
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.persist, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), addR(subR(pos2, Vec3.init(2.0 * box_extent, 0, 0)), new_velocity.divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), addR(pos2, new_velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}

// Two boxes colliding in the center, linear cast vs inactive discrete
test "TestLinearCastBoxVsInactiveDiscreteBox" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(true);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;
    const activation = &ctx.activation;

    const penetration_slop = ctx.penetrationSlop();

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{ .activation = .dont_activate });
    try expect(!box2.isActive());

    try ctx.simulateSingleStep();

    // The bodies should have collided and body 2 should be activated, have velocity, but not moved in this step
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    const new_velocity = velocity.mulScalar(0.5);
    try checkApproxEqual(box1.getPosition(), subR(pos2, Vec3.init(2.0 * box_extent, 0, 0)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), pos2, .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
    try expect(box2.isActive());
    try expect(activation.contains(.activated, box2.getID()));

    listener.clear();
    try ctx.simulateSingleStep();

    // In the next step body 2 should have started to move
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.persist, box1.getID(), box2.getID()));
    try checkApproxEqual(box1.getPosition(), addR(subR(pos2, Vec3.init(2.0 * box_extent, 0, 0)), new_velocity.divScalar(frequency)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), addR(pos2, new_velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}

// Two boxes colliding under an angle, linear cast vs inactive discrete
test "TestLinearCastBoxVsInactiveDiscreteBoxAngled" {
    const angled_offset1 = Vec3.init(1, 0, -2);
    const angled_velocity = angled_offset1.mulScalar(-frequency * 2);

    var ctx: LinearCastContext = undefined;
    try ctx.init(true);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;
    const activation = &ctx.activation;

    const penetration_slop = ctx.penetrationSlop();

    // Make sure box1 exactly hits the face of box2 in the center
    const angled_pos1 = addR(RVec3.init(2.0 * box_extent, 0, 0), angled_offset1);
    const box1 = try c.createBox(angled_pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(angled_velocity);
    box1.setRestitution(1.0);
    box1.setFriction(0.0);

    const box2 = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{ .activation = .dont_activate });
    box2.setRestitution(1.0);
    box2.setFriction(0.0);
    try expect(!box2.isActive());

    try ctx.simulateSingleStep();

    // The bodies should have collided and body 2 should be activated, have inherited the x velocity of body 1, but not moved in this step. Body 1 should have lost all of its velocity in x direction.
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    const new_velocity1 = Vec3.init(0, 0, angled_velocity.getZ());
    const new_velocity2 = Vec3.init(angled_velocity.getX(), 0, 0);
    try checkApproxEqual(box1.getPosition(), RVec3.init(2.0 * box_extent, 0, 0), .{ .tolerance = 2.3 * penetration_slop }); // We're moving 2x as fast in the z direction and the slop is allowed in x direction: sqrt(1^2 + 2^2) ~ 2.3
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity1, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 2.0e-4 });
    try checkApproxEqual(box2.getPosition(), RVec3.zero(), .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity2, .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 2.0e-4 });
    try expect(box2.isActive());
    try expect(activation.contains(.activated, box2.getID()));
}

// Two boxes colliding in the center, linear cast vs fast moving discrete, should tunnel through because all discrete bodies are moved before linear cast bodies are tested
test "TestLinearCastBoxVsFastDiscreteBox" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(false);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{});
    box2.setLinearVelocity(velocity.negate());

    try ctx.simulateSingleStep();

    // No collisions should be reported and the bodies should have moved according to their velocity (tunneling through each other)
    try expect(listener.getEntryCount() == 0);
    try checkApproxEqual(box1.getPosition(), addR(pos1, velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box1.getLinearVelocity(), velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), subR(pos2, velocity.divScalar(frequency)), .{});
    try checkApproxEqual(box2.getLinearVelocity(), velocity.negate(), .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}

// Two boxes colliding in the center, linear cast vs moving discrete, discrete is slow enough not to tunnel through linear cast body
test "TestLinearCastBoxVsSlowDiscreteBox" {
    var ctx: LinearCastContext = undefined;
    try ctx.init(false);
    defer ctx.deinit();
    const c = &ctx.c;
    const listener = &ctx.listener;

    const penetration_slop = ctx.penetrationSlop();

    const box1 = try c.createBox(pos1, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(box_extent), .{});
    box1.setLinearVelocity(velocity);

    // In 1 step it should move -0.1 meter on the X axis
    const box2_velocity = Vec3.init(-0.1 * frequency, 0, 0);

    const box2 = try c.createBox(pos2, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(box_extent), .{});
    box2.setLinearVelocity(box2_velocity);

    try ctx.simulateSingleStep();

    // The bodies should have collided and body 2 should have moved according to its discrete step
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, box1.getID(), box2.getID()));
    try expect(listener.contains(.add, box1.getID(), box2.getID()));
    const new_pos2 = addR(pos2, box2_velocity.divScalar(frequency));
    const new_velocity = velocity.add(box2_velocity).mulScalar(0.5);
    try checkApproxEqual(box1.getPosition(), subR(new_pos2, Vec3.init(2.0 * box_extent, 0, 0)), .{ .tolerance = penetration_slop });
    try checkApproxEqual(box1.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box1.getAngularVelocity(), Vec3.zero(), .{});
    try checkApproxEqual(box2.getPosition(), new_pos2, .{});
    try checkApproxEqual(box2.getLinearVelocity(), new_velocity, .{});
    try checkApproxEqual(box2.getAngularVelocity(), Vec3.zero(), .{});
}
