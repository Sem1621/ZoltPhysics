//! Port of: UnitTests/Physics/PhysicsStepListenerTests.cpp
//! Status: complete
//!
//! The listeners are called from the job system (the CHECKs inside OnStep cannot `try`): a failed check is counted in
//! an atomic and the test checks that the count is zero afterwards.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Core = zolt.Core;
const BodyID = zolt.BodyID;
const BodyInterface = zolt.BodyInterface;
const PhysicsStepListener = zolt.PhysicsStepListener;
const PhysicsStepListenerContext = zolt.PhysicsStepListenerContext;

const expect = std.testing.expect;

/// Custom step listener that keeps track how often it has been called
const TestStepListener = struct {
    count: std.atomic.Value(i32) = .init(0),
    expected_steps: i32 = undefined,
    expected_delta_time: f32 = 0.0,
    /// Zolt: failed checks inside onStep
    num_failures: std.atomic.Value(u32) = .init(0),

    fn check(self: *TestStepListener, ok: bool) void {
        if (!ok) _ = self.num_failures.fetchAdd(1, .monotonic);
    }

    pub fn onStep(self: *TestStepListener, context: *const PhysicsStepListenerContext) void {
        self.check(context.delta_time == self.expected_delta_time);
        self.check(context.is_first_step == (@mod(self.count.load(.seq_cst), self.expected_steps) == 0));
        const new_count = self.count.fetchAdd(1, .seq_cst) + 1;
        self.check(context.is_last_step == (@mod(new_count, self.expected_steps) == 0));
    }
};

/// Perform the actual listener test with a variable amount of collision steps
fn doTest(collision_steps: u32) !void {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = collision_steps });
    defer c.deinit();

    // Initialize and add listeners
    var listeners: [10]TestStepListener = @splat(.{});
    for (&listeners) |*l| {
        l.expected_delta_time = 1.0 / 60.0 / @as(f32, @floatFromInt(collision_steps));
        l.expected_steps = @intCast(collision_steps);
    }
    for (&listeners) |*l|
        try c.getSystem().addStepListener(.init(l));

    // Stepping without delta time should not trigger step listeners
    _ = try c.simulateNoDeltaTime();
    for (&listeners) |*l|
        try expect(l.count.load(.seq_cst) == 0);

    // Stepping with delta time should call the step listeners as they can activate bodies
    _ = try c.simulateSingleStep();
    for (&listeners) |*l|
        try expect(l.count.load(.seq_cst) == collision_steps);

    // Adding an active body should have no effect, step listeners should still be called
    _ = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{});

    // Step the simulation
    _ = try c.simulateSingleStep();
    for (&listeners) |*l|
        try expect(l.count.load(.seq_cst) == 2 * collision_steps);

    // Unregister all listeners
    for (&listeners) |*l|
        c.getSystem().removeStepListener(.init(l));

    // Step the simulation
    _ = try c.simulateSingleStep();

    // Check that no further callbacks were triggered
    for (&listeners) |*l| {
        try expect(l.count.load(.seq_cst) == 2 * collision_steps);
        try expect(l.num_failures.load(.seq_cst) == 0);
    }
}

// Test the step listeners with a single collision step
test "TestStepListener1" {
    try doTest(1);
}

// Test the step listeners with two collision steps
test "TestStepListener2" {
    try doTest(2);
}

// Test the step listeners with four collision steps
test "TestStepListener4" {
    try doTest(4);
}

// Activate a body in a step listener
test "TestActivateInStepListener" {
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 2 });
    defer c.deinit();
    c.zeroGravity();

    // Create a box
    const body = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.one(), .{ .activation = .dont_activate });
    body.getMotionPropertiesMut().setLinearDamping(0.0);
    const body_id = body.getID();

    const velocity = Vec3.init(10.0, 0, 0);

    const MyStepListener = struct {
        body_interface: *BodyInterface,
        body_id: BodyID,
        num_failures: std.atomic.Value(u32) = .init(0),

        fn check(self: *@This(), ok: bool) void {
            if (!ok) _ = self.num_failures.fetchAdd(1, .monotonic);
        }

        pub fn onStep(self: *@This(), context: *const PhysicsStepListenerContext) void {
            if (context.is_first_step) {
                // We activate the body and set a velocity in the first step
                self.check(!self.body_interface.isActive(self.body_id));
                self.body_interface.setLinearVelocity(self.body_id, Vec3.init(10.0, 0, 0));
                self.check(self.body_interface.isActive(self.body_id));
            } else {
                // In the second step, the body should already have been activated
                self.check(self.body_interface.isActive(self.body_id));
            }
        }
    };

    var listener: MyStepListener = .{ .body_interface = c.getSystem().getBodyInterfaceNoLockMut(), .body_id = body_id };
    try c.getSystem().addStepListener(.init(&listener));

    _ = try c.simulateSingleStep();

    const bi = c.getBodyInterface();
    try expect(listener.num_failures.load(.seq_cst) == 0);
    try expect(bi.isActive(body_id));
    try expect(bi.getLinearVelocity(body_id).eql(velocity));
    const expected_position = velocity.mulScalar(c.getDeltaTime());
    try expect(bi.getPosition(body_id).eql(if (Core.double_precision) RVec3.fromVec3(expected_position) else expected_position));
}
