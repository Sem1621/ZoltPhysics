//! Port of: UnitTests/Physics/PhysicsDeterminismTests.cpp
//! Status: complete
//!
//! Each case steps the same scene in a context without worker threads and in one with 15 worker threads, and checks
//! after every step that the bodies are bit identical. `uniform_real_distribution<float>` is
//! `UniformFloatDistribution` (libstdc++'s algorithm on UnitTestRandom).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const AABox = zolt.AABox;
const BodyID = zolt.BodyID;
const BodyIDVector = zolt.BodyIDVector;
const BodyLockRead = zolt.BodyLockRead;
const Body = zolt.Body;
const Ref = zolt.Ref;
const CollisionGroup = zolt.CollisionGroup;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const SwingTwistConstraint = zolt.SwingTwistConstraint;
const SwingTwistConstraintSettings = zolt.SwingTwistConstraintSettings;
const degreesToRadians = zolt.math.degreesToRadians;
const UnitTestRandom = fw.UnitTestRandom;
const UniformFloatDistribution = fw.UniformFloatDistribution;

const expect = std.testing.expect;

const BodyProperties = struct {
    position_com: RVec3,
    rotation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    bounds: AABox,
    is_active: bool,
};

/// Extract all relevant properties of a body for the test
fn getBodyProperties(context: *PhysicsTestContext, body_id: BodyID) !BodyProperties {
    var lock: BodyLockRead = .init(context.getSystem().getBodyLockInterface(), body_id);
    defer lock.deinit();
    if (lock.succeededAndIsInBroadPhase()) {
        const body = lock.getBody();
        return .{
            .is_active = body.isActive(),
            .position_com = body.getCenterOfMassPosition(),
            .rotation = body.getRotation(),
            .linear_velocity = body.getLinearVelocity(),
            .angular_velocity = body.getAngularVelocity(),
            .bounds = body.getWorldSpaceBounds().*,
        };
    } else {
        return error.TestUnexpectedResult; // CHECK(false)
    }
}

/// Step two physics simulations for inTotalTime and check after each step that the simulations are identical
fn compareSimulations(context1: *PhysicsTestContext, context2: *PhysicsTestContext, total_time: f32) !void {
    const allocator = std.testing.allocator;
    try expect(context1.getDeltaTime() == context2.getDeltaTime());

    // Step until we've stepped for inTotalTime
    var t: f32 = 0;
    while (t <= total_time) : (t += context1.getDeltaTime()) {
        // Step the simulation
        _ = try context1.simulateSingleStep();
        _ = try context2.simulateSingleStep();

        // Get all bodies
        var bodies1: BodyIDVector = .empty;
        defer bodies1.deinit(allocator);
        var bodies2: BodyIDVector = .empty;
        defer bodies2.deinit(allocator);
        try context1.getSystem().getBodies(allocator, &bodies1);
        try context2.getSystem().getBodies(allocator, &bodies2);
        try expect(bodies1.items.len == bodies2.items.len);

        // Loop over all bodies
        for (0..@min(bodies1.items.len, bodies2.items.len)) |b| {
            // Check that the body ID's match
            const b1_id = bodies1.items[b];
            const b2_id = bodies2.items[b];
            try expect(b1_id.eql(b2_id));

            // Get the properties of the body
            const properties1 = try getBodyProperties(context1, b1_id);
            const properties2 = try getBodyProperties(context2, b2_id);
            try expect(properties1.is_active == properties2.is_active);
            try expect(properties1.position_com.eql(properties2.position_com));
            try expect(properties1.rotation.eql(properties2.rotation));
            try expect(properties1.linear_velocity.eql(properties2.linear_velocity));
            try expect(properties1.angular_velocity.eql(properties2.angular_velocity));
            try expect(properties1.bounds.min.eql(properties2.bounds.min));
            try expect(properties1.bounds.max.eql(properties2.bounds.max));
        }
    }
}

fn createGridOfBoxesDiscrete(context: *PhysicsTestContext) !void {
    var random = UnitTestRandom.init(UnitTestRandom.default_seed);
    const restitution = UniformFloatDistribution.init(0.0, 1.0);

    _ = try context.createFloor();

    for (0..5) |x| {
        for (0..5) |z| {
            const body = try context.createBox(RVec3.init(@floatFromInt(x), 5.0, @floatFromInt(z)), Quat.random(&random), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.1), .{});
            body.setRestitution(restitution.next(&random));
            body.setLinearVelocity(Vec3.random(&random));
        }
    }
}

test "TestGridOfBoxesDiscrete" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0 });
    defer c1.deinit();
    try createGridOfBoxesDiscrete(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 15 });
    defer c2.deinit();
    try createGridOfBoxesDiscrete(&c2);

    try compareSimulations(&c1, &c2, 5.0);
}

fn createGridOfBoxesLinearCast(context: *PhysicsTestContext) !void {
    var random = UnitTestRandom.init(UnitTestRandom.default_seed);
    const restitution = UniformFloatDistribution.init(0.0, 1.0);

    _ = try context.createFloor();

    for (0..5) |x| {
        for (0..5) |z| {
            const body = try context.createBox(RVec3.init(@floatFromInt(x), 5.0, @floatFromInt(z)), Quat.random(&random), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(0.1), .{});
            body.setRestitution(restitution.next(&random));
            body.setLinearVelocity(Vec3.random(&random).sub(Vec3.init(0, -5.0, 0)));
        }
    }
}

test "TestGridOfBoxesLinearCast" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0 });
    defer c1.deinit();
    try createGridOfBoxesLinearCast(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 15 });
    defer c2.deinit();
    try createGridOfBoxesLinearCast(&c2);

    try compareSimulations(&c1, &c2, 5.0);
}

fn createGridOfBoxesConstrained(context: *PhysicsTestContext) !void {
    const allocator = std.testing.allocator;
    var random = UnitTestRandom.init(UnitTestRandom.default_seed);
    const restitution = UniformFloatDistribution.init(0.0, 1.0);

    _ = try context.createFloor();

    const num_per_axis = 5;

    // Build a collision group filter that disables collision between adjacent bodies
    const group_filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = num_per_axis });
    var group_filter_ref = Ref(GroupFilter).init(group_filter.asGroupFilterMut());
    defer group_filter_ref.deinit();
    var i: CollisionGroup.SubGroupID = 0;
    while (i < num_per_axis - 1) : (i += 1)
        group_filter.disableCollision(i, i + 1);

    // Create a number of chains
    for (0..num_per_axis) |x| {
        // Create a chain of bodies connected with swing twist constraints
        var prev_body: ?*Body = null;
        for (0..num_per_axis) |z| {
            const body_pos = RVec3.init(@floatFromInt(x), 5.0, 0.2 * @as(f32, @floatFromInt(z)));
            const body = try context.createBox(body_pos, Quat.random(&random), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.1), .{});
            body.setRestitution(restitution.next(&random));
            body.setLinearVelocity(Vec3.random(&random));
            var group = CollisionGroup.init(group_filter.asGroupFilter(), @intCast(x), @intCast(z));
            defer group.deinit();
            body.setCollisionGroup(&group);

            // Constrain the body to the previous body
            if (prev_body) |prev| {
                var st = SwingTwistConstraintSettings.init(allocator);
                defer st.asSettings().deinit();
                st.position1 = body_pos.subVec3(Vec3.init(0, 0, 0.1));
                st.position2 = st.position1;
                st.twist_axis1 = Vec3.axisZ();
                st.twist_axis2 = st.twist_axis1;
                st.plane_axis1 = Vec3.axisX();
                st.plane_axis2 = st.plane_axis1;
                st.normal_half_cone_angle = degreesToRadians(45.0);
                st.plane_half_cone_angle = degreesToRadians(30.0);
                st.twist_min_angle = degreesToRadians(-15.0);
                st.twist_max_angle = degreesToRadians(15.0);
                _ = try context.createConstraint(SwingTwistConstraint, prev, body, st.asTwoBodySettingsConst());
            }

            prev_body = body;
        }
    }
}

test "TestGridOfBoxesConstrained" {
    const allocator = std.testing.allocator;
    var c1: PhysicsTestContext = undefined;
    try c1.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 0 });
    defer c1.deinit();
    try createGridOfBoxesConstrained(&c1);

    var c2: PhysicsTestContext = undefined;
    try c2.init(allocator, .{ .delta_time = 1.0 / 60.0, .collision_steps = 1, .worker_threads = 15 });
    defer c2.deinit();
    try createGridOfBoxesConstrained(&c2);

    try compareSimulations(&c1, &c2, 5.0);
}
