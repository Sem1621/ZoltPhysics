//! Port of: UnitTests/Physics/EstimateCollisionResponseTest.cpp
//! Status: complete
//!
//! Zolt additions besides Jolt's TestEstimateCollisionResponse (which compares the estimate made in OnContactAdded
//! with the velocities after the simulation step):
//! - "TestEstimateCollisionResponseWithoutSimulation" runs the same loops over Jolt's values (motion types,
//!   restitution, friction, offsets, angular velocities) with the bodies of the C++ test, builds the manifold the way
//!   PhysicsSystem::ProcessBodyPair does without manifold reduction (NarrowPhaseQuery::CollideShape with the body 1
//!   shape, ManifoldBetweenTwoFaces, PruneContactPoints) and checks what a collision response must satisfy: a static
//!   body 2 keeps zero velocity, a kinematic body 2 keeps its velocity, the linear momentum of two dynamic bodies is
//!   conserved, the contact impulses are not negative and the bodies no longer approach each other along the normal
//!   (without restitution).
//! - The iteration count edge cases of the last test.
//! The bit exactness of the estimate is checked by ZoltParity/Physics/NarrowPhaseQueryParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const Body = zolt.Body;
const BodyFilter = zolt.BodyFilter;
const BodyID = zolt.BodyID;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionEstimationResult = zolt.CollisionEstimationResult;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const MotionType = zolt.MotionType;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

// Test CastShape ordering according to penetration depth
test "TestEstimateCollisionResponse" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    const box1_half_extents = Vec3.init(0.1, 1, 2);
    const box2_half_extents = Vec3.init(0.2, 3, 4);

    // Test different motion types, restitution, positions and angular velocities
    for ([_]MotionType{ .static, .kinematic, .dynamic }) |mt|
        for ([_]f32{ 0.0, 0.3, 1.0 }) |restitution|
            for ([_]f32{ 0.0, 0.3, 1.0 }) |friction|
                for ([_]f32{ 0.0, 0.5, box2_half_extents.getY() }) |y|
                    for ([_]f32{ 0.0, 0.5, box2_half_extents.getZ() }) |z|
                        for ([_]f32{ 0.0, -1.0, 1.0 }) |w| {
                            // Install a listener that predicts the collision response
                            const MyListener = struct {
                                pub fn onContactAdded(self: *@This(), body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
                                    self.result = zolt.estimateCollisionResponse(body1, body2, manifold, settings.combined_friction, settings.combined_restitution, .{});
                                    self.has_result = true;
                                }

                                result: CollisionEstimationResult = .{},
                                /// Zolt: OnContactAdded was called (Jolt would compare an uninitialized result)
                                has_result: bool = false,
                            };

                            var listener: MyListener = .{};
                            c.getSystem().setContactListener(.init(&listener));

                            const base_offset = RVec3.init(1, 2, 3);
                            const epsilon: Real = 0.0001;

                            const box1 = try c.createBox(base_offset, Quat.identity(), .dynamic, .discrete, Layers.moving, box1_half_extents, .{});
                            box1.setFriction(friction);
                            box1.setRestitution(restitution);
                            box1.setLinearVelocity(Vec3.init(1, 1, 0));
                            box1.setAngularVelocity(Vec3.init(0, w, 0));

                            const box2 = try c.createBox(base_offset.add(RVec3.init(box1_half_extents.getX() + box2_half_extents.getX() - epsilon, y, z)), Quat.identity(), mt, .discrete, if (mt == .static) Layers.non_moving else Layers.moving, box2_half_extents, .{});
                            box2.setFriction(friction);
                            box2.setRestitution(restitution);
                            if (mt != .static)
                                box2.setLinearVelocity(Vec3.init(-1, 0, 0));

                            // Step the simulation
                            _ = try c.simulateSingleStep();

                            // Check that the predicted velocities are correct
                            try fw.expect(listener.has_result);
                            try fw.checkApproxEqual(listener.result.linear_velocity1, box1.getLinearVelocity(), .{});
                            try fw.checkApproxEqual(listener.result.angular_velocity1, box1.getAngularVelocity(), .{});
                            try fw.checkApproxEqual(listener.result.linear_velocity2, box2.getLinearVelocity(), .{});
                            try fw.checkApproxEqual(listener.result.angular_velocity2, box2.getAngularVelocity(), .{});

                            // Remove the bodies in reverse order
                            const bi = c.getBodyInterface();
                            bi.removeBody(box2.getID());
                            bi.removeBody(box1.getID());
                            bi.destroyBody(box2.getID());
                            bi.destroyBody(box1.getID());
                        };
}

/// Only body 2 passes (the body pair of ProcessBodyPair)
const OnlyBodyFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: BodyFilter = .init(@This()),
    body_id: BodyID,

    pub fn shouldCollide(self: *const @This(), body_id: BodyID) bool {
        return body_id.eql(self.body_id);
    }
};

/// The manifold of PhysicsSystem::ProcessBodyPair without manifold reduction (one manifold per hit, the deepest hit)
fn calculateManifold(c: *PhysicsTestContext, body1: *const Body, body2: *const Body) !?ContactManifold {
    const physics_settings = c.getSystem().getPhysicsSettings();

    // Collide the shapes relative to the center of mass of body 1
    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    settings.active_edge_mode = if (physics_settings.check_active_edges) .collide_only_with_active else .collide_with_all;
    settings.max_separation_distance = physics_settings.speculative_contact_distance;
    settings.active_edge_movement_direction = body1.getLinearVelocity().sub(body2.getLinearVelocity());
    const base_offset = body1.getCenterOfMassPosition();
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const filter: OnlyBodyFilter = .{ .body_id = body2.getID() };
    c.getSystem().getNarrowPhaseQuery().collideShape(body1.getShape(), Vec3.one(), body1.getCenterOfMassTransform(), &settings, base_offset, &collector.base, .{ .body_filter = &filter.base });
    try collector.checkError();
    if (collector.hits.items.len == 0)
        return null;

    // The deepest hit
    var deepest = &collector.hits.items[0];
    for (collector.hits.items[1..]) |*hit| {
        if (hit.penetration_depth > deepest.penetration_depth)
            deepest = hit;
    }

    var manifold: ContactManifold = .{};
    manifold.base_offset = base_offset;
    zolt.manifoldBetweenTwoFaces(deepest.contact_point_on1, deepest.contact_point_on2, deepest.penetration_axis, physics_settings.speculative_contact_distance + physics_settings.manifold_tolerance, &deepest.shape1_face, &deepest.shape2_face, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2);
    manifold.sub_shape_id1 = deepest.sub_shape_id1;
    manifold.sub_shape_id2 = deepest.sub_shape_id2;
    manifold.penetration_depth = deepest.penetration_depth;
    manifold.world_space_normal = deepest.penetration_axis.normalized();
    if (manifold.relative_contact_points_on1.len > 4)
        zolt.pruneContactPoints(manifold.world_space_normal, &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2);
    return manifold;
}

/// The velocity of `point` (relative to the base offset of the manifold) on a body with its center of mass at `com`
fn pointVelocity(linear_velocity: Vec3, angular_velocity: Vec3, com: Vec3, point: Vec3) Vec3 {
    return linear_velocity.add(angular_velocity.cross(point.sub(com)));
}

test "TestEstimateCollisionResponseWithoutSimulation" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    const box1_half_extents = Vec3.init(0.1, 1, 2);
    const box2_half_extents = Vec3.init(0.2, 3, 4);

    var num_tested: u32 = 0;

    // Test different motion types, restitution, positions and angular velocities
    for ([_]MotionType{ .static, .kinematic, .dynamic }) |mt|
        for ([_]f32{ 0.0, 0.3, 1.0 }) |restitution|
            for ([_]f32{ 0.0, 0.3, 1.0 }) |friction|
                for ([_]f32{ 0.0, 0.5, box2_half_extents.getY() }) |y|
                    for ([_]f32{ 0.0, 0.5, box2_half_extents.getZ() }) |z|
                        for ([_]f32{ 0.0, -1.0, 1.0 }) |w| {
                            const base_offset = RVec3.init(1, 2, 3);
                            const epsilon: Real = 0.0001;

                            const box1 = try c.createBox(base_offset, Quat.identity(), .dynamic, .discrete, Layers.moving, box1_half_extents, .{});
                            box1.setFriction(friction);
                            box1.setRestitution(restitution);
                            box1.setLinearVelocity(Vec3.init(1, 1, 0));
                            box1.setAngularVelocity(Vec3.init(0, w, 0));

                            const box2 = try c.createBox(base_offset.add(RVec3.init(box1_half_extents.getX() + box2_half_extents.getX() - epsilon, y, z)), Quat.identity(), mt, .discrete, if (mt == .static) Layers.non_moving else Layers.moving, box2_half_extents, .{});
                            box2.setFriction(friction);
                            box2.setRestitution(restitution);
                            if (mt != .static)
                                box2.setLinearVelocity(Vec3.init(-1, 0, 0));

                            // The contact settings of the default combine functions (geometric mean of the friction, max of the restitution)
                            const combined_friction = @sqrt(box1.getFriction() * box2.getFriction());
                            const combined_restitution = math.max(box1.getRestitution(), box2.getRestitution());

                            // Predict the collision response
                            const manifold = (try calculateManifold(&c, box1, box2)) orelse return error.TestExpectedContact;
                            const result: CollisionEstimationResult = zolt.estimateCollisionResponse(box1, box2, &manifold, combined_friction, combined_restitution, .{});

                            // The impulses are not negative, one per contact point
                            try fw.expect(result.contact_impulse.len == manifold.relative_contact_points_on1.len);
                            for (result.contact_impulse.constSlice()) |impulse|
                                try fw.expect(impulse >= 0.0);

                            // A static body does not move, a kinematic body keeps its velocity
                            switch (mt) {
                                .static => {
                                    try fw.expect(result.linear_velocity2.eql(Vec3.zero()));
                                    try fw.expect(result.angular_velocity2.eql(Vec3.zero()));
                                },
                                .kinematic => {
                                    try fw.expect(result.linear_velocity2.eql(Vec3.init(-1, 0, 0)));
                                    try fw.expect(result.angular_velocity2.eql(Vec3.zero()));
                                },
                                .dynamic => {
                                    // Linear momentum is conserved
                                    const mass1 = 1.0 / box1.getMotionProperties().getInverseMass();
                                    const mass2 = 1.0 / box2.getMotionProperties().getInverseMass();
                                    const momentum_before = Vec3.init(1, 1, 0).mulScalar(mass1).add(Vec3.init(-1, 0, 0).mulScalar(mass2));
                                    const momentum_after = result.linear_velocity1.mulScalar(mass1).add(result.linear_velocity2.mulScalar(mass2));
                                    try fw.expect(momentum_after.sub(momentum_before).length() <= 1.0e-5 * momentum_before.length());
                                },
                            }

                            // Without restitution the bodies no longer approach each other at the contact points
                            const com1 = box1.getCenterOfMassPosition().sub(manifold.base_offset).toVec3();
                            const com2 = box2.getCenterOfMassPosition().sub(manifold.base_offset).toVec3();
                            for (manifold.relative_contact_points_on1.constSlice(), manifold.relative_contact_points_on2.constSlice()) |p1, p2| {
                                const p = p1.add(p2).mulScalar(0.5);
                                const relative_velocity = pointVelocity(result.linear_velocity2, result.angular_velocity2, com2, p).sub(pointVelocity(result.linear_velocity1, result.angular_velocity1, com1, p));
                                // (with restitution the contact points have different targets, which the iterations do not reach exactly)
                                if (combined_restitution == 0.0)
                                    try fw.expect(relative_velocity.dot(manifold.world_space_normal) > -1.0e-2);
                            }

                            // Remove the bodies in reverse order
                            const bi = c.getBodyInterface();
                            bi.removeBody(box2.getID());
                            bi.removeBody(box1.getID());
                            bi.destroyBody(box2.getID());
                            bi.destroyBody(box1.getID());
                            num_tested += 1;
                        };
    try fw.expectEqual(@as(u32, 3 * 3 * 3 * 3 * 3 * 3), num_tested);
}

test "estimateCollisionResponse: single contact point without friction uses one iteration, iteration count edge cases" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Two dynamic spheres colliding head on, equal mass: an elastic collision swaps the velocities
    const sphere1 = try c.createSphere(RVec3.init(0, 0, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere1.setLinearVelocity(Vec3.init(1, 0, 0));
    const sphere2 = try c.createSphere(RVec3.init(2, 0, 0), 1.0, .dynamic, .discrete, Layers.moving, .{});
    sphere2.setLinearVelocity(Vec3.init(-1, 0, 0));

    var manifold: ContactManifold = .{};
    manifold.base_offset = sphere1.getCenterOfMassPosition();
    manifold.world_space_normal = Vec3.init(1, 0, 0);
    manifold.penetration_depth = 0.0;
    manifold.relative_contact_points_on1.append(Vec3.init(1, 0, 0));
    manifold.relative_contact_points_on2.append(Vec3.init(1, 0, 0));

    // Elastic: the velocities swap (min velocity for restitution is 1, the approach velocity is 2)
    const elastic = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.0, 1.0, .{});
    try fw.checkApproxEqual(elastic.linear_velocity1, Vec3.init(-1, 0, 0), .{ .tolerance = 1.0e-5 });
    try fw.checkApproxEqual(elastic.linear_velocity2, Vec3.init(1, 0, 0), .{ .tolerance = 1.0e-5 });
    try fw.checkApproxEqual(elastic.friction_point, Vec3.init(1, 0, 0), .{});
    try fw.expect(elastic.friction_impulse.eql(Vec3.zero()));
    try fw.expectEqual(@as(f32, 0.0), elastic.angular_friction_impulse);
    const mass = 1.0 / sphere1.getMotionProperties().getInverseMass();
    try fw.expect(@abs(elastic.contact_impulse.get(0) - 2.0 * mass) <= 1.0e-5 * 2.0 * mass);

    // Inelastic: both stop
    const inelastic = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.0, 0.0, .{});
    try fw.checkApproxEqual(inelastic.linear_velocity1, Vec3.zero(), .{ .tolerance = 1.0e-5 });
    try fw.checkApproxEqual(inelastic.linear_velocity2, Vec3.zero(), .{ .tolerance = 1.0e-5 });

    // Below the min velocity for restitution the collision is inelastic
    const slow = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.0, 1.0, .{ .min_velocity_for_restitution = 3.0 });
    try fw.checkApproxEqual(slow.linear_velocity1, Vec3.zero(), .{ .tolerance = 1.0e-5 });

    // With friction (and more than one iteration) the iteration count is used: 0 iterations leave the velocities as they are
    const no_iterations = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.5, 0.0, .{ .num_iterations = 0 });
    try fw.expect(no_iterations.linear_velocity1.eql(Vec3.init(1, 0, 0)));
    try fw.expect(no_iterations.linear_velocity2.eql(Vec3.init(-1, 0, 0)));
    try fw.expectEqual(@as(f32, 0.0), no_iterations.contact_impulse.get(0));

    // A count above INT_MAX is a negative int in Jolt: no iterations either
    const negative_iterations = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.5, 0.0, .{ .num_iterations = 0x80000000 });
    try fw.expect(negative_iterations.linear_velocity1.eql(Vec3.init(1, 0, 0)));

    // A single contact point without friction always uses 1 iteration
    const one_iteration = zolt.estimateCollisionResponse(sphere1, sphere2, &manifold, 0.0, 0.0, .{ .num_iterations = 0 });
    try fw.checkApproxEqual(one_iteration.linear_velocity1, Vec3.zero(), .{ .tolerance = 1.0e-5 });
}
