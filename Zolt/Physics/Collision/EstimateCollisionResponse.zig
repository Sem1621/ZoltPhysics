//! Port of: Jolt/Physics/Collision/EstimateCollisionResponse.h, Jolt/Physics/Collision/EstimateCollisionResponse.cpp
//! Status: complete
//!
//! - The out parameter `CollisionEstimationResult &outResult` is the return value (porting guide section 4): Jolt
//!   writes every member of it (the contact impulses are resized to the number of contact points).
//! - The defaulted arguments `inMinVelocityForRestitution = 1.0f` and `inNumIterations = 10` are an options struct.
//! - `int num_iterations = ... : inNumIterations` converts the `uint` to `int` like C++ (`@bitCast`): a count above
//!   INT_MAX runs no iteration, as in Jolt.
//! - The function only reads the bodies and the manifold, so it can be called from ContactListener.onContactAdded on
//!   any worker thread (the bodies are locked by the simulation at that point).

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const StaticArray = @import("../../Core/StaticArray.zig").StaticArray;
const Body = @import("../Body/Body.zig").Body;
const ContactListenerFile = @import("ContactListener.zig");
const ContactManifold = ContactListenerFile.ContactManifold;
const ContactPoints = ContactListenerFile.ContactPoints;
const ContactConstraintPart = @import("../Constraints/ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart;
const AngularFrictionConstraintPart = @import("../Constraints/ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart;

/// A structure that contains the estimated contact and friction impulses and the resulting body velocities
pub const CollisionEstimationResult = struct {
    /// The estimated linear velocity of body 1 after collision
    linear_velocity1: Vec3 = undefined,
    /// The estimated angular velocity of body 1 after collision
    angular_velocity1: Vec3 = undefined,
    /// The estimated linear velocity of body 2 after collision
    linear_velocity2: Vec3 = undefined,
    /// The estimated angular velocity of body 2 after collision
    angular_velocity2: Vec3 = undefined,

    /// Point at which friction was applied (relative to mBaseOffset of the manifold)
    friction_point: Vec3 = undefined,
    /// Estimated linear friction impulse (kg m / s)
    friction_impulse: Vec3 = undefined,
    /// Estimated angular friction impulse around the world space normal (kg m^2 / s)
    angular_friction_impulse: f32 = undefined,

    /// Estimated contact impulses (kg m / s)
    contact_impulse: StaticArray(f32, ContactPoints.capacity) = .empty,
};

/// This function estimates the contact impulses and body velocity changes as a result of a collision.
/// It can be used in the ContactListener::OnContactAdded to determine the strength of the collision to e.g. play a sound or trigger a particle system.
/// This function is accurate when two bodies collide but will not be accurate when more than 2 bodies collide at the same time as it does not know about these other collisions.
///
/// @param body1 Colliding body 1
/// @param body2 Colliding body 2
/// @param manifold The collision manifold
/// @param combined_friction The combined friction of body 1 and body 2 (see ContactSettings::mCombinedFriction)
/// @param combined_restitution The combined restitution of body 1 and body 2 (see ContactSettings::mCombinedRestitution)
/// @param opts.min_velocity_for_restitution Minimal velocity required for restitution to be applied (see PhysicsSettings::mMinVelocityForRestitution)
/// @param opts.num_iterations Number of iterations to use for the impulse estimation (see PhysicsSettings::mNumVelocitySteps, note you can probably use a lower number for a decent estimate). If you set the number of iterations to 1 then no friction will be calculated.
/// @return A structure that contains the estimated contact and friction impulses and the resulting body velocities
pub fn estimateCollisionResponse(body1: *const Body, body2: *const Body, manifold: *const ContactManifold, combined_friction: f32, combined_restitution: f32, opts: struct { min_velocity_for_restitution: f32 = 1.0, num_iterations: u32 = 10 }) CollisionEstimationResult {
    var result: CollisionEstimationResult = .{};

    const num_points: u32 = manifold.relative_contact_points_on1.len;
    if (Core.enable_asserts) std.debug.assert(num_points == manifold.relative_contact_points_on2.len); // Jolt's release build uses the points of shape 1

    // Calculate friction directions
    const tangent1 = manifold.world_space_normal.getNormalizedPerpendicular();
    const tangent2 = manifold.world_space_normal.cross(tangent1);

    // Get body velocities
    const motion_type1 = body1.getMotionType();
    const motion_properties1 = body1.getMotionPropertiesUnchecked();
    if (motion_type1 != .static) {
        result.linear_velocity1 = motion_properties1.?.getLinearVelocity();
        result.angular_velocity1 = motion_properties1.?.getAngularVelocity();
    } else {
        result.angular_velocity1 = Vec3.zero();
        result.linear_velocity1 = result.angular_velocity1;
    }

    const motion_type2 = body2.getMotionType();
    const motion_properties2 = body2.getMotionPropertiesUnchecked();
    if (motion_type2 != .static) {
        result.linear_velocity2 = motion_properties2.?.getLinearVelocity();
        result.angular_velocity2 = motion_properties2.?.getAngularVelocity();
    } else {
        result.angular_velocity2 = Vec3.zero();
        result.linear_velocity2 = result.angular_velocity2;
    }

    // Get inverse mass and inertia
    var inv_m1: f32 = undefined;
    var inv_m2: f32 = undefined;
    var inv_i1: Mat44 = undefined;
    var inv_i2: Mat44 = undefined;
    if (motion_type1 == .dynamic) {
        inv_m1 = motion_properties1.?.getInverseMass();
        inv_i1 = body1.getInverseInertia();
    } else {
        inv_m1 = 0.0;
        inv_i1 = Mat44.zero();
    }

    if (motion_type2 == .dynamic) {
        inv_m2 = motion_properties2.?.getInverseMass();
        inv_i2 = body2.getInverseInertia();
    } else {
        inv_m2 = 0.0;
        inv_i2 = Mat44.zero();
    }

    // Get center of masses relative to the base offset
    const com1 = body1.getCenterOfMassPosition().sub(manifold.base_offset).toVec3();
    const com2 = body2.getCenterOfMassPosition().sub(manifold.base_offset).toVec3();

    // Initialize the constraint properties
    const Part = ContactConstraintPart(.dynamic, .dynamic);
    var contact_constraints: [ContactPoints.capacity]Part = undefined;
    var contact_points: [ContactPoints.capacity]Vec3 = undefined;
    var friction_point = Vec3.zero();
    for (0..num_points) |c| {
        // Calculate contact points relative to body 1 and 2
        const p = manifold.relative_contact_points_on1.get(@intCast(c)).add(manifold.relative_contact_points_on2.get(@intCast(c))).mulScalar(0.5);

        // Calculate friction point
        contact_points[c] = p;
        friction_point = friction_point.add(p);

        // Calculate contact point relative to com
        const r1 = p.sub(com1);
        const r2 = p.sub(com2);

        // Handle elastic collisions
        var bias: f32 = 0.0;
        if (combined_restitution > 0.0) {
            // Calculate velocity of contact point
            const relative_velocity = result.linear_velocity2.add(result.angular_velocity2.cross(r2)).sub(result.linear_velocity1).sub(result.angular_velocity1.cross(r1));
            const normal_velocity = relative_velocity.dot(manifold.world_space_normal);

            // If it is big enough, apply restitution
            if (normal_velocity < -opts.min_velocity_for_restitution)
                bias = combined_restitution * normal_velocity;
        }

        // Initialize contact constraint
        const constraint = &contact_constraints[c];
        constraint.setTotalLambda(0.0);
        constraint.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, manifold.world_space_normal, .{ .bias = bias });
    }

    // Calculate distance to friction center for each point
    const num_points_f: f32 = @floatFromInt(num_points);
    friction_point = friction_point.divScalar(num_points_f);
    var distance_to_friction_center: [ContactPoints.capacity]f32 = undefined;
    for (0..num_points) |c| {
        const delta = contact_points[c].sub(friction_point);
        distance_to_friction_center[c] = delta.sub(manifold.world_space_normal.mulScalar(delta.dot(manifold.world_space_normal))).length();
    }
    result.friction_point = friction_point;

    // Initialize friction constraints
    var friction1: Part = .{};
    var friction2: Part = .{};
    var angular_friction: AngularFrictionConstraintPart(.dynamic, .dynamic) = .{};
    angular_friction.setTotalLambda(0.0);
    friction1.setTotalLambda(0.0);
    friction2.setTotalLambda(0.0);
    if (combined_friction > 0.0) {
        const r1 = friction_point.sub(com1);
        const r2 = friction_point.sub(com2);

        friction1.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, tangent1, .{});
        friction2.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, tangent2, .{});

        if (num_points > 1)
            angular_friction.calculateConstraintProperties(inv_i1, inv_i2, manifold.world_space_normal, .{});
    }

    // If there's only 1 contact point, we only need 1 iteration
    const num_iterations: i32 = if (combined_friction <= 0.0 and num_points == 1) 1 else @bitCast(opts.num_iterations);

    // Solve iteratively
    var iteration: i32 = 0;
    while (iteration < num_iterations) : (iteration += 1) {
        // Solve friction constraints first
        if (combined_friction > 0.0) {
            // Calculate max impulse that can be applied
            var max_linear_lambda: f32 = 0.0;
            var max_angular_lambda: f32 = 0.0;
            for (0..num_points) |c| {
                const lambda = contact_constraints[c].getTotalLambda();
                max_linear_lambda += lambda;
                max_angular_lambda += distance_to_friction_center[c] * lambda;
            }
            max_linear_lambda *= combined_friction;
            max_angular_lambda *= combined_friction;

            // Calculate impulse to stop motion in tangential direction
            var lambda1 = friction1.solveVelocityConstraintGetTotalLambda(result.linear_velocity1, result.angular_velocity1, result.linear_velocity2, result.angular_velocity2, tangent1);
            var lambda2 = friction2.solveVelocityConstraintGetTotalLambda(result.linear_velocity1, result.angular_velocity1, result.linear_velocity2, result.angular_velocity2, tangent2);

            // If the total lambda that we will apply is too large, scale it back
            const total_lambda_sq = math.square(lambda1) + math.square(lambda2);
            if (total_lambda_sq > math.square(max_linear_lambda) + math.flt_min) { // ensure total_lambda_sq > FLT_MIN to avoid division by zero in MulRSqrtApproximate
                const scale = math.mulRSqrtApproximate(max_linear_lambda, total_lambda_sq);
                lambda1 *= scale;
                lambda2 *= scale;
            }

            // Apply the friction impulse
            _ = friction1.solveVelocityConstraintApplyLambda(&result.linear_velocity1, &result.angular_velocity1, &result.linear_velocity2, &result.angular_velocity2, inv_m1, inv_m2, tangent1, lambda1);
            _ = friction2.solveVelocityConstraintApplyLambda(&result.linear_velocity1, &result.angular_velocity1, &result.linear_velocity2, &result.angular_velocity2, inv_m1, inv_m2, tangent2, lambda2);

            // Apply angular friction
            if (num_points > 1)
                _ = angular_friction.solveVelocityConstraint(&result.angular_velocity1, &result.angular_velocity2, manifold.world_space_normal, -max_angular_lambda, max_angular_lambda);
        }

        // Solve contact constraints last
        for (0..num_points) |c|
            _ = contact_constraints[c].solveVelocityConstraint(&result.linear_velocity1, &result.angular_velocity1, &result.linear_velocity2, &result.angular_velocity2, inv_m1, inv_m2, manifold.world_space_normal, 0.0, math.flt_max);
    }

    // Store impulses
    result.contact_impulse.resize(num_points);
    for (0..num_points) |c|
        result.contact_impulse.at(@intCast(c)).* = contact_constraints[c].getTotalLambda();
    result.friction_impulse = tangent1.mulScalar(friction1.getTotalLambda()).add(tangent2.mulScalar(friction2.getTotalLambda()));
    result.angular_friction_impulse = angular_friction.getTotalLambda();
    return result;
}
