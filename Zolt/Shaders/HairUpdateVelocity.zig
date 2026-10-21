//! Port of: Jolt/Shaders/HairUpdateVelocity.hlsl, Jolt/Shaders/HairUpdateVelocityBindings.h, Jolt/Shaders/HairUpdateVelocity.h
//! Status: complete
//!
//! The hair shader that applies the global pose and the collisions to every simulated vertex and derives its velocity
//! from the distance it moved (Docs/Zolt/RemainingFeaturesArchitecture.md R12).
//! - `applyCollisionAndUpdateVelocity` and `limitVelocity` port the helper header HairUpdateVelocity.h.
//!   HairUpdateVelocityIntegrate.hlsl includes that header too, so they are public. In Jolt they are members of the
//!   wrapper class and read the buffers and constants directly; Zolt passes the bindings `b` of the shader (either
//!   shader's `Bindings`).
//! - HairUpdateVelocity.h includes HairApplyGlobalPose.h: Zolt calls `HairApplyGlobalPose.applyGlobalPose`.
//! - JPH_IN_OUT parameters are pointers (`pos`, `vel`), the JPH_OUT parameter `outVel` is the return value.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderPlane = @import("ShaderPlane.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const HairApplyGlobalPose = @import("HairApplyGlobalPose.zig");
const float3 = hlsl.float3;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const HairCollisionPlane = HairStructs.HairCollisionPlane;
const HairCollisionShape = HairStructs.HairCollisionShape;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairUpdateVelocity";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairUpdateVelocity): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gInitialBishops: hlsl.Buffer(JPH_uint) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gPreviousPositions: hlsl.Buffer(HairPosition) = .{},
    gGlobalPoseTransforms: hlsl.Buffer(HairGlobalPoseTransform) = .{},
    gCollisionShapes: hlsl.Buffer(HairCollisionShape) = .{},
    gCollisionPlanes: hlsl.Buffer(HairCollisionPlane) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gVelocities: hlsl.RWBuffer(HairVelocity) = .{},
};

/// ApplyCollisionAndUpdateVelocity (HairUpdateVelocity.h)
pub fn applyCollisionAndUpdateVelocity(b: anytype, vtx: u32, pos: *HairPosition, previous_pos: HairPosition, material: HairMaterial, strand_fraction: f32) HairVelocity {
    const c = &b.gContext.value;
    const ic = &b.gIterationContext.value;

    // Update velocities
    var out_vel: HairVelocity = .{
        .velocity = pos.position.sub(previous_pos.position).divScalar(c.delta_time),
        .angular_velocity = ShaderQuat.quatMulQuat(pos.rotation, ShaderQuat.quatConjugate(previous_pos.rotation)).xyz().mulScalar(c.two_div_delta_time),
    };

    if (material.enable_collision != 0) {
        // Calculate closest point on the collision plane
        const plane = b.gCollisionPlanes.get(vtx);
        const distance_to_plane = ShaderPlane.planeSignedDistance(plane.plane, pos.position);
        const contact_normal = ShaderPlane.planeGetNormal(plane.plane);
        const point_on_plane = pos.position.sub(contact_normal.mulScalar(distance_to_plane));

        // Calculate how much the plane moved in this time step
        const shape = b.gCollisionShapes.get(plane.shape_index);
        const plane_velocity = shape.linear_velocity.add(hlsl.cross(shape.angular_velocity, point_on_plane.sub(shape.center_of_mass)));
        const plane_movement = hlsl.dot(plane_velocity, contact_normal) * ic.accumulated_delta_time;

        const projected_distance = -distance_to_plane + plane_movement + HairCommon.gradientSamplerSample(material.hair_radius, strand_fraction);
        if (projected_distance > 0.0) {
            // Resolve penetration
            pos.position = pos.position.add(contact_normal.mulScalar(projected_distance));

            // Only update velocity when moving towards each other
            const v_relative = out_vel.velocity.sub(plane_velocity);
            const v_relative_dot_normal = hlsl.dot(contact_normal, v_relative);
            if (v_relative_dot_normal < 0.0) {
                // Calculate normal and tangential velocity (equation 30)
                const v_normal = contact_normal.mulScalar(v_relative_dot_normal);
                const v_tangential = v_relative.sub(v_normal);
                const v_tangential_length = hlsl.length(v_tangential);

                // Apply friction as described in Detailed Rigid Body Simulation with Extended Position Based Dynamics - Matthias Muller et al. (modified equation 31)
                if (v_tangential_length > 0.0)
                    out_vel.velocity = out_vel.velocity.sub(v_tangential.mulScalar(hlsl.min(material.friction * projected_distance / (v_tangential_length * c.delta_time), 1.0)));

                // Apply restitution of zero (equation 35)
                out_vel.velocity = out_vel.velocity.sub(v_normal);
            }
        }
    }

    return out_vel;
}

/// LimitVelocity (HairUpdateVelocity.h)
pub fn limitVelocity(vel: *HairVelocity, material: HairMaterial) void {
    // Limit linear velocity
    const linear_velocity_sq = hlsl.dot(vel.velocity, vel.velocity);
    if (linear_velocity_sq > material.max_linear_velocity_sq)
        vel.velocity = vel.velocity.mulScalar(hlsl.sqrt(material.max_linear_velocity_sq / linear_velocity_sq));

    // Limit angular velocity
    const angular_velocity_sq = hlsl.dot(vel.angular_velocity, vel.angular_velocity);
    if (angular_velocity_sq > material.max_angular_velocity_sq)
        vel.angular_velocity = vel.angular_velocity.mulScalar(hlsl.sqrt(material.max_angular_velocity_sq / angular_velocity_sq));
}

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid vertex
    const vtx = tid.x +% c.num_strands; // Skip the root of each strand, it's fixed
    if (vtx >= c.num_vertices)
        return;
    if (HairCommon.isVertexFixed(b.gVerticesFixed, vtx))
        return;

    // Load the material
    const strand_idx = vtx % c.num_strands;
    const material = b.gMaterials.get(HairCommon.getStrandMaterialIndex(b.gStrandMaterialIndex, strand_idx));

    // Load the vertex
    const strand_fraction = HairCommon.getVertexStrandFraction(b.gStrandFractions, vtx);
    const initial_pos = b.gInitialPositions.get(vtx);
    const initial_bishop = ShaderQuat.quatDecompress(b.gInitialBishops.get(vtx));
    const global_pose_transform = b.gGlobalPoseTransforms.get(strand_idx);
    var pos = b.gPositions.get(vtx);
    const prev_pos = b.gPreviousPositions.get(vtx);

    HairApplyGlobalPose.applyGlobalPose(&pos, initial_pos, initial_bishop, global_pose_transform, material, strand_fraction);
    var vel = applyCollisionAndUpdateVelocity(b, vtx, &pos, prev_pos, material, strand_fraction);
    limitVelocity(&vel, material);

    // Write back vertex
    b.gPositions.set(vtx, pos);
    b.gVelocities.set(vtx, vel);
}
