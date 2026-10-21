//! Port of: Jolt/Shaders/HairIntegrate.hlsl, Jolt/Shaders/HairIntegrateBindings.h, Jolt/Shaders/HairIntegrate.h
//! Status: complete
//!
//! The hair shader that drives the velocity of every simulated vertex towards the grid, then integrates its velocity
//! and its position (Docs/Zolt/RemainingFeaturesArchitecture.md R12).
//! - `deltaDensity`, `applyGrid` and `integrate` port the helper header HairIntegrate.h.
//!   HairUpdateVelocityIntegrate.hlsl includes that header too, so they are public. In Jolt they are members of the
//!   wrapper class and read the buffers and constants directly; Zolt passes the bindings `b` of the shader (either
//!   shader's `Bindings`).
//! - JPH_IN_OUT parameters are pointers: `pos` of `integrate` and `vel` of `applyGrid`.
//! - `uint3(grid_pos)` in `applyGrid` is undefined in Jolt for a NaN grid position. Jolt's result depends on the call
//!   site (UB11 of R20, see HLSLToCPP.zig and HairCommon.zig). Zolt uses the scalar conversion and the parity keeps NaN
//!   positions away from this site.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float3 = hlsl.float3;
const JPH_float = ShaderCore.JPH_float;
const JPH_float4 = ShaderCore.JPH_float4;
const JPH_uint = ShaderCore.JPH_uint;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairIntegrate";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairIntegrate): the field names are the binding
/// names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gNeutralDensity: hlsl.Buffer(JPH_float) = .{},
    gVelocityAndDensity: hlsl.Buffer(JPH_float4) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gVelocities: hlsl.Buffer(HairVelocity) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gPreviousPositions: hlsl.RWBuffer(HairPosition) = .{},
};

/// DeltaDensity (HairIntegrate.h)
pub fn deltaDensity(b: anytype, index: u32) f32 {
    return b.gVelocityAndDensity.get(index).w - b.gNeutralDensity.get(index);
}

/// ApplyGrid (HairIntegrate.h)
pub fn applyGrid(b: anytype, pos: HairPosition, vel: *HairVelocity, material: HairMaterial, strand_fraction: f32) void {
    const c = &b.gContext.value;

    if (material.enable_grid == 0)
        return;

    // Convert position to grid index and fraction
    const index_and_fraction = HairCommon.gridPositionToIndexAndFraction(c, pos.position);
    const index = index_and_fraction.index;
    const ma = index_and_fraction.fraction;
    const a = float3.init(1, 1, 1).sub(ma);

    // Get average velocity at the vertex position (trilinear sample). HLSL's `float3 * float` with a scalar weight.
    const stride = c.grid_stride;
    const adr_000 = HairCommon.gridIndexToBufferIndex(c, index);
    const adr_100 = adr_000 +% 1;
    const adr_010 = adr_000 +% stride.y;
    const adr_110 = adr_010 +% 1;
    var velocity = b.gVelocityAndDensity.get(adr_000).xyz().mulScalar(a.x * a.y * a.z);
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_100).xyz().mulScalar(ma.x * a.y * a.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_010).xyz().mulScalar(a.x * ma.y * a.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_110).xyz().mulScalar(ma.x * ma.y * a.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_000 +% stride.z).xyz().mulScalar(a.x * a.y * ma.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_100 +% stride.z).xyz().mulScalar(ma.x * a.y * ma.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_010 +% stride.z).xyz().mulScalar(a.x * ma.y * ma.z));
    velocity = velocity.add(b.gVelocityAndDensity.get(adr_110 +% stride.z).xyz().mulScalar(ma.x * ma.y * ma.z));

    // Drive towards the average velocity of the cell
    vel.velocity = vel.velocity.add(velocity.sub(vel.velocity).mulScalar(HairCommon.gradientSamplerSample(material.grid_velocity_factor, strand_fraction)));

    // Calculate force to go towards neutral density
    // Based on eq 3 of Volumetric Methods for Simulation and Rendering of Hair - Lena Petrovic, Mark Henne and John Anderson
    const dd000 = deltaDensity(b, adr_000);
    const dd100 = deltaDensity(b, adr_100);
    const dd010 = deltaDensity(b, adr_010);
    const dd110 = deltaDensity(b, adr_110);
    const dd001 = deltaDensity(b, adr_000 +% stride.z);
    const dd101 = deltaDensity(b, adr_100 +% stride.z);
    const dd011 = deltaDensity(b, adr_010 +% stride.z);
    const dd111 = deltaDensity(b, adr_110 +% stride.z);

    const force = float3.init(
        a.y * a.z * (dd000 - dd100) + ma.y * a.z * (dd010 - dd110) + a.y * ma.z * (dd001 - dd101) + ma.y * ma.z * (dd011 - dd111),
        a.x * a.z * (dd000 - dd010) + ma.x * a.z * (dd100 - dd110) + a.x * ma.z * (dd001 - dd011) + ma.x * ma.z * (dd101 - dd111),
        a.x * a.y * (dd000 - dd001) + ma.x * a.y * (dd100 - dd101) + a.x * ma.y * (dd010 - dd011) + ma.x * ma.y * (dd110 - dd111),
    );

    vel.velocity = vel.velocity.add(force.mulScalar(material.grid_density_force_factor).mulScalar(c.delta_time)); // / mass, but mass is 1
}

/// Integrate (HairIntegrate.h)
pub fn integrate(b: anytype, pos: *HairPosition, in_vel: HairVelocity, material: HairMaterial, strand_fraction: f32) void {
    const c = &b.gContext.value;

    var vel = in_vel;

    // Gravity
    vel.velocity = vel.velocity.add(c.sub_step_gravity.mulScalar(HairCommon.gradientSamplerSample(material.gravity_factor, strand_fraction)));

    // Damping
    vel.velocity = vel.velocity.mulScalar(material.exp_linear_damping_delta_time);
    vel.angular_velocity = vel.angular_velocity.mulScalar(material.exp_angular_damping_delta_time);

    // Integrate position
    pos.position = pos.position.add(vel.velocity.mulScalar(c.delta_time));

    // Integrate rotation
    const rotation = pos.rotation;
    const delta_rotation = ShaderQuat.quatImaginaryMulQuat(vel.angular_velocity, rotation).mulScalar(c.half_delta_time);
    pos.rotation = hlsl.normalize(rotation.add(delta_rotation));
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
    var pos = b.gPositions.get(vtx);
    var vel = b.gVelocities.get(vtx);

    // Update previous position
    b.gPreviousPositions.set(vtx, pos);

    applyGrid(b, pos, &vel, material, strand_fraction);
    integrate(b, &pos, vel, material, strand_fraction);
    b.gPositions.set(vtx, pos);
}
