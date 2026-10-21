//! Port of: Jolt/Shaders/HairUpdateStrands.hlsl, Jolt/Shaders/HairUpdateStrandsBindings.h
//! Status: complete
//!
//! The hair shader that solves the constraints of a strand, one thread per strand
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12): the long range attachments, the stretch and shear constraints and
//! the bend and twist constraints of Cosserat rods.
//! - JPH_IN_OUT parameters are pointers (`v0`, `v1`).
//! - `inMaterial.mBendComplianceMultiplier[idx + 1]` reads past the float4 when the strand fraction of the middle
//!   vertex is 1 (its byte is 255, so idx is 3). This is undefined in Jolt: HLSLToCPP's `operator []` is
//!   `(&x)[inIndex]`, so Jolt's build reads the next float of JPH_HairMaterial, `mGridVelocityFactor.x`, and multiplies
//!   it by a fraction of 0. Zolt reads that member too (`bendComplianceMultiplier`), pinned by a test. A one-off
//!   dispatch on both sides gave the same bits for a finite and an infinite `mGridVelocityFactor.x`. Hair never gives a
//!   vertex in the middle of a strand the fraction 1, unless the last segment is too short to change the float sum of
//!   the segment lengths (for example zero). The parity does not generate it.
//! - Zolt does not panic there, because valid Hair data reaches this read. A step gradient (equal min and max
//!   fractions) gives an infinite or NaN `mGridVelocityFactor.x`. Then the multiplier is NaN, and Jolt and Zolt both
//!   skip the bend and twist constraint of that vertex.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMath = @import("ShaderMath.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float3 = hlsl.float3;
const JPH_float = ShaderCore.JPH_float;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const JPH_Quat = ShaderCore.JPH_Quat;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairUpdateStrands";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairUpdateStrands): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gOmega0s: hlsl.Buffer(JPH_uint) = .{},
    gInitialLengths: hlsl.Buffer(JPH_float) = .{},
    gStrandVertexCounts: hlsl.Buffer(JPH_uint) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
};

/// ApplyLRA
fn applyLRA(x0: float3, max_dist: f32, v0: *HairPosition) void {
    const delta = v0.position.sub(x0);
    const delta_len_sq = hlsl.dot(delta, delta);
    if (delta_len_sq > ShaderMath.square(max_dist))
        v0.position = x0.add(delta.mulScalar(max_dist).divScalar(hlsl.sqrt(delta_len_sq)));
}

/// ApplyStretchShear
fn applyStretchShear(v0: *HairPosition, v1: *HairPosition, length01: f32, inv_mass0: f32, inv_mass1: f32, material: *const HairMaterial) void {
    // Inertia of a thin rod of length L ~ m * L^2, we take the maximum mass of the two vertices
    const rod_inv_mass = hlsl.min(inv_mass0, inv_mass1) / material.inertia_multiplier; // / L^2, which we'll apply later

    // Equation 37 from "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
    const denom = inv_mass0 + inv_mass1 + 4.0 * rod_inv_mass + material.stretch_compliance_inv_delta_time_sq; // 4 * rod_inv_mass: / L^2 * L^2 cancels
    if (denom >= 1.0e-12) {
        const x0 = v0.position;
        const x1 = v1.position;
        var rotation1 = v0.rotation;
        const d3 = ShaderQuat.quatRotateAxisZ(rotation1);
        const delta = x1.sub(x0).sub(d3.mulScalar(length01)).divScalar(denom);
        v0.position = x0.add(delta.mulScalar(inv_mass0));
        v1.position = x1.sub(delta.mulScalar(inv_mass1));
        // q * e3_bar = q * (0, 0, -1, 0) = [-qy, qx, -qw, qz]
        const q_e3_bar = JPH_Quat.init(-rotation1.y, rotation1.x, -rotation1.w, rotation1.z);
        rotation1 = rotation1.add(ShaderQuat.quatImaginaryMulQuat(delta, q_e3_bar).mulScalar(2.0 * rod_inv_mass / length01)); // / L^2 * L => / L
        v0.rotation = hlsl.normalize(rotation1);
    }
}

/// `inMaterial.mBendComplianceMultiplier[index]` for an index up to 4. HLSLToCPP's `operator []` is `(&x)[inIndex]`,
/// so index 4 reads the float after the float4 in JPH_HairMaterial. Undefined in Jolt, see the file header.
fn bendComplianceMultiplier(material: *const HairMaterial, index: u32) f32 {
    if (index < 4)
        return material.bend_compliance_multiplier.getComponent(index);
    return material.grid_velocity_factor.x;
}

/// ApplyBendTwist
fn applyBendTwist(v0: *HairPosition, v1: *HairPosition, omega0: JPH_Quat, length01: f32, length12: f32, strand_fraction1: f32, inv_mass0: f32, inv_mass1: f32, inv_mass2: f32, material: *const HairMaterial) void {
    // Inertia of a thin rod of length L ~ m * L^2, we take the maximum mass of the two vertices
    const rod_inv_mass = hlsl.min(inv_mass0, inv_mass1) / (material.inertia_multiplier * ShaderMath.square(length01));
    const rod2_inv_mass = hlsl.min(inv_mass1, inv_mass2) / (material.inertia_multiplier * ShaderMath.square(length12));

    // Calculate multiplier for the bend compliance based on strand fraction
    var fraction = strand_fraction1 * 3.0;
    const idx = hlsl.toUint(fraction);
    fraction = fraction - @as(f32, @floatFromInt(idx));
    const multiplier = bendComplianceMultiplier(material, idx) * (1.0 - fraction) + bendComplianceMultiplier(material, idx + 1) * fraction;

    // Equation 40 from "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
    const denom = rod_inv_mass + rod2_inv_mass + material.bend_compliance_inv_delta_time_sq * multiplier;
    if (denom >= 1.0e-12) {
        var rotation1 = v0.rotation;
        var rotation2 = v1.rotation;
        const omega = ShaderQuat.quatMulQuat(ShaderQuat.quatConjugate(rotation1), rotation2);
        const omega_min_omega0 = omega.sub(omega0);
        const omega_plus_omega0 = omega.add(omega0);
        // Take the shortest of the two rotations
        var delta_omega = if (hlsl.dot(omega_plus_omega0, omega_plus_omega0) < hlsl.dot(omega_min_omega0, omega_min_omega0)) omega_plus_omega0 else omega_min_omega0;
        delta_omega = delta_omega.divScalar(denom);
        delta_omega.w = 0; // Scalar part needs to be zero because the real part of the Darboux vector doesn't vanish, see text between eq. 39 and 40.
        const delta_rod2 = ShaderQuat.quatMulQuat(rotation1, delta_omega).mulScalar(rod2_inv_mass);
        rotation1 = rotation1.add(ShaderQuat.quatMulQuat(rotation2, delta_omega).mulScalar(rod_inv_mass));
        rotation2 = rotation2.sub(delta_rod2);
        v0.rotation = hlsl.normalize(rotation1);
        v1.rotation = hlsl.normalize(rotation2);
    }
}

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerStrandBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid strand
    const strand_idx = tid.x;
    if (strand_idx >= c.num_strands)
        return;
    const strand_vtx_count = HairCommon.getStrandVertexCount(b.gStrandVertexCounts, strand_idx);

    // Load the material
    const material = b.gMaterials.get(HairCommon.getStrandMaterialIndex(b.gStrandMaterialIndex, strand_idx));

    // Load the first two vertices
    var vtx_idx_to_load = strand_idx;
    var inv_mass0 = HairCommon.getVertexInvMass(b.gVerticesFixed, vtx_idx_to_load);
    var length0 = b.gInitialLengths.get(vtx_idx_to_load);
    var v0 = b.gPositions.get(vtx_idx_to_load);

    // LRA: Vertex everything is attached to
    const x0 = b.gInitialPositions.get(vtx_idx_to_load);

    // LRA: Tracks the distance from the first vertex
    var max_dist = length0;

    vtx_idx_to_load +%= c.num_strands;
    var inv_mass1 = HairCommon.getVertexInvMass(b.gVerticesFixed, vtx_idx_to_load);
    var strand_fraction1 = HairCommon.getVertexStrandFraction(b.gStrandFractions, vtx_idx_to_load);
    var length1 = b.gInitialLengths.get(vtx_idx_to_load);
    var v1 = b.gPositions.get(vtx_idx_to_load);

    // Process 2nd vertex
    if (material.enable_lra != 0 and inv_mass1 > 0.0)
        applyLRA(x0, max_dist, &v1);
    max_dist += length1;

    var vtx_idx_to_retire = strand_idx;
    var vtx: u32 = 2;
    while (vtx < strand_vtx_count) : (vtx += 1) {
        // Get the initial rotation difference from the middle vertex
        const omega0 = ShaderQuat.quatDecompress(b.gOmega0s.get(vtx_idx_to_load));

        // Load the next vertex
        vtx_idx_to_load +%= c.num_strands;
        const inv_mass2 = HairCommon.getVertexInvMass(b.gVerticesFixed, vtx_idx_to_load);
        const strand_fraction2 = HairCommon.getVertexStrandFraction(b.gStrandFractions, vtx_idx_to_load);
        const length2 = b.gInitialLengths.get(vtx_idx_to_load);
        var v2 = b.gPositions.get(vtx_idx_to_load);

        // Process newly added vertex
        if (material.enable_lra != 0 and inv_mass2 > 0.0)
            applyLRA(x0, max_dist, &v2);
        max_dist += length2;

        // Stitched mode as per Strand-based Hair System - Pedersen - SIGGRAPH 2022
        applyStretchShear(&v1, &v2, length1, inv_mass1, inv_mass2, &material);
        applyStretchShear(&v0, &v1, length0, inv_mass0, inv_mass1, &material);
        applyBendTwist(&v0, &v1, omega0, length0, length1, strand_fraction1, inv_mass0, inv_mass1, inv_mass2, &material);

        // Retire vertex
        b.gPositions.set(vtx_idx_to_retire, v0);
        vtx_idx_to_retire +%= c.num_strands;

        // Shift the vertices
        inv_mass0 = inv_mass1;
        inv_mass1 = inv_mass2;
        strand_fraction1 = strand_fraction2;
        length0 = length1;
        length1 = length2;
        v0 = v1;
        v1 = v2;
    }

    // Retire 2nd to last vertex
    b.gPositions.set(vtx_idx_to_retire, v0);
    vtx_idx_to_retire +%= c.num_strands;

    // Cannot calculate rotation for last vertex, take the 2nd last
    v1.rotation = v0.rotation;

    // Retire last vertex
    b.gPositions.set(vtx_idx_to_retire, v1);
}

test "HairUpdateStrands: a middle vertex with strand fraction 1 reads the grid velocity factor (undefined in Jolt)" {
    const std = @import("std");

    // The fraction 1 gives idx 3 and fraction 0, so the multiplier is `[3] * 1 + [4] * 0`. Index 4 is
    // mGridVelocityFactor.x: when it is infinite or NaN, the multiplier is NaN and the bend twist constraint is
    // skipped. A step gradient (equal min and max fractions) gives such a factor.
    var material = std.mem.zeroes(HairMaterial);
    material.bend_compliance_multiplier = .init(1, 2, 3, 4);
    material.grid_velocity_factor = .init(5, 6, 7, 8);
    try std.testing.expectEqual(4, bendComplianceMultiplier(&material, 3));
    try std.testing.expectEqual(5, bendComplianceMultiplier(&material, 4));

    // The byte 255 is the fraction 1
    var fractions: [1]u32 align(16) = .{0xff00};
    var b: hlsl.Buffer(u32) = .{};
    b.bind(std.mem.sliceAsBytes(&fractions));
    try std.testing.expectEqual(1.0, HairCommon.getVertexStrandFraction(b, 1));

    // A strand of 3 vertices: vertex 1 has fraction 1. Run it with a finite, an infinite and a NaN factor.
    material.inertia_multiplier = 1;
    material.bend_compliance_inv_delta_time_sq = 1;
    const omega0 = ShaderQuat.quatDecompress(0);
    var results: [3][2]HairPosition = undefined;
    for (&results, [_]f32{ 0.5, std.math.inf(f32), std.math.nan(f32) }) |*result, factor| {
        material.grid_velocity_factor.x = factor;
        var v0: HairPosition = .{ .position = .init(0, 0, 0), .rotation = .init(0, 0, 0, 1) };
        var v1: HairPosition = .{ .position = .init(0, 0, 1), .rotation = .init(0.6, 0, 0, 0.8) };
        applyBendTwist(&v0, &v1, omega0, 1, 1, 1.0, 1, 1, 1, &material);
        result.* = .{ v0, v1 };
    }
    try std.testing.expect(!results[0][0].rotation.eql(.init(0, 0, 0, 1))); // Applied
    try std.testing.expect(results[1][0].rotation.eql(.init(0, 0, 0, 1))); // Skipped: the multiplier is NaN
    try std.testing.expect(results[1][1].rotation.eql(.init(0.6, 0, 0, 0.8)));
    try std.testing.expect(results[2][0].rotation.eql(.init(0, 0, 0, 1))); // Skipped: the multiplier is NaN
    try std.testing.expect(results[2][1].rotation.eql(.init(0.6, 0, 0, 0.8)));
}
