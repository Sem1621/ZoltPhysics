//! Port of: Jolt/Shaders/HairSkinRoots.hlsl, Jolt/Shaders/HairSkinRootsBindings.h
//! Status: complete
//!
//! The hair shader that attaches the root of every strand to its triangle of the skinned scalp mesh
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12). It also computes the transform from the stored global pose to the
//! skinned root.
//! - The binding override macros JPH_SHADER_BIND_SCALP_VERTICES and JPH_SHADER_BIND_SCALP_TRIANGLES are not ported
//!   (section 8): the bindings have the default buffer formats.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMat44 = @import("ShaderMat44.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairPosition = HairStructs.HairPosition;
const HairSkinPoint = HairStructs.HairSkinPoint;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairSkinRoots";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairSkinRoots): the field names are the binding
/// names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gSkinPoints: hlsl.Buffer(HairSkinPoint) = .{},
    /// JPH_SHADER_BIND_SCALP_VERTICES(gScalpVertices), by default a JPH_float3 buffer
    gScalpVertices: hlsl.Buffer(JPH_float3) = .{},
    /// JPH_SHADER_BIND_SCALP_TRIANGLES(gScalpTriangles), by default a JPH_uint buffer
    gScalpTriangles: hlsl.Buffer(JPH_uint) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gInitialBishops: hlsl.Buffer(JPH_uint) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gGlobalPoseTransforms: hlsl.RWBuffer(HairGlobalPoseTransform) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerStrandBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid strand
    const strand_idx = tid.x;
    if (strand_idx >= c.num_strands)
        return;

    const sp = b.gSkinPoints.get(strand_idx);

    // Get the vertices of the attached triangle
    const tri_idx = sp.triangle_index *% 3;
    const v0 = ShaderMat44.mat44Mul3x3Vec3(c.scalp_to_head, b.gScalpVertices.get(b.gScalpTriangles.get(tri_idx +% 0)));
    const v1 = ShaderMat44.mat44Mul3x3Vec3(c.scalp_to_head, b.gScalpVertices.get(b.gScalpTriangles.get(tri_idx +% 1)));
    const v2 = ShaderMat44.mat44Mul3x3Vec3(c.scalp_to_head, b.gScalpVertices.get(b.gScalpTriangles.get(tri_idx +% 2)));

    var root: HairPosition = undefined;

    // Set the position of the root
    root.position = v0.mulScalar(sp.u).add(v1.mulScalar(sp.v)).add(v2.mulScalar(1.0 - sp.u - sp.v)).add(c.scalp_to_head.c[3].xyz());

    // Get tangent vector
    const tangent = hlsl.normalize(v1.sub(v0));

    // Get normal of the triangle
    const normal = hlsl.normalize(hlsl.cross(tangent, v2.sub(v0)));

    // Calculate basis for the triangle
    const binormal = hlsl.cross(tangent, normal);
    const triangle_basis = ShaderQuat.quatFromMat33(normal, binormal, tangent);

    // Calculate the new Bishop frame of the root
    root.rotation = ShaderQuat.quatMulQuat(triangle_basis, ShaderQuat.quatDecompress(sp.to_bishop));

    b.gPositions.set(strand_idx, root);

    // Calculate the transform that transforms the stored global pose to the space of the skinned root of the strand
    var transform: HairGlobalPoseTransform = undefined;
    transform.rotation = ShaderQuat.quatMulQuat(root.rotation, ShaderQuat.quatConjugate(ShaderQuat.quatDecompress(b.gInitialBishops.get(strand_idx))));
    transform.position = root.position.sub(ShaderQuat.quatMulVec3(transform.rotation, b.gInitialPositions.get(strand_idx)));
    b.gGlobalPoseTransforms.set(strand_idx, transform);
}
