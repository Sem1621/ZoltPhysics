//! Port of: Jolt/Shaders/HairUpdateRoots.hlsl, Jolt/Shaders/HairUpdateRootsBindings.h
//! Status: complete
//!
//! The hair shader that moves the roots and their global pose transforms a fraction of the way to their targets in each
//! iteration (Docs/Zolt/RemainingFeaturesArchitecture.md R12). It reads `gIterationContext`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const HairStructs = @import("HairStructs.zig");
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairPosition = HairStructs.HairPosition;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairUpdateRoots";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairUpdateRoots): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gTargetPositions: hlsl.Buffer(HairPosition) = .{},
    gTargetGlobalPoseTransforms: hlsl.Buffer(HairGlobalPoseTransform) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gGlobalPoseTransforms: hlsl.RWBuffer(HairGlobalPoseTransform) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerStrandBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;
    const ic = &b.gIterationContext.value;

    // Check if this is a valid strand
    const strand_idx = tid.x;
    if (strand_idx >= c.num_strands)
        return;

    const inv_fraction = 1.0 - ic.iteration_fraction;

    var pos = b.gPositions.get(strand_idx);
    const target_pos = b.gTargetPositions.get(strand_idx);
    pos.position = pos.position.mulScalar(inv_fraction).add(target_pos.position.mulScalar(ic.iteration_fraction));
    pos.rotation = hlsl.normalize(pos.rotation.mulScalar(inv_fraction).add(target_pos.rotation.mulScalar(ic.iteration_fraction)));
    b.gPositions.set(strand_idx, pos);

    var transf = b.gGlobalPoseTransforms.get(strand_idx);
    const target_transf = b.gTargetGlobalPoseTransforms.get(strand_idx);
    transf.position = transf.position.mulScalar(inv_fraction).add(target_transf.position.mulScalar(ic.iteration_fraction));
    transf.rotation = hlsl.normalize(transf.rotation.mulScalar(inv_fraction).add(target_transf.rotation.mulScalar(ic.iteration_fraction)));
    b.gGlobalPoseTransforms.set(strand_idx, transf);
}
