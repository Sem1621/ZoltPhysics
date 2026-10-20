//! Port of: Jolt/Shaders/TestCompute2.hlsl, Jolt/Shaders/TestCompute2Bindings.h
//! Status: complete
//!
//! The second test shader of UnitTests/Compute/ComputeTests.cpp (Docs/Zolt/RemainingFeaturesArchitecture.md R12): it
//! calls the matrix helpers of ShaderMat44.h and the decompression helpers of ShaderVec3.h and ShaderQuat.h.

const std = @import("std");
const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMat44 = @import("ShaderMat44.zig");
const ShaderVec3 = @import("ShaderVec3.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const JPH_Quat = ShaderCore.JPH_Quat;
const JPH_Mat44 = ShaderCore.JPH_Mat44;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "TestCompute2";

/// JPH_SHADER_CONSTANT(int, cTestCompute2GroupSize, 1), a comptime_int (see HairStructs.zig)
pub const test_compute2_group_size = 1;

/// JPH_SHADER_STRUCT_BEGIN(TestCompute2Input)
pub const TestCompute2Input = extern struct {
    mat44_value: JPH_Mat44,
    mat44_mul_value: JPH_float3,
    compressed_vec3: JPH_uint,
    compressed_quat: JPH_uint,
};

/// JPH_SHADER_STRUCT_BEGIN(TestCompute2Output)
pub const TestCompute2Output = extern struct {
    mul3x4_output: JPH_float3,
    mul3x3_output: JPH_float3,
    decompressed_vec3: JPH_float3,
    decompressed_quat: JPH_Quat,
};

comptime {
    std.debug.assert(@sizeOf(TestCompute2Input) == 84);
    std.debug.assert(@sizeOf(TestCompute2Output) == 52);
}

/// JPH_SHADER_BIND_BEGIN(JPH_TestCompute2): the field names are the binding names of the HLSL code
pub const Bindings = struct {
    gInput: hlsl.Buffer(TestCompute2Input) = .{},
    gOutput: hlsl.RWBuffer(TestCompute2Output) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cTestCompute2GroupSize, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const input = b.gInput.get(tid.x);
    var output: TestCompute2Output = undefined;

    output.mul3x4_output = ShaderMat44.mat44Mul3x4Vec3(input.mat44_value, input.mat44_mul_value);
    output.mul3x3_output = ShaderMat44.mat44Mul3x3Vec3(input.mat44_value, input.mat44_mul_value);

    output.decompressed_vec3 = ShaderVec3.vec3DecompressUnit(input.compressed_vec3);

    output.decompressed_quat = ShaderQuat.quatDecompress(input.compressed_quat);

    b.gOutput.set(tid.x, output);
}
