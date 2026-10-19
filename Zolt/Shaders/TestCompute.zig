//! Port of: Jolt/Shaders/TestCompute.hlsl, Jolt/Shaders/TestComputeBindings.h
//! Status: complete
//!
//! The first test shader of UnitTests/Compute/ComputeTests.cpp (Docs/Zolt/RemainingFeaturesArchitecture.md R12): it
//! reads an optional buffer and an upload buffer and does a read-modify-write of a RW buffer.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "TestCompute";

/// JPH_SHADER_CONSTANT(int, cTestComputeGroupSize, 64), a comptime_int (see HairStructs.zig)
pub const test_compute_group_size = 64;

/// JPH_SHADER_CONSTANTS_BEGIN(TestComputeContext, gContext)
pub const TestComputeContext = extern struct {
    float3_value: JPH_float3,
    /// Test that this value packs correctly with the float3 preceding it
    uint_value: JPH_uint,
    float3_value2: JPH_float3,
    uint_value2: JPH_uint,
    num_elements: JPH_uint,

    comptime {
        @import("std").debug.assert(@sizeOf(TestComputeContext) == 36);
    }
};

/// The constant block, then JPH_SHADER_BIND_BEGIN(JPH_TestCompute): the field names are the binding names of the HLSL
/// code
pub const Bindings = struct {
    gContext: hlsl.Constants(TestComputeContext) = .{},
    gUploadData: hlsl.Buffer(JPH_uint) = .{},
    gOptionalData: hlsl.Buffer(JPH_uint) = .{},
    gData: hlsl.RWBuffer(JPH_uint) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cTestComputeGroupSize, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Ensure that we do not write out of bounds
    if (tid.x >= c.num_elements)
        return;

    if (c.uint_value2 == 0) {
        // First write, uses optional data and tests that the packing of float3/uint3's works
        // (uint + int: the int converts to uint, the sums wrap)
        b.gData.set(tid.x, b.gOptionalData.get(tid.x) +% @as(u32, @bitCast(hlsl.toInt(c.float3_value2.y))) +% b.gUploadData.get(0));
    } else {
        // Read-modify-write gData
        b.gData.set(tid.x, (b.gData.get(tid.x) +% c.uint_value) *% c.uint_value2);
    }
}
