//! Port of: Jolt/Shaders/TestComputeWrapper.cpp
//! Status: complete
//!
//! The register functions of the two test shaders (Docs/Zolt/RemainingFeaturesArchitecture.md R12). Jolt's file wraps
//! TestCompute.hlsl and TestCompute2.hlsl in a ShaderWrapper class each and generates `RegisterShaderTestCompute` and
//! `RegisterShaderTestCompute2` (JPH_REGISTER_SHADER(sys, TestCompute)). Zolt generates the wrapper classes with
//! `ShaderWrapper.Of(Shader)` when a shader is registered.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ComputeSystemCPU = @import("../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
const TestCompute = @import("TestCompute.zig");
const TestCompute2 = @import("TestCompute2.zig");

/// JPH_REGISTER_SHADER(inComputeSystem, TestCompute)
pub fn registerTestCompute(compute_system: *ComputeSystemCPU) Allocator.Error!void {
    try compute_system.registerShader(TestCompute);
}

/// JPH_REGISTER_SHADER(inComputeSystem, TestCompute2)
pub fn registerTestCompute2(compute_system: *ComputeSystemCPU) Allocator.Error!void {
    try compute_system.registerShader(TestCompute2);
}
