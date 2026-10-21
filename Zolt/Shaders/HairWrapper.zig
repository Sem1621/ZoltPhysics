//! Port of: Jolt/Shaders/HairWrapper.cpp, Jolt/Shaders/HairWrapper.h
//! Status: complete
//!
//! The register function of the 15 hair shaders (Docs/Zolt/RemainingFeaturesArchitecture.md R12). Jolt's file wraps
//! every hair shader in a ShaderWrapper class and generates `HairRegisterShaders`, which registers them with
//! JPH_REGISTER_SHADER. Zolt generates the wrapper classes with `ShaderWrapper.Of(Shader)` when a shader is registered.
//! The registry of ComputeSystemCPU is only searched, so the order does not matter. Zolt keeps Jolt's order anyway.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ComputeSystemCPU = @import("../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;

/// The hair shaders in the order of HairRegisterShaders
pub const shaders = .{
    @import("HairTeleport.zig"),
    @import("HairApplyDeltaTransform.zig"),
    @import("HairSkinVertices.zig"),
    @import("HairSkinRoots.zig"),
    @import("HairApplyGlobalPose.zig"),
    @import("HairCalculateCollisionPlanes.zig"),
    @import("HairGridClear.zig"),
    @import("HairGridAccumulate.zig"),
    @import("HairGridNormalize.zig"),
    @import("HairIntegrate.zig"),
    @import("HairUpdateRoots.zig"),
    @import("HairUpdateStrands.zig"),
    @import("HairUpdateVelocity.zig"),
    @import("HairUpdateVelocityIntegrate.zig"),
    @import("HairCalculateRenderPositions.zig"),
};

/// HairRegisterShaders: register all hair shaders with the CPU compute system (JPH_REGISTER_SHADER(inComputeSystem, X)
/// for each of them). Note that HairShaders.init needs them when the hair simulation runs on the CPU.
pub fn registerHairShaders(compute_system: *ComputeSystemCPU) Allocator.Error!void {
    inline for (shaders) |Shader|
        try compute_system.registerShader(Shader);
}

test "registerHairShaders registers the 15 hair shaders by their Jolt names" {
    const virtual = @import("../Core/Virtual.zig");

    var system = try ComputeSystemCPU.create(std.testing.allocator);
    defer system.deinit();
    const cpu = virtual.downcast(ComputeSystemCPU, system.get().?);
    try registerHairShaders(cpu);

    // The names of HairShaders::Init
    const names = [_][]const u8{
        "HairTeleport",        "HairApplyDeltaTransform",      "HairSkinVertices",             "HairSkinRoots",
        "HairApplyGlobalPose", "HairCalculateCollisionPlanes", "HairGridClear",                "HairGridAccumulate",
        "HairGridNormalize",   "HairIntegrate",                "HairUpdateRoots",              "HairUpdateStrands",
        "HairUpdateVelocity",  "HairUpdateVelocityIntegrate",  "HairCalculateRenderPositions",
    };
    try std.testing.expectEqual(names.len, cpu.shader_registry.count());
    inline for (shaders, names) |Shader, n|
        try std.testing.expectEqualStrings(n, Shader.name);
    for (names) |n| {
        var shader = try system.get().?.createComputeShader(n, 64, .{});
        shader.deinit();
    }
}
