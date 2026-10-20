//! Port of: Jolt/Compute/CPU/ComputeShaderCPU.h
//! Status: complete
//!
//! The shader handle of the CPU compute backend (Docs/Zolt/RemainingFeaturesArchitecture.md R10). It holds the factory
//! function of the shader's wrapper (ShaderWrapper.Create). `CreateWrapper` allocates the wrapper, so it returns
//! `Allocator.Error`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ComputeShader = @import("../ComputeShader.zig").ComputeShader;
const ShaderWrapper = @import("ShaderWrapper.zig").ShaderWrapper;

/// Compute shader handle for CPU compute
pub const ComputeShaderCPU = struct {
    /// The class has no virtual function
    pub const overrides = .{};

    /// using CreateShader = ShaderWrapper *(*)()
    pub const CreateShader = ShaderWrapper.Create;

    base: ComputeShader,
    create_shader: CreateShader,

    /// Constructor (`new ComputeShaderCPU(...)`, reference count 0)
    pub fn create(allocator: Allocator, create_shader: CreateShader, group_size_x: u32, group_size_y: u32, group_size_z: u32) Allocator.Error!*ComputeShaderCPU {
        const self = try allocator.create(ComputeShaderCPU);
        self.* = .{
            .base = .init(ComputeShaderCPU, allocator, group_size_x, group_size_y, group_size_z),
            .create_shader = create_shader,
        };
        return self;
    }

    /// Create an instance of the shader wrapper, allocated with `allocator`
    pub fn createWrapper(self: *const ComputeShaderCPU, allocator: Allocator) Allocator.Error!*ShaderWrapper {
        return self.create_shader(allocator);
    }
};
