//! Port of: Jolt/Compute/ComputeShader.h
//! Status: complete
//!
//! Pattern A RefTarget root (Docs/Zolt/RemainingFeaturesArchitecture.md R9). Its only virtual function is the
//! destructor. `ComputeShaderResult` (`Result<Ref<ComputeShader>>`) is `ComputeSystem.CreateShaderError!Ref(ComputeShader)`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RefCount = @import("../Core/Reference.zig").RefCount;
const virtual = @import("../Core/Virtual.zig");

/// Compute shader handle
pub const ComputeShader = struct {
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *ComputeShader) void,
        /// delete this (generated)
        destroy: *const fn (self: *ComputeShader) void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ComputeShader>)
    ref_count: RefCount = .{},
    /// Frees this shader and the memory it owns
    allocator: Allocator,
    group_size_x: u32,
    group_size_y: u32,
    group_size_z: u32,

    /// The class has no virtual function with a body
    pub const impl = struct {};

    /// Constructor, called by a backend with its most derived type
    pub fn init(comptime T: type, allocator: Allocator, group_size_x: u32, group_size_y: u32, group_size_z: u32) ComputeShader {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator, .group_size_x = group_size_x, .group_size_y = group_size_y, .group_size_z = group_size_z };
    }

    // RefTarget<ComputeShader>

    /// Add a reference to this object
    pub fn addRef(self: *const ComputeShader) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ComputeShader) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Get group sizes
    pub fn getGroupSizeX(self: *const ComputeShader) u32 {
        return self.group_size_x;
    }

    pub fn getGroupSizeY(self: *const ComputeShader) u32 {
        return self.group_size_y;
    }

    pub fn getGroupSizeZ(self: *const ComputeShader) u32 {
        return self.group_size_z;
    }
};
