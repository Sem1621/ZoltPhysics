//! Port of: Jolt/Compute/ComputeSystem.h, Jolt/Compute/ComputeSystem.cpp
//! Status: complete
//! Not ported: mShaderLoader (only the GPU backends call it, and they stay deferred: RemainingFeaturesArchitecture.md R21), CreateComputeSystemDX12 / VK / MTL
//!
//! Pattern A RefTarget root (Docs/Zolt/RemainingFeaturesArchitecture.md R9). Hair and HairShaders are written against
//! this interface, like Jolt's, so the only backend (Compute/CPU, JPH_USE_CPU_COMPUTE) plugs in through the vtable and a
//! later GPU backend would too.
//! - JPH_DECLARE_RTTI_ABSTRACT_BASE / JPH_IMPLEMENT_RTTI_ABSTRACT_BASE: `rtti` (`.abstract`, no attributes) with the
//!   `rtti` vtable data entry, so `RTTI.isKindOfObject` works on a ComputeSystem (ComputeTests checks it). Neither
//!   ComputeSystem nor ComputeSystemCPU is registered in a Factory, like in Jolt.
//! - Jolt's `Result<Ref<T>>` returns are error unions with a specific error set (porting guide section 5): "Compute
//!   shader not found" is `error.ShaderNotFound`, "Not implemented" is `error.NotImplemented`. The returned `Ref` holds
//!   the caller's reference.
//! - `CreateComputeSystemCPU()` is `ComputeSystemCPU.create(allocator)` (Compute/CPU/ComputeSystemCPU.zig).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const virtual = @import("../Core/Virtual.zig");
const ComputeBuffer = @import("ComputeBuffer.zig").ComputeBuffer;
const ComputeQueue = @import("ComputeQueue.zig").ComputeQueue;
const ComputeShader = @import("ComputeShader.zig").ComputeShader;

/// Interface to run a workload on the GPU
pub const ComputeSystem = struct {
    /// JPH_DECLARE_RTTI_ABSTRACT_BASE / JPH_IMPLEMENT_RTTI_ABSTRACT_BASE(ComputeSystem)
    pub const rtti = RTTI.declare(@This(), .{ .name = "ComputeSystem", .kind = .abstract });

    /// ComputeShaderResult: the errors of CreateComputeShader
    pub const CreateShaderError = Allocator.Error || error{
        /// "Compute shader not found"
        ShaderNotFound,
    };

    pub const VTable = struct {
        /// GetRTTI(): JPH_RTTI of the most derived class (a data entry: every concrete class declares `rtti`)
        rtti: *const RTTI,
        /// Destructor (generated destructor chain)
        deinit: *const fn (self: *ComputeSystem) void,
        /// delete this (generated)
        destroy: *const fn (self: *ComputeSystem) void,
        /// Compile a compute shader
        createComputeShader: *const fn (self: *ComputeSystem, name: []const u8, group_size_x: u32, group_size_y: u32, group_size_z: u32) CreateShaderError!Ref(ComputeShader),
        /// Create a buffer for use with a compute shader (`data`: Jolt's inData, null for none)
        createComputeBuffer: *const fn (self: *ComputeSystem, buffer_type: ComputeBuffer.Type, size: u64, stride: u32, data: ?[]const u8) Allocator.Error!Ref(ComputeBuffer),
        /// Create a queue for executing compute shaders
        createComputeQueue: *const fn (self: *ComputeSystem) Allocator.Error!Ref(ComputeQueue),
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ComputeSystem>)
    ref_count: RefCount = .{},
    /// Frees this system and the memory it owns
    allocator: Allocator,

    /// All virtual functions with a body are pure (an abstract class)
    pub const impl = struct {};

    /// Constructor, called by a backend with its most derived type
    pub fn init(comptime T: type, allocator: Allocator) ComputeSystem {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator };
    }

    // RefTarget<ComputeSystem>

    /// Add a reference to this object
    pub fn addRef(self: *const ComputeSystem) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ComputeSystem) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Compile a compute shader
    pub fn createComputeShader(self: *ComputeSystem, name: []const u8, group_size_x: u32, opts: struct { group_size_y: u32 = 1, group_size_z: u32 = 1 }) CreateShaderError!Ref(ComputeShader) {
        return self.vtable.createComputeShader(self, name, group_size_x, opts.group_size_y, opts.group_size_z);
    }

    /// Create a buffer for use with a compute shader
    pub fn createComputeBuffer(self: *ComputeSystem, buffer_type: ComputeBuffer.Type, size: u64, stride: u32, data: ?[]const u8) Allocator.Error!Ref(ComputeBuffer) {
        return self.vtable.createComputeBuffer(self, buffer_type, size, stride, data);
    }

    /// Create a queue for executing compute shaders
    pub fn createComputeQueue(self: *ComputeSystem) Allocator.Error!Ref(ComputeQueue) {
        return self.vtable.createComputeQueue(self);
    }
};

/// The errors of CreateComputeSystem (ComputeSystemResult)
pub const CreateSystemError = Allocator.Error || error{
    /// "Not implemented"
    NotImplemented,
};

/// Factory function to create the default compute system for this platform. Fallback implementation when no compute
/// system is available: Zolt has no GPU backend (RemainingFeaturesArchitecture.md R21), use ComputeSystemCPU.create.
pub fn createComputeSystem(allocator: Allocator) CreateSystemError!Ref(ComputeSystem) {
    _ = allocator;
    return error.NotImplemented;
}

/// A minimal backend for the tests of this file (the CPU backend is Compute/CPU/ComputeSystemCPU.zig)
const TestComputeSystem = struct {
    pub const rtti = RTTI.declare(@This(), .{ .name = "TestComputeSystem", .kind = .virtual, .bases = &.{ComputeSystem} });
    pub const overrides = .{ .createComputeShader, .createComputeBuffer, .createComputeQueue };

    base: ComputeSystem,
    /// Set to true by the destructor
    destructed: ?*bool = null,

    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestComputeSystem {
        const self = try allocator.create(TestComputeSystem);
        self.* = .{ .base = .init(TestComputeSystem, allocator) };
        return self;
    }

    pub fn destruct(self: *TestComputeSystem) void {
        if (self.destructed) |d| d.* = true;
    }

    pub fn createComputeShader(self: *TestComputeSystem, name: []const u8, group_size_x: u32, group_size_y: u32, group_size_z: u32) ComputeSystem.CreateShaderError!Ref(ComputeShader) {
        _ = .{ self, name, group_size_x, group_size_y, group_size_z };
        return error.ShaderNotFound;
    }

    pub fn createComputeBuffer(self: *TestComputeSystem, buffer_type: ComputeBuffer.Type, size: u64, stride: u32, data: ?[]const u8) Allocator.Error!Ref(ComputeBuffer) {
        _ = .{ self, buffer_type, size, stride, data };
        return error.OutOfMemory;
    }

    pub fn createComputeQueue(self: *TestComputeSystem) Allocator.Error!Ref(ComputeQueue) {
        _ = self;
        return error.OutOfMemory;
    }
};

test "ComputeSystem: RTTI, virtual calls, reference counting" {
    const testing = std.testing;
    const allocator = testing.allocator;

    // CreateComputeSystem without a GPU backend
    try testing.expectError(error.NotImplemented, createComputeSystem(allocator));

    var destructed = false;
    const system = try TestComputeSystem.createDefault(allocator);
    system.destructed = &destructed;
    var ref: Ref(ComputeSystem) = .init(&system.base);

    // IsKindOf, as ComputeTests checks it for the CPU system
    try testing.expect(ComputeSystem.rtti.kind == .abstract);
    try testing.expect(RTTI.isKindOfObject(ComputeSystem, &system.base, ComputeSystem.rtti));
    try testing.expect(RTTI.isKindOfObject(ComputeSystem, &system.base, TestComputeSystem.rtti));
    try testing.expect(RTTI.getRTTI(ComputeSystem, &system.base) == TestComputeSystem.rtti);

    // The virtual functions dispatch to the backend
    try testing.expectError(error.ShaderNotFound, ref.get().?.createComputeShader("NonExistingShader", 64, .{}));
    try testing.expectError(error.OutOfMemory, ref.get().?.createComputeQueue());

    // The last reference destroys the backend through the vtable
    ref.deinit();
    try testing.expect(destructed);
}
