//! Port of: Jolt/Compute/CPU/ComputeSystemCPU.h, Jolt/Compute/CPU/ComputeSystemCPU.cpp
//! Status: complete
//!
//! The CPU compute backend (JPH_USE_CPU_COMPUTE, Docs/Zolt/RemainingFeaturesArchitecture.md R10). It runs the Zig ports
//! of Jolt's HLSL shaders (Zolt/Shaders) on the calling thread. It is always compiled.
//! - `CreateComputeSystemCPU()` is `ComputeSystemCPU.create(allocator)`; `rtti` is JPH_DECLARE_RTTI_VIRTUAL with the
//!   base ComputeSystem, so `createDefault` exists. The class is not in a Factory, like in Jolt.
//! - The shader registry is per instance (no global registry): `shader_registry` maps a shader name to the factory
//!   function of its wrapper. It is only searched, so its order does not matter. A second registration of a name
//!   overwrites the first, like Jolt's `mShaderRegistry[inName] = inCreateShader`.
//! - JPH_REGISTER_SHADER(sys, X) is `sys.registerShader(X)` for the Zig shader file X. Jolt's member function
//!   `RegisterShader(inName, inCreateShader)` is `registerShaderFunction` (the name is taken by the macro's port).
//! - `createComputeShader` looks the name up. The group sizes come from the caller, not from the shader, like Jolt.
//! - Everything the system creates uses the system's allocator and keeps it, so it may outlive the system.
//! - Zolt: registering allocates in the registry, so it returns `Allocator.Error`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const virtual = @import("../../Core/Virtual.zig");
const ComputeBuffer = @import("../ComputeBuffer.zig").ComputeBuffer;
const ComputeQueue = @import("../ComputeQueue.zig").ComputeQueue;
const ComputeShader = @import("../ComputeShader.zig").ComputeShader;
const ComputeSystem = @import("../ComputeSystem.zig").ComputeSystem;
const ComputeBufferCPU = @import("ComputeBufferCPU.zig").ComputeBufferCPU;
const ComputeQueueCPU = @import("ComputeQueueCPU.zig").ComputeQueueCPU;
const ComputeShaderCPU = @import("ComputeShaderCPU.zig").ComputeShaderCPU;
const ShaderWrapper = @import("ShaderWrapper.zig").ShaderWrapper;

/// Interface to run a workload on the CPU
/// This is intended mainly for debugging purposes and is not optimized for performance
pub const ComputeSystemCPU = struct {
    /// JPH_DECLARE_RTTI_VIRTUAL / JPH_IMPLEMENT_RTTI_VIRTUAL(ComputeSystemCPU)
    pub const rtti = RTTI.declare(@This(), .{ .name = "ComputeSystemCPU", .kind = .virtual, .bases = &.{ComputeSystem} });

    pub const overrides = .{ .createComputeShader, .createComputeBuffer, .createComputeQueue };

    /// using CreateShader = ComputeShaderCPU::CreateShader
    pub const CreateShader = ComputeShaderCPU.CreateShader;

    base: ComputeSystem,
    /// UnorderedMap<string_view, CreateShader>: the names are not copied, they must outlive the system (the names of
    /// the Zig shader files are string literals)
    shader_registry: std.StringHashMapUnmanaged(CreateShader) = .empty,

    /// `new ComputeSystemCPU()` (reference count 0)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*ComputeSystemCPU {
        const self = try allocator.create(ComputeSystemCPU);
        self.* = .{ .base = .init(ComputeSystemCPU, allocator) };
        return self;
    }

    /// Destructor
    pub fn destruct(self: *ComputeSystemCPU) void {
        self.shader_registry.deinit(self.base.allocator);
    }

    /// Factory function to create a compute system that uses the CPU (CreateComputeSystemCPU). The returned `Ref` holds
    /// the caller's reference.
    pub fn create(allocator: Allocator) Allocator.Error!Ref(ComputeSystem) {
        return .init(&(try createDefault(allocator)).base);
    }

    /// RegisterShader(inName, inCreateShader): register the factory function of a shader wrapper under a name. `name`
    /// must outlive the system.
    pub fn registerShaderFunction(self: *ComputeSystemCPU, name: []const u8, create_shader: CreateShader) Allocator.Error!void {
        try self.shader_registry.put(self.base.allocator, name, create_shader);
    }

    /// JPH_REGISTER_SHADER(sys, Shader): register the Zig shader file `Shader` (Zolt/Shaders) under `Shader.name`
    pub fn registerShader(self: *ComputeSystemCPU, comptime Shader: type) Allocator.Error!void {
        try self.registerShaderFunction(Shader.name, ShaderWrapper.Of(Shader).create);
    }

    // See: ComputeSystem

    /// "Compute shader not found" is error.ShaderNotFound
    pub fn createComputeShader(self: *ComputeSystemCPU, name: []const u8, group_size_x: u32, group_size_y: u32, group_size_z: u32) ComputeSystem.CreateShaderError!Ref(ComputeShader) {
        const create_shader = self.shader_registry.get(name) orelse return error.ShaderNotFound; // "Compute shader not found"
        const shader = try ComputeShaderCPU.create(self.base.allocator, create_shader, group_size_x, group_size_y, group_size_z);
        return .init(&shader.base);
    }

    pub fn createComputeBuffer(self: *ComputeSystemCPU, buffer_type: ComputeBuffer.Type, size: u64, stride: u32, data: ?[]const u8) Allocator.Error!Ref(ComputeBuffer) {
        const buffer = try ComputeBufferCPU.create(self.base.allocator, buffer_type, size, stride, data);
        return .init(&buffer.base);
    }

    pub fn createComputeQueue(self: *ComputeSystemCPU) Allocator.Error!Ref(ComputeQueue) {
        const queue = try ComputeQueueCPU.create(self.base.allocator);
        return .init(&queue.base);
    }
};

// Tests of the backend. ZoltTests/Compute/ComputeTests.zig has Jolt's test (TestComputeCPU) with the two test shaders.

const testing = std.testing;
const hlsl = @import("HLSLToCPP.zig");

/// A shader for the tests below: out[i] = in[i] * mul + i + 1000 * y + 100000 * z, for i < num
const TestShader = struct {
    pub const name = "ZoltTestShader";

    const Context = extern struct { mul: u32, num: u32 };

    pub const Bindings = struct {
        gContext: hlsl.Constants(Context) = .{},
        gInput: hlsl.Buffer(u32) = .{},
        gOutput: hlsl.RWBuffer(u32) = .{},
    };

    pub fn main(b: *Bindings, tid: hlsl.uint3) void {
        if (tid.x >= b.gContext.value.num)
            return;
        const index = tid.x + tid.y * b.gContext.value.num + tid.z * 2 * b.gContext.value.num;
        b.gOutput.set(index, b.gInput.get(tid.x) *% b.gContext.value.mul +% tid.x +% 1000 * tid.y +% 100000 * tid.z);
    }
};

test "ComputeSystemCPU: RTTI, registry, dispatch order, references" {
    const allocator = testing.allocator;

    var system = try ComputeSystemCPU.create(allocator);
    defer system.deinit();
    const s = system.get().?;
    try testing.expect(RTTI.isKindOfObject(ComputeSystem, s, ComputeSystem.rtti));
    try testing.expect(RTTI.isKindOfObject(ComputeSystem, s, ComputeSystemCPU.rtti));
    try testing.expect(RTTI.getRTTI(ComputeSystem, s) == ComputeSystemCPU.rtti);
    const cpu = virtual.downcast(ComputeSystemCPU, s);

    // Unknown shader
    try testing.expectError(error.ShaderNotFound, s.createComputeShader(TestShader.name, 4, .{}));
    try cpu.registerShader(TestShader);
    try cpu.registerShader(TestShader); // A second registration overwrites the first

    // The group sizes come from the caller
    var shader = try s.createComputeShader(TestShader.name, 4, .{ .group_size_y = 2, .group_size_z = 1 });
    defer shader.deinit();
    try testing.expectEqual(@as(u32, 4), shader.get().?.getGroupSizeX());
    try testing.expectEqual(@as(u32, 2), shader.get().?.getGroupSizeY());
    try testing.expectEqual(@as(u32, 1), shader.get().?.getGroupSizeZ());

    var queue = try s.createComputeQueue();
    defer queue.deinit();
    const q = queue.get().?;

    // Buffers: the context, the input, the output (2 * 2 * 3 elements: y < 2, z < 2)
    var context = try s.createComputeBuffer(.constant_buffer, 1, 8, std.mem.sliceAsBytes(&[_]u32{ 3, 3 }));
    const input_data = [_]u32{ 10, 20, 30, 40 };
    var input = try s.createComputeBuffer(.buffer, 4, 4, std.mem.sliceAsBytes(&input_data));
    var output = try s.createComputeBuffer(.rw_buffer, 12, 4, null);
    defer output.deinit();

    try q.setShader(shader.get().?);
    try q.setConstantBuffer("gContext", context.get().?);
    try q.setBuffer("gInput", input.get().?);
    try q.setBuffer("gInput", input.get().?); // A second bind of the same buffer keeps one reference
    try q.setRWBuffer("gOutput", output.get().?, .{});
    try q.setBuffer("gUnused", null); // A null buffer is ignored

    // The queue keeps the buffers alive
    try testing.expectEqual(@as(u32, 2), input.get().?.ref_count.get());
    context.deinit();
    input.deinit();

    // 1 group of 4 x 2 x 1 threads, twice in z
    q.dispatch(1, .{ .thread_groups_y = 1, .thread_groups_z = 2 });
    q.executeAndWait();

    // The dispatch released the buffers (the context and the input are destroyed: no leak) and the shader
    try testing.expectEqual(@as(u32, 1), output.get().?.ref_count.get());
    try testing.expectEqual(@as(u32, 1), shader.get().?.ref_count.get());

    var readback = try output.get().?.createReadBackBuffer();
    defer readback.deinit();
    const result = readback.get().?.mapAs(u32, .read);
    try testing.expectEqualSlices(u32, &.{ 30, 61, 92, 1030, 1061, 1092, 100030, 100061, 100092, 101030, 101061, 101092 }, result);
    readback.get().?.unmap();

    // The shader can be set again after a dispatch
    try q.setShader(shader.get().?);
    q.dispatch(0, .{});
}

test "ComputeSystemCPU: the dispatch order is z, y, x" {
    const allocator = testing.allocator;

    const OrderShader = struct {
        pub const name = "ZoltOrderShader";

        pub const Bindings = struct {
            gCounter: hlsl.RWBuffer(u32) = .{},
            gOrder: hlsl.RWBuffer(hlsl.uint3) = .{},
        };

        pub fn main(b: *Bindings, tid: hlsl.uint3) void {
            const index = hlsl.atomicAdd(b.gCounter.at(0), 1) - 1;
            b.gOrder.set(index, tid);
        }
    };

    var system = try ComputeSystemCPU.create(allocator);
    defer system.deinit();
    const s = system.get().?;
    try virtual.downcast(ComputeSystemCPU, s).registerShader(OrderShader);

    var shader = try s.createComputeShader(OrderShader.name, 2, .{ .group_size_y = 1, .group_size_z = 1 });
    defer shader.deinit();
    var queue = try s.createComputeQueue();
    defer queue.deinit();
    var counter = try s.createComputeBuffer(.rw_buffer, 1, 4, null);
    defer counter.deinit();
    var order = try s.createComputeBuffer(.rw_buffer, 8, 12, null);
    defer order.deinit();

    const q = queue.get().?;
    try q.setShader(shader.get().?);
    try q.setRWBuffer("gCounter", counter.get().?, .{});
    try q.setRWBuffer("gOrder", order.get().?, .{ .barrier = .no });
    q.dispatch(1, .{ .thread_groups_y = 2, .thread_groups_z = 2 });

    const result = order.get().?.mapAs(hlsl.uint3, .read);
    defer order.get().?.unmap();
    const expected = [_]hlsl.uint3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0), .init(1, 1, 0), .init(0, 0, 1), .init(1, 0, 1), .init(0, 1, 1), .init(1, 1, 1) };
    for (expected, result) |e, r|
        try testing.expect(e.eql(r));
}

test "ComputeSystemCPU: allocation failures" {
    try testing.checkAllAllocationFailures(testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            var system = try ComputeSystemCPU.create(allocator);
            defer system.deinit();
            const s = system.get().?;
            try virtual.downcast(ComputeSystemCPU, s).registerShader(TestShader);

            var shader = try s.createComputeShader(TestShader.name, 4, .{});
            defer shader.deinit();
            var queue = try s.createComputeQueue();
            defer queue.deinit();
            var context = try s.createComputeBuffer(.constant_buffer, 1, 8, std.mem.sliceAsBytes(&[_]u32{ 1, 4 }));
            defer context.deinit();
            var input = try s.createComputeBuffer(.upload_buffer, 4, 4, null);
            defer input.deinit();
            var output = try s.createComputeBuffer(.rw_buffer, 4, 4, null);
            defer output.deinit();

            // A failed bind drops the pending command (nothing leaks, the queue is ready for the next shader)
            const q = queue.get().?;
            try q.setShader(shader.get().?);
            try q.setConstantBuffer("gContext", context.get().?);
            try q.setBuffer("gInput", input.get().?);
            try q.setRWBuffer("gOutput", output.get().?, .{});
            q.dispatch(1, .{});
        }
    }.f, .{});
}
