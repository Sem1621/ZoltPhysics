//! Port of: Jolt/Compute/CPU/ComputeQueueCPU.h, Jolt/Compute/CPU/ComputeQueueCPU.cpp
//! Status: complete
//!
//! The command queue of the CPU compute backend (Docs/Zolt/RemainingFeaturesArchitecture.md R10).
//! - `setShader` asserts that no shader is pending and creates the shader's wrapper.
//! - A bind of a null buffer is ignored. The buffer types are asserted: ConstantBuffer for `setConstantBuffer`; Upload,
//!   Buffer or RW for `setBuffer`; RW for `setRWBuffer`. Like Jolt's release build, a wrong type is bound anyway.
//! - A bound buffer gets a reference in `used_buffers` until the dispatch ends.
//! - `dispatch` runs synchronously on the calling thread, for z, then y, then x (x fastest), with the global thread ID.
//!   Then it destroys the wrapper, releases the buffers and clears the shader.
//! - `scheduleReadback`, `execute` and `wait` do nothing; barriers are ignored.
//! - Zolt: the binding functions return `Allocator.Error` (Jolt aborts when `new` fails). A failed `setShader` changes
//!   nothing. A failed buffer bind drops the pending command: the wrapper, the shader and the buffers bound so far.
//!   Either way the queue is ready for the next `setShader`.
//! - Zolt: `used_buffers` (Jolt: `UnorderedSet<RefConst<ComputeBuffer>>`) is a std hash map. It is never iterated in an
//!   order that changes a result: the dispatch only releases the buffers.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const ReferenceFile = @import("../../Core/Reference.zig");
const RefConst = ReferenceFile.RefConst;
const virtual = @import("../../Core/Virtual.zig");
const ComputeBuffer = @import("../ComputeBuffer.zig").ComputeBuffer;
const ComputeQueue = @import("../ComputeQueue.zig").ComputeQueue;
const ComputeShader = @import("../ComputeShader.zig").ComputeShader;
const ComputeBufferCPU = @import("ComputeBufferCPU.zig").ComputeBufferCPU;
const ComputeShaderCPU = @import("ComputeShaderCPU.zig").ComputeShaderCPU;
const ShaderWrapper = @import("ShaderWrapper.zig").ShaderWrapper;
const hlsl = @import("HLSLToCPP.zig");

/// A command queue for the CPU compute system
pub const ComputeQueueCPU = struct {
    pub const overrides = .{ .setShader, .setConstantBuffer, .setBuffer, .setRWBuffer, .scheduleReadback, .dispatch, .execute, .wait };

    base: ComputeQueue,
    /// Current active shader (Jolt: RefConst<ComputeShaderCPU>)
    shader: RefConst(ComputeShader) = .empty,
    /// The active shader wrapper
    wrapper: ?*ShaderWrapper = null,
    /// Buffers that are in use by the current execution, these will be retained until execution is finished so that we
    /// don't free buffers that are in use. Each key holds a reference.
    used_buffers: std.AutoHashMapUnmanaged(*const ComputeBuffer, void) = .empty,

    /// Constructor (`new ComputeQueueCPU()`, reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*ComputeQueueCPU {
        const self = try allocator.create(ComputeQueueCPU);
        self.* = .{ .base = .init(ComputeQueueCPU, allocator) };
        return self;
    }

    /// Destructor
    pub fn destruct(self: *ComputeQueueCPU) void {
        if (Core.enable_asserts) std.debug.assert(self.shader.get() == null and self.wrapper == null);

        // Jolt's members release the shader and the buffers. Zolt also frees a wrapper that was never dispatched (Jolt
        // leaks it, after the assert above).
        if (self.wrapper) |wrapper| wrapper.destroy();
        self.releaseUsedBuffers();
        self.used_buffers.deinit(self.base.allocator);
        self.shader.deinit();
    }

    /// See: ComputeQueue
    pub fn setShader(self: *ComputeQueueCPU, shader: *const ComputeShader) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(self.shader.get() == null and self.wrapper == null);

        const shader_cpu = virtual.downcast(ComputeShaderCPU, shader); // static_cast<const ComputeShaderCPU *>
        const wrapper = try shader_cpu.createWrapper(self.base.allocator);
        self.shader.set(shader);
        self.wrapper = wrapper;
    }

    /// See: ComputeQueue
    pub fn setConstantBuffer(self: *ComputeQueueCPU, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void {
        const b = buffer orelse return;
        if (Core.enable_asserts) std.debug.assert(b.getType() == .constant_buffer);
        try self.bindBuffer(name, b);
    }

    /// See: ComputeQueue
    pub fn setBuffer(self: *ComputeQueueCPU, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void {
        const b = buffer orelse return;
        if (Core.enable_asserts) std.debug.assert(b.getType() == .upload_buffer or b.getType() == .buffer or b.getType() == .rw_buffer);
        try self.bindBuffer(name, b);
    }

    /// See: ComputeQueue
    pub fn setRWBuffer(self: *ComputeQueueCPU, name: []const u8, buffer: ?*ComputeBuffer, barrier: ComputeQueue.Barrier) Allocator.Error!void {
        _ = barrier;
        const b = buffer orelse return;
        if (Core.enable_asserts) std.debug.assert(b.getType() == .rw_buffer);
        try self.bindBuffer(name, b);
    }

    /// The common part of the three binding functions: `mWrapper->Bind(...)` and `mUsedBuffers.insert(buffer)`. Zolt
    /// inserts first, because the insert can fail.
    fn bindBuffer(self: *ComputeQueueCPU, name: []const u8, buffer: *const ComputeBuffer) Allocator.Error!void {
        const buffer_cpu = virtual.downcast(ComputeBufferCPU, buffer); // static_cast<const ComputeBufferCPU *>
        const entry = self.used_buffers.getOrPut(self.base.allocator, buffer) catch |err| {
            self.dropCommand();
            return err;
        };
        if (!entry.found_existing)
            buffer.addRef();
        self.wrapper.?.bind(name, buffer_cpu.getData()[0..@intCast(buffer.getSize() * buffer.getStride())]);
    }

    /// See: ComputeQueue
    pub fn scheduleReadback(self: *ComputeQueueCPU, dst: *ComputeBuffer, src: *const ComputeBuffer) void {
        _ = .{ self, dst, src }; // Nothing to read back
    }

    /// See: ComputeQueue
    pub fn dispatch(self: *ComputeQueueCPU, thread_groups_x: u32, thread_groups_y: u32, thread_groups_z: u32) void {
        const shader = self.shader.get().?;
        const wrapper = self.wrapper.?;
        const nx = thread_groups_x *% shader.getGroupSizeX();
        const ny = thread_groups_y *% shader.getGroupSizeY();
        const nz = thread_groups_z *% shader.getGroupSizeZ();

        var z: u32 = 0;
        while (z < nz) : (z += 1) {
            var y: u32 = 0;
            while (y < ny) : (y += 1) {
                var x: u32 = 0;
                while (x < nx) : (x += 1) {
                    const tid: hlsl.uint3 = .init(x, y, z);
                    wrapper.main(tid);
                }
            }
        }

        wrapper.destroy();
        self.wrapper = null;

        self.releaseUsedBuffers();
        self.shader.deinit();
    }

    /// Zolt: drop the pending command after a failed bind (the end of `dispatch` without running the shader)
    fn dropCommand(self: *ComputeQueueCPU) void {
        if (self.wrapper) |wrapper| wrapper.destroy();
        self.wrapper = null;
        self.releaseUsedBuffers();
        self.shader.deinit();
    }

    /// See: ComputeQueue
    pub fn execute(self: *ComputeQueueCPU) void {
        _ = self; // Nothing to do
    }

    /// See: ComputeQueue
    pub fn wait(self: *ComputeQueueCPU) void {
        _ = self; // Nothing to do
    }

    /// mUsedBuffers.clear(): release the reference of every used buffer
    fn releaseUsedBuffers(self: *ComputeQueueCPU) void {
        var it = self.used_buffers.keyIterator();
        while (it.next()) |buffer|
            buffer.*.release();
        self.used_buffers.clearRetainingCapacity();
    }
};
