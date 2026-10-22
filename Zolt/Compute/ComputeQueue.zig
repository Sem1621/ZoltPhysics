//! Port of: Jolt/Compute/ComputeQueue.h
//! Status: complete
//!
//! Pattern A RefTarget root (Docs/Zolt/RemainingFeaturesArchitecture.md R9).
//! - The binding functions return `Allocator.Error!void`: the CPU queue allocates the shader wrapper (`SetShader`) and
//!   the entry that keeps a bound buffer alive (Jolt calls `new` and inserts into an UnorderedSet).
//! - `const char *inName` is a `[]const u8`. Jolt's default arguments are options structs.
//! - `ComputeQueueResult` (`Result<Ref<ComputeQueue>>`) is `Allocator.Error!Ref(ComputeQueue)`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RefCount = @import("../Core/Reference.zig").RefCount;
const virtual = @import("../Core/Virtual.zig");
const ComputeBuffer = @import("ComputeBuffer.zig").ComputeBuffer;
const ComputeShader = @import("ComputeShader.zig").ComputeShader;

/// A command queue for executing compute workloads on the GPU.
///
/// Note that only a single thread should be using a ComputeQueue at any time (although an implementation could be made that is thread safe).
pub const ComputeQueue = struct {
    /// If a barrier should be placed before accessing the buffer (EBarrier)
    pub const Barrier = enum { yes, no };

    pub const VTable = struct {
        /// Destructor (generated destructor chain)
        deinit: *const fn (self: *ComputeQueue) void,
        /// delete this (generated)
        destroy: *const fn (self: *ComputeQueue) void,
        /// Activate a shader. Shader must be set first before buffers can be bound.
        /// After every Dispatch call, the shader must be set again and all buffers must be bound again.
        setShader: *const fn (self: *ComputeQueue, shader: *const ComputeShader) Allocator.Error!void,
        /// Bind a constant buffer to the shader. Note that the contents of the buffer cannot be modified until execution finishes.
        /// A reference to the buffer is added to make sure it stays alive until execution finishes.
        setConstantBuffer: *const fn (self: *ComputeQueue, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void,
        /// Bind a read only buffer to the shader. Note that the contents of the buffer cannot be modified on CPU until execution finishes (only relevant for buffers of type UploadBuffer).
        /// A reference to the buffer is added to make sure it stays alive until execution finishes.
        setBuffer: *const fn (self: *ComputeQueue, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void,
        /// Bind a read/write buffer to the shader.
        /// A reference to the buffer is added to make sure it stays alive until execution finishes.
        setRWBuffer: *const fn (self: *ComputeQueue, name: []const u8, buffer: ?*ComputeBuffer, barrier: Barrier) Allocator.Error!void,
        /// Dispatch a compute shader with the specified number of thread groups
        dispatch: *const fn (self: *ComputeQueue, thread_groups_x: u32, thread_groups_y: u32, thread_groups_z: u32) void,
        /// Schedule buffer to be copied from GPU to CPU.
        /// A reference to the buffers is added to make sure they stay alive until execution finishes.
        scheduleReadback: *const fn (self: *ComputeQueue, dst: *ComputeBuffer, src: *const ComputeBuffer) void,
        /// Execute accumulated command list.
        /// No more commands can be added until Wait is called.
        execute: *const fn (self: *ComputeQueue) void,
        /// After executing, this waits until execution is done.
        /// This also makes sure that any readback operations have completed and the data is available on CPU.
        wait: *const fn (self: *ComputeQueue) void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ComputeQueue>)
    ref_count: RefCount = .{},
    /// Frees this queue and the memory it owns
    allocator: Allocator,

    /// All virtual functions with a body are pure (an abstract class)
    pub const impl = struct {};

    /// Constructor, called by a backend with its most derived type
    pub fn init(comptime T: type, allocator: Allocator) ComputeQueue {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator };
    }

    // RefTarget<ComputeQueue>

    /// Add a reference to this object
    pub fn addRef(self: *const ComputeQueue) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ComputeQueue) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Activate a shader. Shader must be set first before buffers can be bound.
    /// After every Dispatch call, the shader must be set again and all buffers must be bound again.
    pub fn setShader(self: *ComputeQueue, shader: *const ComputeShader) Allocator.Error!void {
        return self.vtable.setShader(self, shader);
    }

    /// Bind a constant buffer to the shader (see the vtable entry)
    pub fn setConstantBuffer(self: *ComputeQueue, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void {
        return self.vtable.setConstantBuffer(self, name, buffer);
    }

    /// Bind a read only buffer to the shader (see the vtable entry)
    pub fn setBuffer(self: *ComputeQueue, name: []const u8, buffer: ?*const ComputeBuffer) Allocator.Error!void {
        return self.vtable.setBuffer(self, name, buffer);
    }

    /// Bind a read/write buffer to the shader (see the vtable entry)
    pub fn setRWBuffer(self: *ComputeQueue, name: []const u8, buffer: ?*ComputeBuffer, opts: struct { barrier: Barrier = .yes }) Allocator.Error!void {
        return self.vtable.setRWBuffer(self, name, buffer, opts.barrier);
    }

    /// Dispatch a compute shader with the specified number of thread groups
    pub fn dispatch(self: *ComputeQueue, thread_groups_x: u32, opts: struct { thread_groups_y: u32 = 1, thread_groups_z: u32 = 1 }) void {
        self.vtable.dispatch(self, thread_groups_x, opts.thread_groups_y, opts.thread_groups_z);
    }

    /// Schedule buffer to be copied from GPU to CPU (see the vtable entry)
    pub fn scheduleReadback(self: *ComputeQueue, dst: *ComputeBuffer, src: *const ComputeBuffer) void {
        self.vtable.scheduleReadback(self, dst, src);
    }

    /// Execute accumulated command list.
    pub fn execute(self: *ComputeQueue) void {
        self.vtable.execute(self);
    }

    /// After executing, this waits until execution is done.
    pub fn wait(self: *ComputeQueue) void {
        self.vtable.wait(self);
    }

    /// Execute and wait for the command list to finish
    /// @see Execute, Wait
    pub fn executeAndWait(self: *ComputeQueue) void {
        self.execute();
        self.wait();
    }
};
