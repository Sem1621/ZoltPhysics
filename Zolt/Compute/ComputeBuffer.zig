//! Port of: Jolt/Compute/ComputeBuffer.h
//! Status: complete
//!
//! Pattern A RefTarget root (Docs/Zolt/RemainingFeaturesArchitecture.md R9): the vtable, the atomic reference count and
//! the allocator that frees the object. A backend (Compute/CPU/ComputeBufferCPU.zig) embeds `base: ComputeBuffer`, sets
//! it with `.init(@This(), allocator, ...)` and lists its `overrides`.
//! - `ComputeBufferResult` (`Result<Ref<ComputeBuffer>>`) is `Allocator.Error!Ref(ComputeBuffer)`: a backend's only
//!   failure is running out of memory (the GPU backends stay deferred).
//! - `Map` returns a slice of `size * stride` bytes (Jolt: `void *`), `Map<T>` is `mapAs(T, mode)`.
//! - `mIsMapped` only exists with asserts (JPH_ENABLE_ASSERTS): `void` without them.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const virtual = @import("../Core/Virtual.zig");

/// Buffer that can be read from / written to by a compute shader
pub const ComputeBuffer = struct {
    /// Type of buffer (EType)
    pub const Type = enum {
        /// Buffer that can be written on the CPU and then uploaded to the GPU.
        upload_buffer,
        /// Buffer to be sent from the GPU to the CPU, used to read back data.
        readback_buffer,
        /// A smallish buffer that is used to pass constants to a shader.
        constant_buffer,
        /// Buffer that can be read from by a shader. Must be initialized with data at construction time and is read only thereafter.
        buffer,
        /// Buffer that can be read from and written to by a shader.
        rw_buffer,
    };

    /// Mode in which the buffer is accessed (EMode)
    pub const Mode = enum {
        /// Read only access to the buffer
        read,
        /// Write only access to the buffer (this will discard all previous data in the buffer)
        write,
    };

    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *ComputeBuffer) void,
        /// delete this (generated)
        destroy: *const fn (self: *ComputeBuffer) void,
        /// Create a readback buffer of the same size and stride that can be used to read the data stored in this buffer on CPU.
        /// Note that this could also be implemented as 'return this' in case the underlying implementation allows locking GPU data on CPU directly.
        createReadBackBuffer: *const fn (self: *const ComputeBuffer) Allocator.Error!Ref(ComputeBuffer),
        /// Map the buffer (protected in Jolt)
        mapInternal: *const fn (self: *ComputeBuffer, mode: Mode) [*]u8,
        /// Unmap the buffer (protected in Jolt)
        unmapInternal: *const fn (self: *ComputeBuffer) void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ComputeBuffer>)
    ref_count: RefCount = .{},
    /// Frees this buffer and the memory it owns
    allocator: Allocator,
    type: Type,
    /// Number of elements
    size: u64,
    /// Size of an element in bytes
    stride: u32,
    /// JPH_ENABLE_ASSERTS only
    is_mapped: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {},

    /// All virtual functions with a body are pure (an abstract class)
    pub const impl = struct {};

    /// Constructor, called by a backend with its most derived type
    pub fn init(comptime T: type, allocator: Allocator, buffer_type: Type, size: u64, stride: u32) ComputeBuffer {
        return .{ .vtable = virtual.vtablePtr(VTable, T), .allocator = allocator, .type = buffer_type, .size = size, .stride = stride };
    }

    /// Destructor of this level (virtual ~ComputeBuffer)
    pub fn destruct(self: *ComputeBuffer) void {
        if (Core.enable_asserts) std.debug.assert(!self.is_mapped);
    }

    // RefTarget<ComputeBuffer>

    /// Add a reference to this object
    pub fn addRef(self: *const ComputeBuffer) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ComputeBuffer) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Properties
    pub fn getType(self: *const ComputeBuffer) Type {
        return self.type;
    }

    pub fn getSize(self: *const ComputeBuffer) u64 {
        return self.size;
    }

    pub fn getStride(self: *const ComputeBuffer) u32 {
        return self.stride;
    }

    /// Map / unmap buffer (get pointer to data).
    pub fn map(self: *ComputeBuffer, mode: Mode) []u8 {
        if (Core.enable_asserts) {
            std.debug.assert(!self.is_mapped);
            self.is_mapped = true;
        }
        return self.vtable.mapInternal(self, mode)[0..@intCast(self.size * self.stride)];
    }

    /// Map<T>(EMode): the elements as `T`, whose size must be the stride
    pub fn mapAs(self: *ComputeBuffer, comptime T: type, mode: Mode) []T {
        if (Core.enable_asserts) {
            std.debug.assert(!self.is_mapped);
            self.is_mapped = true;
            std.debug.assert(@sizeOf(T) == self.stride);
        }
        const data: [*]T = @ptrCast(@alignCast(self.vtable.mapInternal(self, mode)));
        return data[0..@intCast(self.size)];
    }

    pub fn unmap(self: *ComputeBuffer) void {
        if (Core.enable_asserts) {
            std.debug.assert(self.is_mapped);
            self.is_mapped = false;
        }
        self.vtable.unmapInternal(self);
    }

    /// Create a readback buffer of the same size and stride that can be used to read the data stored in this buffer on CPU.
    /// Note that this could also be implemented as 'return this' in case the underlying implementation allows locking GPU data on CPU directly.
    pub fn createReadBackBuffer(self: *const ComputeBuffer) Allocator.Error!Ref(ComputeBuffer) {
        return self.vtable.createReadBackBuffer(self);
    }
};
