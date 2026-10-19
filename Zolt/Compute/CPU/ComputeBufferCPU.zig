//! Port of: Jolt/Compute/CPU/ComputeBufferCPU.h, Jolt/Compute/CPU/ComputeBufferCPU.cpp
//! Status: complete
//!
//! The buffer of the CPU compute backend (Docs/Zolt/RemainingFeaturesArchitecture.md R10).
//! - One 16-byte aligned allocation of `size * stride` bytes, with the buffer's allocator.
//! - Zolt (UB4 of R20): without data the memory is zero. Jolt's `Allocate` leaves it uninitialized (the parity reference
//!   runs with zero-filling allocation hooks, so it is zero there too).
//! - `CreateReadBackBuffer` returns the buffer itself with a new reference: the CPU can read the memory directly.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const ComputeBuffer = @import("../ComputeBuffer.zig").ComputeBuffer;

/// Buffer that can be used with the CPU compute system
pub const ComputeBufferCPU = struct {
    pub const overrides = .{ .createReadBackBuffer, .mapInternal, .unmapInternal };

    base: ComputeBuffer,
    /// The memory of the buffer (`size * stride` bytes)
    data: []align(16) u8,

    /// Constructor (`new ComputeBufferCPU(inType, inSize, inStride, inData)`, reference count 0). `data` (Jolt's
    /// inData, null for none) must hold at least `size * stride` bytes, which are copied.
    pub fn create(allocator: Allocator, buffer_type: ComputeBuffer.Type, size: u64, stride: u32, data: ?[]const u8) Allocator.Error!*ComputeBufferCPU {
        const self = try allocator.create(ComputeBufferCPU);
        errdefer allocator.destroy(self);

        const buffer_size = std.math.cast(usize, size * stride) orelse return error.OutOfMemory;
        const memory = try allocator.alignedAlloc(u8, .@"16", buffer_size);
        if (data) |d|
            @memcpy(memory, d[0..buffer_size])
        else
            @memset(memory, 0); // Zolt (UB4): Jolt leaves the memory uninitialized

        self.* = .{ .base = .init(ComputeBufferCPU, allocator, buffer_type, size, stride), .data = memory };
        return self;
    }

    /// Destructor
    pub fn destruct(self: *ComputeBufferCPU) void {
        self.base.allocator.free(self.data);
    }

    /// The read back buffer of the CPU backend is the buffer itself, with a new reference
    pub fn createReadBackBuffer(self: *const ComputeBufferCPU) Allocator.Error!Ref(ComputeBuffer) {
        return .init(@constCast(&self.base)); // Jolt: const_cast<ComputeBufferCPU *>(this)
    }

    /// GetData
    pub fn getData(self: *const ComputeBufferCPU) []align(16) u8 {
        return self.data;
    }

    pub fn mapInternal(self: *ComputeBufferCPU, mode: ComputeBuffer.Mode) [*]u8 {
        _ = mode;
        return self.data.ptr;
    }

    pub fn unmapInternal(self: *ComputeBufferCPU) void {
        _ = self; // Nothing to do
    }
};

test "ComputeBufferCPU: zero fill, data, read back buffer, map" {
    const testing = std.testing;
    const allocator = testing.allocator;

    // Without data the memory is zero (UB4)
    var buffer: Ref(ComputeBuffer) = .init(&(try ComputeBufferCPU.create(allocator, .rw_buffer, 5, 12, null)).base);
    defer buffer.deinit();
    const cpu: *ComputeBufferCPU = @fieldParentPtr("base", buffer.get().?);
    try testing.expectEqual(@as(usize, 60), cpu.getData().len);
    try testing.expect(std.mem.allEqual(u8, cpu.getData(), 0));
    try testing.expect(std.mem.isAligned(@intFromPtr(cpu.getData().ptr), 16));
    try testing.expectEqual(ComputeBuffer.Type.rw_buffer, buffer.get().?.getType());
    try testing.expectEqual(@as(u64, 5), buffer.get().?.getSize());
    try testing.expectEqual(@as(u32, 12), buffer.get().?.getStride());

    // The read back buffer is the buffer itself with a new reference
    var readback = try buffer.get().?.createReadBackBuffer();
    try testing.expect(readback.get().? == buffer.get().?);
    try testing.expectEqual(@as(u32, 2), buffer.get().?.ref_count.get());
    const values = readback.get().?.mapAs([3]f32, .write);
    try testing.expectEqual(@as(usize, 5), values.len);
    values[4] = .{ 1, 2, 3 };
    readback.get().?.unmap();
    readback.deinit();
    try testing.expectEqual(@as(u32, 1), buffer.get().?.ref_count.get());
    try testing.expectEqual(@as(f32, 3), std.mem.bytesAsSlice(f32, buffer.get().?.map(.read))[14]);
    buffer.get().?.unmap();

    // With data the bytes are copied
    const data = [_]u32{ 1, 2, 3 };
    var buffer2: Ref(ComputeBuffer) = .init(&(try ComputeBufferCPU.create(allocator, .buffer, 3, 4, std.mem.sliceAsBytes(&data))).base);
    defer buffer2.deinit();
    try testing.expectEqualSlices(u32, &data, buffer2.get().?.mapAs(u32, .read));
    buffer2.get().?.unmap();

    // Allocation failures
    try testing.checkAllAllocationFailures(allocator, struct {
        fn f(a: Allocator) !void {
            const b = try ComputeBufferCPU.create(a, .upload_buffer, 4, 4, null);
            var r: Ref(ComputeBuffer) = .init(&b.base);
            r.deinit();
        }
    }.f, .{});
}
