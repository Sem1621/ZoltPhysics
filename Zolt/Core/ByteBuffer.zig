//! Port of: Jolt/Core/ByteBuffer.h
//! Status: complete
//!
//! Jolt's `ByteBuffer` derives from `Array<uint8, STLAlignedAllocator<uint8, JPH_CACHE_LINE_SIZE>>`. In Zolt
//! it wraps that array (`vector`, an unmanaged cache line aligned std.ArrayList) and the allocator is
//! passed to every call that may allocate. Use `vector` for the inherited Array functions (`size()` is
//! `vector.items.len`, `reserve(n)` is `vector.ensureTotalCapacityPrecise(allocator, n)`, ...).
//! - The buffer grows like Jolt's `Array::resize`, which reserves exactly the new size (no growth factor).
//! - The pointers returned by `allocate` and `get` point into the buffer, so they are invalidated when it
//!   grows, like in the C++ (reserve the final size upfront to keep them valid).
//! - The position of an object must be aligned for its type: the C++ reinterpret_casts the address, which
//!   is undefined behavior to dereference when misaligned. Zig checks the alignment in safe builds.

const std = @import("std");
const Core = @import("Core.zig");
const math = @import("../Math/Math.zig");

/// Underlying data type for ByteBuffer
pub const ByteBufferVector = std.array_list.Aligned(u8, .fromByteUnits(Core.cache_line_size));

/// Simple byte buffer, aligned to a cache line
pub const ByteBuffer = struct {
    /// The bytes (the ByteBufferVector base class)
    vector: ByteBufferVector = .empty,

    pub const empty: ByteBuffer = .{};

    /// Free the buffer (destructor)
    pub fn deinit(self: *ByteBuffer, allocator: std.mem.Allocator) void {
        self.vector.deinit(allocator);
        self.* = undefined;
    }

    /// Align the size to a multiple of size, returns the length after alignment (Align)
    pub fn alignTo(self: *ByteBuffer, allocator: std.mem.Allocator, size: usize) std.mem.Allocator.Error!usize {
        // Assert power of 2
        std.debug.assert(math.isPowerOf2(size));

        // Calculate new size and resize buffer
        const s = math.alignUp(self.vector.items.len, size);
        try self.resizeWithValue(allocator, s, 0);

        return s;
    }

    /// Allocate block of data of size elements and return the pointer
    pub fn allocate(self: *ByteBuffer, allocator: std.mem.Allocator, comptime T: type, opts: struct { size: usize = 1 }) std.mem.Allocator.Error![]T {
        // Reserve space
        const s = self.vector.items.len;
        const num_bytes = std.math.mul(usize, opts.size, @sizeOf(T)) catch return error.OutOfMemory;
        try self.resize(allocator, std.math.add(usize, s, num_bytes) catch return error.OutOfMemory);

        // Get data pointer
        const data: [*]T = @ptrCast(@alignCast(self.vector.items.ptr + s));
        const result = data[0..opts.size];

        // Construct elements
        if (comptime needsDefaultInitialization(T)) {
            for (result) |*d|
                defaultInitialize(T, d);
        }

        // Return pointer
        return result;
    }

    /// Append data to the buffer
    pub fn appendVector(self: *ByteBuffer, allocator: std.mem.Allocator, comptime T: type, data: []const T) std.mem.Allocator.Error!void {
        const size = data.len * @sizeOf(T);
        const dest = try self.allocate(allocator, u8, .{ .size = size });
        @memcpy(dest, std.mem.sliceAsBytes(data));
    }

    /// Get object at position (an offset in bytes)
    pub fn get(self: *const ByteBuffer, comptime T: type, position: usize) *const T {
        const address: *const u8 = &self.vector.items[position];
        if (T == anyopaque) // Get<void>
            return address;
        return @ptrCast(@alignCast(address));
    }

    /// Get object at position (an offset in bytes), non-const version
    pub fn getMut(self: *ByteBuffer, comptime T: type, position: usize) *T {
        const address: *u8 = &self.vector.items[position];
        if (T == anyopaque) // Get<void>
            return address;
        return @ptrCast(@alignCast(address));
    }

    /// Array::resize(inNewSize): reserve exactly the new size, the new bytes are not initialized
    fn resize(self: *ByteBuffer, allocator: std.mem.Allocator, new_size: usize) std.mem.Allocator.Error!void {
        try self.vector.ensureTotalCapacityPrecise(allocator, new_size);
        self.vector.items.len = new_size;
    }

    /// Array::resize(inNewSize, inValue): reserve exactly the new size, new bytes are set to value
    fn resizeWithValue(self: *ByteBuffer, allocator: std.mem.Allocator, new_size: usize, value: u8) std.mem.Allocator.Error!void {
        try self.vector.ensureTotalCapacityPrecise(allocator, new_size);
        const old_size = self.vector.items.len;
        self.vector.items.len = new_size;
        if (new_size > old_size)
            @memset(self.vector.items[old_size..new_size], value);
    }
};

/// True if default initialization of T (`new (d) T` in C++) writes anything: T or one of its members has
/// field default values (the equivalent of C++ default member initializers)
fn needsDefaultInitialization(comptime T: type) bool {
    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            for (info.fields) |field| {
                if (field.is_comptime)
                    continue;
                if (field.default_value_ptr != null or (info.layout != .@"packed" and needsDefaultInitialization(field.type)))
                    return true;
            }
            return false;
        },
        .array => |info| return needsDefaultInitialization(info.child),
        else => return false,
    }
}

/// Default initialization (`new (d) T`): fields with a default value get it, recursively for nested
/// structs and arrays, other fields stay undefined like members without initializer in C++
fn defaultInitialize(comptime T: type, ptr: *T) void {
    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            inline for (info.fields) |field| {
                if (field.is_comptime) {
                    // Nothing to initialize
                } else if (field.defaultValue()) |value| {
                    @field(ptr, field.name) = value;
                } else if (info.layout != .@"packed" and comptime needsDefaultInitialization(field.type)) {
                    defaultInitialize(field.type, &@field(ptr, field.name));
                }
            }
        },
        .array => |info| {
            for (ptr) |*element|
                defaultInitialize(info.child, element);
        },
        else => {},
    }
}

test "ByteBuffer" {
    const allocator = std.testing.allocator;
    var buffer: ByteBuffer = .empty;
    defer buffer.deinit(allocator);

    // Allocate a single value
    const a = try buffer.allocate(allocator, u32, .{});
    try std.testing.expectEqual(@as(usize, 1), a.len);
    a[0] = 0x12345678;
    try std.testing.expectEqual(@as(usize, 4), buffer.vector.items.len);
    try std.testing.expect(std.mem.isAligned(@intFromPtr(buffer.vector.items.ptr), Core.cache_line_size));

    // The buffer grows exactly (Array::resize reserves the new size)
    try std.testing.expectEqual(@as(usize, 4), buffer.vector.capacity);

    // Allocate several values
    const b = try buffer.allocate(allocator, u16, .{ .size = 3 });
    try std.testing.expectEqual(@as(usize, 3), b.len);
    b[0] = 1;
    b[1] = 2;
    b[2] = 3;
    try std.testing.expectEqual(@as(usize, 10), buffer.vector.items.len);
    try std.testing.expectEqual(@as(usize, 10), buffer.vector.capacity);

    // Align pads with zeros
    try std.testing.expectEqual(@as(usize, 16), try buffer.alignTo(allocator, 8));
    try std.testing.expectEqual(@as(usize, 16), buffer.vector.items.len);
    for (buffer.vector.items[10..16]) |v| try std.testing.expectEqual(@as(u8, 0), v);
    try std.testing.expectEqual(@as(usize, 16), try buffer.alignTo(allocator, 16));
    try std.testing.expectEqual(@as(usize, 16), try buffer.alignTo(allocator, 1));
    try std.testing.expectEqual(@as(usize, 32), try buffer.alignTo(allocator, 32));

    // Append a vector
    const values = [_]u32{ 7, 8, 9 };
    try buffer.appendVector(allocator, u32, &values);
    try std.testing.expectEqual(@as(usize, 44), buffer.vector.items.len);
    try buffer.appendVector(allocator, u32, &.{});
    try std.testing.expectEqual(@as(usize, 44), buffer.vector.items.len);

    // Read back
    try std.testing.expectEqual(@as(u32, 0x12345678), buffer.get(u32, 0).*);
    try std.testing.expectEqual(@as(u16, 2), buffer.get(u16, 6).*);
    try std.testing.expectEqual(@as(u32, 8), buffer.get(u32, 36).*);
    buffer.getMut(u32, 36).* = 10;
    try std.testing.expectEqual(@as(u32, 10), buffer.get(u32, 36).*);
    const any: *const anyopaque = buffer.get(anyopaque, 32);
    try std.testing.expectEqual(@intFromPtr(buffer.vector.items.ptr) + 32, @intFromPtr(any));
    const any_mut: *anyopaque = buffer.getMut(anyopaque, 40);
    try std.testing.expectEqual(@intFromPtr(buffer.vector.items.ptr) + 40, @intFromPtr(any_mut));

    // Zero elements
    const none = try buffer.allocate(allocator, u32, .{ .size = 0 });
    try std.testing.expectEqual(@as(usize, 0), none.len);
    try std.testing.expectEqual(@as(usize, 44), buffer.vector.items.len);
}

test "ByteBuffer constructs elements" {
    const Inner = struct {
        x: u8 = 5,
        y: u8,
    };
    const Header = struct {
        magic: u32 = 0xabcd,
        count: u32,
        inner: [2]Inner,
        flags: packed struct { a: bool = true, b: bool = false } = .{},
    };
    const Plain = extern struct {
        a: u32,
        b: f32,
    };
    comptime std.debug.assert(needsDefaultInitialization(Header));
    comptime std.debug.assert(needsDefaultInitialization(Inner));
    comptime std.debug.assert(needsDefaultInitialization([3]Inner));
    comptime std.debug.assert(!needsDefaultInitialization(Plain));
    comptime std.debug.assert(!needsDefaultInitialization(u32));

    const allocator = std.testing.allocator;
    var buffer: ByteBuffer = .empty;
    defer buffer.deinit(allocator);

    const headers = try buffer.allocate(allocator, Header, .{ .size = 2 });
    for (headers) |*h| {
        try std.testing.expectEqual(@as(u32, 0xabcd), h.magic);
        try std.testing.expectEqual(@as(u8, 5), h.inner[0].x);
        try std.testing.expectEqual(@as(u8, 5), h.inner[1].x);
        try std.testing.expect(h.flags.a and !h.flags.b);
    }
    try std.testing.expectEqual(2 * @sizeOf(Header), buffer.vector.items.len);

    const plain = try buffer.allocate(allocator, Plain, .{});
    plain[0] = .{ .a = 1, .b = 2.0 };
    try std.testing.expectEqual(@as(f32, 2.0), buffer.get(Plain, 2 * @sizeOf(Header)).b);

    // Out of memory
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var failing_buffer: ByteBuffer = .empty;
    defer failing_buffer.deinit(failing.allocator());
    try std.testing.expectError(error.OutOfMemory, failing_buffer.allocate(failing.allocator(), u8, .{}));
    try std.testing.expectError(error.OutOfMemory, failing_buffer.allocate(failing.allocator(), u64, .{ .size = std.math.maxInt(usize) / 4 }));
    try std.testing.expectEqual(@as(usize, 0), failing_buffer.vector.items.len);
}
