//! Port of: Jolt/Core/StreamOut.h
//! Status: complete
//!
//! `StreamOut` is an interface (guide pattern B): a type-erased pointer plus a vtable with the pure virtual
//! functions, created with `StreamOut.init(&impl)` from any type that declares `writeBytes` and `isFailed`
//! (see `StreamOutWrapper` in StreamWrapper.zig). The C++ `Write` overloads become:
//! - `write(value)`: primitives / trivially copyable types (raw bytes), Vec3 / DVec3 (without W) and DMat44
//! - `writeArray(T, items)`: Array<T> of trivially copyable T (u32 length + elements)
//! - `writeString(string)`: std::basic_string (u32 length + characters)
//! - `writeArrayWith(T, items, context, writeElement)`: Array<T> with a custom function to write the elements
//! The bytes written are the same as Jolt's, so recorded state and serialized data stay compatible.

const std = @import("std");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const DMat44 = @import("../Math/DMat44.zig").DMat44;

/// Simple binary output stream
pub const StreamOut = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Write a string of bytes to the binary stream
        writeBytes: *const fn (ptr: *anyopaque, data: []const u8) void,

        /// Returns true if there was an IO failure
        isFailed: *const fn (ptr: *anyopaque) bool,
    };

    /// Wrap any `*T` that declares `writeBytes(self, data: []const u8) void` and `isFailed(self) bool`
    pub fn init(impl: anytype) StreamOut {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn writeBytesThunk(ptr: *anyopaque, data: []const u8) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.writeBytes(data);
            }

            fn isFailedThunk(ptr: *anyopaque) bool {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.isFailed();
            }

            const vtable: VTable = .{
                .writeBytes = writeBytesThunk,
                .isFailed = isFailedThunk,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Write a string of bytes to the binary stream
    pub fn writeBytes(self: StreamOut, data: []const u8) void {
        self.vtable.writeBytes(self.ptr, data);
    }

    /// Returns true if there was an IO failure
    pub fn isFailed(self: StreamOut) bool {
        return self.vtable.isFailed(self.ptr);
    }

    /// Write a primitive (e.g. float, int, etc.) to the binary stream.
    /// Also handles the C++ overloads:
    /// - Write a Vec3 (don't write W)
    /// - Write a DVec3 (don't write W)
    /// - Write a DMat44 (don't write W component of translation)
    /// Other types must be trivially copyable with a defined layout (see `checkTriviallyCopyable`), their bytes are written as is.
    pub fn write(self: StreamOut, value: anytype) void {
        const T = @TypeOf(value);
        if (T == Vec3) {
            self.writeBytes(std.mem.asBytes(&value)[0 .. 3 * @sizeOf(f32)]);
        } else if (T == DVec3) {
            self.writeBytes(std.mem.asBytes(&value)[0 .. 3 * @sizeOf(f64)]);
        } else if (T == DMat44) {
            self.write(value.getColumn4(0));
            self.write(value.getColumn4(1));
            self.write(value.getColumn4(2));

            self.write(value.getTranslation());
        } else {
            comptime checkTriviallyCopyable(T);
            self.writeBytes(std.mem.asBytes(&value));
        }
    }

    /// Write a vector of primitives to the binary stream (Write(const Array<T> &))
    pub fn writeArray(self: StreamOut, comptime T: type, items: []const T) void {
        const len: u32 = @intCast(items.len);
        self.write(len);
        if (!self.isFailed()) {
            if (comptime hasUnusedComponents(T)) {
                // These types have unused components that we don't want to write
                for (items) |item|
                    self.write(item);
            } else {
                // Write all elements at once
                comptime checkTriviallyCopyable(T);
                self.writeBytes(std.mem.sliceAsBytes(items));
            }
        }
    }

    /// Write a string to the binary stream (writes the number of characters and then the characters)
    pub fn writeString(self: StreamOut, string: []const u8) void {
        const len: u32 = @intCast(string.len);
        self.write(len);
        if (!self.isFailed())
            self.writeBytes(string);
    }

    /// Write a vector of primitives to the binary stream using a custom write function
    /// (Write(const Array<T> &, const F &inWriteElement)). `writeElement(context, element, stream)` writes one element.
    pub fn writeArrayWith(
        self: StreamOut,
        comptime T: type,
        items: []const T,
        context: anytype,
        comptime writeElement: fn (@TypeOf(context), *const T, StreamOut) void,
    ) void {
        const len: u32 = @intCast(items.len);
        self.write(len);
        if (!self.isFailed())
            for (items) |*item|
                writeElement(context, item, self);
    }
};

/// True for the types that are not streamed with all their bytes because they have unused components (Vec3, DVec3, DMat44)
pub fn hasUnusedComponents(comptime T: type) bool {
    return T == Vec3 or T == DVec3 or T == DMat44;
}

/// The Zolt equivalent of the `std::is_trivially_copyable_v<T>` requirement of `Read<T>` / `Write<T>`: the bytes of T
/// are streamed as is, so T needs a defined layout that matches the C++ type (integers and floats without padding
/// bits, bools, enums, arrays and vectors of them, extern / packed structs).
pub fn checkTriviallyCopyable(comptime T: type) void {
    switch (@typeInfo(T)) {
        .int, .float => if (@bitSizeOf(T) != 8 * @sizeOf(T))
            @compileError("stream: " ++ @typeName(T) ++ " has padding bits"),
        .bool => {},
        .@"enum" => |e| checkTriviallyCopyable(e.tag_type),
        .array => |a| checkTriviallyCopyable(a.child),
        .vector => |v| {
            checkTriviallyCopyable(v.child);
            if (@sizeOf(T) != v.len * @sizeOf(v.child))
                @compileError("stream: " ++ @typeName(T) ++ " has padding");
        },
        .@"struct" => |s| switch (s.layout) {
            .@"extern" => {},
            .@"packed" => if (@bitSizeOf(T) != 8 * @sizeOf(T))
                @compileError("stream: " ++ @typeName(T) ++ " has padding bits"),
            .auto => @compileError("stream: " ++ @typeName(T) ++ " must be an extern or packed struct so that its bytes match the C++ type"),
        },
        .comptime_int, .comptime_float => @compileError("stream: give " ++ @typeName(T) ++ " values an explicit type"),
        else => @compileError("stream: " ++ @typeName(T) ++ " can't be streamed as raw bytes"),
    }
}

/// Collects the written bytes, for the tests
const TestStream = struct {
    bytes: std.ArrayList(u8) = .empty,
    failed: bool = false,

    fn writeBytes(self: *TestStream, data: []const u8) void {
        self.bytes.appendSlice(std.testing.allocator, data) catch {
            self.failed = true;
        };
    }

    fn isFailed(self: *const TestStream) bool {
        return self.failed;
    }
};

test "StreamOut writes the same bytes as Jolt" {
    const Float3 = @import("../Math/Float3.zig").Float3;
    const Vec4 = @import("../Math/Vec4.zig").Vec4;
    const Quat = @import("../Math/Quat.zig").Quat;

    var impl: TestStream = .{};
    defer impl.bytes.deinit(std.testing.allocator);
    const stream: StreamOut = .init(&impl);

    // Primitives
    stream.write(@as(u8, 0x12));
    stream.write(@as(u16, 0x3456));
    stream.write(@as(u32, 0x789abcde));
    stream.write(@as(u64, 0x0123456789abcdef));
    stream.write(@as(i32, -2));
    stream.write(@as(f32, 1.0));
    stream.write(@as(f64, -2.0));
    stream.write(true);
    const E = enum(u16) { a = 0x1234 };
    stream.write(E.a);
    stream.write([2]u8{ 1, 2 });
    try std.testing.expectEqualSlices(u8, &.{
        0x12, 0x56, 0x34, 0xde, 0xbc, 0x9a, 0x78, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01, 0xfe, 0xff, 0xff, 0xff,
        0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x34, 0x12, 0x01, 0x02,
    }, impl.bytes.items);
    impl.bytes.clearRetainingCapacity();

    // Math types: Vec3 / DVec3 / DMat44 without W, the others with all their bytes
    stream.write(Vec3.init(1, 2, 3));
    try std.testing.expectEqual(@as(usize, 12), impl.bytes.items.len);
    stream.write(DVec3.init(1, 2, 3));
    try std.testing.expectEqual(@as(usize, 12 + 24), impl.bytes.items.len);
    stream.write(DMat44.identity());
    try std.testing.expectEqual(@as(usize, 12 + 24 + 72), impl.bytes.items.len);
    stream.write(Vec4.init(1, 2, 3, 4));
    stream.write(Quat.identity());
    stream.write(Float3{ .x = 1, .y = 2, .z = 3 });
    try std.testing.expectEqual(@as(usize, 12 + 24 + 72 + 16 + 16 + 12), impl.bytes.items.len);
    impl.bytes.clearRetainingCapacity();

    // Arrays and strings: u32 length followed by the elements
    stream.writeArray(u16, &.{ 1, 2, 3 });
    try std.testing.expectEqualSlices(u8, &.{ 3, 0, 0, 0, 1, 0, 2, 0, 3, 0 }, impl.bytes.items);
    impl.bytes.clearRetainingCapacity();
    stream.writeArray(Vec3, &.{ Vec3.init(1, 2, 3), Vec3.init(4, 5, 6) });
    try std.testing.expectEqual(@as(usize, 4 + 2 * 12), impl.bytes.items.len);
    impl.bytes.clearRetainingCapacity();
    stream.writeArray(DVec3, &.{DVec3.init(1, 2, 3)});
    stream.writeArray(DMat44, &.{DMat44.identity()});
    try std.testing.expectEqual(@as(usize, 4 + 24 + 4 + 72), impl.bytes.items.len);
    impl.bytes.clearRetainingCapacity();
    stream.writeString("abc");
    try std.testing.expectEqualSlices(u8, &.{ 3, 0, 0, 0, 'a', 'b', 'c' }, impl.bytes.items);
    impl.bytes.clearRetainingCapacity();

    // Custom element function
    const Pair = struct { a: u8, b: u8 };
    stream.writeArrayWith(Pair, &.{ .{ .a = 1, .b = 2 }, .{ .a = 3, .b = 4 } }, @as(u8, 10), struct {
        fn writeElement(offset: u8, element: *const Pair, s: StreamOut) void {
            s.write(element.b + offset);
            s.write(element.a + offset);
        }
    }.writeElement);
    try std.testing.expectEqualSlices(u8, &.{ 2, 0, 0, 0, 12, 11, 14, 13 }, impl.bytes.items);
    try std.testing.expect(!stream.isFailed());
}
