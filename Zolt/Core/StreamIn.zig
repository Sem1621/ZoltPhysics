//! Port of: Jolt/Core/StreamIn.h
//! Status: complete
//!
//! `StreamIn` is an interface (guide pattern B): a type-erased pointer plus a vtable with the pure virtual
//! functions, created with `StreamIn.init(&impl)` from any type that declares `readBytes`, `isEOF` and `isFailed`
//! (see `StreamInWrapper` in StreamWrapper.zig). The C++ `Read` overloads become:
//! - `read(&value)`: primitives / trivially copyable types (raw bytes), Vec3 / DVec3 (without W) and DMat44
//! - `readArray(T, allocator, &list)`: Array<T> of trivially copyable T (u32 length + elements)
//! - `readString(allocator, &string)`: std::basic_string (u32 length + characters)
//! - `readArrayWith(T, allocator, &list, context, readElement)`: Array<T> with a custom function to read the elements
//! Like in C++ the destination is an in/out parameter: implementations that validate (StateRecorderImpl) compare
//! the bytes that are read with the current value.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const DMat44 = @import("../Math/DMat44.zig").DMat44;
const checkTriviallyCopyable = @import("StreamOut.zig").checkTriviallyCopyable;
const hasUnusedComponents = @import("StreamOut.zig").hasUnusedComponents;

/// Simple binary input stream
pub const StreamIn = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Read a string of bytes from the binary stream
        readBytes: *const fn (ptr: *anyopaque, data: []u8) void,

        /// Returns true when an attempt has been made to read past the end of the file.
        /// Note that this follows the convention of std::basic_ios::eof which only returns true when an attempt is made to read past the end, not when the read pointer is at the end.
        isEOF: *const fn (ptr: *anyopaque) bool,

        /// Returns true if there was an IO failure
        isFailed: *const fn (ptr: *anyopaque) bool,
    };

    /// Wrap any `*T` that declares `readBytes(self, data: []u8) void`, `isEOF(self) bool` and `isFailed(self) bool`
    pub fn init(impl: anytype) StreamIn {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn readBytesThunk(ptr: *anyopaque, data: []u8) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.readBytes(data);
            }

            fn isEOFThunk(ptr: *anyopaque) bool {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.isEOF();
            }

            fn isFailedThunk(ptr: *anyopaque) bool {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.isFailed();
            }

            const vtable: VTable = .{
                .readBytes = readBytesThunk,
                .isEOF = isEOFThunk,
                .isFailed = isFailedThunk,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Read a string of bytes from the binary stream
    pub fn readBytes(self: StreamIn, data: []u8) void {
        self.vtable.readBytes(self.ptr, data);
    }

    /// Returns true when an attempt has been made to read past the end of the file.
    /// Note that this follows the convention of std::basic_ios::eof which only returns true when an attempt is made to read past the end, not when the read pointer is at the end.
    pub fn isEOF(self: StreamIn) bool {
        return self.vtable.isEOF(self.ptr);
    }

    /// Returns true if there was an IO failure
    pub fn isFailed(self: StreamIn) bool {
        return self.vtable.isFailed(self.ptr);
    }

    /// Read a primitive (e.g. float, int, etc.) from the binary stream. `value` is a pointer to the destination.
    /// Also handles the C++ overloads:
    /// - Read a Vec3 (don't read W)
    /// - Read a DVec3 (don't read W)
    /// - Read a DMat44 (don't read W component of translation)
    /// Other types must be trivially copyable with a defined layout (see `StreamOut.checkTriviallyCopyable`).
    pub fn read(self: StreamIn, value: anytype) void {
        const T = DestinationType(@TypeOf(value));
        if (T == Vec3) {
            self.readBytes(std.mem.asBytes(value)[0 .. 3 * @sizeOf(f32)]);
            value.* = Vec3.fromType(value.value); // sFixW
        } else if (T == DVec3) {
            self.readBytes(std.mem.asBytes(value)[0 .. 3 * @sizeOf(f64)]);
            value.* = DVec3.fromType(value.value); // sFixW
        } else if (T == DMat44) {
            // Jolt reads into uninitialized locals, Zolt starts from the current value so that validating streams compare against it
            var x: Vec4 = value.getColumn4(0);
            var y: Vec4 = value.getColumn4(1);
            var z: Vec4 = value.getColumn4(2);
            self.read(&x);
            self.read(&y);
            self.read(&z);

            var t: DVec3 = value.getTranslation();
            self.read(&t);

            value.* = DMat44.init(x, y, z, t);
        } else {
            comptime checkTriviallyCopyable(T);
            self.readBytes(std.mem.asBytes(value));
        }
    }

    /// Read a vector of primitives from the binary stream (Read(Array<T> &))
    pub fn readArray(self: StreamIn, comptime T: type, allocator: Allocator, list: *std.ArrayList(T)) Allocator.Error!void {
        var len: u32 = @intCast(list.items.len); // Initialize to previous array size, this is used for validation in the StateRecorder class
        self.read(&len);
        if (!self.isEOF() and !self.isFailed()) {
            try resize(T, allocator, list, len);
            if (comptime hasUnusedComponents(T)) {
                // These types have unused components that we don't want to read
                for (list.items) |*item|
                    self.read(item);
            } else {
                // Read all elements at once
                comptime checkTriviallyCopyable(T);
                self.readBytes(std.mem.sliceAsBytes(list.items));
            }
        } else list.clearRetainingCapacity();
    }

    /// Read a string from the binary stream (reads the number of characters and then the characters).
    /// `string` is owned by `allocator` and is reallocated to the length that is read (an empty string on failure).
    pub fn readString(self: StreamIn, allocator: Allocator, string: *[]u8) Allocator.Error!void {
        var len: u32 = 0;
        self.read(&len);
        if (!self.isEOF() and !self.isFailed()) {
            // std::string::resize, new characters are zero
            const old_len = string.len;
            string.* = try allocator.realloc(string.*, len);
            if (len > old_len)
                @memset(string.*[old_len..], 0);
            self.readBytes(string.*);
        } else {
            allocator.free(string.*);
            string.* = &.{};
        }
    }

    /// Read a vector of primitives from the binary stream using a custom function to read the elements
    /// (Read(Array<T> &, const F &inReadElement)). `readElement(context, stream, element)` reads one element, it can
    /// allocate (e.g. a LinearCurve member), pass the allocator through the context.
    pub fn readArrayWith(
        self: StreamIn,
        comptime T: type,
        allocator: Allocator,
        list: *std.ArrayList(T),
        context: anytype,
        comptime readElement: fn (@TypeOf(context), StreamIn, *T) Allocator.Error!void,
    ) Allocator.Error!void {
        var len: u32 = @intCast(list.items.len); // Initialize to previous array size, this is used for validation in the StateRecorder class
        self.read(&len);
        if (!self.isEOF() and !self.isFailed()) {
            try resize(T, allocator, list, len);
            for (list.items) |*item|
                try readElement(context, self, item);
        } else list.clearRetainingCapacity();
    }
};

/// The type that `read` writes to: `value` must be a single item pointer to a mutable value
fn DestinationType(comptime P: type) type {
    const info = @typeInfo(P);
    if (info != .pointer or info.pointer.size != .one or info.pointer.is_const)
        @compileError("StreamIn.read: expected a pointer to the destination, got " ++ @typeName(P));
    return info.pointer.child;
}

/// Resize `list` to `len` elements (Array<T>::resize). Like Jolt's Array, new elements are default constructed
/// when T has a default constructor (in Zig: a struct whose fields all have default values) and left undefined otherwise.
fn resize(comptime T: type, allocator: Allocator, list: *std.ArrayList(T), len: u32) Allocator.Error!void {
    const old_len = list.items.len;
    try list.resize(allocator, len);
    if (comptime hasDefaultValue(T))
        for (list.items[@min(old_len, len)..]) |*item| {
            item.* = .{};
        };
}

/// True when `T{}` is valid (a struct with default values for all fields)
fn hasDefaultValue(comptime T: type) bool {
    return switch (@typeInfo(T)) {
        .@"struct" => |s| for (s.fields) |field| {
            if (field.default_value_ptr == null) break false;
        } else true,
        else => false,
    };
}

/// Reads from a byte slice, for the tests
const TestStream = struct {
    bytes: []const u8,
    pos: usize = 0,
    eof: bool = false,

    fn readBytes(self: *TestStream, data: []u8) void {
        const n = @min(data.len, self.bytes.len - self.pos);
        @memcpy(data[0..n], self.bytes[self.pos..][0..n]);
        self.pos += n;
        if (n < data.len) self.eof = true;
    }

    fn isEOF(self: *const TestStream) bool {
        return self.eof;
    }

    fn isFailed(self: *const TestStream) bool {
        return self.eof;
    }
};

test "StreamIn reads the same bytes as Jolt" {
    const allocator = std.testing.allocator;

    const bytes = [_]u8{
        0x12, 0x56, 0x34, 0xde, 0xbc, 0x9a, 0x78, 0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01, 0xfe, 0xff, 0xff, 0xff,
        0x00, 0x00, 0x80, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x34, 0x12, 0x01, 0x02,
    };
    var impl: TestStream = .{ .bytes = &bytes };
    const stream: StreamIn = .init(&impl);

    var v_u8: u8 = 0;
    var v_u16: u16 = 0;
    var v_u32: u32 = 0;
    var v_u64: u64 = 0;
    var v_i32: i32 = 0;
    var v_f32: f32 = 0;
    var v_f64: f64 = 0;
    var v_bool = false;
    const E = enum(u16) { a = 0x1234, b = 0 };
    var v_enum: E = .b;
    var v_array: [2]u8 = .{ 0, 0 };
    stream.read(&v_u8);
    stream.read(&v_u16);
    stream.read(&v_u32);
    stream.read(&v_u64);
    stream.read(&v_i32);
    stream.read(&v_f32);
    stream.read(&v_f64);
    stream.read(&v_bool);
    stream.read(&v_enum);
    stream.read(&v_array);
    try std.testing.expectEqual(@as(u8, 0x12), v_u8);
    try std.testing.expectEqual(@as(u16, 0x3456), v_u16);
    try std.testing.expectEqual(@as(u32, 0x789abcde), v_u32);
    try std.testing.expectEqual(@as(u64, 0x0123456789abcdef), v_u64);
    try std.testing.expectEqual(@as(i32, -2), v_i32);
    try std.testing.expectEqual(@as(f32, 1.0), v_f32);
    try std.testing.expectEqual(@as(f64, -2.0), v_f64);
    try std.testing.expectEqual(true, v_bool);
    try std.testing.expectEqual(E.a, v_enum);
    try std.testing.expectEqual([2]u8{ 1, 2 }, v_array);
    try std.testing.expect(!stream.isEOF() and !stream.isFailed());

    // Reading at the end of the stream is not EOF yet, reading past it is
    var v_extra: u8 = 0;
    stream.read(&v_extra);
    try std.testing.expect(stream.isEOF() and stream.isFailed());

    // Vec3 / DVec3 / DMat44 don't read W and replicate Z into W
    {
        const vec3_bytes = std.mem.toBytes([3]f32{ 1, 2, 3 }) ++ std.mem.toBytes([3]f64{ 4, 5, 6 });
        impl = .{ .bytes = &vec3_bytes };
        var v: Vec3 = Vec3.zero();
        var d: DVec3 = DVec3.zero();
        stream.read(&v);
        stream.read(&d);
        try std.testing.expectEqual(Vec3.Type{ 1, 2, 3, 3 }, v.value);
        try std.testing.expectEqual(DVec3.Type{ 4, 5, 6, 6 }, d.value);
        try std.testing.expect(!stream.isEOF());
    }
    {
        const dmat44_bytes = std.mem.toBytes([12]f32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }) ++ std.mem.toBytes([3]f64{ 13, 14, 15 });
        impl = .{ .bytes = &dmat44_bytes };
        var m = DMat44.identity();
        stream.read(&m);
        try std.testing.expect(m.eql(DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15))));
        try std.testing.expect(!stream.isEOF());
    }

    // Arrays
    {
        const array_bytes = [_]u8{ 3, 0, 0, 0, 1, 0, 2, 0, 3, 0 };
        impl = .{ .bytes = &array_bytes };
        var list: std.ArrayList(u16) = .empty;
        defer list.deinit(allocator);
        try stream.readArray(u16, allocator, &list);
        try std.testing.expectEqualSlices(u16, &.{ 1, 2, 3 }, list.items);

        // Reading past the end clears the array
        try stream.readArray(u16, allocator, &list);
        try std.testing.expect(stream.isEOF());
        try std.testing.expectEqual(@as(usize, 0), list.items.len);
    }
    {
        const array_bytes = std.mem.toBytes(@as(u32, 2)) ++ std.mem.toBytes([6]f32{ 1, 2, 3, 4, 5, 6 }) ++ std.mem.toBytes(@as(u32, 1)) ++ std.mem.toBytes([3]f64{ 7, 8, 9 });
        impl = .{ .bytes = &array_bytes };
        var list: std.ArrayList(Vec3) = .empty;
        defer list.deinit(allocator);
        try stream.readArray(Vec3, allocator, &list);
        try std.testing.expectEqual(@as(usize, 2), list.items.len);
        try std.testing.expectEqual(Vec3.Type{ 4, 5, 6, 6 }, list.items[1].value);
        var dlist: std.ArrayList(DVec3) = .empty;
        defer dlist.deinit(allocator);
        try stream.readArray(DVec3, allocator, &dlist);
        try std.testing.expectEqual(@as(usize, 1), dlist.items.len);
        try std.testing.expectEqual(DVec3.Type{ 7, 8, 9, 9 }, dlist.items[0].value);
        try std.testing.expect(!stream.isEOF());
    }
    {
        const array_bytes = std.mem.toBytes(@as(u32, 1)) ++ std.mem.toBytes([12]f32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 }) ++ std.mem.toBytes([3]f64{ 13, 14, 15 });
        impl = .{ .bytes = &array_bytes };
        var list: std.ArrayList(DMat44) = .empty;
        defer list.deinit(allocator);
        try stream.readArray(DMat44, allocator, &list);
        try std.testing.expectEqual(@as(usize, 1), list.items.len);
        try std.testing.expect(list.items[0].eql(DMat44.init(Vec4.init(1, 2, 3, 4), Vec4.init(5, 6, 7, 8), Vec4.init(9, 10, 11, 12), DVec3.init(13, 14, 15))));
        try std.testing.expect(!stream.isEOF());
    }

    // Strings
    {
        const string_bytes = [_]u8{ 3, 0, 0, 0, 'a', 'b', 'c', 5, 0, 0, 0, 'x' };
        impl = .{ .bytes = &string_bytes };
        var string: []u8 = &.{};
        defer allocator.free(string);
        try stream.readString(allocator, &string);
        try std.testing.expectEqualStrings("abc", string);

        // The string is resized before reading (keeping its prefix, new characters are zero), then partially overwritten
        try stream.readString(allocator, &string);
        try std.testing.expect(stream.isEOF());
        try std.testing.expectEqualStrings("xbc\x00\x00", string);

        // Reading past the end clears the string
        try stream.readString(allocator, &string);
        try std.testing.expectEqual(@as(usize, 0), string.len);
    }

    // Custom element function, new elements are default constructed when the type has default values
    {
        const Pair = struct { a: u8 = 7, b: u8 = 8 };
        const pair_bytes = [_]u8{ 2, 0, 0, 0, 1, 2, 3 };
        impl = .{ .bytes = &pair_bytes };
        var list: std.ArrayList(Pair) = .empty;
        defer list.deinit(allocator);
        try stream.readArrayWith(Pair, allocator, &list, @as(u8, 10), struct {
            fn readElement(offset: u8, s: StreamIn, element: *Pair) Allocator.Error!void {
                s.read(&element.b);
                s.read(&element.a);
                element.a += offset;
            }
        }.readElement);
        try std.testing.expect(stream.isEOF());
        try std.testing.expectEqual(@as(usize, 2), list.items.len);
        try std.testing.expectEqual(Pair{ .a = 12, .b = 1 }, list.items[0]);
        try std.testing.expectEqual(Pair{ .a = 17, .b = 3 }, list.items[1]); // a was not read
    }
}
