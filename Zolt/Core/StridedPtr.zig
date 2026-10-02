//! Port of: Jolt/Core/StridedPtr.h
//! Status: complete
//!
//! `StridedPtr<T>` is `StridedPtr(T)` and `StridedPtr<const T>` is `StridedPtrConst(T)` (Zig types have no
//! const qualifier, compare `Ref` / `RefConst`). The operators become methods:
//!
//! | C++                              | Zig                                                     |
//! |----------------------------------|---------------------------------------------------------|
//! | `StridedPtr()`                   | `.{}` (null pointer, stride 0)                          |
//! | `StridedPtr(ptr)` / `(ptr, s)`   | `init(ptr, .{})` / `init(ptr, .{ .stride = s })`        |
//! | `++p` / `--p`                    | `p.increment()` / `p.decrement()`                       |
//! | `p++` / `p--`                    | `p.postIncrement()` / `p.postDecrement()` (return the old value) |
//! | `p + i` / `p - i`                | `p.add(i)` / `p.sub(i)`                                 |
//! | `p += i` / `p -= i`              | `p = p.add(i)` / `p = p.sub(i)`                         |
//! | `p - q`                          | `p.distance(q)`                                         |
//! | `==` `!=` `<` `<=` `>` `>=`      | `eql`, `!eql`, `less`, `lessOrEqual`, `greater`, `greaterOrEqual` |
//! | `*p`, `p->`                      | `p.deref()` (a pointer, Zig has no references)          |
//! | `p[i]`                           | `p.at(i)` (a pointer)                                   |
//! | `GetPtr()`                       | `getPtr()` (null for a default constructed StridedPtr)  |
//!
//! Offsets and strides are `int` like in the C++; the pointer moves by `offset * stride` bytes.

const std = @import("std");

/// A strided pointer behaves exactly like a normal pointer except that the
/// elements that the pointer points to can be part of a larger structure.
/// The stride gives the number of bytes from one element to the next.
pub fn StridedPtr(comptime T: type) type {
    return StridedPtrImpl(T, *T);
}

/// StridedPtr<const T>: a StridedPtr that gives read only access to the elements
pub fn StridedPtrConst(comptime T: type) type {
    return StridedPtrImpl(T, *const T);
}

fn StridedPtrImpl(comptime T: type, comptime Ptr: type) type {
    return struct {
        const Self = @This();

        /// The element type (value_type)
        pub const ValueType = T;

        /// Pointer to element
        ptr: ?[*]u8 = null,

        /// Stride (number of bytes) between elements
        stride: i32 = 0,

        /// Constructor
        pub fn init(ptr: Ptr, opts: struct { stride: i32 = @sizeOf(T) }) Self {
            return .{ .ptr = @ptrCast(@constCast(ptr)), .stride = opts.stride };
        }

        /// Incrementing / decrementing (++ptr)
        pub fn increment(self: *Self) void {
            self.ptr = advance(self.ptr, self.stride);
        }

        /// --ptr
        pub fn decrement(self: *Self) void {
            self.ptr = retreat(self.ptr, self.stride);
        }

        /// ptr++, returns the value before incrementing
        pub fn postIncrement(self: *Self) Self {
            const old_ptr = self.*;
            self.ptr = advance(self.ptr, self.stride);
            return old_ptr;
        }

        /// ptr--, returns the value before decrementing
        pub fn postDecrement(self: *Self) Self {
            const old_ptr = self.*;
            self.ptr = retreat(self.ptr, self.stride);
            return old_ptr;
        }

        /// ptr + offset
        pub fn add(self: Self, offset: i32) Self {
            var new_ptr = self;
            new_ptr.ptr = advance(new_ptr.ptr, offset * self.stride);
            return new_ptr;
        }

        /// ptr - offset
        pub fn sub(self: Self, offset: i32) Self {
            var new_ptr = self;
            new_ptr.ptr = retreat(new_ptr.ptr, offset * self.stride);
            return new_ptr;
        }

        /// Distance between two pointers in elements (ptr - other)
        pub fn distance(self: Self, other: Self) i32 {
            std.debug.assert(other.stride == self.stride);
            const diff: isize = @bitCast(address(self.ptr) -% address(other.ptr));
            return @intCast(@divTrunc(diff, self.stride));
        }

        /// Comparison operators
        pub fn eql(self: Self, other: Self) bool {
            return address(self.ptr) == address(other.ptr);
        }

        /// ptr < other
        pub fn less(self: Self, other: Self) bool {
            return address(self.ptr) < address(other.ptr);
        }

        /// ptr <= other
        pub fn lessOrEqual(self: Self, other: Self) bool {
            return address(self.ptr) <= address(other.ptr);
        }

        /// ptr > other
        pub fn greater(self: Self, other: Self) bool {
            return address(self.ptr) > address(other.ptr);
        }

        /// ptr >= other
        pub fn greaterOrEqual(self: Self, other: Self) bool {
            return address(self.ptr) >= address(other.ptr);
        }

        /// Access value (operator * and operator ->)
        pub fn deref(self: Self) Ptr {
            return @ptrCast(@alignCast(self.ptr.?));
        }

        /// Access value at offset elements (operator [])
        pub fn at(self: Self, offset: i32) Ptr {
            const ptr = advance(self.ptr, offset * self.stride);
            return @ptrCast(@alignCast(ptr.?));
        }

        /// Explicit conversion
        pub fn getPtr(self: Self) ?Ptr {
            return @ptrCast(@alignCast(self.ptr));
        }

        /// Get stride in bytes
        pub fn getStride(self: Self) i32 {
            return self.stride;
        }

        fn address(ptr: ?[*]u8) usize {
            return if (ptr) |p| @intFromPtr(p) else 0;
        }

        /// ptr + bytes (pointer arithmetic with a signed offset)
        fn advance(ptr: ?[*]u8, bytes: isize) ?[*]u8 {
            return @ptrFromInt(address(ptr) +% @as(usize, @bitCast(bytes)));
        }

        /// ptr - bytes (pointer arithmetic with a signed offset)
        fn retreat(ptr: ?[*]u8, bytes: isize) ?[*]u8 {
            return @ptrFromInt(address(ptr) -% @as(usize, @bitCast(bytes)));
        }
    };
}

test "StridedPtr" {
    const Vertex = extern struct {
        position: [3]f32,
        inv_mass: f32,
        index: i32,
    };
    var vertices: [4]Vertex = undefined;
    for (&vertices, 0..) |*v, i| {
        const f: f32 = @floatFromInt(i);
        v.* = .{ .position = .{ f, 2 * f, 3 * f }, .inv_mass = 1 / (f + 1), .index = @intCast(10 * i) };
    }

    const Ptr = StridedPtr(i32);
    comptime std.debug.assert(Ptr.ValueType == i32);
    const start = Ptr.init(&vertices[0].index, .{ .stride = @sizeOf(Vertex) });
    try std.testing.expectEqual(@as(i32, @sizeOf(Vertex)), start.getStride());
    try std.testing.expectEqual(&vertices[0].index, start.getPtr().?);
    try std.testing.expectEqual(&vertices[0].index, start.deref());
    try std.testing.expectEqual(@as(i32, 20), start.at(2).*);
    start.at(3).* = 42;
    try std.testing.expectEqual(@as(i32, 42), vertices[3].index);

    // Arithmetic
    const end = start.add(4);
    try std.testing.expectEqual(@as(i32, 4), end.distance(start));
    try std.testing.expectEqual(@as(i32, -4), start.distance(end));
    try std.testing.expect(end.sub(2).eql(start.add(2)));
    try std.testing.expectEqual(&vertices[1].index, end.sub(3).deref());
    try std.testing.expectEqual(&vertices[1].index, end.at(-3));

    // Comparison
    try std.testing.expect(start.less(end) and !end.less(start) and !start.less(start));
    try std.testing.expect(start.lessOrEqual(end) and start.lessOrEqual(start) and !end.lessOrEqual(start));
    try std.testing.expect(end.greater(start) and !start.greater(end) and !start.greater(start));
    try std.testing.expect(end.greaterOrEqual(start) and start.greaterOrEqual(start) and !start.greaterOrEqual(end));
    try std.testing.expect(start.eql(start) and !start.eql(end));

    // Incrementing / decrementing
    var p = start;
    p.increment();
    try std.testing.expect(p.eql(start.add(1)));
    p.decrement();
    try std.testing.expect(p.eql(start));
    const old_inc = p.postIncrement();
    try std.testing.expect(old_inc.eql(start) and p.eql(start.add(1)));
    const old_dec = p.postDecrement();
    try std.testing.expect(old_dec.eql(start.add(1)) and p.eql(start));
    p = p.add(2); // p += 2
    try std.testing.expectEqual(&vertices[2].index, p.deref());
    p = p.sub(2); // p -= 2
    try std.testing.expect(p.eql(start));

    // Iterate like a pointer
    var sum: i32 = 0;
    var it = start;
    while (it.less(end)) : (it.increment())
        sum += it.deref().*;
    try std.testing.expectEqual(@as(i32, 0 + 10 + 20 + 42), sum);

    // Const version
    const masses = StridedPtrConst(f32).init(&vertices[0].inv_mass, .{ .stride = @sizeOf(Vertex) });
    comptime std.debug.assert(@TypeOf(masses.deref()) == *const f32);
    try std.testing.expectEqual(@as(f32, 0.5), masses.at(1).*);
    try std.testing.expectEqual(@as(f32, 0.25), masses.add(3).deref().*);

    // Default stride is the size of the element, negative strides walk backwards
    var array = [_]u32{ 1, 2, 3, 4 };
    const forward = StridedPtr(u32).init(&array[0], .{});
    try std.testing.expectEqual(@as(i32, 4), forward.getStride());
    try std.testing.expectEqual(@as(u32, 3), forward.at(2).*);
    var backward = StridedPtr(u32).init(&array[3], .{ .stride = -4 });
    try std.testing.expectEqual(@as(u32, 3), backward.at(1).*);
    backward.increment();
    try std.testing.expectEqual(&array[2], backward.deref());
    try std.testing.expectEqual(@as(i32, 1), backward.distance(StridedPtr(u32).init(&array[3], .{ .stride = -4 })));

    // Default constructed
    const null_ptr: StridedPtr(f32) = .{};
    try std.testing.expectEqual(@as(?*f32, null), null_ptr.getPtr());
    try std.testing.expectEqual(@as(i32, 0), null_ptr.getStride());
    try std.testing.expect(null_ptr.eql(.{}));
}
