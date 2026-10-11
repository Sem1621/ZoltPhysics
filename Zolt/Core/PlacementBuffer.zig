//! Zolt addition, no Jolt file: C++ placement new into a caller buffer
//! Status: complete
//!
//! A buffer that the caller owns and in which the callee constructs an object in place: C++ placement new into
//! `Shape::GetTrianglesContext` and `ConvexShape::SupportBuffer` (and any other "fixed aligned byte buffer"). See
//! Docs/Zolt/CollisionArchitecture.md, decisions D9 and D10.
//!
//! - `emplace(T)` checks size and alignment at compile time (Jolt's static_assert / IsAligned assert) and returns
//!   uninitialized storage. The callee initializes it in place (`ctx.* = .{ ... }` or field by field), which is
//!   required when the object points into itself (ConvexShape's CSGetTrianglesContext holds a SupportBuffer and a
//!   `*const Support` into it) and avoids building a 4 KB object on the stack and copying it.
//! - `get(T)` is the cast back (`(CSGetTrianglesContext &)ioContext`). With `.type_check = true` the buffer
//!   remembers the emplaced type in safe builds and `get` asserts it.
//! - Objects in the buffer are never destroyed (Jolt: "Virtual destructor will not be called on this object!"),
//!   so they must not own resources. The buffer must not be moved while the object is in use.
//!
//! Usage (the support function of a convex shape):
//! ```zig
//! const support = buffer.emplace(SphereWithConvex);
//! support.* = .init(scaled_radius);
//! return &support.base;
//! ```
//!
//! Negative compile checks (verified by hand, Zig cannot test that code does not compile):
//! - a type that is too big: `X (5000 bytes) does not fit in a buffer of 4160 bytes`;
//! - a type that needs more alignment: `X needs a bigger alignment than the buffer`.

const std = @import("std");

pub const Options = struct {
    /// Remember the emplaced type in safe builds and check it in `get`
    type_check: bool = false,
};

/// A buffer of `size` bytes aligned to `alignment` in which one object at a time is constructed in place
pub fn PlacementBuffer(comptime size: usize, comptime alignment: u16, comptime options: Options) type {
    return struct {
        const Self = @This();
        const checked = options.type_check and std.debug.runtime_safety;

        /// Size of the buffer in bytes
        pub const buffer_size = size;
        /// Alignment of the buffer in bytes
        pub const buffer_alignment = alignment;

        data: [size]u8 align(alignment) = undefined,

        /// Name of the emplaced type (safe builds with type_check only)
        type_name: if (checked) ?[]const u8 else void = if (checked) null else {},

        /// Placement new: storage for a `T`, to be initialized in place by the caller
        pub fn emplace(self: *Self, comptime T: type) *T {
            comptime {
                if (@sizeOf(T) > size) @compileError(std.fmt.comptimePrint("{s} ({d} bytes) does not fit in a buffer of {d} bytes", .{ @typeName(T), @sizeOf(T), size }));
                if (@alignOf(T) > alignment) @compileError(@typeName(T) ++ " needs a bigger alignment than the buffer");
            }
            if (checked) self.type_name = @typeName(T);
            return @ptrCast(@alignCast(&self.data));
        }

        /// The object that was emplaced (C style cast back to the derived type)
        pub fn get(self: *Self, comptime T: type) *T {
            if (checked) std.debug.assert(std.mem.eql(u8, self.type_name.?, @typeName(T)));
            return @ptrCast(@alignCast(&self.data));
        }
    };
}

test "PlacementBuffer: emplace in place, get back, self references survive" {
    // An object that points into itself, like ConvexShape's CSGetTrianglesContext
    const SelfReferencing = struct {
        const Self = @This();
        values: [4]u32,
        current: *const u32,

        fn init(self: *Self, first: u32) void {
            self.values = .{ first, first + 1, first + 2, first + 3 };
            self.current = &self.values[2];
        }
    };
    const Small = extern struct { a: u16, b: u16 };

    var buffer: PlacementBuffer(64, 16, .{ .type_check = true }) = .{};
    buffer.emplace(SelfReferencing).init(10);
    const object = buffer.get(SelfReferencing);
    try std.testing.expectEqual(@as(u32, 12), object.current.*);
    try std.testing.expect(@intFromPtr(object) == @intFromPtr(&buffer.data));
    try std.testing.expect(std.mem.isAligned(@intFromPtr(&buffer.data), 16));

    // Reuse the buffer for another type (the previous object is not destroyed, like in Jolt)
    buffer.emplace(Small).* = .{ .a = 1, .b = 2 };
    try std.testing.expectEqual(@as(u16, 2), buffer.get(Small).b);

    // Without type checking the buffer is only the bytes
    var unchecked: PlacementBuffer(8, 4, .{}) = .{};
    unchecked.emplace(Small).* = .{ .a = 3, .b = 4 };
    try std.testing.expectEqual(@as(u16, 3), unchecked.get(Small).a);
    try std.testing.expectEqual(@as(usize, 8), @sizeOf(@TypeOf(unchecked)));
    try std.testing.expectEqual(@as(usize, 8), @TypeOf(unchecked).buffer_size);
}
