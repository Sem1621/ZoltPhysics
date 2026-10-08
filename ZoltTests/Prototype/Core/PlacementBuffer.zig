//! Zolt addition, no Jolt file: C++ placement new into a caller buffer (proposed location Zolt/Core/PlacementBuffer.zig)
//! Status: complete
//!
//! A buffer that the caller owns and in which the callee constructs an object in place: C++ placement new into
//! `Shape::GetTrianglesContext` and `ConvexShape::SupportBuffer` (and any other "fixed aligned byte buffer").
//!
//! - `emplace(T)` checks size and alignment at compile time (Jolt's static_assert / IsAligned assert) and returns
//!   uninitialized storage. The callee initializes it in place (`ctx.* = .{ ... }` or field by field), which is
//!   required when the object points into itself (ConvexShape's CSGetTrianglesContext holds a SupportBuffer and a
//!   `*const Support` into it) and avoids building a 4 KB object on the stack and copying it.
//! - `get(T)` is the cast back (`(CSGetTrianglesContext &)ioContext`). With `.type_check = true` the buffer
//!   remembers the emplaced type in safe builds and `get` asserts it.
//! - Objects in the buffer are never destroyed (Jolt: "Virtual destructor will not be called on this object!"),
//!   so they must not own resources. The buffer must not be moved while the object is in use.

const std = @import("std");

pub const Options = struct {
    /// Remember the emplaced type in safe builds and check it in `get`
    type_check: bool = false,
};

pub fn PlacementBuffer(comptime size: usize, comptime alignment: u16, comptime options: Options) type {
    return struct {
        const Self = @This();
        const checked = options.type_check and std.debug.runtime_safety;

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
