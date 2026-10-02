//! Zolt addition, no Jolt file: Jolt's `VERTEX_ARRAY` template parameter (`AABox::GetSupportingFace`, the
//! `ClipPoly*` functions, `ConvexSupport`, ...).
//! Status: complete
//!
//! Jolt's templates take any array of Vec3 with `size()`, `operator []`, `push_back`, `resize` and `clear`: in
//! practice a `StaticArray<Vec3, N>` (e.g. `Shape::SupportingFace`) or an `Array<Vec3>`. In Zolt a function that
//! writes a vertex array takes it as an `anytype` argument, which is one of:
//! - `*StaticArray(Vec3, N)`: fixed capacity, never allocates (exceeding the capacity asserts, like Jolt).
//!   This is what Jolt's own code uses.
//! - `VertexArrayList`: a `std.ArrayList(Vec3)` together with the allocator it uses (Jolt's `Array<Vec3>`).
//!
//! Such a function returns `VertexArray.Error(@TypeOf(out))!void`. For a StaticArray this error set is empty, so
//! callers can always `try` the call, also from a function that does not return an error union:
//! ```
//! var face: StaticArray(Vec3, 32) = .empty;
//! try box.getSupportingFace(direction, &face); // never fails
//!
//! var list: std.ArrayList(Vec3) = .empty;
//! defer list.deinit(allocator);
//! try box.getSupportingFace(direction, VertexArrayList.init(allocator, &list)); // can fail with error.OutOfMemory
//! ```
//! Generic code that forwards a vertex array declares the same return type and uses the helpers below
//! (`items`, `len`, `append`, `resize`, `clear`, `Temporary`) instead of the methods of the array.
//! Read only vertex arrays (Jolt's `const VERTEX_ARRAY &`) are plain `[]const Vec3` slices.

const std = @import("std");
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// A `std.ArrayList(Vec3)` with the allocator that it uses, so that it can be passed where Jolt takes a `VERTEX_ARRAY`
/// (Jolt's `Array<Vec3>`). It does not own the list: pass it by value, the list itself stays with the caller.
pub const VertexArrayList = struct {
    allocator: std.mem.Allocator,
    list: *std.ArrayList(Vec3),

    pub fn init(allocator: std.mem.Allocator, list: *std.ArrayList(Vec3)) VertexArrayList {
        return .{ .allocator = allocator, .list = list };
    }
};

const Kind = enum { static_array, array_list };

/// Classify a vertex array argument type
fn kind(comptime T: type) Kind {
    if (T == VertexArrayList)
        return .array_list;
    switch (@typeInfo(T)) {
        .pointer => |pointer| if (pointer.size == .one and !pointer.is_const) {
            const C = pointer.child;
            if (@typeInfo(C) == .@"struct" and @hasDecl(C, "capacity") and C == StaticArray(Vec3, C.capacity))
                return .static_array;
        },
        else => {},
    }
    @compileError("expected a vertex array (*StaticArray(Vec3, N) or VertexArrayList), got " ++ @typeName(T));
}

/// Error set of the operations on a vertex array of type `T`: empty for a StaticArray, error.OutOfMemory for a VertexArrayList
pub fn Error(comptime T: type) type {
    return switch (kind(T)) {
        .static_array => error{},
        .array_list => std.mem.Allocator.Error,
    };
}

/// The vertices of the array (`begin()` .. `end()`, `operator []`)
pub fn items(array: anytype) []Vec3 {
    return switch (comptime kind(@TypeOf(array))) {
        .static_array => array.slice(),
        .array_list => array.list.items,
    };
}

/// Number of vertices in the array (`size()`)
pub fn len(array: anytype) usize {
    return items(array).len;
}

/// Remove all vertices (`clear()`), keeps the memory
pub fn clear(array: anytype) void {
    switch (comptime kind(@TypeOf(array))) {
        .static_array => array.clear(),
        .array_list => array.list.clearRetainingCapacity(),
    }
}

/// Add a vertex at the end (`push_back`)
pub fn append(array: anytype, vertex: Vec3) Error(@TypeOf(array))!void {
    switch (comptime kind(@TypeOf(array))) {
        .static_array => array.append(vertex),
        .array_list => try array.list.append(array.allocator, vertex),
    }
}

/// Resize the array, new vertices are undefined (`resize`)
pub fn resize(array: anytype, new_len: usize) Error(@TypeOf(array))!void {
    switch (comptime kind(@TypeOf(array))) {
        .static_array => array.resize(@intCast(new_len)),
        .array_list => try array.list.resize(array.allocator, new_len),
    }
}

/// Storage of a temporary vertex array of the same kind as the vertex array type `T` (Jolt: a local `VERTEX_ARRAY`):
/// a `StaticArray(Vec3, N)` for a `*StaticArray(Vec3, N)`, a `std.ArrayList(Vec3)` for a VertexArrayList.
/// Initialize it with `.empty`, use it through `temporary(like, &storage)` and free it with `deinitTemporary`.
pub fn Temporary(comptime T: type) type {
    return switch (kind(T)) {
        .static_array => @typeInfo(T).pointer.child,
        .array_list => std.ArrayList(Vec3),
    };
}

/// A vertex array of the same type as `like` that refers to the temporary `storage`
pub fn temporary(like: anytype, storage: *Temporary(@TypeOf(like))) @TypeOf(like) {
    return switch (comptime kind(@TypeOf(like))) {
        .static_array => storage,
        .array_list => .init(like.allocator, storage),
    };
}

/// Free the memory of a temporary created for a vertex array like `like`
pub fn deinitTemporary(like: anytype, storage: *Temporary(@TypeOf(like))) void {
    switch (comptime kind(@TypeOf(like))) {
        .static_array => {},
        .array_list => storage.deinit(like.allocator),
    }
}

test "VertexArray with StaticArray" {
    var array: StaticArray(Vec3, 4) = .empty;
    const out = &array;
    try append(out, Vec3.init(1, 2, 3));
    try append(out, Vec3.init(4, 5, 6));
    try std.testing.expectEqual(@as(usize, 2), len(out));
    try std.testing.expect(items(out)[1].eql(Vec3.init(4, 5, 6)));
    try resize(out, 3);
    try std.testing.expectEqual(@as(usize, 3), len(out));
    clear(out);
    try std.testing.expectEqual(@as(usize, 0), len(out));
    try std.testing.expect(Error(@TypeOf(out)) == error{});

    var storage: Temporary(@TypeOf(out)) = .empty;
    defer deinitTemporary(out, &storage);
    const tmp = temporary(out, &storage);
    try append(tmp, Vec3.init(7, 8, 9));
    try std.testing.expectEqual(@as(u32, 1), storage.len);
}

test "VertexArray with std.ArrayList" {
    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    const out = VertexArrayList.init(allocator, &list);
    try append(out, Vec3.init(1, 2, 3));
    try append(out, Vec3.init(4, 5, 6));
    try std.testing.expectEqual(@as(usize, 2), len(out));
    try std.testing.expect(items(out)[1].eql(Vec3.init(4, 5, 6)));
    try resize(out, 3);
    try std.testing.expectEqual(@as(usize, 3), list.items.len);
    clear(out);
    try std.testing.expectEqual(@as(usize, 0), len(out));

    var storage: Temporary(@TypeOf(out)) = .empty;
    defer deinitTemporary(out, &storage);
    const tmp = temporary(out, &storage);
    try append(tmp, Vec3.init(7, 8, 9));
    try std.testing.expectEqual(@as(usize, 1), storage.items.len);

    // Out of memory is reported
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var failing_list: std.ArrayList(Vec3) = .empty;
    try std.testing.expectError(error.OutOfMemory, append(VertexArrayList.init(failing.allocator(), &failing_list), Vec3.zero()));
}
