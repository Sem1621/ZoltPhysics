//! Port of: Jolt/Core/ObjectToIDMap.h
//! Status: complete
//!
//! The C++ type aliases live in namespace `StreamUtils`; Zolt re-exports them flat
//! (`zolt.ObjectToIDMap`, `zolt.IDToObjectMap`).

const std = @import("std");
const UnorderedMap = @import("UnorderedMap.zig").UnorderedMap;
const Ref = @import("Reference.zig").Ref;

/// Maps an object to the ID it was saved with (StreamUtils::ObjectToIDMap)
pub fn ObjectToIDMap(comptime Type: type) type {
    return UnorderedMap(*const Type, u32, .{});
}

/// Maps an ID back to the restored object (StreamUtils::IDToObjectMap)
pub fn IDToObjectMap(comptime Type: type) type {
    return std.ArrayList(Ref(Type));
}

test "ObjectToIDMap / IDToObjectMap" {
    const RefCount = @import("Reference.zig").RefCount;
    const Object = struct {
        const Self = @This();
        ref_count: RefCount = .{},
        allocator: std.mem.Allocator,

        pub fn addRef(self: *const Self) void {
            self.ref_count.addRef();
        }
        pub fn release(self: *const Self) void {
            if (self.ref_count.release())
                self.allocator.destroy(self);
        }
    };

    const allocator = std.testing.allocator;

    // Restore: create objects and store them by ID
    var id_to_object: IDToObjectMap(Object) = .empty;
    defer {
        for (id_to_object.items) |*ref| ref.deinit();
        id_to_object.deinit(allocator);
    }
    for (0..4) |_| {
        const object = try allocator.create(Object);
        object.* = .{ .allocator = allocator };
        try id_to_object.append(allocator, Ref(Object).init(object));
    }

    // Save: assign IDs to objects
    var object_to_id: ObjectToIDMap(Object) = .empty;
    defer object_to_id.deinit(allocator);
    for (id_to_object.items, 0..) |ref, id|
        try std.testing.expect((try object_to_id.tryEmplace(allocator, ref.get().?, @intCast(id))).inserted);
    try std.testing.expectEqual(4, object_to_id.count());
    for (id_to_object.items, 0..) |ref, id|
        try std.testing.expectEqual(@as(u32, @intCast(id)), object_to_id.find(ref.get().?).?.value);
}
