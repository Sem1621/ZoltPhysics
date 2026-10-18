//! Port of: Jolt/Core/Factory.h, Jolt/Core/Factory.cpp
//! Status: partial
//! Missing: `RegisterTypes.factory` (Jolt's RegisterTypes() list of 49 types plus `zolt_user_types.serializable_types`)
//! and the binary state restores through it (Phase 8 foundation, Docs/Zolt/SerializationArchitecture.md S8 and S9).
//!
//! Jolt's Factory is a global singleton (`Factory::sInstance`) filled at startup by RegisterTypes() and looked up by
//! name (ObjectStreamIn) and by hash (StreamUtils::RestoreObject). Zolt's Factory is a value built at compile time,
//! `Factory.init(&.{ T, ... })`, with Jolt's registration: every listed class, then its bases, then the classes of its
//! attributes, recursively, each once, in Jolt's pre-order (`classes`). Two classes with the same name or the same hash
//! are a compile error (Jolt: "Hash collision registering type!"). There is no global, no Register at run time and no
//! Clear: ObjectStreamIn takes the factory as an option, so a test or an application can read with another one
//! (`Factory.init(RegisterTypes.factory_types ++ &[_]type{MyClass})`, Jolt's `Factory::sInstance->Register(...)`).
//!
//! GetAllClasses returns the registration order; Jolt's returns its hash map order (nothing depends on it).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("RTTI.zig").RTTI;

/// This class is responsible for creating instances of classes based on their name or hash and is mainly used for
/// deserialization of saved data.
pub const Factory = struct {
    /// All registered classes in Jolt's registration order (GetAllClasses)
    classes: []const *const RTTI,
    /// The same classes sorted by hash (Find)
    by_hash: []const *const RTTI,

    /// Register the classes `types` (and recursively their bases and attribute types) (Factory::Register). Must be
    /// evaluated at compile time.
    pub fn init(comptime types: []const type) Factory {
        comptime {
            @setEvalBranchQuota(1_000_000);
            var classes: []const *const RTTI = &.{};
            for (types) |T| classes = register(classes, RTTI.of(T));

            // Unique names and hashes
            for (classes, 0..) |a, i| {
                for (classes[0..i]) |b| {
                    if (std.mem.eql(u8, a.name, b.name))
                        @compileError("two different classes are named " ++ a.name);
                    if (a.hash == b.hash)
                        @compileError("Hash collision registering type " ++ a.name ++ " (same hash as " ++ b.name ++ ")");
                }
            }

            // Sort by hash (insertion sort, at most a few hundred classes)
            var sorted: [classes.len]*const RTTI = classes[0..classes.len].*;
            for (1..sorted.len) |i| {
                var j = i;
                while (j > 0 and sorted[j - 1].hash > sorted[j].hash) : (j -= 1) {
                    const t = sorted[j];
                    sorted[j] = sorted[j - 1];
                    sorted[j - 1] = t;
                }
            }
            const final_classes = classes[0..classes.len].*;
            const final_sorted = sorted;
            return .{ .classes = &final_classes, .by_hash = &final_sorted };
        }
    }

    /// Jolt's Register: the class first, then its bases, then the classes of its attributes (pre-order), each once
    fn register(comptime classes: []const *const RTTI, comptime rtti: *const RTTI) []const *const RTTI {
        // Check if we already know the type
        for (classes) |c| {
            if (c == rtti) return classes;
        }

        // Insert this class by name and by hash
        var result: []const *const RTTI = classes ++ &[_]*const RTTI{rtti};

        // Register base classes
        for (rtti.bases) |base| result = register(result, base.rtti);

        // Register attribute classes
        for (rtti.attributes) |attribute| {
            if (attribute.getMemberPrimitiveType()) |t| result = register(result, t);
        }
        return result;
    }

    /// Find type info for a specific class by hash, null if not found
    pub fn findHash(self: *const Factory, hash: u32) ?*const RTTI {
        var low: usize = 0;
        var high: usize = self.by_hash.len;
        while (low < high) {
            const mid = low + (high - low) / 2;
            const h = self.by_hash[mid].hash;
            if (h == hash) return self.by_hash[mid];
            if (h < hash) low = mid + 1 else high = mid;
        }
        return null;
    }

    /// Find type info for a specific class by name, null if not found
    pub fn find(self: *const Factory, name: []const u8) ?*const RTTI {
        // The hashes of a factory are unique: the class with the hash of the name, if it has the name
        const rtti = self.findHash(RTTI.hashName(name)) orelse return null;
        return if (std.mem.eql(u8, rtti.name, name)) rtti else null;
    }

    /// Create an object of the class `name` (CreateObject), null if the class is unknown or abstract
    pub fn createObject(self: *const Factory, allocator: Allocator, name: []const u8) Allocator.Error!?*anyopaque {
        const rtti = self.find(name) orelse return null;
        return rtti.createObject(allocator);
    }

    /// Get all registered classes
    pub fn getAllClasses(self: *const Factory) []const *const RTTI {
        return self.classes;
    }
};

test "Factory: Jolt's registration order, find by name and hash" {
    const Color = @import("Color.zig").Color;
    const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
    const factory: Factory = comptime .init(&.{ IndexedTriangle, Color });
    const names = [_][]const u8{ "IndexedTriangle", "IndexedTriangleNoMaterial", "uint32", "Color", "uint8" };
    try std.testing.expectEqual(names.len, factory.getAllClasses().len);
    for (names, factory.getAllClasses()) |n, c| try std.testing.expectEqualStrings(n, c.name);
    try std.testing.expectEqual(RTTI.of(Color), factory.find("Color").?);
    try std.testing.expectEqual(RTTI.of(Color), factory.findHash(0xa7d2746c).?);
    try std.testing.expect(factory.find("Colour") == null and factory.find("float") == null);
}
