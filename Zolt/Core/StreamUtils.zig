//! Port of: Jolt/Core/StreamUtils.h
//! Status: complete
//!
//! - `RestoreObject` finds the class of the hash in `RegisterTypes.factory` (Jolt: `Factory::sInstance`). The class must
//!   be creatable and derive from `Type`. Otherwise the result is Jolt's error "Failed to create instance of type"
//!   (Jolt is undefined for such a hash, SerializationArchitecture.md D8).
//! - Restoring and saving allocate, so the functions return `Allocator.Error`. Jolt's errors stay values of the Result.
//! - `RestoreObjectArray` appends the objects to an empty list (Jolt returns `Result<ArrayType>`). The result holds
//!   Jolt's error text on failure.
//! - `ObjectToIDMap` and `IDToObjectMap` are in Core/ObjectToIDMap.zig, like Jolt's.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("Core.zig");
const RTTI = @import("RTTI.zig").RTTI;
const ReferenceFile = @import("Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const Result = @import("Result.zig").Result;
const StreamIn = @import("StreamIn.zig").StreamIn;
const StreamOut = @import("StreamOut.zig").StreamOut;
const ObjectToIDMapFile = @import("ObjectToIDMap.zig");
const ObjectToIDMap = ObjectToIDMapFile.ObjectToIDMap;
const IDToObjectMap = ObjectToIDMapFile.IDToObjectMap;
const RegisterTypes = @import("../RegisterTypes.zig");

/// Restore a single object by reading the hash of the type, constructing it and then calling the restore function
pub fn restoreObject(comptime Type: type, allocator: Allocator, stream: StreamIn, comptime restoreBinaryStateFunction: fn (*Type, StreamIn) Allocator.Error!void) Allocator.Error!Result(Ref(Type)) {
    var result: Result(Ref(Type)) = .empty;

    // Read the hash of the type
    var hash: u32 = 0;
    stream.read(&hash);
    if (stream.isEOF() or stream.isFailed()) {
        result.setError("Failed to read type hash");
        return result;
    }

    // Get the RTTI for the type (Zolt: the class must be creatable and a Type, D8)
    const rtti = RegisterTypes.factory.findHash(hash) orelse {
        result.setError("Failed to create instance of type");
        return result;
    };
    if (rtti.isAbstract() or !rtti.isKindOf(RTTI.of(Type))) {
        result.setError("Failed to create instance of type");
        return result;
    }

    // Construct and read the data of the type
    const created = (try rtti.createObject(allocator)).?;
    const object: *Type = @ptrCast(@alignCast(rtti.castTo(created, RTTI.of(Type)).?));
    var ref = Ref(Type).init(object);
    defer ref.deinit();
    try restoreBinaryStateFunction(object, stream);
    if (stream.isEOF() or stream.isFailed()) {
        result.setError("Failed to restore object");
        return result;
    }

    result.set(ref.clone());
    return result;
}

/// Save an object reference to a stream. Uses a map to map objects to IDs which is also used to prevent writing duplicates.
pub fn saveObjectReference(comptime Type: type, allocator: Allocator, stream: StreamOut, object: ?*const Type, object_to_id_map: ?*ObjectToIDMap(Type)) Allocator.Error!void {
    if (object_to_id_map == null or object == null) {
        // Write null ID
        stream.write(~@as(u32, 0));
    } else if (object_to_id_map.?.find(object.?)) |id| {
        // Existing object, write ID
        stream.write(id.value);
    } else {
        // New object, write the ID
        const new_id: u32 = object_to_id_map.?.count();
        (try object_to_id_map.?.getOrPutValue(allocator, object.?, 0)).* = new_id;
        stream.write(new_id);

        // Write the object
        object.?.saveBinaryState(stream);
    }
}

/// Restore an object reference from stream.
pub fn restoreObjectReference(comptime Type: type, allocator: Allocator, stream: StreamIn, id_to_object_map: *IDToObjectMap(Type)) Allocator.Error!Result(Ref(Type)) {
    var result: Result(Ref(Type)) = .empty;
    errdefer result.deinit();

    // Read id
    var id: u32 = ~@as(u32, 0);
    stream.read(&id);

    // Check null
    if (id == ~@as(u32, 0)) {
        result.set(.empty);
        return result;
    }

    // Check if it already exists
    if (id >= id_to_object_map.items.len) {
        // New object, restore it
        result.assignMove(try Type.restoreFromBinaryState(allocator, stream));
        if (result.hasError())
            return result;
        if (Core.enable_asserts) std.debug.assert(id == id_to_object_map.items.len); // A corrupt stream can violate this
        try id_to_object_map.ensureUnusedCapacity(allocator, 1);
        id_to_object_map.appendAssumeCapacity(result.get().clone());
    } else {
        // Existing object filter
        result.set(id_to_object_map.items[id].clone());
    }

    return result;
}

/// Save an array of objects to a stream.
pub fn saveObjectArray(comptime Type: type, allocator: Allocator, stream: StreamOut, array: []const RefConst(Type), object_to_id_map: ?*ObjectToIDMap(Type)) Allocator.Error!void {
    const len: u32 = @intCast(array.len);
    stream.write(len);
    for (array) |value|
        try saveObjectReference(Type, allocator, stream, value.get(), object_to_id_map);
}

/// Restore an array of objects from a stream. The objects are appended to the empty list `out_values` (Jolt returns
/// Result<ArrayType>). The returned result holds Jolt's error text on failure.
pub fn restoreObjectArray(comptime Type: type, allocator: Allocator, stream: StreamIn, id_to_object_map: *IDToObjectMap(Type), out_values: *std.ArrayList(RefConst(Type))) Allocator.Error!Result(void) {
    var result: Result(void) = .empty;

    var len: u32 = 0;
    stream.read(&len);
    if (stream.isEOF() or stream.isFailed()) {
        result.setError("Failed to read stream");
        return result;
    }

    try out_values.ensureTotalCapacity(allocator, len);
    for (0..len) |_| {
        var value = try restoreObjectReference(Type, allocator, stream, id_to_object_map);
        defer value.deinit();
        if (value.hasError()) {
            result.setError(value.getError());
            return result;
        }
        try out_values.append(allocator, .init(value.getPtr()));
    }

    result.set({});
    return result;
}

const StreamInWrapper = @import("StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("StreamWrapper.zig").StreamOutWrapper;
const PhysicsMaterial = @import("../Physics/Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("../Physics/Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const GroupFilter = @import("../Physics/Collision/GroupFilter.zig").GroupFilter;
const GroupFilterTable = @import("../Physics/Collision/GroupFilterTable.zig").GroupFilterTable;
const Color = @import("Color.zig").Color;

fn restoreMaterial(allocator: Allocator, bytes: []const u8) Allocator.Error!Result(Ref(PhysicsMaterial)) {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return restoreObject(PhysicsMaterial, allocator, in.streamIn(), PhysicsMaterial.restoreBinaryState);
}

test "StreamUtils: restoreObject finds the class in the factory" {
    const allocator = std.testing.allocator;

    // A registered class that derives from the type
    const material = try PhysicsMaterialSimple.create(allocator, "Ice", Color.cyan);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    var buffer: [64]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    material.material().saveBinaryState(out.streamOut());
    var restored = try restoreMaterial(allocator, writer.buffered());
    defer restored.deinit();
    try std.testing.expect(restored.isValid());
    try std.testing.expect(restored.getPtr().?.vtable.rtti == PhysicsMaterialSimple.rtti);
    try std.testing.expectEqualStrings("Ice", restored.getPtr().?.getDebugName());

    // An unknown hash. Zolt (D8): a registered class of another hierarchy and an abstract class are not created.
    for ([_]u32{ 0x01020304, GroupFilterTable.rtti.hash, GroupFilter.rtti.hash }) |hash| {
        var r = try restoreMaterial(allocator, std.mem.asBytes(&hash));
        defer r.deinit();
        try std.testing.expectEqualStrings("Failed to create instance of type", r.getError());
    }
    try std.testing.expect(RegisterTypes.factory.findHash(GroupFilterTable.rtti.hash) == GroupFilterTable.rtti);

    // A truncated object: created, restored, destroyed again (leak checked)
    var truncated = try restoreMaterial(allocator, writer.buffered()[0..6]);
    defer truncated.deinit();
    try std.testing.expectEqualStrings("Failed to restore object", truncated.getError());

    // Every allocation can fail
    try std.testing.checkAllAllocationFailures(allocator, struct {
        fn f(a: Allocator, bytes: []const u8) !void {
            var r = try restoreMaterial(a, bytes);
            r.deinit();
        }
    }.f, .{writer.buffered()});
}
