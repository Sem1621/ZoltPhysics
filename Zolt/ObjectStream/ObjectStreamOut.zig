//! Port of: Jolt/ObjectStream/ObjectStreamOut.h, Jolt/ObjectStream/ObjectStreamOut.cpp
//! Status: complete
//!
//! - Jolt's two output classes (ObjectStreamTextOut, ObjectStreamBinaryOut) are a closed set of final classes: the
//!   format is a tagged union (guide section 6) and the virtual Write* functions switch over it.
//! - `sWriteObject(T, allocator, writer, stream_type, object)`: `allocator` holds the identifier map, the queues and the
//!   binary string table for the duration of the call; the caller flushes `writer`. Errors: `error.OutOfMemory` and
//!   the writer's `error.WriteFailed` (Jolt returns `!mStream.fail()`).
//! - `sWriteObjectToFile(T, allocator, io, path, stream_type, object)` creates or truncates the file and writes through
//!   a buffered `std.Io.File` writer, which it flushes. A file that cannot be created is an error of
//!   `std.Io.File.OpenError` (Jolt: false).
//! - Objects are identified by the address of their most derived object (Jolt: by the `T *` that is written, the same
//!   address for every class of Jolt, which has single inheritance at offset 0 only), so that an object reached through
//!   two different base classes is written once.
//! - The class set starts empty and `queueRTTI` skips primitives (Jolt fills the set with the primitives first): the
//!   primitives are never declared.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Writer = std.Io.Writer;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const ObjectStream = @import("ObjectStream.zig");
const StreamType = ObjectStream.StreamType;
const Identifier = ObjectStream.Identifier;
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const ObjectStreamTextOut = @import("ObjectStreamTextOut.zig").ObjectStreamTextOut;
const ObjectStreamBinaryOut = @import("ObjectStreamBinaryOut.zig").ObjectStreamBinaryOut;

/// ObjectStreamOut contains all logic for writing an object to disk. It is the base for the binary and text object
/// streams (ObjectStreamBinaryOut and ObjectStreamTextOut).
pub const ObjectStreamOut = struct {
    pub const Error = Allocator.Error || Writer.Error;

    /// The errors of the file variant: the errors of creating the file, then those of the write
    pub const FileError = std.Io.File.OpenError || Error;

    const ObjectInfo = struct {
        identifier: Identifier,
        rtti: *const RTTI,
    };

    allocator: Allocator,
    format: Format,
    /// Next free identifier for this stream
    next_identifier: Identifier = ObjectStream.null_identifier + 1,
    /// Links object pointer to an identifier
    identifier_map: std.AutoHashMapUnmanaged(*const anyopaque, ObjectInfo) = .empty,
    /// Queue of objects to be written
    object_queue: std.ArrayList(*const anyopaque) = .empty,
    /// List of classes already written
    class_set: std.AutoHashMapUnmanaged(*const RTTI, void) = .empty,
    /// List of classes waiting to be written
    class_queue: std.ArrayList(*const RTTI) = .empty,

    const Format = union(StreamType) {
        text: ObjectStreamTextOut,
        binary: ObjectStreamBinaryOut,
    };

    /// Main function to write an object to a stream (sWriteObject). `object` is written as its dynamic class.
    pub fn sWriteObject(comptime T: type, allocator: Allocator, writer: *Writer, stream_type: StreamType, object: *const T) Error!void {
        var stream = try open(allocator, writer, stream_type);
        defer stream.deinit();
        try stream.write(RTTI.mostDerived(T, object), RTTI.getRTTI(T, object));
    }

    /// Main function to write an object to a file (sWriteObject(const char *, ...)). `path` is absolute or relative to
    /// the current working directory; an existing file is truncated.
    pub fn sWriteObjectToFile(comptime T: type, allocator: Allocator, io: std.Io, path: []const u8, stream_type: StreamType, object: *const T) FileError!void {
        const file = try std.Io.Dir.cwd().createFile(io, path, .{});
        defer file.close(io);
        var buffer: [file_buffer_size]u8 = undefined;
        var file_writer = file.writer(io, &buffer);
        try sWriteObject(T, allocator, &file_writer.interface, stream_type, object);
        try file_writer.interface.flush();
    }

    /// The buffer of the file writer (on the stack)
    const file_buffer_size = 4096;

    /// Static constructor (Open): writes the header
    pub fn open(allocator: Allocator, writer: *Writer, stream_type: StreamType) Writer.Error!ObjectStreamOut {
        return .{ .allocator = allocator, .format = switch (stream_type) {
            .text => .{ .text = try .init(writer) },
            .binary => .{ .binary = try .init(writer) },
        } };
    }

    pub fn deinit(self: *ObjectStreamOut) void {
        switch (self.format) {
            .text => {},
            .binary => |*b| b.deinit(self.allocator),
        }
        self.class_queue.deinit(self.allocator);
        self.class_set.deinit(self.allocator);
        self.object_queue.deinit(self.allocator);
        self.identifier_map.deinit(self.allocator);
    }

    /// Write an object to the stream: `object` is the most derived object of class `rtti`. Writes everything it
    /// references too.
    pub fn write(self: *ObjectStreamOut, object: *const anyopaque, rtti: *const RTTI) Error!void {
        // Assign a new identifier to the object and write it (try_emplace: an object that already has an identifier
        // keeps it and its class, the counter still goes up)
        const entry = try self.identifier_map.getOrPut(self.allocator, object);
        if (!entry.found_existing) entry.value_ptr.* = .{ .identifier = self.next_identifier, .rtti = rtti };
        self.next_identifier += 1;
        try self.writeObject(object);

        // Write all linked objects (the queue grows while it is written)
        var cur: usize = 0;
        while (cur < self.object_queue.items.len) : (cur += 1)
            try self.writeObject(self.object_queue.items[cur]);
        self.object_queue.clearRetainingCapacity();
    }

    fn writeObject(self: *ObjectStreamOut, object: *const anyopaque) Error!void {
        // Find object identifier
        const info = self.identifier_map.get(object).?;

        // Write class description and associated descriptions
        try self.queueRTTI(info.rtti);
        var cur: usize = 0;
        while (cur < self.class_queue.items.len) : (cur += 1)
            try self.writeRTTI(self.class_queue.items[cur]);
        self.class_queue.clearRetainingCapacity();

        try self.hintNextItem();
        try self.hintNextItem();

        // Write object header
        try self.writeDataType(.object);
        try self.writeName(info.rtti.name);
        try self.writeIdentifier(info.identifier);

        // Write attribute data
        try self.writeClassData(info.rtti, object);
    }

    fn queueRTTI(self: *ObjectStreamOut, rtti: *const RTTI) Allocator.Error!void {
        if (rtti.kind == .primitive) return; // Primitives are never declared
        const entry = try self.class_set.getOrPut(self.allocator, rtti);
        if (!entry.found_existing) {
            errdefer _ = self.class_set.remove(rtti);
            try self.class_queue.append(self.allocator, rtti);
        }
    }

    fn writeRTTI(self: *ObjectStreamOut, rtti: *const RTTI) Error!void {
        try self.hintNextItem();
        try self.hintNextItem();

        // Write class header. E.g. in text mode: "declare <name> <attr-count>"
        try self.writeDataType(.declare);
        try self.writeName(rtti.name);
        try self.writeCount(@intCast(rtti.attributes.len));

        // Write class attribute info
        self.hintIndentUp();
        for (rtti.attributes) |*attribute| {
            // Write definition of attribute class if undefined
            if (attribute.getMemberPrimitiveType()) |t| try self.queueRTTI(t);

            try self.hintNextItem();

            // Write attribute information
            try self.writeName(attribute.name);
            try attribute.writeDataType(self);
        }
        self.hintIndentDown();
    }

    /// Write the attributes of `instance` (an object of class `rtti`)
    pub fn writeClassData(self: *ObjectStreamOut, rtti: *const RTTI, instance: *const anyopaque) Error!void {
        // Write attributes
        self.hintIndentUp();
        for (rtti.attributes) |*attribute| try attribute.writeData(self, instance);
        self.hintIndentDown();
    }

    /// Write a reference to `pointer` (the most derived object of class `rtti`, or null): its identifier. The object
    /// itself is queued the first time.
    pub fn writePointerData(self: *ObjectStreamOut, rtti: ?*const RTTI, pointer: ?*const anyopaque) Error!void {
        var identifier: Identifier = ObjectStream.null_identifier;
        if (pointer) |p| {
            // Check if this object has an identifier
            const entry = try self.identifier_map.getOrPut(self.allocator, p);
            if (entry.found_existing) {
                // Object already has an identifier
                identifier = entry.value_ptr.identifier;
            } else {
                // Assign a new identifier to this object and queue it for serialization
                errdefer _ = self.identifier_map.remove(p);
                try self.object_queue.append(self.allocator, p);
                identifier = self.next_identifier;
                self.next_identifier += 1;
                entry.value_ptr.* = .{ .identifier = identifier, .rtti = rtti.? };
            }
        }

        // Write the identifier
        try self.hintNextItem();
        try self.writeIdentifier(identifier);
    }

    // Output type specific operations (the virtual functions of IObjectStreamOut)

    pub fn writeDataType(self: *ObjectStreamOut, data_type: OSDataType) Error!void {
        switch (self.format) {
            inline else => |*f| try f.writeDataType(data_type),
        }
    }

    pub fn writeName(self: *ObjectStreamOut, name: []const u8) Error!void {
        switch (self.format) {
            .text => |*f| try f.writeName(name),
            .binary => |*f| try f.writeString(self.allocator, name),
        }
    }

    pub fn writeIdentifier(self: *ObjectStreamOut, identifier: Identifier) Error!void {
        switch (self.format) {
            inline else => |*f| try f.writeIdentifier(identifier),
        }
    }

    pub fn writeCount(self: *ObjectStreamOut, count: u32) Error!void {
        switch (self.format) {
            inline else => |*f| try f.writeCount(count),
        }
    }

    /// WritePrimitiveData of a primitive (the String overload is `writeString`)
    pub fn writePrimitive(self: *ObjectStreamOut, comptime T: type, value: T) Error!void {
        switch (self.format) {
            inline else => |*f| try f.writePrimitive(T, value),
        }
    }

    /// WritePrimitiveData(const String &)
    pub fn writeString(self: *ObjectStreamOut, value: []const u8) Error!void {
        switch (self.format) {
            .text => |*f| try f.writeString(value),
            .binary => |*f| try f.writeString(self.allocator, value),
        }
    }

    // Layout hints (for text output)

    pub fn hintNextItem(self: *ObjectStreamOut) Error!void {
        switch (self.format) {
            .text => |*f| try f.hintNextItem(),
            .binary => {},
        }
    }

    pub fn hintIndentUp(self: *ObjectStreamOut) void {
        switch (self.format) {
            .text => |*f| f.hintIndentUp(),
            .binary => {},
        }
    }

    pub fn hintIndentDown(self: *ObjectStreamOut) void {
        switch (self.format) {
            .text => |*f| f.hintIndentDown(),
            .binary => {},
        }
    }
};
