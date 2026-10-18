//! Port of: Jolt/ObjectStream/ObjectStreamIn.h, Jolt/ObjectStream/ObjectStreamIn.cpp
//! Status: partial
//! Missing: `sReadObject` from a file (`std.Io`), and `RegisterTypes.factory` as the default factory (Phase 8
//! foundation, Docs/Zolt/SerializationArchitecture.md).
//!
//! `sReadObject(T, allocator, reader, .{ .factory })` reads the first object of the stream and everything it
//! references.
//! - Ownership: every object and all memory it owns are allocated with `allocator` (objects that keep an allocator,
//!   the RefTargets, keep this one). The result has the reference count that the stream's links gave it (0 unless an
//!   object of the stream references it): `sReadObjectRef` wraps a RefTarget in a Ref, a value class is freed with
//!   `RTTI.of(T).destructObject(p, allocator)`. Objects that nothing references are destroyed, as in Jolt.
//! - Errors: error.OutOfMemory, error.ReadFailed (the reader failed) and error.InvalidObjectStream (every failure
//!   that makes Jolt's sReadObject return false, with Jolt's Trace text logged at info level). On an error everything
//!   the read created is destroyed.
//! - Jolt's reading semantics are kept for every stream (also partial reads: an object whose data cannot be read is
//!   dropped and the read goes on; an unknown word at the top level of a text stream ends the read). Jolt's quirks are
//!   reproduced: a null identifier leaves the member as it is, a second `declare` of a class appends its attributes,
//!   a Ref attribute that must be skipped is skipped as class data (it misreads the stream), an object of an unknown
//!   class is not skipped.
//!
//! Zolt's defined behavior where Jolt has undefined behavior or leaks (SerializationArchitecture.md, S16):
//! - D1: the links into an object whose data cannot be read are dropped with it (Jolt writes into the freed object);
//! - D2: an object with an identifier that is already used is destroyed with its links (Jolt leaks it);
//! - D4: the main object is kept alive while the unreferenced objects are destroyed (Jolt deletes a main object that
//!   only an unreferenced object references and returns a dangling pointer);
//! - D11: two stream attributes that map to the same attribute of a class (a repeated declaration) are an invalid
//!   stream (Jolt reads the member twice: a reallocated array leaves dangling links, a Ref leaks);
//! - D12: references (Ref / RefConst) that form a cycle are an invalid stream (Jolt reads the objects and leaks them).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Reader = std.Io.Reader;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const Factory = @import("../Core/Factory.zig").Factory;
const Ref = @import("../Core/Reference.zig").Ref;
const ObjectStream = @import("ObjectStream.zig");
const StreamType = ObjectStream.StreamType;
const Identifier = ObjectStream.Identifier;
const OSDataType = @import("SerializableAttribute.zig").OSDataType;
const PrimitiveType = @import("ObjectStreamTypes.zig").PrimitiveType;
const ObjectStreamTextIn = @import("ObjectStreamTextIn.zig").ObjectStreamTextIn;
const ObjectStreamBinaryIn = @import("ObjectStreamBinaryIn.zig").ObjectStreamBinaryIn;

const log = std.log.scoped(.zolt);

/// ObjectStreamIn contains all logic for reading an object from disk. It is the base for the binary and text object
/// streams (ObjectStreamBinaryIn and ObjectStreamTextIn).
pub const ObjectStreamIn = struct {
    pub const Error = error{ OutOfMemory, ReadFailed, InvalidObjectStream };

    /// Link resolution: make the member point to `object` (of class `object_rtti`), or to nothing
    pub const StoreFn = *const fn (member: *anyopaque, object: ?*anyopaque, object_rtti: *const RTTI) void;

    pub const Options = struct {
        /// The classes that the stream may declare (Jolt: Factory::sInstance). Phase 8 foundation: defaults to
        /// `&RegisterTypes.factory`.
        factory: *const Factory,
    };

    /// Description of an attribute in the stream
    const AttributeDescription = struct {
        array_depth: u32 = 0,
        source_type: OSDataType = .invalid,
        destination_type: OSDataType = .invalid,
        /// Class of an `instance` / `pointer` attribute (owned)
        class_name: []u8 = &.{},
        /// Index in the RTTI's attributes, null when the data is skipped (Jolt: -1)
        index: ?usize = null,
    };

    /// Description of a class in the stream
    const ClassDescription = struct {
        rtti: ?*const RTTI,
        attributes: std.ArrayList(AttributeDescription) = .empty,
    };

    const ObjectInfo = struct {
        instance: *anyopaque,
        rtti: *const RTTI,
        identifier: Identifier,
        /// A link points to this object (Jolt's set of referenced objects)
        referenced: bool = false,
    };

    const Link = struct {
        member: *anyopaque,
        identifier: Identifier,
        rtti: *const RTTI,
        store: StoreFn,
        /// The object whose data contains the member
        owner: Identifier,
        /// A Ref / RefConst (adds a reference), not a raw pointer
        counted: bool,
    };

    allocator: Allocator,
    factory: *const Factory,
    format: Format,
    /// Class descriptions by name (keys owned)
    class_descriptions: std.StringHashMapUnmanaged(ClassDescription) = .empty,
    /// The objects that were read, in stream order
    objects: std.ArrayList(ObjectInfo) = .empty,
    /// Links identifier to an object (an index in `objects`)
    identifier_map: std.AutoHashMapUnmanaged(Identifier, u32) = .empty,
    /// All pointers (links) are resolved after reading the entire file, e.g. when all object exist
    unresolved_links: std.ArrayList(Link) = .empty,
    /// The identifier of the object that is being read (the owner of the links)
    current_object: Identifier = ObjectStream.null_identifier,

    const Format = union(StreamType) {
        text: ObjectStreamTextIn,
        binary: ObjectStreamBinaryIn,
    };

    /// Main function to read an object from a stream (sReadObject(istream &, T *&)): the first object of the stream,
    /// which must be a T, with the reference count that the stream's links gave it (usually 0)
    pub fn sReadObject(comptime T: type, allocator: Allocator, reader: *Reader, options: Options) Error!*T {
        var stream = try open(allocator, reader, options.factory);
        defer stream.deinit();
        const main = try stream.read(RTTI.of(T));
        return @ptrCast(@alignCast(main.rtti.castTo(main.instance, RTTI.of(T)).?));
    }

    /// sReadObject(istream &, Ref<T> &): the main object wrapped in a Ref (one more reference)
    pub fn sReadObjectRef(comptime T: type, allocator: Allocator, reader: *Reader, options: Options) Error!Ref(T) {
        return .init(try sReadObject(T, allocator, reader, options));
    }

    /// Check the header ("TOS 1.00" / "BOS 1.00", GetInfo) and create a stream of that type (Open)
    pub fn open(allocator: Allocator, reader: *Reader, factory: *const Factory) Error!ObjectStreamIn {
        // Read header and check if it is the correct format, e.g. "TOS 1.00"
        var header: [8]u8 = @splat(0);
        const n = reader.readSliceShort(&header) catch return error.ReadFailed;
        const valid = n == header.len and (header[0] == 'B' or header[0] == 'T') and header[1] == 'O' and header[2] == 'S' and
            (header[3] == ' ' or std.ascii.isDigit(header[3])) and std.ascii.isDigit(header[4]) and
            header[5] == '.' and std.ascii.isDigit(header[6]) and std.ascii.isDigit(header[7]);
        if (!valid) {
            log.info("ObjectStreamIn: Not a valid object stream.", .{});
            return error.InvalidObjectStream;
        }

        // Extract version and revision (atoi)
        const stream_version = std.fmt.parseInt(u32, std.mem.trimStart(u8, header[3..5], " "), 10) catch unreachable;
        const stream_revision = std.fmt.parseInt(u32, header[6..8], 10) catch unreachable;
        if (stream_version != ObjectStream.version or stream_revision != ObjectStream.revision) {
            log.info("ObjectStreamIn: Different version stream ({d}.{d:0>2}, expected {d}.{d:0>2}).", .{ stream_version, stream_revision, ObjectStream.version, ObjectStream.revision });
            return error.InvalidObjectStream;
        }

        // Create an input stream of the correct type
        return .{ .allocator = allocator, .factory = factory, .format = if (header[0] == 'T') .{ .text = .{ .reader = reader } } else .{ .binary = .{ .reader = reader } } };
    }

    pub fn deinit(self: *ObjectStreamIn) void {
        self.destroyObjects(); // Only the objects of a read that failed are left
        self.objects.deinit(self.allocator);
        self.identifier_map.deinit(self.allocator);
        self.unresolved_links.deinit(self.allocator);
        var it = self.class_descriptions.iterator();
        while (it.next()) |entry| {
            for (entry.value_ptr.attributes.items) |a| self.allocator.free(a.class_name);
            entry.value_ptr.attributes.deinit(self.allocator);
            self.allocator.free(entry.key_ptr.*);
        }
        self.class_descriptions.deinit(self.allocator);
        switch (self.format) {
            inline else => |*f| f.deinit(self.allocator),
        }
    }

    /// Release all objects (after a fatal error no link is resolved, so no object references another)
    fn destroyObjects(self: *ObjectStreamIn) void {
        for (self.objects.items) |info| info.rtti.destructObject(info.instance, self.allocator);
        self.objects.clearRetainingCapacity();
        self.identifier_map.clearRetainingCapacity();
    }

    pub const MainObject = struct { instance: *anyopaque, rtti: *const RTTI };

    /// Read all objects of the stream, resolve the links and return the main object (the first object, which must be
    /// kind of `rtti`). Ownership of every object passes to the main object, the links and the caller.
    pub fn read(self: *ObjectStreamIn, rtti: *const RTTI) Error!MainObject {
        // Read all information on the stream
        var main_object: ?MainObject = null;
        errdefer self.destroyObjects(); // Release all objects if a fatal error occurred
        while (true) {
            // Get type of next operation (the read ends when there is none: the end of the stream, an unknown word)
            const data_type = self.readDataType() catch |err| switch (err) {
                error.InvalidObjectStream => break,
                else => |e| return e,
            };
            switch (data_type) {
                .declare => self.readRTTI() catch |err| {
                    // Read type declaration
                    if (err == error.InvalidObjectStream)
                        log.info("ObjectStreamIn: Fatal error while reading class description for class {s}.", .{rtti.name});
                    return err;
                },
                .object => if (try self.readObject()) |object| {
                    if (main_object == null) {
                        // This is the first and thus main object of the file
                        if (!object.rtti.isKindOf(rtti)) {
                            log.info("ObjectStreamIn: Main object of different type. Expected {s}, but found {s}.", .{ rtti.name, object.rtti.name });
                            return error.InvalidObjectStream;
                        }
                        main_object = object;
                    }
                },
                else => {
                    // Invalid or out of place token found
                    log.info("ObjectStreamIn: Invalid or out of place token found.", .{});
                    return error.InvalidObjectStream;
                },
            }
        }
        const main = main_object orelse return error.InvalidObjectStream; // No object could be created

        // Zolt: references that form a cycle would keep their objects alive forever (D12)
        if (try self.hasReferenceCycle()) {
            log.info("ObjectStreamIn: The references between the objects form a cycle.", .{});
            return error.InvalidObjectStream;
        }

        // Resolve links (pointer, references). Nothing below can fail.
        for (self.unresolved_links.items) |link| {
            if (self.identifier_map.get(link.identifier)) |index| {
                const info = &self.objects.items[index];
                if (info.rtti.isKindOf(link.rtti)) {
                    // Set pointer (and increment the reference count if it is a Ref)
                    link.store(link.member, info.instance, info.rtti);

                    // Add referenced object to the list
                    info.referenced = true;
                    continue;
                }
            }

            // Referenced object not found, set pointer to nullptr
            log.info("ObjectStreamIn: Setting incorrect pointer to class of type {s} to nullptr.", .{link.rtti.name});
            link.store(link.member, null, link.rtti);
        }
        self.unresolved_links.clearRetainingCapacity();

        // Release unreferenced objects except the main object. A temporary reference keeps the main object alive while
        // an unreferenced object that references it is destroyed (D4); it is dropped without deleting the object.
        const main_ref_count = if (main.rtti.ref_count) |f| f(main.instance) else null;
        if (main_ref_count) |r| r.addRef();
        for (self.objects.items) |info| {
            if (info.instance != main.instance and !info.referenced) {
                log.info("ObjectStreamIn: Releasing unreferenced object of type {s}.", .{info.rtti.name});
                info.rtti.destructObject(info.instance, self.allocator);
            }
        }
        if (main_ref_count) |r| _ = r.release();
        self.objects.clearRetainingCapacity();
        self.identifier_map.clearRetainingCapacity();
        return main;
    }

    /// True when the Ref / RefConst links that will resolve form a cycle (depth first search over the objects)
    fn hasReferenceCycle(self: *ObjectStreamIn) Allocator.Error!bool {
        const Edge = struct {
            owner: u32,
            target: u32,
            fn less(_: void, a: @This(), b: @This()) bool {
                return a.owner < b.owner;
            }
        };

        // The edges owner -> target of the counted links that resolve, grouped by owner (indices in `objects`)
        var edges: std.ArrayList(Edge) = .empty;
        defer edges.deinit(self.allocator);
        for (self.unresolved_links.items) |link| {
            if (!link.counted) continue;
            const target = self.identifier_map.get(link.identifier) orelse continue;
            if (!self.objects.items[target].rtti.isKindOf(link.rtti)) continue;
            const owner = self.identifier_map.get(link.owner) orelse continue;
            try edges.append(self.allocator, .{ .owner = owner, .target = target });
        }
        if (edges.items.len == 0) return false;
        std.mem.sort(Edge, edges.items, {}, Edge.less);

        const Node = struct { state: enum { new, open, done } = .new, first_edge: u32 = 0, edge_count: u32 = 0 };
        const nodes = try self.allocator.alloc(Node, self.objects.items.len);
        defer self.allocator.free(nodes);
        @memset(nodes, .{});
        for (edges.items, 0..) |e, i| {
            if (nodes[e.owner].edge_count == 0) nodes[e.owner].first_edge = @intCast(i);
            nodes[e.owner].edge_count += 1;
        }

        // Iterative depth first search, an edge to an open node is a cycle
        const Frame = struct { node: u32, next_edge: u32 };
        var stack: std.ArrayList(Frame) = .empty;
        defer stack.deinit(self.allocator);
        for (nodes, 0..) |*root, root_index| {
            if (root.state != .new) continue;
            root.state = .open;
            try stack.append(self.allocator, .{ .node = @intCast(root_index), .next_edge = 0 });
            while (stack.items.len > 0) {
                const frame = &stack.items[stack.items.len - 1];
                const node = &nodes[frame.node];
                if (frame.next_edge == node.edge_count) {
                    node.state = .done;
                    _ = stack.pop();
                    continue;
                }
                const target = edges.items[node.first_edge + frame.next_edge].target;
                frame.next_edge += 1;
                switch (nodes[target].state) {
                    .open => return true,
                    .done => {},
                    .new => {
                        nodes[target].state = .open;
                        try stack.append(self.allocator, .{ .node = target, .next_edge = 0 });
                    },
                }
            }
        }
        return false;
    }

    /// Read an object (ReadObject): null when it was skipped or could not be read (not fatal, as in Jolt)
    fn readObject(self: *ObjectStreamIn) Error!?MainObject {
        // Read the object class
        const class_name = self.readName() catch |err| return softFailure(err);

        // Get class description
        const class_entry = self.class_descriptions.getEntry(class_name) orelse {
            // Jolt does not read the identifier: the next word is not a data type and the read ends
            log.info("ObjectStreamIn: Found object of unknown class {s}.", .{class_name});
            return null;
        };
        const class_desc = class_entry.value_ptr;
        const name = class_entry.key_ptr.*; // The text stream reuses the buffer of class_name

        // Read object identifier
        const identifier = self.readIdentifier() catch |err| return softFailure(err);

        // Check if this object can be read or must be skipped
        const rtti = class_desc.rtti orelse return self.skipObject(name, class_desc);
        if (identifier == ObjectStream.null_identifier or rtti.isAbstract()) return self.skipObject(name, class_desc);

        // Create object instance
        const object = (try rtti.createObject(self.allocator)).?;
        const link_count = self.unresolved_links.items.len;
        self.current_object = identifier;
        errdefer {
            self.unresolved_links.shrinkRetainingCapacity(link_count);
            rtti.destructObject(object, self.allocator);
        }

        // Read object attributes. Fatal error while reading attributes: release the object and drop its links (D1).
        self.readDescribedClassData(class_desc, object) catch |err| {
            if (err != error.InvalidObjectStream) return err;
            self.unresolved_links.shrinkRetainingCapacity(link_count);
            rtti.destructObject(object, self.allocator);
            return null;
        };

        // Add object to identifier map. An object with an identifier that is already used is destroyed (D2).
        const entry = try self.identifier_map.getOrPut(self.allocator, identifier);
        if (entry.found_existing) {
            self.unresolved_links.shrinkRetainingCapacity(link_count);
            rtti.destructObject(object, self.allocator);
            return null;
        }
        self.objects.append(self.allocator, .{ .instance = object, .rtti = rtti, .identifier = identifier }) catch |err| {
            self.identifier_map.removeByPtr(entry.key_ptr);
            return err;
        };
        entry.value_ptr.* = @intCast(self.objects.items.len - 1);
        return .{ .instance = object, .rtti = rtti };
    }

    fn skipObject(self: *ObjectStreamIn, class_name: []const u8, class_desc: *const ClassDescription) Error!?MainObject {
        // Skip this object (Jolt: "This operation can fail, but there is no check yet")
        log.info("ObjectStreamIn: Found uncreatable object {s}.", .{class_name});
        self.readDescribedClassData(class_desc, null) catch |err| return softFailure(err);
        return null;
    }

    /// Read a class description (ReadRTTI)
    fn readRTTI(self: *ObjectStreamIn) Error!void {
        // Read class name and find it's attribute info
        const name = try self.allocator.dupe(u8, try self.readName());

        // Find class
        const rtti = self.factory.find(name);
        if (rtti == null) log.info("ObjectStreamIn: Unknown class: \"{s}\".", .{name});

        // Insert class description (try_emplace: a second declaration of a class appends its attributes to the first)
        const entry = self.class_descriptions.getOrPut(self.allocator, name) catch |err| {
            self.allocator.free(name);
            return err;
        };
        if (entry.found_existing) self.allocator.free(name) else entry.value_ptr.* = .{ .rtti = rtti };
        const class_desc = entry.value_ptr;

        // Read the number of entries in the description
        const count = try self.readCount();

        // Read the entries
        for (0..count) |_| {
            var attribute: AttributeDescription = .{};

            // Read name (copied: the reads below reuse the text stream's word buffer)
            const attribute_name = try self.allocator.dupe(u8, try self.readName());
            defer self.allocator.free(attribute_name);

            // Read type
            attribute.source_type = try self.readDataType();

            // Read array depth
            while (attribute.source_type == .array) {
                attribute.array_depth += 1;
                attribute.source_type = try self.readDataType();
            }

            // Read instance/pointer class name
            if (attribute.source_type == .instance or attribute.source_type == .pointer)
                attribute.class_name = try self.allocator.dupe(u8, try self.readName());
            errdefer self.allocator.free(attribute.class_name);

            // Find attribute in rtti
            if (class_desc.rtti) |r| {
                // Find attribute index
                attribute.index = for (r.attributes, 0..) |a, idx| {
                    if (std.mem.eql(u8, attribute_name, a.name)) break idx;
                } else null;

                // Check if attribute is of expected type
                if (attribute.index) |idx| {
                    const a = &r.attributes[idx];
                    if (a.isType(attribute.array_depth, attribute.source_type, attribute.class_name)) {
                        // No conversion needed
                        attribute.destination_type = attribute.source_type;
                    } else if (attribute.array_depth == 0 and attribute.class_name.len == 0) {
                        // Try to apply type conversions
                        if (attribute.source_type == .t_vec3 and a.isType(0, .t_dvec3, ""))
                            attribute.destination_type = .t_dvec3
                        else if (attribute.source_type == .t_dvec3 and a.isType(0, .t_vec3, ""))
                            attribute.destination_type = .t_vec3
                        else
                            attribute.index = null;
                    } else {
                        // No conversion exists
                        attribute.index = null;
                    }
                }

                // Zolt: two stream attributes for one attribute of the class (D11)
                if (attribute.index) |idx| {
                    for (class_desc.attributes.items) |other| {
                        if (other.index == idx) {
                            log.info("ObjectStreamIn: Attribute {s} of class {s} is declared twice.", .{ r.attributes[idx].name, r.name });
                            return error.InvalidObjectStream;
                        }
                    }
                }
            }

            // Add attribute to the class description
            try class_desc.attributes.append(self.allocator, attribute);
        }
    }

    /// Read the attributes of an object or by value instance (null: skip them), in the order of the stream's
    /// description (ReadClassData(const ClassDescription &, void *))
    fn readDescribedClassData(self: *ObjectStreamIn, class_desc: *const ClassDescription, instance: ?*anyopaque) Error!void {
        for (class_desc.attributes.items) |attr_desc| {
            // Read or skip the attribute data
            if (attr_desc.index) |idx| {
                if (instance) |object| {
                    // Read the data, converting Vec3 <-> DVec3 when the stream was written with the other precision
                    try class_desc.rtti.?.attributes[idx].readData(self, object, attr_desc.source_type);
                    continue;
                }
            }
            try self.skipAttributeData(attr_desc.array_depth, attr_desc.source_type, attr_desc.class_name);
        }
    }

    /// OSReadData of a class by value (ReadClassData(const char *, void *)): the attributes of `instance`, an object of
    /// class `rtti`, as the stream describes the class (skipped when the stream's class is not this RTTI, e.g. a class
    /// that the factory lacks)
    pub fn readClassData(self: *ObjectStreamIn, rtti: *const RTTI, instance: *anyopaque) Error!void {
        const class_desc = self.class_descriptions.getPtr(rtti.name) orelse return error.InvalidObjectStream;
        return self.readDescribedClassData(class_desc, if (class_desc.rtti == rtti) instance else null);
    }

    /// OSReadData of a reference (ReadPointerData): read the identifier and remember the link. A null identifier leaves
    /// the member as it is (Jolt assigns its local parameter).
    pub fn readPointerData(self: *ObjectStreamIn, rtti: *const RTTI, member: *anyopaque, store: StoreFn, counted: bool) Error!void {
        const identifier = try self.readIdentifier();
        if (identifier == ObjectStream.null_identifier) return;

        // Put pointer on the list to be resolved later on
        try self.unresolved_links.append(self.allocator, .{ .member = member, .identifier = identifier, .rtti = rtti, .store = store, .owner = self.current_object, .counted = counted });
    }

    /// Skip the data of an attribute that is not read (SkipAttributeData). Like Jolt, an `instance` is skipped as class
    /// data, also when it is a Ref (which is an identifier): such a stream cannot be read.
    fn skipAttributeData(self: *ObjectStreamIn, array_depth: u32, data_type: OSDataType, class_name: []const u8) Error!void {
        // Get number of items to read (uint32, multiplied for multi dimensional arrays, wrapping like Jolt's)
        var count: u32 = 1;
        for (0..array_depth) |_| count *%= try self.readCount();

        // Read data for all items
        if (data_type == .instance) {
            // Get the class description
            const class_desc = self.class_descriptions.getPtr(class_name) orelse {
                log.info("ObjectStreamIn: Found instance of unknown class {s}.", .{class_name});
                return error.InvalidObjectStream;
            };
            for (0..count) |_| try self.readDescribedClassData(class_desc, null);
            return;
        }
        for (0..count) |_| {
            switch (data_type) {
                .pointer => _ = try self.readIdentifier(),
                .t_string => self.allocator.free(try self.readString()),
                inline .t_uint8, .t_uint16, .t_int, .t_uint32, .t_uint64, .t_float, .t_double, .t_bool, .t_float3, .t_float4, .t_double3, .t_vec3, .t_dvec3, .t_vec4, .t_uvec4, .t_quat, .t_mat44, .t_dmat44 => |t| {
                    const T = PrimitiveType(t);
                    var temporary: T = undefined;
                    try self.readPrimitive(T, &temporary);
                },
                else => return error.InvalidObjectStream,
            }
        }
    }

    /// A failure that Jolt ignores (returns null / false without stopping): only allocation and reader failures pass
    fn softFailure(err: Error) Error!?MainObject {
        return if (err == error.InvalidObjectStream) null else err;
    }

    // Input type specific operations (the virtual functions of IObjectStreamIn)

    pub fn readDataType(self: *ObjectStreamIn) Error!OSDataType {
        switch (self.format) {
            inline else => |*f| return f.readDataType(self.allocator),
        }
    }

    /// A name, valid until the next read
    pub fn readName(self: *ObjectStreamIn) Error![]const u8 {
        switch (self.format) {
            inline else => |*f| return f.readName(self.allocator),
        }
    }

    pub fn readIdentifier(self: *ObjectStreamIn) Error!Identifier {
        switch (self.format) {
            inline else => |*f| return f.readIdentifier(self.allocator),
        }
    }

    pub fn readCount(self: *ObjectStreamIn) Error!u32 {
        switch (self.format) {
            inline else => |*f| return f.readCount(self.allocator),
        }
    }

    pub fn readPrimitive(self: *ObjectStreamIn, comptime T: type, value: *T) Error!void {
        switch (self.format) {
            inline else => |*f| return f.readPrimitive(self.allocator, T, value),
        }
    }

    /// A String value, owned by the stream's allocator
    pub fn readString(self: *ObjectStreamIn) Error![]u8 {
        switch (self.format) {
            inline else => |*f| return f.readString(self.allocator),
        }
    }
};
