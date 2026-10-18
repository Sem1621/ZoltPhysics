//! Zolt addition, no Jolt file: Zolt-only tests of the object streams.
//! Status: complete
//!
//! - The file variants of sReadObject / sWriteObject and the default factory (SerializationArchitecture.md S10).
//! - Pins of Zolt's defined behavior where Jolt's behavior is undefined (S16). Jolt cannot run these streams safely,
//!   so the parity tests never generate them. D8 (binary state hashes) belongs to Core/StreamUtils. D10 (allocation
//!   failure) is the Z2 sweep of ObjectStreamSweeps.zig. D15 (class data nested too deep) is new: S16 does not list
//!   it yet.

const std = @import("std");
const zolt = @import("zolt");
const ost = @import("ObjectStreamTest.zig");
const LimitedAllocator = @import("ObjectStreamSweeps.zig").LimitedAllocator;

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const ObjectStreamIn = zolt.ObjectStreamIn;
const ObjectStreamOut = zolt.ObjectStreamOut;
const StreamType = zolt.ObjectStream.StreamType;
const MotorSettings = zolt.MotorSettings;
const Ref = zolt.Ref;
const RefCount = zolt.RefCount;

const testing = std.testing;

/// A reference counted class without virtual functions (like Skeleton), with the members that the pins need
const Node = struct {
    pub const rtti = RTTI.declare(@This(), .{
        .name = "Node",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mChild", .child),
            .attr("mValue", .value),
            .attr("mScale", .scale),
            .enumAttr("mMode", .mode),
            .attr("mName", .name),
            .attr("mFlag", .flag),
        },
    });

    /// An exhaustive enum (like EMotionType)
    pub const Mode = enum(u8) { a, b };

    ref_count: RefCount = .{},
    allocator: Allocator,
    child: Ref(Node) = .empty,
    value: u32 = 0,
    scale: f32 = 1,
    mode: Mode = .b,
    /// Owned by `allocator`
    name: []u8 = &.{},
    flag: bool = false,

    /// new Node (reference count 0)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*Node {
        const self = try allocator.create(Node);
        self.* = .{ .allocator = allocator };
        return self;
    }

    pub fn deinit(self: *Node) void {
        self.child.deinit();
        self.allocator.free(self.name);
    }

    pub fn addRef(self: *const Node) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const Node) void {
        if (self.ref_count.release()) {
            const self_mut: *Node = @constCast(self);
            const allocator = self_mut.allocator;
            self_mut.deinit();
            allocator.destroy(self_mut);
        }
    }
};

const node_factory: Factory = .init(&.{Node});

/// Read a Node with the factory of the pins
fn readNode(bytes: []const u8) ObjectStreamIn.Error!*Node {
    var reader: std.Io.Reader = .fixed(bytes);
    return ObjectStreamIn.sReadObject(Node, testing.allocator, &reader, .{ .factory = &node_factory });
}

/// Write `node` to an owned slice
fn writeNode(node: *const Node, stream_type: StreamType) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(testing.allocator);
    defer out.deinit();
    try ObjectStreamOut.sWriteObject(Node, testing.allocator, &out.writer, stream_type, node);
    return out.toOwnedSlice();
}

/// The path of `name` in the temporary directory `tmp`, relative to the working directory (owned)
fn tmpPath(tmp: *const testing.TmpDir, name: []const u8) ![]u8 {
    return std.fmt.allocPrint(testing.allocator, ".zig-cache/tmp/{s}/{s}", .{ &tmp.sub_path, name });
}

test "ObjectStream: sWriteObjectToFile and sReadObjectFromFile, the default factory" {
    const allocator = testing.allocator;
    const io = testing.io;
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    const path = try tmpPath(&tmp, "motor.tof");
    defer allocator.free(path);

    // The default factory is RegisterTypes.factory (Jolt: Factory::sInstance)
    const options: ObjectStreamIn.Options = .{};
    try testing.expectEqual(&zolt.RegisterTypes.factory, options.factory);

    const motor: MotorSettings = .initModeLimits(.stiffness_and_damping, 3.5, 0.25, 100.0, 200.0);
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        // The file holds the bytes of sWriteObject (writing the same path again truncates the file)
        try ObjectStreamOut.sWriteObjectToFile(MotorSettings, allocator, io, path, stream_type, &motor);
        var expected: std.Io.Writer.Allocating = .init(allocator);
        defer expected.deinit();
        try ObjectStreamOut.sWriteObject(MotorSettings, allocator, &expected.writer, stream_type, &motor);
        const file_bytes = try tmp.dir.readFileAlloc(io, "motor.tof", allocator, .unlimited);
        defer allocator.free(file_bytes);
        try testing.expectEqualSlices(u8, expected.written(), file_bytes);

        // Read it back with the default factory
        const read = try ObjectStreamIn.sReadObjectFromFile(MotorSettings, allocator, io, path, .{});
        defer RTTI.of(MotorSettings).destructObject(read, allocator);
        try testing.expectEqual(motor, read.*);
    }

    // A file that does not exist, a directory that does not exist (Jolt: false)
    try testing.expectError(error.FileNotFound, ObjectStreamIn.sReadObjectFromFile(MotorSettings, allocator, io, "Assets/does_not_exist.tof", .{}));
    try testing.expectError(error.FileNotFound, ObjectStreamOut.sWriteObjectToFile(MotorSettings, allocator, io, ".zig-cache/tmp/does_not_exist/motor.tof", .text, &motor));
}

test "ObjectStream: sReadObjectRefFromFile wraps the main object in a Ref" {
    const allocator = testing.allocator;
    const io = testing.io;
    var tmp = testing.tmpDir(.{});
    defer tmp.cleanup();
    const path = try tmpPath(&tmp, "test.bof");
    defer allocator.free(path);

    const object = try ost.createTestObject(allocator);
    defer object.destroy();
    try ObjectStreamOut.sWriteObjectToFile(ost.TestSerializable, allocator, io, path, .binary, object);

    // Read as the base class: the main object is a TestSerializable with one reference, the Ref's
    var ref = try ObjectStreamIn.sReadObjectRefFromFile(ost.TestSerializableBase, allocator, io, path, .{ .factory = &ost.factory });
    defer ref.deinit();
    try testing.expectEqual(@as(u32, 1), ref.ptr.?.getRefCount());
    try testing.expectEqual(RTTI.of(ost.TestSerializable), RTTI.getRTTI(ost.TestSerializableBase, ref.ptr.?));
    const read: *ost.TestSerializable = RTTI.dynamicCast(ost.TestSerializable, ost.TestSerializableBase, ref.ptr).?;
    try testing.expectEqual(@as(f32, 4.5), read.pointer.?.base.float);
}

test "ObjectStream D1: the links of an object whose data fails are dropped with it" {
    // Object 2 links to object 1, then its value does not parse: Jolt resolves the link into the freed object
    const node = try readNode(
        \\TOS 1.00
        \\declare Node 2 mChild instance Node mValue uint32
        \\object Node 00000001 00000002 10
        \\object Node 00000002 00000001 abc
    );
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expectEqual(@as(u32, 10), node.value);
    try testing.expectEqual(@as(?*Node, null), node.child.ptr); // Object 2 does not exist
    try testing.expectEqual(@as(u32, 0), node.ref_count.get()); // The dropped link added no reference
}

test "ObjectStream D2: a second object with a used identifier is destroyed with its links" {
    // Jolt leaks the third object, and its link to object 1 still resolves
    const node = try readNode(
        \\TOS 1.00
        \\declare Node 2 mChild instance Node mValue uint32
        \\object Node 00000001 00000002 1
        \\object Node 00000002 00000000 2
        \\object Node 00000002 00000001 3
    );
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expectEqual(@as(u32, 2), node.child.ptr.?.value); // The first object with identifier 2
    try testing.expectEqual(@as(u32, 1), node.child.ptr.?.ref_count.get());
    try testing.expectEqual(@as(u32, 0), node.ref_count.get());
}

test "ObjectStream D3: an unnamed value of an exhaustive enum keeps the member" {
    // Jolt stores the invalid value
    const unnamed = try readNode(
        \\TOS 1.00
        \\declare Node 1 mMode uint32
        \\object Node 00000001 7
    );
    defer RTTI.of(Node).destructObject(unnamed, testing.allocator);
    try testing.expectEqual(Node.Mode.b, unnamed.mode); // The default
    const named = try readNode(
        \\TOS 1.00
        \\declare Node 1 mMode uint32
        \\object Node 00000001 256
    );
    defer RTTI.of(Node).destructObject(named, testing.allocator);
    try testing.expectEqual(Node.Mode.a, named.mode); // static_cast<uint8>(256) is 0
}

test "ObjectStream D4: the main object survives an unreferenced object that references it" {
    // Jolt deletes the main object when it deletes object 2 and returns a dangling pointer
    const node = try readNode(
        \\TOS 1.00
        \\declare Node 2 mChild instance Node mValue uint32
        \\object Node 00000001 00000000 1
        \\object Node 00000002 00000001 2
    );
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expectEqual(@as(u32, 1), node.value);
    try testing.expectEqual(@as(u32, 0), node.ref_count.get());
}

test "ObjectStream D5: a binary bool byte other than 0 or 1 is true" {
    const object = try Node.createDefault(testing.allocator);
    defer RTTI.of(Node).destructObject(object, testing.allocator);
    object.flag = true;
    const bytes = try writeNode(object, .binary);
    defer testing.allocator.free(bytes);
    try testing.expectEqual(@as(u8, 1), bytes[bytes.len - 1]); // mFlag is the last attribute
    bytes[bytes.len - 1] = 2; // Undefined in Jolt
    const node = try readNode(bytes);
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expect(node.flag);
}

test "ObjectStream D6: a binary string length beyond the stream fails" {
    const object = try Node.createDefault(testing.allocator);
    defer RTTI.of(Node).destructObject(object, testing.allocator);
    object.name = try testing.allocator.dupe(u8, "abc");
    const bytes = try writeNode(object, .binary);
    defer testing.allocator.free(bytes);
    const at = std.mem.indexOf(u8, bytes, &[_]u8{ 3, 0, 0, 0, 'a', 'b', 'c' }).?;
    std.mem.writeInt(u32, bytes[at..][0..4], 0x7fffffff, .little); // Jolt: JPH_STACK_ALLOC(len + 1), a stack overflow

    // Zolt reads the string in chunks. The allocator refuses requests above 64 KB, so a reader that allocated the
    // corrupt length up front would fail with error.OutOfMemory. (The testing allocator would hand out the 2 GB.)
    var limited: LimitedAllocator = .{ .child = testing.allocator, .limit = 64 << 10 };
    var reader: std.Io.Reader = .fixed(bytes);
    try testing.expectError(error.InvalidObjectStream, ObjectStreamIn.sReadObject(Node, limited.allocator(), &reader, .{ .factory = &node_factory }));
    try testing.expect(!limited.refused);
}

test "ObjectStream D7: a text identifier after an ERANGE of an earlier read" {
    // Object 2 fails with ERANGE (a subnormal float). Jolt's ReadIdentifier tests errno without clearing it, so it may
    // fail on object 3. Zolt only fails a 64 bit overflow: "100000003" is identifier 3 (truncated to 32 bits).
    const node = try readNode(
        \\TOS 1.00
        \\declare Node 2 mChild instance Node mScale float
        \\object Node 00000001 00000003 2
        \\object Node 00000002 00000000 1e-40
        \\object Node 100000003 00000000 3
    );
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expectEqual(@as(f32, 3), node.child.ptr.?.scale);
}

test "ObjectStream D9: members that C++ leaves indeterminate are zero" {
    const color: *zolt.Color = @ptrCast(@alignCast((try RTTI.of(zolt.Color).createObject(testing.allocator)).?));
    defer RTTI.of(zolt.Color).destructObject(color, testing.allocator);
    try testing.expectEqual([4]u8{ 0, 0, 0, 0 }, [4]u8{ color.r, color.g, color.b, color.a }); // new Color: garbage
}

test "ObjectStream D11: two stream attributes for one attribute of a class are an invalid stream" {
    // Jolt reads the member twice
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 2 mValue uint32 mValue uint32
        \\object Node 00000001 1 2
    ));
    // A second declaration appends its attributes to the first (Jolt's try_emplace)
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 1 mValue uint32
        \\declare Node 1 mValue uint32
        \\object Node 00000001 1 2
    ));
}

test "ObjectStream D12: references that form a cycle are an invalid stream" {
    // Jolt reads the objects and leaks them
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 1 mChild instance Node
        \\object Node 00000001 00000002
        \\object Node 00000002 00000001
    ));
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 1 mChild instance Node
        \\object Node 00000001 00000001
    ));
}

test "ObjectStream D13: an I/O error of the reader is error.ReadFailed" {
    // Jolt ends the read like at the end of the stream
    const object = try Node.createDefault(testing.allocator);
    defer RTTI.of(Node).destructObject(object, testing.allocator);
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        const bytes = try writeNode(object, stream_type);
        defer testing.allocator.free(bytes);
        for ([_]usize{ 0, 8, bytes.len / 2, bytes.len }) |len| {
            // A reader that delivers `len` bytes, then fails
            var reader: std.Io.Reader = .failing;
            reader.buffer = bytes[0..len];
            reader.end = len;
            try testing.expectError(error.ReadFailed, ObjectStreamIn.sReadObject(Node, testing.allocator, &reader, .{ .factory = &node_factory }));
        }
    }
}

test "ObjectStream D14: a hexadecimal float that is exactly subnormal fails like every subnormal" {
    // glibc reads it, Jolt's ObjectStreamTextIn too
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 1 mScale float
        \\object Node 00000001 0x1p-140
    ));
    const node = try readNode(
        \\TOS 1.00
        \\declare Node 1 mScale float
        \\object Node 00000001 0x1p-126
    );
    defer RTTI.of(Node).destructObject(node, testing.allocator);
    try testing.expectEqual(std.math.floatMin(f32), node.scale);
}

/// A value class that holds its children by value, so its data nests as deep as the stream says
const Tree = struct {
    pub const rtti = RTTI.declare(@This(), .{
        .name = "Tree",
        .kind = .non_virtual,
        .attributes = &.{.attr("mKids", .kids)},
    });

    kids: std.ArrayList(Tree) = .empty,

    pub fn deinit(self: *Tree, allocator: Allocator) void {
        for (self.kids.items) |*kid| kid.deinit(allocator);
        self.kids.deinit(allocator);
    }

    /// The number of levels of the first branch
    fn depth(self: *const Tree) usize {
        var levels: usize = 1;
        var tree = self;
        while (tree.kids.items.len > 0) : (levels += 1) tree = &tree.kids.items[0];
        return levels;
    }
};

const tree_factory: Factory = .init(&.{ Node, Tree });

/// A text stream with `levels` levels of class data:
/// - `skip = false`: the main object is a Tree with one child per level;
/// - `skip = true`: the main Node links to object 2. Then an object of the unknown class V is skipped, whose data
///   nests `levels` levels. Object 2 follows, so the link resolves only when the skip succeeds.
fn deepStream(levels: usize, skip: bool) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(testing.allocator);
    defer out.deinit();
    const w = &out.writer;
    if (skip)
        try w.writeAll(
            \\TOS 1.00
            \\declare Node 2 mChild instance Node mValue uint32
            \\declare A 1 mKids array instance A
            \\declare V 1 mOther array instance A
            \\object Node 00000001 00000002 1
            \\object V 00000003
        )
    else
        try w.writeAll(
            \\TOS 1.00
            \\declare Tree 1 mKids array instance Tree
            \\object Tree 00000001
        );
    for (1..levels) |_| try w.writeAll(" 1"); // The count of each level but the last opens the next level
    try w.writeAll(" 0\n");
    if (skip) try w.writeAll("object Node 00000002 00000000 2\n");
    return out.toOwnedSlice();
}

fn readTree(bytes: []const u8) ObjectStreamIn.Error!*Tree {
    var reader: std.Io.Reader = .fixed(bytes);
    return ObjectStreamIn.sReadObject(Tree, testing.allocator, &reader, .{ .factory = &tree_factory });
}

test "ObjectStream D15: class data nested too deep is an invalid stream" {
    // Jolt: a stack overflow. V is not in the factory, so its object is skipped, and skipping mOther skips a V again.
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare V 1 mOther instance V
        \\object V 00000001
    ));
    // The same with a known class: Node has no attribute mOther, so it is skipped as an instance of Node, which has
    // the attribute mOther again
    try testing.expectError(error.InvalidObjectStream, readNode(
        \\TOS 1.00
        \\declare Node 1 mOther instance Node
        \\object Node 00000001
    ));

    // A skipped object: up to the limit the skip succeeds and the read goes on to object 2. One level more fails the
    // skip, and the read ends at the next word (Jolt: a stack overflow for a deep enough stream).
    const max = ObjectStreamIn.max_nesting_depth;
    for ([_]usize{ max, max + 1 }) |levels| {
        const bytes = try deepStream(levels, true);
        defer testing.allocator.free(bytes);
        const node = try readNode(bytes);
        defer RTTI.of(Node).destructObject(node, testing.allocator);
        try testing.expectEqual(levels == max, node.child.ptr != null);
    }

    // An object that is read: a Tree with `max` levels is read, one level more is an invalid stream
    {
        const bytes = try deepStream(max, false);
        defer testing.allocator.free(bytes);
        const tree = try readTree(bytes);
        defer RTTI.of(Tree).destructObject(tree, testing.allocator);
        try testing.expectEqual(max, tree.depth());
    }
    {
        const bytes = try deepStream(max + 1, false);
        defer testing.allocator.free(bytes);
        try testing.expectError(error.InvalidObjectStream, readTree(bytes));
    }
}
