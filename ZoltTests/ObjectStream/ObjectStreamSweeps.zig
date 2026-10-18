//! Zolt addition, no Jolt file: the Zolt-only sweeps Z1 to Z4 of the object streams
//! (Docs/Zolt/SerializationArchitecture.md, section 5).
//! Status: complete
//!
//! - Z1: every creatable class of the factory. A default object is written as text, read, written as binary, read and
//!   written as text again. The bytes do not change.
//! - Z2: every allocation of reading and writing fails once: the assets, the object of ObjectStreamTest, the linked
//!   objects and every Z1 default. The result is error.OutOfMemory, and nothing leaks.
//! - Z3: every prefix of Assets/Human.tof and of its binary form is read, and every prefix of the streams of the
//!   object of ObjectStreamTest and of the linked objects. The result is an error or a success, and nothing leaks.
//! - Z4: byte and word mutations of the streams are read with an allocator that refuses large requests. Nothing
//!   crashes, leaks or hits a safety check. One fixed word mutation of Human.tof makes a class contain itself (D15).
//!
//! The sweeps take the classes from the factory at run time (`getAllClasses`), so they pick up every class that is
//! declared later. An asset is read as the class of its main object, found by name in the factory. Phase 8 is done, so
//! every asset must read: a failure fails the sweep.
//!
//! The assets are read relative to the working directory, so the sweeps need the repository root as working
//! directory. build.zig gives the unit test run no working directory (only the parity run and the test run of
//! ZoltPerformanceTest have one), so `zig build test` must be started in the repository root. Started elsewhere, Z2 to
//! Z4 fail with error.FileNotFound.
//!
//! Limits of the sweeps (a read in Debug is about 10 times slower than in ReleaseFast, and both run in CI):
//! - Z2 makes every allocation of the Z1 defaults, of the object of ObjectStreamTest and of the linked objects fail.
//!   For an asset it makes at most `max_asset_runs` allocations fail, spread evenly over all of them, because every
//!   run reads the whole asset.
//! - Z3 reads every prefix in an optimized build, and every `prefix_step`-th prefix in Debug. It counts the bytes of
//!   a fast allocator to find leaks: the testing allocator would make it 5 times slower.
//! - Z4 does not mutate the object of ObjectStreamTest. Its raw pointer `mPointer` does not own its object, as in
//!   Jolt, so a mutated stream can leave an object that only a raw pointer reaches. That is a leak in Jolt too.
//!   Z4 mutates the streams of `Linked` objects instead: they link to each other through Refs only, so the mutations
//!   reach the links (their resolution, the cycle check, the objects that nothing references) before the assets can
//!   be read.
//!
//! A change that makes the sweeps much slower (more classes in the assets) adjusts the limits below.

const std = @import("std");
const builtin = @import("builtin");
const zolt = @import("zolt");
const ost = @import("ObjectStreamTest.zig");

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const ObjectStreamIn = zolt.ObjectStreamIn;
const ObjectStreamOut = zolt.ObjectStreamOut;
const StreamType = zolt.ObjectStream.StreamType;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RefCount = zolt.RefCount;

/// The classes of RegisterTypes and of ObjectStreamTest, Linked, plus SpringSettings and the classes that Jolt declares
/// outside the class (TypeDeclarations). Jolt's classes reach the last ones through their attributes.
const factory: Factory = .init(zolt.RegisterTypes.factory_types ++ &[_]type{ ost.TestSerializable, Linked, zolt.SpringSettings, zolt.Color, zolt.AABox, zolt.Triangle, zolt.IndexedTriangle, zolt.Plane });

/// Zolt: a reference counted class whose objects link to each other through Refs only. Raw pointers are left out: an
/// object that only raw pointers reach is never released, in Jolt too.
const Linked = struct {
    pub const rtti = RTTI.declare(@This(), .{
        .name = "Linked",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mValue", .value),
            .attr("mFirst", .first),
            .attr("mSecond", .second),
            .attr("mList", .list),
            .attr("mName", .name),
        },
    });

    ref_count: RefCount = .{},
    allocator: Allocator,
    value: u32 = 0,
    first: Ref(Linked) = .empty,
    second: RefConst(Linked) = .empty,
    list: std.ArrayList(Ref(Linked)) = .empty,
    /// Owned by `allocator`
    name: []u8 = &.{},

    /// new Linked (reference count 0)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*Linked {
        const self = try allocator.create(Linked);
        self.* = .{ .allocator = allocator };
        return self;
    }

    pub fn deinit(self: *Linked) void {
        self.first.deinit();
        self.second.deinit();
        for (self.list.items) |*r| r.deinit();
        self.list.deinit(self.allocator);
        self.allocator.free(self.name);
    }

    pub fn addRef(self: *const Linked) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const Linked) void {
        if (self.ref_count.release()) {
            const self_mut: *Linked = @constCast(self);
            const allocator = self_mut.allocator;
            self_mut.deinit();
            allocator.destroy(self_mut);
        }
    }

    /// A new object with `value`, held by `holder` (so that the holder frees it)
    fn add(holder: *Ref(Linked), allocator: Allocator, value: u32) Allocator.Error!*Linked {
        const object = try createDefault(allocator);
        object.value = value;
        holder.set(object);
        return object;
    }

    /// Five objects with shared references: root -> a, b and [a, c, b], a -> c, b -> [c], c -> d. The root has
    /// reference count 0, `RTTI.of(Linked).destructObject` frees all of them.
    fn createGraph(allocator: Allocator) Allocator.Error!*Linked {
        const root = try createDefault(allocator);
        errdefer RTTI.of(Linked).destructObject(root, allocator);
        root.value = 1;
        root.name = try allocator.dupe(u8, "root \"of\" the graph");
        const a = try add(&root.first, allocator, 2);
        var b_ref: Ref(Linked) = .empty;
        defer b_ref.deinit();
        const b = try add(&b_ref, allocator, 3);
        root.second.set(b);
        const c = try add(&a.first, allocator, 4);
        try root.list.ensureTotalCapacityPrecise(allocator, 3);
        for ([_]*Linked{ a, c, b }) |o| root.list.appendAssumeCapacity(.init(o));
        try b.list.ensureTotalCapacityPrecise(allocator, 1);
        b.list.appendAssumeCapacity(.init(c));
        var d_ref: Ref(Linked) = .empty;
        defer d_ref.deinit();
        const d = try add(&d_ref, allocator, 5);
        d.name = try allocator.dupe(u8, "d");
        c.second.set(d);
        return root;
    }
};

const Asset = struct {
    path: []const u8,
    /// The class of the main object
    class_name: []const u8,
};

/// The assets of PerformanceTest's Ragdoll scene (milestone M9)
const assets = [_]Asset{
    .{ .path = "Assets/Human.tof", .class_name = "RagdollSettings" },
    .{ .path = "Assets/Human/dead_pose1.tof", .class_name = "SkeletalAnimation" },
    .{ .path = "Assets/terrain2.bof", .class_name = "PhysicsScene" },
};

const debug = builtin.mode == .Debug;

/// Z2 on an asset with more allocations tests this many of them
const max_asset_runs: usize = if (debug) 64 else 256;

/// Z3 in Debug reads every `prefix_step`-th prefix (a prime, so the cuts fall on every position of the words)
const prefix_step: usize = if (debug) 23 else 1;

/// Z4: the number of mutated streams per asset stream, per Z1 stream and per stream of the linked objects
const asset_mutations: usize = if (debug) 64 else 256;
const default_mutations: usize = 16;
const linked_mutations: usize = if (debug) 512 else 2048;

/// Z3 and Z4: the allocator refuses requests above this size (a corrupt count must not exhaust the machine)
const sweep_limit = 16 << 20;

/// An allocator that refuses requests above `limit` bytes and remembers that it did
pub const LimitedAllocator = struct {
    child: Allocator,
    limit: usize = 256 << 20,
    /// A request was refused
    refused: bool = false,

    pub fn allocator(self: *LimitedAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = resize, .remap = remap, .free = free } };
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *LimitedAllocator = @ptrCast(@alignCast(ctx));
        if (len > self.limit) {
            self.refused = true;
            return null;
        }
        return self.child.rawAlloc(len, alignment, ret_addr);
    }

    fn resize(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) bool {
        const self: *LimitedAllocator = @ptrCast(@alignCast(ctx));
        if (new_len > self.limit) {
            self.refused = true;
            return false;
        }
        return self.child.rawResize(memory, alignment, new_len, ret_addr);
    }

    fn remap(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, new_len: usize, ret_addr: usize) ?[*]u8 {
        const self: *LimitedAllocator = @ptrCast(@alignCast(ctx));
        if (new_len > self.limit) {
            self.refused = true;
            return null;
        }
        return self.child.rawRemap(memory, alignment, new_len, ret_addr);
    }

    fn free(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        const self: *LimitedAllocator = @ptrCast(@alignCast(ctx));
        self.child.rawFree(memory, alignment, ret_addr);
    }
};

/// A creatable class (the primitives are creatable too, but a stream cannot hold one as an object)
fn isSweepClass(rtti: *const RTTI) bool {
    return rtti.kind != .primitive and !rtti.isAbstract();
}

/// The class to read an asset as: `class_name` from the factory
fn mainClass(class_name: []const u8) error{UnknownClass}!*const RTTI {
    return factory.find(class_name) orelse {
        std.debug.print("{s} is not in the factory\n", .{class_name});
        return error.UnknownClass;
    };
}

/// The bytes of a file in the repository (owned by the testing allocator)
fn loadAsset(path: []const u8) ![]u8 {
    return std.Io.Dir.cwd().readFileAlloc(std.testing.io, path, std.testing.allocator, .limited(64 << 20)) catch |err| {
        if (err == error.FileNotFound) std.debug.print("{s} not found: start `zig build test` in the repository root\n", .{path});
        return err;
    };
}

/// Write `object` (the most derived object of class `rtti`) with `allocator`. The bytes are owned by the testing
/// allocator, so that an allocation failure of the sweeps never hits them.
fn write(allocator: Allocator, object: *const anyopaque, rtti: *const RTTI, stream_type: StreamType) ObjectStreamOut.Error![]u8 {
    var out: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer out.deinit();
    {
        var stream = try ObjectStreamOut.open(allocator, &out.writer, stream_type);
        defer stream.deinit();
        try stream.write(object, rtti);
    }
    return out.toOwnedSlice();
}

/// Read the main object of `bytes`, which must be kind of `rtti`
fn read(allocator: Allocator, f: *const Factory, bytes: []const u8, rtti: *const RTTI) ObjectStreamIn.Error!ObjectStreamIn.MainObject {
    var reader: std.Io.Reader = .fixed(bytes);
    var stream = try ObjectStreamIn.open(allocator, &reader, f);
    defer stream.deinit();
    return stream.read(rtti);
}

fn destroy(allocator: Allocator, object: ObjectStreamIn.MainObject) void {
    object.rtti.destructObject(object.instance, allocator);
}

/// std.testing.checkAllAllocationFailures with at most `max_runs` failing runs, spread evenly over the allocations
/// (null: one run per allocation)
fn checkAllocationFailures(comptime test_fn: anytype, args: anytype, max_runs: ?usize) !void {
    // Count the allocations of a run that succeeds
    const count = blk: {
        var failing: std.testing.FailingAllocator = .init(std.testing.allocator, .{});
        try @call(.auto, test_fn, .{failing.allocator()} ++ args);
        break :blk failing.alloc_index;
    };

    const runs = if (max_runs) |m| @min(m, count) else count;
    for (0..runs) |run| {
        const fail_index = run * count / runs;
        var failing: std.testing.FailingAllocator = .init(std.testing.allocator, .{ .fail_index = fail_index });
        if (@call(.auto, test_fn, .{failing.allocator()} ++ args)) |_| {
            return if (failing.has_induced_failure) error.SwallowedOutOfMemoryError else error.NondeterministicMemoryUsage;
        } else |err| switch (err) {
            error.OutOfMemory => if (failing.allocated_bytes != failing.freed_bytes) {
                std.debug.print("allocation {d} of {d} failed and leaked\n", .{ fail_index, count });
                return error.MemoryLeakDetected;
            },
            else => |e| return e,
        }
    }
}

/// Z1 of one class: default, text, read, binary, read, text. The bytes do not change.
fn roundTripDefault(allocator: Allocator, rtti: *const RTTI) !void {
    const object = (try rtti.createObject(allocator)).?;
    defer rtti.destructObject(object, allocator);
    const text = try write(allocator, object, rtti, .text);
    defer std.testing.allocator.free(text);
    const binary = try write(allocator, object, rtti, .binary);
    defer std.testing.allocator.free(binary);

    // Text, read, binary
    const from_text = try read(allocator, &factory, text, rtti);
    defer destroy(allocator, from_text);
    try std.testing.expectEqual(rtti, from_text.rtti);
    const binary2 = try write(allocator, from_text.instance, from_text.rtti, .binary);
    defer std.testing.allocator.free(binary2);
    try std.testing.expectEqualSlices(u8, binary, binary2);

    // Binary, read, text
    const from_binary = try read(allocator, &factory, binary2, rtti);
    defer destroy(allocator, from_binary);
    try std.testing.expectEqual(rtti, from_binary.rtti);
    const text2 = try write(allocator, from_binary.instance, from_binary.rtti, .text);
    defer std.testing.allocator.free(text2);
    try std.testing.expectEqualStrings(text, text2);
}

/// Z2 of the object of ObjectStreamTest: create, write, read, write again
fn roundTripTestObject(allocator: Allocator, stream_type: StreamType) !void {
    const object = try ost.createTestObject(allocator);
    defer object.destroy();
    const rtti = RTTI.of(ost.TestSerializable);
    const bytes = try write(allocator, object, rtti, stream_type);
    defer std.testing.allocator.free(bytes);

    const read_object = try read(allocator, &ost.factory, bytes, rtti);
    defer destroy(allocator, read_object);
    const bytes2 = try write(allocator, read_object.instance, read_object.rtti, stream_type);
    defer std.testing.allocator.free(bytes2);
    try std.testing.expectEqualSlices(u8, bytes, bytes2);
}

/// Z2 of the linked objects: create, write, read, write again
fn roundTripLinked(allocator: Allocator, stream_type: StreamType) !void {
    const root = try Linked.createGraph(allocator);
    defer RTTI.of(Linked).destructObject(root, allocator);
    const rtti = RTTI.of(Linked);
    const bytes = try write(allocator, root, rtti, stream_type);
    defer std.testing.allocator.free(bytes);

    const read_object = try read(allocator, &factory, bytes, rtti);
    defer destroy(allocator, read_object);
    const bytes2 = try write(allocator, read_object.instance, read_object.rtti, stream_type);
    defer std.testing.allocator.free(bytes2);
    try std.testing.expectEqualSlices(u8, bytes, bytes2);
}

/// Z2 of an asset: read it and write it in both formats
fn readWriteAsset(allocator: Allocator, bytes: []const u8, rtti: *const RTTI) !void {
    var limited: LimitedAllocator = .{ .child = allocator };
    const a = limited.allocator();
    const object = read(a, &factory, bytes, rtti) catch |err| switch (err) {
        error.InvalidObjectStream, error.ReadFailed => return err,
        error.OutOfMemory => {
            if (limited.refused) return;
            return err;
        },
    };
    defer destroy(a, object);
    for ([_]StreamType{ .text, .binary }) |stream_type|
        std.testing.allocator.free(try write(a, object.instance, object.rtti, stream_type));
}

/// Read a stream that may be cut or corrupt with the limited allocator over `backing`, then write what was read. Only
/// a request that the limit refuses may make it run out of memory.
fn readAnything(backing: Allocator, bytes: []const u8, rtti: *const RTTI) !void {
    var limited: LimitedAllocator = .{ .child = backing, .limit = sweep_limit };
    const a = limited.allocator();
    const object = read(a, &factory, bytes, rtti) catch |err| switch (err) {
        error.InvalidObjectStream => return,
        error.OutOfMemory => {
            if (limited.refused) return;
            return err;
        },
        error.ReadFailed => return err,
    };
    defer destroy(a, object);
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        const out = write(a, object.instance, object.rtti, stream_type) catch |err| switch (err) {
            error.OutOfMemory => if (limited.refused) continue else return err,
            else => return err,
        };
        std.testing.allocator.free(out);
    }
}

/// Z3 of one prefix: read it with a fast allocator that counts the bytes, which must all be freed
fn readPrefix(bytes: []const u8, rtti: *const RTTI) !void {
    var counting: std.testing.FailingAllocator = .init(std.heap.smp_allocator, .{});
    try readAnything(counting.allocator(), bytes, rtti);
    if (counting.allocated_bytes != counting.freed_bytes) return error.MemoryLeakDetected;
}

test "ObjectStream Z1: the default object of every creatable class keeps its bytes through text and binary" {
    var count: usize = 0;
    for (factory.getAllClasses()) |rtti| {
        if (!isSweepClass(rtti)) continue;
        roundTripDefault(std.testing.allocator, rtti) catch |err| {
            std.debug.print("Z1: class {s}\n", .{rtti.name});
            return err;
        };
        count += 1;
    }

    // At least the classes of the foundation: MotorSettings, SpringSettings, the 3 classes of ObjectStreamTest and the
    // 6 classes declared outside
    try std.testing.expect(count >= 11);
}

test "ObjectStream Z2: every allocation failure while reading and writing" {
    // Every Z1 default
    for (factory.getAllClasses()) |rtti| {
        if (!isSweepClass(rtti)) continue;
        checkAllocationFailures(roundTripDefault, .{rtti}, null) catch |err| {
            std.debug.print("Z2: class {s}\n", .{rtti.name});
            return err;
        };
    }

    // The object of ObjectStreamTest and the linked objects
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        try checkAllocationFailures(roundTripTestObject, .{stream_type}, null);
        try checkAllocationFailures(roundTripLinked, .{stream_type}, null);
    }

    // The assets
    for (assets) |asset| {
        const bytes = try loadAsset(asset.path);
        defer std.testing.allocator.free(bytes);
        checkAllocationFailures(readWriteAsset, .{ bytes, try mainClass(asset.class_name) }, max_asset_runs) catch |err| {
            std.debug.print("Z2: {s}\n", .{asset.path});
            return err;
        };
    }
}

test "ObjectStream Z2: an asset that cannot be read fails the sweep" {
    const bytes = "TOS 1.00\ndeclare Unknown 0\nobject Unknown 00000001\n";
    try std.testing.expectError(error.InvalidObjectStream, readWriteAsset(std.testing.allocator, bytes, RTTI.of(zolt.SerializableObject)));
}

/// Z3 of one stream: read every `prefix_step`-th prefix and the whole stream
fn readPrefixes(bytes: []const u8, rtti: *const RTTI) !void {
    var len: usize = 0;
    while (len <= bytes.len) : (len += prefix_step) {
        readPrefix(bytes[0..len], rtti) catch |err| {
            std.debug.print("Z3: prefix of {d} bytes of a stream of class {s}\n", .{ len, rtti.name });
            return err;
        };
    }
    try readPrefix(bytes, rtti);
}

test "ObjectStream Z3: every prefix of Human.tof and of its binary form" {
    const text = try loadAsset(assets[0].path);
    defer std.testing.allocator.free(text);
    const rtti = try mainClass(assets[0].class_name);
    try readPrefixes(text, rtti);

    // The binary form
    const object = try read(std.testing.allocator, &factory, text, rtti);
    defer destroy(std.testing.allocator, object);
    const binary = try write(std.testing.allocator, object.instance, object.rtti, .binary);
    defer std.testing.allocator.free(binary);
    try readPrefixes(binary, rtti);
}

test "ObjectStream Z3: every prefix of the object of ObjectStreamTest and of the linked objects" {
    // Zolt: the streams of the unit test have every kind of member and two objects, the linked objects have five
    // objects with shared references. Human.tof only has these once its classes are declared.
    const object = try ost.createTestObject(std.testing.allocator);
    defer object.destroy();
    const root = try Linked.createGraph(std.testing.allocator);
    defer RTTI.of(Linked).destructObject(root, std.testing.allocator);
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        const bytes = try write(std.testing.allocator, object, RTTI.of(ost.TestSerializable), stream_type);
        defer std.testing.allocator.free(bytes);
        try readPrefixes(bytes, RTTI.of(ost.TestSerializable));
        const linked = try write(std.testing.allocator, root, RTTI.of(Linked), stream_type);
        defer std.testing.allocator.free(linked);
        try readPrefixes(linked, RTTI.of(Linked));
    }
}

/// Words that a mutation writes into a text stream: numbers at the limits, words that fail, and the keywords
const text_words = [_][]const u8{
    "0",        "1",        "-1",       "2",        "255",      "65536",   "4294967295", "4294967296", "-2147483649",
    "00000000", "00000001", "00000002", "FFFFFFFF", "1e39",     "1e-40",   "-0",         "nan",        "inf",
    "0x10",     "1.5abc",   "declare",  "object",   "instance", "pointer", "array",      "uint8",      "uint32",
    "float",    "vec3",     "dvec3",    "string",   "true",     "\"",      "\"\\\"",     "",
};

/// Values that a mutation writes into a binary stream: counts and lengths at the limits, string ids and data types
const binary_words = [_]u32{ 0, 1, 2, 3, 4, 7, 24, 25, 0x10000, 0x7fffffff, 0x80000000, 0x80000001, 0xffffffff };

/// The bounds of the text word at or after `pos`, null when only white space follows
fn wordAt(bytes: []const u8, pos: usize) ?struct { start: usize, end: usize } {
    var start = pos;
    while (start < bytes.len and std.ascii.isWhitespace(bytes[start])) start += 1;
    if (start == bytes.len) return null;
    while (start > 0 and !std.ascii.isWhitespace(bytes[start - 1])) start -= 1;
    var end = start;
    while (end < bytes.len and !std.ascii.isWhitespace(bytes[end])) end += 1;
    return .{ .start = start, .end = end };
}

/// Apply 1 to 3 random mutations to `stream` (the header stays)
fn mutate(random: std.Random, stream: *std.ArrayList(u8)) !void {
    const allocator = std.testing.allocator;
    const header_len = 8;
    const text = stream.items[0] == 'T';
    for (0..1 + random.uintLessThan(usize, 3)) |_| {
        const len = stream.items.len;
        if (len <= header_len) return;
        const pos = header_len + random.uintLessThan(usize, len - header_len);
        switch (random.uintLessThan(u32, 4)) {
            // A random byte
            0 => stream.items[pos] = random.int(u8),
            // A flipped bit
            1 => stream.items[pos] ^= @as(u8, 1) << random.int(u3),
            // A deleted range of 1 to 8 bytes
            2 => try stream.replaceRange(allocator, pos, @min(len - pos, 1 + random.uintLessThan(usize, 8)), &.{}),
            // A replaced word
            3 => if (text) {
                const word = wordAt(stream.items, pos) orelse continue;
                var replacement_buffer: [64]u8 = undefined;
                const replacement: []const u8 = if (random.boolean()) text_words[random.uintLessThan(usize, text_words.len)] else blk: {
                    // Another word of the stream
                    const other = wordAt(stream.items, header_len + random.uintLessThan(usize, len - header_len)) orelse continue;
                    const n = @min(other.end - other.start, replacement_buffer.len);
                    @memcpy(replacement_buffer[0..n], stream.items[other.start..][0..n]);
                    break :blk replacement_buffer[0..n];
                };
                try stream.replaceRange(allocator, word.start, word.end - word.start, replacement);
            } else {
                if (len - pos < 4) continue;
                const value = if (random.boolean()) binary_words[random.uintLessThan(usize, binary_words.len)] else random.int(u32);
                std.mem.writeInt(u32, stream.items[pos..][0..4], value, .little);
            },
            else => unreachable,
        }
    }
}

/// Z4 of one stream: `count` mutated copies are read and written back
fn readMutations(random: std.Random, bytes: []const u8, rtti: *const RTTI, count: usize) !void {
    var stream: std.ArrayList(u8) = .empty;
    defer stream.deinit(std.testing.allocator);
    for (0..count) |i| {
        stream.clearRetainingCapacity();
        try stream.appendSlice(std.testing.allocator, bytes);
        try mutate(random, &stream);
        readAnything(std.testing.allocator, stream.items, rtti) catch |err| {
            std.debug.print("Z4: mutation {d} of a stream of class {s}\n", .{ i, rtti.name });
            return err;
        };
    }
}

test "ObjectStream Z4: mutated streams never crash or leak" {
    var prng: std.Random.DefaultPrng = .init(0x5a4f4c54);
    const random = prng.random();

    // The text assets and their binary forms
    for (assets[0..2]) |asset| {
        const text = try loadAsset(asset.path);
        defer std.testing.allocator.free(text);
        const rtti = try mainClass(asset.class_name);
        try readMutations(random, text, rtti, asset_mutations);

        const object = try read(std.testing.allocator, &factory, text, rtti);
        defer destroy(std.testing.allocator, object);
        const binary = try write(std.testing.allocator, object.instance, object.rtti, .binary);
        defer std.testing.allocator.free(binary);
        try readMutations(random, binary, rtti, asset_mutations);
    }

    // The linked objects: the mutations reach the links (D1, D2, D4, D12)
    {
        const root = try Linked.createGraph(std.testing.allocator);
        defer RTTI.of(Linked).destructObject(root, std.testing.allocator);
        for ([_]StreamType{ .text, .binary }) |stream_type| {
            const bytes = try write(std.testing.allocator, root, RTTI.of(Linked), stream_type);
            defer std.testing.allocator.free(bytes);
            try readMutations(random, bytes, RTTI.of(Linked), linked_mutations);
        }
    }

    // The streams of every Z1 default
    for (factory.getAllClasses()) |rtti| {
        if (!isSweepClass(rtti)) continue;
        const object = (try rtti.createObject(std.testing.allocator)).?;
        defer rtti.destructObject(object, std.testing.allocator);
        for ([_]StreamType{ .text, .binary }) |stream_type| {
            const bytes = try write(std.testing.allocator, object, rtti, stream_type);
            defer std.testing.allocator.free(bytes);
            try readMutations(random, bytes, rtti, default_mutations);
        }
    }
}

test "ObjectStream Z4: Human.tof with a class that contains itself" {
    // A word of the stream that makes RagdollSettings contain itself (D15): skipping mSkeleton skips a RagdollSettings
    // again (Jolt: a stack overflow). It is read as RagdollSettings, which skips mSkeleton (a Ref(Skeleton) is not an
    // instance of RagdollSettings). The stream is invalid.
    const text = try loadAsset(assets[0].path);
    defer std.testing.allocator.free(text);
    const mutated = try std.mem.replaceOwned(u8, std.testing.allocator, text, "mSkeleton instance Skeleton", "mSkeleton instance RagdollSettings");
    defer std.testing.allocator.free(mutated);
    try std.testing.expect(mutated.len != text.len); // The asset has the declaration
    const rtti = try mainClass(assets[0].class_name);
    try std.testing.expectError(error.InvalidObjectStream, read(std.testing.allocator, &factory, mutated, rtti));
    try readAnything(std.testing.allocator, mutated, rtti);
}
