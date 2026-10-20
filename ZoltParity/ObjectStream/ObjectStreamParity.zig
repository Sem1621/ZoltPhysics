//! The generic serialization parity harness (Docs/Zolt/SerializationArchitecture.md section 5): Zolt's RTTI, Factory
//! and ObjectStream against the C++ library built with JPH_OBJECT_STREAM (build.zig). C ABI wrappers:
//! ObjectStreamReference.cpp. The class model is ObjectStreamSchema.zig, the stream generator StreamGenerator.zig, the
//! number words NumberWords.zig.
//!
//! The harness has no code for one class. It takes every class of the C++ Factory after RegisterTypes and the Zolt
//! factory (`RegisterTypes.factory`). Phase 8 is done and the soft body classes are declared (Phase 9 group S0), so
//! every class is declared in full. P1 fails when a class is missing, pending (`serialization_pending`, S18) or reaches
//! a pending class. So the other layers compare every class, and a class that is not ready fails them too.
//!
//! - P1 schema: every class against the Zolt factory (name, hash, abstract flag, bases, attributes with their declared
//!   types and primitive types). The IsKindOf matrix.
//! - P2 defaults: every creatable class, C++ CreateObject and write against Zolt create and write, text and binary.
//! - P3 random streams from the C++ schema, both formats; each side reads the other's output.
//! - P4 old schemas: mutations of P3 streams (dropped, reordered, renamed and retyped attributes, vec3 / dvec3, a class
//!   declared twice, unknown and undeclared classes, abstract objects, null, dangling and wrong type identifiers, a main
//!   object of the wrong class), and the old MotorSettings streams of the prototype.
//! - P5 truncation: every prefix of the default and random streams (up to 64 KB), 2000 prefixes of terrain2.bof.
//! - P6 number words: more than 10^5 words through C++'s ObjectStreamTextIn and Zolt's.
//! - P7 assets: Human.tof, dead_pose1.tof and terrain2.bof read by both sides, written again in both formats, and the
//!   SaveBinaryState bytes of the loaded objects. The same for random PhysicsScenes with soft bodies (P3's generator),
//!   without the shapes (no asset has soft bodies).
//! - P9 binary state: each side restores the SaveBinaryState bytes of those random scenes that the other side wrote
//!   (PhysicsScene::sRestoreFromBinaryState) and writes the same bytes again. (The binary state parity of the other
//!   layers, e.g. BodyInterfaceParity.zig, is P9 of SerializationArchitecture.md too.)
//! - P8 the object of UnitTests/ObjectStream/ObjectStreamTest.cpp in both formats and both directions.
//!
//! Comparing bytes: both sides read a stream as the requested class and write the main object in both formats; the
//! result (read or not) and the bytes must be equal. Every class is compared this way, PhysicsScene with its soft bodies
//! too (RemainingFeaturesArchitecture.md R19; until group S2 it was compared by cross-reading, S17).
//!
//! P3 and P4 go on with the next class after a mismatch, and list the classes that failed at the end.
//!
//! The C++ side of P2 to P7 reads with the Factory after RegisterTypes and with allocation hooks that zero every block
//! and defer frees until the call returns. Every such call checks that its hooks were still installed at the end (-5
//! otherwise, a harness error). Every C++ read starts with errno = 0 (D7).

const std = @import("std");
const zolt = @import("zolt");
const ost = @import("objectstream_test");
const schema_mod = @import("ObjectStreamSchema.zig");
const gen = @import("StreamGenerator.zig");
const number_words = @import("NumberWords.zig");

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const ObjectStreamIn = zolt.ObjectStreamIn;
const ObjectStreamOut = zolt.ObjectStreamOut;
const StreamType = zolt.ObjectStream.StreamType;
const Schema = schema_mod.Schema;
const Class = schema_mod.Class;
const cppResult = schema_mod.cppResult;

// Reading and writing on both sides --------------------------------------------------------------------------------

/// A main object written again in both formats
const Resaved = struct {
    text: []u8,
    binary: []u8,

    fn deinit(self: Resaved, allocator: Allocator) void {
        allocator.free(self.text);
        allocator.free(self.binary);
    }

    fn get(self: Resaved, format: gen.Format) []const u8 {
        return if (format == .text) self.text else self.binary;
    }
};

const formats = [_]gen.Format{ .text, .binary };

/// C++: sReadObject of `bytes` as `class`, sWriteObject of the main object in both formats (null: the read failed)
fn cppResave(allocator: Allocator, bytes: []const u8, class: [:0]const u8) !?Resaved {
    const result = try schema_mod.quarantined(schema_mod.jolt_os_resave(bytes.ptr, bytes.len, class));
    if (result == -1) return null;
    if (result != 0) return error.HarnessError;
    const text = try allocator.dupe(u8, cppResult(0));
    errdefer allocator.free(text);
    return .{ .text = text, .binary = try allocator.dupe(u8, cppResult(1)) };
}

/// Zolt: write `object` (the most derived object of class `rtti`) as a stream of `format`
fn zoltWrite(allocator: Allocator, object: *const anyopaque, rtti: *const RTTI, format: gen.Format) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var stream = try ObjectStreamOut.open(allocator, &out.writer, if (format == .text) .text else .binary);
    defer stream.deinit();
    try stream.write(object, rtti);
    return out.toOwnedSlice();
}

/// Zolt: read `bytes` as `class` with the Zolt factory, write the main object in both formats (null: the read failed)
fn zoltResave(allocator: Allocator, bytes: []const u8, class: []const u8) !?Resaved {
    const rtti = schema_mod.factory.find(class) orelse return error.HarnessError;
    var reader: std.Io.Reader = .fixed(bytes);
    var in = ObjectStreamIn.open(allocator, &reader, schema_mod.factory) catch |err| switch (err) {
        error.InvalidObjectStream => return null,
        else => |e| return e,
    };
    defer in.deinit();
    const main = in.read(rtti) catch |err| switch (err) {
        error.InvalidObjectStream => return null,
        else => |e| return e,
    };
    defer main.rtti.destructObject(main.instance, allocator);
    // The generators never put an object of a derived class where the main object can come from, so the C++ side
    // writes the main object as the requested class
    if (main.rtti != rtti) {
        std.debug.print("Zolt read a main object of class {s}, requested {s}\n", .{ main.rtti.name, rtti.name });
        return error.ParityMismatch;
    }
    const text = try zoltWrite(allocator, main.instance, main.rtti, .text);
    errdefer allocator.free(text);
    return .{ .text = text, .binary = try zoltWrite(allocator, main.instance, main.rtti, .binary) };
}

/// The names of the classes that a text stream declares
fn declaredClasses(arena: Allocator, text: []const u8) ![]const []const u8 {
    var names: std.ArrayList([]const u8) = .empty;
    var words = std.mem.tokenizeAny(u8, text, " \r\n\t");
    while (words.next()) |w| {
        if (std.mem.eql(u8, w, "declare")) {
            if (words.next()) |name| try names.append(arena, name);
        }
    }
    return names.items;
}

/// What a comparison was about, printed on a mismatch
const Context = struct {
    layer: []const u8,
    class: []const u8,
    detail: []const u8 = "",
    seed: u64 = 0,
    stream: []const u8 = "",

    fn report(self: Context, what: []const u8) void {
        std.debug.print("{s} {s} {s} (seed {d}): {s}\n", .{ self.layer, self.class, self.detail, self.seed, what });
        printStream("stream", self.stream);
    }
};

fn printStream(label: []const u8, bytes: []const u8) void {
    const shown = bytes[0..@min(bytes.len, 3000)];
    if (bytes.len >= 3 and bytes[0] == 'T') {
        std.debug.print("  {s} ({d} bytes):\n{s}\n", .{ label, bytes.len, shown });
    } else {
        std.debug.print("  {s} ({d} bytes): {x}\n", .{ label, bytes.len, shown });
    }
}

fn expectSame(ctx: Context, what: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes)) return;
    ctx.report(what);
    const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse 0;
    std.debug.print("  bytes differ at {d}\n", .{at});
    printStream("zolt", zolt_bytes);
    printStream("jolt", jolt_bytes);
    return error.ParityMismatch;
}

const Depth = enum {
    /// The result and the bytes written again
    first,
    /// Also: each side reads the other's bytes and writes them again unchanged
    full,
};

/// Both libraries read `bytes` as `class` and write the main object in both formats: the same result and the same bytes.
/// Returns true when both read the stream.
fn roundTrip(allocator: Allocator, bytes: []const u8, class: [:0]const u8, ctx_in: Context, depth: Depth) !bool {
    var ctx = ctx_in;
    ctx.stream = bytes;
    const jolt = try cppResave(allocator, bytes, class);
    defer if (jolt) |j| j.deinit(allocator);
    const z = try zoltResave(allocator, bytes, class);
    defer if (z) |zz| zz.deinit(allocator);
    if ((jolt == null) != (z == null)) {
        ctx.report(if (jolt == null) "Zolt reads the stream, Jolt does not" else "Jolt reads the stream, Zolt does not");
        if (jolt) |j| printStream("jolt", j.text);
        if (z) |zz| printStream("zolt", zz.text);
        return error.ParityMismatch;
    }
    const j = jolt orelse return false;
    const zz = z.?;

    for (formats) |f|
        try expectSame(ctx, if (f == .text) "written again as text" else "written again as binary", zz.get(f), j.get(f));
    if (depth == .first) return true;

    // Each side reads the bytes that the other wrote, and both write the same bytes again. (Not always the bytes that
    // were read: a binary stream cuts a string at its first NUL, S15.)
    for (formats) |f| {
        const c2 = try mustResave(allocator, .jolt, zz.get(f), class, ctx);
        defer c2.deinit(allocator);
        const z2 = try mustResave(allocator, .zolt, j.get(f), class, ctx);
        defer z2.deinit(allocator);
        for (formats) |g|
            try expectSame(ctx, "each side reads the other's bytes", z2.get(g), c2.get(g));
    }
    return true;
}

/// Read and write again bytes that one of the libraries wrote: the read must succeed
fn mustResave(allocator: Allocator, side: enum { jolt, zolt }, bytes: []const u8, class: [:0]const u8, ctx: Context) !Resaved {
    const result = switch (side) {
        .jolt => try cppResave(allocator, bytes, class),
        .zolt => try zoltResave(allocator, bytes, class),
    };
    return result orelse {
        ctx.report(if (side == .jolt) "Jolt cannot read the bytes that Zolt wrote" else "Zolt cannot read the bytes that Jolt wrote");
        printStream("written", bytes);
        return error.ParityMismatch;
    };
}

// The harness itself ------------------------------------------------------------------------------------------------

test {
    // The tests of the generator rules (StreamGenerator.zig)
    _ = gen;
}

test "ObjectStream harness: the C++ allocation quarantine stays installed, zero fills blocks and defers frees" {
    // -5: the hooks are lost after the class lookup of the entry points, -6: the RegisterTypes Factory gained classes
    try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_quarantine_check());
}

// P1 ----------------------------------------------------------------------------------------------------------------

/// The schema line of `rtti` in the format of jolt_os_class_schema
fn zoltSchema(allocator: Allocator, rtti: *const RTTI) ![]u8 {
    var line: std.Io.Writer.Allocating = .init(allocator);
    errdefer line.deinit();
    const w = &line.writer;
    try w.print("{s} {x:0>8} {s} bases=[", .{ rtti.name, rtti.hash, if (rtti.isAbstract()) "abstract" else "creatable" });
    for (rtti.bases, 0..) |base, i| try w.print("{s}{s}", .{ if (i > 0) "," else "", base.rtti.name });
    try w.writeAll("] attributes=[");

    // The declared types, written by a text stream (after its header)
    var types: std.Io.Writer.Allocating = .init(allocator);
    defer types.deinit();
    var out = try ObjectStreamOut.open(allocator, &types.writer, .text);
    defer out.deinit();
    for (rtti.attributes, 0..) |*attribute, i| {
        const start = types.written().len;
        try attribute.writeDataType(&out);
        const primitive = if (attribute.getMemberPrimitiveType()) |p| p.name else "-";
        try w.print("{s}{s}:{s}({s})", .{ if (i > 0) ";" else "", attribute.name, types.written()[start..], primitive });
    }
    try w.writeAll("]");
    return line.toOwnedSlice();
}

test "ObjectStream P1: the schema of every class of the C++ Factory after RegisterTypes, and IsKindOf" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var mismatches: usize = 0;
    var compared: usize = 0;
    for (schema.classes) |*c| {
        const rtti = c.zolt orelse {
            std.debug.print("{s} is not in RegisterTypes.factory\n", .{c.name});
            mismatches += 1;
            continue;
        };
        compared += 1;

        // Phase 8 is done: no class is pending or reaches a pending class (S18). The other layers rely on it.
        if (c.waits_for) |p| {
            if (p == rtti) std.debug.print("{s} is pending (`serialization_pending`): declare its attributes\n", .{c.name}) else std.debug.print("{s} reaches the pending class {s}\n", .{ c.name, p.name });
            mismatches += 1;
        }
        if (rtti.hash != c.hash) {
            std.debug.print("hash of {s}: zolt {x:0>8}, jolt {x:0>8}\n", .{ c.name, rtti.hash, c.hash });
            mismatches += 1;
        }

        // A class that the module does not export: the harness cannot see its `serialization_pending` marker
        if (schema_mod.zoltClass(rtti) == null and rtti.kind != .primitive and !c.outside) {
            std.debug.print("{s}: the zolt module does not export its type, export it in Zolt/zolt.zig\n", .{c.name});
            mismatches += 1;
        }

        const zolt_line = try zoltSchema(allocator, rtti);
        defer allocator.free(zolt_line);
        if (!std.mem.eql(u8, zolt_line, c.line)) {
            std.debug.print("schema of {s}\n  zolt: {s}\n  jolt: {s}\n", .{ c.name, zolt_line, c.line });
            mismatches += 1;
        }
    }

    // Every class of the Zolt factory is a class of the C++ Factory after RegisterTypes
    for (schema_mod.factory.getAllClasses()) |rtti| {
        if (schema.find(rtti.name) == null) {
            std.debug.print("{s} is in the Zolt factory, not in the C++ Factory after RegisterTypes\n", .{rtti.name});
            mismatches += 1;
        }
    }

    // IsKindOf of every pair of classes that both sides know
    for (schema.classes, 0..) |*a, i| {
        const ra = a.zolt orelse continue;
        for (schema.classes, 0..) |*b, j| {
            const rb = b.zolt orelse continue;
            if (schema.isKindOf(i, j) != ra.isKindOf(rb)) {
                std.debug.print("IsKindOf({s}, {s}): jolt {}\n", .{ a.name, b.name, schema.isKindOf(i, j) });
                mismatches += 1;
            }
        }
    }
    try std.testing.expect(compared > 0);
    try std.testing.expectEqual(@as(usize, 0), mismatches);
}

fn printNames(label: []const u8, names: []const []const u8) void {
    if (names.len == 0) return;
    std.debug.print("{s} ({d}):", .{ label, names.len });
    for (names) |name| std.debug.print(" {s}", .{name});
    std.debug.print("\n", .{});
}

/// The classes outside RegisterTypes that the C++ side registers too (EnsureFactory): every primitive, the
/// TypeDeclarations classes and the classes of ObjectStreamTest.cpp
const extra_classes = .{
    .{ u8, "uint8" },
    .{ u16, "uint16" },
    .{ i32, "int" },
    .{ u32, "uint32" },
    .{ u64, "uint64" },
    .{ f32, "float" },
    .{ f64, "double" },
    .{ bool, "bool" },
    .{ []const u8, "String" },
    .{ zolt.Float3, "Float3" },
    .{ zolt.Float4, "Float4" },
    .{ zolt.Double3, "Double3" },
    .{ zolt.Vec3, "Vec3" },
    .{ zolt.DVec3, "DVec3" },
    .{ zolt.Vec4, "Vec4" },
    .{ zolt.UVec4, "UVec4" },
    .{ zolt.Quat, "Quat" },
    .{ zolt.Mat44, "Mat44" },
    .{ zolt.DMat44, "DMat44" },
    .{ zolt.Color, "Color" },
    .{ zolt.AABox, "AABox" },
    .{ zolt.Triangle, "Triangle" },
    .{ zolt.IndexedTriangleNoMaterial, "IndexedTriangleNoMaterial" },
    .{ zolt.IndexedTriangle, "IndexedTriangle" },
    .{ zolt.Plane, "Plane" },
    .{ zolt.SerializableObject, "SerializableObject" },
    .{ ost.TestSerializableBase, "TestSerializableBase" },
    .{ ost.TestSerializableBase2, "TestSerializableBase2" },
    .{ ost.TestSerializable, "TestSerializable" },
};

test "ObjectStream P1: the primitives, the TypeDeclarations classes and the ObjectStreamTest classes" {
    const allocator = std.testing.allocator;
    var mismatches: usize = 0;
    var rttis: [extra_classes.len]*const RTTI = undefined;
    var names: [extra_classes.len][*:0]const u8 = undefined;
    inline for (extra_classes, 0..) |c, i| {
        rttis[i] = RTTI.of(c[0]);
        names[i] = c[1];
        try std.testing.expectEqualStrings(c[1], rttis[i].name);
        try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_class_schema(c[1]));
        const zolt_line = try zoltSchema(allocator, rttis[i]);
        defer allocator.free(zolt_line);
        if (!std.mem.eql(u8, cppResult(0), zolt_line)) {
            std.debug.print("schema of {s}\n  zolt: {s}\n  jolt: {s}\n", .{ c[1], zolt_line, cppResult(0) });
            mismatches += 1;
        }
    }
    for (rttis, names) |a, a_name| {
        for (rttis, names) |b, b_name| {
            const jolt = schema_mod.jolt_os_is_kind_of(a_name, b_name);
            try std.testing.expect(jolt >= 0);
            if ((jolt == 1) != a.isKindOf(b)) {
                std.debug.print("IsKindOf({s}, {s}): jolt {d}\n", .{ a_name, b_name, jolt });
                mismatches += 1;
            }
        }
    }
    try std.testing.expectEqual(@as(usize, 0), mismatches);
}

// P2 ----------------------------------------------------------------------------------------------------------------

test "ObjectStream P2: the defaults of every creatable class, text and binary" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var compared: usize = 0;
    for (schema.classes) |*c| {
        // The outside classes use the global new, which leaves members that C++ does not initialize as garbage
        if (c.abstract or c.outside) continue;
        const ctx: Context = .{ .layer = "P2", .class = c.name };
        const rtti = try readyClass(ctx, c);
        compared += 1;

        try std.testing.expectEqual(@as(c_int, 0), try schema_mod.quarantined(schema_mod.jolt_os_default_write(c.name)));
        const jolt: Resaved = .{ .text = try allocator.dupe(u8, cppResult(0)), .binary = try allocator.dupe(u8, cppResult(1)) };
        defer jolt.deinit(allocator);

        const object = (try rtti.createObject(allocator)) orelse {
            ctx.report("Jolt creates the class, Zolt cannot (is its kind right?)");
            return error.ParityMismatch;
        };
        defer rtti.destructObject(object, allocator);
        const z: Resaved = .{ .text = try zoltWrite(allocator, object, rtti, .text), .binary = try zoltWrite(allocator, object, rtti, .binary) };
        defer z.deinit(allocator);

        for (formats) |f|
            try expectSame(ctx, if (f == .text) "default, text" else "default, binary", z.get(f), jolt.get(f));
    }
    try std.testing.expect(compared > 0);
}

// P3 ----------------------------------------------------------------------------------------------------------------

/// The classes that the generators use as the main object: the creatable classes (all ready, P1)
fn mainClasses(schema: *const Schema, list: []usize) ![]usize {
    var n: usize = 0;
    for (schema.classes, 0..) |*c, i| {
        if (c.creatable()) {
            _ = try readyClass(.{ .layer = "P3/P4", .class = c.name }, c);
            list[n] = i;
            n += 1;
        }
    }
    return list[0..n];
}

/// The Zolt class of `c`. A class that is not ready is a mismatch: Phase 8 is done (P1 says why).
fn readyClass(ctx: Context, c: *const Class) error{ParityMismatch}!*const RTTI {
    if (!c.ready()) {
        ctx.report("the class is not ready: it is missing in the Zolt factory, pending or reaches a pending class (see P1)");
        return error.ParityMismatch;
    }
    return c.zolt.?;
}

const streams_per_class = 12;

test "ObjectStream P3: random streams from the C++ schema, both formats, both directions" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var buffer: [512]usize = undefined;
    const classes = try mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    var failed: std.ArrayList([]const u8) = .empty;
    defer failed.deinit(allocator);
    for (classes) |class| {
        // A mismatch is printed; the other classes are still compared
        randomStreams(allocator, schema, class) catch |err| switch (err) {
            error.ParityMismatch => try failed.append(allocator, schema.classes[class].name),
            else => |e| return e,
        };
    }
    printNames("P3: classes with a mismatch", failed.items);
    try std.testing.expectEqual(@as(usize, 0), failed.items.len);
}

/// P3 of one main class
fn randomStreams(allocator: Allocator, schema: *const Schema, class: usize) !void {
    const c = &schema.classes[class];
    for (0..streams_per_class) |s| {
        for (formats) |f| {
            const seed = (@as(u64, c.hash) << 16) ^ (s * 2 + @intFromEnum(f));
            var prng: std.Random.DefaultPrng = .init(seed);
            var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
            defer arena.deinit();
            const g = try gen.Generator.init(arena.allocator(), schema, prng.random(), class);
            const bytes = try gen.write(arena.allocator(), g.doc, f, prng.random());
            const ctx: Context = .{ .layer = "P3", .class = c.name, .detail = @tagName(f), .seed = seed };
            if (!try roundTrip(allocator, bytes, g.doc.request, ctx, .full)) {
                var failed = ctx;
                failed.stream = bytes;
                failed.report("neither library reads a generated stream (a generator error)");
                return error.ParityMismatch;
            }
        }
    }
}

// P4 ----------------------------------------------------------------------------------------------------------------

const MutationCounts = struct {
    applied: std.EnumArray(gen.Generator.Mutation, usize) = .initFill(0),
    read: std.EnumArray(gen.Generator.Mutation, usize) = .initFill(0),
};

/// P4 of one main class
fn mutatedStreams(allocator: Allocator, schema: *const Schema, class: usize, counts: *MutationCounts) !void {
    const c = &schema.classes[class];
    for (std.enums.values(gen.Generator.Mutation)) |mutation| {
        for (0..2) |s| {
            for (formats) |f| {
                // An object of an undeclared class ends the read of a text stream only (binary misreads the rest)
                if (mutation == .undeclared_object and f == .binary) continue;
                const seed = (@as(u64, c.hash) << 24) ^ (@as(u64, @intFromEnum(mutation)) << 8) ^ (s * 2 + @intFromEnum(f));
                var prng: std.Random.DefaultPrng = .init(seed);
                const r = prng.random();
                var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
                defer arena.deinit();
                var g = try gen.Generator.init(arena.allocator(), schema, r, class);
                if (!try g.mutate(mutation)) continue;
                counts.applied.getPtr(mutation).* += 1;
                // Sometimes more changes on top
                for (0..r.uintLessThan(usize, 3)) |_| {
                    const other = r.enumValue(gen.Generator.Mutation);
                    if (other == .undeclared_object and f == .binary) continue;
                    _ = try g.mutate(other);
                }
                try g.finish();
                const bytes = try gen.write(arena.allocator(), g.doc, f, r);
                const ctx: Context = .{ .layer = "P4", .class = c.name, .detail = @tagName(mutation), .seed = seed };
                if (try roundTrip(allocator, bytes, g.doc.request, ctx, .full)) counts.read.getPtr(mutation).* += 1;
            }
        }
    }
}

test "ObjectStream P4: old schemas and odd streams (mutations of random streams)" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var buffer: [512]usize = undefined;
    const classes = try mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    // Per mutation: how often it was applied, and how often both libraries read the stream
    var counts: MutationCounts = .{};
    var failed: std.ArrayList([]const u8) = .empty;
    defer failed.deinit(allocator);
    for (classes) |class| {
        // A mismatch is printed; the other classes are still compared
        mutatedStreams(allocator, schema, class, &counts) catch |err| switch (err) {
            error.ParityMismatch => try failed.append(allocator, schema.classes[class].name),
            else => |e| return e,
        };
    }
    printNames("P4: classes with a mismatch", failed.items);
    try std.testing.expectEqual(@as(usize, 0), failed.items.len);
    const applied = counts.applied;
    const read = counts.read;

    // The test must be able to fail. A generator error could make every mutated stream unreadable, and both sides would
    // agree. So a mutation that keeps the main object readable must be read at least once, and a main object of the
    // wrong class must make a read fail at least once. (extra_object, undeclared_object and main_null_identifier can put
    // an object of another class first, so they may fail every time.)
    var total: usize = 0;
    var weak: usize = 0;
    for (std.enums.values(gen.Generator.Mutation)) |mutation| {
        const n = applied.get(mutation);
        total += n;
        if (n == 0) continue;
        const bites = switch (mutation) {
            .extra_object, .undeclared_object, .main_null_identifier => true,
            .wrong_main_class => read.get(mutation) < n,
            else => read.get(mutation) > 0,
        };
        if (!bites) {
            std.debug.print("P4 {s}: applied {d} times, both libraries read {d} of the streams\n", .{ @tagName(mutation), n, read.get(mutation) });
            weak += 1;
        }
    }
    try std.testing.expect(total > 0);
    try std.testing.expectEqual(@as(usize, 0), weak);
}

/// Old and odd streams of the prototype (SerializationArchitecture.md section 10), read as `class`
const pinned_streams = [_]struct { class: [:0]const u8, stream: []const u8 }{
    // The old schema of Assets/Human.tof: "mFrequency" / "mDamping" are aliases of the spring settings, no mode
    .{ .class = "MotorSettings", .stream = "TOS 1.00\n\ndeclare MotorSettings 6\n  mFrequency float\n  mDamping float\n  mMinForceLimit float\n  mMaxForceLimit float\n  mMinTorqueLimit float\n  mMaxTorqueLimit float\n\nobject MotorSettings 00000001\n  20\n  0.899999976\n  -3.40282347e+38\n  3.40282347e+38\n  -100\n  100" },
    // Retyped (skipped), unknown (skipped) and missing (default) attributes, LF line ends
    .{ .class = "MotorSettings", .stream = "TOS 1.00\ndeclare MotorSettings 4\n mSpringMode uint32\n mFrequency double\n mFoo vec3\n mMaxTorqueLimit float\nobject MotorSettings 00000001\n 1\n 2.5\n 1 2 3\n 7.5" },
    // Number words that libc++ reads partly: "2xyz" is 2, "-0" is -0, an identifier with a 0x prefix
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 2 mSpringMode uint32 mMinForceLimit float object MotorSettings 0x1 2xyz -0" },
    // A word that is not a data type after the object ends the read: success
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 0.25 garbage" },
    // A float that libc++ does not read: the object is dropped, the read fails (no main object)
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 1.5abc" },
    // A subnormal float (ERANGE): fails like the above
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 1e-40" },
    // Another version
    .{ .class = "MotorSettings", .stream = "TOS 1.01 declare MotorSettings 1 mDamping float object MotorSettings 00000001 0.25" },
    // Not an object stream
    .{ .class = "MotorSettings", .stream = "XOS 1.00" },
    // An object without an identifier is skipped, the next one is the main object
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000000 0.25 object MotorSettings 00000002 0.75" },
    // A class that is declared twice appends its attributes
    .{ .class = "MotorSettings", .stream = "TOS 1.00 declare MotorSettings 1 mDamping float declare MotorSettings 1 mFrequency float object MotorSettings 00000001 0.25 4" },
};

test "ObjectStream P4: the old and odd streams of the prototype" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var compared: usize = 0;
    for (pinned_streams) |p| {
        const c = try schema.get(p.class);
        const ctx: Context = .{ .layer = "P4 pinned", .class = p.class };
        _ = try readyClass(ctx, c);
        compared += 1;
        _ = try roundTrip(allocator, p.stream, p.class, ctx, .full);
    }
    try std.testing.expect(compared > 0);
}

// P5 ----------------------------------------------------------------------------------------------------------------

/// Every prefix of `bytes`: the same result and the same bytes written again
fn everyPrefix(allocator: Allocator, bytes: []const u8, class: [:0]const u8, ctx: Context) !void {
    for (0..bytes.len + 1) |n| {
        var c = ctx;
        c.seed = n;
        _ = try roundTrip(allocator, bytes[0..n], class, c, .first);
    }
}

test "ObjectStream P5: every prefix of the default and random streams" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var buffer: [512]usize = undefined;
    const classes = try mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    for (classes) |class| {
        const c = &schema.classes[class];

        // The defaults (C++ bytes)
        try std.testing.expectEqual(@as(c_int, 0), try schema_mod.quarantined(schema_mod.jolt_os_default_write(c.name)));
        const defaults: Resaved = .{ .text = try allocator.dupe(u8, cppResult(0)), .binary = try allocator.dupe(u8, cppResult(1)) };
        defer defaults.deinit(allocator);
        for (formats) |f| {
            if (defaults.get(f).len > 64 * 1024) continue;
            try everyPrefix(allocator, defaults.get(f), c.name, .{ .layer = "P5 default", .class = c.name, .detail = @tagName(f) });
        }

        // A random stream in each format
        for (formats) |f| {
            const seed = @as(u64, c.hash) ^ 0x5005 ^ @intFromEnum(f);
            var prng: std.Random.DefaultPrng = .init(seed);
            var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
            defer arena.deinit();
            const g = try gen.Generator.init(arena.allocator(), schema, prng.random(), class);
            const bytes = try gen.write(arena.allocator(), g.doc, f, prng.random());
            if (bytes.len > 64 * 1024) continue;
            try everyPrefix(allocator, bytes, g.doc.request, .{ .layer = "P5 random", .class = c.name, .detail = @tagName(f) });
        }
    }
}

// P7 (and the terrain2.bof part of P5) ------------------------------------------------------------------------------

const Asset = struct { path: []const u8, class: [:0]const u8 };

const assets = [_]Asset{
    .{ .path = "Assets/Human.tof", .class = "RagdollSettings" },
    .{ .path = "Assets/Human/dead_pose1.tof", .class = "SkeletalAnimation" },
    .{ .path = "Assets/terrain2.bof", .class = "PhysicsScene" },
};

/// The bytes of an asset. Every class that the asset declares must be ready in Zolt (a mismatch otherwise).
fn loadAsset(allocator: Allocator, asset: Asset) ![]u8 {
    const schema = try schema_mod.get();
    const bytes = try std.Io.Dir.cwd().readFileAlloc(std.testing.io, asset.path, allocator, .limited(64 << 20));
    errdefer allocator.free(bytes);

    // The classes that the C++ side writes for the asset (its declarations)
    const jolt = (try cppResave(allocator, bytes, asset.class)) orelse {
        std.debug.print("{s}: the C++ library cannot read the asset\n", .{asset.path});
        return error.HarnessError;
    };
    defer jolt.deinit(allocator);
    var arena: std.heap.ArenaAllocator = .init(allocator);
    defer arena.deinit();
    for (try declaredClasses(arena.allocator(), jolt.text)) |name| {
        _ = try readyClass(.{ .layer = "P7", .class = name, .detail = asset.path }, try schema.get(name));
    }
    return bytes;
}

/// SaveBinaryState of the object that Zolt reads from an asset
fn zoltBinaryState(allocator: Allocator, bytes: []const u8, class: []const u8) ![]u8 {
    const rtti = schema_mod.factory.find(class) orelse return error.HarnessError;
    var reader: std.Io.Reader = .fixed(bytes);
    var in = try ObjectStreamIn.open(allocator, &reader, schema_mod.factory);
    defer in.deinit();
    const main = try in.read(rtti);
    defer main.rtti.destructObject(main.instance, allocator);
    inline for (.{ zolt.RagdollSettings, zolt.SkeletalAnimation, zolt.PhysicsScene }) |T| {
        if (main.rtti == T.rtti) {
            const object: *T = @ptrCast(@alignCast(main.instance));
            var out: std.Io.Writer.Allocating = .init(allocator);
            errdefer out.deinit();
            var wrapper = zolt.StreamOutWrapper.init(&out.writer);
            const Save = @TypeOf(T.saveBinaryState);
            const result = if (@typeInfo(Save).@"fn".params.len == 2)
                object.saveBinaryState(wrapper.streamOut())
            else
                object.saveBinaryState(wrapper.streamOut(), true, true);
            if (@typeInfo(@TypeOf(result)) == .error_union) try result;
            return try out.toOwnedSlice();
        }
    }
    std.debug.print("{s}: no SaveBinaryState for the main object of class {s}\n", .{ class, main.rtti.name });
    return error.HarnessError;
}

fn assetTest(comptime index: usize) !void {
    const allocator = std.testing.allocator;
    const asset = assets[index];
    const bytes = try loadAsset(allocator, asset);
    defer allocator.free(bytes);
    const ctx: Context = .{ .layer = "P7", .class = asset.class, .detail = asset.path };
    if (!try roundTrip(allocator, bytes, asset.class, ctx, .full)) {
        ctx.report("neither library reads the asset");
        return error.ParityMismatch;
    }

    // SaveBinaryState of the loaded object
    const zolt_state = try zoltBinaryState(allocator, bytes, asset.class);
    defer allocator.free(zolt_state);
    try std.testing.expectEqual(@as(c_int, 0), try schema_mod.quarantined(schema_mod.jolt_os_asset_binary_state(bytes.ptr, bytes.len, asset.class)));
    try expectSame(ctx, "SaveBinaryState", zolt_state, cppResult(2));
}

test "ObjectStream P7: Assets/Human.tof" {
    try assetTest(0);
}

test "ObjectStream P7: Assets/Human/dead_pose1.tof" {
    try assetTest(1);
}

test "ObjectStream P7: Assets/terrain2.bof" {
    try assetTest(2);
}

// P7 and P9 of random PhysicsScenes with soft bodies ------------------------------------------------------------------

/// True when SaveBinaryState can write the scene without its shapes: every constraint has settings (a path constraint
/// also its path) and every soft body has shared settings. Jolt dereferences a null pointer otherwise (Zolt panics).
fn saveableScene(scene: *const zolt.PhysicsScene) bool {
    for (scene.getConstraints()) |*cc| {
        const settings = cc.getSettings() orelse return false;
        if (settings.base.vtable.rtti == zolt.PathConstraintSettings.rtti) {
            if (zolt.virtual.downcast(zolt.PathConstraintSettings, settings).path.get() == null) return false;
        }
    }
    for (scene.getSoftBodies()) |*b| {
        if (b.settings.get() == null) return false;
    }
    return true;
}

/// The random scenes of P7 / P9 with soft bodies, and how many of them must have something
const scene_streams = 240;

test "ObjectStream P7 / P9: the binary state of random PhysicsScenes with soft bodies" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    const class = schema.find("PhysicsScene") orelse return error.HarnessError;
    _ = try readyClass(.{ .layer = "P7 scene", .class = "PhysicsScene" }, &schema.classes[class]);
    var compared: usize = 0;
    var num_soft_bodies: usize = 0;
    var num_shared_settings_with_materials: usize = 0;
    var num_shared_settings_reused: usize = 0;
    for (0..scene_streams) |s| {
        const f: gen.Format = if (s % 2 == 0) .text else .binary;
        const seed = 0x5c3e_7009 ^ (@as(u64, s) << 8);
        var prng: std.Random.DefaultPrng = .init(seed);
        var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
        defer arena.deinit();
        const g = try gen.Generator.init(arena.allocator(), schema, prng.random(), class);
        const bytes = try gen.write(arena.allocator(), g.doc, f, prng.random());
        var ctx: Context = .{ .layer = "P7 scene", .class = "PhysicsScene", .detail = @tagName(f), .seed = seed, .stream = bytes };

        // Both libraries read the stream and write the same bytes (P3)
        if (!try roundTrip(allocator, bytes, g.doc.request, ctx, .first)) {
            ctx.report("neither library reads a generated stream (a generator error)");
            return error.ParityMismatch;
        }

        // The scene that Zolt reads
        var reader: std.Io.Reader = .fixed(bytes);
        var in = try ObjectStreamIn.open(allocator, &reader, schema_mod.factory);
        defer in.deinit();
        const main = try in.read(zolt.PhysicsScene.rtti);
        defer main.rtti.destructObject(main.instance, allocator);
        const scene: *const zolt.PhysicsScene = @ptrCast(@alignCast(main.instance));
        if (!saveableScene(scene)) continue;
        compared += 1;
        num_soft_bodies += scene.getNumSoftBodies();
        for (scene.getSoftBodies(), 0..) |*b, i| {
            const shared = b.settings.get().?;
            for (shared.materials.items) |m| {
                if (m.get() != null) {
                    num_shared_settings_with_materials += 1;
                    break;
                }
            }
            for (scene.getSoftBodies()[0..i]) |*other| {
                if (other.settings.get().? == shared) {
                    num_shared_settings_reused += 1;
                    break;
                }
            }
        }

        // P7: SaveBinaryState of the loaded scene (without the shapes: GetShape would create shapes from random
        // settings), with and without the group filters
        const save_group_filter = prng.random().boolean();
        var zolt_state: std.Io.Writer.Allocating = .init(allocator);
        defer zolt_state.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&zolt_state.writer);
        try scene.saveBinaryState(wrapper.streamOut(), false, save_group_filter);
        try std.testing.expectEqual(@as(c_int, 0), try schema_mod.quarantined(schema_mod.jolt_os_scene_binary_state(bytes.ptr, bytes.len, 0, @intFromBool(save_group_filter))));
        try expectSame(ctx, "SaveBinaryState", zolt_state.written(), cppResult(2));

        // P9: each side restores the other side's binary state and saves it again
        ctx.layer = "P9 scene";
        ctx.stream = zolt_state.written();
        const jolt_state = try allocator.dupe(u8, cppResult(2));
        defer allocator.free(jolt_state);
        const jolt_result = try schema_mod.quarantined(schema_mod.jolt_os_scene_binary_resave(zolt_state.written().ptr, zolt_state.written().len, 0, @intFromBool(save_group_filter)));
        if (jolt_result != 0) {
            ctx.report("Jolt cannot restore Zolt's binary state");
            std.debug.print("  {s}\n", .{cppResult(2)});
            return error.ParityMismatch;
        }
        try expectSame(ctx, "Jolt restores Zolt's binary state and saves it again", cppResult(2), zolt_state.written());
        var state_reader: std.Io.Reader = .fixed(jolt_state);
        var state_in = zolt.StreamInWrapper.init(&state_reader);
        var restored = try zolt.PhysicsScene.restoreFromBinaryState(allocator, state_in.streamIn());
        defer restored.deinit();
        if (restored.hasError()) {
            ctx.report("Zolt cannot restore Jolt's binary state");
            std.debug.print("  {s}\n", .{restored.getError()});
            return error.ParityMismatch;
        }
        var resaved: std.Io.Writer.Allocating = .init(allocator);
        defer resaved.deinit();
        var resaved_wrapper = zolt.StreamOutWrapper.init(&resaved.writer);
        try restored.getPtr().?.saveBinaryState(resaved_wrapper.streamOut(), false, save_group_filter);
        try expectSame(ctx, "Zolt restores Jolt's binary state and saves it again", resaved.written(), jolt_state);
    }

    // The layer must reach soft bodies, their materials and shared settings that soft bodies share (seen: 86 of 240
    // scenes compared, 84 soft bodies, 54 with materials, 18 that share their shared settings)
    try std.testing.expect(compared >= scene_streams / 5);
    try std.testing.expect(num_soft_bodies >= scene_streams / 4);
    try std.testing.expect(num_shared_settings_with_materials >= 20);
    try std.testing.expect(num_shared_settings_reused >= 5);
}

test "ObjectStream P5: 2000 prefixes of Assets/terrain2.bof" {
    const allocator = std.testing.allocator;
    const asset = assets[2];
    const bytes = try loadAsset(allocator, asset);
    defer allocator.free(bytes);
    // The first 1500 prefixes (the declarations and the first objects), then 500 spread over the whole file
    for (0..2000) |i| {
        const n = if (i < 1500) i else 1500 + (i - 1500) * (bytes.len - 1500) / 500;
        _ = try roundTrip(allocator, bytes[0..n], asset.class, .{ .layer = "P5", .class = asset.class, .detail = asset.path, .seed = n }, .first);
    }
}

// P6 ----------------------------------------------------------------------------------------------------------------

/// Zolt reads `word` as `word_type` with its text stream: the value bits, null when the word is not read
fn zoltReadWord(allocator: Allocator, word: []const u8, word_type: number_words.WordType) !?u64 {
    const bytes = try std.mem.concat(allocator, u8, &.{ "TOS 1.00", word });
    defer allocator.free(bytes);
    var reader: std.Io.Reader = .fixed(bytes);
    var in = try ObjectStreamIn.open(allocator, &reader, schema_mod.factory);
    defer in.deinit();
    return readWordAs(&in, word_type) catch |err| switch (err) {
        error.InvalidObjectStream => null,
        else => |e| e,
    };
}

fn readWordAs(in: *ObjectStreamIn, word_type: number_words.WordType) ObjectStreamIn.Error!u64 {
    switch (word_type) {
        .identifier => return try in.readIdentifier(),
        .count => return try in.readCount(),
        inline else => |t| {
            const T = switch (t) {
                .uint8 => u8,
                .uint16 => u16,
                .int => i32,
                .uint32 => u32,
                .uint64 => u64,
                .float => f32,
                .double => f64,
                .bool => bool,
                else => unreachable,
            };
            var value: T = std.mem.zeroes(T);
            try in.readPrimitive(T, &value);
            return switch (T) {
                bool => @intFromBool(value),
                i32 => @as(u32, @bitCast(value)),
                f32 => @as(u32, @bitCast(value)),
                f64 => @bitCast(value),
                else => value,
            };
        },
    }
}

test "ObjectStream P6: number words read like C++ IStringStream >> T and strtoul" {
    const allocator = std.testing.allocator;
    var arena: std.heap.ArenaAllocator = .init(std.heap.page_allocator);
    defer arena.deinit();
    var prng: std.Random.DefaultPrng = .init(0x6006);
    var total: usize = 0;
    var mismatches: usize = 0;
    for (std.enums.values(number_words.WordType)) |word_type| {
        const words = try number_words.generate(arena.allocator(), prng.random(), word_type, 11_000);
        for (words) |word| {
            total += 1;
            var bits: u64 = 0;
            const jolt_read = schema_mod.jolt_os_read_word(word.ptr, word.len, @intFromEnum(word_type), &bits);
            try std.testing.expect(jolt_read >= 0);
            const zolt_bits = try zoltReadWord(allocator, word, word_type);
            const same = if (jolt_read == 1) (zolt_bits != null and zolt_bits.? == bits) else zolt_bits == null;
            if (!same) {
                if (mismatches < 50) {
                    std.debug.print("P6 {s} \"{f}\": jolt {s} {x}, zolt ", .{ @tagName(word_type), std.zig.fmtString(word), if (jolt_read == 1) "reads" else "fails", bits });
                    if (zolt_bits) |z| std.debug.print("reads {x}\n", .{z}) else std.debug.print("fails\n", .{});
                }
                mismatches += 1;
            }
        }
    }
    try std.testing.expect(total > 100_000);
    try std.testing.expectEqual(@as(usize, 0), mismatches);
}

// P8 ----------------------------------------------------------------------------------------------------------------

/// The factory of P8 (Jolt: RegisterTypes and the classes that the C++ side registers in EnsureFactory)
const unit_test_factory: Factory = .init(&.{ zolt.MotorSettings, zolt.SpringSettings, ost.TestSerializable });

fn zoltWriteTyped(comptime T: type, allocator: Allocator, object: *const T, stream_type: StreamType) ![]u8 {
    var stream: std.Io.Writer.Allocating = .init(allocator);
    errdefer stream.deinit();
    try ObjectStreamOut.sWriteObject(T, allocator, &stream.writer, stream_type, object);
    return stream.toOwnedSlice();
}

test "ObjectStream P8: the object of ObjectStreamTest.cpp, bytes and reading in both directions" {
    const allocator = std.testing.allocator;
    const ctx: Context = .{ .layer = "P8", .class = "TestSerializable" };
    const test_object = try ost.createTestObject(allocator);
    defer test_object.destroy();
    for ([_]StreamType{ .text, .binary }, 0..) |stream_type, binary| {
        // Zolt writes Jolt's bytes
        try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_unit_test_write(@intCast(binary)));
        const jolt_bytes = try allocator.dupe(u8, cppResult(0));
        defer allocator.free(jolt_bytes);
        const zolt_bytes = try zoltWriteTyped(ost.TestSerializable, allocator, test_object, stream_type);
        defer allocator.free(zolt_bytes);
        try expectSame(ctx, "write", zolt_bytes, jolt_bytes);

        // Zolt reads Jolt's bytes: the casts, the shared object with 2 references, the bytes written again
        var reader: std.Io.Reader = .fixed(jolt_bytes);
        const read = try ObjectStreamIn.sReadObject(ost.TestSerializable, allocator, &reader, .{ .factory = &unit_test_factory });
        defer read.destroy();
        try std.testing.expectEqual(@as(?*ost.TestSerializableBase2, &read.base2), RTTI.dynamicCast(ost.TestSerializableBase2, ost.TestSerializable, read));
        try std.testing.expectEqual(@as(u32, 2), read.reference.ptr.?.getRefCount());
        for ([_]StreamType{ .text, .binary }, 0..) |other_type, other_binary| {
            var ref_count: u32 = 0;
            var casts_ok: c_int = 0;
            const m = schema_mod.jolt_os_unit_test_resave(jolt_bytes.ptr, jolt_bytes.len, @intCast(other_binary), &ref_count, &casts_ok);
            try std.testing.expect(m == 0 and ref_count == 2 and casts_ok == 1);
            const rewritten = try zoltWriteTyped(ost.TestSerializable, allocator, read, other_type);
            defer allocator.free(rewritten);
            try expectSame(ctx, "read and write again", rewritten, cppResult(0));
        }

        // Jolt reads Zolt's bytes
        var ref_count: u32 = 0;
        var casts_ok: c_int = 0;
        const m = schema_mod.jolt_os_unit_test_resave(zolt_bytes.ptr, zolt_bytes.len, @intCast(binary), &ref_count, &casts_ok);
        try std.testing.expect(m == 0 and ref_count == 2 and casts_ok == 1);
        try expectSame(ctx, "Jolt reads Zolt's bytes", cppResult(0), zolt_bytes);
    }
}
