//! The generic serialization parity harness (Docs/Zolt/SerializationArchitecture.md section 5): Zolt's RTTI, Factory
//! and ObjectStream against the C++ library built with JPH_OBJECT_STREAM (build.zig). C ABI wrappers:
//! ObjectStreamReference.cpp. The class model is ObjectStreamSchema.zig, the stream generator StreamGenerator.zig, the
//! number words NumberWords.zig.
//!
//! The harness has no code for one class. It takes every class of the C++ Factory after RegisterTypes and the Zolt
//! factory (`RegisterTypes.factory`; a stand-in until the migration adds it, see ObjectStreamSchema.zig). A layer skips
//! a class that is pending (`serialization_pending`, S18) or that reaches a pending class, and picks the class up as
//! soon as its group removes the marker.
//!
//! - P1 schema: every class against the Zolt factory (name, hash, abstract flag, bases, attributes with their declared
//!   types and primitive types); a pending class by name, hash, abstract flag and bases only. The Phase 9 classes are
//!   excluded (S17). The IsKindOf matrix.
//! - P2 defaults: every creatable class, C++ CreateObject and write against Zolt create and write, text and binary.
//! - P3 random streams from the C++ schema, both formats; each side reads the other's output.
//! - P4 old schemas: mutations of P3 streams (dropped, reordered, renamed and retyped attributes, vec3 / dvec3, a class
//!   declared twice, unknown and undeclared classes, abstract objects, null, dangling and wrong type identifiers, a main
//!   object of the wrong class), and the old MotorSettings streams of the prototype.
//! - P5 truncation: every prefix of the default and random streams (up to 64 KB), 2000 prefixes of terrain2.bof.
//! - P6 number words: more than 10^5 words through C++'s ObjectStreamTextIn and Zolt's.
//! - P7 assets: Human.tof, dead_pose1.tof and terrain2.bof read by both sides, written again in both formats, and the
//!   SaveBinaryState bytes of the loaded objects. Skipped while a class they need is pending.
//! - P8 the object of UnitTests/ObjectStream/ObjectStreamTest.cpp in both formats and both directions.
//!
//! Comparing bytes: both sides read a stream as the requested class and write the main object in both formats; the
//! result (read or not) and the bytes must be equal. A class that reaches a Phase 9 class (PhysicsScene.mSoftBodies,
//! S17) is compared by cross-reading: the C++ side reads Zolt's bytes and must write its own bytes again.
//!
//! The C++ side of P2 to P7 reads with the Factory after RegisterTypes and with allocation hooks that zero every block
//! and defer frees until the call returns. Every C++ read starts with errno = 0 (D7).

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

const log = std.log.scoped(.zolt_parity);

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
    const result = schema_mod.jolt_os_resave(bytes.ptr, bytes.len, class);
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

/// True when the C++ bytes declare a Phase 9 class: Zolt's cannot (S17), they are compared by cross-reading
fn declaresPhase9(text: []const u8) bool {
    var words = std.mem.tokenizeAny(u8, text, " \r\n\t");
    while (words.next()) |w| {
        if (std.mem.eql(u8, w, "declare")) {
            if (words.next()) |name| {
                if (schema_mod.isPhase9(name)) return true;
            }
        }
    }
    return false;
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

    // A class that reaches a Phase 9 class: the C++ bytes have attributes that Zolt's cannot have (S17). Then the C++
    // side reads Zolt's bytes and must write its own bytes again.
    const cross = declaresPhase9(j.text);
    for (formats) |f| {
        if (!cross) {
            try expectSame(ctx, if (f == .text) "written again as text" else "written again as binary", zz.get(f), j.get(f));
        } else {
            const c2 = try mustResave(allocator, .jolt, zz.get(f), class, ctx);
            defer c2.deinit(allocator);
            try expectSame(ctx, "Jolt reads Zolt's bytes", c2.get(f), j.get(f));
        }
    }
    if (depth == .first) return true;

    // Each side reads the bytes that the other wrote, and both write the same bytes again. (Not always the bytes that
    // were read: a binary stream cuts a string at its first NUL, S15.)
    for (formats) |f| {
        const c2 = try mustResave(allocator, .jolt, zz.get(f), class, ctx);
        defer c2.deinit(allocator);
        const z2 = try mustResave(allocator, .zolt, j.get(f), class, ctx);
        defer z2.deinit(allocator);
        for (formats) |g| {
            if (!cross) {
                try expectSame(ctx, "each side reads the other's bytes", z2.get(g), c2.get(g));
                continue;
            }
            // Cross-read: each side writes the same bytes again for both inputs
            const c3 = try mustResave(allocator, .jolt, j.get(f), class, ctx);
            defer c3.deinit(allocator);
            try expectSame(ctx, "Jolt reads Zolt's bytes", c2.get(g), c3.get(g));
            const z3 = try mustResave(allocator, .zolt, zz.get(f), class, ctx);
            defer z3.deinit(allocator);
            try expectSame(ctx, "Zolt reads Jolt's bytes", z2.get(g), z3.get(g));
        }
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

/// The C++ schema line without the attributes whose class is a Phase 9 class (PhysicsScene.mSoftBodies, S17)
fn expectedSchema(allocator: Allocator, c: *const Class) ![]u8 {
    var line: std.Io.Writer.Allocating = .init(allocator);
    errdefer line.deinit();
    const header_end = std.mem.indexOf(u8, c.line, " attributes=[").?;
    try line.writer.print("{s} attributes=[", .{c.line[0..header_end]});
    var first = true;
    for (c.attributes) |a| {
        if (a.primitive) |p| {
            if (schema_mod.isPhase9(p)) continue;
        }
        try line.writer.print("{s}{s}:{s}({s})", .{ if (first) "" else ";", a.name, a.type_text, a.primitive orelse "-" });
        first = false;
    }
    try line.writer.writeAll("]");
    return line.toOwnedSlice();
}

fn schemaHeader(line: []const u8) []const u8 {
    return line[0 .. std.mem.indexOf(u8, line, " attributes=[") orelse line.len];
}

test "ObjectStream P1: the schema of every class of the C++ Factory after RegisterTypes, and IsKindOf" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var mismatches: usize = 0;
    var compared: usize = 0;
    for (schema.classes) |*c| {
        if (c.phase9) continue;
        const rtti = c.zolt orelse {
            // Before the migration (F2) a class without a declaration is not in the stand-in factory
            if (schema_mod.has_register_types_factory) {
                std.debug.print("{s} is not in RegisterTypes.factory\n", .{c.name});
                mismatches += 1;
            } else log.info("P1: {s} is not declared yet", .{c.name});
            continue;
        };
        compared += 1;
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
        const jolt_line = try expectedSchema(allocator, c);
        defer allocator.free(jolt_line);
        // A pending class (or a class with a pending base) has no attributes yet: name, hash, kind and bases only
        const pending = schema_mod.hasPendingBase(rtti);
        if (pending) log.info("P1: {s} is pending: name, hash, kind and bases only", .{c.name});
        const same = if (pending) std.mem.eql(u8, schemaHeader(zolt_line), schemaHeader(jolt_line)) else std.mem.eql(u8, zolt_line, jolt_line);
        if (!same) {
            std.debug.print("schema of {s}{s}\n  zolt: {s}\n  jolt: {s}\n", .{ c.name, if (pending) " (pending)" else "", zolt_line, jolt_line });
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
        if (c.abstract or c.phase9 or c.outside or !c.ready()) continue;
        const rtti = c.zolt.?;
        const ctx: Context = .{ .layer = "P2", .class = c.name };
        compared += 1;

        try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_default_write(c.name));
        const jolt: Resaved = .{ .text = try allocator.dupe(u8, cppResult(0)), .binary = try allocator.dupe(u8, cppResult(1)) };
        defer jolt.deinit(allocator);

        const object = (try rtti.createObject(allocator)).?;
        defer rtti.destructObject(object, allocator);
        const z: Resaved = .{ .text = try zoltWrite(allocator, object, rtti, .text), .binary = try zoltWrite(allocator, object, rtti, .binary) };
        defer z.deinit(allocator);

        for (formats) |f| {
            if (!c.cross) {
                try expectSame(ctx, if (f == .text) "default, text" else "default, binary", z.get(f), jolt.get(f));
                continue;
            }
            // Cross-read: Jolt reads Zolt's default and writes its own default again
            const c2 = (try cppResave(allocator, z.get(f), c.name)) orelse {
                ctx.report("Jolt cannot read Zolt's default");
                return error.ParityMismatch;
            };
            defer c2.deinit(allocator);
            try expectSame(ctx, "Jolt reads Zolt's default", c2.get(f), jolt.get(f));
        }
    }
    try std.testing.expect(compared > 0);
}

// P3 ----------------------------------------------------------------------------------------------------------------

/// The classes that the generators use as the main object: creatable and ready
fn mainClasses(schema: *const Schema, list: []usize) []usize {
    var n: usize = 0;
    for (schema.classes, 0..) |*c, i| {
        if (c.creatable() and c.ready()) {
            list[n] = i;
            n += 1;
        }
    }
    return list[0..n];
}

const streams_per_class = 12;

test "ObjectStream P3: random streams from the C++ schema, both formats, both directions" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var buffer: [512]usize = undefined;
    const classes = mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    for (classes) |class| {
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
}

// P4 ----------------------------------------------------------------------------------------------------------------

test "ObjectStream P4: old schemas and odd streams (mutations of random streams)" {
    const allocator = std.testing.allocator;
    const schema = try schema_mod.get();
    var buffer: [512]usize = undefined;
    const classes = mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    var applied: usize = 0;
    for (classes) |class| {
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
                    applied += 1;
                    // Sometimes more changes on top
                    for (0..r.uintLessThan(usize, 3)) |_| {
                        const other = r.enumValue(gen.Generator.Mutation);
                        if (other == .undeclared_object and f == .binary) continue;
                        _ = try g.mutate(other);
                    }
                    try g.finish();
                    const bytes = try gen.write(arena.allocator(), g.doc, f, r);
                    const ctx: Context = .{ .layer = "P4", .class = c.name, .detail = @tagName(mutation), .seed = seed };
                    _ = try roundTrip(allocator, bytes, g.doc.request, ctx, .full);
                }
            }
        }
    }
    try std.testing.expect(applied > 0);
}

/// Old and odd streams of the prototype (SerializationArchitecture.md section 10), read as `class` when it is ready
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
        if (!c.ready()) continue;
        compared += 1;
        _ = try roundTrip(allocator, p.stream, p.class, .{ .layer = "P4 pinned", .class = p.class }, .full);
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
    const classes = mainClasses(schema, &buffer);
    try std.testing.expect(classes.len > 0);
    for (classes) |class| {
        const c = &schema.classes[class];

        // The defaults (C++ bytes)
        try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_default_write(c.name));
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

/// The bytes of an asset, null while a class that the asset declares is pending or not declared in Zolt (the test is
/// then skipped: it starts comparing as soon as the groups land)
fn loadAsset(allocator: Allocator, asset: Asset) !?[]u8 {
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
    var waiting = false;
    for (try declaredClasses(arena.allocator(), jolt.text)) |name| {
        if (schema_mod.isPhase9(name)) continue;
        const c = try schema.get(name);
        if (!c.ready()) {
            log.info("{s}: pending, {s} is not declared or reaches a pending class", .{ asset.path, name });
            waiting = true;
        }
    }
    if (waiting) {
        allocator.free(bytes);
        return null;
    }
    return bytes;
}

/// SaveBinaryState of the object that Zolt reads from an asset (null when the class has no saveBinaryState)
fn zoltBinaryState(allocator: Allocator, bytes: []const u8, class: []const u8) !?[]u8 {
    const rtti = schema_mod.factory.find(class) orelse return error.HarnessError;
    var reader: std.Io.Reader = .fixed(bytes);
    var in = try ObjectStreamIn.open(allocator, &reader, schema_mod.factory);
    defer in.deinit();
    const main = try in.read(rtti);
    defer main.rtti.destructObject(main.instance, allocator);
    inline for (.{ zolt.RagdollSettings, zolt.SkeletalAnimation, zolt.PhysicsScene }) |T| {
        if (comptime schema_mod.isSerializable(T)) {
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
    }
    return null;
}

fn assetTest(comptime index: usize) !void {
    const allocator = std.testing.allocator;
    const asset = assets[index];
    const bytes = (try loadAsset(allocator, asset)) orelse return error.SkipZigTest;
    defer allocator.free(bytes);
    const ctx: Context = .{ .layer = "P7", .class = asset.class, .detail = asset.path };
    if (!try roundTrip(allocator, bytes, asset.class, ctx, .full)) {
        ctx.report("neither library reads the asset");
        return error.ParityMismatch;
    }

    // SaveBinaryState of the loaded object
    if (try zoltBinaryState(allocator, bytes, asset.class)) |zolt_state| {
        defer allocator.free(zolt_state);
        try std.testing.expectEqual(@as(c_int, 0), schema_mod.jolt_os_asset_binary_state(bytes.ptr, bytes.len, asset.class));
        try expectSame(ctx, "SaveBinaryState", zolt_state, cppResult(2));
    }
}

test "ObjectStream P7: Assets/Human.tof (skipped while RagdollSettings or a class it reaches is pending)" {
    try assetTest(0);
}

test "ObjectStream P7: Assets/Human/dead_pose1.tof (skipped while SkeletalAnimation or a class it reaches is pending)" {
    try assetTest(1);
}

test "ObjectStream P7: Assets/terrain2.bof (skipped while PhysicsScene or a class it reaches is pending)" {
    try assetTest(2);
}

test "ObjectStream P5: 2000 prefixes of Assets/terrain2.bof (skipped while PhysicsScene or a class it reaches is pending)" {
    const allocator = std.testing.allocator;
    const asset = assets[2];
    const bytes = (try loadAsset(allocator, asset)) orelse return error.SkipZigTest;
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
