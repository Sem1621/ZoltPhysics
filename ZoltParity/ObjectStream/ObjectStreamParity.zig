//! Parity tests of the Phase 8 foundation prototype (Zolt/Core/RTTI.zig, Zolt/Core/Factory.zig, Zolt/ObjectStream):
//! the RTTI schemas, the text and binary bytes of ObjectStreamOut and the reading of ObjectStreamIn, compared with the
//! C++ library built with JPH_OBJECT_STREAM (build.zig). C ABI wrappers: ZoltParity/ObjectStream/ObjectStreamReference.cpp.
//! See Docs/Zolt/SerializationArchitecture.md, section 5 (the foundation extends this file to every class).
//!
//! - Schema: name, hash, abstract flag, bases and every attribute (name, the declared type that
//!   ObjectStreamTextOut::WriteDataType writes, the primitive type) of the 19 primitives, the TypeDeclarations
//!   classes, SerializableObject, SpringSettings, MotorSettings and the classes of UnitTests/ObjectStream/
//!   ObjectStreamTest.cpp; the IsKindOf matrix of these classes.
//! - Bytes: MotorSettings with random values and the object of ObjectStreamTest.cpp (all primitives, an enum, Array,
//!   StaticArray, C arrays, nested arrays, a raw pointer, Ref / RefConst to one shared object, two bases with the second
//!   at a non-zero offset): Zolt writes Jolt's text and binary bytes, Zolt reads Jolt's bytes and writes them again
//!   unchanged (in both formats), Jolt reads Zolt's bytes and writes them again unchanged.
//! - Old and odd streams (text): the old MotorSettings schema of Assets/Human.tof (the aliases "mFrequency" /
//!   "mDamping"), missing, unknown and retyped attributes, number words that libc++ reads partly, a garbage word after
//!   the object, another version, a float that does not parse: both libraries give the same result and bytes.

const std = @import("std");
const zolt = @import("zolt");
const ost = @import("objectstream_test");

const Allocator = std.mem.Allocator;
const RTTI = zolt.RTTI;
const Factory = zolt.Factory;
const ObjectStreamIn = zolt.ObjectStreamIn;
const ObjectStreamOut = zolt.ObjectStreamOut;
const StreamType = zolt.ObjectStream.StreamType;
const MotorSettings = zolt.MotorSettings;
const SpringSettings = zolt.SpringSettings;

extern fn jolt_os_class_schema(name: [*:0]const u8, out: [*]u8, capacity: c_int) c_int;
extern fn jolt_os_is_kind_of(name: [*:0]const u8, base: [*:0]const u8) c_int;
extern fn jolt_os_motor_write(desc: *const MotorDesc, binary: c_int, out: [*]u8, capacity: c_int) c_int;
extern fn jolt_os_motor_resave(data: [*]const u8, size: c_int, binary: c_int, out: [*]u8, capacity: c_int, out_desc: *MotorDesc) c_int;
extern fn jolt_os_unit_test_write(binary: c_int, out: [*]u8, capacity: c_int) c_int;
extern fn jolt_os_unit_test_resave(data: [*]const u8, size: c_int, binary: c_int, out: [*]u8, capacity: c_int, out_ref_count: *u32, out_casts_ok: *c_int) c_int;

/// MotorSettings as plain data (ObjectStreamReference.cpp: MotorDesc)
const MotorDesc = extern struct {
    spring_mode: u32,
    frequency: f32,
    damping: f32,
    min_force_limit: f32,
    max_force_limit: f32,
    min_torque_limit: f32,
    max_torque_limit: f32,

    fn toMotor(self: MotorDesc) MotorSettings {
        var m: MotorSettings = .{};
        m.spring_settings.mode = @enumFromInt(@as(u8, @intCast(self.spring_mode)));
        m.spring_settings.frequency = self.frequency;
        m.spring_settings.damping = self.damping;
        m.min_force_limit = self.min_force_limit;
        m.max_force_limit = self.max_force_limit;
        m.min_torque_limit = self.min_torque_limit;
        m.max_torque_limit = self.max_torque_limit;
        return m;
    }

    fn fromMotor(m: *const MotorSettings) MotorDesc {
        return .{
            .spring_mode = @intFromEnum(m.spring_settings.mode),
            .frequency = m.spring_settings.frequency,
            .damping = m.spring_settings.damping,
            .min_force_limit = m.min_force_limit,
            .max_force_limit = m.max_force_limit,
            .min_torque_limit = m.min_torque_limit,
            .max_torque_limit = m.max_torque_limit,
        };
    }
};

/// Every class of the prototype with its Jolt name
const classes = .{
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
    .{ SpringSettings, "SpringSettings" },
    .{ MotorSettings, "MotorSettings" },
    .{ ost.TestSerializableBase, "TestSerializableBase" },
    .{ ost.TestSerializableBase2, "TestSerializableBase2" },
    .{ ost.TestSerializable, "TestSerializable" },
};

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

test "ObjectStream: the RTTI schemas and IsKindOf equal Jolt's" {
    const allocator = std.testing.allocator;
    var buffer: [8192]u8 = undefined;
    var mismatches: usize = 0;
    inline for (classes) |c| {
        const rtti = RTTI.of(c[0]);
        try std.testing.expectEqualStrings(c[1], rtti.name);
        const n = jolt_os_class_schema(c[1], &buffer, buffer.len);
        try std.testing.expect(n >= 0);
        const zolt_line = try zoltSchema(allocator, rtti);
        defer allocator.free(zolt_line);
        if (!std.mem.eql(u8, buffer[0..@intCast(n)], zolt_line)) {
            std.debug.print("schema of {s}\n  zolt: {s}\n  jolt: {s}\n", .{ c[1], zolt_line, buffer[0..@intCast(n)] });
            mismatches += 1;
        }
    }
    // Runtime table, so that the pairs below are a plain loop
    var rttis: [classes.len]*const RTTI = undefined;
    var names: [classes.len][*:0]const u8 = undefined;
    inline for (classes, 0..) |c, i| {
        rttis[i] = RTTI.of(c[0]);
        names[i] = c[1];
    }
    for (rttis, names) |a, a_name| {
        for (rttis, names) |b, b_name| {
            const jolt = jolt_os_is_kind_of(a_name, b_name);
            try std.testing.expect(jolt >= 0);
            if ((jolt == 1) != a.isKindOf(b)) {
                std.debug.print("IsKindOf({s}, {s}): jolt {d}\n", .{ a_name, b_name, jolt });
                mismatches += 1;
            }
        }
    }
    try std.testing.expectEqual(@as(usize, 0), mismatches);
}

/// Write `object` (a `*const T`) with Zolt, the caller owns the bytes
fn zoltWrite(comptime T: type, allocator: Allocator, object: *const T, stream_type: StreamType) ![]u8 {
    var stream: std.Io.Writer.Allocating = .init(allocator);
    errdefer stream.deinit();
    try ObjectStreamOut.sWriteObject(T, allocator, &stream.writer, stream_type, object);
    return stream.toOwnedSlice();
}

const Format = struct { stream_type: StreamType, binary: c_int };
const formats = [_]Format{ .{ .stream_type = .text, .binary = 0 }, .{ .stream_type = .binary, .binary = 1 } };

fn expectSameBytes(what: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (!std.mem.eql(u8, zolt_bytes, jolt_bytes)) {
        const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse 0;
        std.debug.print("{s}: bytes differ at {d} (zolt {d} bytes, jolt {d} bytes)\n", .{ what, at, zolt_bytes.len, jolt_bytes.len });
        return error.ParityMismatch;
    }
}

fn randomMotor(rng: *std.Random.DefaultPrng) MotorDesc {
    const r = rng.random();
    const specials = [_]f32{ 0, -0.0, 1, -1, 0.1, 1.0e20, -1.0e-5, std.math.floatMax(f32), -std.math.floatMax(f32), std.math.floatMin(f32), 2.0, 0.5 };
    var floats: [6]f32 = undefined;
    for (&floats) |*f| f.* = if (r.uintLessThan(u32, 3) == 0) specials[r.uintLessThan(usize, specials.len)] else (r.float(f32) - 0.5) * std.math.pow(f32, 10, @floatFromInt(r.intRangeAtMost(i32, -6, 6)));
    return .{
        .spring_mode = r.uintLessThan(u32, 3),
        .frequency = floats[0],
        .damping = floats[1],
        .min_force_limit = floats[2],
        .max_force_limit = floats[3],
        .min_torque_limit = floats[4],
        .max_torque_limit = floats[5],
    };
}

test "ObjectStream: MotorSettings bytes, read and write in both directions" {
    const allocator = std.testing.allocator;
    var rng: std.Random.DefaultPrng = .init(1234);
    var buffer: [4096]u8 = undefined;
    var buffer2: [4096]u8 = undefined;
    for (0..300) |_| {
        const desc = randomMotor(&rng);
        const motor = desc.toMotor();
        for (formats) |f| {
            // Zolt writes Jolt's bytes
            const n = jolt_os_motor_write(&desc, f.binary, &buffer, buffer.len);
            try std.testing.expect(n > 0);
            const jolt_bytes = buffer[0..@intCast(n)];
            const zolt_bytes = try zoltWrite(MotorSettings, allocator, &motor, f.stream_type);
            defer allocator.free(zolt_bytes);
            try expectSameBytes("MotorSettings write", zolt_bytes, jolt_bytes);

            // Zolt reads Jolt's bytes (the values and the bytes written again, in both formats)
            var reader: std.Io.Reader = .fixed(jolt_bytes);
            const read = try ObjectStreamIn.sReadObject(MotorSettings, allocator, &reader, .{ .factory = &factory });
            defer RTTI.of(MotorSettings).destructObject(read, allocator);
            try std.testing.expect(std.mem.eql(u8, std.mem.asBytes(&desc), std.mem.asBytes(&MotorDesc.fromMotor(read))));
            for (formats) |g| {
                var jolt_desc: MotorDesc = undefined;
                const m = jolt_os_motor_resave(jolt_bytes.ptr, @intCast(jolt_bytes.len), g.binary, &buffer2, buffer2.len, &jolt_desc);
                try std.testing.expect(m > 0);
                const rewritten = try zoltWrite(MotorSettings, allocator, read, g.stream_type);
                defer allocator.free(rewritten);
                try expectSameBytes("MotorSettings read and write again", rewritten, buffer2[0..@intCast(m)]);
            }

            // Jolt reads Zolt's bytes
            var jolt_desc: MotorDesc = undefined;
            const m = jolt_os_motor_resave(zolt_bytes.ptr, @intCast(zolt_bytes.len), f.binary, &buffer2, buffer2.len, &jolt_desc);
            try std.testing.expect(m > 0);
            try expectSameBytes("Jolt reads Zolt's MotorSettings", buffer2[0..@intCast(m)], zolt_bytes);
        }
    }
}

/// The factory of these tests (Jolt: RegisterTypes and the classes that the C++ side registers in EnsureFactory)
const factory: Factory = .init(&.{ MotorSettings, SpringSettings, ost.TestSerializable });

test "ObjectStream: the object of ObjectStreamTest.cpp, bytes and reading in both directions" {
    const allocator = std.testing.allocator;
    var buffer: [8192]u8 = undefined;
    var buffer2: [8192]u8 = undefined;
    const test_object = try ost.createTestObject(allocator);
    defer test_object.destroy();
    for (formats) |f| {
        // Zolt writes Jolt's bytes
        const n = jolt_os_unit_test_write(f.binary, &buffer, buffer.len);
        try std.testing.expect(n > 0);
        const jolt_bytes = buffer[0..@intCast(n)];
        const zolt_bytes = try zoltWrite(ost.TestSerializable, allocator, test_object, f.stream_type);
        defer allocator.free(zolt_bytes);
        try expectSameBytes("TestSerializable write", zolt_bytes, jolt_bytes);

        // Zolt reads Jolt's bytes: the casts, the shared object with 2 references, the bytes written again
        var reader: std.Io.Reader = .fixed(jolt_bytes);
        const read = try ObjectStreamIn.sReadObject(ost.TestSerializable, allocator, &reader, .{ .factory = &factory });
        defer read.destroy();
        try std.testing.expectEqual(@as(?*ost.TestSerializableBase2, &read.base2), RTTI.dynamicCast(ost.TestSerializableBase2, ost.TestSerializable, read));
        try std.testing.expectEqual(@as(u32, 2), read.reference.ptr.?.getRefCount());
        for (formats) |g| {
            var ref_count: u32 = 0;
            var casts_ok: c_int = 0;
            const m = jolt_os_unit_test_resave(jolt_bytes.ptr, @intCast(jolt_bytes.len), g.binary, &buffer2, buffer2.len, &ref_count, &casts_ok);
            try std.testing.expect(m > 0 and ref_count == 2 and casts_ok == 1);
            const rewritten = try zoltWrite(ost.TestSerializable, allocator, read, g.stream_type);
            defer allocator.free(rewritten);
            try expectSameBytes("TestSerializable read and write again", rewritten, buffer2[0..@intCast(m)]);
        }

        // Jolt reads Zolt's bytes
        var ref_count: u32 = 0;
        var casts_ok: c_int = 0;
        const m = jolt_os_unit_test_resave(zolt_bytes.ptr, @intCast(zolt_bytes.len), f.binary, &buffer2, buffer2.len, &ref_count, &casts_ok);
        try std.testing.expect(m > 0 and ref_count == 2 and casts_ok == 1);
        try expectSameBytes("Jolt reads Zolt's TestSerializable", buffer2[0..@intCast(m)], zolt_bytes);
    }
}

test "ObjectStream: old and odd MotorSettings streams are read like Jolt" {
    const allocator = std.testing.allocator;
    const streams = [_][]const u8{
        // The old schema of Assets/Human.tof: "mFrequency" / "mDamping" are aliases of the spring settings, no mode
        "TOS 1.00\n\ndeclare MotorSettings 6\n  mFrequency float\n  mDamping float\n  mMinForceLimit float\n  mMaxForceLimit float\n  mMinTorqueLimit float\n  mMaxTorqueLimit float\n\nobject MotorSettings 00000001\n  20\n  0.899999976\n  -3.40282347e+38\n  3.40282347e+38\n  -100\n  100",
        // Retyped (skipped), unknown (skipped) and missing (default) attributes, LF line ends
        "TOS 1.00\ndeclare MotorSettings 4\n mSpringMode uint32\n mFrequency double\n mFoo vec3\n mMaxTorqueLimit float\nobject MotorSettings 00000001\n 1\n 2.5\n 1 2 3\n 7.5",
        // Number words that libc++ reads partly: "2xyz" is 2, "-0" is -0, an identifier with a 0x prefix
        "TOS 1.00 declare MotorSettings 2 mSpringMode uint32 mMinForceLimit float object MotorSettings 0x1 2xyz -0",
        // A word that is not a data type after the object ends the read: success
        "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 0.25 garbage",
        // A float that libc++ does not read: the object is dropped, the read fails (no main object)
        "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 1.5abc",
        // A subnormal float (ERANGE): fails like the above
        "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000001 1e-40",
        // Another version
        "TOS 1.01 declare MotorSettings 1 mDamping float object MotorSettings 00000001 0.25",
        // Not an object stream
        "XOS 1.00",
        // An object without an identifier is skipped, the next one is the main object
        "TOS 1.00 declare MotorSettings 1 mDamping float object MotorSettings 00000000 0.25 object MotorSettings 00000002 0.75",
        // A class that is declared twice appends its attributes
        "TOS 1.00 declare MotorSettings 1 mDamping float declare MotorSettings 1 mFrequency float object MotorSettings 00000001 0.25 4",
    };
    var buffer: [4096]u8 = undefined;
    for (streams, 0..) |stream, i| {
        for (formats) |g| {
            var jolt_desc: MotorDesc = undefined;
            const n = jolt_os_motor_resave(stream.ptr, @intCast(stream.len), g.binary, &buffer, buffer.len, &jolt_desc);
            var reader: std.Io.Reader = .fixed(stream);
            const read = ObjectStreamIn.sReadObject(MotorSettings, allocator, &reader, .{ .factory = &factory }) catch |err| {
                if (err != error.InvalidObjectStream or n != -1) {
                    std.debug.print("stream {d}: zolt {s}, jolt {d}\n", .{ i, @errorName(err), n });
                    return error.ParityMismatch;
                }
                continue;
            };
            defer RTTI.of(MotorSettings).destructObject(read, allocator);
            if (n < 0) {
                std.debug.print("stream {d}: zolt read it, jolt {d}\n", .{ i, n });
                return error.ParityMismatch;
            }
            const rewritten = try zoltWrite(MotorSettings, allocator, read, g.stream_type);
            defer allocator.free(rewritten);
            try expectSameBytes("old MotorSettings stream", rewritten, buffer[0..@intCast(n)]);
        }
    }
}
