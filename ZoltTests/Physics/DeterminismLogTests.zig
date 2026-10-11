//! Zolt addition, no Jolt file: the text of the determinism log (Zolt/Physics/DeterminismLog.zig, SimulationArchitecture
//! D11) for a short single threaded scene
//! Status: complete
//!
//! The tests need the library built with `-Ddeterminism_log=true`: `zig build test` runs them in a second unit test
//! binary with the option on (build.zig); in the default binary they are skipped. The expected text is built from
//! Jolt's `operator <<` definitions (DeterminismLog.h): a string as is, `int` in decimal after a string, floats as the
//! 8 hex digits of their bits (lower case, padded with '0'), Vec3 as 3 of them separated by spaces, BodyID as the 8
//! hex digits of its index and sequence number, one line per JPH_DET_LOG. The C++ parity library is built without
//! JPH_ENABLE_DETERMINISM_LOG, so the text is checked by construction, not against a C++ log.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const BodyID = zolt.BodyID;
const BodyCreationSettings = zolt.BodyCreationSettings;
const SphereShape = zolt.SphereShape;
const DeterminismLog = zolt.DeterminismLog;

const io = std.testing.io;

/// The text of one JobIntegrateVelocity line, written with std.fmt from the body's velocities
fn appendIntegrateLine(allocator: std.mem.Allocator, text: *std.ArrayList(u8), id: BodyID, v: Vec3, w: Vec3) !void {
    const s = try std.fmt.allocPrint(allocator, "JobIntegrateVelocity: id: {x:0>8} v: {x:0>8} {x:0>8} {x:0>8} w: {x:0>8} {x:0>8} {x:0>8}\n", .{
        id.getIndexAndSequenceNumber(),
        @as(u32, @bitCast(v.getX())),
        @as(u32, @bitCast(v.getY())),
        @as(u32, @bitCast(v.getZ())),
        @as(u32, @bitCast(w.getX())),
        @as(u32, @bitCast(w.getY())),
        @as(u32, @bitCast(w.getZ())),
    });
    defer allocator.free(s);
    try text.appendSlice(allocator, s);
}

test "DeterminismLog: the log of a short single threaded scene" {
    if (!DeterminismLog.enabled) return error.SkipZigTest; // Needs -Ddeterminism_log=true (build.zig runs it)
    const allocator = std.testing.allocator;

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .worker_threads = 0 }); // Single threaded: the main thread runs every job
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Two dynamic spheres far apart (gravity only, no contacts) and a static body that is never integrated
    var ids: [2]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        var settings: BodyCreationSettings = .initWithShape((try SphereShape.create(allocator, 0.5, .{})).asShape(), RVec3.init(@floatFromInt(10 * i), 0, 0), Quat.identity(), .dynamic, Layers.moving);
        defer settings.deinit();
        settings.angular_velocity = Vec3.init(0, 1 + @as(f32, @floatFromInt(i)), 0);
        id.* = try bi.createAndAddBody(&settings, .activate);
    }
    var static_settings: BodyCreationSettings = .initWithShape((try SphereShape.create(allocator, 1, .{})).asShape(), RVec3.init(0, -100, 0), Quat.identity(), .static, Layers.non_moving);
    defer static_settings.deinit();
    _ = try bi.createAndAddBody(&static_settings, .dont_activate);

    var log: std.Io.Writer.Allocating = .init(allocator);
    defer log.deinit();
    DeterminismLog.writer = &log.writer;
    defer DeterminismLog.writer = null;

    var expected: std.ArrayList(u8) = .empty;
    defer expected.deinit(allocator);

    // Two updates with one collision step and one with two: the update line, then per collision step a line per
    // active body (the velocity before the position integration, which no later job changes in this scene)
    for ([_]u32{ 1, 1, 2 }) |collision_steps| {
        c.collision_steps = collision_steps;
        try expected.appendSlice(allocator, if (collision_steps == 1) "PhysicsSystem::Update: dt: 3c888889 steps: 1\n" else "PhysicsSystem::Update: dt: 3c888889 steps: 2\n");
        const before = log.written().len;
        _ = try c.simulateSingleStep();

        // The velocities of the last collision step are the current ones; the ones of the first of two steps are in
        // the log, check their format through the line structure and the last step's values
        if (collision_steps == 2) {
            const lines = log.written()[before..];
            var it = std.mem.splitScalar(u8, lines, '\n');
            _ = it.next(); // The update line
            for (ids) |id| {
                const line = it.next().?;
                const prefix = try std.fmt.allocPrint(allocator, "JobIntegrateVelocity: id: {x:0>8} v: ", .{id.getIndexAndSequenceNumber()});
                defer allocator.free(prefix);
                try std.testing.expect(std.mem.startsWith(u8, line, prefix));
                try std.testing.expectEqual(prefix.len + 3 * 9 - 1 + 4 + 3 * 9 - 1, line.len);
                try expected.appendSlice(allocator, line);
                try expected.append(allocator, '\n');
            }
        }
        for (ids) |id|
            try appendIntegrateLine(allocator, &expected, id, bi.getLinearVelocity(id), bi.getAngularVelocity(id));
    }
    try std.testing.expectEqualStrings(expected.items, log.written());

    // The first line of the first step, spelled out: the body was at rest, gravity made it fall during one step
    // (v.y is about -9.81 / 60, between -0.25 and -0.125: the bits start with be)
    try std.testing.expect(std.mem.indexOf(u8, log.written(), "PhysicsSystem::Update: dt: 3c888889 steps: 1\nJobIntegrateVelocity: id: 00800000 v: 00000000 be") == 0); // Sequence number 1, index 0

    // Nothing is logged without a writer
    DeterminismLog.writer = null;
    const written = log.written().len;
    _ = try c.simulateSingleStep();
    try std.testing.expectEqual(written, log.written().len);
}

test "DeterminismLog: open and close write detlog.txt into the base path" {
    if (!DeterminismLog.enabled) return error.SkipZigTest; // Needs -Ddeterminism_log=true (build.zig runs it)

    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const base_path = ".zig-cache/tmp/" ++ tmp.sub_path;
    DeterminismLog.base_path = base_path;
    defer DeterminismLog.base_path = null;

    try DeterminismLog.open(io);
    DeterminismLog.log(.{ "label: ", @as(f32, 1.0), " id: ", BodyID.fromIndexAndSequenceNumber(3, 2), " n: ", @as(i32, 42) });
    DeterminismLog.log(.{ "v: ", Vec3.init(1, 2, 3) });
    DeterminismLog.close();
    try std.testing.expect(DeterminismLog.writer == null);

    var buffer: [256]u8 = undefined;
    const text = try tmp.dir.readFile(io, "detlog.txt", &buffer);
    try std.testing.expectEqualStrings("label: 3f800000 id: 01000003 n: 42\nv: 3f800000 40000000 40400000\n", text);
}
