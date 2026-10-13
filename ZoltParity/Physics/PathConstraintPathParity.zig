//! Parity tests of PathConstraintPathHermite (Jolt/Physics/Constraints/PathConstraintPathHermite, PathConstraintPath):
//! GetPathMaxFraction, GetPointOnPath (the Hermite evaluation, the index and fraction of a segment, the normalized frame),
//! GetClosestPoint (the bisection and Newton Raphson search of every segment) and the binary state
//! (SaveBinaryState, sRestoreFromBinaryState across languages, truncated streams), on random paths with 2 to 64 points,
//! looping and not. C ABI wrappers: ZoltParity/Physics/PathConstraintPathReference.cpp.
//!
//! The paths are smooth: open paths are random walks whose direction turns a little at every point, looping paths are
//! deformed circles, both with tangents of 0.5 to 2 times the segment length and normals near the Y axis, plus straight
//! paths like the one of Jolt's TestPathConstraintPathHermite (tangents aligned with the segment, where Newton Raphson
//! needs the bisection). So the frame of GetPointOnPath is never degenerate (Jolt asserts that the normal is
//! normalized, Zolt's Debug build asserts the same). Fractions cover the whole path, the points, beyond both ends of an
//! open path and several turns of a looping path (negative fractions of a looping path extrapolate a segment with a
//! negative t, only a little: further out the Hermite tangent can vanish). The PathConstraint parity, which uses these
//! functions in the solver, is in ConstraintsParity.zig and SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Ref = zolt.Ref;
const PathConstraintPath = zolt.PathConstraintPath;
const PathConstraintPathHermite = zolt.PathConstraintPathHermite;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;

/// Number of random paths
const num_paths = 3000;

/// Fractions and closest point queries per path
const num_fractions = 32;
const num_queries = 32;

const max_points = 64;

/// Must match PCPPath in PathConstraintPathReference.cpp
const PCPPath = extern struct {
    num_points: u32,
    looping: u32,
    points: [max_points * 9]f32,
};

const jolt = struct {
    extern fn jolt_pcp_struct_size() u32;
    extern fn jolt_pcp_query(path: *const PCPPath, fractions: [*]const f32, num_fractions: u32, queries: [*]const f32, num_queries: u32, out_max_fraction: *f32, out_points: [*]f32, out_closest: [*]f32) void;
    extern fn jolt_pcp_save(path: *const PCPPath, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_pcp_resave(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
};

/// The results of the queries of one path
const QueryResult = struct {
    max_fraction: f32,
    points: [num_fractions][12]f32,
    closest: [num_queries]f32,
};

fn store3(v: Vec3, out: []f32) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
}

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

/// The Zolt path of a description (PCPMakePath), one reference owned by the caller
fn makePath(allocator: Allocator, desc: *const PCPPath) !Ref(PathConstraintPath) {
    const hermite = try PathConstraintPathHermite.create(allocator);
    var path = Ref(PathConstraintPath).init(hermite.asPath());
    errdefer path.deinit();
    hermite.base.setIsLooping(desc.looping != 0);
    for (0..desc.num_points) |p| {
        const pf = desc.points[9 * p ..];
        try hermite.addPoint(load3(pf[0..3]), load3(pf[3..6]), load3(pf[6..9]));
    }
    return path;
}

/// jolt_pcp_query on the Zolt side
fn zoltQuery(allocator: Allocator, desc: *const PCPPath, fractions: []const f32, queries: []const [4]f32, out: *QueryResult) !void {
    var path = try makePath(allocator, desc);
    defer path.deinit();
    const p = path.get().?;
    out.max_fraction = p.getPathMaxFraction();
    for (fractions, &out.points) |fraction, *o| {
        const point = p.getPointOnPath(fraction);
        store3(point.position, o[0..3]);
        store3(point.tangent, o[3..6]);
        store3(point.normal, o[6..9]);
        store3(point.binormal, o[9..12]);
    }
    for (queries, &out.closest) |q, *o|
        o.* = p.getClosestPoint(load3(q[0..3]), q[3]);
}

/// SaveBinaryState of the Zolt path of a description
fn zoltSave(allocator: Allocator, desc: *const PCPPath) ![]u8 {
    var path = try makePath(allocator, desc);
    defer path.deinit();
    return saveBytes(allocator, path.get().?);
}

fn saveBytes(allocator: Allocator, path: *const PathConstraintPath) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var w = StreamOutWrapper.init(&data.writer);
    path.saveBinaryState(w.streamOut());
    return data.toOwnedSlice();
}

/// A restore + save: the bytes or the error text
const Resaved = struct {
    bytes: std.ArrayList(u8) = .empty,
    err: ?[]const u8 = null,
    err_buffer: [128]u8 = undefined,

    fn deinit(self: *Resaved, allocator: Allocator) void {
        self.bytes.deinit(allocator);
    }

    fn setError(self: *Resaved, message: []const u8) void {
        const n = @min(message.len, self.err_buffer.len);
        @memcpy(self.err_buffer[0..n], message[0..n]);
        self.err = self.err_buffer[0..n];
    }

    fn eql(self: *const Resaved, other: *const Resaved) bool {
        if (self.err != null or other.err != null)
            return self.err != null and other.err != null and std.mem.eql(u8, self.err.?, other.err.?);
        return std.mem.eql(u8, self.bytes.items, other.bytes.items);
    }
};

/// PathConstraintPath.restoreFromBinaryState of `bytes`, then saveBinaryState of the result
fn zoltResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    if (result.hasError()) {
        out.setError(result.getError());
        return;
    }
    const data = try saveBytes(allocator, result.getPtr().?);
    defer allocator.free(data);
    try out.bytes.appendSlice(allocator, data);
}

var byte_buffer: [65536]u8 = undefined;

fn joltSave(allocator: Allocator, desc: *const PCPPath) ![]u8 {
    const size = jolt.jolt_pcp_save(desc, &byte_buffer, byte_buffer.len);
    try std.testing.expect(size <= byte_buffer.len);
    return allocator.dupe(u8, byte_buffer[0..size]);
}

fn joltResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var err_buffer: [128]u8 = undefined;
    const size = jolt.jolt_pcp_resave(bytes.ptr, @intCast(bytes.len), &byte_buffer, byte_buffer.len, &err_buffer, err_buffer.len);
    if (size == 0xFFFFFFFF) {
        out.setError(std.mem.sliceTo(&err_buffer, 0));
        return;
    }
    try std.testing.expect(size <= byte_buffer.len);
    try out.bytes.appendSlice(allocator, byte_buffer[0..size]);
}

// ---------------------------------------------------------------------------------------------------------------------
// The input generator

const Gen = struct {
    rng: fw.Rng,

    fn index(self: *Gen, n: usize) usize {
        return self.rng.next() % n;
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.rng.next() % n == 0;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn sinCos(a: f32) [2]f32 {
        const sc = Vec4.replicate(a).sinCos();
        return .{ sc.sin.getX(), sc.cos.getX() };
    }

    /// A normal near the Y axis (not normalized: Jolt only uses its direction)
    fn normal(self: *Gen) Vec3 {
        return Vec3.init(self.float(-0.3, 0.3), self.float(0.5, 2.0), self.float(-0.3, 0.3));
    }

    fn setPoint(desc: *PCPPath, p: usize, position: Vec3, tangent: Vec3, n: Vec3) void {
        const pf = desc.points[9 * p ..];
        store3(position, pf[0..3]);
        store3(tangent, pf[3..6]);
        store3(n, pf[6..9]);
    }

    /// A random smooth path (see the file comment)
    fn path(self: *Gen) PCPPath {
        var desc: PCPPath = std.mem.zeroes(PCPPath);
        const far: f32 = if (self.oneIn(4)) 1000.0 else 0.0;
        const origin = Vec3.init(far + self.float(-10, 10), self.float(-10, 10), far + self.float(-10, 10));
        switch (self.index(8)) {
            0 => {
                // A straight path with tangents along the segments (Jolt's TestPathConstraintPathHermite)
                desc.num_points = @intCast(2 + self.index(8));
                const dir = Vec3.init(self.float(-1, 1), self.float(-0.5, 0.5), self.float(-1, 1)).normalized();
                var position = origin;
                for (0..desc.num_points) |p| {
                    const step = self.float(0.5, 30);
                    const tangent = dir.mulScalar(step * self.float(0.5, 2));
                    setPoint(&desc, p, position, tangent, if (self.oneIn(2)) Vec3.axisY() else self.normal());
                    position = position.add(dir.mulScalar(step));
                }
            },
            1, 2, 3 => {
                // A deformed circle (looping)
                desc.looping = 1;
                desc.num_points = @intCast(3 + self.index(max_points - 2));
                const radius = self.float(1, 20);
                const n: f32 = @floatFromInt(desc.num_points);
                for (0..desc.num_points) |p| {
                    const a = 2.0 * std.math.pi * (@as(f32, @floatFromInt(p)) + self.float(-0.2, 0.2)) / n;
                    const sc = sinCos(a);
                    const r = radius * self.float(0.9, 1.1);
                    const position = origin.add(Vec3.init(r * sc[1], self.float(-0.1, 0.1) * radius / n, r * sc[0]));
                    const tangent = Vec3.init(-sc[0], self.float(-0.05, 0.05), sc[1]).mulScalar(2.0 * std.math.pi * radius / n * self.float(0.7, 1.4));
                    setPoint(&desc, p, position, tangent, self.normal());
                }
            },
            else => {
                // A random walk that turns a little at every point (open)
                desc.num_points = @intCast(2 + self.index(max_points - 1));
                var heading = self.float(-std.math.pi, std.math.pi);
                var position = origin;
                for (0..desc.num_points) |p| {
                    const sc = sinCos(heading);
                    const dir = Vec3.init(sc[1], self.float(-0.2, 0.2), sc[0]).normalized();
                    const step = self.float(0.2, 5);
                    setPoint(&desc, p, position, dir.mulScalar(step * self.float(0.5, 2)), self.normal());
                    position = position.add(dir.mulScalar(step));
                    heading += self.float(-0.5, 0.5);
                }
            },
        }
        return desc;
    }

    /// Fractions for GetPointOnPath (see the file comment)
    fn fractions(self: *Gen, desc: *const PCPPath, out: []f32) void {
        const max_fraction: f32 = @floatFromInt(if (desc.looping != 0) desc.num_points else desc.num_points - 1);
        for (out) |*f| {
            f.* = switch (self.index(8)) {
                0 => @floatFromInt(self.index(desc.num_points + 1)), // On a point
                1 => if (self.oneIn(2)) 0.0 else max_fraction,
                2 => if (desc.looping != 0)
                    self.float(max_fraction, 3.0 * max_fraction) // A later turn
                else
                    self.float(-2, max_fraction + 2), // Beyond the ends: clamped
                3 => if (desc.looping != 0)
                    -@as(f32, @floatFromInt(self.index(3 * desc.num_points))) - self.float(0, 0.1) // Earlier turns
                else
                    self.float(-0.1, 0), // t < 0 of the first segment (only a little, see the file comment)
                else => self.float(0, max_fraction),
            };
        }
    }

    /// Positions for GetClosestPoint: near a point of the path, further away, anywhere
    fn queries(self: *Gen, desc: *const PCPPath, out: [][4]f32) void {
        const max_fraction: f32 = @floatFromInt(if (desc.looping != 0) desc.num_points else desc.num_points - 1);
        for (out) |*q| {
            const p = self.index(desc.num_points);
            const base = load3(desc.points[9 * p ..][0..3]);
            const position = switch (self.index(4)) {
                0 => base.add(Vec3.init(self.float(-0.1, 0.1), self.float(-0.1, 0.1), self.float(-0.1, 0.1))),
                1 => base.add(Vec3.init(self.float(-3, 3), self.float(-3, 3), self.float(-3, 3))),
                2 => base.add(Vec3.init(self.float(-30, 30), self.float(-30, 30), self.float(-30, 30))),
                else => if (p + 1 < desc.num_points) base.add(load3(desc.points[9 * (p + 1) ..][0..3]).sub(base).mulScalar(self.float(0, 1))) else base,
            };
            store3(position, q[0..3]);
            q[3] = self.float(0, max_fraction);
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The tests

test "PathConstraintPath parity: struct layout" {
    try std.testing.expectEqual(@as(u32, @sizeOf(PCPPath)), jolt.jolt_pcp_struct_size());
}

test "PathConstraintPath parity: Hermite paths (max fraction, points on the path, closest points)" {
    const allocator = std.testing.allocator;
    var max_checker: Checker = .{ .name = "PathConstraintPathHermite::GetPathMaxFraction" };
    var point_checker: Checker = .{ .name = "PathConstraintPathHermite::GetPointOnPath" };
    var closest_checker: Checker = .{ .name = "PathConstraintPathHermite::GetClosestPoint" };
    var g: Gen = .{ .rng = .{ .state = 0x9A7B0001 } };
    var zolt_result: QueryResult = undefined;
    var jolt_result: QueryResult = undefined;
    var num_looping: u32 = 0;
    var num_many_points: u32 = 0;
    var num_clamped: u32 = 0;
    var num_wrapped: u32 = 0;
    var num_interior_closest: u32 = 0;
    for (0..num_paths) |iteration| {
        const desc = g.path();
        var fractions: [num_fractions]f32 = undefined;
        g.fractions(&desc, &fractions);
        var queries: [num_queries][4]f32 = undefined;
        g.queries(&desc, &queries);

        try zoltQuery(allocator, &desc, &fractions, &queries, &zolt_result);
        jolt.jolt_pcp_query(&desc, &fractions, num_fractions, @ptrCast(&queries), num_queries, &jolt_result.max_fraction, @ptrCast(&jolt_result.points), &jolt_result.closest);

        max_checker.check(iteration, zolt_result.max_fraction, jolt_result.max_fraction);
        for (fractions, zolt_result.points, jolt_result.points) |f, z, j|
            point_checker.check(.{ iteration, f }, z, j);
        for (queries, zolt_result.closest, jolt_result.closest) |q, z, j| {
            closest_checker.check(.{ iteration, q }, z, j);
            if (z != @trunc(z)) num_interior_closest += 1;
        }

        // What the inputs reach
        if (desc.looping != 0) num_looping += 1;
        if (desc.num_points > 32) num_many_points += 1;
        for (fractions) |f| {
            if (desc.looping == 0 and (f < 0.0 or f > jolt_result.max_fraction)) num_clamped += 1;
            if (desc.looping != 0 and (f < 0.0 or f >= jolt_result.max_fraction)) num_wrapped += 1;
        }
    }
    try fw.finishAll(&.{ &max_checker, &point_checker, &closest_checker });
    try std.testing.expect(num_looping > num_paths / 4);
    try std.testing.expect(num_many_points > num_paths / 5);
    try std.testing.expect(num_clamped > num_paths);
    try std.testing.expect(num_wrapped > num_paths);
    try std.testing.expect(num_interior_closest > num_paths * num_queries / 4);
}

test "PathConstraintPath parity: Jolt's straight spline" {
    // The path of TestPathConstraintPathHermite: GetClosestPoint needs the bisection before Newton Raphson
    const allocator = std.testing.allocator;
    var desc: PCPPath = std.mem.zeroes(PCPPath);
    desc.num_points = 2;
    const values = [18]f32{ 1424.96313, 468.565399, 483.655975, 61.4222832, 42.8926392, -1.70530257e-13, 0, 0, 1, 1445.20105, 482.364319, 483.655975, 20.2380009, 13.7989082, -5.68434189e-14, 0, 0, 1 };
    @memcpy(desc.points[0..18], &values);
    var fractions: [num_fractions]f32 = undefined;
    for (&fractions, 0..) |*f, i| f.* = 0.05 * @as(f32, @floatFromInt(i)) - 0.3;
    var queries: [num_queries][4]f32 = @splat(@splat(0));
    var zolt_points: QueryResult = undefined;
    try zoltQuery(allocator, &desc, &fractions, &queries, &zolt_points); // The points on the path are the queries
    for (&queries, zolt_points.points) |*q, p| {
        @memcpy(q[0..3], p[0..3]);
        q[3] = 0.0;
    }
    var zolt_result: QueryResult = undefined;
    var jolt_result: QueryResult = undefined;
    try zoltQuery(allocator, &desc, &fractions, &queries, &zolt_result);
    jolt.jolt_pcp_query(&desc, &fractions, num_fractions, @ptrCast(&queries), num_queries, &jolt_result.max_fraction, @ptrCast(&jolt_result.points), &jolt_result.closest);
    var checker: Checker = .{ .name = "PathConstraintPathHermite: Jolt's straight spline" };
    checker.check(0, zolt_result, jolt_result);
    try checker.finish();
}

test "PathConstraintPath parity: binary state" {
    const allocator = std.testing.allocator;
    var save_checker: Checker = .{ .name = "PathConstraintPathHermite::SaveBinaryState" };
    var resave_checker: Checker = .{ .name = "PathConstraintPath::sRestoreFromBinaryState of the other side's bytes" };
    var error_checker: Checker = .{ .name = "PathConstraintPath::sRestoreFromBinaryState of truncated streams" };
    var g: Gen = .{ .rng = .{ .state = 0x9A7B0002 } };
    var num_errors: u32 = 0;
    for (0..num_paths / 3) |iteration| {
        const desc = g.path();
        const zolt_bytes = try zoltSave(allocator, &desc);
        defer allocator.free(zolt_bytes);
        const jolt_bytes = try joltSave(allocator, &desc);
        defer allocator.free(jolt_bytes);
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes, jolt_bytes), true);

        // Each side restores the other side's bytes and saves them again
        {
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltResave(allocator, jolt_bytes, &z);
            resave_checker.check(iteration, z.err == null and std.mem.eql(u8, z.bytes.items, jolt_bytes), true);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResave(allocator, zolt_bytes, &j);
            resave_checker.check(iteration, j.err == null and std.mem.eql(u8, j.bytes.items, zolt_bytes), true);
        }

        // Truncated streams and a corrupted type hash: the same error on both sides
        {
            const len = g.index(zolt_bytes.len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltResave(allocator, zolt_bytes[0..len], &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResave(allocator, zolt_bytes[0..len], &j);
            if (z.err != null) num_errors += 1;
            if (!z.eql(&j) and error_checker.mismatches < 5)
                std.debug.print("truncated at {d}: zolt '{s}' jolt '{s}'\n", .{ len, z.err orelse "(ok)", j.err orelse "(ok)" });
            error_checker.check(.{ iteration, len }, z.eql(&j), true);
        }
        if (iteration % 16 == 0) {
            const corrupt = try allocator.dupe(u8, zolt_bytes);
            defer allocator.free(corrupt);
            corrupt[g.index(4)] ^= 0x5A;
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltResave(allocator, corrupt, &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResave(allocator, corrupt, &j);
            try std.testing.expectEqualStrings("Failed to create instance of type", z.err.?);
            error_checker.check(.{ iteration, "hash" }, z.eql(&j), true);
        }
    }
    try fw.finishAll(&.{ &save_checker, &resave_checker, &error_checker });
    try std.testing.expect(num_errors == num_paths / 3); // Every truncated stream is an error
}
