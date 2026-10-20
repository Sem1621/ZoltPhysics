//! Parity tests of the soft body foundation (Phase 9 group S0, RemainingFeaturesArchitecture.md R15, R16), compared bit
//! for bit with the C++ library. C ABI wrappers: ZoltParity/Physics/SoftBodyReference.cpp. Adapted from the delivery
//! prototype of the Phase 9 design (commit 3759ab6).
//!
//! - SaveBinaryState of random shared settings: every array (the private update groups and skinned constraint normals
//!   too) holds random values of a generator that both sides run. Zolt's bytes must equal Jolt's. Each side restores
//!   the other side's bytes and saves them again: the same bytes. The restored arrays equal the generated ones.
//! - SaveBinaryState of settings that Jolt builds (sCreateCube for grid sizes 2 to 8, cloths of 2 x 2 to 13 x 13
//!   vertices, tetrahedra): the bytes parse with Zolt's extern structs (every byte is used, the counts equal Jolt's
//!   array sizes), and Zolt restores them and saves the same bytes.
//! - Truncated streams: RestoreBinaryState of every prefix of a stream (random cuts for long streams). The saved bytes
//!   and the state of the input stream (EOF, failed) are the same on both sides.
//! - Creating a soft body: BodyInterface.createSoftBody (BodyManager.allocateSoftBody,
//!   Body.applySoftBodyCreationSettings, SoftBodyMotionProperties.initialize and calculateMassAndInertia) with random
//!   creation settings, on the shared settings that each side restores from the same bytes: the body (position,
//!   rotation, bounds, layers, filter, material values), the motion properties (mass, inertia, damping, the soft body
//!   values, the volume, the reference count of the shared settings), every vertex, every rod state and every skin
//!   state, the local and predicted bounds.
//! - Applying creation settings again: Body.applySoftBodyCreationSettings on the created body, 1 to 3 times, with other
//!   settings (fewer or more vertices, with or without rods and skinned constraints). The same values after the last
//!   call, which pin Jolt's quirks: the rod and skin states of earlier settings are kept when the new settings have
//!   none, and the local bounds keep growing (the new vertices are added to the old bounds).
//!
//! The reference installs a counting AssertFailed (SoftBodyReference.cpp). With `-Dreference_asserts=true` a case
//! fails when its C++ side hit a Jolt assert.
//!
//! The simulation of soft bodies (SaveState, the listener, thread counts) comes with groups SH and S5. The constraint
//! creation and Optimize come with group S1 (its own SoftBodySettings layer).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const TestLayers = @import("test_layers");

const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const PhysicsSystem = zolt.PhysicsSystem;
const SoftBodySharedSettings = zolt.SoftBodySharedSettings;
const SoftBodyCreationSettings = zolt.SoftBodyCreationSettings;
const SoftBodyMotionProperties = zolt.SoftBodyMotionProperties;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const S = SoftBodySharedSettings;

const io = std.testing.io;

const jolt = struct {
    extern fn jolt_sb_settings_bytes(kind: u32, seed: u32, out_bytes: [*]u8, capacity: u32, out_counts: *[11]u32) u32;
    extern fn jolt_sb_resave(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_stream_state: *u32) u32;
    extern fn jolt_sb_create_body(steps: [*]const Step, num_steps: u32, result: *BodyResult, vertices: [*]VertexResult, vertex_capacity: u32, rods: [*]RodResult, rod_capacity: u32, skin_states: [*]SkinResult, skin_capacity: u32) u32;
    extern fn jolt_sb_reference_assert_count() u32;
    extern fn jolt_sb_reference_has_asserts() u32;
    extern fn jolt_sb_test_assert_hook(out_before: *u32, out_after: *u32) void;
};

/// SBSettingsKind of SoftBodyReference.cpp
const SettingsKind = enum(u32) { random, cube, cloth, tetrahedron, tetrahedron_distance_bend };

/// The random generator of the random kind (SBRandom of SoftBodyReference.cpp): xorshift32
const SBRandom = struct {
    state: u32,

    fn init(seed: u32) SBRandom {
        const state = seed ^ 0x9e3779b9;
        return .{ .state = if (state == 0) 1 else state };
    }

    fn next(self: *SBRandom) u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        return self.state;
    }

    /// A value in [0, n)
    fn int(self: *SBRandom, n: u32) u32 {
        return self.next() % n;
    }

    /// A float in [-15.625, 15.625] with 7 fraction bits (exact)
    fn float(self: *SBRandom) f32 {
        const i: i32 = @as(i32, @intCast(self.next() % 4001)) - 2000;
        return @as(f32, @floatFromInt(i)) / 128.0;
    }

    /// An inverse mass in [0, 2] with 2 fraction bits (0 for 1 in 9 vertices: kinematic)
    fn invMass(self: *SBRandom) f32 {
        return @as(f32, @floatFromInt(self.next() % 9)) / 4.0;
    }

    fn quat(self: *SBRandom) Quat {
        const x = self.float();
        const y = self.float();
        const z = self.float();
        const w = self.float();
        return Quat.init(x, y, z, w);
    }
};

/// The settings of SBGenerateRandom (SoftBodyReference.cpp), the private arrays filled directly
fn generateRandom(allocator: Allocator, seed: u32) !Ref(S) {
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    const s = ref.get().?;
    const a = allocator;
    var r: SBRandom = .init(seed);

    const nv = 1 + r.int(8);
    for (0..nv) |_| {
        var v: S.Vertex = .{};
        v.position.x = r.float();
        v.position.y = r.float();
        v.position.z = r.float();
        v.velocity.x = r.float();
        v.velocity.y = r.float();
        v.velocity.z = r.float();
        v.inv_mass = r.invMass();
        try s.vertices.append(a, v);
    }
    const nf = r.int(5);
    for (0..nf) |_| {
        var f: S.Face = .{ .vertex = undefined };
        for (&f.vertex) |*vertex| vertex.* = r.int(nv);
        f.material_index = r.int(3);
        try s.faces.append(a, f);
    }
    const ne = r.int(5);
    for (0..ne) |_| {
        var e: S.Edge = .{ .vertex = undefined };
        for (&e.vertex) |*vertex| vertex.* = r.int(nv);
        e.rest_length = r.float();
        e.compliance = r.float();
        try s.edge_constraints.append(a, e);
    }
    const nd = r.int(4);
    for (0..nd) |_| {
        var d: S.DihedralBend = .{ .vertex = undefined };
        for (&d.vertex) |*vertex| vertex.* = r.int(nv);
        d.compliance = r.float();
        d.initial_angle = r.float();
        try s.dihedral_bend_constraints.append(a, d);
    }
    const nvol = r.int(4);
    for (0..nvol) |_| {
        var v: S.Volume = .{ .vertex = undefined };
        for (&v.vertex) |*vertex| vertex.* = r.int(nv);
        v.six_rest_volume = r.float();
        v.compliance = r.float();
        try s.volume_constraints.append(a, v);
    }
    const ns = r.int(4);
    for (0..ns) |_| {
        var sk: S.Skinned = .{};
        sk.vertex = r.int(nv);
        for (&sk.weights) |*w| {
            w.inv_bind_index = r.int(4);
            w.weight = r.float();
        }
        sk.max_distance = r.float();
        sk.back_stop_distance = r.float();
        sk.back_stop_radius = r.float();
        sk.normal_info = r.next();
        try s.skinned_constraints.append(a, sk);
    }
    const nn = r.int(5);
    for (0..nn) |_| try s.skinned_constraint_normals.append(a, r.next());
    const nl = r.int(4);
    for (0..nl) |_| {
        var l: S.LRA = .{ .vertex = undefined };
        for (&l.vertex) |*vertex| vertex.* = r.int(nv);
        l.max_distance = r.float();
        try s.lra_constraints.append(a, l);
    }
    const nr = r.int(4);
    for (0..nr) |_| {
        var rod: S.RodStretchShear = .{ .vertex = undefined };
        for (&rod.vertex) |*vertex| vertex.* = r.int(nv);
        rod.length = r.float();
        rod.inv_mass = r.float();
        rod.compliance = r.float();
        rod.bishop = r.quat();
        try s.rod_stretch_shear_constraints.append(a, rod);
    }
    const nb = r.int(4);
    for (0..nb) |_| {
        var rod: S.RodBendTwist = .{ .rod = undefined };
        for (&rod.rod) |*index| index.* = r.int(nr + 1);
        rod.compliance = r.float();
        rod.omega0 = r.quat();
        try s.rod_bend_twist_constraints.append(a, rod);
    }
    const ni = r.int(4);
    for (0..ni) |_| {
        var b: S.InvBind = .{};
        b.joint_index = r.next();
        var c: [4]Vec4 = undefined;
        for (&c) |*column| {
            const x = r.float();
            const y = r.float();
            const z = r.float();
            const w = r.float();
            column.* = Vec4.init(x, y, z, w);
        }
        b.inv_bind = Mat44.init(c[0], c[1], c[2], c[3]);
        try s.inv_bind_matrices.append(a, b);
    }
    const ng = 1 + r.int(3);
    for (0..ng) |_| {
        var g: S.UpdateGroup = undefined;
        g.edge_end_index = r.next();
        g.lra_end_index = r.next();
        g.rod_stretch_shear_end_index = r.next();
        g.rod_bend_twist_end_index = r.next();
        g.dihedral_bend_end_index = r.next();
        g.volume_end_index = r.next();
        g.skinned_end_index = r.next();
        try s.update_groups.append(a, g);
    }
    return ref;
}

/// The arrays of `a` and `b` hold the same values (the private closest kinematic array and the materials are not
/// compared: SaveBinaryState does not write them)
fn sameArrays(a: *const S, b: *const S) bool {
    const raw = .{ "vertices", "faces", "edge_constraints", "dihedral_bend_constraints", "volume_constraints", "skinned_constraints", "skinned_constraint_normals", "lra_constraints", "update_groups" };
    inline for (raw) |name|
        if (!std.mem.eql(u8, std.mem.sliceAsBytes(@field(a, name).items), std.mem.sliceAsBytes(@field(b, name).items))) return false;
    if (a.rod_stretch_shear_constraints.items.len != b.rod_stretch_shear_constraints.items.len) return false;
    for (a.rod_stretch_shear_constraints.items, b.rod_stretch_shear_constraints.items) |x, y|
        if (!std.mem.eql(u32, &x.vertex, &y.vertex) or !fw.sameBits(x.length, y.length) or !fw.sameBits(x.inv_mass, y.inv_mass) or !fw.sameBits(x.compliance, y.compliance) or !x.bishop.eql(y.bishop)) return false;
    if (a.rod_bend_twist_constraints.items.len != b.rod_bend_twist_constraints.items.len) return false;
    for (a.rod_bend_twist_constraints.items, b.rod_bend_twist_constraints.items) |x, y|
        if (!std.mem.eql(u32, &x.rod, &y.rod) or !fw.sameBits(x.compliance, y.compliance) or !x.omega0.eql(y.omega0)) return false;
    if (a.inv_bind_matrices.items.len != b.inv_bind_matrices.items.len) return false;
    for (a.inv_bind_matrices.items, b.inv_bind_matrices.items) |x, y|
        if (x.joint_index != y.joint_index or !x.inv_bind.eql(y.inv_bind)) return false;
    return true;
}

/// SaveBinaryState of `s` (owned by the caller)
fn saveBytes(allocator: Allocator, s: *const S) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: StreamOutWrapper = .init(&out.writer);
    s.saveBinaryState(wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory;
    return out.toOwnedSlice();
}

/// RestoreBinaryState of `bytes` into new settings: the settings and the state of the input stream (bit 0 = EOF,
/// bit 1 = failed, like jolt_sb_resave)
fn restore(allocator: Allocator, bytes: []const u8, stream_state: *u32) !Ref(S) {
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    try ref.get().?.restoreBinaryState(in.streamIn());
    stream_state.* = @as(u32, @intFromBool(in.isEOF())) | @as(u32, @intFromBool(in.isFailed())) << 1;
    return ref;
}

var jolt_buffer: [1 << 20]u8 = undefined;
var jolt_buffer2: [1 << 20]u8 = undefined;

/// jolt_sb_settings_bytes into jolt_buffer
fn joltSettingsBytes(kind: SettingsKind, seed: u32, counts: *[11]u32) ![]const u8 {
    const size = jolt.jolt_sb_settings_bytes(@intFromEnum(kind), seed, &jolt_buffer, jolt_buffer.len, counts);
    if (size == 0xFFFFFFFF) return error.JoltSettingsBytes;
    return jolt_buffer[0..size];
}

/// jolt_sb_resave into jolt_buffer2
fn joltResave(bytes: []const u8, stream_state: *u32) ![]const u8 {
    const size = jolt.jolt_sb_resave(bytes.ptr, @intCast(bytes.len), &jolt_buffer2, jolt_buffer2.len, stream_state);
    if (size == 0xFFFFFFFF) return error.JoltResave;
    return jolt_buffer2[0..size];
}

/// Reads a SaveBinaryState stream with Zolt's extern structs: a u32 count followed by count raw elements
/// (StreamOut.writeArray) or by count elements written field by field (StreamOut.writeArrayWith)
const Reader = struct {
    bytes: []const u8,
    pos: usize = 0,

    fn take(self: *Reader, n: usize) ![]const u8 {
        if (self.pos + n > self.bytes.len) return error.Truncated;
        defer self.pos += n;
        return self.bytes[self.pos..][0..n];
    }

    fn int(self: *Reader) !u32 {
        return std.mem.readInt(u32, (try self.take(4))[0..4], .little);
    }

    /// An array of raw elements: returns the count
    fn rawArray(self: *Reader, comptime T: type) !u32 {
        const n = try self.int();
        _ = try self.take(n * @sizeOf(T));
        return n;
    }

    /// An array whose elements are written field by field, `element_size` bytes each
    fn fieldArray(self: *Reader, element_size: usize) !u32 {
        const n = try self.int();
        _ = try self.take(n * element_size);
        return n;
    }
};

/// The array sizes in the order of jolt_sb_settings_bytes (the last one is the number of update groups)
fn parseCounts(bytes: []const u8) ![11]u32 {
    var r: Reader = .{ .bytes = bytes };
    var counts: [11]u32 = @splat(0);
    counts[0] = try r.rawArray(S.Vertex);
    counts[1] = try r.rawArray(S.Face);
    counts[2] = try r.rawArray(S.Edge);
    counts[3] = try r.rawArray(S.DihedralBend);
    counts[4] = try r.rawArray(S.Volume);
    counts[5] = try r.rawArray(S.Skinned);
    _ = try r.rawArray(u32); // mSkinnedConstraintNormals
    counts[7] = try r.rawArray(S.LRA);
    counts[10] = try r.rawArray(S.UpdateGroup);
    counts[8] = try r.fieldArray(8 + 4 + 4 + 4 + 16); // RodStretchShear: mVertex[2], mLength, mInvMass, mCompliance, mBishop
    counts[9] = try r.fieldArray(8 + 4 + 16); // RodBendTwist: mRod[2], mCompliance, mOmega0
    counts[6] = try r.fieldArray(4 + 64); // InvBind: mJointIndex, mInvBind
    if (r.pos != bytes.len) return error.TrailingBytes;
    return counts;
}

/// The array sizes of `s` in the order of jolt_sb_settings_bytes
fn zoltCounts(s: *const S) [11]u32 {
    return .{
        @intCast(s.vertices.items.len),                      @intCast(s.faces.items.len),
        @intCast(s.edge_constraints.items.len),              @intCast(s.dihedral_bend_constraints.items.len),
        @intCast(s.volume_constraints.items.len),            @intCast(s.skinned_constraints.items.len),
        @intCast(s.inv_bind_matrices.items.len),             @intCast(s.lra_constraints.items.len),
        @intCast(s.rod_stretch_shear_constraints.items.len), @intCast(s.rod_bend_twist_constraints.items.len),
        @intCast(s.update_groups.items.len),
    };
}

/// Byte mismatches of one test, the first few are printed
const ByteChecker = struct {
    name: []const u8,
    mismatches: usize = 0,

    fn check(self: *ByteChecker, what: []const u8, input: anytype, zolt_bytes: []const u8, jolt_bytes: []const u8) void {
        if (std.mem.eql(u8, zolt_bytes, jolt_bytes)) return;
        if (self.mismatches < 5) {
            const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
            std.debug.print("{s}: {s} differ for input {any}: {d} vs {d} bytes, first difference at byte {d}\n", .{ self.name, what, input, zolt_bytes.len, jolt_bytes.len, at });
        }
        self.mismatches += 1;
    }

    fn fail(self: *ByteChecker, what: []const u8, input: anytype) void {
        if (self.mismatches < 5) std.debug.print("{s}: {s} for input {any}\n", .{ self.name, what, input });
        self.mismatches += 1;
    }

    fn finish(self: *const ByteChecker) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches\n", .{ self.name, self.mismatches });
            return error.ParityMismatch;
        }
    }
};

const num_random_settings = 3000;

test "SoftBody parity: SaveBinaryState and RestoreBinaryState of random shared settings" {
    const allocator = std.testing.allocator;
    var checker: ByteChecker = .{ .name = "SoftBody random settings" };
    for (0..num_random_settings) |i| {
        const seed: u32 = @intCast(i);
        var counts: [11]u32 = undefined;
        const jolt_bytes = try joltSettingsBytes(.random, seed, &counts);

        // Zolt's settings of the same generator: the same bytes
        var generated = try generateRandom(allocator, seed);
        defer generated.deinit();
        const zolt_bytes = try saveBytes(allocator, generated.get().?);
        defer allocator.free(zolt_bytes);
        checker.check("SaveBinaryState", seed, zolt_bytes, jolt_bytes);

        // Zolt restores Jolt's bytes: the generated values, saved as the same bytes
        var state: u32 = undefined;
        var restored = try restore(allocator, jolt_bytes, &state);
        defer restored.deinit();
        if (state != 0) checker.fail("the stream failed in Zolt's RestoreBinaryState", seed);
        if (!sameArrays(restored.get().?, generated.get().?)) checker.fail("RestoreBinaryState gave other values", seed);
        const resaved = try saveBytes(allocator, restored.get().?);
        defer allocator.free(resaved);
        checker.check("Zolt's resave", seed, resaved, jolt_bytes);

        // Jolt restores Zolt's bytes and saves the same bytes
        var jolt_state: u32 = undefined;
        const jolt_resaved = try joltResave(zolt_bytes, &jolt_state);
        if (jolt_state != 0) checker.fail("the stream failed in Jolt's RestoreBinaryState", seed);
        checker.check("Jolt's resave", seed, zolt_bytes, jolt_resaved);
    }
    try checker.finish();
}

test "SoftBody parity: SaveBinaryState of Jolt's settings parses with Zolt's layouts" {
    const allocator = std.testing.allocator;
    var checker: ByteChecker = .{ .name = "SoftBody Jolt settings" };
    const cases = [_]struct { kind: SettingsKind, seeds: u32 }{
        .{ .kind = .cube, .seeds = 35 }, // Grid sizes 2 to 8, 5 spacings
        .{ .kind = .cloth, .seeds = 12 }, // 2 x 2 to 13 x 13 vertices
        .{ .kind = .tetrahedron, .seeds = 1 },
        .{ .kind = .tetrahedron_distance_bend, .seeds = 1 },
    };
    for (cases) |case| for (0..case.seeds) |i| {
        const seed: u32 = @intCast(i);
        const input = .{ case.kind, seed };
        var counts: [11]u32 = undefined;
        const jolt_bytes = try joltSettingsBytes(case.kind, seed, &counts);

        // Every byte parses with Zolt's extern structs, the counts are Jolt's array sizes (the update groups are private
        // in Jolt: only the parsed count is known)
        const parsed = try parseCounts(jolt_bytes);
        for (0..10) |c| if (parsed[c] != counts[c]) checker.fail("a parsed count differs from Jolt's array size", input);

        // Zolt restores them: the same counts, saved as the same bytes
        var state: u32 = undefined;
        var restored = try restore(allocator, jolt_bytes, &state);
        defer restored.deinit();
        if (state != 0) checker.fail("the stream failed in Zolt's RestoreBinaryState", input);
        if (!std.meta.eql(zoltCounts(restored.get().?), parsed)) checker.fail("Zolt restored other array sizes", input);
        const resaved = try saveBytes(allocator, restored.get().?);
        defer allocator.free(resaved);
        checker.check("Zolt's resave", input, resaved, jolt_bytes);

        // Jolt facts that the prototype pinned (commit 3759ab6)
        if (case.kind == .cube and seed == 4) {
            // sCreateCube(6, ...): 216 vertices, 300 faces, 540 edges, 750 volumes; Optimize puts all vertices in
            // parallel group 0, the non-parallel group (the last one) is empty
            try expectEqual([11]u32{ 216, 300, 540, 0, 750, 0, 0, 0, 0, 0, 2 }, parsed);
            const groups = restored.get().?.update_groups.items;
            try expectEqual(groups[0], groups[1]);
            try expectEqual(@as(u32, 540), groups[0].edge_end_index);
        }
        if (case.kind == .cloth and seed == 8) {
            // 10 x 10 cloth: 162 faces, 342 edges (stretch and shear), 225 dihedral bends, LRA constraints for the 96
            // movable vertices, too few vertices for a parallel group
            try expectEqual([11]u32{ 100, 162, 342, 225, 0, 0, 0, 96, 0, 0, 1 }, parsed);
        }
        if (case.kind == .tetrahedron or case.kind == .tetrahedron_distance_bend) {
            // Not optimized: no update groups
            try expectEqual(@as(u32, 0), parsed[10]);
        }
    };
    try checker.finish();
}

test "SoftBody parity: RestoreBinaryState of truncated streams" {
    const allocator = std.testing.allocator;
    var checker: ByteChecker = .{ .name = "SoftBody truncated settings" };
    var rng: fw.Rng = .{};
    const full = try allocator.alloc(u8, jolt_buffer.len);
    defer allocator.free(full);
    for (0..400) |i| {
        // Mostly random settings, some cubes and cloths
        const kind: SettingsKind = switch (i % 8) {
            0 => .cube,
            1 => .cloth,
            else => .random,
        };
        const seed: u32 = @intCast(i);
        var counts: [11]u32 = undefined;
        const bytes = try joltSettingsBytes(kind, seed, &counts);
        @memcpy(full[0..bytes.len], bytes);

        // Every cut of a short stream, 64 random cuts of a long one
        const num_cuts: usize = if (bytes.len <= 600) bytes.len + 1 else 64;
        for (0..num_cuts) |c| {
            const cut: usize = if (bytes.len <= 600) c else rng.intRange(usize, 0, bytes.len);
            const input = .{ kind, seed, cut };
            var zolt_state: u32 = undefined;
            var restored = try restore(allocator, full[0..cut], &zolt_state);
            defer restored.deinit();
            const resaved = try saveBytes(allocator, restored.get().?);
            defer allocator.free(resaved);
            var jolt_state: u32 = undefined;
            const jolt_resaved = try joltResave(full[0..cut], &jolt_state);
            if (zolt_state != jolt_state) checker.fail("the stream state after the restore differs", input);
            checker.check("the resaved bytes", input, resaved, jolt_resaved);
        }
    }
    try checker.finish();
}

/// SBCreateDesc of SoftBodyReference.cpp: the fields of SoftBodyCreationSettings
const CreateDesc = extern struct {
    position: [3]f64,
    user_data: u64,
    rotation: [4]f32,
    linear_damping: f32,
    max_linear_velocity: f32,
    restitution: f32,
    friction: f32,
    pressure: f32,
    gravity_factor: f32,
    vertex_radius: f32,
    object_layer: u32,
    num_iterations: u32,
    update_position: u32,
    make_rotation_identity: u32,
    allow_sleeping: u32,
    faces_double_sided: u32,
    group_id: u32,
    sub_group_id: u32,
    pad: u32 = 0,
};

/// SBStep of SoftBodyReference.cpp: the bytes of the shared settings (restoreBinaryState) and the other fields of the
/// creation settings of one step
const Step = extern struct {
    settings_bytes: [*]const u8,
    size: u32,
    pad: u32 = 0,
    desc: CreateDesc,

    fn bytes(self: *const Step) []const u8 {
        return self.settings_bytes[0..self.size];
    }
};

/// SBBodyResult of SoftBodyReference.cpp
const BodyResult = extern struct {
    position: [3]f64,
    center_of_mass_position: [3]f64,
    user_data: u64,
    rotation: [4]f32,
    world_bounds: [6]f32,
    local_bounds: [6]f32,
    local_predicted_bounds: [6]f32,
    inv_mass: f32,
    inv_inertia_diagonal: [3]f32,
    inertia_rotation: [4]f32,
    linear_damping: f32,
    angular_damping: f32,
    max_linear_velocity: f32,
    max_angular_velocity: f32,
    gravity_factor: f32,
    friction: f32,
    restitution: f32,
    pressure: f32,
    vertex_radius: f32,
    volume: f32,
    object_layer: u32,
    broad_phase_layer: u32,
    motion_type: u32,
    motion_quality: u32,
    allow_sleeping: u32,
    is_soft_body: u32,
    num_iterations: u32,
    update_position: u32,
    faces_double_sided: u32,
    group_id: u32,
    sub_group_id: u32,
    num_vertices: u32,
    /// The rods of the shared settings of the last step
    num_rods: u32,
    /// The reference count of the shared settings of the last step
    settings_ref_count: u32,
    shape_sub_type: u32,
    enable_skin_constraints: u32,
    /// The rod states of the motion properties
    num_rod_states: u32,
    /// The skin states of the motion properties
    num_skin_states: u32,
    /// The reference count of the shared settings of the first step
    first_settings_ref_count: u32,
    pad: u32,
};

/// SBVertexResult of SoftBodyReference.cpp (SoftBodyMotionProperties.Vertex)
const VertexResult = extern struct {
    previous_position: [3]f32,
    position: [3]f32,
    velocity: [3]f32,
    collision_plane: [4]f32,
    colliding_shape_index: i32,
    has_contact: u32,
    largest_penetration: f32,
    inv_mass: f32,
};

/// SBRodResult of SoftBodyReference.cpp
const RodResult = extern struct {
    rotation: [4]f32,
    angular_velocity: [3]f32,
};

/// SBSkinResult of SoftBodyReference.cpp (SoftBodyMotionProperties.SkinState)
const SkinResult = extern struct {
    previous_position: [3]f32,
    position: [3]f32,
    normal: [3]f32,
};

comptime {
    // The C structs have the same sizes (no implicit padding differences)
    std.debug.assert(@sizeOf(CreateDesc) == 112 and @sizeOf(Step) == 128 and @sizeOf(BodyResult) == 296);
    std.debug.assert(@sizeOf(VertexResult) == 68 and @sizeOf(RodResult) == 28 and @sizeOf(SkinResult) == 36);
}

fn store3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn storeQuat(q: Quat) [4]f32 {
    return .{ q.getX(), q.getY(), q.getZ(), q.getW() };
}

fn storeR3(v: RVec3) [3]f64 {
    return .{ @floatCast(v.getX()), @floatCast(v.getY()), @floatCast(v.getZ()) };
}

/// A random creation desc. The rotation is normalized (Jolt asserts it, Mat44::sRotation).
fn randomDesc(rng: *fw.Rng) CreateDesc {
    const q = Quat.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1) + 0.01).normalized();
    return .{
        .position = .{ rng.float(-100, 100), rng.float(-100, 100), rng.float(-100, 100) },
        .user_data = @as(u64, rng.next()) << 32 | rng.next(),
        .rotation = storeQuat(q),
        .linear_damping = rng.float(0, 1),
        .max_linear_velocity = rng.float(1, 1000),
        .restitution = rng.float(0, 1),
        .friction = rng.float(0, 1),
        .pressure = rng.float(0, 100),
        .gravity_factor = rng.float(-2, 2),
        .vertex_radius = rng.float(0, 0.1),
        .object_layer = rng.intRange(u32, 0, 10),
        .num_iterations = rng.intRange(u32, 1, 10),
        .update_position = rng.intRange(u32, 0, 1),
        .make_rotation_identity = rng.intRange(u32, 0, 1),
        .allow_sleeping = rng.intRange(u32, 0, 1),
        .faces_double_sided = rng.intRange(u32, 0, 1),
        .group_id = rng.next(),
        .sub_group_id = rng.next(),
    };
}

const max_vertices = 1024;
const max_rods = 16;
const max_steps = 4;

/// The creation settings of a step, with the shared settings that Zolt restores from the step's bytes (only the
/// creation settings hold a reference, like the C++ side)
fn creationSettings(allocator: Allocator, step: *const Step) !SoftBodyCreationSettings {
    const desc = &step.desc;
    const shared = try S.create(allocator);
    var settings: SoftBodyCreationSettings = .init(shared, RVec3.init(@floatCast(desc.position[0]), @floatCast(desc.position[1]), @floatCast(desc.position[2])), Quat.init(desc.rotation[0], desc.rotation[1], desc.rotation[2], desc.rotation[3]), @intCast(desc.object_layer));
    errdefer settings.deinit();
    {
        var reader: std.Io.Reader = .fixed(step.bytes());
        var in = StreamInWrapper.init(&reader);
        try shared.restoreBinaryState(in.streamIn());
    }
    settings.user_data = desc.user_data;
    settings.collision_group = .init(null, desc.group_id, desc.sub_group_id);
    settings.num_iterations = desc.num_iterations;
    settings.linear_damping = desc.linear_damping;
    settings.max_linear_velocity = desc.max_linear_velocity;
    settings.restitution = desc.restitution;
    settings.friction = desc.friction;
    settings.pressure = desc.pressure;
    settings.gravity_factor = desc.gravity_factor;
    settings.vertex_radius = desc.vertex_radius;
    settings.update_position = desc.update_position != 0;
    settings.make_rotation_identity = desc.make_rotation_identity != 0;
    settings.allow_sleeping = desc.allow_sleeping != 0;
    settings.faces_double_sided = desc.faces_double_sided != 0;
    return settings;
}

/// The results of one case on one side
const Results = struct {
    body: BodyResult = undefined,
    vertices: [max_vertices]VertexResult = undefined,
    rods: [max_rods]RodResult = undefined,
    skin_states: [max_vertices]SkinResult = undefined,
};

/// BodyInterface.createSoftBody in `system` with the creation settings of step 0, then
/// Body.applySoftBodyCreationSettings on the created body with those of every other step (jolt_sb_create_body on the
/// Zolt side). The results are the state after the last step, then the body is destroyed. Returns the number of
/// vertices.
fn zoltCreateBody(allocator: Allocator, system: *PhysicsSystem, bpl_interface: *const zolt.BroadPhaseLayerInterface, steps: []const Step, results: *Results) !u32 {
    var settings: [max_steps]SoftBodyCreationSettings = undefined;
    var num_settings: usize = 0;
    defer for (settings[0..num_settings]) |*s| s.deinit();
    for (steps) |*step| {
        settings[num_settings] = try creationSettings(allocator, step);
        num_settings += 1;
    }
    const first_shared = settings[0].settings.get().?;
    const last_shared = settings[num_settings - 1].settings.get().?;

    const bi = system.getBodyInterfaceNoLockMut();
    const body = (try bi.createSoftBody(&settings[0])) orelse return error.NoBody;
    defer bi.destroyBody(body.getID());
    for (settings[1..num_settings]) |*s| try body.applySoftBodyCreationSettings(s, bpl_interface);

    const mp = SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties());
    results.body = std.mem.zeroes(BodyResult);
    const r = &results.body;
    r.position = storeR3(body.getPosition());
    r.center_of_mass_position = storeR3(body.getCenterOfMassPosition());
    r.user_data = body.getUserData();
    r.rotation = storeQuat(body.getRotation());
    r.world_bounds[0..3].* = store3(body.getWorldSpaceBounds().min);
    r.world_bounds[3..6].* = store3(body.getWorldSpaceBounds().max);
    r.local_bounds[0..3].* = store3(mp.getLocalBounds().min);
    r.local_bounds[3..6].* = store3(mp.getLocalBounds().max);
    r.local_predicted_bounds[0..3].* = store3(mp.local_predicted_bounds.min);
    r.local_predicted_bounds[3..6].* = store3(mp.local_predicted_bounds.max);
    r.inv_mass = mp.base.getInverseMassUnchecked();
    r.inv_inertia_diagonal = store3(mp.base.getInverseInertiaDiagonal());
    r.inertia_rotation = storeQuat(mp.base.getInertiaRotation());
    r.linear_damping = mp.base.getLinearDamping();
    r.angular_damping = mp.base.getAngularDamping();
    r.max_linear_velocity = mp.base.getMaxLinearVelocity();
    r.max_angular_velocity = mp.base.getMaxAngularVelocity();
    r.gravity_factor = mp.base.getGravityFactor();
    r.friction = body.getFriction();
    r.restitution = body.getRestitution();
    r.pressure = mp.getPressure();
    r.vertex_radius = mp.getVertexRadius();
    r.volume = mp.getVolume();
    r.object_layer = body.getObjectLayer();
    r.broad_phase_layer = body.getBroadPhaseLayer().getValue();
    r.motion_type = @intFromEnum(body.getMotionType());
    r.motion_quality = @intFromEnum(mp.base.getMotionQuality());
    r.allow_sleeping = @intFromBool(mp.base.getAllowSleeping());
    r.is_soft_body = @intFromBool(body.isSoftBody());
    r.num_iterations = mp.getNumIterations();
    r.update_position = @intFromBool(mp.getUpdatePosition());
    r.faces_double_sided = @intFromBool(mp.getFacesDoubleSided());
    r.group_id = body.getCollisionGroup().getGroupID();
    r.sub_group_id = body.getCollisionGroup().getSubGroupID();
    r.num_vertices = @intCast(mp.getVertices().len);
    r.num_rods = @intCast(last_shared.rod_stretch_shear_constraints.items.len);
    r.settings_ref_count = last_shared.getRefCount();
    r.shape_sub_type = @intFromEnum(body.getShape().getSubType());
    r.enable_skin_constraints = @intFromBool(mp.getEnableSkinConstraints());
    r.num_rod_states = @intCast(mp.rod_states.items.len);
    r.num_skin_states = @intCast(mp.skin_state.items.len);
    r.first_settings_ref_count = first_shared.getRefCount();

    if (r.num_vertices > max_vertices or r.num_rod_states > max_rods or r.num_skin_states > max_vertices) return error.TooManyVertices;
    for (mp.getVertices(), results.vertices[0..r.num_vertices]) |*v, *o| {
        o.* = .{
            .previous_position = store3(v.previous_position),
            .position = store3(v.position),
            .velocity = store3(v.velocity),
            .collision_plane = .{ v.collision_plane.getNormal().getX(), v.collision_plane.getNormal().getY(), v.collision_plane.getNormal().getZ(), v.collision_plane.getConstant() },
            .colliding_shape_index = v.colliding_shape_index,
            .has_contact = @intFromBool(v.has_contact),
            .largest_penetration = v.largest_penetration,
            .inv_mass = v.inv_mass,
        };
    }
    for (results.rods[0..r.num_rod_states], 0..) |*o, i| {
        o.* = .{ .rotation = storeQuat(mp.getRodRotation(@intCast(i))), .angular_velocity = store3(mp.getRodAngularVelocity(@intCast(i))) };
    }
    for (mp.skin_state.items, results.skin_states[0..r.num_skin_states]) |*s, *o| {
        o.* = .{ .previous_position = store3(s.previous_position), .position = store3(s.position), .normal = store3(s.normal) };
    }
    return r.num_vertices;
}

/// jolt_sb_create_body: the number of vertices and the number of Jolt asserts that failed during the call (always 0
/// without `-Dreference_asserts=true`)
fn joltCreateBody(steps: []const Step, results: *Results) !struct { u32, u32 } {
    const asserts_before = jolt.jolt_sb_reference_assert_count();
    const n = jolt.jolt_sb_create_body(steps.ptr, @intCast(steps.len), &results.body, &results.vertices, max_vertices, &results.rods, max_rods, &results.skin_states, max_vertices);
    if (n == 0xFFFFFFFF) return error.JoltCreateBody;
    return .{ n, jolt.jolt_sb_reference_assert_count() - asserts_before };
}

/// The checkers of the creation layers
const CreationCheckers = struct {
    body: fw.Checker,
    vertices: fw.Checker,
    rods: fw.Checker,
    skin_states: fw.Checker,
    asserts: fw.Checker,

    fn init(comptime name: []const u8) CreationCheckers {
        return .{
            .body = .{ .name = name ++ ": body and motion properties" },
            .vertices = .{ .name = name ++ ": vertices" },
            .rods = .{ .name = name ++ ": rod states" },
            .skin_states = .{ .name = name ++ ": skin states" },
            .asserts = .{ .name = name ++ ": Jolt asserts that failed (the reference was built with asserts)" },
        };
    }

    /// Compares the results of both sides
    fn check(self: *CreationCheckers, input: anytype, zolt_n: u32, z: *const Results, jolt_n: u32, jolt_asserts: u32, j: *const Results) void {
        self.asserts.check(input, @as(u32, 0), jolt_asserts);
        self.body.check(input, z.body, j.body);
        if (zolt_n == jolt_n) {
            for (z.vertices[0..zolt_n], j.vertices[0..jolt_n], 0..) |zv, jv, v| self.vertices.check(.{ input, v }, zv, jv);
        }
        if (z.body.num_rod_states == j.body.num_rod_states) {
            for (z.rods[0..z.body.num_rod_states], j.rods[0..j.body.num_rod_states], 0..) |zr, jr, r| self.rods.check(.{ input, r }, zr, jr);
        }
        if (z.body.num_skin_states == j.body.num_skin_states) {
            for (z.skin_states[0..z.body.num_skin_states], j.skin_states[0..j.body.num_skin_states], 0..) |zs, js, s| self.skin_states.check(.{ input, s }, zs, js);
        }
    }

    fn finish(self: *const CreationCheckers) !void {
        try fw.finishAll(&.{ &self.body, &self.vertices, &self.rods, &self.skin_states, &self.asserts });
    }
};

/// Settings whose inertia tensor is singular make Jolt's EigenValueSymmetric assert (see hasSingularInertia): a case
/// with such settings is skipped when either side checks its asserts
fn skipSingularInertia() bool {
    return zolt.Core.enable_asserts or jolt.jolt_sb_reference_has_asserts() != 0;
}

var zolt_results: Results = .{};
var jolt_results: Results = .{};

/// A physics system with the layers of UnitTests/Layers.h, like the one of jolt_sb_create_body
const TestSystem = struct {
    layers: TestLayers.BPLayerInterfaceImpl,
    object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl,
    pairs: TestLayers.ObjectLayerPairFilterImpl,
    system: PhysicsSystem,

    fn init(self: *TestSystem, allocator: Allocator) !void {
        self.layers = .init();
        self.object_vs_broad_phase = .{};
        self.pairs = .{};
        self.system = .empty;
        try self.system.init(allocator, io, 4, 0, 4, 4, &self.layers.base, &self.object_vs_broad_phase.base, &self.pairs.base, .{ .broad_phase = .brute_force });
    }

    fn deinit(self: *TestSystem) void {
        self.system.deinit();
    }
};

test "SoftBody parity: the reference counts its asserts" {
    // With -Dreference_asserts=true the hook of SoftBodyReference.cpp prints and counts a failed Jolt assert (the
    // creation layers check the count), without asserts nothing is counted
    var before: u32 = undefined;
    var after: u32 = undefined;
    jolt.jolt_sb_test_assert_hook(&before, &after);
    try expectEqual(before + jolt.jolt_sb_reference_has_asserts(), after);
}

test "SoftBody parity: creating a soft body (allocateSoftBody, applySoftBodyCreationSettings, initialize)" {
    const allocator = std.testing.allocator;
    var ts: TestSystem = undefined;
    try ts.init(allocator);
    defer ts.deinit();

    var checkers: CreationCheckers = .init("SoftBody creation");
    var rng: fw.Rng = .{};
    const bytes = try allocator.alloc(u8, jolt_buffer.len);
    defer allocator.free(bytes);
    var num_kinematic: u32 = 0;
    var num_rods: u32 = 0;
    var num_skinned: u32 = 0;
    var num_skipped: u32 = 0;
    for (0..1500) |i| {
        // Random settings (rods, skinned constraints, kinematic vertices), cubes and cloths (kinematic corners)
        const kind: SettingsKind = switch (i % 10) {
            0 => .cube,
            1 => .cloth,
            else => .random,
        };
        const seed: u32 = @intCast(i);
        var counts: [11]u32 = undefined;
        const jolt_bytes = try joltSettingsBytes(kind, seed, &counts);
        @memcpy(bytes[0..jolt_bytes.len], jolt_bytes);
        const step: Step = .{ .settings_bytes = bytes.ptr, .size = @intCast(jolt_bytes.len), .desc = randomDesc(&rng) };
        const input = .{ kind, seed, step.desc };
        if (skipSingularInertia() and hasSingularInertia(step.bytes())) {
            num_skipped += 1;
            continue;
        }

        const zolt_n = try zoltCreateBody(allocator, &ts.system, &ts.layers.base, &.{step}, &zolt_results);
        const jolt_n, const jolt_asserts = try joltCreateBody(&.{step}, &jolt_results);
        checkers.check(input, zolt_n, &zolt_results, jolt_n, jolt_asserts, &jolt_results);
        if (jolt_results.body.inv_mass == 0) num_kinematic += 1;
        num_rods += jolt_results.body.num_rod_states;
        if (jolt_results.body.num_skin_states != 0) num_skinned += 1;
    }
    try checkers.finish();

    // The cases reach both mass paths, the rods and the skin states
    try expect(num_kinematic > 100 and num_kinematic < 1400);
    try expect(num_rods > 500);
    try expect(num_skinned > 500);
    try expect(num_skipped < 300);
}

test "SoftBody parity: applying creation settings again (applySoftBodyCreationSettings on a created body)" {
    const allocator = std.testing.allocator;
    var ts: TestSystem = undefined;
    try ts.init(allocator);
    defer ts.deinit();

    var checkers: CreationCheckers = .init("SoftBody re-applied settings");
    var rng: fw.Rng = .{ .state = 0x5eed50f7 };
    var num_kept_rod_states: u32 = 0;
    var num_kept_skin_states: u32 = 0;
    var num_grown_bounds: u32 = 0;
    var num_fewer_vertices: u32 = 0;
    var num_skipped: u32 = 0;
    for (0..600) |i| {
        // 2 to 4 steps of random settings, cubes and cloths: the vertex count goes up and down, rods and skinned
        // constraints come and go
        const num_steps = rng.intRange(usize, 2, max_steps);
        var steps: [max_steps]Step = undefined;
        var step_bytes: [max_steps][]u8 = undefined;
        var step_counts: [max_steps][11]u32 = undefined;
        var num_bytes: usize = 0;
        defer for (step_bytes[0..num_bytes]) |b| allocator.free(b);
        var inputs: [max_steps]struct { SettingsKind, u32 } = undefined;
        var skip = false;
        for (0..num_steps) |s| {
            const kind: SettingsKind = switch (rng.intRange(u32, 0, 9)) {
                0, 1 => .cube,
                2, 3 => .cloth,
                else => .random,
            };
            const seed: u32 = @intCast(100_000 + i * max_steps + s);
            var counts: [11]u32 = undefined;
            const jolt_bytes = try joltSettingsBytes(kind, seed, &counts);
            step_bytes[s] = try allocator.dupe(u8, jolt_bytes);
            num_bytes += 1;
            step_counts[s] = try parseCounts(jolt_bytes);
            steps[s] = .{ .settings_bytes = step_bytes[s].ptr, .size = @intCast(jolt_bytes.len), .desc = randomDesc(&rng) };
            inputs[s] = .{ kind, seed };
            // Every call computes the mass and inertia
            if (skipSingularInertia() and hasSingularInertia(jolt_bytes)) skip = true;
        }
        if (skip) {
            num_skipped += 1;
            continue;
        }
        const input = .{ i, inputs[0..num_steps] };

        const zolt_n = try zoltCreateBody(allocator, &ts.system, &ts.layers.base, steps[0..num_steps], &zolt_results);
        const jolt_n, const jolt_asserts = try joltCreateBody(steps[0..num_steps], &jolt_results);
        checkers.check(input, zolt_n, &zolt_results, jolt_n, jolt_asserts, &jolt_results);

        // The cases reach Jolt's quirks: rod and skin states of earlier settings, bounds that keep the old vertices
        const j = &jolt_results.body;
        const last = &step_counts[num_steps - 1];
        if (j.num_rod_states > j.num_rods) num_kept_rod_states += 1;
        if (last[5] == 0 and j.num_skin_states != 0) num_kept_skin_states += 1;
        var bounds: zolt.AABox = .empty;
        for (jolt_results.vertices[0..jolt_n]) |v| bounds.encapsulateVec3(Vec3.init(v.position[0], v.position[1], v.position[2]));
        const vertex_bounds = store3(bounds.min) ++ store3(bounds.max);
        if (!std.mem.eql(f32, &vertex_bounds, &j.local_bounds)) num_grown_bounds += 1;
        for (1..num_steps) |s| {
            if (step_counts[s][0] < step_counts[s - 1][0]) {
                num_fewer_vertices += 1;
                break;
            }
        }
    }
    try checkers.finish();

    try expect(num_kept_rod_states > 50);
    try expect(num_kept_skin_states > 50);
    try expect(num_grown_bounds > 300);
    try expect(num_fewer_vertices > 150);
    try expect(num_skipped < 200);
}

/// Settings with one dynamic vertex, or two on a line through the origin, and no kinematic vertex: their inertia
/// tensor has a zero eigenvalue. Jolt's EigenValueSymmetric asserts on it (the check of the eigenvectors cannot pass
/// for a zero eigenvalue), and so does Zolt's in a safe build. A build without runtime safety compares these cases too.
fn hasSingularInertia(bytes: []const u8) bool {
    const num_vertices = std.mem.readInt(u32, bytes[0..4], .little);
    if (num_vertices >= 3) return false;
    var positions: [2]Vec3 = undefined;
    for (0..num_vertices) |v| {
        var vertex: S.Vertex = undefined;
        @memcpy(std.mem.asBytes(&vertex), bytes[4 + v * @sizeOf(S.Vertex) ..][0..@sizeOf(S.Vertex)]);
        if (vertex.inv_mass == 0) return false; // Kinematic: no inertia tensor
        positions[v] = Vec3.fromFloat3(vertex.position);
    }
    return num_vertices == 1 or positions[0].cross(positions[1]).isNearZero(.{ .max_dist_sq = 0 });
}
