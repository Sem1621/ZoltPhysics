//! Parity of the soft body shared settings algorithms (Phase 9 group S1, RemainingFeaturesArchitecture.md R20 and the
//! SoftBodySettings layer of section 4), compared bit for bit with the C++ library. C ABI wrapper:
//! ZoltParity/Physics/SoftBodySettingsReference.cpp.
//!
//! - A case is settings (their SaveBinaryState bytes, which both sides restore), vertex attributes and a list of
//!   operations: CreateConstraints (every bend type, random and default arguments, once or twice, with constraints that
//!   exist before), the Calculate* functions, Optimize with and without results, Clone, the private
//!   CalculateClosestKinematic, and AppendExtra (constraints added after CreateConstraints filled the closest kinematic
//!   cache, which Optimize then uses: R20). A cube case starts from sCreateCube instead.
//! - Each side writes a transcript of the run: per operation a marker, the 7 remaps of Optimize, the SaveBinaryState
//!   bytes of a clone, and the private closest kinematic array after every operation (Jolt's cache); at the end the
//!   SaveBinaryState bytes, which hold the constraints, the update groups and the skinned constraint normals. The two
//!   transcripts must be equal byte for byte.
//! - Generators: cloths (2 x 2 to 32 x 32, alternating, equal or random diagonals), spheres and bipyramids (closed
//!   meshes: the duplicated last edge), tetrahedra (volume constraints, the 4 faces of every tetrahedron, so many faces
//!   per edge), rods (chains with reversed rods, branches, preset Bishop frames), skinned meshes, triangle soups (fans
//!   of 3 and more faces per edge, back to back and duplicated faces, fans of more than 32 faces on one edge) and
//!   lattice graphs (equal distances from several kinematic vertices: ties in Dijkstra's heap). Vertex attributes per
//!   case or per vertex, with zero, FLT_MAX and other compliances and every LRA type.
//! - A separate test lays out 33 to 44 copies of a few cloths. Each copy becomes one parallel group, so the group size
//!   sort of Optimize gets more than 32 groups, many of equal size: QuickSort partitions them, its insertion sort does
//!   not decide their order alone.
//! - Coverage: the test fails unless the generators reached the hard cases (counted on the Zolt side, which equals the
//!   Jolt side when the transcripts are equal).
//! - The generators stay away from Jolt's asserts (no two vertices at the same position, no constraint that uses a
//!   vertex twice). With `-Dreference_asserts=true` a case fails when its C++ side hits a Jolt assert.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ParityHooks = @import("../ParityHooks.zig");

const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const Float3 = zolt.Float3;
const Ref = zolt.Ref;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const S = zolt.SoftBodySharedSettings;
const flt_max = zolt.math.flt_max;
const trig = zolt.trigonometry;

const jolt = struct {
    extern fn jolt_sbs_struct_sizes(out: *[2]u32) void;
    extern fn jolt_sbs_run(settings: [*]const u8, settings_size: u32, extra: [*]const u8, extra_size: u32, attributes: [*]const VertexAttributes, num_attributes: u32, ops: [*]const Op, num_ops: u32, cube_grid_size: u32, cube_spacing: f32, out: [*]u8, capacity: u32) u32;
};

/// SoftBodySharedSettings::VertexAttributes in Jolt's layout
const VertexAttributes = extern struct {
    compliance: f32,
    shear_compliance: f32,
    bend_compliance: f32,
    lra_type: i32,
    lra_max_distance_multiplier: f32,

    fn toZolt(a: VertexAttributes) S.VertexAttributes {
        return .init(a.compliance, a.shear_compliance, a.bend_compliance, .{ .lra_type = @enumFromInt(a.lra_type), .lra_max_distance_multiplier = a.lra_max_distance_multiplier });
    }
};

/// SBSOpKind of SoftBodySettingsReference.cpp
const OpKind = enum(u32) {
    create_constraints,
    calculate_edge_lengths,
    calculate_rod_properties,
    calculate_lra_lengths,
    calculate_bend_constraint_constants,
    calculate_volume_constraint_volumes,
    calculate_skinned_constraint_normals,
    optimize,
    optimize_no_results,
    calculate_closest_kinematic,
    clone,
    append_extra,
    create_constraints_defaults,
};

/// SBSOp of SoftBodySettingsReference.cpp
const Op = extern struct {
    kind: OpKind,
    bend_type: u32 = 0,
    value: f32 = 0.0,
};

comptime {
    std.debug.assert(@sizeOf(VertexAttributes) == 20);
    std.debug.assert(@sizeOf(Op) == 12);
}

/// The marker before the results of operation i
const op_marker: u32 = 0xABCD0000;

/// The capacity of the transcript buffer of the C++ side
const transcript_capacity = 16 << 20;

/// SaveBinaryState of `s` (owned by the caller)
fn saveBytes(allocator: Allocator, s: *const S) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: StreamOutWrapper = .init(&out.writer);
    s.saveBinaryState(wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory;
    return out.toOwnedSlice();
}

/// RestoreBinaryState of `bytes` into new settings
fn restore(allocator: Allocator, bytes: []const u8) !Ref(S) {
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    try ref.get().?.restoreBinaryState(in.streamIn());
    if (in.isFailed()) return error.RestoreFailed;
    return ref;
}

/// The transcript of a run on the Zolt side (SBSTranscript of SoftBodySettingsReference.cpp)
const Transcript = struct {
    allocator: Allocator,
    bytes: std.ArrayList(u8) = .empty,

    fn deinit(self: *Transcript) void {
        self.bytes.deinit(self.allocator);
    }

    fn write(self: *Transcript, data: []const u8) !void {
        try self.bytes.appendSlice(self.allocator, data);
    }

    fn writeU32(self: *Transcript, value: u32) !void {
        const little = std.mem.nativeToLittle(u32, value);
        try self.write(std.mem.asBytes(&little));
    }

    fn writeF32(self: *Transcript, value: f32) !void {
        try self.writeU32(@bitCast(value));
    }

    fn writeRemap(self: *Transcript, remap: []const u32) !void {
        try self.writeU32(@intCast(remap.len));
        for (remap) |v| try self.writeU32(v);
    }

    /// The SaveBinaryState bytes, prefixed by their size
    fn writeSettings(self: *Transcript, s: *const S) !void {
        const bytes = try saveBytes(self.allocator, s);
        defer self.allocator.free(bytes);
        try self.writeU32(@intCast(bytes.len));
        try self.write(bytes);
    }

    /// The private closest kinematic array: the count, then vertex, hops and distance per element
    fn writeClosestKinematic(self: *Transcript, s: *const S) !void {
        try self.writeU32(@intCast(s.closest_kinematic.items.len));
        for (s.closest_kinematic.items) |c| {
            try self.writeU32(c.vertex);
            try self.writeU32(c.hops);
            try self.writeF32(c.distance);
        }
    }
};

/// The kinds of generated settings
const Kind = enum { cloth, sphere, tetrahedra, rods, skinned, soup, graph };

/// One generated input
const Case = struct {
    kind: Kind,
    /// The SaveBinaryState bytes of the settings (both sides restore them)
    settings_bytes: []const u8 = &.{},
    /// The SaveBinaryState bytes of the settings that AppendExtra appends
    extra_bytes: []const u8 = &.{},
    attributes: std.ArrayList(VertexAttributes) = .empty,
    ops: std.ArrayList(Op) = .empty,
    /// sCreateCube instead of the settings when not 0
    cube_grid_size: u32 = 0,
    cube_spacing: f32 = 0.0,

    fn deinit(self: *Case, allocator: Allocator) void {
        allocator.free(self.settings_bytes);
        allocator.free(self.extra_bytes);
        self.attributes.deinit(allocator);
        self.ops.deinit(allocator);
    }

    fn hasOp(self: *const Case, kind: OpKind) bool {
        for (self.ops.items) |op|
            if (op.kind == kind) return true;
        return false;
    }
};

/// The input generator
const Gen = struct {
    rng: fw.Rng,

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    /// An integer in [lo, hi]
    fn int(self: *Gen, lo: u32, hi: u32) u32 {
        return self.rng.intRange(u32, lo, hi);
    }

    /// True with a probability of `percent` %
    fn chance(self: *Gen, percent: u32) bool {
        return self.rng.next() % 100 < percent;
    }

    /// A float in [lo, hi)
    fn float(self: *Gen, lo: f32, hi: f32) f32 {
        return self.rng.float(lo, hi);
    }

    /// An inverse mass: 0 for a kinematic vertex, else 1 or a random value
    fn invMass(self: *Gen, kinematic: bool) f32 {
        if (kinematic) return 0.0;
        return if (self.chance(50)) 1.0 else self.float(0.1, 2.0);
    }

    /// A compliance: zero, FLT_MAX (disabled) or another value
    fn compliance(self: *Gen) f32 {
        return switch (self.int(0, 5)) {
            0 => 0.0,
            1 => flt_max,
            2 => 1.0e-5,
            3 => self.float(0.0, 1.0e-3),
            4 => self.float(0.0, 1.0),
            else => 0.5,
        };
    }

    /// A normalized random quaternion
    fn quat(self: *Gen) Quat {
        return Quat.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(0.1, 1)).normalized();
    }

    /// A matrix with random entries
    fn mat44(self: *Gen) Mat44 {
        var c: [4]Vec4 = undefined;
        for (&c) |*column| column.* = Vec4.init(self.float(-2, 2), self.float(-2, 2), self.float(-2, 2), self.float(-2, 2));
        return Mat44.init(c[0], c[1], c[2], c[3]);
    }
};

/// Builds the input settings of a case
const Builder = struct {
    allocator: Allocator,
    s: *S,
    g: *Gen,

    fn numVertices(self: *const Builder) u32 {
        return @intCast(self.s.vertices.items.len);
    }

    fn vertex(self: *Builder, x: f32, y: f32, z: f32, inv_mass: f32) !u32 {
        const index = self.numVertices();
        try self.s.vertices.append(self.allocator, .init(.init(x, y, z), .{ .velocity = .init(self.g.float(-1, 1), self.g.float(-1, 1), self.g.float(-1, 1)), .inv_mass = inv_mass }));
        return index;
    }

    fn face(self: *Builder, v0: u32, v1: u32, v2: u32) !void {
        try self.s.addFace(.init(v0, v1, v2, .{ .material_index = self.g.int(0, 2) }));
    }

    /// `n` distinct vertex indices
    fn distinct(self: *Builder, comptime n: usize) [n]u32 {
        var result: [n]u32 = undefined;
        for (0..n) |i| {
            while (true) {
                const v = self.g.int(0, self.numVertices() - 1);
                if (std.mem.indexOfScalar(u32, result[0..i], v) == null) {
                    result[i] = v;
                    break;
                }
            }
        }
        return result;
    }
};

/// Cloth: a grid with alternating, equal or random diagonals, some jitter out of the plane
fn genCloth(b: *Builder) !void {
    const g = b.g;
    const large = g.chance(6);
    const nx = if (large) g.int(17, 32) else g.int(2, 12);
    const nz = if (large) g.int(17, 32) else g.int(2, 12);
    const exact_spacings = [_]f32{ 0.125, 0.25, 0.5, 1.0 };
    const spacing = if (g.chance(50)) exact_spacings[g.int(0, 3)] else g.float(0.05, 1.0);
    const jitter = if (g.chance(50)) 0.0 else g.float(0.0, 0.3) * spacing;
    const kinematic_mode = g.int(0, 3); // Corners, the first row, random vertices or none
    const diagonals = g.int(0, 2); // Alternating, equal or random
    for (0..nz) |z| for (0..nx) |x| {
        const kinematic = switch (kinematic_mode) {
            0 => (x == 0 or x == nx - 1) and (z == 0 or z == nz - 1),
            1 => z == 0,
            2 => g.chance(10),
            else => false,
        };
        const y = if (jitter == 0.0) 0.0 else g.float(-jitter, jitter);
        _ = try b.vertex(spacing * @as(f32, @floatFromInt(x)), y, spacing * @as(f32, @floatFromInt(z)), g.invMass(kinematic));
    };
    for (0..nz - 1) |z| for (0..nx - 1) |x| {
        const v0: u32 = @intCast(z * nx + x);
        const v1 = v0 + 1;
        const v2 = v0 + nx;
        const v3 = v2 + 1;
        const flip = switch (diagonals) {
            0 => (x + z) % 2 == 0,
            1 => false,
            else => g.chance(50),
        };
        if (flip) {
            try b.face(v0, v2, v1);
            try b.face(v1, v2, v3);
        } else {
            try b.face(v0, v3, v1);
            try b.face(v0, v2, v3);
        }
    };
}

/// Closed meshes: a UV sphere or a bipyramid (every edge is shared by exactly 2 faces, so Jolt adds the last edge twice)
fn genSphere(b: *Builder) !void {
    const g = b.g;
    const r = g.float(0.2, 3.0);
    const jitter = if (g.chance(50)) 0.0 else 0.03 * r;
    const random_kinematic = g.chance(30);
    if (g.chance(30)) {
        // Bipyramid over an n-gon
        const n = g.int(3, 16);
        for (0..n) |i| {
            const angle = 2.0 * std.math.pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(n));
            _ = try b.vertex(r * trig.cos(angle) + g.float(-jitter, jitter + 1.0e-30), g.float(-0.1, 0.1), r * trig.sin(angle), g.invMass(random_kinematic and g.chance(15)));
        }
        const top = try b.vertex(0, r, 0, g.invMass(g.chance(30)));
        const bottom = try b.vertex(0, -r, 0, g.invMass(false));
        for (0..n) |i| {
            const v0: u32 = @intCast(i);
            const v1: u32 = @intCast((i + 1) % n);
            try b.face(v0, top, v1);
            try b.face(v1, bottom, v0);
        }
    } else {
        // UV sphere
        const num_lat = g.int(2, 10);
        const num_lon = g.int(3, 16);
        const top = try b.vertex(0, r, 0, g.invMass(g.chance(30)));
        var rings: [9][16]u32 = undefined;
        for (1..num_lat) |i| {
            const theta = std.math.pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_lat));
            for (0..num_lon) |j| {
                const phi = 2.0 * std.math.pi * @as(f32, @floatFromInt(j)) / @as(f32, @floatFromInt(num_lon));
                const ring_r = r * trig.sin(theta);
                rings[i - 1][j] = try b.vertex(ring_r * trig.cos(phi) + g.float(-jitter, jitter + 1.0e-30), r * trig.cos(theta), ring_r * trig.sin(phi), g.invMass(random_kinematic and g.chance(10)));
            }
        }
        const bottom = try b.vertex(0, -r, 0, g.invMass(false));
        for (0..num_lon) |j| {
            const j1 = (j + 1) % num_lon;
            try b.face(top, rings[0][j1], rings[0][j]);
            for (0..num_lat - 2) |i| {
                const a = rings[i][j];
                const c = rings[i][j1];
                const d = rings[i + 1][j1];
                const e = rings[i + 1][j];
                try b.face(a, c, d);
                try b.face(a, d, e);
            }
            try b.face(bottom, rings[num_lat - 2][j], rings[num_lat - 2][j1]);
        }
    }
}

/// The tetrahedra of SoftBodySharedSettings::sCreateCube
const tetra_indices = [6][4][3]u32{
    .{ .{ 0, 0, 0 }, .{ 0, 1, 1 }, .{ 0, 0, 1 }, .{ 1, 1, 1 } },
    .{ .{ 0, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 1, 1 }, .{ 1, 1, 1 } },
    .{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 1 }, .{ 1, 1, 1 } },
    .{ .{ 0, 0, 0 }, .{ 1, 0, 1 }, .{ 1, 0, 0 }, .{ 1, 1, 1 } },
    .{ .{ 0, 0, 0 }, .{ 1, 1, 0 }, .{ 0, 1, 0 }, .{ 1, 1, 1 } },
    .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 1, 1, 0 }, .{ 1, 1, 1 } },
};

/// Tetrahedra: a jittered grid of cubes, 6 tetrahedra per cube with volume constraints, and the 4 faces of every
/// tetrahedron (an inner face is shared by 2 tetrahedra, an edge by many faces) or none
fn genTetrahedra(b: *Builder) !void {
    const g = b.g;
    const n = g.int(2, 4);
    const spacing = g.float(0.1, 1.0);
    const jitter = if (g.chance(50)) 0.0 else 0.2 * spacing;
    for (0..n) |z| for (0..n) |y| for (0..n) |x| {
        _ = try b.vertex(spacing * @as(f32, @floatFromInt(x)) + g.float(-jitter, jitter + 1.0e-30), spacing * @as(f32, @floatFromInt(y)) + g.float(-jitter, jitter + 1.0e-30), spacing * @as(f32, @floatFromInt(z)) + g.float(-jitter, jitter + 1.0e-30), g.invMass(y == 0 and g.chance(20)));
    };
    const with_faces = g.chance(70);
    for (0..n - 1) |z| for (0..n - 1) |y| for (0..n - 1) |x| for (tetra_indices) |t| {
        var v: [4]u32 = undefined;
        for (0..4) |i| v[i] = @intCast((x + t[i][0]) + (y + t[i][1]) * n + (z + t[i][2]) * n * n);
        try b.s.volume_constraints.append(b.allocator, .init(v[0], v[1], v[2], v[3], .{ .compliance = g.compliance() }));
        if (with_faces) {
            try b.face(v[0], v[1], v[2]);
            try b.face(v[0], v[2], v[3]);
            try b.face(v[0], v[3], v[1]);
            try b.face(v[1], v[3], v[2]);
        }
    };
}

/// Appends a rod between v0 and v1 (reversed sometimes), returns its index
fn addRod(b: *Builder, v0: u32, v1: u32) !u32 {
    const g = b.g;
    const index: u32 = @intCast(b.s.rod_stretch_shear_constraints.items.len);
    var rod: S.RodStretchShear = if (g.chance(25)) .init(v1, v0, .{ .compliance = g.compliance() }) else .init(v0, v1, .{ .compliance = g.compliance() });
    rod.length = g.float(0, 2);
    rod.inv_mass = g.float(0, 2);
    if (g.chance(5)) rod.bishop = g.quat(); // Preset: CalculateRodProperties keeps it
    try b.s.rod_stretch_shear_constraints.append(b.allocator, rod);
    return index;
}

/// Appends a bend twist constraint between two different rods
fn addBendTwist(b: *Builder, r0: u32, r1: u32) !void {
    const g = b.g;
    var bend: S.RodBendTwist = if (g.chance(30)) .init(r1, r0, .{ .compliance = g.compliance() }) else .init(r0, r1, .{ .compliance = g.compliance() });
    bend.omega0 = if (g.chance(50)) Quat.zero() else g.quat();
    try b.s.rod_bend_twist_constraints.append(b.allocator, bend);
}

/// Rods: chains of rods along random walks, bend twist constraints between neighbors and branches between chains
fn genRods(b: *Builder) !void {
    const g = b.g;
    const num_chains = g.int(1, 4);
    for (0..num_chains) |_| {
        var p = Vec3.init(g.float(-3, 3), g.float(-3, 3), g.float(-3, 3));
        var prev_vertex = try b.vertex(p.getX(), p.getY(), p.getZ(), g.invMass(g.chance(50)));
        var prev_rod: ?u32 = null;
        var direction = Vec3.init(g.float(-1, 1), g.float(-1, 1), g.float(0.1, 1)).normalized();
        for (0..g.int(1, 15)) |_| {
            // Mostly smooth chains, sometimes a sharp turn
            const turn: f32 = if (g.chance(15)) 2.0 else 0.4;
            direction = direction.add(Vec3.init(g.float(-turn, turn), g.float(-turn, turn), g.float(-turn, turn))).normalizedOr(Vec3.axisY());
            p = p.add(direction.mulScalar(g.float(0.1, 1.0)));
            const v = try b.vertex(p.getX(), p.getY(), p.getZ(), g.invMass(g.chance(5)));
            const rod = try addRod(b, prev_vertex, v);
            if (prev_rod) |r| try addBendTwist(b, r, rod);
            prev_rod = rod;
            prev_vertex = v;
        }
    }

    // Branches
    const num_rods: u32 = @intCast(b.s.rod_stretch_shear_constraints.items.len);
    if (num_rods >= 2) {
        for (0..g.int(0, 3)) |_| {
            const r0 = g.int(0, num_rods - 1);
            var r1 = g.int(0, num_rods - 1);
            while (r1 == r0) r1 = g.int(0, num_rods - 1);
            try addBendTwist(b, r0, r1);
        }
    }

    // Some edges (Dijkstra and Optimize use them too)
    if (b.numVertices() >= 2 and g.chance(30)) {
        for (0..g.int(1, 5)) |_| {
            const v = b.distinct(2);
            try b.s.edge_constraints.append(b.allocator, .init(v[0], v[1], .{ .compliance = g.compliance() }));
        }
    }
}

/// Skinned meshes: a cloth or a sphere with skinned constraints for some or all vertices
fn genSkinned(b: *Builder) !void {
    const g = b.g;
    if (g.chance(70)) try genCloth(b) else try genSphere(b);
    const num_inv_binds = g.int(1, 4);
    for (0..num_inv_binds) |_| try b.s.inv_bind_matrices.append(b.allocator, .init(g.int(0, 10), g.mat44()));
    const all = g.chance(40);
    var v: u32 = 0;
    while (v < b.numVertices()) : (v += 1) {
        if (!all and !g.chance(60)) continue;
        for (0..@as(u32, if (g.chance(10)) 2 else 1)) |_| { // Sometimes 2 constraints for one vertex
            var skinned: S.Skinned = .init(v, if (g.chance(30)) flt_max else g.float(0, 1), if (g.chance(30)) flt_max else g.float(-0.5, 0.5), g.float(0.1, 50));
            for (0..g.int(1, 4)) |k| skinned.weights[k] = .init(g.int(0, num_inv_binds - 1), g.float(0, 1));
            if (g.chance(50)) skinned.normalizeWeights();
            skinned.normal_info = g.next();
            try b.s.skinned_constraints.append(b.allocator, skinned);
        }
    }
}

/// Triangle soups: fans of many faces on one edge (more than 32 sometimes), back to back and duplicated faces
fn genSoup(b: *Builder) !void {
    const g = b.g;
    const big_fan = g.chance(30);
    const fan_size = if (big_fan) g.int(33, 80) else 0;
    const nv = g.int(4, 40) + fan_size;
    for (0..nv) |_| _ = try b.vertex(g.float(-2, 2), g.float(-2, 2), g.float(-2, 2), g.invMass(g.chance(15)));

    // A fan of faces around the edge 0 - 1
    for (0..fan_size) |i| {
        const opposite: u32 = @intCast(2 + i);
        if (g.chance(50)) try b.face(0, 1, opposite) else try b.face(1, 0, opposite);
    }

    const nf = g.int(1, 120);
    for (0..nf) |_| {
        switch (g.int(0, 5)) {
            0 => if (b.s.faces.items.len != 0) {
                // A face on an edge of an existing face (fans)
                const f = b.s.faces.items[g.int(0, @intCast(b.s.faces.items.len - 1))];
                const e = g.int(0, 2);
                const v0 = f.vertex[e];
                const v1 = f.vertex[(e + 1) % 3];
                var v2 = g.int(0, nv - 1);
                while (v2 == v0 or v2 == v1) v2 = g.int(0, nv - 1);
                try b.face(v1, v0, v2);
            },
            1 => if (b.s.faces.items.len != 0) {
                // Back to back (the same triangle, reversed) or a duplicate
                const f = b.s.faces.items[g.int(0, @intCast(b.s.faces.items.len - 1))];
                if (g.chance(50)) try b.face(f.vertex[0], f.vertex[2], f.vertex[1]) else try b.face(f.vertex[0], f.vertex[1], f.vertex[2]);
            },
            else => {
                const v = b.distinct(3);
                try b.face(v[0], v[1], v[2]);
            },
        }
    }
}

/// Lattice graphs: vertices on an integer lattice, edges and rods between neighbors, several kinematic vertices. Many
/// vertices have the same distance to two kinematic vertices: Dijkstra's heap decides.
fn genGraph(b: *Builder) !void {
    const g = b.g;
    const nx = g.int(2, 8);
    const ny = g.int(2, 8);
    const nz = g.int(1, 3);
    const spacing: f32 = if (g.chance(50)) 1.0 else 0.5;
    var index: [8 * 8 * 3]?u32 = @splat(null);
    for (0..nz) |z| for (0..ny) |y| for (0..nx) |x| {
        if (g.chance(85)) index[x + nx * (y + ny * z)] = try b.vertex(spacing * @as(f32, @floatFromInt(x)), spacing * @as(f32, @floatFromInt(y)), spacing * @as(f32, @floatFromInt(z)), g.invMass(false));
    };
    if (b.numVertices() < 2) _ = try b.vertex(-1, -1, -1, 1.0);

    // Edges and rods between lattice neighbors
    for (0..nz) |z| for (0..ny) |y| for (0..nx) |x| {
        const v0 = index[x + nx * (y + ny * z)] orelse continue;
        const neighbors = [_][3]usize{ .{ x + 1, y, z }, .{ x, y + 1, z }, .{ x, y, z + 1 }, .{ x + 1, y + 1, z } };
        for (neighbors, 0..) |n, k| {
            if (n[0] >= nx or n[1] >= ny or n[2] >= nz) continue;
            const v1 = index[n[0] + nx * (n[1] + ny * n[2])] orelse continue;
            if (!g.chance(if (k == 3) 10 else 80)) continue;
            if (g.chance(10))
                _ = try addRod(b, v0, v1)
            else
                try b.s.edge_constraints.append(b.allocator, .init(v0, v1, .{ .compliance = g.compliance() }));
        }
    };

    // Kinematic vertices
    if (g.chance(90)) {
        for (0..g.int(1, 5)) |_| b.s.vertices.items[g.int(0, b.numVertices() - 1)].inv_mass = 0.0;
    }
}

/// Copies of 1 to 3 cloths side by side along x. A copy has 132 to 196 vertices: more than half and less than all of
/// a parallel group (SoftBodyUpdateContext.vertex_constraint_batch), and no constraint joins two copies, so Optimize
/// makes one parallel group per copy. Copies of one cloth get the same constraints, so their groups have equal size.
fn genCopies(b: *Builder) !void {
    const g = b.g;
    const shapes = [_][2]u32{ .{ 12, 12 }, .{ 11, 13 }, .{ 13, 12 }, .{ 13, 13 }, .{ 14, 14 }, .{ 12, 16 } };
    var copy_shapes: [3][2]u32 = undefined;
    const num_shapes = g.int(1, 3);
    for (copy_shapes[0..num_shapes]) |*shape| shape.* = shapes[g.int(0, shapes.len - 1)];
    const num_copies = g.int(33, 44);
    const spacing: f32 = if (g.chance(50)) 0.5 else 1.0; // Exact positions: the copies are equal up to a translation
    const kinematic_mode = g.int(0, 2); // A corner, the first row or none
    const alternating = g.chance(50); // Alternating or equal diagonals
    var x0: u32 = 0;
    for (0..num_copies) |_| {
        const shape = copy_shapes[g.int(0, num_shapes - 1)];
        const nx = shape[0];
        const nz = shape[1];
        const first = b.numVertices();
        for (0..nz) |z| for (0..nx) |x| {
            const kinematic = switch (kinematic_mode) {
                0 => x == 0 and z == 0,
                1 => z == 0,
                else => false,
            };
            _ = try b.vertex(spacing * @as(f32, @floatFromInt(x0 + x)), 0.0, spacing * @as(f32, @floatFromInt(z)), g.invMass(kinematic));
        };
        for (0..nz - 1) |z| for (0..nx - 1) |x| {
            const v0: u32 = first + @as(u32, @intCast(z * nx + x));
            const v1 = v0 + 1;
            const v2 = v0 + nx;
            const v3 = v2 + 1;
            if (alternating and (x + z) % 2 == 0) {
                try b.face(v0, v2, v1);
                try b.face(v1, v2, v3);
            } else {
                try b.face(v0, v3, v1);
                try b.face(v0, v2, v3);
            }
        };
        x0 += nx + 2;
    }
}

/// A case of `genCopies`: one attribute for all vertices (so copies of one cloth get the same constraints),
/// CreateConstraints, then Optimize
fn generateCopies(allocator: Allocator, g: *Gen) !Case {
    var c: Case = .{ .kind = .cloth };
    errdefer c.deinit(allocator);
    var settings: Ref(S) = .init(try S.create(allocator));
    defer settings.deinit();
    var b: Builder = .{ .allocator = allocator, .s = settings.get().?, .g = g };
    try genCopies(&b);
    var empty: Ref(S) = .init(try S.create(allocator));
    defer empty.deinit();
    try genAttributes(allocator, &c, g, 1);
    try c.ops.append(allocator, createConstraintsOp(g));
    if (g.chance(30)) try c.ops.append(allocator, .{ .kind = .clone });
    try c.ops.append(allocator, .{ .kind = if (g.chance(75)) .optimize else .optimize_no_results });
    if (g.chance(30)) try c.ops.append(allocator, .{ .kind = .optimize }); // Again on the sorted constraints
    c.settings_bytes = try saveBytes(allocator, b.s);
    c.extra_bytes = try saveBytes(allocator, empty.get().?);
    return c;
}

/// Constraints that exist before the operations (CreateConstraints clears the edges and appends to the others)
fn addExistingConstraints(b: *Builder) !void {
    const g = b.g;
    if (b.numVertices() < 4) return;
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(2);
        var e: S.Edge = .init(v[0], v[1], .{ .compliance = g.compliance() });
        e.rest_length = g.float(0, 3);
        try b.s.edge_constraints.append(b.allocator, e);
    }
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(4);
        var d: S.DihedralBend = .init(v[0], v[1], v[2], v[3], .{ .compliance = g.compliance() });
        d.initial_angle = g.float(-3, 3);
        try b.s.dihedral_bend_constraints.append(b.allocator, d);
    }
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(2);
        try b.s.lra_constraints.append(b.allocator, .init(v[0], v[1], g.float(0, 5)));
    }
    for (0..g.int(0, 2)) |_| {
        const v = b.distinct(4);
        try b.s.volume_constraints.append(b.allocator, .init(v[0], v[1], v[2], v[3], .{ .compliance = g.compliance() }));
    }
    if (g.chance(30)) {
        const v = b.distinct(3);
        const r0 = try addRod(b, v[0], v[1]);
        const r1 = try addRod(b, v[1], v[2]);
        try addBendTwist(b, r0, r1);
    }
}

/// The settings that AppendExtra appends: constraints on the vertices of `b` (the rod indices of the bend twist
/// constraints count the rods of `b` first)
fn genExtra(allocator: Allocator, b: *Builder) !Ref(S) {
    const g = b.g;
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    const x = ref.get().?;
    if (b.numVertices() < 4) return ref;
    for (0..g.int(0, 4)) |_| {
        const v = b.distinct(3);
        try x.faces.append(allocator, .init(v[0], v[1], v[2], .{}));
    }
    for (0..g.int(0, 8)) |_| {
        const v = b.distinct(2);
        try x.edge_constraints.append(allocator, .init(v[0], v[1], .{ .compliance = g.compliance() }));
    }
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(4);
        try x.dihedral_bend_constraints.append(allocator, .init(v[0], v[1], v[2], v[3], .{ .compliance = g.compliance() }));
    }
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(4);
        try x.volume_constraints.append(allocator, .init(v[0], v[1], v[2], v[3], .{}));
    }
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(2);
        try x.lra_constraints.append(allocator, .init(v[0], v[1], g.float(0.1, 5)));
    }
    for (0..g.int(0, 2)) |_| {
        var skinned: S.Skinned = .init(g.int(0, b.numVertices() - 1), 0.5, flt_max, 40.0);
        skinned.weights[0] = .init(0, 1.0);
        try x.skinned_constraints.append(allocator, skinned);
    }
    if (g.chance(30)) try x.inv_bind_matrices.append(allocator, .init(1, g.mat44()));
    const base_rods: u32 = @intCast(b.s.rod_stretch_shear_constraints.items.len);
    for (0..g.int(0, 3)) |_| {
        const v = b.distinct(2);
        try x.rod_stretch_shear_constraints.append(allocator, .init(v[0], v[1], .{ .compliance = g.compliance() }));
    }
    const num_rods = base_rods + @as(u32, @intCast(x.rod_stretch_shear_constraints.items.len));
    if (num_rods >= 2) {
        for (0..g.int(0, 2)) |_| {
            const r0 = g.int(0, num_rods - 1);
            var r1 = g.int(0, num_rods - 1);
            while (r1 == r0) r1 = g.int(0, num_rods - 1);
            try x.rod_bend_twist_constraints.append(allocator, .init(r0, r1, .{}));
        }
    }
    return ref;
}

/// Vertex attributes: one for all vertices, one per vertex, or fewer (the last one repeats)
fn genAttributes(allocator: Allocator, c: *Case, g: *Gen, num_vertices: u32) !void {
    const n = switch (g.int(0, 2)) {
        0 => 1,
        1 => num_vertices,
        else => g.int(1, @max(num_vertices, 1)),
    };
    const lra_types: u32 = switch (g.int(0, 3)) {
        0 => 0, // No LRA
        1 => 1, // Euclidean
        2 => 2, // Geodesic
        else => 3, // Mixed
    };
    for (0..n) |_| {
        try c.attributes.append(allocator, .{
            .compliance = g.compliance(),
            .shear_compliance = g.compliance(),
            .bend_compliance = g.compliance(),
            .lra_type = @intCast(if (lra_types == 3) g.int(0, 2) else lra_types),
            .lra_max_distance_multiplier = if (g.chance(50)) 1.0 else g.float(0.5, 2.0),
        });
    }
}

/// A CreateConstraints operation with a random bend type and angle tolerance (or Jolt's default arguments)
fn createConstraintsOp(g: *Gen) Op {
    if (g.chance(15)) return .{ .kind = .create_constraints_defaults };
    const tolerance = if (g.chance(50)) zolt.math.degreesToRadians(8.0) else g.float(0.0, 0.8);
    return .{ .kind = .create_constraints, .bend_type = g.int(0, 2), .value = tolerance };
}

/// A random Calculate* operation
fn calculateOp(g: *Gen) Op {
    return switch (g.int(0, 5)) {
        0 => .{ .kind = .calculate_edge_lengths },
        1 => .{ .kind = .calculate_lra_lengths, .value = if (g.chance(50)) 1.0 else g.float(0.5, 2.0) },
        2 => .{ .kind = .calculate_bend_constraint_constants },
        3 => .{ .kind = .calculate_volume_constraint_volumes },
        4 => .{ .kind = .calculate_skinned_constraint_normals },
        else => .{ .kind = .calculate_rod_properties },
    };
}

fn genOps(allocator: Allocator, c: *Case, g: *Gen) !void {
    const ops = &c.ops;
    if (c.kind == .graph or g.chance(10)) try ops.append(allocator, .{ .kind = .calculate_closest_kinematic });
    switch (c.kind) {
        .rods => if (g.chance(90)) try ops.append(allocator, .{ .kind = .calculate_rod_properties }),
        .tetrahedra => if (g.chance(80)) try ops.append(allocator, .{ .kind = .calculate_volume_constraint_volumes }),
        .skinned => if (g.chance(90)) try ops.append(allocator, .{ .kind = .calculate_skinned_constraint_normals }),
        else => {},
    }
    const create_percent: u32 = switch (c.kind) {
        .rods, .graph => 30,
        else => 90,
    };
    if (g.chance(create_percent)) {
        try ops.append(allocator, createConstraintsOp(g));
        if (g.chance(15)) try ops.append(allocator, createConstraintsOp(g)); // Twice: the bends and LRAs are appended again
    }
    if (g.chance(15)) try ops.append(allocator, .{ .kind = .append_extra }); // Constraints added after the cache was filled
    if (g.chance(25)) {
        for (0..g.int(1, 3)) |_| try ops.append(allocator, calculateOp(g));
    }
    if (g.chance(15)) try ops.append(allocator, .{ .kind = .clone });
    if (g.chance(70)) {
        try ops.append(allocator, .{ .kind = if (g.chance(80)) .optimize else .optimize_no_results });
        if (g.chance(15)) try ops.append(allocator, .{ .kind = .optimize }); // Again: the remaps are resized
        if (c.kind == .skinned and g.chance(30)) try ops.append(allocator, .{ .kind = .calculate_skinned_constraint_normals });
    }
    if (g.chance(20)) try ops.append(allocator, .{ .kind = .clone });
}

fn generate(allocator: Allocator, g: *Gen, kind: Kind) !Case {
    var c: Case = .{ .kind = kind };
    errdefer c.deinit(allocator);
    var settings: Ref(S) = .init(try S.create(allocator));
    defer settings.deinit();
    var b: Builder = .{ .allocator = allocator, .s = settings.get().?, .g = g };
    switch (kind) {
        .cloth => try genCloth(&b),
        .sphere => try genSphere(&b),
        .tetrahedra => try genTetrahedra(&b),
        .rods => try genRods(&b),
        .skinned => try genSkinned(&b),
        .soup => try genSoup(&b),
        .graph => try genGraph(&b),
    }
    if (g.chance(20)) try addExistingConstraints(&b);
    var extra = try genExtra(allocator, &b);
    defer extra.deinit();
    try genAttributes(allocator, &c, g, b.numVertices());
    try genOps(allocator, &c, g);
    c.settings_bytes = try saveBytes(allocator, b.s);
    c.extra_bytes = try saveBytes(allocator, extra.get().?);
    return c;
}

/// Counts the hard cases that the generators reached (on the Zolt side)
const Coverage = struct {
    cases: [@typeInfo(Kind).@"enum".fields.len]u32 = @splat(0),
    cube_cases: u32 = 0,
    /// CreateConstraints added the same edge as its last two edges (a closed mesh)
    duplicated_last_edges: u32 = 0,
    /// CreateConstraints sorted more than 32 edges (QuickSort, not only its insertion sort) with equal keys
    quick_sorts_with_ties: u32 = 0,
    /// CreateConstraints sorted more than 32 equal keys
    runs_over_32: u32 = 0,
    /// Shared edges with 3 or more faces
    edges_with_3_faces: u32 = 0,
    /// Every combination of (zero, FLT_MAX, other) for the 3 compliances and the 3 LRA types
    attribute_combinations: [81]bool = @splat(false),
    /// Bend type x LRA (none, some) x Optimize
    bend_lra_optimize: [3][2][2]u32 = @splat(@splat(@splat(0))),
    dihedral_bends: u32 = 0,
    lra_constraints: u32 = 0,
    /// Optimize with several parallel groups, with a non-empty non-parallel group, with parallel groups of equal size
    multi_group_optimizes: u32 = 0,
    non_parallel_constraints: u32 = 0,
    group_size_ties: u32 = 0,
    /// Optimize sorted more than 32 parallel groups, some of equal size (QuickSort partitions them, the copies test)
    group_size_ties_over_32: u32 = 0,
    /// Optimize used a closest kinematic cache that was filled before (by CreateConstraints or the private function)
    cached_optimizes: u32 = 0,
    /// Constraints were appended after the cache was filled, then Optimize used it
    stale_cache_optimizes: u32 = 0,
    /// Vertices with the same distance to two kinematic vertices (Dijkstra's heap decides) in a closest kinematic run
    dijkstra_ties: u32 = 0,
    /// CalculateRodProperties flipped a rod
    flipped_rods: u32 = 0,
    /// CalculateSkinnedConstraintNormals found faces
    skinned_normals: u32 = 0,
    clones: u32 = 0,

    fn check(self: *const Coverage) !void {
        // The cube test counts its cases itself (cube_cases)
        std.debug.print("SoftBodySettings coverage: cases {any}, duplicated last edges {d}, QuickSorts with ties {d}, runs over 32 {d}, edges with 3+ faces {d}, dihedral bends {d}, LRAs {d}, multi group optimizes {d}, non-parallel constraints {d}, group size ties {d}, cached optimizes {d}, stale cache optimizes {d}, Dijkstra ties {d}, flipped rods {d}, skinned normals {d}, clones {d}\n", .{
            self.cases, self.duplicated_last_edges, self.quick_sorts_with_ties, self.runs_over_32, self.edges_with_3_faces, self.dihedral_bends, self.lra_constraints, self.multi_group_optimizes, self.non_parallel_constraints, self.group_size_ties, self.cached_optimizes, self.stale_cache_optimizes, self.dijkstra_ties, self.flipped_rods, self.skinned_normals, self.clones,
        });
        try expect(self.duplicated_last_edges > 50);
        try expect(self.quick_sorts_with_ties > 500);
        try expect(self.runs_over_32 > 50);
        try expect(self.edges_with_3_faces > 200);
        for (self.attribute_combinations) |seen| try expect(seen);
        for (self.bend_lra_optimize) |a| for (a) |b| for (b) |n| try expect(n > 5);
        try expect(self.dihedral_bends > 1000);
        try expect(self.lra_constraints > 1000);
        try expect(self.multi_group_optimizes > 20);
        try expect(self.non_parallel_constraints > 20);
        try expect(self.group_size_ties > 3);
        try expect(self.cached_optimizes > 200);
        try expect(self.stale_cache_optimizes > 20);
        try expect(self.dijkstra_ties > 100);
        try expect(self.flipped_rods > 50);
        try expect(self.skinned_normals > 100);
        try expect(self.clones > 200);
    }

    /// The classes of the attributes and the bend / LRA / Optimize combinations of a case
    fn addCase(self: *Coverage, c: *const Case) void {
        self.cases[@intFromEnum(c.kind)] += 1;
        var has_lra = false;
        for (c.attributes.items) |a| {
            const class = struct {
                fn of(v: f32) usize {
                    return if (v == 0.0) 0 else if (v == flt_max) 1 else 2;
                }
            }.of;
            self.attribute_combinations[((class(a.compliance) * 3 + class(a.shear_compliance)) * 3 + class(a.bend_compliance)) * 3 + @as(usize, @intCast(a.lra_type))] = true;
            if (a.lra_type != 0) has_lra = true;
        }
        const optimized = c.hasOp(.optimize) or c.hasOp(.optimize_no_results);
        for (c.ops.items) |op| {
            if (op.kind == .create_constraints)
                self.bend_lra_optimize[op.bend_type][@intFromBool(has_lra)][@intFromBool(optimized)] += 1;
        }
    }

    /// The equal keys of the edge sort of CreateConstraints on `s`
    fn addEdgeSort(self: *Coverage, allocator: Allocator, s: *const S) !void {
        const keys = try allocator.alloc(u64, 3 * s.faces.items.len);
        defer allocator.free(keys);
        for (s.faces.items, 0..) |f, i| {
            for (0..3) |k| {
                const v0 = f.vertex[k];
                const v1 = f.vertex[(k + 1) % 3];
                keys[3 * i + k] = @as(u64, @min(v0, v1)) << 32 | @max(v0, v1);
            }
        }
        std.mem.sort(u64, keys, {}, std.sort.asc(u64));
        var longest: usize = 0;
        var run: usize = 0;
        for (keys, 0..) |k, i| {
            run = if (i > 0 and keys[i - 1] == k) run + 1 else 1;
            if (run == 3) self.edges_with_3_faces += 1;
            longest = @max(longest, run);
        }
        if (keys.len > 32 and longest >= 2) self.quick_sorts_with_ties += 1;
        if (longest > 32) self.runs_over_32 += 1;
    }

    /// The vertices with the same shortest distance to two kinematic vertices (a Dijkstra per kinematic vertex over the
    /// edges and rods of `s`, with the same float sums along a path as Jolt)
    fn addDijkstraTies(self: *Coverage, allocator: Allocator, s: *const S) !void {
        const n = s.vertices.items.len;
        var num_kinematic: usize = 0;
        for (s.vertices.items) |v| num_kinematic += @intFromBool(v.inv_mass == 0.0);
        if (n > 200 or num_kinematic > 8) return;
        const best = try allocator.alloc(f32, n);
        defer allocator.free(best);
        const count = try allocator.alloc(u32, n);
        defer allocator.free(count);
        const dist = try allocator.alloc(f32, n);
        defer allocator.free(dist);
        const done = try allocator.alloc(bool, n);
        defer allocator.free(done);
        @memset(best, flt_max);
        @memset(count, 0);
        for (s.vertices.items, 0..) |kv, k| {
            if (kv.inv_mass != 0.0) continue;
            @memset(dist, flt_max);
            @memset(done, false);
            dist[k] = 0.0;
            while (true) {
                var current: ?usize = null;
                for (0..n) |v| {
                    if (!done[v] and dist[v] < flt_max and (current == null or dist[v] < dist[current.?])) current = v;
                }
                const cur = current orelse break;
                done[cur] = true;
                const relax = struct {
                    fn f(settings: *const S, d: []f32, from: usize, a: u32, b: u32) void {
                        const to: usize = if (a == from) b else if (b == from) a else return;
                        const length = Vec3.fromFloat3(settings.vertices.items[to].position).sub(Vec3.fromFloat3(settings.vertices.items[from].position)).length();
                        const nd = d[from] + length;
                        if (nd < d[to]) d[to] = nd;
                    }
                }.f;
                for (s.edge_constraints.items) |e| relax(s, dist, cur, e.vertex[0], e.vertex[1]);
                for (s.rod_stretch_shear_constraints.items) |r| relax(s, dist, cur, r.vertex[0], r.vertex[1]);
            }
            for (0..n) |v| {
                if (dist[v] < best[v]) {
                    best[v] = dist[v];
                    count[v] = 1;
                } else if (dist[v] == best[v] and dist[v] < flt_max) {
                    count[v] += 1;
                }
            }
        }
        for (count) |c| {
            if (c >= 2) self.dijkstra_ties += 1;
        }
    }

    /// Several parallel groups, non-parallel constraints, parallel groups of equal size
    fn addOptimize(self: *Coverage, s: *const S) void {
        const groups = s.update_groups.items;
        if (groups.len > 2) self.multi_group_optimizes += 1;
        if (groups.len < 2) return;
        const size = struct {
            fn of(gs: []const S.UpdateGroup, i: usize) u32 {
                const g = gs[i];
                var total = g.edge_end_index + g.lra_end_index + g.rod_stretch_shear_end_index + g.rod_bend_twist_end_index + g.dihedral_bend_end_index + g.volume_end_index + g.skinned_end_index;
                if (i > 0) {
                    const p = gs[i - 1];
                    total -= p.edge_end_index + p.lra_end_index + p.rod_stretch_shear_end_index + p.rod_bend_twist_end_index + p.dihedral_bend_end_index + p.volume_end_index + p.skinned_end_index;
                }
                return total;
            }
        }.of;
        if (size(groups, groups.len - 1) > 0) self.non_parallel_constraints += 1;
        for (1..groups.len - 1) |i| {
            if (size(groups, i) == size(groups, i - 1)) {
                self.group_size_ties += 1;
                if (groups.len - 1 > 32) self.group_size_ties_over_32 += 1;
                break;
            }
        }
    }
};

/// Runs the operations of `c` on Zolt's settings and writes the transcript
fn zoltRun(allocator: Allocator, c: *const Case, coverage: *Coverage, t: *Transcript) !void {
    var settings: Ref(S) = if (c.cube_grid_size != 0) try S.createCube(allocator, c.cube_grid_size, c.cube_spacing) else try restore(allocator, c.settings_bytes);
    defer settings.deinit();
    var extra = try restore(allocator, c.extra_bytes);
    defer extra.deinit();
    const s = settings.get().?;
    const x = extra.get().?;

    const attributes = try allocator.alloc(S.VertexAttributes, c.attributes.items.len);
    defer allocator.free(attributes);
    for (attributes, c.attributes.items) |*z, j| z.* = j.toZolt();

    var results: S.OptimizationResults = .{}; // Shared by every Optimize of the run, like the C++ side
    defer results.deinit(allocator);
    var appended_after_cache = false;
    for (c.ops.items, 0..) |op, i| {
        try t.writeU32(op_marker + @as(u32, @intCast(i)));
        switch (op.kind) {
            .create_constraints, .create_constraints_defaults => {
                try coverage.addEdgeSort(allocator, s);
                if (op.kind == .create_constraints)
                    try s.createConstraints(attributes, .{ .bend_type = @enumFromInt(op.bend_type), .angle_tolerance = op.value })
                else
                    try s.createConstraints(attributes, .{});
                const e = s.edge_constraints.items;
                if (e.len >= 2 and std.mem.eql(u32, &e[e.len - 1].vertex, &e[e.len - 2].vertex)) coverage.duplicated_last_edges += 1;
                coverage.dihedral_bends += @intCast(s.dihedral_bend_constraints.items.len);
                coverage.lra_constraints += @intCast(s.lra_constraints.items.len);
            },
            .calculate_edge_lengths => s.calculateEdgeLengths(),
            .calculate_rod_properties => {
                const before = try allocator.dupe(S.RodStretchShear, s.rod_stretch_shear_constraints.items);
                defer allocator.free(before);
                try s.calculateRodProperties();
                for (before, s.rod_stretch_shear_constraints.items) |r0, r1| {
                    if (r0.vertex[0] != r1.vertex[0]) coverage.flipped_rods += 1;
                }
            },
            .calculate_lra_lengths => s.calculateLRALengths(.{ .max_distance_multiplier = op.value }),
            .calculate_bend_constraint_constants => s.calculateBendConstraintConstants(),
            .calculate_volume_constraint_volumes => s.calculateVolumeConstraintVolumes(),
            .calculate_skinned_constraint_normals => {
                try s.calculateSkinnedConstraintNormals();
                if (s.skinned_constraint_normals.items.len > 0) coverage.skinned_normals += 1;
            },
            .optimize, .optimize_no_results => {
                if (s.closest_kinematic.items.len != 0) {
                    coverage.cached_optimizes += 1;
                    if (appended_after_cache) coverage.stale_cache_optimizes += 1;
                }
                if (op.kind == .optimize) {
                    try s.optimizeWithResults(&results);
                    try t.writeRemap(results.edge_remap.items);
                    try t.writeRemap(results.lra_remap.items);
                    try t.writeRemap(results.rod_stretch_shear_constraint_remap.items);
                    try t.writeRemap(results.rod_bend_twist_constraint_remap.items);
                    try t.writeRemap(results.dihedral_bend_remap.items);
                    try t.writeRemap(results.volume_remap.items);
                    try t.writeRemap(results.skinned_remap.items);
                } else {
                    try s.optimize();
                }
                coverage.addOptimize(s);
                appended_after_cache = false;
            },
            .calculate_closest_kinematic => {
                if (s.closest_kinematic.items.len == 0) try coverage.addDijkstraTies(allocator, s);
                try s.calculateClosestKinematic();
            },
            .clone => {
                var clone = try s.clone(allocator);
                defer clone.deinit();
                try t.writeSettings(clone.get().?);
                try t.writeU32(@intCast(clone.get().?.materials.items.len));
                try t.writeU32(@intCast(clone.get().?.closest_kinematic.items.len));
                coverage.clones += 1;
            },
            .append_extra => {
                if (s.closest_kinematic.items.len != 0) appended_after_cache = true;
                try s.faces.appendSlice(allocator, x.faces.items);
                try s.edge_constraints.appendSlice(allocator, x.edge_constraints.items);
                try s.dihedral_bend_constraints.appendSlice(allocator, x.dihedral_bend_constraints.items);
                try s.volume_constraints.appendSlice(allocator, x.volume_constraints.items);
                try s.skinned_constraints.appendSlice(allocator, x.skinned_constraints.items);
                try s.inv_bind_matrices.appendSlice(allocator, x.inv_bind_matrices.items);
                try s.lra_constraints.appendSlice(allocator, x.lra_constraints.items);
                try s.rod_stretch_shear_constraints.appendSlice(allocator, x.rod_stretch_shear_constraints.items);
                try s.rod_bend_twist_constraints.appendSlice(allocator, x.rod_bend_twist_constraints.items);
            },
        }
        try t.writeClosestKinematic(s);
    }
    try t.writeSettings(s);
}

/// Transcript mismatches of one test, the first few are printed with the operation that differs
const TranscriptChecker = struct {
    name: []const u8,
    mismatches: usize = 0,
    jolt_asserts: usize = 0,

    fn check(self: *TranscriptChecker, index: usize, c: *const Case, zolt_bytes: []const u8, jolt_bytes: []const u8, jolt_asserts: u32) void {
        if (jolt_asserts != 0) {
            if (self.mismatches < 5) std.debug.print("{s}: case {d} ({s}) hit {d} Jolt asserts\n", .{ self.name, index, @tagName(c.kind), jolt_asserts });
            self.jolt_asserts += jolt_asserts;
            self.mismatches += 1;
        }
        if (std.mem.eql(u8, zolt_bytes, jolt_bytes)) return;
        if (self.mismatches < 5) {
            const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
            // The last operation marker before the difference (the transcript is a sequence of 4 byte values)
            var op: ?u32 = null;
            var pos: usize = 0;
            while (pos + 4 <= @min(at + 1, jolt_bytes.len)) : (pos += 4) {
                const v = std.mem.readInt(u32, jolt_bytes[pos..][0..4], .little);
                if (v & 0xFFFF0000 == op_marker and v & 0xFFFF < c.ops.items.len) op = v & 0xFFFF;
            }
            std.debug.print("{s}: case {d} ({s}, cube {d}) differs at byte {d} of {d} / {d}, after operation {?d}; operations:", .{ self.name, index, @tagName(c.kind), c.cube_grid_size, at, zolt_bytes.len, jolt_bytes.len, op });
            for (c.ops.items) |o| std.debug.print(" {s}", .{@tagName(o.kind)});
            std.debug.print("\n", .{});
        }
        self.mismatches += 1;
    }

    fn finish(self: *const TranscriptChecker) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches ({d} Jolt asserts)\n", .{ self.name, self.mismatches, self.jolt_asserts });
            return error.ParityMismatch;
        }
    }
};

/// Runs `c` on both sides and compares the transcripts
fn runCase(allocator: Allocator, checker: *TranscriptChecker, coverage: *Coverage, c: *const Case, index: usize, buffer: []u8) !void {
    // Jolt
    const asserts_before = ParityHooks.assertCount();
    const size = jolt.jolt_sbs_run(c.settings_bytes.ptr, @intCast(c.settings_bytes.len), c.extra_bytes.ptr, @intCast(c.extra_bytes.len), c.attributes.items.ptr, @intCast(c.attributes.items.len), c.ops.items.ptr, @intCast(c.ops.items.len), c.cube_grid_size, c.cube_spacing, buffer.ptr, @intCast(buffer.len));
    const jolt_asserts = ParityHooks.assertCount() - asserts_before;
    if (size == 0xFFFFFFFF) return error.TranscriptTooLarge;

    // Zolt
    var t: Transcript = .{ .allocator = allocator };
    defer t.deinit();
    try zoltRun(allocator, c, coverage, &t);
    checker.check(index, c, t.bytes.items, buffer[0..size], jolt_asserts);
}

const num_cases = 3500;

test "SoftBodySettings parity: struct sizes match the C++" {
    var sizes: [2]u32 = undefined;
    jolt.jolt_sbs_struct_sizes(&sizes);
    try std.testing.expectEqual([2]u32{ @sizeOf(VertexAttributes), @sizeOf(Op) }, sizes);
}

test "SoftBodySettings parity: generated cloths, spheres, tetrahedra, rods, skinned meshes, soups and graphs" {
    const allocator = std.testing.allocator;
    var checker: TranscriptChecker = .{ .name = "SoftBodySettings" };
    var coverage: Coverage = .{};
    var g: Gen = .{ .rng = .{ .state = 0x51f7c0de } };
    const buffer = try allocator.alloc(u8, transcript_capacity);
    defer allocator.free(buffer);
    const kinds = std.enums.values(Kind);
    for (0..num_cases) |index| {
        var c = try generate(allocator, &g, kinds[index % kinds.len]);
        defer c.deinit(allocator);
        coverage.addCase(&c);
        try runCase(allocator, &checker, &coverage, &c, index, buffer);
    }
    try checker.finish();
    try coverage.check();
}

test "SoftBodySettings parity: createCube for grid sizes 2 to 8" {
    const allocator = std.testing.allocator;
    var checker: TranscriptChecker = .{ .name = "SoftBodySettings createCube" };
    var coverage: Coverage = .{};
    var g: Gen = .{ .rng = .{ .state = 0xc0be5eed } };
    const buffer = try allocator.alloc(u8, transcript_capacity);
    defer allocator.free(buffer);
    const spacings = [_]f32{ 0.05, 0.1, 0.25, 0.3, 1.0 };
    var index: usize = 0;
    for (2..9) |grid_size| for (spacings) |spacing| {
        var c: Case = .{ .kind = .cloth, .cube_grid_size = @intCast(grid_size), .cube_spacing = spacing };
        defer c.deinit(allocator);
        // The extra settings: an empty stream of settings (AppendExtra appends nothing)
        var empty: Ref(S) = .init(try S.create(allocator));
        defer empty.deinit();
        c.extra_bytes = try saveBytes(allocator, empty.get().?);
        try genAttributes(allocator, &c, &g, @intCast(grid_size * grid_size * grid_size));
        // Every case compares the bytes of sCreateCube; some run more operations on the cube
        switch (index % 4) {
            0 => {},
            1 => try c.ops.appendSlice(allocator, &.{ .{ .kind = .clone }, .{ .kind = .calculate_closest_kinematic }, .{ .kind = .optimize } }),
            2 => try c.ops.appendSlice(allocator, &.{ .{ .kind = .calculate_edge_lengths }, .{ .kind = .calculate_volume_constraint_volumes }, .{ .kind = .optimize }, .{ .kind = .clone } }),
            else => try c.ops.appendSlice(allocator, &.{ createConstraintsOp(&g), .{ .kind = .optimize }, .{ .kind = .calculate_skinned_constraint_normals } }),
        }
        try runCase(allocator, &checker, &coverage, &c, index, buffer);
        coverage.cube_cases += 1;
        index += 1;
    };
    try checker.finish();
    try expect(coverage.cube_cases == 35);
}

test "SoftBodySettings parity: Optimize sorts more than 32 parallel groups of equal size" {
    const allocator = std.testing.allocator;
    var checker: TranscriptChecker = .{ .name = "SoftBodySettings copies" };
    var coverage: Coverage = .{};
    var g: Gen = .{ .rng = .{ .state = 0x9a7c0b1e } };
    const buffer = try allocator.alloc(u8, transcript_capacity);
    defer allocator.free(buffer);
    const num_copies_cases = 8;
    var num_optimizes: u32 = 0;
    for (0..num_copies_cases) |index| {
        var c = try generateCopies(allocator, &g);
        defer c.deinit(allocator);
        for (c.ops.items) |op| num_optimizes += @intFromBool(op.kind == .optimize or op.kind == .optimize_no_results);
        try runCase(allocator, &checker, &coverage, &c, index, buffer);
    }
    try checker.finish();
    std.debug.print("SoftBodySettings copies coverage: {d} optimizes, {d} with more than 32 parallel groups and ties\n", .{ num_optimizes, coverage.group_size_ties_over_32 });
    try expect(coverage.group_size_ties_over_32 == num_optimizes);
}
