//! Parity tests for the collision filters, groups, contact listener types and active edges: ObjectLayerPairFilterMask
//! (sGetObjectLayer / sGetGroup / sGetMask / ShouldCollide), ObjectLayerPairFilterTable, BroadPhaseLayerInterfaceTable,
//! BroadPhaseLayerInterfaceMask + ObjectVsBroadPhaseLayerFilterMask, ObjectVsBroadPhaseLayerFilterTable (built from
//! the tables and from the masks), GroupFilterTable (IsCollisionEnabled of every pair) + CollisionGroup (CanCollide
//! and operator == of every pair of random groups with table filters and a user filter that is not symmetric, which
//! shows the order of the arguments), the binary state of CollisionGroup and GroupFilterTable (save,
//! copy, restore from intact, truncated and corrupted streams, RTTI hashes), ContactManifold::SwapShapes and the world
//! space contact points, the ContactSettings defaults / ValidateResult values and ActiveEdges::IsEdgeActive /
//! FixNormal. Zolt and the C++ Jolt library run on the same inputs and must produce identical bits (identical
//! decisions for every layer / group combination of each random configuration). C ABI wrappers:
//! ZoltParity/Physics/FiltersReference.cpp. See ZoltParity/parity.zig for how parity tests work.
//!
//! Configurations are random sequences of enable / disable / map / configure operations (including repeated and
//! self pairs), with empty, single and larger tables. The active edge inputs mix random vectors with coplanar and
//! opposite normals, normals at exactly cos(179 degrees) and at the threshold angle (ties of the strict comparisons),
//! edges along the cross product of the normals (convex / concave / zero), degenerate triangles and contact points at
//! vertices, on edges and around the barycentric epsilons. FixNormal also gets normals within a few ulps of its
//! cos(1 degree) threshold and contact points whose barycentric coordinate is exactly 1.0e-4 or 1 - 1.0e-4 (ties of
//! the strict comparisons, the generator checks that they are hit).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const ActiveEdges = zolt.ActiveEdges;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const BroadPhaseLayerInterfaceMask = zolt.BroadPhaseLayerInterfaceMask;
const BroadPhaseLayerInterfaceTable = zolt.BroadPhaseLayerInterfaceTable;
const CollisionGroup = zolt.CollisionGroup;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const Core = zolt.Core;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const ObjectLayerPairFilterMask = zolt.ObjectLayerPairFilterMask;
const ObjectLayerPairFilterTable = zolt.ObjectLayerPairFilterTable;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const ObjectVsBroadPhaseLayerFilterMask = zolt.ObjectVsBroadPhaseLayerFilterMask;
const ObjectVsBroadPhaseLayerFilterTable = zolt.ObjectVsBroadPhaseLayerFilterTable;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RVec3 = zolt.RVec3;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const ValidateResult = zolt.ValidateResult;
const Vec3 = zolt.Vec3;

/// The C++ reference functions, see FiltersReference.cpp
const jolt = struct {
    extern fn jolt_object_layer_mask_constants(out_num_bits: *u32, out_mask: *u32, out_object_layer_bits: *u32) void;
    extern fn jolt_object_layer_mask_get_object_layer(group: u32, mask: u32, use_default_mask: c_int, out_group: *u32, out_mask: *u32) u32;
    extern fn jolt_object_layer_pair_filter_mask(layers1: [*]const u32, layers2: [*]const u32, count: u32, out_groups: [*]u32, out_masks: [*]u32, out_should_collide: [*]u8) void;
    extern fn jolt_object_layer_pair_filter_table(num_layers: u32, ops: [*]const u32, num_ops: u32, out_should_collide: [*]u8) u32;
    extern fn jolt_broad_phase_layer_interface_table(num_object_layers: u32, num_broad_phase_layers: u32, ops: [*]const u32, num_ops: u32, out_broad_phase_layers: [*]u32) u32;
    extern fn jolt_broad_phase_layer_interface_mask(num_broad_phase_layers: u32, ops: [*]const u32, num_ops: u32, layers: [*]const u32, num_layers: u32, out_broad_phase_layers: [*]u32, out_should_collide: [*]u8, out_filter_should_collide: [*]u8) u32;
    extern fn jolt_object_vs_broad_phase_layer_filter_table(from_masks: c_int, num_object_layers: u32, num_broad_phase_layers: u32, pair_ops: [*]const u32, num_pair_ops: u32, broad_phase_ops: [*]const u32, num_broad_phase_ops: u32, out_should_collide: [*]u8) void;
    extern fn jolt_collision_groups(num_sub_groups_a: u32, ops_a: [*]const u32, num_ops_a: u32, num_sub_groups_b: u32, ops_b: [*]const u32, num_ops_b: u32, groups: [*]const u32, num_groups: u32, out_enabled_a: [*]u8, out_enabled_b: [*]u8, out_can_collide: [*]u8, out_equal: [*]u8) void;
    extern fn jolt_collision_group_save(group_id: u32, sub_group_id: u32, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_collision_group_restore(bytes: [*]const u8, size: u32, io_group_id: *u32, io_sub_group_id: *u32, out_eof: *c_int, out_failed: *c_int) void;
    extern fn jolt_group_filter_rtti_hashes(out_hashes: *[2]u32) void;
    extern fn jolt_group_filter_table_save(num_sub_groups: u32, ops: [*]const u32, num_ops: u32, copy: c_int, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_group_filter_restore(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_size: *u32) c_int;
    extern fn jolt_contact_manifold(in: *const CM, out_swapped: *CM, out_world1: [*]Real, out_world2: [*]Real) void;
    extern fn jolt_contact_settings_defaults(out_floats: *[10]f32, out_validate_results: *[4]c_int) void;
    extern fn jolt_is_edge_active(normal1: *const P, normal2: *const P, edge_direction: *const P, cos_threshold_angle: f32) c_int;
    extern fn jolt_fix_normal(v0: *const P, v1: *const P, v2: *const P, triangle_normal: *const P, active_edges: u32, point: *const P, normal: *const P, movement_direction: *const P, out_normal: *P) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// ContactManifold in the format of the C ABI, must match CM in FiltersReference.cpp
const CM = extern struct {
    base_offset: [3]Real,
    world_space_normal: P,
    penetration_depth: f32,
    sub_shape_id1: u32,
    sub_shape_id2: u32,
    count1: u32,
    points1: [64]P,
    count2: u32,
    points2: [64]P,

    fn toManifold(self: *const CM) ContactManifold {
        var m: ContactManifold = .{
            .base_offset = RVec3.init(self.base_offset[0], self.base_offset[1], self.base_offset[2]),
            .world_space_normal = vec3(self.world_space_normal),
            .penetration_depth = self.penetration_depth,
            .sub_shape_id1 = .{ .value = self.sub_shape_id1 },
            .sub_shape_id2 = .{ .value = self.sub_shape_id2 },
        };
        for (self.points1[0..self.count1]) |p| m.relative_contact_points_on1.append(vec3(p));
        for (self.points2[0..self.count2]) |p| m.relative_contact_points_on2.append(vec3(p));
        return m;
    }

    fn fromManifold(m: *const ContactManifold) CM {
        var c = std.mem.zeroes(CM);
        c.base_offset = .{ m.base_offset.getX(), m.base_offset.getY(), m.base_offset.getZ() };
        c.world_space_normal = arr3(m.world_space_normal);
        c.penetration_depth = m.penetration_depth;
        c.sub_shape_id1 = m.sub_shape_id1.getValue();
        c.sub_shape_id2 = m.sub_shape_id2.getValue();
        c.count1 = m.relative_contact_points_on1.len;
        for (m.relative_contact_points_on1.constSlice(), 0..) |p, i| c.points1[i] = arr3(p);
        c.count2 = m.relative_contact_points_on2.len;
        for (m.relative_contact_points_on2.constSlice(), 0..) |p, i| c.points2[i] = arr3(p);
        return c;
    }
};

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

/// Values that tend to expose differences, mixed into the random inputs (1e30 overflows when multiplied)
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-20, 1.0e-40, 100.0, -100.0, 1.0e30, -1.0e30 };

/// Input generator: xorshift32 with helpers for the edge cases
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    /// Random index in [0, n)
    fn index(self: *Gen, n: usize) usize {
        return self.next() % n;
    }

    /// Random u32 in [0, n)
    fn below(self: *Gen, n: u32) u32 {
        return self.next() % n;
    }

    /// True one in n times
    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) return special_values[self.index(special_values.len)];
        return self.rng.float(min, max);
    }

    /// Random float in [min, max) without special values
    fn plain(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.float(min, max), self.float(min, max), self.float(min, max));
    }

    fn plainVec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.plain(min, max), self.plain(min, max), self.plain(min, max));
    }

    /// Random unit vector
    fn unitVec(self: *Gen) Vec3 {
        while (true) {
            const v = self.plainVec(-1, 1);
            const len_sq = v.lengthSq();
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return v.normalized();
        }
    }

    /// Random bits: sparse (a few set bits), dense, all or none, limited to `mask`
    fn bits(self: *Gen, mask: u32) u32 {
        return mask & switch (self.next() % 6) {
            0 => 0,
            1 => 0xffffffff,
            2 => self.next(),
            else => @as(u32, 1) << @intCast(self.index(32)) | (if (self.oneIn(2)) @as(u32, 1) << @intCast(self.index(32)) else 0),
        };
    }

    /// Random object layer value (any bits)
    fn objectLayer(self: *Gen) ObjectLayer {
        return switch (self.next() % 4) {
            0 => @truncate(self.next()),
            1 => ObjectLayerPairFilterMask.getObjectLayer(self.bits(ObjectLayerPairFilterMask.mask), .{ .mask = self.bits(ObjectLayerPairFilterMask.mask) }),
            2 => @truncate(self.below(16)),
            else => std.math.maxInt(ObjectLayer) - @as(ObjectLayer, @truncate(self.below(4))),
        };
    }
};

// ---------------------------------------------------------------------------------------------------------------
// Object layer pair filters

test "ObjectLayerPairFilterMask parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x6d61736b } };
    var constants: Checker = .{ .name = "ObjectLayerPairFilterMask constants" };
    var get_object_layer: Checker = .{ .name = "ObjectLayerPairFilterMask.getObjectLayer" };
    var should_collide: Checker = .{ .name = "ObjectLayerPairFilterMask.shouldCollide / getGroup / getMask" };

    {
        var j: [3]u32 = undefined;
        jolt.jolt_object_layer_mask_constants(&j[0], &j[1], &j[2]);
        constants.check("constants", [3]u32{ ObjectLayerPairFilterMask.num_bits, ObjectLayerPairFilterMask.mask, @bitSizeOf(ObjectLayer) }, j);
    }

    const filter: ObjectLayerPairFilterMask = .{};
    const base: *const ObjectLayerPairFilter = &filter.base;

    const batch = 1000;
    for (0..iterations / batch) |_| {
        // sGetObjectLayer with and without the default mask
        for (0..batch) |_| {
            const group = gen.bits(ObjectLayerPairFilterMask.mask);
            const mask = gen.bits(ObjectLayerPairFilterMask.mask);
            const use_default = gen.oneIn(4);
            const Result = struct { layer: u32, group: u32, mask: u32 };
            var j: Result = undefined;
            j.layer = jolt.jolt_object_layer_mask_get_object_layer(group, mask, @intFromBool(use_default), &j.group, &j.mask);
            const layer = if (use_default) ObjectLayerPairFilterMask.getObjectLayer(group, .{}) else ObjectLayerPairFilterMask.getObjectLayer(group, .{ .mask = mask });
            const z: Result = .{ .layer = layer, .group = ObjectLayerPairFilterMask.getGroup(layer), .mask = ObjectLayerPairFilterMask.getMask(layer) };
            get_object_layer.check(.{ group, mask, use_default }, z, j);
        }

        // ShouldCollide (through the base class), sGetGroup and sGetMask of any layer
        var layers1: [batch]u32 = undefined;
        var layers2: [batch]u32 = undefined;
        for (&layers1, &layers2) |*l1, *l2| {
            l1.* = gen.objectLayer();
            l2.* = if (gen.oneIn(8)) l1.* else gen.objectLayer();
        }
        const Result = struct { groups: [batch]u32, masks: [batch]u32, should_collide: [batch]u8 };
        var j: Result = undefined;
        jolt.jolt_object_layer_pair_filter_mask(&layers1, &layers2, batch, &j.groups, &j.masks, &j.should_collide);
        var z: Result = undefined;
        for (layers1, layers2, 0..) |l1, l2, i| {
            z.groups[i] = ObjectLayerPairFilterMask.getGroup(@intCast(l1));
            z.masks[i] = ObjectLayerPairFilterMask.getMask(@intCast(l1));
            z.should_collide[i] = @intFromBool(base.shouldCollide(@intCast(l1), @intCast(l2)));
        }
        should_collide.check(.{ layers1[0], layers2[0] }, z, j);
    }

    try finishAll(&.{ &constants, &get_object_layer, &should_collide });
}

/// Maximum number of layers / operations of a random configuration
const max_layers = 100;
const max_ops = 3 * max_layers;

/// A random sequence of (layer1, layer2, enable) operations on n layers (pairs include a layer with itself)
fn layerPairOps(gen: *Gen, n: u32, ops: *[max_ops * 3]u32) u32 {
    if (n == 0) return 0;
    const num_ops: u32 = @intCast(gen.index(@min(3 * n, max_ops) + 1));
    for (0..num_ops) |i| {
        ops[3 * i] = gen.below(n);
        ops[3 * i + 1] = if (gen.oneIn(6)) ops[3 * i] else gen.below(n);
        ops[3 * i + 2] = @intFromBool(!gen.oneIn(4));
    }
    return num_ops;
}

fn applyLayerPairOps(table: *ObjectLayerPairFilterTable, ops: []const u32) void {
    var i: usize = 0;
    while (i < ops.len) : (i += 3) {
        if (ops[i + 2] != 0)
            table.enableCollision(@intCast(ops[i]), @intCast(ops[i + 1]))
        else
            table.disableCollision(@intCast(ops[i]), @intCast(ops[i + 1]));
    }
}

/// A random number of layers: often small (including 0 and 1), sometimes up to max_layers
fn numLayers(gen: *Gen) u32 {
    return switch (gen.next() % 8) {
        0 => gen.below(3),
        1 => gen.below(max_layers + 1),
        else => gen.below(40),
    };
}

test "ObjectLayerPairFilterTable parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x7461626c } };
    var checker: Checker = .{ .name = "ObjectLayerPairFilterTable" };

    for (0..2000) |_| {
        const n = numLayers(&gen);
        var ops: [max_ops * 3]u32 = undefined;
        const num_ops = layerPairOps(&gen, n, &ops);

        const Result = struct { num_layers: u32, should_collide: [max_layers * max_layers]u8 };
        var j: Result = .{ .num_layers = 0, .should_collide = @splat(0) };
        j.num_layers = jolt.jolt_object_layer_pair_filter_table(n, &ops, num_ops, &j.should_collide);

        var table = try ObjectLayerPairFilterTable.init(allocator, n);
        defer table.deinit();
        applyLayerPairOps(&table, ops[0 .. 3 * num_ops]);
        const base: *const ObjectLayerPairFilter = &table.base;
        var z: Result = .{ .num_layers = table.getNumObjectLayers(), .should_collide = @splat(0) };
        for (0..n) |a| for (0..n) |b| {
            z.should_collide[a * n + b] = @intFromBool(base.shouldCollide(@intCast(a), @intCast(b)));
        };
        checker.check(.{ n, num_ops }, z, j);
    }

    try finishAll(&.{&checker});
}

// ---------------------------------------------------------------------------------------------------------------
// Broadphase layer interfaces and object vs broadphase layer filters

const max_broad_phase_layers = 16;

/// A random sequence of (object layer, broadphase layer) mappings
fn broadPhaseMapOps(gen: *Gen, num_object_layers: u32, num_broad_phase_layers: u32, ops: *[max_ops * 2]u32) u32 {
    if (num_object_layers == 0 or num_broad_phase_layers == 0) return 0;
    const num_ops: u32 = @intCast(gen.index(@min(2 * num_object_layers, max_ops) + 1));
    for (0..num_ops) |i| {
        ops[2 * i] = gen.below(num_object_layers);
        ops[2 * i + 1] = gen.below(num_broad_phase_layers);
    }
    return num_ops;
}

fn applyBroadPhaseMapOps(table: *BroadPhaseLayerInterfaceTable, ops: []const u32) void {
    var i: usize = 0;
    while (i < ops.len) : (i += 2)
        table.mapObjectToBroadPhaseLayer(@intCast(ops[i]), .init(@intCast(ops[i + 1])));
}

/// A random sequence of (broadphase layer, groups to include, groups to exclude) ConfigureLayer operations
fn broadPhaseMaskOps(gen: *Gen, num_broad_phase_layers: u32, ops: *[max_ops * 3]u32) u32 {
    const num_ops: u32 = @intCast(gen.index(2 * num_broad_phase_layers + 1));
    for (0..num_ops) |i| {
        ops[3 * i] = gen.below(num_broad_phase_layers);
        ops[3 * i + 1] = gen.bits(if (gen.oneIn(8)) 0xffffffff else ObjectLayerPairFilterMask.mask);
        ops[3 * i + 2] = if (gen.oneIn(2)) 0 else gen.bits(if (gen.oneIn(8)) 0xffffffff else ObjectLayerPairFilterMask.mask);
    }
    return num_ops;
}

fn applyBroadPhaseMaskOps(mask: *BroadPhaseLayerInterfaceMask, ops: []const u32) void {
    var i: usize = 0;
    while (i < ops.len) : (i += 3)
        mask.configureLayer(.init(@intCast(ops[i])), ops[i + 1], ops[i + 2]);
}

test "BroadPhaseLayerInterfaceTable parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x62706c74 } };
    var checker: Checker = .{ .name = "BroadPhaseLayerInterfaceTable" };

    for (0..2000) |_| {
        const n = numLayers(&gen);
        const m = gen.below(max_broad_phase_layers + 1);
        var ops: [max_ops * 2]u32 = undefined;
        const num_ops = broadPhaseMapOps(&gen, n, m, &ops);

        const Result = struct { num_broad_phase_layers: u32, layers: [max_layers]u32 };
        var j: Result = .{ .num_broad_phase_layers = 0, .layers = @splat(0) };
        j.num_broad_phase_layers = jolt.jolt_broad_phase_layer_interface_table(n, m, &ops, num_ops, &j.layers);

        var table = try BroadPhaseLayerInterfaceTable.init(allocator, n, m);
        defer table.deinit();
        applyBroadPhaseMapOps(&table, ops[0 .. 2 * num_ops]);
        const base: *const BroadPhaseLayerInterface = &table.base;
        var z: Result = .{ .num_broad_phase_layers = base.getNumBroadPhaseLayers(), .layers = @splat(0) };
        for (0..n) |o| z.layers[o] = base.getBroadPhaseLayer(@intCast(o)).getValue();
        checker.check(.{ n, m, num_ops }, z, j);
    }

    try finishAll(&.{&checker});
}

test "BroadPhaseLayerInterfaceMask / ObjectVsBroadPhaseLayerFilterMask parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x62706d6b } };
    var checker: Checker = .{ .name = "BroadPhaseLayerInterfaceMask / ObjectVsBroadPhaseLayerFilterMask" };

    const num_layers = 64;
    for (0..5000) |_| {
        const m = 1 + gen.below(max_broad_phase_layers);
        var ops: [max_ops * 3]u32 = undefined;
        const num_ops = broadPhaseMaskOps(&gen, m, &ops);
        var layers: [num_layers]u32 = undefined;
        for (&layers) |*l| l.* = gen.objectLayer();

        const Result = struct {
            num_broad_phase_layers: u32,
            layers: [num_layers]u32,
            should_collide: [num_layers * max_broad_phase_layers]u8,
            filter_should_collide: [num_layers * max_broad_phase_layers]u8,
        };
        var j: Result = .{ .num_broad_phase_layers = 0, .layers = @splat(0), .should_collide = @splat(0), .filter_should_collide = @splat(0) };
        j.num_broad_phase_layers = jolt.jolt_broad_phase_layer_interface_mask(m, &ops, num_ops, &layers, num_layers, &j.layers, &j.should_collide, &j.filter_should_collide);

        var mask = try BroadPhaseLayerInterfaceMask.init(allocator, m);
        defer mask.deinit();
        applyBroadPhaseMaskOps(&mask, ops[0 .. 3 * num_ops]);
        const filter = ObjectVsBroadPhaseLayerFilterMask.init(&mask);
        const base: *const BroadPhaseLayerInterface = &mask.base;
        const filter_base: *const ObjectVsBroadPhaseLayerFilter = &filter.base;
        var z: Result = .{ .num_broad_phase_layers = base.getNumBroadPhaseLayers(), .layers = @splat(0), .should_collide = @splat(0), .filter_should_collide = @splat(0) };
        for (layers, 0..) |l, i| {
            const layer: ObjectLayer = @intCast(l);
            z.layers[i] = base.getBroadPhaseLayer(layer).getValue();
            for (0..m) |b| {
                const bp_layer = BroadPhaseLayer.init(@intCast(b));
                z.should_collide[i * m + b] = @intFromBool(mask.shouldCollide(layer, bp_layer));
                z.filter_should_collide[i * m + b] = @intFromBool(filter_base.shouldCollide(layer, bp_layer));
            }
        }
        checker.check(.{ m, num_ops, ops[0..@min(3 * num_ops, 12)] }, z, j);
    }

    try finishAll(&.{&checker});
}

test "ObjectVsBroadPhaseLayerFilterTable parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x6f766274 } };
    var from_tables: Checker = .{ .name = "ObjectVsBroadPhaseLayerFilterTable (from tables)" };
    var from_masks: Checker = .{ .name = "ObjectVsBroadPhaseLayerFilterTable (from masks)" };

    // Built from a BroadPhaseLayerInterfaceTable and an ObjectLayerPairFilterTable
    for (0..2000) |_| {
        const n = numLayers(&gen);
        const m = 1 + gen.below(max_broad_phase_layers);
        var pair_ops: [max_ops * 3]u32 = undefined;
        const num_pair_ops = layerPairOps(&gen, n, &pair_ops);
        var map_ops: [max_ops * 2]u32 = undefined;
        const num_map_ops = broadPhaseMapOps(&gen, n, m, &map_ops);

        var j: [max_layers * max_broad_phase_layers]u8 = @splat(0);
        jolt.jolt_object_vs_broad_phase_layer_filter_table(0, n, m, &pair_ops, num_pair_ops, &map_ops, num_map_ops, &j);

        var bp_interface = try BroadPhaseLayerInterfaceTable.init(allocator, n, m);
        defer bp_interface.deinit();
        applyBroadPhaseMapOps(&bp_interface, map_ops[0 .. 2 * num_map_ops]);
        var pair_filter = try ObjectLayerPairFilterTable.init(allocator, n);
        defer pair_filter.deinit();
        applyLayerPairOps(&pair_filter, pair_ops[0 .. 3 * num_pair_ops]);
        var filter = try ObjectVsBroadPhaseLayerFilterTable.init(allocator, &bp_interface.base, m, &pair_filter.base, n);
        defer filter.deinit();
        const base: *const ObjectVsBroadPhaseLayerFilter = &filter.base;
        var z: [max_layers * max_broad_phase_layers]u8 = @splat(0);
        for (0..n) |o| for (0..m) |b| {
            z[o * m + b] = @intFromBool(base.shouldCollide(@intCast(o), .init(@intCast(b))));
        };
        from_tables.check(.{ n, m, num_pair_ops, num_map_ops }, z, j);
    }

    // Built from a BroadPhaseLayerInterfaceMask and an ObjectLayerPairFilterMask (the object layers 0 .. n - 1: only
    // layers above the group bits have mask bits, 255 with 16 bit layers)
    const max_mask_layers = 1300;
    for (0..40) |_| {
        const n = if (gen.oneIn(4)) gen.below(max_mask_layers + 1) else max_mask_layers - gen.below(400);
        const m = 1 + gen.below(max_broad_phase_layers);
        var bp_ops: [max_ops * 3]u32 = undefined;
        const num_bp_ops = broadPhaseMaskOps(&gen, m, &bp_ops);

        var j: [max_mask_layers * max_broad_phase_layers]u8 = @splat(0);
        jolt.jolt_object_vs_broad_phase_layer_filter_table(1, n, m, &bp_ops, 0, &bp_ops, num_bp_ops, &j);

        var bp_interface = try BroadPhaseLayerInterfaceMask.init(allocator, m);
        defer bp_interface.deinit();
        applyBroadPhaseMaskOps(&bp_interface, bp_ops[0 .. 3 * num_bp_ops]);
        const pair_filter: ObjectLayerPairFilterMask = .{};
        var filter = try ObjectVsBroadPhaseLayerFilterTable.init(allocator, &bp_interface.base, m, &pair_filter.base, n);
        defer filter.deinit();
        const base: *const ObjectVsBroadPhaseLayerFilter = &filter.base;
        var z: [max_mask_layers * max_broad_phase_layers]u8 = @splat(0);
        for (0..n) |o| for (0..m) |b| {
            z[o * m + b] = @intFromBool(base.shouldCollide(@intCast(o), .init(@intCast(b))));
        };
        from_masks.check(.{ n, m, num_bp_ops }, z, j);
    }

    try finishAll(&.{ &from_tables, &from_masks });
}

// ---------------------------------------------------------------------------------------------------------------
// Group filters and collision groups

const max_sub_groups = 48;
const max_groups = 24;

/// A random sequence of (sub group 1, sub group 2, enable) operations on n sub groups (never a sub group with itself)
fn subGroupOps(gen: *Gen, n: u32, ops: *[max_ops * 3]u32) u32 {
    if (n < 2) return 0;
    const num_ops: u32 = @intCast(gen.index(@min(2 * n, max_ops) + 1));
    for (0..num_ops) |i| {
        const a = gen.below(n);
        var b = gen.below(n - 1);
        if (b >= a) b += 1;
        ops[3 * i] = a;
        ops[3 * i + 1] = b;
        ops[3 * i + 2] = @intFromBool(gen.oneIn(3));
    }
    return num_ops;
}

fn applySubGroupOps(table: *GroupFilterTable, ops: []const u32) void {
    var i: usize = 0;
    while (i < ops.len) : (i += 3) {
        if (ops[i + 2] != 0)
            table.enableCollision(ops[i], ops[i + 1])
        else
            table.disableCollision(ops[i], ops[i + 1]);
    }
}

fn numSubGroups(gen: *Gen) u32 {
    return switch (gen.next() % 6) {
        0 => gen.below(3),
        1 => gen.below(max_sub_groups + 1),
        else => gen.below(12),
    };
}

/// Creates a group filter table with n sub groups and applies the operations
fn createGroupFilterTable(n: u32, ops: []const u32) !*GroupFilterTable {
    const table = try GroupFilterTable.create(std.testing.allocator, .{ .num_sub_groups = n });
    applySubGroupOps(table, ops);
    return table;
}

/// A user group filter that is not symmetric (canCollide(g1, g2) != canCollide(g2, g1)), so CollisionGroup.canCollide
/// must pass the groups in the right order. Must match OrderedGroupFilter in FiltersReference.cpp.
const OrderedGroupFilter = struct {
    pub const overrides = .{.canCollide};
    pub const rtti_name = "OrderedGroupFilter";

    base: GroupFilter,

    pub fn canCollide(self: *const OrderedGroupFilter, group1: *const CollisionGroup, group2: *const CollisionGroup) bool {
        _ = self;
        return group1.getSubGroupID() < group2.getSubGroupID();
    }
};

test "GroupFilterTable / CollisionGroup parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x67727073 } };
    var enabled: Checker = .{ .name = "GroupFilterTable.isCollisionEnabled" };
    var can_collide: Checker = .{ .name = "CollisionGroup.canCollide" };
    var equal: Checker = .{ .name = "CollisionGroup.eql" };

    for (0..5000) |_| {
        const na = numSubGroups(&gen);
        const nb = numSubGroups(&gen);
        var ops_a: [max_ops * 3]u32 = undefined;
        const num_ops_a = subGroupOps(&gen, na, &ops_a);
        var ops_b: [max_ops * 3]u32 = undefined;
        const num_ops_b = subGroupOps(&gen, nb, &ops_b);

        // Random groups: a filter (none, A, B or the ordered user filter), a group ID (few values to get equal ones, the
        // invalid group) and a sub group ID that is valid for the filter (a group with a table filter that has less
        // than 2 sub groups only uses sub group 0, the table is then never accessed, like Jolt requires)
        const num_groups = 1 + gen.below(max_groups);
        var groups: [max_groups * 3]u32 = undefined;
        for (0..num_groups) |i| {
            const filter = gen.below(4);
            groups[3 * i] = filter;
            groups[3 * i + 1] = switch (gen.next() % 6) {
                0 => CollisionGroup.invalid_group,
                1 => gen.next(),
                else => gen.below(3),
            };
            const n = switch (filter) {
                1 => na,
                2 => nb,
                else => 0,
            };
            groups[3 * i + 2] = if (filter == 0 or filter == 3) switch (gen.next() % 4) {
                0 => CollisionGroup.invalid_sub_group,
                1 => gen.next(),
                else => gen.below(4),
            } else if (n < 2) 0 else gen.below(n);
        }

        const Enabled = struct { a: [max_sub_groups * max_sub_groups]u8, b: [max_sub_groups * max_sub_groups]u8 };
        const Pairs = [max_groups * max_groups]u8;
        var j_enabled: Enabled = .{ .a = @splat(0), .b = @splat(0) };
        var j_can_collide: Pairs = @splat(0);
        var j_equal: Pairs = @splat(0);
        jolt.jolt_collision_groups(na, &ops_a, num_ops_a, nb, &ops_b, num_ops_b, &groups, num_groups, &j_enabled.a, &j_enabled.b, &j_can_collide, &j_equal);

        const a = try createGroupFilterTable(na, ops_a[0 .. 3 * num_ops_a]);
        var a_ref = Ref(GroupFilter).init(a.asGroupFilterMut());
        defer a_ref.deinit();
        const b = try createGroupFilterTable(nb, ops_b[0 .. 3 * num_ops_b]);
        var b_ref = Ref(GroupFilter).init(b.asGroupFilterMut());
        defer b_ref.deinit();

        var z_enabled: Enabled = .{ .a = @splat(0), .b = @splat(0) };
        for (0..na) |x| for (0..na) |y| {
            z_enabled.a[x * na + y] = @intFromBool(x != y and a.isCollisionEnabled(@intCast(x), @intCast(y)));
        };
        for (0..nb) |x| for (0..nb) |y| {
            z_enabled.b[x * nb + y] = @intFromBool(x != y and b.isCollisionEnabled(@intCast(x), @intCast(y)));
        };
        enabled.check(.{ na, nb, num_ops_a, num_ops_b }, z_enabled, j_enabled);

        var ordered: OrderedGroupFilter = .{ .base = .init(OrderedGroupFilter, std.testing.allocator) };
        ordered.base.setEmbedded();
        defer ordered.base.deinit();

        var collision_groups: [max_groups]CollisionGroup = undefined;
        for (0..num_groups) |i| {
            const filter: ?*const GroupFilter = switch (groups[3 * i]) {
                1 => a.asGroupFilter(),
                2 => b.asGroupFilter(),
                3 => &ordered.base,
                else => null,
            };
            collision_groups[i] = .init(filter, groups[3 * i + 1], groups[3 * i + 2]);
        }
        defer for (collision_groups[0..num_groups]) |*g| g.deinit();
        var z_can_collide: Pairs = @splat(0);
        var z_equal: Pairs = @splat(0);
        for (0..num_groups) |x| for (0..num_groups) |y| {
            z_can_collide[x * num_groups + y] = @intFromBool(collision_groups[x].canCollide(&collision_groups[y]));
            z_equal[x * num_groups + y] = @intFromBool(collision_groups[x].eql(&collision_groups[y]));
        };
        can_collide.check(.{ na, nb, num_groups, groups[0..@min(3 * num_groups, 12)] }, z_can_collide, j_can_collide);
        equal.check(.{ na, nb, num_groups }, z_equal, j_equal);
    }

    try finishAll(&.{ &enabled, &can_collide, &equal });
}

/// Bytes written by a save function, zero padded
const Bytes = struct {
    size: u32,
    bytes: [512]u8,

    const empty: Bytes = .{ .size = 0, .bytes = @splat(0) };

    fn fromSlice(slice: []const u8) Bytes {
        var b: Bytes = .empty;
        b.size = @intCast(slice.len);
        @memcpy(b.bytes[0..slice.len], slice);
        return b;
    }
};

/// Saves a group filter with Zolt
fn zoltSaveGroupFilter(filter: *const GroupFilter) Bytes {
    var b: Bytes = .empty;
    var writer: std.Io.Writer = .fixed(&b.bytes);
    var out = StreamOutWrapper.init(&writer);
    filter.saveBinaryState(out.streamOut());
    std.debug.assert(!out.isFailed());
    b.size = @intCast(writer.end);
    return b;
}

/// Result of GroupFilter::sRestoreFromBinaryState: valid + the binary state of the restored filter, or the error text
const GroupFilterRestore = struct {
    valid: c_int,
    bytes: Bytes,
};

fn zoltRestoreGroupFilter(bytes: []const u8) !GroupFilterRestore {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try GroupFilter.restoreFromBinaryState(std.testing.allocator, in.streamIn());
    defer result.deinit();
    if (result.isValid())
        return .{ .valid = 1, .bytes = zoltSaveGroupFilter(result.getPtr().?) };
    return .{ .valid = 0, .bytes = .fromSlice(result.getError()) };
}

fn joltRestoreGroupFilter(bytes: []const u8) GroupFilterRestore {
    var r: GroupFilterRestore = .{ .valid = 0, .bytes = .empty };
    r.valid = jolt.jolt_group_filter_restore(bytes.ptr, @intCast(bytes.len), &r.bytes.bytes, r.bytes.bytes.len, &r.bytes.size);
    return r;
}

test "CollisionGroup / GroupFilterTable binary state parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x62696e73 } };
    var group_save: Checker = .{ .name = "CollisionGroup.saveBinaryState" };
    var group_restore: Checker = .{ .name = "CollisionGroup.restoreBinaryState" };
    var rtti: Checker = .{ .name = "GroupFilter RTTI hashes" };
    var table_save: Checker = .{ .name = "GroupFilterTable.saveBinaryState" };
    var table_restore: Checker = .{ .name = "GroupFilter.restoreFromBinaryState" };

    // CollisionGroup: save, then restore from intact, truncated or random bytes into a group with other values
    for (0..iterations / 10) |_| {
        const group_id = if (gen.oneIn(4)) CollisionGroup.invalid_group else gen.next();
        const sub_group_id = if (gen.oneIn(4)) CollisionGroup.invalid_sub_group else gen.next();
        var j_bytes: [16]u8 = @splat(0);
        const j_size = jolt.jolt_collision_group_save(group_id, sub_group_id, &j_bytes, j_bytes.len);
        const group = CollisionGroup.init(null, group_id, sub_group_id);
        var z_bytes: [16]u8 = @splat(0);
        var writer: std.Io.Writer = .fixed(&z_bytes);
        var out = StreamOutWrapper.init(&writer);
        group.saveBinaryState(out.streamOut());
        const Saved = struct { size: u32, bytes: [16]u8 };
        group_save.check(.{ group_id, sub_group_id }, Saved{ .size = @intCast(writer.end), .bytes = z_bytes }, Saved{ .size = j_size, .bytes = j_bytes });

        var bytes = j_bytes;
        var size = j_size;
        switch (gen.next() % 3) {
            0 => {},
            1 => size = gen.below(j_size + 1),
            else => {
                for (&bytes) |*b| b.* = @truncate(gen.next());
                size = gen.below(bytes.len + 1);
            },
        }
        const Restored = struct { group_id: u32, sub_group_id: u32, eof: c_int, failed: c_int };
        const initial_group_id = gen.next();
        const initial_sub_group_id = gen.next();
        var j: Restored = .{ .group_id = initial_group_id, .sub_group_id = initial_sub_group_id, .eof = 0, .failed = 0 };
        jolt.jolt_collision_group_restore(&bytes, size, &j.group_id, &j.sub_group_id, &j.eof, &j.failed);
        var restored = CollisionGroup.init(null, initial_group_id, initial_sub_group_id);
        var reader: std.Io.Reader = .fixed(bytes[0..size]);
        var in = StreamInWrapper.init(&reader);
        restored.restoreBinaryState(in.streamIn());
        const z: Restored = .{ .group_id = restored.getGroupID(), .sub_group_id = restored.getSubGroupID(), .eof = @intFromBool(in.isEOF()), .failed = @intFromBool(in.isFailed()) };
        group_restore.check(.{ size, bytes }, z, j);
    }

    // RTTI hashes
    {
        var j: [2]u32 = undefined;
        jolt.jolt_group_filter_rtti_hashes(&j);
        rtti.check("rtti", [2]u32{ GroupFilter.rttiHash("GroupFilter"), GroupFilter.rttiHash(GroupFilterTable.rtti_name) }, j);
    }

    // GroupFilterTable: save (the table or a copy of it), then restore from intact, truncated or corrupted bytes
    for (0..5000) |_| {
        const n = numSubGroups(&gen);
        var ops: [max_ops * 3]u32 = undefined;
        const num_ops = subGroupOps(&gen, n, &ops);
        const copy = gen.oneIn(3);

        var j: Bytes = .empty;
        j.size = jolt.jolt_group_filter_table_save(n, &ops, num_ops, @intFromBool(copy), &j.bytes, j.bytes.len);

        const table = try createGroupFilterTable(n, ops[0 .. 3 * num_ops]);
        var table_ref = Ref(GroupFilter).init(table.asGroupFilterMut());
        defer table_ref.deinit();
        const z = if (copy) blk: {
            var copied = try table.clone(allocator);
            copied.base.setEmbedded();
            defer copied.deinit();
            break :blk zoltSaveGroupFilter(copied.asGroupFilter());
        } else zoltSaveGroupFilter(table.asGroupFilter());
        table_save.check(.{ n, num_ops, copy }, z, j);

        var bytes = j.bytes;
        var size = j.size;
        switch (gen.next() % 6) {
            0, 1 => {},
            // Truncated
            2 => size = gen.below(j.size + 1),
            // Unknown type hash
            3 => std.mem.writeInt(u32, bytes[0..4], gen.next(), .little),
            // Another number of sub groups (not validated)
            4 => std.mem.writeInt(u32, bytes[4..8], gen.next(), .little),
            // Another table length: shorter (the rest of the stream is ignored) or longer (EOF)
            else => std.mem.writeInt(u32, bytes[8..12], gen.below(j.size), .little),
        }
        table_restore.check(.{ n, num_ops, size, bytes[0..@min(size, 16)] }, try zoltRestoreGroupFilter(bytes[0..size]), joltRestoreGroupFilter(bytes[0..size]));
    }

    try finishAll(&.{ &group_save, &group_restore, &rtti, &table_save, &table_restore });
}

// ---------------------------------------------------------------------------------------------------------------
// Contact listener types

test "ContactManifold parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x6d616e69 } };
    var swap: Checker = .{ .name = "ContactManifold.swapShapes" };
    var world: Checker = .{ .name = "ContactManifold.getWorldSpaceContactPointOn1 / On2" };
    var defaults: Checker = .{ .name = "ContactSettings defaults / ValidateResult values" };

    {
        var j_floats: [10]f32 = undefined;
        var j_values: [4]c_int = undefined;
        jolt.jolt_contact_settings_defaults(&j_floats, &j_values);
        const settings: ContactSettings = .{};
        const z_floats = [4]f32{ settings.inv_mass_scale1, settings.inv_inertia_scale1, settings.inv_mass_scale2, settings.inv_inertia_scale2 } ++ arr3(settings.relative_linear_surface_velocity) ++ arr3(settings.relative_angular_surface_velocity);
        const z_values = [4]c_int{
            @intFromEnum(ValidateResult.accept_all_contacts_for_this_body_pair),
            @intFromEnum(ValidateResult.accept_contact),
            @intFromEnum(ValidateResult.reject_contact),
            @intFromEnum(ValidateResult.reject_all_contacts_for_this_body_pair),
        };
        const Defaults = struct { floats: [10]f32, values: [4]c_int };
        defaults.check("defaults", Defaults{ .floats = z_floats, .values = z_values }, Defaults{ .floats = j_floats, .values = j_values });
    }

    const Points = [64][3]Real;
    for (0..iterations / 10) |_| {
        // A random manifold: the base offset can be far away (large values, which matter in double precision)
        var in = std.mem.zeroes(CM);
        const scale: f32 = if (gen.oneIn(2)) 1.0e5 else 10.0;
        for (&in.base_offset) |*c| c.* = @as(Real, gen.float(-scale, scale)) * (if (gen.oneIn(4)) @as(Real, 1.0e3) else 1.0) + @as(Real, gen.plain(-1, 1));
        in.world_space_normal = arr3(if (gen.oneIn(2)) gen.unitVec() else gen.vec(-2, 2));
        in.penetration_depth = gen.float(-1, 1);
        in.sub_shape_id1 = if (gen.oneIn(3)) SubShapeID.empty_value else gen.next();
        in.sub_shape_id2 = if (gen.oneIn(3)) SubShapeID.empty_value else gen.next();
        in.count1 = gen.below(65);
        in.count2 = if (gen.oneIn(2)) in.count1 else gen.below(65);
        for (in.points1[0..in.count1]) |*p| p.* = arr3(gen.vec(-5, 5));
        for (in.points2[0..in.count2], 0..) |*p, i| p.* = if (i < in.count1 and gen.oneIn(2)) in.points1[i] else arr3(gen.vec(-5, 5));

        var j_swapped: CM = undefined;
        var j_world1: Points = std.mem.zeroes(Points);
        var j_world2: Points = std.mem.zeroes(Points);
        jolt.jolt_contact_manifold(&in, &j_swapped, @ptrCast(&j_world1), @ptrCast(&j_world2));

        const manifold = in.toManifold();
        const z_swapped = CM.fromManifold(&manifold.swapShapes());
        swap.check(.{ in.world_space_normal, in.count1, in.count2 }, z_swapped, j_swapped);

        var z_world1: Points = std.mem.zeroes(Points);
        var z_world2: Points = std.mem.zeroes(Points);
        for (0..in.count1) |i| {
            const p = manifold.getWorldSpaceContactPointOn1(@intCast(i));
            z_world1[i] = .{ p.getX(), p.getY(), p.getZ() };
        }
        for (0..in.count2) |i| {
            const p = manifold.getWorldSpaceContactPointOn2(@intCast(i));
            z_world2[i] = .{ p.getX(), p.getY(), p.getZ() };
        }
        const World = struct { on1: Points, on2: Points };
        world.check(.{ in.base_offset, in.count1, in.count2 }, World{ .on1 = z_world1, .on2 = z_world2 }, World{ .on1 = j_world1, .on2 = j_world2 });
    }

    try finishAll(&.{ &defaults, &swap, &world });
}

// ---------------------------------------------------------------------------------------------------------------
// Active edges

/// Rotates v around the unit axis by angle (radians) with Zolt's quaternion
fn rotate(v: Vec3, axis: Vec3, angle: f32) Vec3 {
    return zolt.Quat.rotation(axis, angle).mulVec3(v);
}

/// Runs FixNormal with Zolt and Jolt on the same inputs and compares the results
fn checkFixNormal(checker: *Checker, v0: Vec3, v1: Vec3, v2: Vec3, triangle_normal: Vec3, active_edges: u8, point: Vec3, normal: Vec3, movement: Vec3) void {
    const pv0 = arr3(v0);
    const pv1 = arr3(v1);
    const pv2 = arr3(v2);
    const ptn = arr3(triangle_normal);
    const pp = arr3(point);
    const pn = arr3(normal);
    const pm = arr3(movement);
    var j: P = undefined;
    jolt.jolt_fix_normal(&pv0, &pv1, &pv2, &ptn, active_edges, &pp, &pn, &pm, &j);
    const z = arr3(ActiveEdges.fixNormal(v0, v1, v2, triangle_normal, active_edges, point, normal, movement));
    checker.check(.{ pv0, pv1, pv2, ptn, active_edges, pp, pn, pm }, z, j);
}

/// Places the local coordinates `c` (first leg, height, second leg) in 3D: coordinate i goes to axis `axes[i]`,
/// multiplied by `signs[i]` (+1 or -1, exact)
fn embed(c: [3]f32, axes: [3]usize, signs: [3]f32) Vec3 {
    var out: [3]f32 = undefined;
    for (0..3) |i| out[axes[i]] = signs[i] * c[i];
    return Vec3.init(out[0], out[1], out[2]);
}

test "ActiveEdges parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x65646765 } };
    var is_edge_active: Checker = .{ .name = "ActiveEdges.isEdgeActive" };
    var fix_normal: Checker = .{ .name = "ActiveEdges.fixNormal" };
    var fix_normal_threshold: Checker = .{ .name = "ActiveEdges.fixNormal (normals at the cos(1 degree) threshold)" };
    var fix_normal_bary: Checker = .{ .name = "ActiveEdges.fixNormal (barycentric coordinate at the epsilons)" };

    const cos_179: f32 = -0.999848;
    const one_degree: f32 = 0.017453292;
    const threshold_angle: f32 = 0.017435349; // acos(0.999848), the cos(1 degree) threshold of FixNormal
    const epsilon: f32 = 1.0e-4;
    const one_minus_epsilon: f32 = 1.0 - epsilon;

    // Number of generated inputs that hit what they aim at (computed with Zolt in the generator)
    var threshold_ties: usize = 0; // dot == 0.999848 * normal_length * triangle_normal_length
    var threshold_regrouped: usize = 0; // dot > 0.999848 * normal_length * triangle_normal_length != dot > 0.999848 * (normal_length * triangle_normal_length)
    var bary_ties: usize = 0; // The barycentric coordinate is exactly epsilon / one_minus_epsilon

    for (0..iterations) |_| {
        // IsEdgeActive
        {
            var normal1 = if (gen.oneIn(8)) gen.vec(-2, 2) else gen.unitVec();
            const axis = gen.unitVec();
            const kind = gen.next() % 9;
            const normal2 = switch (kind) {
                // Coplanar
                0 => normal1,
                // Opposite
                1 => normal1.negate(),
                // Close to opposite (around 179 degrees)
                2 => rotate(normal1.negate(), axis, gen.plain(-2.0 * one_degree, 2.0 * one_degree)),
                // Exactly at cos(179 degrees) and the floats next to it (normal1 becomes the X axis below, so the dot
                // product is the X component)
                3 => Vec3.init(@bitCast(@as(u32, @bitCast(cos_179)) +% @as(u32, @intCast(gen.index(3))) -% 1), gen.float(-1, 1), gen.float(-1, 1)),
                // Rotated around an axis by a random angle
                4, 5 => rotate(normal1, axis, gen.plain(-3.2, 3.2)),
                else => if (gen.oneIn(4)) gen.vec(-2, 2) else gen.unitVec(),
            };
            if (kind == 3) normal1 = Vec3.axisX();
            const edge = switch (gen.next() % 6) {
                // Convex / concave edges (the sign of the triple product), zero when the normals are parallel
                0 => normal1.cross(normal2),
                1 => normal1.cross(normal2).negate(),
                2 => Vec3.zero(),
                3 => axis,
                else => gen.vec(-1, 1),
            };
            const cos_angle = normal1.dot(normal2);
            const threshold: f32 = switch (gen.next() % 6) {
                // Exactly the cosine (tie of the strict comparison) or the floats next to it
                0 => cos_angle,
                1 => @bitCast(@as(u32, @bitCast(cos_angle)) +% 1),
                2 => @bitCast(@as(u32, @bitCast(cos_angle)) -% 1),
                3 => ([_]f32{ 1.0, -1.0, 0.0, -0.0, 0.9961947, 0.5, cos_179 })[gen.index(7)],
                else => gen.plain(-1, 1),
            };
            const p1 = arr3(normal1);
            const p2 = arr3(normal2);
            const pe = arr3(edge);
            const j = jolt.jolt_is_edge_active(&p1, &p2, &pe, threshold);
            const z: c_int = @intFromBool(ActiveEdges.isEdgeActive(normal1, normal2, edge, threshold));
            is_edge_active.check(.{ p1, p2, pe, threshold }, z, j);
        }

        // FixNormal
        {
            var v0 = gen.vec(-10, 10);
            var v1 = gen.vec(-10, 10);
            var v2 = gen.vec(-10, 10);
            switch (gen.next() % 10) {
                // Degenerate: collinear or coincident vertices
                0 => v2 = v0.add(v1.sub(v0).mulScalar(gen.float(-2, 2))),
                1 => v1 = v0,
                else => {},
            }
            const cross = v1.sub(v0).cross(v2.sub(v0));
            const triangle_normal = switch (gen.next() % 4) {
                0 => cross,
                1 => cross.normalizedOr(Vec3.axisY()),
                2 => cross.mulScalar(gen.float(-3, 3)),
                else => gen.vec(-1, 1),
            };

            // The contact point: a vertex, on an edge, around the barycentric epsilons or anywhere
            const eps_values = [_]f32{ 0.0, 1.0, 1.0e-4, 1.0 - 1.0e-4, 0.5 };
            var u: f32 = undefined;
            var v: f32 = undefined;
            switch (gen.next() % 4) {
                0 => {
                    u = eps_values[gen.index(eps_values.len)];
                    v = 1.0 - u;
                },
                1 => {
                    u = eps_values[gen.index(eps_values.len)] + gen.plain(-2.0e-4, 2.0e-4);
                    v = gen.plain(0, 1 - u);
                },
                2 => {
                    u = gen.plain(0, 1);
                    v = gen.plain(0, 1 - u);
                },
                else => {
                    u = gen.float(-1, 2);
                    v = gen.float(-1, 2);
                },
            }
            const w = 1.0 - u - v;
            const weights = [3]f32{ u, v, w };
            const perm = gen.index(3);
            const point = if (gen.oneIn(10)) gen.vec(-10, 10) else v0.mulScalar(weights[perm]).add(v1.mulScalar(weights[(perm + 1) % 3])).add(v2.mulScalar(weights[(perm + 2) % 3]));

            const normal = switch (gen.next() % 6) {
                // The triangle normal (scaled) or close to it (around 1 degree)
                0 => triangle_normal.mulScalar(gen.float(0.1, 3)),
                1 => rotate(triangle_normal, gen.unitVec(), gen.plain(0.5 * one_degree, 1.5 * one_degree)),
                // Opposite
                2 => triangle_normal.negate(),
                else => gen.vec(-2, 2),
            };
            const movement = switch (gen.next() % 5) {
                0, 1 => Vec3.zero(),
                2 => normal.mulScalar(gen.float(-2, 2)),
                3 => triangle_normal.mulScalar(gen.float(-2, 2)),
                else => gen.vec(-2, 2),
            };
            var active_edges: u8 = @intCast(gen.index(7)); // 0b111 is not allowed (assert)
            if (gen.oneIn(10)) {
                active_edges = @truncate(gen.next());
                if (active_edges == 0b111) active_edges = 0b1000;
            }

            checkFixNormal(&fix_normal, v0, v1, v2, triangle_normal, active_edges, point, normal, movement);
        }

        // FixNormal at the cos(1 degree) threshold: the angle between the normals is acos(0.999848) give or take a few
        // ulps, so the dot product lands on 0.999848 * normal_length * triangle_normal_length (a tie of the strict
        // comparison), next to it, or between it and 0.999848 * (normal_length * triangle_normal_length) (which a
        // regrouped product would use). The contact point is inside the triangle and there is no movement, so a normal
        // that is not parallel gives the triangle normal.
        {
            const triangle_normal = gen.unitVec().mulScalar(if (gen.oneIn(4)) 1.0 else gen.plain(0.01, 10));
            var axis = triangle_normal.cross(gen.unitVec());
            while (axis.lengthSq() < 1.0e-2 * triangle_normal.lengthSq()) axis = triangle_normal.cross(gen.unitVec());
            const angle = threshold_angle + gen.plain(-8.0e-6, 8.0e-6);
            const normal = rotate(triangle_normal, axis.normalized(), angle).mulScalar(if (gen.oneIn(4)) 1.0 else gen.plain(0.01, 10));
            const v0 = gen.plainVec(-10, 10);
            const v1 = gen.plainVec(-10, 10);
            const v2 = gen.plainVec(-10, 10);
            const point = v0.add(v1).add(v2).mulScalar(1.0 / 3.0);
            const active_edges: u8 = @intCast(1 + gen.index(6));
            checkFixNormal(&fix_normal_threshold, v0, v1, v2, triangle_normal, active_edges, point, normal, Vec3.zero());

            const dot = triangle_normal.dot(normal);
            const normal_length = normal.length();
            const triangle_normal_length = triangle_normal.length();
            const threshold = 0.999848 * normal_length * triangle_normal_length;
            if (dot == threshold) threshold_ties += 1;
            if ((dot > threshold) != (dot > 0.999848 * (normal_length * triangle_normal_length))) threshold_regrouped += 1;
        }

        // FixNormal with a barycentric coordinate of the contact point exactly at epsilon or one_minus_epsilon (ties of
        // the strict comparisons). Relative to the contact point, a right triangle P, Q, R with the right angle at
        // P = (-k * s, -o * short), Q = P + (10000 * s, 0) and R = P + (0, short) (s and short powers of 2, short >= 4 * s)
        // keeps GetBaryCentricCoordinates exact up to its final divisions: the weight of Q is k / 10000 (k = 1 or 9999,
        // which rounds to exactly epsilon / one_minus_epsilon) and the weight of R is o (a short dyadic number). P is v0
        // (the d00 <= d22 branch, the tie is v or w) or v2 (the other branch, the tie is u or v). Exact transformations
        // vary the input: a translation on the grid of the coordinates, the height of the plane and of the contact
        // point, axis permutations and mirroring.
        {
            const k: f32 = if (gen.oneIn(2)) 1.0 else 9999.0;
            // The weight of R: inside the triangle for epsilon, small for one_minus_epsilon (P gets the rest)
            const o: f32 = if (k == 1.0) @as(f32, @floatFromInt(1 + gen.below(14))) / 16.0 else ([_]f32{ 0.0, 0x1p-17, 0x1p-15, 0x1p-14 })[gen.index(4)];
            const s = std.math.ldexp(@as(f32, 1.0), @as(i32, @intCast(gen.below(13))) - 6);
            const short = s * std.math.ldexp(@as(f32, 1.0), @as(i32, @intCast(gen.below(3))) + 2);
            const t1 = @as(f32, @floatFromInt(gen.rng.intRange(i32, -8, 8))) * 1024.0 * s;
            const t2 = @as(f32, @floatFromInt(gen.rng.intRange(i32, -8, 8))) * short;
            const height = gen.plain(-10, 10);
            const point_height = if (gen.oneIn(2)) height else gen.plain(-10, 10);
            const axes = ([_][3]usize{ .{ 0, 1, 2 }, .{ 0, 2, 1 }, .{ 1, 0, 2 }, .{ 1, 2, 0 }, .{ 2, 0, 1 }, .{ 2, 1, 0 } })[gen.index(6)];
            var signs: [3]f32 = undefined;
            for (&signs) |*sign| sign.* = if (gen.oneIn(2)) 1.0 else -1.0;
            const p = [3]f32{ t1 - k * s, height, t2 - o * short };
            const vp = embed(p, axes, signs);
            const vq = embed(.{ p[0] + 10000.0 * s, height, p[2] }, axes, signs);
            const vr = embed(.{ p[0], height, p[2] + short }, axes, signs);
            const point = embed(.{ t1, point_height, t2 }, axes, signs);
            const roles = gen.index(4);
            const triangle: [3]Vec3 = switch (roles) {
                0 => .{ vp, vq, vr }, // v = k / 10000, w = o
                1 => .{ vp, vr, vq }, // v = o, w = k / 10000
                2 => .{ vq, vr, vp }, // u = k / 10000, v = o
                else => .{ vr, vq, vp }, // u = o, v = k / 10000
            };
            const v0 = triangle[0];
            const v1 = triangle[1];
            const v2 = triangle[2];
            const triangle_normal = if (gen.oneIn(2)) v1.sub(v0).cross(v2.sub(v0)) else gen.unitVec();
            const normal = if (gen.oneIn(2)) gen.unitVec() else gen.plainVec(-2, 2);
            const active_edges: u8 = @intCast(1 + gen.index(6));
            checkFixNormal(&fix_normal_bary, v0, v1, v2, triangle_normal, active_edges, point, normal, Vec3.zero());

            const bary = zolt.ClosestPoint.getBaryCentricCoordinatesTriangle(v0.sub(point), v1.sub(point), v2.sub(point));
            const weights = [3]f32{ bary.u, bary.v, bary.w };
            const q_index: usize = switch (roles) {
                0, 3 => 1,
                1 => 2,
                else => 0,
            };
            const r_index: usize = switch (roles) {
                0 => 2,
                1, 2 => 1,
                else => 0,
            };
            if (weights[q_index] == (if (k == 1.0) epsilon else one_minus_epsilon) and weights[r_index] == o) bary_ties += 1;
        }
    }

    try finishAll(&.{ &is_edge_active, &fix_normal, &fix_normal_threshold, &fix_normal_bary });

    // The generator hits the ties it aims at
    try std.testing.expectEqual(iterations, bary_ties);
    try std.testing.expect(threshold_ties >= iterations / 100);
    try std.testing.expect(threshold_regrouped >= iterations / 1000);
}
