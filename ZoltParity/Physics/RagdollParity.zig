//! Parity tests of RagdollSettings (Jolt/Physics/Ragdoll, Phase 7 Ragdoll group), compared bit for bit with the C++
//! library on random ragdolls (fixed seeds). C ABI wrappers: ZoltParity/Physics/RagdollReference.cpp. The runtime Ragdoll
//! (CreateRagdoll, AddToPhysicsSystem, poses, motors, ...) is compared by the ragdoll scenes of the simulation parity
//! (SimScenes.zig, Blocker.ragdolls).
//!
//! Both sides build the same RagdollSettings from a description (`Desc`): a random skeleton of 1 to 24 joints (several
//! roots, sometimes parents after their children), one part per joint with every BodyCreationSettings member (shapes,
//! materials and group filters from shared tables, shapes created from shape settings by GetShape, static mesh parts,
//! static / kinematic / dynamic parts with calculated or provided mass properties), mToParent and the additional
//! constraints from a table of constraint settings of every ported kind (the descriptions and settings of the
//! constraint parity, ConstraintsParity.randomDescription / makeDescriptionSettings and CSMakeConstraintSettings;
//! several parts can share one settings object, a root can have a constraint). Then both sides apply the same
//! operations in order and compare:
//! - Stabilize: its return value and the mass properties it writes (the binary state of the parts);
//! - CalculateConstraintPriorities (the priorities in the binary state of the constraint settings);
//! - DisableParentChildCollisions without and with joint matrices (random poses with rotations and uniform scales, where
//!   parts overlap or are within the minimum separation distance): IsCollisionEnabled of every sub group pair of the new
//!   GroupFilterTable, the sub group IDs, that every part shares the filter, and the table in the binary state;
//! - CalculateBodyIndexToConstraintIndex / CalculateConstraintIndexToBodyIdxPair and their getters (also
//!   GetConstraintIndexForBodyIndex / GetBodyIndicesForConstraintIndex);
//! - SaveBinaryState with every combination of inSaveShapes / inSaveGroupFilter;
//! - the cross restore: each side restores the other side's bytes with sRestoreFromBinaryState (the maps it calculates,
//!   the sub group IDs, the part and constraint counts) and saves them again with the same flags, which must give the
//!   original bytes;
//! - truncated streams (every length for the first cases, random lengths after that): the same error text on both
//!   sides, or the same restored settings (Jolt does not check the stream after the part count, the group filter ID of
//!   a part, the has-constraint flag and the additional constraint count, so some truncations restore settings with
//!   fewer parts, a last part without group filter or constraint, or no additional constraints).
//! Release only (`-Doptimize=ReleaseFast`, Jolt asserts in its debug build and so does Zolt in safe builds): Stabilize of
//! an inertia tensor with NaNs (it returns false), CalculateConstraintPriorities on skeletons whose parents come after
//! their children and with a base priority that overflows.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ConstraintsParity = @import("ConstraintsParity.zig");

const Checker = fw.Checker;
const Rng = fw.Rng;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const Color = zolt.Color;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const virtual = zolt.virtual;
const BodyCreationSettings = zolt.BodyCreationSettings;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const MeshShapeSettings = zolt.MeshShapeSettings;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const Triangle = zolt.Triangle;
const ConstraintSettings = zolt.ConstraintSettings;
const TwoBodyConstraintSettings = zolt.TwoBodyConstraintSettings;
const Skeleton = zolt.Skeleton;
const RagdollSettings = zolt.RagdollSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;

const max_joints = 24;
const max_additional = 8;
const max_ops = 8;
const max_pairs = max_joints + max_additional;
/// No index (cRPNone)
const none: u32 = 0xffffffff;

/// The C++ reference functions, see RagdollReference.cpp
const jolt = struct {
    extern fn jolt_rp_struct_sizes(out_sizes: *[12]u32) void;
    extern fn jolt_rp_run(tables: *const Tables, desc: *const Desc, out_result: *Result, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_rp_resave(bytes: [*]const u8, size: u32, save_shapes: u32, save_group_filter: u32, out_result: *Result, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
};

/// Must match RPShapeDesc in RagdollReference.cpp
const ShapeDesc = extern struct {
    /// 0 sphere, 1 box, 2 capsule, 3 offset COM box, 4 rotated translated capsule, 5 static compound of a box and a
    /// sphere, 6 uniformly scaled box, 7 mesh (a quad of two triangles in the XZ plane)
    kind: u32 = 0,
    /// Index in the material table or `none`
    material: u32 = none,
    /// 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
    params: [8]f32 = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
};

/// Must match RPMaterialDesc in RagdollReference.cpp
const MaterialDesc = extern struct {
    name: [16]u8 = @splat(0),
    color: u32 = 0,
};

/// Must match RPGroupFilterDesc in RagdollReference.cpp
const GroupFilterDesc = extern struct {
    num_sub_groups: u32 = 0,
    num_disabled: u32 = 0,
    disabled: [16]u32 = @splat(0),
};

/// Must match RPTables in RagdollReference.cpp
const Tables = extern struct {
    shapes: [*]const ShapeDesc,
    num_shapes: u32,
    materials: [*]const MaterialDesc,
    num_materials: u32,
    filters: [*]const GroupFilterDesc,
    num_filters: u32,
    /// The constraint settings (descriptions of the constraint parity)
    constraints: [*]const ConstraintsParity.CSSettings,
    num_constraints: u32,
};

/// Must match RPPartDesc in RagdollReference.cpp: the BodyCreationSettings of a part (Jolt's defaults), its parent joint
/// and its constraint
const PartDesc = extern struct {
    user_data: u64 = 0,
    position: [3]Real = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    linear_velocity: [3]f32 = @splat(0),
    angular_velocity: [3]f32 = @splat(0),
    friction: f32 = 0.2,
    restitution: f32 = 0.0,
    linear_damping: f32 = 0.05,
    angular_damping: f32 = 0.05,
    max_linear_velocity: f32 = 500.0,
    max_angular_velocity: f32 = 0.25 * std.math.pi * 60.0,
    gravity_factor: f32 = 1.0,
    inertia_multiplier: f32 = 1.0,
    mass: f32 = 0.0,
    inertia: [16]f32 = @splat(0),
    object_layer: u32 = 0,
    motion_type: u32 = 2,
    allowed_dofs: u32 = 0b111111,
    allow_dynamic_or_kinematic: u32 = 0,
    is_sensor: u32 = 0,
    collide_kinematic_vs_non_dynamic: u32 = 0,
    use_manifold_reduction: u32 = 1,
    apply_gyroscopic_force: u32 = 0,
    motion_quality: u32 = 0,
    enhanced_internal_edge_removal: u32 = 0,
    allow_sleeping: u32 = 1,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    override_mass_properties: u32 = 0,
    /// Index in the shape table or `none` (no shape)
    shape: u32 = none,
    /// 1: new ShapeSettings from the description of the shape (GetShape creates the shape)
    use_shape_settings: u32 = 0,
    /// Index in the group filter table or `none`
    group_filter: u32 = none,
    group_id: u32 = 0xffffffff,
    sub_group_id: u32 = 0xffffffff,
    /// The parent joint index of the skeleton (-1: a root)
    parent: i32 = -1,
    /// mToParent: index in the constraint table or `none`
    to_parent: u32 = none,
};

/// Must match RPAdditionalDesc in RagdollReference.cpp
const AdditionalDesc = extern struct {
    body_idx: [2]i32 = .{ 0, 0 },
    /// Index in the constraint table
    constraint: u32 = 0,
};

/// The operations (RPOp.mKind)
const OpKind = enum(u32) {
    stabilize = 0,
    calculate_constraint_priorities = 1,
    disable_parent_child_collisions = 2,
    disable_parent_child_collisions_with_matrices = 3,
    calculate_body_index_to_constraint_index = 4,
    calculate_constraint_index_to_body_idx_pair = 5,
};

/// Must match RPOp in RagdollReference.cpp
const Op = extern struct {
    kind: u32 = 0,
    base_priority: u32 = 0,
    min_separation_distance: f32 = 0,
};

/// Must match RPDesc in RagdollReference.cpp
const Desc = extern struct {
    num_joints: u32 = 0,
    num_additional: u32 = 0,
    num_ops: u32 = 0,
    parts: [max_joints]PartDesc = @splat(.{}),
    additional: [max_additional]AdditionalDesc = @splat(.{}),
    ops: [max_ops]Op = @splat(.{}),
    joint_matrices: [max_joints][16]f32 = @splat(@splat(0)),
};

/// Must match RPResult in RagdollReference.cpp (zeroed before every call)
const Result = extern struct {
    /// Per operation: Stabilize's result (0 / 1), `none` for the other operations
    stabilize: [max_ops]u32,
    num_parts: u32,
    num_additional: u32,
    num_body_to_constraint: u32,
    /// GetBodyIndexToConstraintIndex
    body_to_constraint: [max_joints]i32,
    /// GetConstraintIndexForBodyIndex(i)
    constraint_index_for_body_index: [max_joints]i32,
    num_pairs: u32,
    /// GetConstraintIndexToBodyIdxPair
    pairs: [max_pairs][2]i32,
    /// GetBodyIndicesForConstraintIndex(i)
    pairs_for_constraint_index: [max_pairs][2]i32,
    /// The joint count after a DisableParentChildCollisions, else 0
    filter_sub_groups: u32,
    /// Every part has the group filter of part 0
    filter_shared: u32,
    /// IsCollisionEnabled(i, j) of part 0's GroupFilterTable (i != j, after a DisableParentChildCollisions)
    collision_enabled: [max_joints][max_joints]u8,
    /// The sub group ID of every part
    sub_group_ids: [max_joints]u32,
    /// The sizes of the SaveBinaryState bytes of jolt_rp_run, index = save shapes << 1 | save group filter
    save_sizes: [4]u32,
};

const bytes_capacity = 1 << 20;
const error_capacity = 128;

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn load4(v: []const f32) Vec4 {
    return Vec4.init(v[0], v[1], v[2], v[3]);
}

fn loadQuat(v: []const f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

fn loadR3(v: []const Real) RVec3 {
    return RVec3.init(v[0], v[1], v[2]);
}

fn loadMat44(m: *const [16]f32) Mat44 {
    return Mat44.init(load4(m[0..4]), load4(m[4..8]), load4(m[8..12]), load4(m[12..16]));
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side (RPTableObjects / jolt_rp_run / jolt_rp_resave of RagdollReference.cpp)

/// The shapes, materials, group filters and constraint settings of a test
const TableObjects = struct {
    allocator: Allocator,
    materials: std.ArrayList(RefConst(PhysicsMaterial)) = .empty,
    filters: std.ArrayList(RefConst(GroupFilter)) = .empty,
    shapes: std.ArrayList(RefConst(Shape)) = .empty,
    constraints: std.ArrayList(Ref(ConstraintSettings)) = .empty,
    shape_descs: []const ShapeDesc = &.{},

    fn init(self: *TableObjects, allocator: Allocator, tables: *const Tables) !void {
        self.* = .{ .allocator = allocator };
        errdefer self.deinit();
        for (tables.materials[0..tables.num_materials]) |*m| {
            const name = std.mem.sliceTo(&m.name, 0);
            const material = try PhysicsMaterialSimple.create(allocator, name, Color.fromUInt32(m.color));
            try self.materials.append(allocator, .init(material.material()));
        }
        for (tables.filters[0..tables.num_filters]) |*f| {
            const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = f.num_sub_groups });
            try self.filters.append(allocator, .init(filter.asGroupFilter()));
            for (0..f.num_disabled) |j|
                filter.disableCollision(f.disabled[2 * j], f.disabled[2 * j + 1]);
        }
        self.shape_descs = tables.shapes[0..tables.num_shapes];
        for (self.shape_descs) |*d| {
            var settings = Ref(ShapeSettings).init(try self.makeShapeSettings(d));
            defer settings.deinit();
            var result = try settings.get().?.createShape(allocator);
            defer result.deinit();
            try self.shapes.append(allocator, .init(result.getPtr().?));
        }
        for (tables.constraints[0..tables.num_constraints]) |*c| {
            var settings = try ConstraintsParity.makeDescriptionSettings(allocator, c);
            errdefer settings.deinit();
            try self.constraints.append(allocator, settings);
        }
    }

    fn deinit(self: *TableObjects) void {
        for (self.constraints.items) |*c| c.deinit();
        self.constraints.deinit(self.allocator);
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(self.allocator);
        for (self.filters.items) |*f| f.deinit();
        self.filters.deinit(self.allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(self.allocator);
    }

    fn getMaterial(self: *const TableObjects, index: u32) ?*const PhysicsMaterial {
        return if (index < self.materials.items.len) self.materials.items[index].get() else null;
    }

    fn makeBoxSettings(self: *const TableObjects, desc: *const ShapeDesc) !*ShapeSettings {
        return (try BoxShapeSettings.create(self.allocator, load3(desc.params[0..3]), .{ .convex_radius = desc.params[3], .material = self.getMaterial(desc.material) })).asShapeSettings();
    }

    /// MakeShapeSettings of RagdollReference.cpp: the returned settings have no reference yet
    fn makeShapeSettings(self: *const TableObjects, desc: *const ShapeDesc) !*ShapeSettings {
        const a = self.allocator;
        const p = desc.params;
        const material = self.getMaterial(desc.material);
        return switch (desc.kind) {
            0 => (try SphereShapeSettings.create(a, p[0], .{ .material = material })).asShapeSettings(),
            1 => try self.makeBoxSettings(desc),
            2 => (try CapsuleShapeSettings.create(a, p[0], p[1], .{ .material = material })).asShapeSettings(),
            3 => (try OffsetCenterOfMassShapeSettings.create(a, load3(p[4..7]), try self.makeBoxSettings(desc))).asShapeSettings(),
            4 => (try RotatedTranslatedShapeSettings.create(a, load3(p[4..7]), loadQuat(&desc.rotation), (try CapsuleShapeSettings.create(a, p[0], p[1], .{ .material = material })).asShapeSettings())).asShapeSettings(),
            5 => blk: {
                const compound = try StaticCompoundShapeSettings.create(a);
                try compound.base.addShape(load3(p[4..7]), loadQuat(&desc.rotation), try self.makeBoxSettings(desc), .{});
                try compound.base.addShape(load3(p[4..7]).negate(), Quat.identity(), (try SphereShapeSettings.create(a, p[0], .{ .material = material })).asShapeSettings(), .{});
                break :blk compound.asShapeSettings();
            },
            6 => (try ScaledShapeSettings.create(a, try self.makeBoxSettings(desc), Vec3.replicate(p[4]))).asShapeSettings(),
            else => blk: {
                const triangles = [_]Triangle{
                    .init(Vec3.init(-p[0], 0, -p[0]), Vec3.init(-p[0], 0, p[0]), Vec3.init(p[0], 0, p[0]), .{}),
                    .init(Vec3.init(-p[0], 0, -p[0]), Vec3.init(p[0], 0, p[0]), Vec3.init(p[0], 0, -p[0]), .{}),
                };
                const materials: []const *const PhysicsMaterial = if (material) |m| &.{m} else &.{};
                break :blk (try MeshShapeSettings.create(a, &triangles, .{ .materials = materials })).asShapeSettings();
            },
        };
    }

    /// MakeBodyCreationSettings of RagdollReference.cpp (the caller deinits the result)
    fn makeBodyCreationSettings(self: *const TableObjects, desc: *const PartDesc) !BodyCreationSettings {
        var s: BodyCreationSettings = .{};
        errdefer s.deinit();
        s.position = loadR3(&desc.position);
        s.rotation = loadQuat(&desc.rotation);
        s.linear_velocity = load3(&desc.linear_velocity);
        s.angular_velocity = load3(&desc.angular_velocity);
        s.user_data = desc.user_data;
        s.object_layer = @intCast(desc.object_layer);
        s.collision_group = .init(if (desc.group_filter < self.filters.items.len) self.filters.items[desc.group_filter].get() else null, desc.group_id, desc.sub_group_id);
        s.motion_type = @enumFromInt(desc.motion_type);
        s.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
        s.allow_dynamic_or_kinematic = desc.allow_dynamic_or_kinematic != 0;
        s.is_sensor = desc.is_sensor != 0;
        s.collide_kinematic_vs_non_dynamic = desc.collide_kinematic_vs_non_dynamic != 0;
        s.use_manifold_reduction = desc.use_manifold_reduction != 0;
        s.apply_gyroscopic_force = desc.apply_gyroscopic_force != 0;
        s.motion_quality = @enumFromInt(desc.motion_quality);
        s.enhanced_internal_edge_removal = desc.enhanced_internal_edge_removal != 0;
        s.allow_sleeping = desc.allow_sleeping != 0;
        s.friction = desc.friction;
        s.restitution = desc.restitution;
        s.linear_damping = desc.linear_damping;
        s.angular_damping = desc.angular_damping;
        s.max_linear_velocity = desc.max_linear_velocity;
        s.max_angular_velocity = desc.max_angular_velocity;
        s.gravity_factor = desc.gravity_factor;
        s.num_velocity_steps_override = desc.num_velocity_steps_override;
        s.num_position_steps_override = desc.num_position_steps_override;
        s.override_mass_properties = @enumFromInt(desc.override_mass_properties);
        s.inertia_multiplier = desc.inertia_multiplier;
        s.mass_properties_override.mass = desc.mass;
        s.mass_properties_override.inertia = loadMat44(&desc.inertia);
        if (desc.shape < self.shapes.items.len) {
            if (desc.use_shape_settings != 0)
                s.setShapeSettings(try self.makeShapeSettings(&self.shape_descs[desc.shape]))
            else
                s.setShape(self.shapes.items[desc.shape].get());
        }
        return s;
    }

    /// The constraint settings of the table (TwoBodyConstraintSettings)
    fn constraint(self: *const TableObjects, index: u32) *TwoBodyConstraintSettings {
        return virtual.downcast(TwoBodyConstraintSettings, self.constraints.items[index].get().?);
    }

    /// MakeRagdollSettings of RagdollReference.cpp (one reference owned by the caller)
    fn makeRagdollSettings(self: *const TableObjects, desc: *const Desc) !Ref(RagdollSettings) {
        const a = self.allocator;
        var settings = Ref(RagdollSettings).init(try RagdollSettings.create(a));
        errdefer settings.deinit();
        const s = settings.get().?;

        const skeleton = try Skeleton.create(a);
        s.skeleton.set(skeleton);
        var name_buffer: [16]u8 = undefined;
        for (desc.parts[0..desc.num_joints], 0..) |*d, j| {
            const name = try std.fmt.bufPrint(&name_buffer, "J{d}", .{j});
            if (d.parent < @as(i32, @intCast(j)))
                _ = try skeleton.addJointWithParentIndex(name, d.parent)
            else
                _ = try skeleton.addJoint(name, .{});
        }
        for (desc.parts[0..desc.num_joints], 0..) |*d, j|
            skeleton.getJointMut(j).parent_joint_index = d.parent;

        try s.parts.ensureTotalCapacity(a, desc.num_joints);
        for (desc.parts[0..desc.num_joints]) |*d| {
            var body = try self.makeBodyCreationSettings(d);
            defer body.deinit();
            s.parts.appendAssumeCapacity(.init(&body, if (d.to_parent != none) self.constraint(d.to_parent) else null));
        }
        try s.additional_constraints.ensureTotalCapacity(a, desc.num_additional);
        for (desc.additional[0..desc.num_additional]) |*d|
            s.additional_constraints.appendAssumeCapacity(.init(d.body_idx[0], d.body_idx[1], self.constraint(d.constraint)));
        return settings;
    }
};

/// RPWriteMaps of RagdollReference.cpp
fn writeMaps(settings: *const RagdollSettings, result: *Result) void {
    result.num_parts = @intCast(settings.parts.items.len);
    result.num_additional = @intCast(settings.additional_constraints.items.len);
    const body_to_constraint = settings.getBodyIndexToConstraintIndex();
    result.num_body_to_constraint = @intCast(body_to_constraint.len);
    for (body_to_constraint[0..@min(body_to_constraint.len, max_joints)], 0..) |c, i| {
        result.body_to_constraint[i] = c;
        result.constraint_index_for_body_index[i] = settings.getConstraintIndexForBodyIndex(i);
    }
    const pairs = settings.getConstraintIndexToBodyIdxPair();
    result.num_pairs = @intCast(pairs.len);
    for (pairs[0..@min(pairs.len, max_pairs)], 0..) |pair, i| {
        result.pairs[i] = .{ pair.first, pair.second };
        const p = settings.getBodyIndicesForConstraintIndex(i);
        result.pairs_for_constraint_index[i] = .{ p.first, p.second };
    }
}

/// RPWriteGroups of RagdollReference.cpp
fn writeGroups(settings: *const RagdollSettings, filter_sub_groups: u32, result: *Result) void {
    const parts = settings.parts.items;
    result.filter_sub_groups = filter_sub_groups;
    const filter: ?*const GroupFilter = if (parts.len > 0) parts[0].base.collision_group.getGroupFilter() else null;
    result.filter_shared = 1;
    for (parts, 0..) |*p, i| {
        const group = &p.base.collision_group;
        if (group.getGroupFilter() != filter)
            result.filter_shared = 0;
        result.sub_group_ids[i] = group.getSubGroupID();
    }
    if (filter_sub_groups > 0) {
        const table = virtual.downcast(GroupFilterTable, filter.?);
        for (0..filter_sub_groups) |i| {
            for (0..filter_sub_groups) |j| {
                if (i != j)
                    result.collision_enabled[i][j] = @intFromBool(table.isCollisionEnabled(@intCast(i), @intCast(j)));
            }
        }
    }
}

/// SaveBinaryState into an allocating writer, returns the bytes (owned by the writer)
fn save(settings: *const RagdollSettings, out: *std.Io.Writer.Allocating, save_shapes: bool, save_group_filter: bool) ![]const u8 {
    var wrapper = StreamOutWrapper.init(&out.writer);
    try settings.saveBinaryState(wrapper.streamOut(), save_shapes, save_group_filter);
    if (wrapper.isFailed()) return error.OutOfMemory;
    return out.written();
}

/// jolt_rp_run on the Zolt side: the four byte strings are appended to `bytes`
fn zoltRun(allocator: Allocator, tables_desc: *const Tables, desc: *const Desc, result: *Result, bytes: *std.ArrayList(u8)) !void {
    var tables: TableObjects = undefined;
    try tables.init(allocator, tables_desc);
    defer tables.deinit();
    var settings = try tables.makeRagdollSettings(desc);
    defer settings.deinit();
    const s = settings.get().?;

    var filter_sub_groups: u32 = 0;
    for (desc.ops[0..desc.num_ops], 0..) |*op, o| {
        result.stabilize[o] = none;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .stabilize => result.stabilize[o] = @intFromBool(try s.stabilize()),
            .calculate_constraint_priorities => try s.calculateConstraintPriorities(.{ .base_priority = op.base_priority }),
            .disable_parent_child_collisions => {
                try s.disableParentChildCollisions(.{});
                filter_sub_groups = desc.num_joints;
            },
            .disable_parent_child_collisions_with_matrices => {
                var matrices: [max_joints]Mat44 = undefined;
                for (matrices[0..desc.num_joints], desc.joint_matrices[0..desc.num_joints]) |*m, *d|
                    m.* = loadMat44(d);
                try s.disableParentChildCollisions(.{ .joint_matrices = matrices[0..desc.num_joints], .min_separation_distance = op.min_separation_distance });
                filter_sub_groups = desc.num_joints;
            },
            .calculate_body_index_to_constraint_index => try s.calculateBodyIndexToConstraintIndex(),
            .calculate_constraint_index_to_body_idx_pair => try s.calculateConstraintIndexToBodyIdxPair(),
        }
    }

    writeMaps(s, result);
    writeGroups(s, filter_sub_groups, result);

    for (0..4) |k| {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        const data = try save(s, &out, (k & 2) != 0, (k & 1) != 0);
        result.save_sizes[k] = @intCast(data.len);
        try bytes.appendSlice(allocator, data);
    }
}

/// The outcome of a restore and save: the bytes or Jolt's error text
const Resaved = struct {
    bytes: []u8 = &.{},
    error_text: [error_capacity]u8 = @splat(0),
    failed: bool = false,
    result: Result = std.mem.zeroes(Result),

    fn deinit(self: *Resaved, allocator: Allocator) void {
        allocator.free(self.bytes);
    }

    fn errorText(self: *const Resaved) []const u8 {
        return std.mem.sliceTo(&self.error_text, 0);
    }
};

/// jolt_rp_resave on the Zolt side
fn zoltResave(allocator: Allocator, bytes: []const u8, save_shapes: bool, save_group_filter: bool) !Resaved {
    var resaved: Resaved = .{};
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try RagdollSettings.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    if (result.hasError()) {
        const text = result.getError();
        @memcpy(resaved.error_text[0..text.len], text);
        resaved.failed = true;
        return resaved;
    }
    const s = result.getPtr().?;
    writeMaps(s, &resaved.result);
    writeGroups(s, 0, &resaved.result);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    resaved.bytes = try allocator.dupe(u8, try save(s, &out, save_shapes, save_group_filter));
    return resaved;
}

/// jolt_rp_resave
fn joltResave(allocator: Allocator, bytes: []const u8, save_shapes: bool, save_group_filter: bool) !Resaved {
    var resaved: Resaved = .{};
    const buffer = try allocator.alloc(u8, bytes_capacity);
    defer allocator.free(buffer);
    const size = jolt.jolt_rp_resave(bytes.ptr, @intCast(bytes.len), @intFromBool(save_shapes), @intFromBool(save_group_filter), &resaved.result, buffer.ptr, bytes_capacity, &resaved.error_text, error_capacity);
    if (size == none) {
        resaved.failed = true;
        return resaved;
    }
    if (size > bytes_capacity) return error.BufferTooSmall;
    resaved.bytes = try allocator.dupe(u8, buffer[0..size]);
    return resaved;
}

// ---------------------------------------------------------------------------------------------------------------------
// The generator

const Gen = struct {
    rng: Rng,

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn index(self: *Gen, n: usize) usize {
        return self.rng.intRange(usize, 0, n - 1);
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.rng.intRange(u32, 0, n - 1) == 0;
    }

    fn unitQuat(self: *Gen) Quat {
        const axis = Vec3.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1));
        if (axis.lengthSq() < 1.0e-4) return Quat.identity();
        return Quat.rotation(axis.normalized(), self.float(-std.math.pi, std.math.pi));
    }

    fn storeQuat(q: Quat, out: *[4]f32) void {
        out.* = .{ q.getX(), q.getY(), q.getZ(), q.getW() };
    }

    fn storeMat44(m: Mat44, out: *[16]f32) void {
        for (0..4) |c| {
            const col = m.getColumn4(@intCast(c));
            out[4 * c ..][0..4].* = .{ col.getX(), col.getY(), col.getZ(), col.getW() };
        }
    }

    /// A shape of each kind (sizes of body parts, the mesh large enough to touch the other parts)
    fn shapes(self: *Gen, num_materials: usize) [8]ShapeDesc {
        var result: [8]ShapeDesc = undefined;
        for (&result, 0..) |*s, kind| {
            s.* = .{ .kind = @intCast(kind) };
            s.material = if (self.oneIn(3)) none else @intCast(self.index(num_materials));
            s.params[0] = self.float(0.05, 0.3);
            s.params[1] = self.float(0.03, 0.15);
            s.params[2] = self.float(0.05, 0.3);
            s.params[3] = @min(s.params[0], s.params[1], s.params[2]) * self.float(0, 0.5);
            for (s.params[4..7]) |*p| p.* = self.float(-0.2, 0.2);
            if (kind == 6) s.params[4] = self.float(0.5, 2.0); // The uniform scale
            if (kind == 7) s.params[0] = self.float(0.5, 2.0); // The half size of the quad
            storeQuat(self.unitQuat(), &s.rotation);
        }
        return result;
    }

    /// A random symmetric positive definite inertia tensor R * D * R^T
    fn inertia(self: *Gen) Mat44 {
        const r = Mat44.rotationQuat(self.unitQuat());
        const d = Mat44.scaleVec3(Vec3.init(self.float(0.05, 2), self.float(0.05, 2), self.float(0.05, 2)));
        var m = r.mul(d).mul(r.transposed());
        m.setColumn4(3, Vec4.init(0, 0, 0, 1));
        return m;
    }

    /// The parents of a skeleton of `n` joints: a random tree (several roots) in index order, or in a random order
    /// (parents after their children)
    fn parents(self: *Gen, n: usize, ordered: bool, out: []i32) void {
        var order: [max_joints]usize = undefined;
        for (order[0..n], 0..) |*o, i| o.* = i;
        if (!ordered) self.shuffle(order[0..n]);
        for (order[0..n], 0..) |joint, k| {
            out[joint] = if (k == 0 or self.oneIn(6)) -1 else @intCast(order[self.index(k)]);
        }
    }

    fn shuffle(self: *Gen, items: []usize) void {
        var i: usize = items.len;
        while (i > 1) {
            i -= 1;
            const j = self.index(i + 1);
            std.mem.swap(usize, &items[i], &items[j]);
        }
    }

    /// A part: every member random, a shape of the table (static mesh parts), the mass properties calculated or
    /// provided
    fn part(self: *Gen, num_shapes: usize, num_filters: usize, num_constraints: usize, parent: i32) PartDesc {
        var p: PartDesc = .{ .parent = parent };
        p.user_data = (@as(u64, self.rng.next()) << 32) | self.rng.next();
        for (&p.position) |*v| v.* = self.float(-1.5, 1.5);
        storeQuat(self.unitQuat(), &p.rotation);
        for (&p.linear_velocity) |*v| v.* = self.float(-3, 3);
        for (&p.angular_velocity) |*v| v.* = self.float(-2, 2);
        p.friction = self.float(0, 1);
        p.restitution = self.float(0, 1);
        p.linear_damping = self.float(0, 0.5);
        p.angular_damping = self.float(0, 0.5);
        p.max_linear_velocity = self.float(50, 500);
        p.max_angular_velocity = self.float(10, 50);
        p.gravity_factor = self.float(0, 2);
        p.inertia_multiplier = if (self.oneIn(2)) 1.0 else self.float(0.5, 2.0);
        p.object_layer = @intCast(self.index(4));
        p.motion_type = switch (self.index(8)) {
            0 => 0, // Static
            1 => 1, // Kinematic
            else => 2, // Dynamic
        };
        p.allow_dynamic_or_kinematic = @intFromBool(p.motion_type == 0 and self.oneIn(2));
        p.allowed_dofs = switch (self.index(4)) {
            0 => 0b100011, // Plane2D
            1 => 0b000111, // Translations only
            else => 0b111111,
        };
        p.is_sensor = @intFromBool(self.oneIn(8));
        p.collide_kinematic_vs_non_dynamic = @intFromBool(self.oneIn(4));
        p.use_manifold_reduction = @intFromBool(!self.oneIn(4));
        p.apply_gyroscopic_force = @intFromBool(self.oneIn(4));
        p.motion_quality = @intCast(self.index(2));
        p.enhanced_internal_edge_removal = @intFromBool(self.oneIn(4));
        p.allow_sleeping = @intFromBool(!self.oneIn(4));
        p.num_velocity_steps_override = if (self.oneIn(2)) 0 else @intCast(self.index(20));
        p.num_position_steps_override = if (self.oneIn(2)) 0 else @intCast(self.index(20));

        // A static part that cannot become dynamic can be a mesh, the others have a convex shape or a compound
        const mesh = p.motion_type == 0 and p.allow_dynamic_or_kinematic == 0 and self.oneIn(3);
        p.shape = if (mesh) 7 else @intCast(self.index(num_shapes - 1));
        p.use_shape_settings = @intFromBool(self.oneIn(4));
        p.override_mass_properties = @intCast(self.index(3));
        p.mass = self.float(0.5, 20);
        storeMat44(self.inertia(), &p.inertia);

        p.group_filter = if (self.oneIn(3)) none else @intCast(self.index(num_filters));
        p.group_id = if (self.oneIn(2)) 0xffffffff else self.rng.next() % 8;
        p.sub_group_id = self.rng.next() % 4;
        p.to_parent = if ((parent >= 0 and !self.oneIn(6)) or (parent < 0 and self.oneIn(10))) @intCast(self.index(num_constraints)) else none;
        return p;
    }

    /// A ragdoll description with random operations. `release_only` adds the cases where Jolt asserts in its debug build.
    fn desc(self: *Gen, num_shapes: usize, num_filters: usize, num_constraints: usize, max_num_joints: usize, release_only: bool) Desc {
        var d: Desc = .{};
        d.num_joints = @intCast(1 + self.index(max_num_joints));
        const n = d.num_joints;
        const ordered = !self.oneIn(5);
        var parent_indices: [max_joints]i32 = undefined;
        self.parents(n, ordered, parent_indices[0..n]);
        for (0..n) |j|
            d.parts[j] = self.part(num_shapes, num_filters, num_constraints, parent_indices[j]);
        d.num_additional = if (self.oneIn(3)) 0 else @intCast(self.index(@min(4, max_additional) + 1));
        for (d.additional[0..d.num_additional]) |*a|
            a.* = .{ .body_idx = .{ @intCast(self.index(n)), @intCast(self.index(n)) }, .constraint = @intCast(self.index(num_constraints)) };

        // A pose: around the positions of the parts (overlapping or close parts), rotated, sometimes uniformly scaled
        for (d.joint_matrices[0..n], d.parts[0..n]) |*m, *p| {
            var matrix = Mat44.rotationTranslation(self.unitQuat(), Vec3.init(@floatCast(p.position[0]), @floatCast(p.position[1]), @floatCast(p.position[2])).mulScalar(self.float(0.1, 1)));
            if (self.oneIn(4)) matrix = matrix.mul(Mat44.scale(self.float(0.8, 1.25)));
            storeMat44(matrix, m);
        }

        // The operations: the asserts of Jolt's debug build only in release builds (a NaN inertia that Stabilize cannot
        // decompose, CalculateConstraintPriorities of an unordered skeleton or with an overflowing base priority)
        d.num_ops = @intCast(1 + self.index(max_ops));
        for (d.ops[0..d.num_ops]) |*op| {
            var kind: OpKind = @enumFromInt(self.index(6));
            if (kind == .calculate_constraint_priorities and !ordered and !release_only)
                kind = .stabilize;
            op.* = .{ .kind = @intFromEnum(kind) };
            switch (kind) {
                .calculate_constraint_priorities => op.base_priority = if (release_only and self.oneIn(3)) 0xffffffff - @as(u32, @intCast(self.index(4))) else @intCast(self.index(100)),
                .disable_parent_child_collisions_with_matrices => op.min_separation_distance = if (self.oneIn(3)) 0 else self.float(0, 0.3),
                else => {},
            }
        }
        if (release_only and self.oneIn(2)) {
            // A provided inertia with NaNs
            const p = &d.parts[self.index(n)];
            p.motion_type = 2;
            p.override_mass_properties = 2;
            if (p.shape == 7) p.shape = 0;
            for (p.inertia[0..12]) |*v| v.* = std.math.nan(f32);
        }
        return d;
    }
};

/// The tables of a test (shape, material, group filter and constraint descriptions)
const TestTables = struct {
    shapes: [8]ShapeDesc,
    materials: [2]MaterialDesc,
    filters: [2]GroupFilterDesc,
    constraints: [12]ConstraintsParity.CSSettings,
    num_constraints: u32,

    fn generate(g: *Gen) TestTables {
        var t: TestTables = undefined;
        t.materials = .{
            .{ .name = "Mat0".* ++ @as([12]u8, @splat(0)), .color = 0xff00ff00 },
            .{ .name = "Mat1".* ++ @as([12]u8, @splat(0)), .color = 0xffff0000 },
        };
        t.shapes = g.shapes(t.materials.len);
        t.filters = .{ .{ .num_sub_groups = 4, .num_disabled = 1, .disabled = .{ 0, 1 } ++ @as([14]u32, @splat(0)) }, .{ .num_sub_groups = 8, .num_disabled = 2, .disabled = .{ 2, 3, 7, 1 } ++ @as([12]u32, @splat(0)) } };
        t.num_constraints = @intCast(1 + g.index(t.constraints.len));
        for (t.constraints[0..t.num_constraints]) |*c| {
            const kinds = ConstraintsParity.ported_kinds;
            c.* = ConstraintsParity.randomDescription(g.rng.next(), kinds[g.index(kinds.len)]);
        }
        return t;
    }

    fn tables(self: *const TestTables) Tables {
        return .{
            .shapes = &self.shapes,
            .num_shapes = self.shapes.len,
            .materials = &self.materials,
            .num_materials = self.materials.len,
            .filters = &self.filters,
            .num_filters = self.filters.len,
            .constraints = &self.constraints,
            .num_constraints = self.num_constraints,
        };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The tests

test "Ragdoll parity: struct sizes" {
    var sizes: [12]u32 = undefined;
    jolt.jolt_rp_struct_sizes(&sizes);
    try std.testing.expectEqual([12]u32{ @sizeOf(ShapeDesc), @sizeOf(MaterialDesc), @sizeOf(GroupFilterDesc), @sizeOf(Tables), @sizeOf(PartDesc), @sizeOf(AdditionalDesc), @sizeOf(Op), @sizeOf(Desc), @sizeOf(Result), @offsetOf(PartDesc, "object_layer"), @offsetOf(Desc, "joint_matrices"), @offsetOf(Result, "save_sizes") }, sizes);
}

/// Statistics of a run (what the cases reached)
const Stats = struct {
    stabilize_true: u32 = 0,
    stabilize_false: u32 = 0,
    filter_pairs_disabled_by_pose: u32 = 0,
    valid_truncations: u32 = 0,
    truncation_errors: u32 = 0,
    restored_without_parts: u32 = 0,
    /// CalculateConstraintPriorities with a base priority that overflows (Jolt asserts "Base priority is too high and
    /// will cause overflows": release only)
    priority_base_overflows: u32 = 0,
    /// CalculateConstraintPriorities of a skeleton whose joints are not ordered (Jolt asserts AreJointsCorrectlyOrdered:
    /// release only)
    priority_unordered_skeletons: u32 = 0,
};

/// Count the CalculateConstraintPriorities operations of a description that Jolt's debug build asserts on
fn countPriorityBranches(desc: *const Desc, stats: *Stats) void {
    var ordered = true;
    for (desc.parts[0..desc.num_joints], 0..) |p, j| {
        if (p.parent >= @as(i32, @intCast(j))) ordered = false;
    }
    for (desc.ops[0..desc.num_ops]) |op| {
        if (op.kind != @intFromEnum(OpKind.calculate_constraint_priorities)) continue;
        if (op.base_priority +% desc.num_joints <= op.base_priority) stats.priority_base_overflows += 1;
        if (!ordered) stats.priority_unordered_skeletons += 1;
    }
}

/// Compare two results field by field: the first differing field (and element) is reported with both values
fn checkResult(checker: *Checker, case: usize, what: []const u8, z: *const Result, j: *const Result) void {
    inline for (@typeInfo(Result).@"struct".fields) |field| {
        const zv = @field(z, field.name);
        const jv = @field(j, field.name);
        if (!fw.sameValue(zv, jv)) {
            if (checker.mismatches < 5) {
                switch (@typeInfo(field.type)) {
                    .array => {
                        for (zv, jv, 0..) |a, b, i| {
                            if (!fw.sameValue(a, b)) {
                                std.debug.print("{s}: case {d} ({s}): {s}[{d}] zolt {any} jolt {any}\n", .{ checker.name, case, what, field.name, i, a, b });
                                break;
                            }
                        }
                    },
                    else => std.debug.print("{s}: case {d} ({s}): {s} zolt {any} jolt {any}\n", .{ checker.name, case, what, field.name, zv, jv }),
                }
            }
            checker.mismatches += 1;
            return;
        }
    }
}

/// Compare the restore and save of `bytes` on both sides
fn checkResave(allocator: Allocator, checker: *Checker, case: usize, what: []const u8, bytes: []const u8, save_shapes: bool, save_group_filter: bool, expected: ?[]const u8, stats: *Stats) !void {
    var z = try zoltResave(allocator, bytes, save_shapes, save_group_filter);
    defer z.deinit(allocator);
    var j = try joltResave(allocator, bytes, save_shapes, save_group_filter);
    defer j.deinit(allocator);
    const input = .{ case, what, bytes.len, save_shapes, save_group_filter };
    checker.check(input, z.failed, j.failed);
    if (z.failed or j.failed) {
        if (z.failed and j.failed and !std.mem.eql(u8, z.errorText(), j.errorText())) {
            if (checker.mismatches < 5)
                std.debug.print("case {d} {s} ({d} bytes): zolt error \"{s}\", jolt error \"{s}\"\n", .{ case, what, bytes.len, z.errorText(), j.errorText() });
            checker.mismatches += 1;
        }
        if (expected != null) {
            if (checker.mismatches < 5)
                std.debug.print("case {d} {s}: the restore failed: zolt \"{s}\" jolt \"{s}\"\n", .{ case, what, z.errorText(), j.errorText() });
            checker.mismatches += 1;
        }
        stats.truncation_errors += 1;
        return;
    }
    stats.valid_truncations += 1;
    if (z.result.num_parts == 0) stats.restored_without_parts += 1;
    checkResult(checker, case, what, &z.result, &j.result);
    if (!std.mem.eql(u8, z.bytes, j.bytes)) {
        if (checker.mismatches < 5)
            std.debug.print("case {d} {s}: the saved bytes differ at {?d} (zolt {d} bytes, jolt {d} bytes)\n", .{ case, what, std.mem.indexOfDiff(u8, z.bytes, j.bytes), z.bytes.len, j.bytes.len });
        checker.mismatches += 1;
    }
    if (expected) |e| {
        if (!std.mem.eql(u8, z.bytes, e)) {
            if (checker.mismatches < 5)
                std.debug.print("case {d} {s}: the restored settings save other bytes (at {?d})\n", .{ case, what, std.mem.indexOfDiff(u8, z.bytes, e) });
            checker.mismatches += 1;
        }
    }
}

/// Run `num_cases` random cases (release_only: with the cases where Jolt asserts in its debug build)
fn runCases(seed: u32, num_cases: usize, release_only: bool) !Stats {
    const allocator = std.testing.allocator;
    var g: Gen = .{ .rng = .{ .state = seed } };
    var stats: Stats = .{};
    var run_checker: Checker = .{ .name = "RagdollSettings run (results)" };
    var bytes_checker: Checker = .{ .name = "RagdollSettings run (SaveBinaryState bytes)" };
    var resave_checker: Checker = .{ .name = "RagdollSettings sRestoreFromBinaryState / SaveBinaryState" };

    const jolt_bytes = try allocator.alloc(u8, bytes_capacity);
    defer allocator.free(jolt_bytes);

    for (0..num_cases) |case| {
        const test_tables = TestTables.generate(&g);
        const tables = test_tables.tables();
        const max_num_joints: usize = if (case < 40) 6 else max_joints;
        const desc = g.desc(tables.num_shapes, tables.num_filters, tables.num_constraints, max_num_joints, release_only);
        countPriorityBranches(&desc, &stats);

        // Both sides build the settings, apply the operations and save them
        var jolt_result = std.mem.zeroes(Result);
        const jolt_size = jolt.jolt_rp_run(&tables, &desc, &jolt_result, jolt_bytes.ptr, bytes_capacity);
        try std.testing.expect(jolt_size <= bytes_capacity);
        var zolt_result = std.mem.zeroes(Result);
        var zolt_bytes: std.ArrayList(u8) = .empty;
        defer zolt_bytes.deinit(allocator);
        try zoltRun(allocator, &tables, &desc, &zolt_result, &zolt_bytes);
        checkResult(&run_checker, case, "run", &zolt_result, &jolt_result);
        if (!std.mem.eql(u8, zolt_bytes.items, jolt_bytes[0..jolt_size])) {
            if (bytes_checker.mismatches < 5)
                std.debug.print("case {d}: the SaveBinaryState bytes differ at {?d} (sizes zolt {any} jolt {any})\n", .{ case, std.mem.indexOfDiff(u8, zolt_bytes.items, jolt_bytes[0..jolt_size]), zolt_result.save_sizes, jolt_result.save_sizes });
            bytes_checker.mismatches += 1;
            continue;
        }

        for (jolt_result.stabilize[0..desc.num_ops]) |r| {
            if (r == 1) stats.stabilize_true += 1;
            if (r == 0) stats.stabilize_false += 1;
        }
        for (0..desc.num_joints) |a| {
            for (0..a) |b| {
                if (jolt_result.filter_sub_groups > 0 and jolt_result.collision_enabled[a][b] == 0 and desc.parts[a].parent != @as(i32, @intCast(b)) and desc.parts[b].parent != @as(i32, @intCast(a)))
                    stats.filter_pairs_disabled_by_pose += 1;
            }
        }

        // The cross restore: each side restores the bytes of the other side (equal, see above) and saves them again
        var offset: usize = 0;
        for (0..4) |k| {
            const size = jolt_result.save_sizes[k];
            const bytes = jolt_bytes[offset..][0..size];
            offset += size;
            const save_shapes = (k & 2) != 0;
            const save_group_filter = (k & 1) != 0;
            try checkResave(allocator, &resave_checker, case, "restore", bytes, save_shapes, save_group_filter, bytes, &stats);

            // Truncated streams: every length for the first cases, some lengths later
            if (case < 12) {
                for (0..bytes.len) |len|
                    try checkResave(allocator, &resave_checker, case, "truncated", bytes[0..len], save_shapes, save_group_filter, null, &stats);
            } else {
                for (0..8) |_|
                    try checkResave(allocator, &resave_checker, case, "truncated", bytes[0..g.index(bytes.len)], save_shapes, save_group_filter, null, &stats);
            }
        }
    }
    try fw.finishAll(&.{ &run_checker, &bytes_checker, &resave_checker });
    return stats;
}

test "Ragdoll parity: RagdollSettings" {
    const stats = try runCases(0x5a3c1e07, 400, false);

    // The cases reach what they are written for
    try std.testing.expect(stats.stabilize_true > 50);
    try std.testing.expect(stats.filter_pairs_disabled_by_pose > 20);
    try std.testing.expect(stats.valid_truncations > 100);
    try std.testing.expect(stats.truncation_errors > 1000);
    try std.testing.expect(stats.restored_without_parts > 0);
    // The cases that Jolt asserts on are left to the release only test
    try std.testing.expectEqual(@as(u32, 0), stats.priority_base_overflows);
    try std.testing.expectEqual(@as(u32, 0), stats.priority_unordered_skeletons);
}

test "Ragdoll parity: RagdollSettings (release only)" {
    if (Core.enable_asserts) return error.SkipZigTest;
    const stats = try runCases(0x0badcafe, 200, true);
    // The cases reach the branches that Jolt asserts on: a NaN inertia that Stabilize cannot decompose,
    // CalculateConstraintPriorities with an overflowing base priority and of an unordered skeleton
    try std.testing.expect(stats.stabilize_false > 5);
    try std.testing.expect(stats.priority_base_overflows > 0);
    try std.testing.expect(stats.priority_unordered_skeletons > 0);
}
