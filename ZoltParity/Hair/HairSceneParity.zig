//! The "Hair scenes" layer of the parity (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, R14): Zolt's Hair in a
//! PhysicsSystem against Jolt's, bit for bit. C ABI wrapper: ZoltParity/Hair/HairSceneReference.cpp.
//!
//! - A scene is a PhysicsSystem with rigid bodies, a hair and a list of operations. Both sides build the same scene and
//!   write a transcript. The transcripts must be equal byte for byte; a mismatch is reported with the operation, the
//!   record (a member, a compute buffer or a read back array) and the first differing element.
//! - The bodies: static and kinematic convex hulls, kinematic static and mutable compounds of hulls (a child scaled, a
//!   sphere child), scaled hulls, inside out hulls (a scale with an odd number of negative components), rotated and
//!   translated hulls, a dynamic hull. Hair collects ConvexHullShape leaves only, so it must ignore the boxes, spheres
//!   and capsules, a sensor hull, a hull in a layer that does not collide with the hair and a hull outside the bounds.
//! - The settings: the procedural grooms of the HairSettings layer (HairSettingsParity.zig): straight and curly strands
//!   of different lengths, several materials with a simulation fraction below 1, the scalp of Assets/face.bin with its
//!   skin or without a scalp. The Zig side runs InitRenderAndSimulationStrands and Init, both sides restore the
//!   SaveBinaryState bytes and run InitCompute.
//! - The operations: Hair::Update with delta times 0, 1/60, 1/90, odd values and values above mMaxDeltaTime, with the
//!   joint matrices of the face animation (the hair follows the neck joint like Jolt's HairTest sample) or without;
//!   SetPosition and SetRotation (the delta transform), OnTeleported, the movement and the velocities of a body.
//! - The materials: the defaults, a global pose only material (alone: the ApplyGlobalPose path; with others), the grid
//!   off (no grid velocity and no density force) and on through the density force only, LRA off, gravity factor
//!   gradients, damping, friction, the velocity limits, the hair radius, another gravity.
//! - After every operation the transcript holds the members of the hair (teleported, previous position and rotation)
//!   and the bytes of every compute buffer of the hair, the iteration constant buffers included. After an update also
//!   the data of ReadBackGPUState and LockReadBackBuffers: the positions, rotations, velocities, angular velocities,
//!   the grid, the render positions and the skinned scalp vertices.
//! - The scenes avoid Jolt's undefined behavior (R20): every scene with collision has a hull in the bounds, so the
//!   collision shapes buffer is never empty (UB3); the bounds have padding and the strands stay finite, so no grid
//!   position is NaN (UB11); the gradients are never steps and no last segment is too short to change the strand
//!   length (UB16). The reference zero-fills its allocations (UB4).
//! - The layer runs in both precisions: the hair position and the previous position are RVec3.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const TestLayers = @import("test_layers");
const HairSettingsParity = @import("HairSettingsParity.zig");

const Hair = zolt.Hair;
const HairSettings = zolt.HairSettings;
const HairShaders = zolt.HairShaders;
const Material = HairSettings.Material;
const Gradient = HairSettings.Gradient;
const HairStructs = zolt.HairStructs;
const ComputeSystem = zolt.ComputeSystem;
const ComputeSystemCPU = zolt.ComputeSystemCPU;
const ComputeQueue = zolt.ComputeQueue;
const ComputeBuffer = zolt.ComputeBuffer;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const Float3 = zolt.Float3;
const Float4 = zolt.Float4;
const UVec4 = zolt.UVec4;
const PhysicsSystem = zolt.PhysicsSystem;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const ShapeSettings = zolt.ShapeSettings;
const Layers = TestLayers.Layers;
const Face = HairSettingsParity.Face;
const Groom = HairSettingsParity.Groom;

const io = std.testing.io;

const jolt = struct {
    extern fn jolt_hs_struct_sizes(out: *[4]u32) void;
    extern fn jolt_hs_run(world: *const World, bodies: [*]const Body, points: [*]const f32, settings: [*]const u8, settings_size: u32, hair: *const HairDesc, ops: [*]const Op, num_ops: u32, pool: [*]const f32, out: [*]u8, capacity: u32) u32;
};

/// HSWorldDesc of HairSceneReference.cpp
const World = extern struct {
    gravity: [3]f32 = .{ 0, -9.81, 0 },
    num_bodies: u32 = 0,
    optimize_broad_phase: u32 = 0,
    padding: u32 = 0,
};

/// The shapes of the bodies (HSShape of HairSceneReference.cpp)
const ShapeKind = enum(u32) {
    /// ConvexHullShape of the points, max convex radius params[0]
    hull,
    /// ScaledShape of the hull, scale params[0..2]
    scaled_hull,
    /// StaticCompoundShape: the hull at params[0..2] with rotation params[3..6], the hull scaled by params[7..9] at -params[0..2], a sphere of radius params[10]
    static_compound,
    /// MutableCompoundShape: the hull at params[0..2], the hull scaled by params[3..5] at -params[0..2]
    mutable_compound,
    /// RotatedTranslatedShape of the hull: offset params[0..2], rotation params[3..6]
    rotated_translated_hull,
    /// BoxShape: half extent params[0..2]
    box,
    /// SphereShape: radius params[0]
    sphere,
    /// CapsuleShape: half height params[0], radius params[1]
    capsule,
};

/// HSBodyDesc of HairSceneReference.cpp
const Body = extern struct {
    position: [3]f64,
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    linear_velocity: [3]f32 = .{ 0, 0, 0 },
    angular_velocity: [3]f32 = .{ 0, 0, 0 },
    params: [12]f32 = @splat(0),
    shape: ShapeKind,
    motion_type: u32 = @intFromEnum(zolt.MotionType.static),
    is_sensor: u32 = 0,
    object_layer: u32 = Layers.non_moving,
    /// The hull points: the index of the first one in the point pool (3 floats per point)
    first_point: u32 = 0,
    num_points: u32 = 0,
};

/// HSHairDesc of HairSceneReference.cpp
const HairDesc = extern struct {
    position: [3]f64 = .{ 0, 0, 0 },
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    object_layer: u32 = Layers.moving,
    padding: u32 = 0,
};

/// HSOpKind of HairSceneReference.cpp
const OpKind = enum(u32) {
    /// Hair.update(f[0]): joint to hair the 16 floats at pool index u[1], joint matrices the u[2] matrices at pool index u[0] (no_joints: none)
    update,
    /// Hair.setPosition(d)
    set_position,
    /// Hair.setRotation(f[0..3])
    set_rotation,
    /// Hair.onTeleported()
    on_teleported,
    /// body u[0]: BodyInterface.setPositionRotationAndVelocity(d, f[0..3], f[4..6], f[7..9])
    move_body,
};

const no_joints: u32 = 0xFFFFFFFF;

/// HSOp of HairSceneReference.cpp
const Op = extern struct {
    kind: OpKind,
    u: [3]u32 = .{ 0, 0, 0 },
    d: [3]f64 = .{ 0, 0, 0 },
    f: [12]f32 = @splat(0),
};

// Tags of the transcript (HairSceneReference.cpp)
const tag_op: u32 = 0x45000000;
const tag_member: u32 = 0x45100000;
const tag_buffer: u32 = 0x45200000;
const tag_read: u32 = 0x45300000;
const after_init: u32 = 0xFFFFF;
const null_size: u32 = 0xFFFFFFFF;

const member_names = [_][]const u8{ "teleported", "previous position", "previous rotation" };
const buffer_names = [_][]const u8{
    "scalp joint matrices", "scalp vertices", "scalp triangles", "target positions",     "target global pose transforms", "global pose transforms",
    "shape planes",         "shape vertices", "shape indices",   "collision planes",     "collision shapes",              "materials",
    "previous positions",   "positions",      "velocities",      "velocity and density", "constants",                     "render positions",
};
const read_names = [_][]const u8{ "read back positions", "read back rotations", "read back velocities", "read back angular velocities", "read back grid", "read back render positions", "read back scalp vertices" };

// The scene ------------------------------------------------------------------------------------------------------------

/// A scene: the world, the bodies, the settings, the hair and the operations (the inputs of jolt_hs_run)
const Scene = struct {
    name: []const u8,
    world: World = .{},
    bodies: std.ArrayList(Body) = .empty,
    points: std.ArrayList(f32) = .empty,
    settings: []u8 = &.{},
    hair: HairDesc = .{},
    ops: std.ArrayList(Op) = .empty,
    pool: std.ArrayList(f32) = .empty,
    /// The pool index of an identity matrix (the joint to hair matrix of an update without joints)
    identity: u32 = 0,

    fn init(allocator: Allocator, name: []const u8) !Scene {
        var scene: Scene = .{ .name = name };
        errdefer scene.deinit(allocator);
        scene.identity = try scene.addMatrix(allocator, Mat44.identity());
        return scene;
    }

    fn deinit(self: *Scene, allocator: Allocator) void {
        self.bodies.deinit(allocator);
        self.points.deinit(allocator);
        allocator.free(self.settings);
        self.ops.deinit(allocator);
        self.pool.deinit(allocator);
    }

    /// Add a matrix to the pool, returns its index
    fn addMatrix(self: *Scene, allocator: Allocator, m: Mat44) !u32 {
        const index: u32 = @intCast(self.pool.items.len);
        var columns: [4]Float4 = undefined;
        m.storeFloat4x4(&columns);
        try self.pool.appendSlice(allocator, @ptrCast(&columns));
        return index;
    }

    /// Add the points of a hull: a jittered ellipsoid of `num_points` points with radii `radii`
    fn addHull(self: *Scene, allocator: Allocator, rng: *fw.Rng, radii: Vec3, num_points: u32) !struct { u32, u32 } {
        const first: u32 = @intCast(self.points.items.len / 3);
        for (0..num_points) |_| {
            const dir = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1)).normalizedOr(Vec3.axisX());
            const p = dir.mul(radii).mulScalar(rng.float(0.8, 1.0));
            try self.points.appendSlice(allocator, &.{ p.getX(), p.getY(), p.getZ() });
        }
        return .{ first, num_points };
    }

    /// Add a body, returns its index
    fn addBody(self: *Scene, allocator: Allocator, body: Body) !u32 {
        try self.bodies.append(allocator, body);
        self.world.num_bodies += 1;
        return @intCast(self.bodies.items.len - 1);
    }

    /// Add a body with a hull
    fn addHullBody(self: *Scene, allocator: Allocator, rng: *fw.Rng, radii: Vec3, body: Body) !u32 {
        var b = body;
        const hull = try self.addHull(allocator, rng, radii, 6 + rng.next() % 14);
        b.first_point = hull[0];
        b.num_points = hull[1];
        return self.addBody(allocator, b);
    }

    fn op(self: *Scene, allocator: Allocator, o: Op) !void {
        try self.ops.append(allocator, o);
    }

    /// An update without joint matrices
    fn update(self: *Scene, allocator: Allocator, delta_time: f32) !void {
        try self.op(allocator, .{ .kind = .update, .u = .{ no_joints, self.identity, 0 }, .f = .{delta_time} ++ @as([11]f32, @splat(0)) });
    }

    /// An update with the joint matrices of a frame of the face animation; the hair follows the neck joint like Jolt's
    /// HairTest sample (SetPosition, SetRotation, then Update with the inverse of the neck joint)
    fn updateWithFrame(self: *Scene, allocator: Allocator, face: *const Face, frame: []const Mat44, delta_time: f32, follow: bool) !void {
        const neck = frame[face.head_joint];
        if (follow) {
            try self.setPosition(allocator, RVec3.fromVec3(neck.getTranslation()));
            try self.setRotation(allocator, neck.getQuaternion());
        }
        const joints: u32 = @intCast(self.pool.items.len);
        for (frame) |m| _ = try self.addMatrix(allocator, m);
        const joint_to_hair = try self.addMatrix(allocator, neck.inversed());
        try self.op(allocator, .{ .kind = .update, .u = .{ joints, joint_to_hair, @intCast(frame.len) }, .f = .{delta_time} ++ @as([11]f32, @splat(0)) });
    }

    fn setPosition(self: *Scene, allocator: Allocator, position: RVec3) !void {
        try self.op(allocator, .{ .kind = .set_position, .d = .{ position.getX(), position.getY(), position.getZ() } });
    }

    fn setRotation(self: *Scene, allocator: Allocator, rotation: Quat) !void {
        try self.op(allocator, .{ .kind = .set_rotation, .f = .{ rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW() } ++ @as([8]f32, @splat(0)) });
    }

    fn moveBody(self: *Scene, allocator: Allocator, body: u32, position: RVec3, rotation: Quat, linear_velocity: Vec3, angular_velocity: Vec3) !void {
        try self.op(allocator, .{ .kind = .move_body, .u = .{ body, 0, 0 }, .d = .{ position.getX(), position.getY(), position.getZ() }, .f = .{
            rotation.getX(),         rotation.getY(),         rotation.getZ(),        rotation.getW(),
            linear_velocity.getX(),  linear_velocity.getY(),  linear_velocity.getZ(), angular_velocity.getX(),
            angular_velocity.getY(), angular_velocity.getZ(), 0,                      0,
        } });
    }
};

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn loadR3(v: *const [3]f64) RVec3 {
    return RVec3.init(@floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]));
}

fn loadQuat(v: []const f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

fn loadMat44(m: []const f32) Mat44 {
    return Mat44.init(Vec4.init(m[0], m[1], m[2], m[3]), Vec4.init(m[4], m[5], m[6], m[7]), Vec4.init(m[8], m[9], m[10], m[11]), Vec4.init(m[12], m[13], m[14], m[15]));
}

/// HSMakeShapeSettings of HairSceneReference.cpp: the returned settings have no reference yet
fn makeShapeSettings(allocator: Allocator, desc: *const Body, points: []const f32) !*ShapeSettings {
    const p = &desc.params;
    const Hull = struct {
        fn make(a: Allocator, d: *const Body, pool: []const f32, max_convex_radius: f32) !*ShapeSettings {
            var list: std.ArrayList(Vec3) = .empty;
            defer list.deinit(a);
            for (0..d.num_points) |i|
                try list.append(a, load3(pool[3 * (d.first_point + i) ..][0..3]));
            return (try zolt.ConvexHullShapeSettings.create(a, list.items, .{ .max_convex_radius = max_convex_radius })).asShapeSettings();
        }
    };
    return switch (desc.shape) {
        .hull => try Hull.make(allocator, desc, points, p[0]),
        .scaled_hull => (try zolt.ScaledShapeSettings.create(allocator, try Hull.make(allocator, desc, points, 0.0), load3(p[0..3]))).asShapeSettings(),
        .static_compound => blk: {
            const compound = try zolt.StaticCompoundShapeSettings.create(allocator);
            try compound.base.addShape(load3(p[0..3]), loadQuat(p[3..7]), try Hull.make(allocator, desc, points, 0.0), .{});
            try compound.base.addShape(load3(p[0..3]).negate(), Quat.identity(), (try zolt.ScaledShapeSettings.create(allocator, try Hull.make(allocator, desc, points, 0.0), load3(p[7..10]))).asShapeSettings(), .{});
            try compound.base.addShape(Vec3.zero(), Quat.identity(), (try zolt.SphereShapeSettings.create(allocator, p[10], .{})).asShapeSettings(), .{});
            break :blk compound.asShapeSettings();
        },
        .mutable_compound => blk: {
            const compound = try zolt.MutableCompoundShapeSettings.create(allocator);
            try compound.base.addShape(load3(p[0..3]), Quat.identity(), try Hull.make(allocator, desc, points, 0.0), .{});
            try compound.base.addShape(load3(p[0..3]).negate(), Quat.identity(), (try zolt.ScaledShapeSettings.create(allocator, try Hull.make(allocator, desc, points, 0.0), load3(p[3..6]))).asShapeSettings(), .{});
            break :blk compound.asShapeSettings();
        },
        .rotated_translated_hull => (try zolt.RotatedTranslatedShapeSettings.create(allocator, load3(p[0..3]), loadQuat(p[3..7]), try Hull.make(allocator, desc, points, 0.0))).asShapeSettings(),
        .box => (try zolt.BoxShapeSettings.create(allocator, load3(p[0..3]), .{})).asShapeSettings(),
        .sphere => (try zolt.SphereShapeSettings.create(allocator, p[0], .{})).asShapeSettings(),
        .capsule => (try zolt.CapsuleShapeSettings.create(allocator, p[0], p[1], .{})).asShapeSettings(),
    };
}

// The transcript -------------------------------------------------------------------------------------------------------

/// The transcript of the Zolt side (HSTranscript of HairSceneReference.cpp)
const Transcript = struct {
    allocator: Allocator,
    data: std.ArrayList(u8) = .empty,

    fn deinit(self: *Transcript) void {
        self.data.deinit(self.allocator);
    }

    fn u32v(self: *Transcript, value: u32) !void {
        try self.data.appendSlice(self.allocator, std.mem.asBytes(&value));
    }

    fn record(self: *Transcript, tag: u32, bytes: []const u8) !void {
        try self.u32v(tag);
        try self.u32v(@intCast(bytes.len));
        try self.data.appendSlice(self.allocator, bytes);
    }

    fn nullRecord(self: *Transcript, tag: u32) !void {
        try self.u32v(tag);
        try self.u32v(null_size);
    }

    fn buffer(self: *Transcript, tag: u32, b: ?*ComputeBuffer) !void {
        const buf = b orelse return self.nullRecord(tag);
        const bytes = buf.map(.read);
        defer buf.unmap();
        try self.record(tag, bytes);
    }
};

/// HSHair::Dump of HairSceneReference.cpp
fn dumpHair(t: *Transcript, hair: *const Hair) !void {
    const teleported: u32 = @intFromBool(hair.teleported);
    try t.record(tag_member | 0, std.mem.asBytes(&teleported));
    const prev_position = [3]Real{ hair.prev_position.getX(), hair.prev_position.getY(), hair.prev_position.getZ() };
    try t.record(tag_member | 1, std.mem.asBytes(&prev_position));
    const prev_rotation = [4]f32{ hair.prev_rotation.getX(), hair.prev_rotation.getY(), hair.prev_rotation.getZ(), hair.prev_rotation.getW() };
    try t.record(tag_member | 2, std.mem.asBytes(&prev_rotation));

    const buffers = [_]?*ComputeBuffer{
        hair.scalp_joint_matrices_cb.get(),
        hair.scalp_vertices_cb.get(),
        hair.scalp_triangles_cb.get(),
        hair.target_positions_cb.get(),
        hair.target_global_pose_transforms_cb.get(),
        hair.global_pose_transforms_cb.get(),
        hair.shape_planes_cb.get(),
        hair.shape_vertices_cb.get(),
        hair.shape_indices_cb.get(),
        hair.collision_planes_cb.get(),
        hair.collision_shapes_cb.get(),
        hair.materials_cb.get(),
        hair.previous_positions_cb.get(),
        hair.positions_cb.get(),
        hair.velocities_cb.get(),
        hair.velocity_and_density_cb.get(),
        hair.constants_cb.get(),
        hair.render_positions_cb.get(),
    };
    comptime std.debug.assert(buffers.len == buffer_names.len);
    for (buffers, 0..) |b, i|
        try t.buffer(tag_buffer | @as(u32, @intCast(i)), b);
    const num_iteration_buffers: u32 = @intCast(hair.iteration_constants_cb.items.len);
    try t.record(tag_buffer | 100, std.mem.asBytes(&num_iteration_buffers));
    for (hair.iteration_constants_cb.items, 0..) |b, i|
        try t.buffer(tag_buffer | (101 + @as(u32, @intCast(i))), b.get());
}

/// HSHair::DumpReadBack of HairSceneReference.cpp
fn dumpReadBack(t: *Transcript, hair: *Hair, queue: *ComputeQueue) !void {
    const allocator = t.allocator;
    try hair.readBackGPUState(queue);
    try hair.lockReadBackBuffers();
    defer hair.unlockReadBackBuffers();
    const settings = hair.getHairSettings();

    const num_vertices = settings.sim_vertices.items.len;
    for (hair.getPositions().?) |p| {
        // The scenes keep the strands finite, so no grid position is NaN (R20 UB11)
        if (std.math.isNan(p.x) or std.math.isNan(p.y) or std.math.isNan(p.z)) return error.NaNPosition;
    }
    try t.record(tag_read | 0, std.mem.sliceAsBytes(hair.getPositions().?));
    var floats: std.ArrayList(f32) = .empty;
    defer floats.deinit(allocator);
    for (hair.getRotations().?) |q|
        try floats.appendSlice(allocator, &.{ q.getX(), q.getY(), q.getZ(), q.getW() });
    try t.record(tag_read | 1, std.mem.sliceAsBytes(floats.items));
    var velocities: std.ArrayList(Float3) = .empty;
    defer velocities.deinit(allocator);
    var angular_velocities: std.ArrayList(Float3) = .empty;
    defer angular_velocities.deinit(allocator);
    const v_ptr = hair.getVelocities();
    const w_ptr = hair.getAngularVelocities();
    for (0..num_vertices) |v| {
        try velocities.append(allocator, v_ptr.at(@intCast(v)).*);
        try angular_velocities.append(allocator, w_ptr.at(@intCast(v)).*);
    }
    try t.record(tag_read | 2, std.mem.sliceAsBytes(velocities.items));
    try t.record(tag_read | 3, std.mem.sliceAsBytes(angular_velocities.items));
    try t.record(tag_read | 4, std.mem.sliceAsBytes(hair.getGridVelocityAndDensity().?));
    try t.record(tag_read | 5, std.mem.sliceAsBytes(hair.getRenderPositions().?));
    if (hair.getScalpVertices()) |s|
        try t.record(tag_read | 6, std.mem.sliceAsBytes(s))
    else
        try t.nullRecord(tag_read | 6);
}

/// jolt_hs_run of HairSceneReference.cpp on Zolt
fn zoltRun(allocator: Allocator, scene: *const Scene, t: *Transcript, coverage: *Coverage) !void {
    // The settings
    const settings_ptr = try HairSettings.create(allocator);
    var settings: RefConst(HairSettings) = .init(settings_ptr);
    defer settings.deinit();
    {
        var reader: std.Io.Reader = .fixed(scene.settings);
        var in = zolt.StreamInWrapper.init(&reader);
        try settings_ptr.restoreBinaryState(in.streamIn());
    }

    // The compute system, the shaders and the queue
    var compute_system = try ComputeSystemCPU.create(allocator);
    defer compute_system.deinit();
    const system = compute_system.get().?;
    try zolt.registerHairShaders(zolt.virtual.downcast(ComputeSystemCPU, system));
    var shaders: Ref(HairShaders) = .init(try HairShaders.create(allocator));
    defer shaders.deinit();
    try shaders.get().?.init(system);
    var queue = try system.createComputeQueue();
    defer queue.deinit();
    try settings_ptr.initCompute(system);

    // The world (the layers of UnitTests/Layers.h)
    var layers: TestLayers.BPLayerInterfaceImpl = .init();
    var object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{};
    var pairs: TestLayers.ObjectLayerPairFilterImpl = .{};
    var physics_system: PhysicsSystem = .empty;
    try physics_system.init(allocator, io, 256, 0, 256, 256, &layers.base, &object_vs_broad_phase.base, &pairs.base, .{});
    defer physics_system.deinit();
    physics_system.setGravity(load3(&scene.world.gravity));
    const bi = physics_system.getBodyInterfaceMut();

    // The bodies
    var bodies: std.ArrayList(BodyID) = .empty;
    defer bodies.deinit(allocator);
    defer for (bodies.items) |id| {
        bi.removeBody(id);
        bi.destroyBody(id);
    };
    for (scene.bodies.items) |*d| {
        var s: BodyCreationSettings = .initWithSettings(try makeShapeSettings(allocator, d, scene.points.items), loadR3(&d.position), loadQuat(&d.rotation), @enumFromInt(d.motion_type), @intCast(d.object_layer));
        defer s.deinit();
        s.linear_velocity = load3(&d.linear_velocity);
        s.angular_velocity = load3(&d.angular_velocity);
        s.is_sensor = d.is_sensor != 0;
        const body = (try bi.createBody(&s)) orelse return error.NoBody;
        try bi.addBody(body.getID(), if (d.motion_type == @intFromEnum(zolt.MotionType.static)) .dont_activate else .activate);
        try bodies.append(allocator, body.getID());
    }
    if (scene.world.optimize_broad_phase != 0)
        try physics_system.optimizeBroadPhase();

    // The hair
    var hair: Hair = .init(allocator, settings.get().?, loadR3(&scene.hair.position), loadQuat(&scene.hair.rotation), @intCast(scene.hair.object_layer));
    defer hair.deinit();
    try hair.initCompute(system);
    try t.record(tag_op | after_init, &.{});
    try dumpHair(t, &hair);

    // The operations
    const pool = scene.pool.items;
    for (scene.ops.items, 0..) |*o, op_index| {
        switch (o.kind) {
            .update => {
                var joints: std.ArrayList(Mat44) = .empty;
                defer joints.deinit(allocator);
                if (o.u[0] != no_joints) {
                    for (0..o.u[2]) |j|
                        try joints.append(allocator, loadMat44(pool[o.u[0] + 16 * j ..][0..16]));
                }
                try hair.update(o.f[0], loadMat44(pool[o.u[1]..][0..16]), if (o.u[0] != no_joints) joints.items else null, &physics_system, shaders.get().?, system, queue.get().?);
                try checkScene(scene, &hair, coverage);
            },
            .set_position => hair.setPosition(loadR3(&o.d)),
            .set_rotation => hair.setRotation(loadQuat(&o.f)),
            .on_teleported => hair.onTeleported(),
            .move_body => bi.setPositionRotationAndVelocity(bodies.items[o.u[0]], loadR3(&o.d), loadQuat(o.f[0..4]), load3(o.f[4..7]), load3(o.f[7..10])),
        }

        try t.record(tag_op | @as(u32, @intCast(op_index)), &.{});
        try dumpHair(t, &hair);
        if (o.kind == .update)
            try dumpReadBack(t, &hair, queue.get().?);
    }
}

/// The name of a record tag
fn tagName(buffer: []u8, tag: u32) []const u8 {
    const index = tag & 0xFFFFF;
    return switch (tag & 0xFFF00000) {
        tag_op => "operation",
        tag_member => if (index < member_names.len) member_names[index] else "member",
        tag_buffer => if (index < buffer_names.len) buffer_names[index] else if (index == 100) "number of iteration constant buffers" else std.fmt.bufPrint(buffer, "iteration constants {d}", .{index - 101}) catch "iteration constants",
        tag_read => if (index < read_names.len) read_names[index] else "read back",
        else => "unknown",
    };
}

/// Compare the transcripts record by record, report the first difference
fn compareTranscripts(scene: *const Scene, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    var z: usize = 0;
    var j: usize = 0;
    var op: u32 = after_init;
    var name_buffer: [64]u8 = undefined;
    while (z < zolt_bytes.len and j < jolt_bytes.len) {
        if (z + 8 > zolt_bytes.len or j + 8 > jolt_bytes.len) break;
        const z_tag = std.mem.readInt(u32, zolt_bytes[z..][0..4], .little);
        const j_tag = std.mem.readInt(u32, jolt_bytes[j..][0..4], .little);
        const z_size = std.mem.readInt(u32, zolt_bytes[z + 4 ..][0..4], .little);
        const j_size = std.mem.readInt(u32, jolt_bytes[j + 4 ..][0..4], .little);
        if (z_tag != j_tag) {
            std.debug.print("Hair scene {s}, operation {x}: record {s} (Zolt) where Jolt has {s}\n", .{ scene.name, op, tagName(&name_buffer, z_tag), tagName(&name_buffer, j_tag) });
            return error.ParityMismatch;
        }
        if (z_tag & 0xFFF00000 == tag_op) op = z_tag & 0xFFFFF;
        if (z_size != j_size) {
            std.debug.print("Hair scene {s}, operation {x}: {s} has {d} bytes in Zolt, {d} in Jolt (0x{x}: a null buffer)\n", .{ scene.name, op, tagName(&name_buffer, z_tag), z_size, j_size, null_size });
            return error.ParityMismatch;
        }
        z += 8;
        j += 8;
        if (z_size == null_size) continue;
        const z_data = zolt_bytes[z..][0..z_size];
        const j_data = jolt_bytes[j..][0..j_size];
        if (!std.mem.eql(u8, z_data, j_data)) {
            const at = std.mem.indexOfDiff(u8, z_data, j_data).?;
            const word = at & ~@as(usize, 3);
            const z_word = std.mem.readInt(u32, z_data[word..][0..4], .little);
            const j_word = std.mem.readInt(u32, j_data[word..][0..4], .little);
            std.debug.print("Hair scene {s}, operation {x}: {s} differs at byte {d} of {d}: Zolt 0x{x:0>8} ({e}), Jolt 0x{x:0>8} ({e})\n", .{ scene.name, op, tagName(&name_buffer, z_tag), at, z_size, z_word, @as(f32, @bitCast(z_word)), j_word, @as(f32, @bitCast(j_word)) });
            return error.ParityMismatch;
        }
        z += z_size;
        j += j_size;
    }
    if (z != zolt_bytes.len or j != jolt_bytes.len) {
        std.debug.print("Hair scene {s}: the transcripts have different lengths (Zolt {d} bytes, Jolt {d})\n", .{ scene.name, zolt_bytes.len, jolt_bytes.len });
        return error.ParityMismatch;
    }
}

/// Coverage of the hard paths, counted on the Zolt side (equal to Jolt's when the transcripts are equal)
const Coverage = struct {
    scenes: u32 = 0,
    /// Updates with collision against at least one hull
    collision_updates: u32 = 0,
    /// Updates that ran the grid
    grid_updates: u32 = 0,
    /// Updates of a global pose only hair
    global_pose_only_updates: u32 = 0,
    /// Updates that skinned the scalp
    skinned_updates: u32 = 0,
    /// Updates that interpolated the roots to the skinned targets
    interpolated_updates: u32 = 0,
};

/// After an update on the Zolt side: count the paths, and check that the scene stays away from Jolt's undefined
/// behavior (R20). UB3: when a material collides, the collision buffers were written in this update and hold at least
/// one hull (the first word of the shape indices is the number of faces of the first hull, 0 is the terminator).
fn checkScene(scene: *const Scene, hair: *Hair, coverage: *Coverage) !void {
    const materials = hair.materials_cb.get().?.mapAs(HairStructs.HairMaterial, .read);
    defer hair.materials_cb.get().?.unmap();
    var collides = false;
    var grid = false;
    var global_pose_only = true;
    for (materials, hair.getHairSettings().materials.items) |m, *m_in| {
        collides = collides or m.enable_collision != 0;
        grid = grid or m_in.needsGrid();
        global_pose_only = global_pose_only and m_in.globalPoseOnly();
    }
    if (collides) {
        const indices = hair.shape_indices_cb.get().?.mapAs(u32, .read);
        defer hair.shape_indices_cb.get().?.unmap();
        if (indices[0] == 0) {
            std.debug.print("Hair scene {s}: an update collides without a hull in the bounds (R20 UB3)\n", .{scene.name});
            return error.ParityMismatch;
        }
        coverage.collision_updates += 1;
    }
    if (grid and !global_pose_only) coverage.grid_updates += 1;
    if (global_pose_only) coverage.global_pose_only_updates += 1;
    if (hair.scalp_joint_matrices_cb.get() != null) coverage.skinned_updates += 1;
    if (hair.target_positions_cb.get() != null) coverage.interpolated_updates += 1;
}

/// Run a scene on both sides and compare the transcripts
fn runScene(allocator: Allocator, scene: *const Scene, coverage: *Coverage) !void {
    const capacity: u32 = 256 << 20;
    const jolt_buffer = try allocator.alloc(u8, capacity);
    defer allocator.free(jolt_buffer);
    const jolt_size = jolt.jolt_hs_run(&scene.world, scene.bodies.items.ptr, scene.points.items.ptr, scene.settings.ptr, @intCast(scene.settings.len), &scene.hair, scene.ops.items.ptr, @intCast(scene.ops.items.len), scene.pool.items.ptr, jolt_buffer.ptr, capacity);
    if (jolt_size == null_size) {
        std.debug.print("Hair scene {s}: the C++ side failed (transcript overflow, settings or a body)\n", .{scene.name});
        return error.ParityMismatch;
    }

    var t: Transcript = .{ .allocator = allocator };
    defer t.deinit();
    try zoltRun(allocator, scene, &t, coverage);
    try compareTranscripts(scene, t.data.items, jolt_buffer[0..jolt_size]);

    coverage.scenes += 1;
}

// The scenes -----------------------------------------------------------------------------------------------------------

/// The SaveBinaryState bytes of settings after InitRenderAndSimulationStrands and Init
fn settingsBytes(allocator: Allocator, face: ?*const Face, materials: []const Material, groom: *const Groom, opts: struct { grid_size: UVec4 = .init(12, 14, 10, 0), num_iterations_per_second: u32 = 360, max_delta_time: f32 = 1.0 / 30.0 }) ![]u8 {
    var settings = try HairSettingsParity.baseSettings(allocator, face, materials);
    defer settings.deinit();
    settings.grid_size = opts.grid_size;
    settings.num_iterations_per_second = opts.num_iterations_per_second;
    settings.max_delta_time = opts.max_delta_time;
    try settings.initRenderAndSimulationStrands(groom.vertices.items, groom.strands.items);
    _ = try settings.init();
    return HairSettingsParity.saveState(allocator, &settings);
}

/// The bodies around a hair at `center`: a static hull floor that always overlaps the bounds of the hair (so a scene
/// with collision always has a hull: R20 UB3), hulls of every kind and the bodies that Hair ignores. Returns the index
/// of the first kinematic body (the compound that the scenes move).
fn addBodies(allocator: Allocator, scene: *Scene, rng: *fw.Rng, center: Vec3) !u32 {
    const c = center;
    const at = struct {
        fn f(base: Vec3, x: f32, y: f32, z: f32) [3]f64 {
            return .{ base.getX() + x, base.getY() + y, base.getZ() + z };
        }
    }.f;
    const rot = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.7);
    const rotation = [4]f32{ rot.getX(), rot.getY(), rot.getZ(), rot.getW() };

    // The floor: a static hull below the hair
    _ = try scene.addHullBody(allocator, rng, Vec3.init(1.5, 0.08, 1.5), .{ .position = at(c, 0, -0.25, 0), .shape = .hull });

    // A kinematic static compound with velocities, a kinematic mutable compound
    const compound = try scene.addHullBody(allocator, rng, Vec3.init(0.06, 0.05, 0.07), .{
        .position = at(c, 0.15, -0.2, 0.05),
        .rotation = rotation,
        .linear_velocity = .{ 0.3, -0.1, 0.2 },
        .angular_velocity = .{ 0.5, 1.0, -0.4 },
        .params = .{ 0.06, 0.02, -0.03, rot.getX(), rot.getY(), rot.getZ(), rot.getW(), 1.5, 0.5, 1.2, 0.03, 0 },
        .shape = .static_compound,
        .motion_type = @intFromEnum(zolt.MotionType.kinematic),
        .object_layer = Layers.moving,
    });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.05, 0.05, 0.05), .{
        .position = at(c, -0.15, -0.25, -0.1),
        .linear_velocity = .{ -0.2, 0.0, 0.1 },
        .angular_velocity = .{ 0.0, 2.0, 0.0 },
        .params = .{ 0.05, 0.0, 0.04, 0.7, 1.3, 0.9, 0, 0, 0, 0, 0, 0 },
        .shape = .mutable_compound,
        .motion_type = @intFromEnum(zolt.MotionType.kinematic),
        .object_layer = Layers.moving,
    });

    // A scaled hull, inside out hulls (one and three negative components), a rotated and translated hull, a dynamic hull
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.05, 0.08, 0.05), .{ .position = at(c, 0.0, -0.3, 0.2), .params = .{ 1.2, 0.6, 1.8, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, .shape = .scaled_hull });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.06, 0.04, 0.05), .{ .position = at(c, 0.05, -0.15, -0.2), .rotation = rotation, .params = .{ -1.0, 1.3, 0.8, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, .shape = .scaled_hull });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.04, 0.04, 0.06), .{
        .position = at(c, -0.2, -0.1, 0.15),
        .angular_velocity = .{ 0.3, 0.0, 0.0 },
        .params = .{ -0.8, -1.1, -0.9, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        .shape = .scaled_hull,
        .motion_type = @intFromEnum(zolt.MotionType.kinematic),
        .object_layer = Layers.moving,
    });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.05, 0.03, 0.05), .{ .position = at(c, 0.2, -0.05, -0.15), .params = .{ 0.02, -0.03, 0.01, rot.getX(), rot.getY(), rot.getZ(), rot.getW(), 0, 0, 0, 0, 0 }, .shape = .rotated_translated_hull });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.04, 0.04, 0.04), .{
        .position = at(c, -0.05, -0.35, 0.25),
        .linear_velocity = .{ 0.0, -0.5, 0.0 },
        .angular_velocity = .{ 1.0, 0.0, 0.5 },
        .params = .{ 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        .shape = .hull,
        .motion_type = @intFromEnum(zolt.MotionType.dynamic),
        .object_layer = Layers.moving,
    });

    // Bodies that Hair ignores: a box, a sphere and a capsule (not hulls), a sensor hull, a hull in a layer that does not
    // collide with the hair and a hull outside the bounds
    _ = try scene.addBody(allocator, .{ .position = at(c, 0.1, -0.1, 0.1), .params = .{ 0.06, 0.07, 0.08, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, .shape = .box });
    _ = try scene.addBody(allocator, .{ .position = at(c, -0.1, -0.2, 0.0), .params = .{ 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, .shape = .sphere, .motion_type = @intFromEnum(zolt.MotionType.kinematic), .object_layer = Layers.moving });
    _ = try scene.addBody(allocator, .{ .position = at(c, 0.0, -0.05, -0.1), .params = .{ 0.05, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, .shape = .capsule });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.1, 0.1, 0.1), .{ .position = at(c, 0.0, -0.2, 0.0), .shape = .hull, .is_sensor = 1, .motion_type = @intFromEnum(zolt.MotionType.kinematic), .object_layer = Layers.moving });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.1, 0.1, 0.1), .{ .position = at(c, 0.0, -0.15, 0.0), .shape = .hull, .motion_type = @intFromEnum(zolt.MotionType.kinematic), .object_layer = Layers.moving2 });
    _ = try scene.addHullBody(allocator, rng, Vec3.init(0.1, 0.1, 0.1), .{ .position = at(c, 5.0, 0.0, 0.0), .shape = .hull });
    return compound;
}

/// A material with different values than the defaults (`seed` picks the variant)
fn variantMaterial(seed: u32) Material {
    var m: Material = .{};
    switch (seed % 4) {
        0 => {
            // Grid off, LRA off
            m.grid_velocity_factor = .init(0.0, 0.0, .{});
            m.grid_density_force_factor = 0.0;
            m.enable_lra = false;
        },
        1 => {
            // The grid through the density force only, damping, friction, velocity limits
            m.grid_velocity_factor = .init(0.0, 0.0, .{});
            m.grid_density_force_factor = 0.3;
            m.linear_damping = 5.0;
            m.angular_damping = 0.5;
            m.friction = 1.0;
            m.max_linear_velocity = 2.0;
            m.max_angular_velocity = 10.0;
        },
        2 => {
            // Gradients along the strand
            m.gravity_factor = .init(0.2, 1.0, .{ .min_fraction = 0.1, .max_fraction = 0.6 });
            m.hair_radius = .init(0.004, 0.001, .{ .min_fraction = 0.0, .max_fraction = 0.8 });
            m.world_transform_influence = .init(0.3, 0.9, .{ .min_fraction = 0.1, .max_fraction = 0.9 });
            m.grid_velocity_factor = .init(0.1, 0.02, .{ .min_fraction = 0.2, .max_fraction = 0.7 });
            m.global_pose = .init(0.05, 0.001, .{ .min_fraction = 0.0, .max_fraction = 0.5 });
            m.skin_global_pose = .init(0.8, 0.1, .{ .min_fraction = 0.0, .max_fraction = 0.3 });
            m.inertia_multiplier = 3.0;
        },
        else => {
            // Stiff and soft compliances
            m.bend_compliance = 1.0e-4;
            m.bend_compliance_multiplier = .{ .x = 2.0, .y = 20.0, .z = 5.0, .w = 0.5 };
            m.stretch_compliance = 1.0e-6;
            m.linear_damping = 0.0;
            m.angular_damping = 0.0;
        },
    }
    return m;
}

/// A global pose only material: no collision and a global pose of 1 everywhere
fn globalPoseOnlyMaterial(fraction: f32) Material {
    return .{ .enable_collision = false, .global_pose = .init(1.0, 1.0, .{}), .simulation_strands_fraction = fraction };
}

/// The operations of a scene without a scalp: delta times, moves of the hair and of a body, a teleport
fn addOps(allocator: Allocator, scene: *Scene, center: Vec3, body: u32) !void {
    const c = RVec3.fromVec3(center);
    try scene.update(allocator, 1.0 / 60.0); // The first update teleports
    try scene.update(allocator, 1.0 / 60.0);
    try scene.moveBody(allocator, body, c.addVec3(Vec3.init(0.12, -0.18, 0.04)), Quat.rotation(Vec3.axisY(), 0.3), Vec3.init(-0.4, 0.2, 0.0), Vec3.init(0.0, -1.5, 0.3));
    try scene.update(allocator, 1.0 / 90.0);
    try scene.setPosition(allocator, c.addVec3(Vec3.init(0.02, 0.01, -0.015)));
    try scene.update(allocator, 0.0123);
    try scene.setRotation(allocator, Quat.rotation(Vec3.init(0.3, 1.0, 0.2).normalized(), 0.15));
    try scene.update(allocator, 1.0 / 60.0);
    try scene.update(allocator, 0.0); // No iterations
    try scene.update(allocator, 0.1); // Above mMaxDeltaTime
    try scene.setPosition(allocator, c.addVec3(Vec3.init(-0.01, 0.02, 0.0)));
    try scene.setRotation(allocator, Quat.rotation(Vec3.init(-0.2, 1.0, 0.1).normalized(), -0.1));
    try scene.update(allocator, 0.0071);
    try scene.op(allocator, .{ .kind = .on_teleported });
    try scene.update(allocator, 1.0 / 60.0);
    try scene.update(allocator, 1.0 / 60.0);
}

fn checkStructSizes() !void {
    var sizes: [4]u32 = undefined;
    jolt.jolt_hs_struct_sizes(&sizes);
    try std.testing.expectEqual(@as(u32, @sizeOf(World)), sizes[0]);
    try std.testing.expectEqual(@as(u32, @sizeOf(Body)), sizes[1]);
    try std.testing.expectEqual(@as(u32, @sizeOf(HairDesc)), sizes[2]);
    try std.testing.expectEqual(@as(u32, @sizeOf(Op)), sizes[3]);
}

test "Hair scenes parity: grooms without a scalp among hulls" {
    const allocator = std.testing.allocator;
    try checkStructSizes();
    var rng: fw.Rng = .{ .state = 0x4a17_5ce1 };
    var coverage: Coverage = .{};

    const Setup = struct { name: []const u8, shape: HairSettingsParity.Shape, materials: []const Material, center: Vec3, rotation: Quat = .identity(), gravity: [3]f32 = .{ 0, -9.81, 0 }, optimize: bool = false, num_iterations_per_second: u32 = 360 };
    const m3 = materials3();
    const setups = [_]Setup{
        .{ .name = "straight, 3 materials", .shape = .straight, .materials = &m3, .center = Vec3.zero() },
        .{ .name = "curly, variants", .shape = .curly, .materials = &.{ withFraction(variantMaterial(0), 0.6), withFraction(variantMaterial(1), 0.4), withFraction(variantMaterial(2), 0.5), withFraction(variantMaterial(3), 0.7) }, .center = Vec3.init(1.0, 0.5, -2.0), .rotation = Quat.rotation(Vec3.axisZ(), 0.4), .optimize = true },
        .{ .name = "global pose only", .shape = .straight, .materials = &.{globalPoseOnlyMaterial(0.5)}, .center = Vec3.init(-3.0, 1.0, 2.0) },
        .{ .name = "global pose only and simulated", .shape = .curly, .materials = &.{ globalPoseOnlyMaterial(0.5), withFraction(variantMaterial(2), 0.5) }, .center = Vec3.zero() },
        .{ .name = "other gravity, 180 iterations", .shape = .curly, .materials = &.{ withFraction(variantMaterial(3), 0.8), withFraction(variantMaterial(1), 0.3) }, .center = Vec3.init(0.5, -1.0, 0.5), .gravity = .{ 2.0, -15.0, 1.0 }, .num_iterations_per_second = 180 },
        .{ .name = "no collision", .shape = .straight, .materials = &.{.{ .enable_collision = false, .simulation_strands_fraction = 0.5 }}, .center = Vec3.zero(), .rotation = Quat.rotation(Vec3.axisX(), -0.3) },
    };
    for (setups) |setup| {
        var scene = try Scene.init(allocator, setup.name);
        defer scene.deinit(allocator);
        scene.world.gravity = setup.gravity;
        scene.world.optimize_broad_phase = @intFromBool(setup.optimize);
        const c = setup.center;
        scene.hair.position = .{ c.getX(), c.getY(), c.getZ() };
        scene.hair.rotation = .{ setup.rotation.getX(), setup.rotation.getY(), setup.rotation.getZ(), setup.rotation.getW() };

        var groom = try HairSettingsParity.makeGroom(allocator, &rng, null, setup.shape, 40, 3, 16, .{ .random = @intCast(setup.materials.len) });
        defer groom.deinit(allocator);
        scene.settings = try settingsBytes(allocator, null, setup.materials, &groom, .{ .num_iterations_per_second = setup.num_iterations_per_second });
        const body = try addBodies(allocator, &scene, &rng, c);
        try addOps(allocator, &scene, c, body);
        try runScene(allocator, &scene, &coverage);
    }
    try std.testing.expectEqual(@as(u32, setups.len), coverage.scenes);
    // The paths of Update: collision against hulls, the grid, the global pose only path
    try std.testing.expect(coverage.collision_updates > 10 and coverage.grid_updates > 10 and coverage.global_pose_only_updates > 5);
}

/// The 3 materials of HairSettingsParity.materials3 (fractions below 1, a gravity preload, a global pose only material)
fn materials3() [3]Material {
    var m: [3]Material = .{ .{}, .{}, .{} };
    m[0].simulation_strands_fraction = 0.5;
    m[1].simulation_strands_fraction = 0.4;
    m[1].gravity_preload_factor = 1.0;
    m[1].bend_compliance = 1.0e-5;
    m[1].stretch_compliance = 1.0e-7;
    m[1].gravity_factor = .init(0.3, 0.9, .{ .min_fraction = 0.1, .max_fraction = 0.7 });
    m[2] = globalPoseOnlyMaterial(0.6);
    return m;
}

fn withFraction(material: Material, fraction: f32) Material {
    var m = material;
    m.simulation_strands_fraction = fraction;
    return m;
}

test "Hair scenes parity: grooms on the skinned scalp of Assets/face.bin" {
    const allocator = std.testing.allocator;
    var face = (try HairSettingsParity.loadFace(allocator)) orelse return error.SkipZigTest;
    defer face.deinit(allocator);
    var rng: fw.Rng = .{ .state = 0x6f1e_3a2d };
    var coverage: Coverage = .{};

    const Scalp = enum { skinned, no_inverse_bind_pose };
    const Setup = struct { name: []const u8, shape: HairSettingsParity.Shape, materials: []const Material, scalp: Scalp = .skinned, follow: bool = true };
    const m3 = materials3();
    const setups = [_]Setup{
        .{ .name = "face, 3 materials", .shape = .straight, .materials = &m3 },
        .{ .name = "face, curly variants", .shape = .curly, .materials = &.{ withFraction(variantMaterial(2), 0.4), withFraction(variantMaterial(0), 0.3) } },
        .{ .name = "face, global pose only", .shape = .curly, .materials = &.{globalPoseOnlyMaterial(0.5)} },
        .{ .name = "face, the hair stays", .shape = .straight, .materials = &.{ withFraction(variantMaterial(1), 0.5), withFraction(variantMaterial(3), 0.5) }, .follow = false },
        .{ .name = "face without inverse bind pose (no skinning)", .shape = .straight, .materials = &.{withFraction(variantMaterial(2), 0.5)}, .scalp = .no_inverse_bind_pose },
    };
    for (setups) |setup| {
        var scene = try Scene.init(allocator, setup.name);
        defer scene.deinit(allocator);

        // The hair starts at the neck joint of frame 0, like Jolt's HairTest sample
        const neck = face.frames[0][face.head_joint];
        const neck_position = neck.getTranslation();
        const neck_rotation = neck.getQuaternion();
        scene.hair.position = .{ neck_position.getX(), neck_position.getY(), neck_position.getZ() };
        scene.hair.rotation = .{ neck_rotation.getX(), neck_rotation.getY(), neck_rotation.getZ(), neck_rotation.getW() };

        var groom = try HairSettingsParity.makeGroom(allocator, &rng, &face, setup.shape, 60, 4, 14, .{ .random = @intCast(setup.materials.len) });
        defer groom.deinit(allocator);
        var settings = try HairSettingsParity.baseSettings(allocator, &face, setup.materials);
        defer settings.deinit();
        if (setup.scalp == .no_inverse_bind_pose) settings.scalp_inverse_bind_pose.clearRetainingCapacity();
        settings.grid_size = .init(10, 12, 9, 0);
        try settings.initRenderAndSimulationStrands(groom.vertices.items, groom.strands.items);
        _ = try settings.init();
        scene.settings = try HairSettingsParity.saveState(allocator, &settings);

        const body = try addBodies(allocator, &scene, &rng, neck_position);

        // The first update at frame 0 (Jolt's HairTest: Update(0.0f, inv_neck_transform, frame 0)), then the frames
        try scene.updateWithFrame(allocator, &face, face.frames[0], 0.0, false);
        const frames = [_]u32{ 1, 2, 1, 0, 2 };
        const delta_times = [_]f32{ 1.0 / 60.0, 1.0 / 90.0, 0.0123, 0.05, 1.0 / 60.0 };
        for (frames, delta_times, 0..) |f, dt, i| {
            if (i == 2)
                try scene.moveBody(allocator, body, RVec3.fromVec3(neck_position.add(Vec3.init(0.1, -0.15, 0.08))), Quat.rotation(Vec3.axisX(), 0.4), Vec3.init(0.1, 0.3, -0.2), Vec3.init(1.0, 0.0, 0.0));
            if (i == 3)
                try scene.op(allocator, .{ .kind = .on_teleported });
            try scene.updateWithFrame(allocator, &face, face.frames[f], dt, setup.follow);
        }
        try scene.updateWithFrame(allocator, &face, face.frames[2], 0.0, setup.follow);
        try scene.update(allocator, 1.0 / 60.0); // Without joint matrices: the scalp keeps its last skin
        try runScene(allocator, &scene, &coverage);
    }
    try std.testing.expectEqual(@as(u32, setups.len), coverage.scenes);
    // The paths of Update: skinning, the interpolation of the roots, collision, the global pose only path
    try std.testing.expect(coverage.skinned_updates > 10 and coverage.interpolated_updates > 10 and coverage.collision_updates > 10 and coverage.global_pose_only_updates > 5);
}

test "Hair scenes parity: random scenes" {
    const allocator = std.testing.allocator;
    var face_or_null = try HairSettingsParity.loadFace(allocator);
    defer if (face_or_null) |*f| f.deinit(allocator);
    var rng: fw.Rng = .{ .state = 0x2c0f_fee5 };
    var coverage: Coverage = .{};
    const num_scenes = 8;
    for (0..num_scenes) |round| {
        const face: ?*const Face = if (face_or_null != null and round % 2 == 1) &face_or_null.? else null;
        const name = try std.fmt.allocPrint(allocator, "random {d}", .{round});
        defer allocator.free(name);
        var scene = try Scene.init(allocator, name);
        defer scene.deinit(allocator);
        scene.world.gravity = .{ rng.float(-3, 3), rng.float(-15, -5), rng.float(-3, 3) };
        scene.world.optimize_broad_phase = rng.next() % 2;

        // Random materials: never a step gradient (R20 UB16)
        const num_materials = 1 + rng.next() % 3;
        var materials: [3]Material = undefined;
        for (materials[0..num_materials]) |*m| {
            m.* = if (rng.next() % 5 == 0) globalPoseOnlyMaterial(0.5) else variantMaterial(rng.next());
            m.simulation_strands_fraction = rng.float(0.2, 0.9);
            if (rng.next() % 2 == 0) m.grid_velocity_factor = .init(rng.float(0, 0.2), rng.float(0, 0.2), .{ .min_fraction = rng.float(0, 0.4), .max_fraction = rng.float(0.6, 1.0) });
        }
        const shape: HairSettingsParity.Shape = if (rng.next() % 2 == 0) .straight else .curly;
        var groom = try HairSettingsParity.makeGroom(allocator, &rng, face, shape, 20 + rng.next() % 40, 2 + rng.next() % 4, 6 + rng.next() % 14, .{ .random = num_materials });
        defer groom.deinit(allocator);
        scene.settings = try settingsBytes(allocator, face, materials[0..num_materials], &groom, .{
            .grid_size = .init(4 + rng.next() % 12, 4 + rng.next() % 12, 4 + rng.next() % 12, 0),
            .num_iterations_per_second = 120 + rng.next() % 400,
            .max_delta_time = rng.float(0.01, 0.05),
        });

        var center = Vec3.init(rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2));
        if (face) |f| {
            const neck = f.frames[0][f.head_joint];
            center = neck.getTranslation();
            const r = neck.getQuaternion();
            scene.hair.rotation = .{ r.getX(), r.getY(), r.getZ(), r.getW() };
        }
        scene.hair.position = .{ center.getX(), center.getY(), center.getZ() };
        const body = try addBodies(allocator, &scene, &rng, center);
        if (face) |f| {
            try scene.updateWithFrame(allocator, f, f.frames[0], 0.0, false);
            for (0..5) |_|
                try scene.updateWithFrame(allocator, f, f.frames[rng.next() % 3], rng.float(0.0, 0.06), rng.next() % 2 == 0);
        } else {
            try addOps(allocator, &scene, center, body);
            for (0..3) |_|
                try scene.update(allocator, rng.float(0.0, 0.06));
        }
        try runScene(allocator, &scene, &coverage);
    }
    try std.testing.expectEqual(@as(u32, num_scenes), coverage.scenes);
    try std.testing.expect(coverage.collision_updates > 10 and coverage.grid_updates > 10);
}
