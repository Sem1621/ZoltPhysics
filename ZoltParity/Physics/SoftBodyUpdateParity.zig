//! The SoftBodyUpdate parity layer (Phase 9 group S4, RemainingFeaturesArchitecture.md section 4): the update of a soft
//! body without PhysicsSystem.update, compared bit for bit with the C++ library. C ABI wrapper:
//! ZoltParity/Physics/SoftBodyUpdateReference.cpp.
//!
//! - A case is a PhysicsSystem with a few rigid bodies (boxes, spheres, capsules, a mesh, a compound, a scaled box,
//!   sensors, kinematic and dynamic bodies, a body in a layer that does not collide) and one soft body, and a list of
//!   operations: customUpdate with random delta times (whole or staged, see below), skinVertices (hard skin all or
//!   not), the setters (iterations, update position, double sided, pressure, vertex radius, skin constraints and the max
//!   distance multiplier, gravity factor, linear damping), a force, a vertex velocity and inverse mass,
//!   calculateMassAndInertia, SaveState / an update / RestoreState, the contact listener mode (none, accept, reject,
//!   sensor, mass scales that are sometimes exactly 0) and rigid body velocities and positions.
//! - Some rigid bodies and the soft body have a collision group: one group filter table with 4 sub groups and random
//!   disabled pairs, or a group without a filter, so that CanCollide rejects some bodies.
//! - The soft bodies come from generators like those of the SoftBodySettings layer (group S1): a cube (sCreateCube), a
//!   cloth with fixed corners and LRA constraints, a cloth with dihedral bends, a pressure sphere, a rod chain, a skinned
//!   cloth with back stops, and a 30 x 30 cloth (several parallel groups and a non-empty non-parallel group, 900
//!   vertices: a partial last batch of 64). The Zolt side builds the settings, both sides restore their
//!   SaveBinaryState bytes.
//! - Each side writes a transcript: after every operation the soft body (position, rotation, bounds, the motion
//!   properties, every vertex, rod and skin state, the local and predicted bounds, the volume, the private flags), its
//!   SaveState bytes and the rigid bodies (position, rotation, velocities, active). The transcripts must be equal byte
//!   for byte; a mismatch is reported with the operation and the field.
//! - After a whole operation the collision data of the vertices (colliding shape index, contact flag, largest
//!   penetration) and the lists of colliding shapes and sensors always have their reset values: UpdateSoftBodyState
//!   resets the vertices and UpdateRigidBodyVelocities clears the lists. So they are also written where they carry
//!   information:
//!   - The staged update is CustomUpdate step by step. After DetermineCollidingShapes it writes the colliding shapes
//!     and sensors: every member that Jolt initializes (the body, the motion type, the transform, friction,
//!     restitution, the mass scale, the leaf shapes, and for a dynamic body its inverse mass, inertia and velocities).
//!     After the collision phase it writes the collision data of every vertex and the sensor contacts. When the update
//!     is done it writes the colliding shapes again (the velocities and update flags of the collision response).
//!   - The contact listener writes its calls when they happen: the validate arguments and results, and the manifold of
//!     OnSoftBodyContactAdded with the collision data of every vertex.
//! - Coverage: the test fails unless the cases reached the hard paths (counted on the Zolt side, which equals the Jolt
//!   side when the transcripts are equal).
//! - The generators stay away from Jolt's asserts. With `-Dreference_asserts=true` a case fails when its C++ side hits a
//!   Jolt assert.
//!
//! A soft body without vertices (R20 UB1) is Zolt-only: SoftBodyMotionProperties.zig pins it.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ParityHooks = @import("../ParityHooks.zig");
const TestLayers = @import("test_layers");

const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const Float3 = zolt.Float3;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const AABox = zolt.AABox;
const Ref = zolt.Ref;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SimShapeFilter = zolt.SimShapeFilter;
const Triangle = zolt.Triangle;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const MeshShapeSettings = zolt.MeshShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SoftBodyContactListener = zolt.SoftBodyContactListener;
const SoftBodyContactSettings = zolt.SoftBodyContactSettings;
const SoftBodyValidateResult = zolt.SoftBodyValidateResult;
const SoftBodyManifold = zolt.SoftBodyManifold;
const SoftBodyCreationSettings = zolt.SoftBodyCreationSettings;
const SoftBodyMotionProperties = zolt.SoftBodyMotionProperties;
const SoftBodyUpdateContext = zolt.SoftBodyUpdateContext;
const SoftBodyVertex = zolt.SoftBodyVertex;
const RefConst = zolt.RefConst;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const CollisionGroup = zolt.CollisionGroup;
const S = zolt.SoftBodySharedSettings;
const Layers = TestLayers.Layers;
const flt_max = zolt.math.flt_max;
const trig = zolt.trigonometry;

const io = std.testing.io;

const jolt = struct {
    extern fn jolt_sbu_struct_sizes(out: *[4]u32) void;
    extern fn jolt_sbu_run(world: *const World, bodies: [*]const RigidBody, triangles: [*]const f32, settings: [*]const u8, settings_size: u32, soft_body: *const SoftBody, ops: [*]const Op, num_ops: u32, pool: [*]const f32, out: [*]u8, capacity: u32) u32;
};

/// SBUWorldDesc of SoftBodyUpdateReference.cpp
const World = extern struct {
    gravity: [3]f32,
    num_bodies: u32,
    optimize_broad_phase: u32,
    sim_shape_filter: u32,
    /// The pairs of sub groups (`group_filter_pairs`) whose collision the group filter table disables, one bit per pair
    group_filter_disabled: u32,
};

/// The shapes of the rigid bodies (SBURigidBodyDesc::mShape)
const ShapeKind = enum(u32) { box, sphere, capsule, mesh, compound, scaled_box };

/// SBURigidBodyDesc of SoftBodyUpdateReference.cpp
const RigidBody = extern struct {
    position: [3]f64,
    rotation: [4]f32,
    params: [8]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    shape: ShapeKind,
    motion_type: u32,
    is_sensor: u32,
    object_layer: u32,
    friction: f32,
    restitution: f32,
    user_data: u32,
    first_triangle: u32,
    num_triangles: u32,
    /// The collision group (see `makeCollisionGroup`)
    group: u32 = 0,
};

/// SBUSoftBodyDesc of SoftBodyUpdateReference.cpp
const SoftBody = extern struct {
    position: [3]f64,
    rotation: [4]f32,
    make_rotation_identity: u32,
    object_layer: u32,
    num_iterations: u32,
    linear_damping: f32,
    max_linear_velocity: f32,
    restitution: f32,
    friction: f32,
    pressure: f32,
    gravity_factor: f32,
    vertex_radius: f32,
    update_position: u32,
    faces_double_sided: u32,
    allow_sleeping: u32,
    add_to_system: u32,
    /// The collision group (see `makeCollisionGroup`)
    group: u32 = 0,
    padding: u32 = 0,
};

/// SBUOpKind of SoftBodyUpdateReference.cpp
const OpKind = enum(u32) {
    custom_update,
    skin_vertices,
    set_options,
    add_force,
    save_update_restore,
    calculate_mass_and_inertia,
    set_vertex,
    set_listener,
    set_body_velocity,
    set_body_position,
    staged_update,
};

/// SBUOp of SoftBodyUpdateReference.cpp
const Op = extern struct {
    kind: OpKind,
    u: [4]u32 = @splat(0),
    f: [8]f32 = @splat(0),
};

comptime {
    std.debug.assert(@sizeOf(World) == 28);
    std.debug.assert(@sizeOf(RigidBody) == 136);
    std.debug.assert(@sizeOf(SoftBody) == 104);
    std.debug.assert(@sizeOf(Op) == 52);
}

// Tags of the transcript (cSBUTag* of SoftBodyUpdateReference.cpp)
const tag_op: u32 = 0x5B000000;
const tag_validate: u32 = 0x5B100000;
const tag_added: u32 = 0x5B200000;
const tag_staged: u32 = 0x5B300000;

/// The operation index of the state before the first operation
const initial_op: u32 = 0xFFFFF;

/// The capacity of the transcript buffer of the C++ side
const transcript_capacity = 64 << 20;

/// The pairs of sub groups that World.group_filter_disabled can disable (cSBUGroupFilterPairs of
/// SoftBodyUpdateReference.cpp). The group filter table has 4 sub groups.
const group_filter_pairs = [_][2]u32{ .{ 0, 1 }, .{ 0, 2 }, .{ 0, 3 }, .{ 1, 2 }, .{ 1, 3 }, .{ 2, 3 } };

/// RigidBody.group and SoftBody.group: 0 is the default collision group. Otherwise bit 31 is set, bits 0 to 7 are the
/// sub group, bits 8 to 15 the group (0xFF: the invalid group) and bit 16 selects the group filter table.
fn encodeGroup(group: u32, sub_group: u32, with_filter: bool) u32 {
    return 0x80000000 | (@as(u32, @intFromBool(with_filter)) << 16) | (group << 8) | sub_group;
}

/// The collision group of RigidBody.group and SoftBody.group (SBUMakeCollisionGroup of SoftBodyUpdateReference.cpp)
fn makeCollisionGroup(group: u32, filter: *const GroupFilter) CollisionGroup {
    if (group == 0) return .{};
    const id = (group >> 8) & 0xFF;
    return .init(if ((group & 0x10000) != 0) filter else null, if (id == 0xFF) CollisionGroup.invalid_group else id, group & 0xFF);
}

// ---------------------------------------------------------------------------------------------------------------------
// The transcript

/// The transcript of the Zolt side (SBUTranscript of SoftBodyUpdateReference.cpp). It also records a label for every
/// field, so that a mismatch can be reported with its operation and field.
const Transcript = struct {
    allocator: Allocator,
    bytes: std.ArrayList(u8) = .empty,
    labels: std.ArrayList(Label) = .empty,
    /// The operation that is being written (for the labels)
    op: u32 = initial_op,
    /// An allocation failed (the listener cannot return errors)
    failed: bool = false,

    const Label = struct {
        offset: usize,
        name: []const u8,
        op: u32,
        item: u32,
    };

    fn deinit(self: *Transcript) void {
        self.bytes.deinit(self.allocator);
        self.labels.deinit(self.allocator);
    }

    /// The fields after this belong to `name` (item `item` of a list)
    fn label(self: *Transcript, name: []const u8, item: u32) void {
        self.labels.append(self.allocator, .{ .offset = self.bytes.items.len, .name = name, .op = self.op, .item = item }) catch {
            self.failed = true;
        };
    }

    fn raw(self: *Transcript, data: []const u8) void {
        self.bytes.appendSlice(self.allocator, data) catch {
            self.failed = true;
        };
    }

    fn u32v(self: *Transcript, value: u32) void {
        self.raw(std.mem.asBytes(&std.mem.nativeToLittle(u32, value)));
    }

    fn f32v(self: *Transcript, value: f32) void {
        self.u32v(@bitCast(value));
    }

    fn real(self: *Transcript, value: Real) void {
        self.raw(std.mem.asBytes(&value));
    }

    fn v3(self: *Transcript, value: Vec3) void {
        self.f32v(value.getX());
        self.f32v(value.getY());
        self.f32v(value.getZ());
    }

    fn quat(self: *Transcript, value: Quat) void {
        self.f32v(value.getX());
        self.f32v(value.getY());
        self.f32v(value.getZ());
        self.f32v(value.getW());
    }

    fn r3(self: *Transcript, value: RVec3) void {
        self.real(value.getX());
        self.real(value.getY());
        self.real(value.getZ());
    }

    fn box(self: *Transcript, value: AABox) void {
        self.v3(value.min);
        self.v3(value.max);
    }

    fn boolean(self: *Transcript, value: bool) void {
        self.u32v(@intFromBool(value));
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of a case

/// The contact listener (SBUListener of SoftBodyUpdateReference.cpp): writes its calls to the transcript
const Listener = struct {
    transcript: *Transcript,
    mode: u32 = 0,
    scales: [3]f32 = .{ 1.0, 1.0, 1.0 },
    /// Coverage (Zolt side)
    coverage: *Coverage,

    pub fn onSoftBodyContactValidate(self: *Listener, soft_body: *const Body, other_body: *const Body, settings: *SoftBodyContactSettings) SoftBodyValidateResult {
        const t = self.transcript;
        t.label("validate", 0);
        t.u32v(tag_validate);
        t.u32v(soft_body.getID().getIndexAndSequenceNumber());
        t.u32v(other_body.getID().getIndexAndSequenceNumber());
        t.boolean(settings.is_sensor);

        var result: SoftBodyValidateResult = .accept_contact;
        const odd = (other_body.getID().getIndex() & 1) != 0;
        switch (self.mode) {
            2 => if (odd) {
                result = .reject_contact;
            },
            3 => if (odd) {
                settings.is_sensor = true;
            },
            4 => {
                settings.inv_mass_scale1 = self.scales[0];
                settings.inv_mass_scale2 = self.scales[1];
                settings.inv_inertia_scale2 = self.scales[2];
            },
            else => {},
        }

        t.f32v(settings.inv_mass_scale1);
        t.f32v(settings.inv_mass_scale2);
        t.f32v(settings.inv_inertia_scale2);
        t.boolean(settings.is_sensor);
        t.u32v(@intFromEnum(result));
        self.coverage.validate_calls += 1;
        if (result == .reject_contact) self.coverage.rejected += 1;
        if (settings.is_sensor) self.coverage.sensor_validates += 1;
        // DetermineCollidingShapes' "Check if there will be any interaction" returns
        if (result == .accept_contact and !settings.is_sensor and settings.inv_mass_scale1 == 0.0 and (other_body.getMotionType() != .dynamic or settings.inv_mass_scale2 == 0.0))
            self.coverage.no_interaction += 1;
        return result;
    }

    pub fn onSoftBodyContactAdded(self: *Listener, soft_body: *const Body, manifold: *const SoftBodyManifold) void {
        const t = self.transcript;
        t.label("added", 0);
        t.u32v(tag_added);
        t.u32v(soft_body.getID().getIndexAndSequenceNumber());
        const vertices = manifold.getVertices();
        t.u32v(@intCast(vertices.len));
        for (vertices, 0..) |*v, i| {
            const has_contact = manifold.hasContact(v);
            t.label("added vertex", @intCast(i));
            t.boolean(has_contact);
            if (has_contact) {
                t.u32v(manifold.getContactBodyID(v).getIndexAndSequenceNumber());
                t.v3(manifold.getLocalContactPoint(v));
                t.v3(manifold.getContactNormal(v));
                self.coverage.vertex_contacts += 1;
            }
            // The collision data of the vertex before UpdateSoftBodyState resets it
            t.label("added vertex collision", @intCast(i));
            dumpVertexCollision(t, v);
        }
        t.label("added sensors", 0);
        t.u32v(manifold.getNumSensorContacts());
        for (0..manifold.getNumSensorContacts()) |i|
            t.u32v(manifold.getSensorContactBodyID(@intCast(i)).getIndexAndSequenceNumber());
        self.coverage.added_calls += 1;
        self.coverage.sensor_contacts += manifold.getNumSensorContacts();
    }
};

/// The simulation shape filter (SBUSimShapeFilter of SoftBodyUpdateReference.cpp): rejects the sub shapes with an odd
/// sub shape ID value of the bodies with user data 1
const ZoltSimShapeFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: SimShapeFilter = .init(ZoltSimShapeFilter),

    pub fn shouldCollide(self: *const ZoltSimShapeFilter, body1: *const Body, shape1: *const Shape, sub_shape_id1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id2: SubShapeID) bool {
        _ = .{ self, body1, shape1, sub_shape_id1, shape2 };
        return body2.getUserData() != 1 or (sub_shape_id2.getValue() & 1) == 0;
    }
};

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn loadR3(v: *const [3]f64) RVec3 {
    return RVec3.init(@floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]));
}

fn loadQuat(v: *const [4]f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

/// SBUMakeShapeSettings of SoftBodyUpdateReference.cpp: the returned settings have no reference yet
fn makeShapeSettings(allocator: Allocator, desc: *const RigidBody, triangles: []const f32) !*ShapeSettings {
    const p = &desc.params;
    return switch (desc.shape) {
        .box => (try BoxShapeSettings.create(allocator, load3(p[0..3]), .{ .convex_radius = p[3] })).asShapeSettings(),
        .sphere => (try SphereShapeSettings.create(allocator, p[0], .{})).asShapeSettings(),
        .capsule => (try CapsuleShapeSettings.create(allocator, p[0], p[1], .{})).asShapeSettings(),
        .mesh => blk: {
            var list: std.ArrayList(Triangle) = .empty;
            defer list.deinit(allocator);
            for (0..desc.num_triangles) |i| {
                const t = triangles[9 * (desc.first_triangle + i) ..][0..9];
                try list.append(allocator, .init(load3(t[0..3]), load3(t[3..6]), load3(t[6..9]), .{}));
            }
            break :blk (try MeshShapeSettings.create(allocator, list.items, .{})).asShapeSettings();
        },
        .compound => blk: {
            const compound = try StaticCompoundShapeSettings.create(allocator);
            try compound.base.addShape(load3(p[4..7]), Quat.identity(), (try BoxShapeSettings.create(allocator, load3(p[0..3]), .{ .convex_radius = p[3] })).asShapeSettings(), .{});
            try compound.base.addShape(load3(p[4..7]).negate(), Quat.identity(), (try SphereShapeSettings.create(allocator, p[7], .{})).asShapeSettings(), .{});
            break :blk compound.asShapeSettings();
        },
        .scaled_box => (try ScaledShapeSettings.create(allocator, (try BoxShapeSettings.create(allocator, load3(p[0..3]), .{ .convex_radius = p[3] })).asShapeSettings(), load3(p[4..7]))).asShapeSettings(),
    };
}

/// SBUDumpState of SoftBodyUpdateReference.cpp
fn dumpState(t: *Transcript, op_index: u32, soft_body: *const Body, bodies: []const *Body) !void {
    const mp = SoftBodyMotionProperties.fromBaseConst(soft_body.getMotionPropertiesUnchecked().?);

    t.op = op_index;
    t.label("op tag", 0);
    t.u32v(tag_op | op_index);

    // The body
    t.label("body position", 0);
    t.r3(soft_body.getPosition());
    t.label("body rotation", 0);
    t.quat(soft_body.getRotation());
    t.label("body center of mass position", 0);
    t.r3(soft_body.getCenterOfMassPosition());
    t.label("body world space bounds", 0);
    t.box(soft_body.getWorldSpaceBounds().*);

    // The motion properties
    t.label("linear velocity", 0);
    t.v3(mp.base.getLinearVelocity());
    t.label("angular velocity", 0);
    t.v3(mp.base.getAngularVelocity());
    t.label("inverse mass", 0);
    t.f32v(mp.base.getInverseMassUnchecked());
    t.label("inverse inertia diagonal", 0);
    t.v3(mp.base.getInverseInertiaDiagonal());
    t.label("inertia rotation", 0);
    t.quat(mp.base.getInertiaRotation());
    t.label("accumulated force", 0);
    t.v3(mp.base.getAccumulatedForce());
    t.label("settings", 0);
    t.u32v(mp.getNumIterations());
    t.f32v(mp.getPressure());
    t.f32v(mp.getVertexRadius());
    t.boolean(mp.getUpdatePosition());
    t.boolean(mp.getFacesDoubleSided());
    t.boolean(mp.getEnableSkinConstraints());
    t.f32v(mp.getSkinnedMaxDistanceMultiplier());

    // The vertices
    const vertices = mp.getVertices();
    t.label("number of vertices", 0);
    t.u32v(@intCast(vertices.len));
    for (vertices, 0..) |*v, i| {
        const item: u32 = @intCast(i);
        t.label("vertex previous position", item);
        t.v3(v.previous_position);
        t.label("vertex position", item);
        t.v3(v.position);
        t.label("vertex velocity", item);
        t.v3(v.velocity);
        t.label("vertex collision plane", item);
        t.v3(v.collision_plane.getNormal());
        t.f32v(v.collision_plane.getConstant());
        t.label("vertex colliding shape index", item);
        t.u32v(@bitCast(v.colliding_shape_index));
        t.label("vertex has contact", item);
        t.boolean(v.has_contact);
        t.label("vertex largest penetration", item);
        t.f32v(v.largest_penetration);
        t.label("vertex inverse mass", item);
        t.f32v(v.inv_mass);
    }

    // The rods
    const num_rods: u32 = @intCast(mp.getSettings().?.rod_stretch_shear_constraints.items.len);
    t.label("number of rods", 0);
    t.u32v(num_rods);
    for (0..num_rods) |i| {
        t.label("rod rotation", @intCast(i));
        t.quat(mp.getRodRotation(@intCast(i)));
        t.label("rod angular velocity", @intCast(i));
        t.v3(mp.getRodAngularVelocity(@intCast(i)));
    }

    // The skin states
    t.label("number of skin states", 0);
    t.u32v(@intCast(mp.skin_state.items.len));
    for (mp.skin_state.items, 0..) |*s, i| {
        t.label("skin previous position", @intCast(i));
        t.v3(s.previous_position);
        t.label("skin position", @intCast(i));
        t.v3(s.position);
        t.label("skin normal", @intCast(i));
        t.v3(s.normal);
    }

    // Bounds, volume and the private state
    t.label("local bounds", 0);
    t.box(mp.getLocalBounds());
    t.label("local predicted bounds", 0);
    t.box(mp.local_predicted_bounds);
    t.label("volume", 0);
    t.f32v(mp.getVolume());
    t.label("skin state previous position valid", 0);
    t.boolean(mp.skin_state_previous_position_valid);
    t.label("need contact callback", 0);
    t.boolean(mp.need_contact_callback.load(.monotonic));
    t.label("number of colliding shapes and sensors", 0);
    t.u32v(@intCast(mp.colliding_shapes.items.len));
    t.u32v(@intCast(mp.colliding_sensors.items.len));

    // The SaveState bytes of the body
    var recorder: StateRecorderImpl = .init(t.allocator);
    defer recorder.deinit();
    soft_body.saveState(recorder.recorder());
    if (recorder.bad_bit) return error.OutOfMemory;
    t.label("save state size", 0);
    t.u32v(@intCast(recorder.getDataSize()));
    t.label("save state bytes", 0);
    t.raw(recorder.getData());

    // The rigid bodies
    for (bodies, 0..) |b, i| {
        const item: u32 = @intCast(i);
        t.label("rigid body position", item);
        t.r3(b.getPosition());
        t.label("rigid body rotation", item);
        t.quat(b.getRotation());
        t.label("rigid body linear velocity", item);
        t.v3(b.getLinearVelocity());
        t.label("rigid body angular velocity", item);
        t.v3(b.getAngularVelocity());
        t.label("rigid body active", item);
        t.boolean(b.isActive());
    }

    if (t.failed) return error.OutOfMemory;
}

/// The collision data of a vertex (SBUDumpVertexCollision of SoftBodyUpdateReference.cpp)
fn dumpVertexCollision(t: *Transcript, v: *const SoftBodyVertex) void {
    t.v3(v.collision_plane.getNormal());
    t.f32v(v.collision_plane.getConstant());
    t.u32v(@bitCast(v.colliding_shape_index));
    t.boolean(v.has_contact);
    t.f32v(v.largest_penetration);
}

/// The 3x3 part and the translation of a matrix (SBUTranscript::Mat of SoftBodyUpdateReference.cpp)
fn dumpMat(t: *Transcript, m: Mat44, num_columns: u32) void {
    for (0..num_columns) |i| t.v3(m.getColumn3(@intCast(i)));
}

/// The leaf shapes of a colliding shape or sensor
fn dumpLeafShapes(t: *Transcript, shapes: []const SoftBodyMotionProperties.LeafShape) void {
    t.u32v(@intCast(shapes.len));
    for (shapes) |*l| {
        dumpMat(t, l.transform, 4);
        t.v3(l.scale);
        t.u32v(@intFromEnum(l.shape.get().?.getSubType()));
    }
}

/// The colliding shapes and sensors of the motion properties (SBUDumpColliding of SoftBodyUpdateReference.cpp). The
/// members that Jolt leaves uninitialized are skipped: the dynamic members of a body that is not dynamic, and the contact
/// flag of a sensor before DetermineSensorCollisions (`sensor_contacts` false).
fn dumpColliding(t: *Transcript, mp: *const SoftBodyMotionProperties, sensor_contacts: bool) void {
    t.label("staged colliding shapes", 0);
    t.u32v(@intCast(mp.colliding_shapes.items.len));
    for (mp.colliding_shapes.items, 0..) |*cs, i| {
        const item: u32 = @intCast(i);
        t.label("staged colliding shape", item);
        t.u32v(cs.body_id.getIndexAndSequenceNumber());
        t.u32v(@intFromEnum(cs.motion_type));
        dumpMat(t, cs.center_of_mass_transform, 4);
        t.f32v(cs.friction);
        t.f32v(cs.restitution);
        t.f32v(cs.soft_body_inv_mass_scale);
        t.boolean(cs.update_velocities);
        t.label("staged colliding shape leaf shapes", item);
        dumpLeafShapes(t, cs.shapes.items);
        if (cs.motion_type == .dynamic) {
            t.label("staged colliding shape mass and velocities", item);
            t.f32v(cs.inv_mass);
            dumpMat(t, cs.inv_inertia, 3);
            t.v3(cs.linear_velocity);
            t.v3(cs.angular_velocity);
            t.v3(cs.original_linear_velocity);
            t.v3(cs.original_angular_velocity);
        }
    }
    t.label("staged colliding sensors", 0);
    t.u32v(@intCast(mp.colliding_sensors.items.len));
    for (mp.colliding_sensors.items, 0..) |*cs, i| {
        t.label("staged colliding sensor", @intCast(i));
        t.u32v(cs.body_id.getIndexAndSequenceNumber());
        dumpMat(t, cs.center_of_mass_transform, 4);
        dumpLeafShapes(t, cs.shapes.items);
        if (sensor_contacts) t.boolean(cs.has_contact);
    }
}

/// The world space bounds that DetermineCollidingShapes queries the broad phase with
fn queryBounds(mp: *const SoftBodyMotionProperties, soft_body: *const Body) AABox {
    var local_bounds = mp.getLocalBounds();
    local_bounds.encapsulate(mp.local_predicted_bounds);
    local_bounds.expandBy(Vec3.replicate(mp.getVertexRadius()));
    return local_bounds.transformedRMat44(soft_body.getCenterOfMassTransform());
}

/// RestoreBinaryState of `bytes` into new settings
fn restoreSettings(allocator: Allocator, bytes: []const u8) !Ref(S) {
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    try ref.get().?.restoreBinaryState(in.streamIn());
    if (in.isFailed()) return error.RestoreFailed;
    return ref;
}

/// Coverage: the rigid bodies in the query bounds of the soft body whose collision group does not collide with the soft
/// body's (the false branch of CanCollide in DetermineCollidingShapes)
fn countGroupFiltered(mp: *const SoftBodyMotionProperties, soft_body: *const Body, bodies: []const *Body, descs: []const RigidBody, coverage: *Coverage) void {
    const bounds = queryBounds(mp, soft_body);
    for (bodies, descs) |b, *d| {
        if (d.object_layer != Layers.lq_debris and bounds.overlaps(b.getWorldSpaceBounds().*) and !soft_body.getCollisionGroup().canCollide(b.getCollisionGroup()))
            coverage.group_filtered += 1;
    }
}

/// SBUStagedUpdate of SoftBodyUpdateReference.cpp: CustomUpdate with the state written after DetermineCollidingShapes
/// (stage 0), after the collision phase (stage 1, the vertex collision data and the sensor contacts) and when the
/// update is done (stage 2, before UpdateRigidBodyVelocities clears the colliding shapes)
fn stagedUpdate(t: *Transcript, mp: *SoftBodyMotionProperties, delta_time: f32, soft_body: *Body, system: *PhysicsSystem, coverage: *Coverage) !void {
    // Create update context
    var context: SoftBodyUpdateContext = .{};
    mp.initializeUpdateContext(delta_time, soft_body, system, &context);

    // Determine bodies we're colliding with
    try mp.determineCollidingShapes(&context, system, system.getBodyLockInterface());
    t.label("staged stage 0", 0);
    t.u32v(tag_staged | 0);
    t.v3(context.gravity);
    t.v3(context.displacement_due_to_gravity);
    t.f32v(context.sub_step_delta_time);
    t.u32v(mp.num_sensors.load(.monotonic));
    dumpColliding(t, mp, false);
    for (mp.colliding_shapes.items) |*cs| {
        if (cs.motion_type == .dynamic) coverage.staged_dynamic_shapes += 1;
    }

    // The collision phase: the collision planes, then the sensors
    const settings = system.getPhysicsSettings();
    while (true) {
        const state = context.state.load(.acquire);
        if (state != .determine_collision_planes and state != .determine_sensor_collisions) break;
        if (mp.parallelUpdate(&context, settings) != .did_work) return error.NoProgress;
    }
    t.label("staged stage 1", 0);
    t.u32v(tag_staged | 1);
    t.u32v(@intFromEnum(context.state.load(.acquire)));
    for (mp.getVertices(), 0..) |*v, i| {
        t.label("staged vertex collision", @intCast(i));
        dumpVertexCollision(t, v);
        if (v.colliding_shape_index >= 0) coverage.staged_vertex_hits += 1;
    }
    t.label("staged sensor contacts", 0);
    for (mp.colliding_sensors.items) |*cs| {
        t.boolean(cs.has_contact);
        if (cs.has_contact) coverage.staged_sensor_contacts += 1;
    }
    t.boolean(mp.need_contact_callback.load(.monotonic));

    // Call the internal update until it finishes
    var status = mp.parallelUpdate(&context, settings);
    while (status == .did_work)
        status = mp.parallelUpdate(&context, settings);
    if (status != .done) return error.NoProgress;
    t.label("staged stage 2", 0);
    t.u32v(tag_staged | 2);
    t.v3(context.delta_position);
    t.u32v(@intFromEnum(context.can_sleep));
    dumpColliding(t, mp, true);
    for (mp.colliding_shapes.items) |*cs| {
        if (cs.update_velocities) coverage.staged_pushed += 1;
        if (cs.update_velocities and cs.soft_body_inv_mass_scale == 0.0) coverage.zero_scale_pushes += 1;
    }

    // Update the state of the bodies we've collided with
    mp.updateRigidBodyVelocities(&context, system.getBodyInterfaceMut());

    // Update position of the soft body
    if (mp.getUpdatePosition())
        system.getBodyInterfaceMut().setPosition(soft_body.getID(), soft_body.getPosition().addVec3(context.delta_position), .dont_activate);
}

/// jolt_sbu_run of SoftBodyUpdateReference.cpp on Zolt
fn zoltRun(allocator: Allocator, c: *const Case, t: *Transcript, coverage: *Coverage) !void {
    // The shared settings
    var shared = try restoreSettings(allocator, c.settings_bytes);
    defer shared.deinit();

    // The world (the layers of UnitTests/Layers.h)
    var layers: TestLayers.BPLayerInterfaceImpl = .init();
    var object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{};
    var pairs: TestLayers.ObjectLayerPairFilterImpl = .{};
    var system: PhysicsSystem = .empty;
    try system.init(allocator, io, 64, 0, 64, 64, &layers.base, &object_vs_broad_phase.base, &pairs.base, .{});
    defer system.deinit();
    system.setGravity(load3(&c.world.gravity));
    var sim_shape_filter: ZoltSimShapeFilter = .{};
    if (c.world.sim_shape_filter != 0)
        system.setSimShapeFilter(&sim_shape_filter.base);
    var listener: Listener = .{ .transcript = t, .coverage = coverage };
    const bi = system.getBodyInterfaceMut();

    // The group filter of the collision groups
    const group_filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 4 });
    var group_filter_ref: RefConst(GroupFilter) = .init(group_filter.asGroupFilter());
    defer group_filter_ref.deinit();
    for (group_filter_pairs, 0..) |pair, k| {
        if (((c.world.group_filter_disabled >> @intCast(k)) & 1) != 0)
            group_filter.disableCollision(pair[0], pair[1]);
    }

    // The rigid bodies
    var bodies: std.ArrayList(*Body) = .empty;
    defer bodies.deinit(allocator);
    defer for (bodies.items) |b| {
        const id = b.getID();
        bi.removeBody(id);
        bi.destroyBody(id);
    };
    for (c.bodies.items) |*d| {
        var s: BodyCreationSettings = .initWithSettings(try makeShapeSettings(allocator, d, c.triangles.items), loadR3(&d.position), loadQuat(&d.rotation), @enumFromInt(d.motion_type), @intCast(d.object_layer));
        defer s.deinit();
        s.linear_velocity = load3(&d.linear_velocity);
        s.angular_velocity = load3(&d.angular_velocity);
        s.is_sensor = d.is_sensor != 0;
        s.friction = d.friction;
        s.restitution = d.restitution;
        s.user_data = d.user_data;
        s.collision_group = makeCollisionGroup(d.group, group_filter.asGroupFilter());
        const body = (try bi.createBody(&s)) orelse return error.NoBody;
        try bodies.append(allocator, body);
        try bi.addBody(body.getID(), if (d.motion_type == @intFromEnum(zolt.MotionType.static)) .dont_activate else .activate);
    }

    // The soft body
    const sd = &c.soft_body;
    var ss: SoftBodyCreationSettings = .init(shared.get(), loadR3(&sd.position), loadQuat(&sd.rotation), @intCast(sd.object_layer));
    defer ss.deinit();
    ss.make_rotation_identity = sd.make_rotation_identity != 0;
    ss.num_iterations = sd.num_iterations;
    ss.linear_damping = sd.linear_damping;
    ss.max_linear_velocity = sd.max_linear_velocity;
    ss.restitution = sd.restitution;
    ss.friction = sd.friction;
    ss.pressure = sd.pressure;
    ss.gravity_factor = sd.gravity_factor;
    ss.vertex_radius = sd.vertex_radius;
    ss.update_position = sd.update_position != 0;
    ss.faces_double_sided = sd.faces_double_sided != 0;
    ss.allow_sleeping = sd.allow_sleeping != 0;
    ss.collision_group = makeCollisionGroup(sd.group, group_filter.asGroupFilter());
    const soft_body = (try bi.createSoftBody(&ss)) orelse return error.NoBody;
    const soft_body_id = soft_body.getID();
    defer {
        if (sd.add_to_system != 0) bi.removeBody(soft_body_id);
        bi.destroyBody(soft_body_id);
    }
    if (sd.add_to_system != 0)
        try bi.addBody(soft_body_id, .activate);
    if (c.world.optimize_broad_phase != 0)
        try system.optimizeBroadPhase();
    const mp = SoftBodyMotionProperties.fromBase(soft_body.getMotionPropertiesUncheckedMut().?);

    var temp_allocator: TempAllocatorImpl = try .init(allocator, 1024 * 1024);
    defer temp_allocator.deinit();

    // The operations
    try dumpState(t, initial_op, soft_body, bodies.items);
    for (c.ops.items, 0..) |*op, op_index| {
        t.op = @intCast(op_index);
        switch (op.kind) {
            .custom_update, .staged_update => {
                // Coverage: the bodies that the collision group filters out, the rigid bodies that the soft body pushes
                countGroupFiltered(mp, soft_body, bodies.items, c.bodies.items, coverage);
                var velocities: [max_bodies]Vec3 = undefined;
                for (bodies.items, 0..) |b, i| velocities[i] = b.getLinearVelocity();
                if (op.kind == .custom_update)
                    try mp.customUpdate(op.f[0], soft_body, &system)
                else
                    try stagedUpdate(t, mp, op.f[0], soft_body, &system, coverage);
                for (bodies.items, 0..) |b, i| {
                    if (!b.getLinearVelocity().eql(velocities[i])) {
                        coverage.pushed_bodies += 1;
                        // An infinitely heavy soft body pushes a dynamic body (vertex_inv_mass 0; stagedUpdate counts its own)
                        if (op.kind == .custom_update and listener.mode == 4 and listener.scales[0] == 0.0 and listener.scales[1] != 0.0) coverage.zero_scale_pushes += 1;
                    }
                }
                coverage.updates += 1;
            },
            .skin_vertices => {
                var joints: [max_joints]Mat44 = undefined;
                for (0..op.u[1]) |j| {
                    const m = c.pool.items[op.u[0] + 16 * j ..][0..16];
                    joints[j] = Mat44.init(Vec4.init(m[0], m[1], m[2], m[3]), Vec4.init(m[4], m[5], m[6], m[7]), Vec4.init(m[8], m[9], m[10], m[11]), Vec4.init(m[12], m[13], m[14], m[15]));
                }
                try mp.skinVertices(soft_body.getCenterOfMassTransform(), joints[0..op.u[1]], op.u[2] != 0, temp_allocator.tempAllocator());
                coverage.skins += 1;
            },
            .set_options => {
                mp.setNumIterations(op.u[0]);
                mp.setUpdatePosition(op.u[1] != 0);
                mp.setFacesDoubleSided(op.u[2] != 0);
                mp.setEnableSkinConstraints(op.u[3] != 0);
                mp.setPressure(op.f[0]);
                mp.setVertexRadius(op.f[1]);
                mp.setSkinnedMaxDistanceMultiplier(op.f[2]);
                mp.base.setGravityFactor(op.f[3]);
                mp.base.setLinearDamping(op.f[4]);
            },
            .add_force => soft_body.addForce(load3(op.f[0..3])),
            .save_update_restore => {
                var recorder: StateRecorderImpl = .init(allocator);
                defer recorder.deinit();
                soft_body.saveState(recorder.recorder());
                try mp.customUpdate(op.f[0], soft_body, &system);
                recorder.rewind();
                soft_body.restoreState(recorder.recorder());
                coverage.restores += 1;
            },
            .calculate_mass_and_inertia => mp.calculateMassAndInertia(),
            .set_vertex => {
                const v = mp.getVertexMut(op.u[0]);
                v.velocity = load3(op.f[0..3]);
                v.inv_mass = op.f[3];
            },
            .set_listener => {
                listener.mode = op.u[0];
                listener.scales = op.f[0..3].*;
                system.setSoftBodyContactListener(if (op.u[0] == 0) null else SoftBodyContactListener.init(&listener));
            },
            .set_body_velocity => bi.setLinearAndAngularVelocity(bodies.items[op.u[0]].getID(), load3(op.f[0..3]), load3(op.f[3..6])),
            .set_body_position => bi.setPosition(bodies.items[op.u[0]].getID(), RVec3.fromVec3(load3(op.f[0..3])), .dont_activate),
        }

        try dumpState(t, @intCast(op_index), soft_body, bodies.items);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The cases

/// The kinds of soft bodies
const Kind = enum { cube, cloth_lra, dihedral_cloth, pressure_sphere, rod_chain, skinned_cloth, big_cloth };

const max_bodies = 12;
const max_joints = 4;

/// One generated case
const Case = struct {
    kind: Kind,
    world: World,
    bodies: std.ArrayList(RigidBody) = .empty,
    triangles: std.ArrayList(f32) = .empty,
    /// The SaveBinaryState bytes of the shared settings (both sides restore them)
    settings_bytes: []const u8 = &.{},
    soft_body: SoftBody,
    ops: std.ArrayList(Op) = .empty,
    /// The joint matrices of the skin operations (16 floats each)
    pool: std.ArrayList(f32) = .empty,
    /// Number of vertices and update groups of the settings (coverage)
    num_vertices: u32 = 0,
    num_update_groups: u32 = 0,
    non_parallel_group_empty: bool = true,

    fn deinit(self: *Case, allocator: Allocator) void {
        self.bodies.deinit(allocator);
        self.triangles.deinit(allocator);
        allocator.free(self.settings_bytes);
        self.ops.deinit(allocator);
        self.pool.deinit(allocator);
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

    /// A normalized quaternion near the identity (`angle` radians at most)
    fn smallRotation(self: *Gen, angle: f32) Quat {
        const axis = Vec3.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1)).normalizedOr(Vec3.axisY());
        return Quat.rotation(axis, self.float(-angle, angle));
    }

    fn quat(self: *Gen) Quat {
        return Quat.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(0.1, 1)).normalized();
    }
};

/// A compliance: zero or a small value
fn genCompliance(g: *Gen) f32 {
    return switch (g.int(0, 3)) {
        0 => 0.0,
        1 => 1.0e-5,
        2 => g.float(0.0, 1.0e-3),
        else => 1.0e-4,
    };
}

/// Cloth: nx x nz vertices in the XZ plane around the origin, alternating diagonals
fn addCloth(allocator: Allocator, s: *S, g: *Gen, nx: u32, nz: u32, spacing: f32, fixed_corners: bool) !void {
    const ox = 0.5 * spacing * @as(f32, @floatFromInt(nx - 1));
    const oz = 0.5 * spacing * @as(f32, @floatFromInt(nz - 1));
    for (0..nz) |z| for (0..nx) |x| {
        const corner = (x == 0 or x == nx - 1) and (z == 0 or z == nz - 1);
        const inv_mass: f32 = if (fixed_corners and corner) 0.0 else if (g.chance(70)) 1.0 else g.float(0.5, 2.0);
        try s.vertices.append(allocator, .init(.{ .x = spacing * @as(f32, @floatFromInt(x)) - ox, .y = 0.0, .z = spacing * @as(f32, @floatFromInt(z)) - oz }, .{ .inv_mass = inv_mass }));
    };
    for (0..nz - 1) |z| for (0..nx - 1) |x| {
        const v0: u32 = @intCast(z * nx + x);
        const v1 = v0 + 1;
        const v2 = v0 + nx;
        const v3 = v2 + 1;
        if ((x + z) % 2 == 0) {
            try s.addFace(.init(v0, v2, v1, .{}));
            try s.addFace(.init(v1, v2, v3, .{}));
        } else {
            try s.addFace(.init(v0, v3, v1, .{}));
            try s.addFace(.init(v0, v2, v3, .{}));
        }
    };
}

/// A UV sphere around the origin with outward faces (positive volume, so pressure applies)
fn addSphere(allocator: Allocator, s: *S, g: *Gen, radius: f32, num_lat: u32, num_lon: u32) !void {
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = radius, .z = 0 }, .{}));
    for (1..num_lat) |i| {
        const theta = std.math.pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_lat));
        for (0..num_lon) |j| {
            const phi = 2.0 * std.math.pi * @as(f32, @floatFromInt(j)) / @as(f32, @floatFromInt(num_lon));
            const ring = radius * trig.sin(theta);
            try s.vertices.append(allocator, .init(.{ .x = ring * trig.cos(phi), .y = radius * trig.cos(theta), .z = ring * trig.sin(phi) }, .{ .inv_mass = if (g.chance(80)) 1.0 else g.float(0.5, 2.0) }));
        }
    }
    const bottom: u32 = @intCast(s.vertices.items.len);
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = -radius, .z = 0 }, .{}));
    const ringVertex = struct {
        fn f(lon: u32, i: usize, j: usize) u32 {
            return @intCast(1 + i * lon + j % lon);
        }
    }.f;
    for (0..num_lon) |j| {
        try s.addFace(.init(0, ringVertex(num_lon, 0, j + 1), ringVertex(num_lon, 0, j), .{}));
        for (0..num_lat - 2) |i| {
            try s.addFace(.init(ringVertex(num_lon, i, j), ringVertex(num_lon, i, j + 1), ringVertex(num_lon, i + 1, j + 1), .{}));
            try s.addFace(.init(ringVertex(num_lon, i, j), ringVertex(num_lon, i + 1, j + 1), ringVertex(num_lon, i + 1, j), .{}));
        }
        try s.addFace(.init(bottom, ringVertex(num_lon, num_lat - 2, j), ringVertex(num_lon, num_lat - 2, j + 1), .{}));
    }

    // Flip the faces when they point inwards
    var six_volume: f32 = 0.0;
    for (s.faces.items) |f| {
        const x1 = Vec3.loadFloat3Unsafe(&s.vertices.items[f.vertex[0]].position);
        const x2 = Vec3.loadFloat3Unsafe(&s.vertices.items[f.vertex[1]].position);
        const x3 = Vec3.loadFloat3Unsafe(&s.vertices.items[f.vertex[2]].position);
        six_volume += x1.cross(x2).dot(x3);
    }
    if (six_volume < 0.0) {
        for (s.faces.items) |*f| std.mem.swap(u32, &f.vertex[1], &f.vertex[2]);
    }
}

/// Builds the shared settings of a kind, optimized
fn genSettings(allocator: Allocator, g: *Gen, kind: Kind, c: *Case) !Ref(S) {
    if (kind == .cube) {
        const grid = g.int(2, 8);
        var cube = try S.createCube(allocator, grid, g.float(0.1, 0.3));
        errdefer cube.deinit();
        return cube;
    }

    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    const s = ref.get().?;
    switch (kind) {
        .cube => unreachable,
        .cloth_lra, .dihedral_cloth, .big_cloth => {
            const n: u32 = if (kind == .big_cloth) 30 else g.int(3, 12);
            const m: u32 = if (kind == .big_cloth) 30 else g.int(3, 12);
            try addCloth(allocator, s, g, n, m, if (kind == .big_cloth) 0.05 else g.float(0.08, 0.25), kind != .dihedral_cloth or g.chance(50));
            const lra: S.LRAType = if (kind == .cloth_lra) (if (g.chance(50)) .euclidean_distance else .geodesic_distance) else .none;
            const bend: S.BendType = if (kind == .dihedral_cloth) .dihedral else if (g.chance(50)) .distance else .none;
            try s.createConstraints(&.{.init(genCompliance(g), genCompliance(g), genCompliance(g), .{ .lra_type = lra, .lra_max_distance_multiplier = g.float(1.0, 1.1) })}, .{ .bend_type = bend });
        },
        .pressure_sphere => {
            try addSphere(allocator, s, g, g.float(0.2, 0.6), g.int(3, 8), g.int(4, 12));
            try s.createConstraints(&.{.init(genCompliance(g), genCompliance(g), genCompliance(g), .{})}, .{ .bend_type = if (g.chance(50)) .distance else .dihedral });
        },
        .rod_chain => {
            const num_chains = g.int(1, 3);
            for (0..num_chains) |chain| {
                var p = Vec3.init(g.float(-0.3, 0.3), g.float(-0.2, 0.2), 0.2 * @as(f32, @floatFromInt(chain)));
                var direction = Vec3.init(g.float(-1, 1), g.float(-0.3, 0.3), g.float(-1, 1)).normalizedOr(Vec3.axisX());
                var prev: u32 = @intCast(s.vertices.items.len);
                try s.vertices.append(allocator, .init(.{ .x = p.getX(), .y = p.getY(), .z = p.getZ() }, .{ .inv_mass = if (g.chance(50)) 0.0 else 1.0 }));
                var prev_rod: ?u32 = null;
                for (0..g.int(2, 12)) |_| {
                    direction = direction.add(Vec3.init(g.float(-0.3, 0.3), g.float(-0.3, 0.3), g.float(-0.3, 0.3))).normalizedOr(Vec3.axisX());
                    p = p.add(direction.mulScalar(g.float(0.05, 0.15)));
                    const v: u32 = @intCast(s.vertices.items.len);
                    try s.vertices.append(allocator, .init(.{ .x = p.getX(), .y = p.getY(), .z = p.getZ() }, .{ .inv_mass = if (g.chance(80)) 1.0 else g.float(0.5, 2.0) }));
                    const rod: u32 = @intCast(s.rod_stretch_shear_constraints.items.len);
                    var r: S.RodStretchShear = .init(prev, v, .{ .compliance = genCompliance(g) });
                    r.inv_mass = g.float(0.5, 2.0);
                    try s.rod_stretch_shear_constraints.append(allocator, r);
                    if (prev_rod) |pr| try s.rod_bend_twist_constraints.append(allocator, .init(pr, rod, .{ .compliance = genCompliance(g) }));
                    prev_rod = rod;
                    prev = v;
                }
            }
            try s.calculateRodProperties();
        },
        .skinned_cloth => {
            const n = g.int(3, 10);
            const m = g.int(3, 10);
            try addCloth(allocator, s, g, n, m, g.float(0.08, 0.2), false);
            try s.createConstraints(&.{.init(genCompliance(g), genCompliance(g), genCompliance(g), .{})}, .{ .bend_type = if (g.chance(50)) .distance else .none });
            // Joint 0 at the origin, joint 1 and 2 offset: the inverse bind matrices undo the offsets
            for (0..3) |j| {
                const offset = Vec3.init(0.1 * @as(f32, @floatFromInt(j)), 0.05 * @as(f32, @floatFromInt(j)), 0);
                try s.inv_bind_matrices.append(allocator, .init(@intCast(j), Mat44.translation(offset.negate())));
            }
            for (0..s.vertices.items.len) |v| {
                if (g.chance(15)) continue;
                const max_distance: f32 = switch (g.int(0, 3)) {
                    0 => 0.0, // Hard skinned
                    1 => flt_max, // Only the back stop
                    else => g.float(0.01, 0.2),
                };
                const back_stop_distance: f32 = if (g.chance(30)) flt_max else g.float(-0.05, 0.05);
                var skinned: S.Skinned = .init(@intCast(v), max_distance, back_stop_distance, g.float(0.05, 0.5));
                skinned.weights[0] = .init(g.int(0, 2), g.float(0.2, 1.0));
                if (g.chance(50)) skinned.weights[1] = .init(g.int(0, 2), g.float(0.2, 1.0));
                skinned.normalizeWeights();
                try s.skinned_constraints.append(allocator, skinned);
            }
            try s.calculateSkinnedConstraintNormals();
        },
    }
    try s.optimize();
    _ = c;
    return ref;
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

/// A rigid body near the soft body
fn genBody(allocator: Allocator, g: *Gen, c: *Case, index: usize) !RigidBody {
    var d: RigidBody = .{
        .position = .{ g.float(-0.8, 0.8), g.float(-0.6, 0.3), g.float(-0.8, 0.8) },
        .rotation = .{ 0, 0, 0, 1 },
        .params = @splat(0),
        .linear_velocity = @splat(0),
        .angular_velocity = @splat(0),
        .shape = @enumFromInt(g.int(0, 5)),
        .motion_type = g.int(0, 2),
        .is_sensor = 0,
        .object_layer = 0,
        .friction = g.float(0.0, 1.0),
        .restitution = g.float(0.0, 1.0),
        .user_data = if (g.chance(30)) 1 else 0,
        .first_triangle = 0,
        .num_triangles = 0,
    };
    if (g.chance(50)) d.rotation = blk: {
        const q = g.quat();
        break :blk .{ q.getX(), q.getY(), q.getZ(), q.getW() };
    };

    // The first body is a floor under the soft body
    if (index == 0) {
        d.shape = if (g.chance(50)) .box else .mesh;
        d.motion_type = 0;
        d.position = .{ 0, -0.6, 0 };
        d.rotation = .{ 0, 0, 0, 1 };
    }

    switch (d.shape) {
        .box => d.params = .{ g.float(0.1, 0.5), g.float(0.1, 0.5), g.float(0.1, 0.5), 0.0, 0, 0, 0, 0 },
        .sphere => d.params[0] = g.float(0.1, 0.4),
        .capsule => {
            d.params[0] = g.float(0.05, 0.3);
            d.params[1] = g.float(0.05, 0.2);
        },
        .mesh => {
            // A small terrain of n x n quads, always static
            d.motion_type = 0;
            const n = g.int(2, 5);
            const size = g.float(0.3, 1.0);
            d.first_triangle = @intCast(c.triangles.items.len / 9);
            var heights: [6][6]f32 = undefined;
            for (0..n + 1) |z| for (0..n + 1) |x| {
                heights[z][x] = g.float(-0.1, 0.1);
            };
            for (0..n) |z| for (0..n) |x| {
                const fx = @as(f32, @floatFromInt(x)) / @as(f32, @floatFromInt(n));
                const fz = @as(f32, @floatFromInt(z)) / @as(f32, @floatFromInt(n));
                const step = 1.0 / @as(f32, @floatFromInt(n));
                const p00 = [3]f32{ size * (2 * fx - 1), heights[z][x], size * (2 * fz - 1) };
                const p10 = [3]f32{ size * (2 * (fx + step) - 1), heights[z][x + 1], size * (2 * fz - 1) };
                const p01 = [3]f32{ size * (2 * fx - 1), heights[z + 1][x], size * (2 * (fz + step) - 1) };
                const p11 = [3]f32{ size * (2 * (fx + step) - 1), heights[z + 1][x + 1], size * (2 * (fz + step) - 1) };
                try c.triangles.appendSlice(allocator, &(p00 ++ p01 ++ p10));
                try c.triangles.appendSlice(allocator, &(p10 ++ p01 ++ p11));
            };
            d.num_triangles = 2 * n * n;
        },
        .compound => d.params = .{ g.float(0.05, 0.2), g.float(0.05, 0.2), g.float(0.05, 0.2), 0.0, g.float(-0.2, 0.2), g.float(-0.2, 0.2), g.float(-0.2, 0.2), g.float(0.05, 0.2) },
        .scaled_box => d.params = .{ g.float(0.1, 0.3), g.float(0.1, 0.3), g.float(0.1, 0.3), 0.0, g.float(0.5, 2.0), g.float(0.5, 2.0), if (g.chance(30)) -1.0 else 1.0, 0 },
    }
    if (d.params[3] == 0.0 and (d.shape == .box or d.shape == .compound or d.shape == .scaled_box)) {
        // A convex radius that fits the box
        const smallest = @min(d.params[0], @min(d.params[1], d.params[2]));
        d.params[3] = if (g.chance(50)) 0.0 else 0.5 * smallest;
    }

    // Sensors (static or kinematic) in the sensor layer, a body in a layer that does not collide with the soft body
    if (index > 0 and d.shape != .mesh and g.chance(15)) {
        d.is_sensor = 1;
        d.motion_type = g.int(0, 1);
    }
    d.object_layer = if (d.is_sensor != 0) Layers.sensor else if (g.chance(8)) Layers.lq_debris else if (d.motion_type == 0) Layers.non_moving else Layers.moving;

    // A collision group: the soft body's group 0 (sub group 0), another group or the invalid group, with or without the
    // group filter table
    if (g.chance(50)) {
        const r = g.int(0, 99);
        d.group = encodeGroup(if (r < 60) 0 else if (r < 85) 1 else 0xFF, g.int(0, 3), g.chance(80));
    }
    if (d.motion_type != 0) {
        d.linear_velocity = .{ g.float(-1, 1), g.float(-1, 1), g.float(-1, 1) };
        d.angular_velocity = .{ g.float(-1, 1), g.float(-1, 1), g.float(-1, 1) };
    }
    return d;
}

/// Joint matrices near the bind pose of genSettings' skinned cloth: the offsets of the inverse bind matrices, a small
/// rotation and a translation that moves with `phase`
fn appendJoints(allocator: Allocator, g: *Gen, c: *Case, phase: f32) !u32 {
    const first: u32 = @intCast(c.pool.items.len);
    for (0..3) |j| {
        const offset = Vec3.init(0.1 * @as(f32, @floatFromInt(j)), 0.05 * @as(f32, @floatFromInt(j)), 0);
        const m = Mat44.rotationTranslation(g.smallRotation(0.2), offset.add(Vec3.init(0.02 * phase, 0.05 * trig.sin(phase), g.float(-0.02, 0.02))));
        for (0..4) |col| {
            const v = m.getColumn4(@intCast(col));
            try c.pool.appendSlice(allocator, &.{ v.getX(), v.getY(), v.getZ(), v.getW() });
        }
    }
    return first;
}

/// A delta time
fn genDeltaTime(g: *Gen) f32 {
    return switch (g.int(0, 4)) {
        0 => 1.0 / 60.0,
        1 => 1.0 / 30.0,
        2 => 1.0 / 90.0,
        else => g.float(0.004, 0.04),
    };
}

/// A mass scale of listener mode 4: exactly zero ("infinite mass", no interaction) or in [0, 2)
fn genScale(g: *Gen) f32 {
    return if (g.chance(25)) 0.0 else g.float(0.0, 2.0);
}

/// Generates a case of `kind`
fn generate(allocator: Allocator, g: *Gen, kind: Kind) !Case {
    var c: Case = .{
        .kind = kind,
        .world = .{ .gravity = .{ g.float(-1, 1), g.float(-12, -2), g.float(-1, 1) }, .num_bodies = 0, .optimize_broad_phase = @intFromBool(g.chance(30)), .sim_shape_filter = @intFromBool(g.chance(30)), .group_filter_disabled = g.int(0, 63) },
        .soft_body = undefined,
    };
    errdefer c.deinit(allocator);

    // The soft body
    var settings = try genSettings(allocator, g, kind, &c);
    defer settings.deinit();
    const s = settings.get().?;
    c.settings_bytes = try saveBytes(allocator, s);
    c.num_vertices = @intCast(s.vertices.items.len);
    c.num_update_groups = @intCast(s.update_groups.items.len);
    if (s.update_groups.items.len >= 2) {
        const last = s.update_groups.items[s.update_groups.items.len - 1];
        const prev = s.update_groups.items[s.update_groups.items.len - 2];
        c.non_parallel_group_empty = std.meta.eql(last, prev);
    } else {
        c.non_parallel_group_empty = std.meta.eql(s.update_groups.items[0], std.mem.zeroes(S.UpdateGroup));
    }
    const rotation = if (g.chance(40)) g.smallRotation(0.5) else Quat.identity();
    c.soft_body = .{
        .position = .{ g.float(-0.2, 0.2), g.float(-0.1, 0.4), g.float(-0.2, 0.2) },
        .rotation = .{ rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getW() },
        .make_rotation_identity = @intFromBool(g.chance(70)),
        .object_layer = Layers.moving,
        .num_iterations = if (kind == .big_cloth) g.int(1, 3) else g.int(1, 8),
        .linear_damping = if (g.chance(50)) 0.1 else g.float(0.0, 2.0),
        .max_linear_velocity = if (g.chance(30)) g.float(0.5, 3.0) else 500.0,
        .restitution = g.float(0.0, 1.0),
        .friction = g.float(0.0, 1.0),
        .pressure = if (kind == .pressure_sphere) g.float(10.0, 2000.0) else if (g.chance(10)) g.float(0.0, 10.0) else 0.0,
        .gravity_factor = if (g.chance(70)) 1.0 else g.float(0.0, 2.0),
        .vertex_radius = if (g.chance(50)) 0.0 else g.float(0.0, 0.05),
        .update_position = @intFromBool(g.chance(70)),
        .faces_double_sided = @intFromBool(g.chance(50)),
        .allow_sleeping = @intFromBool(g.chance(50)),
        .add_to_system = @intFromBool(g.chance(50)),
        .group = if (g.chance(60)) encodeGroup(if (g.chance(80)) 0 else 0xFF, 0, g.chance(70)) else 0,
    };

    // The rigid bodies
    const num_bodies = g.int(1, if (kind == .big_cloth) 4 else max_bodies);
    for (0..num_bodies) |i|
        try c.bodies.append(allocator, try genBody(allocator, g, &c, i));
    c.world.num_bodies = num_bodies;

    // The operations
    const skinned = kind == .skinned_cloth;
    var phase: f32 = 0.0;
    if (g.chance(70))
        try c.ops.append(allocator, .{ .kind = .set_listener, .u = .{ g.int(1, 4), 0, 0, 0 }, .f = .{ genScale(g), genScale(g), genScale(g), 0, 0, 0, 0, 0 } });
    if (skinned) {
        // The skin states start as NaN: the first SkinVertices hard skins all vertices (like Jolt's samples), a second
        // one sometimes gives a valid previous position without hard skinning
        try c.ops.append(allocator, .{ .kind = .skin_vertices, .u = .{ try appendJoints(allocator, g, &c, phase), 3, 1, 0 } });
        if (g.chance(30))
            try c.ops.append(allocator, .{ .kind = .skin_vertices, .u = .{ try appendJoints(allocator, g, &c, phase), 3, 0, 0 } });
    }
    const num_ops = if (kind == .big_cloth) g.int(2, 5) else g.int(3, 14);
    for (0..num_ops) |_| {
        const r = g.int(0, 99);
        if (r < 50) {
            if (skinned and g.chance(60)) {
                phase += 0.3;
                try c.ops.append(allocator, .{ .kind = .skin_vertices, .u = .{ try appendJoints(allocator, g, &c, phase), 3, @intFromBool(g.chance(10)), 0 } });
            }
            try c.ops.append(allocator, .{ .kind = if (g.chance(35)) .staged_update else .custom_update, .f = .{ genDeltaTime(g), 0, 0, 0, 0, 0, 0, 0 } });
        } else if (r < 58) {
            try c.ops.append(allocator, .{ .kind = .set_options, .u = .{ g.int(1, 6), @intFromBool(g.chance(70)), @intFromBool(g.chance(50)), @intFromBool(g.chance(70)) }, .f = .{ if (kind == .pressure_sphere or g.chance(20)) g.float(0.0, 1000.0) else 0.0, if (g.chance(50)) 0.0 else g.float(0.0, 0.05), if (g.chance(50)) 1.0 else g.float(0.0, 2.0), g.float(0.0, 2.0), g.float(0.0, 1.0), 0, 0, 0 } });
        } else if (r < 64) {
            try c.ops.append(allocator, .{ .kind = .add_force, .f = .{ g.float(-50, 50), g.float(-50, 50), g.float(-50, 50), 0, 0, 0, 0, 0 } });
        } else if (r < 70) {
            if (c.soft_body.add_to_system == 0)
                try c.ops.append(allocator, .{ .kind = .save_update_restore, .f = .{ genDeltaTime(g), 0, 0, 0, 0, 0, 0, 0 } });
        } else if (r < 74) {
            try c.ops.append(allocator, .{ .kind = .calculate_mass_and_inertia });
        } else if (r < 82) {
            try c.ops.append(allocator, .{ .kind = .set_vertex, .u = .{ g.int(0, c.num_vertices - 1), 0, 0, 0 }, .f = .{ g.float(-2, 2), g.float(-2, 2), g.float(-2, 2), if (g.chance(20)) 0.0 else g.float(0.5, 2.0), 0, 0, 0, 0 } });
        } else if (r < 88) {
            try c.ops.append(allocator, .{ .kind = .set_listener, .u = .{ g.int(0, 4), 0, 0, 0 }, .f = .{ genScale(g), genScale(g), genScale(g), 0, 0, 0, 0, 0 } });
        } else if (r < 94) {
            try c.ops.append(allocator, .{ .kind = .set_body_velocity, .u = .{ g.int(0, num_bodies - 1), 0, 0, 0 }, .f = .{ g.float(-2, 2), g.float(-2, 2), g.float(-2, 2), g.float(-1, 1), g.float(-1, 1), g.float(-1, 1), 0, 0 } });
        } else {
            try c.ops.append(allocator, .{ .kind = .set_body_position, .u = .{ g.int(0, num_bodies - 1), 0, 0, 0 }, .f = .{ g.float(-0.8, 0.8), g.float(-0.6, 0.3), g.float(-0.8, 0.8), 0, 0, 0, 0, 0 } });
        }
    }
    try c.ops.append(allocator, .{ .kind = .custom_update, .f = .{ genDeltaTime(g), 0, 0, 0, 0, 0, 0, 0 } });
    return c;
}

// ---------------------------------------------------------------------------------------------------------------------
// Running and comparing

/// What the cases reached (counted on the Zolt side)
const Coverage = struct {
    cases: u32 = 0,
    updates: u32 = 0,
    skins: u32 = 0,
    restores: u32 = 0,
    validate_calls: u32 = 0,
    rejected: u32 = 0,
    sensor_validates: u32 = 0,
    added_calls: u32 = 0,
    vertex_contacts: u32 = 0,
    sensor_contacts: u32 = 0,
    pushed_bodies: u32 = 0,
    /// Staged updates: dynamic colliding shapes, vertices with a colliding shape and sensors with a contact after the
    /// collision phase, colliding shapes whose velocities the update changed
    staged_dynamic_shapes: u32 = 0,
    staged_vertex_hits: u32 = 0,
    staged_sensor_contacts: u32 = 0,
    staged_pushed: u32 = 0,
    /// Validated contacts without any interaction (zero mass scales)
    no_interaction: u32 = 0,
    /// Dynamic bodies pushed by an infinitely heavy soft body (soft body inverse mass scale 0)
    zero_scale_pushes: u32 = 0,
    /// Rigid bodies in range whose collision group does not collide with the soft body
    group_filtered: u32 = 0,
    /// Cases with more than 64 vertices that are not a multiple of 64 (a partial last batch)
    partial_batches: u32 = 0,
    /// Cases with several parallel groups and a non-empty non-parallel group
    parallel_and_serial_groups: u32 = 0,
    kinds: [@typeInfo(Kind).@"enum".fields.len]u32 = @splat(0),
    bytes: usize = 0,
};

/// Runs a case on both sides and compares the transcripts
fn runCase(allocator: Allocator, c: *const Case, index: usize, coverage: *Coverage, buffer: []u8) !void {
    var t: Transcript = .{ .allocator = allocator };
    defer t.deinit();
    try zoltRun(allocator, c, &t, coverage);

    const asserts_before = ParityHooks.assertCount();
    const size = jolt.jolt_sbu_run(&c.world, c.bodies.items.ptr, c.triangles.items.ptr, c.settings_bytes.ptr, @intCast(c.settings_bytes.len), &c.soft_body, c.ops.items.ptr, @intCast(c.ops.items.len), c.pool.items.ptr, buffer.ptr, @intCast(buffer.len));
    if (ParityHooks.assertCount() != asserts_before) {
        std.debug.print("SoftBodyUpdate parity: case {d} ({s}) hits a Jolt assert\n", .{ index, @tagName(c.kind) });
        return error.ReferenceAssert;
    }
    if (size == 0xFFFFFFFF) {
        std.debug.print("SoftBodyUpdate parity: case {d} ({s}): the C++ side failed (transcript too large or settings do not restore)\n", .{ index, @tagName(c.kind) });
        return error.ReferenceFailed;
    }
    const jolt_bytes = buffer[0..size];
    const zolt_bytes = t.bytes.items;
    if (!std.mem.eql(u8, zolt_bytes, jolt_bytes)) {
        reportMismatch(&t, c, index, jolt_bytes);
        return error.TranscriptMismatch;
    }

    coverage.cases += 1;
    coverage.bytes += size;
    coverage.kinds[@intFromEnum(c.kind)] += 1;
    if (c.num_vertices > 64 and c.num_vertices % 64 != 0) coverage.partial_batches += 1;
    if (c.num_update_groups > 2 and !c.non_parallel_group_empty) coverage.parallel_and_serial_groups += 1;
}

/// Prints the operation and the field of the first difference
fn reportMismatch(t: *const Transcript, c: *const Case, index: usize, jolt_bytes: []const u8) void {
    const zolt_bytes = t.bytes.items;
    const n = @min(zolt_bytes.len, jolt_bytes.len);
    var offset: usize = 0;
    while (offset < n and zolt_bytes[offset] == jolt_bytes[offset]) offset += 1;
    var found: ?Transcript.Label = null;
    for (t.labels.items) |l| {
        if (l.offset > offset) break;
        found = l;
    }
    std.debug.print("SoftBodyUpdate parity: case {d} ({s}, {d} vertices, {d} ops): the transcripts differ at byte {d} (Zolt {d} bytes, Jolt {d} bytes)\n", .{ index, @tagName(c.kind), c.num_vertices, c.ops.items.len, offset, zolt_bytes.len, jolt_bytes.len });
    if (found) |l| {
        const op_name = if (l.op == initial_op) "initial state" else if (l.op < c.ops.items.len) @tagName(c.ops.items[l.op].kind) else "?";
        std.debug.print("  after operation {d} ({s}), field '{s}' item {d} (starts at byte {d})\n", .{ l.op, op_name, l.name, l.item, l.offset });
        var o = l.offset;
        var words: u32 = 0;
        while (o + 4 <= n and words < 8) : ({
            o += 4;
            words += 1;
        }) {
            const zw = std.mem.readInt(u32, zolt_bytes[o..][0..4], .little);
            const jw = std.mem.readInt(u32, jolt_bytes[o..][0..4], .little);
            std.debug.print("  +{d}: Zolt 0x{x:0>8} ({e}) Jolt 0x{x:0>8} ({e}){s}\n", .{ o - l.offset, zw, @as(f32, @bitCast(zw)), jw, @as(f32, @bitCast(jw)), if (zw != jw) " <-" else "" });
        }
    }
}

var transcript_buffer: [transcript_capacity]u8 = undefined;

test "SoftBodyUpdate parity: struct sizes match the C++" {
    var sizes: [4]u32 = undefined;
    jolt.jolt_sbu_struct_sizes(&sizes);
    try std.testing.expectEqual([4]u32{ @sizeOf(World), @sizeOf(RigidBody), @sizeOf(SoftBody), @sizeOf(Op) }, sizes);
}

/// Number of random cases per kind
const cases_per_kind = [_]u32{ 40, 40, 40, 40, 40, 50, 4 };

test "SoftBodyUpdate parity: customUpdate and skinVertices of cubes, cloths, spheres, rods and skinned cloths" {
    const allocator = std.testing.allocator;
    var coverage: Coverage = .{};
    var g: Gen = .{ .rng = .{ .state = 0x50F7B0D4 } };
    inline for (@typeInfo(Kind).@"enum".fields, 0..) |field, k| {
        for (0..cases_per_kind[k]) |i| {
            var c = try generate(allocator, &g, @enumFromInt(field.value));
            defer c.deinit(allocator);
            try runCase(allocator, &c, i, &coverage, &transcript_buffer);
        }
    }

    std.debug.print("SoftBodyUpdate parity: {d} cases ({any} per kind), {d} updates, {d} skins, {d} restores, {d} validates ({d} rejected, {d} sensors, {d} without interaction), {d} contact callbacks, {d} vertex contacts, {d} sensor contacts, {d} pushed bodies ({d} by a soft body with mass scale 0), {d} bodies filtered by the collision group, {d} partial batches, {d} cases with parallel and serial groups, {d} transcript bytes\n", .{ coverage.cases, coverage.kinds, coverage.updates, coverage.skins, coverage.restores, coverage.validate_calls, coverage.rejected, coverage.sensor_validates, coverage.no_interaction, coverage.added_calls, coverage.vertex_contacts, coverage.sensor_contacts, coverage.pushed_bodies, coverage.zero_scale_pushes, coverage.group_filtered, coverage.partial_batches, coverage.parallel_and_serial_groups, coverage.bytes });
    std.debug.print("SoftBodyUpdate parity: staged updates: {d} dynamic colliding shapes, {d} vertices with a colliding shape and {d} sensors with a contact after the collision phase, {d} colliding shapes with new velocities\n", .{ coverage.staged_dynamic_shapes, coverage.staged_vertex_hits, coverage.staged_sensor_contacts, coverage.staged_pushed });

    // The cases reach the hard paths
    try expect(coverage.rejected > 0);
    try expect(coverage.sensor_validates > 0);
    try expect(coverage.vertex_contacts > 0);
    try expect(coverage.sensor_contacts > 0);
    try expect(coverage.pushed_bodies > 0);
    try expect(coverage.restores > 0);
    try expect(coverage.skins > 0);
    try expect(coverage.partial_batches > 0);
    try expect(coverage.parallel_and_serial_groups > 0);
    try expect(coverage.no_interaction > 0);
    try expect(coverage.zero_scale_pushes > 0);
    try expect(coverage.group_filtered > 0);
    try expect(coverage.staged_dynamic_shapes > 0);
    try expect(coverage.staged_vertex_hits > 0);
    try expect(coverage.staged_sensor_contacts > 0);
    try expect(coverage.staged_pushed > 0);
}
