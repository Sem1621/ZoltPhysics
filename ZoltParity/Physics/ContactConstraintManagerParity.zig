//! Parity tests for the ContactConstraintManager (Phase 5 Wave A4, SimulationArchitecture D10).
//!
//! Both sides drive their manager like PhysicsSystem does in a collision step, with the same calls in the same order
//! (single threaded, so the contact allocation order, the hash map handles and the callback order are the same):
//! PrepareConstraintBuffer (the constraint buffer is zeroed afterwards on both sides, so members that Jolt leaves
//! uninitialized compare equal) and the island builder's contact links; per body pair GetContactsFromCache, AddBodyPair
//! and AddContactConstraint with 0 to 3 random manifolds of 1 to 4 points (one of 4 contact allocators per pair, like
//! the FindCollisions jobs); OnCCDContactAdded on some pairs; the island finalize and per island
//! ConstraintIdxToConstraintOffset, SortContacts, GetAffectedBodies, WarmStartVelocityConstraints (CalculateSolverSteps
//! or the dummy), SolveVelocityConstraints, StoreAppliedImpulses and SolvePositionConstraints;
//! FinalizeContactCacheAndCallContactPointRemovedCallbacks and FinishConstraintBuffer. Between steps bodies move (less
//! than the body pair cache thresholds or more), change velocity and get forces (the restitution path compensates
//! them).
//!
//! Scenarios randomize the bodies (every motion type, sensors, allowed DOFs, friction, restitution, gravity factor,
//! velocity step overrides, far from the origin), the PhysicsSettings, the listener (none, recording, recording and
//! overriding the contact settings with a deterministic function of the bodies and sub shapes: mass / inertia scales,
//! friction, restitution, surface velocities, sensor), custom combine functions and the limits (small limits trigger
//! the contact constraints full, manifold cache full and body pair cache full soft errors). Contacts persist, are
//! removed and come back; manifolds are penetrating or speculative.
//!
//! Compared bit for bit after every call: the return values; the constraint buffer (the raw bytes of every contact
//! constraint with the body pointers replaced by the body IDs: normals, tangents, effective masses, lambdas, biases of
//! all parts), the constraint index to offset table and counter, the cached manifold of every constraint in the write
//! cache (key, chain, normal, flags, local contact points, after solving the stored impulses); the island contents,
//! the contact offsets before and after SortContacts and the solver step counts; the bodies (position, rotation,
//! velocities, activation, active body list); the listener callbacks in order (with the manifolds the cache
//! reconstructs and the contact settings they receive); the contact allocator counters and soft errors; the SaveState
//! bytes of the contact cache (with and without filters) after every step; RestoreState (round trips, validating,
//! restore filters, two parts, a cache that is too small); WereBodiesInContact for every pair; ValidateContactPoint.
//!
//! Known Jolt behavior the generator avoids, because Jolt reads uninitialized memory there (the results would be
//! garbage on both sides): a manifold whose contact constraint could not be created (contact constraints full) keeps
//! uninitialized impulses and is found again by the same key in the next step, and so is a manifold created by
//! OnCCDContactAdded (it has no impulses either). Scenarios with small limits use fresh sub shape keys every step and
//! no CCD contacts; CCD contacts use their own sub shape keys or the key of a manifold of the same step.
//!
//! C ABI wrappers: ZoltParity/Physics/ContactConstraintManagerReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const CalculateSolverSteps = zolt.CalculateSolverSteps;
const CollideShapeResult = zolt.CollideShapeResult;
const ContactConstraintManager = zolt.ContactConstraintManager;
const ContactListener = zolt.ContactListener;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const Core = zolt.Core;
const DummyCalculateSolverSteps = zolt.DummyCalculateSolverSteps;
const Float3 = zolt.Float3;
const IslandBuilder = zolt.IslandBuilder;
const ObjectLayer = zolt.ObjectLayer;
const PhysicsSettings = zolt.PhysicsSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsUpdateContext = zolt.PhysicsUpdateContext;
const Quat = zolt.Quat;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const StateRecorderFilter = zolt.StateRecorderFilter;
const StateRecorderImpl = zolt.StateRecorderImpl;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDPair = zolt.SubShapeIDPair;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const ValidateResult = zolt.ValidateResult;
const Vec3 = zolt.Vec3;

const CCM = ContactConstraintManager;
const ContactAllocator = CCM.ContactAllocator;

const io = std.testing.io;

/// Number of random scenarios
const num_scenarios = 240;

/// Capacity of a C++ output record (words)
const max_words = 1 << 18;

/// The C++ reference functions, see ContactConstraintManagerReference.cpp
const jolt = struct {
    extern fn jolt_ccm_world_create(config: *const CCMConfig) *anyopaque;
    extern fn jolt_ccm_world_destroy(world: *anyopaque) void;
    extern fn jolt_ccm_add_body(world: *anyopaque, desc: *const CCMBodyDesc) u32;
    extern fn jolt_ccm_move_body(world: *anyopaque, id: u32, position: *const [3]Real, rotation: *const [4]f32) void;
    extern fn jolt_ccm_set_velocity(world: *anyopaque, id: u32, linear: *const [3]f32, angular: *const [3]f32, force: *const [3]f32) void;
    extern fn jolt_ccm_begin_step(world: *anyopaque, delta_time: f32, warm_start_impulse_ratio: f32) void;
    extern fn jolt_ccm_process_pair(world: *anyopaque, id1: u32, id2: u32, manifolds: [*]const CCMManifold, num_manifolds: u32, use_cache: u32, allocator: u32) u32;
    extern fn jolt_ccm_ccd_contact(world: *anyopaque, id1: u32, id2: u32, manifold: *const CCMManifold, allocator: u32, out: [*]u32, capacity: u32, num: *u32) void;
    extern fn jolt_ccm_validate(world: *anyopaque, id1: u32, id2: u32, base_offset: *const [3]Real) u32;
    extern fn jolt_ccm_dump_constraints(world: *anyopaque, with_impulses: u32, out: [*]u32, capacity: u32, num: *u32) void;
    extern fn jolt_ccm_solve(world: *anyopaque, velocity_steps: u32, position_steps: u32, use_dummy: u32, out: [*]u32, capacity: u32, num: *u32) void;
    extern fn jolt_ccm_end_step(world: *anyopaque) void;
    extern fn jolt_ccm_allocator_info(world: *anyopaque, allocator: u32, out: *[4]u32) void;
    extern fn jolt_ccm_dump_bodies(world: *anyopaque, out: [*]u32, capacity: u32, num: *u32) void;
    extern fn jolt_ccm_records(world: *anyopaque, out: [*]u32, capacity: u32, num: *u32) void;
    extern fn jolt_ccm_were_in_contact(world: *anyopaque, id1: u32, id2: u32) u32;
    extern fn jolt_ccm_save(world: *anyopaque, filter_mode: u32, filter_body: u32, out: [*]u8, capacity: u32) u32;
    extern fn jolt_ccm_restore(world: *anyopaque, data: [*]const u8, size: u32, validating: u32, filter_mode: u32, filter_body: u32, last_part: u32) u32;
    extern fn jolt_ccm_sizes(out: *[4]u32) void;
};

/// Must match CCMConfig in ContactConstraintManagerReference.cpp
const CCMConfig = extern struct {
    gravity: [3]f32,
    baumgarte: f32,
    penetration_slop: f32,
    max_penetration_distance: f32,
    min_velocity_for_restitution: f32,
    body_pair_cache_max_delta_position_sq: f32,
    body_pair_cache_cos_max_delta_rotation_div2: f32,
    contact_point_preserve_lambda_max_dist_sq: f32,
    num_velocity_steps: u32,
    num_position_steps: u32,
    max_bodies: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    listener: u32,
    modifier_mode: u32,
    custom_combine: u32,
};

/// Must match CCMBodyDesc in ContactConstraintManagerReference.cpp
const CCMBodyDesc = extern struct {
    position: [3]Real,
    rotation: [4]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    half_extent: [3]f32,
    convex_radius: f32,
    friction: f32,
    restitution: f32,
    gravity_factor: f32,
    motion_type: u32,
    is_sensor: u32,
    allowed_dofs: u32,
    activate: u32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
};

/// Must match CCMManifold in ContactConstraintManagerReference.cpp
const CCMManifold = extern struct {
    base_offset: [3]Real,
    normal: [3]f32,
    penetration_depth: f32,
    sub_shape_id1: u32,
    sub_shape_id2: u32,
    num_points: u32,
    on1: [4][3]f32 = @splat(@splat(0)),
    on2: [4][3]f32 = @splat(@splat(0)),
};

fn load3(v: *const [3]f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn loadR3(v: *const [3]Real) RVec3 {
    return RVec3.init(v[0], v[1], v[2]);
}

fn loadQuat(v: *const [4]f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

/// The CCMRec of the C++ side: appends uint32 words
const Rec = struct {
    list: *std.ArrayList(u32),
    allocator: Allocator,

    fn init(list: *std.ArrayList(u32), allocator: Allocator) Rec {
        list.clearRetainingCapacity();
        return .{ .list = list, .allocator = allocator };
    }

    fn u(self: Rec, v: u32) void {
        self.list.append(self.allocator, v) catch @panic("out of memory");
    }

    fn f(self: Rec, v: f32) void {
        self.u(@bitCast(v));
    }

    fn r(self: Rec, v: Real) void {
        if (Real == f64) {
            const bits: u64 = @bitCast(v);
            self.u(@truncate(bits));
            self.u(@truncate(bits >> 32));
        } else self.f(v);
    }

    fn v3(self: Rec, v: Vec3) void {
        self.f(v.getX());
        self.f(v.getY());
        self.f(v.getZ());
    }

    fn f3(self: Rec, v: Float3) void {
        self.f(v.x);
        self.f(v.y);
        self.f(v.z);
    }

    fn r3(self: Rec, v: RVec3) void {
        self.r(v.getX());
        self.r(v.getY());
        self.r(v.getZ());
    }

    fn q(self: Rec, v: Quat) void {
        self.f(v.getX());
        self.f(v.getY());
        self.f(v.getZ());
        self.f(v.getW());
    }

    fn bytes(self: Rec, data: []const u8) void {
        var i: usize = 0;
        while (i < data.len) : (i += 4)
            self.u(std.mem.readInt(u32, data[i..][0..4], .little));
    }
};

fn settingsRecord(rec: Rec, settings: *const ContactSettings) void {
    rec.f(settings.combined_friction);
    rec.f(settings.combined_restitution);
    rec.f(settings.inv_mass_scale1);
    rec.f(settings.inv_inertia_scale1);
    rec.f(settings.inv_mass_scale2);
    rec.f(settings.inv_inertia_scale2);
    rec.u(@intFromBool(settings.is_sensor));
    rec.v3(settings.relative_linear_surface_velocity);
    rec.v3(settings.relative_angular_surface_velocity);
}

/// CCMModify of the C++ side: the deterministic override of the contact settings
fn modify(body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
    const k = (body1.getID().getIndex() *% 7 +% body2.getID().getIndex() *% 13 +% manifold.sub_shape_id1.getValue() *% 3 +% manifold.sub_shape_id2.getValue()) % 10;
    switch (k) {
        1 => {
            settings.inv_mass_scale1 = 0.5;
            settings.inv_inertia_scale1 = 0.25;
        },
        2 => {
            settings.inv_mass_scale2 = 2.0;
            settings.inv_inertia_scale2 = 0.0;
        },
        3 => {
            settings.inv_mass_scale1 = 0.0;
            settings.inv_mass_scale2 = 0.0;
        },
        4 => settings.combined_friction = 0.0,
        5 => settings.combined_restitution = 0.9,
        6 => {
            settings.relative_linear_surface_velocity = Vec3.init(0.5, 0.0, -0.25);
            settings.relative_angular_surface_velocity = Vec3.init(0.0, 1.0, 0.2);
        },
        7 => settings.combined_friction *= 2.0,
        8 => settings.is_sensor = true,
        9 => settings.inv_mass_scale1 = 0.0,
        else => {},
    }
}

/// CCMListener of the C++ side
const Listener = struct {
    words: std.ArrayList(u32) = .empty,
    allocator: Allocator,
    modifier_mode: u32 = 0,

    pub fn onContactValidate(self: *Listener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = .{ self, collision_result };
        return @enumFromInt((body1.getID().getIndex() + body2.getID().getIndex() + @as(u32, if (base_offset.getX() > 0.0) 1 else 0)) % 4);
    }

    pub fn onContactAdded(self: *Listener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(0, body1, body2, manifold, settings);
    }

    pub fn onContactPersisted(self: *Listener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(1, body1, body2, manifold, settings);
    }

    pub fn onContactRemoved(self: *Listener, sub_shape_pair: *const SubShapeIDPair) void {
        const rec: Rec = .{ .list = &self.words, .allocator = self.allocator };
        rec.u(2);
        rec.u(sub_shape_pair.getBody1ID().getIndexAndSequenceNumber());
        rec.u(sub_shape_pair.getBody2ID().getIndexAndSequenceNumber());
        rec.u(sub_shape_pair.getSubShapeID1().getValue());
        rec.u(sub_shape_pair.getSubShapeID2().getValue());
    }

    fn record(self: *Listener, kind: u32, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        const rec: Rec = .{ .list = &self.words, .allocator = self.allocator };
        rec.u(kind);
        rec.u(body1.getID().getIndexAndSequenceNumber());
        rec.u(body2.getID().getIndexAndSequenceNumber());
        rec.u(manifold.sub_shape_id1.getValue());
        rec.u(manifold.sub_shape_id2.getValue());
        rec.r3(manifold.base_offset);
        rec.v3(manifold.world_space_normal);
        rec.f(manifold.penetration_depth);
        rec.u(manifold.relative_contact_points_on1.len);
        for (manifold.relative_contact_points_on1.constSlice()) |v|
            rec.v3(v);
        for (manifold.relative_contact_points_on2.constSlice()) |v|
            rec.v3(v);
        settingsRecord(rec, settings);

        if (self.modifier_mode != 0)
            modify(body1, body2, manifold, settings);
    }
};

fn combineFriction(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
    _ = sub_shape_id1;
    return body1.getFriction() + body2.getFriction() * (if (sub_shape_id2.getValue() == 1) @as(f32, 0.5) else 1.0);
}

fn combineRestitution(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
    _ = .{ sub_shape_id1, sub_shape_id2 };
    return body1.getRestitution() * body2.getRestitution();
}

/// CCMFilter of the C++ side: mode 1 only the contacts of `body`, mode 2 all others
const Filter = struct {
    pub const overrides = .{ .shouldSaveContact, .shouldRestoreContact };

    base: StateRecorderFilter = .init(Filter),
    mode: u32,
    body: BodyID,

    fn accept(self: *const Filter, body1: BodyID, body2: BodyID) bool {
        const has_body = body1.eql(self.body) or body2.eql(self.body);
        return if (self.mode == 1) has_body else !has_body;
    }

    pub fn shouldSaveContact(self: *const Filter, body1: BodyID, body2: BodyID) bool {
        return self.accept(body1, body2);
    }

    pub fn shouldRestoreContact(self: *const Filter, body1: BodyID, body2: BodyID) bool {
        return self.accept(body1, body2);
    }
};

/// Broad phase layer = object layer % 2 (CCMLayers)
const Layers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(Layers),

    pub fn getNumBroadPhaseLayers(self: *const Layers) u32 {
        _ = self;
        return 2;
    }

    pub fn getBroadPhaseLayer(self: *const Layers, layer: ObjectLayer) BroadPhaseLayer {
        _ = self;
        return .init(@intCast(layer % 2));
    }
};

fn loadManifold(desc: *const CCMManifold) ContactManifold {
    var m: ContactManifold = .{};
    m.base_offset = loadR3(&desc.base_offset);
    m.world_space_normal = load3(&desc.normal);
    m.penetration_depth = desc.penetration_depth;
    m.sub_shape_id1.setValue(desc.sub_shape_id1);
    m.sub_shape_id2.setValue(desc.sub_shape_id2);
    m.relative_contact_points_on1.resize(desc.num_points);
    m.relative_contact_points_on2.resize(desc.num_points);
    for (0..desc.num_points) |i| {
        m.relative_contact_points_on1.slice()[i] = load3(&desc.on1[i]);
        m.relative_contact_points_on2.slice()[i] = load3(&desc.on2[i]);
    }
    return m;
}

/// CCMWorld of the C++ side
const World = struct {
    allocator: Allocator,
    layers: Layers = .{},
    settings: PhysicsSettings = .{},
    body_manager: BodyManager = .empty,
    island_builder: IslandBuilder = .empty,
    system: PhysicsSystem = .empty,
    manager: CCM = .empty,
    temp: TempAllocatorImpl = undefined,
    context: PhysicsUpdateContext = undefined,
    allocators: [4]ContactAllocator = undefined,
    listener: Listener,

    /// In place (the managers point into the world)
    fn init(self: *World, allocator: Allocator, config: *const CCMConfig) !void {
        self.* = .{ .allocator = allocator, .listener = .{ .allocator = allocator, .modifier_mode = config.modifier_mode } };
        self.settings.baumgarte = config.baumgarte;
        self.settings.penetration_slop = config.penetration_slop;
        self.settings.max_penetration_distance = config.max_penetration_distance;
        self.settings.min_velocity_for_restitution = config.min_velocity_for_restitution;
        self.settings.body_pair_cache_max_delta_position_sq = config.body_pair_cache_max_delta_position_sq;
        self.settings.body_pair_cache_cos_max_delta_rotation_div2 = config.body_pair_cache_cos_max_delta_rotation_div2;
        self.settings.contact_point_preserve_lambda_max_dist_sq = config.contact_point_preserve_lambda_max_dist_sq;
        self.settings.num_velocity_steps = config.num_velocity_steps;
        self.settings.num_position_steps = config.num_position_steps;

        self.temp = try .init(allocator, 64 * 1024 * 1024);
        errdefer self.temp.deinit();
        try self.body_manager.init(allocator, io, config.max_bodies, 0, &self.layers.base);
        errdefer self.body_manager.deinit();
        try self.island_builder.init(allocator, config.max_bodies);
        errdefer self.island_builder.deinit();
        self.system.setGravity(load3(&config.gravity));
        try self.manager.init(allocator, &self.settings, config.max_body_pairs, config.max_contact_constraints);
        if (config.listener != 0)
            self.manager.setContactListener(ContactListener.init(&self.listener));
        if (config.custom_combine != 0) {
            self.manager.setCombineFriction(combineFriction);
            self.manager.setCombineRestitution(combineRestitution);
        }
    }

    fn deinit(self: *World) void {
        // A failed comparison can leave a step open
        if (self.manager.update_context != null) {
            self.island_builder.resetIslands(self.temp.tempAllocator());
            self.manager.finishConstraintBuffer();
        }
        self.manager.deinit();
        self.island_builder.deinit();
        var ids: zolt.BodyIDVector = .empty;
        defer ids.deinit(self.allocator);
        self.body_manager.getBodyIDs(self.allocator, &ids) catch @panic("out of memory");
        for (ids.items) |id| {
            self.body_manager.deactivateBodies(&.{id});
            self.body_manager.getBodyMut(id).setInBroadPhaseInternal(false);
            self.body_manager.destroyBodies(&.{id});
        }
        self.body_manager.deinit();
        self.temp.deinit();
        self.listener.words.deinit(self.allocator);
    }

    fn getBody(self: *World, id: u32) *Body {
        return self.body_manager.getBodyMut(.init(id));
    }

    fn addBody(self: *World, desc: *const CCMBodyDesc) !u32 {
        const motion_type: zolt.MotionType = @enumFromInt(desc.motion_type);
        const shape = try BoxShape.create(self.allocator, load3(&desc.half_extent), .{ .convex_radius = desc.convex_radius });
        var settings = BodyCreationSettings.initWithShape(shape.asShape(), loadR3(&desc.position), loadQuat(&desc.rotation), motion_type, if (motion_type == .static) 0 else 1);
        defer settings.deinit();
        settings.linear_velocity = load3(&desc.linear_velocity);
        settings.angular_velocity = load3(&desc.angular_velocity);
        settings.friction = desc.friction;
        settings.restitution = desc.restitution;
        settings.gravity_factor = desc.gravity_factor;
        settings.is_sensor = desc.is_sensor != 0;
        settings.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
        settings.num_velocity_steps_override = desc.num_velocity_steps_override;
        settings.num_position_steps_override = desc.num_position_steps_override;
        const body = try self.body_manager.allocateBody(&settings);
        if (!self.body_manager.addBody(body)) @panic("out of bodies");
        body.setInBroadPhaseInternal(true);
        const id = body.getID();
        if (desc.activate != 0 and !body.isStatic())
            self.body_manager.activateBodies(&.{id});
        return id.getIndexAndSequenceNumber();
    }

    fn moveBody(self: *World, id: u32, position: *const [3]Real, rotation: *const [4]f32) void {
        self.getBody(id).setPositionAndRotationInternal(loadR3(position), loadQuat(rotation), .{});
    }

    fn setVelocity(self: *World, id: u32, linear: *const [3]f32, angular: *const [3]f32, force: *const [3]f32) void {
        const body = self.getBody(id);
        body.setLinearVelocity(load3(linear));
        body.setAngularVelocity(load3(angular));
        if (body.isDynamic())
            body.addForce(load3(force));
    }

    fn beginStep(self: *World, delta_time: f32, warm_start_impulse_ratio: f32) !void {
        self.context = .{
            .physics_system = &self.system,
            .temp_allocator = self.temp.tempAllocator(),
            .job_system = undefined,
            .barrier = undefined,
            .body_manager = &self.body_manager,
            .island_builder = &self.island_builder,
            .step_delta_time = delta_time,
            .warm_start_impulse_ratio = warm_start_impulse_ratio,
        };
        try self.manager.prepareConstraintBuffer(&self.context);
        @memset(self.manager.constraints.items, 0);
        try self.island_builder.prepareContactConstraints(self.manager.getMaxConstraints(), self.temp.tempAllocator());
        for (&self.allocators) |*a|
            a.* = self.manager.getContactAllocator();
    }

    fn processPair(self: *World, id1: u32, id2: u32, manifolds: []const CCMManifold, use_cache: bool, allocator_index: u32) u32 {
        const contact_allocator = &self.allocators[allocator_index];
        const body1 = self.getBody(id1);
        const body2 = self.getBody(id2);
        if (use_cache and self.manager.getContactsFromCache(contact_allocator, body1, body2))
            return 1;
        const handle = self.manager.addBodyPair(contact_allocator, body1, body2) orelse return 2;
        var link_and_activate = true;
        for (manifolds) |*desc| {
            const manifold = loadManifold(desc);
            self.manager.addContactConstraint(contact_allocator, &link_and_activate, handle, body1, body2, &manifold);
        }
        return if (link_and_activate) 4 else 0;
    }

    fn ccdContact(self: *World, id1: u32, id2: u32, desc: *const CCMManifold, allocator_index: u32, rec: Rec) void {
        const manifold = loadManifold(desc);
        var settings: ContactSettings = .{};
        self.manager.onCCDContactAdded(&self.allocators[allocator_index], self.getBody(id1), self.getBody(id2), &manifold, &settings);
        settingsRecord(rec, &settings);
    }

    fn validate(self: *World, id1: u32, id2: u32, base_offset: *const [3]Real) u32 {
        const result: CollideShapeResult = .{};
        return @intCast(@intFromEnum(self.manager.validateContactPoint(self.getBody(id1), self.getBody(id2), loadR3(base_offset), &result)));
    }

    fn dumpConstraints(self: *World, with_impulses: bool, rec: Rec) void {
        const m = &self.manager;
        const counter = m.num_constraints_and_next_constraint_offset.load(.monotonic);
        rec.u(@truncate(counter));
        rec.u(@truncate(counter >> 32));
        const num_constraints = m.getNumConstraints();
        rec.u(num_constraints);
        const buffer_size = m.getMaxConstraints() * CCM.max_constraint_size;
        for (0..num_constraints) |i| {
            const offset = m.constraint_idx_to_offset.items[i];
            const end = if (i + 1 < num_constraints) m.constraint_idx_to_offset.items[i + 1] else @min(offset + CCM.max_constraint_size, buffer_size);
            rec.u(offset);
            rec.u(end - offset);
            const c = m.constraints.items[offset..end];
            const base: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(c.ptr));
            rec.u(base.body1.getID().getIndexAndSequenceNumber());
            rec.u(0);
            rec.u(base.body2.getID().getIndexAndSequenceNumber());
            rec.u(0);
            rec.bytes(c[16..]);

            // The cached manifold
            const kv = m.write_cache.?.fromHandle(base.cached_manifold_handle);
            rec.bytes(std.mem.asBytes(kv.getKey()));
            const cm = kv.getValueConst();
            rec.u(cm.next_with_same_body_pair);
            rec.f3(cm.contact_normal);
            rec.u(cm.flags.load(.seq_cst));
            rec.u(cm.num_contact_points);
            for (cm.getContactPointsConst()[0..base.num_contact_points]) |*cp| {
                rec.f3(cp.position1);
                rec.f3(cp.position2);
                if (with_impulses)
                    rec.f(cp.non_penetration_lambda);
            }
            if (with_impulses) {
                rec.f(cm.friction_lambda.f32s[0]);
                rec.f(cm.friction_lambda.f32s[1]);
                rec.f(cm.angular_friction_lambda);
            }
        }
    }

    fn solve(self: *World, velocity_steps: u32, position_steps: u32, use_dummy: bool, rec: Rec) !void {
        const m = &self.manager;
        const ib = &self.island_builder;
        const bm = &self.body_manager;
        const num_active = bm.getNumActiveBodies(.rigid_body);
        try ib.finalize(bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active], m.getNumConstraints(), self.temp.tempAllocator());
        rec.u(ib.getNumIslands());
        var island: u32 = 0;
        while (island < ib.getNumIslands()) : (island += 1) {
            const bodies = ib.getBodiesInIsland(island);
            rec.u(@intCast(bodies.len));
            for (bodies) |b|
                rec.u(b.getIndexAndSequenceNumber());

            const contacts = ib.getContactsInIsland(island);
            if (contacts.len == 0) {
                rec.u(0);
                continue;
            }
            rec.u(@intCast(contacts.len));
            m.constraintIdxToConstraintOffset(contacts);
            for (contacts) |c|
                rec.u(c);
            m.sortContacts(contacts);
            for (contacts) |c|
                rec.u(c);
            for (contacts) |c| {
                const affected = m.getAffectedBodies(c);
                rec.u(affected.body1.getID().getIndexAndSequenceNumber());
                rec.u(affected.body2.getID().getIndexAndSequenceNumber());
            }
            if (use_dummy) {
                const dummy: DummyCalculateSolverSteps = .{};
                m.warmStartVelocityConstraints(contacts, self.context.warm_start_impulse_ratio, &dummy);
            } else {
                var steps_calculator = CalculateSolverSteps.init(&self.settings);
                m.warmStartVelocityConstraints(contacts, self.context.warm_start_impulse_ratio, &steps_calculator);
                steps_calculator.finalize();
                rec.u(steps_calculator.getNumVelocitySteps());
                rec.u(steps_calculator.getNumPositionSteps());
            }
            for (0..velocity_steps) |_|
                rec.u(@intFromBool(m.solveVelocityConstraints(contacts)));
            m.storeAppliedImpulses(contacts);
            for (0..position_steps) |_|
                rec.u(@intFromBool(m.solvePositionConstraints(contacts)));
        }
    }

    fn endStep(self: *World) void {
        var num_body_pairs: u32 = 0;
        var num_manifolds: u32 = 0;
        for (self.allocators) |a| {
            num_body_pairs += a.num_body_pairs;
            num_manifolds += a.num_manifolds;
        }
        self.manager.finalizeContactCacheAndCallContactPointRemovedCallbacks(num_body_pairs, num_manifolds);
        self.island_builder.resetIslands(self.temp.tempAllocator());
        self.manager.finishConstraintBuffer();
    }

    fn allocatorInfo(self: *World, index: u32) [4]u32 {
        const a = &self.allocators[index];
        return .{ a.num_body_pairs, a.num_manifolds, a.errors.toInt(), self.manager.getNumConstraints() };
    }

    fn dumpBodies(self: *World, rec: Rec) void {
        const bm = &self.body_manager;
        for (bm.getBodies()) |slot| {
            if (BodyManager.isValidBodyPointer(slot)) {
                const body = slot.getBody();
                rec.u(body.getID().getIndexAndSequenceNumber());
                rec.r3(body.getCenterOfMassPosition());
                rec.q(body.getRotation());
                rec.v3(body.getLinearVelocity());
                rec.v3(body.getAngularVelocity());
                rec.u(@intFromBool(body.isActive()));
            }
        }
        const num_active = bm.getNumActiveBodies(.rigid_body);
        rec.u(num_active);
        for (bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active]) |id|
            rec.u(id.getIndexAndSequenceNumber());
    }

    fn records(self: *World, rec: Rec) void {
        for (self.listener.words.items) |v|
            rec.u(v);
        self.listener.words.clearRetainingCapacity();
    }

    fn save(self: *World, filter_mode: u32, filter_body: u32, out: *std.ArrayList(u8)) !void {
        var recorder: StateRecorderImpl = .init(self.allocator);
        defer recorder.deinit();
        const filter: Filter = .{ .mode = filter_mode, .body = .init(filter_body) };
        try self.manager.saveState(recorder.recorder(), if (filter_mode != 0) &filter.base else null);
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    fn restore(self: *World, data: []const u8, validating: bool, filter_mode: u32, filter_body: u32, last_part: bool) !u32 {
        var recorder: StateRecorderImpl = .init(self.allocator);
        defer recorder.deinit();
        recorder.writeBytes(data);
        recorder.base.setValidating(validating);
        recorder.base.setIsLastPart(last_part);
        const filter: Filter = .{ .mode = filter_mode, .body = .init(filter_body) };
        const success = try self.manager.restoreState(recorder.recorder(), if (filter_mode != 0) &filter.base else null);
        return @as(u32, @intFromBool(success)) | (@as(u32, @intFromBool(recorder.isFailed())) << 1);
    }
};

/// Branches that only some inputs reach (each must be reached `min_hits` times)
const Coverage = struct {
    const min_hits = 10;
    const Kind = enum {
        cache_handled,
        body_pair_full_handle,
        added,
        persisted,
        removed,
        ccd_persisted,
        contact_constraints_full,
        manifold_cache_full,
        body_pair_cache_full,
        validating_restore,
        two_part_restore,
        filtered_restore,
        dynamic_vs_static,
        dynamic_vs_kinematic,
        dynamic_vs_dynamic,
        static_vs_dynamic,
        kinematic_vs_dynamic,
        no_constraint_pair,
        multi_island,
    };
    hits: [@typeInfo(Kind).@"enum".fields.len]u32 = @splat(0),

    fn hit(self: *Coverage, kind: Kind) void {
        self.hits[@intFromEnum(kind)] += 1;
    }

    fn check(self: *const Coverage) !void {
        var ok = true;
        for (self.hits, 0..) |h, i| {
            if (h < min_hits) {
                std.debug.print("ContactConstraintManagerParity: coverage of {s} is {d} < {d}\n", .{ @tagName(@as(Kind, @enumFromInt(i))), h, min_hits });
                ok = false;
            }
        }
        if (!ok) return error.CoverageNotReached;
    }
};

/// Both sides of a scenario and the comparison of their records
const Pair = struct {
    allocator: Allocator,
    zolt_world: World = undefined,
    jolt_world: *anyopaque = undefined,
    zolt_words: std.ArrayList(u32) = .empty,
    jolt_words: []u32 = &.{},
    jolt_num: u32 = 0,
    zolt_bytes: std.ArrayList(u8) = .empty,
    jolt_bytes: []u8 = &.{},
    scenario: usize = 0,
    frame: usize = 0,

    fn init(self: *Pair, allocator: Allocator, config: *const CCMConfig) !void {
        self.* = .{ .allocator = allocator };
        try self.zolt_world.init(allocator, config);
        errdefer self.zolt_world.deinit();
        self.jolt_words = try allocator.alloc(u32, max_words);
        errdefer allocator.free(self.jolt_words);
        self.jolt_bytes = try allocator.alloc(u8, 1 << 20);
        self.jolt_world = jolt.jolt_ccm_world_create(config);
    }

    fn deinit(self: *Pair) void {
        jolt.jolt_ccm_world_destroy(self.jolt_world);
        self.allocator.free(self.jolt_bytes);
        self.allocator.free(self.jolt_words);
        self.zolt_words.deinit(self.allocator);
        self.zolt_bytes.deinit(self.allocator);
        self.zolt_world.deinit();
    }

    fn rec(self: *Pair) Rec {
        return .init(&self.zolt_words, self.allocator);
    }

    /// Compare the record of the last call on both sides
    fn compare(self: *Pair, what: []const u8) !void {
        if (self.jolt_num > max_words) {
            std.debug.print("ContactConstraintManagerParity: {s}: the C++ record does not fit ({d} words)\n", .{ what, self.jolt_num });
            return error.ParityMismatch;
        }
        const z = self.zolt_words.items;
        const j = self.jolt_words[0..self.jolt_num];
        if (std.mem.eql(u32, z, j))
            return;
        var first: usize = @min(z.len, j.len);
        for (z[0..@min(z.len, j.len)], j[0..@min(z.len, j.len)], 0..) |a, b, i| {
            if (a != b) {
                first = i;
                break;
            }
        }
        std.debug.print("ContactConstraintManagerParity: scenario {d} frame {d}: {s} differs: {d} zolt words, {d} jolt words, first difference at {d}", .{ self.scenario, self.frame, what, z.len, j.len, first });
        if (first < z.len and first < j.len)
            std.debug.print(": zolt 0x{x:0>8} jolt 0x{x:0>8}", .{ z[first], j[first] });
        std.debug.print("\n  zolt: {any}\n  jolt: {any}\n", .{ z[first -| 8..@min(z.len, first + 8)], j[first -| 8..@min(j.len, first + 8)] });
        return error.ParityMismatch;
    }

    fn compareValues(self: *Pair, what: []const u8, zolt_value: anytype, jolt_value: @TypeOf(zolt_value)) !void {
        if (std.meta.eql(zolt_value, jolt_value))
            return;
        std.debug.print("ContactConstraintManagerParity: scenario {d} frame {d}: {s} differs: zolt {any} jolt {any}\n", .{ self.scenario, self.frame, what, zolt_value, jolt_value });
        return error.ParityMismatch;
    }

    fn compareBytes(self: *Pair, what: []const u8, jolt_size: u32) !void {
        if (jolt_size > self.jolt_bytes.len) return error.ParityMismatch;
        if (std.mem.eql(u8, self.zolt_bytes.items, self.jolt_bytes[0..jolt_size]))
            return;
        std.debug.print("ContactConstraintManagerParity: scenario {d} frame {d}: {s} differs: {d} zolt bytes, {d} jolt bytes\n", .{ self.scenario, self.frame, what, self.zolt_bytes.items.len, jolt_size });
        return error.ParityMismatch;
    }

    fn compareRecords(self: *Pair, coverage: *Coverage) !void {
        const r = self.rec();
        self.zolt_world.records(r);
        jolt.jolt_ccm_records(self.jolt_world, self.jolt_words.ptr, max_words, &self.jolt_num);
        try self.compare("listener records");

        // Count the callbacks
        const words = self.zolt_words.items;
        var i: usize = 0;
        while (i < words.len) {
            switch (words[i]) {
                0, 1 => {
                    coverage.hit(if (words[i] == 0) .added else .persisted);
                    const num_points = words[i + 5 + 3 * @sizeOf(Real) / 4 + 4];
                    i += 5 + 3 * @sizeOf(Real) / 4 + 4 + 1 + 6 * num_points + 13;
                },
                else => {
                    coverage.hit(.removed);
                    i += 5;
                },
            }
        }
    }

    fn compareBodies(self: *Pair) !void {
        self.zolt_world.dumpBodies(self.rec());
        jolt.jolt_ccm_dump_bodies(self.jolt_world, self.jolt_words.ptr, max_words, &self.jolt_num);
        try self.compare("bodies");
    }

    fn compareConstraints(self: *Pair, with_impulses: bool, coverage: *Coverage) !void {
        self.zolt_world.dumpConstraints(with_impulses, self.rec());
        jolt.jolt_ccm_dump_constraints(self.jolt_world, @intFromBool(with_impulses), self.jolt_words.ptr, max_words, &self.jolt_num);
        try self.compare(if (with_impulses) "constraints after solving" else "constraints");

        if (!with_impulses) {
            const m = &self.zolt_world.manager;
            for (0..m.getNumConstraints()) |i| {
                const base: *const CCM.ContactConstraintBase = @ptrCast(@alignCast(m.constraints.items.ptr + m.constraint_idx_to_offset.items[i]));
                coverage.hit(switch (base.body1.getMotionType()) {
                    .static => .static_vs_dynamic,
                    .kinematic => .kinematic_vs_dynamic,
                    .dynamic => switch (base.body2.getMotionType()) {
                        .static => .dynamic_vs_static,
                        .kinematic => .dynamic_vs_kinematic,
                        .dynamic => .dynamic_vs_dynamic,
                    },
                });
            }
        }
    }

    fn compareSave(self: *Pair, filter_mode: u32, filter_body: u32) !void {
        try self.zolt_world.save(filter_mode, filter_body, &self.zolt_bytes);
        const size = jolt.jolt_ccm_save(self.jolt_world, filter_mode, filter_body, self.jolt_bytes.ptr, @intCast(self.jolt_bytes.len));
        try self.compareBytes(if (filter_mode == 0) "SaveState" else "SaveState with a filter", size);
    }

    /// Both sides restore the same bytes (`self.zolt_bytes`)
    fn restoreBoth(self: *Pair, validating: bool, filter_mode: u32, filter_body: u32, last_part: bool) !u32 {
        const data = try self.allocator.dupe(u8, self.zolt_bytes.items);
        defer self.allocator.free(data);
        const z = try self.zolt_world.restore(data, validating, filter_mode, filter_body, last_part);
        const j = jolt.jolt_ccm_restore(self.jolt_world, data.ptr, @intCast(data.len), @intFromBool(validating), filter_mode, filter_body, @intFromBool(last_part));
        try self.compareValues("RestoreState", z, j);
        return z;
    }
};

fn randomUnit(rng: *fw.Rng) Vec3 {
    while (true) {
        const v = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
        if (v.lengthSq() > 0.01)
            return v.normalized();
    }
}

fn randomRotation(rng: *fw.Rng) [4]f32 {
    while (true) {
        const v = zolt.Vec4.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
        if (v.lengthSq() > 0.01) {
            const n = v.normalized();
            return .{ n.getX(), n.getY(), n.getZ(), n.getW() };
        }
    }
}

fn store3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn storeR3(v: RVec3) [3]Real {
    return .{ v.getX(), v.getY(), v.getZ() };
}

/// A random manifold between body 1 and body 2 (in that order, base offset at the center of mass of body 1)
fn randomManifold(rng: *fw.Rng, p: *Pair, id1: u32, id2: u32, sub_shape_id1: u32, sub_shape_id2: u32) CCMManifold {
    const body1 = p.zolt_world.getBody(id1);
    const body2 = p.zolt_world.getBody(id2);
    const base_offset = body1.getCenterOfMassPosition();
    const middle = toVec3(body2.getCenterOfMassPosition().sub(base_offset)).mulScalar(0.5);
    const normal = randomUnit(rng);
    const penetration: f32 = if (rng.intRange(u32, 0, 3) == 0) rng.float(-0.05, 0.0) else rng.float(0.0, 0.1);
    var m: CCMManifold = .{
        .base_offset = storeR3(base_offset),
        .normal = store3(normal),
        .penetration_depth = penetration,
        .sub_shape_id1 = sub_shape_id1,
        .sub_shape_id2 = sub_shape_id2,
        .num_points = rng.intRange(u32, 1, 4),
    };
    for (0..m.num_points) |i| {
        const on1 = middle.add(Vec3.init(rng.float(-0.8, 0.8), rng.float(-0.8, 0.8), rng.float(-0.8, 0.8)));
        const on2 = on1.sub(normal.mulScalar(penetration));
        m.on1[i] = store3(on1);
        m.on2[i] = store3(on2);
    }
    return m;
}

fn toVec3(v: RVec3) Vec3 {
    return if (Core.double_precision) v.toVec3() else v;
}

fn runScenario(allocator: Allocator, rng: *fw.Rng, scenario_index: usize, coverage: *Coverage) !void {
    const small_limits = scenario_index % 5 == 4;
    const listener_mode = rng.intRange(u32, 0, 4); // 0: none, 1-2: recording, 3-4: recording and overriding
    const config: CCMConfig = .{
        .gravity = .{ rng.float(-2, 2), rng.float(-15, -5), rng.float(-2, 2) },
        .baumgarte = rng.float(0.1, 0.5),
        .penetration_slop = rng.float(0.0, 0.04),
        .max_penetration_distance = rng.float(0.05, 0.3),
        .min_velocity_for_restitution = rng.float(0.1, 2.0),
        .body_pair_cache_max_delta_position_sq = if (rng.intRange(u32, 0, 1) == 0) 1.0e-6 else rng.float(1.0e-7, 1.0e-4),
        .body_pair_cache_cos_max_delta_rotation_div2 = if (rng.intRange(u32, 0, 1) == 0) 0.99984769515639123915701155881391 else rng.float(0.99, 0.9999),
        .contact_point_preserve_lambda_max_dist_sq = if (rng.intRange(u32, 0, 1) == 0) 1.0e-4 else rng.float(1.0e-5, 1.0e-2),
        .num_velocity_steps = rng.intRange(u32, 1, 12),
        .num_position_steps = rng.intRange(u32, 1, 4),
        .max_bodies = 64,
        .max_body_pairs = if (small_limits) rng.intRange(u32, 0, 12) else 256,
        .max_contact_constraints = if (small_limits) rng.intRange(u32, 0, 12) else 512,
        .listener = @intFromBool(listener_mode != 0),
        .modifier_mode = @intFromBool(listener_mode >= 3),
        .custom_combine = @intFromBool(rng.intRange(u32, 0, 3) == 0),
    };

    var p: Pair = .{ .allocator = allocator };
    try p.init(allocator, &config);
    defer p.deinit();
    p.scenario = scenario_index;

    // The bodies, in a cluster (far from the origin in some scenarios)
    const origin: [3]Real = if (rng.intRange(u32, 0, 2) == 0) .{ rng.float(-2000, 2000), rng.float(-100, 100), rng.float(-2000, 2000) } else .{ 0, 0, 0 };
    const num_bodies = rng.intRange(u32, 3, 10);
    var ids: [16]u32 = undefined;
    var sensors: [16]bool = undefined;
    var types: [16]zolt.MotionType = undefined;
    for (0..num_bodies) |i| {
        const motion_type: zolt.MotionType = switch (rng.intRange(u32, 0, 9)) {
            0, 1 => .static,
            2, 3 => .kinematic,
            else => .dynamic,
        };
        const is_sensor = rng.intRange(u32, 0, 9) == 0;
        const allowed_dofs: u32 = switch (rng.intRange(u32, 0, 5)) {
            0 => zolt.AllowedDOFs.plane_2d.toInt(),
            1 => 0b010111, // Translation + rotation around Y
            else => zolt.AllowedDOFs.all.toInt(),
        };
        const desc: CCMBodyDesc = .{
            .position = .{ origin[0] + @as(Real, rng.float(-3, 3)), origin[1] + @as(Real, rng.float(-3, 3)), origin[2] + @as(Real, rng.float(-3, 3)) },
            .rotation = randomRotation(rng),
            .linear_velocity = if (motion_type != .static) .{ rng.float(-3, 3), rng.float(-3, 3), rng.float(-3, 3) } else .{ 0, 0, 0 },
            .angular_velocity = if (motion_type != .static) .{ rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2) } else .{ 0, 0, 0 },
            .half_extent = .{ rng.float(0.2, 2), rng.float(0.2, 2), rng.float(0.2, 2) },
            .convex_radius = rng.float(0.0, 0.1),
            .friction = if (rng.intRange(u32, 0, 4) == 0) 0.0 else rng.float(0.0, 1.0),
            .restitution = if (rng.intRange(u32, 0, 1) == 0) 0.0 else rng.float(0.0, 1.0),
            .gravity_factor = rng.float(0.0, 2.0),
            .motion_type = @intFromEnum(motion_type),
            .is_sensor = @intFromBool(is_sensor),
            .allowed_dofs = allowed_dofs,
            .activate = @intFromBool(rng.intRange(u32, 0, 3) != 0),
            .num_velocity_steps_override = if (rng.intRange(u32, 0, 3) == 0) rng.intRange(u32, 1, 15) else 0,
            .num_position_steps_override = if (rng.intRange(u32, 0, 3) == 0) rng.intRange(u32, 1, 5) else 0,
        };
        const z = try p.zolt_world.addBody(&desc);
        const j = jolt.jolt_ccm_add_body(p.jolt_world, &desc);
        try p.compareValues("body id", z, j);
        ids[i] = z;
        sensors[i] = is_sensor;
        types[i] = motion_type;
    }
    try p.compareBodies();

    const num_frames = rng.intRange(u32, 4, 9);
    for (0..num_frames) |frame| {
        p.frame = frame;

        // Move bodies (a little: the body pair cache stays valid, or more), change velocities and add forces
        if (frame > 0) {
            for (ids[0..num_bodies], 0..) |id, i| {
                const body = p.zolt_world.getBody(id);
                switch (rng.intRange(u32, 0, 9)) {
                    0, 1 => {
                        const small = rng.intRange(u32, 0, 1) == 0;
                        const d: f32 = if (small) 0.0003 else 0.2;
                        const position = storeR3(body.getPosition().add(RVec3.init(rng.float(-d, d), rng.float(-d, d), rng.float(-d, d))));
                        var rotation: [4]f32 = undefined;
                        if (small) {
                            const r = body.getRotation();
                            rotation = .{ r.getX(), r.getY(), r.getZ(), r.getW() };
                        } else rotation = randomRotation(rng);
                        p.zolt_world.moveBody(id, &position, &rotation);
                        jolt.jolt_ccm_move_body(p.jolt_world, id, &position, &rotation);
                    },
                    2 => if (types[i] != .static) {
                        const linear: [3]f32 = .{ rng.float(-4, 4), rng.float(-4, 4), rng.float(-4, 4) };
                        const angular: [3]f32 = .{ rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2) };
                        const force: [3]f32 = .{ rng.float(-100, 100), rng.float(-100, 100), rng.float(-100, 100) };
                        p.zolt_world.setVelocity(id, &linear, &angular, &force);
                        jolt.jolt_ccm_set_velocity(p.jolt_world, id, &linear, &angular, &force);
                    },
                    else => {},
                }
            }
            try p.compareBodies();
        }

        // PrepareConstraintBuffer and the contact links
        const delta_time = rng.float(1.0 / 120.0, 1.0 / 30.0);
        const ratio: f32 = if (frame == 0) 0.0 else rng.float(0.0, 2.0);
        try p.zolt_world.beginStep(delta_time, ratio);
        jolt.jolt_ccm_begin_step(p.jolt_world, delta_time, ratio);

        // The body pairs (not static vs static), each processed by one of the contact allocators
        var ccd_candidates: [64]struct { id1: u32, id2: u32, manifold: CCMManifold } = undefined;
        var num_ccd_candidates: usize = 0;
        for (0..num_bodies) |a| {
            for (a + 1..num_bodies) |b| {
                if (types[a] == .static and types[b] == .static) continue;
                if (rng.intRange(u32, 0, 9) < 4) continue;
                const swap = rng.intRange(u32, 0, 1) == 0;
                const id1 = if (swap) ids[b] else ids[a];
                const id2 = if (swap) ids[a] else ids[b];
                const allocator_index = rng.intRange(u32, 0, 3);
                const use_cache = rng.intRange(u32, 0, 9) < 7;

                // Manifolds with distinct sub shape pairs (fresh keys every frame with small limits)
                var manifolds: [3]CCMManifold = undefined;
                const num_manifolds = rng.intRange(u32, 0, 3);
                const key_offset: u32 = if (small_limits) @intCast(frame * 10) else 0;
                var used: [9]bool = @splat(false);
                for (manifolds[0..num_manifolds]) |*m| {
                    var key = rng.intRange(u32, 0, 8);
                    while (used[key]) key = (key + 1) % 9;
                    used[key] = true;
                    m.* = randomManifold(rng, &p, id1, id2, key_offset + key / 3, key_offset + key % 3);
                }

                const z = p.zolt_world.processPair(id1, id2, manifolds[0..num_manifolds], use_cache, allocator_index);
                const j = jolt.jolt_ccm_process_pair(p.jolt_world, id1, id2, &manifolds, num_manifolds, @intFromBool(use_cache), allocator_index);
                try p.compareValues("ProcessPair", z, j);
                if (z == 1) coverage.hit(.cache_handled);
                if (z == 2) coverage.hit(.body_pair_full_handle);
                if (z == 4 and num_manifolds > 0) coverage.hit(.no_constraint_pair);

                if (!small_limits and num_manifolds > 0 and !sensors[a] and !sensors[b] and num_ccd_candidates < ccd_candidates.len) {
                    ccd_candidates[num_ccd_candidates] = .{ .id1 = id1, .id2 = id2, .manifold = manifolds[0] };
                    num_ccd_candidates += 1;
                }
            }
        }
        try p.compareRecords(coverage);

        // CCD contacts: on the key of a manifold of this step, or on their own keys (twice: the second one persists)
        if (!small_limits) {
            for (ccd_candidates[0..num_ccd_candidates]) |*c| {
                if (rng.intRange(u32, 0, 2) != 0) continue;
                var m = c.manifold;
                const own_key = rng.intRange(u32, 0, 1) == 0;
                if (own_key) {
                    m.sub_shape_id1 = 100 + rng.intRange(u32, 0, 1);
                    m.sub_shape_id2 = 100;
                }
                const swap = rng.intRange(u32, 0, 1) == 0;
                const times = rng.intRange(u32, 1, 2);
                for (0..times) |_| {
                    const allocator_index = rng.intRange(u32, 0, 3);
                    if (swap)
                        p.zolt_world.ccdContact(c.id2, c.id1, &m, allocator_index, p.rec())
                    else
                        p.zolt_world.ccdContact(c.id1, c.id2, &m, allocator_index, p.rec());
                    if (swap)
                        jolt.jolt_ccm_ccd_contact(p.jolt_world, c.id2, c.id1, &m, allocator_index, p.jolt_words.ptr, max_words, &p.jolt_num)
                    else
                        jolt.jolt_ccm_ccd_contact(p.jolt_world, c.id1, c.id2, &m, allocator_index, p.jolt_words.ptr, max_words, &p.jolt_num);
                    try p.compare("OnCCDContactAdded settings");
                }
                if (own_key and times == 2 and p.zolt_world.manager.getContactListener() != null) coverage.hit(.ccd_persisted);
            }
            try p.compareRecords(coverage);
        }

        // ValidateContactPoint
        for (0..3) |_| {
            const a = rng.intRange(u32, 0, num_bodies - 1);
            const b = rng.intRange(u32, 0, num_bodies - 1);
            const base_offset: [3]Real = .{ rng.float(-1, 1), 0, 0 };
            try p.compareValues("ValidateContactPoint", p.zolt_world.validate(ids[a], ids[b], &base_offset), jolt.jolt_ccm_validate(p.jolt_world, ids[a], ids[b], &base_offset));
        }

        // The contact allocators
        for (0..4) |i| {
            var info: [4]u32 = undefined;
            jolt.jolt_ccm_allocator_info(p.jolt_world, @intCast(i), &info);
            const z = p.zolt_world.allocatorInfo(@intCast(i));
            try p.compareValues("contact allocator", z, info);
            const errors: zolt.PhysicsUpdateError = .fromInt(z[2]);
            if (errors.contact_constraints_full) coverage.hit(.contact_constraints_full);
            if (errors.manifold_cache_full) coverage.hit(.manifold_cache_full);
            if (errors.body_pair_cache_full) coverage.hit(.body_pair_cache_full);
        }

        // The constraints and the bodies (activated by the contacts)
        try p.compareConstraints(false, coverage);
        try p.compareBodies();

        // Solve
        const velocity_steps = rng.intRange(u32, 1, 10);
        const position_steps = rng.intRange(u32, 0, 3);
        const use_dummy = rng.intRange(u32, 0, 3) == 0;
        try p.zolt_world.solve(velocity_steps, position_steps, use_dummy, p.rec());
        jolt.jolt_ccm_solve(p.jolt_world, velocity_steps, position_steps, @intFromBool(use_dummy), p.jolt_words.ptr, max_words, &p.jolt_num);
        try p.compare("solve");
        if (p.zolt_world.island_builder.getNumIslands() > 1) coverage.hit(.multi_island);
        try p.compareConstraints(true, coverage);
        try p.compareBodies();

        // Finalize the contact cache
        p.zolt_world.endStep();
        jolt.jolt_ccm_end_step(p.jolt_world);
        try p.compareRecords(coverage);
        try p.compareSave(0, 0);
        for (ids[0..num_bodies]) |a| {
            for (ids[0..num_bodies]) |b|
                try p.compareValues("WereBodiesInContact", @as(u32, @intFromBool(p.zolt_world.manager.wereBodiesInContact(.init(a), .init(b)))), jolt.jolt_ccm_were_in_contact(p.jolt_world, a, b));
        }

        // State: filtered saves, round trips, validation, restore filters, two parts. Not with small limits: a restore
        // that does not fit stops reading in the middle of the stream, Jolt then reads the rest of the stream as
        // garbage (and a validating restore traps), see the test "restore into a cache that is too small".
        const filter_body = ids[rng.intRange(u32, 0, num_bodies - 1)];
        try p.compareSave(rng.intRange(u32, 1, 2), filter_body);
        if (!small_limits) switch (rng.intRange(u32, 0, 5)) {
            0 => {
                // Round trip of the full state
                try p.compareSave(0, 0);
                try p.compareValues("RestoreState round trip", try p.restoreBoth(false, 0, 0, true), 1);
                try p.compareSave(0, 0);
            },
            1 => {
                // Validating restore of the current state
                try p.compareSave(0, 0);
                try p.compareValues("validating RestoreState", try p.restoreBoth(true, 0, 0, true), 1);
                coverage.hit(.validating_restore);
                try p.compareSave(0, 0);
            },
            2 => {
                // Restore filters: the contacts of one body, or all others
                try p.compareSave(0, 0);
                _ = try p.restoreBoth(false, rng.intRange(u32, 1, 2), filter_body, true);
                coverage.hit(.filtered_restore);
                try p.compareSave(0, 0);
            },
            3 => {
                // Two parts with complementary filters
                try p.compareSave(0, 0);
                _ = try p.restoreBoth(false, 1, filter_body, false);
                _ = try p.restoreBoth(false, 2, filter_body, true);
                coverage.hit(.two_part_restore);
                try p.compareSave(0, 0);
            },
            else => {},
        };
        try p.compareRecords(coverage);
    }
}

test "ContactConstraintManager parity: sizes" {
    var sizes: [4]u32 = undefined;
    jolt.jolt_ccm_sizes(&sizes);
    try std.testing.expectEqual(sizes[0], CCM.max_constraint_size);
    try std.testing.expectEqual(sizes[1], CCM.max_contact_constraints_limit);
    try std.testing.expectEqual(sizes[2], CCM.max_body_pairs_limit);
    try std.testing.expectEqual(sizes[3], CCM.max_contact_points);
}

test "ContactConstraintManager parity: restore into a cache that is too small" {
    // A state with only CCD manifolds (no body pairs): the restore runs out of cache space in the last section of the
    // stream, so the failure leaves no garbage to read
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{ .state = 0x7007_5a11 };
    var source_config: CCMConfig = .{
        .gravity = .{ 0, -9.81, 0 },
        .baumgarte = 0.2,
        .penetration_slop = 0.02,
        .max_penetration_distance = 0.2,
        .min_velocity_for_restitution = 1.0,
        .body_pair_cache_max_delta_position_sq = 1.0e-6,
        .body_pair_cache_cos_max_delta_rotation_div2 = 0.99984769515639123915701155881391,
        .contact_point_preserve_lambda_max_dist_sq = 1.0e-4,
        .num_velocity_steps = 10,
        .num_position_steps = 2,
        .max_bodies = 64,
        .max_body_pairs = 256,
        .max_contact_constraints = 512,
        .listener = 1,
        .modifier_mode = 0,
        .custom_combine = 0,
    };
    var source: Pair = .{ .allocator = allocator };
    try source.init(allocator, &source_config);
    defer source.deinit();
    var ids: [6]u32 = undefined;
    for (&ids, 0..) |*id, i| {
        const desc: CCMBodyDesc = .{
            .position = .{ @floatFromInt(i * 3), 0, 0 },
            .rotation = .{ 0, 0, 0, 1 },
            .linear_velocity = .{ 0, -1, 0 },
            .angular_velocity = .{ 0, 0, 0 },
            .half_extent = .{ 1, 1, 1 },
            .convex_radius = 0.05,
            .friction = 0.5,
            .restitution = 0.0,
            .gravity_factor = 1.0,
            .motion_type = @intFromEnum(zolt.MotionType.dynamic),
            .is_sensor = 0,
            .allowed_dofs = zolt.AllowedDOFs.all.toInt(),
            .activate = 1,
            .num_velocity_steps_override = 0,
            .num_position_steps_override = 0,
        };
        id.* = try source.zolt_world.addBody(&desc);
        try source.compareValues("body id", id.*, jolt.jolt_ccm_add_body(source.jolt_world, &desc));
    }
    try source.zolt_world.beginStep(1.0 / 60.0, 0.0);
    jolt.jolt_ccm_begin_step(source.jolt_world, 1.0 / 60.0, 0.0);
    var num_ccd: u32 = 0;
    for (0..ids.len) |a| {
        for (a + 1..ids.len) |b| {
            const m = randomManifold(&rng, &source, ids[a], ids[b], 100, 100 + @as(u32, @intCast(b)));
            source.zolt_world.ccdContact(ids[a], ids[b], &m, 0, source.rec());
            jolt.jolt_ccm_ccd_contact(source.jolt_world, ids[a], ids[b], &m, 0, source.jolt_words.ptr, max_words, &source.jolt_num);
            try source.compare("OnCCDContactAdded settings");
            num_ccd += 1;
        }
    }
    try source.zolt_world.solve(1, 1, false, source.rec());
    jolt.jolt_ccm_solve(source.jolt_world, 1, 1, 0, source.jolt_words.ptr, max_words, &source.jolt_num);
    try source.compare("solve");
    source.zolt_world.endStep();
    jolt.jolt_ccm_end_step(source.jolt_world);
    try source.compareSave(0, 0);
    try std.testing.expectEqual(@as(usize, 4 + 4 + 16 * num_ccd), source.zolt_bytes.items.len);

    // 4 body pairs and 4 constraints: 816 bytes, room for 10 CCD manifolds of 80 bytes
    var small_config = source_config;
    small_config.max_body_pairs = 4;
    small_config.max_contact_constraints = 4;
    var small: Pair = .{ .allocator = allocator };
    try small.init(allocator, &small_config);
    defer small.deinit();
    try small.zolt_bytes.appendSlice(allocator, source.zolt_bytes.items);
    try small.compareValues("RestoreState into a small cache", try small.restoreBoth(false, 0, 0, true), 0);
    try small.compareSave(0, 0);
    try std.testing.expect(small.zolt_bytes.items.len < source.zolt_bytes.items.len);
}

test "ContactConstraintManager parity: random frames" {
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{ .state = 0x51ce_a4a4 };
    var coverage: Coverage = .{};
    for (0..num_scenarios) |scenario|
        try runScenario(allocator, &rng, scenario, &coverage);
    try coverage.check();
}
