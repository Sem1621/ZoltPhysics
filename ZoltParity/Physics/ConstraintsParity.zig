//! Parity tests of the Phase 6 constraint classes (Jolt/Physics/Constraints/XConstraint): Zolt and the C++ library
//! create the same constraint between the same bodies and are compared bit for bit after every call.
//! C ABI wrappers: ZoltParity/Physics/ConstraintsReference.cpp (it covers every two body constraint class already).
//!
//! A scenario (`CSScenario`) is:
//! - two bodies created through a BodyManager on both sides (every combination of motion types that the kind supports,
//!   `Body.fixedToWorld()` as body 1, static bodies with motion properties (mAllowDynamicOrKinematic), random box
//!   shapes, sometimes behind an OffsetCenterOfMassShape, positions far from the origin in double precision, rotations,
//!   velocities, allowed DOFs and mass properties: calculated, calculated inertia with a given mass, or a given rotated
//!   inertia tensor);
//! - the constraint settings (`CSSettings`: the ConstraintSettings members and per kind arrays, the layout is documented
//!   at CSMakeSettings in the .cpp file and in the generator of the kind below), created with
//!   TwoBodyConstraintSettings.createConstraint (the virtual dispatcher);
//! - operations (`CSOp`): the generic ones (`GenericOp`: enable / disable, NotifyShapeChanged of either body or another
//!   body, ResetWarmStart, a SaveState / ResetWarmStart / RestoreState round trip, the solver settings and the user data)
//!   and the setters of the kind (numbers from `first_kind_op` = 16, documented at CSApplyKindOp), applied before a
//!   frame or after the last one;
//! - 1 to 4 frames of SetupVelocityConstraint, WarmStartVelocityConstraint (a random ratio), SolveVelocityConstraint
//!   (0 to 6 times) and SolvePositionConstraint (0 to 4 times, random Baumgarte factors).
//! Both sides record after the creation and after every call: the step id and the return value, position, rotation and
//! velocities of both bodies, IsActive, GetEnabled, the SaveState bytes, GetConstraintToBody1/2Matrix and every getter of
//! the kind (`recordGetters`), and at the start and the end the SaveBinaryState bytes of GetConstraintSettings with the
//! base class values (`CSOutput`, compared value by value, NaNs equal). Besides the scenario the settings binary state is
//! compared: the bytes of SaveBinaryState, each side restores the other side's bytes (ConstraintSettings::
//! sRestoreFromBinaryState through RegisterTypes.constraint_settings_types) and saves them again, truncated streams give
//! the same error text.
//!
//! The test "Constraint parity: <kind>" runs a kind; it returns error.SkipZigTest (visible in the build summary) while
//! the slot of the kind is a stub. "Constraint parity: the C++ side of every kind" runs every kind on the C++ side
//! (twice, the records must be equal) so that the descriptions and the C++ wrappers are checked before the Zolt class
//! exists, and "Constraint parity: the slots of the ported classes are filled" fails when a class is ported (its settings
//! no longer declare `is_stub`) but its slot here is still a stub.
//!
//! **How a Phase 6 worker extends this file** (only the slot and the generator of its own kind, nothing shared):
//! 1. Replace the stub slot `XKind = StubKind(...)` of the kind with a struct like `FixedKind`: `Settings`,
//!    `Constraint`, `makeSettings` (the kind specific part of CSMakeSettings, including the helpers of the settings class
//!    that it calls: SetSliderAxis, SetRatio, MakeFreeAxis / MakeFixedAxis / SetLimitedAxis; the base class members are
//!    set by the driver), `applyOp` (the kind specific operations of CSApplyKindOp, same numbers), `recordGetters`
//!    (exactly the getters of CSRecordGetters, in the same order), `recordSettingsGetters(rec, settings: *const
//!    Settings)` when CSRecordSettingsGetters records the kind (SixDOF: IsFreeAxis / IsFixedAxis of the settings),
//!    optionally `coverage` (the branches the scenarios must reach) and `settingsSaveable` (PathConstraint: `pub fn
//!    settingsSaveable(c) bool { return false; }`, Jolt's PathConstraint::GetConstraintSettings is not implemented: it
//!    asserts and returns nullptr, CSRecordSettings skips it).
//!    The filled slot also adds the kind to the PhysicsScene parity of BodyInterfaceParity.zig (random constraints of
//!    the ported kinds from `randomDescription` / `makeDescriptionSettings` in a PhysicsScene: binary state,
//!    CreateBodies, FromPhysicsSystem; Path excepted, see its `scene_constraint_kinds`), nothing to add there.
//! 2. Check the generator `genX` of the kind (it defines the layout the C++ side reads; widen what it covers, keep away
//!    from what Jolt asserts: Zolt's Debug build asserts the same).
//! 3. Prove that the test bites: break one operation of the Zig class temporarily, the test must fail.
//! Reserved numbers: the kind is the EConstraintSubType value (`Kind`); the generic operations are 0..15, the kind
//! operations 16..; the step ids are 0 (created), 1 (setup), 2 (warm start), 3 (velocity), 4 (position) and
//! 0x100 | op (after an operation). A new generic operation is a new `GenericOp` value plus its prong in CSApplyOp
//! (C++) and `applyOp` (Zig).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const AnyConstraint = zolt.Constraint;
const ConstraintSettings = zolt.ConstraintSettings;
const ConstraintSpace = zolt.ConstraintSpace;
const Core = zolt.Core;
const Mat44 = zolt.Mat44;
const MotionType = zolt.MotionType;
const MotorSettings = zolt.MotorSettings;
const ObjectLayer = zolt.ObjectLayer;
const OffsetCenterOfMassShape = zolt.OffsetCenterOfMassShape;
const Quat = zolt.Quat;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const SpringSettings = zolt.SpringSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const TwoBodyConstraint = zolt.TwoBodyConstraint;
const TwoBodyConstraintSettings = zolt.TwoBodyConstraintSettings;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;
const virtual = zolt.virtual;

const io = std.testing.io;

/// Number of random scenarios per kind
const scenario_iterations = 3000;

/// The C++ reference functions, see ConstraintsReference.cpp
const jolt = struct {
    extern fn jolt_cs_struct_sizes(out: *[12]u32) void;
    extern fn jolt_cs_world_create() *anyopaque;
    extern fn jolt_cs_world_destroy(world: *anyopaque) void;
    extern fn jolt_cs_run(world: *anyopaque, scenario: *const CSScenario, output: *CSOutput) void;
    extern fn jolt_cs_settings_save(settings: *const CSSettings, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_cs_settings_resave(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
};

// ---------------------------------------------------------------------------------------------------------------------
// The C ABI (must match the structs in ConstraintsReference.cpp, checked by "Constraint parity: struct layout")

/// Must match CSBodyDesc
const CSBodyDesc = extern struct {
    position: [3]Real,
    rotation: [4]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    half_extent: [3]f32,
    /// Not zero: the box is wrapped in an OffsetCenterOfMassShape
    com_offset: [3]f32,
    mass: f32,
    inertia_multiplier: f32,
    inertia: [16]f32,
    motion_type: u32,
    allowed_dofs: u32,
    override_mass_properties: u32,
    /// Body 1 only: Body::sFixedToWorld instead of a new body
    fixed_to_world: u32,
    /// A static body with motion properties (CanBeKinematicOrDynamic)
    allow_dynamic_or_kinematic: u32,
};

/// Must match CSSettings: the settings of a constraint, `i` / `r` / `f` per kind (see CSMakeSettings). Also the
/// constraint description of the PhysicsScene parity (BodyInterfaceParity.zig).
pub const CSSettings = extern struct {
    kind: u32,
    enabled: u32 = 1,
    constraint_priority: u32 = 0,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    draw_constraint_size: f32 = 1.0,
    user_data: u64 = 0,
    i: [32]u32 = @splat(0),
    r: [12]Real = @splat(0),
    f: [192]f32 = @splat(0),
};

/// Must match CSOp: an operation applied before frame `frame` (`frame == num_frames`: after the last frame)
const CSOp = extern struct {
    op: u32,
    frame: u32,
    i: [4]u32 = @splat(0),
    f: [12]f32 = @splat(0),
    r: [3]Real = @splat(0),
};

const max_ops = 24;

/// Must match CSScenario
const CSScenario = extern struct {
    body: [2]CSBodyDesc,
    settings: CSSettings,
    delta_time: f32,
    baumgarte: f32,
    warm_start_ratio: [4]f32,
    num_frames: u32,
    num_velocity_steps: u32,
    num_position_steps: u32,
    num_ops: u32,
    ops: [max_ops]CSOp,
};

const max_r = 1024;
const max_f = 16384;
const max_i = 8192;
const max_b = 32768;

/// Must match CSOutput
const CSOutput = extern struct {
    r: [max_r]Real,
    f: [max_f]f32,
    i: [max_i]u32,
    b: [max_b]u8,
    num_r: u32,
    num_f: u32,
    num_i: u32,
    num_b: u32,
};

/// The kind of a constraint: its EConstraintSubType value
pub const Kind = enum(u32) {
    fixed = 0,
    point = 1,
    hinge = 2,
    slider = 3,
    distance = 4,
    cone = 5,
    swing_twist = 6,
    six_dof = 7,
    path = 8,
    rack_and_pinion = 10,
    gear = 11,
    pulley = 12,
};

/// The generic operations (ECSOp), the kind operations start at `first_kind_op`
const GenericOp = enum(u32) {
    /// i[0]
    set_enabled = 0,
    /// i[0]: 0 body 1, 1 body 2, 2 another body ID; f[0..3] delta COM
    notify_shape_changed = 1,
    reset_warm_start = 2,
    /// SaveState, ResetWarmStart, RestoreState of the saved bytes
    save_restore_state = 3,
    /// SetConstraintPriority(i[0]), SetNumVelocityStepsOverride(i[1]), SetNumPositionStepsOverride(i[2]), SetUserData(i[3])
    set_solver_settings = 4,
};

const first_kind_op = 16;

// ---------------------------------------------------------------------------------------------------------------------
// Loading the descriptions

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

fn loadMat44(m: []const f32) Mat44 {
    return Mat44.init(load4(m[0..4]), load4(m[4..8]), load4(m[8..12]), load4(m[12..16]));
}

/// CSLoadSpring: mode, frequency / stiffness, damping
fn loadSpring(mode: u32, f: []const f32) SpringSettings {
    var s: SpringSettings = .{};
    s.mode = @enumFromInt(@as(u8, @intCast(mode)));
    s.frequency = f[0];
    s.damping = f[1];
    return s;
}

/// CSLoadMotor: the spring, min / max force limit, min / max torque limit
fn loadMotor(mode: u32, f: []const f32) MotorSettings {
    var m: MotorSettings = .{};
    m.spring_settings = loadSpring(mode, f);
    m.min_force_limit = f[2];
    m.max_force_limit = f[3];
    m.min_torque_limit = f[4];
    m.max_torque_limit = f[5];
    return m;
}

fn store3(v: Vec3, out: []f32) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
}

fn storeQuat(q: Quat, out: []f32) void {
    out[0] = q.getX();
    out[1] = q.getY();
    out[2] = q.getZ();
    out[3] = q.getW();
}

fn storeR3(v: RVec3, out: []Real) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
}

fn storeMat44(m: Mat44, out: []f32) void {
    for (0..4) |c| {
        const col = m.getColumn4(@intCast(c));
        out[4 * c + 0] = col.getX();
        out[4 * c + 1] = col.getY();
        out[4 * c + 2] = col.getZ();
        out[4 * c + 3] = col.getW();
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The record

/// The Rec class of ConstraintsReference.cpp: appends values to the output record
const Rec = struct {
    out: *CSOutput,
    allocator: Allocator,

    fn init(out: *CSOutput, allocator: Allocator) Rec {
        out.num_r = 0;
        out.num_f = 0;
        out.num_i = 0;
        out.num_b = 0;
        return .{ .out = out, .allocator = allocator };
    }

    fn r(self: *Rec, v: Real) void {
        if (self.out.num_r < max_r) self.out.r[self.out.num_r] = v;
        self.out.num_r += 1;
    }

    fn f(self: *Rec, v: f32) void {
        if (self.out.num_f < max_f) self.out.f[self.out.num_f] = v;
        self.out.num_f += 1;
    }

    fn i(self: *Rec, v: u32) void {
        if (self.out.num_i < max_i) self.out.i[self.out.num_i] = v;
        self.out.num_i += 1;
    }

    fn b(self: *Rec, v: bool) void {
        self.i(@intFromBool(v));
    }

    fn v3(self: *Rec, v: Vec3) void {
        self.f(v.getX());
        self.f(v.getY());
        self.f(v.getZ());
    }

    fn r3(self: *Rec, v: RVec3) void {
        self.r(v.getX());
        self.r(v.getY());
        self.r(v.getZ());
    }

    fn q(self: *Rec, v: Quat) void {
        self.f(v.getX());
        self.f(v.getY());
        self.f(v.getZ());
        self.f(v.getW());
    }

    fn m44(self: *Rec, m: Mat44) void {
        for (0..4) |c| {
            for (0..4) |row|
                self.f(m.get(@intCast(row), @intCast(c)));
        }
    }

    fn v2(self: *Rec, v: zolt.Vector(2)) void {
        self.f(v.getComponent(0));
        self.f(v.getComponent(1));
    }

    fn spring(self: *Rec, s: SpringSettings) void {
        self.i(@as(u8, @intFromEnum(s.mode)));
        self.f(s.frequency);
        self.f(s.damping);
    }

    fn motor(self: *Rec, m: *const MotorSettings) void {
        self.spring(m.spring_settings);
        self.f(m.min_force_limit);
        self.f(m.max_force_limit);
        self.f(m.min_torque_limit);
        self.f(m.max_torque_limit);
    }

    /// A block of bytes: the size in `i`, the bytes in `b`
    fn bytes(self: *Rec, data: []const u8) void {
        self.i(@intCast(data.len));
        for (data) |byte| {
            if (self.out.num_b < max_b) self.out.b[self.out.num_b] = byte;
            self.out.num_b += 1;
        }
    }

    fn bodyState(self: *Rec, body: *const Body) void {
        self.r3(body.getCenterOfMassPosition());
        self.q(body.getRotation());
        self.v3(body.getLinearVelocity());
        self.v3(body.getAngularVelocity());
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The world: a BodyManager on both sides

/// Broad phase layer = object layer (CSLayers)
const CSLayers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(CSLayers),

    pub fn getNumBroadPhaseLayers(self: *const CSLayers) u32 {
        _ = self;
        return 1;
    }

    pub fn getBroadPhaseLayer(self: *const CSLayers, layer: ObjectLayer) BroadPhaseLayer {
        _ = self;
        return .init(@intCast(layer));
    }
};

const World = struct {
    allocator: Allocator,
    layers: CSLayers = .{},
    body_manager: BodyManager = .empty,
    jolt_world: *anyopaque = undefined,
    zolt_out: *CSOutput = undefined,
    jolt_out: *CSOutput = undefined,

    fn init(self: *World, allocator: Allocator) !void {
        self.* = .{ .allocator = allocator };
        try self.body_manager.init(allocator, io, 8, 0, &self.layers.base);
        errdefer self.body_manager.deinit();
        self.zolt_out = try allocator.create(CSOutput);
        errdefer allocator.destroy(self.zolt_out);
        self.jolt_out = try allocator.create(CSOutput);
        self.jolt_world = jolt.jolt_cs_world_create();
    }

    fn deinit(self: *World) void {
        jolt.jolt_cs_world_destroy(self.jolt_world);
        self.allocator.destroy(self.jolt_out);
        self.allocator.destroy(self.zolt_out);
        self.body_manager.deinit();
    }

    /// CSCreateBody of ConstraintsReference.cpp
    fn createBody(self: *World, desc: *const CSBodyDesc) !*Body {
        var shape = RefConst(Shape).init((try BoxShape.create(self.allocator, load3(&desc.half_extent), .{})).asShape());
        defer shape.deinit();
        const com_offset = load3(&desc.com_offset);
        if (!com_offset.eql(Vec3.zero()))
            shape.set((try OffsetCenterOfMassShape.create(self.allocator, shape.get().?, com_offset)).asShape());
        var settings = BodyCreationSettings.initWithShape(shape.get().?, loadR3(&desc.position), loadQuat(&desc.rotation), @enumFromInt(desc.motion_type), 0);
        defer settings.deinit();
        settings.linear_velocity = load3(&desc.linear_velocity);
        settings.angular_velocity = load3(&desc.angular_velocity);
        settings.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
        settings.allow_dynamic_or_kinematic = desc.allow_dynamic_or_kinematic != 0;
        settings.override_mass_properties = @enumFromInt(desc.override_mass_properties);
        settings.inertia_multiplier = desc.inertia_multiplier;
        settings.mass_properties_override.mass = desc.mass;
        settings.mass_properties_override.inertia = loadMat44(&desc.inertia);
        const bm = &self.body_manager;
        const body = try bm.allocateBody(&settings);
        if (!bm.addBody(body)) @panic("out of bodies");
        body.setInBroadPhaseInternal(true);
        if (!body.isStatic())
            bm.activateBodies(&.{body.getID()});
        return body;
    }

    /// CSDestroyBody of ConstraintsReference.cpp
    fn destroyBody(self: *World, body: *Body) void {
        if (body == Body.fixedToWorld()) return;
        const id = body.getID();
        self.body_manager.deactivateBodies(&.{id});
        body.setInBroadPhaseInternal(false);
        self.body_manager.destroyBodies(&.{id});
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The slots: the Zolt side of each kind (see the file comment). A slot is a struct with `Settings`, `Constraint`,
// `makeSettings`, `applyOp`, `recordGetters` and optionally `coverage` / `settingsSaveable`, or `StubKind(Settings)`.

/// The slot of a kind whose class is not ported yet
fn StubKind(comptime S: type) type {
    return struct {
        pub const is_stub = true;
        pub const Settings = S;
    };
}

// ---- SLOT fixed (Zolt/Physics/Constraints/FixedConstraint.zig) ----
const FixedKind = struct {
    pub const Settings = zolt.FixedConstraintSettings;
    pub const Constraint = zolt.FixedConstraint;

    pub fn makeSettings(allocator: Allocator, d: *const CSSettings) !*Settings {
        const s = try Settings.create(allocator);
        s.space = @enumFromInt(@as(i32, @bitCast(d.i[0])));
        s.auto_detect_point = d.i[1] != 0;
        s.point1 = loadR3(d.r[0..3]);
        s.axis_x1 = load3(d.f[0..3]);
        s.axis_y1 = load3(d.f[3..6]);
        s.point2 = loadR3(d.r[3..6]);
        s.axis_x2 = load3(d.f[6..9]);
        s.axis_y2 = load3(d.f[9..12]);
        return s;
    }

    pub fn applyOp(c: *Constraint, op: *const CSOp, d: *const CSSettings, allocator: Allocator) !void {
        _ = c;
        _ = d;
        _ = allocator;
        std.debug.print("fixed: unknown operation {d}\n", .{op.op});
        return error.UnknownOperation;
    }

    pub fn recordGetters(rec: *Rec, c: *const Constraint) void {
        rec.v3(c.getTotalLambdaPosition());
        rec.v3(c.getTotalLambdaRotation());
    }

    pub fn coverage(cov: *Coverage, sc: *const CSScenario, c: *const Constraint) void {
        _ = c;
        const d = &sc.settings;
        const world = d.i[0] == @intFromEnum(ConstraintSpace.world_space);
        const auto = world and d.i[1] != 0;
        const can1 = sc.body[0].fixed_to_world == 0 and (sc.body[0].motion_type != @intFromEnum(MotionType.static) or sc.body[0].allow_dynamic_or_kinematic != 0);
        const can2 = sc.body[1].motion_type != @intFromEnum(MotionType.static) or sc.body[1].allow_dynamic_or_kinematic != 0;
        cov.hit("local space", !world);
        cov.hit("world space, given points", world and !auto);
        cov.hit("auto detect, body 1 static", auto and !can1);
        cov.hit("auto detect, body 2 static", auto and can1 and !can2);
        cov.hit("auto detect, weighted anchor", auto and can1 and can2);
        cov.hit("same axes (identity orientation)", load3(d.f[0..3]).eql(load3(d.f[6..9])) and load3(d.f[3..6]).eql(load3(d.f[9..12])));
        cov.hit("body 1 is fixed to world", sc.body[0].fixed_to_world != 0);
    }
};

// ---- SLOT point (Zolt/Physics/Constraints/PointConstraint.zig) ----
const PointKind = struct {
    pub const Settings = zolt.PointConstraintSettings;
    pub const Constraint = zolt.PointConstraint;

    pub fn makeSettings(allocator: Allocator, d: *const CSSettings) !*Settings {
        const s = try Settings.create(allocator);
        s.space = @enumFromInt(@as(i32, @bitCast(d.i[0])));
        s.point1 = loadR3(d.r[0..3]);
        s.point2 = loadR3(d.r[3..6]);
        return s;
    }

    pub fn applyOp(c: *Constraint, op: *const CSOp, d: *const CSSettings, allocator: Allocator) !void {
        _ = d;
        _ = allocator;
        switch (op.op) {
            16 => c.setPoint1(@enumFromInt(@as(i32, @bitCast(op.i[0]))), loadR3(&op.r)),
            17 => c.setPoint2(@enumFromInt(@as(i32, @bitCast(op.i[0]))), loadR3(&op.r)),
            else => {
                std.debug.print("point: unknown operation {d}\n", .{op.op});
                return error.UnknownOperation;
            },
        }
    }

    pub fn recordGetters(rec: *Rec, c: *const Constraint) void {
        rec.v3(c.getLocalSpacePoint1());
        rec.v3(c.getLocalSpacePoint2());
        rec.v3(c.getTotalLambdaPosition());
    }

    pub fn coverage(cov: *Coverage, sc: *const CSScenario, c: *const Constraint) void {
        _ = c;
        cov.hit("local space", sc.settings.i[0] == @intFromEnum(ConstraintSpace.local_to_body_com));
        cov.hit("world space", sc.settings.i[0] == @intFromEnum(ConstraintSpace.world_space));
        var set_world = false;
        var set_local = false;
        for (sc.ops[0..sc.num_ops]) |op| {
            if (op.op == 16 or op.op == 17) {
                if (op.i[0] == @intFromEnum(ConstraintSpace.world_space)) set_world = true else set_local = true;
            }
        }
        cov.hit("SetPoint in world space", set_world);
        cov.hit("SetPoint in local space", set_local);
        cov.hit("body 1 is fixed to world", sc.body[0].fixed_to_world != 0);
    }
};

// ---- SLOT hinge (Zolt/Physics/Constraints/HingeConstraint.zig, Phase 6 worker: replace with a slot like FixedKind) ----
const HingeKind = StubKind(zolt.HingeConstraintSettings);

// ---- SLOT slider (Zolt/Physics/Constraints/SliderConstraint.zig) ----
const SliderKind = StubKind(zolt.SliderConstraintSettings);

// ---- SLOT distance (Zolt/Physics/Constraints/DistanceConstraint.zig) ----
const DistanceKind = StubKind(zolt.DistanceConstraintSettings);

// ---- SLOT cone (Zolt/Physics/Constraints/ConeConstraint.zig) ----
const ConeKind = StubKind(zolt.ConeConstraintSettings);

// ---- SLOT swing_twist (Zolt/Physics/Constraints/SwingTwistConstraint.zig) ----
const SwingTwistKind = StubKind(zolt.SwingTwistConstraintSettings);

// ---- SLOT six_dof (Zolt/Physics/Constraints/SixDOFConstraint.zig) ----
const SixDOFKind = StubKind(zolt.SixDOFConstraintSettings);

// ---- SLOT path (Zolt/Physics/Constraints/PathConstraint.zig, PathConstraintPathHermite.zig) ----
const PathKind = StubKind(zolt.PathConstraintSettings);

// ---- SLOT rack_and_pinion (Zolt/Physics/Constraints/RackAndPinionConstraint.zig) ----
const RackAndPinionKind = StubKind(zolt.RackAndPinionConstraintSettings);

// ---- SLOT gear (Zolt/Physics/Constraints/GearConstraint.zig) ----
const GearKind = StubKind(zolt.GearConstraintSettings);

// ---- SLOT pulley (Zolt/Physics/Constraints/PulleyConstraint.zig) ----
const PulleyKind = StubKind(zolt.PulleyConstraintSettings);

/// The slot of a kind
fn Slot(comptime kind: Kind) type {
    return switch (kind) {
        .fixed => FixedKind,
        .point => PointKind,
        .hinge => HingeKind,
        .slider => SliderKind,
        .distance => DistanceKind,
        .cone => ConeKind,
        .swing_twist => SwingTwistKind,
        .six_dof => SixDOFKind,
        .path => PathKind,
        .rack_and_pinion => RackAndPinionKind,
        .gear => GearKind,
        .pulley => PulleyKind,
    };
}

fn isStubSlot(comptime K: type) bool {
    return @hasDecl(K, "is_stub");
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of a scenario (jolt_cs_run)

/// SaveState bytes of a constraint
fn recordState(rec: *Rec, c: *const AnyConstraint) void {
    var recorder = StateRecorderImpl.init(rec.allocator);
    defer recorder.deinit();
    c.saveState(recorder.recorder());
    rec.bytes(recorder.getData());
}

/// CSStep: the step id, the return value, both bodies and the generic state of the constraint, then the getters
fn recordStep(comptime K: type, rec: *Rec, id: u32, result: u32, c: *const K.Constraint) void {
    const tb: *const TwoBodyConstraint = &c.base;
    rec.i(0xC0DE0000 | id);
    rec.i(result);
    rec.bodyState(tb.getBody1());
    rec.bodyState(tb.getBody2());
    rec.b(tb.base.isActive());
    rec.b(tb.base.getEnabled());
    recordState(rec, &tb.base);
    rec.m44(tb.getConstraintToBody1Matrix());
    rec.m44(tb.getConstraintToBody2Matrix());
    K.recordGetters(rec, c);
}

/// The SaveBinaryState bytes of settings
fn settingsBytes(allocator: Allocator, settings: *const ConstraintSettings) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var w = StreamOutWrapper.init(&data.writer);
    settings.saveBinaryState(w.streamOut());
    return data.toOwnedSlice();
}

/// CSRecordSettings: GetConstraintSettings and the base class values
fn recordSettings(comptime K: type, rec: *Rec, c: *const K.Constraint) !void {
    if (@hasDecl(K, "settingsSaveable") and !K.settingsSaveable(c)) {
        rec.i(0xFFFFFFFF);
        return;
    }
    const base: *const AnyConstraint = &c.base.base;
    var settings = try base.getConstraintSettings();
    defer settings.deinit();
    const data = try settingsBytes(rec.allocator, settings.get().?);
    defer rec.allocator.free(data);
    rec.bytes(data);
    rec.i(settings.get().?.getRefCount());
    rec.i(base.getConstraintPriority());
    rec.i(base.getNumVelocityStepsOverride());
    rec.i(base.getNumPositionStepsOverride());
    rec.i(@truncate(base.getUserData()));
    rec.i(@truncate(base.getUserData() >> 32));
}

/// The ConstraintSettings members of a description (the end of CSMakeSettings)
fn setBaseSettings(s: *ConstraintSettings, d: *const CSSettings) void {
    s.enabled = d.enabled != 0;
    s.constraint_priority = d.constraint_priority;
    s.num_velocity_steps_override = d.num_velocity_steps_override;
    s.num_position_steps_override = d.num_position_steps_override;
    s.draw_constraint_size = d.draw_constraint_size;
    s.user_data = d.user_data;
}

/// The settings of a description (CSMakeSettings), one reference owned by the caller
fn makeSettings(comptime K: type, allocator: Allocator, d: *const CSSettings) !Ref(ConstraintSettings) {
    const s = try K.makeSettings(allocator, d);
    setBaseSettings(s.asSettings(), d);
    return .init(s.asSettings());
}

/// CSApplyOp
fn applyOp(comptime K: type, c: *K.Constraint, op: *const CSOp, d: *const CSSettings, allocator: Allocator) !void {
    const base: *AnyConstraint = &c.base.base;
    if (op.op >= first_kind_op)
        return K.applyOp(c, op, d, allocator);
    switch (@as(GenericOp, @enumFromInt(op.op))) {
        .set_enabled => base.setEnabled(op.i[0] != 0),
        .notify_shape_changed => {
            const id = switch (op.i[0]) {
                0 => c.base.getBody1().getID(),
                1 => c.base.getBody2().getID(),
                else => BodyID.init(12345),
            };
            base.notifyShapeChanged(id, load3(op.f[0..3]));
        },
        .reset_warm_start => base.resetWarmStart(),
        .save_restore_state => {
            var recorder = StateRecorderImpl.init(allocator);
            defer recorder.deinit();
            base.saveState(recorder.recorder());
            base.resetWarmStart();
            base.restoreState(recorder.recorder());
        },
        .set_solver_settings => {
            base.setConstraintPriority(op.i[0]);
            base.setNumVelocityStepsOverride(op.i[1]);
            base.setNumPositionStepsOverride(op.i[2]);
            base.setUserData(op.i[3]);
        },
    }
}

fn applyOps(comptime K: type, rec: *Rec, c: *K.Constraint, sc: *const CSScenario, frame: u32) !void {
    for (sc.ops[0..sc.num_ops]) |*op| {
        if (op.frame == frame) {
            try applyOp(K, c, op, &sc.settings, rec.allocator);
            recordStep(K, rec, 0x100 | op.op, 0, c);
        }
    }
}

/// jolt_cs_run on the Zolt side. Returns the constraint's coverage through `cov`.
fn zoltRun(comptime K: type, world: *World, sc: *const CSScenario, out: *CSOutput, cov: ?*Coverage) !void {
    const allocator = world.allocator;
    var rec = Rec.init(out, allocator);

    const body1 = if (sc.body[0].fixed_to_world != 0) Body.fixedToWorld() else try world.createBody(&sc.body[0]);
    defer world.destroyBody(body1);
    const body2 = try world.createBody(&sc.body[1]);
    defer world.destroyBody(body2);

    var settings = try makeSettings(K, allocator, &sc.settings);
    defer settings.deinit();
    const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
    const two_body = try two_body_settings.createConstraint(allocator, body1, body2);
    var constraint = Ref(AnyConstraint).init(&two_body.base);
    defer constraint.deinit();
    const c = virtual.downcast(K.Constraint, two_body);
    const base = &two_body.base;

    // Created
    rec.i(@intFromEnum(base.getType()));
    rec.i(@intFromEnum(base.getSubType()));
    if (@hasDecl(K, "recordSettingsGetters")) K.recordSettingsGetters(&rec, virtual.downcast(K.Settings, settings.get().?));
    recordStep(K, &rec, 0, 0, c);
    try recordSettings(K, &rec, c);

    // The frames
    for (0..sc.num_frames) |frame| {
        try applyOps(K, &rec, c, sc, @intCast(frame));

        base.setupVelocityConstraint(sc.delta_time);
        recordStep(K, &rec, 1, 0, c);

        base.warmStartVelocityConstraint(sc.warm_start_ratio[frame]);
        recordStep(K, &rec, 2, 0, c);

        for (0..sc.num_velocity_steps) |_| {
            const result = base.solveVelocityConstraint(sc.delta_time);
            if (cov) |cv| {
                cv.hit("SolveVelocityConstraint applied an impulse", result);
                cv.hit("SolveVelocityConstraint applied no impulse", !result);
            }
            recordStep(K, &rec, 3, @intFromBool(result), c);
        }

        for (0..sc.num_position_steps) |_| {
            const result = base.solvePositionConstraint(sc.delta_time, sc.baumgarte);
            if (cov) |cv| {
                cv.hit("SolvePositionConstraint applied an impulse", result);
                cv.hit("SolvePositionConstraint applied no impulse", !result);
            }
            recordStep(K, &rec, 4, @intFromBool(result), c);
        }
    }
    try applyOps(K, &rec, c, sc, sc.num_frames);

    // The final settings
    try recordSettings(K, &rec, c);

    if (cov) |cv| {
        if (@hasDecl(K, "coverage")) K.coverage(cv, sc, c);
    }
}

/// The SaveBinaryState bytes of the settings of a description (jolt_cs_settings_save)
fn zoltSettingsSave(comptime K: type, allocator: Allocator, d: *const CSSettings) ![]u8 {
    var settings = try makeSettings(K, allocator, d);
    defer settings.deinit();
    return settingsBytes(allocator, settings.get().?);
}

/// The result of a restore + save (jolt_cs_settings_resave): the bytes or the error text
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

/// ConstraintSettings.restoreFromBinaryState of `bytes`, then saveBinaryState of the result
fn zoltResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    if (result.hasError()) {
        out.setError(result.getError());
        return;
    }
    const data = try settingsBytes(allocator, result.getPtr().?);
    defer allocator.free(data);
    try out.bytes.appendSlice(allocator, data);
}

var resave_buffer: [65536]u8 = undefined;

/// jolt_cs_settings_resave into a Resaved
fn joltResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var err_buffer: [128]u8 = undefined;
    const size = jolt.jolt_cs_settings_resave(bytes.ptr, @intCast(bytes.len), &resave_buffer, resave_buffer.len, &err_buffer, err_buffer.len);
    if (size == 0xFFFFFFFF) {
        out.setError(std.mem.sliceTo(&err_buffer, 0));
        return;
    }
    try std.testing.expect(size <= resave_buffer.len);
    try out.bytes.appendSlice(allocator, resave_buffer[0..size]);
}

/// jolt_cs_settings_save into an owned slice
fn joltSettingsSave(allocator: Allocator, d: *const CSSettings) ![]u8 {
    const size = jolt.jolt_cs_settings_save(d, &resave_buffer, resave_buffer.len);
    try std.testing.expect(size <= resave_buffer.len);
    return allocator.dupe(u8, resave_buffer[0..size]);
}

// ---------------------------------------------------------------------------------------------------------------------
// Comparing records

/// Compare the records of both sides
fn compareOutputs(check: *Checker, name: []const u8, iteration: usize, a: *const CSOutput, b: *const CSOutput) void {
    const counts_ok = a.num_r == b.num_r and a.num_f == b.num_f and a.num_i == b.num_i and a.num_b == b.num_b;
    const overflow = a.num_r > max_r or a.num_f > max_f or a.num_i > max_i or a.num_b > max_b;
    var first_mismatch: ?struct { kind: u8, index: usize } = null;
    if (counts_ok and !overflow) {
        for (0..a.num_r) |k| {
            if (!fw.sameBits(a.r[k], b.r[k])) {
                first_mismatch = .{ .kind = 'r', .index = k };
                break;
            }
        }
        if (first_mismatch == null) for (0..a.num_f) |k| {
            if (!fw.sameBits(a.f[k], b.f[k])) {
                first_mismatch = .{ .kind = 'f', .index = k };
                break;
            }
        };
        if (first_mismatch == null) for (0..a.num_i) |k| {
            if (a.i[k] != b.i[k]) {
                first_mismatch = .{ .kind = 'i', .index = k };
                break;
            }
        };
        if (first_mismatch == null) for (0..a.num_b) |k| {
            if (a.b[k] != b.b[k]) {
                first_mismatch = .{ .kind = 'b', .index = k };
                break;
            }
        };
    }
    if (!counts_ok or overflow or first_mismatch != null) {
        if (check.mismatches < 5) {
            std.debug.print("{s}: iteration {d}: mismatch (counts {d}/{d}/{d}/{d} vs {d}/{d}/{d}/{d})", .{ name, iteration, a.num_r, a.num_f, a.num_i, a.num_b, b.num_r, b.num_f, b.num_i, b.num_b });
            if (first_mismatch) |m| {
                switch (m.kind) {
                    'r' => std.debug.print(" first difference r[{d}]: {d} vs {d}", .{ m.index, a.r[m.index], b.r[m.index] }),
                    'f' => std.debug.print(" first difference f[{d}]: {e} vs {e}", .{ m.index, a.f[m.index], b.f[m.index] }),
                    'i' => std.debug.print(" first difference i[{d}]: {x} vs {x}", .{ m.index, a.i[m.index], b.i[m.index] }),
                    else => std.debug.print(" first difference b[{d}]: {x} vs {x}", .{ m.index, a.b[m.index], b.b[m.index] }),
                }
            }
            std.debug.print("\n", .{});
        }
        check.mismatches += 1;
    }
}

/// Branches that only some inputs reach (each must be reached `min_hits` times)
const Coverage = struct {
    const min_hits = 10;
    names: [32][]const u8 = undefined,
    hits: [32]u32 = @splat(0),
    count: usize = 0,

    fn hit(self: *Coverage, comptime name: []const u8, condition: bool) void {
        for (self.names[0..self.count], 0..) |n, k| {
            if (std.mem.eql(u8, n, name)) {
                if (condition) self.hits[k] += 1;
                return;
            }
        }
        self.names[self.count] = name;
        self.hits[self.count] = @intFromBool(condition);
        self.count += 1;
    }

    fn expectAllReached(self: *const Coverage, test_name: []const u8) !void {
        var ok = true;
        for (self.names[0..self.count], self.hits[0..self.count]) |n, h| {
            if (h < min_hits) {
                std.debug.print("{s}: branch '{s}' reached {d} times, need {d}\n", .{ test_name, n, h, min_hits });
                ok = false;
            }
        }
        if (!ok) return error.CoverageNotReached;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The input generator

/// Which motion types a kind supports in a scenario (Jolt only solves a constraint with a dynamic body; parts that
/// need motion properties or dynamic bodies restrict it further)
const MotionPolicy = enum {
    /// Every combination, also static / kinematic only (the parts deactivate themselves)
    all,
    /// At least one dynamic body (TwoBodyConstraint::IsActive)
    any_dynamic,
    /// Static or dynamic bodies, at least one dynamic (IndependentAxisConstraintPart: no kinematic body)
    static_or_dynamic,
    /// Two dynamic bodies (GearConstraintPart, RackAndPinionConstraintPart)
    both_dynamic,
};

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

    fn vec(self: *Gen, scale: f32) Vec3 {
        return Vec3.init(self.float(-scale, scale), self.float(-scale, scale), self.float(-scale, scale));
    }

    fn quat(self: *Gen) Quat {
        if (self.oneIn(8)) return Quat.identity();
        while (true) {
            const v = Vec4.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1));
            if (v.lengthSq() > 0.01) return Quat.fromVec4(v.normalized());
        }
    }

    /// A random unit vector, sometimes axis aligned
    fn unit(self: *Gen) Vec3 {
        return switch (self.index(8)) {
            0 => Vec3.axisX(),
            1 => Vec3.axisY().negate(),
            2 => Vec3.axisZ(),
            else => self.quat().rotateAxisX(),
        };
    }

    /// Two perpendicular unit vectors (an X and a Y axis of a frame)
    fn frame(self: *Gen) [2]Vec3 {
        const q = self.quat();
        return .{ q.rotateAxisX(), q.rotateAxisY() };
    }

    fn angle(self: *Gen, min: f32, max: f32) f32 {
        return switch (self.index(6)) {
            0 => min,
            1 => max,
            else => self.float(min, max),
        };
    }

    fn motionType(self: *Gen) MotionType {
        return @enumFromInt(self.index(3));
    }

    fn motionTypes(self: *Gen, policy: MotionPolicy) [2]MotionType {
        while (true) {
            const pair = [2]MotionType{ self.motionType(), self.motionType() };
            const any_dynamic = pair[0] == .dynamic or pair[1] == .dynamic;
            switch (policy) {
                .all => return pair,
                .any_dynamic => if (any_dynamic) return pair,
                .static_or_dynamic => if (any_dynamic and pair[0] != .kinematic and pair[1] != .kinematic) return pair,
                .both_dynamic => if (pair[0] == .dynamic and pair[1] == .dynamic) return pair,
            }
        }
    }

    fn bodyDesc(self: *Gen, motion_type: MotionType, far: Real) CSBodyDesc {
        var desc: CSBodyDesc = undefined;
        desc.position = .{ far + self.float(-3, 3), self.float(-3, 3), far + self.float(-3, 3) };
        storeQuat(self.quat(), &desc.rotation);
        store3(if (self.oneIn(6)) Vec3.zero() else self.vec(5), &desc.linear_velocity);
        store3(if (self.oneIn(6)) Vec3.zero() else self.vec(3), &desc.angular_velocity);
        desc.half_extent = .{ self.float(0.1, 2), self.float(0.1, 2), self.float(0.1, 2) };
        store3(if (self.oneIn(3)) self.vec(0.5) else Vec3.zero(), &desc.com_offset);
        desc.mass = self.float(0.5, 50);
        desc.inertia_multiplier = if (self.oneIn(2)) 1.0 else self.float(0.5, 2);
        const rotation = Mat44.rotationQuat(self.quat());
        const diagonal = Vec3.init(self.float(0.05, 5), self.float(0.05, 5), self.float(0.05, 5));
        storeMat44(rotation.mul(Mat44.scaleVec3(diagonal)).mul(rotation.transposed()), &desc.inertia);
        desc.motion_type = @intFromEnum(motion_type);
        desc.allowed_dofs = if (self.oneIn(2)) 0b111111 else @intCast(1 + self.index(63));
        desc.override_mass_properties = @intCast(self.index(3));
        desc.fixed_to_world = 0;
        desc.allow_dynamic_or_kinematic = @intFromBool(motion_type == .static and self.oneIn(3));
        return desc;
    }

    /// A point in the space of the settings: near the bodies in world space, near the center of mass in local space
    fn point(self: *Gen, sc: *const CSScenario, space_value: u32, out: []Real) void {
        if (space_value == @intFromEnum(ConstraintSpace.world_space)) {
            const b = if (sc.body[0].fixed_to_world != 0) &sc.body[1] else &sc.body[0];
            for (0..3) |k| out[k] = b.position[k] + self.float(-2, 2);
        } else {
            storeR3(RVec3.fromVec3(if (self.oneIn(6)) Vec3.zero() else self.vec(2)), out);
        }
    }

    fn space(self: *Gen) u32 {
        return @intCast(@intFromEnum(if (self.oneIn(4)) ConstraintSpace.local_to_body_com else ConstraintSpace.world_space));
    }

    /// A spring at `i` / `f[0..2]`: a mode, a frequency or stiffness (sometimes zero: no spring), a damping
    fn spring(self: *Gen, i: *u32, f: []f32) void {
        i.* = @intCast(self.index(3));
        f[0] = if (self.oneIn(3)) 0.0 else if (i.* == 0) self.float(0.5, 20) else self.float(1, 500);
        f[1] = if (self.oneIn(3)) 0.0 else self.float(0, 2);
    }

    /// A valid motor (MotorSettings::IsValid, with stiffness) at `i` / `f[0..6]`
    fn motor(self: *Gen, i: *u32, f: []f32) void {
        i.* = @intCast(self.index(3));
        f[0] = if (i.* == 0) self.float(0.5, 20) else self.float(1, 500);
        f[1] = if (self.oneIn(3)) 0.0 else self.float(0, 2);
        const force = if (self.oneIn(4)) math.flt_max else self.float(0, 100);
        const torque = if (self.oneIn(4)) math.flt_max else self.float(0, 100);
        if (self.oneIn(3)) {
            const a = self.float(-50, 50);
            const b = self.float(-50, 50);
            f[2] = @min(a, b);
            f[3] = @max(a, b);
        } else {
            f[2] = -force;
            f[3] = force;
        }
        f[4] = -torque;
        f[5] = torque;
    }

    fn motorState(self: *Gen) u32 {
        return @intCast(self.index(4));
    }

    /// A generic operation
    fn genericOp(self: *Gen, op: *CSOp) void {
        op.op = @intCast(self.index(5));
        switch (@as(GenericOp, @enumFromInt(op.op))) {
            .set_enabled => op.i[0] = @intFromBool(!self.oneIn(3)),
            .notify_shape_changed => {
                op.i[0] = @intCast(self.index(3));
                store3(self.vec(0.5), op.f[0..3]);
            },
            .reset_warm_start, .save_restore_state => {},
            .set_solver_settings => {
                op.i[0] = self.rng.next();
                op.i[1] = @intCast(self.index(256));
                op.i[2] = @intCast(self.index(256));
                op.i[3] = self.rng.next();
            },
        }
    }

    /// A scenario of `kind` (the bodies, the base settings, the frames, the generic operations, then the generator of
    /// the kind for its settings and operations)
    fn scenario(self: *Gen, comptime kind: Kind) CSScenario {
        var sc: CSScenario = std.mem.zeroes(CSScenario);
        const policy = motionPolicy(kind);
        const pair = self.motionTypes(policy);
        const far: Real = if (Core.double_precision and self.oneIn(4)) 1.0e6 else 0.0;
        sc.body = .{ self.bodyDesc(pair[0], far), self.bodyDesc(pair[1], far) };
        if (pair[0] == .static and (policy == .all or policy == .any_dynamic or policy == .static_or_dynamic) and self.oneIn(3))
            sc.body[0].fixed_to_world = 1;

        sc.settings = .{ .kind = @intFromEnum(kind) };
        sc.settings.enabled = @intFromBool(!self.oneIn(8));
        sc.settings.constraint_priority = if (self.oneIn(2)) 0 else self.rng.next();
        sc.settings.num_velocity_steps_override = if (self.oneIn(2)) 0 else @intCast(self.index(256));
        sc.settings.num_position_steps_override = if (self.oneIn(2)) 0 else @intCast(self.index(256));
        sc.settings.draw_constraint_size = if (self.oneIn(2)) 1.0 else self.float(0.1, 3);
        sc.settings.user_data = (@as(u64, self.rng.next()) << 32) | self.rng.next();

        sc.delta_time = if (self.oneIn(2)) 1.0 / 60.0 else self.float(0.001, 0.1);
        sc.baumgarte = if (self.oneIn(4)) 0.2 else self.float(0, 1);
        for (&sc.warm_start_ratio) |*r| r.* = switch (self.index(4)) {
            0 => 1.0,
            1 => 0.0,
            else => self.float(0, 2),
        };
        sc.num_frames = @intCast(1 + self.index(4));
        sc.num_velocity_steps = @intCast(self.index(7));
        sc.num_position_steps = @intCast(self.index(5));

        const num_generic: u32 = if (self.oneIn(3)) 0 else @intCast(self.index(6));
        for (0..num_generic) |_| {
            const op = &sc.ops[sc.num_ops];
            self.genericOp(op);
            op.frame = @intCast(self.index(sc.num_frames + 1));
            sc.num_ops += 1;
        }

        genKind(self, kind, &sc);

        // Operations in frame order (the order of the list within a frame is the order they are applied)
        std.mem.sort(CSOp, sc.ops[0..sc.num_ops], {}, struct {
            fn lessThan(_: void, a: CSOp, b: CSOp) bool {
                return a.frame < b.frame;
            }
        }.lessThan);
        return sc;
    }

    /// Adds a kind operation in a random frame (`last` restricts it to after the last frame)
    fn kindOp(self: *Gen, sc: *CSScenario, op: u32, opts: struct { last: bool = false }) *CSOp {
        std.debug.assert(sc.num_ops < max_ops);
        const result = &sc.ops[sc.num_ops];
        sc.num_ops += 1;
        result.* = .{ .op = op, .frame = if (opts.last) sc.num_frames else @intCast(self.index(sc.num_frames + 1)) };
        return result;
    }

    fn numKindOps(self: *Gen) usize {
        return if (self.oneIn(3)) 0 else 1 + self.index(6);
    }
};

fn motionPolicy(kind: Kind) MotionPolicy {
    return switch (kind) {
        .fixed, .point => .all,
        .hinge, .slider, .distance, .cone, .swing_twist, .six_dof, .path => .any_dynamic,
        .pulley => .static_or_dynamic,
        .gear, .rack_and_pinion => .both_dynamic,
    };
}

fn genKind(g: *Gen, kind: Kind, sc: *CSScenario) void {
    switch (kind) {
        .fixed => genFixed(g, sc),
        .point => genPoint(g, sc),
        .hinge => genHinge(g, sc),
        .slider => genSlider(g, sc),
        .distance => genDistance(g, sc),
        .cone => genCone(g, sc),
        .swing_twist => genSwingTwist(g, sc),
        .six_dof => genSixDOF(g, sc),
        .path => genPath(g, sc),
        .rack_and_pinion => genRackAndPinion(g, sc),
        .gear => genGear(g, sc),
        .pulley => genPulley(g, sc),
    }
}

// The generators of the kinds: they define the layout of CSSettings (CSMakeSettings) and of the kind operations
// (CSApplyKindOp). They keep away from what Jolt asserts.

/// Fixed: I[0] space, I[1] auto detect point; R[0..3] point 1, R[3..6] point 2; F[0..12] axis X1, Y1, X2, Y2
fn genFixed(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    s.i[1] = @intFromBool(s.i[0] == @intFromEnum(ConstraintSpace.world_space) and g.oneIn(2));
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(3)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const frame1 = g.frame();
    const frame2 = if (g.oneIn(3)) frame1 else g.frame();
    store3(frame1[0], s.f[0..3]);
    store3(frame1[1], s.f[3..6]);
    store3(frame2[0], s.f[6..9]);
    store3(frame2[1], s.f[9..12]);
}

/// Point: I[0] space; R[0..3] point 1, R[3..6] point 2. Operations: 16 SetPoint1, 17 SetPoint2 (I[0] space, R point)
fn genPoint(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(3)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, if (g.oneIn(2)) 16 else 17, .{});
        op.i[0] = g.space();
        g.point(sc, op.i[0], &op.r);
    }
}

/// Hinge limits: none, or min in [-pi, 0], max in [0, pi] (min != max unless the spring is soft)
fn genHingeLimits(g: *Gen, out: []f32) void {
    if (g.oneIn(3)) {
        out[0] = -std.math.pi;
        out[1] = std.math.pi;
    } else {
        out[0] = g.angle(-std.math.pi, 0);
        out[1] = g.angle(0, std.math.pi);
        if (out[0] == out[1]) out[1] = 0.5;
    }
}

/// Hinge: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0..12] hinge axis 1, normal axis 1, hinge axis 2, normal axis
/// 2, F[12] limits min, F[13] limits max, limits spring I[1] F[14..16], F[16] max friction torque, motor I[2] F[17..23]
fn genHinge(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(2)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const frame1 = g.frame();
    const frame2 = if (g.oneIn(2)) frame1 else g.frame();
    store3(frame1[0], s.f[0..3]);
    store3(frame1[1], s.f[3..6]);
    store3(frame2[0], s.f[6..9]);
    store3(frame2[1], s.f[9..12]);
    genHingeLimits(g, s.f[12..14]);
    g.spring(&s.i[1], s.f[14..16]);
    s.f[16] = if (g.oneIn(2)) 0.0 else g.float(0, 10);
    g.motor(&s.i[2], s.f[17..23]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(8)), .{});
        switch (op.op) {
            16 => op.f[0] = g.float(0, 10),
            17 => op.i[0] = g.motorState(),
            18 => op.f[0] = g.float(-10, 10),
            19 => op.f[0] = g.float(-4, 4),
            20 => storeQuat(g.quat(), op.f[0..4]),
            21 => genHingeLimits(g, op.f[0..2]),
            22 => g.spring(&op.i[0], op.f[0..2]),
            23 => g.motor(&op.i[0], op.f[0..6]),
            else => unreachable,
        }
    }
}

/// Slider limits: none, or min <= 0, max >= 0 (min != max)
fn genSliderLimits(g: *Gen, out: []f32) void {
    if (g.oneIn(3)) {
        out[0] = -math.flt_max;
        out[1] = math.flt_max;
    } else {
        out[0] = if (g.oneIn(6)) 0.0 else g.float(-2, 0);
        out[1] = if (g.oneIn(6)) 0.0 else g.float(0, 2);
        if (out[0] == out[1]) out[1] = 1.0;
    }
}

/// Slider: I[0] space, I[1] auto detect point, I[2] SetSliderAxis(F[0..3]); R[0..3] point 1, R[3..6] point 2; F[0..12]
/// slider axis 1, normal axis 1, slider axis 2, normal axis 2, F[12] limits min, F[13] limits max, limits spring I[3]
/// F[14..16], F[16] max friction force, motor I[4] F[17..23]
fn genSlider(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    const world = s.i[0] == @intFromEnum(ConstraintSpace.world_space);
    s.i[1] = @intFromBool(world and g.oneIn(2));
    s.i[2] = @intFromBool(world and g.oneIn(3));
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(2)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const frame1 = g.frame();
    const frame2 = if (g.oneIn(2)) frame1 else g.frame();
    store3(frame1[0], s.f[0..3]);
    store3(frame1[1], s.f[3..6]);
    store3(frame2[0], s.f[6..9]);
    store3(frame2[1], s.f[9..12]);
    genSliderLimits(g, s.f[12..14]);
    g.spring(&s.i[3], s.f[14..16]);
    s.f[16] = if (g.oneIn(2)) 0.0 else g.float(0, 10);
    g.motor(&s.i[4], s.f[17..23]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(7)), .{});
        switch (op.op) {
            16 => op.f[0] = g.float(0, 10),
            17 => op.i[0] = g.motorState(),
            18 => op.f[0] = g.float(-10, 10),
            19 => op.f[0] = g.float(-3, 3),
            20 => genSliderLimits(g, op.f[0..2]),
            21 => g.spring(&op.i[0], op.f[0..2]),
            22 => g.motor(&op.i[0], op.f[0..6]),
            else => unreachable,
        }
    }
}

/// Distance: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0] min distance, F[1] max distance (-1: the distance at
/// creation), limits spring I[1] F[2..4]
fn genDistance(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    g.point(sc, s.i[0], s.r[0..3]);
    g.point(sc, s.i[0], s.r[3..6]);
    if (g.oneIn(3)) {
        s.f[0] = -1.0;
        s.f[1] = -1.0;
    } else {
        s.f[0] = g.float(0, 2);
        s.f[1] = s.f[0] + (if (g.oneIn(4)) 0.0 else g.float(0, 2));
    }
    g.spring(&s.i[1], s.f[2..4]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(2)), .{});
        switch (op.op) {
            16 => {
                op.f[0] = g.float(0, 2);
                op.f[1] = op.f[0] + g.float(0, 2);
            },
            17 => g.spring(&op.i[0], op.f[0..2]),
            else => unreachable,
        }
    }
}

/// Cone: I[0] space; R[0..3] point 1, R[3..6] point 2; F[0..3] twist axis 1, F[3..6] twist axis 2, F[6] half cone angle
fn genCone(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(2)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const axis1 = g.unit();
    store3(axis1, s.f[0..3]);
    store3(if (g.oneIn(2)) axis1 else g.unit(), s.f[3..6]);
    s.f[6] = g.angle(0, std.math.pi);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, 16, .{});
        op.f[0] = g.angle(0, std.math.pi);
    }
}

/// SwingTwist: I[0] space, I[1] swing type; R[0..3] position 1, R[3..6] position 2; F[0..12] twist axis 1, plane axis 1,
/// twist axis 2, plane axis 2, F[12] normal half cone angle, F[13] plane half cone angle, F[14] twist min angle, F[15]
/// twist max angle, F[16] max friction torque, swing motor I[2] F[17..23], twist motor I[3] F[23..29]
fn genSwingTwist(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    s.i[1] = @intCast(g.index(2));
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(2)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const frame1 = g.frame();
    const frame2 = if (g.oneIn(2)) frame1 else g.frame();
    store3(frame1[0], s.f[0..3]);
    store3(frame1[1], s.f[3..6]);
    store3(frame2[0], s.f[6..9]);
    store3(frame2[1], s.f[9..12]);
    s.f[12] = g.angle(0, std.math.pi);
    s.f[13] = g.angle(0, std.math.pi);
    s.f[14] = g.angle(-std.math.pi, 0);
    s.f[15] = g.angle(0, std.math.pi);
    s.f[16] = if (g.oneIn(2)) 0.0 else g.float(0, 10);
    g.motor(&s.i[2], s.f[17..23]);
    g.motor(&s.i[3], s.f[23..29]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(13)), .{});
        switch (op.op) {
            16, 17 => op.f[0] = g.angle(0, std.math.pi),
            18 => op.f[0] = g.angle(-std.math.pi, 0),
            19 => op.f[0] = g.angle(0, std.math.pi),
            20 => op.f[0] = g.float(0, 10),
            21, 22 => op.i[0] = g.motorState(),
            23, 24 => store3(g.vec(5), op.f[0..3]),
            25, 26 => storeQuat(g.quat(), op.f[0..4]),
            27, 28 => g.motor(&op.i[0], op.f[0..6]),
            else => unreachable,
        }
    }
}

/// The limits of one SixDOF axis: free, fixed or a range (rotation Y / Z: symmetric for the cone swing type)
fn genSixDOFLimit(g: *Gen, axis: usize, swing_type: u32, min: *f32, max: *f32) void {
    switch (g.index(4)) {
        0 => {
            min.* = -math.flt_max;
            max.* = math.flt_max;
        },
        1 => {
            min.* = math.flt_max;
            max.* = -math.flt_max;
        },
        else => if (axis < 3) {
            min.* = g.float(-2, 0);
            max.* = g.float(0, 2);
        } else if (axis == 3 or swing_type == 1) {
            min.* = g.angle(-std.math.pi, 0);
            max.* = g.angle(0, std.math.pi);
        } else {
            max.* = g.angle(0, std.math.pi);
            min.* = -max.*;
        },
    }
}

/// SixDOF: I[0] space, I[1] swing type; R[0..3] position 1, R[3..6] position 2; F[0..12] axis X1, Y1, X2, Y2, F[12..18]
/// max friction, F[18..24] limit min, F[24..30] limit max, limits spring k I[2 + k] F[30 + 2k..], motor k I[5 + k]
/// F[36 + 6k..]; I[11 + k] the settings helper that sets the limits of axis k: 1 SetLimitedAxis(k, F[18 + k], F[24 + k]),
/// 2 MakeFreeAxis(k), 3 MakeFixedAxis(k), otherwise the members (see CSMakeSettings)
fn genSixDOF(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    s.i[1] = @intCast(g.index(2));
    g.point(sc, s.i[0], s.r[0..3]);
    if (g.oneIn(2)) @memcpy(s.r[3..6], s.r[0..3]) else g.point(sc, s.i[0], s.r[3..6]);
    const frame1 = g.frame();
    const frame2 = if (g.oneIn(2)) frame1 else g.frame();
    store3(frame1[0], s.f[0..3]);
    store3(frame1[1], s.f[3..6]);
    store3(frame2[0], s.f[6..9]);
    store3(frame2[1], s.f[9..12]);
    for (0..6) |a| {
        s.f[12 + a] = if (g.oneIn(2)) 0.0 else g.float(0, 10);
        genSixDOFLimit(g, a, s.i[1], &s.f[18 + a], &s.f[24 + a]);
        g.motor(&s.i[5 + a], s.f[36 + 6 * a .. 42 + 6 * a]);
    }
    for (0..3) |a| g.spring(&s.i[2 + a], s.f[30 + 2 * a .. 32 + 2 * a]);
    for (0..6) |a| s.i[11 + a] = @intCast(g.index(4)); // Members, SetLimitedAxis, MakeFreeAxis, MakeFixedAxis
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(11)), .{});
        switch (op.op) {
            16, 17 => for (0..3) |a| genSixDOFLimit(g, if (op.op == 16) a else 3 + a, s.i[1], &op.f[a], &op.f[3 + a]),
            18 => {
                op.i[0] = @intCast(g.index(3));
                g.spring(&op.i[1], op.f[0..2]);
            },
            19 => {
                op.i[0] = @intCast(g.index(6));
                op.f[0] = if (g.oneIn(3)) 0.0 else g.float(0, 10);
            },
            20 => {
                op.i[0] = @intCast(g.index(6));
                op.i[1] = g.motorState();
            },
            21, 22, 23 => store3(g.vec(3), op.f[0..3]),
            24, 25 => storeQuat(g.quat(), op.f[0..4]),
            26 => {
                op.i[0] = @intCast(g.index(6));
                g.motor(&op.i[1], op.f[0..6]);
            },
            else => unreachable,
        }
    }
}

/// Path: I[1] looping, I[2] number of points, I[3] rotation constraint type, motor I[4]; F[0..3] path position, F[3..7]
/// path rotation, F[7] path fraction, F[8] max friction force, motor F[9..15], point p at F[16 + 9p..]: position,
/// tangent, normal. The points lie in the XZ plane with normal Y (a circle when looping), so the interpolated normals are
/// never degenerate.
fn genPath(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    const looping = g.oneIn(3);
    const num_points: u32 = @intCast(if (looping) 3 + g.index(6) else 2 + g.index(7));
    s.i[0] = 1;
    s.i[1] = @intFromBool(looping);
    s.i[2] = num_points;
    s.i[3] = @intCast(g.index(6));
    const radius = g.float(1, 4);
    for (0..num_points) |p| {
        const pf = s.f[16 + 9 * p ..];
        if (looping) {
            const a = 2.0 * std.math.pi * @as(f32, @floatFromInt(p)) / @as(f32, @floatFromInt(num_points));
            const sin_cos = Vec4.replicate(a).sinCos();
            const sin = sin_cos.sin.getX();
            const cos = sin_cos.cos.getX();
            store3(Vec3.init(radius * cos, 0, radius * sin), pf[0..3]);
            store3(Vec3.init(-sin, 0, cos).mulScalar(radius), pf[3..6]);
        } else {
            store3(Vec3.init(@floatFromInt(p), 0, g.float(-0.3, 0.3)), pf[0..3]);
            store3(Vec3.init(1, 0, g.float(-0.3, 0.3)).mulScalar(g.float(0.5, 2)), pf[3..6]);
        }
        store3(Vec3.axisY(), pf[6..9]);
    }
    store3(g.vec(1), s.f[0..3]);
    storeQuat(g.quat(), s.f[3..7]);
    const max_fraction: f32 = @floatFromInt(if (looping) num_points else num_points - 1);
    s.f[7] = g.angle(0, max_fraction);
    s.f[8] = if (g.oneIn(2)) 0.0 else g.float(0, 10);
    g.motor(&s.i[4], s.f[9..15]);
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, @intCast(16 + g.index(6)), .{});
        switch (op.op) {
            16 => {
                op.i[0] = 1;
                op.f[0] = g.angle(0, max_fraction);
            },
            17 => op.f[0] = g.float(0, 10),
            18 => op.i[0] = g.motorState(),
            19 => op.f[0] = g.float(-5, 5),
            20 => op.f[0] = if (looping) g.float(-2 * max_fraction, 2 * max_fraction) else g.angle(0, max_fraction),
            21 => g.motor(&op.i[0], op.f[0..6]),
            else => unreachable,
        }
    }
    if (g.oneIn(4)) {
        // SetPath(nullptr) after the last frame (the solver would dereference the path)
        const op = g.kindOp(sc, 16, .{ .last = true });
        op.i[0] = 0;
        op.f[0] = 0.0;
    }
}

/// A world space reference frame for the reference constraints of the gear / rack and pinion operations: an axis and a
/// normal at `f[0..6]`
fn genReferenceFrame(g: *Gen, f: []f32) void {
    const frame = g.frame();
    store3(frame[0], f[0..3]);
    store3(frame[1], f[3..6]);
}

/// RackAndPinion: I[0] space, I[1] != 0: SetRatio(I[2] teeth of the rack, F[7] rack length, I[3] teeth of the pinion)
/// instead of F[6]; F[0..3] hinge axis, F[3..6] slider axis, F[6] ratio. Operation 16 SetConstraints (I[0] bit 0 pinion
/// hinge, bit 1 rack slider; F[0..6] hinge axis and normal, F[6..12] slider axis and normal, world space)
fn genRackAndPinion(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    store3(g.unit(), s.f[0..3]);
    store3(g.unit(), s.f[3..6]);
    s.f[6] = if (g.oneIn(3)) 1.0 else g.float(-5, 5);
    if (g.oneIn(2)) {
        s.i[1] = 1;
        s.i[2] = @intCast(1 + g.index(50));
        s.i[3] = @intCast(1 + g.index(50));
        s.f[7] = g.float(0.5, 5);
    }
    if (!g.oneIn(3)) {
        const op = g.kindOp(sc, 16, .{});
        op.i[0] = @intCast(g.index(4));
        genReferenceFrame(g, op.f[0..6]);
        genReferenceFrame(g, op.f[6..12]);
    }
}

/// Gear: I[0] space, I[1] != 0: SetRatio(I[2] teeth of gear 1, I[3] teeth of gear 2) instead of F[6]; F[0..3] hinge axis
/// 1, F[3..6] hinge axis 2, F[6] ratio. Operation 16 SetConstraints (I[0] bit 0 hinge of body 1, bit 1 hinge of body 2;
/// F[0..6] / F[6..12] axis and normal, world space)
fn genGear(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    store3(g.unit(), s.f[0..3]);
    store3(g.unit(), s.f[3..6]);
    s.f[6] = if (g.oneIn(3)) 1.0 else g.float(-5, 5);
    if (g.oneIn(2)) {
        s.i[1] = 1;
        s.i[2] = @intCast(1 + g.index(50));
        s.i[3] = @intCast(1 + g.index(50));
    }
    if (!g.oneIn(3)) {
        const op = g.kindOp(sc, 16, .{});
        op.i[0] = @intCast(g.index(4));
        genReferenceFrame(g, op.f[0..6]);
        genReferenceFrame(g, op.f[6..12]);
    }
}

/// Pulley: I[0] space; R[0..3] body point 1, R[3..6] fixed point 1, R[6..9] body point 2, R[9..12] fixed point 2 (world
/// space); F[0] ratio, F[1] min length, F[2] max length (-1: the length at creation). Operation 16 SetLength(F[0], F[1])
fn genPulley(g: *Gen, sc: *CSScenario) void {
    const s = &sc.settings;
    s.i[0] = g.space();
    g.point(sc, s.i[0], s.r[0..3]);
    for (0..3) |k| s.r[3 + k] = sc.body[1].position[k] + g.float(-1, 1) + @as(Real, if (k == 1) 5.0 else 0.0);
    g.point(sc, s.i[0], s.r[6..9]);
    for (0..3) |k| s.r[9 + k] = sc.body[1].position[k] + g.float(-1, 1) + @as(Real, if (k == 1) 5.0 else 0.0);
    s.f[0] = if (g.oneIn(3)) 1.0 else g.float(0.5, 3);
    if (g.oneIn(3)) {
        s.f[1] = -1.0;
        s.f[2] = -1.0;
    } else {
        s.f[1] = g.float(0, 5);
        s.f[2] = if (g.oneIn(4)) -1.0 else s.f[1] + g.float(0, 5);
    }
    for (0..g.numKindOps()) |_| {
        const op = g.kindOp(sc, 16, .{});
        op.f[0] = g.float(0, 5);
        op.f[1] = op.f[0] + g.float(0, 5);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The descriptions for the other parity files (BodyInterfaceParity.zig: the constraints of a PhysicsScene, whose C++ side
// calls CSMakeConstraintSettings of ConstraintsReference.cpp)

/// The kinds whose slot is filled (the Zolt class is ported), in EConstraintSubType order
pub const ported_kinds: []const Kind = blk: {
    const fields = @typeInfo(Kind).@"enum".fields;
    var kinds: [fields.len]Kind = undefined;
    var n: usize = 0;
    for (fields) |field| {
        if (!isStubSlot(Slot(@enumFromInt(field.value)))) {
            kinds[n] = @enumFromInt(field.value);
            n += 1;
        }
    }
    const result = kinds[0..n].*;
    break :blk &result;
};

/// A random description of `kind`: the settings of a random scenario of the generator (`seed` selects it)
pub fn randomDescription(seed: u32, kind: Kind) CSSettings {
    var g: Gen = .{ .rng = .{ .state = seed } };
    return switch (kind) {
        inline else => |k| g.scenario(k).settings,
    };
}

/// The Zolt settings of a description (CSMakeSettings, one reference owned by the caller): error.UnknownKind when the
/// kind is not a two body EConstraintSubType of `Kind`, error.KindNotPorted while the slot of the kind is a stub
pub fn makeDescriptionSettings(allocator: Allocator, d: *const CSSettings) !Ref(ConstraintSettings) {
    const kind = std.enums.fromInt(Kind, d.kind) orelse return error.UnknownKind;
    switch (kind) {
        inline else => |k| {
            const K = Slot(k);
            if (comptime isStubSlot(K)) return error.KindNotPorted;
            return makeSettings(K, allocator, d);
        },
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The tests

/// The parity test of a kind: scenarios on both sides, the settings binary state and its cross language round trip
fn runKind(comptime kind: Kind) !void {
    const K = Slot(kind);
    if (comptime isStubSlot(K)) return error.SkipZigTest; // The class is not ported yet (see the file comment)

    const allocator = std.testing.allocator;
    var world: World = undefined;
    try world.init(allocator);
    defer world.deinit();

    const name = "Constraint parity: " ++ @tagName(kind);
    var run_checker: Checker = .{ .name = name ++ " (scenario records)" };
    var save_checker: Checker = .{ .name = name ++ " (settings SaveBinaryState bytes)" };
    var resave_checker: Checker = .{ .name = name ++ " (settings restored from the other side's bytes)" };
    var error_checker: Checker = .{ .name = name ++ " (truncated settings streams)" };
    var cov: Coverage = .{};
    var g: Gen = .{ .rng = .{ .state = 0xC0457000 +% @intFromEnum(kind) } };
    var num_errors: u32 = 0;

    for (0..scenario_iterations) |iteration| {
        const sc = g.scenario(kind);

        // The scenario
        try zoltRun(K, &world, &sc, world.zolt_out, &cov);
        jolt.jolt_cs_run(world.jolt_world, &sc, world.jolt_out);
        compareOutputs(&run_checker, name, iteration, world.zolt_out, world.jolt_out);

        // The settings binary state
        const zolt_bytes = try zoltSettingsSave(K, allocator, &sc.settings);
        defer allocator.free(zolt_bytes);
        const jolt_bytes = try joltSettingsSave(allocator, &sc.settings);
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

        // Truncated streams: the same error (or the same settings) on both sides
        {
            const len = g.index(zolt_bytes.len + 1);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltResave(allocator, zolt_bytes[0..len], &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResave(allocator, zolt_bytes[0..len], &j);
            if (z.err != null) num_errors += 1;
            if (!z.eql(&j) and error_checker.mismatches < 5)
                std.debug.print("{s}: truncated at {d}: zolt '{s}' jolt '{s}'\n", .{ name, len, z.err orelse "(ok)", j.err orelse "(ok)" });
            error_checker.check(.{ iteration, len }, z.eql(&j), true);
        }
    }

    try fw.finishAll(&.{ &run_checker, &save_checker, &resave_checker, &error_checker });
    try cov.expectAllReached(name);
    try std.testing.expect(num_errors > scenario_iterations / 2);
}

test "Constraint parity: fixed" {
    try runKind(.fixed);
}

test "Constraint parity: point" {
    try runKind(.point);
}

test "Constraint parity: hinge" {
    try runKind(.hinge);
}

test "Constraint parity: slider" {
    try runKind(.slider);
}

test "Constraint parity: distance" {
    try runKind(.distance);
}

test "Constraint parity: cone" {
    try runKind(.cone);
}

test "Constraint parity: swing_twist" {
    try runKind(.swing_twist);
}

test "Constraint parity: six_dof" {
    try runKind(.six_dof);
}

test "Constraint parity: path" {
    try runKind(.path);
}

test "Constraint parity: rack_and_pinion" {
    try runKind(.rack_and_pinion);
}

test "Constraint parity: gear" {
    try runKind(.gear);
}

test "Constraint parity: pulley" {
    try runKind(.pulley);
}

// Every kind runs on the C++ side already (twice, in two worlds: the records must be equal), so the descriptions, the
// generators and the C++ wrappers are checked before the Zolt class exists. The settings bytes of every kind survive a
// C++ restore + save, and a description with the RTTI hash of a stub class is "Failed to create instance of type" in
// Zolt (the Factory does not know it yet).
test "Constraint parity: the C++ side of every kind" {
    const allocator = std.testing.allocator;
    const world_a = jolt.jolt_cs_world_create();
    defer jolt.jolt_cs_world_destroy(world_a);
    const world_b = jolt.jolt_cs_world_create();
    defer jolt.jolt_cs_world_destroy(world_b);
    const out_a = try allocator.create(CSOutput);
    defer allocator.destroy(out_a);
    const out_b = try allocator.create(CSOutput);
    defer allocator.destroy(out_b);

    inline for (@typeInfo(Kind).@"enum".fields) |field| {
        const kind: Kind = @enumFromInt(field.value);
        var checker: Checker = .{ .name = "C++ side of " ++ field.name };
        var g: Gen = .{ .rng = .{ .state = 0x5EED0000 +% field.value } };
        var num_velocity_impulses: u32 = 0;
        for (0..200) |iteration| {
            const sc = g.scenario(kind);
            jolt.jolt_cs_run(world_a, &sc, out_a);
            jolt.jolt_cs_run(world_b, &sc, out_b);
            compareOutputs(&checker, field.name, iteration, out_a, out_b);

            // The constraint has the sub type of the kind (the second value of the record)
            try std.testing.expectEqual(@as(u32, 1), out_a.i[0]); // EConstraintType::TwoBodyConstraint
            try std.testing.expectEqual(field.value, out_a.i[1]);

            // Steps with id 3 (SolveVelocityConstraint) that applied an impulse: the scenarios do something
            for (0..@min(out_a.num_i, max_i) -| 1) |k| {
                if (out_a.i[k] == 0xC0DE0003 and out_a.i[k + 1] == 1) num_velocity_impulses += 1;
            }

            // The settings bytes survive a C++ restore + save
            const bytes = try joltSettingsSave(allocator, &sc.settings);
            defer allocator.free(bytes);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResave(allocator, bytes, &j);
            if (j.err) |e| std.debug.print("{s}: C++ restore failed: {s}\n", .{ field.name, e });
            try std.testing.expect(j.err == null and std.mem.eql(u8, j.bytes.items, bytes));

            // Zolt: a stub class is unknown, a ported class restores
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltResave(allocator, bytes, &z);
            if (comptime isStubSlot(Slot(kind))) {
                try std.testing.expectEqualStrings("Failed to create instance of type", z.err.?);
            } else {
                try std.testing.expect(z.err == null and std.mem.eql(u8, z.bytes.items, bytes));
            }
        }
        try checker.finish();
        try std.testing.expect(num_velocity_impulses > 50);
    }
}

test "Constraint parity: the slots of the ported classes are filled" {
    inline for (@typeInfo(Kind).@"enum".fields) |field| {
        const K = Slot(@enumFromInt(field.value));
        const class_is_stub = @hasDecl(K.Settings, "is_stub");
        if (class_is_stub != isStubSlot(K)) {
            std.debug.print("ConstraintsParity.zig: the slot of {s} is {s} but its class is {s}: fill the slot (see the file comment)\n", .{ field.name, if (isStubSlot(K)) "a stub" else "filled", if (class_is_stub) "a stub" else "ported" });
            return error.SlotOutOfDate;
        }
    }
}

test "Constraint parity: the C++ side calls the settings helpers" {
    // The descriptions select the helpers of the settings classes that compute values (CSMakeSettings), so their porters
    // get parity coverage: SetRatio of Gear / RackAndPinion (mRatio is the last value of their binary state) and the axis
    // helpers of SixDOF (CSRecordSettingsGetters records IsFreeAxis / IsFixedAxis of the settings right after the type
    // and the sub type)
    const allocator = std.testing.allocator;
    const world = jolt.jolt_cs_world_create();
    defer jolt.jolt_cs_world_destroy(world);
    const out = try allocator.create(CSOutput);
    defer allocator.destroy(out);
    var g: Gen = .{ .rng = .{ .state = 0x4E1BE000 } };
    var num_set_ratio: [2]u32 = @splat(0);
    var num_axis_helpers: [4]u32 = @splat(0);
    for (0..300) |_| {
        for ([_]Kind{ .gear, .rack_and_pinion }, &num_set_ratio) |kind, *count| {
            const d = randomDescription(g.rng.next(), kind);
            const bytes = try joltSettingsSave(allocator, &d);
            defer allocator.free(bytes);
            const ratio: f32 = @bitCast(bytes[bytes.len - 4 ..][0..4].*);
            const teeth2: f32 = @floatFromInt(d.i[2]);
            const teeth3: f32 = @floatFromInt(d.i[3]);
            const expected = if (d.i[1] == 0)
                d.f[6]
            else if (kind == .gear)
                teeth3 / teeth2 // SetRatio(inNumTeethGear1, inNumTeethGear2): float(inNumTeethGear2) / float(inNumTeethGear1)
            else
                2.0 * math.pi * teeth2 / (d.f[7] * teeth3); // 2.0f * JPH_PI * inNumTeethRack / (inRackLength * inNumTeethPinion)
            try std.testing.expect(fw.sameBits(expected, ratio));
            if (d.i[1] != 0) count.* += 1;
        }

        const sc = g.scenario(.six_dof);
        jolt.jolt_cs_run(world, &sc, out);
        try std.testing.expectEqual(@intFromEnum(Kind.six_dof), out.i[1]);
        for (0..6) |a| {
            const helper = sc.settings.i[11 + a];
            const min = switch (helper) {
                2 => -math.flt_max,
                3 => math.flt_max,
                else => sc.settings.f[18 + a],
            };
            const max = switch (helper) {
                2 => math.flt_max,
                3 => -math.flt_max,
                else => sc.settings.f[24 + a],
            };
            try std.testing.expectEqual(@intFromBool(min == -math.flt_max and max == math.flt_max), out.i[2 + 2 * a]); // IsFreeAxis
            try std.testing.expectEqual(@intFromBool(min >= max), out.i[3 + 2 * a]); // IsFixedAxis
            num_axis_helpers[@min(helper, 3)] += 1;
        }
        try std.testing.expectEqual(@as(u32, 0xC0DE0000), out.i[14]); // The first step follows
    }
    for (num_set_ratio) |n| try std.testing.expect(n > 100);
    for (num_axis_helpers) |n| try std.testing.expect(n > 300);
}

test "Constraint parity: descriptions for the other parity files" {
    // randomDescription / makeDescriptionSettings (BodyInterfaceParity.zig): the settings of a ported kind have the bytes
    // of the C++ settings, a stub kind is error.KindNotPorted, a number that is no two body kind of `Kind` (Vehicle,
    // the user types, anything else) is error.UnknownKind
    const allocator = std.testing.allocator;
    var num_ported: u32 = 0;
    for (std.enums.values(Kind)) |kind| {
        for (0..20) |seed| {
            const d = randomDescription(@intCast(0xDE5C0000 + 64 * @intFromEnum(kind) + seed), kind);
            try std.testing.expectEqual(@intFromEnum(kind), d.kind);
            var settings = makeDescriptionSettings(allocator, &d) catch |err| {
                try std.testing.expectEqual(error.KindNotPorted, err);
                try std.testing.expect(std.mem.indexOfScalar(Kind, ported_kinds, kind) == null);
                continue;
            };
            defer settings.deinit();
            try std.testing.expect(std.mem.indexOfScalar(Kind, ported_kinds, kind) != null);
            const zolt_bytes = try settingsBytes(allocator, settings.get().?);
            defer allocator.free(zolt_bytes);
            const jolt_bytes = try joltSettingsSave(allocator, &d);
            defer allocator.free(jolt_bytes);
            try std.testing.expectEqualSlices(u8, jolt_bytes, zolt_bytes);
            num_ported += 1;
        }
    }
    try std.testing.expectEqual(20 * ported_kinds.len, num_ported);
    try std.testing.expect(ported_kinds.len >= 2); // Fixed and Point
    for ([_]u32{ 9, 13, 14, 15, 16, 17, 0xFFFFFFFF }) |kind| {
        const d: CSSettings = .{ .kind = kind };
        try std.testing.expectError(error.UnknownKind, makeDescriptionSettings(allocator, &d));
    }
}

test "Constraint parity: struct layout" {
    var sizes: [12]u32 = undefined;
    jolt.jolt_cs_struct_sizes(&sizes);
    const expected = [12]u32{
        @sizeOf(CSBodyDesc),
        @sizeOf(CSSettings),
        @sizeOf(CSOp),
        @sizeOf(CSScenario),
        @sizeOf(CSOutput),
        @offsetOf(CSSettings, "user_data"),
        @offsetOf(CSSettings, "r"),
        @offsetOf(CSSettings, "f"),
        @offsetOf(CSOp, "r"),
        @offsetOf(CSScenario, "settings"),
        @offsetOf(CSScenario, "ops"),
        @offsetOf(CSOutput, "num_r"),
    };
    try std.testing.expectEqualSlices(u32, &expected, &sizes);
}
