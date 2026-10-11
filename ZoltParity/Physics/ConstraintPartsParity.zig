//! Parity tests for the constraint parts (Phase 5 Wave A2 / A3): SpringSettings, MotorSettings, CalculateSolverSteps,
//! SpringPart and the body based parts of Jolt/Physics/Constraints/ConstraintPart (Axis, Contact, AngularFriction,
//! DualAxis, Angle, Point, HingeRotation, RotationEuler, RotationQuat, SwingTwist, IndependentAxis, Gear,
//! RackAndPinion).
//!
//! A body based scenario creates two bodies through a BodyManager on both sides (every combination of static /
//! kinematic / dynamic that the part supports, random box shapes, positions, rotations, velocities, allowed DOFs and
//! mass properties: calculated, calculated inertia with a given mass, or a given rotated inertia tensor), constructs the
//! part in zeroed memory and calls, in Jolt's order of use: RestoreState / SetTotalLambda (an initial accumulated
//! impulse), CalculateConstraintProperties (every overload, springs in every mode with zero and nonzero damping and
//! bias), WarmStart with a random ratio, several SolveVelocityConstraint iterations with lambda limits (also the split
//! GetTotalLambda / ApplyLambda version of the contact part), several SolvePositionConstraint iterations with random
//! Baumgarte factors, and Deactivate. After every call both sides record the return value, the position, rotation and
//! velocities of both bodies, the members of the part (effective masses, accumulated lambdas, the cached vectors and
//! matrices that are defined for the motion types) and the SaveState bytes into a `CPOutput`, which must be identical
//! value by value (bitwise, NaNs equal).
//!
//! Nearly degenerate inputs are mixed in: parallel, anti-parallel and perpendicular axes, zero effective mass (static or
//! kinematic bodies only, a body that can only rotate with the constraint at its center of mass), zero lever arms,
//! angles at and beyond the swing twist limits, locked and free limit axes, zero lambda ranges, zero C.
//!
//! Parts that Jolt only supports for dynamic bodies (Gear, RackAndPinion, IndependentAxis with kinematic bodies, an
//! AngularFriction part without a dynamic body) trip Jolt's (and Zolt's) motion type asserts: those combinations only
//! run when asserts are off (`zig build parity -Doptimize=ReleaseFast`).
//!
//! `Coverage` counts the branches that only some inputs reach and requires each of them.
//! C ABI wrappers: ZoltParity/Physics/ConstraintPartsReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;

const Allocator = std.mem.Allocator;
const AngleConstraintPart = zolt.AngleConstraintPart;
const AngularFrictionConstraintPart = zolt.AngularFrictionConstraintPart;
const AxisConstraintPart = zolt.AxisConstraintPart;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyManager = zolt.BodyManager;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const CalculateSolverSteps = zolt.CalculateSolverSteps;
const ContactConstraintPart = zolt.ContactConstraintPart;
const Core = zolt.Core;
const DualAxisConstraintPart = zolt.DualAxisConstraintPart;
const DummyCalculateSolverSteps = zolt.DummyCalculateSolverSteps;
const Float3 = zolt.Float3;
const GearConstraintPart = zolt.GearConstraintPart;
const HingeRotationConstraintPart = zolt.HingeRotationConstraintPart;
const IndependentAxisConstraintPart = zolt.IndependentAxisConstraintPart;
const Mat44 = zolt.Mat44;
const Matrix = zolt.Matrix;
const MotionType = zolt.MotionType;
const MotorSettings = zolt.MotorSettings;
const MotorState = zolt.MotorState;
const ObjectLayer = zolt.ObjectLayer;
const PhysicsSettings = zolt.PhysicsSettings;
const PointConstraintPart = zolt.PointConstraintPart;
const Quat = zolt.Quat;
const RackAndPinionConstraintPart = zolt.RackAndPinionConstraintPart;
const Real = zolt.Real;
const RotationEulerConstraintPart = zolt.RotationEulerConstraintPart;
const RotationQuatConstraintPart = zolt.RotationQuatConstraintPart;
const RVec3 = zolt.RVec3;
const SpringMode = zolt.SpringMode;
const SpringPart = zolt.SpringPart;
const SpringSettings = zolt.SpringSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const SwingTwistConstraintPart = zolt.SwingTwistConstraintPart;
const SwingType = zolt.SwingType;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Vector = zolt.Vector;
const math = zolt.math;

const io = std.testing.io;

/// Number of random scenarios per body based part
const scenario_iterations = 20_000;

/// The C++ reference functions, see ConstraintPartsReference.cpp
const jolt = struct {
    extern fn jolt_cp_world_create() *anyopaque;
    extern fn jolt_cp_world_destroy(world: *anyopaque) void;
    extern fn jolt_cp_spring_part(f: *const [7]f32, op: u32, mode: u32, out: *[5]f32) void;
    extern fn jolt_cp_spring_settings(mode: u32, frequency: f32, damping: f32, restore: *const [9]u8, out_bytes: *[9]u8, out_info: *[6]u32, out_restored: *[3]f32) void;
    extern fn jolt_cp_motor_settings(constructor: u32, mode: u32, f: *const [7]f32, setter: u32, restore: *const [25]u8, out_bytes: *[25]u8, out_f: *[14]f32, out_info: *[5]u32) void;
    extern fn jolt_cp_motor_state(state: c_int) u32;
    extern fn jolt_cp_solver_steps(default_velocity: u32, default_position: u32, overrides: [*]const u32, count: u32, out_steps: *[4]u32) void;
    extern fn jolt_cp_axis(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_angle(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_contact(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_angular_friction(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_dual_axis(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_point(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_hinge_rotation(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_rotation_euler(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_rotation_quat(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_swing_twist_clamp(f: *const [16]f32, swing_type: u32, out_f: *[8]f32, out_i: *[2]u32) void;
    extern fn jolt_cp_swing_twist(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_independent_axis(world: *anyopaque, input: *const CPInput, output: *CPOutput) void;
    extern fn jolt_cp_gear(world: *anyopaque, input: *const CPInput, rack: u32, output: *CPOutput) void;
    extern fn jolt_cp_contact_sizes(out_contact: *[9]u32, out_angular_friction: *[9]u32) void;
};

/// Must match CPBodyDesc in ConstraintPartsReference.cpp
const CPBodyDesc = extern struct {
    position: [3]Real,
    rotation: [4]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    half_extent: [3]f32,
    mass: f32,
    inertia_multiplier: f32,
    inertia: [16]f32,
    motion_type: u32,
    allowed_dofs: u32,
    override_mass_properties: u32,
};

/// Must match CPInput in ConstraintPartsReference.cpp
const CPInput = extern struct {
    body: [2]CPBodyDesc,
    f: [64]f32 = @splat(0),
    i: [16]u32 = @splat(0),
};

const max_r = 256;
const max_f = 4096;
const max_i = 1024;

/// Must match CPOutput in ConstraintPartsReference.cpp
const CPOutput = extern struct {
    r: [max_r]Real,
    f: [max_f]f32,
    i: [max_i]u32,
    num_r: u32,
    num_f: u32,
    num_i: u32,
};

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

fn storeMat44(m: Mat44, out: []f32) void {
    for (0..4) |c| {
        const col = m.getColumn4(@intCast(c));
        out[4 * c + 0] = col.getX();
        out[4 * c + 1] = col.getY();
        out[4 * c + 2] = col.getZ();
        out[4 * c + 3] = col.getW();
    }
}

/// The Rec class of ConstraintPartsReference.cpp: appends values to the output record
const Rec = struct {
    out: *CPOutput,
    allocator: Allocator,

    fn init(out: *CPOutput, allocator: Allocator) Rec {
        out.num_r = 0;
        out.num_f = 0;
        out.num_i = 0;
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

    fn f3(self: *Rec, v: Float3) void {
        self.f(v.x);
        self.f(v.y);
        self.f(v.z);
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

    fn v2(self: *Rec, v: Vector(2)) void {
        self.f(v.getComponent(0));
        self.f(v.getComponent(1));
    }

    fn m22(self: *Rec, m: Matrix(2, 2)) void {
        for (0..2) |c| {
            for (0..2) |row|
                self.f(m.get(@intCast(row), @intCast(c)));
        }
    }

    /// All floats of an extern struct (the templated contact parts, which have Jolt's layout)
    fn bytes(self: *Rec, part: anytype) void {
        const floats: []align(1) const f32 = std.mem.bytesAsSlice(f32, std.mem.asBytes(part));
        for (floats) |v|
            self.f(v);
    }

    fn bodyState(self: *Rec, body: *const Body) void {
        const p = body.getCenterOfMassPosition();
        self.r(p.getX());
        self.r(p.getY());
        self.r(p.getZ());
        self.q(body.getRotation());
        self.v3(body.getLinearVelocity());
        self.v3(body.getAngularVelocity());
    }

    fn state(self: *Rec, part: anytype) !void {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        const data = recorder.getData();
        self.i(@intCast(data.len));
        for (data) |byte|
            self.i(byte);
    }

    /// Start of a step: the step id, the return value and both bodies
    fn step(self: *Rec, id: u32, result: u32, body1: *const Body, body2: *const Body) void {
        self.i(0xC0DE0000 | id);
        self.i(result);
        self.bodyState(body1);
        self.bodyState(body2);
    }
};

/// RestoreFloats of ConstraintPartsReference.cpp
fn restoreFloats(allocator: Allocator, part: anytype, values: []const f32) !void {
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    for (values) |v|
        recorder.recorder().write(v);
    part.restoreState(recorder.recorder());
}

/// Broad phase layer = object layer (CPLayers)
const CPLayers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(CPLayers),

    pub fn getNumBroadPhaseLayers(self: *const CPLayers) u32 {
        _ = self;
        return 1;
    }

    pub fn getBroadPhaseLayer(self: *const CPLayers, layer: ObjectLayer) BroadPhaseLayer {
        _ = self;
        return .init(@intCast(layer));
    }
};

/// A BodyManager on both sides, and the output records
const World = struct {
    allocator: Allocator,
    layers: CPLayers = .{},
    body_manager: BodyManager = .empty,
    jolt_world: *anyopaque = undefined,
    zolt_out: *CPOutput = undefined,
    jolt_out: *CPOutput = undefined,

    fn init(self: *World, allocator: Allocator) !void {
        self.* = .{ .allocator = allocator };
        try self.body_manager.init(allocator, io, 8, 0, &self.layers.base);
        errdefer self.body_manager.deinit();
        self.zolt_out = try allocator.create(CPOutput);
        errdefer allocator.destroy(self.zolt_out);
        self.jolt_out = try allocator.create(CPOutput);
        self.jolt_world = jolt.jolt_cp_world_create();
    }

    fn deinit(self: *World) void {
        jolt.jolt_cp_world_destroy(self.jolt_world);
        self.allocator.destroy(self.jolt_out);
        self.allocator.destroy(self.zolt_out);
        self.body_manager.deinit();
    }

    /// CreateBody of ConstraintPartsReference.cpp
    fn createBody(self: *World, desc: *const CPBodyDesc) !*Body {
        const shape = try BoxShape.create(self.allocator, load3(&desc.half_extent), .{});
        var settings = BodyCreationSettings.initWithShape(shape.asShape(), loadR3(&desc.position), loadQuat(&desc.rotation), @enumFromInt(desc.motion_type), 0);
        defer settings.deinit();
        settings.linear_velocity = load3(&desc.linear_velocity);
        settings.angular_velocity = load3(&desc.angular_velocity);
        settings.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
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

    /// DestroyBody of ConstraintPartsReference.cpp
    fn destroyBody(self: *World, body: *Body) void {
        const id = body.getID();
        self.body_manager.deactivateBodies(&.{id});
        body.setInBroadPhaseInternal(false);
        self.body_manager.destroyBodies(&.{id});
    }
};

/// Compare the records of both sides
fn compareOutputs(check: *Checker, name: []const u8, input: *const CPInput, zolt_out: *const CPOutput, jolt_out: *const CPOutput) void {
    const counts_ok = zolt_out.num_r == jolt_out.num_r and zolt_out.num_f == jolt_out.num_f and zolt_out.num_i == jolt_out.num_i;
    const overflow = zolt_out.num_r > max_r or zolt_out.num_f > max_f or zolt_out.num_i > max_i;
    var first_mismatch: ?struct { kind: u8, index: usize } = null;
    if (counts_ok and !overflow) {
        for (0..zolt_out.num_r) |k| {
            if (!fw.sameBits(zolt_out.r[k], jolt_out.r[k])) {
                first_mismatch = .{ .kind = 'r', .index = k };
                break;
            }
        }
        if (first_mismatch == null) for (0..zolt_out.num_f) |k| {
            if (!fw.sameBits(zolt_out.f[k], jolt_out.f[k])) {
                first_mismatch = .{ .kind = 'f', .index = k };
                break;
            }
        };
        if (first_mismatch == null) for (0..zolt_out.num_i) |k| {
            if (zolt_out.i[k] != jolt_out.i[k]) {
                first_mismatch = .{ .kind = 'i', .index = k };
                break;
            }
        };
    }
    if (!counts_ok or overflow or first_mismatch != null) {
        if (check.mismatches < 5) {
            std.debug.print("{s}: mismatch (counts zolt {d}/{d}/{d} jolt {d}/{d}/{d})", .{ name, zolt_out.num_r, zolt_out.num_f, zolt_out.num_i, jolt_out.num_r, jolt_out.num_f, jolt_out.num_i });
            if (first_mismatch) |m| {
                switch (m.kind) {
                    'r' => std.debug.print(" first difference r[{d}]: zolt {d} jolt {d}", .{ m.index, zolt_out.r[m.index], jolt_out.r[m.index] }),
                    'f' => std.debug.print(" first difference f[{d}]: zolt {e} jolt {e}", .{ m.index, zolt_out.f[m.index], jolt_out.f[m.index] }),
                    else => std.debug.print(" first difference i[{d}]: zolt {x} jolt {x}", .{ m.index, zolt_out.i[m.index], jolt_out.i[m.index] }),
                }
            }
            std.debug.print("\n  input: {any}\n", .{input.*});
        }
        check.mismatches += 1;
    }
}

/// Branches that only some inputs reach (each must be reached `min_hits` times)
const Coverage = struct {
    const min_hits = 20;
    names: [16][]const u8 = undefined,
    hits: [16]u32 = @splat(0),
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

/// Input generator
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

    /// A random unit vector, sometimes axis aligned
    fn unit(self: *Gen) Vec3 {
        return switch (self.index(10)) {
            0 => Vec3.axisX(),
            1 => Vec3.axisY().negate(),
            2 => Vec3.axisZ(),
            else => blk: {
                while (true) {
                    const v = self.vec(1);
                    if (v.lengthSq() > 0.01) break :blk v.normalized();
                }
            },
        };
    }

    /// A lever arm: zero or at least 0.05 long
    fn arm(self: *Gen) Vec3 {
        if (self.oneIn(8)) return Vec3.zero();
        while (true) {
            const v = self.vec(2);
            if (v.length() > 0.05) return v;
        }
    }

    fn quat(self: *Gen) Quat {
        if (self.oneIn(8)) return Quat.identity();
        while (true) {
            const v = Vec4.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1));
            if (v.lengthSq() > 0.01) return Quat.fromVec4(v.normalized());
        }
    }

    fn lambda(self: *Gen) f32 {
        return if (self.oneIn(3)) 0.0 else self.float(-10, 10);
    }

    fn ratio(self: *Gen) f32 {
        return switch (self.index(4)) {
            0 => 1.0,
            1 => 0.0,
            else => self.float(0, 2),
        };
    }

    fn baumgarte(self: *Gen) f32 {
        return if (self.oneIn(4)) 0.2 else self.float(0, 1);
    }

    /// Lambda limits: unlimited, one sided, symmetric, a zero range or random
    fn limits(self: *Gen, out: []f32) void {
        switch (self.index(6)) {
            0 => {
                out[0] = -math.flt_max;
                out[1] = math.flt_max;
            },
            1 => {
                out[0] = 0.0;
                out[1] = math.flt_max;
            },
            2 => {
                out[0] = -math.flt_max;
                out[1] = 0.0;
            },
            3 => {
                const l = self.float(0, 5);
                out[0] = -l;
                out[1] = l;
            },
            4 => {
                const l = self.float(-1, 1);
                out[0] = l;
                out[1] = l;
            },
            else => {
                const a = self.float(-5, 5);
                const b = self.float(-5, 5);
                out[0] = @min(a, b);
                out[1] = @max(a, b);
            },
        }
    }

    fn bodyDesc(self: *Gen, motion_type: MotionType) CPBodyDesc {
        var desc: CPBodyDesc = undefined;
        const far: Real = if (Core.double_precision and self.oneIn(4)) 1.0e6 else 0.0;
        desc.position = .{ far + self.float(-10, 10), self.float(-10, 10), far + self.float(-10, 10) };
        storeQuat(self.quat(), &desc.rotation);
        store3(if (self.oneIn(6)) Vec3.zero() else self.vec(5), &desc.linear_velocity);
        store3(if (self.oneIn(6)) Vec3.zero() else self.vec(3), &desc.angular_velocity);
        desc.half_extent = .{ self.float(0.1, 2), self.float(0.1, 2), self.float(0.1, 2) };
        desc.mass = self.float(0.5, 50);
        desc.inertia_multiplier = if (self.oneIn(2)) 1.0 else self.float(0.5, 2);
        const rotation = Mat44.rotationQuat(self.quat());
        const diagonal = Vec3.init(self.float(0.05, 5), self.float(0.05, 5), self.float(0.05, 5));
        storeMat44(rotation.mul(Mat44.scaleVec3(diagonal)).mul(rotation.transposed()), &desc.inertia);
        desc.motion_type = @intFromEnum(motion_type);
        desc.allowed_dofs = if (self.oneIn(2)) 0b111111 else @intCast(1 + self.index(63));
        desc.override_mass_properties = @intCast(self.index(3));
        return desc;
    }

    fn motionType(self: *Gen) MotionType {
        return @enumFromInt(self.index(3));
    }

    /// A spring mode byte: mostly one of the named ESpringMode values, sometimes another uint8 (a valid ESpringMode in
    /// Jolt, e.g. a restored mode byte, that no case of a switch on the mode matches)
    fn springMode(self: *Gen) u32 {
        return if (self.oneIn(10)) @intCast(3 + self.index(253)) else @intCast(self.index(3));
    }

    /// A pair of motion types (every combination, or only those with a dynamic body, or only dynamic bodies)
    fn motionTypes(self: *Gen, comptime which: enum { all, any_dynamic, dynamic_or_kinematic }) [2]MotionType {
        while (true) {
            const pair = [2]MotionType{ self.motionType(), self.motionType() };
            switch (which) {
                .all => return pair,
                .any_dynamic => if (pair[0] == .dynamic or pair[1] == .dynamic) return pair,
                .dynamic_or_kinematic => if (pair[0] != .static and pair[1] != .static) return pair,
            }
        }
    }

    fn bodies(self: *Gen, input: *CPInput, pair: [2]MotionType) void {
        input.body = .{ self.bodyDesc(pair[0]), self.bodyDesc(pair[1]) };
    }

    /// Spring scenario values at `f` (see RunSpringScenario): bias, C, frequency / stiffness, damping, dt, warm start
    /// ratio, min lambda, max lambda, C (first position step), initial lambda, C (next position steps), baumgarte. Sets
    /// the overload and spring mode in `i`.
    fn springScenario(self: *Gen, f: []f32, i: []u32) void {
        const overload: u32 = @intCast(self.index(6));
        const mode = self.springMode();
        i[0] = overload;
        i[1] = mode;
        f[0] = if (self.oneIn(3)) 0.0 else self.float(-1, 1);
        f[1] = if (self.oneIn(4)) 0.0 else self.float(-0.5, 0.5);
        var frequency: f32 = if (self.oneIn(4)) 0.0 else switch (mode) {
            0 => self.float(0.1, 20),
            1 => self.float(1, 1.0e4),
            else => self.float(1, 500),
        };
        const damping: f32 = if (self.oneIn(3)) 0.0 else self.float(0, 10);
        switch (overload) {
            1 => {
                // Frequency and damping: frequency in Hz
                frequency = if (frequency == 0.0) 0.0 else self.float(0.1, 20);
            },
            4 => {
                // A motor needs stiffness or damping
                if (!SpringSettings.init(@enumFromInt(mode), frequency, damping).hasStiffnessOrDamping()) {
                    frequency = if (mode == 0) self.float(0.1, 20) else self.float(1, 500);
                }
            },
            else => {},
        }
        f[2] = frequency;
        f[3] = damping;
        f[4] = if (self.oneIn(2)) 1.0 / 60.0 else self.float(1.0 / 240.0, 1.0 / 30.0);
        f[5] = self.ratio();
        self.limits(f[6..8]);
        f[8] = if (self.oneIn(4)) 0.0 else self.float(-0.5, 0.5);
        f[9] = self.lambda();
        f[10] = self.float(-0.5, 0.5);
        f[11] = self.baumgarte();
        i[2] = @intCast(self.index(5));
        i[3] = @intCast(self.index(4));
    }

    /// A random symmetric positive semi definite inverse inertia (or zero)
    fn invInertia(self: *Gen, out: []f32) void {
        if (self.oneIn(6)) {
            storeMat44(Mat44.zero(), out);
            return;
        }
        const rotation = Mat44.rotationQuat(self.quat());
        const diagonal = Vec3.init(self.float(0, 5), if (self.oneIn(5)) 0.0 else self.float(0, 5), self.float(0, 5));
        storeMat44(rotation.mul(Mat44.scaleVec3(diagonal)).mul(rotation.transposed()), out);
    }
};

/// Run one scenario on both sides and compare
fn compareScenario(world: *World, check: *Checker, name: []const u8, input: *const CPInput, zolt_run: anytype, jolt_run: anytype) !void {
    jolt_run.run(world.jolt_world, input, world.jolt_out);
    try zolt_run.run(world, input, world.zolt_out);
    compareOutputs(check, name, input, world.zolt_out, world.jolt_out);
}

/// The two bodies of a scenario (BodyPairScope)
const BodyPair = struct {
    world: *World,
    body1: *Body,
    body2: *Body,

    fn init(world: *World, input: *const CPInput) !BodyPair {
        const body1 = try world.createBody(&input.body[0]);
        errdefer world.destroyBody(body1);
        const body2 = try world.createBody(&input.body[1]);
        return .{ .world = world, .body1 = body1, .body2 = body2 };
    }

    fn deinit(self: *BodyPair) void {
        self.world.destroyBody(self.body2);
        self.world.destroyBody(self.body1);
    }
};

fn dumpSpring(rec: *Rec, spring: *const SpringPart) void {
    rec.f(spring.bias);
    rec.f(spring.softness);
    rec.b(spring.isActive());
}

fn dumpAxis(rec: *Rec, part: *const AxisConstraintPart, body1: *const Body, body2: *const Body) !void {
    if (!body1.isStatic()) rec.f3(part.r1_plus_u_x_axis);
    if (!body2.isStatic()) rec.f3(part.r2_x_axis);
    if (body1.isDynamic()) rec.f3(part.inv_i1_r1_plus_u_x_axis);
    if (body2.isDynamic()) rec.f3(part.inv_i2_r2_x_axis);
    rec.f(part.effective_mass);
    dumpSpring(rec, &part.spring_part);
    rec.f(part.getTotalLambda());
    rec.b(part.isActive());
    try rec.state(part);
}

fn dumpAngle(rec: *Rec, part: *const AngleConstraintPart) !void {
    rec.v3(part.inv_i1_axis);
    rec.v3(part.inv_i2_axis);
    rec.f(part.effective_mass);
    dumpSpring(rec, &part.spring_part);
    rec.f(part.getTotalLambda());
    rec.b(part.isActive());
    try rec.state(part);
}

/// RunSpringScenario of ConstraintPartsReference.cpp
fn runSpringScenario(comptime Part: type, rec: *Rec, body1: *Body, body2: *Body, input: *const CPInput, first: usize, ops: anytype) !void {
    const f = input.f[first..];
    const ui = &input.i;

    var part = std.mem.zeroes(Part);
    try restoreFloats(rec.allocator, &part, f[9..10]);
    rec.step(0, 0, body1, body2);
    try ops.dump(rec, &part);

    const settings = SpringSettings.init(@enumFromInt(ui[1]), f[2], f[3]);
    ops.calc(&part, ui[0], f[4], f[0], f[1], f[2], f[3], &settings);
    rec.step(1, 0, body1, body2);
    try ops.dump(rec, &part);

    if (ui[4] != 0)
        return; // Static vs static: only the properties

    ops.warmStart(&part, f[5]);
    rec.step(2, 0, body1, body2);
    try ops.dump(rec, &part);

    for (0..ui[2]) |_| {
        const result = ops.solveVel(&part, f[6], f[7]);
        rec.step(3, @intFromBool(result), body1, body2);
        try ops.dump(rec, &part);
    }

    for (0..ui[3]) |k| {
        const result = ops.solvePos(&part, if (k == 0) f[8] else f[10], f[11]);
        rec.step(4, @intFromBool(result), body1, body2);
        try ops.dump(rec, &part);
    }

    part.deactivate();
    rec.step(5, 0, body1, body2);
    try ops.dump(rec, &part);
}

const AxisRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const f = &input.f;
        const Ops = struct {
            body1: *Body,
            body2: *Body,
            r1_plus_u: Vec3,
            r2: Vec3,
            axis: Vec3,

            fn calc(self: @This(), part: *AxisConstraintPart, overload: u32, dt: f32, bias: f32, c: f32, frequency: f32, damping: f32, settings: *const SpringSettings) void {
                switch (overload) {
                    0 => part.calculateConstraintProperties(self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, .{ .bias = bias }),
                    1 => part.calculateConstraintPropertiesWithFrequencyAndDamping(dt, self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, bias, c, frequency, damping),
                    2 => part.calculateConstraintPropertiesWithStiffnessAndDamping(dt, self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, bias, c, frequency, damping),
                    3 => part.calculateConstraintPropertiesWithSettingsForLimit(dt, self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, bias, c, settings),
                    4 => part.calculateConstraintPropertiesWithSettingsForMotor(dt, self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, bias, c, settings),
                    else => part.calculateConstraintProperties(self.body1, self.r1_plus_u, self.body2, self.r2, self.axis, .{}),
                }
            }

            fn warmStart(self: @This(), part: *AxisConstraintPart, ratio: f32) void {
                part.warmStart(self.body1, self.body2, self.axis, ratio);
            }

            fn solveVel(self: @This(), part: *AxisConstraintPart, min: f32, max: f32) bool {
                return part.solveVelocityConstraint(self.body1, self.body2, self.axis, min, max);
            }

            fn solvePos(self: @This(), part: *AxisConstraintPart, c: f32, baumgarte: f32) bool {
                return part.solvePositionConstraint(self.body1, self.body2, self.axis, c, baumgarte);
            }

            fn dump(self: @This(), r: *Rec, part: *const AxisConstraintPart) !void {
                try dumpAxis(r, part, self.body1, self.body2);
            }
        };
        const ops: Ops = .{ .body1 = pair.body1, .body2 = pair.body2, .r1_plus_u = load3(f[0..3]), .r2 = load3(f[3..6]), .axis = load3(f[6..9]) };
        try runSpringScenario(AxisConstraintPart, &rec, pair.body1, pair.body2, input, 9, ops);
    }
};

const AngleRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const Ops = struct {
            body1: *Body,
            body2: *Body,
            axis: Vec3,

            fn calc(self: @This(), part: *AngleConstraintPart, overload: u32, dt: f32, bias: f32, c: f32, frequency: f32, damping: f32, settings: *const SpringSettings) void {
                switch (overload) {
                    0 => part.calculateConstraintProperties(self.body1, self.body2, self.axis, .{ .bias = bias }),
                    1 => part.calculateConstraintPropertiesWithFrequencyAndDamping(dt, self.body1, self.body2, self.axis, bias, c, frequency, damping),
                    2 => part.calculateConstraintPropertiesWithStiffnessAndDamping(dt, self.body1, self.body2, self.axis, bias, c, frequency, damping),
                    3 => part.calculateConstraintPropertiesWithSettingsForLimit(dt, self.body1, self.body2, self.axis, bias, c, settings),
                    4 => part.calculateConstraintPropertiesWithSettingsForMotor(dt, self.body1, self.body2, self.axis, bias, c, settings),
                    else => part.calculateConstraintProperties(self.body1, self.body2, self.axis, .{}),
                }
            }

            fn warmStart(self: @This(), part: *AngleConstraintPart, ratio: f32) void {
                part.warmStart(self.body1, self.body2, ratio);
            }

            fn solveVel(self: @This(), part: *AngleConstraintPart, min: f32, max: f32) bool {
                return part.solveVelocityConstraint(self.body1, self.body2, self.axis, min, max);
            }

            fn solvePos(self: @This(), part: *AngleConstraintPart, c: f32, baumgarte: f32) bool {
                return part.solvePositionConstraint(self.body1, self.body2, c, baumgarte);
            }

            fn dump(self: @This(), r: *Rec, part: *const AngleConstraintPart) !void {
                _ = self;
                try dumpAngle(r, part);
            }
        };
        const ops: Ops = .{ .body1 = pair.body1, .body2 = pair.body2, .axis = load3(input.f[0..3]) };
        try runSpringScenario(AngleConstraintPart, &rec, pair.body1, pair.body2, input, 3, ops);
    }
};

fn runContact(comptime type1: MotionType, comptime type2: MotionType, rec: *Rec, body1: *Body, body2: *Body, input: *const CPInput) !void {
    const Part = ContactConstraintPart(type1, type2);
    const f = &input.f;
    const ui = &input.i;

    const inv_m1 = f[0];
    const inv_m2 = f[17];
    const inv_i1 = loadMat44(f[1..17]);
    const inv_i2 = loadMat44(f[18..34]);
    const r1_plus_u = load3(f[34..37]);
    const r2 = load3(f[37..40]);
    const axis = load3(f[40..43]);
    var v = [4]Vec3{ body1.getLinearVelocity(), body1.getAngularVelocity(), body2.getLinearVelocity(), body2.getAngularVelocity() };

    const Dump = struct {
        fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, vel: *const [4]Vec3, part: *const Part) void {
            r.step(id, result, b1, b2);
            for (vel) |x| r.v3(x);
            r.bytes(part);
            r.f(part.getTotalLambda());
            r.b(part.isActive());
        }
    };

    var part = std.mem.zeroes(Part);
    part.setTotalLambda(f[44]);
    Dump.dump(rec, 0, 0, body1, body2, &v, &part);

    if (ui[0] != 0)
        part.calculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, axis, .{ .bias = f[43] })
    else
        part.calculateConstraintProperties(inv_m1, inv_i1, r1_plus_u, inv_m2, inv_i2, r2, axis, .{});
    Dump.dump(rec, 1, 0, body1, body2, &v, &part);

    var result = part.warmStart(&v[0], &v[1], &v[2], &v[3], inv_m1, inv_m2, axis, f[45]);
    Dump.dump(rec, 2, @intFromBool(result), body1, body2, &v, &part);

    if (comptime type1 != .static or type2 != .static) {
        for (0..ui[2]) |_| {
            if (ui[5] != 0) {
                var total_lambda = part.solveVelocityConstraintGetTotalLambda(v[0], v[1], v[2], v[3], axis);
                rec.f(total_lambda);
                total_lambda = math.clamp(total_lambda, f[46], f[47]);
                result = part.solveVelocityConstraintApplyLambda(&v[0], &v[1], &v[2], &v[3], inv_m1, inv_m2, axis, total_lambda);
            } else result = part.solveVelocityConstraint(&v[0], &v[1], &v[2], &v[3], inv_m1, inv_m2, axis, f[46], f[47]);
            Dump.dump(rec, 3, @intFromBool(result), body1, body2, &v, &part);
        }
    }

    for (0..ui[3]) |k| {
        result = part.solvePositionConstraint(body1, inv_m1, body2, inv_m2, axis, if (k == 0) f[48] else f[50], f[49]);
        Dump.dump(rec, 4, @intFromBool(result), body1, body2, &v, &part);
    }

    part.deactivate();
    Dump.dump(rec, 5, 0, body1, body2, &v, &part);
}

fn runAngularFriction(comptime type1: MotionType, comptime type2: MotionType, rec: *Rec, body1: *Body, body2: *Body, input: *const CPInput) !void {
    const Part = AngularFrictionConstraintPart(type1, type2);
    const f = &input.f;
    const ui = &input.i;

    const inv_i1 = loadMat44(f[0..16]);
    const inv_i2 = loadMat44(f[16..32]);
    const axis = load3(f[32..35]);
    var w = [2]Vec3{ body1.getAngularVelocity(), body2.getAngularVelocity() };

    const Dump = struct {
        fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, vel: *const [2]Vec3, part: *const Part) void {
            r.step(id, result, b1, b2);
            for (vel) |x| r.v3(x);
            r.bytes(part);
            r.f(part.getTotalLambda());
            r.b(part.isActive());
        }
    };

    var part = std.mem.zeroes(Part);
    part.setTotalLambda(f[36]);
    Dump.dump(rec, 0, 0, body1, body2, &w, &part);

    if (ui[0] != 0)
        part.calculateConstraintProperties(inv_i1, inv_i2, axis, .{ .bias = f[35] })
    else
        part.calculateConstraintProperties(inv_i1, inv_i2, axis, .{});
    Dump.dump(rec, 1, 0, body1, body2, &w, &part);

    var result = part.warmStart(&w[0], &w[1], f[37]);
    Dump.dump(rec, 2, @intFromBool(result), body1, body2, &w, &part);

    if (comptime type1 != .static or type2 != .static) {
        for (0..ui[2]) |_| {
            result = part.solveVelocityConstraint(&w[0], &w[1], axis, f[38], f[39]);
            Dump.dump(rec, 3, @intFromBool(result), body1, body2, &w, &part);
        }
    }

    part.deactivate();
    Dump.dump(rec, 5, 0, body1, body2, &w, &part);
}

const ContactRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        switch (pair.body1.getMotionType()) {
            inline else => |type1| switch (pair.body2.getMotionType()) {
                inline else => |type2| try runContact(type1, type2, &rec, pair.body1, pair.body2, input),
            },
        }
    }
};

const AngularFrictionRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        switch (pair.body1.getMotionType()) {
            inline else => |type1| switch (pair.body2.getMotionType()) {
                inline else => |type2| try runAngularFriction(type1, type2, &rec, pair.body1, pair.body2, input),
            },
        }
    }
};

const DualAxisRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;
        const r1_plus_u = load3(f[0..3]);
        const r2 = load3(f[3..6]);
        const n1 = load3(f[6..9]);
        const n2 = load3(f[9..12]);

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const DualAxisConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.v3(part.r1_plus_u_x_n1);
                r.v3(part.r1_plus_u_x_n2);
                r.v3(part.r2_x_n1);
                r.v3(part.r2_x_n2);
                if (b1.isDynamic()) {
                    r.v3(part.inv_i1_r1_plus_u_x_n1);
                    r.v3(part.inv_i1_r1_plus_u_x_n2);
                }
                if (b2.isDynamic()) {
                    r.v3(part.inv_i2_r2_x_n1);
                    r.v3(part.inv_i2_r2_x_n2);
                }
                r.m22(part.effective_mass);
                r.v2(part.getTotalLambda().*);
                r.b(part.isActive());
                try r.state(part);
            }
        };

        var part = std.mem.zeroes(DualAxisConstraintPart);
        part.setTotalLambda(.{ .f32s = .{ f[12], f[13] } });
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), r1_plus_u, body2, Mat44.rotationQuat(body2.getRotation()), r2, n1, n2);
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, n1, n2, f[14]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2, n1, n2);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |k| {
            const result = part.solvePositionConstraint(body1, body2, load3(if (k == 0) f[15..18] else f[19..22]), n1, n2, f[18]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

const PointRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const PointConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.v3(part.r1);
                r.v3(part.r2);
                if (b1.isDynamic()) r.m44(part.inv_i1_r1x);
                if (b2.isDynamic()) r.m44(part.inv_i2_r2x);
                r.m44(part.effective_mass);
                r.v3(part.getTotalLambda());
                r.b(part.isActive());
                try r.state(part);
            }
        };

        var part = std.mem.zeroes(PointConstraintPart);
        try restoreFloats(rec.allocator, &part, f[6..9]);
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), load3(f[0..3]), body2, Mat44.rotationQuat(body2.getRotation()), load3(f[3..6]));
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, f[9]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, f[10]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

const HingeRotationRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const HingeRotationConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.v3(part.a1);
                r.v3(part.b2);
                r.v3(part.c2);
                r.m44(part.inv_i1);
                r.m44(part.inv_i2);
                r.v3(part.b2_x_a1);
                r.v3(part.c2_x_a1);
                r.m22(part.effective_mass);
                r.v2(part.getTotalLambda().*);
                try r.state(part);
            }
        };

        var part = std.mem.zeroes(HingeRotationConstraintPart);
        try restoreFloats(rec.allocator, &part, f[6..8]);
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), load3(f[0..3]), body2, Mat44.rotationQuat(body2.getRotation()), load3(f[3..6]));
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, f[8]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, f[9]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

const RotationEulerRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const RotationEulerConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.m44(part.inv_i1);
                r.m44(part.inv_i2);
                r.m44(part.effective_mass);
                r.v3(part.getTotalLambda());
                r.b(part.isActive());
                try r.state(part);
            }
        };

        const from_bodies = RotationEulerConstraintPart.getInvInitialOrientation(body1, body2);
        const xy = RotationEulerConstraintPart.getInvInitialOrientationXY(load3(f[0..3]), load3(f[3..6]), load3(f[6..9]), load3(f[9..12]));
        const xz = RotationEulerConstraintPart.getInvInitialOrientationXZ(load3(f[0..3]), load3(f[3..6]), load3(f[6..9]), load3(f[9..12]));
        rec.q(from_bodies);
        rec.q(xy);
        rec.q(xz);
        const inv_initial_orientation = switch (ui[0]) {
            0 => from_bodies,
            1 => xy,
            2 => xz,
            else => loadQuat(f[17..21]),
        };

        var part = std.mem.zeroes(RotationEulerConstraintPart);
        try restoreFloats(rec.allocator, &part, f[12..15]);
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()));
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, f[15]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, inv_initial_orientation, f[16]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

const RotationQuatRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const RotationQuatConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.m44(part.inv_i1_jpt);
                r.m44(part.inv_i2_jpt);
                r.m44(part.effective_mass);
                r.m44(part.effective_mass_jp);
                r.v3(part.getTotalLambda());
                r.b(part.isActive());
                try r.state(part);
            }
        };

        const from_bodies = RotationQuatConstraintPart.getInvInitialOrientation(body1, body2);
        rec.q(from_bodies);
        const inv_initial_orientation = if (ui[0] == 0) from_bodies else loadQuat(f[0..4]);

        var part = std.mem.zeroes(RotationQuatConstraintPart);
        try restoreFloats(rec.allocator, &part, f[4..7]);
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()), inv_initial_orientation);
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, f[7]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, inv_initial_orientation, f[8]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

fn dumpSwingTwist(rec: *Rec, part: *const SwingTwistConstraintPart) !void {
    rec.i(part.rotation_flags);
    rec.i(@intFromEnum(part.getSwingType()));
    rec.f(part.sin_twist_half_min_angle);
    rec.f(part.sin_twist_half_max_angle);
    rec.f(part.cos_twist_half_min_angle);
    rec.f(part.cos_twist_half_max_angle);
    rec.f(part.swing_y_half_min_angle);
    rec.f(part.swing_y_half_max_angle);
    rec.f(part.swing_z_half_min_angle);
    rec.f(part.swing_z_half_max_angle);
    rec.f(part.sin_swing_y_half_min_angle);
    rec.f(part.sin_swing_y_half_max_angle);
    rec.f(part.sin_swing_z_half_min_angle);
    rec.f(part.sin_swing_z_half_max_angle);
    rec.f(part.cos_swing_y_half_min_angle);
    rec.f(part.cos_swing_y_half_max_angle);
    rec.f(part.cos_swing_z_half_min_angle);
    rec.f(part.cos_swing_z_half_max_angle);
    rec.v3(part.world_space_swing_limit_y_rotation_axis);
    rec.v3(part.world_space_swing_limit_z_rotation_axis);
    rec.v3(part.world_space_twist_limit_rotation_axis);
    try dumpAngle(rec, &part.swing_limit_y_constraint_part);
    try dumpAngle(rec, &part.swing_limit_z_constraint_part);
    try dumpAngle(rec, &part.twist_limit_constraint_part);
    rec.f(part.getTotalSwingYLambda());
    rec.f(part.getTotalSwingZLambda());
    rec.f(part.getTotalTwistLambda());
    rec.b(part.isActive());
    try rec.state(part);
}

const SwingTwistRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;
        const constraint_rotation = loadQuat(f[6..10]);
        const constraint_to_body1 = loadQuat(f[10..14]);
        const constraint_to_body2 = loadQuat(f[14..18]);

        var part = std.mem.zeroes(SwingTwistConstraintPart);
        part.setSwingType(@enumFromInt(ui[0]));
        part.setLimits(f[0], f[1], f[2], f[3], f[4], f[5]);
        try restoreFloats(rec.allocator, &part, f[18..21]);
        rec.step(0, 0, body1, body2);
        try dumpSwingTwist(&rec, &part);

        part.calculateConstraintProperties(body1, body2, constraint_rotation, body1.getRotation().mul(constraint_to_body1));
        rec.step(1, 0, body1, body2);
        try dumpSwingTwist(&rec, &part);

        part.warmStart(body1, body2, f[21]);
        rec.step(2, 0, body1, body2);
        try dumpSwingTwist(&rec, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2);
            rec.step(3, @intFromBool(result), body1, body2);
            try dumpSwingTwist(&rec, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, constraint_rotation, constraint_to_body1, constraint_to_body2, f[22]);
            rec.step(4, @intFromBool(result), body1, body2);
            try dumpSwingTwist(&rec, &part);
        }

        part.deactivate();
        rec.step(5, 0, body1, body2);
        try dumpSwingTwist(&rec, &part);
    }
};

const IndependentAxisRun = struct {
    fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
        var rec = Rec.init(out, world.allocator);
        var pair = try BodyPair.init(world, input);
        defer pair.deinit();
        const body1 = pair.body1;
        const body2 = pair.body2;
        const f = &input.f;
        const ui = &input.i;
        const n1 = load3(f[3..6]);
        const n2 = load3(f[9..12]);
        const ratio = f[12];

        const Dump = struct {
            fn dump(r: *Rec, id: u32, result: u32, b1: *const Body, b2: *const Body, part: *const IndependentAxisConstraintPart) !void {
                r.step(id, result, b1, b2);
                r.v3(part.r1_x_n1);
                r.v3(part.inv_i1_r1_x_n1);
                r.v3(part.ratio_r2_x_n2);
                r.v3(part.inv_i2_ratio_r2_x_n2);
                r.f(part.effective_mass);
                r.f(part.getTotalLambda());
                r.b(part.isActive());
                try r.state(part);
            }
        };

        var part = std.mem.zeroes(IndependentAxisConstraintPart);
        try restoreFloats(rec.allocator, &part, f[13..14]);
        try Dump.dump(&rec, 0, 0, body1, body2, &part);

        part.calculateConstraintProperties(body1, body2, load3(f[0..3]), n1, load3(f[6..9]), n2, ratio);
        try Dump.dump(&rec, 1, 0, body1, body2, &part);

        part.warmStart(body1, body2, n1, n2, ratio, f[14]);
        try Dump.dump(&rec, 2, 0, body1, body2, &part);

        for (0..ui[2]) |_| {
            const result = part.solveVelocityConstraint(body1, body2, n1, n2, ratio, f[15], f[16]);
            try Dump.dump(&rec, 3, @intFromBool(result), body1, body2, &part);
        }

        for (0..ui[3]) |_| {
            const result = part.solvePositionConstraint(body1, body2, n1, n2, ratio, f[17], f[18]);
            try Dump.dump(&rec, 4, @intFromBool(result), body1, body2, &part);
        }

        part.deactivate();
        try Dump.dump(&rec, 5, 0, body1, body2, &part);
    }
};

/// jolt_cp_gear for GearConstraintPart (`rack` false) and RackAndPinionConstraintPart (`rack` true)
fn GearRun(comptime rack: bool) type {
    return struct {
        const Part = if (rack) RackAndPinionConstraintPart else GearConstraintPart;

        fn dump(r: *Rec, part: *const Part) !void {
            if (rack) {
                r.v3(part.inv_i1_a);
                r.v3(part.ratio_inv_m2_b);
            } else {
                r.v3(part.inv_i1_a);
                r.v3(part.inv_i2_b);
            }
            r.f(part.effective_mass);
            r.f(part.getTotalLambda());
            r.b(part.isActive());
            try r.state(part);
        }

        fn run(world: *World, input: *const CPInput, out: *CPOutput) !void {
            var rec = Rec.init(out, world.allocator);
            var pair = try BodyPair.init(world, input);
            defer pair.deinit();
            const body1 = pair.body1;
            const body2 = pair.body2;
            const f = &input.f;
            const ui = &input.i;
            const a = load3(f[0..3]);
            const b = load3(f[3..6]);
            const ratio = f[6];

            var part = std.mem.zeroes(Part);
            try restoreFloats(rec.allocator, &part, f[7..8]);
            rec.step(0, 0, body1, body2);
            try dump(&rec, &part);

            part.calculateConstraintProperties(body1, a, body2, b, ratio);
            rec.step(1, 0, body1, body2);
            try dump(&rec, &part);

            part.warmStart(body1, body2, f[8]);
            rec.step(2, 0, body1, body2);
            try dump(&rec, &part);

            for (0..ui[2]) |_| {
                const result = part.solveVelocityConstraint(body1, a, body2, b, ratio);
                rec.step(3, @intFromBool(result), body1, body2);
                try dump(&rec, &part);
            }

            for (0..ui[3]) |_| {
                const result = part.solvePositionConstraint(body1, body2, f[9], f[10]);
                rec.step(4, @intFromBool(result), body1, body2);
                try dump(&rec, &part);
            }

            part.deactivate();
            rec.step(5, 0, body1, body2);
            try dump(&rec, &part);
        }
    };
}

/// Wraps a C function `fn(world, input, output)` for compareScenario
fn JoltRun(comptime function: anytype) type {
    return struct {
        fn run(world: *anyopaque, input: *const CPInput, out: *CPOutput) void {
            function(world, input, out);
        }
    };
}

fn JoltGearRun(comptime rack: bool) type {
    return struct {
        fn run(world: *anyopaque, input: *const CPInput, out: *CPOutput) void {
            jolt.jolt_cp_gear(world, input, @intFromBool(rack), out);
        }
    };
}

/// The step results (ids 3 and 4) of the last Zolt record: counts true and false results of velocity / position steps
fn countResults(out: *const CPOutput, id: u32) struct { true_count: u32, false_count: u32 } {
    var t: u32 = 0;
    var f: u32 = 0;
    var k: usize = 0;
    const n = @min(out.num_i, max_i);
    while (k + 1 < n) : (k += 1) {
        if (out.i[k] == (0xC0DE0000 | id)) {
            if (out.i[k + 1] != 0) t += 1 else f += 1;
        }
    }
    return .{ .true_count = t, .false_count = f };
}

test "SpringPart parity" {
    var check: Checker = .{ .name = "SpringPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x51A1B0A7 } };
    var coverage: Coverage = .{};
    for (0..fw.iterations) |_| {
        var in: [7]f32 = undefined;
        in[0] = if (gen.oneIn(2)) 1.0 / 60.0 else gen.float(1.0e-3, 0.1);
        in[1] = switch (gen.index(5)) {
            0 => 1.0,
            1 => gen.float(1.0e-6, 1.0e-3),
            else => gen.float(1.0e-3, 100),
        };
        in[2] = if (gen.oneIn(3)) 0.0 else gen.float(-2, 2);
        in[3] = if (gen.oneIn(3)) 0.0 else gen.float(-1, 1);
        const op: u32 = @intCast(gen.index(5));
        const mode = gen.springMode();
        // Positive frequency / stiffness, damping may be zero (the asserts of the functions)
        in[4] = switch (op) {
            2 => gen.float(0.01, 30),
            4 => if (mode == 0) gen.float(0.01, 30) else gen.float(1.0e-3, 1.0e5),
            else => if (gen.oneIn(4)) 0.0 else gen.float(1.0e-3, 1.0e5),
        };
        in[5] = if (gen.oneIn(3)) 0.0 else gen.float(0, 20);
        if (in[4] == 0.0 and in[5] == 0.0) in[5] = 1.0;
        in[6] = gen.float(-50, 50);

        var jolt_out: [5]f32 = undefined;
        jolt.jolt_cp_spring_part(&in, op, mode, &jolt_out);

        var part: SpringPart = .{};
        var effective_mass: f32 = -12345.0;
        switch (op) {
            0 => part.calculateSpringPropertiesWithBias(in[2]),
            1 => effective_mass = part.calculateSpringPropertiesWithStiffnessAndDamping(in[0], in[1], in[2], in[3], in[4], in[5]),
            2 => effective_mass = part.calculateSpringPropertiesWithFrequencyAndDamping(in[0], in[1], in[2], in[3], in[4], in[5]),
            3 => effective_mass = part.calculateSpringPropertiesWithMassNormalizedStiffnessAndDamping(in[0], in[1], in[2], in[3], in[4], in[5]),
            else => {
                const settings = SpringSettings.init(@enumFromInt(mode), in[4], in[5]);
                part.calculateSpringPropertiesWithSettings(in[0], in[1], in[2], in[3], &settings, &effective_mass);
            },
        }
        const zolt_out = [5]f32{ effective_mass, part.bias, part.softness, part.getBias(in[6]), if (part.isActive()) 1.0 else 0.0 };
        check.check(.{ in, op, mode }, zolt_out, jolt_out);
        coverage.hit("inactive", !part.isActive());
        coverage.hit("damping only", op == 1 and in[4] == 0.0);
        coverage.hit("unnamed spring mode", op == 4 and mode > 2);
    }
    try check.finish();
    try coverage.expectAllReached("SpringPart");
}

test "SpringSettings and MotorSettings parity" {
    var check_spring: Checker = .{ .name = "SpringSettings" };
    var check_motor: Checker = .{ .name = "MotorSettings" };
    var check_state: Checker = .{ .name = "MotorState" };
    var gen: Gen = .{ .rng = .{ .state = 0x5E771265 } };
    const allocator = std.testing.allocator;

    // The named states and other ints (EMotorState has the fixed underlying type int, e.g. a restored state)
    const states = [_]i32{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 13, 255, 256, -1, -2, -3, -4, std.math.maxInt(i32), std.math.minInt(i32) };
    for (states) |state| {
        const s: MotorState = @enumFromInt(state);
        const zolt_bits: u32 = @as(u32, @intFromBool(zolt.isVelocityMotor(s))) | @as(u32, @intFromBool(zolt.isPositionMotor(s))) << 1;
        check_state.check(state, zolt_bits, jolt.jolt_cp_motor_state(state));
    }

    for (0..fw.iterations / 10) |_| {
        // SpringSettings
        const mode = gen.springMode();
        const frequency: f32 = switch (gen.index(4)) {
            0 => 0.0,
            1 => -gen.float(0, 5),
            else => gen.float(0, 1000),
        };
        const damping: f32 = switch (gen.index(4)) {
            0 => 0.0,
            1 => -gen.float(0, 5),
            else => gen.float(0, 10),
        };
        var restore: [9]u8 = undefined;
        for (&restore) |*byte| byte.* = @truncate(gen.rng.next());
        restore[0] = @intCast(gen.springMode());

        var jolt_bytes: [9]u8 = undefined;
        var jolt_info: [6]u32 = undefined;
        var jolt_restored: [3]f32 = undefined;
        jolt.jolt_cp_spring_settings(mode, frequency, damping, &restore, &jolt_bytes, &jolt_info, &jolt_restored);

        const settings = SpringSettings.init(@enumFromInt(mode), frequency, damping);
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        settings.saveBinaryState(recorder.recorder().streamOut());
        var zolt_bytes: [9]u8 = @splat(0);
        const data = recorder.getData();
        @memcpy(zolt_bytes[0..@min(data.len, 9)], data[0..@min(data.len, 9)]);
        var restore_recorder = StateRecorderImpl.init(allocator);
        defer restore_recorder.deinit();
        restore_recorder.recorder().writeBytes(&restore);
        var restored: SpringSettings = .{};
        restored.restoreBinaryState(restore_recorder.recorder().streamIn());
        const zolt_info = [6]u32{ @intFromBool(settings.hasStiffness()), @intFromBool(settings.hasStiffnessOrDamping()), @intFromEnum(restored.mode), @intCast(data.len), @intFromBool(restored.hasStiffness()), @intFromBool(restored.hasStiffnessOrDamping()) };
        const zolt_restored = [3]f32{ restored.frequency, restored.damping, restored.getStiffness() };
        check_spring.check(.{ mode, frequency, damping, restore }, .{ zolt_bytes, zolt_info, zolt_restored }, .{ jolt_bytes, jolt_info, jolt_restored });

        // MotorSettings: the constructors assert IsValid, so they get valid values; the setters may make it invalid
        const constructor: u32 = @intCast(gen.index(5));
        const motor_mode = gen.springMode();
        var f: [7]f32 = undefined;
        f[0] = if (gen.oneIn(4)) 0.0 else gen.float(0, 50);
        f[1] = if (gen.oneIn(4)) 0.0 else gen.float(0, 5);
        f[2] = if (gen.oneIn(4)) 0.0 else gen.float(0, 1000);
        f[3] = if (gen.oneIn(4)) math.flt_max else gen.float(0, 1000);
        f[4] = 0;
        const setter: u32 = @intCast(gen.index(5));
        f[5] = gen.float(-100, 100);
        f[6] = gen.float(-100, 100);
        if ((setter == 1 or setter == 2) and f[5] > f[6]) std.mem.swap(f32, &f[5], &f[6]);
        var motor_restore: [25]u8 = undefined;
        for (&motor_restore) |*byte| byte.* = @truncate(gen.rng.next());
        motor_restore[0] = @intCast(gen.springMode());

        var jolt_motor_bytes: [25]u8 = undefined;
        var jolt_motor_f: [14]f32 = undefined;
        var jolt_motor_info: [5]u32 = undefined;
        jolt.jolt_cp_motor_settings(constructor, motor_mode, &f, setter, &motor_restore, &jolt_motor_bytes, &jolt_motor_f, &jolt_motor_info);

        const sm: SpringMode = @enumFromInt(motor_mode);
        var motor: MotorSettings = switch (constructor) {
            0 => .{},
            1 => .init(f[0], f[1]),
            2 => .initMode(sm, f[0], f[1]),
            3 => .initLimits(f[0], f[1], f[2], f[3]),
            else => .initModeLimits(sm, f[0], f[1], f[2], f[3]),
        };
        switch (setter) {
            0 => {},
            1 => motor.setForceLimits(f[5], f[6]),
            2 => motor.setTorqueLimits(f[5], f[6]),
            3 => motor.setForceLimit(f[5]),
            else => motor.setTorqueLimit(f[5]),
        }
        var motor_recorder = StateRecorderImpl.init(allocator);
        defer motor_recorder.deinit();
        motor.saveBinaryState(motor_recorder.recorder().streamOut());
        const motor_data = motor_recorder.getData();
        var zolt_motor_bytes: [25]u8 = @splat(0);
        @memcpy(zolt_motor_bytes[0..@min(motor_data.len, 25)], motor_data[0..@min(motor_data.len, 25)]);
        var motor_restore_recorder = StateRecorderImpl.init(allocator);
        defer motor_restore_recorder.deinit();
        motor_restore_recorder.recorder().writeBytes(&motor_restore);
        var motor_restored: MotorSettings = .{};
        motor_restored.restoreBinaryState(motor_restore_recorder.recorder().streamIn());
        const fields = struct {
            fn get(m: *const MotorSettings) [7]f32 {
                return .{ m.spring_settings.frequency, m.spring_settings.damping, m.min_force_limit, m.max_force_limit, m.min_torque_limit, m.max_torque_limit, m.spring_settings.getStiffness() };
            }
        }.get;
        var zolt_motor_f: [14]f32 = undefined;
        zolt_motor_f[0..7].* = fields(&motor);
        zolt_motor_f[7..14].* = fields(&motor_restored);
        const zolt_motor_info = [5]u32{ @intFromBool(motor.isValid()), @intFromEnum(motor.spring_settings.mode), @intFromEnum(motor_restored.spring_settings.mode), @intCast(motor_data.len), @intFromBool(motor_restored.isValid()) };
        check_motor.check(.{ constructor, motor_mode, f, setter }, .{ zolt_motor_bytes, zolt_motor_f, zolt_motor_info }, .{ jolt_motor_bytes, jolt_motor_f, jolt_motor_info });
    }
    try fw.finishAll(&.{ &check_spring, &check_motor, &check_state });
}

/// An object with step overrides for CalculateSolverSteps
const StepsObject = struct {
    velocity: u32,
    position: u32,

    pub fn getNumVelocityStepsOverride(self: *const StepsObject) u32 {
        return self.velocity;
    }

    pub fn getNumPositionStepsOverride(self: *const StepsObject) u32 {
        return self.position;
    }
};

test "CalculateSolverSteps parity" {
    var check: Checker = .{ .name = "CalculateSolverSteps" };
    var gen: Gen = .{ .rng = .{ .state = 0x57E95000 } };
    for (0..fw.iterations / 10) |_| {
        var settings: PhysicsSettings = .{};
        settings.num_velocity_steps = @intCast(gen.index(20));
        settings.num_position_steps = @intCast(gen.index(10));
        const count = gen.index(9);
        var overrides: [16]u32 = undefined;
        for (0..count) |k| {
            overrides[2 * k] = if (gen.oneIn(3)) 0 else @intCast(gen.index(256));
            overrides[2 * k + 1] = if (gen.oneIn(3)) 0 else @intCast(gen.index(256));
        }

        var jolt_steps: [4]u32 = undefined;
        jolt.jolt_cp_solver_steps(settings.num_velocity_steps, settings.num_position_steps, &overrides, @intCast(count), &jolt_steps);

        var calculator = CalculateSolverSteps.init(&settings);
        const dummy: DummyCalculateSolverSteps = .{};
        for (0..count) |k| {
            const object: StepsObject = .{ .velocity = overrides[2 * k], .position = overrides[2 * k + 1] };
            calculator.call(StepsObject, &object);
            dummy.call(StepsObject, &object);
        }
        var zolt_steps: [4]u32 = undefined;
        zolt_steps[0] = calculator.getNumVelocitySteps();
        zolt_steps[1] = calculator.getNumPositionSteps();
        calculator.finalize();
        zolt_steps[2] = calculator.getNumVelocitySteps();
        zolt_steps[3] = calculator.getNumPositionSteps();
        check.check(.{ settings.num_velocity_steps, settings.num_position_steps, overrides[0 .. 2 * count] }, zolt_steps, jolt_steps);
    }
    try check.finish();
}

test "ContactConstraintPart and AngularFrictionConstraintPart layout parity" {
    var jolt_contact: [9]u32 = undefined;
    var jolt_angular: [9]u32 = undefined;
    jolt.jolt_cp_contact_sizes(&jolt_contact, &jolt_angular);
    const types = [_]MotionType{ .static, .kinematic, .dynamic };
    inline for (types, 0..) |t1, k1| {
        inline for (types, 0..) |t2, k2| {
            try std.testing.expectEqual(jolt_contact[3 * k1 + k2], @as(u32, @sizeOf(ContactConstraintPart(t1, t2))));
            try std.testing.expectEqual(jolt_angular[3 * k1 + k2], @as(u32, @sizeOf(AngularFrictionConstraintPart(t1, t2))));
        }
    }
}

test "AxisConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "AxisConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xA1150001 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        store3(gen.arm(), input.f[0..3]);
        store3(gen.arm(), input.f[3..6]);
        store3(gen.unit(), input.f[6..9]);
        gen.springScenario(input.f[9..21], &input.i);
        input.i[4] = @intFromBool(pair[0] == .static and pair[1] == .static);
        try compareScenario(&world, &check, "AxisConstraintPart", &input, AxisRun, JoltRun(jolt.jolt_cp_axis));

        // Coverage: deactivated by the calculation, a spring, velocity and position results
        const results = countResults(world.zolt_out, 4);
        coverage.hit("position applied", results.true_count > 0);
        coverage.hit("position skipped", results.false_count > 0);
        coverage.hit("velocity zero range", input.f[9 + 6] == input.f[9 + 7] and countResults(world.zolt_out, 3).false_count > 0);
        coverage.hit("static vs static", input.i[4] != 0);
        coverage.hit("deactivated", pair[0] != .dynamic and pair[1] != .dynamic);
        coverage.hit("spring", input.i[0] >= 1 and input.i[0] <= 4 and input.f[9 + 2] > 0.0);
        coverage.hit("kinematic vs dynamic", pair[0] == .kinematic and pair[1] == .dynamic);
        coverage.hit("unnamed spring mode", input.i[1] > 2 and (input.i[0] == 4 or (input.i[0] == 3 and input.f[9 + 2] > 0.0)));
    }
    try check.finish();
    try coverage.expectAllReached("AxisConstraintPart");
}

test "AngleConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "AngleConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xA9C1E002 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        store3(gen.unit(), input.f[0..3]);
        gen.springScenario(input.f[3..15], &input.i);
        try compareScenario(&world, &check, "AngleConstraintPart", &input, AngleRun, JoltRun(jolt.jolt_cp_angle));

        const results = countResults(world.zolt_out, 4);
        coverage.hit("position applied", results.true_count > 0);
        coverage.hit("position skipped", results.false_count > 0);
        coverage.hit("motor", input.i[0] == 4);
        coverage.hit("unnamed spring mode", input.i[1] > 2 and (input.i[0] == 4 or (input.i[0] == 3 and input.f[3 + 2] > 0.0)));
    }
    try check.finish();
    try coverage.expectAllReached("AngleConstraintPart");
}

test "ContactConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "ContactConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xC0417AC7 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        f[0] = if (gen.oneIn(5)) 0.0 else gen.float(0, 2);
        gen.invInertia(f[1..17]);
        f[17] = if (gen.oneIn(5)) 0.0 else gen.float(0, 2);
        gen.invInertia(f[18..34]);
        store3(gen.arm(), f[34..37]);
        store3(gen.arm(), f[37..40]);
        store3(gen.unit(), f[40..43]);
        f[43] = if (gen.oneIn(3)) 0.0 else gen.float(-1, 1);
        f[44] = gen.lambda();
        f[45] = gen.ratio();
        gen.limits(f[46..48]);
        f[48] = if (gen.oneIn(4)) 0.0 else gen.float(-0.5, 0.5);
        f[49] = gen.baumgarte();
        f[50] = gen.float(-0.5, 0.5);
        input.i[0] = @intFromBool(gen.oneIn(2));
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        input.i[5] = @intFromBool(gen.oneIn(2));
        try compareScenario(&world, &check, "ContactConstraintPart", &input, ContactRun, JoltRun(jolt.jolt_cp_contact));

        coverage.hit("split solve", input.i[5] != 0 and input.i[2] > 0 and !(pair[0] == .static and pair[1] == .static));
        coverage.hit("dynamic vs dynamic", pair[0] == .dynamic and pair[1] == .dynamic);
        coverage.hit("no inverse mass", pair[0] == .dynamic and pair[1] != .dynamic and f[0] == 0.0 and std.mem.allEqual(f32, f[1..17], 0.0));
        coverage.hit("static vs kinematic", pair[0] == .static and pair[1] == .kinematic);
    }
    try check.finish();
    try coverage.expectAllReached("ContactConstraintPart");
}

test "AngularFrictionConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "AngularFrictionConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xF51C7100 } };
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        // Jolt asserts when neither body is dynamic
        const pair = if (Core.enable_asserts) gen.motionTypes(.any_dynamic) else gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        gen.invInertia(f[0..16]);
        gen.invInertia(f[16..32]);
        store3(gen.unit(), f[32..35]);
        f[35] = if (gen.oneIn(3)) 0.0 else gen.float(-1, 1);
        f[36] = gen.lambda();
        f[37] = gen.ratio();
        gen.limits(f[38..40]);
        input.i[0] = @intFromBool(gen.oneIn(2));
        input.i[2] = @intCast(gen.index(5));
        try compareScenario(&world, &check, "AngularFrictionConstraintPart", &input, AngularFrictionRun, JoltRun(jolt.jolt_cp_angular_friction));
    }
    try check.finish();
}

test "DualAxisConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "DualAxisConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xD0A1A71 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        store3(gen.arm(), f[0..3]);
        store3(gen.arm(), f[3..6]);
        const n1 = gen.unit();
        const n2 = switch (gen.index(4)) {
            0 => n1.getNormalizedPerpendicular(),
            1 => n1.cross(gen.unit()).normalizedOr(n1.getNormalizedPerpendicular()),
            2 => n1.add(gen.vec(0.01)).normalized(), // Nearly parallel
            else => gen.unit(),
        };
        store3(n1, f[6..9]);
        store3(n2, f[9..12]);
        f[12] = gen.lambda();
        f[13] = gen.lambda();
        f[14] = gen.ratio();
        store3(if (gen.oneIn(4)) Vec3.zero() else gen.vec(0.5), f[15..18]);
        f[18] = gen.baumgarte();
        store3(gen.vec(0.5), f[19..22]);
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "DualAxisConstraintPart", &input, DualAxisRun, JoltRun(jolt.jolt_cp_dual_axis));

        coverage.hit("position skipped", countResults(world.zolt_out, 4).false_count > 0);
        coverage.hit("position applied", countResults(world.zolt_out, 4).true_count > 0);
    }
    try check.finish();
    try coverage.expectAllReached("DualAxisConstraintPart");
}

test "PointConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "PointConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x9017C0 } };
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        store3(gen.arm(), f[0..3]);
        store3(gen.arm(), f[3..6]);
        store3(if (gen.oneIn(3)) Vec3.zero() else gen.vec(10), f[6..9]);
        f[9] = gen.ratio();
        f[10] = gen.baumgarte();
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "PointConstraintPart", &input, PointRun, JoltRun(jolt.jolt_cp_point));
    }
    try check.finish();
}

test "HingeRotationConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "HingeRotationConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x41A6E000 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        const a1 = gen.unit();
        const a2 = switch (gen.index(6)) {
            0 => a1, // Aligned
            1 => a1.negate(), // Opposite: a random perpendicular is used
            2 => a1.getNormalizedPerpendicular(), // 90 degrees: blend towards a1
            3 => Quat.rotation(a1.getNormalizedPerpendicular(), gen.float(-0.3, 0.3)).mulVec3(a1).normalized(), // Close
            4 => a1.negate().add(gen.vec(1.0e-4)).normalized(), // Nearly opposite
            else => gen.unit(),
        };
        store3(a1, f[0..3]);
        store3(a2, f[3..6]);
        f[6] = gen.lambda();
        f[7] = gen.lambda();
        f[8] = gen.ratio();
        f[9] = gen.baumgarte();
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "HingeRotationConstraintPart", &input, HingeRotationRun, JoltRun(jolt.jolt_cp_hinge_rotation));

        // The branches of CalculateConstraintProperties for axes more than 90 degrees apart
        const dot = a1.dot(a2);
        coverage.hit("blend axes", dot <= 1.0e-3);
        coverage.hit("random perpendicular", dot <= 1.0e-3 and a2.sub(a1.mulScalar(dot)).lengthSq() < 1.0e-6);
        coverage.hit("singular", pair[0] != .dynamic and pair[1] != .dynamic);
    }
    try check.finish();
    try coverage.expectAllReached("HingeRotationConstraintPart");
}

test "RotationEulerConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "RotationEulerConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0xE01E7000 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        // Two orthonormal frames (identical in some inputs: the identity branch)
        const x1 = gen.unit();
        const y1 = x1.getNormalizedPerpendicular();
        const same = gen.oneIn(4);
        const frame = Quat.rotation(gen.unit(), gen.float(-3, 3));
        const x2 = if (same) x1 else frame.mulVec3(x1).normalized();
        const y2 = if (same) y1 else x2.cross(frame.mulVec3(y1)).cross(x2).normalized();
        store3(x1, f[0..3]);
        store3(y1, f[3..6]);
        store3(x2, f[6..9]);
        store3(y2, f[9..12]);
        f[12] = gen.lambda();
        f[13] = gen.lambda();
        f[14] = gen.lambda();
        f[15] = gen.ratio();
        f[16] = gen.baumgarte();
        storeQuat(gen.quat(), f[17..21]);
        input.i[0] = @intCast(gen.index(4));
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "RotationEulerConstraintPart", &input, RotationEulerRun, JoltRun(jolt.jolt_cp_rotation_euler));

        coverage.hit("identical frames", same);
        coverage.hit("zero inertia columns", pair[0] != .dynamic and pair[1] != .dynamic);
        coverage.hit("locked rotation axis", (pair[0] == .dynamic and input.body[0].allowed_dofs & 0b111000 != 0b111000) or (pair[1] == .dynamic and input.body[1].allowed_dofs & 0b111000 != 0b111000));
        coverage.hit("position applied", countResults(world.zolt_out, 4).true_count > 0);
    }
    try check.finish();
    try coverage.expectAllReached("RotationEulerConstraintPart");
}

test "RotationQuatConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "RotationQuatConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x0A7E0000 } };
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        storeQuat(gen.quat(), f[0..4]);
        f[4] = gen.lambda();
        f[5] = gen.lambda();
        f[6] = gen.lambda();
        f[7] = gen.ratio();
        f[8] = gen.baumgarte();
        input.i[0] = @intFromBool(gen.oneIn(2));
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "RotationQuatConstraintPart", &input, RotationQuatRun, JoltRun(jolt.jolt_cp_rotation_quat));
    }
    try check.finish();
}

/// Random swing twist limits: per axis locked (exactly zero or within half a degree), free (beyond 179.5 degrees),
/// or a random range (with a positive max for cone swing limits, mostly symmetric)
fn swingTwistLimits(gen: *Gen, out: []f32, swing_type: SwingType) void {
    for (0..3) |axis| {
        const range: f32 = math.pi;
        var lo: f32 = undefined;
        var hi: f32 = undefined;
        switch (gen.index(5)) {
            0 => {
                lo = 0.0;
                hi = 0.0;
            },
            1 => {
                lo = -gen.float(0, 0.008);
                hi = gen.float(0, 0.008);
            },
            2 => {
                lo = -range;
                hi = range;
            },
            else => {
                if (axis != 0 and swing_type == .cone) {
                    // Cone limits: Jolt only supports limits that are symmetric around 0 (the ellipse needs a positive
                    // max), so asymmetric limits keep a positive max
                    hi = gen.float(0.01, range);
                    lo = if (gen.oneIn(4)) gen.float(-range, hi) else -hi;
                } else {
                    const a = gen.float(-range, range);
                    const b = gen.float(-range, range);
                    lo = @min(a, b);
                    hi = @max(a, b);
                }
            },
        }
        out[2 * axis] = lo;
        out[2 * axis + 1] = hi;
    }
}

test "SwingTwistConstraintPart clamp parity" {
    var check: Checker = .{ .name = "SwingTwistConstraintPart.clampSwingTwist" };
    var gen: Gen = .{ .rng = .{ .state = 0x5C1A4D00 } };
    var coverage: Coverage = .{};
    for (0..fw.iterations) |_| {
        var in: [16]f32 = @splat(0);
        const swing_type: SwingType = @enumFromInt(gen.index(2));
        swingTwistLimits(&gen, in[0..6], swing_type);
        const rotation = gen.quat();
        const swing_twist = rotation.getSwingTwist();
        storeQuat(swing_twist.swing, in[6..10]);
        storeQuat(swing_twist.twist, in[10..14]);
        in[14] = gen.float(-2, 2);
        in[15] = gen.float(-2, 2);

        var jolt_f: [8]f32 = undefined;
        var jolt_i: [2]u32 = undefined;
        jolt.jolt_cp_swing_twist_clamp(&in, @intFromEnum(swing_type), &jolt_f, &jolt_i);

        var part: SwingTwistConstraintPart = .{};
        part.setSwingType(swing_type);
        part.setLimits(in[0], in[1], in[2], in[3], in[4], in[5]);
        var swing = swing_twist.swing;
        var twist = swing_twist.twist;
        const clamped = part.clampSwingTwist(&swing, &twist);
        var zolt_f: [8]f32 = undefined;
        storeQuat(swing, zolt_f[0..4]);
        storeQuat(twist, zolt_f[4..8]);
        const zolt_i = [2]u32{ clamped, @intFromBool(SwingTwistConstraintPart.distanceToMinShorter(in[14], in[15])) };
        check.check(.{ in, swing_type }, .{ zolt_f, zolt_i }, .{ jolt_f, jolt_i });

        coverage.hit("twist min", clamped & SwingTwistConstraintPart.clamped_twist_min != 0 and clamped & SwingTwistConstraintPart.clamped_twist_max == 0);
        coverage.hit("twist max", clamped & SwingTwistConstraintPart.clamped_twist_max != 0 and clamped & SwingTwistConstraintPart.clamped_twist_min == 0);
        coverage.hit("swing y min", clamped & SwingTwistConstraintPart.clamped_swing_y_min != 0 and clamped & SwingTwistConstraintPart.clamped_swing_y_max == 0);
        coverage.hit("swing z max", clamped & SwingTwistConstraintPart.clamped_swing_z_max != 0 and clamped & SwingTwistConstraintPart.clamped_swing_z_min == 0);
        coverage.hit("cone clamped", swing_type == .cone and clamped == 0b111100);
        coverage.hit("pyramid clamped", swing_type == .pyramid and clamped & 0b111100 == 0b111100);
        coverage.hit("not clamped", clamped == 0);
        coverage.hit("negative w", swing_twist.swing.getW() < 0.0 or swing_twist.twist.getW() < 0.0);
    }
    try check.finish();
    try coverage.expectAllReached("SwingTwistConstraintPart.clampSwingTwist");
}

test "SwingTwistConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "SwingTwistConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x5717A000 } };
    var coverage: Coverage = .{};
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        const pair = gen.motionTypes(.all);
        gen.bodies(&input, pair);
        const f = &input.f;
        const swing_type: SwingType = @enumFromInt(gen.index(2));
        swingTwistLimits(&gen, f[0..6], swing_type);
        storeQuat(gen.quat(), f[6..10]);
        storeQuat(gen.quat(), f[10..14]);
        storeQuat(gen.quat(), f[14..18]);
        f[18] = gen.lambda();
        f[19] = gen.lambda();
        f[20] = gen.lambda();
        f[21] = gen.ratio();
        f[22] = gen.baumgarte();
        input.i[0] = @intFromEnum(swing_type);
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "SwingTwistConstraintPart", &input, SwingTwistRun, JoltRun(jolt.jolt_cp_swing_twist));

        coverage.hit("position applied", countResults(world.zolt_out, 4).true_count > 0);
        coverage.hit("position skipped", countResults(world.zolt_out, 4).false_count > 0);
        coverage.hit("velocity applied", countResults(world.zolt_out, 3).true_count > 0);

        // The branches of CalculateConstraintProperties: which limit axes are locked / free and what got clamped
        var probe: SwingTwistConstraintPart = .{};
        probe.setSwingType(swing_type);
        probe.setLimits(f[0], f[1], f[2], f[3], f[4], f[5]);
        const swing_twist = loadQuat(f[6..10]).getSwingTwist();
        var swing = swing_twist.swing;
        var twist = swing_twist.twist;
        const clamped = probe.clampSwingTwist(&swing, &twist);
        const flags = probe.rotation_flags;
        coverage.hit("swing y and z locked", flags & 0b110 == 0b110);
        coverage.hit("swing y locked, z clamped min", flags & 0b110 == 0b010 and clamped & SwingTwistConstraintPart.clamped_swing_z_min != 0);
        coverage.hit("swing z locked, y clamped min", flags & 0b110 == 0b100 and clamped & SwingTwistConstraintPart.clamped_swing_y_min != 0);
        coverage.hit("swing limited and clamped", flags & 0b110110 == 0 and clamped & 0b111100 != 0);
        coverage.hit("swing free", flags & 0b110000 == 0b110000);
        coverage.hit("twist locked", flags & 0b1 != 0);
        coverage.hit("twist clamped min", flags & 0b1001 == 0 and clamped & SwingTwistConstraintPart.clamped_twist_min != 0);
        coverage.hit("twist free", flags & 0b1000 != 0);
    }
    try check.finish();
    try coverage.expectAllReached("SwingTwistConstraintPart");
}

test "IndependentAxisConstraintPart parity" {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    var check: Checker = .{ .name = "IndependentAxisConstraintPart" };
    var gen: Gen = .{ .rng = .{ .state = 0x14DE9000 } };
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        // A kinematic body trips the dynamic body assert of GetInverseMass
        var pair = gen.motionTypes(.all);
        if (Core.enable_asserts) {
            while (pair[0] == .kinematic or pair[1] == .kinematic)
                pair = gen.motionTypes(.all);
        }
        gen.bodies(&input, pair);
        const f = &input.f;
        store3(gen.arm(), f[0..3]);
        store3(gen.unit(), f[3..6]);
        store3(gen.arm(), f[6..9]);
        store3(gen.unit(), f[9..12]);
        f[12] = switch (gen.index(4)) {
            0 => 1.0,
            1 => -1.0,
            else => gen.float(-3, 3),
        };
        f[13] = gen.lambda();
        f[14] = gen.ratio();
        gen.limits(f[15..17]);
        f[17] = if (gen.oneIn(4)) 0.0 else gen.float(-0.5, 0.5);
        f[18] = gen.baumgarte();
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, "IndependentAxisConstraintPart", &input, IndependentAxisRun, JoltRun(jolt.jolt_cp_independent_axis));
    }
    try check.finish();
}

fn gearParity(comptime rack: bool) !void {
    var world: World = undefined;
    try world.init(std.testing.allocator);
    defer world.deinit();
    const name = if (rack) "RackAndPinionConstraintPart" else "GearConstraintPart";
    var check: Checker = .{ .name = name };
    var gen: Gen = .{ .rng = .{ .state = if (rack) 0x4AC0000 else 0x6EA40000 } };
    for (0..scenario_iterations) |_| {
        var input: CPInput = .{ .body = undefined };
        // Both bodies must be dynamic (Jolt asserts in GetInverseMass / MultiplyWorldSpaceInverseInertiaByVector)
        const pair: [2]MotionType = if (Core.enable_asserts) .{ .dynamic, .dynamic } else gen.motionTypes(.dynamic_or_kinematic);
        gen.bodies(&input, pair);
        const f = &input.f;
        store3(gen.unit(), f[0..3]);
        store3(gen.unit(), f[3..6]);
        f[6] = switch (gen.index(4)) {
            0 => 1.0,
            1 => -2.0,
            else => gen.float(-3, 3),
        };
        f[7] = gen.lambda();
        f[8] = gen.ratio();
        f[9] = if (gen.oneIn(4)) 0.0 else gen.float(-0.5, 0.5);
        f[10] = gen.baumgarte();
        input.i[2] = @intCast(gen.index(5));
        input.i[3] = @intCast(gen.index(4));
        try compareScenario(&world, &check, name, &input, GearRun(rack), JoltGearRun(rack));
    }
    try check.finish();
}

test "GearConstraintPart parity" {
    try gearParity(false);
}

test "RackAndPinionConstraintPart parity" {
    try gearParity(true);
}
