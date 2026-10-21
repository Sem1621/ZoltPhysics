//! Parity tests of the constraint and body draws (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, "Draw call
//! log", group D3). C ABI wrappers: `jolt_dr_run_physics_ops` and `jolt_dr_body_world_*` in
//! ZoltParity/Renderer/DebugRendererReference.cpp, the constraint world `jolt_cs_draw_world_*` in
//! ZoltParity/Physics/ConstraintsReference.cpp (`DrawWorld` in ConstraintsParity.zig). The ops (96 to 104) are in
//! ParityRenderer.zig and ParityRenderer.h.
//!
//! The layers (they exist only with -Ddebug_renderer=true, without the option they skip):
//! - constraints: a DrawWorld with 3 random scenarios of every constraint kind (the generator of ConstraintsParity:
//!   random bodies, settings, draw sizes, operations and solver steps; far from the origin too in double precision) and
//!   the forced scenarios of ConstraintsParity (a SixDOF constraint with the pyramid swing type and rotation motors, a
//!   path constraint with a velocity motor). In every frame every scenario runs its next frame (its operations and solver
//!   steps), then each constraint is drawn with DrawConstraint, DrawConstraintLimits and DrawConstraintReferenceFrame,
//!   the ConstraintManager draws all of them the three ways, BodyManager::Draw draws the bodies of the scenarios and some
//!   draw sizes change (0.001 to 4). The frames go past the last frame of every scenario.
//! - bodies: a BodyManager with two bodies per shape of the pairwise catalogue (PairwiseParity.makeCatalogue): every
//!   motion type (static only for the shapes that must be static), sensors, restricted DOFs, the three ways to get the
//!   mass and inertia (provided for shapes without mass), some active and some asleep. Every frame draws with all flags
//!   in every EShapeColor, with each flag alone, and with random flag sets, times before sleep (zero too) and
//!   ESoftBodyConstraintColors, with and without a BodyDrawFilter. Between the frames bodies move, update their sleep
//!   state (the ECanSleep results are compared too), activate or deactivate and get island indices (the island builder
//!   of PhysicsSystem sets them; EShapeColor::IslandColor reads them).
//! - DebugRendererSimple: a few frames of both on a DebugRendererSimple that logs its lines.
//! - DebugRendererRecorder: a few frames of both on Jolt's and Zolt's recorder, with EndFrame after each frame. The .jor
//!   bytes after the construction must be equal. In double precision too: the .jor check of the performance test scenes
//!   (DebugRendererCheck.zig) draws only bodies.
//! The call logs must match call for call.
//!
//! The coverage of the conditional draws: the constraint and body layers fail when a branch of a draw function was never
//! reached (`DrawCoverage`): the motor, limit, swing type and length branches of the constraints' DrawConstraint and
//! DrawConstraintLimits, and the island color of a body with and without an island index. The branches are evaluated on
//! the Zolt side before each draw; the logs match call for call, so Jolt takes the same branches.

const std = @import("std");
const zolt = @import("zolt");
const pr = @import("ParityRenderer.zig");
const pw = @import("../Physics/PairwiseParity.zig");
const cs = @import("../Physics/ConstraintsParity.zig");

const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyManager = zolt.BodyManager;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const Mat44 = zolt.Mat44;
const ObjectLayer = zolt.ObjectLayer;
const Quat = zolt.Quat;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const Vec3 = zolt.Vec3;
const virtual = zolt.virtual;
const BodyDrawBits = pr.BodyDrawBits;
const JoltResult = pr.JoltResult;

const allocator = std.testing.allocator;
const io = std.testing.io;

const jolt = struct {
    extern fn jolt_dr_available() c_int;
    extern fn jolt_dr_body_desc_size() u32;
    extern fn jolt_dr_body_world_create(catalogue: *anyopaque, bodies: [*]const DRBodyDesc, count: u32) ?*anyopaque;
    extern fn jolt_dr_body_world_destroy(world: *anyopaque) void;
    extern fn jolt_dr_run_physics_ops(constraint_world: ?*anyopaque, num_constraints: u32, body_world: ?*anyopaque, ops: [*]const u8, size: u64, kind: c_int) ?*anyopaque;
};

/// Skip without the option (the C++ reference has no DebugRenderer then)
fn requireDebugRenderer() !void {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;
    try std.testing.expectEqual(@as(c_int, 1), jolt.jolt_dr_available()); // Both sides follow -Ddebug_renderer
}

/// The renderer that runs the ops (the `kind` of jolt_dr_run_physics_ops)
const Kind = enum(c_int) { parity_renderer = 0, simple = 1, recorder = 2 };

// ---------------------------------------------------------------------------------------------------------------------
// The body world

/// A body of the body world, its shape is a catalogue shape. Must match DRBodyDesc in DebugRendererReference.cpp.
const DRBodyDesc = extern struct {
    position: [3]Real,
    rotation: [4]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    /// The mass of MassAndInertiaProvided / CalculateInertia (inertia: identity)
    mass: f32,
    /// Index into the catalogue
    shape: u32,
    motion_type: u32,
    override_mass_properties: u32,
    allowed_dofs: u32,
    is_sensor: u32,
    allow_sleeping: u32,
    /// ActivateBodies after adding the body (not static)
    activate: u32,
};

/// One broad phase layer (DRLayers)
const Layers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(Layers),

    pub fn getNumBroadPhaseLayers(self: *const Layers) u32 {
        _ = self;
        return 1;
    }

    pub fn getBroadPhaseLayer(self: *const Layers, layer: ObjectLayer) BroadPhaseLayer {
        _ = .{ self, layer };
        return .init(0);
    }
};

/// The bodies of the catalogue shapes on both sides (jolt_dr_body_world_create)
const BodyWorld = struct {
    buffer: []u32,
    built: *pw.Built,
    descs: []DRBodyDesc,
    layers: Layers = .{},
    body_manager: BodyManager = .empty,
    bodies: []*Body,
    jolt_world: *anyopaque,

    fn create() !*BodyWorld {
        const self = try allocator.create(BodyWorld);
        errdefer allocator.destroy(self);
        self.buffer = try allocator.alloc(u32, pw.stream_capacity);
        errdefer allocator.free(self.buffer);
        self.built = try pw.Built.init(allocator, try pw.makeCatalogue(allocator), self.buffer);
        errdefer self.built.deinit();
        self.descs = try makeBodies(self.built);
        errdefer allocator.free(self.descs);
        self.layers = .{};
        self.body_manager = .empty;
        try self.body_manager.init(allocator, io, @intCast(self.descs.len), 0, &self.layers.base);
        errdefer self.body_manager.deinit();
        self.bodies = try allocator.alloc(*Body, self.descs.len);
        errdefer allocator.free(self.bodies);
        for (self.descs, self.bodies) |*d, *body| body.* = try self.createBody(d);
        self.jolt_world = jolt.jolt_dr_body_world_create(self.built.handle, self.descs.ptr, @intCast(self.descs.len)).?;
        return self;
    }

    fn destroy(self: *BodyWorld) void {
        jolt.jolt_dr_body_world_destroy(self.jolt_world);
        var i = self.bodies.len;
        while (i > 0) {
            i -= 1;
            const id = self.bodies[i].getID();
            self.body_manager.deactivateBodies(&.{id});
            self.bodies[i].setInBroadPhaseInternal(false);
            self.body_manager.destroyBodies(&.{id});
        }
        allocator.free(self.bodies);
        self.body_manager.deinit();
        allocator.free(self.descs);
        self.built.deinit();
        allocator.free(self.buffer);
        allocator.destroy(self);
    }

    /// jolt_dr_body_world_create for one body
    fn createBody(self: *BodyWorld, d: *const DRBodyDesc) !*Body {
        const position = RVec3.init(d.position[0], d.position[1], d.position[2]);
        const rotation = Quat.init(d.rotation[0], d.rotation[1], d.rotation[2], d.rotation[3]);
        var settings = BodyCreationSettings.initWithShape(self.built.shape(d.shape), position, rotation, @enumFromInt(d.motion_type), 0);
        defer settings.deinit();
        settings.linear_velocity = Vec3.init(d.linear_velocity[0], d.linear_velocity[1], d.linear_velocity[2]);
        settings.angular_velocity = Vec3.init(d.angular_velocity[0], d.angular_velocity[1], d.angular_velocity[2]);
        settings.override_mass_properties = @enumFromInt(d.override_mass_properties);
        settings.mass_properties_override.mass = d.mass;
        settings.mass_properties_override.inertia = Mat44.identity();
        settings.allowed_dofs = .fromInt(@intCast(d.allowed_dofs));
        settings.is_sensor = d.is_sensor != 0;
        settings.allow_sleeping = d.allow_sleeping != 0;
        const body = try self.body_manager.allocateBody(&settings);
        if (!self.body_manager.addBody(body)) @panic("out of bodies");
        body.setInBroadPhaseInternal(true);
        if (d.activate != 0 and !body.isStatic())
            self.body_manager.activateBodies(&.{body.getID()});
        return body;
    }
};

/// Two bodies per catalogue shape: motion types i % 3 and (i + 1) % 3 (static for the shapes that must be static),
/// sensors, restricted DOFs, the mass and inertia calculated, with a given mass, or provided (always for shapes without
/// mass or inertia), active or asleep
fn makeBodies(built: *const pw.Built) ![]DRBodyDesc {
    var gen: pw.Gen = .{ .rng = .{ .state = 0xd3b0d1e5 } };
    const num_shapes: u32 = @intCast(built.shapes.items.len);
    const descs = try allocator.alloc(DRBodyDesc, 2 * num_shapes);
    for (descs, 0..) |*d, i| {
        const shape_index: u32 = @intCast(i / 2);
        const shape = built.shape(shape_index);
        const motion_type: u32 = if (shape.mustBeStatic()) 0 else @intCast((shape_index + i % 2) % 3);
        const mp = shape.getMassProperties();
        const diagonal = mp.inertia.getDiagonal3();
        const has_mass = mp.mass > 0 and diagonal.getX() > 0 and diagonal.getY() > 0 and diagonal.getZ() > 0;
        const far = zolt.Core.double_precision and gen.chance(15);
        const q = gen.rotation();
        d.* = .{
            .position = if (far)
                .{ @as(Real, gen.float(-1.0e5, 1.0e5)) + @as(Real, gen.float(0, 1)) / 7, gen.float(-1.0e3, 1.0e3), gen.float(-1.0e5, 1.0e5) }
            else
                .{ gen.float(-20, 20), gen.float(-20, 20), gen.float(-20, 20) },
            .rotation = .{ q.getX(), q.getY(), q.getZ(), q.getW() },
            .linear_velocity = .{ 0, 0, 0 },
            .angular_velocity = .{ 0, 0, 0 },
            .mass = gen.float(0.5, 20),
            .shape = shape_index,
            .motion_type = motion_type,
            .override_mass_properties = if (!has_mass) 2 else gen.index(3),
            .allowed_dofs = switch (gen.index(6)) {
                0 => @as(u8, @bitCast(zolt.AllowedDOFs.plane_2d)),
                1 => 0b000111, // Translations only: no inverse inertia, no mass and inertia box
                else => @as(u8, @bitCast(zolt.AllowedDOFs.all)),
            },
            .is_sensor = @intFromBool(gen.chance(10)),
            .allow_sleeping = @intFromBool(!gen.chance(10)),
            .activate = @intFromBool(!gen.chance(25)),
        };
        if (motion_type != 0) {
            const v = gen.vec(-5, 5);
            const w = gen.vec(-3, 3);
            d.linear_velocity = .{ v.getX(), v.getY(), v.getZ() };
            d.angular_velocity = .{ w.getX(), w.getY(), w.getZ() };
        }
    }
    return descs;
}

// ---------------------------------------------------------------------------------------------------------------------
// The op streams

/// The flags of BodyManager::DrawSettings without the soft body flags
const rigid_body_bits: u32 = (1 << BodyDrawBits.first_soft_body) - 1;

/// The support function and supporting face flags
const support_bits: u32 = BodyDrawBits.draw_get_support_function | BodyDrawBits.draw_support_direction | BodyDrawBits.draw_get_supporting_face;

/// Writes the ops of the layers
const Generator = struct {
    gen: pw.Gen,
    w: *pr.OpWriter,

    fn shapeColor(self: *Generator) u8 {
        return @intCast(self.gen.index(BodyDrawBits.num_shape_colors));
    }

    fn softBodyConstraintColor(self: *Generator) u8 {
        return @intCast(self.gen.index(BodyDrawBits.num_soft_body_constraint_colors));
    }

    /// A time before sleep, now and then zero (the sleep color is then 0 / 0 or x / 0)
    fn timeBeforeSleep(self: *Generator) f32 {
        return if (self.gen.chance(10)) 0 else self.gen.float(0.1, 2);
    }

    /// BodyManager::Draw with all flags in every EShapeColor, with each flag alone and with random sets. The support
    /// function and supporting face draws are big (a sphere of support points per body): only the first frame has them,
    /// alone, with the support direction and together with all other flags.
    fn bodyDraws(self: *Generator, frame: usize, random_draws: u32) !void {
        if (frame == 0) {
            try self.w.drawBodies(BodyDrawBits.draw_get_support_function, self.shapeColor(), 0, 1, false);
            try self.w.drawBodies(BodyDrawBits.draw_get_support_function | BodyDrawBits.draw_support_direction, self.shapeColor(), 0, 1, true);
            try self.w.drawBodies(BodyDrawBits.draw_get_supporting_face, self.shapeColor(), 0, 1, false);
            try self.w.drawBodies(rigid_body_bits, self.shapeColor(), 0, 1, true);
        }
        for (0..BodyDrawBits.num_shape_colors) |color|
            try self.w.drawBodies(rigid_body_bits & ~support_bits, @intCast(color), self.softBodyConstraintColor(), self.timeBeforeSleep(), color % 2 == 1);
        for (0..BodyDrawBits.first_soft_body) |bit| {
            const bits = @as(u32, 1) << @intCast(bit);
            if (bits & support_bits == 0)
                try self.w.drawBodies(bits, self.shapeColor(), self.softBodyConstraintColor(), self.timeBeforeSleep(), self.gen.chance(50));
        }
        for (0..random_draws) |_|
            try self.w.drawBodies(self.gen.rng.next() & BodyDrawBits.all & ~support_bits, self.shapeColor(), self.softBodyConstraintColor(), self.timeBeforeSleep(), self.gen.chance(50));
    }

    /// A draw size: tiny, small, one, big or random
    fn drawSize(self: *Generator) f32 {
        return switch (self.gen.index(5)) {
            0 => 0.001,
            1 => 0.5,
            2 => 1.0,
            3 => 2.5,
            else => self.gen.float(0.05, 4),
        };
    }
};

/// The ops of the constraint layer: in every frame every scenario steps, then every constraint is drawn the three
/// ways, the ConstraintManager draws, BodyManager::Draw draws the bodies and some draw sizes change. The steps come first
/// because some draws read members that SetupVelocityConstraint computes and that Jolt leaves uninitialized before
/// (PathConstraint's mR1, mR2 and path frame; Zolt starts them at zero, see PathConstraint.zig). The parity never draws
/// a constraint before its first step.
fn makeConstraintOps(w: *pr.OpWriter, num_constraints: u32, seed: u32, frames: u32, end_frames: bool) !void {
    var g: Generator = .{ .gen = .{ .rng = .{ .state = seed } }, .w = w };
    for (0..frames) |_| {
        for (0..num_constraints) |i| try w.stepConstraint(@intCast(i));
        for (0..num_constraints) |i| {
            const c: u32 = @intCast(i);
            if (g.gen.chance(30)) try w.setDrawConstraintSize(c, g.drawSize());
            try w.drawConstraint(c, .constraint);
            try w.drawConstraint(c, .limits);
            try w.drawConstraint(c, .reference_frame);
        }
        try w.drawConstraintManager(.constraint);
        try w.drawConstraintManager(.limits);
        try w.drawConstraintManager(.reference_frame);
        try w.drawBodies(rigid_body_bits & ~support_bits, g.shapeColor(), 0, g.timeBeforeSleep(), g.gen.chance(50));
        // Random draws
        for (0..num_constraints) |_| {
            const c = g.gen.index(num_constraints);
            try w.drawConstraint(c, @enumFromInt(g.gen.index(3)));
        }
        if (end_frames) try w.endFrame();
    }
}

/// The ops of the body layer: the body draws of every frame, and between the frames the bodies move, update their sleep
/// state, activate or deactivate and get island indices
fn makeBodyOps(w: *pr.OpWriter, world: *const BodyWorld, seed: u32, frames: u32, random_draws: u32, end_frames: bool) !void {
    var g: Generator = .{ .gen = .{ .rng = .{ .state = seed } }, .w = w };
    // The positions of the bodies as the ops move them
    const positions = try allocator.alloc(RVec3, world.descs.len);
    defer allocator.free(positions);
    for (world.descs, positions) |*d, *p| p.* = RVec3.init(d.position[0], d.position[1], d.position[2]);

    for (0..frames) |frame| {
        try g.bodyDraws(frame, random_draws);
        if (end_frames) try w.endFrame();
        for (world.descs, positions, 0..) |*d, *p, i| {
            if (d.motion_type == 0) continue; // Static bodies have no sleep state
            const b: u32 = @intCast(i);
            // A small move (the body can sleep), a big one (it cannot) or none
            switch (g.gen.index(3)) {
                0 => {
                    p.* = p.addVec3(g.gen.vec(-0.01, 0.01));
                    try w.moveBody(b, p.*, g.gen.rotation());
                },
                1 => {
                    p.* = p.addVec3(g.gen.vec(-2, 2));
                    try w.moveBody(b, p.*, g.gen.rotation());
                },
                else => {},
            }
            try w.updateSleepState(b, g.gen.float(0.01, 0.4), g.gen.float(0.01, 1), g.gen.float(0.2, 1));
            if (g.gen.chance(10)) try w.activateBody(b, g.gen.chance(50));
            // An island index (also above the 32 distinct colors) or none (DeactivateBodies resets it too)
            if (g.gen.chance(40)) try w.setIslandIndex(b, if (g.gen.chance(20)) Body.inactive_index else @intCast(g.gen.index(70)));
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The coverage of the conditional draws

/// The branches of the draw functions that a layer must reach (each at least once). They are evaluated on the Zolt side
/// before each draw (`PhysicsContext.on_draw_constraint` / `on_draw_bodies`) with the conditions of the draw function.
const DrawCoverage = struct {
    names: [40][]const u8 = undefined,
    hits: [40]u32 = @splat(0),
    count: usize = 0,

    fn hit(self: *DrawCoverage, comptime name: []const u8, condition: bool) void {
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

    fn expectAllReached(self: *const DrawCoverage, layer: []const u8) !void {
        var ok = true;
        for (self.names[0..self.count], self.hits[0..self.count]) |n, h| {
            if (h == 0) {
                std.debug.print("{s}: the draw branch '{s}' was never reached\n", .{ layer, n });
                ok = false;
            }
        }
        if (!ok) return error.CoverageNotReached;
    }

    /// The branches of DrawConstraint / DrawConstraintLimits of `c` (DrawConstraintReferenceFrame has none)
    fn constraintDraw(self: *DrawCoverage, c: *zolt.Constraint, what: pr.ConstraintDraw) void {
        const two_body = virtual.downcast(zolt.TwoBodyConstraint, c);
        switch (what) {
            .constraint => switch (c.getSubType()) {
                .distance => {
                    // DistanceConstraint::DrawConstraint: the length against the min and max distance
                    const d = virtual.downcast(zolt.DistanceConstraint, two_body);
                    const len = d.world_space_position2.sub(d.world_space_position1).toVec3().length();
                    self.hit("Distance: shorter than the min distance", len < d.min_distance);
                    self.hit("Distance: longer than the max distance", !(len < d.min_distance) and len > d.max_distance);
                    self.hit("Distance: between the min and max distance", !(len < d.min_distance) and !(len > d.max_distance));
                },
                .pulley => {
                    // PulleyConstraint::DrawConstraint: the color of the length
                    const p = virtual.downcast(zolt.PulleyConstraint, two_body);
                    const length = p.getCurrentLength();
                    self.hit("Pulley: shorter than the min length (yellow)", length < p.min_length);
                    self.hit("Pulley: longer than the max length (red)", !(length < p.min_length) and length > p.max_length);
                    self.hit("Pulley: between the min and max length (green)", !(length < p.min_length) and !(length > p.max_length));
                },
                .slider => {
                    const sl = virtual.downcast(zolt.SliderConstraint, two_body);
                    self.hit("Slider: position motor marker", zolt.isPositionMotor(sl.motor_state));
                    self.hit("Slider: velocity motor arrow", zolt.isVelocityMotor(sl.motor_state));
                    self.hit("Slider: motor off", sl.motor_state == .off);
                },
                .swing_twist => {
                    const st = virtual.downcast(zolt.SwingTwistConstraint, two_body);
                    self.hit("SwingTwist: velocity motor arrow", zolt.isVelocityMotor(st.swing_motor_state) or zolt.isVelocityMotor(st.twist_motor_state));
                    self.hit("SwingTwist: position motor swing and twist", zolt.isPositionMotor(st.swing_motor_state) or zolt.isPositionMotor(st.twist_motor_state));
                    self.hit("SwingTwist: motors off", st.swing_motor_state == .off and st.twist_motor_state == .off);
                },
                .six_dof => {
                    // SixDOFConstraint::DrawConstraint (IsRotationConstrained and IsRotationFullyConstrained are private)
                    const sd = virtual.downcast(zolt.SixDOFConstraint, two_body);
                    const rotation_constrained = (sd.free_axis & 0b111000) != 0b111000;
                    const rotation_fully_constrained = (sd.fixed_axis & 0b111000) == 0b111000;
                    const swing_twist = (rotation_constrained or sd.rotation_position_motor_active != 0) and !rotation_fully_constrained;
                    self.hit("SixDOF: current swing and twist", swing_twist);
                    self.hit("SixDOF: no current swing and twist", !swing_twist);
                    const rotation_x = @intFromEnum(zolt.SixDOFConstraint.Axis.rotation_x);
                    self.hit("SixDOF: target twist (rotation X position motor)", zolt.isPositionMotor(sd.motor_state[rotation_x]));
                    self.hit("SixDOF: target swing (rotation Y or Z position motor)", zolt.isPositionMotor(sd.motor_state[rotation_x + 1]) or zolt.isPositionMotor(sd.motor_state[rotation_x + 2]));
                    var velocity_motor = false;
                    var target_angular_velocity = Vec3.zero();
                    for (0..3) |i| {
                        if (zolt.isVelocityMotor(sd.motor_state[rotation_x + i])) {
                            velocity_motor = true;
                            target_angular_velocity.setComponent(@intCast(i), sd.target_angular_velocity.getComponent(@intCast(i)));
                        }
                    }
                    self.hit("SixDOF: rotation velocity motor", velocity_motor);
                    self.hit("SixDOF: target angular velocity arrow", !target_angular_velocity.eql(Vec3.zero()));
                },
                .path => {
                    const pc = virtual.downcast(zolt.PathConstraint, two_body);
                    const has_path = pc.path.get() != null;
                    self.hit("Path: no path", !has_path);
                    self.hit("Path: position motor target marker", has_path and zolt.isPositionMotor(pc.position_motor_state));
                    self.hit("Path: velocity motor arrow", has_path and zolt.isVelocityMotor(pc.position_motor_state));
                },
                else => {},
            },
            .limits => switch (c.getSubType()) {
                .hinge => {
                    const h = virtual.downcast(zolt.HingeConstraint, two_body);
                    self.hit("Hinge: limits pie", h.has_limits and h.limits_max > h.limits_min);
                    self.hit("Hinge: no limits pie", !(h.has_limits and h.limits_max > h.limits_min));
                },
                .slider => {
                    const sl = virtual.downcast(zolt.SliderConstraint, two_body);
                    self.hit("Slider: limits", sl.has_limits);
                    self.hit("Slider: no limits", !sl.has_limits);
                },
                .swing_twist => {
                    const st = virtual.downcast(zolt.SwingTwistConstraint, two_body);
                    self.hit("SwingTwist: swing pyramid limits", st.swing_twist_constraint_part.getSwingType() == .pyramid);
                    self.hit("SwingTwist: swing cone limits", st.swing_twist_constraint_part.getSwingType() == .cone);
                },
                .six_dof => {
                    const sd = virtual.downcast(zolt.SixDOFConstraint, two_body);
                    self.hit("SixDOF: swing pyramid limits", sd.swing_twist_constraint_part.getSwingType() == .pyramid);
                    self.hit("SixDOF: swing cone limits", sd.swing_twist_constraint_part.getSwingType() == .cone);
                },
                else => {},
            },
            .reference_frame => {},
        }
    }

    fn onDrawConstraint(context: *anyopaque, c: *zolt.Constraint, what: pr.ConstraintDraw) void {
        const self: *DrawCoverage = @ptrCast(@alignCast(context));
        self.constraintDraw(c, what);
    }

    /// The island color branch of BodyManager::Draw (`bodies`: the bodies of the layer, the ParityBodyDrawFilter skips
    /// the bodies whose index % 3 is 1). The color is used by the shape, the bounding box and the support function.
    fn bodyDraw(self: *DrawCoverage, bodies: []const *Body, settings: *const BodyManager.DrawSettings, filter: bool) void {
        if (comptime !zolt.Core.debug_renderer) return;
        const uses_color = settings.draw_shape or settings.draw_bounding_box or settings.draw_get_support_function;
        if (settings.draw_shape_color != .island_color or !uses_color) return;
        var with_index = false;
        var without_index = false;
        for (bodies) |body| {
            if (!body.isInBroadPhase() or body.isSensor() or body.isStatic()) continue;
            if (filter and body.getID().getIndex() % 3 == 1) continue;
            if (body.getMotionProperties().getIslandIndexInternal() != Body.inactive_index) with_index = true else without_index = true;
        }
        self.hit("BodyManager: island color of a body with an island index", with_index);
        self.hit("BodyManager: island color of a body without an island index (light grey)", without_index);
    }

    /// `bodyDraw` with the bodies of a BodyWorld
    const BodyContext = struct {
        coverage: *DrawCoverage,
        bodies: []const *Body,

        fn onDrawBodies(context: *anyopaque, settings: *const BodyManager.DrawSettings, filter: bool) void {
            const self: *BodyContext = @ptrCast(@alignCast(context));
            self.coverage.bodyDraw(self.bodies, settings, filter);
        }
    };
};

// ---------------------------------------------------------------------------------------------------------------------
// The layers

/// Compare the results of the ops (one u32 ECanSleep per update_sleep_state)
fn compareResults(name: []const u8, zolt_results: []const u8, jolt_results: []const u8) !void {
    if (std.mem.eql(u8, zolt_results, jolt_results))
        return;
    if (zolt_results.len != jolt_results.len) {
        std.debug.print("{s}: {d} result bytes (zolt), {d} (jolt)\n", .{ name, zolt_results.len, jolt_results.len });
        return error.ParityMismatch;
    }
    const index = std.mem.indexOfDiff(u8, zolt_results, jolt_results).? / 4;
    std.debug.print("{s}: UpdateSleepStateInternal {d} returned {d} (zolt) and {d} (jolt)\n", .{ name, index, zolt_results[4 * index], jolt_results[4 * index] });
    return error.ParityMismatch;
}

/// Compare the .jor bytes of the recorders (after the construction)
fn compareBytes(name: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes))
        return;
    const index = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
    std.debug.print("{s}: the .jor bytes differ at byte {d} of {d} (zolt) / {d} (jolt)\n  zolt: {x}\n  jolt: {x}\n", .{ name, index, zolt_bytes.len, jolt_bytes.len, zolt_bytes[index..@min(index + 64, zolt_bytes.len)], jolt_bytes[index..@min(index + 64, jolt_bytes.len)] });
    return error.ParityMismatch;
}

/// Run the ops on Zolt's and Jolt's renderer of `kind` with the objects of `context` / the C++ worlds, compare the logs
/// (the .jor bytes for the recorder) and the results. `context.results` holds the Zolt results afterwards.
fn runLayer(name: []const u8, ops: []const u8, kind: Kind, context: *pr.PhysicsContext, jolt_constraint_world: ?*anyopaque, jolt_body_world: ?*anyopaque) !void {
    // Jolt first: both sides start from the same state, the ops change it (steps, moves, sleep states)
    const result: JoltResult = .{ .handle = jolt.jolt_dr_run_physics_ops(jolt_constraint_world, @intCast(context.constraints.len), jolt_body_world, ops.ptr, ops.len, @intFromEnum(kind)).? };
    defer result.deinit();

    switch (kind) {
        .parity_renderer => {
            var renderer: pr.ParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithContext(allocator, &renderer.base, ops, null, null, null, context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .simple => {
            var renderer: pr.SimpleParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithContext(allocator, renderer.base.debugRenderer(), ops, &renderer.base, null, null, context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .recorder => {
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
            var recorder: zolt.DebugRendererRecorder = undefined;
            try recorder.init(allocator, io, wrapper.streamOut());
            defer recorder.deinit();
            const construction_len = out.written().len;
            try pr.runOpsWithContext(allocator, recorder.debugRenderer(), ops, null, &recorder, null, context);
            try compareBytes(name, out.written()[construction_len..], result.part(0));
        },
    }
    try compareResults(name, context.results.items, result.part(1));
}

/// Options of the constraint and body layers
const LayerOptions = struct {
    /// Number of frames
    frames: u32,
    /// The constraint layer: the number of seeds (each makes a scenario of every kind)
    num_seeds: u32 = 1,
    /// The constraint layer: the forced scenarios of ConstraintsParity (`DrawWorld.Options.forced_draws`)
    forced_draws: bool = true,
    /// The body layer: random BodyManager::Draw calls per frame
    random_draws: u32 = 4,
    /// EndFrame after each frame (the recorder)
    end_frames: bool = false,
    /// Fail when a branch of a draw function was never reached (`DrawCoverage`)
    coverage: bool = false,
};

/// The constraint layer: a DrawWorld of `options.num_seeds` seeds from `seed`, `options.frames` frames of
/// makeConstraintOps
fn runConstraintLayer(name: []const u8, seed: u32, kind: Kind, options: LayerOptions) !void {
    var seeds: [8]u32 = undefined;
    for (seeds[0..options.num_seeds], 0..) |*s, i| s.* = seed +% @as(u32, @intCast(1000 * i));
    const world = try cs.DrawWorld.create(allocator, seeds[0..options.num_seeds], .{ .forced_draws = options.forced_draws });
    defer world.destroy();

    var w: pr.OpWriter = .{ .allocator = allocator };
    defer w.deinit();
    try makeConstraintOps(&w, @intCast(world.constraints.len), seed, options.frames, options.end_frames);

    var coverage: DrawCoverage = .{};
    var context: pr.PhysicsContext = .{
        .allocator = allocator,
        .constraints = world.constraints,
        .step_constraint = .{ .context = world, .call = cs.DrawWorld.stepCallback },
        .constraint_manager = &world.constraint_manager,
        .body_manager = &world.world.body_manager,
        .on_draw_constraint = if (options.coverage) .{ .context = &coverage, .call = DrawCoverage.onDrawConstraint } else null,
    };
    defer context.deinit();
    try runLayer(name, w.bytes.items, kind, &context, world.jolt_world, null);
    if (options.coverage) try coverage.expectAllReached(name);
}

/// The body layer: a BodyWorld, `options.frames` frames of makeBodyOps
fn runBodyLayer(name: []const u8, seed: u32, kind: Kind, options: LayerOptions) !void {
    try std.testing.expectEqual(@as(u32, @sizeOf(DRBodyDesc)), jolt.jolt_dr_body_desc_size());
    const world = try BodyWorld.create();
    defer world.destroy();

    var w: pr.OpWriter = .{ .allocator = allocator };
    defer w.deinit();
    try makeBodyOps(&w, world, seed, options.frames, options.random_draws, options.end_frames);

    var coverage: DrawCoverage = .{};
    var body_context: DrawCoverage.BodyContext = .{ .coverage = &coverage, .bodies = world.bodies };
    var context: pr.PhysicsContext = .{
        .allocator = allocator,
        .body_manager = &world.body_manager,
        .bodies = world.bodies,
        .on_draw_bodies = if (options.coverage) .{ .context = &body_context, .call = DrawCoverage.BodyContext.onDrawBodies } else null,
    };
    defer context.deinit();
    try runLayer(name, w.bytes.items, kind, &context, null, world.jolt_world);
    if (options.coverage) try coverage.expectAllReached(name);
}

test "Physics draw parity: constraints of every kind, the three draws, draw sizes, frames (call log)" {
    try requireDebugRenderer();
    try runConstraintLayer("constraint draws", 0xd3000001, .parity_renderer, .{ .frames = 6, .num_seeds = 3, .coverage = true });
}

test "Physics draw parity: BodyManager::Draw of catalogue bodies, every flag and color, with a filter (call log)" {
    try requireDebugRenderer();
    try runBodyLayer("body draws", 0xd3000002, .parity_renderer, .{ .frames = 3, .random_draws = 12, .coverage = true });
}

test "Physics draw parity: DebugRendererSimple (call log)" {
    try requireDebugRenderer();
    try runConstraintLayer("constraint draws: DebugRendererSimple", 0xd3000003, .simple, .{ .frames = 3 });
    try runBodyLayer("body draws: DebugRendererSimple", 0xd3000004, .simple, .{ .frames = 1 });
}

test "Physics draw parity: DebugRendererRecorder (.jor bytes of constraint and body frames)" {
    try requireDebugRenderer();
    try runConstraintLayer("constraint draws: recorder", 0xd3000005, .recorder, .{ .frames = 4, .end_frames = true });
    try runBodyLayer("body draws: recorder", 0xd3000006, .recorder, .{ .frames = 2, .end_frames = true });
}
