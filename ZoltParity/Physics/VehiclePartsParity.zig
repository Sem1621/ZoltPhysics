//! Parity tests of the standalone vehicle parts (Jolt/Physics/Vehicle), compared bit for bit with the C++ library on
//! random cases (fixed seeds). C ABI wrappers: ZoltParity/Physics/VehiclePartsReference.cpp. The vehicle simulation
//! itself (VehicleConstraint, the collision testers, the wheeled controller) is compared by the simulation parity
//! (SimulationParity.zig, the vehicle scenes).
//!
//! - VehicleEngine: random settings (torque curves with 0 to 8 points, sorted or not; RPM ranges with min > max,
//!   zero inertia, negative damping) run random scripts of SetCurrentRPM, ApplyTorque, ApplyDamping, GetTorque,
//!   GetAngularVelocity, AllowSleep, ClampRPM and the RPM conversion `cAngularVelocityToRPM`; the current RPM and the
//!   result are compared after every call. The defaults that the binary state does not hold (mInertia,
//!   mAngularDamping, the initial current RPM) are compared too.
//! - VehicleTransmission: random settings (auto, manual and unknown mode bytes, 1 to 8 gear ratios each way, zero times)
//!   run random scripts of Update (delta times, RPMs around the shift points, forward / reverse / no input, can shift up
//!   or not) and Set (gears in range, clutch frictions); after every call the gear, clutch friction, ratio,
//!   IsSwitchingGear, AllowSleep and the SaveState bytes (the timers) are compared.
//! - VehicleDifferentialSettings::CalculateTorqueRatio: random settings (open and limited slip differentials) and wheel
//!   speeds (zero, tiny, negative, huge, infinite, NaN). Release only (Jolt asserts on the settings, Zolt checks the same
//!   asserts in safe builds): splits outside [0, 1], limited slip ratios <= 1 and both speeds infinite.
//! - The default curves (WheelSettingsWV's longitudinal and lateral friction, VehicleEngineSettings' normalized torque)
//!   at random x.
//! - The binary state of every settings class: VehicleEngineSettings, VehicleTransmissionSettings,
//!   VehicleDifferentialSettings, VehicleAntiRollBar, VehicleTrackSettings, WheelSettings, WheelSettingsWV,
//!   WheeledVehicleControllerSettings and VehicleConstraintSettings without wheels (Jolt cannot restore settings with
//!   wheels, CharacterVehicleArchitecture E6). For each: the bytes of the default object and of random objects (random
//!   bit patterns, NaNs and unknown enum bytes included), the cross restore (each side restores the other side's bytes
//!   and saves again) and truncated streams at every length. A truncated stream is restored into an object built from
//!   a template whose arrays have the maximum size, so that the arrays never grow (Jolt's Array::resize leaves new
//!   trivial elements uninitialized); the EOF / failed flags and the bytes saved again are compared. A truncated
//!   VehicleConstraintSettings is cut after the controller hash (Jolt dereferences a null RTTI when the hash cannot be
//!   read) and compared through the restore error.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Ref = zolt.Ref;
const math = zolt.math;
const virtual = zolt.virtual;
const LinearCurve = zolt.LinearCurve;
const StreamIn = zolt.StreamIn;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const StateRecorderImpl = zolt.StateRecorderImpl;
const ConstraintSettings = zolt.ConstraintSettings;
const VehicleEngine = zolt.VehicleEngine;
const VehicleEngineSettings = zolt.VehicleEngineSettings;
const VehicleTransmission = zolt.VehicleTransmission;
const VehicleTransmissionSettings = zolt.VehicleTransmissionSettings;
const VehicleDifferentialSettings = zolt.VehicleDifferentialSettings;
const VehicleAntiRollBar = zolt.VehicleAntiRollBar;
const VehicleTrackSettings = zolt.VehicleTrackSettings;
const WheelSettings = zolt.WheelSettings;
const WheelSettingsWV = zolt.WheelSettingsWV;
const VehicleControllerSettings = zolt.VehicleControllerSettings;
const WheeledVehicleControllerSettings = zolt.WheeledVehicleControllerSettings;
const VehicleConstraintSettings = zolt.VehicleConstraintSettings;

const testing_allocator = std.testing.allocator;

const max_points = 8;
const max_ratios = 8;
const max_track_wheels = 8;
const max_differentials = 4;
const max_anti_roll_bars = 4;
const max_bytes = 4096;

/// Must match VPCurve in VehiclePartsReference.cpp
const VPCurve = extern struct {
    num_points: u32 = 0,
    /// x, y of each point (in this order, not sorted)
    points: [2 * max_points]f32 = @splat(0),
};

/// Must match VPEngine in VehiclePartsReference.cpp: VehicleEngineSettings
const VPEngine = extern struct {
    max_torque: f32,
    min_rpm: f32,
    max_rpm: f32,
    normalized_torque: VPCurve,
    inertia: f32,
    angular_damping: f32,
};

/// Must match VPTransmission in VehiclePartsReference.cpp: VehicleTransmissionSettings
const VPTransmission = extern struct {
    /// Any byte
    mode: u32,
    num_gear_ratios: u32,
    gear_ratios: [max_ratios]f32,
    num_reverse_gear_ratios: u32,
    reverse_gear_ratios: [max_ratios]f32,
    switch_time: f32,
    clutch_release_time: f32,
    switch_latency: f32,
    shift_up_rpm: f32,
    shift_down_rpm: f32,
    clutch_strength: f32,
};

/// Must match VPDifferential in VehiclePartsReference.cpp: VehicleDifferentialSettings
const VPDifferential = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    differential_ratio: f32,
    left_right_split: f32,
    limited_slip_ratio: f32,
    engine_torque_ratio: f32,
};

/// Must match VPAntiRollBar in VehiclePartsReference.cpp: VehicleAntiRollBar
const VPAntiRollBar = extern struct {
    left_wheel: i32,
    right_wheel: i32,
    stiffness: f32,
};

/// Must match VPTrack in VehiclePartsReference.cpp: VehicleTrackSettings
const VPTrack = extern struct {
    driven_wheel: u32,
    num_wheels: u32,
    wheels: [max_track_wheels]u32,
    inertia: f32,
    angular_damping: f32,
    max_brake_torque: f32,
    differential_ratio: f32,
};

/// Must match VPWheel in VehiclePartsReference.cpp: WheelSettings and the members of WheelSettingsWV
const VPWheel = extern struct {
    position: [3]f32,
    suspension_force_point: [3]f32,
    suspension_direction: [3]f32,
    steering_axis: [3]f32,
    wheel_up: [3]f32,
    wheel_forward: [3]f32,
    suspension_min_length: f32,
    suspension_max_length: f32,
    suspension_preload_length: f32,
    /// Any byte
    spring_mode: u32,
    spring_frequency: f32,
    spring_damping: f32,
    radius: f32,
    width: f32,
    enable_suspension_force_point: u32,
    // WheelSettingsWV
    inertia: f32,
    angular_damping: f32,
    max_steer_angle: f32,
    longitudinal_friction: VPCurve,
    lateral_friction: VPCurve,
    max_brake_torque: f32,
    max_hand_brake_torque: f32,
};

/// Must match VPController in VehiclePartsReference.cpp: WheeledVehicleControllerSettings
const VPController = extern struct {
    engine: VPEngine,
    transmission: VPTransmission,
    num_differentials: u32,
    differentials: [max_differentials]VPDifferential,
    differential_limited_slip_ratio: f32,
};

/// Must match VPConstraint in VehiclePartsReference.cpp: VehicleConstraintSettings without wheels
const VPConstraint = extern struct {
    enabled: u32,
    draw_constraint_size: f32,
    constraint_priority: u32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
    up: [3]f32,
    forward: [3]f32,
    max_pitch_roll_angle: f32,
    num_anti_roll_bars: u32,
    anti_roll_bars: [max_anti_roll_bars]VPAntiRollBar,
    controller: VPController,
};

/// The settings classes of jolt_vp_save / jolt_vp_resave (must match VPKind in VehiclePartsReference.cpp)
const VPKind = enum(u32) {
    engine,
    transmission,
    differential,
    anti_roll_bar,
    track,
    wheel_settings,
    wheel_settings_wv,
    controller,
    constraint,
};

/// Must match VPEngineOp in VehiclePartsReference.cpp
const VPEngineOp = extern struct {
    /// 0 SetCurrentRPM(a), 1 ApplyTorque(a, b), 2 ApplyDamping(a), 3 GetTorque(a), 4 GetAngularVelocity, 5 AllowSleep,
    /// 6 ClampRPM, 7 a * cAngularVelocityToRPM
    kind: u32,
    a: f32,
    b: f32,
};

/// Must match VPTransmissionOp in VehiclePartsReference.cpp
const VPTransmissionOp = extern struct {
    /// 0 Update(delta time, rpm, forward, can shift up), 1 Set(gear, clutch friction)
    kind: u32,
    delta_time: f32,
    rpm: f32,
    forward: f32,
    can_shift_up: u32,
    gear: i32,
    clutch_friction: f32,
};

/// Must match VPTransmissionResult in VehiclePartsReference.cpp
const VPTransmissionResult = extern struct {
    gear: i32,
    clutch_friction: f32,
    ratio: f32,
    is_switching_gear: u32,
    allow_sleep: u32,
    /// SaveState: gear, clutch friction and the three timers
    state: [20]u8,
};

const jolt = struct {
    extern fn jolt_vp_struct_sizes(out_sizes: *[12]u32) void;
    extern fn jolt_vp_save(kind: u32, desc: ?*const anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_vp_resave(kind: u32, template: ?*const anyopaque, bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_eof: *u32, out_failed: *u32, out_error: *u32) u32;
    extern fn jolt_vp_engine_defaults(out_inertia: *f32, out_angular_damping: *f32, out_current_rpm: *f32) void;
    extern fn jolt_vp_engine(desc: *const VPEngine, ops: [*]const VPEngineOp, num_ops: u32, out_rpm: [*]f32, out_result: [*]f32) void;
    extern fn jolt_vp_transmission(desc: *const VPTransmission, ops: [*]const VPTransmissionOp, num_ops: u32, out_results: [*]VPTransmissionResult) void;
    extern fn jolt_vp_differential(desc: *const VPDifferential, velocities: [*]const f32, count: u32, out_fractions: [*]f32) void;
    extern fn jolt_vp_default_curves(x: [*]const f32, count: u32, out_values: [*]f32) void;
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt objects of the descriptions (VPMake* of the .cpp)

fn makeCurve(allocator: Allocator, desc: *const VPCurve, curve: *LinearCurve) !void {
    curve.clear();
    for (0..desc.num_points) |i|
        try curve.addPoint(allocator, desc.points[2 * i], desc.points[2 * i + 1]);
}

fn makeEngine(allocator: Allocator, desc: *const VPEngine, s: *VehicleEngineSettings) !void {
    s.max_torque = desc.max_torque;
    s.min_rpm = desc.min_rpm;
    s.max_rpm = desc.max_rpm;
    try makeCurve(allocator, &desc.normalized_torque, &s.normalized_torque);
    s.inertia = desc.inertia;
    s.angular_damping = desc.angular_damping;
}

fn makeTransmission(allocator: Allocator, desc: *const VPTransmission, s: *VehicleTransmissionSettings) !void {
    s.mode = @enumFromInt(@as(u8, @truncate(desc.mode)));
    s.gear_ratios.clearRetainingCapacity();
    try s.gear_ratios.appendSlice(allocator, desc.gear_ratios[0..desc.num_gear_ratios]);
    s.reverse_gear_ratios.clearRetainingCapacity();
    try s.reverse_gear_ratios.appendSlice(allocator, desc.reverse_gear_ratios[0..desc.num_reverse_gear_ratios]);
    s.switch_time = desc.switch_time;
    s.clutch_release_time = desc.clutch_release_time;
    s.switch_latency = desc.switch_latency;
    s.shift_up_rpm = desc.shift_up_rpm;
    s.shift_down_rpm = desc.shift_down_rpm;
    s.clutch_strength = desc.clutch_strength;
}

fn makeDifferential(desc: *const VPDifferential) VehicleDifferentialSettings {
    return .{ .left_wheel = desc.left_wheel, .right_wheel = desc.right_wheel, .differential_ratio = desc.differential_ratio, .left_right_split = desc.left_right_split, .limited_slip_ratio = desc.limited_slip_ratio, .engine_torque_ratio = desc.engine_torque_ratio };
}

fn makeAntiRollBar(desc: *const VPAntiRollBar) VehicleAntiRollBar {
    return .{ .left_wheel = desc.left_wheel, .right_wheel = desc.right_wheel, .stiffness = desc.stiffness };
}

fn makeTrack(allocator: Allocator, desc: *const VPTrack, s: *VehicleTrackSettings) !void {
    s.driven_wheel = desc.driven_wheel;
    s.wheels.clearRetainingCapacity();
    try s.wheels.appendSlice(allocator, desc.wheels[0..desc.num_wheels]);
    s.inertia = desc.inertia;
    s.angular_damping = desc.angular_damping;
    s.max_brake_torque = desc.max_brake_torque;
    s.differential_ratio = desc.differential_ratio;
}

fn vec3(v: *const [3]f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn makeWheel(desc: *const VPWheel, s: *WheelSettings) void {
    s.position = vec3(&desc.position);
    s.suspension_force_point = vec3(&desc.suspension_force_point);
    s.suspension_direction = vec3(&desc.suspension_direction);
    s.steering_axis = vec3(&desc.steering_axis);
    s.wheel_up = vec3(&desc.wheel_up);
    s.wheel_forward = vec3(&desc.wheel_forward);
    s.suspension_min_length = desc.suspension_min_length;
    s.suspension_max_length = desc.suspension_max_length;
    s.suspension_preload_length = desc.suspension_preload_length;
    s.suspension_spring.mode = @enumFromInt(@as(u8, @truncate(desc.spring_mode)));
    s.suspension_spring.frequency = desc.spring_frequency;
    s.suspension_spring.damping = desc.spring_damping;
    s.radius = desc.radius;
    s.width = desc.width;
    s.enable_suspension_force_point = desc.enable_suspension_force_point != 0;
}

fn makeWheelWV(allocator: Allocator, desc: *const VPWheel, s: *WheelSettingsWV) !void {
    makeWheel(desc, &s.base);
    s.inertia = desc.inertia;
    s.angular_damping = desc.angular_damping;
    s.max_steer_angle = desc.max_steer_angle;
    try makeCurve(allocator, &desc.longitudinal_friction, &s.longitudinal_friction);
    try makeCurve(allocator, &desc.lateral_friction, &s.lateral_friction);
    s.max_brake_torque = desc.max_brake_torque;
    s.max_hand_brake_torque = desc.max_hand_brake_torque;
}

fn makeController(allocator: Allocator, desc: *const VPController, s: *WheeledVehicleControllerSettings) !void {
    try makeEngine(allocator, &desc.engine, &s.engine);
    try makeTransmission(allocator, &desc.transmission, &s.transmission);
    s.differentials.clearRetainingCapacity();
    for (desc.differentials[0..desc.num_differentials]) |*d|
        try s.differentials.append(allocator, makeDifferential(d));
    s.differential_limited_slip_ratio = desc.differential_limited_slip_ratio;
}

/// VehicleConstraintSettings of a description (one reference owned by the caller); a null description gives the default
/// settings with a default controller (SaveBinaryState needs one)
fn makeConstraint(allocator: Allocator, desc: ?*const VPConstraint) !Ref(ConstraintSettings) {
    const s = try VehicleConstraintSettings.create(allocator);
    var result = Ref(ConstraintSettings).init(s.asSettings());
    errdefer result.deinit();
    const controller = try WheeledVehicleControllerSettings.create(allocator);
    s.setController(&controller.base);
    if (desc) |d| {
        s.base.enabled = d.enabled != 0;
        s.base.draw_constraint_size = d.draw_constraint_size;
        s.base.constraint_priority = d.constraint_priority;
        s.base.num_velocity_steps_override = d.num_velocity_steps_override;
        s.base.num_position_steps_override = d.num_position_steps_override;
        s.up = vec3(&d.up);
        s.forward = vec3(&d.forward);
        s.max_pitch_roll_angle = d.max_pitch_roll_angle;
        for (d.anti_roll_bars[0..d.num_anti_roll_bars]) |*r|
            try s.addAntiRollBar(makeAntiRollBar(r));
        try makeController(allocator, &d.controller, controller);
    }
    return result;
}

/// SaveBinaryState into an owned slice
fn saveBytes(allocator: Allocator, object: anytype) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var w = StreamOutWrapper.init(&data.writer);
    object.saveBinaryState(w.streamOut());
    if (w.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return data.toOwnedSlice();
}

/// jolt_vp_save on the Zolt side: the bytes of the object of `kind` built from `desc` (null: the default object)
fn zoltSave(allocator: Allocator, kind: VPKind, desc: ?*const anyopaque) ![]u8 {
    switch (kind) {
        .engine => {
            var s = try VehicleEngineSettings.init(allocator);
            defer s.deinit(allocator);
            if (desc) |d| try makeEngine(allocator, @ptrCast(@alignCast(d)), &s);
            return saveBytes(allocator, &s);
        },
        .transmission => {
            var s = try VehicleTransmissionSettings.init(allocator);
            defer s.deinit(allocator);
            if (desc) |d| try makeTransmission(allocator, @ptrCast(@alignCast(d)), &s);
            return saveBytes(allocator, &s);
        },
        .differential => {
            const s: VehicleDifferentialSettings = if (desc) |d| makeDifferential(@ptrCast(@alignCast(d))) else .{};
            return saveBytes(allocator, &s);
        },
        .anti_roll_bar => {
            const s: VehicleAntiRollBar = if (desc) |d| makeAntiRollBar(@ptrCast(@alignCast(d))) else .{};
            return saveBytes(allocator, &s);
        },
        .track => {
            var s: VehicleTrackSettings = .{};
            defer s.deinit(allocator);
            if (desc) |d| try makeTrack(allocator, @ptrCast(@alignCast(d)), &s);
            return saveBytes(allocator, &s);
        },
        .wheel_settings => {
            var ref = Ref(WheelSettings).init(try WheelSettings.create(allocator));
            defer ref.deinit();
            if (desc) |d| makeWheel(@ptrCast(@alignCast(d)), ref.get().?);
            return saveBytes(allocator, ref.get().?);
        },
        .wheel_settings_wv => {
            const s = try WheelSettingsWV.create(allocator);
            var ref = Ref(WheelSettings).init(&s.base);
            defer ref.deinit();
            if (desc) |d| try makeWheelWV(allocator, @ptrCast(@alignCast(d)), s);
            return saveBytes(allocator, ref.get().?);
        },
        .controller => {
            const s = try WheeledVehicleControllerSettings.create(allocator);
            var ref = Ref(VehicleControllerSettings).init(&s.base);
            defer ref.deinit();
            if (desc) |d| try makeController(allocator, @ptrCast(@alignCast(d)), s);
            return saveBytes(allocator, ref.get().?);
        },
        .constraint => {
            var ref = try makeConstraint(allocator, if (desc) |d| @ptrCast(@alignCast(d)) else null);
            defer ref.deinit();
            return saveBytes(allocator, ref.get().?);
        },
    }
}

/// The result of a restore followed by a save
const Resaved = struct {
    bytes: []u8 = &.{},
    eof: u32 = 0,
    failed: u32 = 0,
    /// VehicleConstraintSettings only: 0 restored, 1 "Failed to read type hash", 2 "Failed to create instance of type",
    /// 3 "Failed to restore object"
    err: u32 = 0,

    fn deinit(self: *Resaved, allocator: Allocator) void {
        allocator.free(self.bytes);
    }

    fn eql(self: *const Resaved, other: *const Resaved) bool {
        return self.eof == other.eof and self.failed == other.failed and self.err == other.err and std.mem.eql(u8, self.bytes, other.bytes);
    }
};

/// ConstraintSettings.restoreFromBinaryState (StreamUtils::RestoreObject), the error as a number (see Resaved.err)
fn restoreConstraintSettings(allocator: Allocator, stream: StreamIn, out: *Resaved) !?Ref(ConstraintSettings) {
    var result = try ConstraintSettings.restoreFromBinaryState(allocator, stream);
    defer result.deinit();
    if (result.hasError()) {
        const message = result.getError();
        out.err = if (std.mem.eql(u8, message, "Failed to read type hash")) 1 else if (std.mem.eql(u8, message, "Failed to create instance of type")) 2 else if (std.mem.eql(u8, message, "Failed to restore object")) 3 else 4;
        return null;
    }
    return result.get().clone();
}

/// jolt_vp_resave on the Zolt side
fn zoltResave(allocator: Allocator, kind: VPKind, template: ?*const anyopaque, bytes: []const u8) !Resaved {
    var result: Resaved = .{};
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    const stream = in.streamIn();
    const flags = struct {
        fn set(r: *Resaved, w: *const StreamInWrapper) void {
            r.eof = @intFromBool(w.isEOF());
            r.failed = @intFromBool(w.isFailed());
        }
    };
    switch (kind) {
        .engine => {
            var s = try VehicleEngineSettings.init(allocator);
            defer s.deinit(allocator);
            if (template) |d| try makeEngine(allocator, @ptrCast(@alignCast(d)), &s);
            try s.restoreBinaryState(allocator, stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, &s);
        },
        .transmission => {
            var s = try VehicleTransmissionSettings.init(allocator);
            defer s.deinit(allocator);
            if (template) |d| try makeTransmission(allocator, @ptrCast(@alignCast(d)), &s);
            try s.restoreBinaryState(allocator, stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, &s);
        },
        .differential => {
            var s: VehicleDifferentialSettings = if (template) |d| makeDifferential(@ptrCast(@alignCast(d))) else .{};
            s.restoreBinaryState(stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, &s);
        },
        .anti_roll_bar => {
            var s: VehicleAntiRollBar = if (template) |d| makeAntiRollBar(@ptrCast(@alignCast(d))) else .{};
            s.restoreBinaryState(stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, &s);
        },
        .track => {
            var s: VehicleTrackSettings = .{};
            defer s.deinit(allocator);
            if (template) |d| try makeTrack(allocator, @ptrCast(@alignCast(d)), &s);
            try s.restoreBinaryState(allocator, stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, &s);
        },
        .wheel_settings => {
            var ref = Ref(WheelSettings).init(try WheelSettings.create(allocator));
            defer ref.deinit();
            if (template) |d| makeWheel(@ptrCast(@alignCast(d)), ref.get().?);
            try ref.get().?.restoreBinaryState(stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, ref.get().?);
        },
        .wheel_settings_wv => {
            const s = try WheelSettingsWV.create(allocator);
            var ref = Ref(WheelSettings).init(&s.base);
            defer ref.deinit();
            if (template) |d| try makeWheelWV(allocator, @ptrCast(@alignCast(d)), s);
            try ref.get().?.restoreBinaryState(stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, ref.get().?);
        },
        .controller => {
            const s = try WheeledVehicleControllerSettings.create(allocator);
            var ref = Ref(VehicleControllerSettings).init(&s.base);
            defer ref.deinit();
            if (template) |d| try makeController(allocator, @ptrCast(@alignCast(d)), s);
            try ref.get().?.restoreBinaryState(stream);
            flags.set(&result, &in);
            result.bytes = try saveBytes(allocator, ref.get().?);
        },
        .constraint => {
            const restored = try restoreConstraintSettings(allocator, stream, &result);
            flags.set(&result, &in);
            if (restored) |r| {
                var ref = r;
                defer ref.deinit();
                result.bytes = try saveBytes(allocator, ref.get().?);
            }
        },
    }
    return result;
}

/// jolt_vp_resave
fn joltResave(allocator: Allocator, kind: VPKind, template: ?*const anyopaque, bytes: []const u8) !Resaved {
    var result: Resaved = .{};
    var buffer: [max_bytes]u8 = undefined;
    const size = jolt.jolt_vp_resave(@intFromEnum(kind), template, bytes.ptr, @intCast(bytes.len), &buffer, max_bytes, &result.eof, &result.failed, &result.err);
    std.debug.assert(size <= max_bytes);
    result.bytes = try allocator.dupe(u8, buffer[0..size]);
    return result;
}

/// jolt_vp_save
fn joltSave(allocator: Allocator, kind: VPKind, desc: ?*const anyopaque) ![]u8 {
    var buffer: [max_bytes]u8 = undefined;
    const size = jolt.jolt_vp_save(@intFromEnum(kind), desc, &buffer, max_bytes);
    std.debug.assert(size <= max_bytes);
    return allocator.dupe(u8, buffer[0..size]);
}

// ---------------------------------------------------------------------------------------------------------------------
// Random descriptions

const Rng = fw.Rng;

/// A random float: mostly in [min, max), sometimes a special value (zero, -0, tiny, huge, infinite, NaN when `nan`)
fn randomFloat(rng: *Rng, min: f32, max: f32, opts: struct { specials: bool = true, nan: bool = false }) f32 {
    if (opts.specials) {
        switch (rng.next() % 32) {
            0 => return 0.0,
            1 => return -0.0,
            2 => return 1.0e-30,
            3 => return -1.0e20,
            4 => return 1.0e20,
            5 => return if (opts.nan) std.math.nan(f32) else std.math.inf(f32),
            6 => return @bitCast(rng.next()), // Any bit pattern (NaNs included)
            else => {},
        }
    }
    return rng.float(min, max);
}

/// A random bit pattern float for the binary state tests (finite, special or any bits)
fn randomBits(rng: *Rng) f32 {
    return switch (rng.next() % 4) {
        0 => @bitCast(rng.next()),
        else => randomFloat(rng, -100.0, 100.0, .{ .nan = true }),
    };
}

fn randomCurve(rng: *Rng, opts: struct { max: u32 = max_points, sorted: bool = false, bits: bool = false }) VPCurve {
    var c: VPCurve = .{ .num_points = rng.intRange(u32, 0, opts.max) };
    var x: f32 = rng.float(-0.5, 0.5);
    for (0..c.num_points) |i| {
        if (opts.bits) {
            c.points[2 * i] = randomBits(rng);
            c.points[2 * i + 1] = randomBits(rng);
        } else {
            x = if (opts.sorted or rng.next() % 4 != 0) x + rng.float(0.0, 0.5) else rng.float(-0.5, 1.5);
            c.points[2 * i] = x;
            c.points[2 * i + 1] = rng.float(0.0, 1.5);
        }
    }
    return c;
}

fn randomEngine(rng: *Rng, opts: struct { bits: bool = false, curve_max: u32 = max_points }) VPEngine {
    if (opts.bits)
        return .{ .max_torque = randomBits(rng), .min_rpm = randomBits(rng), .max_rpm = randomBits(rng), .normalized_torque = randomCurve(rng, .{ .max = opts.curve_max, .bits = true }), .inertia = randomBits(rng), .angular_damping = randomBits(rng) };
    const min_rpm = randomFloat(rng, 0.0, 2000.0, .{});
    return .{
        .max_torque = randomFloat(rng, 0.0, 1000.0, .{}),
        .min_rpm = min_rpm,
        .max_rpm = if (rng.next() % 8 == 0) randomFloat(rng, 0.0, 2000.0, .{}) else min_rpm + rng.float(0.0, 8000.0), // Sometimes below the minimum
        .normalized_torque = randomCurve(rng, .{ .max = opts.curve_max }),
        .inertia = randomFloat(rng, 0.01, 2.0, .{}),
        .angular_damping = randomFloat(rng, -0.5, 2.0, .{}),
    };
}

fn randomTransmission(rng: *Rng, opts: struct { bits: bool = false, min_ratios: u32 = 0, max_ratios: u32 = max_ratios }) VPTransmission {
    var t: VPTransmission = undefined;
    t.mode = switch (rng.next() % 8) {
        0, 1, 2, 3 => 0,
        4, 5, 6 => 1,
        else => rng.next() & 0xff, // Unknown modes behave like manual
    };
    t.num_gear_ratios = rng.intRange(u32, opts.min_ratios, opts.max_ratios);
    t.num_reverse_gear_ratios = rng.intRange(u32, opts.min_ratios, opts.max_ratios);
    for (&t.gear_ratios, 0..) |*r, i| r.* = if (opts.bits) randomBits(rng) else 3.0 - @as(f32, @floatFromInt(i)) * 0.3;
    for (&t.reverse_gear_ratios, 0..) |*r, i| r.* = if (opts.bits) randomBits(rng) else -3.0 + @as(f32, @floatFromInt(i)) * 0.3;
    if (opts.bits) {
        t.switch_time = randomBits(rng);
        t.clutch_release_time = randomBits(rng);
        t.switch_latency = randomBits(rng);
        t.shift_up_rpm = randomBits(rng);
        t.shift_down_rpm = randomBits(rng);
        t.clutch_strength = randomBits(rng);
    } else {
        t.switch_time = if (rng.next() % 4 == 0) 0.0 else rng.float(0.0, 0.6);
        t.clutch_release_time = if (rng.next() % 4 == 0) 0.0 else rng.float(0.0, 0.6);
        t.switch_latency = if (rng.next() % 4 == 0) 0.0 else rng.float(0.0, 0.6);
        t.shift_down_rpm = rng.float(1000.0, 3000.0);
        t.shift_up_rpm = t.shift_down_rpm + rng.float(500.0, 3000.0);
        t.clutch_strength = rng.float(1.0, 20.0);
    }
    return t;
}

fn randomDifferential(rng: *Rng, opts: struct { bits: bool = false, release_only: bool = false }) VPDifferential {
    if (opts.bits)
        return .{ .left_wheel = @bitCast(rng.next()), .right_wheel = @bitCast(rng.next()), .differential_ratio = randomBits(rng), .left_right_split = randomBits(rng), .limited_slip_ratio = randomBits(rng), .engine_torque_ratio = randomBits(rng) };
    return .{
        .left_wheel = rng.intRange(i32, -1, 3),
        .right_wheel = rng.intRange(i32, -1, 3),
        .differential_ratio = rng.float(0.5, 6.0),
        .left_right_split = if (opts.release_only) rng.float(-1.0, 2.0) else switch (rng.next() % 4) {
            0 => 0.0,
            1 => 1.0,
            else => rng.float(0.0, 1.0),
        },
        .limited_slip_ratio = if (opts.release_only and rng.next() % 2 == 0) rng.float(0.0, 1.0) else switch (rng.next() % 4) {
            0 => math.flt_max, // Open differential
            1 => if (opts.release_only) 1.0 else 1.0001,
            else => rng.float(1.01, 10.0),
        },
        .engine_torque_ratio = rng.float(0.0, 1.0),
    };
}

fn randomAntiRollBar(rng: *Rng) VPAntiRollBar {
    return .{ .left_wheel = @bitCast(rng.next()), .right_wheel = rng.intRange(i32, -2, 8), .stiffness = randomBits(rng) };
}

fn randomTrack(rng: *Rng, opts: struct { max_wheels: u32 = max_track_wheels }) VPTrack {
    var t: VPTrack = .{ .driven_wheel = rng.next(), .num_wheels = rng.intRange(u32, 0, opts.max_wheels), .wheels = undefined, .inertia = randomBits(rng), .angular_damping = randomBits(rng), .max_brake_torque = randomBits(rng), .differential_ratio = randomBits(rng) };
    for (&t.wheels) |*w| w.* = rng.next() % 16;
    return t;
}

fn randomVec3Bits(rng: *Rng) [3]f32 {
    return .{ randomBits(rng), randomBits(rng), randomBits(rng) };
}

fn randomWheel(rng: *Rng, opts: struct { curve_max: u32 = max_points }) VPWheel {
    return .{
        .position = randomVec3Bits(rng),
        .suspension_force_point = randomVec3Bits(rng),
        .suspension_direction = randomVec3Bits(rng),
        .steering_axis = randomVec3Bits(rng),
        .wheel_up = randomVec3Bits(rng),
        .wheel_forward = randomVec3Bits(rng),
        .suspension_min_length = randomBits(rng),
        .suspension_max_length = randomBits(rng),
        .suspension_preload_length = randomBits(rng),
        .spring_mode = if (rng.next() % 4 == 0) rng.next() & 0xff else rng.next() % 3,
        .spring_frequency = randomBits(rng),
        .spring_damping = randomBits(rng),
        .radius = randomBits(rng),
        .width = randomBits(rng),
        .enable_suspension_force_point = rng.next() % 2,
        .inertia = randomBits(rng),
        .angular_damping = randomBits(rng),
        .max_steer_angle = randomBits(rng),
        .longitudinal_friction = randomCurve(rng, .{ .max = opts.curve_max, .bits = true }),
        .lateral_friction = randomCurve(rng, .{ .max = opts.curve_max, .bits = true }),
        .max_brake_torque = randomBits(rng),
        .max_hand_brake_torque = randomBits(rng),
    };
}

fn randomController(rng: *Rng, opts: struct { curve_max: u32 = max_points, max_ratios: u32 = max_ratios, max_differentials: u32 = max_differentials }) VPController {
    var c: VPController = .{
        .engine = randomEngine(rng, .{ .bits = true, .curve_max = opts.curve_max }),
        .transmission = randomTransmission(rng, .{ .bits = true, .max_ratios = opts.max_ratios }),
        .num_differentials = rng.intRange(u32, 0, opts.max_differentials),
        .differentials = undefined,
        .differential_limited_slip_ratio = randomBits(rng),
    };
    for (&c.differentials) |*d| d.* = randomDifferential(rng, .{ .bits = true });
    return c;
}

fn randomConstraint(rng: *Rng) VPConstraint {
    var c: VPConstraint = .{
        .enabled = rng.next() % 2,
        .draw_constraint_size = randomBits(rng),
        .constraint_priority = rng.next(),
        .num_velocity_steps_override = rng.next(),
        .num_position_steps_override = rng.next(),
        .up = randomVec3Bits(rng),
        .forward = randomVec3Bits(rng),
        .max_pitch_roll_angle = randomBits(rng),
        .num_anti_roll_bars = rng.intRange(u32, 0, max_anti_roll_bars),
        .anti_roll_bars = undefined,
        .controller = randomController(rng, .{}),
    };
    for (&c.anti_roll_bars) |*r| r.* = randomAntiRollBar(rng);
    return c;
}

/// Fill the arrays of a template to their maximum size (truncated restores never grow an array)
fn fullCurve(c: *VPCurve) void {
    c.num_points = max_points;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "VehicleParts: struct sizes match the C++ side" {
    var sizes: [12]u32 = undefined;
    jolt.jolt_vp_struct_sizes(&sizes);
    const expected = [12]usize{ @sizeOf(VPCurve), @sizeOf(VPEngine), @sizeOf(VPTransmission), @sizeOf(VPDifferential), @sizeOf(VPAntiRollBar), @sizeOf(VPTrack), @sizeOf(VPWheel), @sizeOf(VPController), @sizeOf(VPConstraint), @sizeOf(VPEngineOp), @sizeOf(VPTransmissionOp), @sizeOf(VPTransmissionResult) };
    for (sizes, expected) |s, e|
        try std.testing.expectEqual(e, @as(usize, s));
}

test "VehicleParts: the default settings, the engine defaults and the default curves" {
    const allocator = testing_allocator;
    var checker: Checker = .{ .name = "VehicleParts defaults" };

    // The binary state of every default settings object
    inline for (comptime std.enums.values(VPKind)) |kind| {
        const z = try zoltSave(allocator, kind, null);
        defer allocator.free(z);
        const j = try joltSave(allocator, kind, null);
        defer allocator.free(j);
        if (!std.mem.eql(u8, z, j)) {
            std.debug.print("VehicleParts defaults: {s} bytes differ\n  zolt: {any}\n  jolt: {any}\n", .{ @tagName(kind), z, j });
            checker.mismatches += 1;
        }
    }

    // The defaults that the binary state does not hold
    var jolt_defaults: [3]f32 = undefined;
    jolt.jolt_vp_engine_defaults(&jolt_defaults[0], &jolt_defaults[1], &jolt_defaults[2]);
    var engine = try VehicleEngine.init(allocator);
    defer engine.deinit(allocator);
    checker.check("engine defaults", [3]f32{ engine.base.inertia, engine.base.angular_damping, engine.getCurrentRPM() }, jolt_defaults);

    // The default curves (WheelSettingsWV's frictions, VehicleEngineSettings' normalized torque)
    var rng: Rng = .{ .state = 0x5eed_c0de };
    const n = 4096;
    var x: [n]f32 = undefined;
    for (&x, 0..) |*v, i| v.* = switch (i % 8) {
        0 => rng.float(-1.0, 1.0),
        1 => rng.float(0.0, 0.3),
        2 => rng.float(0.0, 30.0),
        3 => @as(f32, @floatFromInt(i % 25)), // On the points
        4 => randomFloat(&rng, -10.0, 10.0, .{ .nan = true }),
        else => rng.float(-5.0, 50.0),
    };
    var jolt_values: [3 * n]f32 = undefined;
    jolt.jolt_vp_default_curves(&x, n, &jolt_values);
    const wheel = try WheelSettingsWV.create(allocator);
    var wheel_ref = Ref(WheelSettings).init(&wheel.base);
    defer wheel_ref.deinit();
    for (x, 0..) |v, i| {
        const z = [3]f32{ wheel.longitudinal_friction.getValue(v), wheel.lateral_friction.getValue(v), engine.base.normalized_torque.getValue(v) };
        checker.check(v, z, jolt_values[3 * i ..][0..3].*);
    }
    try checker.finish();
}

test "VehicleParts: VehicleEngine torque, damping, clamping and RPM conversions" {
    const allocator = testing_allocator;
    var checker: Checker = .{ .name = "VehicleEngine" };
    var rng: Rng = .{ .state = 0xe9e1_0001 };
    const max_ops = 24;
    for (0..20_000) |_| {
        const desc = randomEngine(&rng, .{});
        var ops: [max_ops]VPEngineOp = undefined;
        const num_ops = rng.intRange(u32, 1, max_ops);
        for (ops[0..num_ops]) |*op| {
            op.kind = rng.next() % 8;
            op.a = switch (op.kind) {
                0 => randomFloat(&rng, -1000.0, 9000.0, .{ .nan = true }), // RPM
                1 => randomFloat(&rng, -1000.0, 1000.0, .{ .nan = true }), // Torque
                2 => randomFloat(&rng, 0.0, 0.1, .{}), // Delta time
                3 => randomFloat(&rng, -0.5, 1.5, .{ .nan = true }), // Acceleration
                else => randomFloat(&rng, -500.0, 500.0, .{ .nan = true }), // Angular velocity
            };
            op.b = randomFloat(&rng, 0.0, 0.1, .{});
        }

        var jolt_rpm: [max_ops]f32 = undefined;
        var jolt_result: [max_ops]f32 = undefined;
        jolt.jolt_vp_engine(&desc, &ops, num_ops, &jolt_rpm, &jolt_result);

        var engine: VehicleEngine = .{};
        defer engine.deinit(allocator);
        try makeEngine(allocator, &desc, &engine.base);
        for (ops[0..num_ops], 0..) |op, i| {
            var result: f32 = 0.0;
            switch (op.kind) {
                0 => engine.setCurrentRPM(op.a),
                1 => engine.applyTorque(op.a, op.b),
                2 => engine.applyDamping(op.a),
                3 => result = engine.getTorque(op.a),
                4 => result = engine.getAngularVelocity(),
                5 => result = if (engine.allowSleep()) 1.0 else 0.0,
                6 => engine.clampRPM(),
                else => result = op.a * VehicleEngine.angular_velocity_to_rpm,
            }
            checker.check(.{ desc, op }, [2]f32{ engine.getCurrentRPM(), result }, [2]f32{ jolt_rpm[i], jolt_result[i] });
        }
    }
    try checker.finish();
}

test "VehicleParts: VehicleTransmission gear switching, clutch and timers" {
    const allocator = testing_allocator;
    var checker: Checker = .{ .name = "VehicleTransmission" };
    var rng: Rng = .{ .state = 0x7a45_0002 };
    const max_ops = 48;
    for (0..10_000) |_| {
        const desc = randomTransmission(&rng, .{ .min_ratios = 1 });
        var ops: [max_ops]VPTransmissionOp = undefined;
        const num_ops = rng.intRange(u32, 1, max_ops);
        const forward_choices = [_]f32{ 1.0, 0.5, 0.0, -0.0, -0.5, -1.0 };
        for (ops[0..num_ops]) |*op| {
            op.* = .{
                .kind = if (rng.next() % 10 == 0) 1 else 0,
                .delta_time = switch (rng.next() % 4) {
                    0 => 1.0 / 60.0,
                    1 => 0.0,
                    else => rng.float(0.0, 0.3),
                },
                .rpm = switch (rng.next() % 4) {
                    0 => desc.shift_up_rpm, // On the shift points
                    1 => desc.shift_down_rpm,
                    else => rng.float(0.0, 7000.0),
                },
                .forward = if (rng.next() % 2 == 0) forward_choices[rng.next() % forward_choices.len] else rng.float(-1.0, 1.0),
                .can_shift_up = @intFromBool(rng.next() % 4 != 0),
                .gear = rng.intRange(i32, -@as(i32, @intCast(desc.num_reverse_gear_ratios)), @intCast(desc.num_gear_ratios)),
                .clutch_friction = rng.float(0.0, 1.0),
            };
        }

        var jolt_results: [max_ops]VPTransmissionResult = undefined;
        jolt.jolt_vp_transmission(&desc, &ops, num_ops, &jolt_results);

        var transmission: VehicleTransmission = .{};
        defer transmission.deinit(allocator);
        try makeTransmission(allocator, &desc, &transmission.base);
        for (ops[0..num_ops], 0..) |op, i| {
            if (op.kind == 0)
                transmission.update(op.delta_time, op.rpm, op.forward, op.can_shift_up != 0)
            else
                transmission.set(op.gear, op.clutch_friction);

            var r: VPTransmissionResult = .{ .gear = transmission.getCurrentGear(), .clutch_friction = transmission.getClutchFriction(), .ratio = transmission.getCurrentRatio(), .is_switching_gear = @intFromBool(transmission.isSwitchingGear()), .allow_sleep = @intFromBool(transmission.allowSleep()), .state = undefined };
            var recorder = StateRecorderImpl.init(allocator);
            defer recorder.deinit();
            transmission.saveState(recorder.recorder());
            @memcpy(&r.state, recorder.getData());
            checker.check(.{ desc.mode, op }, r, jolt_results[i]);
        }
    }
    try checker.finish();
}

fn checkDifferentials(checker: *Checker, rng: *Rng, count: usize, release_only: bool) void {
    const n = 64;
    for (0..count) |_| {
        const desc = randomDifferential(rng, .{ .release_only = release_only });
        var velocities: [2 * n]f32 = undefined;
        for (&velocities) |*v| {
            v.* = switch (rng.next() % 8) {
                0 => rng.float(-1.0e-3, 1.0e-3), // Around the minimum velocity
                1 => randomFloat(rng, -100.0, 100.0, .{ .nan = true }),
                else => rng.float(-50.0, 50.0),
            };
        }
        if (!release_only) {
            // Jolt asserts that alpha >= 0: both infinite gives inf / inf (NaN)
            for (0..n) |i| {
                if (std.math.isInf(velocities[2 * i]) and std.math.isInf(velocities[2 * i + 1]))
                    velocities[2 * i] = 1.0;
            }
        }
        var jolt_fractions: [2 * n]f32 = undefined;
        jolt.jolt_vp_differential(&desc, &velocities, n, &jolt_fractions);
        const d = makeDifferential(&desc);
        for (0..n) |i| {
            const r = d.calculateTorqueRatio(velocities[2 * i], velocities[2 * i + 1]);
            checker.check(.{ desc, velocities[2 * i], velocities[2 * i + 1] }, [2]f32{ r.left, r.right }, jolt_fractions[2 * i ..][0..2].*);
        }
    }
}

test "VehicleParts: VehicleDifferentialSettings CalculateTorqueRatio" {
    var checker: Checker = .{ .name = "VehicleDifferential" };
    var rng: Rng = .{ .state = 0xd1ff_0003 };
    checkDifferentials(&checker, &rng, 2_000, false);
    try checker.finish();
}

test "VehicleParts: VehicleDifferentialSettings CalculateTorqueRatio outside Jolt's asserts (release only)" {
    if (zolt.Core.enable_asserts) return error.SkipZigTest; // Jolt asserts on these settings, Zolt checks the same asserts in safe builds
    var checker: Checker = .{ .name = "VehicleDifferential release only" };
    var rng: Rng = .{ .state = 0xd1ff_0004 };
    checkDifferentials(&checker, &rng, 2_000, true);
    try checker.finish();
}

/// Save, cross restore and truncated restores of one settings class. `random` makes a description, `template` the
/// description of the objects that the truncated streams restore into (arrays of the maximum size).
fn checkBinaryState(comptime kind: VPKind, comptime Desc: type, count: usize, seed: u32, comptime random: fn (*Rng) Desc, comptime template: fn (*Rng) Desc) !void {
    const allocator = testing_allocator;
    var checker: Checker = .{ .name = "VehicleParts binary state " ++ @tagName(kind) };
    var rng: Rng = .{ .state = seed };
    for (0..count) |_| {
        const desc = random(&rng);

        // The bytes
        const z = try zoltSave(allocator, kind, &desc);
        defer allocator.free(z);
        const j = try joltSave(allocator, kind, &desc);
        defer allocator.free(j);
        if (!std.mem.eql(u8, z, j)) {
            if (checker.mismatches < 5) std.debug.print("{s}: bytes differ for {any}\n  zolt: {any}\n  jolt: {any}\n", .{ checker.name, desc, z, j });
            checker.mismatches += 1;
            continue;
        }

        // Cross restore: each side restores the other side's bytes and saves again
        {
            var zr = try zoltResave(allocator, kind, null, j);
            defer zr.deinit(allocator);
            var jr = try joltResave(allocator, kind, null, z);
            defer jr.deinit(allocator);
            if (!zr.eql(&jr) or !std.mem.eql(u8, zr.bytes, j) or zr.eof != 0 or zr.failed != 0 or zr.err != 0) {
                if (checker.mismatches < 5) std.debug.print("{s}: cross restore differs for {any}\n  zolt: {any} eof {d} failed {d} err {d}\n  jolt: {any} eof {d} failed {d} err {d}\n", .{ checker.name, desc, zr.bytes, zr.eof, zr.failed, zr.err, jr.bytes, jr.eof, jr.failed, jr.err });
                checker.mismatches += 1;
            }
        }

        // Truncated streams
        const t = template(&rng);
        const first = if (kind == .constraint) constraintHashEnd(&desc) else 0;
        for (first..j.len) |len| {
            var zr = try zoltResave(allocator, kind, &t, j[0..len]);
            defer zr.deinit(allocator);
            var jr = try joltResave(allocator, kind, &t, j[0..len]);
            defer jr.deinit(allocator);
            if (!zr.eql(&jr) or zr.eof == 0) {
                if (checker.mismatches < 5) std.debug.print("{s}: truncated restore at {d} differs for {any}\n  zolt: {any} eof {d} failed {d} err {d}\n  jolt: {any} eof {d} failed {d} err {d}\n", .{ checker.name, len, desc, zr.bytes, zr.eof, zr.failed, zr.err, jr.bytes, jr.eof, jr.failed, jr.err });
                checker.mismatches += 1;
            }
        }
    }
    try checker.finish();
}

/// The offset after the controller hash in the bytes of a VehicleConstraintSettings without wheels
fn constraintHashEnd(desc: anytype) usize {
    const d: *const VPConstraint = desc;
    // RTTI hash, enabled, draw size, priority, velocity / position steps, up, forward, max pitch roll angle, the
    // anti-roll bars, the number of wheels, the controller hash
    return 4 + 1 + 4 + 4 + 4 + 4 + 12 + 12 + 4 + 4 + 12 * @as(usize, d.num_anti_roll_bars) + 4 + 4;
}

const descs = struct {
    fn engine(rng: *Rng) VPEngine {
        return randomEngine(rng, .{ .bits = true });
    }
    fn engineTemplate(rng: *Rng) VPEngine {
        var e = randomEngine(rng, .{ .bits = true });
        fullCurve(&e.normalized_torque);
        return e;
    }
    fn transmission(rng: *Rng) VPTransmission {
        return randomTransmission(rng, .{ .bits = true });
    }
    fn transmissionTemplate(rng: *Rng) VPTransmission {
        var t = randomTransmission(rng, .{ .bits = true });
        t.num_gear_ratios = max_ratios;
        t.num_reverse_gear_ratios = max_ratios;
        return t;
    }
    fn differential(rng: *Rng) VPDifferential {
        return randomDifferential(rng, .{ .bits = true });
    }
    fn antiRollBar(rng: *Rng) VPAntiRollBar {
        return randomAntiRollBar(rng);
    }
    fn track(rng: *Rng) VPTrack {
        return randomTrack(rng, .{});
    }
    fn trackTemplate(rng: *Rng) VPTrack {
        var t = randomTrack(rng, .{});
        t.num_wheels = max_track_wheels;
        return t;
    }
    fn wheel(rng: *Rng) VPWheel {
        return randomWheel(rng, .{});
    }
    fn wheelTemplate(rng: *Rng) VPWheel {
        var w = randomWheel(rng, .{});
        fullCurve(&w.longitudinal_friction);
        fullCurve(&w.lateral_friction);
        return w;
    }
    fn controller(rng: *Rng) VPController {
        return randomController(rng, .{});
    }
    fn controllerTemplate(rng: *Rng) VPController {
        var c = randomController(rng, .{});
        fullCurve(&c.engine.normalized_torque);
        c.transmission.num_gear_ratios = max_ratios;
        c.transmission.num_reverse_gear_ratios = max_ratios;
        return c;
    }
    fn constraint(rng: *Rng) VPConstraint {
        return randomConstraint(rng);
    }
};

test "VehicleParts: binary state of VehicleEngineSettings" {
    try checkBinaryState(.engine, VPEngine, 600, 0xb1a5_0001, descs.engine, descs.engineTemplate);
}

test "VehicleParts: binary state of VehicleTransmissionSettings" {
    try checkBinaryState(.transmission, VPTransmission, 600, 0xb1a5_0002, descs.transmission, descs.transmissionTemplate);
}

test "VehicleParts: binary state of VehicleDifferentialSettings" {
    try checkBinaryState(.differential, VPDifferential, 1000, 0xb1a5_0003, descs.differential, descs.differential);
}

test "VehicleParts: binary state of VehicleAntiRollBar" {
    try checkBinaryState(.anti_roll_bar, VPAntiRollBar, 1000, 0xb1a5_0004, descs.antiRollBar, descs.antiRollBar);
}

test "VehicleParts: binary state of VehicleTrackSettings" {
    try checkBinaryState(.track, VPTrack, 600, 0xb1a5_0005, descs.track, descs.trackTemplate);
}

test "VehicleParts: binary state of WheelSettings" {
    try checkBinaryState(.wheel_settings, VPWheel, 300, 0xb1a5_0006, descs.wheel, descs.wheelTemplate);
}

test "VehicleParts: binary state of WheelSettingsWV" {
    try checkBinaryState(.wheel_settings_wv, VPWheel, 300, 0xb1a5_0007, descs.wheel, descs.wheelTemplate);
}

test "VehicleParts: binary state of WheeledVehicleControllerSettings" {
    try checkBinaryState(.controller, VPController, 200, 0xb1a5_0008, descs.controller, descs.controllerTemplate);
}

test "VehicleParts: binary state of VehicleConstraintSettings without wheels" {
    try checkBinaryState(.constraint, VPConstraint, 200, 0xb1a5_0009, descs.constraint, descs.constraint);
}
