//! Parity tests for CharacterBase and CharacterID (Phase 7 foundation): the CharacterBaseSettings defaults,
//! SetMaxSlopeAngle (Jolt's Cos, through the constructor and the setter) / GetCosMaxSlopeAngle, IsSlopeTooSteep (random
//! up vectors, normals and angles, angles around the no-max-slope threshold 0.9999 and normals at the exact cos), the
//! SaveState bytes of a CharacterBase with random ground members and the cross RestoreState (each side restores the
//! other side's bytes, the ground user data and material are reset), sToString of every EGroundState, CharacterID's
//! sSetNextCharacterID / sNextCharacterID sequence (the wrap-around skips the invalid ID) and GetHash. Zolt and the C++
//! Jolt library run on the same inputs and must produce identical bits. C ABI wrappers:
//! ZoltParity/Physics/CharacterBaseReference.cpp. See ZoltParity/parity.zig for how parity tests work.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const BodyID = zolt.BodyID;
const CharacterBase = zolt.CharacterBase;
const CharacterBaseSettings = zolt.CharacterBaseSettings;
const CharacterID = zolt.CharacterID;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsSystem = zolt.PhysicsSystem;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const StateRecorderImpl = zolt.StateRecorderImpl;
const SubShapeID = zolt.SubShapeID;
const Vec3 = zolt.Vec3;

/// The C++ reference functions, see CharacterBaseReference.cpp
const jolt = struct {
    extern fn jolt_cb_settings_defaults(out_up: *[3]f32, out_plane: *[4]f32, out_max_slope_angle: *f32, out_enhanced_internal_edge_removal: *u32, out_has_shape: *u32) void;
    extern fn jolt_cb_cos_max_slope_angle(angles: [*]const f32, count: u32, out_cos: [*]f32) void;
    extern fn jolt_cb_is_slope_too_steep(up: *const [3]f32, max_slope_angle: f32, normals: [*]const [3]f32, count: u32, out_results: [*]u32) void;
    extern fn jolt_cb_save_state(ground: *const CBGround, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_cb_restore_state(bytes: [*]const u8, size: u32, out_ground: *CBGround, out_reset: *u32) u32;
    extern fn jolt_cb_to_string(state: u32, out_text: [*]u8, capacity: u32) u32;
    extern fn jolt_cid_sequence(next_value: u32, count: u32, out_ids: [*]u32) void;
    extern fn jolt_cid_hash(ids: [*]const u32, count: u32, out_hashes: [*]u64) void;
};

/// The ground members of a CharacterBase (CBGround of CharacterBaseReference.cpp)
const CBGround = extern struct {
    state: u32,
    body_id: u32,
    sub_shape_id: u32,
    position: [3]Real,
    normal: [3]f32,
    velocity: [3]f32,
};

/// The concrete character of the tests (CBTestCharacter of the .cpp): CharacterBase's SaveState / RestoreState
const TestCharacter = struct {
    pub const overrides = .{};

    base: CharacterBase,
};

/// The settings of TestCharacter
const TestCharacterSettings = struct {
    pub const overrides = .{};

    base: CharacterBaseSettings,
};

/// A test character from settings with `max_slope_angle` and `up` (the system is only stored by CharacterBase)
fn makeCharacter(allocator: Allocator, system: *PhysicsSystem, max_slope_angle: f32, up: Vec3) TestCharacter {
    var settings = TestCharacterSettings{ .base = .init(TestCharacterSettings, allocator) };
    defer settings.base.deinit();
    settings.base.max_slope_angle = max_slope_angle;
    settings.base.up = up;
    return .{ .base = .init(TestCharacter, allocator, &settings.base, system) };
}

fn setGround(base: *CharacterBase, g: *const CBGround) void {
    base.ground_state = @enumFromInt(@as(i32, @bitCast(g.state)));
    base.ground_body_id = .{ .id = g.body_id };
    base.ground_body_sub_shape_id = .{ .value = g.sub_shape_id };
    base.ground_position = RVec3.init(g.position[0], g.position[1], g.position[2]);
    base.ground_normal = Vec3.init(g.normal[0], g.normal[1], g.normal[2]);
    base.ground_velocity = Vec3.init(g.velocity[0], g.velocity[1], g.velocity[2]);
}

fn getGround(base: *const CharacterBase) CBGround {
    const p = base.getGroundPosition();
    const n = base.getGroundNormal();
    const v = base.getGroundVelocity();
    return .{
        .state = @bitCast(@intFromEnum(base.getGroundState())),
        .body_id = base.getGroundBodyID().getIndexAndSequenceNumber(),
        .sub_shape_id = base.getGroundSubShapeID().getValue(),
        .position = .{ p.getX(), p.getY(), p.getZ() },
        .normal = .{ n.getX(), n.getY(), n.getZ() },
        .velocity = .{ v.getX(), v.getY(), v.getZ() },
    };
}

fn randomUnit(rng: *fw.Rng) [3]f32 {
    while (true) {
        const v = rng.floatArray(3, -1, 1);
        const len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        if (len_sq > 0.01 and len_sq <= 1) {
            const len = @sqrt(len_sq);
            return .{ v[0] / len, v[1] / len, v[2] / len };
        }
    }
}

test "CharacterBaseSettings defaults are Jolt's" {
    var up: [3]f32 = undefined;
    var plane: [4]f32 = undefined;
    var max_slope_angle: f32 = undefined;
    var enhanced_internal_edge_removal: u32 = undefined;
    var has_shape: u32 = undefined;
    jolt.jolt_cb_settings_defaults(&up, &plane, &max_slope_angle, &enhanced_internal_edge_removal, &has_shape);

    var settings = TestCharacterSettings{ .base = .init(TestCharacterSettings, std.testing.allocator) };
    defer settings.base.deinit();
    const s = &settings.base;
    var checker: Checker = .{ .name = "CharacterBaseSettings defaults" };
    checker.check({}, [3]f32{ s.up.getX(), s.up.getY(), s.up.getZ() }, up);
    const n = s.supporting_volume.getNormal();
    checker.check({}, [4]f32{ n.getX(), n.getY(), n.getZ(), s.supporting_volume.getConstant() }, plane);
    checker.check({}, s.max_slope_angle, max_slope_angle);
    checker.check({}, @as(u32, @intFromBool(s.enhanced_internal_edge_removal)), enhanced_internal_edge_removal);
    checker.check({}, @as(u32, @intFromBool(s.shape.get() != null)), has_shape);
    try checker.finish();
}

test "CharacterBase SetMaxSlopeAngle / GetCosMaxSlopeAngle" {
    const allocator = std.testing.allocator;
    var system: PhysicsSystem = .empty;
    var rng: fw.Rng = .{};
    var checker: Checker = .{ .name = "CharacterBase SetMaxSlopeAngle" };
    const batch = 256;
    var angles: [batch]f32 = undefined;
    var jolt_cos: [batch]f32 = undefined;
    const special = [_]f32{ 0, zolt.math.degreesToRadians(50.0), 0.25 * std.math.pi, 0.5 * std.math.pi, std.math.pi, -0.5 * std.math.pi, 2 * std.math.pi, 1.0e-3, 0.01414, 0.014142, 0.0141422 };
    for (0..fw.iterations / batch) |iteration| {
        for (&angles, 0..) |*a, i| {
            a.* = if (iteration == 0 and i < special.len) special[i] else rng.float(-7, 7);
        }
        jolt.jolt_cb_cos_max_slope_angle(&angles, batch, &jolt_cos);
        for (angles, jolt_cos) |angle, expected| {
            var character = makeCharacter(allocator, &system, angle, Vec3.axisY());
            defer character.base.deinit();
            checker.check(angle, character.base.getCosMaxSlopeAngle(), expected);
            character.base.setMaxSlopeAngle(0);
            character.base.setMaxSlopeAngle(angle);
            checker.check(angle, character.base.getCosMaxSlopeAngle(), expected);
        }
    }
    try checker.finish();
}

test "CharacterBase IsSlopeTooSteep" {
    const allocator = std.testing.allocator;
    var system: PhysicsSystem = .empty;
    var rng: fw.Rng = .{};
    var checker: Checker = .{ .name = "CharacterBase IsSlopeTooSteep" };
    const batch = 64;
    var normals: [batch][3]f32 = undefined;
    var jolt_results: [batch]u32 = undefined;
    // Angles whose cos is around the no-max-slope threshold 0.9999 (Jolt: the slope is never too steep at or above it)
    const threshold_angle: f32 = @floatCast(std.math.acos(@as(f64, CharacterBase.no_max_slope_angle)));
    for (0..fw.iterations / batch) |iteration| {
        const up = if (iteration % 4 == 0) [3]f32{ 0, 1, 0 } else randomUnit(&rng);
        const angle = switch (iteration % 5) {
            0 => threshold_angle + rng.float(-1.0e-4, 1.0e-4),
            1 => std.math.nextAfter(f32, threshold_angle, if (rng.next() % 2 == 0) 0 else 1),
            else => rng.float(0, std.math.pi),
        };
        var character = makeCharacter(allocator, &system, angle, Vec3.init(up[0], up[1], up[2]));
        defer character.base.deinit();
        const cos = character.base.getCosMaxSlopeAngle();
        for (&normals, 0..) |*n, i| {
            n.* = switch (i % 4) {
                // A normal whose dot with the up vector is about the cos of the max slope angle
                0 => blk: {
                    const t = randomUnit(&rng);
                    const sin_sq = 1 - cos * cos;
                    const sin = if (sin_sq > 0) @sqrt(sin_sq) else 0;
                    break :blk .{ up[0] * cos + t[0] * sin, up[1] * cos + t[1] * sin, up[2] * cos + t[2] * sin };
                },
                1 => up,
                2 => rng.floatArray(3, -2, 2), // Not normalized
                else => randomUnit(&rng),
            };
        }
        jolt.jolt_cb_is_slope_too_steep(&up, angle, &normals, batch, &jolt_results);
        for (normals, jolt_results) |n, expected|
            checker.check(.{ up, angle, n }, @as(u32, @intFromBool(character.base.isSlopeTooSteep(Vec3.init(n[0], n[1], n[2])))), expected);
    }
    try checker.finish();
}

test "CharacterBase SaveState bytes and cross RestoreState" {
    const allocator = std.testing.allocator;
    var system: PhysicsSystem = .empty;
    var rng: fw.Rng = .{};
    var bytes_checker: Checker = .{ .name = "CharacterBase SaveState" };
    var zolt_restore_checker: Checker = .{ .name = "CharacterBase RestoreState (zolt restores the jolt bytes)" };
    var jolt_restore_checker: Checker = .{ .name = "CharacterBase RestoreState (jolt restores the zolt bytes)" };
    var jolt_bytes: [128]u8 = undefined;
    for (0..fw.iterations / 10) |_| {
        const ground: CBGround = .{
            .state = rng.intRange(u32, 0, 3),
            // A BodyID value: bit 31 (BodyID::cBroadPhaseBit) only in the invalid ID (BodyID(uint32) asserts it)
            .body_id = if (rng.intRange(u32, 0, 7) == 0) 0xffffffff else rng.next() & 0x7fffffff,
            .sub_shape_id = rng.next(),
            .position = .{ rng.float(-1.0e4, 1.0e4), rng.float(-1.0e4, 1.0e4), rng.float(-1.0e4, 1.0e4) },
            .normal = randomUnit(&rng),
            .velocity = rng.floatArray(3, -100, 100),
        };
        var character = makeCharacter(allocator, &system, 0.5, Vec3.axisY());
        defer character.base.deinit();
        setGround(&character.base, &ground);
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        character.base.saveState(recorder.recorder());
        try std.testing.expect(!recorder.isFailed());
        const zolt_bytes = recorder.getData();
        const size = jolt.jolt_cb_save_state(&ground, &jolt_bytes, jolt_bytes.len);
        try std.testing.expect(size <= jolt_bytes.len);
        bytes_checker.check(ground.state, zolt_bytes.len, size);
        if (zolt_bytes.len == size) {
            for (zolt_bytes, jolt_bytes[0..size]) |z, j| bytes_checker.check(ground.state, z, j);
        }

        // Zolt restores the C++ bytes into another character
        var other = makeCharacter(allocator, &system, 0.5, Vec3.axisY());
        defer other.base.deinit();
        other.base.ground_user_data = 0x1234;
        var restore = StateRecorderImpl.init(allocator);
        defer restore.deinit();
        restore.writeBytes(jolt_bytes[0..size]);
        restore.rewind();
        other.base.restoreState(restore.recorder());
        try std.testing.expect(!restore.isFailed());
        zolt_restore_checker.check(ground.state, getGround(&other.base), ground);
        zolt_restore_checker.check(ground.state, @as(u32, @intFromBool(other.base.getGroundUserData() == 0 and other.base.getGroundMaterial() == PhysicsMaterial.default)), 1);

        // C++ restores the Zolt bytes
        var jolt_ground: CBGround = undefined;
        var reset: u32 = undefined;
        jolt_restore_checker.check(ground.state, jolt.jolt_cb_restore_state(zolt_bytes.ptr, @intCast(zolt_bytes.len), &jolt_ground, &reset), 1);
        jolt_restore_checker.check(ground.state, jolt_ground, ground);
        jolt_restore_checker.check(ground.state, reset, 1);
    }
    try finishAll(&.{ &bytes_checker, &zolt_restore_checker, &jolt_restore_checker });
}

test "CharacterBase sToString" {
    var text: [32]u8 = undefined;
    for (std.enums.values(CharacterBase.GroundState)) |state| {
        const length = jolt.jolt_cb_to_string(@bitCast(@intFromEnum(state)), &text, text.len);
        try std.testing.expectEqualStrings(text[0..length], CharacterBase.toString(state));
    }
}

test "CharacterID sNextCharacterID / sSetNextCharacterID / GetHash" {
    // The sequences after sSetNextCharacterID (the wrap-around skips cInvalidCharacterID), both counters are set back
    // to 1 afterwards (the simulation parity gives its character virtuals explicit IDs)
    var jolt_ids: [6]u32 = undefined;
    var checker: Checker = .{ .name = "CharacterID sequence" };
    for ([_]u32{ 1, 0, 1000, 0xfffffffc, 0xfffffffe, 0xffffffff }) |next_value| {
        jolt.jolt_cid_sequence(next_value, jolt_ids.len, &jolt_ids);
        CharacterID.setNextCharacterID(.{ .next_value = next_value });
        var zolt_ids: [6]u32 = undefined;
        for (&zolt_ids) |*id| id.* = CharacterID.nextCharacterID().getValue();
        CharacterID.setNextCharacterID(.{});
        checker.check(next_value, zolt_ids, jolt_ids);
    }
    try checker.finish();

    var rng: fw.Rng = .{};
    var hash_checker: Checker = .{ .name = "CharacterID GetHash" };
    var ids: [256]u32 = undefined;
    var jolt_hashes: [256]u64 = undefined;
    for (0..fw.iterations / ids.len) |iteration| {
        for (&ids, 0..) |*id, i| id.* = if (iteration == 0 and i < 4) ([_]u32{ 0, 1, 0xfffffffe, 0xffffffff })[i] else rng.next();
        jolt.jolt_cid_hash(&ids, ids.len, &jolt_hashes);
        for (ids, jolt_hashes) |id, expected| hash_checker.check(id, CharacterID.init(id).getHash(), expected);
    }
    try hash_checker.finish();
}
