//! Parity of Zolt/Compute/CPU/HLSLToCPP.zig with Jolt's HLSLToCPP.h (Docs/Zolt/RemainingFeaturesArchitecture.md R11):
//! every operator and function, called on both sides on random operands with special values. The C++ side calls them in
//! the namespace context of a shader wrapper (ShaderHelpersReference.cpp), so the overloads that a shader gets
//! (`std::min` for scalars, HLSLToCPP's `min` for vectors) are the ones compared. Floats compare with `sameBits` (NaNs
//! are equal), integers exactly.
//!
//! The operations are numbered in three families. The C++ side exports the name of every operation, and the first test
//! checks that the enums below match, so a reordered or missing operation fails.
//! Undefined behavior in C++ is never generated: no integer division by zero, no signed overflow (the signed operands
//! of the arithmetic are small), no negation of the smallest int.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const specialFloat = @import("ComputeParity.zig").specialFloat;
const inputFloat = @import("ComputeParity.zig").inputFloat;

const hlsl = zolt.HLSLToCPP;

/// The operands of one operation. Must match HLSLOpInput in ShaderHelpersReference.cpp.
const HLSLOpInput = extern struct {
    f: [3][4]f32,
    i: [3][4]i32,
    u: [3][4]u32,
};

/// The results of one operation, zero before the call. Must match HLSLOpOutput in ShaderHelpersReference.cpp.
const HLSLOpOutput = extern struct {
    f: [4]f32 = @splat(0),
    i: [4]i32 = @splat(0),
    u: [4]u32 = @splat(0),
};

comptime {
    std.debug.assert(@sizeOf(HLSLOpInput) == 144 and @sizeOf(HLSLOpOutput) == 48);
}

/// Family 0: the operators and functions of float2, float3 and float4
const FloatVectorOp = enum(u32) { add, sub, mul, div, mul_scalar, scalar_mul, div_scalar, negate, add_assign, sub_assign, mul_assign, div_assign, mul_assign_scalar, div_assign_scalar, eq, ne, dot, min, max, length, normalize, round, clamp, splat, get, set };

/// Family 1: the operators and functions of uint3, uint4, int3 and int4
const IntVectorOp = enum(u32) { add, sub, mul, div, mul_scalar, scalar_mul, div_scalar, negate, add_assign, sub_assign, mul_assign, div_assign, mul_assign_scalar, div_assign_scalar, eq, ne, dot, min, max, clamp, splat, get, set };

/// Family 2: swizzles, constructors, conversions and the scalar functions
const MiscOp = enum(u32) {
    f2_swizzle_xy,
    f2_swizzle_yx,
    f3_swizzle_xy,
    f3_swizzle_yx,
    f3_swizzle_xyz,
    f3_swizzle_xzy,
    f3_swizzle_yxz,
    f3_swizzle_yzx,
    f3_swizzle_zxy,
    f3_swizzle_zyx,
    f4_swizzle_xy,
    f4_swizzle_yx,
    f4_swizzle_xyz,
    f4_swizzle_xzy,
    f4_swizzle_yxz,
    f4_swizzle_yzx,
    f4_swizzle_zxy,
    f4_swizzle_zyx,
    f4_swizzle_xywz,
    f4_swizzle_xwyz,
    f4_swizzle_wxyz,
    u3_swizzle_xyz,
    u3_swizzle_xzy,
    u3_swizzle_yxz,
    u3_swizzle_yzx,
    u3_swizzle_zxy,
    u3_swizzle_zyx,
    u4_swizzle_xyz,
    u4_swizzle_xzy,
    u4_swizzle_yxz,
    u4_swizzle_yzx,
    u4_swizzle_zxy,
    u4_swizzle_zyx,
    u4_swizzle_xywz,
    u4_swizzle_xwyz,
    u4_swizzle_wxyz,
    i3_swizzle_xyz,
    i3_swizzle_xzy,
    i3_swizzle_yxz,
    i3_swizzle_yzx,
    i3_swizzle_zxy,
    i3_swizzle_zyx,
    i4_swizzle_xyz,
    i4_swizzle_xzy,
    i4_swizzle_yxz,
    i4_swizzle_yzx,
    i4_swizzle_zxy,
    i4_swizzle_zyx,
    i4_swizzle_xywz,
    i4_swizzle_xwyz,
    i4_swizzle_wxyz,
    f3_from_float2,
    f3_from_uint3,
    f4_from_float3,
    f4_from_int4,
    f4_asint,
    f3_cross,
    u3_from_float3,
    u4_from_uint3,
    i3_from_float3,
    i4_from_int3,
    i4_from_float4,
    mat44_column,
    float_min,
    float_max,
    float_clamp,
    float_sqrt,
    float_round,
    uint_min,
    uint_max,
    uint_clamp,
    int_min,
    int_max,
    int_clamp,
    to_int,
    to_uint,
    float_from_uint,
    float_from_int,
    atomic_add_int,
    atomic_add_uint,
};

extern fn jolt_hlsl_op_name(family: u32, op: u32) ?[*:0]const u8;
extern fn jolt_hlsl_float_vector_op(dim: u32, op: u32, in: [*]const HLSLOpInput, out: [*]HLSLOpOutput, count: u32) void;
extern fn jolt_hlsl_int_vector_op(type_index: u32, op: u32, in: [*]const HLSLOpInput, out: [*]HLSLOpOutput, count: u32) void;
extern fn jolt_hlsl_misc_op(op: u32, in: [*]const HLSLOpInput, out: [*]HLSLOpOutput, count: u32) void;

/// Inputs per operation and vector type
const count = 20_000;

fn dim(comptime T: type) u32 {
    return @typeInfo(T).@"struct".fields.len;
}

fn load(comptime T: type, values: anytype) T {
    var v: T = undefined;
    inline for (@typeInfo(T).@"struct".fields, 0..) |field, k|
        @field(v, field.name) = values[k];
    return v;
}

fn store(v: anytype, out: anytype) void {
    inline for (@typeInfo(@TypeOf(v)).@"struct".fields, 0..) |field, k|
        out[k] = @field(v, field.name);
}

fn floatVectorOp(comptime T: type, op: FloatVectorOp, in: HLSLOpInput) HLSLOpOutput {
    const a = load(T, in.f[0]);
    const b = load(T, in.f[1]);
    const c = load(T, in.f[2]);
    const s = in.f[1][0];
    const index = in.u[0][0] % dim(T);
    var out: HLSLOpOutput = .{};
    switch (op) {
        .add, .add_assign => store(a.add(b), &out.f),
        .sub, .sub_assign => store(a.sub(b), &out.f),
        .mul, .mul_assign => store(a.mul(b), &out.f),
        .div, .div_assign => store(a.div(b), &out.f),
        .mul_scalar, .scalar_mul, .mul_assign_scalar => store(a.mulScalar(s), &out.f),
        .div_scalar, .div_assign_scalar => store(a.divScalar(s), &out.f),
        .negate => store(a.negate(), &out.f),
        .eq => out.u[0] = @intFromBool(a.eql(b)),
        .ne => out.u[0] = @intFromBool(!a.eql(b)),
        .dot => out.f[0] = hlsl.dot(a, b),
        .min => store(hlsl.min(a, b), &out.f),
        .max => store(hlsl.max(a, b), &out.f),
        .length => out.f[0] = hlsl.length(a),
        .normalize => store(hlsl.normalize(a), &out.f),
        .round => store(hlsl.round(a), &out.f),
        .clamp => store(hlsl.clamp(a, b, c), &out.f),
        .splat => store(T.splat(s), &out.f),
        .get => out.f[0] = a.getComponent(index),
        .set => {
            var r = a;
            r.setComponent(index, s);
            store(r, &out.f);
        },
    }
    return out;
}

fn intVectorOp(comptime T: type, op: IntVectorOp, in: HLSLOpInput) HLSLOpOutput {
    const S = hlsl.Scalar(T);
    const src = if (S == u32) in.u else in.i;
    const a = load(T, src[0]);
    const b = load(T, src[1]);
    const c = load(T, src[2]);
    const s = src[1][0];
    const index = in.u[2][3] % dim(T);
    var out: HLSLOpOutput = .{};
    const dst = if (S == u32) &out.u else &out.i;
    switch (op) {
        .add, .add_assign => store(a.add(b), dst),
        .sub, .sub_assign => store(a.sub(b), dst),
        .mul, .mul_assign => store(a.mul(b), dst),
        .div, .div_assign => store(a.div(b), dst),
        .mul_scalar, .scalar_mul, .mul_assign_scalar => store(a.mulScalar(s), dst),
        .div_scalar, .div_assign_scalar => store(a.divScalar(s), dst),
        .negate => if (T == hlsl.int3 or T == hlsl.int4) store(a.negate(), dst), // uint3 / uint4 have no unary minus
        .eq => out.u[3] = @intFromBool(a.eql(b)),
        .ne => out.u[3] = @intFromBool(!a.eql(b)),
        .dot => dst[0] = hlsl.dot(a, b),
        .min => store(hlsl.min(a, b), dst),
        .max => store(hlsl.max(a, b), dst),
        .clamp => store(hlsl.clamp(a, b, c), dst),
        .splat => if (T == hlsl.uint4 or T == hlsl.int4) store(T.splat(s), dst), // uint3 / int3 have no such constructor
        .get => dst[0] = a.getComponent(index),
        .set => {
            var r = a;
            r.setComponent(index, s);
            store(r, dst);
        },
    }
    return out;
}

fn miscOp(op: MiscOp, in: HLSLOpInput) HLSLOpOutput {
    const f2 = load(hlsl.float2, in.f[0]);
    const f3 = load(hlsl.float3, in.f[0]);
    const f4 = load(hlsl.float4, in.f[0]);
    const f3b = load(hlsl.float3, in.f[1]);
    const uint3_a = load(hlsl.uint3, in.u[0]);
    const uint4_a = load(hlsl.uint4, in.u[0]);
    const int3_a = load(hlsl.int3, in.i[0]);
    const int4_a = load(hlsl.int4, in.i[0]);
    const fa = in.f[0][0];
    const fb = in.f[1][0];
    const fc = in.f[2][0];
    const ua = in.u[0][0];
    const ub = in.u[1][0];
    const uc = in.u[2][0];
    const ia = in.i[0][0];
    const ib = in.i[1][0];
    const ic = in.i[2][0];
    var out: HLSLOpOutput = .{};
    switch (op) {
        .f2_swizzle_xy => store(f2.xy(), &out.f),
        .f2_swizzle_yx => store(f2.yx(), &out.f),

        .f3_swizzle_xy => store(f3.xy(), &out.f),
        .f3_swizzle_yx => store(f3.yx(), &out.f),
        .f3_swizzle_xyz => store(f3.xyz(), &out.f),
        .f3_swizzle_xzy => store(f3.xzy(), &out.f),
        .f3_swizzle_yxz => store(f3.yxz(), &out.f),
        .f3_swizzle_yzx => store(f3.yzx(), &out.f),
        .f3_swizzle_zxy => store(f3.zxy(), &out.f),
        .f3_swizzle_zyx => store(f3.zyx(), &out.f),

        .f4_swizzle_xy => store(f4.xy(), &out.f),
        .f4_swizzle_yx => store(f4.yx(), &out.f),
        .f4_swizzle_xyz => store(f4.xyz(), &out.f),
        .f4_swizzle_xzy => store(f4.xzy(), &out.f),
        .f4_swizzle_yxz => store(f4.yxz(), &out.f),
        .f4_swizzle_yzx => store(f4.yzx(), &out.f),
        .f4_swizzle_zxy => store(f4.zxy(), &out.f),
        .f4_swizzle_zyx => store(f4.zyx(), &out.f),
        .f4_swizzle_xywz => store(f4.xywz(), &out.f),
        .f4_swizzle_xwyz => store(f4.xwyz(), &out.f),
        .f4_swizzle_wxyz => store(f4.wxyz(), &out.f),

        .u3_swizzle_xyz => store(uint3_a.xyz(), &out.u),
        .u3_swizzle_xzy => store(uint3_a.xzy(), &out.u),
        .u3_swizzle_yxz => store(uint3_a.yxz(), &out.u),
        .u3_swizzle_yzx => store(uint3_a.yzx(), &out.u),
        .u3_swizzle_zxy => store(uint3_a.zxy(), &out.u),
        .u3_swizzle_zyx => store(uint3_a.zyx(), &out.u),

        .u4_swizzle_xyz => store(uint4_a.xyz(), &out.u),
        .u4_swizzle_xzy => store(uint4_a.xzy(), &out.u),
        .u4_swizzle_yxz => store(uint4_a.yxz(), &out.u),
        .u4_swizzle_yzx => store(uint4_a.yzx(), &out.u),
        .u4_swizzle_zxy => store(uint4_a.zxy(), &out.u),
        .u4_swizzle_zyx => store(uint4_a.zyx(), &out.u),
        .u4_swizzle_xywz => store(uint4_a.xywz(), &out.u),
        .u4_swizzle_xwyz => store(uint4_a.xwyz(), &out.u),
        .u4_swizzle_wxyz => store(uint4_a.wxyz(), &out.u),

        .i3_swizzle_xyz => store(int3_a.xyz(), &out.i),
        .i3_swizzle_xzy => store(int3_a.xzy(), &out.i),
        .i3_swizzle_yxz => store(int3_a.yxz(), &out.i),
        .i3_swizzle_yzx => store(int3_a.yzx(), &out.i),
        .i3_swizzle_zxy => store(int3_a.zxy(), &out.i),
        .i3_swizzle_zyx => store(int3_a.zyx(), &out.i),

        .i4_swizzle_xyz => store(int4_a.xyz(), &out.i),
        .i4_swizzle_xzy => store(int4_a.xzy(), &out.i),
        .i4_swizzle_yxz => store(int4_a.yxz(), &out.i),
        .i4_swizzle_yzx => store(int4_a.yzx(), &out.i),
        .i4_swizzle_zxy => store(int4_a.zxy(), &out.i),
        .i4_swizzle_zyx => store(int4_a.zyx(), &out.i),
        .i4_swizzle_xywz => store(int4_a.xywz(), &out.i),
        .i4_swizzle_xwyz => store(int4_a.xwyz(), &out.i),
        .i4_swizzle_wxyz => store(int4_a.wxyz(), &out.i),

        .f3_from_float2 => store(hlsl.float3.fromFloat2(f2, fb), &out.f),
        .f3_from_uint3 => store(hlsl.float3.fromUint3(uint3_a), &out.f),
        .f4_from_float3 => store(hlsl.float4.fromFloat3(f3, fb), &out.f),
        .f4_from_int4 => store(hlsl.float4.fromInt4(int4_a), &out.f),
        .f4_asint => store(hlsl.asint(f4), &out.i),
        .f3_cross => store(hlsl.cross(f3, f3b), &out.f),
        .u3_from_float3 => store(hlsl.uint3.fromFloat3(f3), &out.u),
        .u4_from_uint3 => store(hlsl.uint4.fromUint3(uint3_a, ub), &out.u),
        .i3_from_float3 => store(hlsl.int3.fromFloat3(f3), &out.i),
        .i4_from_int3 => store(hlsl.int4.fromInt3(int3_a, ib), &out.i),
        .i4_from_float4 => store(hlsl.int4.fromFloat4(f4), &out.i),
        .mat44_column => {
            const m = hlsl.Mat44.init(f4, load(hlsl.float4, in.f[1]), load(hlsl.float4, in.f[2]), .init(fa, fb, fc, 1.0));
            store(m.c[ua % 4], &out.f);
        },

        .float_min => out.f[0] = hlsl.min(fa, fb),
        .float_max => out.f[0] = hlsl.max(fa, fb),
        .float_clamp => out.f[0] = hlsl.clamp(fa, fb, fc),
        .float_sqrt => out.f[0] = hlsl.sqrt(fa),
        .float_round => out.f[0] = hlsl.round(fa),
        .uint_min => out.u[0] = hlsl.min(ua, ub),
        .uint_max => out.u[0] = hlsl.max(ua, ub),
        .uint_clamp => out.u[0] = hlsl.clamp(ua, ub, uc),
        .int_min => out.i[0] = hlsl.min(ia, ib),
        .int_max => out.i[0] = hlsl.max(ia, ib),
        .int_clamp => out.i[0] = hlsl.clamp(ia, ib, ic),

        .to_int => out.i[0] = hlsl.toInt(fa),
        .to_uint => out.u[0] = hlsl.toUint(fa),
        .float_from_uint => out.f[0] = @floatFromInt(ua),
        .float_from_int => out.f[0] = @floatFromInt(ia),
        .atomic_add_int => {
            var value = ia;
            out.i[0] = hlsl.atomicAdd(&value, ib);
            out.i[1] = value;
        },
        .atomic_add_uint => {
            var value = ua;
            out.u[0] = hlsl.atomicAdd(&value, ub);
            out.u[1] = value;
        },
    }
    return out;
}

/// A float for the float to int conversions: the edges of the int, uint and int64 ranges, otherwise specialFloat
fn conversionFloat(rng: *fw.Rng) f32 {
    const edges = [_]f32{ -0x1p31, 0x1p31, -0x1.000002p31, 0x1.fffffep30, 0x1p32, 0x1.fffffep31, 0x1p63, -0x1p63, 0x1.fffffep62, -0x1.000002p63, -1.0, -0.5, 0.99999994, -0.99999994, 4294967040.0 };
    return if (rng.next() % 3 == 0) edges[rng.next() % edges.len] * (if (rng.next() % 2 == 0) @as(f32, 1) else -1) else specialFloat(rng, 1.0e4);
}

/// A signed operand without overflow in the arithmetic: |x| <= 2^14
fn smallInt(rng: *fw.Rng) i32 {
    return rng.intRange(i32, -(1 << 14), 1 << 14);
}

/// Random operands. `equal`: the second operand equals the first (for ==, != and the min / max ties).
fn randomInput(rng: *fw.Rng, small_ints: bool, conversions: bool) HLSLOpInput {
    var in: HLSLOpInput = undefined;
    for (&in.f) |*v| {
        for (v) |*x| x.* = if (conversions) conversionFloat(rng) else inputFloat(rng, 100);
    }
    for (&in.i) |*v| {
        for (v) |*x| x.* = if (small_ints) smallInt(rng) else @bitCast(rng.next());
    }
    for (&in.u) |*v| {
        for (v) |*x| x.* = switch (rng.next() % 8) {
            0 => 0,
            1 => 0xffffffff,
            2 => rng.next() % 16,
            else => rng.next(),
        };
    }
    switch (rng.next() % 8) {
        // Equal operands (ties of min / max, == and !=), sometimes equal but for one component
        0 => {
            in.f[1] = in.f[0];
            in.i[1] = in.i[0];
            in.u[1] = in.u[0];
        },
        1 => {
            in.f[1] = in.f[0];
            in.f[1][rng.next() % 4] = specialFloat(rng, 100);
            in.i[1] = in.i[0];
            in.i[1][rng.next() % 4] +%= 1;
            in.u[1] = in.u[0];
            in.u[1][rng.next() % 4] +%= 1;
        },
        else => {},
    }
    return in;
}

/// Compare the results of one operation on both sides (the number of mismatches of every failing operation is printed)
fn compareOp(checker: *fw.Checker, type_name: []const u8, op: anytype, inputs: []const HLSLOpInput, zolt_out: []const HLSLOpOutput, jolt_out: []const HLSLOpOutput) void {
    const before = checker.mismatches;
    for (inputs, zolt_out, jolt_out) |in, z, j|
        checker.check(.{ .op = op, .input = in }, z, j);
    if (checker.mismatches != before)
        std.debug.print("{s}: {s} {t}: {d} mismatches\n", .{ checker.name, type_name, op, checker.mismatches - before });
}

test "HLSLToCPPParity: the operation tables match" {
    inline for (.{ FloatVectorOp, IntVectorOp, MiscOp }, 0..) |Op, family| {
        const fields = @typeInfo(Op).@"enum".fields;
        inline for (fields) |field| {
            const name = jolt_hlsl_op_name(family, field.value) orelse return error.MissingOperation;
            try std.testing.expectEqualStrings(field.name, std.mem.span(name));
        }
        try std.testing.expect(jolt_hlsl_op_name(family, fields.len) == null);
    }
}

test "HLSLToCPPParity: float2, float3 and float4 operators and functions" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "HLSLToCPP float vectors" };
    var rng: fw.Rng = .{ .state = 0x6789abc };

    const inputs = try allocator.alloc(HLSLOpInput, count);
    defer allocator.free(inputs);
    const zolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(zolt_out);
    const jolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(jolt_out);

    inline for (.{ hlsl.float2, hlsl.float3, hlsl.float4 }) |T| {
        for (0..@typeInfo(FloatVectorOp).@"enum".fields.len) |op_index| {
            const op: FloatVectorOp = @enumFromInt(op_index);
            for (inputs, zolt_out) |*in, *z| {
                in.* = randomInput(&rng, false, false);
                z.* = floatVectorOp(T, op, in.*);
            }
            @memset(jolt_out, .{});
            jolt_hlsl_float_vector_op(dim(T), @intFromEnum(op), inputs.ptr, jolt_out.ptr, count);
            compareOp(&checker, @typeName(T), op, inputs, zolt_out, jolt_out);
        }
    }
    try checker.finish();
}

test "HLSLToCPPParity: uint3, uint4, int3 and int4 operators and functions" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "HLSLToCPP int vectors" };
    var rng: fw.Rng = .{ .state = 0x789abcd };

    const inputs = try allocator.alloc(HLSLOpInput, count);
    defer allocator.free(inputs);
    const zolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(zolt_out);
    const jolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(jolt_out);

    inline for (.{ hlsl.uint3, hlsl.uint4, hlsl.int3, hlsl.int4 }, 0..) |T, type_index| {
        for (0..@typeInfo(IntVectorOp).@"enum".fields.len) |op_index| {
            const op: IntVectorOp = @enumFromInt(op_index);
            const signed = hlsl.Scalar(T) == i32;
            // Signed arithmetic gets small operands (no overflow); min, max, clamp, ==, get and set any value
            const small = signed and switch (op) {
                .min, .max, .clamp, .eq, .ne, .get, .set, .splat => false,
                else => true,
            };
            for (inputs, zolt_out) |*in, *z| {
                in.* = randomInput(&rng, small, false);
                switch (op) {
                    // No division by zero
                    .div, .div_assign, .div_scalar, .div_assign_scalar => {
                        for (&in.u[1]) |*x| x.* |= @as(u32, 1) << @intCast(rng.next() % 32);
                        for (&in.i[1]) |*x| x.* = if (x.* == 0) 1 else x.*;
                    },
                    // No negation of the smallest int
                    .negate => for (&in.i[0]) |*x| {
                        if (x.* == std.math.minInt(i32)) x.* = 0;
                    },
                    else => {},
                }
                z.* = intVectorOp(T, op, in.*);
            }
            @memset(jolt_out, .{});
            jolt_hlsl_int_vector_op(type_index, @intFromEnum(op), inputs.ptr, jolt_out.ptr, count);
            compareOp(&checker, @typeName(T), op, inputs, zolt_out, jolt_out);
        }
    }
    try checker.finish();
}

test "HLSLToCPPParity: swizzles, constructors, conversions and scalar functions" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "HLSLToCPP misc" };
    var rng: fw.Rng = .{ .state = 0x89abcde };

    const inputs = try allocator.alloc(HLSLOpInput, count);
    defer allocator.free(inputs);
    const zolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(zolt_out);
    const jolt_out = try allocator.alloc(HLSLOpOutput, count);
    defer allocator.free(jolt_out);

    for (0..@typeInfo(MiscOp).@"enum".fields.len) |op_index| {
        const op: MiscOp = @enumFromInt(op_index);
        const conversions = switch (op) {
            .to_int, .to_uint, .u3_from_float3, .i3_from_float3, .i4_from_float4 => true,
            else => false,
        };
        for (inputs, zolt_out) |*in, *z| {
            // atomic_add_int: `fetch_add(v) + v` overflows an int for large operands (undefined in C++)
            in.* = randomInput(&rng, op == .atomic_add_int, conversions);
            z.* = miscOp(op, in.*);
        }
        @memset(jolt_out, .{});
        jolt_hlsl_misc_op(@intFromEnum(op), inputs.ptr, jolt_out.ptr, count);
        compareOp(&checker, "", op, inputs, zolt_out, jolt_out);
    }
    try checker.finish();
}
