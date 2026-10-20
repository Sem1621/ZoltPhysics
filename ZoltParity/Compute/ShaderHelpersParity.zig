//! Parity of the shader helper files (Zolt/Shaders/ShaderMath.zig, ShaderMat44.zig, ShaderQuat.zig, ShaderPlane.zig,
//! ShaderVec3.zig, HairCommon.zig) with Jolt's HLSL helpers compiled in HLSLToCPP mode, as members of a shader wrapper
//! class like every hair shader (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, R12). C++ side:
//! ShaderHelpersReference.cpp. Every helper runs on ~100k random inputs with special values; floats compare with
//! `sameBits` (NaNs are equal). The grid helper never gets a grid position whose `uint3(grid_pos)` is undefined in C++
//! (R20, see `gridPositionDefined`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ComputeParity = @import("ComputeParity.zig");
const specialFloat = ComputeParity.specialFloat;
const inputFloat = ComputeParity.inputFloat;

const hlsl = zolt.HLSLToCPP;
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const uint3 = hlsl.uint3;
const Mat44 = hlsl.Mat44;
const ShaderMath = zolt.ShaderMath;
const ShaderMat44 = zolt.ShaderMat44;
const ShaderQuat = zolt.ShaderQuat;
const ShaderPlane = zolt.ShaderPlane;
const ShaderVec3 = zolt.ShaderVec3;
const HairCommon = zolt.HairCommon;
const HairUpdateContext = zolt.HairStructs.HairUpdateContext;

extern fn jolt_shader_square(values: [*]const f32, out: [*]f32, count: u32) void;
extern fn jolt_shader_get_closest_point_on_line(a: [*]const float3, ab: [*]const float3, out: [*]float3, count: u32) void;
extern fn jolt_shader_mat44_mul3x4_vec3(m: [*]const Mat44, v: [*]const float3, out: [*]float3, count: u32) void;
extern fn jolt_shader_mat44_mul3x3_vec3(m: [*]const Mat44, v: [*]const float3, out: [*]float3, count: u32) void;
extern fn jolt_shader_quat_mul_vec3(q: [*]const float4, v: [*]const float3, out: [*]float3, count: u32) void;
extern fn jolt_shader_quat_mul_quat(a: [*]const float4, b: [*]const float4, out: [*]float4, count: u32) void;
extern fn jolt_shader_quat_imaginary_mul_quat(a: [*]const float3, b: [*]const float4, out: [*]float4, count: u32) void;
extern fn jolt_shader_quat_rotate_axis_z(q: [*]const float4, out: [*]float3, count: u32) void;
extern fn jolt_shader_quat_conjugate(q: [*]const float4, out: [*]float4, count: u32) void;
extern fn jolt_shader_quat_decompress(values: [*]const u32, out: [*]float4, count: u32) void;
extern fn jolt_shader_quat_from_mat33(col0: [*]const float3, col1: [*]const float3, col2: [*]const float3, out: [*]float4, count: u32) void;
extern fn jolt_shader_plane_from_point_and_normal(point: [*]const float3, normal: [*]const float3, out: [*]float4, count: u32) void;
extern fn jolt_shader_plane_get_normal(plane: [*]const float4, out: [*]float3, count: u32) void;
extern fn jolt_shader_plane_signed_distance(plane: [*]const float4, point: [*]const float3, out: [*]f32, count: u32) void;
extern fn jolt_shader_vec3_decompress_unit(values: [*]const u32, out: [*]float3, count: u32) void;
extern fn jolt_shader_hair_packed(words: [*]const u32, indices: [*]const u32, out_fixed: [*]u32, out_inv_mass: [*]f32, out_fraction: [*]f32, out_vertex_count: [*]u32, out_material_index: [*]u32, count: u32) void;
extern fn jolt_shader_gradient_sampler_sample(sampler: [*]const float4, fraction: [*]const f32, out: [*]f32, count: u32) void;
extern fn jolt_shader_grid(context: *const HairUpdateContext, positions: [*]const float3, indices: [*]const uint3, out_index: [*]uint3, out_fraction: [*]float3, out_buffer_index: [*]u32, out_buffer_index2: [*]u32, count: u32) void;
extern fn jolt_shader_hair_common_constants(out_float_to_fixed: *c_int, out_fixed_to_float: *f32) void;

const n = fw.iterations;

fn randomFloat3(rng: *fw.Rng, range: f32) float3 {
    return .init(inputFloat(rng, range), inputFloat(rng, range), inputFloat(rng, range));
}

/// True when `uint3(grid_pos)` of GridPositionToIndexAndFraction is defined in C++: every component of the grid
/// position is in (-1, 2^32), never NaN. For other values Jolt's result depends on how clang compiles the call site
/// (R20, hlsl.uint3.fromFloat3).
fn gridPositionDefined(context: *const HairUpdateContext, position: float3) bool {
    const grid_pos = hlsl.min(hlsl.max(position.sub(context.grid_offset), float3.init(0, 0, 0)).mul(context.grid_scale), context.grid_size_min1);
    for ([_]f32{ grid_pos.x, grid_pos.y, grid_pos.z }) |c| {
        if (!(c > -1.0 and c < 0x1p32)) return false; // False for NaN
    }
    return true;
}

fn randomFloat4(rng: *fw.Rng, range: f32) float4 {
    return .init(inputFloat(rng, range), inputFloat(rng, range), inputFloat(rng, range), inputFloat(rng, range));
}

/// A unit quaternion most of the time (the helpers are called with rotations), otherwise any float4
fn randomQuat(rng: *fw.Rng) float4 {
    if (rng.next() % 4 == 0) return randomFloat4(rng, 10);
    const q = zolt.Vec4.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
    const u = if (q.lengthSq() > 1.0e-6) q.normalized() else zolt.Vec4.init(0, 0, 0, 1);
    return .init(u.getX(), u.getY(), u.getZ(), u.getW());
}

/// Compare the float components of two arrays of extern structs of floats
fn checkFloats(checker: *fw.Checker, what: []const u8, comptime T: type, inputs: anytype, zolt_out: []const T, jolt_out: []const T) void {
    for (zolt_out, jolt_out, 0..) |z, j, i| {
        const zf = std.mem.bytesAsSlice(f32, std.mem.asBytes(&z));
        const jf = std.mem.bytesAsSlice(f32, std.mem.asBytes(&j));
        for (zf, jf) |a, b| {
            if (!fw.sameBits(a, b)) {
                checker.check(.{ .function = what, .input = inputs[i] }, z, j);
                break;
            }
        }
    }
}

test "ShaderHelpersParity: ShaderMath (JPH_Square, JPH_GetClosestPointOnLine)" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "ShaderMath" };
    var rng: fw.Rng = .{ .state = 0x1234567 };

    const values = try allocator.alloc(f32, n);
    defer allocator.free(values);
    const a = try allocator.alloc(float3, n);
    defer allocator.free(a);
    const ab = try allocator.alloc(float3, n);
    defer allocator.free(ab);
    for (values, a, ab, 0..) |*v, *pa, *pab, i| {
        v.* = specialFloat(&rng, 1.0e3);
        pa.* = randomFloat3(&rng, 10);
        // Degenerate segments: zero length and a = 0
        pab.* = if (i % 50 == 0) float3.splat(0) else randomFloat3(&rng, 10);
        if (i % 77 == 0) pa.* = float3.splat(0);
    }

    const zolt_square = try allocator.alloc(f32, n);
    defer allocator.free(zolt_square);
    const jolt_square = try allocator.alloc(f32, n);
    defer allocator.free(jolt_square);
    for (values, zolt_square) |v, *o| o.* = ShaderMath.square(v);
    jolt_shader_square(values.ptr, jolt_square.ptr, n);
    for (values, zolt_square, jolt_square) |v, z, j| checker.check(.{ .function = "square", .value = v }, z, j);

    const zolt_point = try allocator.alloc(float3, n);
    defer allocator.free(zolt_point);
    const jolt_point = try allocator.alloc(float3, n);
    defer allocator.free(jolt_point);
    for (a, ab, zolt_point) |pa, pab, *o| o.* = ShaderMath.getClosestPointOnLine(pa, pab);
    jolt_shader_get_closest_point_on_line(a.ptr, ab.ptr, jolt_point.ptr, n);
    checkFloats(&checker, "getClosestPointOnLine", float3, a, zolt_point, jolt_point);

    try checker.finish();
}

test "ShaderHelpersParity: ShaderMat44 (JPH_Mat44Mul3x4Vec3, JPH_Mat44Mul3x3Vec3)" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "ShaderMat44" };
    var rng: fw.Rng = .{ .state = 0x2345678 };

    const m = try allocator.alloc(Mat44, n);
    defer allocator.free(m);
    const v = try allocator.alloc(float3, n);
    defer allocator.free(v);
    for (m, v) |*pm, *pv| {
        pm.* = .init(randomFloat4(&rng, 10), randomFloat4(&rng, 10), randomFloat4(&rng, 10), randomFloat4(&rng, 100));
        pv.* = randomFloat3(&rng, 100);
    }

    const zolt_out = try allocator.alloc(float3, n);
    defer allocator.free(zolt_out);
    const jolt_out = try allocator.alloc(float3, n);
    defer allocator.free(jolt_out);

    for (m, v, zolt_out) |pm, pv, *o| o.* = ShaderMat44.mat44Mul3x4Vec3(pm, pv);
    jolt_shader_mat44_mul3x4_vec3(m.ptr, v.ptr, jolt_out.ptr, n);
    checkFloats(&checker, "mat44Mul3x4Vec3", float3, v, zolt_out, jolt_out);

    for (m, v, zolt_out) |pm, pv, *o| o.* = ShaderMat44.mat44Mul3x3Vec3(pm, pv);
    jolt_shader_mat44_mul3x3_vec3(m.ptr, v.ptr, jolt_out.ptr, n);
    checkFloats(&checker, "mat44Mul3x3Vec3", float3, v, zolt_out, jolt_out);

    try checker.finish();
}

test "ShaderHelpersParity: ShaderQuat (the 7 quaternion helpers)" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "ShaderQuat" };
    var rng: fw.Rng = .{ .state = 0x3456789 };

    const q = try allocator.alloc(float4, n);
    defer allocator.free(q);
    const q2 = try allocator.alloc(float4, n);
    defer allocator.free(q2);
    const v = try allocator.alloc(float3, n);
    defer allocator.free(v);
    const cols = try allocator.alloc([3]float3, n);
    defer allocator.free(cols);
    const c0 = try allocator.alloc(float3, n);
    defer allocator.free(c0);
    const c1 = try allocator.alloc(float3, n);
    defer allocator.free(c1);
    const c2 = try allocator.alloc(float3, n);
    defer allocator.free(c2);
    const compressed = try allocator.alloc(u32, n);
    defer allocator.free(compressed);
    for (q, q2, v, cols, c0, c1, c2, compressed, 0..) |*pq, *pq2, *pv, *pc, *p0, *p1, *p2, *pcomp, i| {
        pq.* = randomQuat(&rng);
        pq2.* = randomQuat(&rng);
        pv.* = randomFloat3(&rng, 100);

        // Rotation matrices (all four branches of JPH_QuatFromMat33), sometimes any 3x3 matrix
        if (i % 3 == 0) {
            pc.* = .{ randomFloat3(&rng, 2), randomFloat3(&rng, 2), randomFloat3(&rng, 2) };
        } else {
            const r = zolt.Mat44.rotationQuat(zolt.Quat.fromVec4(blk: {
                const u = randomQuat(&rng);
                const qv = zolt.Vec4.init(u.x, u.y, u.z, u.w);
                break :blk if (qv.lengthSq() > 1.0e-6 and qv.lengthSq() < 1.0e6) qv.normalized() else zolt.Vec4.init(0, 0, 0, 1);
            }));
            for (&pc.*, 0..) |*col, k| {
                const cv = r.getColumn3(@intCast(k));
                col.* = .init(cv.getX(), cv.getY(), cv.getZ());
            }
        }
        p0.* = pc[0];
        p1.* = pc[1];
        p2.* = pc[2];

        // Every bit pattern, and the compressed values of unit quaternions
        if (i % 2 == 0) {
            pcomp.* = rng.next();
        } else {
            const u = randomQuat(&rng);
            const qv = zolt.Vec4.init(u.x, u.y, u.z, u.w);
            pcomp.* = zolt.Quat.fromVec4(if (qv.lengthSq() > 1.0e-6 and qv.lengthSq() < 1.0e6) qv.normalized() else zolt.Vec4.init(0, 0, 0, 1)).compressUnitQuat();
        }
    }

    const out3_z = try allocator.alloc(float3, n);
    defer allocator.free(out3_z);
    const out3_j = try allocator.alloc(float3, n);
    defer allocator.free(out3_j);
    const out4_z = try allocator.alloc(float4, n);
    defer allocator.free(out4_z);
    const out4_j = try allocator.alloc(float4, n);
    defer allocator.free(out4_j);

    for (q, v, out3_z) |pq, pv, *o| o.* = ShaderQuat.quatMulVec3(pq, pv);
    jolt_shader_quat_mul_vec3(q.ptr, v.ptr, out3_j.ptr, n);
    checkFloats(&checker, "quatMulVec3", float3, q, out3_z, out3_j);

    for (q, q2, out4_z) |pq, pq2, *o| o.* = ShaderQuat.quatMulQuat(pq, pq2);
    jolt_shader_quat_mul_quat(q.ptr, q2.ptr, out4_j.ptr, n);
    checkFloats(&checker, "quatMulQuat", float4, q, out4_z, out4_j);

    for (v, q2, out4_z) |pv, pq2, *o| o.* = ShaderQuat.quatImaginaryMulQuat(pv, pq2);
    jolt_shader_quat_imaginary_mul_quat(v.ptr, q2.ptr, out4_j.ptr, n);
    checkFloats(&checker, "quatImaginaryMulQuat", float4, v, out4_z, out4_j);

    for (q, out3_z) |pq, *o| o.* = ShaderQuat.quatRotateAxisZ(pq);
    jolt_shader_quat_rotate_axis_z(q.ptr, out3_j.ptr, n);
    checkFloats(&checker, "quatRotateAxisZ", float3, q, out3_z, out3_j);

    for (q, out4_z) |pq, *o| o.* = ShaderQuat.quatConjugate(pq);
    jolt_shader_quat_conjugate(q.ptr, out4_j.ptr, n);
    checkFloats(&checker, "quatConjugate", float4, q, out4_z, out4_j);

    for (compressed, out4_z) |c, *o| o.* = ShaderQuat.quatDecompress(c);
    jolt_shader_quat_decompress(compressed.ptr, out4_j.ptr, n);
    checkFloats(&checker, "quatDecompress", float4, compressed, out4_z, out4_j);

    for (c0, c1, c2, out4_z) |p0, p1, p2, *o| o.* = ShaderQuat.quatFromMat33(p0, p1, p2);
    jolt_shader_quat_from_mat33(c0.ptr, c1.ptr, c2.ptr, out4_j.ptr, n);
    checkFloats(&checker, "quatFromMat33", float4, cols, out4_z, out4_j);

    try checker.finish();
}

test "ShaderHelpersParity: ShaderPlane and ShaderVec3" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "ShaderPlane / ShaderVec3" };
    var rng: fw.Rng = .{ .state = 0x456789a };

    const point = try allocator.alloc(float3, n);
    defer allocator.free(point);
    const normal = try allocator.alloc(float3, n);
    defer allocator.free(normal);
    const plane = try allocator.alloc(float4, n);
    defer allocator.free(plane);
    const compressed = try allocator.alloc(u32, n);
    defer allocator.free(compressed);
    for (point, normal, plane, compressed, 0..) |*pp, *pn, *ppl, *pc, i| {
        pp.* = randomFloat3(&rng, 100);
        pn.* = randomFloat3(&rng, 1);
        ppl.* = randomFloat4(&rng, 10);
        if (i % 2 == 0) {
            pc.* = rng.next();
        } else {
            const u = zolt.Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1)).normalizedOr(zolt.Vec3.axisY());
            pc.* = u.compressUnitVector();
        }
    }

    const out4_z = try allocator.alloc(float4, n);
    defer allocator.free(out4_z);
    const out4_j = try allocator.alloc(float4, n);
    defer allocator.free(out4_j);
    const out3_z = try allocator.alloc(float3, n);
    defer allocator.free(out3_z);
    const out3_j = try allocator.alloc(float3, n);
    defer allocator.free(out3_j);
    const out_z = try allocator.alloc(f32, n);
    defer allocator.free(out_z);
    const out_j = try allocator.alloc(f32, n);
    defer allocator.free(out_j);

    for (point, normal, out4_z) |pp, pn, *o| o.* = ShaderPlane.planeFromPointAndNormal(pp, pn);
    jolt_shader_plane_from_point_and_normal(point.ptr, normal.ptr, out4_j.ptr, n);
    checkFloats(&checker, "planeFromPointAndNormal", float4, point, out4_z, out4_j);

    for (plane, out3_z) |ppl, *o| o.* = ShaderPlane.planeGetNormal(ppl);
    jolt_shader_plane_get_normal(plane.ptr, out3_j.ptr, n);
    checkFloats(&checker, "planeGetNormal", float3, plane, out3_z, out3_j);

    for (plane, point, out_z) |ppl, pp, *o| o.* = ShaderPlane.planeSignedDistance(ppl, pp);
    jolt_shader_plane_signed_distance(plane.ptr, point.ptr, out_j.ptr, n);
    for (plane, out_z, out_j) |ppl, z, j| checker.check(.{ .function = "planeSignedDistance", .plane = ppl }, z, j);

    for (compressed, out3_z) |c, *o| o.* = ShaderVec3.vec3DecompressUnit(c);
    jolt_shader_vec3_decompress_unit(compressed.ptr, out3_j.ptr, n);
    checkFloats(&checker, "vec3DecompressUnit", float3, compressed, out3_z, out3_j);

    try checker.finish();
}

test "ShaderHelpersParity: HairCommon (packed bits and bytes, gradient sampler, grid helpers, constants)" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "HairCommon" };
    var rng: fw.Rng = .{ .state = 0x56789ab };

    // Constants
    var float_to_fixed: c_int = undefined;
    var fixed_to_float: f32 = undefined;
    jolt_shader_hair_common_constants(&float_to_fixed, &fixed_to_float);
    checker.check("cFloatToFixed", @as(c_int, HairCommon.float_to_fixed), float_to_fixed);
    checker.check("cFixedToFloat", HairCommon.fixed_to_float, fixed_to_float);

    // The packed buffers: 1000 words, every index of them
    var words: [1000]u32 = undefined;
    for (&words) |*w| w.* = rng.next();
    const buffer: hlsl.Buffer(u32) = .{ .data = &words };
    const indices = try allocator.alloc(u32, n);
    defer allocator.free(indices);
    for (indices, 0..) |*idx, i| idx.* = @intCast(i % (words.len * 4)); // The byte readers read word index >> 2
    const fixed_z = try allocator.alloc(u32, n);
    defer allocator.free(fixed_z);
    const fixed_j = try allocator.alloc(u32, n);
    defer allocator.free(fixed_j);
    const inv_mass_z = try allocator.alloc(f32, n);
    defer allocator.free(inv_mass_z);
    const inv_mass_j = try allocator.alloc(f32, n);
    defer allocator.free(inv_mass_j);
    const fraction_z = try allocator.alloc(f32, n);
    defer allocator.free(fraction_z);
    const fraction_j = try allocator.alloc(f32, n);
    defer allocator.free(fraction_j);
    const count_z = try allocator.alloc(u32, n);
    defer allocator.free(count_z);
    const count_j = try allocator.alloc(u32, n);
    defer allocator.free(count_j);
    const material_z = try allocator.alloc(u32, n);
    defer allocator.free(material_z);
    const material_j = try allocator.alloc(u32, n);
    defer allocator.free(material_j);
    for (indices, 0..) |idx, i| {
        fixed_z[i] = @intFromBool(HairCommon.isVertexFixed(buffer, idx));
        inv_mass_z[i] = HairCommon.getVertexInvMass(buffer, idx);
        fraction_z[i] = HairCommon.getVertexStrandFraction(buffer, idx);
        count_z[i] = HairCommon.getStrandVertexCount(buffer, idx);
        material_z[i] = HairCommon.getStrandMaterialIndex(buffer, idx);
    }
    jolt_shader_hair_packed(&words, indices.ptr, fixed_j.ptr, inv_mass_j.ptr, fraction_j.ptr, count_j.ptr, material_j.ptr, n);
    for (indices, 0..) |idx, i| {
        checker.check(.{ .function = "isVertexFixed", .index = idx }, fixed_z[i], fixed_j[i]);
        checker.check(.{ .function = "getVertexInvMass", .index = idx }, inv_mass_z[i], inv_mass_j[i]);
        checker.check(.{ .function = "getVertexStrandFraction", .index = idx }, fraction_z[i], fraction_j[i]);
        checker.check(.{ .function = "getStrandVertexCount", .index = idx }, count_z[i], count_j[i]);
        checker.check(.{ .function = "getStrandMaterialIndex", .index = idx }, material_z[i], material_j[i]);
    }

    // GradientSamplerSample
    const sampler = try allocator.alloc(float4, n);
    defer allocator.free(sampler);
    const strand_fraction = try allocator.alloc(f32, n);
    defer allocator.free(strand_fraction);
    for (sampler, strand_fraction) |*s, *f| {
        s.* = randomFloat4(&rng, 10);
        f.* = if (rng.next() % 4 == 0) specialFloat(&rng, 2) else rng.float(0, 1);
    }
    const sample_z = try allocator.alloc(f32, n);
    defer allocator.free(sample_z);
    const sample_j = try allocator.alloc(f32, n);
    defer allocator.free(sample_j);
    for (sampler, strand_fraction, sample_z) |s, f, *o| o.* = HairCommon.gradientSamplerSample(s, f);
    jolt_shader_gradient_sampler_sample(sampler.ptr, strand_fraction.ptr, sample_j.ptr, n);
    for (sampler, strand_fraction, sample_z, sample_j) |s, f, z, j| checker.check(.{ .function = "gradientSamplerSample", .sampler = s, .fraction = f }, z, j);

    // GridPositionToIndexAndFraction and GridIndexToBufferIndex, 100 random grids
    const per_grid = n / 100;
    const positions = try allocator.alloc(float3, per_grid);
    defer allocator.free(positions);
    const grid_indices = try allocator.alloc(uint3, per_grid);
    defer allocator.free(grid_indices);
    const index_z = try allocator.alloc(uint3, per_grid);
    defer allocator.free(index_z);
    const index_j = try allocator.alloc(uint3, per_grid);
    defer allocator.free(index_j);
    const frac_z = try allocator.alloc(float3, per_grid);
    defer allocator.free(frac_z);
    const frac_j = try allocator.alloc(float3, per_grid);
    defer allocator.free(frac_j);
    const buffer_index_z = try allocator.alloc(u32, per_grid);
    defer allocator.free(buffer_index_z);
    const buffer_index_j = try allocator.alloc(u32, per_grid);
    defer allocator.free(buffer_index_j);
    const buffer_index2_z = try allocator.alloc(u32, per_grid);
    defer allocator.free(buffer_index2_z);
    const buffer_index2_j = try allocator.alloc(u32, per_grid);
    defer allocator.free(buffer_index2_j);
    var num_compared: usize = 0;
    for (0..100) |grid| {
        var context = std.mem.zeroes(HairUpdateContext);
        // Garbage in the members the helpers do not read
        for (std.mem.bytesAsSlice(u32, std.mem.asBytes(&context))) |*w| w.* = rng.next();
        const size = uint3.init(rng.intRange(u32, 2, 40), rng.intRange(u32, 2, 40), rng.intRange(u32, 2, 40));
        context.grid_size_min2 = .init(size.x - 2, size.y - 2, size.z - 2);
        context.grid_size_min1 = .init(@floatFromInt(size.x - 1), @floatFromInt(size.y - 1), @floatFromInt(size.z - 1));
        context.grid_offset = .init(rng.float(-10, 10), rng.float(-10, 10), rng.float(-10, 10));
        context.grid_scale = .init(rng.float(0.1, 10), rng.float(0.1, 10), rng.float(0.1, 10));
        context.grid_stride = if (grid % 4 == 0) .init(rng.next(), rng.next(), rng.next()) else .init(1, size.x, size.x * size.y);
        if (grid % 10 == 0) {
            // Any floats in the grid constants. A NaN offset or scale or a negative scale or size would make every grid
            // position undefined: they become 0, 1 or positive.
            context.grid_offset = randomFloat3(&rng, 100);
            context.grid_scale = randomFloat3(&rng, 100);
            context.grid_size_min1 = randomFloat3(&rng, 100);
            inline for (.{ "x", "y", "z" }) |c| {
                if (std.math.isNan(@field(context.grid_offset, c))) @field(context.grid_offset, c) = 0;
                @field(context.grid_scale, c) = if (std.math.isNan(@field(context.grid_scale, c))) 1 else @abs(@field(context.grid_scale, c));
                @field(context.grid_size_min1, c) = @abs(@field(context.grid_size_min1, c)); // A NaN size is defined: min(v, NaN) is v
            }
        }
        // Only positions with a defined grid index (R20), the others are drawn again
        var num_positions: usize = 0;
        var attempts: usize = 0;
        while (num_positions < per_grid and attempts < 4 * per_grid) : (attempts += 1) {
            const p = if (rng.next() % 8 == 0) randomFloat3(&rng, 100) else context.grid_offset.add(.init(rng.float(-1, 50), rng.float(-1, 50), rng.float(-1, 50)));
            if (!gridPositionDefined(&context, p)) continue;
            positions[num_positions] = p;
            grid_indices[num_positions] = .init(rng.next(), rng.next(), rng.next());
            num_positions += 1;
        }
        num_compared += num_positions;
        for (positions[0..num_positions], grid_indices[0..num_positions], index_z[0..num_positions], frac_z[0..num_positions], buffer_index_z[0..num_positions], buffer_index2_z[0..num_positions]) |p, gi, *iz, *fz, *bz, *b2z| {
            const r = HairCommon.gridPositionToIndexAndFraction(&context, p);
            iz.* = r.index;
            fz.* = r.fraction;
            bz.* = HairCommon.gridIndexToBufferIndex(&context, r.index);
            b2z.* = HairCommon.gridIndexToBufferIndex(&context, gi);
        }
        jolt_shader_grid(&context, positions.ptr, grid_indices.ptr, index_j.ptr, frac_j.ptr, buffer_index_j.ptr, buffer_index2_j.ptr, @intCast(num_positions));
        for (positions[0..num_positions], 0..) |p, i| {
            checker.check(.{ .function = "gridPositionToIndexAndFraction (index)", .position = p }, index_z[i], index_j[i]);
            checker.check(.{ .function = "gridPositionToIndexAndFraction (fraction)", .position = p }, frac_z[i], frac_j[i]);
            checker.check(.{ .function = "gridIndexToBufferIndex", .position = p }, buffer_index_z[i], buffer_index_j[i]);
            checker.check(.{ .function = "gridIndexToBufferIndex", .index = grid_indices[i] }, buffer_index2_z[i], buffer_index2_j[i]);
        }
    }
    // The filter keeps nearly every position
    try std.testing.expect(num_compared * 10 >= 9 * 100 * per_grid);

    try checker.finish();
}
