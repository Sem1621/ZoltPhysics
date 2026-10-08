//! Parity tests for the basics of the physics code (Phase 4 foundation): MassProperties (every function, including
//! the binary state bytes), BodyID, SubShapeIDCreator push / SubShapeID pop sequences, SubShapeIDPair, RayCast /
//! RRayCast (Transformed, Translated, GetPointOnRay, the conversions), AABoxCast, CollideShapeResult::Reversed, the
//! default values of the settings / result types and of PhysicsSettings, the layer constants, SortReverseAndStore /
//! CountAndSortTrues and the PhysicsMaterial / PhysicsMaterialSimple binary state, restore and RTTI hashes. Zolt and
//! the C++ Jolt library run on the same inputs and must produce identical bits. C ABI wrappers:
//! ZoltParity/Physics/BasicsReference.cpp. See ZoltParity/parity.zig for how parity tests work.
//!
//! The inputs are random values mixed with special values (zero, -0, tiny, huge values that overflow), small integer
//! grids (exact ties in sorts and comparisons), degenerate inertia tensors (zero, repeated eigenvalues, rank 1 and 2,
//! rotated and translated boxes, indefinite tensors) and truncated or corrupted streams.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const AABox = zolt.AABox;
const AABoxCast = zolt.AABoxCast;
const BodyID = zolt.BodyID;
const CollidePointResult = zolt.CollidePointResult;
const CollideSettingsBase = zolt.CollideSettingsBase;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const Color = zolt.Color;
const Core = zolt.Core;
const DMat44 = zolt.DMat44;
const HashCombine = zolt.HashCombine;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const PhysicsSettings = zolt.PhysicsSettings;
const physics_settings = zolt.physics_settings;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Real = zolt.Real;
const RMat44 = zolt.RMat44;
const RRayCast = zolt.RRayCast;
const RVec3 = zolt.RVec3;
const BroadPhaseCastResult = zolt.BroadPhaseCastResult;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const SubShapeIDPair = zolt.SubShapeIDPair;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// The C++ reference functions, see BasicsReference.cpp
const jolt = struct {
    extern fn jolt_mass_properties_solid_box(box_size: *const P, density: f32, out: *MP) void;
    extern fn jolt_mass_properties_scale_to_mass(in: *const MP, mass: f32, out: *MP) void;
    extern fn jolt_mass_properties_equivalent_solid_box_size(mass: f32, inertia_diagonal: *const P, out: *P) void;
    extern fn jolt_mass_properties_rotate(in: *const MP, rotation: *const [16]f32, out: *MP) void;
    extern fn jolt_mass_properties_translate(in: *const MP, translation: *const P, out: *MP) void;
    extern fn jolt_mass_properties_scale(in: *const MP, scale: *const P, out: *MP) void;
    extern fn jolt_mass_properties_decompose(in: *const MP, io_rotation: *[16]f32, io_diagonal: *P) c_int;
    extern fn jolt_mass_properties_equal(a: *const MP, b: *const MP) c_int;
    extern fn jolt_mass_properties_save(in: *const MP, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_mass_properties_restore(bytes: [*]const u8, size: u32, io: *MP, out_eof: *c_int, out_failed: *c_int) void;

    extern fn jolt_body_id(index: u32, sequence_number: u32, out_values: *[4]u32, out_hash: *u64) void;
    extern fn jolt_body_id_compare(a: u32, b: u32, out_results: *[5]c_int) void;

    extern fn jolt_sub_shape_id_push(values: [*]const u32, bits: [*]const u32, count: c_int, out_ids: [*]u32, out_bits_written: [*]u32) void;
    extern fn jolt_sub_shape_id_pop(value: u32, bits: u32, out_remainder: *u32, out_is_empty: *c_int) u32;
    extern fn jolt_sub_shape_id_pair(a: *const [4]u32, b: *const [4]u32, out_results: *[3]c_int, out_hash: *[2]u64) void;

    extern fn jolt_ray_cast(origin: *const P, direction: *const P, transform: *const [16]f32, translation: *const P, fraction: f32, out_transformed: *[6]f32, out_translated: *[6]f32, out_point: *P) void;
    extern fn jolt_r_ray_cast(origin: *const R3, direction: *const P, transform_columns: *const [12]f32, transform_translation: *const R3, translation: *const R3, fraction: f32, out_transformed_origin: *R3, out_transformed_direction: *P, out_translated_origin: *R3, out_translated_direction: *P, out_point: *R3, out_ray_cast: *[6]f32) void;
    extern fn jolt_r_ray_cast_from_ray_cast(origin: *const P, direction: *const P, out_origin: *R3, out_direction: *P) void;
    extern fn jolt_ray_cast_settings_defaults(out_values: *[5]u32) void;
    extern fn jolt_aabox_cast(min: *const P, max: *const P, direction: *const P, out_values: *[9]f32) void;

    extern fn jolt_collide_shape_result_reversed(in: *const CSR, out: *CSR, out_early_out: *[2]f32) void;
    extern fn jolt_collide_shape_settings_defaults(out_enums: *[5]u32, out_floats: *[12]f32) void;
    extern fn jolt_result_defaults(out_ids: *[18]u32, out_fractions: *[5]f32) void;
    extern fn jolt_layer_constants(out_values: *[4]u32) void;
    extern fn jolt_physics_settings(out_constants: *[5]f32, out_int_constants: *[2]i32, out_ints: *[3]i32, out_floats: *[15]f32, out_uints: *[2]u32, out_bools: *[6]c_int) void;

    extern fn jolt_sort_reverse_and_store(values: *const [4]f32, max_value: f32, io_identifiers: *[4]u32, out_values: *[4]f32) c_int;
    extern fn jolt_count_and_sort_trues(value: *const [4]u32, io_identifiers: *[4]u32) c_int;

    extern fn jolt_physics_material_rtti_hashes(out_hashes: *[2]u32) void;
    extern fn jolt_physics_material_save(simple: c_int, name: [*]const u8, name_length: u32, color: u32, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_physics_material_restore(bytes: [*]const u8, size: u32, out_hash: *u32, out_text: [*]u8, text_capacity: u32, out_text_length: *u32, out_color: *u32) c_int;
    extern fn jolt_default_material(out_name: [*]u8, capacity: u32, out_color: *u32, out_hash: *u32) u32;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// An RVec3 as passed to the C ABI
const R3 = [3]Real;

/// MassProperties in the format of the C ABI (mass, inertia in column major order), must match MP in BasicsReference.cpp
const MP = extern struct {
    mass: f32,
    inertia: [16]f32,

    fn toMassProperties(self: MP) MassProperties {
        return .{ .mass = self.mass, .inertia = mat44(self.inertia) };
    }

    fn fromMassProperties(m: MassProperties) MP {
        return .{ .mass = m.mass, .inertia = arr16(m.inertia) };
    }
};

/// CollideShapeResult in the format of the C ABI, must match CSR in BasicsReference.cpp
const CSR = extern struct {
    contact_point_on1: P,
    contact_point_on2: P,
    penetration_axis: P,
    penetration_depth: f32,
    sub_shape_id1: u32,
    sub_shape_id2: u32,
    body_id2: u32,
    face1_count: u32,
    face1: [32]P,
    face2_count: u32,
    face2: [32]P,

    fn toResult(self: *const CSR) CollideShapeResult {
        var r = CollideShapeResult.init(vec3(self.contact_point_on1), vec3(self.contact_point_on2), vec3(self.penetration_axis), self.penetration_depth, .{ .value = self.sub_shape_id1 }, .{ .value = self.sub_shape_id2 }, .{ .id = self.body_id2 });
        for (self.face1[0..self.face1_count]) |v| r.shape1_face.append(vec3(v));
        for (self.face2[0..self.face2_count]) |v| r.shape2_face.append(vec3(v));
        return r;
    }

    fn fromResult(r: *const CollideShapeResult) CSR {
        var c = std.mem.zeroes(CSR);
        c.contact_point_on1 = arr3(r.contact_point_on1);
        c.contact_point_on2 = arr3(r.contact_point_on2);
        c.penetration_axis = arr3(r.penetration_axis);
        c.penetration_depth = r.penetration_depth;
        c.sub_shape_id1 = r.sub_shape_id1.getValue();
        c.sub_shape_id2 = r.sub_shape_id2.getValue();
        c.body_id2 = r.body_id2.getIndexAndSequenceNumber();
        c.face1_count = r.shape1_face.len;
        for (r.shape1_face.constSlice(), 0..) |v, i| c.face1[i] = arr3(v);
        c.face2_count = r.shape2_face.len;
        for (r.shape2_face.constSlice(), 0..) |v, i| c.face2[i] = arr3(v);
        return c;
    }
};

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn arr4(v: Vec4) [4]f32 {
    return .{ v.getX(), v.getY(), v.getZ(), v.getW() };
}

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn rvec3(a: R3) RVec3 {
    return RVec3.init(a[0], a[1], a[2]);
}

fn arrR3(v: RVec3) R3 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

/// RMat44(Vec4, Vec4, Vec4, RVec3): 3 columns and a translation
fn rmat44(columns: [12]f32, translation: R3) RMat44 {
    const c0 = vec4(columns[0..4].*);
    const c1 = vec4(columns[4..8].*);
    const c2 = vec4(columns[8..12].*);
    return if (Core.double_precision) DMat44.init(c0, c1, c2, rvec3(translation)) else Mat44.fromColumnsTranslation(c0, c1, c2, rvec3(translation));
}

/// Values that tend to expose differences, mixed into the random inputs (1e30 overflows when multiplied)
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 12.0, 1.0e-6, -1.0e-6, 1.0e-20, -1.0e-20, 1.0e-40, 100.0, -100.0, 1.0e30, -1.0e30 };

/// Input generator: xorshift32 with helpers for the edge cases
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    /// Random index in [0, n)
    fn index(self: *Gen, n: usize) usize {
        return self.next() % n;
    }

    /// True one in n times
    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) return special_values[self.index(special_values.len)];
        return self.rng.float(min, max);
    }

    /// Random float in [min, max) without special values
    fn plain(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) P {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    fn plainVec(self: *Gen, min: f32, max: f32) P {
        return .{ self.plain(min, max), self.plain(min, max), self.plain(min, max) };
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// Random rotation matrix
    fn rotation(self: *Gen) Mat44 {
        while (true) {
            const q = self.rng.floatArray(4, -1, 1);
            const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return Mat44.rotationQuat(Quat.init(q[0], q[1], q[2], q[3]).normalized());
        }
    }

    /// Random 4x4 matrix: a rotation + translation, or random values in every element (including the bottom row)
    fn matrix(self: *Gen) [16]f32 {
        if (self.oneIn(2)) {
            var m = self.rotation();
            m.setTranslation(vec3(self.vec(-10, 10)));
            return arr16(m);
        }
        var a: [16]f32 = undefined;
        for (&a) |*v| v.* = self.float(-10, 10);
        return a;
    }

    /// Random mass properties: a box, a transformed box or random values in every element
    fn massProperties(self: *Gen) MP {
        var mp: MP = undefined;
        switch (self.next() % 4) {
            0 => {
                var m: MassProperties = .{};
                m.setMassAndInertiaOfSolidBox(vec3(self.vec(0.1, 10)), self.float(1, 1000));
                return .fromMassProperties(m);
            },
            1 => {
                var m: MassProperties = .{};
                m.setMassAndInertiaOfSolidBox(vec3(self.plainVec(0.1, 10)), self.plain(1, 1000));
                m.rotate(self.rotation());
                m.translate(vec3(self.vec(-5, 5)));
                return .fromMassProperties(m);
            },
            else => {
                mp.mass = self.float(-10, 100);
                for (&mp.inertia) |*v| v.* = self.float(-100, 100);
                return mp;
            },
        }
    }

    /// Inertia tensors for DecomposePrincipalMomentsOfInertia: physical (boxes, rotated, translated), random symmetric,
    /// indefinite and degenerate ones (zero, repeated eigenvalues, rank 1 and 2, tiny and huge). The values outside of
    /// the 3x3 part are random (they must be ignored).
    fn inertiaTensor(self: *Gen) MP {
        var m: MassProperties = .{ .mass = self.float(0, 10) };
        switch (self.next() % 10) {
            // A rotated and translated box
            0, 1 => {
                m.setMassAndInertiaOfSolidBox(vec3(self.plainVec(0.01, 10)), self.plain(1, 1000));
                if (!self.oneIn(4)) m.rotate(self.rotation());
                if (self.oneIn(2)) m.translate(vec3(self.plainVec(-5, 5)));
            },
            // Random symmetric, sometimes indefinite
            2, 3 => {
                const s = if (self.oneIn(4)) self.float(-10, 10) else 1.0;
                for (0..3) |r| for (r..3) |c| {
                    const v = s * self.float(-10, 10);
                    m.inertia.set(@intCast(r), @intCast(c), v);
                    m.inertia.set(@intCast(c), @intCast(r), v);
                };
            },
            // Diagonal with repeated values in a random order (sorting ties), sometimes rotated
            4, 5 => {
                const values = [_]f32{ self.grid(3), self.grid(3), self.float(-5, 5) };
                const p = self.index(6);
                const perm = [6][3]u32{ .{ 0, 1, 2 }, .{ 0, 2, 1 }, .{ 1, 0, 2 }, .{ 1, 2, 0 }, .{ 2, 0, 1 }, .{ 2, 1, 0 } };
                for (0..3) |i| m.inertia.set(@intCast(i), @intCast(i), values[perm[p][i]]);
                if (self.oneIn(3)) m.rotate(self.rotation());
            },
            // Rank 1 and rank 2 (a rod and a plate)
            6 => {
                const v = vec3(self.plainVec(-2, 2));
                m.inertia = Mat44.outerProduct(v, v);
                if (self.oneIn(2)) {
                    const w = vec3(self.plainVec(-2, 2));
                    m.inertia = m.inertia.add(Mat44.outerProduct(w, w));
                }
            },
            // Zero, a multiple of the identity, tiny or huge
            7 => {
                const k = if (self.oneIn(3)) 0.0 else self.float(-1.0e6, 1.0e6);
                m.inertia = Mat44.scale(k);
                if (self.oneIn(2)) m.rotate(self.rotation());
            },
            // Small integer grid (exact values, many ties)
            else => {
                for (0..3) |r| for (r..3) |c| {
                    const v = self.grid(2);
                    m.inertia.set(@intCast(r), @intCast(c), v);
                    m.inertia.set(@intCast(c), @intCast(r), v);
                };
            },
        }
        var mp = MP.fromMassProperties(m);
        if (self.oneIn(2)) {
            // Garbage outside of the 3x3 part
            for (0..4) |i| {
                mp.inertia[12 + i] = self.float(-10, 10);
                mp.inertia[4 * i + 3] = self.float(-10, 10);
            }
        }
        return mp;
    }
};

/// True if the 3x3 part of a column major matrix is finite
fn isFinite3x3(m: [16]f32) bool {
    for (0..3) |c| for (0..3) |r| {
        if (!std.math.isFinite(m[4 * c + r])) return false;
    };
    return true;
}

/// Saves with Zolt into a buffer, returns the bytes
fn saveMassProperties(m: *const MassProperties, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    m.saveBinaryState(out.streamOut());
    return writer.buffered();
}

test "MassProperties parity" {
    var gen: Gen = .{};
    var solid_box: Checker = .{ .name = "MassProperties.setMassAndInertiaOfSolidBox" };
    var scale_to_mass: Checker = .{ .name = "MassProperties.scaleToMass" };
    var equivalent: Checker = .{ .name = "MassProperties.getEquivalentSolidBoxSize" };
    var rotate: Checker = .{ .name = "MassProperties.rotate" };
    var translate: Checker = .{ .name = "MassProperties.translate" };
    var scale: Checker = .{ .name = "MassProperties.scale" };
    var equal: Checker = .{ .name = "MassProperties.eql" };

    for (0..iterations) |_| {
        const mp = gen.massProperties();
        const m = mp.toMassProperties();

        // SetMassAndInertiaOfSolidBox
        {
            const size = gen.vec(-10, 10);
            const density = gen.float(-10, 1000);
            var j: MP = undefined;
            jolt.jolt_mass_properties_solid_box(&size, density, &j);
            var z = m;
            z.setMassAndInertiaOfSolidBox(vec3(size), density);
            solid_box.check(.{ size, density }, MP.fromMassProperties(z), j);
        }

        // ScaleToMass (also with a zero / negative mass)
        {
            const mass = gen.float(-10, 100);
            var j: MP = undefined;
            jolt.jolt_mass_properties_scale_to_mass(&mp, mass, &j);
            var z = m;
            z.scaleToMass(mass);
            scale_to_mass.check(.{ mp, mass }, MP.fromMassProperties(z), j);
        }

        // sGetEquivalentSolidBoxSize (negative values under the square root give NaN)
        {
            const mass = gen.float(0.01, 100);
            const diagonal = if (gen.oneIn(2)) arr3(m.inertia.getDiagonal3()) else gen.vec(-10, 100);
            var j: P = undefined;
            jolt.jolt_mass_properties_equivalent_solid_box_size(mass, &diagonal, &j);
            equivalent.check(.{ mass, diagonal }, arr3(MassProperties.getEquivalentSolidBoxSize(mass, vec3(diagonal))), j);
        }

        // Rotate. Mat44::Multiply3x3(Mat44) and Multiply3x3RightTransposed assert that W of the first 3 columns is 0: the
        // rotation gets W = 0, the 3x3 part can be anything. An infinite inertia makes W of the intermediate product NaN
        // (0 * inf), Jolt's assert fires on that too, so builds with asserts skip those inputs (only the ReleaseFast
        // parity run, which CI does too, compares them).
        if (!Core.enable_asserts or isFinite3x3(mp.inertia)) {
            var rotation = gen.matrix();
            for (0..3) |c| rotation[4 * c + 3] = 0.0;
            var j: MP = undefined;
            jolt.jolt_mass_properties_rotate(&mp, &rotation, &j);
            var z = m;
            z.rotate(mat44(rotation));
            rotate.check(.{ mp, rotation }, MP.fromMassProperties(z), j);
        }

        // Translate
        {
            const translation = gen.vec(-10, 10);
            var j: MP = undefined;
            jolt.jolt_mass_properties_translate(&mp, &translation, &j);
            var z = m;
            z.translate(vec3(translation));
            translate.check(.{ mp, translation }, MP.fromMassProperties(z), j);
        }

        // Scale (negative scales flip the shape)
        {
            const s = if (gen.oneIn(4)) [3]f32{ gen.grid(2), gen.grid(2), gen.grid(2) } else gen.vec(-3, 3);
            var j: MP = undefined;
            jolt.jolt_mass_properties_scale(&mp, &s, &j);
            var z = m;
            z.scale(vec3(s));
            scale.check(.{ mp, s }, MP.fromMassProperties(z), j);
        }

        // operator == (equal, one element different, -0 vs +0)
        {
            var other = mp;
            switch (gen.next() % 4) {
                0 => {},
                1 => other.mass = gen.float(-10, 100),
                2 => other.inertia[gen.index(16)] = gen.float(-100, 100),
                else => for (&other.inertia) |*v| {
                    if (v.* == 0) v.* = -v.*;
                },
            }
            const z: c_int = @intFromBool(m.eql(&other.toMassProperties()));
            equal.check(.{ mp, other }, z, jolt.jolt_mass_properties_equal(&mp, &other));
        }
    }

    try finishAll(&.{ &solid_box, &scale_to_mass, &equivalent, &rotate, &translate, &scale, &equal });
}

/// True if DecomposePrincipalMomentsOfInertia asserts (in a build with asserts) for the tensor `mp`, given the result
/// of the decomposition: EigenValueSymmetric does not converge ("Too many iterations"), its sanity checks (the
/// eigenvectors are normalized and inMatrix * eigen_vector = eigen_value * eigen_vector) or the final check of the
/// reconstructed tensor fail. The checks are evaluated with the same operations as the asserts. The eigenvectors are the
/// columns of the rotation (possibly negated, which does not change the checks) and the eigenvalues the diagonal.
fn wouldAssert(mp: MP, ok: c_int, rotation_values: [16]f32, diagonal_values: P) bool {
    if (ok == 0) return true;

    // EigenValueSymmetric
    const Matrix3 = zolt.Matrix(3, 3);
    const Vector3 = zolt.Vector(3);
    var matrix: Matrix3 = undefined;
    matrix.copyPart(mat44(mp.inertia), 0, 0, 3, 3, 0, 0);
    for (0..3) |c| {
        const eig_vec: Vector3 = .{ .f32s = rotation_values[4 * c ..][0..3].* };
        const eig_val = diagonal_values[c];
        if (!eig_vec.isNormalized(.{})) return true;
        const mat_eigvec = matrix.mulVector(eig_vec);
        const eigval_eigvec = eig_vec.mulScalar(eig_val);
        if (!mat_eigvec.isClose(eigval_eigvec, .{ .max_dist_sq = zolt.math.max(mat_eigvec.lengthSq(), eigval_eigvec.lengthSq()) * 1.0e-6 })) return true;
    }

    // MassProperties::DecomposePrincipalMomentsOfInertia
    const rotation = mat44(rotation_values);
    const new_inertia = rotation.mul(Mat44.scaleVec3(vec3(diagonal_values))).mul(rotation.inversed());
    const inertia = mat44(mp.inertia);
    for (0..3) |i| {
        const c: u32 = @intCast(i);
        if (!new_inertia.getColumn3(c).isClose(inertia.getColumn3(c), .{ .max_dist_sq = inertia.getColumn3(c).lengthSq() * 1.0e-10 }))
            return true;
    }
    return false;
}

test "MassProperties.decomposePrincipalMomentsOfInertia parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x2468ace1 } };
    var decompose: Checker = .{ .name = "MassProperties.decomposePrincipalMomentsOfInertia" };
    var num_compared: usize = 0;
    var num_skipped: usize = 0;

    const Result = struct { ok: c_int, rotation: [16]f32, diagonal: P };
    const sentinel_rotation: [16]f32 = @splat(42.0);
    const sentinel_diagonal: P = @splat(-42.0);

    for (0..iterations) |_| {
        const mp = gen.inertiaTensor();

        var j: Result = .{ .ok = 0, .rotation = sentinel_rotation, .diagonal = sentinel_diagonal };
        j.ok = jolt.jolt_mass_properties_decompose(&mp, &j.rotation, &j.diagonal);

        // Jolt checks the decomposition with asserts (JPH_ENABLE_ASSERTS, enabled in Zolt's safe builds) that ill
        // conditioned and rank deficient tensors violate. Skip the inputs for which Jolt's own asserts would fire,
        // decided on Jolt's result: Zolt computes the same bits, so it would assert on exactly these. Builds without
        // asserts compare every input: the failing decompositions (ok == 0, the outputs keep the sentinels) and the
        // degenerate tensors are only covered by the ReleaseFast parity run (part of the CI parity matrix).
        if (Core.enable_asserts and wouldAssert(mp, j.ok, j.rotation, j.diagonal)) {
            num_skipped += 1;
            continue;
        }

        var z: Result = .{ .ok = 0, .rotation = sentinel_rotation, .diagonal = sentinel_diagonal };
        if (mp.toMassProperties().decomposePrincipalMomentsOfInertia()) |d| {
            z = .{ .ok = 1, .rotation = arr16(d.rotation), .diagonal = arr3(d.diagonal) };
        }
        decompose.check(mp, z, j);
        num_compared += 1;
    }

    try finishAll(&.{&decompose});
    if (num_compared < iterations * 8 / 10) {
        std.debug.print("MassProperties.decomposePrincipalMomentsOfInertia: only {d} of {d} inputs compared ({d} skipped)\n", .{ num_compared, iterations, num_skipped });
        return error.TooManySkipped;
    }
}

test "MassProperties binary state parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x13572468 } };
    var save: Checker = .{ .name = "MassProperties.saveBinaryState" };
    var restore: Checker = .{ .name = "MassProperties.restoreBinaryState" };

    const Restored = struct { mp: MP, eof: c_int, failed: c_int };

    for (0..iterations / 10) |_| {
        const mp = gen.massProperties();
        const m = mp.toMassProperties();

        var j_bytes: [128]u8 = undefined;
        const j_size = jolt.jolt_mass_properties_save(&mp, &j_bytes, j_bytes.len);
        var z_buffer: [128]u8 = undefined;
        const z_bytes = saveMassProperties(&m, &z_buffer);
        const Bytes = struct { size: u32, bytes: [68]u8 };
        var z_saved: Bytes = .{ .size = @intCast(z_bytes.len), .bytes = @splat(0) };
        @memcpy(z_saved.bytes[0..@min(z_bytes.len, 68)], z_bytes[0..@min(z_bytes.len, 68)]);
        var j_saved: Bytes = .{ .size = j_size, .bytes = @splat(0) };
        @memcpy(j_saved.bytes[0..@min(j_size, 68)], j_bytes[0..@min(j_size, 68)]);
        save.check(mp, z_saved, j_saved);

        // Restore the full stream or a truncated one into other mass properties
        const size: u32 = if (gen.oneIn(2)) j_size else @intCast(gen.index(j_size + 1));
        const initial = gen.massProperties();
        var j: Restored = .{ .mp = initial, .eof = 0, .failed = 0 };
        jolt.jolt_mass_properties_restore(&j_bytes, size, &j.mp, &j.eof, &j.failed);

        var reader: std.Io.Reader = .fixed(j_bytes[0..size]);
        var in = StreamInWrapper.init(&reader);
        var restored = initial.toMassProperties();
        restored.restoreBinaryState(in.streamIn());
        const z: Restored = .{ .mp = .fromMassProperties(restored), .eof = @intFromBool(in.isEOF()), .failed = @intFromBool(in.isFailed()) };
        restore.check(.{ mp, size }, z, j);
    }

    try finishAll(&.{ &save, &restore });
}

test "BodyID parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x0badf00d } };
    var body_id: Checker = .{ .name = "BodyID" };
    var compare: Checker = .{ .name = "BodyID comparisons" };

    const Values = struct { values: [4]u32, hash: u64 };

    for (0..iterations) |_| {
        const idx: u32 = if (gen.oneIn(10)) @intCast(gen.index(4)) else gen.next() & BodyID.max_body_index;
        const seq: u32 = gen.next() & 0xff;
        var j: Values = undefined;
        jolt.jolt_body_id(idx, seq, &j.values, &j.hash);
        const id = BodyID.fromIndexAndSequenceNumber(idx, @intCast(seq));
        const z: Values = .{ .values = .{ id.getIndexAndSequenceNumber(), id.getIndex(), id.getSequenceNumber(), @intFromBool(id.isInvalid()) }, .hash = HashCombine.hash(id) };
        body_id.check(.{ idx, seq }, z, j);

        // Comparisons, with equal ids and the invalid id
        const a: u32 = if (gen.oneIn(8)) BodyID.invalid_body_id else gen.next() & 0x7fffffff & (if (gen.oneIn(2)) @as(u32, 7) else 0xffffffff);
        const b: u32 = if (gen.oneIn(4)) a else if (gen.oneIn(8)) BodyID.invalid_body_id else gen.next() & 0x7fffffff & (if (gen.oneIn(2)) @as(u32, 7) else 0xffffffff);
        var jc: [5]c_int = undefined;
        jolt.jolt_body_id_compare(a, b, &jc);
        const ida = BodyID.init(a);
        const idb = BodyID.init(b);
        const zc = [5]c_int{ @intFromBool(ida.eql(idb)), @intFromBool(!ida.eql(idb)), @intFromBool(ida.lessThan(idb)), @intFromBool(ida.greaterThan(idb)), @intFromBool(ida.isInvalid()) };
        compare.check(.{ a, b }, zc, jc);
    }

    try finishAll(&.{ &body_id, &compare });
}

test "SubShapeID parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x51ab5eed } };
    var push: Checker = .{ .name = "SubShapeIDCreator.pushID" };
    var pop: Checker = .{ .name = "SubShapeID.popID" };
    var pair: Checker = .{ .name = "SubShapeIDPair" };

    const max_pushes = 12;
    const PushResult = struct { ids: [max_pushes]u32, bits: [max_pushes]u32 };
    const PopResult = struct { id: u32, remainder: u32, is_empty: c_int };
    const PairResult = struct { results: [3]c_int, hash: [2]u64 };

    for (0..iterations) |_| {
        // Push sequences: random bit counts (including 0 and 32 at once) that fit in 32 bits, random values that fit
        var values: [max_pushes]u32 = @splat(0);
        var bits: [max_pushes]u32 = @splat(0);
        const count = 1 + gen.index(max_pushes);
        var total: u32 = 0;
        for (0..count) |i| {
            const left = 32 - total;
            const b: u32 = if (gen.oneIn(10)) left else if (gen.oneIn(8)) 0 else @intCast(gen.index(@min(left, 12) + 1));
            const max_value: u64 = (@as(u64, 1) << @intCast(b)) - 1;
            const v: u32 = switch (gen.next() % 4) {
                0 => @intCast(max_value), // All ones
                1 => 0,
                else => @intCast((@as(u64, gen.next()) << 32 | gen.next()) & max_value),
            };
            values[i] = v;
            bits[i] = b;
            total += b;
        }
        var j: PushResult = .{ .ids = @splat(0), .bits = @splat(0) };
        jolt.jolt_sub_shape_id_push(&values, &bits, @intCast(count), &j.ids, &j.bits);
        var z: PushResult = .{ .ids = @splat(0), .bits = @splat(0) };
        var creator: SubShapeIDCreator = .{};
        for (0..count) |i| {
            creator = creator.pushID(values[i], bits[i]);
            z.ids[i] = creator.getID().getValue();
            z.bits[i] = creator.getNumBitsWritten();
        }
        push.check(.{ values, bits, count }, z, j);

        // Pop: the id that was just built or a random value, popping 0..32 bits
        {
            const value: u32 = switch (gen.next() % 4) {
                0 => z.ids[count - 1],
                1 => SubShapeID.empty_value,
                else => gen.next(),
            };
            const b: u32 = if (gen.oneIn(2)) bits[gen.index(count)] else @intCast(gen.index(33));
            var jp: PopResult = undefined;
            jp.id = jolt.jolt_sub_shape_id_pop(value, b, &jp.remainder, &jp.is_empty);
            const id: SubShapeID = .{ .value = value };
            const r = id.popID(b);
            pop.check(.{ value, b }, PopResult{ .id = r.id, .remainder = r.remainder.getValue(), .is_empty = @intFromBool(id.isEmpty()) }, jp);
        }

        // SubShapeIDPair: equality, ordering (equal components are frequent) and hash
        {
            var a: [4]u32 = undefined;
            var b: [4]u32 = undefined;
            for (&a, &b, 0..) |*x, *y, i| {
                const body = i % 2 == 0;
                x.* = if (body) (if (gen.oneIn(8)) BodyID.invalid_body_id else gen.next() & 0x7fffffff & 3) else gen.next() & (if (gen.oneIn(2)) @as(u32, 3) else 0xffffffff);
                y.* = if (gen.oneIn(2)) x.* else if (body) (if (gen.oneIn(8)) BodyID.invalid_body_id else gen.next() & 0x7fffffff & 3) else gen.next() & 3;
            }
            var jr: PairResult = undefined;
            jolt.jolt_sub_shape_id_pair(&a, &b, &jr.results, &jr.hash);
            const pa = SubShapeIDPair.init(.init(a[0]), .{ .value = a[1] }, .init(a[2]), .{ .value = a[3] });
            const pb = SubShapeIDPair.init(.init(b[0]), .{ .value = b[1] }, .init(b[2]), .{ .value = b[3] });
            const zr: PairResult = .{ .results = .{ @intFromBool(pa.eql(&pb)), @intFromBool(pa.lessThan(&pb)), @intFromBool(pb.lessThan(&pa)) }, .hash = .{ pa.getHash(), HashCombine.hash(pa) } };
            pair.check(.{ a, b }, zr, jr);
        }
    }

    try finishAll(&.{ &push, &pop, &pair });
}

test "RayCast / RRayCast parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x7a7a7a7a } };
    var ray_cast: Checker = .{ .name = "RayCast" };
    var r_ray_cast: Checker = .{ .name = "RRayCast" };
    var from_ray_cast: Checker = .{ .name = "RRayCast.fromRayCast" };
    var aabox_cast: Checker = .{ .name = "AABoxCast" };

    const RayResult = struct { transformed: [6]f32, translated: [6]f32, point: P };
    const RRayResult = struct { transformed_origin: R3, transformed_direction: P, translated_origin: R3, translated_direction: P, point: R3, ray_cast: [6]f32 };
    const FromResult = struct { origin: R3, direction: P };

    for (0..iterations) |_| {
        const origin = gen.vec(-100, 100);
        const direction = gen.vec(-100, 100);
        const transform = gen.matrix();
        const translation = gen.vec(-100, 100);
        const fraction = if (gen.oneIn(4)) gen.float(-1, 2) else gen.plain(0, 1);

        var j: RayResult = undefined;
        jolt.jolt_ray_cast(&origin, &direction, &transform, &translation, fraction, &j.transformed, &j.translated, &j.point);
        const ray = RayCast.init(vec3(origin), vec3(direction));
        const t = ray.transformed(mat44(transform));
        const tr = ray.translated(vec3(translation));
        const z: RayResult = .{ .transformed = arr3(t.origin) ++ arr3(t.direction), .translated = arr3(tr.origin) ++ arr3(tr.direction), .point = arr3(ray.getPointOnRay(fraction)) };
        ray_cast.check(.{ origin, direction, transform, translation, fraction }, z, j);

        // World space ray: with double precision the origin and translations are far from the origin
        const far: Real = if (Core.double_precision and gen.oneIn(2)) 1.0e7 else 1.0;
        const r_origin: R3 = .{ far * gen.float(-100, 100), far * gen.float(-100, 100), far * gen.float(-100, 100) };
        const r_translation: R3 = .{ far * gen.float(-100, 100), far * gen.float(-100, 100), far * gen.float(-100, 100) };
        const r_transform_translation: R3 = .{ far * gen.float(-100, 100), far * gen.float(-100, 100), far * gen.float(-100, 100) };
        var columns: [12]f32 = undefined;
        @memcpy(&columns, gen.matrix()[0..12]);
        var rj: RRayResult = undefined;
        jolt.jolt_r_ray_cast(&r_origin, &direction, &columns, &r_transform_translation, &r_translation, fraction, &rj.transformed_origin, &rj.transformed_direction, &rj.translated_origin, &rj.translated_direction, &rj.point, &rj.ray_cast);
        const r_ray = RRayCast.init(rvec3(r_origin), vec3(direction));
        const rt = r_ray.transformed(rmat44(columns, r_transform_translation));
        const rtr = r_ray.translated(rvec3(r_translation));
        const single = r_ray.toRayCast();
        const rz: RRayResult = .{
            .transformed_origin = arrR3(rt.origin),
            .transformed_direction = arr3(rt.direction),
            .translated_origin = arrR3(rtr.origin),
            .translated_direction = arr3(rtr.direction),
            .point = arrR3(r_ray.getPointOnRay(fraction)),
            .ray_cast = arr3(single.origin) ++ arr3(single.direction),
        };
        r_ray_cast.check(.{ r_origin, direction, columns, r_transform_translation, r_translation, fraction }, rz, rj);

        var fj: FromResult = undefined;
        jolt.jolt_r_ray_cast_from_ray_cast(&origin, &direction, &fj.origin, &fj.direction);
        const from = RRayCast.fromRayCast(ray);
        from_ray_cast.check(.{ origin, direction }, FromResult{ .origin = arrR3(from.origin), .direction = arr3(from.direction) }, fj);

        // AABoxCast (an aggregate of an AABox and a direction, also with an inverted box)
        var aj: [9]f32 = undefined;
        jolt.jolt_aabox_cast(&origin, &translation, &direction, &aj);
        const cast: AABoxCast = .{ .box = .init(vec3(origin), vec3(translation)), .direction = vec3(direction) };
        aabox_cast.check(.{ origin, translation, direction }, arr3(cast.box.min) ++ arr3(cast.box.max) ++ arr3(cast.direction), aj);
    }

    try finishAll(&.{ &ray_cast, &r_ray_cast, &from_ray_cast, &aabox_cast });
}

test "CollideShapeResult.reversed parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x3e3e3e3e } };
    var reversed: Checker = .{ .name = "CollideShapeResult.reversed" };

    const Result = struct { csr: CSR, early_out: [2]f32 };

    for (0..iterations / 4) |_| {
        var in = std.mem.zeroes(CSR);
        in.contact_point_on1 = gen.vec(-100, 100);
        in.contact_point_on2 = gen.vec(-100, 100);
        in.penetration_axis = gen.vec(-1, 1);
        in.penetration_depth = gen.float(-1, 1);
        in.sub_shape_id1 = gen.next();
        in.sub_shape_id2 = if (gen.oneIn(4)) SubShapeID.empty_value else gen.next();
        in.body_id2 = if (gen.oneIn(4)) BodyID.invalid_body_id else gen.next() & 0x7fffffff;
        in.face1_count = @intCast(gen.index(33));
        for (in.face1[0..in.face1_count]) |*v| v.* = gen.vec(-10, 10);
        in.face2_count = @intCast(gen.index(33));
        for (in.face2[0..in.face2_count]) |*v| v.* = gen.vec(-10, 10);

        var j: Result = .{ .csr = std.mem.zeroes(CSR), .early_out = undefined };
        jolt.jolt_collide_shape_result_reversed(&in, &j.csr, &j.early_out);
        const r = in.toResult();
        const rev = r.reversed();
        const z: Result = .{ .csr = .fromResult(&rev), .early_out = .{ r.getEarlyOutFraction(), rev.getEarlyOutFraction() } };
        reversed.check(in, z, j);
    }

    try finishAll(&.{&reversed});
}

test "Default values and constants parity" {
    var checker: Checker = .{ .name = "default values" };

    // CollideShapeSettings / CollideSettingsBase
    {
        var enums: [5]u32 = undefined;
        var floats: [12]f32 = undefined;
        jolt.jolt_collide_shape_settings_defaults(&enums, &floats);
        const s: CollideShapeSettings = .{};
        const b: CollideSettingsBase = .{};
        const z_enums = [5]u32{ @intFromEnum(s.active_edge_mode), @intFromEnum(s.collect_faces_mode), @intFromEnum(s.back_face_mode), @intFromEnum(b.active_edge_mode), @intFromEnum(b.collect_faces_mode) };
        const z_floats = [_]f32{ s.collision_tolerance, s.penetration_tolerance } ++ arr3(s.active_edge_movement_direction) ++ [_]f32{ s.max_separation_distance, s.internal_edge_removal_vertex_tolerance_sq, b.collision_tolerance, b.penetration_tolerance } ++ arr3(b.active_edge_movement_direction);
        const S = struct { enums: [5]u32, floats: [12]f32 };
        checker.check("CollideShapeSettings", S{ .enums = z_enums, .floats = z_floats }, S{ .enums = enums, .floats = floats });
    }

    // RayCastSettings
    {
        var values: [5]u32 = undefined;
        jolt.jolt_ray_cast_settings_defaults(&values);
        var s: RayCastSettings = .{};
        var z: [5]u32 = .{ @intFromEnum(s.back_face_mode_triangles), @intFromEnum(s.back_face_mode_convex), @intFromBool(s.treat_convex_as_solid), 0, 0 };
        s.setBackFaceMode(.collide_with_back_faces);
        z[3] = @intFromEnum(s.back_face_mode_triangles);
        z[4] = @intFromEnum(s.back_face_mode_convex);
        checker.check("RayCastSettings", z, values);
    }

    // Results, SubShapeIDPair, SubShapeIDCreator
    {
        var ids: [18]u32 = undefined;
        var fractions: [5]f32 = undefined;
        jolt.jolt_result_defaults(&ids, &fractions);
        const broad_phase: BroadPhaseCastResult = .{};
        const ray: RayCastResult = .{};
        var ray2: RayCastResult = .{ .body_id = .init(5), .fraction = 0.5, .sub_shape_id2 = .{ .value = 7 } };
        ray2.reset();
        const point: CollidePointResult = .{};
        const shape: CollideShapeResult = .{};
        const pair: SubShapeIDPair = .{};
        const creator: SubShapeIDCreator = .{};
        const z_ids = [18]u32{
            broad_phase.body_id.getIndexAndSequenceNumber(),
            ray.body_id.getIndexAndSequenceNumber(),
            ray.sub_shape_id2.getValue(),
            ray2.body_id.getIndexAndSequenceNumber(),
            ray2.sub_shape_id2.getValue(),
            point.body_id.getIndexAndSequenceNumber(),
            point.sub_shape_id2.getValue(),
            shape.sub_shape_id1.getValue(),
            shape.sub_shape_id2.getValue(),
            shape.body_id2.getIndexAndSequenceNumber(),
            shape.shape1_face.len,
            shape.shape2_face.len,
            pair.getBody1ID().getIndexAndSequenceNumber(),
            pair.getSubShapeID1().getValue(),
            pair.getBody2ID().getIndexAndSequenceNumber(),
            pair.getSubShapeID2().getValue(),
            creator.getID().getValue(),
            creator.getNumBitsWritten(),
        };
        const z_fractions = [5]f32{ broad_phase.fraction, broad_phase.getEarlyOutFraction(), ray.fraction, ray2.fraction, point.getEarlyOutFraction() };
        const S = struct { ids: [18]u32, fractions: [5]f32 };
        checker.check("results", S{ .ids = z_ids, .fractions = z_fractions }, S{ .ids = ids, .fractions = fractions });
    }

    // Layers
    {
        var values: [4]u32 = undefined;
        jolt.jolt_layer_constants(&values);
        const z = [4]u32{ zolt.object_layer_invalid, @sizeOf(zolt.ObjectLayer), zolt.broad_phase_layer_invalid.getValue(), @sizeOf(zolt.BroadPhaseLayer) };
        checker.check("layers", z, values);
    }

    // PhysicsSettings
    {
        var constants: [5]f32 = undefined;
        var int_constants: [2]i32 = undefined;
        var ints: [3]i32 = undefined;
        var floats: [15]f32 = undefined;
        var uints: [2]u32 = undefined;
        var bools: [6]c_int = undefined;
        jolt.jolt_physics_settings(&constants, &int_constants, &ints, &floats, &uints, &bools);
        const s: PhysicsSettings = .{};
        const z_constants = [5]f32{ physics_settings.default_collision_tolerance, physics_settings.default_penetration_tolerance, physics_settings.default_convex_radius, physics_settings.capsule_projection_slop, physics_settings.default_internal_edge_removal_vertex_tolerance_sq };
        const z_int_constants = [2]i32{ physics_settings.max_physics_jobs, physics_settings.max_physics_barriers };
        const z_ints = [3]i32{ s.max_in_flight_body_pairs, s.step_listeners_batch_size, s.step_listener_batches_per_job };
        const z_floats = [15]f32{ s.baumgarte, s.speculative_contact_distance, s.penetration_slop, s.linear_cast_threshold, s.linear_cast_max_penetration, s.manifold_tolerance, s.max_penetration_distance, s.body_pair_cache_max_delta_position_sq, s.body_pair_cache_cos_max_delta_rotation_div2, s.contact_normal_cos_max_delta_rotation, s.contact_point_preserve_lambda_max_dist_sq, s.internal_edge_removal_vertex_tolerance_sq, s.min_velocity_for_restitution, s.time_before_sleep, s.point_velocity_sleep_threshold };
        const z_uints = [2]u32{ s.num_velocity_steps, s.num_position_steps };
        const z_bools = [6]c_int{ @intFromBool(s.constraint_warm_start), @intFromBool(s.use_body_pair_contact_cache), @intFromBool(s.use_manifold_reduction), @intFromBool(s.use_large_island_splitter), @intFromBool(s.allow_sleeping), @intFromBool(s.check_active_edges) };
        const S = struct { constants: [5]f32, int_constants: [2]i32, ints: [3]i32, floats: [15]f32, uints: [2]u32, bools: [6]c_int };
        checker.check("PhysicsSettings", S{ .constants = z_constants, .int_constants = z_int_constants, .ints = z_ints, .floats = z_floats, .uints = z_uints, .bools = z_bools }, S{ .constants = constants, .int_constants = int_constants, .ints = ints, .floats = floats, .uints = uints, .bools = bools });
    }

    try finishAll(&.{&checker});
}

test "SortReverseAndStore / CountAndSortTrues parity" {
    var gen: Gen = .{ .rng = .{ .state = 0x5011dea1 } };
    var sort: Checker = .{ .name = "sortReverseAndStore" };
    var count: Checker = .{ .name = "countAndSortTrues" };

    const values_pool = [_]f32{ 0.0, -0.0, 1.0, 2.0, std.math.floatMax(f32), -std.math.floatMax(f32), std.math.inf(f32), 1.0e-30 };
    const SortResult = struct { num: c_int, values: [4]f32, identifiers: [4]u32 };
    const CountResult = struct { num: c_int, identifiers: [4]u32 };

    for (0..iterations) |_| {
        // Values with ties (small grid, -0 / +0, FLT_MAX for a miss, infinity)
        var values: [4]f32 = undefined;
        for (&values) |*v| v.* = switch (gen.next() % 3) {
            0 => gen.grid(2),
            1 => values_pool[gen.index(values_pool.len)],
            else => gen.float(-10, 10),
        };
        const max_value = switch (gen.next() % 3) {
            0 => values[gen.index(4)], // Equal to one of the values (strict less)
            1 => gen.grid(3),
            else => gen.float(-20, 20),
        };
        var identifiers: [4]u32 = .{ gen.next(), gen.next(), gen.next(), gen.next() };
        if (gen.oneIn(2)) identifiers = .{ 0, 1, 2, 3 };

        var j: SortResult = .{ .num = 0, .values = @splat(0), .identifiers = identifiers };
        j.num = jolt.jolt_sort_reverse_and_store(&values, max_value, &j.identifiers, &j.values);
        var z_identifiers = UVec4.init(identifiers[0], identifiers[1], identifiers[2], identifiers[3]);
        var z_values: [4]f32 = @splat(0);
        const z_num = zolt.sortReverseAndStore(vec4(values), max_value, &z_identifiers, &z_values);
        sort.check(.{ values, max_value, identifiers }, SortResult{ .num = z_num, .values = z_values, .identifiers = .{ z_identifiers.getX(), z_identifiers.getY(), z_identifiers.getZ(), z_identifiers.getW() } }, j);

        // CountAndSortTrues: lanes that are true (all bits), false or arbitrary (only the top bit counts)
        var mask: [4]u32 = undefined;
        for (&mask) |*m| m.* = switch (gen.next() % 3) {
            0 => 0xffffffff,
            1 => 0,
            else => gen.next(),
        };
        var cj: CountResult = .{ .num = 0, .identifiers = identifiers };
        cj.num = jolt.jolt_count_and_sort_trues(&mask, &cj.identifiers);
        var c_identifiers = UVec4.init(identifiers[0], identifiers[1], identifiers[2], identifiers[3]);
        const c_num = zolt.countAndSortTrues(UVec4.init(mask[0], mask[1], mask[2], mask[3]), &c_identifiers);
        count.check(.{ mask, identifiers }, CountResult{ .num = c_num, .identifiers = .{ c_identifiers.getX(), c_identifiers.getY(), c_identifiers.getZ(), c_identifiers.getW() } }, cj);
    }

    try finishAll(&.{ &sort, &count });
}

/// Result of restoring a material, see jolt_physics_material_restore
const MaterialRestore = struct {
    valid: c_int,
    hash: u32,
    color: u32,
    text_length: u32,
    text: [128]u8,
};

fn zoltRestoreMaterial(bytes: []const u8) !MaterialRestore {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try PhysicsMaterial.restoreFromBinaryState(std.testing.allocator, in.streamIn());
    defer result.deinit();
    var r: MaterialRestore = .{ .valid = 0, .hash = 0, .color = 0, .text_length = 0, .text = @splat(0) };
    const text = if (result.isValid()) blk: {
        const material = result.getPtr().?;
        r.valid = 1;
        r.hash = material.getRTTIHash();
        r.color = material.getDebugColor().getUInt32();
        break :blk material.getDebugName(); // Names with 0 bytes: Jolt returns mDebugName.c_str(), which ends at the first one
    } else result.getError();
    r.text_length = @intCast(text.len);
    @memcpy(r.text[0..@min(text.len, r.text.len)], text[0..@min(text.len, r.text.len)]);
    return r;
}

fn jolt_restore_material(bytes: []const u8) MaterialRestore {
    var r: MaterialRestore = .{ .valid = 0, .hash = 0, .color = 0, .text_length = 0, .text = @splat(0) };
    r.valid = jolt.jolt_physics_material_restore(bytes.ptr, @intCast(bytes.len), &r.hash, &r.text, r.text.len, &r.text_length, &r.color);
    return r;
}

test "PhysicsMaterial / PhysicsMaterialSimple parity" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x6d617465 } };
    var rtti: Checker = .{ .name = "PhysicsMaterial RTTI hashes" };
    var default: Checker = .{ .name = "PhysicsMaterial.default" };
    var save: Checker = .{ .name = "PhysicsMaterial.saveBinaryState" };
    var restore: Checker = .{ .name = "PhysicsMaterial.restoreFromBinaryState" };

    // RTTI hashes
    {
        var j: [2]u32 = undefined;
        jolt.jolt_physics_material_rtti_hashes(&j);
        const z = [2]u32{ PhysicsMaterial.rttiHash(PhysicsMaterial.rtti_name), PhysicsMaterial.rttiHash(PhysicsMaterialSimple.rtti_name) };
        rtti.check("rtti", z, j);
    }

    // The default material
    {
        const Default = struct { name_length: u32, name: [32]u8, color: u32, hash: u32 };
        var j: Default = .{ .name_length = 0, .name = @splat(0), .color = 0, .hash = 0 };
        j.name_length = jolt.jolt_default_material(&j.name, j.name.len, &j.color, &j.hash);
        const material = PhysicsMaterial.default;
        var z: Default = .{ .name_length = @intCast(material.getDebugName().len), .name = @splat(0), .color = material.getDebugColor().getUInt32(), .hash = material.getRTTIHash() };
        @memcpy(z.name[0..material.getDebugName().len], material.getDebugName());
        default.check("default", z, j);
    }

    for (0..2000) |_| {
        // A material with a random name (any bytes, including 0) and color, or the base class
        var name: [40]u8 = undefined;
        const name_length = gen.index(name.len + 1);
        for (name[0..name_length]) |*c| c.* = if (gen.oneIn(2)) @intCast(gen.index(256)) else "abcXYZ"[gen.index(6)];
        const color: u32 = gen.next();
        const simple = !gen.oneIn(4);

        var j_bytes: [128]u8 = undefined;
        const j_size = jolt.jolt_physics_material_save(@intFromBool(simple), &name, @intCast(name_length), color, &j_bytes, j_bytes.len);

        const material: *PhysicsMaterial = if (simple)
            &(try PhysicsMaterialSimple.create(allocator, name[0..name_length], .fromUInt32(color))).base
        else
            try PhysicsMaterial.create(allocator);
        var ref = zolt.Ref(PhysicsMaterial).init(material);
        defer ref.deinit();
        var z_buffer: [128]u8 = undefined;
        var writer: std.Io.Writer = .fixed(&z_buffer);
        var out = StreamOutWrapper.init(&writer);
        material.saveBinaryState(out.streamOut());

        const Bytes = struct { size: u32, bytes: [128]u8 };
        var zb: Bytes = .{ .size = @intCast(writer.end), .bytes = @splat(0) };
        @memcpy(zb.bytes[0..writer.end], writer.buffered());
        var jb: Bytes = .{ .size = j_size, .bytes = @splat(0) };
        @memcpy(jb.bytes[0..j_size], j_bytes[0..j_size]);
        save.check(.{ simple, name_length, color }, zb, jb);

        // Restore: the full stream, a truncated stream or a corrupted type hash
        var bytes = j_bytes;
        var size = j_size;
        switch (gen.next() % 4) {
            0, 1 => {},
            2 => size = @intCast(gen.index(j_size + 1)),
            else => bytes[gen.index(4)] ^= @as(u8, 1) << @intCast(gen.index(8)),
        }
        restore.check(.{ simple, name_length, color, size }, try zoltRestoreMaterial(bytes[0..size]), jolt_restore_material(bytes[0..size]));
    }

    try finishAll(&.{ &rtti, &default, &save, &restore });
}
