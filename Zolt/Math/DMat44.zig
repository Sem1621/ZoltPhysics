//! Port of: Jolt/Math/DMat44.h, Jolt/Math/DMat44.inl
//! Status: complete
//!
//! Semantics follow the portable scalar fallback of DMat44.inl, written with vector operations that
//! do the same operations in the same order in every lane. Like DVec3, the translation column keeps
//! W equal to Z.
//!
//! Overloads: the functions that take the same arguments as their Mat44 counterpart keep Mat44's name
//! (`mulVec3(Vec3)`, `multiply3x3(Vec3)`, `preTranslated(Vec3)`, `postTranslated(Vec3)`), the overloads
//! that take a DVec3 get a `DVec3` suffix. Matrix * matrix is `mul(DMat44)` / `mulMat44(Mat44)`.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const DVec3 = @import("DVec3.zig").DVec3;
const Mat44 = @import("Mat44.zig").Mat44;
const Quat = @import("Quat.zig").Quat;
const Vec3 = @import("Vec3.zig").Vec3;
const Vec4 = @import("Vec4.zig").Vec4;

/// Holds a 4x4 matrix of floats with the last column consisting of doubles
pub const DMat44 = extern struct {
    /// Underlying column type
    pub const Type = Vec4.Type;
    pub const DType = DVec3.Type;

    /// Argument type (DMat44Arg). Zig decides itself how to pass parameters, so this is just DMat44.
    pub const ArgType = DMat44;

    /// Rotation columns (mCol)
    col: [3]Vec4,

    /// Translation column, 4th element is assumed to be 1 (mCol3)
    col3: DVec3,

    comptime {
        // alignas(max(JPH_VECTOR_ALIGNMENT, JPH_DVECTOR_ALIGNMENT))
        std.debug.assert(@alignOf(DMat44) == @max(Core.vector_alignment, Core.dvector_alignment));
        std.debug.assert(@sizeOf(DMat44) == 96);
    }

    /// Constructor
    pub fn init(c1: Vec4, c2: Vec4, c3: Vec4, c4: DVec3) DMat44 {
        return .{ .col = .{ c1, c2, c3 }, .col3 = c4 };
    }

    /// Constructor from raw SIMD values, W of c4 is replaced by Z (DMat44(Type, Type, Type, DTypeArg))
    pub fn fromTypes(c1: Type, c2: Type, c3: Type, c4: DType) DMat44 {
        return .{ .col = .{ .{ .value = c1 }, .{ .value = c2 }, .{ .value = c3 } }, .col3 = DVec3.fromType(c4) };
    }

    /// Convert from a Mat44, the translation is converted to doubles (explicit DMat44(Mat44Arg))
    pub fn fromMat44(m: Mat44) DMat44 {
        return init(m.getColumn4(0), m.getColumn4(1), m.getColumn4(2), DVec3.fromVec3(m.getTranslation()));
    }

    /// Construct from the rotation part of a Mat44 and a translation (DMat44(Mat44Arg inRot, DVec3Arg inT))
    pub fn fromMat44Translation(rot: Mat44, t: DVec3) DMat44 {
        return init(rot.getColumn4(0), rot.getColumn4(1), rot.getColumn4(2), t);
    }

    /// Zero matrix
    pub fn zero() DMat44 {
        return init(Vec4.zero(), Vec4.zero(), Vec4.zero(), DVec3.zero());
    }

    /// Identity matrix
    pub fn identity() DMat44 {
        return init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), DVec3.zero());
    }

    /// Rotate from quaternion (sRotation(QuatArg))
    pub fn rotationQuat(quat: Quat) DMat44 {
        return fromMat44Translation(Mat44.rotationQuat(quat), DVec3.zero());
    }

    /// Get matrix that translates
    pub fn translation(v: DVec3) DMat44 {
        return init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), v);
    }

    /// Get matrix that rotates and translates
    pub fn rotationTranslation(r: Quat, t: DVec3) DMat44 {
        return fromMat44Translation(Mat44.rotationQuat(r), t);
    }

    /// Get inverse matrix of rotationTranslation
    pub fn inverseRotationTranslation(r: Quat, t: DVec3) DMat44 {
        const m = Mat44.rotationQuat(r.conjugated());
        var dm = fromMat44Translation(m, DVec3.zero());
        dm.setTranslation(dm.multiply3x3DVec3(t).negate());
        return dm;
    }

    /// Get matrix that scales (produces a matrix with (v, 1) on its diagonal) (sScale(Vec3Arg))
    pub fn scaleVec3(v: Vec3) DMat44 {
        return fromMat44Translation(Mat44.scaleVec3(v), DVec3.zero());
    }

    /// Convert to Mat44 rounding to nearest
    pub fn toMat44(self: DMat44) Mat44 {
        return Mat44.fromColumnsTranslation(self.col[0], self.col[1], self.col[2], self.col3.toVec3());
    }

    /// Comparison (operator ==), `!=` is `!a.eql(b)`
    pub fn eql(self: DMat44, other: DMat44) bool {
        return self.col[0].eql(other.col[0]) and
            self.col[1].eql(other.col[1]) and
            self.col[2].eql(other.col[2]) and
            self.col3.eql(other.col3);
    }

    /// Test if two matrices are close
    pub fn isClose(self: DMat44, other: DMat44, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        for (0..3) |i|
            if (!self.col[i].isClose(other.col[i], .{ .max_dist_sq = opts.max_dist_sq }))
                return false;
        return self.col3.isClose(other.col3, .{ .max_dist_sq = @as(f64, opts.max_dist_sq) });
    }

    /// Multiply matrix by matrix (operator * (Mat44Arg))
    pub fn mulMat44(self: DMat44, m: Mat44) DMat44 {
        var result: DMat44 = undefined;

        // Rotation part
        for (0..3) |i|
            result.col[i] = self.rotate3x3Float(m.col[i]);

        // Translation part
        result.col3 = self.mulVec3(m.getTranslation());

        return result;
    }

    /// Multiply matrix by matrix (operator * (DMat44Arg))
    pub fn mul(self: DMat44, m: DMat44) DMat44 {
        var result: DMat44 = undefined;

        // Rotation part
        for (0..3) |i|
            result.col[i] = self.rotate3x3Float(m.col[i]);

        // Translation part
        result.col3 = self.mulDVec3(m.getTranslation());

        return result;
    }

    /// Multiply vector by matrix (operator * (Vec3Arg))
    pub fn mulVec3(self: DMat44, v: Vec3) DVec3 {
        // Per component: col3[r] + double(col[0][r] * v.x + col[1][r] * v.y + col[2][r] * v.z), the 3x3 part is calculated in floats
        const t = self.col[0].mul(v.splatX()).add(self.col[1].mul(v.splatY())).add(self.col[2].mul(v.splatZ()));
        return self.col3.addVec3(Vec3.fromVec4(t));
    }

    /// Multiply vector by matrix (operator * (DVec3Arg))
    pub fn mulDVec3(self: DMat44, v: DVec3) DVec3 {
        // Per component: col3[r] + double(col[0][r]) * v.x + double(col[1][r]) * v.y + double(col[2][r]) * v.z
        const c = self.columnsAsDouble();
        return DVec3.fromType(self.col3.value + c[0] * splat(v.value[0]) + c[1] * splat(v.value[1]) + c[2] * splat(v.value[2]));
    }

    /// Multiply vector by only 3x3 part of the matrix (Multiply3x3(Vec3Arg))
    pub fn multiply3x3(self: DMat44, v: Vec3) Vec3 {
        return self.getRotation().multiply3x3(v);
    }

    /// Multiply vector by only 3x3 part of the matrix (Multiply3x3(DVec3Arg))
    pub fn multiply3x3DVec3(self: DMat44, v: DVec3) DVec3 {
        // Per component: double(col[0][r]) * v.x + double(col[1][r]) * v.y + double(col[2][r]) * v.z
        const c = self.columnsAsDouble();
        return DVec3.fromType(c[0] * splat(v.value[0]) + c[1] * splat(v.value[1]) + c[2] * splat(v.value[2]));
    }

    /// Multiply vector by only 3x3 part of the transpose of the matrix (\f$result = this^T \: v\f$)
    pub fn multiply3x3Transposed(self: DMat44, v: Vec3) Vec3 {
        return self.getRotation().multiply3x3Transposed(v);
    }

    /// Scale a matrix: result = this * Mat44.scaleVec3(scale_value)
    pub fn preScaled(self: DMat44, scale_value: Vec3) DMat44 {
        return init(self.col[0].mulScalar(scale_value.getX()), self.col[1].mulScalar(scale_value.getY()), self.col[2].mulScalar(scale_value.getZ()), self.col3);
    }

    /// Scale a matrix: result = Mat44.scaleVec3(scale_value) * this
    pub fn postScaled(self: DMat44, scale_value: Vec3) DMat44 {
        const scale4 = Vec4.fromVec3W(scale_value, 1);
        return init(scale4.mul(self.col[0]), scale4.mul(self.col[1]), scale4.mul(self.col[2]), DVec3.fromVec3(scale_value).mul(self.col3));
    }

    /// Pre multiply by translation matrix: result = this * Mat44.translation(translation_value) (PreTranslated(Vec3Arg))
    pub fn preTranslated(self: DMat44, translation_value: Vec3) DMat44 {
        return init(self.col[0], self.col[1], self.col[2], self.getTranslation().addVec3(self.multiply3x3(translation_value)));
    }

    /// Pre multiply by translation matrix: result = this * Mat44.translation(translation_value) (PreTranslated(DVec3Arg))
    pub fn preTranslatedDVec3(self: DMat44, translation_value: DVec3) DMat44 {
        return init(self.col[0], self.col[1], self.col[2], self.getTranslation().add(self.multiply3x3DVec3(translation_value)));
    }

    /// Post multiply by translation matrix: result = Mat44.translation(translation_value) * this (i.e. add translation_value to the 4-th column) (PostTranslated(Vec3Arg))
    pub fn postTranslated(self: DMat44, translation_value: Vec3) DMat44 {
        return init(self.col[0], self.col[1], self.col[2], self.getTranslation().addVec3(translation_value));
    }

    /// Post multiply by translation matrix: result = Mat44.translation(translation_value) * this (i.e. add translation_value to the 4-th column) (PostTranslated(DVec3Arg))
    pub fn postTranslatedDVec3(self: DMat44, translation_value: DVec3) DMat44 {
        return init(self.col[0], self.col[1], self.col[2], self.getTranslation().add(translation_value));
    }

    /// Access to the columns
    pub fn getAxisX(self: DMat44) Vec3 {
        return Vec3.fromVec4(self.col[0]);
    }
    pub fn setAxisX(self: *DMat44, v: Vec3) void {
        self.col[0] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getAxisY(self: DMat44) Vec3 {
        return Vec3.fromVec4(self.col[1]);
    }
    pub fn setAxisY(self: *DMat44, v: Vec3) void {
        self.col[1] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getAxisZ(self: DMat44) Vec3 {
        return Vec3.fromVec4(self.col[2]);
    }
    pub fn setAxisZ(self: *DMat44, v: Vec3) void {
        self.col[2] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getTranslation(self: DMat44) DVec3 {
        return self.col3;
    }
    pub fn setTranslation(self: *DMat44, v: DVec3) void {
        self.col3 = v;
    }
    pub fn getColumn3(self: DMat44, column: u32) Vec3 {
        std.debug.assert(column < 3);
        return Vec3.fromVec4(self.col[column]);
    }
    pub fn setColumn3(self: *DMat44, column: u32, v: Vec3) void {
        std.debug.assert(column < 3);
        self.col[column] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getColumn4(self: DMat44, column: u32) Vec4 {
        std.debug.assert(column < 3);
        return self.col[column];
    }
    pub fn setColumn4(self: *DMat44, column: u32, v: Vec4) void {
        std.debug.assert(column < 3);
        self.col[column] = v;
    }

    /// Transpose 3x3 subpart of matrix
    pub fn transposed3x3(self: DMat44) Mat44 {
        return self.getRotation().transposed3x3();
    }

    /// Inverse 4x4 matrix
    pub fn inversed(self: DMat44) DMat44 {
        var m = fromMat44(self.getRotation().inversed3x3());
        m.col3 = m.multiply3x3DVec3(self.col3).negate();
        return m;
    }

    /// Inverse 4x4 matrix when it only contains rotation and translation
    pub fn inversedRotationTranslation(self: DMat44) DMat44 {
        var m = fromMat44(self.getRotation().transposed3x3());
        m.col3 = m.multiply3x3DVec3(self.col3).negate();
        return m;
    }

    /// Get rotation part only (note: retains the first 3 values from the bottom row)
    pub fn getRotation(self: DMat44) Mat44 {
        return Mat44.init(self.col[0], self.col[1], self.col[2], Vec4.init(0, 0, 0, 1));
    }

    /// Updates the rotation part of this matrix (the first 3 columns)
    pub fn setRotation(self: *DMat44, rotation_value: Mat44) void {
        self.col[0] = rotation_value.getColumn4(0);
        self.col[1] = rotation_value.getColumn4(1);
        self.col[2] = rotation_value.getColumn4(2);
    }

    /// Convert to quaternion
    pub fn getQuaternion(self: DMat44) Quat {
        return self.getRotation().getQuaternion();
    }

    /// Get matrix that transforms a direction with the same transform as this matrix (length is not preserved)
    pub fn getDirectionPreservingMatrix(self: DMat44) Mat44 {
        return self.getRotation().inversed3x3().transposed3x3();
    }

    /// Result of `decompose`
    pub const Decomposition = struct {
        /// The rotation and translation part
        rotation_translation: DMat44,
        /// The scale part (outScale)
        scale: Vec3,
    };

    /// Works identical to Mat44.decompose
    pub fn decompose(self: DMat44) Decomposition {
        const d = self.getRotation().decompose();
        return .{ .rotation_translation = fromMat44Translation(d.rotation_translation, self.col3), .scale = d.scale };
    }

    /// To String
    pub fn format(self: DMat44, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{f}, {f}, {f}, {f}", .{ self.col[0], self.col[1], self.col[2], self.col3 });
    }

    // Zolt helpers (no Jolt equivalent)

    /// col[0] * v.x + col[1] * v.y + col[2] * v.z in floats, the rotation part of a matrix multiplication
    fn rotate3x3Float(self: DMat44, v: Vec4) Vec4 {
        return self.col[0].mulScalar(v.getX()).add(self.col[1].mulScalar(v.getY())).add(self.col[2].mulScalar(v.getZ()));
    }

    /// The rotation columns converted to doubles (double(mCol[i].mF32[r]))
    fn columnsAsDouble(self: DMat44) [3]DType {
        return .{ @floatCast(self.col[0].value), @floatCast(self.col[1].value), @floatCast(self.col[2].value) };
    }

    /// Replicate a double across all components
    fn splat(v: f64) DType {
        return @splat(v);
    }
};
