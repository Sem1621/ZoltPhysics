//! Port of: Jolt/Math/Mat44.h, Jolt/Math/Mat44.inl
//! Status: complete

const std = @import("std");
const Core = @import("../Core/Core.zig");
const trigonometry = @import("Trigonometry.zig");
const Float4 = @import("Float4.zig").Float4;
const UVec4 = @import("UVec4.zig").UVec4;
const Vec3 = @import("Vec3.zig").Vec3;
const Vec4 = @import("Vec4.zig").Vec4;
const Quat = @import("Quat.zig").Quat;

/// Holds a 4x4 matrix of floats, but supports also operations on the 3x3 upper left part of the matrix.
pub const Mat44 = extern struct {
    /// Underlying column type
    pub const Type = Vec4.Type;

    /// Argument type (Mat44Arg). Zig decides itself how to pass parameters, so this is just Mat44.
    pub const ArgType = Mat44;

    /// Column (mCol)
    col: [4]Vec4,

    comptime {
        std.debug.assert(@sizeOf(Mat44) == 64);
        std.debug.assert(@alignOf(Mat44) == 16);
    }

    /// Constructor
    pub fn init(c1: Vec4, c2: Vec4, c3: Vec4, c4: Vec4) Mat44 {
        return .{ .col = .{ c1, c2, c3, c4 } };
    }

    /// Constructor, the 4th column is c4 with W = 1 (Mat44(Vec4Arg, Vec4Arg, Vec4Arg, Vec3Arg))
    pub fn fromColumnsTranslation(c1: Vec4, c2: Vec4, c3: Vec4, c4: Vec3) Mat44 {
        return .{ .col = .{ c1, c2, c3, Vec4.fromVec3W(c4, 1.0) } };
    }

    /// Constructor from raw SIMD values (Mat44(Type, Type, Type, Type))
    pub fn fromTypes(c1: Type, c2: Type, c3: Type, c4: Type) Mat44 {
        return .{ .col = .{ .{ .value = c1 }, .{ .value = c2 }, .{ .value = c3 }, .{ .value = c4 } } };
    }

    /// Zero matrix
    pub fn zero() Mat44 {
        return init(Vec4.zero(), Vec4.zero(), Vec4.zero(), Vec4.zero());
    }

    /// Identity matrix
    pub fn identity() Mat44 {
        return init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), Vec4.init(0, 0, 0, 1));
    }

    /// Matrix filled with NaN's
    pub fn nan() Mat44 {
        return init(Vec4.nan(), Vec4.nan(), Vec4.nan(), Vec4.nan());
    }

    /// Load 16 floats from memory
    pub fn loadFloat4x4(v: *const [4]Float4) Mat44 {
        var result: Mat44 = undefined;
        inline for (0..4) |c|
            result.col[c] = Vec4.loadFloat4(&v[c]);
        return result;
    }

    /// Load 16 floats from memory, 16 bytes aligned
    pub fn loadFloat4x4Aligned(v: *align(16) const [4]Float4) Mat44 {
        var result: Mat44 = undefined;
        inline for (0..4) |c|
            result.col[c] = Vec4.loadFloat4Aligned(&v[c]);
        return result;
    }

    /// Rotate around X axis (angle in radians)
    pub fn rotationX(x: f32) Mat44 {
        const sc = Vec4.replicate(x).sinCos();
        const s = sc.sin.getX();
        const c = sc.cos.getX();
        return init(Vec4.init(1, 0, 0, 0), Vec4.init(0, c, s, 0), Vec4.init(0, -s, c, 0), Vec4.init(0, 0, 0, 1));
    }

    /// Rotate around Y axis (angle in radians)
    pub fn rotationY(y: f32) Mat44 {
        const sc = Vec4.replicate(y).sinCos();
        const s = sc.sin.getX();
        const c = sc.cos.getX();
        return init(Vec4.init(c, 0, -s, 0), Vec4.init(0, 1, 0, 0), Vec4.init(s, 0, c, 0), Vec4.init(0, 0, 0, 1));
    }

    /// Rotate around Z axis (angle in radians)
    pub fn rotationZ(z: f32) Mat44 {
        const sc = Vec4.replicate(z).sinCos();
        const s = sc.sin.getX();
        const c = sc.cos.getX();
        return init(Vec4.init(c, s, 0, 0), Vec4.init(-s, c, 0, 0), Vec4.init(0, 0, 1, 0), Vec4.init(0, 0, 0, 1));
    }

    /// Rotate around arbitrary axis
    pub fn rotation(axis: Vec3, angle: f32) Mat44 {
        return rotationQuat(Quat.rotation(axis, angle));
    }

    /// Rotate from quaternion (sRotation(QuatArg))
    pub fn rotationQuat(quat: Quat) Mat44 {
        std.debug.assert(quat.isNormalized(.{}));

        // See: https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation section 'Quaternion-derived rotation matrix'
        const x = quat.getX();
        const y = quat.getY();
        const z = quat.getZ();
        const w = quat.getW();

        const tx = x + x; // Note: Using x + x instead of 2.0f * x to force this function to return the same value as the SSE4.1 version across platforms.
        const ty = y + y;
        const tz = z + z;

        const xx = tx * x;
        const yy = ty * y;
        const zz = tz * z;
        const xy = tx * y;
        const xz = tx * z;
        const xw = tx * w;
        const yz = ty * z;
        const yw = ty * w;
        const zw = tz * w;

        return init(Vec4.init((1.0 - yy) - zz, xy + zw, xz - yw, 0.0), // Note: Added extra brackets to force this function to return the same value as the SSE4.1 version across platforms.
            Vec4.init(xy - zw, (1.0 - zz) - xx, yz + xw, 0.0), Vec4.init(xz + yw, yz - xw, (1.0 - xx) - yy, 0.0), Vec4.init(0.0, 0.0, 0.0, 1.0));
    }

    /// Get matrix that translates
    pub fn translation(v: Vec3) Mat44 {
        return init(Vec4.init(1, 0, 0, 0), Vec4.init(0, 1, 0, 0), Vec4.init(0, 0, 1, 0), Vec4.fromVec3W(v, 1));
    }

    /// Get matrix that rotates and translates
    pub fn rotationTranslation(r: Quat, t: Vec3) Mat44 {
        var m = rotationQuat(r);
        m.setTranslation(t);
        return m;
    }

    /// Get inverse matrix of rotationTranslation
    pub fn inverseRotationTranslation(r: Quat, t: Vec3) Mat44 {
        var m = rotationQuat(r.conjugated());
        m.setTranslation(m.multiply3x3(t).negate());
        return m;
    }

    /// Get matrix that scales uniformly
    pub fn scale(s: f32) Mat44 {
        return init(Vec4.init(s, 0, 0, 0), Vec4.init(0, s, 0, 0), Vec4.init(0, 0, s, 0), Vec4.init(0, 0, 0, 1));
    }

    /// Get matrix that scales (produces a matrix with (v, 1) on its diagonal) (sScale(Vec3Arg))
    pub fn scaleVec3(v: Vec3) Mat44 {
        return init(Vec4.init(v.getX(), 0, 0, 0), Vec4.init(0, v.getY(), 0, 0), Vec4.init(0, 0, v.getZ(), 0), Vec4.init(0, 0, 0, 1));
    }

    /// Get outer product of v1 and v2 (equivalent to \f$v1 \otimes v2\f$)
    pub fn outerProduct(v1: Vec3, v2: Vec3) Mat44 {
        const v1_4 = Vec4.fromVec3W(v1, 0);
        return init(v1_4.mul(v2.splatX()), v1_4.mul(v2.splatY()), v1_4.mul(v2.splatZ()), Vec4.init(0, 0, 0, 1));
    }

    /// Get matrix that represents a cross product \f$A \times B = \text{crossProduct}(A) \: B\f$
    pub fn crossProduct(v: Vec3) Mat44 {
        const x = v.getX();
        const y = v.getY();
        const z = v.getZ();

        // Jolt's SSE4.1 path negates with 0 - v (a zero stays +0), its scalar fallback with -x (a zero becomes -0).
        // Zolt follows the SSE4.1 path, which is the reference of the parity tests (results only differ in the sign of zero).
        const min_v = v.negate();

        return init(
            Vec4.init(0, z, min_v.getY(), 0),
            Vec4.init(min_v.getZ(), 0, x, 0),
            Vec4.init(y, min_v.getX(), 0, 0),
            Vec4.init(0, 0, 0, 1),
        );
    }

    /// Returns matrix ML so that \f$ML(q) \: p = q \: p\f$ (where p and q are quaternions)
    pub fn quatLeftMultiply(q: Quat) Mat44 {
        return init(
            q.value.swizzle(.w, .z, .y, .x).flipSign(1, 1, -1, -1),
            q.value.swizzle(.z, .w, .x, .y).flipSign(-1, 1, 1, -1),
            q.value.swizzle(.y, .x, .w, .z).flipSign(1, -1, 1, -1),
            q.value,
        );
    }

    /// Returns matrix MR so that \f$MR(q) \: p = p \: q\f$ (where p and q are quaternions)
    pub fn quatRightMultiply(q: Quat) Mat44 {
        return init(
            q.value.swizzle(.w, .z, .y, .x).flipSign(1, -1, 1, -1),
            q.value.swizzle(.z, .w, .x, .y).flipSign(1, 1, -1, -1),
            q.value.swizzle(.y, .x, .w, .z).flipSign(-1, 1, 1, -1),
            q.value,
        );
    }

    /// Returns a look at matrix that transforms from world space to view space
    /// @param pos Position of the camera
    /// @param target Target of the camera
    /// @param up Up vector
    pub fn lookAt(pos: Vec3, target: Vec3, up: Vec3) Mat44 {
        const direction = target.sub(pos).normalizedOr(Vec3.axisZ().negate());
        const right = direction.cross(up).normalizedOr(Vec3.axisX());
        const new_up = right.cross(direction);

        return init(Vec4.fromVec3W(right, 0), Vec4.fromVec3W(new_up, 0), Vec4.fromVec3W(direction.negate(), 0), Vec4.fromVec3W(pos, 1)).inversedRotationTranslation();
    }

    /// Returns a right-handed perspective projection matrix
    pub fn perspective(fov_y: f32, aspect: f32, near: f32, far: f32) Mat44 {
        const height = 1.0 / trigonometry.tan(0.5 * fov_y);
        const width = height / aspect;
        const range = far / (near - far);

        return init(Vec4.init(width, 0.0, 0.0, 0.0), Vec4.init(0.0, height, 0.0, 0.0), Vec4.init(0.0, 0.0, range, -1.0), Vec4.init(0.0, 0.0, range * near, 0.0));
    }

    /// Get float component by element index (operator () (uint, uint) const)
    pub fn get(self: Mat44, row: u32, col: u32) f32 {
        std.debug.assert(row < 4);
        std.debug.assert(col < 4);
        return self.col[col].getComponent(row);
    }

    /// Set float component by element index (operator () (uint, uint), which returns a reference in Jolt)
    pub fn set(self: *Mat44, row: u32, col: u32, v: f32) void {
        std.debug.assert(row < 4);
        std.debug.assert(col < 4);
        self.col[col].setComponent(row, v);
    }

    /// Comparison (operator ==), `!=` is `!a.eql(b)`
    pub fn eql(self: Mat44, other: Mat44) bool {
        return UVec4.bitAnd(
            UVec4.bitAnd(Vec4.equals(self.col[0], other.col[0]), Vec4.equals(self.col[1], other.col[1])),
            UVec4.bitAnd(Vec4.equals(self.col[2], other.col[2]), Vec4.equals(self.col[3], other.col[3])),
        ).testAllTrue();
    }

    /// Test if two matrices are close
    pub fn isClose(self: Mat44, other: Mat44, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        for (0..4) |i|
            if (!self.col[i].isClose(other.col[i], .{ .max_dist_sq = opts.max_dist_sq }))
                return false;
        return true;
    }

    /// Multiply matrix by matrix (operator * (Mat44Arg))
    pub fn mul(self: Mat44, m: Mat44) Mat44 {
        var result: Mat44 = undefined;
        for (0..4) |i|
            result.col[i] = self.col[0].mulScalar(m.col[i].getX()).add(self.col[1].mulScalar(m.col[i].getY())).add(self.col[2].mulScalar(m.col[i].getZ())).add(self.col[3].mulScalar(m.col[i].getW()));
        return result;
    }

    /// Multiply vector by matrix (operator * (Vec3Arg))
    pub fn mulVec3(self: Mat44, v: Vec3) Vec3 {
        // Per component: col[0][r] * v.x + col[1][r] * v.y + col[2][r] * v.z + col[3][r]
        return Vec3.fromVec4(self.col[0].mul(v.splatX()).add(self.col[1].mul(v.splatY())).add(self.col[2].mul(v.splatZ())).add(self.col[3]));
    }

    /// Multiply vector by matrix (operator * (Vec4Arg))
    pub fn mulVec4(self: Mat44, v: Vec4) Vec4 {
        // Per component: col[0][r] * v.x + col[1][r] * v.y + col[2][r] * v.z + col[3][r] * v.w
        return self.col[0].mul(v.splatX()).add(self.col[1].mul(v.splatY())).add(self.col[2].mul(v.splatZ())).add(self.col[3].mul(v.splatW()));
    }

    /// Multiply vector by only 3x3 part of the matrix
    pub fn multiply3x3(self: Mat44, v: Vec3) Vec3 {
        // Per component: col[0][r] * v.x + col[1][r] * v.y + col[2][r] * v.z
        return Vec3.fromVec4(self.col[0].mul(v.splatX()).add(self.col[1].mul(v.splatY())).add(self.col[2].mul(v.splatZ())));
    }

    /// Multiply vector by only 3x3 part of the transpose of the matrix (\f$result = this^T \: v\f$)
    pub fn multiply3x3Transposed(self: Mat44, v: Vec3) Vec3 {
        return self.transposed3x3().multiply3x3(v);
    }

    /// Multiply 3x3 matrix by 3x3 matrix (Multiply3x3(Mat44Arg))
    pub fn multiply3x3Mat44(self: Mat44, m: Mat44) Mat44 {
        std.debug.assert(self.col[0].getW() == 0.0);
        std.debug.assert(self.col[1].getW() == 0.0);
        std.debug.assert(self.col[2].getW() == 0.0);

        var result: Mat44 = undefined;
        for (0..3) |i|
            result.col[i] = self.col[0].mulScalar(m.col[i].getX()).add(self.col[1].mulScalar(m.col[i].getY())).add(self.col[2].mulScalar(m.col[i].getZ()));
        result.col[3] = Vec4.init(0, 0, 0, 1);
        return result;
    }

    /// Multiply transpose of 3x3 matrix by 3x3 matrix (\f$result = this^T \: m\f$)
    pub fn multiply3x3LeftTransposed(self: Mat44, m: Mat44) Mat44 {
        // Transpose left hand side
        const trans = self.transposed3x3();

        // Do 3x3 matrix multiply
        var result: Mat44 = undefined;
        result.col[0] = trans.col[0].mul(m.col[0].splatX()).add(trans.col[1].mul(m.col[0].splatY())).add(trans.col[2].mul(m.col[0].splatZ()));
        result.col[1] = trans.col[0].mul(m.col[1].splatX()).add(trans.col[1].mul(m.col[1].splatY())).add(trans.col[2].mul(m.col[1].splatZ()));
        result.col[2] = trans.col[0].mul(m.col[2].splatX()).add(trans.col[1].mul(m.col[2].splatY())).add(trans.col[2].mul(m.col[2].splatZ()));
        result.col[3] = Vec4.init(0, 0, 0, 1);
        return result;
    }

    /// Multiply 3x3 matrix by the transpose of a 3x3 matrix (\f$result = this \: m^T\f$)
    pub fn multiply3x3RightTransposed(self: Mat44, m: Mat44) Mat44 {
        std.debug.assert(self.col[0].getW() == 0.0);
        std.debug.assert(self.col[1].getW() == 0.0);
        std.debug.assert(self.col[2].getW() == 0.0);

        var result: Mat44 = undefined;
        result.col[0] = self.col[0].mul(m.col[0].splatX()).add(self.col[1].mul(m.col[1].splatX())).add(self.col[2].mul(m.col[2].splatX()));
        result.col[1] = self.col[0].mul(m.col[0].splatY()).add(self.col[1].mul(m.col[1].splatY())).add(self.col[2].mul(m.col[2].splatY()));
        result.col[2] = self.col[0].mul(m.col[0].splatZ()).add(self.col[1].mul(m.col[1].splatZ())).add(self.col[2].mul(m.col[2].splatZ()));
        result.col[3] = Vec4.init(0, 0, 0, 1);
        return result;
    }

    /// Multiply matrix with float (operator * (float), float * Mat44, `*=` is `m = m.mulScalar(v)`)
    pub fn mulScalar(self: Mat44, v: f32) Mat44 {
        const multiplier = Vec4.replicate(v);

        var result: Mat44 = undefined;
        for (0..4) |c|
            result.col[c] = self.col[c].mul(multiplier);
        return result;
    }

    /// Per element addition of matrix (operator +, `+=` is `m = m.add(m2)`)
    pub fn add(self: Mat44, m: Mat44) Mat44 {
        var result: Mat44 = undefined;
        for (0..4) |i|
            result.col[i] = self.col[i].add(m.col[i]);
        return result;
    }

    /// Negate (operator - ()), computed as 0 - m (JPH_CROSS_PLATFORM_DETERMINISTIC, see Vec4.negate)
    pub fn negate(self: Mat44) Mat44 {
        var result: Mat44 = undefined;
        for (0..4) |i|
            result.col[i] = self.col[i].negate();
        return result;
    }

    /// Per element subtraction of matrix (operator -)
    pub fn sub(self: Mat44, m: Mat44) Mat44 {
        var result: Mat44 = undefined;
        for (0..4) |i|
            result.col[i] = self.col[i].sub(m.col[i]);
        return result;
    }

    /// Access to the columns
    pub fn getAxisX(self: Mat44) Vec3 {
        return Vec3.fromVec4(self.col[0]);
    }
    pub fn setAxisX(self: *Mat44, v: Vec3) void {
        self.col[0] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getAxisY(self: Mat44) Vec3 {
        return Vec3.fromVec4(self.col[1]);
    }
    pub fn setAxisY(self: *Mat44, v: Vec3) void {
        self.col[1] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getAxisZ(self: Mat44) Vec3 {
        return Vec3.fromVec4(self.col[2]);
    }
    pub fn setAxisZ(self: *Mat44, v: Vec3) void {
        self.col[2] = Vec4.fromVec3W(v, 0.0);
    }
    pub fn getTranslation(self: Mat44) Vec3 {
        return Vec3.fromVec4(self.col[3]);
    }
    pub fn setTranslation(self: *Mat44, v: Vec3) void {
        self.col[3] = Vec4.fromVec3W(v, 1.0);
    }
    pub fn getDiagonal3(self: Mat44) Vec3 {
        return Vec3.init(self.col[0].getX(), self.col[1].getY(), self.col[2].getZ());
    }
    pub fn setDiagonal3(self: *Mat44, v: Vec3) void {
        self.col[0].setX(v.getX());
        self.col[1].setY(v.getY());
        self.col[2].setZ(v.getZ());
    }
    pub fn getDiagonal4(self: Mat44) Vec4 {
        return Vec4.init(self.col[0].getX(), self.col[1].getY(), self.col[2].getZ(), self.col[3].getW());
    }
    pub fn setDiagonal4(self: *Mat44, v: Vec4) void {
        self.col[0].setX(v.getX());
        self.col[1].setY(v.getY());
        self.col[2].setZ(v.getZ());
        self.col[3].setW(v.getW());
    }
    pub fn getColumn3(self: Mat44, column: u32) Vec3 {
        std.debug.assert(column < 4);
        return Vec3.fromVec4(self.col[column]);
    }
    pub fn setColumn3(self: *Mat44, column: u32, v: Vec3) void {
        std.debug.assert(column < 4);
        self.col[column] = Vec4.fromVec3W(v, if (column == 3) 1.0 else 0.0);
    }
    pub fn getColumn4(self: Mat44, column: u32) Vec4 {
        std.debug.assert(column < 4);
        return self.col[column];
    }
    pub fn setColumn4(self: *Mat44, column: u32, v: Vec4) void {
        std.debug.assert(column < 4);
        self.col[column] = v;
    }

    /// Store matrix to memory
    pub fn storeFloat4x4(self: Mat44, out: *[4]Float4) void {
        for (0..4) |c|
            self.col[c].storeFloat4(&out[c]);
    }

    /// Transpose matrix
    pub fn transposed(self: Mat44) Mat44 {
        var result: Mat44 = undefined;
        inline for (0..4) |r|
            result.col[r] = Vec4.init(self.col[0].value[r], self.col[1].value[r], self.col[2].value[r], self.col[3].value[r]);
        return result;
    }

    /// Transpose 3x3 subpart of matrix
    pub fn transposed3x3(self: Mat44) Mat44 {
        var result: Mat44 = undefined;
        inline for (0..3) |c|
            result.col[c] = Vec4.init(self.col[0].value[c], self.col[1].value[c], self.col[2].value[c], 0);
        result.col[3] = Vec4.init(0, 0, 0, 1);
        return result;
    }

    /// Inverse 4x4 matrix
    ///
    /// Zolt ports the algorithm of Jolt's SSE path (which its NEON and RVV paths reproduce exactly), not the
    /// scalar fallback: the fallback calculates the cofactors in a different order and divides by the determinant
    /// instead of multiplying by its reciprocal, so it doesn't produce the same bits as Jolt on x86 / ARM / RISC-V RVV.
    pub fn inversed(self: Mat44) Mat44 {
        // Algorithm from: http://download.intel.com/design/PentiumIII/sml/24504301.pdf
        // Streaming SIMD Extensions - Inverse of 4x4 Matrix
        // Adapted to load data using _mm_shuffle_ps instead of loading from memory
        // Replaced _mm_rcp_ps with _mm_div_ps for better accuracy
        const c = self.col;

        var tmp1 = shuffle4(c[0].value, c[1].value, 0, 1, 0, 1);
        var row1 = shuffle4(c[2].value, c[3].value, 0, 1, 0, 1);
        const row0 = shuffle4(tmp1, row1, 0, 2, 0, 2);
        row1 = shuffle4(row1, tmp1, 1, 3, 1, 3);
        tmp1 = shuffle4(c[0].value, c[1].value, 2, 3, 2, 3);
        var row3 = shuffle4(c[2].value, c[3].value, 2, 3, 2, 3);
        var row2 = shuffle4(tmp1, row3, 0, 2, 0, 2);
        row3 = shuffle4(row3, tmp1, 1, 3, 1, 3);

        tmp1 = row2 * row3;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        var minor0 = row1 * tmp1;
        var minor1 = row0 * tmp1;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor0 = row1 * tmp1 - minor0;
        minor1 = row0 * tmp1 - minor1;
        minor1 = shuffle4(minor1, minor1, 2, 3, 0, 1);

        tmp1 = row1 * row2;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        minor0 = row3 * tmp1 + minor0;
        var minor3 = row0 * tmp1;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor0 = minor0 - row3 * tmp1;
        minor3 = row0 * tmp1 - minor3;
        minor3 = shuffle4(minor3, minor3, 2, 3, 0, 1);

        tmp1 = shuffle4(row1, row1, 2, 3, 0, 1) * row3;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        row2 = shuffle4(row2, row2, 2, 3, 0, 1);
        minor0 = row2 * tmp1 + minor0;
        var minor2 = row0 * tmp1;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor0 = minor0 - row2 * tmp1;
        minor2 = row0 * tmp1 - minor2;
        minor2 = shuffle4(minor2, minor2, 2, 3, 0, 1);

        tmp1 = row0 * row1;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        minor2 = row3 * tmp1 + minor2;
        minor3 = row2 * tmp1 - minor3;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor2 = row3 * tmp1 - minor2;
        minor3 = minor3 - row2 * tmp1;

        tmp1 = row0 * row3;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        minor1 = minor1 - row2 * tmp1;
        minor2 = row1 * tmp1 + minor2;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor1 = row2 * tmp1 + minor1;
        minor2 = minor2 - row1 * tmp1;

        tmp1 = row0 * row2;
        tmp1 = shuffle4(tmp1, tmp1, 1, 0, 3, 2);
        minor1 = row3 * tmp1 + minor1;
        minor3 = minor3 - row1 * tmp1;
        tmp1 = shuffle4(tmp1, tmp1, 2, 3, 0, 1);
        minor1 = minor1 - row3 * tmp1;
        minor3 = row1 * tmp1 + minor3;

        // Determinant, summed as (x + y) + (z + w) (Jolt changed the order of the original code to match its ARM code and make the result cross platform deterministic)
        const det4 = row0 * minor0;
        const det = (det4[0] + det4[1]) + (det4[2] + det4[3]);
        const inv_det: Type = @splat(1.0 / det);

        return fromTypes(inv_det * minor0, inv_det * minor1, inv_det * minor2, inv_det * minor3);
    }

    /// Inverse 4x4 matrix when it only contains rotation and translation
    pub fn inversedRotationTranslation(self: Mat44) Mat44 {
        var m = self.transposed3x3();
        m.setTranslation(m.multiply3x3(self.getTranslation()).negate());
        return m;
    }

    /// Get the determinant of a 3x3 matrix
    pub fn getDeterminant3x3(self: Mat44) f32 {
        return self.getAxisX().dot(self.getAxisY().crossPrecise(self.getAxisZ()));
    }

    /// Get the adjoint of a 3x3 matrix
    pub fn adjointed3x3(self: Mat44) Mat44 {
        return init(
            Vec4.init(self.el(1, 1), self.el(1, 2), self.el(1, 0), 0).mul(Vec4.init(self.el(2, 2), self.el(2, 0), self.el(2, 1), 0))
                .sub(Vec4.init(self.el(1, 2), self.el(1, 0), self.el(1, 1), 0).mul(Vec4.init(self.el(2, 1), self.el(2, 2), self.el(2, 0), 0))),
            Vec4.init(self.el(0, 2), self.el(0, 0), self.el(0, 1), 0).mul(Vec4.init(self.el(2, 1), self.el(2, 2), self.el(2, 0), 0))
                .sub(Vec4.init(self.el(0, 1), self.el(0, 2), self.el(0, 0), 0).mul(Vec4.init(self.el(2, 2), self.el(2, 0), self.el(2, 1), 0))),
            Vec4.init(self.el(0, 1), self.el(0, 2), self.el(0, 0), 0).mul(Vec4.init(self.el(1, 2), self.el(1, 0), self.el(1, 1), 0))
                .sub(Vec4.init(self.el(0, 2), self.el(0, 0), self.el(0, 1), 0).mul(Vec4.init(self.el(1, 1), self.el(1, 2), self.el(1, 0), 0))),
            Vec4.init(0, 0, 0, 1),
        );
    }

    /// Inverse 3x3 matrix
    pub fn inversed3x3(self: Mat44) Mat44 {
        const det = self.getDeterminant3x3();

        return init(
            Vec4.init(self.el(1, 1), self.el(1, 2), self.el(1, 0), 0).mul(Vec4.init(self.el(2, 2), self.el(2, 0), self.el(2, 1), 0))
                .sub(Vec4.init(self.el(1, 2), self.el(1, 0), self.el(1, 1), 0).mul(Vec4.init(self.el(2, 1), self.el(2, 2), self.el(2, 0), 0))).divScalar(det),
            Vec4.init(self.el(0, 2), self.el(0, 0), self.el(0, 1), 0).mul(Vec4.init(self.el(2, 1), self.el(2, 2), self.el(2, 0), 0))
                .sub(Vec4.init(self.el(0, 1), self.el(0, 2), self.el(0, 0), 0).mul(Vec4.init(self.el(2, 2), self.el(2, 0), self.el(2, 1), 0))).divScalar(det),
            Vec4.init(self.el(0, 1), self.el(0, 2), self.el(0, 0), 0).mul(Vec4.init(self.el(1, 2), self.el(1, 0), self.el(1, 1), 0))
                .sub(Vec4.init(self.el(0, 2), self.el(0, 0), self.el(0, 1), 0).mul(Vec4.init(self.el(1, 1), self.el(1, 2), self.el(1, 0), 0))).divScalar(det),
            Vec4.init(0, 0, 0, 1),
        );
    }

    /// self = m.inversed3x3(), returns false if the matrix is singular in which case self is unchanged
    pub fn setInversed3x3(self: *Mat44, m: Mat44) bool {
        const det = m.getDeterminant3x3();

        // If the determinant is zero the matrix is singular and we return false
        if (det == 0.0)
            return false;

        // Finish calculating the inverse
        self.* = m.adjointed3x3();
        self.col[0] = self.col[0].divScalar(det);
        self.col[1] = self.col[1].divScalar(det);
        self.col[2] = self.col[2].divScalar(det);
        return true;
    }

    /// Get rotation part only (note: retains the first 3 values from the bottom row)
    pub fn getRotation(self: Mat44) Mat44 {
        std.debug.assert(self.col[0].getW() == 0.0);
        std.debug.assert(self.col[1].getW() == 0.0);
        std.debug.assert(self.col[2].getW() == 0.0);

        return init(self.col[0], self.col[1], self.col[2], Vec4.init(0, 0, 0, 1));
    }

    /// Get rotation part only (note: also clears the bottom row)
    pub fn getRotationSafe(self: Mat44) Mat44 {
        return init(
            Vec4.init(self.col[0].getX(), self.col[0].getY(), self.col[0].getZ(), 0),
            Vec4.init(self.col[1].getX(), self.col[1].getY(), self.col[1].getZ(), 0),
            Vec4.init(self.col[2].getX(), self.col[2].getY(), self.col[2].getZ(), 0),
            Vec4.init(0, 0, 0, 1),
        );
    }

    /// Updates the rotation part of this matrix (the first 3 columns)
    pub fn setRotation(self: *Mat44, rotation_value: Mat44) void {
        self.col[0] = rotation_value.col[0];
        self.col[1] = rotation_value.col[1];
        self.col[2] = rotation_value.col[2];
    }

    /// Convert to quaternion
    pub fn getQuaternion(self: Mat44) Quat {
        const c = self.col;
        const tr = c[0].value[0] + c[1].value[1] + c[2].value[2];

        if (tr >= 0.0) {
            const s = @sqrt(tr + 1.0);
            const is = 0.5 / s;
            return Quat.init(
                (c[1].value[2] - c[2].value[1]) * is,
                (c[2].value[0] - c[0].value[2]) * is,
                (c[0].value[1] - c[1].value[0]) * is,
                0.5 * s,
            );
        } else {
            var i: u32 = 0;
            if (c[1].value[1] > c[0].value[0]) i = 1;
            if (c[2].value[2] > c[i].getComponent(i)) i = 2;

            if (i == 0) {
                const s = @sqrt(c[0].value[0] - (c[1].value[1] + c[2].value[2]) + 1);
                const is = 0.5 / s;
                return Quat.init(
                    0.5 * s,
                    (c[1].value[0] + c[0].value[1]) * is,
                    (c[0].value[2] + c[2].value[0]) * is,
                    (c[1].value[2] - c[2].value[1]) * is,
                );
            } else if (i == 1) {
                const s = @sqrt(c[1].value[1] - (c[2].value[2] + c[0].value[0]) + 1);
                const is = 0.5 / s;
                return Quat.init(
                    (c[1].value[0] + c[0].value[1]) * is,
                    0.5 * s,
                    (c[2].value[1] + c[1].value[2]) * is,
                    (c[2].value[0] - c[0].value[2]) * is,
                );
            } else {
                std.debug.assert(i == 2);

                const s = @sqrt(c[2].value[2] - (c[0].value[0] + c[1].value[1]) + 1);
                const is = 0.5 / s;
                return Quat.init(
                    (c[0].value[2] + c[2].value[0]) * is,
                    (c[2].value[1] + c[1].value[2]) * is,
                    0.5 * s,
                    (c[0].value[1] - c[1].value[0]) * is,
                );
            }
        }
    }

    /// Get matrix that transforms a direction with the same transform as this matrix (length is not preserved)
    pub fn getDirectionPreservingMatrix(self: Mat44) Mat44 {
        return self.getRotation().inversed3x3().transposed3x3();
    }

    /// Pre multiply by translation matrix: result = this * Mat44.translation(translation_value)
    pub fn preTranslated(self: Mat44, translation_value: Vec3) Mat44 {
        return init(self.col[0], self.col[1], self.col[2], Vec4.fromVec3W(self.getTranslation().add(self.multiply3x3(translation_value)), 1));
    }

    /// Post multiply by translation matrix: result = Mat44.translation(translation_value) * this (i.e. add translation_value to the 4-th column)
    pub fn postTranslated(self: Mat44, translation_value: Vec3) Mat44 {
        return init(self.col[0], self.col[1], self.col[2], Vec4.fromVec3W(self.getTranslation().add(translation_value), 1));
    }

    /// Scale a matrix: result = this * Mat44.scaleVec3(scale_value)
    pub fn preScaled(self: Mat44, scale_value: Vec3) Mat44 {
        return init(self.col[0].mulScalar(scale_value.getX()), self.col[1].mulScalar(scale_value.getY()), self.col[2].mulScalar(scale_value.getZ()), self.col[3]);
    }

    /// Scale a matrix: result = Mat44.scaleVec3(scale_value) * this
    pub fn postScaled(self: Mat44, scale_value: Vec3) Mat44 {
        const scale4 = Vec4.fromVec3W(scale_value, 1);
        return init(scale4.mul(self.col[0]), scale4.mul(self.col[1]), scale4.mul(self.col[2]), scale4.mul(self.col[3]));
    }

    /// Result of `decompose`
    pub const Decomposition = struct {
        /// The rotation and translation part
        rotation_translation: Mat44,
        /// The scale part (outScale)
        scale: Vec3,
    };

    /// Decompose a matrix into a rotation & translation part and into a scale part so that:
    /// this = result.rotation_translation * Mat44.scaleVec3(result.scale).
    /// This equation only holds when the matrix is orthogonal, if it is not the returned matrix
    /// will be made orthogonal using the modified Gram-Schmidt algorithm (see: https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
    pub fn decompose(self: Mat44) Decomposition {
        // Start the modified Gram-Schmidt algorithm
        // X axis will just be normalized
        const x = self.getAxisX();

        // Make Y axis perpendicular to X
        var y = self.getAxisY();
        const x_dot_x = x.lengthSq();
        y = y.sub(x.mulScalar(x.dot(y) / x_dot_x));

        // Make Z axis perpendicular to X
        var z = self.getAxisZ();
        z = z.sub(x.mulScalar(x.dot(z) / x_dot_x));

        // Make Z axis perpendicular to Y
        const y_dot_y = y.lengthSq();
        z = z.sub(y.mulScalar(y.dot(z) / y_dot_y));

        // Determine the scale
        const z_dot_z = z.lengthSq();
        var out_scale = Vec3.init(x_dot_x, y_dot_y, z_dot_z).sqrt();

        // If the resulting x, y and z vectors don't form a right handed matrix, flip the z axis.
        if (x.cross(y).dot(z) < 0.0)
            out_scale.setZ(-out_scale.getZ());

        // Determine the rotation and translation
        return .{
            .rotation_translation = init(Vec4.fromVec3W(x.divScalar(out_scale.getX()), 0), Vec4.fromVec3W(y.divScalar(out_scale.getY()), 0), Vec4.fromVec3W(z.divScalar(out_scale.getZ()), 0), self.getColumn4(3)),
            .scale = out_scale,
        };
    }

    /// In single precision mode just return the matrix itself.
    /// Like in Jolt, this only exists when not compiling with double precision (in double precision mode RMat44 is DMat44, which has its own toMat44).
    pub const toMat44 = if (Core.double_precision) {} else toMat44SinglePrecision;

    fn toMat44SinglePrecision(self: Mat44) Mat44 {
        return self;
    }

    /// To String
    pub fn format(self: Mat44, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{f}, {f}, {f}, {f}", .{ self.col[0], self.col[1], self.col[2], self.col[3] });
    }

    /// Element at row r, column c with comptime indices (JPH_EL(r, c))
    fn el(self: Mat44, comptime r: usize, comptime c: usize) f32 {
        return self.col[c].value[r];
    }

    /// _mm_shuffle_ps(a, b, _MM_SHUFFLE(b1, b0, a1, a0)): returns [a[a0], a[a1], b[b0], b[b1]]
    fn shuffle4(a: Type, b: Type, comptime a0: i32, comptime a1: i32, comptime b0: i32, comptime b1: i32) Type {
        return @shuffle(f32, a, b, @Vector(4, i32){ a0, a1, ~b0, ~b1 });
    }
};
