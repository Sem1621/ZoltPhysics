//! Port of: Jolt/Math/Quat.h, Jolt/Math/Quat.inl
//! Status: complete

const std = @import("std");
const math = @import("Math.zig");
const trigonometry = @import("Trigonometry.zig");
const Float3 = @import("Float3.zig").Float3;
const Float4 = @import("Float4.zig").Float4;
const UVec4 = @import("UVec4.zig").UVec4;
const Vec3 = @import("Vec3.zig").Vec3;
const Vec4 = @import("Vec4.zig").Vec4;

/// Quaternion class, quaternions are 4 dimensional vectors which can describe rotations in 3 dimensional
/// space if their length is 1.
///
/// They are written as:
///
/// \f$q = w + x \: i + y \: j + z \: k\f$
///
/// or in vector notation:
///
/// \f$q = [w, v] = [w, x, y, z]\f$
///
/// Where:
///
/// w = the real part
/// v = the imaginary part, (x, y, z)
///
/// Note that we store the quaternion in a Vec4 as [x, y, z, w] because that makes
/// it easy to extract the rotation axis of the quaternion:
///
/// q = [cos(angle / 2), sin(angle / 2) * rotation_axis]
pub const Quat = extern struct {
    /// 4 vector that stores [x, y, z, w] parts of the quaternion
    value: Vec4,

    comptime {
        std.debug.assert(@sizeOf(Quat) == 16);
        std.debug.assert(@alignOf(Quat) == 16);
    }

    // Constructors

    /// Create a quaternion from its components
    pub fn init(x: f32, y: f32, z: f32, w: f32) Quat {
        return .{ .value = Vec4.init(x, y, z, w) };
    }

    /// Load from 4 floats (explicit Quat(const Float4 &))
    pub fn fromFloat4(v: Float4) Quat {
        return .{ .value = Vec4.loadFloat4(&v) };
    }

    /// Create from a Vec4 that stores [x, y, z, w] (explicit Quat(Vec4Arg))
    pub fn fromVec4(v: Vec4) Quat {
        return .{ .value = v };
    }

    // Tests

    /// Check if two quaternions are exactly equal (operator ==)
    pub fn eql(self: Quat, other: Quat) bool {
        return self.value.eql(other.value);
    }

    /// If this quaternion is close to other. Note that q and -q represent the same rotation, this is not checked here.
    pub fn isClose(self: Quat, other: Quat, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        return self.value.isClose(other.value, .{ .max_dist_sq = opts.max_dist_sq });
    }

    /// If the length^2 of this quaternion is within the range [1 - tolerance, 1 + tolerance]
    pub fn isNormalized(self: Quat, opts: struct { tolerance: f32 = 1.0e-5 }) bool {
        return self.value.isNormalized(.{ .tolerance = opts.tolerance });
    }

    /// If any component of this quaternion is a NaN (not a number)
    pub fn isNaN(self: Quat) bool {
        return self.value.isNaN();
    }

    // Get components

    /// Get X component (imaginary part i)
    pub fn getX(self: Quat) f32 {
        return self.value.getX();
    }

    /// Get Y component (imaginary part j)
    pub fn getY(self: Quat) f32 {
        return self.value.getY();
    }

    /// Get Z component (imaginary part k)
    pub fn getZ(self: Quat) f32 {
        return self.value.getZ();
    }

    /// Get W component (real part)
    pub fn getW(self: Quat) f32 {
        return self.value.getW();
    }

    /// Get the imaginary part of the quaternion
    pub fn getXYZ(self: Quat) Vec3 {
        return Vec3.fromVec4(self.value);
    }

    /// Get the quaternion as a Vec4
    pub fn getXYZW(self: Quat) Vec4 {
        return self.value;
    }

    /// Set individual components
    pub fn setX(self: *Quat, x: f32) void {
        self.value.setX(x);
    }
    pub fn setY(self: *Quat, y: f32) void {
        self.value.setY(y);
    }
    pub fn setZ(self: *Quat, z: f32) void {
        self.value.setZ(z);
    }
    pub fn setW(self: *Quat, w: f32) void {
        self.value.setW(w);
    }

    /// Set all components
    pub fn set(self: *Quat, x: f32, y: f32, z: f32, w: f32) void {
        self.value.set(x, y, z, w);
    }

    // Default quaternions

    /// @return [0, 0, 0, 0]
    pub fn zero() Quat {
        return fromVec4(Vec4.zero());
    }

    /// @return [1, 0, 0, 0] (or in storage format Quat(0, 0, 0, 1))
    pub fn identity() Quat {
        return init(0, 0, 0, 1);
    }

    /// Rotation from axis and angle
    pub fn rotation(axis: Vec3, angle: f32) Quat {
        // returns [axis * sin(0.5f * angle), cos(0.5f * angle)]
        std.debug.assert(axis.isNormalized(.{}));
        const sc = Vec4.replicate(0.5 * angle).sinCos();
        return fromVec4(Vec4.select(Vec4.fromVec3(axis).mul(sc.sin), sc.cos, UVec4.init(0, 0, 0, 0xffffffff)));
    }

    /// Result of `getAxisAngle`
    pub const AxisAngle = struct { axis: Vec3, angle: f32 };

    /// Get axis and angle that represents this quaternion, angle will always be in the range \f$[0, \pi]\f$
    pub fn getAxisAngle(self: Quat) AxisAngle {
        std.debug.assert(self.isNormalized(.{}));
        const w_pos = self.ensureWPositive();
        const abs_w = w_pos.getW();
        if (abs_w >= 1.0) {
            return .{ .axis = Vec3.zero(), .angle = 0.0 };
        } else {
            const angle = 2.0 * trigonometry.acos(abs_w);
            return .{ .axis = w_pos.getXYZ().normalizedOr(Vec3.zero()), .angle = angle };
        }
    }

    /// Calculate angular velocity given that this quaternion represents the rotation that is reached after delta_time when starting from identity rotation
    pub fn getAngularVelocity(self: Quat, delta_time: f32) Vec3 {
        std.debug.assert(self.isNormalized(.{}));

        // w = cos(angle / 2), ensure it is positive so that we get an angle in the range [0, PI]
        const w_pos = self.ensureWPositive();

        // The imaginary part of the quaternion is axis * sin(angle / 2),
        // if the length is small use the approximation sin(x) = x to calculate angular velocity
        const xyz = w_pos.getXYZ();
        const xyz_len_sq = xyz.lengthSq();
        if (xyz_len_sq < 4.0e-4) // Max error introduced is sin(0.02) - 0.02 = 7e-5 (when w is near 1 the angle becomes more inaccurate in the code below, so don't make this number too small)
            return xyz.mulScalar(2.0 / delta_time);

        // Otherwise calculate the angle from w = cos(angle / 2) and determine the axis by normalizing the imaginary part
        // Note that it is also possible to calculate the angle through angle = 2 * atan2(|xyz|, w). This is more accurate but also 2x as expensive.
        const angle = 2.0 * trigonometry.acos(w_pos.getW());
        return xyz.divScalar(@sqrt(xyz_len_sq) * delta_time).mulScalar(angle);
    }

    /// Create quaternion that rotates a vector from the direction of from to the direction of to along the shortest path
    /// @see https://www.euclideanspace.com/maths/algebra/vectors/angleBetween/index.htm
    pub fn fromTo(from: Vec3, to: Vec3) Quat {
        //  Uses (from = v1, to = v2):
        //
        //  angle = arcos(v1 . v2 / |v1||v2|)
        //  axis = normalize(v1 x v2)
        //
        //  Quaternion is then:
        //
        //  s = sin(angle / 2)
        //  x = axis.x * s
        //  y = axis.y * s
        //  z = axis.z * s
        //  w = cos(angle / 2)
        //
        //  Using identities:
        //
        //  sin(2 * a) = 2 * sin(a) * cos(a)
        //  cos(2 * a) = cos(a)^2 - sin(a)^2
        //  sin(a)^2 + cos(a)^2 = 1
        //
        //  This reduces to:
        //
        //  x = (v1 x v2).x
        //  y = (v1 x v2).y
        //  z = (v1 x v2).z
        //  w = |v1||v2| + v1 . v2
        //
        //  which then needs to be normalized because the whole equation was multiplied by 2 cos(angle / 2)

        const len_v1_v2 = @sqrt(from.lengthSq() * to.lengthSq());
        const w = len_v1_v2 + from.dot(to);

        if (w == 0.0) {
            if (len_v1_v2 == 0.0) {
                // If either of the vectors has zero length, there is no rotation and we return identity
                return identity();
            } else {
                // If vectors are perpendicular, take one of the many 180 degree rotations that exist
                return fromVec4(Vec4.fromVec3W(from.getNormalizedPerpendicular(), 0));
            }
        }

        const v = from.cross(to);
        return fromVec4(Vec4.fromVec3W(v, w)).normalized();
    }

    /// Random unit quaternion.
    /// `rng` is a pointer to a random bit generator (see Core/Mt19937.zig), the equivalent of a C++
    /// UniformRandomBitGenerator: it needs `next() -> u32` and `min_value` / `max_value` declarations.
    pub fn random(rng: anytype) Quat {
        const R = @TypeOf(rng.*);
        const range: f32 = @floatFromInt(R.max_value - R.min_value);

        // Using Uniform Random Rotations - Graphics Gems III - Ken Shoemake
        const x0 = @as(f32, @floatFromInt(rng.next() - R.min_value)) / range;
        const r1 = @sqrt(1.0 - x0);
        const r2 = @sqrt(x0);
        const theta1 = 2.0 * math.pi * @as(f32, @floatFromInt(rng.next() - R.min_value)) / range;
        const theta2 = 2.0 * math.pi * @as(f32, @floatFromInt(rng.next() - R.min_value)) / range;
        const sc = Vec4.init(theta1, theta2, 0, 0).sinCos();
        return init(sc.sin.getX() * r1, sc.cos.getX() * r1, sc.sin.getY() * r2, sc.cos.getY() * r2);
    }

    /// Conversion from Euler angles. Rotation order is X then Y then Z (RotZ * RotY * RotX). Angles in radians.
    pub fn eulerAngles(angles: Vec3) Quat {
        const half = Vec4.fromVec3(angles.mulScalar(0.5));
        const sc = half.sinCos();

        const cx = sc.cos.getX();
        const sx = sc.sin.getX();
        const cy = sc.cos.getY();
        const sy = sc.sin.getY();
        const cz = sc.cos.getZ();
        const sz = sc.sin.getZ();

        return init(
            cz * sx * cy - sz * cx * sy,
            cz * cx * sy + sz * sx * cy,
            sz * cx * cy - cz * sx * sy,
            cz * cx * cy + sz * sx * sy,
        );
    }

    /// Conversion to Euler angles. Rotation order is X then Y then Z (RotZ * RotY * RotX). Angles in radians.
    pub fn getEulerAngles(self: Quat) Vec3 {
        const x = self.getX();
        const y = self.getY();
        const z = self.getZ();
        const w = self.getW();
        const y_sq = y * y;

        // X
        const t0 = 2.0 * (w * x + y * z);
        const t1 = 1.0 - 2.0 * (x * x + y_sq);

        // Y
        var t2 = 2.0 * (w * y - z * x);
        t2 = if (t2 > 1.0) 1.0 else t2;
        t2 = if (t2 < -1.0) -1.0 else t2;

        // Z
        const t3 = 2.0 * (w * z + x * y);
        const t4 = 1.0 - 2.0 * (y_sq + z * z);

        return Vec3.init(trigonometry.atan2(t0, t1), trigonometry.asin(t2), trigonometry.atan2(t3, t4));
    }

    // Length / normalization operations

    /// Squared length of quaternion.
    /// @return Squared length of quaternion (\f$|v|^2\f$)
    pub fn lengthSq(self: Quat) f32 {
        return self.value.lengthSq();
    }

    /// Length of quaternion.
    /// @return Length of quaternion (\f$|v|\f$)
    pub fn length(self: Quat) f32 {
        return self.value.length();
    }

    /// Normalize the quaternion (make it length 1)
    pub fn normalized(self: Quat) Quat {
        return fromVec4(self.value.normalized());
    }

    // Additions / multiplications

    /// Negate (operator - ()), computed as 0 - q (JPH_CROSS_PLATFORM_DETERMINISTIC, see Vec4.negate)
    pub fn negate(self: Quat) Quat {
        return fromVec4(self.value.negate());
    }

    /// Add two quaternions (component wise) (operator +)
    pub fn add(self: Quat, other: Quat) Quat {
        return fromVec4(self.value.add(other.value));
    }

    /// Subtract two quaternions (component wise) (operator -)
    pub fn sub(self: Quat, other: Quat) Quat {
        return fromVec4(self.value.sub(other.value));
    }

    /// Multiply two quaternions (operator * (QuatArg))
    pub fn mul(self: Quat, other: Quat) Quat {
        // Scalar version: [(aw+bz)+(dx-cy),(bw+cx)+(dy-az),(cw+ay)+(dz-bx),-(ax+by)+(dw-cz)] with
        // [a, b, c, d] = self and [x, y, z, w] = other. Written with vectors, every lane does the same operations in the same order.
        const abcd = self.value.value;
        const xyzw = other.value.value;

        // Names based on logical order
        const abca = @shuffle(f32, abcd, undefined, @Vector(4, i32){ 0, 1, 2, 0 });
        const bcab = @shuffle(f32, abcd, undefined, @Vector(4, i32){ 1, 2, 0, 1 });
        const cabc = @shuffle(f32, abcd, undefined, @Vector(4, i32){ 2, 0, 1, 2 });
        const dddd = @shuffle(f32, abcd, undefined, @Vector(4, i32){ 3, 3, 3, 3 });

        const wwwx = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 3, 3, 3, 0 });
        const zxyy = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 2, 0, 1, 1 });
        const yzxz = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 1, 2, 0, 2 });

        // Negate last (logical) component (a true negation like the scalar -(ax+by), not 0 - x)
        const m3 = Vec4.bitXor(.{ .value = abca * wwwx + bcab * zxyy }, Vec4.init(0.0, 0.0, 0.0, -0.0));

        return fromVec4(.{ .value = m3.value + (dddd * xyzw - cabc * yzxz) });
    }

    /// Multiply quaternion with float (operator * (float), float * Quat)
    pub fn mulScalar(self: Quat, v: f32) Quat {
        return fromVec4(self.value.mulScalar(v));
    }

    /// Divide quaternion by float (operator / (float))
    pub fn divScalar(self: Quat, v: f32) Quat {
        return fromVec4(self.value.divScalar(v));
    }

    /// Rotate a vector by this quaternion (operator * (Vec3Arg))
    pub fn mulVec3(self: Quat, v: Vec3) Vec3 {
        // Rotating a vector by a quaternion is done by: p' = q * (p, 0) * q^-1 (q^-1 = conjugated(q) for a unit quaternion)
        // Using Rodrigues formula: https://en.m.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula
        // This is equivalent to: p' = p + 2 * (q.w * q.xyz x p + q.xyz x (q.xyz x p))
        //
        // This is:
        //
        // Vec3 xyz = GetXYZ();
        // Vec3 q_cross_p = xyz.Cross(inValue);
        // Vec3 q_cross_q_cross_p = xyz.Cross(q_cross_p);
        // Vec3 v = mValue.SplatW3() * q_cross_p + q_cross_q_cross_p;
        // return inValue + (v + v);
        //
        // But we can write out the cross products in a more efficient way:
        std.debug.assert(self.isNormalized(.{}));
        const xyz = self.getXYZ();
        const yzx = xyz.swizzle(.y, .z, .x);
        const q_cross_p = v.swizzle(.y, .z, .x).mul(xyz).sub(yzx.mul(v)).swizzle(.y, .z, .x);
        const q_cross_q_cross_p = q_cross_p.swizzle(.y, .z, .x).mul(xyz).sub(yzx.mul(q_cross_p)).swizzle(.y, .z, .x);
        const t = self.value.splatW3().mul(q_cross_p).add(q_cross_q_cross_p);
        return v.add(t.add(t));
    }

    /// Multiply a quaternion with imaginary components and no real component (x, y, z, 0) with a quaternion
    pub fn multiplyImaginary(lhs: Vec3, rhs: Quat) Quat {
        // Scalar version: [(aw+bz)-cy,(bw+cx)-az,(cw+ay)-bx,-(ax+by)-cz] with [a, b, c] = lhs and [x, y, z, w] = rhs.
        // Written with vectors, every lane does the same operations in the same order.
        const abc0 = lhs.value;
        const xyzw = rhs.value.value;

        // Names based on logical order
        const abca = @shuffle(f32, abc0, undefined, @Vector(4, i32){ 0, 1, 2, 0 });
        const bcab = @shuffle(f32, abc0, undefined, @Vector(4, i32){ 1, 2, 0, 1 });
        const cabc = @shuffle(f32, abc0, undefined, @Vector(4, i32){ 2, 0, 1, 2 });

        const wwwx = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 3, 3, 3, 0 });
        const zxyy = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 2, 0, 1, 1 });
        const yzxz = @shuffle(f32, xyzw, undefined, @Vector(4, i32){ 1, 2, 0, 2 });

        // Negate last (logical) component (a true negation like the scalar -(ax+by), not 0 - x)
        const m3 = Vec4.bitXor(.{ .value = abca * wwwx + bcab * zxyy }, Vec4.init(0.0, 0.0, 0.0, -0.0));

        return fromVec4(.{ .value = m3.value - cabc * yzxz });
    }

    /// Rotate a vector by the inverse of this quaternion
    pub fn inverseRotate(self: Quat, v: Vec3) Vec3 {
        std.debug.assert(self.isNormalized(.{}));
        const xyz = self.getXYZ(); // Needs to be negated, but we do this in the equations below
        const yzx = xyz.swizzle(.y, .z, .x);
        const q_cross_p = yzx.mul(v).sub(v.swizzle(.y, .z, .x).mul(xyz)).swizzle(.y, .z, .x);
        const q_cross_q_cross_p = yzx.mul(q_cross_p).sub(q_cross_p.swizzle(.y, .z, .x).mul(xyz)).swizzle(.y, .z, .x);
        const t = self.value.splatW3().mul(q_cross_p).add(q_cross_q_cross_p);
        return v.add(t.add(t));
    }

    /// Rotate a the vector (1, 0, 0) with this quaternion
    pub fn rotateAxisX(self: Quat) Vec3 {
        // This is self.mulVec3(Vec3.axisX()) written out:
        std.debug.assert(self.isNormalized(.{ .tolerance = 2.0e-5 }));
        const t = self.value.add(self.value);
        return Vec3.fromVec4(t.splatX().mul(self.value).add(t.splatW().mul(self.value.swizzle(.w, .z, .y, .x)).flipSign(1, 1, -1, 1)).sub(Vec4.init(1, 0, 0, 0)));
    }

    /// Rotate a the vector (0, 1, 0) with this quaternion
    pub fn rotateAxisY(self: Quat) Vec3 {
        // This is self.mulVec3(Vec3.axisY()) written out:
        std.debug.assert(self.isNormalized(.{ .tolerance = 2.0e-5 }));
        const t = self.value.add(self.value);
        return Vec3.fromVec4(t.splatY().mul(self.value).add(t.splatW().mul(self.value.swizzle(.z, .w, .x, .y)).flipSign(-1, 1, 1, 1)).sub(Vec4.init(0, 1, 0, 0)));
    }

    /// Rotate a the vector (0, 0, 1) with this quaternion
    pub fn rotateAxisZ(self: Quat) Vec3 {
        // This is self.mulVec3(Vec3.axisZ()) written out:
        std.debug.assert(self.isNormalized(.{ .tolerance = 2.0e-5 }));
        const t = self.value.add(self.value);
        return Vec3.fromVec4(t.splatZ().mul(self.value).add(t.splatW().mul(self.value.swizzle(.y, .x, .w, .z)).flipSign(1, -1, 1, 1)).sub(Vec4.init(0, 0, 1, 0)));
    }

    /// Dot product
    pub fn dot(self: Quat, other: Quat) f32 {
        return self.value.dot(other.value);
    }

    /// The conjugate [w, -x, -y, -z] is the same as the inverse for unit quaternions
    pub fn conjugated(self: Quat) Quat {
        return fromVec4(self.value.flipSign(-1, -1, -1, 1));
    }

    /// Get inverse quaternion
    pub fn inversed(self: Quat) Quat {
        return self.conjugated().divScalar(self.length());
    }

    /// Ensures that the W component is positive by negating the entire quaternion if it is not. This is useful when you want to store a quaternion as a 3 vector by discarding W and reconstructing it as sqrt(1 - x^2 - y^2 - z^2).
    pub fn ensureWPositive(self: Quat) Quat {
        return fromVec4(Vec4.bitXor(self.value, Vec4.bitAnd(self.value.splatW(), UVec4.replicate(0x80000000).reinterpretAsFloat())));
    }

    /// Get a quaternion that is perpendicular to this quaternion
    pub fn getPerpendicular(self: Quat) Quat {
        return fromVec4(self.value.swizzle(.y, .x, .w, .z).flipSign(1, -1, 1, -1));
    }

    /// Get rotation angle around axis (uses Swing Twist Decomposition to get the twist quaternion and uses q(axis, angle) = [cos(angle / 2), axis * sin(angle / 2)])
    pub fn getRotationAngle(self: Quat, axis: Vec3) f32 {
        return if (self.getW() == 0.0) math.pi else 2.0 * trigonometry.atan(self.getXYZ().dot(axis) / self.getW());
    }

    /// Swing Twist Decomposition: any quaternion can be split up as:
    ///
    /// \f[q = q_{swing} \: q_{twist}\f]
    ///
    /// where \f$q_{twist}\f$ rotates only around axis v.
    ///
    /// \f$q_{twist}\f$ is:
    ///
    /// \f[q_{twist} = \frac{[q_w, q_{ijk} \cdot v \: v]}{\left|[q_w, q_{ijk} \cdot v \: v]\right|}\f]
    ///
    /// where q_w is the real part of the quaternion and q_i the imaginary part (a 3 vector).
    ///
    /// The swing can then be calculated as:
    ///
    /// \f[q_{swing} = q \: q_{twist}^* \f]
    ///
    /// Where \f$q_{twist}^*\f$ = complex conjugate of \f$q_{twist}\f$
    pub fn getTwist(self: Quat, axis: Vec3) Quat {
        const twist = fromVec4(Vec4.fromVec3W(axis.mulScalar(self.getXYZ().dot(axis)), self.getW()));
        const twist_len = twist.lengthSq();
        if (twist_len != 0.0)
            return twist.divScalar(@sqrt(twist_len))
        else
            return identity();
    }

    /// Result of `getSwingTwist`
    pub const SwingTwist = struct { swing: Quat, twist: Quat };

    /// Decomposes quaternion into swing and twist component:
    ///
    /// \f$q = q_{swing} \: q_{twist}\f$
    ///
    /// where \f$q_{swing} \: \hat{x} = q_{twist} \: \hat{y} = q_{twist} \: \hat{z} = 0\f$
    ///
    /// In other words:
    ///
    /// - \f$q_{twist}\f$ only rotates around the X-axis.
    /// - \f$q_{swing}\f$ only rotates around the Y and Z-axis.
    ///
    /// @see Gino van den Bergen - Rotational Joint Limits in Quaternion Space - GDC 2016
    pub fn getSwingTwist(self: Quat) SwingTwist {
        const x = self.getX();
        const y = self.getY();
        const z = self.getZ();
        const w = self.getW();
        const s = @sqrt(math.square(w) + math.square(x));
        if (s != 0.0) {
            return .{
                .twist = init(x / s, 0, 0, w / s),
                .swing = init(0, (w * y - x * z) / s, (w * z + x * y) / s, s),
            };
        } else {
            // If both x and w are zero, this must be a 180 degree rotation around either y or z
            return .{ .twist = identity(), .swing = self };
        }
    }

    /// Linear interpolation between two quaternions (for small steps).
    /// @param fraction is in the range [0, 1]
    /// @param destination The destination quaternion
    /// @return (1 - fraction) * this + fraction * destination
    pub fn lerp(self: Quat, destination: Quat, fraction: f32) Quat {
        const scale0 = 1.0 - fraction;
        return fromVec4(self.value.mulScalar(scale0).add(destination.value.mulScalar(fraction)));
    }

    /// Spherical linear interpolation between two quaternions.
    /// @param fraction is in the range [0, 1]
    /// @param destination The destination quaternion
    /// @return When fraction is zero this quaternion is returned, when fraction is 1 destination is returned.
    /// When fraction is between 0 and 1 an interpolation along the shortest path is returned.
    pub fn slerp(self: Quat, destination: Quat, fraction: f32) Quat {
        // Difference at which to LERP instead of SLERP
        const delta: f32 = 0.0001;

        // Calc cosine
        var sign_scale1: f32 = 1.0;
        var cos_omega = self.dot(destination);

        // Adjust signs (if necessary)
        if (cos_omega < 0.0) {
            cos_omega = -cos_omega;
            sign_scale1 = -1.0;
        }

        // Calculate coefficients
        var scale0: f32 = undefined;
        var scale1: f32 = undefined;
        if (1.0 - cos_omega > delta) {
            // Standard case (slerp)
            const omega = trigonometry.acos(cos_omega);
            const sin_omega = trigonometry.sin(omega);
            scale0 = trigonometry.sin((1.0 - fraction) * omega) / sin_omega;
            scale1 = sign_scale1 * trigonometry.sin(fraction * omega) / sin_omega;
        } else {
            // Quaternions are very close so we can do a linear interpolation
            scale0 = 1.0 - fraction;
            scale1 = sign_scale1 * fraction;
        }

        // Interpolate between the two quaternions
        return fromVec4(self.value.mulScalar(scale0).add(destination.value.mulScalar(scale1))).normalized();
    }

    /// Load 3 floats from memory (X, Y and Z component and then calculates W) reads 32 bits extra which it doesn't use (Zolt only reads 3 floats)
    pub fn loadFloat3Unsafe(v: *const Float3) Quat {
        const xyz = Vec3.loadFloat3Unsafe(v);
        const w = @sqrt(math.max(1.0 - xyz.lengthSq(), 0.0)); // It is possible that the length of v is a fraction above 1, and we don't want to introduce NaN's in that case so we clamp to 0
        return fromVec4(Vec4.fromVec3W(xyz, w));
    }

    /// Store as 3 floats to memory (X, Y and Z component). Ensures that W is positive before storing.
    pub fn storeFloat3(self: Quat, out: *Float3) void {
        std.debug.assert(self.isNormalized(.{}));
        self.ensureWPositive().getXYZ().storeFloat3(out);
    }

    /// Store as 4 floats
    pub fn storeFloat4(self: Quat, out: *Float4) void {
        self.value.storeFloat4(out);
    }

    /// Compress a unit quaternion to a 32 bit value, precision is around 0.5 degree
    pub fn compressUnitQuat(self: Quat) u32 {
        return self.value.compressUnitVector();
    }

    /// Decompress a unit quaternion from a 32 bit value
    pub fn decompressUnitQuat(v: u32) Quat {
        return fromVec4(Vec4.decompressUnitVector(v));
    }

    /// To String
    pub fn format(self: Quat, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try self.value.format(writer);
    }
};
