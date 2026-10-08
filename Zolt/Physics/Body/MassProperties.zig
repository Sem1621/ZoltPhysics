//! Port of: Jolt/Physics/Body/MassProperties.h, Jolt/Physics/Body/MassProperties.cpp
//! Status: complete
//!
//! - `DecomposePrincipalMomentsOfInertia(outRotation, outDiagonal) -> bool` returns
//!   `?PrincipalMomentsOfInertia{ .rotation, .diagonal }` (null when the eigen decomposition fails; Jolt does not
//!   write the out parameters in that case and its callers do not read them).
//! - `sGetEquivalentSolidBoxSize` is `getEquivalentSolidBoxSize`, `Scale(inScale)` is `scale(scale_value)`.
//! - `operator ==` is `eql` (`!=` is `!a.eql(b)`).
//! - Binary state: the mass (4 bytes) and the inertia tensor (a Mat44, 64 bytes), like Jolt.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const insertionSort = @import("../../Core/InsertionSort.zig").insertionSort;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const eigenValueSymmetric = @import("../../Math/EigenValueSymmetric.zig").eigenValueSymmetric;
const math = @import("../../Math/Math.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Matrix = @import("../../Math/Matrix.zig").Matrix;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Vector = @import("../../Math/Vector.zig").Vector;

/// Describes the mass and inertia properties of a body. Used during body construction only.
pub const MassProperties = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, MassProperties)

    /// Mass of the shape (kg)
    mass: f32 = 0.0,

    /// Inertia tensor of the shape (kg m^2)
    inertia: Mat44 = Mat44.zero(),

    /// Result of `decomposePrincipalMomentsOfInertia` (Jolt's out parameters)
    pub const PrincipalMomentsOfInertia = struct {
        /// The rotation matrix R
        rotation: Mat44,
        /// The diagonal of the diagonal matrix D
        diagonal: Vec3,
    };

    /// Test if two MassProperties are equal (operator ==)
    pub fn eql(self: *const MassProperties, other: *const MassProperties) bool {
        return self.mass == other.mass and self.inertia.eql(other.inertia);
    }

    /// Using eigendecomposition, decompose the inertia tensor into a diagonal matrix D and a right-handed rotation matrix R so that the inertia tensor is \f$R \: D \: R^{-1}\f$.
    /// @see https://en.wikipedia.org/wiki/Moment_of_inertia section 'Principal axes'
    /// @return The rotation matrix R and the diagonal of the diagonal matrix D, null if failed
    pub fn decomposePrincipalMomentsOfInertia(self: *const MassProperties) ?PrincipalMomentsOfInertia {
        // Using eigendecomposition to get the principal components of the inertia tensor
        // See: https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix
        var inertia: Matrix(3, 3) = undefined;
        inertia.copyPart(self.inertia, 0, 0, 3, 3, 0, 0);
        var eigen_vec = Matrix(3, 3).identity();
        var eigen_val: Vector(3) = undefined;
        if (!eigenValueSymmetric(inertia, &eigen_vec, &eigen_val))
            return null;

        // Sort so that the biggest value goes first
        var indices = [_]i32{ 0, 1, 2 };
        insertionSort(i32, &indices, &eigen_val, struct {
            fn greater(values: *const Vector(3), left: i32, right: i32) bool {
                return values.getComponent(@intCast(left)) > values.getComponent(@intCast(right));
            }
        }.greater);

        // Convert to a regular Mat44 and Vec3
        var out_rotation = Mat44.identity();
        var out_diagonal = Vec3.zero();
        for (0..3) |i| {
            const column = eigen_vec.getColumn(@intCast(indices[i]));
            out_rotation.setColumn3(@intCast(i), Vec3.init(column.f32s[0], column.f32s[1], column.f32s[2]));
            out_diagonal.setComponent(@intCast(i), eigen_val.getComponent(@intCast(indices[i])));
        }

        // Make sure that the rotation matrix is a right handed matrix
        if (out_rotation.getAxisX().cross(out_rotation.getAxisY()).dot(out_rotation.getAxisZ()) < 0.0)
            out_rotation.setAxisZ(out_rotation.getAxisZ().negate());

        if (Core.enable_asserts) {
            // Validate that the solution is correct, for each axis we want to make sure that the difference in inertia is
            // smaller than some fraction of the inertia itself in that axis
            const new_inertia = out_rotation.mul(Mat44.scaleVec3(out_diagonal)).mul(out_rotation.inversed());
            for (0..3) |i| {
                const c: u32 = @intCast(i);
                std.debug.assert(new_inertia.getColumn3(c).isClose(self.inertia.getColumn3(c), .{ .max_dist_sq = self.inertia.getColumn3(c).lengthSq() * 1.0e-10 }));
            }
        }

        return .{ .rotation = out_rotation, .diagonal = out_diagonal };
    }

    /// Set the mass and inertia of a box with edge size inBoxSize and density inDensity
    pub fn setMassAndInertiaOfSolidBox(self: *MassProperties, box_size: Vec3, density: f32) void {
        // Calculate mass
        self.mass = box_size.getX() * box_size.getY() * box_size.getZ() * density;

        // Calculate inertia
        const size_sq = box_size.mul(box_size);
        const scale_value = size_sq.swizzle(.y, .x, .x).add(size_sq.swizzle(.z, .z, .y)).mulScalar(self.mass / 12.0);
        self.inertia = Mat44.scaleVec3(scale_value);
    }

    /// Set the mass and scale the inertia tensor to match the mass
    pub fn scaleToMass(self: *MassProperties, mass: f32) void {
        if (self.mass > 0.0) {
            // Calculate how much we have to scale the inertia tensor
            const mass_scale = mass / self.mass;

            // Update mass
            self.mass = mass;

            // Update inertia tensor
            for (0..3) |i|
                self.inertia.setColumn4(@intCast(i), self.inertia.getColumn4(@intCast(i)).mulScalar(mass_scale));
        } else {
            // Just set the mass
            self.mass = mass;
        }
    }

    /// Calculates the size of the solid box that has an inertia tensor diagonal inInertiaDiagonal
    pub fn getEquivalentSolidBoxSize(mass: f32, inertia_diagonal: Vec3) Vec3 {
        // Moment of inertia of a solid box has diagonal:
        // mass / 12 * [size_y^2 + size_z^2, size_x^2 + size_z^2, size_x^2 + size_y^2]
        // Solving for size_x, size_y and size_y (diagonal and mass are known):
        const diagonal = inertia_diagonal.mulScalar(12.0 / mass);
        const d0 = diagonal.getComponent(0);
        const d1 = diagonal.getComponent(1);
        const d2 = diagonal.getComponent(2);
        return Vec3.init(math.sqrt(0.5 * (-d0 + d1 + d2)), math.sqrt(0.5 * (d0 - d1 + d2)), math.sqrt(0.5 * (d0 + d1 - d2)));
    }

    /// Rotate the inertia by 3x3 matrix inRotation
    pub fn rotate(self: *MassProperties, rotation: Mat44) void {
        self.inertia = rotation.multiply3x3Mat44(self.inertia).multiply3x3RightTransposed(rotation);
    }

    /// Translate the inertia by a vector inTranslation
    pub fn translate(self: *MassProperties, translation: Vec3) void {
        // Transform the inertia using the parallel axis theorem: I' = I + m * (translation^2 E - translation translation^T)
        // Where I is the original body's inertia and E the identity matrix
        // See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
        self.inertia = self.inertia.add(Mat44.scale(translation.dot(translation)).sub(Mat44.outerProduct(translation, translation)).mulScalar(self.mass));

        // Ensure that inertia is a 3x3 matrix, adding inertias causes the bottom right element to change
        self.inertia.setColumn4(3, Vec4.init(0, 0, 0, 1));
    }

    /// Scale the mass and inertia by inScale, note that elements can be < 0 to flip the shape
    pub fn scale(self: *MassProperties, scale_value: Vec3) void {
        // See: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
        // The diagonal of the inertia tensor can be calculated like this:
        // Ixx = sum_{k = 1 to n}(m_k * (y_k^2 + z_k^2))
        // Iyy = sum_{k = 1 to n}(m_k * (x_k^2 + z_k^2))
        // Izz = sum_{k = 1 to n}(m_k * (x_k^2 + y_k^2))
        //
        // We want to isolate the terms x_k, y_k and z_k:
        // d = [0.5, 0.5, 0.5].[Ixx, Iyy, Izz]
        // [sum_{k = 1 to n}(m_k * x_k^2), sum_{k = 1 to n}(m_k * y_k^2), sum_{k = 1 to n}(m_k * z_k^2)] = [d, d, d] - [Ixx, Iyy, Izz]
        const diagonal = self.inertia.getDiagonal3();
        const xyz_sq = Vec3.replicate(Vec3.replicate(0.5).dot(diagonal)).sub(diagonal);

        // When scaling a shape these terms change like this:
        // sum_{k = 1 to n}(m_k * (scale_x * x_k)^2) = scale_x^2 * sum_{k = 1 to n}(m_k * x_k^2)
        // Same for y_k and z_k
        // Using these terms we can calculate the new diagonal of the inertia tensor:
        const xyz_scaled_sq = scale_value.mul(scale_value).mul(xyz_sq);
        const i_xx = xyz_scaled_sq.getY() + xyz_scaled_sq.getZ();
        const i_yy = xyz_scaled_sq.getX() + xyz_scaled_sq.getZ();
        const i_zz = xyz_scaled_sq.getX() + xyz_scaled_sq.getY();

        // The off diagonal elements are calculated like:
        // Ixy = -sum_{k = 1 to n}(x_k y_k)
        // Ixz = -sum_{k = 1 to n}(x_k z_k)
        // Iyz = -sum_{k = 1 to n}(y_k z_k)
        // Scaling these is simple:
        const i_xy = scale_value.getX() * scale_value.getY() * self.inertia.get(0, 1);
        const i_xz = scale_value.getX() * scale_value.getZ() * self.inertia.get(0, 2);
        const i_yz = scale_value.getY() * scale_value.getZ() * self.inertia.get(1, 2);

        // Update inertia tensor
        self.inertia.set(0, 0, i_xx);
        self.inertia.set(0, 1, i_xy);
        self.inertia.set(1, 0, i_xy);
        self.inertia.set(1, 1, i_yy);
        self.inertia.set(0, 2, i_xz);
        self.inertia.set(2, 0, i_xz);
        self.inertia.set(1, 2, i_yz);
        self.inertia.set(2, 1, i_yz);
        self.inertia.set(2, 2, i_zz);

        // Mass scales linear with volume (note that the scaling can be negative and we don't want the mass to become negative)
        const mass_scale = @abs(scale_value.getX() * scale_value.getY() * scale_value.getZ());
        self.mass *= mass_scale;

        // Inertia scales linear with mass. This updates the m_k terms above.
        self.inertia = self.inertia.mulScalar(mass_scale);

        // Ensure that the bottom right element is a 1 again
        self.inertia.set(3, 3, 1.0);
    }

    /// Saves the state of this object in binary form to inStream.
    pub fn saveBinaryState(self: *const MassProperties, stream: StreamOut) void {
        stream.write(self.mass);
        stream.write(self.inertia);
    }

    /// Restore the state of this object from inStream.
    pub fn restoreBinaryState(self: *MassProperties, stream: StreamIn) void {
        stream.read(&self.mass);
        stream.read(&self.inertia);
    }
};

test "MassProperties: solid box, translate, rotate, scale, decompose" {
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;

    var m: MassProperties = .{};
    try expect(m.eql(&.{}));
    m.setMassAndInertiaOfSolidBox(Vec3.init(2, 4, 6), 10.0);
    try expectEqual(@as(f32, 480.0), m.mass);
    try expectEqual(@as(f32, 480.0 / 12.0 * (16.0 + 36.0)), m.inertia.get(0, 0));
    try expectEqual(@as(f32, 480.0 / 12.0 * (4.0 + 36.0)), m.inertia.get(1, 1));
    try expectEqual(@as(f32, 480.0 / 12.0 * (4.0 + 16.0)), m.inertia.get(2, 2));

    // The equivalent solid box of a solid box is the box itself
    try expect(MassProperties.getEquivalentSolidBoxSize(m.mass, m.inertia.getDiagonal3()).isClose(Vec3.init(2, 4, 6), .{ .max_dist_sq = 1.0e-10 }));

    // Decomposing a diagonal tensor sorts the moments from big to small and gives a right handed rotation
    const d = m.decomposePrincipalMomentsOfInertia().?;
    try expect(d.diagonal.isClose(Vec3.init(m.inertia.get(0, 0), m.inertia.get(1, 1), m.inertia.get(2, 2)), .{}));
    try expect(d.rotation.getAxisX().cross(d.rotation.getAxisY()).dot(d.rotation.getAxisZ()) > 0.0);

    // Translating adds m * |t|^2 on the diagonal of the perpendicular axes, the bottom right element stays 1
    var t = m;
    t.translate(Vec3.init(1, 0, 0));
    try expectEqual(m.inertia.get(0, 0), t.inertia.get(0, 0));
    try expectEqual(m.inertia.get(1, 1) + 480.0, t.inertia.get(1, 1));
    try expectEqual(@as(f32, 1.0), t.inertia.get(3, 3));

    // Rotating by 90 degrees around Z swaps the X and Y moments
    var r = m;
    r.rotate(Mat44.rotationZ(0.5 * math.pi));
    try expect(r.inertia.getDiagonal3().isClose(Vec3.init(m.inertia.get(1, 1), m.inertia.get(0, 0), m.inertia.get(2, 2)), .{ .max_dist_sq = 1.0e-6 }));

    // Scaling a box gives the mass properties of the scaled box, a negative scale doesn't change them
    var s = m;
    s.scale(Vec3.init(-0.5, 0.5, 0.5));
    var expected: MassProperties = .{};
    expected.setMassAndInertiaOfSolidBox(Vec3.init(1, 2, 3), 10.0);
    try expectEqual(expected.mass, s.mass);
    try expect(s.inertia.isClose(expected.inertia, .{}));

    // Scale to mass
    s.scaleToMass(2.0 * s.mass);
    try expectEqual(2.0 * expected.mass, s.mass);
    try expect(s.inertia.getDiagonal3().isClose(expected.inertia.getDiagonal3().mulScalar(2.0), .{}));
    var zero: MassProperties = .{};
    zero.scaleToMass(5.0);
    try expectEqual(@as(f32, 5.0), zero.mass);
    try expect(zero.inertia.eql(Mat44.zero()));
}

test "MassProperties: binary state" {
    const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;

    var m: MassProperties = .{};
    m.setMassAndInertiaOfSolidBox(Vec3.init(1, 2, 3), 7.0);
    m.translate(Vec3.init(0.5, -1, 2));

    var buffer: [128]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    m.saveBinaryState(out.streamOut());
    try std.testing.expectEqual(@as(usize, 4 + 64), writer.end);

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamInWrapper.init(&reader);
    var restored: MassProperties = .{};
    restored.restoreBinaryState(in.streamIn());
    try std.testing.expect(!in.streamIn().isFailed() and !in.streamIn().isEOF());
    try std.testing.expect(restored.eql(&m));
}
