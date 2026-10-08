//! Port of: Jolt/Physics/Body/MassProperties.h, Jolt/Physics/Body/MassProperties.cpp (prototype, reduced)
//! Status: partial
//! Missing: DecomposePrincipalMomentsOfInertia, ScaleToMass, sGetEquivalentSolidBoxSize, SaveBinaryState, RestoreBinaryState

const zolt = @import("zolt");
const Mat44 = zolt.Mat44;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// Describes the mass and inertia properties of a body. Used during body construction only.
pub const MassProperties = struct {
    /// Mass of the shape (kg)
    mass: f32 = 0.0,
    /// Inertia tensor of the shape (kg m^2)
    inertia: Mat44 = Mat44.zero(),

    /// Set the mass and inertia of a box with edge size inBoxSize and density inDensity
    pub fn setMassAndInertiaOfSolidBox(self: *MassProperties, box_size: Vec3, density: f32) void {
        // Calculate mass
        self.mass = box_size.getX() * box_size.getY() * box_size.getZ() * density;

        // Calculate inertia
        const size_sq = box_size.mul(box_size);
        const scale_value = size_sq.swizzle(.y, .x, .x).add(size_sq.swizzle(.z, .z, .y)).mulScalar(self.mass / 12.0);
        self.inertia = Mat44.scaleVec3(scale_value);
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
};
