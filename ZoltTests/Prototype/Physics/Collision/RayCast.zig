//! Port of: Jolt/Physics/Collision/RayCast.h
//! Status: complete
//!
//! `RayCastT<Vec, Mat, RayCastType>` is the comptime function `RayCastT(Vec, Mat)`; `RayCast` (local space, single
//! precision) and `RRayCast` (world space, RVec3) are its instances. Code uses the precision independent spellings
//! (`mulRVec3`, `addVec3`) so that it compiles with and without `-Ddouble_precision`.

const zolt = @import("zolt");
const Vec3 = zolt.Vec3;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const RMat44 = zolt.RMat44;
const BackFaceMode = @import("BackFaceMode.zig").BackFaceMode;

/// Structure that holds a single ray cast
pub fn RayCastT(comptime Vec: type, comptime Mat: type) type {
    return struct {
        const Self = @This();

        /// Origin of the ray
        origin: Vec,
        /// Direction and length of the ray (anything beyond this length will not be reported as a hit)
        direction: Vec3,

        /// Constructor
        pub fn init(origin: Vec, direction: Vec3) Self {
            return .{ .origin = origin, .direction = direction };
        }

        /// Transform this ray using transform
        pub fn transformed(self: Self, transform: Mat) Self {
            const ray_origin = transform.mulRVec3(self.origin);
            const ray_direction = toVec3(transform.mulRVec3(self.origin.addVec3(self.direction)).sub(ray_origin));
            return .{ .origin = ray_origin, .direction = ray_direction };
        }

        /// Translate ray using translation
        pub fn translated(self: Self, translation: Vec) Self {
            return .{ .origin = translation.add(self.origin), .direction = self.direction };
        }

        /// Get point with fraction on ray (0 = start of ray, 1 = end of ray)
        pub fn getPointOnRay(self: Self, fraction: f32) Vec {
            return self.origin.addVec3(self.direction.mulScalar(fraction));
        }

        /// Convert to RayCast, which implies casting from double precision to single precision (explicit operator RayCast())
        pub fn toRayCast(self: Self) RayCastT(Vec3, Mat44) {
            return .{ .origin = toVec3(self.origin), .direction = self.direction };
        }

        /// Convert from RayCast, converts single to double precision (explicit RRayCast(const RayCast &))
        pub fn fromRayCast(ray: RayCastT(Vec3, Mat44)) Self {
            return .{ .origin = if (Vec == Vec3) ray.origin else Vec.fromVec3(ray.origin), .direction = ray.direction };
        }

        fn toVec3(v: Vec) Vec3 {
            return if (Vec == Vec3) v else v.toVec3();
        }
    };
}

pub const RayCast = RayCastT(Vec3, Mat44);
pub const RRayCast = RayCastT(RVec3, RMat44);

/// Settings to be passed with a ray cast
pub const RayCastSettings = struct {
    /// How backfacing triangles should be treated (should we report back facing hits for triangle based shapes, e.g. MeshShape/HeightFieldShape?)
    back_face_mode_triangles: BackFaceMode = .ignore_back_faces,
    /// How backfacing convex objects should be treated (should we report back facing hits for convex shapes?)
    back_face_mode_convex: BackFaceMode = .ignore_back_faces,
    /// If convex shapes should be treated as solid. When true, a ray starting inside a convex shape will generate a hit at fraction 0.
    treat_convex_as_solid: bool = true,

    /// Set the backfacing mode for all shapes
    pub fn setBackFaceMode(self: *RayCastSettings, mode: BackFaceMode) void {
        self.back_face_mode_triangles = mode;
        self.back_face_mode_convex = mode;
    }
};
