//! Port of: Jolt/Physics/Collision/RayCast.h
//! Status: complete
//!
//! `RayCastT<Vec, Mat, RayCastType>` is the comptime function `RayCastT(Vec, Mat, kind)`. Jolt's third template
//! parameter is the derived class (RayCast or RRayCast, the return type of Transformed / Translated); Zolt passes a
//! `RayCastKind` instead, which also keeps `RayCast` (local space, Vec3 / Mat44) and `RRayCast` (world space, RVec3 /
//! RMat44) different types in single precision builds, where RVec3 is Vec3. Like in C++, converting between them is
//! explicit: `RRayCast.fromRayCast(ray)` (explicit RRayCast(const RayCast &)) and `r_ray.toRayCast()` (explicit
//! operator RayCast()); these two only exist on RRayCast.
//!
//! The default constructor leaves the ray uninitialized (`var ray: RayCast = undefined;`). The code uses the
//! precision independent spellings (`mulRVec3`, `addVec3`) so that it compiles with and without `-Ddouble_precision`.

const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const BackFaceMode = @import("BackFaceMode.zig").BackFaceMode;

/// Which ray cast type a RayCastT instance is (Jolt's RayCastType template parameter)
pub const RayCastKind = enum {
    /// RayCast: Vec3 origin, transformed by a Mat44
    ray_cast,
    /// RRayCast: RVec3 origin, transformed by an RMat44
    r_ray_cast,
};

/// Structure that holds a single ray cast
pub fn RayCastT(comptime Vec: type, comptime Mat: type, comptime kind: RayCastKind) type {
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

        /// Transform this ray using inTransform
        pub fn transformed(self: Self, transform: Mat) Self {
            const ray_origin = transform.mulRVec3(self.origin);
            const ray_direction = toVec3(transform.mulRVec3(self.origin.addVec3(self.direction)).sub(ray_origin));
            return .{ .origin = ray_origin, .direction = ray_direction };
        }

        /// Translate ray using inTranslation
        pub fn translated(self: Self, translation: Vec) Self {
            return .{ .origin = translation.add(self.origin), .direction = self.direction };
        }

        /// Get point with fraction inFraction on ray (0 = start of ray, 1 = end of ray)
        pub fn getPointOnRay(self: Self, fraction: f32) Vec {
            return self.origin.addVec3(self.direction.mulScalar(fraction));
        }

        /// Convert from RayCast, converts single to double precision (RRayCast only)
        pub const fromRayCast = switch (kind) {
            .r_ray_cast => fromRayCastImpl,
            .ray_cast => {},
        };

        /// Convert to RayCast, which implies casting from double precision to single precision (RRayCast only)
        pub const toRayCast = switch (kind) {
            .r_ray_cast => toRayCastImpl,
            .ray_cast => {},
        };

        fn fromRayCastImpl(ray: RayCast) Self {
            return .init(if (Vec == Vec3) ray.origin else Vec.fromVec3(ray.origin), ray.direction);
        }

        fn toRayCastImpl(self: Self) RayCast {
            return .init(toVec3(self.origin), self.direction);
        }

        /// Vec3(inV): explicit conversion to single precision (a copy for Vec3)
        fn toVec3(v: Vec) Vec3 {
            return if (Vec == Vec3) v else v.toVec3();
        }
    };
}

/// Ray cast in local space (single precision)
pub const RayCast = RayCastT(Vec3, Mat44, .ray_cast);

/// Ray cast in world space (RVec3 origin, double precision with `-Ddouble_precision`)
pub const RRayCast = RayCastT(RVec3, RMat44, .r_ray_cast);

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

test "RayCast / RRayCast" {
    const std = @import("std");
    const expect = std.testing.expect;

    try expect(RayCast != RRayCast);

    const ray = RayCast.init(Vec3.init(1, 2, 3), Vec3.init(4, 0, 0));
    try expect(ray.getPointOnRay(0.5).eql(Vec3.init(3, 2, 3)));
    try expect(ray.translated(Vec3.init(1, 1, 1)).origin.eql(Vec3.init(2, 3, 4)));
    const t = ray.transformed(Mat44.translation(Vec3.init(0, 0, 1)).mul(Mat44.rotationZ(0.5 * std.math.pi)));
    try expect(t.origin.isClose(Vec3.init(-2, 1, 4), .{}));
    try expect(t.direction.isClose(Vec3.init(0, 4, 0), .{}));

    const r_ray = RRayCast.fromRayCast(ray);
    try expect(r_ray.origin.eql(RVec3.init(1, 2, 3)));
    try expect(r_ray.getPointOnRay(1.0).eql(RVec3.init(5, 2, 3)));
    try expect(r_ray.translated(RVec3.init(1, 0, 0)).origin.eql(RVec3.init(2, 2, 3)));
    try expect(r_ray.transformed(RMat44.identity()).direction.eql(ray.direction));
    const back = r_ray.toRayCast();
    try expect(back.origin.eql(ray.origin) and back.direction.eql(ray.direction));
    try expect(@TypeOf(RayCast.fromRayCast) == void and @TypeOf(RayCast.toRayCast) == void);

    var settings: RayCastSettings = .{};
    try expect(settings.back_face_mode_triangles == .ignore_back_faces and settings.back_face_mode_convex == .ignore_back_faces and settings.treat_convex_as_solid);
    settings.setBackFaceMode(.collide_with_back_faces);
    try expect(settings.back_face_mode_triangles == .collide_with_back_faces and settings.back_face_mode_convex == .collide_with_back_faces);
}
