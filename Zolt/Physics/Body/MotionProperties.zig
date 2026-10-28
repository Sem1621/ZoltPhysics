//! Port of: Jolt/Physics/Body/MotionProperties.h, Jolt/Physics/Body/MotionProperties.inl, Jolt/Physics/Body/MotionProperties.cpp
//! Status: complete
//!
//! SimulationArchitecture D1:
//! - An `extern struct` (Phase 14), so that the fields keep Jolt's order and offsets (Jolt's cache line comments hold:
//!   the fields that the solver and SplitIsland read share Jolt's first two cache lines; the sleep test spheres and
//!   timer stay in the third, least used one, only the sleep test's `allow_sleeping` is in the second). The offsets
//!   and the size are checked at compile time against Jolt's (x86-64 clang's -fdump-record-layouts, with and without
//!   double precision, asserts and simulation stats). A plain struct let Zig move the 1 byte fields (allowed DOFs,
//!   motion quality, step overrides) next to the sleep test spheres, a third cache line that Jolt's solver does not
//!   touch.
//!   The `void` fields (asserts, double precision, simulation stats) take no space. `SimulationStats` is an
//!   `extern struct` too (an extern struct may only hold extern compatible fields). Members that Jolt leaves
//!   uninitialized default to `undefined` (BodyManager fills them in through Body.applyBodyCreationSettings).
//! - `mIndexInActiveBodies` is a `std.atomic.Value(u32)` loaded and stored with `.monotonic` (Jolt's TSAN-suppressed
//!   race: FindCollidingPairsCanCollide reads it while another job activates the body).
//! - The JPH_ENABLE_ASSERTS members (`mCachedBodyType`, `mCachedMotionType`) are typed `void` without asserts.
//! - The BodyAccess checks are `BodyAccess.assertVelocityAccess(.read)` etc. (only evaluated with asserts).
//! - JPH_TRACK_SIMULATION_STATS (off by default, like Jolt's CMake option) is the comptime switch
//!   `track_simulation_stats`; `simulation_stats` is `void` while it is off.
//! - `JPH_DET_LOG` in ApplyLinearVelocityStep / ApplyAngularVelocityStep: `if (DeterminismLog.enabled)
//!   DeterminismLog.log(.{ ... })` (SimulationArchitecture D11, the build option `-Ddeterminism_log`).
//! - `ResetSleepTestSpheres(const RVec3 *)` takes `*const [3]RVec3`.
//! - Jolt's `JPH_ASSERT`s on user values (finite velocities, positive masses, valid DOFs) are
//!   `if (Core.enable_asserts) std.debug.assert(...)`, so ReleaseFast keeps Jolt's release behavior.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const UVec4 = @import("../../Math/UVec4.zig").UVec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Float3 = @import("../../Math/Float3.zig").Float3;
const Double3 = @import("../../Math/Double3.zig").Double3;
const DVec3 = @import("../../Math/DVec3.zig").DVec3;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Sphere = @import("../../Geometry/Sphere.zig").Sphere;
const AllowedDOFs = @import("AllowedDOFs.zig").AllowedDOFs;
const MotionQuality = @import("MotionQuality.zig").MotionQuality;
const MotionType = @import("MotionType.zig").MotionType;
const BodyType = @import("BodyType.zig").BodyType;
const BodyAccess = @import("BodyAccess.zig").BodyAccess;
const MassProperties = @import("MassProperties.zig").MassProperties;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const DeterminismLog = @import("../DeterminismLog.zig");

/// JPH_TRACK_SIMULATION_STATS: track per body simulation statistics (off, like Jolt's default)
pub const track_simulation_stats = false;

/// Enum that determines if an object can go to sleep (ECanSleep)
pub const CanSleep = enum(u32) {
    /// Object cannot go to sleep
    cannot_sleep = 0,
    /// Object can go to sleep
    can_sleep = 1,
};

/// The Body class only keeps track of state for static bodies, the MotionProperties class keeps the additional state needed for a moving Body. It has a 1-on-1 relationship with the body.
pub const MotionProperties = extern struct {
    /// Constant indicating that body is not active
    pub const inactive_index: u32 = 0xffffffff;

    /// Stats for this body. These are average for the simulation island the body was part of. (JPH_TRACK_SIMULATION_STATS)
    /// Zolt: an `extern struct` like MotionProperties, which holds it (an extern struct cannot hold an auto layout one)
    pub const SimulationStats = extern struct {
        /// Number of processor ticks spent doing broad phase collision detection
        broad_phase_ticks: u64 = 0,
        /// Number of ticks spent doing narrow phase collision detection
        narrow_phase_ticks: std.atomic.Value(u64) = .init(0),
        /// Number of ticks spent solving velocity constraints
        velocity_constraint_ticks: u64 = 0,
        /// Number of ticks spent solving position constraints
        position_constraint_ticks: u64 = 0,
        /// Number of ticks spent updating the broadphase and checking if the body should go to sleep
        update_bounds_ticks: u64 = 0,
        /// Number of ticks spent doing CCD
        ccd_ticks: std.atomic.Value(u64) = .init(0),
        /// Number of contact constraints created for this body
        num_contact_constraints: std.atomic.Value(u32) = .init(0),
        /// Number of collision steps this body was active (see PhysicsSystem::Update(..., inCollisionSteps, ...)). All other properties are aggregated over multiple steps, so if you want e.g. the number of contact constraints per step you need to divide by this number.
        num_collision_steps: u8 = 0,
        /// Number of velocity iterations performed
        num_velocity_steps: u8 = 0,
        /// Number of position iterations performed
        num_position_steps: u8 = 0,
        /// If this body was part of a large island
        is_large_island: bool = false,

        pub fn reset(self: *SimulationStats) void {
            self.broad_phase_ticks = 0;
            self.narrow_phase_ticks.store(0, .monotonic);
            self.velocity_constraint_ticks = 0;
            self.position_constraint_ticks = 0;
            self.update_bounds_ticks = 0;
            self.ccd_ticks.store(0, .monotonic);
            self.num_contact_constraints.store(0, .monotonic);
            self.num_collision_steps = 0;
            self.num_velocity_steps = 0;
            self.num_position_steps = 0;
            self.is_large_island = false;
        }
    };

    // 1st cache line
    // 16 byte aligned
    /// World space linear velocity of the center of mass (m/s)
    linear_velocity: Vec3 = Vec3.zero(),
    /// World space angular velocity (rad/s)
    angular_velocity: Vec3 = Vec3.zero(),
    /// Diagonal of inverse inertia matrix: D
    inv_inertia_diagonal: Vec3 = undefined,
    /// Rotation (R) that takes inverse inertia diagonal to local space: Ibody^-1 = R * D * R^-1
    inertia_rotation: Quat = undefined,

    // 2nd cache line
    // 4 byte aligned
    /// Accumulated world space force (N). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
    force: Float3 = .{ .x = 0, .y = 0, .z = 0 },
    /// Accumulated world space torque (N m). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
    torque: Float3 = .{ .x = 0, .y = 0, .z = 0 },
    /// Inverse mass of the object (1/kg)
    inv_mass: f32 = undefined,
    /// Linear damping: dv/dt = -c * v. Value should be zero or positive and is usually close to 0.
    linear_damping: f32 = undefined,
    /// Angular damping: dw/dt = -c * w. Value should be zero or positive and is usually close to 0.
    angular_damping: f32 = undefined,
    /// Maximum linear velocity that this body can reach (m/s)
    max_linear_velocity: f32 = undefined,
    /// Maximum angular velocity that this body can reach (rad/s)
    max_angular_velocity: f32 = undefined,
    /// Factor to multiply gravity with
    gravity_factor: f32 = undefined,
    /// If the body is active, this is the index in the active body list or cInactiveIndex if it is not active (note that there are 2 lists, one for rigid and one for soft bodies).
    /// Jolt: a plain uint32 with a TSAN-suppressed race, Zolt: a `.monotonic` atomic (SimulationArchitecture section 0 rule 3).
    index_in_active_bodies: std.atomic.Value(u32) = .init(inactive_index),
    /// Index of the island that this body is part of, when the body has not yet been updated or is not active this is cInactiveIndex
    island_index: u32 = inactive_index,

    // 1 byte aligned
    /// Motion quality, or how well it detects collisions when it has a high velocity
    motion_quality: MotionQuality = undefined,
    /// If this body can go to sleep
    allow_sleeping: bool = undefined,
    /// Allowed degrees of freedom for this body
    allowed_dofs: AllowedDOFs = .all,
    /// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_velocity_steps_override: u8 = 0,
    /// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_position_steps_override: u8 = 0,

    // 3rd cache line (least frequently used)
    // 4 byte aligned (or 8 byte if running in double precision)
    /// mSleepTestSpheres are relative to this offset to prevent floating point inaccuracies (JPH_DOUBLE_PRECISION only)
    sleep_test_offset: if (Core.double_precision) Double3 else void = undefined,
    /// Measure motion for 3 points on the body to see if it is resting: COM, COM + largest bounding box axis, COM + second largest bounding box axis
    sleep_test_spheres: [3]Sphere = undefined,
    /// How long this body has been within the movement tolerance
    sleep_test_timer: f32 = undefined,

    /// Copied from Body::mBodyType and cached for asserting purposes (JPH_ENABLE_ASSERTS)
    cached_body_type: if (Core.enable_asserts) BodyType else void = undefined,
    /// Copied from Body::mMotionType and cached for asserting purposes (JPH_ENABLE_ASSERTS)
    cached_motion_type: if (Core.enable_asserts) MotionType else void = undefined,

    /// JPH_TRACK_SIMULATION_STATS
    simulation_stats: if (track_simulation_stats) SimulationStats else void = if (track_simulation_stats) .{} else {},

    // Zolt: `force` and `torque` are read through Vec3.loadFloat3Unsafe, which reads 4 bytes past them
    comptime {
        std.debug.assert(Vec3.canLoadFloat3Unsafe(MotionProperties, "force"));
        std.debug.assert(Vec3.canLoadFloat3Unsafe(MotionProperties, "torque"));
    }

    // Zolt: Jolt's offsets and size (x86-64 clang's -fdump-record-layouts of MotionProperties.h with and without
    // JPH_DOUBLE_PRECISION, JPH_ENABLE_ASSERTS and JPH_TRACK_SIMULATION_STATS)
    comptime {
        const dp = Core.double_precision;
        const offsets = .{
            .{ "linear_velocity", 0 },
            .{ "angular_velocity", 16 },
            .{ "inv_inertia_diagonal", 32 },
            .{ "inertia_rotation", 48 },
            .{ "force", 64 },
            .{ "torque", 76 },
            .{ "inv_mass", 88 },
            .{ "linear_damping", 92 },
            .{ "angular_damping", 96 },
            .{ "max_linear_velocity", 100 },
            .{ "max_angular_velocity", 104 },
            .{ "gravity_factor", 108 },
            .{ "index_in_active_bodies", 112 },
            .{ "island_index", 116 },
            .{ "motion_quality", 120 },
            .{ "allow_sleeping", 121 },
            .{ "allowed_dofs", 122 },
            .{ "num_velocity_steps_override", 123 },
            .{ "num_position_steps_override", 124 },
            .{ "sleep_test_spheres", if (dp) 152 else 128 },
            .{ "sleep_test_timer", if (dp) 200 else 176 },
        };
        for (offsets) |o| {
            if (@offsetOf(MotionProperties, o[0]) != o[1])
                @compileError(std.fmt.comptimePrint("MotionProperties.{s} is at {d}, Jolt's at {d}", .{ o[0], @offsetOf(MotionProperties, o[0]), o[1] }));
        }
        if (dp) std.debug.assert(@offsetOf(MotionProperties, "sleep_test_offset") == 128);
        if (Core.enable_asserts) {
            std.debug.assert(@offsetOf(MotionProperties, "cached_body_type") == if (dp) 204 else 180);
            std.debug.assert(@offsetOf(MotionProperties, "cached_motion_type") == if (dp) 205 else 181);
        }
        if (track_simulation_stats) {
            std.debug.assert(@offsetOf(MotionProperties, "simulation_stats") == if (dp) 208 else 184);
            std.debug.assert(@sizeOf(SimulationStats) == 56 and @offsetOf(SimulationStats, "num_contact_constraints") == 48 and @offsetOf(SimulationStats, "is_large_island") == 55);
        }
        std.debug.assert(@sizeOf(MotionProperties) == if (track_simulation_stats) (if (dp) 272 else 240) else if (dp) 208 else 192);
        std.debug.assert(@alignOf(MotionProperties) == Core.vector_alignment);
    }

    /// Motion quality, or how well it detects collisions when it has a high velocity
    pub fn getMotionQuality(self: *const MotionProperties) MotionQuality {
        return self.motion_quality;
    }

    /// Get the allowed degrees of freedom that this body has (this can be changed by calling SetMassProperties)
    pub fn getAllowedDOFs(self: *const MotionProperties) AllowedDOFs {
        return self.allowed_dofs;
    }

    /// If this body can go to sleep.
    pub fn getAllowSleeping(self: *const MotionProperties) bool {
        return self.allow_sleeping;
    }

    /// Get world space linear velocity of the center of mass
    pub fn getLinearVelocity(self: *const MotionProperties) Vec3 {
        BodyAccess.assertVelocityAccess(.read);
        return self.linear_velocity;
    }

    /// Set world space linear velocity of the center of mass.
    /// Note that it is illegal to set a non-zero linear velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
    pub fn setLinearVelocity(self: *MotionProperties, linear_velocity: Vec3) void {
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) std.debug.assert(linear_velocity.length() <= self.max_linear_velocity);
        self.linear_velocity = self.lockTranslation(linear_velocity);
    }

    /// Set world space linear velocity of the center of mass, will make sure the value is clamped against the maximum linear velocity.
    /// Note that it is illegal to set a non-zero linear velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
    pub fn setLinearVelocityClamped(self: *MotionProperties, linear_velocity: Vec3) void {
        self.linear_velocity = self.lockTranslation(linear_velocity);
        self.clampLinearVelocity();
    }

    /// Get world space angular velocity of the center of mass
    pub fn getAngularVelocity(self: *const MotionProperties) Vec3 {
        BodyAccess.assertVelocityAccess(.read);
        return self.angular_velocity;
    }

    /// Set world space angular velocity of the center of mass.
    /// Note that it is illegal to set a non-zero angular velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
    pub fn setAngularVelocity(self: *MotionProperties, angular_velocity: Vec3) void {
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) std.debug.assert(angular_velocity.length() <= self.max_angular_velocity);
        self.angular_velocity = self.lockAngular(angular_velocity);
    }

    /// Set world space angular velocity of the center of mass, will make sure the value is clamped against the maximum angular velocity.
    /// Note that it is illegal to set a non-zero angular velocity on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
    pub fn setAngularVelocityClamped(self: *MotionProperties, angular_velocity: Vec3) void {
        self.angular_velocity = self.lockAngular(angular_velocity);
        self.clampAngularVelocity();
    }

    /// Set velocity of body such that it will be rotate/translate by delta_position/rotation in delta_time seconds.
    /// Note that it is illegal to provide non-zero delta position/rotation on a sleeping body without waking it up afterwards.
    /// If you want the body to wake up when it is sleeping, use BodyInterface::MoveKinematic instead.
    pub fn moveKinematic(self: *MotionProperties, delta_position: Vec3, delta_rotation: Quat, delta_time: f32) void {
        BodyAccess.assertVelocityAccess(.read_write);
        BodyAccess.assertPositionAccess(.read);
        if (Core.enable_asserts) std.debug.assert(self.cached_body_type == .rigid_body);
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type != .static);

        // Calculate required linear velocity
        self.linear_velocity = self.lockTranslation(delta_position.divScalar(delta_time));

        // Calculate required angular velocity
        self.angular_velocity = self.lockAngular(delta_rotation.getAngularVelocity(delta_time));
    }

    ///@name Velocity limits
    /// Maximum linear velocity that a body can achieve. Used to prevent the system from exploding.
    pub fn getMaxLinearVelocity(self: *const MotionProperties) f32 {
        return self.max_linear_velocity;
    }

    pub fn setMaxLinearVelocity(self: *MotionProperties, linear_velocity: f32) void {
        if (Core.enable_asserts) std.debug.assert(linear_velocity >= 0.0);
        self.max_linear_velocity = linear_velocity;
    }

    /// Maximum angular velocity that a body can achieve. Used to prevent the system from exploding.
    pub fn getMaxAngularVelocity(self: *const MotionProperties) f32 {
        return self.max_angular_velocity;
    }

    pub fn setMaxAngularVelocity(self: *MotionProperties, angular_velocity: f32) void {
        if (Core.enable_asserts) std.debug.assert(angular_velocity >= 0.0);
        self.max_angular_velocity = angular_velocity;
    }

    /// Clamp velocity according to limit
    pub fn clampLinearVelocity(self: *MotionProperties) void {
        BodyAccess.assertVelocityAccess(.read_write);

        const len_sq = self.linear_velocity.lengthSq();
        if (Core.enable_asserts) std.debug.assert(std.math.isFinite(len_sq));
        if (len_sq > math.square(self.max_linear_velocity))
            self.linear_velocity = self.linear_velocity.mulScalar(self.max_linear_velocity / @sqrt(len_sq));
    }

    /// Clamp velocity according to limit
    pub fn clampAngularVelocity(self: *MotionProperties) void {
        BodyAccess.assertVelocityAccess(.read_write);

        const len_sq = self.angular_velocity.lengthSq();
        if (Core.enable_asserts) std.debug.assert(std.math.isFinite(len_sq));
        if (len_sq > math.square(self.max_angular_velocity))
            self.angular_velocity = self.angular_velocity.mulScalar(self.max_angular_velocity / @sqrt(len_sq));
    }

    /// Get linear damping: dv/dt = -c * v. c. Value should be zero or positive and is usually close to 0.
    pub fn getLinearDamping(self: *const MotionProperties) f32 {
        return self.linear_damping;
    }

    pub fn setLinearDamping(self: *MotionProperties, linear_damping: f32) void {
        if (Core.enable_asserts) std.debug.assert(linear_damping >= 0.0);
        self.linear_damping = linear_damping;
    }

    /// Get angular damping: dw/dt = -c * w. c. Value should be zero or positive and is usually close to 0.
    pub fn getAngularDamping(self: *const MotionProperties) f32 {
        return self.angular_damping;
    }

    pub fn setAngularDamping(self: *MotionProperties, angular_damping: f32) void {
        if (Core.enable_asserts) std.debug.assert(angular_damping >= 0.0);
        self.angular_damping = angular_damping;
    }

    /// Get gravity factor (1 = normal gravity, 0 = no gravity)
    pub fn getGravityFactor(self: *const MotionProperties) f32 {
        return self.gravity_factor;
    }

    pub fn setGravityFactor(self: *MotionProperties, gravity_factor: f32) void {
        self.gravity_factor = gravity_factor;
    }

    /// Set the mass and inertia tensor
    pub fn setMassProperties(self: *MotionProperties, allowed_dofs: AllowedDOFs, mass_properties: *const MassProperties) void {
        // Store allowed DOFs
        self.allowed_dofs = allowed_dofs;

        // Decompose DOFs
        const allowed_translation_axis: u32 = allowed_dofs.toInt() & 0b111;
        const allowed_rotation_axis: u32 = (allowed_dofs.toInt() >> 3) & 0b111;

        // Set inverse mass
        if (allowed_translation_axis == 0) {
            // No translation possible
            self.inv_mass = 0.0;
        } else {
            // Invalid mass. Some shapes like MeshShape or TriangleShape cannot calculate mass automatically, in this case
            // you need to provide it by setting BodyCreationSettings::mOverrideMassProperties and mMassPropertiesOverride.
            if (Core.enable_asserts) std.debug.assert(mass_properties.mass > 0.0);
            self.inv_mass = 1.0 / mass_properties.mass;
        }

        if (allowed_rotation_axis == 0) {
            // No rotation possible
            self.inv_inertia_diagonal = Vec3.zero();
            self.inertia_rotation = Quat.identity();
        } else {
            // Set inverse inertia
            const principal = mass_properties.decomposePrincipalMomentsOfInertia();
            if (principal != null and !principal.?.diagonal.isNearZero(.{})) {
                self.inv_inertia_diagonal = principal.?.diagonal.reciprocal();
                self.inertia_rotation = principal.?.rotation.getQuaternion();
            } else {
                // Failed! Fall back to inertia tensor of sphere with radius 1.
                self.inv_inertia_diagonal = Vec3.replicate(2.5 * self.inv_mass);
                self.inertia_rotation = Quat.identity();
            }
        }

        // Can't lock all axes, use a static body for this. This will crash with a division by zero later!
        if (Core.enable_asserts) std.debug.assert(self.inv_mass != 0.0 or !self.inv_inertia_diagonal.eql(Vec3.zero()));
    }

    /// Get inverse mass (1 / mass). Should only be called on a dynamic object (static or kinematic bodies have infinite mass so should be treated as 1 / mass = 0)
    pub fn getInverseMass(self: *const MotionProperties) f32 {
        if (Core.enable_asserts) Core.assertRecoverable(self.cached_motion_type == .dynamic, "mCachedMotionType == EMotionType::Dynamic", @src());
        return self.inv_mass;
    }

    pub fn getInverseMassUnchecked(self: *const MotionProperties) f32 {
        return self.inv_mass;
    }

    /// Set the inverse mass (1 / mass).
    /// Note that mass and inertia are linearly related (e.g. inertia of a sphere with mass m and radius r is \f$2/5 \: m \: r^2\f$).
    /// If you change mass, inertia should probably change as well. You can use ScaleToMass to update mass and inertia at the same time.
    /// If all your translation degrees of freedom are restricted, make sure this is zero (see EAllowedDOFs).
    pub fn setInverseMass(self: *MotionProperties, inverse_mass: f32) void {
        self.inv_mass = inverse_mass;
    }

    /// Diagonal of inverse inertia matrix: D. Should only be called on a dynamic object (static or kinematic bodies have infinite mass so should be treated as D = 0)
    pub fn getInverseInertiaDiagonal(self: *const MotionProperties) Vec3 {
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type == .dynamic);
        return self.inv_inertia_diagonal;
    }

    /// Rotation (R) that takes inverse inertia diagonal to local space: \f$I_{body}^{-1} = R \: D \: R^{-1}\f$
    pub fn getInertiaRotation(self: *const MotionProperties) Quat {
        return self.inertia_rotation;
    }

    /// Set the inverse inertia tensor in local space by setting the diagonal and the rotation: \f$I_{body}^{-1} = R \: D \: R^{-1}\f$.
    /// Note that mass and inertia are linearly related (e.g. inertia of a sphere with mass m and radius r is \f$2/5 \: m \: r^2\f$).
    /// If you change inertia, mass should probably change as well. You can use ScaleToMass to update mass and inertia at the same time.
    /// If all your rotation degrees of freedom are restricted, make sure this is zero (see EAllowedDOFs).
    pub fn setInverseInertia(self: *MotionProperties, diagonal: Vec3, rot: Quat) void {
        self.inv_inertia_diagonal = diagonal;
        self.inertia_rotation = rot;
    }

    /// Sets the mass to mass and scale the inertia tensor based on the ratio between the old and new mass.
    /// Note that this only works when the current mass is finite (i.e. the body is dynamic and translational degrees of freedom are not restricted).
    pub fn scaleToMass(self: *MotionProperties, mass: f32) void {
        if (Core.enable_asserts) std.debug.assert(self.inv_mass > 0.0); // Body must have finite mass
        if (Core.enable_asserts) std.debug.assert(mass > 0.0); // New mass cannot be zero

        const new_inv_mass = 1.0 / mass;
        self.inv_inertia_diagonal = self.inv_inertia_diagonal.mulScalar(new_inv_mass / self.inv_mass);
        self.inv_mass = new_inv_mass;
    }

    /// Get inverse inertia matrix (\f$I_{body}^{-1}\f$). Will be a matrix of zeros for a static or kinematic object.
    pub fn getLocalSpaceInverseInertia(self: *const MotionProperties) Mat44 {
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type == .dynamic);
        return self.getLocalSpaceInverseInertiaUnchecked();
    }

    /// Same as GetLocalSpaceInverseInertia() but doesn't check if the body is dynamic
    pub fn getLocalSpaceInverseInertiaUnchecked(self: *const MotionProperties) Mat44 {
        const rotation = Mat44.rotationQuat(self.inertia_rotation);
        const rotation_mul_scale_transposed = Mat44.init(self.inv_inertia_diagonal.splatX().mul(rotation.getColumn4(0)), self.inv_inertia_diagonal.splatY().mul(rotation.getColumn4(1)), self.inv_inertia_diagonal.splatZ().mul(rotation.getColumn4(2)), Vec4.init(0, 0, 0, 1));
        return rotation.multiply3x3RightTransposed(rotation_mul_scale_transposed);
    }

    /// Get inverse inertia matrix (\f$I^{-1}\f$) for a given object rotation (translation will be ignored). Zero if object is static or kinematic.
    pub fn getInverseInertiaForRotation(self: *const MotionProperties, rotation_matrix: Mat44) Mat44 {
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type == .dynamic);

        const rotation = rotation_matrix.multiply3x3Mat44(Mat44.rotationQuat(self.inertia_rotation));
        const rotation_mul_scale_transposed = Mat44.init(self.inv_inertia_diagonal.splatX().mul(rotation.getColumn4(0)), self.inv_inertia_diagonal.splatY().mul(rotation.getColumn4(1)), self.inv_inertia_diagonal.splatZ().mul(rotation.getColumn4(2)), Vec4.init(0, 0, 0, 1));
        var inverse_inertia = rotation.multiply3x3RightTransposed(rotation_mul_scale_transposed);

        // We need to mask out both the rows and columns of DOFs that are not allowed
        const angular_dofs_mask = self.getAngularDOFsMask().reinterpretAsFloat();
        inverse_inertia.setColumn4(0, Vec4.bitAnd(inverse_inertia.getColumn4(0), Vec4.bitAnd(angular_dofs_mask, angular_dofs_mask.splatX())));
        inverse_inertia.setColumn4(1, Vec4.bitAnd(inverse_inertia.getColumn4(1), Vec4.bitAnd(angular_dofs_mask, angular_dofs_mask.splatY())));
        inverse_inertia.setColumn4(2, Vec4.bitAnd(inverse_inertia.getColumn4(2), Vec4.bitAnd(angular_dofs_mask, angular_dofs_mask.splatZ())));

        return inverse_inertia;
    }

    /// Multiply a vector with the inverse world space inertia tensor (\f$I_{world}^{-1}\f$). Zero if object is static or kinematic.
    pub fn multiplyWorldSpaceInverseInertiaByVector(self: *const MotionProperties, body_rotation: Quat, v_in: Vec3) Vec3 {
        if (Core.enable_asserts) Core.assertRecoverable(self.cached_motion_type == .dynamic, "mCachedMotionType == EMotionType::Dynamic", @src());

        // Mask out columns of DOFs that are not allowed
        const angular_dofs_mask = Vec3.fromVec4(self.getAngularDOFsMask().reinterpretAsFloat());
        const v = Vec3.bitAnd(v_in, angular_dofs_mask);

        // Multiply vector by inverse inertia
        const rotation = Mat44.rotationQuat(body_rotation.mul(self.inertia_rotation));
        const result = rotation.multiply3x3(self.inv_inertia_diagonal.mul(rotation.multiply3x3Transposed(v)));

        // Mask out rows of DOFs that are not allowed
        return Vec3.bitAnd(result, angular_dofs_mask);
    }

    /// Velocity of point point_relative_to_com (in center of mass space, e.g. on the surface of the body) of the body (unit: m/s)
    pub fn getPointVelocityCOM(self: *const MotionProperties, point_relative_to_com: Vec3) Vec3 {
        return self.linear_velocity.add(self.angular_velocity.cross(point_relative_to_com));
    }

    // Get the total amount of force applied to the center of mass this time step (through Body::AddForce calls). Note that it will reset to zero after PhysicsSystem::Update.
    pub fn getAccumulatedForce(self: *const MotionProperties) Vec3 {
        return Vec3.loadFloat3Unsafe(&self.force);
    }

    // Get the total amount of torque applied to the center of mass this time step (through Body::AddForce/Body::AddTorque calls). Note that it will reset to zero after PhysicsSystem::Update.
    pub fn getAccumulatedTorque(self: *const MotionProperties) Vec3 {
        return Vec3.loadFloat3Unsafe(&self.torque);
    }

    // Reset the total accumulated force, note that this will be done automatically after every time step.
    pub fn resetForce(self: *MotionProperties) void {
        self.force = .{ .x = 0, .y = 0, .z = 0 };
    }

    // Reset the total accumulated torque, note that this will be done automatically after every time step.
    pub fn resetTorque(self: *MotionProperties) void {
        self.torque = .{ .x = 0, .y = 0, .z = 0 };
    }

    // Reset the current velocity and accumulated force and torque.
    pub fn resetMotion(self: *MotionProperties) void {
        BodyAccess.assertVelocityAccess(.read_write);
        self.linear_velocity = Vec3.zero();
        self.angular_velocity = Vec3.zero();
        self.force = .{ .x = 0, .y = 0, .z = 0 };
        self.torque = .{ .x = 0, .y = 0, .z = 0 };
    }

    /// Returns a vector where the linear components that are not allowed by mAllowedDOFs are set to 0 and the rest to 0xffffffff
    pub fn getLinearDOFsMask(self: *const MotionProperties) UVec4 {
        const mask = UVec4.init(AllowedDOFs.toInt(.{ .translation_x = true }), AllowedDOFs.toInt(.{ .translation_y = true }), AllowedDOFs.toInt(.{ .translation_z = true }), 0);
        return UVec4.equals(UVec4.bitAnd(UVec4.replicate(self.allowed_dofs.toInt()), mask), mask);
    }

    /// Takes a translation vector v and returns a vector where the components that are not allowed by mAllowedDOFs are set to 0
    pub fn lockTranslation(self: *const MotionProperties, v: Vec3) Vec3 {
        return Vec3.bitAnd(v, Vec3.fromVec4(self.getLinearDOFsMask().reinterpretAsFloat()));
    }

    /// Returns a vector where the angular components that are not allowed by mAllowedDOFs are set to 0 and the rest to 0xffffffff
    pub fn getAngularDOFsMask(self: *const MotionProperties) UVec4 {
        const mask = UVec4.init(AllowedDOFs.toInt(.{ .rotation_x = true }), AllowedDOFs.toInt(.{ .rotation_y = true }), AllowedDOFs.toInt(.{ .rotation_z = true }), 0);
        return UVec4.equals(UVec4.bitAnd(UVec4.replicate(self.allowed_dofs.toInt()), mask), mask);
    }

    /// Takes an angular velocity / torque vector v and returns a vector where the components that are not allowed by mAllowedDOFs are set to 0
    pub fn lockAngular(self: *const MotionProperties, v: Vec3) Vec3 {
        return Vec3.bitAnd(v, Vec3.fromVec4(self.getAngularDOFsMask().reinterpretAsFloat()));
    }

    /// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    pub fn setNumVelocityStepsOverride(self: *MotionProperties, n: u32) void {
        if (Core.enable_asserts) std.debug.assert(n < 256);
        self.num_velocity_steps_override = @truncate(n);
    }

    pub fn getNumVelocityStepsOverride(self: *const MotionProperties) u32 {
        return self.num_velocity_steps_override;
    }

    /// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    pub fn setNumPositionStepsOverride(self: *MotionProperties, n: u32) void {
        if (Core.enable_asserts) std.debug.assert(n < 256);
        self.num_position_steps_override = @truncate(n);
    }

    pub fn getNumPositionStepsOverride(self: *const MotionProperties) u32 {
        return self.num_position_steps_override;
    }

    /// Stats for this body (JPH_TRACK_SIMULATION_STATS: only declared when `track_simulation_stats` is on)
    pub const getSimulationStats = if (track_simulation_stats) getSimulationStatsImpl else {};

    /// Stats for this body, non-const overload (only declared when `track_simulation_stats` is on)
    pub const getSimulationStatsMut = if (track_simulation_stats) getSimulationStatsMutImpl else {};

    fn getSimulationStatsImpl(self: *const MotionProperties) *const SimulationStats {
        return &self.simulation_stats;
    }

    fn getSimulationStatsMutImpl(self: *MotionProperties) *SimulationStats {
        return &self.simulation_stats;
    }

    ////////////////////////////////////////////////////////////
    // FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
    ////////////////////////////////////////////////////////////

    ///@name Update linear and angular velocity (used during constraint solving)
    pub fn applyLinearVelocityStep(self: *MotionProperties, linear_velocity: Vec3) void {
        if (DeterminismLog.enabled) DeterminismLog.log(.{ "ApplyLinearVelocityStep: ", linear_velocity });
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) Core.assertRecoverable(self.cached_motion_type == .dynamic, "mCachedMotionType == EMotionType::Dynamic", @src());
        if (Core.enable_asserts) std.debug.assert(!linear_velocity.isNaN());
        self.linear_velocity = self.lockTranslation(linear_velocity);
    }

    pub fn addLinearVelocityStep(self: *MotionProperties, linear_velocity_change: Vec3) void {
        self.applyLinearVelocityStep(self.linear_velocity.add(linear_velocity_change));
    }

    pub fn subLinearVelocityStep(self: *MotionProperties, linear_velocity_change: Vec3) void {
        self.applyLinearVelocityStep(self.linear_velocity.sub(linear_velocity_change));
    }

    pub fn applyAngularVelocityStep(self: *MotionProperties, angular_velocity: Vec3) void {
        if (DeterminismLog.enabled) DeterminismLog.log(.{ "ApplyAngularVelocityStep: ", angular_velocity });
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) Core.assertRecoverable(self.cached_motion_type == .dynamic, "mCachedMotionType == EMotionType::Dynamic", @src());
        if (Core.enable_asserts) std.debug.assert(!angular_velocity.isNaN());
        self.angular_velocity = angular_velocity;
    }

    pub fn addAngularVelocityStep(self: *MotionProperties, angular_velocity_change: Vec3) void {
        self.applyAngularVelocityStep(self.angular_velocity.add(angular_velocity_change));
    }

    pub fn subAngularVelocityStep(self: *MotionProperties, angular_velocity_change: Vec3) void {
        self.applyAngularVelocityStep(self.angular_velocity.sub(angular_velocity_change));
    }

    /// Apply the gyroscopic force (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
    pub fn applyGyroscopicForceInternal(self: *MotionProperties, body_rotation: Quat, delta_time: f32) void {
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) std.debug.assert(self.cached_body_type == .rigid_body);
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type == .dynamic);

        // Calculate local space inertia tensor (a diagonal in local space)
        const is_zero = Vec3.equals(self.inv_inertia_diagonal, Vec3.zero());
        const denominator = Vec3.select(self.inv_inertia_diagonal, Vec3.one(), is_zero);
        const nominator = Vec3.select(Vec3.one(), Vec3.zero(), is_zero);
        const local_inertia = nominator.div(denominator); // Avoid dividing by zero, inertia in this axis will be zero

        // Calculate local space angular momentum
        const inertia_space_to_world_space = body_rotation.mul(self.inertia_rotation);
        const local_angular_velocity = inertia_space_to_world_space.inverseRotate(self.angular_velocity);
        const local_momentum = local_inertia.mul(local_angular_velocity);

        // The gyroscopic force applies a torque: T = -w x I w where w is angular velocity and I the inertia tensor
        // Calculate the new angular momentum by applying the gyroscopic force and make sure the new magnitude is the same as the old one
        // to avoid introducing energy into the system due to the Euler step
        var new_local_momentum = local_momentum.sub(local_angular_velocity.cross(local_momentum).mulScalar(delta_time));
        const new_local_momentum_len_sq = new_local_momentum.lengthSq();
        new_local_momentum = if (new_local_momentum_len_sq > 0.0) new_local_momentum.mulScalar(@sqrt(local_momentum.lengthSq() / new_local_momentum_len_sq)) else Vec3.zero();

        // Convert back to world space angular velocity
        self.angular_velocity = inertia_space_to_world_space.mulVec3(self.inv_inertia_diagonal.mul(new_local_momentum));
    }

    /// Apply all accumulated forces, torques and drag (should only be called by the PhysicsSystem)
    pub fn applyForceTorqueAndDragInternal(self: *MotionProperties, body_rotation: Quat, gravity: Vec3, delta_time: f32) void {
        BodyAccess.assertVelocityAccess(.read_write);
        if (Core.enable_asserts) std.debug.assert(self.cached_body_type == .rigid_body);
        if (Core.enable_asserts) std.debug.assert(self.cached_motion_type == .dynamic);

        // Update linear velocity
        self.linear_velocity = self.lockTranslation(self.linear_velocity.add(gravity.mulScalar(self.gravity_factor).add(self.getAccumulatedForce().mulScalar(self.inv_mass)).mulScalar(delta_time)));

        // Update angular velocity
        self.angular_velocity = self.angular_velocity.add(self.multiplyWorldSpaceInverseInertiaByVector(body_rotation, self.getAccumulatedTorque()).mulScalar(delta_time));

        // Linear damping: dv/dt = -c * v
        // Solution: v(t) = v(0) * e^(-c * t) or v2 = v1 * e^(-c * dt)
        // Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
        // Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
        self.linear_velocity = self.linear_velocity.mulScalar(math.max(@as(f32, 0.0), 1.0 - self.linear_damping * delta_time));
        self.angular_velocity = self.angular_velocity.mulScalar(math.max(@as(f32, 0.0), 1.0 - self.angular_damping * delta_time));

        // Clamp velocities
        self.clampLinearVelocity();
        self.clampAngularVelocity();
    }

    /// Access to the island index
    pub fn getIslandIndexInternal(self: *const MotionProperties) u32 {
        return self.island_index;
    }

    pub fn setIslandIndexInternal(self: *MotionProperties, index: u32) void {
        self.island_index = index;
    }

    /// Access to the index in the active bodies array
    pub fn getIndexInActiveBodiesInternal(self: *const MotionProperties) u32 {
        return self.index_in_active_bodies.load(.monotonic);
    }

    /// The offset that the sleep test spheres are relative to (JPH_DOUBLE_PRECISION: only declared in double precision)
    pub const getSleepTestOffset = if (Core.double_precision) getSleepTestOffsetImpl else {};

    fn getSleepTestOffsetImpl(self: *const MotionProperties) DVec3 {
        return DVec3.loadDouble3Unsafe(&self.sleep_test_offset);
    }

    /// Reset spheres to center around points with radius 0
    pub fn resetSleepTestSpheres(self: *MotionProperties, points: *const [3]RVec3) void {
        if (Core.double_precision) {
            // Make spheres relative to the first point and initialize them to zero radius
            const offset: DVec3 = points[0];
            offset.storeDouble3(&self.sleep_test_offset);
            self.sleep_test_spheres[0] = Sphere.init(Vec3.zero(), 0.0);
            for (1..3) |i|
                self.sleep_test_spheres[i] = Sphere.init(points[i].sub(offset).toVec3(), 0.0);
        } else {
            // Initialize the spheres to zero radius around the supplied points
            for (0..3) |i|
                self.sleep_test_spheres[i] = Sphere.init(points[i], 0.0);
        }

        self.sleep_test_timer = 0.0;
    }

    /// Reset the sleep test timer without resetting the sleep test spheres
    pub fn resetSleepTestTimer(self: *MotionProperties) void {
        self.sleep_test_timer = 0.0;
    }

    /// Accumulate sleep time and return if a body can go to sleep
    pub fn accumulateSleepTime(self: *MotionProperties, delta_time: f32, time_before_sleep: f32) CanSleep {
        self.sleep_test_timer += delta_time;
        return if (self.sleep_test_timer >= time_before_sleep) .can_sleep else .cannot_sleep;
    }

    /// Saving state for replay
    pub fn saveState(self: *const MotionProperties, stream: *StateRecorder) void {
        // Only write properties that can change at runtime
        stream.write(self.linear_velocity);
        stream.write(self.angular_velocity);
        stream.write(self.force);
        stream.write(self.torque);
        if (Core.double_precision)
            stream.write(self.sleep_test_offset);
        stream.write(self.sleep_test_spheres);
        stream.write(self.sleep_test_timer);
        stream.write(self.allow_sleeping);
    }

    /// Restoring state for replay
    pub fn restoreState(self: *MotionProperties, stream: *StateRecorder) void {
        stream.read(&self.linear_velocity);
        stream.read(&self.angular_velocity);
        stream.read(&self.force);
        stream.read(&self.torque);
        if (Core.double_precision)
            stream.read(&self.sleep_test_offset);
        stream.read(&self.sleep_test_spheres);
        stream.read(&self.sleep_test_timer);
        stream.read(&self.allow_sleeping);
    }
};
