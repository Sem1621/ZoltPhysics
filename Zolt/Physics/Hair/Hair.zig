//! Port of: Jolt/Physics/Hair/Hair.h, Jolt/Physics/Hair/Hair.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R14, R20:
//! - A plain struct (Jolt: NonCopyable, not a RefTarget). `init(allocator, settings, position, rotation, layer)` is
//!   Jolt's constructor and `deinit` its destructor. Jolt's `Init` is `initCompute`. The allocator frees the read back
//!   arrays and holds Jolt's JPH_STACK_ALLOC arrays during `update` (they are freed before `update` returns).
//! - Hair runs on the caller's thread outside PhysicsSystem.update, like CharacterVirtual. It reads bodies through the
//!   locking interfaces and never writes them. Never update a hair from a step listener.
//! - Collision data comes from ConvexHullShape leaves only (compound children included, with their scale), like Jolt.
//! - The render position override (`std::function`) is a `{ context, function }` callback (RenderPositionsToFloat3).
//! - The C++ getters that return a pointer return an optional slice here: null until ReadBackGPUState and
//!   LockReadBackBuffers make the data valid.
//! - JPH_PROFILE scopes are dropped (Zolt has no profiler). Their blocks keep their braces and comments.
//! - Allocation failures (Jolt aborts): `initCompute`, `update`, `readBackGPUState` and `lockReadBackBuffers` return
//!   `error.OutOfMemory`.
//!   - `initCompute`: some buffers may be set. Call it again before `update`.
//!   - `update`: the step is partly done. Some shaders may have run, and the previous position and rotation are updated.
//!     Call `onTeleported` to put the hair back in its default pose at the next update. The buffers that `update`
//!     creates on the way are kept only when they are complete: the iteration constant buffers grow one buffer at a
//!     time, and the two target buffers are set together.
//!   - `readBackGPUState` sets the five read back buffers together. `lockReadBackBuffers` allocates the render positions
//!     of an override before it maps a buffer, so a failure leaves nothing mapped.
//! - Inputs where Jolt is undefined (R20; Zolt-only tests pin them):
//!   - UB3: collision is enabled and there are leaf shapes in the bounds, but none is a convex hull. Jolt creates a
//!     collision shapes buffer of 0 elements, and HairUpdateVelocity reads element 0. Zolt creates `max(n, 1)` zeroed
//!     elements, like the planes and vertices buffers.
//!   - UB4: the buffers that Hair creates without data are zero (ComputeBufferCPU). Jolt leaves them uninitialized.
//!   - A HairShaders that misses a shader (HairShaders.init without registerHairShaders): Jolt binds a null shader and
//!     crashes. Zolt panics with a message.
//! - Jolt's quirk (R20): mScalpJointMatricesCB is 64 times too large, its size counts bytes where it should count
//!   matrices. Only memory is lost.
//! - JPH_DEBUG_RENDERER (ERenderStrandColor, DrawSettings, Draw): TODO(debug_renderer), group H3. Until then `draw` is a
//!   gated placeholder that panics.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const LibM = @import("../../Math/LibM.zig");
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const StridedPtrConst = @import("../../Core/StridedPtr.zig").StridedPtrConst;
const Float3 = @import("../../Math/Float3.zig").Float3;
const Float4 = @import("../../Math/Float4.zig").Float4;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Real = @import("../../Math/Real.zig");
const RVec3 = Real.RVec3;
const RMat44 = Real.RMat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const ComputeSystem = @import("../../Compute/ComputeSystem.zig").ComputeSystem;
const ComputeBuffer = @import("../../Compute/ComputeBuffer.zig").ComputeBuffer;
const ComputeQueue = @import("../../Compute/ComputeQueue.zig").ComputeQueue;
const ComputeShader = @import("../../Compute/ComputeShader.zig").ComputeShader;
const hlsl = @import("../../Compute/CPU/HLSLToCPP.zig");
const HairStructs = @import("../../Shaders/HairStructs.zig");
const HairSettings = @import("HairSettings.zig").HairSettings;
const HairShaders = @import("HairShaders.zig").HairShaders;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const Body = @import("../Body/Body.zig").Body;
const BodyLockRead = @import("../Body/BodyLock.zig").BodyLockRead;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const ShapeFilter = @import("../Collision/ShapeFilter.zig").ShapeFilter;
const TransformedShape = @import("../Collision/TransformedShape.zig").TransformedShape;
const CollideShapeBodyCollector = @import("../Collision/BroadPhase/BroadPhaseQuery.zig").CollideShapeBodyCollector;
const ConvexHullShape = @import("../Collision/Shape/ConvexHullShape.zig").ConvexHullShape;
const ScaleHelpers = @import("../Collision/Shape/ScaleHelpers.zig");
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;

const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairCollisionPlane = HairStructs.HairCollisionPlane;
const HairCollisionShape = HairStructs.HairCollisionShape;
const HairMaterial = HairStructs.HairMaterial;
const HairUpdateContext = HairStructs.HairUpdateContext;
const HairIterationContext = HairStructs.HairIterationContext;

/// Hair simulation instance
///
/// Note that this system is currently still in development, it is missing important features like:
///
/// - Level of detail
/// - Wind forces
/// - Advection step for the grid velocity field
/// - Support for collision detection against shapes other than ConvexHullShape
/// - The Gradient class is very limited and will be replaced by a texture lookup
/// - Gravity preload factor is not fully functioning yet
/// - It is wasteful of memory (e.g. stores everything both on CPU and GPU)
/// - Only supports a single neutral pose to drive towards
/// - It could use further optimizations
pub const Hair = struct {
    /// Function that converts the render positions buffer to Float3 vertices for debugging purposes. It maps an application defined format to Float3. Third parameter is the number of vertices.
    /// (Jolt: std::function<void(ComputeBuffer *, Float3 *, uint)>. The vertices are a slice: its length is the number of vertices.)
    pub const RenderPositionsToFloat3 = struct {
        /// Passed to `function` as its first argument
        context: ?*anyopaque = null,
        function: *const fn (context: ?*anyopaque, buffer: *ComputeBuffer, out_vertices: []Float3) void,

        pub fn call(self: RenderPositionsToFloat3, buffer: *ComputeBuffer, out_vertices: []Float3) void {
            self.function(self.context, buffer, out_vertices);
        }
    };

    const GradientSampler = HairSettings.GradientSampler;

    /// Information about a colliding shape. Is always a leaf shape, compound shapes are expanded. (Protected in Jolt.)
    pub const LeafShape = struct {
        transform: Mat44,
        scale: Vec3,
        linear_velocity: Vec3,
        angular_velocity: Vec3,
        shape: RefConst(Shape) = .empty,

        /// Constructor LeafShape(inTransform, inScale, inLinearVelocity, inAngularVelocity, inShape) (adds a reference to the shape)
        pub fn init(transform: Mat44, scale: Vec3, linear_velocity: Vec3, angular_velocity: Vec3, shape: *const Shape) LeafShape {
            return .{ .transform = transform, .scale = scale, .linear_velocity = linear_velocity, .angular_velocity = angular_velocity, .shape = .init(shape) };
        }

        /// Destructor (releases the shape)
        pub fn deinit(self: *LeafShape) void {
            self.shape.deinit();
        }
    };

    /// Internal context used during a simulation step. (Protected in Jolt.) InitializeContext sets every member.
    pub const UpdateContext = struct {
        /// Transforms positions from the old hair transform to the new
        delta_transform: Mat44 = undefined,
        /// Rotation part of mDeltaTransform
        delta_transform_quat: Quat = undefined,
        /// Number of iterations to run the solver for
        num_iterations: u32 = undefined,
        /// If collision detection should be performed
        needs_collision: bool = undefined,
        /// If the grid should be calculated
        needs_grid: bool = undefined,
        /// If no simulation is needed and only the global pose needs to be applied
        global_pose_only: bool = undefined,
        /// If the world transform has changed
        has_transform_changed: bool = undefined,
        /// Delta time for a sub step
        delta_time: f32 = undefined,
        /// 0.5 * mDeltaTime
        half_delta_time: f32 = undefined,
        /// 1 / mDeltaTime^2
        inv_delta_time_sq: f32 = undefined,
        /// 2 / mDeltaTime
        two_div_delta_time: f32 = undefined,
        /// Ratio between sub step delta time and default sub step delta time
        time_ratio: f32 = undefined,
        /// Gravity to apply in a sub step
        sub_step_gravity: Vec3 = undefined,
        /// List of colliding shapes
        shapes: std.ArrayList(LeafShape) = .empty,

        /// Zolt: releases the shapes and frees the list (`allocator` is the one of the Hair)
        pub fn deinit(self: *UpdateContext, allocator: Allocator) void {
            for (self.shapes.items) |*s| s.deinit();
            self.shapes.deinit(allocator);
        }
    };

    /// Zolt: allocates the read back arrays, the list of iteration constant buffers and the temporary arrays of `update`
    allocator: Allocator,

    /// Shared hair settings, must be kept alive during the lifetime of this hair instance
    settings: RefConst(HairSettings),

    /// Position at the start of the last time step
    prev_position: RVec3,
    /// Current position in world space
    position: RVec3,
    /// Rotation at the start of the last time step
    prev_rotation: Quat,
    /// Current rotation in world space
    rotation: Quat,
    /// If the hair got teleported and should be set to the default pose
    teleported: bool = true,
    /// Layer for the hair to collide with
    layer: ObjectLayer,

    /// When skipping skinning, this allow specifying a transform that transforms the scalp mesh into head space
    scalp_to_head: Mat44 = .identity(),

    /// Indicates that the render positions buffer is provided externally
    render_positions_overridden: bool = false,
    /// Function that transforms the render positions buffer to Float3 vertices for debugging purposes
    render_positions_to_float3: ?RenderPositionsToFloat3 = null,

    scalp_joint_matrices_cb: Ref(ComputeBuffer) = .empty,
    scalp_vertices_cb: Ref(ComputeBuffer) = .empty,
    scalp_triangles_cb: Ref(ComputeBuffer) = .empty,
    /// Target root positions determined by skinning (where we're interpolating to, eventually written to mPositionsCB)
    target_positions_cb: Ref(ComputeBuffer) = .empty,
    /// Target global pose transforms determined by skinning (where we're interpolating to, eventually written to mGlobalPoseTransformsCB)
    target_global_pose_transforms_cb: Ref(ComputeBuffer) = .empty,
    /// Current global pose transforms used for skinning the hairs
    global_pose_transforms_cb: Ref(ComputeBuffer) = .empty,
    shape_planes_cb: Ref(ComputeBuffer) = .empty,
    shape_vertices_cb: Ref(ComputeBuffer) = .empty,
    shape_indices_cb: Ref(ComputeBuffer) = .empty,
    collision_planes_cb: Ref(ComputeBuffer) = .empty,
    collision_shapes_cb: Ref(ComputeBuffer) = .empty,
    materials_cb: Ref(ComputeBuffer) = .empty,
    previous_positions_cb: Ref(ComputeBuffer) = .empty,
    positions_cb: Ref(ComputeBuffer) = .empty,
    velocities_cb: Ref(ComputeBuffer) = .empty,
    velocity_and_density_cb: Ref(ComputeBuffer) = .empty,
    constants_cb: Ref(ComputeBuffer) = .empty,
    iteration_constants_cb: std.ArrayList(Ref(ComputeBuffer)) = .empty,
    render_positions_cb: Ref(ComputeBuffer) = .empty,

    // Only valid after ReadBackGPUState has been called
    scalp_vertices_read_back_cb: Ref(ComputeBuffer) = .empty,
    positions_read_back_cb: Ref(ComputeBuffer) = .empty,
    velocities_read_back_cb: Ref(ComputeBuffer) = .empty,
    velocity_and_density_read_back_cb: Ref(ComputeBuffer) = .empty,
    render_positions_read_back_cb: Ref(ComputeBuffer) = .empty,
    /// Mapped by LockReadBackBuffers
    scalp_vertices: ?[]const Float3 = null,
    /// Owned (Jolt: new Float3 [])
    positions: ?[]Float3 = null,
    /// Owned (Jolt: new Quat [])
    rotations: ?[]Quat = null,
    /// Owned (Jolt: new JPH_HairVelocity [])
    velocities: ?[]HairVelocity = null,
    /// Mapped by LockReadBackBuffers
    velocity_and_density: ?[]const Float4 = null,
    /// Mapped by LockReadBackBuffers, or `render_positions_owned` when the render positions are overridden
    render_positions: ?[]const Float3 = null,
    /// Zolt: the array that Jolt allocates for mRenderPositions when the render positions are overridden (owned)
    render_positions_owned: ?[]Float3 = null,

    /// Constructor (adds a reference to the settings)
    pub fn init(allocator: Allocator, settings: *const HairSettings, position: RVec3, rotation: Quat, layer: ObjectLayer) Hair {
        return .{
            .allocator = allocator,
            .settings = .init(settings),
            .prev_position = position,
            .position = position,
            .prev_rotation = rotation,
            .rotation = rotation,
            .layer = layer,
        };
    }

    /// Destructor
    pub fn deinit(self: *Hair) void {
        const allocator = self.allocator;

        // Delete debug data
        if (self.positions) |p|
            allocator.free(p);
        if (self.rotations) |r|
            allocator.free(r);
        if (self.velocities) |v|
            allocator.free(v);
        if (self.render_positions_overridden) {
            if (self.render_positions_owned) |r|
                allocator.free(r);
        }

        // Zolt: the members (Jolt's Ref destructors)
        self.scalp_joint_matrices_cb.deinit();
        self.scalp_vertices_cb.deinit();
        self.scalp_triangles_cb.deinit();
        self.target_positions_cb.deinit();
        self.target_global_pose_transforms_cb.deinit();
        self.global_pose_transforms_cb.deinit();
        self.shape_planes_cb.deinit();
        self.shape_vertices_cb.deinit();
        self.shape_indices_cb.deinit();
        self.collision_planes_cb.deinit();
        self.collision_shapes_cb.deinit();
        self.materials_cb.deinit();
        self.previous_positions_cb.deinit();
        self.positions_cb.deinit();
        self.velocities_cb.deinit();
        self.velocity_and_density_cb.deinit();
        self.constants_cb.deinit();
        for (self.iteration_constants_cb.items) |*b|
            b.deinit();
        self.iteration_constants_cb.deinit(allocator);
        self.render_positions_cb.deinit();
        self.scalp_vertices_read_back_cb.deinit();
        self.positions_read_back_cb.deinit();
        self.velocities_read_back_cb.deinit();
        self.velocity_and_density_read_back_cb.deinit();
        self.render_positions_read_back_cb.deinit();
        self.settings.deinit();
    }

    /// Initialize (Jolt: Init)
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). Some buffers may then be set: call it again
    /// before `update`.
    pub fn initCompute(self: *Hair, compute_system: *ComputeSystem) Allocator.Error!void {
        const settings = self.getHairSettings();

        // Create compute buffers
        const num_vertices_padded: usize = settings.getNumVerticesPadded();
        const grid_size: usize = settings.neutral_density.items.len;
        const num_render_vertices: usize = settings.render_vertices.items.len;

        if (settings.scalp_inverse_bind_pose.items.len != 0 and settings.scalp_vertices.items.len != 0) {
            // Jolt's size is the number of bytes of the matrices, not the number of matrices: the buffer is 64 times too large (R20, memory only)
            setBuffer(&self.scalp_joint_matrices_cb, try compute_system.createComputeBuffer(.upload_buffer, settings.scalp_inverse_bind_pose.items.len * @sizeOf(Mat44), @sizeOf(Mat44), null));
            setBuffer(&self.scalp_vertices_cb, try compute_system.createComputeBuffer(.rw_buffer, settings.scalp_vertices.items.len, @sizeOf(Float3), null));
            self.scalp_triangles_cb.set(settings.scalp_triangles_cb.get());
        }

        if (self.scalp_vertices_cb.get() != null) {
            setBuffer(&self.global_pose_transforms_cb, try compute_system.createComputeBuffer(.rw_buffer, settings.sim_strands.items.len, @sizeOf(HairGlobalPoseTransform), null));
        } else {
            // No vertices provided externally and none in settings, use identity transforms
            const identity: HairGlobalPoseTransform = .{
                .position = .init(0, 0, 0),
                .rotation = .init(0, 0, 0, 1),
            };
            const identity_array = try self.allocator.alloc(HairGlobalPoseTransform, settings.sim_strands.items.len);
            defer self.allocator.free(identity_array);
            @memset(identity_array, identity);
            setBuffer(&self.global_pose_transforms_cb, try compute_system.createComputeBuffer(.rw_buffer, settings.sim_strands.items.len, @sizeOf(HairGlobalPoseTransform), std.mem.sliceAsBytes(identity_array)));
        }

        setBuffer(&self.collision_planes_cb, try compute_system.createComputeBuffer(.rw_buffer, num_vertices_padded, @sizeOf(HairCollisionPlane), null));
        setBuffer(&self.materials_cb, try compute_system.createComputeBuffer(.upload_buffer, settings.materials.items.len, @sizeOf(HairMaterial), null));
        setBuffer(&self.previous_positions_cb, try compute_system.createComputeBuffer(.rw_buffer, num_vertices_padded, @sizeOf(HairPosition), null));
        setBuffer(&self.positions_cb, try compute_system.createComputeBuffer(.rw_buffer, num_vertices_padded, @sizeOf(HairPosition), null));
        setBuffer(&self.velocities_cb, try compute_system.createComputeBuffer(.rw_buffer, num_vertices_padded, @sizeOf(HairVelocity), null));
        setBuffer(&self.constants_cb, try compute_system.createComputeBuffer(.constant_buffer, 1, @sizeOf(HairUpdateContext), null));
        setBuffer(&self.velocity_and_density_cb, try compute_system.createComputeBuffer(.rw_buffer, grid_size, @sizeOf(Float4), null));
        if (!self.render_positions_overridden)
            setBuffer(&self.render_positions_cb, try compute_system.createComputeBuffer(.rw_buffer, num_render_vertices, @sizeOf(Float3), null));
    }

    /// `mFooCB = CreateComputeBuffer(...).Get()`: the new buffer replaces (and releases) the old one
    fn setBuffer(member: *Ref(ComputeBuffer), buffer: Ref(ComputeBuffer)) void {
        member.deinit();
        member.* = buffer;
    }

    /// Position and rotation of the hair in world space
    pub fn setPosition(self: *Hair, position: RVec3) void {
        self.position = position;
    }

    pub fn setRotation(self: *Hair, rotation: Quat) void {
        self.rotation = rotation;
    }

    pub fn getWorldTransform(self: *const Hair) RMat44 {
        return RMat44.rotationTranslation(self.rotation, self.position);
    }

    /// Access to the hair settings object which contains the configuration of the hair
    pub fn getHairSettings(self: *const Hair) *const HairSettings {
        return self.settings.get().?;
    }

    /// The hair will be initialized in its default pose with zero velocity at the new position and rotation during the next update
    pub fn onTeleported(self: *Hair) void {
        self.teleported = true;
    }

    /// Ability to externally provide the scalp vertices buffer. This allows skipping skinning the scalp during the simulation update. You may need to override JPH_SHADER_BIND_SCALP_VERTICES in HairSkinRootsBindings.h to match the format of the provided buffer.
    pub fn setScalpVerticesCB(self: *Hair, buffer: ?*ComputeBuffer) void {
        self.scalp_vertices_cb.set(buffer);
    }

    /// Ability to externally provide the scalp triangle indices buffer. This allows skipping skinning the scalp in during the simulation update. You may need to override JPH_SHADER_BIND_SCALP_TRIANGLES in HairSkinRootsBindings.h to match the format of the provided buffer.
    pub fn setScalpTrianglesCB(self: *Hair, buffer: ?*ComputeBuffer) void {
        self.scalp_triangles_cb.set(buffer);
    }

    /// When skipping skinning, this allow specifying a transform that transforms the scalp mesh into head space.
    pub fn setScalpToHead(self: *Hair, mat: Mat44) void {
        self.scalp_to_head = mat;
    }

    /// Enable externally set render vertices buffer (with potentially different vertex layout). Note that this also requires replacing the HairCalculateRenderPositions shader.
    pub fn overrideRenderPositionsCB(self: *Hair, render_positions_to_float3: RenderPositionsToFloat3) void {
        if (Core.enable_asserts) std.debug.assert(self.render_positions_cb.get() == null); // Must be called before Init
        self.render_positions_overridden = true;
        self.render_positions_to_float3 = render_positions_to_float3;
    }

    /// Allow setting the render vertices buffer externally in case it has special requirements for the calling application. You may need to override JPH_SHADER_BIND_RENDER_POSITIONS in HairCalculateRenderPositionsBindings.h to match the format of the provided buffer.
    pub fn setRenderPositionsCB(self: *Hair, buffer: ?*ComputeBuffer) void {
        if (Core.enable_asserts) std.debug.assert(self.render_positions_overridden); // Must call OverrideRenderPositionsCB first
        self.render_positions_cb.set(buffer);
    }

    /// Access to the resulting simulation data
    /// Skinned scalp vertices
    pub fn getScalpVerticesCB(self: *const Hair) ?*ComputeBuffer {
        return self.scalp_vertices_cb.get();
    }

    /// Skinned scalp triangle indices
    pub fn getScalpTrianglesCB(self: *const Hair) ?*ComputeBuffer {
        return self.scalp_triangles_cb.get();
    }

    /// Note transposed for better memory access
    pub fn getPositionsCB(self: *const Hair) ?*ComputeBuffer {
        return self.positions_cb.get();
    }

    /// Note transposed for better memory access
    pub fn getVelocitiesCB(self: *const Hair) ?*ComputeBuffer {
        return self.velocities_cb.get();
    }

    /// Velocity grid
    pub fn getVelocityAndDensityCB(self: *const Hair) ?*ComputeBuffer {
        return self.velocity_and_density_cb.get();
    }

    /// Render positions of the hair strands (see HairSettings::mRenderStrands to see where each strand starts and ends)
    pub fn getRenderPositionsCB(self: *const Hair) ?*ComputeBuffer {
        return self.render_positions_cb.get();
    }

    /// Access to the resulting simulation data (only valid when ReadBackGPUState has been called and the buffers have been locked)
    pub fn getScalpVertices(self: *const Hair) ?[]const Float3 {
        return self.scalp_vertices;
    }

    pub fn getPositions(self: *const Hair) ?[]const Float3 {
        return self.positions;
    }

    pub fn getRotations(self: *const Hair) ?[]const Quat {
        return self.rotations;
    }

    /// Zolt: a null StridedPtr before ReadBackGPUState (Jolt offsets a null pointer)
    pub fn getVelocities(self: *const Hair) StridedPtrConst(Float3) {
        const velocities = self.velocities orelse return .{};
        return .init(@ptrCast(&velocities.ptr[0].velocity), .{ .stride = @sizeOf(HairVelocity) });
    }

    /// Zolt: a null StridedPtr before ReadBackGPUState (Jolt offsets a null pointer)
    pub fn getAngularVelocities(self: *const Hair) StridedPtrConst(Float3) {
        const velocities = self.velocities orelse return .{};
        return .init(@ptrCast(&velocities.ptr[0].angular_velocity), .{ .stride = @sizeOf(HairVelocity) });
    }

    pub fn getGridVelocityAndDensity(self: *const Hair) ?[]const Float4 {
        return self.velocity_and_density;
    }

    pub fn getRenderPositions(self: *const Hair) ?[]const Float3 {
        return self.render_positions;
    }

    /// Calculate the UpdateContext parameters (protected in Jolt)
    /// Zolt: `error.OutOfMemory` when the colliding shapes cannot be stored. The context then holds the shapes that were
    /// stored; the caller releases them with `ctx.deinit`.
    pub fn initializeContext(self: *Hair, ctx: *UpdateContext, delta_time: f32, system: *const PhysicsSystem) Allocator.Error!void {
        const settings = self.getHairSettings();

        const clamped_delta_time = math.min(delta_time, settings.max_delta_time);
        ctx.num_iterations = truncToUint(@round(clamped_delta_time * @as(f32, @floatFromInt(settings.num_iterations_per_second))));
        ctx.delta_time = if (ctx.num_iterations > 0) clamped_delta_time / @as(f32, @floatFromInt(ctx.num_iterations)) else 0.0;
        ctx.time_ratio = ctx.delta_time * @as(f32, @floatFromInt(HairSettings.default_iterations_per_second));
        ctx.half_delta_time = 0.5 * ctx.delta_time;
        ctx.inv_delta_time_sq = if (ctx.delta_time > 0.0) 1.0 / math.square(ctx.delta_time) else 1.0e12;
        ctx.two_div_delta_time = if (ctx.delta_time > 0.0) 2.0 / ctx.delta_time else 1.0e12;
        ctx.sub_step_gravity = self.rotation.conjugated().mulVec3(system.getGravity()).mulScalar(ctx.delta_time);

        // Calculate delta transform from previous to current position and rotation
        ctx.has_transform_changed = !self.position.eql(self.prev_position) or !self.rotation.eql(self.prev_rotation);
        const prev_com = RMat44.rotationTranslation(self.prev_rotation, self.prev_position);
        ctx.delta_transform = self.getWorldTransform().inversedRotationTranslation().mul(prev_com).toMat44();
        ctx.delta_transform_quat = ctx.delta_transform.getQuaternion();
        self.prev_position = self.position;
        self.prev_rotation = self.rotation;

        // Check if we need collision detection / grid
        ctx.needs_collision = false;
        ctx.needs_grid = false;
        ctx.global_pose_only = true;
        for (settings.materials.items) |*material| {
            ctx.needs_collision = ctx.needs_collision or material.enable_collision;
            ctx.needs_grid = ctx.needs_grid or material.needsGrid();
            ctx.global_pose_only = ctx.global_pose_only and material.globalPoseOnly();
        }

        if (ctx.needs_collision) {
            const Collector = struct {
                pub const overrides = .{.addHit};

                base: CollideShapeBodyCollector = .init(@This()),
                system: *const PhysicsSystem,
                transform: RMat44,
                inverse_transform: RMat44,
                local_bounds: AABox,
                hits: *std.ArrayList(LeafShape),
                /// Zolt: grows `hits`
                allocator: Allocator,
                /// Zolt: an allocation failed, the query stops
                out_of_memory: bool = false,

                pub fn addHit(c: *@This(), result: *const BodyID) void {
                    var lock: BodyLockRead = .init(c.system.getBodyLockInterface(), result.*);
                    defer lock.deinit();
                    if (lock.succeeded()) {
                        const body = lock.getBody();
                        if (body.isRigidBody() and !body.isSensor()) {
                            // Calculate transform of this body relative to the hair instance
                            const com = c.inverse_transform.mul(body.getCenterOfMassTransform()).toMat44();

                            // Collect leaf shapes
                            const LeafShapeCollector = struct {
                                pub const overrides = .{.addHit};

                                base: TransformedShapeCollector = .init(@This()),
                                head_transform: RMat44,
                                body: *const Body,
                                hits: *std.ArrayList(LeafShape),
                                /// Zolt: grows `hits`
                                allocator: Allocator,
                                /// Zolt: an allocation failed, the collection stops
                                out_of_memory: bool = false,

                                pub fn addHit(lc: *@This(), leaf: *const TransformedShape) void {
                                    lc.hits.ensureUnusedCapacity(lc.allocator, 1) catch {
                                        lc.out_of_memory = true;
                                        lc.base.forceEarlyOut();
                                        return;
                                    };
                                    lc.hits.appendAssumeCapacity(.init(
                                        Mat44.rotationTranslation(leaf.shape_rotation, leaf.shape_position_com.toVec3()),
                                        leaf.getShapeScale(),
                                        lc.head_transform.multiply3x3Transposed(lc.body.getPointVelocity(lc.head_transform.mulRVec3(leaf.shape_position_com))), // Calculate velocity of shape at its center of mass position
                                        lc.head_transform.multiply3x3Transposed(lc.body.getAngularVelocity()),
                                        leaf.shape.get().?,
                                    ));
                                }
                            };
                            var collector: LeafShapeCollector = .{ .head_transform = c.transform, .body = body, .hits = c.hits, .allocator = c.allocator };
                            const default_filter: ShapeFilter = .{};
                            body.getShape().collectTransformedShapes(c.local_bounds, com.getTranslation(), com.getQuaternion(), Vec3.one(), .{}, &collector.base, &default_filter);
                            if (collector.out_of_memory) {
                                c.out_of_memory = true;
                                c.base.forceEarlyOut();
                            }
                        }
                    }
                }
            };

            // Calculate world space bounding box
            const transform = self.getWorldTransform();
            const world_bounds = settings.simulation_bounds.transformedRMat44(transform);

            // Collect shapes that intersect with the bounding box
            var collector: Collector = .{
                .system = system,
                .transform = transform,
                .inverse_transform = transform.inversedRotationTranslation(),
                .local_bounds = settings.simulation_bounds,
                .hits = &ctx.shapes,
                .allocator = self.allocator,
            };
            const broadphase_layer_filter = system.getDefaultBroadPhaseLayerFilter(self.layer);
            const object_layer_filter = system.getDefaultLayerFilter(self.layer);
            system.getBroadPhaseQuery().collideAABox(&world_bounds, &collector.base, .{ .broad_phase_layer_filter = &broadphase_layer_filter.base, .object_layer_filter = &object_layer_filter.base });
            if (collector.out_of_memory)
                return error.OutOfMemory;

            // If no shapes were found, we don't need collision
            if (ctx.shapes.items.len == 0)
                ctx.needs_collision = false;
        }
    }

    /// Step the hair simulation forward in time
    /// @param delta_time Time step
    /// @param joint_to_hair Transform that transforms from joint space to hair local space (as defined by GetWorldTransform)
    /// @param joint_matrices Array of joint matrices in world space, length needs to match HairSettings::mScalpInverseBindPose.size() (null for none)
    /// @param system Physics system used for collision detection
    /// @param shaders Preloaded hair compute shaders
    /// @param compute_system Compute system to use
    /// @param queue Compute queue to use
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The step is then partly done; call
    /// `onTeleported` to reset the hair at the next update (see the file header).
    pub fn update(self: *Hair, delta_time: f32, joint_to_hair: Mat44, joint_matrices: ?[]const Mat44, system: *const PhysicsSystem, shaders: *const HairShaders, compute_system: *ComputeSystem, queue: *ComputeQueue) Allocator.Error!void {
        const allocator = self.allocator;
        const settings = self.getHairSettings();

        var ctx: UpdateContext = .{};
        defer ctx.deinit(allocator);
        try self.initializeContext(&ctx, delta_time, system);

        if (joint_matrices != null and self.scalp_joint_matrices_cb.get() != null) {
            // Prepare for Skinning
            const buffer = self.scalp_joint_matrices_cb.get().?;
            const joints = buffer.mapAs(Mat44, .write);
            settings.prepareForScalpSkinning(joint_to_hair, joint_matrices.?, joints);
            buffer.unmap();
        }

        if (ctx.needs_collision) {
            // Create Collision Shapes

            // First determine buffer sizes
            var num_shapes: u32 = 0;
            var num_faces: u32 = 0;
            var num_vertices: u32 = 0;
            var num_header: u32 = 0;
            var num_indices: u32 = 0;
            var max_vertices_per_face: u32 = 0;
            var max_points: u32 = 0;
            for (ctx.shapes.items) |*shape| {
                if (shape.shape.get().?.getSubType() == .convex_hull) {
                    const ch = shape.shape.get().?.cast(ConvexHullShape);
                    num_shapes += 1;
                    num_header += 1; // Write number of vertices
                    const np = ch.getNumPoints();
                    max_points = @max(max_points, np);
                    num_vertices += np;
                    const nf = ch.getNumFaces();
                    num_faces += nf;
                    var f: u32 = 0;
                    while (f < nf) : (f += 1) {
                        num_header += 2; // Write indices start + end
                        const num_vertices_in_face = ch.getNumVerticesInFace(f);
                        num_indices += num_vertices_in_face;
                        max_vertices_per_face = @max(max_vertices_per_face, num_vertices_in_face);
                    }
                }
            }
            num_header += 1; // Terminator
            num_indices += num_header;

            // Now allocate buffers
            if (self.collision_shapes_cb.get() == null or self.collision_shapes_cb.get().?.getSize() < num_shapes) {
                self.collision_shapes_cb.deinit();
                // Zolt (R20 UB3): max(num_shapes, 1) zeroed elements, Jolt creates num_shapes elements and HairUpdateVelocity reads element 0 when there are none
                self.collision_shapes_cb = try compute_system.createComputeBuffer(.upload_buffer, @max(num_shapes, 1), @sizeOf(HairCollisionShape), null);
            }
            if (self.shape_planes_cb.get() == null or self.shape_planes_cb.get().?.getSize() < num_faces) {
                self.shape_planes_cb.deinit();
                self.shape_planes_cb = try compute_system.createComputeBuffer(.upload_buffer, @max(num_faces, 1), @sizeOf(Float4), null);
            }
            if (self.shape_vertices_cb.get() == null or self.shape_vertices_cb.get().?.getSize() < num_vertices) {
                self.shape_vertices_cb.deinit();
                self.shape_vertices_cb = try compute_system.createComputeBuffer(.upload_buffer, @max(num_vertices, 1), @sizeOf(Float3), null);
            }
            if (self.shape_indices_cb.get() == null or self.shape_indices_cb.get().?.getSize() < num_indices) {
                self.shape_indices_cb.deinit();
                self.shape_indices_cb = try compute_system.createComputeBuffer(.upload_buffer, num_indices, @sizeOf(u32), null);
            }

            // Zolt: Jolt's JPH_STACK_ALLOC arrays use the allocator of the hair. They are allocated before the buffers
            // are mapped, so that a failure leaves nothing mapped.
            const face_indices = try allocator.alloc(u32, max_vertices_per_face);
            defer allocator.free(face_indices);
            const points = try allocator.alloc(Vec3, max_points);
            defer allocator.free(points);
            const collision_shapes = self.collision_shapes_cb.get().?.mapAs(HairCollisionShape, .write);
            const shape_planes = self.shape_planes_cb.get().?.mapAs(Float4, .write);
            const shape_vertices = self.shape_vertices_cb.get().?.mapAs(Float3, .write);
            const shape_indices = self.shape_indices_cb.get().?.mapAs(u32, .write);

            // Convert the hulls to compute buffers
            var sp: u32 = 0; // Index in shape_planes
            var sv: u32 = 0; // Index in shape_vertices
            var sh: u32 = 0; // Index in shape_indices (the header)
            var cs: u32 = 0; // Index in collision_shapes
            var si: u32 = num_header; // Index in shape_indices (the indices)
            for (ctx.shapes.items) |*shape| {
                if (shape.shape.get().?.getSubType() == .convex_hull) {
                    const ch = shape.shape.get().?.cast(ConvexHullShape);

                    // Store collision shape
                    collision_shapes[cs] = .{
                        .center_of_mass = toFloat3(shape.transform.getTranslation()),
                        .linear_velocity = toFloat3(shape.linear_velocity),
                        .angular_velocity = toFloat3(shape.angular_velocity),
                    };
                    cs += 1;

                    // Store points transformed to hair space
                    const shape_transform = shape.transform.preScaled(shape.scale);
                    const first_vertex_index: u32 = sv;
                    var p: u32 = 0;
                    const np = ch.getNumPoints();
                    while (p < np) : (p += 1) {
                        const v = shape_transform.mulVec3(ch.getPoint(p));
                        points[p] = v; // Store points in a temporary buffer so we avoid reading from GPU memory
                        v.storeFloat3(&shape_vertices[sv]);
                        sv += 1;
                    }

                    // Store number of faces
                    const nf = ch.getNumFaces();
                    shape_indices[sh] = nf;
                    sh += 1;

                    // Store the indices
                    if (ScaleHelpers.isInsideOut(shape.scale)) {
                        // Reverse winding order
                        var f: u32 = 0;
                        while (f < nf) : (f += 1) {
                            // Store indices
                            const nv = ch.getFaceVertices(f, face_indices);
                            const indices_start: u32 = si;
                            shape_indices[sh] = indices_start;
                            sh += 1;
                            shape_indices[sh] = indices_start + nv;
                            sh += 1;
                            var v: i32 = @as(i32, @intCast(nv)) - 1;
                            while (v >= 0) : ({
                                v -= 1;
                                si += 1;
                            }) {
                                shape_indices[si] = face_indices[@intCast(v)] + first_vertex_index;
                            }

                            // Calculate plane (avoids reading from GPU memory)
                            Plane.fromPointsCCW(points[face_indices[2]], points[face_indices[1]], points[face_indices[0]]).storeFloat4(&shape_planes[sp]);
                            sp += 1;
                        }
                    } else {
                        // Keep winding order
                        var f: u32 = 0;
                        while (f < nf) : (f += 1) {
                            // Store indices
                            const nv = ch.getFaceVertices(f, face_indices);
                            const indices_start: u32 = si;
                            shape_indices[sh] = indices_start;
                            sh += 1;
                            shape_indices[sh] = indices_start + nv;
                            sh += 1;
                            var v: u32 = 0;
                            while (v < nv) : (v += 1) {
                                shape_indices[si] = face_indices[v] + first_vertex_index;
                                si += 1;
                            }

                            // Calculate plane (avoids reading from GPU memory)
                            Plane.fromPointsCCW(points[face_indices[0]], points[face_indices[1]], points[face_indices[2]]).storeFloat4(&shape_planes[sp]);
                            sp += 1;
                        }
                    }
                }
            }
            shape_indices[sh] = 0; // Terminator
            sh += 1;
            if (Core.enable_asserts) {
                std.debug.assert(cs == num_shapes);
                std.debug.assert(sp == num_faces);
                std.debug.assert(sv == num_vertices);
                std.debug.assert(sh == num_header);
                std.debug.assert(si == num_indices);
            }

            // Unmap buffers
            self.collision_shapes_cb.get().?.unmap();
            self.shape_planes_cb.get().?.unmap();
            self.shape_vertices_cb.get().?.unmap();
            self.shape_indices_cb.get().?.unmap();
        }

        {
            // Set materials
            const materials = self.materials_cb.get().?.mapAs(HairMaterial, .write);
            for (settings.materials.items, 0..) |*m_in, i| {
                const m_out = &materials[i];

                const world_transform_influence: GradientSampler = .init(m_in.world_transform_influence);
                m_out.world_transform_influence = toFloat4(world_transform_influence.toFloat4());
                const global_pose: GradientSampler = .init(if (ctx.global_pose_only) m_in.global_pose else m_in.global_pose.makeStepDependent(ctx.time_ratio));
                m_out.global_pose = toFloat4(global_pose.toFloat4());
                const global_pose_skin_to_root: GradientSampler = .init(m_in.skin_global_pose);
                m_out.skin_global_pose = toFloat4(global_pose_skin_to_root.toFloat4());
                const gravity_factor: GradientSampler = .init(m_in.gravity_factor);
                m_out.gravity_factor = toFloat4(gravity_factor.toFloat4());
                const hair_radius: GradientSampler = .init(m_in.hair_radius);
                m_out.hair_radius = toFloat4(hair_radius.toFloat4());
                m_out.bend_compliance_multiplier = toFloat4(m_in.bend_compliance_multiplier);
                const grid_velocity_factor: GradientSampler = .init(m_in.grid_velocity_factor.makeStepDependent(ctx.time_ratio));
                m_out.grid_velocity_factor = toFloat4(grid_velocity_factor.toFloat4());
                m_out.enable_collision = if (ctx.needs_collision and m_in.enable_collision) 1 else 0;
                m_out.enable_lra = if (m_in.enable_lra) 1 else 0;
                m_out.enable_grid = @intFromBool(m_in.grid_velocity_factor.min != 0.0 or m_in.grid_velocity_factor.max != 0.0 or m_in.grid_density_force_factor != 0.0);
                m_out.friction = m_in.friction;
                m_out.exp_linear_damping_delta_time = LibM.expf(-m_in.linear_damping * ctx.delta_time);
                m_out.exp_angular_damping_delta_time = LibM.expf(-m_in.angular_damping * ctx.delta_time);
                m_out.bend_compliance_inv_delta_time_sq = m_in.bend_compliance * ctx.inv_delta_time_sq;
                m_out.stretch_compliance_inv_delta_time_sq = m_in.stretch_compliance * ctx.inv_delta_time_sq;
                m_out.grid_density_force_factor = m_in.grid_density_force_factor;
                m_out.inertia_multiplier = m_in.inertia_multiplier;
                m_out.max_linear_velocity_sq = math.square(m_in.max_linear_velocity);
                m_out.max_angular_velocity_sq = math.square(m_in.max_angular_velocity);
            }
            self.materials_cb.get().?.unmap();
        }

        {
            // Set constants
            const cdata = &self.constants_cb.get().?.mapAs(HairUpdateContext, .write)[0];
            cdata.num_strands = @truncate(settings.sim_strands.items.len);
            cdata.num_vertices = settings.getNumVerticesPadded();
            cdata.num_grid_points = @truncate(settings.neutral_density.items.len);
            cdata.num_render_vertices = @truncate(settings.render_vertices.items.len);
            const grid_sampler: HairSettings.GridSampler = .init(settings);
            cdata.grid_size_min2 = .init(grid_sampler.grid_size_min2.getX(), grid_sampler.grid_size_min2.getY(), grid_sampler.grid_size_min2.getZ()); // memcpy of 3 components
            cdata.two_div_delta_time = ctx.two_div_delta_time;
            cdata.grid_size_min1 = toFloat3(grid_sampler.grid_size_min1);
            cdata.delta_time = ctx.delta_time;
            cdata.grid_offset = toFloat3(grid_sampler.offset);
            cdata.half_delta_time = ctx.half_delta_time;
            cdata.grid_scale = toFloat3(grid_sampler.scale);
            cdata.inv_delta_time_sq = ctx.inv_delta_time_sq;
            cdata.sub_step_gravity = toFloat3(ctx.sub_step_gravity);
            cdata.num_skin_vertices = @truncate(settings.scalp_vertices.items.len);
            cdata.grid_stride = .init(grid_sampler.grid_stride.getX(), grid_sampler.grid_stride.getY(), grid_sampler.grid_stride.getZ()); // memcpy of 3 components
            cdata.num_skin_weights_per_vertex = settings.scalp_num_skin_weights_per_vertex;
            for (0..4) |i|
                cdata.delta_transform.c[i] = vec4ToFloat4(ctx.delta_transform.getColumn4(@intCast(i)));
            for (0..4) |i|
                cdata.scalp_to_head.c[i] = vec4ToFloat4(self.scalp_to_head.getColumn4(@intCast(i)));
            cdata.delta_transform_quat = .init(ctx.delta_transform_quat.getX(), ctx.delta_transform_quat.getY(), ctx.delta_transform_quat.getZ(), ctx.delta_transform_quat.getW());
            self.constants_cb.get().?.unmap();
        }

        {
            // Set iteration constants

            // Ensure that we have the right number of constant buffers allocated
            // Zolt: the list grows one complete buffer at a time (Jolt resizes first), so that an allocation failure
            // leaves no empty entry
            const old_size: u32 = @intCast(self.iteration_constants_cb.items.len);
            if (old_size < ctx.num_iterations) {
                try self.iteration_constants_cb.ensureTotalCapacity(allocator, ctx.num_iterations);
                var i: u32 = old_size;
                while (i < ctx.num_iterations) : (i += 1)
                    self.iteration_constants_cb.appendAssumeCapacity(try compute_system.createComputeBuffer(.constant_buffer, 1, @sizeOf(HairIterationContext), null));
            }

            // Fill in the constant buffers
            var iteration_data: HairIterationContext = undefined;
            var i: u32 = 0;
            while (i < ctx.num_iterations) : (i += 1) {
                iteration_data.accumulated_delta_time = ctx.delta_time * @as(f32, @floatFromInt(i + 1));
                iteration_data.iteration_fraction = 1.0 / @as(f32, @floatFromInt(ctx.num_iterations - i));

                const buffer = self.iteration_constants_cb.items[i].get().?;
                const idata = &buffer.mapAs(HairIterationContext, .write)[0];
                idata.* = iteration_data;
                buffer.unmap();
            }
        }

        {
            // Queue Compute

            const dispatch_per_vertex: u32 = (settings.getNumVerticesPadded() +% HairStructs.hair_per_vertex_batch -% 1) / HairStructs.hair_per_vertex_batch;
            const dispatch_per_vertex_skip_first_vertex: u32 = (settings.getNumVerticesPadded() -% @as(u32, @truncate(settings.sim_strands.items.len)) +% HairStructs.hair_per_vertex_batch -% 1) / HairStructs.hair_per_vertex_batch; // Skip the first vertex of each strand
            const dispatch_per_grid_cell: u32 = @truncate((settings.neutral_density.items.len + HairStructs.hair_per_grid_cell_batch - 1) / HairStructs.hair_per_grid_cell_batch);
            const dispatch_per_strand: u32 = @truncate((settings.sim_strands.items.len + HairStructs.hair_per_strand_batch - 1) / HairStructs.hair_per_strand_batch);
            const dispatch_per_render_vertex: u32 = @truncate((settings.render_vertices.items.len + HairStructs.hair_per_render_vertex_batch - 1) / HairStructs.hair_per_render_vertex_batch);

            const was_teleported = self.teleported;
            self.teleported = false;
            if (was_teleported) {
                // Initialize positions and velocities
                try queue.setShader(getShader(shaders.teleport_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                try queue.setBuffer("gInitialBishops", settings.vertices_bishop_cb.get());
                try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                try queue.setRWBuffer("gVelocities", self.velocities_cb.get(), .{});
                queue.dispatch(dispatch_per_vertex, .{});
            } else if (!ctx.global_pose_only and ctx.has_transform_changed) {
                // Apply delta transform
                try queue.setShader(getShader(shaders.apply_delta_transform_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                try queue.setBuffer("gMaterials", self.materials_cb.get());
                try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                try queue.setRWBuffer("gVelocities", self.velocities_cb.get(), .{});
                queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});
            }

            if (self.scalp_joint_matrices_cb.get() != null) {
                // Skin the scalp mesh
                try queue.setShader(getShader(shaders.skin_vertices_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gScalpVertices", settings.scalp_vertices_cb.get());
                try queue.setBuffer("gScalpSkinWeights", settings.scalp_skin_weights_cb.get());
                try queue.setBuffer("gScalpJointMatrices", self.scalp_joint_matrices_cb.get());
                try queue.setRWBuffer("gScalpVerticesOut", self.scalp_vertices_cb.get(), .{});
                queue.dispatch(@truncate((settings.scalp_vertices.items.len + HairStructs.hair_per_vertex_batch - 1) / HairStructs.hair_per_vertex_batch), .{});
            }

            if (self.scalp_vertices_cb.get() != null) {
                // Determine if we directly write to the position / transform buffers or if we need to interpolate
                const needs_interpolate = !ctx.global_pose_only and !was_teleported;

                // Create target buffers if they don't exist yet
                // Zolt: both are set together, so that an allocation failure leaves neither
                if (self.target_positions_cb.get() == null and needs_interpolate) {
                    var target_positions = try compute_system.createComputeBuffer(.rw_buffer, settings.sim_strands.items.len, @sizeOf(HairPosition), null);
                    errdefer target_positions.deinit();
                    const target_global_pose_transforms = try compute_system.createComputeBuffer(.rw_buffer, settings.sim_strands.items.len, @sizeOf(HairGlobalPoseTransform), null);
                    self.target_positions_cb = target_positions;
                    self.target_global_pose_transforms_cb = target_global_pose_transforms;
                }

                // Skin the strand roots to the scalp mesh
                try queue.setShader(getShader(shaders.skin_roots_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gSkinPoints", settings.skin_points_cb.get());
                try queue.setBuffer("gScalpVertices", self.scalp_vertices_cb.get());
                try queue.setBuffer("gScalpTriangles", self.scalp_triangles_cb.get());
                try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                try queue.setBuffer("gInitialBishops", settings.vertices_bishop_cb.get());
                try queue.setRWBuffer("gPositions", if (needs_interpolate) self.target_positions_cb.get() else self.positions_cb.get(), .{});
                try queue.setRWBuffer("gGlobalPoseTransforms", if (needs_interpolate) self.target_global_pose_transforms_cb.get() else self.global_pose_transforms_cb.get(), .{});
                queue.dispatch(dispatch_per_strand, .{});
            }

            if (ctx.global_pose_only) {
                // Only run global pose logic
                try queue.setShader(getShader(shaders.apply_global_pose_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                try queue.setBuffer("gInitialBishops", settings.vertices_bishop_cb.get());
                try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                try queue.setBuffer("gMaterials", self.materials_cb.get());
                try queue.setBuffer("gGlobalPoseTransforms", self.global_pose_transforms_cb.get());
                try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});
            } else if (ctx.num_iterations > 0) {
                if (ctx.needs_collision) {
                    // Calculate collision planes
                    try queue.setShader(getShader(shaders.calculate_collision_planes_cs));
                    try queue.setConstantBuffer("gContext", self.constants_cb.get());
                    try queue.setBuffer("gPositions", self.positions_cb.get());
                    try queue.setBuffer("gShapePlanes", self.shape_planes_cb.get());
                    try queue.setBuffer("gShapeVertices", self.shape_vertices_cb.get());
                    try queue.setBuffer("gShapeIndices", self.shape_indices_cb.get());
                    try queue.setRWBuffer("gCollisionPlanes", self.collision_planes_cb.get(), .{});
                    queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});
                }

                if (ctx.needs_grid) {
                    // Clear the grid
                    try queue.setShader(getShader(shaders.grid_clear_cs));
                    try queue.setConstantBuffer("gContext", self.constants_cb.get());
                    try queue.setRWBuffer("gVelocityAndDensity", self.velocity_and_density_cb.get(), .{});
                    queue.dispatch(dispatch_per_grid_cell, .{});

                    // Accumulate vertices into the grid
                    try queue.setShader(getShader(shaders.grid_accumulate_cs));
                    try queue.setConstantBuffer("gContext", self.constants_cb.get());
                    try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                    try queue.setBuffer("gPositions", self.positions_cb.get());
                    try queue.setBuffer("gVelocities", self.velocities_cb.get());
                    try queue.setRWBuffer("gVelocityAndDensity", self.velocity_and_density_cb.get(), .{});
                    queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});

                    // Normalize velocities in the grid
                    try queue.setShader(getShader(shaders.grid_normalize_cs));
                    try queue.setConstantBuffer("gContext", self.constants_cb.get());
                    try queue.setRWBuffer("gVelocityAndDensity", self.velocity_and_density_cb.get(), .{});
                    queue.dispatch(dispatch_per_grid_cell, .{});
                }

                // First integrate
                try queue.setShader(getShader(shaders.integrate_cs));
                try queue.setConstantBuffer("gContext", self.constants_cb.get());
                try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                try queue.setBuffer("gNeutralDensity", settings.neutral_density_cb.get());
                try queue.setBuffer("gVelocityAndDensity", self.velocity_and_density_cb.get());
                try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                try queue.setBuffer("gMaterials", self.materials_cb.get());
                try queue.setBuffer("gVelocities", self.velocities_cb.get());
                try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                try queue.setRWBuffer("gPreviousPositions", self.previous_positions_cb.get(), .{});
                queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});

                var it: u32 = 0;
                while (it < ctx.num_iterations) : (it += 1) {
                    if (self.target_positions_cb.get() != null and !was_teleported) {
                        // Update skinned roots for this iteration (interpolate them towards the target positions)
                        try queue.setShader(getShader(shaders.update_roots_cs));
                        try queue.setConstantBuffer("gContext", self.constants_cb.get());
                        try queue.setConstantBuffer("gIterationContext", self.iteration_constants_cb.items[it].get());
                        try queue.setBuffer("gTargetPositions", self.target_positions_cb.get());
                        try queue.setBuffer("gTargetGlobalPoseTransforms", self.target_global_pose_transforms_cb.get());
                        try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                        try queue.setRWBuffer("gGlobalPoseTransforms", self.global_pose_transforms_cb.get(), .{});
                        queue.dispatch(dispatch_per_strand, .{});
                    }

                    // Then update the constraints per strand
                    try queue.setShader(getShader(shaders.update_strands_cs));
                    try queue.setConstantBuffer("gContext", self.constants_cb.get());
                    try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                    try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                    try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                    try queue.setBuffer("gOmega0s", settings.vertices_omega0_cb.get());
                    try queue.setBuffer("gInitialLengths", settings.vertices_length_cb.get());
                    try queue.setBuffer("gStrandVertexCounts", settings.strand_vertex_counts_cb.get());
                    try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                    try queue.setBuffer("gMaterials", self.materials_cb.get());
                    try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                    queue.dispatch(dispatch_per_strand, .{});

                    if (it == ctx.num_iterations - 1) {
                        // Last iteration: only update velocities
                        try queue.setShader(getShader(shaders.update_velocity_cs));
                        try queue.setConstantBuffer("gContext", self.constants_cb.get());
                        try queue.setConstantBuffer("gIterationContext", self.iteration_constants_cb.items[it].get());
                        try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                        try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                        try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                        try queue.setBuffer("gInitialBishops", settings.vertices_bishop_cb.get());
                        try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                        try queue.setBuffer("gMaterials", self.materials_cb.get());
                        try queue.setBuffer("gPreviousPositions", self.previous_positions_cb.get());
                        try queue.setBuffer("gGlobalPoseTransforms", self.global_pose_transforms_cb.get());
                        try queue.setBuffer("gCollisionShapes", self.collision_shapes_cb.get());
                        try queue.setBuffer("gCollisionPlanes", self.collision_planes_cb.get());
                        try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                        try queue.setRWBuffer("gVelocities", self.velocities_cb.get(), .{});
                        queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});
                    } else {
                        // Other iterations: update velocities then integrate again
                        try queue.setShader(getShader(shaders.update_velocity_integrate_cs));
                        try queue.setConstantBuffer("gContext", self.constants_cb.get());
                        try queue.setConstantBuffer("gIterationContext", self.iteration_constants_cb.items[it].get());
                        try queue.setBuffer("gVerticesFixed", settings.vertices_fixed_cb.get());
                        try queue.setBuffer("gStrandFractions", settings.vertices_strand_fraction_cb.get());
                        try queue.setBuffer("gInitialPositions", settings.vertices_position_cb.get());
                        try queue.setBuffer("gInitialBishops", settings.vertices_bishop_cb.get());
                        try queue.setBuffer("gNeutralDensity", settings.neutral_density_cb.get());
                        try queue.setBuffer("gVelocityAndDensity", self.velocity_and_density_cb.get());
                        try queue.setBuffer("gStrandMaterialIndex", settings.strand_material_index_cb.get());
                        try queue.setBuffer("gMaterials", self.materials_cb.get());
                        try queue.setBuffer("gGlobalPoseTransforms", self.global_pose_transforms_cb.get());
                        try queue.setBuffer("gCollisionShapes", self.collision_shapes_cb.get());
                        try queue.setBuffer("gCollisionPlanes", self.collision_planes_cb.get());
                        try queue.setRWBuffer("gPreviousPositions", self.previous_positions_cb.get(), .{});
                        try queue.setRWBuffer("gPositions", self.positions_cb.get(), .{});
                        queue.dispatch(dispatch_per_vertex_skip_first_vertex, .{});
                    }
                }
            }

            // Remap simulation positions to render positions
            try queue.setShader(getShader(shaders.calculate_render_positions_cs));
            try queue.setConstantBuffer("gContext", self.constants_cb.get());
            try queue.setBuffer("gSVertexInfluences", settings.s_vertex_influences_cb.get());
            try queue.setBuffer("gPositions", self.positions_cb.get());
            try queue.setRWBuffer("gRenderPositions", self.render_positions_cb.get(), .{});
            queue.dispatch(dispatch_per_render_vertex, .{});
        }
    }

    /// Read back the GPU state so that the functions below can be used. For debugging purposes only, this is slow!
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The read back buffers are set together.
    pub fn readBackGPUState(self: *Hair, queue: *ComputeQueue) Allocator.Error!void {
        const allocator = self.allocator;
        const settings = self.getHairSettings();

        if (self.positions_read_back_cb.get() == null) {
            // Create read back buffers
            var scalp_vertices: Ref(ComputeBuffer) = .empty;
            errdefer scalp_vertices.deinit();
            if (self.scalp_vertices_cb.get()) |b|
                scalp_vertices = try b.createReadBackBuffer();
            var positions = try self.positions_cb.get().?.createReadBackBuffer();
            errdefer positions.deinit();
            var velocities = try self.velocities_cb.get().?.createReadBackBuffer();
            errdefer velocities.deinit();
            var velocity_and_density = try self.velocity_and_density_cb.get().?.createReadBackBuffer();
            errdefer velocity_and_density.deinit();
            const render_positions = try self.render_positions_cb.get().?.createReadBackBuffer();
            self.scalp_vertices_read_back_cb = scalp_vertices;
            self.positions_read_back_cb = positions;
            self.velocities_read_back_cb = velocities;
            self.velocity_and_density_read_back_cb = velocity_and_density;
            self.render_positions_read_back_cb = render_positions;
        }

        {
            // Transfer data from GPU

            // Read back the skinned vertices
            // Zolt: the read back buffer is null when the scalp vertices buffer was set after the first read back. Jolt
            // passes the null buffer, which its CPU backend ignores.
            if (self.scalp_vertices_cb.get()) |b| {
                if (self.scalp_vertices_read_back_cb.get()) |read_back|
                    queue.scheduleReadback(read_back, b);
            }

            // Read back the vertices
            queue.scheduleReadback(self.positions_read_back_cb.get().?, self.positions_cb.get().?);
            queue.scheduleReadback(self.velocities_read_back_cb.get().?, self.velocities_cb.get().?);
            queue.scheduleReadback(self.render_positions_read_back_cb.get().?, self.render_positions_cb.get().?);

            // Read back the velocity and density
            queue.scheduleReadback(self.velocity_and_density_read_back_cb.get().?, self.velocity_and_density_cb.get().?);

            // Wait for the compute queue to finish
            queue.executeAndWait();
        }

        {
            // Reorder hair data

            // Zolt: the arrays are allocated before the buffers are mapped, so that a failure leaves nothing mapped
            const num_vertices = settings.sim_vertices.items.len;
            if (self.positions == null)
                self.positions = try allocator.alloc(Float3, num_vertices);
            if (self.rotations == null)
                self.rotations = try allocator.alloc(Quat, num_vertices);
            if (self.velocities == null)
                self.velocities = try allocator.alloc(HairVelocity, num_vertices);

            // Reorder position and velocity data
            const positions = self.positions_read_back_cb.get().?.mapAs(HairPosition, .read);
            const velocities = self.velocities_read_back_cb.get().?.mapAs(HairVelocity, .read);
            const out_positions = self.positions.?;
            const out_rotations = self.rotations.?;
            const out_velocities = self.velocities.?;
            const num_strands: u32 = @truncate(settings.sim_strands.items.len);
            var s: u32 = 0;
            while (s < num_strands) : (s += 1) {
                const strand = settings.sim_strands.items[s];
                var v: u32 = 0;
                while (v < strand.vertexCount()) : (v += 1) {
                    const in_index = s + v * num_strands;
                    const out_index = strand.start_vtx + v;
                    const p = positions[in_index].position;
                    out_positions[out_index] = .{ .x = p.x, .y = p.y, .z = p.z };
                    const r = positions[in_index].rotation;
                    out_rotations[out_index] = Quat.init(r.x, r.y, r.z, r.w);
                    out_velocities[out_index] = velocities[in_index];
                }
            }
            self.positions_read_back_cb.get().?.unmap();
            self.velocities_read_back_cb.get().?.unmap();
        }
    }

    /// Lock/unlock the data buffers so that the functions below return valid values.
    /// Zolt: an allocation failure (the render positions of an override) returns `error.OutOfMemory` before a buffer is
    /// mapped (Jolt aborts).
    pub fn lockReadBackBuffers(self: *Hair) Allocator.Error!void {
        const num_render_vertices = self.getHairSettings().render_vertices.items.len;
        if (self.render_positions_overridden and self.render_positions_owned == null)
            self.render_positions_owned = try self.allocator.alloc(Float3, num_render_vertices);

        if (self.scalp_vertices_read_back_cb.get()) |b|
            self.scalp_vertices = b.mapAs(Float3, .read);
        self.velocity_and_density = self.velocity_and_density_read_back_cb.get().?.mapAs(Float4, .read);
        if (self.render_positions_overridden) {
            const render_positions = self.render_positions_owned.?;
            self.render_positions = render_positions;
            self.render_positions_to_float3.?.call(self.render_positions_read_back_cb.get().?, render_positions);
        } else self.render_positions = self.render_positions_read_back_cb.get().?.mapAs(Float3, .read);
    }

    pub fn unlockReadBackBuffers(self: *Hair) void {
        if (self.scalp_vertices_read_back_cb.get()) |b|
            b.unmap();
        self.velocity_and_density_read_back_cb.get().?.unmap();
        if (!self.render_positions_overridden)
            self.render_positions_read_back_cb.get().?.unmap();
    }

    // See Hair::Draw (JPH_DEBUG_RENDERER). TODO(debug_renderer): ERenderStrandColor, DrawSettings and Draw are ported by
    // group H3, this placeholder panics until then.
    pub const draw = if (Core.debug_renderer) drawImpl else {};
    fn drawImpl(self: *Hair, renderer: *DebugRenderer) void {
        _ = .{ self, renderer };
        @panic("TODO(debug_renderer): Hair.draw is ported by group H3");
    }
};

/// The shader of a HairShaders member (Jolt binds it without a check)
fn getShader(shader: Ref(ComputeShader)) *const ComputeShader {
    // Zolt: Jolt binds a null shader and crashes
    return shader.get() orelse @panic("Hair.update: a hair shader is missing (register the shaders with registerHairShaders before HairShaders.init)");
}

/// Vec3::StoreFloat3 into a JPH_float3
fn toFloat3(v: Vec3) hlsl.float3 {
    var f: Float3 = undefined;
    v.storeFloat3(&f);
    return .init(f.x, f.y, f.z);
}

/// A Float4 as a JPH_float4 (the same layout)
fn toFloat4(f: Float4) hlsl.float4 {
    return .init(f.x, f.y, f.z, f.w);
}

/// Vec4::StoreFloat4 into a JPH_float4
fn vec4ToFloat4(v: Vec4) hlsl.float4 {
    var f: Float4 = undefined;
    v.storeFloat4(&f);
    return toFloat4(f);
}

/// `(uint)f` of a float as Jolt's x86-64 build computes it (porting guide section 8, rule 12): a 64-bit cvttss2si of
/// which the low 32 bits are used. NaN and values outside the range of an int64 give 0.
fn truncToUint(value: f32) u32 {
    if (value >= -0x1p63 and value < 0x1p63) // False for NaN
        return @truncate(@as(u64, @bitCast(@as(i64, @intFromFloat(value)))));
    return 0;
}

// Tests ---------------------------------------------------------------------------------------------------------------

const testing = std.testing;
const virtual = @import("../../Core/Virtual.zig");
const ComputeSystemCPU = @import("../../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
const registerHairShaders = @import("../../Shaders/HairWrapper.zig").registerHairShaders;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const BodyCreationSettings = @import("../Body/BodyCreationSettings.zig").BodyCreationSettings;
const ShapeSettings = ShapeFile.ShapeSettings;
const ConvexHullShapeSettings = @import("../Collision/Shape/ConvexHullShape.zig").ConvexHullShapeSettings;
const BoxShapeSettings = @import("../Collision/Shape/BoxShape.zig").BoxShapeSettings;
const ScaledShapeSettings = @import("../Collision/Shape/ScaledShape.zig").ScaledShapeSettings;

/// A physics system with one object layer for the hair tests (in place: it must not move)
const TestWorld = struct {
    const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
    const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
    const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;

    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,

    fn init(self: *TestWorld, allocator: Allocator) !void {
        self.layer_interface = try .init(allocator, 1, 1);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(0, .init(0));
        self.pair_filter = try .init(allocator, 1);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(0, 0);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 1, &self.pair_filter.base, 1);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, testing.io, 16, 0, 64, 64, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
    }

    fn deinit(self: *TestWorld) void {
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// A body made from `settings` (reference count 0) added to the system
    fn addBody(self: *TestWorld, settings: *ShapeSettings, position: RVec3, opts: struct { motion_type: MotionType = .static, is_sensor: bool = false }) !BodyID {
        var body_settings: BodyCreationSettings = .initWithSettings(settings, position, Quat.identity(), opts.motion_type, 0);
        defer body_settings.deinit();
        body_settings.is_sensor = opts.is_sensor;
        const id = try self.system.getBodyInterfaceMut().createAndAddBody(&body_settings, .dont_activate);
        try testing.expect(!id.isInvalid());
        return id;
    }

    /// A body with a convex hull of a box (8 points), scaled when `scale` is set
    fn addHullBox(self: *TestWorld, half_extent: Vec3, position: RVec3, opts: struct { scale: ?Vec3 = null }) !BodyID {
        const allocator = self.system.allocator;
        var points: [8]Vec3 = undefined;
        for (&points, 0..) |*p, i|
            p.* = Vec3.init(if (i & 1 != 0) half_extent.getX() else -half_extent.getX(), if (i & 2 != 0) half_extent.getY() else -half_extent.getY(), if (i & 4 != 0) half_extent.getZ() else -half_extent.getZ());
        var settings: *ShapeSettings = (try ConvexHullShapeSettings.create(allocator, &points, .{ .max_convex_radius = 0.0 })).asShapeSettings();
        if (opts.scale) |scale|
            settings = (try ScaledShapeSettings.create(allocator, settings, scale)).asShapeSettings();
        return self.addBody(settings, position, .{});
    }
};

/// The compute system with the hair shaders, a queue and the loaded shaders
const TestCompute = struct {
    system: Ref(ComputeSystem),
    queue: Ref(ComputeQueue),
    shaders: Ref(HairShaders),

    fn init(allocator: Allocator) !TestCompute {
        var system = try ComputeSystemCPU.create(allocator);
        errdefer system.deinit();
        try registerHairShaders(virtual.downcast(ComputeSystemCPU, system.get().?));
        var queue = try system.get().?.createComputeQueue();
        errdefer queue.deinit();
        var shaders: Ref(HairShaders) = .init(try HairShaders.create(allocator));
        errdefer shaders.deinit();
        try shaders.get().?.init(system.get().?);
        return .{ .system = system, .queue = queue, .shaders = shaders };
    }

    fn deinit(self: *TestCompute) void {
        self.shaders.deinit();
        self.queue.deinit();
        self.system.deinit();
    }
};

/// The options of a test groom (testSettings)
const TestGroom = struct {
    /// Strand s has `num_vertices + s % vertex_spread` vertices (strands of different lengths have padding vertices)
    vertex_spread: u32 = 1,
    /// All vertices in the plane z = 0 (the bounds then have no z size without padding)
    flat: bool = false,
    /// The last vertex of each strand is one float step in x away from the vertex before it: a rod of nonzero length
    /// that is too short to change the float sum of the segment lengths
    tiny_last_segment: bool = false,
    padding: Vec3 = .replicate(0.1),
    /// A skinned scalp: a disc of 8 triangles around the roots in the plane y = 0, skinned to the two joints of
    /// sweepJoints
    scalp: bool = false,
};

/// A groom of `num_strands` strands of `num_vertices` vertices that hang down from a ring at y = 0, every strand
/// simulated, the first vertex of each strand fixed, ready for Hair (initRenderAndSimulationStrands, init and
/// initCompute ran). The caller owns the returned reference.
fn testSettings(allocator: Allocator, compute_system: *ComputeSystem, num_strands: u32, num_vertices: u32, material: HairSettings.Material, groom: TestGroom) !RefConst(HairSettings) {
    const settings = try HairSettings.create(allocator);
    var ref: RefConst(HairSettings) = .init(settings);
    errdefer ref.deinit();
    var m = material;
    m.simulation_strands_fraction = 1.0;
    try settings.materials.append(allocator, m);
    settings.grid_size = .init(8, 8, 8, 0);
    settings.simulation_bounds_padding = groom.padding;

    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    for (0..num_strands) |s| {
        const angle = 6.2831855 * @as(f32, @floatFromInt(s)) / @as(f32, @floatFromInt(num_strands));
        const first: u32 = @intCast(vertices.items.len);
        const count = num_vertices + @as(u32, @intCast(s)) % groom.vertex_spread;
        for (0..count) |i| {
            const fi: f32 = @floatFromInt(i);
            var position: Float3 = .{ .x = 0.05 * @cos(angle) + 0.002 * fi, .y = -0.02 * fi, .z = if (groom.flat) 0.0 else 0.05 * @sin(angle) + 0.001 * fi * fi };
            if (groom.tiny_last_segment and i == count - 1) {
                position = vertices.items[vertices.items.len - 1].position;
                position.x = std.math.nextAfter(f32, position.x, std.math.inf(f32));
            }
            try vertices.append(allocator, .init(position, .{ .inv_mass = if (i == 0) 0.0 else 1.0 }));
        }
        try strands.append(allocator, .init(first, first + count, 0));
    }
    if (groom.scalp) {
        // The center and 8 vertices on a ring of radius 0.1, half a sector away from the roots
        try settings.scalp_vertices.append(allocator, .{ .x = 0, .y = 0, .z = 0 });
        for (0..8) |k| {
            const angle = 6.2831855 * (@as(f32, @floatFromInt(k)) + 0.5) / 8.0;
            try settings.scalp_vertices.append(allocator, .{ .x = 0.1 * @cos(angle), .y = 0.0, .z = 0.1 * @sin(angle) });
        }
        for (0..8) |k|
            try settings.scalp_triangles.append(allocator, .init(0, @intCast(1 + k), @intCast(1 + (k + 1) % 8)));
        try settings.scalp_inverse_bind_pose.appendSlice(allocator, &.{ Mat44.identity(), Mat44.identity() });
        settings.scalp_num_skin_weights_per_vertex = 2;
        for (0..settings.scalp_vertices.items.len) |v| {
            const weight: f32 = if (v == 0) 1.0 else 0.25 * @as(f32, @floatFromInt(v % 4));
            try settings.scalp_skin_weights.appendSlice(allocator, &.{ .{ .joint_idx = 0, .weight = weight }, .{ .joint_idx = 1, .weight = 1.0 - weight } });
        }
    }
    try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    _ = try settings.init();
    try settings.initCompute(compute_system);
    return ref;
}

/// The bytes of a compute buffer
fn bufferBytes(allocator: Allocator, buffer: *ComputeBuffer) ![]u8 {
    const bytes = try allocator.dupe(u8, buffer.map(.read));
    buffer.unmap();
    return bytes;
}

test "Hair: deinit without initCompute, the reference to the settings" {
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    settings.setEmbedded();
    defer settings.deinit();
    const embedded = settings.getRefCount();
    {
        var hair: Hair = .init(allocator, &settings, RVec3.init(1, 2, 3), Quat.rotation(Vec3.axisY(), 0.5), 0);
        defer hair.deinit();
        try testing.expectEqual(embedded + 1, settings.getRefCount()); // The hair holds a reference
        try testing.expect(hair.getHairSettings() == &settings);
        try testing.expect(hair.teleported);
        try testing.expect(hair.getPositionsCB() == null and hair.getPositions() == null and hair.getRenderPositions() == null);
        try testing.expect(hair.getVelocities().getPtr() == null and hair.getAngularVelocities().getPtr() == null);
        try testing.expect(hair.getWorldTransform().getTranslation().eql(RVec3.init(1, 2, 3)));
    }
    try testing.expectEqual(embedded, settings.getRefCount());
}

test "Hair: InitializeContext, the delta time and the sub steps" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();

    var settings: HairSettings = .{ .allocator = allocator };
    settings.setEmbedded();
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .enable_collision = false });
    var hair: Hair = .init(allocator, &settings, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();

    // 360 iterations per second, at most 1 / 30 s: (delta time, iterations)
    const cases = [_]struct { f32, u32 }{
        .{ 0.0, 0 }, .{ 1.0 / 60.0, 6 }, .{ 1.0 / 90.0, 4 }, .{ 0.0123, 4 }, .{ 0.1, 12 }, .{ std.math.nan(f32), 0 }, .{ 0.001, 0 }, .{ 0.002, 1 },
    };
    for (cases) |c| {
        var ctx: Hair.UpdateContext = .{};
        defer ctx.deinit(allocator);
        try hair.initializeContext(&ctx, c[0], &world.system);
        try testing.expectEqual(c[1], ctx.num_iterations);
        if (c[1] == 0) {
            try testing.expectEqual(@as(f32, 0.0), ctx.delta_time);
            try testing.expectEqual(@as(f32, 1.0e12), ctx.inv_delta_time_sq);
            try testing.expectEqual(@as(f32, 1.0e12), ctx.two_div_delta_time);
        } else {
            try testing.expectEqual(math.min(c[0], settings.max_delta_time) / @as(f32, @floatFromInt(c[1])), ctx.delta_time);
        }
        try testing.expect(!ctx.needs_collision and ctx.needs_grid and !ctx.global_pose_only and !ctx.has_transform_changed);
        try testing.expect(ctx.delta_transform.eql(Mat44.identity()));
    }

    // A moved hair: the delta transform takes the old transform to the new one
    hair.setPosition(RVec3.init(0, 1, 0));
    var ctx: Hair.UpdateContext = .{};
    defer ctx.deinit(allocator);
    try hair.initializeContext(&ctx, 1.0 / 60.0, &world.system);
    try testing.expect(ctx.has_transform_changed);
    try testing.expect(ctx.delta_transform.getTranslation().isClose(Vec3.init(0, -1, 0), .{}));
    try testing.expect(hair.prev_position.eql(hair.position));
    try testing.expect(ctx.sub_step_gravity.isClose(world.system.getGravity().mulScalar(ctx.delta_time), .{}));
}

test "Hair: a groom falls onto a hull, the read back data, the colliding shapes" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();

    // A static hull below the strands, a box and a sensor hull: Hair collides with the first only
    _ = try world.addHullBox(Vec3.init(0.2, 0.05, 0.2), RVec3.init(0, -0.25, 0), .{});
    _ = try world.addBody((try BoxShapeSettings.create(allocator, Vec3.replicate(0.05), .{})).asShapeSettings(), RVec3.init(0.1, -0.1, 0), .{});
    _ = try world.addBody((try ConvexHullShapeSettings.create(allocator, &.{ Vec3.init(0, 0, 0), Vec3.init(0.1, 0, 0), Vec3.init(0, 0.1, 0), Vec3.init(0, 0, 0.1) }, .{})).asShapeSettings(), RVec3.init(-0.1, -0.1, 0), .{ .is_sensor = true });

    var settings = try testSettings(allocator, compute.system.get().?, 16, 12, .{}, .{});
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();
    try hair.initCompute(compute.system.get().?);

    // The colliding shapes: the hull and the box (the sensor is skipped)
    {
        var ctx: Hair.UpdateContext = .{};
        defer ctx.deinit(allocator);
        try hair.initializeContext(&ctx, 1.0 / 60.0, &world.system);
        try testing.expect(ctx.needs_collision);
        try testing.expectEqual(@as(usize, 2), ctx.shapes.items.len);
    }

    for (0..60) |_|
        try hair.update(1.0 / 60.0, Mat44.identity(), null, &world.system, compute.shaders.get().?, compute.system.get().?, compute.queue.get().?);
    try testing.expect(!hair.teleported);
    try testing.expectEqual(@as(u64, 1), hair.collision_shapes_cb.get().?.getSize());

    try hair.readBackGPUState(compute.queue.get().?);
    try hair.lockReadBackBuffers();
    defer hair.unlockReadBackBuffers();
    const s = settings.get().?;
    const positions = hair.getPositions().?;
    try testing.expectEqual(s.sim_vertices.items.len, positions.len);
    for (s.sim_strands.items) |strand| {
        // The roots stay, the strands do not sink into the hull (its top is at y = -0.2)
        try testing.expect(Vec3.fromFloat3(positions[strand.start_vtx]).isClose(Vec3.fromFloat3(s.sim_vertices.items[strand.start_vtx].position), .{}));
        var v = strand.start_vtx;
        while (v < strand.end_vtx) : (v += 1) {
            const p = Vec3.fromFloat3(positions[v]);
            try testing.expect(!p.isNaN());
            try testing.expect(p.getY() > -0.21);
        }
    }
    try testing.expectEqual(s.render_vertices.items.len, hair.getRenderPositions().?.len);
    try testing.expectEqual(s.neutral_density.items.len, hair.getGridVelocityAndDensity().?.len);
    try testing.expect(hair.getScalpVertices() == null);
    const velocity: *const Float3 = @ptrCast(&hair.velocities.?[1].velocity);
    try testing.expect(hair.getVelocities().at(1) == velocity);
}

test "Hair: the render positions override" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();

    var settings = try testSettings(allocator, compute.system.get().?, 4, 6, .{ .enable_collision = false }, .{});
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();

    // A callback that copies the buffer and counts its calls
    const Context = struct {
        calls: u32 = 0,
        fn convert(context: ?*anyopaque, buffer: *ComputeBuffer, out_vertices: []Float3) void {
            const self: *@This() = @ptrCast(@alignCast(context.?));
            self.calls += 1;
            @memcpy(out_vertices, buffer.mapAs(Float3, .read));
            buffer.unmap();
        }
    };
    var context: Context = .{};
    hair.overrideRenderPositionsCB(.{ .context = &context, .function = Context.convert });
    var render_positions = try compute.system.get().?.createComputeBuffer(.rw_buffer, settings.get().?.render_vertices.items.len, @sizeOf(Float3), null);
    defer render_positions.deinit();
    hair.setRenderPositionsCB(render_positions.get());
    try hair.initCompute(compute.system.get().?);
    try testing.expect(hair.getRenderPositionsCB() == render_positions.get());

    try hair.update(1.0 / 60.0, Mat44.identity(), null, &world.system, compute.shaders.get().?, compute.system.get().?, compute.queue.get().?);
    try hair.readBackGPUState(compute.queue.get().?);
    for (0..2) |_| {
        try hair.lockReadBackBuffers();
        hair.unlockReadBackBuffers();
    }
    try testing.expectEqual(@as(u32, 2), context.calls);
    const bytes = try bufferBytes(allocator, render_positions.get().?);
    defer allocator.free(bytes);
    try testing.expectEqualSlices(u8, bytes, std.mem.sliceAsBytes(hair.getRenderPositions().?));
}

/// The state of a hair after its updates: the bytes of its positions, velocities, grid and render positions buffers
const TestState = struct {
    buffers: [4][]u8,

    fn init(allocator: Allocator, hair: *const Hair) !TestState {
        var self: TestState = .{ .buffers = @splat(&.{}) };
        errdefer self.deinit(allocator);
        for ([_]*ComputeBuffer{ hair.positions_cb.get().?, hair.velocities_cb.get().?, hair.velocity_and_density_cb.get().?, hair.render_positions_cb.get().? }, 0..) |buffer, i|
            self.buffers[i] = try bufferBytes(allocator, buffer);
        return self;
    }

    fn deinit(self: *TestState, allocator: Allocator) void {
        for (self.buffers) |b| allocator.free(b);
    }

    fn expectEqual(self: *const TestState, expected: *const TestState) !void {
        for (self.buffers, expected.buffers) |a, b|
            try testing.expectEqualSlices(u8, b, a);
    }
};

/// The bodies of the sweeps: a hull below the strands, an inside out hull between them and a box. A fourth hull is
/// outside the bounds of a hair at the origin and inside them once the hair moved to x = 0.1 (sweep_updates).
fn addSweepBodies(world: *TestWorld) !void {
    _ = try world.addHullBox(Vec3.init(0.2, 0.05, 0.2), RVec3.init(0, -0.25, 0), .{});
    _ = try world.addHullBox(Vec3.init(0.03, 0.03, 0.03), RVec3.init(0.06, -0.1, 0), .{ .scale = Vec3.init(1, -1, 1) });
    _ = try world.addBody((try BoxShapeSettings.create(world.system.allocator, Vec3.replicate(0.02), .{})).asShapeSettings(), RVec3.init(-0.06, -0.1, 0), .{});
    _ = try world.addHullBox(Vec3.init(0.03, 0.03, 0.03), RVec3.init(0.24, -0.1, 0), .{});
}

/// The updates of a sweep case. The second update needs more than the first: 6 iterations instead of 4 (two more
/// iteration constant buffers), and the hair moves so that a third hull enters its bounds (larger collision buffers).
/// With a scalp, the second update is the first one that interpolates the roots: it creates the two target buffers.
const sweep_updates = [_]struct { delta_time: f32, x: f32 }{
    .{ .delta_time = 1.0 / 90.0, .x = 0.0 },
    .{ .delta_time = 1.0 / 60.0, .x = 0.1 },
};

/// The joint matrices of update `index` of a sweep case (the scalp of TestGroom has two joints)
fn sweepJoints(index: usize) [2]Mat44 {
    const f: f32 = @floatFromInt(index);
    return .{ Mat44.translation(Vec3.init(0.01 * f, 0, 0)), Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.1 * f), Vec3.init(0, -0.005 * f, 0)) };
}

const SweepPhase = enum { none, init_compute, update };

/// A case of the allocation failure sweeps
const SweepCase = struct {
    phase: SweepPhase = .none,
    /// The update under test (phase update)
    update_index: usize = 0,
    /// The phase under test gets this many allocations, then every allocation fails
    fail_after: usize = 0,
    /// Call onTeleported before this update: the state that a failed update and its retry give
    teleport_at: ?usize = null,
};

/// The result of a sweep case
const SweepResult = struct {
    /// The phase under test returned error.OutOfMemory
    failed: bool,
    state: TestState,
};

/// After a failed update: the buffers that update creates on the way are complete. The two target buffers are set
/// together, and the list of iteration constant buffers has no empty entry.
fn expectCompleteBuffers(hair: *const Hair) !void {
    try testing.expectEqual(hair.target_positions_cb.get() == null, hair.target_global_pose_transforms_cb.get() == null);
    for (hair.iteration_constants_cb.items) |b|
        try testing.expect(b.get() != null);
}

/// One case of the allocation failure sweeps. Everything that the hair uses allocates from one failing allocator: the
/// compute system with its buffers, the queue with its shader wrappers, the shaders, the settings and the hair. The
/// phase under test gets `fail_after` allocations, then every allocation fails. After a failure the case recovers: it
/// calls initCompute again, or calls onTeleported and repeats the update.
fn sweepCase(world: *TestWorld, groom: TestGroom, case: SweepCase) !SweepResult {
    var failing: std.testing.FailingAllocator = .init(testing.allocator, .{});
    const allocator = failing.allocator();
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    const system = compute.system.get().?;
    var settings = try testSettings(allocator, system, 8, 9, .{}, groom);
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();

    var failed = false;
    if (case.phase == .init_compute)
        failing.fail_index = failing.alloc_index + case.fail_after;
    hair.initCompute(system) catch |err| {
        try testing.expectEqual(error.OutOfMemory, err);
        failed = true;
        failing.fail_index = std.math.maxInt(usize);
        try hair.initCompute(system); // Call it again
    };
    failing.fail_index = std.math.maxInt(usize);
    try testing.expectEqual(groom.scalp, hair.scalp_joint_matrices_cb.get() != null);

    for (sweep_updates, 0..) |u, i| {
        hair.setPosition(RVec3.init(u.x, 0, 0));
        if (case.teleport_at == i)
            hair.onTeleported();
        const joints = sweepJoints(i);
        if (case.phase == .update and case.update_index == i)
            failing.fail_index = failing.alloc_index + case.fail_after;
        hair.update(u.delta_time, Mat44.identity(), &joints, &world.system, compute.shaders.get().?, system, compute.queue.get().?) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            failed = true;
            failing.fail_index = std.math.maxInt(usize);
            try expectCompleteBuffers(&hair);

            // The step is partly done: put the hair back in its default pose
            hair.onTeleported();
            try hair.update(u.delta_time, Mat44.identity(), &joints, &world.system, compute.shaders.get().?, system, compute.queue.get().?);
        };
        failing.fail_index = std.math.maxInt(usize);

        if (case.phase == .none and case.teleport_at == null) {
            // Without a failure the second update grows the buffers of the first (see sweep_updates)
            const num_iterations = [_]usize{ 4, 6 };
            const num_hulls = [_]u64{ 2, 3 };
            try testing.expectEqual(num_iterations[i], hair.iteration_constants_cb.items.len);
            try testing.expectEqual(num_hulls[i], hair.collision_shapes_cb.get().?.getSize());
            try testing.expectEqual(groom.scalp and i == 1, hair.target_positions_cb.get() != null);

            // The strands stay finite
            const positions = hair.positions_cb.get().?.mapAs(HairPosition, .read);
            defer hair.positions_cb.get().?.unmap();
            for (positions) |p|
                try testing.expect(!Vec3.init(p.position.x, p.position.y, p.position.z).isNaN());
        }
    }
    return .{ .failed = failed, .state = try .init(testing.allocator, &hair) };
}

test "Hair: allocation failure sweeps of initCompute and update" {
    var world: TestWorld = undefined;
    try world.init(testing.allocator);
    defer world.deinit();
    try addSweepBodies(&world);

    // Without a scalp, and with a skinned scalp (initCompute creates the scalp buffers, the second update the two target
    // buffers). The number of allocations of initCompute for each.
    var init_compute_allocations: [2]usize = undefined;
    for ([_]bool{ false, true }, &init_compute_allocations) |scalp, *num_allocations| {
        const groom: TestGroom = .{ .vertex_spread = 3, .scalp = scalp };

        // The state without a failure
        var expected = try sweepCase(&world, groom, .{});
        defer expected.state.deinit(testing.allocator);

        // initCompute: after a failure, calling it again gives the same state
        var fail_after: usize = 0;
        while (true) : (fail_after += 1) {
            var result = try sweepCase(&world, groom, .{ .phase = .init_compute, .fail_after = fail_after });
            defer result.state.deinit(testing.allocator);
            try result.state.expectEqual(&expected.state);
            if (!result.failed) break;
        }
        num_allocations.* = fail_after;

        // Every update: a failed update and its teleported retry give the state of a teleport before that update
        for (0..sweep_updates.len) |update_index| {
            var teleported = try sweepCase(&world, groom, .{ .teleport_at = update_index });
            defer teleported.state.deinit(testing.allocator);
            fail_after = 0;
            while (true) : (fail_after += 1) {
                var result = try sweepCase(&world, groom, .{ .phase = .update, .update_index = update_index, .fail_after = fail_after });
                defer result.state.deinit(testing.allocator);
                try result.state.expectEqual(if (result.failed) &teleported.state else &expected.state);
                if (!result.failed) break;
            }
            try testing.expect(fail_after > 10);
        }
    }

    // The scalp branch of initCompute allocates
    try testing.expect(init_compute_allocations[0] > 5 and init_compute_allocations[1] > init_compute_allocations[0]);
}

/// The render positions override of the read back sweeps: copies the read back buffer
fn copyRenderPositions(context: ?*anyopaque, buffer: *ComputeBuffer, out_vertices: []Float3) void {
    _ = context;
    @memcpy(out_vertices, buffer.mapAs(Float3, .read));
    buffer.unmap();
}

/// The data of a locked hair: the bytes of its positions, rotations, velocities and render positions
fn readBackState(allocator: Allocator, hair: *const Hair) !TestState {
    var self: TestState = .{ .buffers = @splat(&.{}) };
    errdefer self.deinit(allocator);
    self.buffers[0] = try allocator.dupe(u8, std.mem.sliceAsBytes(hair.getPositions().?));
    self.buffers[1] = try allocator.dupe(u8, std.mem.sliceAsBytes(hair.getRotations().?));
    self.buffers[2] = try allocator.dupe(u8, std.mem.sliceAsBytes(hair.velocities.?));
    self.buffers[3] = try allocator.dupe(u8, std.mem.sliceAsBytes(hair.getRenderPositions().?));
    return self;
}

const ReadBackPhase = enum { none, read_back, lock };

/// One case of the read back sweeps. Like sweepCase, everything allocates from one failing allocator. The hair overrides
/// its render positions, so that lockReadBackBuffers allocates. After a failure the case calls the function again.
fn readBackSweepCase(world: *TestWorld, phase: ReadBackPhase, fail_after: usize) !SweepResult {
    var failing: std.testing.FailingAllocator = .init(testing.allocator, .{});
    const allocator = failing.allocator();
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    const system = compute.system.get().?;
    var settings = try testSettings(allocator, system, 8, 9, .{}, .{ .vertex_spread = 3 });
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();
    hair.overrideRenderPositionsCB(.{ .function = copyRenderPositions });
    var render_positions = try system.createComputeBuffer(.rw_buffer, settings.get().?.render_vertices.items.len, @sizeOf(Float3), null);
    defer render_positions.deinit();
    hair.setRenderPositionsCB(render_positions.get());
    try hair.initCompute(system);
    try hair.update(1.0 / 60.0, Mat44.identity(), null, &world.system, compute.shaders.get().?, system, compute.queue.get().?);

    var failed = false;
    if (phase == .read_back)
        failing.fail_index = failing.alloc_index + fail_after;
    hair.readBackGPUState(compute.queue.get().?) catch |err| {
        try testing.expectEqual(error.OutOfMemory, err);
        failed = true;
        failing.fail_index = std.math.maxInt(usize);
        try hair.readBackGPUState(compute.queue.get().?); // Call it again
    };
    failing.fail_index = std.math.maxInt(usize);

    if (phase == .lock)
        failing.fail_index = failing.alloc_index + fail_after;
    hair.lockReadBackBuffers() catch |err| {
        try testing.expectEqual(error.OutOfMemory, err);
        failed = true;
        failing.fail_index = std.math.maxInt(usize);
        try hair.lockReadBackBuffers(); // Nothing was mapped: lock again
    };
    failing.fail_index = std.math.maxInt(usize);
    defer hair.unlockReadBackBuffers();
    return .{ .failed = failed, .state = try readBackState(testing.allocator, &hair) };
}

test "Hair: allocation failure sweeps of readBackGPUState and lockReadBackBuffers" {
    var world: TestWorld = undefined;
    try world.init(testing.allocator);
    defer world.deinit();
    try addSweepBodies(&world);

    // The read back data without a failure
    var expected = try readBackSweepCase(&world, .none, 0);
    defer expected.state.deinit(testing.allocator);

    // readBackGPUState allocates the positions, rotations and velocities, lockReadBackBuffers the render positions of the
    // override. A failed call leaks nothing, and the call after it gives the same data.
    for ([_]ReadBackPhase{ .read_back, .lock }, [_]usize{ 3, 1 }) |phase, num_allocations| {
        var fail_after: usize = 0;
        while (true) : (fail_after += 1) {
            var result = try readBackSweepCase(&world, phase, fail_after);
            defer result.state.deinit(testing.allocator);
            try result.state.expectEqual(&expected.state);
            if (!result.failed) break;
        }
        try testing.expectEqual(num_allocations, fail_after);
    }
}

test "Hair: collision without a convex hull in the bounds (R20 UB3)" {
    // Jolt creates a collision shapes buffer of 0 elements and HairUpdateVelocity reads element 0. Zolt creates 1
    // zeroed element: the far away plane of HairCalculateCollisionPlanes then has no effect.
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    _ = try world.addBody((try BoxShapeSettings.create(allocator, Vec3.init(0.2, 0.05, 0.2), .{})).asShapeSettings(), RVec3.init(0, -0.25, 0), .{});
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    var settings = try testSettings(allocator, compute.system.get().?, 6, 8, .{}, .{});
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();
    try hair.initCompute(compute.system.get().?);
    for (0..3) |_|
        try hair.update(1.0 / 60.0, Mat44.identity(), null, &world.system, compute.shaders.get().?, compute.system.get().?, compute.queue.get().?);

    try testing.expectEqual(@as(u64, 1), hair.collision_shapes_cb.get().?.getSize());
    const shapes = try bufferBytes(allocator, hair.collision_shapes_cb.get().?);
    defer allocator.free(shapes);
    try testing.expect(std.mem.allEqual(u8, shapes, 0));
    try testing.expectEqual(@as(u64, 1), hair.shape_planes_cb.get().?.getSize());
    try testing.expectEqual(@as(u64, 1), hair.shape_vertices_cb.get().?.getSize());
    const indices = hair.shape_indices_cb.get().?.mapAs(u32, .read);
    defer hair.shape_indices_cb.get().?.unmap();
    try testing.expectEqualSlices(u32, &.{0}, indices); // Only the terminator
    const materials = hair.materials_cb.get().?.mapAs(HairMaterial, .read);
    defer hair.materials_cb.get().?.unmap();
    try testing.expectEqual(@as(u32, 1), materials[0].enable_collision);

    // The strands fall through the box
    try hair.readBackGPUState(compute.queue.get().?);
    for (hair.getPositions().?) |p|
        try testing.expect(!Vec3.fromFloat3(p).isNaN());
}

test "Hair: the buffers of initCompute are zero (R20 UB4)" {
    // Jolt leaves the buffers that it creates without data uninitialized, the padding vertices of short strands too
    const allocator = testing.allocator;
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    var settings = try testSettings(allocator, compute.system.get().?, 5, 4, .{}, .{ .vertex_spread = 4 });
    defer settings.deinit();
    try testing.expect(settings.get().?.getNumVerticesPadded() > settings.get().?.sim_vertices.items.len);
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();
    try hair.initCompute(compute.system.get().?);
    for ([_]*ComputeBuffer{ hair.collision_planes_cb.get().?, hair.materials_cb.get().?, hair.previous_positions_cb.get().?, hair.positions_cb.get().?, hair.velocities_cb.get().?, hair.constants_cb.get().?, hair.velocity_and_density_cb.get().?, hair.render_positions_cb.get().? }) |buffer| {
        const bytes = try bufferBytes(allocator, buffer);
        defer allocator.free(bytes);
        try testing.expect(std.mem.allEqual(u8, bytes, 0));
    }

    // The global pose transforms are the identity
    const transforms = hair.global_pose_transforms_cb.get().?.mapAs(HairGlobalPoseTransform, .read);
    defer hair.global_pose_transforms_cb.get().?.unmap();
    for (transforms) |t| {
        try testing.expectEqual(hlsl.float3.init(0, 0, 0), t.position);
        try testing.expectEqual(hlsl.float4.init(0, 0, 0, 1), t.rotation);
    }
}

/// Run `num_updates` updates of a hair with `material` on a groom and return its state
fn runGroom(world: *TestWorld, material: HairSettings.Material, groom: TestGroom, num_vertices: u32, num_updates: u32) !TestState {
    const allocator = testing.allocator;
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    var settings = try testSettings(allocator, compute.system.get().?, 6, num_vertices, material, groom);
    defer settings.deinit();
    var hair: Hair = .init(allocator, settings.get().?, RVec3.zero(), Quat.identity(), 0);
    defer hair.deinit();
    try hair.initCompute(compute.system.get().?);
    for (0..num_updates) |_|
        try hair.update(1.0 / 60.0, Mat44.identity(), null, &world.system, compute.shaders.get().?, compute.system.get().?, compute.queue.get().?);
    return .init(allocator, &hair);
}

test "Hair: NaN grid positions of a groom without a z extent (R20 UB11)" {
    // Without padding the bounds of a flat groom have no z size: the grid scale is infinite and the grid positions are
    // NaN. Jolt's uint3(float3) of NaN depends on how clang compiles the call site, Zolt's is the scalar conversion of
    // porting guide rule 12. The update runs and two runs give the same bytes.
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();

    const groom: TestGroom = .{ .flat = true, .padding = Vec3.init(0.1, 0.1, 0.0) };
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    var settings = try testSettings(allocator, compute.system.get().?, 6, 8, .{}, groom);
    defer settings.deinit();
    const sampler: HairSettings.GridSampler = .init(settings.get().?);
    try testing.expect(std.math.isInf(sampler.scale.getZ()));

    var a = try runGroom(&world, .{}, groom, 8, 3);
    defer a.deinit(allocator);
    var b = try runGroom(&world, .{}, groom, 8, 3);
    defer b.deinit(allocator);
    try a.expectEqual(&b);
}

test "Hair: a last segment too short to change the strand length (R20 UB16)" {
    // The strand fraction of the vertex before the last is then exactly 1 (byte 255), and HairUpdateStrands reads the
    // bend compliance multiplier at index 4: Zolt reads mGridVelocityFactor.x like the measured Jolt build. A step
    // gradient on the grid velocity factor makes it infinite. The update runs and two runs give the same bytes.
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();

    const material: HairSettings.Material = .{ .enable_collision = false, .grid_velocity_factor = .init(0.05, 0.01, .{ .min_fraction = 0.5, .max_fraction = 0.5 }) };
    const groom: TestGroom = .{ .tiny_last_segment = true };
    var compute = try TestCompute.init(allocator);
    defer compute.deinit();
    var settings = try testSettings(allocator, compute.system.get().?, 6, 16, material, groom);
    defer settings.deinit();
    const s = settings.get().?;
    for (s.sim_strands.items) |strand| {
        try testing.expectEqual(@as(f32, 1.0), s.sim_vertices.items[strand.end_vtx - 2].strand_fraction);
        try testing.expect(s.sim_vertices.items[strand.end_vtx - 2].length > 0.0);
    }
    try testing.expect(std.math.isInf(HairSettings.GradientSampler.init(material.grid_velocity_factor).toFloat4().x));

    var a = try runGroom(&world, material, groom, 16, 3);
    defer a.deinit(allocator);
    var b = try runGroom(&world, material, groom, 16, 3);
    defer b.deinit(allocator);
    try a.expectEqual(&b);
}
