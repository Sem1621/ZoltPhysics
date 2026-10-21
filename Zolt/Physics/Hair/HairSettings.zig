//! Port of: Jolt/Physics/Hair/HairSettings.h, Jolt/Physics/Hair/HairSettings.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R13, R14, R20:
//! - A RefTarget without virtual functions, like SoftBodySharedSettings. It keeps the allocator that owns it and its
//!   arrays. `create(allocator)` is `new HairSettings` (reference count 0, put it in a `Ref` / `RefConst`). Settings on
//!   the stack are `.{ .allocator = allocator }` (no allocation), freed with `deinit`.
//! - The functions are Jolt's, statement by statement. The ones that allocate return `Allocator.Error` (Jolt aborts).
//!   `Init(float &outMaxDistSqHairToScalp)` is `init() Allocator.Error!f32`, `sResample` is `resample`.
//! - InitRenderAndSimulationStrands appends to the simulation arrays (Jolt's quirk, R20). It shuffles the strands of a
//!   material with a default seeded Mt19937 and `Mt19937.shuffleLibcxx`: Jolt's std::shuffle depends on the standard
//!   library, Zolt follows libc++ (R13).
//! - Gradient::MakeStepDependent uses `LibM.powf` (Jolt: std::pow, R13).
//! - The C++ GridSampler uses Vec3 min / max (the shader's GridPositionToIndexAndFraction uses std::min / std::max, each
//!   keeps its own semantics, R14).
//! - SaveBinaryState writes mSimulationBounds as the raw 32 byte AABox. Zolt's Vec3 keeps W = Z (porting guide section
//!   8, rule 9), so the bytes are Jolt's with one exception. Jolt's StreamIn::Read(Vec3) keeps the old W of
//!   mSimulationBoundsPadding when JPH_FLOATING_POINT_EXCEPTIONS_ENABLED is not defined (Jolt's CMake defines it for
//!   MSVC only). Init after RestoreBinaryState then expands the W lanes of the bounds by that old W. Zolt's W lanes are
//!   Jolt's with floating point exceptions enabled. The parity layer models Jolt's W lanes (HairSettingsParity.zig).
//! - SkinWeight, SkinPoint and SVertexInfluence are the extern structs of Shaders/HairStructs.zig (they derive from the
//!   shader structs without adding members), which carry their RTTI. The other 7 serializable classes are declared
//!   here with Jolt's nested names ("HairSettings::SVertex"). SStrand has the base RStrand (a flattened prefix).
//!   `RVertex.mInfluences[3]` is a fixed array.
//! - InitCompute: the padding vertices of short strands are zero (UB4 of R20: Jolt leaves them uninitialized; the
//!   parity reference runs with zero filling allocation hooks).
//! - Inputs where Jolt is undefined (R20; Zolt-only tests pin them):
//!   - UB8: on any material run, mSimulationStrandsFraction selects a number of strands, ceil(fraction * run size),
//!     that is NaN, negative or more than the strands left from the start of the run (a fraction that size_t cannot
//!     hold, or a fraction above 1 whose strands run past the last material). Jolt reads past the index array. Zolt
//!     panics ("HairSettings: mSimulationStrandsFraction selects more strands than there are"). A run whose simulated
//!     strands stay inside the index array is literal: they can run into the next material.
//!   - UB19: Init with a simulated strand of 0 vertices, or of 1 vertex that starts at vertex 0 or that no strand of 2
//!     or more vertices comes before (in the order of mSimStrands). Jolt reads past x or mSimVertices. Zolt panics
//!     ("HairSettings.init: a simulated strand has no vertices", "HairSettings.init: a simulated strand of 1 vertex
//!     starts at vertex 0 or comes before every longer strand").
//!   - UB20: Init with a scalp on settings that already have skin points: Init twice, or Init after restoring the state
//!     of an initialized groom. Jolt appends more skin points and reads mSimStrands past its end. Zolt panics before it
//!     finalizes the skin points ("HairSettings.init: the settings have more skin points than simulated strands").
//!   - UB19: resample with a strand of 1 vertex at the end of the vertex array, or a strand of 0 vertices at its start
//!     or end. Jolt reads past the vertex array. Zolt panics ("HairSettings.resample: a strand of 0 or 1 vertices reads
//!     past the vertex array").
//!   - UB30: Init samples a grid point past the end of mNeutralDensity. Init samples the 8 grid points around each
//!     vertex with an inv mass above 0. With a GridSize below 2 on X, Y or Z a point can be past the end: on an axis
//!     of size 1 the index is 0, and the point at index 1 is in the next row or slice. An axis of size 0 leaves no
//!     grid point. Jolt writes past mNeutralDensity there. Zolt checks each point and panics at the first one past the
//!     end, in every build mode ("HairSettings.init: a vertex samples a grid point past the end of the grid"). Points
//!     inside the grid are literal, so a grid of size 1 works when its points stay inside. Hair's grid shaders have
//!     the same check (see Hair.zig).
//!   - UB31: InitRenderAndSimulationStrands with a simulated strand of 0 vertices at the end of mSimVertices, or at its
//!     start (its last vertex index wraps, so the walk along the strand can pass the end). The search for the closest
//!     simulated strand of the other strands of its material then can read past mSimVertices in Jolt. Zolt panics at a
//!     read past the array ("HairSettings.initRenderAndSimulationStrands: a simulated strand of 0 vertices reads past
//!     mSimVertices"). The reads inside the array are literal (Init then panics for the strand, UB19).
//!   - UB33: InitCompute before Init (or after a change of the strands): a strand has more vertices than
//!     mMaxVerticesPerStrand, and Jolt writes past the transposed vertex arrays. Zolt panics ("HairSettings.initCompute:
//!     a strand has more vertices than max_vertices_per_strand (call init first)").
//!   - No test pins these panics: a test cannot catch a panic.
//! - Init with a strand of 1 vertex that a longer strand comes before follows Jolt's release build. Jolt reads x[1]
//!   past the size of x (an assert of Jolt's Array). Its Array still holds the x[1] of the last strand of 2 or more
//!   vertices there. That x[1] gives the rod length of the vertex. The Bishop frame of the vertex is then copied from
//!   the vertex before it. Zolt keeps that x[1] and reads it after a recoverable assert (R8). The HairSettings parity
//!   compares this path and counts the asserts.
//! - getNeutralDensity outside the grid follows Jolt's release build: its assert is recoverable (R8), and it reads
//!   another cell of the grid. Hair.draw gets there through Jolt's loop bound quirk (R20).
//! - resample with any other strand of 0 or 1 vertices is literal: it reads the next vertex of the array with a
//!   fraction of 0. A num_vertices_per_strand of 0 is literal too: the loop bound wraps, and resample appends vertices
//!   until memory runs out.
//! - Asserts on the input that Jolt's release build passes (rods of zero length, a strand of 256 or more vertices,
//!   material index 256 or more, resample past the end of a strand) are `if (Core.enable_asserts)` asserts.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const Mt19937 = @import("../../Core/Mt19937.zig");
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const LibM = @import("../../Math/LibM.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Float4 = @import("../../Math/Float4.zig").Float4;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const UVec4 = @import("../../Math/UVec4.zig").UVec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const IndexedTriangleFile = @import("../../Geometry/IndexedTriangle.zig");
const IndexedTriangle = IndexedTriangleFile.IndexedTriangle;
const IndexedTriangleNoMaterial = IndexedTriangleFile.IndexedTriangleNoMaterial;
const ClosestPoint = @import("../../Geometry/ClosestPoint.zig");
const TriangleSplitterBinning = @import("../../TriangleSplitter/TriangleSplitterBinning.zig").TriangleSplitterBinning;
const AABBTreeBuilderFile = @import("../../AABBTree/AABBTreeBuilder.zig");
const AABBTreeBuilder = AABBTreeBuilderFile.AABBTreeBuilder;
const AABBTreeBuilderStats = AABBTreeBuilderFile.AABBTreeBuilderStats;
const ComputeSystem = @import("../../Compute/ComputeSystem.zig").ComputeSystem;
const ComputeBuffer = @import("../../Compute/ComputeBuffer.zig").ComputeBuffer;
const HairStructs = @import("../../Shaders/HairStructs.zig");

/// This class defines the setup of a hair groom, it can be shared between multiple hair instances
pub const HairSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "HairSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mSimVertices", .sim_vertices),
            .attr("mSimStrands", .sim_strands),
            .attr("mRenderVertices", .render_vertices),
            .attr("mRenderStrands", .render_strands),
            .attr("mScalpVertices", .scalp_vertices),
            .attr("mScalpTriangles", .scalp_triangles),
            .attr("mScalpInverseBindPose", .scalp_inverse_bind_pose),
            .attr("mScalpSkinWeights", .scalp_skin_weights),
            .attr("mScalpNumSkinWeightsPerVertex", .scalp_num_skin_weights_per_vertex),
            .attr("mNumIterationsPerSecond", .num_iterations_per_second),
            .attr("mMaxDeltaTime", .max_delta_time),
            .attr("mGridSize", .grid_size),
            .attr("mSimulationBoundsPadding", .simulation_bounds_padding),
            .attr("mInitialGravity", .initial_gravity),
            .attr("mMaterials", .materials),
        },
    });

    /// How much a vertex is influenced by a joint (HairStructs.zig carries its RTTI)
    pub const SkinWeight = HairStructs.HairSkinWeight;

    /// Information about where a hair strand is attached to the scalp mesh (HairStructs.zig carries its RTTI)
    pub const SkinPoint = HairStructs.HairSkinPoint;

    /// cNoInfluence
    pub const no_influence: u32 = HairStructs.HairSVertexInfluence.no_influence;

    /// Describes how a render vertex is influenced by a simulated vertex (HairStructs.zig carries its RTTI and the
    /// defaults of Jolt's constructor)
    pub const SVertexInfluence = HairStructs.HairSVertexInfluence;

    /// A render vertex
    pub const RVertex = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::RVertex)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::RVertex",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mPosition", .position),
                .attr("mInfluences", .influences),
            },
        });

        /// Initial position of the vertex
        position: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Attach to X simulated vertices (computed during Init)
        influences: [HairStructs.hair_num_s_vertex_influences]SVertexInfluence = @splat(.{}),

        comptime {
            std.debug.assert(@sizeOf(RVertex) == 72);
        }
    };

    /// A simulated vertex in a hair strand
    pub const SVertex = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::SVertex)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::SVertex",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mPosition", .position),
                .attr("mInvMass", .inv_mass),
            },
        });

        /// Initial position of the vertex in its modeled pose
        position: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Inverse of the mass of the vertex
        inv_mass: f32 = 1.0,
        /// Initial distance of this vertex to the next of the unloaded strand, computed by Init
        length: f32 = 0.0,
        /// Fraction along the strand, 0 = start, 1 = end, computed by Init
        strand_fraction: f32 = 0.0,
        /// Bishop frame of the strand in its modeled pose, computed by Init
        bishop: Float4 = .{ .x = 0, .y = 0, .z = 0, .w = 1.0 },
        /// Conjugate(Previous Bishop) * Bishop, defines the rotation difference between the previous rod and this one of the unloaded strand, computed by Init
        omega0: Float4 = .{ .x = 0, .y = 0, .z = 0, .w = 1.0 },

        comptime {
            std.debug.assert(@sizeOf(SVertex) == 56);
        }

        /// Constructor SVertex(inPosition, inInvMass = 1.0f)
        pub fn init(position: Float3, opts: struct { inv_mass: f32 = 1.0 }) SVertex {
            return .{ .position = position, .inv_mass = opts.inv_mass };
        }
    };

    /// A hair render strand
    pub const RStrand = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::RStrand)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::RStrand",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mStartVtx", .start_vtx),
                .attr("mEndVtx", .end_vtx),
            },
        });

        // Jolt's default constructor leaves both uninitialized
        start_vtx: u32,
        end_vtx: u32,

        comptime {
            std.debug.assert(@sizeOf(RStrand) == 8);
        }

        /// Constructor RStrand(inStartVtx, inEndVtx)
        pub fn init(start_vtx: u32, end_vtx: u32) RStrand {
            return .{ .start_vtx = start_vtx, .end_vtx = end_vtx };
        }

        pub fn vertexCount(self: RStrand) u32 {
            return self.end_vtx -% self.start_vtx;
        }

        pub fn measureLength(self: RStrand, vertices: []const SVertex) f32 {
            return measureStrandLength(self.start_vtx, self.end_vtx, vertices);
        }
    };

    /// A hair simulation strand
    pub const SStrand = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::SStrand)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::SStrand",
            .kind = .non_virtual,
            .bases = &.{RStrand}, // A flattened prefix: the fields of RStrand come first
            .attributes = &.{
                .attr("mMaterialIndex", .material_index),
            },
        });

        // RStrand (Jolt's default constructor leaves both uninitialized)
        start_vtx: u32,
        end_vtx: u32,

        /// Index in mMaterials
        material_index: u32 = 0,

        comptime {
            std.debug.assert(@sizeOf(SStrand) == 12);
        }

        /// Constructor SStrand(inStartVtx, inEndVtx, inMaterialIndex)
        pub fn init(start_vtx: u32, end_vtx: u32, material_index: u32) SStrand {
            return .{ .start_vtx = start_vtx, .end_vtx = end_vtx, .material_index = material_index };
        }

        /// RStrand::VertexCount
        pub fn vertexCount(self: SStrand) u32 {
            return self.end_vtx -% self.start_vtx;
        }

        /// RStrand::MeasureLength
        pub fn measureLength(self: SStrand, vertices: []const SVertex) f32 {
            return measureStrandLength(self.start_vtx, self.end_vtx, vertices);
        }
    };

    /// RStrand::MeasureLength
    fn measureStrandLength(start_vtx: u32, end_vtx: u32, vertices: []const SVertex) f32 {
        var length: f32 = 0.0;
        var v = start_vtx;
        while (v < end_vtx -% 1) : (v += 1)
            length += Vec3.fromFloat3(vertices[v + 1].position).sub(Vec3.fromFloat3(vertices[v].position)).length();
        return length;
    }

    /// Gradient along a hair strand of a value, e.g. compliance, friction, etc.
    pub const Gradient = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::Gradient)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::Gradient",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mMin", .min),
                .attr("mMax", .max),
                .attr("mMinFraction", .min_fraction),
                .attr("mMaxFraction", .max_fraction),
            },
        });

        /// Minimum value of the gradient
        min: f32 = 0.0,
        /// Maximum value of the gradient
        max: f32 = 1.0,
        /// Fraction along the hair strand that corresponds to the minimum value
        min_fraction: f32 = 0.0,
        /// Fraction along the hair strand that corresponds to the maximum value
        max_fraction: f32 = 1.0,

        /// Constructor Gradient(inMin, inMax, inMinFraction = 0.0f, inMaxFraction = 1.0f)
        pub fn init(min: f32, max: f32, opts: struct { min_fraction: f32 = 0.0, max_fraction: f32 = 1.0 }) Gradient {
            return .{ .min = min, .max = max, .min_fraction = opts.min_fraction, .max_fraction = opts.max_fraction };
        }

        /// We drive a value to its target with fixed time steps using:
        ///
        /// x(t + fixed_dt) = target + (1 - k) * (x(t) - target)
        ///
        /// For varying time steps we can rewrite this to:
        ///
        /// x(t + dt) = target + (1 - k)^inTimeRatio * (x(t) - target)
        ///
        /// Where inTimeRatio is defined as dt / fixed_dt.
        ///
        /// This means k' = 1 - (1 - k)^inTimeRatio
        pub fn makeStepDependent(self: Gradient, time_ratio: f32) Gradient {
            const make_dependent = struct {
                fn f(ratio: f32, value: f32) f32 {
                    return 1.0 - LibM.powf(1.0 - value, ratio);
                }
            }.f;

            return .init(make_dependent(time_ratio, self.min), make_dependent(time_ratio, self.max), .{ .min_fraction = self.min_fraction, .max_fraction = self.max_fraction });
        }

        /// Saves the state of this object in binary form to inStream. Doesn't store the compute buffers.
        pub fn saveBinaryState(self: *const Gradient, stream: StreamOut) void {
            stream.write(self.min);
            stream.write(self.max);
            stream.write(self.min_fraction);
            stream.write(self.max_fraction);
        }

        /// Restore the state of this object from inStream.
        pub fn restoreBinaryState(self: *Gradient, stream: StreamIn) void {
            stream.read(&self.min);
            stream.read(&self.max);
            stream.read(&self.min_fraction);
            stream.read(&self.max_fraction);
        }
    };

    pub const GradientSampler = struct {
        multiplier: f32,
        offset: f32,
        min: f32,
        max: f32,

        pub fn init(gradient: Gradient) GradientSampler {
            const multiplier = (gradient.max - gradient.min) / (gradient.max_fraction - gradient.min_fraction);
            return .{
                .multiplier = multiplier,
                .offset = gradient.min - gradient.min_fraction * multiplier,
                .min = math.min(gradient.min, gradient.max),
                .max = math.max(gradient.min, gradient.max),
            };
        }

        /// Sample the value along the strand
        pub fn sample(self: GradientSampler, fraction: f32) f32 {
            return math.min(self.max, math.max(self.min, self.offset + fraction * self.multiplier));
        }

        /// Sample(const SStrand &inStrand, uint32 inVertex)
        pub fn sampleStrand(self: GradientSampler, strand: SStrand, vertex: u32) f32 {
            return self.sample(@as(f32, @floatFromInt(vertex -% strand.start_vtx)) / @as(f32, @floatFromInt(strand.vertexCount() -% 1)));
        }

        /// Convert to Float4 to pass to shader
        pub fn toFloat4(self: GradientSampler) Float4 {
            return .{ .x = self.multiplier, .y = self.offset, .z = self.min, .w = self.max };
        }
    };

    /// The material determines the simulation parameters for a hair strand
    pub const Material = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::Material)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "HairSettings::Material",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mEnableCollision", .enable_collision),
                .attr("mEnableLRA", .enable_lra),
                .attr("mLinearDamping", .linear_damping),
                .attr("mAngularDamping", .angular_damping),
                .attr("mMaxLinearVelocity", .max_linear_velocity),
                .attr("mMaxAngularVelocity", .max_angular_velocity),
                .attr("mGravityFactor", .gravity_factor),
                .attr("mFriction", .friction),
                .attr("mBendCompliance", .bend_compliance),
                .attr("mBendComplianceMultiplier", .bend_compliance_multiplier),
                .attr("mStretchCompliance", .stretch_compliance),
                .attr("mInertiaMultiplier", .inertia_multiplier),
                .attr("mHairRadius", .hair_radius),
                .attr("mWorldTransformInfluence", .world_transform_influence),
                .attr("mGridVelocityFactor", .grid_velocity_factor),
                .attr("mGridDensityForceFactor", .grid_density_force_factor),
                .attr("mGlobalPose", .global_pose),
                .attr("mSkinGlobalPose", .skin_global_pose),
                .attr("mSimulationStrandsFraction", .simulation_strands_fraction),
                .attr("mGravityPreloadFactor", .gravity_preload_factor),
            },
        });

        /// Enable collision detection between hair strands and the environment.
        enable_collision: bool = true,
        /// Enable Long Range Attachments to keep hair close to the modeled pose. This prevents excessive stretching when the head moves quickly.
        enable_lra: bool = true,
        /// Linear damping coefficient for the simulated rods.
        linear_damping: f32 = 2.0,
        /// Angular damping coefficient for the simulated rods.
        angular_damping: f32 = 2.0,
        /// Maximum linear velocity of a vertex.
        max_linear_velocity: f32 = 10.0,
        /// Maximum angular velocity of a vertex.
        max_angular_velocity: f32 = 50.0,
        /// How much gravity affects the hair along its length, 0 = no gravity, 1 = full gravity. Can be used to reduce the effect of gravity.
        gravity_factor: Gradient = .init(0.1, 1.0, .{ .min_fraction = 0.2, .max_fraction = 0.8 }),
        /// Collision friction coefficient. Usually in the range [0, 1]. 0 = no friction.
        friction: f32 = 0.2,
        /// Compliance for bend constraints: 1 / stiffness.
        bend_compliance: f32 = 1.0e-7,
        /// Multiplier for bend compliance at 0%, 33%, 66% and 100% of the strand length.
        bend_compliance_multiplier: Float4 = .{ .x = 1.0, .y = 100.0, .z = 100.0, .w = 1.0 },
        /// Compliance for stretch constraints: 1 / stiffness.
        stretch_compliance: f32 = 1.0e-8,
        /// Multiplier applied to the mass of a rod to calculate its inertia.
        inertia_multiplier: f32 = 10.0,
        /// Radius of the hair strand along its length, used for collision detection.
        hair_radius: Gradient = .init(0.001, 0.001, .{}),
        /// How much rotating the head influences the hair, 0 = not at all, the hair will move with the head as if it had no inertia. 1 = hair stays in place as the head moves and is correctly simulated. This can be used to reduce the effect of turning the head towards the root of strands.
        world_transform_influence: Gradient = .init(0.0, 1.0, .{}),
        /// Every iteration this fraction of the grid velocity will be applied to the vertex velocity. Defined at cDefaultIterationsPerSecond, if this changes, the value will internally be adjusted to result in the same behavior.
        grid_velocity_factor: Gradient = .init(0.05, 0.01, .{}),
        /// This factor will try to push the density of the hair towards the neutral density defined in the density grid. Note that can result in artifacts so defaults to 0.
        grid_density_force_factor: f32 = 0.0,
        /// Every iteration this fraction of the neutral pose will be applied to the vertex position. Defined at cDefaultIterationsPerSecond, if this changes, the value will internally be adjusted to result in the same behavior.
        global_pose: Gradient = .init(0.01, 0, .{ .min_fraction = 0.0, .max_fraction = 0.3 }),
        /// How much the global pose follows the skin of the scalp. 0 is not following, 1 is fully following.
        skin_global_pose: Gradient = .init(1.0, 0.0, .{ .min_fraction = 0.0, .max_fraction = 0.1 }),
        /// Used by InitRenderAndSimulationStrands only. Indicates the fraction of strands that should be simulated.
        simulation_strands_fraction: f32 = 0.1,
        /// Note: Not fully functional yet! This controls how much of the gravity we will remove from the modeled pose when initializing. A value of 1 fully removes gravity and should result in no sagging when the simulation starts. A value of 0 doesn't remove gravity.
        gravity_preload_factor: f32 = 0.0,

        /// Returns if this material needs a density/velocity grid
        pub fn needsGrid(self: *const Material) bool {
            return self.grid_velocity_factor.min != 0.0 or self.grid_velocity_factor.max != 0.0 or self.grid_density_force_factor != 0.0;
        }

        /// If this material only needs running the global pose logic
        pub fn globalPoseOnly(self: *const Material) bool {
            return !self.enable_collision and self.global_pose.min == 1.0 and self.global_pose.max == 1.0;
        }

        /// Calculate the bend compliance at a fraction along the strand
        pub fn getBendCompliance(self: *const Material, strand_fraction: f32) f32 {
            var fraction = strand_fraction * 3.0;
            const idx: u32 = math.min(truncToUint(fraction), 2);
            fraction = fraction - @as(f32, @floatFromInt(idx));
            if (Core.enable_asserts) std.debug.assert(fraction >= 0.0 and fraction <= 1.0);
            const multiplier = self.bend_compliance_multiplier.getComponent(idx) * (1.0 - fraction) + self.bend_compliance_multiplier.getComponent(idx + 1) * fraction;
            return multiplier * self.bend_compliance;
        }
    };

    pub const GridSampler = struct {
        grid_size_min2: UVec4,
        grid_size_min1: Vec3,
        grid_stride: UVec4,
        offset: Vec3,
        scale: Vec3,

        pub fn init(settings: *const HairSettings) GridSampler {
            const grid_size = settings.grid_size;
            return .{
                .grid_size_min2 = grid_size.sub(UVec4.replicate(2)),
                .grid_size_min1 = Vec3.fromVec4(grid_size.sub(UVec4.replicate(1)).toFloat()),
                .grid_stride = UVec4.init(1, grid_size.getX(), grid_size.getX() *% grid_size.getY(), 0),
                .offset = settings.simulation_bounds.min,
                .scale = Vec3.fromVec4(grid_size.toFloat()).div(settings.simulation_bounds.getSize()),
            };
        }

        pub const IndexAndFraction = struct {
            index: UVec4,
            fraction: Vec3,
        };

        /// Convert a position in hair space to a grid index and fraction
        pub fn positionToIndexAndFraction(self: *const GridSampler, position: Vec3) IndexAndFraction {
            // Get position in grid space
            const grid_pos = Vec3.min(Vec3.max(position.sub(self.offset), Vec3.zero()).mul(self.scale), self.grid_size_min1);
            const index = UVec4.min(Vec4.fromVec3(grid_pos).toInt(), self.grid_size_min2);
            return .{ .index = index, .fraction = grid_pos.sub(Vec3.fromVec4(index.toFloat())) };
        }

        /// Sample(UVec4Arg inIndex, Vec3Arg inFraction, const F &inFunc): calls `func.call(index, fraction)` for the 8
        /// grid points around the position
        pub fn sampleIndexAndFraction(self: *const GridSampler, index: UVec4, fraction_in: Vec3, func: anytype) void {
            const fraction = [2]Vec3{ Vec3.replicate(1.0).sub(fraction_in), fraction_in };

            // Sample the grid
            for (0..2) |z| {
                for (0..2) |y| {
                    for (0..2) |x| {
                        const grid_index = self.grid_stride.dot(index.add(UVec4.init(@intCast(x), @intCast(y), @intCast(z), 0)));
                        const combined_fraction = fraction[x].getX() * fraction[y].getY() * fraction[z].getZ();
                        func.call(grid_index, combined_fraction);
                    }
                }
            }
        }

        /// Sample(Vec3Arg inPosition, const F &inFunc)
        pub fn sample(self: *const GridSampler, position: Vec3, func: anytype) void {
            const r = self.positionToIndexAndFraction(position);
            self.sampleIndexAndFraction(r.index, r.fraction, func);
        }
    };

    pub const default_iterations_per_second: u32 = 360;

    /// RefTarget<HairSettings>
    ref_count: RefCount = .{},
    /// Allocates the arrays and frees the settings
    allocator: Allocator,

    /// Simulated vertices. Used by mSimStrands.
    sim_vertices: std.ArrayList(SVertex) = .empty,
    /// Defines the start and end of each simulated strand.
    sim_strands: std.ArrayList(SStrand) = .empty,

    /// Rendered vertices. Used by mRenderStrands.
    render_vertices: std.ArrayList(RVertex) = .empty,
    /// Defines the start and end of each rendered strand.
    render_strands: std.ArrayList(RStrand) = .empty,

    /// Vertices of the scalp mesh, used to attach hairs. Note that the hair vertices mSimVertices must be in the same space as these vertices.
    scalp_vertices: std.ArrayList(Float3) = .empty,
    /// Triangles of the scalp mesh.
    scalp_triangles: std.ArrayList(IndexedTriangleNoMaterial) = .empty,
    /// Inverse bind pose of the scalp mesh, joints are in model space
    scalp_inverse_bind_pose: std.ArrayList(Mat44) = .empty,
    /// Skin weights of the scalp mesh, for each vertex we have mScalpNumSkinWeightsPerVertex entries
    scalp_skin_weights: std.ArrayList(SkinWeight) = .empty,
    /// Number of skin weights per vertex
    scalp_num_skin_weights_per_vertex: u32 = 0,

    num_iterations_per_second: u32 = default_iterations_per_second,
    /// Maximum delta time for the simulation step (to avoid running an excessively long step, note that this will effectively slow down time)
    max_delta_time: f32 = 1.0 / 30.0,
    /// Number of grid cells used to simulate the hair. W unused.
    grid_size: UVec4 = .init(32, 32, 32, 0),
    /// Padding around the simulation bounds to ensure that the grid is large enough and that we detect collisions with the hairs. This is added on all sides after calculating the bounds in the neutral pose.
    simulation_bounds_padding: Vec3 = .replicate(0.1),
    /// Initial gravity in local space of the hair, used to calculate the unloaded rest pose
    initial_gravity: Vec3 = .init(0, -9.81, 0),
    /// Materials used by the hair strands
    materials: std.ArrayList(Material) = .empty,

    // Values computed by Init

    /// For each simulated vertex, where it is attached to the scalp mesh
    skin_points: std.ArrayList(SkinPoint) = .empty,
    /// Bounds that the simulation is supposed to fit in
    simulation_bounds: AABox = .fromCenterAndRadius(.zero(), 1.0),
    /// Neutral density grid used to apply forces to keep the hair in place
    neutral_density: std.ArrayList(f32) = .empty,
    /// Highest density value in the neutral density grid, used to scale the density for rendering
    density_scale: f32 = 0.0,
    /// Maximum number of vertices per strand, used for padding the compute buffers
    max_vertices_per_strand: u32 = 0,

    // Compute data
    scalp_vertices_cb: Ref(ComputeBuffer) = .empty,
    scalp_triangles_cb: Ref(ComputeBuffer) = .empty,
    scalp_skin_weights_cb: Ref(ComputeBuffer) = .empty,
    skin_points_cb: Ref(ComputeBuffer) = .empty,
    vertices_fixed_cb: Ref(ComputeBuffer) = .empty,
    vertices_position_cb: Ref(ComputeBuffer) = .empty,
    vertices_bishop_cb: Ref(ComputeBuffer) = .empty,
    vertices_omega0_cb: Ref(ComputeBuffer) = .empty,
    vertices_length_cb: Ref(ComputeBuffer) = .empty,
    vertices_strand_fraction_cb: Ref(ComputeBuffer) = .empty,
    strand_vertex_counts_cb: Ref(ComputeBuffer) = .empty,
    strand_material_index_cb: Ref(ComputeBuffer) = .empty,
    neutral_density_cb: Ref(ComputeBuffer) = .empty,
    s_vertex_influences_cb: Ref(ComputeBuffer) = .empty,

    /// new HairSettings: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*HairSettings {
        const self = try allocator.create(HairSettings);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// new HairSettings (RTTI create)
    pub const createDefault = create;

    /// Destructor: frees the arrays and releases the compute buffers
    pub fn deinit(self: *HairSettings) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *HairSettings) void {
        const allocator = self.allocator;
        self.sim_vertices.deinit(allocator);
        self.sim_strands.deinit(allocator);
        self.render_vertices.deinit(allocator);
        self.render_strands.deinit(allocator);
        self.scalp_vertices.deinit(allocator);
        self.scalp_triangles.deinit(allocator);
        self.scalp_inverse_bind_pose.deinit(allocator);
        self.scalp_skin_weights.deinit(allocator);
        self.materials.deinit(allocator);
        self.skin_points.deinit(allocator);
        self.neutral_density.deinit(allocator);
        self.scalp_vertices_cb.deinit();
        self.scalp_triangles_cb.deinit();
        self.scalp_skin_weights_cb.deinit();
        self.skin_points_cb.deinit();
        self.vertices_fixed_cb.deinit();
        self.vertices_position_cb.deinit();
        self.vertices_bishop_cb.deinit();
        self.vertices_omega0_cb.deinit();
        self.vertices_length_cb.deinit();
        self.vertices_strand_fraction_cb.deinit();
        self.strand_vertex_counts_cb.deinit();
        self.strand_material_index_cb.deinit();
        self.neutral_density_cb.deinit();
        self.s_vertex_influences_cb.deinit();
    }

    /// Add a reference to this object
    pub fn addRef(self: *const HairSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const HairSettings) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this object as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const HairSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const HairSettings) u32 {
        return self.ref_count.get();
    }

    /// `mSimVertices[index].mPosition` in the closest strand search of initRenderAndSimulationStrands. Zolt (R20 UB31): an
    /// index past the array panics, where Jolt reads past the array. A simulated strand of 0 vertices gets there: at the
    /// end of the array, or at vertex 0 (its last vertex index wraps, so the walk can pass the end).
    fn closestSearchPosition(sim_vertices: []const SVertex, index: u32) Float3 {
        if (index >= sim_vertices.len)
            @panic("HairSettings.initRenderAndSimulationStrands: a simulated strand of 0 vertices reads past mSimVertices");
        return sim_vertices[index].position;
    }

    /// Split the supplied render strands into render and simulation strands and calculate connections between them.
    /// When this function returns mSimVertices, mSimStrands, mRenderVertices and mRenderStrands are overwritten.
    /// @param vertices Vertices for the strands.
    /// @param strands The strands that this instance should have.
    /// Zolt: Jolt's function appends to mSimVertices and mSimStrands (R20, reproduced). An allocation failure returns
    /// `error.OutOfMemory`; the four arrays may then hold part of the result.
    pub fn initRenderAndSimulationStrands(self: *HairSettings, vertices: []const SVertex, strands: []const SStrand) Allocator.Error!void {
        const allocator = self.allocator;

        // Copy original strands to render strands
        try resizeDefault(RVertex, allocator, &self.render_vertices, vertices.len);
        for (0..vertices.len) |i|
            self.render_vertices.items[i].position = vertices[i].position;
        try self.render_strands.resize(allocator, strands.len);
        for (0..strands.len) |i|
            self.render_strands.items[i] = .init(strands[i].start_vtx, strands[i].end_vtx);

        // Create buffer that holds indices to the strands
        const indices_shuffle = try allocator.alloc(u32, strands.len);
        defer allocator.free(indices_shuffle);
        for (indices_shuffle, 0..) |*index, i|
            index.* = @intCast(i);

        // Order on material index
        quickSort(u32, indices_shuffle, strands, struct {
            fn lessThan(s: []const SStrand, lhs: u32, rhs: u32) bool {
                return s[lhs].material_index < s[rhs].material_index;
            }
        }.lessThan);

        // Loop over all materials
        const num_indices = indices_shuffle.len;
        var begin_material: usize = 0;
        while (begin_material < num_indices) {
            const material_index: u32 = strands[indices_shuffle[begin_material]].material_index;

            // Find end of this material
            var end_material = begin_material;
            while (true) {
                end_material += 1;
                if (!(end_material < num_indices and strands[indices_shuffle[end_material]].material_index == material_index)) break;
            }

            // Select X% random strands to simulate
            var random = Mt19937.init(Mt19937.default_seed);
            random.shuffleLibcxx(u32, indices_shuffle[begin_material..end_material]);
            const simulated = @ceil(@as(f64, self.materials.items[material_index].simulation_strands_fraction) * @as(f64, @floatFromInt(end_material - begin_material)));
            // Zolt (R20 UB8): Jolt reads past the index array when ceil(fraction * run size) is NaN, negative or more
            // than the strands left from the start of this run (on any run)
            if (!(simulated >= 0.0 and simulated <= @as(f64, @floatFromInt(num_indices - begin_material))))
                @panic("HairSettings: mSimulationStrandsFraction selects more strands than there are");
            const num_simulated: usize = @max(@as(usize, @intFromFloat(simulated)), 1);
            const end_simulation = begin_material + num_simulated;
            if (end_simulation > num_indices) // A fraction of 0 on an empty run cannot happen (a run has a strand), max(0, 1) fits
                @panic("HairSettings: mSimulationStrandsFraction selects more strands than there are");
            quickSort(u32, indices_shuffle[begin_material..end_simulation], {}, struct { // Sort simulated strands back to original order
                fn lessThan(_: void, lhs: u32, rhs: u32) bool {
                    return lhs < rhs;
                }
            }.lessThan);
            for (indices_shuffle[begin_material..end_simulation]) |idx| {
                // Add simulation strand
                const sim_strand = strands[idx];
                const first: u32 = @intCast(self.sim_vertices.items.len);
                try self.sim_strands.append(allocator, .init(first, first +% sim_strand.vertexCount(), sim_strand.material_index));

                var v = sim_strand.start_vtx;
                while (v < sim_strand.end_vtx) : (v += 1) {
                    // Link render vertex to simulation vertex
                    self.render_vertices.items[v].influences[0].vertex_index = @intCast(self.sim_vertices.items.len);

                    // Add simulation vertex
                    try self.sim_vertices.append(allocator, vertices[v]);
                }
            }

            // Get influences for remaining strands
            var idx_pos = end_simulation;
            while (idx_pos < end_material) : (idx_pos += 1) {
                const render_strand = strands[indices_shuffle[idx_pos]];

                // Find closest simulation strand
                var closest_d_sq: f32 = math.flt_max;
                var closest_strand_idx: u32 = 0;
                for (self.sim_strands.items, 0..) |sim_strand, sim_strand_idx| {
                    if (sim_strand.material_index == render_strand.material_index) {
                        // Get the first 2 vertices of the simulation strand
                        const v_max: u32 = sim_strand.end_vtx -% 1;
                        var v: u32 = sim_strand.start_vtx;
                        var v_next: u32 = math.min(v +% 1, v_max);
                        var v_pos = Vec3.fromFloat3(closestSearchPosition(self.sim_vertices.items, v));
                        var v_next_pos = Vec3.fromFloat3(closestSearchPosition(self.sim_vertices.items, v_next));

                        // Track total error when selecting this sim strand as parent for the render strand
                        var d_sq_total: f32 = 0.0;

                        // Loop over the render strand
                        var rv = render_strand.start_vtx;
                        while (rv < render_strand.end_vtx) : (rv += 1) {
                            const rv_pos = Vec3.fromFloat3(self.render_vertices.items[rv].position);

                            // Find closest simulated vertex (note that we assume that the strands do not loop back
                            // on themselves so that an earlier vertex in the strand could be the closest)
                            var d_sq = rv_pos.sub(v_pos).lengthSq();
                            var d_sq_next = rv_pos.sub(v_next_pos).lengthSq();
                            while (d_sq_next < d_sq) {
                                // Get the next vertex of the simulation strand
                                v = v_next;
                                v_next = math.min(v +% 1, v_max);
                                v_pos = v_next_pos;
                                v_next_pos = Vec3.fromFloat3(closestSearchPosition(self.sim_vertices.items, v_next));

                                // Update distance to render vertex
                                d_sq = d_sq_next;
                                d_sq_next = rv_pos.sub(v_next_pos).lengthSq();
                            }

                            // Accumulate total error
                            d_sq_total += d_sq;

                            // No point in continuing the search if our result is worse already
                            if (d_sq_total > closest_d_sq)
                                break;
                        }

                        // If this is the smallest error, accept
                        if (d_sq_total < closest_d_sq) {
                            closest_d_sq = d_sq_total;
                            closest_strand_idx = @intCast(sim_strand_idx);
                        }
                    }
                }
                const closest_strand = self.sim_strands.items[closest_strand_idx];

                // Link render vertices to simulation vertices
                var v = render_strand.start_vtx;
                while (v < render_strand.end_vtx) : (v += 1) {
                    const rv = &self.render_vertices.items[v];

                    // Find closest simulated vertex
                    closest_d_sq = math.flt_max;
                    var cv = closest_strand.start_vtx;
                    while (cv < closest_strand.end_vtx) : (cv += 1) {
                        const d_sq = Vec3.fromFloat3(self.sim_vertices.items[cv].position).sub(Vec3.fromFloat3(rv.position)).lengthSq();
                        if (d_sq < closest_d_sq) {
                            closest_d_sq = d_sq;
                            rv.influences[0].vertex_index = cv;
                        }
                    }
                }
            }

            // Next material
            begin_material = end_material;
        }
    }

    /// Resample the hairs to a new fixed number of vertices per strand. Must be called prior to Init if desired.
    /// (sResample.) Zolt: an allocation failure returns `error.OutOfMemory` and leaves both arrays unchanged. The arrays
    /// are owned by `allocator`.
    pub fn resample(allocator: Allocator, io_vertices: *std.ArrayList(SVertex), io_strands: *std.ArrayList(SStrand), num_vertices_per_strand: u32) Allocator.Error!void {
        const vertices = io_vertices.items;
        const strands = io_strands.items;
        var out_vertices: std.ArrayList(SVertex) = .empty;
        errdefer out_vertices.deinit(allocator);
        var out_strands: std.ArrayList(SStrand) = .empty;
        errdefer out_strands.deinit(allocator);

        for (strands) |strand| {
            // Zolt: Jolt reads past the vertex array for a strand of 1 vertex at the end of the array and for a strand of
            // 0 vertices at its start or end (see the header)
            if (@as(usize, strand.start_vtx) + 1 >= vertices.len or strand.end_vtx == 0)
                @panic("HairSettings.resample: a strand of 0 or 1 vertices reads past the vertex array");

            // Determine output strand
            var out_strand: SStrand = .{ .start_vtx = undefined, .end_vtx = undefined };
            out_strand.start_vtx = @intCast(out_vertices.items.len);
            out_strand.end_vtx = out_strand.start_vtx +% num_vertices_per_strand;
            out_strand.material_index = strand.material_index;
            try out_strands.append(allocator, out_strand);

            // Measure length of the strand
            const length = strand.measureLength(vertices);

            // Add the first vertex of the strand
            try out_vertices.append(allocator, vertices[strand.start_vtx]);

            // Resample the strand
            var cur_length: f32 = 0.0;
            var v0: usize = strand.start_vtx;
            var v1: usize = strand.start_vtx + 1;
            var segment_length = Vec3.fromFloat3(vertices[v1].position).sub(Vec3.fromFloat3(vertices[v0].position)).length();
            var resampled_point: u32 = 1;
            while (resampled_point < num_vertices_per_strand -% 1) : (resampled_point += 1) {
                const desired_len = @as(f32, @floatFromInt(resampled_point)) * length / @as(f32, @floatFromInt(num_vertices_per_strand -% 1));

                while (cur_length + segment_length < desired_len) {
                    cur_length += segment_length;
                    v0 += 1;
                    v1 += 1;
                    if (Core.enable_asserts) std.debug.assert(v1 < strand.end_vtx);
                    segment_length = Vec3.fromFloat3(vertices[v1].position).sub(Vec3.fromFloat3(vertices[v0].position)).length();
                }

                var out_v = vertices[v0];
                const fraction = (desired_len - cur_length) / segment_length;
                const p0 = Vec3.fromFloat3(vertices[v0].position);
                p0.add(Vec3.fromFloat3(vertices[v1].position).sub(p0).mulScalar(fraction)).storeFloat3(&out_v.position);
                out_v.inv_mass = if (vertices[v0].inv_mass + (vertices[v1].inv_mass - vertices[v0].inv_mass) * fraction < 0.5) 0.0 else 1.0;
                try out_vertices.append(allocator, out_v);
            }

            // Add the last vertex of the strand
            try out_vertices.append(allocator, vertices[strand.end_vtx - 1]);

            if (Core.enable_asserts) std.debug.assert(out_vertices.items.len == out_strand.end_vtx);
        }

        io_vertices.deinit(allocator);
        io_vertices.* = out_vertices;
        io_strands.deinit(allocator);
        io_strands.* = out_strands;
    }

    /// sHairSettingsFindClosestTriangle
    fn findClosestTriangle(point: Vec3, builder: *const AABBTreeBuilder, node: *const AABBTreeBuilder.Node, scalp_vertices: []const Float3, closest_dist_sq: *f32, skin_point: *SkinPoint) void {
        if (node.hasChildren()) {
            // Get children
            var child0 = node.getChild(0, builder.getNodes());
            var child1 = node.getChild(1, builder.getNodes());

            // Order so that the first one is closest
            var dist_sq0: f32 = if (child0) |c| c.bounds.getSqDistanceTo(point) else math.flt_max;
            var dist_sq1: f32 = if (child1) |c| c.bounds.getSqDistanceTo(point) else math.flt_max;
            if (dist_sq1 < dist_sq0) {
                std.mem.swap(?*const AABBTreeBuilder.Node, &child0, &child1);
                std.mem.swap(f32, &dist_sq0, &dist_sq1);
            }

            // Visit in order of closeness
            if (dist_sq0 < closest_dist_sq.*)
                findClosestTriangle(point, builder, child0.?, scalp_vertices, closest_dist_sq, skin_point);
            if (dist_sq1 < closest_dist_sq.*)
                findClosestTriangle(point, builder, child1.?, scalp_vertices, closest_dist_sq, skin_point);
        } else {
            // Loop over the triangles
            const triangles = builder.getTriangles()[node.triangles_begin..][0..node.num_triangles];
            for (triangles) |t| {
                const v0 = Vec3.fromFloat3(scalp_vertices[t.idx[0]]).sub(point);
                const v1 = Vec3.fromFloat3(scalp_vertices[t.idx[1]]).sub(point);
                const v2 = Vec3.fromFloat3(scalp_vertices[t.idx[2]]).sub(point);

                // Check if it is the closest triangle
                const closest_point = ClosestPoint.getClosestPointOnTriangle(v0, v1, v2, .{}).point;
                const dist_sq = closest_point.lengthSq();
                if (dist_sq < closest_dist_sq.*) {
                    closest_dist_sq.* = dist_sq;
                    skin_point.triangle_index = t.material_index;

                    // Get barycentric coordinates of attachment point
                    const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(v0, v1, v2);
                    skin_point.u = bary.u;
                    skin_point.v = bary.v;
                }
            }
        }
    }

    /// Initialize the structure, calculating simulation bounds and vertex properties
    /// Returns outMaxDistSqHairToScalp: maximum distance^2 the root vertex of a hair is from the scalp, can be used to
    /// check if the hair matched the scalp correctly.
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts); the settings may then be partly
    /// initialized.
    pub fn init(self: *HairSettings) Allocator.Error!f32 {
        const allocator = self.allocator;
        var max_dist_sq_hair_to_scalp: f32 = 0.0;

        if (self.scalp_triangles.items.len != 0) {
            // Build a tree for all scalp triangles
            var triangles: std.ArrayList(IndexedTriangle) = .empty;
            defer triangles.deinit(allocator);
            try triangles.ensureTotalCapacity(allocator, self.scalp_triangles.items.len);
            for (self.scalp_triangles.items, 0..) |t, t_index|
                triangles.appendAssumeCapacity(.init(t.idx[0], t.idx[1], t.idx[2], .{ .material_index = @intCast(t_index) }));
            var splitter = try TriangleSplitterBinning.init(allocator, self.scalp_vertices.items, triangles.items, .{});
            var builder: AABBTreeBuilder = .init(splitter.splitter(), .{ .max_triangles_per_leaf = 8 });
            defer builder.deinit(allocator);
            var builder_stats: AABBTreeBuilderStats = .{};
            const root = builder.build(allocator, &builder_stats) catch |err| {
                splitter.deinit(allocator);
                return err;
            };
            splitter.deinit(allocator);

            try self.skin_points.ensureTotalCapacity(allocator, self.sim_strands.items.len);
            for (self.sim_strands.items) |strand| {
                var sp: SkinPoint = undefined;
                sp.triangle_index = 0;
                sp.u = 0.0;
                sp.v = 0.0;
                sp.to_bishop = 0; // Zolt: Jolt leaves it uninitialized until the loop below

                // Get root position
                var p = Vec3.fromFloat3(self.sim_vertices.items[strand.start_vtx].position);

                // Find closest triangle on scalp
                var closest_dist_sq: f32 = math.flt_max;
                findClosestTriangle(p, &builder, root, self.scalp_vertices.items, &closest_dist_sq, &sp);
                max_dist_sq_hair_to_scalp = math.max(max_dist_sq_hair_to_scalp, closest_dist_sq);

                // Project root to the triangle as we will during simulation.
                // This ensures that we calculate the Bishop frame for the root correctly.
                const t = self.scalp_triangles.items[sp.triangle_index];
                const v0 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[0]]);
                const v1 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[1]]);
                const v2 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[2]]);
                p = v0.mulScalar(sp.u).add(v1.mulScalar(sp.v)).add(v2.mulScalar(1.0 - sp.u - sp.v));
                p.storeFloat3(&self.sim_vertices.items[strand.start_vtx].position);

                try self.skin_points.append(allocator, sp);
            }
        }

        var r: std.ArrayList(Vec3) = .empty; // Outside loop to avoid reallocations
        defer r.deinit(allocator);
        var x: std.ArrayList(Vec3) = .empty;
        defer x.deinit(allocator);
        var k: std.ArrayList(Vec3) = .empty; // (bend_compliance, bend_compliance, stretch_compliance)
        defer k.deinit(allocator);
        var g: std.ArrayList(Vec3) = .empty;
        defer g.deinit(allocator);
        var bishop: std.ArrayList(Quat) = .empty;
        defer bishop.deinit(allocator);
        // Zolt: x[1] of the last strand of 2 or more vertices. Jolt's Array keeps it when x shrinks, and a strand of 1
        // vertex reads it (see the header).
        var last_x1: ?Vec3 = null;
        self.max_vertices_per_strand = 0;
        for (self.sim_strands.items) |strand| {
            // Calculate max number of vertices per strand
            const vertex_count = strand.vertexCount();
            self.max_vertices_per_strand = math.max(self.max_vertices_per_strand, vertex_count);

            // Zolt: Jolt reads past its arrays for a strand of 0 vertices, and for a strand of 1 vertex that starts at
            // vertex 0 or that no strand of 2 or more vertices comes before (see the header)
            if (vertex_count == 0)
                @panic("HairSettings.init: a simulated strand has no vertices");
            if (vertex_count == 1 and (strand.start_vtx == 0 or last_x1 == null))
                @panic("HairSettings.init: a simulated strand of 1 vertex starts at vertex 0 or comes before every longer strand");

            // Calculate strand fraction for each vertex
            const total_length = strand.measureLength(self.sim_vertices.items);
            var cur_length: f32 = 0.0;
            var i: u32 = strand.start_vtx;
            while (i < strand.end_vtx - 1) : (i += 1) {
                const v = &self.sim_vertices.items[i];
                v.strand_fraction = cur_length / total_length;
                cur_length += Vec3.fromFloat3(self.sim_vertices.items[i + 1].position).sub(Vec3.fromFloat3(v.position)).length();
            }
            self.sim_vertices.items[strand.end_vtx - 1].strand_fraction = 1.0;

            // Particles
            // i=0     1       2
            // +------>+------>+
            //    x1      x2
            //
            // Let r_i be the edge between particle i - 1 and i in the rest pose
            // Let x_i be the edge between particle i - 1 and i in the deformed pose
            //
            // The force on particle i is:
            // f_i = k_i * (r_i - x_i) - k_{i+1} * (r_{i+1} - x_{i+1})
            // Where k_i = 1 / compliance_i
            //
            // We want to counter gravity, so:
            // f_i = -m_i * g
            //
            // Rearranging gives:
            // x_{i+1} * k_{i+1} - x_i * k_i = k_{i+1} * r_{i+1} - k_i * r_i + m_i * g
            //
            // Solving this with Gauss Seidel iteration:
            // x_i = (k_i * r_i - k_{i+1} * (r_{i+1} - x_{i+1}) - m_i * g) / k_i

            try r.resize(allocator, vertex_count); // Rest edge
            try x.resize(allocator, vertex_count); // Deformed edge
            try k.resize(allocator, vertex_count); // Spring constant
            try g.resize(allocator, vertex_count); // Gravity
            try bishop.resize(allocator, vertex_count);

            // First element unused
            x.items[0] = Vec3.nan();
            r.items[0] = Vec3.nan();
            g.items[0] = Vec3.nan();
            k.items[0] = Vec3.nan();

            const material = &self.materials.items[strand.material_index];
            const gravity_sampler: GradientSampler = .init(material.gravity_factor);
            i = 1;
            while (i < vertex_count) : (i += 1) {
                const v1 = self.sim_vertices.items[strand.start_vtx + i - 1];
                const v2 = self.sim_vertices.items[strand.start_vtx + i];
                r.items[i] = Vec3.fromFloat3(v2.position).sub(Vec3.fromFloat3(v1.position));
                x.items[i] = r.items[i];
                const min_compliance: f32 = 1.0e-10;
                const bend_compliance = 1.0 / math.max(min_compliance, material.getBendCompliance(v2.strand_fraction));
                const stretch_compliance = 1.0 / math.max(min_compliance, material.stretch_compliance);
                k.items[i] = Vec3.init(bend_compliance, bend_compliance, stretch_compliance);
                g.items[i] = if (v2.inv_mass > 0.0) self.initial_gravity.mulScalar(material.gravity_preload_factor / v2.inv_mass).mulScalar(gravity_sampler.sample(v2.strand_fraction)) else Vec3.zero();
            }

            // Solve for x
            if (material.gravity_preload_factor > 0.0) {
                for (0..10) |_| {
                    // Don't modify the 1st vertex since it's fixed
                    // Loop backwards so that we can use the latest value of x[i + 1]
                    var j: u32 = vertex_count - 1;
                    while (j >= 1) : (j -= 1) {
                        // Calculate reference frame for this edge
                        const frame_x = x.items[j].normalized();
                        const frame_y = frame_x.getNormalizedPerpendicular();
                        const frame_z = frame_x.cross(frame_y);
                        const frame = Mat44.init(Vec4.fromVec3W(frame_y, 0), Vec4.fromVec3W(frame_z, 0), Vec4.fromVec3W(frame_x, 0), Vec4.init(0, 0, 0, 1));

                        // Gauss Seidel iteration
                        // Note that we take all quantities to local space so that we can separate bend and stretch compliance and apply those as a simple vector multiplication
                        var x_local = k.items[j].mul(frame.multiply3x3Transposed(r.items[j])).sub(frame.multiply3x3Transposed(g.items[j]));
                        if (j < vertex_count - 1)
                            x_local = x_local.sub(k.items[j + 1].mul(frame.multiply3x3Transposed(r.items[j + 1].sub(x.items[j + 1]))));
                        x.items[j] = frame.multiply3x3(x_local.div(k.items[j]));
                    }
                }
            }

            // Calculate the Bishop frame for the first rod in the strand
            {
                const v1 = &self.sim_vertices.items[strand.start_vtx];
                var tangent = if (vertex_count >= 2) x.items[1] else stale: {
                    // Zolt: Jolt's x[1] is past the size of x (the assert of Array::operator[]). Its release build reads
                    // the x[1] of the last strand of 2 or more vertices.
                    if (Core.enable_asserts) Core.assertFailed("inIdx < mSize", @src());
                    break :stale last_x1.?;
                };
                v1.length = tangent.length();
                if (Core.enable_asserts) std.debug.assert(v1.length > 0.0); // Rods of zero length are not supported!
                tangent = tangent.divScalar(v1.length);
                const normal = tangent.getNormalizedPerpendicular();
                const binormal = tangent.cross(normal);
                bishop.items[0] = Mat44.init(Vec4.fromVec3W(normal, 0), Vec4.fromVec3W(binormal, 0), Vec4.fromVec3W(tangent, 0), Vec4.init(0, 0, 0, 1)).getQuaternion().normalized();
                bishop.items[0].storeFloat4(&v1.bishop);
            }

            // Calculate the Bishop frames for the rest of the rods in the strand
            i = 1;
            while (i < vertex_count - 1) : (i += 1) {
                const v1 = &self.sim_vertices.items[strand.start_vtx + i];
                const v2 = self.sim_vertices.items[strand.start_vtx + i + 1];

                // Get the normal and tangent of the first rod's Bishop frame (that was already calculated)
                const r1_frame = Mat44.rotationQuat(bishop.items[i - 1]);
                const tangent1 = r1_frame.getAxisZ();
                const normal1 = r1_frame.getAxisX();

                // Calculate the Bishop frame for the 2nd rod
                var tangent2 = x.items[i + 1];
                v1.length = tangent2.length();
                if (Core.enable_asserts) std.debug.assert(v1.length > 0.0); // Rods of zero length are not supported!
                tangent2 = tangent2.divScalar(v1.length);
                var t1_cross_t2 = tangent1.cross(tangent2);
                const sin_angle = t1_cross_t2.length();
                var normal2 = normal1;
                if (sin_angle > 1.0e-6) {
                    // Rotate normal2
                    t1_cross_t2 = t1_cross_t2.divScalar(sin_angle);
                    normal2 = Quat.rotation(t1_cross_t2, trigonometry.asin(sin_angle)).mulVec3(normal2);

                    // Ensure normal2 is perpendicular to tangent2
                    normal2 = normal2.sub(tangent2.mulScalar(normal2.dot(tangent2)));
                    normal2 = normal2.normalized();
                }
                const binormal2 = tangent2.cross(normal2);
                bishop.items[i] = Mat44.init(Vec4.fromVec3W(normal2, 0), Vec4.fromVec3W(binormal2, 0), Vec4.fromVec3W(tangent2, 0), Vec4.init(0, 0, 0, 1)).getQuaternion().normalized();

                // Calculate the delta, used in simulation
                bishop.items[i - 1].conjugated().mul(bishop.items[i]).normalized().storeFloat4(&v1.omega0);

                // Calculate the Bishop frame in the modeled pose for initializing the simulation
                const modeled_tangent2 = Vec3.fromFloat3(v2.position).sub(Vec3.fromFloat3(v1.position)).normalized();
                const modeled_bishop = Quat.fromTo(tangent2, modeled_tangent2).mul(bishop.items[i]);
                modeled_bishop.storeFloat4(&v1.bishop);
            }

            // Copy Bishop frame to the last vertex
            self.sim_vertices.items[strand.end_vtx - 1].bishop = self.sim_vertices.items[strand.end_vtx - 2].bishop;

            // Zolt: keep x[1] for a later strand of 1 vertex
            if (vertex_count >= 2)
                last_x1 = x.items[1];
        }

        // Zolt: the skin points of an earlier Init (Init twice with a scalp, or Init after restoring the state of an
        // initialized groom) make Jolt read mSimStrands past its end below (see the header)
        if (self.skin_points.items.len > self.sim_strands.items.len)
            @panic("HairSettings.init: the settings have more skin points than simulated strands (init called twice with a scalp?)");

        // Finalize skin points by calculating how to go from triangle frame to Bishop frame
        for (self.skin_points.items, 0..) |*sp, sp_index| {
            const t = self.scalp_triangles.items[sp.triangle_index];
            const v0 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[0]]);
            const v1 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[1]]);
            const v2 = Vec3.fromFloat3(self.scalp_vertices.items[t.idx[2]]);

            // Get tangent vector
            const tangent = v1.sub(v0).normalized();

            // Get normal of the triangle
            const normal = tangent.cross(v2.sub(v0)).normalized();

            // Calculate basis for the triangle
            const binormal = tangent.cross(normal);
            const triangle_basis = Mat44.init(Vec4.fromVec3W(normal, 0), Vec4.fromVec3W(binormal, 0), Vec4.fromVec3W(tangent, 0), Vec4.init(0, 0, 0, 1)).getQuaternion();

            // Calculate how to rotate from the triangle basis to the Bishop frame of the root
            const to_bishop = triangle_basis.conjugated().mul(Quat.fromFloat4(self.sim_vertices.items[self.sim_strands.items[sp_index].start_vtx].bishop));
            sp.to_bishop = to_bishop.compressUnitQuat();
        }

        // Calculate the grid size
        self.simulation_bounds = .empty;
        for (self.sim_vertices.items) |v|
            self.simulation_bounds.encapsulateVec3(Vec3.fromFloat3(v.position));
        self.simulation_bounds.expandBy(self.simulation_bounds_padding);

        // Prepare neutral density grid
        try resizeValue(f32, allocator, &self.neutral_density, self.grid_size.getX() *% self.grid_size.getY() *% self.grid_size.getZ(), 0.0);
        const sampler: GridSampler = .init(self);
        for (self.sim_vertices.items) |*v| {
            if (v.inv_mass > 0.0) {
                sampler.sample(Vec3.fromFloat3(v.position), struct {
                    density: []f32,
                    vertex: *const SVertex,

                    fn call(c: @This(), index: u32, fraction: f32) void {
                        // Zolt (R20 UB30): with a GridSize below 2 on X, Y or Z a grid point can be past the end of the
                        // grid, and Jolt writes past mNeutralDensity
                        if (index >= c.density.len)
                            @panic("HairSettings.init: a vertex samples a grid point past the end of the grid");
                        c.density[index] += fraction / c.vertex.inv_mass;
                    }
                }{ .density = self.neutral_density.items, .vertex = v });
            }
        }

        // Calculate density scale for drawing the grid
        self.density_scale = 0.0;
        for (self.neutral_density.items) |density|
            self.density_scale = math.max(self.density_scale, density);
        if (self.density_scale > 0.0)
            self.density_scale = 1.0 / self.density_scale;

        // Prepare render vertices
        for (self.render_vertices.items) |*v| {
            const render_pos = Vec3.fromFloat3(v.position);

            var total_weight: f32 = 0.0;
            for (&v.influences) |*inf| {
                if (inf.vertex_index != no_influence) {
                    const simulated_vertex = self.sim_vertices.items[inf.vertex_index];
                    const simulated_pos = Vec3.fromFloat3(simulated_vertex.position);
                    const local_position = Quat.fromFloat4(simulated_vertex.bishop).inverseRotate(render_pos.sub(simulated_pos));
                    var relative_position: Float3 = undefined;
                    local_position.storeFloat3(&relative_position);
                    inf.relative_position = .{ .x = relative_position.x, .y = relative_position.y, .z = relative_position.z };

                    // Weigh according to inverse distance to the simulated vertex
                    inf.weight = 1.0 / (local_position.length() + 1.0e-6);
                    total_weight += inf.weight;
                } else inf.weight = 0.0;
            }

            // Normalize weights
            if (total_weight > 0.0) {
                for (&v.influences) |*a| {
                    if (a.vertex_index != no_influence)
                        a.weight /= total_weight;
                }
            }

            // Order so that largest weight comes first
            quickSort(SVertexInfluence, &v.influences, {}, struct {
                fn lessThan(_: void, lhs: SVertexInfluence, rhs: SVertexInfluence) bool {
                    return lhs.weight > rhs.weight;
                }
            }.lessThan);
        }

        return max_dist_sq_hair_to_scalp;
    }

    /// Must be called after Init to setup the compute buffers
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts); some buffers may then be set.
    pub fn initCompute(self: *HairSettings, compute_system: *ComputeSystem) Allocator.Error!void {
        const allocator = self.allocator;
        const num_strands = self.sim_strands.items.len;

        // Zolt (R20 UB33): before Init mMaxVerticesPerStrand is 0, and Jolt writes past the transposed vertex arrays
        for (self.sim_strands.items) |strand| {
            if (strand.vertexCount() > self.max_vertices_per_strand)
                @panic("HairSettings.initCompute: a strand has more vertices than max_vertices_per_strand (call init first)");
        }

        // Optional: We can attach the roots of the hairs to the scalp
        if (self.scalp_triangles.items.len != 0 and self.skin_points.items.len != 0) {
            setBuffer(&self.scalp_triangles_cb, try compute_system.createComputeBuffer(.buffer, self.scalp_triangles.items.len * 3, @sizeOf(u32), std.mem.sliceAsBytes(self.scalp_triangles.items)));
            setBuffer(&self.skin_points_cb, try compute_system.createComputeBuffer(.buffer, self.skin_points.items.len, @sizeOf(SkinPoint), std.mem.sliceAsBytes(self.skin_points.items)));

            // We can skin the scalp or the skinned vertices can be provided externally
            if (self.scalp_vertices.items.len != 0 and self.scalp_inverse_bind_pose.items.len != 0 and self.scalp_skin_weights.items.len != 0) {
                setBuffer(&self.scalp_vertices_cb, try compute_system.createComputeBuffer(.buffer, self.scalp_vertices.items.len, @sizeOf(Float3), std.mem.sliceAsBytes(self.scalp_vertices.items)));
                setBuffer(&self.scalp_skin_weights_cb, try compute_system.createComputeBuffer(.buffer, self.scalp_skin_weights.items.len, @sizeOf(HairStructs.HairSkinWeight), std.mem.sliceAsBytes(self.scalp_skin_weights.items)));
            }
        }

        // Calculate the number of vertices for every strand
        const padded_strands = (num_strands + @sizeOf(u32) - 1) & ~@as(usize, @sizeOf(u32) - 1); // Make size multiple of sizeof(uint32)
        const strand_vertex_counts = try allocator.alloc(u8, padded_strands);
        defer allocator.free(strand_vertex_counts);
        @memset(strand_vertex_counts, 0);
        for (self.sim_strands.items, 0..) |strand, i| {
            const count = strand.vertexCount();
            if (Core.enable_asserts) std.debug.assert(count < 256);
            strand_vertex_counts[i] = @truncate(count);
        }

        // Calculate material index for every strand
        const strand_material_indices = try allocator.alloc(u8, padded_strands);
        defer allocator.free(strand_material_indices);
        @memset(strand_material_indices, 0);
        for (self.sim_strands.items, 0..) |strand, i| {
            const material_index = strand.material_index;
            if (Core.enable_asserts) std.debug.assert(material_index < 256);
            strand_material_indices[i] = @truncate(material_index);
        }

        // Create buffers that contain information about the rest pose of the hair
        // Rearrange vertices so that the first vertices of all strands are grouped together, then the second vertices, etc.
        // Zolt (R20 UB4): the padding vertices of short strands are zero, Jolt leaves them uninitialized
        const num_vertices: u32 = @truncate(@as(u64, self.max_vertices_per_strand) * num_strands);
        const vertices_position = try allocator.alloc(Float3, num_vertices);
        defer allocator.free(vertices_position);
        @memset(vertices_position, .{ .x = 0, .y = 0, .z = 0 });
        const vertices_bishop = try allocator.alloc(u32, num_vertices);
        defer allocator.free(vertices_bishop);
        @memset(vertices_bishop, 0);
        const vertices_omega0 = try allocator.alloc(u32, num_vertices);
        defer allocator.free(vertices_omega0);
        @memset(vertices_omega0, 0);
        const vertices_fixed = try allocator.alloc(u32, (num_vertices + 31) / 32);
        defer allocator.free(vertices_fixed);
        @memset(vertices_fixed, 0);
        const vertices_length = try allocator.alloc(f32, num_vertices);
        defer allocator.free(vertices_length);
        @memset(vertices_length, 0);
        const vertices_strand_fraction = try allocator.alloc(u32, (num_vertices + 3) / 4);
        defer allocator.free(vertices_strand_fraction);
        @memset(vertices_strand_fraction, 0);
        for (self.sim_strands.items, 0..) |strand, s| {
            var v: u32 = 0;
            const nv = strand.vertexCount();
            while (v < nv) : (v += 1) {
                const in_v = self.sim_vertices.items[strand.start_vtx + v];
                const idx: usize = v * num_strands + s;

                vertices_position[idx] = in_v.position;
                vertices_bishop[idx] = Vec4.loadFloat4(&in_v.bishop).compressUnitVector();
                vertices_omega0[idx] = Vec4.loadFloat4(&in_v.omega0).compressUnitVector();
                vertices_length[idx] = in_v.length;
                if (in_v.inv_mass <= 0.0)
                    vertices_fixed[idx >> 5] |= @as(u32, 1) << @intCast(idx & 31);
                vertices_strand_fraction[idx >> 2] |= truncToUint(in_v.strand_fraction * 255.0) << @intCast((idx & 3) << 3);
            }
        }

        // Calculate a map from render vertex to strand index
        const simulation_vertex_to_strand_idx = try allocator.alloc(u32, self.sim_vertices.items.len);
        defer allocator.free(simulation_vertex_to_strand_idx);
        @memset(simulation_vertex_to_strand_idx, ~@as(u32, 0));
        for (self.sim_strands.items, 0..) |strand, strand_idx| {
            var v = strand.start_vtx;
            while (v < strand.end_vtx) : (v += 1)
                simulation_vertex_to_strand_idx[v] = @intCast(strand_idx);
        }

        // Create buffer for simulated vertex influences
        const num_influences = self.render_vertices.items.len * HairStructs.hair_num_s_vertex_influences;
        const svertex_influences = try allocator.alloc(HairStructs.HairSVertexInfluence, num_influences);
        defer allocator.free(svertex_influences);
        for (self.render_vertices.items, 0..) |render_vertex, v| {
            for (0..HairStructs.hair_num_s_vertex_influences) |a| {
                const inf = &svertex_influences[v * HairStructs.hair_num_s_vertex_influences + a];
                inf.* = render_vertex.influences[a];

                // Remap vertex index to reflect the transposing of the position buffer
                if (inf.vertex_index != no_influence) {
                    const strand_idx = simulation_vertex_to_strand_idx[inf.vertex_index];
                    const start_vtx = self.sim_strands.items[strand_idx].start_vtx;
                    inf.vertex_index = strand_idx +% (inf.vertex_index -% start_vtx) *% @as(u32, @truncate(num_strands));
                } else {
                    // The shader doesn't check if weight is zero, it just takes the vertex. Make sure the index points to something.
                    inf.vertex_index = 0;
                }
            }
        }

        setBuffer(&self.vertices_position_cb, try compute_system.createComputeBuffer(.buffer, vertices_position.len, @sizeOf(Float3), std.mem.sliceAsBytes(vertices_position)));
        setBuffer(&self.vertices_bishop_cb, try compute_system.createComputeBuffer(.buffer, vertices_bishop.len, @sizeOf(u32), std.mem.sliceAsBytes(vertices_bishop)));
        setBuffer(&self.vertices_omega0_cb, try compute_system.createComputeBuffer(.buffer, vertices_omega0.len, @sizeOf(u32), std.mem.sliceAsBytes(vertices_omega0)));
        setBuffer(&self.vertices_length_cb, try compute_system.createComputeBuffer(.buffer, vertices_length.len, @sizeOf(f32), std.mem.sliceAsBytes(vertices_length)));
        setBuffer(&self.vertices_fixed_cb, try compute_system.createComputeBuffer(.buffer, vertices_fixed.len, @sizeOf(u32), std.mem.sliceAsBytes(vertices_fixed)));
        setBuffer(&self.vertices_strand_fraction_cb, try compute_system.createComputeBuffer(.buffer, vertices_strand_fraction.len, @sizeOf(u32), std.mem.sliceAsBytes(vertices_strand_fraction)));
        setBuffer(&self.strand_vertex_counts_cb, try compute_system.createComputeBuffer(.buffer, strand_vertex_counts.len / @sizeOf(u32), @sizeOf(u32), strand_vertex_counts));
        setBuffer(&self.strand_material_index_cb, try compute_system.createComputeBuffer(.buffer, strand_material_indices.len / @sizeOf(u32), @sizeOf(u32), strand_material_indices));
        setBuffer(&self.neutral_density_cb, try compute_system.createComputeBuffer(.buffer, self.neutral_density.items.len, @sizeOf(f32), std.mem.sliceAsBytes(self.neutral_density.items)));
        setBuffer(&self.s_vertex_influences_cb, try compute_system.createComputeBuffer(.buffer, num_influences, @sizeOf(HairStructs.HairSVertexInfluence), std.mem.sliceAsBytes(svertex_influences)));
    }

    /// `mFooCB = CreateComputeBuffer(...).Get()`: the new buffer replaces (and releases) the old one
    fn setBuffer(member: *Ref(ComputeBuffer), buffer: Ref(ComputeBuffer)) void {
        member.deinit();
        member.* = buffer;
    }

    /// Sample the neutral density at a grid position
    /// Zolt: the assert is recoverable (R8). Hair.draw's density loops let x pass GetX() (Jolt's quirk, R20), and Jolt's
    /// release build then reads another cell of the grid.
    pub fn getNeutralDensity(self: *const HairSettings, x: u32, y: u32, z: u32) f32 {
        if (Core.enable_asserts) Core.assertRecoverable(x < self.grid_size.getX() and y < self.grid_size.getY() and z < self.grid_size.getZ(), "inX < mGridSize.GetX() && inY < mGridSize.GetY() && inZ < mGridSize.GetZ()", @src());
        return self.neutral_density.items[x +% y *% self.grid_size.getX() +% z *% self.grid_size.getX() *% self.grid_size.getY()];
    }

    /// Get the number of vertices in the vertex buffers padded to a multiple of mMaxVerticesPerStrand.
    pub fn getNumVerticesPadded(self: *const HairSettings) u32 {
        return @as(u32, @truncate(self.sim_strands.items.len)) *% self.max_vertices_per_strand;
    }

    /// @brief Calculates the pose used for skinning the scalp
    /// @param joint_to_hair Transform to bring the model space joint matrices to the hair local space
    /// @param joint_matrices Model space joint matrices of the joints in the face
    /// @param out_joint_matrices Joint matrices combined with the inverse bind pose
    pub fn prepareForScalpSkinning(self: *const HairSettings, joint_to_hair: Mat44, joint_matrices: []const Mat44, out_joint_matrices: []Mat44) void {
        for (self.scalp_inverse_bind_pose.items, 0..) |inverse_bind_pose, i|
            out_joint_matrices[i] = joint_to_hair.mul(joint_matrices[i]).mul(inverse_bind_pose);
    }

    /// Skin the scalp mesh to the given joint matrices and output the skinned scalp vertices
    /// @param joint_to_hair Transform to bring the model space joint matrices to the hair local space
    /// @param joint_matrices Model space joint matrices of the joints in the face
    /// @param out_vertices Returns skinned vertices (owned by `allocator`, which also allocates the temporary joint matrices)
    pub fn skinScalpVertices(self: *const HairSettings, allocator: Allocator, joint_to_hair: Mat44, joint_matrices: []const Mat44, out_vertices: *std.ArrayList(Vec3)) Allocator.Error!void {
        try out_vertices.resize(allocator, self.scalp_vertices.items.len);

        // Pre transform all joint matrices
        const transformed = try allocator.alloc(Mat44, self.scalp_inverse_bind_pose.items.len);
        defer allocator.free(transformed);
        self.prepareForScalpSkinning(joint_to_hair, joint_matrices, transformed);

        // Skin all vertices
        const n = self.scalp_num_skin_weights_per_vertex;
        for (out_vertices.items, 0..) |*v, i| {
            v.* = Vec3.zero();
            const first: usize = @as(u32, @truncate(i)) *% n;
            for (self.scalp_skin_weights.items[first..][0..n]) |w| {
                if (w.weight > 0.0)
                    v.* = v.add(transformed[w.joint_idx].mulScalar(w.weight).mulVec3(Vec3.fromFloat3(self.scalp_vertices.items[i])));
            }
        }
    }

    /// Saves the state of this object in binary form to inStream. Doesn't store the compute buffers.
    pub fn saveBinaryState(self: *const HairSettings, stream: StreamOut) void {
        stream.writeArray(SVertex, self.sim_vertices.items);
        stream.writeArray(SStrand, self.sim_strands.items);
        stream.writeArray(RVertex, self.render_vertices.items);
        stream.writeArray(RStrand, self.render_strands.items);
        stream.writeArray(Float3, self.scalp_vertices.items);
        stream.writeArray(IndexedTriangleNoMaterial, self.scalp_triangles.items);
        stream.writeArray(Mat44, self.scalp_inverse_bind_pose.items);
        stream.writeArray(SkinWeight, self.scalp_skin_weights.items);
        stream.write(self.scalp_num_skin_weights_per_vertex);
        stream.write(self.num_iterations_per_second);
        stream.write(self.max_delta_time);
        stream.write(self.grid_size);
        stream.write(self.simulation_bounds_padding);
        stream.write(self.initial_gravity);
        stream.writeArrayWith(Material, self.materials.items, {}, struct {
            fn writeElement(_: void, element: *const Material, s: StreamOut) void {
                s.write(element.enable_collision);
                s.write(element.enable_lra);
                s.write(element.linear_damping);
                s.write(element.angular_damping);
                s.write(element.max_linear_velocity);
                s.write(element.max_angular_velocity);
                element.gravity_factor.saveBinaryState(s);
                s.write(element.friction);
                s.write(element.bend_compliance);
                s.write(element.bend_compliance_multiplier);
                s.write(element.stretch_compliance);
                s.write(element.inertia_multiplier);
                element.hair_radius.saveBinaryState(s);
                element.world_transform_influence.saveBinaryState(s);
                element.grid_velocity_factor.saveBinaryState(s);
                s.write(element.grid_density_force_factor);
                element.global_pose.saveBinaryState(s);
                element.skin_global_pose.saveBinaryState(s);
                s.write(element.simulation_strands_fraction);
                s.write(element.gravity_preload_factor);
            }
        }.writeElement);
        stream.writeArray(SkinPoint, self.skin_points.items);
        stream.write(self.simulation_bounds); // The raw 32 bytes of the AABox (W = Z, like Jolt's)
        stream.writeArray(f32, self.neutral_density.items);
        stream.write(self.density_scale);
        stream.write(self.max_vertices_per_strand);
    }

    /// Restore the state of this object from inStream.
    /// Zolt: an allocation failure returns `error.OutOfMemory`; the arrays may then hold part of the stream.
    pub fn restoreBinaryState(self: *HairSettings, stream: StreamIn) Allocator.Error!void {
        const allocator = self.allocator;
        try stream.readArray(SVertex, allocator, &self.sim_vertices);
        try stream.readArray(SStrand, allocator, &self.sim_strands);
        try stream.readArray(RVertex, allocator, &self.render_vertices);
        try stream.readArray(RStrand, allocator, &self.render_strands);
        try stream.readArray(Float3, allocator, &self.scalp_vertices);
        try stream.readArray(IndexedTriangleNoMaterial, allocator, &self.scalp_triangles);
        try stream.readArray(Mat44, allocator, &self.scalp_inverse_bind_pose);
        try stream.readArray(SkinWeight, allocator, &self.scalp_skin_weights);
        stream.read(&self.scalp_num_skin_weights_per_vertex);
        stream.read(&self.num_iterations_per_second);
        stream.read(&self.max_delta_time);
        stream.read(&self.grid_size);
        stream.read(&self.simulation_bounds_padding);
        stream.read(&self.initial_gravity);
        try stream.readArrayWith(Material, allocator, &self.materials, {}, struct {
            fn readElement(_: void, s: StreamIn, element: *Material) Allocator.Error!void {
                s.read(&element.enable_collision);
                s.read(&element.enable_lra);
                s.read(&element.linear_damping);
                s.read(&element.angular_damping);
                s.read(&element.max_linear_velocity);
                s.read(&element.max_angular_velocity);
                element.gravity_factor.restoreBinaryState(s);
                s.read(&element.friction);
                s.read(&element.bend_compliance);
                s.read(&element.bend_compliance_multiplier);
                s.read(&element.stretch_compliance);
                s.read(&element.inertia_multiplier);
                element.hair_radius.restoreBinaryState(s);
                element.world_transform_influence.restoreBinaryState(s);
                element.grid_velocity_factor.restoreBinaryState(s);
                s.read(&element.grid_density_force_factor);
                element.global_pose.restoreBinaryState(s);
                element.skin_global_pose.restoreBinaryState(s);
                s.read(&element.simulation_strands_fraction);
                s.read(&element.gravity_preload_factor);
            }
        }.readElement);
        try stream.readArray(SkinPoint, allocator, &self.skin_points);
        stream.read(&self.simulation_bounds);
        try stream.readArray(f32, allocator, &self.neutral_density);
        stream.read(&self.density_scale);
        stream.read(&self.max_vertices_per_strand);
    }
};

/// Jolt's `Array<T>::resize(n)` of a class with a default constructor: the elements that stay keep their values, new
/// elements are default constructed
fn resizeDefault(comptime T: type, allocator: Allocator, list: *std.ArrayList(T), new_len: usize) Allocator.Error!void {
    const old_len = list.items.len;
    try list.resize(allocator, new_len);
    if (new_len > old_len)
        @memset(list.items[old_len..], .{});
}

/// Jolt's `Array<T>::resize(n, value)`: the elements that stay keep their values, new elements are `value`
fn resizeValue(comptime T: type, allocator: Allocator, list: *std.ArrayList(T), new_len: usize, value: T) Allocator.Error!void {
    const old_len = list.items.len;
    try list.resize(allocator, new_len);
    if (new_len > old_len)
        @memset(list.items[old_len..], value);
}

/// `uint32(f)` of a float as Jolt's x86-64 build computes it (porting guide section 8, rule 12): a 64-bit cvttss2si of
/// which the low 32 bits are used. NaN and values outside the range of an int64 give 0.
fn truncToUint(value: f32) u32 {
    if (value >= -0x1p63 and value < 0x1p63) // False for NaN
        return @truncate(@as(u64, @bitCast(@as(i64, @intFromFloat(value)))));
    return 0;
}

// Tests ---------------------------------------------------------------------------------------------------------------

const testing = std.testing;
const IssueReporting = @import("../../Core/IssueReporting.zig");

/// A groom of `num_strands` straight strands of `num_vertices` vertices along -y, side by side in x, material
/// `material_index`, the first vertex of each strand fixed
fn testGroom(allocator: Allocator, num_strands: u32, num_vertices: u32, material_index: u32, vertices: *std.ArrayList(HairSettings.SVertex), strands: *std.ArrayList(HairSettings.SStrand)) Allocator.Error!void {
    for (0..num_strands) |s| {
        const first: u32 = @intCast(vertices.items.len);
        for (0..num_vertices) |i| {
            try vertices.append(allocator, .init(.{ .x = 0.01 * @as(f32, @floatFromInt(s)), .y = -0.02 * @as(f32, @floatFromInt(i)), .z = 0.001 * @as(f32, @floatFromInt(i * i)) }, .{ .inv_mass = if (i == 0) 0.0 else 1.0 }));
        }
        try strands.append(allocator, .init(first, first + num_vertices, material_index));
    }
}

test "HairSettings: defaults of Jolt's constructors and the serializable classes" {
    const settings = try HairSettings.create(testing.allocator);
    var ref: Ref(HairSettings) = .init(settings);
    defer ref.deinit();
    try testing.expectEqual(@as(u32, 360), settings.num_iterations_per_second);
    try testing.expectEqual(@as(f32, 1.0 / 30.0), settings.max_delta_time);
    try testing.expect(settings.grid_size.eql(UVec4.init(32, 32, 32, 0)));
    try testing.expect(settings.simulation_bounds.min.eql(Vec3.replicate(-1.0)) and settings.simulation_bounds.max.eql(Vec3.replicate(1.0)));

    // RTTI create: SVertexInfluence has the values of Jolt's constructor, RVertex 3 of them
    const influence: *HairSettings.SVertexInfluence = @ptrCast(@alignCast((try HairSettings.SVertexInfluence.rtti.createObject(testing.allocator)).?));
    defer HairSettings.SVertexInfluence.rtti.destructObject(influence, testing.allocator);
    try testing.expectEqual(HairSettings.no_influence, influence.vertex_index);
    try testing.expectEqual(@as(f32, 0.0), influence.weight);
    const vertex: HairSettings.RVertex = .{};
    for (vertex.influences) |inf| try testing.expectEqual(HairSettings.no_influence, inf.vertex_index);

    // The material and gradient defaults
    const material: HairSettings.Material = .{};
    try testing.expectEqual(@as(f32, 0.2), material.gravity_factor.min_fraction);
    try testing.expect(material.needsGrid());
    try testing.expect(!material.globalPoseOnly());
    try testing.expectEqual(@as(f32, 1.0e-7), material.getBendCompliance(0.0));
    try testing.expectEqual(@as(f32, 1.0e-7), material.getBendCompliance(1.0));
    try testing.expectEqual(@as(f32, 100.0 * 1.0e-7), material.getBendCompliance(0.5));

    // The names of the nested classes
    try testing.expectEqualStrings("HairSettings::SkinWeight", HairSettings.SkinWeight.rtti.name);
    try testing.expectEqualStrings("HairSettings::SStrand", HairSettings.SStrand.rtti.name);
    try testing.expect(HairSettings.SStrand.rtti.isKindOf(HairSettings.RStrand.rtti));
}

test "HairSettings: initRenderAndSimulationStrands simulates a fraction, appends to the simulation arrays (Jolt's quirk)" {
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .simulation_strands_fraction = 0.25 });

    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    try testGroom(allocator, 8, 5, 0, &vertices, &strands);

    try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    try testing.expectEqual(@as(usize, 2), settings.sim_strands.items.len); // ceil(0.25 * 8)
    try testing.expectEqual(@as(usize, 10), settings.sim_vertices.items.len);
    try testing.expectEqual(@as(usize, 8), settings.render_strands.items.len);
    try testing.expectEqual(@as(usize, 40), settings.render_vertices.items.len);
    for (settings.render_vertices.items) |v| try testing.expect(v.influences[0].vertex_index < 10);

    // A second call keeps the render arrays' size and appends to the simulation arrays
    try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    try testing.expectEqual(@as(usize, 4), settings.sim_strands.items.len);
    try testing.expectEqual(@as(usize, 20), settings.sim_vertices.items.len);
    try testing.expectEqual(@as(usize, 40), settings.render_vertices.items.len);
}

test "HairSettings: a simulation fraction of exactly 1 on the last material (next to R20 UB8)" {
    // A fraction that selects more strands than there are left from the start of its run panics (UB8: Jolt reads past
    // the index array). Exactly 1 simulates every strand, and a fraction above 1 whose strands stay inside the index
    // array runs into the next material (literal).
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .simulation_strands_fraction = 1.5 });
    try settings.materials.append(allocator, .{ .simulation_strands_fraction = 1.0 });

    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    try testGroom(allocator, 2, 4, 0, &vertices, &strands);
    try testGroom(allocator, 4, 4, 1, &vertices, &strands);

    try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    // Material 0 simulates ceil(1.5 * 2) = 3 strands (one of material 1), material 1 then simulates its 4
    try testing.expectEqual(@as(usize, 7), settings.sim_strands.items.len);
    var num_material1: usize = 0;
    for (settings.sim_strands.items) |s| num_material1 += s.material_index;
    try testing.expectEqual(@as(usize, 5), num_material1);
}

test "HairSettings: init calculates the strand fractions, the rods, the Bishop frames and the grid" {
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .simulation_strands_fraction = 1.0 });
    settings.grid_size = .init(4, 5, 6, 0);
    try testGroom(allocator, 3, 4, 0, &settings.sim_vertices, &settings.sim_strands);
    try settings.sim_strands.append(allocator, .init(12, 14, 0)); // A strand of 2 vertices
    try settings.sim_vertices.append(allocator, .init(.{ .x = 1, .y = 0, .z = 0 }, .{ .inv_mass = 0.0 }));
    try settings.sim_vertices.append(allocator, .init(.{ .x = 1, .y = 0.5, .z = 0 }, .{}));

    try testing.expectEqual(@as(f32, 0.0), try settings.init());
    try testing.expectEqual(@as(u32, 4), settings.max_vertices_per_strand);
    try testing.expectEqual(@as(usize, 4 * 5 * 6), settings.neutral_density.items.len);
    try testing.expect(settings.density_scale > 0.0);
    for (settings.sim_strands.items) |strand| {
        try testing.expectEqual(@as(f32, 0.0), settings.sim_vertices.items[strand.start_vtx].strand_fraction);
        try testing.expectEqual(@as(f32, 1.0), settings.sim_vertices.items[strand.end_vtx - 1].strand_fraction);
        var v = strand.start_vtx;
        while (v < strand.end_vtx - 1) : (v += 1) {
            const sv = settings.sim_vertices.items[v];
            try testing.expect(sv.length > 0.0);
            try testing.expect(Quat.fromFloat4(sv.bishop).isNormalized(.{}));
        }
    }

    // The bounds hold every vertex with the padding
    for (settings.sim_vertices.items) |v|
        try testing.expect(settings.simulation_bounds.containsVec3(Vec3.fromFloat3(v.position)));

    // The neutral density sums the masses of the vertices that are not fixed (1 each)
    var total: f32 = 0;
    for (settings.neutral_density.items) |d| total += d;
    try testing.expectApproxEqAbs(@as(f32, 10.0), total, 1.0e-4);
}

test "HairSettings: getNeutralDensity outside the grid follows Jolt's release build" {
    // Hair.draw's density loops let x run to GridSize.GetZ() (Jolt's quirk, R20). Jolt asserts on x >= GetX(), and its
    // release build reads another cell of the grid. The assert is recoverable (R8).
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    settings.grid_size = .init(2, 3, 4, 0);
    for (0..2 * 3 * 4) |i| try settings.neutral_density.append(allocator, @floatFromInt(i));
    try testing.expectEqual(@as(f32, 1 + 2 * 2 + 3 * 2 * 3), settings.getNeutralDensity(1, 2, 3)); // Inside the grid

    var expect_assert: IssueReporting.ExpectAssert = undefined;
    expect_assert.init(2);
    errdefer expect_assert.restore();
    try testing.expectEqual(@as(f32, 2 + 1 * 2 + 2 * 2 * 3), settings.getNeutralDensity(2, 1, 2)); // The cell (0, 2, 2)
    try testing.expectEqual(@as(f32, 3 + 2 * 2 + 1 * 2 * 3), settings.getNeutralDensity(3, 2, 1)); // The cell (1, 0, 2)
    try expect_assert.finish();
}

test "HairSettings: init of a strand of 2 vertices, the smallest one" {
    // A strand of 2 vertices is the smallest one that Jolt's debug build accepts. A strand of 0 vertices, or of 1 vertex
    // that is the first one, makes Jolt read past its arrays: Zolt panics.
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{});
    try testGroom(allocator, 1, 2, 0, &settings.sim_vertices, &settings.sim_strands);
    _ = try settings.init();
    try testing.expectEqual(@as(u32, 2), settings.max_vertices_per_strand);
}

test "HairSettings: init with a grid of 2 cells per axis (R20 UB30)" {
    // With 2 or more cells on each axis every grid point that a vertex samples is inside the grid. Every vertex with an
    // inv mass above 0 samples the 8 points around it.
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{});
    settings.grid_size = .init(2, 2, 2, 0);
    try testGroom(allocator, 3, 4, 0, &settings.sim_vertices, &settings.sim_strands);
    _ = try settings.init();
    try testing.expectEqual(@as(usize, 8), settings.neutral_density.items.len);
    var total: f32 = 0.0;
    for (settings.neutral_density.items) |d| total += d;
    try testing.expectApproxEqRel(@as(f32, 9.0), total, 1.0e-5); // 9 vertices of inv mass 1, each sample sums to 1
}

test "HairSettings: init with a grid of 1 cell on X whose grid points stay inside the grid (R20 UB30)" {
    // GridSize (1, 4, 4) has 16 points and the stride (1, 1, 4). On X the index is always 0, and the point at x + 1 is
    // the point at y + 1. A vertex at the indices (0, y, z) samples the points up to y + 4 * z + 6. That is past the
    // end only for y = 2 and z = 2: Init then panics, like Jolt writes past mNeutralDensity (not tested: a test cannot
    // catch a panic). These vertices reach point 15, the last one, and Jolt's Init reads inside the grid.
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{});
    settings.grid_size = .init(1, 4, 4, 0);
    settings.simulation_bounds_padding = .zero();
    // Two strands of 2 vertices. The fixed roots span the bounds (0, 0, 0) to (1, 4, 4), so the grid position of a
    // vertex is its position, clamped to (0, 3, 3).
    try settings.sim_vertices.appendSlice(allocator, &.{
        .init(.{ .x = 0, .y = 0, .z = 0 }, .{ .inv_mass = 0.0 }),
        .init(.{ .x = 0.5, .y = 1.5, .z = 3.0 }, .{}), // Indices (0, 1, 2), fraction (0, 0.5, 1): points 9 to 15
        .init(.{ .x = 1, .y = 4, .z = 4 }, .{ .inv_mass = 0.0 }),
        .init(.{ .x = 0.5, .y = 3.0, .z = 1.5 }, .{}), // Indices (0, 2, 1), fraction (0, 1, 0.5): points 6 to 12
    });
    try settings.sim_strands.appendSlice(allocator, &.{ .init(0, 2, 0), .init(2, 4, 0) });
    _ = try settings.init();
    try testing.expectEqualSlices(f32, &.{ 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0.5, 0, 0.5, 0.5, 0 }, settings.neutral_density.items);
    try testing.expectEqual(@as(f32, 2.0), settings.density_scale);
}

test "HairSettings: init of strands of 1 vertex after longer strands follows Jolt's release build" {
    // Jolt reads x[1] past the size of x (an assert). Its Array still holds the x[1] of the last strand of 2 or more
    // vertices. A strand of 1 vertex at vertex 0, or before every longer strand, panics (Jolt reads past its arrays).
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .gravity_preload_factor = 1.0, .bend_compliance = 1.0e-3, .stretch_compliance = 1.0e-4 });
    try testGroom(allocator, 1, 3, 0, &settings.sim_vertices, &settings.sim_strands); // Vertices 0 to 2
    try testGroom(allocator, 1, 5, 0, &settings.sim_vertices, &settings.sim_strands); // Vertices 3 to 7
    for (0..2) |i| {
        try settings.sim_vertices.append(allocator, .init(.{ .x = 1, .y = @floatFromInt(i), .z = 0 }, .{}));
        try settings.sim_strands.append(allocator, .init(@intCast(8 + i), @intCast(9 + i), 0));
    }

    var expect_assert: IssueReporting.ExpectAssert = undefined;
    expect_assert.init(2); // One per strand of 1 vertex
    errdefer expect_assert.restore();
    _ = try settings.init();
    try expect_assert.finish();
    try testing.expectEqual(@as(u32, 5), settings.max_vertices_per_strand);

    // The gravity preload makes x[1] depend on the strand: the rods of 1 vertex get the length of the strand of 5
    // vertices
    const v = settings.sim_vertices.items;
    try testing.expect(v[3].length != v[0].length);
    for ([_]usize{ 8, 9 }) |i| {
        try testing.expectEqual(v[3].length, v[i].length);
        try testing.expectEqual(@as(f32, 1.0), v[i].strand_fraction);
        try testing.expectEqual(v[i - 1].bishop, v[i].bishop); // Copied from the vertex before
    }
    try testing.expectEqual(v[6].bishop, v[9].bishop);
}

test "HairSettings: init with a scalp makes one skin point per simulated strand (next to the second init)" {
    // A second init with a scalp, or an init after restoring this state, would append more skin points: Jolt reads
    // mSimStrands past its end, Zolt panics. Restored settings without their skin points initialize again.
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{});
    try settings.scalp_vertices.appendSlice(allocator, &.{ .{ .x = -1, .y = 0, .z = -1 }, .{ .x = 1, .y = 0, .z = -1 }, .{ .x = 0, .y = 0, .z = 1 } });
    try settings.scalp_triangles.append(allocator, .init(0, 2, 1));
    try testGroom(allocator, 4, 3, 0, &settings.sim_vertices, &settings.sim_strands);
    _ = try settings.init();
    try testing.expectEqual(@as(usize, 4), settings.skin_points.items.len);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());

    var restored: HairSettings = .{ .allocator = allocator };
    defer restored.deinit();
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper = StreamWrapper.StreamInWrapper.init(&reader);
    try restored.restoreBinaryState(in_wrapper.streamIn());
    try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(settings.skin_points.items), std.mem.sliceAsBytes(restored.skin_points.items));

    restored.skin_points.clearRetainingCapacity();
    _ = try restored.init();
    try testing.expectEqual(@as(usize, 4), restored.skin_points.items.len);
}

test "HairSettings: resample keeps the ends of the strands" {
    const allocator = testing.allocator;
    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    try testGroom(allocator, 3, 7, 1, &vertices, &strands);
    const first = vertices.items[0];
    const last = vertices.items[6];

    try HairSettings.resample(allocator, &vertices, &strands, 4);
    try testing.expectEqual(@as(usize, 12), vertices.items.len);
    try testing.expectEqual(@as(usize, 3), strands.items.len);
    try testing.expectEqual(@as(u32, 4), strands.items[1].start_vtx);
    try testing.expectEqual(@as(u32, 1), strands.items[2].material_index);
    try testing.expectEqual(first.position, vertices.items[0].position);
    try testing.expectEqual(last.position, vertices.items[3].position);
    try testing.expectEqual(@as(f32, 0.0), vertices.items[0].inv_mass);
    try testing.expectEqual(@as(f32, 1.0), vertices.items[1].inv_mass);
}

test "HairSettings: resample of strands of 0 and 1 vertices inside the vertex array (literal)" {
    // Jolt reads the next vertex of the array with a fraction of 0. At the end of the array it would read past it:
    // Zolt panics there.
    const allocator = testing.allocator;
    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    try testGroom(allocator, 1, 4, 0, &vertices, &strands); // Vertices 0 to 3
    try vertices.append(allocator, .init(.{ .x = 1, .y = 2, .z = 3 }, .{ .inv_mass = 0.0 }));
    try strands.append(allocator, .init(4, 5, 0)); // 1 vertex
    try strands.append(allocator, .init(5, 5, 0)); // 0 vertices
    try testGroom(allocator, 1, 3, 0, &vertices, &strands); // Vertices 5 to 7
    const in = try allocator.dupe(HairSettings.SVertex, vertices.items);
    defer allocator.free(in);

    try HairSettings.resample(allocator, &vertices, &strands, 3);
    try testing.expectEqual(@as(usize, 12), vertices.items.len);
    const out = vertices.items;
    // The strand of 1 vertex: 3 times its vertex
    for (out[3..6]) |v| try testing.expectEqual(in[4].position, v.position);
    // The strand of 0 vertices: the vertex after it twice, then the vertex before it
    try testing.expectEqual(in[5].position, out[6].position);
    try testing.expectEqual(in[5].position, out[7].position);
    try testing.expectEqual(in[4].position, out[8].position);
    try testing.expectEqual(@as(u32, 6), strands.items[2].start_vtx);
}

test "HairSettings: saveBinaryState / restoreBinaryState round trip" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .friction = 0.7, .global_pose = .init(0.5, 0.25, .{}) });
    var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
    defer vertices.deinit(allocator);
    var strands: std.ArrayList(HairSettings.SStrand) = .empty;
    defer strands.deinit(allocator);
    try testGroom(allocator, 6, 5, 0, &vertices, &strands);
    try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    _ = try settings.init();

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());

    var restored: HairSettings = .{ .allocator = allocator };
    defer restored.deinit();
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper = StreamWrapper.StreamInWrapper.init(&reader);
    try restored.restoreBinaryState(in_wrapper.streamIn());
    try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(settings.render_vertices.items), std.mem.sliceAsBytes(restored.render_vertices.items));
    try testing.expectEqual(@as(f32, 0.7), restored.materials.items[0].friction);
    try testing.expectEqual(settings.simulation_bounds, restored.simulation_bounds);

    var again: std.Io.Writer.Allocating = .init(allocator);
    defer again.deinit();
    var again_wrapper = StreamWrapper.StreamOutWrapper.init(&again.writer);
    restored.saveBinaryState(again_wrapper.streamOut());
    try testing.expectEqualSlices(u8, out.written(), again.written());
}

test "HairSettings: initCompute transposes the vertices, the padding vertices are zero (R20 UB4)" {
    const allocator = testing.allocator;
    const ComputeSystemCPU = @import("../../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.materials.append(allocator, .{ .simulation_strands_fraction = 1.0 });
    try testGroom(allocator, 1, 5, 0, &settings.sim_vertices, &settings.sim_strands);
    try testGroom(allocator, 1, 3, 0, &settings.sim_vertices, &settings.sim_strands);
    settings.sim_strands.items[1] = .init(5, 8, 0);
    _ = try settings.init();

    var system = try ComputeSystemCPU.create(allocator);
    defer system.deinit();
    try settings.initCompute(system.get().?);
    try testing.expect(settings.scalp_triangles_cb.get() == null and settings.skin_points_cb.get() == null);

    // 2 strands of at most 5 vertices: 10 vertices, vertex v of strand s at v * 2 + s
    const positions = settings.vertices_position_cb.get().?;
    try testing.expectEqual(@as(u64, 10), positions.getSize());
    const p = positions.mapAs(Float3, .read);
    try testing.expectEqual(settings.sim_vertices.items[5 + 2].position, p[2 * 2 + 1]);
    for ([_]usize{ 7, 9 }) |padding| try testing.expectEqual(Float3{ .x = 0, .y = 0, .z = 0 }, p[padding]);
    positions.unmap();

    // The first vertices of both strands are fixed
    const fixed = settings.vertices_fixed_cb.get().?.mapAs(u32, .read);
    try testing.expectEqual(@as(u32, 0b11), fixed[0]);
    settings.vertices_fixed_cb.get().?.unmap();

    // Vertex counts as bytes, padded to a multiple of 4
    const counts = settings.strand_vertex_counts_cb.get().?;
    try testing.expectEqual(@as(u64, 1), counts.getSize());
    const c = counts.map(.read);
    try testing.expectEqualSlices(u8, &.{ 5, 3, 0, 0 }, c);
    counts.unmap();
}

test "HairSettings: out of memory in initRenderAndSimulationStrands, init, initCompute and the binary state" {
    const ComputeSystemCPU = @import("../../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");
    try testing.checkAllAllocationFailures(testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            const settings = try HairSettings.create(allocator);
            var ref: Ref(HairSettings) = .init(settings);
            defer ref.deinit();
            try settings.materials.append(allocator, .{ .simulation_strands_fraction = 0.5, .gravity_preload_factor = 1.0 });
            settings.grid_size = .init(4, 4, 4, 0);
            try settings.scalp_vertices.appendSlice(allocator, &.{ .{ .x = -1, .y = 0, .z = -1 }, .{ .x = 1, .y = 0, .z = -1 }, .{ .x = 0, .y = 0, .z = 1 } });
            try settings.scalp_triangles.append(allocator, .init(0, 2, 1));
            var vertices: std.ArrayList(HairSettings.SVertex) = .empty;
            defer vertices.deinit(allocator);
            var strands: std.ArrayList(HairSettings.SStrand) = .empty;
            defer strands.deinit(allocator);
            try testGroom(allocator, 4, 4, 0, &vertices, &strands);
            try HairSettings.resample(allocator, &vertices, &strands, 5);
            try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
            _ = try settings.init();

            var system = try ComputeSystemCPU.create(allocator);
            defer system.deinit();
            try settings.initCompute(system.get().?);

            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var out_wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
            settings.saveBinaryState(out_wrapper.streamOut());
            if (out_wrapper.isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
            var restored: HairSettings = .{ .allocator = allocator };
            defer restored.deinit();
            var reader: std.Io.Reader = .fixed(out.written());
            var in_wrapper = StreamWrapper.StreamInWrapper.init(&reader);
            try restored.restoreBinaryState(in_wrapper.streamIn());
        }
    }.f, .{});
}

test "HairSettings: skinScalpVertices and prepareForScalpSkinning" {
    const allocator = testing.allocator;
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    try settings.scalp_vertices.appendSlice(allocator, &.{ .{ .x = 1, .y = 0, .z = 0 }, .{ .x = 0, .y = 2, .z = 0 } });
    try settings.scalp_inverse_bind_pose.appendSlice(allocator, &.{ Mat44.identity(), Mat44.translation(Vec3.init(0, -1, 0)) });
    settings.scalp_num_skin_weights_per_vertex = 2;
    try settings.scalp_skin_weights.appendSlice(allocator, &.{ .{ .joint_idx = 0, .weight = 1.0 }, .{ .joint_idx = 1, .weight = 0.0 }, .{ .joint_idx = 0, .weight = 0.5 }, .{ .joint_idx = 1, .weight = 0.5 } });
    const joints = [_]Mat44{ Mat44.translation(Vec3.init(10, 0, 0)), Mat44.identity() };

    var out: std.ArrayList(Vec3) = .empty;
    defer out.deinit(allocator);
    try settings.skinScalpVertices(allocator, Mat44.identity(), &joints, &out);
    try testing.expect(out.items[0].eql(Vec3.init(11, 0, 0)));
    // 0.5 * (10, 2, 0) + 0.5 * (0, 1, 0)
    try testing.expect(out.items[1].eql(Vec3.init(5, 1.5, 0)));

    var prepared: [2]Mat44 = undefined;
    settings.prepareForScalpSkinning(Mat44.translation(Vec3.init(0, 0, 1)), &joints, &prepared);
    try testing.expect(prepared[1].getTranslation().eql(Vec3.init(0, -1, 1)));
}
