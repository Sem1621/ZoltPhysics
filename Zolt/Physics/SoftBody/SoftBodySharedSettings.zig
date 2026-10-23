//! Port of: Jolt/Physics/SoftBody/SoftBodySharedSettings.h, Jolt/Physics/SoftBody/SoftBodySharedSettings.cpp
//! Status: complete
//!
//! RemainingFeaturesArchitecture.md R15, R19:
//! - A RefTarget without virtual functions, like PhysicsScene. It keeps the allocator that owns it and its arrays:
//!   `create(allocator)` is `new SoftBodySharedSettings` (reference count 0, put it in a `Ref` / `RefConst`),
//!   `init(allocator)` / `deinit` settings on the stack or embedded (`setEmbedded` before references are taken). Both
//!   allocate: `materials` starts with one element, `PhysicsMaterial.default` (Jolt: `mMaterials { sDefault }`).
//! - The nested structs that SaveBinaryState writes as raw bytes (Vertex, Face, Edge, DihedralBend, Volume, Skinned,
//!   LRA, UpdateGroup, and SkinWeight inside Skinned) are extern structs with Jolt's sizes, checked at compile time.
//!   Members that Jolt leaves uninitialized (`mVertex[]`, the end indices of UpdateGroup) have no default value. The
//!   structs that SaveBinaryState writes field by field (InvBind, RodStretchShear, RodBendTwist) are plain structs.
//! - Private members of Jolt (`mClosestKinematic`, `mUpdateGroups`, `mSkinnedConstraintNormals`) are fields with the
//!   doc comment "private in Jolt". They are not serialized.
//! - `Clone()` is `clone(allocator)`: it returns a `Ref` that holds the only reference, like Jolt's `Ref`.
//! - `AddFace` returns `Allocator.Error`. Jolt's constructors of the nested structs with default arguments are `init`
//!   functions with an options struct (`Face.init(v1, v2, v3, .{ .material_index = 1 })`).
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL: `rtti` of the class and of its 11
//!   nested classes, with Jolt's names (`"SoftBodySharedSettings::Vertex"`). `mVertex`, `mRod` and `mWeights` are fixed
//!   arrays.
//!
//! RemainingFeaturesArchitecture.md R20, the algorithms (group S1). They are Jolt's, statement by statement, with Jolt's
//! quirks:
//! - CreateConstraints adds the last shared edge of a closed mesh twice: when the equal keys run to the end of the
//!   sorted list, the inner loop ends without `break`, so the outer loop processes the duplicate again.
//! - The edge sort, the group size sort and the sorts in Optimize use Zolt's `quickSort`. Equal keys keep the tie order
//!   of Jolt's QuickSort. Dijkstra in CalculateClosestKinematic uses `binaryHeapPush` / `binaryHeapPop`, so ties
//!   between kinematic vertices are broken like Jolt's heap.
//! - CalculateClosestKinematic caches its result. Optimize uses the cache that CreateConstraints filled, even when
//!   constraints were added after, and frees it at the end.
//! - Zolt: the functions that allocate return `Allocator.Error` (Jolt aborts). After a failure Optimize and
//!   CalculateClosestKinematic leave the settings as they were. CreateConstraints, CalculateRodProperties and
//!   CalculateSkinnedConstraintNormals may leave part of their result: call them again.
//! - `sCreateCube(inGridSize, inGridSpacing)` is `createCube(allocator, grid_size, grid_spacing)`.
//! - Asserts on the input (two vertices at the same position, a rod of zero length, constraints that use a vertex
//!   twice) are `if (Core.enable_asserts)` asserts: Jolt's release build continues there.
//! - Inputs where Jolt is undefined (R20, settled in S1, implemented in S2). Zolt-only tests pin the inputs next to
//!   them:
//!   - UB12: CreateConstraints with an LRA type, or Optimize, after vertices were added or removed while the closest
//!     kinematic cache was filled. The cache then has another size than `vertices`, and Jolt reads past the end of one
//!     of them. Zolt panics ("vertices were added or removed after the closest kinematic vertices were cached") at the
//!     LRA block of CreateConstraints and before the sorts of Optimize.
//!   - UB13: CreateConstraints with an empty `vertex_attributes` and at least one face: Jolt's `attr` reads
//!     `inVertexAttributes[min(v, 0u - 1)]`. Zolt panics ("no vertex attributes").
//!   - UB14: Optimize with a vertex that no group reaches: its position is NaN, +inf or FLT_MAX, so it is never a start
//!     vertex, and no open group connects to it. Jolt asserts. Its release build puts a constraint with only some
//!     vertices in a group into the non-parallel group: Zolt does the same, after an `if (Core.enable_asserts)` assert.
//!     Jolt's release build reads `groups[-1]` for a constraint with no vertex in a group and for a skinned constraint
//!     on that vertex: Zolt panics with a message in every build (the assert does not come first).
//!   - UB29: createCube with a grid size of 0. Jolt's `inGridSize - 1` wraps, and its volume constraint loops run about
//!     2^96 times until memory runs out. Zolt panics ("SoftBodySharedSettings.createCube: the grid size is 0"). No test
//!     pins it: a test cannot catch a panic. A grid size of 1 is literal: one vertex and no constraints.
//!
//! RemainingFeaturesArchitecture.md R19, the binary state with materials (group S2): `saveWithMaterials` /
//! `restoreWithMaterials` (Jolt's SaveWithMaterials / sRestoreWithMaterials) write and read Jolt's bytes with the ID maps
//! of StreamUtils. An ID at or above the size of the map creates new settings, like Jolt (no check). Jolt's error texts
//! are Result values; allocation failure is `error.OutOfMemory`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const RefCount = ReferenceFile.RefCount;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StreamUtils = @import("../../Core/StreamUtils.zig");
const Result = @import("../../Core/Result.zig").Result;
const ObjectToIDMap = @import("../../Core/ObjectToIDMap.zig").ObjectToIDMap;
const IDToObjectMap = @import("../../Core/ObjectToIDMap.zig").IDToObjectMap;
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const BinaryHeap = @import("../../Core/BinaryHeap.zig");
const binaryHeapPush = BinaryHeap.binaryHeapPush;
const binaryHeapPop = BinaryHeap.binaryHeapPop;
const UnorderedSet = @import("../../Core/UnorderedSet.zig").UnorderedSet;
const UnorderedMap = @import("../../Core/UnorderedMap.zig").UnorderedMap;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const SoftBodyUpdateContext = @import("SoftBodyUpdateContext.zig").SoftBodyUpdateContext;
const PhysicsMaterialFile = @import("../Collision/PhysicsMaterial.zig");
const PhysicsMaterial = PhysicsMaterialFile.PhysicsMaterial;
const PhysicsMaterialList = PhysicsMaterialFile.PhysicsMaterialList;

/// This class defines the setup of all particles and their constraints.
/// It is used during the simulation and can be shared between multiple soft bodies.
pub const SoftBodySharedSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SoftBodySharedSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mVertices", .vertices),
            .attr("mFaces", .faces),
            .attr("mEdgeConstraints", .edge_constraints),
            .attr("mDihedralBendConstraints", .dihedral_bend_constraints),
            .attr("mVolumeConstraints", .volume_constraints),
            .attr("mSkinnedConstraints", .skinned_constraints),
            .attr("mInvBindMatrices", .inv_bind_matrices),
            .attr("mLRAConstraints", .lra_constraints),
            .attr("mRodStretchShearConstraints", .rod_stretch_shear_constraints),
            .attr("mRodBendTwistConstraints", .rod_bend_twist_constraints),
            .attr("mMaterials", .materials),
        },
    });

    /// Which type of bend constraint should be created
    pub const BendType = enum(i32) {
        /// No bend constraints will be created
        none,
        /// A simple distance constraint
        distance,
        /// A dihedral bend constraint (most expensive, but also supports triangles that are initially not in the same plane)
        dihedral,
    };

    /// The type of long range attachment constraint to create
    pub const LRAType = enum(i32) {
        /// Don't create a LRA constraint
        none,
        /// Create a LRA constraint based on Euclidean distance between the closest kinematic vertex and this vertex
        euclidean_distance,
        /// Create a LRA constraint based on the geodesic distance between the closest kinematic vertex and this vertex (follows the edge constraints)
        geodesic_distance,
    };

    /// Per vertex attributes used during the CreateConstraints function.
    /// For an edge or shear constraint, the compliance is averaged between the two attached vertices.
    /// For a bend constraint, the compliance is averaged between the two vertices on the shared edge.
    pub const VertexAttributes = struct {
        /// The compliance of the normal edges. Set to FLT_MAX to disable regular edges for any edge involving this vertex.
        compliance: f32 = 0.0,
        /// The compliance of the shear edges. Set to FLT_MAX to disable shear edges for any edge involving this vertex.
        shear_compliance: f32 = 0.0,
        /// The compliance of the bend edges. Set to FLT_MAX to disable bend edges for any bend constraint involving this vertex.
        bend_compliance: f32 = math.flt_max,
        /// The type of long range attachment constraint to create.
        lra_type: LRAType = .none,
        /// Multiplier for the max distance of the LRA constraint, e.g. 1.01 means the max distance is 1% longer than the calculated distance in the rest pose.
        lra_max_distance_multiplier: f32 = 1.0,

        /// Constructor VertexAttributes(inCompliance, inShearCompliance, inBendCompliance, inLRAType = None, inLRAMaxDistanceMultiplier = 1)
        pub fn init(compliance: f32, shear_compliance: f32, bend_compliance: f32, opts: struct { lra_type: LRAType = .none, lra_max_distance_multiplier: f32 = 1.0 }) VertexAttributes {
            return .{ .compliance = compliance, .shear_compliance = shear_compliance, .bend_compliance = bend_compliance, .lra_type = opts.lra_type, .lra_max_distance_multiplier = opts.lra_max_distance_multiplier };
        }
    };

    /// Information about the optimization of the soft body, the indices of certain elements may have changed.
    pub const OptimizationResults = struct {
        /// Maps old edge index to new edge index
        edge_remap: std.ArrayList(u32) = .empty,
        /// Maps old LRA index to new LRA index
        lra_remap: std.ArrayList(u32) = .empty,
        /// Maps old rod stretch shear constraint index to new stretch shear rod constraint index
        rod_stretch_shear_constraint_remap: std.ArrayList(u32) = .empty,
        /// Maps old rod bend twist constraint index to new bend twist rod constraint index
        rod_bend_twist_constraint_remap: std.ArrayList(u32) = .empty,
        /// Maps old dihedral bend index to new dihedral bend index
        dihedral_bend_remap: std.ArrayList(u32) = .empty,
        /// Maps old volume constraint index to new volume constraint index
        volume_remap: std.ArrayList(u32) = .empty,
        /// Maps old skinned constraint index to new skinned constraint index
        skinned_remap: std.ArrayList(u32) = .empty,

        /// Destructor: frees the arrays (allocated with the allocator of the settings that were optimized)
        pub fn deinit(self: *OptimizationResults, allocator: Allocator) void {
            self.edge_remap.deinit(allocator);
            self.lra_remap.deinit(allocator);
            self.rod_stretch_shear_constraint_remap.deinit(allocator);
            self.rod_bend_twist_constraint_remap.deinit(allocator);
            self.dihedral_bend_remap.deinit(allocator);
            self.volume_remap.deinit(allocator);
            self.skinned_remap.deinit(allocator);
        }
    };

    /// A vertex is a particle, the data in this structure is only used during creation of the soft body and not during simulation
    pub const Vertex = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Vertex)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::Vertex",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mPosition", .position),
                .attr("mVelocity", .velocity),
                .attr("mInvMass", .inv_mass),
            },
        });

        /// Initial position of the vertex
        position: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Initial velocity of the vertex
        velocity: Float3 = .{ .x = 0, .y = 0, .z = 0 },
        /// Initial inverse of the mass of the vertex
        inv_mass: f32 = 1.0,

        comptime {
            std.debug.assert(@sizeOf(Vertex) == 28);
        }

        /// Constructor Vertex(inPosition, inVelocity = Float3(0, 0, 0), inInvMass = 1)
        pub fn init(position: Float3, opts: struct { velocity: Float3 = .{ .x = 0, .y = 0, .z = 0 }, inv_mass: f32 = 1.0 }) Vertex {
            return .{ .position = position, .velocity = opts.velocity, .inv_mass = opts.inv_mass };
        }
    };

    /// A face defines the surface of the body
    pub const Face = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Face)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::Face",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mMaterialIndex", .material_index),
            },
        });

        /// Indices of the vertices that form the face
        vertex: [3]u32,
        /// Index of the material of the face in SoftBodySharedSettings::mMaterials
        material_index: u32 = 0,

        comptime {
            std.debug.assert(@sizeOf(Face) == 16);
        }

        /// Constructor Face(inVertex1, inVertex2, inVertex3, inMaterialIndex = 0)
        pub fn init(vertex1: u32, vertex2: u32, vertex3: u32, opts: struct { material_index: u32 = 0 }) Face {
            return .{ .vertex = .{ vertex1, vertex2, vertex3 }, .material_index = opts.material_index };
        }

        /// Check if this is a degenerate face (a face which points to the same vertex twice)
        pub fn isDegenerate(self: Face) bool {
            return self.vertex[0] == self.vertex[1] or self.vertex[0] == self.vertex[2] or self.vertex[1] == self.vertex[2];
        }
    };

    /// An edge keeps two vertices at a constant distance using a spring: |x1 - x2| = rest length
    pub const Edge = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Edge)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::Edge",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mRestLength", .rest_length),
                .attr("mCompliance", .compliance),
            },
        });

        /// Indices of the vertices that form the edge
        vertex: [2]u32,
        /// Rest length of the spring, calculated by CalculateEdgeLengths
        rest_length: f32 = 1.0,
        /// Inverse of the stiffness of the spring
        compliance: f32 = 0.0,

        comptime {
            std.debug.assert(@sizeOf(Edge) == 16);
        }

        /// Constructor Edge(inVertex1, inVertex2, inCompliance = 0)
        pub fn init(vertex1: u32, vertex2: u32, opts: struct { compliance: f32 = 0.0 }) Edge {
            return .{ .vertex = .{ vertex1, vertex2 }, .compliance = opts.compliance };
        }

        /// Return the lowest vertex index of this constraint
        pub fn getMinVertexIndex(self: Edge) u32 {
            return @min(self.vertex[0], self.vertex[1]);
        }
    };

    /// A dihedral bend constraint keeps the angle between two triangles constant along their shared edge.
    ///
    ///        x2
    ///       /  \
    ///      / t0 \
    ///     x0----x1
    ///      \ t1 /
    ///       \  /
    ///        x3
    ///
    /// x0..x3 are the vertices, t0 and t1 are the triangles that share the edge x0..x1
    ///
    /// Based on:
    /// - "Position Based Dynamics" - Matthias Muller et al.
    /// - "Strain Based Dynamics" - Matthias Muller et al.
    /// - "Simulation of Clothing with Folds and Wrinkles" - R. Bridson et al.
    pub const DihedralBend = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::DihedralBend)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::DihedralBend",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mCompliance", .compliance),
                .attr("mInitialAngle", .initial_angle),
            },
        });

        /// Indices of the vertices of the 2 triangles that share an edge (the first 2 vertices are the shared edge)
        vertex: [4]u32,
        /// Inverse of the stiffness of the constraint
        compliance: f32 = 0.0,
        /// Initial angle between the normals of the triangles (pi - dihedral angle), calculated by CalculateBendConstraintConstants
        initial_angle: f32 = 0.0,

        comptime {
            std.debug.assert(@sizeOf(DihedralBend) == 24);
        }

        /// Constructor DihedralBend(inVertex1, inVertex2, inVertex3, inVertex4, inCompliance = 0)
        pub fn init(vertex1: u32, vertex2: u32, vertex3: u32, vertex4: u32, opts: struct { compliance: f32 = 0.0 }) DihedralBend {
            return .{ .vertex = .{ vertex1, vertex2, vertex3, vertex4 }, .compliance = opts.compliance };
        }

        /// Return the lowest vertex index of this constraint
        pub fn getMinVertexIndex(self: DihedralBend) u32 {
            return @min(@min(self.vertex[0], self.vertex[1]), @min(self.vertex[2], self.vertex[3]));
        }
    };

    /// Volume constraint, keeps the volume of a tetrahedron constant
    pub const Volume = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Volume)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::Volume",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mSixRestVolume", .six_rest_volume),
                .attr("mCompliance", .compliance),
            },
        });

        /// Indices of the vertices that form the tetrahedron
        vertex: [4]u32,
        /// 6 times the rest volume of the tetrahedron, calculated by CalculateVolumeConstraintVolumes
        six_rest_volume: f32 = 1.0,
        /// Inverse of the stiffness of the constraint
        compliance: f32 = 0.0,

        comptime {
            std.debug.assert(@sizeOf(Volume) == 24);
        }

        /// Constructor Volume(inVertex1, inVertex2, inVertex3, inVertex4, inCompliance = 0)
        pub fn init(vertex1: u32, vertex2: u32, vertex3: u32, vertex4: u32, opts: struct { compliance: f32 = 0.0 }) Volume {
            return .{ .vertex = .{ vertex1, vertex2, vertex3, vertex4 }, .compliance = opts.compliance };
        }

        /// Return the lowest vertex index of this constraint
        pub fn getMinVertexIndex(self: Volume) u32 {
            return @min(@min(self.vertex[0], self.vertex[1]), @min(self.vertex[2], self.vertex[3]));
        }
    };

    /// An inverse bind matrix take a skinned vertex from its bind pose into joint local space
    pub const InvBind = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::InvBind)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::InvBind",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mJointIndex", .joint_index),
                .attr("mInvBind", .inv_bind),
            },
        });

        /// Joint index to which this is attached
        joint_index: u32 = 0,
        /// The inverse bind matrix, this takes a vertex in its bind pose (Vertex::mPosition) to joint local space
        inv_bind: Mat44 = Mat44.identity(),

        /// Constructor InvBind(inJointIndex, inInvBind)
        pub fn init(joint_index: u32, inv_bind: Mat44) InvBind {
            return .{ .joint_index = joint_index, .inv_bind = inv_bind };
        }
    };

    /// A joint and its skin weight
    pub const SkinWeight = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::SkinWeight)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::SkinWeight",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mInvBindIndex", .inv_bind_index),
                .attr("mWeight", .weight),
            },
        });

        /// Index in mInvBindMatrices
        inv_bind_index: u32 = 0,
        /// Weight with which it is skinned
        weight: f32 = 0.0,

        /// Constructor SkinWeight(inInvBindIndex, inWeight)
        pub fn init(inv_bind_index: u32, weight: f32) SkinWeight {
            return .{ .inv_bind_index = inv_bind_index, .weight = weight };
        }
    };

    /// A constraint that skins a vertex to joints and limits the distance that the simulated vertex can travel from this vertex
    pub const Skinned = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Skinned)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::Skinned",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mWeights", .weights),
                .attr("mMaxDistance", .max_distance),
                .attr("mBackStopDistance", .back_stop_distance),
                .attr("mBackStopRadius", .back_stop_radius),
            },
        });

        /// Maximum number of skin weights
        pub const max_skin_weights: u32 = 4;

        /// Index in mVertices which indicates which vertex is being skinned
        vertex: u32 = 0,
        /// Skin weights, the bind pose of the vertex is assumed to be stored in Vertex::mPosition. The first weight that is zero indicates the end of the list. Weights should add up to 1.
        weights: [max_skin_weights]SkinWeight = @splat(.{}),
        /// Maximum distance that this vertex can reach from the skinned vertex, disabled when FLT_MAX. 0 when you want to hard skin the vertex to the skinned vertex.
        max_distance: f32 = math.flt_max,
        /// Disabled if mBackStopDistance >= mMaxDistance. The faces surrounding mVertex determine an average normal. mBackStopDistance behind the vertex in the opposite direction of this normal, the back stop sphere starts. The simulated vertex will be pushed out of this sphere and it can be used to approximate the volume of the skinned mesh behind the skinned vertex.
        back_stop_distance: f32 = math.flt_max,
        /// Radius of the backstop sphere. By default this is a fairly large radius so the sphere approximates a plane.
        back_stop_radius: f32 = 40.0,
        /// Information needed to calculate the normal of this vertex, lowest 24 bit is start index in mSkinnedConstraintNormals, highest 8 bit is number of faces (generated by CalculateSkinnedConstraintNormals)
        normal_info: u32 = 0,

        comptime {
            std.debug.assert(@sizeOf(Skinned) == 52);
        }

        /// Constructor Skinned(inVertex, inMaxDistance, inBackStopDistance, inBackStopRadius)
        pub fn init(vertex: u32, max_distance: f32, back_stop_distance: f32, back_stop_radius: f32) Skinned {
            return .{ .vertex = vertex, .max_distance = max_distance, .back_stop_distance = back_stop_distance, .back_stop_radius = back_stop_radius };
        }

        /// Normalize the weights so that they add up to 1
        pub fn normalizeWeights(self: *Skinned) void {
            // Get the total weight
            var total: f32 = 0.0;
            for (self.weights) |w|
                total += w.weight;

            // Normalize
            if (total > 0.0) {
                for (&self.weights) |*w|
                    w.weight /= total;
            }
        }
    };

    /// A long range attachment constraint, this is a constraint that sets a max distance between a kinematic vertex and a dynamic vertex
    /// See: "Long Range Attachments - A Method to Simulate Inextensible Clothing in Computer Games", Tae-Yong Kim, Nuttapong Chentanez and Matthias Mueller-Fischer
    pub const LRA = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::LRA)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::LRA",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mMaxDistance", .max_distance),
            },
        });

        /// The vertices that are connected. The first vertex should be kinematic, the 2nd dynamic.
        vertex: [2]u32,
        /// The maximum distance between the vertices, calculated by CalculateLRALengths
        max_distance: f32 = 0.0,

        comptime {
            std.debug.assert(@sizeOf(LRA) == 12);
        }

        /// Constructor LRA(inVertex1, inVertex2, inMaxDistance)
        pub fn init(vertex1: u32, vertex2: u32, max_distance: f32) LRA {
            return .{ .vertex = .{ vertex1, vertex2 }, .max_distance = max_distance };
        }

        /// Return the lowest vertex index of this constraint
        pub fn getMinVertexIndex(self: LRA) u32 {
            return @min(self.vertex[0], self.vertex[1]);
        }
    };

    /// A discrete Cosserat rod connects two particles with a rigid rod that has fixed length and inertia.
    /// A rod can be used instead of an Edge to constraint two vertices. The orientation of the rod can be
    /// used to orient geometry attached to the rod (e.g. a plant leaf). Note that each rod needs to be constrained
    /// by at least one RodBendTwist constraint in order to constrain the rotation of the rod. If you don't do
    /// this then the orientation is likely to rotate around the rod axis with constant velocity.
    /// Based on "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
    /// See: https://www.researchgate.net/publication/325597548_Position_and_Orientation_Based_Cosserat_Rods
    pub const RodStretchShear = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::RodStretchShear)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::RodStretchShear",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mVertex", .vertex),
                .attr("mLength", .length),
                .attr("mInvMass", .inv_mass),
                .attr("mCompliance", .compliance),
                .attr("mBishop", .bishop),
            },
        });

        /// Indices of the vertices that form the rod
        vertex: [2]u32,
        /// Fixed length of the rod, calculated by CalculateRodProperties
        length: f32 = 1.0,
        /// Inverse of the mass of the rod (0 for static rods), calculated by CalculateRodProperties but can be overridden afterwards
        inv_mass: f32 = 1.0,
        /// Inverse of the stiffness of the rod
        compliance: f32 = 0.0,
        /// The Bishop frame of the rod (the rotation of the rod in its rest pose so that it has zero twist towards adjacent rods), calculated by CalculateRodProperties
        bishop: Quat = Quat.zero(),

        /// Constructor RodStretchShear(inVertex1, inVertex2, inCompliance = 0)
        pub fn init(vertex1: u32, vertex2: u32, opts: struct { compliance: f32 = 0.0 }) RodStretchShear {
            return .{ .vertex = .{ vertex1, vertex2 }, .compliance = opts.compliance };
        }

        /// Return the lowest vertex index of this constraint
        pub fn getMinVertexIndex(self: RodStretchShear) u32 {
            return @min(self.vertex[0], self.vertex[1]);
        }
    };

    /// A constraint that connects two Cosserat rods and limits bend and twist between the rods.
    pub const RodBendTwist = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::RodBendTwist)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "SoftBodySharedSettings::RodBendTwist",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mRod", .rod),
                .attr("mCompliance", .compliance),
                .attr("mOmega0", .omega0),
            },
        });

        /// Indices of rods that are constrained (index in mRodStretchShearConstraints)
        rod: [2]u32,
        /// Inverse of the stiffness of the rod
        compliance: f32 = 0.0,
        /// The initial rotation between the rods: rod1.mBishop.Conjugated() * rod2.mBishop, calculated by CalculateRodProperties
        omega0: Quat = Quat.zero(),

        /// Constructor RodBendTwist(inRod1, inRod2, inCompliance = 0)
        pub fn init(rod1: u32, rod2: u32, opts: struct { compliance: f32 = 0.0 }) RodBendTwist {
            return .{ .rod = .{ rod1, rod2 }, .compliance = opts.compliance };
        }
    };

    /// Tracks the closest kinematic vertex (private in Jolt)
    pub const ClosestKinematic = struct {
        /// Vertex index of closest kinematic vertex
        vertex: u32 = 0xffffffff,
        /// Number of hops to the closest kinematic vertex
        hops: u32 = 0xffffffff,
        /// Distance to the closest kinematic vertex
        distance: f32 = math.flt_max,
    };

    /// Tracks the end indices of the various constraint groups (private in Jolt)
    pub const UpdateGroup = extern struct {
        /// The end index of the edge constraints in this group
        edge_end_index: u32,
        /// The end index of the LRA constraints in this group
        lra_end_index: u32,
        /// The end index of the rod stretch shear constraints in this group
        rod_stretch_shear_end_index: u32,
        /// The end index of the rod bend twist constraints in this group
        rod_bend_twist_end_index: u32,
        /// The end index of the dihedral bend constraints in this group
        dihedral_bend_end_index: u32,
        /// The end index of the volume constraints in this group
        volume_end_index: u32,
        /// The end index of the skinned constraints in this group
        skinned_end_index: u32,

        comptime {
            std.debug.assert(@sizeOf(UpdateGroup) == 28);
        }
    };

    /// RefTarget<SoftBodySharedSettings>
    ref_count: RefCount = .{},
    /// Allocates the arrays and frees the settings
    allocator: Allocator,

    /// The list of vertices or particles of the body
    vertices: std.ArrayList(Vertex) = .empty,
    /// The list of faces of the body
    faces: std.ArrayList(Face) = .empty,
    /// The list of edges or springs of the body
    edge_constraints: std.ArrayList(Edge) = .empty,
    /// The list of dihedral bend constraints of the body
    dihedral_bend_constraints: std.ArrayList(DihedralBend) = .empty,
    /// The list of volume constraints of the body that keep the volume of tetrahedra in the soft body constant
    volume_constraints: std.ArrayList(Volume) = .empty,
    /// The list of vertices that are constrained to a skinned vertex
    skinned_constraints: std.ArrayList(Skinned) = .empty,
    /// The list of inverse bind matrices for skinning vertices
    inv_bind_matrices: std.ArrayList(InvBind) = .empty,
    /// The list of long range attachment constraints
    lra_constraints: std.ArrayList(LRA) = .empty,
    /// The list of Cosserat rod constraints that connect two vertices and that limit stretch and shear
    rod_stretch_shear_constraints: std.ArrayList(RodStretchShear) = .empty,
    /// The list of Cosserat rod constraints that connect two rods and limit the bend and twist
    rod_bend_twist_constraints: std.ArrayList(RodBendTwist) = .empty,
    /// The materials of the faces of the body, referenced by Face::mMaterialIndex (`init` / `create` add
    /// PhysicsMaterial::sDefault)
    materials: PhysicsMaterialList = .empty,

    /// The closest kinematic vertex to each vertex in mVertices (private in Jolt)
    closest_kinematic: std.ArrayList(ClosestKinematic) = .empty,
    /// The end indices for each group of constraints that can be updated in parallel (private in Jolt)
    update_groups: std.ArrayList(UpdateGroup) = .empty,
    /// A list of indices in the mFaces array used by mSkinnedConstraints, calculated by CalculateSkinnedConstraintNormals (private in Jolt)
    skinned_constraint_normals: std.ArrayList(u32) = .empty,

    pub const SharedSettingsToIDMap = ObjectToIDMap(SoftBodySharedSettings);
    pub const IDToSharedSettingsMap = IDToObjectMap(SoftBodySharedSettings);
    pub const MaterialToIDMap = ObjectToIDMap(PhysicsMaterial);
    pub const IDToMaterialMap = IDToObjectMap(PhysicsMaterial);

    pub const SettingsResult = Result(Ref(SoftBodySharedSettings));

    /// Constructor of settings that are not on the heap (`deinit` them). Allocates the material list
    /// (`mMaterials { PhysicsMaterial::sDefault }`).
    pub fn init(allocator: Allocator) Allocator.Error!SoftBodySharedSettings {
        var self: SoftBodySharedSettings = .{ .allocator = allocator };
        try self.materials.append(allocator, .init(PhysicsMaterial.default));
        return self;
    }

    /// new SoftBodySharedSettings: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*SoftBodySharedSettings {
        const self = try allocator.create(SoftBodySharedSettings);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator);
        return self;
    }

    /// new SoftBodySharedSettings (RTTI create)
    pub const createDefault = create;

    /// Destructor: frees the arrays and releases the materials
    pub fn deinit(self: *SoftBodySharedSettings) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *SoftBodySharedSettings) void {
        const allocator = self.allocator;
        self.vertices.deinit(allocator);
        self.faces.deinit(allocator);
        self.edge_constraints.deinit(allocator);
        self.dihedral_bend_constraints.deinit(allocator);
        self.volume_constraints.deinit(allocator);
        self.skinned_constraints.deinit(allocator);
        self.inv_bind_matrices.deinit(allocator);
        self.lra_constraints.deinit(allocator);
        self.rod_stretch_shear_constraints.deinit(allocator);
        self.rod_bend_twist_constraints.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        self.closest_kinematic.deinit(allocator);
        self.update_groups.deinit(allocator);
        self.skinned_constraint_normals.deinit(allocator);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const SoftBodySharedSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const SoftBodySharedSettings) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this object as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const SoftBodySharedSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const SoftBodySharedSettings) u32 {
        return self.ref_count.get();
    }

    /// Automatically create constraints based on the faces of the soft body
    /// @param vertex_attributes A list of attributes for each vertex (1-on-1 with mVertices, note that if the list is smaller than mVertices the last element will be repeated). This defines the properties of the constraints that are created.
    /// @param opts.bend_type The type of bend constraint to create
    /// @param opts.angle_tolerance Shear edges are created when two connected triangles form a quad (are roughly in the same plane and form a square with roughly 90 degree angles). This defines the tolerance (in radians).
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The settings stay memory safe, but the
    /// constraint arrays may hold part of the result: create the constraints again from the start.
    pub fn createConstraints(self: *SoftBodySharedSettings, vertex_attributes: []const VertexAttributes, opts: struct { bend_type: BendType = .distance, angle_tolerance: f32 = math.degreesToRadians(8.0) }) Allocator.Error!void {
        const allocator = self.allocator;

        // Zolt (R20 UB13): every edge of a face reads the attributes of its vertices. Jolt reads
        // inVertexAttributes[min(v, 0u - 1)] when there are none.
        if (vertex_attributes.len == 0 and self.faces.items.len > 0)
            @panic("SoftBodySharedSettings.createConstraints: no vertex attributes");

        // Create list of all edges
        var edges: std.ArrayList(EdgeHelper) = .empty;
        defer edges.deinit(allocator);
        try edges.ensureTotalCapacity(allocator, self.faces.items.len * 3);
        for (self.faces.items, 0..) |f, face_index| {
            for (0..3) |i| {
                const v0 = f.vertex[i];
                const v1 = f.vertex[(i + 1) % 3];

                edges.appendAssumeCapacity(.{
                    .vertex = .{ @min(v0, v1), @max(v0, v1) },
                    .edge_idx = @as(u32, @intCast(face_index)) * 3 + @as(u32, @intCast(i)),
                });
            }
        }

        // Sort the edges
        quickSort(EdgeHelper, edges.items, {}, EdgeHelper.lessThan);

        // Helper function to get the attributes of a vertex
        const num_attributes: u32 = @intCast(vertex_attributes.len);
        const Attr = struct {
            attributes: []const VertexAttributes,
            num_attributes: u32,

            fn get(a: @This(), vertex: u32) VertexAttributes {
                return a.attributes[@min(vertex, a.num_attributes -% 1)];
            }
        };
        const attr: Attr = .{ .attributes = vertex_attributes, .num_attributes = num_attributes };

        // Create the constraints
        const sq_sin_tolerance = math.square(trigonometry.sin(opts.angle_tolerance));
        const sq_cos_tolerance = math.square(trigonometry.cos(opts.angle_tolerance));
        self.edge_constraints.clearRetainingCapacity();
        try self.edge_constraints.ensureTotalCapacity(allocator, edges.items.len);
        var i: usize = 0;
        while (i < edges.items.len) : (i += 1) {
            const e0 = edges.items[i];

            // Get attributes for the vertices of the edge
            const a0 = attr.get(e0.vertex[0]);
            const a1 = attr.get(e0.vertex[1]);

            // Flag that indicates if this edge is a shear edge (if 2 triangles form a quad-like shape and this edge is on the diagonal)
            var is_shear = false;

            // Test if there are any shared edges
            var j = i + 1;
            while (j < edges.items.len) : (j += 1) {
                const e1 = edges.items[j];
                if (e0.vertex[0] == e1.vertex[0] and e0.vertex[1] == e1.vertex[1]) {
                    // Get opposing vertices
                    const f0 = self.faces.items[e0.edge_idx / 3];
                    const f1 = self.faces.items[e1.edge_idx / 3];
                    const vopposite0 = f0.vertex[(e0.edge_idx + 2) % 3];
                    const vopposite1 = f1.vertex[(e1.edge_idx + 2) % 3];
                    const a_opposite0 = attr.get(vopposite0);
                    const a_opposite1 = attr.get(vopposite1);

                    // If the opposite vertices happen to be the same vertex then we have 2 triangles back to back and we skip creating shear / bend constraints
                    if (vopposite0 == vopposite1)
                        continue;

                    // Faces should be roughly in a plane
                    const n0 = self.vertexPosition(f0.vertex[2]).sub(self.vertexPosition(f0.vertex[0])).cross(self.vertexPosition(f0.vertex[1]).sub(self.vertexPosition(f0.vertex[0])));
                    const n1 = self.vertexPosition(f1.vertex[2]).sub(self.vertexPosition(f1.vertex[0])).cross(self.vertexPosition(f1.vertex[1]).sub(self.vertexPosition(f1.vertex[0])));
                    const n0_dot_n1 = n0.dot(n1);
                    if (n0_dot_n1 > 0.0 and math.square(n0_dot_n1) > sq_cos_tolerance * n0.lengthSq() * n1.lengthSq()) {
                        // Faces should approximately form a quad
                        const e0_dir = self.vertexPosition(vopposite0).sub(self.vertexPosition(e0.vertex[0]));
                        const e1_dir = self.vertexPosition(vopposite1).sub(self.vertexPosition(e0.vertex[0]));
                        if (math.square(e0_dir.dot(e1_dir)) < sq_sin_tolerance * e0_dir.lengthSq() * e1_dir.lengthSq()) {
                            // Shear constraint
                            try self.addEdge(vopposite0, vopposite1, a_opposite0.shear_compliance, a_opposite1.shear_compliance);
                            is_shear = true;
                        }
                    }

                    // Bend constraint
                    switch (opts.bend_type) {
                        .none => {
                            // Do nothing
                        },

                        .distance => {
                            // Create an edge constraint to represent the bend constraint
                            // Use the bend compliance of the shared edge
                            if (!is_shear)
                                try self.addEdge(vopposite0, vopposite1, a0.bend_compliance, a1.bend_compliance);
                        },

                        .dihedral => {
                            // Test if both opposite vertices are free to move
                            if ((self.vertices.items[vopposite0].inv_mass > 0.0 or self.vertices.items[vopposite1].inv_mass > 0.0) and
                                a0.bend_compliance < math.flt_max and a1.bend_compliance < math.flt_max)
                            {
                                // Create a bend constraint
                                // Use the bend compliance of the shared edge
                                try self.dihedral_bend_constraints.append(allocator, .init(e0.vertex[0], e0.vertex[1], vopposite0, vopposite1, .{ .compliance = 0.5 * (a0.bend_compliance + a1.bend_compliance) }));
                            }
                        },
                    }
                } else {
                    // Start iterating from the first non-shared edge
                    i = j - 1;
                    break;
                }
            }

            // Create a edge constraint for the current edge
            try self.addEdge(e0.vertex[0], e0.vertex[1], if (is_shear) a0.shear_compliance else a0.compliance, if (is_shear) a1.shear_compliance else a1.compliance);
        }
        self.edge_constraints.shrinkAndFree(allocator, self.edge_constraints.items.len);

        // Calculate the initial angle for all bend constraints
        self.calculateBendConstraintConstants();

        // Check if any vertices have LRA constraints
        var has_lra_constraints = false;
        for (vertex_attributes) |va| {
            if (va.lra_type != .none) {
                has_lra_constraints = true;
                break;
            }
        }
        if (has_lra_constraints) {
            // Zolt (R20 UB12): a cache of another size than the vertices makes Jolt read out of bounds
            self.checkClosestKinematicCache();

            // Ensure we have calculated the closest kinematic vertex for each vertex
            try self.calculateClosestKinematic();

            // Find non-kinematic vertices
            var v: u32 = 0;
            while (v < @as(u32, @intCast(self.vertices.items.len))) : (v += 1) {
                if (self.vertices.items[v].inv_mass > 0.0) {
                    // Check if a closest vertex was found
                    const closest = self.closest_kinematic.items[v].vertex;
                    if (closest != 0xffffffff) {
                        // Check which LRA constraint to create
                        const va = attr.get(v);
                        switch (va.lra_type) {
                            .none => {},

                            .euclidean_distance => try self.lra_constraints.append(allocator, .init(closest, v, va.lra_max_distance_multiplier * self.vertexPosition(closest).sub(self.vertexPosition(v)).length())),

                            .geodesic_distance => try self.lra_constraints.append(allocator, .init(closest, v, va.lra_max_distance_multiplier * self.closest_kinematic.items[v].distance)),
                        }
                    }
                }
            }
        }
    }

    /// Zolt (R20 UB12): a filled closest kinematic cache must have one element per vertex. CalculateClosestKinematic
    /// keeps a filled cache, so after vertices were added or removed Jolt reads mClosestKinematic or mVertices out of
    /// bounds.
    fn checkClosestKinematicCache(self: *const SoftBodySharedSettings) void {
        const num_cached = self.closest_kinematic.items.len;
        if (num_cached != 0 and num_cached != self.vertices.items.len)
            @panic("SoftBodySharedSettings: vertices were added or removed after the closest kinematic vertices were cached");
    }

    /// Zolt (R20 UB14): the index in `groups` of a constraint whose vertices are all in group `g`. Jolt reads groups[-1]
    /// when no vertex of the constraint is in a group (a vertex at a NaN, infinite or FLT_MAX position).
    fn groupIndex(g: i32, comptime message: []const u8) usize {
        if (g < 0)
            @panic(message);
        return @intCast(g);
    }

    const no_grouped_vertex = "SoftBodySharedSettings.optimize: a constraint has no vertex in a group (a vertex position is NaN, infinite or FLT_MAX)";
    const skinned_ungrouped_vertex = "SoftBodySharedSettings.optimize: a skinned constraint is on a vertex that is not in a group (its position is NaN, infinite or FLT_MAX)";

    /// An edge of a face, used by CreateConstraints (EdgeHelper)
    const EdgeHelper = struct {
        vertex: [2]u32,
        edge_idx: u32,

        /// The comparator of the edge sort. Two faces that share an edge give equal keys: their order is the tie order
        /// of Jolt's QuickSort (RemainingFeaturesArchitecture.md R20).
        fn lessThan(_: void, lhs: EdgeHelper, rhs: EdgeHelper) bool {
            return lhs.vertex[0] < rhs.vertex[0] or (lhs.vertex[0] == rhs.vertex[0] and lhs.vertex[1] < rhs.vertex[1]);
        }
    };

    /// Only add edges if one of the vertices is movable (the add_edge lambda of CreateConstraints)
    fn addEdge(self: *SoftBodySharedSettings, vtx1: u32, vtx2: u32, compliance1: f32, compliance2: f32) Allocator.Error!void {
        if ((self.vertices.items[vtx1].inv_mass > 0.0 or self.vertices.items[vtx2].inv_mass > 0.0) and
            compliance1 < math.flt_max and compliance2 < math.flt_max)
        {
            var temp_edge: Edge = .{ .vertex = .{ vtx1, vtx2 } };
            temp_edge.compliance = 0.5 * (compliance1 + compliance2);
            temp_edge.rest_length = self.vertexPosition(vtx2).sub(self.vertexPosition(vtx1)).length();
            if (Core.enable_asserts) std.debug.assert(temp_edge.rest_length > 0.0); // Two vertices at the same position: Jolt's release build continues
            try self.edge_constraints.append(self.allocator, temp_edge);
        }
    }

    /// Vec3(mVertices[inVertex].mPosition)
    fn vertexPosition(self: *const SoftBodySharedSettings, vertex: u32) Vec3 {
        return Vec3.fromFloat3(self.vertices.items[vertex].position);
    }

    /// Calculate the initial lengths of all springs of the edges of this soft body (if you use CreateConstraint, this is already done)
    pub fn calculateEdgeLengths(self: *SoftBodySharedSettings) void {
        for (self.edge_constraints.items) |*e| {
            if (Core.enable_asserts) std.debug.assert(e.vertex[0] != e.vertex[1]); // Edges need to connect 2 different vertices
            e.rest_length = self.vertexPosition(e.vertex[1]).sub(self.vertexPosition(e.vertex[0])).length();
            if (Core.enable_asserts) std.debug.assert(e.rest_length > 0.0);
        }
    }

    /// Calculate the properties of the rods
    /// Note that this can swap mVertex of the RodStretchShear constraints if two rods are connected through a RodBendTwist constraint but point in opposite directions.
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The rods may then be partially updated.
    pub fn calculateRodProperties(self: *SoftBodySharedSettings) Allocator.Error!void {
        const allocator = self.allocator;
        const rods = self.rod_stretch_shear_constraints.items;

        // Mark connections through bend twist constraints
        const connections = try allocator.alloc(std.ArrayList(u32), rods.len);
        @memset(connections, .empty);
        defer freeLists(u32, allocator, connections);
        for (self.rod_bend_twist_constraints.items) |c| {
            if (Core.enable_asserts) std.debug.assert(c.rod[0] != c.rod[1]); // A bend twist constraint needs to be attached to different rods
            try connections[c.rod[1]].append(allocator, c.rod[0]);
            try connections[c.rod[0]].append(allocator, c.rod[1]);
        }

        // Now calculate the Bishop frames for all rods
        const Entry = struct {
            /// Rod we're coming from
            from: u32,
            /// Rod we're going to
            to: u32,
        };
        var stack: std.ArrayList(Entry) = .empty;
        defer stack.deinit(allocator);
        try stack.ensureTotalCapacity(allocator, rods.len);
        var r0_idx: u32 = 0;
        while (r0_idx < @as(u32, @intCast(rods.len))) : (r0_idx += 1) {
            const r0 = &rods[r0_idx];

            // Do not calculate a 2nd time
            if (r0.bishop.eql(Quat.zero())) {
                // Calculate the frame for this rod
                {
                    var tangent = self.vertexPosition(r0.vertex[1]).sub(self.vertexPosition(r0.vertex[0]));
                    r0.length = tangent.length();
                    if (Core.enable_asserts) std.debug.assert(r0.length > 0.0); // Rods of zero length are not supported!
                    tangent = tangent.divScalar(r0.length);
                    const normal = tangent.getNormalizedPerpendicular();
                    const binormal = tangent.cross(normal);
                    r0.bishop = Mat44.init(Vec4.fromVec3W(normal, 0), Vec4.fromVec3W(binormal, 0), Vec4.fromVec3W(tangent, 0), Vec4.init(0, 0, 0, 1)).getQuaternion().normalized();
                }

                // Add connected rods to the stack if they haven't been calculated yet
                for (connections[r0_idx].items) |r1_idx| {
                    if (rods[r1_idx].bishop.eql(Quat.zero()))
                        try stack.append(allocator, .{ .from = r0_idx, .to = r1_idx });
                }

                // Now connect the bishop frame for all connected rods on the stack
                // This follows the procedure outlined in "Discrete Elastic Rods" - M. Bergou et al.
                // See: https://www.cs.columbia.edu/cg/pdfs/143-rods.pdf
                while (stack.items.len != 0) {
                    const r1_idx = stack.getLast().from;
                    const r2_idx = stack.getLast().to;
                    _ = stack.pop();

                    const r1 = &rods[r1_idx];
                    const r2 = &rods[r2_idx];

                    // Get the normal and tangent of the first rod's Bishop frame (that was already calculated)
                    const r1_frame = Mat44.rotationQuat(r1.bishop);
                    const tangent1 = r1_frame.getAxisZ();
                    const normal1 = r1_frame.getAxisX();

                    // Calculate the Bishop frame for the 2nd rod
                    var tangent2 = self.vertexPosition(r2.vertex[1]).sub(self.vertexPosition(r2.vertex[0]));
                    if (tangent1.dot(tangent2) < 0.0) {
                        // Edge is oriented in the opposite direction of the previous edge, flip it
                        Core.swap(u32, &r2.vertex[0], &r2.vertex[1]);
                        tangent2 = tangent2.negate();
                    }
                    r2.length = tangent2.length();
                    if (Core.enable_asserts) std.debug.assert(r2.length > 0.0); // Rods of zero length are not supported!
                    tangent2 = tangent2.divScalar(r2.length);
                    var t1_cross_t2 = tangent1.cross(tangent2);
                    const sin_angle = t1_cross_t2.length();
                    var normal2 = normal1;
                    if (sin_angle > 1.0e-6) {
                        t1_cross_t2 = t1_cross_t2.divScalar(sin_angle);
                        normal2 = Quat.rotation(t1_cross_t2, trigonometry.asin(sin_angle)).mulVec3(normal2);
                    }
                    const binormal2 = tangent2.cross(normal2);
                    r2.bishop = Mat44.init(Vec4.fromVec3W(normal2, 0), Vec4.fromVec3W(binormal2, 0), Vec4.fromVec3W(tangent2, 0), Vec4.init(0, 0, 0, 1)).getQuaternion().normalized();

                    // Add connected rods to the stack if they haven't been calculated yet
                    for (connections[r2_idx].items) |r3_idx| {
                        if (rods[r3_idx].bishop.eql(Quat.zero()))
                            try stack.append(allocator, .{ .from = r2_idx, .to = r3_idx });
                    }
                }
            }
        }

        // Calculate inverse mass for all rods by taking the minimum inverse mass (aka the heaviest vertex) of both vertices
        for (rods) |*r| {
            if (Core.enable_asserts) std.debug.assert(r.vertex[0] != r.vertex[1]); // A rod stretch shear constraint requires two different vertices
            r.inv_mass = math.min(self.vertices.items[r.vertex[0]].inv_mass, self.vertices.items[r.vertex[1]].inv_mass);
        }

        // Calculate the initial rotation between the rods
        for (self.rod_bend_twist_constraints.items) |*r|
            r.omega0 = rods[r.rod[0]].bishop.conjugated().mul(rods[r.rod[1]].bishop).normalized();
    }

    /// Calculate the max lengths for the long range attachment constraints based on Euclidean distance (if you use CreateConstraints, this is already done)
    /// @param opts.max_distance_multiplier Multiplier for the max distance of the LRA constraint, e.g. 1.01 means the max distance is 1% longer than the calculated distance in the rest pose.
    pub fn calculateLRALengths(self: *SoftBodySharedSettings, opts: struct { max_distance_multiplier: f32 = 1.0 }) void {
        for (self.lra_constraints.items) |*l| {
            if (Core.enable_asserts) std.debug.assert(l.vertex[0] != l.vertex[1]); // LRA constraints need to connect 2 different vertices
            l.max_distance = opts.max_distance_multiplier * self.vertexPosition(l.vertex[1]).sub(self.vertexPosition(l.vertex[0])).length();
            if (Core.enable_asserts) std.debug.assert(l.max_distance > 0.0);
        }
    }

    /// Calculate the constants for the bend constraints (if you use CreateConstraints, this is already done)
    pub fn calculateBendConstraintConstants(self: *SoftBodySharedSettings) void {
        for (self.dihedral_bend_constraints.items) |*b| {
            // Bend constraints need 4 different vertices
            if (Core.enable_asserts) std.debug.assert(b.vertex[0] != b.vertex[1] and b.vertex[0] != b.vertex[2] and b.vertex[0] != b.vertex[3] and
                b.vertex[1] != b.vertex[2] and b.vertex[1] != b.vertex[3] and
                b.vertex[2] != b.vertex[3]);

            // Get positions
            const x0 = self.vertexPosition(b.vertex[0]);
            const x1 = self.vertexPosition(b.vertex[1]);
            const x2 = self.vertexPosition(b.vertex[2]);
            const x3 = self.vertexPosition(b.vertex[3]);

            //    x2
            // e1/  \e3
            //  /    \
            // x0----x1
            //  \ e0 /
            // e2\  /e4
            //    x3

            // Calculate edges
            const e0 = x1.sub(x0);
            const e1 = x2.sub(x0);
            const e2 = x3.sub(x0);

            // Normals of both triangles
            const n1 = e0.cross(e1);
            const n2 = e2.cross(e0);
            const denom = @sqrt(n1.lengthSq() * n2.lengthSq());
            if (denom < 1.0e-12) {
                b.initial_angle = 0.0;
            } else {
                const sign = math.sign(n2.cross(n1).dot(e0));
                b.initial_angle = sign * trigonometry.acosApproximate(n1.dot(n2) / denom); // Runtime uses the approximation too
            }
        }
    }

    /// Calculates the initial volume of all tetrahedra of this soft body
    pub fn calculateVolumeConstraintVolumes(self: *SoftBodySharedSettings) void {
        for (self.volume_constraints.items) |*v| {
            // Volume constraints need 4 different vertices
            if (Core.enable_asserts) std.debug.assert(v.vertex[0] != v.vertex[1] and v.vertex[0] != v.vertex[2] and v.vertex[0] != v.vertex[3] and
                v.vertex[1] != v.vertex[2] and v.vertex[1] != v.vertex[3] and
                v.vertex[2] != v.vertex[3]);

            const x1 = self.vertexPosition(v.vertex[0]);
            const x2 = self.vertexPosition(v.vertex[1]);
            const x3 = self.vertexPosition(v.vertex[2]);
            const x4 = self.vertexPosition(v.vertex[3]);

            const x1x2 = x2.sub(x1);
            const x1x3 = x3.sub(x1);
            const x1x4 = x4.sub(x1);

            v.six_rest_volume = @abs(x1x2.cross(x1x3).dot(x1x4));
        }
    }

    /// Calculate information needed to be able to calculate the skinned constraint normals at run-time
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The normals may then be incomplete: call it
    /// again.
    pub fn calculateSkinnedConstraintNormals(self: *SoftBodySharedSettings) Allocator.Error!void {
        const allocator = self.allocator;

        // Clear any previous results
        self.skinned_constraint_normals.clearRetainingCapacity();

        // If there are no skinned constraints, we're done
        if (self.skinned_constraints.items.len == 0)
            return;

        // First collect all vertices that are skinned
        const VertexIndexSet = UnorderedSet(u32, .{});
        var skinned_vertices: VertexIndexSet = .empty;
        defer skinned_vertices.deinit(allocator);
        try skinned_vertices.ensureTotalCapacity(allocator, @intCast(self.skinned_constraints.items.len));
        for (self.skinned_constraints.items) |s|
            _ = try skinned_vertices.insert(allocator, s.vertex);

        // Now collect all faces that connect only to skinned vertices
        const ConnectedFacesMap = UnorderedMap(u32, VertexIndexSet, .{});
        var connected_faces: ConnectedFacesMap = .empty;
        defer {
            var it = connected_faces.iterator();
            while (it.next()) |kv| kv.value.deinit(allocator);
            connected_faces.deinit(allocator);
        }
        try connected_faces.ensureTotalCapacity(allocator, @intCast(self.vertices.items.len));
        for (self.faces.items, 0..) |f, face_index| {
            // Must connect to only skinned vertices
            var valid = true;
            for (f.vertex) |v|
                valid = valid and skinned_vertices.find(v) != null;
            if (!valid)
                continue;

            // Store faces that connect to vertices
            for (f.vertex) |v| {
                const faces = try connected_faces.getOrPutValue(allocator, v, .empty);
                _ = try faces.insert(allocator, @intCast(face_index));
            }
        }

        // Populate the list of connecting faces per skinned vertex
        try self.skinned_constraint_normals.ensureTotalCapacity(allocator, self.faces.items.len);
        for (self.skinned_constraints.items) |*s| {
            const start: u32 = @intCast(self.skinned_constraint_normals.items.len);
            if (Core.enable_asserts) std.debug.assert((start >> 24) == 0);
            if (connected_faces.find(s.vertex)) |connected_faces_it| {
                const faces = &connected_faces_it.value;
                const num: u32 = faces.count();
                if (Core.enable_asserts) std.debug.assert(num < 256);
                var it = faces.constIterator();
                while (it.next()) |face|
                    try self.skinned_constraint_normals.append(allocator, face.*);
                quickSort(u32, self.skinned_constraint_normals.items[start .. start + num], {}, std.sort.asc(u32));
                s.normal_info = start +% (num << 24);
            } else s.normal_info = 0;
        }
        self.skinned_constraint_normals.shrinkAndFree(allocator, self.skinned_constraint_normals.items.len);
    }

    /// Optimize the soft body settings for simulation. This will reorder constraints so they can be executed in parallel.
    /// `results` holds arrays allocated with the allocator of the settings (`results.deinit(settings.allocator)`).
    /// Zolt: an allocation failure returns `error.OutOfMemory` (Jolt aborts). The settings are then unchanged: the new
    /// constraint arrays, the remaps and the update groups are allocated before anything moves, and the update groups
    /// are cleared after that (Jolt clears them first, which only matters when Jolt aborts). The remaps may be resized.
    pub fn optimizeWithResults(self: *SoftBodySharedSettings, results: *OptimizationResults) Allocator.Error!void {
        const allocator = self.allocator;
        const num_vertices = self.vertices.items.len;
        const batch = SoftBodyUpdateContext.vertex_constraint_batch;

        // Create a list of connected vertices
        var connectivity = try allocator.alloc(std.ArrayList(Connection), num_vertices);
        @memset(connectivity, .empty);
        defer freeLists(Connection, allocator, connectivity);
        for (self.edge_constraints.items) |c|
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[1]);
        for (self.lra_constraints.items) |c|
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[1]);
        for (self.rod_stretch_shear_constraints.items) |c|
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[1]);
        const rods = self.rod_stretch_shear_constraints.items;
        for (self.rod_bend_twist_constraints.items) |c| {
            try addConnection(allocator, connectivity, rods[c.rod[0]].vertex[0], rods[c.rod[1]].vertex[0]);
            try addConnection(allocator, connectivity, rods[c.rod[0]].vertex[1], rods[c.rod[1]].vertex[0]);
            try addConnection(allocator, connectivity, rods[c.rod[0]].vertex[0], rods[c.rod[1]].vertex[1]);
            try addConnection(allocator, connectivity, rods[c.rod[0]].vertex[1], rods[c.rod[1]].vertex[1]);
        }
        for (self.dihedral_bend_constraints.items) |c| {
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[1]);
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[2]);
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[3]);
            try addConnection(allocator, connectivity, c.vertex[1], c.vertex[2]);
            try addConnection(allocator, connectivity, c.vertex[1], c.vertex[3]);
            try addConnection(allocator, connectivity, c.vertex[2], c.vertex[3]);
        }
        for (self.volume_constraints.items) |c| {
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[1]);
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[2]);
            try addConnection(allocator, connectivity, c.vertex[0], c.vertex[3]);
            try addConnection(allocator, connectivity, c.vertex[1], c.vertex[2]);
            try addConnection(allocator, connectivity, c.vertex[1], c.vertex[3]);
            try addConnection(allocator, connectivity, c.vertex[2], c.vertex[3]);
        }
        // Skinned constraints only update 1 vertex, so we don't need special logic here

        // Maps each of the vertices to a group index
        const group_idx = try allocator.alloc(i32, num_vertices);
        defer allocator.free(group_idx);
        @memset(group_idx, -1);

        // Which group we are currently filling and its vertices
        var current_group_idx: i32 = 0;
        var current_group: std.ArrayList(u32) = .empty;
        defer current_group.deinit(allocator);

        // Start greedy algorithm to group vertices
        while (true) {
            // Find the bounding box of the ungrouped vertices
            var bounds: AABox = .empty;
            for (0..num_vertices) |i| {
                if (group_idx[i] == -1)
                    bounds.encapsulateVec3(Vec3.fromFloat3(self.vertices.items[i].position));
            }

            // If the bounds are invalid, it means that there were no ungrouped vertices
            if (!bounds.isValid())
                break;

            // Determine longest and shortest axis
            const bounds_size = bounds.getSize();
            const max_axis = bounds_size.getHighestComponentIndex();
            var min_axis = bounds_size.getLowestComponentIndex();
            if (min_axis == max_axis)
                min_axis = (min_axis + 1) % 3;
            const mid_axis = 3 - min_axis - max_axis;

            // Find the vertex that has the lowest value on the axis with the largest extent
            var current_vertex: u32 = std.math.maxInt(u32);
            var current_vertex_position: Float3 = .init(math.flt_max, math.flt_max, math.flt_max);
            for (0..num_vertices) |i| {
                if (group_idx[i] == -1) {
                    const vertex_position = self.vertices.items[i].position;
                    const max_axis_value = vertex_position.getComponent(max_axis);
                    const mid_axis_value = vertex_position.getComponent(mid_axis);
                    const min_axis_value = vertex_position.getComponent(min_axis);

                    if (max_axis_value < current_vertex_position.getComponent(max_axis) or
                        (max_axis_value == current_vertex_position.getComponent(max_axis) and
                            (mid_axis_value < current_vertex_position.getComponent(mid_axis) or
                                (mid_axis_value == current_vertex_position.getComponent(mid_axis) and
                                    min_axis_value < current_vertex_position.getComponent(min_axis)))))
                    {
                        current_vertex_position = self.vertices.items[i].position;
                        current_vertex = @intCast(i);
                    }
                }
            }
            if (current_vertex == std.math.maxInt(u32))
                break;

            // Initialize the current group with 1 vertex
            try current_group.append(allocator, current_vertex);
            group_idx[current_vertex] = current_group_idx;

            // Fill up the group
            while (true) {
                // Find the vertex that is most connected to the current group
                var best_vertex: u32 = std.math.maxInt(u32);
                var best_num_connections: u32 = 0;
                var best_dist_sq: f32 = math.flt_max;
                for (current_group.items) |group_vertex| { // For all vertices in the current group
                    for (connectivity[group_vertex].items) |c| { // For all connections to other vertices
                        const v = c.vertex;
                        if (group_idx[v] == -1) { // Ungrouped vertices only
                            // Count the number of connections to this group
                            var num_connections: u32 = 0;
                            for (connectivity[v].items) |v2| {
                                if (group_idx[v2.vertex] == current_group_idx)
                                    num_connections +%= v2.count;
                            }

                            // Calculate distance to group centroid
                            const dist_sq = self.vertexPosition(v).sub(self.vertexPosition(current_group.items[0])).lengthSq();

                            if (best_vertex == std.math.maxInt(u32) or
                                num_connections > best_num_connections or
                                (num_connections == best_num_connections and dist_sq < best_dist_sq))
                            {
                                best_vertex = v;
                                best_num_connections = num_connections;
                                best_dist_sq = dist_sq;
                            }
                        }
                    }
                }

                // Add the best vertex to the current group
                if (best_vertex != std.math.maxInt(u32)) {
                    try current_group.append(allocator, best_vertex);
                    group_idx[best_vertex] = current_group_idx;
                }

                // Create a new group?
                if (current_group.items.len >= batch // If full, yes
                or (current_group.items.len > batch / 2 and best_vertex == std.math.maxInt(u32))) // If half full and we found no connected vertex, yes
                {
                    current_group.clearRetainingCapacity();
                    current_group_idx += 1;
                    break;
                }

                // If we didn't find a connected vertex, we need to find a new starting vertex
                if (best_vertex == std.math.maxInt(u32))
                    break;
            }
        }

        // If the last group is more than half full, we'll keep it as a separate group, otherwise we merge it with the 'non parallel' group
        if (current_group.items.len > batch / 2)
            current_group_idx += 1;

        // We no longer need the current group array, free the memory
        current_group.clearAndFree(allocator);

        // We're done with the connectivity list, free the memory
        freeLists(Connection, allocator, connectivity);
        connectivity = &.{};

        // Assign the constraints to their groups. Zolt (R20 UB14): Jolt asserts that every vertex of a constraint is in a
        // group. Its release build puts a constraint with only some vertices in a group in the non parallel group. Zolt
        // does the same, and the assert moves into that branch. A constraint without a grouped vertex and a skinned
        // constraint on an ungrouped vertex make Jolt read groups[-1]. There groupIndex panics with a message, in every
        // build. (For a skinned constraint groupIndex has the condition of Jolt's assert.)
        const groups = try allocator.alloc(Group, @as(usize, @intCast(current_group_idx)) + 1); // + non parallel group
        @memset(groups, .{});
        defer {
            for (groups) |*group| group.deinit(allocator);
            allocator.free(groups);
        }
        const non_parallel_group = &groups[groups.len - 1];
        for (self.edge_constraints.items, 0..) |e, index| {
            const g1 = group_idx[e.vertex[0]];
            const g2 = group_idx[e.vertex[1]];
            if (g1 == g2) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].edge_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0);
                try non_parallel_group.edge_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.lra_constraints.items, 0..) |l, index| {
            const g1 = group_idx[l.vertex[0]];
            const g2 = group_idx[l.vertex[1]];
            if (g1 == g2) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].lra_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0);
                try non_parallel_group.lra_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.rod_stretch_shear_constraints.items, 0..) |r, index| {
            const g1 = group_idx[r.vertex[0]];
            const g2 = group_idx[r.vertex[1]];
            if (g1 == g2) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].rod_stretch_shear_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0);
                try non_parallel_group.rod_stretch_shear_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.rod_bend_twist_constraints.items, 0..) |r, index| {
            const g1 = group_idx[rods[r.rod[0]].vertex[0]];
            const g2 = group_idx[rods[r.rod[0]].vertex[1]];
            const g3 = group_idx[rods[r.rod[1]].vertex[0]];
            const g4 = group_idx[rods[r.rod[1]].vertex[1]];
            if (g1 == g2 and g1 == g3 and g1 == g4) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].rod_bend_twist_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0 and g3 >= 0 and g4 >= 0);
                try non_parallel_group.rod_bend_twist_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.dihedral_bend_constraints.items, 0..) |d, index| {
            const g1 = group_idx[d.vertex[0]];
            const g2 = group_idx[d.vertex[1]];
            const g3 = group_idx[d.vertex[2]];
            const g4 = group_idx[d.vertex[3]];
            if (g1 == g2 and g1 == g3 and g1 == g4) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].dihedral_bend_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0 and g3 >= 0 and g4 >= 0);
                try non_parallel_group.dihedral_bend_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.volume_constraints.items, 0..) |v, index| {
            const g1 = group_idx[v.vertex[0]];
            const g2 = group_idx[v.vertex[1]];
            const g3 = group_idx[v.vertex[2]];
            const g4 = group_idx[v.vertex[3]];
            if (g1 == g2 and g1 == g3 and g1 == g4) // In the same group
                try groups[groupIndex(g1, no_grouped_vertex)].volume_constraints.append(allocator, @intCast(index))
            else { // In different groups -> parallel group
                if (Core.enable_asserts) std.debug.assert(g1 >= 0 and g2 >= 0 and g3 >= 0 and g4 >= 0);
                try non_parallel_group.volume_constraints.append(allocator, @intCast(index));
            }
        }
        for (self.skinned_constraints.items, 0..) |s, index| {
            const g1 = group_idx[s.vertex];
            try groups[groupIndex(g1, skinned_ungrouped_vertex)].skinned_constraints.append(allocator, @intCast(index));
        }

        // Sort the parallel groups from big to small (this means the big groups will be scheduled first and have more time to complete)
        quickSort(Group, groups[0 .. groups.len - 1], {}, Group.greaterSize);

        // Make sure we know the closest kinematic vertex so we can sort
        // (Zolt: a cache that this call fills is cleared again when the call fails, a cache from CreateConstraints stays)
        const had_closest_kinematic = self.closest_kinematic.items.len != 0;
        errdefer if (!had_closest_kinematic) self.closest_kinematic.clearAndFree(allocator);
        try self.calculateClosestKinematic();

        // Zolt (R20 UB12): the sorts read the closest kinematic vertex of every constraint vertex
        self.checkClosestKinematicCache();

        // Sort within each group
        const sort_context: *const SoftBodySharedSettings = self;
        for (groups) |*group| {
            // Sort the edge constraints
            quickSort(u32, group.edge_constraints.items, sort_context, edgeLessThan);

            // Sort the LRA constraints
            quickSort(u32, group.lra_constraints.items, sort_context, lraLessThan);

            // Sort the rod stretch shear constraints
            quickSort(u32, group.rod_stretch_shear_constraints.items, sort_context, rodStretchShearLessThan);

            // Sort the rod bend twist constraints
            quickSort(u32, group.rod_bend_twist_constraints.items, sort_context, rodBendTwistLessThan);

            // Bilateral interleaving, see figure 4 of "Position and Orientation Based Cosserat Rods" - Kugelstadt and Schoemer - SIGGRAPH 2016
            // Keeping the twist constraints sorted often results in an unstable simulation
            {
                const items = group.rod_bend_twist_constraints.items;
                const s = items.len;
                const s2 = s >> 1;
                var i: usize = 1;
                while (i < s2) : (i += 2)
                    Core.swap(u32, &items[i], &items[s - i]);
            }

            // Sort the dihedral bend constraints
            quickSort(u32, group.dihedral_bend_constraints.items, sort_context, dihedralBendLessThan);

            // Sort the volume constraints
            quickSort(u32, group.volume_constraints.items, sort_context, volumeLessThan);

            // Sort the skinned constraints
            quickSort(u32, group.skinned_constraints.items, sort_context, skinnedLessThan);
        }

        // Temporary store constraints as we reorder them. Zolt: every allocation comes first (see the doc comment), the
        // swaps with the constraint arrays follow below.
        var temp_edges: std.ArrayList(Edge) = .empty;
        defer temp_edges.deinit(allocator);
        try temp_edges.ensureTotalCapacityPrecise(allocator, self.edge_constraints.items.len);
        try resizeRemap(allocator, &results.edge_remap, self.edge_constraints.items.len);

        var temp_lra: std.ArrayList(LRA) = .empty;
        defer temp_lra.deinit(allocator);
        try temp_lra.ensureTotalCapacityPrecise(allocator, self.lra_constraints.items.len);
        try resizeRemap(allocator, &results.lra_remap, self.lra_constraints.items.len);

        var temp_rod_stretch_shear: std.ArrayList(RodStretchShear) = .empty;
        defer temp_rod_stretch_shear.deinit(allocator);
        try temp_rod_stretch_shear.ensureTotalCapacityPrecise(allocator, self.rod_stretch_shear_constraints.items.len);
        try resizeRemap(allocator, &results.rod_stretch_shear_constraint_remap, self.rod_stretch_shear_constraints.items.len);

        var temp_rod_bend_twist: std.ArrayList(RodBendTwist) = .empty;
        defer temp_rod_bend_twist.deinit(allocator);
        try temp_rod_bend_twist.ensureTotalCapacityPrecise(allocator, self.rod_bend_twist_constraints.items.len);
        try resizeRemap(allocator, &results.rod_bend_twist_constraint_remap, self.rod_bend_twist_constraints.items.len);

        var temp_dihedral_bend: std.ArrayList(DihedralBend) = .empty;
        defer temp_dihedral_bend.deinit(allocator);
        try temp_dihedral_bend.ensureTotalCapacityPrecise(allocator, self.dihedral_bend_constraints.items.len);
        try resizeRemap(allocator, &results.dihedral_bend_remap, self.dihedral_bend_constraints.items.len);

        var temp_volume: std.ArrayList(Volume) = .empty;
        defer temp_volume.deinit(allocator);
        try temp_volume.ensureTotalCapacityPrecise(allocator, self.volume_constraints.items.len);
        try resizeRemap(allocator, &results.volume_remap, self.volume_constraints.items.len);

        var temp_skinned: std.ArrayList(Skinned) = .empty;
        defer temp_skinned.deinit(allocator);
        try temp_skinned.ensureTotalCapacityPrecise(allocator, self.skinned_constraints.items.len);
        try resizeRemap(allocator, &results.skinned_remap, self.skinned_constraints.items.len);

        // Clear any previous results (Jolt does this first). Zolt: the capacity comes first, so a failure keeps the old
        // update groups.
        try self.update_groups.ensureTotalCapacity(allocator, groups.len);
        self.update_groups.clearRetainingCapacity();

        // Nothing fails from here on. temp_x.swap(mX): the temp arrays get the old constraints, the constraint arrays
        // get the reserved memory.
        Core.swap(std.ArrayList(Edge), &temp_edges, &self.edge_constraints);
        Core.swap(std.ArrayList(LRA), &temp_lra, &self.lra_constraints);
        Core.swap(std.ArrayList(RodStretchShear), &temp_rod_stretch_shear, &self.rod_stretch_shear_constraints);
        Core.swap(std.ArrayList(RodBendTwist), &temp_rod_bend_twist, &self.rod_bend_twist_constraints);
        Core.swap(std.ArrayList(DihedralBend), &temp_dihedral_bend, &self.dihedral_bend_constraints);
        Core.swap(std.ArrayList(Volume), &temp_volume, &self.volume_constraints);
        Core.swap(std.ArrayList(Skinned), &temp_skinned, &self.skinned_constraints);

        // Finalize update groups
        for (groups) |group| {
            // Reorder edge constraints for this group
            for (group.edge_constraints.items) |idx| {
                results.edge_remap.items[idx] = @intCast(self.edge_constraints.items.len);
                self.edge_constraints.appendAssumeCapacity(temp_edges.items[idx]);
            }

            // Reorder LRA constraints for this group
            for (group.lra_constraints.items) |idx| {
                results.lra_remap.items[idx] = @intCast(self.lra_constraints.items.len);
                self.lra_constraints.appendAssumeCapacity(temp_lra.items[idx]);
            }

            // Reorder rod stretch shear constraints for this group
            for (group.rod_stretch_shear_constraints.items) |idx| {
                results.rod_stretch_shear_constraint_remap.items[idx] = @intCast(self.rod_stretch_shear_constraints.items.len);
                self.rod_stretch_shear_constraints.appendAssumeCapacity(temp_rod_stretch_shear.items[idx]);
            }

            // Reorder rod bend twist constraints for this group
            for (group.rod_bend_twist_constraints.items) |idx| {
                results.rod_bend_twist_constraint_remap.items[idx] = @intCast(self.rod_bend_twist_constraints.items.len);
                self.rod_bend_twist_constraints.appendAssumeCapacity(temp_rod_bend_twist.items[idx]);
            }

            // Reorder dihedral bend constraints for this group
            for (group.dihedral_bend_constraints.items) |idx| {
                results.dihedral_bend_remap.items[idx] = @intCast(self.dihedral_bend_constraints.items.len);
                self.dihedral_bend_constraints.appendAssumeCapacity(temp_dihedral_bend.items[idx]);
            }

            // Reorder volume constraints for this group
            for (group.volume_constraints.items) |idx| {
                results.volume_remap.items[idx] = @intCast(self.volume_constraints.items.len);
                self.volume_constraints.appendAssumeCapacity(temp_volume.items[idx]);
            }

            // Reorder skinned constraints for this group
            for (group.skinned_constraints.items) |idx| {
                results.skinned_remap.items[idx] = @intCast(self.skinned_constraints.items.len);
                self.skinned_constraints.appendAssumeCapacity(temp_skinned.items[idx]);
            }

            // Store end indices
            self.update_groups.appendAssumeCapacity(.{
                .edge_end_index = @intCast(self.edge_constraints.items.len),
                .lra_end_index = @intCast(self.lra_constraints.items.len),
                .rod_stretch_shear_end_index = @intCast(self.rod_stretch_shear_constraints.items.len),
                .rod_bend_twist_end_index = @intCast(self.rod_bend_twist_constraints.items.len),
                .dihedral_bend_end_index = @intCast(self.dihedral_bend_constraints.items.len),
                .volume_end_index = @intCast(self.volume_constraints.items.len),
                .skinned_end_index = @intCast(self.skinned_constraints.items.len),
            });
        }

        // Remap bend twist indices because mRodStretchShearConstraints has been reordered
        for (self.rod_bend_twist_constraints.items) |*r| {
            for (&r.rod) |*rod|
                rod.* = results.rod_stretch_shear_constraint_remap.items[rod.*];
        }

        // Free closest kinematic buffer
        self.closest_kinematic.clearAndFree(allocator);
    }

    /// A connection of a vertex in Optimize (Connection)
    const Connection = struct {
        vertex: u32,
        count: u32,
    };

    /// The add_connection lambda of Optimize
    fn addConnection(allocator: Allocator, connectivity: []std.ArrayList(Connection), v1_in: u32, v2_in: u32) Allocator.Error!void {
        var v1 = v1_in;
        var v2 = v2_in;
        for (0..2) |_| {
            var found = false;
            for (connectivity[v1].items) |*c| {
                if (c.vertex == v2) {
                    c.count += 1;
                    found = true;
                    break;
                }
            }
            if (!found)
                try connectivity[v1].append(allocator, .{ .vertex = v2, .count = 1 });

            Core.swap(u32, &v1, &v2);
        }
    }

    /// The constraints of one update group in Optimize (Group)
    const Group = struct {
        edge_constraints: std.ArrayList(u32) = .empty,
        lra_constraints: std.ArrayList(u32) = .empty,
        rod_stretch_shear_constraints: std.ArrayList(u32) = .empty,
        rod_bend_twist_constraints: std.ArrayList(u32) = .empty,
        dihedral_bend_constraints: std.ArrayList(u32) = .empty,
        volume_constraints: std.ArrayList(u32) = .empty,
        skinned_constraints: std.ArrayList(u32) = .empty,

        fn getSize(self: *const Group) u32 {
            return @as(u32, @intCast(self.edge_constraints.items.len)) +% @as(u32, @intCast(self.lra_constraints.items.len)) +% @as(u32, @intCast(self.rod_stretch_shear_constraints.items.len)) +% @as(u32, @intCast(self.rod_bend_twist_constraints.items.len)) +% @as(u32, @intCast(self.dihedral_bend_constraints.items.len)) +% @as(u32, @intCast(self.volume_constraints.items.len)) +% @as(u32, @intCast(self.skinned_constraints.items.len));
        }

        /// The comparator of the group sort: big groups first. Groups of equal size keep the tie order of Jolt's
        /// QuickSort (RemainingFeaturesArchitecture.md R20).
        fn greaterSize(_: void, lhs: Group, rhs: Group) bool {
            return lhs.getSize() > rhs.getSize();
        }

        fn deinit(self: *Group, allocator: Allocator) void {
            self.edge_constraints.deinit(allocator);
            self.lra_constraints.deinit(allocator);
            self.rod_stretch_shear_constraints.deinit(allocator);
            self.rod_bend_twist_constraints.deinit(allocator);
            self.dihedral_bend_constraints.deinit(allocator);
            self.volume_constraints.deinit(allocator);
            self.skinned_constraints.deinit(allocator);
        }
    };

    /// Sort the edge constraints (the comparator of Optimize)
    fn edgeLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const e1 = self.edge_constraints.items[lhs];
        const e2 = self.edge_constraints.items[rhs];
        const ck = self.closest_kinematic.items;

        // First sort so that the edge with the smallest distance to a kinematic vertex comes first
        const d1 = math.min(ck[e1.vertex[0]].distance, ck[e1.vertex[1]].distance);
        const d2 = math.min(ck[e2.vertex[0]].distance, ck[e2.vertex[1]].distance);
        if (d1 != d2)
            return d1 < d2;

        // Order the edges so that the ones with the smallest index go first (hoping to get better cache locality when we process the edges).
        // Note we could also re-order the vertices but that would be much more of a burden to the end user
        const m1 = e1.getMinVertexIndex();
        const m2 = e2.getMinVertexIndex();
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the LRA constraints (the comparator of Optimize)
    fn lraLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const l1 = self.lra_constraints.items[lhs];
        const l2 = self.lra_constraints.items[rhs];

        // First sort so that the longest constraint comes first (meaning the shortest constraint has the most influence on the end result)
        // Most of the time there will be a single LRA constraint per vertex and since the LRA constraint only modifies a single vertex,
        // updating one constraint will not violate another constraint.
        if (l1.max_distance != l2.max_distance)
            return l1.max_distance > l2.max_distance;

        // Order constraints so that the ones with the smallest index go first
        const m1 = l1.getMinVertexIndex();
        const m2 = l2.getMinVertexIndex();
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the rod stretch shear constraints (the comparator of Optimize)
    fn rodStretchShearLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const r1 = self.rod_stretch_shear_constraints.items[lhs];
        const r2 = self.rod_stretch_shear_constraints.items[rhs];
        const ck = self.closest_kinematic.items;

        // First sort so that the rod with the smallest distance to a kinematic vertex comes first
        const d1 = math.min(ck[r1.vertex[0]].distance, ck[r1.vertex[1]].distance);
        const d2 = math.min(ck[r2.vertex[0]].distance, ck[r2.vertex[1]].distance);
        if (d1 != d2)
            return d1 < d2;

        // Then sort on the rod that connects to the smallest kinematic vertex
        var m1 = @min(ck[r1.vertex[0]].vertex, ck[r1.vertex[1]].vertex);
        var m2 = @min(ck[r2.vertex[0]].vertex, ck[r2.vertex[1]].vertex);
        if (m1 != m2)
            return m1 < m2;

        // Order the rods so that the ones with the smallest index go first (hoping to get better cache locality when we process the rods).
        m1 = r1.getMinVertexIndex();
        m2 = r2.getMinVertexIndex();
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the rod bend twist constraints (the comparator of Optimize)
    fn rodBendTwistLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const rods = self.rod_stretch_shear_constraints.items;
        const ck = self.closest_kinematic.items;

        const b1 = self.rod_bend_twist_constraints.items[lhs];
        const b1_r1 = rods[b1.rod[0]];
        const b1_r2 = rods[b1.rod[1]];

        const b2 = self.rod_bend_twist_constraints.items[rhs];
        const b2_r1 = rods[b2.rod[0]];
        const b2_r2 = rods[b2.rod[1]];

        // First sort so that the rod with the smallest number of hops to a kinematic vertex comes first.
        // Note that we don't use distance because of the bilateral interleaving below.
        var m1 = @min(
            @min(ck[b1_r1.vertex[0]].hops, ck[b1_r1.vertex[1]].hops),
            @min(ck[b1_r2.vertex[0]].hops, ck[b1_r2.vertex[1]].hops),
        );
        var m2 = @min(
            @min(ck[b2_r1.vertex[0]].hops, ck[b2_r1.vertex[1]].hops),
            @min(ck[b2_r2.vertex[0]].hops, ck[b2_r2.vertex[1]].hops),
        );
        if (m1 != m2)
            return m1 < m2;

        // Then sort on the rod that connects to the kinematic vertex with lowest index.
        // This ensures that we consistently order the rods that are attached to other kinematic constraints.
        // Again, this helps bilateral interleaving below.
        m1 = @min(
            @min(ck[b1_r1.vertex[0]].vertex, ck[b1_r1.vertex[1]].vertex),
            @min(ck[b1_r2.vertex[0]].vertex, ck[b1_r2.vertex[1]].vertex),
        );
        m2 = @min(
            @min(ck[b2_r1.vertex[0]].vertex, ck[b2_r1.vertex[1]].vertex),
            @min(ck[b2_r2.vertex[0]].vertex, ck[b2_r2.vertex[1]].vertex),
        );
        if (m1 != m2)
            return m1 < m2;

        // Finally order so that the smallest vertex index goes first
        m1 = @min(b1_r1.getMinVertexIndex(), b1_r2.getMinVertexIndex());
        m2 = @min(b2_r1.getMinVertexIndex(), b2_r2.getMinVertexIndex());
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the dihedral bend constraints (the comparator of Optimize)
    fn dihedralBendLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const b1 = self.dihedral_bend_constraints.items[lhs];
        const b2 = self.dihedral_bend_constraints.items[rhs];
        const ck = self.closest_kinematic.items;

        // First sort so that the constraint with the smallest distance to a kinematic vertex comes first
        const d1 = math.min(
            math.min(ck[b1.vertex[0]].distance, ck[b1.vertex[1]].distance),
            math.min(ck[b1.vertex[2]].distance, ck[b1.vertex[3]].distance),
        );
        const d2 = math.min(
            math.min(ck[b2.vertex[0]].distance, ck[b2.vertex[1]].distance),
            math.min(ck[b2.vertex[2]].distance, ck[b2.vertex[3]].distance),
        );
        if (d1 != d2)
            return d1 < d2;

        // Finally order so that the smallest vertex index goes first
        const m1 = b1.getMinVertexIndex();
        const m2 = b2.getMinVertexIndex();
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the volume constraints (the comparator of Optimize)
    fn volumeLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const v1 = self.volume_constraints.items[lhs];
        const v2 = self.volume_constraints.items[rhs];
        const ck = self.closest_kinematic.items;

        // First sort so that the constraint with the smallest distance to a kinematic vertex comes first
        const d1 = math.min(
            math.min(ck[v1.vertex[0]].distance, ck[v1.vertex[1]].distance),
            math.min(ck[v1.vertex[2]].distance, ck[v1.vertex[3]].distance),
        );
        const d2 = math.min(
            math.min(ck[v2.vertex[0]].distance, ck[v2.vertex[1]].distance),
            math.min(ck[v2.vertex[2]].distance, ck[v2.vertex[3]].distance),
        );
        if (d1 != d2)
            return d1 < d2;

        // Order constraints so that the ones with the smallest index go first
        const m1 = v1.getMinVertexIndex();
        const m2 = v2.getMinVertexIndex();
        if (m1 != m2)
            return m1 < m2;

        return lhs < rhs;
    }

    /// Sort the skinned constraints (the comparator of Optimize)
    fn skinnedLessThan(self: *const SoftBodySharedSettings, lhs: u32, rhs: u32) bool {
        const s1 = self.skinned_constraints.items[lhs];
        const s2 = self.skinned_constraints.items[rhs];

        // Order the skinned constraints so that the ones with the smallest index go first (hoping to get better cache locality when we process the edges).
        if (s1.vertex != s2.vertex)
            return s1.vertex < s2.vertex;

        return lhs < rhs;
    }

    /// `outResults.mXRemap.resize(inSize, ~uint(0))`: the existing elements stay, the new ones are ~0
    fn resizeRemap(allocator: Allocator, remap: *std.ArrayList(u32), size: usize) Allocator.Error!void {
        const old_len = remap.items.len;
        try remap.resize(allocator, size);
        if (size > old_len)
            @memset(remap.items[old_len..], ~@as(u32, 0));
    }

    /// Frees an array of arrays (Array<Array<T>>)
    fn freeLists(comptime T: type, allocator: Allocator, lists: []std.ArrayList(T)) void {
        for (lists) |*list| list.deinit(allocator);
        allocator.free(lists);
    }

    /// Optimize the soft body settings without results
    pub fn optimize(self: *SoftBodySharedSettings) Allocator.Error!void {
        var results: OptimizationResults = .{};
        defer results.deinit(self.allocator);
        try self.optimizeWithResults(&results);
    }

    /// Clone this object (the clone is allocated with `allocator`, the returned Ref holds its only reference)
    pub fn clone(self: *const SoftBodySharedSettings, allocator: Allocator) Allocator.Error!Ref(SoftBodySharedSettings) {
        const result = try create(allocator);
        var ref: Ref(SoftBodySharedSettings) = .init(result);
        errdefer ref.deinit();
        try result.vertices.appendSlice(allocator, self.vertices.items);
        try result.faces.appendSlice(allocator, self.faces.items);
        try result.edge_constraints.appendSlice(allocator, self.edge_constraints.items);
        try result.dihedral_bend_constraints.appendSlice(allocator, self.dihedral_bend_constraints.items);
        try result.volume_constraints.appendSlice(allocator, self.volume_constraints.items);
        try result.skinned_constraints.appendSlice(allocator, self.skinned_constraints.items);
        try result.skinned_constraint_normals.appendSlice(allocator, self.skinned_constraint_normals.items);
        try result.inv_bind_matrices.appendSlice(allocator, self.inv_bind_matrices.items);
        try result.lra_constraints.appendSlice(allocator, self.lra_constraints.items);
        try result.rod_stretch_shear_constraints.appendSlice(allocator, self.rod_stretch_shear_constraints.items);
        try result.rod_bend_twist_constraints.appendSlice(allocator, self.rod_bend_twist_constraints.items);

        // clone->mMaterials = mMaterials (the copy holds a reference to every material)
        try result.materials.ensureTotalCapacityPrecise(allocator, self.materials.items.len);
        for (result.materials.items) |*m| m.deinit();
        result.materials.clearRetainingCapacity();
        for (self.materials.items) |m| result.materials.appendAssumeCapacity(m.clone());

        try result.update_groups.appendSlice(allocator, self.update_groups.items);
        return ref;
    }

    /// Saves the state of this object in binary form to stream. Doesn't store the material list.
    pub fn saveBinaryState(self: *const SoftBodySharedSettings, stream: StreamOut) void {
        stream.writeArray(Vertex, self.vertices.items);
        stream.writeArray(Face, self.faces.items);
        stream.writeArray(Edge, self.edge_constraints.items);
        stream.writeArray(DihedralBend, self.dihedral_bend_constraints.items);
        stream.writeArray(Volume, self.volume_constraints.items);
        stream.writeArray(Skinned, self.skinned_constraints.items);
        stream.writeArray(u32, self.skinned_constraint_normals.items);
        stream.writeArray(LRA, self.lra_constraints.items);
        stream.writeArray(UpdateGroup, self.update_groups.items);

        // Can't write mRodStretchShearConstraints directly because the class contains padding
        stream.writeArrayWith(RodStretchShear, self.rod_stretch_shear_constraints.items, {}, struct {
            fn write(_: void, element: *const RodStretchShear, s: StreamOut) void {
                s.write(element.vertex);
                s.write(element.length);
                s.write(element.inv_mass);
                s.write(element.compliance);
                s.write(element.bishop);
            }
        }.write);

        // Can't write mRodBendTwistConstraints directly because the class contains padding
        stream.writeArrayWith(RodBendTwist, self.rod_bend_twist_constraints.items, {}, struct {
            fn write(_: void, element: *const RodBendTwist, s: StreamOut) void {
                s.write(element.rod);
                s.write(element.compliance);
                s.write(element.omega0);
            }
        }.write);

        // Can't write mInvBindMatrices directly because the class contains padding
        stream.writeArrayWith(InvBind, self.inv_bind_matrices.items, {}, struct {
            fn write(_: void, element: *const InvBind, s: StreamOut) void {
                s.write(element.joint_index);
                s.write(element.inv_bind);
            }
        }.write);
    }

    /// Restore the state of this object from stream. Doesn't restore the material list.
    /// Zolt: the arrays grow with the settings' allocator (`error.OutOfMemory` for a count that cannot be allocated,
    /// where Jolt's resize aborts).
    pub fn restoreBinaryState(self: *SoftBodySharedSettings, stream: StreamIn) Allocator.Error!void {
        const allocator = self.allocator;
        try stream.readArray(Vertex, allocator, &self.vertices);
        try stream.readArray(Face, allocator, &self.faces);
        try stream.readArray(Edge, allocator, &self.edge_constraints);
        try stream.readArray(DihedralBend, allocator, &self.dihedral_bend_constraints);
        try stream.readArray(Volume, allocator, &self.volume_constraints);
        try stream.readArray(Skinned, allocator, &self.skinned_constraints);
        try stream.readArray(u32, allocator, &self.skinned_constraint_normals);
        try stream.readArray(LRA, allocator, &self.lra_constraints);
        try stream.readArray(UpdateGroup, allocator, &self.update_groups);

        try stream.readArrayWith(RodStretchShear, allocator, &self.rod_stretch_shear_constraints, {}, struct {
            fn read(_: void, s: StreamIn, element: *RodStretchShear) Allocator.Error!void {
                s.read(&element.vertex);
                s.read(&element.length);
                s.read(&element.inv_mass);
                s.read(&element.compliance);
                s.read(&element.bishop);
            }
        }.read);

        try stream.readArrayWith(RodBendTwist, allocator, &self.rod_bend_twist_constraints, {}, struct {
            fn read(_: void, s: StreamIn, element: *RodBendTwist) Allocator.Error!void {
                s.read(&element.rod);
                s.read(&element.compliance);
                s.read(&element.omega0);
            }
        }.read);

        try stream.readArrayWith(InvBind, allocator, &self.inv_bind_matrices, {}, struct {
            fn read(_: void, s: StreamIn, element: *InvBind) Allocator.Error!void {
                s.read(&element.joint_index);
                s.read(&element.inv_bind);
            }
        }.read);
    }

    /// Save this shared settings and its materials. Pass in an empty map settings_map / material_map or reuse the same map while saving multiple settings objects to the same stream in order to avoid writing duplicates.
    /// (Zolt: `allocator` grows the maps)
    pub fn saveWithMaterials(self: *const SoftBodySharedSettings, allocator: Allocator, stream: StreamOut, settings_map: *SharedSettingsToIDMap, material_map: *MaterialToIDMap) Allocator.Error!void {
        const settings_iter = settings_map.find(self);
        if (settings_iter == null) {
            // Write settings ID
            const settings_id: u32 = settings_map.count();
            (try settings_map.getOrPutValue(allocator, self, 0)).* = settings_id;
            stream.write(settings_id);

            // Write the settings
            self.saveBinaryState(stream);

            // Write materials
            try StreamUtils.saveObjectArray(PhysicsMaterial, allocator, stream, self.materials.items, material_map);
        } else {
            // Known settings, just write the ID
            stream.write(settings_iter.?.value);
        }
    }

    /// Restore a shape and materials. Pass in an empty map in settings_map / material_map or reuse the same map while reading multiple settings objects from the same stream in order to restore duplicates.
    /// (Jolt: sRestoreWithMaterials. New settings are allocated with `allocator`. The result holds a reference, `deinit`
    /// it. An ID at or above the size of `settings_map` creates new settings, like Jolt: the ID is not checked.)
    pub fn restoreWithMaterials(allocator: Allocator, stream: StreamIn, settings_map: *IDToSharedSettingsMap, material_map: *IDToMaterialMap) Allocator.Error!SettingsResult {
        var result: SettingsResult = .empty;
        errdefer result.deinit();

        // Read settings id
        var settings_id: u32 = 0;
        stream.read(&settings_id);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read settings id");
            return result;
        }

        // Check nullptr settings
        if (settings_id == ~@as(u32, 0)) {
            result.set(.empty);
            return result;
        }

        // Check if we already read this settings
        if (settings_id < settings_map.items.len) {
            result.set(settings_map.items[settings_id].clone());
            return result;
        }

        // Create new object
        var settings = Ref(SoftBodySharedSettings).init(try create(allocator));
        defer settings.deinit();
        const s = settings.get().?;

        // Read state
        try s.restoreBinaryState(stream);

        // Read materials
        var materials: PhysicsMaterialList = .empty;
        defer {
            for (materials.items) |*m| m.deinit();
            materials.deinit(allocator);
        }
        var mlresult = try StreamUtils.restoreObjectArray(PhysicsMaterial, allocator, stream, material_map, &materials);
        defer mlresult.deinit();
        if (mlresult.hasError()) {
            result.setError(mlresult.getError());
            return result;
        }
        Core.swap(PhysicsMaterialList, &s.materials, &materials); // settings->mMaterials = mlresult.Get(), the defer releases the old list

        // Add the settings to the map
        try settings_map.ensureUnusedCapacity(allocator, 1);
        settings_map.appendAssumeCapacity(settings.clone());

        result.set(settings.clone());
        return result;
    }

    /// Create a cube. This can be used to create a simple soft body for testing purposes.
    /// It will contain edge constraints, volume constraints and faces.
    /// @param grid_size Number of points along each axis
    /// @param grid_spacing Distance between points
    pub fn createCube(allocator: Allocator, grid_size: u32, grid_spacing: f32) Allocator.Error!Ref(SoftBodySharedSettings) {
        // Zolt (UB29): Jolt's `inGridSize - 1` wraps for a grid size of 0, and its loops then run until memory runs out
        if (grid_size == 0) @panic("SoftBodySharedSettings.createCube: the grid size is 0");
        const offset = Vec3.replicate(-0.5 * grid_spacing * @as(f32, @floatFromInt(grid_size - 1)));

        // Create settings
        var ref: Ref(SoftBodySharedSettings) = .init(try create(allocator));
        errdefer ref.deinit();
        const settings = ref.get().?;
        for (0..grid_size) |z| {
            for (0..grid_size) |y| {
                for (0..grid_size) |x| {
                    var v: Vertex = .{};
                    offset.add(Vec3.replicate(grid_spacing).mul(Vec3.init(@floatFromInt(x), @floatFromInt(y), @floatFromInt(z)))).storeFloat3(&v.position);
                    try settings.vertices.append(allocator, v);
                }
            }
        }

        // Function to get the vertex index of a point on the cube
        const Grid = struct {
            grid_size: u32,

            fn vertexIndex(g: @This(), x: u32, y: u32, z: u32) u32 {
                return x + y * g.grid_size + z * g.grid_size * g.grid_size;
            }
        };
        const grid: Grid = .{ .grid_size = grid_size };

        // Create edges
        var z: u32 = 0;
        while (z < grid_size) : (z += 1) {
            var y: u32 = 0;
            while (y < grid_size) : (y += 1) {
                var x: u32 = 0;
                while (x < grid_size) : (x += 1) {
                    var e: Edge = .{ .vertex = .{ grid.vertexIndex(x, y, z), undefined } };
                    if (x < grid_size - 1) {
                        e.vertex[1] = grid.vertexIndex(x + 1, y, z);
                        try settings.edge_constraints.append(allocator, e);
                    }
                    if (y < grid_size - 1) {
                        e.vertex[1] = grid.vertexIndex(x, y + 1, z);
                        try settings.edge_constraints.append(allocator, e);
                    }
                    if (z < grid_size - 1) {
                        e.vertex[1] = grid.vertexIndex(x, y, z + 1);
                        try settings.edge_constraints.append(allocator, e);
                    }
                }
            }
        }
        settings.calculateEdgeLengths();

        // Tetrahedrons to fill a cube
        const tetra_indices = [6][4][3]u32{
            .{ .{ 0, 0, 0 }, .{ 0, 1, 1 }, .{ 0, 0, 1 }, .{ 1, 1, 1 } },
            .{ .{ 0, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 1, 1 }, .{ 1, 1, 1 } },
            .{ .{ 0, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 1 }, .{ 1, 1, 1 } },
            .{ .{ 0, 0, 0 }, .{ 1, 0, 1 }, .{ 1, 0, 0 }, .{ 1, 1, 1 } },
            .{ .{ 0, 0, 0 }, .{ 1, 1, 0 }, .{ 0, 1, 0 }, .{ 1, 1, 1 } },
            .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 1, 1, 0 }, .{ 1, 1, 1 } },
        };

        // Create volume constraints
        z = 0;
        while (z < grid_size - 1) : (z += 1) {
            var y: u32 = 0;
            while (y < grid_size - 1) : (y += 1) {
                var x: u32 = 0;
                while (x < grid_size - 1) : (x += 1) {
                    for (tetra_indices) |tetra| {
                        var v: Volume = .{ .vertex = undefined };
                        for (0..4) |i|
                            v.vertex[i] = grid.vertexIndex(x + tetra[i][0], y + tetra[i][1], z + tetra[i][2]);
                        try settings.volume_constraints.append(allocator, v);
                    }
                }
            }
        }

        settings.calculateVolumeConstraintVolumes();

        // Create faces
        const last = grid_size - 1;
        var y: u32 = 0;
        while (y < grid_size - 1) : (y += 1) {
            var x: u32 = 0;
            while (x < grid_size - 1) : (x += 1) {
                var f: Face = .{ .vertex = undefined };

                // Face 1
                f.vertex[0] = grid.vertexIndex(x, y, 0);
                f.vertex[1] = grid.vertexIndex(x, y + 1, 0);
                f.vertex[2] = grid.vertexIndex(x + 1, y + 1, 0);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(x + 1, y + 1, 0);
                f.vertex[2] = grid.vertexIndex(x + 1, y, 0);
                try settings.addFace(f);

                // Face 2
                f.vertex[0] = grid.vertexIndex(x, y, last);
                f.vertex[1] = grid.vertexIndex(x + 1, y + 1, last);
                f.vertex[2] = grid.vertexIndex(x, y + 1, last);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(x + 1, y, last);
                f.vertex[2] = grid.vertexIndex(x + 1, y + 1, last);
                try settings.addFace(f);

                // Face 3
                f.vertex[0] = grid.vertexIndex(x, 0, y);
                f.vertex[1] = grid.vertexIndex(x + 1, 0, y + 1);
                f.vertex[2] = grid.vertexIndex(x, 0, y + 1);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(x + 1, 0, y);
                f.vertex[2] = grid.vertexIndex(x + 1, 0, y + 1);
                try settings.addFace(f);

                // Face 4
                f.vertex[0] = grid.vertexIndex(x, last, y);
                f.vertex[1] = grid.vertexIndex(x, last, y + 1);
                f.vertex[2] = grid.vertexIndex(x + 1, last, y + 1);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(x + 1, last, y + 1);
                f.vertex[2] = grid.vertexIndex(x + 1, last, y);
                try settings.addFace(f);

                // Face 5
                f.vertex[0] = grid.vertexIndex(0, x, y);
                f.vertex[1] = grid.vertexIndex(0, x, y + 1);
                f.vertex[2] = grid.vertexIndex(0, x + 1, y + 1);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(0, x + 1, y + 1);
                f.vertex[2] = grid.vertexIndex(0, x + 1, y);
                try settings.addFace(f);

                // Face 6
                f.vertex[0] = grid.vertexIndex(last, x, y);
                f.vertex[1] = grid.vertexIndex(last, x + 1, y + 1);
                f.vertex[2] = grid.vertexIndex(last, x, y + 1);
                try settings.addFace(f);

                f.vertex[1] = grid.vertexIndex(last, x + 1, y);
                f.vertex[2] = grid.vertexIndex(last, x + 1, y + 1);
                try settings.addFace(f);
            }
        }

        // Optimize the settings
        try settings.optimize();

        return ref;
    }

    /// Add a face to this soft body
    pub fn addFace(self: *SoftBodySharedSettings, face: Face) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(!face.isDegenerate());
        try self.faces.append(self.allocator, face);
    }

    /// Calculate the closest kinematic vertex array (private in Jolt)
    /// The result is cached: it is calculated again only after Optimize freed it (RemainingFeaturesArchitecture.md R20).
    /// Zolt: an allocation failure returns `error.OutOfMemory` and caches nothing.
    pub fn calculateClosestKinematic(self: *SoftBodySharedSettings) Allocator.Error!void {
        // Check if we already calculated this
        if (self.closest_kinematic.items.len != 0)
            return;

        const allocator = self.allocator;
        const num_vertices = self.vertices.items.len;

        // Reserve output size
        try self.closest_kinematic.appendNTimes(allocator, .{}, num_vertices);
        errdefer self.closest_kinematic.clearAndFree(allocator);
        const closest_kinematic = self.closest_kinematic.items;

        // Create a list of connected vertices
        const connectivity = try allocator.alloc(std.ArrayList(u32), num_vertices);
        @memset(connectivity, .empty);
        defer freeLists(u32, allocator, connectivity);
        for (self.edge_constraints.items) |e| {
            try connectivity[e.vertex[0]].append(allocator, e.vertex[1]);
            try connectivity[e.vertex[1]].append(allocator, e.vertex[0]);
        }
        for (self.rod_stretch_shear_constraints.items) |r| {
            try connectivity[r.vertex[0]].append(allocator, r.vertex[1]);
            try connectivity[r.vertex[1]].append(allocator, r.vertex[0]);
        }

        // Use Dijkstra's algorithm to find the closest kinematic vertex for each vertex
        // See: https://en.wikipedia.org/wiki/Dijkstra's_algorithm
        //
        // An element in the open list
        const Open = struct {
            vertex: u32,
            distance: f32,

            /// Order so that we get the shortest distance first (operator <)
            fn lessThan(_: void, lhs: @This(), rhs: @This()) bool {
                return lhs.distance > rhs.distance;
            }
        };

        // Start with all kinematic elements
        var to_visit: std.ArrayList(Open) = .empty;
        defer to_visit.deinit(allocator);
        var v: u32 = 0;
        while (v < @as(u32, @intCast(num_vertices))) : (v += 1) {
            if (self.vertices.items[v].inv_mass == 0.0) {
                closest_kinematic[v].vertex = v;
                closest_kinematic[v].hops = 0;
                closest_kinematic[v].distance = 0.0;
                try to_visit.append(allocator, .{ .vertex = v, .distance = 0.0 });
                binaryHeapPush(Open, to_visit.items, {}, Open.lessThan);
            }
        }

        // Visit all vertices remembering the closest kinematic vertex and its distance
        var last_closest: f32 = 0.0; // JPH_IF_ENABLE_ASSERTS
        while (to_visit.items.len != 0) {
            // Pop element from the open list
            binaryHeapPop(Open, to_visit.items, {}, Open.lessThan);
            const current = to_visit.pop().?;
            if (Core.enable_asserts) {
                std.debug.assert(current.distance >= last_closest);
                last_closest = current.distance;
            }

            // Loop through all of its connected vertices
            for (connectivity[current.vertex].items) |target| {
                // Calculate distance from the current vertex to this target vertex and check if it is smaller
                const new_distance = current.distance + self.vertexPosition(target).sub(self.vertexPosition(current.vertex)).length();
                if (new_distance < closest_kinematic[target].distance) {
                    // Remember new closest vertex
                    closest_kinematic[target].vertex = closest_kinematic[current.vertex].vertex;
                    closest_kinematic[target].hops = closest_kinematic[current.vertex].hops +% 1;
                    closest_kinematic[target].distance = new_distance;
                    try to_visit.append(allocator, .{ .vertex = target, .distance = new_distance });
                    binaryHeapPush(Open, to_visit.items, {}, Open.lessThan);
                }
            }
        }
    }
};

/// The bytes of saveBinaryState (a test helper)
fn saveBytes(allocator: Allocator, settings: *const SoftBodySharedSettings) ![]u8 {
    const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;
    var buffer: std.Io.Writer.Allocating = .init(allocator);
    defer buffer.deinit();
    var wrapper: StreamOutWrapper = .init(&buffer.writer);
    settings.saveBinaryState(wrapper.streamOut());
    if (wrapper.streamOut().isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
    return buffer.toOwnedSlice();
}

/// Settings with one element in every array, with values that differ per field
fn fillAllArrays(settings: *SoftBodySharedSettings) Allocator.Error!void {
    const S = SoftBodySharedSettings;
    const a = settings.allocator;
    try settings.vertices.append(a, .init(.{ .x = 1, .y = 2, .z = 3 }, .{ .velocity = .{ .x = 4, .y = 5, .z = 6 }, .inv_mass = 0.5 }));
    try settings.vertices.append(a, .init(.{ .x = -1, .y = 0, .z = 7 }, .{}));
    try settings.addFace(.init(0, 1, 2, .{ .material_index = 3 }));
    try settings.edge_constraints.append(a, .init(0, 1, .{ .compliance = 0.25 }));
    try settings.dihedral_bend_constraints.append(a, .init(0, 1, 2, 3, .{ .compliance = 0.125 }));
    try settings.volume_constraints.append(a, .init(4, 5, 6, 7, .{ .compliance = 2.0 }));
    var skinned: S.Skinned = .init(1, 0.5, 0.25, 9.0);
    skinned.weights[0] = .init(0, 3.0);
    skinned.weights[1] = .init(1, 1.0);
    skinned.normalizeWeights();
    skinned.normal_info = 0x02000003;
    try settings.skinned_constraints.append(a, skinned);
    try settings.skinned_constraint_normals.appendSlice(a, &.{ 7, 8, 9 });
    try settings.inv_bind_matrices.append(a, .init(2, Mat44.scale(2.0)));
    try settings.lra_constraints.append(a, .init(1, 0, 1.5));
    var rod: S.RodStretchShear = .init(0, 1, .{ .compliance = 0.75 });
    rod.bishop = Quat.init(0, 0, 1, 0);
    rod.length = 2.0;
    try settings.rod_stretch_shear_constraints.append(a, rod);
    var bend: S.RodBendTwist = .init(0, 0, .{ .compliance = 1.5 });
    bend.omega0 = Quat.identity();
    try settings.rod_bend_twist_constraints.append(a, bend);
    try settings.update_groups.append(a, .{ .edge_end_index = 1, .lra_end_index = 1, .rod_stretch_shear_end_index = 1, .rod_bend_twist_end_index = 1, .dihedral_bend_end_index = 1, .volume_end_index = 1, .skinned_end_index = 1 });
}

test "SoftBodySharedSettings: layouts and defaults" {
    const S = SoftBodySharedSettings;
    const expectEqual = std.testing.expectEqual;

    // The raw layouts that SaveBinaryState writes (Jolt's sizeof), and the offsets inside them
    try expectEqual(@as(usize, 28), @sizeOf(S.Vertex));
    try expectEqual(@as(usize, 16), @sizeOf(S.Face));
    try expectEqual(@as(usize, 16), @sizeOf(S.Edge));
    try expectEqual(@as(usize, 24), @sizeOf(S.DihedralBend));
    try expectEqual(@as(usize, 24), @sizeOf(S.Volume));
    try expectEqual(@as(usize, 52), @sizeOf(S.Skinned));
    try expectEqual(@as(usize, 12), @sizeOf(S.LRA));
    try expectEqual(@as(usize, 28), @sizeOf(S.UpdateGroup));
    try expectEqual(@as(usize, 8), @sizeOf(S.SkinWeight));
    try expectEqual(@as(usize, 4), @offsetOf(S.Skinned, "weights"));
    try expectEqual(@as(usize, 48), @offsetOf(S.Skinned, "normal_info"));
    try expectEqual(@as(usize, 20), @offsetOf(S.Volume, "compliance"));

    // Jolt's defaults
    const vertex: S.Vertex = .{};
    try expectEqual(@as(f32, 1.0), vertex.inv_mass);
    const skinned: S.Skinned = .{};
    try expectEqual(math.flt_max, skinned.max_distance);
    try expectEqual(math.flt_max, skinned.back_stop_distance);
    try expectEqual(@as(f32, 40.0), skinned.back_stop_radius);
    try expectEqual(@as(f32, 0.0), skinned.weights[3].weight);
    const attributes: S.VertexAttributes = .{};
    try expectEqual(math.flt_max, attributes.bend_compliance);
    try expectEqual(S.LRAType.none, attributes.lra_type);
    const closest: S.ClosestKinematic = .{};
    try expectEqual(@as(u32, 0xffffffff), closest.vertex);
    try expectEqual(math.flt_max, closest.distance);
    try std.testing.expect(Quat.zero().eql((S.RodStretchShear{ .vertex = .{ 0, 1 } }).bishop));
    try expectEqual(@as(u32, 2), S.Edge.init(3, 2, .{}).getMinVertexIndex());
    try expectEqual(@as(u32, 1), S.DihedralBend.init(4, 3, 1, 2, .{}).getMinVertexIndex());
    try std.testing.expect(S.Face.init(1, 2, 1, .{}).isDegenerate());
    try std.testing.expect(!S.Face.init(1, 2, 3, .{}).isDegenerate());

    // Skinned::NormalizeWeights: the weights add up to 1, zero weights stay zero
    var s: S.Skinned = .{};
    s.weights[0] = .init(0, 2.0);
    s.weights[1] = .init(1, 6.0);
    s.normalizeWeights();
    try expectEqual(@as(f32, 0.25), s.weights[0].weight);
    try expectEqual(@as(f32, 0.75), s.weights[1].weight);
    try expectEqual(@as(f32, 0.0), s.weights[2].weight);

    // The settings start with Jolt's default material
    var settings: S = try .init(std.testing.allocator);
    defer settings.deinit();
    try expectEqual(@as(usize, 1), settings.materials.items.len);
    try std.testing.expect(settings.materials.items[0].get() == PhysicsMaterial.default);
}

test "SoftBodySharedSettings: binary state round trip, clone and references" {
    const allocator = std.testing.allocator;
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;

    var settings: Ref(SoftBodySharedSettings) = .init(try SoftBodySharedSettings.create(allocator));
    defer settings.deinit();
    try std.testing.expectEqual(@as(u32, 1), settings.get().?.getRefCount());
    try fillAllArrays(settings.get().?);

    const bytes = try saveBytes(allocator, settings.get().?);
    defer allocator.free(bytes);

    // 9 raw arrays (count + bytes), then the 3 arrays written field by field
    const expected_size = (4 + 2 * 28) + (4 + 16) + (4 + 16) + (4 + 24) + (4 + 24) + (4 + 52) + (4 + 3 * 4) + (4 + 12) + (4 + 28) + (4 + 36) + (4 + 28) + (4 + 68);
    try std.testing.expectEqual(@as(usize, expected_size), bytes.len);

    // Read into settings that already hold other data: every array gets the stream's size
    var restored: SoftBodySharedSettings = try .init(allocator);
    defer restored.deinit();
    for (0..3) |_| try restored.vertices.append(allocator, .{});
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamInWrapper = .init(&reader);
    try restored.restoreBinaryState(wrapper.streamIn());
    try std.testing.expect(!wrapper.streamIn().isFailed());
    const again = try saveBytes(allocator, &restored);
    defer allocator.free(again);
    try std.testing.expectEqualSlices(u8, bytes, again);

    // Clone copies everything but the closest kinematic vertices, and adds references to the materials
    const s = settings.get().?;
    try s.closest_kinematic.append(allocator, .{});
    var copy = try s.clone(allocator);
    defer copy.deinit();
    try std.testing.expectEqual(@as(u32, 1), copy.get().?.getRefCount());
    try std.testing.expectEqual(@as(usize, 0), copy.get().?.closest_kinematic.items.len);
    try std.testing.expectEqual(@as(usize, 1), copy.get().?.materials.items.len);
    const cloned = try saveBytes(allocator, copy.get().?);
    defer allocator.free(cloned);
    try std.testing.expectEqualSlices(u8, bytes, cloned);

    // A truncated stream fails and leaves no leak
    for (0..bytes.len) |n| {
        var short_reader: std.Io.Reader = .fixed(bytes[0..n]);
        var short: StreamInWrapper = .init(&short_reader);
        var target: SoftBodySharedSettings = try .init(allocator);
        defer target.deinit();
        try target.restoreBinaryState(short.streamIn());
        try std.testing.expect(short.streamIn().isEOF() or short.streamIn().isFailed());
    }
}

test "SoftBodySharedSettings: out of memory in create, clone and restoreBinaryState" {
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            const settings = try SoftBodySharedSettings.create(allocator);
            var ref: Ref(SoftBodySharedSettings) = .init(settings);
            defer ref.deinit();
            try fillAllArrays(settings);
            var copy = try settings.clone(allocator);
            defer copy.deinit();
            const bytes = try saveBytes(allocator, settings);
            defer allocator.free(bytes);
            var reader: std.Io.Reader = .fixed(bytes);
            var wrapper: StreamInWrapper = .init(&reader);
            try copy.get().?.restoreBinaryState(wrapper.streamIn());
        }
    }.f, .{});
}

/// The maps of a restore with materials, released by deinit (a test helper)
const WithMaterialsRestoreMaps = struct {
    settings: SoftBodySharedSettings.IDToSharedSettingsMap = .empty,
    materials: SoftBodySharedSettings.IDToMaterialMap = .empty,

    fn deinit(self: *WithMaterialsRestoreMaps, allocator: Allocator) void {
        for (self.settings.items) |*s| s.deinit();
        self.settings.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
    }
};

/// Two settings that share a material, the first also has the default material and a null material, written with
/// saveWithMaterials as settings 0, 1 and 0 again (a test helper, returns the bytes)
fn saveWithMaterialsBytes(allocator: Allocator, settings: *[2]Ref(SoftBodySharedSettings)) ![]u8 {
    const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;
    const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
    const Color = @import("../../Core/Color.zig").Color;

    for (settings) |*s| s.* = .init(try SoftBodySharedSettings.create(allocator));
    const rubber = try PhysicsMaterialSimple.create(allocator, "Rubber", Color.init(1, 2, 3, 4));
    var rubber_ref: RefConst(PhysicsMaterial) = .init(rubber.material());
    defer rubber_ref.deinit();
    const ice = try PhysicsMaterialSimple.create(allocator, "Ice", Color.init(5, 6, 7, 8));
    var ice_ref: RefConst(PhysicsMaterial) = .init(ice.material());
    defer ice_ref.deinit();
    const s0 = settings[0].get().?;
    try fillAllArrays(s0);
    try s0.materials.ensureUnusedCapacity(allocator, 2);
    s0.materials.appendAssumeCapacity(rubber_ref.clone());
    s0.materials.appendAssumeCapacity(.empty);
    const s1 = settings[1].get().?;
    try s1.vertices.append(allocator, .init(.{ .x = 3, .y = 2, .z = 1 }, .{}));
    try s1.materials.ensureUnusedCapacity(allocator, 1);
    s1.materials.items[0].deinit();
    s1.materials.items[0] = ice_ref.clone();
    s1.materials.appendAssumeCapacity(rubber_ref.clone());

    var buffer: std.Io.Writer.Allocating = .init(allocator);
    defer buffer.deinit();
    var wrapper: StreamOutWrapper = .init(&buffer.writer);
    var settings_map: SoftBodySharedSettings.SharedSettingsToIDMap = .empty;
    defer settings_map.deinit(allocator);
    var material_map: SoftBodySharedSettings.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    for ([_]usize{ 0, 1, 0 }) |i|
        try settings[i].get().?.saveWithMaterials(allocator, wrapper.streamOut(), &settings_map, &material_map);
    if (wrapper.streamOut().isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
    try std.testing.expectEqual(@as(u32, 2), settings_map.count());
    try std.testing.expectEqual(@as(u32, 3), material_map.count()); // The default, rubber and ice (null is not a material)
    return buffer.toOwnedSlice();
}

/// restoreWithMaterials of the 3 settings that saveWithMaterialsBytes wrote (a test helper)
fn restoreWithMaterialsBytes(allocator: Allocator, bytes: []const u8, maps: *WithMaterialsRestoreMaps, out: *[3]Ref(SoftBodySharedSettings)) !void {
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamInWrapper = .init(&reader);
    for (out) |*o| {
        var result = try SoftBodySharedSettings.restoreWithMaterials(allocator, wrapper.streamIn(), &maps.settings, &maps.materials);
        defer result.deinit();
        if (!result.isValid()) return error.TestUnexpectedResult;
        o.set(result.getPtr());
    }
}

test "SoftBodySharedSettings: saveWithMaterials / restoreWithMaterials" {
    const allocator = std.testing.allocator;
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
    var settings: [2]Ref(SoftBodySharedSettings) = @splat(.empty);
    defer for (&settings) |*s| s.deinit();
    const bytes = try saveWithMaterialsBytes(allocator, &settings);
    defer allocator.free(bytes);

    // Settings 0 again is its ID only: the bytes end with it
    try std.testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, bytes[bytes.len - 4 ..][0..4], .little));

    var maps: WithMaterialsRestoreMaps = .{};
    defer maps.deinit(allocator);
    var restored: [3]Ref(SoftBodySharedSettings) = @splat(.empty);
    defer for (&restored) |*r| r.deinit();
    try restoreWithMaterialsBytes(allocator, bytes, &maps, &restored);

    // The sharing: the third settings are the first, the material is shared, the null material stays null
    try std.testing.expectEqual(@as(usize, 2), maps.settings.items.len);
    try std.testing.expectEqual(@as(usize, 3), maps.materials.items.len);
    const r0 = restored[0].get().?;
    const r1 = restored[1].get().?;
    try std.testing.expect(restored[2].get().? == r0);
    try std.testing.expect(r0 != settings[0].get().?);
    try std.testing.expectEqual(@as(usize, 3), r0.materials.items.len);
    try std.testing.expect(r0.materials.items[0].get().? != PhysicsMaterial.default); // A new PhysicsMaterial, like Jolt
    try std.testing.expectEqualStrings("Rubber", r0.materials.items[1].get().?.getDebugName());
    try std.testing.expect(r0.materials.items[2].get() == null);
    try std.testing.expect(r1.materials.items[1].get().? == r0.materials.items[1].get().?);
    try std.testing.expectEqualStrings("Ice", r1.materials.items[0].get().?.getDebugName());
    for ([_]*const SoftBodySharedSettings{ r0, r1 }, &settings) |r, *original| {
        const expected = try saveBytes(allocator, original.get().?);
        defer allocator.free(expected);
        const actual = try saveBytes(allocator, r);
        defer allocator.free(actual);
        try std.testing.expectEqualSlices(u8, expected, actual);
    }

    // The null ID gives no settings
    {
        const null_id = [_]u8{ 0xff, 0xff, 0xff, 0xff };
        var reader: std.Io.Reader = .fixed(&null_id);
        var wrapper: StreamInWrapper = .init(&reader);
        var result = try SoftBodySharedSettings.restoreWithMaterials(allocator, wrapper.streamIn(), &maps.settings, &maps.materials);
        defer result.deinit();
        try std.testing.expect(result.isValid() and result.getPtr() == null);
    }

    // An ID at or above the size of the map creates new settings (Jolt does not check the ID)
    {
        var new_maps: WithMaterialsRestoreMaps = .{};
        defer new_maps.deinit(allocator);
        const moved = try allocator.dupe(u8, bytes);
        defer allocator.free(moved);
        std.mem.writeInt(u32, moved[0..4], 7, .little);
        var reader: std.Io.Reader = .fixed(moved);
        var wrapper: StreamInWrapper = .init(&reader);
        var result = try SoftBodySharedSettings.restoreWithMaterials(allocator, wrapper.streamIn(), &new_maps.settings, &new_maps.materials);
        defer result.deinit();
        try std.testing.expect(result.isValid() and result.getPtr() == new_maps.settings.items[0].get().?);
        try std.testing.expectEqual(@as(usize, 1), new_maps.settings.items.len);
    }

    // Truncated streams: Jolt's texts, no leak
    for (0..bytes.len) |len| {
        var new_maps: WithMaterialsRestoreMaps = .{};
        defer new_maps.deinit(allocator);
        var reader: std.Io.Reader = .fixed(bytes[0..len]);
        var wrapper: StreamInWrapper = .init(&reader);
        var failed = false;
        for (0..3) |_| {
            var result = try SoftBodySharedSettings.restoreWithMaterials(allocator, wrapper.streamIn(), &new_maps.settings, &new_maps.materials);
            defer result.deinit();
            if (result.hasError()) {
                if (len < 4) try std.testing.expectEqualStrings("Failed to read settings id", result.getError());
                failed = true;
                break;
            }
        }
        try std.testing.expect(failed);
    }
}

test "SoftBodySharedSettings: out of memory in saveWithMaterials / restoreWithMaterials" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            var settings: [2]Ref(SoftBodySharedSettings) = @splat(.empty);
            defer for (&settings) |*s| s.deinit();
            const bytes = try saveWithMaterialsBytes(allocator, &settings);
            defer allocator.free(bytes);
            var maps: WithMaterialsRestoreMaps = .{};
            defer maps.deinit(allocator);
            var restored: [3]Ref(SoftBodySharedSettings) = @splat(.empty);
            defer for (&restored) |*r| r.deinit();
            try restoreWithMaterialsBytes(allocator, bytes, &maps, &restored);
        }
    }.f, .{});
}

/// The cloth of SoftBodyCreator::CreateClothWithFixatedCorners (Samples/Utils/SoftBodyCreator.cpp): a grid in the XZ
/// plane, the 4 corners kinematic, 2 faces per quad (a test helper)
fn createTestCloth(allocator: Allocator, grid_size: u32, spacing: f32) Allocator.Error!Ref(SoftBodySharedSettings) {
    var ref: Ref(SoftBodySharedSettings) = .init(try SoftBodySharedSettings.create(allocator));
    errdefer ref.deinit();
    const s = ref.get().?;
    const offset = -0.5 * spacing * @as(f32, @floatFromInt(grid_size - 1));
    for (0..grid_size) |z| for (0..grid_size) |x| {
        var v: SoftBodySharedSettings.Vertex = .{};
        Vec3.init(offset + @as(f32, @floatFromInt(x)) * spacing, 0.0, offset + @as(f32, @floatFromInt(z)) * spacing).storeFloat3(&v.position);
        const corner = (x == 0 or x == grid_size - 1) and (z == 0 or z == grid_size - 1);
        v.inv_mass = if (corner) 0.0 else 1.0;
        try s.vertices.append(allocator, v);
    };
    var z: u32 = 0;
    while (z < grid_size - 1) : (z += 1) {
        var x: u32 = 0;
        while (x < grid_size - 1) : (x += 1) {
            try s.addFace(.init(x + z * grid_size, x + (z + 1) * grid_size, x + 1 + (z + 1) * grid_size, .{}));
            try s.addFace(.init(x + z * grid_size, x + 1 + (z + 1) * grid_size, x + 1 + z * grid_size, .{}));
        }
    }
    return ref;
}

/// The surface of a tetrahedron, without constraints (a test helper)
fn createTestTetrahedron(allocator: Allocator) Allocator.Error!Ref(SoftBodySharedSettings) {
    var ref: Ref(SoftBodySharedSettings) = .init(try SoftBodySharedSettings.create(allocator));
    errdefer ref.deinit();
    const s = ref.get().?;
    for ([_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 1, 0 } }) |p|
        try s.vertices.append(allocator, .init(.init(p[0], p[1], p[2]), .{}));
    for ([_][3]u32{ .{ 0, 1, 2 }, .{ 0, 3, 1 }, .{ 0, 2, 3 }, .{ 1, 3, 2 } }) |f|
        try s.addFace(.init(f[0], f[1], f[2], .{}));
    return ref;
}

/// True when `remap` maps 0..n-1 onto 0..n-1
fn isPermutation(allocator: Allocator, remap: []const u32) !bool {
    const seen = try allocator.alloc(bool, remap.len);
    defer allocator.free(seen);
    @memset(seen, false);
    for (remap) |r| {
        if (r >= remap.len or seen[r]) return false;
        seen[r] = true;
    }
    return true;
}

test "SoftBodySharedSettings: createConstraints adds the last shared edge of a closed mesh twice" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // Without bend constraints: the 6 edges of the tetrahedron, the last one twice (Jolt's quirk, R20)
    var no_bends = try createTestTetrahedron(allocator);
    defer no_bends.deinit();
    const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, math.flt_max, .{})};
    try no_bends.get().?.createConstraints(&attributes, .{ .bend_type = .none });
    const edges = no_bends.get().?.edge_constraints.items;
    try std.testing.expectEqual(@as(usize, 7), edges.len);
    try std.testing.expectEqual(edges[5].vertex, edges[6].vertex);
    for (edges) |e| try std.testing.expect(e.rest_length > 0.0);

    // With distance bends: a bend edge per shared edge, the duplicate adds no bend
    var distance = try createTestTetrahedron(allocator);
    defer distance.deinit();
    const bend_attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{})};
    try distance.get().?.createConstraints(&bend_attributes, .{ .bend_type = .distance });
    try std.testing.expectEqual(@as(usize, 13), distance.get().?.edge_constraints.items.len);

    // Dihedral bends: one per shared edge, all with an initial angle
    var dihedral = try createTestTetrahedron(allocator);
    defer dihedral.deinit();
    try dihedral.get().?.createConstraints(&bend_attributes, .{ .bend_type = .dihedral });
    try std.testing.expectEqual(@as(usize, 7), dihedral.get().?.edge_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 6), dihedral.get().?.dihedral_bend_constraints.items.len);
    for (dihedral.get().?.dihedral_bend_constraints.items) |b| try std.testing.expect(b.initial_angle != 0.0);
}

test "SoftBodySharedSettings: createCube" {
    // sCreateCube(6, ...): 216 vertices, 300 faces, 540 edges, 750 volumes. Optimize puts all vertices in parallel group
    // 0, the non-parallel group (the last one) is empty. These are Jolt's values (SoftBodyParity.zig pins them).
    var cube = try SoftBodySharedSettings.createCube(std.testing.allocator, 6, 0.2);
    defer cube.deinit();
    const s = cube.get().?;
    try std.testing.expectEqual(@as(usize, 216), s.vertices.items.len);
    try std.testing.expectEqual(@as(usize, 300), s.faces.items.len);
    try std.testing.expectEqual(@as(usize, 540), s.edge_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 750), s.volume_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 2), s.update_groups.items.len);
    try std.testing.expectEqual(s.update_groups.items[0], s.update_groups.items[1]);
    try std.testing.expectEqual(@as(u32, 540), s.update_groups.items[0].edge_end_index);
    try std.testing.expectEqual(@as(u32, 750), s.update_groups.items[0].volume_end_index);
    try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len); // Optimize frees it
    for (s.edge_constraints.items) |e| try std.testing.expectApproxEqAbs(@as(f32, 0.2), e.rest_length, 1.0e-6);
    for (s.volume_constraints.items) |v| try std.testing.expectApproxEqAbs(@as(f32, 0.008), v.six_rest_volume, 1.0e-6);
}

test "SoftBodySharedSettings: createCube with a grid size of 1 (UB29)" {
    // A grid size of 1 is literal: one vertex at the origin and no constraints. A grid size of 0 panics (UB29: Jolt's
    // `inGridSize - 1` wraps), which a test cannot catch.
    var cube = try SoftBodySharedSettings.createCube(std.testing.allocator, 1, 0.2);
    defer cube.deinit();
    const s = cube.get().?;
    try std.testing.expectEqual(@as(usize, 1), s.vertices.items.len);
    try std.testing.expect(Vec3.loadFloat3Unsafe(&s.vertices.items[0].position).eql(Vec3.zero()));
    try std.testing.expectEqual(@as(usize, 0), s.faces.items.len);
    try std.testing.expectEqual(@as(usize, 0), s.edge_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 0), s.volume_constraints.items.len);
}

test "SoftBodySharedSettings: cloth with dihedral bends and LRA, optimize" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // A 10 x 10 cloth: 162 faces, 342 edges (stretch and shear), 225 dihedral bends, LRA constraints for the 96 movable
    // vertices, too few vertices for a parallel group (Jolt's values, SoftBodyParity.zig pins them)
    var cloth = try createTestCloth(allocator, 10, 0.1);
    defer cloth.deinit();
    const s = cloth.get().?;
    const attributes = [_]S.VertexAttributes{.init(1.0e-5, 1.0e-5, 1.0e-5, .{ .lra_type = .euclidean_distance, .lra_max_distance_multiplier = 1.1 })};
    try s.createConstraints(&attributes, .{ .bend_type = .dihedral });
    try std.testing.expectEqual(@as(usize, 342), s.edge_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 225), s.dihedral_bend_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 96), s.lra_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 100), s.closest_kinematic.items.len); // Cached for Optimize

    var results: S.OptimizationResults = .{};
    defer results.deinit(allocator);
    try s.optimizeWithResults(&results);
    try std.testing.expectEqual(@as(usize, 1), s.update_groups.items.len);
    try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len);
    try std.testing.expect(try isPermutation(allocator, results.edge_remap.items));
    try std.testing.expect(try isPermutation(allocator, results.lra_remap.items));
    try std.testing.expect(try isPermutation(allocator, results.dihedral_bend_remap.items));
    try std.testing.expectEqual(@as(usize, 342), results.edge_remap.items.len);

    // The LRA constraints are sorted from long to short
    const lras = s.lra_constraints.items;
    for (lras[0 .. lras.len - 1], lras[1..]) |a, b| try std.testing.expect(a.max_distance >= b.max_distance);
}

test "SoftBodySharedSettings: optimize makes parallel groups for a large cloth" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // A 30 x 30 cloth has more vertices than a group holds: several parallel groups and a non-parallel group
    var cloth = try createTestCloth(allocator, 30, 0.1);
    defer cloth.deinit();
    const s = cloth.get().?;
    const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{ .lra_type = .geodesic_distance })};
    try s.createConstraints(&attributes, .{ .bend_type = .distance });
    const num_edges = s.edge_constraints.items.len;
    try s.optimize();
    const groups = s.update_groups.items;
    try std.testing.expect(groups.len >= 4);
    try std.testing.expectEqual(@as(u32, @intCast(num_edges)), groups[groups.len - 1].edge_end_index);
    try std.testing.expect(groups[groups.len - 1].edge_end_index > groups[groups.len - 2].edge_end_index); // Non-parallel edges

    // The end indices grow, every group is a range
    for (groups[0 .. groups.len - 1], groups[1..]) |a, b| {
        try std.testing.expect(a.edge_end_index <= b.edge_end_index);
        try std.testing.expect(a.lra_end_index <= b.lra_end_index);
    }
}

test "SoftBodySharedSettings: rods, bend twist constraints and the remap of their rods" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // A chain of 4 rods along a bent line, the second rod points backwards, vertex 0 is kinematic
    var settings: S = try .init(allocator);
    defer settings.deinit();
    const s = &settings;
    for ([_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 2, 0.5, 0 }, .{ 3, 0.5, 0.5 }, .{ 4, 1, 0.5 } }, 0..) |p, i|
        try s.vertices.append(allocator, .init(.init(p[0], p[1], p[2]), .{ .inv_mass = if (i == 0) 0.0 else 1.0 }));
    try s.rod_stretch_shear_constraints.append(allocator, .init(0, 1, .{}));
    try s.rod_stretch_shear_constraints.append(allocator, .init(2, 1, .{})); // Opposite direction
    try s.rod_stretch_shear_constraints.append(allocator, .init(2, 3, .{}));
    try s.rod_stretch_shear_constraints.append(allocator, .init(3, 4, .{}));
    for (0..3) |i| try s.rod_bend_twist_constraints.append(allocator, .init(@intCast(i), @intCast(i + 1), .{}));
    try s.calculateRodProperties();

    // The second rod was flipped to follow the first one
    try std.testing.expectEqual([2]u32{ 1, 2 }, s.rod_stretch_shear_constraints.items[1].vertex);
    for (s.rod_stretch_shear_constraints.items) |r| {
        try std.testing.expect(!r.bishop.eql(Quat.zero()));
        try std.testing.expect(r.bishop.isNormalized(.{}));
        try std.testing.expect(r.length > 0.0);
    }
    try std.testing.expectEqual(@as(f32, 0.0), s.rod_stretch_shear_constraints.items[0].inv_mass);
    try std.testing.expectEqual(@as(f32, 1.0), s.rod_stretch_shear_constraints.items[1].inv_mass);
    for (s.rod_bend_twist_constraints.items) |b| try std.testing.expect(b.omega0.isNormalized(.{}));

    // Optimize reorders the rods and remaps the rods of the bend twist constraints
    const old_rods = try allocator.dupe(S.RodStretchShear, s.rod_stretch_shear_constraints.items);
    defer allocator.free(old_rods);
    const old_bends = try allocator.dupe(S.RodBendTwist, s.rod_bend_twist_constraints.items);
    defer allocator.free(old_bends);
    var results: S.OptimizationResults = .{};
    defer results.deinit(allocator);
    try s.optimizeWithResults(&results);
    for (old_rods, 0..) |r, i|
        try std.testing.expectEqual(r.vertex, s.rod_stretch_shear_constraints.items[results.rod_stretch_shear_constraint_remap.items[i]].vertex);
    for (old_bends, 0..) |b, i| {
        const new = s.rod_bend_twist_constraints.items[results.rod_bend_twist_constraint_remap.items[i]];
        for (0..2) |k| try std.testing.expectEqual(results.rod_stretch_shear_constraint_remap.items[b.rod[k]], new.rod[k]);
    }
}

test "SoftBodySharedSettings: calculateSkinnedConstraintNormals" {
    const allocator = std.testing.allocator;

    // A 3 x 3 cloth (8 faces), every vertex but vertex 8 skinned
    var cloth = try createTestCloth(allocator, 3, 1.0);
    defer cloth.deinit();
    const s = cloth.get().?;
    for (0..8) |v| try s.skinned_constraints.append(allocator, .init(@intCast(v), 0.1, 0.0, 1.0));
    try s.calculateSkinnedConstraintNormals();

    // The faces that use only skinned vertices: the faces without vertex 8, sorted per vertex
    for (s.skinned_constraints.items) |c| {
        const start = c.normal_info & 0xffffff;
        const num = c.normal_info >> 24;
        const faces = s.skinned_constraint_normals.items[start .. start + num];
        for (faces) |f| {
            try std.testing.expect(std.mem.indexOfScalar(u32, &s.faces.items[f].vertex, c.vertex) != null);
            try std.testing.expect(std.mem.indexOfScalar(u32, &s.faces.items[f].vertex, 8) == null);
        }
        if (faces.len > 1)
            for (faces[0 .. faces.len - 1], faces[1..]) |a, b| try std.testing.expect(a < b);
    }
    try std.testing.expectEqual(@as(u32, 2), s.skinned_constraints.items[0].normal_info >> 24); // Vertex 0: both faces of its quad
    try std.testing.expectEqual(@as(u32, 1), s.skinned_constraints.items[2].normal_info >> 24); // Vertex 2: 1 face
    try std.testing.expectEqual(@as(u32, 4), s.skinned_constraints.items[4].normal_info >> 24); // Vertex 4: 4 of its 6 faces

    // A second call gives the same result
    const first = try allocator.dupe(u32, s.skinned_constraint_normals.items);
    defer allocator.free(first);
    try s.calculateSkinnedConstraintNormals();
    try std.testing.expectEqualSlices(u32, first, s.skinned_constraint_normals.items);
}

test "SoftBodySharedSettings: out of memory in the algorithms" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            const S = SoftBodySharedSettings;
            var cloth = try createTestCloth(allocator, 4, 0.5);
            defer cloth.deinit();
            const s = cloth.get().?;
            const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{ .lra_type = .geodesic_distance })};
            try s.createConstraints(&attributes, .{ .bend_type = .dihedral });
            for (0..4) |v| try s.skinned_constraints.append(allocator, .init(@intCast(v), 0.1, 0.0, 1.0));
            try s.calculateSkinnedConstraintNormals();
            try s.rod_stretch_shear_constraints.append(allocator, .init(1, 2, .{}));
            try s.rod_stretch_shear_constraints.append(allocator, .init(2, 3, .{}));
            try s.rod_bend_twist_constraints.append(allocator, .init(0, 1, .{}));
            try s.calculateRodProperties();
            try s.optimize();
            var cube = try S.createCube(allocator, 3, 0.5);
            defer cube.deinit();
        }
    }.f, .{});
}

test "SoftBodySharedSettings: optimize leaves the settings unchanged when an allocation fails" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var cloth = try createTestCloth(allocator, 20, 0.1);
        defer cloth.deinit();
        const s = cloth.get().?;
        const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{ .lra_type = .euclidean_distance })};
        try s.createConstraints(&attributes, .{ .bend_type = .dihedral });
        for (0..10) |v| try s.skinned_constraints.append(allocator, .init(@intCast(v * 7), 0.1, 0.0, 1.0));
        // An old update group, which optimize replaces with more groups: a failure must keep it
        try s.update_groups.ensureTotalCapacityPrecise(allocator, 1);
        s.update_groups.appendAssumeCapacity(std.mem.zeroes(S.UpdateGroup));
        const before = try saveBytes(allocator, s);
        defer allocator.free(before);
        const num_closest = s.closest_kinematic.items.len;

        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        s.allocator = failing.allocator();
        var results: S.OptimizationResults = .{};
        const result = s.optimizeWithResults(&results);
        s.allocator = allocator;
        results.deinit(allocator);
        if (result) |_| {
            try std.testing.expect(fail_index > 10);
            try std.testing.expect(s.update_groups.items.len > 1); // So the old update group array had to grow
            break;
        } else |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            const after = try saveBytes(allocator, s);
            defer allocator.free(after);
            try std.testing.expectEqualSlices(u8, before, after);
            try std.testing.expectEqual(num_closest, s.closest_kinematic.items.len);
        }
    }
}

test "SoftBodySharedSettings: optimize and calculateClosestKinematic free the cache they filled when an allocation fails" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // Without LRA createConstraints does not fill the closest kinematic cache, so optimize fills it itself. An
    // allocation that fails after that must free it again.
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var cloth = try createTestCloth(allocator, 8, 0.1);
        defer cloth.deinit();
        const s = cloth.get().?;
        const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{})};
        try s.createConstraints(&attributes, .{ .bend_type = .dihedral });
        try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len);
        const before = try saveBytes(allocator, s);
        defer allocator.free(before);

        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        s.allocator = failing.allocator();
        var results: S.OptimizationResults = .{};
        const result = s.optimizeWithResults(&results);
        s.allocator = allocator;
        results.deinit(allocator);
        try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len); // Optimize also frees it when it succeeds
        if (result) |_| {
            try std.testing.expect(fail_index > 10);
            break;
        } else |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            const after = try saveBytes(allocator, s);
            defer allocator.free(after);
            try std.testing.expectEqualSlices(u8, before, after);
        }
    }

    // calculateClosestKinematic called directly caches nothing when it fails
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var cloth = try createTestCloth(allocator, 5, 0.1);
        defer cloth.deinit();
        const s = cloth.get().?;
        const attributes = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{})};
        try s.createConstraints(&attributes, .{});

        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        s.allocator = failing.allocator();
        const result = s.calculateClosestKinematic();
        s.allocator = allocator;
        if (result) |_| {
            try std.testing.expect(fail_index > 2);
            try std.testing.expectEqual(s.vertices.items.len, s.closest_kinematic.items.len);
            break;
        } else |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len);
        }
    }
}

test "SoftBodySharedSettings: createConstraints without vertex attributes and without faces (next to R20 UB13)" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;

    // Without faces no edge reads the attributes: an empty attribute list is fine (with faces it panics, UB13)
    var settings: Ref(S) = .init(try S.create(allocator));
    defer settings.deinit();
    const s = settings.get().?;
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 0 }, .{}));
    try s.vertices.append(allocator, .init(.{ .x = 1, .y = 0, .z = 0 }, .{}));
    try s.createConstraints(&.{}, .{});
    try std.testing.expectEqual(@as(usize, 0), s.edge_constraints.items.len);
    try std.testing.expectEqual(@as(usize, 0), s.lra_constraints.items.len);

    // One attribute is repeated for every vertex
    try s.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 1 }, .{}));
    try s.addFace(.init(0, 1, 2, .{}));
    try s.createConstraints(&.{.init(0.5, 0.5, 0.5, .{})}, .{});
    try std.testing.expectEqual(@as(usize, 3), s.edge_constraints.items.len);
    for (s.edge_constraints.items) |e| try std.testing.expectEqual(@as(f32, 0.5), e.compliance);
}

test "SoftBodySharedSettings: closest kinematic cache of the right size (next to R20 UB12)" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;
    const lra = [_]S.VertexAttributes{.init(0.0, 0.0, 0.0, .{ .lra_type = .euclidean_distance })};

    var cloth = try createTestCloth(allocator, 4, 0.5);
    defer cloth.deinit();
    const s = cloth.get().?;

    // createConstraints fills the cache, optimize uses and frees it
    try s.createConstraints(&lra, .{});
    try std.testing.expectEqual(@as(usize, 16), s.closest_kinematic.items.len);
    try s.optimize();
    try std.testing.expectEqual(@as(usize, 0), s.closest_kinematic.items.len);

    // A vertex added while the cache is empty: the next cache has its size
    try s.vertices.append(allocator, .init(.{ .x = 2, .y = 0, .z = 0 }, .{}));
    try s.addFace(.init(3, 16, 7, .{}));
    try s.createConstraints(&lra, .{});
    try std.testing.expectEqual(@as(usize, 17), s.closest_kinematic.items.len);

    // A vertex added while the cache is filled: createConstraints without an LRA type does not read the cache. With a
    // cache of the right size again (here: cleared), optimize works. (With the stale cache both panic, UB12.)
    try s.vertices.append(allocator, .init(.{ .x = 2, .y = 0, .z = 0.5 }, .{}));
    try s.addFace(.init(7, 16, 17, .{}));
    try s.createConstraints(&.{.init(0.0, 0.0, 0.0, .{})}, .{});
    try std.testing.expectEqual(@as(usize, 17), s.closest_kinematic.items.len);
    s.closest_kinematic.clearAndFree(allocator);
    try s.optimize();
    try std.testing.expect(s.update_groups.items.len > 0);
}

/// A 20 x 20 cloth with its constraints and an extra vertex at FLT_MAX that no group reaches (a test helper)
fn createClothWithFarVertex(allocator: Allocator) Allocator.Error!Ref(SoftBodySharedSettings) {
    var cloth = try createTestCloth(allocator, 20, 0.1);
    errdefer cloth.deinit();
    const s = cloth.get().?;
    try s.createConstraints(&.{.init(0.0, 0.0, 0.0, .{})}, .{ .bend_type = .distance });
    try s.vertices.append(allocator, .init(.{ .x = math.flt_max, .y = math.flt_max, .z = math.flt_max }, .{}));
    return cloth;
}

test "SoftBodySharedSettings: optimize with a vertex that no group reaches and no constraint uses (next to R20 UB14)" {
    const allocator = std.testing.allocator;
    var cloth = try createClothWithFarVertex(allocator);
    defer cloth.deinit();
    const s = cloth.get().?;
    const num_edges = s.edge_constraints.items.len;

    // The far vertex is never a start vertex and has no connection: it stays outside every group. No constraint reads
    // its group, so optimize works in every build.
    try s.optimize();
    const groups = s.update_groups.items;
    try std.testing.expect(groups.len >= 2);
    try std.testing.expectEqual(@as(u32, @intCast(num_edges)), groups[groups.len - 1].edge_end_index);
}

test "SoftBodySharedSettings: optimize with a vertex at FLT_MAX that a group reaches (next to R20 UB14)" {
    const allocator = std.testing.allocator;
    const S = SoftBodySharedSettings;
    var settings: Ref(S) = .init(try S.create(allocator));
    defer settings.deinit();
    const s = settings.get().?;

    // The far vertex is never a start vertex. But an edge connects it to a group that is not full, so the group takes it.
    // Then its edge and its skinned constraint are in that group, and optimize works in every build. (Without the edge
    // to vertex 0, the skinned constraint panics, UB14.)
    for ([_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 }, .{ math.flt_max, math.flt_max, math.flt_max } }) |p|
        try s.vertices.append(allocator, .init(.{ .x = p[0], .y = p[1], .z = p[2] }, .{}));
    for (1..5) |v| try s.edge_constraints.append(allocator, .init(0, @intCast(v), .{}));
    try s.skinned_constraints.append(allocator, .{ .vertex = 4 });
    try s.optimize();

    // One group, more than half empty: it is merged into the non parallel group
    try std.testing.expectEqual(@as(usize, 1), s.update_groups.items.len);
    try std.testing.expectEqual(@as(u32, 4), s.update_groups.items[0].edge_end_index);
    try std.testing.expectEqual(@as(u32, 1), s.update_groups.items[0].skinned_end_index);
}

test "SoftBodySharedSettings: optimize with a constraint that has some vertices in a group (R20 UB14, Jolt's release path)" {
    // Jolt asserts: the builds with asserts stop there, like Jolt's debug build
    if (Core.enable_asserts) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var cloth = try createClothWithFarVertex(allocator);
    defer cloth.deinit();
    const s = cloth.get().?;

    // An edge from vertex 0 (the first start vertex) to the far vertex. The first group fills up with cloth vertices
    // that are closer, and no later group connects to the far vertex: it stays outside every group.
    const far: u32 = @intCast(s.vertices.items.len - 1);
    try s.edge_constraints.append(allocator, .init(0, far, .{ .compliance = 0.25 }));
    try s.optimize();

    // Jolt's release build puts the edge in the non parallel group (the last update group)
    const groups = s.update_groups.items;
    try std.testing.expect(groups.len >= 3);
    var found = false;
    for (s.edge_constraints.items, 0..) |e, i| {
        if (e.vertex[0] == far or e.vertex[1] == far) {
            try std.testing.expect(i >= groups[groups.len - 2].edge_end_index);
            found = true;
        }
    }
    try std.testing.expect(found);
}
