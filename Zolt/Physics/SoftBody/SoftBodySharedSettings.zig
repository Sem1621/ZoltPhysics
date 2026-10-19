//! Port of: Jolt/Physics/SoftBody/SoftBodySharedSettings.h, Jolt/Physics/SoftBody/SoftBodySharedSettings.cpp
//! Status: partial
//! Missing: CreateConstraints, CalculateEdgeLengths, CalculateRodProperties, CalculateLRALengths,
//!   CalculateBendConstraintConstants, CalculateVolumeConstraintVolumes, CalculateSkinnedConstraintNormals, Optimize,
//!   sCreateCube, CalculateClosestKinematic (stubs that panic, group S1); SaveWithMaterials, sRestoreWithMaterials
//!   (group S2)
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
const math = @import("../../Math/Math.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
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

    pub const SettingsResult = @import("../../Core/Result.zig").Result(Ref(SoftBodySharedSettings));

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
    pub fn createConstraints(self: *SoftBodySharedSettings, vertex_attributes: []const VertexAttributes, opts: struct { bend_type: BendType = .distance, angle_tolerance: f32 = math.degreesToRadians(8.0) }) Allocator.Error!void {
        _ = .{ self, vertex_attributes, opts };
        @panic("SoftBodySharedSettings.createConstraints: not ported yet (Phase 9 group S1)");
    }

    /// Calculate the initial lengths of all springs of the edges of this soft body (if you use CreateConstraint, this is already done)
    pub fn calculateEdgeLengths(self: *SoftBodySharedSettings) void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateEdgeLengths: not ported yet (Phase 9 group S1)");
    }

    /// Calculate the properties of the rods
    /// Note that this can swap mVertex of the RodStretchShear constraints if two rods are connected through a RodBendTwist constraint but point in opposite directions.
    pub fn calculateRodProperties(self: *SoftBodySharedSettings) Allocator.Error!void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateRodProperties: not ported yet (Phase 9 group S1)");
    }

    /// Calculate the max lengths for the long range attachment constraints based on Euclidean distance (if you use CreateConstraints, this is already done)
    /// @param opts.max_distance_multiplier Multiplier for the max distance of the LRA constraint, e.g. 1.01 means the max distance is 1% longer than the calculated distance in the rest pose.
    pub fn calculateLRALengths(self: *SoftBodySharedSettings, opts: struct { max_distance_multiplier: f32 = 1.0 }) void {
        _ = .{ self, opts };
        @panic("SoftBodySharedSettings.calculateLRALengths: not ported yet (Phase 9 group S1)");
    }

    /// Calculate the constants for the bend constraints (if you use CreateConstraints, this is already done)
    pub fn calculateBendConstraintConstants(self: *SoftBodySharedSettings) void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateBendConstraintConstants: not ported yet (Phase 9 group S1)");
    }

    /// Calculates the initial volume of all tetrahedra of this soft body
    pub fn calculateVolumeConstraintVolumes(self: *SoftBodySharedSettings) void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateVolumeConstraintVolumes: not ported yet (Phase 9 group S1)");
    }

    /// Calculate information needed to be able to calculate the skinned constraint normals at run-time
    pub fn calculateSkinnedConstraintNormals(self: *SoftBodySharedSettings) Allocator.Error!void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateSkinnedConstraintNormals: not ported yet (Phase 9 group S1)");
    }

    /// Optimize the soft body settings for simulation. This will reorder constraints so they can be executed in parallel.
    /// `results` holds arrays allocated with the allocator of the settings (`results.deinit(settings.allocator)`).
    pub fn optimizeWithResults(self: *SoftBodySharedSettings, results: *OptimizationResults) Allocator.Error!void {
        _ = .{ self, results };
        @panic("SoftBodySharedSettings.optimizeWithResults: not ported yet (Phase 9 group S1)");
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

    /// Create a cube. This can be used to create a simple soft body for testing purposes.
    /// It will contain edge constraints, volume constraints and faces.
    /// @param grid_size Number of points along each axis
    /// @param grid_spacing Distance between points
    pub fn createCube(allocator: Allocator, grid_size: u32, grid_spacing: f32) Allocator.Error!Ref(SoftBodySharedSettings) {
        _ = .{ allocator, grid_size, grid_spacing };
        @panic("SoftBodySharedSettings.createCube: not ported yet (Phase 9 group S1)");
    }

    /// Add a face to this soft body
    pub fn addFace(self: *SoftBodySharedSettings, face: Face) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(!face.isDegenerate());
        try self.faces.append(self.allocator, face);
    }

    /// Calculate the closest kinematic vertex array (private in Jolt)
    pub fn calculateClosestKinematic(self: *SoftBodySharedSettings) Allocator.Error!void {
        _ = self;
        @panic("SoftBodySharedSettings.calculateClosestKinematic: not ported yet (Phase 9 group S1)");
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
