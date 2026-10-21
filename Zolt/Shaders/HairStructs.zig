//! Port of: Jolt/Shaders/HairStructs.h
//! Status: complete
//!
//! The structs and constants that the hair shaders and the C++ hair code share (Docs/Zolt/RemainingFeaturesArchitecture.md
//! R12). Hair (Physics/Hair) fills buffers of these structs and the shaders (Zolt/Shaders/Hair*.zig) read them, so each
//! is one extern struct with Jolt's layout. The sizes are checked at compile time and against the C++ structs by the
//! parity (ZoltParity/Compute/ComputeParity.zig).
//! - `JPH_HairFoo` is `HairFoo`. Struct members drop the `m` (`mPosition` is `position`), constant block members drop
//!   the `c` (`cNumStrands` is `num_strands`).
//! - JPH_SHADER_CONSTANT(int, cHairFoo, value) is `hair_foo`, a comptime_int: every use mixes it with a uint or an int,
//!   where C++ converts it to that type, and the values are small and positive, so the results are the same.
//! - The two constant blocks are `HairUpdateContext` (`gContext`) and `HairIterationContext` (`gIterationContext`). A
//!   shader that includes HairStructs.h binds both, first (R12).
//! - The C++ structs have no default member values. Zolt gives none either, except HairSVertexInfluence: it also serves
//!   as HairSettings::SVertexInfluence (see below). Buffers of these structs are zero-filled (ComputeBufferCPU) or
//!   written completely by Hair.
//! - HairSettings::SkinWeight, SkinPoint and SVertexInfluence derive from JPH_HairSkinWeight, JPH_HairSkinPoint and
//!   JPH_HairSVertexInfluence without adding members, so in Zolt they are these structs (R14; `HairSettings.SkinWeight`
//!   is an alias). The three carry the RTTI of the HairSettings class (JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL) with
//!   `.renamed = true`: the Zig name is the name of the shader struct. HairSVertexInfluence has the default values of
//!   the constructor of HairSettings::SVertexInfluence.

const std = @import("std");
const RTTI = @import("../Core/RTTI.zig").RTTI;
const Float3 = @import("../Math/Float3.zig").Float3;
const ShaderCore = @import("ShaderCore.zig");
const JPH_float = ShaderCore.JPH_float;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_float4 = ShaderCore.JPH_float4;
const JPH_uint = ShaderCore.JPH_uint;
const JPH_uint3 = ShaderCore.JPH_uint3;
const JPH_Quat = ShaderCore.JPH_Quat;
const JPH_Plane = ShaderCore.JPH_Plane;
const JPH_Mat44 = ShaderCore.JPH_Mat44;

/// JPH_SHADER_CONSTANT(int, cHairPerVertexBatch, 64)
pub const hair_per_vertex_batch = 64;
/// JPH_SHADER_CONSTANT(int, cHairPerGridCellBatch, 32)
pub const hair_per_grid_cell_batch = 32;
/// JPH_SHADER_CONSTANT(int, cHairPerStrandBatch, 32)
pub const hair_per_strand_batch = 32;
/// JPH_SHADER_CONSTANT(int, cHairPerRenderVertexBatch, 128)
pub const hair_per_render_vertex_batch = 128;

/// JPH_SHADER_CONSTANT(int, cHairNumSVertexInfluences, 3)
pub const hair_num_s_vertex_influences = 3;

/// JPH_HairSkinWeight
/// Also HairSettings::SkinWeight: how much a vertex is influenced by a joint
pub const HairSkinWeight = extern struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::SkinWeight)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "HairSettings::SkinWeight",
        .kind = .non_virtual,
        .renamed = true, // HairSettings::SkinWeight is this struct
        .attributes = &.{
            .attr("mJointIdx", .joint_idx),
            .attr("mWeight", .weight),
        },
    });

    joint_idx: JPH_uint,
    weight: JPH_float,
};

/// JPH_HairSkinPoint
/// Also HairSettings::SkinPoint: information about where a hair strand is attached to the scalp mesh
pub const HairSkinPoint = extern struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::SkinPoint)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "HairSettings::SkinPoint",
        .kind = .non_virtual,
        .renamed = true, // HairSettings::SkinPoint is this struct
        .attributes = &.{
            .attr("mTriangleIndex", .triangle_index),
            .attr("mU", .u),
            .attr("mV", .v),
        },
    });

    /// Index of triangle in mScalpVertices to which this skin point is attached
    triangle_index: JPH_uint,
    /// Barycentric u coordinate of skin point
    u: JPH_float,
    /// Barycentric v coordinate of skin point
    v: JPH_float,
    /// Compressed quaternion to rotate the frame defined by the triangle normal and the first edge to the Bishop frame of the first vertex of the strand
    to_bishop: JPH_uint,
};

/// JPH_HairGlobalPoseTransform
pub const HairGlobalPoseTransform = extern struct {
    position: JPH_float3,
    rotation: JPH_Quat,
};

/// JPH_HairSVertexInfluence
/// Also HairSettings::SVertexInfluence: describes how a render vertex is influenced by a simulated vertex
pub const HairSVertexInfluence = extern struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(HairSettings::SVertexInfluence)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "HairSettings::SVertexInfluence",
        .kind = .non_virtual,
        .renamed = true, // HairSettings::SVertexInfluence is this struct
        .attributes = &.{
            .attr("mVertexIndex", .vertex_index),
            .custom("mRelativePosition", RelativePositionAttribute), // A JPH_float3, which the stream sees as a Float3
            .attr("mWeight", .weight),
        },
    });

    /// HairSettings::cNoInfluence
    pub const no_influence: u32 = ~@as(u32, 0);

    // The defaults are those of the constructor of HairSettings::SVertexInfluence:
    // SVertexInfluence() { mVertexIndex = cNoInfluence; mRelativePosition = JPH_float3(0, 0, 0); mWeight = 0.0f; }

    /// Index in mSimVertices that indicates to which simulated vertex this vertex is attached.
    vertex_index: JPH_uint = no_influence,
    /// Position in local space from the simulated vertex to the render vertex
    relative_position: JPH_float3 = .{ .x = 0, .y = 0, .z = 0 },
    /// Influence weight, 0 = not attached, 1 = fully attached
    weight: JPH_float = 0.0,

    /// The attribute mRelativePosition: JPH_float3 is Float3 in C++ (ShaderCore.h), the same 12 bytes as `hlsl.float3`
    const RelativePositionAttribute = struct {
        pub const Member = Float3;

        pub fn get(self: *const HairSVertexInfluence) Float3 {
            return .{ .x = self.relative_position.x, .y = self.relative_position.y, .z = self.relative_position.z };
        }

        pub fn prepare(self: *HairSVertexInfluence) *Float3 {
            return @ptrCast(&self.relative_position);
        }
    };
};

/// JPH_HairPosition
pub const HairPosition = extern struct {
    position: JPH_float3,
    rotation: JPH_Quat,
};

/// JPH_HairVelocity
pub const HairVelocity = extern struct {
    velocity: JPH_float3,
    angular_velocity: JPH_float3,
};

/// JPH_HairMaterial
pub const HairMaterial = extern struct {
    world_transform_influence: JPH_float4,
    global_pose: JPH_float4,
    skin_global_pose: JPH_float4,
    gravity_factor: JPH_float4,
    hair_radius: JPH_float4,
    bend_compliance_multiplier: JPH_float4,
    grid_velocity_factor: JPH_float4,
    enable_collision: JPH_uint,
    enable_lra: JPH_uint,
    enable_grid: JPH_uint,
    friction: JPH_float,
    exp_linear_damping_delta_time: JPH_float,
    exp_angular_damping_delta_time: JPH_float,
    bend_compliance_inv_delta_time_sq: JPH_float,
    stretch_compliance_inv_delta_time_sq: JPH_float,
    grid_density_force_factor: JPH_float,
    inertia_multiplier: JPH_float,
    max_linear_velocity_sq: JPH_float,
    max_angular_velocity_sq: JPH_float,
};

/// JPH_HairCollisionPlane
pub const HairCollisionPlane = extern struct {
    plane: JPH_Plane,
    shape_index: JPH_uint,
};

/// JPH_HairCollisionShape
pub const HairCollisionShape = extern struct {
    center_of_mass: JPH_float3,
    linear_velocity: JPH_float3,
    angular_velocity: JPH_float3,
};

/// JPH_SHADER_CONSTANTS_BEGIN(JPH_HairUpdateContext, gContext)
/// Note: The order was chosen to match the struct between C++ and HLSL.
pub const HairUpdateContext = extern struct {
    num_strands: JPH_uint,
    num_vertices: JPH_uint,
    num_grid_points: JPH_uint,
    num_render_vertices: JPH_uint,
    grid_size_min2: JPH_uint3,
    two_div_delta_time: JPH_float,
    grid_size_min1: JPH_float3,
    delta_time: JPH_float,
    grid_offset: JPH_float3,
    half_delta_time: JPH_float,
    grid_scale: JPH_float3,
    inv_delta_time_sq: JPH_float,
    sub_step_gravity: JPH_float3,
    num_skin_vertices: JPH_uint,
    grid_stride: JPH_uint3,
    num_skin_weights_per_vertex: JPH_uint,
    delta_transform: JPH_Mat44,
    scalp_to_head: JPH_Mat44,
    delta_transform_quat: JPH_Quat,
};

/// JPH_SHADER_CONSTANTS_BEGIN(JPH_HairIterationContext, gIterationContext)
/// Note: The order was chosen to match the struct between C++ and HLSL.
pub const HairIterationContext = extern struct {
    /// = Iteration * DeltaTime
    accumulated_delta_time: JPH_float,
    /// = 1 / (NumIterations - Iteration) or the fraction to apply to get from current to target for this iteration step
    iteration_fraction: JPH_float,
};

comptime {
    // The sizes of the C++ structs (ShaderCore.h's C++ types: Float3 is 12 bytes, Float4 16, uint32[3] 12, Float4[4] 64)
    std.debug.assert(@sizeOf(HairSkinWeight) == 8);
    std.debug.assert(@sizeOf(HairSkinPoint) == 16);
    std.debug.assert(@sizeOf(HairGlobalPoseTransform) == 28);
    std.debug.assert(@sizeOf(HairSVertexInfluence) == 20);
    std.debug.assert(@sizeOf(HairPosition) == 28);
    std.debug.assert(@sizeOf(HairVelocity) == 24);
    std.debug.assert(@sizeOf(HairMaterial) == 160);
    std.debug.assert(@sizeOf(HairCollisionPlane) == 20);
    std.debug.assert(@sizeOf(HairCollisionShape) == 36);
    std.debug.assert(@sizeOf(HairUpdateContext) == 256);
    std.debug.assert(@sizeOf(HairIterationContext) == 8);
    std.debug.assert(@sizeOf(Float3) == @sizeOf(JPH_float3) and @alignOf(Float3) == @alignOf(JPH_float3));

    // The members that the comment "the order was chosen to match the struct between C++ and HLSL" is about: HLSL packs
    // a constant buffer in 16 byte rows, so every float3 / uint3 starts a row and the scalar after it fills the row
    std.debug.assert(@offsetOf(HairUpdateContext, "grid_size_min2") == 16);
    std.debug.assert(@offsetOf(HairUpdateContext, "grid_size_min1") == 32);
    std.debug.assert(@offsetOf(HairUpdateContext, "grid_offset") == 48);
    std.debug.assert(@offsetOf(HairUpdateContext, "grid_scale") == 64);
    std.debug.assert(@offsetOf(HairUpdateContext, "sub_step_gravity") == 80);
    std.debug.assert(@offsetOf(HairUpdateContext, "grid_stride") == 96);
    std.debug.assert(@offsetOf(HairUpdateContext, "delta_transform") == 112);
    std.debug.assert(@offsetOf(HairUpdateContext, "scalp_to_head") == 176);
    std.debug.assert(@offsetOf(HairUpdateContext, "delta_transform_quat") == 240);
}
