//! Port of: Jolt/Shaders/HairCalculateCollisionPlanes.hlsl, Jolt/Shaders/HairCalculateCollisionPlanesBindings.h
//! Status: complete
//!
//! The hair shader that finds, for every simulated vertex that is not a root, the collision plane of the convex hull it
//! penetrates most (Docs/Zolt/RemainingFeaturesArchitecture.md R12). Hair dispatches it once per update when there are
//! hulls to collide with.
//! - `gShapeIndices` holds a header and the vertex indices of the faces, like Hair writes it: per shape the number of
//!   faces and per face the start and end of its vertex indices (in `gShapeIndices` itself), then a terminating 0.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMath = @import("ShaderMath.zig");
const ShaderPlane = @import("ShaderPlane.zig");
const HairStructs = @import("HairStructs.zig");
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const JPH_Plane = ShaderCore.JPH_Plane;
const HairCollisionPlane = HairStructs.HairCollisionPlane;
const HairPosition = HairStructs.HairPosition;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairCalculateCollisionPlanes";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairCalculateCollisionPlanes): the field names
/// are the binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gPositions: hlsl.Buffer(HairPosition) = .{},
    gShapePlanes: hlsl.Buffer(JPH_Plane) = .{},
    gShapeVertices: hlsl.Buffer(JPH_float3) = .{},
    gShapeIndices: hlsl.Buffer(JPH_uint) = .{},
    gCollisionPlanes: hlsl.RWBuffer(HairCollisionPlane) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid vertex
    const vtx = tid.x +% c.num_strands; // Skip the root of each strand, it's fixed
    if (vtx >= c.num_vertices)
        return;

    // Load the vertex
    const pos = b.gPositions.get(vtx).position;

    // Start with a plane that is far away (i.e. no collision)
    var collision_plane: HairCollisionPlane = .{
        .plane = float4.init(1, 0, 0, 1.0e6),
        .shape_index = 0,
    };
    var largest_penetration: f32 = -1.0e6;

    // Loop over all shapes
    var current_idx: u32 = 0;
    var current_plane: u32 = 0;
    var current_shape_idx: u32 = 0;
    while (true) : (current_shape_idx +%= 1) {
        // Find most facing plane
        var max_distance: f32 = -1.0e6;
        var max_plane_normal = float3.init(0, 0, 0);
        var max_plane_face_info: u32 = 0;

        // Get number of faces in this shape
        const nf = b.gShapeIndices.get(current_idx);
        current_idx +%= 1;
        if (nf == 0)
            break;

        var f: u32 = 0;
        while (f < nf) : (f += 1) {
            // Get the plane
            const plane = b.gShapePlanes.get(current_plane);
            current_plane +%= 1;
            const distance = ShaderPlane.planeSignedDistance(plane, pos);
            if (distance > max_distance) {
                max_distance = distance;
                max_plane_normal = ShaderPlane.planeGetNormal(plane);
                max_plane_face_info = current_idx;
            }

            // Skip over vertex start and end
            current_idx +%= 2;
        }

        // Project point onto that plane, in local space to the vertex
        var closest_point = max_plane_normal.mulScalar(-max_distance);

        // Check edges if we're outside the hull (when inside we know the closest face is also the closest point to the surface)
        const is_outside = max_distance > 0.0;
        if (is_outside) {
            // Loop over edges
            var closest_point_dist_sq: f32 = 1.0e12;
            var vi = b.gShapeIndices.get(max_plane_face_info);
            const vi_end = b.gShapeIndices.get(max_plane_face_info +% 1);
            var p1 = b.gShapeVertices.get(b.gShapeIndices.get(vi_end -% 1));
            while (vi < vi_end) : (vi += 1) {
                // Get edge points
                const p2 = b.gShapeVertices.get(b.gShapeIndices.get(vi));

                // Check if the position is outside the edge (if not, the face will be closer)
                const p1_p2 = p2.sub(p1);
                const p1_pos = p1.sub(pos);
                const edge_normal = hlsl.cross(p1_p2, max_plane_normal);
                if (hlsl.dot(edge_normal, p1_pos) <= 0.0) {
                    // Get closest point on edge
                    const closest = ShaderMath.getClosestPointOnLine(p1_pos, p1_p2);
                    const distance_sq = hlsl.dot(closest, closest);
                    if (distance_sq < closest_point_dist_sq) {
                        closest_point_dist_sq = distance_sq;
                        closest_point = closest;
                    }
                }

                // Cycle vertex
                p1 = p2;
            }
        }

        // Check if this is the largest penetration
        var normal = closest_point.negate();
        const normal_length = hlsl.length(normal);
        var penetration = normal_length;
        if (is_outside)
            penetration = -penetration
        else
            normal = normal.negate();
        if (penetration > largest_penetration) {
            // Calculate contact plane
            normal = if (normal_length > 0.0) normal.divScalar(normal_length) else max_plane_normal;
            collision_plane.plane = ShaderPlane.planeFromPointAndNormal(pos.add(closest_point), normal);
            collision_plane.shape_index = current_shape_idx;
            largest_penetration = penetration;
        }
    }

    b.gCollisionPlanes.set(vtx, collision_plane);
}

test "HairCalculateCollisionPlanes: a vertex inside and a vertex outside a box" {
    const std = @import("std");

    // One strand of 3 vertices: the root is skipped, vertex 1 is inside the box [-1, 1]^3, vertex 2 is outside next to
    // the edge between the +x and +y faces
    var context = std.mem.zeroes(HairStructs.HairUpdateContext);
    context.num_strands = 1;
    context.num_vertices = 3;
    const context_bytes: [@sizeOf(HairStructs.HairUpdateContext)]u8 align(16) = @bitCast(context);
    var positions: [3]HairPosition align(16) = .{
        .{ .position = .init(0, 0, 0), .rotation = .init(0, 0, 0, 1) },
        .{ .position = .init(0.5, 0.1, 0), .rotation = .init(0, 0, 0, 1) },
        .{ .position = .init(2, 2, 0), .rotation = .init(0, 0, 0, 1) },
    };

    // The +x and +y faces of the box (enough for these vertices)
    var planes: [2]JPH_Plane align(16) = .{ .init(1, 0, 0, -1), .init(0, 1, 0, -1) };
    var vertices: [6]float3 align(16) = .{ .init(1, -1, -1), .init(1, 1, -1), .init(1, 1, 1), .init(1, -1, 1), .init(-1, 1, -1), .init(-1, 1, 1) };
    var indices: [14]u32 align(16) = .{
        2, // Number of faces
        6, 10, // +x face
        10, 14, // +y face
        0, // Terminator
        0, 1, 2, 3, // +x face
        1, 4, 5, 2, // +y face
    };
    var collision_planes: [3]HairCollisionPlane align(16) = @splat(.{ .plane = .init(0, 0, 0, 0), .shape_index = 0xffffffff });

    var b: Bindings = .{};
    b.gContext.bind(&context_bytes);
    b.gPositions.bind(std.mem.sliceAsBytes(&positions));
    b.gShapePlanes.bind(std.mem.sliceAsBytes(&planes));
    b.gShapeVertices.bind(std.mem.sliceAsBytes(&vertices));
    b.gShapeIndices.bind(std.mem.sliceAsBytes(&indices));
    b.gCollisionPlanes.bind(std.mem.sliceAsBytes(&collision_planes));
    for (0..3) |x| main(&b, .init(@intCast(x), 0, 0));

    // The root keeps its value
    try std.testing.expectEqual(0xffffffff, collision_planes[0].shape_index);

    // Inside: the plane of the closest face (+x) through the vertex projected onto it
    try std.testing.expect(collision_planes[1].plane.eql(.init(1, 0, 0, -1)));
    try std.testing.expectEqual(0, collision_planes[1].shape_index);

    // Outside: the closest point is the corner (1, 1, 0) of the edge, the normal points from it to the vertex
    const n = 1.0 / @sqrt(@as(f32, 2.0));
    const plane = collision_planes[2].plane;
    try std.testing.expectApproxEqAbs(n, plane.x, 1.0e-6);
    try std.testing.expectApproxEqAbs(n, plane.y, 1.0e-6);
    try std.testing.expectEqual(0, plane.z);
    try std.testing.expectApproxEqAbs(-2.0 * n, plane.w, 1.0e-6);
}
