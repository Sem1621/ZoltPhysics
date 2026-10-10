//! Port of: UnitTests/Physics/ConvexVsTrianglesTest.cpp
//! Status: partial
//! Missing: the TriangleShape / MeshShape halves of sCheckCollision / sCheckCollisionNoHit (need PhysicsTestContext,
//!   Phase 5), TestCapsuleVsNeedleTriangle (needs CapsuleShape, Phase 4 Wave A)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BodyID = zolt.BodyID;
const CollideConvexVsTriangles = zolt.CollideConvexVsTriangles;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSphereVsTriangles = zolt.CollideSphereVsTriangles;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const SubShapeID = zolt.SubShapeID;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

const edge_length: f32 = 4.0;

/// `Collider collider(sphere, ...)`: CollideConvexVsTriangles takes the sphere as a ConvexShape
fn initCollider(comptime Collider: type, sphere: *const SphereShape, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id1: SubShapeID, settings: *const CollideShapeSettings, collector: *CollideShapeCollector) Collider {
    const shape1 = if (Collider == CollideConvexVsTriangles) &sphere.base else sphere;
    return Collider.init(shape1, Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, sub_shape_id1, settings, collector);
}

fn checkCollisionNoHit(comptime Collider: type, settings: *const CollideShapeSettings, center: Vec3, radius: f32, active_edges: u8) !void {
    // Our sphere
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, radius, .{})).asShape());
    defer sphere.deinit();

    // Our default triangle
    const v1 = Vec3.init(0, 0, 0);
    const v2 = Vec3.init(0, 0, edge_length);
    const v3 = Vec3.init(edge_length, 0, 0);

    {
        // Collide sphere
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        var collider = initCollider(Collider, sphere.get().?.cast(SphereShape), Mat44.translation(center), Mat44.identity(), .empty, settings, &collector.base);
        collider.collide(v1, v2, v3, active_edges, .empty);
        try collector.checkError();
        try fw.expect(!collector.hadHit());
    }

    // Not ported: the TriangleShape and MeshShape (a single triangle) versions through NarrowPhaseQuery::CollideShape
    // (need PhysicsTestContext, Phase 5)
}

fn checkCollision(comptime Collider: type, settings_in: *const CollideShapeSettings, center: Vec3, radius: f32, active_edges: u8, expected_contact_on1: Vec3, expected_contact_on2: Vec3, expected_penetration_axis: Vec3, expected_penetration_depth: f32) !void {
    // Our sphere
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, radius, .{})).asShape());
    defer sphere.deinit();

    // Our default triangle
    const v1 = Vec3.init(0, 0, 0);
    const v2 = Vec3.init(0, 0, edge_length);
    const v3 = Vec3.init(edge_length, 0, 0);

    // A semi random transform for the triangle
    const translation = Vec3.init(1, 2, 3);
    const rotation = Quat.rotation(Vec3.axisX(), 0.25 * math.pi);
    const transform = Mat44.rotationTranslation(rotation, translation);
    const inv_transform = transform.inversedRotationTranslation();

    // The transform for the sphere
    const sphere_transform = transform.mul(Mat44.translation(center));

    // Transform incoming settings
    var settings = settings_in.*;
    settings.active_edge_movement_direction = transform.multiply3x3(settings_in.active_edge_movement_direction);

    // Test the specified collider
    {
        var sub_shape_id1: SubShapeID = .empty;
        var sub_shape_id2: SubShapeID = .empty;
        sub_shape_id1.setValue(123);
        sub_shape_id2.setValue(456);

        // Collide sphere
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        var collider = initCollider(Collider, sphere.get().?.cast(SphereShape), sphere_transform, transform, sub_shape_id1, &settings, &collector.base);
        collider.collide(v1, v2, v3, active_edges, sub_shape_id2);
        try collector.checkError();

        // Test result
        try fw.expectEqual(@as(usize, 1), collector.hits.items.len);
        const hit = &collector.hits.items[0];
        try fw.expect(hit.body_id2.eql(BodyID.invalid));
        try fw.expectEqual(sub_shape_id1.getValue(), hit.sub_shape_id1.getValue());
        try fw.expectEqual(sub_shape_id2.getValue(), hit.sub_shape_id2.getValue());
        const contact1 = inv_transform.mulVec3(hit.contact_point_on1);
        const contact2 = inv_transform.mulVec3(hit.contact_point_on2);
        const pen_axis = transform.multiply3x3Transposed(hit.penetration_axis).normalized();
        const expected_pen_axis = expected_penetration_axis.normalized();
        try fw.checkApproxEqual(contact1, expected_contact_on1, .{ .tolerance = 1.0e-4 });
        try fw.checkApproxEqual(contact2, expected_contact_on2, .{ .tolerance = 1.0e-4 });
        try fw.checkApproxEqual(pen_axis, expected_pen_axis, .{ .tolerance = 1.0e-4 });
        try fw.checkApproxEqual(hit.penetration_depth, expected_penetration_depth, .{ .tolerance = 1.0e-4 });
    }

    // Not ported: the TriangleShape and MeshShape (a single triangle) versions through NarrowPhaseQuery::CollideShape
    // (need PhysicsTestContext, Phase 5)
}

// Compares CollideShapeResult for two spheres with given positions and radii
fn testConvexVsTriangles(comptime Collider: type) !void {
    const radius: f32 = 0.5;
    const radius_rs2: f32 = radius / @sqrt(@as(f32, 2.0));
    const distance_to_triangle: f32 = 0.1;
    const distance_to_triangle_rs2: f32 = distance_to_triangle / @sqrt(@as(f32, 2.0));
    const epsilon: f32 = 1.0e-6; // A small epsilon to ensure we hit the front side
    const max_separation_distance: f32 = 0.5;
    const separation_distance: f32 = 0.1;

    // Loop over all possible active edge combinations
    var active_edges: u8 = 0;
    while (active_edges <= 0b111) : (active_edges += 1) {
        // Create settings
        var settings: CollideShapeSettings = .{};
        settings.back_face_mode = .collide_with_back_faces;

        // Settings with ignore back faces
        var settings_no_bf: CollideShapeSettings = .{};
        settings_no_bf.back_face_mode = .ignore_back_faces;

        // Settings with max separation distance
        var settings_max_distance: CollideShapeSettings = .{};
        settings_max_distance.back_face_mode = .collide_with_back_faces;
        settings_max_distance.max_separation_distance = max_separation_distance;

        {
            // There should be no hit in front of the triangle
            const sphere_center = Vec3.init(0.25 * edge_length, radius + separation_distance, 0.25 * edge_length);
            try checkCollisionNoHit(Collider, &settings, sphere_center, radius, active_edges);

            // But if there's a max separation distance there should be
            const expected1 = sphere_center.add(Vec3.init(0, -radius, 0));
            const expected2 = Vec3.init(0.25 * edge_length, 0, 0.25 * edge_length);
            const pen_axis = Vec3.init(0, -1, 0);
            const pen_depth = -separation_distance;
            try checkCollision(Collider, &settings_max_distance, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
        }

        {
            // But if we go beyond the separation distance we should again have no hit
            const sphere_center = Vec3.init(0.25 * edge_length, radius + max_separation_distance + separation_distance, 0.25 * edge_length);
            try checkCollisionNoHit(Collider, &settings_max_distance, sphere_center, radius, active_edges);
        }

        {
            // There should be no hit in behind the triangle
            const sphere_center = Vec3.init(0.25 * edge_length, -radius - separation_distance, 0.25 * edge_length);
            try checkCollisionNoHit(Collider, &settings, sphere_center, radius, active_edges);

            // But if there's a max separation distance there should be
            const expected1 = sphere_center.add(Vec3.init(0, radius, 0));
            const expected2 = Vec3.init(0.25 * edge_length, 0, 0.25 * edge_length);
            const pen_axis = Vec3.init(0, 1, 0);
            const pen_depth = -separation_distance;
            try checkCollision(Collider, &settings_max_distance, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
        }

        {
            // But if we go beyond the separation distance we should again have no hit
            const sphere_center = Vec3.init(0.25 * edge_length, -radius - max_separation_distance - separation_distance, 0.25 * edge_length);
            try checkCollisionNoHit(Collider, &settings_max_distance, sphere_center, radius, active_edges);
        }

        {
            // Hit interior from front side
            const expected2 = Vec3.init(0.25 * edge_length, 0, 0.25 * edge_length);
            const sphere_center = expected2.add(Vec3.init(0, distance_to_triangle, 0));
            const expected1 = sphere_center.add(Vec3.init(0, -radius, 0));
            const pen_axis = Vec3.init(0, -1, 0);
            const pen_depth = radius - distance_to_triangle;
            try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);

            // Ignore back faces should not matter
            try checkCollision(Collider, &settings_no_bf, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
        }

        {
            // Hit interior from back side
            const expected2 = Vec3.init(0.25 * edge_length, 0, 0.25 * edge_length);
            const sphere_center = expected2.add(Vec3.init(0, -distance_to_triangle, 0));
            const expected1 = sphere_center.add(Vec3.init(0, radius, 0));
            const pen_axis = Vec3.init(0, 1, 0);
            const pen_depth = radius - distance_to_triangle;
            try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);

            // Back face hit should be filtered
            try checkCollisionNoHit(Collider, &settings_no_bf, sphere_center, radius, active_edges);
        }

        // Loop over possible active edge movement direction permutations
        for (0..3) |movement_direction| {
            switch (movement_direction) {
                // Disable the system
                0 => settings.active_edge_movement_direction = Vec3.zero(),

                // Move into the triangle, this should always give us the normal from the edge
                1 => settings.active_edge_movement_direction = Vec3.init(0, -1, 0),

                // Move out of the triangle, we should always get the normal of the triangle
                else => settings.active_edge_movement_direction = Vec3.init(0, 1, 0),
            }

            {
                // Hit edge 1
                const expected2 = Vec3.init(0, 0, 0.5 * edge_length);
                const sphere_center = expected2.add(Vec3.init(-distance_to_triangle, epsilon, 0));
                const expected1 = sphere_center.add(Vec3.init(radius, 0, 0));
                const pen_axis = if ((active_edges & 0b001) != 0 or movement_direction == 1) Vec3.init(1, 0, 0) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }

            {
                // Hit edge 2
                const expected2 = Vec3.init(0.5 * edge_length, 0, 0.5 * edge_length);
                const sphere_center = expected2.add(Vec3.init(distance_to_triangle_rs2, epsilon, distance_to_triangle_rs2));
                const expected1 = sphere_center.sub(Vec3.init(radius_rs2, 0, radius_rs2));
                const pen_axis = if ((active_edges & 0b010) != 0 or movement_direction == 1) Vec3.init(-1, 0, -1) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }

            {
                // Hit edge 3
                const expected2 = Vec3.init(0.5 * edge_length, 0, 0);
                const sphere_center = expected2.add(Vec3.init(0, epsilon, -distance_to_triangle));
                const expected1 = sphere_center.add(Vec3.init(0, 0, radius));
                const pen_axis = if ((active_edges & 0b100) != 0 or movement_direction == 1) Vec3.init(0, 0, 1) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }

            {
                // Hit vertex 1
                const expected2 = Vec3.init(0, 0, 0);
                const sphere_center = expected2.add(Vec3.init(-distance_to_triangle_rs2, epsilon, -distance_to_triangle_rs2));
                const expected1 = sphere_center.add(Vec3.init(radius_rs2, 0, radius_rs2));
                const pen_axis = if ((active_edges & 0b101) != 0 or movement_direction == 1) Vec3.init(1, 0, 1) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }

            {
                // Hit vertex 2
                const expected2 = Vec3.init(0, 0, edge_length);
                const sphere_center = expected2.add(Vec3.init(-distance_to_triangle_rs2, epsilon, distance_to_triangle_rs2));
                const expected1 = sphere_center.add(Vec3.init(radius_rs2, 0, -radius_rs2));
                const pen_axis = if ((active_edges & 0b011) != 0 or movement_direction == 1) Vec3.init(1, 0, -1) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }

            {
                // Hit vertex 3
                const expected2 = Vec3.init(edge_length, 0, 0);
                const sphere_center = expected2.add(Vec3.init(distance_to_triangle_rs2, epsilon, -distance_to_triangle_rs2));
                const expected1 = sphere_center.add(Vec3.init(-radius_rs2, 0, radius_rs2));
                const pen_axis = if ((active_edges & 0b110) != 0 or movement_direction == 1) Vec3.init(-1, 0, 1) else Vec3.init(0, -1, 0);
                const pen_depth = radius - distance_to_triangle;
                try checkCollision(Collider, &settings, sphere_center, radius, active_edges, expected1, expected2, pen_axis, pen_depth);
            }
        }
    }
}

test "TestConvexVsTriangles" {
    try testConvexVsTriangles(CollideConvexVsTriangles);
}

test "TestSphereVsTriangles" {
    try testConvexVsTriangles(CollideSphereVsTriangles);
}

// Not ported: TestCapsuleVsNeedleTriangle (needs CapsuleShape, Phase 4 Wave A)
