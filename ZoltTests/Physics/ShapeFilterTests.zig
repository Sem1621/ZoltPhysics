//! Port of: UnitTests/Physics/ShapeFilterTests.cpp
//! Status: complete
//!
//! The `Filter` class is a SimShapeFilter (pattern A, `base: SimShapeFilter = .init(@This())`). The body IDs it reads
//! are set after it is installed, like Jolt (the filter is only called during the update). SimShapeFilter itself is
//! also tested inline in Zolt/Physics/Collision/SimShapeFilter.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;

const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SimShapeFilter = zolt.SimShapeFilter;
const SphereShape = zolt.SphereShape;
const StaticCompoundShape = zolt.StaticCompoundShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const Vec3 = zolt.Vec3;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

// Tests two spheres in one simulated body, one collides with a static platform, the other doesn't
test "TestSimShapeFilter" {
    // Test once per motion quality type
    for (0..2) |q| {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        // Log contacts
        var contact_listener: LoggingContactListener = .init(allocator);
        defer contact_listener.deinit();
        c.getSystem().setContactListener(.init(&contact_listener));

        // Install simulation shape filter
        const Filter = struct {
            pub const overrides = .{.shouldCollide};

            base: SimShapeFilter = .init(@This()),
            platform_id: BodyID = .invalid,
            compound_id: BodyID = .invalid,

            pub fn shouldCollide(self: *const @This(), body1: *const Body, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
                _ = .{ sub_shape_id_of_shape1, sub_shape_id_of_shape2 };

                // If the platform is colliding with the compound, filter out collisions where the shape has user data 1
                if (body1.getID().eql(self.platform_id) and body2.getID().eql(self.compound_id))
                    return shape2.getUserData() != 1
                else if (body1.getID().eql(self.compound_id) and body2.getID().eql(self.platform_id))
                    return shape1.getUserData() != 1;
                return true;
            }
        };
        var shape_filter: Filter = .{};
        c.getSystem().setSimShapeFilter(&shape_filter.base);

        // Floor
        const floor_id = (try c.createFloor()).getID();

        // Platform
        const bi = c.getBodyInterface();
        var platform_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.init(10, 0.5, 10), .{})).asShape(), RVec3.init(0, 3.5, 0), Quat.identity(), .static, Layers.non_moving);
        defer platform_settings.deinit();
        shape_filter.platform_id = try bi.createAndAddBody(&platform_settings, .dont_activate);

        // Compound shape that starts above platform
        var sphere = Ref(Shape).init((try SphereShape.create(allocator, 0.5, .{})).asShapeMut());
        defer sphere.deinit();
        sphere.get().?.setUserData(1); // Don't want sphere to collide with the platform
        var sphere2 = Ref(Shape).init((try SphereShape.create(allocator, 0.5, .{})).asShapeMut());
        defer sphere2.deinit();
        const compound_settings = try StaticCompoundShapeSettings.create(allocator);
        var compound_settings_ref = Ref(ShapeSettings).init(compound_settings.asShapeSettings());
        defer compound_settings_ref.deinit();
        try compound_settings.base.addShapePtr(Vec3.init(0, -2, 0), Quat.identity(), sphere.get(), .{});
        try compound_settings.base.addShapePtr(Vec3.init(0, 2, 0), Quat.identity(), sphere2.get(), .{});
        var compound_result = try compound_settings.asShapeSettings().createShape(allocator);
        defer compound_result.deinit();
        var compound_ref = RefConst(Shape).init(compound_result.getPtr());
        defer compound_ref.deinit();
        const compound = compound_ref.get().?.cast(StaticCompoundShape);
        var bcs = BodyCreationSettings.initWithShape(compound.asShape(), RVec3.init(0, 7, 0), Quat.identity(), .dynamic, Layers.moving);
        defer bcs.deinit();
        if (q == 1) {
            // For the 2nd iteration activate CCD
            bcs.motion_quality = .linear_cast;
            bcs.linear_velocity = Vec3.init(0, -50, 0);
        }
        shape_filter.compound_id = try bi.createAndAddBody(&bcs, .activate);

        // Get sub shape IDs
        const sphere_id = compound.base.getSubShapeIDFromIndex(0, .{}).getID();
        const sphere2_id = compound.base.getSubShapeIDFromIndex(1, .{}).getID();

        // Simulate for 2 seconds
        _ = try c.simulate(2.0, {});
        try contact_listener.checkError();

        // The compound should now be resting with sphere on the platform and the sphere2 on the floor
        try checkApproxEqual(bi.getPosition(shape_filter.compound_id), RVec3.init(0, 2.5, 0), .{ .tolerance = 1.01 * c.getSystem().getPhysicsSettings().penetration_slop });
        try checkApproxEqual(bi.getRotation(shape_filter.compound_id), Quat.identity(), .{});

        // Check that sphere2 collided with the platform but sphere did not
        try expect(contact_listener.containsSubShape(.add, shape_filter.platform_id, SubShapeID.empty, shape_filter.compound_id, sphere2_id));
        try expect(!contact_listener.containsSubShape(.add, shape_filter.platform_id, SubShapeID.empty, shape_filter.compound_id, sphere_id));

        // Check that sphere2 didn't collide with the floor but that the sphere did
        try expect(contact_listener.containsSubShape(.add, floor_id, SubShapeID.empty, shape_filter.compound_id, sphere_id));
        try expect(!contact_listener.containsSubShape(.add, floor_id, SubShapeID.empty, shape_filter.compound_id, sphere2_id));
    }
}
