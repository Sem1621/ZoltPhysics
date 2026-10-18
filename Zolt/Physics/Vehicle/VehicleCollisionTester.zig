//! Port of: Jolt/Physics/Vehicle/VehicleCollisionTester.h, Jolt/Physics/Vehicle/VehicleCollisionTester.cpp
//! Status: complete
//!
//! - CharacterVehicleArchitecture E2: `VehicleCollisionTester` is an abstract pattern A root with `ref_count`
//!   (RefTarget<VehicleCollisionTester>, Jolt's `Collide` and `PredictContactProperties` are pure virtual). Ray,
//!   CastSphere and CastCylinder embed it as `base` and list their `overrides`; `create(allocator, ...)` is Jolt's `new`
//!   (reference count 0, put it in a `RefConst`, VehicleConstraint.setVehicleCollisionTester takes a reference).
//! - `Collide` / `PredictContactProperties` keep Jolt's in / out reference parameters as pointers (`out_body`,
//!   `out_sub_shape_id`, ...): Jolt's testers write them only when they find a hit (Collide) or only some of them
//!   (PredictContactProperties), and VehicleConstraint relies on the values they keep (the contact position and normal
//!   of a wheel that loses its contact are kept and saved in the state).
//! - E6: the testers go through `NarrowPhaseQuery.castRayCollector` / `castShape` of the no-lock interface, with Jolt's
//!   default filters (the default broad phase / object layer filter of the tester's object layer and an
//!   IgnoreSingleBodyFilter of the vehicle body) unless filters are set. CastSphere and CastCylinder build their query
//!   shape on the stack and embed it like Jolt (`setEmbedded`, `deinit` at the end of the query). Their collectors are
//!   local structs (`overrides = .{.addHit}`) that lock the hit body with a BodyLockRead of the no-lock interface and
//!   store it; the output is `@constCast` like Jolt's `const_cast<Body *>` (VehicleConstraint applies impulses to the
//!   contact body in the solver, outside this function).
//! - The constructors' default arguments (up vector, max slope angle, convex radius fraction) are options structs.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const math = @import("../../Math/Math.zig");
const Trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyFilterFile = @import("../Body/BodyFilter.zig");
const BodyFilter = BodyFilterFile.BodyFilter;
const IgnoreSingleBodyFilter = BodyFilterFile.IgnoreSingleBodyFilter;
const BodyLockRead = @import("../Body/BodyLock.zig").BodyLockRead;
const ObjectLayerFile = @import("../Collision/ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const ObjectLayerFilter = ObjectLayerFile.ObjectLayerFilter;
const object_layer_invalid = ObjectLayerFile.object_layer_invalid;
const BroadPhaseLayerFilter = @import("../Collision/BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
const RayCastFile = @import("../Collision/RayCast.zig");
const RRayCast = RayCastFile.RRayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const ShapeCastFile = @import("../Collision/ShapeCast.zig");
const RShapeCast = ShapeCastFile.RShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const RayCastResult = @import("../Collision/CastResult.zig").RayCastResult;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const CylinderShape = @import("../Collision/Shape/CylinderShape.zig").CylinderShape;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const VehicleConstraint = @import("VehicleConstraint.zig").VehicleConstraint;

/// Class that does collision detection between wheels and ground
pub const VehicleCollisionTester = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *VehicleCollisionTester) void,
        /// delete this (generated)
        destroy: *const fn (self: *VehicleCollisionTester) void,

        /// Do a collision test with the world
        /// @param inPhysicsSystem The physics system that should be tested against
        /// @param inVehicleConstraint The vehicle constraint
        /// @param inWheelIndex Index of the wheel that we're testing collision for
        /// @param inOrigin Origin for the test, corresponds to the world space position for the suspension attachment point
        /// @param inDirection Direction for the test (unit vector, world space)
        /// @param inVehicleBodyID This body should be filtered out during collision detection to avoid self collisions
        /// @param outBody Body that the wheel collided with
        /// @param outSubShapeID Sub shape ID that the wheel collided with
        /// @param outContactPosition Contact point between wheel and floor, in world space
        /// @param outContactNormal Contact normal between wheel and floor, pointing away from the floor
        /// @param outSuspensionLength New length of the suspension [0, inSuspensionMaxLength]
        /// @return True when collision found, false if not
        collide: *const fn (self: *const VehicleCollisionTester, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, out_body: *?*Body, out_sub_shape_id: *SubShapeID, out_contact_position: *RVec3, out_contact_normal: *Vec3, out_suspension_length: *f32) bool,

        /// Do a cheap contact properties prediction based on the contact properties from the last collision test (provided as input parameters)
        /// @param inPhysicsSystem The physics system that should be tested against
        /// @param inVehicleConstraint The vehicle constraint
        /// @param inWheelIndex Index of the wheel that we're testing collision for
        /// @param inOrigin Origin for the test, corresponds to the world space position for the suspension attachment point
        /// @param inDirection Direction for the test (unit vector, world space)
        /// @param inVehicleBodyID The body ID for the vehicle itself
        /// @param ioBody Body that the wheel previously collided with
        /// @param ioSubShapeID Sub shape ID that the wheel collided with during the last check
        /// @param ioContactPosition Contact point between wheel and floor during the last check, in world space
        /// @param ioContactNormal Contact normal between wheel and floor during the last check, pointing away from the floor
        /// @param ioSuspensionLength New length of the suspension [0, inSuspensionMaxLength]
        predictContactProperties: *const fn (self: *const VehicleCollisionTester, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, io_body: *?*Body, io_sub_shape_id: *SubShapeID, io_contact_position: *RVec3, io_contact_normal: *Vec3, io_suspension_length: *f32) void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<VehicleCollisionTester>)
    ref_count: RefCount = .{},
    /// Frees this tester (heap testers)
    allocator: Allocator,

    broad_phase_layer_filter: ?*const BroadPhaseLayerFilter = null,
    object_layer_filter: ?*const ObjectLayerFilter = null,
    body_filter: ?*const BodyFilter = null,
    object_layer: ObjectLayer = object_layer_invalid,

    /// Constructors (VehicleCollisionTester() / explicit VehicleCollisionTester(ObjectLayer)), called by derived
    /// classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator, opts: struct { object_layer: ObjectLayer = object_layer_invalid }) VehicleCollisionTester {
        return .{ .vtable = vtableFor(T), .allocator = allocator, .object_layer = opts.object_layer };
    }

    /// The vtable of tester class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<VehicleCollisionTester>

    /// Add a reference to this object
    pub fn addRef(self: *const VehicleCollisionTester) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const VehicleCollisionTester) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this tester as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const VehicleCollisionTester) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const VehicleCollisionTester) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a tester that is not on the heap (embedded / stack)
    pub fn deinit(self: *VehicleCollisionTester) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// The object layer the wheel belongs to. This is used to determine which objects the wheel can collide with. Works as BodyCreationSettings::mObjectLayer.
    /// Used only when the ObjectLayerFilter is not overridden.
    pub fn getObjectLayer(self: *const VehicleCollisionTester) ObjectLayer {
        return self.object_layer;
    }

    pub fn setObjectLayer(self: *VehicleCollisionTester, object_layer: ObjectLayer) void {
        self.object_layer = object_layer;
    }

    /// Access to the broad phase layer filter, when set this overrides the object layer supplied in the constructor
    pub fn setBroadPhaseLayerFilter(self: *VehicleCollisionTester, filter: ?*const BroadPhaseLayerFilter) void {
        self.broad_phase_layer_filter = filter;
    }

    pub fn getBroadPhaseLayerFilter(self: *const VehicleCollisionTester) ?*const BroadPhaseLayerFilter {
        return self.broad_phase_layer_filter;
    }

    /// Access to the object layer filter, when set this overrides the object layer supplied in the constructor
    pub fn setObjectLayerFilter(self: *VehicleCollisionTester, filter: ?*const ObjectLayerFilter) void {
        self.object_layer_filter = filter;
    }

    pub fn getObjectLayerFilter(self: *const VehicleCollisionTester) ?*const ObjectLayerFilter {
        return self.object_layer_filter;
    }

    /// Access to the body filter, when set this overrides the default filter that filters out the vehicle body
    pub fn setBodyFilter(self: *VehicleCollisionTester, filter: ?*const BodyFilter) void {
        self.body_filter = filter;
    }

    pub fn getBodyFilter(self: *const VehicleCollisionTester) ?*const BodyFilter {
        return self.body_filter;
    }

    /// Do a collision test with the world (see VTable.collide)
    pub fn collide(self: *const VehicleCollisionTester, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, out_body: *?*Body, out_sub_shape_id: *SubShapeID, out_contact_position: *RVec3, out_contact_normal: *Vec3, out_suspension_length: *f32) bool {
        return self.vtable.collide(self, physics_system, vehicle_constraint, wheel_index, origin, direction, vehicle_body_id, out_body, out_sub_shape_id, out_contact_position, out_contact_normal, out_suspension_length);
    }

    /// Do a cheap contact properties prediction based on the contact properties from the last collision test (see VTable.predictContactProperties)
    pub fn predictContactProperties(self: *const VehicleCollisionTester, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, io_body: *?*Body, io_sub_shape_id: *SubShapeID, io_contact_position: *RVec3, io_contact_normal: *Vec3, io_suspension_length: *f32) void {
        self.vtable.predictContactProperties(self, physics_system, vehicle_constraint, wheel_index, origin, direction, vehicle_body_id, io_body, io_sub_shape_id, io_contact_position, io_contact_normal, io_suspension_length);
    }

    /// Abstract: Collide and PredictContactProperties are pure virtual
    pub const impl = struct {};

    /// The filters of a query: the tester's filters or the defaults for its object layer and the vehicle body (the
    /// first lines of every Collide)
    const QueryFilters = struct {
        default_broadphase_layer_filter: @import("../Collision/BroadPhase/BroadPhaseLayer.zig").DefaultBroadPhaseLayerFilter,
        default_object_layer_filter: ObjectLayerFile.DefaultObjectLayerFilter,
        default_body_filter: IgnoreSingleBodyFilter,

        fn init(tester: *const VehicleCollisionTester, physics_system: *const PhysicsSystem, vehicle_body_id: BodyID) QueryFilters {
            return .{
                .default_broadphase_layer_filter = physics_system.getDefaultBroadPhaseLayerFilter(tester.object_layer),
                .default_object_layer_filter = physics_system.getDefaultLayerFilter(tester.object_layer),
                .default_body_filter = .init(vehicle_body_id),
            };
        }

        fn broadPhaseLayerFilter(self: *const QueryFilters, tester: *const VehicleCollisionTester) *const BroadPhaseLayerFilter {
            return tester.broad_phase_layer_filter orelse &self.default_broadphase_layer_filter.base;
        }

        fn objectLayerFilter(self: *const QueryFilters, tester: *const VehicleCollisionTester) *const ObjectLayerFilter {
            return tester.object_layer_filter orelse &self.default_object_layer_filter.base;
        }

        fn bodyFilter(self: *const QueryFilters, tester: *const VehicleCollisionTester) *const BodyFilter {
            return tester.body_filter orelse &self.default_body_filter.base;
        }
    };
};

/// Collision tester that tests collision using a raycast
pub const VehicleCollisionTesterRay = struct {
    pub const overrides = .{ .collide, .predictContactProperties };

    base: VehicleCollisionTester,
    up: Vec3,
    cos_max_slope_angle: f32,

    /// Constructor
    /// @param inObjectLayer The object layer the wheel belongs to. This is used to determine which objects the wheel can collide with. Works as BodyCreationSettings::mObjectLayer.
    /// @param inUp World space up vector, used to avoid colliding with vertical walls.
    /// @param inMaxSlopeAngle Max angle (rad) that is considered for colliding wheels. This is to avoid colliding with vertical walls.
    pub fn init(allocator: Allocator, object_layer: ObjectLayer, opts: struct { up: Vec3 = Vec3.axisY(), max_slope_angle: f32 = math.degreesToRadians(80.0) }) VehicleCollisionTesterRay {
        return .{ .base = .init(VehicleCollisionTesterRay, allocator, .{ .object_layer = object_layer }), .up = opts.up, .cos_max_slope_angle = Trigonometry.cos(opts.max_slope_angle) };
    }

    /// new VehicleCollisionTesterRay(...): reference count 0, put it in a RefConst
    pub fn create(allocator: Allocator, object_layer: ObjectLayer, opts: struct { up: Vec3 = Vec3.axisY(), max_slope_angle: f32 = math.degreesToRadians(80.0) }) Allocator.Error!*VehicleCollisionTesterRay {
        const self = try allocator.create(VehicleCollisionTesterRay);
        self.* = .init(allocator, object_layer, .{ .up = opts.up, .max_slope_angle = opts.max_slope_angle });
        return self;
    }

    /// Upcast (implicit in C++)
    pub fn asTester(self: *VehicleCollisionTesterRay) *VehicleCollisionTester {
        return &self.base;
    }

    // See: VehicleCollisionTester
    pub fn collide(self: *const VehicleCollisionTesterRay, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, out_body: *?*Body, out_sub_shape_id: *SubShapeID, out_contact_position: *RVec3, out_contact_normal: *Vec3, out_suspension_length: *f32) bool {
        const filters: VehicleCollisionTester.QueryFilters = .init(&self.base, physics_system, vehicle_body_id);
        const broadphase_layer_filter = filters.broadPhaseLayerFilter(&self.base);
        const object_layer_filter = filters.objectLayerFilter(&self.base);
        const body_filter = filters.bodyFilter(&self.base);

        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const wheel_radius = wheel_settings.radius;
        const ray_length = wheel_settings.suspension_max_length + wheel_radius;
        const ray: RRayCast = .init(origin, direction.mulScalar(ray_length));

        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CastRayCollector,

            // Configuration
            physics_system: *PhysicsSystem,
            ray: RRayCast,
            up_direction: Vec3,
            cos_max_slope_angle: f32,

            // Resulting closest collision
            body: ?*const Body = null,
            sub_shape_id2: SubShapeID = .empty,
            contact_position: RVec3 = undefined,
            contact_normal: Vec3 = undefined,

            pub fn addHit(c: *@This(), result: *const RayCastResult) void {
                // Test if this collision is closer than the previous one
                if (result.fraction < c.base.getEarlyOutFraction()) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.physics_system.getBodyLockInterfaceNoLock(), result.body_id);
                    defer lock.deinit();
                    std.debug.assert(lock.succeeded()); // When this runs all bodies are locked so this should not fail
                    const body = lock.getBody();

                    if (body.isSensor())
                        return;

                    // Test that we're not hitting a vertical wall
                    const contact_pos = c.ray.getPointOnRay(result.fraction);
                    const normal = body.getWorldSpaceSurfaceNormal(result.sub_shape_id2, contact_pos);
                    if (normal.dot(c.up_direction) > c.cos_max_slope_angle) {
                        // Update early out fraction to this hit
                        c.base.updateEarlyOutFraction(result.fraction);

                        // Get the contact properties
                        c.body = body;
                        c.sub_shape_id2 = result.sub_shape_id2;
                        c.contact_position = contact_pos;
                        c.contact_normal = normal;
                    }
                }
            }
        };

        const settings: RayCastSettings = .{};

        var collector: MyCollector = .{ .base = .init(MyCollector), .physics_system = physics_system, .ray = ray, .up_direction = self.up, .cos_max_slope_angle = self.cos_max_slope_angle };
        physics_system.getNarrowPhaseQueryNoLock().castRayCollector(ray, &settings, &collector.base, .{ .broad_phase_layer_filter = broadphase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter });
        const body = collector.body orelse return false;

        out_body.* = @constCast(body); // Jolt: const_cast<Body *>
        out_sub_shape_id.* = collector.sub_shape_id2;
        out_contact_position.* = collector.contact_position;
        out_contact_normal.* = collector.contact_normal;
        out_suspension_length.* = math.max(@as(f32, 0.0), ray_length * collector.base.getEarlyOutFraction() - wheel_radius);

        return true;
    }

    pub fn predictContactProperties(self: *const VehicleCollisionTesterRay, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, io_body: *?*Body, io_sub_shape_id: *SubShapeID, io_contact_position: *RVec3, io_contact_normal: *Vec3, io_suspension_length: *f32) void {
        _ = .{ self, physics_system, vehicle_body_id, io_body, io_sub_shape_id };

        // Recalculate the contact points assuming the contact point is on an infinite plane
        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const d_dot_n = direction.dot(io_contact_normal.*);
        if (d_dot_n < -1.0e-6) {
            // Reproject the contact position using the suspension ray and the plane formed by the contact position and normal
            io_contact_position.* = origin.addVec3(direction.mulScalar(io_contact_position.sub(origin).toVec3().dot(io_contact_normal.*) / d_dot_n));

            // The suspension length is simply the distance between the contact position and the suspension origin excluding the wheel radius
            io_suspension_length.* = math.clamp(io_contact_position.sub(origin).toVec3().dot(direction) - wheel_settings.radius, @as(f32, 0.0), wheel_settings.suspension_max_length);
        } else {
            // If the normal is pointing away we assume there's no collision anymore
            io_suspension_length.* = wheel_settings.suspension_max_length;
        }
    }
};

/// Collision tester that tests collision using a sphere cast
pub const VehicleCollisionTesterCastSphere = struct {
    pub const overrides = .{ .collide, .predictContactProperties };

    base: VehicleCollisionTester,
    radius: f32,
    up: Vec3,
    cos_max_slope_angle: f32,

    /// Constructor
    /// @param inObjectLayer The object layer the wheel belongs to. This is used to determine which objects the wheel can collide with. Works as BodyCreationSettings::mObjectLayer.
    /// @param inUp World space up vector, used to avoid colliding with vertical walls.
    /// @param inRadius Radius of sphere
    /// @param inMaxSlopeAngle Max angle (rad) that is considered for colliding wheels. This is to avoid colliding with vertical walls.
    pub fn init(allocator: Allocator, object_layer: ObjectLayer, radius: f32, opts: struct { up: Vec3 = Vec3.axisY(), max_slope_angle: f32 = math.degreesToRadians(80.0) }) VehicleCollisionTesterCastSphere {
        return .{ .base = .init(VehicleCollisionTesterCastSphere, allocator, .{ .object_layer = object_layer }), .radius = radius, .up = opts.up, .cos_max_slope_angle = Trigonometry.cos(opts.max_slope_angle) };
    }

    /// new VehicleCollisionTesterCastSphere(...): reference count 0, put it in a RefConst
    pub fn create(allocator: Allocator, object_layer: ObjectLayer, radius: f32, opts: struct { up: Vec3 = Vec3.axisY(), max_slope_angle: f32 = math.degreesToRadians(80.0) }) Allocator.Error!*VehicleCollisionTesterCastSphere {
        const self = try allocator.create(VehicleCollisionTesterCastSphere);
        self.* = .init(allocator, object_layer, radius, .{ .up = opts.up, .max_slope_angle = opts.max_slope_angle });
        return self;
    }

    /// Upcast (implicit in C++)
    pub fn asTester(self: *VehicleCollisionTesterCastSphere) *VehicleCollisionTester {
        return &self.base;
    }

    // See: VehicleCollisionTester
    pub fn collide(self: *const VehicleCollisionTesterCastSphere, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, out_body: *?*Body, out_sub_shape_id: *SubShapeID, out_contact_position: *RVec3, out_contact_normal: *Vec3, out_suspension_length: *f32) bool {
        const filters: VehicleCollisionTester.QueryFilters = .init(&self.base, physics_system, vehicle_body_id);
        const broadphase_layer_filter = filters.broadPhaseLayerFilter(&self.base);
        const object_layer_filter = filters.objectLayerFilter(&self.base);
        const body_filter = filters.bodyFilter(&self.base);

        var sphere = SphereShape.init(self.base.allocator, self.radius, .{});
        sphere.asShape().setEmbedded();
        defer sphere.asShapeMut().deinit();

        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const wheel_radius = wheel_settings.radius;
        const shape_cast_length = wheel_settings.suspension_max_length + wheel_radius - self.radius;
        const shape_cast: RShapeCast = .init(sphere.asShape(), Vec3.one(), RMat44.translation(origin), direction.mulScalar(shape_cast_length));

        var settings: ShapeCastSettings = .{};
        settings.use_shrunken_shape_and_convex_radius = true;
        settings.return_deepest_point = true;

        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CastShapeCollector,

            // Configuration
            physics_system: *PhysicsSystem,
            shape_cast: *const RShapeCast,
            up_direction: Vec3,
            cos_max_slope_angle: f32,

            // Resulting closest collision
            body: ?*const Body = null,
            sub_shape_id2: SubShapeID = .empty,
            contact_position: RVec3 = undefined,
            contact_normal: Vec3 = undefined,
            fraction: f32 = undefined,

            pub fn addHit(c: *@This(), result: *const ShapeCastResult) void {
                // Test if this collision is closer/deeper than the previous one
                const early_out = result.getEarlyOutFraction();
                if (early_out < c.base.getEarlyOutFraction()) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.physics_system.getBodyLockInterfaceNoLock(), result.base.body_id2);
                    defer lock.deinit();
                    std.debug.assert(lock.succeeded()); // When this runs all bodies are locked so this should not fail
                    const body = lock.getBody();

                    if (body.isSensor())
                        return;

                    // Test that we're not hitting a vertical wall
                    const normal = result.base.penetration_axis.normalized().negate();
                    if (normal.dot(c.up_direction) > c.cos_max_slope_angle) {
                        // Update early out fraction to this hit
                        c.base.updateEarlyOutFraction(early_out);

                        // Get the contact properties
                        c.body = body;
                        c.sub_shape_id2 = result.base.sub_shape_id2;
                        c.contact_position = c.shape_cast.center_of_mass_start.getTranslation().addVec3(result.base.contact_point_on2);
                        c.contact_normal = normal;
                        c.fraction = result.fraction;
                    }
                }
            }
        };

        var collector: MyCollector = .{ .base = .init(MyCollector), .physics_system = physics_system, .shape_cast = &shape_cast, .up_direction = self.up, .cos_max_slope_angle = self.cos_max_slope_angle };
        physics_system.getNarrowPhaseQueryNoLock().castShape(&shape_cast, &settings, shape_cast.center_of_mass_start.getTranslation(), &collector.base, .{ .broad_phase_layer_filter = broadphase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter });
        const body = collector.body orelse return false;

        out_body.* = @constCast(body); // Jolt: const_cast<Body *>
        out_sub_shape_id.* = collector.sub_shape_id2;
        out_contact_position.* = collector.contact_position;
        out_contact_normal.* = collector.contact_normal;
        out_suspension_length.* = math.max(@as(f32, 0.0), shape_cast_length * collector.fraction + self.radius - wheel_radius);

        return true;
    }

    pub fn predictContactProperties(self: *const VehicleCollisionTesterCastSphere, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, io_body: *?*Body, io_sub_shape_id: *SubShapeID, io_contact_position: *RVec3, io_contact_normal: *Vec3, io_suspension_length: *f32) void {
        _ = .{ physics_system, vehicle_body_id, io_body, io_sub_shape_id };

        // Recalculate the contact points assuming the contact point is on an infinite plane
        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const d_dot_n = direction.dot(io_contact_normal.*);
        if (d_dot_n < -1.0e-6) {
            // Reproject the contact position using the suspension cast sphere and the plane formed by the contact position and normal
            // This solves x = inOrigin + fraction * inDirection and (x - ioContactPosition) . ioContactNormal = mRadius for fraction
            const oc_dot_n = io_contact_position.sub(origin).toVec3().dot(io_contact_normal.*);
            const fraction = (self.radius + oc_dot_n) / d_dot_n;
            io_contact_position.* = origin.addVec3(direction.mulScalar(fraction)).subVec3(io_contact_normal.mulScalar(self.radius));

            // Calculate the new suspension length in the same way as the cast sphere normally does
            io_suspension_length.* = math.clamp(fraction + self.radius - wheel_settings.radius, @as(f32, 0.0), wheel_settings.suspension_max_length);
        } else {
            // If the normal is pointing away we assume there's no collision anymore
            io_suspension_length.* = wheel_settings.suspension_max_length;
        }
    }
};

/// Collision tester that tests collision using a cylinder shape
pub const VehicleCollisionTesterCastCylinder = struct {
    pub const overrides = .{ .collide, .predictContactProperties };

    base: VehicleCollisionTester,
    convex_radius_fraction: f32,

    /// Constructor
    /// @param inObjectLayer The object layer the wheel belongs to. This is used to determine which objects the wheel can collide with. Works as BodyCreationSettings::mObjectLayer.
    /// @param inConvexRadiusFraction Fraction of half the wheel width (or wheel radius if it is smaller) that is used as the convex radius
    pub fn init(allocator: Allocator, object_layer: ObjectLayer, opts: struct { convex_radius_fraction: f32 = 0.1 }) VehicleCollisionTesterCastCylinder {
        if (Core.enable_asserts) std.debug.assert(opts.convex_radius_fraction >= 0.0 and opts.convex_radius_fraction <= 1.0);
        return .{ .base = .init(VehicleCollisionTesterCastCylinder, allocator, .{ .object_layer = object_layer }), .convex_radius_fraction = opts.convex_radius_fraction };
    }

    /// new VehicleCollisionTesterCastCylinder(...): reference count 0, put it in a RefConst
    pub fn create(allocator: Allocator, object_layer: ObjectLayer, opts: struct { convex_radius_fraction: f32 = 0.1 }) Allocator.Error!*VehicleCollisionTesterCastCylinder {
        const self = try allocator.create(VehicleCollisionTesterCastCylinder);
        self.* = .init(allocator, object_layer, .{ .convex_radius_fraction = opts.convex_radius_fraction });
        return self;
    }

    /// Upcast (implicit in C++)
    pub fn asTester(self: *VehicleCollisionTesterCastCylinder) *VehicleCollisionTester {
        return &self.base;
    }

    // See: VehicleCollisionTester
    pub fn collide(self: *const VehicleCollisionTesterCastCylinder, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, out_body: *?*Body, out_sub_shape_id: *SubShapeID, out_contact_position: *RVec3, out_contact_normal: *Vec3, out_suspension_length: *f32) bool {
        const filters: VehicleCollisionTester.QueryFilters = .init(&self.base, physics_system, vehicle_body_id);
        const broadphase_layer_filter = filters.broadPhaseLayerFilter(&self.base);
        const object_layer_filter = filters.objectLayerFilter(&self.base);
        const body_filter = filters.bodyFilter(&self.base);

        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const max_suspension_length = wheel_settings.suspension_max_length;

        // Get the wheel transform given that the cylinder rotates around the Y axis
        var shape_cast_start = vehicle_constraint.getWheelWorldTransform(wheel_index, Vec3.axisY(), Vec3.axisX());
        shape_cast_start.setTranslation(origin);

        // Construct a cylinder with the dimensions of the wheel
        const wheel_half_width = 0.5 * wheel_settings.width;
        var cylinder = CylinderShape.init(self.base.allocator, wheel_half_width, wheel_settings.radius, .{ .convex_radius = math.min(wheel_half_width, wheel_settings.radius) * self.convex_radius_fraction });
        cylinder.asShape().setEmbedded();
        defer cylinder.asShapeMut().deinit();

        const shape_cast: RShapeCast = .init(cylinder.asShape(), Vec3.one(), shape_cast_start, direction.mulScalar(max_suspension_length));

        var settings: ShapeCastSettings = .{};
        settings.use_shrunken_shape_and_convex_radius = true;
        settings.return_deepest_point = true;

        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CastShapeCollector,

            // Configuration
            physics_system: *PhysicsSystem,
            shape_cast: *const RShapeCast,

            // Resulting closest collision
            body: ?*const Body = null,
            sub_shape_id2: SubShapeID = .empty,
            contact_position: RVec3 = undefined,
            contact_normal: Vec3 = undefined,
            fraction: f32 = undefined,

            pub fn addHit(c: *@This(), result: *const ShapeCastResult) void {
                // Test if this collision is closer/deeper than the previous one
                const early_out = result.getEarlyOutFraction();
                if (early_out < c.base.getEarlyOutFraction()) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.physics_system.getBodyLockInterfaceNoLock(), result.base.body_id2);
                    defer lock.deinit();
                    std.debug.assert(lock.succeeded()); // When this runs all bodies are locked so this should not fail
                    const body = lock.getBody();

                    if (body.isSensor())
                        return;

                    // Update early out fraction to this hit
                    c.base.updateEarlyOutFraction(early_out);

                    // Get the contact properties
                    c.body = body;
                    c.sub_shape_id2 = result.base.sub_shape_id2;
                    c.contact_position = c.shape_cast.center_of_mass_start.getTranslation().addVec3(result.base.contact_point_on2);
                    c.contact_normal = result.base.penetration_axis.normalized().negate();
                    c.fraction = result.fraction;
                }
            }
        };

        var collector: MyCollector = .{ .base = .init(MyCollector), .physics_system = physics_system, .shape_cast = &shape_cast };
        physics_system.getNarrowPhaseQueryNoLock().castShape(&shape_cast, &settings, shape_cast.center_of_mass_start.getTranslation(), &collector.base, .{ .broad_phase_layer_filter = broadphase_layer_filter, .object_layer_filter = object_layer_filter, .body_filter = body_filter });
        const body = collector.body orelse return false;

        out_body.* = @constCast(body); // Jolt: const_cast<Body *>
        out_sub_shape_id.* = collector.sub_shape_id2;
        out_contact_position.* = collector.contact_position;
        out_contact_normal.* = collector.contact_normal;
        out_suspension_length.* = max_suspension_length * collector.fraction;

        return true;
    }

    pub fn predictContactProperties(self: *const VehicleCollisionTesterCastCylinder, physics_system: *PhysicsSystem, vehicle_constraint: *const VehicleConstraint, wheel_index: u32, origin: RVec3, direction: Vec3, vehicle_body_id: BodyID, io_body: *?*Body, io_sub_shape_id: *SubShapeID, io_contact_position: *RVec3, io_contact_normal: *Vec3, io_suspension_length: *f32) void {
        _ = .{ self, physics_system, vehicle_body_id, io_body, io_sub_shape_id };

        // Recalculate the contact points assuming the contact point is on an infinite plane
        const wheel_settings = vehicle_constraint.getWheel(wheel_index).getSettings();
        const d_dot_n = direction.dot(io_contact_normal.*);
        if (d_dot_n < -1.0e-6) {
            // Wheel size
            const half_width = 0.5 * wheel_settings.width;
            const radius = wheel_settings.radius;

            // Get the inverse local space contact normal for a cylinder pointing along Y
            const wheel_transform = vehicle_constraint.getWheelWorldTransform(wheel_index, Vec3.axisY(), Vec3.axisX());
            const inverse_local_normal = wheel_transform.multiply3x3Transposed(io_contact_normal.*).negate();

            // Get the support point of this normal in local space of the cylinder
            // See CylinderShape::Cylinder::GetSupport
            const x = inverse_local_normal.getX();
            const y = inverse_local_normal.getY();
            const z = inverse_local_normal.getZ();
            const o = @sqrt(math.square(x) + math.square(z));
            var support_point: Vec3 = undefined;
            if (o > 0.0)
                support_point = Vec3.init((radius * x) / o, math.sign(y) * half_width, (radius * z) / o)
            else
                support_point = Vec3.init(0, math.sign(y) * half_width, 0);

            // Rotate back to world space
            support_point = wheel_transform.multiply3x3(support_point);

            // Now we can use inOrigin + support_point as the start of a ray of our suspension to the contact plane
            // as know that it is the first point on the wheel that will hit the plane
            const ray_origin = origin.addVec3(support_point);

            // Calculate contact position and suspension length, the is the same as VehicleCollisionTesterRay
            // but we don't need to take the radius into account anymore
            const oc = io_contact_position.sub(ray_origin).toVec3();
            io_contact_position.* = ray_origin.addVec3(direction.mulScalar(oc.dot(io_contact_normal.*) / d_dot_n));
            io_suspension_length.* = math.clamp(oc.dot(direction), @as(f32, 0.0), wheel_settings.suspension_max_length);
        } else {
            // If the normal is pointing away we assume there's no collision anymore
            io_suspension_length.* = wheel_settings.suspension_max_length;
        }
    }
};

test "VehicleCollisionTester constructors, object layer, filters and reference count" {
    const testing = std.testing;
    const allocator = testing.allocator;

    // The ray tester stores the cosine of the max slope angle
    const ray = try VehicleCollisionTesterRay.create(allocator, 3, .{ .up = Vec3.axisZ(), .max_slope_angle = 0.5 });
    const tester = ray.asTester();
    tester.addRef();
    defer tester.release(); // Destroys the tester through the vtable (leak checked)
    try testing.expectEqual(@as(ObjectLayer, 3), tester.getObjectLayer());
    try testing.expect(ray.up.eql(Vec3.axisZ()));
    try testing.expectEqual(Trigonometry.cos(@as(f32, 0.5)), ray.cos_max_slope_angle);
    tester.setObjectLayer(5);
    try testing.expectEqual(@as(ObjectLayer, 5), tester.getObjectLayer());

    // No filters by default (the queries use the defaults for the object layer and the vehicle body)
    try testing.expect(tester.getBroadPhaseLayerFilter() == null);
    try testing.expect(tester.getObjectLayerFilter() == null);
    try testing.expect(tester.getBodyFilter() == null);
    const body_filter: IgnoreSingleBodyFilter = .init(BodyID.init(7));
    tester.setBodyFilter(&body_filter.base);
    try testing.expect(tester.getBodyFilter() == &body_filter.base);
    tester.setBodyFilter(null);

    // The sphere and cylinder testers, embedded
    var sphere: VehicleCollisionTesterCastSphere = .init(allocator, 1, 0.5, .{});
    sphere.base.setEmbedded();
    defer sphere.base.deinit();
    try testing.expectEqual(@as(f32, 0.5), sphere.radius);
    try testing.expectEqual(Trigonometry.cos(math.degreesToRadians(80.0)), sphere.cos_max_slope_angle);
    var cylinder: VehicleCollisionTesterCastCylinder = .init(allocator, 2, .{});
    cylinder.base.setEmbedded();
    defer cylinder.base.deinit();
    try testing.expectEqual(@as(f32, 0.1), cylinder.convex_radius_fraction);
    try testing.expectEqual(@as(ObjectLayer, 2), cylinder.base.getObjectLayer());
}
