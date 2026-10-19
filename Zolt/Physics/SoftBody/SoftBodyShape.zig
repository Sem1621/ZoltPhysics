//! Port of: Jolt/Physics/SoftBody/SoftBodyShape.h, Jolt/Physics/SoftBody/SoftBodyShape.cpp
//! Status: partial
//! Missing: GetMaterial, GetSurfaceNormal, GetSupportingFace, GetSubmergedVolume, CastRay (both), CollidePoint,
//!   CollideSoftBodyVertices, GetTrianglesStart / GetTrianglesNext (SBSGetTrianglesContext), the collision dispatch
//!   functions and sRegister (stubs that panic, an empty `register`: group S3); Draw (group D4)
//!
//! RemainingFeaturesArchitecture.md R15: a final pattern A shape (`base: Shape`, built with `Shape.vtableFor`). It is
//! only created by BodyManager.allocateSoftBody, embedded in the soft body next to its motion properties
//! (`SoftBodyWithMotionPropertiesAndShape`): `init` is Jolt's constructor, then BodyManager sets `motion_properties`
//! (Jolt: `mSoftBodyMotionProperties`, BodyManager is a friend) and marks the shape embedded.

const std = @import("std");
const Allocator = std.mem.Allocator;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const Float3 = @import("../../Math/Float3.zig").Float3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Quat = @import("../../Math/Quat.zig").Quat;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const MassProperties = @import("../Body/MassProperties.zig").MassProperties;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const SubShapeIDFile = @import("../Collision/Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const RayCastFile = @import("../Collision/RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../Collision/CastResult.zig").RayCastResult;
const ShapeFilter = @import("../Collision/ShapeFilter.zig").ShapeFilter;
const CollideSoftBodyVertexIterator = @import("../Collision/CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;
const Registry = @import("../Collision/CollisionDispatch.zig").Registry;
const SoftBodyMotionProperties = @import("SoftBodyMotionProperties.zig").SoftBodyMotionProperties;

/// Shape used exclusively for soft bodies. Adds the ability to perform collision checks against soft bodies.
pub const SoftBodyShape = struct {
    /// Concrete class: `Shape.cast(SoftBodyShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .soft_body;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .mustBeStatic, .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume };

    base: Shape,
    /// The motion properties of the soft body (Jolt: mSoftBodyMotionProperties, set by BodyManager)
    motion_properties: *const SoftBodyMotionProperties = undefined,

    /// Constructor (SoftBodyShape()): BodyManager sets `motion_properties` and marks the shape embedded
    pub fn init(allocator: Allocator) SoftBodyShape {
        return .{ .base = .init(Shape.vtableFor(SoftBodyShape), allocator, .soft_body, shape_sub_type) };
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const SoftBodyShape) *const Shape {
        return &self.base;
    }

    pub fn asShapeMut(self: *SoftBodyShape) *Shape {
        return &self.base;
    }

    /// Determine amount of bits needed to encode sub shape id
    pub fn getSubShapeIDBits(self: *const SoftBodyShape) u32 {
        // Ensure we have enough bits to encode our shape [0, n - 1]
        const n: u32 = @as(u32, @intCast(self.motion_properties.getFaces().len)) -% 1;
        return 32 - @as(u32, @clz(n));
    }

    /// Convert a sub shape ID back to a face index
    pub fn getFaceIndex(self: *const SoftBodyShape, sub_shape_id: SubShapeID) u32 {
        const result = sub_shape_id.popID(self.getSubShapeIDBits());
        std.debug.assert(result.remainder.isEmpty());
        return result.id;
    }

    // See Shape
    pub fn mustBeStatic(self: *const SoftBodyShape) bool {
        _ = self;
        return false;
    }

    pub fn getCenterOfMass(self: *const SoftBodyShape) Vec3 {
        _ = self;
        return Vec3.zero();
    }

    pub fn getLocalBounds(self: *const SoftBodyShape) AABox {
        return self.motion_properties.getLocalBounds();
    }

    pub fn getSubShapeIDBitsRecursive(self: *const SoftBodyShape) u32 {
        return self.getSubShapeIDBits();
    }

    pub fn getInnerRadius(self: *const SoftBodyShape) f32 {
        _ = self;
        return 0.0;
    }

    pub fn getMassProperties(self: *const SoftBodyShape) MassProperties {
        _ = self;
        return .{};
    }

    pub fn getMaterial(self: *const SoftBodyShape, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        _ = .{ self, sub_shape_id };
        @panic("SoftBodyShape.getMaterial: not ported yet (Phase 9 group S3)");
    }

    pub fn getSurfaceNormal(self: *const SoftBodyShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        _ = .{ self, sub_shape_id, local_surface_position };
        @panic("SoftBodyShape.getSurfaceNormal: not ported yet (Phase 9 group S3)");
    }

    pub fn getSupportingFace(self: *const SoftBodyShape, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        _ = .{ self, sub_shape_id, direction, scale, center_of_mass_transform, out_vertices };
        @panic("SoftBodyShape.getSupportingFace: not ported yet (Phase 9 group S3)");
    }

    pub fn getSubmergedVolume(self: *const SoftBodyShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        _ = .{ self, center_of_mass_transform, scale, surface };
        @panic("SoftBodyShape.getSubmergedVolume: not ported yet (Phase 9 group S3)");
    }

    // TODO(debug_renderer): Draw (group D4)

    pub fn castRay(self: *const SoftBodyShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        _ = .{ self, ray, sub_shape_id_creator, hit };
        @panic("SoftBodyShape.castRay: not ported yet (Phase 9 group S3)");
    }

    pub fn castRayCollector(self: *const SoftBodyShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        _ = .{ self, ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter };
        @panic("SoftBodyShape.castRayCollector: not ported yet (Phase 9 group S3)");
    }

    pub fn collidePoint(self: *const SoftBodyShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        _ = .{ self, point, sub_shape_id_creator, collector, shape_filter };
        @panic("SoftBodyShape.collidePoint: not ported yet (Phase 9 group S3)");
    }

    pub fn collideSoftBodyVertices(self: *const SoftBodyShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = .{ self, center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index };
        @panic("SoftBodyShape.collideSoftBodyVertices: not ported yet (Phase 9 group S3)");
    }

    pub fn getTrianglesStart(self: *const SoftBodyShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, context, box, position_com, rotation, scale };
        @panic("SoftBodyShape.getTrianglesStart: not ported yet (Phase 9 group S3)");
    }

    pub fn getTrianglesNext(self: *const SoftBodyShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        @panic("SoftBodyShape.getTrianglesNext: not ported yet (Phase 9 group S3)");
    }

    pub fn getStats(self: *const SoftBodyShape) Shape.Stats {
        return .init(@sizeOf(SoftBodyShape), @intCast(self.motion_properties.getFaces().len));
    }

    pub fn getVolume(self: *const SoftBodyShape) f32 {
        return self.motion_properties.getVolume();
    }

    /// SoftBodyShape::sRegister (not ported yet: group S3 fills in the soft body dispatch functions and the color)
    pub fn register(comptime r: *Registry) void {
        _ = r;
    }
};

test "SoftBodyShape: the shape of a soft body" {
    const allocator = std.testing.allocator;
    const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;
    const SoftBodyCreationSettings = @import("SoftBodyCreationSettings.zig").SoftBodyCreationSettings;
    const expectEqual = std.testing.expectEqual;

    // A tetrahedron surface of 4 faces
    const shared = try SoftBodySharedSettings.create(allocator);
    var creation: SoftBodyCreationSettings = .init(shared, .zero(), Quat.identity(), 0);
    defer creation.deinit();
    for ([_][3]f32{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 1, 0 } }) |p|
        try shared.vertices.append(allocator, .init(.{ .x = p[0], .y = p[1], .z = p[2] }, .{}));
    for ([_][3]u32{ .{ 0, 1, 2 }, .{ 0, 3, 1 }, .{ 0, 2, 3 }, .{ 1, 3, 2 } }) |f|
        try shared.addFace(.init(f[0], f[1], f[2], .{}));
    try shared.update_groups.append(allocator, std.mem.zeroes(SoftBodySharedSettings.UpdateGroup));

    var mp: SoftBodyMotionProperties = .{ .allocator = allocator };
    defer mp.deinit();
    try mp.initialize(&creation);

    var shape: SoftBodyShape = .init(allocator);
    shape.motion_properties = &mp;
    shape.asShape().setEmbedded();
    defer shape.asShapeMut().deinit();

    const s = shape.asShape();
    try std.testing.expect(s.isKindOf(SoftBodyShape));
    try expectEqual(ShapeFile.ShapeType.soft_body, s.getType());
    try std.testing.expect(!s.mustBeStatic());
    try std.testing.expect(s.getCenterOfMass().eql(Vec3.zero()));
    try std.testing.expect(s.getLocalBounds().eql(AABox.init(Vec3.zero(), Vec3.one())));
    try expectEqual(@as(f32, 0.0), s.getInnerRadius());
    try expectEqual(@as(f32, 0.0), s.getMassProperties().mass);

    // 4 faces need 2 bits; a face index survives the sub shape ID round trip
    try expectEqual(@as(u32, 2), s.getSubShapeIDBitsRecursive());
    const creator: SubShapeIDCreator = .{};
    const id = creator.pushID(3, shape.getSubShapeIDBits()).getID();
    try expectEqual(@as(u32, 3), shape.getFaceIndex(id));
    try expectEqual(@as(u32, 4), s.getStats().num_triangles);

    // The tetrahedron has volume 1/6, its faces point inwards so the volume is negative
    try std.testing.expect(@abs(@abs(s.getVolume()) - 1.0 / 6.0) < 1.0e-6);

    // The bits for 1 face and for no face at all (n - 1 wraps around, like Jolt's uint32)
    shared.faces.items.len = 1;
    try expectEqual(@as(u32, 0), shape.getSubShapeIDBits());
    shared.faces.items.len = 0;
    try expectEqual(@as(u32, 32), shape.getSubShapeIDBits());
    shared.faces.items.len = 4;
}
