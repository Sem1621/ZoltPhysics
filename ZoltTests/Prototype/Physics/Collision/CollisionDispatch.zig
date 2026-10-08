//! Port of: Jolt/Physics/Collision/CollisionDispatch.h, Jolt/Physics/Collision/CollisionDispatch.cpp
//! Status: complete
//!
//! D4: Jolt's static tables `sCollideShape` / `sCastShape` (filled by RegisterTypes() at startup) and
//! `ShapeFunctions::sRegistry` are one immutable `Registry` built at compile time (RegisterTypes.zig): `Registry.build`
//! runs `init()` (sInit) and then every shape's `register` (a line by line port of its sRegister) in the order of
//! Jolt/RegisterTypes.cpp, so later registrations override earlier ones exactly like in Jolt. There is no mutable
//! global, no initialization call and nothing to synchronize. The dispatch functions keep Jolt's signatures (`void`,
//! no registry parameter) and read `RegisterTypes.registry`.

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const Mat44 = zolt.Mat44;
const Vec3 = zolt.Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSubType = ShapeFile.ShapeSubType;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const num_sub_shape_types = ShapeFile.num_sub_shape_types;
const SubShapeIDCreator = @import("Shape/SubShapeID.zig").SubShapeIDCreator;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const ShapeCastFile = @import("ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const ShapeFilterFile = @import("ShapeFilter.zig");
const ShapeFilter = ShapeFilterFile.ShapeFilter;
const ReversedShapeFilter = ShapeFilterFile.ReversedShapeFilter;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// Function that collides 2 shapes (see collideShapeVsShape)
pub const CollideShape = *const fn (shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void;

/// Function that casts a shape vs another shape (see castShapeVsShapeLocalSpace)
pub const CastShape = *const fn (shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void;

/// The collision function tables and the ShapeFunctions, built at compile time (see RegisterTypes.zig)
pub const Registry = struct {
    collide_shape: [num_sub_shape_types][num_sub_shape_types]CollideShape,
    cast_shape: [num_sub_shape_types][num_sub_shape_types]CastShape,
    shape_functions: [num_sub_shape_types]ShapeFunctions,

    /// Initialize all collision functions with a function that asserts and returns no collision (sInit)
    pub fn init() Registry {
        var r: Registry = undefined;
        for (&r.collide_shape) |*row| @memset(row, &collideUnsupported);
        for (&r.cast_shape) |*row| @memset(row, &castUnsupported);
        @memset(&r.shape_functions, .{});
        return r;
    }

    /// Run the registrations in order (every element of `registrations` is a type with
    /// `pub fn register(comptime r: *Registry) void`) and return the final, immutable tables
    pub fn build(comptime registrations: anytype) Registry {
        comptime {
            @setEvalBranchQuota(1_000_000);
            var r = init();
            for (registrations) |T| T.register(&r);
            return r;
        }
    }

    /// Register a collide shape function in the collision table (sRegisterCollideShape)
    pub fn registerCollideShape(self: *Registry, type1: ShapeSubType, type2: ShapeSubType, function: CollideShape) void {
        self.collide_shape[@intFromEnum(type1)][@intFromEnum(type2)] = function;
    }

    /// Register a cast shape function in the collision table (sRegisterCastShape)
    pub fn registerCastShape(self: *Registry, type1: ShapeSubType, type2: ShapeSubType, function: CastShape) void {
        self.cast_shape[@intFromEnum(type1)][@intFromEnum(type2)] = function;
    }

    /// Get an entry in the ShapeFunctions registry for a particular sub type (ShapeFunctions::sGet during registration)
    pub fn shapeFunctions(self: *Registry, sub_type: ShapeSubType) *ShapeFunctions {
        return &self.shape_functions[@intFromEnum(sub_type)];
    }

    /// The registered collide function (for tests and tools)
    pub fn getCollideShape(self: *const Registry, type1: ShapeSubType, type2: ShapeSubType) CollideShape {
        return self.collide_shape[@intFromEnum(type1)][@intFromEnum(type2)];
    }

    /// The registered cast function (for tests and tools)
    pub fn getCastShape(self: *const Registry, type1: ShapeSubType, type2: ShapeSubType) CastShape {
        return self.cast_shape[@intFromEnum(type1)][@intFromEnum(type2)];
    }
};

pub fn collideUnsupported(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    _ = .{ shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter };
    if (Core.enable_asserts) @panic("Unsupported shape pair");
}

pub fn castUnsupported(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    _ = .{ shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector };
    if (Core.enable_asserts) @panic("Unsupported shape pair");
}

/// Collide 2 shapes and pass any collision on to collector
/// @param shape1 The first shape
/// @param shape2 The second shape
/// @param scale1 Local space scale of shape 1 (scales relative to its center of mass)
/// @param scale2 Local space scale of shape 2 (scales relative to its center of mass)
/// @param center_of_mass_transform1 Transform to transform center of mass of shape 1 into world space
/// @param center_of_mass_transform2 Transform to transform center of mass of shape 2 into world space
/// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for shape 1
/// @param sub_shape_id_creator2 Class that tracks the current sub shape ID for shape 2
/// @param collide_shape_settings Options for the CollideShape test
/// @param collector The collector that receives the results.
/// @param shape_filter allows selectively disabling collisions between pairs of (sub) shapes.
pub fn collideShapeVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    // Only test shape if it passes the shape filter
    if (shape_filter.shouldCollidePair(shape1, sub_shape_id_creator1.getID(), shape2, sub_shape_id_creator2.getID()))
        RegisterTypes.registry.getCollideShape(shape1.getSubType(), shape2.getSubType())(shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
}

/// Cast a shape against this shape, passes any hits found to collector.
/// Note: This version takes the shape cast in local space relative to the center of mass of shape, take a look at castShapeVsShapeWorldSpace if you have a shape cast in world space.
pub fn castShapeVsShapeLocalSpace(shape_cast_local: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    // Only test shape if it passes the shape filter
    if (shape_filter.shouldCollidePair(shape_cast_local.shape, sub_shape_id_creator1.getID(), shape, sub_shape_id_creator2.getID()))
        RegisterTypes.registry.getCastShape(shape_cast_local.shape.getSubType(), shape.getSubType())(shape_cast_local, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
}

/// See: castShapeVsShapeLocalSpace.
/// The only difference is that the shape cast (shape_cast_world) is provided in world space.
pub fn castShapeVsShapeWorldSpace(shape_cast_world: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    const local_shape_cast = shape_cast_world.postTransformed(center_of_mass_transform2.inversedRotationTranslation());
    castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
}

/// An implementation of CollideShape that swaps shape1 and shape2 and swaps the result back, can be registered if the collision function only exists the other way around
pub fn reversedCollideShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
    // A collision collector that flips the collision results
    const ReversedCollector = struct {
        pub const overrides = .{.addHit};

        base: CollideShapeCollector,
        collector: *CollideShapeCollector,

        pub fn addHit(self: *@This(), result: *const CollideShapeResult) void {
            // Add the reversed hit
            const reversed = result.reversed();
            self.collector.addHit(&reversed);

            // If our chained collector updated its early out fraction, we need to follow
            self.base.updateEarlyOutFraction(self.collector.getEarlyOutFraction());
        }
    };

    const reversed_filter: ReversedShapeFilter = .init(shape_filter);
    var reversed_collector: ReversedCollector = .{ .base = .initFrom(ReversedCollector, collector), .collector = collector };
    collideShapeVsShape(shape2, shape1, scale2, scale1, center_of_mass_transform2, center_of_mass_transform1, sub_shape_id_creator2, sub_shape_id_creator1, collide_shape_settings, &reversed_collector.base, &reversed_filter.base);
}

/// An implementation of CastShape that swaps shape1 and shape2 and swaps the result back, can be registered if the collision function only exists the other way around
pub fn reversedCastShape(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
    // A collision collector that flips the collision results
    const ReversedCollector = struct {
        pub const overrides = .{.addHit};

        base: CastShapeCollector,
        collector: *CastShapeCollector,
        world_direction: Vec3,

        pub fn addHit(self: *@This(), result: *const ShapeCastResult) void {
            // Add the reversed hit
            const reversed = result.reversed(self.world_direction);
            self.collector.addHit(&reversed);

            // If our chained collector updated its early out fraction, we need to follow
            self.base.updateEarlyOutFraction(self.collector.getEarlyOutFraction());
        }
    };

    // Reverse the shape cast (shape cast is in local space to shape 2)
    const com_start_inv = shape_cast.center_of_mass_start.inversedRotationTranslation();
    const local_shape_cast: ShapeCast = .init(shape, scale, com_start_inv, com_start_inv.multiply3x3(shape_cast.direction).negate());

    // Calculate the center of mass of shape 1 at start of sweep
    const shape1_com = center_of_mass_transform2.mul(shape_cast.center_of_mass_start);

    // Calculate the world space direction vector of the shape cast
    const world_direction = center_of_mass_transform2.multiply3x3(shape_cast.direction).negate();

    // Forward the cast
    const reversed_filter: ReversedShapeFilter = .init(shape_filter);
    var reversed_collector: ReversedCollector = .{ .base = .initFrom(ReversedCollector, collector), .collector = collector, .world_direction = world_direction };
    castShapeVsShapeLocalSpace(&local_shape_cast, shape_cast_settings, shape_cast.shape, shape_cast.scale, &reversed_filter.base, shape1_com, sub_shape_id_creator2, sub_shape_id_creator1, &reversed_collector.base);
}
