//! Zolt addition, no Jolt file: a per character cache of what CharacterVsCharacterCollisionSimple computes of the other
//! characters (Roadmap Phase 12, class A: no result changes).
//! Status: complete
//!
//! CharacterVsCharacterCollisionSimple::CollideCharacter and CastCharacter (Jolt/Physics/Character/CharacterVirtual.cpp)
//! loop over all characters of the collision group. For every other character they build its center of mass transform
//! and its world space bounds, and the bounds test then rejects almost every pair. With 100 characters these loops took
//! 65% of the instructions of PerformanceTest's CharacterVirtual scene, though the other characters do not move while a
//! character queries them. This cache keeps that center of mass transform with every character.
//!
//! - **Owner and readers.** A CharacterVirtual owns its cache (`CharacterVirtual.bounds_cache`) and is the only one
//!   that writes it (`CharacterVirtual.refreshBoundsCache`). The loops of CharacterVsCharacterCollisionSimple read the
//!   caches of the other characters. Jolt's loops are not thread safe either: only one character checks collision at
//!   a time.
//! - **Key.** The cached values are a pure function of the key: the position, the rotation, the shape, the shape
//!   offset, the up vector and the character padding (GetCenterOfMassTransform uses exactly these). A reader compares
//!   the key with the character's current members bit for bit, every time (`find`). Any difference (a direct write to
//!   the public fields, CharacterBase.setUp, `base.shape.set`, the in place position of MoveShape before its
//!   UpdateInnerBodyTransform, an error between a write and a refresh) makes the cache stale, and the loop runs Jolt's
//!   code for that pair. So the refresh points only decide how often the cache hits, never a result.
//! - **Shape.** The cache holds a reference to the shape of its key (`pin`), so that the address cannot be freed and
//!   reused by another shape while the key compares it. A shape is only cached when the values it contributes cannot
//!   change after its creation (`supportedShape`): the convex shapes sphere, box, capsule, tapered capsule, cylinder,
//!   tapered cylinder and convex hull, and RotatedTranslatedShape, ScaledShape and OffsetCenterOfMassShape around them
//!   (at most `max_decorator_depth` levels). Everything else is unsupported and takes Jolt's code: compounds (a
//!   MutableCompoundShape changes in place), meshes, height fields, triangles, planes, empty shapes, soft body shapes and
//!   user shapes. A null shape is never cached.
//! - **Refresh points.** The end of the construction, UpdateInnerBodyTransform (SetPosition, SetRotation,
//!   SetShapeOffset, Update, MoveToContact), SetShape (on every exit) and RestoreState (also on its error path). A
//!   refresh with an unchanged key does nothing. `deinit` releases the reference (the destructor of the character).
//! - **Exactness.** The cached center of mass transform is the value GetCenterOfMassTransform() returns for the key:
//!   the same function of the same bits. The loops then compute `PostTranslated(-inBaseOffset).ToMat44()` and the
//!   bounds of the other character from it with Jolt's code. A rotation that is not normalized is not cached (Jolt's
//!   Mat44::sRotation asserts on it), so a refresh never asserts where Jolt would not.
//! - **Counters.** In test builds the loops count what the caches decided (`Stats`, through the optional
//!   `CharacterVsCharacterCollisionSimple.stats` pointer); in other builds the counters are `void`.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const DecoratedShape = @import("../Collision/Shape/DecoratedShape.zig").DecoratedShape;
const RotatedTranslatedShape = @import("../Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
const ScaledShape = @import("../Collision/Shape/ScaledShape.zig").ScaledShape;
const OffsetCenterOfMassShape = @import("../Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;

/// What the bounds caches decided in the loops of CharacterVsCharacterCollisionSimple (test builds only)
const StatsType = struct {
    /// The cache of the other character was used and the pair was rejected (no overlap / no ray hit)
    rejected: u64 = 0,
    /// The cache of the other character was used and the pair goes on to the collision / cast
    accepted: u64 = 0,
    /// The cache of the other character does not match its current members: Jolt's code ran
    stale: u64 = 0,
    /// The shape (or the rotation) of the other character is not supported: Jolt's code ran
    unsupported: u64 = 0,
};

/// The most decorated shapes (RotatedTranslatedShape, ScaledShape, OffsetCenterOfMassShape) around a convex shape
pub const max_decorator_depth = 4;

/// The members of a character that the cached values are a function of
const KeyType = struct {
    position: RVec3,
    rotation: Quat,
    shape: ?*const Shape,
    shape_offset: Vec3,
    up: Vec3,
    character_padding: f32,

    /// Bit for bit comparison (NaN equals NaN with the same bits, -0 differs from +0)
    pub fn eql(a: *const KeyType, b: *const KeyType) bool {
        return a.shape == b.shape and
            @as(u32, @bitCast(a.character_padding)) == @as(u32, @bitCast(b.character_padding)) and
            bitsEql(a.position.value, b.position.value) and
            bitsEql(a.rotation.value.value, b.rotation.value.value) and
            bitsEql(a.shape_offset.value, b.shape_offset.value) and
            bitsEql(a.up.value, b.up.value);
    }
};

/// Bit for bit equality of two float vectors
inline fn bitsEql(a: anytype, b: @TypeOf(a)) bool {
    const info = @typeInfo(@TypeOf(a)).vector;
    const U = @Vector(info.len, std.meta.Int(.unsigned, @bitSizeOf(info.child)));
    return @reduce(.And, @as(U, @bitCast(a)) == @as(U, @bitCast(b)));
}

/// The state of a cache
pub const State = enum(u8) {
    /// Never refreshed (or deinitialized): every read is stale
    empty,
    /// The key is stored, but its shape or rotation is not supported: Jolt's code runs
    unsupported,
    /// The key and the values are stored
    valid,
};

/// A shape whose center of mass (and bounds) cannot change after its creation, see the file header
pub fn supportedShape(shape: *const Shape) bool {
    var s = shape;
    var depth: u32 = 0;
    while (true) : (depth += 1) {
        switch (s.getSubType()) {
            .sphere, .box, .capsule, .tapered_capsule, .cylinder, .tapered_cylinder, .convex_hull => return true,
            .rotated_translated, .scaled, .offset_center_of_mass => {
                if (depth == max_decorator_depth)
                    return false;
                s = innerShape(s) orelse return false;
            },
            else => return false,
        }
    }
}

/// The inner shape of a decorated shape
fn innerShape(shape: *const Shape) ?*const Shape {
    return switch (shape.getSubType()) {
        .rotated_translated => shape.cast(RotatedTranslatedShape).base.getInnerShape(),
        .scaled => shape.cast(ScaledShape).base.getInnerShape(),
        .offset_center_of_mass => shape.cast(OffsetCenterOfMassShape).base.getInnerShape(),
        else => null,
    };
}

/// The cache of one character (`CharacterVirtual.bounds_cache`)
pub const CharacterBoundsCache = struct {
    pub const Stats = StatsType;
    pub const Key = KeyType;

    /// See State
    state: State = .empty,
    /// The key of the stored values (undefined while `state` is `.empty`; its shape is `pin`)
    position: RVec3 = undefined,
    rotation: Quat = undefined,
    shape_offset: Vec3 = undefined,
    up: Vec3 = undefined,
    character_padding: f32 = undefined,
    /// The shape of the key, with a reference so that its address is not reused while it is cached
    pin: RefConst(Shape) = .empty,
    /// GetCenterOfMassTransform() of the character for the key (when `state` is `.valid`)
    center_of_mass_transform: RMat44 = undefined,

    /// Release the reference to the shape (the destructor of the character)
    pub fn deinit(self: *CharacterBoundsCache) void {
        self.pin.deinit();
        self.state = .empty;
    }

    /// The stored key (`state` is not `.empty`)
    fn storedKey(self: *const CharacterBoundsCache) Key {
        return .{ .position = self.position, .rotation = self.rotation, .shape = self.pin.get(), .shape_offset = self.shape_offset, .up = self.up, .character_padding = self.character_padding };
    }

    /// The key needs a refresh (it differs from the stored one, or nothing is stored)
    pub fn needsRefresh(self: *const CharacterBoundsCache, key: *const Key) bool {
        return self.state == .empty or !self.storedKey().eql(key);
    }

    /// Does the key support caching: a supported shape and a normalized rotation (Mat44::sRotation asserts on others)
    pub fn supports(key: *const Key) bool {
        const shape = key.shape orelse return false;
        return key.rotation.isNormalized(.{}) and supportedShape(shape);
    }

    /// Store `key` with its center of mass transform (GetCenterOfMassTransform() of the character for the key; null when
    /// `supports(key)` is false)
    pub fn store(self: *CharacterBoundsCache, key: *const Key, center_of_mass_transform: ?RMat44) void {
        self.position = key.position;
        self.rotation = key.rotation;
        self.shape_offset = key.shape_offset;
        self.up = key.up;
        self.character_padding = key.character_padding;
        self.pin.set(key.shape);
        if (center_of_mass_transform) |transform| {
            self.center_of_mass_transform = transform;
            self.state = .valid;
        } else {
            self.state = .unsupported;
        }
    }

    /// The result of `find`
    pub const Find = enum { valid, stale, unsupported };

    /// Can the loops use the stored values for a character whose current members are `key`
    pub inline fn find(self: *const CharacterBoundsCache, key: *const Key) Find {
        if (self.state == .empty or !self.storedKey().eql(key))
            return .stale;
        return if (self.state == .valid) .valid else .unsupported;
    }
};
