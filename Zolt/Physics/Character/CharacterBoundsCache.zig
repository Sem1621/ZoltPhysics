//! Zolt addition, no Jolt file: a per character cache of what CharacterVsCharacterCollisionSimple computes of the other
//! characters (Roadmap Phase 12, class A: no result changes).
//! Status: complete
//!
//! CharacterVsCharacterCollisionSimple::CollideCharacter and CastCharacter (Jolt/Physics/Character/CharacterVirtual.cpp)
//! loop over all characters of the collision group. For every other character they build its center of mass transform
//! and its world space bounds, and the bounds test then rejects almost every pair. With 100 characters these loops took
//! 65% of the instructions of PerformanceTest's CharacterVirtual scene, though the other characters do not move while a
//! character queries them. This cache keeps that center of mass transform with every character, and the parts of its
//! world space bounds that do not depend on the base offset of the query (`Replay`).
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
//! - **The center of mass transform.** The cached transform is the value GetCenterOfMassTransform() returns for the
//!   key: the same function of the same bits. The loops compute `transform2 = PostTranslated(-inBaseOffset).ToMat44()`
//!   from it with Jolt's code, and pass that transform to the collision / cast of an accepted pair. A rotation that is
//!   not normalized is not cached (Jolt's Mat44::sRotation asserts on it), so a refresh never asserts where Jolt would
//!   not; neither does a replay (it checks the IsValidScale and rotation asserts of the shapes it replays).
//! - **The bounds replay.** The loops only need the world space bounds `shape2.GetWorldSpaceBounds(transform2,
//!   Vec3::sOne())` for their reject decision. For a supported chain these are a fixed sequence of float operations on
//!   the rotation columns c0, c1, c2 of transform2 (the cached bits), its translation t (`transform2.GetTranslation()`,
//!   Jolt's bits) and constants of the shapes. `Replay.build` evaluates, once per refresh, every operation that does not
//!   involve t; `Replay.bounds` evaluates, per pair, the operations that do, in Jolt's order and association:
//!   - OffsetCenterOfMassShape: `T + M.Multiply3x3(-scale * offset)` (PreTranslated), the addends in chain order;
//!   - SphereShape: `-half_extent + T`, `half_extent + T` (AABox::Translate);
//!   - CapsuleShape and TaperedCapsuleShape: `M * v` is `Multiply3x3(v) + T` lane by lane, then Jolt's min / max and
//!     the extents;
//!   - box, cylinder, tapered cylinder, convex hull (Shape::GetWorldSpaceBounds, AABox::Transformed):
//!     `((T + m0) + m1) + m2` and `((T + M0) + M1) + M2` with the per column min / max hoisted;
//!   - RotatedTranslatedShape: `M * Mat44::sRotation(q)` gives the rotation columns `r + T * 0` and the translation
//!     `((c0 * 0 + c1 * 0) + c2 * 0) + T * 1`. `build` computes them with T = 0 (the product with a translation column
//!     (0, 0, 0, 1)), and the replay keeps T.
//!
//!   Why the replayed box equals Jolt's box as real numbers (+0 equals -0) and has no NaN:
//!   1. IEEE add, sub and mul (round to nearest) give results that are equal as real numbers for operands that are
//!      equal as real numbers, with the same infinities and NaNs; only the sign of a zero result can differ. The select
//!      based Vec3 min / max (`a < b ? a : b`) choose by a comparison, which reads only real values. Vector negation
//!      (`0 - x`) and abs keep the property too. No division and no sign bit test is involved.
//!   2. Where Jolt combines T with a value that does not depend on it, the replay does the same operation in the same
//!      order on the same T (the list above).
//!   3. T enters Jolt's code in one other place: the products of RotatedTranslatedShape. For a finite T, `T * 0` is +-0
//!      and the rotation columns are finite, so `r + T * 0` equals the hoisted column `r + 0` and
//!      `((c0 * 0 + c1 * 0) + c2 * 0) + T` equals T, as real numbers. This is the only place where bits can differ: a
//!      hoisted `T * 0` term can be +0 where the exact path has -0. Facts 1 to 3 give, operation by operation, values
//!      that are equal as real numbers, up to the box.
//!   4. Guards: every hoisted value must be finite (`build`, else the shape is unsupported), and so must the last
//!      translation of the chain (`bounds`, else the loop runs Jolt's GetWorldSpaceBounds on transform2). Adding finite
//!      addends to an infinite or NaN translation never gives a finite one, so every translation of the chain is
//!      finite then, and no `inf * 0` makes a NaN column in Jolt's code. The tails only add finite hoisted values to
//!      values that depend on T (which may overflow to an infinity) and combine two such values only with min / max,
//!      so neither computation can make a NaN (that needs `inf - inf` or `inf * 0`).
//!   5. The loops use the box only in AABox::ExpandBy (sub, add), AABox::Overlaps (comparisons) and
//!      RayAABoxHits(origin, direction, min, max) (sub, mul, abs, swizzles and comparisons; a zero product compared
//!      with `>= 0` is true for both signs, and a NaN that the direction brings in arises in both computations alike).
//!      Their answers depend only on real values and on which values are NaN, so the reject / accept decision is
//!      Jolt's. The box is never stored, returned or passed on.
//! - **Counters.** In test builds the loops count what the caches decided (`Stats`, through the optional
//!   `CharacterVsCharacterCollisionSimple.stats` pointer); in other builds the counters are `void`.
//! - **Tests.** ZoltTests/Physics/CharacterBoundsCacheTests.zig (the replay against GetWorldSpaceBounds, the decisions
//!   at their flips, the Simple against an oracle copy of Jolt's loops, staleness, the pin, MutableCompoundShape) and
//!   the SimulationParity scenes character_virtual_vs_character_changes and
//!   character_virtual_vs_character_changes_far.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const math = @import("../../Math/Math.zig");
const AABox = @import("../../Geometry/AABox.zig").AABox;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const DecoratedShape = @import("../Collision/Shape/DecoratedShape.zig").DecoratedShape;
const RotatedTranslatedShape = @import("../Collision/Shape/RotatedTranslatedShape.zig").RotatedTranslatedShape;
const ScaledShape = @import("../Collision/Shape/ScaledShape.zig").ScaledShape;
const OffsetCenterOfMassShape = @import("../Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const CapsuleShape = @import("../Collision/Shape/CapsuleShape.zig").CapsuleShape;
const TaperedCapsuleShape = @import("../Collision/Shape/TaperedCapsuleShape.zig").TaperedCapsuleShape;

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
    /// The cache was used (counted in `rejected` / `accepted` too), but a translation of the replayed bounds was not
    /// finite: Jolt's GetWorldSpaceBounds ran on the cached center of mass transform
    non_finite: u64 = 0,
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

    /// Bit for bit comparison (NaN equals NaN with the same bits, -0 differs from +0): the bits that differ, or'ed together
    pub fn eql(a: *const KeyType, b: *const KeyType) bool {
        const padding_diff = @as(u32, @bitCast(a.character_padding)) ^ @as(u32, @bitCast(b.character_padding));
        const diff = bitsDiff(a.position.value, b.position.value) | bitsDiff(a.rotation.value.value, b.rotation.value.value) |
            bitsDiff(a.shape_offset.value, b.shape_offset.value) | bitsDiff(a.up.value, b.up.value) | @as(@Vector(4, u32), @splat(padding_diff));
        return a.shape == b.shape and @reduce(.Or, diff) == 0;
    }
};

/// The bits that differ between two float vectors of 128 or 256 bits, folded into 4 x u32 (zero when they are equal bit
/// for bit)
inline fn bitsDiff(a: anytype, b: @TypeOf(a)) @Vector(4, u32) {
    const num_words = @bitSizeOf(@TypeOf(a)) / 32;
    const U = @Vector(num_words, u32);
    const d = @as(U, @bitCast(a)) ^ @as(U, @bitCast(b));
    return switch (num_words) {
        4 => d,
        8 => @shuffle(u32, d, undefined, @Vector(4, i32){ 0, 1, 2, 3 }) | @shuffle(u32, d, undefined, @Vector(4, i32){ 4, 5, 6, 7 }),
        else => @compileError("bitsDiff: a 128 or 256 bit vector"),
    };
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

/// The world space bounds of a shape (Shape::GetWorldSpaceBounds(inCenterOfMassTransform, Vec3::sOne())) as a function of
/// the translation of `inCenterOfMassTransform`, for one rotation: everything that does not depend on the translation
/// is computed once (`build`), the operations that involve it run per pair (`bounds`) in Jolt's order. See the file
/// header for why the result equals Jolt's as real numbers.
const ReplayType = struct {
    /// The most OffsetCenterOfMassShapes in a chain of decorated shapes
    pub const max_addends = 2;

    /// The code of the convex shape at the end of the chain
    pub const Tail = enum(u8) {
        /// SphereShape: AABox(-half extent, half extent).Translate(T): min = v[0] + T, max = v[1] + T
        translated,
        /// CapsuleShape: p1 = v[0] + T, p2 = v[1] + T (M * -height, M * height), min(p1, p2) - v[2], max(p1, p2) + v[2]
        capsule,
        /// TaperedCapsuleShape: top = v[0] + T, bottom = v[1] + T, min(top - v[2], bottom - v[3]), max(top + v[2],
        /// bottom + v[3])
        tapered_capsule,
        /// Shape::GetWorldSpaceBounds, AABox::Transformed: min = ((T + v[0]) + v[1]) + v[2], max = ((T + v[3]) + v[4]) +
        /// v[5] (v[c] = min(col_c * min_c, col_c * max_c), v[3 + c] = max(...) of the scaled local bounds)
        transformed,
    };

    tail: Tail,
    /// The translations that OffsetCenterOfMassShape::GetWorldSpaceBounds adds (PreTranslated: T + M.Multiply3x3(-scale
    /// * offset)), in chain order
    num_addends: u8,
    addends: [max_addends]Vec3,
    /// The values of the tail
    v: [6]Vec3,

    /// Compute the parts of the bounds of `shape_in` that do not depend on the translation, for a center of mass
    /// transform with the rotation columns of `rotation` (its translation is ignored). Returns false for an unsupported
    /// chain, a value that Jolt would assert on, or a value that is not finite.
    pub fn build(self: *ReplayType, shape_in: *const Shape, rotation: Mat44) bool {
        // The rotation of the current transform (its translation column stays (0, 0, 0, 1): the translation is replayed)
        var m = rotation;
        var scale = Vec3.one();
        var shape = shape_in;
        self.num_addends = 0;
        var depth: u32 = 0;
        while (true) : (depth += 1) {
            switch (shape.getSubType()) {
                .rotated_translated => {
                    if (depth == max_decorator_depth) return false;
                    // RotatedTranslatedShape::GetWorldSpaceBounds: inCenterOfMassTransform * Mat44::sRotation(mRotation),
                    // TransformScale(inScale). The translation column of the product is ((c0 * 0 + c1 * 0) + c2 * 0) + T * 1
                    // (T as a real number) and its rotation columns get + T * 0 (+-0 for a finite T)
                    const rt = shape.cast(RotatedTranslatedShape);
                    if (!rt.getRotation().isNormalized(.{})) return false; // Mat44::sRotation asserts
                    m = m.mul(Mat44.rotationQuat(rt.getRotation()));
                    scale = rt.transformScale(scale);
                    shape = rt.base.getInnerShape() orelse return false;
                },
                .scaled => {
                    if (depth == max_decorator_depth) return false;
                    // ScaledShape::GetWorldSpaceBounds: inScale * mScale
                    const scaled = shape.cast(ScaledShape);
                    scale = scale.mul(scaled.scale);
                    shape = scaled.base.getInnerShape() orelse return false;
                },
                .offset_center_of_mass => {
                    if (depth == max_decorator_depth or self.num_addends == max_addends) return false;
                    // OffsetCenterOfMassShape::GetWorldSpaceBounds: inCenterOfMassTransform.PreTranslated(-inScale * mOffset)
                    const offset = shape.cast(OffsetCenterOfMassShape);
                    self.addends[self.num_addends] = m.multiply3x3(scale.negate().mul(offset.offset));
                    self.num_addends += 1;
                    shape = offset.base.getInnerShape() orelse return false;
                },
                .sphere => {
                    // SphereShape::GetWorldSpaceBounds (GetScaledRadius asserts IsValidScale)
                    const sphere = shape.cast(SphereShape);
                    if (!sphere.isValidScale(scale)) return false;
                    const abs_scale = scale.abs();
                    const scaled_radius = abs_scale.getX() * sphere.radius;
                    const half_extent = Vec3.replicate(scaled_radius);
                    self.tail = .translated;
                    self.v[0] = half_extent.negate();
                    self.v[1] = half_extent;
                    return self.isFiniteBuild(2);
                },
                .capsule => {
                    // CapsuleShape::GetWorldSpaceBounds (asserts IsValidScale)
                    const capsule = shape.cast(CapsuleShape);
                    if (!capsule.isValidScale(scale)) return false;
                    const abs_scale = scale.abs();
                    const uniform_scale = abs_scale.getX();
                    const extent = Vec3.replicate(uniform_scale * capsule.radius);
                    const height = Vec3.init(0, uniform_scale * capsule.half_height_of_cylinder, 0);
                    self.tail = .capsule;
                    self.v[0] = m.multiply3x3(height.negate());
                    self.v[1] = m.multiply3x3(height);
                    self.v[2] = extent;
                    return self.isFiniteBuild(3);
                },
                .tapered_capsule => {
                    // TaperedCapsuleShape::GetWorldSpaceBounds (asserts IsValidScale)
                    const tapered = shape.cast(TaperedCapsuleShape);
                    if (!tapered.isValidScale(scale)) return false;
                    const abs_scale = scale.abs();
                    const scale_xz = abs_scale.getX();
                    const scale_y = scale.getY(); // The sign of y is important as it flips the tapered capsule
                    const bottom_extent = Vec3.replicate(scale_xz * tapered.bottom_radius);
                    const bottom_center = m.multiply3x3(Vec3.init(0, scale_y * tapered.bottom_center, 0));
                    const top_extent = Vec3.replicate(scale_xz * tapered.top_radius);
                    const top_center = m.multiply3x3(Vec3.init(0, scale_y * tapered.top_center, 0));
                    self.tail = .tapered_capsule;
                    self.v[0] = top_center;
                    self.v[1] = bottom_center;
                    self.v[2] = top_extent;
                    self.v[3] = bottom_extent;
                    return self.isFiniteBuild(4);
                },
                .box, .cylinder, .tapered_cylinder, .convex_hull => {
                    // Shape::GetWorldSpaceBounds: GetLocalBounds().Scaled(inScale).Transformed(inCenterOfMassTransform)
                    const local_bounds = shape.getLocalBounds().scaled(scale);
                    for (0..3) |c| {
                        const col = m.getColumn3(@intCast(c));
                        const a = col.mulScalar(local_bounds.min.getComponent(@intCast(c)));
                        const b = col.mulScalar(local_bounds.max.getComponent(@intCast(c)));
                        self.v[c] = Vec3.min(a, b);
                        self.v[3 + c] = Vec3.max(a, b);
                    }
                    self.tail = .transformed;
                    return self.isFiniteBuild(6);
                },
                else => return false,
            }
        }
    }

    /// Every value of the replay is finite
    fn isFiniteBuild(self: *const ReplayType, num_values: usize) bool {
        for (self.addends[0..self.num_addends]) |a| {
            if (!isFinite(a)) return false;
        }
        for (self.v[0..num_values]) |v| {
            if (!isFinite(v)) return false;
        }
        return true;
    }

    /// Shape::GetWorldSpaceBounds(inCenterOfMassTransform, Vec3::sOne()) of the shape of `build` as real numbers, for a
    /// transform with the rotation of `build` and the translation `t`. Null when a translation of the chain is not
    /// finite: then the caller must run Jolt's code.
    pub inline fn bounds(self: *const ReplayType, t: Vec3) ?AABox {
        var translation = t;
        if (self.num_addends != 0) {
            for (self.addends[0..self.num_addends]) |a|
                translation = translation.add(a);
        }
        if (!isFinite(translation))
            return null;
        const v = &self.v;
        switch (self.tail) {
            .translated => return .init(v[0].add(translation), v[1].add(translation)),
            .capsule => {
                const p1 = v[0].add(translation);
                const p2 = v[1].add(translation);
                return .init(Vec3.min(p1, p2).sub(v[2]), Vec3.max(p1, p2).add(v[2]));
            },
            .tapered_capsule => {
                const top_center = v[0].add(translation);
                const bottom_center = v[1].add(translation);
                return .init(Vec3.min(top_center.sub(v[2]), bottom_center.sub(v[3])), Vec3.max(top_center.add(v[2]), bottom_center.add(v[3])));
            },
            .transformed => return .init(translation.add(v[0]).add(v[1]).add(v[2]), translation.add(v[3]).add(v[4]).add(v[5])),
        }
    }
};

/// All components are finite (no infinity, no NaN); W is Z in a Vec3
inline fn isFinite(v: Vec3) bool {
    return @reduce(.And, @abs(v.value) <= @as(Vec3.Type, @splat(math.flt_max)));
}

/// The rotation columns of a center of mass transform, with the translation column (0, 0, 0, 1)
fn rotationColumns(transform: RMat44) Mat44 {
    return .init(transform.col[0], transform.col[1], transform.col[2], Vec4.init(0, 0, 0, 1));
}

/// The cache of one character (`CharacterVirtual.bounds_cache`)
pub const CharacterBoundsCache = struct {
    pub const Stats = StatsType;
    pub const Key = KeyType;
    pub const Replay = ReplayType;
    pub const supported_shape = supportedShape;

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
    /// The world space bounds of the shape for the rotation of `center_of_mass_transform` (when `state` is `.valid`)
    replay: ReplayType = undefined,

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
            if (self.replay.build(key.shape.?, rotationColumns(transform))) {
                self.center_of_mass_transform = transform;
                self.state = .valid;
                return;
            }
        }
        self.state = .unsupported;
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
