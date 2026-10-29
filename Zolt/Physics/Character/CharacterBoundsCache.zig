//! Zolt addition, no Jolt file: a per character cache of what CharacterVsCharacterCollisionSimple computes of the other
//! characters (Roadmap Phase 12, class A: no result changes).
//! Status: complete
//!
//! CharacterVsCharacterCollisionSimple::CollideCharacter and CastCharacter (Jolt/Physics/Character/CharacterVirtual.cpp)
//! loop over all characters of the collision group. For every other character they build its center of mass transform
//! and its world space bounds, and the bounds test then rejects almost every pair. With 100 characters these loops took
//! 65% of the instructions of the physics updates and the character updates (UpdateTest) of PerformanceTest's
//! CharacterVirtual scene, though the other characters do not move while a character queries them. This cache keeps
//! that center of mass transform with every character, and the parts of its world space bounds that do not depend on
//! the base offset of the query (`Replay`).
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
//! - **Shape.** The cache holds a reference to the shape of its stored key (the pin, `m.shape`), so that the address
//!   cannot be freed and reused by another shape while the key compares it. The cache is an extern struct (Layout
//!   below), which cannot hold a RefConst, so the pin is a raw pointer whose references are taken and released by hand,
//!   in one place (`setPin`), with RefConst's own code: RefConst.set on a view of the pointer (a reference to the new
//!   shape first, then the old one released; nothing when the shape is the same). `store` points the pin to the shape
//!   of every key it stores, `deinit` to null. Every stored key holds it, whatever the shape (a supported or an
//!   unsupported one, see `store`), for as long as it is the stored key: the reference count of a shape
//!   (Shape.getRefCount) is higher than in Jolt by the number of character caches that hold it (each character has its
//!   own cache, so a shape that N characters share counts N more), and after a direct write to `base.shape` the old
//!   shape stays alive until the next refresh point or the destructor. The values are only cached when the shape
//!   cannot change them after its creation (`CharacterBoundsCache.supportedShape`): the convex shapes sphere, box,
//!   capsule, tapered capsule, cylinder, tapered cylinder and convex hull, and RotatedTranslatedShape, ScaledShape and
//!   OffsetCenterOfMassShape around them (at most `max_decorator_depth` levels). Everything else is unsupported and
//!   takes Jolt's code: compounds (a MutableCompoundShape changes in place), meshes, height fields, triangles, planes,
//!   empty shapes, soft body shapes and user shapes. A null shape is never cached (and holds no reference).
//! - **Refresh points.** The end of the construction, UpdateInnerBodyTransform (SetPosition, SetRotation,
//!   SetShapeOffset, Update, MoveToContact), SetShape (on every exit) and RestoreState (also on its error path). A
//!   refresh with an unchanged key does nothing. `deinit` releases the reference (the destructor of the character).
//! - **The center of mass transform.** The cached transform is the value GetCenterOfMassTransform() returns for the
//!   key: the same function of the same bits. Jolt's loops build `transform2 = PostTranslated(-inBaseOffset).ToMat44()`
//!   for every pair, but the bounds replay reads only its translation. So with a valid cache the loops compute just
//!   that translation (`translation2`, from the cached `translation`: Jolt's bits in X, Y and Z, see translation2 for
//!   W), and build transform2 with Jolt's code only where Jolt's transform2 is used: for an accepted pair (the
//!   collision / cast gets it, from the cached transform), and, from the cached or the live transform, for a stale or
//!   unsupported cache and before Jolt's GetWorldSpaceBounds on the non finite path. A rotation that is not normalized
//!   is not cached (Jolt's Mat44::sRotation asserts on it), so a refresh never asserts where Jolt would not; neither
//!   does a replay (it checks the IsValidScale and rotation asserts of the shapes it replays).
//! - **The bounds replay.** The loops only need the world space bounds `shape2.GetWorldSpaceBounds(transform2,
//!   Vec3::sOne())` for their reject decision. For a supported chain these are a fixed sequence of float operations on
//!   the rotation columns c0, c1, c2 of transform2 (the cached bits), its translation t (`transform2.GetTranslation()`,
//!   Jolt's bits in X, Y and Z, from `translation2`) and constants of the shapes. `Replay.build` evaluates, once per
//!   refresh, every operation that does not involve t; `Replay.bounds` evaluates, per pair, the operations that do, in
//!   Jolt's order and association:
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
//! - **Layout (Roadmap Phase 14, class A, layout only).** With 100 characters the lines that the loops read of the other
//!   characters do not stay in the L1 cache from one query to the next, so every cache line that a rejected pair reads
//!   is a miss. Such a pair reads the key twice (the live members of the character and the stored copy), the state,
//!   the cached translation and the head of the replay. The cache is therefore an extern struct with a field order per
//!   precision (`SingleLayout`, `DoubleLayout`, offsets asserted at compile time), and CharacterVirtual holds it
//!   `align(64)`: Zig's auto layout puts the most aligned field first, so the cache is at offset 0 and the character
//!   is 64 byte aligned. CharacterVirtual's comptime block pins where its own members land behind the cache. For a
//!   sphere or a capsule (also inside RotatedTranslatedShape / ScaledShape, like PerformanceTest's characters), a
//!   rejected pair reads 6 cache lines of the other character in single precision (Phase 13's layout: 9):
//!   - [0, 64): the stored position, rotation, shape offset and up;
//!   - [64, 128): the head of the replay (`tail`, `num_addends`, `v[0..2]`);
//!   - [320, 384): shape, padding, state and translation of the cache (the 352 byte cache ends there), together with
//!     CharacterBase.up, which the compare reads anyway;
//!   - CharacterBase.shape; rotation, position and shape offset (one line); the character padding.
//!   In double precision RVec3 is 32 bytes and no line can be shared: the cache reads [0, 192), the character's members
//!   5 more lines, 8 in all. The gain depends on the shape: a tapered capsule and the transformed tail (box, cylinder,
//!   tapered cylinder, convex hull) read `v[3..5]` from the next replay line, and OffsetCenterOfMassShape addends one or
//!   two lines more (CharacterBoundsCacheTests counts them). A new member of CharacterBase or CharacterVirtual, or a
//!   compiler that orders the fields differently, fails to compile; PortingGuide.md section 5 ("Line pinned layouts")
//!   says how to re-tune.
//! - **Counters.** In test builds the loops count what the caches decided (`Stats`, through the optional
//!   `CharacterVsCharacterCollisionSimple.stats` pointer); in other builds the counters are `void`.
//! - **Tests.** ZoltTests/Physics/CharacterBoundsCacheTests.zig (the replay against GetWorldSpaceBounds, the decisions
//!   at their flips and on boxes that differ only in the signs of zeros (fact 5), `translation2` against Jolt's
//!   transform2 bit for bit (signed zeros, roundings, a base offset whose W differs from its Z), the Simple against an
//!   oracle copy of Jolt's loops (the hits and the accept decisions, also through the `non_finite` fallback),
//!   staleness, the pin and its reference counts, `find` in every state, the alignment and the cache lines of the
//!   layout, MutableCompoundShape) and the SimulationParity scenes character_virtual_vs_character_changes and
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

    /// Bit for bit comparison (NaN equals NaN with the same bits, -0 differs from +0): the shape addresses, the bits of
    /// the padding, and the bits that differ in the vectors, or'ed together and tested as one 128 bit integer
    pub fn eql(a: *const KeyType, b: *const KeyType) bool {
        const diff = bitsDiff(a.position.value, b.position.value) | bitsDiff(a.rotation.value.value, b.rotation.value.value) |
            bitsDiff(a.shape_offset.value, b.shape_offset.value) | bitsDiff(a.up.value, b.up.value);
        return a.shape == b.shape and @as(u32, @bitCast(a.character_padding)) == @as(u32, @bitCast(b.character_padding)) and
            @as(u128, @bitCast(diff)) == 0;
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
const StateType = enum(u8) {
    /// Never refreshed (or deinitialized): every read is stale
    empty,
    /// The key is stored, but its shape or rotation is not supported: Jolt's code runs
    unsupported,
    /// The key and the values are stored
    valid,
};

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
///
/// An extern struct (a member of the extern cache layouts) with `tail`, `num_addends` and `v` first: the part that a
/// sphere or a capsule reads, `tail` to `v[2]`, is its first 64 bytes, one cache line (file header, Layout).
const ReplayType = extern struct {
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
    /// The number of `addends`
    num_addends: u8,
    /// The values of the tail
    v: [6]Vec3,
    /// The translations that OffsetCenterOfMassShape::GetWorldSpaceBounds adds (PreTranslated: T + M.Multiply3x3(-scale
    /// * offset)), in chain order
    addends: [max_addends]Vec3,

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

/// The members of a cache in single precision, in a fixed order (an extern struct; the offsets are asserted below), so
/// that a rejected pair of the loops of CharacterVsCharacterCollisionSimple reads few cache lines (file header, Layout).
/// CharacterVirtual holds the cache 64 byte aligned at its offset 0 (`bounds_cache`). A rejected pair with a valid cache
/// reads:
/// - [0, 64): the stored `position`, `rotation`, `shape_offset` and `up`;
/// - [64, 128): the head of the replay, `tail` to `v[2]` (a sphere or a capsule reads no more of it);
/// - [320, 352): `shape`, `character_padding`, `state` and `translation`. The cache is 352 bytes (32 mod 64, set by
///   `reserved`), so this block shares the line [320, 384) with CharacterBase.up at offset 352 of CharacterVirtual,
///   which the key compare reads too, and CharacterVirtual's rotation, position and shape offset, which Zig's layout
///   puts behind the cache, share one line (see CharacterVirtual's comptime block).
const SingleLayout = extern struct {
    /// The key of the stored values (undefined while `state` is `.empty`; its shape is `shape`)
    position: Vec3 = undefined,
    rotation: Quat = undefined,
    shape_offset: Vec3 = undefined,
    up: Vec3 = undefined,
    /// The world space bounds of the shape for the rotation of `center_of_mass_transform` (when `state` is `.valid`)
    replay: ReplayType = undefined,
    /// GetCenterOfMassTransform() of the character for the key (when `state` is `.valid`); the loops read it only for an
    /// accepted pair
    center_of_mass_transform: Mat44 = undefined,
    /// Makes the cache 352 bytes: the block from `shape` to `translation` ends the cache (see above)
    reserved: [48]u8 = undefined,
    /// The shape of the key (the pin: it holds a reference, see `setPin`, so that the address is not reused while it is
    /// cached; null when nothing is stored)
    shape: ?*const Shape = null,
    character_padding: f32 = undefined,
    /// See CharacterBoundsCache.State
    state: StateType = .empty,
    /// `center_of_mass_transform.getTranslation()` (when `state` is `.valid`): `translation2` adds the negated base
    /// offset to it
    translation: Vec3 = undefined,

    comptime {
        std.debug.assert(@offsetOf(SingleLayout, "position") == 0 and @offsetOf(SingleLayout, "rotation") == 16 and @offsetOf(SingleLayout, "shape_offset") == 32 and @offsetOf(SingleLayout, "up") == 48);
        std.debug.assert(@offsetOf(SingleLayout, "replay") == 64 and @offsetOf(ReplayType, "tail") == 0 and @offsetOf(ReplayType, "num_addends") == 1 and @offsetOf(ReplayType, "v") == 16);
        std.debug.assert(@offsetOf(SingleLayout, "center_of_mass_transform") == 208 and @offsetOf(SingleLayout, "shape") == 320);
        if (@sizeOf(usize) == 8) std.debug.assert(@offsetOf(SingleLayout, "character_padding") == 328 and @offsetOf(SingleLayout, "state") == 332);
        std.debug.assert(@offsetOf(SingleLayout, "translation") == 336 and @sizeOf(SingleLayout) == 352);
    }
};

/// The members of a cache in double precision (see SingleLayout). RVec3 is 32 bytes and CharacterBase starts with a
/// DVec3, so no line can be shared with the character. A rejected pair with a valid cache reads [0, 64) the stored
/// `position`, `rotation` and `shape_offset`; [64, 128) `up`, `shape`, `character_padding`, `state` and `translation`;
/// [128, 192) the head of the replay, `tail` to `v[2]`.
const DoubleLayout = extern struct {
    /// The key of the stored values (undefined while `state` is `.empty`; its shape is `shape`)
    position: RVec3 = undefined,
    rotation: Quat = undefined,
    shape_offset: Vec3 = undefined,
    up: Vec3 = undefined,
    /// The shape of the key (the pin: it holds a reference, see `setPin`, so that the address is not reused while it is
    /// cached; null when nothing is stored)
    shape: ?*const Shape = null,
    character_padding: f32 = undefined,
    /// See CharacterBoundsCache.State
    state: StateType = .empty,
    /// `center_of_mass_transform.getTranslation()` (when `state` is `.valid`): `translation2` adds the negated base
    /// offset to it
    translation: RVec3 = undefined,
    /// The world space bounds of the shape for the rotation of `center_of_mass_transform` (when `state` is `.valid`)
    replay: ReplayType = undefined,
    /// GetCenterOfMassTransform() of the character for the key (when `state` is `.valid`); the loops read it only for an
    /// accepted pair
    center_of_mass_transform: RMat44 = undefined,

    comptime {
        if (Core.double_precision) {
            std.debug.assert(@offsetOf(DoubleLayout, "position") == 0 and @offsetOf(DoubleLayout, "rotation") == 32 and @offsetOf(DoubleLayout, "shape_offset") == 48);
            std.debug.assert(@offsetOf(DoubleLayout, "up") == 64 and @offsetOf(DoubleLayout, "shape") == 80);
            if (@sizeOf(usize) == 8) std.debug.assert(@offsetOf(DoubleLayout, "character_padding") == 88 and @offsetOf(DoubleLayout, "state") == 92);
            std.debug.assert(@offsetOf(DoubleLayout, "translation") == 96 and @offsetOf(DoubleLayout, "replay") == 128 and @offsetOf(ReplayType, "v") == 16);
            std.debug.assert(@sizeOf(DoubleLayout) == 384);
        }
    }
};

/// The cache of one character (`CharacterVirtual.bounds_cache`)
pub const CharacterBoundsCache = extern struct {
    pub const Stats = StatsType;
    pub const Key = KeyType;
    pub const State = StateType;
    pub const Replay = ReplayType;
    pub const Layout = if (Core.double_precision) DoubleLayout else SingleLayout;

    /// The members, see SingleLayout and DoubleLayout
    m: Layout = .{},

    /// Release the reference to the shape (the destructor of the character)
    pub fn deinit(self: *CharacterBoundsCache) void {
        self.setPin(null);
        self.m.state = .empty;
    }

    /// Point the pin (`m.shape`) to `shape`, the one place that changes its references. The extern layout cannot hold a
    /// RefConst(Shape), so this runs RefConst.set on a view of the raw pointer, which owns the pin's reference: a
    /// reference to the new shape first, then the old one released (which can destroy it); nothing when they are the
    /// same.
    fn setPin(self: *CharacterBoundsCache, shape: ?*const Shape) void {
        var pin: RefConst(Shape) = .{ .ptr = self.m.shape };
        pin.set(shape);
        self.m.shape = pin.ptr;
    }

    /// The stored key (`state` is not `.empty`)
    fn storedKey(self: *const CharacterBoundsCache) Key {
        const m = &self.m;
        return .{ .position = m.position, .rotation = m.rotation, .shape = m.shape, .shape_offset = m.shape_offset, .up = m.up, .character_padding = m.character_padding };
    }

    /// The key needs a refresh (it differs from the stored one, or nothing is stored)
    pub fn needsRefresh(self: *const CharacterBoundsCache, key: *const Key) bool {
        return self.m.state == .empty or !self.storedKey().eql(key);
    }

    /// The translation of the loops' `transform2 = center_of_mass_transform.postTranslatedRVec3(base_offset.negate())
    /// .toMat44()`, computed without transform2 (`state` is `.valid`). It equals `transform2.getTranslation()` bit for
    /// bit in X, Y and Z, and in W too when the W of `base_offset` equals its Z (every RVec3 that Vec3 / DVec3
    /// arithmetic makes; only a vector built from raw lanes can differ):
    /// - single precision: Mat44.postTranslated stores `getTranslation().add(base_offset.negate())` in column 3 with
    ///   W = 1, toMat44 is the identity and getTranslation copies Z into W. `m.translation` is getTranslation() of the
    ///   same cached transform (W = Z), so this is the same lane by lane addition of the same operands. Its W is
    ///   `translation.Z + (0 - base_offset.W)`, which is Jolt's W, `translation.Z + (0 - base_offset.Z)`, when the W of
    ///   base_offset equals its Z;
    /// - double precision: DMat44.postTranslatedDVec3 adds the DVec3s, toMat44 rounds the sum with DVec3.toVec3 (one
    ///   rounding to nearest per lane, then Z copied into W) and getTranslation copies Z into W again. Here the same
    ///   addition and the same toVec3: all 4 lanes are Jolt's, whatever the W of base_offset.
    /// The W lane reaches only the finite test of `Replay.bounds` and the W lanes of the box, which no decision reads
    /// (AABox.overlaps and rayAABoxHitsDirection test X, Y and Z; ExpandBy works lane by lane). A W that is not finite
    /// makes `bounds` return null, and the loop then runs Jolt's GetWorldSpaceBounds on Jolt's transform2: a W that
    /// differs from Jolt's can change the counters (`non_finite`), never a decision or a result.
    /// The negation is `0 - base_offset` (`negate`), as in Jolt: `m.translation.sub(base_offset)` would differ in the
    /// sign of a zero sum (`-0 + (0 - 0) = +0`, `-0 - 0 = -0`).
    pub inline fn translation2(self: *const CharacterBoundsCache, base_offset: RVec3) Vec3 {
        const sum = self.m.translation.add(base_offset.negate());
        return if (Core.double_precision) sum.toVec3() else sum;
    }

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

    /// Does the key support caching: a supported shape and a normalized rotation (Mat44::sRotation asserts on others)
    pub fn supports(key: *const Key) bool {
        const shape = key.shape orelse return false;
        return key.rotation.isNormalized(.{}) and supportedShape(shape);
    }

    /// Store `key` with its center of mass transform (GetCenterOfMassTransform() of the character for the key; null when
    /// `supports(key)` is false). The pin takes a reference to the shape of every stored key, also an unsupported one.
    pub fn store(self: *CharacterBoundsCache, key: *const Key, center_of_mass_transform: ?RMat44) void {
        const m = &self.m;
        m.position = key.position;
        m.rotation = key.rotation;
        m.shape_offset = key.shape_offset;
        m.up = key.up;
        m.character_padding = key.character_padding;
        self.setPin(key.shape);
        if (center_of_mass_transform) |transform| {
            if (m.replay.build(key.shape.?, rotationColumns(transform))) {
                m.center_of_mass_transform = transform;
                m.translation = transform.getTranslation();
                m.state = .valid;
                return;
            }
        }
        m.state = .unsupported;
    }

    /// The result of `find`
    pub const Find = enum { valid, stale, unsupported };

    /// Can the loops use the stored values for a character whose current members are `key`: `.valid` when the state is
    /// `.valid` and the key equals the stored one, `.unsupported` when the state is `.unsupported` and the key equals
    /// the stored one, `.stale` otherwise (also for `.empty`, whose stored key is undefined)
    pub inline fn find(self: *const CharacterBoundsCache, key: *const Key) Find {
        // `.valid` is tested first: it is the one result that the loops use (in other than test builds `.stale` and
        // `.unsupported` take the same path, Jolt's code)
        if (self.m.state == .valid)
            return if (self.storedKey().eql(key)) .valid else .stale;
        if (self.m.state == .empty or !self.storedKey().eql(key))
            return .stale;
        return .unsupported;
    }
};
