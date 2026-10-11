//! Port of: Jolt/Physics/Constraints/ContactConstraintManager.h, Jolt/Physics/Constraints/ContactConstraintManager.cpp
//! Status: complete
//! Not ported: JPH_MANIFOLD_CACHE_DEBUG (a commented out debug define that traces the hash map statistics in
//! ManifoldCache::Finalize)
//!
//! SimulationArchitecture D10:
//! - The cache types are extern structs with Jolt v5.6.1's layout and comptime size / alignment checks:
//!   `CachedContactPoint` (28 bytes), `CachedManifold` (60, the flexible array of contact points follows it in the
//!   LFHMAllocator, `getRequiredExtraSize` / `getRequiredTotalSize`), `CachedBodyPair` (28). The key value pairs of
//!   the maps are 80 (`MKeyValue`) and 40 (`BPKeyValue`) bytes like Jolt's, so the handles (offsets in the
//!   LFHMAllocator) are Jolt's.
//! - `CachedManifold::EFlags` (a `mutable atomic<uint16>` changed with `|=`) is `CachedManifold.Flags` on
//!   `flags: std.atomic.Value(u16)`. Jolt's `mFlags |= x` and the implicit loads are sequentially consistent, so they
//!   are `.seq_cst` here.
//! - The ManifoldCache is double buffered (`cache: [2]ManifoldCache`) on LockFreeHashMap. Jolt's `const_cast` of a
//!   Find result and its writes to the `mutable` flags through a `const` manifold are `findManifoldMut` /
//!   `fromHandleMut` on the non-const cache pointers (Rule M).
//! - The contact constraints live in a byte buffer from the TempAllocator (`constraints: TempArray(u8)`, Jolt:
//!   `uint8 *mConstraints`), records of `ContactConstraint(type1, type2)` (an extern struct whose first field `base`
//!   is the `ContactConstraintBase`, followed by the contact point flexible array) at the offsets in
//!   `constraint_idx_to_offset`. The parts are A2's extern structs `ContactConstraintPart(type1, type2)` /
//!   `AngularFrictionConstraintPart(type1, type2)`, so `max_constraint_size` is Jolt's 504 bytes. A constraint is
//!   constructed like Jolt's placement new without initialization: only the members that Jolt writes are written.
//! - Jolt's dispatch tables over the motion types are `inline else` switches; a combination for which Jolt's table
//!   has a nullptr (static vs static, and every pair without a dynamic body in the solver tables) panics, Jolt calls
//!   a null function pointer there. `ContactConstraint<Type1, Type2> &` from `ContactConstraintBase &` (static_cast)
//!   is `@fieldParentPtr("base", base)`.
//! - `SortContacts` and `GetAll*Sorted` use `zolt.quickSort` with Jolt's comparators. The listener calls
//!   (ContactListener, pattern B) are at Jolt's call sites.
//! - The body pair queue slots of D10 are PhysicsUpdateContext.BodyPairSlot (`.monotonic` 8 byte atomics, F2).
//! - Zolt addition `finalizeContactCacheAfterOutOfMemory` (D10 "Out of memory"): a poisoned ContactRemovedCallbacks
//!   job discards the incomplete write cache instead of swapping it in.
//!
//! Signatures:
//! - Jolt's constructor `ContactConstraintManager(const PhysicsSettings &)` and `Init` are one in place `init` that
//!   takes the allocator of the caches (porting guide: constructor + Init). The manager (and so the PhysicsSystem
//!   that embeds it) must not move after `init`: the hash maps point to the LFHMAllocators of the caches.
//! - Pointer + end pairs (`uint32 *ioConstraintOffsetBegin, const uint32 *inConstraintOffsetEnd`) are slices; out
//!   parameters are return values: `getContactsFromCache` returns Jolt's `outPairHandled`, `getAffectedBodies` an
//!   `AffectedBodies` struct, `ContactConstraintBase::GetTangents` a `Tangents` struct.
//! - Overloads: `ManifoldCache::Find(SubShapeIDPair / BodyPair)` -> `findManifold` / `findBodyPair`, `Create` ->
//!   `createManifold` / `createBodyPair`, the non-const `FromHandle` -> `fromHandleMut`, Zolt's `findManifoldMut`
//!   replaces the `const_cast` writes.
//! - The functions that build Jolt's `Array`s (`SaveState`, `RestoreState` when validating, the `GetAll*Sorted`)
//!   return `Allocator.Error`. A RestoreState that runs out of memory leaves an empty write cache (Zolt).
//! - `WarmStartVelocityConstraints<MotionPropertiesCallback>` takes the callback as `anytype` (a pointer to a
//!   CalculateSolverSteps or a DummyCalculateSolverSteps): `callback.call(MotionProperties, mp)`.
//! - The private nested types of Jolt are `pub` (the tests and the parity tests read them); they are internal.
//!
//! JPH_DET_LOG: TODO(determinism_log). JPH_DEBUG_RENDERER (sDrawContactPoint, sDrawSupportingFaces,
//! sDrawContactPointReduction, sDrawContactManifolds, ContactConstraint::Draw): TODO(debug_renderer).
//! JPH_TRACK_SIMULATION_STATS follows MotionProperties' comptime switch `track_simulation_stats` (off). JPH_PROFILE is
//! dropped.

const std = @import("std");
const builtin = @import("builtin");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Float3 = @import("../../Math/Float3.zig").Float3;
const Vector = @import("../../Math/Vector.zig").Vector;
const RealFile = @import("../../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const prefetchL1 = @import("../../Core/Prefetch.zig").prefetchL1;
const LockFreeHashMapFile = @import("../../Core/LockFreeHashMap.zig");
const LFHMAllocator = LockFreeHashMapFile.LFHMAllocator;
const LFHMAllocatorContext = LockFreeHashMapFile.LFHMAllocatorContext;
const LockFreeHashMap = LockFreeHashMapFile.LockFreeHashMap;
const TempAllocatorFile = @import("../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyPair = @import("../Body/BodyPair.zig").BodyPair;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const MotionPropertiesFile = @import("../Body/MotionProperties.zig");
const MotionProperties = MotionPropertiesFile.MotionProperties;
const track_simulation_stats = MotionPropertiesFile.track_simulation_stats;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const SubShapeIDPair = @import("../Collision/Shape/SubShapeIDPair.zig").SubShapeIDPair;
const CollideShapeResult = @import("../Collision/CollideShape.zig").CollideShapeResult;
const ContactListenerFile = @import("../Collision/ContactListener.zig");
const ContactListener = ContactListenerFile.ContactListener;
const ContactManifold = ContactListenerFile.ContactManifold;
const ContactSettings = ContactListenerFile.ContactSettings;
const ValidateResult = ContactListenerFile.ValidateResult;
const ContactConstraintPart = @import("ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart;
const AngularFrictionConstraintPart = @import("ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart;
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;
const PhysicsUpdateError = @import("../EPhysicsUpdateError.zig").PhysicsUpdateError;
const PhysicsUpdateContext = @import("../PhysicsUpdateContext.zig").PhysicsUpdateContext;
const StateRecorderFile = @import("../StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;

/// Jolt's `Vec3(RVec3)`: identity in single precision, DVec3.toVec3 in double precision
fn toVec3(v: RVec3) Vec3 {
    return if (Core.double_precision) v.toVec3() else v;
}

/// The motion type combinations that the dispatch tables of GetContactsFromCache and AddContactConstraint have a
/// function for: everything but static vs static (non-dynamic vs non-dynamic can happen due to one body being a sensor)
fn hasCacheFunction(comptime type1: MotionType, comptime type2: MotionType) bool {
    return !(type1 == .static and type2 == .static);
}

/// The motion type combinations that the dispatch tables of the solver (warm start, solve velocity / position, store
/// impulses) have a function for: a contact constraint always has a dynamic body
fn hasSolverFunction(comptime type1: MotionType, comptime type2: MotionType) bool {
    return type1 == .dynamic or type2 == .dynamic;
}

/// A contact constraint manager manages all contacts between two bodies
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const ContactConstraintManager = struct {
    /// Max 4 contact points are needed for a stable manifold
    pub const max_contact_points = 4;

    /// Callback function to combine the restitution or friction of two bodies
    /// Note that when merging manifolds (when PhysicsSettings::mUseManifoldReduction is true) you will only get a callback for the merged manifold.
    /// It is not possible in that case to get all sub shape ID pairs that were colliding, you'll get the first encountered pair.
    pub const CombineFunction = *const fn (body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32;

    /// Contacts are allocated in a lock free hash map
    pub const ContactAllocator = struct {
        /// Base class LFHMAllocatorContext
        base: LFHMAllocatorContext,
        /// Total number of body pairs added using this allocator
        num_body_pairs: u32 = 0,
        /// Total number of manifolds added using this allocator
        num_manifolds: u32 = 0,
        /// Errors reported on this allocator
        errors: PhysicsUpdateError = .none,

        /// using LFHMAllocatorContext::LFHMAllocatorContext
        pub fn init(lfhm_allocator: *LFHMAllocator, block_size: u32) ContactAllocator {
            return .{ .base = .init(lfhm_allocator, block_size) };
        }
    };

    /// Handle used to keep track of the current body pair
    pub const BodyPairHandle = ?*anyopaque;

    /// The bodies of a contact constraint (GetAffectedBodies' out parameters)
    pub const AffectedBodies = struct {
        body1: *const Body,
        body2: *const Body,
    };

    /// Local space contact point, used for caching impulses
    pub const CachedContactPoint = extern struct {
        /// Local space positions on body 1 and 2.
        /// Note: these values are read through sLoadFloat3Unsafe.
        position1: Float3,
        position2: Float3,

        /// Total applied impulse during the last update that it was used
        non_penetration_lambda: f32,

        /// Saving / restoring state for replay
        pub fn saveState(self: *const CachedContactPoint, stream: *StateRecorder) void {
            stream.write(self.position1);
            stream.write(self.position2);
            stream.write(self.non_penetration_lambda);
        }

        pub fn restoreState(self: *CachedContactPoint, stream: *StateRecorder) void {
            stream.read(&self.position1);
            stream.read(&self.position2);
            stream.read(&self.non_penetration_lambda);
        }

        comptime {
            std.debug.assert(@sizeOf(CachedContactPoint) == 28); // Unexpected size
            std.debug.assert(@alignOf(CachedContactPoint) == 4); // Assuming 4 byte aligned
        }
    };

    /// A single cached manifold
    pub const CachedManifold = extern struct {
        /// Flags for this cached manifold (EFlags, the values of `flags`)
        pub const Flags = struct {
            /// If this cache entry was reused in the next simulation update
            pub const contact_persisted: u16 = 1;
            /// This is a cached manifold reported by continuous collision detection and was only used to create a contact callback
            pub const ccd_contact: u16 = 2;
        };

        /// Handle to next cached contact points in ManifoldCache::mCachedManifolds for the same body pair
        next_with_same_body_pair: u32 = undefined,

        /// Contact normal in the space of 2.
        /// Note: this value is read through sLoadFloat3Unsafe.
        contact_normal: Float3 = undefined,

        /// Total applied impulse during the last update that it was used
        friction_lambda: Vector(2) = undefined,
        angular_friction_lambda: f32 = undefined,

        /// @see Flags (Jolt: `mutable atomic<uint16> mFlags { 0 }`)
        flags: std.atomic.Value(u16) = .init(0),

        /// Number of contact points in the array below
        num_contact_points: u16 = undefined,

        /// Contact points that this manifold consists of (the first of `num_contact_points`, the others follow this
        /// struct, see getContactPoints)
        contact_points: [1]CachedContactPoint = undefined,

        /// Calculate size in bytes needed beyond the size of the class to store inNumContactPoints
        pub fn getRequiredExtraSize(num_contact_points: i32) i32 {
            return @as(i32, @max(0, num_contact_points - 1)) * @as(i32, @sizeOf(CachedContactPoint));
        }

        /// Calculate total class size needed for storing inNumContactPoints
        pub fn getRequiredTotalSize(num_contact_points: i32) i32 {
            return @as(i32, @sizeOf(CachedManifold)) + getRequiredExtraSize(num_contact_points);
        }

        /// The contact points (Jolt: mContactPoints[i] for i < mNumContactPoints)
        pub fn getContactPoints(self: *CachedManifold) [*]CachedContactPoint {
            return &self.contact_points;
        }

        pub fn getContactPointsConst(self: *const CachedManifold) [*]const CachedContactPoint {
            return &self.contact_points;
        }

        /// Saving / restoring state for replay
        pub fn saveState(self: *const CachedManifold, stream: *StateRecorder) void {
            stream.write(self.contact_normal);
            stream.write(self.friction_lambda);
            stream.write(self.angular_friction_lambda);
        }

        pub fn restoreState(self: *CachedManifold, stream: *StateRecorder) void {
            stream.read(&self.contact_normal);
            stream.read(&self.friction_lambda);
            stream.read(&self.angular_friction_lambda);
        }

        comptime {
            std.debug.assert(@sizeOf(CachedManifold) == 60); // This structure is expect to not contain any waste due to alignment
            std.debug.assert(@alignOf(CachedManifold) == 4); // Assuming 4 byte aligned
        }
    };

    /// Define a map that maps SubShapeIDPair -> manifold
    pub const ManifoldMap = LockFreeHashMap(SubShapeIDPair, CachedManifold);
    pub const MKeyValue = ManifoldMap.KeyValue;

    /// std::pair<MKeyValue *, bool> (`first` / `second`)
    pub const MKVAndCreated = struct {
        kv: ?*MKeyValue,
        created: bool,
    };

    /// Start of list of contact points for a particular pair of bodies
    pub const CachedBodyPair = extern struct {
        /// Local space position difference from Body A to Body B.
        /// Note: this value is read through sLoadFloat3Unsafe
        delta_position: Float3 = undefined,

        /// Local space rotation difference from Body A to Body B, fourth component of quaternion is not stored but is guaranteed >= 0.
        /// Note: this value is read through sLoadFloat3Unsafe
        delta_rotation: Float3 = undefined,

        /// Handle to first manifold in ManifoldCache::mCachedManifolds
        first_cached_manifold: u32 = undefined,

        /// Saving / restoring state for replay
        pub fn saveState(self: *const CachedBodyPair, stream: *StateRecorder) void {
            stream.write(self.delta_position);
            stream.write(self.delta_rotation);
        }

        pub fn restoreState(self: *CachedBodyPair, stream: *StateRecorder) void {
            stream.read(&self.delta_position);
            stream.read(&self.delta_rotation);
        }

        comptime {
            std.debug.assert(@sizeOf(CachedBodyPair) == 28); // Unexpected size
            std.debug.assert(@alignOf(CachedBodyPair) == 4); // Assuming 4 byte aligned
        }
    };

    /// Define a map that maps BodyPair -> CachedBodyPair
    pub const BodyPairMap = LockFreeHashMap(BodyPair, CachedBodyPair);
    pub const BPKeyValue = BodyPairMap.KeyValue;

    comptime {
        // The key value pairs have Jolt's sizes, so the handles (offsets in the LFHMAllocator) are Jolt's
        std.debug.assert(@sizeOf(MKeyValue) == 80);
        std.debug.assert(@alignOf(MKeyValue) == 4);
        std.debug.assert(@sizeOf(BPKeyValue) == 40);
        std.debug.assert(@alignOf(BPKeyValue) == 8);
    }

    /// Holds all caches that are needed to quickly find cached body pairs / manifolds
    pub const ManifoldCache = struct {
        /// Block size used when allocating new blocks in the contact cache
        pub const allocator_block_size: u32 = 4096;

        /// Allocator used by both mCachedManifolds and mCachedBodyPairs, this makes it more likely that a body pair and its manifolds are close in memory
        lfhm_allocator: LFHMAllocator = .empty,

        /// Simple hash map for SubShapeIDPair -> CachedManifold (points to lfhm_allocator, valid after init)
        cached_manifolds: ManifoldMap = undefined,

        /// Simple hash map for BodyPair -> CachedBodyPair (points to lfhm_allocator, valid after init)
        cached_body_pairs: BodyPairMap = undefined,

        /// Marks if this buffer is complete (JPH_ENABLE_ASSERTS)
        is_finalized: if (Core.enable_asserts) bool else void = if (Core.enable_asserts) false else {},

        /// Initialize the cache (in place: the maps point to the LFHMAllocator of this cache)
        pub fn init(self: *ManifoldCache, allocator: Allocator, max_body_pairs_in: u32, max_contact_constraints: u32, cached_manifolds_size: u32) Allocator.Error!void {
            std.debug.assert(max_contact_constraints <= max_contact_constraints_limit); // Should have been enforced by caller

            var max_body_pairs = @min(max_body_pairs_in, max_body_pairs_limit);
            if (Core.enable_asserts) std.debug.assert(max_body_pairs == max_body_pairs_in); // Cannot support this many body pairs!
            max_body_pairs = @max(max_body_pairs, 4); // Because our hash map requires at least 4 buckets, we need to have a minimum number of body pairs

            self.lfhm_allocator = try .init(allocator, @intCast(@min(@as(u64, max_body_pairs) * @sizeOf(BPKeyValue) + cached_manifolds_size, @as(u64, std.math.maxInt(u32)))));
            errdefer self.lfhm_allocator.deinit(allocator);

            self.cached_manifolds = try .init(allocator, &self.lfhm_allocator, math.getNextPowerOf2(max_contact_constraints));
            errdefer self.cached_manifolds.deinit(allocator);
            self.cached_body_pairs = try .init(allocator, &self.lfhm_allocator, math.getNextPowerOf2(max_body_pairs));
        }

        /// Destructor (only after a successful init)
        pub fn deinit(self: *ManifoldCache, allocator: Allocator) void {
            self.cached_body_pairs.deinit(allocator);
            self.cached_manifolds.deinit(allocator);
            self.lfhm_allocator.deinit(allocator);
            self.* = .{};
        }

        /// Reset all entries from the cache
        pub fn clear(self: *ManifoldCache) void {
            self.cached_manifolds.clear();
            self.cached_body_pairs.clear();
            self.lfhm_allocator.clear();

            // Mark as incomplete
            if (Core.enable_asserts) self.is_finalized = false;
        }

        /// Prepare cache before creating new contacts.
        /// inExpectedNumBodyPairs / inExpectedNumManifolds are the amount of body pairs / manifolds found in the previous step and is used to determine the amount of buckets the contact cache hash map will use.
        pub fn prepare(self: *ManifoldCache, expected_num_body_pairs: u32, expected_num_manifolds: u32) void {
            // Minimum amount of buckets to use in the hash map
            const min_buckets: u32 = 1024;

            // Use the next higher power of 2 of amount of objects in the cache from last frame to determine the amount of buckets in this frame
            self.cached_manifolds.setNumBuckets(@min(@max(min_buckets, math.getNextPowerOf2(expected_num_manifolds)), self.cached_manifolds.getMaxBuckets()));
            self.cached_body_pairs.setNumBuckets(@min(@max(min_buckets, math.getNextPowerOf2(expected_num_body_pairs)), self.cached_body_pairs.getMaxBuckets()));
        }

        /// Get a new allocator context for storing contacts. Note that you should call this once and then add multiple contacts using the context.
        pub fn getContactAllocator(self: *ManifoldCache) ContactAllocator {
            return .init(&self.lfhm_allocator, allocator_block_size);
        }

        /// Find / create cached entry for SubShapeIDPair -> CachedManifold
        pub fn findManifold(self: *const ManifoldCache, key: SubShapeIDPair, key_hash: u64) ?*const MKeyValue {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);
            return self.cached_manifolds.find(key, key_hash);
        }

        /// Zolt addition: `findManifold` with a mutable result, where Jolt writes the `mutable` flags of a manifold
        /// that it found with the const Find (Rule M)
        pub fn findManifoldMut(self: *ManifoldCache, key: SubShapeIDPair, key_hash: u64) ?*MKeyValue {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);
            return self.cached_manifolds.findMut(key, key_hash);
        }

        pub fn createManifold(self: *ManifoldCache, contact_allocator: *ContactAllocator, key: SubShapeIDPair, key_hash: u64, num_contact_points: i32) ?*MKeyValue {
            if (Core.enable_asserts) std.debug.assert(!self.is_finalized);
            const kv = self.cached_manifolds.create(&contact_allocator.base, key, key_hash, CachedManifold.getRequiredExtraSize(num_contact_points), .{}) orelse {
                contact_allocator.errors = contact_allocator.errors.bitOr(.{ .manifold_cache_full = true });
                return null;
            };
            kv.getValue().num_contact_points = @intCast(num_contact_points);
            contact_allocator.num_manifolds +%= 1;
            return kv;
        }

        pub fn findOrCreate(self: *ManifoldCache, contact_allocator: *ContactAllocator, key: SubShapeIDPair, key_hash: u64, num_contact_points: i32) MKVAndCreated {
            if (self.cached_manifolds.findMut(key, key_hash)) |kv|
                return .{ .kv = kv, .created = false };

            return .{ .kv = self.createManifold(contact_allocator, key, key_hash, num_contact_points), .created = true };
        }

        pub fn toHandle(self: *const ManifoldCache, key_value: *const MKeyValue) u32 {
            if (Core.enable_asserts) std.debug.assert(!self.is_finalized);
            return self.cached_manifolds.toHandle(key_value);
        }

        pub fn fromHandle(self: *const ManifoldCache, handle: u32) *const MKeyValue {
            return self.cached_manifolds.fromHandle(handle);
        }

        /// The non-const FromHandle
        pub fn fromHandleMut(self: *ManifoldCache, handle: u32) *MKeyValue {
            return self.cached_manifolds.fromHandle(handle);
        }

        /// Find / create entry for BodyPair -> CachedBodyPair
        pub fn findBodyPair(self: *const ManifoldCache, key: BodyPair, key_hash: u64) ?*const BPKeyValue {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);
            return self.cached_body_pairs.find(key, key_hash);
        }

        pub fn createBodyPair(self: *ManifoldCache, contact_allocator: *ContactAllocator, key: BodyPair, key_hash: u64) ?*BPKeyValue {
            if (Core.enable_asserts) std.debug.assert(!self.is_finalized);
            const kv = self.cached_body_pairs.create(&contact_allocator.base, key, key_hash, 0, .{}) orelse {
                contact_allocator.errors = contact_allocator.errors.bitOr(.{ .body_pair_cache_full = true });
                return null;
            };
            contact_allocator.num_body_pairs +%= 1;
            return kv;
        }

        pub fn getAllBodyPairsSorted(self: *const ManifoldCache, allocator: Allocator, out_all: *std.ArrayList(*const BPKeyValue)) Allocator.Error!void {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);
            try self.cached_body_pairs.getAllKeyValues(allocator, out_all);

            // Sort by key
            quickSort(*const BPKeyValue, out_all.items, {}, struct {
                fn lessThan(_: void, lhs: *const BPKeyValue, rhs: *const BPKeyValue) bool {
                    return lhs.getKey().lessThan(rhs.getKey().*);
                }
            }.lessThan);
        }

        pub fn getAllManifoldsSorted(self: *const ManifoldCache, body_pair: *const CachedBodyPair, allocator: Allocator, out_all: *std.ArrayList(*const MKeyValue)) Allocator.Error!void {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);

            // Iterate through the attached manifolds
            var handle = body_pair.first_cached_manifold;
            while (handle != ManifoldMap.invalid_handle) : (handle = self.fromHandle(handle).getValueConst().next_with_same_body_pair) {
                const kv = self.cached_manifolds.fromHandle(handle);
                try out_all.append(allocator, kv);
            }

            // Sort by key
            sortManifoldsByKey(out_all.items);
        }

        pub fn getAllCCDManifoldsSorted(self: *const ManifoldCache, allocator: Allocator, out_all: *std.ArrayList(*const MKeyValue)) Allocator.Error!void {
            try self.cached_manifolds.getAllKeyValues(allocator, out_all);

            var i: isize = @as(isize, @intCast(out_all.items.len)) - 1;
            while (i >= 0) : (i -= 1) {
                const index: usize = @intCast(i);
                if ((out_all.items[index].getValueConst().flags.load(.seq_cst) & CachedManifold.Flags.ccd_contact) == 0) {
                    out_all.items[index] = out_all.getLast();
                    _ = out_all.pop();
                }
            }

            // Sort by key
            sortManifoldsByKey(out_all.items);
        }

        /// The comparator of GetAllManifoldsSorted / GetAllCCDManifoldsSorted: sort by key
        fn sortManifoldsByKey(all: []*const MKeyValue) void {
            quickSort(*const MKeyValue, all, {}, struct {
                fn lessThan(_: void, lhs: *const MKeyValue, rhs: *const MKeyValue) bool {
                    return lhs.getKey().lessThan(rhs.getKey());
                }
            }.lessThan);
        }

        pub fn contactPointRemovedCallbacks(self: *ManifoldCache, listener: ContactListener) void {
            var it = self.cached_manifolds.begin();
            while (it.next()) |kv| {
                if ((kv.getValue().flags.load(.seq_cst) & CachedManifold.Flags.contact_persisted) == 0)
                    listener.onContactRemoved(kv.getKey());
            }
        }

        /// Get the amount of manifolds in the cache (JPH_ENABLE_ASSERTS: only declared when Core.enable_asserts)
        pub const getNumManifolds = if (Core.enable_asserts) getNumManifoldsImpl else {};

        fn getNumManifoldsImpl(self: *const ManifoldCache) u32 {
            return self.cached_manifolds.getNumKeyValues();
        }

        /// Get the amount of body pairs in the cache (JPH_ENABLE_ASSERTS: only declared when Core.enable_asserts)
        pub const getNumBodyPairs = if (Core.enable_asserts) getNumBodyPairsImpl else {};

        fn getNumBodyPairsImpl(self: *const ManifoldCache) u32 {
            return self.cached_body_pairs.getNumKeyValues();
        }

        /// Before a cache is finalized you can only do Create(), after only Find() or Clear() (JPH_ENABLE_ASSERTS: only
        /// declared when Core.enable_asserts)
        pub const finalize = if (Core.enable_asserts) finalizeImpl else {};

        fn finalizeImpl(self: *ManifoldCache) void {
            self.is_finalized = true;
        }

        /// Zolt addition (D10): reset the ContactPersisted marks that an update set on this (read) cache. A read
        /// cache has none before an update: its manifolds were created (or copied before they were marked) with
        /// flags 0 or ccd_contact.
        fn resetContactPersisted(self: *ManifoldCache) void {
            var it = self.cached_manifolds.begin();
            while (it.next()) |kv|
                _ = kv.getValue().flags.fetchAnd(~CachedManifold.Flags.contact_persisted, .seq_cst);
        }

        /// Saving / restoring state for replay
        pub fn saveState(self: *const ManifoldCache, allocator: Allocator, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
            if (Core.enable_asserts) std.debug.assert(self.is_finalized);

            // Get contents of cache
            var all_bp: std.ArrayList(*const BPKeyValue) = .empty;
            defer all_bp.deinit(allocator);
            try self.getAllBodyPairsSorted(allocator, &all_bp);

            // Determine which ones to save
            var filtered_bp: std.ArrayList(*const BPKeyValue) = .empty;
            defer filtered_bp.deinit(allocator);
            const selected_bp: []const *const BPKeyValue = if (filter) |f| blk: {
                try filtered_bp.ensureTotalCapacity(allocator, all_bp.items.len);
                for (all_bp.items) |bp_kv| {
                    if (f.shouldSaveContact(bp_kv.getKey().body_a, bp_kv.getKey().body_b))
                        filtered_bp.appendAssumeCapacity(bp_kv);
                }
                break :blk filtered_bp.items;
            } else all_bp.items;

            // Write body pairs
            const num_body_pairs: u32 = @intCast(selected_bp.len);
            stream.write(num_body_pairs);
            for (selected_bp) |bp_kv| {
                // Write body pair key
                stream.write(bp_kv.getKey().*);

                // Write body pair
                const bp = bp_kv.getValueConst();
                bp.saveState(stream);

                // Get attached manifolds
                var all_m: std.ArrayList(*const MKeyValue) = .empty;
                defer all_m.deinit(allocator);
                try self.getAllManifoldsSorted(bp, allocator, &all_m);

                // Write num manifolds
                const num_manifolds: u32 = @intCast(all_m.items.len);
                stream.write(num_manifolds);

                // Write all manifolds
                for (all_m.items) |m_kv| {
                    // Write key
                    stream.write(m_kv.getKey().*);
                    const cm = m_kv.getValueConst();
                    std.debug.assert((cm.flags.load(.seq_cst) & CachedManifold.Flags.ccd_contact) == 0);

                    // Write amount of contacts
                    stream.write(cm.num_contact_points);

                    // Write manifold
                    cm.saveState(stream);

                    // Write contact points
                    for (cm.getContactPointsConst()[0..cm.num_contact_points]) |*cp|
                        cp.saveState(stream);
                }
            }

            // Get CCD manifolds
            var all_m: std.ArrayList(*const MKeyValue) = .empty;
            defer all_m.deinit(allocator);
            try self.getAllCCDManifoldsSorted(allocator, &all_m);

            // Determine which ones to save
            var filtered_m: std.ArrayList(*const MKeyValue) = .empty;
            defer filtered_m.deinit(allocator);
            const selected_m: []const *const MKeyValue = if (filter) |f| blk: {
                try filtered_m.ensureTotalCapacity(allocator, all_m.items.len);
                for (all_m.items) |m_kv| {
                    if (f.shouldSaveContact(m_kv.getKey().getBody1ID(), m_kv.getKey().getBody2ID()))
                        filtered_m.appendAssumeCapacity(m_kv);
                }
                break :blk filtered_m.items;
            } else all_m.items;

            // Write all CCD manifold keys
            const num_manifolds: u32 = @intCast(selected_m.len);
            stream.write(num_manifolds);
            for (selected_m) |m_kv|
                stream.write(m_kv.getKey().*);
        }

        /// (Zolt: the counts that Jolt leaves uninitialized before reading them from the stream start at 0, and the
        /// temporaries of a skipped contact are zeroed, so a stream that ends early or a validating stream never reads
        /// undefined memory. The validating path allocates the sorted arrays: Allocator.Error.)
        pub fn restoreState(self: *ManifoldCache, allocator: Allocator, read_cache: *const ManifoldCache, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!bool {
            if (Core.enable_asserts) std.debug.assert(!self.is_finalized);

            var success = true;

            // Create a contact allocator for restoring the contact cache
            var contact_allocator = self.getContactAllocator();

            // When validating, get all existing body pairs
            var all_bp: std.ArrayList(*const BPKeyValue) = .empty;
            defer all_bp.deinit(allocator);
            if (stream.isValidating())
                try read_cache.getAllBodyPairsSorted(allocator, &all_bp);

            // Read amount of body pairs
            var num_body_pairs: u32 = 0;
            if (stream.isValidating())
                num_body_pairs = @intCast(all_bp.items.len);
            stream.read(&num_body_pairs);

            // Read entire cache
            var i: u32 = 0;
            while (i < num_body_pairs) : (i += 1) {
                // Read key
                var body_pair_key: BodyPair = .{};
                if (stream.isValidating() and i < all_bp.items.len)
                    body_pair_key = all_bp.items[i].getKey().*;
                stream.read(&body_pair_key);

                // Check if we want to restore this contact
                if (filter == null or filter.?.shouldRestoreContact(body_pair_key.body_a, body_pair_key.body_b)) {
                    // Create new entry for this body pair
                    const body_pair_hash = body_pair_key.getHash();
                    const bp_kv = self.createBodyPair(&contact_allocator, body_pair_key, body_pair_hash) orelse {
                        // Out of cache space
                        success = false;
                        break;
                    };
                    const bp = bp_kv.getValue();

                    // Read body pair
                    if (stream.isValidating() and i < all_bp.items.len)
                        bp.* = all_bp.items[i].getValueConst().*;
                    bp.restoreState(stream);

                    // When validating, get all existing manifolds
                    var all_m: std.ArrayList(*const MKeyValue) = .empty;
                    defer all_m.deinit(allocator);
                    if (stream.isValidating() and i < all_bp.items.len)
                        try read_cache.getAllManifoldsSorted(all_bp.items[i].getValueConst(), allocator, &all_m);

                    // Read amount of manifolds
                    var num_manifolds: u32 = 0;
                    if (stream.isValidating())
                        num_manifolds = @intCast(all_m.items.len);
                    stream.read(&num_manifolds);

                    var handle: u32 = ManifoldMap.invalid_handle;
                    var j: u32 = 0;
                    while (j < num_manifolds) : (j += 1) {
                        // Read key
                        var sub_shape_key: SubShapeIDPair = .{};
                        if (stream.isValidating() and j < all_m.items.len)
                            sub_shape_key = all_m.items[j].getKey().*;
                        stream.read(&sub_shape_key);
                        const sub_shape_key_hash = sub_shape_key.getHash();

                        // Read amount of contact points
                        var num_contact_points: u16 = 0;
                        if (stream.isValidating() and j < all_m.items.len)
                            num_contact_points = all_m.items[j].getValueConst().num_contact_points;
                        stream.read(&num_contact_points);

                        // Read manifold
                        const m_kv = self.createManifold(&contact_allocator, sub_shape_key, sub_shape_key_hash, num_contact_points) orelse {
                            // Out of cache space
                            success = false;
                            break;
                        };
                        const cm = m_kv.getValue();
                        if (stream.isValidating() and j < all_m.items.len) {
                            const size: usize = @intCast(CachedManifold.getRequiredTotalSize(num_contact_points));
                            @memcpy(@as([*]u8, @ptrCast(cm))[0..size], @as([*]const u8, @ptrCast(all_m.items[j].getValueConst()))[0..size]);
                            cm.num_contact_points = num_contact_points; // Restore num contact points
                        }
                        cm.restoreState(stream);
                        cm.next_with_same_body_pair = handle;
                        handle = self.toHandle(m_kv);

                        // Read contact points
                        for (cm.getContactPoints()[0..num_contact_points]) |*cp|
                            cp.restoreState(stream);
                    }
                    bp.first_cached_manifold = handle;
                } else {
                    // Skip the contact
                    var bp = std.mem.zeroes(CachedBodyPair);
                    bp.restoreState(stream);
                    var num_manifolds: u32 = 0;
                    stream.read(&num_manifolds);
                    var j: u32 = 0;
                    while (j < num_manifolds) : (j += 1) {
                        var sub_shape_key: SubShapeIDPair = .{};
                        stream.read(&sub_shape_key);
                        var num_contact_points: u16 = 0;
                        stream.read(&num_contact_points);
                        var cm = std.mem.zeroes(CachedManifold);
                        cm.restoreState(stream);
                        var k: u32 = 0;
                        while (k < num_contact_points) : (k += 1)
                            cm.contact_points[0].restoreState(stream);
                    }
                }
            }

            // When validating, get all existing CCD manifolds
            var all_m: std.ArrayList(*const MKeyValue) = .empty;
            defer all_m.deinit(allocator);
            if (stream.isValidating())
                try read_cache.getAllCCDManifoldsSorted(allocator, &all_m);

            // Read amount of CCD manifolds
            var num_manifolds: u32 = 0;
            if (stream.isValidating())
                num_manifolds = @intCast(all_m.items.len);
            stream.read(&num_manifolds);

            var j: u32 = 0;
            while (j < num_manifolds) : (j += 1) {
                // Read key
                var sub_shape_key: SubShapeIDPair = .{};
                if (stream.isValidating() and j < all_m.items.len)
                    sub_shape_key = all_m.items[j].getKey().*;
                stream.read(&sub_shape_key);

                // Check if we want to restore this contact
                if (filter == null or filter.?.shouldRestoreContact(sub_shape_key.getBody1ID(), sub_shape_key.getBody2ID())) {
                    // Create CCD manifold
                    const sub_shape_key_hash = sub_shape_key.getHash();
                    const m_kv = self.createManifold(&contact_allocator, sub_shape_key, sub_shape_key_hash, 0) orelse {
                        // Out of cache space
                        success = false;
                        break;
                    };
                    const cm = m_kv.getValue();
                    _ = cm.flags.fetchOr(CachedManifold.Flags.ccd_contact, .seq_cst);
                }
            }

            // We don't finalize until the last part is restored
            if (Core.enable_asserts) {
                if (stream.isLastPart())
                    self.is_finalized = true;
            }

            return success;
        }
    };

    /// World space contact point, used for solving penetrations
    pub fn WorldContactPoint(comptime type1: MotionType, comptime type2: MotionType) type {
        return extern struct {
            const Self = @This();

            pub const ConstraintPart = ContactConstraintPart(type1, type2);

            /// The constraint parts
            non_penetration_constraint: ConstraintPart,
            // Note that this needs to be followed by data of size float, see comment at ContactConstraintPart

            /// Distance between the friction center and this contact point
            distance_to_friction_center: f32,

            /// Calculate constraint properties for the non penetration constraint
            pub fn calculateNonPenetrationConstraintProperties(self: *Self, delta_time: f32, gravity: Vec3, body1: *const Body, body2: *const Body, inv_m1: f32, inv_m2: f32, inv_i1: Mat44, inv_i2: Mat44, world_space_position1: RVec3, world_space_position2: RVec3, world_space_normal: Vec3, settings: *const ContactSettings, min_velocity_for_restitution: f32) void {
                // TODO(determinism_log): JPH_DET_LOG("CalculateNonPenetrationConstraintProperties: p1: " << inWorldSpacePosition1 << " p2: " << inWorldSpacePosition2
                //   << " normal: " << inWorldSpaceNormal << " restitution: " << inSettings.mCombinedRestitution << " minv: " << inMinVelocityForRestitution);

                // Calculate collision points relative to body
                const p = world_space_position1.add(world_space_position2).mulScalar(0.5);
                const r1 = toVec3(p.sub(body1.getCenterOfMassPosition()));
                const r2 = toVec3(p.sub(body2.getCenterOfMassPosition()));

                const mp1 = body1.getMotionPropertiesUnchecked();
                const mp2 = body2.getMotionPropertiesUnchecked();

                // Calculate velocity of collision points
                var relative_velocity: Vec3 = undefined;
                if (comptime type1 != .static and type2 != .static) {
                    relative_velocity = mp2.?.getPointVelocityCOM(r2).sub(mp1.?.getPointVelocityCOM(r1));
                } else if (comptime type1 != .static) {
                    relative_velocity = mp1.?.getPointVelocityCOM(r1).negate();
                } else if (comptime type2 != .static) {
                    relative_velocity = mp2.?.getPointVelocityCOM(r2);
                } else {
                    if (Core.enable_asserts) std.debug.assert(false); // Static vs static makes no sense
                    relative_velocity = Vec3.zero();
                }
                const normal_velocity = relative_velocity.dot(world_space_normal);

                // How much the shapes are penetrating (> 0 if penetrating, < 0 if separated)
                const penetration = toVec3(world_space_position1.sub(world_space_position2)).dot(world_space_normal);

                // If there is no penetration, this is a speculative contact and we will apply a bias to the contact constraint
                // so that the constraint becomes relative_velocity . contact normal > -penetration / delta_time
                // instead of relative_velocity . contact normal > 0
                // See: GDC 2013: "Physics for Game Programmers; Continuous Collision" - Erin Catto
                const speculative_contact_velocity_bias = math.max(@as(f32, 0.0), -penetration / delta_time);

                // Determine if the velocity is big enough for restitution
                var normal_velocity_bias: f32 = undefined;
                if (settings.combined_restitution > 0.0 and normal_velocity < -min_velocity_for_restitution) {
                    // We have a velocity that is big enough for restitution. This is where speculative contacts don't work
                    // great as we have to decide now if we're going to apply the restitution or not. If the relative
                    // velocity is big enough for a hit, we apply the restitution (in the end, due to other constraints,
                    // the objects may actually not collide and we will have applied restitution incorrectly). Another
                    // artifact that occurs because of this approximation is that the object will bounce from its current
                    // position rather than from a position where it is touching the other object. This causes the object
                    // to appear to move faster for 1 frame (the opposite of time stealing).
                    if (normal_velocity < -speculative_contact_velocity_bias) {
                        // The gravity / constant forces are applied in the beginning of the time step.
                        // If we get here, there was a collision at the beginning of the time step, so we've applied too much force.
                        // This means that our calculated restitution can be too high resulting in an increase in energy.
                        // So, when we apply restitution, we cancel the added velocity due to these forces.
                        var relative_acceleration: Vec3 = undefined;

                        // Calculate effect of gravity
                        if (comptime type1 != .static and type2 != .static) {
                            relative_acceleration = gravity.mulScalar(mp2.?.getGravityFactor() - mp1.?.getGravityFactor());
                        } else if (comptime type1 != .static) {
                            relative_acceleration = gravity.negate().mulScalar(mp1.?.getGravityFactor());
                        } else if (comptime type2 != .static) {
                            relative_acceleration = gravity.mulScalar(mp2.?.getGravityFactor());
                        } else {
                            if (Core.enable_asserts) std.debug.assert(false); // Static vs static makes no sense
                            relative_acceleration = Vec3.zero();
                        }

                        // Calculate effect of accumulated forces
                        if (comptime type1 == .dynamic)
                            relative_acceleration = relative_acceleration.sub(mp1.?.getAccumulatedForce().mulScalar(mp1.?.getInverseMass()));
                        if (comptime type2 == .dynamic)
                            relative_acceleration = relative_acceleration.add(mp2.?.getAccumulatedForce().mulScalar(mp2.?.getInverseMass()));

                        // We only compensate forces towards the contact normal.
                        const force_delta_velocity = math.min(@as(f32, 0.0), relative_acceleration.dot(world_space_normal) * delta_time);

                        normal_velocity_bias = settings.combined_restitution * (normal_velocity - force_delta_velocity);
                    } else {
                        // In this case we have predicted that we don't hit the other object, but if we do (due to other constraints changing velocities)
                        // the speculative contact will prevent penetration but will not apply restitution leading to another artifact.
                        normal_velocity_bias = speculative_contact_velocity_bias;
                    }
                } else {
                    // No restitution. We can safely apply our contact velocity bias.
                    normal_velocity_bias = speculative_contact_velocity_bias;
                }

                self.non_penetration_constraint.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, world_space_normal, .{ .bias = normal_velocity_bias });
            }
        };
    }

    /// The two tangents of a contact constraint (GetTangents' out parameters)
    pub const Tangents = struct {
        tangent1: Vec3,
        tangent2: Vec3,
    };

    pub const ContactConstraintBase = extern struct {
        body1: *Body,
        body2: *Body,
        sort_key: u64,
        world_space_normal: Float3,
        world_space_tangent1: Float3,
        world_space_tangent2: Float3,
        combined_friction: f32,
        inv_mass1: f32,
        inv_inertia_scale1: f32,
        inv_mass2: f32,
        inv_inertia_scale2: f32,
        cached_manifold_handle: u32,
        num_contact_points: u32,

        /// Convert the world space normal to a Vec3
        pub fn getWorldSpaceNormal(self: *const ContactConstraintBase) Vec3 {
            return Vec3.loadFloat3Unsafe(&self.world_space_normal);
        }

        /// Get the tangents for this contact constraint
        pub fn getTangents(self: *const ContactConstraintBase) Tangents {
            return .{
                .tangent1 = Vec3.loadFloat3Unsafe(&self.world_space_tangent1),
                .tangent2 = Vec3.loadFloat3Unsafe(&self.world_space_tangent2),
            };
        }

        comptime {
            if (@sizeOf(usize) == 8)
                std.debug.assert(@sizeOf(ContactConstraintBase) == 88);
        }
    };

    /// Contact constraint class, used for solving penetrations
    pub fn ContactConstraint(comptime type1: MotionType, comptime type2: MotionType) type {
        return extern struct {
            const Self = @This();

            /// The motion types this constraint was specialized for (the template arguments)
            pub const motion_type1 = type1;
            pub const motion_type2 = type2;

            pub const ContactPoint = WorldContactPoint(type1, type2);

            /// Base class ContactConstraintBase
            base: ContactConstraintBase,

            friction_constraint1: ContactConstraintPart(type1, type2),
            friction_constraint2: ContactConstraintPart(type1, type2),
            angular_friction_constraint: AngularFrictionConstraintPart(type1, type2),

            /// The first of `base.num_contact_points` contact points, the others follow this struct (see getContactPoints)
            contact_points: [1]ContactPoint,

            /// The contact points (Jolt: mContactPoints[i] for i < mNumContactPoints)
            pub fn getContactPoints(self: *Self) [*]ContactPoint {
                return &self.contact_points;
            }

            pub fn getContactPointsConst(self: *const Self) [*]const ContactPoint {
                return &self.contact_points;
            }

            /// static_cast<ContactConstraint<Type1, Type2> &>(ContactConstraintBase &)
            pub fn fromBase(base: *ContactConstraintBase) *Self {
                return @fieldParentPtr("base", base);
            }

            /// Calculate constraint properties for the friction constraint
            pub fn calculateFrictionConstraintProperties(self: *Self, body1: *const Body, body2: *const Body, inv_m1: f32, inv_m2: f32, inv_i1: Mat44, inv_i2: Mat44, world_space_contacts: [*]const RVec3, world_space_normal: Vec3, world_space_tangent1: Vec3, world_space_tangent2: Vec3, settings: *const ContactSettings) void {
                // Calculate friction part
                if (settings.combined_friction > 0.0) {
                    // Calculate point where the friction applies by averaging the contact points
                    var friction_point = RVec3.zero();
                    for (world_space_contacts[0..self.base.num_contact_points]) |c|
                        friction_point = friction_point.add(c);
                    friction_point = friction_point.divScalar(@as(Real, @floatFromInt(self.base.num_contact_points)));

                    // TODO(determinism_log): JPH_DET_LOG("CalculateFrictionConstraintProperties: point: " << friction_point
                    //   << " friction: " << inSettings.mCombinedFriction
                    //   << " surface_vel: " << inSettings.mRelativeLinearSurfaceVelocity << " surface_ang: " << inSettings.mRelativeAngularSurfaceVelocity);

                    // Calculate distance of contact points to friction center in the normal plane
                    for (self.getContactPoints()[0..self.base.num_contact_points], world_space_contacts[0..self.base.num_contact_points]) |*wcp, c| {
                        const delta = toVec3(c.sub(friction_point));
                        wcp.distance_to_friction_center = delta.sub(world_space_normal.mulScalar(delta.dot(world_space_normal))).length();
                    }

                    // Calculate relative friction points
                    const r1 = toVec3(friction_point.sub(body1.getCenterOfMassPosition()));
                    const r2 = toVec3(friction_point.sub(body2.getCenterOfMassPosition()));

                    // Get surface velocity relative to tangents
                    const ws_surface_velocity = settings.relative_linear_surface_velocity.add(settings.relative_angular_surface_velocity.cross(r1));
                    const surface_velocity1 = world_space_tangent1.dot(ws_surface_velocity);
                    const surface_velocity2 = world_space_tangent2.dot(ws_surface_velocity);

                    // Implement friction as 2 ContactConstraintParts
                    self.friction_constraint1.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, world_space_tangent1, .{ .bias = surface_velocity1 });
                    self.friction_constraint2.calculateConstraintProperties(inv_m1, inv_i1, r1, inv_m2, inv_i2, r2, world_space_tangent2, .{ .bias = surface_velocity2 });

                    // Only apply angular friction if we have more than 1 contact point
                    if (self.base.num_contact_points > 1)
                        self.angular_friction_constraint.calculateConstraintProperties(inv_i1, inv_i2, world_space_normal, .{ .bias = settings.relative_angular_surface_velocity.dot(world_space_normal) })
                    else
                        self.angular_friction_constraint.deactivate();
                } else {
                    // Turn off friction constraint
                    self.friction_constraint1.deactivate();
                    self.friction_constraint2.deactivate();
                    self.angular_friction_constraint.deactivate();
                }
            }

            // TODO(debug_renderer): Draw(DebugRenderer *, const ManifoldCache &, ColorArg): draw the state of the
            // contact constraint (the cached contact points, the normal and the tangents)
        };
    }

    /// Maximum size a single constraint can take
    pub const max_constraint_size: u32 = @sizeOf(ContactConstraint(.dynamic, .dynamic)) + (max_contact_points - 1) * @sizeOf(WorldContactPoint(.dynamic, .dynamic));

    /// The maximum value that can be passed to Init for inMaxContactConstraints. Note you should really use a lower value, using this value will cost a lot of memory!
    pub const max_contact_constraints_limit: u32 = std.math.maxInt(u32) / max_constraint_size;

    /// The maximum value that can be passed to Init for inMaxBodyPairs. Note you should really use a lower value, using this value will cost a lot of memory!
    pub const max_body_pairs_limit: u32 = std.math.maxInt(u32) / @sizeOf(BPKeyValue);

    comptime {
        // Jolt v5.6.1: 312 bytes for ContactConstraint<Dynamic, Dynamic> + 3 * 64 bytes (both precisions, 64 bit)
        if (@sizeOf(usize) == 8)
            std.debug.assert(max_constraint_size == 504);
    }

    /// The main physics settings instance (Jolt: const PhysicsSettings &)
    physics_settings: *const PhysicsSettings = undefined,

    /// Allocates the caches and the arrays of SaveState / RestoreState (Zolt)
    allocator: Allocator = undefined,

    /// Listener that is notified whenever a contact point between two bodies is added/updated/removed
    contact_listener: ?ContactListener = null,

    /// Functions that are used to combine friction and restitution of 2 bodies
    combine_friction: CombineFunction = defaultCombineFriction,
    combine_restitution: CombineFunction = defaultCombineRestitution,

    /// The constraints that were added this frame (cast to ContactConstraint<MotionType1, MotionType2> depending on the
    /// motion types of the bodies in the constraint)
    constraints: TempArray(u8) = .empty,

    /// Maps from constraint index to offset in mConstraints
    constraint_idx_to_offset: TempArray(u32) = .empty,

    max_constraints: u32 = 0,

    /// Lower 32 bits: number of constraints, upper 32 bits next constraint offset
    num_constraints_and_next_constraint_offset: std.atomic.Value(u64) = .init(0),

    /// Context used for this physics update
    update_context: ?*PhysicsUpdateContext = null,

    /// We have one cache to read from and one to write to
    cache: [2]ManifoldCache = .{ .{}, .{} },

    /// Which cache we're currently writing to
    cache_write_idx: u1 = 0,

    /// The cache we're currently reading from (fixed at the start of the step so that it remains even after we swap mCacheWriteIdx in FinalizeContactCacheAndCallContactPointRemovedCallbacks, valid only during stepping)
    read_cache: ?*ManifoldCache = null,

    /// The cache we're currently writing to (fixed at the start of the step so that it remains even after we swap mCacheWriteIdx in FinalizeContactCacheAndCallContactPointRemovedCallbacks, valid only during stepping)
    write_cache: ?*ManifoldCache = null,

    /// Not initialized (Jolt's constructor without the settings), call init
    pub const empty: ContactConstraintManager = .{};

    /// mCombineFriction's default: the geometric mean, sqrt(friction1 * friction2)
    fn defaultCombineFriction(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        _ = sub_shape_id1;
        _ = sub_shape_id2;
        return math.sqrt(body1.getFriction() * body2.getFriction());
    }

    /// mCombineRestitution's default: max(restitution1, restitution2)
    fn defaultCombineRestitution(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        _ = sub_shape_id1;
        _ = sub_shape_id2;
        return math.max(body1.getRestitution(), body2.getRestitution());
    }

    /// Constructor + Initialize the system (in place, the manager must not move afterwards).
    /// @param max_body_pairs Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching
    /// @param max_contact_constraints Maximum amount of contact constraints to process (anything else will fall through the world)
    /// (Zolt: on error nothing is left allocated and the manager is `empty`)
    pub fn init(self: *ContactConstraintManager, allocator: Allocator, physics_settings: *const PhysicsSettings, max_body_pairs: u32, max_contact_constraints: u32) Allocator.Error!void {
        self.* = .{ .physics_settings = physics_settings, .allocator = allocator };
        errdefer self.* = .empty;

        // Constructor: for the first frame mark this empty buffer as finalized
        if (Core.enable_asserts) self.cache[self.cache_write_idx ^ 1].finalize();

        // Limit the number of constraints so that the allocation size fits in an unsigned integer
        self.max_constraints = @min(max_contact_constraints, max_contact_constraints_limit);
        if (Core.enable_asserts) std.debug.assert(self.max_constraints == max_contact_constraints); // Cannot support this many contact constraints!
        self.max_constraints = @max(self.max_constraints, 4); // Because our hash map requires at least 4 buckets, we need to have a minimum number of constraints

        // Calculate worst case cache usage
        const max_manifold_size_per_constraint: u32 = comptime @sizeOf(MKeyValue) + @as(u32, @intCast(CachedManifold.getRequiredExtraSize(max_contact_points)));
        comptime std.debug.assert(max_manifold_size_per_constraint < max_constraint_size); // If not true, then the next line can overflow
        const cached_manifolds_size = self.max_constraints * max_manifold_size_per_constraint;

        // Init the caches
        try self.cache[0].init(allocator, max_body_pairs, self.max_constraints, cached_manifolds_size);
        errdefer self.cache[0].deinit(allocator);
        try self.cache[1].init(allocator, max_body_pairs, self.max_constraints, cached_manifolds_size);
    }

    /// Destructor
    pub fn deinit(self: *ContactConstraintManager) void {
        std.debug.assert(self.constraints.isEmpty());
        std.debug.assert(self.constraint_idx_to_offset.isEmpty());

        // Zolt: `init` sets max_constraints to at least 4, an `empty` manager owns nothing
        if (self.max_constraints != 0) {
            self.cache[1].deinit(self.allocator);
            self.cache[0].deinit(self.allocator);
        }
        self.* = .empty;
    }

    /// Listener that is notified whenever a contact point between two bodies is added/updated/removed
    pub fn setContactListener(self: *ContactConstraintManager, listener: ?ContactListener) void {
        self.contact_listener = listener;
    }

    pub fn getContactListener(self: *const ContactConstraintManager) ?ContactListener {
        return self.contact_listener;
    }

    /// Set the function that combines the friction of two bodies and returns it
    /// Default method is the geometric mean: sqrt(friction1 * friction2).
    pub fn setCombineFriction(self: *ContactConstraintManager, combine_friction: CombineFunction) void {
        self.combine_friction = combine_friction;
    }

    pub fn getCombineFriction(self: *const ContactConstraintManager) CombineFunction {
        return self.combine_friction;
    }

    /// Set the function that combines the restitution of two bodies and returns it
    /// Default method is max(restitution1, restitution1)
    pub fn setCombineRestitution(self: *ContactConstraintManager, combine_restitution: CombineFunction) void {
        self.combine_restitution = combine_restitution;
    }

    pub fn getCombineRestitution(self: *const ContactConstraintManager) CombineFunction {
        return self.combine_restitution;
    }

    /// Get the max number of contact constraints that are allowed
    pub fn getMaxConstraints(self: *const ContactConstraintManager) u32 {
        return self.max_constraints;
    }

    /// Check with the listener if inBody1 and inBody2 could collide, returns false if not
    pub fn validateContactPoint(self: *const ContactConstraintManager, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        const listener = self.contact_listener orelse return .accept_all_contacts_for_this_body_pair;

        return listener.onContactValidate(body1, body2, base_offset, collision_result);
    }

    /// Sets up the constraint buffer. Should be called before starting collision detection.
    /// (Zolt: TempAllocator.Error; the buffers that were allocated are freed by finishConstraintBuffer)
    pub fn prepareConstraintBuffer(self: *ContactConstraintManager, context: *PhysicsUpdateContext) TempAllocator.Error!void {
        // Store context
        self.update_context = context;

        // Store read / write cache
        self.read_cache = &self.cache[self.cache_write_idx ^ 1];
        self.write_cache = &self.cache[self.cache_write_idx];

        // Allocate temporary constraint buffer
        std.debug.assert(self.constraints.isEmpty());
        self.constraints = try .alloc(context.temp_allocator, @as(usize, self.max_constraints) * max_constraint_size);
        std.debug.assert(self.constraint_idx_to_offset.isEmpty());
        self.constraint_idx_to_offset = try .alloc(context.temp_allocator, self.max_constraints);
    }

    /// Get a new allocator context for storing contacts. Note that you should call this once and then add multiple contacts using the context.
    pub fn getContactAllocator(self: *ContactConstraintManager) ContactAllocator {
        return self.cache[self.cache_write_idx].getContactAllocator();
    }

    /// The constraint at an offset in the constraint buffer
    fn getConstraintBase(self: *const ContactConstraintManager, constraint_offset: u32) *ContactConstraintBase {
        return @ptrCast(@alignCast(self.constraints.items.ptr + constraint_offset));
    }

    /// Create a new contact constraint
    fn createConstraint(self: *ContactConstraintManager, comptime type1: MotionType, comptime type2: MotionType, activate_and_link_bodies: *bool, body1: *Body, body2: *Body, sort_key: u64, cached_manifold_handle: u32, world_space_normal: Vec3, settings: *const ContactSettings, num_contact_points: u32) ?*ContactConstraint(type1, type2) {
        const Constraint = ContactConstraint(type1, type2);

        // Calculate the size of this constraint
        const constraint_size: u32 = @truncate(math.alignUp(@as(usize, @sizeOf(Constraint)) + @as(usize, num_contact_points -% 1) * @sizeOf(Constraint.ContactPoint), @alignOf(Constraint)));
        std.debug.assert(constraint_size <= max_constraint_size);

        // Reserve space for constraint
        const constraint_idx_and_constraint_offset = self.num_constraints_and_next_constraint_offset.fetchAdd((@as(u64, constraint_size) << 32) + 1, .monotonic);
        const constraint_idx: u32 = @truncate(constraint_idx_and_constraint_offset);
        if (constraint_idx >= self.max_constraints)
            return null;

        const body1_dynamic = body1.isDynamic();
        const body2_dynamic = body2.isDynamic();

        const context = self.update_context.?;
        if (activate_and_link_bodies.*) {
            // Do this only once
            activate_and_link_bodies.* = false;

            // Wake up sleeping bodies
            var body_ids: [2]BodyID = undefined;
            var num_bodies: usize = 0;
            if (body1_dynamic and !body1.isActive()) {
                body_ids[num_bodies] = body1.getID();
                num_bodies += 1;
            }
            if (body2_dynamic and !body2.isActive()) {
                body_ids[num_bodies] = body2.getID();
                num_bodies += 1;
            }
            if (num_bodies > 0)
                context.body_manager.activateBodies(body_ids[0..num_bodies]);

            // Link the two bodies only if both are dynamic. If one of them is static or kinematic they don't need to go into
            // the same simulation island as a constraint cannot affect the velocity of a kinematic body.
            if (body1_dynamic and body2_dynamic)
                context.island_builder.linkBodies(body1.getIndexInActiveBodiesInternal(), body2.getIndexInActiveBodiesInternal());
        }

        // Link the contact to the first dynamic body
        if (body1_dynamic) {
            context.island_builder.linkContact(constraint_idx, body1.getIndexInActiveBodiesInternal());
        } else {
            std.debug.assert(body2_dynamic);
            context.island_builder.linkContact(constraint_idx, body2.getIndexInActiveBodiesInternal());
        }

        // Store offset for constraint
        const constraint_offset: u32 = @truncate(constraint_idx_and_constraint_offset >> 32);
        std.debug.assert(@as(u64, constraint_offset) + constraint_size <= @as(u64, self.max_constraints) * max_constraint_size);
        self.constraint_idx_to_offset.items[constraint_idx] = constraint_offset;

        // Construct constraint (Jolt: placement new of a class without constructor, only the members below are written)
        const constraint = Constraint.fromBase(self.getConstraintBase(constraint_offset));
        std.debug.assert(math.isAligned(constraint, @alignOf(Constraint)));
        constraint.base.body1 = body1;
        constraint.base.body2 = body2;
        constraint.base.sort_key = sort_key;
        world_space_normal.storeFloat3(&constraint.base.world_space_normal);
        const tangent1 = world_space_normal.getNormalizedPerpendicular();
        tangent1.storeFloat3(&constraint.base.world_space_tangent1);
        world_space_normal.cross(tangent1).storeFloat3(&constraint.base.world_space_tangent2);
        constraint.base.combined_friction = settings.combined_friction;
        constraint.base.inv_inertia_scale1 = settings.inv_inertia_scale1;
        constraint.base.inv_inertia_scale2 = settings.inv_inertia_scale2;
        constraint.base.cached_manifold_handle = cached_manifold_handle;
        constraint.base.num_contact_points = num_contact_points;

        // Track new contact constraints (JPH_TRACK_SIMULATION_STATS)
        if (track_simulation_stats) {
            if (comptime type1 != .static)
                _ = body1.getMotionPropertiesUncheckedMut().?.getSimulationStatsMut().num_contact_constraints.fetchAdd(1, .monotonic);
            if (comptime type2 != .static)
                _ = body2.getMotionPropertiesUncheckedMut().?.getSimulationStatsMut().num_contact_constraints.fetchAdd(1, .monotonic);
        }

        return constraint;
    }

    /// Internal helper function to add a contact constraint from the cache. Templated to the motion type to reduce the amount of branches and calculations.
    fn templatedGetContactsFromCache(self: *ContactConstraintManager, comptime type1: MotionType, comptime type2: MotionType, contact_allocator: *ContactAllocator, body1: *Body, body2: *Body, input_cached_body_pair: *const CachedBodyPair, output_cached_body_pair: *CachedBodyPair) void {
        // Get body transforms
        const transform_body1 = body1.getCenterOfMassTransform();
        const transform_body2 = body2.getCenterOfMassTransform();

        // Get time step and gravity
        const context = self.update_context.?;
        const delta_time = context.step_delta_time;
        const gravity = context.physics_system.getGravity();

        const read_cache = self.read_cache.?;
        const write_cache = self.write_cache.?;

        // Copy manifolds
        var output_handle: u32 = ManifoldMap.invalid_handle;
        var input_handle = input_cached_body_pair.first_cached_manifold;
        var link_bodies = true;
        while (true) {
            // Find the existing manifold (Zolt: the mutable FromHandle, Jolt sets the `mutable` flags through the const result)
            const input_kv = read_cache.fromHandleMut(input_handle);
            const input_key = input_kv.getKey().*;
            const input_cm = input_kv.getValue();
            std.debug.assert(input_cm.num_contact_points > 0); // There should be contact points in this manifold!

            // Create room for manifold in write buffer and copy data
            const input_hash = input_key.getHash();
            const output_kv = write_cache.createManifold(contact_allocator, input_key, input_hash, input_cm.num_contact_points) orelse break; // Out of cache space
            const output_cm = output_kv.getValue();
            const total_size: usize = @intCast(CachedManifold.getRequiredTotalSize(input_cm.num_contact_points));
            @memcpy(@as([*]u8, @ptrCast(output_cm))[0..total_size], @as([*]const u8, @ptrCast(input_cm))[0..total_size]);

            // Link the object under the body pairs
            output_cm.next_with_same_body_pair = output_handle;
            output_handle = write_cache.toHandle(output_kv);

            // Calculate default contact settings
            var settings: ContactSettings = .{};
            settings.combined_friction = self.combine_friction(body1, input_key.getSubShapeID1(), body2, input_key.getSubShapeID2());
            settings.combined_restitution = self.combine_restitution(body1, input_key.getSubShapeID1(), body2, input_key.getSubShapeID2());
            settings.is_sensor = body1.isSensor() or body2.isSensor();

            // Calculate world space contact normal
            const world_space_normal = transform_body2.multiply3x3(Vec3.loadFloat3Unsafe(&output_cm.contact_normal)).normalized();

            // Call contact listener to update settings
            if (self.contact_listener) |listener| {
                // Convert constraint to manifold structure for callback
                var manifold: ContactManifold = .{};
                manifold.world_space_normal = world_space_normal;
                manifold.sub_shape_id1 = input_key.getSubShapeID1();
                manifold.sub_shape_id2 = input_key.getSubShapeID2();
                manifold.base_offset = transform_body1.getTranslation();
                manifold.relative_contact_points_on1.resize(output_cm.num_contact_points);
                manifold.relative_contact_points_on2.resize(output_cm.num_contact_points);
                const local_transform_body2 = transform_body2.postTranslatedRVec3(manifold.base_offset.negate()).toMat44();
                var penetration_depth: f32 = -math.flt_max;
                for (output_cm.getContactPointsConst()[0..output_cm.num_contact_points], manifold.relative_contact_points_on1.slice(), manifold.relative_contact_points_on2.slice()) |*ccp, *on1, *on2| {
                    on1.* = transform_body1.multiply3x3(Vec3.loadFloat3Unsafe(&ccp.position1));
                    on2.* = local_transform_body2.mulVec3(Vec3.loadFloat3Unsafe(&ccp.position2));
                    penetration_depth = math.max(penetration_depth, on1.sub(on2.*).dot(world_space_normal));
                }
                manifold.penetration_depth = penetration_depth; // We don't have the penetration depth anymore, estimate it

                // Notify callback
                listener.onContactPersisted(body1, body2, &manifold, &settings);
            }

            if (Core.enable_asserts) std.debug.assert(settings.is_sensor or !(body1.isSensor() or body2.isSensor())); // Sensors cannot be converted into regular bodies by a contact callback!
            if (!settings.is_sensor // If one of the bodies is a sensor, don't actually create the constraint
            and ((type1 == .dynamic and settings.inv_mass_scale1 != 0.0) // One of the bodies must have mass to be able to create a contact constraint
                or (type2 == .dynamic and settings.inv_mass_scale2 != 0.0)))
            {
                // Create a new constraint
                const constraint = self.createConstraint(type1, type2, &link_bodies, body1, body2, input_hash, output_handle, world_space_normal, &settings, output_cm.num_contact_points) orelse {
                    contact_allocator.errors = contact_allocator.errors.bitOr(.{ .contact_constraints_full = true });
                    break;
                };

                // TODO(determinism_log): JPH_DET_LOG("GetContactsFromCache: id1: " << inBody1.GetID() << " id2: " << inBody2.GetID() << " key: " << constraint->mSortKey);

                // Calculate scaled mass and inertia
                var inv_i1: Mat44 = undefined;
                if (comptime type1 == .dynamic) {
                    const mp1 = body1.getMotionPropertiesUnchecked().?;
                    constraint.base.inv_mass1 = settings.inv_mass_scale1 * mp1.getInverseMass();
                    inv_i1 = mp1.getInverseInertiaForRotation(transform_body1.getRotation()).mulScalar(settings.inv_inertia_scale1);
                } else {
                    constraint.base.inv_mass1 = 0.0;
                    inv_i1 = Mat44.zero();
                }

                var inv_i2: Mat44 = undefined;
                if (comptime type2 == .dynamic) {
                    const mp2 = body2.getMotionPropertiesUnchecked().?;
                    constraint.base.inv_mass2 = settings.inv_mass_scale2 * mp2.getInverseMass();
                    inv_i2 = mp2.getInverseInertiaForRotation(transform_body2.getRotation()).mulScalar(settings.inv_inertia_scale2);
                } else {
                    constraint.base.inv_mass2 = 0.0;
                    inv_i2 = Mat44.zero();
                }

                // Setup non-penetration constraints
                var ws_contacts: [max_contact_points]RVec3 = undefined;
                for (output_cm.getContactPointsConst()[0..constraint.base.num_contact_points], constraint.getContactPoints()[0..constraint.base.num_contact_points], ws_contacts[0..constraint.base.num_contact_points]) |*ccp, *wcp, *ws_contact| {
                    const p1_ws = transform_body1.mulVec3(Vec3.loadFloat3Unsafe(&ccp.position1));
                    const p2_ws = transform_body2.mulVec3(Vec3.loadFloat3Unsafe(&ccp.position2));

                    // Remember where to apply friction
                    ws_contact.* = p1_ws.add(p2_ws).mulScalar(0.5);

                    wcp.non_penetration_constraint.setTotalLambda(ccp.non_penetration_lambda);
                    wcp.calculateNonPenetrationConstraintProperties(delta_time, gravity, body1, body2, constraint.base.inv_mass1, constraint.base.inv_mass2, inv_i1, inv_i2, p1_ws, p2_ws, world_space_normal, &settings, self.physics_settings.min_velocity_for_restitution);
                }

                // Calculate tangents
                const t = constraint.base.getTangents();

                // Setup friction constraints
                constraint.friction_constraint1.setTotalLambda(output_cm.friction_lambda.f32s[0]);
                constraint.friction_constraint2.setTotalLambda(output_cm.friction_lambda.f32s[1]);
                constraint.angular_friction_constraint.setTotalLambda(output_cm.angular_friction_lambda);
                constraint.calculateFrictionConstraintProperties(body1, body2, constraint.base.inv_mass1, constraint.base.inv_mass2, inv_i1, inv_i2, &ws_contacts, world_space_normal, t.tangent1, t.tangent2, &settings);

                // TODO(debug_renderer): Draw the manifold (if sDrawContactManifolds: constraint->Draw(DebugRenderer::sInstance, *mWriteCache, Color::sYellow))
            }

            // Mark contact as persisted so that we won't fire OnContactRemoved callbacks
            _ = input_cm.flags.fetchOr(CachedManifold.Flags.contact_persisted, .seq_cst);

            // Fetch the next manifold
            input_handle = input_cm.next_with_same_body_pair;
            if (input_handle == ManifoldMap.invalid_handle)
                break;
        }
        output_cached_body_pair.first_cached_manifold = output_handle;
    }

    /// Check if the contact points from the previous frame are reusable and if so copy them.
    /// When the cache was usable and the pair has been handled: returns true (Jolt's outPairHandled).
    pub fn getContactsFromCache(self: *ContactConstraintManager, contact_allocator: *ContactAllocator, body1_in: *Body, body2_in: *Body) bool {
        // Start with not handled (Zolt: the return value)

        // Swap bodies so that body 1 id < body 2 id
        var body1: *Body = undefined;
        var body2: *Body = undefined;
        if (body1_in.getID().lessThan(body2_in.getID())) {
            body1 = body1_in;
            body2 = body2_in;
        } else {
            body1 = body2_in;
            body2 = body1_in;
        }

        // Find the cached body pair
        const body_pair_key = BodyPair.init(body1.getID(), body2.getID());
        const body_pair_hash = body_pair_key.getHash();
        const kv = self.read_cache.?.findBodyPair(body_pair_key, body_pair_hash) orelse return false;
        const input_cbp = kv.getValueConst();

        // Get relative translation
        const inv_r1 = body1.getRotation().conjugated();
        const delta_position = inv_r1.mulVec3(toVec3(body2.getCenterOfMassPosition().sub(body1.getCenterOfMassPosition())));

        // Get old position delta
        const old_delta_position = Vec3.loadFloat3Unsafe(&input_cbp.delta_position);

        // Check if bodies are still roughly in the same relative position
        if (delta_position.sub(old_delta_position).lengthSq() > self.physics_settings.body_pair_cache_max_delta_position_sq)
            return false;

        // Determine relative orientation
        const delta_rotation = inv_r1.mul(body2.getRotation());

        // Reconstruct old quaternion delta
        const old_delta_rotation = Quat.loadFloat3Unsafe(&input_cbp.delta_rotation);

        // Check if bodies are still roughly in the same relative orientation
        // The delta between 2 quaternions p and q is: p q^* = [rotation_axis * sin(angle / 2), cos(angle / 2)]
        // From the W component we can extract the angle: cos(angle / 2) = px * qx + py * qy + pz * qz + pw * qw = p . q
        // Since we want to abort if the rotation is smaller than -angle or bigger than angle, we can write the comparison as |p . q| < cos(angle / 2)
        if (@abs(delta_rotation.dot(old_delta_rotation)) < self.physics_settings.body_pair_cache_cos_max_delta_rotation_div2)
            return false;

        // The cache is valid, return that we've handled this body pair

        // Copy the cached body pair to this frame
        const output_bp_kv = self.write_cache.?.createBodyPair(contact_allocator, body_pair_key, body_pair_hash) orelse return true; // Out of cache space
        const output_cbp = output_bp_kv.getValue();
        output_cbp.* = input_cbp.*;

        // If there were no contacts, we have handled the contact
        if (input_cbp.first_cached_manifold == ManifoldMap.invalid_handle)
            return true;

        // Dispatch to the correct templated form
        // Note: Non-dynamic vs non-dynamic can happen in this case due to one body being a sensor, so we need to have an extended table here
        switch (body1.getMotionType()) {
            inline else => |type1| switch (body2.getMotionType()) {
                inline else => |type2| if (comptime hasCacheFunction(type1, type2))
                    self.templatedGetContactsFromCache(type1, type2, contact_allocator, body1, body2, input_cbp, output_cbp)
                else
                    @panic("ContactConstraintManager.getContactsFromCache: static vs static doesn't exist"),
            },
        }
        return true;
    }

    /// Create a handle for a colliding body pair so that contact constraints can be added between them.
    /// Needs to be called once per body pair per frame before calling AddContactConstraint.
    pub fn addBodyPair(self: *ContactConstraintManager, contact_allocator: *ContactAllocator, body1_in: *const Body, body2_in: *const Body) BodyPairHandle {
        // Swap bodies so that body 1 id < body 2 id
        var body1: *const Body = undefined;
        var body2: *const Body = undefined;
        if (body1_in.getID().lessThan(body2_in.getID())) {
            body1 = body1_in;
            body2 = body2_in;
        } else {
            body1 = body2_in;
            body2 = body1_in;
        }

        // Add an entry
        const body_pair_key = BodyPair.init(body1.getID(), body2.getID());
        const body_pair_hash = body_pair_key.getHash();
        const body_pair_kv = self.write_cache.?.createBodyPair(contact_allocator, body_pair_key, body_pair_hash) orelse return null; // Out of cache space
        const cbp = body_pair_kv.getValue();
        cbp.first_cached_manifold = ManifoldMap.invalid_handle;

        // Get relative translation
        const inv_r1 = body1.getRotation().conjugated();
        const delta_position = inv_r1.mulVec3(toVec3(body2.getCenterOfMassPosition().sub(body1.getCenterOfMassPosition())));

        // Store it
        delta_position.storeFloat3(&cbp.delta_position);

        // Determine relative orientation
        const delta_rotation = inv_r1.mul(body2.getRotation());

        // Store it
        delta_rotation.storeFloat3(&cbp.delta_rotation);

        return cbp;
    }

    /// Internal helper function to add a contact constraint. Templated to the motion type to reduce the amount of branches and calculations.
    fn templatedAddContactConstraint(self: *ContactConstraintManager, comptime type1: MotionType, comptime type2: MotionType, contact_allocator: *ContactAllocator, activate_and_link_bodies: *bool, body_pair_handle: BodyPairHandle, body1: *Body, body2: *Body, manifold: *const ContactManifold) void {
        // Calculate hash
        const key = SubShapeIDPair.init(body1.getID(), manifold.sub_shape_id1, body2.getID(), manifold.sub_shape_id2);
        const key_hash = key.getHash();

        // Determine number of contact points
        const num_contact_points: u32 = manifold.relative_contact_points_on1.len;
        std.debug.assert(num_contact_points <= max_contact_points);
        std.debug.assert(num_contact_points == manifold.relative_contact_points_on2.len);

        // Reserve space for new contact cache entry
        // Note that for dynamic vs dynamic we always require the first body to have a lower body id to get a consistent key
        // under which to look up the contact
        const write_cache = self.write_cache.?;
        const new_manifold_kv = write_cache.createManifold(contact_allocator, key, key_hash, @intCast(num_contact_points)) orelse return; // Out of cache space
        const new_manifold = new_manifold_kv.getValue();
        const new_manifold_handle = write_cache.toHandle(new_manifold_kv);

        // Transform the world space normal to the space of body 2 (this is usually the static body)
        const inverse_transform_body2 = body2.getInverseCenterOfMassTransform();
        inverse_transform_body2.multiply3x3(manifold.world_space_normal).normalized().storeFloat3(&new_manifold.contact_normal);

        // Settings object that gets passed to the callback
        var settings: ContactSettings = .{};
        settings.combined_friction = self.combine_friction(body1, manifold.sub_shape_id1, body2, manifold.sub_shape_id2);
        settings.combined_restitution = self.combine_restitution(body1, manifold.sub_shape_id1, body2, manifold.sub_shape_id2);
        settings.is_sensor = body1.isSensor() or body2.isSensor();

        // Get the contact points for the old cache entry (Zolt: the mutable Find, Jolt sets the `mutable` flags through
        // the const result)
        const old_manifold_kv = self.read_cache.?.findManifoldMut(key, key_hash);
        var old_contact_points: []const CachedContactPoint = &.{};
        if (old_manifold_kv) |old_kv| {
            // Call point persisted listener
            if (self.contact_listener) |listener|
                listener.onContactPersisted(body1, body2, manifold, &settings);

            // Fetch the contact points from the old manifold
            const old_manifold = old_kv.getValue();
            old_contact_points = old_manifold.getContactPointsConst()[0..old_manifold.num_contact_points];

            // Mark contact as persisted so that we won't fire OnContactRemoved callbacks
            _ = old_manifold.flags.fetchOr(CachedManifold.Flags.contact_persisted, .seq_cst);
        } else {
            // Call point added listener
            if (self.contact_listener) |listener|
                listener.onContactAdded(body1, body2, manifold, &settings);

            // No contact points available from old manifold
        }

        // Get inverse transform for body 1
        const inverse_transform_body1 = body1.getInverseCenterOfMassTransform();

        // If one of the bodies is a sensor, don't actually create the constraint
        if (Core.enable_asserts) std.debug.assert(settings.is_sensor or !(body1.isSensor() or body2.isSensor())); // Sensors cannot be converted into regular bodies by a contact callback!
        if (!settings.is_sensor and ((type1 == .dynamic and settings.inv_mass_scale1 != 0.0) // One of the bodies must have mass to be able to create a contact constraint
            or (type2 == .dynamic and settings.inv_mass_scale2 != 0.0)))
        {
            // Create a new constraint
            const constraint = self.createConstraint(type1, type2, activate_and_link_bodies, body1, body2, key_hash, new_manifold_handle, manifold.world_space_normal, &settings, num_contact_points) orelse {
                contact_allocator.errors = contact_allocator.errors.bitOr(.{ .contact_constraints_full = true });

                // Manifold has been created already, we're not filling it in, so we need to reset the contact number of points.
                // Note that we don't hook it up to the body pair cache so that it won't be used as a cache during the next simulation.
                new_manifold.num_contact_points = 0;
                return;
            };

            // TODO(determinism_log): JPH_DET_LOG("AddContactConstraint: id1: " << constraint->mBody1->GetID() << " id2: " << constraint->mBody2->GetID() << " key: " << constraint->mSortKey);

            // Get time step and gravity
            const context = self.update_context.?;
            const delta_time = context.step_delta_time;
            const gravity = context.physics_system.getGravity();

            // Calculate scaled mass and inertia
            var inv_i1: Mat44 = undefined;
            if (comptime type1 == .dynamic) {
                const mp1 = body1.getMotionPropertiesUnchecked().?;
                constraint.base.inv_mass1 = settings.inv_mass_scale1 * mp1.getInverseMass();
                inv_i1 = mp1.getInverseInertiaForRotation(inverse_transform_body1.transposed3x3()).mulScalar(settings.inv_inertia_scale1);
            } else {
                constraint.base.inv_mass1 = 0.0;
                inv_i1 = Mat44.zero();
            }

            var inv_i2: Mat44 = undefined;
            if (comptime type2 == .dynamic) {
                const mp2 = body2.getMotionPropertiesUnchecked().?;
                constraint.base.inv_mass2 = settings.inv_mass_scale2 * mp2.getInverseMass();
                inv_i2 = mp2.getInverseInertiaForRotation(inverse_transform_body2.transposed3x3()).mulScalar(settings.inv_inertia_scale2);
            } else {
                constraint.base.inv_mass2 = 0.0;
                inv_i2 = Mat44.zero();
            }

            var ws_contacts: [max_contact_points]RVec3 = undefined;
            for (0..num_contact_points) |i| {
                // Convert to world space and set positions
                const wcp = &constraint.getContactPoints()[i];
                const p1_ws = manifold.base_offset.addVec3(manifold.relative_contact_points_on1.get(@intCast(i)));
                const p2_ws = manifold.base_offset.addVec3(manifold.relative_contact_points_on2.get(@intCast(i)));

                // Remember where to apply friction
                ws_contacts[i] = p1_ws.add(p2_ws).mulScalar(0.5);

                // Convert to local space to the body
                const p1_ls = toVec3(inverse_transform_body1.mulRVec3(p1_ws));
                const p2_ls = toVec3(inverse_transform_body2.mulRVec3(p2_ws));

                // Store contact points
                const cp = &new_manifold.getContactPoints()[i];
                p1_ls.storeFloat3(&cp.position1);
                p2_ls.storeFloat3(&cp.position2);

                // Check if we have a close contact point from last update
                wcp.non_penetration_constraint.setTotalLambda(0.0);
                for (old_contact_points) |*ccp| {
                    if (Vec3.loadFloat3Unsafe(&ccp.position1).isClose(p1_ls, .{ .max_dist_sq = self.physics_settings.contact_point_preserve_lambda_max_dist_sq }) and
                        Vec3.loadFloat3Unsafe(&ccp.position2).isClose(p2_ls, .{ .max_dist_sq = self.physics_settings.contact_point_preserve_lambda_max_dist_sq }))
                    {
                        // Get lambdas from previous frame
                        wcp.non_penetration_constraint.setTotalLambda(ccp.non_penetration_lambda);
                        break;
                    }
                }

                // Setup velocity constraint
                wcp.calculateNonPenetrationConstraintProperties(delta_time, gravity, body1, body2, constraint.base.inv_mass1, constraint.base.inv_mass2, inv_i1, inv_i2, p1_ws, p2_ws, manifold.world_space_normal, &settings, self.physics_settings.min_velocity_for_restitution);
            }

            // Calculate tangents
            const t = constraint.base.getTangents();

            // Setup friction constraint
            if (old_manifold_kv) |old_kv| {
                const old_manifold = old_kv.getValueConst();
                constraint.friction_constraint1.setTotalLambda(old_manifold.friction_lambda.f32s[0]);
                constraint.friction_constraint2.setTotalLambda(old_manifold.friction_lambda.f32s[1]);
                constraint.angular_friction_constraint.setTotalLambda(old_manifold.angular_friction_lambda);
            } else {
                constraint.friction_constraint1.setTotalLambda(0.0);
                constraint.friction_constraint2.setTotalLambda(0.0);
                constraint.angular_friction_constraint.setTotalLambda(0.0);
            }
            constraint.calculateFrictionConstraintProperties(body1, body2, constraint.base.inv_mass1, constraint.base.inv_mass2, inv_i1, inv_i2, &ws_contacts, manifold.world_space_normal, t.tangent1, t.tangent2, &settings);

            // TODO(debug_renderer): Draw the manifold (if sDrawContactManifolds: constraint->Draw(DebugRenderer::sInstance, *mWriteCache, Color::sOrange))
        } else {
            // Store the contact manifold in the cache
            for (0..num_contact_points) |i| {
                // Convert to local space to the body
                const p1 = toVec3(inverse_transform_body1.mulRVec3(manifold.base_offset.addVec3(manifold.relative_contact_points_on1.get(@intCast(i)))));
                const p2 = toVec3(inverse_transform_body2.mulRVec3(manifold.base_offset.addVec3(manifold.relative_contact_points_on2.get(@intCast(i)))));

                // Create new contact point
                const cp = &new_manifold.getContactPoints()[i];
                p1.storeFloat3(&cp.position1);
                p2.storeFloat3(&cp.position2);

                // Reset contact impulses, we haven't applied any
                cp.non_penetration_lambda = 0.0;
            }

            new_manifold.friction_lambda.f32s[0] = 0.0;
            new_manifold.friction_lambda.f32s[1] = 0.0;
            new_manifold.angular_friction_lambda = 0.0;
        }

        // Store cached contact point in body pair cache
        const cbp: *CachedBodyPair = @ptrCast(@alignCast(body_pair_handle.?));
        new_manifold.next_with_same_body_pair = cbp.first_cached_manifold;
        cbp.first_cached_manifold = new_manifold_handle;
    }

    /// Add a contact constraint for this frame.
    ///
    /// @param contact_allocator The allocator that reserves memory for the contacts
    /// @param activate_and_link_bodies When true, this function attempts to activate and link the bodies and then sets ioActivateAndLinkBodies to false so that it only happens once
    /// @param body_pair_handle The handle for the contact cache for this body pair
    /// @param body1 The first body that is colliding
    /// @param body2 The second body that is colliding
    /// @param manifold The manifold that describes the collision
    ///
    /// This is using the approach described in 'Modeling and Solving Constraints' by Erin Catto presented at GDC 2009 (and later years with slight modifications).
    /// We're using the formulas from slide 50 - 53 combined.
    ///
    /// Euler velocity integration:
    ///
    /// v1' = v1 + M^-1 P
    ///
    /// Impulse:
    ///
    /// P = J^T lambda
    ///
    /// Constraint force:
    ///
    /// lambda = -K^-1 J v1
    ///
    /// Inverse effective mass:
    ///
    /// K = J M^-1 J^T
    ///
    /// Constraint equation (limits movement in 1 axis):
    ///
    /// C = (p2 - p1) . n
    ///
    /// Jacobian (for position constraint)
    ///
    /// J = [-n, -r1 x n, n, r2 x n]
    ///
    /// n = contact normal (pointing away from body 1).
    /// p1, p2 = positions of collision on body 1 and 2.
    /// r1, r2 = contact point relative to center of mass of body 1 and body 2 (r1 = p1 - x1, r2 = p2 - x2).
    /// v1, v2 = (linear velocity, angular velocity): 6 vectors containing linear and angular velocity for body 1 and 2.
    /// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].
    pub fn addContactConstraint(self: *ContactConstraintManager, contact_allocator: *ContactAllocator, activate_and_link_bodies: *bool, body_pair_handle: BodyPairHandle, body1_in: *Body, body2_in: *Body, manifold_in: *const ContactManifold) void {
        // TODO(determinism_log): JPH_DET_LOG("AddContactConstraint: id1: " << inBody1.GetID() << " id2: " << inBody2.GetID()
        //   << " subshape1: " << inManifold.mSubShapeID1 << " subshape2: " << inManifold.mSubShapeID2
        //   << " normal: " << inManifold.mWorldSpaceNormal << " pendepth: " << inManifold.mPenetrationDepth);

        if (Core.enable_asserts) std.debug.assert(manifold_in.world_space_normal.isNormalized(.{}));

        // Swap bodies so that body 1 id < body 2 id
        var manifold: *const ContactManifold = undefined;
        var body1: *Body = undefined;
        var body2: *Body = undefined;
        var temp: ContactManifold = undefined;
        if (body2_in.getID().lessThan(body1_in.getID())) {
            body1 = body2_in;
            body2 = body1_in;
            temp = manifold_in.swapShapes();
            manifold = &temp;
        } else {
            body1 = body1_in;
            body2 = body2_in;
            manifold = manifold_in;
        }

        // Dispatch to the correct templated form
        // Note: Non-dynamic vs non-dynamic can happen in this case due to one body being a sensor, so we need to have an extended table here
        switch (body1.getMotionType()) {
            inline else => |type1| switch (body2.getMotionType()) {
                inline else => |type2| if (comptime hasCacheFunction(type1, type2))
                    self.templatedAddContactConstraint(type1, type2, contact_allocator, activate_and_link_bodies, body_pair_handle, body1, body2, manifold)
                else
                    @panic("ContactConstraintManager.addContactConstraint: static vs static doesn't exist"),
            },
        }
    }

    /// Called by continuous collision detection to notify the contact listener that a contact was added
    /// @param contact_allocator The allocator that reserves memory for the contacts
    /// @param body1 The first body that is colliding
    /// @param body2 The second body that is colliding
    /// @param manifold The manifold that describes the collision
    /// @param settings The calculated contact settings (may be overridden by the contact listener)
    pub fn onCCDContactAdded(self: *ContactConstraintManager, contact_allocator: *ContactAllocator, body1_in: *const Body, body2_in: *const Body, manifold_in: *const ContactManifold, settings: *ContactSettings) void {
        if (Core.enable_asserts) std.debug.assert(manifold_in.world_space_normal.isNormalized(.{}));

        // Calculate contact settings
        settings.combined_friction = self.combine_friction(body1_in, manifold_in.sub_shape_id1, body2_in, manifold_in.sub_shape_id2);
        settings.combined_restitution = self.combine_restitution(body1_in, manifold_in.sub_shape_id1, body2_in, manifold_in.sub_shape_id2);
        settings.is_sensor = false; // For now, no sensors are supported during CCD

        // The remainder of this function only deals with calling contact callbacks, if there's no contact callback we also don't need to do this work
        if (self.contact_listener) |listener| {
            // Swap bodies so that body 1 id < body 2 id
            var manifold: *const ContactManifold = undefined;
            var body1: *const Body = undefined;
            var body2: *const Body = undefined;
            var temp: ContactManifold = undefined;
            var swapped = false;
            if (body2_in.getID().lessThan(body1_in.getID())) {
                body1 = body2_in;
                body2 = body1_in;
                temp = manifold_in.swapShapes();
                manifold = &temp;
                swapped = true;
            } else {
                body1 = body1_in;
                body2 = body2_in;
                manifold = manifold_in;
            }

            // Calculate hash
            const key = SubShapeIDPair.init(body1.getID(), manifold.sub_shape_id1, body2.getID(), manifold.sub_shape_id2);
            const key_hash = key.getHash();

            // Check if we already created this contact this physics update
            const new_manifold_kv = self.write_cache.?.findOrCreate(contact_allocator, key, key_hash, 0);
            if (new_manifold_kv.created) {
                // This contact is new for this physics update, check if previous update we already had this contact.
                // (Zolt: the mutable Find, Jolt sets the `mutable` flags through the const result)
                if (self.read_cache.?.findManifoldMut(key, key_hash)) |old_manifold_kv| {
                    // Existing contact
                    listener.onContactPersisted(body1, body2, manifold, settings);

                    // Mark contact as persisted so that we won't fire OnContactRemoved callbacks
                    _ = old_manifold_kv.getValue().flags.fetchOr(CachedManifold.Flags.contact_persisted, .seq_cst);
                } else {
                    // New contact
                    listener.onContactAdded(body1, body2, manifold, settings);
                }

                // Check if the cache is full
                if (new_manifold_kv.kv) |kv| {
                    // We don't store any contact points in this manifold as it is not for caching impulses, we only need to know that the contact was created
                    const new_manifold = kv.getValue();
                    new_manifold.contact_normal = .{ .x = 0, .y = 0, .z = 0 };
                    _ = new_manifold.flags.fetchOr(CachedManifold.Flags.ccd_contact, .seq_cst);
                }
            } else {
                // Already found this contact this physics update.
                // Note that we can trigger OnContactPersisted multiple times per physics update, but otherwise we have no way of obtaining the settings
                listener.onContactPersisted(body1, body2, manifold, settings);
            }

            // If we swapped body1 and body2 we need to swap the mass scales back
            if (swapped) {
                std.mem.swap(f32, &settings.inv_mass_scale1, &settings.inv_mass_scale2);
                std.mem.swap(f32, &settings.inv_inertia_scale1, &settings.inv_inertia_scale2);
                // Note we do not need to negate the relative surface velocity as it is not applied by the CCD collision constraint
            }
        }

        if (Core.enable_asserts) std.debug.assert(settings.is_sensor or !(body1_in.isSensor() or body2_in.isSensor())); // Sensors cannot be converted into regular bodies by a contact callback!
    }

    /// Finalizes the contact cache, the contact cache that was generated during the calls to AddContactConstraint in this update
    /// will be used from now on to read from. After finalizing the contact cache, the contact removed callbacks will be called.
    /// inExpectedNumBodyPairs / inExpectedNumManifolds are the amount of body pairs / manifolds found in the previous step and is
    /// used to determine the amount of buckets the contact cache hash map will use in the next update.
    pub fn finalizeContactCacheAndCallContactPointRemovedCallbacks(self: *ContactConstraintManager, expected_num_body_pairs: u32, expected_num_manifolds: u32) void {
        if (Core.enable_asserts) {
            // Mark cache as finalized
            const old_write_cache = &self.cache[self.cache_write_idx];
            old_write_cache.finalize();

            // Check that the count of body pairs and manifolds that we tracked outside of the cache (to avoid contention on an atomic) is correct
            std.debug.assert(old_write_cache.getNumBodyPairs() == expected_num_body_pairs);
            std.debug.assert(old_write_cache.getNumManifolds() == expected_num_manifolds);
        }

        // Buffers are now complete, make write buffer the read buffer
        self.cache_write_idx ^= 1;

        // Get the old read cache / new write cache
        const old_read_cache = &self.cache[self.cache_write_idx];

        // Call the contact point removal callbacks
        if (self.contact_listener) |listener|
            old_read_cache.contactPointRemovedCallbacks(listener);

        // We're done with the old read cache now
        old_read_cache.clear();

        // Use the amount of contacts from the last iteration to determine the amount of buckets to use in the hash map for the next iteration
        old_read_cache.prepare(expected_num_body_pairs, expected_num_manifolds);
    }

    /// Zolt addition (SimulationArchitecture D10): called by a poisoned ContactRemovedCallbacks job instead of
    /// FinalizeContactCacheAndCallContactPointRemovedCallbacks. The write cache of the update is incomplete: it is
    /// discarded (cleared, keeping its bucket count) instead of becoming the read cache, the read cache stays (the
    /// ContactPersisted marks that the update set on it are reset, so it is exactly the cache from before the update)
    /// and no listener is called. Both caches stay structurally valid, so `restoreState` works afterwards.
    pub fn finalizeContactCacheAfterOutOfMemory(self: *ContactConstraintManager) void {
        // Discard the incomplete write cache
        self.cache[self.cache_write_idx].clear();

        // Keep the read cache as it was before the update
        self.cache[self.cache_write_idx ^ 1].resetContactPersisted();
    }

    /// Check if 2 bodies were in contact during the last simulation step. Since contacts are only detected between active bodies, at least one of the bodies must be active.
    /// Uses the read collision cache to determine if 2 bodies are in contact.
    pub fn wereBodiesInContact(self: *const ContactConstraintManager, body1_id: BodyID, body2_id: BodyID) bool {
        // The body pair needs to be in the cache and it needs to have a manifold (otherwise it's just a record indicating that there are no collisions)
        const read_cache = &self.cache[self.cache_write_idx ^ 1];
        const key = if (body1_id.lessThan(body2_id)) BodyPair.init(body1_id, body2_id) else BodyPair.init(body2_id, body1_id);
        const key_hash = key.getHash();
        const kv = read_cache.findBodyPair(key, key_hash) orelse return false;
        return kv.getValueConst().first_cached_manifold != ManifoldMap.invalid_handle;
    }

    /// Get the number of contact constraints that were found
    pub fn getNumConstraints(self: *const ContactConstraintManager) u32 {
        return @min(@as(u32, @truncate(self.num_constraints_and_next_constraint_offset.load(.monotonic))), self.max_constraints);
    }

    /// Update constraint indices to constraint offsets
    pub fn constraintIdxToConstraintOffset(self: *const ContactConstraintManager, constraint_idx: []u32) void {
        for (constraint_idx) |*i|
            i.* = self.constraint_idx_to_offset.items[i.*];
    }

    /// Sort contact constraints deterministically
    pub fn sortContacts(self: *const ContactConstraintManager, constraint_offsets: []u32) void {
        quickSort(u32, constraint_offsets, self, struct {
            fn lessThan(ccm: *const ContactConstraintManager, lhs_offset: u32, rhs_offset: u32) bool {
                const lhs = ccm.getConstraintBase(lhs_offset);
                const rhs = ccm.getConstraintBase(rhs_offset);

                // Most of the time the sort key will be different so we sort on that
                if (lhs.sort_key != rhs.sort_key)
                    return lhs.sort_key < rhs.sort_key;

                // If they're equal we use the IDs of body 1 to order
                if (lhs.body1 != rhs.body1)
                    return lhs.body1.getID().lessThan(rhs.body1.getID());

                // If they're still equal we use the IDs of body 2 to order
                if (lhs.body2 != rhs.body2)
                    return lhs.body2.getID().lessThan(rhs.body2.getID());

                if (Core.enable_asserts) std.debug.assert(lhs_offset == rhs_offset); // Hash collision, ordering will be inconsistent
                return false;
            }
        }.lessThan);
    }

    /// Get the affected bodies for a given constraint
    pub fn getAffectedBodies(self: *const ContactConstraintManager, constraint_offset: u32) AffectedBodies {
        const constraint = self.getConstraintBase(constraint_offset);
        return .{ .body1 = constraint.body1, .body2 = constraint.body2 };
    }

    /// The velocities that the solver works on (sGetVelocities' out parameters)
    const Velocities = struct {
        linear_velocity1: Vec3,
        angular_velocity1: Vec3,
        linear_velocity2: Vec3,
        angular_velocity2: Vec3,
    };

    /// Read the velocities from the motion properties
    fn getVelocities(comptime type1: MotionType, comptime type2: MotionType, motion_properties1: ?*const MotionProperties, motion_properties2: ?*const MotionProperties) Velocities {
        var v: Velocities = undefined;

        if (comptime type1 != .static) {
            v.linear_velocity1 = motion_properties1.?.getLinearVelocity();
            v.angular_velocity1 = motion_properties1.?.getAngularVelocity();
        } else if (builtin.mode == .Debug) {
            v.linear_velocity1 = Vec3.nan();
            v.angular_velocity1 = Vec3.nan();
        }

        if (comptime type2 != .static) {
            v.linear_velocity2 = motion_properties2.?.getLinearVelocity();
            v.angular_velocity2 = motion_properties2.?.getAngularVelocity();
        } else if (builtin.mode == .Debug) {
            v.linear_velocity2 = Vec3.nan();
            v.angular_velocity2 = Vec3.nan();
        }

        return v;
    }

    /// Apply changed velocities to the motion properties
    fn setVelocities(comptime type1: MotionType, comptime type2: MotionType, motion_properties1: ?*MotionProperties, motion_properties2: ?*MotionProperties, v: *const Velocities) void {
        if (comptime type1 == .dynamic) {
            motion_properties1.?.applyLinearVelocityStep(v.linear_velocity1);
            motion_properties1.?.applyAngularVelocityStep(v.angular_velocity1);
        }

        if (comptime type2 == .dynamic) {
            motion_properties2.?.applyLinearVelocityStep(v.linear_velocity2);
            motion_properties2.?.applyAngularVelocityStep(v.angular_velocity2);
        }
    }

    /// Internal helper function to warm start contact constraint. Templated to the motion type to reduce the amount of branches and calculations.
    fn warmStartConstraint(comptime type1: MotionType, comptime type2: MotionType, constraint_base: *ContactConstraintBase, motion_properties1: ?*MotionProperties, motion_properties2: ?*MotionProperties, warm_start_impulse_ratio: f32) void {
        const constraint = ContactConstraint(type1, type2).fromBase(constraint_base);

        var any_impulse_applied = false;

        // Calculate tangents
        const t = constraint.base.getTangents();

        // Get velocities
        var v = getVelocities(type1, type2, motion_properties1, motion_properties2);

        const ws_normal = constraint.base.getWorldSpaceNormal();

        // Warm starting: Apply impulse from last frame
        if (constraint.friction_constraint1.isActive() and constraint.friction_constraint1.warmStart(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, t.tangent1, warm_start_impulse_ratio))
            any_impulse_applied = true;
        if (constraint.friction_constraint2.isActive() and constraint.friction_constraint2.warmStart(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, t.tangent2, warm_start_impulse_ratio))
            any_impulse_applied = true;
        if (constraint.angular_friction_constraint.isActive() and constraint.angular_friction_constraint.warmStart(&v.angular_velocity1, &v.angular_velocity2, warm_start_impulse_ratio))
            any_impulse_applied = true;

        for (constraint.getContactPoints()[0..constraint.base.num_contact_points]) |*wcp| {
            if (wcp.non_penetration_constraint.warmStart(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, ws_normal, warm_start_impulse_ratio))
                any_impulse_applied = true;
        }

        // Apply changed velocities
        if (any_impulse_applied)
            setVelocities(type1, type2, motion_properties1, motion_properties2, &v);
    }

    /// Apply last frame's impulses as an initial guess for this frame's impulses
    /// (`callback`: a pointer to a CalculateSolverSteps or DummyCalculateSolverSteps, called with the motion properties
    /// of each dynamic body: `callback.call(MotionProperties, mp)`)
    pub fn warmStartVelocityConstraints(self: *ContactConstraintManager, constraint_offsets: []const u32, warm_start_impulse_ratio: f32, callback: anytype) void {
        if (constraint_offsets.len == 0)
            return;

        var next_constraint: ?*ContactConstraintBase = self.getConstraintBase(constraint_offsets[0]);
        var next_constraint_offset: usize = 1;
        while (next_constraint) |constraint| : (next_constraint_offset += 1) {
            if (next_constraint_offset < constraint_offsets.len) {
                next_constraint = self.getConstraintBase(constraint_offsets[next_constraint_offset]);
                prefetchL1(next_constraint.?);
            } else next_constraint = null;

            // Dispatch to the correct templated form
            const body1 = constraint.body1;
            const body2 = constraint.body2;
            const motion_properties1 = body1.getMotionPropertiesUncheckedMut();
            const motion_properties2 = body2.getMotionPropertiesUncheckedMut();
            switch (body1.getMotionType()) {
                inline else => |type1| switch (body2.getMotionType()) {
                    inline else => |type2| if (comptime hasSolverFunction(type1, type2))
                        warmStartConstraint(type1, type2, constraint, motion_properties1, motion_properties2, warm_start_impulse_ratio)
                    else
                        @panic("ContactConstraintManager.warmStartVelocityConstraints: a contact without a dynamic body doesn't exist"),
                },
            }

            // Call callbacks
            if (body1.isDynamic())
                callback.call(MotionProperties, motion_properties1.?);
            if (body2.isDynamic())
                callback.call(MotionProperties, motion_properties2.?);
        }
    }

    /// Internal helper function to solve a single velocity constraint. Templated to the motion type to reduce the amount of branches and calculations.
    fn solveVelocityConstraint(comptime type1: MotionType, comptime type2: MotionType, constraint_base: *ContactConstraintBase, motion_properties1: ?*MotionProperties, motion_properties2: ?*MotionProperties) bool {
        const constraint = ContactConstraint(type1, type2).fromBase(constraint_base);

        var any_impulse_applied = false;

        // Calculate tangents
        const t = constraint.base.getTangents();

        // Get velocities
        var v = getVelocities(type1, type2, motion_properties1, motion_properties2);

        const linear_friction_active = constraint.friction_constraint1.isActive() or constraint.friction_constraint2.isActive();
        const angular_friction_active = constraint.angular_friction_constraint.isActive();

        const contact_points = constraint.getContactPoints()[0..constraint.base.num_contact_points];

        // Calculate max impulse that can be applied. Note that we're using the non-penetration impulse from the previous iteration here.
        // We do this because non-penetration is more important so is solved last (the last things that are solved in an iterative solver
        // contribute the most).
        var max_linear_lambda: f32 = 0.0;
        var max_angular_lambda: f32 = 0.0;
        if (linear_friction_active or angular_friction_active) {
            for (contact_points) |*wcp| {
                const lambda = wcp.non_penetration_constraint.getTotalLambda();
                max_linear_lambda += lambda;
                max_angular_lambda += wcp.distance_to_friction_center * lambda;
            }
            max_linear_lambda *= constraint.base.combined_friction;
            max_angular_lambda *= constraint.base.combined_friction;
        }

        // First apply friction constraint (non-penetration is more important than friction)
        if (linear_friction_active) {
            // Calculate impulse to stop motion in tangential direction
            var lambda1 = constraint.friction_constraint1.solveVelocityConstraintGetTotalLambda(v.linear_velocity1, v.angular_velocity1, v.linear_velocity2, v.angular_velocity2, t.tangent1);
            var lambda2 = constraint.friction_constraint2.solveVelocityConstraintGetTotalLambda(v.linear_velocity1, v.angular_velocity1, v.linear_velocity2, v.angular_velocity2, t.tangent2);

            // If the total lambda that we will apply is too large, scale it back
            const total_lambda_sq = math.square(lambda1) + math.square(lambda2);
            if (total_lambda_sq > math.square(max_linear_lambda) + math.flt_min) // ensure total_lambda_sq > FLT_MIN to avoid division by zero in MulRSqrtApproximate
            {
                const scale = math.mulRSqrtApproximate(max_linear_lambda, total_lambda_sq);
                lambda1 *= scale;
                lambda2 *= scale;
            }

            // Apply the friction impulse
            if (constraint.friction_constraint1.solveVelocityConstraintApplyLambda(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, t.tangent1, lambda1))
                any_impulse_applied = true;
            if (constraint.friction_constraint2.solveVelocityConstraintApplyLambda(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, t.tangent2, lambda2))
                any_impulse_applied = true;
        }

        // Apply angular friction
        const ws_normal = constraint.base.getWorldSpaceNormal();
        if (angular_friction_active and constraint.angular_friction_constraint.solveVelocityConstraint(&v.angular_velocity1, &v.angular_velocity2, ws_normal, -max_angular_lambda, max_angular_lambda))
            any_impulse_applied = true;

        // Then apply all non-penetration constraints
        for (contact_points) |*wcp| {
            // Calculate impulse
            var total_lambda = wcp.non_penetration_constraint.solveVelocityConstraintGetTotalLambda(v.linear_velocity1, v.angular_velocity1, v.linear_velocity2, v.angular_velocity2, ws_normal);

            // Contact constraints can only push and not pull
            total_lambda = math.max(total_lambda, 0.0);

            // Apply impulse
            if (wcp.non_penetration_constraint.solveVelocityConstraintApplyLambda(&v.linear_velocity1, &v.angular_velocity1, &v.linear_velocity2, &v.angular_velocity2, constraint.base.inv_mass1, constraint.base.inv_mass2, ws_normal, total_lambda))
                any_impulse_applied = true;
        }

        if (!any_impulse_applied)
            return false;

        setVelocities(type1, type2, motion_properties1, motion_properties2, &v);
        return true;
    }

    /// Solve velocity constraints, when almost nothing changes this should only apply very small impulses
    /// since we're warm starting with the total impulse applied in the last frame above.
    ///
    /// Friction wise we're using the Coulomb friction model which says that:
    ///
    /// |F_T| <= mu |F_N|
    ///
    /// Where F_T is the tangential force, F_N is the normal force and mu is the friction coefficient
    ///
    /// In impulse terms this becomes:
    ///
    /// |lambda_T| <= mu |lambda_N|
    ///
    /// And the constraint that needs to be applied is exactly the same as a non penetration constraint
    /// except that we use a tangent instead of a normal. The tangent should point in the direction of the
    /// tangential velocity of the point:
    ///
    /// J = [-T, -r1 x T, T, r2 x T]
    ///
    /// Where T is the tangent.
    ///
    /// See slide 42 and 43.
    ///
    /// Restitution is implemented as a velocity bias (see slide 41):
    ///
    /// b = e v_n^-
    ///
    /// e = the restitution coefficient, v_n^- is the normal velocity prior to the collision
    ///
    /// Restitution is only applied when v_n^- is large enough and the points are moving towards collision
    pub fn solveVelocityConstraints(self: *ContactConstraintManager, constraint_offsets: []const u32) bool {
        if (constraint_offsets.len == 0)
            return false;

        var any_impulse_applied = false;

        var next_constraint: ?*ContactConstraintBase = self.getConstraintBase(constraint_offsets[0]);
        var next_constraint_offset: usize = 1;
        while (next_constraint) |constraint| : (next_constraint_offset += 1) {
            if (next_constraint_offset < constraint_offsets.len) {
                next_constraint = self.getConstraintBase(constraint_offsets[next_constraint_offset]);
                prefetchL1(next_constraint.?);
            } else next_constraint = null;

            // Dispatch to the correct templated form
            const body1 = constraint.body1;
            const body2 = constraint.body2;
            const applied = switch (body1.getMotionType()) {
                inline else => |type1| switch (body2.getMotionType()) {
                    inline else => |type2| if (comptime hasSolverFunction(type1, type2))
                        solveVelocityConstraint(type1, type2, constraint, body1.getMotionPropertiesUncheckedMut(), body2.getMotionPropertiesUncheckedMut())
                    else
                        @panic("ContactConstraintManager.solveVelocityConstraints: a contact without a dynamic body doesn't exist"),
                },
            };
            any_impulse_applied = any_impulse_applied or applied;
        }

        return any_impulse_applied;
    }

    /// Internal helper function to store lambdas applied during sSolveVelocityConstraint.
    fn storeConstraintAppliedImpulses(comptime type1: MotionType, comptime type2: MotionType, constraint_base: *ContactConstraintBase, manifold_cache: *ManifoldCache) void {
        const constraint = ContactConstraint(type1, type2).fromBase(constraint_base);
        const cached_manifold = manifold_cache.fromHandleMut(constraint.base.cached_manifold_handle).getValue();

        for (constraint.getContactPointsConst()[0..constraint.base.num_contact_points], cached_manifold.getContactPoints()[0..constraint.base.num_contact_points]) |*wcp, *ccp|
            ccp.non_penetration_lambda = wcp.non_penetration_constraint.getTotalLambda();

        cached_manifold.friction_lambda.f32s[0] = constraint.friction_constraint1.getTotalLambda();
        cached_manifold.friction_lambda.f32s[1] = constraint.friction_constraint2.getTotalLambda();
        cached_manifold.angular_friction_lambda = constraint.angular_friction_constraint.getTotalLambda();
    }

    /// Save back the lambdas to the contact cache for the next warm start
    pub fn storeAppliedImpulses(self: *const ContactConstraintManager, constraint_offsets: []const u32) void {
        if (constraint_offsets.len == 0)
            return;

        // Copy back total applied impulse to cache for the next frame
        const write_cache = self.write_cache.?;
        var next_constraint: ?*ContactConstraintBase = self.getConstraintBase(constraint_offsets[0]);
        var next_constraint_offset: usize = 1;
        while (next_constraint) |constraint| : (next_constraint_offset += 1) {
            if (next_constraint_offset < constraint_offsets.len) {
                next_constraint = self.getConstraintBase(constraint_offsets[next_constraint_offset]);
                prefetchL1(next_constraint.?);
            } else next_constraint = null;

            // Dispatch to the correct templated form
            switch (constraint.body1.getMotionType()) {
                inline else => |type1| switch (constraint.body2.getMotionType()) {
                    inline else => |type2| if (comptime hasSolverFunction(type1, type2))
                        storeConstraintAppliedImpulses(type1, type2, constraint, write_cache)
                    else
                        @panic("ContactConstraintManager.storeAppliedImpulses: a contact without a dynamic body doesn't exist"),
                },
            }
        }
    }

    /// Internal helper function to solve a single position constraint. Templated to the motion type to reduce the amount of branches and calculations.
    fn solvePositionConstraint(comptime type1: MotionType, comptime type2: MotionType, constraint_base: *ContactConstraintBase, body1: *Body, body2: *Body, settings: *const PhysicsSettings, manifold_cache: *const ManifoldCache) bool {
        const constraint = ContactConstraint(type1, type2).fromBase(constraint_base);
        const cached_manifold = manifold_cache.fromHandle(constraint.base.cached_manifold_handle).getValueConst();

        // Get transforms
        const transform1 = body1.getCenterOfMassTransform();
        const transform2 = body2.getCenterOfMassTransform();

        const ws_normal = constraint.base.getWorldSpaceNormal();

        var any_impulse_applied = false;

        for (constraint.getContactPoints()[0..constraint.base.num_contact_points], cached_manifold.getContactPointsConst()[0..constraint.base.num_contact_points]) |*wcp, *ccp| {
            // Calculate new contact point positions in world space (the bodies may have moved)
            const p1 = transform1.mulVec3(Vec3.loadFloat3Unsafe(&ccp.position1));
            const p2 = transform2.mulVec3(Vec3.loadFloat3Unsafe(&ccp.position2));

            // Calculate separation along the normal (negative if interpenetrating)
            // Allow a little penetration by default (PhysicsSettings::mPenetrationSlop) to avoid jittering between contact/no-contact which wipes out the contact cache and warm start impulses
            // Clamp penetration to a max PhysicsSettings::mMaxPenetrationDistance so that we don't apply a huge impulse if we're penetrating a lot
            const separation = math.max(toVec3(p2.sub(p1)).dot(ws_normal) + settings.penetration_slop, -settings.max_penetration_distance);

            // Only enforce constraint when separation < 0 (otherwise we're apart)
            if (separation < 0.0) {
                // Calculate scaled inertia
                const inv_i1 = if (comptime type1 == .dynamic) body1.getInverseInertia().mulScalar(constraint.base.inv_inertia_scale1) else Mat44.zero();
                const inv_i2 = if (comptime type2 == .dynamic) body2.getInverseInertia().mulScalar(constraint.base.inv_inertia_scale2) else Mat44.zero();

                // Calculate collision points relative to body
                const p = p1.add(p2).mulScalar(0.5);
                const r1 = toVec3(p.sub(body1.getCenterOfMassPosition()));
                const r2 = toVec3(p.sub(body2.getCenterOfMassPosition()));

                // Update constraint properties (bodies may have moved)
                wcp.non_penetration_constraint.calculateConstraintProperties(constraint.base.inv_mass1, inv_i1, r1, constraint.base.inv_mass2, inv_i2, r2, ws_normal, .{});

                // Solve position errors
                if (wcp.non_penetration_constraint.solvePositionConstraint(body1, constraint.base.inv_mass1, body2, constraint.base.inv_mass2, ws_normal, separation, settings.baumgarte))
                    any_impulse_applied = true;
            }
        }

        return any_impulse_applied;
    }

    /// Solve position constraints.
    /// This is using the approach described in 'Modeling and Solving Constraints' by Erin Catto presented at GDC 2007.
    /// On slide 78 it is suggested to split up the Baumgarte stabilization for positional drift so that it does not
    /// actually add to the momentum. We combine an Euler velocity integrate + a position integrate and then discard the velocity
    /// change.
    ///
    /// Constraint force:
    ///
    /// lambda = -K^-1 b
    ///
    /// Baumgarte stabilization:
    ///
    /// b = beta / dt C
    ///
    /// beta = baumgarte stabilization factor.
    /// dt = delta time.
    pub fn solvePositionConstraints(self: *ContactConstraintManager, constraint_offsets: []const u32) bool {
        if (constraint_offsets.len == 0)
            return false;

        var any_impulse_applied = false;

        const write_cache = self.write_cache.?;
        var next_constraint: ?*ContactConstraintBase = self.getConstraintBase(constraint_offsets[0]);
        var next_constraint_offset: usize = 1;
        while (next_constraint) |constraint| : (next_constraint_offset += 1) {
            if (next_constraint_offset < constraint_offsets.len) {
                next_constraint = self.getConstraintBase(constraint_offsets[next_constraint_offset]);
                prefetchL1(next_constraint.?);
            } else next_constraint = null;

            // Fetch bodies
            const body1 = constraint.body1;
            const body2 = constraint.body2;

            // Dispatch to the correct templated form
            const applied = switch (body1.getMotionType()) {
                inline else => |type1| switch (body2.getMotionType()) {
                    inline else => |type2| if (comptime hasSolverFunction(type1, type2))
                        solvePositionConstraint(type1, type2, constraint, body1, body2, self.physics_settings, write_cache)
                    else
                        @panic("ContactConstraintManager.solvePositionConstraints: a contact without a dynamic body doesn't exist"),
                },
            };
            any_impulse_applied = any_impulse_applied or applied;
        }

        return any_impulse_applied;
    }

    /// Recycle the constraint buffer. Should be called between collision simulation steps.
    pub fn recycleConstraintBuffer(self: *ContactConstraintManager) void {
        // Reset constraint array
        self.num_constraints_and_next_constraint_offset.store(0, .seq_cst);

        // Store read / write cache
        self.read_cache = &self.cache[self.cache_write_idx ^ 1];
        self.write_cache = &self.cache[self.cache_write_idx];
    }

    /// Terminate the constraint buffer. Should be called after simulation ends.
    /// (Zolt: TempArrays that were never allocated are skipped, also when PrepareConstraintBuffer did not run)
    pub fn finishConstraintBuffer(self: *ContactConstraintManager) void {
        // Free constraints buffer
        if (self.update_context) |context| {
            self.constraint_idx_to_offset.free(context.temp_allocator);
            self.constraints.free(context.temp_allocator);
        }
        self.num_constraints_and_next_constraint_offset.store(0, .seq_cst);

        // Reset update context
        self.update_context = null;
        self.read_cache = null;
        self.write_cache = null;
    }

    // TODO(debug_renderer): Drawing properties (static bool sDrawContactPoint, sDrawSupportingFaces,
    // sDrawContactPointReduction, sDrawContactManifolds)

    /// Saving state for replay
    /// (Zolt: Allocator.Error from the sorted arrays)
    pub fn saveState(self: *const ContactConstraintManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        try self.cache[self.cache_write_idx ^ 1].saveState(self.allocator, stream, filter);
    }

    /// Restoring state for replay. Returns false when failed.
    /// (Zolt: Allocator.Error from the sorted arrays of the validating path; the write cache is empty afterwards)
    pub fn restoreState(self: *ContactConstraintManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!bool {
        const success = self.cache[self.cache_write_idx].restoreState(self.allocator, &self.cache[self.cache_write_idx ^ 1], stream, filter) catch |err| {
            self.cache[self.cache_write_idx].clear();
            return err;
        };

        // If this is the last part, the cache is finalized
        if (stream.isLastPart()) {
            self.cache_write_idx ^= 1;
            self.cache[self.cache_write_idx].clear();
        }

        return success;
    }
};

test "ContactConstraintManager layout" {
    const CCM = ContactConstraintManager;
    try std.testing.expectEqual(@as(u32, 504), CCM.max_constraint_size);
    try std.testing.expectEqual(@as(u32, std.math.maxInt(u32) / 504), CCM.max_contact_constraints_limit);
    try std.testing.expectEqual(@as(u32, std.math.maxInt(u32) / 40), CCM.max_body_pairs_limit);
    try std.testing.expectEqual(@as(i32, 0), CCM.CachedManifold.getRequiredExtraSize(0));
    try std.testing.expectEqual(@as(i32, 0), CCM.CachedManifold.getRequiredExtraSize(1));
    try std.testing.expectEqual(@as(i32, 84), CCM.CachedManifold.getRequiredExtraSize(4));
    try std.testing.expectEqual(@as(i32, 144), CCM.CachedManifold.getRequiredTotalSize(4));
    try std.testing.expectEqual(@as(usize, 312), @sizeOf(CCM.ContactConstraint(.dynamic, .dynamic)));
    try std.testing.expectEqual(@as(usize, 64), @sizeOf(CCM.WorldContactPoint(.dynamic, .dynamic)));
    try std.testing.expectEqual(@as(usize, 0), @offsetOf(CCM.ContactConstraint(.static, .dynamic), "base"));
    try std.testing.expectEqual(@as(usize, 16), @offsetOf(CCM.ContactConstraintBase, "sort_key"));
    try std.testing.expectEqual(@as(usize, 28), @offsetOf(CCM.CachedManifold, "flags"));
    try std.testing.expectEqual(@as(usize, 32), @offsetOf(CCM.CachedManifold, "contact_points"));
}
