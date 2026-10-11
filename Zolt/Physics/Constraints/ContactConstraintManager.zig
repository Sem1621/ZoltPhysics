//! Port of: Jolt/Physics/Constraints/ContactConstraintManager.h, Jolt/Physics/Constraints/ContactConstraintManager.cpp
//! Status: stub
//! Missing: everything that needs contacts (Wave A, A4, SimulationArchitecture D10): the double buffered ManifoldCache
//! on LockFreeHashMap (CachedContactPoint, CachedManifold, CachedBodyPair, the contact allocator context),
//! GetContactsFromCache, AddBodyPair, AddContactConstraint, ValidateContactPoint, ConstraintIdxToConstraintOffset,
//! SortContacts, GetAffectedBodies, WarmStartVelocityConstraints, SolveVelocityConstraints, StoreAppliedImpulses,
//! SolvePositionConstraints, OnCCDContactAdded, the removed contact callbacks, WereBodiesInContact on a real cache, and
//! the save / restore of a non empty cache. JPH_DEBUG_RENDERER drawing: TODO(debug_renderer).
//!
//! F2 stub with the final names and signatures of what PhysicsSystem uses (SimulationArchitecture section 2):
//! - The contact cache is always empty (no contact constraint can be created yet). What PhysicsSystem calls on the
//!   gravity-only path is real for an empty cache: the temp constraint buffer (two TempArrays, allocated and freed at
//!   Jolt's positions with Jolt's sizes), the contact allocator counters, the cache swap of
//!   FinalizeContactCacheAndCallContactPointRemovedCallbacks, and SaveState / RestoreState, which write / read Jolt's
//!   exact bytes for an empty cache (0 body pairs, 0 CCD manifolds).
//! - The functions that would need a contact `@panic` with "ContactConstraintManager: A4". They cannot be reached
//!   while the broad phase reports no body pair to the narrow phase (PhysicsSystem's ProcessBodyPair is a REGION R2
//!   stub that panics first).
//! - Jolt's constructor (`ContactConstraintManager(const PhysicsSettings &)`) and `Init` are one `init` (porting
//!   guide: constructor + Init), called in place by PhysicsSystem.init.
//! - `finalizeContactCacheAfterOutOfMemory` is the Zolt addition of D10 (a poisoned ContactRemovedCallbacks job calls
//!   it instead of swapping the caches).

const std = @import("std");
const Allocator = std.mem.Allocator;
const math = @import("../../Math/Math.zig");
const TempAllocatorFile = @import("../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempArray = TempAllocatorFile.TempArray;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const ContactListener = @import("../Collision/ContactListener.zig").ContactListener;
const PhysicsSettings = @import("../PhysicsSettings.zig").PhysicsSettings;
const PhysicsUpdateError = @import("../EPhysicsUpdateError.zig").PhysicsUpdateError;
const PhysicsUpdateContext = @import("../PhysicsUpdateContext.zig").PhysicsUpdateContext;
const StateRecorderFile = @import("../StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;

/// A contact constraint manager manages all contacts between two bodies
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
pub const ContactConstraintManager = struct {
    /// Max 4 contact points are needed for a stable manifold
    pub const max_contact_points = 4;

    /// Maximum size a single constraint can take
    /// (Jolt: sizeof(ContactConstraint<Dynamic, Dynamic>) + 3 * sizeof(WorldContactPoint<Dynamic, Dynamic>), 504 bytes in
    /// both precisions; A4 derives it from its types with a comptime check against this value)
    pub const max_constraint_size: u32 = 504;

    /// The maximum value that can be passed to Init for inMaxContactConstraints. Note you should really use a lower value, using this value will cost a lot of memory!
    pub const max_contact_constraints_limit: u32 = std.math.maxInt(u32) / max_constraint_size;

    /// The maximum value that can be passed to Init for inMaxBodyPairs. Note you should really use a lower value, using this value will cost a lot of memory!
    /// (Jolt: ~uint(0) / sizeof(BodyPairMap::KeyValue), the key value is 40 bytes)
    pub const max_body_pairs_limit: u32 = std.math.maxInt(u32) / 40;

    /// Callback function to combine the restitution or friction of two bodies
    /// Note that when merging manifolds (when PhysicsSettings::mUseManifoldReduction is true) you will only get a callback for the merged manifold.
    /// It is not possible in that case to get all sub shape ID pairs that were colliding, you'll get the first encountered pair.
    pub const CombineFunction = *const fn (body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32;

    /// Contacts are allocated in a lock free hash map
    /// (A4: extends LFHMAllocatorContext, the stub only has the counters)
    pub const ContactAllocator = struct {
        /// Total number of body pairs added using this allocator
        num_body_pairs: u32 = 0,
        /// Total number of manifolds added using this allocator
        num_manifolds: u32 = 0,
        /// Errors reported on this allocator
        errors: PhysicsUpdateError = .none,
    };

    /// Handle used to keep track of the current body pair
    pub const BodyPairHandle = ?*anyopaque;

    /// The physics settings of the system (Jolt: const PhysicsSettings &)
    physics_settings: *const PhysicsSettings = undefined,

    /// Listener that is notified whenever a contact point between two bodies is added/updated/removed
    contact_listener: ?ContactListener = null,

    /// Functions that are used to combine friction and restitution of 2 bodies
    combine_friction: CombineFunction = defaultCombineFriction,
    combine_restitution: CombineFunction = defaultCombineRestitution,

    /// The constraints that were added this frame (A4: ContactConstraint<Type1, Type2> records of up to
    /// max_constraint_size bytes)
    constraints: TempArray(u8) = .empty,
    max_constraints: u32 = 0,
    num_constraints_and_next_constraint_offset: std.atomic.Value(u64) = .init(0),

    /// Mapping from constraint index to constraint offset
    constraint_idx_to_offset: TempArray(u32) = .empty,

    /// Context used for this physics update
    update_context: ?*PhysicsUpdateContext = null,

    /// Which cache we're currently writing to (A4: of `mCache[2]`, the stub's caches are always empty)
    cache_write_idx: u1 = 0,

    /// Not initialized, call init
    pub const empty: ContactConstraintManager = .{};

    /// Default friction combine function: the geometric mean, sqrt(friction1 * friction2)
    fn defaultCombineFriction(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        _ = sub_shape_id1;
        _ = sub_shape_id2;
        return math.sqrt(body1.getFriction() * body2.getFriction());
    }

    /// Default restitution combine function: max(restitution1, restitution1)
    fn defaultCombineRestitution(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        _ = sub_shape_id1;
        _ = sub_shape_id2;
        return math.max(body1.getRestitution(), body2.getRestitution());
    }

    /// Constructor + Initialize the system.
    /// @param max_body_pairs Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching
    /// @param max_contact_constraints Maximum amount of contact constraints to process (anything else will fall through the world)
    /// (Stub: the caches are not allocated yet, A4 allocates them with `allocator`)
    pub fn init(self: *ContactConstraintManager, allocator: Allocator, physics_settings: *const PhysicsSettings, max_body_pairs: u32, max_contact_constraints: u32) Allocator.Error!void {
        _ = allocator;
        _ = max_body_pairs;
        self.* = .{ .physics_settings = physics_settings };

        // Limit the number of constraints so that the allocation size fits in an unsigned integer
        self.max_constraints = @min(max_contact_constraints, max_contact_constraints_limit);
        std.debug.assert(self.max_constraints == max_contact_constraints); // Cannot support this many contact constraints!
        self.max_constraints = @max(self.max_constraints, 4); // Because our hash map requires at least 4 buckets, we need to have a minimum number of constraints
    }

    /// Destructor
    pub fn deinit(self: *ContactConstraintManager) void {
        std.debug.assert(self.constraints.isEmpty());
        std.debug.assert(self.constraint_idx_to_offset.isEmpty());
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

    /// Sets up the constraint buffer. Should be called before starting collision detection.
    pub fn prepareConstraintBuffer(self: *ContactConstraintManager, context: *PhysicsUpdateContext) TempAllocator.Error!void {
        // Store context
        self.update_context = context;

        // Allocate temporary constraint buffer
        std.debug.assert(self.constraints.isEmpty());
        self.constraints = try .alloc(context.temp_allocator, @as(usize, self.max_constraints) * max_constraint_size);
        std.debug.assert(self.constraint_idx_to_offset.isEmpty());
        self.constraint_idx_to_offset = try .alloc(context.temp_allocator, self.max_constraints);
    }

    /// Get a new allocator context for storing contacts. Note that you should call this once and then add multiple contacts using the context.
    pub fn getContactAllocator(self: *const ContactConstraintManager) ContactAllocator {
        _ = self;
        return .{};
    }

    /// Finalizes the contact cache, the contact cache that was generated during the calls to AddContactConstraint in this update
    /// will be used from now on to read from. After finalizing the contact cache, the contact removed callbacks will be called.
    /// inExpectedNumBodyPairs / inExpectedNumManifolds are the amount of body pairs / manifolds found in the previous step and is
    /// used to determine the amount of buckets the contact cache hash map will use in the next update.
    /// (Stub: both caches are empty, so this only swaps them)
    pub fn finalizeContactCacheAndCallContactPointRemovedCallbacks(self: *ContactConstraintManager, expected_num_body_pairs: u32, expected_num_manifolds: u32) void {
        // Check that the count of body pairs and manifolds that we tracked outside of the cache (to avoid contention on an atomic) is correct
        std.debug.assert(expected_num_body_pairs == 0);
        std.debug.assert(expected_num_manifolds == 0);

        // Buffers are now complete, make write buffer the read buffer
        self.cache_write_idx ^= 1;
    }

    /// Zolt addition (D10): a poisoned update left the write cache incomplete, discard it instead of swapping (no
    /// listener is called, the read cache stays). Stub: both caches are empty.
    pub fn finalizeContactCacheAfterOutOfMemory(self: *ContactConstraintManager) void {
        _ = self;
    }

    /// Check if 2 bodies were in contact during the last simulation step. Since contacts are only detected between active bodies, at least one of the bodies must be active.
    /// Uses the read collision cache to determine if 2 bodies are in contact.
    /// (Stub: the read cache is always empty)
    pub fn wereBodiesInContact(self: *const ContactConstraintManager, body1_id: BodyID, body2_id: BodyID) bool {
        _ = self;
        _ = body1_id;
        _ = body2_id;
        return false;
    }

    /// Get the number of contact constraints that were found
    pub fn getNumConstraints(self: *const ContactConstraintManager) u32 {
        return @min(@as(u32, @truncate(self.num_constraints_and_next_constraint_offset.load(.monotonic))), self.max_constraints);
    }

    /// Sort contact constraints deterministically (stub: there are no contact constraints)
    pub fn sortContacts(self: *const ContactConstraintManager, constraint_offsets: []u32) void {
        _ = self;
        if (constraint_offsets.len > 0) @panic("ContactConstraintManager.sortContacts: A4");
    }

    /// Solve position constraints (stub: there are no contact constraints)
    pub fn solvePositionConstraints(self: *ContactConstraintManager, constraint_offsets: []const u32) bool {
        _ = self;
        if (constraint_offsets.len > 0) @panic("ContactConstraintManager.solvePositionConstraints: A4");
        return false;
    }

    /// Recycle the constraint buffer. Should be called between collision simulation steps.
    pub fn recycleConstraintBuffer(self: *ContactConstraintManager) void {
        // Reset constraint array
        self.num_constraints_and_next_constraint_offset.store(0, .seq_cst);
    }

    /// Terminate the constraint buffer. Should be called after simulation ends.
    /// (Zolt: the TempArrays that were never allocated are skipped, also when PrepareConstraintBuffer did not run)
    pub fn finishConstraintBuffer(self: *ContactConstraintManager) void {
        // Free constraints buffer
        if (self.update_context) |context| {
            self.constraint_idx_to_offset.free(context.temp_allocator);
            self.constraints.free(context.temp_allocator);
        }
        self.num_constraints_and_next_constraint_offset.store(0, .seq_cst);

        // Reset update context
        self.update_context = null;
    }

    /// Saving state for replay
    /// (Stub: the read cache is empty: 0 body pairs, 0 CCD manifolds, Jolt's bytes for an empty cache)
    pub fn saveState(self: *const ContactConstraintManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        _ = self;
        _ = filter;

        // Write body pairs
        const num_body_pairs: u32 = 0;
        stream.write(num_body_pairs);

        // Write all CCD manifold keys
        const num_manifolds: u32 = 0;
        stream.write(num_manifolds);
    }

    /// Restoring state for replay. Returns false when failed.
    /// (Stub: only an empty cache can be restored, a stream with contacts panics)
    pub fn restoreState(self: *ContactConstraintManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) bool {
        _ = filter;

        // Read amount of body pairs (when validating: the size of the (empty) read cache)
        var num_body_pairs: u32 = 0;
        stream.read(&num_body_pairs);
        if (num_body_pairs != 0) @panic("ContactConstraintManager.restoreState: restoring contacts needs A4");

        // Read amount of CCD manifolds
        var num_manifolds: u32 = 0;
        stream.read(&num_manifolds);
        if (num_manifolds != 0) @panic("ContactConstraintManager.restoreState: restoring contacts needs A4");

        // If this is the last part, the cache is finalized
        if (stream.isLastPart())
            self.cache_write_idx ^= 1;

        return true;
    }
};
