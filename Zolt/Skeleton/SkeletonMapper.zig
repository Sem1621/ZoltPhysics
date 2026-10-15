//! Port of: Jolt/Skeleton/SkeletonMapper.h, Jolt/Skeleton/SkeletonMapper.cpp
//! Status: complete
//!
//! CharacterVehicleArchitecture E2: a RefTarget without virtual functions, like PhysicsScene. The mapper keeps the
//! allocator that owns it and its arrays (the chains own their joint index arrays): `create(allocator)` is `new
//! SkeletonMapper` (reference count 0, put it in a `Ref` / `RefConst`), `init(allocator)` / `deinit` a mapper on the stack
//! or embedded.
//! - `CanMapJoint` (`std::function<bool (const Skeleton *, int, const Skeleton *, int)>`) is a function pointer with a
//!   context (`CanMapJoint.init(&context, function)`); Jolt's default argument `sDefaultCanMapJoint` is
//!   `CanMapJoint.default`: `initialize(..., .{})` or `initialize(..., .{ .can_map_joint = c })`.
//! - The `const Mat44 *` / `Mat44 *` / `const bool *` arrays are slices with (at least) one element per joint.
//! - Joint indices: the fields of Mapping, Chain, Unmapped and Locked mirror Jolt's `int` members as `i32`, also where
//!   they are always valid joints (Mapping.joint_idx1 / joint_idx2, the chains), so they compare directly with
//!   `Skeleton.Joint.parent_joint_index` and with -1; so do getMappedJointIdx and isJointTranslationLocked. CanMapJoint
//!   gets the joint indices as `usize` (always valid joints, like `Skeleton.getJoint`).
//! - Initialize, LockTranslations and LockAllTranslations allocate and return `Allocator.Error`. They are all or
//!   nothing: when an allocation fails, the mapper is left as it was. LockAllTranslations' JPH_STACK_ALLOC (one bool per
//!   joint, unbounded) is allocated with the mapper's allocator.
//! - Asserts on the input that Jolt's release build ignores ("Should not be initialized yet", "Skeleton 1 should be the
//!   low detail skeleton!", ordered joints) are checked in safe builds only (`if (Core.enable_asserts)`), so that
//!   optimized builds compute what Jolt's release build computes (a second Initialize appends, like Jolt's).
//! - Where Jolt has undefined behavior Zolt panics:
//!   - LockAllTranslations before Initialize ("Call Initialize first!", Jolt reads mMappings[0] of an empty array) or
//!     when the first mapped joint is not a joint of `skeleton2` (Jolt writes outside its array);
//!   - Map with an empty chain (Jolt reads the first / last element of an empty array; chains of 1 or 2 joints, which
//!     a user can add through getChainsMut, are fine and handled like Jolt's signed loop);
//!   - Map with a locked joint that has no parent. LockTranslations stores the model space translation of a locked
//!     joint without a parent, but Map reads `outPose2ModelSpace[-1]` for it (a Jolt bug), so such a lock can only be
//!     used without Map.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const RefCount = @import("../Core/Reference.zig").RefCount;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Quat = @import("../Math/Quat.zig").Quat;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Skeleton = @import("Skeleton.zig").Skeleton;

/// Class that is able to map a low detail (ragdoll) skeleton to a high detail (animation) skeleton and vice versa
pub const SkeletonMapper = struct {
    /// A joint that maps 1-on-1 to a joint in the other skeleton
    pub const Mapping = struct {
        /// Index of joint from skeleton 1
        joint_idx1: i32,
        /// Corresponding index of joint from skeleton 2
        joint_idx2: i32,
        /// Transforms this joint from skeleton 1 to 2
        joint1_to2: Mat44,
        /// Inverse of the transform above
        joint2_to1: Mat44,

        /// Constructor Mapping(inJointIdx1, inJointIdx2, inJoint1To2)
        pub fn init(joint_idx1: i32, joint_idx2: i32, joint1_to2: Mat44) Mapping {
            var mapping: Mapping = .{ .joint_idx1 = joint_idx1, .joint_idx2 = joint_idx2, .joint1_to2 = joint1_to2, .joint2_to1 = joint1_to2.inversed() };

            // Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
            mapping.joint2_to1.set(3, 3, 1.0);
            return mapping;
        }
    };

    /// A joint chain that starts with a 1-on-1 mapped joint and ends with a 1-on-1 mapped joint with intermediate joints that cannot be mapped
    pub const Chain = struct {
        /// Joint chain from skeleton 1 (allocated with the mapper's allocator)
        joint_indices1: std.ArrayList(i32) = .empty,
        /// Corresponding joint chain from skeleton 2 (allocated with the mapper's allocator)
        joint_indices2: std.ArrayList(i32) = .empty,

        /// Constructor Chain(Array<int> &&, Array<int> &&): takes ownership of the arrays
        pub fn init(joint_indices1: std.ArrayList(i32), joint_indices2: std.ArrayList(i32)) Chain {
            return .{ .joint_indices1 = joint_indices1, .joint_indices2 = joint_indices2 };
        }

        /// Destructor: frees the arrays
        pub fn deinit(self: *Chain, allocator: Allocator) void {
            self.joint_indices1.deinit(allocator);
            self.joint_indices2.deinit(allocator);
        }
    };

    /// Joints that could not be mapped from skeleton 1 to 2
    pub const Unmapped = struct {
        /// Joint index of unmappable joint
        joint_idx: i32,
        /// Parent joint index of unmappable joint
        parent_joint_idx: i32,
    };

    /// Joints that should have their translation locked (fixed)
    pub const Locked = struct {
        /// Joint index of joint with locked translation (in skeleton 2)
        joint_idx: i32,
        /// Parent joint index of joint with locked translation (in skeleton 2)
        parent_joint_idx: i32,
        /// Translation of neutral pose
        translation: Vec3,
    };

    /// A function that is called to determine if a joint can be mapped from source to target skeleton
    /// (std::function<bool (const Skeleton *, int, const Skeleton *, int)>: a function and its context)
    pub const CanMapJoint = struct {
        /// The context that is passed to `function`
        context: ?*anyopaque = null,
        /// The function
        function: *const fn (context: ?*anyopaque, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool,

        /// sDefaultCanMapJoint
        pub const default: CanMapJoint = .{ .function = defaultCanMapJointThunk };

        /// `function(context, skeleton1, index1, skeleton2, index2)`, `context` is a pointer (it may be const)
        pub fn init(context: anytype, comptime function: fn (@TypeOf(context), *const Skeleton, usize, *const Skeleton, usize) bool) CanMapJoint {
            const Context = @TypeOf(context);
            const gen = struct {
                fn call(ptr: ?*anyopaque, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
                    const typed: Context = @ptrCast(@alignCast(ptr.?));
                    return function(typed, skeleton1, index1, skeleton2, index2);
                }
            };
            return .{ .context = @ptrCast(@constCast(context)), .function = gen.call };
        }

        /// Call the function
        pub fn call(self: CanMapJoint, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
            return self.function(self.context, skeleton1, index1, skeleton2, index2);
        }

        fn defaultCanMapJointThunk(_: ?*anyopaque, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
            return defaultCanMapJoint(skeleton1, index1, skeleton2, index2);
        }
    };

    pub const MappingVector = std.ArrayList(Mapping);
    pub const ChainVector = std.ArrayList(Chain);
    pub const UnmappedVector = std.ArrayList(Unmapped);
    pub const LockedVector = std.ArrayList(Locked);

    /// RefTarget<SkeletonMapper>
    ref_count: RefCount = .{},
    /// Allocates the arrays, frees the mapper
    allocator: Allocator,

    /// Joint mappings (private in Jolt, use getMappings / getMappingsMut)
    mappings: MappingVector = .empty,
    /// (private in Jolt, use getChains / getChainsMut)
    chains: ChainVector = .empty,
    /// Joint indices that could not be mapped from 1 to 2 (these are indices in 2) (private in Jolt, use getUnmapped /
    /// getUnmappedMut)
    unmapped: UnmappedVector = .empty,
    /// (private in Jolt, use getLockedTranslations / getLockedTranslationsMut)
    locked_translations: LockedVector = .empty,

    /// Constructor of a mapper that is not on the heap (`deinit` it)
    pub fn init(allocator: Allocator) SkeletonMapper {
        return .{ .allocator = allocator };
    }

    /// new SkeletonMapper: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*SkeletonMapper {
        const self = try allocator.create(SkeletonMapper);
        self.* = .init(allocator);
        return self;
    }

    /// Destructor: frees the arrays
    pub fn deinit(self: *SkeletonMapper) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *SkeletonMapper) void {
        self.mappings.deinit(self.allocator);
        for (self.chains.items) |*c| c.deinit(self.allocator);
        self.chains.deinit(self.allocator);
        self.unmapped.deinit(self.allocator);
        self.locked_translations.deinit(self.allocator);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const SkeletonMapper) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const SkeletonMapper) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this mapper as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const SkeletonMapper) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const SkeletonMapper) u32 {
        return self.ref_count.get();
    }

    /// Default function that checks if the names of the joints are equal
    pub fn defaultCanMapJoint(skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
        return std.mem.eql(u8, skeleton1.getJoint(index1).name, skeleton2.getJoint(index2).name);
    }

    /// Initialize the skeleton mapper. Skeleton 1 should be the (low detail) ragdoll skeleton and skeleton 2 the (high detail) animation skeleton.
    /// We assume that each joint in skeleton 1 can be mapped to a joint in skeleton 2 (if not mapping from animation skeleton to ragdoll skeleton will be undefined).
    /// Skeleton 2 should have the same hierarchy as skeleton 1 but can contain extra joints between those in skeleton 1 and it can have extra joints at the root and leaves of the skeleton.
    /// @param skeleton1 Source skeleton to map from.
    /// @param neutral_pose1 Neutral pose of the source skeleton (model space)
    /// @param skeleton2 Target skeleton to map to.
    /// @param neutral_pose2 Neutral pose of the target skeleton (model space), inNeutralPose1 and inNeutralPose2 must match as closely as possible, preferably the position of the mappable joints should be identical.
    /// @param opts.can_map_joint Function that checks if joints in skeleton 1 and skeleton 2 are equal.
    pub fn initialize(self: *SkeletonMapper, skeleton1: *const Skeleton, neutral_pose1: []const Mat44, skeleton2: *const Skeleton, neutral_pose2: []const Mat44, opts: struct { can_map_joint: CanMapJoint = .default }) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(self.mappings.items.len == 0 and self.chains.items.len == 0 and self.unmapped.items.len == 0); // Should not be initialized yet

        const allocator = self.allocator;

        // Zolt: all or nothing, remove what was added when an allocation fails (Jolt's release build appends to the arrays
        // when they are not empty, so remember where they ended)
        const old_num_mappings = self.mappings.items.len;
        const old_num_chains = self.chains.items.len;
        const old_num_unmapped = self.unmapped.items.len;
        errdefer {
            self.mappings.shrinkRetainingCapacity(old_num_mappings);
            for (self.chains.items[old_num_chains..]) |*c| c.deinit(allocator);
            self.chains.shrinkRetainingCapacity(old_num_chains);
            self.unmapped.shrinkRetainingCapacity(old_num_unmapped);
        }

        // Count joints
        const n1 = skeleton1.getJointCount();
        const n2 = skeleton2.getJointCount();
        if (Core.enable_asserts) std.debug.assert(n1 <= n2); // Skeleton 1 should be the low detail skeleton!

        // Keep track of mapped joints (initialize to false)
        const mapped1 = try allocator.alloc(bool, n1);
        defer allocator.free(mapped1);
        @memset(mapped1, false);
        const mapped2 = try allocator.alloc(bool, n2);
        defer allocator.free(mapped2);
        @memset(mapped2, false);

        // Find joints that can be mapped directly
        for (0..n1) |j1| {
            for (0..n2) |j2| {
                if (opts.can_map_joint.call(skeleton1, j1, skeleton2, j2)) {
                    // Calculate the transform that takes this joint from skeleton 1 to 2
                    var joint_1_to_2 = neutral_pose1[j1].inversed().mul(neutral_pose2[j2]);

                    // Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
                    joint_1_to_2.set(3, 3, 1.0);

                    try self.mappings.append(allocator, .init(@intCast(j1), @intCast(j2), joint_1_to_2));
                    mapped1[j1] = true;
                    mapped2[j2] = true;
                    break;
                }
            }
        }

        var cur_chain: std.ArrayList(i32) = .empty; // Taken out of the loop to minimize amount of allocations
        defer cur_chain.deinit(allocator);

        // Find joint chains
        for (0..self.mappings.items.len) |m1| {
            var chain2: std.ArrayList(i32) = .empty;
            defer chain2.deinit(allocator);
            var chain2_m: usize = undefined;

            for (m1 + 1..self.mappings.items.len) |m2| {
                // Find the chain from back from m2 to m1
                const start = self.mappings.items[m1].joint_idx2;
                const end = self.mappings.items[m2].joint_idx2;
                var cur = end;
                cur_chain.clearRetainingCapacity(); // Should preserve memory
                while (true) {
                    try cur_chain.append(allocator, cur);
                    cur = skeleton2.getJoint(@intCast(cur)).parent_joint_index;
                    if (!(cur >= 0 and cur != start and !mapped2[@intCast(cur)]))
                        break;
                }
                try cur_chain.append(allocator, start);

                if (cur == start // This should be the correct chain
                and cur_chain.items.len > 2 // It should have joints between the mapped joints
                and cur_chain.items.len > chain2.items.len) // And it should be the longest so far
                {
                    std.mem.swap(std.ArrayList(i32), &chain2, &cur_chain);
                    chain2_m = m2;
                }
            }

            if (chain2.items.len != 0) {
                // Get the chain for 1
                var chain1: std.ArrayList(i32) = .empty;
                defer chain1.deinit(allocator);
                const start = self.mappings.items[m1].joint_idx1;
                var cur = self.mappings.items[chain2_m].joint_idx1;
                while (true) {
                    try chain1.append(allocator, cur);
                    cur = skeleton1.getJoint(@intCast(cur)).parent_joint_index;
                    if (!(cur >= 0 and cur != start and !mapped1[@intCast(cur)]))
                        break;
                }
                try chain1.append(allocator, start);

                // If the chain exists in 1 too
                if (cur == start) {
                    // Zolt: reserve the chain before anything changes
                    try self.chains.ensureUnusedCapacity(allocator, 1);

                    // Reverse the chains
                    std.mem.reverse(i32, chain1.items);
                    std.mem.reverse(i32, chain2.items);

                    // Mark elements mapped
                    for (chain1.items) |j1|
                        mapped1[@intCast(j1)] = true;
                    for (chain2.items) |j2|
                        mapped2[@intCast(j2)] = true;

                    // Insert the chain (it takes ownership of the arrays)
                    self.chains.appendAssumeCapacity(.init(chain1, chain2));
                    chain1 = .empty;
                    chain2 = .empty;
                }
            }
        }

        // Collect unmapped joints from 2
        for (0..n2) |j2|
            if (!mapped2[j2])
                try self.unmapped.append(allocator, .{ .joint_idx = @intCast(j2), .parent_joint_idx = skeleton2.getJoint(j2).parent_joint_index });
    }

    /// This can be called so lock the translation of a specified set of joints in skeleton 2.
    /// Because constraints are never 100% rigid, there's always a little bit of stretch in the ragdoll when the ragdoll is under stress.
    /// Locking the translations of the pose will remove the visual stretch from the ragdoll but will introduce a difference between the
    /// physical simulation and the visual representation.
    /// @param skeleton2 Target skeleton to map to.
    /// @param locked_translations An array of bools the size of inSkeleton2->GetJointCount(), for each joint indicating if the joint is locked.
    /// @param neutral_pose2 Neutral pose to take reference translations from
    pub fn lockTranslations(self: *SkeletonMapper, skeleton2: *const Skeleton, locked_translations: []const bool, neutral_pose2: []const Mat44) Allocator.Error!void {
        if (Core.enable_asserts) std.debug.assert(skeleton2.areJointsCorrectlyOrdered());

        const n = skeleton2.getJointCount();

        // Zolt: reserve first, so that nothing is added when the allocation fails
        var num_locked: usize = 0;
        for (locked_translations[0..n]) |locked| {
            if (locked) num_locked += 1;
        }
        try self.locked_translations.ensureUnusedCapacity(self.allocator, num_locked);

        // Copy locked joints to array but don't actually include the first joint (this is physics driven)
        for (0..n) |i| {
            if (locked_translations[i]) {
                var l: Locked = undefined;
                l.joint_idx = @intCast(i);
                l.parent_joint_idx = skeleton2.getJoint(i).parent_joint_index;
                if (l.parent_joint_idx >= 0)
                    l.translation = neutral_pose2[@intCast(l.parent_joint_idx)].inversed().mulVec3(neutral_pose2[i].getTranslation())
                else
                    l.translation = neutral_pose2[i].getTranslation();
                self.locked_translations.appendAssumeCapacity(l);
            }
        }
    }

    /// After Initialize(), this can be called to lock the translation of all joints in skeleton 2 below the first mapped joint to those of the neutral pose.
    /// Because constraints are never 100% rigid, there's always a little bit of stretch in the ragdoll when the ragdoll is under stress.
    /// Locking the translations of the pose will remove the visual stretch from the ragdoll but will introduce a difference between the
    /// physical simulation and the visual representation.
    /// @param skeleton2 Target skeleton to map to.
    /// @param neutral_pose2 Neutral pose to take reference translations from
    pub fn lockAllTranslations(self: *SkeletonMapper, skeleton2: *const Skeleton, neutral_pose2: []const Mat44) Allocator.Error!void {
        if (self.mappings.items.len == 0)
            @panic("Call Initialize first!"); // Jolt asserts, its release build reads mMappings[0] of an empty array
        if (Core.enable_asserts) std.debug.assert(skeleton2.areJointsCorrectlyOrdered());

        // The first mapping is the top most one (remember that joints should be ordered so that parents go before children).
        // Because we created the mappings from the lowest joint first, this should contain the first mappable joint.
        const root = self.mappings.items[0].joint_idx2;

        // Create temp array to hold locked joints (Zolt: allocated with the mapper's allocator instead of on the stack)
        const n = skeleton2.getJointCount();
        if (root < 0 or root >= n)
            @panic("SkeletonMapper::LockAllTranslations: the first mapped joint is not a joint of skeleton 2 (Jolt writes outside its array)");
        const root_idx: usize = @intCast(root);
        const locked_translations = try self.allocator.alloc(bool, n);
        defer self.allocator.free(locked_translations);
        @memset(locked_translations, false);

        // Mark root as locked
        locked_translations[root_idx] = true;

        // Loop over all joints and propagate the locked flag to all children
        for (root_idx + 1..n) |i| {
            const parent_idx = skeleton2.getJoint(i).parent_joint_index;
            if (parent_idx >= 0)
                locked_translations[i] = locked_translations[@intCast(parent_idx)];
        }

        // Unmark root because we don't actually want to include this (this determines the position of the entire ragdoll)
        locked_translations[root_idx] = false;

        // Call the generic function
        try self.lockTranslations(skeleton2, locked_translations, neutral_pose2);
    }

    /// Map a pose. Joints that were directly mappable will be copied in model space from pose 1 to pose 2. Any joints that are only present in skeleton 2
    /// will get their model space transform calculated through the local space transforms of pose 2. Joints that are part of a joint chain between two
    /// mapped joints will be reoriented towards the next joint in skeleton 1. This means that it is possible for unmapped joints to have some animation,
    /// but very extreme animation poses will show artifacts.
    /// @param pose1_model_space Pose on skeleton 1 in model space
    /// @param pose2_local_space Pose on skeleton 2 in local space (used for the joints that cannot be mapped)
    /// @param out_pose2_model_space Model space pose on skeleton 2 (the output of the mapping)
    pub fn map(self: *const SkeletonMapper, pose1_model_space: []const Mat44, pose2_local_space: []const Mat44, out_pose2_model_space: []Mat44) void {
        // Apply direct mappings
        for (self.mappings.items) |*m|
            out_pose2_model_space[@intCast(m.joint_idx2)] = pose1_model_space[@intCast(m.joint_idx1)].mul(m.joint1_to2);

        // Apply chain mappings
        for (self.chains.items) |*c| {
            const indices1 = c.joint_indices1.items;
            const indices2 = c.joint_indices2.items;
            if (indices1.len == 0 or indices2.len == 0)
                @panic("SkeletonMapper::Map: an empty chain (Jolt reads the first / last element of an empty array)");

            // Calculate end of chain given local space transforms of the joints of the chain
            const chain_start = &out_pose2_model_space[@intCast(indices2[0])];
            var chain_end = chain_start.*;
            for (indices2[1..]) |j|
                chain_end = chain_end.mul(pose2_local_space[@intCast(j)]);

            // Calculate the direction in world space for skeleton 1 and skeleton 2 and the rotation between them
            const actual = chain_end.getTranslation().sub(chain_start.getTranslation());
            const desired = pose1_model_space[@intCast(indices1[indices1.len - 1])].getTranslation().sub(pose1_model_space[@intCast(indices1[0])].getTranslation());
            const rotation = Quat.fromTo(actual, desired);

            // Rotate the start of the chain
            chain_start.setRotation(Mat44.rotationQuat(rotation).mul(chain_start.getRotation()));

            // Update all joints but the first and the last joint using their local space transforms (Jolt's signed
            // `j < size - 1`: no joints for a chain of 1 or 2 joints, which a user can add through getChainsMut)
            var j: usize = 1;
            while (j + 1 < indices2.len) : (j += 1) {
                const parent = indices2[j - 1];
                const child = indices2[j];
                out_pose2_model_space[@intCast(child)] = out_pose2_model_space[@intCast(parent)].mul(pose2_local_space[@intCast(child)]);
            }
        }

        // All unmapped joints take the local pose and convert it to model space
        for (self.unmapped.items) |*u| {
            if (u.parent_joint_idx >= 0) {
                if (Core.enable_asserts) std.debug.assert(u.parent_joint_idx < u.joint_idx); // Joints must be ordered: parents first
                out_pose2_model_space[@intCast(u.joint_idx)] = out_pose2_model_space[@intCast(u.parent_joint_idx)].mul(pose2_local_space[@intCast(u.joint_idx)]);
            } else out_pose2_model_space[@intCast(u.joint_idx)] = pose2_local_space[@intCast(u.joint_idx)];
        }

        // Update all locked joint translations
        for (self.locked_translations.items) |*l| {
            if (l.parent_joint_idx < 0)
                @panic("SkeletonMapper::Map: a locked joint without a parent (Jolt reads outPose2ModelSpace[-1])");
            out_pose2_model_space[@intCast(l.joint_idx)].setTranslation(out_pose2_model_space[@intCast(l.parent_joint_idx)].mulVec3(l.translation));
        }
    }

    /// Reverse map a pose, this will only use the mappings and not the chains (it assumes that all joints in skeleton 1 are mapped)
    /// @param pose2_model_space Model space pose on skeleton 2
    /// @param out_pose1_model_space When the function returns this will contain the model space pose for skeleton 1
    pub fn mapReverse(self: *const SkeletonMapper, pose2_model_space: []const Mat44, out_pose1_model_space: []Mat44) void {
        // Normally each joint in skeleton 1 should be present in the mapping, so we only need to apply the direct mappings
        for (self.mappings.items) |*m|
            out_pose1_model_space[@intCast(m.joint_idx1)] = pose2_model_space[@intCast(m.joint_idx2)].mul(m.joint2_to1);
    }

    /// Search through the directly mapped joints (mMappings) and find inJoint1Idx, returns the corresponding Joint2Idx or -1 if not found.
    pub fn getMappedJointIdx(self: *const SkeletonMapper, joint1_idx: i32) i32 {
        for (self.mappings.items) |*m|
            if (m.joint_idx1 == joint1_idx)
                return m.joint_idx2;

        return -1;
    }

    /// Search through the locked translations (mLockedTranslations) and find if joint inJoint2Idx is locked.
    pub fn isJointTranslationLocked(self: *const SkeletonMapper, joint2_idx: i32) bool {
        for (self.locked_translations.items) |*l|
            if (l.joint_idx == joint2_idx)
                return true;

        return false;
    }

    // Access to the mapped joints

    /// The direct mappings (GetMappings() const)
    pub fn getMappings(self: *const SkeletonMapper) []const Mapping {
        return self.mappings.items;
    }

    /// The direct mappings (the non-const GetMappings: the array itself, allocated with `allocator`)
    pub fn getMappingsMut(self: *SkeletonMapper) *MappingVector {
        return &self.mappings;
    }

    /// The chains (GetChains() const)
    pub fn getChains(self: *const SkeletonMapper) []const Chain {
        return self.chains.items;
    }

    /// The chains (the non-const GetChains: the array itself, it and the arrays of its chains are allocated with
    /// `allocator`)
    pub fn getChainsMut(self: *SkeletonMapper) *ChainVector {
        return &self.chains;
    }

    /// The unmapped joints (GetUnmapped() const)
    pub fn getUnmapped(self: *const SkeletonMapper) []const Unmapped {
        return self.unmapped.items;
    }

    /// The unmapped joints (the non-const GetUnmapped: the array itself, allocated with `allocator`)
    pub fn getUnmappedMut(self: *SkeletonMapper) *UnmappedVector {
        return &self.unmapped;
    }

    /// The joints with locked translations (GetLockedTranslations() const)
    pub fn getLockedTranslations(self: *const SkeletonMapper) []const Locked {
        return self.locked_translations.items;
    }

    /// The joints with locked translations (the non-const GetLockedTranslations: the array itself, allocated with
    /// `allocator`)
    pub fn getLockedTranslationsMut(self: *SkeletonMapper) *LockedVector {
        return &self.locked_translations;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;
const Ref = @import("../Core/Reference.zig").Ref;
const SkeletonPose = @import("SkeletonPose.zig").SkeletonPose;

/// The ragdoll skeleton: Pelvis - Spine - Head, Spine - Arm (4 joints)
fn buildRagdollSkeleton(skeleton: *Skeleton) !void {
    _ = try skeleton.addJoint("Pelvis", .{});
    _ = try skeleton.addJointWithParentIndex("Spine", 0);
    _ = try skeleton.addJointWithParentIndex("Head", 1);
    _ = try skeleton.addJointWithParentIndex("Arm", 1);
}

/// The animation skeleton: an extra root, Spine1 / Spine2 between Pelvis and Spine (a chain), Neck between Spine and
/// Head (a chain), Hand below Arm (a leaf), Tail below Pelvis (10 joints)
fn buildAnimationSkeleton(skeleton: *Skeleton) !void {
    _ = try skeleton.addJoint("Root", .{}); // 0
    _ = try skeleton.addJointWithParentIndex("Pelvis", 0); // 1
    _ = try skeleton.addJointWithParentIndex("Spine1", 1); // 2
    _ = try skeleton.addJointWithParentIndex("Spine2", 2); // 3
    _ = try skeleton.addJointWithParentIndex("Spine", 3); // 4
    _ = try skeleton.addJointWithParentIndex("Neck", 4); // 5
    _ = try skeleton.addJointWithParentIndex("Head", 5); // 6
    _ = try skeleton.addJointWithParentIndex("Arm", 4); // 7
    _ = try skeleton.addJointWithParentIndex("Hand", 7); // 8
    _ = try skeleton.addJointWithParentIndex("Tail", 1); // 9
}

/// Local space translations of the animation skeleton
const animation_offsets = [_]Vec3{
    Vec3.init(0, 0, 0),   Vec3.init(0, 1, 0),    Vec3.init(0, 0.2, 0),   Vec3.init(0, 0.2, 0.05), Vec3.init(0, 0.2, 0),
    Vec3.init(0, 0.1, 0), Vec3.init(0, 0.15, 0), Vec3.init(0.2, 0.1, 0), Vec3.init(0.3, 0, 0),    Vec3.init(0, -0.1, -0.2),
};

/// Model space neutral poses where the shared joints are at the same place
const NeutralPoses = struct {
    pose1: [4]Mat44,
    pose2: [10]Mat44,
    local2: [10]Mat44,
};

fn neutralPoses(skeleton1: *const Skeleton, skeleton2: *const Skeleton) NeutralPoses {
    var poses: NeutralPoses = undefined;
    for (&poses.local2, animation_offsets) |*m, offset|
        m.* = Mat44.translation(offset);
    for (&poses.pose2, 0..) |*m, i| {
        const parent = skeleton2.getJoint(i).parent_joint_index;
        m.* = if (parent >= 0) poses.pose2[@intCast(parent)].mul(poses.local2[i]) else poses.local2[i];
    }
    for (&poses.pose1, 0..) |*m, i|
        m.* = poses.pose2[@intCast(skeleton2.getJointIndex(skeleton1.getJoint(i).name))];
    return poses;
}

const TestSkeletons = struct {
    skeleton1: Skeleton,
    skeleton2: Skeleton,
    poses: NeutralPoses,

    fn init(self: *TestSkeletons, allocator: Allocator) !void {
        self.skeleton1 = .init(allocator);
        self.skeleton1.setEmbedded(); // A pose may reference it
        errdefer self.skeleton1.deinit();
        self.skeleton2 = .init(allocator);
        self.skeleton2.setEmbedded();
        errdefer self.skeleton2.deinit();
        try buildRagdollSkeleton(&self.skeleton1);
        try buildAnimationSkeleton(&self.skeleton2);
        self.poses = neutralPoses(&self.skeleton1, &self.skeleton2);
    }

    fn deinit(self: *TestSkeletons) void {
        self.skeleton2.deinit();
        self.skeleton1.deinit();
    }
};

test "SkeletonMapper: initialize finds mappings, chains and unmapped joints" {
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();

    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});

    // Every ragdoll joint maps to the joint with the same name, at the same place (identity transforms)
    const mappings = mapper.getMappings();
    try testing.expectEqual(@as(usize, 4), mappings.len);
    const expected2 = [_]i32{ 1, 4, 6, 7 };
    for (mappings, 0..) |*m, i| {
        try testing.expectEqual(@as(i32, @intCast(i)), m.joint_idx1);
        try testing.expectEqual(expected2[i], m.joint_idx2);
        try testing.expect(m.joint1_to2.eql(Mat44.identity()));
        try testing.expect(m.joint2_to1.eql(Mat44.identity()));
        try testing.expectEqual(expected2[i], mapper.getMappedJointIdx(@intCast(i)));
    }
    try testing.expectEqual(@as(i32, -1), mapper.getMappedJointIdx(4));
    try testing.expectEqual(@as(i32, -1), mapper.getMappedJointIdx(-1));

    // The chains Pelvis - Spine1 - Spine2 - Spine and Spine - Neck - Head
    const chains = mapper.getChains();
    try testing.expectEqual(@as(usize, 2), chains.len);
    try testing.expectEqualSlices(i32, &.{ 0, 1 }, chains[0].joint_indices1.items);
    try testing.expectEqualSlices(i32, &.{ 1, 2, 3, 4 }, chains[0].joint_indices2.items);
    try testing.expectEqualSlices(i32, &.{ 1, 2 }, chains[1].joint_indices1.items);
    try testing.expectEqualSlices(i32, &.{ 4, 5, 6 }, chains[1].joint_indices2.items);

    // The extra root, the hand and the tail are unmapped
    const unmapped = mapper.getUnmapped();
    try testing.expectEqual(@as(usize, 3), unmapped.len);
    try testing.expectEqual(SkeletonMapper.Unmapped{ .joint_idx = 0, .parent_joint_idx = -1 }, unmapped[0]);
    try testing.expectEqual(SkeletonMapper.Unmapped{ .joint_idx = 8, .parent_joint_idx = 7 }, unmapped[1]);
    try testing.expectEqual(SkeletonMapper.Unmapped{ .joint_idx = 9, .parent_joint_idx = 1 }, unmapped[2]);
    try testing.expectEqual(@as(usize, 0), mapper.getLockedTranslations().len);

    // Mutable access
    try testing.expectEqual(@as(usize, 4), mapper.getMappingsMut().items.len);
    try testing.expectEqual(@as(usize, 2), mapper.getChainsMut().items.len);
    try testing.expectEqual(@as(usize, 3), mapper.getUnmappedMut().items.len);
    try testing.expectEqual(@as(usize, 0), mapper.getLockedTranslationsMut().items.len);
}

test "SkeletonMapper: the transforms of the mappings" {
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();

    // The ragdoll's head is rotated: joint1_to2 takes it back, joint2_to1 is its inverse with a bottom right of 1
    s.poses.pose1[2] = s.poses.pose1[2].mul(Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.3), Vec3.init(0, 0.01, 0)));
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});
    const m = mapper.getMappings()[2];
    var expected = s.poses.pose1[2].inversed().mul(s.poses.pose2[6]);
    expected.set(3, 3, 1.0);
    try testing.expect(m.joint1_to2.eql(expected));
    var expected_inverse = expected.inversed();
    expected_inverse.set(3, 3, 1.0);
    try testing.expect(m.joint2_to1.eql(expected_inverse));
    try testing.expect(s.poses.pose1[2].mul(m.joint1_to2).isClose(s.poses.pose2[6], .{ .max_dist_sq = 1.0e-10 }));

    // Mapping.init does the same
    const direct: SkeletonMapper.Mapping = .init(2, 6, expected);
    try testing.expect(direct.joint2_to1.eql(m.joint2_to1));
}

test "SkeletonMapper: map and mapReverse" {
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper = Ref(SkeletonMapper).init(try SkeletonMapper.create(allocator));
    defer mapper.deinit();
    const mp = mapper.get().?;
    try mp.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});

    // The neutral pose maps to the neutral pose
    var out2: [10]Mat44 = undefined;
    mp.map(&s.poses.pose1, &s.poses.local2, &out2);
    for (out2, s.poses.pose2) |a, b|
        try testing.expect(a.isClose(b, .{ .max_dist_sq = 1.0e-10 }));
    var out1: [4]Mat44 = undefined;
    mp.mapReverse(&s.poses.pose2, &out1);
    for (out1, s.poses.pose1) |a, b|
        try testing.expect(a.eql(b));

    // Moving the ragdoll's spine (and everything below it) rotates the start of the spine chain (the pelvis) so that the
    // end of the chain points at the ragdoll's spine, the joints below the spine follow the mapping, the tail follows the
    // rotated pelvis
    var pose1 = s.poses.pose1;
    const offset = Vec3.init(0.3, 0, 0);
    for (pose1[1..]) |*m| m.setTranslation(m.getTranslation().add(offset));
    mp.map(&pose1, &s.poses.local2, &out2);
    try testing.expect(out2[1].getTranslation().eql(pose1[0].getTranslation()));
    try testing.expect(!out2[1].getRotation().eql(pose1[0].getRotation()));
    try testing.expect(out2[4].eql(pose1[1])); // The start of the neck chain, which already points at the head
    const desired = pose1[1].getTranslation().sub(pose1[0].getTranslation()).normalized();
    const neutral_direction = s.poses.pose2[4].getTranslation().sub(s.poses.pose2[1].getTranslation()).normalized();
    try testing.expect(!neutral_direction.isClose(desired, .{ .max_dist_sq = 1.0e-6 }));
    const rotated_end = out2[1].mul(s.poses.local2[2]).mul(s.poses.local2[3]).mul(s.poses.local2[4]).getTranslation();
    try testing.expect(rotated_end.sub(out2[1].getTranslation()).normalized().isClose(desired, .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(out2[2].eql(out2[1].mul(s.poses.local2[2])));
    try testing.expect(out2[3].eql(out2[2].mul(s.poses.local2[3])));
    try testing.expect(out2[8].eql(out2[7].mul(s.poses.local2[8])));
    try testing.expect(out2[9].eql(out2[1].mul(s.poses.local2[9])));
    try testing.expect(out2[0].eql(s.poses.local2[0]));

    // Lock all translations below the first mapped joint (the pelvis): they come from the neutral pose relative to the
    // parent, the pelvis itself and the extra root are not locked
    try mp.lockAllTranslations(&s.skeleton2, &s.poses.pose2);
    const locked = mp.getLockedTranslations();
    try testing.expectEqual(@as(usize, 8), locked.len);
    for (locked, 2..) |*l, i| {
        try testing.expectEqual(@as(i32, @intCast(i)), l.joint_idx);
        try testing.expectEqual(s.skeleton2.getJoint(i).parent_joint_index, l.parent_joint_idx);
        try testing.expect(l.translation.isClose(animation_offsets[i], .{}));
    }
    try testing.expect(!mp.isJointTranslationLocked(0));
    try testing.expect(!mp.isJointTranslationLocked(1));
    try testing.expect(mp.isJointTranslationLocked(4));
    try testing.expect(mp.isJointTranslationLocked(9));
    try testing.expect(!mp.isJointTranslationLocked(10));
    mp.map(&pose1, &s.poses.local2, &out2);
    try testing.expect(out2[4].getTranslation().eql(out2[3].mulVec3(locked[2].translation))); // The spine is pulled back to the chain
    try testing.expect(out2[4].getRotation().eql(pose1[1].getRotation())); // But keeps its rotation
}

fn countingCanMapJoint(count: *u32, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
    count.* += 1;
    // Only the pelvis and the head, by name
    return index1 != 1 and index1 != 3 and std.mem.eql(u8, skeleton1.getJoint(index1).name, skeleton2.getJoint(index2).name);
}

test "SkeletonMapper: initialize with a can map joint function, unmapped ragdoll joints" {
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    try testing.expect(SkeletonMapper.defaultCanMapJoint(&s.skeleton1, 2, &s.skeleton2, 6));
    try testing.expect(!SkeletonMapper.defaultCanMapJoint(&s.skeleton1, 2, &s.skeleton2, 5));
    try testing.expect(SkeletonMapper.CanMapJoint.default.call(&s.skeleton1, 0, &s.skeleton2, 1));

    var count: u32 = 0;
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{ .can_map_joint = .init(&count, countingCanMapJoint) });
    try testing.expectEqual(@as(u32, 2 + 7 + 10 + 10), count); // Each ragdoll joint until the first match

    // Pelvis and head are mapped. Spine1 ... Neck lie between them in skeleton 2, and in skeleton 1 the chain from the
    // head goes up through the spine (which is not mapped) to the pelvis: one chain that includes the ragdoll's spine
    try testing.expectEqual(@as(usize, 2), mapper.getMappings().len);
    try testing.expectEqual(@as(usize, 1), mapper.getChains().len);
    try testing.expectEqualSlices(i32, &.{ 0, 1, 2 }, mapper.getChains()[0].joint_indices1.items);
    try testing.expectEqualSlices(i32, &.{ 1, 2, 3, 4, 5, 6 }, mapper.getChains()[0].joint_indices2.items);
    try testing.expectEqual(@as(usize, 4), mapper.getUnmapped().len); // Root, Arm, Hand, Tail
    try testing.expectEqual(@as(i32, 7), mapper.getUnmapped()[1].joint_idx);
    try testing.expectEqual(@as(i32, -1), mapper.getMappedJointIdx(1));
    try testing.expectEqual(@as(i32, 6), mapper.getMappedJointIdx(2));

    // mapReverse leaves the ragdoll joints that are not mapped alone
    var out1 = [_]Mat44{Mat44.zero()} ** 4;
    mapper.mapReverse(&s.poses.pose2, &out1);
    try testing.expect(out1[1].eql(Mat44.zero()));
    try testing.expect(out1[3].eql(Mat44.zero()));
    try testing.expect(out1[2].eql(s.poses.pose1[2]));
}

test "SkeletonMapper: lockTranslations" {
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();

    // Can be used without initialize. A joint without a parent stores its model space translation.
    var locked = [_]bool{false} ** 10;
    locked[0] = true;
    locked[5] = true;
    s.poses.pose2[0] = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 1.0), Vec3.init(1, 2, 3));
    try mapper.lockTranslations(&s.skeleton2, &locked, &s.poses.pose2);
    const l = mapper.getLockedTranslations();
    try testing.expectEqual(@as(usize, 2), l.len);
    try testing.expectEqual(SkeletonMapper.Locked{ .joint_idx = 0, .parent_joint_idx = -1, .translation = Vec3.init(1, 2, 3) }, l[0]);
    try testing.expectEqual(@as(i32, 5), l[1].joint_idx);
    try testing.expectEqual(@as(i32, 4), l[1].parent_joint_idx);
    try testing.expect(l[1].translation.eql(s.poses.pose2[4].inversed().mulVec3(s.poses.pose2[5].getTranslation())));

    // Locks are appended
    try mapper.lockTranslations(&s.skeleton2, &locked, &s.poses.pose2);
    try testing.expectEqual(@as(usize, 4), mapper.getLockedTranslations().len);
    try testing.expect(mapper.isJointTranslationLocked(0));
    try testing.expect(!mapper.isJointTranslationLocked(1));
}

/// Add a chain like a user of Jolt's non-const GetChains (the arrays are allocated with the mapper's allocator)
fn addChain(mapper: *SkeletonMapper, joints1: []const i32, joints2: []const i32) !void {
    const allocator = mapper.allocator;
    var chain: SkeletonMapper.Chain = .{};
    errdefer chain.deinit(allocator);
    try chain.joint_indices1.appendSlice(allocator, joints1);
    try chain.joint_indices2.appendSlice(allocator, joints2);
    try mapper.getChainsMut().append(allocator, chain);
}

test "SkeletonMapper: map with chains of 1 and 2 joints" {
    // Initialize only makes chains with joints in between, but a user can add any chain through getChainsMut (Jolt's
    // non-const GetChains). Jolt's Map handles chains of 1 joint (it points nowhere, sFromTo gives the identity) and of
    // 2 joints (no joints in between to update).
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});

    // The ragdoll's arm moved: the output without user chains
    var pose1 = s.poses.pose1;
    pose1[3].setTranslation(pose1[3].getTranslation().add(Vec3.init(0, 0.1, 0.05)));
    var reference: [10]Mat44 = undefined;
    mapper.map(&pose1, &s.poses.local2, &reference);

    // A chain of 1 joint (the head) changes nothing
    try addChain(&mapper, &.{2}, &.{6});
    var out2: [10]Mat44 = undefined;
    mapper.map(&pose1, &s.poses.local2, &out2);
    for (out2, reference) |a, b|
        try testing.expect(a.eql(b));

    // A chain of 2 joints (spine to arm) rotates the spine towards the ragdoll's arm, the arm keeps its mapping
    try addChain(&mapper, &.{ 1, 3 }, &.{ 4, 7 });
    mapper.map(&pose1, &s.poses.local2, &out2);
    try testing.expect(out2[4].getTranslation().eql(reference[4].getTranslation()));
    try testing.expect(!out2[4].getRotation().eql(reference[4].getRotation()));
    const direction = out2[4].mul(s.poses.local2[7]).getTranslation().sub(out2[4].getTranslation()).normalized();
    try testing.expect(direction.isClose(pose1[3].getTranslation().sub(pose1[1].getTranslation()).normalized(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(out2[7].eql(reference[7]));
    try testing.expect(out2[8].eql(reference[8]));
}

fn initializeOutOfMemory(allocator: Allocator) !void {
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper = Ref(SkeletonMapper).init(try SkeletonMapper.create(allocator));
    defer mapper.deinit();
    const mp = mapper.get().?;
    mp.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{}) catch |err| {
        // Nothing was added
        try testing.expectEqual(@as(usize, 0), mp.getMappings().len);
        try testing.expectEqual(@as(usize, 0), mp.getChains().len);
        try testing.expectEqual(@as(usize, 0), mp.getUnmapped().len);
        return err;
    };
    try testing.expectEqual(@as(usize, 2), mp.getChains().len);
    mp.lockAllTranslations(&s.skeleton2, &s.poses.pose2) catch |err| {
        try testing.expectEqual(@as(usize, 0), mp.getLockedTranslations().len);
        return err;
    };
    try testing.expectEqual(@as(usize, 8), mp.getLockedTranslations().len);
}

test "SkeletonMapper: out of memory in initialize and lockAllTranslations" {
    try testing.checkAllAllocationFailures(testing.allocator, initializeOutOfMemory, .{});
}

/// The arrays of a mapper, to compare its state before and after a failed call
const MapperState = struct {
    mappings: [16]SkeletonMapper.Mapping = undefined,
    num_mappings: usize = 0,
    chain_lengths: [8][2]usize = undefined,
    num_chains: usize = 0,
    unmapped: [16]SkeletonMapper.Unmapped = undefined,
    num_unmapped: usize = 0,

    fn of(mapper: *const SkeletonMapper) MapperState {
        var state: MapperState = .{ .num_mappings = mapper.getMappings().len, .num_chains = mapper.getChains().len, .num_unmapped = mapper.getUnmapped().len };
        @memcpy(state.mappings[0..state.num_mappings], mapper.getMappings());
        for (mapper.getChains(), 0..) |*c, i| state.chain_lengths[i] = .{ c.joint_indices1.items.len, c.joint_indices2.items.len };
        @memcpy(state.unmapped[0..state.num_unmapped], mapper.getUnmapped());
        return state;
    }

    fn expectEqual(self: *const MapperState, other: *const MapperState) !void {
        try testing.expectEqual(self.num_mappings, other.num_mappings);
        for (self.mappings[0..self.num_mappings], other.mappings[0..other.num_mappings]) |a, b| {
            try testing.expectEqual(a.joint_idx1, b.joint_idx1);
            try testing.expectEqual(a.joint_idx2, b.joint_idx2);
            try testing.expect(a.joint1_to2.eql(b.joint1_to2));
        }
        try testing.expectEqualSlices([2]usize, self.chain_lengths[0..self.num_chains], other.chain_lengths[0..other.num_chains]);
        try testing.expectEqualSlices(SkeletonMapper.Unmapped, self.unmapped[0..self.num_unmapped], other.unmapped[0..other.num_unmapped]);
    }
};

fn initializeTwiceOutOfMemory(allocator: Allocator) !void {
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});
    const first = MapperState.of(&mapper);

    // Jolt's release build appends a second initialization (its chain search also runs over the first mappings); when an
    // allocation fails, Zolt removes what the second call added
    mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{}) catch |err| {
        try first.expectEqual(&MapperState.of(&mapper));
        return err;
    };
    try testing.expectEqual(2 * first.num_mappings, mapper.getMappings().len);
    try testing.expectEqual(2 * first.num_chains, mapper.getChains().len);
    try testing.expectEqual(2 * first.num_unmapped, mapper.getUnmapped().len);
    var first_part = MapperState.of(&mapper);
    first_part.num_mappings = first.num_mappings;
    first_part.num_chains = first.num_chains;
    first_part.num_unmapped = first.num_unmapped;
    try first.expectEqual(&first_part);
}

test "SkeletonMapper: out of memory in a second initialize (release only)" {
    if (Core.enable_asserts) return error.SkipZigTest; // Jolt asserts "Should not be initialized yet"
    try testing.checkAllAllocationFailures(testing.allocator, initializeTwiceOutOfMemory, .{});
}

test "SkeletonMapper: map a sampled pose" {
    // The ragdoll pose of a SkeletonPose (model space joint matrices) mapped to the animation skeleton
    const allocator = testing.allocator;
    var s: TestSkeletons = undefined;
    try s.init(allocator);
    defer s.deinit();
    var mapper: SkeletonMapper = .init(allocator);
    defer mapper.deinit();
    try mapper.initialize(&s.skeleton1, &s.poses.pose1, &s.skeleton2, &s.poses.pose2, .{});

    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(&s.skeleton1);
    for (pose.getJointsMut().items, 0..) |*state, i| {
        const parent = s.skeleton1.getJoint(i).parent_joint_index;
        state.translation = s.poses.pose1[i].getTranslation().sub(if (parent >= 0) s.poses.pose1[@intCast(parent)].getTranslation() else Vec3.zero());
        state.rotation = Quat.rotation(Vec3.axisX(), 0.1 * @as(f32, @floatFromInt(i + 1)));
    }

    // Bend the spine and the head away from the chains of the animation skeleton
    pose.getJointMut(1).translation = pose.getJoint(1).translation.add(Vec3.init(0.2, 0, 0));
    pose.getJointMut(2).translation = pose.getJoint(2).translation.add(Vec3.init(0.1, 0, 0));
    pose.calculateJointMatrices();
    var out2: [10]Mat44 = undefined;
    mapper.map(pose.getJointMatrices(), &s.poses.local2, &out2);
    for (mapper.getMappings()) |*m| {
        // The mapped joints are where the ragdoll's joints are, the starts of the chains (pelvis, spine) are rotated
        const mapped = out2[@intCast(m.joint_idx2)];
        const ragdoll = pose.getJointMatrix(@intCast(m.joint_idx1)).*;
        try testing.expect(mapped.getTranslation().eql(ragdoll.getTranslation()));
        try testing.expect(mapped.eql(ragdoll) == (m.joint_idx1 >= 2));
    }
}
