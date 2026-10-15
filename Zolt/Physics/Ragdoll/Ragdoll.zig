//! Port of: Jolt/Physics/Ragdoll/Ragdoll.h, Jolt/Physics/Ragdoll/Ragdoll.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL (RagdollSettings, Part,
//! AdditionalConstraint): TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary
//! state is ported). JPH_OVERRIDE_NEW_DELETE: dropped.
//!
//! CharacterVehicleArchitecture E2 / E7:
//! - `RagdollSettings` and `Ragdoll` are RefTargets without virtual functions, like PhysicsScene: they keep the
//!   allocator that owns them (`create(allocator)` is `new`, reference count 0, put it in a `Ref` / `RefConst`;
//!   `init` / `deinit` an object on the stack or embedded). `RagdollSettings.Part : BodyCreationSettings` embeds
//!   `base: BodyCreationSettings`.
//! - Jolt's `Ref<TwoBodyConstraintSettings>` members (`Part.mToParent`, `AdditionalConstraint.mConstraint`) are
//!   `Ref(ConstraintSettings)` fields that hold a TwoBodyConstraintSettings (an abstract level has no addRef / release of
//!   its own, guide section 5, as PhysicsScene.ConnectedConstraint): `getToParent` / `setToParent` and
//!   `AdditionalConstraint.getConstraint` / `init` give and take the TwoBodyConstraintSettings. `Ragdoll.mConstraints`
//!   (`Array<Ref<TwoBodyConstraint>>`) is `constraints: std.ArrayList(Ref(Constraint))`, `getConstraint` /
//!   `getConstraintMut` return the TwoBodyConstraint.
//! - `CreateRagdoll` creates the bodies with BodyInterface.createBody and the constraints with
//!   TwoBodyConstraintSettings.createConstraint; `AddToPhysicsSystem` uses addBodiesPrepare / addBodiesFinalize and
//!   PhysicsSystem.addConstraints; `inLockBodies` (`opts.lock_bodies`) selects the locking or the no-lock BodyInterface /
//!   BodyLockInterface, as Jolt does.
//! - Indices (as in the Skeleton files): `usize` where an index is always a valid body / joint / constraint index
//!   (`getBodyID`, `getConstraint`, `getConstraintIndexForBodyIndex`'s argument), `i32` for Jolt's ints that can be -1
//!   (`getConstraintIndexForBodyIndex`'s result, `BodyIdxPair`) and for `AdditionalConstraint.body_idx` (Jolt's
//!   `int mBodyIdx[2]`, written as is to the binary state). `BodyIdxPair` (`std::pair<int, int>`) keeps the member
//!   names `first` / `second`.
//! - Overloads and out parameters: `SetPose(SkeletonPose)` / `SetPose(root offset, joint matrices)` are `setPose` /
//!   `setPoseMatrices`, likewise `getPose` / `getPoseMatrices` and `driveToPoseUsingKinematics` /
//!   `driveToPoseUsingKinematicsMatrices`; `DriveToPoseUsingMotors(prev pose, pose, delta time)` is
//!   `driveToPoseUsingMotorsWithVelocity`. The joint matrices are slices (at least one matrix per body). `GetPose(RVec3
//!   &outRootOffset, ...)` keeps the out parameter (Jolt does not write it for a ragdoll without bodies);
//!   `GetRootTransform(outPosition, outRotation)` returns a `RootTransform`. Jolt's defaulted `inLockBodies = true`,
//!   `inBasePriority = 0`, `inJointMatrices = nullptr` and `inMinSeparationDistance = 0.0f` are options.
//!
//! Errors (Zolt, E2: every function that allocates returns the error and leaves nothing half-built):
//! - The unbounded JPH_STACK_ALLOCs of `CreateRagdoll` (the body pointers), `AddToPhysicsSystem` and
//!   `RemoveFromPhysicsSystem` (the copies of the body IDs) allocate with the allocator of the object that owns the
//!   function (guide section 5), so these functions return `Allocator.Error`. `AddToPhysicsSystem` and
//!   `RemoveFromPhysicsSystem` also allocate the `*Constraint` list that PhysicsSystem.addConstraints /
//!   removeConstraints take (Jolt reinterprets its `Ref<TwoBodyConstraint>` array as `Constraint **`).
//! - `createRagdoll` allocates the ragdoll, its arrays and its constraints with the settings' allocator (Jolt: `new`)
//!   and returns `CreateBodyError!?*Ragdoll`: null when out of bodies (Jolt's soft failure, it deletes the ragdoll,
//!   which destroys the bodies it created), `error.InvalidShape` for a part without a shape (Jolt crashes in
//!   CreateBody) and `error.OutOfMemory`; on an error the bodies and constraints it created are destroyed. The body ID
//!   of a new body is stored right after its creation (Jolt: after its constraint), so that a failing constraint
//!   creation destroys that body too (nothing reads the ID list in between).
//! - `addToPhysicsSystem` allocates first, then prepares the bodies (QuadTree's AddBodiesPrepare allocates), then adds
//!   the constraints and only then finalizes the bodies (Jolt: AddBodiesFinalize, then AddConstraints). Adding the
//!   constraints is the last step that can fail; when it fails the prepared bodies are aborted (AddBodiesAbort), so a
//!   failed call leaves nothing in the system (undoing AddBodiesFinalize with RemoveBodies would zero the velocities of
//!   activated bodies). The constraint manager does not read the bodies, so the state after a successful call is
//!   Jolt's. `removeFromPhysicsSystem` allocates before it removes anything.
//! - `stabilize` calculates the mass properties that it converts before it changes the first part, and
//!   `disableParentChildCollisions` creates the shapes of the parts (`getShape`, which can convert shape settings)
//!   before it assigns the group filter: a failure (out of memory, `error.InvalidShape` for a part without a shape,
//!   where Jolt dereferences null) changes no part. The `calculate*` functions reserve before they clear, so a failure
//!   keeps the old table. `calculateConstraintPriorities` allocates its priorities (Allocator.Error).
//! - `saveBinaryState` returns `CreateBodyError` (BodyCreationSettings.saveWithChildren: the maps of the shapes,
//!   materials and group filters, `error.InvalidShape` when a part without a shape is saved with the shapes; the bytes
//!   of the parts before it are written then). `restoreFromBinaryState` (Jolt: sRestoreFromBinaryState) returns
//!   Jolt's `RagdollResult` with Jolt's error texts, only allocation failure is a Zig error (a part or constraint count
//!   that cannot be allocated, a corrupt stream where Jolt's resize aborts, is error.OutOfMemory).
//! - Jolt's `DynamicCast<TwoBodyConstraintSettings>` of a restored constraint is `asTwoBodyConstraintSettings`: the
//!   restored class must derive from TwoBodyConstraintSettings (checked with the RTTI hash against
//!   `RegisterTypes.constraint_settings_types`), otherwise the part has no constraint (Jolt's null), like Jolt for a
//!   VehicleConstraintSettings in the stream.
//! - The has-constraint flag that Jolt reads into a `bool` is read as a byte (`!= 0`), like BodyCreationSettings' bools.
//!
//! Where Jolt has undefined behavior or dereferences null, Zolt panics with a message (also in optimized builds): a
//! null skeleton (`mSkeleton->`, where Jolt dereferences it), the null settings of a ragdoll made with the public
//! constructor (`mRagdollSettings->` in DriveToPoseUsingMotors), a null additional constraint (SaveBinaryState,
//! CreateRagdoll, CalculateConstraintPriorities), a part whose constraint connects it to a parent that is not created
//! before it
//! (CreateRagdoll reads an uninitialized slot of its JPH_STACK_ALLOC), a body that cannot be locked in IsActive /
//! SetGroupID / GetPose (Jolt dereferences the null body of a destroyed or invalid body ID). Out of range indices
//! (GetRootTransform of a ragdoll without bodies, a body index of an additional constraint) are Zig's bounds checks. A
//! pose of a ragdoll without bodies: Jolt's GetPose(SkeletonPose &) sets the pose's root offset to an uninitialized
//! RVec3, Zolt to zero.
//!
//! Jolt's asserts on the user's input are checked in safe builds only (`if (Core.enable_asserts)`), so that optimized
//! builds compute what Jolt's release build computes: the pose skeleton in SetPose / GetPose / DriveToPose*, `inDeltaTime
//! > 0`, "Base priority is too high and will cause overflows" (also for a ragdoll without parts, as in Jolt),
//! AreJointsCorrectlyOrdered in CalculateConstraintPriorities, the joint count in DisableParentChildCollisions;
//! `JPH_ASSERT(false)` ("Failed to decompose the inertia tensor!", "Constraint type not implemented!") is a panic in safe
//! builds followed by Jolt's release behavior (return false / do nothing).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const math = @import("../../Math/Math.zig");
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const RefCount = ReferenceFile.RefCount;
const Result = @import("../../Core/Result.zig").Result;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const Skeleton = @import("../../Skeleton/Skeleton.zig").Skeleton;
const SkeletonPose = @import("../../Skeleton/SkeletonPose.zig").SkeletonPose;
const RegisterTypes = @import("../../RegisterTypes.zig");
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyInterface = @import("../Body/BodyInterface.zig").BodyInterface;
const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const BodyLockRead = @import("../Body/BodyLock.zig").BodyLockRead;
const BodyLockMultiFile = @import("../Body/BodyLockMulti.zig");
const BodyLockMultiRead = BodyLockMultiFile.BodyLockMultiRead;
const BodyLockMultiWrite = BodyLockMultiFile.BodyLockMultiWrite;
const BodyCreationSettingsFile = @import("../Body/BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const MassProperties = @import("../Body/MassProperties.zig").MassProperties;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const GroupFilter = @import("../Collision/GroupFilter.zig").GroupFilter;
const GroupFilterTable = @import("../Collision/GroupFilterTable.zig").GroupFilterTable;
const CollisionDispatch = @import("../Collision/CollisionDispatch.zig");
const CollideShapeSettings = @import("../Collision/CollideShape.zig").CollideShapeSettings;
const CollideShapeCollector = @import("../Collision/Shape/Shape.zig").CollideShapeCollector;
const AnyHitCollisionCollector = @import("../Collision/CollisionCollectorImpl.zig").AnyHitCollisionCollector;
const ConstraintFile = @import("../Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const TwoBodyConstraintFile = @import("../Constraints/TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const SwingTwistConstraint = @import("../Constraints/SwingTwistConstraint.zig").SwingTwistConstraint;
const HingeConstraint = @import("../Constraints/HingeConstraint.zig").HingeConstraint;
const Activation = @import("../EActivation.zig").Activation;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;

/// sRagdollGetBodyInterface: the locking or the no-lock body interface
fn ragdollGetBodyInterface(system: *PhysicsSystem, lock_bodies: bool) *BodyInterface {
    return if (lock_bodies) system.getBodyInterfaceMut() else system.getBodyInterfaceNoLockMut();
}

/// sRagdollGetBodyLockInterface: the locking or the no-lock body lock interface
fn ragdollGetBodyLockInterface(system: *const PhysicsSystem, lock_bodies: bool) *const BodyLockInterface {
    return if (lock_bodies) system.getBodyLockInterface() else system.getBodyLockInterfaceNoLock();
}

/// Jolt's `DynamicCast<TwoBodyConstraintSettings>` of a restored constraint: null unless the class of `settings` (a
/// class that ConstraintSettings.restoreFromBinaryState can create, `RegisterTypes.constraint_settings_types`) derives
/// from TwoBodyConstraintSettings
fn asTwoBodyConstraintSettings(settings: *ConstraintSettings) ?*TwoBodyConstraintSettings {
    const hash = settings.getRTTIHash();
    inline for (ConstraintSettings.constraint_settings_types) |T| {
        if (comptime @hasDecl(T, "rtti_name") and virtual.isDerivedFrom(T, TwoBodyConstraintSettings)) {
            if (hash == comptime ConstraintSettings.rttiHash(T.rtti_name))
                return virtual.downcast(TwoBodyConstraintSettings, settings);
        }
    }
    return null;
}

/// A `Ref(ConstraintSettings)` that holds a TwoBodyConstraintSettings (Jolt's `Ref<TwoBodyConstraintSettings>`)
fn twoBodySettingsOf(ref: Ref(ConstraintSettings)) ?*TwoBodyConstraintSettings {
    const settings = ref.get() orelse return null;
    return virtual.downcast(TwoBodyConstraintSettings, settings);
}

/// Contains the structure of a ragdoll
pub const RagdollSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, RagdollSettings)

    /// TEMPORARY (Ragdoll group, removed with the simulation parity slot): the class is ported, but the scenes of
    /// `Blocker.ragdolls` stay skipped until RagdollSlot of SimulationParity.zig is filled
    pub const is_stub = true;

    pub const RagdollResult = Result(Ref(RagdollSettings));

    /// std::pair<int, int>: the indices of the two bodies of a constraint (index in mBodyIDs)
    pub const BodyIdxPair = struct {
        first: i32,
        second: i32,
    };

    /// A single rigid body sub part of the ragdoll
    pub const Part = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Part)

        /// BodyCreationSettings (the C++ base class)
        base: BodyCreationSettings = .{},

        /// The constraint that connects this part to its parent part (should be null for the root): a
        /// TwoBodyConstraintSettings (Jolt's `Ref<TwoBodyConstraintSettings> mToParent`, use getToParent / setToParent)
        to_parent: Ref(ConstraintSettings) = .empty,

        /// A part with a copy of `settings` (references to its shape and group filter) and a reference to `to_parent`
        pub fn init(settings: *const BodyCreationSettings, to_parent: ?*TwoBodyConstraintSettings) Part {
            return .{ .base = settings.clone(), .to_parent = .init(if (to_parent) |c| &c.base else null) };
        }

        /// Copy constructor: adds references to the shape, the group filter and the constraint settings
        pub fn clone(self: *const Part) Part {
            return .{ .base = self.base.clone(), .to_parent = self.to_parent.clone() };
        }

        /// Destructor: releases the references
        pub fn deinit(self: *Part) void {
            self.to_parent.deinit();
            self.base.deinit();
        }

        /// The constraint that connects this part to its parent part (mToParent)
        pub fn getToParent(self: *const Part) ?*TwoBodyConstraintSettings {
            return twoBodySettingsOf(self.to_parent);
        }

        /// mToParent = inSettings (adds a reference to the new settings, releases the old ones)
        pub fn setToParent(self: *Part, settings: ?*TwoBodyConstraintSettings) void {
            self.to_parent.set(if (settings) |s| &s.base else null);
        }
    };

    /// List of ragdoll parts
    pub const PartVector = std.ArrayList(Part);

    /// A constraint that connects two bodies in a ragdoll (for non parent child related constraints)
    pub const AdditionalConstraint = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, AdditionalConstraint)

        /// Indices of the bodies that this constraint connects (Jolt's default constructor leaves them uninitialized,
        /// Zolt: 0)
        body_idx: [2]i32 = .{ 0, 0 },
        /// The constraint that connects these bodies: a TwoBodyConstraintSettings (Jolt's
        /// `Ref<TwoBodyConstraintSettings> mConstraint`, use getConstraint)
        constraint: Ref(ConstraintSettings) = .empty,

        /// Constructor AdditionalConstraint(inBodyIdx1, inBodyIdx2, inConstraint) (adds a reference to the constraint)
        pub fn init(body_idx1: i32, body_idx2: i32, constraint: ?*TwoBodyConstraintSettings) AdditionalConstraint {
            return .{ .body_idx = .{ body_idx1, body_idx2 }, .constraint = .init(if (constraint) |c| &c.base else null) };
        }

        /// Copy constructor: adds a reference to the constraint
        pub fn clone(self: *const AdditionalConstraint) AdditionalConstraint {
            return .{ .body_idx = self.body_idx, .constraint = self.constraint.clone() };
        }

        /// Destructor: releases the constraint
        pub fn deinit(self: *AdditionalConstraint) void {
            self.constraint.deinit();
        }

        /// The constraint that connects these bodies (mConstraint)
        pub fn getConstraint(self: *const AdditionalConstraint) ?*TwoBodyConstraintSettings {
            return twoBodySettingsOf(self.constraint);
        }
    };

    /// List of additional constraints
    pub const AdditionalConstraintVector = std.ArrayList(AdditionalConstraint);

    /// RefTarget<RagdollSettings>
    ref_count: RefCount = .{},
    /// Allocates the arrays, frees the settings, and allocates what the functions of the settings allocate (the
    /// temporary arrays, the group filter of disableParentChildCollisions, the ragdolls of createRagdoll)
    allocator: Allocator,

    /// The skeleton for this ragdoll
    skeleton: Ref(Skeleton) = .empty,

    /// For each of the joints, the body and constraint attaching it to its parent body (1-on-1 with mSkeleton.GetJoints())
    /// (the parts are owned by the settings: allocated with `allocator`, released by `deinit`)
    parts: PartVector = .empty,

    /// A list of constraints that connects two bodies in a ragdoll (for non parent child related constraints) (owned by
    /// the settings like `parts`)
    additional_constraints: AdditionalConstraintVector = .empty,

    /// Table that maps a body index (index in mBodyIDs) to the constraint index with which it is connected to its parent. -1 if there is no constraint associated with the body.
    /// (private in Jolt, use getBodyIndexToConstraintIndex)
    body_index_to_constraint_index: std.ArrayList(i32) = .empty,

    /// Table that maps a constraint index (index in mConstraints) to the indices of the bodies that the constraint is connected to (index in mBodyIDs)
    /// (private in Jolt, use getConstraintIndexToBodyIdxPair)
    constraint_index_to_body_idx_pair: std.ArrayList(BodyIdxPair) = .empty,

    /// Constructor of settings that are not on the heap (`deinit` them)
    pub fn init(allocator: Allocator) RagdollSettings {
        return .{ .allocator = allocator };
    }

    /// new RagdollSettings: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*RagdollSettings {
        const self = try allocator.create(RagdollSettings);
        self.* = .init(allocator);
        return self;
    }

    /// Destructor: releases the skeleton and the references of the parts and additional constraints, frees the arrays
    pub fn deinit(self: *RagdollSettings) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *RagdollSettings) void {
        self.constraint_index_to_body_idx_pair.deinit(self.allocator);
        self.body_index_to_constraint_index.deinit(self.allocator);
        for (self.additional_constraints.items) |*c| c.deinit();
        self.additional_constraints.deinit(self.allocator);
        for (self.parts.items) |*p| p.deinit();
        self.parts.deinit(self.allocator);
        self.skeleton.deinit();
    }

    /// Add a reference to this object
    pub fn addRef(self: *const RagdollSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const RagdollSettings) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark these settings as embedded (on the stack or a member), the last release will not destroy them
    pub fn setEmbedded(self: *const RagdollSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const RagdollSettings) u32 {
        return self.ref_count.get();
    }

    /// `mSkeleton->`: Jolt dereferences the skeleton without a check
    fn skeletonOf(self: *const RagdollSettings) *Skeleton {
        return self.skeleton.get() orelse @panic("RagdollSettings: no skeleton (Jolt dereferences a null mSkeleton)");
    }

    /// Stabilize the constraints of the ragdoll
    /// @return True on success, false on failure.
    /// (Zolt: `error.InvalidShape` for a part without a shape whose mass properties are calculated, error.OutOfMemory;
    /// nothing changes then, see the file header)
    pub fn stabilize(self: *RagdollSettings) CreateBodyError!bool {
        // Based on: Stop my Constraints from Blowing Up! - Oliver Strunk (Havok)
        // Do 2 things:
        // 1. Limit the mass ratios between parents and children (slide 16)
        // 2. Increase the inertia of parents so that they're bigger or equal to the sum of their children (slide 34)

        const skeleton = self.skeletonOf();
        const joint_count = skeleton.getJointCount();

        // If we don't have any joints there's nothing to stabilize
        if (joint_count == 0)
            return true;

        const allocator = self.allocator;
        const parts = self.parts.items;

        // Zolt: the arrays that Jolt allocates in the loops below (indices, mass_ratios and principals, once per chain)
        // are allocated once here, and the mass properties that are calculated are calculated before the first part
        // changes, so that a failure changes nothing
        const visited = try allocator.alloc(bool, joint_count);
        defer allocator.free(visited);
        const indices = try allocator.alloc(usize, joint_count);
        defer allocator.free(indices);
        const mass_ratios = try allocator.alloc(f32, joint_count);
        defer allocator.free(mass_ratios);
        const principals = try allocator.alloc(Principal, parts.len);
        defer allocator.free(principals);
        const calculated = try allocator.alloc(MassProperties, joint_count);
        defer allocator.free(calculated);
        for (0..joint_count) |v| {
            const p = &parts[v];
            if (p.base.hasMassProperties() and p.base.override_mass_properties != .mass_and_inertia_provided)
                calculated[v] = try p.base.getMassProperties(allocator);
        }

        // The skeleton can contain one or more static bodies. We can't modify the mass for those so we start a new stabilization chain for each joint under a static body until we reach the next static body.
        // This array keeps track of which joints have been processed.
        for (visited, 0..) |*visited_v, v| {
            // Mark static bodies as visited so we won't process these
            const p = &parts[v];
            const has_mass_properties = p.base.hasMassProperties();
            visited_v.* = !has_mass_properties;

            if (has_mass_properties and p.base.override_mass_properties != .mass_and_inertia_provided) {
                // Mass properties not yet calculated, do it now
                p.base.mass_properties_override = calculated[v];
                p.base.override_mass_properties = .mass_and_inertia_provided;
            }
        }

        // Find first unvisited part that either has no parent or that has a parent that was visited
        for (0..joint_count) |first_idx| {
            const first_idx_parent = skeleton.getJoint(first_idx).parent_joint_index;
            if (!visited[first_idx] and (first_idx_parent == -1 or visited[@intCast(first_idx_parent)])) {
                // Find all children of first_idx and their children up to the next static part
                var next_to_process: usize = 0;
                var num_indices: usize = 0;
                visited[first_idx] = true;
                indices[num_indices] = first_idx;
                num_indices += 1;
                while (true) {
                    const parent_idx = indices[next_to_process];
                    next_to_process += 1;
                    for (0..joint_count) |child_idx| {
                        if (!visited[child_idx] and skeleton.getJoint(child_idx).parent_joint_index == @as(i32, @intCast(parent_idx))) {
                            visited[child_idx] = true;
                            indices[num_indices] = child_idx;
                            num_indices += 1;
                        }
                    }
                    if (next_to_process >= num_indices) break;
                }
                const chain = indices[0..num_indices];

                // If there's only 1 body, we can't redistribute mass
                if (chain.len == 1)
                    continue;

                const min_mass_ratio: f32 = 0.8;
                const max_mass_ratio: f32 = 1.2;

                // Ensure that the mass ratio from parent to child is within a range
                var total_mass_ratio: f32 = 1.0;
                mass_ratios[chain[0]] = 1.0;
                for (chain[1..]) |child_idx| {
                    const parent_idx: usize = @intCast(skeleton.getJoint(child_idx).parent_joint_index);
                    const ratio = parts[child_idx].base.mass_properties_override.mass / parts[parent_idx].base.mass_properties_override.mass;
                    mass_ratios[child_idx] = mass_ratios[parent_idx] * math.clamp(ratio, min_mass_ratio, max_mass_ratio);
                    total_mass_ratio += mass_ratios[child_idx];
                }

                // Calculate total mass of this chain
                var total_mass: f32 = 0.0;
                for (chain) |idx|
                    total_mass += parts[idx].base.mass_properties_override.mass;

                // Calculate how much mass belongs to a ratio of 1
                const ratio_to_mass = total_mass / total_mass_ratio;

                // Adjust all masses and inertia tensors for the new mass
                for (chain) |i| {
                    const mp = &parts[i].base.mass_properties_override;
                    const old_mass = mp.mass;
                    const new_mass = mass_ratios[i] * ratio_to_mass;
                    mp.mass = new_mass;
                    mp.inertia = mp.inertia.mulScalar(new_mass / old_mass);
                    mp.inertia.setColumn4(3, Vec4.init(0, 0, 0, 1));
                }

                const max_inertia_increase: f32 = 2.0;

                // Get the principal moments of inertia for all parts
                // (Zolt: Jolt's array of Principals is new for every chain, mChildSum starts at 0)
                for (chain) |i|
                    principals[i].child_sum = 0.0;
                for (chain) |i| {
                    const decomposition = parts[i].base.mass_properties_override.decomposePrincipalMomentsOfInertia() orelse {
                        if (Core.enable_asserts) @panic("Failed to decompose the inertia tensor!");
                        return false;
                    };
                    principals[i].rotation = decomposition.rotation;
                    principals[i].diagonal = decomposition.diagonal;
                }

                // Calculate sum of child inertias
                // Walk backwards so we sum the leaves first
                var i: usize = chain.len - 1;
                while (i > 0) : (i -= 1) {
                    const child_idx = chain[i];
                    const parent_idx: usize = @intCast(skeleton.getJoint(child_idx).parent_joint_index);
                    principals[parent_idx].child_sum += principals[child_idx].diagonal.getX() + principals[child_idx].child_sum;
                }

                // Adjust inertia tensors for all parts
                for (chain) |idx| {
                    const p = &parts[idx];
                    const principal = &principals[idx];
                    if (principal.child_sum != 0.0) {
                        // Calculate minimum inertia this object should have based on it children
                        const minimum = math.min(max_inertia_increase * principal.diagonal.getX(), principal.child_sum);
                        principal.diagonal = Vec3.max(principal.diagonal, Vec3.replicate(minimum));

                        // Recalculate moment of inertia in body space
                        p.base.mass_properties_override.inertia = principal.rotation.mul(Mat44.scaleVec3(principal.diagonal)).mul(principal.rotation.inversed3x3());
                    }
                }
            }
        }

        return true;
    }

    /// The principal moments of inertia of a part (the local struct of Stabilize)
    const Principal = struct {
        rotation: Mat44,
        diagonal: Vec3,
        child_sum: f32 = 0.0,
    };

    /// Initializes the constraint priorities so that constraints near the leaves of the ragdoll have a lower priority
    /// than constraints near the root of the ragdoll.
    /// @param opts.base_priority The lowest priority that will be used in the ragdoll.
    pub fn calculateConstraintPriorities(self: *RagdollSettings, opts: struct { base_priority: u32 = 0 }) Allocator.Error!void {
        const base_priority = opts.base_priority;
        const num_parts = self.parts.items.len;
        if (Core.enable_asserts) std.debug.assert(base_priority +% @as(u32, @truncate(num_parts)) > base_priority); // Base priority is too high and will cause overflows
        if (Core.enable_asserts) std.debug.assert(self.skeletonOf().areJointsCorrectlyOrdered());

        // Calculate priority for each part. Start with the base priority and increment towards the root
        const priorities = try self.allocator.alloc(u32, num_parts);
        defer self.allocator.free(priorities);
        @memset(priorities, base_priority);
        var i: usize = num_parts;
        while (i > 0) {
            i -= 1;
            var cur_priority = base_priority;
            var j: i32 = @intCast(i);
            while (true) {
                const jj: usize = @intCast(j);
                priorities[jj] = @max(priorities[jj], cur_priority);
                cur_priority +%= 1;

                j = self.skeletonOf().getJoint(jj).parent_joint_index;
                if (j == -1) break;
            }
        }

        // Copy the priorities to the constraints
        for (self.parts.items, priorities) |*p, priority| {
            if (p.getToParent()) |to_parent|
                to_parent.base.constraint_priority = priority;
        }

        // Use the minimum of the priorities of connected bodies for additional constraints
        for (self.additional_constraints.items) |*constraint| {
            const c = constraint.getConstraint() orelse @panic("RagdollSettings.calculateConstraintPriorities: an additional constraint without settings (Jolt dereferences a null pointer)");
            c.base.constraint_priority = @min(priorities[@intCast(constraint.body_idx[0])], priorities[@intCast(constraint.body_idx[1])]);
        }
    }

    /// After the ragdoll has been fully configured, call this function to automatically create and add a GroupFilterTable collision filter to all bodies
    /// and configure them so that parent and children don't collide.
    ///
    /// This will:
    /// - Create a GroupFilterTable and assign it to all of the bodies in a ragdoll.
    /// - Each body in your ragdoll will get a SubGroupID that is equal to the joint index in the Skeleton that it is attached to.
    /// - Loop over all joints in the Skeleton and call GroupFilterTable::DisableCollision(joint index, parent joint index).
    /// - When a pose is provided through inJointMatrices the function will detect collisions between joints
    /// (they must be separated by more than inMinSeparationDistance to be treated as not colliding) and automatically disable collisions.
    ///
    /// When you create an instance using Ragdoll::CreateRagdoll pass in a unique GroupID for each ragdoll (e.g. a simple counter), note that this number
    /// should be unique throughout the PhysicsSystem, so if you have different types of ragdolls they should not share the same GroupID.
    /// (Zolt: `opts.joint_matrices` holds a matrix per joint; the group filter is allocated with the settings' allocator.
    /// `error.InvalidShape` for a part without a shape when joint matrices are given, error.OutOfMemory; no part changes
    /// then.)
    pub fn disableParentChildCollisions(self: *RagdollSettings, opts: struct { joint_matrices: ?[]const Mat44 = null, min_separation_distance: f32 = 0.0 }) CreateBodyError!void {
        const skeleton = self.skeletonOf();
        const joint_count = skeleton.getJointCount();
        if (Core.enable_asserts) std.debug.assert(joint_count == self.parts.items.len);

        // Create a group filter table that disables collisions between parent and child
        const group_filter = try GroupFilterTable.create(self.allocator, .{ .num_sub_groups = @intCast(joint_count) });
        var group_filter_ref = RefConst(GroupFilter).init(group_filter.asGroupFilter());
        defer group_filter_ref.deinit();
        for (0..joint_count) |joint_idx| {
            const parent_joint = skeleton.getJoint(joint_idx).parent_joint_index;
            if (parent_joint >= 0)
                group_filter.disableCollision(@intCast(joint_idx), @intCast(parent_joint));
        }

        // If joint matrices are provided
        if (opts.joint_matrices) |joint_matrices| {
            // Loop over all joints
            for (0..joint_count) |j1| {
                // Shape and transform for joint 1
                const part1 = &self.parts.items[j1];
                const shape1 = (try part1.base.getShape(self.allocator)) orelse return error.InvalidShape;
                const com1 = joint_matrices[j1].preTranslated(shape1.getCenterOfMass()).decompose();

                // Loop over all other joints
                for (j1 + 1..joint_count) |j2| {
                    if (group_filter.isCollisionEnabled(@intCast(j1), @intCast(j2))) { // Only if collision is still enabled we need to test
                        // Shape and transform for joint 2
                        const part2 = &self.parts.items[j2];
                        const shape2 = (try part2.base.getShape(self.allocator)) orelse return error.InvalidShape;
                        const com2 = joint_matrices[j2].preTranslated(shape2.getCenterOfMass()).decompose();

                        // Collision settings
                        var settings: CollideShapeSettings = .{};
                        settings.active_edge_mode = .collide_with_all;
                        settings.back_face_mode = .collide_with_back_faces;
                        settings.max_separation_distance = opts.min_separation_distance;

                        // Only check if one of the two bodies can become dynamic
                        if (part1.base.hasMassProperties() or part2.base.hasMassProperties()) {
                            // If there is a collision, disable the collision between the joints
                            var collector = AnyHitCollisionCollector(CollideShapeCollector).init();
                            defer collector.deinit();
                            if (part1.base.hasMassProperties()) // Ensure that the first shape is always a dynamic one (we can't check mesh vs convex but we can check convex vs mesh)
                                CollisionDispatch.collideShapeVsShape(shape1, shape2, com1.scale, com2.scale, com1.rotation_translation, com2.rotation_translation, .{}, .{}, &settings, &collector.base, &.{})
                            else
                                CollisionDispatch.collideShapeVsShape(shape2, shape1, com2.scale, com1.scale, com2.rotation_translation, com1.rotation_translation, .{}, .{}, &settings, &collector.base, &.{});
                            if (collector.hadHit())
                                group_filter.disableCollision(@intCast(j1), @intCast(j2));
                        }
                    }
                }
            }
        }

        // Loop over the body parts and assign them a sub group ID and the group filter
        for (self.parts.items[0..joint_count], 0..) |*part, joint_idx| {
            part.base.collision_group.setSubGroupID(@intCast(joint_idx));
            part.base.collision_group.setGroupFilter(group_filter.asGroupFilter());
        }
    }

    /// Saves the state of this object in binary form to inStream.
    /// @param stream The stream to save the state to
    /// @param save_shapes If the shapes should be saved as well (these could be shared between ragdolls, in which case the calling application may want to write custom code to restore them)
    /// @param save_group_filter If the group filter should be saved as well (these could be shared)
    /// (Zolt: the maps are allocated with the settings' allocator; `error.InvalidShape` when a part without a shape is
    /// saved with its shape)
    pub fn saveBinaryState(self: *const RagdollSettings, stream: StreamOut, save_shapes: bool, save_group_filter: bool) CreateBodyError!void {
        var shape_to_id: BodyCreationSettings.ShapeToIDMap = .empty;
        defer shape_to_id.deinit(self.allocator);
        var material_to_id: BodyCreationSettings.MaterialToIDMap = .empty;
        defer material_to_id.deinit(self.allocator);
        var group_filter_to_id: BodyCreationSettings.GroupFilterToIDMap = .empty;
        defer group_filter_to_id.deinit(self.allocator);

        // Save skeleton
        self.skeletonOf().saveBinaryState(stream);

        // Save parts
        stream.write(@as(u32, @intCast(self.parts.items.len)));
        for (self.parts.items) |*p| {
            // Write body creation settings
            try p.base.saveWithChildren(self.allocator, stream, if (save_shapes) &shape_to_id else null, if (save_shapes) &material_to_id else null, if (save_group_filter) &group_filter_to_id else null);

            // Save constraint
            const to_parent = p.to_parent.get();
            stream.write(to_parent != null);
            if (to_parent) |c|
                c.saveBinaryState(stream);
        }

        // Save additional constraints
        stream.write(@as(u32, @intCast(self.additional_constraints.items.len)));
        for (self.additional_constraints.items) |*c| {
            // Save bodies indices
            stream.write(c.body_idx);

            // Save constraint
            const constraint = c.constraint.get() orelse @panic("RagdollSettings.saveBinaryState: an additional constraint without settings (Jolt dereferences a null pointer)");
            constraint.saveBinaryState(stream);
        }
    }

    /// Restore a saved ragdoll from inStream (Jolt: sRestoreFromBinaryState; the result holds a reference to the new
    /// settings, `deinit` it)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!RagdollResult {
        var result: RagdollResult = .empty;

        // Restore skeleton
        var skeleton_result = try Skeleton.restoreFromBinaryState(allocator, stream);
        defer skeleton_result.deinit();
        if (skeleton_result.hasError()) {
            result.setError(skeleton_result.getError());
            return result;
        }

        // Create ragdoll
        var ragdoll = Ref(RagdollSettings).init(try create(allocator));
        defer ragdoll.deinit();
        const r = ragdoll.get().?;
        r.skeleton.set(skeleton_result.getPtr());

        var id_to_shape: BodyCreationSettings.IDToShapeMap = .empty;
        defer releaseList(allocator, &id_to_shape);
        var id_to_material: BodyCreationSettings.IDToMaterialMap = .empty;
        defer releaseList(allocator, &id_to_material);
        var id_to_group_filter: BodyCreationSettings.IDToGroupFilterMap = .empty;
        defer releaseList(allocator, &id_to_group_filter);

        // Reserve some memory to avoid frequent reallocations
        try id_to_shape.ensureTotalCapacity(allocator, 1024);
        try id_to_material.ensureTotalCapacity(allocator, 128);
        try id_to_group_filter.ensureTotalCapacity(allocator, 128);

        // Read parts (Jolt resizes the array to the length that is read, then restores the elements in place)
        var len: u32 = 0;
        stream.read(&len);
        try r.parts.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            // Read creation settings
            var bcs_result = try BodyCreationSettings.restoreWithChildren(allocator, stream, &id_to_shape, &id_to_material, &id_to_group_filter);
            defer bcs_result.deinit();
            if (bcs_result.hasError()) {
                result.setError(bcs_result.getError());
                return result;
            }
            r.parts.appendAssumeCapacity(.{ .base = bcs_result.get().clone() });
            const p = &r.parts.items[r.parts.items.len - 1];

            // Read constraint (Jolt reads a bool, Zolt the byte, see the file header)
            var has_constraint: u8 = 0;
            stream.read(&has_constraint);
            if (has_constraint != 0) {
                var constraint_result = try ConstraintSettings.restoreFromBinaryState(allocator, stream);
                defer constraint_result.deinit();
                if (constraint_result.hasError()) {
                    result.setError(constraint_result.getError());
                    return result;
                }
                p.setToParent(asTwoBodyConstraintSettings(constraint_result.getPtr().?));
            }
        }

        // Read additional constraints
        len = 0;
        stream.read(&len);
        try r.additional_constraints.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            r.additional_constraints.appendAssumeCapacity(.{});
            const c = &r.additional_constraints.items[r.additional_constraints.items.len - 1];

            // Read body indices
            stream.read(&c.body_idx);

            // Read constraint
            var constraint_result = try ConstraintSettings.restoreFromBinaryState(allocator, stream);
            defer constraint_result.deinit();
            if (constraint_result.hasError()) {
                result.setError(constraint_result.getError());
                return result;
            }
            const constraint = asTwoBodyConstraintSettings(constraint_result.getPtr().?);
            c.constraint.set(if (constraint) |s| &s.base else null);
        }

        // Create mapping tables
        try r.calculateBodyIndexToConstraintIndex();
        try r.calculateConstraintIndexToBodyIdxPair();

        result.set(ragdoll.clone());
        return result;
    }

    fn releaseList(allocator: Allocator, list: anytype) void {
        for (list.items) |*item| item.deinit();
        list.deinit(allocator);
    }

    /// Create ragdoll instance from these settings
    /// @return Newly created ragdoll or null when out of bodies
    /// (Zolt: the ragdoll has reference count 0 like Jolt's `new`, put it in a `Ref`. It, its arrays and its constraints
    /// are allocated with the settings' allocator. `error.InvalidShape` for a part without a shape, error.OutOfMemory:
    /// the bodies and constraints that were created are destroyed again.)
    pub fn createRagdoll(self: *const RagdollSettings, collision_group: CollisionGroup.GroupID, user_data: u64, system: *PhysicsSystem) CreateBodyError!?*Ragdoll {
        const r = try Ragdoll.create(self.allocator, system);
        errdefer r.destroy(); // delete r: destroys the bodies and releases the constraints that were created
        r.ragdoll_settings.set(self);
        try r.body_ids.ensureTotalCapacity(r.allocator, self.parts.items.len);
        try r.constraints.ensureTotalCapacity(r.allocator, self.parts.items.len + self.additional_constraints.items.len);

        // Create bodies and constraints
        const bi = system.getBodyInterfaceMut();
        const bodies = try self.allocator.alloc(*Body, self.parts.items.len); // JPH_STACK_ALLOC
        defer self.allocator.free(bodies);
        for (self.parts.items, 0..) |*p, joint_idx| {
            const body2 = (try bi.createBody(&p.base)) orelse {
                // Out of bodies, failed to create ragdoll
                r.destroy();
                return null;
            };
            body2.getCollisionGroupMut().setGroupID(collision_group);
            body2.setUserData(user_data);

            // Temporarily store body pointer for hooking up constraints
            bodies[joint_idx] = body2;

            // Store body ID and constraint in parallel arrays (Zolt: the ID before the constraint is created, see the
            // file header)
            r.body_ids.appendAssumeCapacity(body2.getID());

            // Create constraint
            if (p.getToParent()) |to_parent| {
                const parent = self.skeletonOf().getJoint(joint_idx).parent_joint_index;
                if (parent < 0 or parent >= @as(i32, @intCast(joint_idx)))
                    @panic("RagdollSettings.createRagdoll: a part is connected to a parent part that is not created before it (Jolt reads an uninitialized body pointer)");
                const body1 = bodies[@intCast(parent)];
                const constraint = try to_parent.createConstraint(self.allocator, body1, body2);
                r.constraints.appendAssumeCapacity(.init(&constraint.base));
            }
        }

        // Add additional constraints
        for (self.additional_constraints.items) |*c| {
            const body1 = bodies[@intCast(c.body_idx[0])];
            const body2 = bodies[@intCast(c.body_idx[1])];
            const settings = c.getConstraint() orelse @panic("RagdollSettings.createRagdoll: an additional constraint without settings (Jolt dereferences a null pointer)");
            const constraint = try settings.createConstraint(self.allocator, body1, body2);
            r.constraints.appendAssumeCapacity(.init(&constraint.base));
        }

        return r;
    }

    /// Access to the skeleton of this ragdoll (GetSkeleton() const)
    pub fn getSkeleton(self: *const RagdollSettings) ?*const Skeleton {
        return self.skeleton.get();
    }

    /// Access to the skeleton of this ragdoll (the non-const GetSkeleton)
    pub fn getSkeletonMut(self: *RagdollSettings) ?*Skeleton {
        return self.skeleton.get();
    }

    /// Calculate the map needed for GetBodyIndexToConstraintIndex()
    /// (Zolt: Allocator.Error, the old table is kept then)
    pub fn calculateBodyIndexToConstraintIndex(self: *RagdollSettings) Allocator.Error!void {
        // Zolt: reserve before clearing, so that a failure keeps the old table
        try self.body_index_to_constraint_index.ensureTotalCapacity(self.allocator, self.parts.items.len);
        self.body_index_to_constraint_index.clearRetainingCapacity();

        var constraint_index: i32 = 0;
        for (self.parts.items) |*p| {
            if (p.to_parent.get() != null) {
                self.body_index_to_constraint_index.appendAssumeCapacity(constraint_index);
                constraint_index += 1;
            } else self.body_index_to_constraint_index.appendAssumeCapacity(-1);
        }
    }

    /// Get table that maps a body index to the constraint index with which it is connected to its parent. -1 if there is no constraint associated with the body.
    /// Note that this will only tell you which constraint connects the body to its parent, it will not look in the additional constraint list.
    pub fn getBodyIndexToConstraintIndex(self: *const RagdollSettings) []const i32 {
        return self.body_index_to_constraint_index.items;
    }

    /// Map a single body index to a constraint index
    pub fn getConstraintIndexForBodyIndex(self: *const RagdollSettings, body_index: usize) i32 {
        return self.body_index_to_constraint_index.items[body_index];
    }

    /// Calculate the map needed for GetConstraintIndexToBodyIdxPair()
    /// (Zolt: Allocator.Error, the old table is kept then)
    pub fn calculateConstraintIndexToBodyIdxPair(self: *RagdollSettings) Allocator.Error!void {
        // Zolt: reserve before clearing, so that a failure keeps the old table
        try self.constraint_index_to_body_idx_pair.ensureTotalCapacity(self.allocator, self.parts.items.len + self.additional_constraints.items.len);
        self.constraint_index_to_body_idx_pair.clearRetainingCapacity();

        // Add constraints between parts
        for (self.parts.items, 0..) |*p, joint_idx| {
            if (p.to_parent.get() != null) {
                const parent_joint_idx = self.skeletonOf().getJoint(joint_idx).parent_joint_index;
                self.constraint_index_to_body_idx_pair.appendAssumeCapacity(.{ .first = parent_joint_idx, .second = @intCast(joint_idx) });
            }
        }

        // Add additional constraints
        for (self.additional_constraints.items) |*c|
            self.constraint_index_to_body_idx_pair.appendAssumeCapacity(.{ .first = c.body_idx[0], .second = c.body_idx[1] });
    }

    /// Table that maps a constraint index (index in mConstraints) to the indices of the bodies that the constraint is connected to (index in mBodyIDs)
    pub fn getConstraintIndexToBodyIdxPair(self: *const RagdollSettings) []const BodyIdxPair {
        return self.constraint_index_to_body_idx_pair.items;
    }

    /// Map a single constraint index (index in mConstraints) to the indices of the bodies that the constraint is connected to (index in mBodyIDs)
    pub fn getBodyIndicesForConstraintIndex(self: *const RagdollSettings, constraint_index: usize) BodyIdxPair {
        return self.constraint_index_to_body_idx_pair.items[constraint_index];
    }
};

/// Runtime ragdoll information
pub const Ragdoll = struct {
    /// The position and rotation of the root of the ragdoll (GetRootTransform's out parameters)
    pub const RootTransform = struct {
        position: RVec3,
        rotation: Quat,
    };

    /// RefTarget<Ragdoll>
    ref_count: RefCount = .{},
    /// Allocates the arrays, frees the ragdoll, allocates the temporary arrays of the functions
    allocator: Allocator,

    /// The settings that created this ragdoll (private in Jolt, use getRagdollSettings)
    ragdoll_settings: RefConst(RagdollSettings) = .empty,

    /// The bodies and constraints that this ragdoll consists of (1-on-1 with mRagdollSettings->mParts) (private in Jolt,
    /// use getBodyIDs)
    body_ids: std.ArrayList(BodyID) = .empty,

    /// Array of constraints that connect the bodies together: TwoBodyConstraints (Jolt's `Array<Ref<TwoBodyConstraint>>`,
    /// private in Jolt, use getConstraint)
    constraints: std.ArrayList(Ref(Constraint)) = .empty,

    /// Cached physics system
    system: *PhysicsSystem,

    /// Constructor Ragdoll(PhysicsSystem *inSystem) of a ragdoll that is not on the heap (`deinit` it)
    pub fn init(allocator: Allocator, system: *PhysicsSystem) Ragdoll {
        return .{ .allocator = allocator, .system = system };
    }

    /// new Ragdoll(inSystem): reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator, system: *PhysicsSystem) Allocator.Error!*Ragdoll {
        const self = try allocator.create(Ragdoll);
        self.* = .init(allocator, system);
        return self;
    }

    /// Destructor: destroys the bodies (the ragdoll must have been removed from the system), releases the constraints
    /// and the settings
    pub fn deinit(self: *Ragdoll) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *Ragdoll) void {
        // Destroy all bodies
        self.system.getBodyInterfaceMut().destroyBodies(self.body_ids.items);

        // The members, in the reverse order of their declaration
        for (self.constraints.items) |*c| c.deinit();
        self.constraints.deinit(self.allocator);
        self.body_ids.deinit(self.allocator);
        self.ragdoll_settings.deinit();
    }

    /// delete this of a ragdoll without references (RagdollSettings.createRagdoll)
    fn destroy(self: *Ragdoll) void {
        self.destruct();
        self.allocator.destroy(self);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const Ragdoll) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const Ragdoll) void {
        if (self.ref_count.release())
            @constCast(self).destroy(); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this ragdoll as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const Ragdoll) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const Ragdoll) u32 {
        return self.ref_count.get();
    }

    /// The constraints as Jolt passes them to AddConstraints / RemoveConstraints (`(Constraint **)mConstraints.data()`),
    /// allocated with the ragdoll's allocator
    fn constraintList(self: *const Ragdoll) Allocator.Error![]*Constraint {
        const list = try self.allocator.alloc(*Constraint, self.constraints.items.len);
        for (list, self.constraints.items) |*dst, c|
            dst.* = c.get().?;
        return list;
    }

    /// Add bodies and constraints to the system and optionally activate the bodies
    /// (Zolt: Allocator.Error, nothing is added then; see the file header for the order)
    pub fn addToPhysicsSystem(self: *Ragdoll, activation_mode: Activation, opts: struct { lock_bodies: bool = true }) Allocator.Error!void {
        // Zolt: the list of constraints for AddConstraints
        const constraints = try self.constraintList();
        defer self.allocator.free(constraints);

        // Scope for JPH_STACK_ALLOC
        {
            // Create copy of body ids since they will be shuffled
            const bodies = try self.allocator.dupe(BodyID, self.body_ids.items);
            defer self.allocator.free(bodies);

            // Insert bodies as a batch
            const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
            const add_state = try bi.addBodiesPrepare(bodies);

            // Add all constraints (Zolt: before AddBodiesFinalize, so that nothing is added when it fails)
            self.system.addConstraints(constraints) catch |err| {
                bi.addBodiesAbort(bodies, add_state);
                return err;
            };

            bi.addBodiesFinalize(bodies, add_state, activation_mode);
        }
    }

    /// Remove bodies and constraints from the system
    /// (Zolt: Allocator.Error, nothing is removed then)
    pub fn removeFromPhysicsSystem(self: *Ragdoll, opts: struct { lock_bodies: bool = true }) Allocator.Error!void {
        // Zolt: the list of constraints for RemoveConstraints and the copy of the body IDs (Jolt: JPH_STACK_ALLOC after
        // removing the constraints), so that a failure happens before anything is removed
        const constraints = try self.constraintList();
        defer self.allocator.free(constraints);
        const bodies = try self.allocator.dupe(BodyID, self.body_ids.items);
        defer self.allocator.free(bodies);

        // Remove all constraints before removing the bodies
        self.system.removeConstraints(constraints);

        // Scope for JPH_STACK_ALLOC
        {
            // Create copy of body ids since they will be shuffled (done above)

            // Remove all bodies as a batch
            ragdollGetBodyInterface(self.system, opts.lock_bodies).removeBodies(bodies);
        }
    }

    /// Wake up all bodies in the ragdoll
    pub fn activate(self: *Ragdoll, opts: struct { lock_bodies: bool = true }) void {
        ragdollGetBodyInterface(self.system, opts.lock_bodies).activateBodies(self.body_ids.items);
    }

    /// Check if one or more of the bodies in the ragdoll are active.
    /// Note that this involves locking the bodies (if inLockBodies is true) and looping over them. An alternative and possibly faster
    /// way could be to install a BodyActivationListener and count the number of active bodies of a ragdoll as they're activated / deactivated
    /// (basically check if the body that activates / deactivates is in GetBodyIDs() and increment / decrement a counter).
    pub fn isActive(self: *const Ragdoll, opts: struct { lock_bodies: bool = true }) bool {
        // Lock the bodies
        const body_count = self.body_ids.items.len;
        var lock: BodyLockMultiRead = .init(ragdollGetBodyLockInterface(self.system, opts.lock_bodies), self.body_ids.items);
        defer lock.deinit();

        // Test if any body is active
        for (0..body_count) |b| {
            const body = lockedBody(BodyLockMultiRead, &lock, b);
            if (body.isActive())
                return true;
        }

        return false;
    }

    /// `lock.GetBody(b)`, which Jolt dereferences without a check
    fn lockedBody(comptime Lock: type, lock: *const Lock, index: usize) Lock.BodyPtr {
        return lock.getBody(@intCast(index)) orelse @panic("Ragdoll: a body of the ragdoll cannot be locked (Jolt dereferences null)");
    }

    /// Set the group ID on all bodies in the ragdoll
    pub fn setGroupID(self: *Ragdoll, group_id: CollisionGroup.GroupID, opts: struct { lock_bodies: bool = true }) void {
        // Lock the bodies
        const body_count = self.body_ids.items.len;
        var lock: BodyLockMultiWrite = .init(ragdollGetBodyLockInterface(self.system, opts.lock_bodies), self.body_ids.items);
        defer lock.deinit();

        // Update group ID
        for (0..body_count) |b| {
            const body = lockedBody(BodyLockMultiWrite, &lock, b);
            body.getCollisionGroupMut().setGroupID(group_id);
        }
    }

    /// `mRagdollSettings->`: Jolt dereferences the settings without a check (a ragdoll made with the public constructor
    /// has none)
    fn settingsOf(self: *const Ragdoll) *const RagdollSettings {
        return self.ragdoll_settings.get() orelse @panic("Ragdoll: no ragdoll settings (Jolt dereferences a null mRagdollSettings)");
    }

    /// JPH_ASSERT(inPose.GetSkeleton() == mRagdollSettings->mSkeleton)
    fn checkPoseSkeleton(self: *const Ragdoll, pose: *const SkeletonPose) void {
        if (Core.enable_asserts)
            std.debug.assert(pose.getSkeleton() == self.settingsOf().skeleton.get());
    }

    /// Set the ragdoll to a pose (calls BodyInterface::SetPositionAndRotation to instantly move the ragdoll)
    pub fn setPose(self: *Ragdoll, pose: *const SkeletonPose, opts: struct { lock_bodies: bool = true }) void {
        self.checkPoseSkeleton(pose);

        self.setPoseMatrices(pose.getRootOffset(), pose.getJointMatrices(), .{ .lock_bodies = opts.lock_bodies });
    }

    /// Lower level version of SetPose that directly takes the world space joint matrices
    pub fn setPoseMatrices(self: *Ragdoll, root_offset: RVec3, joint_matrices: []const Mat44, opts: struct { lock_bodies: bool = true }) void {
        // Move bodies instantly into the correct position
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items, 0..) |body_id, i| {
            const joint = joint_matrices[i];
            bi.setPositionAndRotation(body_id, root_offset.addVec3(joint.getTranslation()), joint.getQuaternion(), .dont_activate);
        }
    }

    /// Get the ragdoll pose (uses the world transform of the bodies to calculate the pose)
    pub fn getPose(self: *Ragdoll, out_pose: *SkeletonPose, opts: struct { lock_bodies: bool = true }) void {
        self.checkPoseSkeleton(out_pose);

        var root_offset = RVec3.zero(); // Jolt: uninitialized, see the file header
        self.getPoseMatrices(&root_offset, out_pose.getJointMatricesMut().items, .{ .lock_bodies = opts.lock_bodies });
        out_pose.setRootOffset(root_offset);
    }

    /// Lower level version of GetPose that directly returns the world space joint matrices
    /// (`out_root_offset` is not written for a ragdoll without bodies)
    pub fn getPoseMatrices(self: *Ragdoll, out_root_offset: *RVec3, out_joint_matrices: []Mat44, opts: struct { lock_bodies: bool = true }) void {
        // Lock the bodies
        const body_count = self.body_ids.items.len;
        if (body_count == 0)
            return;
        var lock: BodyLockMultiRead = .init(ragdollGetBodyLockInterface(self.system, opts.lock_bodies), self.body_ids.items);
        defer lock.deinit();

        // Get root matrix
        const root = lockedBody(BodyLockMultiRead, &lock, 0);
        const root_transform = root.getWorldTransform();
        out_root_offset.* = root_transform.getTranslation();
        out_joint_matrices[0] = Mat44.init(root_transform.getColumn4(0), root_transform.getColumn4(1), root_transform.getColumn4(2), Vec4.init(0, 0, 0, 1));

        // Get other matrices
        for (1..body_count) |b| {
            const body = lockedBody(BodyLockMultiRead, &lock, b);
            const transform = body.getWorldTransform();
            out_joint_matrices[b] = Mat44.init(transform.getColumn4(0), transform.getColumn4(1), transform.getColumn4(2), Vec4.fromVec3W(transform.getTranslation().sub(out_root_offset.*).toVec3(), 1));
        }
    }

    /// This function calls ResetWarmStart on all constraints. It can be used after calling SetPose to reset previous frames impulses. See: Constraint::ResetWarmStart.
    pub fn resetWarmStart(self: *Ragdoll) void {
        for (self.constraints.items) |c|
            c.get().?.resetWarmStart();
    }

    /// Drive the ragdoll to a specific pose by setting velocities on each of the bodies so that it will reach inPose in inDeltaTime
    pub fn driveToPoseUsingKinematics(self: *Ragdoll, pose: *const SkeletonPose, delta_time: f32, opts: struct { lock_bodies: bool = true }) void {
        self.checkPoseSkeleton(pose);

        self.driveToPoseUsingKinematicsMatrices(pose.getRootOffset(), pose.getJointMatrices(), delta_time, .{ .lock_bodies = opts.lock_bodies });
    }

    /// Lower level version of DriveToPoseUsingKinematics that directly takes the world space joint matrices
    pub fn driveToPoseUsingKinematicsMatrices(self: *Ragdoll, root_offset: RVec3, joint_matrices: []const Mat44, delta_time: f32, opts: struct { lock_bodies: bool = true }) void {
        // Move bodies into the correct position using kinematics
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items, 0..) |body_id, i| {
            const joint = joint_matrices[i];
            bi.moveKinematic(body_id, root_offset.addVec3(joint.getTranslation()), joint.getQuaternion(), delta_time);
        }
    }

    /// Drive the ragdoll to a specific pose by activating the motors on each constraint.
    pub fn driveToPoseUsingMotors(self: *Ragdoll, pose: *const SkeletonPose) void {
        self.checkPoseSkeleton(pose);

        // Move bodies into the correct position using constraints
        for (0..pose.getJointMatrices().len) |i| {
            const constraint_idx = self.settingsOf().getConstraintIndexForBodyIndex(i);
            if (constraint_idx >= 0) {
                // Get desired rotation of this body relative to its parent
                const joint_state = pose.getJoint(i);

                // Drive constraint to target
                const constraint = self.getConstraintMut(@intCast(constraint_idx));
                const sub_type = constraint.base.getSubType();
                if (sub_type == .swing_twist) {
                    const st_constraint = virtual.downcast(SwingTwistConstraint, constraint);
                    st_constraint.setSwingMotorState(.position);
                    st_constraint.setTwistMotorState(.position);
                    st_constraint.setTargetOrientationBS(joint_state.rotation);
                } else if (sub_type == .hinge) {
                    const h_constraint = virtual.downcast(HingeConstraint, constraint);
                    h_constraint.setMotorState(.position);
                    h_constraint.setTargetOrientationBS(joint_state.rotation);
                } else if (Core.enable_asserts) @panic("Constraint type not implemented!");
            }
        }
    }

    /// Drive the ragdoll to a specific pose by activating the motors on each constraint. Drives both to target position and velocity.
    /// @param prev_pose The previous pose of the ragdoll, used to calculate the target velocity for each motor
    /// @param pose The target pose of the ragdoll, used to calculate the target position for each motor
    /// @param delta_time The time between inPrevPose and inPose, used to calculate the target velocity for each motor
    /// (Jolt: the DriveToPoseUsingMotors overload with a previous pose)
    pub fn driveToPoseUsingMotorsWithVelocity(self: *Ragdoll, prev_pose: *const SkeletonPose, pose: *const SkeletonPose, delta_time: f32) void {
        self.checkPoseSkeleton(prev_pose);
        self.checkPoseSkeleton(pose);
        if (Core.enable_asserts) std.debug.assert(delta_time > 0.0);

        // Move bodies into the correct position using constraints
        for (0..pose.getJointMatrices().len) |i| {
            const constraint_idx = self.settingsOf().getConstraintIndexForBodyIndex(i);
            if (constraint_idx >= 0) {
                // Get desired rotation of this body relative to its parent
                const prev_joint_state = prev_pose.getJoint(i);
                const joint_state = pose.getJoint(i);

                // Calculate the angular velocity needed to get from the previous pose to the current pose in the given delta time in body 1 space
                const angular_velocity = joint_state.rotation.mul(prev_joint_state.rotation.conjugated()).getAngularVelocity(delta_time);

                // Drive constraint to target
                const constraint = self.getConstraintMut(@intCast(constraint_idx));
                const sub_type = constraint.base.getSubType();
                if (sub_type == .swing_twist) {
                    const st_constraint = virtual.downcast(SwingTwistConstraint, constraint);
                    st_constraint.setSwingMotorState(.position_and_velocity);
                    st_constraint.setTwistMotorState(.position_and_velocity);
                    const body1_to_body2 = constraint.getBody2().getRotation().conjugated().mul(constraint.getBody1().getRotation());
                    st_constraint.setTargetAngularVelocityBS(body1_to_body2.mulVec3(angular_velocity));
                    st_constraint.setTargetOrientationBS(joint_state.rotation);
                } else if (sub_type == .hinge) {
                    const h_constraint = virtual.downcast(HingeConstraint, constraint);
                    h_constraint.setMotorState(.position_and_velocity);
                    h_constraint.setTargetAngularVelocity(h_constraint.getLocalSpaceHingeAxis1().dot(angular_velocity));
                    h_constraint.setTargetOrientationBS(joint_state.rotation);
                } else if (Core.enable_asserts) @panic("Constraint type not implemented!");
            }
        }
    }

    /// Control the linear and velocity of all bodies in the ragdoll
    pub fn setLinearAndAngularVelocity(self: *Ragdoll, linear_velocity: Vec3, angular_velocity: Vec3, opts: struct { lock_bodies: bool = true }) void {
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items) |body_id|
            bi.setLinearAndAngularVelocity(body_id, linear_velocity, angular_velocity);
    }

    /// Set the world space linear velocity of all bodies in the ragdoll.
    pub fn setLinearVelocity(self: *Ragdoll, linear_velocity: Vec3, opts: struct { lock_bodies: bool = true }) void {
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items) |body_id|
            bi.setLinearVelocity(body_id, linear_velocity);
    }

    /// Add a world space velocity (in m/s) to all bodies in the ragdoll.
    pub fn addLinearVelocity(self: *Ragdoll, linear_velocity: Vec3, opts: struct { lock_bodies: bool = true }) void {
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items) |body_id|
            bi.addLinearVelocity(body_id, linear_velocity);
    }

    /// Add impulse to all bodies of the ragdoll (center of mass of each of them)
    pub fn addImpulse(self: *Ragdoll, impulse: Vec3, opts: struct { lock_bodies: bool = true }) void {
        const bi = ragdollGetBodyInterface(self.system, opts.lock_bodies);
        for (self.body_ids.items) |body_id|
            bi.addImpulse(body_id, impulse);
    }

    /// Get the position and orientation of the root of the ragdoll
    pub fn getRootTransform(self: *const Ragdoll, opts: struct { lock_bodies: bool = true }) RootTransform {
        var lock: BodyLockRead = .init(ragdollGetBodyLockInterface(self.system, opts.lock_bodies), self.body_ids.items[0]);
        defer lock.deinit();
        if (lock.succeeded()) {
            const body = lock.getBody();
            return .{ .position = body.getPosition(), .rotation = body.getRotation() };
        } else {
            return .{ .position = RVec3.zero(), .rotation = Quat.identity() };
        }
    }

    /// Get number of bodies in the ragdoll
    pub fn getBodyCount(self: *const Ragdoll) usize {
        return self.body_ids.items.len;
    }

    /// Access a body ID
    pub fn getBodyID(self: *const Ragdoll, body_index: usize) BodyID {
        return self.body_ids.items[body_index];
    }

    /// Access to the array of body IDs
    pub fn getBodyIDs(self: *const Ragdoll) []const BodyID {
        return self.body_ids.items;
    }

    /// Get number of constraints in the ragdoll
    pub fn getConstraintCount(self: *const Ragdoll) usize {
        return self.constraints.items.len;
    }

    /// Access a constraint by index (the non-const GetConstraint)
    pub fn getConstraintMut(self: *Ragdoll, constraint_index: usize) *TwoBodyConstraint {
        return virtual.downcast(TwoBodyConstraint, self.constraints.items[constraint_index].get().?);
    }

    /// Access a constraint by index (GetConstraint() const)
    pub fn getConstraint(self: *const Ragdoll, constraint_index: usize) *const TwoBodyConstraint {
        const constraint: *const Constraint = self.constraints.items[constraint_index].get().?;
        return virtual.downcast(TwoBodyConstraint, constraint);
    }

    /// Get world space bounding box for all bodies of the ragdoll
    pub fn getWorldSpaceBounds(self: *const Ragdoll, opts: struct { lock_bodies: bool = true }) AABox {
        // Lock the bodies
        const body_count = self.body_ids.items.len;
        var lock: BodyLockMultiRead = .init(ragdollGetBodyLockInterface(self.system, opts.lock_bodies), self.body_ids.items);
        defer lock.deinit();

        // Encapsulate all bodies
        var bounds: AABox = .empty;
        for (0..body_count) |b| {
            if (lock.getBody(@intCast(b))) |body|
                bounds.encapsulate(body.getWorldSpaceBounds().*);
        }
        return bounds;
    }

    /// Get the settings object that created this ragdoll
    pub fn getRagdollSettings(self: *const Ragdoll) ?*const RagdollSettings {
        return self.ragdoll_settings.get();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests: procedurally built humanoids (Jolt has no Ragdoll unit tests; the parity of the numbers is in
// ZoltParity/Physics/RagdollParity.zig and in the ragdoll scenes of the simulation parity)

const testing = std.testing;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const TempAllocatorImpl = @import("../../Core/TempAllocator.zig").TempAllocatorImpl;
const JobSystemSingleThreaded = @import("../../Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
const PhysicsSettingsFile = @import("../PhysicsSettings.zig");
const Real = @import("../../Math/Real.zig").Real;
const ShapeFile = @import("../Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const SphereShapeSettings = @import("../Collision/Shape/SphereShape.zig").SphereShapeSettings;
const CapsuleShape = @import("../Collision/Shape/CapsuleShape.zig").CapsuleShape;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const SwingTwistConstraintSettings = @import("../Constraints/SwingTwistConstraint.zig").SwingTwistConstraintSettings;
const HingeConstraintSettings = @import("../Constraints/HingeConstraint.zig").HingeConstraintSettings;
const SixDOFConstraintSettings = @import("../Constraints/SixDOFConstraint.zig").SixDOFConstraintSettings;
const FixedConstraintSettings = @import("../Constraints/FixedConstraint.zig").FixedConstraintSettings;
const DistanceConstraintSettings = @import("../Constraints/DistanceConstraint.zig").DistanceConstraintSettings;
const TestConstraintSettings = @import("../Constraints/TestConstraints.zig").TestConstraintSettings;

/// A physics system with a static layer and a moving layer, a temp allocator and a single threaded job system (in
/// place: it must not move)
const TestWorld = struct {
    const non_moving: ObjectLayer = 0;
    const moving: ObjectLayer = 1;
    const delta_time: f32 = 1.0 / 60.0;

    allocator: Allocator,
    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    temp_allocator: TempAllocatorImpl,
    job_system: JobSystemSingleThreaded,
    system: PhysicsSystem,

    fn init(self: *TestWorld, allocator: Allocator, max_bodies: u32) !void {
        self.allocator = allocator;
        self.layer_interface = try .init(allocator, 2, 2);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(non_moving, .init(0));
        self.layer_interface.mapObjectToBroadPhaseLayer(moving, .init(1));
        self.pair_filter = try .init(allocator, 2);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(non_moving, moving);
        self.pair_filter.enableCollision(moving, moving);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 2, &self.pair_filter.base, 2);
        errdefer self.ovb_filter.deinit();
        self.temp_allocator = try .init(allocator, 1024 * 1024);
        errdefer self.temp_allocator.deinit();
        self.job_system = try .init(allocator, testing.io, @intCast(PhysicsSettingsFile.max_physics_jobs));
        errdefer self.job_system.deinit();
        self.system = .empty;
        try self.system.init(allocator, testing.io, max_bodies, 0, 1024, 1024, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
    }

    fn deinit(self: *TestWorld) void {
        self.system.deinit();
        self.job_system.deinit();
        self.temp_allocator.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// A static floor whose top is at y = 0
    fn addFloor(self: *TestWorld) !BodyID {
        const box = try BoxShape.create(self.allocator, Vec3.init(20, 1, 20), .{ .convex_radius = 0 });
        var settings: BodyCreationSettings = .initWithShape(box.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, non_moving);
        defer settings.deinit();
        return self.system.getBodyInterfaceMut().createAndAddBody(&settings, .dont_activate);
    }

    fn update(self: *TestWorld, count: u32) !void {
        for (0..count) |_| {
            const errors = try self.system.update(delta_time, 1, self.temp_allocator.tempAllocator(), self.job_system.jobSystem());
            try testing.expect(errors.isNone());
        }
    }

    fn bodyInterface(self: *TestWorld) *BodyInterface {
        return self.system.getBodyInterfaceMut();
    }
};

/// The constraint that connects a part of the test humanoid to its parent
const JointKind = enum { none, swing_twist, hinge, six_dof, fixed };

/// A joint of the test humanoid: the part (its shape and position) and the joint to its parent (position, kind; the
/// mixed humanoid uses `mixed_kind`)
const TestJoint = struct {
    name: []const u8,
    parent: i32,
    shape: u32,
    position: [3]f32,
    joint: [3]f32,
    kind: JointKind,
    mixed_kind: JointKind,
};

/// A standing humanoid with the pelvis at y = 1 whose lower legs end 0.08 above y = 0. The parts do not touch: every
/// joint has a small gap, the lower arms are 0.055 from the pelvis and 0.045 from the upper legs
const humanoid_joints = [_]TestJoint{
    .{ .name = "Pelvis", .parent = -1, .shape = 0, .position = .{ 0, 1.0, 0 }, .joint = .{ 0, 1.0, 0 }, .kind = .none, .mixed_kind = .none },
    .{ .name = "Spine", .parent = 0, .shape = 1, .position = .{ 0, 1.3, 0 }, .joint = .{ 0, 1.12, 0 }, .kind = .swing_twist, .mixed_kind = .swing_twist },
    .{ .name = "Head", .parent = 1, .shape = 2, .position = .{ 0, 1.62, 0 }, .joint = .{ 0, 1.48, 0 }, .kind = .swing_twist, .mixed_kind = .six_dof },
    .{ .name = "UpperArmL", .parent = 1, .shape = 3, .position = .{ -0.25, 1.25, 0 }, .joint = .{ -0.25, 1.42, 0 }, .kind = .swing_twist, .mixed_kind = .swing_twist },
    .{ .name = "LowerArmL", .parent = 3, .shape = 4, .position = .{ -0.25, 0.93, 0 }, .joint = .{ -0.25, 1.09, 0 }, .kind = .hinge, .mixed_kind = .hinge },
    .{ .name = "UpperArmR", .parent = 1, .shape = 3, .position = .{ 0.25, 1.25, 0 }, .joint = .{ 0.25, 1.42, 0 }, .kind = .swing_twist, .mixed_kind = .swing_twist },
    .{ .name = "LowerArmR", .parent = 5, .shape = 4, .position = .{ 0.25, 0.93, 0 }, .joint = .{ 0.25, 1.09, 0 }, .kind = .hinge, .mixed_kind = .fixed },
    .{ .name = "UpperLegL", .parent = 0, .shape = 5, .position = .{ -0.1, 0.65, 0 }, .joint = .{ -0.1, 0.86, 0 }, .kind = .swing_twist, .mixed_kind = .swing_twist },
    .{ .name = "LowerLegL", .parent = 7, .shape = 6, .position = .{ -0.1, 0.25, 0 }, .joint = .{ -0.1, 0.45, 0 }, .kind = .hinge, .mixed_kind = .hinge },
    .{ .name = "UpperLegR", .parent = 0, .shape = 5, .position = .{ 0.1, 0.65, 0 }, .joint = .{ 0.1, 0.86, 0 }, .kind = .swing_twist, .mixed_kind = .swing_twist },
    .{ .name = "LowerLegR", .parent = 9, .shape = 6, .position = .{ 0.1, 0.25, 0 }, .joint = .{ 0.1, 0.45, 0 }, .kind = .hinge, .mixed_kind = .hinge },
};

const num_humanoid_joints = humanoid_joints.len;

/// The shapes of the test humanoid (reference count 0)
fn makeHumanoidShape(allocator: Allocator, index: u32) !*const Shape {
    return switch (index) {
        0 => (try BoxShape.create(allocator, Vec3.init(0.15, 0.1, 0.1), .{ .convex_radius = 0.05 })).asShape(), // Pelvis
        1 => (try BoxShape.create(allocator, Vec3.init(0.12, 0.15, 0.08), .{ .convex_radius = 0.05 })).asShape(), // Spine
        2 => (try SphereShape.create(allocator, 0.1, .{})).asShape(), // Head
        3 => (try CapsuleShape.create(allocator, 0.1, 0.05, .{})).asShape(), // Upper arm
        4 => (try CapsuleShape.create(allocator, 0.1, 0.045, .{})).asShape(), // Lower arm
        5 => (try CapsuleShape.create(allocator, 0.12, 0.06, .{})).asShape(), // Upper leg
        else => (try CapsuleShape.create(allocator, 0.12, 0.05, .{})).asShape(), // Lower leg
    };
}

/// The constraint of a joint (reference count 0), null for `.none`
fn makeJointConstraint(allocator: Allocator, kind: JointKind, joint: RVec3) !?*TwoBodyConstraintSettings {
    switch (kind) {
        .none => return null,
        .swing_twist => {
            const s = try SwingTwistConstraintSettings.create(allocator);
            s.position1 = joint;
            s.position2 = joint;
            s.twist_axis1 = Vec3.axisY();
            s.twist_axis2 = Vec3.axisY();
            s.plane_axis1 = Vec3.axisX();
            s.plane_axis2 = Vec3.axisX();
            s.normal_half_cone_angle = 0.6;
            s.plane_half_cone_angle = 0.4;
            s.twist_min_angle = -0.3;
            s.twist_max_angle = 0.3;
            return s.asTwoBodySettings();
        },
        .hinge => {
            const s = try HingeConstraintSettings.create(allocator);
            s.point1 = joint;
            s.point2 = joint;
            s.hinge_axis1 = Vec3.axisX();
            s.hinge_axis2 = Vec3.axisX();
            s.normal_axis1 = Vec3.axisY();
            s.normal_axis2 = Vec3.axisY();
            s.limits_min = -2.0;
            s.limits_max = 0.1;
            return s.asTwoBodySettings();
        },
        .six_dof => {
            const s = try SixDOFConstraintSettings.create(allocator);
            s.position1 = joint;
            s.position2 = joint;
            s.axis_x1 = Vec3.axisY();
            s.axis_x2 = Vec3.axisY();
            s.axis_y1 = Vec3.axisX();
            s.axis_y2 = Vec3.axisX();
            s.makeFixedAxis(.translation_x);
            s.makeFixedAxis(.translation_y);
            s.makeFixedAxis(.translation_z);
            s.setLimitedAxis(.rotation_x, -0.2, 0.2);
            s.setLimitedAxis(.rotation_y, -0.4, 0.4);
            s.setLimitedAxis(.rotation_z, -0.4, 0.4);
            return s.asTwoBodySettings();
        },
        .fixed => {
            const s = try FixedConstraintSettings.create(allocator);
            s.point1 = joint;
            s.point2 = joint;
            return s.asTwoBodySettings();
        },
    }
}

const HumanoidOptions = struct {
    /// SixDOF at the neck, Fixed at the right elbow and a distance constraint between the lower arms (additional
    /// constraint); else SwingTwist and Hinge constraints only (DriveToPoseUsingMotors drives these)
    mixed: bool = false,
    /// The motion type of the pelvis and of the other parts
    pelvis_motion_type: MotionType = .dynamic,
    motion_type: MotionType = .dynamic,
    /// Added to every position
    origin: [3]Real = .{ 0, 0, 0 },
};

/// The test humanoid (reference count 0): skeleton, parts and constraints; the maps are not calculated
fn buildHumanoid(allocator: Allocator, opts: HumanoidOptions) !*RagdollSettings {
    const settings = try RagdollSettings.create(allocator);
    errdefer destroyUnreferenced(settings);

    const skeleton = try Skeleton.create(allocator);
    settings.skeleton.set(skeleton);
    for (humanoid_joints) |j|
        _ = try skeleton.addJointWithParentIndex(j.name, j.parent);

    var shapes: [7]RefConst(Shape) = @splat(.empty);
    defer for (&shapes) |*s| s.deinit();
    for (&shapes, 0..) |*s, i|
        s.* = .init(try makeHumanoidShape(allocator, @intCast(i)));

    const origin = RVec3.init(opts.origin[0], opts.origin[1], opts.origin[2]);
    try settings.parts.ensureTotalCapacity(allocator, num_humanoid_joints);
    for (humanoid_joints, 0..) |j, i| {
        const position = origin.addVec3(Vec3.init(j.position[0], j.position[1], j.position[2]));
        var body: BodyCreationSettings = .initWithShape(shapes[j.shape].get(), position, Quat.identity(), if (i == 0) opts.pelvis_motion_type else opts.motion_type, TestWorld.moving);
        defer body.deinit();
        body.linear_damping = 0.2;
        body.angular_damping = 0.4;
        const constraint = try makeJointConstraint(allocator, if (opts.mixed) j.mixed_kind else j.kind, origin.addVec3(Vec3.init(j.joint[0], j.joint[1], j.joint[2])));
        var constraint_ref = Ref(ConstraintSettings).init(if (constraint) |c| &c.base else null);
        defer constraint_ref.deinit();
        settings.parts.appendAssumeCapacity(.init(&body, constraint));
    }

    if (opts.mixed) {
        // A distance constraint between the lower arms
        const distance = try DistanceConstraintSettings.create(allocator);
        var distance_ref = Ref(ConstraintSettings).init(distance.asSettings());
        defer distance_ref.deinit();
        distance.point1 = origin.addVec3(Vec3.init(-0.25, 0.85, 0));
        distance.point2 = origin.addVec3(Vec3.init(0.25, 0.85, 0));
        distance.min_distance = 0.4;
        distance.max_distance = 0.6;
        try settings.additional_constraints.ensureUnusedCapacity(allocator, 1);
        settings.additional_constraints.appendAssumeCapacity(.init(4, 6, distance.asTwoBodySettings()));
    }

    return settings;
}

/// Destroy settings that have no reference (a reference is taken and released)
fn destroyUnreferenced(settings: *RagdollSettings) void {
    var ref = Ref(RagdollSettings).init(settings);
    ref.deinit();
}

/// The test humanoid with its maps (one reference owned by the caller)
fn buildPreparedHumanoid(allocator: Allocator, opts: HumanoidOptions) !Ref(RagdollSettings) {
    var settings = Ref(RagdollSettings).init(try buildHumanoid(allocator, opts));
    errdefer settings.deinit();
    try settings.get().?.calculateBodyIndexToConstraintIndex();
    try settings.get().?.calculateConstraintIndexToBodyIdxPair();
    return settings;
}

/// The model space joint matrices of the bind pose of the test humanoid (the positions of the parts)
fn bindPoseMatrices() [num_humanoid_joints]Mat44 {
    var matrices: [num_humanoid_joints]Mat44 = undefined;
    for (&matrices, humanoid_joints) |*m, j|
        m.* = Mat44.translation(Vec3.init(j.position[0], j.position[1], j.position[2]));
    return matrices;
}

fn saveSettings(settings: *const RagdollSettings, out: *std.Io.Writer.Allocating, save_shapes: bool, save_group_filter: bool) ![]const u8 {
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    try settings.saveBinaryState(wrapper.streamOut(), save_shapes, save_group_filter);
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return out.written();
}

fn restoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!RagdollSettings.RagdollResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return RagdollSettings.restoreFromBinaryState(allocator, in.streamIn());
}

test "RagdollSettings: Part, AdditionalConstraint and the reference counts" {
    const allocator = testing.allocator;
    const sphere = try SphereShape.create(allocator, 0.5, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    const hinge = try HingeConstraintSettings.create(allocator);
    var hinge_ref = Ref(ConstraintSettings).init(hinge.asSettings());
    defer hinge_ref.deinit();

    // A part references the shape and the constraint settings, its copy too
    var body: BodyCreationSettings = .initWithShape(sphere.asShape(), RVec3.init(1, 2, 3), Quat.identity(), .dynamic, 1);
    defer body.deinit();
    var part: RagdollSettings.Part = .init(&body, hinge.asTwoBodySettings());
    try testing.expectEqual(@as(u32, 3), sphere.asShape().getRefCount());
    try testing.expectEqual(@as(u32, 2), hinge.asSettings().getRefCount());
    try testing.expect(part.getToParent() == hinge.asTwoBodySettings());
    try testing.expect(part.base.position.eql(RVec3.init(1, 2, 3)));
    var copy = part.clone();
    try testing.expectEqual(@as(u32, 3), hinge.asSettings().getRefCount());
    try testing.expectEqual(@as(u32, 4), sphere.asShape().getRefCount());
    copy.setToParent(null);
    try testing.expect(copy.getToParent() == null);
    try testing.expectEqual(@as(u32, 2), hinge.asSettings().getRefCount());
    copy.deinit();
    part.deinit();
    try testing.expectEqual(@as(u32, 2), sphere.asShape().getRefCount());
    try testing.expectEqual(@as(u32, 1), hinge.asSettings().getRefCount());

    // An additional constraint
    var additional: RagdollSettings.AdditionalConstraint = .init(3, -1, hinge.asTwoBodySettings());
    try testing.expectEqual([2]i32{ 3, -1 }, additional.body_idx);
    try testing.expect(additional.getConstraint() == hinge.asTwoBodySettings());
    var additional_copy = additional.clone();
    try testing.expectEqual(@as(u32, 3), hinge.asSettings().getRefCount());
    additional_copy.deinit();
    additional.deinit();
    const empty: RagdollSettings.AdditionalConstraint = .{};
    try testing.expect(empty.getConstraint() == null);

    // The settings: a second reference keeps them alive, embedded settings are not destroyed by their last release
    var settings = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{}));
    defer settings.deinit();
    var second = settings.clone();
    try testing.expectEqual(@as(u32, 2), settings.get().?.getRefCount());
    second.deinit();
    try testing.expectEqual(@as(u32, 1), settings.get().?.getRefCount());
    try testing.expectEqual(num_humanoid_joints, settings.get().?.getSkeleton().?.getJointCount());
    try testing.expect(settings.get().?.getSkeletonMut().? == settings.get().?.skeleton.get().?);

    var embedded: RagdollSettings = .init(allocator);
    embedded.setEmbedded();
    {
        var ref = RefConst(RagdollSettings).init(&embedded);
        ref.deinit();
    }
    embedded.deinit();
}

test "RagdollSettings: calculateBodyIndexToConstraintIndex / calculateConstraintIndexToBodyIdxPair" {
    const allocator = testing.allocator;
    var settings_ref = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{ .mixed = true }));
    defer settings_ref.deinit();
    const settings = settings_ref.get().?;

    // Nothing calculated yet
    try testing.expectEqual(@as(usize, 0), settings.getBodyIndexToConstraintIndex().len);
    try testing.expectEqual(@as(usize, 0), settings.getConstraintIndexToBodyIdxPair().len);

    // The upper left arm loses its constraint: the parts after it shift their constraint indices
    settings.parts.items[3].setToParent(null);
    try settings.calculateBodyIndexToConstraintIndex();
    try settings.calculateConstraintIndexToBodyIdxPair();
    try testing.expectEqualSlices(i32, &.{ -1, 0, 1, -1, 2, 3, 4, 5, 6, 7, 8 }, settings.getBodyIndexToConstraintIndex());
    try testing.expectEqual(@as(i32, 2), settings.getConstraintIndexForBodyIndex(4));
    try testing.expectEqual(@as(i32, -1), settings.getConstraintIndexForBodyIndex(3));
    const expected_pairs = [_]RagdollSettings.BodyIdxPair{
        .{ .first = 0, .second = 1 }, .{ .first = 1, .second = 2 }, .{ .first = 3, .second = 4 },
        .{ .first = 1, .second = 5 }, .{ .first = 5, .second = 6 }, .{ .first = 0, .second = 7 },
        .{ .first = 7, .second = 8 }, .{ .first = 0, .second = 9 }, .{ .first = 9, .second = 10 },
        .{ .first = 4, .second = 6 }, // The additional constraint
    };
    try testing.expectEqualSlices(RagdollSettings.BodyIdxPair, &expected_pairs, settings.getConstraintIndexToBodyIdxPair());
    try testing.expectEqual(RagdollSettings.BodyIdxPair{ .first = 4, .second = 6 }, settings.getBodyIndicesForConstraintIndex(9));

    // A failed recalculation keeps the old tables (more parts and additional constraints than there is room for)
    try settings.parts.ensureUnusedCapacity(allocator, 100);
    for (0..100) |_| {
        _ = try settings.getSkeletonMut().?.addJointWithParentIndex("Hand", 4);
        settings.parts.appendAssumeCapacity(settings.parts.items[4].clone());
    }
    try settings.additional_constraints.ensureUnusedCapacity(allocator, 100);
    for (0..100) |_| settings.additional_constraints.appendAssumeCapacity(settings.additional_constraints.items[0].clone());
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    settings.allocator = failing.allocator();
    settings.parts.items[1].setToParent(null);
    try testing.expectError(error.OutOfMemory, settings.calculateBodyIndexToConstraintIndex());
    try testing.expectError(error.OutOfMemory, settings.calculateConstraintIndexToBodyIdxPair());
    settings.allocator = allocator;
    try testing.expectEqual(@as(i32, 0), settings.getConstraintIndexForBodyIndex(1));
    try testing.expectEqual(@as(usize, 11), settings.getBodyIndexToConstraintIndex().len);
    try testing.expectEqual(@as(usize, 10), settings.getConstraintIndexToBodyIdxPair().len);
    try settings.calculateBodyIndexToConstraintIndex();
    try settings.calculateConstraintIndexToBodyIdxPair();
    try testing.expectEqual(@as(i32, -1), settings.getConstraintIndexForBodyIndex(1));
    try testing.expectEqual(@as(usize, 111), settings.getBodyIndexToConstraintIndex().len);
    try testing.expectEqual(@as(usize, 8 + 100 + 101), settings.getConstraintIndexToBodyIdxPair().len); // Without the spine and upper left arm constraints
}

test "RagdollSettings: calculateConstraintPriorities" {
    const allocator = testing.allocator;
    var settings_ref = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{ .mixed = true }));
    defer settings_ref.deinit();
    const settings = settings_ref.get().?;

    // The leaves get the base priority, every joint towards the root one more (the maximum over its children)
    try settings.calculateConstraintPriorities(.{ .base_priority = 10 });
    const expected = [num_humanoid_joints]u32{ 13, 12, 10, 11, 10, 11, 10, 11, 10, 11, 10 };
    for (settings.parts.items[1..], expected[1..]) |*p, priority|
        try testing.expectEqual(priority, p.getToParent().?.base.constraint_priority);

    // The additional constraint between the lower arms gets the minimum of the two
    try testing.expectEqual(@as(u32, 10), settings.additional_constraints.items[0].getConstraint().?.base.constraint_priority);
    settings.additional_constraints.items[0].body_idx = .{ 0, 3 };
    try settings.calculateConstraintPriorities(.{});
    try testing.expectEqual(@as(u32, 1), settings.additional_constraints.items[0].getConstraint().?.base.constraint_priority);
    try testing.expectEqual(@as(u32, 2), settings.parts.items[1].getToParent().?.base.constraint_priority);
    try testing.expect(settings.parts.items[0].getToParent() == null);
}

/// A chain of boxes along X with the given masses (and motion types), each one the parent of the next (reference count
/// 0)
fn buildChain(allocator: Allocator, masses: []const f32, motion_types: []const MotionType) !*RagdollSettings {
    const settings = try RagdollSettings.create(allocator);
    errdefer destroyUnreferenced(settings);
    const skeleton = try Skeleton.create(allocator);
    settings.skeleton.set(skeleton);
    const box = try BoxShape.create(allocator, Vec3.init(0.2, 0.1, 0.1), .{ .convex_radius = 0.05 });
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    try settings.parts.ensureTotalCapacity(allocator, masses.len);
    for (masses, motion_types, 0..) |mass, motion_type, i| {
        _ = try skeleton.addJointWithParentIndex("Joint", @as(i32, @intCast(i)) - 1);
        var body: BodyCreationSettings = .initWithShape(box.asShape(), RVec3.init(0.5 * @as(Real, @floatFromInt(i)), 0, 0), Quat.identity(), motion_type, TestWorld.moving);
        defer body.deinit();
        body.override_mass_properties = .calculate_inertia;
        body.mass_properties_override.mass = mass;
        settings.parts.appendAssumeCapacity(.init(&body, null));
    }
    return settings;
}

test "RagdollSettings: stabilize" {
    const allocator = testing.allocator;

    // An empty skeleton has nothing to stabilize
    {
        var settings: RagdollSettings = .init(allocator);
        defer settings.deinit();
        settings.skeleton.set(try Skeleton.create(allocator));
        try testing.expect(try settings.stabilize());
    }

    // Two bodies: the child is 10 times heavier, its ratio is clamped to 1.2; the total mass stays the same
    {
        var settings = Ref(RagdollSettings).init(try buildChain(allocator, &.{ 1, 10 }, &.{ .dynamic, .dynamic }));
        defer settings.deinit();
        const parts = settings.get().?.parts.items;
        const child_inertia_before = (try parts[1].base.getMassProperties(allocator)).decomposePrincipalMomentsOfInertia().?.diagonal;
        try testing.expect(try settings.get().?.stabilize());
        for (parts) |*p|
            try testing.expect(p.base.override_mass_properties == .mass_and_inertia_provided);
        const m0 = parts[0].base.mass_properties_override.mass;
        const m1 = parts[1].base.mass_properties_override.mass;
        try testing.expectApproxEqAbs(@as(f32, 11), m0 + m1, 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 1.2), m1 / m0, 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 5), m0, 1.0e-5);

        // The child is scaled to its new mass (6 instead of 10)
        const child = parts[1].base.mass_properties_override;
        const child_diagonal = child.decomposePrincipalMomentsOfInertia().?.diagonal;
        try testing.expect(child_diagonal.isClose(child_inertia_before.mulScalar(0.6), .{ .max_dist_sq = 1.0e-10 }));

        // The largest moment of the parent is increased to the child sum (the largest moment of the child) when that is
        // less than twice its own, the matrix stays symmetric with a 1 in (3, 3)
        const parent = parts[0].base.mass_properties_override;
        const parent_diagonal = parent.decomposePrincipalMomentsOfInertia().?.diagonal;
        try testing.expect(parent_diagonal.getZ() >= 0.999 * @min(2.0 * parent_diagonal.getX(), child_diagonal.getX()));
        try testing.expectEqual(@as(f32, 1), parent.inertia.get(3, 3));
        try testing.expect(parent.inertia.isClose(parent.inertia.transposed(), .{ .max_dist_sq = 1.0e-10 }));
    }

    // A static part splits the chain: the bodies before it are one chain, the bodies after it another, the static part
    // is not changed; a chain of one body keeps its mass
    {
        var settings = Ref(RagdollSettings).init(try buildChain(allocator, &.{ 1, 3, 7, 2, 9 }, &.{ .dynamic, .dynamic, .static, .kinematic, .dynamic }));
        defer settings.deinit();
        const parts = settings.get().?.parts.items;
        try testing.expect(try settings.get().?.stabilize());
        try testing.expect(parts[2].base.override_mass_properties == .calculate_inertia);
        try testing.expectEqual(@as(f32, 7), parts[2].base.mass_properties_override.mass);
        try testing.expectApproxEqAbs(@as(f32, 4), parts[0].base.mass_properties_override.mass + parts[1].base.mass_properties_override.mass, 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 11), parts[3].base.mass_properties_override.mass + parts[4].base.mass_properties_override.mass, 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 1.2), parts[4].base.mass_properties_override.mass / parts[3].base.mass_properties_override.mass, 1.0e-5);

        var single = Ref(RagdollSettings).init(try buildChain(allocator, &.{ 4, 5 }, &.{ .static, .dynamic }));
        defer single.deinit();
        try testing.expect(try single.get().?.stabilize());
        try testing.expectEqual(@as(f32, 5), single.get().?.parts.items[1].base.mass_properties_override.mass);
        try testing.expect(single.get().?.parts.items[1].base.override_mass_properties == .mass_and_inertia_provided);
    }

    // The humanoid: every dynamic part gets provided mass properties, the masses of a parent and its children differ by
    // at most 20%
    {
        var settings = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{}));
        defer settings.deinit();
        var total_before: f32 = 0;
        for (settings.get().?.parts.items) |*p| total_before += (try p.base.getMassProperties(allocator)).mass;
        try testing.expect(try settings.get().?.stabilize());
        var total_after: f32 = 0;
        for (settings.get().?.parts.items, humanoid_joints) |*p, j| {
            total_after += p.base.mass_properties_override.mass;
            if (j.parent >= 0) {
                const ratio = p.base.mass_properties_override.mass / settings.get().?.parts.items[@intCast(j.parent)].base.mass_properties_override.mass;
                try testing.expect(ratio >= 0.8 * 0.9999 and ratio <= 1.2 * 1.0001);
            }
        }
        try testing.expectApproxEqRel(total_before, total_after, 1.0e-5);
    }

    // A part without a shape whose mass properties are calculated (Jolt dereferences null): error.InvalidShape, nothing
    // changed
    {
        var settings = Ref(RagdollSettings).init(try buildChain(allocator, &.{ 1, 2 }, &.{ .dynamic, .dynamic }));
        defer settings.deinit();
        const parts = settings.get().?.parts.items;
        parts[1].base.setShape(null);
        try testing.expectError(error.InvalidShape, settings.get().?.stabilize());
        try testing.expect(parts[0].base.override_mass_properties == .calculate_inertia);
    }

    // Release builds: an inertia tensor that cannot be decomposed (NaN) makes Stabilize return false (Jolt asserts in
    // its debug build, so does Zolt in safe builds)
    if (!Core.enable_asserts) {
        var settings = Ref(RagdollSettings).init(try buildChain(allocator, &.{ 1, 2 }, &.{ .dynamic, .dynamic }));
        defer settings.deinit();
        const parts = settings.get().?.parts.items;
        parts[1].base.override_mass_properties = .mass_and_inertia_provided;
        const nan = Vec4.replicate(std.math.nan(f32));
        parts[1].base.mass_properties_override = .{ .mass = 2, .inertia = Mat44.init(nan, nan, nan, Vec4.init(0, 0, 0, 1)) };
        try testing.expect(!try settings.get().?.stabilize());
    }
}

test "RagdollSettings: disableParentChildCollisions" {
    const allocator = testing.allocator;
    var settings_ref = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{}));
    defer settings_ref.deinit();
    const settings = settings_ref.get().?;

    // Without joint matrices: only parent and child do not collide
    try settings.disableParentChildCollisions(.{});
    const filter = settings.parts.items[0].base.collision_group.getGroupFilter().?;
    const table = virtual.downcast(GroupFilterTable, filter);
    try testing.expectEqual(@as(u32, num_humanoid_joints), table.num_sub_groups);
    for (settings.parts.items, humanoid_joints, 0..) |*p, j, i| {
        try testing.expect(p.base.collision_group.getGroupFilter() == filter);
        try testing.expectEqual(@as(u32, @intCast(i)), p.base.collision_group.getSubGroupID());
        for (0..i) |other| {
            const is_parent = j.parent == @as(i32, @intCast(other));
            try testing.expectEqual(!is_parent, table.isCollisionEnabled(@intCast(i), @intCast(other)));
        }
    }
    try testing.expectEqual(@as(u32, num_humanoid_joints), filter.getRefCount());

    // With joint matrices and no separation distance: no other parts touch (the first filter is destroyed when the
    // parts release it, keep its bits)
    var parent_child_bits: [7]u8 = undefined;
    @memcpy(&parent_child_bits, table.table.items);
    const matrices = bindPoseMatrices();
    try settings.disableParentChildCollisions(.{ .joint_matrices = &matrices });
    const table2 = virtual.downcast(GroupFilterTable, settings.parts.items[0].base.collision_group.getGroupFilter().?);
    try testing.expectEqualSlices(u8, &parent_child_bits, table2.table.items);

    // A separation distance of 0.06: the lower arms are closer than that to the pelvis and the upper legs
    try settings.disableParentChildCollisions(.{ .joint_matrices = &matrices, .min_separation_distance = 0.06 });
    const table3 = virtual.downcast(GroupFilterTable, settings.parts.items[0].base.collision_group.getGroupFilter().?);
    try testing.expect(!table3.isCollisionEnabled(4, 0)); // Lower left arm - pelvis
    try testing.expect(!table3.isCollisionEnabled(6, 0)); // Lower right arm - pelvis
    try testing.expect(!table3.isCollisionEnabled(4, 7)); // Lower left arm - upper left leg
    try testing.expect(!table3.isCollisionEnabled(6, 9)); // Lower right arm - upper right leg
    try testing.expect(table3.isCollisionEnabled(7, 9)); // The upper legs are 0.08 apart
    try testing.expect(table3.isCollisionEnabled(8, 10)); // The lower legs are 0.1 apart
    try testing.expect(table3.isCollisionEnabled(2, 4)); // Head - lower left arm
    try testing.expect(!table3.isCollisionEnabled(1, 3)); // Spine - upper left arm: parent and child

    // Overlapping parts are found without a separation distance; static parts are not tested against each other
    var overlapping = matrices;
    overlapping[2] = Mat44.translation(Vec3.init(-0.25, 1.25, 0)); // The head inside the upper left arm
    overlapping[10] = Mat44.translation(Vec3.init(-0.1, 0.25, 0)); // The right lower leg on the left one
    settings.parts.items[8].base.motion_type = .static;
    settings.parts.items[10].base.motion_type = .static;
    try settings.disableParentChildCollisions(.{ .joint_matrices = &overlapping });
    const table4 = virtual.downcast(GroupFilterTable, settings.parts.items[0].base.collision_group.getGroupFilter().?);
    try testing.expect(!table4.isCollisionEnabled(2, 3));
    try testing.expect(table4.isCollisionEnabled(8, 10));
    settings.parts.items[10].base.motion_type = .dynamic;
    try settings.disableParentChildCollisions(.{ .joint_matrices = &overlapping });
    const table5 = virtual.downcast(GroupFilterTable, settings.parts.items[0].base.collision_group.getGroupFilter().?);
    try testing.expect(!table5.isCollisionEnabled(8, 10));

    // A part without a shape (Jolt dereferences null): error.InvalidShape, the parts keep their group filter
    const kept = settings.parts.items[0].base.collision_group.getGroupFilter();
    settings.parts.items[5].base.setShape(null);
    try testing.expectError(error.InvalidShape, settings.disableParentChildCollisions(.{ .joint_matrices = &matrices }));
    try testing.expect(settings.parts.items[0].base.collision_group.getGroupFilter() == kept);

    // Shape settings are converted (GetShape)
    const sphere_settings = try SphereShapeSettings.create(allocator, 0.05, .{});
    settings.parts.items[5].base.setShapeSettings(sphere_settings.asShapeSettings());
    try settings.disableParentChildCollisions(.{ .joint_matrices = &matrices });
}

test "RagdollSettings: saveBinaryState / restoreFromBinaryState" {
    const allocator = testing.allocator;
    var settings_ref = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{ .mixed = true }));
    defer settings_ref.deinit();
    const settings = settings_ref.get().?;
    try settings.disableParentChildCollisions(.{});
    try settings.calculateConstraintPriorities(.{ .base_priority = 3 });
    settings.parts.items[3].setToParent(null); // A part without a constraint in the middle

    for ([_]bool{ true, false }) |save_shapes| {
        for ([_]bool{ true, false }) |save_group_filter| {
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            const bytes = try saveSettings(settings, &out, save_shapes, save_group_filter);

            var result = try restoreSettings(allocator, bytes);
            defer result.deinit();
            try testing.expect(result.isValid());
            const restored = result.getPtr().?;
            try testing.expectEqual(@as(u32, 1), restored.getRefCount());
            try testing.expectEqual(num_humanoid_joints, restored.getSkeleton().?.getJointCount());
            try testing.expectEqual(num_humanoid_joints, restored.parts.items.len);
            try testing.expectEqual(@as(usize, 1), restored.additional_constraints.items.len);
            try testing.expectEqual([2]i32{ 4, 6 }, restored.additional_constraints.items[0].body_idx);

            // The maps are calculated by the restore
            try testing.expectEqualSlices(i32, &.{ -1, 0, 1, -1, 2, 3, 4, 5, 6, 7, 8 }, restored.getBodyIndexToConstraintIndex());
            try testing.expectEqual(@as(usize, 10), restored.getConstraintIndexToBodyIdxPair().len);

            // The shapes and group filters as selected, shared where they were shared
            const p = restored.parts.items;
            if (save_shapes) {
                const shape3 = try p[3].base.getShape(allocator);
                const shape5 = try p[5].base.getShape(allocator);
                try testing.expect(shape3 != null and shape3 == shape5);
            } else try testing.expect(try p[3].base.getShape(allocator) == null);
            try testing.expect((p[0].base.collision_group.getGroupFilter() != null) == save_group_filter);
            if (save_group_filter)
                try testing.expect(p[0].base.collision_group.getGroupFilter() == p[10].base.collision_group.getGroupFilter());
            try testing.expectEqual(@as(u32, 4), p[4].base.collision_group.getSubGroupID());
            try testing.expectEqual(settings.parts.items[1].getToParent().?.base.constraint_priority, p[1].getToParent().?.base.constraint_priority);

            // Saving the restored settings gives the same bytes
            var out2: std.Io.Writer.Allocating = .init(allocator);
            defer out2.deinit();
            try testing.expectEqualSlices(u8, bytes, try saveSettings(restored, &out2, save_shapes, save_group_filter));

            // Truncated streams: an error, or settings with no part or every part, and without additional constraints
            // (Jolt reads the part count, the group filter ID of a part (RestoreObjectReference), the has-constraint
            // flag and the additional constraint count without checking the stream: the end of the skeleton gives a
            // ragdoll without parts, the end before the group filter ID of the last part a last part without group
            // filter and constraint)
            var num_valid: u32 = 0;
            for (0..bytes.len) |len| {
                var truncated = try restoreSettings(allocator, bytes[0..len]);
                defer truncated.deinit();
                if (!truncated.hasError()) {
                    const r = truncated.getPtr().?;
                    try testing.expect(r.parts.items.len == 0 or r.parts.items.len == num_humanoid_joints);
                    try testing.expectEqual(@as(usize, 0), r.additional_constraints.items.len);
                    num_valid += 1;
                }
            }
            try testing.expect(num_valid >= 4);
        }
    }

    // A part without a shape cannot be saved with its shape (Jolt dereferences null)
    settings.parts.items[2].base.setShape(null);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    try testing.expectError(error.InvalidShape, saveSettings(settings, &out, true, false));
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    _ = try saveSettings(settings, &out2, false, true);
}

test "RagdollSettings: restoreFromBinaryState of the registered and unknown constraint settings classes" {
    const allocator = testing.allocator;
    var settings_ref = Ref(RagdollSettings).init(try buildHumanoid(allocator, .{}));
    defer settings_ref.deinit();
    const settings = settings_ref.get().?;

    // The test constraint is a registered TwoBodyConstraintSettings (zolt_user_types of the inline tests)
    const test_settings = try TestConstraintSettings.create(allocator);
    settings.parts.items[1].setToParent(&test_settings.base);
    test_settings.value = 7.5;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveSettings(settings, &out, false, false);
    {
        var result = try restoreSettings(allocator, bytes);
        defer result.deinit();
        const restored = virtual.downcast(TestConstraintSettings, result.getPtr().?.parts.items[1].getToParent().?);
        try testing.expectEqual(@as(f32, 7.5), restored.value);
    }

    // An unknown type hash gives Jolt's error
    const hash = ConstraintSettings.rttiHash(TestConstraintSettings.rtti_name);
    const at = std.mem.indexOf(u8, bytes, std.mem.asBytes(&hash)).?;
    const corrupt = try allocator.dupe(u8, bytes);
    defer allocator.free(corrupt);
    std.mem.writeInt(u32, corrupt[at..][0..4], 0x12345678, .little);
    var result = try restoreSettings(allocator, corrupt);
    defer result.deinit();
    try testing.expectEqualStrings("Failed to create instance of type", result.getError());

    // The DynamicCast of a restored constraint: every registered TwoBodyConstraintSettings class passes
    try testing.expect(asTwoBodyConstraintSettings(test_settings.asSettings()) == &test_settings.base);
    const hinge = try HingeConstraintSettings.create(allocator);
    var hinge_ref = Ref(ConstraintSettings).init(hinge.asSettings());
    defer hinge_ref.deinit();
    try testing.expect(asTwoBodyConstraintSettings(hinge.asSettings()) == hinge.asTwoBodySettings());
}

test "Ragdoll: createRagdoll, addToPhysicsSystem, removeFromPhysicsSystem, activate, isActive" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    _ = try world.addFloor();
    var settings = try buildPreparedHumanoid(allocator, .{ .mixed = true });
    defer settings.deinit();
    try settings.get().?.disableParentChildCollisions(.{});

    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(5, 42, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;
    try testing.expectEqual(@as(u32, 1), r.getRefCount());
    try testing.expectEqual(@as(u32, 2), settings.get().?.getRefCount());
    try testing.expect(r.getRagdollSettings() == settings.get().?);
    try testing.expectEqual(num_humanoid_joints, r.getBodyCount());
    try testing.expectEqual(@as(usize, 11), r.getConstraintCount());
    try testing.expectEqual(@as(u32, 12), world.system.getNumBodies());

    // The bodies are created but not added; their user data and group ID come from CreateRagdoll
    const bi = world.bodyInterface();
    for (r.getBodyIDs(), 0..) |id, i| {
        try testing.expect(id.eql(r.getBodyID(i)));
        try testing.expect(!bi.isAdded(id));
        try testing.expectEqual(@as(u64, 42), bi.getUserData(id));
        try testing.expectEqual(@as(u32, 5), bi.getCollisionGroup(id).getGroupID());
        try testing.expectEqual(@as(u32, @intCast(i)), bi.getCollisionGroup(id).getSubGroupID());
    }

    // The constraints connect the bodies of the map
    for (settings.get().?.getConstraintIndexToBodyIdxPair(), 0..) |pair, c| {
        const constraint = r.getConstraint(c);
        try testing.expect(constraint.getBody1().getID().eql(r.getBodyID(@intCast(pair.first))));
        try testing.expect(constraint.getBody2().getID().eql(r.getBodyID(@intCast(pair.second))));
        try testing.expect(r.getConstraintMut(c) == @constCast(constraint));
    }
    try testing.expect(r.getConstraint(0).base.getSubType() == .swing_twist);
    try testing.expect(r.getConstraint(1).base.getSubType() == .six_dof);
    try testing.expect(r.getConstraint(5).base.getSubType() == .fixed);
    try testing.expect(r.getConstraint(10).base.getSubType() == .distance);

    // Added and activated
    try r.addToPhysicsSystem(.activate, .{});
    try testing.expectEqual(@as(u32, 11), world.system.constraint_manager.getNumConstraints());
    for (r.getBodyIDs()) |id| {
        try testing.expect(bi.isAdded(id));
        try testing.expect(bi.isActive(id));
    }
    try testing.expect(r.isActive(.{}));
    try testing.expect(r.isActive(.{ .lock_bodies = false }));
    try world.update(2);

    // Removed: no body is added or active, no constraint is in the system
    try r.removeFromPhysicsSystem(.{ .lock_bodies = false });
    try testing.expectEqual(@as(u32, 0), world.system.constraint_manager.getNumConstraints());
    for (r.getBodyIDs()) |id|
        try testing.expect(!bi.isAdded(id));
    try testing.expect(!r.isActive(.{}));

    // Added without activation, then activated
    try r.addToPhysicsSystem(.dont_activate, .{ .lock_bodies = false });
    try testing.expect(!r.isActive(.{}));
    try testing.expectEqual(@as(u32, 0), world.system.getNumActiveBodies(.rigid_body));
    r.activate(.{});
    try testing.expect(r.isActive(.{ .lock_bodies = false }));
    try testing.expectEqual(@as(u32, 11), world.system.getNumActiveBodies(.rigid_body));
    try r.removeFromPhysicsSystem(.{});
    try r.addToPhysicsSystem(.dont_activate, .{});
    r.activate(.{ .lock_bodies = false });
    try testing.expect(r.isActive(.{}));
    try world.update(1);

    // Destroying the ragdoll (after removing it) destroys its bodies
    try r.removeFromPhysicsSystem(.{});
    ragdoll.deinit();
    try testing.expectEqual(@as(u32, 1), world.system.getNumBodies());
    try testing.expectEqual(@as(u32, 1), settings.get().?.getRefCount());

    // An empty ragdoll (Jolt's public constructor)
    var empty: Ragdoll = .init(allocator, &world.system);
    defer empty.deinit();
    try empty.addToPhysicsSystem(.activate, .{});
    try testing.expect(!empty.isActive(.{}));
    var root_offset = RVec3.init(1, 2, 3);
    empty.getPoseMatrices(&root_offset, &.{}, .{});
    try testing.expect(root_offset.eql(RVec3.init(1, 2, 3))); // Not written without bodies
    try testing.expect(empty.getWorldSpaceBounds(.{}).min.eql(AABox.empty.min));
    try empty.removeFromPhysicsSystem(.{});
    try testing.expect(empty.getRagdollSettings() == null);
}

test "Ragdoll: createRagdoll out of bodies and with a part without a shape" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 6);
    defer world.deinit();
    _ = try world.addFloor();
    var settings = try buildPreparedHumanoid(allocator, .{});
    defer settings.deinit();

    // Room for 5 of the 11 parts: null, the bodies that were created are destroyed again
    try testing.expect(try settings.get().?.createRagdoll(1, 0, &world.system) == null);
    try testing.expectEqual(@as(u32, 1), world.system.getNumBodies());
    try testing.expectEqual(@as(u32, 1), settings.get().?.getRefCount());

    // A part without a shape (Jolt crashes in CreateBody): error.InvalidShape, nothing is left
    var world2: TestWorld = undefined;
    try world2.init(allocator, 64);
    defer world2.deinit();
    settings.get().?.parts.items[6].base.setShape(null);
    try testing.expectError(error.InvalidShape, settings.get().?.createRagdoll(1, 0, &world2.system));
    try testing.expectEqual(@as(u32, 0), world2.system.getNumBodies());
}

test "Ragdoll: setPose / getPose and the root transform, far from the origin" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    var settings = try buildPreparedHumanoid(allocator, .{});
    defer settings.deinit();
    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(1, 0, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;

    // A pose: local joint states relative to the parent with some rotations, a root offset far away in double
    // precision
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(settings.get().?.getSkeleton().?);
    const root_offset: RVec3 = if (Core.double_precision) RVec3.init(1.0e7, 5, -2.0e7) else RVec3.init(100, 5, -200);
    pose.setRootOffset(root_offset);
    for (pose.getJointsMut().items, humanoid_joints, 0..) |*state, j, i| {
        const parent_position: [3]f32 = if (j.parent >= 0) humanoid_joints[@intCast(j.parent)].position else .{ 0, 0, 0 };
        state.translation = Vec3.init(j.position[0] - parent_position[0], j.position[1] - parent_position[1], j.position[2] - parent_position[2]);
        state.rotation = Quat.rotation(Vec3.init(1, 0.5, -0.25).normalized(), 0.1 * @as(f32, @floatFromInt(i)));
    }
    pose.calculateJointMatrices();

    const bi = world.bodyInterface();
    for ([_]bool{ true, false }) |lock_bodies| {
        r.setPose(&pose, .{ .lock_bodies = lock_bodies });

        // The world transforms of the bodies are the root offset plus the joint matrices
        for (r.getBodyIDs(), pose.getJointMatrices()) |id, m| {
            const expected = root_offset.addVec3(m.getTranslation());
            try testing.expect(bi.getPosition(id).isClose(expected, .{ .max_dist_sq = 1.0e-10 }));
            try testing.expect(bi.getRotation(id).isClose(m.getQuaternion(), .{ .max_dist_sq = 1.0e-10 }));
        }

        // GetPose: the root body gives the root offset (the joint matrix of the root has no translation)
        var out_pose: SkeletonPose = .init(allocator);
        defer out_pose.deinit();
        try out_pose.setSkeleton(settings.get().?.getSkeleton().?);
        r.getPose(&out_pose, .{ .lock_bodies = lock_bodies });
        const out_root = out_pose.getRootOffset();
        try testing.expect(out_root.isClose(root_offset.addVec3(pose.getJointMatrix(0).getTranslation()), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(out_pose.getJointMatrix(0).getTranslation().eql(Vec3.zero()));
        for (out_pose.getJointMatrices(), pose.getJointMatrices()) |out_m, m| {
            try testing.expect(out_root.addVec3(out_m.getTranslation()).isClose(root_offset.addVec3(m.getTranslation()), .{ .max_dist_sq = 1.0e-8 }));
            try testing.expect(out_m.getQuaternion().isClose(m.getQuaternion(), .{ .max_dist_sq = 1.0e-10 }) or out_m.getQuaternion().isClose(m.getQuaternion().negate(), .{ .max_dist_sq = 1.0e-10 }));
        }

        // The low level versions give the same
        var matrices: [num_humanoid_joints]Mat44 = undefined;
        var matrices_root = RVec3.zero();
        r.getPoseMatrices(&matrices_root, &matrices, .{ .lock_bodies = lock_bodies });
        try testing.expect(matrices_root.eql(out_root));
        for (matrices, out_pose.getJointMatrices()) |a, b|
            try testing.expect(a.eql(b));

        // The root transform is the pelvis body
        const root = r.getRootTransform(.{ .lock_bodies = lock_bodies });
        try testing.expect(root.position.eql(bi.getPosition(r.getBodyID(0))));
        try testing.expect(root.rotation.eql(bi.getRotation(r.getBodyID(0))));

        // SetPose with matrices puts the bodies back at the bind pose
        const bind = bindPoseMatrices();
        r.setPoseMatrices(root_offset, &bind, .{ .lock_bodies = lock_bodies });
        for (r.getBodyIDs(), bind) |id, m|
            try testing.expect(bi.getPosition(id).eql(root_offset.addVec3(m.getTranslation())));

        // The bounds encapsulate every body
        const bounds = r.getWorldSpaceBounds(.{ .lock_bodies = lock_bodies });
        const root_y: f32 = @floatCast(root_offset.getY());
        try testing.expectApproxEqAbs(@as(f32, 1.72), bounds.max.getY() - root_y, 1.0e-4);
        try testing.expectApproxEqAbs(@as(f32, 0.08), bounds.min.getY() - root_y, 1.0e-4);
    }

    // A root body that does not exist gives the identity
    const root_id = r.getBodyID(0);
    bi.destroyBody(root_id);
    const root = r.getRootTransform(.{});
    try testing.expect(root.position.eql(RVec3.zero()));
    try testing.expect(root.rotation.eql(Quat.identity()));
    _ = r.body_ids.orderedRemove(0); // The destructor destroys the other bodies
}

test "Ragdoll: driveToPoseUsingKinematics" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    world.system.setGravity(Vec3.zero());
    var settings = try buildPreparedHumanoid(allocator, .{ .pelvis_motion_type = .kinematic, .motion_type = .kinematic });
    defer settings.deinit();
    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(1, 0, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;
    try r.addToPhysicsSystem(.activate, .{});

    // A pose 0.5 to the side and turned: after one step of the delta time the bodies are there
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(settings.get().?.getSkeleton().?);
    pose.setRootOffset(RVec3.init(0.5, 0, 0));
    for (pose.getJointsMut().items, humanoid_joints) |*state, j| {
        const parent_position: [3]f32 = if (j.parent >= 0) humanoid_joints[@intCast(j.parent)].position else .{ 0, 0, 0 };
        state.translation = Vec3.init(j.position[0] - parent_position[0], j.position[1] - parent_position[1], j.position[2] - parent_position[2]);
        state.rotation = if (j.parent < 0) Quat.rotation(Vec3.axisY(), 0.2) else Quat.identity();
    }
    pose.calculateJointMatrices();
    r.driveToPoseUsingKinematics(&pose, TestWorld.delta_time, .{});
    const bi = world.bodyInterface();
    try testing.expect(bi.getLinearVelocity(r.getBodyID(0)).isClose(Vec3.init(30, 0, 0), .{ .max_dist_sq = 1.0e-6 }));
    try world.update(1);
    for (r.getBodyIDs(), pose.getJointMatrices()) |id, m|
        try testing.expect(bi.getPosition(id).isClose(RVec3.init(0.5, 0, 0).addVec3(m.getTranslation()), .{ .max_dist_sq = 1.0e-6 }));

    // The low level version without locking: back to the bind pose
    const bind = bindPoseMatrices();
    r.driveToPoseUsingKinematicsMatrices(RVec3.zero(), &bind, TestWorld.delta_time, .{ .lock_bodies = false });
    try world.update(1);
    for (r.getBodyIDs(), bind) |id, m|
        try testing.expect(bi.getPosition(id).isClose(RVec3.zero().addVec3(m.getTranslation()), .{ .max_dist_sq = 1.0e-6 }));
    try r.removeFromPhysicsSystem(.{});
}

/// The rotation of body 2 relative to body 1 of a constraint
fn relativeRotation(constraint: *const TwoBodyConstraint) Quat {
    return constraint.getBody1().getRotation().conjugated().mul(constraint.getBody2().getRotation());
}

test "Ragdoll: driveToPoseUsingMotors" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    world.system.setGravity(Vec3.zero());
    var settings = try buildPreparedHumanoid(allocator, .{ .pelvis_motion_type = .static });
    defer settings.deinit();
    try settings.get().?.disableParentChildCollisions(.{});
    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(1, 0, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;
    try r.addToPhysicsSystem(.activate, .{});
    errdefer r.removeFromPhysicsSystem(.{}) catch {};

    // The target: knees and elbows bent, the spine, head, arms and legs swung forward a little (around X, so that the
    // left and right parts do not touch)
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(settings.get().?.getSkeleton().?);
    for (pose.getJointsMut().items, humanoid_joints) |*state, j| {
        state.rotation = switch (j.kind) {
            .hinge => Quat.rotation(Vec3.axisX(), -0.6),
            .swing_twist => Quat.rotation(Vec3.axisX(), 0.15),
            else => Quat.identity(),
        };
    }
    pose.calculateJointMatrices();
    r.driveToPoseUsingMotors(&pose);
    for (1..num_humanoid_joints) |i| {
        const constraint = r.getConstraint(@intCast(settings.get().?.getConstraintIndexForBodyIndex(i)));
        switch (constraint.base.getSubType()) {
            .swing_twist => {
                const st = virtual.downcast(SwingTwistConstraint, constraint);
                try testing.expect(st.getSwingMotorState() == .position and st.getTwistMotorState() == .position);
            },
            .hinge => try testing.expect(virtual.downcast(HingeConstraint, constraint).getMotorState() == .position),
            else => return error.TestUnexpectedResult,
        }
    }

    // The motors bring the joints to the target rotations
    try world.update(180);
    for (1..num_humanoid_joints) |i| {
        const constraint = r.getConstraint(@intCast(settings.get().?.getConstraintIndexForBodyIndex(i)));
        const target = pose.getJoint(i).rotation;
        const actual = relativeRotation(constraint);
        try testing.expect(actual.isClose(target, .{ .max_dist_sq = 1.0e-3 }) or actual.isClose(target.negate(), .{ .max_dist_sq = 1.0e-3 }));
    }

    // Driving with velocity: the motors drive to the position and the velocity of the change between the poses
    var prev_pose = try pose.clone();
    defer prev_pose.deinit();
    for (pose.getJointsMut().items, humanoid_joints) |*state, j| {
        if (j.kind == .hinge) state.rotation = Quat.rotation(Vec3.axisX(), -0.7);
    }
    pose.calculateJointMatrices();
    r.driveToPoseUsingMotorsWithVelocity(&prev_pose, &pose, 0.5);
    for (1..num_humanoid_joints) |i| {
        const constraint = r.getConstraint(@intCast(settings.get().?.getConstraintIndexForBodyIndex(i)));
        switch (constraint.base.getSubType()) {
            .swing_twist => {
                const st = virtual.downcast(SwingTwistConstraint, constraint);
                try testing.expect(st.getSwingMotorState() == .position_and_velocity and st.getTwistMotorState() == .position_and_velocity);
                try testing.expect(st.getTargetAngularVelocityCS().isNearZero(.{}));
            },
            .hinge => {
                const h = virtual.downcast(HingeConstraint, constraint);
                try testing.expect(h.getMotorState() == .position_and_velocity);
                try testing.expectApproxEqAbs(@as(f32, -0.2), h.getTargetAngularVelocity(), 1.0e-4); // -0.1 rad in 0.5 s
            },
            else => return error.TestUnexpectedResult,
        }
    }
    try world.update(60);
    r.resetWarmStart();
    try world.update(1);
    try r.removeFromPhysicsSystem(.{});
}

test "Ragdoll: velocities, impulses and the group ID" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    world.system.setGravity(Vec3.zero());
    var settings = try buildPreparedHumanoid(allocator, .{});
    defer settings.deinit();
    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(1, 0, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;
    try r.addToPhysicsSystem(.dont_activate, .{});
    const bi = world.bodyInterface();

    r.setLinearAndAngularVelocity(Vec3.init(1, 2, 3), Vec3.init(0.1, 0.2, 0.3), .{});
    for (r.getBodyIDs()) |id| {
        try testing.expect(bi.getLinearVelocity(id).eql(Vec3.init(1, 2, 3)));
        try testing.expect(bi.getAngularVelocity(id).eql(Vec3.init(0.1, 0.2, 0.3)));
    }
    try testing.expect(r.isActive(.{}));
    r.setLinearVelocity(Vec3.init(-1, 0, 0), .{ .lock_bodies = false });
    r.addLinearVelocity(Vec3.init(0, 0.5, 0), .{});
    for (r.getBodyIDs()) |id|
        try testing.expect(bi.getLinearVelocity(id).eql(Vec3.init(-1, 0.5, 0)));

    // An impulse changes the velocity by impulse / mass of each body
    r.setLinearVelocity(Vec3.zero(), .{});
    r.addImpulse(Vec3.init(0, 0, 2), .{ .lock_bodies = false });
    for (r.getBodyIDs(), settings.get().?.parts.items) |id, *p| {
        const mass = (try p.base.getMassProperties(allocator)).mass;
        try testing.expectApproxEqRel(2.0 / mass, bi.getLinearVelocity(id).getZ(), 1.0e-5);
    }

    r.setGroupID(77, .{ .lock_bodies = false });
    for (r.getBodyIDs()) |id|
        try testing.expectEqual(@as(u32, 77), bi.getCollisionGroup(id).getGroupID());
    r.setGroupID(78, .{});
    try testing.expectEqual(@as(u32, 78), bi.getCollisionGroup(r.getBodyID(3)).getGroupID());
    try r.removeFromPhysicsSystem(.{});
}

test "Ragdoll: falls on the floor, falls asleep and wakes up" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    _ = try world.addFloor();
    var settings = try buildPreparedHumanoid(allocator, .{ .mixed = true });
    defer settings.deinit();
    try settings.get().?.disableParentChildCollisions(.{});
    try testing.expect(try settings.get().?.stabilize());
    var ragdoll = Ref(Ragdoll).init((try settings.get().?.createRagdoll(1, 0, &world.system)).?);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;
    try r.addToPhysicsSystem(.activate, .{});
    r.setLinearAndAngularVelocity(Vec3.init(0.5, 0, 0), Vec3.init(0, 0, 1), .{});

    var updates: u32 = 0;
    while (r.isActive(.{}) and updates < 900) : (updates += 1)
        try world.update(1);
    try testing.expect(!r.isActive(.{}));
    const bounds = r.getWorldSpaceBounds(.{});
    try testing.expect(bounds.min.getY() > -0.05 and bounds.max.getY() < 0.8); // Lying on the floor

    r.addImpulse(Vec3.init(0, 5, 0), .{});
    try testing.expect(r.isActive(.{}));
    try world.update(1);
    try r.removeFromPhysicsSystem(.{});
}

/// Remove a ragdoll from the system without allocating (the cleanup after a failed removeFromPhysicsSystem: the
/// failing allocator of checkAllAllocationFailures fails every allocation after the first failure)
fn removeWithoutAllocating(r: *Ragdoll) void {
    var constraints: [16]*Constraint = undefined;
    for (r.constraints.items, 0..) |c, i| constraints[i] = c.get().?;
    r.system.removeConstraints(constraints[0..r.constraints.items.len]);
    var bodies: [16]BodyID = undefined;
    @memcpy(bodies[0..r.body_ids.items.len], r.body_ids.items);
    r.system.getBodyInterfaceMut().removeBodies(bodies[0..r.body_ids.items.len]);
}

/// Build, create, add, remove and destroy a ragdoll with an allocator that can fail: every failure leaves nothing in
/// the system
fn ragdollOutOfMemory(allocator: Allocator) !void {
    var world: TestWorld = undefined;
    try world.init(allocator, 64);
    defer world.deinit();
    _ = try world.addFloor();
    var settings = try buildPreparedHumanoid(allocator, .{ .mixed = true });
    defer settings.deinit();
    try settings.get().?.disableParentChildCollisions(.{});

    const ragdoll_ptr = (settings.get().?.createRagdoll(1, 0, &world.system) catch |err| {
        try testing.expectEqual(@as(u32, 1), world.system.getNumBodies());
        try testing.expectEqual(@as(u32, 1), settings.get().?.getRefCount());
        return err;
    }).?;
    var ragdoll = Ref(Ragdoll).init(ragdoll_ptr);
    defer ragdoll.deinit();
    const r = ragdoll.get().?;

    const bi = world.bodyInterface();
    for ([_]bool{ true, false }) |lock_bodies| {
        r.addToPhysicsSystem(.activate, .{ .lock_bodies = lock_bodies }) catch |err| {
            try testing.expectEqual(@as(u32, 0), world.system.constraint_manager.getNumConstraints());
            for (r.getBodyIDs()) |id| try testing.expect(!bi.isAdded(id));
            try testing.expect(!r.isActive(.{}));
            return err;
        };
        try testing.expect(r.isActive(.{}));
        r.removeFromPhysicsSystem(.{ .lock_bodies = lock_bodies }) catch |err| {
            try testing.expectEqual(@as(u32, 11), world.system.constraint_manager.getNumConstraints());
            for (r.getBodyIDs()) |id| try testing.expect(bi.isAdded(id));
            removeWithoutAllocating(r); // Every allocation fails from now on
            return err;
        };
    }
    try testing.expectEqual(@as(u32, 0), world.system.constraint_manager.getNumConstraints());
}

test "Ragdoll: out of memory in createRagdoll, addToPhysicsSystem and removeFromPhysicsSystem" {
    try testing.checkAllAllocationFailures(testing.allocator, ragdollOutOfMemory, .{});
}

/// Prepare, save and restore settings with an allocator that can fail: stabilize and disableParentChildCollisions
/// change nothing when they fail
fn settingsOutOfMemory(allocator: Allocator) !void {
    var settings = try buildPreparedHumanoid(allocator, .{ .mixed = true });
    defer settings.deinit();
    const s = settings.get().?;
    const before = s.parts.items[1].base.mass_properties_override;
    _ = s.stabilize() catch |err| {
        try testing.expect(s.parts.items[1].base.override_mass_properties == .calculate_mass_and_inertia);
        try testing.expect(s.parts.items[1].base.mass_properties_override.eql(&before));
        return err;
    };
    try s.calculateConstraintPriorities(.{ .base_priority = 5 });
    const matrices = bindPoseMatrices();
    s.disableParentChildCollisions(.{ .joint_matrices = &matrices, .min_separation_distance = 0.06 }) catch |err| {
        try testing.expect(s.parts.items[0].base.collision_group.getGroupFilter() == null);
        return err;
    };
    try s.calculateBodyIndexToConstraintIndex();
    try s.calculateConstraintIndexToBodyIdxPair();
    for ([_]bool{ true, false }) |save_shapes| {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        const bytes = try saveSettings(s, &out, save_shapes, !save_shapes);
        var result = try restoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
    }
}

test "RagdollSettings: out of memory in stabilize, disableParentChildCollisions, saveBinaryState and restoreFromBinaryState" {
    try testing.checkAllAllocationFailures(testing.allocator, settingsOutOfMemory, .{});
}
