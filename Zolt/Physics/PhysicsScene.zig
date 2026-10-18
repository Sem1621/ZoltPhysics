//! Port of: Jolt/Physics/PhysicsScene.h, Jolt/Physics/PhysicsScene.cpp
//! Status: partial
//! Missing: the soft bodies of Phase 9 (AddSoftBody, GetNumSoftBodies, GetSoftBodies, mSoftBodies and their part of
//! CreateBodies, SaveBinaryState, sRestoreFromBinaryState and FromPhysicsSystem: SoftBodyCreationSettings has no binary
//! state and no soft body can be created before Phase 9)
//!
//! SimulationArchitecture D6 / D8:
//! - A RefTarget (`ref_count`, `addRef` / `release`) that keeps its allocator: `create(allocator)` is `new
//!   PhysicsScene()` (reference count 0), `init(allocator)` / `deinit` a scene on the stack or embedded.
//! - `ConnectedConstraint.settings` is a `RefConst(ConstraintSettings)` that holds a TwoBodyConstraintSettings (Jolt:
//!   `RefConst<TwoBodyConstraintSettings>`; a concrete class has no addRef / release of its own, guide section 5),
//!   `getSettings()` returns the TwoBodyConstraintSettings.
//! - The constraints of a restored scene are created by ConstraintSettings.restoreFromBinaryState, which knows the
//!   classes of `RegisterTypes.factory`: Jolt's settings classes and the user's. An unknown type gives Jolt's "Failed
//!   to create instance of type".
//! - Soft bodies (Phase 9): `saveBinaryState` writes Jolt's soft body count (always 0), `restoreFromBinaryState`
//!   reports the Zolt error "Soft bodies are not supported" for a stream that contains soft bodies.
//! - Errors (Zolt): functions that allocate return `Allocator.Error`. Where Jolt dereferences the null shape of a body
//!   without one (`FixInvalidScales`, `SaveBinaryState` with shapes, `CreateBodies`), Zolt returns `error.InvalidShape`
//!   (`CreateBodyError`, like BodyInterface.createBody). `createBodies` is all or nothing for these errors: the bodies
//!   and constraints it created are removed and destroyed again before the error is returned (Jolt's soft failure
//!   "out of bodies" keeps Jolt's behavior: the bodies that could be created are added, the result is false).
//! - A constraint whose GetConstraintSettings returns nullptr (PathConstraint, reached with asserts off) becomes a
//!   ConnectedConstraint without settings in `fromPhysicsSystem`, like Jolt's AddConstraint(nullptr, ...);
//!   `createBodies` and `saveBinaryState` panic on it where Jolt dereferences the null pointer.
//! - TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL (PhysicsScene, ConnectedConstraint).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../Core/RTTI.zig").RTTI;
const Core = @import("../Core/Core.zig");
const virtual = @import("../Core/Virtual.zig");
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const RefCount = ReferenceFile.RefCount;
const Result = @import("../Core/Result.zig").Result;
const StreamIn = @import("../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;
const UnorderedMap = @import("../Core/UnorderedMap.zig").UnorderedMap;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const BodyID = @import("Body/BodyID.zig").BodyID;
const BodyCreationSettingsFile = @import("Body/BodyCreationSettings.zig");
const BodyCreationSettings = BodyCreationSettingsFile.BodyCreationSettings;
const CreateBodyError = BodyCreationSettingsFile.CreateBodyError;
const BodyIDVector = @import("Body/BodyManager.zig").BodyIDVector;
const BodyLockRead = @import("Body/BodyLock.zig").BodyLockRead;
const ConstraintFile = @import("Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const TwoBodyConstraintFile = @import("Constraints/TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const PhysicsSystem = @import("PhysicsSystem.zig").PhysicsSystem;

/// Contains the creation settings of a set of bodies
pub const PhysicsScene = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(PhysicsScene)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "PhysicsScene",
        .kind = .non_virtual,
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

    /// Body constant to use to indicate that the constraint is attached to the fixed world
    pub const fixed_to_world: u32 = 0xffffffff;

    /// A constraint and how it is connected to the bodies in the scene
    pub const ConnectedConstraint = struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(PhysicsScene::ConnectedConstraint)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "PhysicsScene::ConnectedConstraint",
            .kind = .non_virtual,
        });
        /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
        pub const serialization_pending = true;

        /// Constraint settings (a TwoBodyConstraintSettings, see `getSettings`)
        settings: RefConst(ConstraintSettings) = .empty,
        /// Index of first body (in mBodies)
        body1: u32 = 0,
        /// Index of second body (in mBodies)
        body2: u32 = 0,

        /// Constructor ConnectedConstraint(settings, body1, body2) (adds a reference to the settings)
        pub fn init(settings: *const TwoBodyConstraintSettings, body1: u32, body2: u32) ConnectedConstraint {
            return .{ .settings = .init(&settings.base), .body1 = body1, .body2 = body2 };
        }

        /// Copy constructor
        pub fn clone(self: *const ConnectedConstraint) ConnectedConstraint {
            return .{ .settings = self.settings.clone(), .body1 = self.body1, .body2 = self.body2 };
        }

        /// Destructor (releases the settings)
        pub fn deinit(self: *ConnectedConstraint) void {
            self.settings.deinit();
        }

        /// The constraint settings (Jolt's `mSettings`)
        pub fn getSettings(self: *const ConnectedConstraint) ?*const TwoBodyConstraintSettings {
            const settings = self.settings.get() orelse return null;
            return virtual.downcast(TwoBodyConstraintSettings, settings);
        }
    };

    pub const PhysicsSceneResult = Result(Ref(PhysicsScene));

    /// RefTarget<PhysicsScene>
    ref_count: RefCount = .{},
    /// Allocates the arrays and frees the scene (and the maps of the binary state)
    allocator: Allocator,

    /// The bodies that are part of this scene
    bodies: std.ArrayList(BodyCreationSettings) = .empty,

    /// Constraints that are part of this scene
    constraints: std.ArrayList(ConnectedConstraint) = .empty,

    /// Constructor of a scene that is not on the heap (`deinit` it)
    pub fn init(allocator: Allocator) PhysicsScene {
        return .{ .allocator = allocator };
    }

    /// new PhysicsScene(): reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*PhysicsScene {
        const self = try allocator.create(PhysicsScene);
        self.* = .init(allocator);
        return self;
    }

    /// new PhysicsScene (RTTI create)
    pub const createDefault = create;

    /// Destructor: releases the references of the bodies and constraints and frees the arrays
    pub fn deinit(self: *PhysicsScene) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *PhysicsScene) void {
        for (self.bodies.items) |*b| b.deinit();
        self.bodies.deinit(self.allocator);
        for (self.constraints.items) |*c| c.deinit();
        self.constraints.deinit(self.allocator);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const PhysicsScene) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const PhysicsScene) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this scene as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const PhysicsScene) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const PhysicsScene) u32 {
        return self.ref_count.get();
    }

    /// Add a body to the scene (a copy that holds references to the shape and the group filter)
    pub fn addBody(self: *PhysicsScene, body: *const BodyCreationSettings) Allocator.Error!void {
        try self.bodies.ensureUnusedCapacity(self.allocator, 1);
        self.bodies.appendAssumeCapacity(body.clone());
    }

    /// Add a constraint to the scene
    /// @param constraint Constraint settings
    /// @param body1 Index in the bodies list of first body to attach constraint to
    /// @param body2 Index in the bodies list of the second body to attach constraint to
    pub fn addConstraint(self: *PhysicsScene, constraint: *const TwoBodyConstraintSettings, body1: u32, body2: u32) Allocator.Error!void {
        try self.constraints.ensureUnusedCapacity(self.allocator, 1);
        self.constraints.appendAssumeCapacity(.init(constraint, body1, body2));
    }

    /// Get number of bodies in this scene
    pub fn getNumBodies(self: *const PhysicsScene) usize {
        return self.bodies.items.len;
    }

    /// Access to the body settings for this scene
    pub fn getBodies(self: *const PhysicsScene) []const BodyCreationSettings {
        return self.bodies.items;
    }

    /// Access to the body settings for this scene (the non-const overload: the array, allocated with `allocator`)
    pub fn getBodiesMut(self: *PhysicsScene) *std.ArrayList(BodyCreationSettings) {
        return &self.bodies;
    }

    /// Get number of constraints in this scene
    pub fn getNumConstraints(self: *const PhysicsScene) usize {
        return self.constraints.items.len;
    }

    /// Access to the constraints for this scene
    pub fn getConstraints(self: *const PhysicsScene) []const ConnectedConstraint {
        return self.constraints.items;
    }

    /// Access to the constraints for this scene (the non-const overload: the array, allocated with `allocator`)
    pub fn getConstraintsMut(self: *PhysicsScene) *std.ArrayList(ConnectedConstraint) {
        return &self.constraints;
    }

    /// Instantiate all bodies, returns false if not all bodies could be created
    /// (Zolt: `error.InvalidShape` for a body without a shape and `error.OutOfMemory`; the bodies and constraints that
    /// were created are then removed and destroyed again)
    pub fn createBodies(self: *const PhysicsScene, system: *PhysicsSystem) CreateBodyError!bool {
        const bi = system.getBodyInterfaceMut();

        var body_ids: BodyIDVector = .empty;
        defer body_ids.deinit(self.allocator);
        try body_ids.ensureTotalCapacity(self.allocator, self.bodies.items.len);

        // Zolt: everything that the rest of the function allocates, so that it can only fail before a body is added
        var temp_body_ids: BodyIDVector = .empty;
        defer temp_body_ids.deinit(self.allocator);
        try temp_body_ids.ensureTotalCapacity(self.allocator, self.bodies.items.len);
        var added_constraints: std.ArrayList(*Constraint) = .empty;
        defer added_constraints.deinit(self.allocator);
        try added_constraints.ensureTotalCapacity(self.allocator, self.constraints.items.len);

        // Zolt: undo on error
        var bodies_added = false;
        errdefer {
            system.removeConstraints(added_constraints.items);
            if (bodies_added) {
                temp_body_ids.clearRetainingCapacity();
                temp_body_ids.appendSliceAssumeCapacity(body_ids.items);
                bi.removeBodies(temp_body_ids.items);
            }
            bi.destroyBodies(body_ids.items);
        }

        // Create bodies
        for (self.bodies.items) |*b| {
            const body = (try bi.createBody(b)) orelse break;
            body_ids.appendAssumeCapacity(body.getID());
        }

        // Create soft bodies (Phase 9: a scene has none)

        // Batch add bodies
        temp_body_ids.appendSliceAssumeCapacity(body_ids.items); // Body ID's get shuffled by AddBodiesPrepare
        const add_state = try bi.addBodiesPrepare(temp_body_ids.items);
        bi.addBodiesFinalize(temp_body_ids.items, add_state, .activate);
        bodies_added = true;

        // If not all bodies are created, creating constraints will be unreliable
        if (body_ids.items.len != self.bodies.items.len)
            return false;

        // Create constraints
        for (self.constraints.items) |*cc| {
            const body1_id: BodyID = if (cc.body1 == fixed_to_world) .invalid else body_ids.items[cc.body1];
            const body2_id: BodyID = if (cc.body2 == fixed_to_world) .invalid else body_ids.items[cc.body2];
            const settings = cc.getSettings() orelse @panic("PhysicsScene::CreateBodies: a constraint without settings (Jolt dereferences a null pointer)");
            const c = try bi.createConstraint(settings, body1_id, body2_id);

            // Jolt: AddConstraint takes the first reference; Zolt: a temporary one, so that the constraint is freed when
            // AddConstraint fails
            var ref = Ref(Constraint).init(&c.base);
            defer ref.deinit();
            try system.addConstraint(&c.base);
            added_constraints.appendAssumeCapacity(&c.base);
        }

        // Everything was created
        return true;
    }

    /// Go through all body creation settings and fix shapes that are scaled incorrectly (note this will change the scene a bit).
    /// @return False when not all scales could be fixed.
    /// (Zolt: the shapes are created and scaled with the scene's allocator; `error.InvalidShape` for a body without a
    /// shape, `error.OutOfMemory`: the bodies before it may have been fixed already)
    pub fn fixInvalidScales(self: *PhysicsScene) CreateBodyError!bool {
        const unit_scale = Vec3.one();

        var success = true;
        for (self.bodies.items) |*b| {
            // Test if there is an invalid scale in the shape hierarchy
            const shape = (try b.getShape(self.allocator)) orelse return error.InvalidShape;
            if (!shape.isValidScale(unit_scale)) {
                // Fix it up
                var result = try shape.scaleShape(self.allocator, unit_scale);
                defer result.deinit();
                if (result.isValid())
                    b.setShape(result.getPtr())
                else
                    success = false;
            }
        }
        return success;
    }

    /// Saves the state of this object in binary form to inStream.
    /// @param stream The stream to save the state to
    /// @param save_shapes If the shapes should be saved as well (these could be shared between physics scenes, in which case the calling application may want to write custom code to restore them)
    /// @param save_group_filter If the group filter should be saved as well (these could be shared)
    /// (Zolt: the maps are allocated with the scene's allocator; `error.InvalidShape` when a body without a shape is
    /// saved with its shape)
    pub fn saveBinaryState(self: *const PhysicsScene, stream: StreamOut, save_shapes: bool, save_group_filter: bool) CreateBodyError!void {
        var shape_to_id: BodyCreationSettings.ShapeToIDMap = .empty;
        defer shape_to_id.deinit(self.allocator);
        var material_to_id: BodyCreationSettings.MaterialToIDMap = .empty;
        defer material_to_id.deinit(self.allocator);
        var group_filter_to_id: BodyCreationSettings.GroupFilterToIDMap = .empty;
        defer group_filter_to_id.deinit(self.allocator);
        // (SoftBodyCreationSettings::SharedSettingsToIDMap settings_to_id: Phase 9)

        // Save bodies
        stream.write(@as(u32, @intCast(self.bodies.items.len)));
        for (self.bodies.items) |*b|
            try b.saveWithChildren(self.allocator, stream, if (save_shapes) &shape_to_id else null, if (save_shapes) &material_to_id else null, if (save_group_filter) &group_filter_to_id else null);

        // Save constraints
        stream.write(@as(u32, @intCast(self.constraints.items.len)));
        for (self.constraints.items) |*cc| {
            const settings = cc.settings.get() orelse @panic("PhysicsScene::SaveBinaryState: a constraint without settings (Jolt dereferences a null pointer)");
            settings.saveBinaryState(stream);
            stream.write(cc.body1);
            stream.write(cc.body2);
        }

        // Save soft bodies (Phase 9: a scene has none)
        stream.write(@as(u32, 0));
    }

    /// Restore a saved scene from inStream (Jolt: sRestoreFromBinaryState; the result holds a reference to the new
    /// scene, `deinit` it)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!PhysicsSceneResult {
        var result: PhysicsSceneResult = .empty;

        // Create scene
        var scene = Ref(PhysicsScene).init(try create(allocator));
        defer scene.deinit();
        const s = scene.get().?;

        var id_to_shape: BodyCreationSettings.IDToShapeMap = .empty;
        defer releaseList(allocator, &id_to_shape);
        var id_to_material: BodyCreationSettings.IDToMaterialMap = .empty;
        defer releaseList(allocator, &id_to_material);
        var id_to_group_filter: BodyCreationSettings.IDToGroupFilterMap = .empty;
        defer releaseList(allocator, &id_to_group_filter);
        // (SoftBodyCreationSettings::IDToSharedSettingsMap id_to_settings: Phase 9)

        // Reserve some memory to avoid frequent reallocations
        try id_to_shape.ensureTotalCapacity(allocator, 1024);
        try id_to_material.ensureTotalCapacity(allocator, 128);
        try id_to_group_filter.ensureTotalCapacity(allocator, 128);

        // Read bodies (Jolt resizes the array to the length that is read, then restores the elements in place)
        var len: u32 = 0;
        stream.read(&len);
        try s.bodies.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            // Read creation settings
            var bcs_result = try BodyCreationSettings.restoreWithChildren(allocator, stream, &id_to_shape, &id_to_material, &id_to_group_filter);
            defer bcs_result.deinit();
            if (bcs_result.hasError()) {
                result.setError(bcs_result.getError());
                return result;
            }
            s.bodies.appendAssumeCapacity(bcs_result.get().clone());
        }

        // Read constraints
        len = 0;
        stream.read(&len);
        try s.constraints.ensureTotalCapacity(allocator, len);
        for (0..len) |_| {
            var c_result = try ConstraintSettings.restoreFromBinaryState(allocator, stream);
            defer c_result.deinit();
            if (c_result.hasError()) {
                result.setError(c_result.getError());
                return result;
            }
            // Jolt: mConstraints.resize(len) default-constructs the elements, mBody1 / mBody2 stay uninitialized when the
            // stream ends inside them (the reads below do not check the stream); Zolt: 0
            var cc: ConnectedConstraint = .{};
            cc.settings.set(c_result.getPtr()); // Jolt: StaticCast<TwoBodyConstraintSettings>
            if (Core.enable_asserts) std.debug.assert(isTwoBodyConstraintSettings(cc.settings.get().?));
            stream.read(&cc.body1);
            stream.read(&cc.body2);
            s.constraints.appendAssumeCapacity(cc);
        }

        // Read soft bodies (Phase 9: SoftBodyCreationSettings has no binary state yet)
        len = 0;
        stream.read(&len);
        if (len != 0) {
            result.setError("Soft bodies are not supported");
            return result;
        }

        result.set(scene.clone());
        return result;
    }

    /// For debugging purposes: Construct a scene from the current state of the physics system
    /// (Zolt: Allocator.Error, the bodies and constraints added before the failure stay in the scene)
    pub fn fromPhysicsSystem(self: *PhysicsScene, system: *const PhysicsSystem) Allocator.Error!void {
        // This map will track where each body went in mBodies
        const BodyIDToIdxMap = UnorderedMap(BodyID, u32, .{});
        var body_id_to_idx: BodyIDToIdxMap = .empty;
        defer body_id_to_idx.deinit(self.allocator);

        // Map invalid ID
        (try body_id_to_idx.getOrPutValue(self.allocator, BodyID.invalid, 0)).* = fixed_to_world;

        // Get all bodies
        var body_ids: BodyIDVector = .empty;
        defer body_ids.deinit(self.allocator);
        try system.getBodies(self.allocator, &body_ids);

        // Loop over all bodies
        const bli = system.getBodyLockInterface();
        for (body_ids.items) |id| {
            var lock: BodyLockRead = .init(bli, id);
            defer lock.deinit();
            if (lock.succeeded()) {
                // Store location of body
                (try body_id_to_idx.getOrPutValue(self.allocator, id, 0)).* = @intCast(self.bodies.items.len);

                const body = lock.getBody();

                // Convert to body creation settings
                if (body.isRigidBody()) {
                    var settings = body.getBodyCreationSettings();
                    defer settings.deinit();
                    try self.addBody(&settings);
                } else @panic("soft bodies: Phase 9"); // AddSoftBody(body.GetSoftBodyCreationSettings()), no soft body can exist yet
            }
        }

        // Loop over all constraints
        var constraints = try system.getConstraints(self.allocator);
        defer {
            for (constraints.items) |*c| c.deinit();
            constraints.deinit(self.allocator);
        }
        for (constraints.items) |c_ref| {
            const c = c_ref.get().?;
            if (c.getType() == .two_body_constraint) {
                // Cast to two body constraint
                const tbc = virtual.downcast(TwoBodyConstraint, c);

                // Find the body indices
                const b1 = body_id_to_idx.find(tbc.getBody1().getID());
                const b2 = body_id_to_idx.find(tbc.getBody2().getID());
                std.debug.assert(b1 != null and b2 != null);

                // Create constraint settings and add the constraint
                var settings = try c.getConstraintSettings();
                defer settings.deinit();
                if (settings.get()) |s| {
                    try self.addConstraint(virtual.downcast(TwoBodyConstraintSettings, s), b1.?.value, b2.?.value);
                } else {
                    // Jolt: AddConstraint(nullptr, ...), a constraint without settings. GetConstraintSettings returns
                    // nullptr where it is not implemented (PathConstraint), after an assert: reached with asserts off
                    try self.constraints.append(self.allocator, .{ .body1 = b1.?.value, .body2 = b2.?.value });
                }
            }
        }
    }

    /// Jolt's StaticCast<TwoBodyConstraintSettings> asserts the RTTI: true when the class of `settings` derives from
    /// TwoBodyConstraintSettings
    fn isTwoBodyConstraintSettings(settings: *const ConstraintSettings) bool {
        return settings.vtable.rtti.isKindOf(TwoBodyConstraintSettings.rtti);
    }

    fn releaseList(allocator: Allocator, list: anytype) void {
        for (list.items) |*item| item.deinit();
        list.deinit(allocator);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the inline tests register TestConstraintSettings through zolt_user_types, so its settings can be restored)

const testing = std.testing;
const Quat = @import("../Math/Quat.zig").Quat;
const RVec3 = @import("../Math/Real.zig").RVec3;
const Color = @import("../Core/Color.zig").Color;
const StreamWrapper = @import("../Core/StreamWrapper.zig");
const Body = @import("Body/Body.zig").Body;
const BroadPhaseLayerInterfaceTable = @import("Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
const ObjectVsBroadPhaseLayerFilterTable = @import("Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
const ObjectLayerPairFilterTable = @import("Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
const ShapeFile = @import("Collision/Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeSettings = ShapeFile.ShapeSettings;
const BoxShape = @import("Collision/Shape/BoxShape.zig").BoxShape;
const SphereShape = @import("Collision/Shape/SphereShape.zig").SphereShape;
const SphereShapeSettings = @import("Collision/Shape/SphereShape.zig").SphereShapeSettings;
const ScaledShape = @import("Collision/Shape/ScaledShape.zig").ScaledShape;
const PhysicsMaterial = @import("Collision/PhysicsMaterial.zig").PhysicsMaterial;
const PhysicsMaterialSimple = @import("Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const GroupFilter = @import("Collision/GroupFilter.zig").GroupFilter;
const GroupFilterTable = @import("Collision/GroupFilterTable.zig").GroupFilterTable;
const TestConstraints = @import("Constraints/TestConstraints.zig");
const TestConstraint = TestConstraints.TestConstraint;
const TestConstraintSettings = TestConstraints.TestConstraintSettings;

/// A physics system with one object layer (in place: it must not move)
const TestSystem = struct {
    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,

    fn init(self: *TestSystem, allocator: Allocator, max_bodies: u32) !void {
        self.layer_interface = try .init(allocator, 1, 1);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(0, .init(0));
        self.pair_filter = try .init(allocator, 1);
        errdefer self.pair_filter.deinit();
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 1, &self.pair_filter.base, 1);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, testing.io, max_bodies, 0, 64, 64, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
    }

    fn deinit(self: *TestSystem) void {
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }
};

/// A static box, a dynamic sphere from shape settings and a kinematic body that shares the box shape (and a group
/// filter with the sphere), with a constraint between body 1 and 2 and one between the world and body 1
fn buildScene(allocator: Allocator, scene: *PhysicsScene) !void {
    const material = try PhysicsMaterialSimple.create(allocator, "Ice", Color.white);
    var material_ref = RefConst(PhysicsMaterial).init(material.material());
    defer material_ref.deinit();
    const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 4 });
    var filter_ref = RefConst(GroupFilter).init(filter.asGroupFilter());
    defer filter_ref.deinit();
    filter.disableCollision(0, 2);
    const box = try BoxShape.create(allocator, Vec3.init(4, 0.5, 4), .{ .material = material.material() });
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const sphere = try SphereShapeSettings.create(allocator, 0.5, .{});
    var sphere_ref = Ref(ShapeSettings).init(sphere.asShapeSettings());
    defer sphere_ref.deinit();

    var floor: BodyCreationSettings = .initWithShape(box.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, 0);
    defer floor.deinit();
    floor.friction = 0.9;
    try scene.addBody(&floor);
    var ball: BodyCreationSettings = .initWithSettings(sphere.asShapeSettings(), RVec3.init(0, 5, 0), Quat.identity(), .dynamic, 0);
    defer ball.deinit();
    ball.linear_velocity = Vec3.init(1, 0, 0);
    ball.collision_group = .init(filter.asGroupFilter(), 1, 2);
    try scene.addBody(&ball);
    var platform: BodyCreationSettings = .initWithShape(box.asShape(), RVec3.init(10, 2, 0), Quat.rotation(Vec3.axisY(), 0.5), .kinematic, 0);
    defer platform.deinit();
    platform.collision_group = .init(filter.asGroupFilter(), 1, 0);
    platform.user_data = 42;
    try scene.addBody(&platform);

    const c1 = try TestConstraintSettings.create(allocator);
    var c1_ref = Ref(ConstraintSettings).init(c1.asSettings());
    defer c1_ref.deinit();
    c1.value = 2.5;
    c1.base.base.constraint_priority = 3;
    try scene.addConstraint(&c1.base, 1, 2);
    const c2 = try TestConstraintSettings.create(allocator);
    var c2_ref = Ref(ConstraintSettings).init(c2.asSettings());
    defer c2_ref.deinit();
    c2.value = -1;
    try scene.addConstraint(&c2.base, PhysicsScene.fixed_to_world, 1);
}

/// The shape of the settings when they hold one (Jolt's private mShapePtr)
fn shapePtr(settings: *const BodyCreationSettings) ?*const Shape {
    return switch (settings.shape) {
        .shape => |s| s.get(),
        else => null,
    };
}

/// saveBinaryState into an allocating writer, returns the bytes (owned by the writer)
fn saveScene(scene: *const PhysicsScene, out: *std.Io.Writer.Allocating, save_shapes: bool, save_group_filter: bool) ![]const u8 {
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    try scene.saveBinaryState(wrapper.streamOut(), save_shapes, save_group_filter);
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return out.written();
}

fn restoreScene(allocator: Allocator, bytes: []const u8) Allocator.Error!PhysicsScene.PhysicsSceneResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return PhysicsScene.restoreFromBinaryState(allocator, in.streamIn());
}

test "PhysicsScene: addBody, addConstraint, createBodies and fromPhysicsSystem" {
    const allocator = testing.allocator;
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);
    try testing.expectEqual(@as(usize, 3), scene.getNumBodies());
    try testing.expectEqual(@as(usize, 2), scene.getNumConstraints());
    try testing.expectEqual(@as(u64, 42), scene.getBodies()[2].user_data);
    try testing.expectEqual(PhysicsScene.fixed_to_world, scene.getConstraints()[1].body1);

    var ts: TestSystem = undefined;
    try ts.init(allocator, 16);
    defer ts.deinit();
    try testing.expect(try scene.createBodies(&ts.system));

    // The bodies are added and the non-static ones are active
    try testing.expectEqual(@as(u32, 3), ts.system.getNumBodies());
    try testing.expectEqual(@as(u32, 2), ts.system.getNumActiveBodies(.rigid_body));
    var constraints = try ts.system.getConstraints(allocator);
    defer {
        for (constraints.items) |*c| c.deinit();
        constraints.deinit(allocator);
    }
    try testing.expectEqual(@as(usize, 2), constraints.items.len);
    const tc1 = virtual.downcast(TestConstraint, constraints.items[0].get().?);
    const tc2 = virtual.downcast(TestConstraint, constraints.items[1].get().?);
    try testing.expectEqual(@as(f32, 2.5), tc1.value);
    try testing.expectEqual(@as(u32, 3), tc1.base.base.getConstraintPriority());
    try testing.expectEqual(@as(f32, -1), tc2.value);
    try testing.expect(tc2.base.getBody1() == Body.fixedToWorld());
    try testing.expect(tc2.base.getBody2() == tc1.base.getBody1());

    // Back to a scene: the same bodies and constraints (the body settings as Body.getBodyCreationSettings makes them)
    var copy: PhysicsScene = .init(allocator);
    defer copy.deinit();
    try copy.fromPhysicsSystem(&ts.system);
    try testing.expectEqual(@as(usize, 3), copy.getNumBodies());
    try testing.expectEqual(@as(usize, 2), copy.getNumConstraints());
    for (copy.getBodies(), scene.getBodies()) |*b, *original| {
        try testing.expect(b.position.eql(original.position));
        try testing.expect(b.motion_type == original.motion_type);
        try testing.expect(b.collision_group.eql(&original.collision_group));
        try testing.expectEqual(original.user_data, b.user_data);
    }
    try testing.expect(shapePtr(&copy.getBodies()[0]) == shapePtr(&scene.getBodies()[0]));
    try testing.expectEqual(@as(u32, 1), copy.getConstraints()[0].body1);
    try testing.expectEqual(@as(u32, 2), copy.getConstraints()[0].body2);
    try testing.expectEqual(PhysicsScene.fixed_to_world, copy.getConstraints()[1].body1);
    try testing.expectEqual(@as(u32, 1), copy.getConstraints()[1].body2);
    try testing.expectEqual(@as(f32, -1), virtual.downcast(TestConstraintSettings, copy.getConstraints()[1].getSettings().?).value);

    // A body without a shape (Jolt dereferences null): error.InvalidShape, nothing is left in the system
    var broken: PhysicsScene = .init(allocator);
    defer broken.deinit();
    try buildScene(allocator, &broken);
    var no_shape: BodyCreationSettings = .{};
    defer no_shape.deinit();
    try broken.addBody(&no_shape);
    var ts2: TestSystem = undefined;
    try ts2.init(allocator, 16);
    defer ts2.deinit();
    try testing.expectError(error.InvalidShape, broken.createBodies(&ts2.system));
    try testing.expectEqual(@as(u32, 0), ts2.system.getNumBodies());
    try testing.expectError(error.InvalidShape, broken.fixInvalidScales());
}

test "PhysicsScene: createBodies out of bodies" {
    const allocator = testing.allocator;
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);

    // Room for 2 of the 3 bodies: they are added, the constraints are not created, the result is false
    var ts: TestSystem = undefined;
    try ts.init(allocator, 2);
    defer ts.deinit();
    try testing.expect(!try scene.createBodies(&ts.system));
    try testing.expectEqual(@as(u32, 2), ts.system.getNumBodies());
    try testing.expectEqual(@as(u32, 1), ts.system.getNumActiveBodies(.rigid_body));
    var constraints = try ts.system.getConstraints(allocator);
    defer constraints.deinit(allocator);
    try testing.expectEqual(@as(usize, 0), constraints.items.len);
}

fn createBodiesOutOfMemory(allocator: Allocator) !void {
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);
    var ts: TestSystem = undefined;
    try ts.init(allocator, 16);
    defer ts.deinit();
    const ok = scene.createBodies(&ts.system) catch |err| {
        // All or nothing: no body and no constraint is left
        try testing.expectEqual(@as(u32, 0), ts.system.getNumBodies());
        try testing.expectEqual(@as(u32, 0), ts.system.constraint_manager.getNumConstraints());
        return err;
    };
    try testing.expect(ok);
}

fn fromPhysicsSystemOutOfMemory(allocator: Allocator) !void {
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);
    var ts: TestSystem = undefined;
    try ts.init(allocator, 16);
    defer ts.deinit();
    try testing.expect(try scene.createBodies(&ts.system));
    var copy: PhysicsScene = .init(allocator);
    defer copy.deinit();
    try copy.fromPhysicsSystem(&ts.system);
}

test "PhysicsScene: out of memory in createBodies and fromPhysicsSystem" {
    try testing.checkAllAllocationFailures(testing.allocator, createBodiesOutOfMemory, .{});
    try testing.checkAllAllocationFailures(testing.allocator, fromPhysicsSystemOutOfMemory, .{});
}

test "PhysicsScene: saveBinaryState / restoreFromBinaryState" {
    const allocator = testing.allocator;
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);

    for ([_]bool{ true, false }) |save_shapes| {
        for ([_]bool{ true, false }) |save_group_filter| {
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            const bytes = try saveScene(&scene, &out, save_shapes, save_group_filter);

            var result = try restoreScene(allocator, bytes);
            defer result.deinit();
            try testing.expect(result.isValid());
            const restored = result.getPtr().?;
            try testing.expectEqual(@as(u32, 1), restored.getRefCount());
            try testing.expectEqual(@as(usize, 3), restored.getNumBodies());
            try testing.expectEqual(@as(usize, 2), restored.getNumConstraints());

            // Every saved field (not the user data), the shapes and group filters as selected
            for (restored.getBodies(), scene.getBodies()) |*b, *original| {
                var expected = original.clone();
                defer expected.deinit();
                expected.user_data = 0;
                expected.setShape(shapePtr(b));
                expected.collision_group.setGroupFilter(b.collision_group.getGroupFilter());
                try testing.expect(b.eql(&expected));
                try testing.expect((shapePtr(b) != null) == save_shapes);
                try testing.expect((b.collision_group.getGroupFilter() != null) == (save_group_filter and original.collision_group.getGroupFilter() != null));
            }
            if (save_shapes) {
                try testing.expect(shapePtr(&restored.getBodies()[0]) == shapePtr(&restored.getBodies()[2]));
                try testing.expectEqual(@as(f32, 0.5), shapePtr(&restored.getBodies()[1]).?.getInnerRadius());
                try testing.expectEqualStrings("Ice", shapePtr(&restored.getBodies()[0]).?.getMaterial(.empty).getDebugName());
            }
            if (save_group_filter)
                try testing.expect(restored.getBodies()[1].collision_group.getGroupFilter() == restored.getBodies()[2].collision_group.getGroupFilter());
            for (restored.getConstraints(), scene.getConstraints()) |*c, *original| {
                try testing.expectEqual(original.body1, c.body1);
                try testing.expectEqual(original.body2, c.body2);
                const s = virtual.downcast(TestConstraintSettings, c.getSettings().?);
                const o = virtual.downcast(TestConstraintSettings, original.getSettings().?);
                try testing.expectEqual(o.value, s.value);
                try testing.expectEqual(o.base.base.constraint_priority, s.base.base.constraint_priority);
            }

            // Saving the restored scene gives the same bytes
            var out2: std.Io.Writer.Allocating = .init(allocator);
            defer out2.deinit();
            try testing.expectEqualSlices(u8, bytes, try saveScene(restored, &out2, save_shapes, save_group_filter));
        }
    }
}

test "PhysicsScene: restoreFromBinaryState errors" {
    const allocator = testing.allocator;
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveScene(&scene, &out, true, true);

    // Truncated streams: an error with one of Jolt's texts or a scene with fewer parts (Jolt checks the EOF only while
    // reading a body or a constraint), nothing leaks
    for (0..bytes.len) |len| {
        var result = try restoreScene(allocator, bytes[0..len]);
        defer result.deinit();
        try testing.expect(result.hasError() or result.getPtr().?.getNumConstraints() < 2 or len >= bytes.len - 12); // The body indices of the last constraint and the soft body count
    }

    // The soft bodies of Phase 9
    const with_soft_bodies = try allocator.dupe(u8, bytes);
    defer allocator.free(with_soft_bodies);
    std.mem.writeInt(u32, with_soft_bodies[bytes.len - 4 ..][0..4], 1, .little);
    {
        var result = try restoreScene(allocator, with_soft_bodies);
        defer result.deinit();
        try testing.expectEqualStrings("Soft bodies are not supported", result.getError());
    }

    // Constraint settings of a type that is not registered
    var unknown: PhysicsScene = .init(allocator);
    defer unknown.deinit();
    const unknown_settings = try TestConstraintSettings.create(allocator);
    var unknown_ref = Ref(ConstraintSettings).init(unknown_settings.asSettings());
    defer unknown_ref.deinit();
    try unknown.addConstraint(&unknown_settings.base, 0, 1);
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    const unknown_bytes = try allocator.dupe(u8, try saveScene(&unknown, &out2, true, true));
    defer allocator.free(unknown_bytes);
    std.mem.writeInt(u32, unknown_bytes[8..12], 0x12345678, .little); // The RTTI hash after the body and constraint counts
    {
        var result = try restoreScene(allocator, unknown_bytes);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }

    // A body without a shape cannot be saved with its shape (Jolt dereferences null)
    var no_shape: BodyCreationSettings = .{};
    defer no_shape.deinit();
    try unknown.addBody(&no_shape);
    var out3: std.Io.Writer.Allocating = .init(allocator);
    defer out3.deinit();
    try testing.expectError(error.InvalidShape, saveScene(&unknown, &out3, true, false));
    _ = try saveScene(&unknown, &out3, false, false);
}

fn saveRestoreOutOfMemory(allocator: Allocator) !void {
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try buildScene(allocator, &scene);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveScene(&scene, &out, true, true);
    var result = try restoreScene(allocator, bytes);
    defer result.deinit();
    try testing.expect(result.isValid());
}

test "PhysicsScene: out of memory in saveBinaryState / restoreFromBinaryState" {
    try testing.checkAllAllocationFailures(testing.allocator, saveRestoreOutOfMemory, .{});
}

test "PhysicsScene: fixInvalidScales and the reference count" {
    const allocator = testing.allocator;
    var scene = Ref(PhysicsScene).init(try PhysicsScene.create(allocator));
    defer scene.deinit();

    // A sphere scaled non uniformly is not a valid scale, a uniformly scaled one is
    const sphere = try SphereShape.create(allocator, 1, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    const bad = try ScaledShape.create(allocator, sphere.asShape(), Vec3.init(1, 2, 1));
    var bad_ref = RefConst(Shape).init(bad.asShape());
    defer bad_ref.deinit();
    const good = try ScaledShape.create(allocator, sphere.asShape(), Vec3.replicate(2));
    var good_ref = RefConst(Shape).init(good.asShape());
    defer good_ref.deinit();
    try testing.expect(!bad.asShape().isValidScale(Vec3.one()));
    try testing.expect(good.asShape().isValidScale(Vec3.one()));

    var b1: BodyCreationSettings = .initWithShape(bad.asShape(), RVec3.zero(), Quat.identity(), .dynamic, 0);
    defer b1.deinit();
    var b2: BodyCreationSettings = .initWithShape(good.asShape(), RVec3.zero(), Quat.identity(), .dynamic, 0);
    defer b2.deinit();
    try scene.get().?.addBody(&b1);
    try scene.get().?.addBody(&b2);

    try testing.expect(try scene.get().?.fixInvalidScales());
    const bodies = scene.get().?.getBodies();
    try testing.expect(shapePtr(&bodies[0]) != bad.asShape());
    try testing.expect(shapePtr(&bodies[0]).?.isValidScale(Vec3.one()));
    try testing.expect(shapePtr(&bodies[1]) == good.asShape());

    // A second reference keeps the scene alive
    var second = scene.clone();
    try testing.expectEqual(@as(u32, 2), scene.get().?.getRefCount());
    second.deinit();
}

fn fixInvalidScalesOutOfMemory(allocator: Allocator) !void {
    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    const sphere = try SphereShape.create(allocator, 1, .{});
    var sphere_ref = RefConst(Shape).init(sphere.asShape());
    defer sphere_ref.deinit();
    const bad = try ScaledShape.create(allocator, sphere.asShape(), Vec3.init(1, 2, 1));
    var b1: BodyCreationSettings = .initWithShape(bad.asShape(), RVec3.zero(), Quat.identity(), .dynamic, 0);
    defer b1.deinit();
    try scene.addBody(&b1);
    try testing.expect(try scene.fixInvalidScales());
}

test "PhysicsScene: out of memory in fixInvalidScales" {
    try testing.checkAllAllocationFailures(testing.allocator, fixInvalidScalesOutOfMemory, .{});
}
