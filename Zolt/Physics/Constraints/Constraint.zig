//! Port of: Jolt/Physics/Constraints/Constraint.h, Jolt/Physics/Constraints/Constraint.cpp
//! Status: complete
//!
//! SimulationArchitecture D8: pattern A with `virtual.make`, one level per C++ class (the mechanism of Shape and
//! ShapeSettings, CollisionArchitecture D1 / D2):
//! - `ConstraintSettings` and `Constraint` are RefTarget roots that hold Jolt's data, the reference count and the
//!   allocator that frees the most derived object. They keep the bodies of their virtual functions in `impl` (Jolt's
//!   constructors are protected, GetSubType is pure virtual). ConstraintSettings can still be created, like Jolt's
//!   JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL: `createDefault` (SerializationArchitecture.md S9).
//! - A derived class embeds its parent as `base` and constructs it with the most derived type:
//!   `.base = .init(T, allocator, &settings)` (Constraint) / `.init(T, allocator)` (ConstraintSettings). Two body
//!   constraints go through TwoBodyConstraint, whose VTable extends this one.
//! - `Constraint.VTable` follows Jolt's declaration order: `buildIslands(index, *IslandBuilder, *BodyManager)` and
//!   `buildIslandSplits(*LargeIslandSplitter)` are entries, `getConstraintSettings` returns `Allocator.Error` (it
//!   allocates the settings).
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConstraintSettings): `rtti` with Jolt's
//!   attributes. The binary state is ported like GroupFilter's (CollisionArchitecture D8):
//!   `SaveBinaryState` writes the RTTI hash of the most derived class (the `rtti` vtable data entry that every concrete
//!   settings class declares), and `restoreFromBinaryState` (StreamUtils::RestoreObject) finds the class in
//!   `RegisterTypes.factory`. An unknown hash gives Jolt's "Failed to create instance of type".
//! - `ConstraintSpace` is an `enum(i32)` that accepts every value: Jolt's binary state writes the enum as its
//!   underlying int and reads back any value (a value other than world space means local space).
//! - JPH_DEBUG_RENDERER (RemainingFeaturesArchitecture.md R1, R2): `draw_constraint_size` (mDrawConstraintSize) is
//!   `void` without `-Ddebug_renderer`. With it the constructor copies it from the settings and `ToConstraintSettings`
//!   copies it back, like Jolt: this changes `GetConstraintSettings`, so the parity reference gets the define too.
//!   `DrawConstraint`, `DrawConstraintLimits` and `DrawConstraintReferenceFrame` are option-gated vtable entries
//!   (`virtual.OptionalVirtual`, no slot without the option) with gated dispatchers. `DrawConstraint` is pure virtual, like
//!   Jolt's: every concrete constraint overrides it. The other two have Jolt's empty default bodies.
//!   ConstraintSettings keeps its public `mDrawConstraintSize` in every build (it is serialized).
//! - The protected copy constructor and assignment of ConstraintSettings (they do not copy the reference count) are
//!   `initCopy`, used by the copy functions of derived settings.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const SerializableObject = @import("../../ObjectStream/SerializableObject.zig").SerializableObject;
const virtual = @import("../../Core/Virtual.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const Ref = @import("../../Core/Reference.zig").Ref;
const Result = @import("../../Core/Result.zig").Result;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StreamUtils = @import("../../Core/StreamUtils.zig");
const Core = @import("../../Core/Core.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyManager = @import("../Body/BodyManager.zig").BodyManager;
const IslandBuilder = @import("../IslandBuilder.zig").IslandBuilder;
const LargeIslandSplitter = @import("../LargeIslandSplitter.zig").LargeIslandSplitter;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;

/// Enum to identify constraint type
pub const ConstraintType = enum {
    constraint,
    two_body_constraint,
};

/// Enum to identify constraint sub type
pub const ConstraintSubType = enum {
    fixed,
    point,
    hinge,
    slider,
    distance,
    cone,
    swing_twist,
    six_dof,
    path,
    vehicle,
    rack_and_pinion,
    gear,
    pulley,

    /// User defined constraint types start here
    user1,
    user2,
    user3,
    user4,
};

/// Certain constraints support setting them up in local or world space. This governs what is used.
/// (Jolt's enum class has the underlying type int, which the binary state of the settings writes; like MotorState, every
/// int value is a valid EConstraintSpace, so a restored value outside the named ones is not illegal behavior: the
/// constraints treat everything but WorldSpace as LocalToBodyCOM, as Jolt's `== EConstraintSpace::WorldSpace` checks do)
pub const ConstraintSpace = enum(i32) {
    /// All constraint properties are specified in local space to center of mass of the bodies that are being constrained (so e.g. 'constraint position 1' will be local to body 1 COM, 'constraint position 2' will be local to body 2 COM). Note that this means you need to subtract Shape::GetCenterOfMass() from positions!
    local_to_body_com,
    /// All constraint properties are specified in world space
    world_space,
    /// Any other int value
    _,
};

/// Class used to store the configuration of a constraint. Allows run-time creation of constraints.
pub const ConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "ConstraintSettings",
        .kind = .virtual,
        .bases = &.{SerializableObject},
        .attributes = &.{
            .attr("mEnabled", .enabled),
            .attr("mDrawConstraintSize", .draw_constraint_size),
            .attr("mConstraintPriority", .constraint_priority),
            .attr("mNumVelocityStepsOverride", .num_velocity_steps_override),
            .attr("mNumPositionStepsOverride", .num_position_steps_override),
            .attr("mUserData", .user_data),
        },
    });

    pub const ConstraintResult = Result(Ref(ConstraintSettings));

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// GetRTTI(): JPH_RTTI of the most derived class (a data entry: every concrete class declares `rtti`)
        rtti: *const RTTI,
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *ConstraintSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *ConstraintSettings) void,

        /// Saves the contents of the constraint settings in binary form to inStream.
        saveBinaryState: *const fn (self: *const ConstraintSettings, stream: StreamOut) void,

        /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
        restoreBinaryState: *const fn (self: *ConstraintSettings, stream: StreamIn) Allocator.Error!void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<ConstraintSettings>)
    ref_count: RefCount = .{},
    /// Frees this object (heap settings) and the memory it owns
    allocator: Allocator,

    /// If this constraint is enabled initially. Use Constraint::SetEnabled to toggle after creation.
    enabled: bool = true,

    /// Priority of the constraint when solving. Higher numbers are more likely to be solved correctly.
    /// Note that if you want a deterministic simulation and you cannot guarantee the order in which constraints are added/removed, you can make the priority for all constraints unique to get a deterministic ordering.
    constraint_priority: u32 = 0,

    /// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_velocity_steps_override: u32 = 0,

    /// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_position_steps_override: u32 = 0,

    /// Size of constraint when drawing it through the debug renderer
    draw_constraint_size: f32 = 1.0,

    /// User data value (can be used by application)
    user_data: u64 = 0,

    /// Constructor (protected in Jolt), called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) ConstraintSettings {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// new ConstraintSettings (RTTI create, Jolt's protected default constructor): reference count 0
    pub fn createDefault(allocator: Allocator) Allocator.Error!*ConstraintSettings {
        const self = try allocator.create(ConstraintSettings);
        self.* = .init(ConstraintSettings, allocator);
        return self;
    }

    /// Construct from a vtable (used by the constructors of derived classes that add virtual functions)
    pub fn initWithVTable(vtable: *const VTable, allocator: Allocator) ConstraintSettings {
        return .{ .vtable = vtable, .allocator = allocator };
    }

    /// The protected copy constructor: copies the settings of `other`, not its reference count. `vtable` is the vtable
    /// of the most derived class of the copy.
    pub fn initCopy(vtable: *const VTable, allocator: Allocator, other: *const ConstraintSettings) ConstraintSettings {
        var result = other.*;
        result.vtable = vtable;
        result.ref_count = .{};
        result.allocator = allocator;
        return result;
    }

    /// The vtable of settings class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<ConstraintSettings>

    /// Add a reference to this object
    pub fn addRef(self: *const ConstraintSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const ConstraintSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark these settings as embedded (on the stack or a member), the last release will not destroy them
    pub fn setEmbedded(self: *const ConstraintSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const ConstraintSettings) u32 {
        return self.ref_count.get();
    }

    /// Destructor of settings that are not on the heap (embedded / stack)
    pub fn deinit(self: *ConstraintSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// Saves the contents of the constraint settings in binary form to inStream.
    pub fn saveBinaryState(self: *const ConstraintSettings, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *ConstraintSettings, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Creates a constraint of the correct type and restores its contents from the binary stream inStream.
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!ConstraintResult {
        return StreamUtils.restoreObject(ConstraintSettings, allocator, stream, ConstraintSettings.restoreBinaryState);
    }

    /// GetRTTI()->GetHash() of the most derived class
    pub fn getRTTIHash(self: *const ConstraintSettings) u32 {
        return self.vtable.rtti.hash;
    }

    /// The bodies of the virtual functions of ConstraintSettings
    pub const impl = struct {
        pub fn saveBinaryState(self: *const ConstraintSettings, stream: StreamOut) void {
            stream.write(self.getRTTIHash());
            stream.write(self.enabled);
            stream.write(self.draw_constraint_size);
            stream.write(self.constraint_priority);
            stream.write(self.num_velocity_steps_override);
            stream.write(self.num_position_steps_override);
        }

        pub fn restoreBinaryState(self: *ConstraintSettings, stream: StreamIn) Allocator.Error!void {
            // Type hash read by sRestoreFromBinaryState
            stream.read(&self.enabled);
            stream.read(&self.draw_constraint_size);
            stream.read(&self.constraint_priority);
            stream.read(&self.num_velocity_steps_override);
            stream.read(&self.num_position_steps_override);
        }
    };
};

/// Base class for all physics constraints. A constraint removes one or more degrees of freedom for a rigid body.
pub const Constraint = struct {
    /// Index that indicates this constraint is not in the constraint manager
    pub const invalid_constraint_index: u32 = 0xffffffff;

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *Constraint) void,
        /// delete this (generated)
        destroy: *const fn (self: *Constraint) void,

        /// Get the type of a constraint
        getType: *const fn (self: *const Constraint) ConstraintType,

        /// Get the sub type of a constraint
        getSubType: *const fn (self: *const Constraint) ConstraintSubType,

        /// Notify the constraint that the shape of a body has changed and that its center of mass has moved by inDeltaCOM.
        /// Bodies don't know which constraints are connected to them so the user is responsible for notifying the relevant constraints when a body changes.
        /// @param body_id ID of the body that has changed
        /// @param delta_com The delta of the center of mass of the body (shape->GetCenterOfMass() - shape_before_change->GetCenterOfMass())
        notifyShapeChanged: *const fn (self: *Constraint, body_id: BodyID, delta_com: Vec3) void,

        /// Notify the system that the configuration of the bodies and/or constraint has changed enough so that the warm start impulses should not be applied the next frame.
        /// You can use this function for example when repositioning a ragdoll through Ragdoll::SetPose in such a way that the orientation of the bodies completely changes so that
        /// the previous frame impulses are no longer a good approximation of what the impulses will be in the next frame. Calling this function when there are no big changes
        /// will result in the constraints being much 'softer' than usual so they are more easily violated (e.g. a long chain of bodies might sag a bit if you call this every frame).
        resetWarmStart: *const fn (self: *Constraint) void,

        ///@name Solver interface
        isActive: *const fn (self: *const Constraint) bool,
        setupVelocityConstraint: *const fn (self: *Constraint, delta_time: f32) void,
        warmStartVelocityConstraint: *const fn (self: *Constraint, warm_start_impulse_ratio: f32) void,
        solveVelocityConstraint: *const fn (self: *Constraint, delta_time: f32) bool,
        solvePositionConstraint: *const fn (self: *Constraint, delta_time: f32, baumgarte: f32) bool,

        /// Link bodies that are connected by this constraint in the island builder
        buildIslands: *const fn (self: *Constraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void,

        /// Link bodies that are connected by this constraint in the same split. Returns the split index.
        buildIslandSplits: *const fn (self: *const Constraint, splitter: *LargeIslandSplitter) u32,

        // Drawing interface (JPH_DEBUG_RENDERER: no slot without -Ddebug_renderer)
        drawConstraint: virtual.OptionalVirtual(Core.debug_renderer, *const fn (self: *const Constraint, renderer: *DebugRenderer) void),
        drawConstraintLimits: virtual.OptionalVirtual(Core.debug_renderer, *const fn (self: *const Constraint, renderer: *DebugRenderer) void),
        drawConstraintReferenceFrame: virtual.OptionalVirtual(Core.debug_renderer, *const fn (self: *const Constraint, renderer: *DebugRenderer) void),

        /// Saving state for replay
        saveState: *const fn (self: *const Constraint, stream: *StateRecorder) void,

        /// Restoring state for replay
        restoreState: *const fn (self: *Constraint, stream: *StateRecorder) void,

        /// Debug function to convert a constraint to its settings, note that this will not save to which bodies the constraint is connected to
        /// (Zolt: allocates the settings, Allocator.Error)
        getConstraintSettings: *const fn (self: *const Constraint) Allocator.Error!Ref(ConstraintSettings),
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<Constraint>)
    ref_count: RefCount = .{},
    /// Frees this constraint (heap constraints) and the memory it owns
    allocator: Allocator,

    /// Size of constraint when drawing it through the debug renderer (JPH_DEBUG_RENDERER only, `void` without it)
    draw_constraint_size: if (Core.debug_renderer) f32 else void = if (Core.debug_renderer) 1.0 else {},

    /// Index in the mConstraints list of the ConstraintManager for easy finding (private in Jolt, ConstraintManager is a friend)
    constraint_index: u32 = invalid_constraint_index,

    /// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
    constraint_priority: u32 = 0,

    /// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_velocity_steps_override: u8 = 0,

    /// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_position_steps_override: u8 = 0,

    /// If this constraint is currently enabled
    enabled: bool = true,

    /// User data value (can be used by application)
    user_data: u64 = undefined,

    /// Constructor, called by derived classes that derive from Constraint directly with their most derived type
    pub fn init(comptime T: type, allocator: Allocator, settings: *const ConstraintSettings) Constraint {
        return initWithVTable(vtableFor(T), allocator, settings);
    }

    /// Constructor with the vtable of the most derived class (used by TwoBodyConstraint, whose VTable extends this one)
    pub fn initWithVTable(vtable: *const VTable, allocator: Allocator, settings: *const ConstraintSettings) Constraint {
        // An object stream or a binary state can hold any uint32 here.
        // Jolt asserts only with JPH_ENABLE_ASSERTS. Its release build keeps the low 8 bits (uint8).
        if (Core.enable_asserts) Core.assertRecoverable(settings.num_velocity_steps_override < 256, "inSettings.mNumVelocityStepsOverride < 256", @src());
        if (Core.enable_asserts) Core.assertRecoverable(settings.num_position_steps_override < 256, "inSettings.mNumPositionStepsOverride < 256", @src());
        return .{
            .vtable = vtable,
            .allocator = allocator,
            .constraint_priority = settings.constraint_priority,
            .num_velocity_steps_override = @truncate(settings.num_velocity_steps_override),
            .num_position_steps_override = @truncate(settings.num_position_steps_override),
            .enabled = settings.enabled,
            .user_data = settings.user_data,
            .draw_constraint_size = if (Core.debug_renderer) settings.draw_constraint_size else {},
        };
    }

    /// The vtable of constraint class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<Constraint>

    /// Add a reference to this object
    pub fn addRef(self: *const Constraint) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const Constraint) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this constraint as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const Constraint) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const Constraint) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a constraint that is not on the heap (embedded / stack)
    pub fn deinit(self: *Constraint) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// Get the type of a constraint
    pub fn getType(self: *const Constraint) ConstraintType {
        return self.vtable.getType(self);
    }

    /// Get the sub type of a constraint
    pub fn getSubType(self: *const Constraint) ConstraintSubType {
        return self.vtable.getSubType(self);
    }

    /// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
    /// Note that if you want a deterministic simulation and you cannot guarantee the order in which constraints are added/removed, you can make the priority for all constraints unique to get a deterministic ordering.
    pub fn getConstraintPriority(self: *const Constraint) u32 {
        return self.constraint_priority;
    }

    pub fn setConstraintPriority(self: *Constraint, priority: u32) void {
        self.constraint_priority = priority;
    }

    /// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    pub fn setNumVelocityStepsOverride(self: *Constraint, n: u32) void {
        if (Core.enable_asserts) Core.assertRecoverable(n < 256, "inN < 256", @src()); // Jolt's release build keeps the low 8 bits
        self.num_velocity_steps_override = @truncate(n);
    }

    pub fn getNumVelocityStepsOverride(self: *const Constraint) u32 {
        return self.num_velocity_steps_override;
    }

    /// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    pub fn setNumPositionStepsOverride(self: *Constraint, n: u32) void {
        if (Core.enable_asserts) Core.assertRecoverable(n < 256, "inN < 256", @src()); // Jolt's release build keeps the low 8 bits
        self.num_position_steps_override = @truncate(n);
    }

    pub fn getNumPositionStepsOverride(self: *const Constraint) u32 {
        return self.num_position_steps_override;
    }

    /// Enable / disable this constraint. This can e.g. be used to implement a breakable constraint by detecting that the constraint impulse
    /// (see e.g. PointConstraint::GetTotalLambdaPosition) went over a certain limit and then disabling the constraint.
    /// Note that although a disabled constraint will not affect the simulation in any way anymore, it does incur some processing overhead.
    /// Alternatively you can remove a constraint from the constraint manager (which may be more costly if you want to disable the constraint for a short while).
    pub fn setEnabled(self: *Constraint, enabled: bool) void {
        self.enabled = enabled;
    }

    /// Test if a constraint is enabled.
    pub fn getEnabled(self: *const Constraint) bool {
        return self.enabled;
    }

    /// Access to the user data, can be used for anything by the application
    pub fn getUserData(self: *const Constraint) u64 {
        return self.user_data;
    }

    pub fn setUserData(self: *Constraint, user_data: u64) void {
        self.user_data = user_data;
    }

    // Drawing interface (JPH_DEBUG_RENDERER only: gated dispatchers, `void` without -Ddebug_renderer)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintDispatch else {};
    fn drawConstraintDispatch(self: *const Constraint, renderer: *DebugRenderer) void {
        self.vtable.drawConstraint(self, renderer);
    }

    pub const drawConstraintLimits = if (Core.debug_renderer) drawConstraintLimitsDispatch else {};
    fn drawConstraintLimitsDispatch(self: *const Constraint, renderer: *DebugRenderer) void {
        self.vtable.drawConstraintLimits(self, renderer);
    }

    pub const drawConstraintReferenceFrame = if (Core.debug_renderer) drawConstraintReferenceFrameDispatch else {};
    fn drawConstraintReferenceFrameDispatch(self: *const Constraint, renderer: *DebugRenderer) void {
        self.vtable.drawConstraintReferenceFrame(self, renderer);
    }

    /// Size of constraint when drawing it through the debug renderer (JPH_DEBUG_RENDERER only: a gated declaration,
    /// RemainingFeaturesArchitecture.md R2)
    pub const getDrawConstraintSize = if (Core.debug_renderer) getDrawConstraintSizeImpl else {};
    fn getDrawConstraintSizeImpl(self: *const Constraint) f32 {
        return self.draw_constraint_size;
    }

    pub const setDrawConstraintSize = if (Core.debug_renderer) setDrawConstraintSizeImpl else {};
    fn setDrawConstraintSizeImpl(self: *Constraint, size: f32) void {
        self.draw_constraint_size = size;
    }

    /// Notify the constraint that the shape of a body has changed (see VTable.notifyShapeChanged)
    pub fn notifyShapeChanged(self: *Constraint, body_id: BodyID, delta_com: Vec3) void {
        self.vtable.notifyShapeChanged(self, body_id, delta_com);
    }

    /// Notify the system that the warm start impulses should not be applied the next frame (see VTable.resetWarmStart)
    pub fn resetWarmStart(self: *Constraint) void {
        self.vtable.resetWarmStart(self);
    }

    ///@name Solver interface
    pub fn isActive(self: *const Constraint) bool {
        return self.vtable.isActive(self);
    }

    pub fn setupVelocityConstraint(self: *Constraint, delta_time: f32) void {
        self.vtable.setupVelocityConstraint(self, delta_time);
    }

    pub fn warmStartVelocityConstraint(self: *Constraint, warm_start_impulse_ratio: f32) void {
        self.vtable.warmStartVelocityConstraint(self, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *Constraint, delta_time: f32) bool {
        return self.vtable.solveVelocityConstraint(self, delta_time);
    }

    pub fn solvePositionConstraint(self: *Constraint, delta_time: f32, baumgarte: f32) bool {
        return self.vtable.solvePositionConstraint(self, delta_time, baumgarte);
    }

    /// Link bodies that are connected by this constraint in the island builder
    pub fn buildIslands(self: *Constraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void {
        self.vtable.buildIslands(self, constraint_index, builder, body_manager);
    }

    /// Link bodies that are connected by this constraint in the same split. Returns the split index.
    pub fn buildIslandSplits(self: *const Constraint, splitter: *LargeIslandSplitter) u32 {
        return self.vtable.buildIslandSplits(self, splitter);
    }

    /// Saving state for replay
    pub fn saveState(self: *const Constraint, stream: *StateRecorder) void {
        self.vtable.saveState(self, stream);
    }

    /// Restoring state for replay
    pub fn restoreState(self: *Constraint, stream: *StateRecorder) void {
        self.vtable.restoreState(self, stream);
    }

    /// Debug function to convert a constraint to its settings, note that this will not save to which bodies the constraint is connected to
    pub fn getConstraintSettings(self: *const Constraint) Allocator.Error!Ref(ConstraintSettings) {
        return self.vtable.getConstraintSettings(self);
    }

    /// Helper function to copy settings back to constraint settings for this base class (protected in Jolt)
    pub fn toConstraintSettings(self: *const Constraint, out_settings: *ConstraintSettings) void {
        out_settings.enabled = self.enabled;
        out_settings.constraint_priority = self.constraint_priority;
        out_settings.num_velocity_steps_override = self.num_velocity_steps_override;
        out_settings.num_position_steps_override = self.num_position_steps_override;
        out_settings.user_data = self.user_data;
        if (Core.debug_renderer) out_settings.draw_constraint_size = self.draw_constraint_size; // JPH_IF_DEBUG_RENDERER
    }

    /// The bodies of the virtual functions of Constraint that have a default implementation
    pub const impl = struct {
        pub fn getType(self: *const Constraint) ConstraintType {
            _ = self;
            return .constraint;
        }

        pub fn isActive(self: *const Constraint) bool {
            return self.enabled;
        }

        pub const drawConstraintLimits = if (Core.debug_renderer) drawConstraintLimitsImpl else {};
        fn drawConstraintLimitsImpl(self: *const Constraint, renderer: *DebugRenderer) void {
            _ = .{ self, renderer };
        }

        pub const drawConstraintReferenceFrame = if (Core.debug_renderer) drawConstraintReferenceFrameImpl else {};
        fn drawConstraintReferenceFrameImpl(self: *const Constraint, renderer: *DebugRenderer) void {
            _ = .{ self, renderer };
        }

        pub fn saveState(self: *const Constraint, stream: *StateRecorder) void {
            stream.write(self.enabled);
        }

        pub fn restoreState(self: *Constraint, stream: *StateRecorder) void {
            stream.read(&self.enabled);
        }
    };
};

test "ConstraintSettings: creatable like Jolt's JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL, restored from its own hash" {
    const testing = std.testing;
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    const settings = try ConstraintSettings.createDefault(allocator);
    var ref = Ref(ConstraintSettings).init(settings);
    defer ref.deinit();
    try testing.expect(settings.vtable.rtti == ConstraintSettings.rtti);
    settings.constraint_priority = 3;
    settings.enabled = false;

    // The base class binary state: the RTTI hash, then Jolt's members (21 bytes)
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 21), out.written().len);
    try testing.expectEqual(RTTI.hashName("ConstraintSettings"), std.mem.readInt(u32, out.written()[0..4], .little));

    // RegisterTypes.factory creates a ConstraintSettings for this hash
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper = StreamWrapper.StreamInWrapper.init(&reader);
    var restored = try ConstraintSettings.restoreFromBinaryState(allocator, in_wrapper.streamIn());
    defer restored.deinit();
    try testing.expect(restored.getPtr().?.vtable.rtti == ConstraintSettings.rtti);
    try testing.expectEqual(@as(u32, 3), restored.getPtr().?.constraint_priority);
    try testing.expect(!restored.getPtr().?.enabled);
}

test "Constraint: step overrides of 256 or more truncate like Jolt's uint8 in its release build" {
    // An object stream or a binary state can hold any uint32 in the settings.
    // Jolt asserts only with JPH_ENABLE_ASSERTS. Its release build keeps the low 8 bits.
    const testing = std.testing;
    var expect_assert: @import("../../Core/IssueReporting.zig").ExpectAssert = undefined;
    expect_assert.init(4); // The two of the constructor, one per setter
    errdefer expect_assert.restore();

    var velocity_steps: u32 = 300;
    var position_steps: u32 = 513;
    _ = .{ &velocity_steps, &position_steps };
    const settings = try ConstraintSettings.createDefault(testing.allocator);
    var ref = Ref(ConstraintSettings).init(settings);
    defer ref.deinit();
    settings.num_velocity_steps_override = velocity_steps;
    settings.num_position_steps_override = position_steps;
    const vtable: Constraint.VTable = undefined; // Not called
    var constraint = Constraint.initWithVTable(&vtable, testing.allocator, settings);
    try testing.expectEqual(@as(u32, 44), constraint.getNumVelocityStepsOverride());
    try testing.expectEqual(@as(u32, 1), constraint.getNumPositionStepsOverride());

    constraint.setNumVelocityStepsOverride(velocity_steps + 1);
    constraint.setNumPositionStepsOverride(position_steps + 1);
    try expect_assert.finish();
    try testing.expectEqual(@as(u32, 45), constraint.getNumVelocityStepsOverride());
    try testing.expectEqual(@as(u32, 2), constraint.getNumPositionStepsOverride());
}
