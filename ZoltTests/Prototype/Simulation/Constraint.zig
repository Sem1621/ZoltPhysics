//! Phase 5 architecture prototype: reduced Jolt/Physics/Constraints/{Constraint, TwoBodyConstraint, ConstraintManager}.h/.cpp
//! plus a toy distance constraint standing in for the Phase 6 constraints (Docs/Zolt/SimulationArchitecture.md D8).
//!
//! - Constraint and ConstraintSettings are pattern A RefTarget roots (vtable, ref_count, allocator in the root), one
//!   level per C++ class: `ToyDistanceConstraint.base: TwoBodyConstraint`, `TwoBodyConstraint.base: Constraint`.
//!   TwoBodyConstraint adds virtual functions, so its VTable starts with `base: Constraint.VTable`.
//! - TwoBodyConstraintSettings::Create(Body &, Body &) is the virtual `createConstraint(allocator, body1, body2)`:
//!   `create(allocator, ...)` is the guide's name for `new XSettings`.
//! - ConstraintManager keeps `Ref(Constraint)`s with Jolt's swap-remove indices and its `mutable` mutex in the
//!   `mutable_fields` block (Rule M), locked in Jolt's order through PhysicsLock.
//! - A constraint attached to Body.fixedToWorld() never writes it (the solver checks isDynamic).

const std = @import("std");
const zolt = @import("zolt");
const virtual = zolt.virtual;
const Vec3 = zolt.Vec3;
const Mat44 = zolt.Mat44;
const BodyID = zolt.BodyID;
const Ref = zolt.Ref;
const RefCount = zolt.RefCount;
const Mutex = zolt.Mutex;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const Allocator = std.mem.Allocator;
const Body = @import("Body.zig").Body;
const BodyManagerFile = @import("BodyManager.zig");
const BodyManager = BodyManagerFile.BodyManager;
const PhysicsLockContext = BodyManagerFile.PhysicsLockContext;
const PhysicsLock = BodyManagerFile.PhysicsLock;
const UniqueLock = BodyManagerFile.UniqueLock;
const IslandBuilder = @import("IslandBuilder.zig").IslandBuilder;
const StateRecorder = @import("StateRecorder.zig").StateRecorder;

/// Enum to identify constraint type (EConstraintType)
pub const ConstraintType = enum(u8) {
    constraint,
    two_body_constraint,
};

/// Enum to identify constraint sub type (EConstraintSubType, reduced)
pub const ConstraintSubType = enum(u8) {
    fixed,
    point,
    hinge,
    slider,
    distance,
    user1,
};

/// Class used to store the configuration of a constraint. Allows run-time creation of constraints.
pub const ConstraintSettings = struct {
    pub const VTable = struct {
        deinit: *const fn (self: *ConstraintSettings) void,
        destroy: *const fn (self: *ConstraintSettings) void,
        /// Saves the contents of the constraint settings in binary form to inStream.
        saveBinaryState: *const fn (self: *const ConstraintSettings, stream: StreamOut) void,
        /// This function should not be called directly, it is used by sRestoreFromBinaryState.
        restoreBinaryState: *const fn (self: *ConstraintSettings, stream: StreamIn) Allocator.Error!void,
    };

    vtable: *const VTable,
    ref_count: RefCount = .{},
    allocator: Allocator,

    /// If this constraint is enabled initially. Use Constraint::SetEnabled to toggle after creation.
    enabled: bool = true,
    /// Priority of the constraint when solving. Higher numbers are more likely to be solved correctly.
    constraint_priority: u32 = 0,
    /// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_velocity_steps_override: u32 = 0,
    /// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
    num_position_steps_override: u32 = 0,
    /// Size of constraint when drawing it through the debug renderer
    draw_constraint_size: f32 = 1.0,
    /// User data value (can be used by application)
    user_data: u64 = 0,

    pub fn init(vtable: *const VTable, allocator: Allocator) ConstraintSettings {
        return .{ .vtable = vtable, .allocator = allocator };
    }

    pub fn addRef(self: *const ConstraintSettings) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const ConstraintSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    pub fn saveBinaryState(self: *const ConstraintSettings, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    pub fn restoreBinaryState(self: *ConstraintSettings, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    pub const impl = struct {
        pub fn saveBinaryState(self: *const ConstraintSettings, stream: StreamOut) void {
            // TODO(serialization): stream.write(GetRTTI()->GetHash())
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

/// Base class for settings for all constraints that involve 2 bodies
pub const TwoBodyConstraintSettings = struct {
    pub const VTable = struct {
        base: ConstraintSettings.VTable,
        /// Create an instance of this constraint (Jolt: Create). You can use Body.fixedToWorld() for body1 if you want to
        /// attach body2 to the world. Renamed: `create(allocator, ...)` is Zolt's `new XSettings`.
        createConstraint: *const fn (self: *const TwoBodyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint,
    };

    base: ConstraintSettings,

    /// Constructor of the introducing class: builds the vtable of the most derived class T
    pub fn init(comptime T: type, allocator: Allocator) TwoBodyConstraintSettings {
        return .{ .base = .init(&virtual.vtablePtr(VTable, T).base, allocator) };
    }

    fn getVTable(self: *const TwoBodyConstraintSettings) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// Create an instance of this constraint (virtual dispatcher)
    pub fn createConstraint(self: *const TwoBodyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        return self.getVTable().createConstraint(self, allocator, body1, body2);
    }

    /// Abstract: no implementation of createConstraint
    pub const impl = struct {};
};

/// Base class for all physics constraints. A constraint removes one or more degrees of freedom for a rigid body.
pub const Constraint = struct {
    /// Index that indicates this constraint is not in the constraint manager
    pub const invalid_constraint_index: u32 = 0xffffffff;

    /// One entry per C++ virtual function, in declaration order (reduced: no BuildIslandSplits, GetConstraintSettings,
    /// Draw*, which follow the same rules)
    pub const VTable = struct {
        deinit: *const fn (self: *Constraint) void,
        destroy: *const fn (self: *Constraint) void,
        getType: *const fn (self: *const Constraint) ConstraintType,
        getSubType: *const fn (self: *const Constraint) ConstraintSubType,
        notifyShapeChanged: *const fn (self: *Constraint, body_id: BodyID, delta_com: Vec3) void,
        resetWarmStart: *const fn (self: *Constraint) void,
        isActive: *const fn (self: *const Constraint) bool,
        setupVelocityConstraint: *const fn (self: *Constraint, delta_time: f32) void,
        warmStartVelocityConstraint: *const fn (self: *Constraint, warm_start_impulse_ratio: f32) void,
        solveVelocityConstraint: *const fn (self: *Constraint, delta_time: f32) bool,
        solvePositionConstraint: *const fn (self: *Constraint, delta_time: f32, baumgarte: f32) bool,
        buildIslands: *const fn (self: *Constraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void,
        saveState: *const fn (self: *const Constraint, stream: *StateRecorder) void,
        restoreState: *const fn (self: *Constraint, stream: *StateRecorder) void,
    };

    vtable: *const VTable,
    ref_count: RefCount = .{},
    allocator: Allocator,

    /// Index that indicates this constraint is not in the constraint manager
    constraint_index: u32 = invalid_constraint_index,
    /// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
    constraint_priority: u32 = 0,
    /// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps.
    num_velocity_steps_override: u8 = 0,
    /// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps.
    num_position_steps_override: u8 = 0,
    /// If this constraint is currently enabled
    enabled: bool = true,
    /// User data value (can be used by application)
    user_data: u64 = 0,

    /// Constructor
    pub fn init(vtable: *const VTable, allocator: Allocator, settings: *const ConstraintSettings) Constraint {
        std.debug.assert(settings.num_velocity_steps_override < 256);
        std.debug.assert(settings.num_position_steps_override < 256);
        return .{
            .vtable = vtable,
            .allocator = allocator,
            .constraint_priority = settings.constraint_priority,
            .num_velocity_steps_override = @intCast(settings.num_velocity_steps_override),
            .num_position_steps_override = @intCast(settings.num_position_steps_override),
            .enabled = settings.enabled,
            .user_data = settings.user_data,
        };
    }

    pub fn addRef(self: *const Constraint) void {
        self.ref_count.addRef();
    }

    pub fn release(self: *const Constraint) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    pub fn getConstraintPriority(self: *const Constraint) u32 {
        return self.constraint_priority;
    }
    pub fn setEnabled(self: *Constraint, enabled: bool) void {
        self.enabled = enabled;
    }
    pub fn getEnabled(self: *const Constraint) bool {
        return self.enabled;
    }

    // Dispatchers (virtual calls)
    pub fn getType(self: *const Constraint) ConstraintType {
        return self.vtable.getType(self);
    }
    pub fn getSubType(self: *const Constraint) ConstraintSubType {
        return self.vtable.getSubType(self);
    }
    pub fn notifyShapeChanged(self: *Constraint, body_id: BodyID, delta_com: Vec3) void {
        self.vtable.notifyShapeChanged(self, body_id, delta_com);
    }
    pub fn resetWarmStart(self: *Constraint) void {
        self.vtable.resetWarmStart(self);
    }
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
    pub fn buildIslands(self: *Constraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void {
        self.vtable.buildIslands(self, constraint_index, builder, body_manager);
    }
    pub fn saveState(self: *const Constraint, stream: *StateRecorder) void {
        self.vtable.saveState(self, stream);
    }
    pub fn restoreState(self: *Constraint, stream: *StateRecorder) void {
        self.vtable.restoreState(self, stream);
    }

    /// Bodies of virtual functions with a default implementation (abstract class)
    pub const impl = struct {
        pub fn getType(self: *const Constraint) ConstraintType {
            _ = self;
            return .constraint;
        }
        pub fn isActive(self: *const Constraint) bool {
            return self.enabled;
        }
        pub fn saveState(self: *const Constraint, stream: *StateRecorder) void {
            stream.write(self.enabled);
        }
        pub fn restoreState(self: *Constraint, stream: *StateRecorder) void {
            stream.read(&self.enabled);
        }
    };
};

/// Base class for all constraints that involve 2 bodies. Body1 is usually considered the parent, Body2 the child.
pub const TwoBodyConstraint = struct {
    pub const VTable = struct {
        base: Constraint.VTable,
        /// Calculates the transform that transforms from constraint space to body 1 space.
        getConstraintToBody1Matrix: *const fn (self: *const TwoBodyConstraint) Mat44,
        /// Calculates the transform that transforms from constraint space to body 2 space.
        getConstraintToBody2Matrix: *const fn (self: *const TwoBodyConstraint) Mat44,
    };

    base: Constraint,

    /// The two bodies involved (Jolt: Body *, also when attached to Body::sFixedToWorld)
    body1: *Body,
    body2: *Body,

    /// Constructor of the introducing class (for the most derived class T)
    pub fn init(comptime T: type, allocator: Allocator, body1: *Body, body2: *Body, settings: *const TwoBodyConstraintSettings) TwoBodyConstraint {
        return .{ .base = .init(&virtual.vtablePtr(VTable, T).base, allocator, &settings.base), .body1 = body1, .body2 = body2 };
    }

    fn getVTable(self: *const TwoBodyConstraint) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    pub fn getConstraintToBody1Matrix(self: *const TwoBodyConstraint) Mat44 {
        return self.getVTable().getConstraintToBody1Matrix(self);
    }
    pub fn getConstraintToBody2Matrix(self: *const TwoBodyConstraint) Mat44 {
        return self.getVTable().getConstraintToBody2Matrix(self);
    }

    pub fn getBody1(self: *const TwoBodyConstraint) *Body {
        return self.body1;
    }
    pub fn getBody2(self: *const TwoBodyConstraint) *Body {
        return self.body2;
    }

    /// TwoBodyConstraint overrides of Constraint (abstract class: the bodies live in impl)
    pub const impl = struct {
        pub fn getType(self: *const TwoBodyConstraint) ConstraintType {
            _ = self;
            return .two_body_constraint;
        }

        /// Solver interface
        pub fn isActive(self: *const TwoBodyConstraint) bool {
            return Constraint.impl.isActive(&self.base) and (self.body1.isActive() or self.body2.isActive()) and (self.body2.isDynamic() or self.body1.isDynamic());
        }

        /// Link bodies that are connected by this constraint in the island builder
        pub fn buildIslands(self: *TwoBodyConstraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void {
            const body1_dynamic = self.body1.isDynamic();
            const body2_dynamic = self.body2.isDynamic();

            // Activate bodies
            var body_ids: [2]BodyID = undefined;
            var num_bodies: usize = 0;
            if (body1_dynamic and !self.body1.isActive()) {
                body_ids[num_bodies] = self.body1.getID();
                num_bodies += 1;
            }
            if (body2_dynamic and !self.body2.isActive()) {
                body_ids[num_bodies] = self.body2.getID();
                num_bodies += 1;
            }
            if (num_bodies > 0)
                body_manager.activateBodies(body_ids[0..num_bodies]);

            // Link the two bodies only if both are dynamic. If one of them is static or kinematic they don't need to go into
            // the same simulation island as a constraint cannot affect the velocity of a kinematic body.
            if (body1_dynamic and body2_dynamic)
                builder.linkBodies(self.body1.getIndexInActiveBodiesInternal(), self.body2.getIndexInActiveBodiesInternal());

            // Link the constraint to the first dynamic body
            if (body1_dynamic) {
                builder.linkConstraint(constraint_index, self.body1.getIndexInActiveBodiesInternal());
            } else {
                std.debug.assert(body2_dynamic);
                builder.linkConstraint(constraint_index, self.body2.getIndexInActiveBodiesInternal());
            }
        }
    };
};

/// Toy constraint standing in for a Phase 6 constraint: keeps the relative velocity along the line between the centers
/// of mass of two bodies at zero (a velocity-level rod)
pub const ToyDistanceConstraintSettings = struct {
    pub const overrides = .{.createConstraint};

    base: TwoBodyConstraintSettings,

    pub fn init(allocator: Allocator) ToyDistanceConstraintSettings {
        return .{ .base = .init(ToyDistanceConstraintSettings, allocator) };
    }

    /// Create an instance of this constraint (TwoBodyConstraintSettings::Create)
    pub fn createConstraint(self: *const ToyDistanceConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try allocator.create(ToyDistanceConstraint);
        constraint.* = .init(allocator, body1, body2, self);
        return &constraint.base;
    }
};

pub const ToyDistanceConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .resetWarmStart, .setupVelocityConstraint, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,
    world_space_normal: Vec3 = Vec3.zero(),
    effective_mass: f32 = 0.0,
    total_lambda: f32 = 0.0,

    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const ToyDistanceConstraintSettings) ToyDistanceConstraint {
        return .{ .base = .init(ToyDistanceConstraint, allocator, body1, body2, &settings.base) };
    }

    pub fn asConstraint(self: *ToyDistanceConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn getSubType(self: *const ToyDistanceConstraint) ConstraintSubType {
        _ = self;
        return .distance;
    }

    pub fn notifyShapeChanged(self: *ToyDistanceConstraint, body_id: BodyID, delta_com: Vec3) void {
        _ = self;
        _ = body_id;
        _ = delta_com;
    }

    pub fn resetWarmStart(self: *ToyDistanceConstraint) void {
        self.total_lambda = 0.0;
    }

    pub fn setupVelocityConstraint(self: *ToyDistanceConstraint, delta_time: f32) void {
        _ = delta_time;
        const b1 = self.base.body1;
        const b2 = self.base.body2;
        const delta = b2.getCenterOfMassPosition().sub(b1.getCenterOfMassPosition()).toVec3();
        self.world_space_normal = delta.normalizedOr(Vec3.init(0, 1, 0));
        const inv_m1: f32 = if (b1.isDynamic()) b1.getMotionProperties().inv_mass else 0.0;
        const inv_m2: f32 = if (b2.isDynamic()) b2.getMotionProperties().inv_mass else 0.0;
        const inv_effective_mass = inv_m1 + inv_m2;
        self.effective_mass = if (inv_effective_mass == 0.0) 0.0 else 1.0 / inv_effective_mass;
    }

    pub fn warmStartVelocityConstraint(self: *ToyDistanceConstraint, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        self.applyImpulse(self.total_lambda);
    }

    fn applyImpulse(self: *ToyDistanceConstraint, lambda: f32) void {
        const b1 = self.base.body1;
        const b2 = self.base.body2;
        // Only dynamic bodies are written: Body.fixedToWorld() (static, read-only memory) is never touched
        if (b1.isDynamic()) {
            const mp1 = b1.getMotionProperties();
            mp1.linear_velocity = mp1.linear_velocity.sub(self.world_space_normal.mulScalar(lambda * mp1.inv_mass));
        }
        if (b2.isDynamic()) {
            const mp2 = b2.getMotionProperties();
            mp2.linear_velocity = mp2.linear_velocity.add(self.world_space_normal.mulScalar(lambda * mp2.inv_mass));
        }
    }

    pub fn solveVelocityConstraint(self: *ToyDistanceConstraint, delta_time: f32) bool {
        _ = delta_time;
        const relative_velocity = self.world_space_normal.dot(self.base.body2.getLinearVelocity().sub(self.base.body1.getLinearVelocity()));
        const lambda = -self.effective_mass * relative_velocity;
        self.total_lambda += lambda;
        self.applyImpulse(lambda);
        return lambda != 0.0;
    }

    pub fn solvePositionConstraint(self: *ToyDistanceConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = self;
        _ = delta_time;
        _ = baumgarte;
        return false;
    }

    pub fn saveState(self: *const ToyDistanceConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(&self.base.base, stream); // C++ TwoBodyConstraint::SaveState (inherited from Constraint)
        stream.write(self.total_lambda);
    }

    pub fn restoreState(self: *ToyDistanceConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(&self.base.base, stream);
        stream.read(&self.total_lambda);
    }

    pub fn getConstraintToBody1Matrix(self: *const ToyDistanceConstraint) Mat44 {
        _ = self;
        return Mat44.identity();
    }

    pub fn getConstraintToBody2Matrix(self: *const ToyDistanceConstraint) Mat44 {
        _ = self;
        return Mat44.identity();
    }
};

/// A list of constraints
pub const Constraints = std.ArrayList(Ref(Constraint));

/// A constraint manager manages all constraints of the same type
pub const ConstraintManager = struct {
    /// The C++ `mutable` mutex (Rule M: behind a pointer, locked in const functions)
    const MutableFields = struct {
        constraints_mutex: Mutex = .{},
    };

    allocator: Allocator = undefined,
    io: std.Io = undefined,
    lock_context: PhysicsLockContext = undefined,
    constraints: Constraints = .empty,
    mutable_fields: *MutableFields = undefined,

    pub const empty: ConstraintManager = .{};

    pub fn init(self: *ConstraintManager, allocator: Allocator, io: std.Io, lock_context: PhysicsLockContext) Allocator.Error!void {
        self.* = .{ .allocator = allocator, .io = io, .lock_context = lock_context, .mutable_fields = try allocator.create(MutableFields) };
        self.mutable_fields.* = .{};
    }

    pub fn deinit(self: *ConstraintManager) void {
        for (self.constraints.items) |*c| {
            c.get().?.constraint_index = Constraint.invalid_constraint_index;
            c.deinit();
        }
        self.constraints.deinit(self.allocator);
        self.allocator.destroy(self.mutable_fields);
        self.* = .empty;
    }

    /// Add a new constraint. This is thread safe. Zolt: reserves first, so out of memory adds nothing.
    pub fn add(self: *ConstraintManager, constraints: []const *Constraint) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        try self.constraints.ensureUnusedCapacity(self.allocator, constraints.len);

        for (constraints) |constraint| {
            // Assume this constraint has not been added yet
            std.debug.assert(constraint.constraint_index == Constraint.invalid_constraint_index);

            // Add to the list
            constraint.constraint_index = @intCast(self.constraints.items.len);
            self.constraints.appendAssumeCapacity(.init(constraint));
        }
    }

    /// Remove a constraint. This is thread safe.
    pub fn remove(self: *ConstraintManager, constraints: []const *Constraint) void {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        for (constraints) |constraint| {
            // Reset constraint index for this constraint
            const this_constraint_idx = constraint.constraint_index;
            constraint.constraint_index = Constraint.invalid_constraint_index;
            std.debug.assert(this_constraint_idx != Constraint.invalid_constraint_index);

            // Check if this constraint is somewhere in the middle of the constraints, in this case we need to move the last constraint to this position
            const last_constraint_idx: u32 = @intCast(self.constraints.items.len - 1);
            var removed = self.constraints.items[this_constraint_idx];
            if (this_constraint_idx < last_constraint_idx) {
                const last_constraint = self.constraints.items[last_constraint_idx];
                last_constraint.get().?.constraint_index = this_constraint_idx;
                self.constraints.items[this_constraint_idx] = last_constraint;
            }

            // Pop last constraint
            _ = self.constraints.pop();
            removed.deinit();
        }
    }

    /// Get total number of constraints
    pub fn getNumConstraints(self: *const ConstraintManager) u32 {
        return @intCast(self.constraints.items.len);
    }

    /// Determine the active constraints of a subset of the constraints
    pub fn getActiveConstraints(self: *const ConstraintManager, start_constraint_idx: u32, end_constraint_idx: u32, out_active_constraints: []*Constraint) u32 {
        std.debug.assert(end_constraint_idx <= self.constraints.items.len);

        var num_active_constraints: u32 = 0;
        for (start_constraint_idx..end_constraint_idx) |constraint_idx| {
            const c = self.constraints.items[constraint_idx].get().?;
            std.debug.assert(c.constraint_index == constraint_idx);
            if (c.isActive()) {
                out_active_constraints[num_active_constraints] = c;
                num_active_constraints += 1;
            }
        }
        return num_active_constraints;
    }

    /// Link bodies to form islands (sBuildIslands)
    pub fn buildIslands(active_constraints: []const *Constraint, builder: *IslandBuilder, body_manager: *BodyManager) void {
        for (active_constraints, 0..) |c, constraint_idx|
            c.buildIslands(@intCast(constraint_idx), builder, body_manager);
    }

    /// In order to have a deterministic simulation, we need to sort the constraints of an island before solving them (sSortConstraints)
    pub fn sortConstraints(active_constraints: []const *Constraint, constraint_idx: []u32) void {
        zolt.quickSort(u32, constraint_idx, active_constraints, struct {
            fn lessThan(constraints: []const *Constraint, lhs_idx: u32, rhs_idx: u32) bool {
                const lhs = constraints[lhs_idx];
                const rhs = constraints[rhs_idx];

                if (lhs.getConstraintPriority() != rhs.getConstraintPriority())
                    return lhs.getConstraintPriority() < rhs.getConstraintPriority();

                return lhs.constraint_index < rhs.constraint_index;
            }
        }.lessThan);
    }

    /// Prior to solving the velocity constraints, you must call SetupVelocityConstraints once to precalculate values that are independent of velocity
    pub fn setupVelocityConstraints(active_constraints: []const *Constraint, delta_time: f32) void {
        for (active_constraints) |c|
            c.setupVelocityConstraint(delta_time);
    }

    /// Apply last frame's impulses, must be called prior to SolveVelocityConstraints (reduced: no CalculateSolverSteps)
    pub fn warmStartVelocityConstraints(active_constraints: []const *Constraint, constraint_idx: []const u32, warm_start_impulse_ratio: f32) void {
        for (constraint_idx) |idx|
            active_constraints[idx].warmStartVelocityConstraint(warm_start_impulse_ratio);
    }

    /// This function is called multiple times to iteratively come to a solution that meets all velocity constraints
    pub fn solveVelocityConstraints(active_constraints: []const *Constraint, constraint_idx: []const u32, delta_time: f32) bool {
        var any_impulse_applied = false;
        for (constraint_idx) |idx|
            any_impulse_applied = active_constraints[idx].solveVelocityConstraint(delta_time) or any_impulse_applied;
        return any_impulse_applied;
    }

    /// Lock all constraints. This should only be done during PhysicsSystem::Update().
    pub fn lockAllConstraints(self: *const ConstraintManager) void {
        PhysicsLock.lock(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
    }

    pub fn unlockAllConstraints(self: *const ConstraintManager) void {
        PhysicsLock.unlock(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
    }

    /// Save state of constraints
    pub fn saveState(self: *const ConstraintManager, stream: *StateRecorder) void {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        // Write state of constraints
        const num_constraints: u32 = @intCast(self.constraints.items.len);
        stream.write(num_constraints);
        for (self.constraints.items) |c|
            c.get().?.saveState(stream);
    }

    /// Restore the state of constraints. Returns false if failed.
    pub fn restoreState(self: *ConstraintManager, stream: *StateRecorder) bool {
        var lock: UniqueLock(Mutex) = .init(&self.mutable_fields.constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        var num_constraints: u32 = @intCast(self.constraints.items.len);
        stream.read(&num_constraints);
        if (num_constraints != self.constraints.items.len)
            return false;
        for (self.constraints.items) |c|
            c.get().?.restoreState(stream);
        return true;
    }
};
