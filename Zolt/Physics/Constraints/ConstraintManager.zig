//! Port of: Jolt/Physics/Constraints/ConstraintManager.h, Jolt/Physics/Constraints/ConstraintManager.cpp
//! Status: complete
//!
//! SimulationArchitecture D8:
//! - Initialized in place like the BodyManager (`var cm: ConstraintManager = .empty; try cm.init(allocator, io,
//!   lock_context)`): the C++ `mutable` mutex lives in the heap block `mutable_fields` (Rule M), so `getConstraints`,
//!   `saveState` and the lock functions keep `*const` receivers. The lock context is passed in every build (Jolt only
//!   stores it with JPH_ENABLE_ASSERTS, PhysicsLock D3).
//! - `mConstraints` is `constraints: Constraints` (`std.ArrayList(Ref(Constraint))`), Jolt's swap-remove keeps the
//!   indices of the constraints in `Constraint.constraint_index`.
//! - Errors: `add` reserves first (Jolt: `reserve` before the loop), so out of memory adds nothing; `getConstraints`
//!   (a copy) and the filtered `saveState` (a temporary array) allocate and return `Allocator.Error`.
//! - Pointer + count parameters are slices; `Constraint **outActiveConstraints, uint32 &outNumActiveConstraints` is
//!   `out_active_constraints: []*Constraint` plus the returned count. The static `sXxx` functions drop the `s`.
//! - `sWarmStartVelocityConstraints<ConstraintCallback>` takes `callback: anytype`, a pointer to a CalculateSolverSteps
//!   or DummyCalculateSolverSteps (or anything with `call(comptime Type, *const Type)`, Jolt's `operator ()`).
//! - JPH_DEBUG_RENDERER (`DrawConstraints`, `DrawConstraintLimits`, `DrawConstraintReferenceFrame`):
//!   TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const zolt_sort = @import("../../Core/QuickSort.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const Mutex = @import("../../Core/Mutex.zig").Mutex;
const PhysicsLockFile = @import("../PhysicsLock.zig");
const PhysicsLock = PhysicsLockFile.PhysicsLock;
const PhysicsLockContext = PhysicsLockFile.PhysicsLockContext;
const UniqueLock = PhysicsLockFile.UniqueLock;
const StateRecorderFile = @import("../StateRecorder.zig");
const StateRecorder = StateRecorderFile.StateRecorder;
const StateRecorderFilter = StateRecorderFile.StateRecorderFilter;
const BodyManager = @import("../Body/BodyManager.zig").BodyManager;
const IslandBuilder = @import("../IslandBuilder.zig").IslandBuilder;
const Constraint = @import("Constraint.zig").Constraint;

/// A list of constraints
pub const Constraints = std.ArrayList(Ref(Constraint));

/// A constraint manager manages all constraints of the same type
///
/// WARNING: This class is an internal part of PhysicsSystem, it has no functions that can be called by users of the library.
/// Its functionality is exposed through PhysicsSystem and BodyInterface.
pub const ConstraintManager = struct {
    /// The C++ `mutable` members that const functions lock (Rule M: behind a pointer, never through @constCast)
    const MutableFields = struct {
        constraints_mutex: Mutex = .{},
    };

    /// Allocates the list of constraints (and the copies of getConstraints / the filtered saveState)
    allocator: Allocator = undefined,

    /// The io of the mutex
    io: std.Io = undefined,

    /// The lock context of the physics system (Jolt: only with JPH_ENABLE_ASSERTS)
    lock_context: PhysicsLockContext = undefined,

    constraints: Constraints = .empty,

    /// The mutable mutex (allocated at init)
    mutable_fields: ?*MutableFields = null,

    /// Not initialized, call init
    pub const empty: ConstraintManager = .{};

    /// Constructor (Jolt: ConstraintManager(PhysicsLockContext inContext)), allocates the mutable mutex
    pub fn init(self: *ConstraintManager, allocator: Allocator, io: std.Io, lock_context: PhysicsLockContext) Allocator.Error!void {
        std.debug.assert(self.mutable_fields == null); // Init twice
        const mutable_fields = try allocator.create(MutableFields);
        mutable_fields.* = .{};
        self.* = .{ .allocator = allocator, .io = io, .lock_context = lock_context, .mutable_fields = mutable_fields };
    }

    /// Destructor: releases the references to the constraints
    pub fn deinit(self: *ConstraintManager) void {
        const mutable_fields = self.mutable_fields orelse return; // Not initialized
        for (self.constraints.items) |*c|
            c.deinit();
        self.constraints.deinit(self.allocator);
        self.allocator.destroy(mutable_fields);
        self.* = .empty;
    }

    fn mutableFields(self: *const ConstraintManager) *MutableFields {
        return self.mutable_fields.?;
    }

    /// Add a new constraint. This is thread safe.
    /// Zolt: the list is reserved before the first constraint is added, so out of memory adds nothing.
    pub fn add(self: *ConstraintManager, constraints: []const *Constraint) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        try self.constraints.ensureTotalCapacity(self.allocator, self.constraints.items.len + constraints.len);

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
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        for (constraints) |constraint| {
            // Reset constraint index for this constraint
            const this_constraint_idx = constraint.constraint_index;
            constraint.constraint_index = Constraint.invalid_constraint_index;
            std.debug.assert(this_constraint_idx != Constraint.invalid_constraint_index);

            // The reference that the list held, released after the list no longer contains it (the C++ assignment and
            // pop_back release it at the same points)
            var removed = self.constraints.items[this_constraint_idx];

            // Check if this constraint is somewhere in the middle of the constraints, in this case we need to move the last constraint to this position
            const last_constraint_idx: u32 = @intCast(self.constraints.items.len - 1);
            if (this_constraint_idx < last_constraint_idx) {
                const last_constraint = self.constraints.items[last_constraint_idx];
                last_constraint.get().?.constraint_index = this_constraint_idx;
                self.constraints.items[this_constraint_idx] = last_constraint;
            }

            // Pop last constraint
            self.constraints.items.len -= 1;
            removed.deinit();
        }
    }

    /// Get a list of all constraints
    /// Zolt: a copy allocated with `allocator`, every entry holds a reference: release them with `deinit` and free the
    /// list with `allocator`.
    pub fn getConstraints(self: *const ConstraintManager, allocator: Allocator) Allocator.Error!Constraints {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        var copy: Constraints = try .initCapacity(allocator, self.constraints.items.len);
        for (self.constraints.items) |c|
            copy.appendAssumeCapacity(c.clone());
        return copy;
    }

    /// Get total number of constraints
    pub fn getNumConstraints(self: *const ConstraintManager) u32 {
        return @intCast(self.constraints.items.len);
    }

    /// Determine the active constraints of a subset of the constraints
    /// (returns outNumActiveConstraints, the active constraints are written to the start of out_active_constraints)
    pub fn getActiveConstraints(self: *const ConstraintManager, start_constraint_idx: u32, end_constraint_idx: u32, out_active_constraints: []*Constraint) u32 {
        std.debug.assert(end_constraint_idx <= self.constraints.items.len);

        var num_active_constraints: u32 = 0;
        var constraint_idx = start_constraint_idx;
        while (constraint_idx < end_constraint_idx) : (constraint_idx += 1) {
            const c = self.constraints.items[constraint_idx].get().?;
            std.debug.assert(c.constraint_index == constraint_idx);
            if (c.isActive()) {
                out_active_constraints[num_active_constraints] = c;
                num_active_constraints += 1;
            }
        }

        return num_active_constraints;
    }

    /// Link bodies to form islands
    pub fn buildIslands(active_constraints: []const *Constraint, builder: *IslandBuilder, body_manager: *BodyManager) void {
        for (active_constraints, 0..) |c, constraint_idx|
            c.buildIslands(@intCast(constraint_idx), builder, body_manager);
    }

    /// In order to have a deterministic simulation, we need to sort the constraints of an island before solving them
    pub fn sortConstraints(active_constraints: []const *Constraint, constraint_idx: []u32) void {
        zolt_sort.quickSort(u32, constraint_idx, active_constraints, struct {
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

    /// Apply last frame's impulses, must be called prior to SolveVelocityConstraints
    /// `callback`: pointer to a CalculateSolverSteps / DummyCalculateSolverSteps (Jolt's ConstraintCallback)
    pub fn warmStartVelocityConstraints(active_constraints: []const *Constraint, constraint_idx: []const u32, warm_start_impulse_ratio: f32, callback: anytype) void {
        for (constraint_idx) |idx| {
            const c = active_constraints[idx];
            callback.call(Constraint, c);
            c.warmStartVelocityConstraint(warm_start_impulse_ratio);
        }
    }

    /// This function is called multiple times to iteratively come to a solution that meets all velocity constraints
    pub fn solveVelocityConstraints(active_constraints: []const *Constraint, constraint_idx: []const u32, delta_time: f32) bool {
        var any_impulse_applied = false;

        for (constraint_idx) |idx| {
            const c = active_constraints[idx];
            // Jolt: any_impulse_applied |= ..., the constraint is always solved
            any_impulse_applied = c.solveVelocityConstraint(delta_time) or any_impulse_applied;
        }

        return any_impulse_applied;
    }

    /// This function is called multiple times to iteratively come to a solution that meets all position constraints
    pub fn solvePositionConstraints(active_constraints: []const *Constraint, constraint_idx: []const u32, delta_time: f32, baumgarte: f32) bool {
        var any_impulse_applied = false;

        for (constraint_idx) |idx| {
            const c = active_constraints[idx];
            // Jolt: any_impulse_applied |= ..., the constraint is always solved
            any_impulse_applied = c.solvePositionConstraint(delta_time, baumgarte) or any_impulse_applied;
        }

        return any_impulse_applied;
    }

    // TODO(debug_renderer): DrawConstraints, DrawConstraintLimits, DrawConstraintReferenceFrame

    /// Save state of constraints
    /// Zolt: Allocator.Error from the temporary array of the filtered path (nothing is written then)
    pub fn saveState(self: *const ConstraintManager, stream: *StateRecorder, filter: ?*const StateRecorderFilter) Allocator.Error!void {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        // Write state of constraints
        if (filter) |f| {
            // Determine which constraints to save
            var constraints: std.ArrayList(*const Constraint) = try .initCapacity(self.allocator, self.constraints.items.len);
            defer constraints.deinit(self.allocator);
            for (self.constraints.items) |c| {
                if (f.shouldSaveConstraint(c.get().?))
                    constraints.appendAssumeCapacity(c.get().?);
            }

            // Save them
            const num_constraints: u32 = @intCast(constraints.items.len);
            stream.write(num_constraints);
            for (constraints.items) |c| {
                stream.write(c.constraint_index);
                c.saveState(stream);
            }
        } else {
            // Save all constraints
            const num_constraints: u32 = @intCast(self.constraints.items.len);
            stream.write(num_constraints);
            for (self.constraints.items) |ref| {
                const c = ref.get().?;
                stream.write(c.constraint_index);
                c.saveState(stream);
            }
        }
    }

    /// Restore the state of constraints. Returns false if failed.
    pub fn restoreState(self: *ConstraintManager, stream: *StateRecorder) bool {
        var lock: UniqueLock(Mutex) = .init(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
        defer lock.deinit();

        if (stream.isValidating()) {
            // Read state of constraints
            var num_constraints: u32 = @intCast(self.constraints.items.len); // Initialize to current value for validation
            stream.read(&num_constraints);
            if (num_constraints != self.constraints.items.len) {
                if (Core.enable_asserts) @panic("Cannot handle adding/removing constraints");
                return false;
            }
            for (self.constraints.items) |ref| {
                const c = ref.get().?;
                var constraint_index = c.constraint_index;
                stream.read(&constraint_index);
                if (constraint_index != c.constraint_index) {
                    if (Core.enable_asserts) @panic("Unexpected constraint index");
                    return false;
                }
                c.restoreState(stream);
            }
        } else {
            // Not validating, use more flexible reading, read number of constraints
            var num_constraints: u32 = 0;
            stream.read(&num_constraints);

            var idx: u32 = 0;
            while (idx < num_constraints) : (idx += 1) {
                var constraint_index: u32 = undefined;
                stream.read(&constraint_index);
                if (self.constraints.items.len <= constraint_index) {
                    if (Core.enable_asserts) @panic("Restoring state for non-existing constraint");
                    return false;
                }
                self.constraints.items[constraint_index].get().?.restoreState(stream);
            }
        }

        return true;
    }

    /// Lock all constraints. This should only be done during PhysicsSystem::Update().
    pub fn lockAllConstraints(self: *const ConstraintManager) void {
        PhysicsLock.lock(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
    }

    pub fn unlockAllConstraints(self: *const ConstraintManager) void {
        PhysicsLock.unlock(&self.mutableFields().constraints_mutex, self.io, self.lock_context, .constraints_list);
    }
};
