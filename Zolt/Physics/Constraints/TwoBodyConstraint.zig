//! Port of: Jolt/Physics/Constraints/TwoBodyConstraint.h, Jolt/Physics/Constraints/TwoBodyConstraint.cpp
//! Status: complete
//!
//! SimulationArchitecture D8:
//! - `TwoBodyConstraintSettings` extends ConstraintSettings.VTable with Jolt's pure virtual
//!   `TwoBodyConstraintSettings::Create(Body &, Body &)`, renamed `createConstraint(allocator, body1, body2)` because
//!   `create(allocator, ...)` is the porting guide's `new XSettings`. A concrete settings class lists it in its
//!   `overrides`. It returns the new constraint with reference count 0, like Jolt's `new`.
//! - `TwoBodyConstraint` extends Constraint.VTable with the two matrix functions and keeps its bodies of `GetType`,
//!   `IsActive`, `BuildIslands` and `BuildIslandSplits` in `impl` (it is abstract).
//! - The bodies are `*Body` like Jolt's `Body *`, also when a body is `Body.fixedToWorld()` (read-only memory): the
//!   solver only writes dynamic bodies.
//! - JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(TwoBodyConstraintSettings):
//!   TODO(serialization).
//! - JPH_DEBUG_RENDERER (`DrawConstraintReferenceFrame`): TODO(debug_renderer).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyManager = @import("../Body/BodyManager.zig").BodyManager;
const IslandBuilder = @import("../IslandBuilder.zig").IslandBuilder;
const LargeIslandSplitter = @import("../LargeIslandSplitter.zig").LargeIslandSplitter;
const ConstraintFile = @import("Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const ConstraintType = ConstraintFile.ConstraintType;

/// Base class for settings for all constraints that involve 2 bodies
pub const TwoBodyConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, TwoBodyConstraintSettings)

    /// The virtual function that TwoBodyConstraintSettings adds
    pub const VTable = struct {
        base: ConstraintSettings.VTable,

        /// Create an instance of this constraint
        /// You can use Body.fixedToWorld() for body1 if you want to attach body2 to the world
        /// (Jolt: Create, renamed: `create(allocator, ...)` is the constructor `new XSettings`)
        createConstraint: *const fn (self: *const TwoBodyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint,
    };

    base: ConstraintSettings,

    /// Constructor of the introducing class: builds the vtable of the most derived class T
    pub fn init(comptime T: type, allocator: Allocator) TwoBodyConstraintSettings {
        return .{ .base = .initWithVTable(&vtableFor(T).base, allocator) };
    }

    /// The vtable of settings class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    fn getVTable(self: *const TwoBodyConstraintSettings) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// Create an instance of this constraint (virtual dispatcher, see VTable.createConstraint)
    pub fn createConstraint(self: *const TwoBodyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        return self.getVTable().createConstraint(self, allocator, body1, body2);
    }

    /// Abstract: createConstraint is pure virtual
    pub const impl = struct {};
};

/// Base class for all constraints that involve 2 bodies. Body1 is usually considered the parent, Body2 the child.
pub const TwoBodyConstraint = struct {
    /// The virtual functions that TwoBodyConstraint adds
    pub const VTable = struct {
        base: Constraint.VTable,

        // TODO(debug_renderer): DrawConstraintReferenceFrame (an override of Constraint's)

        /// Calculates the transform that transforms from constraint space to body 1 space. The first column of the matrix is the primary constraint axis (e.g. the hinge axis / slider direction), second column the secondary etc.
        getConstraintToBody1Matrix: *const fn (self: *const TwoBodyConstraint) Mat44,

        /// Calculates the transform that transforms from constraint space to body 2 space. The first column of the matrix is the primary constraint axis (e.g. the hinge axis / slider direction), second column the secondary etc.
        getConstraintToBody2Matrix: *const fn (self: *const TwoBodyConstraint) Mat44,
    };

    base: Constraint,

    /// The two bodies involved (protected in Jolt)
    body1: *Body,
    body2: *Body,

    /// Constructor of the introducing class: builds the vtable of the most derived class T
    pub fn init(comptime T: type, allocator: Allocator, body1: *Body, body2: *Body, settings: *const TwoBodyConstraintSettings) TwoBodyConstraint {
        return .{ .base = .initWithVTable(&vtableFor(T).base, allocator, &settings.base), .body1 = body1, .body2 = body2 };
    }

    /// The vtable of constraint class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    fn getVTable(self: *const TwoBodyConstraint) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// The Constraint base class
    pub fn asConstraint(self: *TwoBodyConstraint) *Constraint {
        return &self.base;
    }

    /// Access to the connected bodies
    pub fn getBody1(self: *const TwoBodyConstraint) *Body {
        return self.body1;
    }

    pub fn getBody2(self: *const TwoBodyConstraint) *Body {
        return self.body2;
    }

    /// Calculates the transform that transforms from constraint space to body 1 space (see VTable)
    pub fn getConstraintToBody1Matrix(self: *const TwoBodyConstraint) Mat44 {
        return self.getVTable().getConstraintToBody1Matrix(self);
    }

    /// Calculates the transform that transforms from constraint space to body 2 space (see VTable)
    pub fn getConstraintToBody2Matrix(self: *const TwoBodyConstraint) Mat44 {
        return self.getVTable().getConstraintToBody2Matrix(self);
    }

    /// TwoBodyConstraint's overrides of Constraint (abstract class: the bodies live in impl)
    pub const impl = struct {
        /// Get the type of a constraint
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
            if (Core.enable_asserts) {
                // Validates that a body that is sleeping has zero velocity.
                self.body1.validateMotion();
                self.body2.validateMotion();
            }

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

        /// Link bodies that are connected by this constraint in the same split. Returns the split index.
        pub fn buildIslandSplits(self: *const TwoBodyConstraint, splitter: *LargeIslandSplitter) u32 {
            return splitter.assignSplit(self.body1, self.body2);
        }
    };
};
