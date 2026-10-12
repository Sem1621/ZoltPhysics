//! Port of: Jolt/Physics/Constraints/FixedConstraint.h, Jolt/Physics/Constraints/FixedConstraint.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(FixedConstraintSettings): TODO(serialization)
//! (the binary state is ported). JPH_DEBUG_RENDERER (DrawConstraint): TODO(debug_renderer).
//!
//! # Porter template: a Phase 6 constraint file
//!
//! FixedConstraint.zig and PointConstraint.zig are the reference ports; every other Phase 6 file
//! (`Zolt/Physics/Constraints/XConstraint.zig`, a stub with the final type names until its worker replaces it) follows
//! them. SimulationArchitecture D8 is the binding design, this is how it looks in a file.
//!
//! **Settings** (`XConstraintSettings`, Jolt's `final` class derived from TwoBodyConstraintSettings):
//! - `base: TwoBodyConstraintSettings`, then Jolt's public members with Jolt's defaults and doc comments
//!   (`mSpace` -> `space: ConstraintSpace = .world_space`, `mAutoDetectPoint` -> `auto_detect_point`, `RVec3` points,
//!   `Vec3` axes, SpringSettings / MotorSettings values, arrays as Zig arrays).
//! - `pub const rtti_name = "XConstraintSettings";` (the RTTI hash of the binary state is
//!   `ConstraintSettings.rttiHash(rtti_name)`, Jolt's `GetRTTI()->GetHash()`, written by ConstraintSettings.impl).
//! - `pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };` in Jolt's declaration
//!   order: compare with the `override` list of the C++ class one by one, an omitted override silently uses the base
//!   class version.
//! - Constructors: `init(allocator)` (the default constructor, for settings on the stack or in a member; the owner
//!   calls `asSettings().deinit()`), `create(allocator) !*X` (`new XConstraintSettings`, reference count 0: wrap it in a
//!   `Ref(ConstraintSettings)` or hand it to a function that takes a reference), `createDefault` (= `create`, the
//!   default constructor that `ConstraintSettings.restoreFromBinaryState` calls through
//!   `RegisterTypes.constraint_settings_types`), `initCopy(allocator, other)` (Jolt's implicit copy constructor: copies
//!   the settings, not the reference count, and adds references to what the settings hold, e.g. a path).
//!   Helper functions of the C++ class (`SetSliderAxis`, `SetRatio`, `MakeFreeAxis`, ...) are methods.
//! - Upcasts: `asSettings() *ConstraintSettings`, `asTwoBodySettings() *TwoBodyConstraintSettings` (+ `Const`
//!   variants). A settings class that owns references (PathConstraintSettings.mPath) releases them in
//!   `destruct(self: *X) void` (the generated destructor chain calls it, see Core/Virtual.zig).
//! - `saveBinaryState(self: *const X, stream: StreamOut) void` / `restoreBinaryState(self: *X, stream: StreamIn)
//!   Allocator.Error!void`: `ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream)` first (Jolt's
//!   `ConstraintSettings::SaveBinaryState`, it writes the RTTI hash), then every member in Jolt's order with
//!   `stream.write` / `stream.read` (enums keep Jolt's underlying type: `ConstraintSpace` is an `enum(i32)` that
//!   accepts every int, `SpringSettings` / `MotorSettings` have their own binary state). The bytes are Jolt's (parity).
//! - `createConstraint(self: *const X, allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint`:
//!   `new XConstraint(inBody1, inBody2, *this)`, returns `&constraint.base` with reference count 0.
//!
//! **Constraint** (`XConstraint`, Jolt's `final` class derived from TwoBodyConstraint):
//! - `base: TwoBodyConstraint`, then Jolt's private members in Jolt's order with Jolt's comments. Members that Jolt
//!   leaves uninitialized are `= undefined` (they are written before they are read: SetupVelocityConstraint runs before
//!   the solver functions), members with an initializer keep it (`motor_state: MotorState = .off`).
//! - The ConstraintPart members are value fields (`point_constraint_part: PointConstraintPart = .{}`), arrays of parts
//!   are arrays (`motor_constraint_part: [3]AngleConstraintPart`). Their functions take the bodies as `*Body`
//!   (`self.base.body1`), like Jolt's `*mBody1`.
//! - `pub const overrides = .{ ... }`: every C++ `override` of the class in header order (`getSubType`,
//!   `notifyShapeChanged`, `setupVelocityConstraint`, `resetWarmStart`, `warmStartVelocityConstraint`,
//!   `solveVelocityConstraint`, `solvePositionConstraint`, `saveState`, `restoreState`, `isActive` (PathConstraint),
//!   `getConstraintSettings`, `getConstraintToBody1Matrix`, `getConstraintToBody2Matrix`; `drawConstraint*` are
//!   TODO(debug_renderer)). The class is `final`, so its own non-virtual helpers may be called statically.
//! - Constructor: `init(allocator, body1, body2, settings: *const XConstraintSettings) X` builds the base with
//!   `.base = .init(X, allocator, body1, body2, &settings.base)` (TwoBodyConstraint makes the vtable of the most derived
//!   class) and then runs the C++ constructor body statement by statement (the world / local space conversions with
//!   `body.getInverseCenterOfMassTransform().mulRVec3(p).toVec3()`, `RVec3.fromVec3(v)` for `RVec3(v)`).
//!   `create(allocator, body1, body2, settings) !*X` is `new XConstraint(...)`. A constraint that owns references
//!   (Gear / RackAndPinion / Path) releases them in `destruct`.
//! - `TwoBodyConstraint::SaveState(inStream)` is `Constraint.impl.saveState(&self.base.base, stream)` (TwoBodyConstraint
//!   does not override it), then the parts and members in Jolt's order; RestoreState mirrors it.
//! - `GetConstraintSettings` is `getConstraintSettings(self: *const X) Allocator.Error!Ref(ConstraintSettings)`:
//!   `XConstraintSettings.create(self.base.base.allocator)`, `self.base.base.toConstraintSettings(settings.asSettings())`,
//!   the members, `return .init(settings.asSettings())` (Jolt returns a `Ref`, the caller owns one reference).
//! - Public setters / getters keep Jolt's names (`setPoint1`, `getLocalSpacePoint1`, `getTotalLambdaPosition`,
//!   `getMotorSettingsMut` for the non-const overload that returns a reference, guide section 4).
//! - Upcasts: `asConstraint() *Constraint`, `asTwoBodyConstraint() *TwoBodyConstraint` (+ `Const` variants). The checked
//!   downcast from a `*Constraint` is `virtual.downcast(XConstraint, two_body)` after checking `getSubType()`.
//!
//! **Registration.** Zolt/zolt.zig re-exports the public types (the stub file is already in `source_files`), and
//! `RegisterTypes.constraint_settings_types` already lists the settings class in Jolt's RegisterTypes.cpp order: the
//! porter only removes `pub const is_stub = true;` from the settings (ConstraintSettings.restoreFromBinaryState skips
//! stubs, they give Jolt's "Failed to create instance of type"). Set `//! Status:` and `//! Missing:`, regenerate
//! Docs/Zolt/Progress.md.
//!
//! **Harnesses.**
//! - ZoltParity/Physics/ConstraintsParity.zig (+ ConstraintsReference.cpp, whose C++ side exists for every kind): fill
//!   the slot `XKind` of the class (makeSettings from the `CSettings` description, applyOp for the kind specific setter
//!   operations, recordGetters) exactly like the C++ functions of the same names; the test "Constraint parity: x" then
//!   runs instead of being skipped (the file header describes the steps and the reserved numbers).
//! - ZoltParity/Physics/SimulationParity.zig: the `.create_constraint` prong of `applyOpInner` gets the
//!   `SimScenes.ConstraintKind` of the class (the C++ side in SimulationReference.cpp creates every kind already); scenes
//!   with that kind then run instead of being skipped.
//! - ZoltParity/Physics/BodyInterfaceParity.zig (PhysicsScene with constraints): a kind of `SceneConstraintKind`.
//!
//! **Tests to write.** The Jolt TEST_CASEs of `UnitTests/Physics/XConstraintTests.cpp` (same names and values) in
//! `ZoltTests/Physics/XConstraintTests.zig`; inline tests of the class: the virtual dispatch through Constraint /
//! TwoBodyConstraint (getType, getSubType, the matrices), the binary state round trip and `restoreFromBinaryState`
//! through the registry, the space conversions of the constructor, `getConstraintSettings`, save / restore state,
//! `std.testing.checkAllAllocationFailures` over everything that allocates (create, createConstraint,
//! getConstraintSettings, restoreFromBinaryState), a simulation through PhysicsTestContext; the whole suite also runs
//! with -Doptimize=ReleaseFast, -Ddouble_precision=true and -Duse_llvm=false.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RealFile = @import("../../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const ConstraintFile = @import("Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const ConstraintSpace = ConstraintFile.ConstraintSpace;
const ConstraintSubType = ConstraintFile.ConstraintSubType;
const TwoBodyConstraintFile = @import("TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const RotationEulerConstraintPart = @import("ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;

/// Fixed constraint settings, used to create a fixed constraint
pub const FixedConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, FixedConstraintSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "FixedConstraintSettings";
    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// When mSpace is WorldSpace mPoint1 and mPoint2 can be automatically calculated based on the positions of the bodies when the constraint is created (they will be fixated in their current relative position/orientation). Set this to false if you want to supply the attachment points yourself.
    auto_detect_point: bool = false,

    /// Body 1 constraint reference frame (space determined by mSpace)
    point1: RVec3 = RVec3.zero(),
    axis_x1: Vec3 = Vec3.axisX(),
    axis_y1: Vec3 = Vec3.axisY(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    point2: RVec3 = RVec3.zero(),
    axis_x2: Vec3 = Vec3.axisX(),
    axis_y2: Vec3 = Vec3.axisY(),

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) FixedConstraintSettings {
        return .{ .base = .init(FixedConstraintSettings, allocator) };
    }

    /// new FixedConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*FixedConstraintSettings {
        const self = try allocator.create(FixedConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const FixedConstraintSettings) FixedConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *FixedConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const FixedConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *FixedConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const FixedConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const FixedConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.auto_detect_point);
        stream.write(self.point1);
        stream.write(self.axis_x1);
        stream.write(self.axis_y1);
        stream.write(self.point2);
        stream.write(self.axis_x2);
        stream.write(self.axis_y2);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const FixedConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try FixedConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *FixedConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.auto_detect_point);
        stream.read(&self.point1);
        stream.read(&self.axis_x1);
        stream.read(&self.axis_y1);
        stream.read(&self.point2);
        stream.read(&self.axis_x2);
        stream.read(&self.axis_y2);
    }
};

/// A fixed constraint welds two bodies together removing all degrees of freedom between them.
/// This variant uses Euler angles for the rotation constraint.
pub const FixedConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Inverse of initial rotation from body 1 to body 2 in body 1 space
    inv_initial_orientation: Quat = undefined,

    // RUN TIME PROPERTIES FOLLOW

    // The constraint parts
    rotation_constraint_part: RotationEulerConstraintPart = .{},
    point_constraint_part: PointConstraintPart = .{},

    /// Constructor
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const FixedConstraintSettings) FixedConstraint {
        var self: FixedConstraint = .{ .base = .init(FixedConstraint, allocator, body1, body2, &settings.base) };

        // Store inverse of initial rotation from body 1 to body 2 in body 1 space
        self.inv_initial_orientation = RotationEulerConstraintPart.getInvInitialOrientationXY(settings.axis_x1, settings.axis_y1, settings.axis_x2, settings.axis_y2);

        if (settings.space == .world_space) {
            if (settings.auto_detect_point) {
                // Determine anchor point: If any of the bodies can never be dynamic use the other body as anchor point
                var anchor: RVec3 = undefined;
                if (!body1.canBeKinematicOrDynamic()) {
                    anchor = body2.getCenterOfMassPosition();
                } else if (!body2.canBeKinematicOrDynamic()) {
                    anchor = body1.getCenterOfMassPosition();
                } else {
                    // Otherwise use weighted anchor point towards the lightest body
                    const inv_m1: Real = body1.getMotionPropertiesUnchecked().?.getInverseMassUnchecked();
                    const inv_m2: Real = body2.getMotionPropertiesUnchecked().?.getInverseMassUnchecked();
                    const total_inv_mass = inv_m1 + inv_m2;
                    if (total_inv_mass != 0.0)
                        anchor = body1.getCenterOfMassPosition().mulScalar(inv_m1).add(body2.getCenterOfMassPosition().mulScalar(inv_m2)).divScalar(inv_m1 + inv_m2)
                    else
                        anchor = body1.getCenterOfMassPosition();
                }

                // Store local positions
                self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(anchor).toVec3();
                self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(anchor).toVec3();
            } else {
                // Store local positions
                self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3();
                self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3();
            }

            // Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
            // => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
            self.inv_initial_orientation = body2.getRotation().conjugated().mul(self.inv_initial_orientation).mul(body1.getRotation());
        } else {
            // Store local positions
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_position2 = settings.point2.toVec3();
        }

        return self;
    }

    /// new FixedConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const FixedConstraintSettings) Allocator.Error!*FixedConstraint {
        const self = try allocator.create(FixedConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *FixedConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const FixedConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *FixedConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const FixedConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const FixedConstraint) ConstraintSubType {
        _ = self;
        return .fixed;
    }

    pub fn notifyShapeChanged(self: *FixedConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    pub fn setupVelocityConstraint(self: *FixedConstraint, delta_time: f32) void {
        _ = delta_time;

        // Calculate constraint values that don't change when the bodies don't change position
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.base.body2, rotation2);
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.local_space_position1, self.base.body2, rotation2, self.local_space_position2);
    }

    pub fn resetWarmStart(self: *FixedConstraint) void {
        self.rotation_constraint_part.deactivate();
        self.point_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *FixedConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.rotation_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.point_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *FixedConstraint, delta_time: f32) bool {
        _ = delta_time;

        // Solve rotation constraint
        const rot = self.rotation_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        // Solve position constraint
        const pos = self.point_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        return rot or pos;
    }

    pub fn solvePositionConstraint(self: *FixedConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        // Solve rotation constraint
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()));
        const rot = self.rotation_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.inv_initial_orientation, baumgarte);

        // Solve position constraint
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.local_space_position1, self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()), self.local_space_position2);
        const pos = self.point_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, baumgarte);

        return rot or pos;
    }

    // TODO(debug_renderer): DrawConstraint

    pub fn saveState(self: *const FixedConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.rotation_constraint_part.saveState(stream);
        self.point_constraint_part.saveState(stream);
    }

    pub fn restoreState(self: *FixedConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.rotation_constraint_part.restoreState(stream);
        self.point_constraint_part.restoreState(stream);
    }

    pub fn getConstraintSettings(self: *const FixedConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try FixedConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.axis_x1 = Vec3.axisX();
        settings.axis_y1 = Vec3.axisY();
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        settings.axis_x2 = self.inv_initial_orientation.rotateAxisX();
        settings.axis_y2 = self.inv_initial_orientation.rotateAxisY();
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const FixedConstraint) Mat44 {
        return Mat44.translation(self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const FixedConstraint) Mat44 {
        return Mat44.rotationTranslation(self.inv_initial_orientation, self.local_space_position2);
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const FixedConstraint) Vec3 {
        return self.point_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaRotation(self: *const FixedConstraint) Vec3 {
        return self.rotation_constraint_part.getTotalLambda();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/FixedConstraintTests.zig)

const testing = std.testing;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const TestContext = @import("ConstraintPart/ConstraintPartTestContext.zig");
const virtual = @import("../../Core/Virtual.zig");

/// The binary state of settings
fn testSaveSettings(allocator: Allocator, settings: *const ConstraintSettings) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var out = StreamWrapper.StreamOutWrapper.init(&data.writer);
    settings.saveBinaryState(out.streamOut());
    return data.toOwnedSlice();
}

/// ConstraintSettings.restoreFromBinaryState of bytes
fn testRestoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

test "FixedConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(!settings.auto_detect_point);
    try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
    try testing.expect(settings.axis_x1.eql(Vec3.axisX()) and settings.axis_y1.eql(Vec3.axisY()));
    try testing.expect(settings.axis_x2.eql(Vec3.axisX()) and settings.axis_y2.eql(Vec3.axisY()));
    try testing.expectEqualStrings("FixedConstraintSettings", settings.asSettings().vtable.rtti_name);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 9;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 0.5;
    settings.space = .local_to_body_com;
    settings.auto_detect_point = true;
    settings.point1 = RVec3.init(1, 2, 3);
    settings.axis_x1 = Vec3.axisY();
    settings.axis_y1 = Vec3.axisZ();
    settings.point2 = RVec3.init(-4, 5, -6);
    settings.axis_x2 = Vec3.axisZ();
    settings.axis_y2 = Vec3.axisX();

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), auto detect (bool), the
    // points (Real) and the axes (3 floats each)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 1 + 2 * 3 * @sizeOf(Real) + 4 * 12), bytes.len);
    try testing.expectEqual(ConstraintSettings.rttiHash("FixedConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM

    // Restore through RegisterTypes.constraint_settings_types
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(FixedConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 9), restored.base.base.constraint_priority);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.auto_detect_point);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
        try testing.expect(restored.axis_x1.eql(settings.axis_x1) and restored.axis_y1.eql(settings.axis_y1));
        try testing.expect(restored.axis_x2.eql(settings.axis_x2) and restored.axis_y2.eql(settings.axis_y2));
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated: Jolt's error
    {
        var result = try testRestoreSettings(allocator, bytes[0 .. bytes.len - 1]);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to restore object", result.getError());
    }

    // Out of memory while restoring: the error arrives, nothing leaks
    const Ctx = struct {
        fn run(a: Allocator, b: []const u8) !void {
            var result = try testRestoreSettings(a, b);
            result.deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Ctx.run, .{bytes});

    // The copy constructor copies the settings, not the reference count
    settings.asSettings().setEmbedded();
    settings.asSettings().addRef();
    defer settings.asSettings().release();
    var copy = FixedConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.point2.eql(settings.point2) and copy.axis_y2.eql(settings.axis_y2));
    try testing.expectEqual(@as(u32, 9), copy.base.base.constraint_priority);
}

test "FixedConstraint: virtual dispatch, space conversions, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // World space with given points and axes
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = RVec3.init(1.5, 2.0, 3.5);
    settings.point2 = RVec3.init(1.5, 2.0, 3.5);
    settings.axis_x1 = Vec3.axisX();
    settings.axis_y1 = Vec3.axisY();
    settings.axis_x2 = Vec3.axisY();
    settings.axis_y2 = Vec3.axisX().negate();
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(FixedConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.fixed, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);

    // The constructor took the points to the local space of the bodies (and the orientation difference)
    try testing.expect(c.local_space_position1.isClose(body1.getInverseCenterOfMassTransform().mulRVec3(settings.point1).toVec3(), .{}));
    try testing.expect(c.local_space_position2.isClose(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3(), .{}));
    const expected_orientation = body2.getRotation().conjugated().mul(RotationEulerConstraintPart.getInvInitialOrientationXY(settings.axis_x1, settings.axis_y1, settings.axis_x2, settings.axis_y2)).mul(body1.getRotation());
    try testing.expect(c.inv_initial_orientation.eql(expected_orientation));
    try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.translation(c.local_space_position1)));
    try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.rotationTranslation(c.inv_initial_orientation, c.local_space_position2)));

    // GetConstraintSettings: local space, axes from the initial orientation; a constraint made from them is the same
    {
        var copy = try base.getConstraintSettings();
        defer copy.deinit();
        const s = virtual.downcast(FixedConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
        try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
        try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
        try testing.expect(s.axis_x1.eql(Vec3.axisX()) and s.axis_y1.eql(Vec3.axisY()));
        var again = Ref(Constraint).init(&(try s.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer again.deinit();
        const c2 = virtual.downcast(FixedConstraint, virtual.downcast(TwoBodyConstraint, again.get().?));
        try testing.expect(c2.local_space_position1.eql(c.local_space_position1) and c2.local_space_position2.eql(c.local_space_position2));
        try testing.expect(c2.inv_initial_orientation.isClose(c.inv_initial_orientation, .{ .max_dist_sq = 1.0e-10 }));
    }

    // Solver: the relative angular velocity of the bodies is removed
    try testing.expect(base.isActive());
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    for (0..10) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
    try testing.expect(body1.getAngularVelocity().isClose(body2.getAngularVelocity(), .{ .max_dist_sq = 1.0e-6 }));
    try testing.expect(!c.getTotalLambdaPosition().eql(Vec3.zero()) and !c.getTotalLambdaRotation().eql(Vec3.zero()));
    try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.2));

    // SaveState / RestoreState: enabled + the lambdas of both parts
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 12 + 12), recorder.getData().len);
    const lambda_position = c.getTotalLambdaPosition();
    const lambda_rotation = c.getTotalLambdaRotation();
    base.resetWarmStart();
    try testing.expect(c.getTotalLambdaPosition().eql(Vec3.zero()) and c.getTotalLambdaRotation().eql(Vec3.zero()));
    base.restoreState(recorder.recorder());
    try testing.expect(c.getTotalLambdaPosition().eql(lambda_position) and c.getTotalLambdaRotation().eql(lambda_rotation));

    // NotifyShapeChanged moves the attachment point of the body that changed
    const p1 = c.local_space_position1;
    const p2 = c.local_space_position2;
    base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
    try testing.expect(c.local_space_position1.eql(p1) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));
    base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))));
    base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
    try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "FixedConstraint: local space and the auto detected anchor" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    // Local space: the points are used as they are
    {
        const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
        defer context.destroyBodies(&bodies);
        var settings = FixedConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.init(0.25, 0.5, 0.75);
        settings.point2 = RVec3.init(-1, -2, -3);
        var c = FixedConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expect(c.local_space_position1.eql(Vec3.init(0.25, 0.5, 0.75)) and c.local_space_position2.eql(Vec3.init(-1, -2, -3)));
        try testing.expect(c.inv_initial_orientation.eql(Quat.identity())); // Same axes: identity
    }

    // Auto detect: the anchor is the center of mass of the body that can move when the other one is static, the
    // weighted center of mass otherwise (towards the lighter body); both points are the anchor in world space
    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        var settings = FixedConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.auto_detect_point = true;
        settings.point1 = RVec3.init(100, 100, 100); // Ignored
        var c = FixedConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        const anchor1 = bodies[0].getCenterOfMassTransform().mulRVec3(RVec3.fromVec3(c.local_space_position1));
        const anchor2 = bodies[1].getCenterOfMassTransform().mulRVec3(RVec3.fromVec3(c.local_space_position2));
        try testing.expect(anchor1.sub(anchor2).toVec3().length() < 1.0e-5);
        const p1 = bodies[0].getCenterOfMassPosition();
        const p2 = bodies[1].getCenterOfMassPosition();
        if (pair[0] == .static) {
            try testing.expect(anchor1.sub(p2).toVec3().length() < 1.0e-5);
        } else if (pair[1] == .static) {
            try testing.expect(anchor1.sub(p1).toVec3().length() < 1.0e-5);
        } else {
            // On the line between the centers of mass
            const t = anchor1.sub(p1).toVec3().length() / p2.sub(p1).toVec3().length();
            try testing.expect(t > 0.0 and t < 1.0);
        }
    }
}

test "FixedConstraint: body 1 fixed to the world" {
    // Body 1 is Body::sFixedToWorld: the constraint attaches body 2 to the world (the solver never writes body 1)
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .linear_velocity = Vec3.init(1, 0, 0) });
    defer context.destroyBodies(&.{body});
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.auto_detect_point = true;
    var c = FixedConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position2.isNearZero(.{})); // The anchor is the center of mass of body 2
    try testing.expect(c.local_space_position1.isClose(body.getCenterOfMassPosition().toVec3(), .{}));
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    try testing.expect(body.getLinearVelocity().isNearZero(.{ .max_dist_sq = 1.0e-10 }));
}

test "FixedConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try FixedConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
