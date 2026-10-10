//! Port of: Jolt/Physics/StateRecorder.h
//! Status: complete
//!
//! SimulationArchitecture D11:
//! - `StateRecorder : public StreamIn, public StreamOut` with data in the base (`mIsValidating`, `mIsLastPart`) is a
//!   pattern A root: `{ vtable{ readBytes, isEOF, isFailed, writeBytes }, is_validating, is_last_part }`. All four
//!   functions are pure virtual (`impl` is empty); implementations embed `base: StateRecorder = .init(@This())` and
//!   list them in `overrides` (see StateRecorderImpl).
//! - `streamIn()` / `streamOut()` hand out the Phase 2 pattern B streams (the C++ base classes), so `write` / `read`
//!   produce Jolt's bytes (Vec3 = 12 bytes, Quat = 16, bool = 1, ...) and every `saveBinaryState(StreamOut)` accepts a
//!   recorder.
//! - `EStateRecorderState` is a `packed struct(u8)` with Jolt's bits; `none` and `all` are decls, the operators are
//!   `bitOr`, `bitAnd`, `bitXor`, `bitNot`.
//! - `StateRecorderFilter` is pattern A with Jolt's accept-all defaults: `StateRecorderFilter{}` saves everything.
//! - The copy constructor `StateRecorder(const StateRecorder &)` only copies `mIsValidating`: it is
//!   `StateRecorder.initCopy(T, &other)`, used by `StateRecorderImpl.move` (Jolt's move constructor).

const std = @import("std");
const virtual = @import("../Core/Virtual.zig");
const StreamIn = @import("../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;
const BodyID = @import("Body/BodyID.zig").BodyID;
const Body = @import("Body/Body.zig").Body;
const Constraint = @import("Constraints/Constraint.zig").Constraint;

/// A bit field that determines which aspects of the simulation to save (EStateRecorderState)
pub const StateRecorderState = packed struct(u8) {
    /// Save global physics system state (delta time, gravity, etc.)
    global: bool = false,
    /// Save the state of bodies
    bodies: bool = false,
    /// Save the state of contacts
    contacts: bool = false,
    /// Save the state of constraints
    constraints: bool = false,
    /// The unused bits of Jolt's uint8 (only `bitNot` sets them)
    _padding: u4 = 0,

    /// Save nothing
    pub const none: StateRecorderState = .{};
    /// Save all state
    pub const all: StateRecorderState = .{ .global = true, .bodies = true, .contacts = true, .constraints = true };

    comptime {
        std.debug.assert(@as(u8, @bitCast(none)) == 0);
        std.debug.assert(@as(u8, @bitCast(StateRecorderState{ .global = true })) == 1);
        std.debug.assert(@as(u8, @bitCast(StateRecorderState{ .bodies = true })) == 2);
        std.debug.assert(@as(u8, @bitCast(StateRecorderState{ .contacts = true })) == 4);
        std.debug.assert(@as(u8, @bitCast(StateRecorderState{ .constraints = true })) == 8);
        std.debug.assert(@as(u8, @bitCast(all)) == 15);
    }

    /// The value as Jolt's uint8
    pub fn toInt(self: StateRecorderState) u8 {
        return @bitCast(self);
    }

    /// `EStateRecorderState(inValue)`
    pub fn fromInt(value: u8) StateRecorderState {
        return @bitCast(value);
    }

    /// Bitwise OR operator for EStateRecorderState
    pub fn bitOr(lhs: StateRecorderState, rhs: StateRecorderState) StateRecorderState {
        return fromInt(lhs.toInt() | rhs.toInt());
    }

    /// Bitwise AND operator for EStateRecorderState
    pub fn bitAnd(lhs: StateRecorderState, rhs: StateRecorderState) StateRecorderState {
        return fromInt(lhs.toInt() & rhs.toInt());
    }

    /// Bitwise XOR operator for EStateRecorderState
    pub fn bitXor(lhs: StateRecorderState, rhs: StateRecorderState) StateRecorderState {
        return fromInt(lhs.toInt() ^ rhs.toInt());
    }

    /// Bitwise NOT operator for EStateRecorderState
    pub fn bitNot(state: StateRecorderState) StateRecorderState {
        return fromInt(~state.toInt());
    }

    /// `==` on the enum (compares the whole byte)
    pub fn eql(lhs: StateRecorderState, rhs: StateRecorderState) bool {
        return lhs.toInt() == rhs.toInt();
    }
};

/// User callbacks that allow determining which parts of the simulation should be saved by a StateRecorder
pub const StateRecorderFilter = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// If the state of a specific body should be saved
        shouldSaveBody: *const fn (self: *const StateRecorderFilter, body: *const Body) bool,

        /// If the state of a specific constraint should be saved
        shouldSaveConstraint: *const fn (self: *const StateRecorderFilter, constraint: *const Constraint) bool,

        /// If the state of a specific contact should be saved
        shouldSaveContact: *const fn (self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool,

        /// If the state of a specific contact should be restored
        shouldRestoreContact: *const fn (self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool,
    };

    vtable: *const VTable = vtableFor(StateRecorderFilter),

    /// Constructor, called by derived filters with their most derived type
    pub fn init(comptime T: type) StateRecorderFilter {
        return .{ .vtable = vtableFor(T) };
    }

    /// The vtable of filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    ///@name Functions called during SaveState

    /// If the state of a specific body should be saved
    pub fn shouldSaveBody(self: *const StateRecorderFilter, body: *const Body) bool {
        return self.vtable.shouldSaveBody(self, body);
    }

    /// If the state of a specific constraint should be saved
    pub fn shouldSaveConstraint(self: *const StateRecorderFilter, constraint: *const Constraint) bool {
        return self.vtable.shouldSaveConstraint(self, constraint);
    }

    /// If the state of a specific contact should be saved
    pub fn shouldSaveContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
        return self.vtable.shouldSaveContact(self, body1, body2);
    }

    ///@name Functions called during RestoreState

    /// If the state of a specific contact should be restored
    pub fn shouldRestoreContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
        return self.vtable.shouldRestoreContact(self, body1, body2);
    }

    /// Jolt's default implementations: everything is saved and restored
    pub const impl = struct {
        pub fn shouldSaveBody(self: *const StateRecorderFilter, body: *const Body) bool {
            _ = .{ self, body };
            return true;
        }

        pub fn shouldSaveConstraint(self: *const StateRecorderFilter, constraint: *const Constraint) bool {
            _ = .{ self, constraint };
            return true;
        }

        pub fn shouldSaveContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
            _ = .{ self, body1, body2 };
            return true;
        }

        pub fn shouldRestoreContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
            _ = .{ self, body1, body2 };
            return true;
        }
    };
};

/// Class that records the state of a physics system. Can be used to check if the simulation is deterministic by putting the recorder in validation mode.
/// Can be used to restore the state to an earlier point in time. Note that only the state that is modified by the simulation is saved, configuration settings
/// like body friction or restitution, motion quality etc. are not saved and need to be saved by the user if desired.
pub const StateRecorder = struct {
    /// The pure virtual functions of the base classes StreamIn and StreamOut
    pub const VTable = struct {
        /// Read a string of bytes from the binary stream (StreamIn::ReadBytes)
        readBytes: *const fn (self: *StateRecorder, data: []u8) void,

        /// Returns true when an attempt has been made to read past the end of the file (StreamIn::IsEOF)
        isEOF: *const fn (self: *const StateRecorder) bool,

        /// Returns true if there was an IO failure (StreamIn::IsFailed / StreamOut::IsFailed)
        isFailed: *const fn (self: *const StateRecorder) bool,

        /// Write a string of bytes to the binary stream (StreamOut::WriteBytes)
        writeBytes: *const fn (self: *StateRecorder, data: []const u8) void,
    };

    vtable: *const VTable,
    is_validating: bool = false,
    is_last_part: bool = true,

    /// Constructor, called by implementations with their most derived type
    pub fn init(comptime T: type) StateRecorder {
        return .{ .vtable = vtableFor(T) };
    }

    /// Copy constructor StateRecorder(const StateRecorder &inRHS): only copies the validation flag
    pub fn initCopy(comptime T: type, rhs: *const StateRecorder) StateRecorder {
        return .{ .vtable = vtableFor(T), .is_validating = rhs.is_validating };
    }

    /// The vtable of implementation class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// All virtual functions are pure virtual
    pub const impl = struct {};

    /// Read a string of bytes from the binary stream
    pub fn readBytes(self: *StateRecorder, data: []u8) void {
        self.vtable.readBytes(self, data);
    }

    /// Returns true when an attempt has been made to read past the end of the file
    pub fn isEOF(self: *const StateRecorder) bool {
        return self.vtable.isEOF(self);
    }

    /// Returns true if there was an IO failure
    pub fn isFailed(self: *const StateRecorder) bool {
        return self.vtable.isFailed(self);
    }

    /// Write a string of bytes to the binary stream
    pub fn writeBytes(self: *StateRecorder, data: []const u8) void {
        self.vtable.writeBytes(self, data);
    }

    /// The StreamIn base class
    pub fn streamIn(self: *StateRecorder) StreamIn {
        return StreamIn.init(self);
    }

    /// The StreamOut base class
    pub fn streamOut(self: *StateRecorder) StreamOut {
        return StreamOut.init(self);
    }

    /// StreamOut::Write: write a primitive (or Vec3, DVec3, DMat44) to the binary stream
    pub fn write(self: *StateRecorder, value: anytype) void {
        self.streamOut().write(value);
    }

    /// StreamIn::Read: `value` points to the destination, which is also the current value that a validating recorder
    /// compares with
    pub fn read(self: *StateRecorder, value: anytype) void {
        self.streamIn().read(value);
    }

    /// Sets the stream in validation mode. In this case the physics system ensures that before it calls ReadBytes that it will
    /// ensure that those bytes contain the current state. This makes it possible to step and save the state, restore to the previous
    /// step and step again and when the recorded state is not the same it can restore the expected state and any byte that changes
    /// due to a ReadBytes function can be caught to find out which part of the simulation is not deterministic.
    /// Note that validation only works when saving the full state of the simulation (EStateRecorderState::All, StateRecorderFilter == nullptr).
    pub fn setValidating(self: *StateRecorder, validating: bool) void {
        self.is_validating = validating;
    }

    pub fn isValidating(self: *const StateRecorder) bool {
        return self.is_validating;
    }

    /// This allows splitting the state in multiple parts. While restoring, only the last part should have this flag set to true.
    /// Note that you should ensure that the different parts contain information for disjoint sets of bodies, constraints and contacts.
    /// E.g. if you restore the same contact twice, you get undefined behavior. In order to create disjoint sets you can use the StateRecorderFilter.
    /// Note that validation is not compatible with restoring a simulation state in multiple parts.
    pub fn setIsLastPart(self: *StateRecorder, is_last_part: bool) void {
        self.is_last_part = is_last_part;
    }

    pub fn isLastPart(self: *const StateRecorder) bool {
        return self.is_last_part;
    }
};

test "StateRecorderState bits and operators" {
    const s = StateRecorderState{ .bodies = true };
    try std.testing.expect(s.bitOr(.{ .contacts = true }).eql(StateRecorderState.fromInt(6)));
    try std.testing.expect(StateRecorderState.all.bitAnd(s).eql(s));
    try std.testing.expect(StateRecorderState.all.bitXor(s).eql(.{ .global = true, .contacts = true, .constraints = true }));
    try std.testing.expectEqual(@as(u8, 0xfd), s.bitNot().toInt());
    try std.testing.expect(StateRecorderState.all.bitAnd(.{ .constraints = true }).constraints);
}

test "StateRecorderFilter defaults and a derived filter" {
    const body: Body = .{ .id = .init(3) };
    const default_filter: StateRecorderFilter = .{};
    try std.testing.expect(default_filter.shouldSaveBody(&body));
    try std.testing.expect(default_filter.shouldSaveConstraint(@ptrCast(&body)));
    try std.testing.expect(default_filter.shouldSaveContact(.init(1), .init(2)));
    try std.testing.expect(default_filter.shouldRestoreContact(.init(1), .init(2)));

    const OnlyBody1 = struct {
        pub const overrides = .{ .shouldSaveBody, .shouldSaveContact };
        base: StateRecorderFilter = .init(@This()),
        pub fn shouldSaveBody(self: *const @This(), b: *const Body) bool {
            _ = self;
            return b.getID().eql(.init(1));
        }
        pub fn shouldSaveContact(self: *const @This(), body1: BodyID, body2: BodyID) bool {
            _ = self;
            return body1.eql(.init(1)) or body2.eql(.init(1));
        }
    };
    const filter: OnlyBody1 = .{};
    try std.testing.expect(!filter.base.shouldSaveBody(&body));
    try std.testing.expect(filter.base.shouldSaveContact(.init(2), .init(1)));
    try std.testing.expect(!filter.base.shouldSaveContact(.init(2), .init(3)));
    try std.testing.expect(filter.base.shouldRestoreContact(.init(2), .init(3))); // Default
}
