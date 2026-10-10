//! Phase 5 architecture prototype: reduced Jolt/Physics/StateRecorder.h + StateRecorderImpl.h/.cpp
//! (Docs/Zolt/SimulationArchitecture.md D11).
//!
//! `StateRecorder : public StreamIn, public StreamOut` with data (`mIsValidating`, `mIsLastPart`) in the base is a
//! pattern A root: implementations embed `base: StateRecorder` and override the stream functions. The recorder hands
//! out the existing pattern B `StreamIn` / `StreamOut` interfaces (`streamIn()` / `streamOut()`), so every
//! `saveBinaryState(StreamOut)` of Phase 4 accepts it, and `write` / `read` reuse their comptime dispatch: the bytes are
//! Jolt's (Vec3 = 12 bytes, Quat = 16, bool = 1, ...), which lets parity compare recorded states byte for byte.

const std = @import("std");
const zolt = @import("zolt");
const virtual = zolt.virtual;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const BodyID = zolt.BodyID;
const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Body = @import("Body.zig").Body;
const Constraint = @import("Constraint.zig").Constraint;

/// A bit field that determines which aspects of the simulation to save (EStateRecorderState).
/// Jolt's flag enum with operators |, &, ^, ~ becomes a packed struct with the same bits.
pub const StateRecorderState = packed struct(u8) {
    /// Save global physics system state (delta time, gravity, etc.)
    global: bool = false,
    /// Save the state of bodies
    bodies: bool = false,
    /// Save the state of contacts
    contacts: bool = false,
    /// Save the state of constraints
    constraints: bool = false,
    _padding: u4 = 0,

    /// Save nothing
    pub const none: StateRecorderState = .{};
    /// Save all state
    pub const all: StateRecorderState = .{ .global = true, .bodies = true, .contacts = true, .constraints = true };
};

/// User callbacks that allow determining which parts of the simulation should be saved by a StateRecorder.
/// Pattern A with Jolt's default implementations in the base, like ShapeFilter: `StateRecorderFilter{}` saves everything.
pub const StateRecorderFilter = struct {
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

    vtable: *const VTable = virtual.vtablePtr(VTable, StateRecorderFilter),

    /// Constructor for derived classes
    pub fn init(comptime T: type) StateRecorderFilter {
        return .{ .vtable = virtual.vtablePtr(VTable, T) };
    }

    pub fn shouldSaveBody(self: *const StateRecorderFilter, body: *const Body) bool {
        return self.vtable.shouldSaveBody(self, body);
    }
    pub fn shouldSaveConstraint(self: *const StateRecorderFilter, constraint: *const Constraint) bool {
        return self.vtable.shouldSaveConstraint(self, constraint);
    }
    pub fn shouldSaveContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
        return self.vtable.shouldSaveContact(self, body1, body2);
    }
    pub fn shouldRestoreContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
        return self.vtable.shouldRestoreContact(self, body1, body2);
    }

    /// Jolt's default implementations (everything is saved)
    pub const impl = struct {
        pub fn shouldSaveBody(self: *const StateRecorderFilter, body: *const Body) bool {
            _ = self;
            _ = body;
            return true;
        }
        pub fn shouldSaveConstraint(self: *const StateRecorderFilter, constraint: *const Constraint) bool {
            _ = self;
            _ = constraint;
            return true;
        }
        pub fn shouldSaveContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
            _ = self;
            _ = body1;
            _ = body2;
            return true;
        }
        pub fn shouldRestoreContact(self: *const StateRecorderFilter, body1: BodyID, body2: BodyID) bool {
            _ = self;
            _ = body1;
            _ = body2;
            return true;
        }
    };
};

/// Class that records the state of a physics system. Can be used to check if the simulation is deterministic by putting
/// the recorder in validation mode. Can be used to restore the state to an earlier point in time.
pub const StateRecorder = struct {
    /// The pure virtual functions of StreamIn and StreamOut
    pub const VTable = struct {
        readBytes: *const fn (self: *StateRecorder, data: []u8) void,
        isEOF: *const fn (self: *const StateRecorder) bool,
        isFailed: *const fn (self: *const StateRecorder) bool,
        writeBytes: *const fn (self: *StateRecorder, data: []const u8) void,
    };

    vtable: *const VTable,
    is_validating: bool = false,
    is_last_part: bool = true,

    /// Constructor (for the most derived class T)
    pub fn init(comptime T: type) StateRecorder {
        return .{ .vtable = virtual.vtablePtr(VTable, T) };
    }

    /// All functions are pure virtual
    pub const impl = struct {};

    pub fn readBytes(self: *StateRecorder, data: []u8) void {
        self.vtable.readBytes(self, data);
    }
    pub fn isEOF(self: *const StateRecorder) bool {
        return self.vtable.isEOF(self);
    }
    pub fn isFailed(self: *const StateRecorder) bool {
        return self.vtable.isFailed(self);
    }
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

    /// StreamOut::Write
    pub fn write(self: *StateRecorder, value: anytype) void {
        self.streamOut().write(value);
    }

    /// StreamIn::Read (in/out: a validating recorder compares with the current value)
    pub fn read(self: *StateRecorder, value: anytype) void {
        self.streamIn().read(value);
    }

    /// Sets the stream in validation mode
    pub fn setValidating(self: *StateRecorder, validating: bool) void {
        self.is_validating = validating;
    }
    pub fn isValidating(self: *const StateRecorder) bool {
        return self.is_validating;
    }

    /// This allows splitting the state in multiple parts. While restoring, only the last part should have this flag set to true.
    pub fn setIsLastPart(self: *StateRecorder, is_last_part: bool) void {
        self.is_last_part = is_last_part;
    }
    pub fn isLastPart(self: *const StateRecorder) bool {
        return self.is_last_part;
    }
};

/// Implementation of the StateRecorder class that uses a byte array (Jolt: std::stringstream) as underlying store
pub const StateRecorderImpl = struct {
    pub const overrides = .{ .readBytes, .isEOF, .isFailed, .writeBytes };

    base: StateRecorder,
    allocator: Allocator,
    data: std.ArrayList(u8) = .empty,
    read_position: usize = 0,
    failed: bool = false,
    /// Zolt: number of validation mismatches (Jolt traces them and hits JPH_BREAKPOINT; Zolt logs, counts and panics
    /// when asserts are enabled)
    num_mismatches: u32 = 0,

    pub fn init(allocator: Allocator) StateRecorderImpl {
        return .{ .base = .init(StateRecorderImpl), .allocator = allocator };
    }

    pub fn deinit(self: *StateRecorderImpl) void {
        self.data.deinit(self.allocator);
    }

    pub fn recorder(self: *StateRecorderImpl) *StateRecorder {
        return &self.base;
    }

    /// Rewind the stream for reading
    pub fn rewind(self: *StateRecorderImpl) void {
        self.read_position = 0;
    }

    /// Clear the stream for reuse
    pub fn clear(self: *StateRecorderImpl) void {
        self.data.clearRetainingCapacity();
        self.read_position = 0;
        self.failed = false;
    }

    /// Compare this state with a reference state and ensure they are the same
    pub fn isEqual(self: *const StateRecorderImpl, reference: *const StateRecorderImpl) bool {
        return std.mem.eql(u8, self.data.items, reference.data.items);
    }

    pub fn getData(self: *const StateRecorderImpl) []const u8 {
        return self.data.items;
    }

    pub fn writeBytes(self: *StateRecorderImpl, data: []const u8) void {
        self.data.appendSlice(self.allocator, data) catch {
            self.failed = true; // Like a stringstream that cannot grow: the stream fails, Write has no error channel
        };
    }

    pub fn readBytes(self: *StateRecorderImpl, out: []u8) void {
        if (self.read_position + out.len > self.data.items.len) {
            self.failed = true;
            return;
        }
        const data = self.data.items[self.read_position..][0..out.len];
        self.read_position += out.len;

        if (self.base.isValidating()) {
            // Compare with the current value
            if (!std.mem.eql(u8, data, out)) {
                // Mismatch, print error (Jolt: Trace per differing byte + JPH_BREAKPOINT)
                self.num_mismatches += 1;
                std.log.scoped(.zolt).err("Mismatch reading {d} bytes", .{out.len});
                for (out, data, 0..) |b1, b2, i| {
                    if (b1 != b2)
                        std.log.scoped(.zolt).err("Offset {d}: {X:0>2} -> {X:0>2}", .{ i, b1, b2 });
                }
                if (Core.enable_asserts) @panic("StateRecorderImpl: validation mismatch");
            }
        }
        @memcpy(out, data);
    }

    pub fn isEOF(self: *const StateRecorderImpl) bool {
        return self.read_position >= self.data.items.len;
    }

    pub fn isFailed(self: *const StateRecorderImpl) bool {
        return self.failed;
    }
};
