//! Port of: Jolt/Physics/StateRecorderImpl.h, Jolt/Physics/StateRecorderImpl.cpp
//! Status: complete
//!
//! Jolt stores the state in a `std::stringstream`. Zolt stores the bytes in an `ArrayList(u8)` with a read position
//! and models the stream state that Jolt's functions observe (SimulationArchitecture D11):
//! - `eof_bit` / `fail_bit` / `bad_bit` are the `std::ios_base` flags. A read past the end copies the bytes that are
//!   left and sets eof and fail (`IsEOF` is `eof()`: true only after a read past the end, not when the read position
//!   is at the end). While the stream is not good, reads read nothing and set fail, writes are ignored, `Rewind`
//!   clears eof but does not move the read position when fail is set. `Clear` resets everything.
//! - A write that cannot grow the buffer (out of memory) sets bad, like `sputn` writing fewer bytes: `Write` has no
//!   error channel, the caller checks `IsFailed`.
//! - Validation (`IsValidating`): a mismatch traces every differing byte (std.log info, Jolt's `Trace`), is counted in
//!   `num_mismatches` (a Zolt addition) and panics when asserts are on (Jolt: `JPH_BREAKPOINT`). On a read past the
//!   end Jolt compares against uninitialized stack memory; Zolt only compares and copies the bytes that were read.
//! - `IsEqual` and `GetDataSize` keep their stream side effects / semantics (`IsEqual` moves both read positions,
//!   `tellg` / `tellp` return -1 on a failed stream).
//! - Jolt's move constructor is `move()`; `GetData()` returns the bytes without copying them.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const StateRecorder = @import("StateRecorder.zig").StateRecorder;

const log = std.log.scoped(.zolt);

/// Implementation of the StateRecorder class that uses a stringstream as underlying store and that implements checking if the state doesn't change upon reading
pub const StateRecorderImpl = struct {
    pub const overrides = .{ .readBytes, .isEOF, .isFailed, .writeBytes };

    base: StateRecorder,
    allocator: Allocator,
    /// The contents of the stringstream (str())
    data: std.ArrayList(u8) = .empty,
    /// The get position of the stringstream
    read_position: usize = 0,
    /// std::ios_base::eofbit
    eof_bit: bool = false,
    /// std::ios_base::failbit
    fail_bit: bool = false,
    /// std::ios_base::badbit (a write that could not store all bytes)
    bad_bit: bool = false,
    /// Zolt addition: number of ReadBytes calls that found a mismatch while validating
    num_mismatches: u32 = 0,

    /// Constructor
    pub fn init(allocator: Allocator) StateRecorderImpl {
        return .{ .base = .init(StateRecorderImpl), .allocator = allocator };
    }

    /// Move constructor StateRecorderImpl(StateRecorderImpl &&): the base copy constructor only copies the validation
    /// flag, the stream (bytes, positions, flags) moves. `self` is left empty.
    pub fn move(self: *StateRecorderImpl) StateRecorderImpl {
        var result: StateRecorderImpl = self.*;
        result.base = .initCopy(StateRecorderImpl, &self.base);
        self.data = .empty;
        self.read_position = 0;
        self.eof_bit = false;
        self.fail_bit = false;
        self.bad_bit = false;
        return result;
    }

    /// Destructor
    pub fn deinit(self: *StateRecorderImpl) void {
        self.data.deinit(self.allocator);
    }

    /// The StateRecorder base class (pass this where Jolt takes a `StateRecorder &`)
    pub fn recorder(self: *StateRecorderImpl) *StateRecorder {
        return &self.base;
    }

    /// std::basic_ios::good()
    fn good(self: *const StateRecorderImpl) bool {
        return !self.eof_bit and !self.fail_bit and !self.bad_bit;
    }

    /// Write a string of bytes to the binary stream
    pub fn writeBytes(self: *StateRecorderImpl, data: []const u8) void {
        // std::ostream::write: the sentry fails (nothing written) when the stream is not good
        if (!self.good())
            return;
        self.data.appendSlice(self.allocator, data) catch {
            self.bad_bit = true; // sputn could not write all bytes
        };
    }

    /// Rewind the stream for reading
    pub fn rewind(self: *StateRecorderImpl) void {
        self.seekg(0);
    }

    /// Clear the stream for reuse
    pub fn clear(self: *StateRecorderImpl) void {
        // mStream.clear(): reset the flags
        self.eof_bit = false;
        self.fail_bit = false;
        self.bad_bit = false;

        // mStream.str({}): empty the buffer
        self.data.clearRetainingCapacity();
        self.read_position = 0;
    }

    /// Read a string of bytes from the binary stream
    pub fn readBytes(self: *StateRecorderImpl, out_data: []u8) void {
        // std::istream::read: the sentry fails (sets failbit, reads nothing) when the stream is not good
        if (!self.good()) {
            self.fail_bit = true;
            return;
        }
        const num_available = @min(out_data.len, self.data.items.len - self.read_position);
        const data = self.data.items[self.read_position..][0..num_available];
        self.read_position += num_available;
        if (num_available != out_data.len) {
            self.eof_bit = true;
            self.fail_bit = true;
        }

        if (self.base.isValidating()) {
            // Read data in temporary buffer to compare with current value (Zolt: the bytes are compared in place)
            if (!std.mem.eql(u8, data, out_data[0..num_available])) {
                // Mismatch, print error
                self.num_mismatches += 1;
                log.info("Mismatch reading {d} bytes", .{out_data.len});
                for (out_data[0..num_available], data, 0..) |b1, b2, i| {
                    if (b1 != b2)
                        log.info("Offset {d}: {X:0>2} -> {X:0>2}", .{ i, b1, b2 });
                }
                if (Core.enable_asserts) @panic("StateRecorderImpl: mismatch while validating"); // JPH_BREAKPOINT
            }

            // Copy the temporary data to the final destination
            @memcpy(out_data[0..num_available], data);
            return;
        }

        @memcpy(out_data[0..num_available], data);
    }

    // See StreamIn
    pub fn isEOF(self: *const StateRecorderImpl) bool {
        return self.eof_bit;
    }

    // See StreamIn / StreamOut
    pub fn isFailed(self: *const StateRecorderImpl) bool {
        return self.fail_bit or self.bad_bit;
    }

    /// Compare this state with a reference state and ensure they are the same
    pub fn isEqual(self: *StateRecorderImpl, reference: *StateRecorderImpl) bool {
        // Get length of new state
        self.seekg(self.data.items.len);
        const this_len = self.tellg();
        self.seekg(0);

        // Get length of old state
        reference.seekg(reference.data.items.len);
        const reference_len = reference.tellg();
        reference.seekg(0);

        // Compare size
        var fail = reference_len != this_len;
        if (fail) {
            log.info("Failed to properly recover state, different stream length!", .{});
            return false;
        }

        // Compare byte by byte
        var i: i64 = 0;
        while (!fail and i < this_len) : (i += 1) {
            fail = reference.get() != self.get();
            if (fail) {
                log.info("Failed to properly recover state, different at offset {d}!", .{i});
                return false;
            }
        }

        return true;
    }

    /// Convert the binary data to a string (Jolt returns a copy, Zolt the bytes in the recorder)
    pub fn getData(self: *const StateRecorderImpl) []const u8 {
        return self.data.items;
    }

    /// Get size of the binary data in bytes (`size_t(mStream.tellp())`: SIZE_MAX when the stream failed)
    pub fn getDataSize(self: *const StateRecorderImpl) usize {
        if (self.isFailed())
            return std.math.maxInt(usize);
        return self.data.items.len;
    }

    /// std::istream::seekg to an absolute position: clears eofbit, fails (sets failbit) when the stream is not good
    fn seekg(self: *StateRecorderImpl, position: usize) void {
        self.eof_bit = false;
        if (!self.good()) {
            self.fail_bit = true;
            return;
        }
        self.read_position = position;
    }

    /// std::istream::tellg: -1 when the stream failed
    fn tellg(self: *const StateRecorderImpl) i64 {
        if (self.isFailed())
            return -1;
        return @intCast(self.read_position);
    }

    /// std::istream::get: the next byte or -1 (EOF)
    fn get(self: *StateRecorderImpl) i32 {
        if (!self.good()) {
            self.fail_bit = true;
            return -1;
        }
        if (self.read_position >= self.data.items.len) {
            self.eof_bit = true;
            self.fail_bit = true;
            return -1;
        }
        const b = self.data.items[self.read_position];
        self.read_position += 1;
        return b;
    }
};

test "StateRecorderImpl write, rewind, read, eof and failure like a stringstream" {
    const expect = std.testing.expect;
    var r = StateRecorderImpl.init(std.testing.allocator);
    defer r.deinit();
    const rec = r.recorder();

    rec.write(@as(u32, 0x12345678));
    rec.write(true);
    try std.testing.expectEqual(@as(usize, 5), r.getDataSize());
    try std.testing.expectEqualSlices(u8, &.{ 0x78, 0x56, 0x34, 0x12, 1 }, r.getData());

    var v: u32 = 0;
    var b = false;
    rec.read(&v);
    rec.read(&b);
    try expect(v == 0x12345678 and b);
    try expect(!rec.isEOF()); // At the end, but no attempt to read past it yet
    try expect(!rec.isFailed());

    // Reading past the end: eof and fail, nothing changes
    var past: u16 = 7;
    rec.read(&past);
    try expect(rec.isEOF() and rec.isFailed());
    try std.testing.expectEqual(@as(u16, 7), past);

    // A failed stream ignores writes and rewinds
    rec.write(@as(u8, 1));
    try std.testing.expectEqual(@as(usize, 5), r.getData().len);
    try std.testing.expectEqual(std.math.maxInt(usize), r.getDataSize());
    r.rewind();
    try expect(!rec.isEOF() and rec.isFailed());

    // Clear resets everything
    r.clear();
    try expect(!rec.isEOF() and !rec.isFailed());
    try std.testing.expectEqual(@as(usize, 0), r.getDataSize());

    // A partial read copies the remaining bytes
    rec.write(@as(u16, 0xbeef));
    r.rewind();
    var partial: u32 = 0;
    rec.read(&partial);
    try std.testing.expectEqual(@as(u32, 0xbeef), partial);
    try expect(rec.isEOF() and rec.isFailed());
}

test "StateRecorderImpl validation, isEqual and move" {
    const expect = std.testing.expect;
    const allocator = std.testing.allocator;
    var a = StateRecorderImpl.init(allocator);
    defer a.deinit();
    var b = StateRecorderImpl.init(allocator);
    defer b.deinit();

    a.recorder().write(@as(f32, 1.5));
    a.recorder().write(@as(u8, 3));
    b.recorder().write(@as(f32, 1.5));
    b.recorder().write(@as(u8, 3));
    try expect(a.isEqual(&b));
    try expect(!a.recorder().isEOF());

    // Validating read of the same value: no mismatch
    a.rewind();
    a.recorder().setValidating(true);
    var f: f32 = 1.5;
    a.recorder().read(&f);
    try std.testing.expectEqual(@as(u32, 0), a.num_mismatches);
    if (!Core.enable_asserts) {
        // A mismatch is counted and the recorded value is restored (with asserts it panics like JPH_BREAKPOINT)
        var x: u8 = 4;
        a.recorder().read(&x);
        try std.testing.expectEqual(@as(u8, 3), x);
        try std.testing.expectEqual(@as(u32, 1), a.num_mismatches);
    }

    // Different contents or lengths
    b.recorder().write(@as(u8, 0));
    try expect(!a.isEqual(&b));
    a.clear();
    b.clear();
    a.recorder().write(@as(u8, 1));
    b.recorder().write(@as(u8, 2));
    try expect(!a.isEqual(&b));

    // The move constructor keeps the bytes and the validation flag, but not the last part flag
    a.recorder().setIsLastPart(false);
    var moved = a.move();
    defer moved.deinit();
    try expect(moved.recorder().isValidating());
    try expect(moved.recorder().isLastPart());
    try std.testing.expectEqualSlices(u8, &.{1}, moved.getData());
    try std.testing.expectEqual(@as(usize, 0), a.getData().len);
}

test "StateRecorderImpl out of memory while writing fails the stream" {
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 0 });
    var r = StateRecorderImpl.init(failing.allocator());
    defer r.deinit();
    r.recorder().write(@as(u64, 1));
    try std.testing.expect(r.recorder().isFailed());
    try std.testing.expect(!r.recorder().isEOF());
    r.clear();
    try std.testing.expect(!r.recorder().isFailed());
}
