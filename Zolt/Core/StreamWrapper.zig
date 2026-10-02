//! Port of: Jolt/Core/StreamWrapper.h
//! Status: complete
//!
//! The std::ostream / std::istream adapters become adapters over `*std.Io.Writer` / `*std.Io.Reader`. The
//! error state follows the iostream rules: an I/O error sets `failed`, reading past the end sets `eof` and
//! `failed`, and once a stream has failed nothing more is written or read. Writers are buffered: flush the
//! `std.Io.Writer` when done.

const std = @import("std");
const StreamIn = @import("StreamIn.zig").StreamIn;
const StreamOut = @import("StreamOut.zig").StreamOut;

/// Wrapper around std::ostream
pub const StreamOutWrapper = struct {
    wrapped: *std.Io.Writer,

    /// The failbit / badbit of the C++ stream
    failed: bool = false,

    /// Constructor
    pub fn init(wrapped: *std.Io.Writer) StreamOutWrapper {
        return .{ .wrapped = wrapped };
    }

    /// The StreamOut interface of this wrapper
    pub fn streamOut(self: *StreamOutWrapper) StreamOut {
        return .init(self);
    }

    /// Write a string of bytes to the binary stream
    pub fn writeBytes(self: *StreamOutWrapper, data: []const u8) void {
        // Like std::ostream::write: nothing is written once the stream has failed
        if (self.failed)
            return;
        self.wrapped.writeAll(data) catch {
            self.failed = true;
        };
    }

    /// Returns true if there was an IO failure
    pub fn isFailed(self: *const StreamOutWrapper) bool {
        return self.failed;
    }
};

/// Wrapper around std::istream
pub const StreamInWrapper = struct {
    wrapped: *std.Io.Reader,

    /// The eofbit of the C++ stream
    eof: bool = false,

    /// The failbit / badbit of the C++ stream
    failed: bool = false,

    /// Constructor
    pub fn init(wrapped: *std.Io.Reader) StreamInWrapper {
        return .{ .wrapped = wrapped };
    }

    /// The StreamIn interface of this wrapper
    pub fn streamIn(self: *StreamInWrapper) StreamIn {
        return .init(self);
    }

    /// Write a string of bytes to the binary stream
    pub fn readBytes(self: *StreamInWrapper, data: []u8) void {
        // Like std::istream::read: nothing is read once the stream is not good anymore
        if (self.eof or self.failed) {
            self.failed = true;
            return;
        }

        // Read as much as is available, a short read sets eof and fail
        const num_read = self.wrapped.readSliceShort(data) catch {
            self.failed = true;
            return;
        };
        if (num_read < data.len) {
            self.eof = true;
            self.failed = true;
        }
    }

    /// Returns true when an attempt has been made to read past the end of the file
    pub fn isEOF(self: *const StreamInWrapper) bool {
        return self.eof;
    }

    /// Returns true if there was an IO failure
    pub fn isFailed(self: *const StreamInWrapper) bool {
        return self.failed;
    }
};

test "StreamOutWrapper / StreamInWrapper round trip" {
    const allocator = std.testing.allocator;
    const Vec3 = @import("../Math/Vec3.zig").Vec3;

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamOutWrapper = .init(&out.writer);
    const stream_out = out_wrapper.streamOut();
    stream_out.write(@as(u32, 42));
    stream_out.write(Vec3.init(1, 2, 3));
    stream_out.writeString("hello");
    stream_out.writeArray(f32, &.{ 1.5, 2.5 });
    try std.testing.expect(!stream_out.isFailed());
    try std.testing.expectEqual(@as(usize, 4 + 12 + 4 + 5 + 4 + 8), out.written().len);

    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamInWrapper = .init(&reader);
    const stream_in = in_wrapper.streamIn();
    var u: u32 = 0;
    var v = Vec3.zero();
    var s: []u8 = &.{};
    defer allocator.free(s);
    var a: std.ArrayList(f32) = .empty;
    defer a.deinit(allocator);
    stream_in.read(&u);
    stream_in.read(&v);
    try stream_in.readString(allocator, &s);
    try stream_in.readArray(f32, allocator, &a);
    try std.testing.expect(!stream_in.isEOF() and !stream_in.isFailed());
    try std.testing.expectEqual(@as(u32, 42), u);
    try std.testing.expect(v.eql(Vec3.init(1, 2, 3)));
    try std.testing.expectEqualStrings("hello", s);
    try std.testing.expectEqualSlices(f32, &.{ 1.5, 2.5 }, a.items);

    // Reading past the end: the available bytes are read, then eof and fail are set and nothing more is read
    const bytes = [_]u8{ 1, 2, 3, 4, 5, 6 };
    reader = .fixed(&bytes);
    in_wrapper = .init(&reader);
    var x: u32 = 0;
    stream_in.read(&x);
    try std.testing.expect(!stream_in.isEOF());
    x = 0;
    stream_in.read(&x);
    try std.testing.expect(stream_in.isEOF() and stream_in.isFailed());
    try std.testing.expectEqual(@as(u32, 0x0605), x);
    x = 0;
    stream_in.read(&x);
    try std.testing.expectEqual(@as(u32, 0), x);

    // A failing writer sets the fail state, after which nothing is written
    var buffer: [6]u8 = undefined;
    var fixed: std.Io.Writer = .fixed(&buffer);
    out_wrapper = .init(&fixed);
    stream_out.write(@as(u32, 0x04030201));
    try std.testing.expect(!stream_out.isFailed());
    stream_out.write(@as(u32, 0x08070605));
    try std.testing.expect(stream_out.isFailed());
    stream_out.writeArray(u8, &.{ 9, 10 }); // Only writes the length, which is not written because the stream failed
    try std.testing.expectEqualSlices(u8, &.{ 1, 2, 3, 4, 5, 6 }, fixed.buffered());
}
