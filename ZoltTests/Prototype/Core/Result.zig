//! Port of: Jolt/Core/Result.h
//! Status: complete
//!
//! `Result<Type>` holds nothing, a value or an error string. Zolt keeps it as a value type (instead of an error union)
//! where Jolt caches and copies results: `ShapeSettings.cached_result`, `ShapeSettings.createShape`,
//! `Shape.restoreFromBinaryState`, `Shape.scaleShape` (ShapeResult) and their relatives (PhysicsMaterialResult,
//! GroupFilterResult, ConstraintSettingsResult, ...). The C++ code passes results along by value
//! (`outResult = child_result`) and tests compare `GetError()` with Jolt's texts.
//!
//! - The error text lives in an inline buffer (`ErrorString`, at most 127 bytes, longer texts are truncated), so
//!   copying, caching and returning a result never allocates. Jolt's longest message in Physics/Collision is 89 bytes.
//! - Zig has no copy constructor: `clone()` is the copy constructor (adds a reference when `Type` is a `Ref`),
//!   `deinit()` the destructor, `assign(&other)` the copy assignment. A plain `=` moves.
//! - `set(value)` takes ownership of `value` (`Set(Type &&)`); use `set(value.clone())` for `Set(const Type &)`.
//! - Allocation failure is not a Result error: functions that allocate return `Allocator.Error!Result(T)`.

const std = @import("std");

/// Inline, fixed capacity error text (no allocation, copyable with `=`)
pub const ErrorString = struct {
    pub const capacity = 127;

    buffer: [capacity]u8 = undefined,
    len: u8 = 0,

    pub fn init(message: []const u8) ErrorString {
        var s: ErrorString = .{};
        const n = @min(message.len, capacity);
        @memcpy(s.buffer[0..n], message[0..n]);
        s.len = @intCast(n);
        return s;
    }

    /// StringFormat(...) (a Zig format string), truncated when the text does not fit
    pub fn initFormat(comptime fmt: []const u8, args: anytype) ErrorString {
        var s: ErrorString = .{};
        var writer: std.Io.Writer = .fixed(&s.buffer);
        writer.print(fmt, args) catch {}; // error.WriteFailed: the buffer is full, keep the truncated text
        s.len = @intCast(writer.end);
        return s;
    }

    pub fn slice(self: *const ErrorString) []const u8 {
        return self.buffer[0..self.len];
    }
};

/// Helper class that either contains a valid result or an error
pub fn Result(comptime Type: type) type {
    return struct {
        const Self = @This();

        const State = enum(u8) { invalid, valid, err };

        /// Which of the members is valid
        state: State = .invalid,
        /// The actual result object (only valid when state == .valid)
        result: Type = undefined,
        /// The error description if the result failed (only valid when state == .err)
        error_message: ErrorString = .{},

        /// Default constructor: the result is empty
        pub const empty: Self = .{};

        /// Copy constructor
        pub fn clone(self: *const Self) Self {
            return switch (self.state) {
                .valid => .{ .state = .valid, .result = cloneValue(&self.result) },
                .err => .{ .state = .err, .error_message = self.error_message },
                .invalid => .{},
            };
        }

        /// Destructor
        pub fn deinit(self: *Self) void {
            self.clear();
        }

        /// Copy assignment (operator =)
        pub fn assign(self: *Self, other: *const Self) void {
            if (self == other) return;
            const copy = other.clone();
            self.clear();
            self.* = copy;
        }

        /// Clear result or error
        pub fn clear(self: *Self) void {
            if (self.state == .valid and comptime hasDecl(Type, "deinit"))
                self.result.deinit();
            self.state = .invalid;
        }

        /// Checks if the result is still uninitialized
        pub fn isEmpty(self: *const Self) bool {
            return self.state == .invalid;
        }

        /// Checks if the result is valid
        pub fn isValid(self: *const Self) bool {
            return self.state == .valid;
        }

        /// Get the result value (borrowed, like the C++ `const Type &`; `get().clone()` for an owned copy)
        pub fn get(self: *const Self) *const Type {
            std.debug.assert(self.isValid());
            return &self.result;
        }

        /// The object a `Ref` / `RefConst` result points to (only compiles for reference types)
        pub fn getPtr(self: *const Self) @FieldType(Type, "ptr") {
            std.debug.assert(self.isValid());
            return self.result.ptr;
        }

        /// Set the result value, takes ownership of `value` (Set(Type &&))
        pub fn set(self: *Self, value: Type) void {
            self.clear();
            self.result = value;
            self.state = .valid;
        }

        /// Check if we had an error
        pub fn hasError(self: *const Self) bool {
            return self.state == .err;
        }

        /// Get the error value
        pub fn getError(self: *const Self) []const u8 {
            std.debug.assert(self.hasError());
            return self.error_message.slice();
        }

        /// Set an error value
        pub fn setError(self: *Self, message: []const u8) void {
            self.clear();
            self.error_message = .init(message);
            self.state = .err;
        }

        /// Set an error value from a format string (SetError(StringFormat(...)))
        pub fn setErrorFmt(self: *Self, comptime fmt: []const u8, args: anytype) void {
            self.clear();
            self.error_message = .initFormat(fmt, args);
            self.state = .err;
        }

        fn cloneValue(value: *const Type) Type {
            return if (comptime hasDecl(Type, "clone")) value.clone() else value.*;
        }
    };
}

fn hasDecl(comptime T: type, comptime name: []const u8) bool {
    return switch (@typeInfo(T)) {
        .@"struct", .@"union", .@"enum", .@"opaque" => @hasDecl(T, name),
        else => false,
    };
}

test "Result value semantics" {
    var a: Result(u32) = .empty;
    try std.testing.expect(a.isEmpty());
    a.set(5);
    var b = a.clone();
    try std.testing.expectEqual(@as(u32, 5), b.get().*);
    b.setErrorFmt("Triangle {d} is degenerate!", .{3});
    a.assign(&b);
    try std.testing.expectEqualStrings("Triangle 3 is degenerate!", a.getError());
    var long: Result(u32) = .empty;
    long.setErrorFmt("{s}{s}", .{ "x" ** 100, "y" ** 100 });
    try std.testing.expectEqual(@as(usize, ErrorString.capacity), long.getError().len);
}
