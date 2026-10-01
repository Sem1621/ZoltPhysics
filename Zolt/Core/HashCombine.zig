//! Port of: Jolt/Core/HashCombine.h
//! Status: complete
//!
//! `Hash<T>` (a C++ functor specialized per type) becomes the comptime-dispatched `hash(value)`.
//! A user type opts in by declaring `pub fn getHash(self: T) u64` (Jolt: `GetHash()`).
//! JPH_MAKE_HASHABLE / JPH_MAKE_HASH_STRUCT are replaced by a `getHash` method that calls `hashCombineArgs`.

const std = @import("std");

/// Seed for FNV-1a
pub const fnv1a_seed: u64 = 0xcbf29ce484222325;

/// Implements the FNV-1a hash algorithm
/// See: https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
pub fn hashBytesSeeded(data: []const u8, seed: u64) u64 {
    var h = seed;
    for (data) |byte| {
        h ^= @as(u64, byte);
        h *%= 0x100000001b3;
    }
    return h;
}

/// FNV-1a hash of `data` with the default seed (HashBytes)
pub fn hashBytes(data: []const u8) u64 {
    return hashBytesSeeded(data, fnv1a_seed);
}

/// Calculate the FNV-1a hash of a string (HashString). Gives the same result as `hashBytes` over the same characters.
pub fn hashString(str: []const u8) u64 {
    return hashBytes(str);
}

/// A 64 bit hash function by Thomas Wang, Jan 1997
/// See: http://web.archive.org/web/20071223173210/http://www.concentric.net/~Ttwang/tech/inthash.htm
pub fn hash64(value: u64) u64 {
    var h = value;
    h = (~h) +% (h << 21); // h = (h << 21) - h - 1;
    h = h ^ (h >> 24);
    h = (h +% (h << 3)) +% (h << 8); // h * 265
    h = h ^ (h >> 14);
    h = (h +% (h << 2)) +% (h << 4); // h * 21
    h = h ^ (h >> 28);
    h = h +% (h << 31);
    return h;
}

/// Hash a value (Jolt: `Hash<T>{}(value)`).
///
/// - floats: -0 is hashed as +0, then FNV-1a over the bytes
/// - integers, bools, enums and pointers: FNV-1a over the bytes (JPH_DEFINE_TRIVIAL_HASH / Hash<T *>)
/// - `[]const u8` / string literals: FNV-1a over the characters (Hash<const char *> / Hash<String>)
/// - anything with a `getHash()` method: that method (Hash<T> primary template)
pub fn hash(value: anytype) u64 {
    const T = @TypeOf(value);
    switch (@typeInfo(T)) {
        .float => {
            const v: T = if (value == 0) 0 else value; // Convert -0.0 to 0.0
            return hashBytes(std.mem.asBytes(&v));
        },
        .comptime_float => @compileError("hash: give comptime_float an explicit type"),
        .int, .bool, .@"enum" => return hashBytes(std.mem.asBytes(&value)),
        .comptime_int => @compileError("hash: give comptime_int an explicit type"),
        .pointer => |ptr| {
            if (ptr.size == .slice and ptr.child == u8) return hashBytes(value);
            if (ptr.size == .one) switch (@typeInfo(ptr.child)) {
                // String literal, e.g. *const [14:0]u8
                .array => |arr| if (arr.child == u8) return hashBytes(value),
                else => {},
            };
            if (ptr.size == .slice) @compileError("hash: only []const u8 slices can be hashed, got " ++ @typeName(T));
            return hashBytes(std.mem.asBytes(&value)); // Hash<T *>: hash the pointer value itself
        },
        .@"struct", .@"union" => {
            if (!@hasDecl(T, "getHash")) @compileError("hash: " ++ @typeName(T) ++ " has no getHash() method");
            return value.getHash();
        },
        else => @compileError("hash: unsupported type " ++ @typeName(T)),
    }
}

/// Mix `value` into the hash `seed` (HashCombine).
/// See: https://github.com/jonmaiga/mx3/blob/master/mx3.h
pub fn hashCombine(seed: *u64, value: anytype) void {
    const c: u64 = 0xbea225f9eb34556d;
    var h = seed.*;
    var x = hash(value);

    // mix_stream(h, x)
    x *%= c;
    x ^= x >> 39;
    h +%= x *% c;
    h *%= c;

    // mix(h)
    h ^= h >> 32;
    h *%= c;
    h ^= h >> 29;
    h *%= c;
    h ^= h >> 32;
    h *%= c;
    h ^= h >> 29;

    seed.* = h;
}

/// Hash all values of a tuple together (HashCombineArgs). `values` must be a non-empty tuple.
pub fn hashCombineArgs(values: anytype) u64 {
    const fields = @typeInfo(@TypeOf(values)).@"struct".fields;
    if (fields.len == 0) @compileError("hashCombineArgs needs at least one value");

    // Prime the seed by hashing the first value
    var seed = hash(values[0]);

    // Hash all remaining values together
    inline for (1..fields.len) |i|
        hashCombine(&seed, values[i]);
    return seed;
}

test "hash -0 and +0 equally" {
    try std.testing.expectEqual(hash(@as(f32, 0.0)), hash(@as(f32, -0.0)));
    try std.testing.expectEqual(hash(@as(f64, 0.0)), hash(@as(f64, -0.0)));
}
