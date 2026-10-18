//! Zolt addition, no Jolt file: default construction and destruction of values, the C++ implicit constructors and
//! destructors that `new T`, `delete p`, `Array<T>::resize` and `Array<T>::clear` run.
//! Status: complete
//!
//! RTTI (`RTTI.createObject` / `RTTI.destructObject` of value classes) and the ObjectStream readers (Array members) use
//! these functions. Phase 8 foundation: Core/StreamIn.zig keeps a private copy of `defaultConstruct` / `destruct` /
//! `hasDestructor` today; the foundation group switches it to this file, so that the binary state and the object
//! streams share one definition (SerializationArchitecture.md, S7).
//!
//! Difference from C++: a field without a default value (a member that C++ leaves indeterminate, e.g. the channels of
//! `new Color`) is zero filled, so that every object that a stream creates is fully defined (decision S7, D9).

const std = @import("std");
const Allocator = std.mem.Allocator;

/// True when `T` declares `pub const empty: T` (std.ArrayList, StaticArray, Ref, ...), the Zig equivalent of a default
/// constructed container
fn hasEmptyValue(comptime T: type) bool {
    return @hasDecl(T, "empty") and @TypeOf(T.empty) == T;
}

/// The default constructor of a value (`T()`):
/// - `T.initDefault(allocator) Allocator.Error!T` or `T.initDefault() T` when T declares it (a constructor that
///   allocates, e.g. VehicleEngineSettings with its default torque curve);
/// - otherwise see `defaultConstruct`.
pub fn defaultValue(comptime T: type, allocator: Allocator) Allocator.Error!T {
    if (comptime @typeInfo(T) == .@"struct" and @hasDecl(T, "initDefault")) {
        const params = @typeInfo(@TypeOf(T.initDefault)).@"fn".params;
        return switch (params.len) {
            0 => T.initDefault(),
            1 => T.initDefault(allocator),
            else => @compileError(@typeName(T) ++ ".initDefault must take no arguments or an allocator"),
        };
    }
    var value: T = undefined;
    defaultConstruct(T, &value);
    return value;
}

/// Default construct `element` in place, like `new (element) T` in Array<T>::resize: a type that declares `empty` is
/// set to it, otherwise fields with a default value get that value, struct / array fields are default constructed
/// recursively (like the members of a C++ class) and all other fields are zero.
pub fn defaultConstruct(comptime T: type, element: *T) void {
    switch (@typeInfo(T)) {
        .@"struct" => |s| if (comptime hasEmptyValue(T)) {
            element.* = T.empty;
        } else if (comptime s.layout == .@"packed") {
            element.* = std.mem.zeroes(T);
        } else inline for (s.fields) |field| {
            if (comptime field.is_comptime) continue;
            if (comptime field.default_value_ptr != null)
                @field(element, field.name) = comptime field.defaultValue().?
            else
                defaultConstruct(field.type, &@field(element, field.name));
        },
        .array => |a| for (element) |*item| defaultConstruct(a.child, item),
        else => element.* = std.mem.zeroes(T),
    }
}

/// True when `destruct` does something for T (the C++ type has a non-trivial destructor)
pub fn hasDestructor(comptime T: type) bool {
    return switch (@typeInfo(T)) {
        .@"struct" => |s| @hasDecl(T, "deinit") or (s.layout != .@"packed" and for (s.fields) |field| {
            if (!field.is_comptime and hasDestructor(field.type))
                break true;
        } else false),
        .array => |a| hasDestructor(a.child),
        else => false,
    };
}

/// The destructor of a value (`element->~T()`): `deinit()` or `deinit(allocator)` when T declares it, otherwise the
/// struct / array fields are destructed recursively (the implicit C++ destructor)
pub fn destruct(comptime T: type, allocator: Allocator, element: *T) void {
    switch (@typeInfo(T)) {
        .@"struct" => |s| if (comptime @hasDecl(T, "deinit")) {
            switch (@typeInfo(@TypeOf(T.deinit)).@"fn".params.len) {
                1 => element.deinit(),
                2 => element.deinit(allocator),
                else => @compileError(@typeName(T) ++ ".deinit must take no arguments or an allocator"),
            }
        } else inline for (s.fields) |field| {
            if (comptime !field.is_comptime and s.layout != .@"packed" and hasDestructor(field.type))
                destruct(field.type, allocator, &@field(element, field.name));
        },
        .array => |a| if (comptime hasDestructor(a.child)) {
            for (element) |*item| destruct(a.child, allocator, item);
        },
        else => {},
    }
}

test "Lifetime: default values, zero filled members, destructors" {
    const Inner = struct {
        a: f32 = 1.5,
        b: u32,
    };
    const Outer = struct {
        inner: Inner = .{ .b = 7 },
        bare: Inner,
        list: std.ArrayList(u32) = .empty,
        array: [2]Inner,
    };
    var o = try defaultValue(Outer, std.testing.allocator);
    try std.testing.expectEqual(@as(u32, 7), o.inner.b);
    try std.testing.expectEqual(@as(f32, 1.5), o.bare.a);
    try std.testing.expectEqual(@as(u32, 0), o.bare.b); // Zero filled (C++: indeterminate)
    try std.testing.expectEqual(@as(u32, 0), o.array[1].b);
    try std.testing.expect(comptime hasDestructor(Outer));
    try o.list.append(std.testing.allocator, 3);
    destruct(Outer, std.testing.allocator, &o);
}
