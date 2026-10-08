//! Zolt addition, no Jolt file: the C++ virtual function machinery (proposed location Zolt/Core/Virtual.zig, imported as `virtual`)
//! Status: complete
//!
//! C++ single inheritance with virtual functions for class hierarchies with data in the base class (porting guide
//! pattern A): Shape, ShapeSettings, ConvexShape::Support, CollisionCollector, ShapeFilter, PhysicsMaterial,
//! GroupFilter and later Constraint, BroadPhase, ... See Docs/Zolt/CollisionArchitecture.md, decision D1.
//!
//! | C++                                          | Zig                                                                  |
//! |----------------------------------------------|----------------------------------------------------------------------|
//! | `class B : public A`                         | `B` embeds its parent as the field `base` (one level per C++ class)   |
//! | vptr                                         | `vtable: *const A.VTable` in the root class                          |
//! | a class that adds virtual functions          | `B.VTable = struct { base: A.VTable, newVirtual: *const fn ... }`    |
//! | vtable of a concrete class T (compiler made) | `virtual.make(B.VTable, T)`, called by the base class constructor   |
//! | `R Foo(...) override` in a concrete class    | `pub fn foo(self: *const T, ...) R` at top level, listed in `pub const overrides = .{ .foo, ... }` |
//! | body of a virtual in an abstract class       | `pub const impl = struct { pub fn foo(self: *const B, ...) R }`      |
//! | `p->Foo()` (virtual call)                    | `p.foo()` on a pointer to the class that introduces `Foo` (dispatcher) |
//! | `B::Foo()` (qualified call to a base)        | `B.impl.foo(&self.base, ...)` (abstract B) / `self.base.foo(...)` (concrete B) |
//! | pure virtual not overridden                  | compile error in `make`                                              |
//! | `virtual ~A()` + `delete this`               | generated vtable entries `deinit` (destructor chain: `destruct` of every level, derived first) and `destroy` (`deinit` + free with the root's `allocator`) |
//! | `static_cast<C *>(a)` / derived to base      | `virtual.downcast(C, a)` / `virtual.upcast(A, c)`                    |
//! | JPH_RTTI (class name for the RTTI hash)      | a data entry `rtti_name: []const u8` in the VTable, every concrete class declares `pub const rtti_name` |
//!
//! Rules enforced at compile time by `make`:
//! - every virtual function has an implementation somewhere in the chain T -> ... -> introducing class
//!   ("T must implement the pure virtual function X.foo");
//! - the implementation has the parameter and return types of the virtual function (the C++ `override` check);
//! - a concrete class declares `overrides`: every listed name is a `pub fn` of the class and a virtual function, and
//!   every `pub` declaration of the class that has the name of a virtual function is listed (so a misspelled,
//!   private or forgotten override is an error instead of a silent fallback to the base class version);
//! - an abstract class keeps the bodies of virtual functions in `impl`; a top-level declaration with the name of a
//!   virtual function that the class does not introduce is an error (it would be a static call where C++ makes a
//!   virtual call);
//! - a class declares `overrides` or `impl`, never both.
//!
//! Overrides are looked up in every level of the chain (not only in the most derived class), so a class that
//! derives from a concrete class (e.g. a user collector derived from ClosestHitPerBodyCollisionCollector) inherits
//! its parent's overrides exactly like in C++.

const std = @import("std");

/// The parent class of `T` (the type of its field `base`)
pub fn Parent(comptime T: type) type {
    return @FieldType(T, "base");
}

/// True when `Derived` is `Base` or derives from it
pub fn isDerivedFrom(comptime Derived: type, comptime Base: type) bool {
    comptime var X: type = Derived;
    inline while (true) {
        if (X == Base) return true;
        if (!@hasField(X, "base")) return false;
        X = Parent(X);
    }
}

/// Pointer to `To` with the constness of the pointer type `P`
pub fn PtrLike(comptime P: type, comptime To: type) type {
    return if (@typeInfo(P).pointer.is_const) *const To else *To;
}

/// Derived to base pointer conversion (implicit in C++), any number of levels, keeps constness
pub fn upcast(comptime To: type, ptr: anytype) PtrLike(@TypeOf(ptr), To) {
    const From = @typeInfo(@TypeOf(ptr)).pointer.child;
    if (From == To) {
        return ptr;
    } else {
        return upcast(To, &ptr.base);
    }
}

/// `static_cast<To *>(ptr)` from a base class pointer (any number of levels), keeps constness. Unchecked: callers
/// that can receive another type check the dynamic type first (e.g. `Shape.cast` asserts the sub shape type).
pub fn downcast(comptime To: type, ptr: anytype) PtrLike(@TypeOf(ptr), To) {
    const From = @typeInfo(@TypeOf(ptr)).pointer.child;
    if (From == To) {
        return ptr;
    } else {
        if (!@hasField(To, "base")) @compileError(@typeName(To) ++ " does not derive from " ++ @typeName(From));
        const parent: PtrLike(@TypeOf(ptr), Parent(To)) = downcast(Parent(To), ptr);
        return @alignCast(@fieldParentPtr("base", parent));
    }
}

/// Value types with non virtual inheritance that are flattened (RayCastResult : BroadPhaseCastResult,
/// CollideShapeSettings : CollideSettingsBase, ...): compile error unless the first fields of `Derived` have the
/// names and types of the fields of `Base`, in the same order.
pub fn checkPrefix(comptime Base: type, comptime Derived: type) void {
    const base_fields = @typeInfo(Base).@"struct".fields;
    const derived_fields = @typeInfo(Derived).@"struct".fields;
    if (derived_fields.len < base_fields.len) @compileError(@typeName(Derived) ++ " must start with the fields of " ++ @typeName(Base));
    for (base_fields, derived_fields[0..base_fields.len]) |b, d| {
        if (!std.mem.eql(u8, b.name, d.name) or b.type != d.type)
            @compileError(@typeName(Derived) ++ "." ++ d.name ++ " does not match the base field " ++ @typeName(Base) ++ "." ++ b.name);
    }
}

/// Build the vtable of type `VTable` for the concrete class `T` (what the C++ compiler generates for a class).
/// A field `base` of `VTable` (the vtable of the parent class) is built the same way. Entries named `deinit` and
/// `destroy` are generated (virtual destructor), all others resolve to the most derived implementation.
pub fn make(comptime VTable: type, comptime T: type) VTable {
    comptime {
        @setEvalBranchQuota(200_000);
        checkClasses(VTable, T);
        return makeTable(VTable, T);
    }
}

fn makeTable(comptime VTable: type, comptime T: type) VTable {
    var vt: VTable = undefined;
    for (@typeInfo(VTable).@"struct".fields) |field| {
        if (isBaseTable(field)) {
            @field(vt, field.name) = makeTable(field.type, T);
        } else if (std.mem.eql(u8, field.name, "deinit")) {
            @field(vt, field.name) = deinitChain(Introducer(field.type), T);
        } else if (std.mem.eql(u8, field.name, "destroy")) {
            @field(vt, field.name) = destroyFn(Introducer(field.type), T);
        } else if (@typeInfo(field.type) != .pointer or @typeInfo(@typeInfo(field.type).pointer.child) != .@"fn") {
            // Data entry (e.g. `rtti_name`, JPH_RTTI): every concrete class declares its own value
            if (!@hasDecl(T, field.name)) @compileError(@typeName(T) ++ " must declare `pub const " ++ field.name ++ "`");
            @field(vt, field.name) = @field(T, field.name);
        } else {
            @field(vt, field.name) = entry(field.type, T, field.name);
        }
    }
    return vt;
}

fn isBaseTable(comptime field: std.builtin.Type.StructField) bool {
    return std.mem.eql(u8, field.name, "base") and @typeInfo(field.type) == .@"struct";
}

fn FnInfo(comptime FnPtr: type) std.builtin.Type.Fn {
    return @typeInfo(@typeInfo(FnPtr).pointer.child).@"fn";
}

/// The class that introduces a virtual function: the type that its first parameter points to
fn Introducer(comptime FnPtr: type) type {
    return @typeInfo(FnInfo(FnPtr).params[0].type.?).pointer.child;
}

fn isGenerated(comptime name: []const u8) bool {
    return std.mem.eql(u8, name, "deinit") or std.mem.eql(u8, name, "destroy");
}

fn isFunctionEntry(comptime field: std.builtin.Type.StructField) bool {
    return @typeInfo(field.type) == .pointer and @typeInfo(@typeInfo(field.type).pointer.child) == .@"fn" and !isGenerated(field.name);
}

/// True if `name` is a virtual function of `VTable` (including the vtables of the parent classes)
fn isVirtual(comptime VTable: type, comptime name: []const u8) bool {
    for (@typeInfo(VTable).@"struct".fields) |field| {
        if (isBaseTable(field)) {
            if (isVirtual(field.type, name)) return true;
        } else if (std.mem.eql(u8, field.name, name) and isFunctionEntry(field)) {
            return true;
        }
    }
    return false;
}

/// True if class `L` introduces virtual function `name` (it is a direct entry of `L.VTable`): then the top-level
/// declaration `L.name` is the dispatcher of that virtual function.
fn introduces(comptime L: type, comptime name: []const u8) bool {
    if (!@hasDecl(L, "VTable")) return false;
    if (@TypeOf(L.VTable) != type or @typeInfo(L.VTable) != .@"struct") return false;
    for (@typeInfo(L.VTable).@"struct".fields) |field| {
        if (!isBaseTable(field) and std.mem.eql(u8, field.name, name)) return true;
    }
    return false;
}

fn isListed(comptime L: type, comptime name: []const u8) bool {
    for (L.overrides) |o| {
        if (std.mem.eql(u8, @tagName(o), name)) return true;
    }
    return false;
}

/// Check the `overrides` / `impl` rules for every class of the chain T -> root
fn checkClasses(comptime VTable: type, comptime T: type) void {
    if (!@hasDecl(T, "overrides") and !@hasDecl(T, "impl"))
        @compileError(@typeName(T) ++ " must declare `pub const overrides = .{ ... }` (the virtual functions it overrides, C++ `override`)");
    var L = T;
    while (true) {
        if (@hasDecl(L, "overrides") and @hasDecl(L, "impl"))
            @compileError(@typeName(L) ++ " declares both `overrides` (concrete class) and `impl` (abstract class)");
        if (@hasDecl(L, "overrides")) {
            for (L.overrides) |o| {
                const name = @tagName(o);
                if (!@hasDecl(L, name))
                    @compileError(@typeName(L) ++ ".overrides lists " ++ name ++ ", but " ++ @typeName(L) ++ " has no pub declaration " ++ name);
                if (!isVirtual(VTable, name))
                    @compileError(@typeName(L) ++ ".overrides lists " ++ name ++ ", which is not a virtual function of " ++ @typeName(VTable));
            }
        }
        checkTopLevel(VTable, VTable, L);
        if (!@hasField(L, "base")) break;
        L = Parent(L);
    }
}

/// A top-level declaration of `L` with the name of a virtual function must be a listed override (concrete class) or
/// the dispatcher of a virtual function that `L` introduces
fn checkTopLevel(comptime Full: type, comptime VTable: type, comptime L: type) void {
    for (@typeInfo(VTable).@"struct".fields) |field| {
        if (isBaseTable(field)) {
            checkTopLevel(Full, field.type, L);
        } else if (isFunctionEntry(field) and @hasDecl(L, field.name) and !introduces(L, field.name)) {
            if (@hasDecl(L, "overrides")) {
                if (!isListed(L, field.name))
                    @compileError(@typeName(L) ++ "." ++ field.name ++ " overrides a virtual function: add ." ++ field.name ++ " to " ++ @typeName(L) ++ ".overrides (or rename it if it is not an override)");
            } else {
                @compileError(@typeName(L) ++ "." ++ field.name ++ " has the name of a virtual function: an abstract class keeps its implementation in " ++ @typeName(L) ++ ".impl (a top-level function would be called statically)");
            }
        }
    }
}

/// The class between `T` and `Root` (inclusive) whose implementation of virtual `name` is the most derived one
fn findLevel(comptime T: type, comptime Root: type, comptime name: []const u8) ?type {
    var L = T;
    while (true) {
        if (@hasDecl(L, "overrides")) {
            if (isListed(L, name)) return L;
        } else if (@hasDecl(L, "impl")) {
            if (@hasDecl(L.impl, name)) return L;
        }
        if (L == Root) return null;
        if (!@hasField(L, "base")) @compileError(@typeName(T) ++ " does not derive from " ++ @typeName(Root));
        L = Parent(L);
    }
}

fn entry(comptime FnPtr: type, comptime T: type, comptime name: []const u8) FnPtr {
    const Root = Introducer(FnPtr);
    const L = findLevel(T, Root, name) orelse
        @compileError(@typeName(T) ++ " must implement the pure virtual function " ++ @typeName(Root) ++ "." ++ name);
    const func = if (@hasDecl(L, "overrides")) @field(L, name) else @field(L.impl, name);
    checkSignature(FnPtr, L, @typeName(L) ++ "." ++ name ++ " (in the vtable of " ++ @typeName(T) ++ ")", @TypeOf(func));
    return thunk(FnPtr, L, func);
}

/// Readable compile errors when an implementation does not have the signature of the virtual function
fn checkSignature(comptime FnPtr: type, comptime L: type, comptime where: []const u8, comptime Impl: type) void {
    if (@typeInfo(Impl) != .@"fn") @compileError(where ++ " is not a function");
    const expected = FnInfo(FnPtr);
    const actual = @typeInfo(Impl).@"fn";
    if (actual.params.len != expected.params.len)
        @compileError(std.fmt.comptimePrint("{s} has {d} parameters, the virtual function has {d}", .{ where, actual.params.len, expected.params.len }));
    const self_type = actual.params[0].type orelse @compileError(where ++ ": self cannot be anytype");
    const virtual_is_const = @typeInfo(expected.params[0].type.?).pointer.is_const;
    if (self_type != *const L and self_type != *L)
        @compileError(where ++ ": the first parameter must be *const " ++ @typeName(L) ++ " or *" ++ @typeName(L) ++ ", found " ++ @typeName(self_type));
    if (virtual_is_const and self_type == *L)
        @compileError(where ++ ": the virtual function is const, the first parameter must be *const " ++ @typeName(L));
    for (expected.params[1..], actual.params[1..], 1..) |e, a, i| {
        if (a.type == null or e.type.? != a.type.?)
            @compileError(std.fmt.comptimePrint("{s}: parameter {d} must have type {s}", .{ where, i, @typeName(e.type.?) }));
    }
    if (actual.return_type == null or actual.return_type.? != expected.return_type.?)
        @compileError(where ++ " must return " ++ @typeName(expected.return_type.?));
}

/// A function of type `FnPtr` that downcasts its first parameter to the class `L` and calls `func`
fn thunk(comptime FnPtr: type, comptime L: type, comptime func: anytype) FnPtr {
    const info = FnInfo(FnPtr);
    const P = info.params;
    const R = info.return_type.?;
    const S = P[0].type.?;
    const C = struct {
        fn self(s: S) PtrLike(S, L) {
            return downcast(L, s);
        }
        fn t(comptime i: usize) type {
            return P[i].type.?;
        }
    };
    const Thunks = struct {
        fn f1(s: S) R {
            return func(C.self(s));
        }
        fn f2(s: S, a1: C.t(1)) R {
            return func(C.self(s), a1);
        }
        fn f3(s: S, a1: C.t(1), a2: C.t(2)) R {
            return func(C.self(s), a1, a2);
        }
        fn f4(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3)) R {
            return func(C.self(s), a1, a2, a3);
        }
        fn f5(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4)) R {
            return func(C.self(s), a1, a2, a3, a4);
        }
        fn f6(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5)) R {
            return func(C.self(s), a1, a2, a3, a4, a5);
        }
        fn f7(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6);
        }
        fn f8(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6), a7: C.t(7)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6, a7);
        }
        fn f9(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6), a7: C.t(7), a8: C.t(8)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6, a7, a8);
        }
        fn f10(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6), a7: C.t(7), a8: C.t(8), a9: C.t(9)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6, a7, a8, a9);
        }
        fn f11(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6), a7: C.t(7), a8: C.t(8), a9: C.t(9), a10: C.t(10)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
        }
        fn f12(s: S, a1: C.t(1), a2: C.t(2), a3: C.t(3), a4: C.t(4), a5: C.t(5), a6: C.t(6), a7: C.t(7), a8: C.t(8), a9: C.t(9), a10: C.t(10), a11: C.t(11)) R {
            return func(C.self(s), a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11);
        }
    };
    return switch (P.len) {
        1 => &Thunks.f1,
        2 => &Thunks.f2,
        3 => &Thunks.f3,
        4 => &Thunks.f4,
        5 => &Thunks.f5,
        6 => &Thunks.f6,
        7 => &Thunks.f7,
        8 => &Thunks.f8,
        9 => &Thunks.f9,
        10 => &Thunks.f10,
        11 => &Thunks.f11,
        12 => &Thunks.f12,
        else => @compileError("virtual functions with more than 12 parameters (including self) are not supported, add a thunk to Virtual.zig"),
    };
}

/// Generated `deinit` entry: the C++ destructor chain. Calls `destruct` of every class from `T` up to `Root`
/// (derived first). A class declares `pub fn destruct(self: *Class) void` only if it owns members (Refs, arrays).
fn deinitChain(comptime Root: type, comptime T: type) *const fn (self: *Root) void {
    return &struct {
        fn f(self: *Root) void {
            comptime var L = T;
            inline while (true) {
                if (@hasDecl(L, "destruct")) L.destruct(downcast(L, self));
                if (L == Root) break;
                L = Parent(L);
            }
        }
    }.f;
}

/// Generated `destroy` entry: `delete this` through the virtual destructor. Runs the destructor chain and frees the
/// object with the allocator stored in the root class.
fn destroyFn(comptime Root: type, comptime T: type) *const fn (self: *Root) void {
    return &struct {
        fn f(self: *Root) void {
            self.ref_count.assertUnreferenced();
            const allocator = self.allocator;
            deinitChain(Root, T)(self);
            allocator.destroy(downcast(T, self));
        }
    }.f;
}

test "Virtual: levels, overrides, impl, pure virtuals, destructor chain" {
    const A = struct {
        const Self = @This();
        pub const VTable = struct {
            deinit: *const fn (self: *Self) void,
            name: *const fn (self: *const Self) []const u8,
            value: *const fn (self: *const Self, x: i32) i32,
        };
        vtable: *const VTable,
        destructed: *u32,

        pub fn name(self: *const Self) []const u8 {
            return self.vtable.name(self);
        }
        pub fn value(self: *const Self, x: i32) i32 {
            return self.vtable.value(self, x);
        }
        pub fn destruct(self: *Self) void {
            self.destructed.* += 1;
        }
        pub const impl = struct {
            pub fn value(self: *const Self, x: i32) i32 {
                _ = self;
                return x;
            }
        };
    };
    const B = struct {
        const Self = @This();
        pub const VTable = struct {
            base: A.VTable,
            extra: *const fn (self: *const Self) i32,
        };
        base: A,
        b: i32,

        pub fn extra(self: *const Self) i32 {
            const vt: *const VTable = downcast(VTable, self.base.vtable);
            return vt.extra(self);
        }
        pub fn destruct(self: *Self) void {
            self.base.destructed.* += 10;
        }
        pub const impl = struct {
            pub fn value(self: *const Self, x: i32) i32 {
                return A.impl.value(&self.base, x) + self.b; // B::Value calls A::Value
            }
        };
    };
    const C = struct {
        const Self = @This();
        pub const overrides = .{ .name, .extra };
        base: B,
        c: i32,

        const vtable = make(B.VTable, Self);

        pub fn name(self: *const Self) []const u8 {
            _ = self;
            return "C";
        }
        pub fn extra(self: *const Self) i32 {
            return upcast(A, self).value(0) * 10 + self.c; // unqualified Value() in C++: virtual call
        }
        pub fn destruct(self: *Self) void {
            self.base.base.destructed.* += 100;
        }
    };

    var destructed: u32 = 0;
    var c: C = .{ .base = .{ .base = .{ .vtable = &C.vtable.base, .destructed = &destructed }, .b = 2 }, .c = 3 };
    const a: *const A = upcast(A, &c);
    try std.testing.expectEqualStrings("C", a.name());
    try std.testing.expectEqual(@as(i32, 7), a.value(5));
    try std.testing.expectEqual(@as(i32, 23), downcast(B, a).extra());
    try std.testing.expect(downcast(C, a) == &c);
    try std.testing.expect(isDerivedFrom(C, A) and !isDerivedFrom(A, C));
    c.base.base.vtable.deinit(upcast(A, &c));
    try std.testing.expectEqual(@as(u32, 111), destructed);
}
