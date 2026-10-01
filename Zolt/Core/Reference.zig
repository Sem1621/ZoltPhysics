//! Port of: Jolt/Core/Reference.h
//! Status: complete
//!
//! Zig has no destructors, so reference counting is explicit:
//! - A reference counted type (C++: derives from `RefTarget<T>`) embeds a `ref_count: RefCount`
//!   field and declares `addRef()` and `release()`. `release()` destroys the object when
//!   `RefCount.release()` returns true:
//!   ```zig
//!   pub fn addRef(self: *const Shape) void {
//!       self.ref_count.addRef();
//!   }
//!   pub fn release(self: *const Shape) void {
//!       if (self.ref_count.release()) self.vtable.destroy(@constCast(self));
//!   }
//!   ```
//! - `Ref<T>` / `RefConst<T>` members become `Ref(T)` / `RefConst(T)`. Assigning (`set`) adds a
//!   reference and releases the previous one, like the C++ assignment operators. The owner must call
//!   `deinit()` where the C++ destructor would run (usually in its own `deinit`).
//! - Plain `*T` / `*const T` are fine for non-owning pointers, exactly where the C++ uses raw pointers.

const std = @import("std");
const HashCombine = @import("HashCombine.zig");

/// The reference count of a RefTarget, embed it in a struct as field `ref_count`.
///
/// Reference counting classes keep an integer which indicates how many references
/// to the object are active. Reference counting objects start their life with a reference
/// count of zero. They can then be assigned to equivalents of pointers (Ref) which will increase
/// the reference count immediately. If the Ref is released or another object is assigned to the
/// reference counting pointer it will decrease the reference count of the object again. If this
/// reference count becomes zero, the object is destroyed.
///
/// This provides a very powerful mechanism to prevent memory leaks, but also gives
/// some responsibility to the programmer. The most notable point is that you cannot
/// have one object reference another and have the other reference the first one
/// back, because this way the reference count of both objects will never become
/// lower than 1, resulting in a memory leak. By carefully designing your classes
/// (and particularly identifying who owns who in the class hierarchy) you can avoid
/// these problems.
pub const RefCount = struct {
    /// A large value that gets added to the refcount to mark the object as embedded (cEmbedded)
    pub const embedded: u32 = 0x0ebedded;

    /// Current reference count (mutable in C++, all methods take a const pointer)
    value: std.atomic.Value(u32) = .init(0),

    /// Mark this object as embedded, this means the type can be used in a compound or constructed on the stack.
    /// The release function will never return true, it is assumed that whoever allocated the object destroys it
    /// and at that point in time it is checked that no references are left to the structure.
    pub fn setEmbedded(self: *const RefCount) void {
        const old = mutableValue(self).fetchAdd(embedded, .monotonic);
        std.debug.assert(old < embedded);
    }

    /// Get current refcount of this object (GetRefCount)
    pub fn get(self: *const RefCount) u32 {
        return self.value.load(.monotonic);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const RefCount) void {
        // Adding a reference can use relaxed memory ordering
        _ = mutableValue(self).fetchAdd(1, .monotonic);
    }

    /// Release a reference to this object. Returns true when this was the last reference,
    /// in which case the caller must destroy the object (the `delete` in the C++ Release).
    pub fn release(self: *const RefCount) bool {
        // Releasing a reference must use release semantics so that we can use acquire to ensure that we see any
        // updates from other threads that released a ref before deleting the object. Zig has no standalone fence,
        // so like Jolt's JPH_TSAN_ENABLED path this uses acq_rel on the decrement.
        const old_value = mutableValue(self).fetchSub(1, .acq_rel);
        std.debug.assert(old_value != 0 and old_value != embedded); // Too many calls to Release
        return old_value == 1;
    }

    /// Assert that no one is referencing the object (the check in the C++ destructor of RefTarget).
    /// Call this when destroying an object.
    pub fn assertUnreferenced(self: *const RefCount) void {
        const value = self.value.load(.monotonic);
        std.debug.assert(value == 0 or value == embedded);
    }

    fn mutableValue(self: *const RefCount) *std.atomic.Value(u32) {
        return &@constCast(self).value;
    }
};

/// Pure virtual version of RefTarget: a type erased reference counted object (interface pattern B)
pub const RefTargetVirtual = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Virtual add reference
        addRef: *const fn (ptr: *anyopaque) void,
        /// Virtual release reference
        release: *const fn (ptr: *anyopaque) void,
    };

    /// Wrap any `*T` with `addRef()` and `release()` methods
    pub fn init(impl: anytype) RefTargetVirtual {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn addRefThunk(ptr: *anyopaque) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.addRef();
            }
            fn releaseThunk(ptr: *anyopaque) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.release();
            }
            const vtable: VTable = .{ .addRef = addRefThunk, .release = releaseThunk };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    pub fn addRef(self: RefTargetVirtual) void {
        self.vtable.addRef(self.ptr);
    }

    pub fn release(self: RefTargetVirtual) void {
        self.vtable.release(self.ptr);
    }
};

/// Owning reference to a reference counted object, the equivalent of `Ref<T>` (a pointer to T that
/// keeps a reference). `T` must have `addRef()` and `release()` methods. Call `deinit` to release it.
pub fn Ref(comptime T: type) type {
    return RefImpl(T, *T);
}

/// Owning reference to a const reference counted object, the equivalent of `RefConst<T>`.
pub fn RefConst(comptime T: type) type {
    return RefImpl(T, *const T);
}

fn RefImpl(comptime T: type, comptime Ptr: type) type {
    return struct {
        const Self = @This();

        /// Pointer to object that we are reference counting
        ptr: ?Ptr = null,

        /// No object (Ref())
        pub const empty: Self = .{};

        /// Reference `ptr` (Ref(T *)), adds a reference
        pub fn init(ptr: ?Ptr) Self {
            if (ptr) |p| p.addRef();
            return .{ .ptr = ptr };
        }

        /// Copy this reference (copy constructor), adds a reference
        pub fn clone(self: Self) Self {
            return init(self.ptr);
        }

        /// Release the reference, use instead of the C++ destructor (or `ref = nullptr`)
        pub fn deinit(self: *Self) void {
            if (self.ptr) |p| p.release();
            self.ptr = null;
        }

        /// Point to another object (operator =), adds a reference to the new object and releases the old one
        pub fn set(self: *Self, ptr: ?Ptr) void {
            if (self.ptr != ptr) {
                if (ptr) |p| p.addRef();
                self.deinit();
                self.ptr = ptr;
            }
        }

        /// Get pointer (GetPtr)
        pub fn get(self: Self) ?Ptr {
            return self.ptr;
        }

        /// Comparison (operator ==)
        pub fn eql(self: Self, other: Self) bool {
            return self.ptr == other.ptr;
        }

        /// Get hash for this object (hashes the pointer, like Hash<T *>)
        pub fn getHash(self: Self) u64 {
            const address: usize = if (self.ptr) |p| @intFromPtr(p) else 0;
            return HashCombine.hash(address);
        }

        comptime {
            if (!@hasDecl(T, "addRef") or !@hasDecl(T, "release"))
                @compileError(@typeName(T) ++ " must declare addRef() and release() to be used with Ref/RefConst");
        }
    };
}

test "Ref / RefConst" {
    const Target = struct {
        const Self = @This();
        ref_count: RefCount = .{},
        allocator: std.mem.Allocator,
        destroyed: *bool,

        fn create(allocator: std.mem.Allocator, destroyed: *bool) !*Self {
            const self = try allocator.create(Self);
            self.* = .{ .allocator = allocator, .destroyed = destroyed };
            return self;
        }
        pub fn addRef(self: *const Self) void {
            self.ref_count.addRef();
        }
        pub fn release(self: *const Self) void {
            if (self.ref_count.release()) {
                self.ref_count.assertUnreferenced();
                self.destroyed.* = true;
                self.allocator.destroy(self);
            }
        }
    };

    var destroyed = false;
    const target = try Target.create(std.testing.allocator, &destroyed);

    var a = Ref(Target).init(target);
    try std.testing.expectEqual(@as(u32, 1), target.ref_count.get());

    var b = RefConst(Target).init(target);
    var c = a.clone();
    try std.testing.expectEqual(@as(u32, 3), target.ref_count.get());
    try std.testing.expect(a.eql(c));
    try std.testing.expectEqual(a.getHash(), c.getHash());

    a.deinit();
    c.set(null);
    try std.testing.expect(!destroyed);
    try std.testing.expectEqual(@as(u32, 1), target.ref_count.get());

    // Releasing the last reference destroys the object (std.testing.allocator checks for leaks)
    b.deinit();
    try std.testing.expect(destroyed);

    // Embedded objects are never destroyed by release
    var embedded_destroyed = false;
    var embedded: Target = .{ .allocator = std.testing.allocator, .destroyed = &embedded_destroyed };
    embedded.ref_count.setEmbedded();
    var d = Ref(Target).init(&embedded);
    d.deinit();
    try std.testing.expect(!embedded_destroyed);
    embedded.ref_count.assertUnreferenced();

    // Type erased version
    var virtual_destroyed = false;
    const virtual_target = try Target.create(std.testing.allocator, &virtual_destroyed);
    const virtual = RefTargetVirtual.init(virtual_target);
    virtual.addRef();
    virtual.release();
    try std.testing.expect(virtual_destroyed);
}
