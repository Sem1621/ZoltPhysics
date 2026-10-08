//! Port of: Jolt/Physics/Collision/PhysicsMaterial.h, Jolt/Physics/Collision/PhysicsMaterial.cpp
//! Status: complete
//!
//! Pattern A RefTarget root (Docs/Zolt/CollisionArchitecture.md, D1, D2 and D8): the vtable, the atomic reference
//! count, the allocator that frees the object (and the memory it owns) and `is_static`.
//!
//! - A derived material embeds `base: PhysicsMaterial = .init(@This(), allocator)` (the base class constructor), lists
//!   its `overrides` and declares `pub const rtti_name` (JPH_RTTI). `new X(...)` is `X.create(allocator, ...)`
//!   (reference count 0, put it in a `Ref` / `RefConst`), the last `release()` destroys it through the vtable.
//!   A material on the stack or embedded in another object calls `setEmbedded()` before references are taken and
//!   `deinit()` at the end.
//! - `PhysicsMaterial::sDefault` (a mutable global that RegisterTypes sets) is `PhysicsMaterial.default`, a pointer
//!   to a compile time constant in read-only memory (PhysicsMaterialSimple("Default", Color::sGrey), what
//!   RegisterTypes creates). Static materials (`is_static`) are never reference counted (addRef / release do
//!   nothing), so `RefConst(PhysicsMaterial).init(PhysicsMaterial.default)` is legal and nothing writes to read-only
//!   memory.
//! - SaveBinaryState writes Jolt's RTTI hash of the class name (`rtti_name` is a vtable data entry);
//!   sRestoreFromBinaryState (StreamUtils::RestoreObject) finds the class in the comptime list `material_types` (the
//!   Factory of Phase 8 replaces it). Restoring allocates, so it returns `Allocator.Error!PhysicsMaterialResult`;
//!   Jolt's errors ("Failed to read type hash", ...) are values in the result.
//! - `default` and `material_types` come from RegisterTypes.zig (`RegisterTypes.default_material` /
//!   `RegisterTypes.material_types`), where the `zolt_user_types` module can replace the default material and add
//!   material types (D4, D8).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../Core/Color.zig").Color;
const HashCombine = @import("../../Core/HashCombine.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const Ref = @import("../../Core/Reference.zig").Ref;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const Result = @import("../../Core/Result.zig").Result;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const virtual = @import("../../Core/Virtual.zig");
const PhysicsMaterialSimple = @import("PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// This structure describes the surface of (part of) a shape. You should inherit from it to define additional
/// information that is interesting for the simulation. The 2 materials involved in a contact could be used
/// to decide which sound or particle effects to play.
///
/// If you inherit from this material, don't forget to create a suitable default material in sDefault
pub const PhysicsMaterial = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, PhysicsMaterial)

    pub const VTable = struct {
        /// Class name of the most derived class (JPH_RTTI, data entry: every concrete class declares `rtti_name`)
        rtti_name: []const u8,
        /// Destructor chain (generated, virtual ~PhysicsMaterial)
        deinit: *const fn (self: *PhysicsMaterial) void,
        /// delete this (generated)
        destroy: *const fn (self: *PhysicsMaterial) void,
        // Properties
        getDebugName: *const fn (self: *const PhysicsMaterial) []const u8,
        getDebugColor: *const fn (self: *const PhysicsMaterial) Color,
        /// Saves the contents of the material in binary form to inStream.
        saveBinaryState: *const fn (self: *const PhysicsMaterial, stream: StreamOut) void,
        /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
        restoreBinaryState: *const fn (self: *PhysicsMaterial, stream: StreamIn) Allocator.Error!void,
    };

    /// Name of the class for Jolt's RTTI hash (JPH_RTTI)
    pub const rtti_name = "PhysicsMaterial";

    vtable: *const VTable,
    /// Reference count (RefTarget<PhysicsMaterial>)
    ref_count: RefCount = .{},
    /// Frees this material (heap materials) and the memory it owns
    allocator: Allocator,
    /// A compile time constant in read-only memory: never reference counted, never destroyed
    is_static: bool = false,

    /// Default material that is used when a shape has no materials defined (PhysicsMaterial::sDefault)
    pub const default: *const PhysicsMaterial = RegisterTypes.default_material;

    /// Material classes that restoreFromBinaryState can create (Factory::sInstance until Phase 8). Each one declares
    /// `rtti_name` and `createDefault(allocator) Allocator.Error!*T` (its default constructor on the heap).
    pub const material_types = RegisterTypes.material_types;

    pub const PhysicsMaterialResult = Result(Ref(PhysicsMaterial));

    /// Constructor, called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) PhysicsMaterial {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// Constructor of a compile time constant (static) material
    pub fn initStatic(comptime T: type) PhysicsMaterial {
        return .{ .vtable = vtableFor(T), .allocator = .failing, .is_static = true };
    }

    /// new PhysicsMaterial: reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator) Allocator.Error!*PhysicsMaterial {
        const self = try allocator.create(PhysicsMaterial);
        self.* = .init(PhysicsMaterial, allocator);
        return self;
    }

    /// new PhysicsMaterial (the default constructor, used by restoreFromBinaryState)
    pub const createDefault = create;

    /// The vtable of material class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<PhysicsMaterial>

    /// Add a reference to this object (nothing for static materials)
    pub fn addRef(self: *const PhysicsMaterial) void {
        if (!self.is_static) self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference (nothing for static materials)
    pub fn release(self: *const PhysicsMaterial) void {
        if (!self.is_static and self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this material as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const PhysicsMaterial) void {
        std.debug.assert(!self.is_static);
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const PhysicsMaterial) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a material that is not on the heap (embedded / stack)
    pub fn deinit(self: *PhysicsMaterial) void {
        std.debug.assert(!self.is_static);
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    // Properties

    pub fn getDebugName(self: *const PhysicsMaterial) []const u8 {
        return self.vtable.getDebugName(self);
    }

    pub fn getDebugColor(self: *const PhysicsMaterial) Color {
        return self.vtable.getDebugColor(self);
    }

    /// Saves the contents of the material in binary form to inStream.
    pub fn saveBinaryState(self: *const PhysicsMaterial, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *PhysicsMaterial, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Creates a PhysicsMaterial of the correct type and restores its contents from the binary stream inStream.
    /// (StreamUtils::RestoreObject with the comptime type list `material_types` instead of the Factory)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!PhysicsMaterialResult {
        var result: PhysicsMaterialResult = .empty;

        // Read the hash of the type
        var hash: u32 = 0;
        stream.read(&hash);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read type hash");
            return result;
        }

        // Get the RTTI for the type and construct it
        const object: *PhysicsMaterial = inline for (material_types) |T| {
            if (hash == comptime rttiHash(T.rtti_name)) break virtual.upcast(PhysicsMaterial, try T.createDefault(allocator));
        } else {
            result.setError("Failed to create instance of type");
            return result;
        };

        // Read the data of the type
        var ref = Ref(PhysicsMaterial).init(object);
        defer ref.deinit();
        try object.restoreBinaryState(stream);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to restore object");
            return result;
        }

        result.set(ref.clone());
        return result;
    }

    /// GetRTTI()->GetHash() of the most derived class
    pub fn getRTTIHash(self: *const PhysicsMaterial) u32 {
        return rttiHash(self.vtable.rtti_name);
    }

    /// RTTI::GetHash: FNV-1a hash of the class name, folded from 64 to 32 bits
    pub fn rttiHash(name: []const u8) u32 {
        // Perform diffusion step to get from 64 to 32 bits (see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function)
        const hash = HashCombine.hashString(name);
        return @truncate(hash ^ (hash >> 32));
    }

    /// Implementations of the virtual functions in PhysicsMaterial
    pub const impl = struct {
        pub fn getDebugName(self: *const PhysicsMaterial) []const u8 {
            _ = self;
            return "Unknown";
        }

        pub fn getDebugColor(self: *const PhysicsMaterial) Color {
            _ = self;
            return Color.grey;
        }

        pub fn saveBinaryState(self: *const PhysicsMaterial, stream: StreamOut) void {
            stream.write(self.getRTTIHash());
        }

        pub fn restoreBinaryState(self: *PhysicsMaterial, stream: StreamIn) Allocator.Error!void {
            // RTTI hash is read in sRestoreFromBinaryState
            _ = self;
            _ = stream;
        }
    };
};

/// Array<RefConst<PhysicsMaterial>>
pub const PhysicsMaterialList = std.ArrayList(RefConst(PhysicsMaterial));

const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// Saves `material` into `buffer`, returns the written bytes
fn saveToBuffer(material: *const PhysicsMaterial, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    material.saveBinaryState(out.streamOut());
    return writer.buffered();
}

/// Restores a material from `bytes`
fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!PhysicsMaterial.PhysicsMaterialResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return PhysicsMaterial.restoreFromBinaryState(allocator, in.streamIn());
}

test "PhysicsMaterial: default material, reference counting, base class" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // The default material is a compile time constant: reference counting it never writes to read-only memory
    const default = PhysicsMaterial.default;
    try expect(default.is_static);
    try std.testing.expectEqualStrings("Default", default.getDebugName());
    try expect(default.getDebugColor().eql(Color.grey));
    var default_ref = RefConst(PhysicsMaterial).init(default);
    var default_ref2 = default_ref.clone();
    default_ref2.deinit();
    default_ref.deinit();
    try std.testing.expectEqual(@as(u32, 0), default.getRefCount());

    // The base class itself is a concrete class
    const material = try PhysicsMaterial.create(allocator);
    var material_ref = Ref(PhysicsMaterial).init(material);
    defer material_ref.deinit();
    try std.testing.expectEqualStrings("Unknown", material.getDebugName());
    try expect(material.getDebugColor().eql(Color.grey));
    try std.testing.expectEqual(@as(u32, 1), material.getRefCount());
    try std.testing.expectEqual(PhysicsMaterial.rttiHash("PhysicsMaterial"), material.getRTTIHash());

    // Its binary state is only the RTTI hash
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(material, &buffer);
    try std.testing.expectEqual(@as(usize, 4), bytes.len);
    var restored = try restoreFromBuffer(allocator, bytes);
    defer restored.deinit();
    try expect(restored.isValid());
    try std.testing.expectEqual(material.getRTTIHash(), restored.getPtr().?.getRTTIHash());

    // A material on the stack
    var embedded = PhysicsMaterial.init(PhysicsMaterial, allocator);
    embedded.setEmbedded();
    var embedded_ref = RefConst(PhysicsMaterial).init(&embedded);
    embedded_ref.deinit();
    embedded.deinit();

    var list: PhysicsMaterialList = .empty;
    defer {
        for (list.items) |*m| m.deinit();
        list.deinit(allocator);
    }
    try list.append(allocator, .init(default));
    try list.append(allocator, .init(material));
    try std.testing.expectEqual(@as(u32, 2), material.getRefCount());
}

test "PhysicsMaterial: restore errors" {
    const allocator = std.testing.allocator;

    // Empty stream
    var r1 = try restoreFromBuffer(allocator, &.{});
    defer r1.deinit();
    try std.testing.expectEqualStrings("Failed to read type hash", r1.getError());

    // Unknown type
    var r2 = try restoreFromBuffer(allocator, &.{ 1, 2, 3, 4 });
    defer r2.deinit();
    try std.testing.expectEqualStrings("Failed to create instance of type", r2.getError());

    // Truncated data of a PhysicsMaterialSimple (the object is created, restoring hits EOF, nothing leaks)
    const hash = PhysicsMaterial.rttiHash("PhysicsMaterialSimple");
    var r3 = try restoreFromBuffer(allocator, std.mem.asBytes(&hash));
    defer r3.deinit();
    try std.testing.expectEqualStrings("Failed to restore object", r3.getError());
}
