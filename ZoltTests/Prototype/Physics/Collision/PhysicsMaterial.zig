//! Port of: Jolt/Physics/Collision/PhysicsMaterial.h, Jolt/Physics/Collision/PhysicsMaterial.cpp
//! Status: complete
//! Not ported: JPH_DECLARE_SERIALIZABLE_VIRTUAL (ObjectStream, Phase 8), see TODO(serialization)
//!
//! Pattern A RefTarget root (D8): vtable, atomic reference count, the allocator that frees it, and `is_static`.
//! - `PhysicsMaterial::sDefault` (a mutable global set by RegisterTypes) is `PhysicsMaterial.default`, a pointer to a
//!   compile time constant in read-only memory. Static materials (`is_static`) are never reference counted (addRef /
//!   release do nothing), so `RefConst(PhysicsMaterial).init(PhysicsMaterial.default)` is legal and never writes to
//!   read-only memory. Jolt's "create a suitable default material in sDefault" becomes the optional
//!   `default_material` declaration of the user types module (see RegisterTypes.zig).
//! - SaveBinaryState writes Jolt's RTTI hash of the class name; sRestoreFromBinaryState finds the type in the
//!   comptime list `RegisterTypes.material_types` (the Factory of Phase 8 replaces it).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const Color = zolt.Color;
const HashCombine = zolt.HashCombine;
const RefCount = zolt.RefCount;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const virtual = @import("../../Core/Virtual.zig");
const Result = @import("../../Core/Result.zig").Result;
const PhysicsMaterialSimple = @import("PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// This structure describes the surface of (part of) a shape. You should inherit from it to define additional
/// information that is interesting for the simulation. The 2 materials involved in a contact could be used
/// to decide which sound or particle effects to play.
///
/// If you inherit from this material, don't forget to create a suitable default material in sDefault
pub const PhysicsMaterial = struct {
    pub const VTable = struct {
        /// Class name of the most derived class (JPH_RTTI, data entry: every concrete class declares `rtti_name`)
        rtti_name: []const u8,
        /// Destructor chain (generated)
        deinit: *const fn (self: *PhysicsMaterial) void,
        /// delete this (generated)
        destroy: *const fn (self: *PhysicsMaterial) void,
        getDebugName: *const fn (self: *const PhysicsMaterial) []const u8,
        getDebugColor: *const fn (self: *const PhysicsMaterial) Color,
        /// Saves the contents of the material in binary form to stream.
        saveBinaryState: *const fn (self: *const PhysicsMaterial, stream: StreamOut) void,
        /// Protected in Jolt, used by restoreFromBinaryState
        restoreBinaryState: *const fn (self: *PhysicsMaterial, stream: StreamIn) Allocator.Error!void,
    };

    /// Name of the class for Jolt's RTTI hash (JPH_RTTI)
    pub const rtti_name = "PhysicsMaterial";

    vtable: *const VTable,
    ref_count: RefCount = .{},
    /// Frees this material (heap materials) and its owned memory
    allocator: Allocator,
    /// A compile time constant in read-only memory: never reference counted, never destroyed
    is_static: bool = false,

    /// Default material that is used when a shape has no materials defined (PhysicsMaterial::sDefault)
    pub const default: *const PhysicsMaterial = RegisterTypes.default_material;

    pub const PhysicsMaterialResult = Result(Ref(PhysicsMaterial));

    /// Constructor, called by derived classes with their own type
    pub fn init(comptime T: type, allocator: Allocator) PhysicsMaterial {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// Constructor of a compile time constant (static) material
    pub fn initStatic(comptime T: type) PhysicsMaterial {
        return .{ .vtable = vtableFor(T), .allocator = .failing, .is_static = true };
    }

    /// new PhysicsMaterial
    pub fn create(allocator: Allocator) Allocator.Error!*PhysicsMaterial {
        const self = try allocator.create(PhysicsMaterial);
        self.* = .init(PhysicsMaterial, allocator);
        return self;
    }

    /// new PhysicsMaterial (default constructor, used by restoreFromBinaryState)
    pub const createDefault = create;

    pub fn vtableFor(comptime T: type) *const VTable {
        return &struct {
            const vt = virtual.make(VTable, T);
        }.vt;
    }

    // RefTarget<PhysicsMaterial>
    pub fn addRef(self: *const PhysicsMaterial) void {
        if (!self.is_static) self.ref_count.addRef();
    }

    pub fn release(self: *const PhysicsMaterial) void {
        if (!self.is_static and self.ref_count.release()) self.vtable.destroy(@constCast(self));
    }

    pub fn setEmbedded(self: *const PhysicsMaterial) void {
        self.ref_count.setEmbedded();
    }

    /// Destructor of a material that is not on the heap (embedded / stack)
    pub fn deinit(self: *PhysicsMaterial) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    // Virtual dispatchers
    pub fn getDebugName(self: *const PhysicsMaterial) []const u8 {
        return self.vtable.getDebugName(self);
    }

    pub fn getDebugColor(self: *const PhysicsMaterial) Color {
        return self.vtable.getDebugColor(self);
    }

    pub fn saveBinaryState(self: *const PhysicsMaterial, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    fn restoreBinaryState(self: *PhysicsMaterial, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Creates a PhysicsMaterial of the correct type and restores its contents from the binary stream
    /// (StreamUtils::RestoreObject with the comptime type list instead of the Factory)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!PhysicsMaterialResult {
        var result: PhysicsMaterialResult = .empty;

        // Read the hash of the type
        var hash: u32 = 0;
        stream.read(&hash);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Failed to read type hash");
            return result;
        }

        // Get the type and construct it
        const object: *PhysicsMaterial = inline for (RegisterTypes.material_types) |T| {
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

    /// Jolt's RTTI::GetHash: FNV-1a of the class name folded to 32 bits
    pub fn rttiHash(name: []const u8) u32 {
        const hash = HashCombine.hashString(name);
        return @truncate(hash ^ (hash >> 32));
    }

    /// Default implementations of the virtual functions
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
            // RTTI hash is read in restoreFromBinaryState
            _ = self;
            _ = stream;
        }
    };
};

/// PhysicsMaterialList (Array<RefConst<PhysicsMaterial>>)
pub const PhysicsMaterialList = std.ArrayList(RefConst(PhysicsMaterial));
