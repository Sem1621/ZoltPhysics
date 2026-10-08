//! Port of: Jolt/Physics/Collision/PhysicsMaterialSimple.h, Jolt/Physics/Collision/PhysicsMaterialSimple.cpp
//! Status: complete
//! Not ported: JPH_DECLARE_SERIALIZABLE_VIRTUAL (ObjectStream, Phase 8), see TODO(serialization)
//!
//! The debug name is an owned copy (Jolt's `String mDebugName`), allocated with the material's allocator. Static
//! materials (`initStatic`, compile time constants) borrow a string literal and are never destroyed.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const Color = zolt.Color;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const PhysicsMaterial = @import("PhysicsMaterial.zig").PhysicsMaterial;

/// Sample implementation of PhysicsMaterial that just holds the needed properties directly
pub const PhysicsMaterialSimple = struct {
    pub const overrides = .{ .getDebugName, .getDebugColor, .saveBinaryState, .restoreBinaryState };
    pub const rtti_name = "PhysicsMaterialSimple";

    base: PhysicsMaterial,
    /// Name of the material, used for debugging purposes (owned by base.allocator unless the material is static)
    debug_name: []const u8 = "",
    /// Color of the material, used to render the shapes
    debug_color: Color = Color.grey,

    /// The default material (PhysicsMaterial::sDefault = new PhysicsMaterialSimple("Default", Color::sGrey)), a constant
    pub const default_material: PhysicsMaterialSimple = .initStatic("Default", Color.grey);

    /// PhysicsMaterialSimple() (used by PhysicsMaterial.restoreFromBinaryState)
    pub fn initDefault(allocator: Allocator) PhysicsMaterialSimple {
        return .{ .base = .init(PhysicsMaterialSimple, allocator) };
    }

    /// new PhysicsMaterialSimple()
    pub fn createDefault(allocator: Allocator) Allocator.Error!*PhysicsMaterialSimple {
        const self = try allocator.create(PhysicsMaterialSimple);
        self.* = .initDefault(allocator);
        return self;
    }

    /// PhysicsMaterialSimple(inName, inColor) on the stack or as a member (setEmbedded() before taking references,
    /// base.deinit() at the end)
    pub fn init(allocator: Allocator, name: []const u8, color: Color) Allocator.Error!PhysicsMaterialSimple {
        return .{ .base = .init(PhysicsMaterialSimple, allocator), .debug_name = try allocator.dupe(u8, name), .debug_color = color };
    }

    /// new PhysicsMaterialSimple(inName, inColor): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, name: []const u8, color: Color) Allocator.Error!*PhysicsMaterialSimple {
        const self = try allocator.create(PhysicsMaterialSimple);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, name, color);
        return self;
    }

    /// A compile time constant material (in read-only memory, never reference counted)
    pub fn initStatic(comptime name: []const u8, color: Color) PhysicsMaterialSimple {
        return .{ .base = .initStatic(PhysicsMaterialSimple), .debug_name = name, .debug_color = color };
    }

    /// ~PhysicsMaterialSimple
    pub fn destruct(self: *PhysicsMaterialSimple) void {
        std.debug.assert(!self.base.is_static);
        self.base.allocator.free(self.debug_name);
    }

    pub fn material(self: *const PhysicsMaterialSimple) *const PhysicsMaterial {
        return &self.base;
    }

    // Properties
    pub fn getDebugName(self: *const PhysicsMaterialSimple) []const u8 {
        return self.debug_name;
    }

    pub fn getDebugColor(self: *const PhysicsMaterialSimple) Color {
        return self.debug_color;
    }

    // See: PhysicsMaterial::SaveBinaryState
    pub fn saveBinaryState(self: *const PhysicsMaterialSimple, stream: StreamOut) void {
        PhysicsMaterial.impl.saveBinaryState(&self.base, stream);

        stream.writeString(self.debug_name);
        stream.write(self.debug_color);
    }

    // See: PhysicsMaterial::RestoreBinaryState
    pub fn restoreBinaryState(self: *PhysicsMaterialSimple, stream: StreamIn) Allocator.Error!void {
        try PhysicsMaterial.impl.restoreBinaryState(&self.base, stream);

        var name: []u8 = &.{};
        try stream.readString(self.base.allocator, &name);
        self.base.allocator.free(self.debug_name);
        self.debug_name = name;
        stream.read(&self.debug_color);
    }
};
