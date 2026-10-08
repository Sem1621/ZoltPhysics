//! Port of: Jolt/Physics/Collision/PhysicsMaterialSimple.h, Jolt/Physics/Collision/PhysicsMaterialSimple.cpp
//! Status: complete
//!
//! The debug name is an owned copy (Jolt's `String mDebugName`), allocated with the material's allocator, so the
//! caller's string can be freed after construction. Static materials (`initStatic`, compile time constants such as
//! `default_material`) borrow a string literal and are never destroyed.
//!
//! Constructors: `PhysicsMaterialSimple()` is `initDefault(allocator)` / `createDefault(allocator)`,
//! `PhysicsMaterialSimple(inName, inColor)` is `init(allocator, name, color)` (on the stack or as a member:
//! `base.setEmbedded()` before taking references, `base.deinit()` at the end) / `create(allocator, name, color)`
//! (`new`, reference count 0).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../Core/Color.zig").Color;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const PhysicsMaterial = @import("PhysicsMaterial.zig").PhysicsMaterial;

/// Sample implementation of PhysicsMaterial that just holds the needed properties directly
pub const PhysicsMaterialSimple = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, PhysicsMaterialSimple)

    pub const overrides = .{ .getDebugName, .getDebugColor, .saveBinaryState, .restoreBinaryState };

    /// Name of the class for Jolt's RTTI hash (JPH_RTTI)
    pub const rtti_name = "PhysicsMaterialSimple";

    base: PhysicsMaterial,
    /// Name of the material, used for debugging purposes (owned by base.allocator unless the material is static)
    debug_name: []const u8 = "",
    /// Color of the material, used to render the shapes
    debug_color: Color = Color.grey,

    /// The default material, what RegisterTypes stores in PhysicsMaterial::sDefault
    /// (`new PhysicsMaterialSimple("Default", Color::sGrey)`), a compile time constant
    pub const default_material: PhysicsMaterialSimple = .initStatic("Default", Color.grey);

    /// Constructor (PhysicsMaterialSimple())
    pub fn initDefault(allocator: Allocator) PhysicsMaterialSimple {
        return .{ .base = .init(PhysicsMaterialSimple, allocator) };
    }

    /// new PhysicsMaterialSimple() (used by PhysicsMaterial.restoreFromBinaryState)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*PhysicsMaterialSimple {
        const self = try allocator.create(PhysicsMaterialSimple);
        self.* = .initDefault(allocator);
        return self;
    }

    /// Constructor (PhysicsMaterialSimple(inName, inColor)), copies the name
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

    /// Implicit upcast to the base class
    pub fn material(self: *const PhysicsMaterialSimple) *const PhysicsMaterial {
        return &self.base;
    }

    /// Implicit upcast to the base class (mutable)
    pub fn materialMut(self: *PhysicsMaterialSimple) *PhysicsMaterial {
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

const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

test "PhysicsMaterialSimple: owned name, binary state" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // A heap material owns a copy of its name
    const name = try allocator.dupe(u8, "Ice");
    const ice = try PhysicsMaterialSimple.create(allocator, name, Color.cyan);
    allocator.free(name);
    var ice_ref = RefConst(PhysicsMaterial).init(ice.material());
    defer ice_ref.deinit();
    try std.testing.expectEqualStrings("Ice", ice.material().getDebugName());
    try expect(ice.material().getDebugColor().eql(Color.cyan));
    try std.testing.expectEqual(PhysicsMaterial.rttiHash("PhysicsMaterialSimple"), ice.material().getRTTIHash());

    // Binary state: Jolt's RTTI hash, then the name and the color
    var buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    ice.material().saveBinaryState(out.streamOut());
    try std.testing.expectEqual(@as(usize, 4 + 4 + 3 + 4), writer.end);

    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamInWrapper.init(&reader);
    var restored = try PhysicsMaterial.restoreFromBinaryState(allocator, in.streamIn());
    defer restored.deinit();
    try std.testing.expectEqualStrings("Ice", restored.getPtr().?.getDebugName());
    try expect(restored.getPtr().?.getDebugColor().eql(Color.cyan));
    try std.testing.expectEqual(ice.material().getRTTIHash(), restored.getPtr().?.getRTTIHash());

    // Restoring into an existing material replaces its name
    var stack = try PhysicsMaterialSimple.init(allocator, "Wood", Color.orange);
    stack.base.setEmbedded();
    defer stack.base.deinit();
    var reader2: std.Io.Reader = .fixed(writer.buffered()[4..]); // Skip the RTTI hash like restoreFromBinaryState
    var in2 = StreamInWrapper.init(&reader2);
    try stack.materialMut().restoreBinaryState(in2.streamIn());
    try std.testing.expectEqualStrings("Ice", stack.getDebugName());
    try expect(stack.getDebugColor().eql(Color.cyan));

    // A default constructed material
    var default = PhysicsMaterialSimple.initDefault(allocator);
    defer default.base.deinit();
    try std.testing.expectEqualStrings("", default.getDebugName());
    try expect(default.getDebugColor().eql(Color.grey));
    try std.testing.expect(PhysicsMaterialSimple.default_material.base.is_static);
}

test "PhysicsMaterialSimple: out of memory" {
    const allocator = std.testing.allocator;

    // create: every allocation failure is returned and nothing leaks
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const material = PhysicsMaterialSimple.create(failing.allocator(), "Rubber", Color.red) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        var ref = RefConst(PhysicsMaterial).init(material.material());
        ref.deinit();
        try std.testing.expectEqual(@as(usize, 2), fail_index); // The object and its name
        break;
    }

    // restoreFromBinaryState: the same
    const rubber = try PhysicsMaterialSimple.create(allocator, "Rubber", Color.red);
    var rubber_ref = RefConst(PhysicsMaterial).init(rubber.material());
    defer rubber_ref.deinit();
    var buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    rubber.material().saveBinaryState(out.streamOut());
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var reader: std.Io.Reader = .fixed(writer.buffered());
        var in = StreamInWrapper.init(&reader);
        var result = PhysicsMaterial.restoreFromBinaryState(failing.allocator(), in.streamIn()) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer result.deinit();
        try std.testing.expectEqualStrings("Rubber", result.getPtr().?.getDebugName());
        try std.testing.expectEqual(@as(usize, 2), fail_index); // The object and its name
        break;
    }
}
