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
//!
//! The class is not final: a material that derives from it (Jolt's FrictionPerTriangleTest::MyMaterial) builds its
//! base with `initDerived(@This(), allocator, name, color)` / `initDefaultDerived(@This(), allocator)`, so the vtable
//! (overrides, destructor chain and the size that `destroy` frees) is the one of the most derived class:
//!
//! ```zig
//! const MyMaterial = struct {
//!     pub const overrides = .{};
//!     pub const rtti_name = PhysicsMaterialSimple.rtti_name; // No JPH_RTTI of its own
//!     base: PhysicsMaterialSimple,
//!     friction: f32,
//!     restitution: f32,
//! };
//! const m = try allocator.create(MyMaterial);
//! m.* = .{ .base = try .initDerived(MyMaterial, allocator, "Slippery", Color.red), .friction = 0.1, .restitution = 0 };
//! ```

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
        return initDefaultDerived(PhysicsMaterialSimple, allocator);
    }

    /// Constructor (PhysicsMaterialSimple()) for classes that derive from this material (T is the most derived class)
    pub fn initDefaultDerived(comptime T: type, allocator: Allocator) PhysicsMaterialSimple {
        return .{ .base = .init(T, allocator) };
    }

    /// new PhysicsMaterialSimple() (used by PhysicsMaterial.restoreFromBinaryState)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*PhysicsMaterialSimple {
        const self = try allocator.create(PhysicsMaterialSimple);
        self.* = .initDefault(allocator);
        return self;
    }

    /// Constructor (PhysicsMaterialSimple(inName, inColor)), copies the name
    pub fn init(allocator: Allocator, name: []const u8, color: Color) Allocator.Error!PhysicsMaterialSimple {
        return initDerived(PhysicsMaterialSimple, allocator, name, color);
    }

    /// Constructor (PhysicsMaterialSimple(inName, inColor)) for classes that derive from this material (T is the most
    /// derived class), copies the name
    pub fn initDerived(comptime T: type, allocator: Allocator, name: []const u8, color: Color) Allocator.Error!PhysicsMaterialSimple {
        return .{ .base = .init(T, allocator), .debug_name = try allocator.dupe(u8, name), .debug_color = color };
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
        // Jolt returns mDebugName.c_str(): a name with an embedded 0 byte ends there
        return std.mem.sliceTo(self.debug_name, 0);
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
const virtual = @import("../../Core/Virtual.zig");
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

test "PhysicsMaterialSimple: name with an embedded 0 byte" {
    const allocator = std.testing.allocator;

    // getDebugName is Jolt's mDebugName.c_str(): it ends at the first 0 byte, the binary state has the whole name
    const material = try PhysicsMaterialSimple.create(allocator, "Ice\x00cold", Color.cyan);
    var ref = RefConst(PhysicsMaterial).init(material.material());
    defer ref.deinit();
    try std.testing.expectEqualStrings("Ice", material.material().getDebugName());
    try std.testing.expectEqual(@as(usize, 8), material.debug_name.len);

    var buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    material.material().saveBinaryState(out.streamOut());
    try std.testing.expectEqual(@as(usize, 4 + 4 + 8 + 4), writer.end);
    try std.testing.expectEqualSlices(u8, "Ice\x00cold", writer.buffered()[8..16]);
}

test "PhysicsMaterialSimple: derived material" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // Jolt's FrictionPerTriangleTest::MyMaterial: adds data, overrides nothing and has no JPH_RTTI of its own
    const MyMaterial = struct {
        pub const overrides = .{};
        pub const rtti_name = PhysicsMaterialSimple.rtti_name;

        base: PhysicsMaterialSimple,
        friction: f32,
        restitution: f32,

        fn create(a: Allocator, name: []const u8, color: Color, friction: f32, restitution: f32) Allocator.Error!*@This() {
            const self = try a.create(@This());
            errdefer a.destroy(self);
            self.* = .{ .base = try .initDerived(@This(), a, name, color), .friction = friction, .restitution = restitution };
            return self;
        }
    };

    const name = try allocator.dupe(u8, "Slippery");
    const slippery = try MyMaterial.create(allocator, name, Color.red, 0.1, 0.25);
    allocator.free(name);
    var slippery_ref = RefConst(PhysicsMaterial).init(&slippery.base.base);
    defer slippery_ref.deinit(); // Destroys a MyMaterial (the size of MyMaterial is freed, the name too)
    try expect(slippery.base.base.vtable == PhysicsMaterial.vtableFor(MyMaterial));
    try std.testing.expectEqualStrings("Slippery", slippery_ref.get().?.getDebugName());
    try expect(slippery_ref.get().?.getDebugColor().eql(Color.red));
    try std.testing.expectEqual(PhysicsMaterial.rttiHash("PhysicsMaterialSimple"), slippery_ref.get().?.getRTTIHash());
    const my_material: *const MyMaterial = virtual.downcast(MyMaterial, slippery_ref.get().?);
    try std.testing.expectEqual(@as(f32, 0.1), my_material.friction);
    try std.testing.expectEqual(@as(f32, 0.25), my_material.restitution);

    // A derived class with overrides and a destructor of its own: both destructors run, derived first
    const Tinted = struct {
        pub const overrides = .{.getDebugColor};
        pub const rtti_name = "Tinted";

        base: PhysicsMaterialSimple,
        tag: []u8,
        destructed: *u32,

        pub fn getDebugColor(self: *const @This()) Color {
            _ = self;
            return Color.green;
        }

        pub fn destruct(self: *@This()) void {
            self.base.base.allocator.free(self.tag);
            self.destructed.* += 1;
        }
    };

    var destructed: u32 = 0;
    const tinted = try allocator.create(Tinted);
    tinted.* = .{ .base = try .initDerived(Tinted, allocator, "Tinted", Color.red), .tag = try allocator.dupe(u8, "tag"), .destructed = &destructed };
    var tinted_ref = RefConst(PhysicsMaterial).init(&tinted.base.base);
    try std.testing.expectEqualStrings("Tinted", tinted_ref.get().?.getDebugName());
    try expect(tinted_ref.get().?.getDebugColor().eql(Color.green));
    try std.testing.expectEqual(PhysicsMaterial.rttiHash("Tinted"), tinted_ref.get().?.getRTTIHash());
    tinted_ref.deinit();
    try std.testing.expectEqual(@as(u32, 1), destructed);

    // The default constructor of a derived material, on the stack
    var stack: Tinted = .{ .base = .initDefaultDerived(Tinted, allocator), .tag = try allocator.dupe(u8, "stack"), .destructed = &destructed };
    stack.base.base.setEmbedded();
    try std.testing.expectEqualStrings("", stack.base.base.getDebugName());
    try expect(stack.base.base.getDebugColor().eql(Color.green));
    stack.base.base.deinit();
    try std.testing.expectEqual(@as(u32, 2), destructed);
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
