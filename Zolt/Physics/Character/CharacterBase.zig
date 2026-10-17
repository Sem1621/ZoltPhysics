//! Port of: Jolt/Physics/Character/CharacterBase.h, Jolt/Physics/Character/CharacterBase.cpp
//! Status: complete
//!
//! CharacterVehicleArchitecture E2 / E4:
//! - `CharacterBaseSettings` (base of CharacterSettings and CharacterVirtualSettings) is a pattern A root with
//!   `ref_count` and the allocator that frees it. Jolt's only virtual function is the destructor, so its VTable has the
//!   generated `deinit` / `destroy` entries only (`impl` is empty). Jolt's class can be instantiated and so can Zolt's:
//!   `init(CharacterBaseSettings, allocator)` gives settings of this class; a derived class sets
//!   `.base = .init(@This(), allocator)` and lists its (empty) `overrides`. The defaulted copy constructor / assignment
//!   are `initCopy` / `assign`, which take a reference to the shape like Jolt's `RefConst` copy; the destructor
//!   (`destruct`) releases it.
//! - `CharacterBase` (base of Character and CharacterVirtual) is a pattern A root with `ref_count`: `saveState` /
//!   `restoreState` are virtual (CharacterVirtual overrides them), the destructor is generated. Its constructor is
//!   `init(T, allocator, settings, system)` with the most derived type `T`, it takes references to the shape and to the
//!   ground material (`PhysicsMaterial.default`, a static material whose references are not counted).
//!   `restoreState` returns `Allocator.Error`: CharacterVirtual's resizes its list of active contacts (Jolt's
//!   `mActiveContacts.resize`), the only allocation of a restore.
//! - `system: *PhysicsSystem` is Jolt's `mSystem` (E4): the derived classes create, move and query bodies through it.
//! - `EGroundState` is restored from a stream (`RestoreState` reads it as an `int`), so `GroundState` is a
//!   non-exhaustive `enum(i32)`: `toString` gives "Unknown" for other values like Jolt's release build (and asserts like
//!   Jolt's debug build when asserts are enabled).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const RefCount = @import("../../Core/Reference.zig").RefCount;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;

/// Base class for configuration of a character
pub const CharacterBaseSettings = struct {
    /// One entry per C++ virtual function: only the virtual destructor
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *CharacterBaseSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *CharacterBaseSettings) void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<CharacterBaseSettings>)
    ref_count: RefCount = .{},
    /// Frees these settings (heap settings)
    allocator: Allocator,

    /// Vector indicating the up direction of the character
    up: Vec3 = Vec3.axisY(),

    /// Plane, defined in local space relative to the character. Every contact behind this plane can support the
    /// character, every contact in front of this plane is treated as only colliding with the player.
    /// Default: Accept any contact.
    supporting_volume: Plane = Plane.init(Vec3.axisY(), -1.0e10),

    /// Maximum angle of slope that character can still walk on (radians).
    max_slope_angle: f32 = math.degreesToRadians(50.0),

    /// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
    enhanced_internal_edge_removal: bool = false,

    /// Initial shape that represents the character's volume.
    /// Usually this is a capsule, make sure the shape is made so that the bottom of the shape is at (0, 0, 0).
    shape: RefConst(Shape) = .empty,

    /// Constructor, called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) CharacterBaseSettings {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// The defaulted copy constructor: copies the settings of `other` (taking a reference to its shape), not its
    /// reference count. `vtable` is the vtable of the most derived class of the copy.
    pub fn initCopy(vtable: *const VTable, allocator: Allocator, other: *const CharacterBaseSettings) CharacterBaseSettings {
        return .{
            .vtable = vtable,
            .allocator = allocator,
            .up = other.up,
            .supporting_volume = other.supporting_volume,
            .max_slope_angle = other.max_slope_angle,
            .enhanced_internal_edge_removal = other.enhanced_internal_edge_removal,
            .shape = other.shape.clone(),
        };
    }

    /// The defaulted copy assignment (operator =): copies the settings of `other` (the shape reference too), keeps the
    /// vtable, the reference count and the allocator
    pub fn assign(self: *CharacterBaseSettings, other: *const CharacterBaseSettings) void {
        self.up = other.up;
        self.supporting_volume = other.supporting_volume;
        self.max_slope_angle = other.max_slope_angle;
        self.enhanced_internal_edge_removal = other.enhanced_internal_edge_removal;
        self.shape.set(other.shape.get());
    }

    /// The vtable of settings class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<CharacterBaseSettings>

    /// Add a reference to this object
    pub fn addRef(self: *const CharacterBaseSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const CharacterBaseSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark these settings as embedded (on the stack or a member), the last release will not destroy them
    pub fn setEmbedded(self: *const CharacterBaseSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const CharacterBaseSettings) u32 {
        return self.ref_count.get();
    }

    /// Destructor of settings that are not on the heap (embedded / stack)
    pub fn deinit(self: *CharacterBaseSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// The destructor of this level (generated chain): releases the shape
    pub fn destruct(self: *CharacterBaseSettings) void {
        self.shape.deinit();
    }

    /// Jolt's class has no virtual function besides the destructor
    pub const impl = struct {};
};

/// Base class for character class
pub const CharacterBase = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *CharacterBase) void,
        /// delete this (generated)
        destroy: *const fn (self: *CharacterBase) void,

        /// Saving state for replay
        saveState: *const fn (self: *const CharacterBase, stream: *StateRecorder) void,

        /// Restoring state for replay (CharacterVirtual's resizes its contact list, so it can fail to allocate)
        restoreState: *const fn (self: *CharacterBase, stream: *StateRecorder) Allocator.Error!void,
    };

    pub const GroundState = enum(i32) {
        /// Character is on the ground and can move freely.
        on_ground = 0,
        /// Character is on a slope that is too steep and can't climb up any further. The caller should start applying downward velocity if sliding from the slope is desired.
        on_steep_ground = 1,
        /// Character is touching an object, but is not supported by it and should fall. The GetGroundXXX functions will return information about the touched object.
        not_supported = 2,
        /// Character is in the air and is not touching anything.
        in_air = 3,
        /// Other values can come from a restored state (Jolt's `enum class` accepts every int)
        _,
    };

    /// Beyond this value there is no max slope (cNoMaxSlopeAngle)
    pub const no_max_slope_angle: f32 = 0.9999;

    vtable: *const VTable,
    /// Reference count (RefTarget<CharacterBase>)
    ref_count: RefCount = .{},
    /// Frees this character (heap characters)
    allocator: Allocator,

    // Cached physics system
    system: *PhysicsSystem,

    // The shape that the body currently has
    shape: RefConst(Shape),

    // The character's world space up axis
    up: Vec3,

    // Every contact behind this plane can support the character
    supporting_volume: Plane,

    // Cosine of the maximum angle of slope that character can still walk on
    cos_max_slope_angle: f32 = undefined,

    // Ground properties
    ground_state: GroundState = .in_air,
    ground_body_id: BodyID = .invalid,
    ground_body_sub_shape_id: SubShapeID = .empty,
    ground_position: RVec3 = RVec3.zero(),
    ground_normal: Vec3 = Vec3.zero(),
    ground_velocity: Vec3 = Vec3.zero(),
    ground_material: RefConst(PhysicsMaterial),
    ground_user_data: u64 = 0,

    /// Constructor, called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator, settings: *const CharacterBaseSettings, system: *PhysicsSystem) CharacterBase {
        var result: CharacterBase = .{
            .vtable = vtableFor(T),
            .allocator = allocator,
            .system = system,
            .shape = settings.shape.clone(),
            .up = settings.up,
            .supporting_volume = settings.supporting_volume,
            .ground_material = RefConst(PhysicsMaterial).init(PhysicsMaterial.default),
        };

        // Initialize max slope angle
        result.setMaxSlopeAngle(settings.max_slope_angle);
        return result;
    }

    /// The vtable of character class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<CharacterBase>

    /// Add a reference to this object
    pub fn addRef(self: *const CharacterBase) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const CharacterBase) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this character as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const CharacterBase) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const CharacterBase) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a character that is not on the heap (embedded / stack)
    pub fn deinit(self: *CharacterBase) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// The destructor of this level (generated chain): releases the shape and the ground material
    pub fn destruct(self: *CharacterBase) void {
        self.shape.deinit();
        self.ground_material.deinit();
    }

    /// Set the maximum angle of slope that character can still walk on (radians)
    pub fn setMaxSlopeAngle(self: *CharacterBase, max_slope_angle: f32) void {
        self.cos_max_slope_angle = trigonometry.cos(max_slope_angle);
    }

    pub fn getCosMaxSlopeAngle(self: *const CharacterBase) f32 {
        return self.cos_max_slope_angle;
    }

    /// Set the up vector for the character
    pub fn setUp(self: *CharacterBase, up: Vec3) void {
        self.up = up;
    }

    pub fn getUp(self: *const CharacterBase) Vec3 {
        return self.up;
    }

    // Every contact behind this plane can support the character
    pub fn getSupportingVolume(self: *const CharacterBase) Plane {
        return self.supporting_volume;
    }

    pub fn setSupportingVolume(self: *CharacterBase, plane: Plane) void {
        self.supporting_volume = plane;
    }

    /// Check if the normal of the ground surface is too steep to walk on
    pub fn isSlopeTooSteep(self: *const CharacterBase, normal: Vec3) bool {
        // If cos max slope angle is close to one the system is turned off,
        // otherwise check the angle between the up and normal vector
        return self.cos_max_slope_angle < no_max_slope_angle and normal.dot(self.up) < self.cos_max_slope_angle;
    }

    /// Get the current shape that the character is using.
    pub fn getShape(self: *const CharacterBase) ?*const Shape {
        return self.shape.get();
    }

    /// Debug function to convert enum values to string
    pub fn toString(state: GroundState) []const u8 {
        switch (state) {
            .on_ground => return "OnGround",
            .on_steep_ground => return "OnSteepGround",
            .not_supported => return "NotSupported",
            .in_air => return "InAir",
            _ => {},
        }

        if (Core.enable_asserts) @panic("CharacterBase.toString: unknown ground state"); // JPH_ASSERT(false), a restored state can hold any value
        return "Unknown";
    }

    ///@name Properties of the ground this character is standing on
    /// Current ground state
    pub fn getGroundState(self: *const CharacterBase) GroundState {
        return self.ground_state;
    }

    /// Returns true if the player is supported by normal or steep ground
    pub fn isSupported(self: *const CharacterBase) bool {
        return self.ground_state == .on_ground or self.ground_state == .on_steep_ground;
    }

    /// Get the contact point with the ground
    pub fn getGroundPosition(self: *const CharacterBase) RVec3 {
        return self.ground_position;
    }

    /// Get the contact normal with the ground
    pub fn getGroundNormal(self: *const CharacterBase) Vec3 {
        return self.ground_normal;
    }

    /// Velocity in world space of ground
    pub fn getGroundVelocity(self: *const CharacterBase) Vec3 {
        return self.ground_velocity;
    }

    /// Material that the character is standing on
    pub fn getGroundMaterial(self: *const CharacterBase) *const PhysicsMaterial {
        return self.ground_material.get().?;
    }

    /// BodyID of the object the character is standing on. Note may have been removed!
    pub fn getGroundBodyID(self: *const CharacterBase) BodyID {
        return self.ground_body_id;
    }

    /// Sub part of the body that we're standing on.
    pub fn getGroundSubShapeID(self: *const CharacterBase) SubShapeID {
        return self.ground_body_sub_shape_id;
    }

    /// User data value of the body that we're standing on
    pub fn getGroundUserData(self: *const CharacterBase) u64 {
        return self.ground_user_data;
    }

    // Saving / restoring state for replay

    /// Saving state for replay
    pub fn saveState(self: *const CharacterBase, stream: *StateRecorder) void {
        self.vtable.saveState(self, stream);
    }

    /// Restoring state for replay
    pub fn restoreState(self: *CharacterBase, stream: *StateRecorder) Allocator.Error!void {
        return self.vtable.restoreState(self, stream);
    }

    /// The bodies of the virtual functions of CharacterBase
    pub const impl = struct {
        pub fn saveState(self: *const CharacterBase, stream: *StateRecorder) void {
            stream.write(self.ground_state);
            stream.write(self.ground_body_id);
            stream.write(self.ground_body_sub_shape_id);
            stream.write(self.ground_position);
            stream.write(self.ground_normal);
            stream.write(self.ground_velocity);
            // Can't save user data (may be a pointer) and material
        }

        pub fn restoreState(self: *CharacterBase, stream: *StateRecorder) Allocator.Error!void {
            stream.read(&self.ground_state);
            stream.read(&self.ground_body_id);
            stream.read(&self.ground_body_sub_shape_id);
            stream.read(&self.ground_position);
            stream.read(&self.ground_normal);
            stream.read(&self.ground_velocity);
            self.ground_user_data = 0; // Cannot restore user data
            self.ground_material.set(PhysicsMaterial.default); // Cannot restore material
        }
    };
};

// -------------------------------------------------------------------------------------------------------------------
// Tests: a minimal concrete character (Character and CharacterVirtual are the real derived classes)

const testing = std.testing;
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const SphereShape = @import("../Collision/Shape/SphereShape.zig").SphereShape;
const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
const Color = @import("../../Core/Color.zig").Color;
const Real = @import("../../Math/Real.zig").Real;

/// Settings of TestCharacter (the role of CharacterSettings)
const TestCharacterSettings = struct {
    pub const overrides = .{};

    base: CharacterBaseSettings,
    extra: u32 = 7,

    fn create(allocator: Allocator) Allocator.Error!*TestCharacterSettings {
        const self = try allocator.create(TestCharacterSettings);
        self.* = .{ .base = .init(TestCharacterSettings, allocator) };
        return self;
    }

    fn initCopy(allocator: Allocator, other: *const TestCharacterSettings) TestCharacterSettings {
        return .{ .base = .initCopy(CharacterBaseSettings.vtableFor(TestCharacterSettings), allocator, &other.base), .extra = other.extra };
    }
};

/// A character that overrides SaveState / RestoreState like CharacterVirtual (base class state first, then its own)
const TestCharacter = struct {
    pub const overrides = .{ .saveState, .restoreState };

    base: CharacterBase,
    position: RVec3 = RVec3.zero(),
    destructed: ?*u32 = null,

    fn create(allocator: Allocator, settings: *const TestCharacterSettings, system: *PhysicsSystem) Allocator.Error!*TestCharacter {
        const self = try allocator.create(TestCharacter);
        self.* = .{ .base = .init(TestCharacter, allocator, &settings.base, system) };
        return self;
    }

    pub fn destruct(self: *TestCharacter) void {
        if (self.destructed) |d| d.* += 1;
    }

    pub fn saveState(self: *const TestCharacter, stream: *StateRecorder) void {
        CharacterBase.impl.saveState(&self.base, stream);
        stream.write(self.position);
    }

    pub fn restoreState(self: *TestCharacter, stream: *StateRecorder) Allocator.Error!void {
        try CharacterBase.impl.restoreState(&self.base, stream);
        stream.read(&self.position);
    }
};

test "CharacterBaseSettings: Jolt's defaults, copy, references to the shape" {
    const allocator = testing.allocator;

    var settings = TestCharacterSettings{ .base = .init(TestCharacterSettings, allocator) };
    defer settings.base.deinit();
    try testing.expect(settings.base.up.eql(Vec3.init(0, 1, 0)));
    try testing.expect(settings.base.supporting_volume.getNormal().eql(Vec3.axisY()));
    try testing.expectEqual(@as(f32, -1.0e10), settings.base.supporting_volume.getConstant());
    try testing.expectEqual(@as(f32, 50.0) * (math.pi / 180.0), settings.base.max_slope_angle);
    try testing.expect(!settings.base.enhanced_internal_edge_removal);
    try testing.expect(settings.base.shape.get() == null);

    // The copy takes a reference to the shape (RefConst copy), the destructor releases it
    const sphere = try SphereShape.create(allocator, 0.5, .{});
    const shape = virtual.upcast(Shape, sphere);
    var shape_ref = RefConst(Shape).init(shape);
    defer shape_ref.deinit();
    settings.base.shape.set(shape);
    settings.base.max_slope_angle = 0.25;
    settings.base.enhanced_internal_edge_removal = true;
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
    {
        var copy = TestCharacterSettings.initCopy(allocator, &settings);
        defer copy.base.deinit();
        try testing.expectEqual(@as(u32, 3), shape.getRefCount());
        try testing.expectEqual(@as(f32, 0.25), copy.base.max_slope_angle);
        try testing.expect(copy.base.enhanced_internal_edge_removal);
        try testing.expectEqual(@as(u32, 0), copy.base.getRefCount());

        // Copy assignment
        var other = TestCharacterSettings{ .base = .init(TestCharacterSettings, allocator) };
        defer other.base.deinit();
        other.base.assign(&copy.base);
        try testing.expectEqual(@as(u32, 4), shape.getRefCount());
        try testing.expectEqual(@as(f32, 0.25), other.base.max_slope_angle);
    }
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());

    // Heap settings are destroyed through the vtable by the last release
    const heap = try TestCharacterSettings.create(allocator);
    heap.base.shape.set(shape);
    heap.base.addRef();
    try testing.expectEqual(@as(u32, 1), heap.base.getRefCount());
    try testing.expectEqual(@as(u32, 3), shape.getRefCount());
    heap.base.release();
    try testing.expectEqual(@as(u32, 2), shape.getRefCount());
}

test "CharacterBaseSettings: settings of the class itself" {
    // Jolt's CharacterBaseSettings has no pure virtual function: it can be instantiated, on the stack and on the heap
    const allocator = testing.allocator;
    var settings: CharacterBaseSettings = .init(CharacterBaseSettings, allocator);
    defer settings.deinit();
    try testing.expectEqual(@as(f32, 50.0) * (math.pi / 180.0), settings.max_slope_angle);

    const heap = try allocator.create(CharacterBaseSettings);
    heap.* = .init(CharacterBaseSettings, allocator);
    heap.assign(&settings);
    heap.addRef();
    try testing.expectEqual(@as(u32, 1), heap.getRefCount());
    heap.release(); // Destroys it through the vtable
}

test "CharacterBase: construction, slope check, up vector, supporting volume, ground getters" {
    const allocator = testing.allocator;
    var system: PhysicsSystem = undefined; // Only stored by CharacterBase

    const sphere = try SphereShape.create(allocator, 0.5, .{});
    const shape = virtual.upcast(Shape, sphere);
    var shape_ref = RefConst(Shape).init(shape);
    defer shape_ref.deinit();

    var settings = TestCharacterSettings{ .base = .init(TestCharacterSettings, allocator) };
    defer settings.base.deinit();
    settings.base.shape.set(shape);
    settings.base.up = Vec3.init(0, 0, 1);
    settings.base.supporting_volume = Plane.init(Vec3.init(0, 0, 1), -0.3);
    settings.base.max_slope_angle = math.degreesToRadians(45.0);

    var character = TestCharacter{ .base = .init(TestCharacter, allocator, &settings.base, &system) };
    defer character.base.deinit();
    const base = &character.base;
    try testing.expect(base.system == &system);
    try testing.expectEqual(@as(u32, 3), shape.getRefCount()); // shape_ref, settings, character
    try testing.expect(base.getShape().? == shape);
    try testing.expect(base.getUp().eql(Vec3.init(0, 0, 1)));
    try testing.expect(base.getSupportingVolume().normal_and_constant.eql(settings.base.supporting_volume.normal_and_constant));
    try testing.expectEqual(trigonometry.cos(math.degreesToRadians(45.0)), base.getCosMaxSlopeAngle());

    // Ground state of a new character
    try testing.expectEqual(CharacterBase.GroundState.in_air, base.getGroundState());
    try testing.expect(!base.isSupported());
    try testing.expect(base.getGroundBodyID().isInvalid());
    try testing.expect(base.getGroundSubShapeID().eql(SubShapeID.empty));
    try testing.expect(base.getGroundPosition().eql(RVec3.zero()));
    try testing.expect(base.getGroundNormal().eql(Vec3.zero()));
    try testing.expect(base.getGroundVelocity().eql(Vec3.zero()));
    try testing.expect(base.getGroundMaterial() == PhysicsMaterial.default);
    try testing.expectEqual(@as(u64, 0), base.getGroundUserData());

    // IsSlopeTooSteep: the angle between up (+Z) and the normal against 45 degrees
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(0, 0, 1)));
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(0, 0.5, 0.8660254)));
    try testing.expect(base.isSlopeTooSteep(Vec3.init(0, 0.8660254, 0.5)));
    try testing.expect(base.isSlopeTooSteep(Vec3.init(1, 0, 0)));
    try testing.expect(base.isSlopeTooSteep(Vec3.init(0, 0, -1)));

    // A max slope angle that is (nearly) zero turns the check off
    base.setMaxSlopeAngle(0.0);
    try testing.expectEqual(@as(f32, 1.0), base.getCosMaxSlopeAngle());
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(1, 0, 0)));
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(0, 0, -1)));
    base.setMaxSlopeAngle(math.degreesToRadians(0.5)); // cos > 0.9999
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(1, 0, 0)));
    base.setMaxSlopeAngle(math.degreesToRadians(1.0)); // cos < 0.9999
    try testing.expect(base.isSlopeTooSteep(Vec3.init(1, 0, 0)));

    // Up vector and supporting volume
    base.setUp(Vec3.init(0, 1, 0));
    try testing.expect(base.getUp().eql(Vec3.axisY()));
    base.setMaxSlopeAngle(math.degreesToRadians(30.0));
    try testing.expect(base.isSlopeTooSteep(Vec3.init(0, 0, 1)));
    try testing.expect(!base.isSlopeTooSteep(Vec3.init(0, 1, 0)));
    base.setSupportingVolume(Plane.init(Vec3.init(0, 1, 0), -0.5));
    try testing.expectEqual(@as(f32, -0.5), base.getSupportingVolume().getConstant());

    // IsSupported for every ground state
    for ([_]CharacterBase.GroundState{ .on_ground, .on_steep_ground, .not_supported, .in_air, @enumFromInt(17) }, [_]bool{ true, true, false, false, false }) |state, supported| {
        base.ground_state = state;
        try testing.expectEqual(supported, base.isSupported());
    }
}

test "CharacterBase: SaveState / RestoreState round trip, user data and material are reset" {
    const allocator = testing.allocator;
    var system: PhysicsSystem = undefined;
    var settings = TestCharacterSettings{ .base = .init(TestCharacterSettings, allocator) };
    defer settings.base.deinit();
    var character = TestCharacter{ .base = .init(TestCharacter, allocator, &settings.base, &system) };
    defer character.base.deinit();
    const base = &character.base;

    var material = try PhysicsMaterialSimple.init(allocator, "Ice", Color.white);
    material.base.setEmbedded();
    defer material.base.deinit();

    base.ground_state = .on_steep_ground;
    base.ground_body_id = BodyID.fromIndexAndSequenceNumber(12, 3);
    base.ground_body_sub_shape_id = .{ .value = 0x1234 };
    base.ground_position = RVec3.init(1.5, -2.25, 1.0e3);
    base.ground_normal = Vec3.init(0, 0.6, 0.8);
    base.ground_velocity = Vec3.init(-1, 2, -3);
    base.ground_user_data = 0xdeadbeef;
    base.ground_material.set(&material.base);
    character.position = RVec3.init(3, 4, 5);

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    character.base.saveState(recorder.recorder());
    try testing.expect(!recorder.isFailed());

    // Jolt's layout: ground state (int), body ID, sub shape ID, RVec3, Vec3, Vec3, then the derived class' state
    const real_size = 3 * @sizeOf(Real);
    try testing.expectEqual(@as(usize, 4 + 4 + 4 + real_size + 12 + 12 + real_size), recorder.getData().len);
    const data = recorder.getData();
    try testing.expectEqual(@as(i32, 1), std.mem.bytesToValue(i32, data[0..4]));
    try testing.expectEqual(BodyID.fromIndexAndSequenceNumber(12, 3).getIndexAndSequenceNumber(), std.mem.bytesToValue(u32, data[4..8]));
    try testing.expectEqual(@as(u32, 0x1234), std.mem.bytesToValue(u32, data[8..12]));

    // Change everything, then restore
    base.ground_state = .in_air;
    base.ground_body_id = .invalid;
    base.ground_body_sub_shape_id = .empty;
    base.ground_position = RVec3.zero();
    base.ground_normal = Vec3.zero();
    base.ground_velocity = Vec3.zero();
    character.position = RVec3.zero();
    recorder.rewind();
    try character.base.restoreState(recorder.recorder());
    try testing.expect(!recorder.isFailed() and !recorder.isEOF());
    try testing.expectEqual(CharacterBase.GroundState.on_steep_ground, base.getGroundState());
    try testing.expect(base.getGroundBodyID().eql(BodyID.fromIndexAndSequenceNumber(12, 3)));
    try testing.expectEqual(@as(u32, 0x1234), base.getGroundSubShapeID().getValue());
    try testing.expect(base.getGroundPosition().eql(RVec3.init(1.5, -2.25, 1.0e3)));
    try testing.expect(base.getGroundNormal().eql(Vec3.init(0, 0.6, 0.8)));
    try testing.expect(base.getGroundVelocity().eql(Vec3.init(-1, 2, -3)));
    try testing.expect(character.position.eql(RVec3.init(3, 4, 5)));

    // User data and material cannot be restored
    try testing.expectEqual(@as(u64, 0), base.getGroundUserData());
    try testing.expect(base.getGroundMaterial() == PhysicsMaterial.default);

    // A state with a ground state Jolt does not know restores (the enum accepts every int)
    var odd = StateRecorderImpl.init(allocator);
    defer odd.deinit();
    odd.writeBytes(data);
    std.mem.writeInt(i32, odd.data.items[0..4], 9, .little);
    odd.rewind();
    try character.base.restoreState(odd.recorder());
    try testing.expectEqual(@as(i32, 9), @intFromEnum(base.getGroundState()));
    try testing.expect(!base.isSupported());
}

test "CharacterBase: toString" {
    try testing.expectEqualStrings("OnGround", CharacterBase.toString(.on_ground));
    try testing.expectEqualStrings("OnSteepGround", CharacterBase.toString(.on_steep_ground));
    try testing.expectEqualStrings("NotSupported", CharacterBase.toString(.not_supported));
    try testing.expectEqualStrings("InAir", CharacterBase.toString(.in_air));
    if (!Core.enable_asserts) // Jolt's debug build asserts, the release build returns "Unknown"
        try testing.expectEqualStrings("Unknown", CharacterBase.toString(@enumFromInt(4)));
}

test "CharacterBase: heap characters, references, destructor chain, allocation failure" {
    const Test = struct {
        fn run(allocator: Allocator) !void {
            var system: PhysicsSystem = undefined;
            const sphere = try SphereShape.create(allocator, 0.5, .{});
            const shape = virtual.upcast(Shape, sphere);
            var shape_ref = RefConst(Shape).init(shape);
            defer shape_ref.deinit();

            const settings = try TestCharacterSettings.create(allocator);
            var settings_ref = RefConst(CharacterBaseSettings).init(&settings.base);
            defer settings_ref.deinit();
            settings.base.shape.set(shape);

            var destructed: u32 = 0;
            const character = try TestCharacter.create(allocator, settings, &system);
            character.destructed = &destructed;
            var ref = RefConst(CharacterBase).init(&character.base);
            try testing.expectEqual(@as(u32, 3), shape.getRefCount());
            var second = ref.clone();
            try testing.expectEqual(@as(u32, 2), character.base.getRefCount());
            second.deinit();
            ref.deinit(); // Destroys the character through the vtable: TestCharacter.destruct, then CharacterBase.destruct
            try testing.expectEqual(@as(u32, 1), destructed);
            try testing.expectEqual(@as(u32, 2), shape.getRefCount());
        }
    };
    try Test.run(testing.allocator);
    try testing.checkAllAllocationFailures(testing.allocator, Test.run, .{});
}
