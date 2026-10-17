//! Port of: PerformanceTest/CharacterVirtualScene.h
//! Status: complete
//!
//! Milestone M8 (CharacterVehicleArchitecture section 4): `zig build performance-test -Doptimize=ReleaseFast --
//! -q=Discrete -t=max -s=CharacterVirtual -validate_hash=0xc1d4e05b164b1221` (0xa0b16727aa16fdb0 with
//! -Ddouble_precision=true) prints Jolt's hash for every thread count.
//! - `new CharacterVirtualScene` is `CharacterVirtualScene.create(allocator)`; the scene is used through the
//!   PerformanceTestScene interface (`scene()`), which owns it. Jolt's class is also the CharacterContactListener of the
//!   characters: the scene declares the callbacks that Jolt's overrides (`CharacterContactListener.init(self)`), so the
//!   scene must not move while characters exist (it is on the heap).
//! - `Array<BodyCreationSettings> mWorld` holds BodyCreationSettings with references to their shapes (settings), the
//!   destructor releases them. `Array<Ref<CharacterVirtual>> mCharacters` holds heap characters with one reference each:
//!   StopTest releases them (which destroys their inner bodies, so it runs before the physics system is destroyed).
//! - `Sin` / `Cos` are Jolt's (Trigonometry), `Hash<uint64>` / `HashCombine` are HashCombine.hash / hashCombine: the
//!   `int` tags of the callbacks are hashed as i32 (Jolt's Hash<int>), the character and body IDs through their
//!   `getHash`, the positions and normals through Vec3 / DVec3.getHash (JPH_MAKE_HASHABLE).
//! - CharacterID::sSetNextCharacterID() is CharacterID.setNextCharacterID: the counter is global like Jolt's (E4), so
//!   every test run numbers its characters 1, 2, ... like Jolt.

const std = @import("std");
const zolt = @import("zolt");
const PerformanceTestScene = @import("PerformanceTestScene.zig").PerformanceTestScene;
const Layers = @import("Layers.zig").Layers;

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const Plane = zolt.Plane;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const BoxShape = zolt.BoxShape;
const CapsuleShape = zolt.CapsuleShape;
const MeshShapeSettings = zolt.MeshShapeSettings;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const SubShapeID = zolt.SubShapeID;
const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;
const TempAllocator = zolt.TempAllocator;
const CharacterID = zolt.CharacterID;
const CharacterVirtual = zolt.CharacterVirtual;
const CharacterVirtualSettings = zolt.CharacterVirtualSettings;
const CharacterContact = zolt.CharacterContact;
const CharacterContactSettings = zolt.CharacterContactSettings;
const CharacterContactListener = zolt.CharacterContactListener;
const CharacterVsCharacterCollisionSimple = zolt.CharacterVsCharacterCollisionSimple;
const HashCombine = zolt.HashCombine;
const sin = zolt.trigonometry.sin;
const cos = zolt.trigonometry.cos;

/// A scene that drops a number of virtual characters on a scene and simulates them
pub const CharacterVirtualScene = struct {
    const num_characters_x = 10;
    const num_characters_y = 10;
    const character_height_standing: f32 = 1.35;
    const character_radius_standing: f32 = 0.3;
    const inner_shape_fraction: f32 = 0.9;
    const stairs_step_height: f32 = 0.3;

    allocator: Allocator,
    time: f32 = 0.0,
    hash: u64 = 0,
    world: std.ArrayList(BodyCreationSettings) = .empty,
    /// Array<Ref<CharacterVirtual>>: one reference per character
    characters: std.ArrayList(*CharacterVirtual) = .empty,
    character_vs_character_collision: CharacterVsCharacterCollisionSimple,

    /// new CharacterVirtualScene
    pub fn create(allocator: Allocator) Allocator.Error!*CharacterVirtualScene {
        const self = try allocator.create(CharacterVirtualScene);
        self.* = .{ .allocator = allocator, .character_vs_character_collision = .init(allocator) };
        return self;
    }

    /// The PerformanceTestScene interface (owns the scene)
    pub fn scene(self: *CharacterVirtualScene) PerformanceTestScene {
        return .init(self);
    }

    pub fn destroy(self: *CharacterVirtualScene) void {
        std.debug.assert(self.characters.items.len == 0); // StopTest released them (their inner bodies are in a system)
        self.characters.deinit(self.allocator);
        self.character_vs_character_collision.deinit();
        for (self.world.items) |*bcs| bcs.deinit();
        self.world.deinit(self.allocator);
        self.allocator.destroy(self);
    }

    pub fn getName(self: *const CharacterVirtualScene) [:0]const u8 {
        _ = self;
        return "CharacterVirtual";
    }

    /// mWorld.push_back(settings) (the list takes over the settings, also when it cannot grow)
    fn addToWorld(self: *CharacterVirtualScene, settings: BodyCreationSettings) Allocator.Error!void {
        var s = settings;
        errdefer s.deinit();
        try self.world.append(self.allocator, s);
    }

    pub fn load(self: *CharacterVirtualScene, asset_path: []const u8) PerformanceTestScene.Error!bool {
        _ = asset_path;
        const allocator = self.allocator;

        const n = 100;
        const cell_size: f32 = 0.5;
        const max_height: f32 = 2.0;
        const center: f32 = @as(f32, n) * cell_size / 2;

        // Create vertices
        const num_vertices = (n + 1) * (n + 1);
        const vertices = try allocator.alloc(Float3, num_vertices);
        defer allocator.free(vertices);
        for (0..n + 1) |x| {
            for (0..n + 1) |z| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                const height = sin(fx * 20.0 / @as(f32, n)) * cos(fz * 20.0 / @as(f32, n));
                vertices[z * (n + 1) + x] = Float3.init(cell_size * fx, max_height * height, cell_size * fz);
            }
        }

        // Create regular grid of triangles
        const num_triangles = n * n * 2;
        const indices = try allocator.alloc(IndexedTriangle, num_triangles);
        defer allocator.free(indices);
        var next: usize = 0;
        for (0..n) |x| {
            for (0..n) |z| {
                const start: u32 = @intCast((n + 1) * z + x);

                indices[next] = .{ .idx = .{ start, start + n + 1, start + 1 } };
                next += 1;

                indices[next] = .{ .idx = .{ start + 1, start + n + 1, start + n + 2 } };
                next += 1;
            }
        }

        // Create mesh
        const mesh_shape_settings = try MeshShapeSettings.createIndexed(allocator, vertices, indices, .{});
        try self.addToWorld(.initWithSettings(mesh_shape_settings.asShapeSettings(), RVec3.init(@as(Real, -center), 0, @as(Real, -center)), Quat.identity(), .static, Layers.non_moving));

        // Create pyramid stairs
        for (0..10) |i| {
            const fi: f32 = @floatFromInt(i);
            const width = 4.0 - 0.4 * fi;
            const step = try BoxShape.create(allocator, Vec3.init(width, 0.5 * stairs_step_height, width), .{});
            try self.addToWorld(.initWithShape(step.asShape(), RVec3.init(-4.0, -1.0 + @as(Real, fi * stairs_step_height), 0), Quat.identity(), .static, Layers.non_moving));
        }

        // Create wall consisting of vertical pillars
        var wall = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(0.1, 2.5, 0.1), .{ .convex_radius = 0.0 })).asShape());
        defer wall.deinit();
        for (0..10) |z| {
            const rz: Real = @floatFromInt(z);
            try self.addToWorld(.initWithShape(wall.get(), RVec3.init(2.0, 1.0, 2.0 + 0.2 * rz), Quat.identity(), .static, Layers.non_moving));
        }

        // Create some dynamic boxes
        var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.25), .{})).asShape());
        defer box.deinit();
        for (0..10) |x| {
            for (0..10) |z| {
                const rx: Real = @floatFromInt(x);
                const rz: Real = @floatFromInt(z);
                var bcs: BodyCreationSettings = .initWithShape(box.get(), RVec3.init(4.0 * rx - 20.0, 5.0, 4.0 * rz - 20.0), Quat.identity(), .dynamic, Layers.moving);
                bcs.override_mass_properties = .calculate_inertia;
                bcs.mass_properties_override.mass = 1.0;
                try self.addToWorld(bcs);
            }
        }

        return true;
    }

    /// `RotatedTranslatedShapeSettings(Vec3(0, offset_y, 0), Quat::sIdentity(), new CapsuleShape(half_height, radius)).Create().Get()`
    fn createCapsule(allocator: Allocator, offset_y: f32, half_height: f32, radius: f32) PerformanceTestScene.Error!RefConst(Shape) {
        var capsule = RefConst(Shape).init((try CapsuleShape.create(allocator, half_height, radius, .{})).asShape());
        defer capsule.deinit();
        var settings = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(0, offset_y, 0), Quat.identity(), capsule.get());
        defer settings.deinit();
        var result = try settings.createShape(allocator);
        defer result.deinit();
        return .init(result.getPtr().?);
    }

    pub fn startTest(self: *CharacterVirtualScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) PerformanceTestScene.Error!void {
        const allocator = self.allocator;

        // Construct bodies
        const bi = physics_system.getBodyInterfaceMut();
        for (self.world.items) |*bcs| {
            if (bcs.motion_type == .dynamic) {
                bcs.motion_quality = motion_quality;
                _ = try bi.createAndAddBody(bcs, .activate);
            } else _ = try bi.createAndAddBody(bcs, .dont_activate);
        }

        // Construct characters
        CharacterID.setNextCharacterID(.{});
        var standing_shape = try createCapsule(allocator, 0.5 * character_height_standing + character_radius_standing, 0.5 * character_height_standing, character_radius_standing);
        defer standing_shape.deinit();
        var inner_standing_shape = try createCapsule(allocator, 0.5 * character_height_standing + character_radius_standing, 0.5 * inner_shape_fraction * character_height_standing, inner_shape_fraction * character_radius_standing);
        defer inner_standing_shape.deinit();
        errdefer self.stopTest(physics_system); // Out of the character vs character collision, then released
        try self.characters.ensureTotalCapacity(allocator, num_characters_x * num_characters_y);
        try self.character_vs_character_collision.characters.ensureTotalCapacity(allocator, num_characters_x * num_characters_y);
        for (0..num_characters_y) |y| {
            for (0..num_characters_x) |x| {
                var settings = CharacterVirtualSettings.init(allocator);
                defer settings.deinit();
                settings.base.shape.set(standing_shape.get());
                settings.base.supporting_volume = Plane.init(Vec3.axisY(), -character_radius_standing); // Accept contacts that touch the lower sphere of the capsule
                settings.inner_body_shape.set(inner_standing_shape.get());
                settings.inner_body_layer = Layers.moving;
                const rx: Real = @floatFromInt(x);
                const ry: Real = @floatFromInt(y);
                const character = try CharacterVirtual.create(allocator, &settings, RVec3.init(4.0 * rx - 20.0, 2.0, 4.0 * ry - 20.0), Quat.identity(), 0, physics_system);
                character.base.addRef();
                character.setCharacterVsCharacterCollision(self.character_vs_character_collision.characterVsCharacterCollision());
                character.setListener(CharacterContactListener.init(self));
                self.characters.appendAssumeCapacity(character);
                self.character_vs_character_collision.add(character) catch unreachable; // Capacity reserved above
            }
        }

        // Start at time 0
        self.time = 0.0;
        self.hash = HashCombine.hashBytes(&.{});
    }

    pub fn updateTest(self: *CharacterVirtualScene, physics_system: *PhysicsSystem, temp_allocator: TempAllocator, delta_time: f32) PerformanceTestScene.Error!void {
        // Change direction every 2 seconds
        self.time += delta_time;
        var count: u64 = @as(u64, @intFromFloat(self.time / 2.0)) * num_characters_x * num_characters_y;

        for (self.characters.items) |ch| {
            // Calculate new vertical velocity
            var new_velocity: Vec3 = undefined;
            if (ch.base.getGroundState() == .on_ground and // If on ground
                ch.getLinearVelocity().getY() < 0.1) // And not moving away from ground
                new_velocity = Vec3.zero()
            else
                new_velocity = ch.getLinearVelocity().mul(Vec3.init(0, 1, 0));
            new_velocity = new_velocity.add(physics_system.getGravity().mulScalar(delta_time));

            // Deterministic random input
            const hash = HashCombine.hash(count);
            const x: i32 = @intCast(hash % 10);
            const y: i32 = @intCast((hash / 10) % 10);
            const speed: i32 = @intCast((hash / 100) % 10);

            // Determine target position
            const target = RVec3.init(4.0 * @as(Real, @floatFromInt(x)) - 20.0, 5.0, 4.0 * @as(Real, @floatFromInt(y)) - 20.0);

            // Determine new character velocity
            var direction = target.sub(ch.getPosition()).toVec3().normalizedOr(Vec3.zero());
            direction.setY(0);
            new_velocity = new_velocity.add(direction.mulScalar(5.0 + 0.5 * @as(f32, @floatFromInt(speed))));
            ch.setLinearVelocity(new_velocity);

            // Update the character position
            const update_settings: CharacterVirtual.ExtendedUpdateSettings = .{};
            const broad_phase_layer_filter = physics_system.getDefaultBroadPhaseLayerFilter(Layers.moving);
            const object_layer_filter = physics_system.getDefaultLayerFilter(Layers.moving);
            try ch.extendedUpdate(delta_time, physics_system.getGravity(), &update_settings, &broad_phase_layer_filter.base, &object_layer_filter.base, &.{}, null, temp_allocator);

            count += 1;
        }
    }

    pub fn updateHash(self: *const CharacterVirtualScene, hash: *u64) void {
        // Hash the contact callback hash
        HashCombine.hashCombine(hash, self.hash);

        // Hash the state of all characters
        for (self.characters.items) |ch|
            HashCombine.hashCombine(hash, ch.getPosition());
    }

    /// mCharacters.clear(): release the characters (destroys them and their inner bodies)
    fn releaseCharacters(self: *CharacterVirtualScene) void {
        for (self.characters.items) |ch| ch.base.release();
        self.characters.clearRetainingCapacity();
    }

    pub fn stopTest(self: *CharacterVirtualScene, physics_system: *PhysicsSystem) void {
        _ = physics_system;
        for (self.characters.items) |ch|
            self.character_vs_character_collision.remove(ch);
        self.releaseCharacters();
    }

    // See: CharacterContactListener
    pub fn onContactAdded(self: *CharacterVirtualScene, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        HashCombine.hashCombine(&self.hash, @as(i32, 1));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, contact.base.body_b);
        HashCombine.hashCombine(&self.hash, contact.base.sub_shape_id_b.getValue());
        HashCombine.hashCombine(&self.hash, contact.position);
        HashCombine.hashCombine(&self.hash, contact.contact_normal);
    }

    pub fn onContactPersisted(self: *CharacterVirtualScene, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        HashCombine.hashCombine(&self.hash, @as(i32, 2));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, contact.base.body_b);
        HashCombine.hashCombine(&self.hash, contact.base.sub_shape_id_b.getValue());
        HashCombine.hashCombine(&self.hash, contact.position);
        HashCombine.hashCombine(&self.hash, contact.contact_normal);
    }

    pub fn onContactRemoved(self: *CharacterVirtualScene, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
        HashCombine.hashCombine(&self.hash, @as(i32, 3));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, body_id2);
        HashCombine.hashCombine(&self.hash, sub_shape_id2.getValue());
    }

    pub fn onCharacterContactAdded(self: *CharacterVirtualScene, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        HashCombine.hashCombine(&self.hash, @as(i32, 4));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, contact.character_b.?.getID());
        HashCombine.hashCombine(&self.hash, contact.base.sub_shape_id_b.getValue());
        HashCombine.hashCombine(&self.hash, contact.position);
        HashCombine.hashCombine(&self.hash, contact.contact_normal);
    }

    pub fn onCharacterContactPersisted(self: *CharacterVirtualScene, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
        _ = settings;
        HashCombine.hashCombine(&self.hash, @as(i32, 5));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, contact.character_b.?.getID());
        HashCombine.hashCombine(&self.hash, contact.base.sub_shape_id_b.getValue());
        HashCombine.hashCombine(&self.hash, contact.position);
        HashCombine.hashCombine(&self.hash, contact.contact_normal);
    }

    pub fn onCharacterContactRemoved(self: *CharacterVirtualScene, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void {
        HashCombine.hashCombine(&self.hash, @as(i32, 6));
        HashCombine.hashCombine(&self.hash, character.getID());
        HashCombine.hashCombine(&self.hash, other_character_id);
        HashCombine.hashCombine(&self.hash, sub_shape_id2.getValue());
    }
};

test "CharacterVirtualScene: an allocation failure in StartTest leaves no character behind" {
    const allocator = std.testing.allocator;
    const LayersFile = @import("Layers.zig");
    const self = try CharacterVirtualScene.create(allocator);
    defer self.destroy();
    try std.testing.expect(try self.load(""));
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();
    const object_vs_broadphase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{};
    const object_vs_object_layer_filter: LayersFile.ObjectLayerPairFilterImpl = .{};
    const scene_interface = self.scene();

    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var physics_system: PhysicsSystem = .empty;
        try physics_system.init(allocator, std.testing.io, scene_interface.getMaxBodies(), 0, scene_interface.getMaxBodyPairs(), scene_interface.getMaxContactConstraints(), &broad_phase_layer_interface.base, &object_vs_broadphase_layer_filter.base, &object_vs_object_layer_filter.base, .{});
        defer physics_system.deinit();

        // Every allocation of the scene in StartTest fails once (the FailingAllocator frees through the same allocator)
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        self.allocator = failing.allocator();
        defer self.allocator = allocator;
        self.startTest(&physics_system, .discrete) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            try std.testing.expectEqual(@as(usize, 0), self.characters.items.len);
            try std.testing.expectEqual(@as(usize, 0), self.character_vs_character_collision.characters.items.len);
            continue;
        };
        try std.testing.expect(!failing.has_induced_failure);
        try std.testing.expectEqual(@as(usize, CharacterVirtualScene.num_characters_x * CharacterVirtualScene.num_characters_y), self.character_vs_character_collision.characters.items.len);
        self.stopTest(&physics_system);
        try std.testing.expect(fail_index > CharacterVirtualScene.num_characters_x * CharacterVirtualScene.num_characters_y); // Every character was created under the sweep
        break;
    }
}
