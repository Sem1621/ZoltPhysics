//! Port of: UnitTests/Physics/CharacterVirtualTests.cpp
//! Status: complete
//!
//! - Jolt's test class `Character : public CharacterContactListener` is the struct `Character`: it is initialized in
//!   place (`var character = Character.init(&c); defer character.deinit();`, the character keeps a pointer to it as
//!   its listener) and declares the callbacks Jolt's class overrides, which forward to the LoggingCharacterContactListener
//!   (OnContactSolve prevents sliding when the character doesn't want to move). `Ref<CharacterVirtual>` is a heap
//!   character with one reference that `deinit` releases.
//! - `Step` uses a TempAllocatorMalloc on the testing allocator like Jolt, and reports an append that failed in the
//!   contact log after the update (`LoggingCharacterContactListener.checkError`).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingCharacterContactListener = @import("../LoggingCharacterContactListener.zig").LoggingCharacterContactListener;

const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const CapsuleShape = zolt.CapsuleShape;
const CharacterBase = zolt.CharacterBase;
const CharacterContactListener = zolt.CharacterContactListener;
const CharacterVirtual = zolt.CharacterVirtual;
const CharacterVirtualSettings = zolt.CharacterVirtualSettings;
const CharacterVsCharacterCollisionSimple = zolt.CharacterVsCharacterCollisionSimple;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const MeshShapeSettings = zolt.MeshShapeSettings;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RMat44 = zolt.RMat44;
const RVec3 = zolt.RVec3;
const Real = zolt.Real;
const RefConst = zolt.RefConst;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const Shape = zolt.Shape;
const SubShapeID = zolt.SubShapeID;
const TempAllocatorMalloc = zolt.TempAllocatorMalloc;
const Triangle = zolt.Triangle;
const TriangleList = zolt.TriangleList;
const Vec3 = zolt.Vec3;
const math = zolt.math;
const trigonometry = zolt.trigonometry;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

const EType = LoggingCharacterContactListener.Type;

const Character = struct {
    // Configuration
    initial_position: RVec3 = RVec3.zero(),
    height_standing: f32 = 1.35,
    radius_standing: f32 = 0.3,
    character_settings: CharacterVirtualSettings,
    update_settings: CharacterVirtual.ExtendedUpdateSettings = .{},

    // Character movement settings (update to control the movement of the character)
    horizontal_speed: Vec3 = Vec3.zero(),
    jump_speed: f32 = 0.0, // Character will jump when not 0, will auto reset

    // The character (Ref<CharacterVirtual>)
    character: ?*CharacterVirtual = null,

    // Character vs character
    character_vs_character: CharacterVsCharacterCollisionSimple,

    // Calculated effective velocity after a step
    effective_velocity: Vec3 = Vec3.zero(),

    // Log of contact events
    contact_log: LoggingCharacterContactListener,

    context: *PhysicsTestContext,

    // Construct
    fn init(context: *PhysicsTestContext) Character {
        return .{ .character_settings = .init(allocator), .character_vs_character = .init(allocator), .contact_log = .init(allocator), .context = context };
    }

    // Destruct (Jolt's members in reverse order)
    fn deinit(self: *Character) void {
        self.contact_log.deinit();
        self.character_vs_character.deinit();
        if (self.character) |character| character.base.release();
        self.character_settings.deinit();
    }

    // Create the character
    fn create(self: *Character) !void {
        // Create capsule
        var capsule = RefConst(Shape).init((try CapsuleShape.create(allocator, 0.5 * self.height_standing, self.radius_standing, .{})).asShape());
        defer capsule.deinit();
        var rt_settings = RotatedTranslatedShapeSettings.initPtr(allocator, Vec3.init(0, 0.5 * self.height_standing + self.radius_standing, 0), Quat.identity(), capsule.get());
        defer rt_settings.deinit();
        var result = try rt_settings.createShape(allocator);
        defer result.deinit();
        self.character_settings.base.shape.set(result.getPtr());

        // Configure supporting volume
        self.character_settings.base.supporting_volume = Plane.init(Vec3.axisY(), -self.height_standing); // Accept contacts that touch the lower sphere of the capsule

        // Create character
        const character = try CharacterVirtual.create(allocator, &self.character_settings, self.initial_position, Quat.identity(), 0, self.context.getSystem());
        character.base.addRef();
        self.character = character;
        character.setListener(CharacterContactListener.init(self));
        character.setCharacterVsCharacterCollision(self.character_vs_character.characterVsCharacterCollision());
    }

    // Step the character and the world
    fn step(self: *Character) !void {
        const character = self.character.?;

        // Step the world
        _ = try self.context.simulateSingleStep();

        // Determine new basic velocity
        const current_vertical_velocity = Vec3.init(0, character.getLinearVelocity().getY(), 0);
        const ground_velocity = character.base.getGroundVelocity();
        var new_velocity: Vec3 = undefined;
        if (character.base.getGroundState() == .on_ground and // If on ground
            (current_vertical_velocity.getY() - ground_velocity.getY()) < 0.1) // And not moving away from ground
        {
            // Assume velocity of ground when on ground
            new_velocity = ground_velocity;

            // Jump
            new_velocity = new_velocity.add(Vec3.init(0, self.jump_speed, 0));
            self.jump_speed = 0.0;
        } else new_velocity = current_vertical_velocity;

        // Gravity
        const system = self.context.getSystem();
        const delta_time = self.context.getDeltaTime();
        new_velocity = new_velocity.add(system.getGravity().mulScalar(delta_time));

        // Player input
        new_velocity = new_velocity.add(self.horizontal_speed);

        // Update character velocity
        character.setLinearVelocity(new_velocity);

        const start_pos = self.getPosition();

        // Update the character position
        var temp_allocator = TempAllocatorMalloc.init(allocator);
        const broad_phase_layer_filter = system.getDefaultBroadPhaseLayerFilter(Layers.moving);
        const object_layer_filter = system.getDefaultLayerFilter(Layers.moving);
        try character.extendedUpdate(delta_time, system.getGravity(), &self.update_settings, &broad_phase_layer_filter.base, &object_layer_filter.base, &.{}, null, temp_allocator.tempAllocator());
        try self.contact_log.checkError();

        // Calculate effective velocity in this step
        self.effective_velocity = self.getPosition().sub(start_pos).toVec3().divScalar(delta_time);
    }

    // Simulate a longer period of time
    fn simulate(self: *Character, time: f32) !void {
        const num_steps: i32 = @intFromFloat(@round(time / self.context.getDeltaTime()));
        var s: i32 = 0;
        while (s < num_steps) : (s += 1)
            try self.step();
    }

    // Get the number of active contacts
    fn getNumContacts(self: *const Character) usize {
        return self.character.?.getActiveContacts().len;
    }

    // Check if the character is in contact with another body
    fn hasCollidedWith(self: *const Character, body: BodyID) bool {
        return self.character.?.hasCollidedWith(body);
    }

    // Check if the character is in contact with another character
    fn hasCollidedWithCharacter(self: *const Character, character: *const CharacterVirtual) bool {
        return self.character.?.hasCollidedWithCharacter(character);
    }

    // Get position of character
    fn getPosition(self: *const Character) RVec3 {
        return self.character.?.getPosition();
    }

    // CharacterContactListener callbacks
    pub fn onContactValidate(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact) bool {
        return self.contact_log.onContactValidate(character, contact);
    }

    pub fn onCharacterContactValidate(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact) bool {
        return self.contact_log.onCharacterContactValidate(character, contact);
    }

    pub fn onContactAdded(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact, settings: *zolt.CharacterContactSettings) void {
        self.contact_log.onContactAdded(character, contact, settings);
    }

    pub fn onContactPersisted(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact, settings: *zolt.CharacterContactSettings) void {
        self.contact_log.onContactPersisted(character, contact, settings);
    }

    pub fn onContactRemoved(self: *Character, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
        self.contact_log.onContactRemoved(character, body_id2, sub_shape_id2);
    }

    pub fn onCharacterContactAdded(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact, settings: *zolt.CharacterContactSettings) void {
        self.contact_log.onCharacterContactAdded(character, contact, settings);
    }

    pub fn onCharacterContactPersisted(self: *Character, character: *const CharacterVirtual, contact: *const zolt.CharacterContact, settings: *zolt.CharacterContactSettings) void {
        self.contact_log.onCharacterContactPersisted(character, contact, settings);
    }

    pub fn onCharacterContactRemoved(self: *Character, character: *const CharacterVirtual, other_character_id: zolt.CharacterID, sub_shape_id2: SubShapeID) void {
        self.contact_log.onCharacterContactRemoved(character, other_character_id, sub_shape_id2);
    }

    pub fn onContactSolve(self: *Character, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
        _ = .{ body_id2, sub_shape_id2, contact_position, contact_material, character_velocity };

        // Don't allow sliding if the character doesn't want to move
        if (self.horizontal_speed.isNearZero(.{}) and contact_velocity.isNearZero(.{}) and !character.base.isSlopeTooSteep(contact_normal))
            new_character_velocity.* = Vec3.zero();
    }
};

test "TestFallingAndJumping" {
    // Create floor
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    _ = try c.createFloor();

    // Create character
    var character = Character.init(&c);
    defer character.deinit();
    character.initial_position = RVec3.init(0, 2, 0);
    try character.create();

    // After 1 step we should still be in air
    try character.step();
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.in_air);

    // After some time we should be on the floor
    try character.simulate(1.0);
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
    try checkApproxEqual(character.getPosition(), RVec3.zero(), .{});
    try checkApproxEqual(character.effective_velocity, Vec3.zero(), .{});

    // Jump
    character.jump_speed = 1.0;
    try character.step();
    const velocity = Vec3.init(0, 1.0 + c.getDeltaTime() * c.getSystem().getGravity().getY(), 0);
    try checkApproxEqual(character.getPosition(), RVec3.fromVec3(velocity.mulScalar(c.getDeltaTime())), .{});
    try checkApproxEqual(character.effective_velocity, velocity, .{});
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.in_air);

    // After some time we should be on the floor again
    try character.simulate(1.0);
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
    try checkApproxEqual(character.getPosition(), RVec3.zero(), .{});
    try checkApproxEqual(character.effective_velocity, Vec3.zero(), .{});
}

test "TestMovingOnSlope" {
    const floor_half_height: f32 = 1.0;
    const movement_time: f32 = 1.5;

    // Iterate various slope angles
    var slope_angle = math.degreesToRadians(5.0);
    while (slope_angle < math.degreesToRadians(85.0)) : (slope_angle += math.degreesToRadians(10.0)) {
        // Create sloped floor
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        const slope_rotation = Quat.rotation(Vec3.axisZ(), slope_angle);
        _ = try c.createBox(RVec3.zero(), slope_rotation, .static, .discrete, Layers.non_moving, Vec3.init(100.0, floor_half_height, 100.0), .{});

        // Create character so that it is touching the slope
        var character = Character.init(&c);
        defer character.deinit();
        const radius_and_padding = character.radius_standing + character.character_settings.character_padding;
        character.initial_position = RVec3.init(0, (radius_and_padding + floor_half_height) / trigonometry.cos(slope_angle) - radius_and_padding, 0);
        try character.create();

        // Determine if the slope is too steep for the character
        const too_steep = slope_angle > character.character_settings.base.max_slope_angle;
        const expected_ground_state: CharacterBase.GroundState = if (too_steep) .on_steep_ground else .on_ground;

        const gravity = c.getSystem().getGravity();
        const time_step = c.getDeltaTime();
        const slope_normal = slope_rotation.rotateAxisY();

        // Calculate expected position after 1 time step
        var position_after_1_step = character.initial_position;
        if (too_steep) {
            // Apply 1 frame of gravity and cancel movement in the slope normal direction
            var velocity = gravity.mulScalar(time_step);
            velocity = velocity.sub(slope_normal.mulScalar(velocity.dot(slope_normal)));
            position_after_1_step = position_after_1_step.addVec3(velocity.mulScalar(time_step));
        }

        // After 1 step we should be on the slope
        try character.step();
        try expect(character.character.?.base.getGroundState() == expected_ground_state);
        try checkApproxEqual(character.getPosition(), position_after_1_step, .{ .tolerance = 1.0e-5 });

        // Cancel any velocity to make the calculation below easier (otherwise we have to take gravity for 1 time step into account)
        character.character.?.setLinearVelocity(Vec3.zero());

        const start_pos = character.getPosition();

        // Start moving in X direction
        character.horizontal_speed = Vec3.init(2.0, 0, 0);
        try character.simulate(movement_time);
        try expect(character.character.?.base.getGroundState() == expected_ground_state);

        // Calculate resulting translation
        const translation = character.getPosition().sub(start_pos).toVec3();

        // Calculate expected translation
        var expected_translation: Vec3 = undefined;
        if (too_steep) {
            // If too steep, we're just falling. Integrate using an Euler integrator.
            var velocity = Vec3.zero();
            expected_translation = Vec3.zero();
            const num_steps: i32 = @intFromFloat(@round(movement_time / time_step));
            var i: i32 = 0;
            while (i < num_steps) : (i += 1) {
                velocity = velocity.add(gravity.mulScalar(time_step));
                expected_translation = expected_translation.add(velocity.mulScalar(time_step));
            }
        } else {
            // Every frame we apply 1 delta time * gravity which gets reset on the next update, add this to the horizontal speed
            expected_translation = character.horizontal_speed.add(gravity.mulScalar(time_step)).mulScalar(movement_time);
        }

        // Cancel movement in slope direction
        expected_translation = expected_translation.sub(slope_normal.mulScalar(expected_translation.dot(slope_normal)));

        // Check that we traveled the right amount
        try checkApproxEqual(translation, expected_translation, .{ .tolerance = 1.0e-4 });
    }
}

test "TestStickToFloor" {
    const floor_half_height: f32 = 1.0;
    const slope_angle = math.degreesToRadians(45.0);
    const movement_time: f32 = 1.5;

    for (0..2) |mode| {
        // If this run is with 'stick to floor' enabled
        const stick_to_floor = mode == 0;

        // Create sloped floor
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();
        const slope_rotation = Quat.rotation(Vec3.axisZ(), slope_angle);
        const floor = try c.createBox(RVec3.zero(), slope_rotation, .static, .discrete, Layers.non_moving, Vec3.init(100.0, floor_half_height, 100.0), .{});

        // Create character so that it is touching the slope
        var character = Character.init(&c);
        defer character.deinit();
        const radius_and_padding = character.radius_standing + character.character_settings.character_padding;
        character.initial_position = RVec3.init(0, (radius_and_padding + floor_half_height) / trigonometry.cos(slope_angle) - radius_and_padding, 0);
        character.update_settings.stick_to_floor_step_down = if (stick_to_floor) Vec3.init(0, -0.5, 0) else Vec3.zero();
        try character.create();
        const cv = character.character.?;

        // After 1 step we should be on the slope
        try character.step();
        try expect(cv.base.getGroundState() == CharacterBase.GroundState.on_ground);
        try expect(character.contact_log.getEntryCount() == 2);
        try expect(character.contact_log.contains(EType.validate_body, cv, floor.getID()));
        try expect(character.contact_log.contains(EType.add_body, cv, floor.getID()));
        character.contact_log.clear();

        // Cancel any velocity to make the calculation below easier (otherwise we have to take gravity for 1 time step into account)
        cv.setLinearVelocity(Vec3.zero());

        const start_pos = character.getPosition();

        const time_step = c.getDeltaTime();
        const num_steps: i32 = @intFromFloat(@round(movement_time / time_step));

        var i: i32 = 0;
        while (i < num_steps) : (i += 1) {
            // Start moving down the slope at a speed high enough so that gravity will not keep us on the floor
            character.horizontal_speed = Vec3.init(-10.0, 0, 0);
            try character.step();

            if (stick_to_floor) {
                // Should stick to floor
                try expect(cv.base.getGroundState() == CharacterBase.GroundState.on_ground);

                // Should have received callbacks
                try expect(character.contact_log.getEntryCount() == 2);
                try expect(character.contact_log.contains(EType.validate_body, cv, floor.getID()));
                try expect(character.contact_log.contains(EType.persist_body, cv, floor.getID()));
                character.contact_log.clear();
            } else {
                // Should be off ground
                try expect(cv.base.getGroundState() == CharacterBase.GroundState.in_air);

                // Remove callbacks
                try expect(character.contact_log.getEntryCount() == 1);
                try expect(character.contact_log.contains(EType.remove_body, cv, floor.getID()));
            }
        }

        // Calculate resulting translation
        const translation = character.getPosition().sub(start_pos).toVec3();

        // Calculate expected translation
        var expected_translation: Vec3 = undefined;
        if (stick_to_floor) {
            // We should stick to the floor, so the vertical translation follows the slope perfectly
            expected_translation = character.horizontal_speed.mulScalar(movement_time);
            expected_translation.setY(expected_translation.getX() * trigonometry.tan(slope_angle));
        } else {
            // If too steep, we're just falling. Integrate using an Euler integrator.
            var velocity = character.horizontal_speed;
            expected_translation = Vec3.zero();
            const gravity = c.getSystem().getGravity();
            var s: i32 = 0;
            while (s < num_steps) : (s += 1) {
                velocity = velocity.add(gravity.mulScalar(time_step));
                expected_translation = expected_translation.add(velocity.mulScalar(time_step));
            }
        }

        // Check that we traveled the right amount
        try checkApproxEqual(translation, expected_translation, .{ .tolerance = 1.0e-4 });
    }
}

test "TestWalkStairs" {
    const step_height: f32 = 0.3;
    const num_steps = 10;

    // Create stairs from triangles
    var triangles: TriangleList = .empty;
    defer triangles.deinit(allocator);
    for (0..num_steps) |i| {
        const fi: f32 = @floatFromInt(i);

        // Start of step
        const base = Vec3.init(0, step_height * fi, step_height * fi);

        // Left side
        const b1 = base.add(Vec3.init(2.0, 0, 0));
        const s1 = b1.add(Vec3.init(0, step_height, 0));
        const p1 = s1.add(Vec3.init(0, 0, step_height));

        // Right side
        const width = Vec3.init(-4.0, 0, 0);
        const b2 = b1.add(width);
        const s2 = s1.add(width);
        const p2 = p1.add(width);

        try triangles.append(allocator, .init(s1, b1, s2, .{}));
        try triangles.append(allocator, .init(b1, b2, s2, .{}));
        try triangles.append(allocator, .init(s1, p2, p1, .{}));
        try triangles.append(allocator, .init(s1, s2, p2, .{}));
    }

    var mesh = try MeshShapeSettings.init(allocator, triangles.items, .{});
    defer mesh.deinit();
    mesh.asShapeSettings().setEmbedded();
    var mesh_stairs: BodyCreationSettings = .initWithSettings(mesh.asShapeSettings(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer mesh_stairs.deinit();

    // Stair stepping is very delta time sensitive, so test various update frequencies
    const frequencies = [_]f32{ 60.0, 120.0, 240.0, 360.0 };
    for (frequencies) |frequency| {
        const time_step = 1.0 / frequency;

        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{ .delta_time = time_step });
        defer c.deinit();
        _ = try c.createFloor();
        _ = try c.getBodyInterface().createAndAddBody(&mesh_stairs, .dont_activate);

        // Create character so that it is touching the slope
        var character = Character.init(&c);
        defer character.deinit();
        character.initial_position = RVec3.init(0, 0, -2.0); // Start in front of the stairs
        character.update_settings.walk_stairs_step_up = Vec3.zero(); // No stair walking
        try character.create();

        // Start moving towards the stairs
        character.horizontal_speed = Vec3.init(0, 0, 4.0);
        try character.simulate(1.0);

        // We should have gotten stuck at the start of the stairs (can't move up)
        try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
        const radius_and_padding = character.radius_standing + character.character_settings.character_padding;
        try checkApproxEqual(character.getPosition(), RVec3.init(0, 0, -radius_and_padding), .{ .tolerance = 1.1e-2 });

        // Enable stair walking
        character.update_settings.walk_stairs_step_up = Vec3.init(0, 0.4, 0);

        // Calculate time it should take to move up the stairs at constant speed
        const movement_time = (num_steps * step_height + radius_and_padding) / character.horizontal_speed.getZ();
        const max_steps: i32 = @intFromFloat(1.5 * @round(movement_time / time_step)); // In practice there is a bit of slowdown while stair stepping, so add a bit of slack

        // Step until we reach the top of the stairs
        var last_position = character.getPosition();
        var reached_goal = false;
        var i: i32 = 0;
        while (i < max_steps) : (i += 1) {
            try character.step();

            // We should always be on the floor during stair stepping
            try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);

            // Check position progression
            const position = character.getPosition();
            try checkApproxEqual(position.getX(), 0, .{}); // No movement in X
            try expect(position.getZ() > last_position.getZ()); // Always moving forward
            try expect(position.getZ() < @as(Real, num_steps * step_height)); // No movement beyond stairs
            if (position.getY() > @as(Real, num_steps * step_height - 1.0e-3)) {
                reached_goal = true;
                break;
            }

            last_position = position;
        }
        try expect(reached_goal);
    }
}

test "TestRotatingPlatform" {
    const floor_half_height: f32 = 1.0;
    const floor_half_width: f32 = 10.0;
    const character_position: f32 = 0.9 * floor_half_width;
    const angular_velocity: f32 = 2.0 * math.pi;

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create box
    const box = try c.createBox(RVec3.zero(), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.init(floor_half_width, floor_half_height, floor_half_width), .{});
    box.setAllowSleeping(false);

    // Create character so that it is touching the box at the
    var character = Character.init(&c);
    defer character.deinit();
    character.initial_position = RVec3.init(character_position, floor_half_height, 0);
    try character.create();

    // Step to ensure the character is on the box
    try character.step();
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);

    // Set the box to rotate a full circle per second
    box.setAngularVelocity(Vec3.init(0, angular_velocity, 0));

    // Rotate and check that character stays on the box
    for (0..60) |t| {
        try character.step();
        try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);

        // Note that the character moves according to the ground velocity and the ground velocity is updated at the end of the step
        // so the character is always 1 time step behind the platform. This is why we use t and not t + 1 to calculate the expected position.
        const expected_position = RMat44.rotationQuat(Quat.rotation(Vec3.axisY(), @as(f32, @floatFromInt(t)) * c.getDeltaTime() * angular_velocity)).mulRVec3(character.initial_position);
        try checkApproxEqual(character.getPosition(), expected_position, .{ .tolerance = 1.0e-4 });
    }
}

test "TestMovingPlatformUp" {
    const floor_half_height: f32 = 1.0;
    const floor_half_width: f32 = 10.0;
    const linear_velocity: f32 = 0.5;

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create box
    const box = try c.createBox(RVec3.zero(), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.init(floor_half_width, floor_half_height, floor_half_width), .{});
    box.setAllowSleeping(false);

    // Create character so that it is touching the box at the
    var character = Character.init(&c);
    defer character.deinit();
    character.initial_position = RVec3.init(0, floor_half_height, 0);
    try character.create();

    // Step to ensure the character is on the box
    try character.step();
    try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);

    // Set the box to move up
    box.setLinearVelocity(Vec3.init(0, linear_velocity, 0));

    // Check that character stays on the box
    for (0..60) |_| {
        try character.step();
        try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
        const expected_position = box.getPosition().add(character.initial_position);
        try checkApproxEqual(character.getPosition(), expected_position, .{ .tolerance = 1.0e-2 });
    }

    // Stop box
    box.setLinearVelocity(Vec3.zero());
    try character.simulate(0.5);

    // Set the box to move down
    box.setLinearVelocity(Vec3.init(0, -linear_velocity, 0));

    // Check that character stays on the box
    for (0..60) |_| {
        try character.step();
        try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
        const expected_position = box.getPosition().add(character.initial_position);
        try checkApproxEqual(character.getPosition(), expected_position, .{ .tolerance = 1.0e-2 });
    }
}

test "TestContactPointLimit" {
    var ctx: PhysicsTestContext = undefined;
    try ctx.init(allocator, .{});
    defer ctx.deinit();
    const floor = try ctx.createFloor();

    // Create character at the origin
    var character = Character.init(&ctx);
    defer character.deinit();
    character.initial_position = RVec3.init(0, 1, 0);
    character.update_settings.stick_to_floor_step_down = Vec3.zero();
    character.update_settings.walk_stairs_step_up = Vec3.zero();
    try character.create();
    const cv = character.character.?;

    // Radius including padding
    const character_radius = character.radius_standing + character.character_settings.character_padding;

    // Create a half cylinder with caps for testing contact point limit
    var vertices: zolt.VertexList = .empty;
    defer vertices.deinit(allocator);
    var triangles: std.ArrayList(IndexedTriangle) = .empty;
    defer triangles.deinit(allocator);

    // The half cylinder
    const pos_segments = 2;
    const angle_segments = 768;
    const cylinder_length: f32 = 2.0;
    for (0..pos_segments) |pos| {
        for (0..angle_segments) |angle| {
            const start: u32 = @intCast(vertices.items.len);

            const radius = character_radius + 0.01;
            const angle_rad = (-0.5 + @as(f32, @floatFromInt(angle)) / angle_segments) * math.pi;
            const s = trigonometry.sin(angle_rad);
            const cs = trigonometry.cos(angle_rad);
            const x = cylinder_length * (-0.5 + @as(f32, @floatFromInt(pos)) / (pos_segments - 1));
            const y = if (angle == 0 or angle == angle_segments - 1) 0.5 else (1.0 - cs) * radius;
            const z = s * radius;
            try vertices.append(allocator, Float3.init(x, y, z));

            if (pos > 0 and angle > 0) {
                try triangles.append(allocator, .init(start, start - 1, start - angle_segments, .{}));
                try triangles.append(allocator, .init(start - 1, start - angle_segments - 1, start - angle_segments, .{}));
            }
        }
    }

    // Add end caps
    const end: u32 = angle_segments * (pos_segments - 1);
    for (0..angle_segments - 1) |angle_usize| {
        const angle: u32 = @intCast(angle_usize);
        try triangles.append(allocator, .init(0, angle + 1, angle, .{}));
        try triangles.append(allocator, .init(end, end + angle, end + angle + 1, .{}));
    }

    // Create test body
    var mesh = try MeshShapeSettings.initIndexed(allocator, vertices.items, triangles.items, .{});
    defer mesh.deinit();
    mesh.asShapeSettings().setEmbedded();
    var mesh_cylinder: BodyCreationSettings = .initWithSettings(mesh.asShapeSettings(), character.initial_position, Quat.identity(), .static, Layers.non_moving);
    defer mesh_cylinder.deinit();
    const cylinder_id = try ctx.getBodyInterface().createAndAddBody(&mesh_cylinder, .dont_activate);

    // End positions that can be reached by character
    const character_radius_r: Real = character_radius;
    const pos_end = RVec3.init(@as(Real, 0.5) * @as(Real, cylinder_length) - character_radius_r, 1, 0);
    const neg_end = RVec3.init(@as(Real, -0.5) * @as(Real, cylinder_length) + character_radius_r, 1, 0);

    // Move towards positive cap and test if we hit the end
    character.horizontal_speed = Vec3.init(cylinder_length, 0, 0);
    for (0..60) |_| {
        try character.step();
        try expect(cv.getMaxHitsExceeded());
        try expect(character.getNumContacts() <= cv.getMaxNumHits());
        try expect(cv.base.getGroundBodyID().eql(cylinder_id));
        try expect(cv.base.getGroundNormal().dot(Vec3.axisY()) > 0.999);
    }
    try checkApproxEqual(character.getPosition(), pos_end, .{ .tolerance = 1.0e-4 });

    // Move towards negative cap and test if we hit the end
    character.horizontal_speed = Vec3.init(-cylinder_length, 0, 0);
    for (0..60) |_| {
        try character.step();
        try expect(cv.getMaxHitsExceeded());
        try expect(character.getNumContacts() <= cv.getMaxNumHits());
        try expect(cv.base.getGroundBodyID().eql(cylinder_id));
        try expect(cv.base.getGroundNormal().dot(Vec3.axisY()) > 0.999);
    }
    try checkApproxEqual(character.getPosition(), neg_end, .{ .tolerance = 1.0e-4 });

    // Turn off contact point reduction
    cv.setHitReductionCosMaxAngle(-1.0);

    // Move towards positive cap and test that we did not reach the end
    character.horizontal_speed = Vec3.init(cylinder_length, 0, 0);
    for (0..60) |_| {
        try character.step();
        try expect(cv.getMaxHitsExceeded());
        try expect(character.getNumContacts() == cv.getMaxNumHits());
    }
    const cur_pos = character.getPosition();
    try expect(pos_end.sub(cur_pos).length() > 0.01);

    // Move towards negative cap and test that we got stuck
    character.horizontal_speed = Vec3.init(-cylinder_length, 0, 0);
    for (0..60) |_| {
        try character.step();
        try expect(cv.getMaxHitsExceeded());
        try expect(character.getNumContacts() == cv.getMaxNumHits());
    }
    try expect(cur_pos.isClose(character.getPosition(), .{ .max_dist_sq = 1.0e-6 }));

    // Now teleport the character next to the half cylinder
    cv.setPosition(RVec3.init(0, 0, 1));

    // Move in positive X and check that we did not exceed max hits and that we were able to move unimpeded
    character.horizontal_speed = Vec3.init(cylinder_length, 0, 0);
    for (0..60) |_| {
        try character.step();
        try expect(!cv.getMaxHitsExceeded());
        try expect(character.getNumContacts() == 1); // We should only hit the floor
        try expect(cv.base.getGroundBodyID().eql(floor.getID()));
        try expect(cv.base.getGroundNormal().dot(Vec3.axisY()) > 0.999);
    }
    try checkApproxEqual(character.getPosition(), RVec3.init(cylinder_length, 0, 1), .{ .tolerance = 1.0e-4 });
}

test "TestStairWalkAlongWall" {
    // Stair stepping is very delta time sensitive, so test various update frequencies
    const frequencies = [_]f32{ 60.0, 120.0, 240.0, 360.0 };
    for (frequencies) |frequency| {
        const time_step = 1.0 / frequency;

        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{ .delta_time = time_step });
        defer c.deinit();
        _ = try c.createFloor();

        // Create character
        var character = Character.init(&c);
        defer character.deinit();
        try character.create();

        // Create a wall
        const wall_half_thickness: f32 = 0.05;
        const wall = try BoxShape.create(allocator, Vec3.init(50.0, 1.0, wall_half_thickness), .{});
        var wall_settings: BodyCreationSettings = .initWithShape(wall.asShape(), RVec3.init(0, 1.0, -character.radius_standing - character.character.?.getCharacterPadding() - wall_half_thickness), Quat.identity(), .static, Layers.non_moving);
        defer wall_settings.deinit();
        _ = try c.getBodyInterface().createAndAddBody(&wall_settings, .dont_activate);

        // Start moving along the wall, if the stair stepping algorithm is working correctly it should not trigger and not apply extra speed to the character
        character.horizontal_speed = Vec3.init(5.0, 0, -1.0);
        try character.simulate(1.0);

        // We should have moved along the wall at the desired speed
        try expect(character.character.?.base.getGroundState() == CharacterBase.GroundState.on_ground);
        try checkApproxEqual(character.getPosition(), RVec3.init(5.0, 0, 0), .{ .tolerance = 1.0e-2 });
    }
}

test "TestInitiallyIntersecting" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    _ = try c.createFloor();

    // Create box that is intersecting with the character
    _ = try c.createBox(RVec3.init(-0.5, 0.5, 0), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.replicate(0.5), .{});

    // Try various penetration recovery values
    for ([_]f32{ 0.0, 0.5, 0.75, 1.0 }) |penetration_recovery| {
        // Create character
        var character = Character.init(&c);
        defer character.deinit();
        character.character_settings.penetration_recovery_speed = penetration_recovery;
        try character.create();
        try checkApproxEqual(character.getPosition(), RVec3.zero(), .{});

        // Total radius of character
        const radius_and_padding = character.radius_standing + character.character_settings.character_padding;

        var x: f32 = 0.0;
        for (0..3) |_| {
            // Calculate expected position
            x += penetration_recovery * (radius_and_padding - x);

            // Step character and check that it matches expected recovery
            try character.step();
            try checkApproxEqual(character.getPosition(), RVec3.init(x, 0, 0), .{});
        }
    }
}

test "TestInitiallyIntersecting2" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    _ = try c.createFloor();

    // Create box that is intersecting with the character
    _ = try c.createBox(RVec3.init(-0.5, 0.5, 0), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.replicate(0.5), .{});

    // Create two very steep sloped floors that are initially intersecting with the character
    const size: f32 = 1.0;
    const thickness: f32 = 0.1;
    const slope_angle = math.degreesToRadians(75.0);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(size, thickness, size), .{})).asShape());
    defer box.deinit();
    var settings: BodyCreationSettings = .initWithShape(box.get(), RVec3.init(0, 0, 0), Quat.rotation(Vec3.axisZ(), slope_angle), .static, Layers.non_moving);
    defer settings.deinit();
    _ = try c.getBodyInterface().createAndAddBody(&settings, .dont_activate);
    settings.rotation = Quat.rotation(Vec3.axisZ(), -slope_angle);
    _ = try c.getBodyInterface().createAndAddBody(&settings, .dont_activate);

    // Create character
    var character = Character.init(&c);
    defer character.deinit();
    try character.create();
    try checkApproxEqual(character.getPosition(), RVec3.zero(), .{});

    // Allow the character to step and see that it gets driven upwards
    var y: Real = 0;
    for (0..10) |_| {
        try character.step();
        const new_y = character.getPosition().getY();
        try expect(new_y >= y);
        y = new_y;
    }
    try expect(@abs(character.getPosition().getX()) < 1.0e-3);
    try expect(y > 0.5);
    try expect(@abs(character.getPosition().getZ()) < 1.0e-3);

    // When moving along the to sloped floors, check that we are not stuck and fall back on the floor when we've passed them
    character.horizontal_speed = Vec3.axisX().mulScalar(2.0);
    const num_steps = 60; // 1 second
    const starting_position = character.getPosition();
    for (0..num_steps) |_|
        try character.step();
    var expected_position = starting_position.addVec3(character.horizontal_speed.mulScalar(num_steps).mulScalar(c.getDeltaTime()));
    expected_position.setY(0);
    try checkApproxEqual(character.getPosition(), expected_position, .{ .tolerance = 0.05 });
}

test "TestCharacterVsCharacter" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const floor_id = (try c.createFloor()).getID();

    // Create characters with different radii and padding
    var character1 = Character.init(&c);
    defer character1.deinit();
    character1.initial_position = RVec3.zero();
    character1.radius_standing = 0.2;
    character1.character_settings.character_padding = 0.04;
    try character1.create();

    var character2 = Character.init(&c);
    defer character2.deinit();
    character2.initial_position = RVec3.init(1, 0, 0);
    character2.radius_standing = 0.3;
    character2.character_settings.character_padding = 0.03;
    try character2.create();

    // Make both collide
    try character1.character_vs_character.add(character2.character.?);
    try character2.character_vs_character.add(character1.character.?);

    // Add a box behind character 2, we should never hit this
    const box_extent = Vec3.init(0.1, 1.0, 1.0);
    _ = try c.createBox(RVec3.init(1.5, 0, 0), Quat.identity(), .static, .discrete, Layers.non_moving, box_extent, .{ .activation = .dont_activate });

    // Move character 1 towards character 2 so that in 1 step it will hit both character 2 and the box
    character1.horizontal_speed = Vec3.init(600.0, 0, 0);
    try character1.step();

    // Character 1 should have stopped at character 2
    const character1_radius = character1.radius_standing + character1.character_settings.character_padding;
    const character2_radius = character2.radius_standing + character2.character_settings.character_padding;
    const separation = character1_radius + character2_radius;
    const expected_colliding_with_character = character2.initial_position.subVec3(Vec3.init(separation, 0, 0));
    try checkApproxEqual(character1.getPosition(), expected_colliding_with_character, .{});
    try expect(character1.getNumContacts() == 2);
    try expect(character1.hasCollidedWith(floor_id));
    try expect(character1.hasCollidedWithCharacter(character2.character.?));

    // Move character 1 back to its initial position
    character1.character.?.setPosition(character1.initial_position);
    character1.character.?.setLinearVelocity(Vec3.zero());

    // Now move slowly so that we will detect the collision during the normal collide shape step
    character1.horizontal_speed = Vec3.init(1.0, 0, 0);
    try character1.step();
    try expect(character1.getNumContacts() == 1);
    try expect(character1.hasCollidedWith(floor_id));
    try character1.simulate(1.0);

    // Character 1 should have stopped at character 2
    try checkApproxEqual(character1.getPosition(), expected_colliding_with_character, .{});
    try expect(character1.getNumContacts() == 2);
    try expect(character1.hasCollidedWith(floor_id));
    try expect(character1.hasCollidedWithCharacter(character2.character.?));

    // Move character 1 back to its initial position
    character1.character.?.setPosition(character1.initial_position);
    character1.character.?.setLinearVelocity(Vec3.zero());

    // Add a box in between the characters
    const box_position = RVec3.init(0.5, 0, 0);
    const box_id = (try c.createBox(box_position, Quat.identity(), .static, .discrete, Layers.non_moving, box_extent, .{ .activation = .dont_activate })).getID();

    // Move character 1 so that it will step through both the box and the character in 1 time step
    character1.horizontal_speed = Vec3.init(600.0, 0, 0);
    try character1.step();

    // Expect that it ends up at the box
    const expected_colliding_with_box = box_position.subVec3(Vec3.init(character1_radius + box_extent.getX(), 0, 0));
    try checkApproxEqual(character1.getPosition(), expected_colliding_with_box, .{});
    try expect(character1.getNumContacts() == 2);
    try expect(character1.hasCollidedWith(floor_id));
    try expect(character1.hasCollidedWith(box_id));

    // Move character 1 back to its initial position
    character1.character.?.setPosition(character1.initial_position);
    character1.character.?.setLinearVelocity(Vec3.zero());

    // Now move slowly so that we will detect the collision during the normal collide shape step
    character1.horizontal_speed = Vec3.init(1.0, 0, 0);
    try character1.step();
    try expect(character1.getNumContacts() == 1);
    try expect(character1.hasCollidedWith(floor_id));
    try character1.simulate(1.0);

    // Expect that it ends up at the box
    try checkApproxEqual(character1.getPosition(), expected_colliding_with_box, .{});
    try expect(character1.getNumContacts() == 2);
    try expect(character1.hasCollidedWith(floor_id));
    try expect(character1.hasCollidedWith(box_id));
}
