//! Port of: UnitTests/Physics/HeightFieldShapeTests.cpp
//! Status: complete
//!
//! - `uniform_int_distribution<uint>` is `uniformUint` (libstdc++'s algorithm on UnitTestRandom, like
//!   `fw.UniformFloatDistribution`).
//! - `Ref<HeightFieldShape>` is a `RefConst(Shape)` / `Ref(Shape)` plus `cast(HeightFieldShape)` (`castMut` for the
//!   functions that modify the shape: SetHeights, SetMaterials).
//! - `TempAllocatorMalloc temp_allocator` is `zolt.TempAllocatorMalloc.init(allocator)`.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const Color = zolt.Color;
const HeightFieldShape = zolt.HeightFieldShape;
const HeightFieldShapeConstants = zolt.HeightFieldShapeConstants;
const HeightFieldShapeSettings = zolt.HeightFieldShapeSettings;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialList = zolt.PhysicsMaterialList;
const PhysicsMaterialRefC = zolt.PhysicsMaterialRefC;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TempAllocatorMalloc = zolt.TempAllocatorMalloc;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

/// std::uniform_int_distribution<uint>(a, b) on UnitTestRandom, reproducing libstdc++ (Lemire's nearly divisionless
/// method on one 32 bit draw, with rejection)
fn uniformUint(random: *fw.UnitTestRandom, a: u32, b: u32) u32 {
    const range: u32 = b - a + 1;
    var product: u64 = @as(u64, random.next() - fw.UnitTestRandom.min_value) * range;
    var low: u32 = @truncate(product);
    if (low < range) {
        const threshold = (0 -% range) % range;
        while (low < threshold) {
            product = @as(u64, random.next() - fw.UnitTestRandom.min_value) * range;
            low = @truncate(product);
        }
    }
    return a + @as(u32, @intCast(product >> 32));
}

/// new PhysicsMaterialSimple("Material " + ConvertToString(index), Color::sGetDistinctColor(index)) (reference count 0)
fn createMaterial(index: u32) !*const PhysicsMaterial {
    const name = try std.fmt.allocPrint(allocator, "Material {d}", .{index});
    defer allocator.free(name);
    const material = try PhysicsMaterialSimple.create(allocator, name, Color.getDistinctColor(@intCast(index)));
    return material.material();
}

fn randomizeMaterials(settings: *HeightFieldShapeSettings, max_materials: u32) !void {
    // Create materials
    for (0..max_materials) |i| {
        const material = try createMaterial(@intCast(i));
        try settings.materials.ensureUnusedCapacity(allocator, 1);
        settings.materials.appendAssumeCapacity(.init(material));
    }

    if (max_materials > 1) {
        // Make random material indices
        var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
        const count_min_1 = settings.sample_count - 1;
        try settings.material_indices.resize(allocator, count_min_1 * count_min_1);
        for (0..count_min_1) |y|
            for (0..count_min_1) |x| {
                settings.material_indices.items[y * count_min_1 + x] = @intCast(uniformUint(&random, 0, max_materials - 1));
            };
    }
}

/// Creates the shape and checks every sample (sValidateGetPosition), returns a reference to the shape
fn validateGetPosition(settings: *HeightFieldShapeSettings, max_error: f32) !RefConst(Shape) {
    // Create shape
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    var shape_ref = RefConst(Shape).init(result.getPtr());
    errdefer shape_ref.deinit();
    const shape = shape_ref.get().?.cast(HeightFieldShape);

    // Validate it
    var max_diff: f32 = -1.0;
    for (0..settings.sample_count) |yi| {
        for (0..settings.sample_count) |xi| {
            const x: u32 = @intCast(xi);
            const y: u32 = @intCast(yi);

            // Perform a raycast from above the height field on this location
            const ray = RayCast.init(settings.offset.add(settings.scale.mul(Vec3.init(@floatFromInt(x), 100.0, @floatFromInt(y)))), Vec3.init(0, -200, 0).mulScalar(settings.scale.getY()));
            var hit: RayCastResult = .{};
            _ = shape.asShape().castRay(ray, .{}, &hit);

            // Get original (unscaled) height
            const height = settings.height_samples.items[y * settings.sample_count + x];
            if (height != HeightFieldShapeConstants.no_collision_value) {
                // Check there is collision
                try fw.expect(!shape.isNoCollision(x, y));

                // Calculate position
                const original_pos = settings.offset.add(settings.scale.mul(Vec3.init(@floatFromInt(x), height, @floatFromInt(y))));

                // Calculate position from the shape
                const shape_pos = shape.getPosition(x, y);

                // Calculate delta
                const diff = original_pos.sub(shape_pos).length();
                max_diff = math.max(max_diff, diff);

                // Materials are defined on the triangle, not on the sample points
                if (x < settings.sample_count - 1 and y < settings.sample_count - 1) {
                    var m1: *const PhysicsMaterial = PhysicsMaterial.default;
                    if (settings.material_indices.items.len != 0)
                        m1 = settings.materials.items[settings.material_indices.items[y * (settings.sample_count - 1) + x]].get().?
                    else if (settings.materials.items.len != 0)
                        m1 = settings.materials.items[0].get().?;

                    const m2 = shape.getMaterialAt(x, y);
                    try fw.expect(m1 == m2);
                }

                // Don't test borders, the ray may or may not hit
                if (x > 0 and y > 0 and x < settings.sample_count - 1 and y < settings.sample_count - 1) {
                    // Check that the ray hit the height field
                    const hit_pos = ray.getPointOnRay(hit.fraction);
                    try fw.checkApproxEqual(hit_pos, shape_pos, .{ .tolerance = 1.0e-3 });
                }
            } else {
                // Should be no collision here
                try fw.expect(shape.isNoCollision(x, y));

                // Ray should not have given a hit
                try fw.expect(hit.fraction > 1.0);
            }
        }
    }

    // Check error
    try fw.expect(max_diff <= max_error);

    return shape_ref;
}

test "TestPlane" {
    // Create flat plane with offset and scale
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.offset = Vec3.init(3, 5, 7);
    settings.scale = Vec3.init(9, 13, 17);
    settings.sample_count = 32;
    settings.bits_per_sample = 1;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, settings.sample_count * settings.sample_count);
    for (settings.height_samples.items) |*h|
        h.* = 1.0;

    // Make some random holes
    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    for (0..10) |_|
        settings.height_samples.items[uniformUint(&random, 0, @as(u32, @intCast(settings.height_samples.items.len)) - 1)] = HeightFieldShapeConstants.no_collision_value;

    // We should be able to encode a flat plane in 1 bit
    try fw.expect(settings.calculateBitsPerSampleForError(0.0) == 1);

    try randomizeMaterials(&settings, 256);
    var shape = try validateGetPosition(&settings, 0.0);
    shape.deinit();
}

test "TestPlaneCloseToOrigin" {
    // Create flat plane very close to origin, this tests that we don't introduce a quantization error on a flat plane
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.sample_count = 32;
    settings.bits_per_sample = 1;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, settings.sample_count * settings.sample_count);
    for (settings.height_samples.items) |*h|
        h.* = 1.0e-6;

    // We should be able to encode a flat plane in 1 bit
    try fw.expect(settings.calculateBitsPerSampleForError(0.0) == 1);

    try randomizeMaterials(&settings, 50);
    var shape = try validateGetPosition(&settings, 0.0);
    shape.deinit();
}

test "TestRandomHeightField" {
    const min_height: f32 = -5.0;
    const max_height: f32 = 10.0;

    // Check if bits per sample is ok
    var bits_per_sample: u32 = 1;
    while (bits_per_sample <= HeightFieldShapeConstants.max_bits_per_sample) : (bits_per_sample += 1) {
        var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
        const height_distribution = fw.UniformFloatDistribution.init(min_height, max_height);

        // Create height field with random samples
        var settings = HeightFieldShapeSettings.initDefault(allocator);
        defer settings.deinit();
        settings.offset = Vec3.init(0.3, 0.5, 0.7);
        settings.scale = Vec3.init(1.1, 1.2, 1.3);
        settings.sample_count = 32;
        settings.bits_per_sample = bits_per_sample;
        settings.block_size = 4;
        try settings.height_samples.resize(allocator, settings.sample_count * settings.sample_count);
        for (settings.height_samples.items) |*h|
            h.* = height_distribution.next(&random);

        // Calculate maximum error you can get if you quantize using bits_per_sample.
        // We ignore the fact that we have range blocks that give much better compression, although
        // with random input data there shouldn't be much benefit of that.
        const max_error = 0.5 * (max_height - min_height) / @as(f32, @floatFromInt((@as(u32, 1) << @intCast(bits_per_sample)) - 1));
        const calculated_bits_per_sample = settings.calculateBitsPerSampleForError(max_error);
        try fw.expect(calculated_bits_per_sample <= bits_per_sample);

        try randomizeMaterials(&settings, 1);
        var shape = try validateGetPosition(&settings, settings.scale.getY() * (max_height - min_height) / @as(f32, @floatFromInt((@as(u32, 1) << @intCast(settings.bits_per_sample)) - 1)));
        shape.deinit();
    }
}

test "TestEmptyHeightField" {
    // Create height field with no collision
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.sample_count = 32;
    try settings.height_samples.resize(allocator, settings.sample_count * settings.sample_count);
    for (settings.height_samples.items) |*h|
        h.* = HeightFieldShapeConstants.no_collision_value;

    // This should use the minimum amount of bits
    try fw.expect(settings.calculateBitsPerSampleForError(0.0) == 1);

    try randomizeMaterials(&settings, 50);
    var shape = try validateGetPosition(&settings, 0.0);
    defer shape.deinit();

    // Check that we allocated the minimum amount of memory
    const stats = shape.get().?.getStats();
    try fw.expect(stats.num_triangles == 0);
    try fw.expect(stats.size_bytes == @sizeOf(HeightFieldShape));
}

test "TestGetHeights" {
    const min_height: f32 = -5.0;
    const max_height: f32 = 10.0;
    const sample_count: u32 = 32;
    const no_collision_index: u32 = 10;

    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    const height_distribution = fw.UniformFloatDistribution.init(min_height, max_height);

    // Create height field with random samples
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.offset = Vec3.init(0.3, 0.5, 0.7);
    settings.scale = Vec3.init(1.1, 1.2, 1.3);
    settings.sample_count = sample_count;
    settings.bits_per_sample = 8;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, sample_count * sample_count);
    for (settings.height_samples.items) |*h|
        h.* = height_distribution.next(&random);

    // Add 1 sample that has no collision
    settings.height_samples.items[no_collision_index] = HeightFieldShapeConstants.no_collision_value;

    // Create shape
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const height_field = result.get().get().?.cast(HeightFieldShape);

    {
        // Check that the GetHeights function returns the same values as the original height samples
        const sampled_heights = try allocator.alloc(f32, sample_count * sample_count);
        defer allocator.free(sampled_heights);
        height_field.getHeights(0, 0, sample_count, sample_count, sampled_heights.ptr, sample_count);
        for (0..sample_count * sample_count) |i| {
            if (i == no_collision_index)
                try fw.expect(sampled_heights[i] == HeightFieldShapeConstants.no_collision_value)
            else
                try fw.checkApproxEqual(sampled_heights[i], settings.offset.getY() + settings.scale.getY() * settings.height_samples.items[i], .{ .tolerance = 0.05 });
        }
    }

    {
        // With a random height field the max error is going to be limited by the amount of bits we have per sample as we will not get any benefit from a reduced range per block
        const tolerance = (max_height - min_height) / @as(f32, @floatFromInt((@as(u32, 1) << @intCast(settings.bits_per_sample)) - 2));

        // Check a sub rect of the height field
        const sx: u32 = 4;
        const sy: u32 = 8;
        const cx: u32 = 16;
        const cy: u32 = 8;
        const sampled_heights = try allocator.alloc(f32, cx * cy);
        defer allocator.free(sampled_heights);
        height_field.getHeights(sx, sy, cx, cy, sampled_heights.ptr, cx);
        for (0..cy) |y|
            for (0..cx) |x| {
                try fw.checkApproxEqual(sampled_heights[y * cx + x], settings.offset.getY() + settings.scale.getY() * settings.height_samples.items[(sy + y) * sample_count + sx + x], .{ .tolerance = tolerance });
            };
    }
}

test "TestSetHeights" {
    const min_height: f32 = -5.0;
    const max_height: f32 = 10.0;
    const sample_count: u32 = 32;

    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    const height_distribution = fw.UniformFloatDistribution.init(min_height, max_height);

    // Create height field with random samples
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.offset = Vec3.init(0.3, 0.5, 0.7);
    settings.scale = Vec3.init(1.1, 1.2, 1.3);
    settings.sample_count = sample_count;
    settings.bits_per_sample = 8;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, sample_count * sample_count);
    settings.min_height_value = min_height;
    settings.max_height_value = max_height;
    for (settings.height_samples.items) |*h|
        h.* = height_distribution.next(&random);

    // Create shape
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const height_field = result.getPtr().?.castMut(HeightFieldShape);

    // Get the original (quantized) heights
    const original_heights = try allocator.alloc(f32, sample_count * sample_count);
    defer allocator.free(original_heights);
    height_field.getHeights(0, 0, sample_count, sample_count, original_heights.ptr, sample_count);

    // Create new data for height field
    const sx: u32 = 4;
    const sy: u32 = 16;
    const cx: u32 = 16;
    const cy: u32 = 8;
    const patched_heights = try allocator.alloc(f32, cx * cy);
    defer allocator.free(patched_heights);
    for (0..cy) |y|
        for (0..cx) |x| {
            patched_heights[y * cx + x] = height_distribution.next(&random);
        };

    // Add 1 sample that has no collision
    const no_collision_idx = (sy + 1) * sample_count + sx + 2;
    patched_heights[1 * cx + 2] = HeightFieldShapeConstants.no_collision_value;

    // Update the height field
    var temp_allocator = TempAllocatorMalloc.init(allocator);
    try height_field.setHeights(sx, sy, cx, cy, patched_heights.ptr, cx, temp_allocator.tempAllocator(), .{});

    // With a random height field the max error is going to be limited by the amount of bits we have per sample as we will not get any benefit from a reduced range per block
    const tolerance = (max_height - min_height) / @as(f32, @floatFromInt((@as(u32, 1) << @intCast(settings.bits_per_sample)) - 2));

    // Check a sub rect of the height field
    const verify_heights = try allocator.alloc(f32, sample_count * sample_count);
    defer allocator.free(verify_heights);
    height_field.getHeights(0, 0, sample_count, sample_count, verify_heights.ptr, sample_count);
    for (0..sample_count) |y|
        for (0..sample_count) |x| {
            const idx = y * sample_count + x;
            if (idx == no_collision_idx)
                try fw.expect(verify_heights[idx] == HeightFieldShapeConstants.no_collision_value)
            else if (x >= sx and x < sx + cx and y >= sy and y < sy + cy)
                try fw.checkApproxEqual(verify_heights[y * sample_count + x], patched_heights[(y - sy) * cx + x - sx], .{ .tolerance = tolerance })
            else if (x >= sx - settings.block_size and x < sx + cx and y >= sy - settings.block_size and y < sy + cy)
                try fw.checkApproxEqual(verify_heights[idx], original_heights[idx], .{ .tolerance = tolerance }) // We didn't modify this but it has been quantized again
            else
                try fw.expect(verify_heights[idx] == original_heights[idx]); // We didn't modify this and it is outside of the affected range
        };
}

test "TestSetHeightsNonPowerOf2BlockCount" {
    // 20 x 20 samples with a block size of 2 gives 10 blocks per side, which is not a power of 2.
    // The coarser levels of the range block hierarchy are stored at stride 1 << level, only the most
    // detailed level is stored at stride (num_blocks + 1) / 2. SetHeights must update them at the same
    // strides, otherwise the updated ranges land on unrelated nodes and the cells below those nodes
    // stop colliding until something big enough overlaps the wrong range.
    const sample_count: u32 = 20;

    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.sample_count = sample_count;
    settings.block_size = 2;
    settings.min_height_value = 0.0;
    settings.max_height_value = 200.0;
    try settings.height_samples.appendNTimes(allocator, 0.0, sample_count * sample_count);

    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?;
    const height_field = shape.castMut(HeightFieldShape);

    // Raise a block aligned rectangle in one corner of the height field
    const sx: u32 = 0;
    const sy: u32 = 16;
    const cx: u32 = 6;
    const cy: u32 = 4;
    const raised_height: f32 = 100.0;
    var raised_heights: [cx * cy]f32 = @splat(raised_height);
    var temp_allocator = TempAllocatorMalloc.init(allocator);
    try height_field.setHeights(sx, sy, cx, cy, &raised_heights, cx, temp_allocator.tempAllocator(), .{});

    // Heights as the shape quantized them, so the sphere below can be placed on the actual surface
    var heights: [sample_count * sample_count]f32 = undefined;
    height_field.getHeights(0, 0, sample_count, sample_count, &heights, sample_count);

    // A sphere that dips 0.25 below the highest corner of a cell must collide with that cell.
    // Its bounding box is 1 high, so it only overlaps the range of the node it is really under.
    const sphere_radius: f32 = 0.5;
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, sphere_radius, .{})).asShape());
    defer sphere.deinit();
    const collide_settings: CollideShapeSettings = .{};
    const TestCell = struct {
        fn hasHit(s: *const Shape, hf: *const Shape, h: *const [sample_count * sample_count]f32, settings_ptr: *const CollideShapeSettings, x: u32, y: u32) !bool {
            const top = math.max(math.max(h[y * sample_count + x], h[y * sample_count + x + 1]), math.max(h[(y + 1) * sample_count + x], h[(y + 1) * sample_count + x + 1]));
            const sphere_position = Vec3.init(@as(f32, @floatFromInt(x)) + 0.5, top + sphere_radius - 0.25, @as(f32, @floatFromInt(y)) + 0.5);
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            CollisionDispatch.collideShapeVsShape(s, hf, Vec3.one(), Vec3.one(), Mat44.translation(sphere_position), Mat44.identity(), .{}, .{}, settings_ptr, &collector.base, &.{});
            try collector.checkError();
            return collector.hadHit();
        }
    };

    // Every cell that was not raised must still collide
    var y: u32 = 0;
    while (y + 1 < sample_count) : (y += 1) {
        var x: u32 = 0;
        while (x + 1 < sample_count) : (x += 1) {
            // Skip the raised rectangle and the cells that connect to it
            if (x < sx + cx and y + 1 >= sy)
                continue;

            if (!try TestCell.hasHit(sphere.get().?, shape, &heights, &collide_settings, x, y)) {
                std.debug.print("TestSetHeightsNonPowerOf2BlockCount: no collision at x = {d}, y = {d}\n", .{ x, y });
                return error.TestUnexpectedResult;
            }
        }
    }

    // And the raised part must collide at its new height
    try fw.expect(try TestCell.hasHit(sphere.get().?, shape, &heights, &collide_settings, sx + 1, sy + 1));
}

test "TestSetMaterials" {
    const sample_count: u32 = 32;

    var materials: [6]PhysicsMaterialRefC = undefined;
    for (&materials, 0..) |*m, i|
        m.* = .init(try createMaterial(@intCast(i)));
    defer for (&materials) |*m| m.deinit();
    const material_0 = materials[0].get().?;

    // Create height field with a single material
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    settings.sample_count = sample_count;
    settings.bits_per_sample = 8;
    settings.block_size = 4;
    try settings.height_samples.resize(allocator, sample_count * sample_count);
    for (settings.height_samples.items) |*h|
        h.* = 0.0;
    try settings.materials.append(allocator, materials[0].clone());
    try settings.material_indices.resize(allocator, (sample_count - 1) * (sample_count - 1));
    for (settings.material_indices.items) |*m|
        m.* = 0;

    // Store the current state
    var current_state: [(sample_count - 1) * (sample_count - 1)]*const PhysicsMaterial = @splat(material_0);

    // Create shape
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const height_field = result.getPtr().?.castMut(HeightFieldShape);

    const Helpers = struct {
        // Check that the material is set
        fn checkMaterials(hf: *const HeightFieldShape, state: []const *const PhysicsMaterial) !void {
            const material_list = hf.getMaterialList();

            const sample_count_min_1 = hf.getSampleCount() - 1;

            const material_indices = try allocator.alloc(u8, sample_count_min_1 * sample_count_min_1);
            defer allocator.free(material_indices);
            hf.getMaterials(0, 0, sample_count_min_1, sample_count_min_1, material_indices.ptr, sample_count_min_1);

            for (state, 0..) |m, i|
                try fw.expect(m == material_list[material_indices[i]].get().?);
        }

        // Function to randomize materials
        fn updateMaterials(hf: *HeightFieldShape, state: []*const PhysicsMaterial, start_x: u32, start_y: u32, size_x: u32, size_y: u32, material_list_in: ?[]const PhysicsMaterialRefC) !void {
            var temp_allocator = TempAllocatorMalloc.init(allocator);

            const material_list = material_list_in orelse hf.getMaterialList();

            var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);

            const sample_count_min_1 = hf.getSampleCount() - 1;

            const patched_materials = try allocator.alloc(u8, size_x * size_y);
            defer allocator.free(patched_materials);
            for (0..size_y) |y|
                for (0..size_x) |x| {
                    // Initialize the patch
                    const index: u8 = @intCast(uniformUint(&random, 0, @as(u32, @intCast(material_list.len)) - 1));
                    patched_materials[y * size_x + x] = index;

                    // Update reference state
                    state[(start_y + y) * sample_count_min_1 + start_x + x] = material_list[index].get().?;
                };
            try fw.expect(try hf.setMaterials(start_x, start_y, size_x, size_y, patched_materials.ptr, size_x, material_list_in, temp_allocator.tempAllocator()));
        }
    };
    try Helpers.checkMaterials(height_field, &current_state);

    {
        // Add material 1
        const patched_materials_list = [_]PhysicsMaterialRefC{ materials[1], materials[0] };
        try Helpers.updateMaterials(height_field, &current_state, 4, 16, 16, 8, &patched_materials_list);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    {
        // Add material 2
        const patched_materials_list = [_]PhysicsMaterialRefC{ materials[0], materials[2] };
        try Helpers.updateMaterials(height_field, &current_state, 8, 16, 16, 8, &patched_materials_list);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    {
        // Add material 3
        const patched_materials_list = [_]PhysicsMaterialRefC{ materials[0], materials[1], materials[2], materials[3] };
        try Helpers.updateMaterials(height_field, &current_state, 8, 8, 16, 8, &patched_materials_list);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    {
        // Add material 4
        const patched_materials_list = [_]PhysicsMaterialRefC{ materials[0], materials[1], materials[4], materials[2], materials[3] };
        try Helpers.updateMaterials(height_field, &current_state, 0, 0, 30, 30, &patched_materials_list);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    {
        // Add material 5
        const patched_materials_list = [_]PhysicsMaterialRefC{ materials[4], materials[3], materials[0], materials[1], materials[2], materials[5] };
        try Helpers.updateMaterials(height_field, &current_state, 1, 1, 30, 30, &patched_materials_list);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    {
        // Update materials without new material list
        try Helpers.updateMaterials(height_field, &current_state, 2, 5, 10, 15, null);
        try Helpers.checkMaterials(height_field, &current_state);
    }

    // Check materials using GetMaterial call
    for (0..sample_count - 1) |y|
        for (0..sample_count - 1) |x| {
            try fw.expect(height_field.getMaterialAt(@intCast(x), @intCast(y)) == current_state[y * (sample_count - 1) + x]);
        };
}
