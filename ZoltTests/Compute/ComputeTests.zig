//! Port of: UnitTests/Compute/ComputeTests.cpp
//! Status: complete
//! Not ported: TestComputeDX12, TestComputeMTL, TestComputeVK (the GPU backends stay deferred, Docs/Zolt/RemainingFeaturesArchitecture.md R21); the shader loader lambda of RunTests (only the GPU backends call mShaderLoader, R9)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const Allocator = std.mem.Allocator;
const ComputeBuffer = zolt.ComputeBuffer;
const ComputeSystem = zolt.ComputeSystem;
const ComputeSystemCPU = zolt.ComputeSystemCPU;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RTTI = zolt.RTTI;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const hlsl = zolt.HLSLToCPP;
const TestCompute = zolt.TestCompute;
const TestCompute2 = zolt.TestCompute2;
const TestComputeContext = TestCompute.TestComputeContext;
const TestCompute2Input = TestCompute2.TestCompute2Input;
const TestCompute2Output = TestCompute2.TestCompute2Output;
const JPH_float3 = zolt.ShaderCore.JPH_float3;

const expect = fw.expect;
const expectEqual = fw.expectEqual;

fn runTests(compute_system: *ComputeSystem) !void {
    // inComputeSystem->mShaderLoader = ...: only the GPU backends load shaders (not ported)

    // Create a queue
    var queue = try compute_system.createComputeQueue();
    defer queue.deinit();
    try expect(queue.get() != null);

    // Test failing shader creation
    {
        const shader_result = compute_system.createComputeShader("NonExistingShader", 64, .{});
        try std.testing.expectError(error.ShaderNotFound, shader_result);
    }

    {
        const num_elements: u32 = 1234; // Not a multiple of cTestComputeGroupSize
        const num_iterations: u32 = 10;
        const float3_value: JPH_float3 = .init(0, 0, 0);
        const float3_value2: JPH_float3 = .init(0, 13, 0);
        const uint_value: u32 = 7;
        const upload_value: u32 = 42;

        // Can't change context buffer while commands are queued, so create multiple constant buffers
        var context: [num_iterations]Ref(ComputeBuffer) = @splat(.empty);
        defer for (&context) |*c| c.deinit();
        for (&context) |*c| {
            c.* = try compute_system.createComputeBuffer(.constant_buffer, 1, @sizeOf(TestComputeContext), null);
            try expect(c.get() != null);
        }

        // Create an upload buffer
        var upload_buffer = try compute_system.createComputeBuffer(.upload_buffer, 1, @sizeOf(u32), null);
        defer upload_buffer.deinit();
        try expect(upload_buffer.get() != null);
        const upload_data = upload_buffer.get().?.mapAs(u32, .write);
        upload_data[0] = upload_value;
        upload_buffer.get().?.unmap();

        // Create a read buffer
        var rnd = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
        var optional_data: [num_elements]u32 = undefined;
        for (&optional_data) |*d|
            d.* = rnd.next();
        var optional_buffer = try compute_system.createComputeBuffer(.buffer, num_elements, @sizeOf(u32), std.mem.sliceAsBytes(&optional_data));
        defer optional_buffer.deinit();
        try expect(optional_buffer.get() != null);

        // Create a read-write buffer
        var buffer = try compute_system.createComputeBuffer(.rw_buffer, num_elements, @sizeOf(u32), null);
        defer buffer.deinit();
        try expect(buffer.get() != null);

        // Create a read back buffer
        var readback_buffer = try buffer.get().?.createReadBackBuffer();
        defer readback_buffer.deinit();
        try expect(readback_buffer.get() != null);

        // Create the shader
        var shader = try compute_system.createComputeShader("TestCompute", TestCompute.test_compute_group_size, .{});
        defer shader.deinit();
        try expect(shader.get() != null);

        // Schedule work
        const q = queue.get().?;
        for (0..num_iterations) |iter| {
            // Fill in the context
            const value = &context[iter].get().?.mapAs(TestComputeContext, .write)[0];
            value.float3_value = float3_value;
            value.uint_value = uint_value;
            value.float3_value2 = float3_value2;
            value.uint_value2 = @intCast(iter);
            value.num_elements = num_elements;
            context[iter].get().?.unmap();

            try q.setShader(shader.get().?);
            try q.setConstantBuffer("gContext", context[iter].get());
            context[iter].deinit(); // Release the reference to ensure the queue keeps ownership
            try q.setBuffer("gOptionalData", optional_buffer.get());
            optional_buffer.deinit(); // Release the reference so we test that the queue keeps ownership and that in the 2nd iteration we can set a null buffer
            try q.setBuffer("gUploadData", upload_buffer.get());
            try q.setRWBuffer("gData", buffer.get(), .{});
            q.dispatch((num_elements + TestCompute.test_compute_group_size - 1) / TestCompute.test_compute_group_size, .{});
        }

        // Run all queued commands
        q.scheduleReadback(readback_buffer.get().?, buffer.get().?);
        q.executeAndWait();

        // Calculate the expected result
        var expected_data: [num_elements]u32 = undefined;
        for (0..num_iterations) |iter| {
            // Copy of the shader logic
            const uint_value2: u32 = @intCast(iter);
            if (uint_value2 == 0) {
                // First write, uses optional data and tests that the packing of float3/uint3's works
                for (0..num_elements) |i|
                    expected_data[i] = optional_data[i] +% @as(u32, @bitCast(@as(i32, @intFromFloat(float3_value2.y)))) +% upload_value;
            } else {
                // Read-modify-write gData
                for (0..num_elements) |i|
                    expected_data[i] = (expected_data[i] +% uint_value) *% uint_value2;
            }
        }

        // Compare computed data with expected data
        const data = readback_buffer.get().?.mapAs(u32, .read);
        for (0..num_elements) |i|
            try expect(data[i] == expected_data[i]);
        readback_buffer.get().?.unmap();
    }

    // Test helper functions
    {
        // Create the shader
        var shader = try compute_system.createComputeShader("TestCompute2", TestCompute2.test_compute2_group_size, .{});
        defer shader.deinit();
        try expect(shader.get() != null);

        const mat44_value: Mat44 = .init(.init(2, 3, 5, 0), .init(7, 11, 13, 0), .init(13, 15, 17, 0), .init(17, 19, 23, 0));
        const mat44_mul_value: Vec3 = .init(29, 31, 37);

        const decompressed_vec3 = Vec3.init(-2, 3, -5).normalized();
        const compressed_vec3 = decompressed_vec3.compressUnitVector();

        const decompressed_quat = Quat.fromVec4(Vec4.init(2, -3, 5, -7).normalized());
        const compressed_quat = decompressed_quat.compressUnitQuat();

        // Generate input data
        var input: TestCompute2Input = undefined;
        var columns: [4]zolt.Float4 = undefined;
        mat44_value.storeFloat4x4(&columns);
        for (&input.mat44_value.c, columns) |*c, f|
            c.* = .init(f.x, f.y, f.z, f.w);
        var mul_value: zolt.Float3 = undefined;
        mat44_mul_value.storeFloat3(&mul_value);
        input.mat44_mul_value = .init(mul_value.x, mul_value.y, mul_value.z);
        input.compressed_vec3 = compressed_vec3;
        input.compressed_quat = compressed_quat;

        // Create input buffer
        var input_buffer = try compute_system.createComputeBuffer(.buffer, 1, @sizeOf(TestCompute2Input), std.mem.asBytes(&input));
        defer input_buffer.deinit();

        // Create a read-write buffer for the output
        var output_buffer = try compute_system.createComputeBuffer(.rw_buffer, 1, @sizeOf(TestCompute2Output), null);
        defer output_buffer.deinit();
        try expect(output_buffer.get() != null);

        // Create a read back buffer
        var readback_buffer = try output_buffer.get().?.createReadBackBuffer();
        defer readback_buffer.deinit();
        try expect(readback_buffer.get() != null);

        // Execute the shader
        const q = queue.get().?;
        try q.setShader(shader.get().?);
        try q.setBuffer("gInput", input_buffer.get());
        try q.setRWBuffer("gOutput", output_buffer.get(), .{});
        q.dispatch(1, .{});
        q.scheduleReadback(readback_buffer.get().?, output_buffer.get().?);
        q.executeAndWait();

        // Verify the output
        const output = &readback_buffer.get().?.mapAs(TestCompute2Output, .read)[0];

        const expected_mul3x4 = mat44_value.mulVec3(mat44_mul_value);
        try expect(toVec3(output.mul3x4_output).eql(expected_mul3x4));

        const expected_mul3x3 = mat44_value.multiply3x3(mat44_mul_value);
        try expect(toVec3(output.mul3x3_output).eql(expected_mul3x3));

        const expected_decompressed_vec3 = Vec3.decompressUnitVector(compressed_vec3);
        try expect(toVec3(output.decompressed_vec3).isClose(expected_decompressed_vec3, .{}));

        const expected_decompressed_quat = Quat.decompressUnitQuat(compressed_quat);
        const q_out = output.decompressed_quat;
        try expect(Quat.init(q_out.x, q_out.y, q_out.z, q_out.w).isClose(expected_decompressed_quat, .{}));

        readback_buffer.get().?.unmap();
    }
}

/// Vec3(const Float3 &)
fn toVec3(v: JPH_float3) Vec3 {
    return .fromFloat3(.init(v.x, v.y, v.z));
}

test "TestComputeCPU" {
    var compute_system = try ComputeSystemCPU.create(std.testing.allocator);
    defer compute_system.deinit();
    const system = compute_system.get().?;
    try expect(compute_system.get() != null);
    try expect(RTTI.isKindOfObject(ComputeSystem, system, RTTI.of(ComputeSystem)));
    try expect(RTTI.isKindOfObject(ComputeSystem, system, RTTI.of(ComputeSystemCPU)));
    try expect(!RTTI.isKindOfObject(ComputeSystem, system, RTTI.of(zolt.ShapeSettings)));

    const system_cpu = zolt.virtual.downcast(ComputeSystemCPU, system); // StaticCast<ComputeSystemCPU>
    try zolt.registerTestCompute(system_cpu); // JPH_REGISTER_SHADER(..., TestCompute)
    try zolt.registerTestCompute2(system_cpu); // JPH_REGISTER_SHADER(..., TestCompute2)
    try runTests(system);
}

test "TestComputeCPU: allocation failures" {
    // Zolt: every allocation of the test can fail; the error must arrive and nothing may leak
    try std.testing.checkAllAllocationFailures(std.testing.allocator, struct {
        fn f(allocator: Allocator) !void {
            var compute_system = try ComputeSystemCPU.create(allocator);
            defer compute_system.deinit();
            const system_cpu = zolt.virtual.downcast(ComputeSystemCPU, compute_system.get().?);
            try zolt.registerTestCompute(system_cpu);
            try zolt.registerTestCompute2(system_cpu);
            try runTests(compute_system.get().?);
        }
    }.f, .{});
}
