//! Parity of Zolt's CPU compute backend with Jolt's ComputeSystemCPU (JPH_USE_CPU_COMPUTE, the HLSL shaders compiled as
//! C++ by TestComputeWrapper.cpp and HairWrapper.cpp). Docs/Zolt/RemainingFeaturesArchitecture.md section 4, row
//! "Compute dispatch". C++ side: ComputeReference.cpp.
//!
//! `jolt_compute_dispatch` is a generic C ABI: a shader by name, buffers by name, the RW buffers read back. `Dispatcher`
//! runs the same dispatch on both sides, each on its own copy of the buffers, and compares the RW buffers: float
//! buffers with `sameBits` (NaNs are equal: clang may commute a product's operands, which changes a NaN payload),
//! integer buffers byte for byte. The hair shader groups (C1a, C1b) add their shaders to `Dispatcher` and reuse it.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const ComputeBuffer = zolt.ComputeBuffer;
const ComputeSystem = zolt.ComputeSystem;
const ComputeSystemCPU = zolt.ComputeSystemCPU;
const Ref = zolt.Ref;
const RTTI = zolt.RTTI;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const hlsl = zolt.HLSLToCPP;
const HairStructs = zolt.HairStructs;
const TestCompute = zolt.TestCompute;
const TestCompute2 = zolt.TestCompute2;

/// One buffer of a dispatch. Must match ComputeBindingDesc in ComputeReference.cpp.
pub const BindingDesc = extern struct {
    name: [*:0]const u8,
    /// Initial contents (`size * stride` bytes); an RW buffer receives its contents after the dispatch
    data: [*]u8,
    /// Number of elements
    size: u64,
    /// Size of an element in bytes
    stride: u32,
    kind: Kind,
    /// ComputeBuffer::EType of the buffer
    type: u32,
    flags: u32 = 0,

    pub const Kind = enum(u32) { constant_buffer, buffer, rw_buffer };

    /// Bind a null buffer (the queue ignores it)
    pub const flag_null = 1;
    /// Create the buffer without data (its memory is zero, UB4)
    pub const flag_no_data = 2;

    /// A binding of `items`, with the buffer type that fits the kind (upload buffers are bound with setBuffer too)
    pub fn of(name: [*:0]const u8, kind: Kind, comptime T: type, items: []T) BindingDesc {
        const buffer_type: ComputeBuffer.Type = switch (kind) {
            .constant_buffer => .constant_buffer,
            .buffer => .buffer,
            .rw_buffer => .rw_buffer,
        };
        return .{ .name = name, .data = @ptrCast(items.ptr), .size = items.len, .stride = @sizeOf(T), .kind = kind, .type = @intFromEnum(buffer_type) };
    }

    /// The same binding with another buffer type
    pub fn withType(self: BindingDesc, buffer_type: ComputeBuffer.Type) BindingDesc {
        var result = self;
        result.type = @intFromEnum(buffer_type);
        return result;
    }

    /// The same binding with flags
    pub fn withFlags(self: BindingDesc, flags: u32) BindingDesc {
        var result = self;
        result.flags = flags;
        return result;
    }

    pub fn bytes(self: BindingDesc) []u8 {
        return self.data[0..@intCast(self.size * self.stride)];
    }
};

extern fn jolt_compute_is_kind_of(out_values: *[3]u32) void;
extern fn jolt_compute_dispatch(name: [*:0]const u8, group_size: *const [3]u32, num_groups: *const [3]u32, bindings: [*]BindingDesc, num_bindings: u32) u32;
extern fn jolt_compute_layouts(out_values: [*]u32, max_values: u32) u32;
extern fn jolt_compute_constants(out_values: *[7]c_int) void;

/// How the RW buffers of a dispatch are compared
pub const Compare = enum {
    /// Byte for byte (integer buffers)
    bytes,
    /// As 32 bit floats with `sameBits` (float buffers, also buffers that mix floats with integers)
    floats,
};

/// Zolt's side of jolt_compute_dispatch, call by call. Returns false when the shader does not exist.
fn zoltDispatch(allocator: Allocator, system: *ComputeSystem, name: []const u8, group_size: [3]u32, num_groups: [3]u32, bindings: []BindingDesc) !bool {
    var shader = system.createComputeShader(name, group_size[0], .{ .group_size_y = group_size[1], .group_size_z = group_size[2] }) catch |err| switch (err) {
        error.ShaderNotFound => return false,
        else => |e| return e,
    };
    defer shader.deinit();

    const buffers = try allocator.alloc(Ref(ComputeBuffer), bindings.len);
    defer allocator.free(buffers);
    @memset(buffers, .empty);
    defer for (buffers) |*b| b.deinit();
    for (bindings, buffers) |b, *buffer| {
        if (b.flags & BindingDesc.flag_null == 0)
            buffer.* = try system.createComputeBuffer(@enumFromInt(b.type), b.size, b.stride, if (b.flags & BindingDesc.flag_no_data != 0) null else b.bytes());
    }

    var queue = try system.createComputeQueue();
    defer queue.deinit();
    const q = queue.get().?;
    try q.setShader(shader.get().?);
    for (bindings, buffers) |b, buffer| {
        const name_slice = std.mem.span(b.name);
        switch (b.kind) {
            .constant_buffer => try q.setConstantBuffer(name_slice, buffer.get()),
            .buffer => try q.setBuffer(name_slice, buffer.get()),
            .rw_buffer => try q.setRWBuffer(name_slice, buffer.get(), .{}),
        }
    }

    // The queue holds the buffers now (like ComputeTests, which releases its references after binding)
    for (buffers) |*buffer| {
        if (buffer.get()) |b| {
            if (b.getType() != .rw_buffer) buffer.deinit();
        }
    }

    q.dispatch(num_groups[0], .{ .thread_groups_y = num_groups[1], .thread_groups_z = num_groups[2] });
    q.executeAndWait();

    // Copy the RW buffers back
    for (bindings, buffers) |b, buffer| {
        if (b.kind == .rw_buffer) {
            if (buffer.get()) |rw| {
                var readback = try rw.createReadBackBuffer();
                defer readback.deinit();
                @memcpy(b.bytes(), readback.get().?.map(.read));
                readback.get().?.unmap();
            }
        }
    }
    return true;
}

/// A Zolt compute system with the shaders of the parity, and the dispatch on both sides
pub const Dispatcher = struct {
    allocator: Allocator,
    system: Ref(ComputeSystem),

    /// Zolt's system with TestCompute and TestCompute2 (Jolt's system also has the hair shaders)
    pub fn init(allocator: Allocator) !Dispatcher {
        var system = try ComputeSystemCPU.create(allocator);
        errdefer system.deinit();
        const cpu = zolt.virtual.downcast(ComputeSystemCPU, system.get().?);
        try zolt.registerTestCompute(cpu);
        try zolt.registerTestCompute2(cpu);
        return .{ .allocator = allocator, .system = system };
    }

    pub fn deinit(self: *Dispatcher) void {
        self.system.deinit();
    }

    /// Run the dispatch on both sides, each on its own copy of the buffers. Checks that the shader exists on both sides
    /// or on neither, and that every RW buffer has the same contents. Then the RW buffers of `bindings` receive Jolt's
    /// results (a pipeline feeds them to the next dispatch). Returns false when the shader exists on neither side.
    pub fn dispatchBoth(self: *Dispatcher, checker: *fw.Checker, name: [:0]const u8, group_size: [3]u32, num_groups: [3]u32, bindings: []const BindingDesc, compare: Compare) !bool {
        const allocator = self.allocator;
        const zolt_bindings = try allocator.dupe(BindingDesc, bindings);
        defer allocator.free(zolt_bindings);
        const jolt_bindings = try allocator.dupe(BindingDesc, bindings);
        defer allocator.free(jolt_bindings);
        var copies: std.ArrayList([]u8) = .empty;
        defer {
            for (copies.items) |c| allocator.free(c);
            copies.deinit(allocator);
        }
        for (zolt_bindings, jolt_bindings) |*z, *j| {
            const zc = try allocator.dupe(u8, z.bytes());
            try copies.append(allocator, zc);
            z.data = zc.ptr;
            const jc = try allocator.dupe(u8, j.bytes());
            try copies.append(allocator, jc);
            j.data = jc.ptr;
        }

        const zolt_found = try zoltDispatch(allocator, self.system.get().?, name, group_size, num_groups, zolt_bindings);
        const jolt_found = jolt_compute_dispatch(name.ptr, &group_size, &num_groups, jolt_bindings.ptr, @intCast(jolt_bindings.len)) == 0;
        checker.check(.{ .shader = name, .what = "shader found" }, zolt_found, jolt_found);

        for (bindings, zolt_bindings, jolt_bindings) |b, z, j| {
            if (b.kind != .rw_buffer or b.flags & BindingDesc.flag_null != 0) continue;
            const first_mismatch = firstMismatch(z.bytes(), j.bytes(), compare);
            if (first_mismatch) |first|
                checker.check(.{ .shader = name, .buffer = std.mem.span(b.name), .first_mismatching_byte = first, .mismatching_words = countMismatches(z.bytes(), j.bytes(), compare) }, false, true);
            @memcpy(b.bytes(), j.bytes());
        }
        return jolt_found;
    }
};

/// The offset of the first difference, null when equal (floats: whole 32 bit words compared with sameBits)
pub fn firstMismatch(zolt_bytes: []const u8, jolt_bytes: []const u8, compare: Compare) ?usize {
    switch (compare) {
        .bytes => return std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes),
        .floats => {
            std.debug.assert(zolt_bytes.len % 4 == 0);
            var i: usize = 0;
            while (i < zolt_bytes.len) : (i += 4) {
                const z: f32 = @bitCast(zolt_bytes[i..][0..4].*);
                const j: f32 = @bitCast(jolt_bytes[i..][0..4].*);
                if (!fw.sameBits(z, j)) return i;
            }
            return null;
        },
    }
}

/// The number of mismatching 32 bit words (compared like `firstMismatch`)
pub fn countMismatches(zolt_bytes: []const u8, jolt_bytes: []const u8, compare: Compare) usize {
    var count: usize = 0;
    var i: usize = 0;
    while (i < zolt_bytes.len) : (i += 4) {
        const end = @min(i + 4, zolt_bytes.len);
        if (firstMismatch(zolt_bytes[i..end], jolt_bytes[i..end], compare) != null) count += 1;
    }
    return count;
}

/// A float that reaches the special cases: zeros, infinities, NaN, denormals, any bit pattern, values far outside the
/// integer ranges, otherwise uniform in [-range, range)
pub fn specialFloat(rng: *fw.Rng, range: f32) f32 {
    return switch (rng.next() % 32) {
        0 => 0.0,
        1 => -0.0,
        2 => std.math.inf(f32),
        3 => -std.math.inf(f32),
        4 => std.math.nan(f32),
        5 => @bitCast(rng.next() & 0x807fffff), // Denormal
        6 => @bitCast(rng.next()), // Any bit pattern
        7 => rng.float(-1.0e10, 1.0e10),
        8 => @floatFromInt(@as(i32, @bitCast(rng.next()))), // An integer
        else => rng.float(-range, range),
    };
}

/// A float for vector inputs: uniform in [-range, range), one in eight is a specialFloat. With specialFloat on every
/// component nearly every vector would hold a special value, which hides the rounding of the other components.
pub fn inputFloat(rng: *fw.Rng, range: f32) f32 {
    return if (rng.next() % 8 == 0) specialFloat(rng, range) else rng.float(-range, range);
}

test "ComputeParity: RTTI of the CPU compute system" {
    var jolt: [3]u32 = undefined;
    jolt_compute_is_kind_of(&jolt);

    var system = try ComputeSystemCPU.create(std.testing.allocator);
    defer system.deinit();
    const s = system.get().?;
    const zolt_values: [3]u32 = .{
        @intFromBool(RTTI.isKindOfObject(ComputeSystem, s, RTTI.of(ComputeSystem))),
        @intFromBool(RTTI.isKindOfObject(ComputeSystem, s, RTTI.of(ComputeSystemCPU))),
        @intFromBool(RTTI.isKindOfObject(ComputeSystem, s, RTTI.of(zolt.ShapeSettings))),
    };
    try std.testing.expectEqual([3]u32{ 1, 1, 0 }, jolt);
    try std.testing.expectEqual(jolt, zolt_values);
}

test "ComputeParity: layouts and constants of the shared structs" {
    // For every struct its size, then the offset of every member, in the order of jolt_compute_layouts
    const types = .{
        TestCompute.TestComputeContext,   TestCompute2.TestCompute2Input,   TestCompute2.TestCompute2Output,
        HairStructs.HairSkinWeight,       HairStructs.HairSkinPoint,        HairStructs.HairGlobalPoseTransform,
        HairStructs.HairSVertexInfluence, HairStructs.HairPosition,         HairStructs.HairVelocity,
        HairStructs.HairMaterial,         HairStructs.HairCollisionPlane,   HairStructs.HairCollisionShape,
        HairStructs.HairUpdateContext,    HairStructs.HairIterationContext,
    };
    var expected: std.ArrayList(u32) = .empty;
    defer expected.deinit(std.testing.allocator);
    inline for (types) |T| {
        try expected.append(std.testing.allocator, @sizeOf(T));
        inline for (@typeInfo(T).@"struct".fields) |field|
            try expected.append(std.testing.allocator, @offsetOf(T, field.name));
    }

    var jolt: [256]u32 = undefined;
    const count = jolt_compute_layouts(&jolt, jolt.len);
    try std.testing.expectEqualSlices(u32, expected.items, jolt[0..count]);

    var constants: [7]c_int = undefined;
    jolt_compute_constants(&constants);
    try std.testing.expectEqual([7]c_int{
        TestCompute.test_compute_group_size,      TestCompute2.test_compute2_group_size, HairStructs.hair_per_vertex_batch,
        HairStructs.hair_per_grid_cell_batch,     HairStructs.hair_per_strand_batch,     HairStructs.hair_per_render_vertex_batch,
        HairStructs.hair_num_s_vertex_influences,
    }, constants);
}

test "ComputeParity: RunTests of ComputeTests.cpp on both sides" {
    const allocator = std.testing.allocator;
    var dispatcher: Dispatcher = try .init(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "ComputeParity RunTests" };

    // Test failing shader creation
    try std.testing.expect(!try dispatcher.dispatchBoth(&checker, "NonExistingShader", .{ 64, 1, 1 }, .{ 1, 1, 1 }, &.{}, .bytes));

    // TestCompute: 10 dispatches with Jolt's values. gData is created without data (zero), the optional buffer is only
    // bound in the first dispatch, later it is a null buffer that the queue ignores.
    const num_elements = 1234; // Not a multiple of cTestComputeGroupSize
    var rnd = zolt.Mt19937.init(zolt.Mt19937.default_seed); // UnitTestRandom
    var optional_data: [num_elements]u32 = undefined;
    for (&optional_data) |*d| d.* = rnd.next();
    var upload_data = [_]u32{42};
    var data: [num_elements]u32 = @splat(0);
    var expected: [num_elements]u32 = undefined;
    for (0..10) |iter| {
        var context = [_]TestCompute.TestComputeContext{.{ .float3_value = .init(0, 0, 0), .uint_value = 7, .float3_value2 = .init(0, 13, 0), .uint_value2 = @intCast(iter), .num_elements = num_elements }};
        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, TestCompute.TestComputeContext, &context),
            BindingDesc.of("gOptionalData", .buffer, u32, &optional_data).withFlags(if (iter == 0) 0 else BindingDesc.flag_null),
            BindingDesc.of("gUploadData", .buffer, u32, &upload_data).withType(.upload_buffer),
            BindingDesc.of("gData", .rw_buffer, u32, &data).withFlags(if (iter == 0) BindingDesc.flag_no_data else 0),
        };
        const groups = (num_elements + TestCompute.test_compute_group_size - 1) / TestCompute.test_compute_group_size;
        try std.testing.expect(try dispatcher.dispatchBoth(&checker, TestCompute.name, .{ TestCompute.test_compute_group_size, 1, 1 }, .{ groups, 1, 1 }, &bindings, .bytes));

        // Copy of the shader logic (RunTests' expected values)
        for (&expected, optional_data) |*e, o|
            e.* = if (iter == 0) o +% 13 +% 42 else (e.* +% 7) *% @as(u32, @intCast(iter));
    }
    try std.testing.expectEqualSlices(u32, &expected, &data);

    // TestCompute2 with Jolt's values: the products are exactly Zolt's Mat44 products
    const m: zolt.Mat44 = .init(.init(2, 3, 5, 0), .init(7, 11, 13, 0), .init(13, 15, 17, 0), .init(17, 19, 23, 0));
    const v: Vec3 = .init(29, 31, 37);
    var input = [_]TestCompute2.TestCompute2Input{.{
        .mat44_value = .init(.init(2, 3, 5, 0), .init(7, 11, 13, 0), .init(13, 15, 17, 0), .init(17, 19, 23, 0)),
        .mat44_mul_value = .init(29, 31, 37),
        .compressed_vec3 = Vec3.init(-2, 3, -5).normalized().compressUnitVector(),
        .compressed_quat = Quat.fromVec4(Vec4.init(2, -3, 5, -7).normalized()).compressUnitQuat(),
    }};
    var output: [1]TestCompute2.TestCompute2Output = undefined;
    const bindings2 = [_]BindingDesc{
        .of("gInput", .buffer, TestCompute2.TestCompute2Input, &input),
        BindingDesc.of("gOutput", .rw_buffer, TestCompute2.TestCompute2Output, &output).withFlags(BindingDesc.flag_no_data),
    };
    try std.testing.expect(try dispatcher.dispatchBoth(&checker, TestCompute2.name, .{ 1, 1, 1 }, .{ 1, 1, 1 }, &bindings2, .floats));
    const mul3x4 = m.mulVec3(v);
    const mul3x3 = m.multiply3x3(v);
    try std.testing.expect(output[0].mul3x4_output.eql(.init(mul3x4.getX(), mul3x4.getY(), mul3x4.getZ())));
    try std.testing.expect(output[0].mul3x3_output.eql(.init(mul3x3.getX(), mul3x3.getY(), mul3x3.getZ())));

    try checker.finish();
}

test "ComputeParity: TestCompute on random buffers (unsigned wrap, int(float) of any float, null and zero buffers)" {
    const allocator = std.testing.allocator;
    var dispatcher: Dispatcher = try .init(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "ComputeParity TestCompute" };
    var rng: fw.Rng = .{};

    for (0..300) |round| {
        const num_elements = rng.intRange(u32, 0, 300);
        const first_write = round % 3 == 0;
        var context = [_]TestCompute.TestComputeContext{.{
            .float3_value = .init(specialFloat(&rng, 100), specialFloat(&rng, 100), specialFloat(&rng, 100)),
            .uint_value = rng.next(),
            .float3_value2 = .init(specialFloat(&rng, 100), specialFloat(&rng, 3.0e9), specialFloat(&rng, 100)),
            .uint_value2 = if (first_write) 0 else rng.next(),
            .num_elements = num_elements,
        }};
        var upload = [_]u32{rng.next()};
        // The buffers have room for the threads of the last group, which return without reading
        const optional = try allocator.alloc(u32, @max(num_elements, 1));
        defer allocator.free(optional);
        for (optional) |*d| d.* = rng.next();
        const data = try allocator.alloc(u32, @max(num_elements, 1));
        defer allocator.free(data);
        for (data) |*d| d.* = rng.next();

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, TestCompute.TestComputeContext, &context),
            BindingDesc.of("gUploadData", .buffer, u32, &upload).withType(if (rng.next() % 2 == 0) .upload_buffer else .buffer),
            // The optional buffer is null when the shader does not read it
            BindingDesc.of("gOptionalData", .buffer, u32, optional).withFlags(if (first_write or rng.next() % 2 == 0) 0 else BindingDesc.flag_null),
            BindingDesc.of("gData", .rw_buffer, u32, data).withFlags(if (rng.next() % 4 == 0) BindingDesc.flag_no_data else 0),
        };
        const group_size = rng.intRange(u32, 1, 70);
        const groups = (num_elements + group_size - 1) / group_size;
        // Any y / z dimension: the shader runs again for every y and z on the same x
        const num_groups_y: u32 = if (round % 5 == 0) 2 else 1;
        _ = try dispatcher.dispatchBoth(&checker, TestCompute.name, .{ group_size, 1, 1 }, .{ groups, num_groups_y, 1 }, &bindings, .bytes);
    }
    try checker.finish();
}

test "ComputeParity: TestCompute2 on 100k random inputs (HLSL Mat44 products, unit vector and quaternion decompression)" {
    const allocator = std.testing.allocator;
    var dispatcher: Dispatcher = try .init(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "ComputeParity TestCompute2" };
    var rng: fw.Rng = .{ .state = 0x9e3779b9 };

    const n = fw.iterations;
    const inputs = try allocator.alloc(TestCompute2.TestCompute2Input, n);
    defer allocator.free(inputs);
    const outputs = try allocator.alloc(TestCompute2.TestCompute2Output, n);
    defer allocator.free(outputs);
    for (inputs, 0..) |*in, i| {
        for (&in.mat44_value.c) |*c| c.* = .init(inputFloat(&rng, 100), inputFloat(&rng, 100), inputFloat(&rng, 100), inputFloat(&rng, 100));
        in.mat44_mul_value = .init(inputFloat(&rng, 100), inputFloat(&rng, 100), inputFloat(&rng, 100));
        if (i % 2 == 0) {
            // Every bit pattern
            in.compressed_vec3 = rng.next();
            in.compressed_quat = rng.next();
        } else {
            // The compressed values of unit vectors and quaternions
            const v = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1)).normalizedOr(Vec3.axisX());
            in.compressed_vec3 = v.compressUnitVector();
            const q = Vec4.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
            in.compressed_quat = Quat.fromVec4(if (q.lengthSq() > 1.0e-6) q.normalized() else Vec4.init(0, 0, 0, 1)).compressUnitQuat();
        }
    }
    @memset(std.mem.sliceAsBytes(outputs), 0xcd);

    const bindings = [_]BindingDesc{
        .of("gInput", .buffer, TestCompute2.TestCompute2Input, inputs),
        .of("gOutput", .rw_buffer, TestCompute2.TestCompute2Output, outputs),
    };
    // Group size 1 like the shader's, and 64 x 1 x 1 groups of other sizes: the global thread ID is the same
    _ = try dispatcher.dispatchBoth(&checker, TestCompute2.name, .{ 1, 1, 1 }, .{ n, 1, 1 }, &bindings, .floats);
    _ = try dispatcher.dispatchBoth(&checker, TestCompute2.name, .{ 100, 1, 1 }, .{ n / 100, 1, 1 }, &bindings, .floats);
    try checker.finish();
}
