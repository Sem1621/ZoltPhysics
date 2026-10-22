//! Port of: Jolt/Physics/Hair/HairShaders.h, Jolt/Physics/Hair/HairShaders.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R14: a RefTarget without virtual functions with the 15 shader references.
//! `create(allocator)` is `new HairShaders`; shaders on the stack are `.{ .allocator = allocator }`. `init(system)` is
//! Jolt's `Init`: a shader that the system does not know stays null, like Jolt's. An allocation failure returns
//! `error.OutOfMemory` (Jolt's CPU backend aborts); the shaders created before stay set.

const std = @import("std");
const Allocator = std.mem.Allocator;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const ComputeSystem = @import("../../Compute/ComputeSystem.zig").ComputeSystem;
const ComputeShader = @import("../../Compute/ComputeShader.zig").ComputeShader;
const HairStructs = @import("../../Shaders/HairStructs.zig");

/// This class loads the shaders used by the hair system. This can be shared among all hair instances.
pub const HairShaders = struct {
    /// RefTarget<HairShaders>
    ref_count: RefCount = .{},
    /// Frees the object (create)
    allocator: Allocator,

    teleport_cs: Ref(ComputeShader) = .empty,
    apply_delta_transform_cs: Ref(ComputeShader) = .empty,
    skin_vertices_cs: Ref(ComputeShader) = .empty,
    skin_roots_cs: Ref(ComputeShader) = .empty,
    apply_global_pose_cs: Ref(ComputeShader) = .empty,
    calculate_collision_planes_cs: Ref(ComputeShader) = .empty,
    grid_clear_cs: Ref(ComputeShader) = .empty,
    grid_accumulate_cs: Ref(ComputeShader) = .empty,
    grid_normalize_cs: Ref(ComputeShader) = .empty,
    integrate_cs: Ref(ComputeShader) = .empty,
    update_roots_cs: Ref(ComputeShader) = .empty,
    update_strands_cs: Ref(ComputeShader) = .empty,
    update_velocity_cs: Ref(ComputeShader) = .empty,
    update_velocity_integrate_cs: Ref(ComputeShader) = .empty,
    calculate_render_positions_cs: Ref(ComputeShader) = .empty,

    /// new HairShaders: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*HairShaders {
        const self = try allocator.create(HairShaders);
        self.* = .{ .allocator = allocator };
        return self;
    }

    /// Destructor: releases the shaders
    pub fn deinit(self: *HairShaders) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *HairShaders) void {
        inline for (shader_fields) |field|
            @field(self, field.member).deinit();
    }

    /// Add a reference to this object
    pub fn addRef(self: *const HairShaders) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const HairShaders) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this object as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const HairShaders) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const HairShaders) u32 {
        return self.ref_count.get();
    }

    const ShaderField = struct {
        member: []const u8,
        name: []const u8,
        group_size_x: u32,
    };

    /// The shaders in Jolt's order: the member, the shader name, the group size
    const shader_fields = [_]ShaderField{
        .{ .member = "teleport_cs", .name = "HairTeleport", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "apply_delta_transform_cs", .name = "HairApplyDeltaTransform", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "skin_vertices_cs", .name = "HairSkinVertices", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "skin_roots_cs", .name = "HairSkinRoots", .group_size_x = HairStructs.hair_per_strand_batch },
        .{ .member = "apply_global_pose_cs", .name = "HairApplyGlobalPose", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "calculate_collision_planes_cs", .name = "HairCalculateCollisionPlanes", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "grid_clear_cs", .name = "HairGridClear", .group_size_x = HairStructs.hair_per_grid_cell_batch },
        .{ .member = "grid_accumulate_cs", .name = "HairGridAccumulate", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "grid_normalize_cs", .name = "HairGridNormalize", .group_size_x = HairStructs.hair_per_grid_cell_batch },
        .{ .member = "integrate_cs", .name = "HairIntegrate", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "update_roots_cs", .name = "HairUpdateRoots", .group_size_x = HairStructs.hair_per_strand_batch },
        .{ .member = "update_strands_cs", .name = "HairUpdateStrands", .group_size_x = HairStructs.hair_per_strand_batch },
        .{ .member = "update_velocity_cs", .name = "HairUpdateVelocity", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "update_velocity_integrate_cs", .name = "HairUpdateVelocityIntegrate", .group_size_x = HairStructs.hair_per_vertex_batch },
        .{ .member = "calculate_render_positions_cs", .name = "HairCalculateRenderPositions", .group_size_x = HairStructs.hair_per_render_vertex_batch },
    };

    /// Loads all shaders
    /// Note that if you want to run the sim on CPU you need call HairRegisterShaders first.
    pub fn init(self: *HairShaders, compute_system: *ComputeSystem) Allocator.Error!void {
        // auto get = [](const ComputeShaderResult &inResult) { return inResult.IsValid()? inResult.Get() : nullptr; };
        inline for (shader_fields) |field| {
            const shader: Ref(ComputeShader) = compute_system.createComputeShader(field.name, field.group_size_x, .{}) catch |err| switch (err) {
                error.ShaderNotFound => .empty,
                error.OutOfMemory => |e| return e,
            };
            const member = &@field(self, field.member);
            member.deinit();
            member.* = shader;
        }
    }
};

test "HairShaders: Jolt's 15 shaders, a missing shader stays null" {
    const testing = std.testing;
    const ComputeSystemCPU = @import("../../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
    const virtual = @import("../../Core/Virtual.zig");
    const HairGridClear = @import("../../Shaders/HairGridClear.zig");
    const HairTeleport = @import("../../Shaders/HairTeleport.zig");

    try testing.expectEqual(@as(usize, 15), HairShaders.shader_fields.len);

    var system = try ComputeSystemCPU.create(testing.allocator);
    defer system.deinit();
    const cpu = virtual.downcast(ComputeSystemCPU, system.get().?); // StaticCast<ComputeSystemCPU>
    try cpu.registerShader(HairGridClear);
    try cpu.registerShader(HairTeleport);

    const shaders = try HairShaders.create(testing.allocator);
    var ref: Ref(HairShaders) = .init(shaders);
    defer ref.deinit();
    try shaders.init(system.get().?);
    try testing.expect(shaders.grid_clear_cs.get() != null);
    try testing.expect(shaders.teleport_cs.get() != null);
    try testing.expect(shaders.integrate_cs.get() == null);
    try testing.expect(shaders.calculate_render_positions_cs.get() == null);

    // A second Init replaces the shaders (Jolt's Ref assignment releases the old ones)
    const old = shaders.grid_clear_cs.get().?;
    try shaders.init(system.get().?);
    try testing.expect(shaders.grid_clear_cs.get().? != old);
}

test "HairShaders: allocation failures in init" {
    // Fail each allocation of init in turn: init returns error.OutOfMemory, the shaders created before stay set (a
    // prefix of the 15 fields), nothing leaks, and a second init with memory sets every shader
    const testing = std.testing;
    const ComputeSystemCPU = @import("../../Compute/CPU/ComputeSystemCPU.zig").ComputeSystemCPU;
    const virtual = @import("../../Core/Virtual.zig");
    const registerHairShaders = @import("../../Shaders/HairWrapper.zig").registerHairShaders;

    var fail_at: usize = 0;
    while (true) : (fail_at += 1) {
        var failing = std.testing.FailingAllocator.init(testing.allocator, .{});
        const allocator = failing.allocator();
        var system = try ComputeSystemCPU.create(allocator);
        defer system.deinit();
        try registerHairShaders(virtual.downcast(ComputeSystemCPU, system.get().?));
        var ref: Ref(HairShaders) = .init(try HairShaders.create(allocator));
        defer ref.deinit();
        const shaders = ref.get().?;

        failing.fail_index = failing.alloc_index + fail_at;
        shaders.init(system.get().?) catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            var num_set: usize = 0;
            inline for (HairShaders.shader_fields) |field| {
                if (@field(shaders, field.member).get() != null) num_set += 1;
            }
            inline for (HairShaders.shader_fields, 0..) |field, f|
                try testing.expectEqual(f < num_set, @field(shaders, field.member).get() != null);
            try testing.expect(num_set < HairShaders.shader_fields.len);

            failing.fail_index = std.math.maxInt(usize);
            try shaders.init(system.get().?);
            inline for (HairShaders.shader_fields) |field|
                try testing.expect(@field(shaders, field.member).get() != null);
            continue;
        };
        try testing.expect(!failing.has_induced_failure);
        inline for (HairShaders.shader_fields) |field|
            try testing.expect(@field(shaders, field.member).get() != null);
        break;
    }
    try testing.expect(fail_at >= HairShaders.shader_fields.len); // At least one allocation per shader
}
