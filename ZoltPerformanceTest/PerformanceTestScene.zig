//! Port of: PerformanceTest/PerformanceTestScene.h
//! Status: complete
//!
//! The base class of the scenes is an interface (porting guide pattern B): a type erased `ptr` and a vtable generated
//! from the scene type. The pure virtual functions (`GetName`, `StartTest`) are non-null entries that every scene
//! declares, the virtual functions with a default body are nullable entries: a scene that does not declare one gets
//! Jolt's default (below). The interface owns the scene like PerformanceTest.cpp's `unique_ptr<PerformanceTestScene>`:
//! `destroy` is the virtual destructor plus `delete` (every scene is created on the heap with its `create`).
//!
//! Functions that allocate return `Error` (out of memory, or a body whose shape could not be created: Jolt crashes on
//! both).

const std = @import("std");
const zolt = @import("zolt");

const PhysicsSystem = zolt.PhysicsSystem;
const MotionQuality = zolt.MotionQuality;
const TempAllocator = zolt.TempAllocator;

/// Base class for a test scene to test performance
pub const PerformanceTestScene = struct {
    /// The errors of the scene functions (Zolt: Jolt crashes when it runs out of memory)
    pub const Error = zolt.CreateBodyError;

    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Virtual destructor (and `delete`)
        destroy: *const fn (ptr: *anyopaque) void,

        /// Get name of test for debug purposes
        getName: *const fn (ptr: *anyopaque) [:0]const u8,

        /// Get the number of MB that the temp allocator should preallocate (null: 32)
        getTempAllocatorSizeMB: ?*const fn (ptr: *anyopaque) usize = null,

        /// Get the max number of bodies to support in the physics system (null: 10240)
        getMaxBodies: ?*const fn (ptr: *anyopaque) u32 = null,

        /// Get the max number of body pairs to support in the physics system (null: 65536)
        getMaxBodyPairs: ?*const fn (ptr: *anyopaque) u32 = null,

        /// Get the max number of contact constraints to support in the physics system (null: 20480)
        getMaxContactConstraints: ?*const fn (ptr: *anyopaque) u32 = null,

        /// Load assets for the scene (null: nothing to load, returns true)
        load: ?*const fn (ptr: *anyopaque, asset_path: []const u8) Error!bool = null,

        /// Start a new test by adding objects to physics_system
        startTest: *const fn (ptr: *anyopaque, physics_system: *PhysicsSystem, motion_quality: MotionQuality) Error!void,

        /// Step the test (null: nothing)
        updateTest: ?*const fn (ptr: *anyopaque, physics_system: *PhysicsSystem, temp_allocator: TempAllocator, delta_time: f32) Error!void = null,

        /// Update the hash with the state of the scene (null: nothing)
        updateHash: ?*const fn (ptr: *anyopaque, hash: *u64) void = null,

        /// Stop a test and remove objects from physics_system (null: nothing)
        stopTest: ?*const fn (ptr: *anyopaque, physics_system: *PhysicsSystem) void = null,
    };

    /// Wrap a heap allocated `*T` (the interface owns it): `T` declares `destroy`, `getName` and `startTest`, and
    /// optionally the other functions of VTable
    pub fn init(impl: anytype) PerformanceTestScene {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing the functions of the outer struct
            fn cast(ptr: *anyopaque) *T {
                return @ptrCast(@alignCast(ptr));
            }

            fn destroyThunk(ptr: *anyopaque) void {
                cast(ptr).destroy();
            }

            fn getNameThunk(ptr: *anyopaque) [:0]const u8 {
                return cast(ptr).getName();
            }

            fn getTempAllocatorSizeMBThunk(ptr: *anyopaque) usize {
                return cast(ptr).getTempAllocatorSizeMB();
            }

            fn getMaxBodiesThunk(ptr: *anyopaque) u32 {
                return cast(ptr).getMaxBodies();
            }

            fn getMaxBodyPairsThunk(ptr: *anyopaque) u32 {
                return cast(ptr).getMaxBodyPairs();
            }

            fn getMaxContactConstraintsThunk(ptr: *anyopaque) u32 {
                return cast(ptr).getMaxContactConstraints();
            }

            fn loadThunk(ptr: *anyopaque, asset_path: []const u8) Error!bool {
                return cast(ptr).load(asset_path);
            }

            fn startTestThunk(ptr: *anyopaque, physics_system: *PhysicsSystem, motion_quality: MotionQuality) Error!void {
                return cast(ptr).startTest(physics_system, motion_quality);
            }

            fn updateTestThunk(ptr: *anyopaque, physics_system: *PhysicsSystem, temp_allocator: TempAllocator, delta_time: f32) Error!void {
                return cast(ptr).updateTest(physics_system, temp_allocator, delta_time);
            }

            fn updateHashThunk(ptr: *anyopaque, hash: *u64) void {
                cast(ptr).updateHash(hash);
            }

            fn stopTestThunk(ptr: *anyopaque, physics_system: *PhysicsSystem) void {
                cast(ptr).stopTest(physics_system);
            }

            const vtable: VTable = .{
                .destroy = destroyThunk,
                .getName = getNameThunk,
                .getTempAllocatorSizeMB = if (@hasDecl(T, "getTempAllocatorSizeMB")) getTempAllocatorSizeMBThunk else null,
                .getMaxBodies = if (@hasDecl(T, "getMaxBodies")) getMaxBodiesThunk else null,
                .getMaxBodyPairs = if (@hasDecl(T, "getMaxBodyPairs")) getMaxBodyPairsThunk else null,
                .getMaxContactConstraints = if (@hasDecl(T, "getMaxContactConstraints")) getMaxContactConstraintsThunk else null,
                .load = if (@hasDecl(T, "load")) loadThunk else null,
                .startTest = startTestThunk,
                .updateTest = if (@hasDecl(T, "updateTest")) updateTestThunk else null,
                .updateHash = if (@hasDecl(T, "updateHash")) updateHashThunk else null,
                .stopTest = if (@hasDecl(T, "stopTest")) stopTestThunk else null,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Virtual destructor: destroys the scene
    pub fn destroy(self: PerformanceTestScene) void {
        self.vtable.destroy(self.ptr);
    }

    /// Get name of test for debug purposes
    pub fn getName(self: PerformanceTestScene) [:0]const u8 {
        return self.vtable.getName(self.ptr);
    }

    /// Get the number of MB that the temp allocator should preallocate
    pub fn getTempAllocatorSizeMB(self: PerformanceTestScene) usize {
        if (self.vtable.getTempAllocatorSizeMB) |f| return f(self.ptr);
        return 32;
    }

    /// Get the max number of bodies to support in the physics system
    pub fn getMaxBodies(self: PerformanceTestScene) u32 {
        if (self.vtable.getMaxBodies) |f| return f(self.ptr);
        return 10240;
    }

    /// Get the max number of body pairs to support in the physics system
    pub fn getMaxBodyPairs(self: PerformanceTestScene) u32 {
        if (self.vtable.getMaxBodyPairs) |f| return f(self.ptr);
        return 65536;
    }

    /// Get the max number of contact constraints to support in the physics system
    pub fn getMaxContactConstraints(self: PerformanceTestScene) u32 {
        if (self.vtable.getMaxContactConstraints) |f| return f(self.ptr);
        return 20480;
    }

    /// Load assets for the scene
    pub fn load(self: PerformanceTestScene, asset_path: []const u8) Error!bool {
        if (self.vtable.load) |f| return f(self.ptr, asset_path);
        return true;
    }

    /// Start a new test by adding objects to physics_system
    pub fn startTest(self: PerformanceTestScene, physics_system: *PhysicsSystem, motion_quality: MotionQuality) Error!void {
        return self.vtable.startTest(self.ptr, physics_system, motion_quality);
    }

    /// Step the test
    pub fn updateTest(self: PerformanceTestScene, physics_system: *PhysicsSystem, temp_allocator: TempAllocator, delta_time: f32) Error!void {
        if (self.vtable.updateTest) |f| return f(self.ptr, physics_system, temp_allocator, delta_time);
    }

    /// Update the hash with the state of the scene
    pub fn updateHash(self: PerformanceTestScene, hash: *u64) void {
        if (self.vtable.updateHash) |f| f(self.ptr, hash);
    }

    /// Stop a test and remove objects from physics_system
    pub fn stopTest(self: PerformanceTestScene, physics_system: *PhysicsSystem) void {
        if (self.vtable.stopTest) |f| f(self.ptr, physics_system);
    }
};
