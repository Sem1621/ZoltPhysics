//! Port of: Jolt/Physics/PhysicsStepListener.h
//! Status: complete
//!
//! SimulationArchitecture D7: PhysicsStepListener is a pure interface (porting guide pattern B). `OnStep` is pure
//! virtual, so its vtable entry is non-null and an implementation must declare it:
//! ```zig
//! const MyListener = struct {
//!     pub fn onStep(self: *MyListener, context: *const PhysicsStepListenerContext) void { ... }
//! };
//! try physics_system.addStepListener(.init(&my_listener));
//! ```
//! The listener is stored by value in the PhysicsSystem and does not own the implementation (the C++ virtual
//! destructor has no counterpart). `removeStepListener` finds it by its `ptr`. The callbacks run concurrently on the
//! job system's worker threads during PhysicsSystem.update.

const PhysicsSystem = @import("PhysicsSystem.zig").PhysicsSystem;

/// Context information for the step listener
pub const PhysicsStepListenerContext = struct {
    /// Delta time of the current step
    delta_time: f32,
    /// True if this is the first step
    is_first_step: bool,
    /// True if this is the last step
    is_last_step: bool,
    /// The physics system that is being stepped
    physics_system: *PhysicsSystem,
};

/// A listener class that receives a callback before every physics simulation step
pub const PhysicsStepListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Called before every simulation step (received inCollisionSteps times for every PhysicsSystem::Update(...) call)
        /// This is called while all body and constraint mutexes are locked. You can read/write bodies and constraints but not add/remove them.
        /// Multiple listeners can be executed in parallel and it is the responsibility of the listener to avoid race conditions.
        /// The best way to do this is to have each step listener operate on a subset of the bodies and constraints
        /// and making sure that these bodies and constraints are not touched by any other step listener.
        /// Note that this function is not called if there aren't any active bodies or when the physics system is updated with 0 delta time.
        onStep: *const fn (ptr: *anyopaque, context: *const PhysicsStepListenerContext) void,
    };

    /// Wrap any `*T` that declares `onStep`
    pub fn init(impl: anytype) PhysicsStepListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing the functions of the outer struct
            fn onStepThunk(ptr: *anyopaque, context: *const PhysicsStepListenerContext) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onStep(context);
            }

            const vtable: VTable = .{ .onStep = onStepThunk };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Called before every simulation step (see VTable)
    pub fn onStep(self: PhysicsStepListener, context: *const PhysicsStepListenerContext) void {
        self.vtable.onStep(self.ptr, context);
    }
};

test "PhysicsStepListener forwards onStep" {
    const std = @import("std");
    const Counter = struct {
        count: u32 = 0,
        last_delta_time: f32 = 0,

        pub fn onStep(self: *@This(), context: *const PhysicsStepListenerContext) void {
            self.count += 1;
            self.last_delta_time = context.delta_time;
        }
    };
    var counter: Counter = .{};
    const listener = PhysicsStepListener.init(&counter);
    var system: PhysicsSystem = .empty;
    const context: PhysicsStepListenerContext = .{ .delta_time = 0.5, .is_first_step = true, .is_last_step = false, .physics_system = &system };
    listener.onStep(&context);
    listener.onStep(&context);
    try std.testing.expectEqual(@as(u32, 2), counter.count);
    try std.testing.expectEqual(@as(f32, 0.5), counter.last_delta_time);
    try std.testing.expect(listener.ptr == @as(*anyopaque, &counter));
}
