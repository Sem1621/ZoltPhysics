//! Zolt addition, no Jolt file: a BodyManager with the unit test layers, for the tests of the Body package
//! Status: complete
//!
//! Phase 5 F1 has no broad phase yet (it comes with F2), so `createAndAddBody` does what BodyInterface::CreateAndAddBody
//! does to the body manager and marks the body as being in the broad phase (`setInBroadPhaseInternal`), which is the
//! state that BodyManager's activation and save / restore functions expect. PhysicsTestContext (SimulationArchitecture
//! D12) replaces it for the tests that need a PhysicsSystem.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig");

const Allocator = std.mem.Allocator;
const Activation = zolt.Activation;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const CreateBodyError = zolt.CreateBodyError;

pub const io = std.testing.io;

pub const BodyTestContext = struct {
    layer_interface: Layers.BPLayerInterfaceImpl = .init(),
    body_manager: BodyManager = .empty,

    /// Initialize in place (the body manager must not move)
    pub fn init(self: *BodyTestContext, allocator: Allocator, max_bodies: u32, num_body_mutexes: u32) Allocator.Error!void {
        self.* = .{};
        try self.body_manager.init(allocator, io, max_bodies, num_body_mutexes, &self.layer_interface.base);
    }

    pub fn deinit(self: *BodyTestContext) void {
        self.body_manager.deinit();
    }

    /// AllocateBody + AddBody (null when out of bodies), then "add it to the broad phase" and activate it like
    /// BodyInterface::AddBody
    pub fn createAndAddBody(self: *BodyTestContext, settings: *const BodyCreationSettings, activation: Activation) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return null;
        }
        self.addToBroadPhaseAndActivate(body, activation);
        return body;
    }

    /// AllocateBody + AddBodyWithCustomID (null when the ID is not available), then like createAndAddBody
    pub fn createAndAddBodyWithID(self: *BodyTestContext, body_id: BodyID, settings: *const BodyCreationSettings, activation: Activation) CreateBodyError!?*Body {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBodyWithCustomID(body, body_id)) {
            self.body_manager.freeBody(body);
            return null;
        }
        self.addToBroadPhaseAndActivate(body, activation);
        return body;
    }

    fn addToBroadPhaseAndActivate(self: *BodyTestContext, body: *Body, activation: Activation) void {
        body.setInBroadPhaseInternal(true);
        if (activation == .activate and !body.isStatic())
            self.body_manager.activateBodies(&.{body.getID()});
    }

    /// Like BodyInterface::RemoveBody + DestroyBody: deactivate, take it out of the "broad phase", destroy
    pub fn removeAndDestroyBody(self: *BodyTestContext, body_id: BodyID) void {
        self.body_manager.deactivateBodies(&.{body_id});
        self.body_manager.getBodyMut(body_id).setInBroadPhaseInternal(false);
        self.body_manager.destroyBodies(&.{body_id});
    }
};
