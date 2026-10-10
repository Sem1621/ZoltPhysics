//! Port of: Jolt/Physics/Body/BodyLockInterface.h
//! Status: complete
//!
//! SimulationArchitecture D3: Jolt has exactly two `final` implementations of the abstract BodyLockInterface
//! (BodyLockInterfaceNoLock and BodyLockInterfaceLocking) and nothing else derives from it. The closed set is a tagged
//! struct with an exhaustive switch: both C++ class bodies stay side by side in every function. Construct the two
//! variants with `initNoLock` / `initLocking`; PhysicsSystem embeds both and hands out `*const BodyLockInterface`.
//! - Overloads get suffixes: `LockRead(BodyID)` / `LockRead(MutexMask)` are `lockRead` / `lockReadMask` (same for the
//!   unlock and write functions).
//! - Jolt's `BodyManager &mBodyManager` is a non-const reference used from const functions: `body_manager: *BodyManager`
//!   (state behind a pointer, Rule M).

const std = @import("std");
const SharedMutex = @import("../../Core/Mutex.zig").SharedMutex;
const BodyID = @import("BodyID.zig").BodyID;
const Body = @import("Body.zig").Body;
const BodyManager = @import("BodyManager.zig").BodyManager;
const PhysicsLock = @import("../PhysicsLock.zig").PhysicsLock;

/// Base class interface for locking a body. Usually you will use BodyLockRead / BodyLockWrite / BodyLockMultiRead / BodyLockMultiWrite instead.
pub const BodyLockInterface = struct {
    /// Redefine MutexMask
    pub const MutexMask = BodyManager.MutexMask;

    /// The C++ class
    pub const Kind = enum {
        /// BodyLockInterfaceNoLock: implementation that performs no locking (assumes the lock has already been taken)
        no_lock,
        /// BodyLockInterfaceLocking: implementation that uses the body manager to lock the correct mutex for a body
        locking,
    };

    body_manager: *BodyManager,
    kind: Kind,

    /// Constructor of BodyLockInterfaceNoLock: implementation that performs no locking (assumes the lock has already been taken)
    pub fn initNoLock(body_manager: *BodyManager) BodyLockInterface {
        return .{ .body_manager = body_manager, .kind = .no_lock };
    }

    /// Constructor of BodyLockInterfaceLocking: implementation that uses the body manager to lock the correct mutex for a body
    pub fn initLocking(body_manager: *BodyManager) BodyLockInterface {
        return .{ .body_manager = body_manager, .kind = .locking };
    }

    ///@name Locking functions

    pub fn lockRead(self: *const BodyLockInterface, body_id: BodyID) ?*SharedMutex {
        switch (self.kind) {
            .no_lock => return null,
            .locking => {
                const bm = self.body_manager;
                const mutex = bm.getMutexForBody(body_id);
                PhysicsLock.lockShared(mutex, bm.io, bm, .per_body);
                return mutex;
            },
        }
    }

    pub fn unlockRead(self: *const BodyLockInterface, mutex: ?*SharedMutex) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => {
                const bm = self.body_manager;
                PhysicsLock.unlockShared(mutex.?, bm.io, bm, .per_body);
            },
        }
    }

    pub fn lockWrite(self: *const BodyLockInterface, body_id: BodyID) ?*SharedMutex {
        switch (self.kind) {
            .no_lock => return null,
            .locking => {
                const bm = self.body_manager;
                const mutex = bm.getMutexForBody(body_id);
                PhysicsLock.lock(mutex, bm.io, bm, .per_body);
                return mutex;
            },
        }
    }

    pub fn unlockWrite(self: *const BodyLockInterface, mutex: ?*SharedMutex) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => {
                const bm = self.body_manager;
                PhysicsLock.unlock(mutex.?, bm.io, bm, .per_body);
            },
        }
    }

    /// Get the mask needed to lock all bodies
    pub fn getAllBodiesMutexMask(self: *const BodyLockInterface) MutexMask {
        return self.body_manager.getAllBodiesMutexMask();
    }

    ///@name Batch locking functions

    pub fn getMutexMask(self: *const BodyLockInterface, bodies: []const BodyID) MutexMask {
        return switch (self.kind) {
            .no_lock => 0,
            .locking => self.body_manager.getMutexMask(bodies),
        };
    }

    pub fn lockReadMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => self.body_manager.lockRead(mutex_mask),
        }
    }

    pub fn unlockReadMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => self.body_manager.unlockRead(mutex_mask),
        }
    }

    pub fn lockWriteMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => self.body_manager.lockWrite(mutex_mask),
        }
    }

    pub fn unlockWriteMask(self: *const BodyLockInterface, mutex_mask: MutexMask) void {
        switch (self.kind) {
            .no_lock => {}, // Nothing to do
            .locking => self.body_manager.unlockWrite(mutex_mask),
        }
    }

    /// Convert body ID to body
    pub fn tryGetBody(self: *const BodyLockInterface, body_id: BodyID) ?*Body {
        return self.body_manager.tryGetBodyMut(body_id);
    }
};
