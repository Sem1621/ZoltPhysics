//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.h, Jolt/Physics/Collision/BroadPhase/BroadPhaseBruteForce.cpp
//! Status: complete
//!
//! SimulationArchitecture D9: the `final` concrete class of the hierarchy BroadPhaseQuery -> BroadPhase. It is
//! created on the heap like Jolt's `new BroadPhaseBruteForce` (`create(allocator)`) and freed through the virtual
//! destructor (`broad_phase.base.destroy()`).
//! - `mutable SharedMutex mMutex` lives in the heap block `mutable_fields` (Rule M), so the const queries and
//!   FindCollidingPairs keep `*const` receivers. Like Jolt, the mutex is locked directly (`lock_guard` /
//!   `shared_lock`), without PhysicsLock's lock order bookkeeping. `io` is the body manager's.
//! - Zolt deviation: `mBodyIDs.resize(idx + inNumber)` in AddBodiesFinalize can allocate in Jolt, but Finalize cannot
//!   fail in Zolt (D9). This class therefore overrides Init (`initBroadPhase`, which Jolt's BruteForce does not
//!   override) to reserve the list for the maximum number of bodies of the body manager: a body is in the broad phase
//!   at most once, so Finalize never grows the list. Everything else is a literal port, including Jolt's empty
//!   NotifyBodiesAABBChanged / NotifyBodiesLayerChanged (the bodies are referenced directly).
//! - `std::lower_bound` on the sorted list is `lowerBound` (a binary search with BodyID's `operator <`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const SharedMutex = @import("../../../Core/Mutex.zig").SharedMutex;
const quickSort = @import("../../../Core/QuickSort.zig").quickSort;
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const Body = @import("../../Body/Body.zig").Body;
const BodyPair = @import("../../Body/BodyPair.zig").BodyPair;
const BodyManager = @import("../../Body/BodyManager.zig").BodyManager;
const RayCast = @import("../RayCast.zig").RayCast;
const AABoxCast = @import("../AABoxCast.zig").AABoxCast;
const BroadPhaseCastResult = @import("../CastResult.zig").BroadPhaseCastResult;
const ObjectLayerFilter = @import("../ObjectLayer.zig").ObjectLayerFilter;
const ObjectLayerPairFilter = @import("../ObjectLayer.zig").ObjectLayerPairFilter;
const BroadPhaseLayerFile = @import("BroadPhaseLayer.zig");
const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = BroadPhaseLayerFile.BroadPhaseLayerFilter;
const ObjectVsBroadPhaseLayerFilter = BroadPhaseLayerFile.ObjectVsBroadPhaseLayerFilter;
const BroadPhaseQueryFile = @import("BroadPhaseQuery.zig");
const RayCastBodyCollector = BroadPhaseQueryFile.RayCastBodyCollector;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;
const CollideShapeBodyCollector = BroadPhaseQueryFile.CollideShapeBodyCollector;
const BroadPhaseFile = @import("BroadPhase.zig");
const BroadPhase = BroadPhaseFile.BroadPhase;
const BodyPairCollector = BroadPhaseFile.BodyPairCollector;

/// Test BroadPhase implementation that does not do anything to speed up the operations. Can be used as a reference implementation.
pub const BroadPhaseBruteForce = struct {
    // Implementing interface of BroadPhase (see BroadPhase for documentation)
    pub const overrides = .{ .initBroadPhase, .addBodiesFinalize, .removeBodies, .notifyBodiesAABBChanged, .notifyBodiesLayerChanged, .castRay, .collideAABox, .collideSphere, .collidePoint, .collideOrientedBox, .castAABoxNoLock, .castAABox, .findCollidingPairs, .getBounds };

    /// The C++ `mutable` members, locked by the const queries (Rule M: behind a pointer, never through @constCast)
    const MutableFields = struct {
        mutex: SharedMutex = .{},
    };

    base: BroadPhase,
    body_ids: std.ArrayList(BodyID) = .empty,
    mutable_fields: *MutableFields,

    /// new BroadPhaseBruteForce
    pub fn create(allocator: Allocator) Allocator.Error!*BroadPhaseBruteForce {
        const mutable_fields = try allocator.create(MutableFields);
        errdefer allocator.destroy(mutable_fields);
        mutable_fields.* = .{};
        const self = try allocator.create(BroadPhaseBruteForce);
        self.* = .{ .base = .init(BroadPhaseBruteForce, allocator), .mutable_fields = mutable_fields };
        return self;
    }

    /// ~BroadPhaseBruteForce (run by the generated destructor chain)
    pub fn destruct(self: *BroadPhaseBruteForce) void {
        const allocator = self.base.base.allocator;
        self.body_ids.deinit(allocator);
        allocator.destroy(self.mutable_fields);
    }

    /// The BroadPhase base class
    pub fn asBroadPhase(self: *BroadPhaseBruteForce) *BroadPhase {
        return &self.base;
    }

    fn bodyManager(self: *const BroadPhaseBruteForce) *BodyManager {
        return self.base.getBodyManager();
    }

    fn io(self: *const BroadPhaseBruteForce) std.Io {
        return self.bodyManager().io;
    }

    /// Zolt override of BroadPhase::Init (see the file comment): reserves the body list so that AddBodiesFinalize cannot fail
    pub fn initBroadPhase(self: *BroadPhaseBruteForce, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        try BroadPhase.impl.initBroadPhase(&self.base, body_manager, layer_interface);
        try self.body_ids.ensureTotalCapacityPrecise(self.base.base.allocator, body_manager.getMaxBodies());
    }

    fn bodyIDLessThan(_: void, lhs: BodyID, rhs: BodyID) bool {
        return lhs.lessThan(rhs);
    }

    /// std::lower_bound: the first index whose body ID is not less than `body_id`
    fn lowerBound(body_ids: []const BodyID, body_id: BodyID) usize {
        var first: usize = 0;
        var count: usize = body_ids.len;
        while (count > 0) {
            const step = count / 2;
            const it = first + step;
            if (body_ids[it].lessThan(body_id)) {
                first = it + 1;
                count -= step + 1;
            } else {
                count = step;
            }
        }
        return first;
    }

    pub fn addBodiesFinalize(self: *BroadPhaseBruteForce, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        _ = add_state;
        const mutex = &self.mutable_fields.mutex;
        mutex.lock(self.io());
        defer mutex.unlock(self.io());

        const body_manager = self.bodyManager();

        // Allocate space
        var idx: u32 = @intCast(self.body_ids.items.len);
        std.debug.assert(self.body_ids.capacity >= idx + bodies.len); // Zolt: reserved in initBroadPhase
        self.body_ids.items.len = idx + bodies.len;

        // Add bodies
        for (bodies) |b| {
            const body = body_manager.getBodyMut(b);

            // Validate that body ID is consistent with array index
            std.debug.assert(body.getID().eql(b));
            std.debug.assert(!body.isInBroadPhase());

            // Add it to the list
            self.body_ids.items[idx] = body.getID();
            idx += 1;

            // Indicate body is in the broadphase
            body.setInBroadPhaseInternal(true);
        }

        // Resort
        quickSort(BodyID, self.body_ids.items, {}, bodyIDLessThan);
    }

    pub fn removeBodies(self: *BroadPhaseBruteForce, bodies: []BodyID) void {
        const mutex = &self.mutable_fields.mutex;
        mutex.lock(self.io());
        defer mutex.unlock(self.io());

        const body_manager = self.bodyManager();

        std.debug.assert(self.body_ids.items.len >= bodies.len);

        // Remove bodies
        for (bodies) |b| {
            const body = body_manager.getBodyMut(b);

            // Validate that body ID is consistent with array index
            std.debug.assert(body.getID().eql(b));
            std.debug.assert(body.isInBroadPhase());

            // Find body id
            const it = lowerBound(self.body_ids.items, body.getID());
            std.debug.assert(it != self.body_ids.items.len);

            // Remove element
            _ = self.body_ids.orderedRemove(it);

            // Indicate body is no longer in the broadphase
            body.setInBroadPhaseInternal(false);
        }
    }

    pub fn notifyBodiesAABBChanged(self: *BroadPhaseBruteForce, bodies: []BodyID, take_lock: bool) void {
        // Do nothing, we directly reference the body
        _ = .{ self, bodies, take_lock };
    }

    pub fn notifyBodiesLayerChanged(self: *BroadPhaseBruteForce, bodies: []BodyID) Allocator.Error!void {
        // Do nothing, we directly reference the body
        _ = .{ self, bodies };
    }

    pub fn castRay(self: *const BroadPhaseBruteForce, ray: *const RayCast, collector: *RayCastBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // Load ray
        const origin = ray.origin;
        const inv_direction = RayInvDirection.init(ray.direction);

        // For all bodies
        var early_out_fraction = collector.getEarlyOutFraction();
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with ray
                const bounds = body.getWorldSpaceBounds();
                const fraction = RayAABox.rayAABox(origin, inv_direction, bounds.min, bounds.max);
                if (fraction < early_out_fraction) {
                    // Store hit
                    const result: BroadPhaseCastResult = .{ .body_id = b, .fraction = fraction };
                    collector.addHit(&result);
                    if (collector.shouldEarlyOut())
                        break;
                    early_out_fraction = collector.getEarlyOutFraction();
                }
            }
        }
    }

    pub fn collideAABox(self: *const BroadPhaseBruteForce, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // For all bodies
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with box
                const bounds = body.getWorldSpaceBounds();
                if (bounds.overlaps(box.*)) {
                    // Store hit
                    collector.addHit(&b);
                    if (collector.shouldEarlyOut())
                        break;
                }
            }
        }
    }

    pub fn collideSphere(self: *const BroadPhaseBruteForce, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        const radius_sq = math.square(radius);

        // For all bodies
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with box
                const bounds = body.getWorldSpaceBounds();
                if (bounds.getSqDistanceTo(center) <= radius_sq) {
                    // Store hit
                    collector.addHit(&b);
                    if (collector.shouldEarlyOut())
                        break;
                }
            }
        }
    }

    pub fn collidePoint(self: *const BroadPhaseBruteForce, point: Vec3, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // For all bodies
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with box
                const bounds = body.getWorldSpaceBounds();
                if (bounds.containsVec3(point)) {
                    // Store hit
                    collector.addHit(&b);
                    if (collector.shouldEarlyOut())
                        break;
                }
            }
        }
    }

    pub fn collideOrientedBox(self: *const BroadPhaseBruteForce, box: *const OrientedBox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // For all bodies
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with box
                const bounds = body.getWorldSpaceBounds();
                if (box.overlapsAABox(bounds.*, .{})) {
                    // Store hit
                    collector.addHit(&b);
                    if (collector.shouldEarlyOut())
                        break;
                }
            }
        }
    }

    pub fn castAABoxNoLock(self: *const BroadPhaseBruteForce, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        _ = broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // Load box
        const origin = box.box.getCenter();
        const extent = box.box.getExtent();
        const inv_direction = RayInvDirection.init(box.direction);

        // For all bodies
        var early_out_fraction = collector.getPositiveEarlyOutFraction();
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);

            // Test layer
            if (object_layer_filter.shouldCollide(body.getObjectLayer())) {
                // Test intersection with ray
                const bounds = body.getWorldSpaceBounds();
                const fraction = RayAABox.rayAABox(origin, inv_direction, bounds.min.sub(extent), bounds.max.add(extent));
                if (fraction < early_out_fraction) {
                    // Store hit
                    const result: BroadPhaseCastResult = .{ .body_id = b, .fraction = fraction };
                    collector.addHit(&result);
                    if (collector.shouldEarlyOut())
                        break;
                    early_out_fraction = collector.getPositiveEarlyOutFraction();
                }
            }
        }
    }

    pub fn castAABox(self: *const BroadPhaseBruteForce, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.castAABoxNoLock(box, collector, broad_phase_layer_filter, object_layer_filter);
    }

    pub fn findCollidingPairs(self: *const BroadPhaseBruteForce, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) Allocator.Error!void {
        _ = object_vs_broad_phase_layer_filter;
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // Loop through all active bodies
        const num_bodies = self.body_ids.items.len;
        for (active_bodies) |b1_id| {
            const body1 = body_manager.getBody(b1_id);
            const layer1 = body1.getObjectLayer();

            // Expand the bounding box by the speculative contact distance
            var bounds1 = body1.getWorldSpaceBounds().*;
            bounds1.expandBy(Vec3.replicate(speculative_contact_distance));

            // For all other bodies
            for (0..num_bodies) |b2| {
                // Check if bodies can collide
                const b2_id = self.body_ids.items[b2];
                const body2 = body_manager.getBody(b2_id);
                if (!Body.findCollidingPairsCanCollide(body1, body2))
                    continue;

                // Check if layers can collide
                const layer2 = body2.getObjectLayer();
                if (!object_layer_pair_filter.shouldCollide(layer1, layer2))
                    continue;

                // Check if bounds overlap
                const bounds2 = body2.getWorldSpaceBounds();
                if (!bounds1.overlaps(bounds2.*))
                    continue;

                // Store overlapping pair
                pair_collector.addHit(&BodyPair.init(b1_id, b2_id));
            }
        }
    }

    pub fn getBounds(self: *const BroadPhaseBruteForce) AABox {
        const mutex = &self.mutable_fields.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        var bounds: AABox = .empty;
        for (self.body_ids.items) |b|
            bounds.encapsulate(self.bodyManager().getBody(b).getWorldSpaceBounds().*);
        return bounds;
    }
};

test "BroadPhaseBruteForce lowerBound is std::lower_bound" {
    const ids = [_]BodyID{ .init(1), .init(3), .init(3), .init(7) };
    try std.testing.expectEqual(@as(usize, 0), BroadPhaseBruteForce.lowerBound(&ids, .init(0)));
    try std.testing.expectEqual(@as(usize, 0), BroadPhaseBruteForce.lowerBound(&ids, .init(1)));
    try std.testing.expectEqual(@as(usize, 1), BroadPhaseBruteForce.lowerBound(&ids, .init(2)));
    try std.testing.expectEqual(@as(usize, 1), BroadPhaseBruteForce.lowerBound(&ids, .init(3)));
    try std.testing.expectEqual(@as(usize, 3), BroadPhaseBruteForce.lowerBound(&ids, .init(4)));
    try std.testing.expectEqual(@as(usize, 4), BroadPhaseBruteForce.lowerBound(&ids, .init(8)));
    try std.testing.expectEqual(@as(usize, 0), BroadPhaseBruteForce.lowerBound(&.{}, .init(8)));
}
