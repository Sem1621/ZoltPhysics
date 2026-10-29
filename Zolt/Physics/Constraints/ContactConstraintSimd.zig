//! Zolt addition, no Jolt file: solves the velocity constraints of the contacts of a batch of a parallel split 8 at a
//! time with SIMD (Roadmap Phase 11, Docs/Zolt/SimulationArchitecture.md "SIMD contact solver").
//! Status: complete
//!
//! ContactConstraintManager.solveVelocityConstraints solves one contact constraint after the other. In a batch of a
//! parallel split of the LargeIslandSplitter no dynamic body appears twice. So a contact constraint of such a batch
//! reads and writes only its own lambdas and the velocities of its own two bodies (a kinematic body is only read, a
//! static body is not read). The order in which the contacts of the batch are solved does not change a bit. This file
//! solves 8 dynamic vs dynamic contact constraints at once: lane i of every `@Vector(8, f32)` holds a value of
//! constraint i (structure of arrays).
//!
//! Rules (no result changes, Roadmap Phase 10 class A):
//! - Each lane runs the operations of the scalar ContactConstraintManager.solveVelocityConstraint in the same order
//!   with the same association. No FMA. Vec3.dot is `(x * x' + y * y') + (z * z' + 0)` like Vec3.reduceSum. math.max
//!   is `@select(f32, a < b, b, a)` (never `@max`), math.clamp is built from it and from math.min.
//!   mulRSqrtApproximate is `a / @sqrt(b)` (IEEE exact per lane). `-x` is fneg.
//! - A branch of the scalar code becomes a lane mask. A lane that does not take the branch keeps its old values
//!   exactly: `@select`, not arithmetic with a zero (that can turn -0 into +0).
//! - A contact point past the number of points of a lane reads point 0 of that lane again. Its point mask is false, so
//!   it applies nothing, and it stores back the lambda it read.
//! - A group of fewer than 8 constraints fills the free lanes with copies of lane 0. A copy computes the same bits as
//!   lane 0, so its lambda stores write the same values again. It never writes velocities.
//! - The velocities are written back with MotionProperties.applyLinearVelocityStep / applyAngularVelocityStep, only for
//!   the lanes that applied an impulse, like ContactConstraintManager's setVelocities.
//! - Contacts with a static or kinematic body and a group of fewer than `min_lanes` constraints use the scalar solver.
//!
//! Gates: PhysicsSystem calls `solveBatch` (or `solvePlanned`, below), which uses this solver only when `enabled` and
//! only for the batches of a parallel split (LargeIslandSplitter.Batch.parallel). `enabled` needs the LLVM backend (the
//! self-hosted backend of Zig 0.16 miscompiles some @Vector code), is off with `-Ddeterminism_log=true` (the order of the
//! log lines would change) and with `-Dsimd_contact_solver=false` (the A/B switch). The warm start and
//! storeAppliedImpulses stay scalar.
//!
//! The plan of a batch (Phase 13, class A). solveVelocityConstraints sorts the contacts of a batch into groups in every
//! velocity iteration, about 4% of Pyramid's instructions. The groups depend only on the batch's contacts, so they are
//! planned once per step:
//! - When `plans(batch.parallel, batch.contacts.len)` (a batch of a parallel split with min_lanes to max_chunk contacts),
//!   PhysicsSystem warm starts the batch in Jolt's order and then calls `planBatch`. It permutes the batch's own slice of
//!   the splitter's contact indices to [the dynamic vs dynamic constraints with 1 to 4 points in the group order of
//!   solveVelocityConstraints][the others in their original order] and writes `plan_marker | <number of the first>` to
//!   the batch's plan word (LargeIslandSplitter.batchPlanWord: an entry of contact_and_constraints_split_idx that
//!   splitIsland no longer needs). Each velocity iteration calls `solvePlanned`, which builds the groups of 8 straight
//!   from that prefix (the same groups as solveVelocityConstraints, which the bits do not depend on anyway) and solves a
//!   tail under min_lanes and the others with one call of the scalar solver.
//! - The plan word checks itself: splitIsland writes split indices (below 32) into these words, a plan has 0x8000 in
//!   its high 16 bits and a count no larger than the batch (`plannedCount`). A word without a plan makes solvePlanned run
//!   solveVelocityConstraints on the slice, with the same bits (any order of a parallel batch gives the same bits).
//!   PhysicsSystem asserts (with asserts on) that the word holds a plan.
//!
//! Why the warm start of a batch always comes before its velocity iterations, within one step:
//! - A split island's status starts at iteration 0 (startFirstBatch, the last statement of splitIsland). The status moves
//!   to the next split only in the markBatchProcessed that completes every item of the split, and to the next iteration
//!   only after the non parallel split. So a batch of iteration i >= 1 is fetched only after every batch of iteration 0
//!   (warm start and plan) was processed and marked.
//! - The contacts of a batch are a function of its split and its first item: the item counter restarts at 0 for every
//!   split and iteration and grows by batch_size in fetchAdd (a fetchAdd on a status that changed after the pre-check
//!   claims item 0, 16, ... of the new split). So iteration i solves exactly the slice that iteration 0 planned, in every
//!   iteration and also in the position iterations, and no two batches share a plan word (their first contacts differ,
//!   a batch without contacts is not planned).
//! - Visibility: the permutation and the plan reach the job of a later iteration through the chain that carries the
//!   lambdas of the warm start: the acq_rel fetchAdd of items_processed, the release store of the status, the acquire
//!   fetchAdd of the status. The position iterations start after every SolveVelocityConstraints job ended (job
//!   dependencies).
//! - 0 velocity steps (num_iterations 1): the warm start plans and no velocity iteration reads the plan;
//!   storeAppliedImpulses and the position iterations visit the permuted order (below).
//! - Several collision steps, islands split again in every step: each step prepares the splitter, splits every large
//!   island again and resets the splitter. splitIsland rewrites its island's whole range of both buffers (the split
//!   indices, then the contact and constraint indices from the island builder) before startFirstBatch, so no plan and no
//!   permutation of an earlier step survives, also when the temp allocator hands out the same memory again.
//!
//! Why the permuted order changes no bit, phase by phase. No dynamic body appears twice in a batch of a parallel split:
//! createConstraint activates the dynamic bodies of a contact before it links them, so assignSplit masks every one of
//! them (the same property lets Jolt solve the batches of a split on several threads at once). So:
//! - Warm start: runs before planBatch, in Jolt's order. (It would be order free too, like the velocity iterations.)
//! - Velocity iterations, SIMD and scalar: a contact reads and writes its own lambdas and the velocities of its dynamic
//!   bodies, reads a kinematic body, does not read a static body. The "any impulse applied" result is an OR that the
//!   batch path ignores. A lane gives the bits of the scalar solver whatever the other lanes hold (see above).
//! - storeAppliedImpulses (after the last velocity iteration): writes the lambdas of a constraint into its own cached
//!   manifold (cached_manifold_handle: one manifold per constraint, created under its unique SubShapeIDPair key) and
//!   nothing else.
//! - Position iterations: a contact recomputes the properties of its own contact points and moves and rotates its
//!   dynamic bodies (comptime `type == .dynamic`); kinematic and static bodies and the cached manifolds are only read.
//! - Nothing else depends on the order: after splitIsland, contact_and_constraint_indices is read only through the batch
//!   slices of fetchNextBatch (the phases above, and markBatchProcessed, which reads only their lengths). No listener
//!   runs in the solve phases, the bounds and sleep update of the final batch iterates the island's bodies, the debug
//!   renderer (-Ddebug_renderer) and the StateRecorder read the constraint buffer and the caches, never the splitter.
//!   The determinism log turns this file off.
//!
//! Gates of the plan: `plans` is comptime false when `enabled` is (-Dsimd_contact_solver=false,
//! -Ddeterminism_log=true, the self-hosted backend), so then nothing is ever permuted. planBatch and solvePlanned are
//! never inlined: the code of the solve job stays as it was for the islands that are not split (callgrind, Ragdoll 15
//! steps without a split island: +0.001% instructions; inlining solvePlanned gave +0.008% there and -0.1% in Pyramid).
//! Pyramid (60 steps): -2.45% instructions (4,148,175,876 to 4,046,679,602). planBatch places each constraint once at
//! a counted position: its first version copied buckets back with loops that LLVM turned into 5 memcpy calls per batch
//! (221,226 more instructions in Pyramid).
//!
//! LaneGroups (Phase 14, class A). solveGroup gathered the step constant data of its lanes from the contact constraints
//! in every velocity iteration (the transposed loads, the address math of the contact points, setLane's reads of the
//! bodies and motion properties) and scattered the lambdas back: 7% of Pyramid's instructions. A planned batch now
//! keeps that data in lane order for the whole step, in `LaneGroup`s (3,840 bytes on 64 bit targets, 480 per lane):
//! - Memory: one heap block, kept between updates (`SimdLanes`, ContactConstraintManager.simd_lanes): a 64 byte header
//!   with the bump counter of the step on its own cache line, then the LaneGroups. PhysicsSystem grows it before the jobs
//!   of an update, never in a job (SimulationArchitecture.md D5): to what the steps of the previous update asked for and
//!   at least to twice its size (the first time to `laneGroupBound`). A failed growth keeps the old block or none.
//! - Claim: at the warm start of a planned batch whose split island has velocity steps, after planBatch, PhysicsSystem
//!   adds numLaneGroups(the planned count) to the counter. When the groups fit, `buildLanes` fills them and the batch's
//!   lane word (LargeIslandSplitter.batchLaneWord, the entry after the plan word) gets `laneWord(first)`, otherwise
//!   `no_lanes`. Lanes or gather is decided once per batch and step.
//! - Velocity iterations: `laneGroups` decodes the lane word (the marker 0x4c in its high 8 bits, the plan word, the
//!   groups lie in the block) and `solvePlannedLanes` solves the groups from their LaneGroups (`solveGroupFrom` with
//!   `LaneSource`: the kernel of solveGroup, only the operands come from the block), the scalar rest from the
//!   constraints. A word without the marker gathers (solvePlanned). A word with the marker whose plan word holds no plan
//!   or whose groups do not lie in the block asserts with asserts on and gathers otherwise (not reachable).
//! - Write back: after the batch's last velocity iteration, in the job that marked it processed and before
//!   storeAppliedImpulses, `writeBackLanes` copies the lambdas of the lanes below each group's count and of the points
//!   below each lane's count to the constraints.
//!
//! Why the LaneGroups change no bit:
//! - Build: buildLanes copies bits (the loads and transposes of the gather path, then vector stores) with no arithmetic,
//!   so NaN payloads and signed zeros stay. Its groups and lanes are those of solvePlanned: the same planned order, tail
//!   rule (min_lanes) and padding (copies of lane 0).
//! - The block stays current for the whole step: FindCollisions writes the step constant fields of the constraints
//!   before the warm start, and between the build and the write back only the batch's own kernel writes the lambdas of
//!   its contacts (a contact is in one batch, the iterations of a batch follow each other, see "Why the warm start of a
//!   batch always comes before its velocity iterations"). By induction over the iterations every slot of the block holds
//!   the bits that the gather path reads. The velocities stay in the motion properties, read and written as before.
//! - The slots that differ never leave their lane. A point at or past a lane's count holds the build time copy of the
//!   lane's point 0 where the gather path reads point 0's current value: its point mask keeps the max lambdas
//!   (@select), applies no impulse and keeps the slot's own lambda, and it is never written back. A padded lane (at or
//!   past `num`) is a copy of lane 0: masked out of the velocity write back and of the result, never written back (the
//!   gather path wrote lane 0's own value into lane 0's constraint again).
//! - Write back: once the last iteration is marked, no other job reads these lambdas before storeAppliedImpulses in
//!   the same job; the position iterations start after every SolveVelocityConstraints job and read no lambda. So
//!   storeAppliedImpulses, the contact cache, the debug renderer and the StateRecorder see the bits of the gather path.
//! - 0 velocity steps: no LaneGroups (the warm start is the batch's last iteration, nothing would read them).
//! - Visibility: the block and the lane word reach the jobs of the later iterations through the chain that carries the
//!   plan (the acq_rel fetchAdd of items_processed, the release store and the acquire fetchAdd of the status).
//! - Across steps: jobFinalizeIslands resets the counter after the previous step's solve jobs ended and before this
//!   step's start, every planned warm start writes its lane word, and splitIsland rewrites the words of its island
//!   before startFirstBatch. Which groups a batch gets depends on the claim order (the threads), which only selects
//!   memory; a batch that finds no room gathers, with the same bits.
//! Gates: as the plans (comptime off without `enabled`). buildLanes, solvePlannedLanes and writeBackLanes are never
//! inlined. Callgrind (-t=1): Pyramid (60 steps) -6.87% instructions, -5.92% data side cycle estimate; Ragdoll (80
//! steps) -0.46% and -0.43%; the other benchmark scenes change by less than 0.1% (SimulationArchitecture.md section 10).

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const MotionProperties = @import("../Body/MotionProperties.zig").MotionProperties;
const DeterminismLog = @import("../DeterminismLog.zig");
const ContactConstraintManager = @import("ContactConstraintManager.zig").ContactConstraintManager;
const ContactConstraintPart = @import("ConstraintPart/ContactConstraintPart.zig").ContactConstraintPart;
const AngularFrictionConstraintPart = @import("ConstraintPart/AngularFrictionConstraintPart.zig").AngularFrictionConstraintPart;

/// The SIMD kernel can be used: it needs the LLVM backend
pub const supported = builtin.zig_backend == .stage2_llvm;

/// PhysicsSystem solves the contacts of a parallel batch with this file
pub const enabled = supported and Core.simd_contact_solver and !DeterminismLog.enabled;

/// Number of contact constraints that are solved at once
pub const lanes = 8;

/// A group with fewer dynamic vs dynamic constraints than this uses the scalar solver
pub const min_lanes = 4;

const V = @Vector(lanes, f32);
const M = @Vector(lanes, bool);
const U = @Vector(lanes, u32);
const Row = @Vector(4, f32);

const Constraint = ContactConstraintManager.ContactConstraint(.dynamic, .dynamic);
const ContactPoint = Constraint.ContactPoint;
const Part = ContactConstraintPart(.dynamic, .dynamic);
const AngularPart = AngularFrictionConstraintPart(.dynamic, .dynamic);

const zero: V = @splat(0.0);
const false_mask: M = @splat(false);

/// Byte offset of the member at `path` (a field name or a dotted path of nested fields) in a `T`
fn offsetOf(comptime T: type, comptime path: []const u8) usize {
    return comptime result: {
        var offset: usize = 0;
        var U2 = T;
        var it = std.mem.splitScalar(u8, path, '.');
        while (it.next()) |name| {
            offset += @offsetOf(U2, name);
            U2 = @FieldType(U2, name);
        }
        break :result offset;
    };
}

// The layout that the loads below read: the members of a part are consecutive floats
const off_normal = offsetOf(Constraint, "base.world_space_normal");
const off_tangent1 = offsetOf(Constraint, "base.world_space_tangent1");
const off_tangent2 = offsetOf(Constraint, "base.world_space_tangent2");
const off_inv_mass1 = offsetOf(Constraint, "base.inv_mass1");
const off_friction1 = offsetOf(Constraint, "friction_constraint1");
const off_friction2 = offsetOf(Constraint, "friction_constraint2");
const off_angular = offsetOf(Constraint, "angular_friction_constraint");
const off_points = offsetOf(Constraint, "contact_points");
const off_point_lambda = offsetOf(ContactPoint, "non_penetration_constraint.base1.total_lambda");
const off_point_distance = offsetOf(ContactPoint, "distance_to_friction_center");
const off_part_lambda = offsetOf(Part, "base1.total_lambda");
const off_angular_lambda = offsetOf(AngularPart, "base1.base.total_lambda");

comptime {
    // Base: normal, tangent1, tangent2, combined_friction (read as the 4th float of tangent2), inv_mass1,
    // inv_inertia_scale1, inv_mass2
    std.debug.assert(off_tangent1 == off_normal + 12);
    std.debug.assert(off_tangent2 == off_normal + 24);
    std.debug.assert(offsetOf(Constraint, "base.combined_friction") == off_tangent2 + 12);
    std.debug.assert(offsetOf(Constraint, "base.inv_mass2") == off_inv_mass1 + 8);

    // ContactConstraintPart(.dynamic, .dynamic): 15 floats
    std.debug.assert(offsetOf(Part, "base1.effective_mass") == 0);
    std.debug.assert(off_part_lambda == 4);
    std.debug.assert(offsetOf(Part, "base1.r1_plus_u_x_axis") == 8);
    std.debug.assert(offsetOf(Part, "base1.inv_i1_r1_plus_u_x_axis") == 20);
    std.debug.assert(offsetOf(Part, "base2.r2_x_axis") == 32);
    std.debug.assert(offsetOf(Part, "base2.inv_i2_r2_x_axis") == 44);
    std.debug.assert(offsetOf(Part, "bias") == 56);
    std.debug.assert(@sizeOf(Part) == 60);

    // AngularFrictionConstraintPart(.dynamic, .dynamic): 9 floats
    std.debug.assert(offsetOf(AngularPart, "base1.base.effective_mass") == 0);
    std.debug.assert(off_angular_lambda == 4);
    std.debug.assert(offsetOf(AngularPart, "base1.inv_i1_axis") == 8);
    std.debug.assert(offsetOf(AngularPart, "base2.inv_i2_axis") == 20);
    std.debug.assert(offsetOf(AngularPart, "bias") == 32);
    std.debug.assert(@sizeOf(AngularPart) == 36);

    // A contact point is a part followed by the distance to the friction center
    std.debug.assert(offsetOf(ContactPoint, "non_penetration_constraint") == 0);
    std.debug.assert(off_point_distance == 60);
    std.debug.assert(@sizeOf(ContactPoint) == 64);
}

/// Transpose 8 rows of 4 floats: element j of the result holds float j of every row
inline fn transpose(r: [lanes]Row) [4]V {
    const a0: V = @shuffle(f32, r[0], r[4], [8]i32{ 0, 1, 2, 3, -1, -2, -3, -4 });
    const a1: V = @shuffle(f32, r[1], r[5], [8]i32{ 0, 1, 2, 3, -1, -2, -3, -4 });
    const a2: V = @shuffle(f32, r[2], r[6], [8]i32{ 0, 1, 2, 3, -1, -2, -3, -4 });
    const a3: V = @shuffle(f32, r[3], r[7], [8]i32{ 0, 1, 2, 3, -1, -2, -3, -4 });
    const t0: V = @shuffle(f32, a0, a1, [8]i32{ 0, -1, 1, -2, 4, -5, 5, -6 });
    const t1: V = @shuffle(f32, a0, a1, [8]i32{ 2, -3, 3, -4, 6, -7, 7, -8 });
    const t2: V = @shuffle(f32, a2, a3, [8]i32{ 0, -1, 1, -2, 4, -5, 5, -6 });
    const t3: V = @shuffle(f32, a2, a3, [8]i32{ 2, -3, 3, -4, 6, -7, 7, -8 });
    return .{
        @shuffle(f32, t0, t2, [8]i32{ 0, 1, -1, -2, 4, 5, -5, -6 }),
        @shuffle(f32, t0, t2, [8]i32{ 2, 3, -3, -4, 6, 7, -7, -8 }),
        @shuffle(f32, t1, t3, [8]i32{ 0, 1, -1, -2, 4, 5, -5, -6 }),
        @shuffle(f32, t1, t3, [8]i32{ 2, 3, -3, -4, 6, 7, -7, -8 }),
    };
}

/// Load 4 floats at `extra + offset` bytes past the pointer of every lane, transposed (see `transpose`)
inline fn load4(ptrs: *const [lanes][*]u8, extra: usize, comptime offset: usize) [4]V {
    var rows: [lanes]Row = undefined;
    inline for (0..lanes) |i|
        rows[i] = @as(*align(1) const Row, @ptrCast(ptrs[i] + extra + offset)).*;
    return transpose(rows);
}

/// Load the float at `extra + offset` bytes past the pointer of every lane
inline fn load1(ptrs: *const [lanes][*]u8, extra: usize, comptime offset: usize) V {
    var v: V = undefined;
    inline for (0..lanes) |i|
        v[i] = @as(*const f32, @ptrCast(@alignCast(ptrs[i] + extra + offset))).*;
    return v;
}

/// Store lane i of `v` at `extra + offset` bytes past the pointer of lane i
inline fn store1(ptrs: *const [lanes][*]u8, extra: usize, comptime offset: usize, v: V) void {
    inline for (0..lanes) |i|
        @as(*f32, @ptrCast(@alignCast(ptrs[i] + extra + offset))).* = v[i];
}

/// 8 Vec3s, one per lane
const Vec3x8 = struct {
    x: V,
    y: V,
    z: V,

    /// The first 3 vectors of a transposed load
    fn fromGroup(g: [4]V) Vec3x8 {
        return .{ .x = g[0], .y = g[1], .z = g[2] };
    }

    /// Vec3.add
    fn add(a: Vec3x8, b: Vec3x8) Vec3x8 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    /// Vec3.sub
    fn sub(a: Vec3x8, b: Vec3x8) Vec3x8 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    /// Vec3.mulScalar
    fn mulScalar(a: Vec3x8, s: V) Vec3x8 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
    }

    /// Vec3.dot: the products, then Vec3.reduceSum `(x + y) + (z + 0.0)`
    fn dot(a: Vec3x8, b: Vec3x8) V {
        const x = a.x * b.x;
        const y = a.y * b.y;
        const z = a.z * b.z;
        return (x + y) + (z + zero);
    }

    /// `a` in the lanes of `mask`, `b` in the others
    fn select(mask: M, a: Vec3x8, b: Vec3x8) Vec3x8 {
        return .{ .x = @select(f32, mask, a.x, b.x), .y = @select(f32, mask, a.y, b.y), .z = @select(f32, mask, a.z, b.z) };
    }
};

/// The velocities of the two bodies of every lane (ContactConstraintManager.Velocities)
const Velocities = struct {
    linear_velocity1: Vec3x8,
    angular_velocity1: Vec3x8,
    linear_velocity2: Vec3x8,
    angular_velocity2: Vec3x8,

    /// ContactConstraintPart.applyVelocityStep in the lanes of `applied` (dynamic vs dynamic). The other lanes keep their
    /// velocities.
    inline fn applyVelocityStep(v: *Velocities, inv_mass1: V, inv_mass2: V, world_space_axis: Vec3x8, inv_i1_r1_plus_u_x_axis: Vec3x8, inv_i2_r2_x_axis: Vec3x8, lambda: V, applied: M) void {
        if (@reduce(.And, applied)) {
            v.linear_velocity1 = v.linear_velocity1.sub(world_space_axis.mulScalar(lambda * inv_mass1));
            v.angular_velocity1 = v.angular_velocity1.sub(inv_i1_r1_plus_u_x_axis.mulScalar(lambda));
            v.linear_velocity2 = v.linear_velocity2.add(world_space_axis.mulScalar(lambda * inv_mass2));
            v.angular_velocity2 = v.angular_velocity2.add(inv_i2_r2_x_axis.mulScalar(lambda));
        } else if (@reduce(.Or, applied)) {
            v.linear_velocity1 = .select(applied, v.linear_velocity1.sub(world_space_axis.mulScalar(lambda * inv_mass1)), v.linear_velocity1);
            v.angular_velocity1 = .select(applied, v.angular_velocity1.sub(inv_i1_r1_plus_u_x_axis.mulScalar(lambda)), v.angular_velocity1);
            v.linear_velocity2 = .select(applied, v.linear_velocity2.add(world_space_axis.mulScalar(lambda * inv_mass2)), v.linear_velocity2);
            v.angular_velocity2 = .select(applied, v.angular_velocity2.add(inv_i2_r2_x_axis.mulScalar(lambda)), v.angular_velocity2);
        }
    }

    /// AngularFrictionConstraintPart.applyVelocityStep in the lanes of `applied` (dynamic vs dynamic). The other lanes
    /// keep their velocities.
    inline fn applyAngularVelocityStep(v: *Velocities, inv_i1_axis: Vec3x8, inv_i2_axis: Vec3x8, lambda: V, applied: M) void {
        if (@reduce(.And, applied)) {
            v.angular_velocity1 = v.angular_velocity1.sub(inv_i1_axis.mulScalar(lambda));
            v.angular_velocity2 = v.angular_velocity2.add(inv_i2_axis.mulScalar(lambda));
        } else if (@reduce(.Or, applied)) {
            v.angular_velocity1 = .select(applied, v.angular_velocity1.sub(inv_i1_axis.mulScalar(lambda)), v.angular_velocity1);
            v.angular_velocity2 = .select(applied, v.angular_velocity2.add(inv_i2_axis.mulScalar(lambda)), v.angular_velocity2);
        }
    }
};

/// ContactConstraintPart(.dynamic, .dynamic) of every lane
const PartX8 = struct {
    effective_mass: V,
    total_lambda: V,
    r1_plus_u_x_axis: Vec3x8,
    inv_i1_r1_plus_u_x_axis: Vec3x8,
    r2_x_axis: Vec3x8,
    inv_i2_r2_x_axis: Vec3x8,
    bias: V,

    /// Load the part at `extra` bytes past the pointer of every lane (4 transposed loads, none reads past the part)
    inline fn load(ptrs: *const [lanes][*]u8, extra: usize) PartX8 {
        const g0 = load4(ptrs, extra, 0); // effective_mass, total_lambda, r1_plus_u_x_axis.xy
        const g1 = load4(ptrs, extra, 16); // r1_plus_u_x_axis.z, inv_i1_r1_plus_u_x_axis
        const g2 = load4(ptrs, extra, 32); // r2_x_axis, inv_i2_r2_x_axis.x
        const g3 = load4(ptrs, extra, 44); // inv_i2_r2_x_axis, bias
        return .{
            .effective_mass = g0[0],
            .total_lambda = g0[1],
            .r1_plus_u_x_axis = .{ .x = g0[2], .y = g0[3], .z = g1[0] },
            .inv_i1_r1_plus_u_x_axis = .{ .x = g1[1], .y = g1[2], .z = g1[3] },
            .r2_x_axis = .{ .x = g2[0], .y = g2[1], .z = g2[2] },
            .inv_i2_r2_x_axis = .{ .x = g3[0], .y = g3[1], .z = g3[2] },
            .bias = g3[3],
        };
    }

    /// ContactConstraintPart.solveVelocityConstraintGetTotalLambda (dynamic vs dynamic)
    inline fn solveVelocityConstraintGetTotalLambda(self: *const PartX8, v: *const Velocities, world_space_axis: Vec3x8) V {
        // Calculate jacobian multiplied by linear velocity
        var jv = world_space_axis.dot(v.linear_velocity1.sub(v.linear_velocity2));

        // Calculate jacobian multiplied by angular velocity
        jv += self.r1_plus_u_x_axis.dot(v.angular_velocity1);
        jv -= self.r2_x_axis.dot(v.angular_velocity2);

        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = self.effective_mass * (jv - self.bias);

        // Return the total accumulated lambda
        return self.total_lambda + lambda;
    }

    /// ContactConstraintPart.solveVelocityConstraintApplyLambda in the lanes of `active` (all lanes when `all_active`).
    /// The other lanes keep their lambda and velocities. Returns the lanes that applied an impulse.
    inline fn solveVelocityConstraintApplyLambda(self: *PartX8, v: *Velocities, inv_mass1: V, inv_mass2: V, world_space_axis: Vec3x8, total_lambda: V, comptime all_active: bool, active: M) M {
        const delta_lambda = total_lambda - self.total_lambda; // Calculate change in lambda
        self.total_lambda = if (all_active) total_lambda else @select(f32, active, total_lambda, self.total_lambda); // Store accumulated impulse

        const applied = if (all_active) delta_lambda != zero else active & (delta_lambda != zero);
        v.applyVelocityStep(inv_mass1, inv_mass2, world_space_axis, self.inv_i1_r1_plus_u_x_axis, self.inv_i2_r2_x_axis, delta_lambda, applied);
        return applied;
    }
};

/// AngularFrictionConstraintPart(.dynamic, .dynamic) of every lane
const AngularPartX8 = struct {
    effective_mass: V,
    total_lambda: V,
    inv_i1_axis: Vec3x8,
    inv_i2_axis: Vec3x8,
    bias: V,

    /// Load the part at `extra` bytes past the pointer of every lane (2 transposed loads and the bias)
    inline fn load(ptrs: *const [lanes][*]u8, extra: usize) AngularPartX8 {
        const g0 = load4(ptrs, extra, 0); // effective_mass, total_lambda, inv_i1_axis.xy
        const g1 = load4(ptrs, extra, 16); // inv_i1_axis.z, inv_i2_axis
        return .{
            .effective_mass = g0[0],
            .total_lambda = g0[1],
            .inv_i1_axis = .{ .x = g0[2], .y = g0[3], .z = g1[0] },
            .inv_i2_axis = .{ .x = g1[1], .y = g1[2], .z = g1[3] },
            .bias = load1(ptrs, extra, 32),
        };
    }

    /// AngularFrictionConstraintPart.solveVelocityConstraint (dynamic vs dynamic) in the lanes of `active` (the other
    /// lanes keep their lambda and velocities). Returns the lanes that applied an impulse.
    inline fn solveVelocityConstraint(self: *AngularPartX8, v: *Velocities, world_space_axis: Vec3x8, min_lambda: V, max_lambda: V, active: M) M {
        const jv = world_space_axis.dot(v.angular_velocity1.sub(v.angular_velocity2));

        var lambda = self.effective_mass * (jv - self.bias);
        const new_lambda = clamp(self.total_lambda + lambda, min_lambda, max_lambda); // Clamp impulse
        lambda = new_lambda - self.total_lambda; // Lambda potentially got clamped, calculate the new impulse to apply
        self.total_lambda = @select(f32, active, new_lambda, self.total_lambda); // Store accumulated impulse

        const applied = active & (lambda != zero);
        v.applyAngularVelocityStep(self.inv_i1_axis, self.inv_i2_axis, lambda, applied);
        return applied;
    }
};

/// math.max per lane: `a` unless `a < b`
fn max(a: V, b: V) V {
    return @select(f32, a < b, b, a);
}

/// math.min per lane: `a` unless `b < a`
fn min(a: V, b: V) V {
    return @select(f32, b < a, b, a);
}

/// math.clamp per lane
fn clamp(v: V, lo: V, hi: V) V {
    return min(max(v, lo), hi);
}

/// A dynamic vs dynamic contact constraint and the motion properties of its bodies
const Lane = struct {
    constraint: [*]u8,
    motion_properties1: *MotionProperties,
    motion_properties2: *MotionProperties,
};

/// solveVelocityConstraints sorts at most this many constraints at a time, planBatch plans batches of at most this many
/// (a batch of a parallel split of the LargeIslandSplitter has at most 16 items; the non-parallel split comes as one
/// batch of any size, which the scalar solver solves)
pub const max_chunk = 64;

/// Up to 8 dynamic vs dynamic contact constraints
const Group = struct {
    /// The constraint of every lane (a ContactConstraint(.dynamic, .dynamic))
    constraints: [lanes][*]u8,
    motion_properties1: [lanes]*MotionProperties,
    motion_properties2: [lanes]*MotionProperties,
    num_contact_points: [lanes]u32,
};

/// The pointers to contact point `p` of every lane. A lane with fewer points gets its point 0 (see the header).
inline fn contactPoints(g: *const Group, p: u32) [lanes][*]u8 {
    var ptrs: [lanes][*]u8 = undefined;
    inline for (0..lanes) |i|
        ptrs[i] = g.constraints[i] + off_points + @as(usize, if (p < g.num_contact_points[i]) p else 0) * @sizeOf(ContactPoint);
    return ptrs;
}

/// Load one velocity of every lane, transposed
inline fn loadVelocities(mps: *const [lanes]*MotionProperties, comptime angular: bool) Vec3x8 {
    var rows: [lanes]Row = undefined;
    inline for (0..lanes) |i|
        rows[i] = (if (angular) mps[i].getAngularVelocity() else mps[i].getLinearVelocity()).value;
    return .fromGroup(transpose(rows));
}

/// The Vec3 of every lane (the inverse of `transpose`, W is the same as Z)
inline fn toVec3s(v: Vec3x8) [lanes]Vec3 {
    const t0: V = @shuffle(f32, v.x, v.y, [8]i32{ 0, -1, 1, -2, 4, -5, 5, -6 });
    const t1: V = @shuffle(f32, v.x, v.y, [8]i32{ 2, -3, 3, -4, 6, -7, 7, -8 });
    const t2: V = @shuffle(f32, v.z, v.z, [8]i32{ 0, -1, 1, -2, 4, -5, 5, -6 });
    const t3: V = @shuffle(f32, v.z, v.z, [8]i32{ 2, -3, 3, -4, 6, -7, 7, -8 });
    const a = [4]V{
        @shuffle(f32, t0, t2, [8]i32{ 0, 1, -1, -2, 4, 5, -5, -6 }),
        @shuffle(f32, t0, t2, [8]i32{ 2, 3, -3, -4, 6, 7, -7, -8 }),
        @shuffle(f32, t1, t3, [8]i32{ 0, 1, -1, -2, 4, 5, -5, -6 }),
        @shuffle(f32, t1, t3, [8]i32{ 2, 3, -3, -4, 6, 7, -7, -8 }),
    };
    var result: [lanes]Vec3 = undefined;
    inline for (0..4) |i| {
        result[i] = .{ .value = @shuffle(f32, a[i], undefined, [4]i32{ 0, 1, 2, 3 }) };
        result[i + 4] = .{ .value = @shuffle(f32, a[i], undefined, [4]i32{ 4, 5, 6, 7 }) };
    }
    return result;
}

/// Zolt (Phase 14, class A): where the solver kernel reads the step constant data and the lambdas of its lanes and
/// writes the lambdas back. GatherSource: from the contact constraints of a Group (transposed loads, scattered stores,
/// the path of Phase 11). LaneSource: from a LaneGroup that holds them in lane order. Both give the kernel the same
/// bits: the LaneGroup holds bit copies of what GatherSource would load (see "LaneGroups" at the top of this file).
const GatherSource = struct {
    g: *const Group,

    /// The contact point `p` of every lane (a lane with fewer points gets its point 0 when `masked`)
    const PointRef = struct {
        ptrs: [lanes][*]u8,
        extra: usize,
    };

    inline fn motionProperties1(s: GatherSource) *const [lanes]*MotionProperties {
        return &s.g.motion_properties1;
    }

    inline fn motionProperties2(s: GatherSource) *const [lanes]*MotionProperties {
        return &s.g.motion_properties2;
    }

    inline fn numContactPoints(s: GatherSource) *const [lanes]u32 {
        return &s.g.num_contact_points;
    }

    inline fn point(s: GatherSource, p: u32, comptime masked: bool) PointRef {
        return .{
            .ptrs = if (masked) contactPoints(s.g, p) else s.g.constraints,
            .extra = if (masked) 0 else off_points + p * @sizeOf(ContactPoint),
        };
    }

    inline fn pointLambda(_: GatherSource, r: *const PointRef) V {
        return load1(&r.ptrs, r.extra, off_point_lambda);
    }

    inline fn pointDistance(_: GatherSource, r: *const PointRef) V {
        return load1(&r.ptrs, r.extra, off_point_distance);
    }

    inline fn pointPart(_: GatherSource, r: *const PointRef) PartX8 {
        return .load(&r.ptrs, r.extra);
    }

    inline fn storePointLambda(_: GatherSource, r: *const PointRef, lambda: V) void {
        store1(&r.ptrs, r.extra, off_point_lambda, lambda);
    }

    inline fn combinedFriction(s: GatherSource) V {
        return load4(&s.g.constraints, 0, off_tangent2)[3]; // tangent2, combined_friction
    }

    inline fn inverseMasses(s: GatherSource) [2]V {
        const mass_group = load4(&s.g.constraints, 0, off_inv_mass1); // inv_mass1, inv_inertia_scale1, inv_mass2, inv_inertia_scale2
        return .{ mass_group[0], mass_group[2] };
    }

    inline fn frictionPart(s: GatherSource, comptime which: u1) PartX8 {
        return .load(&s.g.constraints, if (which == 0) off_friction1 else off_friction2);
    }

    inline fn storeFrictionLambda(s: GatherSource, comptime which: u1, lambda: V) void {
        store1(&s.g.constraints, if (which == 0) off_friction1 else off_friction2, off_part_lambda, lambda);
    }

    inline fn tangent1(s: GatherSource) Vec3x8 {
        return .fromGroup(load4(&s.g.constraints, 0, off_tangent1));
    }

    inline fn tangent2(s: GatherSource) Vec3x8 {
        return .fromGroup(load4(&s.g.constraints, 0, off_tangent2));
    }

    inline fn normal(s: GatherSource) Vec3x8 {
        return .fromGroup(load4(&s.g.constraints, 0, off_normal));
    }

    inline fn angularPart(s: GatherSource) AngularPartX8 {
        return .load(&s.g.constraints, off_angular);
    }

    inline fn storeAngularLambda(s: GatherSource, lambda: V) void {
        store1(&s.g.constraints, off_angular, off_angular_lambda, lambda);
    }
};

/// Zolt (Phase 14, class A): contact point `p` of the 8 lanes of a LaneGroup
const PointX8 = struct {
    non_penetration_constraint: PartX8,
    distance_to_friction_center: V,
};

/// Zolt (Phase 14, class A): the step constant data and the lambdas of a planned group of 8 dynamic vs dynamic contact
/// constraints in lane order (lane i of every vector belongs to constraint i), and the motion properties and point
/// counts of the lanes (see "LaneGroups" at the top of this file). Built once per step at the warm start of a planned
/// batch (buildLanes), read and updated in its velocity iterations (solvePlannedLanes); the lambdas go back to the
/// constraints after its last velocity iteration (writeBackLanes). The velocities stay in the motion properties.
/// 3,840 bytes on 64 bit targets: 480 bytes per lane.
/// - Point p of a lane with fewer points holds a copy of the lane's point 0 (what contactPoints gathers); its lambda is
///   never written back. Points from the group's largest number of points on are not written.
/// - Lanes from `num` on (a tail group) are copies of lane 0 and are never written back.
pub const LaneGroup = struct {
    friction_constraint1: PartX8,
    friction_constraint2: PartX8,
    angular_friction_constraint: AngularPartX8,
    points: [ContactConstraintManager.max_contact_points]PointX8,
    ws_normal: Vec3x8,
    tangent1: Vec3x8,
    tangent2: Vec3x8,
    combined_friction: V,
    inv_mass1: V,
    inv_mass2: V,
    motion_properties1: [lanes]*MotionProperties,
    motion_properties2: [lanes]*MotionProperties,
    num_contact_points: [lanes]u32,

    comptime {
        // 480 bytes per lane on 64 bit targets, in both precisions (only f32 parts and pointers: 8 bytes less per lane
        // with 4 byte pointers)
        if (@sizeOf(usize) == 8)
            std.debug.assert(@sizeOf(LaneGroup) == lanes * 480);
    }
};

const LaneSource = struct {
    b: *LaneGroup,

    const PointRef = *PointX8;

    inline fn motionProperties1(s: LaneSource) *const [lanes]*MotionProperties {
        return &s.b.motion_properties1;
    }

    inline fn motionProperties2(s: LaneSource) *const [lanes]*MotionProperties {
        return &s.b.motion_properties2;
    }

    inline fn numContactPoints(s: LaneSource) *const [lanes]u32 {
        return &s.b.num_contact_points;
    }

    inline fn point(s: LaneSource, p: u32, comptime masked: bool) PointRef {
        _ = masked;
        return &s.b.points[p];
    }

    inline fn pointLambda(_: LaneSource, r: *const PointRef) V {
        return r.*.non_penetration_constraint.total_lambda;
    }

    inline fn pointDistance(_: LaneSource, r: *const PointRef) V {
        return r.*.distance_to_friction_center;
    }

    inline fn pointPart(_: LaneSource, r: *const PointRef) PartX8 {
        return r.*.non_penetration_constraint;
    }

    inline fn storePointLambda(_: LaneSource, r: *const PointRef, lambda: V) void {
        r.*.non_penetration_constraint.total_lambda = lambda;
    }

    inline fn combinedFriction(s: LaneSource) V {
        return s.b.combined_friction;
    }

    inline fn inverseMasses(s: LaneSource) [2]V {
        return .{ s.b.inv_mass1, s.b.inv_mass2 };
    }

    inline fn frictionPart(s: LaneSource, comptime which: u1) PartX8 {
        return if (which == 0) s.b.friction_constraint1 else s.b.friction_constraint2;
    }

    inline fn storeFrictionLambda(s: LaneSource, comptime which: u1, lambda: V) void {
        if (which == 0) s.b.friction_constraint1.total_lambda = lambda else s.b.friction_constraint2.total_lambda = lambda;
    }

    inline fn tangent1(s: LaneSource) Vec3x8 {
        return s.b.tangent1;
    }

    inline fn tangent2(s: LaneSource) Vec3x8 {
        return s.b.tangent2;
    }

    inline fn normal(s: LaneSource) Vec3x8 {
        return s.b.ws_normal;
    }

    inline fn angularPart(s: LaneSource) AngularPartX8 {
        return s.b.angular_friction_constraint;
    }

    inline fn storeAngularLambda(s: LaneSource, lambda: V) void {
        s.b.angular_friction_constraint.total_lambda = lambda;
    }
};

/// Add the lambda of contact point `p` to the max impulses (the loop at the start of solveVelocityConstraint).
/// `masked`: lanes with fewer points skip the point.
inline fn addMaxLambda(comptime Source: type, src: Source, p: u32, comptime masked: bool, max_linear_lambda: *V, max_angular_lambda: *V) void {
    const r = src.point(p, masked);
    const lambda = src.pointLambda(&r);
    const distance_to_friction_center = src.pointDistance(&r);
    if (masked) {
        const point_active = @as(U, @splat(p)) < @as(U, src.numContactPoints().*);
        max_linear_lambda.* = @select(f32, point_active, max_linear_lambda.* + lambda, max_linear_lambda.*);
        max_angular_lambda.* = @select(f32, point_active, max_angular_lambda.* + distance_to_friction_center * lambda, max_angular_lambda.*);
    } else {
        max_linear_lambda.* += lambda;
        max_angular_lambda.* += distance_to_friction_center * lambda;
    }
}

/// Solve the non-penetration constraint of contact point `p`. `masked`: lanes with fewer points skip the point.
/// Returns the lanes that applied an impulse.
inline fn solveContactPoint(comptime Source: type, src: Source, p: u32, comptime masked: bool, v: *Velocities, ws_normal: Vec3x8, inv_mass1: V, inv_mass2: V) M {
    const r = src.point(p, masked);
    var non_penetration_constraint: PartX8 = src.pointPart(&r);

    // Calculate impulse
    var total_lambda = non_penetration_constraint.solveVelocityConstraintGetTotalLambda(v, ws_normal);

    // Contact constraints can only push and not pull
    total_lambda = max(total_lambda, zero);

    // Apply impulse
    const point_active = if (masked) @as(U, @splat(p)) < @as(U, src.numContactPoints().*) else undefined;
    const applied = non_penetration_constraint.solveVelocityConstraintApplyLambda(v, inv_mass1, inv_mass2, ws_normal, total_lambda, !masked, point_active);
    src.storePointLambda(&r, non_penetration_constraint.total_lambda);
    return applied;
}

/// ContactConstraintManager.solveVelocityConstraint for the dynamic vs dynamic constraints of a group. Lanes `num`
/// and up are copies of lane 0. Returns true when a lane below `num` applied an impulse.
fn solveGroup(g: *const Group, num: u32) bool {
    return solveGroupFrom(GatherSource, .{ .g = g }, num);
}

/// Zolt (Phase 14, class A): solveGroup for a LaneGroup
fn solveGroupLanes(b: *LaneGroup, num: u32) bool {
    return solveGroupFrom(LaneSource, .{ .b = b }, num);
}

/// The kernel of solveGroup with the step constant data and the lambdas from `src` (GatherSource or LaneSource)
inline fn solveGroupFrom(comptime Source: type, src: Source, num: u32) bool {
    const motion_properties1 = src.motionProperties1();
    const motion_properties2 = src.motionProperties2();

    // Get velocities
    var v: Velocities = .{
        .linear_velocity1 = loadVelocities(motion_properties1, false),
        .angular_velocity1 = loadVelocities(motion_properties1, true),
        .linear_velocity2 = loadVelocities(motion_properties2, false),
        .angular_velocity2 = loadVelocities(motion_properties2, true),
    };

    var any_impulse_applied = false_mask;

    // The number of contact points of the lanes. Points below min_points exist in every lane.
    const min_points = @reduce(.Min, @as(U, src.numContactPoints().*));
    const max_points = @reduce(.Max, @as(U, src.numContactPoints().*));

    // Calculate max impulse that can be applied. Note that we're using the non-penetration impulse from the previous iteration here.
    // We do this because non-penetration is more important so is solved last (the last things that are solved in an iterative solver
    // contribute the most).
    // (Zolt: computed in every lane, used only in the lanes with active friction)
    var max_linear_lambda = zero;
    var max_angular_lambda = zero;
    {
        var p: u32 = 0;
        while (p < min_points) : (p += 1)
            addMaxLambda(Source, src, p, false, &max_linear_lambda, &max_angular_lambda);
        while (p < max_points) : (p += 1)
            addMaxLambda(Source, src, p, true, &max_linear_lambda, &max_angular_lambda);
        const combined_friction = src.combinedFriction();
        max_linear_lambda *= combined_friction;
        max_angular_lambda *= combined_friction;
    }

    const inv_masses = src.inverseMasses();
    const inv_mass1 = inv_masses[0];
    const inv_mass2 = inv_masses[1];

    // First apply friction constraint (non-penetration is more important than friction)
    {
        var friction_constraint1: PartX8 = src.frictionPart(0);
        var friction_constraint2: PartX8 = src.frictionPart(1);
        const linear_friction_active = (friction_constraint1.effective_mass != zero) | (friction_constraint2.effective_mass != zero);
        if (@reduce(.Or, linear_friction_active)) {
            // Calculate tangents
            const tangent1: Vec3x8 = src.tangent1();
            const tangent2: Vec3x8 = src.tangent2();

            // Calculate impulse to stop motion in tangential direction
            var lambda1 = friction_constraint1.solveVelocityConstraintGetTotalLambda(&v, tangent1);
            var lambda2 = friction_constraint2.solveVelocityConstraintGetTotalLambda(&v, tangent2);

            // If the total lambda that we will apply is too large, scale it back
            const total_lambda_sq = lambda1 * lambda1 + lambda2 * lambda2;
            const scale_back = total_lambda_sq > max_linear_lambda * max_linear_lambda + @as(V, @splat(math.flt_min)); // ensure total_lambda_sq > FLT_MIN to avoid division by zero in MulRSqrtApproximate
            if (@reduce(.Or, scale_back)) {
                const scale = max_linear_lambda / @sqrt(total_lambda_sq);
                lambda1 = @select(f32, scale_back, lambda1 * scale, lambda1);
                lambda2 = @select(f32, scale_back, lambda2 * scale, lambda2);
            }

            // Apply the friction impulse
            if (@reduce(.And, linear_friction_active)) {
                any_impulse_applied |= friction_constraint1.solveVelocityConstraintApplyLambda(&v, inv_mass1, inv_mass2, tangent1, lambda1, true, undefined);
                any_impulse_applied |= friction_constraint2.solveVelocityConstraintApplyLambda(&v, inv_mass1, inv_mass2, tangent2, lambda2, true, undefined);
            } else {
                any_impulse_applied |= friction_constraint1.solveVelocityConstraintApplyLambda(&v, inv_mass1, inv_mass2, tangent1, lambda1, false, linear_friction_active);
                any_impulse_applied |= friction_constraint2.solveVelocityConstraintApplyLambda(&v, inv_mass1, inv_mass2, tangent2, lambda2, false, linear_friction_active);
            }
            src.storeFrictionLambda(0, friction_constraint1.total_lambda);
            src.storeFrictionLambda(1, friction_constraint2.total_lambda);
        }
    }

    const ws_normal: Vec3x8 = src.normal();

    // Apply angular friction
    {
        var angular_friction_constraint: AngularPartX8 = src.angularPart();
        const angular_friction_active = angular_friction_constraint.effective_mass != zero;
        if (@reduce(.Or, angular_friction_active)) {
            any_impulse_applied |= angular_friction_constraint.solveVelocityConstraint(&v, ws_normal, -max_angular_lambda, max_angular_lambda, angular_friction_active);
            src.storeAngularLambda(angular_friction_constraint.total_lambda);
        }
    }

    // Then apply all non-penetration constraints
    {
        var p: u32 = 0;
        while (p < min_points) : (p += 1)
            any_impulse_applied |= solveContactPoint(Source, src, p, false, &v, ws_normal, inv_mass1, inv_mass2);
        while (p < max_points) : (p += 1)
            any_impulse_applied |= solveContactPoint(Source, src, p, true, &v, ws_normal, inv_mass1, inv_mass2);
    }

    // Apply changed velocities (ContactConstraintManager.setVelocities)
    const valid = std.simd.iota(u32, lanes) < @as(U, @splat(num));
    const write_back: u8 = @bitCast(any_impulse_applied & valid);
    if (write_back == 0)
        return false;
    const linear_velocity1 = toVec3s(v.linear_velocity1);
    const angular_velocity1 = toVec3s(v.angular_velocity1);
    const linear_velocity2 = toVec3s(v.linear_velocity2);
    const angular_velocity2 = toVec3s(v.angular_velocity2);
    inline for (0..lanes) |i| {
        if (write_back & (1 << i) != 0) {
            motion_properties1[i].applyLinearVelocityStep(linear_velocity1[i]);
            motion_properties1[i].applyAngularVelocityStep(angular_velocity1[i]);
            motion_properties2[i].applyLinearVelocityStep(linear_velocity2[i]);
            motion_properties2[i].applyAngularVelocityStep(angular_velocity2[i]);
        }
    }
    return true;
}

/// ContactConstraintManager.solveVelocityConstraints for the contacts of a batch of a parallel split: no dynamic body
/// may appear in two of the constraints. The dynamic vs dynamic constraints are solved 8 at a time, the others with the
/// scalar solver. Returns true when any impulse was applied.
pub fn solveVelocityConstraints(manager: *ContactConstraintManager, constraint_offsets: []const u32) bool {
    // Too few constraints for a group
    if (constraint_offsets.len < min_lanes)
        return manager.solveVelocityConstraints(constraint_offsets);

    var any_impulse_applied = false;

    var remaining = constraint_offsets;
    while (remaining.len > 0) {
        const chunk = remaining[0..@min(remaining.len, max_chunk)];
        remaining = remaining[chunk.len..];

        // Sort the dynamic vs dynamic constraints by their number of contact points, so that the lanes of a group do
        // the same work. The order does not change a bit (see the top of this file).
        var buckets: [ContactConstraintManager.max_contact_points][max_chunk]Lane = undefined;
        var bucket_sizes: [ContactConstraintManager.max_contact_points]u32 = @splat(0);
        for (chunk, 0..) |constraint_offset, i| {
            const constraint = manager.getConstraintBase(constraint_offset);
            const body1 = constraint.body1;
            const body2 = constraint.body2;
            const bucket = constraint.num_contact_points -% 1;
            if (body1.isDynamic() and body2.isDynamic() and bucket < ContactConstraintManager.max_contact_points) {
                buckets[bucket][bucket_sizes[bucket]] = .{
                    .constraint = @ptrCast(constraint),
                    .motion_properties1 = body1.getMotionPropertiesUncheckedMut().?,
                    .motion_properties2 = body2.getMotionPropertiesUncheckedMut().?,
                };
                bucket_sizes[bucket] += 1;
            } else {
                // Contacts with a static or kinematic body: the scalar solver
                any_impulse_applied = manager.solveVelocityConstraints(chunk[i..][0..1]) or any_impulse_applied;
            }
        }

        // Most contact points first, so that the tail has the fewest
        var group: Group = undefined;
        var num: u32 = 0;
        var bucket: u32 = ContactConstraintManager.max_contact_points;
        while (bucket > 0) {
            bucket -= 1;
            for (buckets[bucket][0..bucket_sizes[bucket]]) |lane| {
                group.constraints[num] = lane.constraint;
                group.motion_properties1[num] = lane.motion_properties1;
                group.motion_properties2[num] = lane.motion_properties2;
                group.num_contact_points[num] = @intCast(bucket + 1);
                num += 1;
                if (num == lanes) {
                    any_impulse_applied = solveGroup(&group, lanes) or any_impulse_applied;
                    num = 0;
                }
            }
        }

        if (num >= min_lanes) {
            // Fill the free lanes with copies of lane 0
            for (num..lanes) |i| {
                group.constraints[i] = group.constraints[0];
                group.motion_properties1[i] = group.motion_properties1[0];
                group.motion_properties2[i] = group.motion_properties2[0];
                group.num_contact_points[i] = group.num_contact_points[0];
            }
            any_impulse_applied = solveGroup(&group, num) or any_impulse_applied;
        } else {
            // (Zolt: through solveVelocityConstraints, so that its only call of solveVelocityConstraint stays inlined)
            for (group.constraints[0..num]) |constraint| {
                const constraint_offset: u32 = @intCast(constraint - manager.constraints.items.ptr);
                any_impulse_applied = manager.solveVelocityConstraints(&.{constraint_offset}) or any_impulse_applied;
            }
        }
    }

    return any_impulse_applied;
}

/// Solve the velocity constraints of the contacts of a batch of the LargeIslandSplitter: with this file when `enabled`
/// and the batch comes from a parallel split, with ContactConstraintManager.solveVelocityConstraints otherwise.
pub fn solveBatch(manager: *ContactConstraintManager, constraint_offsets: []const u32, parallel: bool) bool {
    if (enabled and parallel)
        return solveVelocityConstraints(manager, constraint_offsets);
    return manager.solveVelocityConstraints(constraint_offsets);
}

/// Zolt (Phase 13, class A): the marker in the high 16 bits of a plan word (see "The plan of a batch" at the top of
/// this file). LargeIslandSplitter.splitIsland writes split indices (below 32) into the words that hold the plans, so a
/// word without the marker holds no plan.
pub const plan_marker: u32 = 0x8000_0000;

/// The bits of a plan word that hold the marker, the others hold the number of planned dynamic vs dynamic contacts
pub const plan_marker_mask: u32 = 0xffff_0000;

/// Zolt (Phase 13, class A): true when the warm start of a batch plans its groups and its velocity iterations solve
/// them with solvePlanned: the solver is enabled and the batch of `num_contacts` contacts comes from a parallel split
/// and can fill a group (min_lanes to max_chunk contacts). Comptime false when the solver is not enabled
/// (-Dsimd_contact_solver=false, -Ddeterminism_log=true, the self-hosted backend): then nothing is ever permuted.
pub inline fn plans(parallel: bool, num_contacts: usize) bool {
    return enabled and parallel and num_contacts >= min_lanes and num_contacts <= max_chunk;
}

/// The number of planned dynamic vs dynamic contacts that the plan word of a batch of `num_contacts` contacts holds,
/// null when the word holds no plan (no marker, or a count above the number of contacts)
pub fn plannedCount(plan_word: u32, num_contacts: usize) ?u32 {
    if (plan_word & plan_marker_mask != plan_marker)
        return null;
    const count = plan_word & ~plan_marker_mask;
    return if (count <= num_contacts) count else null;
}

/// Zolt (Phase 13, class A): plan the groups of a batch of a parallel split once per step, at its warm start (see "The
/// plan of a batch" at the top of this file). Permutes the batch's own contact offsets in place to [the dynamic vs
/// dynamic constraints in the group order of solveVelocityConstraints: most contact points first, then the original
/// order][the other constraints in their original order] and writes `plan_marker | <number of dynamic vs dynamic
/// constraints>` to `plan_word`. Never inlined, so that the code of the solve job stays as it was for the batches and
/// islands without a plan.
pub noinline fn planBatch(manager: *const ContactConstraintManager, constraint_offsets: []u32, plan_word: *u32) void {
    std.debug.assert(constraint_offsets.len >= min_lanes and constraint_offsets.len <= max_chunk);

    // The bucket of each constraint: the buckets of solveVelocityConstraints (by number of contact points) and the
    // others. A key holds the bucket in its high half and the constraint offset in its low half.
    const others = ContactConstraintManager.max_contact_points;
    var keys: [max_chunk]u64 = undefined;
    var bucket_sizes: [others + 1]u32 = @splat(0);
    for (constraint_offsets, keys[0..constraint_offsets.len]) |constraint_offset, *key| {
        const constraint = manager.getConstraintBase(constraint_offset);
        const points: u32 = constraint.num_contact_points -% 1;
        const bucket = if (constraint.body1.isDynamic() and constraint.body2.isDynamic() and points < others) points else others;
        bucket_sizes[bucket] += 1;
        key.* = @as(u64, bucket) << 32 | constraint_offset;
    }

    // The first position of each bucket: most contact points first, so that the tail has the fewest, the others last
    var next: [others + 1]u32 = undefined;
    var num_dynamic: u32 = 0;
    var bucket: u32 = others;
    while (bucket > 0) {
        bucket -= 1;
        next[bucket] = num_dynamic;
        num_dynamic += bucket_sizes[bucket];
    }
    next[others] = num_dynamic;

    // Each constraint to the next position of its bucket, so a bucket keeps the original order (one store per
    // constraint, no copy loop that LLVM turns into memcpy calls)
    for (keys[0..constraint_offsets.len]) |key| {
        const b: u32 = @intCast(key >> 32);
        constraint_offsets[next[b]] = @truncate(key);
        next[b] += 1;
    }

    plan_word.* = plan_marker | num_dynamic;
}

/// Lane `l` of a group: the dynamic vs dynamic constraint at `constraint_offset`
inline fn setLane(manager: *const ContactConstraintManager, group: *Group, l: usize, constraint_offset: u32) void {
    const constraint = manager.getConstraintBase(constraint_offset);
    group.constraints[l] = @ptrCast(constraint);
    group.motion_properties1[l] = constraint.body1.getMotionPropertiesUncheckedMut().?;
    group.motion_properties2[l] = constraint.body2.getMotionPropertiesUncheckedMut().?;
    group.num_contact_points[l] = constraint.num_contact_points;
}

/// Zolt (Phase 13, class A): solveVelocityConstraints for a batch that planBatch planned in this step (its contact
/// offsets in the planned order, `plan_word` the word that planBatch wrote): the groups of 8 are built straight from
/// the planned dynamic vs dynamic constraints, the rest goes through the scalar solver in one call. A `plan_word`
/// without a plan (plannedCount is null) takes solveVelocityConstraints, which gives the same bits. Returns true when
/// any impulse was applied. Never inlined (see planBatch).
pub noinline fn solvePlanned(manager: *ContactConstraintManager, constraint_offsets: []const u32, plan_word: u32) bool {
    const num_dynamic = plannedCount(plan_word, constraint_offsets.len) orelse return solveVelocityConstraints(manager, constraint_offsets);

    var any_impulse_applied = false;

    // Full groups
    var group: Group = undefined;
    var i: u32 = 0;
    while (num_dynamic - i >= lanes) : (i += lanes) {
        inline for (0..lanes) |l|
            setLane(manager, &group, l, constraint_offsets[i + l]);
        any_impulse_applied = solveGroup(&group, lanes) or any_impulse_applied;
    }

    // The tail when it can fill a group
    const num = num_dynamic - i;
    if (num >= min_lanes) {
        for (0..num) |l|
            setLane(manager, &group, l, constraint_offsets[i + l]);

        // Fill the free lanes with copies of lane 0
        for (num..lanes) |l| {
            group.constraints[l] = group.constraints[0];
            group.motion_properties1[l] = group.motion_properties1[0];
            group.motion_properties2[l] = group.motion_properties2[0];
            group.num_contact_points[l] = group.num_contact_points[0];
        }
        any_impulse_applied = solveGroup(&group, num) or any_impulse_applied;
        i = num_dynamic;
    }

    // The scalar solver: a tail of fewer than min_lanes dynamic vs dynamic constraints, then the contacts with a static
    // or kinematic body
    if (i < constraint_offsets.len)
        any_impulse_applied = manager.solveVelocityConstraints(constraint_offsets[i..]) or any_impulse_applied;

    return any_impulse_applied;
}

/// Zolt (Phase 14, class A): the number of LaneGroups of a batch with `num_dynamic` planned dynamic vs dynamic contacts:
/// its full groups and a tail of min_lanes or more (the groups of solvePlanned), floor((num_dynamic + lanes - min_lanes)
/// / lanes)
pub fn numLaneGroups(num_dynamic: u32) u32 {
    return num_dynamic / lanes + @intFromBool(num_dynamic % lanes >= min_lanes);
}

/// Zolt (Phase 14, class A): an upper bound on the LaneGroups that the planned batches of an island claim in a step:
/// `num_contacts` contacts in the island and at most `max_batches` batches in its parallel splits
/// (LargeIslandSplitter.maxParallelBatches). A planned batch has at least min_lanes contacts, and d planned contacts
/// take numLaneGroups(d) = floor((d + lanes - min_lanes) / lanes) groups. The contacts of the batches are disjoint, so
/// the island claims at most floor((num_contacts + (lanes - min_lanes) * b) / lanes) groups with b =
/// min(max_batches, num_contacts / min_lanes) planned batches.
pub fn laneGroupBound(num_contacts: u32, max_batches: u32) u64 {
    const batches: u64 = @min(max_batches, num_contacts / min_lanes);
    return (@as(u64, num_contacts) + (lanes - min_lanes) * batches) / lanes;
}

comptime {
    // numLaneGroups is floor((d + lanes - min_lanes) / lanes), which laneGroupBound adds up
    for (0..max_chunk + 1) |d|
        std.debug.assert(numLaneGroups(d) == (d + lanes - min_lanes) / lanes);
}

/// Zolt (Phase 14, class A): the marker in the high 8 bits of a lane word (see "LaneGroups" at the top of this file).
/// LargeIslandSplitter.splitIsland writes split indices (below 32) into the words that hold the lane words and a plan
/// word has 0x80 in its high 8 bits, so neither is taken for a lane word.
pub const lane_marker: u32 = 0x4c00_0000;

/// The bits of a lane word that hold the marker, the others hold the index of the batch's first LaneGroup
pub const lane_marker_mask: u32 = 0xff00_0000;

/// The index of a first LaneGroup is below this (24 bits): PhysicsSystem never holds more LaneGroups
pub const max_lane_groups: u32 = 1 << 24;

/// The lane word of a batch whose velocity iterations gather from the constraints (no marker)
pub const no_lanes: u32 = 0;

/// The lane word of a batch whose LaneGroups start at `first`
pub fn laneWord(first: u32) u32 {
    std.debug.assert(first < max_lane_groups);
    return lane_marker | first;
}

/// Zolt (Phase 14, class A): the heap memory of the LaneGroups of the planned batches of a step, kept between updates
/// (ContactConstraintManager.simd_lanes; PhysicsSystem grows it, claims from it and records the demand, see
/// "LaneGroups" at the top of this file). One block: a BlockHeader, then `capacity` LaneGroups. 16 bytes.
pub const SimdLanes = struct {
    /// The block (null: none, every planned batch gathers)
    block: ?*BlockHeader = null,
    /// The number of LaneGroups of the block (at most max_lane_groups)
    capacity: u32 = 0,
    /// The most LaneGroups that a step asked for since the last growth check: the claims of a step that had a block
    /// (the counter counts them also past the capacity), the bound of a step without one (laneGroupBound). See
    /// PhysicsSystem.growSimdLanes.
    demand: u32 = 0,

    /// The start of the block: the bump counter on its own cache line (PortingGuide section 9: an add per planned batch
    /// from every SolveVelocityConstraints job). The LaneGroups follow, cache line aligned.
    pub const BlockHeader = struct {
        /// The LaneGroups claimed in the current step (also past the capacity: then the claim failed)
        claimed: std.atomic.Value(u32) align(Core.cache_line_size) = .init(0),

        comptime {
            std.debug.assert(@sizeOf(BlockHeader) == Core.cache_line_size);
        }
    };

    /// The bytes of a block of `capacity` LaneGroups
    pub fn blockSize(capacity: u32) usize {
        return @sizeOf(BlockHeader) + @as(usize, capacity) * @sizeOf(LaneGroup);
    }

    /// The LaneGroups of the block (none without a block)
    pub fn groups(self: *const SimdLanes) []LaneGroup {
        const block = self.block orelse return &.{};
        const first: [*]LaneGroup = @ptrCast(@alignCast(@as([*]u8, @ptrCast(block)) + @sizeOf(BlockHeader)));
        return first[0..self.capacity];
    }

    /// Replace the block with a new one of `capacity` LaneGroups (its counter at 0). The new block is allocated before
    /// the old one is freed: on error.OutOfMemory nothing changes.
    pub fn replace(self: *SimdLanes, allocator: std.mem.Allocator, capacity: u32) std.mem.Allocator.Error!void {
        std.debug.assert(capacity > 0 and capacity <= max_lane_groups);
        // (A block larger than the address space, possible on 32 bit targets only, is a failed growth too)
        const groups_size = std.math.mul(usize, capacity, @sizeOf(LaneGroup)) catch return error.OutOfMemory;
        const size = std.math.add(usize, @sizeOf(BlockHeader), groups_size) catch return error.OutOfMemory;
        const memory = try allocator.alignedAlloc(u8, .fromByteUnits(Core.cache_line_size), size);
        self.free(allocator);
        const block: *BlockHeader = @ptrCast(memory.ptr);
        block.* = .{};
        self.block = block;
        self.capacity = capacity;
    }

    /// Free the block (between updates only); the demand is kept
    pub fn free(self: *SimdLanes, allocator: std.mem.Allocator) void {
        if (self.block) |block| {
            const memory: [*]align(Core.cache_line_size) u8 = @ptrCast(block);
            allocator.free(memory[0..blockSize(self.capacity)]);
        }
        self.block = null;
        self.capacity = 0;
    }

    comptime {
        // ContactConstraintManager keeps it in the free bytes at its end (16 bytes, 8 byte aligned on 64 bit targets)
        if (@sizeOf(usize) == 8)
            std.debug.assert(@sizeOf(SimdLanes) == 16 and @alignOf(SimdLanes) == 8);
    }
};

/// The LaneGroups of a planned batch (laneGroups)
pub const BatchLanes = struct {
    /// The number of planned dynamic vs dynamic contacts (plannedCount of the plan word)
    num_dynamic: u32,
    /// Its numLaneGroups(num_dynamic) LaneGroups
    groups: []LaneGroup,
};

/// Zolt (Phase 14, class A): the LaneGroups that the lane word of a planned batch of `num_contacts` contacts names in
/// `groups` (all the LaneGroups of the step), with the batch's plan word. Null when the word has no marker (the batch
/// gathers). Also null, never in a correct run, when the plan word holds no plan or the named groups do not lie in
/// `groups`: that asserts in builds with asserts, and the batch gathers otherwise (with the same bits, as long as all
/// its velocity iterations decide alike, which they do: they read the same two words and the same `groups`, which do
/// not change during an update).
pub fn laneGroups(lane_word: u32, plan_word: u32, num_contacts: usize, groups: []LaneGroup) ?BatchLanes {
    if (lane_word & lane_marker_mask != lane_marker)
        return null;
    const num_dynamic = plannedCount(plan_word, num_contacts) orelse {
        if (Core.enable_asserts) std.debug.assert(false); // A lane word is only written after planBatch
        return null;
    };
    const num_groups = numLaneGroups(num_dynamic);
    const first = lane_word & ~lane_marker_mask;
    if (num_groups == 0 or first > groups.len or num_groups > groups.len - first) {
        if (Core.enable_asserts) std.debug.assert(false); // The claim of the LaneGroups checked that they fit
        return null;
    }
    return .{ .num_dynamic = num_dynamic, .groups = groups[first..][0..num_groups] };
}

/// Copy the step constant data and the lambdas of a group into its LaneGroup (in place, field by field, with no
/// arithmetic: the bits, NaN payloads and signed zeros included). Points from the group's largest number of points on
/// are not written.
fn buildLaneGroup(g: *const Group, b: *LaneGroup) void {
    const c = &g.constraints;
    b.motion_properties1 = g.motion_properties1;
    b.motion_properties2 = g.motion_properties2;
    b.num_contact_points = g.num_contact_points;
    const tangent2_group = load4(c, 0, off_tangent2); // tangent2, combined_friction
    b.tangent2 = .fromGroup(tangent2_group);
    b.combined_friction = tangent2_group[3];
    b.tangent1 = .fromGroup(load4(c, 0, off_tangent1));
    b.ws_normal = .fromGroup(load4(c, 0, off_normal));
    const mass_group = load4(c, 0, off_inv_mass1); // inv_mass1, inv_inertia_scale1, inv_mass2, inv_inertia_scale2
    b.inv_mass1 = mass_group[0];
    b.inv_mass2 = mass_group[2];
    b.friction_constraint1 = .load(c, off_friction1);
    b.friction_constraint2 = .load(c, off_friction2);
    b.angular_friction_constraint = .load(c, off_angular);
    const max_points = @reduce(.Max, @as(U, g.num_contact_points));
    var p: u32 = 0;
    while (p < max_points) : (p += 1) {
        const points = contactPoints(g, p);
        b.points[p].non_penetration_constraint = .load(&points, 0);
        b.points[p].distance_to_friction_center = load1(&points, 0, off_point_distance);
    }
}

/// Zolt (Phase 14, class A): build the LaneGroups of a batch that planBatch planned in this step, after its warm start
/// (see "LaneGroups" at the top of this file): `constraint_offsets` in the planned order, `num_dynamic` the planned
/// count of its plan word, `groups` numLaneGroups(num_dynamic) LaneGroups. The groups and their lanes are those of
/// solvePlanned (the same tail rule and the same padding). Never inlined.
pub noinline fn buildLanes(manager: *const ContactConstraintManager, constraint_offsets: []const u32, num_dynamic: u32, groups: []LaneGroup) void {
    std.debug.assert(num_dynamic <= constraint_offsets.len and groups.len == numLaneGroups(num_dynamic));
    var group: Group = undefined;
    var i: u32 = 0;
    var k: usize = 0;
    while (num_dynamic - i >= lanes) : ({
        i += lanes;
        k += 1;
    }) {
        inline for (0..lanes) |l|
            setLane(manager, &group, l, constraint_offsets[i + l]);
        buildLaneGroup(&group, &groups[k]);
    }
    const num = num_dynamic - i;
    if (num >= min_lanes) {
        for (0..num) |l|
            setLane(manager, &group, l, constraint_offsets[i + l]);

        // Fill the free lanes with copies of lane 0
        for (num..lanes) |l| {
            group.constraints[l] = group.constraints[0];
            group.motion_properties1[l] = group.motion_properties1[0];
            group.motion_properties2[l] = group.motion_properties2[0];
            group.num_contact_points[l] = group.num_contact_points[0];
        }
        buildLaneGroup(&group, &groups[k]);
    }
}

/// Zolt (Phase 14, class A): true when a lane word has the marker of laneWord: the batch's warm start built LaneGroups
/// (solvePlannedLanes and writeBackLanes check the rest of the word)
pub inline fn hasLanes(lane_word: u32) bool {
    return lane_word & lane_marker_mask == lane_marker;
}

/// Zolt (Phase 14, class A): solvePlanned for a batch whose lane word has the marker (hasLanes): the groups read and
/// write the LaneGroups that the word names in `manager.simd_lanes` (laneGroups), the scalar rest (a tail under
/// min_lanes, then the contacts with a static or kinematic body) reads and writes the constraints. A word that names no
/// LaneGroups (not in a correct run, laneGroups asserts with asserts on) takes solvePlanned, with the same bits. Returns
/// true when any impulse was applied. Never inlined.
pub noinline fn solvePlannedLanes(manager: *ContactConstraintManager, constraint_offsets: []const u32, plan_word: u32, lane_word: u32) bool {
    const batch_lanes = laneGroups(lane_word, plan_word, constraint_offsets.len, manager.simd_lanes.groups()) orelse
        return solvePlanned(manager, constraint_offsets, plan_word);
    return solveLanes(manager, constraint_offsets, batch_lanes);
}

/// The groups of a batch from its LaneGroups, the rest with the scalar solver (solvePlannedLanes)
inline fn solveLanes(manager: *ContactConstraintManager, constraint_offsets: []const u32, batch_lanes: BatchLanes) bool {
    const num_dynamic = batch_lanes.num_dynamic;
    const groups = batch_lanes.groups;
    std.debug.assert(num_dynamic <= constraint_offsets.len and groups.len == numLaneGroups(num_dynamic));

    var any_impulse_applied = false;

    // Full groups
    var i: u32 = 0;
    var k: usize = 0;
    while (num_dynamic - i >= lanes) : ({
        i += lanes;
        k += 1;
    })
        any_impulse_applied = solveGroupLanes(&groups[k], lanes) or any_impulse_applied;

    // The tail when it can fill a group
    const num = num_dynamic - i;
    if (num >= min_lanes) {
        any_impulse_applied = solveGroupLanes(&groups[k], num) or any_impulse_applied;
        i = num_dynamic;
    }

    // The scalar solver: a tail of fewer than min_lanes dynamic vs dynamic constraints, then the contacts with a static
    // or kinematic body
    if (i < constraint_offsets.len)
        any_impulse_applied = manager.solveVelocityConstraints(constraint_offsets[i..]) or any_impulse_applied;

    return any_impulse_applied;
}

/// Zolt (Phase 14, class A): after the last velocity iteration of a batch, before storeAppliedImpulses: copy the
/// lambdas of the LaneGroups that the batch's lane word names (laneGroups, nothing when it names none) back to its
/// constraints. Never inlined.
pub noinline fn writeBackLanes(manager: *const ContactConstraintManager, constraint_offsets: []const u32, plan_word: u32, lane_word: u32) void {
    const batch_lanes = laneGroups(lane_word, plan_word, constraint_offsets.len, manager.simd_lanes.groups()) orelse return;
    writeBack(manager, constraint_offsets, batch_lanes);
}

/// The lambdas of the LaneGroups of a batch to its constraints (writeBackLanes): only the lanes below a group's number
/// of constraints (never a padded lane) and the points below a lane's number of points
inline fn writeBack(manager: *const ContactConstraintManager, constraint_offsets: []const u32, batch_lanes: BatchLanes) void {
    const num_dynamic = batch_lanes.num_dynamic;
    const groups = batch_lanes.groups;
    std.debug.assert(num_dynamic <= constraint_offsets.len and groups.len == numLaneGroups(num_dynamic));
    var i: u32 = 0;
    for (groups) |*b| {
        const num = @min(num_dynamic - i, lanes);
        inline for (0..lanes) |l| if (l < num) {
            const constraint = Constraint.fromBase(manager.getConstraintBase(constraint_offsets[i + l]));
            constraint.friction_constraint1.base1.total_lambda = b.friction_constraint1.total_lambda[l];
            constraint.friction_constraint2.base1.total_lambda = b.friction_constraint2.total_lambda[l];
            constraint.angular_friction_constraint.base1.base.total_lambda = b.angular_friction_constraint.total_lambda[l];
            const points = constraint.getContactPoints();
            for (0..b.num_contact_points[l]) |p|
                points[p].non_penetration_constraint.base1.total_lambda = b.points[p].non_penetration_constraint.total_lambda[l];
        };
        i += num;
    }
}
