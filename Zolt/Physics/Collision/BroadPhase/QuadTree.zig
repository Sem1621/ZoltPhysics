//! Port of: Jolt/Physics/Collision/BroadPhase/QuadTree.h, Jolt/Physics/Collision/BroadPhase/QuadTree.cpp
//! Status: complete
//! Not ported: JPH_DUMP_BROADPHASE_TREE (DumpTree, commented out in Jolt), JPH_TRACK_BROADPHASE_STATS (GetTicks100Pct, ReportStats, mStatsMutex and the LayerToStats members, off in Jolt's default build, like BroadPhase.zig), JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (SetName, GetName, mName: Zolt has no profiler)
//!
//! SimulationArchitecture D9, a literal port: the partition code, the node allocator, the double-buffered root swap,
//! UpdatePrepare / UpdateFinalize, AddBodiesPrepare / Finalize / Abort, RemoveBodies, NotifyBodiesAABBChanged,
//! FindCollidingPairs and every query walk the tree with Jolt's code, in Jolt's order, with Jolt's atomics.
//! - Node bounds are `[4]std.atomic.Value(f32)` per axis (Jolt: `atomic<float>`). The setters and getters keep
//!   Jolt's implicit `seq_cst` operations, EncapsulateChildBounds widens them with Jolt's AtomicMin / AtomicMax
//!   (compare-exchange). The tree walks read the 4 bounds of one axis and the 4 child IDs like Jolt: with one aligned
//!   16 byte vector load each (`Vec4::sLoadFloat4Aligned`, `UVec4::sLoadInt4Aligned`). This load races with the
//!   atomic stores of other threads (Jolt does not annotate it for TSAN). Zolt does the load with x86-64 inline asm
//!   (`Node.vectorLoad`). The compiler cannot see into inline asm, so it is not a data race in Zig's memory model
//!   (rule 3). The hardware reads each aligned 4 byte lane atomically, which is all that Jolt relies on.
//!   The child ID load must have acquire order. A node is constructed with plain stores
//!   (FixedSizeFreeList.constructObject copies it in) and published by the `seq_cst` compare-exchange of TryInsertLeaf /
//!   TryCreateNewRoot (or the `seq_cst` root stores). The acquire load of the ID that leads to a node orders its
//!   construction before the walk reads it. x86 does not reorder a load with later loads, so the asm only needs a
//!   `memory` clobber, which stops the compiler from moving loads above it.
//!   Other CPUs, the self-hosted backend (its assembler does not take these operands) and TSAN builds load per lane
//!   instead: the bounds with `.monotonic`, the child IDs with `.acquire` (with `.monotonic` that is a data race,
//!   which TSAN reports). All paths load the same bits. Every other atomic keeps Jolt's (default `seq_cst`) memory
//!   order.
//! - Nodes come from a `NodeAllocator` (Jolt's `QuadTree::Allocator`, a FixedSizeFreeList of 128 byte nodes) whose
//!   pages must all be committed (`reserveAllPages`, BroadPhaseQuadTree does it in initBroadPhase), so AllocateNode
//!   never needs memory: AddBodiesFinalize, AddBodiesAbort and UpdateFinalize cannot fail. Running out of nodes stays
//!   Jolt's abort: `@panic("QuadTree: Out of nodes!")`.
//! - Heap memory (Jolt's global allocator) is the `allocator` passed to `init`: Jolt's `new NodeID[]` (UpdatePrepare,
//!   kept between updates since Phase 13, see below), `new Vec3[]` (BuildTree) and the heap fallback of the walk stacks
//!   (`Array<..., STLLocalAllocator<..., cStackSize>>`, Zolt's STLLocalAllocator). Errors:
//!   - `updatePrepare` and `addBodiesPrepare` return Allocator.Error and leave the tree unchanged: the nodes that were
//!     batched to be freed are in use again (`FixedSizeFreeList.cancelBatch`), `is_dirty` is restored, and BuildTree
//!     fails before it changes anything.
//!   - `findCollidingPairs` returns Allocator.Error when its node stack cannot grow on the heap (it runs inside the
//!     FindCollisions job, which latches the error, D5).
//!   - The queries (`castRay`, `collideAABox`, `collideSphere`, `collidePoint`, `collideOrientedBox`, `castAABox`)
//!     return Allocator.Error when their node stack (or the fraction stack of the casts) cannot grow on the heap, the
//!     hits found so far were reported. Their stacks only leave their 128 local entries in a tree deeper than about 40
//!     levels (Jolt's "Performance warning: Stack full!"). BroadPhaseQuadTree's `castAABoxNoLock` passes the error on
//!     (it runs inside the FindCCDContacts job, which latches it, D5); the locking BroadPhaseQuery queries have no
//!     error channel (CollisionArchitecture rule 4) and panic there, where Jolt's allocator aborts.
//!   - The destructor (`deinit`) cannot fail: it threads its node stack through the `parent_node_index` of the nodes it
//!     frees (they are dead afterwards) instead of allocating one; the nodes are batched in Jolt's order.
//!   - ValidateTree (JPH_DEBUG, Debug builds) is skipped when its stack cannot grow.
//! - Jolt's fixed size stacks keep their size: BuildTree's `StackEntry stack[cStackSize / 4]` and AddBodiesAbort's
//!   `NodeID node_stack[cStackSize]`. Where Jolt only asserts that they don't overflow (undefined behavior in its
//!   release build: a BuildTree over bodies whose positions grow exponentially can be deeper than 32 levels), Zolt
//!   panics.
//! - Out parameters are return values: `getBodyLocation` (BodyLocation), `updatePrepare` (UpdateState),
//!   `addBodiesPrepare` (AddState), `buildTree` (BuildTreeResult), `partition` (the mid point). Pointer + count
//!   parameters are slices. `QuadTree::Allocator` is `NodeAllocator` (it would clash with std.mem.Allocator) and
//!   `mAllocator` is `node_allocator`.
//! - Jolt aligns `mNumBodies` (and the members after it) to a cache line, away from `mAllocator`. Zig reorders struct
//!   fields, so these members are grouped in `tree_state: TreeState` (aligned to, and occupying whole, cache lines).
//! - The private nested classes (NodeID, AtomicNodeID, Node, RootNode) are `pub` so that the parity test can walk the
//!   tree. `Tracking`'s copy constructor (for Jolt's `Array::resize`) is not needed: the tracking array is allocated
//!   once.
//! - `sQuadTreePerformanceWarning`'s `static atomic<bool> triggered_report` (only with asserts) is not a global: the
//!   tree points to the latch (`triggered_report`, `void` without asserts) in the `mutable_fields` of its
//!   BroadPhaseQuadTree, so the warning is reported once per broad phase instead of once per process (a tree without a
//!   broad phase does not report it).
//!
//! Differences with the C++ version (Roadmap Phase 13, class A: memory placement only, no result changes):
//! - UpdatePrepare's `new NodeID[mNumBodies]` and the `new Vec3[]` of the BuildTree that it calls are arrays that the
//!   tree keeps from one update to the next (`update_node_ids`, `update_centers`, 4 + 16 bytes per body of the tree).
//!   They grow (to the tree's number of bodies, the old array is freed first) at Jolt's two allocation points when they
//!   are too short, and are freed in deinit. Jolt allocates and frees both in every update. With an allocator that
//!   maps large blocks directly, as Zig's SmpAllocator does for blocks over 32 KB (the centers of more than 2048
//!   bodies, the node IDs of more than 8192), that costs an mmap, a munmap and a page fault per 4 KB page in every
//!   update (HighSpeed: an 80 KB block of centers, 20 page faults per step before Phase 13). The arrays are written
//!   before they are read in every update, so their old contents never matter.
//! - Memory behavior that differs from Jolt: an update allocates an array only when it is shorter than the tree's
//!   number of bodies. For the node IDs that is the first update of the tree and an update with more bodies than every
//!   update before. For the centers it is the first update whose BuildTree needs them (more than one node ID: an update
//!   that finds the tree as one unchanged node, such as the first one after the tree got all its bodies in one batch,
//!   does not) and growth after that. The walk stack still falls back to the heap like Jolt's. The arrays stay
//!   allocated between updates (at the tree's largest number of bodies so far), and the centers array is sized for all
//!   bodies of the tree instead of the node IDs that BuildTree gets. `releaseUpdateScratch` (Zolt only, also on
//!   BroadPhaseQuadTree) frees them: the next update then allocates at Jolt's points again, which the allocation
//!   failure tests use. AddBodiesPrepare still allocates BuildTree's centers per call like Jolt (it can run on any
//!   thread, concurrently with an update).

const std = @import("std");
const builtin = @import("builtin");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const AABox4 = @import("../../../Geometry/AABox4.zig");
const RayAABox = @import("../../../Geometry/RayAABox.zig");
const RayInvDirection = RayAABox.RayInvDirection;
const FixedSizeFreeList = @import("../../../Core/FixedSizeFreeList.zig").FixedSizeFreeList;
const STLLocalAllocator = @import("../../../Core/STLLocalAllocator.zig").STLLocalAllocator;
const Atomics = @import("../../../Core/Atomics.zig");
const getProcessorTickCount = @import("../../../Core/TickCounter.zig").getProcessorTickCount;
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const Body = @import("../../Body/Body.zig").Body;
const BodyPair = @import("../../Body/BodyPair.zig").BodyPair;
const BodyAccess = @import("../../Body/BodyAccess.zig").BodyAccess;
const BodySlot = @import("../../Body/BodyManager.zig").BodyManager.BodySlot;
const track_simulation_stats = @import("../../Body/MotionProperties.zig").track_simulation_stats;
const ObjectLayerFile = @import("../ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const object_layer_invalid = ObjectLayerFile.object_layer_invalid;
const ObjectLayerFilter = ObjectLayerFile.ObjectLayerFilter;
const ObjectLayerPairFilter = ObjectLayerFile.ObjectLayerPairFilter;
const BroadPhaseLayerFile = @import("BroadPhaseLayer.zig");
const BroadPhaseLayer = BroadPhaseLayerFile.BroadPhaseLayer;
const broad_phase_layer_invalid = BroadPhaseLayerFile.broad_phase_layer_invalid;
const RayCast = @import("../RayCast.zig").RayCast;
const AABoxCast = @import("../AABoxCast.zig").AABoxCast;
const BroadPhaseCastResult = @import("../CastResult.zig").BroadPhaseCastResult;
const SortReverseAndStore = @import("../SortReverseAndStore.zig");
const BroadPhaseQueryFile = @import("BroadPhaseQuery.zig");
const RayCastBodyCollector = BroadPhaseQueryFile.RayCastBodyCollector;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;
const CollideShapeBodyCollector = BroadPhaseQueryFile.CollideShapeBodyCollector;
const BodyPairCollector = @import("BroadPhase.zig").BodyPairCollector;

const log = std.log.scoped(.zolt);

const large_float = math.large_float;

/// Value used to indicate node index is invalid (cInvalidNodeIndex)
pub const invalid_node_index: u32 = 0xffffffff;

/// Maximum size of the stack during tree walk (cStackSize)
pub const stack_size = 128;

/// Invalid bounding box using cLargeFloat (cInvalidBounds)
const invalid_bounds: AABox = .{ .min = .replicate(large_float), .max = .replicate(-large_float) };

/// Internal tree structure in broadphase, is essentially a quad AABB tree.
/// Tree is lockless (except for UpdatePrepare/Finalize() function), modifying objects in the tree will widen the aabbs of parent nodes to make the node fit.
/// During the UpdatePrepare/Finalize() call the tree is rebuilt to achieve a tight fit again.
/// (NonCopyable in Jolt: a QuadTree is initialized in place and must not move, the nodes do not point to it)
pub const QuadTree = struct {
    /// Class that points to either a body or a node in the tree (private in Jolt)
    pub const NodeID = extern struct {
        id: u32,

        /// If this bit is set it means that the ID refers to a node, otherwise it refers to a body
        const is_node: u32 = BodyID.broad_phase_bit;

        /// Construct a node ID
        pub fn invalid() NodeID {
            return .{ .id = invalid_node_index };
        }

        pub fn fromBodyID(id: BodyID) NodeID {
            const node_id: NodeID = .{ .id = id.getIndexAndSequenceNumber() };
            std.debug.assert(node_id.isBody());
            return node_id;
        }

        pub fn fromNodeIndex(idx: u32) NodeID {
            std.debug.assert((idx & is_node) == 0);
            return .{ .id = idx | is_node };
        }

        /// Check what type of ID it is
        pub fn isValid(self: NodeID) bool {
            return self.id != invalid_node_index;
        }

        pub fn isBody(self: NodeID) bool {
            return (self.id & is_node) == 0;
        }

        pub fn isNode(self: NodeID) bool {
            return (self.id & is_node) != 0;
        }

        /// Get body or node index
        pub fn getBodyID(self: NodeID) BodyID {
            std.debug.assert(self.isBody());
            return .init(self.id);
        }

        pub fn getNodeIndex(self: NodeID) u32 {
            std.debug.assert(self.isNode());
            return self.id & ~is_node;
        }

        /// Comparison (operator == (const BodyID &))
        pub fn eqlBodyID(self: NodeID, rhs: BodyID) bool {
            return self.id == rhs.getIndexAndSequenceNumber();
        }

        /// Comparison (operator == (const NodeID &))
        pub fn eql(self: NodeID, rhs: NodeID) bool {
            return self.id == rhs.id;
        }
    };

    comptime {
        std.debug.assert(@sizeOf(NodeID) == @sizeOf(BodyID)); // Body id's should have the same size as NodeIDs
    }

    /// A NodeID that uses atomics to store the value (private in Jolt)
    pub const AtomicNodeID = extern struct {
        id: std.atomic.Value(u32),

        /// Constructor (explicit AtomicNodeID(const NodeID &))
        pub fn init(rhs: NodeID) AtomicNodeID {
            return .{ .id = .init(rhs.id) };
        }

        /// Assignment (operator =)
        pub fn set(self: *AtomicNodeID, rhs: NodeID) void {
            self.id.store(rhs.id, .seq_cst);
        }

        /// Getting the value (operator NodeID)
        pub fn get(self: *const AtomicNodeID) NodeID {
            return .{ .id = self.id.load(.seq_cst) };
        }

        /// Check if the ID is valid
        pub fn isValid(self: *const AtomicNodeID) bool {
            return self.id.load(.seq_cst) != invalid_node_index;
        }

        /// Comparison (operator == (const BodyID &))
        pub fn eqlBodyID(self: *const AtomicNodeID, rhs: BodyID) bool {
            return self.id.load(.seq_cst) == rhs.getIndexAndSequenceNumber();
        }

        /// Comparison (operator == (const NodeID &))
        pub fn eql(self: *const AtomicNodeID, rhs: NodeID) bool {
            return self.id.load(.seq_cst) == rhs.id;
        }

        /// Atomically compare and swap value. Expects inOld value, replaces with inNew value or returns false
        pub fn compareExchange(self: *AtomicNodeID, old: NodeID, new: NodeID) bool {
            return self.id.cmpxchgStrong(old.id, new.id, .seq_cst, .seq_cst) == null;
        }
    };

    /// Class that represents a node in the tree (private in Jolt; an extern struct with Jolt's 124 byte layout)
    pub const Node = extern struct {
        /// Bounding box for child nodes or bodies (all initially set to invalid so no collision test will ever traverse to the leaf)
        bounds_min_x: [4]std.atomic.Value(f32),
        bounds_min_y: [4]std.atomic.Value(f32),
        bounds_min_z: [4]std.atomic.Value(f32),
        bounds_max_x: [4]std.atomic.Value(f32),
        bounds_max_y: [4]std.atomic.Value(f32),
        bounds_max_z: [4]std.atomic.Value(f32),

        /// Index of child node or body ID.
        child_node_id: [4]AtomicNodeID,

        /// Index of the parent node.
        /// Note: This value is unreliable during the UpdatePrepare/Finalize() function as a node may be relinked to the newly built tree.
        parent_node_index: std.atomic.Value(u32) = .init(invalid_node_index),

        /// If this part of the tree has changed, if not, we will treat this sub tree as a single body during the UpdatePrepare/Finalize().
        /// If any changes are made to an object inside this sub tree then the direct path from the body to the top of the tree will become changed.
        is_changed: std.atomic.Value(u32),

        // Padding to align to 124 bytes
        padding: u32 = 0,

        /// Construct node
        pub fn init(is_changed: bool) Node {
            return .{
                // First reset bounds
                .bounds_min_x = @splat(.init(large_float)),
                .bounds_min_y = @splat(.init(large_float)),
                .bounds_min_z = @splat(.init(large_float)),
                .bounds_max_x = @splat(.init(-large_float)),
                .bounds_max_y = @splat(.init(-large_float)),
                .bounds_max_z = @splat(.init(-large_float)),

                // Reset child node ids
                .child_node_id = @splat(.init(.invalid())),

                .is_changed = .init(@intFromBool(is_changed)),
            };
        }

        /// Get bounding box encapsulating all children
        pub fn getNodeBounds(self: *const Node) AABox {
            // Get first child bounds
            var bounds = self.getChildBounds(0);

            // Encapsulate other child bounds
            for (1..4) |child_idx| {
                const tmp = self.getChildBounds(child_idx);
                bounds.encapsulate(tmp);
            }
            return bounds;
        }

        /// Get bounding box in a consistent way with the functions below (check outBounds.IsValid() before using the box)
        pub fn getChildBounds(self: *const Node, child_index: usize) AABox {
            // Read bounding box in order min -> max
            const min = Vec3.init(self.bounds_min_x[child_index].load(.seq_cst), self.bounds_min_y[child_index].load(.seq_cst), self.bounds_min_z[child_index].load(.seq_cst));
            const max = Vec3.init(self.bounds_max_x[child_index].load(.seq_cst), self.bounds_max_y[child_index].load(.seq_cst), self.bounds_max_z[child_index].load(.seq_cst));
            return .{ .min = min, .max = max };
        }

        /// Set the bounds in such a way that other threads will either see a fully correct bounding box or a bounding box with no volume
        pub fn setChildBounds(self: *Node, child_index: usize, bounds: AABox) void {
            // Bounding boxes provided to the quad tree should never be larger than cLargeFloat because this may trigger overflow exceptions
            // e.g. when squaring the value while testing sphere overlaps
            std.debug.assert(bounds.min.getX() >= -large_float and bounds.min.getX() <= large_float and
                bounds.min.getY() >= -large_float and bounds.min.getY() <= large_float and
                bounds.min.getZ() >= -large_float and bounds.min.getZ() <= large_float and
                bounds.max.getX() >= -large_float and bounds.max.getX() <= large_float and
                bounds.max.getY() >= -large_float and bounds.max.getY() <= large_float and
                bounds.max.getZ() >= -large_float and bounds.max.getZ() <= large_float);

            // Set max first (this keeps the bounding box invalid for reading threads)
            self.bounds_max_z[child_index].store(bounds.max.getZ(), .seq_cst);
            self.bounds_max_y[child_index].store(bounds.max.getY(), .seq_cst);
            self.bounds_max_x[child_index].store(bounds.max.getX(), .seq_cst);

            // Then set min (and make box valid)
            self.bounds_min_z[child_index].store(bounds.min.getZ(), .seq_cst);
            self.bounds_min_y[child_index].store(bounds.min.getY(), .seq_cst);
            self.bounds_min_x[child_index].store(bounds.min.getX(), .seq_cst); // Min X becomes valid last
        }

        /// Invalidate bounding box in such a way that other threads will not temporarily see a very large bounding box
        pub fn invalidateChildBounds(self: *Node, child_index: usize) void {
            // First we make the box invalid by setting the min to cLargeFloat
            self.bounds_min_x[child_index].store(large_float, .seq_cst); // Min X becomes invalid first
            self.bounds_min_y[child_index].store(large_float, .seq_cst);
            self.bounds_min_z[child_index].store(large_float, .seq_cst);

            // Then we reset the max values too
            self.bounds_max_x[child_index].store(-large_float, .seq_cst);
            self.bounds_max_y[child_index].store(-large_float, .seq_cst);
            self.bounds_max_z[child_index].store(-large_float, .seq_cst);
        }

        /// Encapsulate inBounds in node bounds, returns true if there were changes
        pub fn encapsulateChildBounds(self: *Node, child_index: usize, bounds: AABox) bool {
            // Jolt's `changed |= ...` evaluates every AtomicMin / AtomicMax (no short circuit)
            var changed = Atomics.atomicMin(f32, &self.bounds_min_x[child_index], bounds.min.getX(), .{});
            if (Atomics.atomicMin(f32, &self.bounds_min_y[child_index], bounds.min.getY(), .{})) changed = true;
            if (Atomics.atomicMin(f32, &self.bounds_min_z[child_index], bounds.min.getZ(), .{})) changed = true;
            if (Atomics.atomicMax(f32, &self.bounds_max_x[child_index], bounds.max.getX(), .{})) changed = true;
            if (Atomics.atomicMax(f32, &self.bounds_max_y[child_index], bounds.max.getY(), .{})) changed = true;
            if (Atomics.atomicMax(f32, &self.bounds_max_z[child_index], bounds.max.getZ(), .{})) changed = true;
            return changed;
        }

        /// The arrays of child bounds, one per axis
        pub const BoundsArray = enum { bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z };

        /// Zolt: the 4 child bounds of one axis as a Vec4 (Jolt: `Vec4::sLoadFloat4Aligned((const Float4 *)&mBoundsMinX)`,
        /// a racy vector load). One vector load on x86-64, else each lane loaded with .monotonic (see the file comment).
        fn loadBounds(self: *const Node, comptime array: BoundsArray) Vec4 {
            if (comptime use_vector_load)
                return .{ .value = self.vectorLoad(@Vector(4, f32), @offsetOf(Node, @tagName(array)), false) };
            const bounds = &@field(self, @tagName(array));
            return .init(bounds[0].load(.monotonic), bounds[1].load(.monotonic), bounds[2].load(.monotonic), bounds[3].load(.monotonic));
        }

        /// Zolt: the 4 child IDs with acquire order (Jolt: `UVec4::sLoadInt4Aligned((const uint32 *)&mChildNodeID[0])`).
        /// A child node is constructed with plain stores (FixedSizeFreeList.constructObject) and then published by the
        /// seq_cst compare-exchange of TryInsertLeaf / TryCreateNewRoot: the acquire makes its construction visible
        /// before the walk reads its bounds and child IDs. One vector load on x86-64, else each lane loaded with
        /// .acquire (see the file comment).
        fn loadChildIDs(self: *const Node) UVec4 {
            if (comptime use_vector_load)
                return .{ .value = self.vectorLoad(@Vector(4, u32), @offsetOf(Node, "child_node_id"), true) };
            return .init(self.child_node_id[0].id.load(.acquire), self.child_node_id[1].id.load(.acquire), self.child_node_id[2].id.load(.acquire), self.child_node_id[3].id.load(.acquire));
        }

        /// Zolt: load the 16 bytes at `offset` in the node with one aligned vector load ((v)movaps / (v)movdqa).
        /// Other threads may store to these bytes at the same time. That is not a data race in Zig's memory model,
        /// because the load is inline asm, which the compiler treats as an opaque instruction. On x86 an aligned
        /// 4 byte lane is read atomically, so each lane holds a value that some store wrote, as with Jolt's load.
        /// With `acquire` the asm also clobbers memory: the compiler cannot move later loads above it, and x86
        /// does not reorder loads with later loads, so the load has acquire order.
        inline fn vectorLoad(self: *const Node, comptime T: type, comptime offset: usize, comptime acquire: bool) T {
            // Nodes are cache line aligned and every array is at a multiple of 16 bytes (the aligned load faults otherwise)
            comptime std.debug.assert(offset % 16 == 0);
            std.debug.assert(std.mem.isAligned(@intFromPtr(self), 16));
            const code = comptime code: {
                const has_avx = std.Target.x86.featureSetHas(builtin.cpu.features, .avx);
                const mnemonic = if (T == @Vector(4, f32)) (if (has_avx) "vmovaps" else "movaps") else (if (has_avx) "vmovdqa" else "movdqa");
                break :code std.fmt.comptimePrint("{s} {d}(%[node]), %[result]", .{ mnemonic, offset });
            };
            if (acquire) {
                return asm volatile (code
                    : [result] "=x" (-> T),
                    : [node] "r" (self),
                    : .{ .memory = true });
            }
            return asm volatile (code
                : [result] "=x" (-> T),
                : [node] "r" (self),
            );
        }

        /// Zolt: use `vectorLoad` in the tree walks. It needs x86-64 and the LLVM backend (the assembler of the
        /// self-hosted backend does not take the memory operand). TSAN builds load per lane, so TSAN sees atomics.
        const use_vector_load = builtin.cpu.arch == .x86_64 and builtin.zig_backend == .stage2_llvm and !builtin.sanitize_thread;
    };

    comptime {
        std.debug.assert(@sizeOf(Node) == 124); // Padding to align to 124 bytes
        std.debug.assert(@sizeOf(std.atomic.Value(f32)) == 4); // Assuming that an atomic doesn't add any additional storage
        std.debug.assert(@sizeOf(std.atomic.Value(u32)) == 4); // Assuming that an atomic doesn't add any additional storage
    }

    /// Class that allocates tree nodes, can be shared between multiple trees (QuadTree::Allocator)
    pub const NodeAllocator = FixedSizeFreeList(Node);

    comptime {
        std.debug.assert(NodeAllocator.object_storage_size == 128); // Node should be 128 bytes
    }

    /// Data to track location of a Body in the tree
    pub const Tracking = struct {
        /// Invalid body location identifier
        pub const invalid_body_location: u32 = 0xffffffff;

        broad_phase_layer: std.atomic.Value(BroadPhaseLayer.Type) = .init(broad_phase_layer_invalid.value),
        object_layer: std.atomic.Value(ObjectLayer) = .init(object_layer_invalid),
        body_location: std.atomic.Value(u32) = .init(invalid_body_location),
    };

    pub const TrackingVector = []Tracking;

    pub const UpdateState = extern struct {
        /// This will be the new root node id
        root_node_id: NodeID,
    };

    /// Temporary data structure to pass information between AddBodiesPrepare and AddBodiesFinalize/Abort
    pub const AddState = struct {
        leaf_id: NodeID = .invalid(),
        leaf_bounds: AABox = .empty,
    };

    /// We alternate between two trees in order to let collision queries complete in parallel to adding/removing objects to the tree
    pub const RootNode = struct {
        /// Index of the root node of the tree (this is always a node, never a body id)
        index: std.atomic.Value(u32) = .init(invalid_node_index),

        /// Get the ID of the root node
        pub fn getNodeID(self: *const RootNode) NodeID {
            return .fromNodeIndex(self.index.load(.seq_cst));
        }
    };

    /// mNumBodies and the members that follow it in Jolt (aligned to, and occupying whole, cache lines, see the file comment)
    pub const TreeState = struct {
        /// Number of bodies currently in the tree
        /// This is aligned to be in a different cache line from the `Allocator` pointer to prevent cross-thread syncs
        /// when reading nodes.
        num_bodies: std.atomic.Value(u32) align(Core.cache_line_size) = .init(0),

        /// We alternate between two tree root nodes. When updating, we activate the new tree and we keep the old tree alive.
        /// for queries that are in progress until the next time DiscardOldTree() is called.
        root_node: [2]RootNode = .{ .{}, .{} },
        root_node_index: std.atomic.Value(u32) = .init(0),

        /// Flag to keep track of changes to the broadphase, if false, we don't need to UpdatePrepare/Finalize()
        is_dirty: std.atomic.Value(bool) = .init(false),
    };

    comptime {
        std.debug.assert(@alignOf(TreeState) == Core.cache_line_size and @sizeOf(TreeState) % Core.cache_line_size == 0);
    }

    /// Location of a body in the tree (out parameters of GetBodyLocation): the body is in node_allocator.get(node_idx).child_node_id[child_idx]
    pub const BodyLocation = struct {
        node_idx: u32,
        child_idx: u32,
    };

    /// The result of BuildTree: the NodeID of the root (the ID of a single body if there was 1) and its bounds (outBounds)
    const BuildTreeResult = struct {
        root: NodeID,
        bounds: AABox,
    };

    /// Allocator that controls adding / freeing nodes (mAllocator)
    node_allocator: ?*NodeAllocator = null,

    /// Zolt: the heap (Jolt's global allocator): `new NodeID[]`, `new Vec3[]` and the heap fallback of the walk stacks
    allocator: Allocator = undefined,

    /// This is a list of nodes that must be deleted after the trees are swapped and the old tree is no longer in use
    free_node_batch: NodeAllocator.Batch = .{},

    /// mNumBodies, mRootNode, mRootNodeIndex and mIsDirty
    tree_state: TreeState = .{},

    /// Zolt (Phase 13): the temporary arrays of updatePrepare, Jolt's `new NodeID[mNumBodies]` and BuildTree's
    /// `new Vec3[inNumber]`, kept from one update to the next and only grown (to the number of bodies of the tree),
    /// freed in deinit and releaseUpdateScratch. Only updatePrepare uses them, which runs for one tree at a time with
    /// the update lock held (BroadPhaseQuadTree.lockModifications).
    update_node_ids: []NodeID = &.{},
    update_centers: []Vec3 = &.{},

    /// Zolt: the latch of sQuadTreePerformanceWarning (Jolt: a function `static atomic<bool>`, only with asserts),
    /// shared by the trees of a BroadPhaseQuadTree, which sets it after init (null: the warning is not reported)
    triggered_report: if (Core.enable_asserts) ?*std.atomic.Value(bool) else void = if (Core.enable_asserts) null else {},

    /// A tree that is not initialized (Jolt's default constructor), call init
    pub const empty: QuadTree = .{};

    fn nodeAllocator(self: *const QuadTree) *NodeAllocator {
        return self.node_allocator.?;
    }

    fn getNode(self: *const QuadTree, node_idx: u32) *Node {
        return self.nodeAllocator().get(node_idx);
    }

    /// Check if there is anything in the tree
    pub fn hasBodies(self: *const QuadTree) bool {
        return self.tree_state.num_bodies.load(.seq_cst) != 0;
    }

    /// Check if the tree needs an UpdatePrepare/Finalize()
    pub fn isDirty(self: *const QuadTree) bool {
        return self.tree_state.is_dirty.load(.seq_cst);
    }

    /// Check if this tree can get an UpdatePrepare/Finalize() or if it needs a DiscardOldTree() first
    pub fn canBeUpdated(self: *const QuadTree) bool {
        return self.free_node_batch.num_objects == 0;
    }

    /// Get the current root of the tree
    fn getCurrentRoot(self: *const QuadTree) *const RootNode {
        return &self.tree_state.root_node[self.tree_state.root_node_index.load(.seq_cst)];
    }

    /// Get the current root of the tree (non-const overload)
    fn getCurrentRootMut(self: *QuadTree) *RootNode {
        return &self.tree_state.root_node[self.tree_state.root_node_index.load(.seq_cst)];
    }

    /// Caches location of body inBodyID in the tracker, body can be found in mNodes[inNodeIdx].mChildNodeID[inChildIdx]
    pub fn getBodyLocation(self: *const QuadTree, tracking: []const Tracking, body_id: BodyID) BodyLocation {
        const body_location = tracking[body_id.getIndex()].body_location.load(.seq_cst);
        std.debug.assert(body_location != Tracking.invalid_body_location);
        const location: BodyLocation = .{ .node_idx = body_location & 0x3fffffff, .child_idx = body_location >> 30 };
        if (Core.enable_asserts) std.debug.assert(self.getNode(location.node_idx).child_node_id[location.child_idx].eqlBodyID(body_id)); // Make sure that the body is in the node where it should be
        return location;
    }

    fn setBodyLocation(self: *const QuadTree, tracking: []Tracking, body_id: BodyID, node_idx: u32, child_idx: u32) void {
        std.debug.assert(node_idx <= 0x3fffffff);
        std.debug.assert(child_idx < 4);
        if (Core.enable_asserts) std.debug.assert(self.getNode(node_idx).child_node_id[child_idx].eqlBodyID(body_id)); // Make sure that the body is in the node where it should be
        tracking[body_id.getIndex()].body_location.store(node_idx + (child_idx << 30), .seq_cst);

        if (Core.enable_asserts) {
            const location = self.getBodyLocation(tracking, body_id);
            std.debug.assert(location.node_idx == node_idx);
            std.debug.assert(location.child_idx == child_idx);
        }
    }

    fn invalidateBodyLocation(tracking: []Tracking, body_id: BodyID) void {
        tracking[body_id.getIndex()].body_location.store(Tracking.invalid_body_location, .seq_cst);
    }

    /// Destructor. Zolt: does nothing for a tree that was never initialized (`empty`).
    pub fn deinit(self: *QuadTree) void {
        const node_allocator = self.node_allocator orelse return;

        // Get rid of any nodes that are still to be freed
        self.discardOldTree();

        // Get the current root node
        const root_node = self.getCurrentRoot();

        // Collect all bodies
        var free_batch: NodeAllocator.Batch = .{};
        // Zolt: Jolt's node stack (an Array with STLLocalAllocator, which can fall back to the heap) is a linked list
        // through the parent_node_index of the nodes on it: they are freed below, so no allocation can fail here. Only
        // nodes are pushed, popped last in first out like Jolt's stack, so the nodes are batched in Jolt's order.
        const root_id = root_node.getNodeID();
        std.debug.assert(root_id.isValid());
        if (root_id.isNode()) {
            var stack_top = root_id.getNodeIndex();
            node_allocator.get(stack_top).parent_node_index.store(invalid_node_index, .monotonic); // The end of the stack
            while (true) {
                // Process node
                const node_idx = stack_top;
                const node = node_allocator.get(node_idx);
                stack_top = node.parent_node_index.load(.monotonic); // Pop

                // Recurse and get all child nodes
                for (&node.child_node_id) |*child| {
                    const child_node_id = child.get();
                    if (child_node_id.isValid() and child_node_id.isNode()) {
                        // Push
                        const child_idx = child_node_id.getNodeIndex();
                        node_allocator.get(child_idx).parent_node_index.store(stack_top, .monotonic);
                        stack_top = child_idx;
                    }
                }

                // Mark node to be freed
                node_allocator.addObjectToBatch(&free_batch, node_idx);

                if (stack_top == invalid_node_index) break;
            }
        }

        // Now free all nodes
        node_allocator.destructObjectBatch(&free_batch);

        // Zolt (Phase 13): the temporary arrays of updatePrepare
        self.releaseUpdateScratch();

        self.* = .empty;
    }

    /// Allocate a new node
    fn allocateNode(self: *const QuadTree, is_changed: bool) u32 {
        const index = self.nodeAllocator().constructObject(.init(is_changed)) catch @panic("QuadTree: the node allocator must have all its pages reserved (FixedSizeFreeList.reserveAllPages)");
        if (index == NodeAllocator.invalid_object_index) {
            // If you're running out of nodes, you're most likely adding too many individual bodies to the tree.
            // Because of the lock free nature of this tree, any individual body is added to the root of the tree.
            // This means that if you add a lot of bodies individually, you will end up with a very deep tree and you'll be
            // using a lot more nodes than you would if you added them in batches.
            // Please look at BodyInterface::AddBodiesPrepare/AddBodiesFinalize.
            //
            // If you have created a wrapper around Jolt then a possible solution is to activate a mode during loading
            // that queues up any bodies that need to be added. When loading is done, insert all of them as a single batch.
            // This could be implemented as a 'start batching' / 'end batching' call to switch in and out of that mode.
            // The rest of the code can then just use the regular 'add single body' call on your wrapper and doesn't need to know
            // if this mode is active or not.
            //
            // Calling PhysicsSystem::Update or PhysicsSystem::OptimizeBroadPhase will perform maintenance
            // on the tree and will make it efficient again. If you're not calling these functions and are adding a lot of bodies
            // you could still be running out of nodes because the tree is not being maintained. If your application is paused,
            // consider still calling PhysicsSystem::Update with a delta time of 0 to keep the tree in good shape.
            //
            // The number of nodes that is allocated is related to the max number of bodies that is passed in PhysicsSystem::Init.
            // For normal situations there are plenty of nodes available. If all else fails, you can increase the number of nodes
            // by increasing the maximum number of bodies.
            log.info("QuadTree: Out of nodes!", .{});
            @panic("QuadTree: Out of nodes!"); // std::abort()
        }
        return index;
    }

    /// Initialization. `node_allocator` must have all its pages reserved (FixedSizeFreeList.reserveAllPages), `allocator`
    /// is the heap of the temporary arrays (Zolt). The tree must not move afterwards.
    pub fn init(self: *QuadTree, node_allocator: *NodeAllocator, allocator: Allocator) void {
        // Store allocator
        self.node_allocator = node_allocator;
        self.allocator = allocator;

        // Allocate root node
        const root_idx = self.allocateNode(false);
        self.tree_state.root_node[self.tree_state.root_node_index.load(.seq_cst)].index.store(root_idx, .seq_cst);
    }

    /// Will throw away the previous frame's nodes so that we can start building a new tree in the background
    pub fn discardOldTree(self: *QuadTree) void {
        // Check if there is an old tree
        const old_root_node = &self.tree_state.root_node[self.tree_state.root_node_index.load(.seq_cst) ^ 1];
        if (old_root_node.index.load(.seq_cst) != invalid_node_index) {
            // Clear the root
            old_root_node.index.store(invalid_node_index, .seq_cst);

            // Now free all old nodes
            self.nodeAllocator().destructObjectBatch(&self.free_node_batch);

            // Clear the batch
            self.free_node_batch = .{};
        }
    }

    /// Get the bounding box for this tree
    pub fn getBounds(self: *const QuadTree) AABox {
        const node_idx = self.getCurrentRoot().index.load(.seq_cst);
        std.debug.assert(node_idx != invalid_node_index);
        const node = self.getNode(node_idx);

        return node.getNodeBounds();
    }

    /// Update the broadphase, needs to be called regularly to achieve a tight fit of the tree when bodies have been modified.
    /// UpdatePrepare() will build the tree, UpdateFinalize() will lock the root of the tree shortly and swap the trees and afterwards clean up temporary data structures.
    /// (Zolt: returns the update state, or Allocator.Error with the tree unchanged)
    pub fn updatePrepare(self: *QuadTree, bodies: []const BodySlot, tracking: []Tracking, full_rebuild: bool) Allocator.Error!UpdateState {
        // We only read positions
        var grant: BodyAccess.Grant = .init(.none, .read);
        defer grant.deinit();

        // Assert we have no nodes pending deletion, this means DiscardOldTree wasn't called yet
        std.debug.assert(self.free_node_batch.num_objects == 0);

        // Mark tree non-dirty
        const was_dirty = self.tree_state.is_dirty.load(.seq_cst);
        self.tree_state.is_dirty.store(false, .seq_cst);
        errdefer self.tree_state.is_dirty.store(was_dirty, .seq_cst); // Zolt: the tree is unchanged on error

        // Get the current root node
        const root_node = self.getCurrentRoot();

        // Assert sane data
        if (builtin.mode == .Debug)
            self.validateTree(bodies, tracking, root_node.index.load(.seq_cst), self.tree_state.num_bodies.load(.seq_cst));

        // Create space for all body ID's
        // Zolt (Phase 13): in the array that the tree keeps (update_node_ids), it allocates only when it grows
        const num_bodies = self.tree_state.num_bodies.load(.seq_cst);
        const node_ids: []NodeID = if (num_bodies > 0) try self.growUpdateScratch(NodeID, &self.update_node_ids, num_bodies) else &.{};
        var cur_node_id: usize = 0;

        // Collect all bodies
        var node_stack_allocator = STLLocalAllocator(NodeID, stack_size).init(self.allocator);
        const node_stack_alloc = node_stack_allocator.allocator();
        var node_stack: std.ArrayList(NodeID) = .empty;
        defer node_stack.deinit(node_stack_alloc);
        node_stack.ensureTotalCapacityPrecise(node_stack_alloc, stack_size) catch unreachable; // reserve(cStackSize): the local buffer
        node_stack.appendAssumeCapacity(root_node.getNodeID());
        std.debug.assert(node_stack.items[0].isValid());

        // Zolt: on error the nodes that were marked to be freed are in use again (the tree is unchanged)
        errdefer self.nodeAllocator().cancelBatch(&self.free_node_batch);

        while (true) {
            // Pop node from stack
            const node_id = node_stack.pop().?;

            // Check if node is a body
            if (node_id.isBody()) {
                // Validate that we're still in the right layer
                if (Core.enable_asserts) {
                    const body_index = node_id.getBodyID().getIndex();
                    std.debug.assert(tracking[body_index].object_layer.load(.seq_cst) == bodies[body_index].getBody().getObjectLayer());
                }

                // Store body
                node_ids[cur_node_id] = node_id;
                cur_node_id += 1;
            } else {
                // Process normal node
                const node_idx = node_id.getNodeIndex();
                const node = self.getNode(node_idx);

                if (node.is_changed.load(.seq_cst) == 0 and !full_rebuild) {
                    // Node is unchanged, treat it as a whole
                    node_ids[cur_node_id] = node_id;
                    cur_node_id += 1;
                } else {
                    // Node is changed, recurse and get all children
                    for (&node.child_node_id) |*child| {
                        const child_node_id = child.get();
                        if (child_node_id.isValid())
                            try node_stack.append(node_stack_alloc, child_node_id);
                    }

                    // Mark node to be freed
                    self.nodeAllocator().addObjectToBatch(&self.free_node_batch, node_idx);
                }
            }

            if (node_stack.items.len == 0) break;
        }

        // Check that our book keeping matches
        const num_node_ids: u32 = @intCast(cur_node_id);
        std.debug.assert(if (full_rebuild) num_node_ids == num_bodies else num_node_ids <= num_bodies);

        // This will be the new root node id
        var root_node_id: NodeID = undefined;

        if (num_node_ids > 0) {
            // We mark the first 5 levels (max 1024 nodes) of the newly built tree as 'changed' so that
            // those nodes get recreated every time when we rebuild the tree. This balances the amount of
            // time we spend on rebuilding the tree ('unchanged' nodes will be put in the new tree as a whole)
            // vs the quality of the built tree.
            const max_depth_mark_changed = 5;

            // Zolt (Phase 13): BuildTree's centers are the array that the tree keeps (update_centers). It grows here,
            // where BuildTree would allocate them (BuildTree allocates nothing for a single node), to the number of
            // bodies of the tree (at least num_node_ids), so that it rarely grows again.
            const centers: ?[]Vec3 = if (num_node_ids > 1) try self.growUpdateScratch(Vec3, &self.update_centers, num_bodies) else null;

            // Build new tree
            const result = try self.buildTree(bodies, tracking, node_ids[0..num_node_ids], max_depth_mark_changed, centers);
            const root_bounds = result.bounds;
            root_node_id = result.root;

            if (root_node_id.isBody()) {
                // For a single body we need to allocate a new root node
                const root_idx = self.allocateNode(false);
                const root = self.getNode(root_idx);
                root.setChildBounds(0, root_bounds);
                root.child_node_id[0].set(root_node_id);
                self.setBodyLocation(tracking, root_node_id.getBodyID(), root_idx, 0);
                root_node_id = .fromNodeIndex(root_idx);
            }
        } else {
            // Empty tree, create root node
            const root_idx = self.allocateNode(false);
            root_node_id = .fromNodeIndex(root_idx);
        }

        return .{ .root_node_id = root_node_id };
    }

    /// Zolt (Phase 13): the first `len` entries of a temporary array of updatePrepare, which is replaced by a new one
    /// of `len` entries when it is shorter (its contents are not kept). On error the array is empty.
    fn growUpdateScratch(self: *QuadTree, comptime T: type, scratch: *[]T, len: usize) Allocator.Error![]T {
        if (scratch.len < len) {
            self.allocator.free(scratch.*);
            scratch.* = &.{};
            scratch.* = try self.allocator.alloc(T, len);
        }
        return scratch.*[0..len];
    }

    /// Zolt (Phase 13): frees the temporary arrays that updatePrepare keeps between updates (update_node_ids,
    /// update_centers). The next updatePrepare then allocates them where Jolt allocates them, which the allocation
    /// failure tests use. Not while an update of the tree runs.
    pub fn releaseUpdateScratch(self: *QuadTree) void {
        if (self.update_node_ids.len > 0) self.allocator.free(self.update_node_ids);
        if (self.update_centers.len > 0) self.allocator.free(self.update_centers);
        self.update_node_ids = &.{};
        self.update_centers = &.{};
    }

    /// Swap the new tree in (see updatePrepare)
    pub fn updateFinalize(self: *QuadTree, bodies: []const BodySlot, tracking: []const Tracking, update_state: UpdateState) void {
        // Tree building is complete, now we switch the old with the new tree
        const new_root_idx = self.tree_state.root_node_index.load(.seq_cst) ^ 1;
        const new_root_node = &self.tree_state.root_node[new_root_idx];
        {
            // Note: We don't need to lock here as the old tree stays available so any queries
            // that use it can continue using it until DiscardOldTree is called. This slot
            // should be empty and unused at this moment.
            std.debug.assert(new_root_node.index.load(.seq_cst) == invalid_node_index);
            new_root_node.index.store(update_state.root_node_id.getNodeIndex(), .seq_cst);
        }

        // All queries that start from now on will use this new tree
        self.tree_state.root_node_index.store(new_root_idx, .seq_cst);

        if (builtin.mode == .Debug)
            self.validateTree(bodies, tracking, new_root_node.index.load(.seq_cst), self.tree_state.num_bodies.load(.seq_cst));
    }

    /// Sorts ioNodeIDs spatially into 2 groups. Second groups starts at ioNodeIDs + outMidPoint (returned).
    /// After the function returns ioNodeIDs and ioNodeCenters will be shuffled
    fn partition(node_ids: []NodeID, node_centers: []Vec3) u32 {
        std.debug.assert(node_ids.len == node_centers.len);
        const number: u32 = @intCast(node_ids.len);

        // Handle trivial case
        if (number <= 4)
            return number / 2;

        // Calculate bounding box of box centers
        var center_min = Vec3.replicate(large_float);
        var center_max = Vec3.replicate(-large_float);
        for (node_centers) |center| {
            center_min = Vec3.min(center_min, center);
            center_max = Vec3.max(center_max, center);
        }

        // Calculate split plane
        const dimension = center_max.sub(center_min).getHighestComponentIndex();
        const split = 0.5 * center_min.add(center_max).getComponent(dimension);

        // Divide bodies
        var start: u32 = 0;
        var end: u32 = number;
        while (start < end) {
            // Search for first element that is on the right hand side of the split plane
            while (start < end and node_centers[start].getComponent(dimension) < split)
                start += 1;

            // Search for the first element that is on the left hand side of the split plane
            while (start < end and node_centers[end - 1].getComponent(dimension) >= split)
                end -= 1;

            if (start < end) {
                // Swap the two elements
                Core.swap(NodeID, &node_ids[start], &node_ids[end - 1]);
                Core.swap(Vec3, &node_centers[start], &node_centers[end - 1]);
                start += 1;
                end -= 1;
            }
        }
        std.debug.assert(start == end);

        if (start > 0 and start < number) {
            // Success!
            return start;
        } else {
            // Failed to divide bodies
            return number / 2;
        }
    }

    /// Sorts ioNodeIDs from inBegin to (but excluding) inEnd spatially into 4 groups.
    /// outSplit needs to be 5 ints long, when the function returns each group runs from outSplit[i] to (but excluding) outSplit[i + 1]
    /// After the function returns ioNodeIDs and ioNodeCenters will be shuffled
    fn partition4(node_ids_in: []NodeID, node_centers_in: []Vec3, begin: u32, end: u32, out_split: *[5]u32) void {
        const node_ids = node_ids_in[begin..end];
        const node_centers = node_centers_in[begin..end];
        const number = end - begin;

        // Partition entire range
        out_split[2] = partition(node_ids, node_centers);

        // Partition lower half
        out_split[1] = partition(node_ids[0..out_split[2]], node_centers[0..out_split[2]]);

        // Partition upper half
        out_split[3] = partition(node_ids[out_split[2]..number], node_centers[out_split[2]..number]);

        // Convert to proper range
        out_split[0] = begin;
        out_split[1] += begin;
        out_split[2] += begin;
        out_split[3] += out_split[2];
        out_split[4] = end;
    }

    /// Depending on if inNodeID is a body or tree node return the bounding box
    fn getNodeOrBodyBounds(self: *const QuadTree, bodies: []const BodySlot, node_id: NodeID) AABox {
        if (node_id.isNode()) {
            // It is a node
            const node_idx = node_id.getNodeIndex();
            const node = self.getNode(node_idx);

            return node.getNodeBounds();
        } else {
            // It is a body
            return bodies[node_id.getBodyID().getIndex()].getBody().getWorldSpaceBounds().*;
        }
    }

    /// Build a tree for ioBodyIDs, returns the NodeID of the root (which will be the ID of a single body if inNumber = 1). All tree levels up to inMaxDepthMarkChanged will be marked as 'changed'.
    /// (Zolt: Allocator.Error from `new Vec3[]`, nothing changed then; `centers_buffer`, when given, replaces that
    /// allocation, it needs at least node_ids.len entries)
    fn buildTree(self: *QuadTree, bodies: []const BodySlot, tracking: []Tracking, node_ids: []NodeID, max_depth_mark_changed: u32, centers_buffer: ?[]Vec3) Allocator.Error!BuildTreeResult {
        const number: u32 = @intCast(node_ids.len);

        // Trivial case: No bodies in tree
        if (number == 0)
            return .{ .root = .invalid(), .bounds = invalid_bounds };

        // Trivial case: When we have 1 body or node, return it
        if (number == 1) {
            if (node_ids[0].isNode()) {
                // When returning an existing node as root, ensure that no parent has been set
                const node = self.getNode(node_ids[0].getNodeIndex());
                node.parent_node_index.store(invalid_node_index, .seq_cst);
            }
            return .{ .root = node_ids[0], .bounds = self.getNodeOrBodyBounds(bodies, node_ids[0]) };
        }

        // Calculate centers of all bodies that are to be inserted
        const allocated_centers: []Vec3 = if (centers_buffer == null) try self.allocator.alloc(Vec3, number) else &.{};
        defer self.allocator.free(allocated_centers); // Delete temporary data
        const centers: []Vec3 = if (centers_buffer) |buffer| buffer[0..number] else allocated_centers;
        std.debug.assert(std.mem.isAligned(@intFromPtr(centers.ptr), Core.vector_alignment));
        for (node_ids, centers) |n, *c|
            c.* = self.getNodeOrBodyBounds(bodies, n).getCenter();

        // The algorithm is a recursive tree build, but to avoid the call overhead we keep track of a stack here
        const StackEntry = extern struct {
            node_idx: u32, // Node index of node that is generated
            child_idx: i32, // Index of child that we're currently processing
            split: [5]u32, // Indices where the node ID's have been split to form 4 partitions
            depth: u32, // Depth of this node in the tree
            node_bounds_min: Vec3, // Bounding box of this node, accumulated while iterating over children
            node_bounds_max: Vec3,
        };
        comptime std.debug.assert(@sizeOf(StackEntry) == 64);
        var stack: [stack_size / 4]StackEntry = undefined; // We don't process 4 at a time in this loop but 1, so the stack can be 4x as small
        var top: usize = 0;

        // Create root node
        stack[0].node_idx = self.allocateNode(max_depth_mark_changed > 0);
        stack[0].child_idx = -1;
        stack[0].depth = 0;
        stack[0].node_bounds_min = Vec3.replicate(large_float);
        stack[0].node_bounds_max = Vec3.replicate(-large_float);
        partition4(node_ids, centers, 0, number, &stack[0].split);

        while (true) {
            const cur_stack = &stack[top];

            // Next child
            cur_stack.child_idx += 1;

            // Check if all children processed
            if (cur_stack.child_idx >= 4) {
                // Terminate if there's nothing left to pop
                if (top <= 0)
                    break;

                // Add our bounds to our parents bounds
                const prev_stack = &stack[top - 1];
                prev_stack.node_bounds_min = Vec3.min(prev_stack.node_bounds_min, cur_stack.node_bounds_min);
                prev_stack.node_bounds_max = Vec3.max(prev_stack.node_bounds_max, cur_stack.node_bounds_max);

                // Store parent node
                const node = self.getNode(cur_stack.node_idx);
                node.parent_node_index.store(prev_stack.node_idx, .seq_cst);

                // Store this node's properties in the parent node
                const parent_node = self.getNode(prev_stack.node_idx);
                const prev_child_idx: usize = @intCast(prev_stack.child_idx);
                parent_node.child_node_id[prev_child_idx].set(.fromNodeIndex(cur_stack.node_idx));
                parent_node.setChildBounds(prev_child_idx, .{ .min = cur_stack.node_bounds_min, .max = cur_stack.node_bounds_max });

                // Pop entry from stack
                top -= 1;
            } else {
                // Get low and high index to bodies to process
                const child_idx: u32 = @intCast(cur_stack.child_idx);
                const low = cur_stack.split[child_idx];
                const high = cur_stack.split[child_idx + 1];
                const num_bodies = high - low;

                if (num_bodies == 1) {
                    // Get body info
                    const child_node_id = node_ids[low];
                    const bounds = self.getNodeOrBodyBounds(bodies, child_node_id);

                    // Update node
                    const node = self.getNode(cur_stack.node_idx);
                    node.child_node_id[child_idx].set(child_node_id);
                    node.setChildBounds(child_idx, bounds);

                    if (child_node_id.isNode()) {
                        // Update parent for this node
                        const child_node = self.getNode(child_node_id.getNodeIndex());
                        child_node.parent_node_index.store(cur_stack.node_idx, .seq_cst);
                    } else {
                        // Set location in tracking
                        self.setBodyLocation(tracking, child_node_id.getBodyID(), cur_stack.node_idx, child_idx);
                    }

                    // Encapsulate bounding box in parent
                    cur_stack.node_bounds_min = Vec3.min(cur_stack.node_bounds_min, bounds.min);
                    cur_stack.node_bounds_max = Vec3.max(cur_stack.node_bounds_max, bounds.max);
                } else if (num_bodies > 1) {
                    // Allocate new node
                    top += 1;
                    if (top >= stack_size / 4) // JPH_ASSERT(top < cStackSize / 4), Zolt: a panic instead of Jolt's out of bounds write
                        @panic("QuadTree: BuildTree stack overflow, the tree is too deep");
                    const new_stack = &stack[top];
                    const next_depth = cur_stack.depth + 1;
                    new_stack.node_idx = self.allocateNode(max_depth_mark_changed > next_depth);
                    new_stack.child_idx = -1;
                    new_stack.depth = next_depth;
                    new_stack.node_bounds_min = Vec3.replicate(large_float);
                    new_stack.node_bounds_max = Vec3.replicate(-large_float);
                    partition4(node_ids, centers, low, high, &new_stack.split);
                }
            }
        }

        // Store bounding box of root and return root
        return .{ .root = .fromNodeIndex(stack[0].node_idx), .bounds = .{ .min = stack[0].node_bounds_min, .max = stack[0].node_bounds_max } };
    }

    /// Mark node and all of its parents as changed
    fn markNodeAndParentsChanged(self: *const QuadTree, node_index: u32) void {
        var node_idx = node_index;

        while (true) {
            // If node has changed, parent will be too
            const node = self.getNode(node_idx);
            if (node.is_changed.load(.seq_cst) != 0)
                break;

            // Mark node as changed
            node.is_changed.store(1, .seq_cst);

            // Get our parent
            node_idx = node.parent_node_index.load(.seq_cst);
            if (node_idx == invalid_node_index)
                break;
        }
    }

    /// Widen parent bounds of node inNodeIndex to encapsulate inNewBounds, also mark node and all of its parents as changed
    fn widenAndMarkNodeAndParentsChanged(self: *const QuadTree, node_index: u32, new_bounds: AABox) void {
        var node_idx = node_index;

        while (true) {
            // Mark node as changed
            const node = self.getNode(node_idx);
            node.is_changed.store(1, .seq_cst);

            // Get our parent
            const parent_idx = node.parent_node_index.load(.seq_cst);
            if (parent_idx == invalid_node_index)
                break;

            // Find which child of the parent we're in
            const parent_node = self.getNode(parent_idx);
            const node_id = NodeID.fromNodeIndex(node_idx);
            var child_idx: i32 = -1;
            for (0..4) |i| {
                if (parent_node.child_node_id[i].eql(node_id)) {
                    // Found one, set the node index and child index and update the bounding box too
                    child_idx = @intCast(i);
                    break;
                }
            }
            std.debug.assert(child_idx != -1); // Nodes don't get removed from the tree, we must have found it

            // To avoid any race conditions with other threads we only enlarge bounding boxes
            if (!parent_node.encapsulateChildBounds(@intCast(child_idx), new_bounds)) {
                // No changes to bounding box, only marking as changed remains to be done
                if (parent_node.is_changed.load(.seq_cst) == 0)
                    self.markNodeAndParentsChanged(parent_idx);
                break;
            }

            // Update node index
            node_idx = parent_idx;
        }
    }

    /// Try to insert a new leaf to the tree at inNodeIndex
    fn tryInsertLeaf(self: *QuadTree, tracking: []Tracking, node_index: u32, leaf_id: NodeID, leaf_bounds: AABox, leaf_num_bodies: u32) bool {
        // Tentatively assign the node as parent
        const leaf_is_node = leaf_id.isNode();
        if (leaf_is_node) {
            const leaf_idx = leaf_id.getNodeIndex();
            self.getNode(leaf_idx).parent_node_index.store(node_index, .seq_cst);
        }

        // Fetch node that we're adding to
        const node = self.getNode(node_index);

        // Find an empty child
        for (0..4) |i| {
            const child_idx: u32 = @intCast(i);
            if (node.child_node_id[child_idx].compareExchange(.invalid(), leaf_id)) { // Check if we can claim it
                // We managed to add it to the node

                // If leaf was a body, we need to update its bookkeeping
                if (!leaf_is_node)
                    self.setBodyLocation(tracking, leaf_id.getBodyID(), node_index, child_idx);

                // Now set the bounding box making the child valid for queries
                node.setChildBounds(child_idx, leaf_bounds);

                // Widen the bounds for our parents too
                self.widenAndMarkNodeAndParentsChanged(node_index, leaf_bounds);

                // Update body counter
                _ = self.tree_state.num_bodies.fetchAdd(leaf_num_bodies, .seq_cst);

                // And we're done
                return true;
            }
        }

        return false;
    }

    /// Try to replace the existing root with a new root that contains both the existing root and the new leaf
    fn tryCreateNewRoot(self: *QuadTree, tracking: []Tracking, root_node_index: *std.atomic.Value(u32), leaf_id: NodeID, leaf_bounds: AABox, leaf_num_bodies: u32) bool {
        // Fetch old root
        const root_idx = root_node_index.load(.seq_cst);
        const root = self.getNode(root_idx);

        // Create new root, mark this new root as changed as we're not creating a very efficient tree at this point
        const new_root_idx = self.allocateNode(true);
        const new_root = self.getNode(new_root_idx);

        // First child is current root, note that since the tree may be modified concurrently we cannot assume that the bounds of our child will be correct so we set a very large bounding box
        new_root.child_node_id[0].set(.fromNodeIndex(root_idx));
        new_root.setChildBounds(0, .{ .min = .replicate(-large_float), .max = .replicate(large_float) });

        // Second child is new leaf
        new_root.child_node_id[1].set(leaf_id);
        new_root.setChildBounds(1, leaf_bounds);

        // Tentatively assign new root as parent
        const leaf_is_node = leaf_id.isNode();
        if (leaf_is_node) {
            const leaf_idx = leaf_id.getNodeIndex();
            self.getNode(leaf_idx).parent_node_index.store(new_root_idx, .seq_cst);
        }

        // Try to swap it
        if (root_node_index.cmpxchgStrong(root_idx, new_root_idx, .seq_cst, .seq_cst) == null) {
            // We managed to set the new root

            // If leaf was a body, we need to update its bookkeeping
            if (!leaf_is_node)
                self.setBodyLocation(tracking, leaf_id.getBodyID(), new_root_idx, 1);

            // Store parent node for old root
            root.parent_node_index.store(new_root_idx, .seq_cst);

            // Update body counter
            _ = self.tree_state.num_bodies.fetchAdd(leaf_num_bodies, .seq_cst);

            // And we're done
            return true;
        }

        // Failed to swap, someone else must have created a new root, try again
        self.nodeAllocator().destructObject(new_root_idx);
        return false;
    }

    /// Prepare adding inNumber bodies at ioBodyIDs to the quad tree, returns the state that should be used in AddBodiesFinalize.
    /// This can be done on a background thread without influencing the broadphase.
    /// ioBodyIDs may be shuffled around by this function.
    /// (Zolt: Allocator.Error from BuildTree's `new Vec3[]`, nothing changed then)
    pub fn addBodiesPrepare(self: *QuadTree, bodies: []const BodySlot, tracking: []Tracking, body_ids: []BodyID) Allocator.Error!AddState {
        return self.addBodiesPrepareWithCenters(bodies, tracking, body_ids, null);
    }

    /// Zolt: AddBodiesPrepare with a buffer for BuildTree's centers (at least body_ids.len entries), which then cannot
    /// fail (BroadPhaseQuadTree.notifyBodiesLayerChanged allocates it before it removes the bodies)
    pub fn addBodiesPrepareWithCenters(self: *QuadTree, bodies: []const BodySlot, tracking: []Tracking, body_ids: []BodyID, centers_buffer: ?[]Vec3) Allocator.Error!AddState {
        // Assert sane input
        std.debug.assert(body_ids.len > 0);

        // Below we just cast the body ID's to node ID's, check here that that is valid
        if (Core.enable_asserts) {
            for (body_ids) |b|
                _ = NodeID.fromBodyID(b);
        }

        // Build subtree for the new bodies, note that we mark all nodes as 'not changed'
        // so they will stay together as a batch and will make the tree rebuild cheaper
        const result = try self.buildTree(bodies, tracking, @ptrCast(body_ids), 0, centers_buffer);

        if (builtin.mode == .Debug and result.root.isNode())
            self.validateTree(bodies, tracking, result.root.getNodeIndex(), @intCast(body_ids.len));

        return .{ .leaf_id = result.root, .leaf_bounds = result.bounds };
    }

    /// Finalize adding bodies to the quadtree, supply the same number of bodies as in AddBodiesPrepare.
    pub fn addBodiesFinalize(self: *QuadTree, tracking: []Tracking, number_bodies: u32, state: *const AddState) void {
        // Assert sane input
        std.debug.assert(number_bodies > 0);

        // Mark tree dirty
        self.tree_state.is_dirty.store(true, .seq_cst);

        // Get the current root node
        const root_node = self.getCurrentRootMut();

        while (true) {
            // Check if we can insert the body in the root
            if (self.tryInsertLeaf(tracking, root_node.index.load(.seq_cst), state.leaf_id, state.leaf_bounds, number_bodies))
                return;

            // Check if we can create a new root
            if (self.tryCreateNewRoot(tracking, &root_node.index, state.leaf_id, state.leaf_bounds, number_bodies))
                return;
        }
    }

    /// Abort adding bodies to the quadtree, supply the same bodies and state as in AddBodiesPrepare.
    /// This can be done on a background thread without influencing the broadphase.
    pub fn addBodiesAbort(self: *QuadTree, tracking: []Tracking, state: *const AddState) void {
        // Collect all bodies
        var free_batch: NodeAllocator.Batch = .{};
        var node_stack: [stack_size]NodeID = undefined;
        node_stack[0] = state.leaf_id;
        std.debug.assert(node_stack[0].isValid());
        var top: i32 = 0;
        while (true) {
            // Check if node is a body
            const child_node_id = node_stack[@intCast(top)];
            if (child_node_id.isBody()) {
                // Reset location of body
                invalidateBodyLocation(tracking, child_node_id.getBodyID());
            } else {
                // Process normal node
                const node_idx = child_node_id.getNodeIndex();
                const node = self.getNode(node_idx);
                for (&node.child_node_id) |*child| {
                    const sub_child_node_id = child.get();
                    if (sub_child_node_id.isValid()) {
                        if (top >= stack_size) // JPH_ASSERT(top < cStackSize), Zolt: a panic instead of Jolt's out of bounds write
                            @panic("QuadTree: AddBodiesAbort stack overflow, the tree is too deep");
                        node_stack[@intCast(top)] = sub_child_node_id;
                        top += 1;
                    }
                }

                // Mark it to be freed
                self.nodeAllocator().addObjectToBatch(&free_batch, node_idx);
            }
            top -= 1;
            if (top < 0) break;
        }

        // Now free all nodes as a single batch
        self.nodeAllocator().destructObjectBatch(&free_batch);
    }

    /// Remove inNumber bodies in ioBodyIDs from the quadtree.
    pub fn removeBodies(self: *QuadTree, bodies: []const BodySlot, tracking: []Tracking, body_ids: []const BodyID) void {
        // Assert sane input
        std.debug.assert(body_ids.len > 0);

        // Mark tree dirty
        self.tree_state.is_dirty.store(true, .seq_cst);

        for (body_ids) |cur| {
            // Check if BodyID is correct
            std.debug.assert(bodies[cur.getIndex()].getBody().getID().eql(cur)); // Provided BodyID doesn't match BodyID in body manager

            // Get location of body
            const location = self.getBodyLocation(tracking, cur);

            // First we reset our internal bookkeeping
            invalidateBodyLocation(tracking, cur);

            // Then we make the bounding box invalid, no queries can find this node anymore
            const node = self.getNode(location.node_idx);
            node.invalidateChildBounds(location.child_idx);

            // Finally we reset the child id, this makes the node available for adds again
            node.child_node_id[location.child_idx].set(.invalid());

            // We don't need to bubble up our bounding box changes to our parents since we never make volumes smaller, only bigger
            // But we do need to mark the nodes as changed so that the tree can be rebuilt
            self.markNodeAndParentsChanged(location.node_idx);
        }

        _ = self.tree_state.num_bodies.fetchSub(@intCast(body_ids.len), .seq_cst);
    }

    /// Call whenever the aabb of a body changes.
    pub fn notifyBodiesAABBChanged(self: *QuadTree, bodies: []const BodySlot, tracking: []const Tracking, body_ids: []const BodyID) void {
        // Assert sane input
        std.debug.assert(body_ids.len > 0);

        for (body_ids) |cur| {
            // Check if BodyID is correct
            const body = bodies[cur.getIndex()].getBody();
            std.debug.assert(body.getID().eql(cur)); // Provided BodyID doesn't match BodyID in body manager

            // Get the new bounding box
            const new_bounds = body.getWorldSpaceBounds().*;

            // Get location of body
            const location = self.getBodyLocation(tracking, cur);

            // Widen bounds for node
            const node = self.getNode(location.node_idx);
            if (node.encapsulateChildBounds(location.child_idx, new_bounds)) {
                // Mark tree dirty
                self.tree_state.is_dirty.store(true, .seq_cst);

                // If bounds changed, widen the bounds for our parents too
                self.widenAndMarkNodeAndParentsChanged(location.node_idx, new_bounds);
            }
        }
    }

    /// A walk stack: Jolt's `Array<T, STLLocalAllocator<T, cStackSize>>` that is resized, cStackSize entries in a local
    /// buffer and the heap beyond that. Initialized in place (the local buffer must not move).
    fn WalkStack(comptime T: type) type {
        return struct {
            const Self = @This();

            local: STLLocalAllocator(T, stack_size),
            list: std.ArrayList(T),

            /// resize(cStackSize): the local buffer (the entries are not initialized, like Jolt's NodeIDs)
            fn init(self: *Self, heap: Allocator) void {
                self.local = .init(heap);
                self.list = .empty;
                self.list.ensureTotalCapacityPrecise(self.local.allocator(), stack_size) catch unreachable; // The local buffer has room
                self.list.items.len = stack_size;
            }

            fn deinit(self: *Self) void {
                self.list.deinit(self.local.allocator());
            }

            fn items(self: *const Self) []T {
                return self.list.items;
            }

            /// resize(inNewSize): falls back to the heap, the entries are kept
            fn resize(self: *Self, new_size: usize) Allocator.Error!void {
                try self.list.ensureTotalCapacityPrecise(self.local.allocator(), new_size);
                self.list.items.len = new_size;
            }
        };
    }

    /// Walk the node tree calling the Visitor::VisitNodes for each node encountered and Visitor::VisitBody for each body encountered
    /// (Zolt: Allocator.Error when the node stack, or the visitor's stack in OnStackResized, cannot grow on the heap)
    fn walkTree(self: *const QuadTree, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking, visitor: anytype) Allocator.Error!void {
        // Get the root
        const root_node = self.getCurrentRoot();

        var node_stack_array: WalkStack(NodeID) = undefined;
        node_stack_array.init(self.allocator);
        defer node_stack_array.deinit();
        var node_stack = node_stack_array.items();
        node_stack[0] = root_node.getNodeID();
        var top: i32 = 0;
        while (true) {
            // Check if node is a body
            const child_node_id = node_stack[@intCast(top)];
            if (child_node_id.isBody()) {
                const body_id = child_node_id.getBodyID();
                const object_layer = tracking[body_id.getIndex()].object_layer.load(.seq_cst); // We're not taking a lock on the body, so it may be in the process of being removed so check if the object layer is invalid
                if (object_layer != object_layer_invalid and object_layer_filter.shouldCollide(object_layer)) {
                    // We found a body we collide with, call our visitor
                    visitor.visitBody(body_id, top);

                    // Check if we're done
                    if (visitor.shouldAbort())
                        break;
                }
            } else if (child_node_id.isValid()) {
                // Ensure there is space on the stack (falls back to heap if there isn't)
                if (top + 4 >= node_stack.len) {
                    quadTreePerformanceWarning(self);
                    try node_stack_array.resize(node_stack.len << 1);
                    node_stack = node_stack_array.items();
                    try visitor.onStackResized(node_stack.len);
                }

                // Process normal node
                const node = self.getNode(child_node_id.getNodeIndex());
                std.debug.assert(std.mem.isAligned(@intFromPtr(node), Core.cache_line_size));

                // Load bounds of 4 children
                const bounds_minx = node.loadBounds(.bounds_min_x);
                const bounds_miny = node.loadBounds(.bounds_min_y);
                const bounds_minz = node.loadBounds(.bounds_min_z);
                const bounds_maxx = node.loadBounds(.bounds_max_x);
                const bounds_maxy = node.loadBounds(.bounds_max_y);
                const bounds_maxz = node.loadBounds(.bounds_max_z);

                // Load ids for 4 children
                var child_ids = node.loadChildIDs();

                // Check which sub nodes to visit
                const num_results = visitor.visitNodes(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, &child_ids, top);
                child_ids.storeInt4(@ptrCast(node_stack[@intCast(top)..][0..4]));
                top += num_results;
            }

            // Fetch next node until we find one that the visitor wants to see
            while (true) {
                top -= 1;
                if (!(top >= 0 and !visitor.shouldVisitNode(top))) break;
            }

            if (top < 0) break;
        }
    }

    /// Cast a ray and get the intersecting bodies in ioCollector.
    pub fn castRay(self: *const QuadTree, ray: *const RayCast, collector: *RayCastBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Visitor = struct {
            origin: Vec3,
            inv_direction: RayInvDirection,
            collector: *RayCastBodyCollector,
            fraction_stack: WalkStack(f32),

            /// Constructor (in place)
            fn init(v: *@This(), in_ray: *const RayCast, in_collector: *RayCastBodyCollector, heap: Allocator) void {
                v.origin = in_ray.origin;
                v.inv_direction = .init(in_ray.direction);
                v.collector = in_collector;
                v.fraction_stack.init(heap);
                v.fraction_stack.items()[0] = -1;
            }

            fn deinit(v: *@This()) void {
                v.fraction_stack.deinit();
            }

            /// Returns true if further processing of the tree should be aborted
            pub fn shouldAbort(v: *const @This()) bool {
                return v.collector.shouldEarlyOut();
            }

            /// Returns true if this node / body should be visited, false if no hit can be generated
            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.fraction_stack.items()[@intCast(stack_top)] < v.collector.getEarlyOutFraction();
            }

            /// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
            pub fn visitNodes(v: *@This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, child_node_ids: *UVec4, stack_top: i32) i32 {
                // Test the ray against 4 bounding boxes
                const fraction = RayAABox.rayAABox4(v.origin, v.inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return SortReverseAndStore.sortReverseAndStore(fraction, v.collector.getEarlyOutFraction(), child_node_ids, v.fraction_stack.items()[@intCast(stack_top)..][0..4]);
            }

            /// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
            pub fn visitBody(v: *@This(), body_id: BodyID, stack_top: i32) void {
                // Store potential hit with body
                const result: BroadPhaseCastResult = .{ .body_id = body_id, .fraction = v.fraction_stack.items()[@intCast(stack_top)] };
                v.collector.addHit(&result);
            }

            /// Called when the stack is resized, this allows us to resize the fraction stack to match the new stack size
            pub fn onStackResized(v: *@This(), new_stack_size: usize) Allocator.Error!void {
                try v.fraction_stack.resize(new_stack_size);
            }
        };

        var visitor: Visitor = undefined;
        visitor.init(ray, collector, self.allocator);
        defer visitor.deinit();
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// The visitor of the collide queries: Test(box) is the 4 box overlap test, every node is visited and every hit
    /// is reported (the visitors of CollideAABox, CollideSphere, CollidePoint and CollideOrientedBox only differ in
    /// their test)
    fn CollideVisitor(comptime Test: type) type {
        return struct {
            test_data: Test,
            collector: *CollideShapeBodyCollector,

            /// Returns true if further processing of the tree should be aborted
            pub fn shouldAbort(v: *const @This()) bool {
                return v.collector.shouldEarlyOut();
            }

            /// Returns true if this node / body should be visited, false if no hit can be generated
            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                _ = .{ v, stack_top };
                return true;
            }

            /// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
            pub fn visitNodes(v: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, child_node_ids: *UVec4, stack_top: i32) i32 {
                _ = stack_top;
                const hitting = v.test_data.hitting(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
                return SortReverseAndStore.countAndSortTrues(hitting, child_node_ids);
            }

            /// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
            pub fn visitBody(v: *const @This(), body_id: BodyID, stack_top: i32) void {
                _ = stack_top;

                // Store potential hit with body
                v.collector.addHit(&body_id);
            }

            /// Called when the stack is resized
            pub fn onStackResized(v: *const @This(), new_stack_size: usize) Allocator.Error!void {
                // Nothing to do
                _ = .{ v, new_stack_size };
            }
        };
    }

    /// Get bodies intersecting with inBox in ioCollector
    pub fn collideAABox(self: *const QuadTree, box: *const AABox, collector: *CollideShapeBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Test = struct {
            box: *const AABox,

            fn hitting(t: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
                // Test the box vs 4 boxes
                return AABox4.aabox4VsBox(t.box.*, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
            }
        };

        const visitor: CollideVisitor(Test) = .{ .test_data = .{ .box = box }, .collector = collector };
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// Get bodies intersecting with a sphere in ioCollector
    pub fn collideSphere(self: *const QuadTree, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Test = struct {
            center_x: Vec4,
            center_y: Vec4,
            center_z: Vec4,
            radius_sq: Vec4,

            fn hitting(t: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
                // Test 4 boxes vs sphere
                return AABox4.aabox4VsSphere(t.center_x, t.center_y, t.center_z, t.radius_sq, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
            }
        };

        const visitor: CollideVisitor(Test) = .{
            .test_data = .{
                .center_x = center.splatX(),
                .center_y = center.splatY(),
                .center_z = center.splatZ(),
                .radius_sq = Vec4.replicate(math.square(radius)),
            },
            .collector = collector,
        };
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// Get bodies intersecting with a point and any hits to ioCollector
    pub fn collidePoint(self: *const QuadTree, point: Vec3, collector: *CollideShapeBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Test = struct {
            point: Vec3,

            fn hitting(t: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
                // Test if point overlaps with box
                return AABox4.aabox4VsPoint(t.point, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
            }
        };

        const visitor: CollideVisitor(Test) = .{ .test_data = .{ .point = point }, .collector = collector };
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// Get bodies intersecting with an oriented box and any hits to ioCollector
    pub fn collideOrientedBox(self: *const QuadTree, box: *const OrientedBox, collector: *CollideShapeBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Test = struct {
            box: OrientedBox,

            fn hitting(t: *const @This(), bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
                // Test if point overlaps with box
                return AABox4.aabox4VsOrientedBox(t.box, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z, .{});
            }
        };

        const visitor: CollideVisitor(Test) = .{ .test_data = .{ .box = box.* }, .collector = collector };
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// Cast a box and get intersecting bodies in ioCollector
    pub fn castAABox(self: *const QuadTree, box: *const AABoxCast, collector: *CastShapeBodyCollector, object_layer_filter: *const ObjectLayerFilter, tracking: []const Tracking) Allocator.Error!void {
        const Visitor = struct {
            origin: Vec3,
            extent: Vec3,
            inv_direction: RayInvDirection,
            collector: *CastShapeBodyCollector,
            fraction_stack: WalkStack(f32),

            /// Constructor (in place)
            fn init(v: *@This(), in_box: *const AABoxCast, in_collector: *CastShapeBodyCollector, heap: Allocator) void {
                v.origin = in_box.box.getCenter();
                v.extent = in_box.box.getExtent();
                v.inv_direction = .init(in_box.direction);
                v.collector = in_collector;
                v.fraction_stack.init(heap);
                v.fraction_stack.items()[0] = -1;
            }

            fn deinit(v: *@This()) void {
                v.fraction_stack.deinit();
            }

            /// Returns true if further processing of the tree should be aborted
            pub fn shouldAbort(v: *const @This()) bool {
                return v.collector.shouldEarlyOut();
            }

            /// Returns true if this node / body should be visited, false if no hit can be generated
            pub fn shouldVisitNode(v: *const @This(), stack_top: i32) bool {
                return v.fraction_stack.items()[@intCast(stack_top)] < v.collector.getPositiveEarlyOutFraction();
            }

            /// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
            pub fn visitNodes(v: *@This(), bounds_min_x_in: Vec4, bounds_min_y_in: Vec4, bounds_min_z_in: Vec4, bounds_max_x_in: Vec4, bounds_max_y_in: Vec4, bounds_max_z_in: Vec4, child_node_ids: *UVec4, stack_top: i32) i32 {
                // Enlarge them by the casted aabox extents
                var bounds_min_x = bounds_min_x_in;
                var bounds_min_y = bounds_min_y_in;
                var bounds_min_z = bounds_min_z_in;
                var bounds_max_x = bounds_max_x_in;
                var bounds_max_y = bounds_max_y_in;
                var bounds_max_z = bounds_max_z_in;
                AABox4.aabox4EnlargeWithExtent(v.extent, &bounds_min_x, &bounds_min_y, &bounds_min_z, &bounds_max_x, &bounds_max_y, &bounds_max_z);

                // Test 4 children
                const fraction = RayAABox.rayAABox4(v.origin, v.inv_direction, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return SortReverseAndStore.sortReverseAndStore(fraction, v.collector.getPositiveEarlyOutFraction(), child_node_ids, v.fraction_stack.items()[@intCast(stack_top)..][0..4]);
            }

            /// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
            pub fn visitBody(v: *@This(), body_id: BodyID, stack_top: i32) void {
                // Store potential hit with body
                const result: BroadPhaseCastResult = .{ .body_id = body_id, .fraction = v.fraction_stack.items()[@intCast(stack_top)] };
                v.collector.addHit(&result);
            }

            /// Called when the stack is resized, this allows us to resize the fraction stack to match the new stack size
            pub fn onStackResized(v: *@This(), new_stack_size: usize) Allocator.Error!void {
                try v.fraction_stack.resize(new_stack_size);
            }
        };

        var visitor: Visitor = undefined;
        visitor.init(box, collector, self.allocator);
        defer visitor.deinit();
        try self.walkTree(object_layer_filter, tracking, &visitor);
    }

    /// Find all colliding pairs between dynamic bodies, calls ioPairCollector for every pair found
    /// (Zolt: Allocator.Error when the node stack cannot grow on the heap, the pairs found so far were reported)
    pub fn findCollidingPairs(self: *const QuadTree, bodies: []const BodySlot, active_bodies: []const BodyID, speculative_contact_distance: f32, pair_collector: *BodyPairCollector, object_layer_pair_filter: *const ObjectLayerPairFilter) Allocator.Error!void {
        // Note that we don't lock the tree at this point. We know that the tree is not going to be swapped or deleted while finding collision pairs due to the way the jobs are scheduled in the PhysicsSystem::Update.
        // We double check this at the end of the function.
        const root_node = self.getCurrentRoot();
        std.debug.assert(root_node.index.load(.seq_cst) != invalid_node_index);

        // Assert sane input
        std.debug.assert(active_bodies.len > 0);

        var node_stack_array: WalkStack(NodeID) = undefined;
        node_stack_array.init(self.allocator);
        defer node_stack_array.deinit();
        var node_stack = node_stack_array.items();

        // Loop over all active bodies
        for (active_bodies) |b1_id| {
            const body1 = bodies[b1_id.getIndex()].getBody();
            std.debug.assert(!body1.isStatic());

            const start_tick = if (track_simulation_stats) getProcessorTickCount() else {};

            // Expand the bounding box by the speculative contact distance
            var bounds1 = body1.getWorldSpaceBounds().*;
            bounds1.expandBy(Vec3.replicate(speculative_contact_distance));

            // Test each body with the tree
            node_stack[0] = root_node.getNodeID();
            var top: i32 = 0;
            while (true) {
                // Check if node is a body
                const child_node_id = node_stack[@intCast(top)];
                if (child_node_id.isBody()) {
                    // Don't collide with self
                    const b2_id = child_node_id.getBodyID();
                    if (!b1_id.eql(b2_id)) {
                        // Collision between dynamic pairs need to be picked up only once
                        const body2 = bodies[b2_id.getIndex()].getBody();
                        if (object_layer_pair_filter.shouldCollide(body1.getObjectLayer(), body2.getObjectLayer()) and
                            Body.findCollidingPairsCanCollide(body1, body2) and
                            bounds1.overlaps(body2.getWorldSpaceBounds().*)) // In the broadphase we widen the bounding box when a body moves, do a final check to see if the bounding boxes actually overlap
                        {
                            // Store potential hit between bodies
                            pair_collector.addHit(&BodyPair.init(b1_id, b2_id));
                        }
                    }
                } else if (child_node_id.isValid()) {
                    // Process normal node
                    const node = self.getNode(child_node_id.getNodeIndex());
                    std.debug.assert(std.mem.isAligned(@intFromPtr(node), Core.cache_line_size));

                    // Get bounds of 4 children
                    const bounds_minx = node.loadBounds(.bounds_min_x);
                    const bounds_miny = node.loadBounds(.bounds_min_y);
                    const bounds_minz = node.loadBounds(.bounds_min_z);
                    const bounds_maxx = node.loadBounds(.bounds_max_x);
                    const bounds_maxy = node.loadBounds(.bounds_max_y);
                    const bounds_maxz = node.loadBounds(.bounds_max_z);

                    // Test overlap
                    const overlap = AABox4.aabox4VsBox(bounds1, bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz);
                    const num_results: i32 = @intCast(overlap.countTrues());
                    if (num_results > 0) {
                        // Load ids for 4 children
                        var child_ids = node.loadChildIDs();

                        // Sort so that overlaps are first
                        child_ids = UVec4.sort4True(overlap, child_ids);

                        // Ensure there is space on the stack (falls back to heap if there isn't)
                        if (top + 4 >= node_stack.len) {
                            quadTreePerformanceWarning(self);
                            try node_stack_array.resize(node_stack.len << 1);
                            node_stack = node_stack_array.items();
                        }

                        // Push them onto the stack
                        child_ids.storeInt4(@ptrCast(node_stack[@intCast(top)..][0..4]));
                        top += num_results;
                    }
                }
                top -= 1;
                if (top < 0) break;
            }

            if (track_simulation_stats) {
                // Zolt (Rule M): the stats are written through the body pointer of the body slot (Jolt: a const_cast)
                const num_ticks = getProcessorTickCount() - start_tick;
                bodies[b1_id.getIndex()].getBody().getMotionPropertiesUncheckedMut().?.getSimulationStatsMut().broad_phase_ticks += num_ticks;
            }
        }

        // Test that the root node was not swapped while finding collision pairs.
        // This would mean that UpdateFinalize/DiscardOldTree ran during collision detection which should not be possible due to the way the jobs are scheduled.
        std.debug.assert(root_node.index.load(.seq_cst) != invalid_node_index);
        std.debug.assert(root_node == self.getCurrentRoot());
    }

    /// Validate that the tree is consistent (JPH_DEBUG, Zolt: Debug builds).
    /// Note: This function only works if the tree is not modified while we're traversing it.
    /// (Zolt: skipped when its stack cannot grow on the heap)
    fn validateTree(self: *const QuadTree, bodies: []const BodySlot, tracking: []const Tracking, node_index: u32, num_expected_bodies: u32) void {
        self.validateTreeImpl(bodies, tracking, node_index, num_expected_bodies) catch {};
    }

    fn validateTreeImpl(self: *const QuadTree, bodies: []const BodySlot, tracking: []const Tracking, node_index: u32, num_expected_bodies: u32) Allocator.Error!void {
        // Root should be valid
        std.debug.assert(node_index != invalid_node_index);

        // To avoid call overhead, create a stack in place
        const StackEntry = struct {
            node_index: u32,
            parent_node_index: u32,
        };
        var stack_allocator = STLLocalAllocator(StackEntry, stack_size).init(self.allocator);
        const stack_alloc = stack_allocator.allocator();
        var stack: std.ArrayList(StackEntry) = .empty;
        defer stack.deinit(stack_alloc);
        stack.ensureTotalCapacityPrecise(stack_alloc, stack_size) catch unreachable; // reserve(cStackSize): the local buffer
        stack.appendAssumeCapacity(.{ .node_index = node_index, .parent_node_index = invalid_node_index });

        var num_bodies: u32 = 0;

        while (true) {
            // Copy entry from the stack
            const cur_stack = stack.pop().?;

            // Validate parent
            const node = self.getNode(cur_stack.node_index);
            std.debug.assert(node.parent_node_index.load(.seq_cst) == cur_stack.parent_node_index);

            // Validate that when a parent is not-changed that all of its children are also
            std.debug.assert(cur_stack.parent_node_index == invalid_node_index or self.getNode(cur_stack.parent_node_index).is_changed.load(.seq_cst) != 0 or node.is_changed.load(.seq_cst) == 0);

            // Loop children
            for (0..4) |i| {
                const child_node_id = node.child_node_id[i].get();
                if (child_node_id.isValid()) {
                    if (child_node_id.isNode()) {
                        // Child is a node, recurse
                        const child_idx = child_node_id.getNodeIndex();
                        try stack.append(stack_alloc, .{ .node_index = child_idx, .parent_node_index = cur_stack.node_index });

                        // Validate that the bounding box is bigger or equal to the bounds in the tree
                        // Bounding box could also be invalid if all children of our child were removed
                        const child_bounds = node.getChildBounds(i);
                        const real_child_bounds = self.getNode(child_idx).getNodeBounds();
                        std.debug.assert(child_bounds.contains(real_child_bounds) or !real_child_bounds.isValid());
                    } else {
                        // Increment number of bodies found
                        num_bodies += 1;

                        // Check if tracker matches position of body
                        const location = self.getBodyLocation(tracking, child_node_id.getBodyID());
                        std.debug.assert(location.node_idx == cur_stack.node_index);
                        std.debug.assert(location.child_idx == i);

                        // Validate that the body cached bounds still match the actual bounds
                        const body = bodies[child_node_id.getBodyID().getIndex()].getBody();
                        body.validateCachedBounds();

                        // Validate that the node bounds are bigger or equal to the body bounds
                        const body_bounds = node.getChildBounds(i);
                        std.debug.assert(body_bounds.contains(body.getWorldSpaceBounds().*));
                    }
                }
            }

            if (stack.items.len == 0) break;
        }

        // Check that the amount of bodies in the tree matches our counter
        std.debug.assert(num_bodies == num_expected_bodies);
    }

    /// Debug function to get the depth of the tree from node inNodeID
    pub fn getMaxTreeDepth(self: *const QuadTree, node_id: NodeID) u32 {
        // Reached a leaf?
        if (!node_id.isValid() or node_id.isBody())
            return 0;

        // Recurse to children
        var max_depth: u32 = 0;
        const node = self.getNode(node_id.getNodeIndex());
        for (&node.child_node_id) |*child|
            max_depth = @max(max_depth, self.getMaxTreeDepth(child.get()));
        return max_depth + 1;
    }
};

/// sQuadTreePerformanceWarning (Zolt: the `static atomic<bool> triggered_report` is the latch of the tree's broad phase)
fn quadTreePerformanceWarning(tree: *const QuadTree) void {
    if (Core.enable_asserts) {
        const triggered_report = tree.triggered_report orelse return;
        if (triggered_report.cmpxchgStrong(false, true, .seq_cst, .seq_cst) == null)
            log.info("QuadTree: Performance warning: Stack full!\n" ++
                "This must be a very deep tree. Are you batch adding bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize?\n" ++
                "If you add lots of bodies through BodyInterface::AddBody you may need to call PhysicsSystem::OptimizeBroadPhase to rebuild the tree.", .{});
    }
}

test "QuadTree NodeID and Node layout" {
    const NodeID = QuadTree.NodeID;
    const body = NodeID.fromBodyID(.init(0x00800005));
    try std.testing.expect(body.isValid() and body.isBody() and !body.isNode());
    try std.testing.expect(body.getBodyID().eql(.init(0x00800005)));
    try std.testing.expect(body.eqlBodyID(.init(0x00800005)));
    const node = NodeID.fromNodeIndex(7);
    try std.testing.expect(node.isValid() and node.isNode() and !node.isBody());
    try std.testing.expectEqual(@as(u32, 7), node.getNodeIndex());
    try std.testing.expectEqual(@as(u32, 0x80000007), node.id);
    try std.testing.expect(!NodeID.invalid().isValid());

    var atomic: QuadTree.AtomicNodeID = .init(.invalid());
    try std.testing.expect(!atomic.isValid());
    try std.testing.expect(atomic.compareExchange(.invalid(), node));
    try std.testing.expect(!atomic.compareExchange(.invalid(), body));
    try std.testing.expect(atomic.eql(node) and atomic.get().eql(node));
    atomic.set(body);
    try std.testing.expect(atomic.eqlBodyID(.init(0x00800005)));

    // A new node: invalid bounds (no query can enter it), invalid children, no parent
    var n: QuadTree.Node align(64) = .init(true); // Aligned like the nodes of the allocator (loadBounds)
    try std.testing.expect(!n.getNodeBounds().isValid());
    try std.testing.expectEqual(@as(u32, 1), n.is_changed.load(.monotonic));
    try std.testing.expectEqual(invalid_node_index, n.parent_node_index.load(.monotonic));
    for (&n.child_node_id) |*c| try std.testing.expect(!c.isValid());
    n.setChildBounds(2, .init(Vec3.init(-1, -2, -3), Vec3.init(1, 2, 3)));
    try std.testing.expect(n.getChildBounds(2).min.eql(Vec3.init(-1, -2, -3)));
    try std.testing.expect(n.getNodeBounds().max.eql(Vec3.init(1, 2, 3)));
    try std.testing.expect(!n.encapsulateChildBounds(2, .init(Vec3.zero(), Vec3.one())));
    try std.testing.expect(n.encapsulateChildBounds(2, .init(Vec3.zero(), Vec3.init(1, 2, 4))));
    try std.testing.expect(n.encapsulateChildBounds(2, .init(Vec3.init(-1, -2, -5), Vec3.zero())));
    try std.testing.expect(n.getChildBounds(2).min.eql(Vec3.init(-1, -2, -5)));
    try std.testing.expect(n.getChildBounds(2).max.eql(Vec3.init(1, 2, 4)));
    n.invalidateChildBounds(2);
    try std.testing.expect(!n.getChildBounds(2).isValid());
    try std.testing.expect(n.loadBounds(.bounds_min_x).eql(Vec4.replicate(large_float)));
    try std.testing.expectEqual(@as(usize, 124), @sizeOf(QuadTree.Node));
}

test "QuadTree node loads give the same bits as per lane loads" {
    var prng = std.Random.DefaultPrng.init(0x9d);
    const random = prng.random();
    var node: QuadTree.Node align(64) = undefined; // Aligned like the nodes of the allocator
    const n = &node;
    for (0..100) |_| {
        // Random bits, including NaN patterns: the loads must not change any bit
        const words: *[31]u32 = @ptrCast(n);
        for (words) |*w| w.* = random.int(u32);
        inline for (std.meta.fields(QuadTree.Node.BoundsArray)) |field| {
            const array = &@field(n, field.name);
            const loaded: [4]u32 = @bitCast(n.loadBounds(@enumFromInt(field.value)).value);
            for (0..4) |i| try std.testing.expectEqual(@as(u32, @bitCast(array[i].load(.monotonic))), loaded[i]);
        }
        const ids: [4]u32 = n.loadChildIDs().value;
        for (0..4) |i| try std.testing.expectEqual(n.child_node_id[i].id.load(.acquire), ids[i]);
    }
}

test "QuadTree partition" {
    const NodeID = QuadTree.NodeID;
    // Trivial cases: half
    var ids: [9]NodeID = undefined;
    var centers: [9]Vec3 = undefined;
    for (&ids, &centers, 0..) |*id, *c, i| {
        id.* = .fromNodeIndex(@intCast(i));
        c.* = Vec3.init(@floatFromInt(8 - i), 0, 0); // Descending x
    }
    try std.testing.expectEqual(@as(u32, 2), QuadTree.partition(ids[0..4], centers[0..4]));
    try std.testing.expectEqual(@as(u32, 1), QuadTree.partition(ids[0..3], centers[0..3]));

    // Split at x = 4: 4 elements below
    try std.testing.expectEqual(@as(u32, 4), QuadTree.partition(&ids, &centers));
    for (centers[0..4]) |c| try std.testing.expect(c.getX() < 4);
    for (centers[4..]) |c| try std.testing.expect(c.getX() >= 4);
    for (&ids, &centers) |id, c| try std.testing.expectEqual(@as(u32, @intFromFloat(8 - c.getX())), id.getNodeIndex()); // Shuffled together

    // All at the same position: the split fails and the middle is used
    for (&centers) |*c| c.* = Vec3.replicate(1);
    try std.testing.expectEqual(@as(u32, 4), QuadTree.partition(&ids, &centers));

    // partition4 makes 4 ranges
    for (&centers, 0..) |*c, i| c.* = Vec3.init(0, @floatFromInt(i * i), 0);
    var split: [5]u32 = undefined;
    QuadTree.partition4(&ids, &centers, 0, 9, &split);
    try std.testing.expectEqual(@as(u32, 0), split[0]);
    try std.testing.expectEqual(@as(u32, 9), split[4]);
    try std.testing.expect(split[0] <= split[1] and split[1] <= split[2] and split[2] <= split[3] and split[3] <= split[4]);
}

/// Test helper: a body manager with `num_levels * 3` static spheres along x and one active dynamic box that overlaps
/// all of them, and a tree built by hand that is deeper than any BuildTree: node i has the children [3 bodies, node
/// i + 1], so the walks visit the deep child first and keep 3 entries per level on their stack
const DeepTreeTest = struct {
    const BodyManager = @import("../../Body/BodyManager.zig").BodyManager;
    const BodyCreationSettings = @import("../../Body/BodyCreationSettings.zig").BodyCreationSettings;
    const SphereShape = @import("../Shape/SphereShape.zig").SphereShape;
    const BoxShape = @import("../Shape/BoxShape.zig").BoxShape;
    const RVec3 = @import("../../../Math/Real.zig").RVec3;
    const Quat = @import("../../../Math/Quat.zig").Quat;
    const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;

    const num_levels = 100;
    const num_bodies = 3 * num_levels;

    const LayerInterface = struct {
        pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };
        base: BroadPhaseLayerInterface = .init(@This()),
        pub fn getNumBroadPhaseLayers(self: *const @This()) u32 {
            _ = self;
            return 1;
        }
        pub fn getBroadPhaseLayer(self: *const @This(), layer: ObjectLayer) BroadPhaseLayer {
            _ = .{ self, layer };
            return .init(0);
        }
    };

    layer_interface: LayerInterface = .{},
    body_manager: BodyManager = .empty,
    ids: [num_bodies]BodyID = undefined,
    active: BodyID = .invalid,
    node_allocator: QuadTree.NodeAllocator = .empty,
    tracking: []QuadTree.Tracking = &.{},

    fn init(self: *DeepTreeTest, allocator: Allocator) !void {
        self.* = .{};
        try self.body_manager.init(allocator, std.testing.io, num_bodies + 1, 0, &self.layer_interface.base);
        var settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 0.25, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, 0);
        defer settings.deinit();
        for (&self.ids, 0..) |*id, i| {
            settings.position = RVec3.init(@floatFromInt(i), 0, 0);
            const body = try self.body_manager.allocateBody(&settings);
            std.debug.assert(self.body_manager.addBody(body));
            id.* = body.getID();
        }
        var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.init(400, 1, 1), .{})).asShape(), RVec3.zero(), Quat.identity(), .dynamic, 0);
        defer box_settings.deinit();
        const body = try self.body_manager.allocateBody(&box_settings);
        std.debug.assert(self.body_manager.addBody(body));
        self.active = body.getID();
        body.setInBroadPhaseInternal(true); // ActivateBodies only activates bodies in the broad phase
        self.body_manager.activateBodies(&.{self.active});
        self.node_allocator = try .init(allocator, std.testing.io, 2 * num_levels, 64);
        try self.node_allocator.reserveAllPages();
        self.tracking = try allocator.alloc(QuadTree.Tracking, num_bodies + 1);
        @memset(self.tracking, .{});
    }

    fn deinit(self: *DeepTreeTest, allocator: Allocator) void {
        allocator.free(self.tracking);
        self.node_allocator.deinit();
        self.body_manager.deinit();
    }

    /// Builds the deep tree in `tree` (initialized, empty) by hand
    fn build(self: *DeepTreeTest, tree: *QuadTree) void {
        const bodies = self.body_manager.getBodies();
        var node_idx: [num_levels]u32 = undefined;
        node_idx[0] = tree.getCurrentRoot().index.load(.monotonic);
        for (node_idx[1..]) |*n| n.* = tree.allocateNode(false);
        var subtree: AABox = .empty;
        var level: usize = num_levels;
        while (level > 0) {
            level -= 1;
            const node = tree.getNode(node_idx[level]);
            var bounds: AABox = .empty;
            for (0..3) |c| {
                const id = self.ids[3 * level + c];
                const body_bounds = bodies[id.getIndex()].getBody().getWorldSpaceBounds().*;
                node.child_node_id[c].set(.fromBodyID(id));
                node.setChildBounds(c, body_bounds);
                tree.setBodyLocation(self.tracking, id, node_idx[level], @intCast(c));
                self.tracking[id.getIndex()].broad_phase_layer.store(0, .monotonic);
                self.tracking[id.getIndex()].object_layer.store(0, .monotonic);
                bounds.encapsulate(body_bounds);
            }
            if (level + 1 < num_levels) {
                node.child_node_id[3].set(.fromNodeIndex(node_idx[level + 1]));
                node.setChildBounds(3, subtree);
                tree.getNode(node_idx[level + 1]).parent_node_index.store(node_idx[level], .monotonic);
                bounds.encapsulate(subtree);
            }
            subtree = bounds;
        }
        tree.tree_state.num_bodies.store(num_bodies, .monotonic);
    }

    /// A snapshot of the current tree (node indices, parents, flags, children and bounds in walk order)
    fn dump(allocator: Allocator, tree: *const QuadTree) ![]u32 {
        var out: std.ArrayList(u32) = .empty;
        errdefer out.deinit(allocator);
        var stack: std.ArrayList(u32) = .empty;
        defer stack.deinit(allocator);
        try out.appendSlice(allocator, &.{ tree.tree_state.num_bodies.load(.monotonic), @intFromBool(tree.isDirty()), tree.free_node_batch.num_objects });
        try stack.append(allocator, tree.getCurrentRoot().index.load(.monotonic));
        while (stack.pop()) |node_idx| {
            const node = tree.getNode(node_idx);
            try out.appendSlice(allocator, &.{ node_idx, node.parent_node_index.load(.monotonic), node.is_changed.load(.monotonic) });
            for (0..4) |i| {
                const child = node.child_node_id[i].get();
                const bounds = node.getChildBounds(i);
                try out.appendSlice(allocator, &.{ child.id, @bitCast(bounds.min.getX()), @bitCast(bounds.max.getX()) });
                if (child.isValid() and child.isNode()) try stack.append(allocator, child.getNodeIndex());
            }
        }
        return out.toOwnedSlice(allocator);
    }
};

test "QuadTree deep tree: the walk stacks grow on the heap, findCollidingPairs, the casts and updatePrepare out of memory" {
    const allocator = std.testing.allocator;
    var t: DeepTreeTest = undefined;
    try t.init(allocator);
    defer t.deinit(allocator);
    const bodies = t.body_manager.getBodies();

    {
        var tree: QuadTree = .empty;
        tree.init(&t.node_allocator, allocator);
        defer tree.deinit(); // Frees the deep tree without allocating
        t.build(&tree);
        if (builtin.mode == .Debug) tree.validateTree(bodies, t.tracking, tree.getCurrentRoot().index.load(.monotonic), DeepTreeTest.num_bodies);
        try std.testing.expectEqual(@as(u32, DeepTreeTest.num_levels), tree.getMaxTreeDepth(tree.getCurrentRoot().getNodeID()));

        // A query that overlaps everything: 3 stack entries per level, the stack leaves its local buffer
        const AllHits = @import("../CollisionCollectorImpl.zig").AllHitCollisionCollector;
        var collector = AllHits(CollideShapeBodyCollector).init(allocator);
        defer collector.deinit();
        try tree.collideAABox(&AABox.init(Vec3.replicate(-1000), Vec3.replicate(1000)), &collector.base, &.{}, t.tracking);
        try collector.checkError();
        try std.testing.expectEqual(@as(usize, DeepTreeTest.num_bodies), collector.hits.items.len);

        // FindCollidingPairs: every body overlaps the active box
        var pairs = AllHits(BodyPairCollector).init(allocator);
        defer pairs.deinit();
        try tree.findCollidingPairs(bodies, &.{t.active}, 0.0, &pairs.base, &.{});
        try pairs.checkError();
        try std.testing.expectEqual(@as(usize, DeepTreeTest.num_bodies), pairs.hits.items.len);

        // The stack cannot grow: error.OutOfMemory (here before any body was reached: the walk descends first)
        var failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = 0 });
        tree.allocator = failing.allocator();
        pairs.reset();
        try std.testing.expectError(error.OutOfMemory, tree.findCollidingPairs(bodies, &.{t.active}, 0.0, &pairs.base, &.{}));
        try std.testing.expect(failing.has_induced_failure);
        try std.testing.expect(pairs.hits.items.len < DeepTreeTest.num_bodies);
        tree.allocator = allocator;

        // The casts, from beyond the last body towards the first: the deep child is the closest hit, so the 3 bodies of
        // every level stay on the stack (node IDs and fractions)
        const cast: AABoxCast = .{ .box = .init(Vec3.init(399.9, -0.1, -0.1), Vec3.init(400.1, 0.1, 0.1)), .direction = Vec3.init(-500, 0, 0) };
        const ray: RayCast = .init(Vec3.init(400, 0, 0), Vec3.init(-500, 0, 0));
        {
            var cast_hits = AllHits(CastShapeBodyCollector).init(allocator);
            defer cast_hits.deinit();
            try tree.castAABox(&cast, &cast_hits.base, &.{}, t.tracking);
            try cast_hits.checkError();
            try std.testing.expectEqual(@as(usize, DeepTreeTest.num_bodies), cast_hits.hits.items.len);
            var ray_hits = AllHits(RayCastBodyCollector).init(allocator);
            defer ray_hits.deinit();
            try tree.castRay(&ray, &ray_hits.base, &.{}, t.tracking);
            try ray_hits.checkError();
            try std.testing.expectEqual(@as(usize, DeepTreeTest.num_bodies), ray_hits.hits.items.len);
        }

        // The node stack (first allocation) or the fraction stack (second allocation, OnStackResized) cannot grow:
        // error.OutOfMemory, which CastAABoxNoLock passes on to the FindCCDContacts job (the hits so far were reported)
        for (0..2) |fail_index| {
            var cast_failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index });
            tree.allocator = cast_failing.allocator();
            var cast_hits = AllHits(CastShapeBodyCollector).init(allocator);
            defer cast_hits.deinit();
            try std.testing.expectError(error.OutOfMemory, tree.castAABox(&cast, &cast_hits.base, &.{}, t.tracking));
            try std.testing.expect(cast_failing.has_induced_failure);
            try std.testing.expect(cast_hits.hits.items.len < DeepTreeTest.num_bodies);

            var ray_failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index });
            tree.allocator = ray_failing.allocator();
            var ray_hits = AllHits(RayCastBodyCollector).init(allocator);
            defer ray_hits.deinit();
            try std.testing.expectError(error.OutOfMemory, tree.castRay(&ray, &ray_hits.base, &.{}, t.tracking));
            try std.testing.expect(ray_failing.has_induced_failure);
            tree.allocator = allocator;
        }
    }

    // A full rebuild in which every allocation fails in turn (the node IDs, the growing node stack while nodes are being
    // batched to be freed, BuildTree's centers): error.OutOfMemory and the tree is unchanged
    var num_allocations: usize = 0;
    var num_failures: usize = 0;
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var tree: QuadTree = .empty;
        tree.init(&t.node_allocator, allocator);
        defer tree.deinit();
        t.build(&tree);
        tree.tree_state.is_dirty.store(true, .monotonic);
        const before = try DeepTreeTest.dump(allocator, &tree);
        defer allocator.free(before);

        var failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = fail_index });
        tree.allocator = failing.allocator();
        const result = tree.updatePrepare(bodies, t.tracking, true);
        tree.allocator = allocator;
        if (result) |state| {
            tree.updateFinalize(bodies, t.tracking, state);
            tree.discardOldTree();
            if (!failing.has_induced_failure) {
                num_allocations = failing.alloc_index;
                try std.testing.expect(tree.getMaxTreeDepth(tree.getCurrentRoot().getNodeID()) < 10);
                break;
            }
            // Otherwise the debug validation of the tree failed to allocate and was skipped
        } else |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            num_failures += 1;
            const after = try DeepTreeTest.dump(allocator, &tree);
            defer allocator.free(after);
            try std.testing.expectEqualSlices(u32, before, after);
            try std.testing.expect(tree.canBeUpdated());
            if (builtin.mode == .Debug) tree.validateTree(bodies, t.tracking, tree.getCurrentRoot().index.load(.monotonic), DeepTreeTest.num_bodies);
        }
    }
    try std.testing.expect(num_failures >= 3); // The node IDs, at least one growth of the node stack, the centers
    try std.testing.expect(num_allocations >= num_failures);
}
