//! Port of: Jolt/Physics/Collision/Shape/StaticCompoundShape.h, Jolt/Physics/Collision/Shape/StaticCompoundShape.cpp
//! Status: complete
//!
//! A concrete compound shape (Docs/Zolt/CollisionArchitecture.md, section 2 "Compound shapes"):
//! - `StaticCompoundShapeSettings` / `StaticCompoundShape` embed `CompoundShapeSettings` / `CompoundShape` as `base`
//!   (D1); `overrides` lists every C++ `override` of the header in header order. The functions that StaticCompoundShape
//!   does not override come from `CompoundShape.impl` (and `Shape.impl`).
//! - D3: `StaticCompoundShapeSettings::Create()` has custom logic (no sub shape: an error, a single sub shape: the shape
//!   itself or a RotatedTranslatedShape, otherwise a StaticCompoundShape). `Create(TempAllocator &)` is
//!   `createShapeWithTempAllocator(allocator, temp_allocator)`; `createShape` runs it with a TempAllocatorMalloc over
//!   `allocator`, as Jolt does. The shapes are constructed in place on the heap and write the settings' cached result
//!   (`new RotatedTranslatedShape(settings, mCachedResult)`, `new StaticCompoundShape(*this, inTempAllocator,
//!   mCachedResult)`), out of memory is returned and not cached.
//! - The constructor that takes the settings is `initFromSettings(self, settings, temp_allocator, result, allocator)`:
//!   Jolt's (settings, temp allocator, result) order plus the allocator that creates the child shapes. The tree
//!   (`mNodes`) is allocated with the shape's allocator (`base.base.allocator`) and freed in `destruct`; the temporary
//!   arrays of the build (bounds, body indices, the build stack) come from the TempAllocator, freed in reverse order like
//!   Jolt's JPH_SCOPE_EXIT.
//! - `WalkTree` takes the visitor as `anytype` (porting guide section 6). Jolt's visitors (`struct Visitor : public
//!   CastRayVisitor { ShouldVisitNode, VisitNodes, mDistanceStack }`) embed the visitor of CompoundShapeVisitors.zig as
//!   `base` and forward ShouldAbort / VisitShape. The functions repeat one of two patterns in Jolt, so they are built by
//!   two helpers: `DistanceVisitor` (ray casts and shape casts: sort the children by distance with SortReverseAndStore and
//!   skip the ones that are further away than the early out fraction) and `OverlapVisitor` (collide point, collect
//!   transformed shapes, collide shape: visit the children that overlap with CountAndSortTrues).
//!   `GetIntersectingSubShapesVisitorSC<BoxType>` (a named private template in Jolt) is the comptime function
//!   `GetIntersectingSubShapesVisitorSC(BoxType)`.
//! - The private `Node` struct, the `IS_SUBSHAPE` / `INVALID_NODE` bits (`is_subshape` / `invalid_node`) and
//!   `cStackSize` (`stack_size`) are pub so that tests can read the tree.
//! - `sPartition(ioBodyIdx, ioBounds, inNumber, outMidPoint)` is `partition(body_idx, bounds) i32` (the slices have
//!   inNumber elements, the mid point is returned) and `sPartition4(..., outSplit)` is `partition4(body_idx, bounds,
//!   begin, end, out_split: *[5]i32)`. The helpers registered in CollisionDispatch drop the `s`
//!   (`collideCompoundVsShape`, `collideShapeVsCompound`, `castShapeVsCompound`).
//! - The assert in sPartition (`start == end`) can be violated by NaN bounds, after which Jolt's release build
//!   continues: guarded with `Core.enable_asserts` (porting guide section 7).
//! - JPH_DEBUG_RENDERER: StaticCompoundShape has no draw code of its own (CompoundShape's Draw is TODO(debug_renderer)).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Color = @import("../../../Core/Color.zig").Color;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const TempAllocatorFile = @import("../../../Core/TempAllocator.zig");
const TempAllocator = TempAllocatorFile.TempAllocator;
const TempAllocatorMalloc = TempAllocatorFile.TempAllocatorMalloc;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const half_float = @import("../../../Math/HalfFloat.zig");
const HalfFloat = half_float.HalfFloat;
const math = @import("../../../Math/Math.zig");
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const ShapeFile = @import("Shape.zig");
const Shape = ShapeFile.Shape;
const ShapeFunctions = ShapeFile.ShapeFunctions;
const ShapeResult = ShapeFile.ShapeResult;
const ShapeSettings = ShapeFile.ShapeSettings;
const ShapeSubType = ShapeFile.ShapeSubType;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const CompoundShapeFile = @import("CompoundShape.zig");
const CompoundShape = CompoundShapeFile.CompoundShape;
const CompoundShapeSettings = CompoundShapeFile.CompoundShapeSettings;
const SubShape = CompoundShape.SubShape;
const RotatedTranslatedShapeFile = @import("RotatedTranslatedShape.zig");
const RotatedTranslatedShape = RotatedTranslatedShapeFile.RotatedTranslatedShape;
const RotatedTranslatedShapeSettings = RotatedTranslatedShapeFile.RotatedTranslatedShapeSettings;
const SubShapeIDFile = @import("SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const RayCastFile = @import("../RayCast.zig");
const RayCast = RayCastFile.RayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const RayCastResult = @import("../CastResult.zig").RayCastResult;
const CollideShapeSettings = @import("../CollideShape.zig").CollideShapeSettings;
const ShapeCastFile = @import("../ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeFilter = @import("../ShapeFilter.zig").ShapeFilter;
const SortReverseAndStore = @import("../SortReverseAndStore.zig");
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Class that constructs a StaticCompoundShape. Note that if you only want a compound of 1 shape, use a RotatedTranslatedShape instead.
pub const StaticCompoundShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, StaticCompoundShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: CompoundShapeSettings,

    /// Constructor. Use addShape / addShapePtr (`base.addShape(...)`) to add the parts.
    pub fn init(allocator: Allocator) StaticCompoundShapeSettings {
        return .{ .base = .init(StaticCompoundShapeSettings, allocator) };
    }

    /// new StaticCompoundShapeSettings: reference count 0, freed by the last release()
    pub fn create(allocator: Allocator) Allocator.Error!*StaticCompoundShapeSettings {
        const self = try allocator.create(StaticCompoundShapeSettings);
        self.* = .init(allocator);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *StaticCompoundShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *StaticCompoundShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    /// Create a shape according to the settings specified by this object.
    /// Note when there is only 1 sub shape, this will create a RotatedTranslatedShape, or when the sub shape has no rotation/translation, it will return the child shape itself.
    pub fn createShape(self: *StaticCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        var temp_allocator = TempAllocatorMalloc.init(allocator);
        return self.createShapeWithTempAllocator(allocator, temp_allocator.tempAllocator());
    }

    /// Specialization of Create() function that allows specifying a temp allocator to avoid temporary memory allocations on the heap
    pub fn createShapeWithTempAllocator(self: *StaticCompoundShapeSettings, allocator: Allocator, temp_allocator: TempAllocator) Allocator.Error!ShapeResult {
        const cached_result = &self.base.base.cached_result;
        if (cached_result.isEmpty()) {
            errdefer cached_result.clear(); // Out of memory is not cached, a later call can succeed

            if (self.base.sub_shapes.items.len == 0) {
                // It's an error to create a compound with no subshapes (the compound cannot encode this)
                cached_result.setError("Compound needs a sub shape!");
            } else if (self.base.sub_shapes.items.len == 1) {
                // If there's only 1 part we don't need a StaticCompoundShape
                const s = &self.base.sub_shapes.items[0];
                if (s.position.eql(Vec3.zero()) and s.rotation.eql(Quat.identity())) {
                    // No rotation or translation, we can use the shape directly
                    if (s.shape_ptr.get()) |shape_ptr|
                        cached_result.set(.init(@constCast(shape_ptr))) // Jolt's const_cast: only the reference count is written through it
                    else if (s.shape.get()) |shape|
                        cached_result.assignMove(try shape.createShape(allocator))
                    else
                        cached_result.setError("Sub shape is null!");
                } else {
                    // We can use a RotatedTranslatedShape instead
                    var settings = RotatedTranslatedShapeSettings.initDefault(allocator);
                    defer settings.deinit();
                    settings.position = s.position;
                    settings.rotation = s.rotation;
                    settings.base.inner_shape.set(s.shape.get());
                    settings.base.inner_shape_ptr.set(s.shape_ptr.get());

                    // Ref<Shape> shape = new RotatedTranslatedShape(settings, mCachedResult);
                    const shape = try allocator.create(RotatedTranslatedShape);
                    shape.* = .initDefault(allocator);
                    var ref = Ref(Shape).init(shape.asShapeMut());
                    defer ref.deinit();
                    try shape.initFromSettings(&settings, cached_result, allocator);
                }
            } else {
                // Build a regular compound shape
                // Ref<Shape> shape = new StaticCompoundShape(*this, inTempAllocator, mCachedResult);
                const shape = try allocator.create(StaticCompoundShape);
                shape.* = .initDefault(allocator);
                var ref = Ref(Shape).init(shape.asShapeMut());
                defer ref.deinit();
                try shape.initFromSettings(self, temp_allocator, cached_result, allocator);
            }
        }
        return cached_result.clone();
    }
};

/// A compound shape, sub shapes can be rotated and translated.
/// Sub shapes cannot be modified once the shape is constructed.
/// Shifts all child objects so that they're centered around the center of mass.
pub const StaticCompoundShape = struct {
    /// Concrete class: `Shape.cast(StaticCompoundShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .static_compound;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .getIntersectingSubShapes, .getIntersectingSubShapesOrientedBox, .saveBinaryState, .getStats, .restoreBinaryState };

    /// Maximum size of the stack during tree walk
    pub const stack_size = 128;

    /// Bits used in Node::mNodeProperties: if this bit is set, the other bits index in mSubShape, otherwise in mNodes
    pub const is_subshape: u32 = 0x80000000;
    /// Bits used in Node::mNodeProperties: signifies an invalid node
    pub const invalid_node: u32 = 0x7fffffff;

    /// Node structure (private in Jolt)
    pub const Node = extern struct {
        /// 4 child bounding boxes
        bounds_min_x: [4]HalfFloat,
        bounds_min_y: [4]HalfFloat,
        bounds_min_z: [4]HalfFloat,
        bounds_max_x: [4]HalfFloat,
        bounds_max_y: [4]HalfFloat,
        bounds_max_z: [4]HalfFloat,
        /// 4 child node properties
        node_properties: [4]u32,

        comptime {
            std.debug.assert(@sizeOf(Node) == 64); // Node should be 64 bytes
        }

        /// Set bounding box for child index to bounds
        pub fn setChildBounds(self: *Node, index: u32, bounds: AABox) void {
            self.bounds_min_x[index] = half_float.fromFloat(.round_to_neg_inf, bounds.min.getX());
            self.bounds_min_y[index] = half_float.fromFloat(.round_to_neg_inf, bounds.min.getY());
            self.bounds_min_z[index] = half_float.fromFloat(.round_to_neg_inf, bounds.min.getZ());
            self.bounds_max_x[index] = half_float.fromFloat(.round_to_pos_inf, bounds.max.getX());
            self.bounds_max_y[index] = half_float.fromFloat(.round_to_pos_inf, bounds.max.getY());
            self.bounds_max_z[index] = half_float.fromFloat(.round_to_pos_inf, bounds.max.getZ());
        }

        /// Mark the child index as invalid and set its bounding box to invalid
        pub fn setChildInvalid(self: *Node, index: u32) void {
            // Make this an invalid node
            self.node_properties[index] = invalid_node;

            // Make bounding box invalid
            self.bounds_min_x[index] = half_float.half_flt_max;
            self.bounds_min_y[index] = half_float.half_flt_max;
            self.bounds_min_z[index] = half_float.half_flt_max;
            self.bounds_max_x[index] = half_float.half_flt_max;
            self.bounds_max_y[index] = half_float.half_flt_max;
            self.bounds_max_z[index] = half_float.half_flt_max;
        }
    };

    pub const Nodes = std.ArrayList(Node);

    base: CompoundShape,
    /// Quad tree node structure (allocated with base.base.allocator)
    nodes: Nodes = .empty,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// StaticCompoundShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct)
    pub fn initDefault(allocator: Allocator) StaticCompoundShape {
        return .{ .base = .init(StaticCompoundShape, allocator, shape_sub_type) };
    }

    /// new StaticCompoundShape (the default constructor, an empty shape for restoreBinaryState): reference count 0
    pub fn create(allocator: Allocator) Allocator.Error!*StaticCompoundShape {
        const self = try allocator.create(StaticCompoundShape);
        self.* = .initDefault(allocator);
        return self;
    }

    /// StaticCompoundShape(const StaticCompoundShapeSettings &inSettings, TempAllocator &inTempAllocator, ShapeResult &outResult):
    /// base part first, then the C++ body. `allocator` creates the child shapes from their settings.
    pub fn initFromSettings(self: *StaticCompoundShape, settings: *const StaticCompoundShapeSettings, temp_allocator: TempAllocator, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base.base, result);
        const c = &self.base;
        const shape_allocator = c.base.allocator;

        // Check that there's at least 1 shape
        const num_subshapes: u32 = @intCast(settings.base.sub_shapes.items.len);
        if (num_subshapes < 2) {
            result.setError("Compound needs at least 2 sub shapes, otherwise you should use a RotatedTranslatedShape!");
            return;
        }

        // Keep track of total mass to calculate center of mass
        var mass: f32 = 0.0;

        try c.sub_shapes.appendNTimes(shape_allocator, .{}, num_subshapes);
        for (settings.base.sub_shapes.items, c.sub_shapes.items) |*shape, *out_shape| {
            // Start constructing the runtime sub shape
            if (!try out_shape.fromSettings(shape, result, allocator))
                return;

            // Calculate mass properties of child
            const child = out_shape.shape.get().?.getMassProperties();

            // Accumulate center of mass
            mass += child.mass;
            c.center_of_mass = c.center_of_mass.add(out_shape.getPositionCOM().mulScalar(child.mass));
        }

        if (mass > 0.0)
            c.center_of_mass = c.center_of_mass.divScalar(mass);

        // Cache the inner radius as it can take a while to recursively iterate over all sub shapes
        c.calculateInnerRadius();

        // Temporary storage for the bounding boxes of all shapes
        const bounds_size: u32 = num_subshapes * @sizeOf(AABox);
        const bounds_block = try temp_allocator.allocate(bounds_size);
        defer temp_allocator.free(bounds_block, bounds_size);
        const bounds = @as([*]AABox, @ptrCast(bounds_block.?))[0..num_subshapes];

        // Temporary storage for body indexes (we're shuffling them)
        const body_idx_size: u32 = num_subshapes * @sizeOf(u32);
        const body_idx_block = try temp_allocator.allocate(body_idx_size);
        defer temp_allocator.free(body_idx_block, body_idx_size);
        const body_idx = @as([*]u32, @ptrCast(body_idx_block.?))[0..num_subshapes];

        // Shift all shapes so that the center of mass is now at the origin and calculate bounds
        for (c.sub_shapes.items, 0..) |*shape, i| {
            // Shift the shape so it's centered around our center of mass
            shape.setPositionCOM(shape.getPositionCOM().sub(c.center_of_mass));

            // Transform the shape's bounds into our local space
            const transform = Mat44.rotationTranslation(shape.getRotation(), shape.getPositionCOM());
            const shape_bounds = shape.shape.get().?.getWorldSpaceBounds(transform, Vec3.one());

            // Store bounds and body index for tree construction
            bounds[i] = shape_bounds;
            body_idx[i] = @intCast(i);

            // Update our local bounds
            c.local_bounds.encapsulate(shape_bounds);
        }

        // The algorithm is a recursive tree build, but to avoid the call overhead we keep track of a stack here
        const StackEntry = struct {
            /// Node index of node that is generated
            node_idx: u32,
            /// Index of child that we're currently processing
            child_idx: i32,
            /// Indices where the node ID's have been split to form 4 partitions
            split: [5]i32,
            /// Bounding box of this node
            bounds: AABox,
        };
        const stack_entries_size: u32 = num_subshapes * @sizeOf(StackEntry);
        const stack_block = try temp_allocator.allocate(stack_entries_size);
        defer temp_allocator.free(stack_block, stack_entries_size);
        const stack = @as([*]StackEntry, @ptrCast(stack_block.?))[0..num_subshapes];
        var top: u32 = 0;

        // Reserve enough space so that every sub shape gets its own leaf node
        var next_node_idx: u32 = 0;
        try self.nodes.resize(shape_allocator, num_subshapes + (num_subshapes + 2) / 3); // = Sum(num_subshapes * 4^-i) with i = [0, Inf].

        // Create root node
        stack[0].node_idx = next_node_idx;
        next_node_idx += 1;
        stack[0].child_idx = -1;
        stack[0].bounds = .empty;
        partition4(body_idx, bounds, 0, @intCast(num_subshapes), &stack[0].split);

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
                prev_stack.bounds.encapsulate(cur_stack.bounds);

                // Store this node's properties in the parent node
                const parent_node = &self.nodes.items[prev_stack.node_idx];
                parent_node.node_properties[@intCast(prev_stack.child_idx)] = cur_stack.node_idx;
                parent_node.setChildBounds(@intCast(prev_stack.child_idx), cur_stack.bounds);

                // Pop entry from stack
                top -= 1;
            } else {
                // Get low and high index to bodies to process
                const low = cur_stack.split[@intCast(cur_stack.child_idx)];
                const high = cur_stack.split[@intCast(cur_stack.child_idx + 1)];
                const num_bodies = high - low;

                if (num_bodies == 0) {
                    // Mark invalid
                    const node = &self.nodes.items[cur_stack.node_idx];
                    node.setChildInvalid(@intCast(cur_stack.child_idx));
                } else if (num_bodies == 1) {
                    // Get body info
                    const child_node_idx = body_idx[@intCast(low)];
                    const child_bounds = &bounds[@intCast(low)];

                    // Update node
                    const node = &self.nodes.items[cur_stack.node_idx];
                    node.node_properties[@intCast(cur_stack.child_idx)] = child_node_idx | is_subshape;
                    node.setChildBounds(@intCast(cur_stack.child_idx), child_bounds.*);

                    // Encapsulate bounding box in parent
                    cur_stack.bounds.encapsulate(child_bounds.*);
                } else {
                    // Allocate new node
                    top += 1;
                    const new_stack = &stack[top];
                    std.debug.assert(top < num_subshapes);
                    new_stack.node_idx = next_node_idx;
                    next_node_idx += 1;
                    new_stack.child_idx = -1;
                    new_stack.bounds = .empty;
                    partition4(body_idx, bounds, low, high, &new_stack.split);
                }
            }
        }

        // Resize nodes to actual size
        std.debug.assert(next_node_idx <= self.nodes.items.len);
        self.nodes.shrinkAndFree(shape_allocator, next_node_idx);

        // Check if we ran out of bits for addressing a node
        if (next_node_idx > is_subshape) {
            result.setError("Compound hierarchy has too many nodes");
            return;
        }

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.asShape().getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// ~StaticCompoundShape
    pub fn destruct(self: *StaticCompoundShape) void {
        self.nodes.deinit(self.base.base.allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const StaticCompoundShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *StaticCompoundShape) *Shape {
        return &self.base.base;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Tree construction

    /// Sorts body_idx spatially into 2 groups. Second groups starts at body_idx + the returned mid point.
    /// After the function returns body_idx and bounds will be shuffled (both slices have Jolt's inNumber elements)
    pub fn partition(body_idx: []u32, bounds: []AABox) i32 {
        const number: i32 = @intCast(bounds.len);
        std.debug.assert(body_idx.len == bounds.len);

        // Handle trivial case
        if (number <= 4)
            return @divTrunc(number, 2);

        // Calculate bounding box of box centers
        var center_min = Vec3.replicate(math.flt_max);
        var center_max = Vec3.replicate(-math.flt_max);
        for (bounds) |*b| {
            const center = b.getCenter();
            center_min = Vec3.min(center_min, center);
            center_max = Vec3.max(center_max, center);
        }

        // Calculate split plane
        const dimension = center_max.sub(center_min).getHighestComponentIndex();
        const split = 0.5 * center_min.add(center_max).getComponent(dimension);

        // Divide bodies
        var start: i32 = 0;
        var end: i32 = number;
        while (start < end) {
            // Search for first element that is on the right hand side of the split plane
            while (start < end and bounds[@intCast(start)].getCenter().getComponent(dimension) < split)
                start += 1;

            // Search for the first element that is on the left hand side of the split plane
            while (start < end and bounds[@intCast(end - 1)].getCenter().getComponent(dimension) >= split)
                end -= 1;

            if (start < end) {
                // Swap the two elements
                std.mem.swap(u32, &body_idx[@intCast(start)], &body_idx[@intCast(end - 1)]);
                std.mem.swap(AABox, &bounds[@intCast(start)], &bounds[@intCast(end - 1)]);
                start += 1;
                end -= 1;
            }
        }
        if (Core.enable_asserts) std.debug.assert(start == end); // NaN bounds can violate this, Jolt's release build continues

        if (start > 0 and start < number) {
            // Success!
            return start;
        } else {
            // Failed to divide bodies
            return @divTrunc(number, 2);
        }
    }

    /// Sorts body_idx from begin to (but excluding) end spatially into 4 groups.
    /// out_split needs to be 5 ints long, when the function returns each group runs from out_split[i] to (but excluding) out_split[i + 1]
    /// After the function returns body_idx and bounds will be shuffled
    pub fn partition4(body_idx_in: []u32, bounds_in: []AABox, begin: i32, end: i32, out_split: *[5]i32) void {
        const body_idx = body_idx_in[@intCast(begin)..@intCast(end)];
        const node_bounds = bounds_in[@intCast(begin)..@intCast(end)];
        const number = end - begin;

        // Partition entire range
        out_split[2] = partition(body_idx, node_bounds);

        // Partition lower half
        out_split[1] = partition(body_idx[0..@intCast(out_split[2])], node_bounds[0..@intCast(out_split[2])]);

        // Partition upper half
        out_split[3] = partition(body_idx[@intCast(out_split[2])..@intCast(number)], node_bounds[@intCast(out_split[2])..@intCast(number)]);

        // Convert to proper range
        out_split[0] = begin;
        out_split[1] += begin;
        out_split[2] += begin;
        out_split[3] += out_split[2];
        out_split[4] = end;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Tree walk

    /// Walk the node tree calling the Visitor::VisitNodes for each node encountered and Visitor::VisitShape for each sub shape encountered
    /// (`visitor` is a pointer to a visitor with shouldAbort, shouldVisitNode, visitNodes and visitShape)
    pub fn walkTree(self: *const StaticCompoundShape, visitor: anytype) void {
        var node_stack: [stack_size]u32 = undefined;
        node_stack[0] = 0;
        var top: i32 = 0;
        while (true) {
            // Test if the node is valid, the node should rarely be invalid but it is possible when testing
            // a really large box against the tree that the invalid nodes will intersect with the box
            const node_properties = node_stack[@intCast(top)];
            if (node_properties != invalid_node) {
                // Test if node contains triangles
                const is_node = (node_properties & is_subshape) == 0;
                if (is_node) {
                    const node = &self.nodes.items[node_properties];
                    const node_words: *const [16]u32 = @ptrCast(node); // reinterpret_cast<const uint32 *>(&node.mBoundsMinX[0]), ...

                    // Unpack bounds
                    const bounds_minxy = UVec4.loadInt4(node_words[0..4]);
                    const bounds_minx = half_float.toFloat(bounds_minxy);
                    const bounds_miny = half_float.toFloat(bounds_minxy.swizzle(.z, .w, .unused, .unused));

                    const bounds_minzmaxx = UVec4.loadInt4(node_words[4..8]);
                    const bounds_minz = half_float.toFloat(bounds_minzmaxx);
                    const bounds_maxx = half_float.toFloat(bounds_minzmaxx.swizzle(.z, .w, .unused, .unused));

                    const bounds_maxyz = UVec4.loadInt4(node_words[8..12]);
                    const bounds_maxy = half_float.toFloat(bounds_maxyz);
                    const bounds_maxz = half_float.toFloat(bounds_maxyz.swizzle(.z, .w, .unused, .unused));

                    // Load properties for 4 children
                    var properties = UVec4.loadInt4(&node.node_properties);

                    // Check which sub nodes to visit
                    const num_results = visitor.visitNodes(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, &properties, top);

                    // Push them onto the stack
                    std.debug.assert(top + 4 < stack_size);
                    properties.storeInt4(node_stack[@intCast(top)..][0..4]);
                    top += num_results;
                } else {
                    // Points to a sub shape
                    const sub_shape_idx = node_properties ^ is_subshape;
                    const sub_shape = &self.base.sub_shapes.items[sub_shape_idx];

                    visitor.visitShape(sub_shape, sub_shape_idx);
                }

                // Check if we're done
                if (visitor.shouldAbort())
                    break;
            }

            // Fetch next node until we find one that the visitor wants to see
            while (true) {
                top -= 1;
                if (!(top >= 0 and !visitor.shouldVisitNode(top)))
                    break;
            }
            if (!(top >= 0))
                break;
        }
    }

    /// The visitors of the ray and shape casts (Jolt's `struct Visitor : public CastRayVisitor` etc. with an
    /// `mDistanceStack`): `fraction(&base)` is the early out fraction the visitor compares with (mHit.mFraction,
    /// mCollector.GetEarlyOutFraction() or mCollector.GetPositiveEarlyOutFraction())
    fn DistanceVisitor(comptime Base: type, comptime fraction: fn (*const Base) f32) type {
        return struct {
            const Self = @This();

            base: Base,
            distance_stack: [stack_size]f32 = undefined,

            pub fn shouldAbort(self: *const Self) bool {
                return self.base.shouldAbort();
            }

            pub fn visitShape(self: *Self, sub_shape: *const SubShape, sub_shape_index: u32) void {
                self.base.visitShape(sub_shape, sub_shape_index);
            }

            pub fn shouldVisitNode(self: *const Self, stack_top: i32) bool {
                return self.distance_stack[@intCast(stack_top)] < fraction(&self.base);
            }

            pub fn visitNodes(self: *Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                // Test bounds of 4 children
                const distance = self.base.testBounds(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

                // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
                return SortReverseAndStore.sortReverseAndStore(distance, fraction(&self.base), properties, self.distance_stack[@intCast(stack_top)..][0..4]);
            }
        };
    }

    /// The visitors that visit every child that overlaps (Jolt's `struct Visitor : public CollidePointVisitor` etc.
    /// whose ShouldVisitNode returns true)
    fn OverlapVisitor(comptime Base: type) type {
        return struct {
            const Self = @This();

            base: Base,

            pub fn shouldAbort(self: *const Self) bool {
                return self.base.shouldAbort();
            }

            pub fn visitShape(self: *Self, sub_shape: *const SubShape, sub_shape_index: u32) void {
                self.base.visitShape(sub_shape, sub_shape_index);
            }

            pub fn shouldVisitNode(self: *const Self, stack_top: i32) bool {
                _ = .{ self, stack_top };
                return true;
            }

            pub fn visitNodes(self: *const Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
                _ = stack_top;

                // Test which nodes collide
                const collides = self.base.testBounds(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
                return SortReverseAndStore.countAndSortTrues(collides, properties);
            }
        };
    }

    /// Visitor for GetIntersectingSubShapes
    pub fn GetIntersectingSubShapesVisitorSC(comptime BoxType: type) type {
        return OverlapVisitor(CompoundShape.GetIntersectingSubShapesVisitor(BoxType));
    }

    fn hitFraction(visitor: *const CompoundShape.CastRayVisitor) f32 {
        return visitor.hit.fraction;
    }

    fn earlyOutFraction(visitor: *const CompoundShape.CastRayVisitorCollector) f32 {
        return visitor.collector.getEarlyOutFraction();
    }

    fn positiveEarlyOutFraction(visitor: *const CompoundShape.CastShapeVisitor) f32 {
        return visitor.collector.getPositiveEarlyOutFraction();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Overrides of virtual functions (listed in `overrides`, signatures checked at compile time)

    // See Shape::CastRay
    pub fn castRay(self: *const StaticCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const Visitor = DistanceVisitor(CompoundShape.CastRayVisitor, hitFraction);
        var visitor: Visitor = .{ .base = .init(&ray, &self.base, sub_shape_id_creator, hit) };
        self.walkTree(&visitor);
        return visitor.base.return_value;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const StaticCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = DistanceVisitor(CompoundShape.CastRayVisitorCollector, earlyOutFraction);
        var visitor: Visitor = .{ .base = .init(&ray, ray_cast_settings, &self.base, sub_shape_id_creator, collector, shape_filter) };
        self.walkTree(&visitor);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const StaticCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        const Visitor = OverlapVisitor(CompoundShape.CollidePointVisitor);
        var visitor: Visitor = .{ .base = .init(point, &self.base, sub_shape_id_creator, collector, shape_filter) };
        self.walkTree(&visitor);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const StaticCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = OverlapVisitor(CompoundShape.CollectTransformedShapesVisitor);
        var visitor: Visitor = .{ .base = .init(box, &self.base, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter) };
        self.walkTree(&visitor);
    }

    // See: CompoundShape::GetIntersectingSubShapes
    pub fn getIntersectingSubShapes(self: *const StaticCompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        var visitor: GetIntersectingSubShapesVisitorSC(AABox) = .{ .base = .init(box, out_sub_shape_indices) };
        self.walkTree(&visitor);
        return visitor.base.getNumResults();
    }

    // See: CompoundShape::GetIntersectingSubShapes
    pub fn getIntersectingSubShapesOrientedBox(self: *const StaticCompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        var visitor: GetIntersectingSubShapesVisitorSC(OrientedBox) = .{ .base = .init(box, out_sub_shape_indices) };
        self.walkTree(&visitor);
        return visitor.base.getNumResults();
    }

    // See Shape: C++ `CompoundShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const StaticCompoundShape, stream: StreamOut) void {
        CompoundShape.impl.saveBinaryState(&self.base, stream);

        stream.writeArray(Node, self.nodes.items);
    }

    // See Shape::GetStats
    pub fn getStats(self: *const StaticCompoundShape) Shape.Stats {
        return .init(@sizeOf(StaticCompoundShape) + self.base.sub_shapes.items.len * @sizeOf(SubShape) + self.nodes.items.len * @sizeOf(Node), 0);
    }

    // See: Shape::RestoreBinaryState: C++ `CompoundShape::RestoreBinaryState(inStream)` is the explicit base call
    pub fn restoreBinaryState(self: *StaticCompoundShape, stream: StreamIn) Allocator.Error!void {
        try CompoundShape.impl.restoreBinaryState(&self.base, stream);

        try stream.readArray(Node, self.base.base.allocator, &self.nodes);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.static_compound);
        f.construct = ShapeFunctions.constructor(StaticCompoundShape);
        f.color = Color.orange;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.static_compound, s, collideCompoundVsShape);
            r.registerCollideShape(s, .static_compound, collideShapeVsCompound);
            r.registerCastShape(s, .static_compound, castShapeVsCompound);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn castShapeVsCompound(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(StaticCompoundShape); // Asserts that the sub type is EShapeSubType::StaticCompound

        const Visitor = DistanceVisitor(CompoundShape.CastShapeVisitor, positiveEarlyOutFraction);
        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, &shape.base, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector) };
        shape.walkTree(&visitor);
    }

    fn collideCompoundVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(StaticCompoundShape); // Asserts that the sub type is EShapeSubType::StaticCompound

        const Visitor = OverlapVisitor(CompoundShape.CollideCompoundVsShapeVisitor);
        var visitor: Visitor = .{ .base = .init(&shape1.base, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter) };
        shape1.walkTree(&visitor);
    }

    fn collideShapeVsCompound(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(StaticCompoundShape); // Asserts that the sub type is EShapeSubType::StaticCompound

        const Visitor = OverlapVisitor(CompoundShape.CollideShapeVsCompoundVisitor);
        var visitor: Visitor = .{ .base = .init(shape1, &shape2.base, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter) };
        shape2.walkTree(&visitor);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/CompoundsParity.zig)

const testing = std.testing;
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const RefCount = @import("../../../Core/Reference.zig").RefCount;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const TempAllocatorImpl = TempAllocatorFile.TempAllocatorImpl;
const RegisterTypes = @import("../../../RegisterTypes.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const AnyHitCollisionCollector = CollisionCollectorImpl.AnyHitCollisionCollector;
const SphereShapeFile = @import("SphereShape.zig");
const SphereShape = SphereShapeFile.SphereShape;
const SphereShapeSettings = SphereShapeFile.SphereShapeSettings;
const BoxShape = @import("BoxShape.zig").BoxShape;
const ScaledShape = @import("ScaledShape.zig").ScaledShape;
const MutableCompoundShapeFile = @import("MutableCompoundShape.zig");
const MutableCompoundShape = MutableCompoundShapeFile.MutableCompoundShape;
const MutableCompoundShapeSettings = MutableCompoundShapeFile.MutableCompoundShapeSettings;
const TransformedShape = @import("../TransformedShape.zig").TransformedShape;
const CollidePointResult = @import("../CollidePointResult.zig").CollidePointResult;
const CollideShapeResult = @import("../CollideShape.zig").CollideShapeResult;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const TestBoxShape = @import("TestShapes.zig").TestBoxShape;

fn saveToBuffer(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!ShapeResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return Shape.restoreFromBinaryState(allocator, in.streamIn());
}

/// Children for the test compounds: two spheres and two boxes (embedded, the caller deinits them)
const TestChildren = struct {
    sphere1: SphereShape,
    sphere2: SphereShape,
    box1: BoxShape,
    box2: BoxShape,

    fn init(self: *TestChildren, allocator: Allocator) void {
        self.sphere1 = .init(allocator, 0.5, .{});
        self.sphere2 = .init(allocator, 1.25, .{});
        self.box1 = .init(allocator, Vec3.init(0.5, 1.0, 1.5), .{});
        self.box2 = .init(allocator, Vec3.init(2.0, 0.25, 0.75), .{ .convex_radius = 0.1 });
        for (self.shapes()) |s| s.setEmbedded();
    }

    fn shapes(self: *const TestChildren) [4]*const Shape {
        return .{ self.sphere1.asShape(), self.sphere2.asShape(), self.box1.asShape(), self.box2.asShape() };
    }

    fn deinit(self: *TestChildren) void {
        self.sphere1.asShapeMut().deinit();
        self.sphere2.asShapeMut().deinit();
        self.box1.asShapeMut().deinit();
        self.box2.asShapeMut().deinit();
    }
};

/// Add `n` sub shapes at random positions (spread over `extent`) with random rotations (some identity) to `settings`
fn addRandomSubShapes(settings: *CompoundShapeSettings, children: *const TestChildren, n: usize, extent: f32, seed: u64) Allocator.Error!void {
    var prng = std.Random.DefaultPrng.init(seed);
    const random = prng.random();
    const shapes = children.shapes();
    for (0..n) |i| {
        const position = Vec3.init(random.float(f32) * 2.0 - 1.0, random.float(f32) * 2.0 - 1.0, random.float(f32) * 2.0 - 1.0).mulScalar(extent);
        const rotation = if (i % 3 == 0) Quat.identity() else Quat.rotation(Vec3.init(random.float(f32) + 0.1, random.float(f32), random.float(f32) - 0.5).normalized(), random.float(f32) * 6.0);
        try settings.addShapePtr(position, rotation, shapes[i % shapes.len], .{ .user_data = @intCast(i) });
    }
}

/// The value of a half float
fn halfToFloat(h: HalfFloat) f32 {
    return half_float.toFloat(UVec4.init(h, 0, 0, 0)).getX();
}

/// The bounds of child `i` of `node` as floats
fn childBounds(node: *const StaticCompoundShape.Node, i: usize) AABox {
    return .init(
        Vec3.init(halfToFloat(node.bounds_min_x[i]), halfToFloat(node.bounds_min_y[i]), halfToFloat(node.bounds_min_z[i])),
        Vec3.init(halfToFloat(node.bounds_max_x[i]), halfToFloat(node.bounds_max_y[i]), halfToFloat(node.bounds_max_z[i])),
    );
}

/// The bounds of a sub shape in the space of the compound
fn subShapeBounds(sub_shape: *const SubShape) AABox {
    return sub_shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM()), Vec3.one());
}

/// Check node `node_idx` and its children: every child is invalid, a sub shape (marked in `seen`) or a node, and the
/// half float bounds of a child contain the bounds of everything below it. Returns the exact bounds of the node.
fn checkNode(shape: *const StaticCompoundShape, node_idx: u32, seen: []bool) !AABox {
    const node = &shape.nodes.items[node_idx];
    var bounds: AABox = .empty;
    var num_valid: u32 = 0;
    for (0..4) |i| {
        const properties = node.node_properties[i];
        if (properties == StaticCompoundShape.invalid_node) {
            try testing.expect(node.bounds_min_x[i] == half_float.half_flt_max and node.bounds_max_z[i] == half_float.half_flt_max);
            continue;
        }
        num_valid += 1;
        const child = if ((properties & StaticCompoundShape.is_subshape) != 0) blk: {
            const idx = properties ^ StaticCompoundShape.is_subshape;
            try testing.expect(!seen[idx]);
            seen[idx] = true;
            break :blk subShapeBounds(&shape.base.sub_shapes.items[idx]);
        } else blk: {
            try testing.expect(properties > node_idx and properties < shape.nodes.items.len); // Children come after their parent
            break :blk try checkNode(shape, properties, seen);
        };
        try testing.expect(childBounds(node, i).contains(child));
        bounds.encapsulate(child);
    }
    try testing.expect(num_valid >= 2);
    return bounds;
}

/// Check the tree of a static compound: every sub shape is in exactly one leaf, the bounds are conservative
fn checkTree(allocator: Allocator, shape: *const StaticCompoundShape) !void {
    const num_sub_shapes = shape.base.sub_shapes.items.len;
    const seen = try allocator.alloc(bool, num_sub_shapes);
    defer allocator.free(seen);
    @memset(seen, false);
    const bounds = try checkNode(shape, 0, seen);
    for (seen) |s| try testing.expect(s);
    try testing.expect(shape.nodes.items.len <= num_sub_shapes + (num_sub_shapes + 2) / 3);

    // The local bounds start from the (zero) bounds at the center of mass
    var local_bounds: AABox = .init(Vec3.zero(), Vec3.zero());
    local_bounds.encapsulate(bounds);
    try testing.expect(shape.asShape().getLocalBounds().eql(local_bounds));
}

/// Sort hits by their sub shape ID(s) so that hit sets of different walkers can be compared
fn sortByID(comptime T: type, items: []T) void {
    std.mem.sort(T, items, {}, struct {
        fn key(r: *const T) u64 {
            return if (T == CollidePointResult or T == RayCastResult)
                r.sub_shape_id2.value
            else if (T == CollideShapeResult)
                (@as(u64, r.sub_shape_id1.value) << 32) | r.sub_shape_id2.value
            else if (T == ShapeCastResult)
                (@as(u64, r.base.sub_shape_id1.value) << 32) | r.base.sub_shape_id2.value
            else
                r.sub_shape_id_creator.getID().value;
        }
        fn lessThan(_: void, a: T, b: T) bool {
            return key(&a) < key(&b);
        }
    }.lessThan);
}

test "StaticCompoundShapeSettings: Jolt's error texts, the single sub shape shortcuts and cached results" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    // No sub shapes
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Compound needs a sub shape!", result.getError());
    }

    // A single sub shape without a shape: at the origin and translated (then RotatedTranslatedShape reports it)
    for ([_]Vec3{ Vec3.zero(), Vec3.init(1, 0, 0) }, [_][]const u8{ "Sub shape is null!", "Inner shape is null!" }) |position, expected| {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShape(position, Quat.identity(), null, .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings(expected, result.getError());
    }

    // A single shape at the origin: the shape itself (Jolt's const_cast)
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try expect(result.getPtr().? == sphere.asShapeMut());
        try testing.expectEqual(RefCount.embedded + 3, sphere.asShape().getRefCount()); // The settings, the cache and the result
    }

    // A single shape from settings at the origin: the child's result, also its error
    for ([_]f32{ 2.0, -1.0 }) |radius| {
        const child = try SphereShapeSettings.create(allocator, radius, .{});
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShape(Vec3.zero(), Quat.identity(), child.asShapeSettings(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        if (radius > 0.0)
            try expect(result.getPtr().? == child.asShapeSettings().cached_result.getPtr().?)
        else
            try testing.expectEqualStrings("Invalid radius", result.getError());
    }

    // A single translated / rotated shape: a RotatedTranslatedShape (built from settings on the stack, so it does not
    // get the user data of the compound settings, like in Jolt)
    {
        const rotation = Quat.rotation(Vec3.axisZ(), 0.3);
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        settings.asShapeSettings().user_data = 7;
        try settings.base.addShapePtr(Vec3.init(1, 2, 3), rotation, sphere.asShape(), .{ .user_data = 5 });
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const rt = result.getPtr().?.cast(RotatedTranslatedShape);
        try expect(rt.getCenterOfMass().eql(Vec3.init(1, 2, 3)) and rt.getRotation().eql(rotation));
        try expect(rt.base.getInnerShape() == sphere.asShape());
        try testing.expectEqual(@as(u64, 0), rt.asShape().getUserData());

        // Cached: the same shape again
        var again = try settings.asShapeSettings().createShape(allocator);
        defer again.deinit();
        try expect(again.getPtr() == result.getPtr());

        // With child settings that fail: the child's error
        const bad = try SphereShapeSettings.create(allocator, 0.0, .{});
        var bad_settings = StaticCompoundShapeSettings.init(allocator);
        defer bad_settings.deinit();
        try bad_settings.base.addShape(Vec3.zero(), rotation, bad.asShapeSettings(), .{});
        var bad_result = try bad_settings.asShapeSettings().createShape(allocator);
        defer bad_result.deinit();
        try testing.expectEqualStrings("Invalid radius", bad_result.getError());
    }

    // Two sub shapes: a StaticCompoundShape with the user data of the settings; a child error is forwarded
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        settings.asShapeSettings().user_data = 7;
        try settings.base.addShapePtr(Vec3.init(-1, 0, 0), Quat.identity(), sphere.asShape(), .{});
        try settings.base.addShapePtr(Vec3.init(1, 0, 0), Quat.identity(), sphere.asShape(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const shape = result.getPtr().?.cast(StaticCompoundShape);
        try testing.expectEqual(@as(u64, 7), shape.asShape().getUserData());
        try testing.expectEqual(ShapeSubType.static_compound, shape.asShape().getSubType());
        try testing.expectEqual(@as(usize, 1), shape.nodes.items.len);

        const bad = try SphereShapeSettings.create(allocator, -2.0, .{});
        try settings.base.addShape(Vec3.zero(), Quat.identity(), bad.asShapeSettings(), .{});
        settings.asShapeSettings().clearCachedResult();
        var bad_result = try settings.asShapeSettings().createShape(allocator);
        defer bad_result.deinit();
        try testing.expectEqualStrings("Invalid radius", bad_result.getError());
    }

    // The constructor itself needs at least 2 sub shapes
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
        var shape = StaticCompoundShape.initDefault(allocator);
        defer shape.asShapeMut().deinit();
        var result: ShapeResult = .empty;
        defer result.deinit();
        var temp_allocator = TempAllocatorMalloc.init(allocator);
        try shape.initFromSettings(&settings, temp_allocator.tempAllocator(), &result, allocator);
        try testing.expectEqualStrings("Compound needs at least 2 sub shapes, otherwise you should use a RotatedTranslatedShape!", result.getError());
    }

    // Nesting compounds of 2 sub shapes (1 bit each, the previous level and a sphere) 33 deep exceeds the 32 sub shape
    // ID bits
    {
        var inner = RefConst(Shape).init(sphere.asShape());
        defer inner.deinit();
        for (1..34) |level| {
            var settings = StaticCompoundShapeSettings.init(allocator);
            defer settings.deinit();
            try settings.base.addShapePtr(Vec3.init(-1, 0, 0), Quat.identity(), inner.get(), .{});
            try settings.base.addShapePtr(Vec3.init(1, 0, 0), Quat.identity(), sphere.asShape(), .{});
            var result = try settings.asShapeSettings().createShape(allocator);
            defer result.deinit();
            if (level <= 32) {
                try testing.expectEqual(@as(u32, @intCast(level)), result.getPtr().?.getSubShapeIDBitsRecursive());
                inner.set(result.getPtr().?);
            } else try testing.expectEqualStrings("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits", result.getError());
        }
    }
}

test "StaticCompoundShape: the tree, center of mass, bounds, inner radius and mass properties" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var children: TestChildren = undefined;
    children.init(allocator);
    defer children.deinit();

    for ([_]usize{ 2, 3, 4, 5, 7, 16, 17, 64, 100, 257 }, 0..) |n, seed| {
        const extent: f32 = if (n < 10) 3.0 else 30.0;
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try addRandomSubShapes(&settings.base, &children, n, extent, seed);
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const shape = result.getPtr().?.cast(StaticCompoundShape);
        try testing.expectEqual(@as(u32, @intCast(n)), shape.base.getNumSubShapes());
        try checkTree(allocator, shape);

        // Center of mass: the mass weighted average of the child positions, the sub shapes are shifted so that it is at
        // the origin
        var mass: f32 = 0.0;
        var center_of_mass = Vec3.zero();
        var inner_radius: f32 = math.flt_max;
        for (settings.base.sub_shapes.items) |*s| {
            const child = s.shape_ptr.get().?;
            const child_mass = child.getMassProperties().mass;
            mass += child_mass;
            center_of_mass = center_of_mass.add(s.position.add(s.rotation.mulVec3(child.getCenterOfMass())).mulScalar(child_mass));
            inner_radius = math.min(inner_radius, child.getInnerRadius());
        }
        center_of_mass = center_of_mass.divScalar(mass);
        try expect(shape.asShape().getCenterOfMass().isClose(center_of_mass, .{ .max_dist_sq = 1.0e-8 }));
        try testing.expectEqual(inner_radius, shape.asShape().getInnerRadius());
        for (settings.base.sub_shapes.items, shape.base.sub_shapes.items) |*s, *sub_shape| {
            try testing.expectEqual(s.user_data, sub_shape.user_data);
            try expect(sub_shape.getPositionCOM().isClose(s.position.sub(shape.asShape().getCenterOfMass()), .{ .max_dist_sq = 1.0e-8 }));
        }
        try testing.expectApproxEqRel(mass, shape.asShape().getMassProperties().mass, 1.0e-5);

        // The same compound as a MutableCompoundShape: same center of mass and sub shapes
        var mutable_settings = MutableCompoundShapeSettings.init(allocator);
        defer mutable_settings.deinit();
        try addRandomSubShapes(&mutable_settings.base, &children, n, extent, seed);
        var mutable_result = try mutable_settings.asShapeSettings().createShape(allocator);
        defer mutable_result.deinit();
        const mutable = mutable_result.getPtr().?.cast(MutableCompoundShape);
        try expect(mutable.asShape().getCenterOfMass().eql(shape.asShape().getCenterOfMass()));
        for (mutable.base.sub_shapes.items, shape.base.sub_shapes.items) |*a, *b|
            try expect(a.getPositionCOM().eql(b.getPositionCOM()) and a.getRotation().eql(b.getRotation()));

        // Stats: the shape, the sub shapes and the nodes
        try testing.expectEqual(@sizeOf(StaticCompoundShape) + n * @sizeOf(SubShape) + shape.nodes.items.len * @sizeOf(StaticCompoundShape.Node), shape.asShape().getStats().size_bytes);
    }

    // All sub shapes at the same position: the partitioning cannot split them and falls back to the middle
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        for (0..40) |_| try settings.base.addShapePtr(Vec3.init(1, 2, 3), Quat.identity(), children.sphere1.asShape(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try checkTree(allocator, result.getPtr().?.cast(StaticCompoundShape));
    }
}

test "StaticCompoundShape: partition and partition4" {
    // 4 or fewer: the middle
    var idx = [_]u32{ 0, 1, 2, 3, 4, 5, 6, 7 };
    var bounds: [8]AABox = undefined;
    for (&bounds, 0..) |*b, i| b.* = .fromCenterAndRadius(Vec3.init(@floatFromInt(7 - i), 0, 0), 0.1);
    try testing.expectEqual(@as(i32, 1), StaticCompoundShape.partition(idx[0..3], bounds[0..3]));
    try testing.expectEqual(@as(i32, 2), StaticCompoundShape.partition(idx[0..4], bounds[0..4]));
    try testing.expectEqual(@as(i32, 0), StaticCompoundShape.partition(idx[0..0], bounds[0..0]));

    // Split along X at the middle of the centers (3.5): the elements left of it are swapped to the front
    try testing.expectEqual(@as(i32, 4), StaticCompoundShape.partition(&idx, &bounds));
    for (idx[0..4]) |i| try testing.expect(i >= 4);
    for (idx[4..]) |i| try testing.expect(i < 4);
    for (bounds, idx) |b, i| try testing.expectEqual(@as(f32, @floatFromInt(7 - i)), b.getCenter().getX());

    // 4 groups
    var split: [5]i32 = undefined;
    for (&bounds, 0..) |*b, i| b.* = .fromCenterAndRadius(Vec3.init(0, 0, @floatFromInt(i)), 0.1);
    StaticCompoundShape.partition4(&idx, &bounds, 0, 8, &split);
    try testing.expectEqualSlices(i32, &.{ 0, 2, 4, 6, 8 }, &split);
    StaticCompoundShape.partition4(&idx, &bounds, 2, 7, &split);
    try testing.expectEqual(@as(i32, 2), split[0]);
    try testing.expectEqual(@as(i32, 7), split[4]);
}

test "StaticCompoundShape: the queries find the same hits as MutableCompoundShape (a linear walk)" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var children: TestChildren = undefined;
    children.init(allocator);
    defer children.deinit();

    var prng = std.Random.DefaultPrng.init(1234);
    const random = prng.random();
    for ([_]usize{ 2, 5, 33, 120 }, 0..) |n, seed| {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try addRandomSubShapes(&settings.base, &children, n, 6.0, seed + 100);
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const static_shape = result.getPtr().?;
        var mutable_settings = MutableCompoundShapeSettings.init(allocator);
        defer mutable_settings.deinit();
        try addRandomSubShapes(&mutable_settings.base, &children, n, 6.0, seed + 100);
        var mutable_result = try mutable_settings.asShapeSettings().createShape(allocator);
        defer mutable_result.deinit();
        const mutable_shape = mutable_result.getPtr().?;
        const creator = SubShapeIDCreator.pushID(.{}, 1, 1);

        for (0..50) |_| {
            const origin = Vec3.init(random.float(f32) * 20.0 - 10.0, random.float(f32) * 20.0 - 10.0, random.float(f32) * 20.0 - 10.0);
            const target = Vec3.init(random.float(f32) * 8.0 - 4.0, random.float(f32) * 8.0 - 4.0, random.float(f32) * 8.0 - 4.0);
            const ray = RayCast.init(origin, target.sub(origin).mulScalar(2.0));

            // Closest hit
            var hit_static: RayCastResult = .{};
            var hit_mutable: RayCastResult = .{};
            try testing.expectEqual(mutable_shape.castRay(ray, creator, &hit_mutable), static_shape.castRay(ray, creator, &hit_static));
            try testing.expectEqual(hit_mutable.fraction, hit_static.fraction);
            if (hit_static.fraction < 1.0) try expect(hit_static.sub_shape_id2.eql(hit_mutable.sub_shape_id2));

            // All hits
            var all_static = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer all_static.deinit();
            var all_mutable = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer all_mutable.deinit();
            var ray_settings: RayCastSettings = .{};
            ray_settings.setBackFaceMode(.collide_with_back_faces);
            static_shape.castRayCollector(ray, &ray_settings, creator, &all_static.base, &.{});
            mutable_shape.castRayCollector(ray, &ray_settings, creator, &all_mutable.base, &.{});
            try all_static.checkError();
            try all_mutable.checkError();
            sortByID(RayCastResult, all_static.hits.items);
            sortByID(RayCastResult, all_mutable.hits.items);
            try testing.expectEqual(all_mutable.hits.items.len, all_static.hits.items.len);
            for (all_static.hits.items, all_mutable.hits.items) |a, b|
                try expect(a.sub_shape_id2.eql(b.sub_shape_id2) and a.fraction == b.fraction);

            // Collide point
            var points_static = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer points_static.deinit();
            var points_mutable = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer points_mutable.deinit();
            static_shape.collidePoint(target, creator, &points_static.base, &.{});
            mutable_shape.collidePoint(target, creator, &points_mutable.base, &.{});
            try points_static.checkError();
            try points_mutable.checkError();
            sortByID(CollidePointResult, points_static.hits.items);
            sortByID(CollidePointResult, points_mutable.hits.items);
            try testing.expectEqual(points_mutable.hits.items.len, points_static.hits.items.len);
            for (points_static.hits.items, points_mutable.hits.items) |a, b| try expect(a.sub_shape_id2.eql(b.sub_shape_id2));

            // Intersecting sub shapes (the static tree tests its half float bounds, which can be a little bigger)
            const box = AABox.fromCenterAndRadius(target, 1.0 + random.float(f32));
            var indices_static: [257]u32 = undefined;
            var indices_mutable: [257]u32 = undefined;
            const num_static = static_shape.cast(CompoundShape).getIntersectingSubShapes(box, &indices_static);
            const num_mutable = mutable_shape.cast(CompoundShape).getIntersectingSubShapes(box, &indices_mutable);
            for (indices_mutable[0..num_mutable]) |m| try expect(std.mem.indexOfScalar(u32, indices_static[0..num_static], m) != null);
            const oriented_box = OrientedBox.fromAABox(Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5), target), AABox.fromCenterAndRadius(Vec3.zero(), 1.5));
            const num_static_obb = static_shape.cast(CompoundShape).getIntersectingSubShapesOrientedBox(oriented_box, &indices_static);
            const num_mutable_obb = mutable_shape.cast(CompoundShape).getIntersectingSubShapesOrientedBox(oriented_box, &indices_mutable);
            for (indices_mutable[0..num_mutable_obb]) |m| try expect(std.mem.indexOfScalar(u32, indices_static[0..num_static_obb], m) != null);

            // Collect transformed shapes
            var collected_static = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
            defer collected_static.deinit();
            var collected_mutable = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
            defer collected_mutable.deinit();
            const rotation = Quat.rotation(Vec3.axisX(), 0.2);
            static_shape.collectTransformedShapes(box, Vec3.init(0.5, 0, 0), rotation, Vec3.replicate(1.5), creator, &collected_static.base, &.{});
            mutable_shape.collectTransformedShapes(box, Vec3.init(0.5, 0, 0), rotation, Vec3.replicate(1.5), creator, &collected_mutable.base, &.{});
            try collected_static.checkError();
            try collected_mutable.checkError();
            for (collected_mutable.hits.items) |m| {
                var found = false;
                for (collected_static.hits.items) |s| {
                    if (s.sub_shape_id_creator.getID().eql(m.sub_shape_id_creator.getID())) {
                        try expect(s.shape_position_com.eql(m.shape_position_com) and s.shape_rotation.eql(m.shape_rotation) and s.shape.get() == m.shape.get());
                        found = true;
                    }
                }
                try expect(found);
            }

            // Collide a sphere with the compound through CollisionDispatch
            var collide_settings: CollideShapeSettings = .{};
            collide_settings.max_separation_distance = 0.1;
            const sphere_transform = Mat44.translation(target);
            var collide_static = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collide_static.deinit();
            var collide_mutable = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collide_mutable.deinit();
            CollisionDispatch.collideShapeVsShape(children.sphere2.asShape(), static_shape, Vec3.one(), Vec3.one(), sphere_transform, Mat44.identity(), .{}, creator, &collide_settings, &collide_static.base, &.{});
            CollisionDispatch.collideShapeVsShape(children.sphere2.asShape(), mutable_shape, Vec3.one(), Vec3.one(), sphere_transform, Mat44.identity(), .{}, creator, &collide_settings, &collide_mutable.base, &.{});
            try collide_static.checkError();
            try collide_mutable.checkError();
            sortByID(CollideShapeResult, collide_static.hits.items);
            sortByID(CollideShapeResult, collide_mutable.hits.items);
            try testing.expectEqual(collide_mutable.hits.items.len, collide_static.hits.items.len);
            for (collide_static.hits.items, collide_mutable.hits.items) |a, b|
                try expect(a.sub_shape_id2.eql(b.sub_shape_id2) and a.penetration_depth == b.penetration_depth and a.contact_point_on2.eql(b.contact_point_on2));

            // The compound as shape 1
            collide_static.base.reset();
            collide_mutable.base.reset();
            CollisionDispatch.collideShapeVsShape(static_shape, children.box1.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), sphere_transform, creator, .{}, &collide_settings, &collide_static.base, &.{});
            CollisionDispatch.collideShapeVsShape(mutable_shape, children.box1.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), sphere_transform, creator, .{}, &collide_settings, &collide_mutable.base, &.{});
            try collide_static.checkError();
            try collide_mutable.checkError();
            sortByID(CollideShapeResult, collide_static.hits.items);
            sortByID(CollideShapeResult, collide_mutable.hits.items);
            try testing.expectEqual(collide_mutable.hits.items.len, collide_static.hits.items.len);
            for (collide_static.hits.items, collide_mutable.hits.items) |a, b|
                try expect(a.sub_shape_id1.eql(b.sub_shape_id1) and a.penetration_depth == b.penetration_depth and a.contact_point_on1.eql(b.contact_point_on1));

            // Cast a sphere against the compound
            const shape_cast = ShapeCast.init(children.sphere1.asShape(), Vec3.one(), Mat44.translation(origin), target.sub(origin).mulScalar(2.0));
            const cast_settings: ShapeCastSettings = .{};
            var cast_static = AllHitCollisionCollector(CastShapeCollector).init(allocator);
            defer cast_static.deinit();
            var cast_mutable = AllHitCollisionCollector(CastShapeCollector).init(allocator);
            defer cast_mutable.deinit();
            CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &cast_settings, static_shape, Vec3.one(), &.{}, Mat44.identity(), .{}, creator, &cast_static.base);
            CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &cast_settings, mutable_shape, Vec3.one(), &.{}, Mat44.identity(), .{}, creator, &cast_mutable.base);
            try cast_static.checkError();
            try cast_mutable.checkError();
            sortByID(ShapeCastResult, cast_static.hits.items);
            sortByID(ShapeCastResult, cast_mutable.hits.items);
            try testing.expectEqual(cast_mutable.hits.items.len, cast_static.hits.items.len);
            for (cast_static.hits.items, cast_mutable.hits.items) |a, b|
                try expect(a.base.sub_shape_id2.eql(b.base.sub_shape_id2) and a.fraction == b.fraction);
        }

        // A query that stops at the first hit
        var any = AnyHitCollisionCollector(CollidePointCollector).init();
        defer any.deinit();
        static_shape.collidePoint(static_shape.cast(CompoundShape).getSubShape(0).getPositionCOM(), creator, &any.base, &.{});
        try expect(any.hadHit());
    }
}

test "StaticCompoundShape: registration, sub shape IDs, shape filters and binary state" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // The registered functions
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.static_compound, .sphere) == &StaticCompoundShape.collideCompoundVsShape);
    try expect(r.getCollideShape(.box, .static_compound) == &StaticCompoundShape.collideShapeVsCompound);
    try expect(r.getCollideShape(.static_compound, .static_compound) == &StaticCompoundShape.collideShapeVsCompound); // The second registration of the pair wins
    try expect(r.getCollideShape(.mutable_compound, .static_compound) == &StaticCompoundShape.collideShapeVsCompound); // StaticCompoundShape registers after MutableCompoundShape
    try expect(r.getCollideShape(.static_compound, .mutable_compound) == &StaticCompoundShape.collideCompoundVsShape);
    try expect(r.getCastShape(.capsule, .static_compound) == &StaticCompoundShape.castShapeVsCompound);
    try expect(r.getCastShape(.static_compound, .sphere) == &CompoundShape.castCompoundVsShape);
    try expect(r.getCollideShape(.static_compound, .rotated_translated) != &StaticCompoundShape.collideCompoundVsShape); // Decorators register later
    try expect(ShapeFunctions.get(.static_compound).color.eql(Color.orange));
    try expect(ShapeFunctions.get(.static_compound).construct != null);

    var children: TestChildren = undefined;
    children.init(allocator);
    defer children.deinit();
    var settings = StaticCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    settings.asShapeSettings().user_data = 42;
    try addRandomSubShapes(&settings.base, &children, 9, 5.0, 77);
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.cast(StaticCompoundShape);

    // Sub shape IDs: 4 bits for 9 sub shapes, the leaf shape of every sub shape
    try testing.expectEqual(@as(u32, 4), shape.base.getSubShapeIDBits());
    for (shape.base.sub_shapes.items, 0..) |*s, i| {
        const id = shape.base.getSubShapeIDFromIndex(@intCast(i), .{}).getID();
        const leaf = shape.asShape().getLeafShape(id);
        try expect(leaf.shape == s.shape.get());
        try testing.expectEqual(@as(u32, @intCast(i)), shape.base.getSubShapeIndexFromID(id).index);
    }

    // A shape filter that rejects the compound itself
    const RejectCompound = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, sub_shape_id_of_shape2 };
            return shape2.getSubType() != .static_compound;
        }
    };
    const reject: RejectCompound = .{};
    var ray_hits = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer ray_hits.deinit();
    shape.asShape().castRayCollector(.init(Vec3.replicate(-20), Vec3.replicate(40)), &.{}, .{}, &ray_hits.base, &reject.base);
    var collected = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collected.deinit();
    shape.asShape().collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), .{}, &collected.base, &reject.base);
    try testing.expectEqual(@as(usize, 0), ray_hits.hits.items.len + collected.hits.items.len);
    shape.asShape().collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), .{}, &collected.base, &.{});
    try collected.checkError();
    try testing.expectEqual(@as(usize, 9), collected.hits.items.len);

    // Binary state: the compound part followed by the nodes
    var buffer: [4096]u8 = undefined;
    const bytes = saveToBuffer(shape.asShape(), &buffer);
    var restored_result = try restoreFromBuffer(allocator, bytes);
    defer restored_result.deinit();
    const restored = restored_result.getPtr().?.castMut(StaticCompoundShape);
    try testing.expectEqual(@as(u64, 42), restored.asShape().getUserData());
    try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(shape.nodes.items), std.mem.sliceAsBytes(restored.nodes.items));
    var sub_shapes: ShapeFile.ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try shape.asShape().saveSubShapeState(allocator, &sub_shapes);
    restored.asShapeMut().restoreSubShapeState(sub_shapes.items);
    var buffer2: [4096]u8 = undefined;
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(restored.asShape(), &buffer2));
    try checkTree(allocator, restored);

    // A truncated stream fails
    var truncated = try restoreFromBuffer(allocator, bytes[0 .. bytes.len - 1]);
    defer truncated.deinit();
    try testing.expectEqualStrings("Failed to restore shape", truncated.getError());
}

test "StaticCompoundShape: every creation path that allocates reports out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var children: TestChildren = undefined;
    children.init(allocator);
    defer children.deinit();

    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, StaticCompoundShapeSettings.create(failing.allocator()));
    try testing.expectError(error.OutOfMemory, StaticCompoundShape.create(failing.allocator()));
    {
        const empty = try StaticCompoundShape.create(allocator);
        empty.asShapeMut().destroy();
    }
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.static_compound).construct.?(failing.allocator()));

    // createShape with child settings (1: a RotatedTranslatedShape, otherwise a StaticCompoundShape): out of memory is
    // returned and not cached, a later call succeeds
    for ([_]usize{ 1, 2, 12 }) |n| {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        const child = try SphereShapeSettings.create(allocator, 1.0, .{});
        try settings.base.addShape(Vec3.init(1, 2, 3), Quat.identity(), child.asShapeSettings(), .{});
        try addRandomSubShapes(&settings.base, &children, n - 1, 5.0, n);
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var result = settings.asShapeSettings().createShape(f.allocator()) catch |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try expect(settings.asShapeSettings().cached_result.isEmpty());
                child.asShapeSettings().clearCachedResult();
                continue;
            };
            defer result.deinit();
            try expect(result.isValid()); // A failure of the shrink of the nodes (shrink_to_fit) keeps the bigger buffer
            try expect(fail_index >= 2);
            child.asShapeSettings().clearCachedResult();
            settings.asShapeSettings().clearCachedResult();
            break;
        }
    }

    // A temp allocator that is too small reports out of memory
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try addRandomSubShapes(&settings.base, &children, 50, 5.0, 3);
        var small = try TempAllocatorImpl.init(allocator, 1024);
        defer small.deinit();
        try testing.expectError(error.OutOfMemory, settings.createShapeWithTempAllocator(allocator, small.tempAllocator()));
        try expect(settings.asShapeSettings().cached_result.isEmpty() and small.isEmpty());
        var big = try TempAllocatorImpl.init(allocator, 64 * 1024);
        defer big.deinit();
        var result = try settings.createShapeWithTempAllocator(allocator, big.tempAllocator());
        defer result.deinit();
        try expect(result.isValid() and big.isEmpty());
        try checkTree(allocator, result.getPtr().?.cast(StaticCompoundShape));
    }

    // Restore: the shape, the sub shapes and the nodes are allocated
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try addRandomSubShapes(&settings.base, &children, 6, 5.0, 4);
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        var buffer: [2048]u8 = undefined;
        const bytes = saveToBuffer(result.getPtr().?, &buffer);
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var restored = restoreFromBuffer(f.allocator(), bytes) catch continue;
            defer restored.deinit();
            try testing.expectEqual(@as(usize, 3), fail_index);
            break;
        }
    }
}

test "Shape.scaleShape: zero scale, the shape itself, a ScaledShape or a StaticCompoundShape of scaled leaves" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var children: TestChildren = undefined;
    children.init(allocator);
    defer children.deinit();

    // Zero scale
    {
        var result = try children.box1.asShape().scaleShape(allocator, Vec3.init(1.0e-7, 0, -1.0e-7));
        defer result.deinit();
        try testing.expectEqualStrings("Can't use zero scale!", result.getError());
    }

    // A unit scale: the shape itself
    {
        var result = try children.box1.asShape().scaleShape(allocator, Vec3.replicate(1.0 + 1.0e-7));
        defer result.deinit();
        try expect(result.getPtr().? == children.box1.asShapeMut());
    }

    // A valid scale: a ScaledShape
    {
        var result = try children.box1.asShape().scaleShape(allocator, Vec3.init(1, 2, 3));
        defer result.deinit();
        const scaled = result.getPtr().?.cast(ScaledShape);
        try expect(scaled.getScale().eql(Vec3.init(1, 2, 3)) and scaled.base.getInnerShape() == children.box1.asShape());
    }

    // A non uniform scale of a rotated box in a compound: a StaticCompoundShape of scaled leaves
    {
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.init(1, 0, 0), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), children.box1.asShape(), .{});
        try settings.base.addShapePtr(Vec3.init(-1, 0, 0), Quat.identity(), children.sphere2.asShape(), .{});
        try settings.base.addShapePtr(Vec3.init(0, 3, 0), Quat.identity(), children.box2.asShape(), .{});
        var compound = try settings.asShapeSettings().createShape(allocator);
        defer compound.deinit();
        const scale = Vec3.init(1, 2, 1);
        try expect(!compound.getPtr().?.isValidScale(scale));
        var result = try compound.getPtr().?.scaleShape(allocator, scale);
        defer result.deinit();
        const scaled = result.getPtr().?.cast(StaticCompoundShape);
        try testing.expectEqual(@as(u32, 3), scaled.base.getNumSubShapes());
        for (scaled.base.getSubShapes()) |*s| try expect(s.shape.get().?.getSubType() == .scaled);
        const bounds = scaled.asShape().getWorldSpaceBounds(Mat44.translation(scaled.asShape().getCenterOfMass()), Vec3.one());
        try expect(bounds.max.getY() > 6.0); // The box at y = 3 moved to y = 6

        // Out of memory: the scaled shapes, the collector, the compound
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var r = compound.getPtr().?.scaleShape(f.allocator(), scale) catch continue;
            defer r.deinit();
            try expect(r.isValid() and fail_index > 3);
            break;
        }
    }

    // A single leaf with an invalid scale: the compound settings make it a RotatedTranslatedShape
    {
        var rt = RotatedTranslatedShape.init(allocator, Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), children.box1.asShape());
        rt.asShape().setEmbedded();
        defer rt.asShapeMut().deinit();
        var result = try rt.asShape().scaleShape(allocator, Vec3.init(1, 2, 1));
        defer result.deinit();
        try expect(result.getPtr().?.getSubType() == .rotated_translated);
    }

    // Leaves with a center of mass that is not at their origin (the spheres and boxes have it at the origin, so the parity
    // test cannot check this part): every leaf of the scaled compound keeps its center of mass where TransformShape put it
    {
        var offset_box = TestBoxShape.init(allocator, Vec3.init(1, 0.5, 0.25), .{ .center_of_mass = Vec3.init(0.5, -0.25, 0.1) });
        offset_box.asShape().setEmbedded();
        defer offset_box.asShapeMut().deinit();
        var settings = StaticCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.init(1, 2, 3), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), offset_box.asShape(), .{});
        try settings.base.addShapePtr(Vec3.init(-1, 0, 0), Quat.rotation(Vec3.axisX(), 0.3), offset_box.asShape(), .{});
        var compound = try settings.asShapeSettings().createShape(allocator);
        defer compound.deinit();
        const shape = compound.getPtr().?;
        const scale = Vec3.init(1, 2, 1.5);
        var leaves = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer leaves.deinit();
        shape.transformShape(Mat44.scaleVec3(scale).mul(Mat44.translation(shape.getCenterOfMass())), &leaves.base);
        try leaves.checkError();
        var result = try shape.scaleShape(allocator, scale);
        defer result.deinit();
        const scaled = result.getPtr().?.cast(StaticCompoundShape);
        try testing.expectEqual(@as(usize, 2), leaves.hits.items.len);
        for (leaves.hits.items, scaled.base.getSubShapes()) |*ts, *s| {
            const world_com = s.getPositionCOM().add(scaled.asShape().getCenterOfMass());
            try expect(world_com.isClose(ts.shape_position_com.toVec3(), .{ .max_dist_sq = 1.0e-10 }));
            try expect(s.shape.get().?.cast(ScaledShape).getScale().isClose(ts.getShapeScale(), .{}));
        }
    }
}

/// sTestHit / sTestMiss of Jolt's CollidePointTests (the shape part, the NarrowPhaseQuery part needs Phase 5)
fn expectPointHits(shape: *const Shape, point: Vec3, expected: usize) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(testing.allocator);
    defer collector.deinit();
    shape.collidePoint(point.sub(shape.getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try testing.expectEqual(expected, collector.hits.items.len);
}

test "StaticCompoundShape: TestCollidePointVsStaticCompound (the compound case of Jolt's shared CollidePointTests, shape part)" {
    const allocator = testing.allocator;

    const translation1 = Vec3.init(10.0, 11.0, 12.0);
    const rotation1 = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3 * math.pi);
    const transform1 = Mat44.rotationTranslation(rotation1, translation1);

    const translation2 = Vec3.init(-1.0, -2.0, -3.0);
    const rotation2 = Quat.rotation(Vec3.init(4, 5, 6).normalized(), 0.2 * math.pi);
    const transform2 = Mat44.rotationTranslation(rotation2, translation2);

    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer box.deinit();

    var settings = StaticCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    try settings.base.addShapePtr(translation1, rotation1, box.get(), .{});
    try settings.base.addShapePtr(translation2, rotation2, box.get(), .{});
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?;

    // Hits
    const cube_and_zero_probes = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(-1.0, 0, 0), Vec3.init(1.0, 0, 0), Vec3.init(0, -1.0, 0), Vec3.init(0, 1.0, 0), Vec3.init(0, 0, -1.0), Vec3.init(0, 0, 1.0) };
    for (cube_and_zero_probes) |probe| {
        const point = half_box_size.mulScalar(0.99).mul(probe);
        try expectPointHits(shape, transform1.mulVec3(point), 1);
        try expectPointHits(shape, transform2.mulVec3(point), 1);
    }

    // Misses
    for (cube_and_zero_probes[1..]) |probe| {
        const point = half_box_size.mulScalar(1.01).mul(probe);
        try expectPointHits(shape, transform1.mulVec3(point), 0);
        try expectPointHits(shape, transform2.mulVec3(point), 0);
    }
}
