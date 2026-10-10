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
