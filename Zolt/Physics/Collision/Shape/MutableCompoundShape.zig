//! Port of: Jolt/Physics/Collision/Shape/MutableCompoundShape.h, Jolt/Physics/Collision/Shape/MutableCompoundShape.cpp
//! Status: complete
//!
//! A concrete compound shape (Docs/Zolt/CollisionArchitecture.md, section 2 "Compound shapes"):
//! - `MutableCompoundShapeSettings` / `MutableCompoundShape` embed `CompoundShapeSettings` / `CompoundShape` as `base`
//!   (D1); `overrides` lists every C++ `override` of the header in header order. The functions that
//!   MutableCompoundShape does not override come from `CompoundShape.impl` (and `Shape.impl`).
//! - Constructors: `initDefault` (default constructor), `initFromSettings` (D3, Jolt's error texts). `Clone()` is
//!   `clone(allocator)`: it creates the copy with `allocator` (creation takes an allocator, D13) and returns it as a
//!   `Ref(Shape)` (Jolt's `Ref<MutableCompoundShape>`: concrete shapes are held as `Ref(Shape)` + `castMut`).
//! - The sub shape array and the bounds blocks (`mSubShapeBounds`, `sub_shape_bounds`) are allocated with the shape's
//!   allocator (`base.base.allocator`) and freed in `destruct`. The mutating functions take `*MutableCompoundShape`
//!   (Rule M). `addShape` is the only one that can grow the arrays: it returns `Allocator.Error!u32` and reserves the
//!   memory before changing anything, so running out of memory leaves the compound unchanged. `removeShape`,
//!   `modifyShape(s)` and `adjustCenterOfMass` never grow the arrays (CalculateSubShapeBounds' EnsureSubShapeBoundsCapacity
//!   only allocates when the number of blocks grows), so they keep Jolt's `void`.
//! - `AddShape(..., inUserData = 0, inIndex = UINT_MAX)` is `addShape(position, rotation, shape, .{ .user_data, .index })`;
//!   the `ModifyShape(index, position, rotation, const Shape *)` overload is `modifyShapeWithShape`; `ModifyShapes(start,
//!   number, const Vec3 *, const Quat *, positionStride, rotationStride)` takes `StridedPtrConst(Vec3)` /
//!   `StridedPtrConst(Quat)` (the pointer and its stride, default stride = the size of the type like Jolt).
//! - `WalkSubShapes` takes the visitor as `anytype` (porting guide section 6). Jolt's visitors (`struct Visitor : public
//!   CastRayVisitor { using Result = Vec4; TestBlock, ShouldVisitBlock, ShouldVisitSubShape }`) embed the visitor of
//!   CompoundShapeVisitors.zig as `base` and forward ShouldAbort / VisitShape; they repeat one of two patterns, built by
//!   `DistanceVisitor` (ray and shape casts: Vec4 distances compared with the early out fraction) and `OverlapVisitor`
//!   (the others: UVec4 overlap masks). `GetIntersectingSubShapesVisitorMC<BoxType>` (a named private template in Jolt)
//!   is the comptime function `GetIntersectingSubShapesVisitorMC(BoxType)`.
//! - The private `Bounds` struct, `GetNumBlocks`, `EnsureSubShapeBoundsCapacity`, `CalculateSubShapeBounds`,
//!   `CalculateLocalBounds` and `WalkSubShapes` are pub so that tests can check them. New bounds blocks are zero
//!   initialized (Jolt leaves the Vec4s uninitialized; they are always written before they are read).
//! - The helpers registered in CollisionDispatch drop the `s` (`collideCompoundVsShape`, `collideShapeVsCompound`,
//!   `castShapeVsCompound`).
//! - JPH_DEBUG_RENDERER: MutableCompoundShape has no draw code of its own (CompoundShape's Draw is TODO(debug_renderer)).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Color = @import("../../../Core/Color.zig").Color;
const Ref = @import("../../../Core/Reference.zig").Ref;
const StreamIn = @import("../../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../../Core/StreamOut.zig").StreamOut;
const StridedPtrConst = @import("../../../Core/StridedPtr.zig").StridedPtrConst;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
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
const CollisionDispatch = @import("../CollisionDispatch.zig");
const Registry = CollisionDispatch.Registry;

/// Class that constructs a MutableCompoundShape.
pub const MutableCompoundShapeSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, MutableCompoundShapeSettings)

    /// The virtual functions this class overrides (C++ `override`)
    pub const overrides = .{.createShape};

    base: CompoundShapeSettings,

    /// Constructor. Use addShape / addShapePtr (`base.addShape(...)`) to add the parts.
    pub fn init(allocator: Allocator) MutableCompoundShapeSettings {
        return .{ .base = .init(MutableCompoundShapeSettings, allocator) };
    }

    /// new MutableCompoundShapeSettings: reference count 0, freed by the last release()
    pub fn create(allocator: Allocator) Allocator.Error!*MutableCompoundShapeSettings {
        const self = try allocator.create(MutableCompoundShapeSettings);
        self.* = .init(allocator);
        return self;
    }

    /// Upcast (mutable: createShape writes the cache)
    pub fn asShapeSettings(self: *MutableCompoundShapeSettings) *ShapeSettings {
        return &self.base.base;
    }

    /// Destructor of settings that are not on the heap
    pub fn deinit(self: *MutableCompoundShapeSettings) void {
        self.asShapeSettings().deinit();
    }

    // See: ShapeSettings
    pub fn createShape(self: *MutableCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        // Build a mutable compound shape
        return ShapeSettings.createCached(MutableCompoundShape, self, allocator);
    }
};

/// A compound shape, sub shapes can be rotated and translated.
/// This shape is optimized for adding / removing and changing the rotation / translation of sub shapes but is less efficient in querying.
/// Shifts all child objects so that they're centered around the center of mass (which needs to be kept up to date by calling AdjustCenterOfMass).
///
/// Note: If you're using MutableCompoundShape and are querying data while modifying the shape you'll have a race condition.
/// In this case it is best to create a new MutableCompoundShape using the Clone function. You replace the shape on a body using BodyInterface::SetShape.
/// If a query is still working on the old shape, it will have taken a reference and keep the old shape alive until the query finishes.
///
/// When you modify a MutableCompoundShape, beware that the SubShapeIDs of all other shapes can change. So be careful when storing SubShapeIDs.
pub const MutableCompoundShape = struct {
    /// Concrete class: `Shape.cast(MutableCompoundShape)` checks the sub shape type
    pub const shape_sub_type: ShapeSubType = .mutable_compound;

    /// The virtual functions this class overrides (C++ `override`, header order), checked by the vtable builder
    pub const overrides = .{ .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .getIntersectingSubShapes, .getIntersectingSubShapesOrientedBox, .saveBinaryState, .getStats, .restoreBinaryState };

    /// Bounding boxes of 4 sub shapes in structure of arrays format (private in Jolt)
    pub const Bounds = extern struct {
        min_x: Vec4,
        min_y: Vec4,
        min_z: Vec4,
        max_x: Vec4,
        max_y: Vec4,
        max_z: Vec4,

        comptime {
            std.debug.assert(@sizeOf(Bounds) == 96); // Streamed as raw bytes like in Jolt
        }
    };

    base: CompoundShape,
    /// Bounding boxes of all sub shapes in SOA format (in blocks of 4 boxes), MinX 0..3, MinY 0..3, MinZ 0..3, MaxX 0..3, MaxY 0..3, MaxZ 0..3, MinX 4..7, MinY 4..7, ...
    /// (allocated with base.base.allocator)
    sub_shape_bounds: std.ArrayList(Bounds) = .empty,

    // ---------------------------------------------------------------------------------------------------------------
    // Construction (C++ constructors)

    /// MutableCompoundShape(): default constructor, used by restoreFromBinaryState (ShapeFunctions.construct) and by clone
    pub fn initDefault(allocator: Allocator) MutableCompoundShape {
        return .{ .base = .init(MutableCompoundShape, allocator, shape_sub_type) };
    }

    /// MutableCompoundShape(const MutableCompoundShapeSettings &inSettings, ShapeResult &outResult): base part first,
    /// then the C++ body. `allocator` creates the child shapes from their settings.
    pub fn initFromSettings(self: *MutableCompoundShape, settings: *const MutableCompoundShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base.base, result);
        const c = &self.base;

        try c.sub_shapes.ensureTotalCapacity(c.base.allocator, settings.base.sub_shapes.items.len);
        for (settings.base.sub_shapes.items) |*shape| {
            // Start constructing the runtime sub shape
            var out_shape: SubShape = .{};
            if (!try out_shape.fromSettings(shape, result, allocator))
                return; // No reference was taken

            c.sub_shapes.appendAssumeCapacity(out_shape);
        }

        self.adjustCenterOfMass();

        try self.calculateSubShapeBounds(0, @intCast(c.sub_shapes.items.len));

        // Check if we're not exceeding the amount of sub shape id bits
        if (self.asShape().getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(self.asShapeMut()));
    }

    /// ~MutableCompoundShape
    pub fn destruct(self: *MutableCompoundShape) void {
        self.sub_shape_bounds.deinit(self.base.base.allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asShape(self: *const MutableCompoundShape) *const Shape {
        return &self.base.base;
    }

    pub fn asShapeMut(self: *MutableCompoundShape) *Shape {
        return &self.base.base;
    }

    /// Clone this shape. Can be used to avoid race conditions. See the documentation of this class for more information.
    /// The clone is created with `allocator` and returned with a reference (Jolt's Ref<MutableCompoundShape>, use
    /// `castMut(MutableCompoundShape)` to modify it).
    pub fn clone(self: *const MutableCompoundShape, allocator: Allocator) Allocator.Error!Ref(Shape) {
        const copy = try allocator.create(MutableCompoundShape);
        copy.* = .initDefault(allocator);
        var copy_ref = Ref(Shape).init(copy.asShapeMut());
        errdefer copy_ref.deinit();
        copy.asShapeMut().setUserData(self.asShape().getUserData());

        copy.base.center_of_mass = self.base.center_of_mass;
        copy.base.local_bounds = self.base.local_bounds;
        try copy.base.sub_shapes.ensureTotalCapacity(allocator, self.base.sub_shapes.items.len);
        for (self.base.sub_shapes.items) |*sub_shape|
            copy.base.sub_shapes.appendAssumeCapacity(sub_shape.clone());
        copy.base.inner_radius = self.base.inner_radius;
        try copy.sub_shape_bounds.appendSlice(allocator, self.sub_shape_bounds.items);

        return copy_ref;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Mutating shapes. Note that this is not thread safe, so you need to ensure that any bodies that use this shape are locked at the time of modification using BodyLockWrite. After modification you need to call BodyInterface::NotifyShapeChanged to update the broadphase and collision caches.

    /// Adding a new shape.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    /// @param position The position of the new shape
    /// @param rotation The orientation of the new shape
    /// @param shape The shape to add
    /// @param opts.user_data User data that will be stored with the shape and can be retrieved using GetCompoundUserData
    /// @param opts.index Index where to insert the shape, UINT_MAX to add to the end
    /// @return The index of the newly added shape (out of memory leaves the compound unchanged)
    pub fn addShape(self: *MutableCompoundShape, position: Vec3, rotation: Quat, shape: *const Shape, opts: struct { user_data: u32 = 0, index: u32 = std.math.maxInt(u32) }) Allocator.Error!u32 {
        // Zolt: reserve the memory first so that running out of memory leaves the compound unchanged (then
        // CalculateSubShapeBounds cannot fail below)
        const allocator = self.base.base.allocator;
        try self.base.sub_shapes.ensureUnusedCapacity(allocator, 1);
        try self.sub_shape_bounds.ensureTotalCapacity(allocator, (self.base.sub_shapes.items.len + 1 + 3) >> 2);

        var sub_shape: SubShape = .{ .shape = .init(shape) };
        sub_shape.user_data = opts.user_data;
        sub_shape.setTransform(position, rotation, self.base.center_of_mass);

        if (opts.index >= self.base.sub_shapes.items.len) {
            const shape_idx: u32 = @intCast(self.base.sub_shapes.items.len);
            self.base.sub_shapes.appendAssumeCapacity(sub_shape);
            self.calculateSubShapeBounds(shape_idx, 1) catch unreachable; // The capacity was reserved above
            return shape_idx;
        } else {
            self.base.sub_shapes.insertAssumeCapacity(opts.index, sub_shape);
            self.calculateSubShapeBounds(opts.index, @as(u32, @intCast(self.base.sub_shapes.items.len)) - opts.index) catch unreachable; // The capacity was reserved above
            return opts.index;
        }
    }

    /// Remove a shape by index.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    pub fn removeShape(self: *MutableCompoundShape, index: u32) void {
        var removed = self.base.sub_shapes.orderedRemove(index);
        removed.deinit();

        // We always need to recalculate the bounds of the sub shapes as we test blocks
        // of 4 sub shapes at a time and removed shapes get their bounds updated
        // to repeat the bounds of the previous sub shape
        const num_bounds: u32 = @as(u32, @intCast(self.base.sub_shapes.items.len)) - index;
        self.calculateSubShapeBounds(index, num_bounds) catch unreachable; // The number of blocks did not grow, nothing is allocated
    }

    /// Modify the position / orientation of a shape.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    pub fn modifyShape(self: *MutableCompoundShape, index: u32, position: Vec3, rotation: Quat) void {
        const sub_shape = &self.base.sub_shapes.items[index];
        sub_shape.setTransform(position, rotation, self.base.center_of_mass);

        self.calculateSubShapeBounds(index, 1) catch unreachable; // The number of blocks did not grow, nothing is allocated
    }

    /// Modify the position / orientation and shape at the same time (ModifyShape(inIndex, inPosition, inRotation, inShape)).
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    pub fn modifyShapeWithShape(self: *MutableCompoundShape, index: u32, position: Vec3, rotation: Quat, shape: *const Shape) void {
        const sub_shape = &self.base.sub_shapes.items[index];
        sub_shape.shape.set(shape);
        sub_shape.setTransform(position, rotation, self.base.center_of_mass);

        self.calculateSubShapeBounds(index, 1) catch unreachable; // The number of blocks did not grow, nothing is allocated
    }

    /// Batch set positions / orientations, this avoids duplicate work due to bounding box calculation.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    /// @param start_index Index of first shape to update
    /// @param number Number of shapes to update
    /// @param positions A list of positions with arbitrary stride (the number of bytes between the first and second element, default the size of a Vec3)
    /// @param rotations A list of orientations with arbitrary stride (the number of bytes between the first and second element, default the size of a Quat)
    pub fn modifyShapes(self: *MutableCompoundShape, start_index: u32, number: u32, positions: StridedPtrConst(Vec3), rotations: StridedPtrConst(Quat)) void {
        std.debug.assert(start_index + number <= self.base.sub_shapes.items.len);

        var pos = positions;
        var rot = rotations;
        for (self.base.sub_shapes.items[start_index .. start_index + number]) |*dest| {
            // Update transform
            dest.setTransform(pos.deref().*, rot.deref().*, self.base.center_of_mass);

            // Advance pointer in position / rotation buffer
            pos.increment();
            rot.increment();
        }

        self.calculateSubShapeBounds(start_index, number) catch unreachable; // The number of blocks did not grow, nothing is allocated
    }

    /// Recalculate the center of mass and shift all objects so they're centered around it
    /// (this needs to be done of dynamic bodies and if the center of mass changes significantly due to adding / removing / repositioning sub shapes or else the simulation will look unnatural)
    /// Note that after adjusting the center of mass of an object you need to call BodyInterface::NotifyShapeChanged and Constraint::NotifyShapeChanged on the relevant bodies / constraints.
    /// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
    pub fn adjustCenterOfMass(self: *MutableCompoundShape) void {
        // First calculate the delta of the center of mass
        var mass: f32 = 0.0;
        var center_of_mass = Vec3.zero();
        for (self.base.sub_shapes.items) |*sub_shape| {
            const child = sub_shape.shape.get().?.getMassProperties();
            mass += child.mass;
            center_of_mass = center_of_mass.add(sub_shape.getPositionCOM().mulScalar(child.mass));
        }
        if (mass > 0.0)
            center_of_mass = center_of_mass.divScalar(mass);

        // Now adjust all shapes to recenter around center of mass
        for (self.base.sub_shapes.items) |*sub_shape|
            sub_shape.setPositionCOM(sub_shape.getPositionCOM().sub(center_of_mass));

        // Update bounding boxes
        for (self.sub_shape_bounds.items) |*bounds| {
            const xxxx = center_of_mass.splatX();
            const yyyy = center_of_mass.splatY();
            const zzzz = center_of_mass.splatZ();
            bounds.min_x = bounds.min_x.sub(xxxx);
            bounds.min_y = bounds.min_y.sub(yyyy);
            bounds.min_z = bounds.min_z.sub(zzzz);
            bounds.max_x = bounds.max_x.sub(xxxx);
            bounds.max_y = bounds.max_y.sub(yyyy);
            bounds.max_z = bounds.max_z.sub(zzzz);
        }
        self.base.local_bounds.translate(center_of_mass.negate());

        // And adjust the center of mass for this shape in the opposite direction
        self.base.center_of_mass = self.base.center_of_mass.add(center_of_mass);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Bounds blocks (private in Jolt)

    /// Get the number of blocks of 4 bounding boxes
    pub fn getNumBlocks(self: *const MutableCompoundShape) u32 {
        return (@as(u32, @intCast(self.base.sub_shapes.items.len)) + 3) >> 2;
    }

    /// Ensure that the mSubShapeBounds has enough space to store bounding boxes equivalent to the number of shapes in mSubShapes
    pub fn ensureSubShapeBoundsCapacity(self: *MutableCompoundShape) Allocator.Error!void {
        // Check if we have enough space
        const new_capacity = (@as(u32, @intCast(self.base.sub_shapes.items.len)) + 3) >> 2;
        if (self.sub_shape_bounds.items.len < new_capacity)
            try self.sub_shape_bounds.appendNTimes(self.base.base.allocator, std.mem.zeroes(Bounds), new_capacity - self.sub_shape_bounds.items.len); // resize
    }

    /// Update mSubShapeBounds
    /// @param start_idx First sub shape to update
    /// @param number Number of shapes to update
    pub fn calculateSubShapeBounds(self: *MutableCompoundShape, start_idx: u32, number: u32) Allocator.Error!void {
        // Ensure that we have allocated the required space for mSubShapeBounds
        try self.ensureSubShapeBoundsCapacity();

        // Loop over blocks of 4 sub shapes
        var sub_shape_idx_start = start_idx & ~@as(u32, 3);
        const sub_shape_idx_end = start_idx + number;
        while (sub_shape_idx_start < sub_shape_idx_end) : (sub_shape_idx_start += 4) {
            var bounds_min: Mat44 = undefined;
            var bounds_max: Mat44 = undefined;

            var sub_shape_bounds: AABox = .empty;
            for (0..4) |c| {
                const col: u32 = @intCast(c);
                const sub_shape_idx = sub_shape_idx_start + col;
                if (sub_shape_idx < self.base.sub_shapes.items.len) { // else reuse sub_shape_bounds from previous iteration
                    const sub_shape = &self.base.sub_shapes.items[sub_shape_idx];

                    // Transform the shape's bounds into our local space
                    const transform = Mat44.rotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM());

                    // Get the bounding box
                    sub_shape_bounds = sub_shape.shape.get().?.getWorldSpaceBounds(transform, Vec3.one());
                }

                // Put the bounds as columns in a matrix
                bounds_min.setColumn3(col, sub_shape_bounds.min);
                bounds_max.setColumn3(col, sub_shape_bounds.max);
            }

            // Transpose to go to structure of arrays format
            const bounds_min_t = bounds_min.transposed();
            const bounds_max_t = bounds_max.transposed();

            // Store in our bounds array
            const bounds = &self.sub_shape_bounds.items[sub_shape_idx_start >> 2];
            bounds.min_x = bounds_min_t.getColumn4(0);
            bounds.min_y = bounds_min_t.getColumn4(1);
            bounds.min_z = bounds_min_t.getColumn4(2);
            bounds.max_x = bounds_max_t.getColumn4(0);
            bounds.max_y = bounds_max_t.getColumn4(1);
            bounds.max_z = bounds_max_t.getColumn4(2);
        }

        self.calculateLocalBounds();
    }

    /// Calculate mLocalBounds from mSubShapeBounds
    pub fn calculateLocalBounds(self: *MutableCompoundShape) void {
        const num_blocks = self.getNumBlocks();
        if (num_blocks > 0) {
            // Initialize min/max for first block
            const bounds = self.sub_shape_bounds.items[0..num_blocks];
            var min_x = bounds[0].min_x;
            var min_y = bounds[0].min_y;
            var min_z = bounds[0].min_z;
            var max_x = bounds[0].max_x;
            var max_y = bounds[0].max_y;
            var max_z = bounds[0].max_z;

            // Accumulate other blocks
            for (bounds[1..]) |*b| {
                min_x = Vec4.min(min_x, b.min_x);
                min_y = Vec4.min(min_y, b.min_y);
                min_z = Vec4.min(min_z, b.min_z);
                max_x = Vec4.max(max_x, b.max_x);
                max_y = Vec4.max(max_y, b.max_y);
                max_z = Vec4.max(max_z, b.max_z);
            }

            // Calculate resulting bounding box
            self.base.local_bounds.min.setX(min_x.reduceMin());
            self.base.local_bounds.min.setY(min_y.reduceMin());
            self.base.local_bounds.min.setZ(min_z.reduceMin());
            self.base.local_bounds.max.setX(max_x.reduceMax());
            self.base.local_bounds.max.setY(max_y.reduceMax());
            self.base.local_bounds.max.setZ(max_z.reduceMax());
        } else {
            // There are no subshapes, make the bounding box empty
            self.base.local_bounds.min = Vec3.zero();
            self.base.local_bounds.max = Vec3.zero();
        }

        // Cache the inner radius as it can take a while to recursively iterate over all sub shapes
        self.base.calculateInnerRadius();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Walking the sub shapes

    /// Walk the sub shapes and call Visitor::VisitShape for each sub shape encountered
    /// (`visitor` is a pointer to a visitor with testBlock, shouldVisitBlock, shouldVisitSubShape, visitShape and shouldAbort)
    pub fn walkSubShapes(self: *const MutableCompoundShape, visitor: anytype) void {
        // Loop over all blocks of 4 bounding boxes
        const num_sub_shapes: u32 = @intCast(self.base.sub_shapes.items.len);
        var block: u32 = 0;
        const num_blocks = self.getNumBlocks();
        while (block < num_blocks) : (block += 1) {
            // Test the bounding boxes
            const bounds = &self.sub_shape_bounds.items[block];
            const result = visitor.testBlock(bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z);

            // Check if any of the bounding boxes collided
            if (visitor.shouldVisitBlock(result)) {
                // Go through the individual boxes
                const sub_shape_start_idx = block << 2;
                var col: u32 = 0;
                const max_col = @min(4, num_sub_shapes - sub_shape_start_idx); // Don't read beyond the end of the subshapes array
                while (col < max_col) : (col += 1) {
                    if (visitor.shouldVisitSubShape(result, col)) { // Because the early out fraction can change, we need to retest every shape
                        // Test sub shape
                        const sub_shape_idx = sub_shape_start_idx + col;
                        const sub_shape = &self.base.sub_shapes.items[sub_shape_idx];
                        visitor.visitShape(sub_shape, sub_shape_idx);

                        // If no better collision is available abort
                        if (visitor.shouldAbort())
                            break;
                    }
                }
            }
        }
    }

    /// The visitors of the ray and shape casts (Jolt's `struct Visitor : public CastRayVisitor { using Result = Vec4; ... }`):
    /// `fraction(&base)` is the early out fraction the visitor compares with (mHit.mFraction,
    /// mCollector.GetEarlyOutFraction() or mCollector.GetPositiveEarlyOutFraction())
    fn DistanceVisitor(comptime Base: type, comptime fraction: fn (*const Base) f32) type {
        return struct {
            const Self = @This();

            base: Base,

            pub fn shouldAbort(self: *const Self) bool {
                return self.base.shouldAbort();
            }

            pub fn visitShape(self: *Self, sub_shape: *const SubShape, sub_shape_index: u32) void {
                self.base.visitShape(sub_shape, sub_shape_index);
            }

            pub fn testBlock(self: *const Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) Vec4 {
                return self.base.testBounds(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
            }

            pub fn shouldVisitBlock(self: *const Self, result: Vec4) bool {
                const closer = Vec4.less(result, Vec4.replicate(fraction(&self.base)));
                return closer.testAnyTrue();
            }

            pub fn shouldVisitSubShape(self: *const Self, result: Vec4, index_in_block: u32) bool {
                return result.getComponent(index_in_block) < fraction(&self.base);
            }
        };
    }

    /// The visitors that visit every sub shape that overlaps (Jolt's `struct Visitor : public CollidePointVisitor {
    /// using Result = UVec4; ... }`)
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

            pub fn testBlock(self: *const Self, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) UVec4 {
                return self.base.testBounds(bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
            }

            pub fn shouldVisitBlock(self: *const Self, result: UVec4) bool {
                _ = self;
                return result.testAnyTrue();
            }

            pub fn shouldVisitSubShape(self: *const Self, result: UVec4, index_in_block: u32) bool {
                _ = self;
                return result.getComponent(index_in_block) != 0;
            }
        };
    }

    /// Visitor for GetIntersectingSubShapes
    pub fn GetIntersectingSubShapesVisitorMC(comptime BoxType: type) type {
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
    pub fn castRay(self: *const MutableCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const Visitor = DistanceVisitor(CompoundShape.CastRayVisitor, hitFraction);
        var visitor: Visitor = .{ .base = .init(&ray, &self.base, sub_shape_id_creator, hit) };
        self.walkSubShapes(&visitor);
        return visitor.base.return_value;
    }

    // See Shape::CastRay (the collector version)
    pub fn castRayCollector(self: *const MutableCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = DistanceVisitor(CompoundShape.CastRayVisitorCollector, earlyOutFraction);
        var visitor: Visitor = .{ .base = .init(&ray, ray_cast_settings, &self.base, sub_shape_id_creator, collector, shape_filter) };
        self.walkSubShapes(&visitor);
    }

    // See: Shape::CollidePoint
    pub fn collidePoint(self: *const MutableCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = OverlapVisitor(CompoundShape.CollidePointVisitor);
        var visitor: Visitor = .{ .base = .init(point, &self.base, sub_shape_id_creator, collector, shape_filter) };
        self.walkSubShapes(&visitor);
    }

    // See Shape::CollectTransformedShapes
    pub fn collectTransformedShapes(self: *const MutableCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const Visitor = OverlapVisitor(CompoundShape.CollectTransformedShapesVisitor);
        var visitor: Visitor = .{ .base = .init(box, &self.base, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter) };
        self.walkSubShapes(&visitor);
    }

    // See: CompoundShape::GetIntersectingSubShapes
    pub fn getIntersectingSubShapes(self: *const MutableCompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        var visitor: GetIntersectingSubShapesVisitorMC(AABox) = .{ .base = .init(box, out_sub_shape_indices) };
        self.walkSubShapes(&visitor);
        return visitor.base.getNumResults();
    }

    // See: CompoundShape::GetIntersectingSubShapes
    pub fn getIntersectingSubShapesOrientedBox(self: *const MutableCompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        var visitor: GetIntersectingSubShapesVisitorMC(OrientedBox) = .{ .base = .init(box, out_sub_shape_indices) };
        self.walkSubShapes(&visitor);
        return visitor.base.getNumResults();
    }

    // See Shape: C++ `CompoundShape::SaveBinaryState(inStream)` is the explicit base call
    pub fn saveBinaryState(self: *const MutableCompoundShape, stream: StreamOut) void {
        CompoundShape.impl.saveBinaryState(&self.base, stream);

        // Write bounds
        stream.writeBytes(std.mem.sliceAsBytes(self.sub_shape_bounds.items[0..self.getNumBlocks()]));
    }

    // See Shape::GetStats
    pub fn getStats(self: *const MutableCompoundShape) Shape.Stats {
        return .init(@sizeOf(MutableCompoundShape) + self.base.sub_shapes.items.len * @sizeOf(SubShape) + self.sub_shape_bounds.items.len * @sizeOf(Bounds), 0);
    }

    // See: Shape::RestoreBinaryState: C++ `CompoundShape::RestoreBinaryState(inStream)` is the explicit base call
    pub fn restoreBinaryState(self: *MutableCompoundShape, stream: StreamIn) Allocator.Error!void {
        try CompoundShape.impl.restoreBinaryState(&self.base, stream);

        // Ensure that we have allocated the required space for mSubShapeBounds
        try self.ensureSubShapeBoundsCapacity();

        // Read bounds
        stream.readBytes(std.mem.sliceAsBytes(self.sub_shape_bounds.items[0..self.getNumBlocks()]));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Registration (sRegister, runs at compile time)

    /// Register shape functions with the registry
    pub fn register(comptime r: *Registry) void {
        const f = r.shapeFunctions(.mutable_compound);
        f.construct = ShapeFunctions.constructor(MutableCompoundShape);
        f.color = Color.dark_orange;

        for (ShapeFile.all_sub_shape_types) |s| {
            r.registerCollideShape(.mutable_compound, s, collideCompoundVsShape);
            r.registerCollideShape(s, .mutable_compound, collideShapeVsCompound);
            r.registerCastShape(s, .mutable_compound, castShapeVsCompound);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Helper functions called by CollisionDispatch

    fn castShapeVsCompound(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape_in: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const shape = shape_in.cast(MutableCompoundShape); // Asserts that the sub type is EShapeSubType::MutableCompound

        const Visitor = DistanceVisitor(CompoundShape.CastShapeVisitor, positiveEarlyOutFraction);
        var visitor: Visitor = .{ .base = .init(shape_cast, shape_cast_settings, &shape.base, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector) };
        shape.walkSubShapes(&visitor);
    }

    fn collideCompoundVsShape(shape1_in: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape1 = shape1_in.cast(MutableCompoundShape); // Asserts that the sub type is EShapeSubType::MutableCompound

        const Visitor = OverlapVisitor(CompoundShape.CollideCompoundVsShapeVisitor);
        var visitor: Visitor = .{ .base = .init(&shape1.base, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter) };
        shape1.walkSubShapes(&visitor);
    }

    fn collideShapeVsCompound(shape1: *const Shape, shape2_in: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const shape2 = shape2_in.cast(MutableCompoundShape); // Asserts that the sub type is EShapeSubType::MutableCompound

        const Visitor = OverlapVisitor(CompoundShape.CollideShapeVsCompoundVisitor);
        var visitor: Visitor = .{ .base = .init(shape1, &shape2.base, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter) };
        shape2.walkSubShapes(&visitor);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/CompoundsParity.zig, the comparison with the
// queries of StaticCompoundShape in StaticCompoundShape.zig)

const testing = std.testing;
const math = @import("../../../Math/Math.zig");
const RefConst = @import("../../../Core/Reference.zig").RefConst;
const RefCount = @import("../../../Core/Reference.zig").RefCount;
const StreamWrapper = @import("../../../Core/StreamWrapper.zig");
const RegisterTypes = @import("../../../RegisterTypes.zig");
const CollisionCollectorImpl = @import("../CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const AnyHitCollisionCollector = CollisionCollectorImpl.AnyHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const SphereShapeFile = @import("SphereShape.zig");
const SphereShape = SphereShapeFile.SphereShape;
const SphereShapeSettings = SphereShapeFile.SphereShapeSettings;
const BoxShape = @import("BoxShape.zig").BoxShape;
const StaticCompoundShape = @import("StaticCompoundShape.zig").StaticCompoundShape;

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

/// Check the bounds blocks against the sub shapes: lane i of block b holds the bounds of sub shape 4 b + i, the lanes
/// after the last sub shape repeat its bounds; the local bounds are the union (zero for no sub shapes)
fn checkBounds(shape: *const MutableCompoundShape) !void {
    const sub_shapes = shape.base.sub_shapes.items;
    try testing.expectEqual((sub_shapes.len + 3) / 4, shape.getNumBlocks());
    try testing.expect(shape.sub_shape_bounds.items.len >= shape.getNumBlocks());
    var local_bounds: AABox = .empty;
    for (0..4 * shape.getNumBlocks()) |i| {
        const sub_shape = &sub_shapes[@min(i, sub_shapes.len - 1)];
        const expected = sub_shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM()), Vec3.one());
        const block = &shape.sub_shape_bounds.items[i / 4];
        const lane: u32 = @intCast(i % 4);
        const actual = AABox.init(
            Vec3.init(block.min_x.getComponent(lane), block.min_y.getComponent(lane), block.min_z.getComponent(lane)),
            Vec3.init(block.max_x.getComponent(lane), block.max_y.getComponent(lane), block.max_z.getComponent(lane)),
        );
        try testing.expect(actual.min.isClose(expected.min, .{ .max_dist_sq = 1.0e-10 }) and actual.max.isClose(expected.max, .{ .max_dist_sq = 1.0e-10 })); // AdjustCenterOfMass shifts the stored bounds instead of recalculating them
        local_bounds.encapsulate(actual);
    }
    if (sub_shapes.len == 0)
        local_bounds = .init(Vec3.zero(), Vec3.zero());
    try testing.expect(shape.asShape().getLocalBounds().eql(local_bounds));
}

test "MutableCompoundShapeSettings: errors, an empty compound, cached results" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    // No sub shapes: allowed, empty bounds
    {
        var settings = MutableCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        settings.asShapeSettings().user_data = 3;
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        const shape = result.getPtr().?.cast(MutableCompoundShape);
        try testing.expectEqual(@as(u32, 0), shape.base.getNumSubShapes());
        try testing.expectEqual(@as(u32, 0), shape.getNumBlocks());
        try expect(shape.asShape().getLocalBounds().eql(.init(Vec3.zero(), Vec3.zero())));
        try expect(shape.asShape().getCenterOfMass().eql(Vec3.zero()));
        try testing.expectEqual(math.flt_max, shape.asShape().getInnerRadius());
        try testing.expectEqual(@as(u64, 3), shape.asShape().getUserData());

        // Cached
        var again = try settings.asShapeSettings().createShape(allocator);
        defer again.deinit();
        try expect(again.getPtr() == result.getPtr());

        // Queries find nothing
        var hit: RayCastResult = .{};
        try expect(!shape.asShape().castRay(.init(Vec3.replicate(-1), Vec3.replicate(2)), .{}, &hit));
    }

    // A child error is forwarded
    {
        var sphere = SphereShape.init(allocator, 1.0, .{});
        sphere.asShape().setEmbedded();
        defer sphere.asShapeMut().deinit();
        const bad = try SphereShapeSettings.create(allocator, -1.0, .{});
        var settings = MutableCompoundShapeSettings.init(allocator);
        defer settings.deinit();
        try settings.base.addShapePtr(Vec3.zero(), Quat.identity(), sphere.asShape(), .{});
        try settings.base.addShape(Vec3.zero(), Quat.identity(), bad.asShapeSettings(), .{});
        var result = try settings.asShapeSettings().createShape(allocator);
        defer result.deinit();
        try testing.expectEqualStrings("Invalid radius", result.getError());
    }

    // Too deep: 33 nested compounds of 2 sub shapes
    {
        var sphere = SphereShape.init(allocator, 1.0, .{});
        sphere.asShape().setEmbedded();
        defer sphere.asShapeMut().deinit();
        var inner = RefConst(Shape).init(sphere.asShape());
        defer inner.deinit();
        for (1..34) |level| {
            var settings = MutableCompoundShapeSettings.init(allocator);
            defer settings.deinit();
            try settings.base.addShapePtr(Vec3.init(-1, 0, 0), Quat.identity(), inner.get(), .{});
            try settings.base.addShapePtr(Vec3.init(1, 0, 0), Quat.identity(), sphere.asShape(), .{});
            var result = try settings.asShapeSettings().createShape(allocator);
            defer result.deinit();
            if (level <= 32)
                inner.set(result.getPtr().?)
            else
                try testing.expectEqualStrings("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits", result.getError());
        }
    }
}

test "MutableCompoundShape: the bounds blocks after adding, removing, modifying and adjusting the center of mass" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    var settings = MutableCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    for (0..6) |i| try settings.base.addShapePtr(Vec3.init(@floatFromInt(3 * i), 1, 0), Quat.rotation(Vec3.axisY(), 0.1 * @as(f32, @floatFromInt(i))), if (i % 2 == 0) sphere.asShape() else box.asShape(), .{ .user_data = @intCast(i) });
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.castMut(MutableCompoundShape);
    try checkBounds(shape);
    const com = shape.asShape().getCenterOfMass();

    // Add at the end, in the middle and at the start (an index beyond the end appends)
    try testing.expectEqual(@as(u32, 6), try shape.addShape(Vec3.init(-5, 0, 0), Quat.identity(), box.asShape(), .{ .user_data = 6 }));
    try testing.expectEqual(@as(u32, 7), try shape.addShape(Vec3.init(-6, 0, 0), Quat.identity(), sphere.asShape(), .{ .user_data = 7, .index = 100 }));
    try testing.expectEqual(@as(u32, 3), try shape.addShape(Vec3.init(0, -4, 0), Quat.rotation(Vec3.axisX(), 1.0), box.asShape(), .{ .user_data = 8, .index = 3 }));
    try testing.expectEqual(@as(u32, 0), try shape.addShape(Vec3.init(0, 4, 0), Quat.identity(), sphere.asShape(), .{ .user_data = 9, .index = 0 }));
    try testing.expectEqual(@as(u32, 10), shape.base.getNumSubShapes());
    try checkBounds(shape);
    try expect(shape.asShape().getCenterOfMass().eql(com)); // Adding does not move the center of mass
    try testing.expectEqual(@as(u32, 9), shape.base.getCompoundUserData(0));
    try testing.expectEqual(@as(u32, 8), shape.base.getCompoundUserData(4));
    try expect(shape.base.getSubShape(0).getPositionCOM().eql(Vec3.init(0, 4, 0).sub(com)));

    // Remove
    shape.removeShape(4);
    try checkBounds(shape);
    shape.removeShape(shape.base.getNumSubShapes() - 1);
    try checkBounds(shape);
    shape.removeShape(0);
    try checkBounds(shape);
    try testing.expectEqual(@as(u32, 7), shape.base.getNumSubShapes());
    try testing.expectEqual(@as(usize, 3), shape.sub_shape_bounds.items.len); // The blocks are not freed

    // Modify one, modify one with another shape, modify several with strides
    shape.modifyShape(2, Vec3.init(1, 1, 1), Quat.rotation(Vec3.axisZ(), 0.5));
    try checkBounds(shape);
    try expect(shape.base.getSubShape(2).getPositionCOM().isClose(Vec3.init(1, 1, 1).sub(com).add(Quat.rotation(Vec3.axisZ(), 0.5).mulVec3(box.asShape().getCenterOfMass())), .{}));
    shape.modifyShapeWithShape(3, Vec3.init(2, 2, 2), Quat.identity(), box.asShape());
    try expect(shape.base.getSubShape(3).shape.get() == box.asShape());
    try checkBounds(shape);
    const Pose = struct { position: Vec3, pad: u32 = 0, rotation: Quat };
    const poses = [_]Pose{
        .{ .position = Vec3.init(10, 0, 0), .rotation = Quat.rotation(Vec3.axisX(), 0.3) },
        .{ .position = Vec3.init(11, 1, 0), .rotation = Quat.identity() },
        .{ .position = Vec3.init(12, 2, 0), .rotation = Quat.rotation(Vec3.axisY(), 0.3) },
    };
    shape.modifyShapes(3, 3, .init(&poses[0].position, .{ .stride = @sizeOf(Pose) }), .init(&poses[0].rotation, .{ .stride = @sizeOf(Pose) }));
    try checkBounds(shape);
    for (poses, 3..) |p, i| try expect(shape.base.getSubShape(@intCast(i)).getRotation().eql(if (p.rotation.isClose(Quat.identity(), .{})) Quat.identity() else p.rotation));
    const positions = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(-2, 0, 0) };
    const rotations = [_]Quat{ Quat.identity(), Quat.identity() };
    shape.modifyShapes(0, 2, .init(&positions[0], .{}), .init(&rotations[0], .{}));
    try checkBounds(shape);

    // Adjust the center of mass: the sub shapes and the bounds move, the world positions stay
    const world_before = shape.base.getSubShape(5).getPositionCOM().add(shape.asShape().getCenterOfMass());
    shape.adjustCenterOfMass();
    try checkBounds(shape);
    const world_after = shape.base.getSubShape(5).getPositionCOM().add(shape.asShape().getCenterOfMass());
    try expect(world_after.isClose(world_before, .{ .max_dist_sq = 1.0e-10 }));

    // Remove everything: empty bounds
    while (shape.base.getNumSubShapes() > 0) shape.removeShape(0);
    try expect(shape.asShape().getLocalBounds().eql(.init(Vec3.zero(), Vec3.zero())));

    // Stats count the bounds blocks
    try testing.expectEqual(@sizeOf(MutableCompoundShape) + 3 * @sizeOf(MutableCompoundShape.Bounds), shape.asShape().getStats().size_bytes);
}

test "MutableCompoundShape: queries, shape filters, early outs and the registration" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    // 6 spheres along X: 2 blocks, the last one has 2 lanes that repeat sub shape 5
    var settings = MutableCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    for (0..6) |i| try settings.base.addShapePtr(Vec3.init(3.0 * @as(f32, @floatFromInt(i)), 0, 0), Quat.identity(), sphere.asShape(), .{});
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.cast(MutableCompoundShape);
    const com = shape.asShape().getCenterOfMass();
    try expect(com.isClose(Vec3.init(7.5, 0, 0), .{ .max_dist_sq = 1.0e-10 }));

    // Closest hit along the row: the first sphere
    var hit: RayCastResult = .{};
    const ray = RayCast.init(Vec3.init(-5, 0, 0).sub(com), Vec3.init(30, 0, 0));
    try expect(shape.asShape().castRay(ray, .{}, &hit));
    try testing.expectEqual(@as(u32, 0), shape.base.getSubShapeIndexFromID(hit.sub_shape_id2).index);
    try testing.expectApproxEqAbs(@as(f32, 4.0 / 30.0), hit.fraction, 1.0e-6);

    // All hits: each sphere, in sub shape order
    var all = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer all.deinit();
    shape.asShape().castRayCollector(ray, &.{}, .{}, &all.base, &.{});
    try all.checkError();
    try testing.expectEqual(@as(usize, 6), all.hits.items.len);
    for (all.hits.items, 0..) |h, i| try testing.expectEqual(@as(u32, @intCast(i)), shape.base.getSubShapeIndexFromID(h.sub_shape_id2).index);

    // The closest hit collector: the early out fraction skips the blocks / sub shapes that are further away
    var closest = ClosestHitCollisionCollector(CastRayCollector).init();
    defer closest.deinit();
    shape.asShape().castRayCollector(ray, &.{}, .{}, &closest.base, &.{});
    try expect(closest.hadHit() and closest.hit.fraction == hit.fraction);

    // Collide point: only sphere 2 (at 6) contains 6.5
    var points = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer points.deinit();
    shape.asShape().collidePoint(Vec3.init(6.5, 0, 0).sub(com), .{}, &points.base, &.{});
    try points.checkError();
    try testing.expectEqual(@as(usize, 1), points.hits.items.len);

    // An any hit collector stops after the first hit
    var any = AnyHitCollisionCollector(CastRayCollector).init();
    defer any.deinit();
    shape.asShape().castRayCollector(ray, &.{}, .{}, &any.base, &.{});
    try expect(any.hadHit());

    // Intersecting sub shapes: AABox and OrientedBox
    var indices: [6]u32 = undefined;
    try testing.expectEqual(@as(u32, 2), shape.base.getIntersectingSubShapes(.fromCenterAndRadius(Vec3.init(4.5, 0, 0).sub(com), 0.6), &indices));
    try testing.expectEqualSlices(u32, &.{ 1, 2 }, indices[0..2]);
    try testing.expectEqual(@as(u32, 6), shape.base.getIntersectingSubShapesOrientedBox(.init(Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.1), Vec3.zero()), Vec3.init(10, 1, 1)), &indices));
    // Note: a buffer that is too small for the results of more than one block is overrun in Jolt (WalkSubShapes only
    // stops the walk through the current block when ShouldAbort returns true), Zolt asserts like Jolt: not tested

    // Shape filters that reject the compound
    const RejectCompound = struct {
        pub const overrides = .{.shouldCollide};
        base: ShapeFilter = .init(@This()),
        pub fn shouldCollide(self: *const @This(), shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, sub_shape_id_of_shape2 };
            return shape2.getSubType() != .mutable_compound;
        }
    };
    const reject: RejectCompound = .{};
    all.base.reset();
    shape.asShape().castRayCollector(ray, &.{}, .{}, &all.base, &reject.base);
    points.base.reset();
    shape.asShape().collidePoint(Vec3.zero(), .{}, &points.base, &reject.base);
    var collected = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collected.deinit();
    shape.asShape().collectTransformedShapes(AABox.biggest(), Vec3.zero(), Quat.identity(), Vec3.one(), .{}, &collected.base, &reject.base);
    try testing.expectEqual(@as(usize, 0), all.hits.items.len + points.hits.items.len + collected.hits.items.len);
    shape.asShape().collectTransformedShapes(.fromCenterAndRadius(Vec3.zero(), 1), Vec3.zero(), Quat.identity(), Vec3.one(), .{}, &collected.base, &.{});
    try collected.checkError();
    try testing.expectEqual(@as(usize, 2), collected.hits.items.len);

    // The registered functions
    const r = &RegisterTypes.registry;
    try expect(r.getCollideShape(.mutable_compound, .sphere) == &MutableCompoundShape.collideCompoundVsShape);
    try expect(r.getCollideShape(.box, .mutable_compound) == &MutableCompoundShape.collideShapeVsCompound);
    try expect(r.getCollideShape(.mutable_compound, .mutable_compound) == &MutableCompoundShape.collideShapeVsCompound);
    try expect(r.getCastShape(.capsule, .mutable_compound) == &MutableCompoundShape.castShapeVsCompound);
    try expect(r.getCastShape(.mutable_compound, .sphere) == &CompoundShape.castCompoundVsShape);
    try expect(ShapeFunctions.get(.mutable_compound).color.eql(Color.dark_orange));
    try expect(ShapeFunctions.get(.mutable_compound).construct != null);

    // Collide and cast through CollisionDispatch: a sphere at the second sphere
    var collide_settings: CollideShapeSettings = .{};
    var collide = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collide.deinit();
    const probe = Mat44.translation(Vec3.init(3.5, 0, 0).sub(com));
    CollisionDispatch.collideShapeVsShape(sphere.asShape(), shape.asShape(), Vec3.one(), Vec3.one(), probe, Mat44.identity(), .{}, .{}, &collide_settings, &collide.base, &.{});
    CollisionDispatch.collideShapeVsShape(shape.asShape(), sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), probe, .{}, .{}, &collide_settings, &collide.base, &.{});
    try collide.checkError();
    try testing.expectEqual(@as(usize, 2), collide.hits.items.len);
    try testing.expectEqual(@as(u32, 1), shape.base.getSubShapeIndexFromID(collide.hits.items[0].sub_shape_id2).index);
    try testing.expectEqual(@as(u32, 1), shape.base.getSubShapeIndexFromID(collide.hits.items[1].sub_shape_id1).index);
    const shape_cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-10, 0, 0).sub(com)), Vec3.init(40, 0, 0));
    var cast = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast.deinit();
    CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &.{}, shape.asShape(), Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast.base);
    try cast.checkError();
    try testing.expectEqual(@as(usize, 6), cast.hits.items.len);
}

test "MutableCompoundShape: clone, binary state and out of memory" {
    const allocator = testing.allocator;
    const expect = testing.expect;

    var box = BoxShape.init(allocator, Vec3.init(1, 2, 3), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var settings = MutableCompoundShapeSettings.init(allocator);
    defer settings.deinit();
    settings.asShapeSettings().user_data = 12;
    for (0..5) |i| try settings.base.addShapePtr(Vec3.init(@floatFromInt(i), 0, 0), Quat.rotation(Vec3.axisZ(), @floatFromInt(i)), box.asShape(), .{ .user_data = @intCast(i) });
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    const shape = result.getPtr().?.castMut(MutableCompoundShape);
    shape.removeShape(4); // Keep an unused bounds block

    // Clone: an independent copy
    var clone_ref = try shape.clone(allocator);
    defer clone_ref.deinit();
    const copy = clone_ref.get().?.castMut(MutableCompoundShape);
    try testing.expectEqual(@as(u64, 12), copy.asShape().getUserData());
    var buffer: [2048]u8 = undefined;
    var buffer2: [2048]u8 = undefined;
    const bytes = saveToBuffer(shape.asShape(), &buffer);
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(copy.asShape(), &buffer2));
    try testing.expectEqual(shape.sub_shape_bounds.items.len, copy.sub_shape_bounds.items.len);
    try testing.expectEqual(shape.asShape().getInnerRadius(), copy.asShape().getInnerRadius());
    copy.modifyShape(0, Vec3.init(5, 5, 5), Quat.identity());
    try expect(!shape.base.getSubShape(0).getPositionCOM().eql(copy.base.getSubShape(0).getPositionCOM()));

    // Binary state: the compound part followed by the used bounds blocks
    try testing.expectEqual(@as(usize, 1 + 8 + 12 + 24 + 4 + 4 + 4 * (4 + 12 + 12) + 96), bytes.len);
    var restored_result = try restoreFromBuffer(allocator, bytes);
    defer restored_result.deinit();
    const restored = restored_result.getPtr().?.castMut(MutableCompoundShape);
    var sub_shapes: ShapeFile.ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try shape.asShape().saveSubShapeState(allocator, &sub_shapes);
    restored.asShapeMut().restoreSubShapeState(sub_shapes.items);
    try testing.expectEqualSlices(u8, bytes, saveToBuffer(restored.asShape(), &buffer2));
    try checkBounds(restored);
    var truncated = try restoreFromBuffer(allocator, bytes[0 .. bytes.len - 1]);
    defer truncated.deinit();
    try testing.expectEqualStrings("Failed to restore shape", truncated.getError());

    // Out of memory: settings, creation (child settings), restore, clone, construct
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try testing.expectError(error.OutOfMemory, MutableCompoundShapeSettings.create(failing.allocator()));
    try testing.expectError(error.OutOfMemory, ShapeFunctions.get(.mutable_compound).construct.?(failing.allocator()));
    {
        const child = try SphereShapeSettings.create(allocator, 1.0, .{});
        try settings.base.addShape(Vec3.init(1, 2, 3), Quat.identity(), child.asShapeSettings(), .{});
        settings.asShapeSettings().clearCachedResult();
        var fail_index: usize = 0;
        while (true) : (fail_index += 1) {
            var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            var r = settings.asShapeSettings().createShape(f.allocator()) catch {
                try expect(settings.asShapeSettings().cached_result.isEmpty());
                child.asShapeSettings().clearCachedResult();
                continue;
            };
            defer r.deinit();
            try expect(r.isValid() and !f.has_induced_failure);
            try testing.expectEqual(@as(usize, 4), fail_index); // The compound, its sub shapes, the child, the bounds
            child.asShapeSettings().clearCachedResult();
            settings.asShapeSettings().clearCachedResult();
            break;
        }
    }
    for (0..4) |fail_index| {
        var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        try testing.expectError(error.OutOfMemory, restoreFromBuffer(f.allocator(), bytes)); // Shape, sub shapes, bounds
        if (fail_index < 3) {
            var g = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
            try testing.expectError(error.OutOfMemory, shape.clone(g.allocator())); // Shape, sub shapes, bounds
        }
        if (fail_index == 2) break;
    }

    // addShape: out of memory leaves the compound unchanged
    {
        var f = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
        var small = MutableCompoundShape.initDefault(f.allocator());
        defer small.asShapeMut().deinit();
        const ref_count = box.asShape().getRefCount();
        try testing.expectError(error.OutOfMemory, small.addShape(Vec3.zero(), Quat.identity(), box.asShape(), .{}));
        try testing.expectEqual(@as(u32, 0), small.base.getNumSubShapes());
        try testing.expectEqual(ref_count, box.asShape().getRefCount()); // No reference was taken
        var g = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 1 });
        var small2 = MutableCompoundShape.initDefault(g.allocator());
        defer small2.asShapeMut().deinit();
        try testing.expectError(error.OutOfMemory, small2.addShape(Vec3.zero(), Quat.identity(), box.asShape(), .{}));
        try testing.expectEqual(@as(u32, 0), small2.base.getNumSubShapes());
        try testing.expectEqual(@as(usize, 0), small2.sub_shape_bounds.items.len);
        try testing.expectEqual(ref_count, box.asShape().getRefCount());
    }
}
